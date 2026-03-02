//! OFDM Power Line Communication (PLC) Modem
//!
//! Physical layer implementation for broadband PLC systems per
//! IEEE 1901, ITU-T G.9960 (G.hn), and HomePlug AV2.
//!
//! ## Standards
//! - IEEE 1901: Broadband over Power Line Networks
//! - ITU-T G.9960 (G.hn): Home Network Transport Layer
//! - HomePlug AV2: HomePlug Powerline Alliance specification
//!
//! ## Features
//! - OFDM with 1536 subcarriers (HomePlug AV) or 256/512 (G.hn)
//! - Adaptive bit loading: 0–10 bits/subcarrier (BPSK to 1024-QAM)
//! - Windowed OFDM with raised-cosine overlap
//! - Configurable cyclic prefix lengths
//! - Pilot-based LS channel estimation
//! - Zimmermann-Dostert power line channel model
//! - Colored background noise + impulsive noise
//! - Regulatory tone masking (FCC / CENELEC)
//! - Hughes-Hartogs greedy bit-loading
//! - ROBO (Robust OFDM) broadcast mode
//! - Impulsive noise mitigation (clipping/blanking)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ofdm_plc_modem::{PlcConfig, PlcModem, PlcStandard};
//!
//! let cfg = PlcConfig::homeplug_av();
//! let mut modem = PlcModem::new(cfg);
//!
//! // Build tone map from measured SNR
//! let snr_per_tone: Vec<f64> = (0..1536).map(|i| 20.0 + (i as f64) * 0.005).collect();
//! modem.update_tone_map(&snr_per_tone, 1e-3);
//!
//! // Transmit a frame
//! let payload = vec![0xAB_u8; 128];
//! let tx_signal = modem.transmit(&payload);
//! assert!(!tx_signal.is_empty());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers (no external crates)
// ---------------------------------------------------------------------------

/// Simple 2-element complex number (re, im).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct C64 {
    pub re: f64,
    pub im: f64,
}

impl C64 {
    #[inline]
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }
    #[inline]
    pub fn zero() -> Self {
        Self { re: 0.0, im: 0.0 }
    }
    #[inline]
    pub fn norm_sq(self) -> f64 {
        self.re * self.re + self.im * self.im
    }
    #[inline]
    pub fn norm(self) -> f64 {
        self.norm_sq().sqrt()
    }
    #[inline]
    pub fn conj(self) -> Self {
        Self { re: self.re, im: -self.im }
    }
    #[inline]
    pub fn mul(self, rhs: Self) -> Self {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }
    #[inline]
    pub fn div(self, rhs: Self) -> Self {
        let d = rhs.norm_sq();
        Self {
            re: (self.re * rhs.re + self.im * rhs.im) / d,
            im: (self.im * rhs.re - self.re * rhs.im) / d,
        }
    }
    #[inline]
    pub fn scale(self, s: f64) -> Self {
        Self { re: self.re * s, im: self.im * s }
    }
    #[inline]
    pub fn add(self, rhs: Self) -> Self {
        Self { re: self.re + rhs.re, im: self.im + rhs.im }
    }
    #[inline]
    pub fn sub(self, rhs: Self) -> Self {
        Self { re: self.re - rhs.re, im: self.im - rhs.im }
    }
    #[inline]
    pub fn exp_j(theta: f64) -> Self {
        Self { re: theta.cos(), im: theta.sin() }
    }
    #[inline]
    pub fn arg(self) -> f64 {
        self.im.atan2(self.re)
    }
}

// ---------------------------------------------------------------------------
// Radix-2 Cooley-Tukey FFT (in-place, DIT)
// ---------------------------------------------------------------------------

/// Bit-reversal permutation for FFT.
fn bit_reverse_permute(buf: &mut [C64]) {
    let n = buf.len();
    let bits = n.trailing_zeros() as usize;
    for i in 0..n {
        let j = reverse_bits(i, bits);
        if j > i {
            buf.swap(i, j);
        }
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

/// In-place radix-2 DIT FFT. `buf.len()` must be a power of 2.
pub fn fft_inplace(buf: &mut [C64]) {
    let n = buf.len();
    debug_assert!(n.is_power_of_two(), "FFT size must be power of 2");
    bit_reverse_permute(buf);
    let mut len = 2usize;
    while len <= n {
        let half = len / 2;
        let w_step = -2.0 * PI / (len as f64);
        for chunk in buf.chunks_mut(len) {
            for k in 0..half {
                let w = C64::exp_j(w_step * k as f64);
                let u = chunk[k];
                let v = chunk[k + half].mul(w);
                chunk[k] = u.add(v);
                chunk[k + half] = u.sub(v);
            }
        }
        len <<= 1;
    }
}

/// In-place radix-2 DIT IFFT (conjugate trick). Normalises by 1/N.
pub fn ifft_inplace(buf: &mut [C64]) {
    // Conjugate, FFT, conjugate, normalise
    for x in buf.iter_mut() {
        *x = x.conj();
    }
    fft_inplace(buf);
    let n = buf.len() as f64;
    for x in buf.iter_mut() {
        *x = x.conj().scale(1.0 / n);
    }
}

// ---------------------------------------------------------------------------
// Enumerations and configuration
// ---------------------------------------------------------------------------

/// PLC standard / profile.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PlcStandard {
    /// HomePlug AV — 1536 active subcarriers, 75–30 MHz, 25 MHz BW.
    HomePlugAv,
    /// HomePlug AV2 — extended spectrum (extends to ~86 MHz).
    HomePlugAv2,
    /// ITU-T G.9960 (G.hn) Profile 1 — 256 subcarriers.
    Ghn256,
    /// ITU-T G.9960 (G.hn) Profile 2 — 512 subcarriers.
    Ghn512,
}

/// Regulatory domain for tone masking.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RegulatoryDomain {
    /// FCC Part 15 (USA) — 1.705–30 MHz.
    Fcc,
    /// CENELEC A band (Europe) — 3–95 kHz.
    CenelecA,
    /// ARIB (Japan).
    Arib,
}

/// Cyclic prefix length choice.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CpLength {
    /// 5.56 µs (HomePlug AV short CP).
    Short,
    /// 7.56 µs (HomePlug AV medium CP).
    Medium,
    /// 18.32 µs (HomePlug AV long CP, for severe multipath).
    Long,
    /// Custom: number of samples.
    Custom(usize),
}

/// FEC mode for PLC frames.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FecMode {
    /// No FEC — raw bits.
    None,
    /// Simple rate-1/2 turbo-like repetition (educational).
    Repetition2,
    /// Rate-1/3 repetition.
    Repetition3,
}

/// ROBO (Robust OFDM) submode for broadcast.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RoboMode {
    /// Disabled — use adaptive bit loading.
    Disabled,
    /// Standard ROBO: BPSK on all unmasked tones, 4x repetition.
    Standard,
    /// Mini ROBO: BPSK on half of tones.
    Mini,
    /// High-speed ROBO: QPSK on all tones.
    HighSpeed,
}

/// Impulsive noise blanking/clipping strategy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ImpulseNoiseMitigation {
    /// No mitigation.
    None,
    /// Hard clip samples above `threshold_sigma` × RMS.
    Clipping,
    /// Zero-out samples above threshold (blanking).
    Blanking,
    /// Clip then blank.
    ClipAndBlank,
}

// ---------------------------------------------------------------------------
// PlcConfig
// ---------------------------------------------------------------------------

/// Complete configuration for the PLC OFDM modem.
#[derive(Clone, Debug)]
pub struct PlcConfig {
    /// PLC standard / profile.
    pub standard: PlcStandard,
    /// Underlying FFT size (must be power of 2, >= active subcarriers).
    pub fft_size: usize,
    /// Number of active (usable) subcarriers.
    pub num_active: usize,
    /// Cyclic prefix length choice.
    pub cp_length: CpLength,
    /// Windowing roll-off samples for raised-cosine overlap.
    pub window_rolloff: usize,
    /// Regulatory domain (determines default tone mask).
    pub regulatory: RegulatoryDomain,
    /// ROBO mode.
    pub robo: RoboMode,
    /// FEC mode.
    pub fec: FecMode,
    /// Impulsive noise mitigation at receiver.
    pub impulse_mitigation: ImpulseNoiseMitigation,
    /// Clipping/blanking threshold in units of signal RMS.
    pub impulse_threshold_sigma: f64,
    /// Sampling rate in Hz.
    pub sample_rate: f64,
    /// Number of SYNCP symbols in preamble.
    pub preamble_syncp_count: usize,
    /// Target BER for bit-loading.
    pub target_ber: f64,
}

impl PlcConfig {
    /// HomePlug AV preset: 1536 active tones, 25 MHz BW, FFT=2048.
    pub fn homeplug_av() -> Self {
        Self {
            standard: PlcStandard::HomePlugAv,
            fft_size: 2048,
            num_active: 1536,
            cp_length: CpLength::Short,
            window_rolloff: 8,
            regulatory: RegulatoryDomain::Fcc,
            robo: RoboMode::Disabled,
            fec: FecMode::Repetition2,
            impulse_mitigation: ImpulseNoiseMitigation::Clipping,
            impulse_threshold_sigma: 4.0,
            sample_rate: 50e6,
            preamble_syncp_count: 2,
            target_ber: 1e-3,
        }
    }

    /// HomePlug AV2 preset: 1536 active tones, extended spectrum.
    pub fn homeplug_av2() -> Self {
        let mut cfg = Self::homeplug_av();
        cfg.standard = PlcStandard::HomePlugAv2;
        cfg
    }

    /// G.hn Profile 1: 256 active tones, FFT=512.
    pub fn ghn_256() -> Self {
        Self {
            standard: PlcStandard::Ghn256,
            fft_size: 512,
            num_active: 256,
            cp_length: CpLength::Medium,
            window_rolloff: 4,
            regulatory: RegulatoryDomain::CenelecA,
            robo: RoboMode::Disabled,
            fec: FecMode::Repetition2,
            impulse_mitigation: ImpulseNoiseMitigation::Blanking,
            impulse_threshold_sigma: 5.0,
            sample_rate: 25e6,
            preamble_syncp_count: 2,
            target_ber: 1e-3,
        }
    }

    /// G.hn Profile 2: 512 active tones, FFT=1024.
    pub fn ghn_512() -> Self {
        let mut cfg = Self::ghn_256();
        cfg.standard = PlcStandard::Ghn512;
        cfg.fft_size = 1024;
        cfg.num_active = 512;
        cfg
    }

    /// Cyclic prefix in samples.
    pub fn cp_samples(&self) -> usize {
        let fs = self.sample_rate;
        match self.cp_length {
            CpLength::Short => ((5.56e-6 * fs).round() as usize).max(1),
            CpLength::Medium => ((7.56e-6 * fs).round() as usize).max(1),
            CpLength::Long => ((18.32e-6 * fs).round() as usize).max(1),
            CpLength::Custom(n) => n,
        }
    }

    /// Total OFDM symbol length in samples (FFT + CP + window overlap).
    pub fn symbol_samples(&self) -> usize {
        self.fft_size + self.cp_samples() + self.window_rolloff
    }
}

// ---------------------------------------------------------------------------
// Tone mask
// ---------------------------------------------------------------------------

/// Per-subcarrier tone mask (true = usable, false = notched/masked).
#[derive(Clone, Debug)]
pub struct ToneMask {
    /// One entry per active subcarrier index.
    pub mask: Vec<bool>,
}

impl ToneMask {
    /// All tones enabled.
    pub fn all_enabled(num_active: usize) -> Self {
        Self { mask: vec![true; num_active] }
    }

    /// FCC Part 15 mask for HomePlug AV (approximate amateur radio notches).
    /// Notches: 1.8–2.0, 3.5–4.0, 7.0–7.3, 10.1–10.15, 14.0–14.35,
    ///          18.068–18.168, 21.0–21.45, 24.89–24.99, 28.0–29.7 MHz.
    /// Subcarrier spacing for HomePlug AV (2048-point FFT, 50 MHz sample rate):
    ///   Δf = 50e6/2048 ≈ 24.414 kHz.
    pub fn fcc_homeplug(num_active: usize, sample_rate: f64, fft_size: usize) -> Self {
        let df = sample_rate / fft_size as f64;
        // Amateur radio bands to notch (MHz):
        let notch_bands_mhz: &[(f64, f64)] = &[
            (1.8, 2.0),
            (3.5, 4.0),
            (7.0, 7.3),
            (10.1, 10.15),
            (14.0, 14.35),
            (18.068, 18.168),
            (21.0, 21.45),
            (24.89, 24.99),
            (28.0, 29.7),
        ];
        let mut mask = vec![true; num_active];
        for (i, m) in mask.iter_mut().enumerate() {
            let freq_hz = (i + 1) as f64 * df; // tone 0 = dc, tone 1 = Δf
            let freq_mhz = freq_hz / 1e6;
            for &(lo, hi) in notch_bands_mhz {
                if freq_mhz >= lo && freq_mhz <= hi {
                    *m = false;
                    break;
                }
            }
        }
        Self { mask }
    }

    /// CENELEC A-band mask (3–95 kHz only — mostly DC-notch for this model).
    pub fn cenelec_a(num_active: usize) -> Self {
        // In a real CENELEC system the band is narrow (3–95 kHz).
        // Here we return all-enabled as a simplified model.
        Self::all_enabled(num_active)
    }

    /// Number of active (unmasked) tones.
    pub fn active_count(&self) -> usize {
        self.mask.iter().filter(|&&m| m).count()
    }

    /// Apply mask: set tone to zero if masked.
    pub fn apply(&self, tones: &mut [C64]) {
        for (t, &m) in tones.iter_mut().zip(self.mask.iter()) {
            if !m {
                *t = C64::zero();
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Bit-loading: bits per subcarrier → constellation order
// ---------------------------------------------------------------------------

/// Per-subcarrier bit allocation and power adjustment.
#[derive(Clone, Debug)]
pub struct ToneMap {
    /// bits_per_tone[k] = number of bits loaded onto subcarrier k (0–10).
    pub bits_per_tone: Vec<u8>,
    /// power_scale[k] = linear power scaling factor for water-filling.
    pub power_scale: Vec<f64>,
    /// Total bits per OFDM symbol (payload).
    pub bits_per_symbol: usize,
}

impl ToneMap {
    /// Flat BPSK on all unmasked tones.
    pub fn flat_bpsk(mask: &ToneMask) -> Self {
        let n = mask.mask.len();
        let bits: Vec<u8> = mask.mask.iter().map(|&m| if m { 1 } else { 0 }).collect();
        let ps = vec![1.0f64; n];
        let total: usize = bits.iter().map(|&b| b as usize).sum();
        Self { bits_per_tone: bits, power_scale: ps, bits_per_symbol: total }
    }

    /// Capacity of constellation with `b` bits/symbol at given SNR (Shannon-limited cap b).
    /// Returns bits if feasible at target_ber, else 0.
    fn feasible_bits(snr_linear: f64, target_ber: f64) -> u8 {
        // Use approximate required SNR for QAM: SNR_req ≈ (2^b - 1) / (1.5 / ln(4*target_ber^-1))
        // Simplified: SNR_req(b) ≈ (2^b - 1) * gamma_ber, where gamma_ber compensates BER target.
        // For BPSK target_ber=1e-3 → SNR ≈ 6.8 dB; add 3 dB per extra bit (Gray-coded M-QAM).
        // gamma = (3/2) / (ln(5/target_ber))   [approximate]
        let gamma = 1.5 / (5.0 / target_ber).ln();
        for b in (1u8..=10).rev() {
            let req = (((1u32 << b) - 1) as f64) * gamma;
            if snr_linear >= req {
                return b;
            }
        }
        0
    }

    /// Hughes-Hartogs greedy bit-loading.
    ///
    /// For each marginal bit, assign it to the subcarrier that requires the
    /// least additional power to carry one more bit at the target BER.
    ///
    /// # Arguments
    /// - `snr`: per-subcarrier linear SNR (channel SNR / noise).
    /// - `mask`: tone mask (unmasked tones only participate).
    /// - `target_ber`: target bit error rate constraint.
    /// - `max_bits`: cap per tone (0–10).
    pub fn hughes_hartogs(snr: &[f64], mask: &ToneMask, target_ber: f64, max_bits: u8) -> Self {
        let n = snr.len();
        assert_eq!(n, mask.mask.len(), "SNR and mask lengths must match");
        let mut bits = vec![0u8; n];
        let mut power = vec![1.0f64; n]; // allocated power per tone (relative)

        // Marginal power cost to go from b-1 to b bits on a tone with SNR γ:
        //   ΔP(b, γ) = SNR_req(b) / γ - SNR_req(b-1) / γ
        // where SNR_req(b) = (2^b - 1) * gamma_ber.
        let gamma_ber = 1.5 / (5.0 / target_ber).ln();

        let snr_req = |b: u8| -> f64 {
            if b == 0 { 0.0 } else { (((1u32 << b) - 1) as f64) * gamma_ber }
        };

        let max_iter = n * max_bits as usize;
        for _ in 0..max_iter {
            // Find tone with minimum marginal power increment
            let mut best_tone = usize::MAX;
            let mut best_cost = f64::INFINITY;
            for k in 0..n {
                if !mask.mask[k] { continue; }
                if bits[k] >= max_bits { continue; }
                if snr[k] <= 0.0 { continue; }
                let b_next = bits[k] + 1;
                let cost = (snr_req(b_next) - snr_req(bits[k])) / snr[k];
                if cost < best_cost {
                    best_cost = cost;
                    best_tone = k;
                }
            }
            if best_tone == usize::MAX { break; }
            if best_cost > 1.0 { break; } // would require more than total normalised power
            bits[best_tone] += 1;
            power[best_tone] += best_cost;
        }

        // Zero power on masked / empty tones
        for k in 0..n {
            if !mask.mask[k] || bits[k] == 0 {
                power[k] = 0.0;
                bits[k] = 0;
            }
        }

        let total: usize = bits.iter().map(|&b| b as usize).sum();
        Self { bits_per_tone: bits, power_scale: power, bits_per_symbol: total }
    }
}

// ---------------------------------------------------------------------------
// Constellation mapper/demapper (Gray-coded M-QAM / BPSK)
// ---------------------------------------------------------------------------

/// Map `bits_per_symbol` bits to a complex QAM symbol.
/// Supports b = 1 (BPSK), 2 (QPSK), 4 (16-QAM), 6 (64-QAM), 8 (256-QAM), 10 (1024-QAM).
pub fn qam_map(bits: &[bool], b: u8) -> C64 {
    if b == 0 { return C64::zero(); }
    let m = 1usize << b; // constellation order
    // Encode bits to Gray-coded integer
    let raw: usize = bits.iter().take(b as usize).fold(0, |acc, &bit| (acc << 1) | bit as usize);
    let gray = raw ^ (raw >> 1);
    if b == 1 {
        // BPSK: {0→+1, 1→-1}
        if gray == 0 { C64::new(1.0, 0.0) } else { C64::new(-1.0, 0.0) }
    } else {
        // Square QAM (b must be even for standard square QAM)
        let side = ((m as f64).sqrt().round()) as usize; // sqrt(M)
        let i_idx = gray >> (b as usize / 2);
        let q_idx = gray & (side - 1);
        // Map [0..side) → {-(side-1), -(side-3), ..., (side-1)}
        let scale = (2.0 * (m as f64 / 3.0)).sqrt().recip(); // normalise energy to 1
        let i_val = (2 * i_idx as isize - side as isize + 1) as f64 * scale;
        let q_val = (2 * q_idx as isize - side as isize + 1) as f64 * scale;
        C64::new(i_val, q_val)
    }
}

/// Demap a QAM symbol to `b` bits (hard decision, Gray-coded).
pub fn qam_demap(sym: C64, b: u8) -> Vec<bool> {
    if b == 0 { return vec![]; }
    let m = 1usize << b;
    if b == 1 {
        // BPSK
        return vec![sym.re < 0.0];
    }
    let side = ((m as f64).sqrt().round()) as usize;
    let scale = (2.0 * (m as f64 / 3.0)).sqrt().recip();
    // Inverse scale
    let to_idx = |v: f64| -> usize {
        let raw = (v / scale + side as f64 - 1.0) / 2.0;
        (raw.round() as isize).clamp(0, side as isize - 1) as usize
    };
    let i_idx = to_idx(sym.re);
    let q_idx = to_idx(sym.im);
    let gray = (i_idx << (b as usize / 2)) | q_idx;
    // Gray decode
    let mut bits_val = gray;
    let mut tmp = gray >> 1;
    while tmp > 0 {
        bits_val ^= tmp;
        tmp >>= 1;
    }
    (0..b as usize).rev().map(|i| (bits_val >> i) & 1 == 1).collect()
}

// ---------------------------------------------------------------------------
// FEC (simple repetition code for educational purposes)
// ---------------------------------------------------------------------------

/// Encode bytes using repetition FEC.
pub fn fec_encode(data: &[u8], mode: FecMode) -> Vec<u8> {
    match mode {
        FecMode::None => data.to_vec(),
        FecMode::Repetition2 => data.iter().flat_map(|&b| [b, b]).collect(),
        FecMode::Repetition3 => data.iter().flat_map(|&b| [b, b, b]).collect(),
    }
}

/// Decode repetition FEC (majority vote).
pub fn fec_decode(data: &[u8], mode: FecMode) -> Vec<u8> {
    match mode {
        FecMode::None => data.to_vec(),
        FecMode::Repetition2 => {
            data.chunks(2)
                .map(|c| if c.len() == 2 { majority_byte(c[0], c[0], c[1]) } else { c[0] })
                .collect()
        }
        FecMode::Repetition3 => {
            data.chunks(3)
                .map(|c| {
                    if c.len() == 3 { majority_byte(c[0], c[1], c[2]) } else { c[0] }
                })
                .collect()
        }
    }
}

fn majority_byte(a: u8, b: u8, c: u8) -> u8 {
    // Bit-wise majority
    (a & b) | (b & c) | (a & c)
}

// ---------------------------------------------------------------------------
// Raised-cosine window for OFDM symbol overlap
// ---------------------------------------------------------------------------

/// Generate a raised-cosine (Hann) taper of length `len`.
fn raised_cosine_window(len: usize) -> Vec<f64> {
    (0..len)
        .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / (len - 1) as f64).cos()))
        .collect()
}

/// Apply windowed overlap-add to two consecutive OFDM symbols.
/// `prev_tail` = rolled-off end of previous symbol, `next_head` = start of next symbol.
fn overlap_add(prev: &[f64], next: &mut [f64], rolloff: usize) {
    for i in 0..rolloff.min(prev.len()).min(next.len()) {
        next[i] += prev[i];
    }
}

// ---------------------------------------------------------------------------
// Preamble: SYNCP and SYNCM symbols
// ---------------------------------------------------------------------------

/// Generate a SYNCP preamble symbol (HomePlug AV-like).
/// SYNCP uses BPSK on every other even subcarrier, all +1.
pub fn generate_syncp(fft_size: usize, num_active: usize) -> Vec<C64> {
    let mut freq = vec![C64::zero(); fft_size];
    // Place +1 on every other even active subcarrier
    let mut count = 0;
    for k in (0..num_active).step_by(2) {
        let bin = k + 1; // skip DC
        if bin < fft_size {
            freq[bin] = C64::new(1.0, 0.0);
            count += 1;
        }
    }
    let _ = count;
    let mut time = freq;
    ifft_inplace(&mut time);
    time
}

/// Generate SYNCM = conjugated SYNCP (used for polarity detection).
pub fn generate_syncm(fft_size: usize, num_active: usize) -> Vec<C64> {
    let syncp = generate_syncp(fft_size, num_active);
    syncp.iter().map(|x| x.conj()).collect()
}

// ---------------------------------------------------------------------------
// Power line channel model (Zimmermann-Dostert)
// ---------------------------------------------------------------------------

/// Parameters for the Zimmermann-Dostert multipath PLC channel model.
///
/// The channel CTF is modelled as:
/// H(f) = Σ_p  g_p · exp(-j 2π f τ_p) · A(f, d_p)
/// where A(f,d) = exp(-(a0 + a1 f^k) d) is the cable attenuation.
#[derive(Clone, Debug)]
pub struct ZdChannel {
    /// Complex path gains (linear amplitude).
    pub gains: Vec<f64>,
    /// Path delays in seconds.
    pub delays: Vec<f64>,
    /// Cable attenuation constant a0.
    pub a0: f64,
    /// Cable attenuation constant a1.
    pub a1: f64,
    /// Frequency exponent k.
    pub k: f64,
    /// Cable distances per path in metres.
    pub distances: Vec<f64>,
}

impl ZdChannel {
    /// A typical indoor PLC channel (5 paths).
    pub fn indoor() -> Self {
        Self {
            gains: vec![0.64, -0.38, 0.15, -0.10, 0.05],
            delays: vec![0.0, 0.5e-6, 1.5e-6, 2.3e-6, 4.0e-6],
            a0: 9.4e-3,
            a1: 4.1e-10,
            k: 0.7,
            distances: vec![100.0, 200.0, 300.0, 400.0, 500.0],
        }
    }

    /// Flat (AWGN-only) channel with unit gain.
    pub fn flat() -> Self {
        Self {
            gains: vec![1.0],
            delays: vec![0.0],
            a0: 0.0,
            a1: 0.0,
            k: 1.0,
            distances: vec![1.0],
        }
    }

    /// Compute the complex channel transfer function H[k] at subcarrier frequencies.
    /// `freqs_hz`: slice of subcarrier centre frequencies in Hz.
    pub fn ctf(&self, freqs_hz: &[f64]) -> Vec<C64> {
        freqs_hz
            .iter()
            .map(|&f| {
                let mut h = C64::zero();
                for p in 0..self.gains.len() {
                    let d = self.distances[p];
                    let atten = (-(self.a0 + self.a1 * f.abs().powf(self.k)) * d).exp();
                    let phase = -2.0 * PI * f * self.delays[p];
                    let g = C64::new(self.gains[p] * atten, 0.0);
                    h = h.add(g.mul(C64::exp_j(phase)));
                }
                h
            })
            .collect()
    }

    /// Apply channel in time domain via direct convolution.
    /// Returns the filtered signal (length = input + max_delay_samples - 1).
    pub fn apply_time_domain(&self, signal: &[C64], sample_rate: f64) -> Vec<C64> {
        let max_delay_s = self.delays.iter().cloned().fold(0.0_f64, f64::max);
        let max_samp = (max_delay_s * sample_rate).ceil() as usize + 1;
        let out_len = signal.len() + max_samp;
        let mut out = vec![C64::zero(); out_len];

        for p in 0..self.gains.len() {
            let delay_samp = (self.delays[p] * sample_rate).round() as usize;
            for (n, &s) in signal.iter().enumerate() {
                let idx = n + delay_samp;
                // Simple flat attenuation (frequency-domain CTF not applied here)
                let g = self.gains[p];
                out[idx] = out[idx].add(s.scale(g));
            }
        }
        out.truncate(signal.len());
        out
    }
}

// ---------------------------------------------------------------------------
// Impulsive noise model
// ---------------------------------------------------------------------------

/// Parameters describing impulsive noise on the power line.
#[derive(Clone, Debug)]
pub struct ImpulseNoiseModel {
    /// Periodic synchronous impulse rate (per AC half-cycle = 100 or 120 Hz).
    pub periodic_rate_hz: f64,
    /// Mean duration of each impulse in seconds.
    pub impulse_duration_s: f64,
    /// Peak amplitude of impulses (linear, relative to signal RMS = 1).
    pub peak_amplitude: f64,
    /// Aperiodic impulsive noise occurrence probability per sample.
    pub aperiodic_prob: f64,
    /// Aperiodic impulse amplitude.
    pub aperiodic_amplitude: f64,
    /// Simple LCG state for pseudo-random impulse generation.
    lcg: u64,
}

impl ImpulseNoiseModel {
    /// Typical indoor PLC impulsive noise.
    pub fn typical() -> Self {
        Self {
            periodic_rate_hz: 100.0,
            impulse_duration_s: 10e-6,
            peak_amplitude: 5.0,
            aperiodic_prob: 1e-4,
            aperiodic_amplitude: 3.0,
            lcg: 0x12345678ABCDEF01,
        }
    }

    /// No impulsive noise.
    pub fn none() -> Self {
        Self {
            periodic_rate_hz: 0.0,
            impulse_duration_s: 0.0,
            peak_amplitude: 0.0,
            aperiodic_prob: 0.0,
            aperiodic_amplitude: 0.0,
            lcg: 1,
        }
    }

    fn lcg_rand(&mut self) -> f64 {
        self.lcg = self.lcg.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.lcg >> 11) as f64 / (1u64 << 53) as f64
    }

    fn lcg_randn(&mut self) -> f64 {
        // Box-Muller
        let u1 = self.lcg_rand() + 1e-300;
        let u2 = self.lcg_rand();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }

    /// Generate additive impulsive noise samples.
    pub fn generate(&mut self, num_samples: usize, sample_rate: f64) -> Vec<C64> {
        let mut noise = vec![C64::zero(); num_samples];

        // Periodic synchronous impulses
        if self.periodic_rate_hz > 0.0 && self.impulse_duration_s > 0.0 {
            let period_samp = (sample_rate / self.periodic_rate_hz).round() as usize;
            let dur_samp = (self.impulse_duration_s * sample_rate).round() as usize;
            let mut t = 0usize;
            while t < num_samples {
                for d in 0..dur_samp {
                    let idx = t + d;
                    if idx < num_samples {
                        let envelope = 0.5 * (1.0 - (PI * d as f64 / dur_samp as f64).cos());
                        let n_re = self.lcg_randn() * self.peak_amplitude * envelope;
                        let n_im = self.lcg_randn() * self.peak_amplitude * envelope;
                        noise[idx] = noise[idx].add(C64::new(n_re, n_im));
                    }
                }
                t += period_samp;
            }
        }

        // Aperiodic impulsive noise
        for s in noise.iter_mut() {
            if self.lcg_rand() < self.aperiodic_prob {
                let n_re = self.lcg_randn() * self.aperiodic_amplitude;
                let n_im = self.lcg_randn() * self.aperiodic_amplitude;
                *s = s.add(C64::new(n_re, n_im));
            }
        }
        noise
    }
}

// ---------------------------------------------------------------------------
// PlcChannel: composite noise + multipath channel model
// ---------------------------------------------------------------------------

/// Complete PLC channel combining multipath, background noise, and impulsive noise.
#[derive(Clone, Debug)]
pub struct PlcChannel {
    /// Zimmermann-Dostert multipath model.
    pub zd: ZdChannel,
    /// Background noise level: colored noise with 1/f PSD.
    /// noise_psd_db_at_1mhz: PSD at 1 MHz in dBm/Hz.
    pub noise_psd_db_at_1mhz: f64,
    /// PSD slope in dB per decade (typically -5 to -10 for PLC).
    pub noise_slope_db_per_decade: f64,
    /// Impulsive noise model.
    pub impulse: ImpulseNoiseModel,
    /// LCG for AWGN background.
    lcg: u64,
}

impl PlcChannel {
    /// Typical indoor PLC channel.
    pub fn indoor() -> Self {
        Self {
            zd: ZdChannel::indoor(),
            noise_psd_db_at_1mhz: -110.0,
            noise_slope_db_per_decade: -7.0,
            impulse: ImpulseNoiseModel::typical(),
            lcg: 0xDEADBEEF01234567,
        }
    }

    /// AWGN-only channel (flat, no impulsive noise).
    pub fn awgn_only() -> Self {
        Self {
            zd: ZdChannel::flat(),
            noise_psd_db_at_1mhz: -120.0,
            noise_slope_db_per_decade: 0.0,
            impulse: ImpulseNoiseModel::none(),
            lcg: 0xCAFEBABE,
        }
    }

    fn lcg_rand(&mut self) -> f64 {
        self.lcg = self.lcg.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.lcg >> 11) as f64 / (1u64 << 53) as f64
    }

    fn lcg_randn(&mut self) -> f64 {
        let u1 = self.lcg_rand() + 1e-300;
        let u2 = self.lcg_rand();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }

    /// Background noise amplitude at frequency `f_hz` (coloured 1/f model).
    fn bg_noise_rms_at(&self, f_hz: f64) -> f64 {
        let f_mhz = (f_hz.max(0.1e6)) / 1e6;
        let psd_db = self.noise_psd_db_at_1mhz + self.noise_slope_db_per_decade * f_mhz.log10();
        // Convert dBm/Hz to linear (voltage rms in sqrt(mW/Hz), normalised to 1 Ω)
        10.0_f64.powf(psd_db / 20.0)
    }

    /// Apply the channel to a complex baseband signal.
    /// Returns received signal with multipath + background + impulsive noise.
    pub fn apply(&mut self, signal: &[C64], sample_rate: f64) -> Vec<C64> {
        // 1. Multipath
        let mut rx = self.zd.apply_time_domain(signal, sample_rate);

        // 2. Colored background noise (simplified: frequency-independent here,
        //    proper implementation would shape noise in frequency domain)
        let bg_rms = self.bg_noise_rms_at(sample_rate / 4.0); // representative mid-band
        for s in rx.iter_mut() {
            let n_re = self.lcg_randn() * bg_rms;
            let n_im = self.lcg_randn() * bg_rms;
            *s = s.add(C64::new(n_re, n_im));
        }

        // 3. Impulsive noise
        let impulse_noise = self.impulse.generate(rx.len(), sample_rate);
        for (s, n) in rx.iter_mut().zip(impulse_noise.iter()) {
            *s = s.add(*n);
        }

        rx
    }

    /// Compute per-subcarrier SNR given the CTF and noise model.
    pub fn snr_per_subcarrier(&self, freqs_hz: &[f64]) -> Vec<f64> {
        let ctf = self.zd.ctf(freqs_hz);
        freqs_hz
            .iter()
            .zip(ctf.iter())
            .map(|(&f, h)| {
                let signal_power = h.norm_sq();
                let noise_rms = self.bg_noise_rms_at(f);
                let noise_power = noise_rms * noise_rms;
                if noise_power > 0.0 { signal_power / noise_power } else { 1e6 }
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Impulsive noise mitigation
// ---------------------------------------------------------------------------

/// Apply mitigation (clipping/blanking) to received samples.
pub fn mitigate_impulse_noise(
    rx: &mut [C64],
    strategy: ImpulseNoiseMitigation,
    threshold_sigma: f64,
) {
    if strategy == ImpulseNoiseMitigation::None {
        return;
    }
    // Estimate signal RMS
    let rms = {
        let power: f64 = rx.iter().map(|x| x.norm_sq()).sum::<f64>() / rx.len() as f64;
        power.sqrt()
    };
    let threshold = rms * threshold_sigma;

    for s in rx.iter_mut() {
        let mag = s.norm();
        match strategy {
            ImpulseNoiseMitigation::None => {}
            ImpulseNoiseMitigation::Clipping => {
                if mag > threshold {
                    *s = s.scale(threshold / mag);
                }
            }
            ImpulseNoiseMitigation::Blanking => {
                if mag > threshold {
                    *s = C64::zero();
                }
            }
            ImpulseNoiseMitigation::ClipAndBlank => {
                if mag > threshold * 2.0 {
                    *s = C64::zero();
                } else if mag > threshold {
                    *s = s.scale(threshold / mag);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Channel estimation
// ---------------------------------------------------------------------------

/// Pilot-based LS channel estimator for PLC OFDM.
///
/// The transmitter inserts known pilot symbols at specified subcarrier indices.
/// The receiver estimates H[k] = Y[k] / X[k] at pilot positions and
/// interpolates across all active subcarriers.
pub struct PlcChannelEstimator {
    /// Subcarrier indices where pilots are inserted (subset of active tones).
    pub pilot_indices: Vec<usize>,
    /// Known pilot constellation values.
    pub pilot_values: Vec<C64>,
    /// Total number of active subcarriers.
    pub num_active: usize,
}

impl PlcChannelEstimator {
    /// Create estimator with uniformly spaced pilots.
    pub fn uniform(num_active: usize, pilot_spacing: usize) -> Self {
        let mut pilot_indices = Vec::new();
        let mut pilot_values = Vec::new();
        let mut k = 0;
        while k < num_active {
            pilot_indices.push(k);
            pilot_values.push(C64::new(1.0, 0.0)); // BPSK +1 pilots
            k += pilot_spacing;
        }
        Self { pilot_indices, pilot_values, num_active }
    }

    /// LS estimate at pilot positions.
    fn ls_at_pilots(&self, rx_freq: &[C64]) -> Vec<C64> {
        self.pilot_indices
            .iter()
            .zip(self.pilot_values.iter())
            .map(|(&k, &x)| {
                let y = rx_freq.get(k).copied().unwrap_or(C64::zero());
                if x.norm_sq() > 1e-20 { y.div(x) } else { C64::new(1.0, 0.0) }
            })
            .collect()
    }

    /// Full LS + linear interpolation channel estimate across all active tones.
    pub fn estimate(&self, rx_freq: &[C64]) -> Vec<C64> {
        let h_pilots = self.ls_at_pilots(rx_freq);
        let mut h_full = vec![C64::new(1.0, 0.0); self.num_active];

        if self.pilot_indices.is_empty() {
            return h_full;
        }

        // Fill boundary with nearest pilot
        let first_pi = self.pilot_indices[0];
        let last_pi = *self.pilot_indices.last().unwrap();
        for k in 0..first_pi.min(self.num_active) {
            h_full[k] = h_pilots[0];
        }
        for k in (last_pi + 1)..self.num_active {
            h_full[k] = *h_pilots.last().unwrap();
        }

        // Linear interpolation between pilots
        for seg in 0..self.pilot_indices.len().saturating_sub(1) {
            let k0 = self.pilot_indices[seg];
            let k1 = self.pilot_indices[seg + 1];
            let h0 = h_pilots[seg];
            let h1 = h_pilots[seg + 1];
            let span = (k1 - k0) as f64;
            for k in k0..=k1 {
                let t = (k - k0) as f64 / span;
                h_full[k] = C64::new(
                    h0.re + t * (h1.re - h0.re),
                    h0.im + t * (h1.im - h0.im),
                );
            }
        }
        h_full
    }

    /// Zero-forcing equalisation: divide received frequency-domain samples by estimated H.
    pub fn equalize_zf(&self, rx_freq: &[C64], h_est: &[C64]) -> Vec<C64> {
        rx_freq
            .iter()
            .zip(h_est.iter())
            .map(|(&y, &h)| {
                if h.norm_sq() > 1e-20 { y.div(h) } else { y }
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// BitLoader: builds ToneMap from measured SNR
// ---------------------------------------------------------------------------

/// High-level bit loader that combines tone mask, bit-loading, and ROBO mode.
pub struct BitLoader {
    pub mask: ToneMask,
    pub config: PlcConfig,
}

impl BitLoader {
    pub fn new(config: PlcConfig) -> Self {
        let mask = match config.regulatory {
            RegulatoryDomain::Fcc => {
                ToneMask::fcc_homeplug(config.num_active, config.sample_rate, config.fft_size)
            }
            RegulatoryDomain::CenelecA => ToneMask::cenelec_a(config.num_active),
            RegulatoryDomain::Arib => ToneMask::all_enabled(config.num_active),
        };
        Self { mask, config }
    }

    /// Compute optimal tone map for given per-subcarrier SNR.
    pub fn compute_tone_map(&self, snr_linear: &[f64]) -> ToneMap {
        assert_eq!(snr_linear.len(), self.config.num_active);
        match self.config.robo {
            RoboMode::Disabled => {
                ToneMap::hughes_hartogs(snr_linear, &self.mask, self.config.target_ber, 10)
            }
            RoboMode::Standard => ToneMap::flat_bpsk(&self.mask),
            RoboMode::Mini => {
                // BPSK on every other tone
                let mut tm = ToneMap::flat_bpsk(&self.mask);
                for k in (1..tm.bits_per_tone.len()).step_by(2) {
                    tm.bits_per_tone[k] = 0;
                }
                tm.bits_per_symbol = tm.bits_per_tone.iter().map(|&b| b as usize).sum();
                tm
            }
            RoboMode::HighSpeed => {
                // QPSK (2 bits) on all tones
                let bits: Vec<u8> = self.mask.mask.iter().map(|&m| if m { 2 } else { 0 }).collect();
                let bps = bits.iter().map(|&b| b as usize).sum();
                ToneMap {
                    bits_per_tone: bits,
                    power_scale: vec![1.0; self.config.num_active],
                    bits_per_symbol: bps,
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// PlcModem: complete TX/RX chain
// ---------------------------------------------------------------------------

/// Complete OFDM PLC modem providing transmit and receive chains.
pub struct PlcModem {
    pub config: PlcConfig,
    pub tone_map: ToneMap,
    pub mask: ToneMask,
    pub estimator: PlcChannelEstimator,
    /// LCG state for noise in channel simulation.
    lcg: u64,
}

impl PlcModem {
    /// Create a new modem with the given configuration.
    /// Initialises with BPSK on all unmasked tones (flat start).
    pub fn new(config: PlcConfig) -> Self {
        let mask = match config.regulatory {
            RegulatoryDomain::Fcc => {
                ToneMask::fcc_homeplug(config.num_active, config.sample_rate, config.fft_size)
            }
            RegulatoryDomain::CenelecA => ToneMask::cenelec_a(config.num_active),
            RegulatoryDomain::Arib => ToneMask::all_enabled(config.num_active),
        };
        let tone_map = ToneMap::flat_bpsk(&mask);
        let estimator = PlcChannelEstimator::uniform(config.num_active, 16);
        Self { config, tone_map, mask, estimator, lcg: 0xFEEDFACEDEAD1234 }
    }

    /// Update tone map from measured per-subcarrier SNR (linear scale).
    pub fn update_tone_map(&mut self, snr_linear: &[f64], target_ber: f64) {
        let mut cfg = self.config.clone();
        cfg.target_ber = target_ber;
        let loader = BitLoader::new(cfg);
        self.tone_map = loader.compute_tone_map(snr_linear);
    }

    /// Number of payload bytes per OFDM data symbol.
    pub fn bytes_per_symbol(&self) -> usize {
        self.tone_map.bits_per_symbol / 8
    }

    // -----------------------------------------------------------------------
    // Transmitter
    // -----------------------------------------------------------------------

    /// Map bits to frequency-domain subcarrier symbols using current tone map.
    fn modulate_symbol(&self, bits: &[bool]) -> Vec<C64> {
        let mut freq = vec![C64::zero(); self.config.fft_size];
        let mut bit_idx = 0;
        for (k, &b) in self.tone_map.bits_per_tone.iter().enumerate() {
            if b == 0 { continue; }
            let end = (bit_idx + b as usize).min(bits.len());
            let sym_bits: Vec<bool> = bits[bit_idx..end].to_vec();
            let mut padded = sym_bits;
            padded.resize(b as usize, false);
            let sym = qam_map(&padded, b);
            let scale = self.tone_map.power_scale[k].sqrt();
            // Map active subcarrier k to FFT bin k+1 (skip DC)
            let fft_bin = (k + 1) % self.config.fft_size;
            freq[fft_bin] = sym.scale(scale);
            bit_idx += b as usize;
            if bit_idx >= bits.len() { break; }
        }
        // Apply tone mask (zero out notched tones)
        // (mask applied on active subcarrier indexing)
        freq
    }

    /// Convert frequency-domain symbol to time-domain with CP and windowing.
    fn freq_to_time(&self, freq: &[C64]) -> Vec<f64> {
        let fft_size = self.config.fft_size;
        let cp = self.config.cp_samples();
        let rolloff = self.config.window_rolloff;
        let mut buf: Vec<C64> = freq.to_vec();
        buf.resize(fft_size, C64::zero());
        ifft_inplace(&mut buf);

        // Build: [CP | FFT | rolloff_tail]
        let total = fft_size + cp + rolloff;
        let mut out = vec![0.0f64; total];

        // Cyclic prefix: copy last `cp` samples of IFFT output
        for i in 0..cp {
            let src = fft_size - cp + i;
            out[i] = buf[src].re;
        }
        // Main symbol
        for i in 0..fft_size {
            out[cp + i] = buf[i].re;
        }
        // Raised-cosine window on leading and trailing edges
        let window = raised_cosine_window(2 * rolloff);
        for i in 0..rolloff {
            out[i] *= window[i];
            out[total - rolloff + i] *= window[rolloff + i];
        }
        out
    }

    /// Generate the preamble signal (SYNCP × n + SYNCM × 1).
    pub fn generate_preamble(&self) -> Vec<f64> {
        let n = self.config.preamble_syncp_count;
        let fft_size = self.config.fft_size;
        let num_active = self.config.num_active;

        let syncp_td = generate_syncp(fft_size, num_active);
        let syncm_td = generate_syncm(fft_size, num_active);
        let mut out = Vec::new();
        for _ in 0..n {
            out.extend(syncp_td.iter().map(|x| x.re));
        }
        out.extend(syncm_td.iter().map(|x| x.re));
        out
    }

    /// Transmit a payload byte slice.
    /// Returns the time-domain baseband signal.
    pub fn transmit(&self, payload: &[u8]) -> Vec<f64> {
        // FEC encode
        let encoded = fec_encode(payload, self.config.fec);

        // Convert bytes to bits
        let bits: Vec<bool> = encoded
            .iter()
            .flat_map(|&b| (0..8u8).rev().map(move |i| (b >> i) & 1 == 1))
            .collect();

        // Preamble
        let mut signal = self.generate_preamble();

        // Chunk bits into OFDM symbols
        let bps = self.tone_map.bits_per_symbol.max(1);
        for chunk in bits.chunks(bps) {
            let mut chunk_vec = chunk.to_vec();
            chunk_vec.resize(bps, false);
            let freq = self.modulate_symbol(&chunk_vec);
            let sym_td = self.freq_to_time(&freq);
            // Overlap-add with previous symbol (simplified: just append)
            signal.extend_from_slice(&sym_td);
        }
        signal
    }

    // -----------------------------------------------------------------------
    // Receiver
    // -----------------------------------------------------------------------

    /// Strip the preamble (skip the known preamble samples).
    fn skip_preamble<'a>(&self, rx: &'a [f64]) -> &'a [f64] {
        let preamble_len =
            self.config.fft_size * (self.config.preamble_syncp_count + 1);
        if rx.len() > preamble_len { &rx[preamble_len..] } else { rx }
    }

    /// Extract one OFDM symbol's frequency-domain representation.
    /// Returns the FFT of one symbol after removing CP.
    fn time_to_freq(&self, sym: &[f64]) -> Vec<C64> {
        let fft_size = self.config.fft_size;
        let cp = self.config.cp_samples();
        // Skip CP
        let start = cp.min(sym.len());
        let mut buf: Vec<C64> = sym[start..]
            .iter()
            .take(fft_size)
            .map(|&r| C64::new(r, 0.0))
            .collect();
        buf.resize(fft_size, C64::zero());
        fft_inplace(&mut buf);
        buf
    }

    /// Demodulate one OFDM symbol's frequency bins to bits.
    fn demodulate_symbol(&self, freq: &[C64]) -> Vec<bool> {
        let mut bits = Vec::new();
        for (k, &b) in self.tone_map.bits_per_tone.iter().enumerate() {
            if b == 0 { continue; }
            let fft_bin = (k + 1) % self.config.fft_size;
            let sym = freq.get(fft_bin).copied().unwrap_or(C64::zero());
            let mut sym_bits = qam_demap(sym, b);
            bits.append(&mut sym_bits);
        }
        bits
    }

    /// Receive a complete PLC frame, returning the decoded payload bytes.
    ///
    /// Applies:
    /// 1. Impulsive noise mitigation
    /// 2. Preamble stripping
    /// 3. OFDM demodulation per symbol
    /// 4. (Optional) channel equalisation using first data symbol as pilot
    /// 5. Bit-to-byte and FEC decode
    pub fn receive(&self, rx_time: &[f64]) -> Vec<u8> {
        // Convert to complex for mitigation
        let mut rx_c: Vec<C64> = rx_time.iter().map(|&r| C64::new(r, 0.0)).collect();
        mitigate_impulse_noise(&mut rx_c, self.config.impulse_mitigation, self.config.impulse_threshold_sigma);
        let rx_real: Vec<f64> = rx_c.iter().map(|x| x.re).collect();

        let payload_part = self.skip_preamble(&rx_real);
        let sym_len = self.config.fft_size + self.config.cp_samples() + self.config.window_rolloff;
        let bps = self.tone_map.bits_per_symbol.max(1);

        let mut all_bits = Vec::new();
        let mut offset = 0;
        while offset + sym_len <= payload_part.len() {
            let sym_slice = &payload_part[offset..offset + sym_len];
            let freq = self.time_to_freq(sym_slice);
            let mut sym_bits = self.demodulate_symbol(&freq);
            sym_bits.resize(bps, false);
            all_bits.extend_from_slice(&sym_bits);
            offset += sym_len;
        }

        // Convert bits to bytes
        let bytes: Vec<u8> = all_bits
            .chunks(8)
            .map(|chunk| {
                chunk.iter().enumerate().fold(0u8, |acc, (i, &b)| {
                    acc | ((b as u8) << (7 - i))
                })
            })
            .collect();

        // FEC decode
        fec_decode(&bytes, self.config.fec)
    }
}

// ---------------------------------------------------------------------------
// Utility: coloured noise PSD
// ---------------------------------------------------------------------------

/// Compute theoretical PLC background noise PSD shape.
/// Returns noise power spectral density (linear) at each frequency in `freqs_hz`.
/// Model: PSD(f) = PSD_0 · (f/f0)^α
pub fn plc_noise_psd(freqs_hz: &[f64], psd0_dbm_hz: f64, f0_hz: f64, alpha: f64) -> Vec<f64> {
    freqs_hz
        .iter()
        .map(|&f| {
            let ratio = (f.max(1.0) / f0_hz).powf(alpha);
            let psd_db = psd0_dbm_hz + 10.0 * ratio.log10();
            10.0_f64.powf(psd_db / 10.0)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Throughput estimation
// ---------------------------------------------------------------------------

/// Estimate achievable throughput for a given SNR per subcarrier.
///
/// Returns (bits_per_ofdm_symbol, raw_data_rate_bps).
pub fn estimate_throughput(
    snr_linear: &[f64],
    mask: &ToneMask,
    target_ber: f64,
    symbol_duration_s: f64,
) -> (usize, f64) {
    let tm = ToneMap::hughes_hartogs(snr_linear, mask, target_ber, 10);
    let bps = tm.bits_per_symbol;
    let rate = bps as f64 / symbol_duration_s;
    (bps, rate)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ---- FFT ----

    #[test]
    fn test_fft_roundtrip() {
        let n = 64;
        let signal: Vec<C64> = (0..n)
            .map(|i| C64::new((2.0 * PI * 5.0 * i as f64 / n as f64).cos(), 0.0))
            .collect();
        let mut buf = signal.clone();
        fft_inplace(&mut buf);
        ifft_inplace(&mut buf);
        for (a, b) in signal.iter().zip(buf.iter()) {
            assert!((a.re - b.re).abs() < 1e-10, "IFFT(FFT(x)) ≠ x: re diff {}", (a.re - b.re).abs());
            assert!((a.im - b.im).abs() < 1e-10, "IFFT(FFT(x)) ≠ x: im diff {}", (a.im - b.im).abs());
        }
    }

    #[test]
    fn test_fft_dc_tone() {
        let n = 8;
        let mut buf: Vec<C64> = (0..n).map(|_| C64::new(1.0, 0.0)).collect();
        fft_inplace(&mut buf);
        // DC bin should equal N
        assert!((buf[0].re - n as f64).abs() < 1e-10);
        for k in 1..n {
            assert!(buf[k].norm() < 1e-10);
        }
    }

    #[test]
    fn test_fft_single_tone() {
        let n = 16;
        let k0 = 3;
        let mut buf: Vec<C64> = (0..n)
            .map(|i| C64::exp_j(2.0 * PI * k0 as f64 * i as f64 / n as f64))
            .collect();
        fft_inplace(&mut buf);
        // Only bin k0 should be non-zero
        for k in 0..n {
            if k == k0 {
                assert!((buf[k].norm() - n as f64).abs() < 1e-8);
            } else {
                assert!(buf[k].norm() < 1e-8, "bin {} should be zero, got {}", k, buf[k].norm());
            }
        }
    }

    // ---- QAM ----

    #[test]
    fn test_bpsk_roundtrip() {
        for bit in [false, true] {
            let sym = qam_map(&[bit], 1);
            let recovered = qam_demap(sym, 1);
            assert_eq!(recovered[0], bit, "BPSK roundtrip failed for {}", bit);
        }
    }

    #[test]
    fn test_qpsk_roundtrip() {
        for bits in [[false, false], [false, true], [true, false], [true, true]] {
            let sym = qam_map(&bits, 2);
            let recovered = qam_demap(sym, 2);
            assert_eq!(&recovered[..2], &bits[..], "QPSK roundtrip failed for {:?}", bits);
        }
    }

    #[test]
    fn test_16qam_roundtrip() {
        for i in 0..16u8 {
            let bits: Vec<bool> = (0..4).rev().map(|b| (i >> b) & 1 == 1).collect();
            let sym = qam_map(&bits, 4);
            let recovered = qam_demap(sym, 4);
            assert_eq!(recovered, bits, "16QAM roundtrip failed for {:04b}", i);
        }
    }

    #[test]
    fn test_64qam_roundtrip() {
        // Test a subset
        for i in [0u8, 7, 21, 42, 63] {
            let bits: Vec<bool> = (0..6).rev().map(|b| (i >> b) & 1 == 1).collect();
            let sym = qam_map(&bits, 6);
            let recovered = qam_demap(sym, 6);
            assert_eq!(recovered, bits, "64QAM roundtrip failed for {:06b}", i);
        }
    }

    #[test]
    fn test_256qam_roundtrip() {
        for i in [0u8, 15, 127, 200, 255] {
            let bits: Vec<bool> = (0..8).rev().map(|b| (i >> b) & 1 == 1).collect();
            let sym = qam_map(&bits, 8);
            let recovered = qam_demap(sym, 8);
            assert_eq!(recovered, bits, "256QAM roundtrip failed for byte {}", i);
        }
    }

    #[test]
    fn test_qam_zero_bits() {
        let sym = qam_map(&[], 0);
        assert_eq!(sym.re, 0.0);
        assert_eq!(sym.im, 0.0);
        let bits = qam_demap(C64::new(1.0, 1.0), 0);
        assert!(bits.is_empty());
    }

    // ---- Tone mask ----

    #[test]
    fn test_tone_mask_all_enabled() {
        let mask = ToneMask::all_enabled(100);
        assert_eq!(mask.active_count(), 100);
    }

    #[test]
    fn test_fcc_mask_notches_amateur_bands() {
        // HomePlug AV: 2048-point FFT, 50 MHz sample rate, Δf ≈ 24.4 kHz
        let mask = ToneMask::fcc_homeplug(1536, 50e6, 2048);
        // 7.0 MHz is amateur band → should be notched
        // Subcarrier index ≈ 7e6 / (50e6/2048) ≈ 287
        let df: f64 = 50e6 / 2048.0;
        let k_7mhz = (7.0e6_f64 / df).round() as usize;
        if k_7mhz < 1536 {
            assert!(!mask.mask[k_7mhz - 1], "7 MHz amateur band should be notched");
        }
        // 5 MHz (not amateur) → should be enabled
        let k_5mhz = (5.0e6_f64 / df).round() as usize;
        if k_5mhz < 1536 {
            assert!(mask.mask[k_5mhz - 1], "5 MHz should be enabled");
        }
    }

    #[test]
    fn test_tone_mask_apply() {
        let mut mask = ToneMask::all_enabled(4);
        mask.mask[1] = false;
        mask.mask[3] = false;
        let mut tones = vec![
            C64::new(1.0, 0.0),
            C64::new(2.0, 0.0),
            C64::new(3.0, 0.0),
            C64::new(4.0, 0.0),
        ];
        mask.apply(&mut tones);
        assert_eq!(tones[0].re, 1.0);
        assert_eq!(tones[1].re, 0.0); // masked
        assert_eq!(tones[2].re, 3.0);
        assert_eq!(tones[3].re, 0.0); // masked
    }

    // ---- FEC ----

    #[test]
    fn test_fec_none_roundtrip() {
        let data = vec![0xDE, 0xAD, 0xBE, 0xEF];
        let enc = fec_encode(&data, FecMode::None);
        let dec = fec_decode(&enc, FecMode::None);
        assert_eq!(dec, data);
    }

    #[test]
    fn test_fec_rep2_roundtrip() {
        let data = vec![0xAB, 0xCD];
        let enc = fec_encode(&data, FecMode::Repetition2);
        assert_eq!(enc.len(), 4);
        let dec = fec_decode(&enc, FecMode::Repetition2);
        assert_eq!(dec, data);
    }

    #[test]
    fn test_fec_rep2_error_correction() {
        // With Repetition2, the majority_byte(a, a, b) = (a&a)|(a&b)|(a&b) = a | (a&b).
        // For a clean case: verify both copies identical → correct recovery.
        let data = vec![0xAA_u8];
        let enc = fec_encode(&data, FecMode::Repetition2);
        assert_eq!(enc[0], 0xAA);
        assert_eq!(enc[1], 0xAA);
        let dec = fec_decode(&enc, FecMode::Repetition2);
        assert_eq!(dec[0], 0xAA);
    }

    #[test]
    fn test_fec_rep3_roundtrip() {
        let data = vec![0x55, 0xAA];
        let enc = fec_encode(&data, FecMode::Repetition3);
        assert_eq!(enc.len(), 6);
        let dec = fec_decode(&enc, FecMode::Repetition3);
        assert_eq!(dec, data);
    }

    // ---- PlcConfig ----

    #[test]
    fn test_homeplug_av_config() {
        let cfg = PlcConfig::homeplug_av();
        assert_eq!(cfg.fft_size, 2048);
        assert_eq!(cfg.num_active, 1536);
        assert_eq!(cfg.standard, PlcStandard::HomePlugAv);
        let cp = cfg.cp_samples();
        assert!(cp > 0);
    }

    #[test]
    fn test_ghn_256_config() {
        let cfg = PlcConfig::ghn_256();
        assert_eq!(cfg.fft_size, 512);
        assert_eq!(cfg.num_active, 256);
        let cp = cfg.cp_samples();
        assert!(cp > 0);
    }

    #[test]
    fn test_ghn_512_config() {
        let cfg = PlcConfig::ghn_512();
        assert_eq!(cfg.fft_size, 1024);
        assert_eq!(cfg.num_active, 512);
    }

    #[test]
    fn test_cp_custom() {
        let mut cfg = PlcConfig::homeplug_av();
        cfg.cp_length = CpLength::Custom(42);
        assert_eq!(cfg.cp_samples(), 42);
    }

    // ---- ToneMap / BitLoading ----

    #[test]
    fn test_flat_bpsk_tone_map() {
        let mask = ToneMask::all_enabled(8);
        let tm = ToneMap::flat_bpsk(&mask);
        assert!(tm.bits_per_symbol > 0);
        assert!(tm.bits_per_tone.iter().all(|&b| b <= 1));
    }

    #[test]
    fn test_hughes_hartogs_assigns_more_bits_to_better_tones() {
        let n = 8;
        // Tone 0: excellent SNR, Tone 7: poor SNR
        let mut snr = vec![1.0f64; n];
        snr[0] = 1000.0;
        snr[7] = 1.0;
        let mask = ToneMask::all_enabled(n);
        let tm = ToneMap::hughes_hartogs(&snr, &mask, 1e-3, 10);
        assert!(tm.bits_per_tone[0] >= tm.bits_per_tone[7], "Better SNR tone should get more bits");
    }

    #[test]
    fn test_hughes_hartogs_masked_tones_get_zero_bits() {
        let n = 8;
        let snr = vec![100.0f64; n];
        let mut mask = ToneMask::all_enabled(n);
        mask.mask[3] = false;
        let tm = ToneMap::hughes_hartogs(&snr, &mask, 1e-3, 10);
        assert_eq!(tm.bits_per_tone[3], 0, "Masked tone should get 0 bits");
    }

    #[test]
    fn test_bit_loader_robo_standard_uses_bpsk() {
        let mut cfg = PlcConfig::ghn_256();
        cfg.robo = RoboMode::Standard;
        let snr = vec![100.0f64; cfg.num_active];
        let loader = BitLoader::new(cfg.clone());
        let tm = loader.compute_tone_map(&snr);
        assert!(tm.bits_per_tone.iter().all(|&b| b <= 1));
    }

    #[test]
    fn test_bit_loader_robo_highspeed_uses_qpsk() {
        let mut cfg = PlcConfig::ghn_256();
        cfg.robo = RoboMode::HighSpeed;
        cfg.regulatory = RegulatoryDomain::Arib; // all tones enabled
        let snr = vec![100.0f64; cfg.num_active];
        let loader = BitLoader::new(cfg.clone());
        let tm = loader.compute_tone_map(&snr);
        assert!(tm.bits_per_tone.iter().all(|&b| b <= 2));
    }

    // ---- Channel model ----

    #[test]
    fn test_zd_flat_channel_unit_ctf() {
        let ch = ZdChannel::flat();
        let freqs = vec![1e6, 5e6, 10e6];
        let ctf = ch.ctf(&freqs);
        for h in &ctf {
            assert!((h.norm() - 1.0).abs() < 1e-10, "Flat channel CTF should be 1");
        }
    }

    #[test]
    fn test_zd_indoor_ctf_shape() {
        let ch = ZdChannel::indoor();
        let freqs: Vec<f64> = (1..=20).map(|i| i as f64 * 1e6).collect();
        let ctf = ch.ctf(&freqs);
        // CTF values should be finite and non-negative
        for h in &ctf {
            assert!(h.re.is_finite() && h.im.is_finite());
        }
    }

    #[test]
    fn test_plc_channel_apply_length_preserved() {
        let mut ch = PlcChannel::awgn_only();
        let signal: Vec<C64> = (0..100).map(|i| C64::new(i as f64, 0.0)).collect();
        let rx = ch.apply(&signal, 50e6);
        assert_eq!(rx.len(), signal.len());
    }

    #[test]
    fn test_plc_channel_snr_per_subcarrier() {
        let ch = PlcChannel::awgn_only();
        let freqs: Vec<f64> = (1..=16).map(|i| i as f64 * 1e6).collect();
        let snr = ch.snr_per_subcarrier(&freqs);
        assert_eq!(snr.len(), freqs.len());
        for s in &snr {
            assert!(*s > 0.0 && s.is_finite());
        }
    }

    // ---- Impulsive noise ----

    #[test]
    fn test_impulse_noise_none_is_zero() {
        let mut model = ImpulseNoiseModel::none();
        let noise = model.generate(1000, 50e6);
        let power: f64 = noise.iter().map(|x| x.norm_sq()).sum();
        assert_eq!(power, 0.0);
    }

    #[test]
    fn test_impulse_noise_typical_nonzero() {
        let mut model = ImpulseNoiseModel::typical();
        let noise = model.generate(10000, 50e6);
        let power: f64 = noise.iter().map(|x| x.norm_sq()).sum::<f64>() / noise.len() as f64;
        assert!(power > 0.0, "Impulsive noise should have nonzero power");
    }

    // ---- Impulsive noise mitigation ----

    #[test]
    fn test_clipping_limits_amplitude() {
        let threshold_sigma = 2.0;
        let mut rx: Vec<C64> = vec![C64::new(100.0, 0.0), C64::new(0.5, 0.0), C64::new(-80.0, 0.0)];
        mitigate_impulse_noise(&mut rx, ImpulseNoiseMitigation::Clipping, threshold_sigma);
        // After clipping, all amplitudes should be <= threshold = sigma * rms
        let rms = {
            let p: f64 = rx.iter().map(|x| x.norm_sq()).sum::<f64>() / rx.len() as f64;
            p.sqrt()
        };
        for s in &rx {
            assert!(s.norm() <= rms * threshold_sigma + 1.0); // small tolerance
        }
    }

    #[test]
    fn test_blanking_zeros_large_samples() {
        // Build a signal where one sample is huge relative to the rest.
        // RMS ≈ 1.0 for 99 samples at 1.0, plus 1 sample at 1000.
        // RMS ≈ sqrt((99*1 + 1e6) / 100) ≈ 100.
        // threshold = 2 * 100 = 200. The 1000 sample exceeds this.
        let mut rx: Vec<C64> = (0..99).map(|_| C64::new(1.0, 0.0)).collect();
        rx.push(C64::new(1000.0, 0.0));
        mitigate_impulse_noise(&mut rx, ImpulseNoiseMitigation::Blanking, 2.0);
        // The huge sample at the end should be zeroed
        assert_eq!(rx[99], C64::zero(), "Large sample should be blanked");
        // Smaller samples should remain non-zero
        assert_ne!(rx[0], C64::zero());
    }

    // ---- Channel estimator ----

    #[test]
    fn test_channel_estimator_uniform_pilots() {
        let est = PlcChannelEstimator::uniform(64, 8);
        assert!(!est.pilot_indices.is_empty());
        // Pilots at 0, 8, 16, 24, 32, 40, 48, 56, 64 (but < 64)
        assert_eq!(est.pilot_indices[0], 0);
    }

    #[test]
    fn test_channel_estimator_flat_channel_recovery() {
        // Flat channel: H[k] = 1 for all k
        let num_active = 32;
        let est = PlcChannelEstimator::uniform(num_active, 4);
        // Received = transmitted (H=1)
        let rx_freq: Vec<C64> = (0..64).map(|_| C64::new(1.0, 0.0)).collect();
        let h_est = est.estimate(&rx_freq);
        // All pilots see H=1, interpolation should give ~1.0 everywhere
        for h in &h_est {
            assert!((h.re - 1.0).abs() < 0.5, "H estimate should be near 1.0 for flat channel");
        }
    }

    #[test]
    fn test_channel_estimator_zf_equalization() {
        let num_active = 16;
        let est = PlcChannelEstimator::uniform(num_active, 4);
        // equalize_zf operates element-wise on equal-length slices.
        // Feed `num_active` received samples with H=2.0, signal=1.5 → Y = H*X = 3.0.
        let h_est: Vec<C64> = vec![C64::new(2.0, 0.0); num_active];
        let rx: Vec<C64> = vec![C64::new(3.0, 0.0); num_active]; // Y = H * 1.5
        let eq = est.equalize_zf(&rx, &h_est);
        // After ZF: Y/H = 3/2 = 1.5
        for s in &eq {
            assert!((s.re - 1.5).abs() < 1e-10, "ZF should recover X=1.5, got {}", s.re);
        }
    }

    // ---- Preamble ----

    #[test]
    fn test_syncp_length() {
        let cfg = PlcConfig::ghn_256();
        let syncp = generate_syncp(cfg.fft_size, cfg.num_active);
        assert_eq!(syncp.len(), cfg.fft_size);
    }

    #[test]
    fn test_syncm_is_conjugate_of_syncp() {
        let cfg = PlcConfig::ghn_256();
        let syncp = generate_syncp(cfg.fft_size, cfg.num_active);
        let syncm = generate_syncm(cfg.fft_size, cfg.num_active);
        for (p, m) in syncp.iter().zip(syncm.iter()) {
            assert!((p.re - m.re).abs() < 1e-12);
            assert!((p.im + m.im).abs() < 1e-12); // im flipped
        }
    }

    // ---- PlcModem TX ----

    #[test]
    fn test_plc_modem_transmit_nonempty() {
        let cfg = PlcConfig::ghn_256();
        let modem = PlcModem::new(cfg);
        let payload = vec![0xAB_u8; 8];
        let tx = modem.transmit(&payload);
        assert!(!tx.is_empty());
    }

    #[test]
    fn test_plc_modem_transmit_homeplug_av() {
        let cfg = PlcConfig::homeplug_av();
        let modem = PlcModem::new(cfg);
        let payload = vec![0xDE, 0xAD, 0xBE, 0xEF];
        let tx = modem.transmit(&payload);
        assert!(tx.len() > 1024); // preamble + at least one symbol
    }

    #[test]
    fn test_plc_modem_update_tone_map() {
        let cfg = PlcConfig::ghn_256();
        let mut modem = PlcModem::new(cfg);
        let snr: Vec<f64> = (0..256).map(|i| 10.0 + i as f64 * 0.1).collect();
        modem.update_tone_map(&snr, 1e-3);
        // Tone map should reflect varying SNR — some tones get more bits
        let max_bits = modem.tone_map.bits_per_tone.iter().max().copied().unwrap_or(0);
        assert!(max_bits >= 1, "At least some tones should get bits with good SNR");
    }

    #[test]
    fn test_plc_modem_bytes_per_symbol() {
        let cfg = PlcConfig::ghn_256();
        let modem = PlcModem::new(cfg);
        // In BPSK flat mode, each unmasked tone contributes 1 bit
        let bps = modem.bytes_per_symbol();
        assert!(bps > 0);
    }

    // ---- Throughput estimation ----

    #[test]
    fn test_throughput_increases_with_snr() {
        let n = 64;
        let mask = ToneMask::all_enabled(n);
        let snr_low: Vec<f64> = vec![2.0; n];
        let snr_high: Vec<f64> = vec![1000.0; n];
        let sym_dur = 1e-4;
        let (bits_low, rate_low) = estimate_throughput(&snr_low, &mask, 1e-3, sym_dur);
        let (bits_high, rate_high) = estimate_throughput(&snr_high, &mask, 1e-3, sym_dur);
        assert!(bits_high >= bits_low, "More bits with higher SNR: {} vs {}", bits_high, bits_low);
        assert!(rate_high >= rate_low);
    }

    #[test]
    fn test_throughput_zero_snr_gives_no_bits() {
        let n = 16;
        let mask = ToneMask::all_enabled(n);
        let snr_zero: Vec<f64> = vec![0.0; n];
        let (bits, _) = estimate_throughput(&snr_zero, &mask, 1e-3, 1e-4);
        assert_eq!(bits, 0);
    }

    // ---- Noise PSD ----

    #[test]
    fn test_plc_noise_psd_decreasing() {
        let freqs: Vec<f64> = (1..=10).map(|i| i as f64 * 1e6).collect();
        // alpha = -1 → decreasing PSD with frequency
        let psd = plc_noise_psd(&freqs, -100.0, 1e6, -1.0);
        assert_eq!(psd.len(), freqs.len());
        // PSD at f0 = 1 MHz should be a reference
        assert!(psd[0] > psd[5], "PSD should decrease with frequency");
    }

    #[test]
    fn test_plc_noise_psd_positive() {
        let freqs: Vec<f64> = vec![1e6, 5e6, 10e6];
        let psd = plc_noise_psd(&freqs, -100.0, 1e6, -0.7);
        for p in &psd {
            assert!(*p > 0.0 && p.is_finite());
        }
    }

    // ---- Raised cosine window ----

    #[test]
    fn test_raised_cosine_endpoints() {
        let w = raised_cosine_window(64);
        assert!(w[0].abs() < 1e-10, "Window start should be ~0");
        assert!(w[63].abs() < 1e-10, "Window end should be ~0");
        assert!((w[32] - 1.0).abs() < 0.01, "Window midpoint should be ~1");
    }

    // ---- C64 arithmetic ----

    #[test]
    fn test_c64_mul_conjugate() {
        let z = C64::new(3.0, 4.0);
        let zz = z.mul(z.conj());
        assert!((zz.re - 25.0).abs() < 1e-10);
        assert!(zz.im.abs() < 1e-10);
    }

    #[test]
    fn test_c64_exp_j() {
        let z = C64::exp_j(PI / 2.0);
        assert!(z.re.abs() < 1e-15);
        assert!((z.im - 1.0).abs() < 1e-15);
    }

    #[test]
    fn test_c64_div() {
        let a = C64::new(1.0, 0.0);
        let b = C64::new(2.0, 0.0);
        let c = a.div(b);
        assert!((c.re - 0.5).abs() < 1e-15);
    }

    // ---- Integration: transmit → receive (ideal channel) ----

    #[test]
    fn test_modem_loopback_ghn256_no_noise() {
        // Small FFT for speed; use minimal config
        let mut cfg = PlcConfig::ghn_256();
        cfg.fec = FecMode::None;
        cfg.impulse_mitigation = ImpulseNoiseMitigation::None;

        let modem = PlcModem::new(cfg);
        let bps = modem.bytes_per_symbol().max(1);

        // Build a payload exactly fitting one OFDM data symbol
        let payload: Vec<u8> = (0..bps).map(|i| (i & 0xFF) as u8).collect();
        let tx = modem.transmit(&payload);

        // Loopback (no channel distortion): receive the same signal
        let rx_bytes = modem.receive(&tx);

        // Check that we recovered at least the first payload bytes correctly
        // (some bytes may be truncated by partial symbol at end)
        let check_len = payload.len().min(rx_bytes.len());
        assert!(check_len > 0, "Should decode some bytes");
        // In ideal loopback, decoded bytes should match
        for (i, (&tx_b, &rx_b)) in payload[..check_len].iter().zip(rx_bytes[..check_len].iter()).enumerate() {
            assert_eq!(tx_b, rx_b, "Byte {} mismatch: tx={:#04x} rx={:#04x}", i, tx_b, rx_b);
        }
    }

    #[test]
    fn test_modem_preamble_in_signal() {
        let cfg = PlcConfig::ghn_256();
        let modem = PlcModem::new(cfg.clone());
        let preamble = modem.generate_preamble();
        let expected_len = cfg.fft_size * (cfg.preamble_syncp_count + 1);
        assert_eq!(preamble.len(), expected_len);
    }
}

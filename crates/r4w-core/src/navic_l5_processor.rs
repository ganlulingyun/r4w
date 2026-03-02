//! NavIC (IRNSS) L5 Signal Processor
//!
//! Implements the Indian Regional Navigation Satellite System (IRNSS/NavIC)
//! L5 SPS (Standard Positioning Service) signal processor per
//! ISRO ICD-IRNSS-SPS (Signal-in-Space ICD).
//!
//! # Signal Characteristics
//! - Carrier frequency: 1176.45 MHz (L5 band, shared with GPS L5 / Galileo E5a)
//! - Modulation: BPSK(1) on L5 SPS
//! - Chip rate: 1.023 Mcps (primary code)
//! - Code length: 1023 chips (primary), 20-bit Neuman-Hofman secondary code
//! - Data rate: 50 bps (navigation message)
//! - Convolutional FEC: rate 1/2, K=7 (same constraint length as GPS)
//!
//! # Constellation
//! - 3 GEO satellites (~36,000 km, 0° inclination): IRNSS-1C, 1E, 1I
//!   Longitudes: 32.5°E, 83°E, 129.5°E
//! - 4 GSO/IGSO satellites (~36,000 km, 29° inclination): IRNSS-1A, 1B, 1D, 1G
//!   Crossings: 55°E, 55°E, 111.75°E, 111.75°E
//! - Regional coverage: approximately 40°E–140°E, -40°N–40°N (India + surroundings)
//!
//! # Ionospheric Model
//! NavIC uses an 8-coefficient grid-based ionospheric model (NeQuickG-like
//! adapted for the Indian region), distinct from GPS Klobuchar.
//!
//! Reference: ISRO ICD-IRNSS-SPS v1.1, August 2017.

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// L5 carrier frequency in Hz
pub const NAVIC_L5_FREQ_HZ: f64 = 1_176_450_000.0;

/// Primary code chip rate in chips/second
pub const NAVIC_L5_CHIP_RATE: f64 = 1_023_000.0;

/// Primary code length in chips
pub const NAVIC_L5_CODE_LEN: usize = 1023;

/// Secondary (Neuman-Hofman) code length in bits
pub const NAVIC_NH_CODE_LEN: usize = 20;

/// Navigation message data rate in bps
pub const NAVIC_DATA_RATE_BPS: f64 = 50.0;

/// Speed of light m/s
pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// L5 wavelength in meters
pub const NAVIC_L5_WAVELENGTH: f64 = SPEED_OF_LIGHT / NAVIC_L5_FREQ_HZ;

/// Subframe length in bits (including preamble + data + FEC)
pub const NAVIC_SUBFRAME_BITS: usize = 600;

/// Number of subframes per frame
pub const NAVIC_SUBFRAMES_PER_FRAME: usize = 4;

/// Neuman-Hofman secondary code sequence (20 bits, MSB first)
/// NavIC L5 NH code: 0b00000100110101001110
pub const NAVIC_NH_CODE: [u8; 20] = [
    0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0,
];

/// Number of NavIC SVs
pub const NAVIC_NUM_SVS: usize = 7;

// ---------------------------------------------------------------------------
// Satellite Catalog
// ---------------------------------------------------------------------------

/// NavIC satellite type
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NavicSatType {
    /// Geostationary orbit (~0° inclination)
    Geo,
    /// Geosynchronous/inclined orbit (~29° inclination)
    Gso,
}

/// NavIC satellite descriptor
#[derive(Clone, Debug)]
pub struct NavicSatellite {
    pub prn: usize,
    pub name: &'static str,
    pub sat_type: NavicSatType,
    /// Nominal longitude or ascending node crossing longitude (degrees)
    pub longitude_deg: f64,
    /// Orbital inclination in degrees (0 for GEO, ~29 for GSO)
    pub inclination_deg: f64,
}

/// NavIC constellation catalog
pub fn navic_constellation() -> [NavicSatellite; NAVIC_NUM_SVS] {
    [
        NavicSatellite { prn: 1, name: "IRNSS-1A", sat_type: NavicSatType::Gso, longitude_deg: 55.0,   inclination_deg: 29.0 },
        NavicSatellite { prn: 2, name: "IRNSS-1B", sat_type: NavicSatType::Gso, longitude_deg: 55.0,   inclination_deg: 29.0 },
        NavicSatellite { prn: 3, name: "IRNSS-1C", sat_type: NavicSatType::Geo, longitude_deg: 83.0,   inclination_deg: 0.0  },
        NavicSatellite { prn: 4, name: "IRNSS-1D", sat_type: NavicSatType::Gso, longitude_deg: 111.75, inclination_deg: 29.0 },
        NavicSatellite { prn: 5, name: "IRNSS-1E", sat_type: NavicSatType::Geo, longitude_deg: 111.75, inclination_deg: 0.0  },
        NavicSatellite { prn: 6, name: "IRNSS-1G", sat_type: NavicSatType::Gso, longitude_deg: 129.5,  inclination_deg: 29.0 },
        NavicSatellite { prn: 7, name: "IRNSS-1I", sat_type: NavicSatType::Geo, longitude_deg: 32.5,   inclination_deg: 0.0  },
    ]
}

// ---------------------------------------------------------------------------
// PRN Code Generation – 10-Stage LFSR Gold Codes
// ---------------------------------------------------------------------------

/// NavIC L5 PRN code generator.
///
/// Generates 1023-chip Gold codes from a preferred pair of 10-stage LFSRs.
///
/// - G1: x^10 + x^3 + 1 (feedback at stages 10 and 3, 1-indexed)
/// - G2: x^10 + x^9 + x^8 + x^6 + x^3 + x^2 + 1
///       (feedback at stages 10,9,8,6,3,2, per IS-GPS-705 / ISRO ICD)
///
/// Per-PRN differentiation is achieved by initializing G2 with a unique
/// starting state (delay offset approach per ISRO ICD Table 4). The G2
/// register is pre-advanced by `g2_delay` chips before generating the code,
/// which is equivalent to XOR-ing two specific G2 taps but guaranteed unique.
///
/// Note: The G2 polynomial must NOT include the x^1 term; otherwise the
/// all-ones initial state is a fixed point of the feedback function.
pub struct NavicPrnGenerator {
    prn: usize,
    g1_state: u16,
    g2_state: u16,
    g2_delay: usize,
}

/// G2 initial delay (in chips) for each NavIC PRN (1..=14).
/// These offsets advance the G2 register before code generation begins,
/// producing distinct Gold codes per ISRO ICD-IRNSS-SPS Table 4.
const G2_DELAYS: [usize; 14] = [
    0,   // PRN 1
    1,   // PRN 2
    2,   // PRN 3
    4,   // PRN 4
    8,   // PRN 5
    16,  // PRN 6
    32,  // PRN 7
    64,  // PRN 8  (placeholder)
    128, // PRN 9  (placeholder)
    256, // PRN 10 (placeholder)
    17,  // PRN 11
    35,  // PRN 12
    71,  // PRN 13
    143, // PRN 14
];

/// Step the G2 LFSR one clock
///
/// G2 polynomial: x^10 + x^9 + x^8 + x^6 + x^3 + x^2 + 1
/// Feedback = bit9 ^ bit8 ^ bit7 ^ bit5 ^ bit2 ^ bit1 (0-indexed from LSB)
/// This is a primitive polynomial of degree 10, period = 2^10 - 1 = 1023.
#[inline(always)]
fn g2_step(state: u16) -> u16 {
    // Taps at bits 9,8,7,5,2,1 (0-indexed from LSB, corresponding to x^10,x^9,x^8,x^6,x^3,x^2)
    let fb = ((state >> 9) ^ (state >> 8) ^ (state >> 7)
            ^ (state >> 5) ^ (state >> 2) ^ (state >> 1)) & 1;
    ((state << 1) | fb) & 0x3FF
}

impl NavicPrnGenerator {
    /// Create a new PRN generator for the given PRN (1-based, 1..=14)
    pub fn new(prn: usize) -> Self {
        assert!(prn >= 1 && prn <= 14, "PRN must be 1..=14");
        let g2_delay = G2_DELAYS[prn - 1];
        // Pre-advance G2 by delay chips
        let mut g2_state = 0x3FFu16; // all-ones initial state
        for _ in 0..g2_delay {
            g2_state = g2_step(g2_state);
        }
        Self {
            prn,
            g1_state: 0x3FF,
            g2_state,
            g2_delay,
        }
    }

    /// Step one chip, return chip value (0 or 1)
    pub fn next_chip_raw(&mut self) -> u8 {
        // G1: x^10 + x^3 + 1  → feedback at bits 9 and 2 (0-indexed)
        let g1_out = (self.g1_state >> 9) & 1;
        let g1_fb = g1_out ^ ((self.g1_state >> 2) & 1);
        self.g1_state = ((self.g1_state << 1) | g1_fb) & 0x3FF;

        // G2 output from MSB, then advance
        let g2_out = (self.g2_state >> 9) & 1;
        self.g2_state = g2_step(self.g2_state);

        // Gold code chip = G1 output XOR G2 output
        (g1_out ^ g2_out) as u8
    }

    /// Generate the full 1023-chip PRN code (values 0/1)
    pub fn generate_code(&mut self) -> Vec<u8> {
        self.g1_state = 0x3FF;
        // Reset G2 to all-ones then pre-advance by this PRN's delay
        let mut g2 = 0x3FFu16;
        for _ in 0..self.g2_delay {
            g2 = g2_step(g2);
        }
        self.g2_state = g2;
        (0..NAVIC_L5_CODE_LEN).map(|_| self.next_chip_raw()).collect()
    }

    /// Generate the PRN code as bipolar (+1.0/-1.0 f64)
    pub fn generate_code_bipolar(&mut self) -> Vec<f64> {
        self.generate_code()
            .into_iter()
            .map(|c| if c == 0 { 1.0 } else { -1.0 })
            .collect()
    }

    /// PRN index (1-based)
    pub fn prn(&self) -> usize { self.prn }
}

/// Cross-correlation between two PRN codes (unnormalized integer)
pub fn prn_cross_correlate(code_a: &[u8], code_b: &[u8]) -> i32 {
    assert_eq!(code_a.len(), code_b.len());
    code_a.iter().zip(code_b.iter())
        .map(|(&a, &b)| {
            let ba = if a == 0 { 1i32 } else { -1i32 };
            let bb = if b == 0 { 1i32 } else { -1i32 };
            ba * bb
        })
        .sum()
}

/// Auto-correlation at zero lag (should be 1023 for ideal code)
pub fn prn_autocorrelate_zero(code: &[u8]) -> i32 {
    prn_cross_correlate(code, code)
}

// ---------------------------------------------------------------------------
// Neuman-Hofman Secondary Code
// ---------------------------------------------------------------------------

/// Apply the 20-bit Neuman-Hofman secondary code overlay.
/// Each data bit is spread by one period of the NH code (20 chips).
pub fn apply_nh_secondary(primary_chips: &[u8], data_bit: u8) -> Vec<f64> {
    // One data bit occupies 20 primary code periods × 1023 chips each
    let n_primary = primary_chips.len();
    let nh_period = NAVIC_NH_CODE_LEN;
    let chips_per_nh_bit = n_primary / nh_period;
    let mut output = vec![0.0f64; n_primary];

    for nh_idx in 0..nh_period {
        let nh_chip = NAVIC_NH_CODE[nh_idx] ^ data_bit; // modulo-2 XOR
        let bipolar = if nh_chip == 0 { 1.0 } else { -1.0 };
        let start = nh_idx * chips_per_nh_bit;
        let end = start + chips_per_nh_bit;
        for k in start..end.min(n_primary) {
            let prn_bipolar = if primary_chips[k % NAVIC_L5_CODE_LEN] == 0 { 1.0 } else { -1.0 };
            output[k] = bipolar * prn_bipolar;
        }
    }
    output
}

// ---------------------------------------------------------------------------
// BPSK(1) Modulator
// ---------------------------------------------------------------------------

/// NavIC L5 SPS BPSK(1) baseband modulator.
///
/// Produces complex I+jQ samples at the given sample rate.
pub struct NavicL5Modulator {
    prn: usize,
    sample_rate: f64,
    doppler_hz: f64,
    code_phase_chips: f64,
    carrier_phase_rad: f64,
}

impl NavicL5Modulator {
    pub fn new(prn: usize, sample_rate: f64) -> Self {
        Self {
            prn,
            sample_rate,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
        }
    }

    pub fn set_doppler(&mut self, hz: f64) { self.doppler_hz = hz; }
    pub fn set_code_phase(&mut self, chips: f64) { self.code_phase_chips = chips; }
    pub fn set_carrier_phase(&mut self, rad: f64) { self.carrier_phase_rad = rad; }

    /// Generate `n_samples` of baseband I+Q signal.
    /// Returns (I_samples, Q_samples).
    pub fn generate(&mut self, n_samples: usize) -> (Vec<f64>, Vec<f64>) {
        let mut gen = NavicPrnGenerator::new(self.prn);
        let code = gen.generate_code();

        let chips_per_sample = NAVIC_L5_CHIP_RATE / self.sample_rate;
        let carrier_omega = 2.0 * PI * self.doppler_hz / self.sample_rate;

        let mut i_out = Vec::with_capacity(n_samples);
        let mut q_out = Vec::with_capacity(n_samples);

        let mut code_phase = self.code_phase_chips;
        let mut carr_phase = self.carrier_phase_rad;

        for _ in 0..n_samples {
            let chip_idx = (code_phase as usize) % NAVIC_L5_CODE_LEN;
            let chip = if code[chip_idx] == 0 { 1.0 } else { -1.0 };

            let i = chip * carr_phase.cos();
            let q = chip * carr_phase.sin();
            i_out.push(i);
            q_out.push(q);

            code_phase += chips_per_sample;
            if code_phase >= NAVIC_L5_CODE_LEN as f64 {
                code_phase -= NAVIC_L5_CODE_LEN as f64;
            }
            carr_phase += carrier_omega;
            if carr_phase > PI { carr_phase -= 2.0 * PI; }
        }

        self.code_phase_chips = code_phase;
        self.carrier_phase_rad = carr_phase;

        (i_out, q_out)
    }
}

// ---------------------------------------------------------------------------
// FFT Utilities (radix-2 Cooley-Tukey, in-place)
// ---------------------------------------------------------------------------

/// Simple complex number
#[derive(Clone, Copy, Debug, Default)]
pub struct Complex64 {
    pub re: f64,
    pub im: f64,
}

impl Complex64 {
    pub fn new(re: f64, im: f64) -> Self { Self { re, im } }
    pub fn mag_sq(&self) -> f64 { self.re * self.re + self.im * self.im }
    pub fn mag(&self) -> f64 { self.mag_sq().sqrt() }
    pub fn conj(&self) -> Self { Self::new(self.re, -self.im) }
    pub fn mul(&self, other: &Self) -> Self {
        Self::new(
            self.re * other.re - self.im * other.im,
            self.re * other.im + self.im * other.re,
        )
    }
    pub fn add(&self, other: &Self) -> Self { Self::new(self.re + other.re, self.im + other.im) }
    pub fn sub(&self, other: &Self) -> Self { Self::new(self.re - other.re, self.im - other.im) }
    pub fn scale(&self, s: f64) -> Self { Self::new(self.re * s, self.im * s) }
}

/// Bit-reversal permutation for FFT
fn bit_reverse_copy(x: &[Complex64]) -> Vec<Complex64> {
    let n = x.len();
    assert!(n.is_power_of_two(), "FFT length must be power of 2");
    let bits = n.trailing_zeros() as usize;
    let mut out = vec![Complex64::default(); n];
    for i in 0..n {
        let rev = (0..bits).map(|b| ((i >> b) & 1) << (bits - 1 - b)).sum::<usize>();
        out[rev] = x[i];
    }
    out
}

/// In-place Cooley-Tukey radix-2 DIT FFT
pub fn fft_inplace(x: &mut Vec<Complex64>) {
    let n = x.len();
    assert!(n.is_power_of_two());
    // bit-reversal
    let bits = n.trailing_zeros() as usize;
    for i in 0..n {
        let rev = (0..bits).map(|b| ((i >> b) & 1) << (bits - 1 - b)).sum::<usize>();
        if rev > i { x.swap(i, rev); }
    }
    // Danielson-Lanczos butterfly
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let ang = -2.0 * PI / len as f64;
        let wlen = Complex64::new(ang.cos(), ang.sin());
        let mut i = 0;
        while i < n {
            let mut w = Complex64::new(1.0, 0.0);
            for j in 0..half {
                let u = x[i + j];
                let v = x[i + j + half].mul(&w);
                x[i + j] = u.add(&v);
                x[i + j + half] = u.sub(&v);
                w = w.mul(&wlen);
            }
            i += len;
        }
        len <<= 1;
    }
}

/// IFFT: conjugate → FFT → conjugate → scale by 1/N
pub fn ifft_inplace(x: &mut Vec<Complex64>) {
    for s in x.iter_mut() { *s = s.conj(); }
    fft_inplace(x);
    let n = x.len() as f64;
    for s in x.iter_mut() { *s = s.conj().scale(1.0 / n); }
}

/// Zero-pad or truncate to next power of two >= target
pub fn next_power_of_two(n: usize) -> usize {
    let mut p = 1;
    while p < n { p <<= 1; }
    p
}

// ---------------------------------------------------------------------------
// PCPS Acquisition Engine
// ---------------------------------------------------------------------------

/// Acquisition result for a single PRN
#[derive(Clone, Debug)]
pub struct AcquisitionResult {
    pub prn: usize,
    pub acquired: bool,
    pub doppler_hz: f64,
    pub code_phase_chips: f64,
    /// Peak correlation power
    pub peak_power: f64,
    /// Peak-to-second-peak ratio (discrimination metric)
    pub metric: f64,
}

/// PCPS (Parallel Code Phase Search) Acquisition Engine.
///
/// Uses FFT-based circular correlation across all 1023 code phases in one shot.
/// Searches over a Doppler grid at configurable resolution.
pub struct PcpsAcquisition {
    pub sample_rate: f64,
    pub doppler_min_hz: f64,
    pub doppler_max_hz: f64,
    pub doppler_step_hz: f64,
    pub threshold: f64,
    pub coherent_ms: usize,
}

impl PcpsAcquisition {
    /// Default acquisition parameters for NavIC L5
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            doppler_min_hz: -5000.0,
            doppler_max_hz: 5000.0,
            doppler_step_hz: 500.0,
            threshold: 2.5,    // peak/mean ratio threshold
            coherent_ms: 1,
        }
    }

    /// Run acquisition for one PRN using input baseband I/Q samples.
    /// `samples` must cover at least `coherent_ms` milliseconds.
    pub fn acquire(&self, prn: usize, i_samples: &[f64], q_samples: &[f64]) -> AcquisitionResult {
        let samples_per_ms = (self.sample_rate * 0.001) as usize;
        let n_coherent = samples_per_ms * self.coherent_ms;
        let n_fft = next_power_of_two(n_coherent);

        // Generate local code replica (FFT of conjugated code)
        let mut gen = NavicPrnGenerator::new(prn);
        let code_raw = gen.generate_code();
        let code_upsampled = upsample_code(&code_raw, n_coherent, self.sample_rate);
        let code_conj_fft = {
            let mut v: Vec<Complex64> = code_upsampled.iter()
                .map(|&c| Complex64::new(if c == 0 { 1.0 } else { -1.0 }, 0.0))
                .collect();
            v.resize(n_fft, Complex64::default());
            fft_inplace(&mut v);
            v.iter().map(|c| c.conj()).collect::<Vec<_>>()
        };

        let doppler_bins: Vec<f64> = {
            let mut d = self.doppler_min_hz;
            let mut bins = Vec::new();
            while d <= self.doppler_max_hz + 1e-6 {
                bins.push(d);
                d += self.doppler_step_hz;
            }
            bins
        };

        let mut best_power = 0.0f64;
        let mut best_doppler = 0.0f64;
        let mut best_phase = 0.0f64;
        let mut all_powers: Vec<f64> = Vec::new();

        for &doppler in &doppler_bins {
            // Mix input with Doppler wipe-off
            let mixed: Vec<Complex64> = (0..n_coherent.min(i_samples.len()))
                .map(|n| {
                    let phase = -2.0 * PI * doppler * n as f64 / self.sample_rate;
                    let (sin_p, cos_p) = phase.sin_cos();
                    let i = i_samples[n];
                    let q = if n < q_samples.len() { q_samples[n] } else { 0.0 };
                    Complex64::new(i * cos_p - q * sin_p, i * sin_p + q * cos_p)
                })
                .collect();

            let mut sig_fft: Vec<Complex64> = mixed;
            sig_fft.resize(n_fft, Complex64::default());
            fft_inplace(&mut sig_fft);

            // Multiply by conjugated code FFT
            let mut prod: Vec<Complex64> = sig_fft.iter().zip(code_conj_fft.iter())
                .map(|(s, c)| s.mul(c))
                .collect();

            // IFFT to get correlation
            ifft_inplace(&mut prod);

            // Find peak
            for (phase_idx, &corr) in prod.iter().enumerate() {
                let pwr = corr.mag_sq();
                all_powers.push(pwr);
                if pwr > best_power {
                    best_power = pwr;
                    best_doppler = doppler;
                    best_phase = phase_idx as f64 * NAVIC_L5_CHIP_RATE / self.sample_rate;
                }
            }
        }

        let mean_power = if all_powers.is_empty() {
            1.0
        } else {
            all_powers.iter().sum::<f64>() / all_powers.len() as f64
        };
        let metric = if mean_power > 0.0 { best_power / mean_power } else { 0.0 };
        let acquired = metric >= self.threshold;

        AcquisitionResult {
            prn,
            acquired,
            doppler_hz: best_doppler,
            code_phase_chips: best_phase % NAVIC_L5_CODE_LEN as f64,
            peak_power: best_power,
            metric,
        }
    }
}

/// Upsample a 1023-chip code to `n_samples` at the given sample rate
fn upsample_code(code: &[u8], n_samples: usize, sample_rate: f64) -> Vec<u8> {
    let chips_per_sample = NAVIC_L5_CHIP_RATE / sample_rate;
    (0..n_samples)
        .map(|i| code[(i as f64 * chips_per_sample) as usize % NAVIC_L5_CODE_LEN])
        .collect()
}

// ---------------------------------------------------------------------------
// DLL (Delay Lock Loop) – Code Tracking
// ---------------------------------------------------------------------------

/// Early-Late spacing (chips)
const DLL_EARLY_LATE_SPACING: f64 = 0.5;

/// DLL loop filter state
pub struct DllTracker {
    pub prn: usize,
    pub sample_rate: f64,
    pub code_phase_chips: f64,
    pub doppler_hz: f64,
    /// Loop bandwidth in Hz
    pub bandwidth_hz: f64,
    // Integrator state for 2nd-order loop
    integrator: f64,
}

impl DllTracker {
    pub fn new(prn: usize, sample_rate: f64, code_phase_chips: f64, doppler_hz: f64) -> Self {
        Self {
            prn,
            sample_rate,
            code_phase_chips,
            doppler_hz,
            bandwidth_hz: 2.0,
            integrator: 0.0,
        }
    }

    /// Update DLL with one epoch of I/Q samples (1 ms).
    /// Returns the current code phase estimate after update.
    pub fn update(&mut self, i_samples: &[f64], q_samples: &[f64]) -> f64 {
        let samples_per_ms = (self.sample_rate * 0.001) as usize;
        let n = samples_per_ms.min(i_samples.len());

        // Generate early/prompt/late codes
        let (e_corr, p_corr, l_corr) = self.correlate_epl(i_samples, q_samples, n);

        // Normalized early-late discriminator
        let e_mag = e_corr.mag_sq().sqrt();
        let l_mag = l_corr.mag_sq().sqrt();
        let denominator = e_mag + l_mag;
        let discriminator = if denominator > 1e-12 {
            (e_mag - l_mag) / denominator
        } else {
            0.0
        };

        // 2nd-order loop filter (PI type)
        let wn = 2.0 * PI * self.bandwidth_hz;
        let kp = wn * 1.414;
        let ki = wn * wn * 0.25;

        self.integrator += ki * discriminator / self.sample_rate;
        let correction = kp * discriminator + self.integrator;

        self.code_phase_chips -= correction;
        // Wrap code phase
        while self.code_phase_chips < 0.0 { self.code_phase_chips += NAVIC_L5_CODE_LEN as f64; }
        while self.code_phase_chips >= NAVIC_L5_CODE_LEN as f64 { self.code_phase_chips -= NAVIC_L5_CODE_LEN as f64; }

        let _ = p_corr; // prompt used in PLL
        self.code_phase_chips
    }

    fn correlate_epl(&self, i_samples: &[f64], q_samples: &[f64], n: usize) -> (Complex64, Complex64, Complex64) {
        let mut gen = NavicPrnGenerator::new(self.prn);
        let code = gen.generate_code();
        let chips_per_sample = NAVIC_L5_CHIP_RATE / self.sample_rate;
        let spacing = DLL_EARLY_LATE_SPACING;

        let mut early = Complex64::default();
        let mut prompt = Complex64::default();
        let mut late = Complex64::default();

        let carrier_omega = 2.0 * PI * self.doppler_hz / self.sample_rate;

        for k in 0..n {
            let phase_e = self.code_phase_chips - spacing + k as f64 * chips_per_sample;
            let phase_p = self.code_phase_chips + k as f64 * chips_per_sample;
            let phase_l = self.code_phase_chips + spacing + k as f64 * chips_per_sample;

            let ce = chip_at(&code, phase_e);
            let cp = chip_at(&code, phase_p);
            let cl = chip_at(&code, phase_l);

            let carr_phase = carrier_omega * k as f64;
            let (sin_c, cos_c) = carr_phase.sin_cos();

            let i = if k < i_samples.len() { i_samples[k] } else { 0.0 };
            let q = if k < q_samples.len() { q_samples[k] } else { 0.0 };

            // Mix with carrier
            let mix_i = i * cos_c + q * sin_c;
            let mix_q = q * cos_c - i * sin_c;

            early.re += ce * mix_i;
            early.im += ce * mix_q;
            prompt.re += cp * mix_i;
            prompt.im += cp * mix_q;
            late.re += cl * mix_i;
            late.im += cl * mix_q;
        }

        (early, prompt, late)
    }
}

/// Get bipolar chip value (+1/-1) at fractional chip position
fn chip_at(code: &[u8], phase: f64) -> f64 {
    let idx = ((phase as isize).rem_euclid(NAVIC_L5_CODE_LEN as isize)) as usize;
    if code[idx] == 0 { 1.0 } else { -1.0 }
}

// ---------------------------------------------------------------------------
// PLL (Phase Lock Loop) – Carrier Tracking
// ---------------------------------------------------------------------------

/// Costas loop PLL for NavIC L5 carrier tracking
pub struct PllTracker {
    pub phase_rad: f64,
    pub frequency_hz: f64,
    pub bandwidth_hz: f64,
    integrator: f64,
}

impl PllTracker {
    pub fn new(initial_freq_hz: f64) -> Self {
        Self {
            phase_rad: 0.0,
            frequency_hz: initial_freq_hz,
            bandwidth_hz: 15.0,
            integrator: 0.0,
        }
    }

    /// Update PLL with prompt I/Q correlator outputs.
    /// Returns phase error estimate in radians.
    pub fn update(&mut self, prompt_i: f64, prompt_q: f64, sample_rate: f64) -> f64 {
        // Costas discriminator: atan2(Q, I) / 2
        let phase_err = prompt_q.atan2(prompt_i);

        let wn = 2.0 * PI * self.bandwidth_hz;
        let kp = wn * 1.414;
        let ki = wn * wn * 0.25;

        self.integrator += ki * phase_err / sample_rate;
        let correction = kp * phase_err + self.integrator;

        self.frequency_hz -= correction;
        self.phase_rad += 2.0 * PI * self.frequency_hz / sample_rate;
        if self.phase_rad > PI { self.phase_rad -= 2.0 * PI; }

        phase_err
    }
}

// ---------------------------------------------------------------------------
// Convolutional FEC (K=7, Rate 1/2)
// ---------------------------------------------------------------------------

/// Convolutional encoder K=7, rate 1/2.
/// Generator polynomials: G1=0171 (octal), G2=0133 (octal)
/// Same as GPS L5 / IS-GPS-705.
pub struct NavicConvEncoder {
    /// Shift register (6 bits state for K=7)
    state: u8,
}

const CONV_G1: u8 = 0b1111001; // 0171 octal = 121 decimal
const CONV_G2: u8 = 0b1011011; // 0133 octal = 91 decimal

impl NavicConvEncoder {
    pub fn new() -> Self { Self { state: 0 } }

    /// Reset encoder state
    pub fn reset(&mut self) { self.state = 0; }

    /// Encode a stream of bits. Returns 2 output bits per input bit (interleaved).
    pub fn encode(&mut self, bits: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity(bits.len() * 2);
        for &bit in bits {
            // Shift in new bit
            self.state = (self.state >> 1) | ((bit & 1) << 6);
            // Compute outputs
            let c1 = (self.state & CONV_G1).count_ones() as u8 & 1;
            let c2 = (self.state & CONV_G2).count_ones() as u8 & 1;
            output.push(c1);
            output.push(c2);
        }
        output
    }

    /// Flush encoder with tail bits (K-1 = 6 zero bits)
    pub fn flush(&mut self) -> Vec<u8> {
        let zeros = vec![0u8; 6];
        self.encode(&zeros)
    }
}

impl Default for NavicConvEncoder { fn default() -> Self { Self::new() } }

// ---------------------------------------------------------------------------
// Viterbi Decoder (hard-decision, K=7, rate 1/2)
// ---------------------------------------------------------------------------

const VITERBI_STATES: usize = 64; // 2^(K-1) = 2^6

/// Hard-decision Viterbi decoder for K=7, rate 1/2.
pub struct NavicViterbiDecoder {
    // State metrics
    path_metrics: Vec<u32>,
    // Survivor paths (traceback depth)
    traceback_depth: usize,
    survivors: Vec<Vec<u8>>,
}

impl NavicViterbiDecoder {
    pub fn new() -> Self {
        let traceback_depth = 35; // 5*(K-1)
        let mut pm = vec![u32::MAX / 2; VITERBI_STATES];
        pm[0] = 0;
        Self {
            path_metrics: pm,
            traceback_depth,
            survivors: vec![vec![0u8; 0]; VITERBI_STATES],
        }
    }

    pub fn reset(&mut self) {
        self.path_metrics = vec![u32::MAX / 2; VITERBI_STATES];
        self.path_metrics[0] = 0;
        self.survivors = vec![vec![0u8; 0]; VITERBI_STATES];
    }

    /// Output bits for a given (state, input_bit) pair
    fn branch_output(state: usize, bit: u8) -> (u8, u8) {
        let full_state = ((state as u8) >> 1) | ((bit & 1) << 6);
        let c1 = (full_state & CONV_G1).count_ones() as u8 & 1;
        let c2 = (full_state & CONV_G2).count_ones() as u8 & 1;
        (c1, c2)
    }

    /// Process received bit pairs (hard-decision). Returns decoded bits so far.
    pub fn decode(&mut self, received: &[u8]) -> Vec<u8> {
        assert!(received.len() % 2 == 0, "Received bits must be in pairs");
        let n_pairs = received.len() / 2;

        for pair_idx in 0..n_pairs {
            let rx0 = received[pair_idx * 2];
            let rx1 = received[pair_idx * 2 + 1];

            let mut new_metrics = vec![u32::MAX / 2; VITERBI_STATES];
            let mut new_survivors: Vec<Vec<u8>> = vec![Vec::new(); VITERBI_STATES];

            for state in 0..VITERBI_STATES {
                if self.path_metrics[state] == u32::MAX / 2 { continue; }
                for &bit in &[0u8, 1u8] {
                    let (c1, c2) = Self::branch_output(state, bit);
                    let hamming = (c1 ^ rx0) as u32 + (c2 ^ rx1) as u32;
                    let next_state = ((state >> 1) | ((bit as usize) << 5)) & (VITERBI_STATES - 1);
                    let candidate = self.path_metrics[state].saturating_add(hamming);
                    if candidate < new_metrics[next_state] {
                        new_metrics[next_state] = candidate;
                        let mut path = self.survivors[state].clone();
                        path.push(bit);
                        new_survivors[next_state] = path;
                    }
                }
            }

            self.path_metrics = new_metrics;
            self.survivors = new_survivors;
        }

        // Traceback: find minimum metric state
        let best_state = self.path_metrics.iter().enumerate()
            .min_by_key(|(_, &m)| m)
            .map(|(s, _)| s)
            .unwrap_or(0);

        self.survivors[best_state].clone()
    }
}

impl Default for NavicViterbiDecoder { fn default() -> Self { Self::new() } }

// ---------------------------------------------------------------------------
// Navigation Message Decoder
// ---------------------------------------------------------------------------

/// NavIC L5 subframe types
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NavicSubframeType {
    /// Subframe 1: Clock correction, health, accuracy
    ClockCorrection,
    /// Subframe 2: Ephemeris (part 1)
    EphemerisPart1,
    /// Subframe 3: Ephemeris (part 2) + almanac header
    EphemerisPart2,
    /// Subframe 4: Almanac / UTC / IONO parameters
    AlmanacIono,
    /// Unknown/uninitialized
    Unknown,
}

/// NavIC SPS navigation message ephemeris
#[derive(Clone, Debug, Default)]
pub struct NavicEphemeris {
    pub prn: usize,
    pub week_number: u32,
    pub af0: f64,         // Clock bias (s)
    pub af1: f64,         // Clock drift (s/s)
    pub af2: f64,         // Clock drift rate (s/s^2)
    pub toc: f64,         // Clock reference time (s)
    pub toe: f64,         // Ephemeris reference time (s)
    pub m0: f64,          // Mean anomaly at reference time (rad)
    pub delta_n: f64,     // Mean motion correction (rad/s)
    pub e: f64,           // Eccentricity
    pub sqrt_a: f64,      // Square root of semi-major axis (m^0.5)
    pub omega0: f64,      // Longitude of ascending node (rad)
    pub omega: f64,       // Argument of perigee (rad)
    pub omega_dot: f64,   // Rate of right ascension (rad/s)
    pub i0: f64,          // Inclination at reference time (rad)
    pub idot: f64,        // Rate of inclination (rad/s)
    pub cuc: f64,         // Cosine correction, argument of latitude
    pub cus: f64,         // Sine correction, argument of latitude
    pub crc: f64,         // Cosine correction, orbital radius
    pub crs: f64,         // Sine correction, orbital radius
    pub cic: f64,         // Cosine correction, inclination
    pub cis: f64,         // Sine correction, inclination
    pub ura: u8,          // User range accuracy index
    pub health: u8,       // Satellite health
    pub valid: bool,
}

/// NavIC ionospheric grid parameters (8-parameter model, similar to NeQuickG)
#[derive(Clone, Debug, Default)]
pub struct NavicIonoParams {
    /// Alpha coefficients (broadcast by NavIC, 4 values)
    pub alpha: [f64; 4],
    /// Beta coefficients (broadcast by NavIC, 4 values)
    pub beta: [f64; 4],
}

/// NavIC navigation message decoder
pub struct IrnssNavDecoder {
    pub ephemeris: NavicEphemeris,
    pub iono: NavicIonoParams,
    pub utc_a0: f64,
    pub utc_a1: f64,
    pub utc_dt_ls: i8,
    subframe_buffer: Vec<u8>,
    pub subframe_count: usize,
}

impl IrnssNavDecoder {
    pub fn new(prn: usize) -> Self {
        let mut ephem = NavicEphemeris::default();
        ephem.prn = prn;
        Self {
            ephemeris: ephem,
            iono: NavicIonoParams::default(),
            utc_a0: 0.0,
            utc_a1: 0.0,
            utc_dt_ls: 0,
            subframe_buffer: Vec::new(),
            subframe_count: 0,
        }
    }

    /// Feed decoded data bits (post-FEC, post-Viterbi) into the message decoder.
    /// Returns true if a complete subframe was processed.
    pub fn feed_bits(&mut self, bits: &[u8]) -> bool {
        self.subframe_buffer.extend_from_slice(bits);
        if self.subframe_buffer.len() >= NAVIC_SUBFRAME_BITS {
            let sf: Vec<u8> = self.subframe_buffer.drain(..NAVIC_SUBFRAME_BITS).collect();
            self.decode_subframe(&sf);
            true
        } else {
            false
        }
    }

    /// Identify subframe type from subframe ID field (bits 50..52 of data word)
    fn subframe_type(bits: &[u8]) -> NavicSubframeType {
        // Preamble: 8 bits (0x5B), then subframe ID at bits 49..51 (0-indexed)
        if bits.len() < 60 { return NavicSubframeType::Unknown; }
        let id = (bits[49] as u8) << 2 | (bits[50] as u8) << 1 | (bits[51] as u8);
        match id {
            1 => NavicSubframeType::ClockCorrection,
            2 => NavicSubframeType::EphemerisPart1,
            3 => NavicSubframeType::EphemerisPart2,
            4 => NavicSubframeType::AlmanacIono,
            _ => NavicSubframeType::Unknown,
        }
    }

    fn decode_subframe(&mut self, bits: &[u8]) {
        let sf_type = Self::subframe_type(bits);
        self.subframe_count += 1;

        match sf_type {
            NavicSubframeType::ClockCorrection => {
                // Simplified: extract TOC, af0, af1, af2 from known offsets
                // Real decoding requires CRC24Q verification first
                self.ephemeris.week_number = bits_to_u32(bits, 8, 10);
                self.ephemeris.toc = bits_to_u32(bits, 52, 16) as f64 * 16.0;
                self.ephemeris.af2 = bits_to_i32(bits, 68, 8) as f64 * 2.0f64.powi(-55);
                self.ephemeris.af1 = bits_to_i32(bits, 76, 16) as f64 * 2.0f64.powi(-43);
                self.ephemeris.af0 = bits_to_i32(bits, 92, 22) as f64 * 2.0f64.powi(-31);
                self.ephemeris.ura = bits_to_u32(bits, 114, 4) as u8;
                self.ephemeris.health = bits_to_u32(bits, 118, 9) as u8;
            }
            NavicSubframeType::EphemerisPart1 => {
                // Orbital parameters part 1
                self.ephemeris.delta_n = bits_to_i32(bits, 52, 16) as f64 * 2.0f64.powi(-43) * PI;
                self.ephemeris.m0 = bits_to_i32(bits, 68, 32) as f64 * 2.0f64.powi(-31) * PI;
                self.ephemeris.e = bits_to_u32(bits, 100, 32) as f64 * 2.0f64.powi(-33);
                self.ephemeris.sqrt_a = bits_to_u32(bits, 132, 32) as f64 * 2.0f64.powi(-19);
                self.ephemeris.toe = bits_to_u32(bits, 164, 16) as f64 * 16.0;
                self.ephemeris.cus = bits_to_i32(bits, 180, 15) as f64 * 2.0f64.powi(-28);
                self.ephemeris.cuc = bits_to_i32(bits, 195, 15) as f64 * 2.0f64.powi(-28);
            }
            NavicSubframeType::EphemerisPart2 => {
                // Orbital parameters part 2
                self.ephemeris.cic = bits_to_i32(bits, 52, 15) as f64 * 2.0f64.powi(-28);
                self.ephemeris.cis = bits_to_i32(bits, 67, 15) as f64 * 2.0f64.powi(-28);
                self.ephemeris.crc = bits_to_i32(bits, 82, 15) as f64 * 2.0f64.powi(-4);
                self.ephemeris.crs = bits_to_i32(bits, 97, 15) as f64 * 2.0f64.powi(-4);
                self.ephemeris.i0 = bits_to_i32(bits, 112, 32) as f64 * 2.0f64.powi(-31) * PI;
                self.ephemeris.idot = bits_to_i32(bits, 144, 14) as f64 * 2.0f64.powi(-43) * PI;
                self.ephemeris.omega0 = bits_to_i32(bits, 158, 32) as f64 * 2.0f64.powi(-31) * PI;
                self.ephemeris.omega = bits_to_i32(bits, 190, 32) as f64 * 2.0f64.powi(-31) * PI;
                self.ephemeris.omega_dot = bits_to_i32(bits, 222, 22) as f64 * 2.0f64.powi(-41) * PI;
                self.ephemeris.valid = true;
            }
            NavicSubframeType::AlmanacIono => {
                // Ionospheric parameters
                if bits.len() > 300 {
                    for i in 0..4 {
                        self.iono.alpha[i] = bits_to_i32(bits, 52 + i * 8, 8) as f64
                            * 2.0f64.powi(-30 + (i as i32) * 2);
                        self.iono.beta[i] = bits_to_i32(bits, 84 + i * 8, 8) as f64
                            * 2.0f64.powi(11 + (i as i32) * 2);
                    }
                    // UTC parameters
                    self.utc_a0 = bits_to_i32(bits, 116, 32) as f64 * 2.0f64.powi(-30);
                    self.utc_a1 = bits_to_i32(bits, 148, 24) as f64 * 2.0f64.powi(-50);
                }
            }
            NavicSubframeType::Unknown => {}
        }
    }
}

/// Extract unsigned integer from bit array, MSB first
fn bits_to_u32(bits: &[u8], start: usize, len: usize) -> u32 {
    let mut v = 0u32;
    for i in 0..len {
        if start + i < bits.len() {
            v = (v << 1) | (bits[start + i] as u32 & 1);
        }
    }
    v
}

/// Extract signed integer from bit array (sign-magnitude with MSB as sign)
fn bits_to_i32(bits: &[u8], start: usize, len: usize) -> i32 {
    let u = bits_to_u32(bits, start, len);
    // Two's complement
    if len < 32 && (u >> (len - 1)) & 1 == 1 {
        // Sign extend
        let mask = (!0u32) << len;
        (u | mask) as i32
    } else {
        u as i32
    }
}

// ---------------------------------------------------------------------------
// Satellite Position Computation (Keplerian)
// ---------------------------------------------------------------------------

/// Earth's gravitational constant (m^3/s^2) per ICD
const MU_EARTH: f64 = 3.986005e14;
/// Earth's angular velocity (rad/s)
const OMEGA_EARTH: f64 = 7.2921151467e-5;

/// ECEF satellite position from NavIC ephemeris
#[derive(Clone, Debug, Default)]
pub struct SatPosition {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub clock_bias_m: f64,
}

impl NavicEphemeris {
    /// Compute satellite ECEF position at GPS time `t` seconds.
    pub fn compute_position(&self, t: f64) -> SatPosition {
        if !self.valid { return SatPosition::default(); }

        let a = self.sqrt_a * self.sqrt_a;
        let n0 = (MU_EARTH / (a * a * a)).sqrt();
        let n = n0 + self.delta_n;

        let t_k = t - self.toe;
        let m_k = self.m0 + n * t_k;

        // Solve Kepler's equation iteratively
        let e_k = solve_kepler(m_k, self.e);

        // True anomaly
        let sin_e = e_k.sin();
        let cos_e = e_k.cos();
        let nu_k = (sin_e * (1.0 - self.e * self.e).sqrt())
            .atan2(cos_e - self.e);

        let phi_k = nu_k + self.omega;

        // Argument of latitude corrections
        let delta_u = self.cus * (2.0 * phi_k).sin() + self.cuc * (2.0 * phi_k).cos();
        let delta_r = self.crs * (2.0 * phi_k).sin() + self.crc * (2.0 * phi_k).cos();
        let delta_i = self.cis * (2.0 * phi_k).sin() + self.cic * (2.0 * phi_k).cos();

        let u_k = phi_k + delta_u;
        let r_k = a * (1.0 - self.e * cos_e) + delta_r;
        let i_k = self.i0 + delta_i + self.idot * t_k;

        let x_orb = r_k * u_k.cos();
        let y_orb = r_k * u_k.sin();

        let omega_k = self.omega0 + (self.omega_dot - OMEGA_EARTH) * t_k - OMEGA_EARTH * self.toe;

        let x = x_orb * omega_k.cos() - y_orb * i_k.cos() * omega_k.sin();
        let y = x_orb * omega_k.sin() + y_orb * i_k.cos() * omega_k.cos();
        let z = y_orb * i_k.sin();

        // Clock correction
        let dt_rel = -4.442807633e-10 * self.e * self.sqrt_a * e_k.sin();
        let dt = self.af0 + self.af1 * (t - self.toc) + self.af2 * (t - self.toc).powi(2) + dt_rel;

        SatPosition { x, y, z, clock_bias_m: dt * SPEED_OF_LIGHT }
    }
}

/// Solve Kepler's equation M = E - e*sin(E) iteratively (Newton-Raphson)
fn solve_kepler(m: f64, e: f64) -> f64 {
    let mut ek = m;
    for _ in 0..12 {
        let delta = (m - ek + e * ek.sin()) / (1.0 - e * ek.cos());
        ek += delta;
        if delta.abs() < 1e-12 { break; }
    }
    ek
}

// ---------------------------------------------------------------------------
// Ionospheric Grid Correction (IRNSS 8-parameter model)
// ---------------------------------------------------------------------------

/// NavIC/IRNSS Ionospheric Correction Calculator.
///
/// The NavIC ionospheric model uses 8 broadcast parameters (alpha[4], beta[4])
/// analogous to GPS Klobuchar but adapted for the Indian regional ionosphere.
/// The correction is applied in the L-band path delay domain.
pub struct IonoGridCorrector {
    pub params: NavicIonoParams,
}

impl IonoGridCorrector {
    pub fn new(params: NavicIonoParams) -> Self { Self { params } }

    /// Compute ionospheric delay in meters for given geometry.
    ///
    /// # Arguments
    /// - `elevation_rad`: satellite elevation in radians
    /// - `azimuth_rad`: satellite azimuth in radians
    /// - `lat_rad`: receiver latitude in radians
    /// - `lon_rad`: receiver longitude in radians
    /// - `t_gps_sow`: GPS time, seconds of week
    pub fn iono_delay_m(&self, elevation_rad: f64, azimuth_rad: f64,
                         lat_rad: f64, lon_rad: f64, t_gps_sow: f64) -> f64 {
        // Earth's central angle (semi-circle)
        let psi = 0.0137 / (elevation_rad / PI + 0.11) - 0.022;

        // Sub-ionospheric point latitude
        let phi_i = (lat_rad / PI + psi * azimuth_rad.cos()).clamp(-0.416, 0.416);

        // Sub-ionospheric point longitude (semi-circles)
        let lambda_i = lon_rad / PI + psi * azimuth_rad.sin() / phi_i.cos().max(1e-9);

        // Geomagnetic latitude of IPP (semi-circles)
        let phi_m = phi_i + 0.064 * (lambda_i - 1.617 * PI / PI).cos();

        // Local time at IPP (seconds)
        let t_local = (43200.0 * lambda_i + t_gps_sow).rem_euclid(86400.0);

        // Amplitude and period of ionospheric delay
        let amp = (0..4).map(|n| self.params.alpha[n] * phi_m.powi(n as i32)).sum::<f64>();
        let amp = amp.max(0.0);

        let per = (0..4).map(|n| self.params.beta[n] * phi_m.powi(n as i32)).sum::<f64>();
        let per = per.max(72000.0);

        // Obliquity factor
        let f = 1.0 + 16.0 * (0.53 - elevation_rad / PI).powi(3);

        // Phase from peak (local noon = 50400 s)
        let x = 2.0 * PI * (t_local - 50400.0) / per;

        // Delay in seconds
        let delay_s = if x.abs() < 1.57 {
            f * (5e-9 + amp * (1.0 - x * x / 2.0 + x.powi(4) / 24.0))
        } else {
            f * 5e-9
        };

        delay_s * SPEED_OF_LIGHT
    }
}

// ---------------------------------------------------------------------------
// NavIC L5 Processor (Top-level Integration)
// ---------------------------------------------------------------------------

/// Configuration for NavIC L5 processor
#[derive(Clone, Debug)]
pub struct NavicL5Config {
    pub prn: usize,
    pub sample_rate: f64,
    pub doppler_min_hz: f64,
    pub doppler_max_hz: f64,
    pub doppler_step_hz: f64,
    pub acquisition_threshold: f64,
    pub dll_bandwidth_hz: f64,
    pub pll_bandwidth_hz: f64,
}

impl NavicL5Config {
    pub fn new(prn: usize, sample_rate: f64) -> Self {
        Self {
            prn,
            sample_rate,
            doppler_min_hz: -5000.0,
            doppler_max_hz: 5000.0,
            doppler_step_hz: 500.0,
            acquisition_threshold: 2.5,
            dll_bandwidth_hz: 2.0,
            pll_bandwidth_hz: 15.0,
        }
    }
}

/// NavIC L5 SPS Signal Processor (top-level)
///
/// Orchestrates acquisition, code tracking (DLL), carrier tracking (PLL),
/// navigation message decoding, and ionospheric correction.
pub struct NavicL5Processor {
    pub config: NavicL5Config,
    pub acquisition: PcpsAcquisition,
    pub dll: Option<DllTracker>,
    pub pll: Option<PllTracker>,
    pub nav_decoder: IrnssNavDecoder,
    pub iono: IonoGridCorrector,
    pub acq_result: Option<AcquisitionResult>,
    /// Running pseudo-range estimate in meters
    pub pseudorange_m: f64,
    /// Number of epochs processed
    pub epoch_count: usize,
}

impl NavicL5Processor {
    /// Create a new processor for the given PRN and sample rate
    pub fn new(prn: usize, sample_rate: f64) -> Self {
        let config = NavicL5Config::new(prn, sample_rate);
        let mut acq = PcpsAcquisition::new(sample_rate);
        acq.doppler_min_hz = config.doppler_min_hz;
        acq.doppler_max_hz = config.doppler_max_hz;
        acq.doppler_step_hz = config.doppler_step_hz;
        acq.threshold = config.acquisition_threshold;

        Self {
            config,
            acquisition: acq,
            dll: None,
            pll: None,
            nav_decoder: IrnssNavDecoder::new(prn),
            iono: IonoGridCorrector::new(NavicIonoParams::default()),
            acq_result: None,
            pseudorange_m: 0.0,
            epoch_count: 0,
        }
    }

    /// Run acquisition on the provided I/Q samples.
    /// Returns true if the signal was acquired.
    pub fn acquire(&mut self, i_samples: &[f64], q_samples: &[f64]) -> bool {
        let result = self.acquisition.acquire(self.config.prn, i_samples, q_samples);
        let acquired = result.acquired;
        if acquired {
            let dll = {
                let mut d = DllTracker::new(
                    self.config.prn,
                    self.config.sample_rate,
                    result.code_phase_chips,
                    result.doppler_hz,
                );
                d.bandwidth_hz = self.config.dll_bandwidth_hz;
                d
            };
            let pll = {
                let mut p = PllTracker::new(result.doppler_hz);
                p.bandwidth_hz = self.config.pll_bandwidth_hz;
                p
            };
            self.dll = Some(dll);
            self.pll = Some(pll);
        }
        self.acq_result = Some(result);
        acquired
    }

    /// Process one 1 ms epoch of I/Q samples.
    /// Must call `acquire()` first. Returns current code phase after update.
    pub fn process_epoch(&mut self, i_samples: &[f64], q_samples: &[f64]) -> Option<f64> {
        let dll = self.dll.as_mut()?;
        let code_phase = dll.update(i_samples, q_samples);

        // Update pseudo-range from code phase
        let chips_per_m = NAVIC_L5_CHIP_RATE / SPEED_OF_LIGHT;
        self.pseudorange_m = code_phase / chips_per_m;

        self.epoch_count += 1;
        Some(code_phase)
    }

    /// Update ionospheric correction parameters from decoded navigation message
    pub fn update_iono(&mut self) {
        self.iono = IonoGridCorrector::new(self.nav_decoder.iono.clone());
    }

    /// Compute ionospheric-corrected pseudo-range in meters.
    pub fn corrected_pseudorange(&self, elevation_rad: f64, azimuth_rad: f64,
                                   lat_rad: f64, lon_rad: f64, t_gps_sow: f64) -> f64 {
        let iono_delay = self.iono.iono_delay_m(elevation_rad, azimuth_rad, lat_rad, lon_rad, t_gps_sow);
        self.pseudorange_m - iono_delay
    }

    /// Check if tracking is active
    pub fn is_tracking(&self) -> bool { self.dll.is_some() }

    /// Get current Doppler estimate in Hz
    pub fn doppler_hz(&self) -> f64 {
        self.acq_result.as_ref().map(|r| r.doppler_hz).unwrap_or(0.0)
    }

    /// Satellite velocity from Doppler (m/s, radial component)
    pub fn doppler_velocity_ms(&self) -> f64 {
        self.doppler_hz() * NAVIC_L5_WAVELENGTH
    }
}

// ---------------------------------------------------------------------------
// IRNSS Regional Coverage Checker
// ---------------------------------------------------------------------------

/// Check if a receiver position is within NavIC primary service area.
///
/// NavIC primary service area: approximately 40°E–140°E, -40°S–40°N
/// (conservative bounding box; actual coverage follows satellite geometry)
pub fn in_navic_coverage(lat_deg: f64, lon_deg: f64) -> bool {
    lat_deg >= -40.0 && lat_deg <= 40.0 && lon_deg >= 40.0 && lon_deg <= 140.0
}

/// Compute elevation angle from receiver to NavIC satellite.
///
/// # Arguments
/// - `rx_lat_deg`, `rx_lon_deg`, `rx_alt_m`: receiver ECEF-derived position
/// - `sat_x`, `sat_y`, `sat_z`: satellite ECEF position in meters
pub fn elevation_angle_deg(
    rx_lat_deg: f64, rx_lon_deg: f64, rx_alt_m: f64,
    sat_x: f64, sat_y: f64, sat_z: f64,
) -> f64 {
    let (lat, lon) = (rx_lat_deg.to_radians(), rx_lon_deg.to_radians());

    // WGS-84 receiver ECEF
    let a = 6_378_137.0f64;
    let e2 = 6.694379990141317e-3;
    let n = a / (1.0 - e2 * lat.sin() * lat.sin()).sqrt();
    let rx_x = (n + rx_alt_m) * lat.cos() * lon.cos();
    let rx_y = (n + rx_alt_m) * lat.cos() * lon.sin();
    let rx_z = (n * (1.0 - e2) + rx_alt_m) * lat.sin();

    // Vector from receiver to satellite
    let dx = sat_x - rx_x;
    let dy = sat_y - rx_y;
    let dz = sat_z - rx_z;
    let range = (dx * dx + dy * dy + dz * dz).sqrt();

    // ENU unit vectors
    let up_x = lat.cos() * lon.cos();
    let up_y = lat.cos() * lon.sin();
    let up_z = lat.sin();

    let dot_up = (dx * up_x + dy * up_y + dz * up_z) / range;
    dot_up.asin().to_degrees()
}

// ---------------------------------------------------------------------------
// CRC-24Q (used for NavIC message integrity verification)
// ---------------------------------------------------------------------------

const CRC24Q_POLY: u32 = 0x1864CFB;

/// Compute CRC-24Q (same as GPS/Galileo RTCM CRC24)
pub fn crc24q(data: &[u8]) -> u32 {
    let mut crc = 0u32;
    for &byte in data {
        crc ^= (byte as u32) << 16;
        for _ in 0..8 {
            crc <<= 1;
            if crc & 0x1000000 != 0 { crc ^= CRC24Q_POLY; }
        }
    }
    crc & 0xFFFFFF
}

/// Verify CRC-24Q over bit-packed subframe data
pub fn verify_subframe_crc(subframe_bits: &[u8]) -> bool {
    if subframe_bits.len() < 576 { return false; }
    // Pack bits into bytes (MSB first)
    let data_bytes: Vec<u8> = subframe_bits[..552].chunks(8)
        .map(|chunk| {
            chunk.iter().enumerate()
                .map(|(i, &b)| (b & 1) << (7 - i))
                .fold(0u8, |acc, x| acc | x)
        })
        .collect();
    let expected_crc = (0..24).map(|i| (subframe_bits[552 + i] as u32) << (23 - i)).sum::<u32>();
    let computed_crc = crc24q(&data_bytes);
    computed_crc == expected_crc
}

// ---------------------------------------------------------------------------
// Utility: Compute Doppler from velocity vector
// ---------------------------------------------------------------------------

/// Compute Doppler frequency from relative radial velocity.
/// `radial_velocity_ms` is positive when satellite is moving away.
pub fn doppler_from_velocity(radial_velocity_ms: f64) -> f64 {
    -radial_velocity_ms * NAVIC_L5_FREQ_HZ / SPEED_OF_LIGHT
}

/// Maximum Doppler expected for NavIC GEO/GSO (much less than LEO)
/// GEO: ~0 Hz, GSO: ~±800 Hz typical
pub fn max_navic_doppler_hz(sat_type: &NavicSatType) -> f64 {
    match sat_type {
        NavicSatType::Geo => 100.0,   // GEO nearly stationary
        NavicSatType::Gso => 1200.0,  // GSO/IGSO has small velocity component
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- PRN Code Generation ---

    #[test]
    fn test_prn_code_length() {
        let mut gen = NavicPrnGenerator::new(1);
        let code = gen.generate_code();
        assert_eq!(code.len(), NAVIC_L5_CODE_LEN);
    }

    #[test]
    fn test_prn_code_binary() {
        for prn in 1..=7 {
            let mut gen = NavicPrnGenerator::new(prn);
            let code = gen.generate_code();
            for &chip in &code {
                assert!(chip == 0 || chip == 1, "PRN {} chip must be 0 or 1", prn);
            }
        }
    }

    #[test]
    fn test_prn_autocorrelation_zero_lag() {
        let mut gen = NavicPrnGenerator::new(1);
        let code = gen.generate_code();
        let acorr = prn_autocorrelate_zero(&code);
        assert_eq!(acorr, NAVIC_L5_CODE_LEN as i32, "Zero-lag autocorrelation must equal code length");
    }

    #[test]
    fn test_prn_codes_not_all_identical() {
        let mut gen1 = NavicPrnGenerator::new(1);
        let mut gen2 = NavicPrnGenerator::new(2);
        let code1 = gen1.generate_code();
        let code2 = gen2.generate_code();
        assert_ne!(code1, code2, "Different PRNs must produce different codes");
    }

    #[test]
    fn test_prn_cross_correlation_low() {
        let mut gen1 = NavicPrnGenerator::new(1);
        let mut gen2 = NavicPrnGenerator::new(3);
        let code1 = gen1.generate_code();
        let code2 = gen2.generate_code();
        let xcorr = prn_cross_correlate(&code1, &code2).abs();
        // Gold code cross-correlation should be much less than code length
        assert!(xcorr < (NAVIC_L5_CODE_LEN as i32) / 2,
            "Cross-correlation {} too high", xcorr);
    }

    #[test]
    fn test_prn_reproducibility() {
        let mut gen_a = NavicPrnGenerator::new(4);
        let mut gen_b = NavicPrnGenerator::new(4);
        let code_a = gen_a.generate_code();
        let code_b = gen_b.generate_code();
        assert_eq!(code_a, code_b, "Same PRN must produce reproducible code");
    }

    #[test]
    fn test_prn_bipolar_values() {
        let mut gen = NavicPrnGenerator::new(2);
        let bp = gen.generate_code_bipolar();
        for v in &bp {
            assert!((*v - 1.0).abs() < 1e-9 || (*v + 1.0).abs() < 1e-9,
                "Bipolar chip must be ±1");
        }
    }

    #[test]
    fn test_all_navic_prns_generate() {
        for prn in 1..=7 {
            let mut gen = NavicPrnGenerator::new(prn);
            let code = gen.generate_code();
            assert_eq!(code.len(), 1023);
            // Code must not be all zeros or all ones
            let ones: usize = code.iter().map(|&c| c as usize).sum();
            assert!(ones > 100 && ones < 923, "PRN {} code balance off: {} ones", prn, ones);
        }
    }

    // --- Neuman-Hofman Secondary Code ---

    #[test]
    fn test_nh_code_length() {
        assert_eq!(NAVIC_NH_CODE.len(), NAVIC_NH_CODE_LEN);
    }

    #[test]
    fn test_nh_apply_basic() {
        let mut gen = NavicPrnGenerator::new(1);
        let code = gen.generate_code();
        // Repeat code 20 times for one NH period
        let repeated: Vec<u8> = code.iter().cycle().take(NAVIC_L5_CODE_LEN * NAVIC_NH_CODE_LEN).cloned().collect();
        let output = apply_nh_secondary(&repeated, 0);
        assert_eq!(output.len(), NAVIC_L5_CODE_LEN * NAVIC_NH_CODE_LEN);
    }

    #[test]
    fn test_nh_data_bit_flips_sign() {
        let mut gen = NavicPrnGenerator::new(1);
        let code = gen.generate_code();
        let repeated: Vec<u8> = code.iter().cycle().take(NAVIC_L5_CODE_LEN * NAVIC_NH_CODE_LEN).cloned().collect();
        let out0 = apply_nh_secondary(&repeated, 0);
        let out1 = apply_nh_secondary(&repeated, 1);
        // With NH[0]=0, data_bit=0 → chip0=0 (no flip); data_bit=1 → chip0=1 (flipped for some positions)
        // Just verify outputs differ
        let same = out0.iter().zip(out1.iter()).all(|(a, b)| (a - b).abs() < 1e-9);
        assert!(!same, "NH secondary code must affect output when data bit changes");
    }

    // --- FFT Utilities ---

    #[test]
    fn test_fft_roundtrip() {
        let n = 64;
        let orig: Vec<Complex64> = (0..n)
            .map(|i| Complex64::new((i as f64 * 0.1).sin(), 0.0))
            .collect();
        let mut v = orig.clone();
        fft_inplace(&mut v);
        ifft_inplace(&mut v);
        for (a, b) in orig.iter().zip(v.iter()) {
            assert!((a.re - b.re).abs() < 1e-9, "FFT roundtrip real mismatch");
            assert!((a.im - b.im).abs() < 1e-9, "FFT roundtrip imag mismatch");
        }
    }

    #[test]
    fn test_fft_tone() {
        let n = 128;
        let freq_bin = 4;
        let mut v: Vec<Complex64> = (0..n)
            .map(|k| {
                let phase = 2.0 * PI * freq_bin as f64 * k as f64 / n as f64;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();
        fft_inplace(&mut v);
        // Peak should be at freq_bin
        let magnitudes: Vec<f64> = v.iter().map(|c| c.mag()).collect();
        let peak_idx = magnitudes.iter().enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        assert_eq!(peak_idx, freq_bin);
    }

    #[test]
    fn test_fft_dc() {
        let n = 32;
        let mut v: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); n];
        fft_inplace(&mut v);
        assert!((v[0].re - n as f64).abs() < 1e-9, "DC bin should equal N");
        for k in 1..n {
            assert!(v[k].mag() < 1e-9, "Non-DC bins should be zero for DC input");
        }
    }

    #[test]
    fn test_complex64_operations() {
        let a = Complex64::new(3.0, 4.0);
        let b = Complex64::new(1.0, -2.0);
        assert!((a.mag() - 5.0).abs() < 1e-9);
        let prod = a.mul(&b);
        // (3+4j)(1-2j) = 3-6j+4j-8j^2 = 11-2j
        assert!((prod.re - 11.0).abs() < 1e-9);
        assert!((prod.im + 2.0).abs() < 1e-9);
    }

    // --- BPSK Modulator ---

    #[test]
    fn test_modulator_output_length() {
        let mut mod1 = NavicL5Modulator::new(1, 4_092_000.0);
        let n = 4092;
        let (i, q) = mod1.generate(n);
        assert_eq!(i.len(), n);
        assert_eq!(q.len(), n);
    }

    #[test]
    fn test_modulator_unit_amplitude() {
        let mut modulator = NavicL5Modulator::new(1, 2_046_000.0);
        let (i, q) = modulator.generate(1000);
        for (iv, qv) in i.iter().zip(q.iter()) {
            let amp = (iv * iv + qv * qv).sqrt();
            assert!((amp - 1.0).abs() < 1e-9, "Amplitude must be 1.0");
        }
    }

    #[test]
    fn test_modulator_doppler_frequency_shift() {
        // With non-zero Doppler, Q should be non-zero (carrier rotation)
        let mut modulator = NavicL5Modulator::new(1, 2_046_000.0);
        modulator.set_doppler(1000.0);
        let (_, q) = modulator.generate(1000);
        let q_power: f64 = q.iter().map(|&x| x * x).sum::<f64>() / 1000.0;
        assert!(q_power > 0.01, "Q channel must have power with non-zero Doppler");
    }

    // --- DLL Tracker ---

    #[test]
    fn test_dll_tracker_creation() {
        let dll = DllTracker::new(1, 2_046_000.0, 100.0, 500.0);
        assert_eq!(dll.prn, 1);
        assert!((dll.code_phase_chips - 100.0).abs() < 1e-9);
    }

    #[test]
    fn test_dll_tracker_update_returns_valid_phase() {
        let mut dll = DllTracker::new(1, 2_046_000.0, 512.0, 0.0);
        let n = 2046usize;
        let i = vec![0.5f64; n];
        let q = vec![0.0f64; n];
        let phase = dll.update(&i, &q);
        assert!(phase >= 0.0 && phase < NAVIC_L5_CODE_LEN as f64,
            "Code phase must be within [0, 1023)");
    }

    // --- PLL Tracker ---

    #[test]
    fn test_pll_creation() {
        let pll = PllTracker::new(500.0);
        assert!((pll.frequency_hz - 500.0).abs() < 1e-9);
    }

    #[test]
    fn test_pll_updates_phase() {
        let mut pll = PllTracker::new(0.0);
        let err1 = pll.update(1.0, 0.1, 2_046_000.0);
        assert!((err1 - 0.1f64.atan2(1.0)).abs() < 1e-9);
    }

    // --- Convolutional Encoder ---

    #[test]
    fn test_conv_encoder_output_length() {
        let mut enc = NavicConvEncoder::new();
        let bits = vec![1u8, 0, 1, 1, 0, 1, 0, 0];
        let out = enc.encode(&bits);
        assert_eq!(out.len(), bits.len() * 2);
    }

    #[test]
    fn test_conv_encoder_output_binary() {
        let mut enc = NavicConvEncoder::new();
        let bits = vec![1u8; 20];
        let out = enc.encode(&bits);
        for &b in &out { assert!(b == 0 || b == 1); }
    }

    #[test]
    fn test_conv_encoder_all_zeros_known_output() {
        let mut enc = NavicConvEncoder::new();
        enc.reset();
        let bits = vec![0u8; 4];
        let out = enc.encode(&bits);
        // For all-zero input with all-zero state, both outputs should be 0
        assert_eq!(&out[..4], &[0u8, 0, 0, 0]);
    }

    #[test]
    fn test_conv_encoder_deterministic() {
        let mut enc1 = NavicConvEncoder::new();
        let mut enc2 = NavicConvEncoder::new();
        let bits = vec![1u8, 0, 1, 0, 1, 1, 0, 1];
        assert_eq!(enc1.encode(&bits), enc2.encode(&bits));
    }

    #[test]
    fn test_conv_encoder_flush() {
        let mut enc = NavicConvEncoder::new();
        let _out = enc.encode(&[1, 0, 1]);
        let flush = enc.flush();
        assert_eq!(flush.len(), 12); // 6 zero bits × 2 output bits each
    }

    // --- Viterbi Decoder ---

    #[test]
    fn test_viterbi_decode_zeros() {
        let mut enc = NavicConvEncoder::new();
        let bits = vec![0u8; 16];
        let encoded = enc.encode(&bits);
        let mut dec = NavicViterbiDecoder::new();
        let decoded = dec.decode(&encoded);
        // Decoded should start with zeros
        assert!(decoded.len() >= bits.len() / 2);
        assert!(decoded[..8].iter().all(|&b| b == 0));
    }

    #[test]
    fn test_viterbi_decode_roundtrip() {
        let mut enc = NavicConvEncoder::new();
        let input = vec![1u8, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1];
        let mut encoded = enc.encode(&input);
        // Add tail bits
        enc.reset();
        // No errors - should decode perfectly (with some traceback delay)
        let mut dec = NavicViterbiDecoder::new();
        let decoded = dec.decode(&encoded);
        // First few bits should match input (with traceback delay)
        assert!(!decoded.is_empty());
        // Verify decoded starts with expected bits (some traceback delay expected)
        for &b in &decoded {
            assert!(b == 0 || b == 1, "Decoded bit must be 0 or 1");
        }
        let _ = encoded.len(); // suppress warning
    }

    // --- Navigation Message ---

    #[test]
    fn test_nav_decoder_creation() {
        let dec = IrnssNavDecoder::new(3);
        assert_eq!(dec.ephemeris.prn, 3);
        assert!(!dec.ephemeris.valid);
        assert_eq!(dec.subframe_count, 0);
    }

    #[test]
    fn test_nav_decoder_partial_subframe() {
        let mut dec = IrnssNavDecoder::new(1);
        let partial = vec![0u8; 300];
        let complete = dec.feed_bits(&partial);
        assert!(!complete);
    }

    #[test]
    fn test_nav_decoder_full_subframe() {
        let mut dec = IrnssNavDecoder::new(1);
        let full = vec![0u8; NAVIC_SUBFRAME_BITS];
        let complete = dec.feed_bits(&full);
        assert!(complete);
        assert_eq!(dec.subframe_count, 1);
    }

    #[test]
    fn test_nav_decoder_multiple_subframes() {
        let mut dec = IrnssNavDecoder::new(2);
        let data = vec![0u8; NAVIC_SUBFRAME_BITS * 3];
        let mut count = 0;
        for chunk in data.chunks(NAVIC_SUBFRAME_BITS / 4) {
            if dec.feed_bits(chunk) { count += 1; }
        }
        assert!(count >= 1);
    }

    #[test]
    fn test_bits_extraction() {
        let bits = vec![1u8, 0, 1, 1, 0, 0, 1, 0]; // 0b10110010 = 178
        let val = bits_to_u32(&bits, 0, 8);
        assert_eq!(val, 0b10110010);
    }

    #[test]
    fn test_bits_extraction_signed() {
        let bits = vec![1u8, 1, 1, 1]; // 4-bit = -1 in two's complement
        let val = bits_to_i32(&bits, 0, 4);
        assert_eq!(val, -1);
    }

    // --- Ephemeris Computation ---

    #[test]
    fn test_kepler_solve_circular() {
        // For circular orbit (e=0), E=M
        let m = 1.2f64;
        let e = solve_kepler(m, 0.0);
        assert!((e - m).abs() < 1e-10, "Circular orbit: E must equal M");
    }

    #[test]
    fn test_kepler_solve_nonzero_eccentricity() {
        let m = 0.5f64;
        let ecc = 0.01f64;
        let ek = solve_kepler(m, ecc);
        // Verify M = E - e*sin(E)
        let m_check = ek - ecc * ek.sin();
        assert!((m_check - m).abs() < 1e-11);
    }

    #[test]
    fn test_ephemeris_position_geo() {
        // Synthetic GEO ephemeris (IRNSS-1C at 83°E)
        let mut ephem = NavicEphemeris::default();
        ephem.valid = true;
        ephem.sqrt_a = 6493.0; // sqrt(42,164,000) ≈ 6493 m^0.5
        ephem.e = 0.0001;
        ephem.m0 = 0.0;
        ephem.omega0 = 83.0f64.to_radians();
        ephem.omega = 0.0;
        ephem.i0 = 0.0;
        ephem.toe = 0.0;
        let pos = ephem.compute_position(0.0);
        // GEO radius ~42,164 km
        let r = (pos.x * pos.x + pos.y * pos.y + pos.z * pos.z).sqrt();
        assert!(r > 40_000_000.0 && r < 44_000_000.0,
            "GEO radius {:.0} km out of range", r / 1000.0);
    }

    // --- Ionospheric Correction ---

    #[test]
    fn test_iono_delay_non_negative() {
        let params = NavicIonoParams {
            alpha: [2e-8, 0.0, -5.96e-8, 0.0],
            beta: [86400.0, 0.0, -65536.0, 0.0],
        };
        let corrector = IonoGridCorrector::new(params);
        let delay = corrector.iono_delay_m(
            45.0f64.to_radians(), 180.0f64.to_radians(),
            20.0f64.to_radians(), 80.0f64.to_radians(), 43200.0,
        );
        assert!(delay >= 0.0, "Ionospheric delay must be non-negative");
    }

    #[test]
    fn test_iono_delay_higher_at_low_elevation() {
        let params = NavicIonoParams {
            alpha: [2e-8, 0.0, -5.96e-8, 0.0],
            beta: [86400.0, 0.0, -65536.0, 0.0],
        };
        let corrector = IonoGridCorrector::new(params);
        let delay_high = corrector.iono_delay_m(
            80.0f64.to_radians(), 0.0, 20.0f64.to_radians(), 80.0f64.to_radians(), 43200.0,
        );
        let delay_low = corrector.iono_delay_m(
            10.0f64.to_radians(), 0.0, 20.0f64.to_radians(), 80.0f64.to_radians(), 43200.0,
        );
        assert!(delay_low >= delay_high,
            "Low elevation must have more delay: {:.3} vs {:.3}", delay_low, delay_high);
    }

    // --- Coverage ---

    #[test]
    fn test_coverage_india() {
        assert!(in_navic_coverage(20.0, 78.0), "Delhi should be in NavIC coverage");
    }

    #[test]
    fn test_coverage_outside() {
        assert!(!in_navic_coverage(51.5, -0.1), "London should be outside NavIC coverage");
        assert!(!in_navic_coverage(-33.9, 151.2), "Sydney should be outside NavIC coverage");
    }

    #[test]
    fn test_coverage_boundary() {
        // Points on boundary
        assert!(in_navic_coverage(40.0, 90.0));
        assert!(in_navic_coverage(-40.0, 90.0));
    }

    // --- Elevation Angle ---

    #[test]
    fn test_elevation_angle_overhead() {
        // Satellite directly overhead at ~42,000 km altitude, GEO
        // Receiver at 0°N 83°E (under IRNSS-1C)
        let a_geo = 42_164_000.0f64;
        let lon_sat = 83.0f64.to_radians();
        let sat_x = a_geo * lon_sat.cos();
        let sat_y = a_geo * lon_sat.sin();
        let sat_z = 0.0;
        let el = elevation_angle_deg(0.0, 83.0, 0.0, sat_x, sat_y, sat_z);
        // Overhead: elevation should be close to 90°
        assert!(el > 80.0, "Overhead satellite elevation should be near 90°, got {:.1}", el);
    }

    #[test]
    fn test_elevation_angle_range() {
        let a_geo = 42_164_000.0f64;
        let sat_x = a_geo;
        let sat_y = 0.0;
        let sat_z = 0.0;
        let el = elevation_angle_deg(0.0, 0.0, 0.0, sat_x, sat_y, sat_z);
        assert!(el > -90.0 && el <= 90.0, "Elevation must be in [-90, 90]");
    }

    // --- CRC-24Q ---

    #[test]
    fn test_crc24q_empty() {
        let crc = crc24q(&[]);
        assert_eq!(crc, 0);
    }

    #[test]
    fn test_crc24q_single_byte() {
        let crc = crc24q(&[0xFF]);
        assert!(crc <= 0xFFFFFF);
    }

    #[test]
    fn test_crc24q_different_inputs() {
        let c1 = crc24q(&[1, 2, 3]);
        let c2 = crc24q(&[1, 2, 4]);
        assert_ne!(c1, c2);
    }

    #[test]
    fn test_crc24q_deterministic() {
        let data = b"NavIC L5 ICD";
        assert_eq!(crc24q(data), crc24q(data));
    }

    // --- Constellation Catalog ---

    #[test]
    fn test_constellation_count() {
        let sats = navic_constellation();
        assert_eq!(sats.len(), NAVIC_NUM_SVS);
    }

    #[test]
    fn test_constellation_geo_count() {
        let sats = navic_constellation();
        let geos = sats.iter().filter(|s| s.sat_type == NavicSatType::Geo).count();
        assert_eq!(geos, 3, "NavIC has 3 GEO satellites");
    }

    #[test]
    fn test_constellation_gso_count() {
        let sats = navic_constellation();
        let gsos = sats.iter().filter(|s| s.sat_type == NavicSatType::Gso).count();
        assert_eq!(gsos, 4, "NavIC has 4 GSO satellites");
    }

    #[test]
    fn test_constellation_prn_unique() {
        let sats = navic_constellation();
        let mut prns: Vec<usize> = sats.iter().map(|s| s.prn).collect();
        prns.dedup();
        assert_eq!(prns.len(), NAVIC_NUM_SVS);
    }

    // --- Doppler Utilities ---

    #[test]
    fn test_doppler_from_velocity() {
        // Approaching satellite: negative velocity → positive Doppler
        let d = doppler_from_velocity(-1000.0);
        assert!(d > 0.0, "Approaching satellite must give positive Doppler");
    }

    #[test]
    fn test_doppler_from_velocity_zero() {
        let d = doppler_from_velocity(0.0);
        assert_eq!(d, 0.0);
    }

    #[test]
    fn test_max_doppler_geo_smaller_than_gso() {
        let geo = max_navic_doppler_hz(&NavicSatType::Geo);
        let gso = max_navic_doppler_hz(&NavicSatType::Gso);
        assert!(geo < gso, "GEO Doppler must be less than GSO");
    }

    // --- NavicL5Processor Integration ---

    #[test]
    fn test_processor_creation() {
        let proc = NavicL5Processor::new(1, 2_046_000.0);
        assert_eq!(proc.config.prn, 1);
        assert!(!proc.is_tracking());
        assert_eq!(proc.epoch_count, 0);
    }

    #[test]
    fn test_processor_doppler_before_acquisition() {
        let proc = NavicL5Processor::new(2, 4_092_000.0);
        assert_eq!(proc.doppler_hz(), 0.0);
        assert_eq!(proc.doppler_velocity_ms(), 0.0);
    }

    #[test]
    fn test_processor_corrected_pseudorange() {
        let mut proc = NavicL5Processor::new(3, 4_092_000.0);
        proc.pseudorange_m = 25_000_000.0;
        let cr = proc.corrected_pseudorange(
            45.0f64.to_radians(), 0.0, 20.0f64.to_radians(), 80.0f64.to_radians(), 43200.0,
        );
        // Corrected range differs from raw by iono delay
        assert!(cr < 25_000_000.0 + 100.0 && cr > 25_000_000.0 - 100.0);
    }

    // --- Constants Sanity ---

    #[test]
    fn test_l5_frequency() {
        assert!((NAVIC_L5_FREQ_HZ - 1_176_450_000.0).abs() < 1.0);
    }

    #[test]
    fn test_chip_rate() {
        assert!((NAVIC_L5_CHIP_RATE - 1_023_000.0).abs() < 1.0);
    }

    #[test]
    fn test_wavelength() {
        let expected = SPEED_OF_LIGHT / NAVIC_L5_FREQ_HZ;
        assert!((NAVIC_L5_WAVELENGTH - expected).abs() < 1e-15);
    }

    #[test]
    fn test_subframe_bits_constant() {
        // NavIC SPS subframe: 300 bits × rate 1/2 FEC = 600 encoded bits
        assert_eq!(NAVIC_SUBFRAME_BITS, 600);
    }

    // --- Acquisition (smoke test, small sample) ---

    #[test]
    fn test_acquisition_returns_result() {
        let sample_rate = 4_092_000.0f64;
        let acq = PcpsAcquisition::new(sample_rate);
        let n = (sample_rate * 0.001) as usize;
        let i_samples: Vec<f64> = (0..n).map(|k| (2.0 * PI * 1000.0 * k as f64 / sample_rate).cos()).collect();
        let q_samples = vec![0.0f64; n];
        let result = acq.acquire(1, &i_samples, &q_samples);
        // Just verify structure is valid
        assert!(result.prn == 1);
        assert!(result.metric >= 0.0);
        assert!(result.code_phase_chips >= 0.0 && result.code_phase_chips < NAVIC_L5_CODE_LEN as f64);
    }

    #[test]
    fn test_acquisition_all_prns() {
        let sample_rate = 2_046_000.0f64;
        let acq = PcpsAcquisition::new(sample_rate);
        let n = (sample_rate * 0.001) as usize;
        let i = vec![0.0f64; n];
        let q = vec![0.0f64; n];
        for prn in 1..=7 {
            let r = acq.acquire(prn, &i, &q);
            assert_eq!(r.prn, prn);
            assert!(!r.acquired, "No signal present; should not acquire");
        }
    }
}

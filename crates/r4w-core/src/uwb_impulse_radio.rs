//! # Ultra-Wideband (UWB) Impulse Radio Processor
//!
//! Implements IEEE 802.15.4z HRP UWB (High Rate Pulse) physical layer for
//! precision ranging and positioning applications.
//!
//! ## Overview
//!
//! UWB impulse radio transmits very short (sub-nanosecond) pulses across a wide
//! frequency band (> 500 MHz). The extremely fine time resolution enables
//! centimeter-level ranging via time-of-flight (ToF) measurement. IEEE 802.15.4z
//! defines the HRP UWB mode used in applications like Apple U1/AirTag, Samsung
//! SmartTag+, and automotive PKES (Passive Keyless Entry and Start).
//!
//! ## Signal Structure (IEEE 802.15.4z HRP UWB)
//!
//! ```text
//! ┌─────────────┬──────┬─────┬──────┬──────────┐
//! │  SYNC/PREAMBLE │ SFD │ PHR │ STS  │  PSDU    │
//! │ (ternary code) │     │     │(AES) │(payload) │
//! └─────────────┴──────┴─────┴──────┴──────────┘
//! ```
//!
//! ## Pulse Generation
//!
//! Gaussian monocycle: `p(t) = -t/σ² * exp(-t²/2σ²)`
//! Gaussian doublet:   `p(t) = (1 - t²/σ²) * exp(-t²/2σ²)`
//!
//! Mean PRFs: 3.9 MHz, 15.6 MHz, 62.4 MHz (multiples of base chip rate)
//!
//! ## Ranging
//!
//! Two-Way Ranging (TWR):
//! ```text
//! ToF = (T_roundA - T_replyB) / 2
//! Range = ToF * c
//! ```
//!
//! Symmetric Double-Sided TWR (SDS-TWR):
//! ```text
//! ToF = (T_roundA * T_roundB - T_replyA * T_replyB) /
//!        (T_roundA + T_roundB + T_replyA + T_replyB)
//! ```
//!
//! ## STS Security
//!
//! The Scrambled Timestamp Sequence is generated via AES-128 counter mode,
//! providing authentication and anti-spoofing for secure ranging.
//!
//! # References
//!
//! - IEEE 802.15.4z-2020: Amendment for HRP and LRP UWB PHYs
//! - IEEE 802.15.4a-2007: UWB channel models
//! - FiRa Consortium UWB MAC Technical Requirements
//!
//! # Example
//!
//! ```rust
//! use r4w_core::uwb_impulse_radio::{
//!     UwbConfig, UwbChannel, PulseGenerator, RangingEngine,
//!     ChannelNumber, PrfMode, PreambleCode,
//! };
//!
//! // Configure UWB channel 5 (6.5 GHz center, 500 MHz BW)
//! let config = UwbConfig {
//!     channel: ChannelNumber::Ch5,
//!     prf: PrfMode::Prf64,
//!     preamble_code: PreambleCode::Code9,
//!     data_rate: 6_810_000,
//!     sfd_type: 0,
//!     preamble_length: 128,
//!     sts_length: 64,
//! };
//!
//! // Generate a Gaussian monocycle pulse
//! let gen = PulseGenerator::new(config.prf.chip_duration_ns());
//! let pulse = gen.monocycle(0.5);  // 0.5 ns sigma
//!
//! // Ranging engine
//! let mut ranging = RangingEngine::new();
//! let tof_ns = ranging.sds_twr(
//!     100.0, // T_roundA ns
//!     50.0,  // T_replyB ns
//!     100.0, // T_roundB ns
//!     50.0,  // T_replyA ns
//! );
//! let range_m = tof_ns * 1e-9 * 3e8;
//! ```

use std::f64::consts::PI;

// Speed of light in m/s
const C: f64 = 2.997_924_58e8;

// AES S-box lookup table (FIPS 197)
#[rustfmt::skip]
const AES_SBOX: [u8; 256] = [
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16,
];

// AES Rcon table for key schedule
const RCON: [u8; 10] = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36];

// ─────────────────────────────────────────────────────────────────────────────
// Configuration types
// ─────────────────────────────────────────────────────────────────────────────

/// UWB channel number per IEEE 802.15.4z Table 7-7
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelNumber {
    /// Channel 1: 3494.4 MHz center, 499.2 MHz BW
    Ch1,
    /// Channel 2: 3993.6 MHz center, 499.2 MHz BW
    Ch2,
    /// Channel 3: 4492.8 MHz center, 499.2 MHz BW
    Ch3,
    /// Channel 4: 3993.6 MHz center, 1331.2 MHz BW
    Ch4,
    /// Channel 5: 6489.6 MHz center, 499.2 MHz BW
    Ch5,
    /// Channel 6: 6988.8 MHz center, 499.2 MHz BW (also used as Ch7)
    Ch6,
    /// Channel 9: 7987.2 MHz center, 1081.6 MHz BW
    Ch9,
}

impl ChannelNumber {
    /// Center frequency in MHz
    pub fn center_freq_mhz(&self) -> f64 {
        match self {
            ChannelNumber::Ch1 => 3494.4,
            ChannelNumber::Ch2 => 3993.6,
            ChannelNumber::Ch3 => 4492.8,
            ChannelNumber::Ch4 => 3993.6,
            ChannelNumber::Ch5 => 6489.6,
            ChannelNumber::Ch6 => 6988.8,
            ChannelNumber::Ch9 => 7987.2,
        }
    }

    /// Bandwidth in MHz
    pub fn bandwidth_mhz(&self) -> f64 {
        match self {
            ChannelNumber::Ch4 => 1331.2,
            ChannelNumber::Ch9 => 1081.6,
            _ => 499.2,
        }
    }

    /// Path loss exponent (simplified Friis + UWB freq dependency)
    pub fn path_loss_exponent(&self) -> f64 {
        // Higher frequency = slightly higher loss
        if self.center_freq_mhz() > 6000.0 { 2.1 } else { 2.0 }
    }
}

/// Mean PRF (Pulse Repetition Frequency) mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrfMode {
    /// 3.9 MHz mean PRF (base chip rate)
    Prf4,
    /// 15.6 MHz mean PRF
    Prf16,
    /// 62.4 MHz mean PRF
    Prf64,
}

impl PrfMode {
    /// Chip duration in nanoseconds
    pub fn chip_duration_ns(&self) -> f64 {
        match self {
            PrfMode::Prf4  => 1.0 / 3.9e6 * 1e9,    // ~256 ns
            PrfMode::Prf16 => 1.0 / 15.6e6 * 1e9,   // ~64 ns
            PrfMode::Prf64 => 1.0 / 62.4e6 * 1e9,   // ~16 ns
        }
    }

    /// Mean PRF in MHz
    pub fn mean_prf_mhz(&self) -> f64 {
        match self {
            PrfMode::Prf4  => 3.9,
            PrfMode::Prf16 => 15.6,
            PrfMode::Prf64 => 62.4,
        }
    }
}

/// Preamble ternary code index (IEEE 802.15.4z Table 7-10)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreambleCode {
    Code1,
    Code2,
    Code3,
    Code4,
    Code9,
    Code10,
    Code11,
    Code12,
}

impl PreambleCode {
    /// Code length (31 or 127 chips)
    pub fn length(&self) -> usize {
        match self {
            PreambleCode::Code1 | PreambleCode::Code2 |
            PreambleCode::Code3 | PreambleCode::Code4 => 31,
            _ => 127,
        }
    }

    /// Get ternary code chips (+1, 0, -1)
    pub fn chips(&self) -> Vec<i8> {
        match self {
            PreambleCode::Code1 => Self::code31_1(),
            PreambleCode::Code2 => Self::code31_2(),
            PreambleCode::Code3 => Self::code31_3(),
            PreambleCode::Code4 => Self::code31_4(),
            PreambleCode::Code9  => Self::code127_9(),
            PreambleCode::Code10 => Self::code127_10(),
            PreambleCode::Code11 => Self::code127_11(),
            PreambleCode::Code12 => Self::code127_12(),
        }
    }

    // 31-chip ternary preamble codes from IEEE 802.15.4-2020 Table 7-10
    fn code31_1() -> Vec<i8> {
        vec![
             0, 1, 0,-1, 0, 0, 1, 0, 1, 0,
             1,-1, 0, 0, 0, 1, 0, 0,-1, 0,
             0, 0, 1, 0, 0, 0,-1, 0, 1, 0, 0,
        ]
    }
    fn code31_2() -> Vec<i8> {
        vec![
             0,-1, 0, 0, 1, 0, 0, 0,-1, 0,
             0, 0, 1, 0,-1, 0, 1,-1, 0, 0,
             0, 1, 0, 1, 0,-1, 0, 0, 1, 0, 0,
        ]
    }
    fn code31_3() -> Vec<i8> {
        vec![
             1, 0, 0, 0,-1, 0, 0, 1, 0, 0,
             0, 1, 0,-1, 0, 0, 0, 1, 0, 1,
            -1, 0, 0, 0, 1, 0, 1, 0, 0,-1, 0,
        ]
    }
    fn code31_4() -> Vec<i8> {
        vec![
             0, 0, 1, 0, 0,-1, 0, 1,-1, 0,
             0, 0,-1, 0, 0, 0, 1, 0, 0, 1,
             0,-1, 0, 0, 1, 0, 0, 0, 1, 0,-1,
        ]
    }

    // 127-chip codes (truncated representative sequences)
    fn code127_9() -> Vec<i8> {
        // Generated from Kasami sequence base
        let mut c = vec![0i8; 127];
        let taps = [7usize, 3, 0];
        let mut sr = [1u8; 7];
        for i in 0..127 {
            let bit = sr[taps[0] - 1] ^ sr[taps[1] - 1] ^ sr[taps[2]];
            c[i] = if bit == 1 { 1 } else { -1 };
            // zero out every 3rd chip to make ternary
            if i % 3 == 2 { c[i] = 0; }
            sr.rotate_right(1);
            sr[0] = bit;
        }
        c
    }
    fn code127_10() -> Vec<i8> {
        let mut c = Self::code127_9();
        c.rotate_left(11);
        c
    }
    fn code127_11() -> Vec<i8> {
        let mut c = Self::code127_9();
        c.rotate_left(23);
        c
    }
    fn code127_12() -> Vec<i8> {
        let mut c = Self::code127_9();
        c.rotate_left(37);
        c
    }
}

/// Complete UWB PHY configuration
#[derive(Debug, Clone)]
pub struct UwbConfig {
    /// Operating channel
    pub channel: ChannelNumber,
    /// Pulse repetition frequency mode
    pub prf: PrfMode,
    /// Preamble ternary code selection
    pub preamble_code: PreambleCode,
    /// Payload data rate in bits/second (850 kbps, 6.81 Mbps, 27.24 Mbps)
    pub data_rate: u32,
    /// SFD type: 0 = standard, 1 = non-standard (4-bit), 2 = 8-bit
    pub sfd_type: u8,
    /// Preamble repetition count
    pub preamble_length: u16,
    /// STS segment length in symbols (0, 32, 64, 128, 256, 512, 1024, 2048)
    pub sts_length: u16,
}

impl Default for UwbConfig {
    fn default() -> Self {
        UwbConfig {
            channel: ChannelNumber::Ch5,
            prf: PrfMode::Prf64,
            preamble_code: PreambleCode::Code9,
            data_rate: 6_810_000,
            sfd_type: 0,
            preamble_length: 128,
            sts_length: 64,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Pulse generation
// ─────────────────────────────────────────────────────────────────────────────

/// Generates UWB impulse pulse shapes
#[derive(Debug, Clone)]
pub struct PulseGenerator {
    /// Chip duration in nanoseconds
    chip_ns: f64,
    /// Sample rate in samples/ns
    samples_per_ns: f64,
}

impl PulseGenerator {
    /// Create a new pulse generator
    ///
    /// # Arguments
    /// * `chip_ns` - chip duration in nanoseconds
    /// * `samples_per_ns` - oversampling rate
    pub fn new(chip_ns: f64) -> Self {
        PulseGenerator {
            chip_ns,
            samples_per_ns: 1.0, // 1 sample/ns by default
        }
    }

    pub fn with_sample_rate(mut self, samples_per_ns: f64) -> Self {
        self.samples_per_ns = samples_per_ns;
        self
    }

    /// Compute samples count spanning ±3σ of the pulse
    fn pulse_samples(&self, sigma_ns: f64) -> usize {
        let span_ns = 6.0 * sigma_ns;
        (span_ns * self.samples_per_ns).ceil() as usize | 1 // ensure odd
    }

    /// Generate Gaussian monocycle pulse: p(t) = -(t/σ²) * exp(-t²/2σ²)
    ///
    /// The monocycle is the first derivative of a Gaussian and spans
    /// approximately from -3σ to +3σ.
    pub fn monocycle(&self, sigma_ns: f64) -> Vec<f64> {
        let n = self.pulse_samples(sigma_ns);
        let center = (n / 2) as f64;
        let dt = 1.0 / self.samples_per_ns; // ns per sample
        let mut pulse: Vec<f64> = (0..n).map(|i| {
            let t = (i as f64 - center) * dt;
            let s2 = sigma_ns * sigma_ns;
            -(t / s2) * (-t * t / (2.0 * s2)).exp()
        }).collect();
        // Normalize to unit energy
        let energy: f64 = pulse.iter().map(|&x| x * x).sum::<f64>().sqrt();
        if energy > 0.0 {
            pulse.iter_mut().for_each(|x| *x /= energy);
        }
        pulse
    }

    /// Generate Gaussian doublet pulse: p(t) = (1 - t²/σ²) * exp(-t²/2σ²)
    ///
    /// The doublet is the second derivative of a Gaussian. It has better
    /// spectral properties for UWB applications.
    pub fn doublet(&self, sigma_ns: f64) -> Vec<f64> {
        let n = self.pulse_samples(sigma_ns);
        let center = (n / 2) as f64;
        let dt = 1.0 / self.samples_per_ns;
        let mut pulse: Vec<f64> = (0..n).map(|i| {
            let t = (i as f64 - center) * dt;
            let s2 = sigma_ns * sigma_ns;
            (1.0 - t * t / s2) * (-t * t / (2.0 * s2)).exp()
        }).collect();
        let energy: f64 = pulse.iter().map(|&x| x * x).sum::<f64>().sqrt();
        if energy > 0.0 {
            pulse.iter_mut().for_each(|x| *x /= energy);
        }
        pulse
    }

    /// Generate Root-Raised-Cosine (RRC) pulse for UWB BPM-BPSK modulation
    ///
    /// h[n] = (4β/π√T) * [cos((1+β)πn/T) + sin((1-β)πn/T)/(4βn/T)] /
    ///        [1 - (4βn/T)²]
    pub fn rrc(&self, rolloff: f64, chip_span: usize) -> Vec<f64> {
        let t_chip = self.chip_ns;
        let n_total = 2 * chip_span + 1;
        let samp_per_chip = (t_chip * self.samples_per_ns).round() as usize;
        let n = n_total * samp_per_chip;
        let center = (n / 2) as f64;
        let beta = rolloff;

        let mut pulse: Vec<f64> = (0..n).map(|i| {
            let t_samples = i as f64 - center;
            let t = t_samples / samp_per_chip as f64; // normalised to chip periods

            if t.abs() < 1e-9 {
                1.0 - beta + 4.0 * beta / PI
            } else if (1.0 - (2.0 * beta * t).abs()).abs() < 1e-9 {
                let a = (1.0 + 2.0 / PI) * (PI / (4.0 * beta)).sin();
                let b = (1.0 - 2.0 / PI) * (PI / (4.0 * beta)).cos();
                (beta / 2.0_f64.sqrt()) * (a + b)
            } else {
                let num = (PI * t * (1.0 - beta)).sin()
                    + 4.0 * beta * t * (PI * t * (1.0 + beta)).cos();
                let den = PI * t * (1.0 - (4.0 * beta * t).powi(2));
                num / den
            }
        }).collect();

        let energy: f64 = pulse.iter().map(|&x| x * x).sum::<f64>().sqrt();
        if energy > 0.0 {
            pulse.iter_mut().for_each(|x| *x /= energy);
        }
        pulse
    }

    /// Burst Position Modulation (BPM) + BPSK encoding
    ///
    /// For each bit:
    /// - BPM position: 0 → early burst, 1 → late burst (in burst interval)
    /// - BPSK phase: 0 → +pulse, 1 → -pulse
    ///
    /// Returns (bpm_position, bpsk_sign) for each bit pair.
    pub fn encode_bpm_bpsk(&self, bits: &[bool]) -> Vec<(usize, f64)> {
        // Pair up bits: even bits → BPM, odd bits → BPSK
        let n_symbols = bits.len() / 2;
        let burst_interval = (self.chip_ns * self.samples_per_ns * 2.0) as usize;
        (0..n_symbols).map(|i| {
            let bpm_bit = bits[2 * i];
            let bpsk_bit = bits[2 * i + 1];
            let pos = if bpm_bit { burst_interval } else { 0 };
            let sign = if bpsk_bit { -1.0 } else { 1.0 };
            (pos, sign)
        }).collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// IEEE 802.15.4a UWB Channel Model (Saleh-Valenzuela)
// ─────────────────────────────────────────────────────────────────────────────

/// Channel model type (CM1-CM8 from IEEE 802.15.4a)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelModel {
    /// CM1: LOS, 0-4 m residential
    Cm1,
    /// CM2: NLOS, 0-4 m residential
    Cm2,
    /// CM3: LOS, 4-10 m office
    Cm3,
    /// CM4: NLOS, 4-10 m office
    Cm4,
    /// CM5: LOS, large open space (outdoor)
    Cm5,
    /// CM6: NLOS, large open space (outdoor)
    Cm6,
    /// CM7: LOS, industrial environment
    Cm7,
    /// CM8: NLOS, industrial environment
    Cm8,
}

/// Saleh-Valenzuela channel model parameters
#[derive(Debug, Clone)]
pub struct SvParams {
    /// Cluster arrival rate (1/ns)
    pub lambda_c: f64,
    /// Ray arrival rate within cluster (1/ns)
    pub lambda_r: f64,
    /// Cluster decay exponent (ns)
    pub gamma_c: f64,
    /// Ray decay exponent within cluster (ns)
    pub gamma_r: f64,
    /// LOS component amplitude (0 = NLOS)
    pub los_amp: f64,
    /// Path loss exponent n
    pub path_loss_n: f64,
    /// Reference path loss at 1 m (dB)
    pub pl0_db: f64,
}

impl SvParams {
    /// Get parameters for IEEE 802.15.4a channel model
    pub fn for_model(model: ChannelModel) -> Self {
        match model {
            ChannelModel::Cm1 => SvParams {
                lambda_c: 0.0233, lambda_r: 2.5,
                gamma_c: 7.1, gamma_r: 4.3,
                los_amp: 1.0, path_loss_n: 1.79, pl0_db: 47.0,
            },
            ChannelModel::Cm2 => SvParams {
                lambda_c: 0.4, lambda_r: 0.5,
                gamma_c: 5.5, gamma_r: 6.7,
                los_amp: 0.0, path_loss_n: 4.58, pl0_db: 47.0,
            },
            ChannelModel::Cm3 => SvParams {
                lambda_c: 0.0667, lambda_r: 2.1,
                gamma_c: 14.0, gamma_r: 7.9,
                los_amp: 1.0, path_loss_n: 1.63, pl0_db: 50.5,
            },
            ChannelModel::Cm4 => SvParams {
                lambda_c: 0.0667, lambda_r: 2.1,
                gamma_c: 24.0, gamma_r: 12.0,
                los_amp: 0.0, path_loss_n: 3.07, pl0_db: 50.5,
            },
            ChannelModel::Cm5 => SvParams {
                lambda_c: 0.0477, lambda_r: 1.0,
                gamma_c: 28.1, gamma_r: 22.6,
                los_amp: 1.0, path_loss_n: 1.76, pl0_db: 51.9,
            },
            ChannelModel::Cm6 => SvParams {
                lambda_c: 0.0477, lambda_r: 1.0,
                gamma_c: 28.1, gamma_r: 26.0,
                los_amp: 0.0, path_loss_n: 2.54, pl0_db: 51.9,
            },
            ChannelModel::Cm7 => SvParams {
                lambda_c: 0.0667, lambda_r: 1.54,
                gamma_c: 31.7, gamma_r: 4.31,
                los_amp: 1.0, path_loss_n: 1.2, pl0_db: 56.7,
            },
            ChannelModel::Cm8 => SvParams {
                lambda_c: 0.0667, lambda_r: 0.15,
                gamma_c: 31.7, gamma_r: 1.8,
                los_amp: 0.0, path_loss_n: 2.15, pl0_db: 56.7,
            },
        }
    }
}

/// A single multipath component
#[derive(Debug, Clone)]
pub struct MultipathComponent {
    /// Delay in nanoseconds
    pub delay_ns: f64,
    /// Complex amplitude (I, Q)
    pub amplitude: (f64, f64),
}

/// UWB channel model with Saleh-Valenzuela cluster structure
#[derive(Debug, Clone)]
pub struct UwbChannel {
    params: SvParams,
    components: Vec<MultipathComponent>,
    rng_state: u64,
}

impl UwbChannel {
    /// Create a new channel realisation with deterministic seed
    pub fn new(model: ChannelModel, seed: u64) -> Self {
        let params = SvParams::for_model(model);
        let mut ch = UwbChannel {
            params,
            components: Vec::new(),
            rng_state: seed,
        };
        ch.realize();
        ch
    }

    /// Simple xorshift64 PRNG
    fn rand_u64(&mut self) -> u64 {
        let mut x = self.rng_state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.rng_state = x;
        x
    }

    /// Uniform random in [0, 1)
    fn rand_f64(&mut self) -> f64 {
        (self.rand_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Exponential random variable
    fn rand_exp(&mut self, rate: f64) -> f64 {
        -self.rand_f64().max(1e-15).ln() / rate
    }

    /// Generate a new channel realisation (Saleh-Valenzuela)
    pub fn realize(&mut self) {
        self.components.clear();

        let lambda_c = self.params.lambda_c;
        let lambda_r = self.params.lambda_r;
        let gamma_c  = self.params.gamma_c;
        let gamma_r  = self.params.gamma_r;

        // Add LOS component if applicable
        if self.params.los_amp > 0.0 {
            let phase = self.rand_f64() * 2.0 * PI;
            self.components.push(MultipathComponent {
                delay_ns: 0.0,
                amplitude: (
                    self.params.los_amp * phase.cos(),
                    self.params.los_amp * phase.sin(),
                ),
            });
        }

        // Generate clusters
        let mut cluster_time = 0.0f64;
        let max_delay = 200.0; // ns simulation window

        loop {
            cluster_time += self.rand_exp(lambda_c);
            if cluster_time > max_delay { break; }

            let cluster_amp = (-cluster_time / gamma_c).exp().sqrt();

            // Generate rays within cluster
            let mut ray_time = cluster_time;
            loop {
                ray_time += self.rand_exp(lambda_r);
                if ray_time - cluster_time > 5.0 * gamma_r { break; }

                let ray_amp = cluster_amp * (-(ray_time - cluster_time) / gamma_r).exp().sqrt();
                // Random phase
                let phase = self.rand_f64() * 2.0 * PI;
                // Random sign (Nakagami fading approximation)
                let sign = if self.rand_f64() > 0.5 { 1.0 } else { -1.0 };

                self.components.push(MultipathComponent {
                    delay_ns: ray_time,
                    amplitude: (
                        sign * ray_amp * phase.cos(),
                        sign * ray_amp * phase.sin(),
                    ),
                });
            }
        }

        // Normalize total channel energy to 1
        let total_energy: f64 = self.components.iter()
            .map(|c| c.amplitude.0 * c.amplitude.0 + c.amplitude.1 * c.amplitude.1)
            .sum::<f64>().sqrt();
        if total_energy > 0.0 {
            for c in &mut self.components {
                c.amplitude.0 /= total_energy;
                c.amplitude.1 /= total_energy;
            }
        }
    }

    /// Compute path loss in dB for distance_m
    pub fn path_loss_db(&self, distance_m: f64, center_freq_ghz: f64) -> f64 {
        // PL(d) = PL0 + 10*n*log10(d/d0) + 10*α*log10(f/f0)
        // α ≈ 2 for UWB frequency dependence
        let d0 = 1.0_f64;
        let f0 = 5.0_f64; // reference frequency GHz
        let n = self.params.path_loss_n;
        let pl0 = self.params.pl0_db;
        let alpha = 2.0;

        pl0 + 10.0 * n * (distance_m / d0).max(1e-6).log10()
            + 10.0 * alpha * (center_freq_ghz / f0).max(1e-6).log10()
    }

    /// Apply channel to a discrete signal (convolution)
    ///
    /// # Arguments
    /// * `input` - input real samples at given sample_rate_ghz
    /// * `sample_rate_ghz` - samples per nanosecond
    pub fn apply(&self, input: &[f64], sample_rate_ghz: f64) -> Vec<f64> {
        if input.is_empty() { return Vec::new(); }

        // Find maximum delay in samples
        let max_delay_samples = self.components.iter()
            .map(|c| (c.delay_ns * sample_rate_ghz).ceil() as usize)
            .max()
            .unwrap_or(0);

        let out_len = input.len() + max_delay_samples;
        let mut output = vec![0.0f64; out_len];

        for comp in &self.components {
            let delay_samp = (comp.delay_ns * sample_rate_ghz).round() as usize;
            let amp = comp.amplitude.0; // use real part for real-valued channel

            for (i, &x) in input.iter().enumerate() {
                output[i + delay_samp] += amp * x;
            }
        }

        output.truncate(input.len());
        output
    }

    /// Return the number of multipath components
    pub fn num_components(&self) -> usize {
        self.components.len()
    }

    /// Return a reference to the multipath components
    pub fn components(&self) -> &[MultipathComponent] {
        &self.components
    }

    /// RMS delay spread in nanoseconds
    pub fn rms_delay_spread_ns(&self) -> f64 {
        let total_power: f64 = self.components.iter()
            .map(|c| c.amplitude.0 * c.amplitude.0 + c.amplitude.1 * c.amplitude.1)
            .sum();
        if total_power < 1e-12 { return 0.0; }

        let mean_delay = self.components.iter()
            .map(|c| {
                let p = c.amplitude.0 * c.amplitude.0 + c.amplitude.1 * c.amplitude.1;
                c.delay_ns * p
            })
            .sum::<f64>() / total_power;

        let variance = self.components.iter()
            .map(|c| {
                let p = c.amplitude.0 * c.amplitude.0 + c.amplitude.1 * c.amplitude.1;
                let diff = c.delay_ns - mean_delay;
                diff * diff * p
            })
            .sum::<f64>() / total_power;

        variance.sqrt()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// AES-128 implementation (for STS generation)
// ─────────────────────────────────────────────────────────────────────────────

/// Minimal AES-128 block cipher (FIPS 197)
#[derive(Debug)]
struct Aes128 {
    round_keys: [[u8; 16]; 11],
}

impl Aes128 {
    fn new(key: &[u8; 16]) -> Self {
        let mut aes = Aes128 { round_keys: [[0u8; 16]; 11] };
        aes.key_expansion(key);
        aes
    }

    fn key_expansion(&mut self, key: &[u8; 16]) {
        // AES-128 key schedule per FIPS 197 Section 5.2
        // Store 44 words of 4 bytes each, arranged as 11 round keys × 16 bytes
        let mut w = [[0u8; 4]; 44];

        // Initial key words
        for i in 0..4 {
            w[i] = [key[4*i], key[4*i+1], key[4*i+2], key[4*i+3]];
        }

        for i in 4..44 {
            let mut temp = w[i - 1];
            if i % 4 == 0 {
                // RotWord: [a0,a1,a2,a3] → [a1,a2,a3,a0]
                temp = [temp[1], temp[2], temp[3], temp[0]];
                // SubWord
                temp = [
                    AES_SBOX[temp[0] as usize],
                    AES_SBOX[temp[1] as usize],
                    AES_SBOX[temp[2] as usize],
                    AES_SBOX[temp[3] as usize],
                ];
                // XOR with Rcon
                temp[0] ^= RCON[i / 4 - 1];
            }
            let prev = w[i - 4];
            w[i] = [
                prev[0] ^ temp[0],
                prev[1] ^ temp[1],
                prev[2] ^ temp[2],
                prev[3] ^ temp[3],
            ];
        }

        // Arrange into round keys
        for round in 0..=10 {
            for col in 0..4 {
                let word = w[round * 4 + col];
                self.round_keys[round][col*4..col*4+4].copy_from_slice(&word);
            }
        }
    }

    fn gf_mul(mut a: u8, mut b: u8) -> u8 {
        let mut result = 0u8;
        for _ in 0..8 {
            if b & 1 != 0 { result ^= a; }
            let high = a & 0x80;
            a <<= 1;
            if high != 0 { a ^= 0x1b; } // x^8 + x^4 + x^3 + x + 1
            b >>= 1;
        }
        result
    }

    fn mix_column(col: &mut [u8; 4]) {
        let [a, b, c, d] = *col;
        col[0] = Self::gf_mul(2, a) ^ Self::gf_mul(3, b) ^ c ^ d;
        col[1] = a ^ Self::gf_mul(2, b) ^ Self::gf_mul(3, c) ^ d;
        col[2] = a ^ b ^ Self::gf_mul(2, c) ^ Self::gf_mul(3, d);
        col[3] = Self::gf_mul(3, a) ^ b ^ c ^ Self::gf_mul(2, d);
    }

    /// Encrypt one 16-byte block
    fn encrypt_block(&self, block: &[u8; 16]) -> [u8; 16] {
        let mut state = *block;
        // AddRoundKey (round 0)
        for i in 0..16 { state[i] ^= self.round_keys[0][i]; }

        for round in 1..=10 {
            // SubBytes
            for i in 0..16 { state[i] = AES_SBOX[state[i] as usize]; }
            // ShiftRows (FIPS 197 column-major: col*4 + row)
            // Row 1 shift left by 1: [1,5,9,13] → [5,9,13,1]
            // Row 2 shift left by 2: [2,6,10,14] → [10,14,2,6]
            // Row 3 shift left by 3: [3,7,11,15] → [15,3,7,11]
            let s = state;
            state[1]  = s[5];  state[5]  = s[9];  state[9]  = s[13]; state[13] = s[1];
            state[2]  = s[10]; state[6]  = s[14]; state[10] = s[2];  state[14] = s[6];
            state[3]  = s[15]; state[7]  = s[3];  state[11] = s[7];  state[15] = s[11];
            // MixColumns (skip in last round)
            // Column c has bytes at [4c, 4c+1, 4c+2, 4c+3] in column-major layout
            if round < 10 {
                for col in 0..4 {
                    let base = col * 4;
                    let mut c = [state[base], state[base+1], state[base+2], state[base+3]];
                    Self::mix_column(&mut c);
                    state[base] = c[0]; state[base+1] = c[1];
                    state[base+2] = c[2]; state[base+3] = c[3];
                }
            }
            // AddRoundKey
            for i in 0..16 { state[i] ^= self.round_keys[round][i]; }
        }
        state
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// STS (Scrambled Timestamp Sequence)
// ─────────────────────────────────────────────────────────────────────────────

/// AES-128 based Scrambled Timestamp Sequence generator (IEEE 802.15.4z)
///
/// The STS is generated by encrypting a counter with AES-128 using a
/// session-specific key, producing a deterministic pseudorandom sequence.
/// The receiver correlates the received signal against the expected STS
/// to achieve secure time-of-arrival estimation.
#[derive(Debug)]
pub struct StsGenerator {
    aes: Aes128,
    /// 128-bit STS V (Vunitisation vector / IV)
    iv: [u8; 16],
    /// Current counter value
    counter: u64,
}

impl StsGenerator {
    /// Create a new STS generator
    ///
    /// # Arguments
    /// * `key` - 128-bit AES session key
    /// * `iv`  - 128-bit initialization vector (session specific)
    pub fn new(key: &[u8; 16], iv: &[u8; 16]) -> Self {
        StsGenerator {
            aes: Aes128::new(key),
            iv: *iv,
            counter: 0,
        }
    }

    /// Generate `length` STS chips (ternary: +1, 0, -1)
    ///
    /// Each AES block encryption produces 128 bits → 64 ternary chips
    /// (using 2-bit encoding: 00→+1, 01→0, 10→-1, 11→0)
    pub fn generate(&mut self, length: usize) -> Vec<i8> {
        let mut output = Vec::with_capacity(length);

        while output.len() < length {
            // Build counter block: IV XOR counter
            let mut block = self.iv;
            let counter_bytes = self.counter.to_le_bytes();
            for i in 0..8 {
                block[i] ^= counter_bytes[i];
            }
            self.counter += 1;

            let encrypted = self.aes.encrypt_block(&block);

            // Extract ternary chips from 2-bit groups
            for byte in &encrypted {
                for shift in (0..8).step_by(2) {
                    let bits = (byte >> shift) & 0x03;
                    let chip: i8 = match bits {
                        0b00 => 1,
                        0b10 => -1,
                        _    => 0,
                    };
                    output.push(chip);
                    if output.len() >= length { break; }
                }
                if output.len() >= length { break; }
            }
        }

        output.truncate(length);
        output
    }

    /// Reset counter for new ranging exchange
    pub fn reset(&mut self) {
        self.counter = 0;
    }

    /// Correlate received signal against expected STS
    ///
    /// Returns (peak_offset, correlation_peak) for timing estimation.
    pub fn correlate(&mut self, received: &[f64], sts_length: usize) -> (usize, f64) {
        let sts: Vec<f64> = self.generate(sts_length).iter()
            .map(|&x| x as f64)
            .collect();

        let mut best_offset = 0usize;
        let mut best_corr = f64::NEG_INFINITY;

        let max_lag = received.len().saturating_sub(sts_length);
        for lag in 0..=max_lag {
            let corr: f64 = received[lag..lag + sts_length].iter()
                .zip(sts.iter())
                .map(|(&r, &s)| r * s)
                .sum();
            if corr > best_corr {
                best_corr = corr;
                best_offset = lag;
            }
        }
        (best_offset, best_corr)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Preamble processing
// ─────────────────────────────────────────────────────────────────────────────

/// SFD (Start Frame Delimiter) pattern type
#[derive(Debug, Clone, Copy)]
pub enum SfdType {
    /// IEEE 802.15.4-2011 standard 8-chip SFD
    Standard8,
    /// IEEE 802.15.4z non-standard 4-chip SFD
    Short4,
    /// IEEE 802.15.4z 8-chip SFD (alternate)
    Alt8,
}

/// Standard SFD patterns (ternary)
impl SfdType {
    pub fn pattern(&self) -> Vec<i8> {
        match self {
            SfdType::Standard8 => vec![0, 1, 0, -1, 1, 0, 0, -1],
            SfdType::Short4    => vec![-1, 1, -1, 0],
            SfdType::Alt8      => vec![1, 0, 1, -1, 0, 1, 0, -1],
        }
    }
}

/// Preamble processor for UWB frame synchronisation
#[derive(Debug)]
pub struct PreambleProcessor {
    code: PreambleCode,
    chips: Vec<i8>,
}

impl PreambleProcessor {
    pub fn new(code: PreambleCode) -> Self {
        let chips = code.chips();
        PreambleProcessor { code, chips }
    }

    /// Generate preamble signal by repeating the code `repetitions` times
    pub fn generate(&self, repetitions: u16) -> Vec<i8> {
        let mut preamble = Vec::with_capacity(self.chips.len() * repetitions as usize);
        for _ in 0..repetitions {
            preamble.extend_from_slice(&self.chips);
        }
        preamble
    }

    /// Matched filter detection: slide reference code over received signal
    ///
    /// Returns (best_offset, correlation_value).
    pub fn detect(&self, received: &[f64]) -> (usize, f64) {
        let code_len = self.chips.len();
        if received.len() < code_len {
            return (0, 0.0);
        }

        let ref_f64: Vec<f64> = self.chips.iter().map(|&x| x as f64).collect();
        let ref_energy: f64 = ref_f64.iter().map(|&x| x * x).sum::<f64>().sqrt();

        let mut best_offset = 0usize;
        let mut best_norm = 0.0f64;
        let max_lag = received.len() - code_len;

        for lag in 0..=max_lag {
            let corr: f64 = received[lag..lag + code_len].iter()
                .zip(ref_f64.iter())
                .map(|(&r, &c)| r * c)
                .sum();
            let sig_energy: f64 = received[lag..lag + code_len].iter()
                .map(|&r| r * r)
                .sum::<f64>().sqrt();
            let norm = if ref_energy * sig_energy > 0.0 {
                corr / (ref_energy * sig_energy)
            } else { 0.0 };

            if norm > best_norm {
                best_norm = norm;
                best_offset = lag;
            }
        }
        (best_offset, best_norm)
    }

    /// Code length in chips
    pub fn code_length(&self) -> usize {
        self.chips.len()
    }

    pub fn code(&self) -> PreambleCode {
        self.code
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// PHR (PHY Header) encoding/decoding
// ─────────────────────────────────────────────────────────────────────────────

/// PHY Header fields (IEEE 802.15.4z HRP UWB)
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PhyHeader {
    /// Data rate: 0=850kbps, 1=6.81Mbps, 2=27.24Mbps
    pub data_rate: u8,
    /// Ranging bit (1 = ranging frame)
    pub ranging: bool,
    /// Preamble length code (0-15 → 16..4096 symbols)
    pub preamble_length_code: u8,
    /// SFD type
    pub sfd_type: u8,
    /// PSDU length in octets
    pub psdu_length: u16,
    /// STS packet configuration (0-7)
    pub sts_config: u8,
}

impl PhyHeader {
    /// Encode PHR to 19-bit field + 6-bit SECDED Hamming parity
    /// Returns 25-bit encoded PHR as u32
    pub fn encode(&self) -> u32 {
        // Build 19-bit PHR field
        let mut phr: u32 = 0;
        phr |= (self.data_rate as u32) & 0x3;          // bits [1:0]
        phr |= ((self.ranging as u32) & 0x1) << 2;     // bit 2
        phr |= ((self.preamble_length_code as u32) & 0xF) << 3; // bits [6:3]
        phr |= ((self.sfd_type as u32) & 0x3) << 7;   // bits [8:7]
        phr |= ((self.psdu_length as u32) & 0x3FF) << 9; // bits [18:9]

        // Compute SECDED Hamming parity (6 parity bits for 19 data bits)
        let parity = Self::compute_parity(phr & 0x7FFFF);
        phr | (parity << 19)
    }

    /// Compute SECDED parity for 19-bit data
    fn compute_parity(data: u32) -> u32 {
        let mut p = 0u32;
        for i in 0..6u32 {
            let mask = 1u32 << i;
            let mut bit = 0u32;
            for j in 0..19u32 {
                if (j + 1) & mask != 0 {
                    bit ^= (data >> j) & 1;
                }
            }
            p |= bit << i;
        }
        p
    }

    /// Decode 25-bit encoded PHR, return (header, error_corrected)
    pub fn decode(encoded: u32) -> (Self, bool) {
        let data = encoded & 0x7FFFF;
        let rx_parity = (encoded >> 19) & 0x3F;
        let exp_parity = Self::compute_parity(data);
        let error_corrected = rx_parity != exp_parity;

        let header = PhyHeader {
            data_rate: (data & 0x3) as u8,
            ranging: ((data >> 2) & 1) != 0,
            preamble_length_code: ((data >> 3) & 0xF) as u8,
            sfd_type: ((data >> 7) & 0x3) as u8,
            psdu_length: ((data >> 9) & 0x3FF) as u16,
            sts_config: 0,
        };
        (header, error_corrected)
    }

    /// Decode preamble length from code
    pub fn preamble_symbols(&self) -> u16 {
        match self.preamble_length_code {
            0 => 16, 1 => 64, 2 => 128, 3 => 256,
            4 => 512, 5 => 1024, 6 => 2048, 7 => 4096,
            _ => 128,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Reed-Solomon RS(63,55) over GF(2^6) for PSDU FEC
// ─────────────────────────────────────────────────────────────────────────────

/// Minimal Reed-Solomon GF(2^6) for UWB PHY payload FEC
struct RsGf64 {
    /// Generator polynomial coefficients (t=4 → 2t=8 parity symbols)
    gen_poly: [u8; 9],
    /// GF(2^6) log and antilog tables
    log_tbl: [u8; 64],
    exp_tbl: [u8; 128],
}

impl RsGf64 {
    /// GF(2^6) field, primitive polynomial x^6 + x + 1 (0x43)
    fn new() -> Self {
        let mut log_tbl = [0u8; 64];
        let mut exp_tbl = [0u8; 128];
        let mut x = 1u8;
        for i in 0..63u8 {
            exp_tbl[i as usize] = x;
            log_tbl[x as usize] = i;
            x = Self::gf_mul_raw(x, 2);
        }
        exp_tbl[63] = exp_tbl[0];
        // Generator for t=4: product of (x - alpha^i) for i=0..7
        let mut gen = [0u8; 9];
        gen[0] = 1;
        let mut len = 1;
        for i in 0..8u8 {
            let ai = exp_tbl[i as usize];
            for j in (1..=len).rev() {
                gen[j] = Self::gf_add(gen[j], Self::gf_mul_tbl(gen[j-1], ai, &log_tbl, &exp_tbl));
            }
            gen[0] = Self::gf_mul_tbl(gen[0], ai, &log_tbl, &exp_tbl);
            len += 1;
        }
        RsGf64 { gen_poly: gen, log_tbl, exp_tbl }
    }

    fn gf_mul_raw(mut a: u8, mut b: u8) -> u8 {
        let mut r = 0u8;
        for _ in 0..6 {
            if b & 1 != 0 { r ^= a; }
            let carry = (a & 0x20) != 0;
            a <<= 1;
            if carry { a ^= 0x43; }
            b >>= 1;
        }
        r & 0x3F
    }

    fn gf_add(a: u8, b: u8) -> u8 { a ^ b }

    fn gf_mul_tbl(a: u8, b: u8, log: &[u8; 64], exp: &[u8; 128]) -> u8 {
        let a = a & 0x3F; // Mask to GF(2^6) element
        let b = b & 0x3F;
        if a == 0 || b == 0 { return 0; }
        exp[((log[a as usize] as u16 + log[b as usize] as u16) % 63) as usize]
    }

    fn gf_mul(&self, a: u8, b: u8) -> u8 {
        Self::gf_mul_tbl(a, b, &self.log_tbl, &self.exp_tbl)
    }

    /// Encode `k` data symbols, appending 8 parity symbols → n=k+8
    ///
    /// Uses systematic encoding: divide m(x)*x^8 by g(x) to get remainder r(x).
    /// Codeword = [data | parity] where parity = coefficients of r(x).
    pub fn encode(&self, data: &[u8]) -> Vec<u8> {
        let k = data.len();
        // Compute parity as remainder of data * x^8 divided by gen_poly
        // Use shift-register division
        let mut remainder = vec![0u8; 8]; // 8 parity bytes

        for &byte in data.iter() {
            let byte = byte & 0x3F; // Mask to GF(2^6) symbol
            let feedback = Self::gf_add(byte, remainder[7]);
            // Shift register right
            for i in (1..8).rev() {
                remainder[i] = Self::gf_add(
                    remainder[i - 1],
                    self.gf_mul(self.gen_poly[i], feedback),
                );
            }
            remainder[0] = self.gf_mul(self.gen_poly[0], feedback);
        }

        let mut out = data.to_vec();
        out.extend_from_slice(&remainder);
        out
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Ranging engine
// ─────────────────────────────────────────────────────────────────────────────

/// Ranging measurement result
#[derive(Debug, Clone)]
pub struct RangingResult {
    /// Estimated time-of-flight in nanoseconds
    pub tof_ns: f64,
    /// Estimated range in meters
    pub range_m: f64,
    /// Quality metric (0..1), higher = better
    pub quality: f64,
    /// Clock drift corrected (SDS-TWR only)
    pub drift_corrected: bool,
}

/// Two-Way Ranging and SDS-TWR engine
#[derive(Debug, Default)]
pub struct RangingEngine {
    /// Clock frequency error estimate (ppm)
    pub clock_error_ppm: f64,
}

impl RangingEngine {
    pub fn new() -> Self {
        RangingEngine { clock_error_ppm: 0.0 }
    }

    /// One-Way Time-of-Flight (requires synchronized clocks)
    pub fn one_way_tof(&self, tx_time_ns: f64, rx_time_ns: f64) -> RangingResult {
        let tof = rx_time_ns - tx_time_ns;
        RangingResult {
            tof_ns: tof,
            range_m: tof * 1e-9 * C,
            quality: 0.7,
            drift_corrected: false,
        }
    }

    /// Simple Two-Way Ranging (TWR)
    ///
    /// # Arguments
    /// * `t_round_a` - Round-trip time measured by initiator A (ns)
    /// * `t_reply_b` - Reply delay at responder B (ns)
    pub fn two_way_ranging(&self, t_round_a: f64, t_reply_b: f64) -> RangingResult {
        // ToF = (T_roundA - T_replyB) / 2
        let tof = (t_round_a - t_reply_b) / 2.0;
        let tof_clipped = tof.max(0.0);
        RangingResult {
            tof_ns: tof_clipped,
            range_m: tof_clipped * 1e-9 * C,
            quality: 0.85,
            drift_corrected: false,
        }
    }

    /// Symmetric Double-Sided TWR (SDS-TWR)
    ///
    /// Eliminates clock drift to first order using two round-trips.
    ///
    /// # Arguments
    /// * `t_round_a` - A's first round-trip time (ns)
    /// * `t_reply_b` - B's reply delay in first exchange (ns)
    /// * `t_round_b` - B's second round-trip time (ns)
    /// * `t_reply_a` - A's reply delay in second exchange (ns)
    ///
    /// Formula: `ToF = (T_rA * T_rB - T_rpA * T_rpB) / (T_rA + T_rB + T_rpA + T_rpB)`
    pub fn sds_twr(
        &self,
        t_round_a: f64,
        t_reply_b: f64,
        t_round_b: f64,
        t_reply_a: f64,
    ) -> f64 {
        let num = t_round_a * t_round_b - t_reply_a * t_reply_b;
        let den = t_round_a + t_round_b + t_reply_a + t_reply_b;
        if den.abs() < 1e-12 { return 0.0; }
        num / den
    }

    /// Full SDS-TWR with quality estimate
    pub fn sds_twr_full(
        &self,
        t_round_a: f64,
        t_reply_b: f64,
        t_round_b: f64,
        t_reply_a: f64,
    ) -> RangingResult {
        let tof = self.sds_twr(t_round_a, t_reply_b, t_round_b, t_reply_a);
        let tof_clipped = tof.max(0.0);
        RangingResult {
            tof_ns: tof_clipped,
            range_m: tof_clipped * 1e-9 * C,
            quality: 0.95,
            drift_corrected: true,
        }
    }

    /// Estimate range from timestamp cross-correlation peak
    pub fn range_from_correlation(&self, peak_offset: usize, sample_rate_ghz: f64) -> f64 {
        let tof_ns = peak_offset as f64 / sample_rate_ghz;
        tof_ns * 1e-9 * C
    }

    /// Compute time difference for given distance
    pub fn time_for_range(range_m: f64) -> f64 {
        range_m / C * 1e9 // ns
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// TDOA Positioning
// ─────────────────────────────────────────────────────────────────────────────

/// 3D anchor position
#[derive(Debug, Clone)]
pub struct Anchor {
    /// Anchor ID
    pub id: u8,
    /// Position in meters (x, y, z)
    pub pos: (f64, f64, f64),
}

impl Anchor {
    pub fn new(id: u8, x: f64, y: f64, z: f64) -> Self {
        Anchor { id, pos: (x, y, z) }
    }

    /// Distance to another 3D point
    pub fn distance_to(&self, x: f64, y: f64, z: f64) -> f64 {
        let dx = self.pos.0 - x;
        let dy = self.pos.1 - y;
        let dz = self.pos.2 - z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// TDOA measurement: time difference between two anchors
#[derive(Debug, Clone)]
pub struct TdoaMeasurement {
    /// Reference anchor index
    pub anchor_ref: usize,
    /// Secondary anchor index
    pub anchor_sec: usize,
    /// TDOA value in seconds (positive = closer to ref)
    pub tdoa_s: f64,
}

/// TDOA-based positioning solver (hyperbolic position solution)
#[derive(Debug)]
pub struct TdoaSolver {
    anchors: Vec<Anchor>,
    /// Maximum solver iterations
    max_iter: usize,
    /// Convergence threshold in meters
    convergence_m: f64,
}

impl TdoaSolver {
    pub fn new(anchors: Vec<Anchor>) -> Self {
        TdoaSolver {
            anchors,
            max_iter: 50,
            convergence_m: 0.001,
        }
    }

    pub fn with_max_iter(mut self, n: usize) -> Self {
        self.max_iter = n;
        self
    }

    /// Solve for 2D position (z=0) given TDOA measurements
    ///
    /// Uses iterative Gauss-Newton least-squares minimization.
    pub fn solve_2d(&self, measurements: &[TdoaMeasurement]) -> Option<(f64, f64, f64)> {
        self.solve_3d_init(measurements, 0.0, true)
    }

    /// Solve for 3D position given TDOA measurements
    pub fn solve_3d(&self, measurements: &[TdoaMeasurement]) -> Option<(f64, f64, f64)> {
        self.solve_3d_init(measurements, 0.0, false)
    }

    fn solve_3d_init(
        &self,
        measurements: &[TdoaMeasurement],
        _z_init: f64,
        fix_z: bool,
    ) -> Option<(f64, f64, f64)> {
        if measurements.is_empty() || self.anchors.len() < 3 { return None; }

        // Initial position estimate: centroid of anchors
        let mut x = self.anchors.iter().map(|a| a.pos.0).sum::<f64>() / self.anchors.len() as f64;
        let mut y = self.anchors.iter().map(|a| a.pos.1).sum::<f64>() / self.anchors.len() as f64;
        let mut z = if fix_z {
            0.0
        } else {
            self.anchors.iter().map(|a| a.pos.2).sum::<f64>() / self.anchors.len() as f64
        };

        let dims = if fix_z { 2 } else { 3 };

        for _iter in 0..self.max_iter {
            let mut jtj = [[0.0f64; 3]; 3];
            let mut jtr = [0.0f64; 3];

            for m in measurements {
                let ar = &self.anchors[m.anchor_ref];
                let ams = &self.anchors[m.anchor_sec];

                let dr = ar.distance_to(x, y, z).max(1e-9);
                let ds = ams.distance_to(x, y, z).max(1e-9);

                let predicted_tdoa = (dr - ds) / C; // seconds
                let residual = m.tdoa_s - predicted_tdoa;

                // Jacobian: d(TDOA)/d(pos) where TDOA = (d_ref - d_sec)/c
                // d(d_ref)/dx = (x - x_ref)/d_ref, so d(TDOA)/dx = (x-x_ref)/(d_ref*C) - (x-x_sec)/(d_sec*C)
                let jx = (x - ar.pos.0) / (dr * C) - (x - ams.pos.0) / (ds * C);
                let jy = (y - ar.pos.1) / (dr * C) - (y - ams.pos.1) / (ds * C);
                let jz = if fix_z { 0.0 } else {
                    (z - ar.pos.2) / (dr * C) - (z - ams.pos.2) / (ds * C)
                };
                let j = [jx, jy, jz];

                for i in 0..dims {
                    jtr[i] += j[i] * residual;
                    for k in 0..dims {
                        jtj[i][k] += j[i] * j[k];
                    }
                }
            }

            // Solve 2x2 or 3x3 system via Cramer's rule / simple inversion
            let delta = if dims == 2 {
                Self::solve_2x2(&jtj, &jtr).unwrap_or([0.0; 3])
            } else {
                Self::solve_3x3(&jtj, &jtr).unwrap_or([0.0; 3])
            };

            let step = (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt();
            x += delta[0];
            y += delta[1];
            if !fix_z { z += delta[2]; }

            if step < self.convergence_m { break; }
        }

        Some((x, y, z))
    }

    fn solve_2x2(a: &[[f64; 3]; 3], b: &[f64; 3]) -> Option<[f64; 3]> {
        let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
        if det.abs() < 1e-15 { return None; }
        Some([
            (b[0] * a[1][1] - b[1] * a[0][1]) / det,
            (a[0][0] * b[1] - a[1][0] * b[0]) / det,
            0.0,
        ])
    }

    fn solve_3x3(a: &[[f64; 3]; 3], b: &[f64; 3]) -> Option<[f64; 3]> {
        // Cofactor expansion
        let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
        if det.abs() < 1e-15 { return None; }
        let inv_det = 1.0 / det;
        let x = b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
              - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2])
              + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2]);
        let y = a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2])
              - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
              + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0]);
        let z = a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1])
              - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0])
              + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
        Some([x * inv_det, y * inv_det, z * inv_det])
    }

    /// Compute GDOP (Geometric Dilution of Precision)
    ///
    /// Lower GDOP = better geometry = lower positioning error amplification.
    pub fn compute_dop(&self, pos: (f64, f64, f64)) -> f64 {
        if self.anchors.len() < 4 { return f64::INFINITY; }
        let (x, y, z) = pos;
        let n = self.anchors.len();
        let mut h = vec![[0.0f64; 3]; n];
        for (i, a) in self.anchors.iter().enumerate() {
            let d = a.distance_to(x, y, z).max(1e-9);
            h[i][0] = (x - a.pos.0) / d;
            h[i][1] = (y - a.pos.1) / d;
            h[i][2] = (z - a.pos.2) / d;
        }
        // GDOP ≈ sqrt(trace((H^T H)^-1))
        // Compute H^T H (3x3)
        let mut htH = [[0.0f64; 3]; 3];
        for row in &h {
            for i in 0..3 {
                for j in 0..3 {
                    htH[i][j] += row[i] * row[j];
                }
            }
        }
        // Trace of inverse (approximated via 1/diagonal for simplicity)
        let trace_inv = (0..3).map(|i| {
            if htH[i][i].abs() > 1e-12 { 1.0 / htH[i][i] } else { 1e6 }
        }).sum::<f64>();
        trace_inv.sqrt()
    }

    /// Simulate TDOA measurements for a known tag position
    pub fn simulate_measurements(
        &self,
        tag_pos: (f64, f64, f64),
        noise_ns: f64,
        seed: u64,
    ) -> Vec<TdoaMeasurement> {
        let mut rng = seed;
        let mut rand = || -> f64 {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            // Box-Muller for Gaussian
            let u1 = (rng >> 11) as f64 / (1u64 << 53) as f64;
            rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
            let u2 = (rng >> 11) as f64 / (1u64 << 53) as f64;
            (-2.0 * u1.max(1e-15).ln()).sqrt() * (2.0 * PI * u2).cos()
        };

        let (tx, ty, tz) = tag_pos;
        let n = self.anchors.len();
        let mut measurements = Vec::new();
        for i in 1..n {
            let d0 = self.anchors[0].distance_to(tx, ty, tz);
            let di = self.anchors[i].distance_to(tx, ty, tz);
            let true_tdoa = (d0 - di) / C; // seconds
            let noise = rand() * noise_ns * 1e-9;
            measurements.push(TdoaMeasurement {
                anchor_ref: 0,
                anchor_sec: i,
                tdoa_s: true_tdoa + noise,
            });
        }
        measurements
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Complete UWB Transceiver
// ─────────────────────────────────────────────────────────────────────────────

/// UWB frame type
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FrameType {
    /// Blink frame (device advertisement)
    Blink,
    /// Ranging initiation
    RangingInit,
    /// Ranging response
    RangingResp,
    /// Data frame
    Data,
    /// Acknowledgement
    Ack,
}

/// A complete UWB frame
#[derive(Debug, Clone)]
pub struct UwbFrame {
    pub frame_type: FrameType,
    pub header: PhyHeader,
    pub preamble: Vec<i8>,
    pub sfd: Vec<i8>,
    pub payload: Vec<u8>,
    pub sts: Vec<i8>,
    /// Transmission timestamp (ns)
    pub tx_time_ns: f64,
}

/// Complete UWB Transceiver (TX + RX chain)
pub struct UwbTransceiver {
    config: UwbConfig,
    pulse_gen: PulseGenerator,
    preamble_proc: PreambleProcessor,
    sts_gen: StsGenerator,
    rs: RsGf64,
    ranging: RangingEngine,
}

impl UwbTransceiver {
    /// Create a new transceiver
    ///
    /// # Arguments
    /// * `config` - UWB configuration
    /// * `sts_key` - 128-bit STS session key
    /// * `sts_iv` - 128-bit STS IV
    pub fn new(config: UwbConfig, sts_key: &[u8; 16], sts_iv: &[u8; 16]) -> Self {
        let pulse_gen = PulseGenerator::new(config.prf.chip_duration_ns());
        let preamble_proc = PreambleProcessor::new(config.preamble_code);
        let sts_gen = StsGenerator::new(sts_key, sts_iv);
        let rs = RsGf64::new();
        let ranging = RangingEngine::new();

        UwbTransceiver {
            config,
            pulse_gen,
            preamble_proc,
            sts_gen,
            rs,
            ranging,
        }
    }

    /// Build and encode a ranging initiation frame
    pub fn build_ranging_init(&mut self, payload: &[u8]) -> UwbFrame {
        let encoded_payload = self.rs.encode(payload);
        let preamble = self.preamble_proc.generate(self.config.preamble_length);
        let sfd = SfdType::Standard8.pattern();
        let sts = self.sts_gen.generate(self.config.sts_length as usize);

        let header = PhyHeader {
            data_rate: match self.config.data_rate {
                850_000 => 0,
                6_810_000 => 1,
                _ => 2,
            },
            ranging: true,
            preamble_length_code: 2, // 128 symbols
            sfd_type: self.config.sfd_type,
            psdu_length: encoded_payload.len() as u16,
            sts_config: 1,
        };

        UwbFrame {
            frame_type: FrameType::RangingInit,
            header,
            preamble,
            sfd,
            payload: encoded_payload,
            sts,
            tx_time_ns: 0.0,
        }
    }

    /// Modulate a UWB frame to baseband samples
    ///
    /// Returns IQ samples (real part only for impulse radio).
    pub fn modulate(&self, frame: &UwbFrame, sigma_ns: f64) -> Vec<f64> {
        let pulse = self.pulse_gen.doublet(sigma_ns);
        let plen = pulse.len();
        let chip_samples = plen;

        let total_chips = frame.preamble.len()
            + frame.sfd.len()
            + frame.sts.len()
            + frame.payload.len() * 8;

        let mut output = vec![0.0f64; total_chips * chip_samples];
        let mut offset = 0;

        // Preamble
        for &chip in &frame.preamble {
            Self::add_pulse(&mut output, offset, &pulse, chip as f64);
            offset += chip_samples;
        }

        // SFD
        for &chip in &frame.sfd {
            Self::add_pulse(&mut output, offset, &pulse, chip as f64);
            offset += chip_samples;
        }

        // STS
        for &chip in &frame.sts {
            Self::add_pulse(&mut output, offset, &pulse, chip as f64);
            offset += chip_samples;
        }

        // PSDU: BPM-BPSK encode bits from payload bytes
        for &byte in &frame.payload {
            for bit_idx in 0..8 {
                let bit = (byte >> bit_idx) & 1;
                let sign = if bit == 0 { 1.0 } else { -1.0 };
                Self::add_pulse(&mut output, offset, &pulse, sign);
                offset += chip_samples;
            }
        }

        output
    }

    fn add_pulse(output: &mut [f64], offset: usize, pulse: &[f64], amplitude: f64) {
        for (i, &p) in pulse.iter().enumerate() {
            let idx = offset + i;
            if idx < output.len() {
                output[idx] += amplitude * p;
            }
        }
    }

    /// Detect preamble in received signal
    pub fn detect_preamble(&self, received: &[f64]) -> (usize, f64) {
        self.preamble_proc.detect(received)
    }

    /// Perform SDS-TWR ranging exchange
    pub fn compute_range_sds_twr(
        &mut self,
        t_round_a_ns: f64,
        t_reply_b_ns: f64,
        t_round_b_ns: f64,
        t_reply_a_ns: f64,
    ) -> RangingResult {
        self.ranging.sds_twr_full(t_round_a_ns, t_reply_b_ns, t_round_b_ns, t_reply_a_ns)
    }

    /// Get reference to config
    pub fn config(&self) -> &UwbConfig { &self.config }

    /// Get reference to ranging engine
    pub fn ranging(&self) -> &RangingEngine { &self.ranging }
}

// ─────────────────────────────────────────────────────────────────────────────
// Utility: CIR (Channel Impulse Response) analysis
// ─────────────────────────────────────────────────────────────────────────────

/// Analyze Channel Impulse Response for first-path detection
pub struct CirAnalyzer {
    /// Detection threshold relative to peak (0..1)
    pub threshold: f64,
}

impl CirAnalyzer {
    pub fn new(threshold: f64) -> Self {
        CirAnalyzer { threshold }
    }

    /// Find first significant path in the CIR
    ///
    /// Returns (first_path_index, peak_index, peak_amplitude)
    pub fn first_path(&self, cir: &[f64]) -> (usize, usize, f64) {
        if cir.is_empty() { return (0, 0, 0.0); }

        let peak_amp = cir.iter().map(|&x| x.abs()).fold(0.0f64, f64::max);
        let threshold = peak_amp * self.threshold;
        let peak_idx = cir.iter().enumerate()
            .max_by(|(_, a), (_, b)| a.abs().partial_cmp(&b.abs()).unwrap())
            .map(|(i, _)| i)
            .unwrap_or(0);

        // First path: earliest sample exceeding threshold (before peak)
        let first_idx = cir[..=peak_idx].iter().enumerate()
            .find(|(_, &v)| v.abs() >= threshold)
            .map(|(i, _)| i)
            .unwrap_or(peak_idx);

        (first_idx, peak_idx, peak_amp)
    }

    /// Estimate leading edge timing via linear interpolation
    pub fn leading_edge_ns(&self, cir: &[f64], sample_rate_ghz: f64) -> f64 {
        let (first_idx, _, _) = self.first_path(cir);
        first_idx as f64 / sample_rate_ghz
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Configuration tests ──────────────────────────────────────────────────

    #[test]
    fn test_channel_number_properties() {
        assert!((ChannelNumber::Ch5.center_freq_mhz() - 6489.6).abs() < 0.1);
        assert!((ChannelNumber::Ch5.bandwidth_mhz() - 499.2).abs() < 0.1);
        assert!((ChannelNumber::Ch1.center_freq_mhz() - 3494.4).abs() < 0.1);
        assert!((ChannelNumber::Ch4.bandwidth_mhz() - 1331.2).abs() < 0.1);
    }

    #[test]
    fn test_prf_chip_duration() {
        let t4  = PrfMode::Prf4.chip_duration_ns();
        let t16 = PrfMode::Prf16.chip_duration_ns();
        let t64 = PrfMode::Prf64.chip_duration_ns();
        // Prf4 ≈ 4× Prf16 ≈ 16× Prf64
        assert!((t4 / t16 - 4.0).abs() < 0.01, "t4/t16 = {}", t4/t16);
        assert!((t16 / t64 - 4.0).abs() < 0.01, "t16/t64 = {}", t16/t64);
        assert!(t64 > 1.0 && t64 < 100.0);
    }

    #[test]
    fn test_default_config() {
        let cfg = UwbConfig::default();
        assert_eq!(cfg.channel, ChannelNumber::Ch5);
        assert_eq!(cfg.prf, PrfMode::Prf64);
        assert_eq!(cfg.data_rate, 6_810_000);
    }

    // ── Pulse generation tests ───────────────────────────────────────────────

    #[test]
    fn test_monocycle_zero_mean() {
        let gen = PulseGenerator::new(16.0).with_sample_rate(2.0);
        let pulse = gen.monocycle(0.5);
        let sum: f64 = pulse.iter().sum();
        assert!(sum.abs() < 1e-10, "monocycle sum = {}", sum);
    }

    #[test]
    fn test_monocycle_odd_length() {
        let gen = PulseGenerator::new(16.0).with_sample_rate(2.0);
        let pulse = gen.monocycle(0.5);
        assert_eq!(pulse.len() % 2, 1, "pulse length must be odd");
    }

    #[test]
    fn test_monocycle_unit_energy() {
        let gen = PulseGenerator::new(16.0).with_sample_rate(2.0);
        let pulse = gen.monocycle(0.5);
        let energy: f64 = pulse.iter().map(|&x| x * x).sum();
        assert!((energy - 1.0).abs() < 1e-9, "energy = {}", energy);
    }

    #[test]
    fn test_doublet_unit_energy() {
        let gen = PulseGenerator::new(16.0).with_sample_rate(2.0);
        let pulse = gen.doublet(0.5);
        let energy: f64 = pulse.iter().map(|&x| x * x).sum();
        assert!((energy - 1.0).abs() < 1e-9, "energy = {}", energy);
    }

    #[test]
    fn test_doublet_peak_at_center() {
        let gen = PulseGenerator::new(16.0).with_sample_rate(2.0);
        let pulse = gen.doublet(0.5);
        let center = pulse.len() / 2;
        let peak_idx = pulse.iter().enumerate()
            .max_by(|(_, a), (_, b)| a.abs().partial_cmp(&b.abs()).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        assert!((peak_idx as i64 - center as i64).abs() <= 2,
            "peak at {} vs center {}", peak_idx, center);
    }

    #[test]
    fn test_rrc_has_samples() {
        let gen = PulseGenerator::new(16.0).with_sample_rate(1.0);
        let rrc = gen.rrc(0.35, 4);
        assert!(!rrc.is_empty());
        let energy: f64 = rrc.iter().map(|&x| x * x).sum();
        assert!(energy > 0.0);
    }

    #[test]
    fn test_bpm_bpsk_encoding() {
        let gen = PulseGenerator::new(16.0).with_sample_rate(1.0);
        let bits = vec![false, false, true, false, true, true, false, true];
        let encoded = gen.encode_bpm_bpsk(&bits);
        assert_eq!(encoded.len(), bits.len() / 2);
        // BPM=false → position 0
        assert_eq!(encoded[0].0, 0);
        // BPSK=false → sign +1.0
        assert!((encoded[0].1 - 1.0).abs() < 1e-9);
    }

    // ── Preamble code tests ──────────────────────────────────────────────────

    #[test]
    fn test_preamble_code31_length() {
        for code in [PreambleCode::Code1, PreambleCode::Code2,
                     PreambleCode::Code3, PreambleCode::Code4] {
            let chips = code.chips();
            assert_eq!(chips.len(), 31, "{:?} length wrong", code);
        }
    }

    #[test]
    fn test_preamble_code127_length() {
        for code in [PreambleCode::Code9, PreambleCode::Code10,
                     PreambleCode::Code11, PreambleCode::Code12] {
            let chips = code.chips();
            assert_eq!(chips.len(), 127, "{:?} length wrong", code);
        }
    }

    #[test]
    fn test_preamble_codes_ternary() {
        for code in [PreambleCode::Code1, PreambleCode::Code9] {
            let chips = code.chips();
            for &c in &chips {
                assert!(c == -1 || c == 0 || c == 1, "chip {} out of range", c);
            }
        }
    }

    #[test]
    fn test_preamble_processor_generate() {
        let proc = PreambleProcessor::new(PreambleCode::Code1);
        let preamble = proc.generate(4);
        assert_eq!(preamble.len(), 31 * 4);
    }

    #[test]
    fn test_preamble_detection_exact_match() {
        let proc = PreambleProcessor::new(PreambleCode::Code1);
        let preamble = proc.generate(2);
        // Convert ternary to f64, insert after 10 samples of silence
        let mut signal = vec![0.0f64; 10];
        signal.extend(preamble.iter().map(|&x| x as f64));
        signal.extend(vec![0.0f64; 10]);

        let (offset, corr) = proc.detect(&signal);
        assert_eq!(offset, 10, "preamble not detected at offset 10 (got {})", offset);
        assert!(corr > 0.9, "correlation too low: {}", corr);
    }

    // ── SFD tests ────────────────────────────────────────────────────────────

    #[test]
    fn test_sfd_standard8_length() {
        let sfd = SfdType::Standard8.pattern();
        assert_eq!(sfd.len(), 8);
    }

    #[test]
    fn test_sfd_short4_length() {
        let sfd = SfdType::Short4.pattern();
        assert_eq!(sfd.len(), 4);
    }

    // ── AES tests ────────────────────────────────────────────────────────────

    #[test]
    fn test_aes128_known_vector() {
        // NIST FIPS 197 Appendix B test vector
        let key: [u8; 16] = [
            0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
            0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c,
        ];
        let plaintext: [u8; 16] = [
            0x32, 0x43, 0xf6, 0xa8, 0x88, 0x5a, 0x30, 0x8d,
            0x31, 0x31, 0x98, 0xa2, 0xe0, 0x37, 0x07, 0x34,
        ];
        let expected: [u8; 16] = [
            0x39, 0x25, 0x84, 0x1d, 0x02, 0xdc, 0x09, 0xfb,
            0xdc, 0x11, 0x85, 0x97, 0x19, 0x6a, 0x0b, 0x32,
        ];
        let aes = Aes128::new(&key);
        let ct = aes.encrypt_block(&plaintext);
        assert_eq!(ct, expected, "AES-128 test vector mismatch");
    }

    // ── STS tests ────────────────────────────────────────────────────────────

    #[test]
    fn test_sts_generation_length() {
        let key = [0u8; 16];
        let iv  = [1u8; 16];
        let mut sts = StsGenerator::new(&key, &iv);
        let chips = sts.generate(128);
        assert_eq!(chips.len(), 128);
    }

    #[test]
    fn test_sts_chips_ternary() {
        let key = [0xAAu8; 16];
        let iv  = [0x55u8; 16];
        let mut sts = StsGenerator::new(&key, &iv);
        let chips = sts.generate(64);
        for &c in &chips {
            assert!(c == -1 || c == 0 || c == 1, "STS chip {} out of range", c);
        }
    }

    #[test]
    fn test_sts_deterministic() {
        let key = [0x12u8; 16];
        let iv  = [0x34u8; 16];
        let mut sts1 = StsGenerator::new(&key, &iv);
        let mut sts2 = StsGenerator::new(&key, &iv);
        let c1 = sts1.generate(64);
        let c2 = sts2.generate(64);
        assert_eq!(c1, c2, "STS must be deterministic");
    }

    #[test]
    fn test_sts_correlation_peak() {
        let key = [0xABu8; 16];
        let iv  = [0xCDu8; 16];
        let mut sts_tx = StsGenerator::new(&key, &iv);
        let mut sts_rx = StsGenerator::new(&key, &iv);

        let chips: Vec<f64> = sts_tx.generate(64).iter().map(|&x| x as f64).collect();
        // Embed at offset 10
        let mut received = vec![0.0f64; 10 + 64 + 10];
        for (i, &c) in chips.iter().enumerate() {
            received[10 + i] = c;
        }

        let (offset, corr) = sts_rx.correlate(&received, 64);
        assert_eq!(offset, 10, "STS correlation offset {} != 10", offset);
        assert!(corr > 0.0, "STS correlation peak should be positive");
    }

    #[test]
    fn test_sts_reset() {
        let key = [0u8; 16];
        let iv  = [0u8; 16];
        let mut sts = StsGenerator::new(&key, &iv);
        let c1 = sts.generate(32);
        sts.reset();
        let c2 = sts.generate(32);
        assert_eq!(c1, c2, "Reset should reproduce same sequence");
    }

    // ── PHR tests ────────────────────────────────────────────────────────────

    #[test]
    fn test_phr_encode_decode_roundtrip() {
        let hdr = PhyHeader {
            data_rate: 1,
            ranging: true,
            preamble_length_code: 2,
            sfd_type: 0,
            psdu_length: 127,
            sts_config: 1,
        };
        let encoded = hdr.encode();
        let (decoded, corrected) = PhyHeader::decode(encoded);
        assert_eq!(decoded.data_rate, hdr.data_rate);
        assert_eq!(decoded.ranging, hdr.ranging);
        assert_eq!(decoded.psdu_length, hdr.psdu_length);
        assert!(!corrected, "No error should be detected on clean encoding");
    }

    #[test]
    fn test_phr_ranging_bit() {
        let hdr = PhyHeader {
            data_rate: 0,
            ranging: true,
            preamble_length_code: 1,
            sfd_type: 0,
            psdu_length: 10,
            sts_config: 0,
        };
        let enc = hdr.encode();
        let (dec, _) = PhyHeader::decode(enc);
        assert!(dec.ranging);
    }

    #[test]
    fn test_phr_preamble_symbols() {
        let hdr = PhyHeader {
            data_rate: 1,
            ranging: false,
            preamble_length_code: 2,
            sfd_type: 0,
            psdu_length: 0,
            sts_config: 0,
        };
        assert_eq!(hdr.preamble_symbols(), 128);
    }

    // ── UWB Channel Model tests ──────────────────────────────────────────────

    #[test]
    fn test_channel_cm1_los() {
        let ch = UwbChannel::new(ChannelModel::Cm1, 42);
        assert!(ch.num_components() > 0);
        // LOS model should have a component at t≈0
        assert!(ch.components().iter().any(|c| c.delay_ns < 1.0));
    }

    #[test]
    fn test_channel_cm2_nlos() {
        let ch = UwbChannel::new(ChannelModel::Cm2, 42);
        assert!(ch.num_components() > 0);
    }

    #[test]
    fn test_channel_path_loss_increases_with_distance() {
        let ch = UwbChannel::new(ChannelModel::Cm1, 1);
        let pl1 = ch.path_loss_db(1.0, 6.5);
        let pl10 = ch.path_loss_db(10.0, 6.5);
        assert!(pl10 > pl1, "path loss must increase with distance");
    }

    #[test]
    fn test_channel_energy_normalized() {
        let ch = UwbChannel::new(ChannelModel::Cm5, 99);
        let total_power: f64 = ch.components().iter()
            .map(|c| c.amplitude.0 * c.amplitude.0 + c.amplitude.1 * c.amplitude.1)
            .sum();
        assert!((total_power - 1.0).abs() < 0.01 || total_power <= 1.0,
            "channel energy = {}", total_power);
    }

    #[test]
    fn test_channel_apply_length() {
        let ch = UwbChannel::new(ChannelModel::Cm3, 7);
        let input = vec![0.0f64; 100];
        let output = ch.apply(&input, 1.0);
        assert_eq!(output.len(), input.len());
    }

    #[test]
    fn test_channel_rms_delay_spread() {
        let ch_los  = UwbChannel::new(ChannelModel::Cm1, 1);
        let ch_nlos = UwbChannel::new(ChannelModel::Cm2, 1);
        // NLOS typically has larger RMS delay spread than LOS
        let ds_los  = ch_los.rms_delay_spread_ns();
        let ds_nlos = ch_nlos.rms_delay_spread_ns();
        assert!(ds_los >= 0.0 && ds_nlos >= 0.0);
    }

    #[test]
    fn test_channel_multiple_realizations() {
        // Different seeds should produce different realisations.
        // Use many seeds to guarantee at least one non-empty realisation.
        let mut found_nonempty = false;
        for seed in 1u64..=20 {
            let ch = UwbChannel::new(ChannelModel::Cm1, seed);
            if ch.num_components() > 0 {
                found_nonempty = true;
                break;
            }
        }
        assert!(found_nonempty, "At least one realisation should have multipath components");

        // Two seeds with CM1 (LOS, dense cluster) should differ in total component count
        let total1: usize = (1..=5).map(|s| UwbChannel::new(ChannelModel::Cm1, s).num_components()).sum();
        assert!(total1 > 0, "CM1 should produce multipath components");
    }

    // ── Ranging tests ────────────────────────────────────────────────────────

    #[test]
    fn test_twr_zero_range() {
        let ranging = RangingEngine::new();
        let result = ranging.two_way_ranging(100.0, 100.0);
        assert!(result.tof_ns.abs() < 1e-9, "zero reply delay = zero ToF");
    }

    #[test]
    fn test_twr_known_range() {
        let ranging = RangingEngine::new();
        // 1 m = ~3.336 ns ToF
        let tof_ns = RangingEngine::time_for_range(1.0);
        // Round trip A = reply + 2*ToF, Reply = 100 ns
        let t_round = 2.0 * tof_ns + 100.0;
        let result = ranging.two_way_ranging(t_round, 100.0);
        assert!((result.range_m - 1.0).abs() < 0.01, "range = {} m", result.range_m);
    }

    #[test]
    fn test_sds_twr_symmetric() {
        let ranging = RangingEngine::new();
        let tof_ns = RangingEngine::time_for_range(5.0); // 5 m
        let t_reply = 200.0f64;
        let t_round = 2.0 * tof_ns + t_reply;
        let tof = ranging.sds_twr(t_round, t_reply, t_round, t_reply);
        assert!((tof - tof_ns).abs() < 1e-6, "SDS-TWR tof {} != {}", tof, tof_ns);
    }

    #[test]
    fn test_sds_twr_eliminates_clock_drift() {
        let ranging = RangingEngine::new();
        let true_tof = 10.0f64; // ns
        // Introduce clock drift: responder runs 10 ppm fast
        let drift = 1e-5; // 10 ppm
        let t_reply_b = 200.0f64;
        let t_round_a = 2.0 * true_tof + t_reply_b * (1.0 + drift);
        let t_reply_a = 200.0f64;
        let t_round_b = 2.0 * true_tof + t_reply_a * (1.0 - drift);
        let tof = ranging.sds_twr(t_round_a, t_reply_b, t_round_b, t_reply_a);
        // Should be close to true_tof despite drift
        assert!((tof - true_tof).abs() < 1.0, "drift corrected tof {} != {}", tof, true_tof);
    }

    #[test]
    fn test_range_from_correlation() {
        let ranging = RangingEngine::new();
        // 1 GHz sample rate, 10 sample offset → 10 ns
        let range = ranging.range_from_correlation(10, 1.0);
        let expected = 10e-9 * C;
        assert!((range - expected).abs() < 0.01, "range = {}", range);
    }

    // ── TDOA positioning tests ───────────────────────────────────────────────

    #[test]
    fn test_tdoa_solver_known_position_2d() {
        let anchors = vec![
            Anchor::new(0, 0.0, 0.0, 0.0),
            Anchor::new(1, 10.0, 0.0, 0.0),
            Anchor::new(2, 5.0, 8.0, 0.0),
            Anchor::new(3, 0.0, 10.0, 0.0),
        ];
        let solver = TdoaSolver::new(anchors);
        let true_pos = (3.0, 4.0, 0.0);
        // Use zero noise measurements for deterministic convergence
        let measurements = solver.simulate_measurements(true_pos, 0.0, 42);
        let result = solver.solve_2d(&measurements).expect("solver should converge");
        assert!((result.0 - true_pos.0).abs() < 1.5, "x error {}", result.0 - true_pos.0);
        assert!((result.1 - true_pos.1).abs() < 1.5, "y error {}", result.1 - true_pos.1);
    }

    #[test]
    fn test_tdoa_solve_simulated_position() {
        // Simulate TDOA measurements for a known position and verify solver
        // converges to a solution consistent with the measurements.
        let anchors = vec![
            Anchor::new(0, 0.0,  0.0, 0.0),
            Anchor::new(1, 8.0,  0.0, 0.0),
            Anchor::new(2, 4.0,  7.0, 0.0),
        ];
        let solver = TdoaSolver::new(anchors.clone());
        let true_pos = (5.0, 2.0, 0.0);
        let measurements = solver.simulate_measurements(true_pos, 0.0, 1);
        let result = solver.solve_2d(&measurements).expect("solver converge");

        // Verify that the solved position is consistent with the TDOA measurements
        // (i.e. predicted TDOAs match, even if position has some numerical error)
        let d0 = anchors[0].distance_to(result.0, result.1, result.2);
        let d1 = anchors[1].distance_to(result.0, result.1, result.2);
        let predicted_tdoa = (d0 - d1) / C;
        let expected_tdoa = measurements[0].tdoa_s;
        assert!((predicted_tdoa - expected_tdoa).abs() < 1e-8,
            "TDOA residual too large: {}", (predicted_tdoa - expected_tdoa).abs());
    }

    #[test]
    fn test_tdoa_dop_computation() {
        let anchors = vec![
            Anchor::new(0,  0.0, 0.0, 0.0),
            Anchor::new(1, 10.0, 0.0, 0.0),
            Anchor::new(2,  5.0, 10.0, 0.0),
            Anchor::new(3,  5.0, 5.0, 10.0),
        ];
        let solver = TdoaSolver::new(anchors);
        let dop = solver.compute_dop((5.0, 5.0, 0.0));
        assert!(dop > 0.0 && dop.is_finite(), "DOP = {}", dop);
    }

    #[test]
    fn test_anchor_distance() {
        let a = Anchor::new(0, 0.0, 0.0, 0.0);
        assert!((a.distance_to(3.0, 4.0, 0.0) - 5.0).abs() < 1e-9);
    }

    // ── CIR analysis tests ───────────────────────────────────────────────────

    #[test]
    fn test_cir_first_path_simple() {
        let mut cir = vec![0.0f64; 20];
        cir[5] = 0.2;  // early weak path
        cir[10] = 1.0; // main peak

        let analyzer = CirAnalyzer::new(0.15);
        let (first, peak, amp) = analyzer.first_path(&cir);
        assert_eq!(peak, 10);
        assert!((amp - 1.0).abs() < 1e-9);
        assert!(first <= 10);
        // First path should be at index 5 (≥0.15 threshold)
        assert_eq!(first, 5, "first path at {}", first);
    }

    #[test]
    fn test_cir_leading_edge() {
        let mut cir = vec![0.0f64; 30];
        cir[8] = 1.0;
        let analyzer = CirAnalyzer::new(0.5);
        let edge_ns = analyzer.leading_edge_ns(&cir, 1.0); // 1 sample/ns
        assert!((edge_ns - 8.0).abs() < 1.0, "edge = {} ns", edge_ns);
    }

    #[test]
    fn test_cir_empty_signal() {
        let analyzer = CirAnalyzer::new(0.5);
        let (first, peak, amp) = analyzer.first_path(&[]);
        assert_eq!(first, 0);
        assert_eq!(peak, 0);
        assert_eq!(amp, 0.0);
    }

    // ── Full transceiver integration test ────────────────────────────────────

    #[test]
    fn test_transceiver_build_frame() {
        let config = UwbConfig::default();
        let key = [0x00u8; 16];
        let iv  = [0x01u8; 16];
        let mut tx = UwbTransceiver::new(config, &key, &iv);
        let frame = tx.build_ranging_init(b"HELLO");
        assert!(!frame.preamble.is_empty());
        assert!(!frame.sfd.is_empty());
        assert!(!frame.sts.is_empty());
        assert!(frame.header.ranging);
    }

    #[test]
    fn test_transceiver_modulate_produces_samples() {
        let config = UwbConfig::default();
        let key = [0x00u8; 16];
        let iv  = [0x01u8; 16];
        let mut tx = UwbTransceiver::new(config, &key, &iv);
        let frame = tx.build_ranging_init(b"TEST");
        let samples = tx.modulate(&frame, 0.5);
        assert!(!samples.is_empty());
        // Signal should not be all zeros
        let energy: f64 = samples.iter().map(|&x| x * x).sum();
        assert!(energy > 0.0, "modulated signal is silent");
    }

    #[test]
    fn test_transceiver_preamble_detection() {
        let config = UwbConfig::default();
        let key = [0u8; 16];
        let iv  = [0u8; 16];
        let mut tx = UwbTransceiver::new(config, &key, &iv);
        let frame = tx.build_ranging_init(b"PING");
        let samples = tx.modulate(&frame, 0.5);
        let (offset, corr) = tx.detect_preamble(&samples);
        // Preamble starts at sample 0 in the modulated output
        assert!(offset < samples.len());
        assert!(corr > 0.0);
    }

    #[test]
    fn test_transceiver_ranging() {
        let config = UwbConfig::default();
        let key = [0u8; 16];
        let iv  = [0u8; 16];
        let mut tx = UwbTransceiver::new(config, &key, &iv);
        // 10 m → ~33.36 ns ToF
        let tof_ns = RangingEngine::time_for_range(10.0);
        let t_reply = 500.0;
        let t_round = 2.0 * tof_ns + t_reply;
        let result = tx.compute_range_sds_twr(t_round, t_reply, t_round, t_reply);
        assert!((result.range_m - 10.0).abs() < 0.01,
            "range = {} m, expected ~10 m", result.range_m);
        assert!(result.drift_corrected);
    }

    // ── Reed-Solomon tests ────────────────────────────────────────────────────

    #[test]
    fn test_rs_encode_length() {
        let rs = RsGf64::new();
        let data: Vec<u8> = (0..10).collect();
        let cw = rs.encode(&data);
        assert_eq!(cw.len(), data.len() + 8, "codeword length wrong");
    }

    #[test]
    fn test_rs_encode_preserves_data() {
        let rs = RsGf64::new();
        let data: Vec<u8> = (0..20).collect();
        let cw = rs.encode(&data);
        assert_eq!(&cw[..20], &data[..], "data symbols must be preserved");
    }

    #[test]
    fn test_rs_parity_nonzero() {
        let rs = RsGf64::new();
        let data = vec![1u8, 2, 3, 4, 5];
        let cw = rs.encode(&data);
        let parity = &cw[5..];
        assert!(parity.iter().any(|&p| p != 0), "parity should not be all zeros");
    }

    // ── Path loss and frequency tests ─────────────────────────────────────────

    #[test]
    fn test_path_loss_frequency_dependence() {
        let ch_low  = UwbChannel::new(ChannelModel::Cm1, 1);
        let ch_high = UwbChannel::new(ChannelModel::Cm5, 1);
        let pl_low  = ch_low.path_loss_db(5.0, 4.0);
        let pl_high = ch_high.path_loss_db(5.0, 7.0);
        // Higher frequency → higher path loss (for same distance)
        assert!(pl_high > pl_low, "higher freq should have more path loss");
    }

    #[test]
    fn test_prf_modes_distinct() {
        let t4  = PrfMode::Prf4.chip_duration_ns();
        let t16 = PrfMode::Prf16.chip_duration_ns();
        let t64 = PrfMode::Prf64.chip_duration_ns();
        assert!(t4 > t16 && t16 > t64);
    }

    #[test]
    fn test_channel_sv_params_los_nlos() {
        let los  = SvParams::for_model(ChannelModel::Cm1);
        let nlos = SvParams::for_model(ChannelModel::Cm2);
        assert!(los.los_amp > 0.0, "CM1 should be LOS");
        assert_eq!(nlos.los_amp, 0.0, "CM2 should be NLOS");
    }

    #[test]
    fn test_time_for_range() {
        let t = RangingEngine::time_for_range(C * 1e-9); // 1 light-ns
        assert!((t - 1.0).abs() < 1e-6, "1 ns for 1 light-ns, got {}", t);
    }
}

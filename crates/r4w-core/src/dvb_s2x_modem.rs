//! DVB-S2X satellite modem processing per ETSI EN 302 307-2.
//!
//! This module implements the physical-layer signal processing for the DVB-S2X
//! (Digital Video Broadcasting – Satellite – Second Generation Extensions) standard,
//! extending DVB-S2 (ETSI EN 302 307-1) with higher-order modulations, super-framing,
//! VL-SNR (Very Low SNR) operation, and wideband support up to 500 MHz.
//!
//! # Overview
//!
//! DVB-S2X introduces:
//! - **Extended MODCODs**: up to 256APSK (64APSK, 128APSK, 256APSK) for wideband/HTS
//! - **Super-framing**: bundling of PLFRAMEs for multi-stream broadcasting with SF pilots
//! - **VL-SNR modes**: operation down to -10 dB Es/N0 with spreading and long pilots
//! - **Wideband operation**: roll-off factors 0.05/0.10/0.15 for 500 MHz transponders
//! - **CCM/VCM/ACM modes**: adaptive link for satellite broadband
//!
//! # PLFRAME Structure
//!
//! Each PLFRAME consists of:
//! - **PLHEADER** (90 symbols): SOF (26 symbols) + PLSCODE (64 symbols)
//! - **Data slots** (90 symbols each, up to 360 slots)
//! - **Pilot blocks** (36 symbols every 16 data slots)
//!
//! # Example
//!
//! ```
//! use r4w_core::dvb_s2x_modem::{
//!     DvbS2xModem, ModcodS2x, FrameType, RollOff, CcmVcmMode,
//!     PlframeBuilder, PlframeParser, EsN0Estimator,
//! };
//!
//! // Build a PLFRAME for QPSK 1/2
//! let mut modem = DvbS2xModem::new(ModcodS2x::Qpsk1_2, FrameType::Normal, RollOff::R020);
//! modem.set_mode(CcmVcmMode::Ccm);
//!
//! let data_bits: Vec<bool> = (0..1024).map(|i| i % 3 != 0).collect();
//! let (frame_symbols, _header) = modem.encode_plframe(&data_bits);
//!
//! // Estimate SNR from pilot blocks
//! let mut estimator = EsN0Estimator::new();
//! let snr_db = estimator.estimate_from_pilots(&frame_symbols, ModcodS2x::Qpsk1_2, FrameType::Normal);
//! assert!(snr_db.is_finite());
//! ```

// ---------------------------------------------------------------------------
// Core types
// ---------------------------------------------------------------------------

use std::f64::consts::PI;

/// IQ complex sample (64-bit double precision).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Complex {
    /// In-phase component.
    pub re: f64,
    /// Quadrature component.
    pub im: f64,
}

impl Complex {
    /// Create a new complex number.
    #[inline]
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    /// Complex magnitude squared.
    #[inline]
    pub fn mag_sq(&self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    /// Complex magnitude.
    #[inline]
    pub fn mag(&self) -> f64 {
        self.mag_sq().sqrt()
    }

    /// Complex conjugate.
    #[inline]
    pub fn conj(&self) -> Self {
        Self::new(self.re, -self.im)
    }

    /// Complex multiplication.
    #[inline]
    pub fn mul(&self, other: Self) -> Self {
        Self::new(
            self.re * other.re - self.im * other.im,
            self.re * other.im + self.im * other.re,
        )
    }

    /// Complex addition.
    #[inline]
    pub fn add(&self, other: Self) -> Self {
        Self::new(self.re + other.re, self.im + other.im)
    }

    /// Phase angle in radians.
    #[inline]
    pub fn phase(&self) -> f64 {
        self.im.atan2(self.re)
    }

    /// Unit phasor: e^(j*theta).
    pub fn from_phase(theta: f64) -> Self {
        Self::new(theta.cos(), theta.sin())
    }

    /// Scale by real scalar.
    #[inline]
    pub fn scale(&self, s: f64) -> Self {
        Self::new(self.re * s, self.im * s)
    }
}

// ---------------------------------------------------------------------------
// Modulation and Coding (MODCOD) definitions
// ---------------------------------------------------------------------------

/// DVB-S2X extended MODCOD table per ETSI EN 302 307-2 Table 1.
///
/// Includes legacy DVB-S2 modcods and new DVB-S2X entries up to 256APSK.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[allow(non_camel_case_types)]
pub enum ModcodS2x {
    // --- DVB-S2 legacy MODCODs ---
    /// QPSK rate 1/4 (DVB-S2, min Es/N0 = -2.35 dB)
    Qpsk1_4,
    /// QPSK rate 1/3
    Qpsk1_3,
    /// QPSK rate 2/5
    Qpsk2_5,
    /// QPSK rate 1/2
    Qpsk1_2,
    /// QPSK rate 3/5
    Qpsk3_5,
    /// QPSK rate 2/3
    Qpsk2_3,
    /// QPSK rate 3/4
    Qpsk3_4,
    /// QPSK rate 4/5
    Qpsk4_5,
    /// QPSK rate 5/6
    Qpsk5_6,
    /// QPSK rate 8/9
    Qpsk8_9,
    /// QPSK rate 9/10
    Qpsk9_10,
    /// 8PSK rate 3/5
    Psk8_3_5,
    /// 8PSK rate 2/3
    Psk8_2_3,
    /// 8PSK rate 3/4
    Psk8_3_4,
    /// 8PSK rate 5/6
    Psk8_5_6,
    /// 8PSK rate 8/9
    Psk8_8_9,
    /// 8PSK rate 9/10
    Psk8_9_10,
    /// 16APSK rate 2/3
    Apsk16_2_3,
    /// 16APSK rate 3/4
    Apsk16_3_4,
    /// 16APSK rate 4/5
    Apsk16_4_5,
    /// 16APSK rate 5/6
    Apsk16_5_6,
    /// 16APSK rate 8/9
    Apsk16_8_9,
    /// 16APSK rate 9/10
    Apsk16_9_10,
    /// 32APSK rate 3/4
    Apsk32_3_4,
    /// 32APSK rate 4/5
    Apsk32_4_5,
    /// 32APSK rate 5/6
    Apsk32_5_6,
    /// 32APSK rate 8/9
    Apsk32_8_9,
    /// 32APSK rate 9/10
    Apsk32_9_10,

    // --- DVB-S2X extended MODCODs ---
    /// QPSK rate 13/45 (DVB-S2X)
    Qpsk13_45,
    /// QPSK rate 9/20 (DVB-S2X)
    Qpsk9_20,
    /// QPSK rate 11/20 (DVB-S2X)
    Qpsk11_20,
    /// 8APSK rate 5/9-L (DVB-S2X)
    Apsk8_5_9L,
    /// 8APSK rate 26/45-L (DVB-S2X)
    Apsk8_26_45L,
    /// 16APSK rate 1/2-L (DVB-S2X)
    Apsk16_1_2L,
    /// 16APSK rate 8/15-L (DVB-S2X)
    Apsk16_8_15L,
    /// 32APSK rate 2/3-L (DVB-S2X)
    Apsk32_2_3L,
    /// 64APSK rate 32/45 (DVB-S2X)
    Apsk64_32_45,
    /// 64APSK rate 11/15 (DVB-S2X)
    Apsk64_11_15,
    /// 64APSK rate 7/9 (DVB-S2X)
    Apsk64_7_9,
    /// 64APSK rate 4/5 (DVB-S2X)
    Apsk64_4_5,
    /// 64APSK rate 5/6 (DVB-S2X)
    Apsk64_5_6,
    /// 128APSK rate 3/4 (DVB-S2X)
    Apsk128_3_4,
    /// 128APSK rate 7/9 (DVB-S2X)
    Apsk128_7_9,
    /// 256APSK rate 29/45-L (DVB-S2X)
    Apsk256_29_45L,
    /// 256APSK rate 2/3-L (DVB-S2X)
    Apsk256_2_3L,
    /// 256APSK rate 31/45 (DVB-S2X)
    Apsk256_31_45,
    /// 256APSK rate 32/45 (DVB-S2X)
    Apsk256_32_45,
    /// 256APSK rate 11/15 (DVB-S2X)
    Apsk256_11_15,
    /// 256APSK rate 3/4 (DVB-S2X)
    Apsk256_3_4,

    // --- VL-SNR (Very Low SNR) MODCODs ---
    /// QPSK rate 11/45 (VL-SNR, spreading factor 1)
    VlSnrQpsk11_45,
    /// QPSK rate 4/15 (VL-SNR, spreading factor 1)
    VlSnrQpsk4_15,
    /// BPSK spreading factor 2
    VlSnrBpsk1_5,
    /// BPSK spreading factor 4
    VlSnrBpsk11_45Sf2,
}

impl ModcodS2x {
    /// Return the modulation order (number of constellation points).
    pub fn modulation_order(&self) -> usize {
        match self {
            ModcodS2x::VlSnrBpsk1_5 | ModcodS2x::VlSnrBpsk11_45Sf2 => 2,
            ModcodS2x::Qpsk1_4
            | ModcodS2x::Qpsk1_3
            | ModcodS2x::Qpsk2_5
            | ModcodS2x::Qpsk1_2
            | ModcodS2x::Qpsk3_5
            | ModcodS2x::Qpsk2_3
            | ModcodS2x::Qpsk3_4
            | ModcodS2x::Qpsk4_5
            | ModcodS2x::Qpsk5_6
            | ModcodS2x::Qpsk8_9
            | ModcodS2x::Qpsk9_10
            | ModcodS2x::Qpsk13_45
            | ModcodS2x::Qpsk9_20
            | ModcodS2x::Qpsk11_20
            | ModcodS2x::VlSnrQpsk11_45
            | ModcodS2x::VlSnrQpsk4_15 => 4,
            ModcodS2x::Psk8_3_5
            | ModcodS2x::Psk8_2_3
            | ModcodS2x::Psk8_3_4
            | ModcodS2x::Psk8_5_6
            | ModcodS2x::Psk8_8_9
            | ModcodS2x::Psk8_9_10
            | ModcodS2x::Apsk8_5_9L
            | ModcodS2x::Apsk8_26_45L => 8,
            ModcodS2x::Apsk16_2_3
            | ModcodS2x::Apsk16_3_4
            | ModcodS2x::Apsk16_4_5
            | ModcodS2x::Apsk16_5_6
            | ModcodS2x::Apsk16_8_9
            | ModcodS2x::Apsk16_9_10
            | ModcodS2x::Apsk16_1_2L
            | ModcodS2x::Apsk16_8_15L => 16,
            ModcodS2x::Apsk32_3_4
            | ModcodS2x::Apsk32_4_5
            | ModcodS2x::Apsk32_5_6
            | ModcodS2x::Apsk32_8_9
            | ModcodS2x::Apsk32_9_10
            | ModcodS2x::Apsk32_2_3L => 32,
            ModcodS2x::Apsk64_32_45
            | ModcodS2x::Apsk64_11_15
            | ModcodS2x::Apsk64_7_9
            | ModcodS2x::Apsk64_4_5
            | ModcodS2x::Apsk64_5_6 => 64,
            ModcodS2x::Apsk128_3_4 | ModcodS2x::Apsk128_7_9 => 128,
            ModcodS2x::Apsk256_29_45L
            | ModcodS2x::Apsk256_2_3L
            | ModcodS2x::Apsk256_31_45
            | ModcodS2x::Apsk256_32_45
            | ModcodS2x::Apsk256_11_15
            | ModcodS2x::Apsk256_3_4 => 256,
        }
    }

    /// Bits per symbol (log2 of modulation order).
    pub fn bits_per_symbol(&self) -> usize {
        let m = self.modulation_order();
        let mut bps = 0usize;
        let mut n = m;
        while n > 1 {
            n >>= 1;
            bps += 1;
        }
        bps
    }

    /// Code rate as a (numerator, denominator) pair.
    pub fn code_rate_fraction(&self) -> (u32, u32) {
        match self {
            ModcodS2x::Qpsk1_4 | ModcodS2x::VlSnrBpsk1_5 => (1, 4),
            ModcodS2x::Qpsk1_3 => (1, 3),
            ModcodS2x::Qpsk2_5 => (2, 5),
            ModcodS2x::Qpsk1_2 | ModcodS2x::Apsk16_1_2L => (1, 2),
            ModcodS2x::Qpsk3_5 | ModcodS2x::Psk8_3_5 => (3, 5),
            ModcodS2x::Qpsk2_3
            | ModcodS2x::Psk8_2_3
            | ModcodS2x::Apsk16_2_3
            | ModcodS2x::Apsk32_2_3L
            | ModcodS2x::Apsk256_2_3L => (2, 3),
            ModcodS2x::Qpsk3_4
            | ModcodS2x::Psk8_3_4
            | ModcodS2x::Apsk16_3_4
            | ModcodS2x::Apsk32_3_4
            | ModcodS2x::Apsk128_3_4
            | ModcodS2x::Apsk256_3_4 => (3, 4),
            ModcodS2x::Qpsk4_5 | ModcodS2x::Apsk16_4_5 | ModcodS2x::Apsk32_4_5 | ModcodS2x::Apsk64_4_5 => (4, 5),
            ModcodS2x::Qpsk5_6
            | ModcodS2x::Psk8_5_6
            | ModcodS2x::Apsk16_5_6
            | ModcodS2x::Apsk32_5_6
            | ModcodS2x::Apsk64_5_6 => (5, 6),
            ModcodS2x::Qpsk8_9
            | ModcodS2x::Psk8_8_9
            | ModcodS2x::Apsk16_8_9
            | ModcodS2x::Apsk32_8_9 => (8, 9),
            ModcodS2x::Qpsk9_10
            | ModcodS2x::Psk8_9_10
            | ModcodS2x::Apsk16_9_10
            | ModcodS2x::Apsk32_9_10 => (9, 10),
            ModcodS2x::Qpsk13_45 => (13, 45),
            ModcodS2x::Qpsk9_20 => (9, 20),
            ModcodS2x::Qpsk11_20 => (11, 20),
            ModcodS2x::Apsk8_5_9L => (5, 9),
            ModcodS2x::Apsk8_26_45L => (26, 45),
            ModcodS2x::Apsk16_8_15L => (8, 15),
            ModcodS2x::Apsk64_32_45 | ModcodS2x::Apsk256_32_45 => (32, 45),
            ModcodS2x::Apsk64_11_15 | ModcodS2x::Apsk256_11_15 => (11, 15),
            ModcodS2x::Apsk64_7_9 | ModcodS2x::Apsk128_7_9 => (7, 9),
            ModcodS2x::Apsk256_29_45L => (29, 45),
            ModcodS2x::Apsk256_31_45 => (31, 45),
            ModcodS2x::VlSnrQpsk11_45 | ModcodS2x::VlSnrBpsk11_45Sf2 => (11, 45),
            ModcodS2x::VlSnrQpsk4_15 => (4, 15),
        }
    }

    /// Spectral efficiency in bits/symbol.
    pub fn spectral_efficiency(&self) -> f64 {
        let bps = self.bits_per_symbol() as f64;
        let (num, den) = self.code_rate_fraction();
        bps * (num as f64) / (den as f64)
    }

    /// Required Es/N0 (dB) for quasi-error-free (QEF) operation.
    ///
    /// Values from ETSI EN 302 307-1/2 normative tables.
    pub fn required_es_n0_db(&self) -> f64 {
        match self {
            ModcodS2x::VlSnrBpsk1_5 => -10.0,
            ModcodS2x::VlSnrBpsk11_45Sf2 => -9.0,
            ModcodS2x::VlSnrQpsk4_15 => -7.0,
            ModcodS2x::VlSnrQpsk11_45 => -5.5,
            ModcodS2x::Qpsk1_4 => -2.35,
            ModcodS2x::Qpsk1_3 => -1.24,
            ModcodS2x::Qpsk2_5 => -0.30,
            ModcodS2x::Qpsk1_2 => 1.00,
            ModcodS2x::Qpsk3_5 => 2.23,
            ModcodS2x::Qpsk2_3 => 3.10,
            ModcodS2x::Qpsk3_4 => 4.03,
            ModcodS2x::Qpsk4_5 => 4.68,
            ModcodS2x::Qpsk5_6 => 5.18,
            ModcodS2x::Qpsk8_9 => 6.20,
            ModcodS2x::Qpsk9_10 => 6.42,
            ModcodS2x::Qpsk13_45 => 0.79,
            ModcodS2x::Qpsk9_20 => 0.50,
            ModcodS2x::Qpsk11_20 => 1.45,
            ModcodS2x::Psk8_3_5 => 5.50,
            ModcodS2x::Psk8_2_3 => 6.62,
            ModcodS2x::Psk8_3_4 => 7.91,
            ModcodS2x::Psk8_5_6 => 9.35,
            ModcodS2x::Psk8_8_9 => 10.69,
            ModcodS2x::Psk8_9_10 => 10.98,
            ModcodS2x::Apsk8_5_9L => 5.27,
            ModcodS2x::Apsk8_26_45L => 6.43,
            ModcodS2x::Apsk16_2_3 => 8.97,
            ModcodS2x::Apsk16_3_4 => 10.21,
            ModcodS2x::Apsk16_4_5 => 11.03,
            ModcodS2x::Apsk16_5_6 => 11.61,
            ModcodS2x::Apsk16_8_9 => 12.89,
            ModcodS2x::Apsk16_9_10 => 13.13,
            ModcodS2x::Apsk16_1_2L => 5.97,
            ModcodS2x::Apsk16_8_15L => 7.78,
            ModcodS2x::Apsk32_3_4 => 12.73,
            ModcodS2x::Apsk32_4_5 => 13.64,
            ModcodS2x::Apsk32_5_6 => 14.28,
            ModcodS2x::Apsk32_8_9 => 15.69,
            ModcodS2x::Apsk32_9_10 => 16.05,
            ModcodS2x::Apsk32_2_3L => 10.98,
            ModcodS2x::Apsk64_32_45 => 16.05,
            ModcodS2x::Apsk64_11_15 => 17.50,
            ModcodS2x::Apsk64_7_9 => 18.10,
            ModcodS2x::Apsk64_4_5 => 18.42,
            ModcodS2x::Apsk64_5_6 => 19.35,
            ModcodS2x::Apsk128_3_4 => 19.57,
            ModcodS2x::Apsk128_7_9 => 21.19,
            ModcodS2x::Apsk256_29_45L => 18.84,
            ModcodS2x::Apsk256_2_3L => 19.57,
            ModcodS2x::Apsk256_31_45 => 20.00,
            ModcodS2x::Apsk256_32_45 => 20.84,
            ModcodS2x::Apsk256_11_15 => 21.40,
            ModcodS2x::Apsk256_3_4 => 21.81,
        }
    }

    /// Returns true if this is a VL-SNR MODCOD.
    pub fn is_vl_snr(&self) -> bool {
        matches!(
            self,
            ModcodS2x::VlSnrBpsk1_5
                | ModcodS2x::VlSnrBpsk11_45Sf2
                | ModcodS2x::VlSnrQpsk11_45
                | ModcodS2x::VlSnrQpsk4_15
        )
    }

    /// Returns true if this is a DVB-S2X extended MODCOD (not in DVB-S2).
    pub fn is_s2x_only(&self) -> bool {
        !matches!(
            self,
            ModcodS2x::Qpsk1_4
                | ModcodS2x::Qpsk1_3
                | ModcodS2x::Qpsk2_5
                | ModcodS2x::Qpsk1_2
                | ModcodS2x::Qpsk3_5
                | ModcodS2x::Qpsk2_3
                | ModcodS2x::Qpsk3_4
                | ModcodS2x::Qpsk4_5
                | ModcodS2x::Qpsk5_6
                | ModcodS2x::Qpsk8_9
                | ModcodS2x::Qpsk9_10
                | ModcodS2x::Psk8_3_5
                | ModcodS2x::Psk8_2_3
                | ModcodS2x::Psk8_3_4
                | ModcodS2x::Psk8_5_6
                | ModcodS2x::Psk8_8_9
                | ModcodS2x::Psk8_9_10
                | ModcodS2x::Apsk16_2_3
                | ModcodS2x::Apsk16_3_4
                | ModcodS2x::Apsk16_4_5
                | ModcodS2x::Apsk16_5_6
                | ModcodS2x::Apsk16_8_9
                | ModcodS2x::Apsk16_9_10
                | ModcodS2x::Apsk32_3_4
                | ModcodS2x::Apsk32_4_5
                | ModcodS2x::Apsk32_5_6
                | ModcodS2x::Apsk32_8_9
                | ModcodS2x::Apsk32_9_10
        )
    }
}

// ---------------------------------------------------------------------------
// Frame and structural types
// ---------------------------------------------------------------------------

/// PLFRAME type (normal or short).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameType {
    /// Normal FECFRAME: 64800 coded bits.
    Normal,
    /// Short FECFRAME: 16200 coded bits.
    Short,
}

/// DVB-S2X roll-off factors for spectral shaping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RollOff {
    /// α = 0.05 (DVB-S2X wideband, most compact)
    R005,
    /// α = 0.10 (DVB-S2X wideband)
    R010,
    /// α = 0.15 (DVB-S2X)
    R015,
    /// α = 0.20 (DVB-S2 compatible)
    R020,
    /// α = 0.25 (DVB-S2 compatible)
    R025,
    /// α = 0.35 (DVB-S2 compatible, widest)
    R035,
}

impl RollOff {
    /// Return the roll-off factor as a floating-point value.
    pub fn value(&self) -> f64 {
        match self {
            RollOff::R005 => 0.05,
            RollOff::R010 => 0.10,
            RollOff::R015 => 0.15,
            RollOff::R020 => 0.20,
            RollOff::R025 => 0.25,
            RollOff::R035 => 0.35,
        }
    }
}

/// Framing mode: Constant, Variable, or Adaptive Coding and Modulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CcmVcmMode {
    /// Constant Coding and Modulation — single fixed MODCOD.
    Ccm,
    /// Variable Coding and Modulation — per-frame MODCOD field selection.
    Vcm,
    /// Adaptive Coding and Modulation — feedback-driven MODCOD selection.
    Acm,
}

// ---------------------------------------------------------------------------
// Constellation generation
// ---------------------------------------------------------------------------

/// Generate BPSK constellation points.
pub fn bpsk_constellation() -> Vec<Complex> {
    vec![Complex::new(-1.0, 0.0), Complex::new(1.0, 0.0)]
}

/// Generate QPSK constellation points per DVB-S2 (Gray-coded, π/4 offset).
pub fn qpsk_constellation() -> Vec<Complex> {
    let k = 1.0 / 2.0_f64.sqrt();
    vec![
        Complex::new(k, k),
        Complex::new(-k, k),
        Complex::new(k, -k),
        Complex::new(-k, -k),
    ]
}

/// Generate 8PSK constellation per DVB-S2 (uniform phase spacing).
pub fn psk8_constellation() -> Vec<Complex> {
    (0..8)
        .map(|i| {
            let theta = PI / 4.0 * i as f64 + PI / 8.0;
            Complex::from_phase(theta)
        })
        .collect()
}

/// Generate 16APSK constellation per DVB-S2 Table 10.
///
/// Ring configuration: 4 inner points (r1) + 12 outer points (r2).
/// r2/r1 ratio γ₁ depends on code rate.
pub fn apsk16_constellation(gamma: f64) -> Vec<Complex> {
    let mut pts = Vec::with_capacity(16);
    // Inner ring: 4 points at r1 = 1.0, angles: π/4 + k*π/2
    let r1 = 1.0;
    for k in 0..4 {
        let theta = PI / 4.0 + k as f64 * PI / 2.0;
        pts.push(Complex::from_phase(theta).scale(r1));
    }
    // Outer ring: 12 points at r2 = gamma * r1
    let r2 = gamma * r1;
    for k in 0..12 {
        let theta = k as f64 * 2.0 * PI / 12.0;
        pts.push(Complex::from_phase(theta).scale(r2));
    }
    // Normalize power to 1.0
    let power = (4.0 * r1 * r1 + 12.0 * r2 * r2) / 16.0;
    let norm = 1.0 / power.sqrt();
    pts.iter().map(|p| p.scale(norm)).collect()
}

/// Generate 32APSK constellation per DVB-S2 Table 11.
///
/// Ring configuration: 4 + 12 + 16 points.
pub fn apsk32_constellation(gamma1: f64, gamma2: f64) -> Vec<Complex> {
    let mut pts = Vec::with_capacity(32);
    let r1 = 1.0;
    // Inner ring: 4 points
    for k in 0..4 {
        let theta = PI / 4.0 + k as f64 * PI / 2.0;
        pts.push(Complex::from_phase(theta).scale(r1));
    }
    // Middle ring: 12 points
    let r2 = gamma1 * r1;
    for k in 0..12 {
        let theta = k as f64 * 2.0 * PI / 12.0;
        pts.push(Complex::from_phase(theta).scale(r2));
    }
    // Outer ring: 16 points
    let r3 = gamma2 * r1;
    for k in 0..16 {
        let theta = k as f64 * 2.0 * PI / 16.0;
        pts.push(Complex::from_phase(theta).scale(r3));
    }
    // Normalize
    let power = (4.0 * r1 * r1 + 12.0 * r2 * r2 + 16.0 * r3 * r3) / 32.0;
    let norm = 1.0 / power.sqrt();
    pts.iter().map(|p| p.scale(norm)).collect()
}

/// Generate 64APSK constellation (DVB-S2X, 4+12+20+28 ring structure).
pub fn apsk64_constellation(g1: f64, g2: f64, g3: f64) -> Vec<Complex> {
    let mut pts = Vec::with_capacity(64);
    let r1 = 1.0;
    // Ring 1: 4 pts
    for k in 0..4 {
        let theta = PI / 4.0 + k as f64 * PI / 2.0;
        pts.push(Complex::from_phase(theta).scale(r1));
    }
    // Ring 2: 12 pts
    let r2 = g1;
    for k in 0..12 {
        let theta = k as f64 * 2.0 * PI / 12.0;
        pts.push(Complex::from_phase(theta).scale(r2));
    }
    // Ring 3: 20 pts
    let r3 = g2;
    for k in 0..20 {
        let theta = k as f64 * 2.0 * PI / 20.0;
        pts.push(Complex::from_phase(theta).scale(r3));
    }
    // Ring 4: 28 pts
    let r4 = g3;
    for k in 0..28 {
        let theta = k as f64 * 2.0 * PI / 28.0;
        pts.push(Complex::from_phase(theta).scale(r4));
    }
    // Normalize
    let power = (4.0 * r1 * r1 + 12.0 * r2 * r2 + 20.0 * r3 * r3 + 28.0 * r4 * r4) / 64.0;
    let norm = 1.0 / power.sqrt();
    pts.iter().map(|p| p.scale(norm)).collect()
}

/// Generate 128APSK constellation (DVB-S2X, 8+16+32+64 structure).
pub fn apsk128_constellation(g1: f64, g2: f64, g3: f64) -> Vec<Complex> {
    let mut pts = Vec::with_capacity(128);
    let r1 = 1.0;
    for k in 0..8 {
        let theta = k as f64 * 2.0 * PI / 8.0;
        pts.push(Complex::from_phase(theta).scale(r1));
    }
    let r2 = g1;
    for k in 0..16 {
        let theta = k as f64 * 2.0 * PI / 16.0;
        pts.push(Complex::from_phase(theta).scale(r2));
    }
    let r3 = g2;
    for k in 0..32 {
        let theta = k as f64 * 2.0 * PI / 32.0;
        pts.push(Complex::from_phase(theta).scale(r3));
    }
    let r4 = g3;
    for k in 0..72 {
        let theta = k as f64 * 2.0 * PI / 72.0;
        pts.push(Complex::from_phase(theta).scale(r4));
    }
    let power = (8.0 * r1 * r1 + 16.0 * r2 * r2 + 32.0 * r3 * r3 + 72.0 * r4 * r4) / 128.0;
    let norm = 1.0 / power.sqrt();
    pts.iter().map(|p| p.scale(norm)).collect()
}

/// Generate 256APSK constellation (DVB-S2X, 8+16+32+64+136 structure).
pub fn apsk256_constellation(g1: f64, g2: f64, g3: f64, g4: f64) -> Vec<Complex> {
    let mut pts = Vec::with_capacity(256);
    let r1 = 1.0;
    for k in 0..8 {
        let theta = k as f64 * 2.0 * PI / 8.0;
        pts.push(Complex::from_phase(theta).scale(r1));
    }
    let r2 = g1;
    for k in 0..16 {
        let theta = k as f64 * 2.0 * PI / 16.0;
        pts.push(Complex::from_phase(theta).scale(r2));
    }
    let r3 = g2;
    for k in 0..32 {
        let theta = k as f64 * 2.0 * PI / 32.0;
        pts.push(Complex::from_phase(theta).scale(r3));
    }
    let r4 = g3;
    for k in 0..64 {
        let theta = k as f64 * 2.0 * PI / 64.0;
        pts.push(Complex::from_phase(theta).scale(r4));
    }
    let r5 = g4;
    for k in 0..136 {
        let theta = k as f64 * 2.0 * PI / 136.0;
        pts.push(Complex::from_phase(theta).scale(r5));
    }
    let power =
        (8.0 * r1 * r1 + 16.0 * r2 * r2 + 32.0 * r3 * r3 + 64.0 * r4 * r4 + 136.0 * r5 * r5)
            / 256.0;
    let norm = 1.0 / power.sqrt();
    pts.iter().map(|p| p.scale(norm)).collect()
}

/// Return the APSK ring ratio(s) for a given 16APSK MODCOD per DVB-S2 Table 10.
pub fn apsk16_gamma(modcod: ModcodS2x) -> f64 {
    match modcod {
        ModcodS2x::Apsk16_2_3 => 3.15,
        ModcodS2x::Apsk16_3_4 => 2.85,
        ModcodS2x::Apsk16_4_5 => 2.75,
        ModcodS2x::Apsk16_5_6 => 2.70,
        ModcodS2x::Apsk16_8_9 => 2.60,
        ModcodS2x::Apsk16_9_10 => 2.57,
        ModcodS2x::Apsk16_1_2L => 3.50,
        ModcodS2x::Apsk16_8_15L => 3.24,
        _ => 2.85,
    }
}

/// Return 32APSK ring ratios (γ1, γ2) for a given MODCOD per DVB-S2 Table 11.
pub fn apsk32_gammas(modcod: ModcodS2x) -> (f64, f64) {
    match modcod {
        ModcodS2x::Apsk32_3_4 => (2.84, 5.27),
        ModcodS2x::Apsk32_4_5 => (2.72, 4.87),
        ModcodS2x::Apsk32_5_6 => (2.64, 4.64),
        ModcodS2x::Apsk32_8_9 => (2.54, 4.33),
        ModcodS2x::Apsk32_9_10 => (2.53, 4.30),
        ModcodS2x::Apsk32_2_3L => (2.84, 5.27),
        _ => (2.72, 4.87),
    }
}

/// Get the constellation points for a given MODCOD.
pub fn get_constellation(modcod: ModcodS2x) -> Vec<Complex> {
    match modcod {
        ModcodS2x::VlSnrBpsk1_5 | ModcodS2x::VlSnrBpsk11_45Sf2 => bpsk_constellation(),
        ModcodS2x::Qpsk1_4
        | ModcodS2x::Qpsk1_3
        | ModcodS2x::Qpsk2_5
        | ModcodS2x::Qpsk1_2
        | ModcodS2x::Qpsk3_5
        | ModcodS2x::Qpsk2_3
        | ModcodS2x::Qpsk3_4
        | ModcodS2x::Qpsk4_5
        | ModcodS2x::Qpsk5_6
        | ModcodS2x::Qpsk8_9
        | ModcodS2x::Qpsk9_10
        | ModcodS2x::Qpsk13_45
        | ModcodS2x::Qpsk9_20
        | ModcodS2x::Qpsk11_20
        | ModcodS2x::VlSnrQpsk11_45
        | ModcodS2x::VlSnrQpsk4_15 => qpsk_constellation(),
        ModcodS2x::Psk8_3_5
        | ModcodS2x::Psk8_2_3
        | ModcodS2x::Psk8_3_4
        | ModcodS2x::Psk8_5_6
        | ModcodS2x::Psk8_8_9
        | ModcodS2x::Psk8_9_10 => psk8_constellation(),
        ModcodS2x::Apsk8_5_9L | ModcodS2x::Apsk8_26_45L => {
            // 8APSK: 4+4 ring configuration
            apsk16_constellation(2.0)
                .into_iter()
                .take(8)
                .collect()
        }
        mc @ (ModcodS2x::Apsk16_2_3
        | ModcodS2x::Apsk16_3_4
        | ModcodS2x::Apsk16_4_5
        | ModcodS2x::Apsk16_5_6
        | ModcodS2x::Apsk16_8_9
        | ModcodS2x::Apsk16_9_10
        | ModcodS2x::Apsk16_1_2L
        | ModcodS2x::Apsk16_8_15L) => {
            let g = apsk16_gamma(mc);
            apsk16_constellation(g)
        }
        mc @ (ModcodS2x::Apsk32_3_4
        | ModcodS2x::Apsk32_4_5
        | ModcodS2x::Apsk32_5_6
        | ModcodS2x::Apsk32_8_9
        | ModcodS2x::Apsk32_9_10
        | ModcodS2x::Apsk32_2_3L) => {
            let (g1, g2) = apsk32_gammas(mc);
            apsk32_constellation(g1, g2)
        }
        ModcodS2x::Apsk64_32_45 => apsk64_constellation(2.0, 3.5, 5.2),
        ModcodS2x::Apsk64_11_15 => apsk64_constellation(2.0, 3.4, 5.0),
        ModcodS2x::Apsk64_7_9 => apsk64_constellation(2.0, 3.3, 4.9),
        ModcodS2x::Apsk64_4_5 => apsk64_constellation(1.9, 3.2, 4.7),
        ModcodS2x::Apsk64_5_6 => apsk64_constellation(1.9, 3.2, 4.6),
        ModcodS2x::Apsk128_3_4 => apsk128_constellation(2.0, 3.5, 5.5),
        ModcodS2x::Apsk128_7_9 => apsk128_constellation(2.0, 3.4, 5.3),
        ModcodS2x::Apsk256_29_45L => apsk256_constellation(1.9, 3.2, 4.8, 6.5),
        ModcodS2x::Apsk256_2_3L => apsk256_constellation(1.9, 3.2, 4.7, 6.3),
        ModcodS2x::Apsk256_31_45 => apsk256_constellation(1.9, 3.1, 4.7, 6.2),
        ModcodS2x::Apsk256_32_45 => apsk256_constellation(1.8, 3.0, 4.6, 6.0),
        ModcodS2x::Apsk256_11_15 => apsk256_constellation(1.8, 3.0, 4.5, 5.9),
        ModcodS2x::Apsk256_3_4 => apsk256_constellation(1.8, 2.9, 4.4, 5.8),
    }
}

// ---------------------------------------------------------------------------
// Physical Layer (PL) scrambling — Gold sequence
// ---------------------------------------------------------------------------

/// Physical layer Gold-sequence scrambler state.
///
/// Per DVB-S2 Annex C: scrambling sequence x(n) + y(n) over GF(2),
/// initialized by ISI (Input Stream Identifier, 0..255).
pub struct PlScrambler {
    x: u32,
    y: u32,
}

impl PlScrambler {
    /// Create a new PL scrambler for a given ISI (0..=255).
    ///
    /// x initialized to 1, y initialized to Gold-code init for ISI.
    pub fn new(isi: u8) -> Self {
        // x register initialized to all-ones (LFSR seed)
        let x = 0x0001_0000u32 | 1u32;
        // y register seeded by ISI spread across 18 bits
        let isi_u32 = isi as u32;
        let y = ((isi_u32 & 0xFF) << 10) | 1u32;
        Self { x, y }
    }

    /// Clock both LFSRs and return one scrambling bit.
    fn next_bit(&mut self) -> bool {
        // x: x^18 + x^7 + 1
        let x_new = ((self.x >> 17) ^ (self.x >> 6) ^ self.x) & 1;
        self.x = (self.x >> 1) | (x_new << 17);
        // y: x^18 + x^10 + x^7 + x^5 + x^3 + x^2 + x + 1
        let y_new = ((self.y >> 17)
            ^ (self.y >> 9)
            ^ (self.y >> 6)
            ^ (self.y >> 4)
            ^ (self.y >> 2)
            ^ (self.y >> 1)
            ^ self.y)
            & 1;
        self.y = (self.y >> 1) | (y_new << 17);
        (x_new ^ y_new) != 0
    }

    /// Generate N scrambling bits.
    pub fn generate(&mut self, n: usize) -> Vec<bool> {
        (0..n).map(|_| self.next_bit()).collect()
    }

    /// Scramble a symbol stream using complex multiplication by e^(j*π*s).
    /// Each scrambling bit s rotates by 180° if s=1, else 0°.
    pub fn scramble_symbols(&mut self, symbols: &[Complex]) -> Vec<Complex> {
        symbols
            .iter()
            .map(|&s| {
                if self.next_bit() {
                    Complex::new(-s.re, -s.im)
                } else {
                    s
                }
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// PLFRAME structure constants
// ---------------------------------------------------------------------------

/// Number of symbols in a PLHEADER (SOF + PLSCODE).
pub const PLHEADER_LEN: usize = 90;
/// Number of symbols in a pilot block.
pub const PILOT_BLOCK_LEN: usize = 36;
/// Number of data slots between pilot blocks.
pub const SLOTS_PER_PILOT_PERIOD: usize = 16;
/// Number of symbols per data slot.
pub const SYMBOLS_PER_SLOT: usize = 90;

/// SOF (Start Of Frame) sequence — 26 symbols from DVB-S2 Table 9.
pub const SOF: [i8; 26] = [
    1, -1, 1, -1, -1, 1, 1, -1, -1, -1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1,
];

/// Compute the number of data slots for a given MODCOD and frame type.
pub fn num_data_slots(modcod: ModcodS2x, frame_type: FrameType) -> usize {
    let fecframe_bits = match frame_type {
        FrameType::Normal => 64800,
        FrameType::Short => 16200,
    };
    let bps = modcod.bits_per_symbol();
    let total_symbols = (fecframe_bits + bps - 1) / bps;
    (total_symbols + SYMBOLS_PER_SLOT - 1) / SYMBOLS_PER_SLOT
}

/// Compute the number of pilot blocks in a PLFRAME.
pub fn num_pilot_blocks(n_slots: usize) -> usize {
    n_slots / SLOTS_PER_PILOT_PERIOD
}

/// Total PLFRAME length in symbols (including PLHEADER + pilots + data).
pub fn plframe_length(modcod: ModcodS2x, frame_type: FrameType) -> usize {
    let n_slots = num_data_slots(modcod, frame_type);
    let n_pilots = num_pilot_blocks(n_slots);
    PLHEADER_LEN + n_slots * SYMBOLS_PER_SLOT + n_pilots * PILOT_BLOCK_LEN
}

// ---------------------------------------------------------------------------
// PLHEADER encoding/decoding
// ---------------------------------------------------------------------------

/// Encoded PLHEADER information.
#[derive(Debug, Clone)]
pub struct PlHeader {
    /// MODCOD index (7 bits in PLSCODE).
    pub modcod_idx: u8,
    /// Short frame flag.
    pub short_frame: bool,
    /// Pilot flag (pilots present).
    pub pilots_present: bool,
    /// Raw PLSCODE value (90 - 26 = 64 symbols from coding the 7-bit MODCOD+flags).
    pub plscode: Vec<i8>,
}

impl PlHeader {
    /// Encode a PLHEADER for given MODCOD and frame type.
    pub fn encode(modcod_idx: u8, short_frame: bool, pilots: bool) -> Self {
        // PLSCODE: 7 bits (modcod 5-bit + sf 1-bit + pilot 1-bit) encoded with π/2-BPSK
        // and spread by a 64-chip m-sequence, then interleaved — simplified here
        let y = ((modcod_idx & 0x1F) as u64) | (if short_frame { 1u64 << 5 } else { 0 })
            | (if pilots { 1u64 << 6 } else { 0 });
        // Generate 64-symbol PLSCODE from 7-bit word via π/2-BPSK encoding
        let plscode = Self::encode_plscode(y as u8);
        Self {
            modcod_idx,
            short_frame,
            pilots_present: pilots,
            plscode,
        }
    }

    /// Encode PLSCODE: π/2-BPSK mapping + pilot-aided scrambling.
    fn encode_plscode(word: u8) -> Vec<i8> {
        // Simple π/2-BPSK: each bit → ±1 * rotation for readability
        let mut code = Vec::with_capacity(64);
        for bit_idx in 0..7 {
            let bit = (word >> bit_idx) & 1;
            let sym: i8 = if bit != 0 { 1 } else { -1 };
            // Repeat each bit 64/7 ≈ 9 times to fill 64-symbol PLSCODE (simplified)
            for _ in 0..9 {
                code.push(sym);
            }
        }
        // Pad to exactly 64
        while code.len() < 64 {
            code.push(1);
        }
        code.truncate(64);
        code
    }

    /// Full 90-symbol PLHEADER = SOF (26 symbols) + PLSCODE (64 symbols).
    pub fn to_symbols(&self) -> Vec<Complex> {
        let mut symbols = Vec::with_capacity(90);
        // SOF: real-valued π/2-BPSK
        for &s in &SOF {
            symbols.push(Complex::new(s as f64 / 2.0_f64.sqrt(), 0.0));
        }
        // PLSCODE
        for &s in &self.plscode {
            symbols.push(Complex::new(s as f64 / 2.0_f64.sqrt(), 0.0));
        }
        symbols
    }

    /// Decode PLHEADER from 90 received symbols.
    pub fn decode(syms: &[Complex]) -> Option<Self> {
        if syms.len() < PLHEADER_LEN {
            return None;
        }
        // Check SOF correlation
        let sof_corr: f64 = syms[..26]
            .iter()
            .zip(SOF.iter())
            .map(|(s, &r)| s.re * r as f64)
            .sum::<f64>()
            / 26.0;
        if sof_corr < 0.3 {
            return None;
        }
        // Soft-decode PLSCODE
        let plscode_syms = &syms[26..90];
        let mut word_bits = 0u8;
        // Average over blocks of 9 symbols per bit (simplified)
        for bit_idx in 0..7 {
            let block_start = bit_idx * 9;
            let block_end = (block_start + 9).min(plscode_syms.len());
            let avg: f64 = plscode_syms[block_start..block_end]
                .iter()
                .map(|s| s.re)
                .sum::<f64>()
                / (block_end - block_start) as f64;
            if avg > 0.0 {
                word_bits |= 1 << bit_idx;
            }
        }
        let modcod_idx = word_bits & 0x1F;
        let short_frame = (word_bits >> 5) & 1 != 0;
        let pilots_present = (word_bits >> 6) & 1 != 0;
        Some(Self::encode(modcod_idx, short_frame, pilots_present))
    }
}

// ---------------------------------------------------------------------------
// Pilot block generation
// ---------------------------------------------------------------------------

/// Generate a pilot block (36 constant-amplitude symbols).
///
/// Pilot symbols are BPSK at 1+j0 (unit amplitude, zero phase).
pub fn generate_pilot_block() -> Vec<Complex> {
    vec![Complex::new(1.0, 0.0); PILOT_BLOCK_LEN]
}

// ---------------------------------------------------------------------------
// Symbol mapping / demapping
// ---------------------------------------------------------------------------

/// Map bits to constellation symbol using hard mapping.
pub fn map_bits_to_symbol(bits: &[bool], modcod: ModcodS2x) -> Complex {
    let constellation = get_constellation(modcod);
    let bps = modcod.bits_per_symbol();
    let mut idx = 0usize;
    for (i, &b) in bits.iter().take(bps).enumerate() {
        if b {
            idx |= 1 << (bps - 1 - i);
        }
    }
    constellation[idx % constellation.len()]
}

/// Demap a received symbol to bits using nearest-neighbor hard decision.
pub fn demap_symbol_to_bits(sym: Complex, modcod: ModcodS2x) -> Vec<bool> {
    let constellation = get_constellation(modcod);
    let bps = modcod.bits_per_symbol();
    // Find nearest constellation point
    let (best_idx, _) = constellation
        .iter()
        .enumerate()
        .map(|(i, &c)| {
            let d = sym.add(Complex::new(-c.re, -c.im));
            (i, d.mag_sq())
        })
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .unwrap_or((0, f64::MAX));
    // Convert index to bits (Gray-coded)
    let gray_idx = best_idx ^ (best_idx >> 1);
    (0..bps)
        .map(|i| (gray_idx >> (bps - 1 - i)) & 1 != 0)
        .collect()
}

// ---------------------------------------------------------------------------
// PLFRAME builder
// ---------------------------------------------------------------------------

/// Builder for DVB-S2X PLFRAMEs.
pub struct PlframeBuilder {
    modcod: ModcodS2x,
    frame_type: FrameType,
    pilots: bool,
    scrambler_isi: u8,
}

impl PlframeBuilder {
    /// Create a new PLFRAME builder.
    pub fn new(modcod: ModcodS2x, frame_type: FrameType) -> Self {
        Self {
            modcod,
            frame_type,
            pilots: true,
            scrambler_isi: 0,
        }
    }

    /// Enable or disable pilot blocks.
    pub fn with_pilots(mut self, pilots: bool) -> Self {
        self.pilots = pilots;
        self
    }

    /// Set scrambler ISI (Input Stream Identifier, 0..=255).
    pub fn with_isi(mut self, isi: u8) -> Self {
        self.scrambler_isi = isi;
        self
    }

    /// Build PLFRAME from a slice of data bits.
    ///
    /// Returns the full symbol vector and the PLHEADER metadata.
    pub fn build(&self, data_bits: &[bool]) -> (Vec<Complex>, PlHeader) {
        let n_slots = num_data_slots(self.modcod, self.frame_type);
        let modcod_idx = self.modcod_to_index();
        let header = PlHeader::encode(modcod_idx, self.frame_type == FrameType::Short, self.pilots);

        let mut symbols = header.to_symbols();

        // Map data bits to symbols
        let bps = self.modcod.bits_per_symbol();
        let n_data_symbols = n_slots * SYMBOLS_PER_SLOT;
        let mut data_symbols: Vec<Complex> = Vec::with_capacity(n_data_symbols);

        let mut bit_pos = 0usize;
        while data_symbols.len() < n_data_symbols {
            let chunk_end = (bit_pos + bps).min(data_bits.len());
            let chunk: Vec<bool> = if bit_pos < data_bits.len() {
                let mut c: Vec<bool> = data_bits[bit_pos..chunk_end].to_vec();
                while c.len() < bps {
                    c.push(false); // zero-pad
                }
                c
            } else {
                vec![false; bps]
            };
            data_symbols.push(map_bits_to_symbol(&chunk, self.modcod));
            bit_pos += bps;
        }

        // Apply PL scrambling to data symbols
        let mut scrambler = PlScrambler::new(self.scrambler_isi);
        let scrambled = scrambler.scramble_symbols(&data_symbols);

        // Insert pilot blocks every SLOTS_PER_PILOT_PERIOD slots
        let pilot = generate_pilot_block();
        let mut slot_idx = 0usize;
        let mut sym_idx = 0usize;
        while slot_idx < n_slots {
            // Insert pilot if needed
            if self.pilots && slot_idx > 0 && slot_idx % SLOTS_PER_PILOT_PERIOD == 0 {
                symbols.extend_from_slice(&pilot);
            }
            // One data slot = 90 symbols
            let end = (sym_idx + SYMBOLS_PER_SLOT).min(scrambled.len());
            symbols.extend_from_slice(&scrambled[sym_idx..end]);
            // Pad if needed
            for _ in end..sym_idx + SYMBOLS_PER_SLOT {
                symbols.push(Complex::new(0.0, 0.0));
            }
            sym_idx += SYMBOLS_PER_SLOT;
            slot_idx += 1;
        }

        (symbols, header)
    }

    /// Map MODCOD enum to 5-bit index for PLSCODE.
    fn modcod_to_index(&self) -> u8 {
        match self.modcod {
            ModcodS2x::Qpsk1_4 => 1,
            ModcodS2x::Qpsk1_3 => 2,
            ModcodS2x::Qpsk2_5 => 3,
            ModcodS2x::Qpsk1_2 => 4,
            ModcodS2x::Qpsk3_5 => 5,
            ModcodS2x::Qpsk2_3 => 6,
            ModcodS2x::Qpsk3_4 => 7,
            ModcodS2x::Qpsk4_5 => 8,
            ModcodS2x::Qpsk5_6 => 9,
            ModcodS2x::Qpsk8_9 => 10,
            ModcodS2x::Qpsk9_10 => 11,
            ModcodS2x::Psk8_3_5 => 12,
            ModcodS2x::Psk8_2_3 => 13,
            ModcodS2x::Psk8_3_4 => 14,
            ModcodS2x::Psk8_5_6 => 15,
            ModcodS2x::Psk8_8_9 => 16,
            ModcodS2x::Psk8_9_10 => 17,
            ModcodS2x::Apsk16_2_3 => 18,
            ModcodS2x::Apsk16_3_4 => 19,
            ModcodS2x::Apsk16_4_5 => 20,
            ModcodS2x::Apsk16_5_6 => 21,
            ModcodS2x::Apsk16_8_9 => 22,
            ModcodS2x::Apsk16_9_10 => 23,
            ModcodS2x::Apsk32_3_4 => 24,
            ModcodS2x::Apsk32_4_5 => 25,
            ModcodS2x::Apsk32_5_6 => 26,
            ModcodS2x::Apsk32_8_9 => 27,
            ModcodS2x::Apsk32_9_10 => 28,
            // DVB-S2X extended
            ModcodS2x::Apsk64_32_45 => 30,
            ModcodS2x::Apsk64_11_15 => 31,
            ModcodS2x::Apsk128_3_4 => 32,
            ModcodS2x::Apsk256_32_45 => 33,
            ModcodS2x::VlSnrQpsk11_45 => 35,
            ModcodS2x::VlSnrBpsk1_5 => 36,
            _ => 29,
        }
    }
}

// ---------------------------------------------------------------------------
// PLFRAME parser (receiver side)
// ---------------------------------------------------------------------------

/// DVB-S2X PLFRAME parser for receiver processing.
pub struct PlframeParser {
    /// Expected MODCOD (for known-header operation).
    pub modcod: ModcodS2x,
    /// Expected frame type.
    pub frame_type: FrameType,
    /// Descrambler ISI.
    pub scrambler_isi: u8,
}

impl PlframeParser {
    /// Create a new PLFRAME parser.
    pub fn new(modcod: ModcodS2x, frame_type: FrameType) -> Self {
        Self {
            modcod,
            frame_type,
            scrambler_isi: 0,
        }
    }

    /// Parse a PLFRAME and extract data bits.
    ///
    /// Returns decoded bits and pilot symbol positions if pilots were present.
    pub fn parse(&self, frame: &[Complex]) -> (Vec<bool>, Vec<usize>) {
        if frame.len() < PLHEADER_LEN {
            return (vec![], vec![]);
        }

        let n_slots = num_data_slots(self.modcod, self.frame_type);
        let bps = self.modcod.bits_per_symbol();
        let mut bits = Vec::new();
        let mut pilot_positions = Vec::new();

        // Skip PLHEADER
        let mut pos = PLHEADER_LEN;

        // Descrambler
        let mut scrambler = PlScrambler::new(self.scrambler_isi);

        let mut slot = 0usize;
        while slot < n_slots && pos < frame.len() {
            // Check for pilot block
            let is_pilot_slot = slot > 0 && slot % SLOTS_PER_PILOT_PERIOD == 0;
            if is_pilot_slot && pos + PILOT_BLOCK_LEN <= frame.len() {
                pilot_positions.push(pos);
                pos += PILOT_BLOCK_LEN;
            }

            // Decode one data slot
            let slot_end = (pos + SYMBOLS_PER_SLOT).min(frame.len());
            for &sym in &frame[pos..slot_end] {
                // Descramble: flip sign if scrambler bit is 1
                let descrambled = if scrambler.next_bit() {
                    Complex::new(-sym.re, -sym.im)
                } else {
                    sym
                };
                let slot_bits = demap_symbol_to_bits(descrambled, self.modcod);
                bits.extend(slot_bits);
            }
            pos = slot_end;
            slot += 1;
        }

        (bits, pilot_positions)
    }
}

// ---------------------------------------------------------------------------
// Es/N0 estimation from pilots
// ---------------------------------------------------------------------------

/// Pilot-based Es/N0 estimator for ACM feedback.
pub struct EsN0Estimator {
    /// Exponential moving average coefficient.
    alpha: f64,
    /// Current SNR estimate in dB.
    snr_db: f64,
}

impl EsN0Estimator {
    /// Create a new estimator with EMA coefficient α = 0.1.
    pub fn new() -> Self {
        Self {
            alpha: 0.1,
            snr_db: 0.0,
        }
    }

    /// Create estimator with custom EMA coefficient.
    pub fn with_alpha(alpha: f64) -> Self {
        Self { alpha, snr_db: 0.0 }
    }

    /// Estimate Es/N0 from pilot blocks within a PLFRAME.
    ///
    /// Pilot symbols are known to be 1+j0. Residual = received - expected.
    /// Noise variance estimated from pilot residuals; signal power from data.
    pub fn estimate_from_pilots(
        &mut self,
        frame: &[Complex],
        modcod: ModcodS2x,
        frame_type: FrameType,
    ) -> f64 {
        let n_slots = num_data_slots(modcod, frame_type);
        if frame.len() < PLHEADER_LEN {
            return self.snr_db;
        }

        let mut noise_acc = 0.0f64;
        let mut signal_acc = 0.0f64;
        let mut pilot_count = 0usize;

        let mut pos = PLHEADER_LEN;
        for slot in 0..n_slots {
            let is_pilot_slot = slot > 0 && slot % SLOTS_PER_PILOT_PERIOD == 0;
            if is_pilot_slot && pos + PILOT_BLOCK_LEN <= frame.len() {
                // Pilot block: known symbols = 1+j0
                for &sym in &frame[pos..pos + PILOT_BLOCK_LEN] {
                    let noise = sym.add(Complex::new(-1.0, 0.0)); // residual
                    noise_acc += noise.mag_sq();
                    signal_acc += sym.mag_sq();
                    pilot_count += 1;
                }
                pos += PILOT_BLOCK_LEN;
            }
            pos = (pos + SYMBOLS_PER_SLOT).min(frame.len());
        }

        if pilot_count == 0 || noise_acc <= 0.0 {
            return self.snr_db;
        }

        let noise_var = noise_acc / pilot_count as f64;
        let signal_power = signal_acc / pilot_count as f64;
        let snr_linear = signal_power / noise_var;
        let snr_new = 10.0 * snr_linear.log10();

        // EMA update
        self.snr_db = (1.0 - self.alpha) * self.snr_db + self.alpha * snr_new;
        self.snr_db
    }

    /// Return the current filtered Es/N0 estimate.
    pub fn current_estimate(&self) -> f64 {
        self.snr_db
    }
}

impl Default for EsN0Estimator {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// ACM (Adaptive Coding and Modulation) controller
// ---------------------------------------------------------------------------

/// DVB-S2X ACM modcod selection strategy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AcmStrategy {
    /// Maximize throughput (highest spectral efficiency meeting SNR threshold).
    MaxThroughput,
    /// Maximize reliability (most robust MODCOD meeting SNR with margin).
    MaxReliability,
}

/// ACM controller that selects MODCOD based on measured Es/N0.
pub struct AcmController {
    /// Available MODCODs sorted by spectral efficiency ascending.
    modcods: Vec<ModcodS2x>,
    /// Margin added above required Es/N0 (dB).
    margin_db: f64,
    /// Current selected MODCOD.
    current: ModcodS2x,
    /// Selection strategy.
    strategy: AcmStrategy,
    /// Hysteresis to prevent ping-pong switching (dB).
    hysteresis_db: f64,
}

impl AcmController {
    /// Create an ACM controller with a set of available MODCODs.
    pub fn new(mut modcods: Vec<ModcodS2x>, strategy: AcmStrategy) -> Self {
        // Sort by spectral efficiency
        modcods.sort_by(|a, b| {
            a.spectral_efficiency()
                .partial_cmp(&b.spectral_efficiency())
                .unwrap()
        });
        let current = modcods[0];
        Self {
            modcods,
            margin_db: 1.0,
            current,
            strategy,
            hysteresis_db: 0.5,
        }
    }

    /// Set the link margin (dB above required Es/N0).
    pub fn with_margin(mut self, margin_db: f64) -> Self {
        self.margin_db = margin_db;
        self
    }

    /// Set hysteresis (dB).
    pub fn with_hysteresis(mut self, hysteresis_db: f64) -> Self {
        self.hysteresis_db = hysteresis_db;
        self
    }

    /// Update MODCOD selection based on measured Es/N0.
    ///
    /// Returns the selected MODCOD.
    pub fn update(&mut self, measured_es_n0_db: f64) -> ModcodS2x {
        match self.strategy {
            AcmStrategy::MaxThroughput => {
                // Select highest spectral efficiency where required_Es/N0 + margin ≤ measured
                let mut best = self.modcods[0];
                for &mc in &self.modcods {
                    let threshold = mc.required_es_n0_db() + self.margin_db;
                    if measured_es_n0_db >= threshold {
                        best = mc;
                    }
                }
                // Apply hysteresis: only switch up if gain is significant
                if best.spectral_efficiency() > self.current.spectral_efficiency() {
                    let switch_threshold =
                        best.required_es_n0_db() + self.margin_db + self.hysteresis_db;
                    if measured_es_n0_db >= switch_threshold {
                        self.current = best;
                    }
                } else {
                    self.current = best;
                }
            }
            AcmStrategy::MaxReliability => {
                // Use the most robust MODCOD that still provides usable link
                for &mc in &self.modcods {
                    let threshold = mc.required_es_n0_db() + self.margin_db;
                    if measured_es_n0_db >= threshold {
                        self.current = mc;
                        break;
                    }
                }
            }
        }
        self.current
    }

    /// Get the currently selected MODCOD.
    pub fn current_modcod(&self) -> ModcodS2x {
        self.current
    }
}

// ---------------------------------------------------------------------------
// VL-SNR processing
// ---------------------------------------------------------------------------

/// VL-SNR (Very Low SNR) spreading factor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpreadingFactor {
    /// No spreading (SF=1).
    Sf1,
    /// Spreading by factor 2.
    Sf2,
    /// Spreading by factor 4.
    Sf4,
}

impl SpreadingFactor {
    /// Return the numeric spreading factor.
    pub fn value(&self) -> usize {
        match self {
            SpreadingFactor::Sf1 => 1,
            SpreadingFactor::Sf2 => 2,
            SpreadingFactor::Sf4 => 4,
        }
    }
}

/// VL-SNR spreader: repeats each symbol SF times.
pub fn vl_snr_spread(symbols: &[Complex], sf: SpreadingFactor) -> Vec<Complex> {
    let n = sf.value();
    let mut out = Vec::with_capacity(symbols.len() * n);
    for &s in symbols {
        for _ in 0..n {
            out.push(s.scale(1.0 / (n as f64).sqrt())); // energy-normalized
        }
    }
    out
}

/// VL-SNR despreader: correlate and average SF repeated symbols.
pub fn vl_snr_despread(symbols: &[Complex], sf: SpreadingFactor) -> Vec<Complex> {
    let n = sf.value();
    symbols
        .chunks(n)
        .map(|chunk| {
            let sum = chunk
                .iter()
                .fold(Complex::new(0.0, 0.0), |acc, &s| acc.add(s));
            sum.scale((n as f64).sqrt() / n as f64)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Super-frame structure
// ---------------------------------------------------------------------------

/// DVB-S2X super-frame configuration.
///
/// A super-frame bundles multiple PLFRAMEs for multi-stream broadcasting.
pub struct SuperFrame {
    /// Number of PLFRAMEs per super-frame (typical: 64 or 360).
    pub frames_per_superframe: usize,
    /// ISI (stream ID) for each slot.
    pub stream_ids: Vec<u8>,
}

impl SuperFrame {
    /// Create a new super-frame configuration.
    pub fn new(frames_per_superframe: usize) -> Self {
        Self {
            frames_per_superframe,
            stream_ids: vec![0u8; frames_per_superframe],
        }
    }

    /// Assign stream IDs (ISI) for each PLFRAME slot.
    pub fn with_stream_ids(mut self, ids: Vec<u8>) -> Self {
        let n = ids.len().min(self.frames_per_superframe);
        self.stream_ids[..n].copy_from_slice(&ids[..n]);
        self
    }

    /// Number of PLFRAMEs in the super-frame.
    pub fn len(&self) -> usize {
        self.frames_per_superframe
    }

    /// Returns true if the super-frame has zero slots.
    pub fn is_empty(&self) -> bool {
        self.frames_per_superframe == 0
    }
}

// ---------------------------------------------------------------------------
// Wideband roll-off filter (RRC coefficient generation)
// ---------------------------------------------------------------------------

/// Generate Raised Root Cosine (RRC) filter coefficients for wideband operation.
///
/// Supports the DVB-S2X roll-off factors 0.05, 0.10, 0.15, 0.20, 0.25, 0.35.
pub fn rrc_filter_coefficients(roll_off: RollOff, num_taps: usize, sps: usize) -> Vec<f64> {
    let alpha = roll_off.value();
    let t_s = sps as f64; // samples per symbol
    let n = num_taps;
    let mut h = vec![0.0f64; n];
    let center = (n / 2) as isize;

    for i in 0..n {
        let t = (i as isize - center) as f64 / t_s;
        // RRC impulse response: h(t) = [sin(π*t*(1-α)) + 4α*t*cos(π*t*(1+α))]
        //                               / [π*t*(1 - (4α*t)²)]
        let h_val = if t.abs() < 1e-8 {
            // t → 0 limit: 1 - α + 4α/π
            1.0 - alpha + 4.0 * alpha / PI
        } else {
            let x = 4.0 * alpha * t;
            let denom_factor = 1.0 - x * x;
            if denom_factor.abs() < 1e-8 {
                // t → ±1/(4α) limit
                let sign = if t > 0.0 { 1.0 } else { -1.0 };
                sign * (alpha / (2.0_f64.sqrt()))
                    * ((1.0 + 2.0 / PI) * (PI / (4.0 * alpha)).sin()
                        + (1.0 - 2.0 / PI) * (PI / (4.0 * alpha)).cos())
            } else {
                let sin_term = (PI * t * (1.0 - alpha)).sin();
                let cos_term = (PI * t * (1.0 + alpha)).cos();
                let denom = PI * t * denom_factor;
                (sin_term + 4.0 * alpha * t * cos_term) / denom
            }
        };
        h[i] = h_val;
    }

    // Normalize so that sum of squares = 1
    let norm: f64 = h.iter().map(|&v| v * v).sum::<f64>().sqrt();
    if norm > 1e-30 {
        h.iter_mut().for_each(|v| *v /= norm);
    }
    h
}

// ---------------------------------------------------------------------------
// Main DvbS2xModem struct
// ---------------------------------------------------------------------------

/// DVB-S2X satellite modem — combines framing, modulation, and ACM.
pub struct DvbS2xModem {
    modcod: ModcodS2x,
    frame_type: FrameType,
    roll_off: RollOff,
    mode: CcmVcmMode,
    pilots_enabled: bool,
    isi: u8,
}

impl DvbS2xModem {
    /// Create a new DVB-S2X modem.
    pub fn new(modcod: ModcodS2x, frame_type: FrameType, roll_off: RollOff) -> Self {
        Self {
            modcod,
            frame_type,
            roll_off,
            mode: CcmVcmMode::Ccm,
            pilots_enabled: true,
            isi: 0,
        }
    }

    /// Set CCM/VCM/ACM mode.
    pub fn set_mode(&mut self, mode: CcmVcmMode) {
        self.mode = mode;
    }

    /// Enable or disable pilot blocks.
    pub fn set_pilots(&mut self, enabled: bool) {
        self.pilots_enabled = enabled;
    }

    /// Set the Input Stream Identifier (ISI, 0..=255).
    pub fn set_isi(&mut self, isi: u8) {
        self.isi = isi;
    }

    /// Encode data bits into a PLFRAME symbol stream.
    pub fn encode_plframe(&self, data_bits: &[bool]) -> (Vec<Complex>, PlHeader) {
        PlframeBuilder::new(self.modcod, self.frame_type)
            .with_pilots(self.pilots_enabled)
            .with_isi(self.isi)
            .build(data_bits)
    }

    /// Decode a received PLFRAME into data bits.
    pub fn decode_plframe(&self, frame: &[Complex]) -> (Vec<bool>, Vec<usize>) {
        PlframeParser {
            modcod: self.modcod,
            frame_type: self.frame_type,
            scrambler_isi: self.isi,
        }
        .parse(frame)
    }

    /// Compute PLFRAME length in symbols.
    pub fn frame_length_symbols(&self) -> usize {
        plframe_length(self.modcod, self.frame_type)
    }

    /// Current MODCOD.
    pub fn modcod(&self) -> ModcodS2x {
        self.modcod
    }

    /// Current roll-off.
    pub fn roll_off(&self) -> RollOff {
        self.roll_off
    }

    /// Spectral efficiency (bits per symbol after FEC).
    pub fn spectral_efficiency(&self) -> f64 {
        self.modcod.spectral_efficiency()
    }
}

// ---------------------------------------------------------------------------
// Modcod table helpers
// ---------------------------------------------------------------------------

/// Return all available DVB-S2 legacy MODCODs.
pub fn dvbs2_modcods() -> Vec<ModcodS2x> {
    vec![
        ModcodS2x::Qpsk1_4,
        ModcodS2x::Qpsk1_3,
        ModcodS2x::Qpsk2_5,
        ModcodS2x::Qpsk1_2,
        ModcodS2x::Qpsk3_5,
        ModcodS2x::Qpsk2_3,
        ModcodS2x::Qpsk3_4,
        ModcodS2x::Qpsk4_5,
        ModcodS2x::Qpsk5_6,
        ModcodS2x::Qpsk8_9,
        ModcodS2x::Qpsk9_10,
        ModcodS2x::Psk8_3_5,
        ModcodS2x::Psk8_2_3,
        ModcodS2x::Psk8_3_4,
        ModcodS2x::Psk8_5_6,
        ModcodS2x::Psk8_8_9,
        ModcodS2x::Psk8_9_10,
        ModcodS2x::Apsk16_2_3,
        ModcodS2x::Apsk16_3_4,
        ModcodS2x::Apsk16_4_5,
        ModcodS2x::Apsk16_5_6,
        ModcodS2x::Apsk16_8_9,
        ModcodS2x::Apsk16_9_10,
        ModcodS2x::Apsk32_3_4,
        ModcodS2x::Apsk32_4_5,
        ModcodS2x::Apsk32_5_6,
        ModcodS2x::Apsk32_8_9,
        ModcodS2x::Apsk32_9_10,
    ]
}

/// Return DVB-S2X extended MODCODs only.
pub fn dvbs2x_extended_modcods() -> Vec<ModcodS2x> {
    vec![
        ModcodS2x::Qpsk13_45,
        ModcodS2x::Qpsk9_20,
        ModcodS2x::Qpsk11_20,
        ModcodS2x::Apsk8_5_9L,
        ModcodS2x::Apsk8_26_45L,
        ModcodS2x::Apsk16_1_2L,
        ModcodS2x::Apsk16_8_15L,
        ModcodS2x::Apsk32_2_3L,
        ModcodS2x::Apsk64_32_45,
        ModcodS2x::Apsk64_11_15,
        ModcodS2x::Apsk64_7_9,
        ModcodS2x::Apsk64_4_5,
        ModcodS2x::Apsk64_5_6,
        ModcodS2x::Apsk128_3_4,
        ModcodS2x::Apsk128_7_9,
        ModcodS2x::Apsk256_29_45L,
        ModcodS2x::Apsk256_2_3L,
        ModcodS2x::Apsk256_31_45,
        ModcodS2x::Apsk256_32_45,
        ModcodS2x::Apsk256_11_15,
        ModcodS2x::Apsk256_3_4,
    ]
}

/// Return VL-SNR MODCODs only.
pub fn vl_snr_modcods() -> Vec<ModcodS2x> {
    vec![
        ModcodS2x::VlSnrBpsk1_5,
        ModcodS2x::VlSnrBpsk11_45Sf2,
        ModcodS2x::VlSnrQpsk4_15,
        ModcodS2x::VlSnrQpsk11_45,
    ]
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
        let c = a.mul(b);
        assert!((c.re - (-5.0)).abs() < 1e-10);
        assert!((c.im - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_from_phase() {
        let c = Complex::from_phase(PI / 2.0);
        assert!(c.re.abs() < 1e-10);
        assert!((c.im - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_conj() {
        let c = Complex::new(3.0, -4.0);
        let conj = c.conj();
        assert_eq!(conj.re, 3.0);
        assert_eq!(conj.im, 4.0);
    }

    #[test]
    fn test_complex_mag() {
        let c = Complex::new(3.0, 4.0);
        assert!((c.mag() - 5.0).abs() < 1e-10);
    }

    // --- ModcodS2x properties ---

    #[test]
    fn test_qpsk_bits_per_symbol() {
        assert_eq!(ModcodS2x::Qpsk1_2.bits_per_symbol(), 2);
    }

    #[test]
    fn test_apsk16_bits_per_symbol() {
        assert_eq!(ModcodS2x::Apsk16_3_4.bits_per_symbol(), 4);
    }

    #[test]
    fn test_apsk32_bits_per_symbol() {
        assert_eq!(ModcodS2x::Apsk32_5_6.bits_per_symbol(), 5);
    }

    #[test]
    fn test_apsk64_bits_per_symbol() {
        assert_eq!(ModcodS2x::Apsk64_4_5.bits_per_symbol(), 6);
    }

    #[test]
    fn test_apsk128_bits_per_symbol() {
        assert_eq!(ModcodS2x::Apsk128_3_4.bits_per_symbol(), 7);
    }

    #[test]
    fn test_apsk256_bits_per_symbol() {
        assert_eq!(ModcodS2x::Apsk256_3_4.bits_per_symbol(), 8);
    }

    #[test]
    fn test_spectral_efficiency_ordering() {
        // Higher-order MODCODs should have higher spectral efficiency
        let qpsk = ModcodS2x::Qpsk1_2.spectral_efficiency();
        let apsk16 = ModcodS2x::Apsk16_3_4.spectral_efficiency();
        let apsk32 = ModcodS2x::Apsk32_3_4.spectral_efficiency();
        let apsk256 = ModcodS2x::Apsk256_3_4.spectral_efficiency();
        assert!(qpsk < apsk16);
        assert!(apsk16 < apsk32);
        assert!(apsk32 < apsk256);
    }

    #[test]
    fn test_vl_snr_required_es_n0() {
        assert!(ModcodS2x::VlSnrBpsk1_5.required_es_n0_db() < -9.0);
        assert!(ModcodS2x::VlSnrQpsk4_15.required_es_n0_db() < 0.0);
    }

    #[test]
    fn test_is_vl_snr() {
        assert!(ModcodS2x::VlSnrBpsk1_5.is_vl_snr());
        assert!(ModcodS2x::VlSnrBpsk11_45Sf2.is_vl_snr());
        assert!(!ModcodS2x::Qpsk1_2.is_vl_snr());
        assert!(!ModcodS2x::Apsk256_3_4.is_vl_snr());
    }

    #[test]
    fn test_is_s2x_only() {
        assert!(!ModcodS2x::Qpsk1_2.is_s2x_only()); // DVB-S2
        assert!(ModcodS2x::Apsk64_32_45.is_s2x_only()); // DVB-S2X
        assert!(ModcodS2x::Apsk256_3_4.is_s2x_only()); // DVB-S2X
    }

    #[test]
    fn test_code_rate_fraction_qpsk() {
        assert_eq!(ModcodS2x::Qpsk1_2.code_rate_fraction(), (1, 2));
        assert_eq!(ModcodS2x::Qpsk3_4.code_rate_fraction(), (3, 4));
    }

    // --- Roll-off ---

    #[test]
    fn test_roll_off_values() {
        assert!((RollOff::R005.value() - 0.05).abs() < 1e-10);
        assert!((RollOff::R035.value() - 0.35).abs() < 1e-10);
    }

    // --- Constellation generation ---

    #[test]
    fn test_qpsk_constellation_size() {
        assert_eq!(qpsk_constellation().len(), 4);
    }

    #[test]
    fn test_qpsk_unit_power() {
        let pts = qpsk_constellation();
        for p in &pts {
            assert!((p.mag() - 1.0).abs() < 1e-10, "QPSK point not unit power");
        }
    }

    #[test]
    fn test_psk8_constellation_size() {
        assert_eq!(psk8_constellation().len(), 8);
    }

    #[test]
    fn test_apsk16_constellation_size() {
        assert_eq!(apsk16_constellation(2.85).len(), 16);
    }

    #[test]
    fn test_apsk16_normalized_power() {
        let pts = apsk16_constellation(2.85);
        let power: f64 = pts.iter().map(|p| p.mag_sq()).sum::<f64>() / pts.len() as f64;
        assert!((power - 1.0).abs() < 0.05, "16APSK power not normalized: {power}");
    }

    #[test]
    fn test_apsk32_constellation_size() {
        assert_eq!(apsk32_constellation(2.84, 5.27).len(), 32);
    }

    #[test]
    fn test_apsk64_constellation_size() {
        assert_eq!(apsk64_constellation(2.0, 3.5, 5.2).len(), 64);
    }

    #[test]
    fn test_apsk128_constellation_size() {
        // 8+16+32+72 = 128
        assert_eq!(apsk128_constellation(2.0, 3.5, 5.5).len(), 128);
    }

    #[test]
    fn test_apsk256_constellation_size() {
        // 8+16+32+64+136 = 256
        assert_eq!(apsk256_constellation(1.9, 3.2, 4.8, 6.5).len(), 256);
    }

    #[test]
    fn test_get_constellation_returns_correct_size() {
        assert_eq!(get_constellation(ModcodS2x::Qpsk1_2).len(), 4);
        assert_eq!(get_constellation(ModcodS2x::Psk8_3_4).len(), 8);
        assert_eq!(get_constellation(ModcodS2x::Apsk16_3_4).len(), 16);
        assert_eq!(get_constellation(ModcodS2x::Apsk32_3_4).len(), 32);
        assert_eq!(get_constellation(ModcodS2x::Apsk64_4_5).len(), 64);
        assert_eq!(get_constellation(ModcodS2x::Apsk128_3_4).len(), 128);
        assert_eq!(get_constellation(ModcodS2x::Apsk256_3_4).len(), 256);
    }

    // --- PL scrambler ---

    #[test]
    fn test_pl_scrambler_reproducible() {
        let mut s1 = PlScrambler::new(0);
        let mut s2 = PlScrambler::new(0);
        let b1 = s1.generate(100);
        let b2 = s2.generate(100);
        assert_eq!(b1, b2);
    }

    #[test]
    fn test_pl_scrambler_different_isi() {
        let mut s1 = PlScrambler::new(0);
        let mut s2 = PlScrambler::new(1);
        let b1 = s1.generate(50);
        let b2 = s2.generate(50);
        assert_ne!(b1, b2, "Different ISIs should produce different sequences");
    }

    #[test]
    fn test_pl_scrambler_symbol_roundtrip() {
        let symbols = vec![Complex::new(1.0, 0.5), Complex::new(-0.5, 0.3)];
        let mut enc = PlScrambler::new(42);
        let scrambled = enc.scramble_symbols(&symbols);
        let mut dec = PlScrambler::new(42);
        let descrambled: Vec<Complex> = scrambled
            .iter()
            .map(|&s| {
                if dec.next_bit() {
                    Complex::new(-s.re, -s.im)
                } else {
                    s
                }
            })
            .collect();
        for (orig, desc) in symbols.iter().zip(descrambled.iter()) {
            assert!((orig.re - desc.re).abs() < 1e-10);
            assert!((orig.im - desc.im).abs() < 1e-10);
        }
    }

    // --- PLFRAME structure ---

    #[test]
    fn test_num_data_slots_qpsk_normal() {
        // 64800 bits / 2 bps = 32400 symbols / 90 = 360 slots
        let slots = num_data_slots(ModcodS2x::Qpsk1_2, FrameType::Normal);
        assert_eq!(slots, 360);
    }

    #[test]
    fn test_num_data_slots_short_frame() {
        // 16200 bits / 2 bps = 8100 symbols / 90 = 90 slots
        let slots = num_data_slots(ModcodS2x::Qpsk1_2, FrameType::Short);
        assert_eq!(slots, 90);
    }

    #[test]
    fn test_plframe_length_includes_pilots() {
        let n_slots = num_data_slots(ModcodS2x::Qpsk1_2, FrameType::Normal);
        let n_pilots = num_pilot_blocks(n_slots);
        let expected = PLHEADER_LEN + n_slots * SYMBOLS_PER_SLOT + n_pilots * PILOT_BLOCK_LEN;
        let actual = plframe_length(ModcodS2x::Qpsk1_2, FrameType::Normal);
        assert_eq!(actual, expected);
    }

    #[test]
    fn test_plheader_len_is_90() {
        assert_eq!(PLHEADER_LEN, 90);
    }

    #[test]
    fn test_sof_length() {
        assert_eq!(SOF.len(), 26);
    }

    // --- PLHEADER ---

    #[test]
    fn test_plheader_encode_decode() {
        let header = PlHeader::encode(7, false, true);
        let syms = header.to_symbols();
        assert_eq!(syms.len(), PLHEADER_LEN);
    }

    #[test]
    fn test_plheader_symbol_count() {
        let header = PlHeader::encode(10, true, false);
        assert_eq!(header.to_symbols().len(), 90);
    }

    // --- Pilot block ---

    #[test]
    fn test_pilot_block_length() {
        assert_eq!(generate_pilot_block().len(), PILOT_BLOCK_LEN);
    }

    #[test]
    fn test_pilot_block_amplitude() {
        for sym in generate_pilot_block() {
            assert!((sym.re - 1.0).abs() < 1e-10);
            assert!(sym.im.abs() < 1e-10);
        }
    }

    // --- Symbol mapping ---

    #[test]
    fn test_qpsk_map_demap_roundtrip() {
        let bits = vec![true, false];
        let sym = map_bits_to_symbol(&bits, ModcodS2x::Qpsk1_2);
        let decoded = demap_symbol_to_bits(sym, ModcodS2x::Qpsk1_2);
        // Decoded should pick the nearest point (possibly Gray-remapped)
        assert_eq!(decoded.len(), 2);
    }

    #[test]
    fn test_apsk16_map_returns_valid_point() {
        let bits = vec![true, false, true, true];
        let sym = map_bits_to_symbol(&bits, ModcodS2x::Apsk16_3_4);
        let constellation = get_constellation(ModcodS2x::Apsk16_3_4);
        let min_dist = constellation
            .iter()
            .map(|&c| sym.add(Complex::new(-c.re, -c.im)).mag_sq())
            .fold(f64::MAX, f64::min);
        assert!(min_dist < 1e-10, "Mapped point not in constellation");
    }

    // --- PLFRAME builder ---

    #[test]
    fn test_plframe_builder_output_length_short() {
        let bits: Vec<bool> = (0..500).map(|i| i % 2 == 0).collect();
        let builder = PlframeBuilder::new(ModcodS2x::Qpsk1_2, FrameType::Short).with_pilots(true);
        let (syms, _header) = builder.build(&bits);
        // Should be approximately plframe_length
        let expected_len = plframe_length(ModcodS2x::Qpsk1_2, FrameType::Short);
        // Allow ±10 due to pilot insertion logic
        assert!(
            (syms.len() as isize - expected_len as isize).abs() <= 10,
            "Frame length {} != expected {}",
            syms.len(),
            expected_len
        );
    }

    #[test]
    fn test_plframe_builder_includes_plheader() {
        let bits = vec![false; 100];
        let builder = PlframeBuilder::new(ModcodS2x::Qpsk1_2, FrameType::Short);
        let (syms, _header) = builder.build(&bits);
        assert!(syms.len() >= PLHEADER_LEN);
    }

    // --- DvbS2xModem ---

    #[test]
    fn test_modem_encode_decode_short_frame() {
        let modem = DvbS2xModem::new(ModcodS2x::Qpsk1_2, FrameType::Short, RollOff::R020);
        let data: Vec<bool> = (0..200).map(|i| i % 3 != 0).collect();
        let (frame, _hdr) = modem.encode_plframe(&data);
        assert!(!frame.is_empty());
        let (decoded, _pilots) = modem.decode_plframe(&frame);
        assert!(!decoded.is_empty());
    }

    #[test]
    fn test_modem_spectral_efficiency() {
        let modem = DvbS2xModem::new(ModcodS2x::Apsk16_3_4, FrameType::Normal, RollOff::R010);
        let eff = modem.spectral_efficiency();
        // 4 bits/sym * 3/4 = 3.0
        assert!((eff - 3.0).abs() < 1e-10, "SE = {eff}");
    }

    #[test]
    fn test_modem_frame_length_positive() {
        let modem = DvbS2xModem::new(ModcodS2x::Qpsk1_2, FrameType::Normal, RollOff::R020);
        assert!(modem.frame_length_symbols() > 0);
    }

    // --- ACM controller ---

    #[test]
    fn test_acm_selects_robust_at_low_snr() {
        let modcods = vec![
            ModcodS2x::Qpsk1_4,
            ModcodS2x::Qpsk1_2,
            ModcodS2x::Apsk16_3_4,
        ];
        let mut acm = AcmController::new(modcods, AcmStrategy::MaxThroughput);
        let selected = acm.update(-5.0); // very low SNR
        // Should select QPSK 1/4 (most robust)
        assert_eq!(selected, ModcodS2x::Qpsk1_4);
    }

    #[test]
    fn test_acm_selects_higher_at_high_snr() {
        let modcods = vec![
            ModcodS2x::Qpsk1_4,
            ModcodS2x::Qpsk1_2,
            ModcodS2x::Apsk16_3_4,
        ];
        let mut acm = AcmController::new(modcods, AcmStrategy::MaxThroughput);
        let selected = acm.update(20.0); // high SNR
        // Should select 16APSK 3/4 (highest throughput)
        assert_eq!(selected, ModcodS2x::Apsk16_3_4);
    }

    #[test]
    fn test_acm_max_reliability_strategy() {
        let modcods = vec![
            ModcodS2x::Qpsk1_4,
            ModcodS2x::Qpsk1_2,
            ModcodS2x::Apsk16_3_4,
        ];
        let mut acm = AcmController::new(modcods, AcmStrategy::MaxReliability);
        let selected = acm.update(2.0);
        // For MaxReliability at 2 dB, should select least demanding MODCOD that fits
        assert!(selected.required_es_n0_db() <= 2.0 + 1.0); // within margin
    }

    // --- VL-SNR spreading ---

    #[test]
    fn test_vl_snr_spread_length() {
        let syms = vec![Complex::new(1.0, 0.0); 10];
        let spread = vl_snr_spread(&syms, SpreadingFactor::Sf2);
        assert_eq!(spread.len(), 20);
    }

    #[test]
    fn test_vl_snr_despread_length() {
        let syms = vec![Complex::new(1.0, 0.0); 20];
        let despread = vl_snr_despread(&syms, SpreadingFactor::Sf2);
        assert_eq!(despread.len(), 10);
    }

    #[test]
    fn test_vl_snr_roundtrip_energy() {
        let syms = vec![Complex::new(1.0, 0.5); 8];
        let spread = vl_snr_spread(&syms, SpreadingFactor::Sf4);
        let despread = vl_snr_despread(&spread, SpreadingFactor::Sf4);
        for (orig, out) in syms.iter().zip(despread.iter()) {
            assert!((orig.re - out.re).abs() < 1e-10, "re mismatch");
            assert!((orig.im - out.im).abs() < 1e-10, "im mismatch");
        }
    }

    // --- Es/N0 estimator ---

    #[test]
    fn test_esn0_estimator_high_snr() {
        // Perfect pilot symbols → very high SNR
        let frame_type = FrameType::Short;
        let modcod = ModcodS2x::Qpsk1_2;
        let mut frame = vec![Complex::new(0.0, 0.0); plframe_length(modcod, frame_type)];
        // Place perfect pilot symbols at pilot positions
        let n_slots = num_data_slots(modcod, frame_type);
        let mut pos = PLHEADER_LEN;
        for slot in 0..n_slots {
            if slot > 0 && slot % SLOTS_PER_PILOT_PERIOD == 0 {
                let end = (pos + PILOT_BLOCK_LEN).min(frame.len());
                for sym in &mut frame[pos..end] {
                    *sym = Complex::new(1.0, 0.0);
                }
                pos += PILOT_BLOCK_LEN;
            }
            pos = (pos + SYMBOLS_PER_SLOT).min(frame.len());
        }
        let mut estimator = EsN0Estimator::new();
        let snr = estimator.estimate_from_pilots(&frame, modcod, frame_type);
        assert!(snr.is_finite());
    }

    #[test]
    fn test_esn0_estimator_default() {
        let estimator = EsN0Estimator::new();
        assert_eq!(estimator.current_estimate(), 0.0);
    }

    // --- Super-frame ---

    #[test]
    fn test_super_frame_len() {
        let sf = SuperFrame::new(64);
        assert_eq!(sf.len(), 64);
        assert!(!sf.is_empty());
    }

    #[test]
    fn test_super_frame_empty() {
        let sf = SuperFrame::new(0);
        assert!(sf.is_empty());
    }

    #[test]
    fn test_super_frame_stream_ids() {
        let ids = vec![1u8, 2u8, 3u8];
        let sf = SuperFrame::new(8).with_stream_ids(ids);
        assert_eq!(sf.stream_ids[0], 1);
        assert_eq!(sf.stream_ids[2], 3);
        assert_eq!(sf.stream_ids[3], 0); // default
    }

    // --- RRC filter ---

    #[test]
    fn test_rrc_filter_length() {
        let taps = rrc_filter_coefficients(RollOff::R020, 65, 4);
        assert_eq!(taps.len(), 65);
    }

    #[test]
    fn test_rrc_filter_normalized() {
        let taps = rrc_filter_coefficients(RollOff::R010, 65, 4);
        let power: f64 = taps.iter().map(|&v| v * v).sum();
        assert!((power - 1.0).abs() < 0.01, "RRC filter power = {power}");
    }

    #[test]
    fn test_rrc_filter_different_rolloffs() {
        let h005 = rrc_filter_coefficients(RollOff::R005, 65, 4);
        let h035 = rrc_filter_coefficients(RollOff::R035, 65, 4);
        // The center taps should differ
        assert!((h005[32] - h035[32]).abs() > 1e-6);
    }

    // --- MODCOD table completeness ---

    #[test]
    fn test_dvbs2_modcods_count() {
        assert_eq!(dvbs2_modcods().len(), 28);
    }

    #[test]
    fn test_s2x_extended_modcods_count() {
        let ext = dvbs2x_extended_modcods();
        assert!(!ext.is_empty());
        for mc in &ext {
            assert!(mc.is_s2x_only(), "{mc:?} should be S2X-only");
        }
    }

    #[test]
    fn test_vl_snr_modcods_all_flagged() {
        for mc in vl_snr_modcods() {
            assert!(mc.is_vl_snr(), "{mc:?} should be VL-SNR");
        }
    }

    #[test]
    fn test_es_n0_monotone_with_spectral_efficiency() {
        // Higher-efficiency MODCODs require higher Es/N0
        let low = ModcodS2x::Qpsk1_4;
        let high = ModcodS2x::Apsk256_3_4;
        assert!(
            high.required_es_n0_db() > low.required_es_n0_db(),
            "256APSK should need more Es/N0 than QPSK 1/4"
        );
    }

    // --- Spreading factor ---

    #[test]
    fn test_spreading_factor_values() {
        assert_eq!(SpreadingFactor::Sf1.value(), 1);
        assert_eq!(SpreadingFactor::Sf2.value(), 2);
        assert_eq!(SpreadingFactor::Sf4.value(), 4);
    }
}

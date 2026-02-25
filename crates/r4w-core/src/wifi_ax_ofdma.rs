//! 802.11ax (Wi-Fi 6) HE OFDMA Processor
//!
//! Implements IEEE 802.11ax-2021 §27 High-Efficiency (HE) OFDMA physical-layer
//! processing.  All arithmetic is pure Rust using only `std`.
//!
//! # Features
//!
//! - **OFDMA Resource Units (RU)**: 26/52/106/242/484/996/2×996-tone RUs with
//!   subcarrier index tables per IEEE 802.11ax-2021 Table 27-26.
//! - **HE Preamble**: HE-SIG-A (BSS colour, BW, MCS, GI, NSTS, coding),
//!   HE-SIG-B (RU allocation + per-user fields), HE-STF, HE-LTF.
//! - **BSS Colouring**: 6-bit colour for OBSS/PD-based spatial reuse; collision
//!   detection helper.
//! - **1024-QAM**: MCS 10 (1024-QAM 3/4) and MCS 11 (1024-QAM 5/6) constellation
//!   generation and soft demapping.
//! - **Trigger-based UL OFDMA**: Trigger frame format, user-info fields, RU
//!   assignment, UL-PPDU timing alignment.
//! - **Extended GI**: 0.8 µs / 1.6 µs / 3.2 µs guard-interval durations.
//! - **DCM**: Dual Carrier Modulation on paired subcarriers (BPSK / QPSK).
//! - **HE MCS Table**: Data-rates for all RU sizes × GI options × Nss 1–8.
//!
//! # Standards
//!
//! IEEE 802.11ax-2021 §27

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// HE OFDM symbol base data duration in seconds (12.8 µs).
pub const HE_SYMBOL_DATA_DURATION_S: f64 = 12.8e-6;

/// HE subcarrier spacing (78.125 kHz = 1/12.8 µs).
pub const HE_SUBCARRIER_SPACING_HZ: f64 = 78_125.0;

/// Maximum BSS colour value (6 bits → 0..=63, value 0 reserved for broadcasts).
pub const MAX_BSS_COLOR: u8 = 63;

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// Guard interval durations for HE PPDUs (IEEE 802.11ax-2021 §27.3.10).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GuardInterval {
    /// 0.8 µs — normal GI (mandatory for HE).
    Gi0_8us,
    /// 1.6 µs — double GI.
    Gi1_6us,
    /// 3.2 µs — quadruple GI (best multipath immunity).
    Gi3_2us,
}

impl GuardInterval {
    /// Guard-interval duration in seconds.
    pub fn duration_s(self) -> f64 {
        match self {
            GuardInterval::Gi0_8us => 0.8e-6,
            GuardInterval::Gi1_6us => 1.6e-6,
            GuardInterval::Gi3_2us => 3.2e-6,
        }
    }

    /// Total HE OFDM symbol duration (data period + GI) in seconds.
    pub fn symbol_duration_s(self) -> f64 {
        HE_SYMBOL_DATA_DURATION_S + self.duration_s()
    }

    /// 2-bit GI field encoding used in HE-SIG-A.
    pub fn field_bits(self) -> u8 {
        match self {
            GuardInterval::Gi0_8us => 0b00,
            GuardInterval::Gi1_6us => 0b01,
            GuardInterval::Gi3_2us => 0b11,
        }
    }

    /// Decode a 2-bit GI field (returns `None` for reserved value `0b10`).
    pub fn from_field_bits(bits: u8) -> Option<Self> {
        match bits & 0b11 {
            0b00 => Some(GuardInterval::Gi0_8us),
            0b01 => Some(GuardInterval::Gi1_6us),
            0b11 => Some(GuardInterval::Gi3_2us),
            _ => None,
        }
    }
}

/// Channel bandwidth classes for HE PPDUs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Bandwidth {
    /// 20 MHz — 256-point FFT.
    Bw20,
    /// 40 MHz — 512-point FFT.
    Bw40,
    /// 80 MHz — 1024-point FFT.
    Bw80,
    /// 160 MHz — 2048-point FFT.
    Bw160,
}

impl Bandwidth {
    /// Bandwidth in Hz.
    pub fn hz(self) -> f64 {
        match self {
            Bandwidth::Bw20 => 20e6,
            Bandwidth::Bw40 => 40e6,
            Bandwidth::Bw80 => 80e6,
            Bandwidth::Bw160 => 160e6,
        }
    }

    /// FFT size for this bandwidth.
    pub fn fft_size(self) -> usize {
        match self {
            Bandwidth::Bw20 => 256,
            Bandwidth::Bw40 => 512,
            Bandwidth::Bw80 => 1024,
            Bandwidth::Bw160 => 2048,
        }
    }

    /// Number of data subcarriers in a full-bandwidth 996-tone RU.
    pub fn total_data_subcarriers(self) -> usize {
        match self {
            Bandwidth::Bw20 => 234,
            Bandwidth::Bw40 => 468,
            Bandwidth::Bw80 => 980,
            Bandwidth::Bw160 => 1960,
        }
    }
}

/// HE MCS index (0–11).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Mcs(pub u8);

impl Mcs {
    /// Bits per symbol per spatial stream (= log2(modulation order) × coding rate numerator /
    /// coding rate denominator, rounded appropriately).  Used for throughput calculation.
    pub fn bits_per_subcarrier(self) -> f64 {
        // (modulation bits) × (code rate)
        match self.0 {
            0 => 1.0 * (1.0 / 2.0), // BPSK  1/2
            1 => 2.0 * (1.0 / 2.0), // QPSK  1/2
            2 => 2.0 * (3.0 / 4.0), // QPSK  3/4
            3 => 4.0 * (1.0 / 2.0), // 16-QAM 1/2
            4 => 4.0 * (3.0 / 4.0), // 16-QAM 3/4
            5 => 6.0 * (2.0 / 3.0), // 64-QAM 2/3
            6 => 6.0 * (3.0 / 4.0), // 64-QAM 3/4
            7 => 6.0 * (5.0 / 6.0), // 64-QAM 5/6
            8 => 8.0 * (3.0 / 4.0), // 256-QAM 3/4
            9 => 8.0 * (5.0 / 6.0), // 256-QAM 5/6
            10 => 10.0 * (3.0 / 4.0), // 1024-QAM 3/4
            11 => 10.0 * (5.0 / 6.0), // 1024-QAM 5/6
            _ => 0.0,
        }
    }

    /// Modulation order (number of constellation points).
    pub fn modulation_order(self) -> u32 {
        match self.0 {
            0 => 2,
            1 | 2 => 4,
            3 | 4 => 16,
            5 | 6 | 7 => 64,
            8 | 9 => 256,
            10 | 11 => 1024,
            _ => 2,
        }
    }

    /// Bits per modulation symbol (log2 of modulation order).
    pub fn bits_per_symbol(self) -> u8 {
        match self.0 {
            0 => 1,
            1 | 2 => 2,
            3 | 4 => 4,
            5 | 6 | 7 => 6,
            8 | 9 => 8,
            10 | 11 => 10,
            _ => 1,
        }
    }

    /// Is this MCS valid?
    pub fn is_valid(self) -> bool {
        self.0 <= 11
    }
}

// ---------------------------------------------------------------------------
// Resource Units
// ---------------------------------------------------------------------------

/// HE OFDMA Resource Unit type (tone count).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RuType {
    /// 26-tone RU — smallest allocation.
    Ru26,
    /// 52-tone RU.
    Ru52,
    /// 106-tone RU.
    Ru106,
    /// 242-tone RU — fills one 20 MHz channel.
    Ru242,
    /// 484-tone RU — fills one 40 MHz channel.
    Ru484,
    /// 996-tone RU — fills one 80 MHz channel.
    Ru996,
    /// 2×996-tone RU — fills a 160 MHz channel.
    Ru2x996,
}

impl RuType {
    /// Number of tones in this RU.
    pub fn tone_count(self) -> usize {
        match self {
            RuType::Ru26 => 26,
            RuType::Ru52 => 52,
            RuType::Ru106 => 106,
            RuType::Ru242 => 242,
            RuType::Ru484 => 484,
            RuType::Ru996 => 996,
            RuType::Ru2x996 => 1992,
        }
    }

    /// Number of data subcarriers in this RU (tones minus pilots).
    pub fn data_subcarriers(self) -> usize {
        match self {
            RuType::Ru26 => 24,
            RuType::Ru52 => 48,
            RuType::Ru106 => 102,
            RuType::Ru242 => 234,
            RuType::Ru484 => 468,
            RuType::Ru996 => 980,
            RuType::Ru2x996 => 1960,
        }
    }

    /// Number of pilot subcarriers in this RU.
    pub fn pilot_subcarriers(self) -> usize {
        self.tone_count() - self.data_subcarriers()
    }

    /// Maximum number of spatial streams (Nss) supported.
    pub fn max_nss(self) -> u8 {
        match self {
            RuType::Ru26 => 4,
            RuType::Ru52 => 4,
            RuType::Ru106 => 4,
            RuType::Ru242 => 8,
            RuType::Ru484 => 8,
            RuType::Ru996 => 8,
            RuType::Ru2x996 => 8,
        }
    }

    /// How many such RUs fit into a 20 MHz channel segment.
    pub fn count_per_20mhz(self) -> usize {
        match self {
            RuType::Ru26 => 9,
            RuType::Ru52 => 4,
            RuType::Ru106 => 2,
            RuType::Ru242 => 1,
            RuType::Ru484 => 0, // spans two 20 MHz segments
            RuType::Ru996 => 0,
            RuType::Ru2x996 => 0,
        }
    }
}

/// Subcarrier index range for one RU within the FFT (signed, relative to DC).
/// Positive indices are above DC; negative are below.  DC is excluded (index 0).
#[derive(Debug, Clone)]
pub struct RuSubcarrierMap {
    pub ru_type: RuType,
    /// Index of this RU within its allocation (1-based, per 802.11ax Table 27-26).
    pub ru_index: u8,
    /// Sorted list of subcarrier indices (signed, DC-centred).
    pub subcarriers: Vec<i16>,
}

impl RuSubcarrierMap {
    /// Build the subcarrier map for a 26-tone RU (20 MHz context).
    /// There are 9 possible 26-tone RUs in a 20 MHz channel.
    /// Indices per IEEE 802.11ax-2021 Table 27-26.
    pub fn ru26_in_20mhz(ru_index: u8) -> Option<Self> {
        // Subcarrier indices for 26-tone RUs in 20 MHz, ru_index 1–9.
        // Each block is 26 tones; guard bands and DC gap are excluded.
        let start: i16 = match ru_index {
            1 => -121,
            2 => -95,
            3 => -69,
            4 => -43,
            5 => -17, // guard around DC
            6 => 17,
            7 => 43,
            8 => 69,
            9 => 95,
            _ => return None,
        };
        let subcarriers: Vec<i16> = (start..start + 26)
            .filter(|&s| s != 0) // skip DC
            .collect();
        Some(Self { ru_type: RuType::Ru26, ru_index, subcarriers })
    }

    /// Build subcarrier map for a 52-tone RU in a 20 MHz channel (4 RUs).
    pub fn ru52_in_20mhz(ru_index: u8) -> Option<Self> {
        let start: i16 = match ru_index {
            1 => -121,
            2 => -69,
            3 => 17,
            4 => 69,
            _ => return None,
        };
        let subcarriers: Vec<i16> = (start..start + 52)
            .filter(|&s| s != 0 && s.abs() <= 121)
            .collect();
        Some(Self { ru_type: RuType::Ru52, ru_index, subcarriers })
    }

    /// Build subcarrier map for a 106-tone RU in a 20 MHz channel (2 RUs).
    pub fn ru106_in_20mhz(ru_index: u8) -> Option<Self> {
        let start: i16 = match ru_index {
            1 => -122,
            2 => 16,
            _ => return None,
        };
        let subcarriers: Vec<i16> = (start..start + 106)
            .filter(|&s| s != 0 && s.abs() <= 122)
            .collect();
        Some(Self { ru_type: RuType::Ru106, ru_index, subcarriers })
    }

    /// Build subcarrier map for the single 242-tone RU in a 20 MHz channel.
    pub fn ru242_in_20mhz() -> Self {
        // -121 to +121, excluding DC (0).
        let subcarriers: Vec<i16> = (-121..=121).filter(|&s| s != 0).collect();
        Self { ru_type: RuType::Ru242, ru_index: 1, subcarriers }
    }

    /// Number of data subcarriers (non-pilot) in this map.
    pub fn data_count(&self) -> usize {
        // Pilot positions depend on RU type; approximate with RuType data_subcarriers.
        self.ru_type.data_subcarriers().min(self.subcarriers.len())
    }
}

// ---------------------------------------------------------------------------
// HE-SIG-A field
// ---------------------------------------------------------------------------

/// HE-SIG-A field (2 OFDM symbols, 52-bit content) for HE SU / HE MU PPDUs.
/// Encodes BSS colour, bandwidth, MCS, GI, NSTS, and coding scheme.
#[derive(Debug, Clone)]
pub struct HeSigA {
    /// BSS colour (6 bits, 0 = broadcast/uncoloured).
    pub bss_color: u8,
    /// Channel bandwidth.
    pub bandwidth: Bandwidth,
    /// Modulation and Coding Scheme.
    pub mcs: Mcs,
    /// Guard interval.
    pub gi: GuardInterval,
    /// Number of spatial streams (1–8).
    pub nsts: u8,
    /// Coding: false = BCC, true = LDPC.
    pub ldpc_coding: bool,
    /// Is this a MU-PPDU? (affects SIG-A format).
    pub mu: bool,
}

impl HeSigA {
    /// Encode into two 26-bit words (SIG-A1, SIG-A2).
    /// Bit layout follows 802.11ax-2021 §27.3.10.9.
    pub fn encode(&self) -> (u32, u32) {
        // SIG-A1 layout (simplified):
        // [1:0] UL/DL indicator | bandwidth
        // [7:2] BSS colour (6 bits)
        // [8]   spatial reuse / OBSS_PD
        // [9]   beam-change
        // [10]  UL/DL
        // [20:11] TX-OP duration (zeroed here)
        // [21]  reserved
        // [22:23] GI + LTF size
        // [25:24] bandwidth (2 bits)

        let bw_bits: u32 = match self.bandwidth {
            Bandwidth::Bw20 => 0,
            Bandwidth::Bw40 => 1,
            Bandwidth::Bw80 => 2,
            Bandwidth::Bw160 => 3,
        };

        let sig_a1: u32 = (bw_bits & 0x3)
            | (((self.bss_color as u32) & 0x3F) << 2)
            | ((self.gi.field_bits() as u32) << 9)
            | (bw_bits << 24);

        // SIG-A2 layout:
        // [0]   LDPC extra symbol segment
        // [3:1] STBC / NSTS
        // [4]   LDPC coding
        // [12:5] MCS (4 bits, zero-padded)
        // [25:13] further fields (DCM, etc.)

        let nsts_bits: u32 = ((self.nsts.saturating_sub(1)) as u32) & 0x7;
        let sig_a2: u32 = ((self.ldpc_coding as u32) << 4)
            | (nsts_bits << 1)
            | ((self.mcs.0 as u32) << 5);

        (sig_a1, sig_a2)
    }

    /// Decode from two 26-bit words.
    pub fn decode(sig_a1: u32, sig_a2: u32) -> Self {
        let bw_bits = (sig_a1 >> 24) & 0x3;
        let bandwidth = match bw_bits {
            0 => Bandwidth::Bw20,
            1 => Bandwidth::Bw40,
            2 => Bandwidth::Bw80,
            _ => Bandwidth::Bw160,
        };

        let bss_color = ((sig_a1 >> 2) & 0x3F) as u8;
        let gi_bits = ((sig_a1 >> 9) & 0x3) as u8;
        let gi = GuardInterval::from_field_bits(gi_bits).unwrap_or(GuardInterval::Gi0_8us);

        let ldpc_coding = ((sig_a2 >> 4) & 1) != 0;
        let nsts = (((sig_a2 >> 1) & 0x7) as u8) + 1;
        let mcs_idx = ((sig_a2 >> 5) & 0xF) as u8;

        HeSigA {
            bss_color,
            bandwidth,
            mcs: Mcs(mcs_idx),
            gi,
            nsts,
            ldpc_coding,
            mu: false,
        }
    }
}

// ---------------------------------------------------------------------------
// HE-SIG-B (MU)
// ---------------------------------------------------------------------------

/// Per-user information element inside HE-SIG-B.
#[derive(Debug, Clone)]
pub struct HeSigBUser {
    /// Station association ID (11 bits, 0x7FF = unassociated/broadcast).
    pub sta_id: u16,
    /// RU allocation index (7 bits, per Table 27-26).
    pub ru_allocation: u8,
    /// MCS for this user.
    pub mcs: Mcs,
    /// Number of spatial streams (1–4 for MU).
    pub nsts: u8,
    /// Coding: false = BCC, true = LDPC.
    pub ldpc: bool,
    /// DCM enabled for this user.
    pub dcm: bool,
}

impl HeSigBUser {
    /// Pack into a 21-bit user-info field.
    pub fn pack(&self) -> u32 {
        // Non-overlapping 26-bit layout:
        // [10:0]  sta_id (11 bits)
        // [17:11] ru_allocation (7 bits)
        // [18]    dcm (1 bit)
        // [20:19] nsts-1 (2 bits)
        // [21]    ldpc (1 bit)
        // [25:22] mcs (4 bits)
        let sta: u32 = (self.sta_id as u32) & 0x7FF;
        let ru: u32 = (self.ru_allocation as u32) & 0x7F;
        let dcm: u32 = self.dcm as u32;
        let nsts: u32 = (self.nsts.saturating_sub(1) as u32) & 0x3;
        let ldpc: u32 = self.ldpc as u32;
        let mcs: u32 = (self.mcs.0 as u32) & 0xF;

        sta | (ru << 11) | (dcm << 18) | (nsts << 19) | (ldpc << 21) | (mcs << 22)
    }

    /// Unpack from a packed user-info field (same layout as `pack`).
    pub fn unpack(word: u32) -> Self {
        let sta_id = (word & 0x7FF) as u16;
        let ru_allocation = ((word >> 11) & 0x7F) as u8;
        let dcm = ((word >> 18) & 1) != 0;
        let nsts = (((word >> 19) & 0x3) as u8) + 1;
        let ldpc = ((word >> 21) & 1) != 0;
        let mcs_idx = ((word >> 22) & 0xF) as u8;

        HeSigBUser { sta_id, ru_allocation, mcs: Mcs(mcs_idx), nsts, ldpc, dcm }
    }
}

/// HE-SIG-B: MU-specific common and per-user fields.
#[derive(Debug, Clone)]
pub struct HeSigB {
    /// Channel centre frequency offset (CCFS, 1 bit in common field).
    pub ccfs: bool,
    /// List of per-user information elements.
    pub users: Vec<HeSigBUser>,
}

impl HeSigB {
    /// Serialise HE-SIG-B into a byte vector (simplified).
    pub fn serialise(&self) -> Vec<u8> {
        let mut out = Vec::new();
        // Common field (4 bytes, simplified)
        let common: u32 = self.ccfs as u32;
        out.extend_from_slice(&common.to_le_bytes());
        // User fields (3 bytes each, 21 bits packed)
        for u in &self.users {
            let w = u.pack();
            out.push((w & 0xFF) as u8);
            out.push(((w >> 8) & 0xFF) as u8);
            out.push(((w >> 16) & 0xFF) as u8);
        }
        out
    }

    /// Deserialise from bytes.
    pub fn deserialise(data: &[u8]) -> Option<Self> {
        if data.len() < 4 {
            return None;
        }
        let common = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let ccfs = (common & 1) != 0;
        let user_bytes = &data[4..];
        let mut users = Vec::new();
        let mut i = 0;
        while i + 2 < user_bytes.len() {
            let w = (user_bytes[i] as u32)
                | ((user_bytes[i + 1] as u32) << 8)
                | ((user_bytes[i + 2] as u32) << 16);
            users.push(HeSigBUser::unpack(w));
            i += 3;
        }
        Some(HeSigB { ccfs, users })
    }
}

// ---------------------------------------------------------------------------
// BSS Colouring
// ---------------------------------------------------------------------------

/// BSS colour management for OBSS/PD-based spatial reuse.
#[derive(Debug, Clone)]
pub struct BssColorManager {
    /// Current BSS colour (0 means disabled / broadcast).
    pub color: u8,
    /// Colours observed from OBSSs (collision set).
    observed: Vec<u8>,
}

impl BssColorManager {
    /// Create with a given BSS colour.
    pub fn new(color: u8) -> Self {
        BssColorManager { color: color & MAX_BSS_COLOR, observed: Vec::new() }
    }

    /// Record an observed BSS colour from a received frame.
    pub fn observe(&mut self, other_color: u8) {
        let c = other_color & MAX_BSS_COLOR;
        if c != 0 && !self.observed.contains(&c) {
            self.observed.push(c);
        }
    }

    /// Returns `true` if our colour collides with an observed colour.
    pub fn has_collision(&self) -> bool {
        self.color != 0 && self.observed.contains(&self.color)
    }

    /// Suggest the next free BSS colour not in the observed set.
    /// Returns `None` if all 63 colours are occupied.
    pub fn suggest_new_color(&self) -> Option<u8> {
        for c in 1..=MAX_BSS_COLOR {
            if !self.observed.contains(&c) {
                return Some(c);
            }
        }
        None
    }

    /// Clear the observed set (e.g. after a colour change).
    pub fn clear_observed(&mut self) {
        self.observed.clear();
    }
}

// ---------------------------------------------------------------------------
// 1024-QAM Constellation
// ---------------------------------------------------------------------------

/// 1024-QAM constellation point (normalised).
#[derive(Debug, Clone, Copy)]
pub struct Complex64 {
    pub re: f64,
    pub im: f64,
}

impl Complex64 {
    pub fn new(re: f64, im: f64) -> Self {
        Complex64 { re, im }
    }

    pub fn magnitude_sq(self) -> f64 {
        self.re * self.re + self.im * self.im
    }
}

/// Generate the full 1024-QAM Gray-coded constellation.
/// Returns a vector of 1024 complex symbols, normalised so that the
/// average power is 1.0.
pub fn generate_1024qam_constellation() -> Vec<Complex64> {
    // 1024-QAM = 32-QAM × 32-QAM on I and Q axes.
    // Levels: {±1, ±3, ±5, …, ±31}
    let levels: Vec<i32> = (0..32).map(|k| 2 * k + 1 - 32).collect(); // -31,-29,...,29,31

    let mut points = Vec::with_capacity(1024);
    for &q in &levels {
        for &i in &levels {
            points.push(Complex64::new(i as f64, q as f64));
        }
    }

    // Normalise to unit average power.
    let avg_power: f64 = points.iter().map(|p| p.magnitude_sq()).sum::<f64>()
        / points.len() as f64;
    let scale = 1.0 / avg_power.sqrt();
    for p in &mut points {
        p.re *= scale;
        p.im *= scale;
    }

    points
}

/// Map 10 bits → 1024-QAM symbol.
/// `bits` must contain exactly 10 elements.
pub fn map_1024qam(bits: &[u8]) -> Option<Complex64> {
    if bits.len() < 10 {
        return None;
    }
    let idx: usize = bits[..10].iter().enumerate().fold(0, |acc, (i, &b)| {
        acc | ((b as usize & 1) << (9 - i))
    });
    let constellation = generate_1024qam_constellation();
    constellation.get(idx).copied()
}

/// Soft-demap a received 1024-QAM symbol into 10 log-likelihood ratios.
/// Uses max-log-MAP approximation.
///
/// # Arguments
/// * `received` – received complex sample (before normalisation removal).
/// * `noise_var` – noise variance σ² per real/imag dimension.
pub fn demap_1024qam_soft(received: Complex64, noise_var: f64) -> [f64; 10] {
    let constellation = generate_1024qam_constellation();
    let scale = 2.0 / noise_var.max(1e-12);

    let mut llr = [0.0_f64; 10];
    for bit_pos in 0..10 {
        let mut max0 = f64::NEG_INFINITY;
        let mut max1 = f64::NEG_INFINITY;

        for (idx, &sym) in constellation.iter().enumerate() {
            let d2 = (received.re - sym.re).powi(2) + (received.im - sym.im).powi(2);
            let metric = -scale * d2;
            let bit_val = (idx >> (9 - bit_pos)) & 1;
            if bit_val == 0 {
                if metric > max0 {
                    max0 = metric;
                }
            } else if metric > max1 {
                max1 = metric;
            }
        }
        llr[bit_pos] = max0 - max1;
    }
    llr
}

// ---------------------------------------------------------------------------
// DCM (Dual Carrier Modulation)
// ---------------------------------------------------------------------------

/// DCM subcarrier pairing scheme (IEEE 802.11ax-2021 §27.3.10.7).
/// DCM duplicates BPSK or QPSK symbols on two separated subcarriers for
/// diversity.  Supported on MCS 0 (BPSK) and MCS 1 (QPSK) with Nss = 1.
pub struct DcmEncoder {
    /// Number of data subcarriers in the RU.
    pub data_sc_count: usize,
}

impl DcmEncoder {
    pub fn new(data_sc_count: usize) -> Self {
        DcmEncoder { data_sc_count }
    }

    /// Encode symbols with DCM: pairs are (k, k + N/2).
    /// Returns 2N output symbols for N input symbols.
    pub fn encode(&self, symbols: &[Complex64]) -> Vec<Complex64> {
        let n = self.data_sc_count / 2;
        let mut out = vec![Complex64::new(0.0, 0.0); self.data_sc_count];
        for (k, sym) in symbols.iter().take(n).enumerate() {
            out[k] = *sym;
            out[k + n] = *sym; // duplicate on paired subcarrier
        }
        out
    }

    /// Decode DCM: MRC-combine the paired subcarriers.
    pub fn decode(&self, received: &[Complex64]) -> Vec<Complex64> {
        let n = self.data_sc_count / 2;
        let mut out = Vec::with_capacity(n);
        for k in 0..n.min(received.len()) {
            if k + n < received.len() {
                // Equal-gain combining
                let re = (received[k].re + received[k + n].re) / 2.0;
                let im = (received[k].im + received[k + n].im) / 2.0;
                out.push(Complex64::new(re, im));
            } else {
                out.push(received[k]);
            }
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Trigger Frame (UL OFDMA)
// ---------------------------------------------------------------------------

/// User info field in a Trigger Frame (IEEE 802.11ax-2021 §9.3.1.23).
#[derive(Debug, Clone)]
pub struct TriggerUserInfo {
    /// Target station AID (12 bits, 0 = unassociated).
    pub aid12: u16,
    /// RU allocation (7 bits, Table 27-26 index).
    pub ru_allocation: u8,
    /// Uplink MCS (4 bits).
    pub ul_mcs: Mcs,
    /// UL target receive power in dBm (7 bits; 255 = unspecified).
    pub ul_target_rssi: u8,
    /// Number of spatial streams (3 bits, 1-based).
    pub nss: u8,
    /// Space-time block coding.
    pub stbc: bool,
    /// Coding: false = BCC, true = LDPC.
    pub ldpc: bool,
    /// DCM enabled for this user.
    pub dcm: bool,
}

impl TriggerUserInfo {
    /// Pack to a 40-bit field (5 bytes).
    pub fn pack(&self) -> [u8; 5] {
        let aid: u64 = (self.aid12 as u64) & 0xFFF;
        let ru: u64 = (self.ru_allocation as u64) & 0x7F;
        let mcs: u64 = (self.ul_mcs.0 as u64) & 0xF;
        let rssi: u64 = (self.ul_target_rssi as u64) & 0x7F;
        let nss: u64 = ((self.nss.saturating_sub(1)) as u64) & 0x7;
        let stbc: u64 = self.stbc as u64;
        let ldpc: u64 = self.ldpc as u64;
        let dcm: u64 = self.dcm as u64;

        let w: u64 = aid
            | (ru << 12)
            | (mcs << 19)
            | (rssi << 23)
            | (nss << 30)
            | (stbc << 33)
            | (ldpc << 34)
            | (dcm << 35);

        let bytes = w.to_le_bytes();
        [bytes[0], bytes[1], bytes[2], bytes[3], bytes[4]]
    }

    /// Unpack from 5 bytes.
    pub fn unpack(b: &[u8]) -> Option<Self> {
        if b.len() < 5 {
            return None;
        }
        let w: u64 = (b[0] as u64)
            | ((b[1] as u64) << 8)
            | ((b[2] as u64) << 16)
            | ((b[3] as u64) << 24)
            | ((b[4] as u64) << 32);

        Some(TriggerUserInfo {
            aid12: (w & 0xFFF) as u16,
            ru_allocation: ((w >> 12) & 0x7F) as u8,
            ul_mcs: Mcs(((w >> 19) & 0xF) as u8),
            ul_target_rssi: ((w >> 23) & 0x7F) as u8,
            nss: (((w >> 30) & 0x7) as u8) + 1,
            stbc: ((w >> 33) & 1) != 0,
            ldpc: ((w >> 34) & 1) != 0,
            dcm: ((w >> 35) & 1) != 0,
        })
    }
}

/// Trigger Frame common info (simplified subset of fields).
#[derive(Debug, Clone)]
pub struct TriggerFrame {
    /// Frame type: 0 = Basic, 1 = BFRP, 2 = MU-BAR, etc.
    pub trigger_type: u8,
    /// UL length (in units of 4 µs).
    pub ul_length: u16,
    /// More TF (1 = another trigger frame follows).
    pub more_tf: bool,
    /// Channel bandwidth for UL (2 bits).
    pub ul_bw: u8,
    /// GI and LTF type (2 bits).
    pub gi_ltf: u8,
    /// User info list.
    pub users: Vec<TriggerUserInfo>,
}

impl TriggerFrame {
    /// Serialise common info + user list into bytes.
    pub fn serialise(&self) -> Vec<u8> {
        let mut out = Vec::new();
        // Common info word (4 bytes)
        let ci: u32 = ((self.trigger_type as u32) & 0xF)
            | (((self.ul_length as u32) & 0xFFF) << 4)
            | ((self.more_tf as u32) << 16)
            | (((self.ul_bw as u32) & 0x3) << 17)
            | (((self.gi_ltf as u32) & 0x3) << 19);
        out.extend_from_slice(&ci.to_le_bytes());
        // User info fields
        for u in &self.users {
            out.extend_from_slice(&u.pack());
        }
        // Padding delimiter (20 bits = 0x1FF_FF for padding user-info field)
        out.extend_from_slice(&[0xFF, 0xFF, 0x0F, 0x00, 0x00]);
        out
    }

    /// UL PPDU timing: total UL frame duration in seconds.
    pub fn ul_ppdu_duration_s(&self) -> f64 {
        (self.ul_length as f64) * 4e-6
    }
}

// ---------------------------------------------------------------------------
// HE Preamble Builder
// ---------------------------------------------------------------------------

/// Complete HE preamble descriptor for simulation.
#[derive(Debug, Clone)]
pub struct HePreamble {
    /// L-STF duration in symbols.
    pub l_stf_symbols: u32,
    /// L-LTF duration in symbols.
    pub l_ltf_symbols: u32,
    /// L-SIG duration in symbols.
    pub l_sig_symbols: u32,
    /// RL-SIG (repeated L-SIG) symbols.
    pub rl_sig_symbols: u32,
    /// HE-SIG-A symbols (always 2).
    pub he_sig_a_symbols: u32,
    /// HE-SIG-B symbols (MU only).
    pub he_sig_b_symbols: u32,
    /// HE-STF symbols (always 1).
    pub he_stf_symbols: u32,
    /// HE-LTF symbols (1, 2, or 4 depending on Nsts).
    pub he_ltf_symbols: u32,
}

impl HePreamble {
    /// Build preamble descriptor for a given configuration.
    pub fn new(nsts: u8, mu: bool, users: usize) -> Self {
        let he_ltf = if nsts <= 2 { 2 } else { 4 };
        let he_sig_b = if mu { (users.div_ceil(8)) as u32 } else { 0 };

        HePreamble {
            l_stf_symbols: 1,
            l_ltf_symbols: 1,
            l_sig_symbols: 1,
            rl_sig_symbols: 1,
            he_sig_a_symbols: 2,
            he_sig_b_symbols: he_sig_b,
            he_stf_symbols: 1,
            he_ltf_symbols: he_ltf,
        }
    }

    /// Total preamble duration in seconds (using legacy 4 µs symbols for L-*,
    /// HE 13.6 µs symbols for HE-SIG-A/B, HE-STF/LTF).
    pub fn duration_s(&self) -> f64 {
        let legacy_sym_s = 4e-6;
        let he_sym_s = 13.6e-6; // 12.8 µs + 0.8 µs GI (HE preamble uses 0.8 µs GI)

        (self.l_stf_symbols + self.l_ltf_symbols + self.l_sig_symbols + self.rl_sig_symbols)
            as f64
            * legacy_sym_s
            + (self.he_sig_a_symbols
                + self.he_sig_b_symbols
                + self.he_stf_symbols
                + self.he_ltf_symbols) as f64
                * he_sym_s
    }
}

// ---------------------------------------------------------------------------
// HE MCS Data Rate Table
// ---------------------------------------------------------------------------

/// Entry in the HE MCS data-rate table.
#[derive(Debug, Clone, Copy)]
pub struct HeMcsEntry {
    pub mcs: u8,
    pub ru_type: RuType,
    pub gi: GuardInterval,
    pub nss: u8,
    /// Gross data rate in Mbps.
    pub data_rate_mbps: f64,
}

/// Compute the HE data rate (Mbps) for given parameters.
///
/// Formula: R = (Nss × Nsd × Nbpsc × R_c) / T_sym
/// where:
/// - Nss = number of spatial streams
/// - Nsd = data subcarriers per RU
/// - Nbpsc = bits per subcarrier (log2 modulation order)
/// - R_c = coding rate
/// - T_sym = symbol duration (data + GI)
pub fn he_data_rate_mbps(ru: RuType, mcs: Mcs, gi: GuardInterval, nss: u8) -> f64 {
    let nsd = ru.data_subcarriers() as f64;
    let bits_per_sc = mcs.bits_per_subcarrier();
    let t_sym = gi.symbol_duration_s();
    (nss as f64) * nsd * bits_per_sc / t_sym / 1e6
}

/// Build a partial HE MCS rate table for a given RU type, covering
/// MCS 0–11, GI 0.8/1.6/3.2 µs, and Nss 1–8.
pub fn build_mcs_table(ru: RuType) -> Vec<HeMcsEntry> {
    let mut table = Vec::new();
    let gis = [GuardInterval::Gi0_8us, GuardInterval::Gi1_6us, GuardInterval::Gi3_2us];
    for mcs_idx in 0..=11u8 {
        let mcs = Mcs(mcs_idx);
        if !mcs.is_valid() {
            continue;
        }
        for &gi in &gis {
            let max_nss = ru.max_nss();
            for nss in 1..=max_nss {
                let rate = he_data_rate_mbps(ru, mcs, gi, nss);
                table.push(HeMcsEntry { mcs: mcs_idx, ru_type: ru, gi, nss, data_rate_mbps: rate });
            }
        }
    }
    table
}

// ---------------------------------------------------------------------------
// OFDMA Allocator
// ---------------------------------------------------------------------------

/// An OFDMA allocation: a set of RU assignments for one PPDU.
#[derive(Debug, Clone)]
pub struct OfdmaAllocation {
    /// Channel bandwidth for this PPDU.
    pub bandwidth: Bandwidth,
    /// Per-user RU assignments.
    pub assignments: Vec<RuAssignment>,
}

/// One user's RU assignment.
#[derive(Debug, Clone)]
pub struct RuAssignment {
    /// Station AID.
    pub sta_id: u16,
    /// RU type assigned.
    pub ru_type: RuType,
    /// RU index within the channel.
    pub ru_index: u8,
    /// MCS for this user.
    pub mcs: Mcs,
    /// Number of spatial streams.
    pub nss: u8,
}

impl OfdmaAllocation {
    /// Construct a new allocation for the given bandwidth.
    pub fn new(bandwidth: Bandwidth) -> Self {
        OfdmaAllocation { bandwidth, assignments: Vec::new() }
    }

    /// Add a 26-tone RU assignment.  Returns `Err` if the RU index is invalid
    /// for the bandwidth or already occupied.
    pub fn add_ru26(&mut self, sta_id: u16, ru_index: u8, mcs: Mcs, nss: u8) -> Result<(), &'static str> {
        let max_idx = match self.bandwidth {
            Bandwidth::Bw20 => 9,
            Bandwidth::Bw40 => 18,
            Bandwidth::Bw80 => 37,
            Bandwidth::Bw160 => 74,
        };
        if ru_index < 1 || ru_index > max_idx {
            return Err("RU index out of range");
        }
        if self.assignments.iter().any(|a| a.ru_type == RuType::Ru26 && a.ru_index == ru_index) {
            return Err("RU already occupied");
        }
        self.assignments.push(RuAssignment {
            sta_id,
            ru_type: RuType::Ru26,
            ru_index,
            mcs,
            nss,
        });
        Ok(())
    }

    /// Total number of data subcarriers allocated across all users.
    pub fn total_data_subcarriers(&self) -> usize {
        self.assignments.iter().map(|a| a.ru_type.data_subcarriers()).sum()
    }

    /// Aggregate throughput estimate in Mbps.
    pub fn aggregate_throughput_mbps(&self, gi: GuardInterval) -> f64 {
        self.assignments
            .iter()
            .map(|a| he_data_rate_mbps(a.ru_type, a.mcs, gi, a.nss))
            .sum()
    }
}

// ---------------------------------------------------------------------------
// HE LTF generation (simplified)
// ---------------------------------------------------------------------------

/// HE-LTF OFDM symbol generator.
/// Uses a simplified P matrix for educational purposes.
pub struct HeLtfGenerator {
    pub ru_type: RuType,
    pub n_ltf: u8, // 1, 2, or 4
}

impl HeLtfGenerator {
    pub fn new(ru_type: RuType, nsts: u8) -> Self {
        let n_ltf = if nsts <= 2 { 2 } else { 4 };
        HeLtfGenerator { ru_type, n_ltf }
    }

    /// Generate HE-LTF sequence values (±1) for one OFDM symbol over the RU.
    /// In practice, these are BPSK-modulated using a PN sequence; here we use
    /// a deterministic pattern for testing.
    pub fn sequence(&self) -> Vec<i8> {
        let n = self.ru_type.data_subcarriers();
        // Simple alternating PN: +1 for even index, -1 for odd.
        (0..n).map(|k| if k % 2 == 0 { 1i8 } else { -1i8 }).collect()
    }
}

// ---------------------------------------------------------------------------
// UL PPDU Assembler
// ---------------------------------------------------------------------------

/// Assemble a UL HE TB PPDU (Trigger-Based) with OFDMA.
/// Returns a byte vector representing the serialised PPDU structure.
pub fn assemble_ul_he_tb_ppdu(
    trigger: &TriggerFrame,
    user_idx: usize,
    payload: &[u8],
) -> Option<Vec<u8>> {
    let user = trigger.users.get(user_idx)?;

    // Preamble + payload mock assembly.
    let mut ppdu = Vec::new();

    // L-STF (8 bytes placeholder)
    ppdu.extend_from_slice(&[0xAA; 8]);
    // L-LTF (8 bytes)
    ppdu.extend_from_slice(&[0x55; 8]);
    // L-SIG (4 bytes)
    ppdu.extend_from_slice(&[0x00, 0x00, 0x00, 0x00]);
    // RL-SIG (4 bytes)
    ppdu.extend_from_slice(&[0x00, 0x00, 0x00, 0x00]);

    // HE-SIG-A (8 bytes, 2 × 4-byte words)
    let sig_a1: u32 = (user.ru_allocation as u32) << 2;
    let sig_a2: u32 = (user.ul_mcs.0 as u32) | ((user.nss.saturating_sub(1) as u32) << 4);
    ppdu.extend_from_slice(&sig_a1.to_le_bytes());
    ppdu.extend_from_slice(&sig_a2.to_le_bytes());

    // HE-STF (8 bytes)
    ppdu.extend_from_slice(&[0xAA; 8]);
    // HE-LTF (16 bytes × 2)
    ppdu.extend_from_slice(&[0x55; 32]);

    // Payload (padded/truncated to UL length)
    let ul_bytes =
        (trigger.ul_ppdu_duration_s() * 6.0e6 / 8.0) as usize; // rough estimate
    let padded_len = ul_bytes.max(payload.len());
    ppdu.extend_from_slice(&payload[..payload.len().min(padded_len)]);
    ppdu.extend(std::iter::repeat(0u8).take(padded_len.saturating_sub(payload.len())));

    Some(ppdu)
}

// ---------------------------------------------------------------------------
// Spectrum mask helper
// ---------------------------------------------------------------------------

/// Check that a set of RU assignments does not exceed the spectral occupancy
/// of the indicated bandwidth (simplified occupancy check).
pub fn check_spectral_occupancy(alloc: &OfdmaAllocation) -> bool {
    let total: usize = alloc.assignments.iter().map(|a| a.ru_type.tone_count()).sum();
    let available = alloc.bandwidth.total_data_subcarriers();
    total <= available
}

// ---------------------------------------------------------------------------
// FFT helpers (radix-2 DIT, in-place)
// ---------------------------------------------------------------------------

/// In-place radix-2 Cooley-Tukey FFT (decimation-in-time).
/// `buf` length must be a power of two.  Forward transform when `forward = true`.
pub fn fft_inplace(buf: &mut [Complex64], forward: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT size must be a power of two");

    // Bit-reversal permutation.
    let bits = n.trailing_zeros() as usize;
    for i in 0..n {
        let j = reverse_bits(i, bits);
        if j > i {
            buf.swap(i, j);
        }
    }

    // Butterfly passes.
    let sign = if forward { -1.0_f64 } else { 1.0_f64 };
    let mut len = 2;
    while len <= n {
        let ang = sign * 2.0 * PI / (len as f64);
        let w_re = ang.cos();
        let w_im = ang.sin();
        for i in (0..n).step_by(len) {
            let (mut twr, mut twi) = (1.0_f64, 0.0_f64);
            for j in 0..len / 2 {
                let ur = buf[i + j].re;
                let ui = buf[i + j].im;
                let vr = buf[i + j + len / 2].re * twr - buf[i + j + len / 2].im * twi;
                let vi = buf[i + j + len / 2].re * twi + buf[i + j + len / 2].im * twr;
                buf[i + j] = Complex64::new(ur + vr, ui + vi);
                buf[i + j + len / 2] = Complex64::new(ur - vr, ui - vi);
                let new_twr = twr * w_re - twi * w_im;
                twi = twr * w_im + twi * w_re;
                twr = new_twr;
            }
        }
        len <<= 1;
    }

    if !forward {
        let nf = n as f64;
        for x in buf.iter_mut() {
            x.re /= nf;
            x.im /= nf;
        }
    }
}

fn reverse_bits(mut x: usize, bits: usize) -> usize {
    let mut result = 0;
    for _ in 0..bits {
        result = (result << 1) | (x & 1);
        x >>= 1;
    }
    result
}

// ---------------------------------------------------------------------------
// OFDMA IFFT modulator
// ---------------------------------------------------------------------------

/// Modulate one HE OFDMA symbol: map complex data onto RU subcarriers and
/// perform IFFT.
///
/// # Arguments
/// * `fft_size` – FFT/IFFT size (must be a power of two).
/// * `ru_map`   – subcarrier index map for the RU.
/// * `data`     – frequency-domain data symbols for each subcarrier in the RU.
///
/// Returns the time-domain samples (IFFT output, without CP).
pub fn modulate_ofdma_symbol(
    fft_size: usize,
    ru_map: &RuSubcarrierMap,
    data: &[Complex64],
) -> Vec<Complex64> {
    assert!(fft_size.is_power_of_two());
    let mut freq = vec![Complex64::new(0.0, 0.0); fft_size];

    for (sc_data, &sc_idx) in data.iter().zip(ru_map.subcarriers.iter()) {
        // Convert signed DC-centred index to FFT bin index.
        let bin = if sc_idx >= 0 {
            sc_idx as usize
        } else {
            (fft_size as i64 + sc_idx as i64) as usize
        };
        if bin < fft_size {
            freq[bin] = *sc_data;
        }
    }

    fft_inplace(&mut freq, false); // IFFT
    freq
}

/// Add a cyclic prefix of `cp_len` samples to a time-domain symbol.
pub fn add_cyclic_prefix(symbol: &[Complex64], cp_len: usize) -> Vec<Complex64> {
    let n = symbol.len();
    let mut out = Vec::with_capacity(n + cp_len);
    out.extend_from_slice(&symbol[n.saturating_sub(cp_len)..]);
    out.extend_from_slice(symbol);
    out
}

/// Remove a cyclic prefix of `cp_len` samples.
pub fn remove_cyclic_prefix(frame: &[Complex64], cp_len: usize) -> &[Complex64] {
    if frame.len() > cp_len { &frame[cp_len..] } else { frame }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Guard Interval tests
    #[test]
    fn gi_duration_values() {
        assert!((GuardInterval::Gi0_8us.duration_s() - 0.8e-6).abs() < 1e-12);
        assert!((GuardInterval::Gi1_6us.duration_s() - 1.6e-6).abs() < 1e-12);
        assert!((GuardInterval::Gi3_2us.duration_s() - 3.2e-6).abs() < 1e-12);
    }

    #[test]
    fn gi_symbol_duration() {
        let t = GuardInterval::Gi0_8us.symbol_duration_s();
        assert!((t - 13.6e-6).abs() < 1e-12);
        let t2 = GuardInterval::Gi3_2us.symbol_duration_s();
        assert!((t2 - 16.0e-6).abs() < 1e-12);
    }

    #[test]
    fn gi_field_bits_roundtrip() {
        for &gi in &[GuardInterval::Gi0_8us, GuardInterval::Gi1_6us, GuardInterval::Gi3_2us] {
            let bits = gi.field_bits();
            let decoded = GuardInterval::from_field_bits(bits).expect("valid bits");
            assert_eq!(decoded, gi);
        }
    }

    #[test]
    fn gi_reserved_bits_returns_none() {
        assert!(GuardInterval::from_field_bits(0b10).is_none());
    }

    // Bandwidth tests
    #[test]
    fn bandwidth_fft_sizes() {
        assert_eq!(Bandwidth::Bw20.fft_size(), 256);
        assert_eq!(Bandwidth::Bw40.fft_size(), 512);
        assert_eq!(Bandwidth::Bw80.fft_size(), 1024);
        assert_eq!(Bandwidth::Bw160.fft_size(), 2048);
    }

    #[test]
    fn bandwidth_hz_values() {
        assert!((Bandwidth::Bw20.hz() - 20e6).abs() < 1.0);
        assert!((Bandwidth::Bw160.hz() - 160e6).abs() < 1.0);
    }

    // MCS tests
    #[test]
    fn mcs_validity() {
        for i in 0..=11 {
            assert!(Mcs(i).is_valid());
        }
        assert!(!Mcs(12).is_valid());
    }

    #[test]
    fn mcs_modulation_orders() {
        assert_eq!(Mcs(0).modulation_order(), 2);    // BPSK
        assert_eq!(Mcs(2).modulation_order(), 4);    // QPSK
        assert_eq!(Mcs(4).modulation_order(), 16);   // 16-QAM
        assert_eq!(Mcs(7).modulation_order(), 64);   // 64-QAM
        assert_eq!(Mcs(9).modulation_order(), 256);  // 256-QAM
        assert_eq!(Mcs(11).modulation_order(), 1024); // 1024-QAM
    }

    #[test]
    fn mcs_bits_per_symbol() {
        assert_eq!(Mcs(0).bits_per_symbol(), 1);
        assert_eq!(Mcs(10).bits_per_symbol(), 10);
        assert_eq!(Mcs(11).bits_per_symbol(), 10);
    }

    #[test]
    fn mcs_bits_per_subcarrier_monotone() {
        // bits_per_subcarrier should generally increase with MCS index.
        for i in 0..11u8 {
            assert!(Mcs(i).bits_per_subcarrier() <= Mcs(i + 1).bits_per_subcarrier() + 0.01,
                    "MCS {} bpc {} > MCS {} bpc {}",
                    i, Mcs(i).bits_per_subcarrier(), i+1, Mcs(i+1).bits_per_subcarrier());
        }
    }

    // RU type tests
    #[test]
    fn ru_tone_counts() {
        assert_eq!(RuType::Ru26.tone_count(), 26);
        assert_eq!(RuType::Ru52.tone_count(), 52);
        assert_eq!(RuType::Ru106.tone_count(), 106);
        assert_eq!(RuType::Ru242.tone_count(), 242);
        assert_eq!(RuType::Ru484.tone_count(), 484);
        assert_eq!(RuType::Ru996.tone_count(), 996);
        assert_eq!(RuType::Ru2x996.tone_count(), 1992);
    }

    #[test]
    fn ru_data_subcarriers_less_than_tone_count() {
        for &ru in &[RuType::Ru26, RuType::Ru52, RuType::Ru106, RuType::Ru242,
                     RuType::Ru484, RuType::Ru996, RuType::Ru2x996] {
            assert!(ru.data_subcarriers() < ru.tone_count(),
                    "{:?}: data_sc {} >= tones {}", ru, ru.data_subcarriers(), ru.tone_count());
        }
    }

    #[test]
    fn ru_pilot_counts_correct() {
        assert_eq!(RuType::Ru26.pilot_subcarriers(), 2);
        assert_eq!(RuType::Ru52.pilot_subcarriers(), 4);
        assert_eq!(RuType::Ru242.pilot_subcarriers(), 8);
    }

    #[test]
    fn ru26_subcarrier_map_count() {
        let map = RuSubcarrierMap::ru26_in_20mhz(1).expect("valid");
        assert!(map.subcarriers.len() <= 26);
        assert!(map.subcarriers.len() >= 24);
    }

    #[test]
    fn ru26_invalid_index() {
        assert!(RuSubcarrierMap::ru26_in_20mhz(0).is_none());
        assert!(RuSubcarrierMap::ru26_in_20mhz(10).is_none());
    }

    #[test]
    fn ru242_covers_full_20mhz() {
        let map = RuSubcarrierMap::ru242_in_20mhz();
        // Should have 241 subcarriers (−121 to +121, skip 0).
        assert_eq!(map.subcarriers.len(), 242); // −121..=121 is 243 values, minus DC = 242
    }

    #[test]
    fn ru52_index_coverage() {
        for idx in 1..=4u8 {
            let map = RuSubcarrierMap::ru52_in_20mhz(idx).expect("valid index");
            assert!(!map.subcarriers.is_empty());
        }
        assert!(RuSubcarrierMap::ru52_in_20mhz(5).is_none());
    }

    // HE-SIG-A tests
    #[test]
    fn he_sig_a_roundtrip() {
        let sig = HeSigA {
            bss_color: 42,
            bandwidth: Bandwidth::Bw80,
            mcs: Mcs(7),
            gi: GuardInterval::Gi1_6us,
            nsts: 2,
            ldpc_coding: true,
            mu: false,
        };
        let (a1, a2) = sig.encode();
        let decoded = HeSigA::decode(a1, a2);
        assert_eq!(decoded.bss_color, 42);
        assert_eq!(decoded.bandwidth, Bandwidth::Bw80);
        assert_eq!(decoded.gi, GuardInterval::Gi1_6us);
        assert!(decoded.ldpc_coding);
    }

    #[test]
    fn he_sig_a_bss_color_preserved() {
        for color in [0u8, 1, 31, 63] {
            let sig = HeSigA {
                bss_color: color,
                bandwidth: Bandwidth::Bw20,
                mcs: Mcs(5),
                gi: GuardInterval::Gi0_8us,
                nsts: 1,
                ldpc_coding: false,
                mu: false,
            };
            let (a1, a2) = sig.encode();
            let dec = HeSigA::decode(a1, a2);
            assert_eq!(dec.bss_color, color, "color {color} not preserved");
        }
    }

    // HE-SIG-B tests
    #[test]
    fn he_sig_b_user_pack_unpack() {
        let u = HeSigBUser {
            sta_id: 0x1AB,
            ru_allocation: 5,
            mcs: Mcs(9),
            nsts: 2,
            ldpc: true,
            dcm: false,
        };
        let packed = u.pack();
        let u2 = HeSigBUser::unpack(packed);
        assert_eq!(u2.sta_id, u.sta_id);
        assert_eq!(u2.ru_allocation, u.ru_allocation);
        assert_eq!(u2.mcs.0, u.mcs.0);
        assert_eq!(u2.ldpc, u.ldpc);
    }

    #[test]
    fn he_sig_b_serialise_deserialise() {
        let b = HeSigB {
            ccfs: true,
            users: vec![
                HeSigBUser { sta_id: 10, ru_allocation: 3, mcs: Mcs(4), nsts: 1, ldpc: false, dcm: false },
                HeSigBUser { sta_id: 20, ru_allocation: 7, mcs: Mcs(8), nsts: 2, ldpc: true, dcm: false },
            ],
        };
        let bytes = b.serialise();
        let b2 = HeSigB::deserialise(&bytes).expect("valid");
        assert_eq!(b2.ccfs, b.ccfs);
        assert_eq!(b2.users.len(), 2);
        assert_eq!(b2.users[0].sta_id, 10);
        assert_eq!(b2.users[1].sta_id, 20);
    }

    // BSS Colour tests
    #[test]
    fn bss_color_no_collision_initially() {
        let mgr = BssColorManager::new(5);
        assert!(!mgr.has_collision());
    }

    #[test]
    fn bss_color_detects_collision() {
        let mut mgr = BssColorManager::new(7);
        mgr.observe(7);
        assert!(mgr.has_collision());
    }

    #[test]
    fn bss_color_observe_different() {
        let mut mgr = BssColorManager::new(7);
        mgr.observe(8);
        assert!(!mgr.has_collision());
    }

    #[test]
    fn bss_color_suggest_new() {
        let mut mgr = BssColorManager::new(1);
        // Observe colours 1..10
        for c in 1..=10u8 {
            mgr.observe(c);
        }
        let suggestion = mgr.suggest_new_color().expect("should find free colour");
        assert!(suggestion > 10 && suggestion <= MAX_BSS_COLOR);
    }

    #[test]
    fn bss_color_suggest_when_all_taken() {
        let mut mgr = BssColorManager::new(1);
        for c in 1..=MAX_BSS_COLOR {
            mgr.observe(c);
        }
        assert!(mgr.suggest_new_color().is_none());
    }

    #[test]
    fn bss_color_clear_observed() {
        let mut mgr = BssColorManager::new(5);
        mgr.observe(5);
        assert!(mgr.has_collision());
        mgr.clear_observed();
        assert!(!mgr.has_collision());
    }

    // 1024-QAM tests
    #[test]
    fn constellation_1024qam_size() {
        let c = generate_1024qam_constellation();
        assert_eq!(c.len(), 1024);
    }

    #[test]
    fn constellation_1024qam_unit_power() {
        let c = generate_1024qam_constellation();
        let avg_pwr: f64 = c.iter().map(|p| p.magnitude_sq()).sum::<f64>() / c.len() as f64;
        assert!((avg_pwr - 1.0).abs() < 1e-9, "avg power = {avg_pwr}");
    }

    #[test]
    fn map_1024qam_valid() {
        let bits = [1u8, 0, 1, 0, 1, 0, 1, 0, 1, 0];
        let sym = map_1024qam(&bits);
        assert!(sym.is_some());
    }

    #[test]
    fn map_1024qam_too_short() {
        let bits = [1u8; 9];
        assert!(map_1024qam(&bits).is_none());
    }

    #[test]
    fn demap_1024qam_soft_length() {
        let recv = Complex64::new(0.1, 0.1);
        let llr = demap_1024qam_soft(recv, 0.1);
        assert_eq!(llr.len(), 10);
    }

    #[test]
    fn demap_1024qam_finite_llr() {
        let recv = Complex64::new(0.5, -0.5);
        let llr = demap_1024qam_soft(recv, 0.05);
        for &l in &llr {
            assert!(l.is_finite(), "LLR is not finite: {l}");
        }
    }

    // DCM tests
    #[test]
    fn dcm_encode_decode_roundtrip() {
        let enc = DcmEncoder::new(48); // 52-tone RU data subcarriers
        let symbols: Vec<Complex64> = (0..24)
            .map(|i| Complex64::new(if i % 2 == 0 { 1.0 } else { -1.0 }, 0.0))
            .collect();
        let encoded = enc.encode(&symbols);
        assert_eq!(encoded.len(), 48);
        let decoded = enc.decode(&encoded);
        assert_eq!(decoded.len(), 24);
        for (a, b) in symbols.iter().zip(decoded.iter()) {
            assert!((a.re - b.re).abs() < 1e-10);
        }
    }

    #[test]
    fn dcm_paired_subcarriers_equal() {
        let enc = DcmEncoder::new(24);
        let sym = vec![Complex64::new(0.7, -0.3)];
        let encoded = enc.encode(&sym);
        // First and second half should match.
        assert!((encoded[0].re - encoded[12].re).abs() < 1e-10);
    }

    // Trigger Frame tests
    #[test]
    fn trigger_user_info_pack_unpack() {
        let u = TriggerUserInfo {
            aid12: 0xABC,
            ru_allocation: 0x1F,
            ul_mcs: Mcs(5),
            ul_target_rssi: 70,
            nss: 2,
            stbc: false,
            ldpc: true,
            dcm: false,
        };
        let packed = u.pack();
        let u2 = TriggerUserInfo::unpack(&packed).expect("valid");
        assert_eq!(u2.aid12, u.aid12);
        assert_eq!(u2.ru_allocation, u.ru_allocation);
        assert_eq!(u2.ul_mcs.0, u.ul_mcs.0);
        assert_eq!(u2.nss, u.nss);
        assert_eq!(u2.ldpc, u.ldpc);
    }

    #[test]
    fn trigger_frame_serialise_nonzero() {
        let tf = TriggerFrame {
            trigger_type: 0,
            ul_length: 100,
            more_tf: false,
            ul_bw: 1,
            gi_ltf: 0,
            users: vec![TriggerUserInfo {
                aid12: 1,
                ru_allocation: 3,
                ul_mcs: Mcs(3),
                ul_target_rssi: 60,
                nss: 1,
                stbc: false,
                ldpc: false,
                dcm: false,
            }],
        };
        let bytes = tf.serialise();
        assert!(!bytes.is_empty());
    }

    #[test]
    fn trigger_frame_ul_duration() {
        let tf = TriggerFrame {
            trigger_type: 0,
            ul_length: 500,
            more_tf: false,
            ul_bw: 0,
            gi_ltf: 0,
            users: Vec::new(),
        };
        let dur = tf.ul_ppdu_duration_s();
        assert!((dur - 500.0 * 4e-6).abs() < 1e-12);
    }

    // HE Preamble tests
    #[test]
    fn he_preamble_ltf_count_su() {
        let p = HePreamble::new(1, false, 1);
        assert_eq!(p.he_ltf_symbols, 2);
    }

    #[test]
    fn he_preamble_ltf_count_3nsts() {
        let p = HePreamble::new(3, false, 1);
        assert_eq!(p.he_ltf_symbols, 4);
    }

    #[test]
    fn he_preamble_mu_sig_b_nonzero() {
        let p = HePreamble::new(1, true, 8);
        assert_eq!(p.he_sig_b_symbols, 1);
    }

    #[test]
    fn he_preamble_duration_positive() {
        let p = HePreamble::new(2, false, 1);
        let d = p.duration_s();
        assert!(d > 0.0);
        assert!(d < 1e-3); // should be a few tens of microseconds
    }

    // MCS data rate table tests
    #[test]
    fn mcs_table_rate_positive() {
        let rate = he_data_rate_mbps(RuType::Ru242, Mcs(11), GuardInterval::Gi0_8us, 1);
        assert!(rate > 0.0, "rate = {rate}");
    }

    #[test]
    fn mcs_table_rate_increases_with_nss() {
        let r1 = he_data_rate_mbps(RuType::Ru242, Mcs(9), GuardInterval::Gi0_8us, 1);
        let r4 = he_data_rate_mbps(RuType::Ru242, Mcs(9), GuardInterval::Gi0_8us, 4);
        assert!(r4 > r1);
    }

    #[test]
    fn mcs_table_rate_decreases_with_gi() {
        let r_short = he_data_rate_mbps(RuType::Ru242, Mcs(9), GuardInterval::Gi0_8us, 1);
        let r_long = he_data_rate_mbps(RuType::Ru242, Mcs(9), GuardInterval::Gi3_2us, 1);
        assert!(r_short > r_long);
    }

    #[test]
    fn build_mcs_table_nonempty() {
        let tbl = build_mcs_table(RuType::Ru242);
        assert!(!tbl.is_empty());
    }

    #[test]
    fn build_mcs_table_covers_all_mcs() {
        let tbl = build_mcs_table(RuType::Ru242);
        for mcs_idx in 0..=11u8 {
            assert!(tbl.iter().any(|e| e.mcs == mcs_idx),
                    "MCS {mcs_idx} missing from table");
        }
    }

    // OFDMA Allocator tests
    #[test]
    fn ofdma_alloc_add_ru26_valid() {
        let mut alloc = OfdmaAllocation::new(Bandwidth::Bw20);
        assert!(alloc.add_ru26(1, 1, Mcs(5), 1).is_ok());
        assert!(alloc.add_ru26(2, 2, Mcs(7), 2).is_ok());
    }

    #[test]
    fn ofdma_alloc_duplicate_ru_rejected() {
        let mut alloc = OfdmaAllocation::new(Bandwidth::Bw20);
        alloc.add_ru26(1, 1, Mcs(5), 1).unwrap();
        assert!(alloc.add_ru26(2, 1, Mcs(5), 1).is_err());
    }

    #[test]
    fn ofdma_alloc_invalid_index_rejected() {
        let mut alloc = OfdmaAllocation::new(Bandwidth::Bw20);
        assert!(alloc.add_ru26(1, 0, Mcs(5), 1).is_err()); // 0-indexed
        assert!(alloc.add_ru26(1, 10, Mcs(5), 1).is_err()); // > 9
    }

    #[test]
    fn ofdma_alloc_total_data_sc() {
        let mut alloc = OfdmaAllocation::new(Bandwidth::Bw20);
        alloc.add_ru26(1, 1, Mcs(5), 1).unwrap();
        alloc.add_ru26(2, 2, Mcs(5), 1).unwrap();
        assert_eq!(alloc.total_data_subcarriers(), 2 * RuType::Ru26.data_subcarriers());
    }

    #[test]
    fn ofdma_alloc_aggregate_throughput_positive() {
        let mut alloc = OfdmaAllocation::new(Bandwidth::Bw20);
        alloc.add_ru26(1, 1, Mcs(9), 2).unwrap();
        let tp = alloc.aggregate_throughput_mbps(GuardInterval::Gi0_8us);
        assert!(tp > 0.0);
    }

    // Spectral occupancy test
    #[test]
    fn spectral_occupancy_check() {
        let mut alloc = OfdmaAllocation::new(Bandwidth::Bw20);
        for idx in 1..=9u8 {
            alloc.add_ru26(idx as u16, idx, Mcs(5), 1).unwrap();
        }
        // 9 × 26 = 234 tones; available = 234 → should pass.
        assert!(check_spectral_occupancy(&alloc));
    }

    // FFT tests
    #[test]
    fn fft_identity() {
        let n = 16;
        let mut buf: Vec<Complex64> = (0..n)
            .map(|k| Complex64::new((k as f64).sin(), 0.0))
            .collect();
        let orig = buf.clone();

        fft_inplace(&mut buf, true);
        fft_inplace(&mut buf, false); // inverse → should recover original

        for (a, b) in orig.iter().zip(buf.iter()) {
            assert!((a.re - b.re).abs() < 1e-9, "re mismatch");
            assert!((a.im - b.im).abs() < 1e-9, "im mismatch");
        }
    }

    #[test]
    fn fft_dc_tone() {
        // All ones → FFT should give a spike at bin 0 with magnitude N.
        let n = 8;
        let mut buf: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); n];
        fft_inplace(&mut buf, true);
        assert!((buf[0].re - n as f64).abs() < 1e-9);
        for k in 1..n {
            assert!(buf[k].magnitude_sq().sqrt() < 1e-9);
        }
    }

    // OFDMA modulator tests
    #[test]
    fn modulate_ofdma_symbol_length() {
        let map = RuSubcarrierMap::ru26_in_20mhz(1).expect("valid");
        let fft_size = 256;
        let data: Vec<Complex64> = map.subcarriers.iter()
            .map(|_| Complex64::new(1.0, 0.0))
            .collect();
        let time = modulate_ofdma_symbol(fft_size, &map, &data);
        assert_eq!(time.len(), fft_size);
    }

    #[test]
    fn cyclic_prefix_adds_samples() {
        let sym: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 64];
        let cp = add_cyclic_prefix(&sym, 16);
        assert_eq!(cp.len(), 80);
        // First 16 samples should match last 16 of original.
        for k in 0..16 {
            assert!((cp[k].re - sym[48 + k].re).abs() < 1e-10);
        }
    }

    #[test]
    fn cyclic_prefix_remove() {
        let sym: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 64];
        let with_cp = add_cyclic_prefix(&sym, 16);
        let stripped = remove_cyclic_prefix(&with_cp, 16);
        assert_eq!(stripped.len(), 64);
    }

    // UL PPDU assembler test
    #[test]
    fn assemble_ul_ppdu_valid_user() {
        let tf = TriggerFrame {
            trigger_type: 0,
            ul_length: 100,
            more_tf: false,
            ul_bw: 0,
            gi_ltf: 0,
            users: vec![TriggerUserInfo {
                aid12: 5,
                ru_allocation: 1,
                ul_mcs: Mcs(4),
                ul_target_rssi: 60,
                nss: 1,
                stbc: false,
                ldpc: false,
                dcm: false,
            }],
        };
        let ppdu = assemble_ul_he_tb_ppdu(&tf, 0, b"Hello HE TB PPDU");
        assert!(ppdu.is_some());
        assert!(!ppdu.unwrap().is_empty());
    }

    #[test]
    fn assemble_ul_ppdu_invalid_user_index() {
        let tf = TriggerFrame {
            trigger_type: 0,
            ul_length: 50,
            more_tf: false,
            ul_bw: 0,
            gi_ltf: 0,
            users: Vec::new(),
        };
        assert!(assemble_ul_he_tb_ppdu(&tf, 0, b"test").is_none());
    }

    // HE-LTF generator test
    #[test]
    fn he_ltf_sequence_length_matches_ru() {
        let gen = HeLtfGenerator::new(RuType::Ru52, 2);
        let seq = gen.sequence();
        assert_eq!(seq.len(), RuType::Ru52.data_subcarriers());
    }

    #[test]
    fn he_ltf_nss_determines_n_ltf() {
        assert_eq!(HeLtfGenerator::new(RuType::Ru26, 1).n_ltf, 2);
        assert_eq!(HeLtfGenerator::new(RuType::Ru26, 2).n_ltf, 2);
        assert_eq!(HeLtfGenerator::new(RuType::Ru26, 3).n_ltf, 4);
        assert_eq!(HeLtfGenerator::new(RuType::Ru26, 4).n_ltf, 4);
    }

    // HE subcarrier spacing sanity
    #[test]
    fn he_subcarrier_spacing_correct() {
        let spacing = 1.0 / HE_SYMBOL_DATA_DURATION_S;
        assert!((spacing - HE_SUBCARRIER_SPACING_HZ).abs() < 1.0);
    }

    // Edge: zero-noise soft demap
    #[test]
    fn demap_1024qam_low_noise_large_llr() {
        let c = generate_1024qam_constellation();
        // Receive the first constellation point exactly.
        let sym = c[0];
        let llr = demap_1024qam_soft(sym, 1e-6);
        // All LLRs should be finite.
        assert!(llr.iter().all(|l| l.is_finite()));
    }
}

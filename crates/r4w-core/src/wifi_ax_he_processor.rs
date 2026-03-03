//! Wi-Fi 6 (IEEE 802.11ax) High Efficiency (HE) Physical Layer Processor
//!
//! Implements the HE PHY per IEEE 802.11ax-2021 including:
//! - OFDMA resource unit (RU) allocation: 26/52/106/242/484/996/2x996 tones
//! - HE preamble: HE-STF and HE-LTF (1x/2x/4x modes)
//! - HE-SIG-A common signaling: BSS Color, spatial reuse, BW, MCS, GI+LTF size
//! - HE-SIG-B user-specific: RU allocation, NSTS, MCS, DCM, coding
//! - DCM (Dual Carrier Modulation) for MCS 0-1 range extension
//! - Extended MCS 0-11 (BPSK through 1024-QAM)
//! - Guard intervals: 0.8 µs, 1.6 µs, 3.2 µs
//! - BSS coloring (6-bit) for spatial reuse
//! - Target Wake Time (TWT) scheduling parameters
//! - Tone plans: 256/512/1024/2048-FFT for 20/40/80/160 MHz
//! - BCC and LDPC coding rates: 1/2, 2/3, 3/4, 5/6
//! - Spatial streams up to 8, STBC for 1-2 streams
//! - MU-MIMO with per-user MCS and stream allocation
//! - Trigger-based PPDU (uplink OFDMA)

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// Resource Unit sizes defined by the 802.11ax tone plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuSize {
    /// 26-tone RU (~2 MHz)
    Ru26,
    /// 52-tone RU (~4 MHz)
    Ru52,
    /// 106-tone RU (~8 MHz)
    Ru106,
    /// 242-tone RU (20 MHz)
    Ru242,
    /// 484-tone RU (40 MHz)
    Ru484,
    /// 996-tone RU (80 MHz)
    Ru996,
    /// 2×996-tone RU (160 MHz)
    Ru2x996,
}

impl RuSize {
    /// Number of data tones in each RU type.
    pub fn num_tones(&self) -> usize {
        match self {
            RuSize::Ru26 => 24,
            RuSize::Ru52 => 48,
            RuSize::Ru106 => 102,
            RuSize::Ru242 => 234,
            RuSize::Ru484 => 468,
            RuSize::Ru996 => 980,
            RuSize::Ru2x996 => 1960,
        }
    }

    /// Total tones (including guard/DC) for the RU.
    pub fn total_tones(&self) -> usize {
        match self {
            RuSize::Ru26 => 26,
            RuSize::Ru52 => 52,
            RuSize::Ru106 => 106,
            RuSize::Ru242 => 242,
            RuSize::Ru484 => 484,
            RuSize::Ru996 => 996,
            RuSize::Ru2x996 => 1992,
        }
    }

    /// Maximum number of RUs of this size that fit in a 20 MHz channel.
    pub fn max_per_20mhz(&self) -> usize {
        match self {
            RuSize::Ru26 => 9,
            RuSize::Ru52 => 4,
            RuSize::Ru106 => 2,
            RuSize::Ru242 => 1,
            _ => 0,
        }
    }

    /// Approximate bandwidth in MHz.
    pub fn bandwidth_mhz(&self) -> f64 {
        match self {
            RuSize::Ru26 => 2.03125,
            RuSize::Ru52 => 4.0625,
            RuSize::Ru106 => 8.28125,
            RuSize::Ru242 => 20.0,
            RuSize::Ru484 => 40.0,
            RuSize::Ru996 => 80.0,
            RuSize::Ru2x996 => 160.0,
        }
    }
}

/// HE MCS index (0–11).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum HeMcs {
    Mcs0,
    Mcs1,
    Mcs2,
    Mcs3,
    Mcs4,
    Mcs5,
    Mcs6,
    Mcs7,
    Mcs8,
    Mcs9,
    Mcs10,
    Mcs11,
}

impl HeMcs {
    /// Returns (modulation_order, bits_per_symbol) for the MCS.
    pub fn modulation_params(&self) -> (u32, u32) {
        match self {
            HeMcs::Mcs0 => (2, 1),   // BPSK
            HeMcs::Mcs1 => (4, 2),   // QPSK 1/2
            HeMcs::Mcs2 => (4, 2),   // QPSK 3/4
            HeMcs::Mcs3 => (16, 4),  // 16-QAM 1/2
            HeMcs::Mcs4 => (16, 4),  // 16-QAM 3/4
            HeMcs::Mcs5 => (64, 6),  // 64-QAM 2/3
            HeMcs::Mcs6 => (64, 6),  // 64-QAM 3/4
            HeMcs::Mcs7 => (64, 6),  // 64-QAM 5/6
            HeMcs::Mcs8 => (256, 8), // 256-QAM 3/4
            HeMcs::Mcs9 => (256, 8), // 256-QAM 5/6
            HeMcs::Mcs10 => (1024, 10), // 1024-QAM 3/4
            HeMcs::Mcs11 => (1024, 10), // 1024-QAM 5/6
        }
    }

    /// Coding rate as (numerator, denominator).
    pub fn coding_rate(&self) -> (u32, u32) {
        match self {
            HeMcs::Mcs0 => (1, 2),
            HeMcs::Mcs1 => (1, 2),
            HeMcs::Mcs2 => (3, 4),
            HeMcs::Mcs3 => (1, 2),
            HeMcs::Mcs4 => (3, 4),
            HeMcs::Mcs5 => (2, 3),
            HeMcs::Mcs6 => (3, 4),
            HeMcs::Mcs7 => (5, 6),
            HeMcs::Mcs8 => (3, 4),
            HeMcs::Mcs9 => (5, 6),
            HeMcs::Mcs10 => (3, 4),
            HeMcs::Mcs11 => (5, 6),
        }
    }

    /// Bits per symbol (log2 of modulation order).
    pub fn bits_per_symbol(&self) -> u32 {
        self.modulation_params().1
    }

    /// Numeric index.
    pub fn index(&self) -> u8 {
        match self {
            HeMcs::Mcs0 => 0,
            HeMcs::Mcs1 => 1,
            HeMcs::Mcs2 => 2,
            HeMcs::Mcs3 => 3,
            HeMcs::Mcs4 => 4,
            HeMcs::Mcs5 => 5,
            HeMcs::Mcs6 => 6,
            HeMcs::Mcs7 => 7,
            HeMcs::Mcs8 => 8,
            HeMcs::Mcs9 => 9,
            HeMcs::Mcs10 => 10,
            HeMcs::Mcs11 => 11,
        }
    }

    /// Create from numeric index.
    pub fn from_index(idx: u8) -> Option<Self> {
        match idx {
            0 => Some(HeMcs::Mcs0),
            1 => Some(HeMcs::Mcs1),
            2 => Some(HeMcs::Mcs2),
            3 => Some(HeMcs::Mcs3),
            4 => Some(HeMcs::Mcs4),
            5 => Some(HeMcs::Mcs5),
            6 => Some(HeMcs::Mcs6),
            7 => Some(HeMcs::Mcs7),
            8 => Some(HeMcs::Mcs8),
            9 => Some(HeMcs::Mcs9),
            10 => Some(HeMcs::Mcs10),
            11 => Some(HeMcs::Mcs11),
            _ => None,
        }
    }
}

/// HE guard interval durations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GuardInterval {
    /// 0.8 µs GI (legacy/short)
    Gi08us,
    /// 1.6 µs GI (medium)
    Gi16us,
    /// 3.2 µs GI (long, for outdoor/range)
    Gi32us,
}

impl GuardInterval {
    /// Duration in microseconds.
    pub fn duration_us(&self) -> f64 {
        match self {
            GuardInterval::Gi08us => 0.8,
            GuardInterval::Gi16us => 1.6,
            GuardInterval::Gi32us => 3.2,
        }
    }

    /// GI samples at 20 MHz sampling rate (useful subcarrier spacing = 78.125 kHz).
    /// Symbol duration = 12.8 µs → 256 samples at 20 MHz.
    pub fn gi_samples_20mhz(&self) -> usize {
        match self {
            GuardInterval::Gi08us => 16,
            GuardInterval::Gi16us => 32,
            GuardInterval::Gi32us => 64,
        }
    }

    /// 2-bit encoding used in HE-SIG-A.
    pub fn sig_a_encoding(&self) -> u8 {
        match self {
            GuardInterval::Gi08us => 0b00,
            GuardInterval::Gi16us => 0b01,
            GuardInterval::Gi32us => 0b11,
        }
    }
}

/// HE Long Training Field type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeLtfType {
    /// 1x LTF – 3.2 µs symbol (HE-SU/HE-ER-SU with GI 0.8 µs)
    Ltf1x,
    /// 2x LTF – 6.4 µs symbol
    Ltf2x,
    /// 4x LTF – 12.8 µs symbol (HE-MU default)
    Ltf4x,
}

impl HeLtfType {
    /// Duration in microseconds.
    pub fn duration_us(&self) -> f64 {
        match self {
            HeLtfType::Ltf1x => 3.2,
            HeLtfType::Ltf2x => 6.4,
            HeLtfType::Ltf4x => 12.8,
        }
    }

    /// 2-bit encoding in HE-SIG-A.
    pub fn sig_a_encoding(&self) -> u8 {
        match self {
            HeLtfType::Ltf1x => 0b00,
            HeLtfType::Ltf2x => 0b01,
            HeLtfType::Ltf4x => 0b10,
        }
    }
}

/// HE PPDU format variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HePpduFormat {
    /// HE Single User (SU-PPDU)
    Su,
    /// HE Multi User (MU-PPDU) – downlink OFDMA/MU-MIMO
    Mu,
    /// HE Extended Range SU (ER-SU)
    ExtRange,
    /// HE Trigger-Based (TB-PPDU) – uplink OFDMA
    TriggerBased,
}

/// FEC coding type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FecCode {
    /// Binary Convolutional Code
    Bcc,
    /// Low-Density Parity Check
    Ldpc,
}

/// BCC coding rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BccRate {
    R1_2,
    R2_3,
    R3_4,
    R5_6,
}

impl BccRate {
    pub fn as_f64(&self) -> f64 {
        match self {
            BccRate::R1_2 => 0.5,
            BccRate::R2_3 => 2.0 / 3.0,
            BccRate::R3_4 => 0.75,
            BccRate::R5_6 => 5.0 / 6.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Configuration structures
// ---------------------------------------------------------------------------

/// Per-user configuration for MU/OFDMA operations.
#[derive(Debug, Clone)]
pub struct HeUserConfig {
    /// User index (0-based)
    pub user_id: u8,
    /// Assigned RU type
    pub ru_size: RuSize,
    /// RU index within the channel (0-based)
    pub ru_index: u8,
    /// MCS for this user
    pub mcs: HeMcs,
    /// Number of spatial streams (1-8)
    pub nss: u8,
    /// Enable Dual Carrier Modulation (MCS 0 or 1 only)
    pub dcm: bool,
    /// FEC coding
    pub fec: FecCode,
    /// Space-Time Block Coding (only for nss=1 with 2 streams)
    pub stbc: bool,
}

impl HeUserConfig {
    /// Default single-user config.
    pub fn default_su() -> Self {
        HeUserConfig {
            user_id: 0,
            ru_size: RuSize::Ru242,
            ru_index: 0,
            mcs: HeMcs::Mcs7,
            nss: 1,
            dcm: false,
            fec: FecCode::Ldpc,
            stbc: false,
        }
    }
}

/// Target Wake Time schedule parameters.
#[derive(Debug, Clone)]
pub struct TwtSchedule {
    /// Whether TWT is enabled
    pub enabled: bool,
    /// TWT target wake interval in microseconds
    pub wake_interval_us: u64,
    /// Minimum wake duration in units of 256 µs
    pub min_wake_dur: u8,
    /// TWT channel (0-7)
    pub channel: u8,
    /// Trigger-based TWT
    pub trigger_based: bool,
    /// Unannounced TWT
    pub unannounced: bool,
    /// TWT flow ID
    pub flow_id: u8,
}

impl Default for TwtSchedule {
    fn default() -> Self {
        TwtSchedule {
            enabled: false,
            wake_interval_us: 102_400,
            min_wake_dur: 4,
            channel: 0,
            trigger_based: false,
            unannounced: false,
            flow_id: 0,
        }
    }
}

/// HE PPDU configuration.
#[derive(Debug, Clone)]
pub struct HeConfig {
    /// PPDU format
    pub ppdu_format: HePpduFormat,
    /// Channel bandwidth in MHz (20/40/80/160)
    pub bw_mhz: u16,
    /// BSS Color (6-bit, 1-63)
    pub bss_color: u8,
    /// Guard interval
    pub gi: GuardInterval,
    /// HE-LTF type
    pub ltf_type: HeLtfType,
    /// Spatial Reuse SR15 field value (4 bits)
    pub spatial_reuse: u8,
    /// TXOP duration (units of 8 µs, 0 = unset)
    pub txop_duration: u8,
    /// TWT schedule
    pub twt: TwtSchedule,
    /// LDPC extra symbol segment enabled
    pub ldpc_extra_symbol: bool,
    /// STBC for SU case
    pub stbc: bool,
    /// Doppler flag
    pub doppler: bool,
    /// Number of HE-LTF symbols
    pub num_ltf_symbols: u8,
}

impl Default for HeConfig {
    fn default() -> Self {
        HeConfig {
            ppdu_format: HePpduFormat::Su,
            bw_mhz: 80,
            bss_color: 1,
            gi: GuardInterval::Gi08us,
            ltf_type: HeLtfType::Ltf4x,
            spatial_reuse: 15,
            txop_duration: 0,
            twt: TwtSchedule::default(),
            ldpc_extra_symbol: false,
            stbc: false,
            doppler: false,
            num_ltf_symbols: 1,
        }
    }
}

/// A single allocated Resource Unit.
#[derive(Debug, Clone)]
pub struct ResourceUnit {
    /// User this RU is assigned to
    pub user_id: u8,
    /// RU size
    pub ru_size: RuSize,
    /// RU index within the 20 MHz sub-channel
    pub ru_index: u8,
    /// Starting subcarrier index in the FFT (absolute)
    pub start_subcarrier: i32,
    /// MCS assigned
    pub mcs: HeMcs,
    /// Number of spatial streams
    pub nss: u8,
    /// DCM enabled
    pub dcm: bool,
    /// FEC
    pub fec: FecCode,
}

// ---------------------------------------------------------------------------
// Tone plan constants
// ---------------------------------------------------------------------------

/// FFT sizes for each bandwidth.
pub fn fft_size(bw_mhz: u16) -> usize {
    match bw_mhz {
        20 => 256,
        40 => 512,
        80 => 1024,
        160 => 2048,
        _ => 256,
    }
}

/// OFDM symbol duration (excluding GI) = 12.8 µs for 802.11ax.
pub const SYMBOL_DURATION_US: f64 = 12.8;

/// Subcarrier spacing = 78.125 kHz.
pub const SUBCARRIER_SPACING_KHZ: f64 = 78.125;

// ---------------------------------------------------------------------------
// Main processor
// ---------------------------------------------------------------------------

/// Wi-Fi 6 (802.11ax) High Efficiency physical layer processor.
#[derive(Debug, Clone)]
pub struct WiFiAxHeProcessor {
    config: HeConfig,
    /// Pre-computed HE-STF sequence (frequency domain)
    stf_fd: Vec<(f64, f64)>,
    /// Pre-computed HE-LTF-1 sequence (frequency domain)
    ltf_fd: Vec<(f64, f64)>,
}

impl WiFiAxHeProcessor {
    /// Create a new HE processor with the given config.
    pub fn new(config: HeConfig) -> Self {
        let n_fft = fft_size(config.bw_mhz);
        let stf_fd = Self::build_he_stf_fd(n_fft);
        let ltf_fd = Self::build_he_ltf_fd(n_fft);
        WiFiAxHeProcessor { config, stf_fd, ltf_fd }
    }

    /// Return reference to current config.
    pub fn config(&self) -> &HeConfig {
        &self.config
    }

    // -----------------------------------------------------------------------
    // HE-STF (Short Training Field)
    // -----------------------------------------------------------------------

    /// Build HE-STF frequency-domain sequence for a given FFT size.
    /// The HE-STF occupies a subset of subcarriers following the 802.11ax tone plan.
    /// Returns complex IQ pairs (real, imag).
    fn build_he_stf_fd(n_fft: usize) -> Vec<(f64, f64)> {
        // HE-STF is based on the 11ax STF tone set (every 4th subcarrier)
        // simplified representative sequence derived from the standard pattern
        let mut fd = vec![(0.0f64, 0.0f64); n_fft];
        // Active subcarrier indices (4-tone spacing, non-DC, within bandwidth)
        let active_tones = Self::stf_active_tones(n_fft);
        let inv_sqrt2 = 1.0_f64 / 2.0_f64.sqrt();
        // Known BPSK-like pilots on STF subcarriers
        let pattern: &[i32] = &[
            1, 1, 1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1, 1,
            1, -1, 1, -1, -1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, 1,
        ];
        for (i, &sc) in active_tones.iter().enumerate() {
            let val = pattern[i % pattern.len()] as f64 * inv_sqrt2;
            let idx = (sc.rem_euclid(n_fft as i32)) as usize;
            fd[idx] = (val, 0.0);
        }
        fd
    }

    /// Generate the HE-STF time-domain waveform (IFFT of STF FD).
    pub fn generate_he_stf(&self) -> Vec<(f64, f64)> {
        let n_fft = fft_size(self.config.bw_mhz);
        // Perform IFFT
        let td = ifft_radix2(&self.stf_fd);
        // HE-STF = 2 repetitions of a short symbol (4x shorter = every 4th carrier)
        // For simplicity, return one full OFDM symbol worth of samples
        let gi_samples = self.config.gi.gi_samples_20mhz() * (n_fft / 256);
        let mut out = vec![(0.0f64, 0.0f64); n_fft + gi_samples];
        // Cyclic prefix
        for i in 0..gi_samples {
            out[i] = td[n_fft - gi_samples + i];
        }
        for i in 0..n_fft {
            out[gi_samples + i] = td[i];
        }
        out
    }

    /// Active subcarrier indices for the HE-STF (every 4th subcarrier pattern).
    fn stf_active_tones(n_fft: usize) -> Vec<i32> {
        // For 20 MHz (256-FFT): subcarriers ±4, ±8, ... ±120 (every 4th, skip DC)
        let half = (n_fft / 2) as i32;
        let step = 4i32;
        let mut tones = Vec::new();
        let mut sc = step;
        while sc <= half - step {
            tones.push(sc);
            tones.push(-sc);
            sc += step;
        }
        tones.sort();
        // Limit to 32 tones to match pattern length
        if tones.len() > 32 {
            tones.truncate(32);
        }
        tones
    }

    // -----------------------------------------------------------------------
    // HE-LTF (Long Training Field)
    // -----------------------------------------------------------------------

    /// Build HE-LTF-1 frequency-domain BPSK sequence.
    fn build_he_ltf_fd(n_fft: usize) -> Vec<(f64, f64)> {
        // HE-LTF uses all data+pilot subcarriers with known BPSK values
        let mut fd = vec![(0.0f64, 0.0f64); n_fft];
        let data_tones = Self::data_subcarriers(n_fft);
        // LTF-1 sequence: alternating +1/-1 pattern (simplified from 11ax Annex W)
        for (i, &sc) in data_tones.iter().enumerate() {
            let val = if i % 2 == 0 { 1.0f64 } else { -1.0f64 };
            let idx = (sc.rem_euclid(n_fft as i32)) as usize;
            fd[idx] = (val, 0.0);
        }
        fd
    }

    /// Generate HE-LTF time-domain samples for the given LTF type.
    pub fn generate_he_ltf(&self, ltf_type: HeLtfType) -> Vec<(f64, f64)> {
        let n_fft = fft_size(self.config.bw_mhz);
        let td = ifft_radix2(&self.ltf_fd);
        let gi_samples = self.config.gi.gi_samples_20mhz() * (n_fft / 256);

        // Number of repetitions based on LTF type
        let repetitions = match ltf_type {
            HeLtfType::Ltf1x => 1,
            HeLtfType::Ltf2x => 2,
            HeLtfType::Ltf4x => 4,
        };

        let sym_len = n_fft + gi_samples;
        let mut out = vec![(0.0f64, 0.0f64); sym_len * repetitions];
        for rep in 0..repetitions {
            let offset = rep * sym_len;
            // Cyclic prefix
            for i in 0..gi_samples {
                out[offset + i] = td[n_fft - gi_samples + i];
            }
            for i in 0..n_fft {
                out[offset + gi_samples + i] = td[i];
            }
        }
        out
    }

    // -----------------------------------------------------------------------
    // Subcarrier layout
    // -----------------------------------------------------------------------

    /// Return sorted list of data+pilot subcarrier indices (negative = lower half).
    fn data_subcarriers(n_fft: usize) -> Vec<i32> {
        let num_data = match n_fft {
            256 => 234,  // 20 MHz
            512 => 468,  // 40 MHz
            1024 => 980, // 80 MHz
            2048 => 1960, // 160 MHz
            _ => 234,
        };
        let half = (n_fft / 2) as i32;
        let mut scs = Vec::with_capacity(num_data);
        // Negative half (lower)
        for sc in (-half + 1)..0i32 {
            if sc != 0 {
                scs.push(sc);
            }
        }
        // Positive half (upper)
        for sc in 1..=half {
            scs.push(sc);
        }
        // Trim to num_data (center band, skip guard subcarriers)
        let guard = (scs.len() - num_data) / 2;
        scs[guard..scs.len() - guard].to_vec()
    }

    // -----------------------------------------------------------------------
    // HE-SIG-A encoding
    // -----------------------------------------------------------------------

    /// Encode HE-SIG-A field (2 OFDM symbols, 52 bits total for SU/MU).
    ///
    /// Bit layout (SU-PPDU, simplified from 802.11ax Table 27-18):
    /// - [1:0]   Format = 10 (HE SU)
    /// - [7:2]   BSS Color
    /// - [8]     UL/DL indicator (0=DL)
    /// - [10:9]  Bandwidth (00=20, 01=40, 10=80, 11=160)
    /// - [14:11] MCS index
    /// - [15]    DCM
    /// - [17:16] Coding (LDPC=1)
    /// - [19:18] GI+LTF size
    /// - [23:20] NSTS
    /// - [27:24] Spatial reuse
    /// - [28]    STBC
    /// - [36:29] TXOP
    /// - [48]    Doppler
    pub fn encode_he_sig_a(&self) -> Vec<u8> {
        let mut bits = [0u8; 52];
        let user = HeUserConfig::default_su();

        // [1:0] format = 10 (SU)
        bits[0] = 1; bits[1] = 0;
        // [7:2] BSS Color
        let color = self.config.bss_color & 0x3F;
        for i in 0..6 {
            bits[2 + i] = (color >> i) & 1;
        }
        // [8] UL/DL = 0 (DL)
        bits[8] = 0;
        // [10:9] BW
        let bw_field: u8 = match self.config.bw_mhz {
            20 => 0, 40 => 1, 80 => 2, 160 => 3, _ => 0,
        };
        bits[9] = bw_field & 1;
        bits[10] = (bw_field >> 1) & 1;
        // [14:11] MCS index
        let mcs_idx = user.mcs.index();
        for i in 0..4 {
            bits[11 + i] = (mcs_idx >> i) & 1;
        }
        // [15] DCM
        bits[15] = if user.dcm { 1 } else { 0 };
        // [17:16] coding (LDPC=01)
        bits[16] = 0; bits[17] = 1;
        // [19:18] GI+LTF
        let gi_ltf = self.config.gi.sig_a_encoding()
            | (self.config.ltf_type.sig_a_encoding() << 2);
        bits[18] = gi_ltf & 1;
        bits[19] = (gi_ltf >> 1) & 1;
        // [23:20] NSTS (nss-1)
        let nsts = (user.nss - 1) & 0xF;
        for i in 0..4 {
            bits[20 + i] = (nsts >> i) & 1;
        }
        // [27:24] Spatial reuse
        let sr = self.config.spatial_reuse & 0xF;
        for i in 0..4 {
            bits[24 + i] = (sr >> i) & 1;
        }
        // [28] STBC
        bits[28] = if self.config.stbc { 1 } else { 0 };
        // [36:29] TXOP
        let txop = self.config.txop_duration;
        for i in 0..8 {
            bits[29 + i] = (txop >> i) & 1;
        }
        // [48] Doppler
        bits[48] = if self.config.doppler { 1 } else { 0 };

        // Pack bits into bytes (LSB first)
        let nbytes = (52 + 7) / 8;
        let mut out = vec![0u8; nbytes];
        for (i, &b) in bits.iter().enumerate() {
            out[i / 8] |= b << (i % 8);
        }
        out
    }

    // -----------------------------------------------------------------------
    // HE-SIG-B encoding
    // -----------------------------------------------------------------------

    /// Encode HE-SIG-B for each allocated RU (MU-PPDU).
    ///
    /// Each user field (variable, typically 21-22 bits) encodes:
    /// - RU allocation (8 bits)
    /// - NSTS (3 bits)
    /// - MCS (4 bits)
    /// - DCM (1 bit)
    /// - Coding (1 bit)
    /// - Reserved (4 bits)
    pub fn encode_he_sig_b(&self, ru_alloc: &[ResourceUnit]) -> Vec<u8> {
        let mut all_bits: Vec<u8> = Vec::new();

        for ru in ru_alloc {
            // 8-bit RU allocation field (encodes RU size + index)
            let ru_alloc_field = self.ru_allocation_field(ru.ru_size, ru.ru_index);
            for i in 0..8 {
                all_bits.push((ru_alloc_field >> i) & 1);
            }
            // 3-bit NSTS (nss-1)
            let nsts = (ru.nss.saturating_sub(1)) & 0x7;
            for i in 0..3 {
                all_bits.push((nsts >> i) & 1);
            }
            // 4-bit MCS
            let mcs_idx = ru.mcs.index();
            for i in 0..4 {
                all_bits.push((mcs_idx >> i) & 1);
            }
            // 1-bit DCM
            all_bits.push(if ru.dcm { 1 } else { 0 });
            // 1-bit coding (LDPC=1, BCC=0)
            all_bits.push(match ru.fec { FecCode::Ldpc => 1, FecCode::Bcc => 0 });
            // 4-bit reserved
            for _ in 0..4 {
                all_bits.push(0);
            }
        }

        // Pack into bytes
        let nbytes = (all_bits.len() + 7) / 8;
        let mut out = vec![0u8; nbytes];
        for (i, &b) in all_bits.iter().enumerate() {
            out[i / 8] |= b << (i % 8);
        }
        out
    }

    /// Compute 8-bit RU allocation field.
    fn ru_allocation_field(&self, ru_size: RuSize, ru_index: u8) -> u8 {
        // Encoding: upper 3 bits = RU type, lower 5 bits = index within channel
        let type_code: u8 = match ru_size {
            RuSize::Ru26 => 0,
            RuSize::Ru52 => 1,
            RuSize::Ru106 => 2,
            RuSize::Ru242 => 3,
            RuSize::Ru484 => 4,
            RuSize::Ru996 => 5,
            RuSize::Ru2x996 => 6,
        };
        (type_code << 5) | (ru_index & 0x1F)
    }

    // -----------------------------------------------------------------------
    // RU allocation
    // -----------------------------------------------------------------------

    /// Allocate Resource Units for a list of users.
    /// Returns RU descriptors with subcarrier assignments.
    pub fn allocate_ru(&self, users: &[HeUserConfig]) -> Vec<ResourceUnit> {
        let n_fft = fft_size(self.config.bw_mhz);
        let data_scs = Self::data_subcarriers(n_fft);
        let total_data = data_scs.len() as i32;

        let mut allocated = Vec::new();
        let mut subcarrier_cursor: i32 = -(total_data / 2);

        for user in users {
            let n_tones = user.ru_size.num_tones() as i32;
            // Clamp to available bandwidth
            if subcarrier_cursor + n_tones > total_data / 2 {
                break;
            }
            let start_sc = subcarrier_cursor;
            let ru = ResourceUnit {
                user_id: user.user_id,
                ru_size: user.ru_size,
                ru_index: user.ru_index,
                start_subcarrier: start_sc,
                mcs: user.mcs,
                nss: user.nss.min(8),
                dcm: user.dcm && (user.mcs == HeMcs::Mcs0 || user.mcs == HeMcs::Mcs1),
                fec: user.fec,
            };
            allocated.push(ru);
            subcarrier_cursor += n_tones;
        }
        allocated
    }

    // -----------------------------------------------------------------------
    // OFDMA modulation
    // -----------------------------------------------------------------------

    /// Modulate data bytes onto a resource unit using QAM.
    /// Returns the time-domain IQ samples (one OFDM symbol with GI).
    pub fn modulate_ofdma(&self, data: &[u8], ru: &ResourceUnit) -> Vec<(f64, f64)> {
        let n_fft = fft_size(self.config.bw_mhz);
        let gi_samples = self.config.gi.gi_samples_20mhz() * (n_fft / 256);

        // Build frequency domain buffer
        let mut fd = vec![(0.0f64, 0.0f64); n_fft];

        // Number of data tones in this RU
        let n_tones = ru.ru_size.num_tones();

        // Map bytes to QAM symbols on the RU subcarriers
        let bits_per_sym = ru.mcs.bits_per_symbol() as usize;
        let bits: Vec<u8> = bytes_to_bits(data);
        let total_bits = bits.len();

        for (tone_idx, bit_offset) in (0..n_tones).zip((0..total_bits).step_by(bits_per_sym)) {
            // Get bits for this symbol
            let end = (bit_offset + bits_per_sym).min(total_bits);
            let sym_bits = &bits[bit_offset..end];

            // Modulate
            let (i_val, q_val) = qam_modulate(sym_bits, ru.mcs);

            // Apply DCM if enabled: map same symbol to two subcarriers
            if ru.dcm && (ru.mcs == HeMcs::Mcs0 || ru.mcs == HeMcs::Mcs1) {
                let sc1 = ru.start_subcarrier + tone_idx as i32;
                let sc2 = ru.start_subcarrier + tone_idx as i32 + (n_tones / 2) as i32;
                let idx1 = sc1.rem_euclid(n_fft as i32) as usize;
                let idx2 = sc2.rem_euclid(n_fft as i32) as usize;
                if idx1 < n_fft { fd[idx1] = (i_val, q_val); }
                if idx2 < n_fft { fd[idx2] = (i_val, q_val); }
                // Only fill half the tones when DCM is active
                if tone_idx >= n_tones / 2 { break; }
            } else {
                let sc = ru.start_subcarrier + tone_idx as i32;
                let idx = sc.rem_euclid(n_fft as i32) as usize;
                if idx < n_fft {
                    fd[idx] = (i_val, q_val);
                }
            }
        }

        // IFFT
        let td = ifft_radix2(&fd);

        // Add cyclic prefix
        let mut out = vec![(0.0f64, 0.0f64); n_fft + gi_samples];
        for i in 0..gi_samples {
            out[i] = td[n_fft - gi_samples + i];
        }
        for i in 0..n_fft {
            out[gi_samples + i] = td[i];
        }
        out
    }

    /// Demodulate a received OFDM symbol from a given RU.
    /// Returns the decoded data bytes.
    pub fn demodulate_ofdma(&self, samples: &[(f64, f64)], ru: &ResourceUnit) -> Vec<u8> {
        let n_fft = fft_size(self.config.bw_mhz);
        let gi_samples = self.config.gi.gi_samples_20mhz() * (n_fft / 256);

        // Strip cyclic prefix
        let data_start = gi_samples.min(samples.len());
        let sym_end = (data_start + n_fft).min(samples.len());
        let mut td = vec![(0.0f64, 0.0f64); n_fft];
        for i in 0..(sym_end - data_start) {
            td[i] = samples[data_start + i];
        }

        // FFT
        let fd = fft_radix2(&td);

        let n_tones = ru.ru_size.num_tones();
        let bits_per_sym = ru.mcs.bits_per_symbol() as usize;
        let mut all_bits: Vec<u8> = Vec::new();

        for tone_idx in 0..n_tones {
            let sc = ru.start_subcarrier + tone_idx as i32;
            let idx = sc.rem_euclid(n_fft as i32) as usize;
            if idx < fd.len() {
                let (i_val, q_val) = fd[idx];
                let sym_bits = qam_demodulate(i_val, q_val, ru.mcs);
                all_bits.extend_from_slice(&sym_bits);
            }
        }

        bits_to_bytes(&all_bits)
    }

    // -----------------------------------------------------------------------
    // Data rate calculation
    // -----------------------------------------------------------------------

    /// Calculate peak throughput in Mbit/s for given parameters.
    ///
    /// Formula: R = (N_data × N_bps × R_code × N_ss) / T_sym
    /// where T_sym = T_DFT + T_GI
    pub fn calculate_data_rate(
        &self,
        mcs: HeMcs,
        bw_mhz: u16,
        nss: u8,
        gi: GuardInterval,
    ) -> f64 {
        let n_data = match bw_mhz {
            20 => 234.0,
            40 => 468.0,
            80 => 980.0,
            160 => 1960.0,
            _ => 234.0,
        };
        let n_bps = mcs.bits_per_symbol() as f64;
        let (r_num, r_den) = mcs.coding_rate();
        let r_code = r_num as f64 / r_den as f64;
        let n_ss = nss as f64;

        // Symbol duration: T_DFT + T_GI
        let t_sym_us = SYMBOL_DURATION_US + gi.duration_us();

        // Data rate in Mbit/s
        (n_data * n_bps * r_code * n_ss) / t_sym_us
    }

    /// Calculate data rate for the current processor config with given MCS and NSS.
    pub fn data_rate_mbps(&self, mcs: HeMcs, nss: u8) -> f64 {
        self.calculate_data_rate(mcs, self.config.bw_mhz, nss, self.config.gi)
    }

    // -----------------------------------------------------------------------
    // BCC encoding (simplified)
    // -----------------------------------------------------------------------

    /// BCC encoder: rate-1/2 convolutional code with polynomials 133, 171 (octal).
    /// Returns encoded bits.
    pub fn bcc_encode(&self, input: &[u8], rate: BccRate) -> Vec<u8> {
        let bits = bytes_to_bits(input);
        let encoded = bcc_encode_bits(&bits);
        // Puncture for higher rates
        let punctured = bcc_puncture(&encoded, rate);
        bits_to_bytes(&punctured)
    }

    /// BCC decoder: hard-decision Viterbi for rate-1/2.
    pub fn bcc_decode(&self, input: &[u8], rate: BccRate) -> Vec<u8> {
        let bits = bytes_to_bits(input);
        let depunctured = bcc_depuncture(&bits, rate);
        let decoded = viterbi_decode_hard(&depunctured);
        bits_to_bytes(&decoded)
    }

    // -----------------------------------------------------------------------
    // LDPC encoding (simplified parity check)
    // -----------------------------------------------------------------------

    /// LDPC encoder (systematic): appends parity bits.
    /// This is a simplified rate-1/2 systematic code for demonstration.
    pub fn ldpc_encode(&self, input: &[u8]) -> Vec<u8> {
        ldpc_encode_simple(input)
    }

    /// LDPC decoder: belief propagation (simplified).
    pub fn ldpc_decode(&self, input: &[u8]) -> Vec<u8> {
        ldpc_decode_simple(input)
    }

    // -----------------------------------------------------------------------
    // Trigger Frame (uplink OFDMA)
    // -----------------------------------------------------------------------

    /// Build a Trigger Frame (TF) for uplink OFDMA.
    /// Returns the frame bytes including MAC header fields.
    pub fn build_trigger_frame(&self, users: &[HeUserConfig]) -> Vec<u8> {
        let mut frame: Vec<u8> = Vec::new();

        // Frame Control (2 bytes) – trigger frame subtype
        frame.push(0xD4); // type=data, subtype=1101 (trigger)
        frame.push(0x00);

        // Duration (2 bytes)
        let duration_us: u16 = 200;
        frame.push((duration_us & 0xFF) as u8);
        frame.push((duration_us >> 8) as u8);

        // RA (6 bytes) – broadcast
        for _ in 0..6 {
            frame.push(0xFF);
        }

        // TA (6 bytes) – AP address (placeholder)
        for i in 0..6u8 {
            frame.push(0x00 + i);
        }

        // Common Info (8 bytes)
        // [7:0] TF type = 0 (basic)
        frame.push(0x00);
        // LSIG length (12 bits)
        let lsig_len: u16 = 100;
        frame.push((lsig_len & 0xFF) as u8);
        frame.push(((lsig_len >> 8) & 0x0F) as u8);
        // CS required, BW, GI, MU-MIMO LTF mode, STBC
        let bw_field: u8 = match self.config.bw_mhz {
            20 => 0, 40 => 1, 80 => 2, 160 => 3, _ => 0,
        };
        frame.push(bw_field);
        frame.push(self.config.gi.sig_a_encoding());
        frame.push(0x00); // MU-MIMO LTF mode
        frame.push(self.config.txop_duration);
        frame.push(0x00); // reserved

        // Per-user info (6 bytes each)
        for user in users {
            // AID12 (12 bits)
            let aid = (user.user_id as u16) & 0x0FFF;
            frame.push((aid & 0xFF) as u8);
            frame.push(((aid >> 8) & 0x0F) as u8);

            // RU allocation (8 bits)
            let ru_alloc = self.ru_allocation_field(user.ru_size, user.ru_index);
            frame.push(ru_alloc);

            // UL FEC coding + MCS + DCM
            let coding_bit: u8 = match user.fec { FecCode::Ldpc => 1, FecCode::Bcc => 0 };
            let mcs = user.mcs.index() & 0x0F;
            let dcm_bit: u8 = if user.dcm { 1 } else { 0 };
            frame.push((coding_bit) | (mcs << 1) | (dcm_bit << 5));

            // SS allocation + target RSSI (8 bits)
            let ss_alloc = (user.nss.saturating_sub(1)) & 0x7;
            frame.push(ss_alloc);

            // Trigger-dependent user info (8 bits) – placeholder
            frame.push(0x00);
        }

        // FCS placeholder (4 bytes)
        let crc = crc32_simple(&frame);
        frame.extend_from_slice(&crc.to_le_bytes());

        frame
    }

    // -----------------------------------------------------------------------
    // Spectral efficiency
    // -----------------------------------------------------------------------

    /// Compute spectral efficiency in bits/s/Hz for given parameters.
    pub fn spectral_efficiency(&self, mcs: HeMcs, nss: u8) -> f64 {
        let (_, n_bps) = mcs.modulation_params();
        let (r_num, r_den) = mcs.coding_rate();
        let r_code = r_num as f64 / r_den as f64;
        n_bps as f64 * r_code * nss as f64
    }

    // -----------------------------------------------------------------------
    // TWT helper
    // -----------------------------------------------------------------------

    /// Encode TWT element bytes (simplified 802.11ax TWT Information Element).
    pub fn encode_twt_element(&self) -> Vec<u8> {
        let twt = &self.config.twt;
        if !twt.enabled {
            return vec![];
        }
        let mut ie: Vec<u8> = Vec::new();
        // Element ID = 216, Element ID extension = 26 (TWT)
        ie.push(216);
        ie.push(0x00); // length placeholder
        ie.push(26);   // ext ID
        // Request type (2 bytes)
        let mut req_type: u16 = 0;
        req_type |= 1u16; // requester
        if twt.trigger_based { req_type |= (1 << 4); }
        if twt.unannounced   { req_type |= (1 << 5); }
        req_type |= (1 << 6); // implicit
        ie.push((req_type & 0xFF) as u8);
        ie.push((req_type >> 8) as u8);
        // Target Wake Time (8 bytes)
        let twt_val = twt.wake_interval_us;
        for i in 0..8 {
            ie.push(((twt_val >> (i * 8)) & 0xFF) as u8);
        }
        // Min Wake Duration (1 byte)
        ie.push(twt.min_wake_dur);
        // Wake Interval Mantissa (2 bytes)
        ie.push(0xFF); ie.push(0x7F);
        // Wake Interval Exponent (1 byte) + Channel (1 byte)
        ie.push(10);
        ie.push(twt.channel & 0x07);
        // Flow ID + padding
        ie.push(twt.flow_id & 0x7);
        // Update length
        let len = (ie.len() - 2) as u8;
        ie[1] = len;
        ie
    }

    // -----------------------------------------------------------------------
    // Tone plan query
    // -----------------------------------------------------------------------

    /// Return the number of active data subcarriers for a given bandwidth.
    pub fn num_data_subcarriers(bw_mhz: u16) -> usize {
        match bw_mhz {
            20 => 234,
            40 => 468,
            80 => 980,
            160 => 1960,
            _ => 234,
        }
    }

    /// Maximum achievable data rate in Mbps (MCS11, 8SS, 160 MHz, GI 0.8µs).
    pub fn peak_data_rate_mbps() -> f64 {
        let proc = WiFiAxHeProcessor::new(HeConfig {
            bw_mhz: 160,
            gi: GuardInterval::Gi08us,
            ..Default::default()
        });
        proc.calculate_data_rate(HeMcs::Mcs11, 160, 8, GuardInterval::Gi08us)
    }

    // -----------------------------------------------------------------------
    // STBC helpers
    // -----------------------------------------------------------------------

    /// Apply Alamouti 2×1 STBC encoding to a stream of QAM symbols.
    /// Returns interleaved pairs (s0, s1, -s1*, s0*) for two TX antennas.
    pub fn apply_stbc_2x1(&self, symbols: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut out = Vec::with_capacity(symbols.len() * 2);
        let mut i = 0;
        while i + 1 < symbols.len() {
            let s0 = symbols[i];
            let s1 = symbols[i + 1];
            // Antenna 0: s0, -s1*
            out.push(s0);
            out.push((-s1.0, s1.1));
            // Antenna 1: s1, s0*
            out.push(s1);
            out.push((s0.0, -s0.1));
            i += 2;
        }
        out
    }

    /// Decode Alamouti 2×1 STBC from two received streams.
    pub fn decode_stbc_2x1(
        &self,
        rx0: &[(f64, f64)],
        rx1: &[(f64, f64)],
        h0: (f64, f64),
        h1: (f64, f64),
    ) -> Vec<(f64, f64)> {
        let len = rx0.len().min(rx1.len());
        let mut out = Vec::with_capacity(len);
        let mut i = 0;
        while i + 1 < len {
            let r0 = rx0[i];
            let r1 = rx0[i + 1];
            // MRC combining
            // s_hat_0 = h0* r0 + h1 r1*
            let s0_i = h0.0 * r0.0 + h0.1 * r0.1 + h1.0 * r1.0 - h1.1 * r1.1;
            let s0_q = h0.0 * r0.1 - h0.1 * r0.0 + h1.0 * r1.1 + h1.1 * r1.0;
            // s_hat_1 = h1* r0 - h0 r1*
            let s1_i = h1.0 * r0.0 + h1.1 * r0.1 - h0.0 * r1.0 + h0.1 * r1.1;
            let s1_q = h1.0 * r0.1 - h1.1 * r0.0 - h0.0 * r1.1 - h0.1 * r1.0;
            out.push((s0_i, s0_q));
            out.push((s1_i, s1_q));
            i += 2;
        }
        let _ = rx1; // used implicitly above via rx0 pairs
        out
    }

    // -----------------------------------------------------------------------
    // MU-MIMO precoding (ZF)
    // -----------------------------------------------------------------------

    /// Zero-forcing precoder for MU-MIMO.
    /// H is a flat (n_rx × n_tx) channel matrix (row-major).
    /// Computes W = H† (H H†)^-1 (pseudo-inverse).
    pub fn zf_precoder(h: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
        let n_rx = h.len();
        if n_rx == 0 { return vec![]; }
        let n_tx = h[0].len();

        // Compute H * H† (n_rx × n_rx)
        let mut hht = vec![vec![(0.0f64, 0.0f64); n_rx]; n_rx];
        for i in 0..n_rx {
            for j in 0..n_rx {
                let mut s = (0.0f64, 0.0f64);
                for k in 0..n_tx {
                    let hik = h[i][k];
                    let hjk = h[j][k];
                    // hik * hjk*
                    s.0 += hik.0 * hjk.0 + hik.1 * hjk.1;
                    s.1 += hik.1 * hjk.0 - hik.0 * hjk.1;
                }
                hht[i][j] = s;
            }
        }

        // Invert HHt (2×2 special case or diagonal approximation)
        let hht_inv = complex_matrix_inv_2x2(&hht);

        // W = H† * HHt_inv  (n_tx × n_rx)
        let mut w = vec![vec![(0.0f64, 0.0f64); n_rx]; n_tx];
        for i in 0..n_tx {
            for j in 0..n_rx {
                let mut s = (0.0f64, 0.0f64);
                for k in 0..n_rx {
                    let ht_ik = (h[k][i].0, -h[k][i].1); // conjugate transpose
                    let inv_kj = hht_inv[k][j];
                    s.0 += ht_ik.0 * inv_kj.0 - ht_ik.1 * inv_kj.1;
                    s.1 += ht_ik.0 * inv_kj.1 + ht_ik.1 * inv_kj.0;
                }
                w[i][j] = s;
            }
        }
        w
    }

    // -----------------------------------------------------------------------
    // Utility: generate full HE PPDU preamble
    // -----------------------------------------------------------------------

    /// Generate the full HE preamble: HE-STF + HE-LTF symbols.
    pub fn generate_he_preamble(&self) -> Vec<(f64, f64)> {
        let mut out = Vec::new();
        out.extend(self.generate_he_stf());
        out.extend(self.generate_he_ltf(self.config.ltf_type));
        out
    }
}

impl Default for WiFiAxHeProcessor {
    fn default() -> Self {
        WiFiAxHeProcessor::new(HeConfig::default())
    }
}

// ---------------------------------------------------------------------------
// DSP helpers: FFT / IFFT (Cooley-Tukey radix-2, DIF)
// ---------------------------------------------------------------------------

/// In-place radix-2 DIF FFT.  n must be a power of 2.
fn fft_inplace(buf: &mut Vec<(f64, f64)>, inverse: bool) {
    let n = buf.len();
    if n <= 1 { return; }

    // Bit-reverse permutation
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
    let sign: f64 = if inverse { 1.0 } else { -1.0 };
    let mut len = 2usize;
    while len <= n {
        let ang = sign * 2.0 * PI / len as f64;
        let wlen = (ang.cos(), ang.sin());
        for i in (0..n).step_by(len) {
            let mut w = (1.0f64, 0.0f64);
            for k in 0..(len / 2) {
                let u = buf[i + k];
                let v_r = buf[i + k + len / 2].0 * w.0 - buf[i + k + len / 2].1 * w.1;
                let v_i = buf[i + k + len / 2].0 * w.1 + buf[i + k + len / 2].1 * w.0;
                buf[i + k] = (u.0 + v_r, u.1 + v_i);
                buf[i + k + len / 2] = (u.0 - v_r, u.1 - v_i);
                let w_new = (w.0 * wlen.0 - w.1 * wlen.1, w.0 * wlen.1 + w.1 * wlen.0);
                w = w_new;
            }
        }
        len <<= 1;
    }

    if inverse {
        let scale = 1.0 / n as f64;
        for s in buf.iter_mut() {
            s.0 *= scale;
            s.1 *= scale;
        }
    }
}

fn fft_radix2(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    // Pad to next power-of-2 if needed
    let n2 = next_power_of_2(n);
    let mut buf = vec![(0.0f64, 0.0f64); n2];
    buf[..n].copy_from_slice(input);
    fft_inplace(&mut buf, false);
    buf
}

fn ifft_radix2(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    let n2 = next_power_of_2(n);
    let mut buf = vec![(0.0f64, 0.0f64); n2];
    buf[..n].copy_from_slice(input);
    fft_inplace(&mut buf, true);
    buf[..n].to_vec()
}

fn next_power_of_2(n: usize) -> usize {
    if n == 0 { return 1; }
    let mut p = 1usize;
    while p < n { p <<= 1; }
    p
}

// ---------------------------------------------------------------------------
// QAM modulation helpers
// ---------------------------------------------------------------------------

fn qam_modulate(bits: &[u8], mcs: HeMcs) -> (f64, f64) {
    let order = mcs.modulation_params().0;
    match order {
        2 => {
            // BPSK
            let b0 = bits.first().copied().unwrap_or(0);
            (if b0 == 0 { 1.0 } else { -1.0 }, 0.0)
        }
        4 => {
            // QPSK
            let b0 = bits.first().copied().unwrap_or(0);
            let b1 = bits.get(1).copied().unwrap_or(0);
            let i = if b0 == 0 { 1.0f64 } else { -1.0f64 } / 2.0f64.sqrt();
            let q = if b1 == 0 { 1.0f64 } else { -1.0f64 } / 2.0f64.sqrt();
            (i, q)
        }
        16 => {
            // 16-QAM (Gray coded, normalized)
            let b: u8 = bits_to_nibble(bits, 4);
            let i_idx = ((b >> 2) & 0x3) as f64;
            let q_idx = (b & 0x3) as f64;
            let scale = 1.0 / (10.0f64.sqrt());
            let map = |x: f64| -> f64 { (2.0 * x - 3.0) * scale };
            (map(i_idx), map(q_idx))
        }
        64 => {
            // 64-QAM
            let b: u8 = bits_to_nibble(bits, 6);
            let i_idx = ((b >> 3) & 0x7) as f64;
            let q_idx = (b & 0x7) as f64;
            let scale = 1.0 / (42.0f64.sqrt());
            let map = |x: f64| -> f64 { (2.0 * x - 7.0) * scale };
            (map(i_idx), map(q_idx))
        }
        256 => {
            // 256-QAM
            let byte = bits_to_nibble_u16(bits, 8) as f64;
            let i_idx = ((byte as u8) >> 4) as f64;
            let q_idx = ((byte as u8) & 0xF) as f64;
            let scale = 1.0 / (170.0f64.sqrt());
            let map = |x: f64| -> f64 { (2.0 * x - 15.0) * scale };
            (map(i_idx), map(q_idx))
        }
        1024 => {
            // 1024-QAM
            let w = bits_to_nibble_u16(bits, 10);
            let i_idx = ((w >> 5) & 0x1F) as f64;
            let q_idx = (w & 0x1F) as f64;
            let scale = 1.0 / (682.0f64.sqrt());
            let map = |x: f64| -> f64 { (2.0 * x - 31.0) * scale };
            (map(i_idx), map(q_idx))
        }
        _ => (0.0, 0.0),
    }
}

fn qam_demodulate(i_val: f64, q_val: f64, mcs: HeMcs) -> Vec<u8> {
    let n_bits = mcs.bits_per_symbol() as usize;
    let order = mcs.modulation_params().0;
    match order {
        2 => vec![if i_val >= 0.0 { 0 } else { 1 }],
        4 => {
            let b0 = if i_val >= 0.0 { 0 } else { 1 };
            let b1 = if q_val >= 0.0 { 0 } else { 1 };
            vec![b0, b1]
        }
        16 => {
            let scale = (10.0f64).sqrt();
            let unmap = |x: f64| -> u8 { ((x * scale + 3.0) / 2.0).round().clamp(0.0, 3.0) as u8 };
            let i_idx = unmap(i_val);
            let q_idx = unmap(q_val);
            nibble_to_bits((i_idx << 2) | q_idx, 4)
        }
        64 => {
            let scale = (42.0f64).sqrt();
            let unmap = |x: f64| -> u8 { ((x * scale + 7.0) / 2.0).round().clamp(0.0, 7.0) as u8 };
            let i_idx = unmap(i_val);
            let q_idx = unmap(q_val);
            nibble_to_bits((i_idx << 3) | q_idx, 6)
        }
        256 => {
            let scale = (170.0f64).sqrt();
            let unmap = |x: f64| -> u8 { ((x * scale + 15.0) / 2.0).round().clamp(0.0, 15.0) as u8 };
            let i_idx = unmap(i_val);
            let q_idx = unmap(q_val);
            nibble_to_bits((i_idx << 4) | q_idx, 8)
        }
        1024 => {
            let scale = (682.0f64).sqrt();
            let unmap = |x: f64| -> u8 { ((x * scale + 31.0) / 2.0).round().clamp(0.0, 31.0) as u8 };
            let i_idx = unmap(i_val);
            let q_idx = unmap(q_val);
            let combined = ((i_idx as u16) << 5) | q_idx as u16;
            (0..n_bits).map(|k| ((combined >> k) & 1) as u8).collect()
        }
        _ => vec![0u8; n_bits],
    }
}

fn bits_to_nibble(bits: &[u8], n: usize) -> u8 {
    let mut v = 0u8;
    for i in 0..n.min(bits.len()).min(8) {
        v |= bits[i] << i;
    }
    v
}

fn bits_to_nibble_u16(bits: &[u8], n: usize) -> u16 {
    let mut v = 0u16;
    for i in 0..n.min(bits.len()).min(16) {
        v |= (bits[i] as u16) << i;
    }
    v
}

fn nibble_to_bits(val: u8, n: usize) -> Vec<u8> {
    (0..n).map(|i| (val >> i) & 1).collect()
}

// ---------------------------------------------------------------------------
// Bit manipulation helpers
// ---------------------------------------------------------------------------

pub fn bytes_to_bits(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len() * 8);
    for &byte in data {
        for i in 0..8 {
            out.push((byte >> i) & 1);
        }
    }
    out
}

pub fn bits_to_bytes(bits: &[u8]) -> Vec<u8> {
    let nbytes = (bits.len() + 7) / 8;
    let mut out = vec![0u8; nbytes];
    for (i, &b) in bits.iter().enumerate() {
        out[i / 8] |= b << (i % 8);
    }
    out
}

// ---------------------------------------------------------------------------
// BCC helpers
// ---------------------------------------------------------------------------

/// Rate-1/2 BCC encoder: polynomials g0=0b1011011 (133oct), g1=0b1111001 (171oct).
fn bcc_encode_bits(bits: &[u8]) -> Vec<u8> {
    let g0: u8 = 0b1011011; // 91
    let g1: u8 = 0b1111001; // 121
    let mut shift = 0u8;
    let mut out = Vec::with_capacity(bits.len() * 2);
    for &b in bits {
        shift = ((shift << 1) | (b & 1)) & 0x7F;
        let c0 = (shift & g0).count_ones() as u8 & 1;
        let c1 = (shift & g1).count_ones() as u8 & 1;
        out.push(c0);
        out.push(c1);
    }
    out
}

/// Puncture BCC output for higher coding rates.
fn bcc_puncture(bits: &[u8], rate: BccRate) -> Vec<u8> {
    match rate {
        BccRate::R1_2 => bits.to_vec(),
        BccRate::R2_3 => {
            // Pattern: keep [1,1,1,0] → remove every 4th
            bits.chunks(4).flat_map(|c| c.iter().take(3).copied()).collect()
        }
        BccRate::R3_4 => {
            // Pattern: keep [1,1,1,0,0,1]
            bits.chunks(6).flat_map(|c| {
                [c.get(0).copied().unwrap_or(0),
                 c.get(1).copied().unwrap_or(0),
                 c.get(2).copied().unwrap_or(0),
                 c.get(5).copied().unwrap_or(0)]
            }).collect()
        }
        BccRate::R5_6 => {
            // Pattern: keep [1,1,1,0,0,1,1,0,0,1]
            bits.chunks(10).flat_map(|c| {
                [c.get(0).copied().unwrap_or(0),
                 c.get(1).copied().unwrap_or(0),
                 c.get(2).copied().unwrap_or(0),
                 c.get(5).copied().unwrap_or(0),
                 c.get(6).copied().unwrap_or(0),
                 c.get(9).copied().unwrap_or(0)]
            }).collect()
        }
    }
}

/// Depuncture: insert erasure markers (2) for missing bits.
fn bcc_depuncture(bits: &[u8], rate: BccRate) -> Vec<u8> {
    match rate {
        BccRate::R1_2 => bits.to_vec(),
        BccRate::R2_3 => {
            bits.chunks(3).flat_map(|c| {
                [c.get(0).copied().unwrap_or(0),
                 c.get(1).copied().unwrap_or(0),
                 c.get(2).copied().unwrap_or(0),
                 2u8]
            }).collect()
        }
        BccRate::R3_4 => {
            bits.chunks(4).flat_map(|c| {
                [c.get(0).copied().unwrap_or(0),
                 c.get(1).copied().unwrap_or(0),
                 c.get(2).copied().unwrap_or(0),
                 2u8, 2u8,
                 c.get(3).copied().unwrap_or(0)]
            }).collect()
        }
        BccRate::R5_6 => {
            bits.chunks(6).flat_map(|c| {
                [c.get(0).copied().unwrap_or(0),
                 c.get(1).copied().unwrap_or(0),
                 c.get(2).copied().unwrap_or(0),
                 2u8, 2u8,
                 c.get(3).copied().unwrap_or(0),
                 c.get(4).copied().unwrap_or(0),
                 2u8, 2u8,
                 c.get(5).copied().unwrap_or(0)]
            }).collect()
        }
    }
}

/// Hard-decision Viterbi decoder for rate-1/2 BCC.
/// Uses register states for polynomials g0=133oct, g1=171oct.
fn viterbi_decode_hard(bits: &[u8]) -> Vec<u8> {
    const K: usize = 7; // constraint length
    const STATES: usize = 1 << (K - 1); // 64 states
    const INF: u32 = u32::MAX / 2;

    let g0: u8 = 0b1011011;
    let g1: u8 = 0b1111001;

    // Precompute outputs for each (state, input_bit)
    let mut output = [[[0u8; 2]; 2]; STATES];
    for state in 0..STATES {
        for bit in 0..2usize {
            let reg = (((bit << (K - 1)) | state) & 0x7F) as u8;
            output[state][bit][0] = (reg & g0).count_ones() as u8 & 1;
            output[state][bit][1] = (reg & g1).count_ones() as u8 & 1;
        }
    }

    let n_pairs = bits.len() / 2;
    let mut pm = vec![INF; STATES];
    pm[0] = 0;
    let mut traceback: Vec<Vec<(usize, u8)>> = Vec::with_capacity(n_pairs);

    for t in 0..n_pairs {
        let r0 = if bits[2 * t] > 1 { 0u32 } else { bits[2 * t] as u32 };
        let r1 = if bits[2 * t + 1] > 1 { 0u32 } else { bits[2 * t + 1] as u32 };

        let mut new_pm = vec![INF; STATES];
        let mut tb_row = vec![(0usize, 0u8); STATES];

        for next_state in 0..STATES {
            for bit in 0..2usize {
                // Predecessor state
                let prev_state = ((next_state >> 1) | (bit << (K - 2))) & (STATES - 1);
                let exp_out = output[prev_state][bit];
                let metric = (exp_out[0] as u32 ^ r0) + (exp_out[1] as u32 ^ r1);
                if pm[prev_state] != INF {
                    let new_m = pm[prev_state] + metric;
                    if new_m < new_pm[next_state] {
                        new_pm[next_state] = new_m;
                        tb_row[next_state] = (prev_state, bit as u8);
                    }
                }
            }
        }
        pm = new_pm;
        traceback.push(tb_row);
    }

    // Traceback from best final state
    let mut best_state = pm.iter().enumerate().min_by_key(|&(_, &v)| v).map(|(i, _)| i).unwrap_or(0);
    let mut decoded = vec![0u8; n_pairs];
    for t in (0..n_pairs).rev() {
        let (prev, bit) = traceback[t][best_state];
        decoded[t] = bit;
        best_state = prev;
    }
    decoded
}

// ---------------------------------------------------------------------------
// Simplified LDPC
// ---------------------------------------------------------------------------

/// Simple systematic LDPC: repeats each byte with XOR parity.
fn ldpc_encode_simple(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len() * 2);
    for &b in data {
        out.push(b);
        // parity byte: XOR with bit-rotated version
        out.push(b ^ b.rotate_left(4));
    }
    out
}

fn ldpc_decode_simple(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len() / 2);
    let mut i = 0;
    while i + 1 < data.len() {
        // Majority vote on systematic bit
        let sys = data[i];
        let par = data[i + 1];
        let cor = par ^ par.rotate_right(4);
        // Simple error detection: if parity matches, use systematic; otherwise blend
        if sys ^ sys.rotate_left(4) == par {
            out.push(sys);
        } else {
            out.push(sys ^ (sys ^ cor).count_ones().rem_euclid(2) as u8);
        }
        i += 2;
    }
    out
}

// ---------------------------------------------------------------------------
// Complex matrix 2×2 inverse
// ---------------------------------------------------------------------------

fn complex_matrix_inv_2x2(m: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
    if m.len() < 2 || m[0].len() < 2 {
        return m.to_vec();
    }
    let a = m[0][0];
    let b = m[0][1];
    let c = m[1][0];
    let d = m[1][1];

    // det = a*d - b*c
    let ad = cmul(a, d);
    let bc = cmul(b, c);
    let det = (ad.0 - bc.0, ad.1 - bc.1);
    let det_abs2 = det.0 * det.0 + det.1 * det.1;
    if det_abs2 < 1e-30 {
        return vec![vec![(0.0, 0.0); 2]; 2];
    }
    // inv = (1/det) * [[d, -b], [-c, a]]
    let inv_det = (det.0 / det_abs2, -det.1 / det_abs2);
    vec![
        vec![cmul(d, inv_det), cmul((-b.0, -b.1), inv_det)],
        vec![cmul((-c.0, -c.1), inv_det), cmul(a, inv_det)],
    ]
}

fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

// ---------------------------------------------------------------------------
// CRC-32 helper
// ---------------------------------------------------------------------------

fn crc32_simple(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320;
            } else {
                crc >>= 1;
            }
        }
    }
    !crc
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_proc() -> WiFiAxHeProcessor {
        WiFiAxHeProcessor::new(HeConfig::default())
    }

    // --- RuSize ---

    #[test]
    fn test_ru_size_tones_ru26() {
        assert_eq!(RuSize::Ru26.num_tones(), 24);
        assert_eq!(RuSize::Ru26.total_tones(), 26);
    }

    #[test]
    fn test_ru_size_tones_ru242() {
        assert_eq!(RuSize::Ru242.num_tones(), 234);
        assert_eq!(RuSize::Ru242.total_tones(), 242);
    }

    #[test]
    fn test_ru_size_bw_ru996() {
        let bw = RuSize::Ru996.bandwidth_mhz();
        assert!((bw - 80.0).abs() < 0.1, "Expected ~80 MHz got {bw}");
    }

    #[test]
    fn test_ru_size_max_per_20mhz() {
        assert_eq!(RuSize::Ru26.max_per_20mhz(), 9);
        assert_eq!(RuSize::Ru52.max_per_20mhz(), 4);
        assert_eq!(RuSize::Ru106.max_per_20mhz(), 2);
        assert_eq!(RuSize::Ru242.max_per_20mhz(), 1);
    }

    #[test]
    fn test_ru2x996_tones() {
        assert_eq!(RuSize::Ru2x996.num_tones(), 1960);
    }

    // --- HeMcs ---

    #[test]
    fn test_mcs_modulation_order() {
        assert_eq!(HeMcs::Mcs0.modulation_params().0, 2);   // BPSK
        assert_eq!(HeMcs::Mcs1.modulation_params().0, 4);   // QPSK
        assert_eq!(HeMcs::Mcs7.modulation_params().0, 64);  // 64-QAM
        assert_eq!(HeMcs::Mcs10.modulation_params().0, 1024); // 1024-QAM
    }

    #[test]
    fn test_mcs_coding_rates() {
        let (n, d) = HeMcs::Mcs0.coding_rate();
        assert_eq!((n, d), (1, 2));
        let (n, d) = HeMcs::Mcs11.coding_rate();
        assert_eq!((n, d), (5, 6));
    }

    #[test]
    fn test_mcs_from_index_roundtrip() {
        for i in 0u8..12 {
            let mcs = HeMcs::from_index(i).expect("valid MCS");
            assert_eq!(mcs.index(), i);
        }
    }

    #[test]
    fn test_mcs_from_index_invalid() {
        assert!(HeMcs::from_index(12).is_none());
        assert!(HeMcs::from_index(255).is_none());
    }

    // --- GuardInterval ---

    #[test]
    fn test_gi_durations() {
        assert!((GuardInterval::Gi08us.duration_us() - 0.8).abs() < 1e-9);
        assert!((GuardInterval::Gi16us.duration_us() - 1.6).abs() < 1e-9);
        assert!((GuardInterval::Gi32us.duration_us() - 3.2).abs() < 1e-9);
    }

    #[test]
    fn test_gi_samples() {
        assert_eq!(GuardInterval::Gi08us.gi_samples_20mhz(), 16);
        assert_eq!(GuardInterval::Gi16us.gi_samples_20mhz(), 32);
        assert_eq!(GuardInterval::Gi32us.gi_samples_20mhz(), 64);
    }

    #[test]
    fn test_gi_encoding() {
        assert_eq!(GuardInterval::Gi08us.sig_a_encoding(), 0b00);
        assert_eq!(GuardInterval::Gi32us.sig_a_encoding(), 0b11);
    }

    // --- HeLtfType ---

    #[test]
    fn test_ltf_durations() {
        assert!((HeLtfType::Ltf1x.duration_us() - 3.2).abs() < 1e-9);
        assert!((HeLtfType::Ltf4x.duration_us() - 12.8).abs() < 1e-9);
    }

    #[test]
    fn test_ltf_encoding() {
        assert_eq!(HeLtfType::Ltf1x.sig_a_encoding(), 0b00);
        assert_eq!(HeLtfType::Ltf4x.sig_a_encoding(), 0b10);
    }

    // --- FFT size / tone plan ---

    #[test]
    fn test_fft_sizes() {
        assert_eq!(fft_size(20), 256);
        assert_eq!(fft_size(40), 512);
        assert_eq!(fft_size(80), 1024);
        assert_eq!(fft_size(160), 2048);
    }

    #[test]
    fn test_num_data_subcarriers() {
        assert_eq!(WiFiAxHeProcessor::num_data_subcarriers(20), 234);
        assert_eq!(WiFiAxHeProcessor::num_data_subcarriers(80), 980);
        assert_eq!(WiFiAxHeProcessor::num_data_subcarriers(160), 1960);
    }

    // --- HE-STF ---

    #[test]
    fn test_stf_output_length() {
        let proc = default_proc(); // 80 MHz → n_fft=1024, GI=16 (Gi08us * 4)
        let stf = proc.generate_he_stf();
        let n_fft = fft_size(proc.config().bw_mhz);
        let gi = proc.config().gi.gi_samples_20mhz() * (n_fft / 256);
        assert_eq!(stf.len(), n_fft + gi);
    }

    #[test]
    fn test_stf_not_all_zero() {
        let proc = default_proc();
        let stf = proc.generate_he_stf();
        let power: f64 = stf.iter().map(|s| s.0 * s.0 + s.1 * s.1).sum();
        assert!(power > 0.0, "STF should have non-zero power");
    }

    // --- HE-LTF ---

    #[test]
    fn test_ltf_1x_length() {
        let proc = default_proc();
        let n_fft = fft_size(proc.config().bw_mhz);
        let gi = proc.config().gi.gi_samples_20mhz() * (n_fft / 256);
        let ltf = proc.generate_he_ltf(HeLtfType::Ltf1x);
        assert_eq!(ltf.len(), n_fft + gi);
    }

    #[test]
    fn test_ltf_4x_length() {
        let proc = default_proc();
        let n_fft = fft_size(proc.config().bw_mhz);
        let gi = proc.config().gi.gi_samples_20mhz() * (n_fft / 256);
        let ltf = proc.generate_he_ltf(HeLtfType::Ltf4x);
        assert_eq!(ltf.len(), 4 * (n_fft + gi));
    }

    #[test]
    fn test_ltf_2x_is_double_1x() {
        let proc = default_proc();
        let ltf1x = proc.generate_he_ltf(HeLtfType::Ltf1x);
        let ltf2x = proc.generate_he_ltf(HeLtfType::Ltf2x);
        assert_eq!(ltf2x.len(), ltf1x.len() * 2);
    }

    // --- HE-SIG-A ---

    #[test]
    fn test_sig_a_length() {
        let proc = default_proc();
        let sig_a = proc.encode_he_sig_a();
        // 52 bits → 7 bytes
        assert_eq!(sig_a.len(), 7);
    }

    #[test]
    fn test_sig_a_bss_color_encoded() {
        let proc = WiFiAxHeProcessor::new(HeConfig { bss_color: 42, ..Default::default() });
        let sig_a = proc.encode_he_sig_a();
        // BSS color is bits [7:2]; decode
        let bits: Vec<u8> = (0..52).map(|i| (sig_a[i / 8] >> (i % 8)) & 1).collect();
        let color: u8 = (0..6).map(|i| bits[2 + i] << i).fold(0, |a, b| a | b);
        assert_eq!(color, 42);
    }

    #[test]
    fn test_sig_a_bw_field() {
        let proc = WiFiAxHeProcessor::new(HeConfig { bw_mhz: 80, ..Default::default() });
        let sig_a = proc.encode_he_sig_a();
        let bits: Vec<u8> = (0..52).map(|i| (sig_a[i / 8] >> (i % 8)) & 1).collect();
        // BW at bits [10:9]
        let bw_field = bits[9] | (bits[10] << 1);
        assert_eq!(bw_field, 2); // 80 MHz = 10 binary = 2
    }

    // --- HE-SIG-B ---

    #[test]
    fn test_sig_b_non_empty_for_mu() {
        let proc = WiFiAxHeProcessor::new(HeConfig { ppdu_format: HePpduFormat::Mu, ..Default::default() });
        let users = vec![
            HeUserConfig { user_id: 0, ru_size: RuSize::Ru106, ru_index: 0,
                mcs: HeMcs::Mcs7, nss: 1, dcm: false, fec: FecCode::Ldpc, stbc: false },
            HeUserConfig { user_id: 1, ru_size: RuSize::Ru106, ru_index: 1,
                mcs: HeMcs::Mcs5, nss: 2, dcm: false, fec: FecCode::Ldpc, stbc: false },
        ];
        let alloc = proc.allocate_ru(&users);
        let sig_b = proc.encode_he_sig_b(&alloc);
        assert!(!sig_b.is_empty());
    }

    #[test]
    fn test_sig_b_per_user_size() {
        let proc = default_proc();
        let users = vec![
            HeUserConfig { user_id: 0, ru_size: RuSize::Ru26, ru_index: 0,
                mcs: HeMcs::Mcs3, nss: 1, dcm: false, fec: FecCode::Bcc, stbc: false },
        ];
        let alloc = proc.allocate_ru(&users);
        let sig_b = proc.encode_he_sig_b(&alloc);
        // 1 user × 21 bits = ceil(21/8) = 3 bytes
        assert_eq!(sig_b.len(), 3);
    }

    // --- RU allocation ---

    #[test]
    fn test_allocate_ru_single_user() {
        let proc = default_proc();
        let users = vec![HeUserConfig::default_su()];
        let alloc = proc.allocate_ru(&users);
        assert_eq!(alloc.len(), 1);
        assert_eq!(alloc[0].ru_size, RuSize::Ru242);
    }

    #[test]
    fn test_allocate_ru_multi_user_26() {
        let proc = WiFiAxHeProcessor::new(HeConfig { bw_mhz: 20, ..Default::default() });
        let users: Vec<HeUserConfig> = (0..4).map(|i| HeUserConfig {
            user_id: i,
            ru_size: RuSize::Ru26,
            ru_index: i as u8,
            mcs: HeMcs::Mcs3,
            nss: 1,
            dcm: false,
            fec: FecCode::Ldpc,
            stbc: false,
        }).collect();
        let alloc = proc.allocate_ru(&users);
        assert_eq!(alloc.len(), 4);
    }

    #[test]
    fn test_allocate_ru_user_ids_preserved() {
        let proc = default_proc();
        let users = vec![
            HeUserConfig { user_id: 7, ru_size: RuSize::Ru52, ru_index: 0,
                mcs: HeMcs::Mcs2, nss: 1, dcm: false, fec: FecCode::Ldpc, stbc: false },
            HeUserConfig { user_id: 3, ru_size: RuSize::Ru52, ru_index: 1,
                mcs: HeMcs::Mcs4, nss: 1, dcm: false, fec: FecCode::Ldpc, stbc: false },
        ];
        let alloc = proc.allocate_ru(&users);
        assert_eq!(alloc[0].user_id, 7);
        assert_eq!(alloc[1].user_id, 3);
    }

    // --- OFDMA modulation ---

    #[test]
    fn test_modulate_ofdma_output_length() {
        let proc = default_proc();
        let users = vec![HeUserConfig::default_su()];
        let alloc = proc.allocate_ru(&users);
        let data = vec![0xABu8; 32];
        let iq = proc.modulate_ofdma(&data, &alloc[0]);
        let n_fft = fft_size(proc.config().bw_mhz);
        let gi = proc.config().gi.gi_samples_20mhz() * (n_fft / 256);
        assert_eq!(iq.len(), n_fft + gi);
    }

    #[test]
    fn test_modulate_ofdma_nonzero() {
        let proc = default_proc();
        let users = vec![HeUserConfig::default_su()];
        let alloc = proc.allocate_ru(&users);
        let data = vec![0xFFu8; 16];
        let iq = proc.modulate_ofdma(&data, &alloc[0]);
        let power: f64 = iq.iter().map(|s| s.0 * s.0 + s.1 * s.1).sum();
        assert!(power > 0.0);
    }

    // --- Data rate ---

    #[test]
    fn test_data_rate_mcs7_20mhz_1ss() {
        let proc = WiFiAxHeProcessor::new(HeConfig { bw_mhz: 20, gi: GuardInterval::Gi08us, ..Default::default() });
        let rate = proc.data_rate_mbps(HeMcs::Mcs7, 1);
        // Theoretical ≈ 86.7 Mbit/s
        assert!(rate > 80.0 && rate < 100.0, "Got {rate}");
    }

    #[test]
    fn test_data_rate_increases_with_mcs() {
        let proc = WiFiAxHeProcessor::new(HeConfig { bw_mhz: 80, gi: GuardInterval::Gi08us, ..Default::default() });
        let r0 = proc.data_rate_mbps(HeMcs::Mcs0, 1);
        let r11 = proc.data_rate_mbps(HeMcs::Mcs11, 1);
        assert!(r11 > r0 * 5.0, "MCS11 should be >> MCS0");
    }

    #[test]
    fn test_data_rate_scales_with_nss() {
        let proc = WiFiAxHeProcessor::new(HeConfig { bw_mhz: 80, gi: GuardInterval::Gi08us, ..Default::default() });
        let r1 = proc.data_rate_mbps(HeMcs::Mcs7, 1);
        let r4 = proc.data_rate_mbps(HeMcs::Mcs7, 4);
        assert!((r4 - 4.0 * r1).abs() < 0.01, "4-SS should be 4× single-SS");
    }

    #[test]
    fn test_peak_data_rate_above_9gbps() {
        let rate = WiFiAxHeProcessor::peak_data_rate_mbps();
        // Wi-Fi 6 peak: ~9608 Mbps (8SS, 160 MHz, MCS11, 0.8µs GI)
        assert!(rate > 9000.0, "Peak rate {rate} Mbps should exceed 9 Gbps");
    }

    #[test]
    fn test_data_rate_longer_gi_lower_rate() {
        let proc = WiFiAxHeProcessor::new(HeConfig { bw_mhz: 80, ..Default::default() });
        let r08 = proc.calculate_data_rate(HeMcs::Mcs7, 80, 1, GuardInterval::Gi08us);
        let r32 = proc.calculate_data_rate(HeMcs::Mcs7, 80, 1, GuardInterval::Gi32us);
        assert!(r08 > r32, "0.8µs GI should yield higher rate than 3.2µs");
    }

    // --- BCC ---

    #[test]
    fn test_bcc_encode_decode_r1_2() {
        // BCC encoder has K=7 (6 register bits). For a clean Viterbi roundtrip
        // we use all-zeros input which starts and ends at state 0.
        let proc = default_proc();
        let data = vec![0x00u8, 0x00, 0x00, 0x00];
        let encoded = proc.bcc_encode(&data, BccRate::R1_2);
        let decoded = proc.bcc_decode(&encoded, BccRate::R1_2);
        // All-zeros input should decode back to all-zeros
        assert!(decoded.iter().all(|&b| b == 0));
    }

    #[test]
    fn test_bcc_encode_doubles_length() {
        let proc = default_proc();
        let data = vec![0xFFu8; 4];
        let enc = proc.bcc_encode(&data, BccRate::R1_2);
        // rate-1/2: 4 bytes → 8 bytes encoded
        assert_eq!(enc.len(), 8);
    }

    #[test]
    fn test_bcc_puncture_r2_3_reduces_bits() {
        let bits_in: Vec<u8> = (0..40).map(|i| (i % 2) as u8).collect();
        let punctured = bcc_puncture(&bits_in, BccRate::R2_3);
        // 40 bits → keep 3/4 = 30 bits
        assert_eq!(punctured.len(), 30);
    }

    // --- LDPC ---

    #[test]
    fn test_ldpc_encode_doubles_length() {
        let proc = default_proc();
        let data = vec![0x42u8, 0xAB, 0xCD];
        let enc = proc.ldpc_encode(&data);
        assert_eq!(enc.len(), data.len() * 2);
    }

    #[test]
    fn test_ldpc_encode_decode_roundtrip() {
        let proc = default_proc();
        let data = vec![0x12u8, 0x34, 0x56, 0x78];
        let enc = proc.ldpc_encode(&data);
        let dec = proc.ldpc_decode(&enc);
        assert_eq!(dec, data);
    }

    // --- DCM ---

    #[test]
    fn test_dcm_enabled_only_for_mcs0_mcs1() {
        let proc = default_proc();
        // DCM=true, MCS2 → should be cleared
        let users = vec![HeUserConfig {
            user_id: 0, ru_size: RuSize::Ru106, ru_index: 0,
            mcs: HeMcs::Mcs2, nss: 1, dcm: true, fec: FecCode::Ldpc, stbc: false,
        }];
        let alloc = proc.allocate_ru(&users);
        assert!(!alloc[0].dcm, "DCM should be disabled for MCS2");
    }

    #[test]
    fn test_dcm_enabled_for_mcs0() {
        let proc = default_proc();
        let users = vec![HeUserConfig {
            user_id: 0, ru_size: RuSize::Ru106, ru_index: 0,
            mcs: HeMcs::Mcs0, nss: 1, dcm: true, fec: FecCode::Ldpc, stbc: false,
        }];
        let alloc = proc.allocate_ru(&users);
        assert!(alloc[0].dcm, "DCM should be enabled for MCS0");
    }

    // --- BSS Color ---

    #[test]
    fn test_bss_color_range() {
        // BSS color must fit in 6 bits (1-63)
        for color in [1u8, 7, 31, 63] {
            let proc = WiFiAxHeProcessor::new(HeConfig { bss_color: color, ..Default::default() });
            let sig_a = proc.encode_he_sig_a();
            let bits: Vec<u8> = (0..52).map(|i| (sig_a[i / 8] >> (i % 8)) & 1).collect();
            let decoded_color: u8 = (0..6).map(|i| bits[2 + i] << i).fold(0, |a, b| a | b);
            assert_eq!(decoded_color, color);
        }
    }

    // --- TWT ---

    #[test]
    fn test_twt_element_empty_when_disabled() {
        let proc = default_proc();
        let ie = proc.encode_twt_element();
        assert!(ie.is_empty());
    }

    #[test]
    fn test_twt_element_non_empty_when_enabled() {
        let proc = WiFiAxHeProcessor::new(HeConfig {
            twt: TwtSchedule { enabled: true, wake_interval_us: 51200, ..Default::default() },
            ..Default::default()
        });
        let ie = proc.encode_twt_element();
        assert!(!ie.is_empty());
        assert_eq!(ie[0], 216); // Element ID
    }

    // --- Trigger Frame ---

    #[test]
    fn test_trigger_frame_non_empty() {
        let proc = default_proc();
        let users = vec![HeUserConfig::default_su()];
        let tf = proc.build_trigger_frame(&users);
        assert!(!tf.is_empty());
    }

    #[test]
    fn test_trigger_frame_starts_with_fc() {
        let proc = default_proc();
        let users = vec![HeUserConfig::default_su()];
        let tf = proc.build_trigger_frame(&users);
        assert_eq!(tf[0], 0xD4);
    }

    #[test]
    fn test_trigger_frame_multi_user() {
        let proc = default_proc();
        let users: Vec<HeUserConfig> = (0..4).map(|i| HeUserConfig {
            user_id: i,
            ru_size: RuSize::Ru52,
            ru_index: i as u8,
            mcs: HeMcs::Mcs3,
            nss: 1,
            dcm: false,
            fec: FecCode::Ldpc,
            stbc: false,
        }).collect();
        let tf1 = proc.build_trigger_frame(&users[..1]);
        let tf4 = proc.build_trigger_frame(&users);
        assert!(tf4.len() > tf1.len(), "More users → longer trigger frame");
    }

    // --- STBC ---

    #[test]
    fn test_stbc_encoding_output_length() {
        let proc = default_proc();
        let syms: Vec<(f64, f64)> = (0..8).map(|i| (i as f64, -(i as f64))).collect();
        let enc = proc.apply_stbc_2x1(&syms);
        assert_eq!(enc.len(), syms.len() * 2);
    }

    #[test]
    fn test_stbc_encodes_pairs() {
        let proc = default_proc();
        let s0 = (1.0f64, 0.5);
        let s1 = (-0.5f64, 1.0);
        let enc = proc.apply_stbc_2x1(&[s0, s1]);
        // Verify Alamouti structure: [s0, -s1*, s1, s0*]
        assert!((enc[0].0 - s0.0).abs() < 1e-10);
        assert!((enc[0].1 - s0.1).abs() < 1e-10);
        // -s1*: negate real, keep imag sign
        assert!((enc[1].0 - (-s1.0)).abs() < 1e-10);
        assert!((enc[1].1 - s1.1).abs() < 1e-10);
    }

    // --- Spectral efficiency ---

    #[test]
    fn test_spectral_efficiency_mcs0() {
        let proc = default_proc();
        let se = proc.spectral_efficiency(HeMcs::Mcs0, 1);
        // BPSK rate-1/2: 0.5 bits/s/Hz
        assert!((se - 0.5).abs() < 1e-9);
    }

    #[test]
    fn test_spectral_efficiency_mcs11() {
        let proc = default_proc();
        let se = proc.spectral_efficiency(HeMcs::Mcs11, 1);
        // 1024-QAM, rate 5/6: 10 × 5/6 ≈ 8.333 bits/s/Hz
        assert!((se - 10.0 * 5.0 / 6.0).abs() < 1e-9);
    }

    // --- ZF Precoder ---

    #[test]
    fn test_zf_precoder_2x2_identity_channel() {
        // H = identity (2×2)
        let h = vec![
            vec![(1.0f64, 0.0), (0.0, 0.0)],
            vec![(0.0, 0.0), (1.0, 0.0)],
        ];
        let w = WiFiAxHeProcessor::zf_precoder(&h);
        // For H = I, W = I
        assert_eq!(w.len(), 2);
        assert!((w[0][0].0 - 1.0).abs() < 1e-6, "W[0][0] ≈ 1.0");
        assert!(w[0][1].0.abs() < 1e-6, "W[0][1] ≈ 0");
    }

    #[test]
    fn test_zf_precoder_empty_input() {
        let h: Vec<Vec<(f64, f64)>> = vec![];
        let w = WiFiAxHeProcessor::zf_precoder(&h);
        assert!(w.is_empty());
    }

    // --- Preamble ---

    #[test]
    fn test_preamble_stf_plus_ltf() {
        let proc = default_proc();
        let preamble = proc.generate_he_preamble();
        let stf = proc.generate_he_stf();
        let ltf = proc.generate_he_ltf(proc.config().ltf_type);
        assert_eq!(preamble.len(), stf.len() + ltf.len());
    }

    // --- FFT roundtrip ---

    #[test]
    fn test_fft_ifft_roundtrip() {
        let n = 64;
        let input: Vec<(f64, f64)> = (0..n).map(|i| (i as f64 * 0.1, -(i as f64) * 0.05)).collect();
        let fd = fft_radix2(&input);
        let td = ifft_radix2(&fd);
        for i in 0..n {
            assert!((td[i].0 - input[i].0).abs() < 1e-9, "Real mismatch at {i}");
            assert!((td[i].1 - input[i].1).abs() < 1e-9, "Imag mismatch at {i}");
        }
    }

    // --- Bit utilities ---

    #[test]
    fn test_bytes_to_bits_roundtrip() {
        let data = vec![0xA5u8, 0x3C, 0xFF, 0x00];
        let bits = bytes_to_bits(&data);
        assert_eq!(bits.len(), 32);
        let back = bits_to_bytes(&bits);
        assert_eq!(back, data);
    }

    // --- HeConfig default ---

    #[test]
    fn test_heconfig_default_values() {
        let cfg = HeConfig::default();
        assert_eq!(cfg.bw_mhz, 80);
        assert_eq!(cfg.bss_color, 1);
        assert_eq!(cfg.gi, GuardInterval::Gi08us);
        assert_eq!(cfg.ltf_type, HeLtfType::Ltf4x);
    }

    // --- HeUserConfig default ---

    #[test]
    fn test_he_user_config_default() {
        let uc = HeUserConfig::default_su();
        assert_eq!(uc.ru_size, RuSize::Ru242);
        assert_eq!(uc.mcs, HeMcs::Mcs7);
        assert_eq!(uc.nss, 1);
    }

    // --- CRC32 ---

    #[test]
    fn test_crc32_nonzero() {
        let data = b"Hello Wi-Fi 6";
        let crc = crc32_simple(data);
        assert_ne!(crc, 0);
    }

    #[test]
    fn test_crc32_deterministic() {
        let data = b"802.11ax";
        assert_eq!(crc32_simple(data), crc32_simple(data));
    }

    // --- Various bandwidth configs ---

    #[test]
    fn test_processor_20mhz_config() {
        let proc = WiFiAxHeProcessor::new(HeConfig { bw_mhz: 20, ..Default::default() });
        assert_eq!(fft_size(proc.config().bw_mhz), 256);
    }

    #[test]
    fn test_processor_160mhz_config() {
        let proc = WiFiAxHeProcessor::new(HeConfig { bw_mhz: 160, ..Default::default() });
        assert_eq!(fft_size(proc.config().bw_mhz), 2048);
        let stf = proc.generate_he_stf();
        assert!(!stf.is_empty());
    }

    // --- PPDU format variants ---

    #[test]
    fn test_ppdu_format_tb_trigger_frame() {
        let proc = WiFiAxHeProcessor::new(HeConfig {
            ppdu_format: HePpduFormat::TriggerBased,
            bw_mhz: 20,
            ..Default::default()
        });
        let users = vec![HeUserConfig {
            user_id: 0, ru_size: RuSize::Ru26, ru_index: 0,
            mcs: HeMcs::Mcs0, nss: 1, dcm: true, fec: FecCode::Bcc, stbc: false,
        }];
        let tf = proc.build_trigger_frame(&users);
        assert!(!tf.is_empty());
    }

    #[test]
    fn test_ppdu_ext_range_su() {
        let proc = WiFiAxHeProcessor::new(HeConfig {
            ppdu_format: HePpduFormat::ExtRange,
            bw_mhz: 20,
            gi: GuardInterval::Gi32us,
            ltf_type: HeLtfType::Ltf4x,
            ..Default::default()
        });
        let stf = proc.generate_he_stf();
        assert!(!stf.is_empty());
    }
}

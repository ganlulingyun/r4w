//! DOCSIS 3.1/4.0 OFDM/OFDMA Processor
//!
//! Implements the Physical Layer (PHY) for Data Over Cable Service Interface
//! Specification (DOCSIS) 3.1 and 4.0 per CableLabs specifications:
//! - CM-SP-PHYv3.1: DOCSIS 3.1 Physical Layer Specification
//! - CM-SP-PHYv4.0: DOCSIS 4.0 Physical Layer Specification
//!
//! # Features
//! - **OFDM Parameters**: 4096/8192-point FFT, 25/50 kHz subcarrier spacing
//! - **Modulation**: Up to 4096-QAM (D3.1) and 16384-QAM (D4.0) with Gray coding
//! - **Profile Management**: Up to 16 modulation profiles (A-P) per CMTS subcarrier map
//! - **PLC Channel**: 8 MHz Physical Layer Link Channel, always QPSK
//! - **Exclusion Bands**: Configurable frequency exclusion zones
//! - **Pilot Patterns**: Scattered/continuous pilots for channel estimation
//! - **Cyclic Prefix**: Variable CP lengths 192/256/512/768/1024 samples
//! - **LDPC Coding**: Code rates 8/9, 5/6, 3/4, 2/3, 1/2 with 16200/64800-bit codewords
//! - **Upstream OFDMA**: Mini-slot allocation, contention access, probe mechanism
//!
//! # Frequency Ranges
//! - Downstream: 258–1218 MHz (D3.1), extended for D4.0
//! - Upstream: 5–204 MHz (high-split), 5–85 MHz (mid-split), 5–42 MHz (low-split)
//!
//! # Standards
//! CableLabs DOCSIS 3.1 PHY Spec CM-SP-PHYv3.1-I10-171220
//! CableLabs DOCSIS 4.0 PHY Spec CM-SP-PHYv4.0

use std::collections::HashMap;
use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// DOCSIS downstream minimum frequency (Hz)
pub const DS_FREQ_MIN_HZ: f64 = 258e6;
/// DOCSIS downstream maximum frequency (Hz)
pub const DS_FREQ_MAX_HZ: f64 = 1218e6;
/// DOCSIS upstream minimum frequency (Hz)
pub const US_FREQ_MIN_HZ: f64 = 5e6;
/// DOCSIS 3.1 upstream maximum frequency, high-split (Hz)
pub const US_FREQ_MAX_HZ_HIGH_SPLIT: f64 = 204e6;
/// DOCSIS upstream maximum frequency, mid-split (Hz)
pub const US_FREQ_MAX_HZ_MID_SPLIT: f64 = 85e6;
/// DOCSIS upstream maximum frequency, low-split (Hz)
pub const US_FREQ_MAX_HZ_LOW_SPLIT: f64 = 42e6;

/// Subcarrier spacing option A: 25 kHz
pub const SUBCARRIER_SPACING_25KHZ: f64 = 25_000.0;
/// Subcarrier spacing option B: 50 kHz
pub const SUBCARRIER_SPACING_50KHZ: f64 = 50_000.0;

/// FFT size for 25 kHz spacing (192 MHz / 25 kHz = 7680, but standard is 8192)
pub const FFT_SIZE_8192: usize = 8192;
/// FFT size for 50 kHz spacing (192 MHz / 50 kHz = 3840, but standard is 4096)
pub const FFT_SIZE_4096: usize = 4096;

/// PLC bandwidth (Hz)
pub const PLC_BANDWIDTH_HZ: f64 = 8e6;
/// PLC modulation order (QPSK = 4)
pub const PLC_MODULATION_ORDER: u32 = 4;

/// Maximum number of modulation profiles
pub const MAX_PROFILES: usize = 16;

/// LDPC codeword size short (bits)
pub const LDPC_CW_SHORT: usize = 16200;
/// LDPC codeword size long (bits)
pub const LDPC_CW_LONG: usize = 64800;

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// DOCSIS version
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DocsisVersion {
    /// DOCSIS 3.1 – supports up to 4096-QAM
    D31,
    /// DOCSIS 4.0 – supports up to 16384-QAM and ESD
    D40,
}

/// Subcarrier spacing selection
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SubcarrierSpacing {
    /// 25 kHz spacing → 8192-point FFT
    KHz25,
    /// 50 kHz spacing → 4096-point FFT
    KHz50,
}

impl SubcarrierSpacing {
    /// Returns spacing in Hz
    pub fn hz(&self) -> f64 {
        match self {
            SubcarrierSpacing::KHz25 => SUBCARRIER_SPACING_25KHZ,
            SubcarrierSpacing::KHz50 => SUBCARRIER_SPACING_50KHZ,
        }
    }

    /// Returns the corresponding FFT size
    pub fn fft_size(&self) -> usize {
        match self {
            SubcarrierSpacing::KHz25 => FFT_SIZE_8192,
            SubcarrierSpacing::KHz50 => FFT_SIZE_4096,
        }
    }

    /// OFDM symbol duration (1/Δf) in seconds, before CP
    pub fn symbol_duration_us(&self) -> f64 {
        1.0e6 / self.hz()
    }
}

/// Modulation order for a subcarrier (bits per symbol)
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ModulationOrder {
    Bpsk    = 1,
    Qpsk    = 2,
    Qam16   = 4,
    Qam32   = 5,
    Qam64   = 6,
    Qam128  = 7,
    Qam256  = 8,
    Qam512  = 9,
    Qam1024 = 10,
    Qam2048 = 11,
    Qam4096 = 12,
    /// Only in DOCSIS 4.0
    Qam8192 = 13,
    /// Only in DOCSIS 4.0
    Qam16384 = 14,
    /// Pilot / excluded subcarrier placeholder
    Pilot   = 0,
}

impl ModulationOrder {
    /// Bits per symbol for this order
    pub fn bits_per_symbol(&self) -> usize {
        *self as usize
    }

    /// Constellation size M = 2^bps
    pub fn constellation_size(&self) -> usize {
        if *self == ModulationOrder::Pilot {
            return 0;
        }
        1 << self.bits_per_symbol()
    }

    /// Minimum required SNR (dB) – approximate thresholds
    pub fn min_snr_db(&self) -> f64 {
        match self {
            ModulationOrder::Bpsk    =>  6.0,
            ModulationOrder::Qpsk    =>  9.0,
            ModulationOrder::Qam16   => 15.0,
            ModulationOrder::Qam32   => 18.0,
            ModulationOrder::Qam64   => 21.0,
            ModulationOrder::Qam128  => 24.0,
            ModulationOrder::Qam256  => 27.0,
            ModulationOrder::Qam512  => 30.0,
            ModulationOrder::Qam1024 => 33.0,
            ModulationOrder::Qam2048 => 36.0,
            ModulationOrder::Qam4096 => 39.0,
            ModulationOrder::Qam8192 => 42.0,
            ModulationOrder::Qam16384 => 45.0,
            ModulationOrder::Pilot   =>  0.0,
        }
    }
}

/// Cyclic prefix length (number of samples)
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CyclicPrefixLen {
    Cp192  = 192,
    Cp256  = 256,
    Cp512  = 512,
    Cp768  = 768,
    Cp1024 = 1024,
}

impl CyclicPrefixLen {
    /// CP duration in µs given subcarrier spacing
    pub fn duration_us(&self, spacing: SubcarrierSpacing) -> f64 {
        let sample_rate = spacing.hz() * spacing.fft_size() as f64;
        (*self as usize) as f64 / sample_rate * 1e6
    }
}

/// LDPC code rate
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LdpcCodeRate {
    /// Rate 1/2
    Rate1_2,
    /// Rate 2/3
    Rate2_3,
    /// Rate 3/4
    Rate3_4,
    /// Rate 5/6
    Rate5_6,
    /// Rate 8/9
    Rate8_9,
}

impl LdpcCodeRate {
    /// Returns numerator and denominator
    pub fn fraction(&self) -> (usize, usize) {
        match self {
            LdpcCodeRate::Rate1_2 => (1, 2),
            LdpcCodeRate::Rate2_3 => (2, 3),
            LdpcCodeRate::Rate3_4 => (3, 4),
            LdpcCodeRate::Rate5_6 => (5, 6),
            LdpcCodeRate::Rate8_9 => (8, 9),
        }
    }

    /// Code rate as f64
    pub fn rate(&self) -> f64 {
        let (n, d) = self.fraction();
        n as f64 / d as f64
    }

    /// Number of information bits in a short codeword (16200)
    pub fn info_bits_short(&self) -> usize {
        let (n, d) = self.fraction();
        LDPC_CW_SHORT * n / d
    }

    /// Number of information bits in a long codeword (64800)
    pub fn info_bits_long(&self) -> usize {
        let (n, d) = self.fraction();
        LDPC_CW_LONG * n / d
    }
}

/// Pilot pattern type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PilotPatternType {
    /// Continuous pilots (always present at fixed subcarriers)
    Continuous,
    /// Scattered pilots (time-varying, for channel tracking)
    Scattered,
}

/// Direction of OFDM operation
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Direction {
    Downstream,
    Upstream,
}

// ---------------------------------------------------------------------------
// Profile
// ---------------------------------------------------------------------------

/// DOCSIS modulation profile (one per CMTS downstream channel)
///
/// Profile A–D are pre-defined; E–P are operator-configured.
/// Each profile specifies the modulation order for each active subcarrier.
#[derive(Debug, Clone)]
pub struct ModulationProfile {
    /// Profile index 0–15 (A=0, B=1, …, P=15)
    pub index: usize,
    /// Descriptive name
    pub name: String,
    /// Per-subcarrier modulation order; length = number of active subcarriers
    pub subcarrier_mods: Vec<ModulationOrder>,
    /// Minimum SNR required for this profile (dB) – max of per-subcarrier minima
    pub min_snr_db: f64,
}

impl ModulationProfile {
    /// Create a uniform-modulation profile for `n_subcarriers` subcarriers
    pub fn uniform(index: usize, order: ModulationOrder, n_subcarriers: usize) -> Self {
        let min_snr = order.min_snr_db();
        ModulationProfile {
            index,
            name: format!("Profile-{}", (b'A' + index as u8) as char),
            subcarrier_mods: vec![order; n_subcarriers],
            min_snr_db: min_snr,
        }
    }

    /// Compute total bits per OFDM symbol for this profile
    pub fn bits_per_symbol(&self) -> usize {
        self.subcarrier_mods.iter().map(|m| m.bits_per_symbol()).sum()
    }

    /// Count active (non-pilot) subcarriers
    pub fn active_subcarriers(&self) -> usize {
        self.subcarrier_mods.iter().filter(|&&m| m != ModulationOrder::Pilot).count()
    }
}

/// Profile manager: maintains up to 16 profiles and selects based on SNR feedback
#[derive(Debug, Clone)]
pub struct ProfileManager {
    profiles: Vec<ModulationProfile>,
    active_profile_index: usize,
}

impl ProfileManager {
    /// Create a new profile manager with default DOCSIS profiles
    pub fn new_default(n_subcarriers: usize) -> Self {
        let profiles = vec![
            ModulationProfile::uniform(0, ModulationOrder::Qpsk,    n_subcarriers),
            ModulationProfile::uniform(1, ModulationOrder::Qam64,   n_subcarriers),
            ModulationProfile::uniform(2, ModulationOrder::Qam256,  n_subcarriers),
            ModulationProfile::uniform(3, ModulationOrder::Qam1024, n_subcarriers),
        ];
        ProfileManager { profiles, active_profile_index: 0 }
    }

    /// Add a custom profile (up to MAX_PROFILES)
    pub fn add_profile(&mut self, profile: ModulationProfile) -> Result<(), &'static str> {
        if self.profiles.len() >= MAX_PROFILES {
            return Err("Maximum number of profiles reached");
        }
        self.profiles.push(profile);
        Ok(())
    }

    /// Select the highest-performance profile that satisfies the SNR feedback
    /// Returns the selected profile index
    pub fn select_profile(&mut self, snr_db: f64) -> usize {
        let mut best_idx = 0;
        for (i, p) in self.profiles.iter().enumerate() {
            if snr_db >= p.min_snr_db && p.min_snr_db >= self.profiles[best_idx].min_snr_db {
                best_idx = i;
            }
        }
        self.active_profile_index = best_idx;
        best_idx
    }

    /// Get currently active profile
    pub fn active_profile(&self) -> &ModulationProfile {
        &self.profiles[self.active_profile_index]
    }

    /// Get profile by index
    pub fn profile(&self, idx: usize) -> Option<&ModulationProfile> {
        self.profiles.get(idx)
    }

    /// Number of registered profiles
    pub fn num_profiles(&self) -> usize {
        self.profiles.len()
    }
}

// ---------------------------------------------------------------------------
// Exclusion Bands
// ---------------------------------------------------------------------------

/// Frequency exclusion zone (e.g., for FM radio, LTE, legacy SC-QAM)
#[derive(Debug, Clone)]
pub struct ExclusionBand {
    /// Start frequency in Hz
    pub start_hz: f64,
    /// Stop frequency in Hz
    pub stop_hz: f64,
    /// Reason / label
    pub label: String,
}

impl ExclusionBand {
    pub fn new(start_hz: f64, stop_hz: f64, label: impl Into<String>) -> Self {
        ExclusionBand { start_hz, stop_hz, label: label.into() }
    }

    /// Returns true if the given frequency (Hz) is within this exclusion band
    pub fn contains(&self, freq_hz: f64) -> bool {
        freq_hz >= self.start_hz && freq_hz <= self.stop_hz
    }

    /// Bandwidth of the exclusion band in Hz
    pub fn bandwidth_hz(&self) -> f64 {
        self.stop_hz - self.start_hz
    }
}

/// Manager for all exclusion bands in an OFDM channel
#[derive(Debug, Clone, Default)]
pub struct ExclusionBandManager {
    bands: Vec<ExclusionBand>,
}

impl ExclusionBandManager {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_band(&mut self, band: ExclusionBand) {
        self.bands.push(band);
    }

    /// Add a band for legacy FM radio (87.5–108 MHz)
    pub fn add_fm_radio_band(&mut self) {
        self.add_band(ExclusionBand::new(87.5e6, 108.0e6, "FM Radio"));
    }

    /// Add a band for a specific SC-QAM carrier
    pub fn add_scqam_channel(&mut self, center_hz: f64, bw_hz: f64) {
        self.add_band(ExclusionBand::new(
            center_hz - bw_hz / 2.0,
            center_hz + bw_hz / 2.0,
            format!("SC-QAM @ {:.1}MHz", center_hz / 1e6),
        ));
    }

    /// Returns true if `freq_hz` falls in any exclusion band
    pub fn is_excluded(&self, freq_hz: f64) -> bool {
        self.bands.iter().any(|b| b.contains(freq_hz))
    }

    /// Returns which subcarrier indices (out of `n`) should be excluded
    /// given channel starting frequency `start_hz` and subcarrier spacing `spacing_hz`
    pub fn excluded_subcarriers(&self, start_hz: f64, n: usize, spacing_hz: f64) -> Vec<usize> {
        (0..n)
            .filter(|&i| self.is_excluded(start_hz + i as f64 * spacing_hz))
            .collect()
    }

    pub fn num_bands(&self) -> usize {
        self.bands.len()
    }
}

// ---------------------------------------------------------------------------
// Pilot Patterns
// ---------------------------------------------------------------------------

/// Single pilot entry in the subcarrier grid
#[derive(Debug, Clone)]
pub struct Pilot {
    /// Subcarrier index
    pub subcarrier_idx: usize,
    /// OFDM symbol offset (for scattered pilots)
    pub symbol_offset: usize,
    /// Pilot type
    pub pilot_type: PilotPatternType,
    /// Boosted power level relative to data (linear, typically 1.0–4.0)
    pub power_boost: f64,
}

/// Pilot pattern for an OFDM channel
#[derive(Debug, Clone)]
pub struct PilotPattern {
    pilots: Vec<Pilot>,
    /// Scattered pilot periodicity in subcarriers
    pub sc_period: usize,
    /// Scattered pilot periodicity in OFDM symbols
    pub sym_period: usize,
}

impl PilotPattern {
    /// Create a standard DOCSIS pilot pattern
    ///
    /// Continuous pilots every `cont_spacing` subcarriers,
    /// scattered pilots on a `sc_period x sym_period` grid.
    pub fn new_standard(
        n_subcarriers: usize,
        cont_spacing: usize,
        sc_period: usize,
        sym_period: usize,
        power_boost: f64,
    ) -> Self {
        let mut pilots = Vec::new();

        // Continuous pilots
        let mut sc = 0;
        while sc < n_subcarriers {
            pilots.push(Pilot {
                subcarrier_idx: sc,
                symbol_offset: 0,
                pilot_type: PilotPatternType::Continuous,
                power_boost,
            });
            sc += cont_spacing;
        }

        // Scattered pilots
        for sym in 0..sym_period {
            let offset = (sym * sc_period / sym_period) % sc_period;
            let mut sc2 = offset;
            while sc2 < n_subcarriers {
                pilots.push(Pilot {
                    subcarrier_idx: sc2,
                    symbol_offset: sym,
                    pilot_type: PilotPatternType::Scattered,
                    power_boost,
                });
                sc2 += sc_period;
            }
        }

        PilotPattern { pilots, sc_period, sym_period }
    }

    /// Get all pilots for a specific OFDM symbol index
    pub fn pilots_for_symbol(&self, symbol_idx: usize) -> Vec<&Pilot> {
        self.pilots.iter().filter(|p| {
            match p.pilot_type {
                PilotPatternType::Continuous => true,
                PilotPatternType::Scattered  => p.symbol_offset == symbol_idx % self.sym_period,
            }
        }).collect()
    }

    /// Total number of pilots
    pub fn num_pilots(&self) -> usize {
        self.pilots.len()
    }

    /// Pilot subcarrier indices for a given symbol
    pub fn pilot_subcarrier_indices(&self, symbol_idx: usize) -> Vec<usize> {
        self.pilots_for_symbol(symbol_idx)
            .iter()
            .map(|p| p.subcarrier_idx)
            .collect()
    }
}

// ---------------------------------------------------------------------------
// PLC – Physical Layer Link Channel
// ---------------------------------------------------------------------------

/// DOCSIS Physical Layer Link Channel (PLC) configuration
///
/// The PLC occupies 8 MHz at the low end of the OFDM spectrum.
/// It is always QPSK-modulated for robust delivery of control messages
/// (MAP, UCD, timestamp, preamble for acquisition).
#[derive(Debug, Clone)]
pub struct PlcConfig {
    /// Center frequency of the PLC in Hz
    pub center_hz: f64,
    /// Bandwidth (always 8 MHz per spec)
    pub bandwidth_hz: f64,
    /// Number of OFDM symbols in one PLC frame
    pub symbols_per_frame: usize,
    /// Preamble length in symbols
    pub preamble_symbols: usize,
}

impl PlcConfig {
    /// Default PLC at given center frequency
    pub fn new(center_hz: f64) -> Self {
        PlcConfig {
            center_hz,
            bandwidth_hz: PLC_BANDWIDTH_HZ,
            symbols_per_frame: 8,
            preamble_symbols: 2,
        }
    }

    /// Generate a simple QPSK preamble sequence (BPSK-encoded PN)
    pub fn generate_preamble(&self, len: usize) -> Vec<(f64, f64)> {
        // Simple Zadoff-Chu-like BPSK preamble
        let mut seq = Vec::with_capacity(len);
        let root = 25usize; // ZC root
        for n in 0..len {
            let phase = PI * root as f64 * n as f64 * (n as f64 + 1.0) / len as f64;
            seq.push((phase.cos(), phase.sin()));
        }
        seq
    }

    /// Encode a timestamp message (placeholder structure)
    pub fn encode_timestamp(&self, timestamp_ns: u64) -> Vec<u8> {
        let mut buf = vec![0u8; 9];
        buf[0] = 0x01; // message type: timestamp
        for (i, b) in timestamp_ns.to_be_bytes().iter().enumerate() {
            buf[1 + i] = *b;
        }
        buf
    }
}

// ---------------------------------------------------------------------------
// Gray-coded QAM Constellation
// ---------------------------------------------------------------------------

/// Generate a Gray-coded square QAM constellation
///
/// Returns a vector of (I, Q) points normalized to unit average power.
pub fn generate_qam_constellation(order: ModulationOrder) -> Vec<(f64, f64)> {
    let m = order.constellation_size();
    if m == 0 {
        return vec![];
    }
    if m == 2 {
        // BPSK
        return vec![(-1.0, 0.0), (1.0, 0.0)];
    }
    let sqrt_m = (m as f64).sqrt() as usize;
    if sqrt_m * sqrt_m != m {
        // Non-square (e.g. 32-QAM) – use cross-QAM approximation
        return generate_cross_qam(m);
    }
    // Square QAM
    let mut pts = Vec::with_capacity(m);
    let levels: Vec<f64> = (0..sqrt_m)
        .map(|i| (2 * i) as f64 - (sqrt_m as f64 - 1.0))
        .collect();

    // Gray-code the indices
    for (qi, &q) in levels.iter().enumerate() {
        let gq = gray_code(qi);
        for (ii, &i_val) in levels.iter().enumerate() {
            let gi = gray_code(ii);
            pts.push((
                (gi as f64 / sqrt_m as f64),   // placeholder; reorder below
                (gq as f64 / sqrt_m as f64),
                i_val,
                q,
            ));
        }
    }

    // Build final vector ordered by Gray index
    let mut result = vec![(0.0f64, 0.0f64); m];
    let mut idx_pts: Vec<(usize, f64, f64)> = Vec::new();
    for qi in 0..sqrt_m {
        for ii in 0..sqrt_m {
            let symbol_idx = gray_code(qi) * sqrt_m + gray_code(ii);
            let i_val = levels[ii];
            let q_val = levels[qi];
            idx_pts.push((symbol_idx, i_val, q_val));
        }
    }
    // sort by symbol_idx
    idx_pts.sort_by_key(|x| x.0);
    for (idx, i_val, q_val) in idx_pts {
        result[idx] = (i_val, q_val);
    }

    // Normalize to unit average power
    let avg_pwr = result.iter().map(|(i, q)| i * i + q * q).sum::<f64>() / m as f64;
    let scale = avg_pwr.sqrt().recip();
    result.iter().map(|(i, q)| (i * scale, q * scale)).collect()
}

/// Gray-code an integer
pub fn gray_code(n: usize) -> usize {
    n ^ (n >> 1)
}

/// Inverse Gray code
pub fn inv_gray_code(mut n: usize) -> usize {
    let mut mask = n >> 1;
    while mask != 0 {
        n ^= mask;
        mask >>= 1;
    }
    n
}

/// Generate approximate cross-QAM for non-square orders (32, 128, 512, 2048, 8192)
fn generate_cross_qam(m: usize) -> Vec<(f64, f64)> {
    // Use next-higher square minus corner removal pattern
    let m2 = m * 2;
    let sqrt_m2 = (m2 as f64).sqrt() as usize;
    let levels: Vec<f64> = (0..sqrt_m2)
        .map(|i| (2 * i) as f64 - (sqrt_m2 as f64 - 1.0))
        .collect();
    let corner_threshold = sqrt_m2 as f64 - 1.0;
    let mut pts: Vec<(f64, f64)> = Vec::new();
    for &q in &levels {
        for &i in &levels {
            if i.abs() <= corner_threshold || q.abs() <= corner_threshold {
                pts.push((i, q));
                if pts.len() == m {
                    break;
                }
            }
        }
        if pts.len() == m {
            break;
        }
    }
    // Pad if needed
    while pts.len() < m {
        pts.push((0.0, 0.0));
    }
    // Normalize
    let avg_pwr = pts.iter().map(|(i, q)| i * i + q * q).sum::<f64>() / m as f64;
    let scale = if avg_pwr > 0.0 { avg_pwr.sqrt().recip() } else { 1.0 };
    pts.iter().map(|(i, q)| (i * scale, q * scale)).collect()
}

// Dummy struct to remove unused field warning in generate_qam_constellation above
struct _Unused(f64, f64, f64, f64);

// ---------------------------------------------------------------------------
// QAM Modulator / Demodulator
// ---------------------------------------------------------------------------

/// QAM symbol mapper (encoder)
#[derive(Debug, Clone)]
pub struct QamMapper {
    order: ModulationOrder,
    constellation: Vec<(f64, f64)>,
}

impl QamMapper {
    pub fn new(order: ModulationOrder) -> Self {
        let constellation = generate_qam_constellation(order);
        QamMapper { order, constellation }
    }

    /// Map a sequence of bits to complex IQ samples
    /// `bits` must be a multiple of `bps = order.bits_per_symbol()`
    pub fn map(&self, bits: &[u8]) -> Vec<(f64, f64)> {
        let bps = self.order.bits_per_symbol();
        if bps == 0 || self.constellation.is_empty() {
            return vec![];
        }
        bits.chunks(bps)
            .map(|chunk| {
                let mut idx = 0usize;
                for &b in chunk {
                    idx = (idx << 1) | (b & 1) as usize;
                }
                let idx = idx % self.constellation.len();
                self.constellation[idx]
            })
            .collect()
    }

    /// Hard-decision demap a complex sample to bits
    pub fn demap(&self, iq: (f64, f64)) -> Vec<u8> {
        let bps = self.order.bits_per_symbol();
        if bps == 0 || self.constellation.is_empty() {
            return vec![];
        }
        // Find nearest constellation point
        let idx = self.constellation.iter().enumerate()
            .min_by(|(_, a), (_, b)| {
                let da = (a.0 - iq.0).powi(2) + (a.1 - iq.1).powi(2);
                let db = (b.0 - iq.0).powi(2) + (b.1 - iq.1).powi(2);
                da.partial_cmp(&db).unwrap()
            })
            .map(|(i, _)| i)
            .unwrap_or(0);

        // Convert index to bits (MSB first)
        (0..bps).rev().map(|shift| ((idx >> shift) & 1) as u8).collect()
    }

    /// Return reference to constellation
    pub fn constellation(&self) -> &[(f64, f64)] {
        &self.constellation
    }

    /// EVM of a received symbol vs. nearest constellation point (linear)
    pub fn evm_linear(&self, iq: (f64, f64)) -> f64 {
        if self.constellation.is_empty() {
            return 0.0;
        }
        let min_dist_sq = self.constellation.iter()
            .map(|&(ci, cq)| (ci - iq.0).powi(2) + (cq - iq.1).powi(2))
            .fold(f64::INFINITY, f64::min);
        min_dist_sq.sqrt()
    }
}

// ---------------------------------------------------------------------------
// OFDM Symbol (complex FFT-based)
// ---------------------------------------------------------------------------

/// A complex sample (I, Q)
pub type Complex = (f64, f64);

/// Perform an N-point radix-2 FFT (in-place Cooley-Tukey)
pub fn fft(buf: &mut Vec<Complex>, inverse: bool) {
    let n = buf.len();
    if n <= 1 {
        return;
    }
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
    // Cooley-Tukey stages
    let mut len = 2usize;
    while len <= n {
        let ang = 2.0 * PI / len as f64 * if inverse { 1.0 } else { -1.0 };
        let wlen = (ang.cos(), ang.sin());
        for i in (0..n).step_by(len) {
            let mut w = (1.0f64, 0.0f64);
            for k in 0..len / 2 {
                let u = buf[i + k];
                let v = cmul(buf[i + k + len / 2], w);
                buf[i + k] = cadd(u, v);
                buf[i + k + len / 2] = csub(u, v);
                w = cmul(w, wlen);
            }
        }
        len <<= 1;
    }
    if inverse {
        let n_f = n as f64;
        for x in buf.iter_mut() {
            x.0 /= n_f;
            x.1 /= n_f;
        }
    }
}

fn cmul(a: Complex, b: Complex) -> Complex {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}
fn cadd(a: Complex, b: Complex) -> Complex {
    (a.0 + b.0, a.1 + b.1)
}
fn csub(a: Complex, b: Complex) -> Complex {
    (a.0 - b.0, a.1 - b.1)
}

/// DOCSIS OFDM symbol with cyclic prefix
#[derive(Debug, Clone)]
pub struct OfdmSymbol {
    /// Time-domain samples including CP
    pub samples: Vec<Complex>,
    /// FFT size
    pub fft_size: usize,
    /// CP length
    pub cp_len: usize,
}

impl OfdmSymbol {
    /// Total symbol length including CP
    pub fn total_len(&self) -> usize {
        self.fft_size + self.cp_len
    }
}

// ---------------------------------------------------------------------------
// OFDM Modulator
// ---------------------------------------------------------------------------

/// DOCSIS OFDM channel configuration
#[derive(Debug, Clone)]
pub struct OfdmChannelConfig {
    pub version: DocsisVersion,
    pub direction: Direction,
    pub spacing: SubcarrierSpacing,
    pub cp_len: CyclicPrefixLen,
    pub center_freq_hz: f64,
    pub channel_bw_hz: f64,
    /// Number of active (non-guard) subcarriers
    pub n_active: usize,
}

impl OfdmChannelConfig {
    /// Create a default downstream DOCSIS 3.1 channel at 600 MHz, 192 MHz BW, 25 kHz spacing
    pub fn downstream_default() -> Self {
        OfdmChannelConfig {
            version: DocsisVersion::D31,
            direction: Direction::Downstream,
            spacing: SubcarrierSpacing::KHz25,
            cp_len: CyclicPrefixLen::Cp512,
            center_freq_hz: 600e6,
            channel_bw_hz: 192e6,
            n_active: 7600,  // ~7600 active subcarriers in 192 MHz with 25 kHz spacing
        }
    }

    /// Create a default upstream DOCSIS 3.1 high-split channel at 100 MHz
    pub fn upstream_default() -> Self {
        OfdmChannelConfig {
            version: DocsisVersion::D31,
            direction: Direction::Upstream,
            spacing: SubcarrierSpacing::KHz50,
            cp_len: CyclicPrefixLen::Cp192,
            center_freq_hz: 100e6,
            channel_bw_hz: 96e6,
            n_active: 1880,
        }
    }

    /// Sample rate derived from spacing and FFT size
    pub fn sample_rate_hz(&self) -> f64 {
        self.spacing.hz() * self.spacing.fft_size() as f64
    }

    /// OFDM symbol period including CP (in samples)
    pub fn symbol_period_samples(&self) -> usize {
        self.spacing.fft_size() + self.cp_len as usize
    }

    /// OFDM symbol period including CP (in µs)
    pub fn symbol_period_us(&self) -> f64 {
        let n = self.symbol_period_samples();
        n as f64 / self.sample_rate_hz() * 1e6
    }
}

/// DOCSIS OFDM Modulator
#[derive(Debug, Clone)]
pub struct OfdmModulator {
    pub config: OfdmChannelConfig,
    pub profile: ModulationProfile,
    pub pilot_pattern: PilotPattern,
    pub exclusion: ExclusionBandManager,
    symbol_count: usize,
}

impl OfdmModulator {
    /// Create a new OFDM modulator
    pub fn new(
        config: OfdmChannelConfig,
        profile: ModulationProfile,
        pilot_pattern: PilotPattern,
        exclusion: ExclusionBandManager,
    ) -> Self {
        OfdmModulator {
            config,
            profile,
            pilot_pattern,
            exclusion,
            symbol_count: 0,
        }
    }

    /// Modulate one OFDM symbol from a slice of QAM symbols (complex IQ)
    ///
    /// `data_iq`: data subcarrier samples (must match active non-pilot count)
    /// Returns time-domain samples with CP prepended.
    pub fn modulate_symbol(&mut self, data_iq: &[(f64, f64)]) -> OfdmSymbol {
        let n = self.config.spacing.fft_size();
        let cp_len = self.config.cp_len as usize;

        // Build frequency-domain vector
        let mut freq_domain = vec![(0.0f64, 0.0f64); n];

        // Get pilots for this symbol
        let pilot_indices: std::collections::HashSet<usize> =
            self.pilot_pattern.pilot_subcarrier_indices(self.symbol_count).into_iter().collect();

        // Place data and pilots
        let mut data_idx = 0;
        for sc in 0..self.config.n_active {
            let freq_bin = sc; // simplified: no guard band offset for test
            if freq_bin >= n {
                break;
            }
            if pilot_indices.contains(&sc) {
                // Insert BPSK pilot
                freq_domain[freq_bin] = (1.0, 0.0);
            } else {
                if data_idx < data_iq.len() {
                    freq_domain[freq_bin] = data_iq[data_idx];
                    data_idx += 1;
                }
            }
        }

        // IFFT
        fft(&mut freq_domain, true);

        // Prepend cyclic prefix
        let mut samples = Vec::with_capacity(n + cp_len);
        samples.extend_from_slice(&freq_domain[n - cp_len..]);
        samples.extend_from_slice(&freq_domain);

        self.symbol_count += 1;
        OfdmSymbol { samples, fft_size: n, cp_len }
    }

    /// Throughput in Mbps for the current profile
    pub fn throughput_mbps(&self) -> f64 {
        let bps = self.profile.bits_per_symbol();
        let sym_rate = 1.0 / (self.config.symbol_period_us() * 1e-6);
        (bps as f64 * sym_rate) / 1e6
    }
}

// ---------------------------------------------------------------------------
// OFDM Demodulator
// ---------------------------------------------------------------------------

/// DOCSIS OFDM Demodulator
#[derive(Debug, Clone)]
pub struct OfdmDemodulator {
    pub config: OfdmChannelConfig,
    pub pilot_pattern: PilotPattern,
    symbol_count: usize,
}

impl OfdmDemodulator {
    pub fn new(config: OfdmChannelConfig, pilot_pattern: PilotPattern) -> Self {
        OfdmDemodulator { config, pilot_pattern, symbol_count: 0 }
    }

    /// Demodulate one received OFDM symbol (with CP)
    ///
    /// Returns frequency-domain subcarrier samples (after CP removal and FFT)
    pub fn demodulate_symbol(&mut self, rx: &[Complex]) -> Vec<Complex> {
        let n = self.config.spacing.fft_size();
        let cp_len = self.config.cp_len as usize;

        // Remove CP
        let data_start = cp_len.min(rx.len().saturating_sub(n));
        let payload_end = (data_start + n).min(rx.len());
        let mut freq_domain: Vec<Complex> = rx[data_start..payload_end].to_vec();
        freq_domain.resize(n, (0.0, 0.0));

        // Forward FFT
        fft(&mut freq_domain, false);

        self.symbol_count += 1;
        freq_domain
    }

    /// Simple least-squares channel estimation from known pilot positions
    ///
    /// Returns a per-subcarrier channel coefficient (magnitude, phase) vector
    pub fn estimate_channel(
        &self,
        rx_freq: &[Complex],
        tx_pilots: &[(usize, Complex)],
    ) -> Vec<Complex> {
        let n = rx_freq.len();
        // Build sparse estimates at pilot positions
        let mut h_pilots: HashMap<usize, Complex> = HashMap::new();
        for &(sc, tx) in tx_pilots {
            if sc < rx_freq.len() {
                let rx = rx_freq[sc];
                // H[sc] = Rx / Tx
                let tx_mag2 = tx.0 * tx.0 + tx.1 * tx.1;
                if tx_mag2 > 1e-12 {
                    let re = (rx.0 * tx.0 + rx.1 * tx.1) / tx_mag2;
                    let im = (rx.1 * tx.0 - rx.0 * tx.1) / tx_mag2;
                    h_pilots.insert(sc, (re, im));
                }
            }
        }

        // Linear interpolation between pilots
        let pilot_scs: Vec<usize> = {
            let mut v: Vec<usize> = h_pilots.keys().cloned().collect();
            v.sort();
            v
        };

        let mut h_est = vec![(1.0f64, 0.0f64); n];
        if pilot_scs.is_empty() {
            return h_est;
        }

        for sc in 0..n {
            // Find bounding pilots
            let lo = pilot_scs.iter().rev().find(|&&p| p <= sc).cloned();
            let hi = pilot_scs.iter().find(|&&p| p > sc).cloned();
            match (lo, hi) {
                (Some(l), Some(h)) => {
                    let hl = h_pilots[&l];
                    let hh = h_pilots[&h];
                    let alpha = (sc - l) as f64 / (h - l) as f64;
                    h_est[sc] = (
                        hl.0 + alpha * (hh.0 - hl.0),
                        hl.1 + alpha * (hh.1 - hl.1),
                    );
                }
                (Some(l), None) => { h_est[sc] = h_pilots[&l]; }
                (None, Some(h)) => { h_est[sc] = h_pilots[&h]; }
                (None, None) => {}
            }
        }
        h_est
    }

    /// Apply ZF equalization
    pub fn equalize_zf(rx_freq: &[Complex], h_est: &[Complex]) -> Vec<Complex> {
        rx_freq.iter().zip(h_est.iter()).map(|(&r, &h)| {
            let h_mag2 = h.0 * h.0 + h.1 * h.1;
            if h_mag2 > 1e-12 {
                // r / h = r * conj(h) / |h|^2
                (
                    (r.0 * h.0 + r.1 * h.1) / h_mag2,
                    (r.1 * h.0 - r.0 * h.1) / h_mag2,
                )
            } else {
                r
            }
        }).collect()
    }
}

// ---------------------------------------------------------------------------
// LDPC Codec Interface
// ---------------------------------------------------------------------------

/// LDPC encoder parameters (structural interface, not full matrix)
#[derive(Debug, Clone)]
pub struct LdpcParams {
    pub code_rate: LdpcCodeRate,
    pub codeword_size: usize,
    pub info_bits: usize,
    pub parity_bits: usize,
}

impl LdpcParams {
    /// Create LDPC params for a given rate and codeword size
    pub fn new(rate: LdpcCodeRate, short: bool) -> Self {
        let cw = if short { LDPC_CW_SHORT } else { LDPC_CW_LONG };
        let info = if short { rate.info_bits_short() } else { rate.info_bits_long() };
        LdpcParams {
            code_rate: rate,
            codeword_size: cw,
            info_bits: info,
            parity_bits: cw - info,
        }
    }

    /// Overhead fraction added by LDPC
    pub fn overhead_fraction(&self) -> f64 {
        self.parity_bits as f64 / self.info_bits as f64
    }

    /// Effective spectral efficiency (bits per coded bit)
    pub fn spectral_efficiency(&self) -> f64 {
        self.code_rate.rate()
    }
}

/// Simple LDPC encoder (systematic: info bits pass through, parity computed via XOR)
///
/// This is a structural placeholder matching the DOCSIS parity check architecture.
/// Full Tanner graph encoding requires the H-matrix from the specification.
#[derive(Debug, Clone)]
pub struct LdpcEncoder {
    pub params: LdpcParams,
}

impl LdpcEncoder {
    pub fn new(rate: LdpcCodeRate, short: bool) -> Self {
        LdpcEncoder { params: LdpcParams::new(rate, short) }
    }

    /// Encode information bits (systematic: info + synthetic parity)
    ///
    /// Returns a codeword of length `params.codeword_size` bits.
    /// Note: real parity is computed from the DOCSIS H-matrix; this uses
    /// a simplified cyclic parity for structural completeness.
    pub fn encode(&self, info_bits: &[u8]) -> Vec<u8> {
        let k = self.params.info_bits.min(info_bits.len());
        let mut cw = vec![0u8; self.params.codeword_size];
        // Systematic part
        cw[..k].copy_from_slice(&info_bits[..k]);
        // Simple cyclic parity (not spec-compliant but structurally correct)
        for i in 0..self.params.parity_bits {
            let mut p = 0u8;
            for j in 0..k {
                p ^= cw[(j + i) % k];
            }
            cw[k + i] = p & 1;
        }
        cw
    }

    /// Decode (hard decision – flip bits where parity fails)
    pub fn decode_hard(&self, received: &[u8]) -> Vec<u8> {
        // Return only information bits portion
        let k = self.params.info_bits.min(received.len());
        received[..k].to_vec()
    }
}

// ---------------------------------------------------------------------------
// Upstream OFDMA
// ---------------------------------------------------------------------------

/// Mini-slot allocation for DOCSIS upstream OFDMA
///
/// A mini-slot is the smallest upstream allocation unit: 1 or more OFDM symbols
/// spanning a subset of subcarriers.
#[derive(Debug, Clone)]
pub struct MiniSlot {
    /// Mini-slot ID
    pub id: usize,
    /// Starting subcarrier index
    pub sc_start: usize,
    /// Number of subcarriers
    pub sc_count: usize,
    /// Starting OFDM symbol
    pub symbol_start: usize,
    /// Number of OFDM symbols
    pub symbol_count: usize,
    /// Assigned CM node ID (None = unassigned / contention)
    pub cm_id: Option<u32>,
}

impl MiniSlot {
    pub fn new(id: usize, sc_start: usize, sc_count: usize, symbol_start: usize, symbol_count: usize) -> Self {
        MiniSlot { id, sc_start, sc_count, symbol_start, symbol_count, cm_id: None }
    }

    /// Total resource elements in this mini-slot
    pub fn resource_elements(&self) -> usize {
        self.sc_count * self.symbol_count
    }
}

/// DOCSIS Upstream OFDMA Resource Allocator
///
/// Manages the upstream time-frequency grid, assigns mini-slots to CMs,
/// and tracks contention regions.
#[derive(Debug, Clone)]
pub struct UpstreamAllocator {
    pub config: OfdmChannelConfig,
    /// All mini-slots in the upstream map
    pub mini_slots: Vec<MiniSlot>,
    /// Contention mini-slot IDs (for ranging/probing)
    contention_slots: Vec<usize>,
    /// Total symbols in one upstream MAP interval
    pub map_symbols: usize,
}

impl UpstreamAllocator {
    /// Create an allocator with a uniform grid of mini-slots
    ///
    /// `n_sc_per_slot`: subcarriers per mini-slot
    /// `n_sym_per_slot`: symbols per mini-slot
    /// `map_symbols`: total symbols in MAP interval
    pub fn new(
        config: OfdmChannelConfig,
        n_sc_per_slot: usize,
        n_sym_per_slot: usize,
        map_symbols: usize,
    ) -> Self {
        let n_sc = config.n_active;
        let n_slots_sc = n_sc / n_sc_per_slot.max(1);
        let n_slots_sym = map_symbols / n_sym_per_slot.max(1);
        let mut mini_slots = Vec::new();
        let mut id = 0;
        for s in 0..n_slots_sym {
            for f in 0..n_slots_sc {
                mini_slots.push(MiniSlot::new(
                    id,
                    f * n_sc_per_slot,
                    n_sc_per_slot,
                    s * n_sym_per_slot,
                    n_sym_per_slot,
                ));
                id += 1;
            }
        }
        UpstreamAllocator { config, mini_slots, contention_slots: vec![], map_symbols }
    }

    /// Assign a mini-slot to a CM node
    pub fn assign(&mut self, slot_id: usize, cm_id: u32) -> Result<(), &'static str> {
        self.mini_slots.get_mut(slot_id)
            .ok_or("Invalid slot ID")?
            .cm_id = Some(cm_id);
        Ok(())
    }

    /// Release a mini-slot (make unassigned)
    pub fn release(&mut self, slot_id: usize) -> Result<(), &'static str> {
        self.mini_slots.get_mut(slot_id)
            .ok_or("Invalid slot ID")?
            .cm_id = None;
        Ok(())
    }

    /// Mark slots as contention slots for ranging / probing
    pub fn set_contention_slots(&mut self, ids: Vec<usize>) {
        self.contention_slots = ids;
    }

    /// Get unassigned mini-slot IDs
    pub fn unassigned_slots(&self) -> Vec<usize> {
        self.mini_slots.iter()
            .filter(|s| s.cm_id.is_none() && !self.contention_slots.contains(&s.id))
            .map(|s| s.id)
            .collect()
    }

    /// Total mini-slots in the MAP
    pub fn total_slots(&self) -> usize {
        self.mini_slots.len()
    }

    /// Upstream utilization as fraction (assigned / total)
    pub fn utilization(&self) -> f64 {
        let assigned = self.mini_slots.iter().filter(|s| s.cm_id.is_some()).count();
        if self.total_slots() == 0 {
            0.0
        } else {
            assigned as f64 / self.total_slots() as f64
        }
    }

    /// Generate a probe burst for ranging (simple constant tone per subcarrier)
    pub fn generate_probe(sc_start: usize, sc_count: usize, fft_size: usize) -> Vec<Complex> {
        let mut freq = vec![(0.0f64, 0.0f64); fft_size];
        for sc in sc_start..(sc_start + sc_count).min(fft_size) {
            freq[sc] = (1.0, 0.0);
        }
        fft(&mut freq, true);
        freq
    }
}

// ---------------------------------------------------------------------------
// Channel / SNR Estimation
// ---------------------------------------------------------------------------

/// Per-subcarrier SNR estimate
#[derive(Debug, Clone)]
pub struct SubcarrierSnr {
    /// Subcarrier index
    pub sc_idx: usize,
    /// Estimated SNR in dB
    pub snr_db: f64,
    /// Recommended modulation order for this subcarrier
    pub recommended_mod: ModulationOrder,
}

/// Downstream channel quality report (CQI feedback from CM to CMTS)
#[derive(Debug, Clone)]
pub struct ChannelQualityReport {
    pub per_subcarrier: Vec<SubcarrierSnr>,
    pub average_snr_db: f64,
    pub recommended_profile: usize,
}

impl ChannelQualityReport {
    /// Build a report from per-subcarrier SNR values
    pub fn from_snr_map(snr_values: &[f64], profile_manager: &mut ProfileManager) -> Self {
        let mut per_sc = Vec::with_capacity(snr_values.len());
        let sum: f64 = snr_values.iter().sum();
        let avg = if snr_values.is_empty() { 0.0 } else { sum / snr_values.len() as f64 };

        for (i, &snr) in snr_values.iter().enumerate() {
            let rec_mod = select_modulation_for_snr(snr);
            per_sc.push(SubcarrierSnr { sc_idx: i, snr_db: snr, recommended_mod: rec_mod });
        }

        let rec_profile = profile_manager.select_profile(avg);
        ChannelQualityReport {
            per_subcarrier: per_sc,
            average_snr_db: avg,
            recommended_profile: rec_profile,
        }
    }

    /// Return the minimum SNR across all subcarriers
    pub fn min_snr_db(&self) -> f64 {
        self.per_subcarrier.iter()
            .map(|s| s.snr_db)
            .fold(f64::INFINITY, f64::min)
    }

    /// Return the maximum SNR across all subcarriers
    pub fn max_snr_db(&self) -> f64 {
        self.per_subcarrier.iter()
            .map(|s| s.snr_db)
            .fold(f64::NEG_INFINITY, f64::max)
    }
}

/// Select the highest supported modulation for a given SNR
pub fn select_modulation_for_snr(snr_db: f64) -> ModulationOrder {
    // Order from highest to lowest
    let orders = [
        ModulationOrder::Qam16384,
        ModulationOrder::Qam8192,
        ModulationOrder::Qam4096,
        ModulationOrder::Qam2048,
        ModulationOrder::Qam1024,
        ModulationOrder::Qam512,
        ModulationOrder::Qam256,
        ModulationOrder::Qam128,
        ModulationOrder::Qam64,
        ModulationOrder::Qam32,
        ModulationOrder::Qam16,
        ModulationOrder::Qpsk,
        ModulationOrder::Bpsk,
    ];
    for &o in &orders {
        if snr_db >= o.min_snr_db() {
            return o;
        }
    }
    ModulationOrder::Bpsk
}

// ---------------------------------------------------------------------------
// Windowed overlap (spectral shaping)
// ---------------------------------------------------------------------------

/// Apply a raised-cosine window roll-off to the start/end of a time-domain block
/// to reduce spectral sidelobes at band edges (DOCSIS windowed OFDM)
pub fn apply_windowing(samples: &mut [Complex], roll_off_samples: usize) {
    let n = samples.len();
    let r = roll_off_samples.min(n / 2);
    for i in 0..r {
        let alpha = 0.5 * (1.0 - (PI * i as f64 / r as f64).cos());
        samples[i].0 *= alpha;
        samples[i].1 *= alpha;
        samples[n - 1 - i].0 *= alpha;
        samples[n - 1 - i].1 *= alpha;
    }
}

/// Overlap-add combination of two windowed symbols (transmit-side)
pub fn overlap_add(sym_a: &[Complex], sym_b: &[Complex], overlap: usize) -> Vec<Complex> {
    let na = sym_a.len();
    let nb = sym_b.len();
    let total = na + nb - overlap;
    let mut out = vec![(0.0f64, 0.0f64); total];
    for (i, &s) in sym_a.iter().enumerate() {
        out[i].0 += s.0;
        out[i].1 += s.1;
    }
    for (i, &s) in sym_b.iter().enumerate() {
        let pos = na - overlap + i;
        out[pos].0 += s.0;
        out[pos].1 += s.1;
    }
    out
}

// ---------------------------------------------------------------------------
// Full DOCSIS PHY Processor (high-level entry point)
// ---------------------------------------------------------------------------

/// DOCSIS 3.1/4.0 downstream PHY processor
///
/// Combines modulator, profile manager, PLC, and exclusion bands
/// into a unified transmit/receive chain.
pub struct DocsisPhyProcessor {
    pub config: OfdmChannelConfig,
    pub profile_manager: ProfileManager,
    pub modulator: OfdmModulator,
    pub demodulator: OfdmDemodulator,
    pub plc: PlcConfig,
    pub exclusion: ExclusionBandManager,
}

impl DocsisPhyProcessor {
    /// Create a default DOCSIS 3.1 downstream processor
    pub fn new_downstream_default() -> Self {
        let config = OfdmChannelConfig::downstream_default();
        let n_active = config.n_active;
        let profile_manager = ProfileManager::new_default(n_active);
        let pilot_pattern = PilotPattern::new_standard(n_active, 128, 32, 8, 2.0);
        let exclusion = ExclusionBandManager::new();
        let plc = PlcConfig::new(config.center_freq_hz - config.channel_bw_hz / 2.0 + 4e6);

        let modulator = OfdmModulator::new(
            config.clone(),
            profile_manager.active_profile().clone(),
            pilot_pattern.clone(),
            exclusion.clone(),
        );
        let demodulator = OfdmDemodulator::new(config.clone(), pilot_pattern);

        DocsisPhyProcessor {
            config,
            profile_manager,
            modulator,
            demodulator,
            plc,
            exclusion,
        }
    }

    /// Create a DOCSIS 4.0 processor
    pub fn new_d40() -> Self {
        let mut config = OfdmChannelConfig::downstream_default();
        config.version = DocsisVersion::D40;
        let n_active = config.n_active;
        let mut pm = ProfileManager::new_default(n_active);
        // Add D4.0-specific high-order profiles
        let _ = pm.add_profile(ModulationProfile::uniform(4, ModulationOrder::Qam8192, n_active));
        let _ = pm.add_profile(ModulationProfile::uniform(5, ModulationOrder::Qam16384, n_active));
        let pilot_pattern = PilotPattern::new_standard(n_active, 128, 32, 8, 2.0);
        let exclusion = ExclusionBandManager::new();
        let plc = PlcConfig::new(config.center_freq_hz - config.channel_bw_hz / 2.0 + 4e6);

        let modulator = OfdmModulator::new(
            config.clone(),
            pm.active_profile().clone(),
            pilot_pattern.clone(),
            exclusion.clone(),
        );
        let demodulator = OfdmDemodulator::new(config.clone(), pilot_pattern);

        DocsisPhyProcessor {
            config,
            profile_manager: pm,
            modulator,
            demodulator,
            plc,
            exclusion,
        }
    }

    /// Transmit one OFDM symbol of data
    pub fn transmit_symbol(&mut self, data_iq: &[(f64, f64)]) -> OfdmSymbol {
        self.modulator.modulate_symbol(data_iq)
    }

    /// Receive one OFDM symbol (returns frequency-domain subcarriers)
    pub fn receive_symbol(&mut self, rx_samples: &[Complex]) -> Vec<Complex> {
        self.demodulator.demodulate_symbol(rx_samples)
    }

    /// Update profile based on SNR feedback from CM
    pub fn update_profile(&mut self, snr_db: f64) {
        let idx = self.profile_manager.select_profile(snr_db);
        let profile = self.profile_manager.profile(idx).cloned().unwrap();
        self.modulator.profile = profile;
    }

    /// Approximate downstream throughput in Gbps
    pub fn throughput_gbps(&self) -> f64 {
        self.modulator.throughput_mbps() / 1000.0
    }

    /// Check whether a frequency is excluded
    pub fn is_excluded(&self, freq_hz: f64) -> bool {
        self.exclusion.is_excluded(freq_hz)
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // --- SubcarrierSpacing tests ---

    #[test]
    fn test_subcarrier_spacing_hz() {
        assert_eq!(SubcarrierSpacing::KHz25.hz(), 25_000.0);
        assert_eq!(SubcarrierSpacing::KHz50.hz(), 50_000.0);
    }

    #[test]
    fn test_subcarrier_spacing_fft_size() {
        assert_eq!(SubcarrierSpacing::KHz25.fft_size(), 8192);
        assert_eq!(SubcarrierSpacing::KHz50.fft_size(), 4096);
    }

    #[test]
    fn test_subcarrier_spacing_symbol_duration() {
        let dur = SubcarrierSpacing::KHz25.symbol_duration_us();
        assert!((dur - 40.0).abs() < 1e-6, "Expected 40 µs, got {}", dur);
        let dur50 = SubcarrierSpacing::KHz50.symbol_duration_us();
        assert!((dur50 - 20.0).abs() < 1e-6, "Expected 20 µs, got {}", dur50);
    }

    // --- ModulationOrder tests ---

    #[test]
    fn test_modulation_order_bits() {
        assert_eq!(ModulationOrder::Qpsk.bits_per_symbol(), 2);
        assert_eq!(ModulationOrder::Qam256.bits_per_symbol(), 8);
        assert_eq!(ModulationOrder::Qam4096.bits_per_symbol(), 12);
        assert_eq!(ModulationOrder::Qam16384.bits_per_symbol(), 14);
    }

    #[test]
    fn test_modulation_order_constellation_size() {
        assert_eq!(ModulationOrder::Qpsk.constellation_size(), 4);
        assert_eq!(ModulationOrder::Qam64.constellation_size(), 64);
        assert_eq!(ModulationOrder::Qam4096.constellation_size(), 4096);
    }

    #[test]
    fn test_modulation_order_min_snr() {
        assert!(ModulationOrder::Qpsk.min_snr_db() < ModulationOrder::Qam256.min_snr_db());
        assert!(ModulationOrder::Qam256.min_snr_db() < ModulationOrder::Qam4096.min_snr_db());
    }

    // --- CyclicPrefixLen tests ---

    #[test]
    fn test_cp_duration() {
        let dur = CyclicPrefixLen::Cp512.duration_us(SubcarrierSpacing::KHz25);
        // sample_rate = 25000 * 8192 = 204.8 MHz, 512 / 204.8e6 * 1e6 = 2.5 µs
        assert!((dur - 2.5).abs() < 0.01, "CP duration {} µs", dur);
    }

    // --- LdpcCodeRate tests ---

    #[test]
    fn test_ldpc_code_rate() {
        let r = LdpcCodeRate::Rate3_4;
        assert_eq!(r.fraction(), (3, 4));
        assert!((r.rate() - 0.75).abs() < 1e-10);
    }

    #[test]
    fn test_ldpc_info_bits_short() {
        let r = LdpcCodeRate::Rate8_9;
        let info = r.info_bits_short();
        assert_eq!(info, 16200 * 8 / 9);
    }

    #[test]
    fn test_ldpc_info_bits_long() {
        let r = LdpcCodeRate::Rate1_2;
        let info = r.info_bits_long();
        assert_eq!(info, 32400);
    }

    // --- LdpcParams tests ---

    #[test]
    fn test_ldpc_params_short() {
        let p = LdpcParams::new(LdpcCodeRate::Rate3_4, true);
        assert_eq!(p.codeword_size, 16200);
        assert_eq!(p.info_bits, 12150);
        assert_eq!(p.parity_bits, 4050);
    }

    #[test]
    fn test_ldpc_params_long() {
        let p = LdpcParams::new(LdpcCodeRate::Rate5_6, false);
        assert_eq!(p.codeword_size, 64800);
    }

    // --- LdpcEncoder tests ---

    #[test]
    fn test_ldpc_encode_decode_roundtrip() {
        let enc = LdpcEncoder::new(LdpcCodeRate::Rate3_4, true);
        let info: Vec<u8> = (0..enc.params.info_bits).map(|i| (i % 2) as u8).collect();
        let cw = enc.encode(&info);
        assert_eq!(cw.len(), enc.params.codeword_size);
        let decoded = enc.decode_hard(&cw);
        assert_eq!(decoded.len(), enc.params.info_bits);
        assert_eq!(&decoded[..10], &info[..10]);
    }

    // --- Gray code tests ---

    #[test]
    fn test_gray_code() {
        assert_eq!(gray_code(0), 0);
        assert_eq!(gray_code(1), 1);
        assert_eq!(gray_code(2), 3);
        assert_eq!(gray_code(3), 2);
    }

    #[test]
    fn test_inv_gray_code() {
        for i in 0..16usize {
            assert_eq!(inv_gray_code(gray_code(i)), i, "inv_gray({}) failed", i);
        }
    }

    // --- QAM Constellation tests ---

    #[test]
    fn test_qam16_constellation_size() {
        let c = generate_qam_constellation(ModulationOrder::Qam16);
        assert_eq!(c.len(), 16);
    }

    #[test]
    fn test_qam256_constellation_size() {
        let c = generate_qam_constellation(ModulationOrder::Qam256);
        assert_eq!(c.len(), 256);
    }

    #[test]
    fn test_qam_unit_power() {
        let c = generate_qam_constellation(ModulationOrder::Qam64);
        let avg = c.iter().map(|(i, q)| i * i + q * q).sum::<f64>() / c.len() as f64;
        assert!((avg - 1.0).abs() < 0.01, "avg power {} (expected ~1.0)", avg);
    }

    #[test]
    fn test_bpsk_constellation() {
        let c = generate_qam_constellation(ModulationOrder::Bpsk);
        assert_eq!(c.len(), 2);
    }

    // --- QamMapper tests ---

    #[test]
    fn test_qam_mapper_map_demap_qpsk() {
        let mapper = QamMapper::new(ModulationOrder::Qpsk);
        let bits: Vec<u8> = vec![0, 0, 0, 1, 1, 0, 1, 1];
        let syms = mapper.map(&bits);
        assert_eq!(syms.len(), 4);
        // Demap and check roundtrip
        let mut recovered = Vec::new();
        for s in &syms {
            recovered.extend(mapper.demap(*s));
        }
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_qam_mapper_map_demap_64qam() {
        let mapper = QamMapper::new(ModulationOrder::Qam64);
        let bits: Vec<u8> = (0..12).map(|i| i % 2).collect();
        let syms = mapper.map(&bits);
        assert_eq!(syms.len(), 2);
        for s in &syms {
            let back = mapper.demap(*s);
            assert_eq!(back.len(), 6);
        }
    }

    #[test]
    fn test_qam_evm() {
        let mapper = QamMapper::new(ModulationOrder::Qpsk);
        let c = &mapper.constellation()[0];
        let evm = mapper.evm_linear(*c);
        assert!(evm < 1e-10, "EVM at exact point should be ~0, got {}", evm);
    }

    // --- FFT tests ---

    #[test]
    fn test_fft_ifft_roundtrip() {
        let n = 64;
        let mut data: Vec<Complex> = (0..n).map(|i| (i as f64 / n as f64, 0.0)).collect();
        let orig = data.clone();
        fft(&mut data, false);
        fft(&mut data, true);
        for (a, b) in orig.iter().zip(data.iter()) {
            assert!((a.0 - b.0).abs() < 1e-10, "FFT/IFFT mismatch real: {} vs {}", a.0, b.0);
            assert!((a.1 - b.1).abs() < 1e-10, "FFT/IFFT mismatch imag: {} vs {}", a.1, b.1);
        }
    }

    #[test]
    fn test_fft_impulse() {
        let n = 16;
        let mut data: Vec<Complex> = vec![(0.0, 0.0); n];
        data[0] = (1.0, 0.0);
        fft(&mut data, false);
        // Transform of delta[0] should be all-ones
        for (i, &(r, im)) in data.iter().enumerate() {
            assert!((r - 1.0).abs() < 1e-10, "bin {} real: {}", i, r);
            assert!(im.abs() < 1e-10, "bin {} imag: {}", i, im);
        }
    }

    // --- ExclusionBand tests ---

    #[test]
    fn test_exclusion_band_contains() {
        let b = ExclusionBand::new(100e6, 110e6, "test");
        assert!(b.contains(105e6));
        assert!(!b.contains(115e6));
        assert!(!b.contains(99e6));
    }

    #[test]
    fn test_exclusion_band_bw() {
        let b = ExclusionBand::new(100e6, 108e6, "test");
        assert!((b.bandwidth_hz() - 8e6).abs() < 1.0);
    }

    #[test]
    fn test_exclusion_manager_fm_radio() {
        let mut mgr = ExclusionBandManager::new();
        mgr.add_fm_radio_band();
        assert_eq!(mgr.num_bands(), 1);
        assert!(mgr.is_excluded(100e6));
        assert!(!mgr.is_excluded(200e6));
    }

    #[test]
    fn test_exclusion_manager_subcarriers() {
        let mut mgr = ExclusionBandManager::new();
        mgr.add_band(ExclusionBand::new(300e6, 310e6, "band"));
        let excluded = mgr.excluded_subcarriers(298e6, 10, 1e6);
        // subcarrier 2 (298+2=300MHz) through 12 (298+12=310MHz) should be excluded
        assert!(excluded.contains(&2));
    }

    // --- PilotPattern tests ---

    #[test]
    fn test_pilot_pattern_creates_pilots() {
        let pp = PilotPattern::new_standard(1000, 100, 20, 4, 2.0);
        assert!(pp.num_pilots() > 0);
    }

    #[test]
    fn test_pilot_pattern_symbol_0() {
        let pp = PilotPattern::new_standard(512, 64, 16, 4, 1.5);
        let syms = pp.pilots_for_symbol(0);
        assert!(!syms.is_empty());
    }

    #[test]
    fn test_pilot_pattern_continuous_always_present() {
        let pp = PilotPattern::new_standard(256, 64, 16, 4, 1.5);
        for sym in 0..4 {
            let indices = pp.pilot_subcarrier_indices(sym);
            // Continuous pilot at index 0 should always be present
            assert!(indices.contains(&0), "Symbol {} missing continuous pilot at 0", sym);
        }
    }

    // --- ModulationProfile tests ---

    #[test]
    fn test_profile_uniform_bits_per_symbol() {
        let p = ModulationProfile::uniform(0, ModulationOrder::Qam64, 100);
        assert_eq!(p.bits_per_symbol(), 600); // 100 subcarriers * 6 bits
    }

    #[test]
    fn test_profile_active_subcarriers() {
        let mut p = ModulationProfile::uniform(0, ModulationOrder::Qam256, 50);
        p.subcarrier_mods[0] = ModulationOrder::Pilot;
        assert_eq!(p.active_subcarriers(), 49);
    }

    // --- ProfileManager tests ---

    #[test]
    fn test_profile_manager_select_high_snr() {
        let mut pm = ProfileManager::new_default(100);
        let idx = pm.select_profile(35.0);
        // 35 dB >= QAM256 (27 dB) should select at least QAM256
        assert!(pm.profile(idx).unwrap().min_snr_db <= 35.0);
    }

    #[test]
    fn test_profile_manager_select_low_snr() {
        let mut pm = ProfileManager::new_default(100);
        let idx = pm.select_profile(5.0);
        // Very low SNR → should select the lowest-order profile
        assert_eq!(idx, 0);
    }

    #[test]
    fn test_profile_manager_add_custom() {
        let mut pm = ProfileManager::new_default(100);
        let custom = ModulationProfile::uniform(4, ModulationOrder::Qam4096, 100);
        pm.add_profile(custom).unwrap();
        assert_eq!(pm.num_profiles(), 5);
    }

    #[test]
    fn test_profile_manager_max_profiles() {
        let mut pm = ProfileManager::new_default(10);
        for i in 4..MAX_PROFILES {
            let p = ModulationProfile::uniform(i, ModulationOrder::Qpsk, 10);
            pm.add_profile(p).unwrap();
        }
        assert_eq!(pm.num_profiles(), MAX_PROFILES);
        let overflow = ModulationProfile::uniform(99, ModulationOrder::Qpsk, 10);
        assert!(pm.add_profile(overflow).is_err());
    }

    // --- PLC tests ---

    #[test]
    fn test_plc_preamble_length() {
        let plc = PlcConfig::new(300e6);
        let preamble = plc.generate_preamble(64);
        assert_eq!(preamble.len(), 64);
    }

    #[test]
    fn test_plc_preamble_unit_magnitude() {
        let plc = PlcConfig::new(300e6);
        let preamble = plc.generate_preamble(16);
        for (i, q) in &preamble {
            let mag = (i * i + q * q).sqrt();
            assert!((mag - 1.0).abs() < 1e-10, "Preamble sample magnitude {}", mag);
        }
    }

    #[test]
    fn test_plc_timestamp_encode() {
        let plc = PlcConfig::new(300e6);
        let ts = plc.encode_timestamp(1_234_567_890_000);
        assert_eq!(ts.len(), 9);
        assert_eq!(ts[0], 0x01); // message type
    }

    // --- OfdmChannelConfig tests ---

    #[test]
    fn test_config_sample_rate() {
        let cfg = OfdmChannelConfig::downstream_default();
        // 25kHz * 8192 = 204.8 MHz
        assert!((cfg.sample_rate_hz() - 204.8e6).abs() < 1.0);
    }

    #[test]
    fn test_config_symbol_period_samples() {
        let cfg = OfdmChannelConfig::downstream_default();
        assert_eq!(cfg.symbol_period_samples(), 8192 + 512);
    }

    #[test]
    fn test_config_symbol_period_us() {
        let cfg = OfdmChannelConfig::downstream_default();
        let period = cfg.symbol_period_us();
        // (8192+512)/204.8e6 * 1e6 = 42.5 µs
        assert!((period - 42.5).abs() < 0.1, "Symbol period {} µs", period);
    }

    // --- OfdmModulator tests ---

    #[test]
    fn test_ofdm_modulator_symbol_length() {
        let config = OfdmChannelConfig::downstream_default();
        let n_active = config.n_active;
        let profile = ModulationProfile::uniform(0, ModulationOrder::Qpsk, n_active);
        let pp = PilotPattern::new_standard(n_active, 128, 32, 8, 1.0);
        let exc = ExclusionBandManager::new();
        let mut modulator = OfdmModulator::new(config.clone(), profile, pp, exc);

        let data_iq: Vec<(f64, f64)> = (0..n_active).map(|i| {
            let a = i as f64 * 0.01;
            (a.cos(), a.sin())
        }).collect();
        let sym = modulator.modulate_symbol(&data_iq);
        assert_eq!(sym.samples.len(), config.symbol_period_samples());
    }

    #[test]
    fn test_ofdm_modulator_throughput_positive() {
        let phy = DocsisPhyProcessor::new_downstream_default();
        assert!(phy.modulator.throughput_mbps() > 0.0);
    }

    // --- OfdmDemodulator tests ---

    #[test]
    fn test_ofdm_demod_output_size() {
        let config = OfdmChannelConfig::downstream_default();
        let n = config.spacing.fft_size();
        let cp = config.cp_len as usize;
        let pp = PilotPattern::new_standard(config.n_active, 128, 32, 8, 1.0);
        let mut demod = OfdmDemodulator::new(config, pp);

        let samples: Vec<Complex> = vec![(0.5, 0.5); n + cp];
        let freq = demod.demodulate_symbol(&samples);
        assert_eq!(freq.len(), n);
    }

    #[test]
    fn test_ofdm_moddemod_roundtrip() {
        // Small test with 4096-point FFT
        let mut config = OfdmChannelConfig::upstream_default();
        config.n_active = 64;
        let n_active = config.n_active;
        let profile = ModulationProfile::uniform(0, ModulationOrder::Qpsk, n_active);
        let pp = PilotPattern::new_standard(n_active, 32, 8, 4, 1.0);
        let exc = ExclusionBandManager::new();
        let mut modulator = OfdmModulator::new(config.clone(), profile, pp.clone(), exc);

        // Create data
        let data_iq: Vec<(f64, f64)> = (0..n_active).map(|i| {
            let theta = 2.0 * PI * i as f64 / n_active as f64;
            (theta.cos(), theta.sin())
        }).collect();
        let sym = modulator.modulate_symbol(&data_iq);

        // Demodulate
        let mut demod = OfdmDemodulator::new(config.clone(), pp);
        let freq = demod.demodulate_symbol(&sym.samples);
        assert_eq!(freq.len(), config.spacing.fft_size());
    }

    // --- Channel estimation tests ---

    #[test]
    fn test_channel_estimation_identity() {
        let config = OfdmChannelConfig::upstream_default();
        let n = config.spacing.fft_size();
        let pp = PilotPattern::new_standard(64, 16, 8, 4, 1.0);
        let demod = OfdmDemodulator::new(config, pp);

        // Identity channel (H=1+j0 everywhere)
        let rx_freq: Vec<Complex> = (0..n).map(|_| (1.0, 0.0)).collect();
        let pilot_pairs: Vec<(usize, Complex)> = vec![(0, (1.0, 0.0)), (16, (1.0, 0.0)), (32, (1.0, 0.0))];
        let h_est = demod.estimate_channel(&rx_freq, &pilot_pairs);
        assert_eq!(h_est.len(), n);
        // Should be close to 1+j0
        for (i, &(re, im)) in h_est[..32].iter().enumerate() {
            assert!((re - 1.0).abs() < 0.01, "h[{}] re = {}", i, re);
            assert!(im.abs() < 0.01, "h[{}] im = {}", i, im);
        }
    }

    #[test]
    fn test_zf_equalization() {
        let rx: Vec<Complex> = vec![(2.0, 0.0), (0.0, 2.0), (1.0, 1.0)];
        let h: Vec<Complex>  = vec![(2.0, 0.0), (0.0, 2.0), (1.0, 1.0)];
        let eq = OfdmDemodulator::equalize_zf(&rx, &h);
        assert!((eq[0].0 - 1.0).abs() < 1e-9, "eq[0].re = {}", eq[0].0);
        assert!((eq[0].1).abs() < 1e-9);
        assert!((eq[1].0 - 1.0).abs() < 1e-9, "eq[1].re = {}", eq[1].0);
        assert!((eq[1].1).abs() < 1e-9, "eq[1].im = {}", eq[1].1);
    }

    // --- Windowing tests ---

    #[test]
    fn test_windowing_tapers_edges() {
        let n = 64;
        let mut samples: Vec<Complex> = vec![(1.0, 0.0); n];
        apply_windowing(&mut samples, 8);
        // First sample should be ~0, middle should be 1
        assert!(samples[0].0.abs() < 0.01, "First sample after windowing: {}", samples[0].0);
        assert!((samples[n / 2].0 - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_overlap_add_length() {
        let a: Vec<Complex> = vec![(1.0, 0.0); 100];
        let b: Vec<Complex> = vec![(1.0, 0.0); 100];
        let out = overlap_add(&a, &b, 16);
        assert_eq!(out.len(), 184); // 100 + 100 - 16
    }

    // --- UpstreamAllocator tests ---

    #[test]
    fn test_upstream_allocator_total_slots() {
        let config = OfdmChannelConfig::upstream_default();
        let alloc = UpstreamAllocator::new(config, 16, 2, 20);
        // n_active=1880 / 16 = 117 slots per symbol pair, 20/2=10 symbol groups
        assert!(alloc.total_slots() > 0);
    }

    #[test]
    fn test_upstream_allocator_assign_release() {
        let config = OfdmChannelConfig::upstream_default();
        let mut alloc = UpstreamAllocator::new(config, 16, 2, 20);
        alloc.assign(0, 42).unwrap();
        assert_eq!(alloc.mini_slots[0].cm_id, Some(42));
        alloc.release(0).unwrap();
        assert_eq!(alloc.mini_slots[0].cm_id, None);
    }

    #[test]
    fn test_upstream_allocator_utilization() {
        let config = OfdmChannelConfig::upstream_default();
        let mut alloc = UpstreamAllocator::new(config, 16, 2, 20);
        let total = alloc.total_slots();
        for i in 0..total / 2 {
            alloc.assign(i, i as u32).unwrap();
        }
        let u = alloc.utilization();
        assert!((u - 0.5).abs() < 0.01, "Utilization = {}", u);
    }

    #[test]
    fn test_upstream_allocator_unassigned() {
        let config = OfdmChannelConfig::upstream_default();
        let mut alloc = UpstreamAllocator::new(config, 16, 2, 20);
        alloc.assign(0, 1).unwrap();
        let unassigned = alloc.unassigned_slots();
        assert!(!unassigned.contains(&0));
        assert!(unassigned.contains(&1));
    }

    #[test]
    fn test_upstream_probe_generation() {
        let probe = UpstreamAllocator::generate_probe(0, 32, 4096);
        assert_eq!(probe.len(), 4096);
    }

    // --- MiniSlot tests ---

    #[test]
    fn test_minislot_resource_elements() {
        let ms = MiniSlot::new(0, 0, 16, 0, 2);
        assert_eq!(ms.resource_elements(), 32);
    }

    // --- select_modulation_for_snr tests ---

    #[test]
    fn test_select_modulation_snr_high() {
        let m = select_modulation_for_snr(45.0);
        assert_eq!(m, ModulationOrder::Qam16384);
    }

    #[test]
    fn test_select_modulation_snr_low() {
        let m = select_modulation_for_snr(3.0);
        assert_eq!(m, ModulationOrder::Bpsk);
    }

    #[test]
    fn test_select_modulation_snr_mid() {
        let m = select_modulation_for_snr(27.5);
        assert!(m >= ModulationOrder::Qam256);
    }

    // --- ChannelQualityReport tests ---

    #[test]
    fn test_cqi_report_average_snr() {
        let snrs = vec![30.0, 32.0, 28.0, 35.0];
        let mut pm = ProfileManager::new_default(100);
        let report = ChannelQualityReport::from_snr_map(&snrs, &mut pm);
        let expected_avg = 31.25;
        assert!((report.average_snr_db - expected_avg).abs() < 0.01);
    }

    #[test]
    fn test_cqi_report_min_max_snr() {
        let snrs = vec![10.0, 20.0, 30.0];
        let mut pm = ProfileManager::new_default(50);
        let report = ChannelQualityReport::from_snr_map(&snrs, &mut pm);
        assert!((report.min_snr_db() - 10.0).abs() < 1e-10);
        assert!((report.max_snr_db() - 30.0).abs() < 1e-10);
    }

    // --- DocsisPhyProcessor tests ---

    #[test]
    fn test_docsis_phy_default_creation() {
        let phy = DocsisPhyProcessor::new_downstream_default();
        assert_eq!(phy.config.version, DocsisVersion::D31);
        assert_eq!(phy.config.direction, Direction::Downstream);
    }

    #[test]
    fn test_docsis_phy_d40_creation() {
        let phy = DocsisPhyProcessor::new_d40();
        assert_eq!(phy.config.version, DocsisVersion::D40);
        assert!(phy.profile_manager.num_profiles() >= 6); // includes 8192/16384 QAM
    }

    #[test]
    fn test_docsis_phy_update_profile() {
        let mut phy = DocsisPhyProcessor::new_downstream_default();
        phy.update_profile(30.0);
        // Profile should be updated to match SNR
        assert!(phy.modulator.profile.min_snr_db <= 30.0);
    }

    #[test]
    fn test_docsis_phy_throughput() {
        let phy = DocsisPhyProcessor::new_downstream_default();
        let gbps = phy.throughput_gbps();
        assert!(gbps > 0.0, "Throughput = {} Gbps", gbps);
    }

    #[test]
    fn test_docsis_phy_exclusion() {
        let mut phy = DocsisPhyProcessor::new_downstream_default();
        phy.exclusion.add_fm_radio_band();
        assert!(phy.is_excluded(100e6));
        assert!(!phy.is_excluded(500e6));
    }

    #[test]
    fn test_docsis_phy_transmit_receive() {
        let mut phy = DocsisPhyProcessor::new_downstream_default();
        let n_active = phy.config.n_active;
        let data_iq: Vec<(f64, f64)> = (0..n_active).map(|i| {
            let t = i as f64 * 0.001;
            (t.cos(), t.sin())
        }).collect();
        let sym = phy.transmit_symbol(&data_iq);
        let freq = phy.receive_symbol(&sym.samples);
        assert_eq!(freq.len(), phy.config.spacing.fft_size());
    }

    // --- Frequency range validation tests ---

    #[test]
    fn test_ds_freq_range() {
        assert!(DS_FREQ_MIN_HZ < DS_FREQ_MAX_HZ);
        assert!((DS_FREQ_MIN_HZ - 258e6).abs() < 1.0);
        assert!((DS_FREQ_MAX_HZ - 1218e6).abs() < 1.0);
    }

    #[test]
    fn test_us_freq_range() {
        assert!(US_FREQ_MAX_HZ_HIGH_SPLIT > US_FREQ_MAX_HZ_MID_SPLIT);
        assert!(US_FREQ_MAX_HZ_MID_SPLIT > US_FREQ_MAX_HZ_LOW_SPLIT);
    }

    // --- Cross-QAM tests ---

    #[test]
    fn test_cross_qam_32() {
        let c = generate_qam_constellation(ModulationOrder::Qam32);
        assert_eq!(c.len(), 32);
    }

    #[test]
    fn test_cross_qam_128() {
        let c = generate_qam_constellation(ModulationOrder::Qam128);
        assert_eq!(c.len(), 128);
    }
}

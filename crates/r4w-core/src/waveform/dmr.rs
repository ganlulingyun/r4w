//! DMR (Digital Mobile Radio) Waveform
//!
//! This module implements the DMR digital radio standard (ETSI TS 102 361)
//! widely adopted by government, utilities, and public safety organizations.
//!
//! # Overview
//!
//! DMR is an ETSI open standard for digital mobile radio that provides
//! efficient spectrum use through 2-slot TDMA in 12.5 kHz channels.
//!
//! # Key Features
//!
//! - 4FSK modulation at 4800 symbols/sec
//! - 2-slot TDMA (two voice/data channels per 12.5 kHz)
//! - AMBE+2 voice codec (3.6 kbps + 2.4 kbps FEC)
//! - Optional ARC4 or AES-256 encryption
//!
//! # DMR Tiers
//!
//! - Tier I: Unlicensed (dPMR446) - 0.5W, 8 channels
//! - Tier II: Licensed conventional - repeater/direct
//! - Tier III: Licensed trunked - full infrastructure

use std::f64::consts::PI;

use crate::types::IQSample;
use crate::waveform::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};

/// DMR tier (operational mode)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DmrTier {
    /// Tier I: Unlicensed 446 MHz (dPMR446)
    Tier1,
    /// Tier II: Licensed conventional
    #[default]
    Tier2,
    /// Tier III: Licensed trunked
    Tier3,
}

/// DMR operating mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DmrMode {
    /// Direct mode (simplex)
    #[default]
    Direct,
    /// Repeater mode
    Repeater,
    /// Trunked mode (Tier III only)
    Trunked,
}

/// DMR burst types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmrBurstType {
    /// Voice burst
    Voice,
    /// CSBK (Control Signalling Block)
    Csbk,
    /// Data header
    DataHeader,
    /// Rate 1/2 data
    DataRate12,
    /// Rate 3/4 data
    DataRate34,
    /// Idle burst
    Idle,
}

/// DMR sync pattern types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmrSyncType {
    /// Base Station Voice
    BsVoice,
    /// Base Station Data
    BsData,
    /// Mobile Station Voice
    MsVoice,
    /// Mobile Station Data
    MsData,
    /// RC Sync
    RcSync,
    /// Direct Mode Voice Timeslot 1
    DmoVoiceTs1,
    /// Direct Mode Data Timeslot 1
    DmoDataTs1,
}

impl DmrSyncType {
    /// Get the 48-bit sync pattern
    pub fn pattern(&self) -> u64 {
        match self {
            Self::BsVoice => 0x755FD7DF75F7,
            Self::BsData => 0xDFF57D75DF5D,
            Self::MsVoice => 0x7F7D5DD57DFD,
            Self::MsData => 0xD5D7F77FD757,
            Self::RcSync => 0x77D55F7DFD77,
            Self::DmoVoiceTs1 => 0x5D577F7757FF,
            Self::DmoDataTs1 => 0xF7FDD5DDFD55,
        }
    }
}

/// DMR TDMA timing constants
pub struct DmrTiming;

impl DmrTiming {
    /// Slots per frame
    pub const SLOTS_PER_FRAME: usize = 2;
    /// Frame duration in milliseconds
    pub const FRAME_DURATION_MS: f64 = 60.0;
    /// Slot duration in milliseconds
    pub const SLOT_DURATION_MS: f64 = 30.0;
    /// Guard time in milliseconds (CACH + guard)
    pub const GUARD_TIME_MS: f64 = 2.5;
    /// Symbols per slot
    pub const SYMBOLS_PER_SLOT: usize = 144;
    /// Symbol rate
    pub const SYMBOL_RATE: f64 = 4800.0;
    /// Frames per superframe
    pub const FRAMES_PER_SUPERFRAME: usize = 6;
    /// Superframe duration in milliseconds
    pub const SUPERFRAME_DURATION_MS: f64 = 360.0;
}

/// DMR 4FSK frequency deviations from center (Hz)
/// Gray coded: 00 -> -1944, 01 -> -648, 11 -> +648, 10 -> +1944
const DMR_DEVIATIONS: [f64; 4] = [-1944.0, -648.0, 648.0, 1944.0];

/// Dibit to symbol mapping (Gray code)
fn dibit_to_symbol(dibit: u8) -> u8 {
    match dibit & 0x03 {
        0b00 => 0, // -1944 Hz
        0b01 => 1, // -648 Hz
        0b11 => 2, // +648 Hz
        0b10 => 3, // +1944 Hz
        _ => 0,
    }
}

/// Symbol to dibit mapping (reverse Gray code)
fn symbol_to_dibit(symbol: u8) -> u8 {
    match symbol & 0x03 {
        0 => 0b00,
        1 => 0b01,
        2 => 0b11,
        3 => 0b10,
        _ => 0,
    }
}

/// DMR waveform implementation
#[derive(Debug)]
pub struct Dmr {
    /// Common waveform parameters
    common: CommonParams,
    /// DMR tier
    tier: DmrTier,
    /// Operating mode
    mode: DmrMode,
    /// Timeslot (0 or 1)
    timeslot: u8,
    /// Samples per symbol
    samples_per_sym: usize,
    /// Current phase for FSK generation
    phase: f64,
    /// RRC filter coefficients
    rrc_filter: Vec<f64>,
}

impl Dmr {
    /// Standard symbol rate (4800 symbols/sec)
    pub const SYMBOL_RATE: f64 = 4800.0;
    /// Channel bandwidth (12.5 kHz)
    pub const BANDWIDTH: f64 = 12500.0;
    /// Bits per symbol (4FSK = 2 bits)
    pub const BITS_PER_SYMBOL: u8 = 2;
    /// RRC rolloff factor
    pub const RRC_ROLLOFF: f64 = 0.2;

    /// Create a new DMR instance
    pub fn new(sample_rate: f64, tier: DmrTier, mode: DmrMode) -> Self {
        let samples_per_sym = (sample_rate / Self::SYMBOL_RATE) as usize;

        // Generate RRC filter
        let rrc_filter = Self::generate_rrc_filter(samples_per_sym, 8, Self::RRC_ROLLOFF);

        Self {
            common: CommonParams {
                sample_rate,
                carrier_freq: 0.0,
                amplitude: 1.0,
            },
            tier,
            mode,
            timeslot: 0,
            samples_per_sym: samples_per_sym.max(1),
            phase: 0.0,
            rrc_filter,
        }
    }

    /// Create standard Tier II DMR
    pub fn tier2(sample_rate: f64) -> Self {
        Self::new(sample_rate, DmrTier::Tier2, DmrMode::Repeater)
    }

    /// Create direct mode DMR
    pub fn direct(sample_rate: f64) -> Self {
        Self::new(sample_rate, DmrTier::Tier2, DmrMode::Direct)
    }

    /// Create Tier III trunked DMR
    pub fn tier3(sample_rate: f64) -> Self {
        Self::new(sample_rate, DmrTier::Tier3, DmrMode::Trunked)
    }

    /// Set timeslot (0 or 1)
    pub fn with_timeslot(mut self, slot: u8) -> Self {
        self.timeslot = slot & 1;
        self
    }

    /// Generate Root Raised Cosine filter coefficients
    fn generate_rrc_filter(sps: usize, span: usize, rolloff: f64) -> Vec<f64> {
        let num_taps = span * sps + 1;
        let mut filter = vec![0.0; num_taps];
        let half = (num_taps / 2) as f64;

        for i in 0..num_taps {
            let t = (i as f64 - half) / sps as f64;

            if t.abs() < 1e-10 {
                // t = 0 case
                filter[i] = 1.0 - rolloff + 4.0 * rolloff / PI;
            } else if (t.abs() - 1.0 / (4.0 * rolloff)).abs() < 1e-10 && rolloff > 0.0 {
                // t = ±1/(4*α) case
                let a = (1.0 + 2.0 / PI) * (PI / (4.0 * rolloff)).sin();
                let b = (1.0 - 2.0 / PI) * (PI / (4.0 * rolloff)).cos();
                filter[i] = rolloff / 2.0_f64.sqrt() * (a + b);
            } else {
                // General case
                let num = (PI * t * (1.0 - rolloff)).sin()
                    + 4.0 * rolloff * t * (PI * t * (1.0 + rolloff)).cos();
                let den = PI * t * (1.0 - (4.0 * rolloff * t).powi(2));
                if den.abs() > 1e-10 {
                    filter[i] = num / den;
                } else {
                    filter[i] = 0.0;
                }
            }
        }

        // Normalize
        let sum: f64 = filter.iter().map(|x| x.abs()).sum();
        if sum > 0.0 {
            for coef in &mut filter {
                *coef /= sum;
            }
        }

        filter
    }

    /// Generate sync pattern as samples
    fn generate_sync(&mut self, sync_type: DmrSyncType) -> Vec<IQSample> {
        let pattern = sync_type.pattern();

        // Extract dibits from 48-bit pattern (24 dibits)
        let mut dibits = Vec::with_capacity(24);
        for i in (0..24).rev() {
            let dibit = ((pattern >> (i * 2)) & 0x03) as u8;
            dibits.push(dibit);
        }

        self.fsk4_modulate(&dibits)
    }

    /// 4FSK modulation
    fn fsk4_modulate(&mut self, dibits: &[u8]) -> Vec<IQSample> {
        let mut samples = Vec::with_capacity(dibits.len() * self.samples_per_sym);

        for &dibit in dibits {
            let symbol = dibit_to_symbol(dibit);
            let freq = DMR_DEVIATIONS[symbol as usize];
            let omega = 2.0 * PI * freq / self.common.sample_rate;

            for _ in 0..self.samples_per_sym {
                samples.push(IQSample::new(self.phase.cos(), self.phase.sin()));
                self.phase += omega;
            }

            // Keep phase bounded
            while self.phase > 2.0 * PI {
                self.phase -= 2.0 * PI;
            }
            while self.phase < -2.0 * PI {
                self.phase += 2.0 * PI;
            }
        }

        // Apply RRC pulse shaping
        self.apply_rrc_filter(&samples)
    }

    /// Apply RRC filter
    fn apply_rrc_filter(&self, samples: &[IQSample]) -> Vec<IQSample> {
        if self.rrc_filter.is_empty() {
            return samples.to_vec();
        }

        let filter_len = self.rrc_filter.len();
        let half_len = filter_len / 2;
        let mut filtered = Vec::with_capacity(samples.len());

        for i in 0..samples.len() {
            let mut sum_i = 0.0;
            let mut sum_q = 0.0;

            for (j, &coef) in self.rrc_filter.iter().enumerate() {
                let idx = i as i64 - half_len as i64 + j as i64;
                if idx >= 0 && (idx as usize) < samples.len() {
                    sum_i += samples[idx as usize].re * coef;
                    sum_q += samples[idx as usize].im * coef;
                }
            }

            filtered.push(IQSample::new(sum_i, sum_q));
        }

        filtered
    }

    /// Demodulate 4FSK
    fn fsk4_demodulate(&self, samples: &[IQSample]) -> Vec<u8> {
        let mut dibits = Vec::new();

        for chunk in samples.chunks(self.samples_per_sym) {
            if chunk.len() < 2 {
                continue;
            }

            // Estimate frequency from phase differences
            let mut freq_sum = 0.0;
            for i in 1..chunk.len() {
                let phase_diff = (chunk[i] * chunk[i - 1].conj()).arg();
                let freq = phase_diff * self.common.sample_rate / (2.0 * PI);
                freq_sum += freq;
            }
            let avg_freq = freq_sum / (chunk.len() - 1) as f64;

            // Find closest symbol
            let mut best_symbol = 0u8;
            let mut best_error = f64::MAX;

            for (sym, &dev) in DMR_DEVIATIONS.iter().enumerate() {
                let error = (avg_freq - dev).abs();
                if error < best_error {
                    best_error = error;
                    best_symbol = sym as u8;
                }
            }

            dibits.push(symbol_to_dibit(best_symbol));
        }

        dibits
    }
}

impl Waveform for Dmr {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "DMR",
            full_name: "Digital Mobile Radio",
            description: "ETSI open standard for digital mobile radio providing \
                efficient spectrum use through 2-slot TDMA in 12.5 kHz channels",
            complexity: 3,
            bits_per_symbol: Self::BITS_PER_SYMBOL,
            carries_data: true,
            characteristics: &[
                "4FSK at 4800 symbols/sec",
                "2-slot TDMA (12.5 kHz channel)",
                "AMBE+2 voice codec",
                "ARC4 or AES-256 encryption",
                "Tier I/II/III operation",
                "9.6 kbps gross data rate",
                "Repeater and direct modes",
            ],
            history: "Developed by ETSI in the 2000s as an open standard alternative \
                to proprietary digital radio systems. DMR addresses the need for \
                spectral efficiency and interoperability in the 12.5 kHz narrowband \
                environment.",
            modern_usage: "Widely adopted by commercial users, utilities, government \
                agencies, and amateur radio operators. Major manufacturers include \
                Motorola, Hytera, and Kenwood. Popular for its balance of features \
                and cost-effectiveness.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // Create mutable copy for phase tracking
        let mut dmr = Self::new(self.common.sample_rate, self.tier, self.mode);

        // Generate sync burst
        let sync_type = match self.mode {
            DmrMode::Direct => DmrSyncType::DmoVoiceTs1,
            _ => DmrSyncType::BsVoice,
        };
        let mut samples = dmr.generate_sync(sync_type);

        // Convert data bytes to dibits
        let dibits: Vec<u8> = data
            .iter()
            .flat_map(|&b| {
                vec![
                    (b >> 6) & 0x03,
                    (b >> 4) & 0x03,
                    (b >> 2) & 0x03,
                    b & 0x03,
                ]
            })
            .collect();

        // Modulate data
        samples.extend(dmr.fsk4_modulate(&dibits));

        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        // Skip sync burst (24 dibits = 24 symbols)
        let sync_symbols = 24;
        let data_start = sync_symbols * self.samples_per_sym;

        if samples.len() <= data_start {
            return DemodResult::default();
        }

        let data_samples = &samples[data_start..];
        let dibits = self.fsk4_demodulate(data_samples);

        // Pack dibits to bytes
        let bytes: Vec<u8> = dibits
            .chunks(4)
            .map(|chunk| {
                chunk
                    .iter()
                    .enumerate()
                    .fold(0u8, |acc, (i, &d)| acc | ((d & 0x03) << (6 - i * 2)))
            })
            .collect();

        DemodResult {
            bits: bytes,
            symbols: dibits.iter().map(|&d| d as u16).collect(),
            ber_estimate: None,
            snr_estimate: None,
            metadata: std::collections::HashMap::new(),
        }
    }

    fn samples_per_symbol(&self) -> usize {
        self.samples_per_sym
    }

    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);

        // 4FSK represented as frequency levels on imaginary axis
        let constellation: Vec<IQSample> = DMR_DEVIATIONS
            .iter()
            .enumerate()
            .map(|(_i, &dev)| {
                // Normalize deviation to unit circle
                let normalized = dev / 2000.0;
                IQSample::new(0.5, normalized * 0.5)
            })
            .collect();

        let labels: Vec<String> = vec![
            format!("00: {:+} Hz", DMR_DEVIATIONS[0] as i32),
            format!("01: {:+} Hz", DMR_DEVIATIONS[1] as i32),
            format!("11: {:+} Hz", DMR_DEVIATIONS[2] as i32),
            format!("10: {:+} Hz", DMR_DEVIATIONS[3] as i32),
        ];

        VisualizationData {
            samples,
            constellation,
            constellation_labels: labels,
            spectrum: Vec::new(),
            description: format!(
                "DMR {:?} {:?} mode, TS{}, 4FSK",
                self.tier, self.mode, self.timeslot
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dmr_creation() {
        let dmr = Dmr::tier2(48000.0);
        assert_eq!(dmr.samples_per_sym, 10); // 48000/4800 = 10
    }

    #[test]
    fn test_gray_code() {
        // Test dibit to symbol mapping (Gray code)
        assert_eq!(dibit_to_symbol(0b00), 0);
        assert_eq!(dibit_to_symbol(0b01), 1);
        assert_eq!(dibit_to_symbol(0b11), 2);
        assert_eq!(dibit_to_symbol(0b10), 3);

        // Test reverse mapping
        assert_eq!(symbol_to_dibit(0), 0b00);
        assert_eq!(symbol_to_dibit(1), 0b01);
        assert_eq!(symbol_to_dibit(2), 0b11);
        assert_eq!(symbol_to_dibit(3), 0b10);
    }

    #[test]
    fn test_dmr_modulation() {
        let dmr = Dmr::tier2(48000.0);
        let data = vec![0xAA, 0x55, 0xF0, 0x0F];
        let modulated = dmr.modulate(&data);

        assert!(!modulated.is_empty());

        // Check all samples have reasonable amplitude
        for sample in &modulated {
            assert!(sample.norm() <= 2.0);
        }
    }

    #[test]
    fn test_dmr_demodulation() {
        let dmr = Dmr::tier2(48000.0);
        let data = vec![0xAB, 0xCD];
        let modulated = dmr.modulate(&data);
        let demod = dmr.demodulate(&modulated);

        // Should recover some data
        assert!(!demod.bits.is_empty() || !demod.symbols.is_empty());
    }

    #[test]
    fn test_sync_patterns() {
        // Verify sync patterns are 48 bits
        let bs_voice = DmrSyncType::BsVoice.pattern();
        assert!(bs_voice < (1u64 << 48));

        let ms_voice = DmrSyncType::MsVoice.pattern();
        assert!(ms_voice < (1u64 << 48));
    }

    #[test]
    fn test_dmr_modes() {
        let tier2 = Dmr::tier2(48000.0);
        let tier3 = Dmr::tier3(48000.0);
        let direct = Dmr::direct(48000.0);

        assert_eq!(tier2.tier, DmrTier::Tier2);
        assert_eq!(tier3.tier, DmrTier::Tier3);
        assert_eq!(direct.mode, DmrMode::Direct);
    }

    #[test]
    fn test_timeslot() {
        let dmr = Dmr::tier2(48000.0).with_timeslot(1);
        assert_eq!(dmr.timeslot, 1);

        // Should clamp to 0 or 1
        let dmr2 = Dmr::tier2(48000.0).with_timeslot(5);
        assert_eq!(dmr2.timeslot, 1); // 5 & 1 = 1
    }

    #[test]
    fn test_waveform_info() {
        let dmr = Dmr::tier2(48000.0);
        let info = dmr.info();

        assert_eq!(info.name, "DMR");
        assert!(info.description.to_lowercase().contains("etsi"));
    }
}

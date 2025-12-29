//! APCO Project 25 (P25) Digital Voice and Data
//!
//! This module implements the APCO Project 25 digital radio standard
//! used by public safety and government agencies.
//!
//! # Overview
//!
//! P25 (Project 25) is a suite of standards for digital two-way radio
//! communications. It provides:
//! - Digital voice with IMBE/AMBE+ codecs
//! - Secure encryption (DES, AES)
//! - Data messaging
//! - Trunking capability
//!
//! # Phases
//!
//! - **Phase 1**: C4FM/CQPSK FDMA, 12.5 kHz channels
//! - **Phase 2**: H-DQPSK TDMA, 6.25 kHz equivalent channels

use std::f64::consts::PI;

use crate::types::IQSample;
use crate::waveform::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};

/// P25 Phase selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Phase {
    /// Phase 1 - FDMA (C4FM or CQPSK)
    #[default]
    Phase1,
    /// Phase 2 - TDMA (H-DQPSK)
    Phase2,
}

/// Modulation types for P25
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum P25Modulation {
    /// Continuous 4-level FM (Phase 1)
    #[default]
    C4fm,
    /// Compatible Quadrature PSK (Phase 1 for simulcast)
    Cqpsk,
    /// H-DQPSK (Phase 2)
    Hdqpsk,
}

impl P25Modulation {
    /// Get symbol rate
    pub fn symbol_rate(&self) -> f64 {
        match self {
            Self::C4fm | Self::Cqpsk => 4800.0,
            Self::Hdqpsk => 6000.0,
        }
    }

    /// Get bits per symbol
    pub fn bits_per_symbol(&self) -> u8 {
        2 // All P25 modulations use 2 bits per symbol
    }
}

/// Network Access Code (NAC) - identifies P25 system
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Nac(pub u16);

impl Nac {
    /// Default NAC (0x293)
    pub const DEFAULT: Self = Self(0x293);

    /// Create new NAC (12 bits)
    pub fn new(value: u16) -> Self {
        Self(value & 0xFFF)
    }
}

impl Default for Nac {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Data Unit ID - identifies type of transmission
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Duid {
    /// Header Data Unit
    Hdu,
    /// Terminator without Link Control
    Tdu,
    /// Logical Link Data Unit 1
    Ldu1,
    /// Logical Link Data Unit 2
    Ldu2,
    /// Packet Data Unit
    Pdu,
    /// Terminator with Link Control
    TduLc,
    /// Trunking Signaling Data Unit
    Tsdu,
}

impl Duid {
    /// Get the DUID value
    pub fn value(&self) -> u8 {
        match self {
            Self::Hdu => 0x0,
            Self::Tdu => 0x3,
            Self::Ldu1 => 0x5,
            Self::Ldu2 => 0xA,
            Self::Pdu => 0xC,
            Self::TduLc => 0xF,
            Self::Tsdu => 0x7,
        }
    }
}

/// Frame sync pattern (48 bits)
const FRAME_SYNC: u64 = 0x5575F5FF77FF;

/// Status symbols
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StatusSymbol {
    /// Inbound busy
    InboundBusy,
    /// Inbound idle
    InboundIdle,
    /// Unknown
    Unknown,
}

/// P25 waveform implementation
#[derive(Debug)]
pub struct P25 {
    /// Common waveform parameters
    common: CommonParams,
    /// Phase (1 or 2)
    phase: Phase,
    /// Modulation type
    modulation: P25Modulation,
    /// Network Access Code
    nac: Nac,
    /// Samples per symbol
    samples_per_sym: usize,
    /// Symbol rate
    #[allow(dead_code)]
    symbol_rate: f64,
    /// Deviation for C4FM (Hz)
    deviation: f64,
    /// Carrier offset
    carrier_offset: f64,
}

impl P25 {
    /// Standard P25 symbol rate
    pub const SYMBOL_RATE: f64 = 4800.0;
    /// Standard deviation for C4FM
    pub const DEVIATION: f64 = 1800.0;
    /// Channel bandwidth
    pub const BANDWIDTH: f64 = 12500.0;

    /// Create a new P25 instance
    pub fn new(sample_rate: f64, phase: Phase, modulation: P25Modulation) -> Self {
        let symbol_rate = modulation.symbol_rate();
        let samples_per_sym = (sample_rate / symbol_rate) as usize;

        Self {
            common: CommonParams {
                sample_rate,
                carrier_freq: 0.0,
                amplitude: 1.0,
            },
            phase,
            modulation,
            nac: Nac::default(),
            samples_per_sym,
            symbol_rate,
            deviation: Self::DEVIATION,
            carrier_offset: 0.0,
        }
    }

    /// Create Phase 1 C4FM instance
    pub fn phase1_c4fm(sample_rate: f64) -> Self {
        Self::new(sample_rate, Phase::Phase1, P25Modulation::C4fm)
    }

    /// Create Phase 1 CQPSK instance
    pub fn phase1_cqpsk(sample_rate: f64) -> Self {
        Self::new(sample_rate, Phase::Phase1, P25Modulation::Cqpsk)
    }

    /// Create Phase 2 TDMA instance
    pub fn phase2(sample_rate: f64) -> Self {
        Self::new(sample_rate, Phase::Phase2, P25Modulation::Hdqpsk)
    }

    /// Set Network Access Code
    pub fn set_nac(&mut self, nac: Nac) {
        self.nac = nac;
    }

    /// Get current NAC
    pub fn nac(&self) -> Nac {
        self.nac
    }

    /// Generate frame sync
    fn generate_frame_sync(&self) -> Vec<IQSample> {
        // Convert 48-bit sync to dibits
        let mut dibits = Vec::with_capacity(24);
        for i in (0..48).step_by(2) {
            let dibit = ((FRAME_SYNC >> (46 - i)) & 0x03) as u8;
            dibits.push(dibit);
        }

        self.modulate_dibits(&dibits)
    }

    /// Generate Network ID (NID) from NAC and DUID
    fn generate_nid(&self, duid: Duid) -> Vec<IQSample> {
        // NID = NAC (12 bits) + DUID (4 bits) + parity
        let nid = ((self.nac.0 as u64) << 4) | (duid.value() as u64);

        // Convert to dibits (simplified - real P25 uses BCH encoding)
        let mut dibits = Vec::with_capacity(32);
        for i in (0..32).step_by(2) {
            let dibit = if i < 16 {
                ((nid >> (14 - i)) & 0x03) as u8
            } else {
                // Parity bits (simplified)
                ((nid >> (30 - i)) & 0x03) as u8
            };
            dibits.push(dibit);
        }

        self.modulate_dibits(&dibits)
    }

    /// Modulate dibits (2-bit symbols) to IQ samples
    fn modulate_dibits(&self, dibits: &[u8]) -> Vec<IQSample> {
        match self.modulation {
            P25Modulation::C4fm => self.c4fm_modulate(dibits),
            P25Modulation::Cqpsk => self.cqpsk_modulate(dibits),
            P25Modulation::Hdqpsk => self.hdqpsk_modulate(dibits),
        }
    }

    /// C4FM modulation (4-level FM)
    fn c4fm_modulate(&self, dibits: &[u8]) -> Vec<IQSample> {
        let mut samples = Vec::with_capacity(dibits.len() * self.samples_per_sym);
        let mut phase = 0.0f64;

        // C4FM symbol mapping: 00->+3, 01->+1, 10->-1, 11->-3
        let levels = [3.0, 1.0, -1.0, -3.0];

        for &dibit in dibits {
            let level = levels[(dibit & 0x03) as usize];
            let freq_dev = level * self.deviation / 3.0;

            for i in 0..self.samples_per_sym {
                // Apply raised cosine shaping
                let t = i as f64 / self.samples_per_sym as f64;
                let shape = if t < 0.25 || t > 0.75 {
                    0.5 * (1.0 - (2.0 * PI * t).cos())
                } else {
                    1.0
                };

                phase += 2.0 * PI * freq_dev * shape / self.common.sample_rate;
                samples.push(IQSample::new(phase.cos(), phase.sin()));
            }
        }

        samples
    }

    /// CQPSK modulation (Compatible QPSK)
    fn cqpsk_modulate(&self, dibits: &[u8]) -> Vec<IQSample> {
        let mut samples = Vec::with_capacity(dibits.len() * self.samples_per_sym);
        let mut phase = 0.0f64;

        // CQPSK constellation points (45, 135, 225, 315 degrees)
        let angles = [PI / 4.0, 3.0 * PI / 4.0, 5.0 * PI / 4.0, 7.0 * PI / 4.0];

        for (sym_idx, &dibit) in dibits.iter().enumerate() {
            let target_phase = angles[(dibit & 0x03) as usize];

            for i in 0..self.samples_per_sym {
                let t = (sym_idx * self.samples_per_sym + i) as f64 / self.common.sample_rate;

                // Smooth phase transition
                let alpha = i as f64 / self.samples_per_sym as f64;
                let current_phase = phase + alpha * (target_phase - phase);

                let carrier_phase = 2.0 * PI * self.carrier_offset * t;
                samples.push(IQSample::new(
                    (current_phase + carrier_phase).cos(),
                    (current_phase + carrier_phase).sin(),
                ));
            }

            phase = target_phase;
        }

        samples
    }

    /// H-DQPSK modulation (Phase 2)
    fn hdqpsk_modulate(&self, dibits: &[u8]) -> Vec<IQSample> {
        let mut samples = Vec::with_capacity(dibits.len() * self.samples_per_sym);
        let mut phase = 0.0f64;

        // Differential phase changes
        let phase_changes = [PI / 4.0, 3.0 * PI / 4.0, -3.0 * PI / 4.0, -PI / 4.0];

        for (sym_idx, &dibit) in dibits.iter().enumerate() {
            let delta_phase = phase_changes[(dibit & 0x03) as usize];
            phase += delta_phase;

            for i in 0..self.samples_per_sym {
                let t = (sym_idx * self.samples_per_sym + i) as f64 / self.common.sample_rate;
                let carrier_phase = 2.0 * PI * self.carrier_offset * t;

                samples.push(IQSample::new(
                    (phase + carrier_phase).cos(),
                    (phase + carrier_phase).sin(),
                ));
            }
        }

        samples
    }

    /// Demodulate C4FM
    fn c4fm_demodulate(&self, samples: &[IQSample]) -> Vec<u8> {
        let mut dibits = Vec::new();

        for chunk in samples.chunks(self.samples_per_sym) {
            if chunk.len() < self.samples_per_sym / 2 {
                break;
            }

            // Frequency discrimination
            let mut freq_sum = 0.0;
            for i in 1..chunk.len() {
                let phase_diff = (chunk[i].im * chunk[i - 1].re - chunk[i].re * chunk[i - 1].im)
                    .atan2(chunk[i].re * chunk[i - 1].re + chunk[i].im * chunk[i - 1].im);
                freq_sum += phase_diff;
            }
            let avg_freq = freq_sum / (chunk.len() - 1) as f64;

            // Map to dibit based on frequency
            let normalized = avg_freq * self.common.sample_rate / (2.0 * PI * self.deviation / 3.0);
            let dibit = if normalized > 2.0 {
                0
            } else if normalized > 0.0 {
                1
            } else if normalized > -2.0 {
                2
            } else {
                3
            };

            dibits.push(dibit);
        }

        dibits
    }

    /// Demodulate QPSK variants
    fn qpsk_demodulate(&self, samples: &[IQSample]) -> Vec<u8> {
        let mut dibits = Vec::new();

        for chunk in samples.chunks(self.samples_per_sym) {
            if chunk.len() < self.samples_per_sym / 2 {
                break;
            }

            // Average over symbol
            let avg_i: f64 = chunk.iter().map(|s| s.re).sum::<f64>() / chunk.len() as f64;
            let avg_q: f64 = chunk.iter().map(|s| s.im).sum::<f64>() / chunk.len() as f64;

            // Determine quadrant
            let dibit = match (avg_i >= 0.0, avg_q >= 0.0) {
                (true, true) => 0,
                (false, true) => 1,
                (false, false) => 2,
                (true, false) => 3,
            };

            dibits.push(dibit);
        }

        dibits
    }
}

impl Waveform for P25 {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "P25",
            full_name: "APCO Project 25 Digital Radio",
            description: "Public safety digital radio standard for voice and data \
                communications used by police, fire, EMS, and government agencies",
            complexity: 4,
            bits_per_symbol: 2,
            carries_data: true,
            characteristics: &[
                "VHF/UHF/700/800 MHz bands",
                "12.5 kHz channel (Phase 1)",
                "6.25 kHz equivalent (Phase 2 TDMA)",
                "C4FM/CQPSK modulation",
                "IMBE/AMBE+2 voice codec",
                "AES/DES encryption",
                "Trunking and conventional",
            ],
            history: "Developed in the 1990s by APCO (Association of Public-Safety \
                Communications Officials) and TIA. Phase 1 standardized FDMA operation. \
                Phase 2 added TDMA for improved spectrum efficiency.",
            modern_usage: "Widely deployed by public safety agencies across North America. \
                Interoperability standard for police, fire, and EMS. Being adopted by \
                military and federal agencies for non-tactical communications.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // Generate frame sync
        let mut samples = self.generate_frame_sync();

        // Generate NID (using LDU1 as default)
        samples.extend(self.generate_nid(Duid::Ldu1));

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
        samples.extend(self.modulate_dibits(&dibits));

        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        // Skip frame sync and NID (simplified)
        let header_symbols = 24 + 32; // sync + NID
        let data_start = header_symbols * self.samples_per_sym;

        if samples.len() <= data_start {
            return DemodResult::default();
        }

        let data_samples = &samples[data_start..];

        // Demodulate based on modulation type
        let dibits = match self.modulation {
            P25Modulation::C4fm => self.c4fm_demodulate(data_samples),
            P25Modulation::Cqpsk | P25Modulation::Hdqpsk => self.qpsk_demodulate(data_samples),
        };

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

        // Constellation depends on modulation
        let constellation = match self.modulation {
            P25Modulation::C4fm => {
                // C4FM shows as 4 points on real axis (frequency)
                vec![
                    IQSample::new(1.0, 0.0),
                    IQSample::new(0.33, 0.0),
                    IQSample::new(-0.33, 0.0),
                    IQSample::new(-1.0, 0.0),
                ]
            }
            P25Modulation::Cqpsk | P25Modulation::Hdqpsk => {
                // QPSK constellation
                let a = 1.0 / 2.0_f64.sqrt();
                vec![
                    IQSample::new(a, a),
                    IQSample::new(-a, a),
                    IQSample::new(-a, -a),
                    IQSample::new(a, -a),
                ]
            }
        };

        VisualizationData {
            samples,
            constellation,
            constellation_labels: vec![
                "00".to_string(),
                "01".to_string(),
                "10".to_string(),
                "11".to_string(),
            ],
            spectrum: Vec::new(),
            description: format!(
                "P25 {:?}: {:?} modulation, NAC 0x{:03X}",
                self.phase, self.modulation, self.nac.0
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_c4fm_modulation() {
        let p25 = P25::phase1_c4fm(48000.0);
        let data = vec![0xAA, 0x55, 0xF0, 0x0F];
        let modulated = p25.modulate(&data);

        assert!(!modulated.is_empty());

        // Check amplitude is reasonable
        for sample in &modulated {
            assert!(sample.norm() <= 1.5);
        }
    }

    #[test]
    fn test_cqpsk_modulation() {
        let p25 = P25::phase1_cqpsk(48000.0);
        let data = vec![0x12, 0x34, 0x56, 0x78];
        let modulated = p25.modulate(&data);

        assert!(!modulated.is_empty());
    }

    #[test]
    fn test_phase2_modulation() {
        let p25 = P25::phase2(48000.0);
        let data = vec![0xFF, 0x00, 0xAA, 0x55];
        let modulated = p25.modulate(&data);

        assert!(!modulated.is_empty());
    }

    #[test]
    fn test_nac() {
        let mut p25 = P25::phase1_c4fm(48000.0);
        assert_eq!(p25.nac().0, 0x293);

        p25.set_nac(Nac::new(0x123));
        assert_eq!(p25.nac().0, 0x123);
    }

    #[test]
    fn test_waveform_info() {
        let p25 = P25::phase1_c4fm(48000.0);
        let info = p25.info();

        assert_eq!(info.name, "P25");
        assert!(info.description.to_lowercase().contains("public safety"));
    }

    #[test]
    fn test_symbol_rates() {
        assert_eq!(P25Modulation::C4fm.symbol_rate(), 4800.0);
        assert_eq!(P25Modulation::Cqpsk.symbol_rate(), 4800.0);
        assert_eq!(P25Modulation::Hdqpsk.symbol_rate(), 6000.0);
    }
}

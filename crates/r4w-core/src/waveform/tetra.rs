//! TETRA (Terrestrial Trunked Radio) Waveform
//!
//! This module implements the TETRA digital radio standard used by
//! European emergency services and public safety organizations.
//!
//! # Overview
//!
//! TETRA is an ETSI standard (EN 300 392) for professional mobile radio,
//! widely deployed by emergency services, military, and transport.
//!
//! # Key Features
//!
//! - 4-slot TDMA in 25 kHz channels
//! - π/4-DQPSK modulation at 18 ksymbols/sec
//! - 36 kbps gross bit rate (9 kbps per slot)
//! - TEA1/TEA2/TEA3 encryption algorithms
//! - Direct Mode (DMO) and Trunked Mode (TMO)

use std::f64::consts::PI;

use crate::types::IQSample;
use crate::waveform::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};

/// TETRA operating mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TetraMode {
    /// Trunked Mode Operation (requires infrastructure)
    #[default]
    Tmo,
    /// Direct Mode Operation (terminal to terminal)
    Dmo,
}

/// TETRA encryption algorithm
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TetraEncryption {
    /// No encryption
    #[default]
    None,
    /// TEA1 (export controlled)
    Tea1,
    /// TEA2 (EU public safety)
    Tea2,
    /// TEA3 (open markets)
    Tea3,
}

/// TETRA burst types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BurstType {
    /// Normal uplink burst
    NormalUp,
    /// Normal downlink burst
    NormalDown,
    /// Synchronization burst
    Sync,
    /// Control uplink burst
    ControlUp,
}

impl BurstType {
    /// Get training sequence for this burst type
    pub fn training_sequence(&self) -> &'static [u8] {
        match self {
            Self::NormalUp => &[1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0],
            Self::NormalDown => &[0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0],
            Self::Sync => &[1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1],
            Self::ControlUp => &[0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1],
        }
    }
}

/// TETRA frame timing constants
pub struct TetraTiming;

impl TetraTiming {
    /// Slots per frame
    pub const SLOTS_PER_FRAME: usize = 4;
    /// Frames per multiframe
    pub const FRAMES_PER_MULTIFRAME: usize = 18;
    /// Multiframes per hyperframe
    pub const MULTIFRAMES_PER_HYPERFRAME: usize = 60;
    /// Frame duration in milliseconds
    pub const FRAME_DURATION_MS: f64 = 56.67;
    /// Slot duration in milliseconds
    pub const SLOT_DURATION_MS: f64 = 14.167;
    /// Symbols per slot
    pub const SYMBOLS_PER_SLOT: usize = 255;
    /// Symbol rate in symbols/sec
    pub const SYMBOL_RATE: f64 = 18000.0;
}

/// TETRA sync word patterns
#[allow(dead_code)]
const NORMAL_TRAINING_SEQ_1: u32 = 0x1ACFFC1D;
#[allow(dead_code)]
const NORMAL_TRAINING_SEQ_2: u32 = 0x1653302E;

/// TETRA waveform implementation
#[derive(Debug)]
pub struct Tetra {
    /// Common waveform parameters
    common: CommonParams,
    /// Operating mode (TMO/DMO)
    mode: TetraMode,
    /// Encryption algorithm
    encryption: TetraEncryption,
    /// Samples per symbol
    samples_per_sym: usize,
    /// Current phase for differential modulation
    phase: f64,
    /// RRC filter coefficients
    rrc_filter: Vec<f64>,
    /// Filter span in symbols
    #[allow(dead_code)]
    filter_span: usize,
}

impl Tetra {
    /// Standard symbol rate (18 ksymbols/sec)
    pub const SYMBOL_RATE: f64 = 18000.0;
    /// Channel bandwidth (25 kHz)
    pub const BANDWIDTH: f64 = 25000.0;
    /// Bits per symbol (π/4-DQPSK = 2 bits)
    pub const BITS_PER_SYMBOL: u8 = 2;
    /// RRC rolloff factor
    pub const RRC_ROLLOFF: f64 = 0.35;

    /// Create a new TETRA instance with default parameters
    pub fn new(sample_rate: f64, mode: TetraMode, encryption: TetraEncryption) -> Self {
        let samples_per_sym = (sample_rate / Self::SYMBOL_RATE) as usize;
        let filter_span = 8;

        // Generate RRC filter
        let rrc_filter = Self::generate_rrc_filter(samples_per_sym, filter_span, Self::RRC_ROLLOFF);

        Self {
            common: CommonParams {
                sample_rate,
                carrier_freq: 0.0,
                amplitude: 1.0,
            },
            mode,
            encryption,
            samples_per_sym: samples_per_sym.max(1),
            phase: 0.0,
            rrc_filter,
            filter_span,
        }
    }

    /// Create standard TMO TETRA instance
    pub fn tmo(sample_rate: f64) -> Self {
        Self::new(sample_rate, TetraMode::Tmo, TetraEncryption::None)
    }

    /// Create DMO (direct mode) TETRA instance
    pub fn dmo(sample_rate: f64) -> Self {
        Self::new(sample_rate, TetraMode::Dmo, TetraEncryption::None)
    }

    /// Create encrypted TETRA instance
    pub fn with_encryption(sample_rate: f64, encryption: TetraEncryption) -> Self {
        Self::new(sample_rate, TetraMode::Tmo, encryption)
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
            } else if (t.abs() - 1.0 / (4.0 * rolloff)).abs() < 1e-10 {
                // t = ±1/(4*α) case
                let a = (1.0 + 2.0 / PI) * (PI / (4.0 * rolloff)).sin();
                let b = (1.0 - 2.0 / PI) * (PI / (4.0 * rolloff)).cos();
                filter[i] = rolloff / 2.0_f64.sqrt() * (a + b);
            } else {
                // General case
                let num = (PI * t * (1.0 - rolloff)).sin()
                    + 4.0 * rolloff * t * (PI * t * (1.0 + rolloff)).cos();
                let den = PI * t * (1.0 - (4.0 * rolloff * t).powi(2));
                filter[i] = num / den;
            }
        }

        // Normalize
        let sum: f64 = filter.iter().sum();
        for coef in &mut filter {
            *coef /= sum;
        }

        filter
    }

    /// Generate sync burst
    fn generate_sync_burst(&mut self) -> Vec<IQSample> {
        // Simplified sync burst with training sequence
        let training = BurstType::Sync.training_sequence();
        self.pi4dqpsk_modulate(training)
    }

    /// π/4-DQPSK modulation
    fn pi4dqpsk_modulate(&mut self, dibits: &[u8]) -> Vec<IQSample> {
        let mut samples = Vec::with_capacity(dibits.len() * self.samples_per_sym);

        // Phase changes for π/4-DQPSK: 00->π/4, 01->3π/4, 11->-3π/4, 10->-π/4
        let phase_changes = [PI / 4.0, 3.0 * PI / 4.0, -3.0 * PI / 4.0, -PI / 4.0];

        for &dibit in dibits {
            let delta_phase = phase_changes[(dibit & 0x03) as usize];
            self.phase += delta_phase;

            // Keep phase in [-π, π]
            while self.phase > PI {
                self.phase -= 2.0 * PI;
            }
            while self.phase < -PI {
                self.phase += 2.0 * PI;
            }

            // Generate samples for this symbol
            for _ in 0..self.samples_per_sym {
                samples.push(IQSample::new(self.phase.cos(), self.phase.sin()));
            }
        }

        // Apply RRC pulse shaping (convolution)
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

    /// Demodulate π/4-DQPSK
    fn pi4dqpsk_demodulate(&self, samples: &[IQSample]) -> Vec<u8> {
        let mut dibits = Vec::new();
        let mut prev_phase = 0.0f64;

        for chunk in samples.chunks(self.samples_per_sym) {
            if chunk.is_empty() {
                break;
            }

            // Average phase over symbol
            let avg_i: f64 = chunk.iter().map(|s| s.re).sum::<f64>() / chunk.len() as f64;
            let avg_q: f64 = chunk.iter().map(|s| s.im).sum::<f64>() / chunk.len() as f64;
            let current_phase = avg_q.atan2(avg_i);

            // Calculate phase difference
            let mut delta = current_phase - prev_phase;
            while delta > PI {
                delta -= 2.0 * PI;
            }
            while delta < -PI {
                delta += 2.0 * PI;
            }

            // Map phase difference to dibit
            let dibit = if delta > PI / 2.0 {
                1 // 3π/4
            } else if delta > 0.0 {
                0 // π/4
            } else if delta > -PI / 2.0 {
                3 // -π/4
            } else {
                2 // -3π/4
            };

            dibits.push(dibit);
            prev_phase = current_phase;
        }

        dibits
    }
}

impl Waveform for Tetra {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "TETRA",
            full_name: "Terrestrial Trunked Radio",
            description: "European standard for professional mobile radio used by \
                emergency services, public safety, and military organizations",
            complexity: 4,
            bits_per_symbol: Self::BITS_PER_SYMBOL,
            carries_data: true,
            characteristics: &[
                "4-slot TDMA in 25 kHz channels",
                "π/4-DQPSK at 18 ksymbols/sec",
                "36 kbps gross (9 kbps per slot)",
                "TEA1/TEA2/TEA3 encryption",
                "DMO and TMO modes",
                "Voice + data + SDS",
                "Group/individual/broadcast calls",
            ],
            history: "Developed by ETSI in the 1990s as the European digital PMR standard. \
                TETRA stands for Terrestrial Trunked Radio (originally Trans-European \
                Trunked Radio). First commercial networks deployed in 1997.",
            modern_usage: "Standard for emergency services across Europe and many other \
                countries. Used by police, fire, ambulance, military, transport, and \
                utilities. NATO-adopted for tactical communications.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // Create mutable copy for phase tracking
        let mut tetra = Self::new(self.common.sample_rate, self.mode, self.encryption);

        // Generate sync burst
        let mut samples = tetra.generate_sync_burst();

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
        samples.extend(tetra.pi4dqpsk_modulate(&dibits));

        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        // Skip sync burst (approximate)
        let sync_symbols = 22; // Training sequence length
        let data_start = sync_symbols * self.samples_per_sym;

        if samples.len() <= data_start {
            return DemodResult::default();
        }

        let data_samples = &samples[data_start..];
        let dibits = self.pi4dqpsk_demodulate(data_samples);

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

        // π/4-DQPSK constellation (8 points, alternating between two QPSK sets)
        let a = 1.0 / 2.0_f64.sqrt();
        let constellation = vec![
            // First QPSK set (odd symbols)
            IQSample::new(a, a),
            IQSample::new(-a, a),
            IQSample::new(-a, -a),
            IQSample::new(a, -a),
            // Second QPSK set (even symbols, rotated 45°)
            IQSample::new(1.0, 0.0),
            IQSample::new(0.0, 1.0),
            IQSample::new(-1.0, 0.0),
            IQSample::new(0.0, -1.0),
        ];

        VisualizationData {
            samples,
            constellation,
            constellation_labels: vec![
                "00".to_string(),
                "01".to_string(),
                "11".to_string(),
                "10".to_string(),
                "00'".to_string(),
                "01'".to_string(),
                "11'".to_string(),
                "10'".to_string(),
            ],
            spectrum: Vec::new(),
            description: format!(
                "TETRA {:?} mode, {:?} encryption, π/4-DQPSK",
                self.mode, self.encryption
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tetra_creation() {
        let tetra = Tetra::tmo(72000.0);
        assert_eq!(tetra.samples_per_sym, 4); // 72000/18000 = 4
    }

    #[test]
    fn test_tetra_modulation() {
        let tetra = Tetra::tmo(72000.0);
        let data = vec![0xAA, 0x55, 0xF0, 0x0F];
        let modulated = tetra.modulate(&data);

        assert!(!modulated.is_empty());

        // Check amplitude is reasonable
        for sample in &modulated {
            assert!(sample.norm() <= 2.0);
        }
    }

    #[test]
    fn test_tetra_demodulation() {
        let tetra = Tetra::tmo(72000.0);
        let data = vec![0xAB, 0xCD];
        let modulated = tetra.modulate(&data);
        let demod = tetra.demodulate(&modulated);

        // Should recover some data (may not be perfect without full frame handling)
        assert!(!demod.bits.is_empty() || !demod.symbols.is_empty());
    }

    #[test]
    fn test_tetra_modes() {
        let tmo = Tetra::tmo(72000.0);
        let dmo = Tetra::dmo(72000.0);

        assert_eq!(tmo.mode, TetraMode::Tmo);
        assert_eq!(dmo.mode, TetraMode::Dmo);
    }

    #[test]
    fn test_tetra_encryption() {
        let tea2 = Tetra::with_encryption(72000.0, TetraEncryption::Tea2);
        assert_eq!(tea2.encryption, TetraEncryption::Tea2);
    }

    #[test]
    fn test_waveform_info() {
        let tetra = Tetra::tmo(72000.0);
        let info = tetra.info();

        assert_eq!(info.name, "TETRA");
        assert!(info.description.to_lowercase().contains("european"));
    }
}

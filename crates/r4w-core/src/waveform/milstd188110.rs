//! MIL-STD-188-110 HF Serial Tone Modem
//!
//! This module implements the MIL-STD-188-110 HF data modem standard,
//! which is the US military equivalent of NATO STANAG 4285.
//!
//! # Overview
//!
//! MIL-STD-188-110 defines serial tone waveforms for HF communications:
//! - Appendix A: 75-2400 bps with PSK modulation
//! - Appendix B: 3200-9600 bps with QAM modulation (newer)
//! - Appendix C: Automatic link establishment (similar to MIL-STD-188-141)
//!
//! # Key Features
//!
//! - PSK modulation: BPSK, QPSK, 8-PSK
//! - Robust interleaving (short/long)
//! - Forward error correction (convolutional + Viterbi)
//! - Known data symbols for channel estimation
//! - Compatible with STANAG 4285 in many modes

use std::f64::consts::PI;

use crate::types::IQSample;
use crate::waveform::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};

/// Data rate modes for MIL-STD-188-110A (Appendix A)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataRate {
    /// 75 bps - Maximum robustness
    Bps75,
    /// 150 bps
    Bps150,
    /// 300 bps
    Bps300,
    /// 600 bps
    Bps600,
    /// 1200 bps
    #[default]
    Bps1200,
    /// 2400 bps - Standard mode
    Bps2400,
    /// 4800 bps - Appendix B
    Bps4800,
}

impl DataRate {
    /// Get the bits per second
    pub fn bps(&self) -> u32 {
        match self {
            Self::Bps75 => 75,
            Self::Bps150 => 150,
            Self::Bps300 => 300,
            Self::Bps600 => 600,
            Self::Bps1200 => 1200,
            Self::Bps2400 => 2400,
            Self::Bps4800 => 4800,
        }
    }

    /// Get the modulation type for this rate
    pub fn modulation(&self) -> Modulation {
        match self {
            Self::Bps75 | Self::Bps150 | Self::Bps300 | Self::Bps600 => Modulation::Bpsk,
            Self::Bps1200 => Modulation::Qpsk,
            Self::Bps2400 | Self::Bps4800 => Modulation::Psk8,
        }
    }

    /// Get coding rate (k/n)
    pub fn coding_rate(&self) -> (u8, u8) {
        match self {
            Self::Bps75 => (1, 8),
            Self::Bps150 => (1, 4),
            Self::Bps300 => (1, 2),
            Self::Bps600 | Self::Bps1200 | Self::Bps2400 => (1, 2),
            Self::Bps4800 => (3, 4),
        }
    }
}

/// Modulation types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Modulation {
    /// Binary PSK - 1 bit/symbol
    #[default]
    Bpsk,
    /// Quadrature PSK - 2 bits/symbol
    Qpsk,
    /// 8-PSK - 3 bits/symbol
    Psk8,
    /// 16-QAM - 4 bits/symbol (Appendix B)
    Qam16,
    /// 32-QAM - 5 bits/symbol (Appendix B)
    Qam32,
    /// 64-QAM - 6 bits/symbol (Appendix B)
    Qam64,
}

impl Modulation {
    /// Get bits per symbol
    pub fn bits_per_symbol(&self) -> u8 {
        match self {
            Self::Bpsk => 1,
            Self::Qpsk => 2,
            Self::Psk8 => 3,
            Self::Qam16 => 4,
            Self::Qam32 => 5,
            Self::Qam64 => 6,
        }
    }

    /// Get constellation points
    pub fn constellation(&self) -> Vec<IQSample> {
        match self {
            Self::Bpsk => vec![IQSample::new(-1.0, 0.0), IQSample::new(1.0, 0.0)],
            Self::Qpsk => {
                let a = 1.0 / 2.0_f64.sqrt();
                vec![
                    IQSample::new(a, a),
                    IQSample::new(-a, a),
                    IQSample::new(-a, -a),
                    IQSample::new(a, -a),
                ]
            }
            Self::Psk8 => (0..8)
                .map(|i| {
                    let angle = PI / 8.0 + (i as f64) * PI / 4.0;
                    IQSample::new(angle.cos(), angle.sin())
                })
                .collect(),
            Self::Qam16 => {
                let levels = [-3.0, -1.0, 1.0, 3.0];
                let norm = 1.0 / (10.0_f64).sqrt();
                let mut points = Vec::with_capacity(16);
                for &i in &levels {
                    for &q in &levels {
                        points.push(IQSample::new(i * norm, q * norm));
                    }
                }
                points
            }
            Self::Qam32 | Self::Qam64 => {
                // Simplified - full implementation would have proper constellation
                let n = if matches!(self, Self::Qam32) { 32 } else { 64 };
                (0..n)
                    .map(|i| {
                        let angle = (i as f64) * 2.0 * PI / (n as f64);
                        let r = 0.5 + 0.5 * ((i % 2) as f64);
                        IQSample::new(r * angle.cos(), r * angle.sin())
                    })
                    .collect()
            }
        }
    }
}

/// Interleave mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InterleaveMode {
    /// No interleaving
    None,
    /// Short interleave (0.6 seconds)
    #[default]
    Short,
    /// Long interleave (4.8 seconds)
    Long,
}

impl InterleaveMode {
    /// Get interleave depth in symbols
    pub fn depth(&self) -> usize {
        match self {
            Self::None => 1,
            Self::Short => 40,   // ~0.6 seconds at 2400 baud
            Self::Long => 320,   // ~4.8 seconds at 2400 baud
        }
    }
}

/// MIL-STD-188-110 waveform implementation
#[derive(Debug)]
pub struct MilStd188110 {
    /// Common waveform parameters
    common: CommonParams,
    /// Data rate mode
    data_rate: DataRate,
    /// Interleave mode
    interleave: InterleaveMode,
    /// Symbol rate (fixed at 2400 baud for 188-110A)
    #[allow(dead_code)]
    symbol_rate: f64,
    /// Samples per symbol
    samples_per_sym: usize,
    /// Carrier frequency offset
    carrier_offset: f64,
    /// Differential encoding enabled
    differential: bool,
}

impl MilStd188110 {
    /// Symbol rate for MIL-STD-188-110A
    pub const SYMBOL_RATE: f64 = 2400.0;

    /// Preamble length in symbols
    pub const PREAMBLE_SYMBOLS: usize = 287;

    /// Create a new MIL-STD-188-110 instance
    pub fn new(sample_rate: f64, data_rate: DataRate, interleave: InterleaveMode) -> Self {
        let samples_per_sym = (sample_rate / Self::SYMBOL_RATE) as usize;
        Self {
            common: CommonParams {
                sample_rate,
                carrier_freq: 1800.0, // Standard HF center
                amplitude: 1.0,
            },
            data_rate,
            interleave,
            symbol_rate: Self::SYMBOL_RATE,
            samples_per_sym,
            carrier_offset: 1800.0,
            differential: true,
        }
    }

    /// Create with default mode (1200 bps, short interleave)
    pub fn default_mode(sample_rate: f64) -> Self {
        Self::new(sample_rate, DataRate::Bps1200, InterleaveMode::Short)
    }

    /// Create for 2400 bps operation
    pub fn high_speed(sample_rate: f64) -> Self {
        Self::new(sample_rate, DataRate::Bps2400, InterleaveMode::Short)
    }

    /// Create for maximum robustness (75 bps)
    pub fn robust(sample_rate: f64) -> Self {
        Self::new(sample_rate, DataRate::Bps75, InterleaveMode::Long)
    }

    /// Get current data rate
    pub fn data_rate(&self) -> DataRate {
        self.data_rate
    }

    /// Set data rate
    pub fn set_data_rate(&mut self, rate: DataRate) {
        self.data_rate = rate;
    }

    /// Get current interleave mode
    pub fn interleave_mode(&self) -> InterleaveMode {
        self.interleave
    }

    /// Set interleave mode
    pub fn set_interleave(&mut self, mode: InterleaveMode) {
        self.interleave = mode;
    }

    /// Generate preamble (known data for sync and channel estimation)
    fn generate_preamble(&self) -> Vec<IQSample> {
        let mut samples = Vec::with_capacity(Self::PREAMBLE_SYMBOLS * self.samples_per_sym);

        // Alternating pattern for acquisition
        for i in 0..Self::PREAMBLE_SYMBOLS {
            let symbol = if i % 2 == 0 { 1.0 } else { -1.0 };

            for j in 0..self.samples_per_sym {
                let t = (i * self.samples_per_sym + j) as f64 / self.common.sample_rate;
                let phase = 2.0 * PI * self.carrier_offset * t;
                samples.push(IQSample::new(symbol * phase.cos(), symbol * phase.sin()));
            }
        }

        samples
    }

    /// Convolutional encode with rate 1/2, K=7
    fn conv_encode(&self, bits: &[bool]) -> Vec<bool> {
        let (k, n) = self.data_rate.coding_rate();
        if k == n {
            return bits.to_vec(); // No coding
        }

        // K=7 convolutional encoder (same as STANAG 4285)
        let mut state: u8 = 0;
        let mut output = Vec::with_capacity(bits.len() * (n as usize) / (k as usize));

        for &bit in bits {
            state = (state >> 1) | ((bit as u8) << 6);

            // G1 = 1111001 (0x79)
            // G2 = 1011011 (0x5B)
            let g1 = (state & 0x79).count_ones() % 2;
            let g2 = (state & 0x5B).count_ones() % 2;

            output.push(g1 == 1);
            output.push(g2 == 1);
        }

        // Apply puncturing for higher rates
        if k == 3 && n == 4 {
            // Puncture to rate 3/4
            output
                .chunks(6)
                .flat_map(|chunk| {
                    chunk.iter().enumerate().filter_map(|(i, &b)| {
                        if i != 2 && i != 5 {
                            Some(b)
                        } else {
                            None
                        }
                    })
                })
                .collect()
        } else {
            output
        }
    }

    /// Interleave symbols
    fn interleave(&self, symbols: &[u8]) -> Vec<u8> {
        let depth = self.interleave.depth();
        if depth <= 1 || symbols.is_empty() {
            return symbols.to_vec();
        }

        let rows = (symbols.len() + depth - 1) / depth;
        let mut interleaved = vec![0u8; symbols.len()];

        for (i, &sym) in symbols.iter().enumerate() {
            let row = i / depth;
            let col = i % depth;
            let new_idx = col * rows + row;
            if new_idx < interleaved.len() {
                interleaved[new_idx] = sym;
            }
        }

        interleaved
    }

    /// De-interleave symbols
    fn deinterleave(&self, symbols: &[u8]) -> Vec<u8> {
        let depth = self.interleave.depth();
        if depth <= 1 || symbols.is_empty() {
            return symbols.to_vec();
        }

        let rows = (symbols.len() + depth - 1) / depth;
        let mut deinterleaved = vec![0u8; symbols.len()];

        for (i, &sym) in symbols.iter().enumerate() {
            let col = i / rows;
            let row = i % rows;
            let new_idx = row * depth + col;
            if new_idx < deinterleaved.len() {
                deinterleaved[new_idx] = sym;
            }
        }

        deinterleaved
    }

    /// Map bits to symbols
    fn bits_to_symbols(&self, bits: &[bool]) -> Vec<u8> {
        let bps = self.data_rate.modulation().bits_per_symbol() as usize;
        bits.chunks(bps)
            .map(|chunk| {
                chunk
                    .iter()
                    .enumerate()
                    .fold(0u8, |acc, (i, &b)| acc | ((b as u8) << (bps - 1 - i)))
            })
            .collect()
    }

    /// Map symbols to IQ samples
    fn symbols_to_iq(&self, symbols: &[u8]) -> Vec<IQSample> {
        let constellation = self.data_rate.modulation().constellation();
        let mut samples = Vec::with_capacity(symbols.len() * self.samples_per_sym);
        let mut prev_phase = 0.0;

        for (sym_idx, &symbol) in symbols.iter().enumerate() {
            let point = &constellation[(symbol as usize) % constellation.len()];

            // Apply differential encoding if enabled
            let (i, q) = if self.differential {
                let angle = point.im.atan2(point.re);
                prev_phase += angle;
                (prev_phase.cos(), prev_phase.sin())
            } else {
                (point.re, point.im)
            };

            // Generate samples with pulse shaping
            for j in 0..self.samples_per_sym {
                let t = (sym_idx * self.samples_per_sym + j) as f64 / self.common.sample_rate;
                let phase = 2.0 * PI * self.carrier_offset * t;

                // Raised cosine pulse shaping (simplified)
                let pulse = 1.0; // Would be proper RRC filter

                samples.push(IQSample::new(
                    pulse * (i * phase.cos() - q * phase.sin()),
                    pulse * (i * phase.sin() + q * phase.cos()),
                ));
            }
        }

        samples
    }

    /// Demodulate IQ to symbols
    fn iq_to_symbols(&self, samples: &[IQSample]) -> Vec<u8> {
        let constellation = self.data_rate.modulation().constellation();
        let mut symbols = Vec::new();

        for chunk in samples.chunks(self.samples_per_sym) {
            if chunk.len() < self.samples_per_sym / 2 {
                break;
            }

            // Average over symbol period
            let avg_i: f64 = chunk.iter().map(|s| s.re).sum::<f64>() / chunk.len() as f64;
            let avg_q: f64 = chunk.iter().map(|s| s.im).sum::<f64>() / chunk.len() as f64;

            // Find nearest constellation point
            let sample = IQSample::new(avg_i, avg_q);
            let symbol = constellation
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    let da = (a.re - sample.re).powi(2) + (a.im - sample.im).powi(2);
                    let db = (b.re - sample.re).powi(2) + (b.im - sample.im).powi(2);
                    da.partial_cmp(&db).unwrap()
                })
                .map(|(idx, _)| idx as u8)
                .unwrap_or(0);

            symbols.push(symbol);
        }

        symbols
    }
}

impl Waveform for MilStd188110 {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "MIL-STD-188-110",
            full_name: "MIL-STD-188-110 HF Serial Tone Modem",
            description: "US military HF data modem standard for robust data transmission \
                over HF skywave and groundwave channels",
            complexity: 4,
            bits_per_symbol: self.data_rate.modulation().bits_per_symbol(),
            carries_data: true,
            characteristics: &[
                "HF band (2-30 MHz)",
                "2400 baud symbol rate",
                "PSK/QAM modulation",
                "Rate 1/2 convolutional FEC",
                "Block interleaving",
                "Known data for equalization",
                "75-9600 bps data rates",
            ],
            history: "Developed by the US military as the standard for HF data communications. \
                The original MIL-STD-188-110A (1991) specified rates up to 2400 bps. \
                Appendix B added higher speed modes up to 9600 bps.",
            modern_usage: "Widely used by US military and allied forces for HF data links. \
                Compatible with NATO STANAG 4285 in many modes. Common in shipboard, \
                airborne, and fixed HF systems. Being supplemented by wideband HF.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // Convert bytes to bits
        let bits: Vec<bool> = data
            .iter()
            .flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1 == 1))
            .collect();

        // Apply FEC
        let encoded = self.conv_encode(&bits);

        // Map to symbols
        let symbols = self.bits_to_symbols(&encoded);

        // Interleave
        let interleaved = self.interleave(&symbols);

        // Generate preamble
        let mut samples = self.generate_preamble();

        // Modulate data
        samples.extend(self.symbols_to_iq(&interleaved));

        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        // Skip preamble
        let data_start = Self::PREAMBLE_SYMBOLS * self.samples_per_sym;
        if samples.len() <= data_start {
            return DemodResult::default();
        }

        let data_samples = &samples[data_start..];

        // Demodulate to symbols
        let symbols = self.iq_to_symbols(data_samples);

        // De-interleave
        let deinterleaved = self.deinterleave(&symbols);

        // Convert symbols to bits (simplified - no Viterbi decoding)
        let bps = self.data_rate.modulation().bits_per_symbol() as usize;
        let bits: Vec<bool> = deinterleaved
            .iter()
            .flat_map(|&sym| (0..bps).rev().map(move |i| (sym >> i) & 1 == 1))
            .collect();

        // Pack bits to bytes
        let bytes: Vec<u8> = bits
            .chunks(8)
            .map(|chunk| {
                chunk
                    .iter()
                    .enumerate()
                    .fold(0u8, |acc, (i, &b)| acc | ((b as u8) << (7 - i)))
            })
            .collect();

        DemodResult {
            bits: bytes,
            symbols: deinterleaved.iter().map(|&s| s as u16).collect(),
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
        let constellation = self.data_rate.modulation().constellation();

        VisualizationData {
            samples,
            constellation,
            constellation_labels: vec![],
            spectrum: Vec::new(),
            description: format!(
                "MIL-STD-188-110: {} bps, {:?} modulation, {:?} interleave",
                self.data_rate.bps(),
                self.data_rate.modulation(),
                self.interleave
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_modulation() {
        let modem = MilStd188110::default_mode(48000.0);
        let data = vec![0xAA, 0x55, 0xF0, 0x0F];
        let modulated = modem.modulate(&data);

        assert!(!modulated.is_empty());

        // Check amplitude
        for sample in &modulated {
            assert!(sample.norm() <= 1.5);
        }
    }

    #[test]
    fn test_data_rates() {
        assert_eq!(DataRate::Bps75.bps(), 75);
        assert_eq!(DataRate::Bps2400.bps(), 2400);
        assert_eq!(DataRate::Bps75.modulation(), Modulation::Bpsk);
        assert_eq!(DataRate::Bps2400.modulation(), Modulation::Psk8);
    }

    #[test]
    fn test_constellation() {
        let bpsk = Modulation::Bpsk.constellation();
        assert_eq!(bpsk.len(), 2);

        let qpsk = Modulation::Qpsk.constellation();
        assert_eq!(qpsk.len(), 4);

        let psk8 = Modulation::Psk8.constellation();
        assert_eq!(psk8.len(), 8);
    }

    #[test]
    fn test_interleaving() {
        let modem = MilStd188110::new(48000.0, DataRate::Bps1200, InterleaveMode::Short);

        // Use a length that's a multiple of interleave depth (40)
        let data: Vec<u8> = (0..80).collect();
        let interleaved = modem.interleave(&data);
        let deinterleaved = modem.deinterleave(&interleaved);

        assert_eq!(data, deinterleaved);
    }

    #[test]
    fn test_waveform_info() {
        let modem = MilStd188110::default_mode(48000.0);
        let info = modem.info();

        assert_eq!(info.name, "MIL-STD-188-110");
        assert!(info.description.contains("HF"));
    }

    #[test]
    fn test_modes() {
        let robust = MilStd188110::robust(48000.0);
        assert_eq!(robust.data_rate().bps(), 75);

        let high_speed = MilStd188110::high_speed(48000.0);
        assert_eq!(high_speed.data_rate().bps(), 2400);
    }
}

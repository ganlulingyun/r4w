//! STANAG 4285 - NATO HF Modem Waveform
//!
//! STANAG 4285 is a publicly documented NATO standard for HF (High Frequency)
//! data modems. It's widely used for military and government HF communications.
//!
//! ## Key Features
//!
//! - **Symbol Rate**: 2400 baud
//! - **Modulation**: PSK (BPSK, QPSK, 8-PSK)
//! - **Data Rates**: 75 to 3600 bps
//! - **Channel Estimation**: 16-tone probe sequences
//! - **Error Correction**: Rate-1/2 convolutional coding
//! - **Interleaving**: Block interleaving for burst protection
//!
//! ## Frame Structure
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────┐
//! │                    STANAG 4285 Frame                    │
//! ├────────┬──────────┬──────────┬──────────┬───────────────┤
//! │ Sync   │ Probe    │ Data     │ Probe    │ Data    ...   │
//! │ 80 sym │ 16 sym   │ 32 sym   │ 16 sym   │ 32 sym        │
//! └────────┴──────────┴──────────┴──────────┴───────────────┘
//! ```

use std::f64::consts::PI;
use num_complex::Complex64;

use crate::types::IQSample;
use super::{
    CommonParams, DemodResult, DemodulationStep, ModulationStage,
    VisualizationData, Waveform, WaveformInfo,
};

/// STANAG 4285 operating modes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stanag4285Mode {
    /// 75 bps, long interleaving (4.8s)
    Mode75Long,
    /// 75 bps, short interleaving (0.6s)
    Mode75Short,
    /// 150 bps, long interleaving
    Mode150Long,
    /// 150 bps, short interleaving
    Mode150Short,
    /// 300 bps, long interleaving
    Mode300Long,
    /// 300 bps, short interleaving
    Mode300Short,
    /// 600 bps, long interleaving
    Mode600Long,
    /// 600 bps, short interleaving
    Mode600Short,
    /// 1200 bps, long interleaving
    Mode1200Long,
    /// 1200 bps, short interleaving
    Mode1200Short,
    /// 2400 bps, long interleaving
    Mode2400Long,
    /// 2400 bps, short interleaving
    Mode2400Short,
    /// 3600 bps, long interleaving (8-PSK)
    Mode3600Long,
    /// 3600 bps, short interleaving (8-PSK)
    Mode3600Short,
}

impl Stanag4285Mode {
    /// Get data rate in bps
    pub fn data_rate(&self) -> u32 {
        match self {
            Self::Mode75Long | Self::Mode75Short => 75,
            Self::Mode150Long | Self::Mode150Short => 150,
            Self::Mode300Long | Self::Mode300Short => 300,
            Self::Mode600Long | Self::Mode600Short => 600,
            Self::Mode1200Long | Self::Mode1200Short => 1200,
            Self::Mode2400Long | Self::Mode2400Short => 2400,
            Self::Mode3600Long | Self::Mode3600Short => 3600,
        }
    }

    /// Get modulation type
    pub fn modulation(&self) -> PskType {
        match self {
            Self::Mode75Long | Self::Mode75Short |
            Self::Mode150Long | Self::Mode150Short |
            Self::Mode300Long | Self::Mode300Short |
            Self::Mode600Long | Self::Mode600Short => PskType::Bpsk,
            Self::Mode1200Long | Self::Mode1200Short |
            Self::Mode2400Long | Self::Mode2400Short => PskType::Qpsk,
            Self::Mode3600Long | Self::Mode3600Short => PskType::Psk8,
        }
    }

    /// Is this a long interleaving mode?
    pub fn is_long_interleave(&self) -> bool {
        matches!(
            self,
            Self::Mode75Long | Self::Mode150Long | Self::Mode300Long |
            Self::Mode600Long | Self::Mode1200Long | Self::Mode2400Long |
            Self::Mode3600Long
        )
    }

    /// Get interleave depth in symbols
    pub fn interleave_depth(&self) -> usize {
        if self.is_long_interleave() {
            match self.data_rate() {
                75 => 11520,  // 4.8 seconds
                150 => 5760,
                300 => 2880,
                600 => 1440,
                1200 => 720,
                2400 => 360,
                3600 => 240,
                _ => 360,
            }
        } else {
            match self.data_rate() {
                75 => 1440,   // 0.6 seconds
                150 => 720,
                300 => 360,
                600 => 180,
                1200 => 90,
                2400 => 45,
                3600 => 30,
                _ => 45,
            }
        }
    }

    /// Get FEC code rate (input bits / output bits)
    pub fn code_rate(&self) -> (u32, u32) {
        // All modes use rate-1/2 convolutional code
        // except 3600 bps which uses rate-3/4
        match self {
            Self::Mode3600Long | Self::Mode3600Short => (3, 4),
            _ => (1, 2),
        }
    }
}

/// PSK modulation type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PskType {
    Bpsk,
    Qpsk,
    Psk8,
}

impl PskType {
    /// Bits per symbol
    pub fn bits_per_symbol(&self) -> u8 {
        match self {
            Self::Bpsk => 1,
            Self::Qpsk => 2,
            Self::Psk8 => 3,
        }
    }

    /// Get constellation point for symbol
    pub fn symbol_to_iq(&self, symbol: u8) -> IQSample {
        match self {
            Self::Bpsk => {
                if symbol == 0 {
                    Complex64::new(1.0, 0.0)
                } else {
                    Complex64::new(-1.0, 0.0)
                }
            }
            Self::Qpsk => {
                // Gray coded: 00->0°, 01->90°, 11->180°, 10->270°
                let phase = match symbol & 0x03 {
                    0b00 => 0.0,
                    0b01 => PI / 2.0,
                    0b11 => PI,
                    0b10 => 3.0 * PI / 2.0,
                    _ => 0.0,
                };
                let scale = 1.0 / 2.0_f64.sqrt();
                Complex64::new(phase.cos() * scale, phase.sin() * scale)
            }
            Self::Psk8 => {
                // Gray coded 8-PSK
                let phase = match symbol & 0x07 {
                    0b000 => 0.0,
                    0b001 => PI / 4.0,
                    0b011 => PI / 2.0,
                    0b010 => 3.0 * PI / 4.0,
                    0b110 => PI,
                    0b111 => 5.0 * PI / 4.0,
                    0b101 => 3.0 * PI / 2.0,
                    0b100 => 7.0 * PI / 4.0,
                    _ => 0.0,
                };
                Complex64::new(phase.cos(), phase.sin())
            }
        }
    }

    /// Detect symbol from IQ sample
    pub fn iq_to_symbol(&self, sample: IQSample) -> u8 {
        let phase = sample.im.atan2(sample.re);
        let normalized = if phase < 0.0 { phase + 2.0 * PI } else { phase };

        match self {
            Self::Bpsk => {
                if normalized < PI / 2.0 || normalized >= 3.0 * PI / 2.0 {
                    0
                } else {
                    1
                }
            }
            Self::Qpsk => {
                let sector = (normalized / (PI / 2.0)).floor() as u8;
                // Reverse Gray coding
                match sector {
                    0 => 0b00,
                    1 => 0b01,
                    2 => 0b11,
                    3 => 0b10,
                    _ => 0,
                }
            }
            Self::Psk8 => {
                // Add small offset to center decision regions on constellation points
                let offset = PI / 8.0; // Half a sector
                let adjusted = (normalized + offset) % (2.0 * PI);
                let sector = (adjusted / (PI / 4.0)).floor() as u8 % 8;
                // Reverse Gray coding
                match sector {
                    0 => 0b000,
                    1 => 0b001,
                    2 => 0b011,
                    3 => 0b010,
                    4 => 0b110,
                    5 => 0b111,
                    6 => 0b101,
                    7 => 0b100,
                    _ => 0,
                }
            }
        }
    }

    /// Get constellation points for visualization
    pub fn constellation(&self) -> Vec<IQSample> {
        let n = 1 << self.bits_per_symbol();
        (0..n).map(|i| self.symbol_to_iq(i as u8)).collect()
    }
}

/// Known probe sequence for channel estimation
/// These are the 16-symbol DPSK probe patterns
const PROBE_SEQUENCE: [u8; 16] = [
    0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0
];

/// Synchronization preamble (80 symbols of alternating pattern)
fn generate_sync_preamble() -> Vec<IQSample> {
    let mut samples = Vec::with_capacity(80);
    for i in 0..80 {
        let phase = if i % 2 == 0 { 0.0 } else { PI };
        samples.push(Complex64::new(phase.cos(), phase.sin()));
    }
    samples
}

/// Generate probe symbols for channel estimation
fn generate_probe_symbols() -> Vec<IQSample> {
    PROBE_SEQUENCE.iter()
        .map(|&bit| {
            let phase = if bit == 0 { 0.0 } else { PI };
            Complex64::new(phase.cos(), phase.sin())
        })
        .collect()
}

/// STANAG 4285 HF Modem
#[derive(Debug)]
pub struct Stanag4285 {
    common: CommonParams,
    mode: Stanag4285Mode,
    /// Symbol rate (always 2400)
    #[allow(dead_code)]
    symbol_rate: f64,
    /// Samples per symbol
    samples_per_symbol: usize,
    /// Convolutional encoder state
    encoder_state: u8,
    /// Interleave buffer
    interleave_buffer: Vec<u8>,
    /// Scrambler state
    scrambler_state: u16,
}

impl Stanag4285 {
    /// Create new STANAG 4285 modem
    pub fn new(sample_rate: f64, mode: Stanag4285Mode) -> Self {
        let symbol_rate = 2400.0;
        let samples_per_symbol = (sample_rate / symbol_rate).round() as usize;

        Self {
            common: CommonParams {
                sample_rate,
                carrier_freq: 1800.0, // Standard HF center frequency
                amplitude: 1.0,
            },
            mode,
            symbol_rate,
            samples_per_symbol,
            encoder_state: 0,
            interleave_buffer: Vec::new(),
            scrambler_state: 0x01FF,
        }
    }

    /// Create with default 2400 bps mode
    pub fn default_mode(sample_rate: f64) -> Self {
        Self::new(sample_rate, Stanag4285Mode::Mode2400Short)
    }

    /// Set operating mode
    pub fn set_mode(&mut self, mode: Stanag4285Mode) {
        self.mode = mode;
    }

    /// Get current mode
    pub fn mode(&self) -> Stanag4285Mode {
        self.mode
    }

    /// Reset encoder state
    pub fn reset(&mut self) {
        self.encoder_state = 0;
        self.interleave_buffer.clear();
        self.scrambler_state = 0x01FF;
    }

    /// Convolutional encoder (K=7, rate 1/2)
    /// Polynomials: G1 = 1111001 (0x79), G2 = 1011011 (0x5B)
    fn encode_convolutional(&mut self, bits: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity(bits.len() * 2);

        for &bit in bits {
            // Shift in new bit
            self.encoder_state = ((self.encoder_state << 1) | (bit & 1)) & 0x7F;

            // Calculate parity bits
            let g1 = (self.encoder_state & 0x79).count_ones() & 1;
            let g2 = (self.encoder_state & 0x5B).count_ones() & 1;

            output.push(g1 as u8);
            output.push(g2 as u8);
        }

        output
    }

    /// Scramble data with LFSR
    fn scramble(&mut self, bits: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity(bits.len());

        for &bit in bits {
            // LFSR: x^9 + x^4 + 1
            let feedback = ((self.scrambler_state >> 8) ^ (self.scrambler_state >> 3)) & 1;
            self.scrambler_state = ((self.scrambler_state << 1) | feedback as u16) & 0x1FF;

            output.push(bit ^ (self.scrambler_state & 1) as u8);
        }

        output
    }

    /// Block interleaver
    fn interleave(&self, bits: &[u8]) -> Vec<u8> {
        let depth = self.mode.interleave_depth();
        if bits.len() < depth {
            return bits.to_vec();
        }

        let rows = depth;
        let cols = bits.len() / rows;
        let mut output = vec![0u8; bits.len()];

        for row in 0..rows {
            for col in 0..cols {
                let in_idx = row * cols + col;
                let out_idx = col * rows + row;
                if in_idx < bits.len() && out_idx < output.len() {
                    output[out_idx] = bits[in_idx];
                }
            }
        }

        output
    }

    /// Convert bits to symbols
    fn bits_to_symbols(&self, bits: &[u8]) -> Vec<u8> {
        let bps = self.mode.modulation().bits_per_symbol() as usize;
        bits.chunks(bps)
            .map(|chunk| {
                chunk.iter().enumerate().fold(0u8, |acc, (i, &bit)| {
                    acc | ((bit & 1) << (bps - 1 - i))
                })
            })
            .collect()
    }

    /// Modulate symbols to I/Q samples
    fn modulate_symbols(&self, symbols: &[u8]) -> Vec<IQSample> {
        let psk = self.mode.modulation();
        let mut samples = Vec::with_capacity(symbols.len() * self.samples_per_symbol);

        for &symbol in symbols {
            let iq = psk.symbol_to_iq(symbol);

            // Generate samples with pulse shaping (raised cosine)
            for i in 0..self.samples_per_symbol {
                let t = i as f64 / self.samples_per_symbol as f64;
                // Simple rectangular pulse (could add RRC shaping)
                let _ = t; // Would use for pulse shaping
                samples.push(iq);
            }
        }

        samples
    }

    /// Build complete frame with sync, probes, and data
    fn build_frame(&mut self, data: &[u8]) -> Vec<IQSample> {
        let mut frame = Vec::new();

        // 1. Sync preamble (80 symbols)
        let sync = generate_sync_preamble();
        for sample in &sync {
            for _ in 0..self.samples_per_symbol {
                frame.push(*sample);
            }
        }

        // 2. Encode data
        let coded = self.encode_convolutional(data);
        let scrambled = self.scramble(&coded);
        let interleaved = self.interleave(&scrambled);
        let symbols = self.bits_to_symbols(&interleaved);

        // 3. Insert data with periodic probes
        let data_block_size = 32;
        let probe = generate_probe_symbols();

        for (i, chunk) in symbols.chunks(data_block_size).enumerate() {
            // Probe before each data block (except first which has sync)
            if i > 0 {
                for sample in &probe {
                    for _ in 0..self.samples_per_symbol {
                        frame.push(*sample);
                    }
                }
            }

            // Data block
            let data_samples = self.modulate_symbols(chunk);
            frame.extend(data_samples);
        }

        // Final probe
        for sample in &probe {
            for _ in 0..self.samples_per_symbol {
                frame.push(*sample);
            }
        }

        frame
    }

    /// Demodulate received samples
    fn demodulate_samples(&self, samples: &[IQSample]) -> Vec<u8> {
        let psk = self.mode.modulation();
        let mut symbols = Vec::new();

        // Simple symbol-by-symbol detection
        // A real implementation would do:
        // 1. Carrier recovery
        // 2. Symbol timing recovery
        // 3. Channel estimation using probes
        // 4. Equalization

        for chunk in samples.chunks(self.samples_per_symbol) {
            if chunk.is_empty() {
                continue;
            }

            // Average samples in symbol
            let sum: IQSample = chunk.iter().fold(Complex64::new(0.0, 0.0), |a, &b| a + b);
            let avg = sum / chunk.len() as f64;

            symbols.push(psk.iq_to_symbol(avg));
        }

        symbols
    }

    /// Convert symbols back to bits
    fn symbols_to_bits(&self, symbols: &[u8]) -> Vec<u8> {
        let bps = self.mode.modulation().bits_per_symbol() as usize;
        let mut bits = Vec::with_capacity(symbols.len() * bps);

        for &symbol in symbols {
            for i in (0..bps).rev() {
                bits.push((symbol >> i) & 1);
            }
        }

        bits
    }

    /// De-interleave
    fn deinterleave(&self, bits: &[u8]) -> Vec<u8> {
        let depth = self.mode.interleave_depth();
        if bits.len() < depth {
            return bits.to_vec();
        }

        let cols = depth;
        let rows = bits.len() / cols;
        let mut output = vec![0u8; bits.len()];

        for col in 0..cols {
            for row in 0..rows {
                let in_idx = col * rows + row;
                let out_idx = row * cols + col;
                if in_idx < bits.len() && out_idx < output.len() {
                    output[out_idx] = bits[in_idx];
                }
            }
        }

        output
    }

    /// Descramble
    fn descramble(&mut self, bits: &[u8]) -> Vec<u8> {
        // Scrambler is self-synchronizing, same operation as scramble
        self.scrambler_state = 0x01FF;
        self.scramble(bits)
    }

    /// Viterbi decoder (simplified)
    fn decode_viterbi(&self, bits: &[u8]) -> Vec<u8> {
        // Simplified hard-decision Viterbi decoder
        // A real implementation would use soft decisions

        if bits.len() < 2 {
            return vec![];
        }

        let mut output = Vec::with_capacity(bits.len() / 2);

        // Very simplified: just take every other bit pair and vote
        for chunk in bits.chunks(2) {
            if chunk.len() == 2 {
                // Simple majority decode (not real Viterbi)
                let g1 = chunk[0];
                let g2 = chunk[1];
                output.push(g1 ^ g2); // Simplified - real Viterbi much more complex
            }
        }

        output
    }
}

impl Waveform for Stanag4285 {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "STANAG 4285",
            full_name: "NATO HF Data Modem Standard",
            description: "Serial-tone PSK modem for HF communications",
            complexity: 4,
            bits_per_symbol: self.mode.modulation().bits_per_symbol(),
            carries_data: true,
            characteristics: &[
                "2400 baud symbol rate",
                "PSK modulation (BPSK/QPSK/8-PSK)",
                "Rate-1/2 convolutional coding",
                "Block interleaving for burst errors",
                "16-symbol probe sequences",
                "Data rates 75-3600 bps",
            ],
            history: "Developed by NATO in 1980s for reliable HF data",
            modern_usage: "Military and government HF networks worldwide",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        let mut modem = Self::new(self.common.sample_rate, self.mode);
        modem.build_frame(data)
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let mut modem = Self::new(self.common.sample_rate, self.mode);

        // Skip sync preamble (80 symbols)
        let sync_samples = 80 * self.samples_per_symbol;
        if samples.len() < sync_samples {
            return DemodResult::default();
        }

        let data_samples = &samples[sync_samples..];
        let symbols = modem.demodulate_samples(data_samples);
        let bits = modem.symbols_to_bits(&symbols);
        let deinterleaved = modem.deinterleave(&bits);
        let descrambled = modem.descramble(&deinterleaved);
        let decoded = modem.decode_viterbi(&descrambled);

        DemodResult {
            bits: decoded,
            symbols: symbols.iter().map(|&s| s as u16).collect(),
            ber_estimate: None,
            snr_estimate: None,
            metadata: std::collections::HashMap::new(),
        }
    }

    fn samples_per_symbol(&self) -> usize {
        self.samples_per_symbol
    }

    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);
        let constellation = self.mode.modulation().constellation();
        let labels: Vec<String> = (0..constellation.len())
            .map(|i| format!("{:0width$b}", i, width = self.mode.modulation().bits_per_symbol() as usize))
            .collect();

        VisualizationData {
            samples,
            constellation,
            constellation_labels: labels,
            spectrum: vec![],
            description: format!(
                "STANAG 4285 {} mode - {} bps, {} modulation",
                if self.mode.is_long_interleave() { "long" } else { "short" },
                self.mode.data_rate(),
                match self.mode.modulation() {
                    PskType::Bpsk => "BPSK",
                    PskType::Qpsk => "QPSK",
                    PskType::Psk8 => "8-PSK",
                }
            ),
        }
    }

    fn get_modulation_stages(&self, data: &[u8]) -> Vec<ModulationStage> {
        let (k, n) = self.mode.code_rate();

        vec![
            ModulationStage::new(
                "Data Input",
                format!("Raw data bytes: {} bytes", data.len()),
            ).with_input_bits(data.to_vec()),

            ModulationStage::new(
                "Convolutional Encoding",
                format!("Rate-{}/{} K=7 convolutional code", k, n),
            ),

            ModulationStage::new(
                "Scrambling",
                "LFSR scrambler (x^9 + x^4 + 1) for spectral shaping",
            ),

            ModulationStage::new(
                "Interleaving",
                format!(
                    "{} interleave ({} symbols)",
                    if self.mode.is_long_interleave() { "Long" } else { "Short" },
                    self.mode.interleave_depth()
                ),
            ),

            ModulationStage::new(
                "Symbol Mapping",
                format!(
                    "{} modulation: {} bits/symbol",
                    match self.mode.modulation() {
                        PskType::Bpsk => "BPSK",
                        PskType::Qpsk => "QPSK",
                        PskType::Psk8 => "8-PSK",
                    },
                    self.mode.modulation().bits_per_symbol()
                ),
            ).with_constellation(self.mode.modulation().constellation()),

            ModulationStage::new(
                "Frame Assembly",
                "Add sync preamble (80 sym) and probe sequences (16 sym each)",
            ),

            ModulationStage::new(
                "Pulse Shaping",
                format!("{} samples/symbol at {} Hz", self.samples_per_symbol, self.common.sample_rate),
            ).with_samples(self.modulate(data)),
        ]
    }

    fn get_demodulation_steps(&self, samples: &[IQSample]) -> Vec<DemodulationStep> {
        vec![
            DemodulationStep::new(
                "Sync Detection",
                "Detect 80-symbol sync preamble",
            ),

            DemodulationStep::new(
                "Symbol Timing Recovery",
                "Lock to 2400 baud symbol rate",
            ),

            DemodulationStep::new(
                "Channel Estimation",
                "Use 16-symbol probe sequences for equalization",
            ),

            DemodulationStep::new(
                "Symbol Detection",
                format!(
                    "{} detection on {} symbols",
                    match self.mode.modulation() {
                        PskType::Bpsk => "BPSK",
                        PskType::Qpsk => "QPSK",
                        PskType::Psk8 => "8-PSK",
                    },
                    samples.len() / self.samples_per_symbol
                ),
            ),

            DemodulationStep::new(
                "De-interleaving",
                "Reverse block interleaving",
            ),

            DemodulationStep::new(
                "Descrambling",
                "Remove LFSR scrambling",
            ),

            DemodulationStep::new(
                "Viterbi Decoding",
                "Decode rate-1/2 convolutional code",
            ),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mode_properties() {
        let mode = Stanag4285Mode::Mode2400Short;
        assert_eq!(mode.data_rate(), 2400);
        assert_eq!(mode.modulation(), PskType::Qpsk);
        assert!(!mode.is_long_interleave());
        assert_eq!(mode.code_rate(), (1, 2));
    }

    #[test]
    fn test_psk_constellation() {
        let bpsk = PskType::Bpsk;
        assert_eq!(bpsk.bits_per_symbol(), 1);
        assert_eq!(bpsk.constellation().len(), 2);

        let qpsk = PskType::Qpsk;
        assert_eq!(qpsk.bits_per_symbol(), 2);
        assert_eq!(qpsk.constellation().len(), 4);

        let psk8 = PskType::Psk8;
        assert_eq!(psk8.bits_per_symbol(), 3);
        assert_eq!(psk8.constellation().len(), 8);
    }

    #[test]
    fn test_psk_roundtrip() {
        for psk in [PskType::Bpsk, PskType::Qpsk, PskType::Psk8] {
            let n = 1 << psk.bits_per_symbol();
            for sym in 0..n {
                let iq = psk.symbol_to_iq(sym as u8);
                let recovered = psk.iq_to_symbol(iq);
                assert_eq!(sym as u8, recovered, "PSK {:?} symbol {} failed", psk, sym);
            }
        }
    }

    #[test]
    fn test_convolutional_encoder() {
        let mut modem = Stanag4285::default_mode(48000.0);
        let input = vec![1, 0, 1, 1, 0];
        let encoded = modem.encode_convolutional(&input);

        // Rate 1/2, so output should be 2x input length
        assert_eq!(encoded.len(), input.len() * 2);
    }

    #[test]
    fn test_modulation() {
        let modem = Stanag4285::default_mode(48000.0);
        let data = vec![0xAA, 0x55];
        let samples = modem.modulate(&data);

        // Should have samples
        assert!(!samples.is_empty());

        // Check amplitude normalization
        for sample in &samples {
            assert!(sample.norm() <= 1.5); // Allow some margin
        }
    }

    #[test]
    fn test_all_modes() {
        let modes = [
            Stanag4285Mode::Mode75Short,
            Stanag4285Mode::Mode150Short,
            Stanag4285Mode::Mode300Short,
            Stanag4285Mode::Mode600Short,
            Stanag4285Mode::Mode1200Short,
            Stanag4285Mode::Mode2400Short,
            Stanag4285Mode::Mode3600Short,
        ];

        for mode in modes {
            let modem = Stanag4285::new(48000.0, mode);
            let info = modem.info();
            assert_eq!(info.name, "STANAG 4285");
        }
    }
}

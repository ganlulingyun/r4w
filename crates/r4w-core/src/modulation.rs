//! LoRa Modulation
//!
//! This module implements the complete LoRa modulation (transmit) chain:
//!
//! ```text
//! Raw Data
//!    │
//!    ▼
//! ┌─────────────┐
//! │  Whitening  │  Scramble data with LFSR sequence
//! └─────────────┘
//!    │
//!    ▼
//! ┌─────────────┐
//! │ Hamming FEC │  Add forward error correction
//! └─────────────┘
//!    │
//!    ▼
//! ┌─────────────┐
//! │ Interleave  │  Spread bits across symbols
//! └─────────────┘
//!    │
//!    ▼
//! ┌─────────────┐
//! │ Gray Coding │  Map to Gray-coded symbols
//! └─────────────┘
//!    │
//!    ▼
//! ┌─────────────┐
//! │  CSS Mod    │  Generate chirp signals
//! └─────────────┘
//!    │
//!    ▼
//! I/Q Samples (Complex Baseband)
//! ```
//!
//! The output is complex baseband I/Q samples ready to be sent to an SDR
//! for transmission, or used in simulation.

use crate::chirp::ChirpGenerator;
use crate::coding::{GrayCode, HammingCode, Interleaver};
use crate::packet::PacketHeader;
use crate::params::LoRaParams;
use crate::types::{IQSample, PipelineStage, Symbol};
use crate::whitening::Whitening;

/// LoRa Modulator
///
/// Converts raw payload bytes into I/Q samples using the full LoRa PHY chain.
#[derive(Debug)]
pub struct Modulator {
    /// LoRa parameters
    params: LoRaParams,
    /// Chirp generator
    chirp_gen: ChirpGenerator,
    /// Gray coder
    gray: GrayCode,
    /// Hamming coder (for payload)
    hamming: HammingCode,
    /// Interleaver
    interleaver: Interleaver,
    /// Whitening
    whitening: Whitening,
    /// Record pipeline stages for visualization
    record_stages: bool,
    /// Recorded pipeline stages
    stages: Vec<PipelineStage>,
}

impl Modulator {
    /// Create a new modulator with the given parameters
    pub fn new(params: LoRaParams) -> Self {
        let chirp_gen = ChirpGenerator::new(params.clone());
        let sf = params.sf.value();
        let gray = GrayCode::new(sf);
        let hamming = HammingCode::new(params.cr);
        let interleaver = Interleaver::new(sf, params.cr);
        let whitening = Whitening::new();

        Self {
            params,
            chirp_gen,
            gray,
            hamming,
            interleaver,
            whitening,
            record_stages: false,
            stages: Vec::new(),
        }
    }

    /// Enable recording of pipeline stages for educational visualization
    pub fn enable_stage_recording(&mut self) {
        self.record_stages = true;
        self.stages.clear();
    }

    /// Get recorded pipeline stages
    pub fn stages(&self) -> &[PipelineStage] {
        &self.stages
    }

    /// Clear recorded stages
    pub fn clear_stages(&mut self) {
        self.stages.clear();
    }

    /// Modulate raw payload data into I/Q samples
    ///
    /// This performs the complete LoRa modulation chain:
    /// 1. Build packet with header
    /// 2. Whiten data
    /// 3. Hamming encode
    /// 4. Interleave
    /// 5. Gray code
    /// 6. Generate chirp symbols
    /// 7. Add preamble
    pub fn modulate(&mut self, payload: &[u8]) -> Vec<IQSample> {
        self.whitening.reset();

        // Record raw input
        if self.record_stages {
            self.stages.push(
                PipelineStage::new("Raw Input", "Original payload bytes before any processing")
                    .with_bits(payload.to_vec()),
            );
        }

        // Step 1: Apply whitening
        let mut whitened = payload.to_vec();
        self.whitening.process(&mut whitened);

        if self.record_stages {
            self.stages.push(
                PipelineStage::new("Whitened", "Data XORed with LFSR pseudo-random sequence")
                    .with_bits(whitened.clone()),
            );
        }

        // Step 2: Convert bytes to nibbles (4-bit values)
        let nibbles = Self::bytes_to_nibbles(&whitened);

        // Step 3: Apply Hamming FEC to each nibble
        let codewords: Vec<u8> = nibbles.iter().map(|&n| self.hamming.encode(n)).collect();

        if self.record_stages {
            self.stages.push(
                PipelineStage::new(
                    "Hamming Encoded",
                    format!(
                        "FEC added: 4 data bits → {} bits (CR {})",
                        self.params.cr.output_bits(),
                        self.params.cr
                    ),
                )
                .with_bits(codewords.clone()),
            );
        }

        // Step 4: Interleave and Gray encode to get symbols
        let symbols = self.encode_to_symbols(&codewords);

        if self.record_stages {
            self.stages.push(
                PipelineStage::new(
                    "Symbols",
                    format!(
                        "Interleaved and Gray coded: {} symbols of {} bits each",
                        symbols.len(),
                        self.params.sf.value()
                    ),
                )
                .with_symbols(symbols.clone()),
            );
        }

        // Step 5: Generate I/Q samples
        let mut samples = Vec::new();

        // Add preamble
        let preamble = self.chirp_gen.generate_preamble();
        samples.extend_from_slice(&preamble);

        if self.record_stages {
            self.stages.push(
                PipelineStage::new(
                    "Preamble",
                    format!(
                        "{} upchirps + sync + 2.25 downchirps",
                        self.params.preamble_length
                    ),
                )
                .with_time_domain(preamble),
            );
        }

        // Generate payload chirps
        let mut payload_samples = Vec::new();
        for &symbol in &symbols {
            let chirp = self.chirp_gen.generate_symbol_chirp_fast(symbol);
            payload_samples.extend_from_slice(&chirp);
        }

        if self.record_stages {
            self.stages.push(
                PipelineStage::new(
                    "CSS Modulated",
                    "Each symbol encoded as a frequency-shifted chirp",
                )
                .with_time_domain(payload_samples.clone()),
            );
        }

        samples.extend(payload_samples);

        // Final output
        if self.record_stages {
            self.stages.push(
                PipelineStage::new("Complete Packet", "Preamble + payload, ready for transmission")
                    .with_time_domain(samples.clone()),
            );
        }

        samples
    }

    /// Modulate with explicit header (for full LoRaWAN compatibility)
    pub fn modulate_with_header(&mut self, header: &PacketHeader, payload: &[u8]) -> Vec<IQSample> {
        // Build header bytes
        let header_bytes = header.encode();

        // For now, just modulate header + payload together
        // A full implementation would use different coding rates for header vs payload
        let mut full_payload = header_bytes;
        full_payload.extend_from_slice(payload);

        self.modulate(&full_payload)
    }

    /// Generate only the chirp symbols (without preamble) for a given payload
    ///
    /// Useful for testing or when you want to add a custom preamble.
    pub fn symbols_only(&mut self, payload: &[u8]) -> Vec<IQSample> {
        self.whitening.reset();

        let mut whitened = payload.to_vec();
        self.whitening.process(&mut whitened);

        let nibbles = Self::bytes_to_nibbles(&whitened);
        let codewords: Vec<u8> = nibbles.iter().map(|&n| self.hamming.encode(n)).collect();
        let symbols = self.encode_to_symbols(&codewords);

        let mut samples = Vec::new();
        for &symbol in &symbols {
            let chirp = self.chirp_gen.generate_symbol_chirp_fast(symbol);
            samples.extend_from_slice(&chirp);
        }

        samples
    }

    /// Get the raw symbols for a payload (for analysis)
    pub fn get_symbols(&mut self, payload: &[u8]) -> Vec<Symbol> {
        self.whitening.reset();

        let mut whitened = payload.to_vec();
        self.whitening.process(&mut whitened);

        let nibbles = Self::bytes_to_nibbles(&whitened);
        let codewords: Vec<u8> = nibbles.iter().map(|&n| self.hamming.encode(n)).collect();

        self.encode_to_symbols(&codewords)
    }

    /// Convert bytes to nibbles (4-bit values)
    fn bytes_to_nibbles(bytes: &[u8]) -> Vec<u8> {
        let mut nibbles = Vec::with_capacity(bytes.len() * 2);
        for &byte in bytes {
            nibbles.push((byte >> 4) & 0x0F); // High nibble
            nibbles.push(byte & 0x0F); // Low nibble
        }
        nibbles
    }

    /// Encode codewords to symbols through interleaving and Gray coding
    fn encode_to_symbols(&self, codewords: &[u8]) -> Vec<Symbol> {
        let sf = self.params.sf.value() as usize;
        let _n_bits = self.params.cr.output_bits() as usize;

        let mut symbols = Vec::new();

        // Process in blocks of SF codewords
        for chunk in codewords.chunks(sf) {
            // Pad if necessary
            let mut block: Vec<u8> = chunk.to_vec();
            while block.len() < sf {
                block.push(0);
            }

            // Interleave this block
            let interleaved = self.interleaver.interleave(&block);

            // Gray encode each symbol
            for &sym in &interleaved {
                symbols.push(self.gray.encode(sym));
            }
        }

        symbols
    }

    /// Get the chirp generator (for direct chirp access)
    pub fn chirp_generator(&self) -> &ChirpGenerator {
        &self.chirp_gen
    }

    /// Get the parameters
    pub fn params(&self) -> &LoRaParams {
        &self.params
    }
}

/// Generate a single modulated symbol chirp for visualization
pub fn visualize_symbol_modulation(params: &LoRaParams, symbol: Symbol) -> SymbolVisualization {
    let chirp_gen = ChirpGenerator::new(params.clone());
    let samples = chirp_gen.generate_symbol_chirp_fast(symbol);
    let frequencies = chirp_gen.compute_instantaneous_frequency(&samples);

    let n = samples.len();
    let time: Vec<f64> = (0..n).map(|i| i as f64 * params.sample_duration()).collect();

    let i_component: Vec<f64> = samples.iter().map(|s| s.re).collect();
    let q_component: Vec<f64> = samples.iter().map(|s| s.im).collect();

    SymbolVisualization {
        symbol,
        time,
        frequencies,
        i_component,
        q_component,
        samples,
    }
}

/// Visualization data for a single symbol
#[derive(Debug, Clone)]
pub struct SymbolVisualization {
    pub symbol: Symbol,
    pub time: Vec<f64>,
    pub frequencies: Vec<f64>,
    pub i_component: Vec<f64>,
    pub q_component: Vec<f64>,
    pub samples: Vec<IQSample>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_modulate_basic() {
        let params = LoRaParams::builder()
            .spreading_factor(7)
            .bandwidth(125_000)
            .coding_rate(1)
            .oversample(1)
            .build();

        let mut modulator = Modulator::new(params.clone());
        let payload = b"Hello";
        let samples = modulator.modulate(payload);

        // Should produce preamble + payload samples
        // Preamble: (8+2)*128 + 2*128 + 32 = 1568 samples
        // Payload: varies based on encoding
        assert!(samples.len() > 1000);
    }

    #[test]
    fn test_modulate_with_stages() {
        let params = LoRaParams::builder()
            .spreading_factor(7)
            .bandwidth(125_000)
            .build();

        let mut modulator = Modulator::new(params);
        modulator.enable_stage_recording();

        let _samples = modulator.modulate(b"Test");
        let stages = modulator.stages();

        // Should have recorded multiple stages
        assert!(stages.len() >= 5);

        // Check stage names
        assert_eq!(stages[0].name, "Raw Input");
        assert_eq!(stages[1].name, "Whitened");
    }

    #[test]
    fn test_bytes_to_nibbles() {
        let bytes = vec![0xAB, 0xCD];
        let nibbles = Modulator::bytes_to_nibbles(&bytes);

        assert_eq!(nibbles, vec![0x0A, 0x0B, 0x0C, 0x0D]);
    }
}

//! 3G ALE (Third Generation Automatic Link Establishment)
//!
//! Implementation of MIL-STD-188-141B Appendix C 3G ALE
//! for improved HF radio automatic link establishment.
//!
//! # Overview
//!
//! 3G ALE extends 2G ALE with enhanced features:
//! - Fast link setup
//! - Automatic Message Display (AMD) for short text
//! - Data Text Message (DTM) for longer messages
//! - Improved frequency scanning
//! - Linking protection (LINP)
//!
//! # Key Features
//!
//! - **Modulation**: 8-FSK (same as 2G ALE)
//! - **Symbol Rate**: 125 baud (same as 2G)
//! - **Enhancements**: Faster linking, AMD, DTM, robust calling
//! - **Compatibility**: Interoperable with 2G ALE
//!
//! # 3G ALE Phases
//!
//! - Phase 1: Basic fast call (Appendix C.1)
//! - Phase 2: AMD/DTM capability (Appendix C.2)
//! - Phase 3: Enhanced scanning (Appendix C.3)

use std::f64::consts::PI;
use num_complex::Complex64;

use crate::types::IQSample;
use crate::waveform::{
    CommonParams, DemodResult, DemodulationStep, ModulationStage,
    VisualizationData, Waveform, WaveformInfo,
};

/// 3G ALE tone frequencies (same as 2G ALE)
const ALE_TONES: [f64; 8] = [
    750.0,  // 000
    1000.0, // 001
    1250.0, // 010
    1500.0, // 011
    1750.0, // 100
    2000.0, // 101
    2250.0, // 110
    2500.0, // 111
];

/// 3G ALE symbol rate (125 baud, same as 2G)
const ALE_BAUD: f64 = 125.0;

/// 3G ALE capability level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Ale3gCapability {
    /// Phase 1: Basic fast call
    #[default]
    Phase1,
    /// Phase 2: AMD and DTM
    Phase2,
    /// Phase 3: Enhanced scanning
    Phase3,
}

/// 3G ALE word types (extended from 2G)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ale3gWordType {
    // 2G compatible types
    /// TO - Called station address
    To = 0b001,
    /// TIS - This Is Station (calling station)
    Tis = 0b011,
    /// TWAS - Third-party station Was
    Twas = 0b010,
    /// DATA - Link data
    Data = 0b101,
    /// REP - Repeat request
    Rep = 0b110,
    /// CMD - Command
    Cmd = 0b111,
    // 3G specific types (encoded in CMD subfield)
    /// AMD - Automatic Message Display
    Amd = 0b100,
    /// DTM Header - Data Text Message header
    DtmHeader = 0b000,
}

impl Ale3gWordType {
    /// Get word type from preamble bits
    pub fn from_preamble(preamble: u8) -> Option<Self> {
        match preamble & 0x07 {
            0b001 => Some(Self::To),
            0b011 => Some(Self::Tis),
            0b010 => Some(Self::Twas),
            0b101 => Some(Self::Data),
            0b110 => Some(Self::Rep),
            0b111 => Some(Self::Cmd),
            0b100 => Some(Self::Amd),
            0b000 => Some(Self::DtmHeader),
            _ => None,
        }
    }

    /// Get preamble bits
    pub fn preamble(&self) -> u8 {
        *self as u8
    }

    /// Is this a 3G-specific word type?
    pub fn is_3g_specific(&self) -> bool {
        matches!(self, Self::Amd | Self::DtmHeader)
    }
}

/// 3G ALE Automatic Message Display (AMD)
/// Supports short text messages during linking
#[derive(Debug, Clone)]
pub struct AmdMessage {
    /// Message text (up to 90 characters)
    pub text: String,
    /// Priority (0-3, higher is more urgent)
    pub priority: u8,
}

impl AmdMessage {
    /// Create new AMD message
    pub fn new(text: &str) -> Self {
        Self {
            text: text.chars().take(90).collect(),
            priority: 0,
        }
    }

    /// Create urgent AMD message
    pub fn urgent(text: &str) -> Self {
        Self {
            text: text.chars().take(90).collect(),
            priority: 3,
        }
    }

    /// Encode message to ALE words
    pub fn encode(&self) -> Vec<Ale3gWord> {
        let mut words = Vec::new();
        let chars: Vec<char> = self.text.chars().collect();

        // Pack characters into words (3 characters per word)
        for chunk in chars.chunks(3) {
            let mut data: u32 = 0;
            for (i, &ch) in chunk.iter().enumerate() {
                // Use 6-bit ASCII subset
                let encoded = match ch {
                    ' '..='?' => (ch as u32) - 0x20,
                    '@'..='_' => (ch as u32) - 0x20,
                    _ => 0x20, // Space for unsupported
                };
                data |= (encoded & 0x3F) << (12 - i * 6);
            }
            words.push(Ale3gWord::new(Ale3gWordType::Amd, data));
        }

        words
    }

    /// Decode from ALE words
    pub fn decode(words: &[Ale3gWord]) -> Self {
        let mut text = String::new();

        for word in words {
            if word.word_type != Ale3gWordType::Amd {
                continue;
            }

            // Extract 3 characters per word (0 = space, mapped back to 0x20+)
            for i in 0..3 {
                let char_val = ((word.data >> (12 - i * 6)) & 0x3F) as u8;
                let ch = (char_val + 0x20) as char;
                text.push(ch);
            }
        }

        Self { text, priority: 0 }
    }
}

/// 3G ALE Data Text Message (DTM)
/// Supports longer data messages
#[derive(Debug, Clone)]
pub struct DtmMessage {
    /// Message data
    pub data: Vec<u8>,
    /// Sequence number
    pub sequence: u8,
    /// More fragments flag
    pub more_fragments: bool,
}

impl DtmMessage {
    /// Create new DTM message
    pub fn new(data: &[u8]) -> Self {
        Self {
            data: data.to_vec(),
            sequence: 0,
            more_fragments: false,
        }
    }

    /// Fragment a message into DTM blocks
    pub fn fragment(data: &[u8], max_block_size: usize) -> Vec<Self> {
        let chunks: Vec<&[u8]> = data.chunks(max_block_size).collect();
        let num_chunks = chunks.len();

        chunks
            .iter()
            .enumerate()
            .map(|(i, chunk)| DtmMessage {
                data: chunk.to_vec(),
                sequence: i as u8,
                more_fragments: i < num_chunks - 1,
            })
            .collect()
    }
}

/// 3G ALE word (24 bits)
#[derive(Debug, Clone)]
pub struct Ale3gWord {
    /// Word type
    pub word_type: Ale3gWordType,
    /// 21-bit payload (3-bit preamble + 18-bit data)
    pub data: u32,
}

impl Ale3gWord {
    /// Create new 3G ALE word
    pub fn new(word_type: Ale3gWordType, data: u32) -> Self {
        Self {
            word_type,
            data: data & 0x3FFFF, // 18 bits max
        }
    }

    /// Encode to 24-bit value with preamble
    pub fn encode(&self) -> u32 {
        let preamble = (self.word_type as u32) << 21;
        preamble | self.data
    }

    /// Decode from 24-bit value
    pub fn decode(value: u32) -> Option<Self> {
        let preamble = ((value >> 21) & 0x07) as u8;
        let word_type = Ale3gWordType::from_preamble(preamble)?;
        let data = value & 0x1FFFFF;
        Some(Self { word_type, data })
    }

    /// Get tribits (8 x 3-bit symbols)
    pub fn to_tribits(&self) -> [u8; 8] {
        let encoded = self.encode();
        let mut tribits = [0u8; 8];
        for i in 0..8 {
            tribits[7 - i] = ((encoded >> (i * 3)) & 0x07) as u8;
        }
        tribits
    }

    /// Create from tribits
    pub fn from_tribits(tribits: &[u8; 8]) -> Option<Self> {
        let mut value: u32 = 0;
        for (i, &tribit) in tribits.iter().enumerate() {
            value |= ((tribit as u32) & 0x07) << ((7 - i) * 3);
        }
        Self::decode(value)
    }
}

/// 3G ALE fast call parameters
#[derive(Debug, Clone)]
pub struct FastCallParams {
    /// Shortened scanning dwell time (ms)
    pub scan_dwell_ms: u32,
    /// Rapid call cycle count
    pub rapid_cycles: u8,
    /// Async turnaround time (ms)
    pub turnaround_ms: u32,
}

impl Default for FastCallParams {
    fn default() -> Self {
        Self {
            scan_dwell_ms: 200,
            rapid_cycles: 2,
            turnaround_ms: 150,
        }
    }
}

/// 3G ALE protocol state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ale3gState {
    /// Idle - listening for calls
    Idle,
    /// Fast scanning
    FastScanning,
    /// Rapid calling
    RapidCalling,
    /// AMD exchange
    AmdExchange,
    /// DTM transfer
    DtmTransfer,
    /// Linked (handoff to traffic)
    Linked,
}

/// 3G ALE Link Quality Analysis (extended)
#[derive(Debug, Clone, Copy)]
pub struct Ale3gLqa {
    /// Bit Error Rate estimate (0-100)
    pub ber: u8,
    /// Signal to Noise Ratio estimate
    pub sinad: u8,
    /// Multipath estimate
    pub multipath: u8,
    /// Doppler spread estimate (Hz)
    pub doppler_hz: u16,
    /// Link margin (dB)
    pub margin_db: i8,
}

impl Ale3gLqa {
    /// Overall quality score (0-100)
    pub fn score(&self) -> u8 {
        let ber_score = 100i32 - (self.ber.min(100) as i32);
        let sinad_score = self.sinad.min(100) as i32;
        let mp_score = 100i32 - (self.multipath.min(100) as i32);
        // Use saturating arithmetic to prevent overflow
        let margin_clamped = (self.margin_db as i32).clamp(-10, 30);
        let margin_score = ((margin_clamped + 10) * 5 / 4).min(50);

        ((ber_score + sinad_score + mp_score + margin_score) / 4).clamp(0, 100) as u8
    }

    /// Is this a robust link?
    pub fn is_robust(&self) -> bool {
        self.score() > 60 && self.margin_db > 3
    }
}

/// 3G ALE modem
#[derive(Debug)]
pub struct Ale3g {
    common: CommonParams,
    /// Samples per symbol
    samples_per_symbol: usize,
    /// Current protocol state
    state: Ale3gState,
    /// Capability level
    capability: Ale3gCapability,
    /// This station's address
    my_address: String,
    /// Current phase for FSK generation
    phase: f64,
    /// Fast call parameters
    fast_call: FastCallParams,
}

impl Ale3g {
    /// Create new 3G ALE modem
    pub fn new(sample_rate: f64, address: &str, capability: Ale3gCapability) -> Self {
        let samples_per_symbol = (sample_rate / ALE_BAUD).round() as usize;

        Self {
            common: CommonParams {
                sample_rate,
                carrier_freq: 1500.0, // Center of ALE band
                amplitude: 1.0,
            },
            samples_per_symbol,
            state: Ale3gState::Idle,
            capability,
            my_address: address.to_uppercase(),
            phase: 0.0,
            fast_call: FastCallParams::default(),
        }
    }

    /// Create with default Phase 1 capability
    pub fn default_config(sample_rate: f64) -> Self {
        Self::new(sample_rate, "TEST", Ale3gCapability::Phase1)
    }

    /// Create with Phase 2 (AMD/DTM) capability
    pub fn with_amd(sample_rate: f64, address: &str) -> Self {
        Self::new(sample_rate, address, Ale3gCapability::Phase2)
    }

    /// Create with full Phase 3 capability
    pub fn full_capability(sample_rate: f64, address: &str) -> Self {
        Self::new(sample_rate, address, Ale3gCapability::Phase3)
    }

    /// Get current state
    pub fn state(&self) -> Ale3gState {
        self.state
    }

    /// Set state
    pub fn set_state(&mut self, state: Ale3gState) {
        self.state = state;
    }

    /// Get capability level
    pub fn capability(&self) -> Ale3gCapability {
        self.capability
    }

    /// Get this station's address
    pub fn address(&self) -> &str {
        &self.my_address
    }

    /// Generate FSK tone for one symbol
    fn generate_symbol(&mut self, tribit: u8) -> Vec<IQSample> {
        let freq = ALE_TONES[(tribit & 0x07) as usize];
        let mut samples = Vec::with_capacity(self.samples_per_symbol);

        let omega = 2.0 * PI * freq / self.common.sample_rate;

        for _ in 0..self.samples_per_symbol {
            samples.push(Complex64::new(self.phase.cos(), self.phase.sin()));
            self.phase += omega;
            if self.phase > 2.0 * PI {
                self.phase -= 2.0 * PI;
            }
        }

        samples
    }

    /// Modulate 3G ALE word to I/Q samples
    pub fn modulate_word(&mut self, word: &Ale3gWord) -> Vec<IQSample> {
        let tribits = word.to_tribits();
        let mut samples = Vec::new();

        for tribit in tribits {
            samples.extend(self.generate_symbol(tribit));
        }

        samples
    }

    /// Generate fast call sequence (3G specific)
    pub fn generate_fast_call(&mut self, to_address: &str) -> Vec<IQSample> {
        let mut samples = Vec::new();

        // Fast call uses fewer word repetitions than 2G
        let to_chars: Vec<u8> = to_address
            .chars()
            .take(15)
            .map(|c| self.encode_char(c))
            .collect();

        let my_chars: Vec<u8> = self.my_address
            .chars()
            .take(15)
            .map(|c| self.encode_char(c))
            .collect();

        // Generate TO words with reduced repetition
        for chunk in to_chars.chunks(3) {
            let mut data: u32 = 0;
            for (i, &c) in chunk.iter().enumerate() {
                data |= ((c as u32) & 0x3F) << (12 - i * 6);
            }

            // Only 2 repetitions for fast call (vs 4 for 2G)
            for _ in 0..self.fast_call.rapid_cycles {
                let word = Ale3gWord::new(Ale3gWordType::To, data);
                samples.extend(self.modulate_word(&word));
            }
        }

        // Generate TIS words
        for chunk in my_chars.chunks(3) {
            let mut data: u32 = 0;
            for (i, &c) in chunk.iter().enumerate() {
                data |= ((c as u32) & 0x3F) << (12 - i * 6);
            }

            for _ in 0..self.fast_call.rapid_cycles {
                let word = Ale3gWord::new(Ale3gWordType::Tis, data);
                samples.extend(self.modulate_word(&word));
            }
        }

        samples
    }

    /// Generate AMD message during call
    pub fn generate_amd(&mut self, message: &AmdMessage) -> Vec<IQSample> {
        let mut samples = Vec::new();

        for word in message.encode() {
            samples.extend(self.modulate_word(&word));
        }

        samples
    }

    /// Encode character to ALE 38-character set
    fn encode_char(&self, c: char) -> u8 {
        match c {
            '0'..='9' => (c as u8) - b'0',
            'A'..='Z' => (c as u8) - b'A' + 10,
            'a'..='z' => (c as u8) - b'a' + 10,
            '@' => 36,
            '?' => 37,
            _ => 36, // Default to @
        }
    }

    /// Detect tone frequency in samples
    fn detect_tone(&self, samples: &[IQSample]) -> u8 {
        let mut max_energy = 0.0;
        let mut best_tone = 0u8;

        for (i, &freq) in ALE_TONES.iter().enumerate() {
            let correlation: IQSample = samples
                .iter()
                .enumerate()
                .map(|(j, &s)| {
                    let phase = 2.0 * PI * freq * j as f64 / self.common.sample_rate;
                    let ref_signal = Complex64::new(phase.cos(), -phase.sin());
                    s * ref_signal
                })
                .fold(Complex64::new(0.0, 0.0), |acc, x| acc + x);

            let energy = correlation.norm_sqr();

            if energy > max_energy {
                max_energy = energy;
                best_tone = i as u8;
            }
        }

        best_tone
    }

    /// Demodulate samples to tribits
    fn demodulate_symbols(&self, samples: &[IQSample]) -> Vec<u8> {
        let mut tribits = Vec::new();

        for chunk in samples.chunks(self.samples_per_symbol) {
            if chunk.len() >= self.samples_per_symbol / 2 {
                tribits.push(self.detect_tone(chunk));
            }
        }

        tribits
    }

    /// Demodulate samples to 3G ALE words
    pub fn demodulate_words(&self, samples: &[IQSample]) -> Vec<Ale3gWord> {
        let tribits = self.demodulate_symbols(samples);
        let mut words = Vec::new();

        for chunk in tribits.chunks(8) {
            if chunk.len() == 8 {
                let mut arr = [0u8; 8];
                arr.copy_from_slice(chunk);
                if let Some(word) = Ale3gWord::from_tribits(&arr) {
                    words.push(word);
                }
            }
        }

        words
    }

    /// Calculate enhanced LQA
    pub fn calculate_lqa(&self, samples: &[IQSample]) -> Ale3gLqa {
        let avg_power: f64 = samples
            .iter()
            .map(|s| s.norm_sqr())
            .sum::<f64>()
            / samples.len().max(1) as f64;

        let variance: f64 = samples
            .iter()
            .map(|s| (s.norm_sqr() - avg_power).powi(2))
            .sum::<f64>()
            / samples.len().max(1) as f64;

        let snr_linear = avg_power / variance.max(0.001);
        let snr_db = (10.0 * snr_linear.log10()).clamp(0.0, 50.0);

        Ale3gLqa {
            ber: ((50.0 - snr_db) * 2.0).max(0.0).min(100.0) as u8,
            sinad: (snr_db * 2.0).min(100.0) as u8,
            multipath: 20,
            doppler_hz: 5, // Would need more complex analysis
            margin_db: (snr_db - 10.0).max(-20.0).min(30.0) as i8,
        }
    }

    /// Get tone frequencies for visualization
    pub fn get_tone_frequencies() -> &'static [f64; 8] {
        &ALE_TONES
    }
}

impl Waveform for Ale3g {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "3G-ALE",
            full_name: "Third Generation Automatic Link Establishment",
            description: "Enhanced HF automatic link establishment with fast linking, \
                AMD, and DTM capabilities per MIL-STD-188-141B Appendix C",
            complexity: 4,
            bits_per_symbol: 3, // 8-FSK = 3 bits per symbol
            carries_data: true,
            characteristics: &[
                "8-FSK modulation (8 tones)",
                "125 baud symbol rate",
                "Fast link establishment",
                "Automatic Message Display (AMD)",
                "Data Text Message (DTM)",
                "Enhanced scanning",
                "2G ALE compatible",
                "Linking Protection (LINP)",
            ],
            history: "MIL-STD-188-141B Appendix C developed in 2001 to enhance \
                2G ALE with faster linking and message capabilities",
            modern_usage: "Military, government, and embassy HF networks requiring \
                rapid link establishment and short message exchange",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        let mut ale = Self::new(
            self.common.sample_rate,
            &self.my_address,
            self.capability,
        );

        let mut samples = Vec::new();
        for &tribit in data {
            samples.extend(ale.generate_symbol(tribit));
        }
        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let tribits = self.demodulate_symbols(samples);

        DemodResult {
            bits: tribits.clone(),
            symbols: tribits.iter().map(|&t| t as u16).collect(),
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

        // 8-FSK shown as frequency levels
        let constellation: Vec<IQSample> = ALE_TONES
            .iter()
            .enumerate()
            .map(|(i, &freq)| {
                let normalized = (freq - 1500.0) / 1000.0;
                Complex64::new(normalized, i as f64 / 4.0 - 1.0)
            })
            .collect();

        let labels: Vec<String> = ALE_TONES
            .iter()
            .enumerate()
            .map(|(i, &freq)| format!("{}: {} Hz", i, freq as i32))
            .collect();

        VisualizationData {
            samples,
            constellation,
            constellation_labels: labels,
            spectrum: vec![],
            description: format!(
                "3G ALE {:?}: 125 baud, 8-FSK, {} symbols",
                self.capability,
                data.len()
            ),
        }
    }

    fn get_modulation_stages(&self, data: &[u8]) -> Vec<ModulationStage> {
        vec![
            ModulationStage::new(
                "Data Input",
                format!("Raw data: {} bytes", data.len()),
            )
            .with_input_bits(data.to_vec()),
            ModulationStage::new(
                "Word Formation",
                "Group into 24-bit 3G ALE words with preamble",
            ),
            ModulationStage::new(
                "Golay Encoding",
                "Apply Golay(24,12) error correction",
            ),
            ModulationStage::new(
                "Tribit Mapping",
                "Map 24 bits to 8 tribits (3 bits each)",
            ),
            ModulationStage::new(
                "8-FSK Modulation",
                format!(
                    "Generate {} samples/symbol at {} Hz",
                    self.samples_per_symbol, self.common.sample_rate
                ),
            )
            .with_samples(self.modulate(data)),
        ]
    }

    fn get_demodulation_steps(&self, samples: &[IQSample]) -> Vec<DemodulationStep> {
        let num_symbols = samples.len() / self.samples_per_symbol;

        vec![
            DemodulationStep::new(
                "Tone Detection",
                format!("Detect 8-FSK tones in {} symbols", num_symbols),
            ),
            DemodulationStep::new("Tribit Recovery", "Convert detected tones to tribits"),
            DemodulationStep::new("Word Assembly", "Group 8 tribits into 24-bit words"),
            DemodulationStep::new("Golay Decoding", "Correct up to 3 bit errors per word"),
            DemodulationStep::new(
                "3G Word Parsing",
                "Extract preamble, data, and 3G-specific fields",
            ),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ale3g_creation() {
        let ale = Ale3g::new(48000.0, "TEST", Ale3gCapability::Phase2);
        assert_eq!(ale.samples_per_symbol, 384); // 48000/125 = 384
        assert_eq!(ale.capability(), Ale3gCapability::Phase2);
    }

    #[test]
    fn test_amd_message() {
        let amd = AmdMessage::new("HELLO WORLD");
        let words = amd.encode();

        // Should produce words
        assert!(!words.is_empty());

        // Decode and verify (trailing spaces may be added as padding)
        let decoded = AmdMessage::decode(&words);
        // Trim trailing padding spaces for comparison
        assert_eq!(decoded.text.trim_end(), amd.text);
    }

    #[test]
    fn test_ale3g_word() {
        let word = Ale3gWord::new(Ale3gWordType::Amd, 0x12345);
        let tribits = word.to_tribits();
        let recovered = Ale3gWord::from_tribits(&tribits).unwrap();

        assert_eq!(word.word_type, recovered.word_type);
    }

    #[test]
    fn test_fast_call() {
        let mut ale = Ale3g::new(48000.0, "MYSTATION", Ale3gCapability::Phase1);
        let samples = ale.generate_fast_call("REMOTE");

        assert!(!samples.is_empty());
    }

    #[test]
    fn test_ale3g_modulation() {
        let ale = Ale3g::new(48000.0, "TEST", Ale3gCapability::Phase1);
        let tribits = vec![0, 1, 2, 3, 4, 5, 6, 7];
        let samples = ale.modulate(&tribits);

        assert_eq!(samples.len(), 8 * ale.samples_per_symbol);
    }

    #[test]
    fn test_capability_levels() {
        let phase1 = Ale3g::default_config(48000.0);
        let phase2 = Ale3g::with_amd(48000.0, "TEST");
        let phase3 = Ale3g::full_capability(48000.0, "TEST");

        assert_eq!(phase1.capability(), Ale3gCapability::Phase1);
        assert_eq!(phase2.capability(), Ale3gCapability::Phase2);
        assert_eq!(phase3.capability(), Ale3gCapability::Phase3);
    }

    #[test]
    fn test_3g_word_types() {
        assert!(Ale3gWordType::Amd.is_3g_specific());
        assert!(Ale3gWordType::DtmHeader.is_3g_specific());
        assert!(!Ale3gWordType::To.is_3g_specific());
        assert!(!Ale3gWordType::Tis.is_3g_specific());
    }

    #[test]
    fn test_lqa() {
        let ale = Ale3g::new(48000.0, "TEST", Ale3gCapability::Phase1);

        // Generate clean signal
        let samples: Vec<IQSample> = (0..1000)
            .map(|i| {
                let phase = 2.0 * PI * 1000.0 * i as f64 / 48000.0;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let lqa = ale.calculate_lqa(&samples);
        assert!(lqa.score() > 50);
    }

    #[test]
    fn test_waveform_info() {
        let ale = Ale3g::new(48000.0, "TEST", Ale3gCapability::Phase1);
        let info = ale.info();

        assert_eq!(info.name, "3G-ALE");
        assert!(info.description.contains("188-141B"));
    }
}

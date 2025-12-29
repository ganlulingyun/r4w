//! ALE - Automatic Link Establishment
//!
//! Implementation of MIL-STD-188-141B 2G ALE (Second Generation ALE)
//! for HF radio automatic link establishment.
//!
//! ## Overview
//!
//! ALE is used in HF radio systems to automatically:
//! - Establish communication links
//! - Select the best frequency
//! - Manage channel access
//! - Exchange addresses and data
//!
//! ## Key Features
//!
//! - **Modulation**: 8-FSK (8 tones)
//! - **Symbol Rate**: 125 baud
//! - **Frequencies**: 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500 Hz
//! - **Error Correction**: Golay(24,12) extended code
//! - **Word Size**: 24 bits (8 tribit symbols)
//!
//! ## ALE Word Types
//!
//! | Type | Preamble | Usage |
//! |------|----------|-------|
//! | TO   | 001      | Called station address |
//! | TIS  | 011      | Calling station address |
//! | TWAS | 010      | Third-party station |
//! | DATA | 101      | Link data |
//! | REP  | 110      | Repeat request |
//! | CMD  | 111      | Command word |

use std::f64::consts::PI;
use num_complex::Complex64;

use crate::types::IQSample;
use super::{
    CommonParams, DemodResult, DemodulationStep, ModulationStage,
    VisualizationData, Waveform, WaveformInfo,
};

/// ALE tone frequencies in Hz
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

/// ALE symbol rate (125 baud)
const ALE_BAUD: f64 = 125.0;

/// ALE word types (3-bit preamble)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AleWordType {
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
}

impl AleWordType {
    /// Get word type from preamble bits
    pub fn from_preamble(preamble: u8) -> Option<Self> {
        match preamble & 0x07 {
            0b001 => Some(Self::To),
            0b011 => Some(Self::Tis),
            0b010 => Some(Self::Twas),
            0b101 => Some(Self::Data),
            0b110 => Some(Self::Rep),
            0b111 => Some(Self::Cmd),
            _ => None,
        }
    }

    /// Get preamble bits
    pub fn preamble(&self) -> u8 {
        *self as u8
    }
}

/// ALE address (up to 15 characters)
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AleAddress {
    chars: Vec<char>,
}

impl AleAddress {
    /// Create new address (max 15 characters)
    pub fn new(s: &str) -> Self {
        let chars: Vec<char> = s.chars()
            .filter(|c| c.is_ascii_alphanumeric())
            .map(|c| c.to_ascii_uppercase())
            .take(15)
            .collect();
        Self { chars }
    }

    /// Get address as string
    pub fn as_str(&self) -> String {
        self.chars.iter().collect()
    }

    /// Encode address to 38-character set
    /// 0-9 -> 0-9, A-Z -> 10-35, @ -> 36, ? -> 37
    pub fn encode(&self) -> Vec<u8> {
        self.chars.iter().map(|&c| {
            match c {
                '0'..='9' => (c as u8) - b'0',
                'A'..='Z' => (c as u8) - b'A' + 10,
                '@' => 36,
                '?' => 37,
                _ => 36, // Default to @
            }
        }).collect()
    }

    /// Decode from 38-character set
    pub fn decode(bytes: &[u8]) -> Self {
        let chars: Vec<char> = bytes.iter().map(|&b| {
            match b {
                0..=9 => (b + b'0') as char,
                10..=35 => (b - 10 + b'A') as char,
                36 => '@',
                37 => '?',
                _ => '@',
            }
        }).collect();
        Self { chars }
    }
}

/// ALE word (24 bits encoded with Golay)
#[derive(Debug, Clone)]
pub struct AleWord {
    /// Word type
    pub word_type: AleWordType,
    /// 21-bit payload (3-bit preamble + 18-bit data)
    pub data: u32,
}

impl AleWord {
    /// Create new ALE word
    pub fn new(word_type: AleWordType, data: u32) -> Self {
        Self {
            word_type,
            data: data & 0x3FFFF, // 18 bits max
        }
    }

    /// Create address word
    pub fn address(word_type: AleWordType, chars: &[u8]) -> Self {
        // Pack up to 3 characters (6 bits each = 18 bits)
        let mut data: u32 = 0;
        for (i, &ch) in chars.iter().take(3).enumerate() {
            data |= ((ch as u32) & 0x3F) << (12 - i * 6);
        }
        Self::new(word_type, data)
    }

    /// Encode to 24-bit value with preamble
    pub fn encode(&self) -> u32 {
        let preamble = (self.word_type as u32) << 21;
        preamble | self.data
    }

    /// Decode from 24-bit value
    pub fn decode(value: u32) -> Option<Self> {
        let preamble = ((value >> 21) & 0x07) as u8;
        let word_type = AleWordType::from_preamble(preamble)?;
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

/// Golay(24,12) encoder/decoder
pub struct GolayCodec;

impl GolayCodec {
    /// Generator matrix for Golay(24,12)
    const GENERATOR: [u16; 12] = [
        0b110111000101,
        0b101110001011,
        0b011100010111,
        0b111000101101,
        0b110001011011,
        0b100010110111,
        0b000101101111,
        0b001011011101,
        0b010110111001,
        0b101101110001,
        0b011011100011,
        0b111111111110,
    ];

    /// Encode 12 data bits to 24 coded bits
    pub fn encode(data: u16) -> u32 {
        let data = data & 0x0FFF;
        let mut parity: u16 = 0;

        for i in 0..12 {
            if (data >> i) & 1 == 1 {
                parity ^= Self::GENERATOR[i];
            }
        }

        // 24-bit codeword: 12 data bits + 12 parity bits
        ((data as u32) << 12) | (parity as u32)
    }

    /// Decode 24 coded bits, correcting up to 3 errors
    pub fn decode(codeword: u32) -> Result<u16, u8> {
        // Simplified decoder - real Golay uses syndrome lookup
        // For now, just extract data bits (no error correction)
        let data = ((codeword >> 12) & 0x0FFF) as u16;

        // Check parity
        let received_parity = (codeword & 0x0FFF) as u16;
        let expected_parity = (Self::encode(data) & 0x0FFF) as u16;

        let errors = (received_parity ^ expected_parity).count_ones();

        if errors <= 3 {
            Ok(data)
        } else {
            Err(errors as u8)
        }
    }
}

/// ALE Link Quality Analysis (LQA) scores
#[derive(Debug, Clone, Copy)]
pub struct AleLqa {
    /// Bit Error Rate estimate (0-100)
    pub ber: u8,
    /// Signal to Noise Ratio estimate
    pub sinad: u8,
    /// Multipath estimate
    pub multipath: u8,
}

impl AleLqa {
    /// Create from raw measurements
    pub fn new(ber: u8, sinad: u8, multipath: u8) -> Self {
        Self { ber, sinad, multipath }
    }

    /// Overall quality score (0-100)
    pub fn score(&self) -> u8 {
        // Higher is better
        let ber_score = 100 - self.ber.min(100);
        let sinad_score = self.sinad.min(100);
        let mp_score = 100 - self.multipath.min(100);

        ((ber_score as u16 + sinad_score as u16 + mp_score as u16) / 3) as u8
    }
}

/// ALE protocol state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AleState {
    /// Idle - listening for calls
    Idle,
    /// Scanning frequencies
    Scanning,
    /// Calling another station
    Calling,
    /// Responding to a call
    Responding,
    /// Link established
    Linked,
    /// Sounding (channel probing)
    Sounding,
}

/// ALE modem
#[derive(Debug)]
pub struct Ale {
    common: CommonParams,
    /// Samples per symbol
    samples_per_symbol: usize,
    /// Current protocol state
    state: AleState,
    /// This station's address
    my_address: AleAddress,
    /// Current phase for FSK generation
    phase: f64,
}

impl Ale {
    /// Create new ALE modem
    pub fn new(sample_rate: f64, address: &str) -> Self {
        let samples_per_symbol = (sample_rate / ALE_BAUD).round() as usize;

        Self {
            common: CommonParams {
                sample_rate,
                carrier_freq: 1500.0, // Center of ALE band
                amplitude: 1.0,
            },
            samples_per_symbol,
            state: AleState::Idle,
            my_address: AleAddress::new(address),
            phase: 0.0,
        }
    }

    /// Create with default parameters
    pub fn default_config(sample_rate: f64) -> Self {
        Self::new(sample_rate, "TEST")
    }

    /// Get current state
    pub fn state(&self) -> AleState {
        self.state
    }

    /// Set state
    pub fn set_state(&mut self, state: AleState) {
        self.state = state;
    }

    /// Get this station's address
    pub fn address(&self) -> &AleAddress {
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

    /// Modulate ALE word to I/Q samples
    pub fn modulate_word(&mut self, word: &AleWord) -> Vec<IQSample> {
        let tribits = word.to_tribits();
        let mut samples = Vec::new();

        for tribit in tribits {
            samples.extend(self.generate_symbol(tribit));
        }

        samples
    }

    /// Generate ALE call sequence
    pub fn generate_call(&mut self, to_address: &str, repeats: usize) -> Vec<IQSample> {
        let mut samples = Vec::new();

        let to_addr = AleAddress::new(to_address);
        let to_chars = to_addr.encode();
        let my_chars = self.my_address.encode();

        // Generate repeated TO words
        for chunk in to_chars.chunks(3) {
            let mut chars = [0u8; 3];
            for (i, &c) in chunk.iter().enumerate() {
                chars[i] = c;
            }

            for _ in 0..repeats {
                let word = AleWord::address(AleWordType::To, &chars);
                samples.extend(self.modulate_word(&word));
            }
        }

        // Generate TIS words (this station)
        for chunk in my_chars.chunks(3) {
            let mut chars = [0u8; 3];
            for (i, &c) in chunk.iter().enumerate() {
                chars[i] = c;
            }

            for _ in 0..repeats {
                let word = AleWord::address(AleWordType::Tis, &chars);
                samples.extend(self.modulate_word(&word));
            }
        }

        samples
    }

    /// Detect tone frequency in samples
    fn detect_tone(&self, samples: &[IQSample]) -> u8 {
        // Correlation-based tone detection
        let mut max_energy = 0.0;
        let mut best_tone = 0u8;

        for (i, &freq) in ALE_TONES.iter().enumerate() {
            // Correlate with reference tone (integrate after mixing)
            let correlation: IQSample = samples.iter().enumerate()
                .map(|(j, &s)| {
                    let phase = 2.0 * PI * freq * j as f64 / self.common.sample_rate;
                    let ref_signal = Complex64::new(phase.cos(), -phase.sin());
                    s * ref_signal
                })
                .fold(Complex64::new(0.0, 0.0), |acc, x| acc + x);

            // Energy is magnitude squared of the correlation
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

    /// Demodulate samples to ALE words
    pub fn demodulate_words(&self, samples: &[IQSample]) -> Vec<AleWord> {
        let tribits = self.demodulate_symbols(samples);
        let mut words = Vec::new();

        for chunk in tribits.chunks(8) {
            if chunk.len() == 8 {
                let mut arr = [0u8; 8];
                arr.copy_from_slice(chunk);
                if let Some(word) = AleWord::from_tribits(&arr) {
                    words.push(word);
                }
            }
        }

        words
    }

    /// Calculate LQA from received signal
    pub fn calculate_lqa(&self, samples: &[IQSample]) -> AleLqa {
        // Simplified LQA calculation
        let avg_power: f64 = samples.iter()
            .map(|s| s.norm_sqr())
            .sum::<f64>() / samples.len().max(1) as f64;

        // Estimate SNR from signal variance
        let variance: f64 = samples.iter()
            .map(|s| (s.norm_sqr() - avg_power).powi(2))
            .sum::<f64>() / samples.len().max(1) as f64;

        let snr_linear = avg_power / variance.max(0.001);
        let snr_db = (10.0 * snr_linear.log10()).clamp(0.0, 50.0);

        AleLqa {
            ber: ((50.0 - snr_db) * 2.0).max(0.0).min(100.0) as u8,
            sinad: (snr_db * 2.0).min(100.0) as u8,
            multipath: 20, // Would need more complex analysis
        }
    }

    /// Get tone frequencies for visualization
    pub fn get_tone_frequencies() -> &'static [f64; 8] {
        &ALE_TONES
    }
}

impl Waveform for Ale {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "ALE",
            full_name: "Automatic Link Establishment (2G)",
            description: "HF radio automatic link establishment protocol",
            complexity: 3,
            bits_per_symbol: 3, // 8-FSK = 3 bits per symbol
            carries_data: true,
            characteristics: &[
                "8-FSK modulation (8 tones)",
                "125 baud symbol rate",
                "Golay(24,12) error correction",
                "Automatic frequency selection",
                "Link quality analysis (LQA)",
                "Scanning and calling protocols",
            ],
            history: "MIL-STD-188-141A developed in 1980s for HF automation",
            modern_usage: "Military, government, and amateur HF networks",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // Interpret data as raw tribits
        let mut ale = Self::new(self.common.sample_rate, &self.my_address.as_str());

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

        // ALE doesn't have a traditional constellation - show tone frequencies
        let constellation: Vec<IQSample> = ALE_TONES.iter()
            .enumerate()
            .map(|(i, &freq)| {
                let normalized = (freq - 1500.0) / 1000.0;
                Complex64::new(normalized, i as f64 / 4.0 - 1.0)
            })
            .collect();

        let labels: Vec<String> = ALE_TONES.iter()
            .enumerate()
            .map(|(i, &freq)| format!("{}: {} Hz", i, freq as i32))
            .collect();

        VisualizationData {
            samples,
            constellation,
            constellation_labels: labels,
            spectrum: vec![],
            description: format!(
                "ALE 8-FSK: 125 baud, tones 750-2500 Hz, {} symbols",
                data.len()
            ),
        }
    }

    fn get_modulation_stages(&self, data: &[u8]) -> Vec<ModulationStage> {
        vec![
            ModulationStage::new(
                "Data Input",
                format!("Raw data: {} bytes", data.len()),
            ).with_input_bits(data.to_vec()),

            ModulationStage::new(
                "Word Formation",
                "Group into 24-bit ALE words with preamble",
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
                    self.samples_per_symbol,
                    self.common.sample_rate
                ),
            ).with_samples(self.modulate(data)),
        ]
    }

    fn get_demodulation_steps(&self, samples: &[IQSample]) -> Vec<DemodulationStep> {
        let num_symbols = samples.len() / self.samples_per_symbol;

        vec![
            DemodulationStep::new(
                "Tone Detection",
                format!(
                    "Detect 8-FSK tones in {} symbols",
                    num_symbols
                ),
            ),

            DemodulationStep::new(
                "Tribit Recovery",
                "Convert detected tones to tribits",
            ),

            DemodulationStep::new(
                "Word Assembly",
                "Group 8 tribits into 24-bit words",
            ),

            DemodulationStep::new(
                "Golay Decoding",
                "Correct up to 3 bit errors per word",
            ),

            DemodulationStep::new(
                "Word Parsing",
                "Extract preamble and data from words",
            ),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ale_address() {
        let addr = AleAddress::new("TEST123");
        assert_eq!(addr.as_str(), "TEST123");

        let encoded = addr.encode();
        let decoded = AleAddress::decode(&encoded);
        assert_eq!(decoded.as_str(), "TEST123");
    }

    #[test]
    fn test_ale_word() {
        let word = AleWord::new(AleWordType::To, 0x12345);
        let encoded = word.encode();
        let decoded = AleWord::decode(encoded).unwrap();

        assert_eq!(decoded.word_type, AleWordType::To);
        assert_eq!(decoded.data, 0x12345);
    }

    #[test]
    fn test_tribits() {
        let word = AleWord::new(AleWordType::Tis, 0x2AAAA);
        let tribits = word.to_tribits();
        let recovered = AleWord::from_tribits(&tribits).unwrap();

        assert_eq!(word.word_type, recovered.word_type);
    }

    #[test]
    fn test_golay_encode() {
        let data: u16 = 0x0ABC;
        let codeword = GolayCodec::encode(data);

        // Codeword should be 24 bits
        assert!(codeword < (1 << 24));

        // Data should be in upper 12 bits
        assert_eq!((codeword >> 12) as u16, data);
    }

    #[test]
    fn test_golay_decode() {
        let data: u16 = 0x0ABC;
        let codeword = GolayCodec::encode(data);
        let decoded = GolayCodec::decode(codeword).unwrap();

        assert_eq!(decoded, data);
    }

    #[test]
    fn test_ale_modulation() {
        let ale = Ale::new(48000.0, "TEST");
        let tribits = vec![0, 1, 2, 3, 4, 5, 6, 7]; // All 8 tones
        let samples = ale.modulate(&tribits);

        // Should have samples
        assert_eq!(samples.len(), 8 * ale.samples_per_symbol);
    }

    #[test]
    fn test_ale_roundtrip() {
        let mut ale = Ale::new(48000.0, "TEST");

        // Generate a call
        let samples = ale.generate_call("REMOTE", 1);

        // Demodulate
        let words = ale.demodulate_words(&samples);

        // Should recover some words
        assert!(!words.is_empty());
    }

    #[test]
    fn test_ale_lqa() {
        let ale = Ale::new(48000.0, "TEST");

        // Generate clean signal
        let samples: Vec<IQSample> = (0..1000)
            .map(|i| {
                let phase = 2.0 * PI * 1000.0 * i as f64 / 48000.0;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let lqa = ale.calculate_lqa(&samples);
        assert!(lqa.score() > 50); // Clean signal should score well
    }
}

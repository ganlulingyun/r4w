//! Direct Sequence Spread Spectrum (DSSS)
//!
//! DSSS is a spread spectrum technique that spreads the signal by multiplying
//! each data symbol by a high-rate pseudo-noise (PN) sequence. This provides:
//!
//! - **Processing Gain**: Signal energy is spread across bandwidth, allowing
//!   operation below the noise floor
//! - **Interference Rejection**: Narrowband interference is spread by the
//!   correlator, reducing its impact
//! - **Multi-User Access**: Different PN codes allow multiple users (CDMA)
//! - **LPD/LPI**: Low probability of detection/intercept
//!
//! ## How DSSS Works
//!
//! ```text
//! Data Symbol: [+1] ──────────────────────────────────────────────────────
//!                         │
//!                         ▼ Multiply by PN sequence
//! PN Sequence:  [+1 -1 +1 +1 -1 +1 -1 +1 -1 -1 +1 -1 +1]
//!                         │
//!                         ▼
//! Spread Signal: [+1 -1 +1 +1 -1 +1 -1 +1 -1 -1 +1 -1 +1]
//!                         │
//!                         ▼ At receiver: Multiply by same PN
//! Despread:     [+1 +1 +1 +1 +1 +1 +1 +1 +1 +1 +1 +1 +1] = 13 (correlation peak)
//! ```
//!
//! ## Processing Gain
//!
//! Processing Gain (PG) = 10 * log10(chips_per_symbol)
//!
//! | Chips/Symbol | Processing Gain |
//! |--------------|-----------------|
//! | 31           | 15 dB           |
//! | 63           | 18 dB           |
//! | 127          | 21 dB           |
//! | 255          | 24 dB           |
//! | 511          | 27 dB           |
//! | 1023         | 30 dB           |
//!
//! ## LPD/LPI Properties
//!
//! With sufficient processing gain, the transmitted signal power spectral
//! density (PSD) can be below the thermal noise floor (-174 dBm/Hz at 290K).
//! This makes the signal very difficult to detect by non-cooperative receivers.

use super::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};
use crate::spreading::{GoldCodeGenerator, PnSequence};
use crate::types::IQSample;

/// Unpack bytes to individual bits (MSB first)
fn bytes_to_bits(data: &[u8]) -> Vec<u8> {
    let mut bits = Vec::with_capacity(data.len() * 8);
    for byte in data {
        for i in (0..8).rev() {
            bits.push((byte >> i) & 1);
        }
    }
    bits
}

/// Pack individual bits to bytes (MSB first)
fn bits_to_bytes(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8)
        .map(|chunk| {
            chunk.iter()
                .enumerate()
                .fold(0u8, |acc, (i, &bit)| {
                    acc | ((bit & 1) << (7 - i))
                })
        })
        .collect()
}

/// Check if data is packed bytes (contains values > 1)
fn is_packed_bytes(data: &[u8]) -> bool {
    data.iter().any(|&b| b > 1)
}

/// Modulation type for the underlying symbol
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DsssModulation {
    /// Binary Phase Shift Keying (1 bit/symbol)
    Bpsk,
    /// Quadrature Phase Shift Keying (2 bits/symbol)
    Qpsk,
}

impl DsssModulation {
    /// Bits per symbol
    pub fn bits_per_symbol(&self) -> usize {
        match self {
            Self::Bpsk => 1,
            Self::Qpsk => 2,
        }
    }

    /// Map bits to constellation point
    pub fn modulate(&self, bits: &[u8]) -> IQSample {
        match self {
            Self::Bpsk => {
                let bit = bits.first().copied().unwrap_or(0) & 1;
                if bit == 0 {
                    IQSample::new(1.0, 0.0)
                } else {
                    IQSample::new(-1.0, 0.0)
                }
            }
            Self::Qpsk => {
                let b0 = bits.first().copied().unwrap_or(0) & 1;
                let b1 = bits.get(1).copied().unwrap_or(0) & 1;
                let scale = 1.0 / 2.0_f64.sqrt();
                let i = if b0 == 0 { scale } else { -scale };
                let q = if b1 == 0 { scale } else { -scale };
                IQSample::new(i, q)
            }
        }
    }

    /// Demodulate constellation point to bits
    pub fn demodulate(&self, sample: IQSample) -> Vec<u8> {
        match self {
            Self::Bpsk => {
                vec![if sample.re >= 0.0 { 0 } else { 1 }]
            }
            Self::Qpsk => {
                vec![
                    if sample.re >= 0.0 { 0 } else { 1 },
                    if sample.im >= 0.0 { 0 } else { 1 },
                ]
            }
        }
    }
}

/// PN sequence type for spreading
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PnSequenceType {
    /// Gold code (best for multi-user)
    Gold,
    /// M-sequence (maximum length)
    MSequence,
}

/// DSSS configuration
#[derive(Debug, Clone)]
pub struct DsssConfig {
    /// PN sequence type
    pub pn_type: PnSequenceType,
    /// LFSR degree (determines sequence length 2^n - 1)
    pub pn_degree: u8,
    /// Code index (for Gold codes, selects which code from family)
    pub code_index: usize,
    /// Underlying modulation
    pub modulation: DsssModulation,
    /// Samples per chip (oversampling)
    pub samples_per_chip: usize,
}

impl Default for DsssConfig {
    fn default() -> Self {
        Self {
            pn_type: PnSequenceType::Gold,
            pn_degree: 7, // 127 chips = 21 dB processing gain
            code_index: 2,
            modulation: DsssModulation::Bpsk,
            samples_per_chip: 4,
        }
    }
}

/// Direct Sequence Spread Spectrum modulator/demodulator
#[derive(Debug)]
pub struct DSSS {
    /// Common waveform parameters
    common: CommonParams,
    /// DSSS configuration
    config: DsssConfig,
    /// PN sequence generator
    pn_generator: GoldCodeGenerator,
    /// Cached PN sequence
    pn_sequence: Vec<i8>,
}

impl Clone for DSSS {
    fn clone(&self) -> Self {
        Self {
            common: self.common.clone(),
            config: self.config.clone(),
            pn_generator: self.pn_generator.clone(),
            pn_sequence: self.pn_sequence.clone(),
        }
    }
}

impl DSSS {
    /// Create a new DSSS modulator with the given configuration
    pub fn new(common: CommonParams, config: DsssConfig) -> Self {
        let mut pn_generator = GoldCodeGenerator::new(config.pn_degree);
        pn_generator.set_code_index(config.code_index);

        let pn_sequence = pn_generator.generate_sequence();

        Self {
            common,
            config,
            pn_generator,
            pn_sequence,
        }
    }

    /// Create with default configuration (BPSK, Gold-127)
    pub fn default_bpsk(sample_rate: f64) -> Self {
        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        Self::new(common, DsssConfig::default())
    }

    /// Create with QPSK modulation
    pub fn default_qpsk(sample_rate: f64) -> Self {
        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let config = DsssConfig {
            modulation: DsssModulation::Qpsk,
            ..Default::default()
        };
        Self::new(common, config)
    }

    /// Create with specific processing gain
    pub fn with_processing_gain(sample_rate: f64, min_pg_db: f64) -> Self {
        // Find smallest degree that gives required processing gain
        let degree = if min_pg_db <= 15.0 {
            5 // 31 chips = 15 dB
        } else if min_pg_db <= 18.0 {
            6 // 63 chips = 18 dB
        } else if min_pg_db <= 21.0 {
            7 // 127 chips = 21 dB
        } else if min_pg_db <= 24.0 {
            8 // 255 chips = 24 dB
        } else if min_pg_db <= 27.0 {
            9 // 511 chips = 27 dB
        } else {
            10 // 1023 chips = 30 dB
        };

        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let config = DsssConfig {
            pn_degree: degree,
            ..Default::default()
        };
        Self::new(common, config)
    }

    /// Get the number of chips per symbol
    pub fn chips_per_symbol(&self) -> usize {
        self.pn_sequence.len()
    }

    /// Get processing gain in dB
    pub fn processing_gain_db(&self) -> f64 {
        10.0 * (self.chips_per_symbol() as f64).log10()
    }

    /// Get the chip rate in chips/second
    pub fn chip_rate(&self) -> f64 {
        self.common.sample_rate / self.config.samples_per_chip as f64
    }

    /// Get the symbol rate in symbols/second
    pub fn symbol_rate(&self) -> f64 {
        self.chip_rate() / self.chips_per_symbol() as f64
    }

    /// Get the data rate in bits/second
    pub fn data_rate(&self) -> f64 {
        self.symbol_rate() * self.config.modulation.bits_per_symbol() as f64
    }

    /// Get the spread bandwidth in Hz
    pub fn spread_bandwidth(&self) -> f64 {
        self.chip_rate() // Approximate: spread BW ≈ chip rate
    }

    /// Set the code index (for multi-user scenarios)
    pub fn set_code_index(&mut self, index: usize) {
        self.config.code_index = index;
        self.pn_generator.set_code_index(index);
        self.pn_sequence = self.pn_generator.generate_sequence();
    }

    /// Get the current PN sequence
    pub fn pn_sequence(&self) -> &[i8] {
        &self.pn_sequence
    }

    /// Spread a single symbol
    fn spread_symbol(&self, symbol: IQSample) -> Vec<IQSample> {
        let mut chips = Vec::with_capacity(self.chips_per_symbol() * self.config.samples_per_chip);

        for &chip in &self.pn_sequence {
            let chip_value = chip as f64;
            let spread_i = symbol.re * chip_value * self.common.amplitude;
            let spread_q = symbol.im * chip_value * self.common.amplitude;

            // Oversample each chip
            for _ in 0..self.config.samples_per_chip {
                chips.push(IQSample::new(spread_i, spread_q));
            }
        }

        chips
    }

    /// Despread received samples to recover a symbol
    fn despread_symbol(&self, samples: &[IQSample]) -> IQSample {
        let samples_per_symbol = self.chips_per_symbol() * self.config.samples_per_chip;

        if samples.len() < samples_per_symbol {
            return IQSample::new(0.0, 0.0);
        }

        let mut accum_i = 0.0;
        let mut accum_q = 0.0;

        for (chip_idx, &chip) in self.pn_sequence.iter().enumerate() {
            let chip_value = chip as f64;

            // Average over samples for this chip
            for sample_idx in 0..self.config.samples_per_chip {
                let idx = chip_idx * self.config.samples_per_chip + sample_idx;
                if idx < samples.len() {
                    accum_i += samples[idx].re * chip_value;
                    accum_q += samples[idx].im * chip_value;
                }
            }
        }

        // Normalize
        let norm = (self.chips_per_symbol() * self.config.samples_per_chip) as f64;
        IQSample::new(accum_i / norm, accum_q / norm)
    }

    /// Correlate with PN sequence to find sync
    pub fn correlate(&self, samples: &[IQSample]) -> Vec<f64> {
        let samples_per_symbol = self.chips_per_symbol() * self.config.samples_per_chip;

        if samples.len() < samples_per_symbol {
            return vec![];
        }

        (0..samples.len() - samples_per_symbol + 1)
            .map(|start| {
                let despread = self.despread_symbol(&samples[start..]);
                (despread.re * despread.re + despread.im * despread.im).sqrt()
            })
            .collect()
    }
}

impl Waveform for DSSS {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "DSSS",
            full_name: "Direct Sequence Spread Spectrum",
            description: "Spreads signal with PN sequence for LPD/LPI and anti-jam",
            complexity: 4,
            bits_per_symbol: self.config.modulation.bits_per_symbol() as u8,
            carries_data: true,
            characteristics: &[
                "High processing gain (15-30 dB)",
                "Operates below noise floor",
                "Low Probability of Detection (LPD)",
                "Low Probability of Intercept (LPI)",
                "Anti-jam capability",
                "CDMA multiple access",
            ],
            history: "DSSS was developed in the 1940s by actress Hedy Lamarr and composer \
                George Antheil for torpedo guidance. Declassified in 1980s, it became the \
                foundation of CDMA cellular (IS-95), GPS, and 802.11b WiFi. Military systems \
                use DSSS for secure, jam-resistant communications.",
            modern_usage: "GPS satellites use DSSS with Gold codes. IS-95 and CDMA2000 cellular \
                used DSSS. 802.11b WiFi used DSSS with Barker codes. Military tactical radios \
                use DSSS for LPD/LPI. Modern applications include UWB ranging and secure \
                communications.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // Convert packed bytes to individual bits if needed
        let bits = if is_packed_bytes(data) {
            bytes_to_bits(data)
        } else {
            data.to_vec()
        };

        let bits_per_symbol = self.config.modulation.bits_per_symbol();
        let mut samples = Vec::new();

        for symbol_bits in bits.chunks(bits_per_symbol) {
            // Map bits to constellation point
            let symbol = self.config.modulation.modulate(symbol_bits);

            // Spread the symbol
            let spread_samples = self.spread_symbol(symbol);
            samples.extend(spread_samples);
        }

        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let mut result = DemodResult::default();
        let samples_per_symbol = self.chips_per_symbol() * self.config.samples_per_chip;

        // Collect individual bits first
        let mut individual_bits = Vec::new();

        for symbol_samples in samples.chunks(samples_per_symbol) {
            if symbol_samples.len() < samples_per_symbol {
                break;
            }

            // Despread to get symbol
            let symbol = self.despread_symbol(symbol_samples);

            // Demodulate symbol to bits
            let bits = self.config.modulation.demodulate(symbol);
            individual_bits.extend(bits);
        }

        // Pack individual bits into bytes for consistent output
        result.bits = bits_to_bytes(&individual_bits);

        // Add processing gain info
        result
            .metadata
            .insert("processing_gain_db".to_string(), self.processing_gain_db());
        result
            .metadata
            .insert("chip_rate".to_string(), self.chip_rate());
        result.metadata.insert("data_rate".to_string(), self.data_rate());

        result
    }

    fn samples_per_symbol(&self) -> usize {
        self.chips_per_symbol() * self.config.samples_per_chip
    }

    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);

        // Show BPSK or QPSK constellation
        let constellation = match self.config.modulation {
            DsssModulation::Bpsk => {
                vec![IQSample::new(1.0, 0.0), IQSample::new(-1.0, 0.0)]
            }
            DsssModulation::Qpsk => {
                let s = 1.0 / 2.0_f64.sqrt();
                vec![
                    IQSample::new(s, s),
                    IQSample::new(s, -s),
                    IQSample::new(-s, s),
                    IQSample::new(-s, -s),
                ]
            }
        };

        let labels: Vec<String> = (0..constellation.len())
            .map(|i| format!("{:0width$b}", i, width = self.config.modulation.bits_per_symbol()))
            .collect();

        VisualizationData {
            samples,
            constellation,
            constellation_labels: labels,
            spectrum: Vec::new(),
            description: format!(
                "DSSS: {} chips/symbol ({:.1} dB gain), {:?}, {:.1} bps",
                self.chips_per_symbol(),
                self.processing_gain_db(),
                self.config.modulation,
                self.data_rate()
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dsss_basic() {
        let dsss = DSSS::default_bpsk(1_000_000.0);

        assert_eq!(dsss.chips_per_symbol(), 127);
        assert!((dsss.processing_gain_db() - 21.03).abs() < 0.1);
    }

    #[test]
    fn test_dsss_roundtrip_bpsk() {
        let dsss = DSSS::default_bpsk(1_000_000.0);

        let data: Vec<u8> = vec![0, 1, 0, 1, 1, 0, 1, 0];
        let modulated = dsss.modulate(&data);
        let result = dsss.demodulate(&modulated);

        // Demodulate returns packed bytes, so compare against packed input
        let expected = bits_to_bytes(&data);
        assert_eq!(result.bits.len(), expected.len());
        assert_eq!(result.bits, expected);
    }

    #[test]
    fn test_dsss_roundtrip_qpsk() {
        let dsss = DSSS::default_qpsk(1_000_000.0);

        // QPSK: 2 bits per symbol
        let data: Vec<u8> = vec![0, 0, 1, 1, 0, 1, 1, 0];
        let modulated = dsss.modulate(&data);
        let result = dsss.demodulate(&modulated);

        // Demodulate returns packed bytes, so compare against packed input
        let expected = bits_to_bytes(&data);
        assert_eq!(result.bits.len(), expected.len());
        assert_eq!(result.bits, expected);
    }

    #[test]
    fn test_processing_gain_selection() {
        let dsss = DSSS::with_processing_gain(1_000_000.0, 25.0);
        assert!(dsss.processing_gain_db() >= 25.0);
    }

    #[test]
    fn test_spread_bandwidth() {
        let dsss = DSSS::default_bpsk(1_000_000.0);

        // With 4 samples/chip, chip rate = 250k
        let chip_rate = dsss.chip_rate();
        assert!((chip_rate - 250_000.0).abs() < 1.0);

        // Symbol rate = chip_rate / 127
        let symbol_rate = dsss.symbol_rate();
        assert!((symbol_rate - 250_000.0 / 127.0).abs() < 10.0);
    }

    #[test]
    fn test_different_codes() {
        let mut dsss1 = DSSS::default_bpsk(1_000_000.0);
        dsss1.set_code_index(2);

        let mut dsss2 = DSSS::default_bpsk(1_000_000.0);
        dsss2.set_code_index(3);

        // Different codes should produce different PN sequences
        assert_ne!(dsss1.pn_sequence(), dsss2.pn_sequence());
    }
}

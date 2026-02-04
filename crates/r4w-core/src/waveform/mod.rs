//! Waveform Development Kit
//!
//! This module provides a generic framework for implementing and experimenting
//! with different digital waveforms, from simple CW tones to complex spread spectrum.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                      Waveform Trait                             │
//! │  ┌───────────┐  ┌───────────┐  ┌───────────┐  ┌───────────┐     │
//! │  │ modulate  │  │demodulate │  │  params   │  │   info    │     │
//! │  └───────────┘  └───────────┘  └───────────┘  └───────────┘     │
//! └─────────────────────────────────────────────────────────────────┘
//!                              │
//!        ┌─────────────────────┼─────────────────────┐
//!        ▼                     ▼                     ▼
//! ┌─────────────┐       ┌─────────────┐       ┌─────────────┐
//! │     CW      │       │    OOK      │       │    FSK      │
//! │  (Tone)     │       │ (On-Off)    │       │ (Freq Shift)│
//! └─────────────┘       └─────────────┘       └─────────────┘
//!        │                     │                     │
//!        ▼                     ▼                     ▼
//! ┌─────────────┐       ┌─────────────┐       ┌─────────────┐
//! │    PSK      │       │    QAM      │       │    CSS      │
//! │(Phase Shift)│       │ (Quadrature)│       │  (LoRa)     │
//! └─────────────┘       └─────────────┘       └─────────────┘
//! ```
//!
//! ## Usage
//!
//! ```rust,ignore
//! use r4w_core::waveform::{Waveform, WaveformParams, cw::CW, fsk::FSK};
//!
//! // Create a simple CW tone
//! let cw = CW::new(1000.0, 125000.0);  // 1kHz tone at 125kHz sample rate
//! let samples = cw.modulate(&[]);       // CW doesn't encode data
//!
//! // Create FSK modulator
//! let fsk = FSK::new(FSKParams {
//!     deviation: 5000.0,
//!     symbol_rate: 1000.0,
//!     sample_rate: 125000.0,
//! });
//! let samples = fsk.modulate(&[1, 0, 1, 1, 0]);
//! ```

pub mod adsb;
pub mod ale;
pub mod am;      // Analog AM (amplitude modulation for audio)
pub mod ask;     // Digital ASK (amplitude shift keying)
pub mod cw;
pub mod dsss;
pub mod fhss;
pub mod fhss_antijam;
pub mod fm;      // Analog FM (frequency modulation for audio)
pub mod fmcw;
pub mod fsk;     // Digital FSK (frequency shift keying)
pub mod lora;
pub mod ofdm;
pub mod ook;
pub mod ppm;
pub mod psk;
pub mod qam;
pub mod sincgars;  // SINCGARS frequency hopping radio
pub mod havequick; // HAVEQUICK UHF frequency hopping radio
pub mod link16;       // Link-16 tactical data link
pub mod milstd188110; // MIL-STD-188-110 HF modem
pub mod p25;          // APCO P25 digital voice
pub mod stanag4285;
pub mod tetra;       // TETRA European emergency services radio
pub mod dmr;         // DMR Digital Mobile Radio
pub mod ale3g;       // 3G ALE - MIL-STD-188-141B Appendix C
pub mod uwb;
pub mod zigbee;
pub mod gnss;     // GNSS (GPS, GLONASS, Galileo) waveforms
pub mod beacon;   // 121.5/243 MHz emergency distress beacons (ELT, EPIRB, PLB)

use crate::types::IQSample;
use serde::{Deserialize, Serialize};
use std::fmt::Debug;

/// Information about a waveform for display and education
/// Note: Not Deserialize since it contains static references
#[derive(Debug, Clone, Serialize)]
pub struct WaveformInfo {
    /// Short name (e.g., "FSK", "QPSK")
    pub name: &'static str,
    /// Full name (e.g., "Frequency Shift Keying")
    pub full_name: &'static str,
    /// Brief description
    pub description: &'static str,
    /// Complexity level (1-5)
    pub complexity: u8,
    /// Bits per symbol (0 for CW which carries no data)
    pub bits_per_symbol: u8,
    /// Whether this waveform encodes data
    pub carries_data: bool,
    /// Key characteristics for education
    pub characteristics: &'static [&'static str],
    /// Historical background
    pub history: &'static str,
    /// Modern usage and applications
    pub modern_usage: &'static str,
}

/// Common parameters shared by all waveforms
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommonParams {
    /// Sample rate in Hz
    pub sample_rate: f64,
    /// Center/carrier frequency in Hz (relative to baseband)
    pub carrier_freq: f64,
    /// Signal amplitude (0.0 to 1.0)
    pub amplitude: f64,
}

impl Default for CommonParams {
    fn default() -> Self {
        Self {
            sample_rate: 125_000.0,
            carrier_freq: 0.0,  // Baseband by default
            amplitude: 1.0,
        }
    }
}

/// Result of demodulation
#[derive(Debug, Clone)]
pub struct DemodResult {
    /// Decoded bits/symbols
    pub bits: Vec<u8>,
    /// Decoded symbols (for multi-bit schemes)
    pub symbols: Vec<u16>,
    /// Bit error estimate (if known)
    pub ber_estimate: Option<f64>,
    /// Signal quality metrics
    pub snr_estimate: Option<f64>,
    /// Additional waveform-specific info
    pub metadata: std::collections::HashMap<String, f64>,
}

impl Default for DemodResult {
    fn default() -> Self {
        Self {
            bits: Vec::new(),
            symbols: Vec::new(),
            ber_estimate: None,
            snr_estimate: None,
            metadata: std::collections::HashMap::new(),
        }
    }
}

/// Visualization data for educational display
#[derive(Debug, Clone)]
pub struct VisualizationData {
    /// I/Q samples for time domain plot
    pub samples: Vec<IQSample>,
    /// Constellation points (for PSK/QAM)
    pub constellation: Vec<IQSample>,
    /// Labels for constellation points
    pub constellation_labels: Vec<String>,
    /// Frequency spectrum data
    pub spectrum: Vec<f64>,
    /// Description of what's being shown
    pub description: String,
}

/// Represents a stage in the modulation pipeline
#[derive(Debug, Clone)]
pub struct ModulationStage {
    /// Name of this stage (e.g., "Bit Mapping", "Symbol Generation")
    pub name: String,
    /// Description of what happens at this stage
    pub description: String,
    /// Input data to this stage (if applicable)
    pub input_bits: Option<Vec<u8>>,
    /// Output symbols after this stage
    pub output_symbols: Option<Vec<u16>>,
    /// I/Q samples after this stage (if applicable)
    pub samples: Option<Vec<IQSample>>,
    /// Constellation points used at this stage
    pub constellation: Option<Vec<IQSample>>,
}

impl ModulationStage {
    /// Create a new modulation stage
    pub fn new(name: impl Into<String>, description: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            description: description.into(),
            input_bits: None,
            output_symbols: None,
            samples: None,
            constellation: None,
        }
    }

    /// Add input bits to this stage
    pub fn with_input_bits(mut self, bits: Vec<u8>) -> Self {
        self.input_bits = Some(bits);
        self
    }

    /// Add output symbols to this stage
    pub fn with_output_symbols(mut self, symbols: Vec<u16>) -> Self {
        self.output_symbols = Some(symbols);
        self
    }

    /// Add samples to this stage
    pub fn with_samples(mut self, samples: Vec<IQSample>) -> Self {
        self.samples = Some(samples);
        self
    }

    /// Add constellation points to this stage
    pub fn with_constellation(mut self, constellation: Vec<IQSample>) -> Self {
        self.constellation = Some(constellation);
        self
    }
}

/// Represents a step in the demodulation pipeline
#[derive(Debug, Clone)]
pub struct DemodulationStep {
    /// Name of this step (e.g., "Symbol Detection", "Bit Recovery")
    pub name: String,
    /// Description of what happens at this step
    pub description: String,
    /// Input samples to this step
    pub input_samples: Option<Vec<IQSample>>,
    /// Detected symbols at this step
    pub detected_symbols: Option<Vec<u16>>,
    /// Recovered bits at this step
    pub recovered_bits: Option<Vec<u8>>,
    /// Decision regions or thresholds used
    pub decision_info: Option<String>,
    /// Confidence/quality metric
    pub confidence: Option<f64>,
}

impl DemodulationStep {
    /// Create a new demodulation step
    pub fn new(name: impl Into<String>, description: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            description: description.into(),
            input_samples: None,
            detected_symbols: None,
            recovered_bits: None,
            decision_info: None,
            confidence: None,
        }
    }

    /// Add input samples to this step
    pub fn with_input_samples(mut self, samples: Vec<IQSample>) -> Self {
        self.input_samples = Some(samples);
        self
    }

    /// Add detected symbols to this step
    pub fn with_detected_symbols(mut self, symbols: Vec<u16>) -> Self {
        self.detected_symbols = Some(symbols);
        self
    }

    /// Add recovered bits to this step
    pub fn with_recovered_bits(mut self, bits: Vec<u8>) -> Self {
        self.recovered_bits = Some(bits);
        self
    }

    /// Add decision info to this step
    pub fn with_decision_info(mut self, info: impl Into<String>) -> Self {
        self.decision_info = Some(info.into());
        self
    }

    /// Add confidence metric to this step
    pub fn with_confidence(mut self, confidence: f64) -> Self {
        self.confidence = Some(confidence);
        self
    }
}

/// The main waveform trait that all modulation schemes implement
pub trait Waveform: Debug + Send + Sync {
    /// Get information about this waveform
    fn info(&self) -> WaveformInfo;

    /// Get the common parameters
    fn common_params(&self) -> &CommonParams;

    /// Modulate data into I/Q samples
    /// For CW, the data slice can be empty
    fn modulate(&self, data: &[u8]) -> Vec<IQSample>;

    /// Demodulate I/Q samples back to data
    fn demodulate(&self, samples: &[IQSample]) -> DemodResult;

    /// Get the number of samples per symbol
    fn samples_per_symbol(&self) -> usize;

    /// Get visualization data for educational display
    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);
        VisualizationData {
            samples,
            constellation: Vec::new(),
            constellation_labels: Vec::new(),
            spectrum: Vec::new(),
            description: format!("{} modulated signal", self.info().name),
        }
    }

    /// Generate a short demo signal for visualization
    fn generate_demo(&self, duration_ms: f64) -> Vec<IQSample> {
        let num_samples = (self.common_params().sample_rate * duration_ms / 1000.0) as usize;
        let demo_data: Vec<u8> = (0..16).map(|i| i % 2).collect();
        let mut samples = self.modulate(&demo_data);
        samples.truncate(num_samples);
        samples
    }

    /// Get modulation stages for educational display
    /// Default implementation provides a basic two-stage pipeline
    fn get_modulation_stages(&self, data: &[u8]) -> Vec<ModulationStage> {
        let info = self.info();
        let samples = self.modulate(data);
        let vis = self.get_visualization(data);

        let mut stages = Vec::new();

        // Stage 1: Bit/Symbol Mapping
        if info.bits_per_symbol > 0 && info.bits_per_symbol <= 16 {
            // Only process if bits_per_symbol is reasonable (≤16 bits = 65536 symbols max)
            let num_symbols = (data.len() * 8) / info.bits_per_symbol as usize;
            let symbols: Vec<u16> = (0..num_symbols)
                .map(|i| {
                    let bit_start = i * info.bits_per_symbol as usize;
                    let mut sym: u16 = 0;
                    for b in 0..info.bits_per_symbol as usize {
                        let byte_idx = (bit_start + b) / 8;
                        let bit_idx = 7 - ((bit_start + b) % 8);
                        if byte_idx < data.len() {
                            sym = (sym << 1) | ((data[byte_idx] >> bit_idx) & 1) as u16;
                        }
                    }
                    sym
                })
                .collect();

            stages.push(
                ModulationStage::new(
                    "Bit-to-Symbol Mapping",
                    format!(
                        "Group {} bits into symbols. {} possible symbol values.",
                        info.bits_per_symbol,
                        1u32 << info.bits_per_symbol
                    ),
                )
                .with_input_bits(data.to_vec())
                .with_output_symbols(symbols)
                .with_constellation(vis.constellation.clone()),
            );
        } else if info.bits_per_symbol > 16 {
            // For complex multi-carrier waveforms like OFDM, provide a summary
            stages.push(
                ModulationStage::new(
                    "Multi-Carrier Symbol Mapping",
                    format!(
                        "Complex modulation with {} bits per symbol (multi-carrier).",
                        info.bits_per_symbol
                    ),
                )
                .with_input_bits(data.to_vec()),
            );
        }

        // Stage 2: Symbol-to-Sample Generation
        stages.push(
            ModulationStage::new(
                "Symbol-to-Sample Generation",
                format!(
                    "Generate {} I/Q samples per symbol at {:.0} Hz sample rate.",
                    self.samples_per_symbol(),
                    self.common_params().sample_rate
                ),
            )
            .with_samples(samples),
        );

        stages
    }

    /// Get demodulation steps for educational display
    /// Default implementation provides basic demodulation info
    fn get_demodulation_steps(&self, samples: &[IQSample]) -> Vec<DemodulationStep> {
        let info = self.info();
        let result = self.demodulate(samples);

        let mut steps = Vec::new();

        // Step 1: Sample Processing
        steps.push(
            DemodulationStep::new(
                "Sample Processing",
                format!(
                    "Process {} I/Q samples, {} samples per symbol.",
                    samples.len(),
                    self.samples_per_symbol()
                ),
            )
            .with_input_samples(samples.to_vec()),
        );

        // Step 2: Symbol Detection
        if !result.symbols.is_empty() {
            steps.push(
                DemodulationStep::new(
                    "Symbol Detection",
                    format!(
                        "Detect {} symbols using {} decision.",
                        result.symbols.len(),
                        info.name
                    ),
                )
                .with_detected_symbols(result.symbols.clone())
                .with_confidence(result.snr_estimate.unwrap_or(0.0)),
            );
        }

        // Step 3: Bit Recovery
        if !result.bits.is_empty() {
            steps.push(
                DemodulationStep::new(
                    "Bit Recovery",
                    format!(
                        "Recover {} bits from detected symbols.",
                        result.bits.len()
                    ),
                )
                .with_recovered_bits(result.bits.clone()),
            );
        }

        steps
    }
}

/// Factory for creating waveforms by name
pub struct WaveformFactory;

impl WaveformFactory {
    /// List all available waveforms
    pub fn list() -> Vec<&'static str> {
        vec![
            "CW", "OOK", "PPM", "ADS-B",
            // Analog modulation
            "AM-Broadcast", "FM-Broadcast", "NBFM",
            // Digital amplitude modulation
            "ASK", "4-ASK",
            // Digital frequency modulation
            "BFSK", "4-FSK",
            // Phase modulation
            "BPSK", "QPSK", "8-PSK",
            // Quadrature amplitude modulation
            "16-QAM", "64-QAM", "256-QAM",
            "OFDM",
            // Spread spectrum
            "DSSS", "DSSS-QPSK",
            "FHSS",
            // LoRa chirp spread spectrum
            "LoRa", "LoRa-SF7", "LoRa-SF12",
            "Zigbee", "UWB", "FMCW",
            // HF/Military/Public Safety
            "STANAG-4285", "ALE", "3G-ALE", "SINCGARS", "HAVEQUICK", "Link-16", "MIL-STD-188-110", "P25",
            // Professional Mobile Radio (PMR)
            "TETRA", "DMR",
            // GNSS (Global Navigation Satellite Systems)
            "GPS-L1CA", "GPS-L5", "GLONASS-L1OF", "Galileo-E1",
            // Emergency beacons
            "ELT-121.5", "EPIRB-121.5", "PLB-121.5", "Beacon-243",
        ]
    }

    /// Create a waveform by name with default parameters
    pub fn create(name: &str, sample_rate: f64) -> Option<Box<dyn Waveform>> {
        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };

        match name.to_uppercase().replace("-", "").replace("_", "").as_str() {
            "CW" => Some(Box::new(cw::CW::new(common, 1000.0))),
            // Analog modulation (audio/voice)
            "AM" | "AMBROADCAST" => Some(Box::new(am::AM::broadcast(sample_rate, 1000.0))),
            "FM" | "FMBROADCAST" | "WBFM" => Some(Box::new(fm::FM::broadcast(sample_rate, 1000.0))),
            "NBFM" => Some(Box::new(fm::FM::narrowband(sample_rate, 1000.0))),
            // Digital amplitude modulation (ASK)
            "ASK" => Some(Box::new(ask::ASK::new_binary(common, 1000.0, 1000.0))),
            "4ASK" | "PAM4" => Some(Box::new(ask::ASK::new_4ask(common, 1000.0, 1000.0))),
            // OOK
            "OOK" => Some(Box::new(ook::OOK::new(common, 1000.0))),
            // PPM
            "PPM" => Some(Box::new(ppm::PPM::new(sample_rate, 1000.0, ppm::PpmVariant::Standard))),
            "ADSB" => Some(Box::new(ppm::PPM::adsb(sample_rate))),
            // FSK (symbol_rate=500, deviation=500 gives h=2.0 and 20 samples/symbol)
            "BFSK" | "FSK" => Some(Box::new(fsk::FSK::new_bfsk(common, 500.0, 500.0))),
            "4FSK" => Some(Box::new(fsk::FSK::new_4fsk(common, 500.0, 500.0))),
            // PSK
            "BPSK" => Some(Box::new(psk::PSK::new_bpsk(common, 1000.0))),
            "QPSK" => Some(Box::new(psk::PSK::new_qpsk(common, 1000.0))),
            "8PSK" => Some(Box::new(psk::PSK::new_8psk(common, 1000.0))),
            // QAM
            "16QAM" | "QAM16" => Some(Box::new(qam::QAM::new_16qam(common, 1000.0))),
            "64QAM" | "QAM64" => Some(Box::new(qam::QAM::new_64qam(common, 1000.0))),
            "256QAM" | "QAM256" => Some(Box::new(qam::QAM::new_256qam(common, 1000.0))),
            // OFDM
            "OFDM" => Some(Box::new(ofdm::OFDM::simple(sample_rate))),
            // DSSS (Spread Spectrum - LPD/LPI)
            "DSSS" => Some(Box::new(dsss::DSSS::default_bpsk(sample_rate))),
            "DSSSQPSK" => Some(Box::new(dsss::DSSS::default_qpsk(sample_rate))),
            // FHSS (Frequency Hopping - LPD/LPI)
            "FHSS" => Some(Box::new(fhss::FHSS::default_config(sample_rate))),
            // LoRa (Chirp Spread Spectrum)
            "LORA" | "CSS" => Some(Box::new(lora::LoRa::default_config(sample_rate))),
            "LORASF7" => Some(Box::new(lora::LoRa::sf7(sample_rate))),
            "LORASF12" => Some(Box::new(lora::LoRa::sf12(sample_rate))),
            // Zigbee (IEEE 802.15.4 O-QPSK DSSS)
            "ZIGBEE" | "802154" => Some(Box::new(zigbee::Zigbee::standard(sample_rate))),
            // UWB Impulse Radio
            "UWB" | "UWBIR" => Some(Box::new(uwb::UwbIr::ieee_802_15_4a(sample_rate))),
            // FMCW Radar
            "FMCW" => Some(Box::new(fmcw::Fmcw::with_defaults(sample_rate))),
            // HF/Military
            "STANAG4285" | "STANAG" => Some(Box::new(stanag4285::Stanag4285::default_mode(sample_rate))),
            "ALE" => Some(Box::new(ale::Ale::default_config(sample_rate))),
            "SINCGARS" => {
                sincgars::SincgarsBuilder::simulator()
                    .build()
                    .ok()
                    .map(|s| Box::new(s) as Box<dyn Waveform>)
            }
            "HAVEQUICK" | "HQ" => {
                havequick::HavequickBuilder::simulator()
                    .with_sample_rate(sample_rate)
                    .build()
                    .ok()
                    .map(|h| Box::new(h) as Box<dyn Waveform>)
            }
            "LINK16" | "TADILJ" | "MIDS" | "JTIDS" => {
                link16::Link16Builder::simulator()
                    .with_sample_rate(sample_rate)
                    .build()
                    .ok()
                    .map(|l| Box::new(l) as Box<dyn Waveform>)
            }
            "MILSTD188110" | "188110" | "MIL188110" => {
                Some(Box::new(milstd188110::MilStd188110::default_mode(sample_rate)))
            }
            "P25" | "APCO25" | "APCOP25" => {
                Some(Box::new(p25::P25::phase1_c4fm(sample_rate)))
            }
            "P25PHASE2" | "P25P2" => {
                Some(Box::new(p25::P25::phase2(sample_rate)))
            }
            // Professional Mobile Radio (PMR)
            "TETRA" => Some(Box::new(tetra::Tetra::tmo(sample_rate))),
            "TETRADMO" => Some(Box::new(tetra::Tetra::dmo(sample_rate))),
            "DMR" | "DMRTIER2" => Some(Box::new(dmr::Dmr::tier2(sample_rate))),
            "DMRTIER3" => Some(Box::new(dmr::Dmr::tier3(sample_rate))),
            "DMRDIRECT" => Some(Box::new(dmr::Dmr::direct(sample_rate))),
            // 3G ALE (MIL-STD-188-141B Appendix C)
            "3GALE" | "ALE3G" | "MILSTD188141B" => Some(Box::new(ale3g::Ale3g::default_config(sample_rate))),
            "3GALEAMD" => Some(Box::new(ale3g::Ale3g::with_amd(sample_rate, "TEST"))),
            // GNSS waveforms
            "GPSL1CA" | "GPSL1" | "GPSCA" => Some(Box::new(gnss::GpsL1Ca::default_config(sample_rate))),
            "GPSL5" => Some(Box::new(gnss::GpsL5::default_config(sample_rate))),
            "GLONASSL1OF" | "GLONASS" => Some(Box::new(gnss::GlonassL1of::default_config(sample_rate))),
            "GALILEOE1" | "GALILEO" | "GAL" => Some(Box::new(gnss::GalileoE1::default_config(sample_rate))),
            // Emergency beacons
            "ELT" | "ELT121.5" | "ELT1215" => Some(Box::new(beacon::Beacon::elt(sample_rate))),
            "EPIRB" | "EPIRB121.5" | "EPIRB1215" => Some(Box::new(beacon::Beacon::epirb(sample_rate))),
            "PLB" | "PLB121.5" | "PLB1215" => Some(Box::new(beacon::Beacon::plb(sample_rate))),
            "BEACON243" | "MILITARY243" | "MIL243" => Some(Box::new(beacon::Beacon::military(sample_rate))),
            name if name.starts_with("GPSL1CAPRN") => {
                let prn_str = &name[10..];
                prn_str.parse::<u8>().ok()
                    .filter(|&p| p >= 1 && p <= 32)
                    .map(|p| Box::new(gnss::GpsL1Ca::new(sample_rate, p)) as Box<dyn Waveform>)
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_factory_list() {
        let waveforms = WaveformFactory::list();
        assert!(waveforms.contains(&"CW"));
        assert!(waveforms.contains(&"BPSK"));
    }

    #[test]
    fn test_factory_create() {
        let cw = WaveformFactory::create("CW", 125000.0);
        assert!(cw.is_some());
        assert_eq!(cw.unwrap().info().name, "CW");
    }
}

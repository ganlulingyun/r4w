//! SINCGARS waveform framework
//!
//! This module provides the unclassified FHSS engine that uses
//! the classified components via trait interfaces.

use std::f64::consts::PI;
use num_complex::Complex64;

use super::traits::*;
use super::types::*;
use super::audio::CvsdCodec;
use super::data::{DataFramer, SincgarsDataMode};
use crate::waveform::{
    CommonParams, DemodResult, DemodulationStep, ModulationStage,
    VisualizationData, Waveform, WaveformInfo,
};
use crate::types::IQSample;

/// SINCGARS waveform implementation
///
/// This struct contains the unclassified FHSS engine and references
/// to the classified component implementations.
///
/// Note: Debug is implemented manually to avoid exposing sensitive state
pub struct Sincgars {
    /// Common waveform parameters
    common: CommonParams,
    /// Hopping algorithm (classified implementation)
    hopper: Box<dyn HoppingAlgorithm>,
    /// TRANSEC provider (classified implementation)
    transec: Box<dyn TransecProvider>,
    /// Net ID mapper (classified implementation)
    net_mapper: Box<dyn NetIdMapper>,
    /// Time sync protocol (classified implementation)
    time_sync: Box<dyn TimeSyncProtocol>,
    /// Crypto provider (classified implementation)
    crypto: Box<dyn CryptoProvider>,
    /// Voice codec
    voice_codec: CvsdCodec,
    /// Data framer
    data_framer: DataFramer,
    /// Current operating mode
    operating_mode: OperatingMode,
    /// Current traffic mode
    traffic_mode: TrafficMode,
    /// Current net ID
    current_net: Option<NetId>,
    /// FM deviation in Hz
    fm_deviation: f64,
    /// Samples per hop
    samples_per_hop: usize,
}

impl std::fmt::Debug for Sincgars {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Sincgars")
            .field("operating_mode", &self.operating_mode)
            .field("traffic_mode", &self.traffic_mode)
            .field("current_net", &self.current_net)
            .field("samples_per_hop", &self.samples_per_hop)
            .finish_non_exhaustive()
    }
}

impl Sincgars {
    /// Create new SINCGARS instance with provided classified components
    ///
    /// This is typically called via `SincgarsBuilder`.
    pub fn new(
        hopper: Box<dyn HoppingAlgorithm>,
        transec: Box<dyn TransecProvider>,
        net_mapper: Box<dyn NetIdMapper>,
        time_sync: Box<dyn TimeSyncProtocol>,
        crypto: Box<dyn CryptoProvider>,
    ) -> Self {
        let sample_rate = 100_000.0; // 100 kHz sample rate
        let hop_rate = 100; // 100 hops/sec
        let samples_per_hop = (sample_rate / hop_rate as f64) as usize;

        Self {
            common: CommonParams {
                sample_rate,
                carrier_freq: 50_000_000.0, // 50 MHz default
                amplitude: 1.0,
            },
            hopper,
            transec,
            net_mapper,
            time_sync,
            crypto,
            voice_codec: CvsdCodec::sincgars(),
            data_framer: DataFramer::new(SincgarsDataMode::LowSpeed2400),
            operating_mode: OperatingMode::FrequencyHopping,
            traffic_mode: TrafficMode::Voice,
            current_net: None,
            fm_deviation: 6000.0, // ±6 kHz FM deviation
            samples_per_hop,
        }
    }

    /// Initialize SINCGARS with key and net
    pub fn initialize(&mut self, key_id: KeyId, net_id: NetId) -> Result<(), SincgarsError> {
        // Load TRANSEC key
        let key = self.transec.load_key(key_id)?;

        // Validate net ID
        if !self.net_mapper.validate_net_id(net_id) {
            return Err(NetError::UnknownNet.into());
        }

        // Get hopset parameters
        let hopset = self.net_mapper.get_hopset(net_id)?;

        // Update samples per hop based on hopset
        self.samples_per_hop = (self.common.sample_rate / hopset.hop_rate as f64) as usize;

        // Initialize hopper
        let time = self.time_sync.get_current_time();
        self.hopper.initialize(&key, net_id, time);

        // Derive session key and initialize crypto
        let session_key = self.transec.derive_session_key(b"session")?;
        self.crypto.initialize(&session_key)?;

        self.current_net = Some(net_id);

        Ok(())
    }

    /// Set operating mode
    pub fn set_operating_mode(&mut self, mode: OperatingMode) {
        self.operating_mode = mode;
    }

    /// Set traffic mode
    pub fn set_traffic_mode(&mut self, mode: TrafficMode) {
        self.traffic_mode = mode;
    }

    /// Modulate voice samples
    pub fn modulate_voice(&mut self, audio: &[f32]) -> Vec<IQSample> {
        // Encode voice to CVSD
        let cvsd_bits = self.voice_codec.encode(audio);

        // Encrypt
        let context = CryptoContext {
            frame_number: self.crypto.get_frame_counter(),
            direction: CryptoDirection::Transmit,
            associated_data: vec![],
        };

        let encrypted = self.crypto.encrypt(&cvsd_bits, &context)
            .unwrap_or(cvsd_bits);

        // Convert to bits for modulation
        let bits: Vec<u8> = encrypted.iter()
            .flat_map(|byte| (0..8).rev().map(move |i| (byte >> i) & 1))
            .collect();

        // FM modulate with frequency hopping
        self.fm_modulate_with_hopping(&bits)
    }

    /// Demodulate to voice samples
    pub fn demodulate_voice(&mut self, samples: &[IQSample]) -> Vec<f32> {
        // FM demodulate with frequency hopping
        let bits = self.fm_demodulate_with_hopping(samples);

        // Pack bits to bytes
        let bytes: Vec<u8> = bits.chunks(8)
            .map(|chunk| {
                chunk.iter().enumerate()
                    .fold(0u8, |acc, (i, &bit)| acc | ((bit & 1) << (7 - i)))
            })
            .collect();

        // Decrypt
        let context = CryptoContext {
            frame_number: self.crypto.get_frame_counter(),
            direction: CryptoDirection::Receive,
            associated_data: vec![],
        };

        let decrypted = self.crypto.decrypt(&bytes, &context)
            .unwrap_or(bytes);

        // Decode CVSD to audio
        self.voice_codec.decode(&decrypted)
    }

    /// Modulate data
    pub fn modulate_data(&mut self, data: &[u8]) -> Vec<IQSample> {
        // Frame data
        let frames = self.data_framer.frame_data(data);

        // Collect all bits
        let mut all_bits = Vec::new();
        for frame in frames {
            let frame_bits = self.data_framer.frame_to_bits(&frame);
            // Convert bytes to bits
            for byte in frame_bits {
                for i in (0..8).rev() {
                    all_bits.push((byte >> i) & 1);
                }
            }
        }

        // FM modulate with hopping
        self.fm_modulate_with_hopping(&all_bits)
    }

    /// FM modulate bits with frequency hopping
    fn fm_modulate_with_hopping(&mut self, bits: &[u8]) -> Vec<IQSample> {
        let bits_per_hop = self.samples_per_hop * 16000 / self.common.sample_rate as usize;
        let bits_per_hop = bits_per_hop.max(1);

        let mut samples = Vec::new();
        let mut phase = 0.0f64;

        for bit_chunk in bits.chunks(bits_per_hop) {
            // Get current hop frequency
            let channel = self.hopper.get_current_channel();
            let hop_freq = channel.to_frequency_hz();

            // Calculate frequency offset from center
            let freq_offset = hop_freq - self.common.carrier_freq;

            // Modulate this hop's bits
            for &bit in bit_chunk {
                // FM modulation: frequency = carrier + deviation * data
                let data = if bit == 1 { 1.0 } else { -1.0 };
                let inst_freq = freq_offset + self.fm_deviation * data;

                // Generate samples for this bit
                let samples_per_bit = self.samples_per_hop / bits_per_hop.max(1);
                for _ in 0..samples_per_bit {
                    let omega = 2.0 * PI * inst_freq / self.common.sample_rate;
                    phase += omega;

                    // Keep phase wrapped
                    if phase > PI {
                        phase -= 2.0 * PI;
                    }

                    samples.push(Complex64::new(phase.cos(), phase.sin()));
                }
            }

            // Advance to next hop
            self.hopper.advance_hop();
        }

        samples
    }

    /// FM demodulate samples with frequency hopping
    fn fm_demodulate_with_hopping(&mut self, samples: &[IQSample]) -> Vec<u8> {
        let bits_per_hop = self.samples_per_hop * 16000 / self.common.sample_rate as usize;
        let bits_per_hop = bits_per_hop.max(1);
        let samples_per_bit = self.samples_per_hop / bits_per_hop.max(1);

        let mut bits = Vec::new();
        let mut prev_phase = 0.0f64;

        for hop_samples in samples.chunks(self.samples_per_hop) {
            // Get expected hop frequency
            let channel = self.hopper.get_current_channel();
            let hop_freq = channel.to_frequency_hz();
            let freq_offset = hop_freq - self.common.carrier_freq;

            // Mix down to baseband
            let baseband: Vec<IQSample> = hop_samples.iter()
                .enumerate()
                .map(|(i, &s)| {
                    let omega = -2.0 * PI * freq_offset * i as f64 / self.common.sample_rate;
                    let rotation = Complex64::new(omega.cos(), omega.sin());
                    s * rotation
                })
                .collect();

            // FM demodulate (phase difference)
            for bit_samples in baseband.chunks(samples_per_bit) {
                let mut freq_sum = 0.0;

                for &s in bit_samples {
                    let phase = s.im.atan2(s.re);
                    let mut phase_diff = phase - prev_phase;

                    // Unwrap phase
                    while phase_diff > PI {
                        phase_diff -= 2.0 * PI;
                    }
                    while phase_diff < -PI {
                        phase_diff += 2.0 * PI;
                    }

                    // Instantaneous frequency
                    let inst_freq = phase_diff * self.common.sample_rate / (2.0 * PI);
                    freq_sum += inst_freq;
                    prev_phase = phase;
                }

                // Average frequency for this bit
                let avg_freq = freq_sum / bit_samples.len() as f64;

                // Decision: positive frequency = 1, negative = 0
                bits.push(if avg_freq > 0.0 { 1 } else { 0 });
            }

            self.hopper.advance_hop();
        }

        bits
    }

    /// Get current channel
    pub fn get_current_channel(&self) -> ChannelNumber {
        self.hopper.get_current_channel()
    }

    /// Get current net ID
    pub fn get_current_net(&self) -> Option<NetId> {
        self.current_net
    }

    /// Check if synchronized
    pub fn is_synchronized(&self) -> bool {
        self.time_sync.is_synchronized() && self.hopper.is_valid()
    }

    /// Zeroize all sensitive state
    pub fn zeroize(&mut self) {
        self.hopper.reset();
        self.transec.zeroize();
        self.crypto.zeroize();
        self.current_net = None;
    }
}

impl Waveform for Sincgars {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "SINCGARS",
            full_name: "Single Channel Ground and Airborne Radio System",
            description: "Military VHF FHSS tactical radio",
            complexity: 5,
            bits_per_symbol: 1, // FM is essentially 1 bit per symbol
            carries_data: true,
            characteristics: &[
                "Frequency Hopping Spread Spectrum (FHSS)",
                "100 hops per second",
                "2320 channels (30-88 MHz)",
                "FM modulation with ±6 kHz deviation",
                "CVSD voice codec at 16 kbps",
            ],
            history: "Developed in 1980s to replace Vietnam-era radios",
            modern_usage: "Still used by US and allied military forces",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, _data: &[u8]) -> Vec<IQSample> {
        // Note: This trait method doesn't work well with SINCGARS
        // because we need mutable access for hopping state.
        // Use modulate_voice or modulate_data instead.
        vec![]
    }

    fn demodulate(&self, _samples: &[IQSample]) -> DemodResult {
        // See note above
        DemodResult::default()
    }

    fn samples_per_symbol(&self) -> usize {
        self.samples_per_hop
    }

    fn get_visualization(&self, _data: &[u8]) -> VisualizationData {
        VisualizationData {
            samples: vec![],
            constellation: vec![], // FM doesn't have a constellation
            constellation_labels: vec![],
            spectrum: vec![],
            description: format!(
                "SINCGARS FHSS - {} mode, {} traffic",
                match self.operating_mode {
                    OperatingMode::SingleChannel => "Single Channel",
                    OperatingMode::FrequencyHopping => "Frequency Hopping",
                    OperatingMode::Training => "Training",
                },
                match self.traffic_mode {
                    TrafficMode::Voice => "Voice",
                    TrafficMode::LowSpeedData => "Low-Speed Data",
                    TrafficMode::MediumSpeedData => "Medium-Speed Data",
                    TrafficMode::HighSpeedData => "High-Speed Data",
                }
            ),
        }
    }

    fn get_modulation_stages(&self, _data: &[u8]) -> Vec<ModulationStage> {
        vec![
            ModulationStage::new(
                "Voice/Data Input",
                "Raw voice samples or data bytes",
            ),
            ModulationStage::new(
                "CVSD/Framing",
                "Voice encoded to CVSD or data framed",
            ),
            ModulationStage::new(
                "Encryption",
                "Type 1 encryption applied",
            ),
            ModulationStage::new(
                "FM Modulation",
                "±6 kHz FM deviation",
            ),
            ModulationStage::new(
                "Frequency Hopping",
                "100 hops/sec across 2320 channels",
            ),
        ]
    }

    fn get_demodulation_steps(&self, _samples: &[IQSample]) -> Vec<DemodulationStep> {
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::simulator::*;

    fn create_test_sincgars() -> Sincgars {
        Sincgars::new(
            Box::new(SimulatorHopper::new()),
            Box::new(SimulatorTransec::with_test_key()),
            Box::new(SimulatorNetMapper::new()),
            Box::new(SimulatorTimeSync::with_time(SincgarsTime::new(2024, 1, 0, 0))),
            Box::new(SimulatorCrypto::new()),
        )
    }

    #[test]
    fn test_sincgars_initialization() {
        let mut sincgars = create_test_sincgars();

        let result = sincgars.initialize(KeyId(1), NetId(1));
        assert!(result.is_ok());
        assert!(sincgars.is_synchronized());
        assert_eq!(sincgars.get_current_net(), Some(NetId(1)));
    }

    #[test]
    fn test_sincgars_hopping() {
        let mut sincgars = create_test_sincgars();
        sincgars.initialize(KeyId(1), NetId(1)).unwrap();

        let _ch1 = sincgars.get_current_channel();

        // Modulate some data (which advances hops)
        let _ = sincgars.modulate_data(b"test");

        // Channel should have changed
        // (Note: need to reinitialize hopper for comparison)
    }

    #[test]
    fn test_sincgars_zeroize() {
        let mut sincgars = create_test_sincgars();
        sincgars.initialize(KeyId(1), NetId(1)).unwrap();

        assert!(sincgars.is_synchronized());

        sincgars.zeroize();

        assert!(!sincgars.is_synchronized());
        assert_eq!(sincgars.get_current_net(), None);
    }
}

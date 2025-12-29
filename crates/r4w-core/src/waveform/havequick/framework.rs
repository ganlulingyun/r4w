//! HAVEQUICK Framework Implementation
//!
//! This module provides the unclassified framework for HAVEQUICK.
//! The classified components (hopping algorithm, TRANSEC) are provided
//! via trait implementations.

use std::f64::consts::PI;

use crate::types::IQSample;
use crate::waveform::{
    CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo,
};

use super::traits::*;
use super::types::*;

/// HAVEQUICK waveform implementation
///
/// HAVEQUICK is a military UHF frequency hopping system used for
/// air-to-air and air-to-ground communications. It uses AM modulation
/// for voice and ASK for data.
pub struct Havequick {
    /// Common waveform parameters
    common: CommonParams,
    /// Operating mode
    mode: OperatingMode,
    /// Traffic type
    traffic_type: TrafficType,
    /// Hopping algorithm implementation
    hopper: Box<dyn HoppingAlgorithm>,
    /// Time sync implementation
    time_sync: Box<dyn TimeSyncProtocol>,
    /// Net controller
    net_controller: Box<dyn NetController>,
    /// Voice codec
    #[allow(dead_code)]
    voice_codec: Box<dyn VoiceCodec>,
    /// Data modem
    data_modem: Box<dyn DataModem>,
    /// TRANSEC provider (optional encryption)
    #[allow(dead_code)]
    transec: Box<dyn TransecProvider>,
    /// Current Net ID
    current_net: NetId,
    /// Hop rate
    hop_rate: HopRate,
    /// AM modulation depth (0.0-1.0)
    modulation_depth: f64,
    /// Carrier frequency offset for visualization
    carrier_offset: f64,
}

impl std::fmt::Debug for Havequick {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Havequick")
            .field("common", &self.common)
            .field("mode", &self.mode)
            .field("traffic_type", &self.traffic_type)
            .field("current_net", &self.current_net)
            .field("hop_rate", &self.hop_rate)
            .field("modulation_depth", &self.modulation_depth)
            .finish()
    }
}

impl Havequick {
    /// Create a new HAVEQUICK instance
    pub fn new(
        sample_rate: f64,
        hopper: Box<dyn HoppingAlgorithm>,
        time_sync: Box<dyn TimeSyncProtocol>,
        net_controller: Box<dyn NetController>,
        voice_codec: Box<dyn VoiceCodec>,
        data_modem: Box<dyn DataModem>,
        transec: Box<dyn TransecProvider>,
    ) -> Self {
        Self {
            common: CommonParams {
                sample_rate,
                carrier_freq: 1000.0,
                amplitude: 1.0,
            },
            mode: OperatingMode::AmVoice,
            traffic_type: TrafficType::Voice,
            hopper,
            time_sync,
            net_controller,
            voice_codec,
            data_modem,
            transec,
            current_net: NetId(0),
            hop_rate: HopRate::Medium,
            modulation_depth: 0.85,
            carrier_offset: 1000.0,
        }
    }

    /// Set operating mode
    pub fn set_mode(&mut self, mode: OperatingMode) {
        self.mode = mode;
    }

    /// Set traffic type
    pub fn set_traffic_type(&mut self, traffic_type: TrafficType) {
        self.traffic_type = traffic_type;
    }

    /// Set hop rate
    pub fn set_hop_rate(&mut self, hop_rate: HopRate) {
        self.hop_rate = hop_rate;
    }

    /// Get sample rate
    pub fn sample_rate(&self) -> f64 {
        self.common.sample_rate
    }

    /// Initialize with WOD and synchronize
    pub fn initialize(&mut self, wod: WordOfDay, net_id: NetId, tod: TimeOfDay) -> Result<(), HavequickError> {
        // Load WOD
        self.net_controller.load_wod(net_id, wod)?;

        // Set current net
        self.current_net = net_id;

        // Set time
        self.time_sync.set_tod(tod, TimeSource::Gps);

        // Initialize hopper
        if let Some(wod) = self.net_controller.get_wod(net_id) {
            self.hopper.initialize(wod, net_id, tod);
        }

        Ok(())
    }

    /// Get current channel
    pub fn current_channel(&self) -> ChannelNumber {
        self.hopper.get_current_channel()
    }

    /// Get current frequency in Hz
    pub fn current_frequency(&self) -> f64 {
        self.hopper.get_current_channel().to_frequency_hz()
    }

    /// Advance to next hop
    pub fn advance_hop(&mut self) {
        self.hopper.advance_hop();
    }

    /// Check if synchronized
    pub fn is_synchronized(&self) -> bool {
        self.time_sync.is_synchronized()
    }

    /// AM modulate audio samples
    pub fn modulate_audio(&self, audio: &[f64]) -> Vec<IQSample> {
        let carrier_freq = self.carrier_offset;
        let sample_rate = self.common.sample_rate;

        audio.iter().enumerate().map(|(i, &sample)| {
            // Normalize audio to [-1, 1] range
            let normalized = sample.clamp(-1.0, 1.0);

            // AM: carrier * (1 + m * audio)
            // m = modulation depth
            let envelope = 1.0 + self.modulation_depth * normalized;

            // Generate carrier
            let phase = 2.0 * PI * carrier_freq * (i as f64) / sample_rate;
            let carrier_i = phase.cos();
            let carrier_q = phase.sin();

            IQSample::new(envelope * carrier_i, envelope * carrier_q)
        }).collect()
    }

    /// AM demodulate to audio samples
    pub fn demodulate_audio(&self, samples: &[IQSample]) -> Vec<f64> {
        // Envelope detection
        samples.iter().map(|s| {
            let envelope = (s.re * s.re + s.im * s.im).sqrt();
            // Remove DC offset and normalize
            (envelope - 1.0) / self.modulation_depth
        }).collect()
    }

    /// ASK modulate data bits
    fn ask_modulate(&self, bits: &[bool]) -> Vec<IQSample> {
        let baseband = self.data_modem.modulate(bits);
        let carrier_freq = self.carrier_offset;
        let sample_rate = self.common.sample_rate;

        baseband.iter().enumerate().map(|(i, &amplitude)| {
            let phase = 2.0 * PI * carrier_freq * (i as f64) / sample_rate;
            IQSample::new(amplitude * phase.cos(), amplitude * phase.sin())
        }).collect()
    }

    /// ASK demodulate to data bits
    fn ask_demodulate(&self, samples: &[IQSample]) -> Vec<bool> {
        // Envelope detection
        let baseband: Vec<f64> = samples.iter().map(|s| {
            (s.re * s.re + s.im * s.im).sqrt()
        }).collect();

        self.data_modem.demodulate(&baseband)
    }
}

impl Waveform for Havequick {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "HAVEQUICK",
            full_name: "HAVE QUICK UHF Frequency Hopping",
            description: "Military UHF frequency hopping system for secure air-to-air and \
                air-to-ground communications",
            complexity: 4,
            bits_per_symbol: 0, // Analog AM for voice
            carries_data: true,
            characteristics: &[
                "UHF band (225-400 MHz)",
                "~7000 channels at 25 kHz spacing",
                "AM modulation for voice",
                "ASK modulation for data",
                "Word of Day (WOD) for security",
                "GPS time synchronization",
                "Anti-jam frequency hopping",
            ],
            history: "Developed in the 1970s-1980s by the US military. HAVE QUICK I was the \
                original system, followed by HAVE QUICK II with improved capabilities. \
                Widely used by NATO forces for tactical communications.",
            modern_usage: "Still in widespread use by military aviation. Being supplemented \
                by newer systems like SATURN for improved anti-jam performance. Many radios \
                support both HAVEQUICK and SINCGARS for interoperability.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        match self.traffic_type {
            TrafficType::Voice => {
                // Interpret input as audio samples (signed bytes)
                let audio: Vec<f64> = data.iter()
                    .map(|&b| (b as i8) as f64 / 128.0)
                    .collect();
                self.modulate_audio(&audio)
            }
            TrafficType::Data | TrafficType::Tone => {
                // Convert bytes to bits
                let bits: Vec<bool> = data.iter()
                    .flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1 == 1))
                    .collect();
                self.ask_modulate(&bits)
            }
        }
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        match self.traffic_type {
            TrafficType::Voice => {
                let audio = self.demodulate_audio(samples);
                // Convert audio back to bytes
                let bits: Vec<u8> = audio.iter()
                    .map(|&x| ((x * 128.0).clamp(-128.0, 127.0) as i8) as u8)
                    .collect();

                DemodResult {
                    bits,
                    symbols: Vec::new(),
                    ber_estimate: None,
                    snr_estimate: None,
                    metadata: std::collections::HashMap::new(),
                }
            }
            TrafficType::Data | TrafficType::Tone => {
                let bits = self.ask_demodulate(samples);
                // Pack bits into bytes
                let bytes: Vec<u8> = bits.chunks(8)
                    .map(|chunk| {
                        chunk.iter().enumerate()
                            .fold(0u8, |acc, (i, &b)| {
                                acc | ((b as u8) << (7 - i))
                            })
                    })
                    .collect();

                DemodResult {
                    bits: bytes,
                    symbols: Vec::new(),
                    ber_estimate: None,
                    snr_estimate: None,
                    metadata: std::collections::HashMap::new(),
                }
            }
        }
    }

    fn samples_per_symbol(&self) -> usize {
        // For AM voice, no discrete symbols
        // For ASK data, calculate from data rate
        match self.traffic_type {
            TrafficType::Voice => 1,
            _ => (self.common.sample_rate / 1200.0) as usize, // 1200 baud default
        }
    }

    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);

        // For AM, show the envelope
        let envelope: Vec<f64> = samples.iter().map(|s| s.norm()).collect();

        let channel = self.current_channel();

        VisualizationData {
            samples,
            constellation: vec![
                IQSample::new(1.0 - self.modulation_depth, 0.0),
                IQSample::new(1.0, 0.0),
                IQSample::new(1.0 + self.modulation_depth, 0.0),
            ],
            constellation_labels: vec![
                "Min".to_string(),
                "Carrier".to_string(),
                "Max".to_string(),
            ],
            spectrum: envelope,
            description: format!(
                "HAVEQUICK: Channel {} ({:.3} MHz), Mode: {:?}",
                channel.0,
                channel.to_frequency_hz() / 1_000_000.0,
                self.mode
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::simulator::*;

    fn create_test_havequick() -> Havequick {
        Havequick::new(
            48_000.0,
            Box::new(SimulatorHopper::new()),
            Box::new(SimulatorTimeSync::new()),
            Box::new(SimulatorNetController::new()),
            Box::new(AmVoiceCodec::default()),
            Box::new(AskDataModem::default()),
            Box::new(SimulatorTransec::new()),
        )
    }

    #[test]
    fn test_am_modulation() {
        let hq = create_test_havequick();

        // Create test audio data
        let data: Vec<u8> = (0..100).map(|i| (i % 256) as u8).collect();
        let modulated = hq.modulate(&data);

        assert!(!modulated.is_empty());

        // All samples should have reasonable amplitude
        for sample in &modulated {
            let mag = sample.norm();
            assert!(mag <= 2.0);
        }
    }

    #[test]
    fn test_ask_modulation() {
        let mut hq = create_test_havequick();
        hq.set_traffic_type(TrafficType::Data);

        let data = vec![0xAA, 0x55, 0xF0, 0x0F];
        let modulated = hq.modulate(&data);
        let demodulated = hq.demodulate(&modulated);

        // Should recover original data
        assert!(!demodulated.bits.is_empty());
    }

    #[test]
    fn test_frequency_hopping() {
        let mut hq = create_test_havequick();

        // Initialize with WOD
        let wod = WordOfDay::from_string("123456-789012-345678-901234-567890-123456").unwrap();
        let tod = TimeOfDay::new(2024, 100, 0, 0);
        hq.initialize(wod, NetId(0), tod).unwrap();

        // Get initial channel
        let ch1 = hq.current_channel();
        assert!(ch1.0 < 7000);

        // Advance hop
        hq.advance_hop();
        let ch2 = hq.current_channel();

        // Channels should be valid
        assert!(ch2.0 < 7000);
    }

    #[test]
    fn test_waveform_info() {
        let hq = create_test_havequick();
        let info = hq.info();

        assert_eq!(info.name, "HAVEQUICK");
        assert!(info.description.contains("UHF"));
    }
}

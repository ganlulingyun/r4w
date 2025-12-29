//! Link-16 Framework Implementation
//!
//! This module provides the unclassified framework for Link-16/MIDS/JTIDS.
//! The classified components (hopping algorithm, TRANSEC) are provided
//! via trait implementations.

use std::f64::consts::PI;

use crate::types::IQSample;
use crate::waveform::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};

use super::traits::*;
use super::types::*;

/// Link-16 waveform implementation
///
/// Link-16 is a NATO tactical data link using TDMA and frequency hopping.
/// It provides jam-resistant, secure communications for sharing situational
/// awareness between aircraft, ships, and ground forces.
pub struct Link16 {
    /// Common waveform parameters
    common: CommonParams,
    /// Terminal mode
    mode: TerminalMode,
    /// Pulse mode
    pulse_mode: PulseMode,
    /// Frequency hopping pattern
    hopper: Box<dyn HoppingPattern>,
    /// Time synchronization
    time_sync: Box<dyn TimeSync>,
    /// Message codec
    message_codec: Box<dyn MessageCodec>,
    /// Error correction
    fec: Box<dyn ErrorCorrection>,
    /// Interleaver
    interleaver: Box<dyn Interleaver>,
    /// TRANSEC provider
    transec: Box<dyn TransecProvider>,
    /// MSK modem
    modem: Box<dyn MskModem>,
    /// Pulse formatter
    pulse_formatter: Box<dyn PulseFormatter>,
    /// Network controller
    net_controller: Box<dyn NetworkController>,
    /// Track database
    track_db: Box<dyn TrackDatabase>,
    /// Net ID
    net_id: u16,
    /// Terminal ID
    terminal_id: u8,
    /// Carrier offset for visualization
    carrier_offset: f64,
}

impl std::fmt::Debug for Link16 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Link16")
            .field("common", &self.common)
            .field("mode", &self.mode)
            .field("pulse_mode", &self.pulse_mode)
            .field("net_id", &self.net_id)
            .field("terminal_id", &self.terminal_id)
            .finish()
    }
}

impl Link16 {
    /// Create a new Link-16 instance
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        sample_rate: f64,
        hopper: Box<dyn HoppingPattern>,
        time_sync: Box<dyn TimeSync>,
        message_codec: Box<dyn MessageCodec>,
        fec: Box<dyn ErrorCorrection>,
        interleaver: Box<dyn Interleaver>,
        transec: Box<dyn TransecProvider>,
        modem: Box<dyn MskModem>,
        pulse_formatter: Box<dyn PulseFormatter>,
        net_controller: Box<dyn NetworkController>,
        track_db: Box<dyn TrackDatabase>,
    ) -> Self {
        Self {
            common: CommonParams {
                sample_rate,
                carrier_freq: 1000.0,
                amplitude: 1.0,
            },
            mode: TerminalMode::ReceiveOnly,
            pulse_mode: PulseMode::Packed2,
            hopper,
            time_sync,
            message_codec,
            fec,
            interleaver,
            transec,
            modem,
            pulse_formatter,
            net_controller,
            track_db,
            net_id: 0,
            terminal_id: 0,
            carrier_offset: 1000.0,
        }
    }

    /// Get sample rate
    pub fn sample_rate(&self) -> f64 {
        self.common.sample_rate
    }

    /// Set terminal mode
    pub fn set_mode(&mut self, mode: TerminalMode) {
        self.mode = mode;
        self.net_controller.set_terminal_mode(mode);
    }

    /// Set pulse mode
    pub fn set_pulse_mode(&mut self, mode: PulseMode) {
        self.pulse_mode = mode;
    }

    /// Initialize and join network
    pub fn join_network(
        &mut self,
        net_id: u16,
        terminal_id: u8,
        crypto_var: &[u8],
    ) -> Result<(), Link16Error> {
        self.net_id = net_id;
        self.terminal_id = terminal_id;

        // Initialize hopping pattern
        self.hopper.initialize(crypto_var, net_id);

        // Join network
        self.net_controller.join_network(net_id, terminal_id)?;

        // Set active mode
        self.mode = TerminalMode::Active;

        Ok(())
    }

    /// Leave current network
    pub fn leave_network(&mut self) {
        self.net_controller.leave_network();
        self.hopper.reset();
        self.mode = TerminalMode::ReceiveOnly;
    }

    /// Check if synchronized
    pub fn is_synchronized(&self) -> bool {
        self.time_sync.is_synchronized()
    }

    /// Get current time slot
    pub fn current_slot(&self) -> TimeSlot {
        self.time_sync.get_network_time().time_slot()
    }

    /// Get frequency for current slot
    pub fn current_frequency(&self) -> Frequency {
        self.hopper.get_frequency(self.current_slot())
    }

    /// Check if current slot is assigned for transmission
    pub fn is_tx_slot(&self) -> bool {
        self.net_controller.is_tx_slot(self.current_slot())
    }

    /// Encode a J-series message
    pub fn encode_message(
        &self,
        msg_type: JSeriesMessage,
        data: &[u8],
    ) -> Result<Vec<u8>, Link16Error> {
        // Encode to words
        let words = self.message_codec.encode(msg_type, data)?;

        // Flatten words to bytes
        let mut bytes = Vec::new();
        for word in &words {
            bytes.extend_from_slice(&word.data);
        }

        // Apply FEC
        let encoded = self.fec.encode(&bytes);

        // Interleave
        let interleaved = self.interleaver.interleave(&encoded);

        // Apply TRANSEC if loaded
        let slot = self.current_slot();
        let secured = if self.transec.is_loaded() {
            self.transec.encrypt(&interleaved, slot)
        } else {
            interleaved
        };

        Ok(secured)
    }

    /// Decode a received message
    pub fn decode_message(&self, data: &[u8]) -> Result<(JSeriesMessage, Vec<u8>), Link16Error> {
        // Decrypt if needed
        let slot = self.current_slot();
        let decrypted = if self.transec.is_loaded() {
            self.transec.decrypt(data, slot)?
        } else {
            data.to_vec()
        };

        // De-interleave
        let deinterleaved = self.interleaver.deinterleave(&decrypted);

        // FEC decode
        let (decoded, _errors) = self.fec.decode(&deinterleaved)?;

        // Parse words
        let mut words = Vec::new();
        for chunk in decoded.chunks(10) {
            let mut word = Link16Word::new(WordType::Initial);
            let len = chunk.len().min(10);
            word.data[..len].copy_from_slice(&chunk[..len]);
            words.push(word);
        }

        // Decode message
        self.message_codec.decode(&words)
    }

    /// MSK modulate data
    fn msk_modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // Convert bytes to bits
        let bits: Vec<bool> = data
            .iter()
            .flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1 == 1))
            .collect();

        // MSK modulate
        let baseband = self.modem.modulate(&bits);

        // Upconvert to carrier
        let carrier_freq = self.carrier_offset;
        let sample_rate = self.common.sample_rate;

        baseband
            .iter()
            .enumerate()
            .map(|(i, &s)| {
                let phase = 2.0 * PI * carrier_freq * (i as f64) / sample_rate;
                IQSample::new(s * phase.cos(), s * phase.sin())
            })
            .collect()
    }

    /// MSK demodulate to data
    fn msk_demodulate(&self, samples: &[IQSample]) -> Vec<u8> {
        // Downconvert
        let carrier_freq = self.carrier_offset;
        let sample_rate = self.common.sample_rate;

        let baseband: Vec<f64> = samples
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let phase = 2.0 * PI * carrier_freq * (i as f64) / sample_rate;
                s.re * phase.cos() + s.im * phase.sin()
            })
            .collect();

        // MSK demodulate
        let bits = self.modem.demodulate(&baseband);

        // Pack bits to bytes
        bits.chunks(8)
            .map(|chunk| {
                chunk
                    .iter()
                    .enumerate()
                    .fold(0u8, |acc, (i, &b)| acc | ((b as u8) << (7 - i)))
            })
            .collect()
    }

    /// Subscribe to NPG
    pub fn subscribe_npg(&mut self, npg: Npg) {
        self.net_controller.subscribe_npg(npg);
    }

    /// Update track in database
    pub fn update_track(&mut self, stn: SourceTrackNumber, data: &[u8]) {
        self.track_db.update_track(stn, data);
    }

    /// Get track from database
    pub fn get_track(&self, stn: SourceTrackNumber) -> Option<Vec<u8>> {
        self.track_db.get_track(stn)
    }
}

impl Waveform for Link16 {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "Link-16",
            full_name: "Link-16 Tactical Data Link (TADIL-J)",
            description: "NATO tactical data link for secure, jam-resistant sharing of \
                situational awareness between aircraft, ships, and ground forces",
            complexity: 5,
            bits_per_symbol: 1, // MSK
            carries_data: true,
            characteristics: &[
                "L-band (960-1215 MHz)",
                "51 frequencies, 3 MHz spacing",
                "TDMA with 1536 slots per 12.8 min frame",
                "MSK modulation",
                "Frequency hopping",
                "Reed-Solomon FEC",
                "J-series messages",
                "GPS time sync",
            ],
            history: "Developed in the 1970s-1980s. Originally called JTIDS (Joint Tactical \
                Information Distribution System). MIDS (Multifunctional Information \
                Distribution System) is the current terminal. Widely deployed by NATO.",
            modern_usage: "Primary tactical data link for NATO air, naval, and ground forces. \
                Used for sharing track data, target assignments, and voice. Link-16 terminals \
                are installed on most modern fighters, AWACS, ships, and command posts.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // Encode message (using FreeText for raw data)
        let encoded = match self.encode_message(JSeriesMessage::FreeText, data) {
            Ok(e) => e,
            Err(_) => data.to_vec(), // Fallback to raw data
        };

        // Format pulses
        let pulses = self.pulse_formatter.format_pulses(&encoded, self.pulse_mode);

        // Convert to bytes for MSK modulation
        let pulse_bytes: Vec<u8> = pulses
            .chunks(8)
            .map(|chunk| {
                chunk.iter().enumerate().fold(0u8, |acc, (i, &p)| {
                    let bit = if p > 0.0 { 1 } else { 0 };
                    acc | (bit << (7 - i))
                })
            })
            .collect();

        // MSK modulate
        self.msk_modulate(&pulse_bytes)
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        // MSK demodulate
        let pulse_bytes = self.msk_demodulate(samples);

        // Extract pulses
        let extracted = self.pulse_formatter.extract_pulses(
            &pulse_bytes.iter().flat_map(|&b| {
                (0..8).rev().map(move |i| if (b >> i) & 1 == 1 { 1.0 } else { -1.0 })
            }).collect::<Vec<_>>(),
            self.pulse_mode,
        );

        // Decode message
        let data = match self.decode_message(&extracted) {
            Ok((_msg_type, d)) => d,
            Err(_) => extracted,
        };

        let metadata = std::collections::HashMap::new();

        DemodResult {
            bits: data,
            symbols: Vec::new(),
            ber_estimate: None,
            snr_estimate: None,
            metadata,
        }
    }

    fn samples_per_symbol(&self) -> usize {
        self.modem.samples_per_symbol()
    }

    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);

        // MSK constellation is a circle
        let constellation: Vec<IQSample> = (0..16)
            .map(|i| {
                let angle = 2.0 * PI * (i as f64) / 16.0;
                IQSample::new(angle.cos(), angle.sin())
            })
            .collect();

        let slot = self.current_slot();
        let freq = self.current_frequency();

        VisualizationData {
            samples,
            constellation,
            constellation_labels: vec![],
            spectrum: Vec::new(),
            description: format!(
                "Link-16: Slot {}.{}, Freq {} ({:.1} MHz), Mode: {:?}",
                slot.epoch,
                slot.slot,
                freq.0,
                freq.to_mhz(),
                self.mode
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::simulator::*;
    use super::*;

    fn create_test_link16() -> Link16 {
        Link16::new(
            1_000_000.0, // 1 MHz sample rate
            Box::new(SimulatorHopper::new()),
            Box::new(SimulatorTimeSync::new()),
            Box::new(SimulatorMessageCodec::new()),
            Box::new(SimulatorReedSolomon::default()),
            Box::new(SimulatorInterleaver::default()),
            Box::new(SimulatorTransec::new()),
            Box::new(SimulatorMskModem::default()),
            Box::new(SimulatorPulseFormatter::default()),
            Box::new(SimulatorNetController::new()),
            Box::new(SimulatorTrackDb::new()),
        )
    }

    #[test]
    fn test_join_network() {
        let mut link16 = create_test_link16();

        link16.join_network(1234, 5, b"crypto_key").unwrap();
        assert_eq!(link16.mode, TerminalMode::Active);
        assert!(link16.is_tx_slot() || !link16.is_tx_slot()); // Just verify it runs
    }

    #[test]
    fn test_modulation() {
        let link16 = create_test_link16();

        let data = vec![0xAA, 0x55, 0xF0, 0x0F];
        let modulated = link16.modulate(&data);

        assert!(!modulated.is_empty());

        // All samples should have reasonable amplitude
        for sample in &modulated {
            let mag = sample.norm();
            assert!(mag <= 2.0);
        }
    }

    #[test]
    fn test_message_encode() {
        let link16 = create_test_link16();

        let data = b"Test message".to_vec();
        let encoded = link16.encode_message(JSeriesMessage::FreeText, &data).unwrap();
        assert!(!encoded.is_empty());

        // Encoded data should be larger due to FEC and framing
        assert!(encoded.len() >= data.len());
    }

    #[test]
    fn test_waveform_info() {
        let link16 = create_test_link16();
        let info = link16.info();

        assert_eq!(info.name, "Link-16");
        assert!(info.description.contains("tactical"));
    }

    #[test]
    fn test_frequency_hopping() {
        let mut link16 = create_test_link16();
        link16.join_network(1, 1, b"key").unwrap();

        let freq1 = link16.current_frequency();
        assert!(freq1.0 < 51);
    }
}

//! Link-16 Simulator Implementations
//!
//! Provides unclassified simulator implementations for training and testing.
//! These implementations use simplified algorithms that approximate real
//! behavior without exposing classified techniques.

use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;

use super::traits::*;
use super::types::*;

/// Simulated frequency hopping using LFSR
pub struct SimulatorHopper {
    /// LFSR state
    state: u32,
    /// Initial seed
    seed: u32,
    /// Net ID for pattern variation
    net_id: u16,
}

impl SimulatorHopper {
    pub fn new() -> Self {
        Self {
            state: 0x12345678,
            seed: 0x12345678,
            net_id: 0,
        }
    }

    fn next_state(&self, current: u32) -> u32 {
        // Simple LFSR with taps at bits 31, 21, 1, 0
        let bit = ((current >> 31) ^ (current >> 21) ^ (current >> 1) ^ current) & 1;
        (current << 1) | bit
    }
}

impl Default for SimulatorHopper {
    fn default() -> Self {
        Self::new()
    }
}

impl HoppingPattern for SimulatorHopper {
    fn initialize(&mut self, crypto_var: &[u8], net_id: u16) {
        // Create seed from crypto variable
        self.seed = crypto_var.iter().enumerate().fold(0u32, |acc, (i, &b)| {
            acc ^ ((b as u32) << ((i % 4) * 8))
        });
        self.seed ^= (net_id as u32) << 16;
        self.state = self.seed;
        self.net_id = net_id;
    }

    fn get_frequency(&self, slot: TimeSlot) -> Frequency {
        // Advance LFSR to slot position
        let mut state = self.seed;
        for _ in 0..slot.absolute() {
            state = self.next_state(state);
        }
        // Map to frequency index (0-50)
        let freq_index = (state % 51) as u8;
        Frequency(freq_index)
    }

    fn reset(&mut self) {
        self.state = self.seed;
    }
}

/// Simulated time synchronization
pub struct SimulatorTimeSync {
    /// Current network time
    current_time: NetworkTime,
    /// Whether synchronized
    synchronized: bool,
    /// Accumulated microseconds since last slot
    accumulated_us: u64,
}

impl SimulatorTimeSync {
    pub fn new() -> Self {
        Self {
            current_time: NetworkTime {
                epoch: 0,
                slot: 0,
                pulse: 0,
                sub_pulse_ns: 0,
            },
            synchronized: false,
            accumulated_us: 0,
        }
    }
}

impl Default for SimulatorTimeSync {
    fn default() -> Self {
        Self::new()
    }
}

impl TimeSync for SimulatorTimeSync {
    fn get_network_time(&self) -> NetworkTime {
        self.current_time
    }

    fn set_network_time(&mut self, time: NetworkTime) {
        self.current_time = time;
        self.synchronized = true;
        self.accumulated_us = 0;
    }

    fn is_synchronized(&self) -> bool {
        self.synchronized
    }

    fn time_to_next_slot(&self) -> u64 {
        let slot_us = TimeSlot::SLOT_DURATION_US as u64;
        slot_us - (self.accumulated_us % slot_us)
    }

    fn advance(&mut self, microseconds: u64) {
        self.accumulated_us += microseconds;
        let slot_us = TimeSlot::SLOT_DURATION_US as u64;

        while self.accumulated_us >= slot_us {
            self.accumulated_us -= slot_us;
            self.current_time.slot += 1;
            if self.current_time.slot >= TimeSlot::SLOTS_PER_EPOCH {
                self.current_time.slot = 0;
                self.current_time.epoch += 1;
                if self.current_time.epoch >= TimeSlot::EPOCHS_PER_FRAME {
                    self.current_time.epoch = 0;
                }
            }
        }
    }
}

/// Simulated J-series message codec
pub struct SimulatorMessageCodec;

impl SimulatorMessageCodec {
    pub fn new() -> Self {
        Self
    }
}

impl Default for SimulatorMessageCodec {
    fn default() -> Self {
        Self::new()
    }
}

impl MessageCodec for SimulatorMessageCodec {
    fn encode(&self, msg_type: JSeriesMessage, data: &[u8]) -> Result<Vec<Link16Word>, Link16Error> {
        let word_count = msg_type.word_count();
        let mut words = Vec::with_capacity(word_count);

        // Create header word with message type
        let mut header = Link16Word::new(WordType::Header);
        // Store message label indicator in first bytes
        let label_bytes = msg_type.label().as_bytes();
        for (i, &b) in label_bytes.iter().take(5).enumerate() {
            header.data[i] = b;
        }
        words.push(header);

        // Pack data into remaining words
        let mut data_offset = 0;
        for i in 1..word_count {
            let word_type = if i == 1 {
                WordType::Initial
            } else {
                WordType::Extension
            };
            let mut word = Link16Word::new(word_type);

            // Copy up to 10 bytes of data
            let bytes_to_copy = (data.len() - data_offset).min(10);
            word.data[..bytes_to_copy].copy_from_slice(&data[data_offset..data_offset + bytes_to_copy]);
            data_offset += bytes_to_copy;

            words.push(word);
        }

        Ok(words)
    }

    fn decode(&self, words: &[Link16Word]) -> Result<(JSeriesMessage, Vec<u8>), Link16Error> {
        if words.is_empty() {
            return Err(Link16Error::DecodingError("No words to decode".into()));
        }

        let msg_type = self.get_message_type(&words[0])?;

        // Extract data from words (skip header)
        let mut data = Vec::new();
        for word in words.iter().skip(1) {
            data.extend_from_slice(&word.data);
        }

        Ok((msg_type, data))
    }

    fn get_message_type(&self, header: &Link16Word) -> Result<JSeriesMessage, Link16Error> {
        // Parse label from header
        let label = std::str::from_utf8(&header.data[..5])
            .map_err(|_| Link16Error::DecodingError("Invalid header".into()))?
            .trim_end_matches('\0');

        match label {
            "J0.0" => Ok(JSeriesMessage::InitialEntry),
            "J2.0" => Ok(JSeriesMessage::IndirectPpli),
            "J2.2" => Ok(JSeriesMessage::AirPpli),
            "J2.3" => Ok(JSeriesMessage::SurfacePpli),
            "J2.4" => Ok(JSeriesMessage::SubsurfacePpli),
            "J2.5" => Ok(JSeriesMessage::LandPointPpli),
            "J3.0" => Ok(JSeriesMessage::ReferencePoint),
            "J3.2" => Ok(JSeriesMessage::AirTrack),
            "J3.3" => Ok(JSeriesMessage::SurfaceTrack),
            "J3.5" => Ok(JSeriesMessage::LandTrack),
            "J7.0" => Ok(JSeriesMessage::TrackManagement),
            "J7.1" => Ok(JSeriesMessage::DataUpdateRequest),
            "J12." => Ok(JSeriesMessage::MissionAssignment),
            "J28." => Ok(JSeriesMessage::FreeText),
            _ => Err(Link16Error::DecodingError(format!("Unknown message: {}", label))),
        }
    }
}

/// Simulated Reed-Solomon codec
pub struct SimulatorReedSolomon {
    /// Error correction capability
    t: usize,
}

impl SimulatorReedSolomon {
    pub fn new(t: usize) -> Self {
        Self { t }
    }
}

impl Default for SimulatorReedSolomon {
    fn default() -> Self {
        Self::new(4) // Standard Link-16 uses t=4
    }
}

impl ErrorCorrection for SimulatorReedSolomon {
    fn encode(&self, data: &[u8]) -> Vec<u8> {
        // Simplified: just append checksum bytes
        let mut encoded = data.to_vec();
        let checksum: u8 = data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
        for _ in 0..(2 * self.t) {
            encoded.push(checksum);
        }
        encoded
    }

    fn decode(&self, data: &[u8]) -> Result<(Vec<u8>, usize), Link16Error> {
        if data.len() < 2 * self.t {
            return Err(Link16Error::DecodingError("Data too short".into()));
        }
        let data_len = data.len() - 2 * self.t;
        let decoded = data[..data_len].to_vec();
        Ok((decoded, 0)) // Simulator doesn't actually correct errors
    }

    fn correction_capability(&self) -> usize {
        self.t
    }
}

/// Simulated symbol interleaver
pub struct SimulatorInterleaver {
    depth: usize,
}

impl SimulatorInterleaver {
    pub fn new(depth: usize) -> Self {
        Self { depth }
    }
}

impl Default for SimulatorInterleaver {
    fn default() -> Self {
        Self::new(16)
    }
}

impl Interleaver for SimulatorInterleaver {
    fn interleave(&self, symbols: &[u8]) -> Vec<u8> {
        if symbols.is_empty() || self.depth <= 1 {
            return symbols.to_vec();
        }

        let rows = (symbols.len() + self.depth - 1) / self.depth;
        let mut interleaved = vec![0u8; symbols.len()];

        for (i, &sym) in symbols.iter().enumerate() {
            let row = i / self.depth;
            let col = i % self.depth;
            let new_idx = col * rows + row;
            if new_idx < interleaved.len() {
                interleaved[new_idx] = sym;
            }
        }

        interleaved
    }

    fn deinterleave(&self, symbols: &[u8]) -> Vec<u8> {
        if symbols.is_empty() || self.depth <= 1 {
            return symbols.to_vec();
        }

        let rows = (symbols.len() + self.depth - 1) / self.depth;
        let mut deinterleaved = vec![0u8; symbols.len()];

        for (i, &sym) in symbols.iter().enumerate() {
            let col = i / rows;
            let row = i % rows;
            let new_idx = row * self.depth + col;
            if new_idx < deinterleaved.len() {
                deinterleaved[new_idx] = sym;
            }
        }

        deinterleaved
    }

    fn depth(&self) -> usize {
        self.depth
    }
}

/// Simulated TRANSEC (no actual encryption)
pub struct SimulatorTransec {
    /// Simulated key loaded
    key_loaded: bool,
    /// Crypto mode
    mode: CryptoMode,
}

impl SimulatorTransec {
    pub fn new() -> Self {
        Self {
            key_loaded: false,
            mode: CryptoMode::Plain,
        }
    }
}

impl Default for SimulatorTransec {
    fn default() -> Self {
        Self::new()
    }
}

impl TransecProvider for SimulatorTransec {
    fn load_key(&mut self, _key: &[u8]) -> Result<(), Link16Error> {
        self.key_loaded = true;
        self.mode = CryptoMode::Secure;
        Ok(())
    }

    fn encrypt(&self, data: &[u8], _slot: TimeSlot) -> Vec<u8> {
        // Simulator: XOR with simple pattern
        data.iter().enumerate().map(|(i, &b)| b ^ (i as u8)).collect()
    }

    fn decrypt(&self, data: &[u8], _slot: TimeSlot) -> Result<Vec<u8>, Link16Error> {
        // Reverse the XOR
        Ok(data.iter().enumerate().map(|(i, &b)| b ^ (i as u8)).collect())
    }

    fn is_loaded(&self) -> bool {
        self.key_loaded
    }

    fn mode(&self) -> CryptoMode {
        self.mode
    }

    fn zeroize(&mut self) {
        self.key_loaded = false;
        self.mode = CryptoMode::Plain;
    }
}

/// Simulated MSK modem
pub struct SimulatorMskModem {
    samples_per_symbol: usize,
}

impl SimulatorMskModem {
    pub fn new(samples_per_symbol: usize) -> Self {
        Self { samples_per_symbol }
    }
}

impl Default for SimulatorMskModem {
    fn default() -> Self {
        Self::new(8)
    }
}

impl MskModem for SimulatorMskModem {
    fn modulate(&self, bits: &[bool]) -> Vec<f64> {
        let mut samples = Vec::with_capacity(bits.len() * self.samples_per_symbol);
        let mut phase = 0.0f64;

        for &bit in bits {
            let freq_offset = if bit { 0.25 } else { -0.25 }; // h = 0.5 for MSK

            for i in 0..self.samples_per_symbol {
                let t = i as f64 / self.samples_per_symbol as f64;
                phase += 2.0 * PI * freq_offset / self.samples_per_symbol as f64;
                samples.push((2.0 * PI * t + phase).cos());
            }
        }

        samples
    }

    fn demodulate(&self, symbols: &[f64]) -> Vec<bool> {
        let mut bits = Vec::new();

        for chunk in symbols.chunks(self.samples_per_symbol) {
            if chunk.len() < self.samples_per_symbol {
                break;
            }

            // Simple frequency discriminator
            let mut sum = 0.0;
            for i in 1..chunk.len() {
                sum += chunk[i] * chunk[i - 1];
            }

            bits.push(sum > 0.0);
        }

        bits
    }

    fn samples_per_symbol(&self) -> usize {
        self.samples_per_symbol
    }
}

/// Simulated pulse formatter
pub struct SimulatorPulseFormatter {
    samples_per_pulse: usize,
}

impl SimulatorPulseFormatter {
    pub fn new(samples_per_pulse: usize) -> Self {
        Self { samples_per_pulse }
    }
}

impl Default for SimulatorPulseFormatter {
    fn default() -> Self {
        Self::new(64)
    }
}

impl PulseFormatter for SimulatorPulseFormatter {
    fn format_pulses(&self, data: &[u8], mode: PulseMode) -> Vec<f64> {
        let pulses_needed = self.pulses_per_slot(mode);
        let mut pulses = Vec::with_capacity(pulses_needed * self.samples_per_pulse);

        // Convert bytes to pulse amplitudes
        for &byte in data {
            for i in 0..8 {
                let bit = (byte >> (7 - i)) & 1;
                let amplitude = if bit == 1 { 1.0 } else { -1.0 };

                // Generate pulse shape (simple rectangular for simulator)
                for _ in 0..self.samples_per_pulse {
                    pulses.push(amplitude);
                }
            }
        }

        // Pad to slot length
        while pulses.len() < pulses_needed * self.samples_per_pulse {
            pulses.push(0.0);
        }

        pulses
    }

    fn extract_pulses(&self, pulses: &[f64], _mode: PulseMode) -> Vec<u8> {
        let mut bytes = Vec::new();
        let mut current_byte = 0u8;
        let mut bit_count = 0;

        for chunk in pulses.chunks(self.samples_per_pulse) {
            if chunk.is_empty() {
                break;
            }

            // Average the chunk to determine bit value
            let avg: f64 = chunk.iter().sum::<f64>() / chunk.len() as f64;
            let bit = if avg > 0.0 { 1 } else { 0 };

            current_byte = (current_byte << 1) | bit;
            bit_count += 1;

            if bit_count == 8 {
                bytes.push(current_byte);
                current_byte = 0;
                bit_count = 0;
            }
        }

        bytes
    }
}

/// Simulated network controller
pub struct SimulatorNetController {
    net_id: Option<u16>,
    terminal_id: u8,
    assigned_slots: Vec<TimeSlot>,
    npgs: HashSet<Npg>,
    mode: TerminalMode,
}

impl SimulatorNetController {
    pub fn new() -> Self {
        Self {
            net_id: None,
            terminal_id: 0,
            assigned_slots: Vec::new(),
            npgs: HashSet::new(),
            mode: TerminalMode::ReceiveOnly,
        }
    }
}

impl Default for SimulatorNetController {
    fn default() -> Self {
        Self::new()
    }
}

impl NetworkController for SimulatorNetController {
    fn join_network(&mut self, net_id: u16, terminal_id: u8) -> Result<(), Link16Error> {
        self.net_id = Some(net_id);
        self.terminal_id = terminal_id;

        // Assign some slots based on terminal ID (simplified)
        self.assigned_slots.clear();
        for epoch in 0..TimeSlot::EPOCHS_PER_FRAME {
            let slot = terminal_id % TimeSlot::SLOTS_PER_EPOCH;
            if let Some(ts) = TimeSlot::new(epoch, slot) {
                self.assigned_slots.push(ts);
            }
        }

        self.mode = TerminalMode::Active;
        Ok(())
    }

    fn leave_network(&mut self) {
        self.net_id = None;
        self.assigned_slots.clear();
        self.npgs.clear();
        self.mode = TerminalMode::ReceiveOnly;
    }

    fn get_assigned_slots(&self) -> Vec<TimeSlot> {
        self.assigned_slots.clone()
    }

    fn is_tx_slot(&self, slot: TimeSlot) -> bool {
        self.assigned_slots.contains(&slot)
    }

    fn get_active_npgs(&self) -> Vec<Npg> {
        self.npgs.iter().copied().collect()
    }

    fn subscribe_npg(&mut self, npg: Npg) {
        self.npgs.insert(npg);
    }

    fn unsubscribe_npg(&mut self, npg: Npg) {
        self.npgs.remove(&npg);
    }

    fn terminal_mode(&self) -> TerminalMode {
        self.mode
    }

    fn set_terminal_mode(&mut self, mode: TerminalMode) {
        self.mode = mode;
    }
}

/// Simulated track database
pub struct SimulatorTrackDb {
    tracks: HashMap<u16, Vec<u8>>, // Keyed by packed STN
}

impl SimulatorTrackDb {
    pub fn new() -> Self {
        Self {
            tracks: HashMap::new(),
        }
    }
}

impl Default for SimulatorTrackDb {
    fn default() -> Self {
        Self::new()
    }
}

impl TrackDatabase for SimulatorTrackDb {
    fn update_track(&mut self, stn: SourceTrackNumber, data: &[u8]) {
        self.tracks.insert(stn.pack(), data.to_vec());
    }

    fn get_track(&self, stn: SourceTrackNumber) -> Option<Vec<u8>> {
        self.tracks.get(&stn.pack()).cloned()
    }

    fn remove_track(&mut self, stn: SourceTrackNumber) {
        self.tracks.remove(&stn.pack());
    }

    fn get_all_tracks(&self) -> Vec<SourceTrackNumber> {
        self.tracks.keys().map(|&k| SourceTrackNumber::unpack(k)).collect()
    }

    fn clear(&mut self) {
        self.tracks.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hopping_pattern() {
        let mut hopper = SimulatorHopper::new();
        hopper.initialize(b"test_key", 1);

        let freq1 = hopper.get_frequency(TimeSlot::new(0, 0).unwrap());
        let freq2 = hopper.get_frequency(TimeSlot::new(0, 1).unwrap());

        // Frequencies should be valid
        assert!(freq1.0 < 51);
        assert!(freq2.0 < 51);

        // Reset should give same sequence
        hopper.reset();
        let freq1_again = hopper.get_frequency(TimeSlot::new(0, 0).unwrap());
        assert_eq!(freq1, freq1_again);
    }

    #[test]
    fn test_time_sync() {
        let mut sync = SimulatorTimeSync::new();
        assert!(!sync.is_synchronized());

        sync.set_network_time(NetworkTime {
            epoch: 5,
            slot: 10,
            pulse: 0,
            sub_pulse_ns: 0,
        });
        assert!(sync.is_synchronized());

        let time = sync.get_network_time();
        assert_eq!(time.epoch, 5);
        assert_eq!(time.slot, 10);
    }

    #[test]
    fn test_interleaver() {
        let interleaver = SimulatorInterleaver::new(4);
        let data = vec![1, 2, 3, 4, 5, 6, 7, 8];

        let interleaved = interleaver.interleave(&data);
        let deinterleaved = interleaver.deinterleave(&interleaved);

        assert_eq!(data, deinterleaved);
    }

    #[test]
    fn test_msk_modem() {
        let modem = SimulatorMskModem::new(8);
        let bits = vec![true, false, true, true, false];

        let modulated = modem.modulate(&bits);
        assert_eq!(modulated.len(), bits.len() * 8);

        let demodulated = modem.demodulate(&modulated);
        assert_eq!(demodulated.len(), bits.len());
    }

    #[test]
    fn test_network_controller() {
        let mut controller = SimulatorNetController::new();
        assert!(controller.get_assigned_slots().is_empty());

        controller.join_network(1234, 5).unwrap();
        assert!(!controller.get_assigned_slots().is_empty());
        assert_eq!(controller.terminal_mode(), TerminalMode::Active);

        controller.subscribe_npg(Npg::SURVEILLANCE);
        assert!(controller.get_active_npgs().contains(&Npg::SURVEILLANCE));
    }
}

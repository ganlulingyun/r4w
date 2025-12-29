//! Simulator implementations for HAVEQUICK testing
//!
//! These implementations provide non-secure, deterministic behavior
//! suitable for testing and development. They do NOT provide actual
//! HAVEQUICK security and should never be used operationally.

use super::traits::*;
use super::types::*;

/// Simulator hopping algorithm (NOT SECURE)
///
/// Uses a simple LFSR-based pseudorandom sequence for testing.
/// The actual HAVEQUICK hopping algorithm is classified.
pub struct SimulatorHopper {
    current_channel: ChannelNumber,
    hop_number: u64,
    seed: u64,
    lfsr_state: u64,
    num_channels: u16,
}

impl SimulatorHopper {
    pub fn new() -> Self {
        Self {
            current_channel: ChannelNumber::new(0),
            hop_number: 0,
            seed: 0,
            lfsr_state: 1,
            num_channels: 7000,
        }
    }

    /// Simple LFSR for pseudorandom sequence
    fn lfsr_next(&mut self) -> u64 {
        // 64-bit LFSR with taps at 64, 63, 61, 60
        let bit = ((self.lfsr_state >> 63) ^ (self.lfsr_state >> 62) ^
                   (self.lfsr_state >> 60) ^ (self.lfsr_state >> 59)) & 1;
        self.lfsr_state = (self.lfsr_state << 1) | bit;
        self.lfsr_state
    }

    /// Derive seed from WOD, Net ID, and TOD
    fn derive_seed(wod: &WordOfDay, net_id: NetId, tod: &TimeOfDay) -> u64 {
        let wod_bytes = wod.as_bytes();
        let mut seed: u64 = 0;

        // Mix WOD bytes
        for (i, &byte) in wod_bytes.iter().enumerate() {
            seed ^= (byte as u64) << ((i % 8) * 8);
        }

        // Mix Net ID
        seed ^= (net_id.0 as u64) << 40;

        // Mix day for daily variation
        seed ^= (tod.julian_day as u64) << 48;
        seed ^= (tod.year as u64) << 32;

        // Ensure non-zero
        if seed == 0 {
            seed = 0x5A5A5A5A5A5A5A5A;
        }

        seed
    }
}

impl Default for SimulatorHopper {
    fn default() -> Self {
        Self::new()
    }
}

impl HoppingAlgorithm for SimulatorHopper {
    fn initialize(&mut self, wod: &WordOfDay, net_id: NetId, tod: TimeOfDay) {
        self.seed = Self::derive_seed(wod, net_id, &tod);
        self.lfsr_state = self.seed;
        self.hop_number = tod.to_hop_number(HopRate::Medium.dwell_time_us());

        // Advance LFSR to current hop
        for _ in 0..self.hop_number {
            self.lfsr_next();
        }

        self.current_channel = ChannelNumber::new((self.lfsr_state % self.num_channels as u64) as u16);
    }

    fn get_current_channel(&self) -> ChannelNumber {
        self.current_channel
    }

    fn advance_hop(&mut self) {
        self.hop_number += 1;
        let rand = self.lfsr_next();
        self.current_channel = ChannelNumber::new((rand % self.num_channels as u64) as u16);
    }

    fn sync_to_time(&mut self, tod: TimeOfDay) {
        let target_hop = tod.to_hop_number(HopRate::Medium.dwell_time_us());

        // Reset LFSR to seed
        self.lfsr_state = self.seed;

        // Advance to target hop
        for _ in 0..target_hop {
            self.lfsr_next();
        }

        self.hop_number = target_hop;
        self.current_channel = ChannelNumber::new((self.lfsr_state % self.num_channels as u64) as u16);
    }

    fn current_hop_number(&self) -> u64 {
        self.hop_number
    }

    fn predict_channel(&self, hop_number: u64) -> ChannelNumber {
        // Save state
        let mut temp_state = self.seed;

        // Advance to requested hop
        for _ in 0..hop_number {
            let bit = ((temp_state >> 63) ^ (temp_state >> 62) ^
                       (temp_state >> 60) ^ (temp_state >> 59)) & 1;
            temp_state = (temp_state << 1) | bit;
        }

        ChannelNumber::new((temp_state % self.num_channels as u64) as u16)
    }
}

/// Simulator time sync (NOT SECURE)
pub struct SimulatorTimeSync {
    current_tod: Option<TimeOfDay>,
    source: TimeSource,
    sync_status: SyncStatus,
    uncertainty_us: u32,
}

impl SimulatorTimeSync {
    pub fn new() -> Self {
        Self {
            current_tod: None,
            source: TimeSource::Internal,
            sync_status: SyncStatus::NotSynced,
            uncertainty_us: 1_000_000, // 1 second uncertainty initially
        }
    }
}

impl Default for SimulatorTimeSync {
    fn default() -> Self {
        Self::new()
    }
}

impl TimeSyncProtocol for SimulatorTimeSync {
    fn get_current_tod(&self) -> Option<TimeOfDay> {
        self.current_tod
    }

    fn set_tod(&mut self, tod: TimeOfDay, source: TimeSource) {
        self.current_tod = Some(tod);
        self.source = source;
        self.sync_status = SyncStatus::Synced;
        self.uncertainty_us = match source {
            TimeSource::Gps => 1,        // GPS: ~1 microsecond
            TimeSource::Network => 100,  // Network: ~100 microseconds
            TimeSource::Manual => 1000,  // Manual: ~1 millisecond
            TimeSource::Internal => 10_000, // Internal: ~10 milliseconds
        };
    }

    fn is_synchronized(&self) -> bool {
        self.sync_status == SyncStatus::Synced
    }

    fn sync_status(&self) -> SyncStatus {
        self.sync_status
    }

    fn advance_time(&mut self, microseconds: u32) {
        if let Some(ref mut tod) = self.current_tod {
            let total_us = tod.microseconds + microseconds;
            tod.microseconds = total_us % 1_000_000;
            tod.seconds += total_us / 1_000_000;

            // Handle day rollover
            if tod.seconds >= 86400 {
                tod.seconds -= 86400;
                tod.julian_day += 1;
                if tod.julian_day > 366 {
                    tod.julian_day = 1;
                    tod.year += 1;
                }
            }
        }
    }

    fn get_time_uncertainty(&self) -> u32 {
        self.uncertainty_us
    }
}

/// Simulator net controller (NOT SECURE)
pub struct SimulatorNetController {
    wods: std::collections::HashMap<u8, WordOfDay>,
    authorized_nets: Vec<NetId>,
}

impl SimulatorNetController {
    pub fn new() -> Self {
        Self {
            wods: std::collections::HashMap::new(),
            authorized_nets: vec![NetId(0), NetId(1), NetId(2)], // Default authorized nets
        }
    }
}

impl Default for SimulatorNetController {
    fn default() -> Self {
        Self::new()
    }
}

impl NetController for SimulatorNetController {
    fn load_wod(&mut self, net_id: NetId, wod: WordOfDay) -> Result<(), HavequickError> {
        if !self.authorized_nets.contains(&net_id) {
            return Err(HavequickError::InvalidNetId);
        }
        self.wods.insert(net_id.0, wod);
        Ok(())
    }

    fn get_wod(&self, net_id: NetId) -> Option<&WordOfDay> {
        self.wods.get(&net_id.0)
    }

    fn is_authorized(&self, net_id: NetId) -> bool {
        self.authorized_nets.contains(&net_id)
    }

    fn get_authorized_nets(&self) -> Vec<NetId> {
        self.authorized_nets.clone()
    }

    fn zeroize(&mut self) {
        self.wods.clear();
    }
}

/// Simulator TRANSEC provider (passthrough - no encryption)
pub struct SimulatorTransec;

impl SimulatorTransec {
    pub fn new() -> Self {
        Self
    }
}

impl Default for SimulatorTransec {
    fn default() -> Self {
        Self::new()
    }
}

impl TransecProvider for SimulatorTransec {
    fn is_available(&self) -> bool {
        false // Simulator has no TRANSEC
    }

    fn encrypt_voice(&self, samples: &[i16]) -> Vec<i16> {
        samples.to_vec() // Passthrough
    }

    fn decrypt_voice(&self, samples: &[i16]) -> Vec<i16> {
        samples.to_vec() // Passthrough
    }

    fn encrypt_data(&self, data: &[u8]) -> Vec<u8> {
        data.to_vec() // Passthrough
    }

    fn decrypt_data(&self, data: &[u8]) -> Vec<u8> {
        data.to_vec() // Passthrough
    }
}

/// Simple AM voice codec for HAVEQUICK
pub struct AmVoiceCodec {
    sample_rate: f64,
    bandwidth: f64,
}

impl AmVoiceCodec {
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            bandwidth: 6_000.0, // 6 kHz voice bandwidth
        }
    }
}

impl Default for AmVoiceCodec {
    fn default() -> Self {
        Self::new(48_000.0)
    }
}

impl VoiceCodec for AmVoiceCodec {
    fn encode(&self, audio: &[f64]) -> Vec<f64> {
        // Simple passthrough for now
        // In real implementation, would apply pre-emphasis, filtering, etc.
        audio.to_vec()
    }

    fn decode(&self, samples: &[f64]) -> Vec<f64> {
        // Simple passthrough
        samples.to_vec()
    }

    fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    fn bandwidth(&self) -> f64 {
        self.bandwidth
    }
}

/// Simple ASK data modem
pub struct AskDataModem {
    #[allow(dead_code)]
    sample_rate: f64,
    data_rate: u32,
    samples_per_bit: usize,
}

impl AskDataModem {
    pub fn new(sample_rate: f64, data_rate: u32) -> Self {
        Self {
            sample_rate,
            data_rate,
            samples_per_bit: (sample_rate / data_rate as f64) as usize,
        }
    }
}

impl Default for AskDataModem {
    fn default() -> Self {
        Self::new(48_000.0, 1200) // 1200 baud default
    }
}

impl DataModem for AskDataModem {
    fn modulate(&self, bits: &[bool]) -> Vec<f64> {
        let mut samples = Vec::with_capacity(bits.len() * self.samples_per_bit);
        for &bit in bits {
            let amplitude = if bit { 1.0 } else { 0.0 };
            for _ in 0..self.samples_per_bit {
                samples.push(amplitude);
            }
        }
        samples
    }

    fn demodulate(&self, samples: &[f64]) -> Vec<bool> {
        let mut bits = Vec::new();
        for chunk in samples.chunks(self.samples_per_bit) {
            let avg: f64 = chunk.iter().sum::<f64>() / chunk.len() as f64;
            bits.push(avg > 0.5);
        }
        bits
    }

    fn data_rate(&self) -> u32 {
        self.data_rate
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulator_hopper() {
        let mut hopper = SimulatorHopper::new();

        // Create test WOD
        let wod = WordOfDay::from_string("123456-789012-345678-901234-567890-123456").unwrap();
        let tod = TimeOfDay::new(2024, 100, 0, 0);

        hopper.initialize(&wod, NetId(0), tod);

        // Should have consistent sequence
        let ch1 = hopper.get_current_channel();
        hopper.advance_hop();
        let ch2 = hopper.get_current_channel();
        hopper.advance_hop();
        let ch3 = hopper.get_current_channel();

        // Channels should be valid
        assert!(ch1.0 < 7000);
        assert!(ch2.0 < 7000);
        assert!(ch3.0 < 7000);

        // Sequence should be repeatable
        let mut hopper2 = SimulatorHopper::new();
        hopper2.initialize(&wod, NetId(0), tod);
        assert_eq!(hopper2.get_current_channel(), ch1);
    }

    #[test]
    fn test_time_sync() {
        let mut sync = SimulatorTimeSync::new();
        assert!(!sync.is_synchronized());

        let tod = TimeOfDay::new(2024, 100, 43200, 0);
        sync.set_tod(tod, TimeSource::Gps);

        assert!(sync.is_synchronized());
        assert_eq!(sync.get_time_uncertainty(), 1); // GPS precision

        sync.advance_time(500_000); // 0.5 seconds
        let new_tod = sync.get_current_tod().unwrap();
        assert_eq!(new_tod.microseconds, 500_000);
    }

    #[test]
    fn test_ask_modem() {
        let modem = AskDataModem::new(48_000.0, 1200);

        let bits = vec![true, false, true, true, false];
        let samples = modem.modulate(&bits);
        let recovered = modem.demodulate(&samples);

        assert_eq!(bits, recovered);
    }
}

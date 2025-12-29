//! Simulator implementations for SINCGARS classified components
//!
//! These implementations provide functional but **NON-SECURE** versions
//! of the classified SINCGARS components. They are intended for:
//!
//! - Development and testing
//! - Educational demonstrations
//! - GUI visualization
//!
//! # WARNING
//!
//! These implementations use deterministic, publicly known algorithms
//! and provide NO security whatsoever. They MUST NOT be used for
//! actual classified communications.

use std::time::Duration;
use super::traits::*;
use super::types::*;

/// Simulator hopping algorithm using LFSR
///
/// Uses a simple Linear Feedback Shift Register for demonstration.
/// The actual SINCGARS hopping algorithm is classified.
#[derive(Debug)]
pub struct SimulatorHopper {
    /// LFSR state
    lfsr_state: u32,
    /// Initial seed (for resync)
    seed: u32,
    /// Current hop number
    hop_number: u64,
    /// Hops per second
    hop_rate: u32,
    /// Whether initialized
    initialized: bool,
    /// Number of channels to use
    num_channels: u16,
}

impl SimulatorHopper {
    pub fn new() -> Self {
        Self {
            lfsr_state: 0,
            seed: 0,
            hop_number: 0,
            hop_rate: 100,
            num_channels: 2320,
            initialized: false,
        }
    }

    /// LFSR step function (non-secure, for simulation only)
    fn lfsr_next(&mut self) -> u32 {
        // 32-bit LFSR with taps at 32, 22, 2, 1 (maximal length)
        let bit = ((self.lfsr_state >> 31) ^ (self.lfsr_state >> 21)
            ^ (self.lfsr_state >> 1) ^ self.lfsr_state) & 1;
        self.lfsr_state = (self.lfsr_state << 1) | bit;
        self.lfsr_state
    }

    /// Generate seed from key material (simulation only)
    fn key_to_seed(key: &TransecKey, net_id: NetId) -> u32 {
        // Simple XOR-based seed derivation (NOT SECURE)
        let mut seed = net_id.0;
        for (i, &byte) in key.material.iter().enumerate() {
            seed ^= (byte as u32) << ((i % 4) * 8);
        }
        // Ensure non-zero
        if seed == 0 { seed = 0xDEADBEEF; }
        seed
    }
}

impl Default for SimulatorHopper {
    fn default() -> Self {
        Self::new()
    }
}

impl HoppingAlgorithm for SimulatorHopper {
    fn initialize(&mut self, key: &TransecKey, net_id: NetId, time: SincgarsTime) {
        self.seed = Self::key_to_seed(key, net_id);
        self.lfsr_state = self.seed;
        self.hop_number = time.to_hop_number(self.hop_rate);

        // Advance LFSR to current hop position
        for _ in 0..self.hop_number {
            self.lfsr_next();
        }

        self.initialized = true;
    }

    fn get_current_channel(&self) -> ChannelNumber {
        ChannelNumber::new((self.lfsr_state % self.num_channels as u32) as u16)
    }

    fn advance_hop(&mut self) {
        self.lfsr_next();
        self.hop_number += 1;
    }

    fn sync_to_time(&mut self, time: SincgarsTime) {
        let target_hop = time.to_hop_number(self.hop_rate);

        // Reset and advance to target position
        self.lfsr_state = self.seed;
        for _ in 0..target_hop {
            self.lfsr_next();
        }
        self.hop_number = target_hop;
    }

    fn time_to_hop_number(&self, time: SincgarsTime) -> u64 {
        time.to_hop_number(self.hop_rate)
    }

    fn is_valid(&self) -> bool {
        self.initialized
    }

    fn reset(&mut self) {
        self.lfsr_state = 0;
        self.seed = 0;
        self.hop_number = 0;
        self.initialized = false;
    }

    fn get_hop_number(&self) -> u64 {
        self.hop_number
    }

    fn peek_channels(&self, count: usize) -> Vec<ChannelNumber> {
        let mut state = self.lfsr_state;
        let mut channels = Vec::with_capacity(count);

        for _ in 0..count {
            channels.push(ChannelNumber::new((state % self.num_channels as u32) as u16));
            // LFSR step
            let bit = ((state >> 31) ^ (state >> 21) ^ (state >> 1) ^ state) & 1;
            state = (state << 1) | bit;
        }

        channels
    }
}

/// Simulator TRANSEC provider
///
/// Accepts any "key" for testing purposes.
#[derive(Debug)]
pub struct SimulatorTransec {
    current_key: Option<TransecKey>,
}

impl SimulatorTransec {
    pub fn new() -> Self {
        Self { current_key: None }
    }

    /// Create with a pre-loaded test key
    pub fn with_test_key() -> Self {
        let key = TransecKey::new(KeyId(1), vec![0x42; 32]);
        Self { current_key: Some(key) }
    }
}

impl Default for SimulatorTransec {
    fn default() -> Self {
        Self::new()
    }
}

impl TransecProvider for SimulatorTransec {
    fn load_key(&mut self, key_id: KeyId) -> Result<TransecKey, TransecError> {
        // In simulator, create a deterministic key from ID
        let material: Vec<u8> = (0..32)
            .map(|i| ((key_id.0 + i as u32) & 0xFF) as u8)
            .collect();
        let key = TransecKey::new(key_id, material);
        self.current_key = Some(key.clone());
        Ok(key)
    }

    fn zeroize(&mut self) {
        self.current_key = None;
    }

    fn has_valid_key(&self) -> bool {
        self.current_key.is_some()
    }

    fn get_key_metadata(&self) -> Option<KeyMetadata> {
        self.current_key.as_ref().map(|k| KeyMetadata {
            id: k.id,
            classification: "UNCLASSIFIED//SIMULATOR".to_string(),
            expires: None,
            key_type: KeyType::Tsk,
        })
    }

    fn derive_session_key(&self, context: &[u8]) -> Result<SessionKey, TransecError> {
        let key = self.current_key.as_ref().ok_or(TransecError::NoKey)?;

        // Simple XOR-based derivation (NOT SECURE)
        let mut material = key.material.clone();
        let len = material.len();
        for (i, byte) in context.iter().enumerate() {
            material[i % len] ^= byte;
        }

        Ok(SessionKey { material })
    }

    fn get_current_key(&self) -> Option<&TransecKey> {
        self.current_key.as_ref()
    }
}

/// Simulator net ID mapper
#[derive(Debug)]
pub struct SimulatorNetMapper {
    authorized_nets: Vec<NetId>,
}

impl SimulatorNetMapper {
    pub fn new() -> Self {
        // Default to allowing nets 1-100
        Self {
            authorized_nets: (1..=100).map(NetId).collect(),
        }
    }

    pub fn with_nets(nets: Vec<NetId>) -> Self {
        Self { authorized_nets: nets }
    }
}

impl Default for SimulatorNetMapper {
    fn default() -> Self {
        Self::new()
    }
}

impl NetIdMapper for SimulatorNetMapper {
    fn get_hopset(&self, net_id: NetId) -> Result<HopsetParams, NetError> {
        if !self.validate_net_id(net_id) {
            return Err(NetError::UnknownNet);
        }

        Ok(HopsetParams {
            net_id,
            num_channels: 2320,
            base_offset: (net_id.0 % 100) as u16,
            hop_rate: 100,
            dwell_time_us: 10_000,
        })
    }

    fn validate_net_id(&self, net_id: NetId) -> bool {
        self.authorized_nets.contains(&net_id)
    }

    fn get_valid_nets(&self) -> Vec<NetId> {
        self.authorized_nets.clone()
    }

    fn nets_compatible(&self, net_a: NetId, net_b: NetId) -> bool {
        // In simulator, all nets are compatible
        self.validate_net_id(net_a) && self.validate_net_id(net_b)
    }

    fn get_default_net(&self) -> Option<NetId> {
        self.authorized_nets.first().copied()
    }
}

/// Simulator time sync protocol
#[derive(Debug)]
pub struct SimulatorTimeSync {
    current_time: SincgarsTime,
    time_source: TimeSource,
    synchronized: bool,
    uncertainty: Duration,
}

impl SimulatorTimeSync {
    pub fn new() -> Self {
        Self {
            current_time: SincgarsTime::new(2024, 1, 0, 0),
            time_source: TimeSource::Internal,
            synchronized: false,
            uncertainty: Duration::from_millis(100),
        }
    }

    pub fn with_time(time: SincgarsTime) -> Self {
        Self {
            current_time: time,
            time_source: TimeSource::Manual,
            synchronized: true,
            uncertainty: Duration::from_millis(1),
        }
    }
}

impl Default for SimulatorTimeSync {
    fn default() -> Self {
        Self::new()
    }
}

impl TimeSyncProtocol for SimulatorTimeSync {
    fn get_current_time(&self) -> SincgarsTime {
        self.current_time
    }

    fn process_sync_burst(&mut self, burst: &SyncBurst) -> Result<(), SyncError> {
        if burst.quality < 0.5 {
            return Err(SyncError::Corrupted);
        }

        self.current_time = burst.time;
        self.synchronized = true;
        self.time_source = TimeSource::Network;
        self.uncertainty = Duration::from_micros(100);

        Ok(())
    }

    fn generate_sync_burst(&self) -> SyncBurst {
        SyncBurst {
            time: self.current_time,
            net_id: NetId(1),
            pattern: vec![0xAA, 0x55, 0xAA, 0x55], // Alternating pattern
            quality: 1.0,
        }
    }

    fn get_time_uncertainty(&self) -> Duration {
        self.uncertainty
    }

    fn is_synchronized(&self) -> bool {
        self.synchronized
    }

    fn set_external_time(&mut self, time: SincgarsTime, source: TimeSource) {
        self.current_time = time;
        self.time_source = source;
        self.synchronized = true;

        self.uncertainty = match source {
            TimeSource::Gps => Duration::from_micros(1),
            TimeSource::Network => Duration::from_micros(100),
            TimeSource::Manual => Duration::from_millis(10),
            TimeSource::Internal => Duration::from_millis(100),
        };
    }

    fn get_time_source(&self) -> TimeSource {
        self.time_source
    }

    fn advance_time(&mut self, duration: Duration) {
        let us = duration.as_micros() as u32;
        let new_us = self.current_time.microseconds + us;

        self.current_time.microseconds = new_us % 1_000_000;
        let new_seconds = self.current_time.seconds + new_us / 1_000_000;

        self.current_time.seconds = new_seconds % 86400;
        if new_seconds >= 86400 {
            self.current_time.day_of_year += (new_seconds / 86400) as u16;
        }
    }
}

/// Simulator crypto provider
///
/// Uses simple XOR "encryption" for demonstration.
/// Provides NO actual security.
#[derive(Debug)]
pub struct SimulatorCrypto {
    session_key: Option<SessionKey>,
    frame_counter: u64,
    ready: bool,
}

impl SimulatorCrypto {
    pub fn new() -> Self {
        Self {
            session_key: None,
            frame_counter: 0,
            ready: false,
        }
    }
}

impl Default for SimulatorCrypto {
    fn default() -> Self {
        Self::new()
    }
}

impl CryptoProvider for SimulatorCrypto {
    fn encrypt(&self, plaintext: &[u8], context: &CryptoContext) -> Result<Vec<u8>, CryptoError> {
        let key = self.session_key.as_ref().ok_or(CryptoError::NoKey)?;

        // Simple XOR "encryption" (NOT SECURE - simulator only)
        let ciphertext: Vec<u8> = plaintext
            .iter()
            .enumerate()
            .map(|(i, &byte)| {
                let key_byte = key.material[i % key.material.len()];
                let frame_byte = (context.frame_number >> ((i % 8) * 8)) as u8;
                byte ^ key_byte ^ frame_byte
            })
            .collect();

        Ok(ciphertext)
    }

    fn decrypt(&self, ciphertext: &[u8], context: &CryptoContext) -> Result<Vec<u8>, CryptoError> {
        // XOR is symmetric
        self.encrypt(ciphertext, context)
    }

    fn generate_sync_vector(&self) -> SyncVector {
        SyncVector {
            iv: vec![0; 16],
            frame_counter: self.frame_counter,
        }
    }

    fn process_sync_vector(&mut self, sync: &SyncVector) -> Result<(), CryptoError> {
        self.frame_counter = sync.frame_counter;
        Ok(())
    }

    fn zeroize(&mut self) {
        self.session_key = None;
        self.frame_counter = 0;
        self.ready = false;
    }

    fn is_ready(&self) -> bool {
        self.ready && self.session_key.is_some()
    }

    fn initialize(&mut self, key: &SessionKey) -> Result<(), CryptoError> {
        self.session_key = Some(SessionKey {
            material: key.material.clone(),
        });
        self.frame_counter = 0;
        self.ready = true;
        Ok(())
    }

    fn get_frame_counter(&self) -> u64 {
        self.frame_counter
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulator_hopper() {
        let mut hopper = SimulatorHopper::new();
        let key = TransecKey::new(KeyId(1), vec![0x42; 32]);
        let time = SincgarsTime::new(2024, 1, 0, 0);

        hopper.initialize(&key, NetId(1), time);
        assert!(hopper.is_valid());

        // Get some channels
        let ch1 = hopper.get_current_channel();
        hopper.advance_hop();
        let ch2 = hopper.get_current_channel();

        // Channels should be different (usually)
        // and within valid range
        assert!(ch1.0 < 2320);
        assert!(ch2.0 < 2320);
    }

    #[test]
    fn test_hopper_resync() {
        let mut hopper = SimulatorHopper::new();
        let key = TransecKey::new(KeyId(1), vec![0x42; 32]);
        let time = SincgarsTime::new(2024, 1, 0, 0);

        hopper.initialize(&key, NetId(1), time);

        // Advance some hops
        for _ in 0..100 {
            hopper.advance_hop();
        }
        let ch_at_100 = hopper.get_current_channel();

        // Resync to same time
        hopper.sync_to_time(time);

        // Advance again
        for _ in 0..100 {
            hopper.advance_hop();
        }
        let ch_at_100_again = hopper.get_current_channel();

        // Should be same channel
        assert_eq!(ch_at_100, ch_at_100_again);
    }

    #[test]
    fn test_simulator_crypto_roundtrip() {
        let mut crypto = SimulatorCrypto::new();
        let key = SessionKey { material: vec![0xAB; 32] };
        crypto.initialize(&key).unwrap();

        let plaintext = b"Hello SINCGARS!";
        let context = CryptoContext {
            frame_number: 42,
            direction: CryptoDirection::Transmit,
            associated_data: vec![],
        };

        let ciphertext = crypto.encrypt(plaintext, &context).unwrap();
        let decrypted = crypto.decrypt(&ciphertext, &context).unwrap();

        assert_eq!(plaintext.as_slice(), decrypted.as_slice());
    }

    #[test]
    fn test_time_sync() {
        let mut sync = SimulatorTimeSync::new();

        let time = SincgarsTime::new(2024, 100, 43200, 0);
        sync.set_external_time(time, TimeSource::Gps);

        assert!(sync.is_synchronized());
        assert_eq!(sync.get_time_source(), TimeSource::Gps);

        // Advance time
        sync.advance_time(Duration::from_secs(1));
        assert_eq!(sync.get_current_time().seconds, 43201);
    }
}

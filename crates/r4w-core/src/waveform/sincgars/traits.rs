//! Trait definitions for classified SINCGARS components
//!
//! These traits define the interface that classified implementations must provide.
//! The framework code uses these traits to interact with classified algorithms
//! without needing to know the implementation details.
//!
//! # Security Note
//!
//! Implementations of these traits contain classified algorithms and must be
//! developed in a secure, access-controlled environment. The simulator
//! implementations provided in this crate are NOT secure and are for
//! development/testing only.

use std::time::Duration;
use super::types::*;

/// Hopping algorithm trait - generates the frequency hopping sequence
///
/// # Security Classification
///
/// Real implementations of this trait are **CLASSIFIED**. The hopping algorithm
/// determines the pseudorandom sequence of frequencies and is critical to
/// SINCGARS' anti-jam and LPI/LPD properties.
///
/// # Implementation Requirements
///
/// - Must generate deterministic sequence from key material
/// - Must support resynchronization to any point in sequence
/// - Must zeroize internal state when reset
pub trait HoppingAlgorithm: Send + Sync {
    /// Initialize the hopper with TRANSEC key material
    ///
    /// This sets up the internal state for generating the hop sequence.
    /// The same key + net_id + time must always produce the same sequence.
    fn initialize(&mut self, key: &TransecKey, net_id: NetId, time: SincgarsTime);

    /// Get the frequency channel for the current hop
    ///
    /// Returns the channel number (0-2319) for the current hop position.
    fn get_current_channel(&self) -> ChannelNumber;

    /// Advance to the next hop in the sequence
    fn advance_hop(&mut self);

    /// Resynchronize to a specific time
    ///
    /// This allows the receiver to jump to any point in the sequence
    /// to match the transmitter's position.
    fn sync_to_time(&mut self, time: SincgarsTime);

    /// Get hop number for given time
    ///
    /// Used by receiver to determine expected hop position.
    fn time_to_hop_number(&self, time: SincgarsTime) -> u64;

    /// Validate that hopper is properly initialized
    fn is_valid(&self) -> bool;

    /// Reset and zeroize internal state
    fn reset(&mut self);

    /// Get the current hop number
    fn get_hop_number(&self) -> u64;

    /// Peek at next N channels without advancing
    fn peek_channels(&self, count: usize) -> Vec<ChannelNumber>;
}

/// TRANSEC key provider - manages key loading and derivation
///
/// # Security Classification
///
/// Real implementations handle **CLASSIFIED** key material and must follow
/// NSA guidelines for COMSEC key management.
///
/// # Implementation Requirements
///
/// - Must support key loading from fill devices (KYK-13, AN/CYZ-10)
/// - Must implement proper key zeroization
/// - Must track key metadata (expiration, classification)
/// - Must support secure key derivation
pub trait TransecProvider: Send + Sync {
    /// Load TRANSEC key from fill device or secure storage
    ///
    /// # Arguments
    ///
    /// * `key_id` - Identifier for the key to load
    ///
    /// # Returns
    ///
    /// The loaded key or an error if loading fails.
    fn load_key(&mut self, key_id: KeyId) -> Result<TransecKey, TransecError>;

    /// Zeroize all key material (secure erase)
    ///
    /// This must securely erase all key material from memory.
    /// Called during key changeover or when radio is zeroed.
    fn zeroize(&mut self);

    /// Check if valid key is loaded
    fn has_valid_key(&self) -> bool;

    /// Get key metadata without exposing key material
    fn get_key_metadata(&self) -> Option<KeyMetadata>;

    /// Derive session key from master key
    ///
    /// Used to create short-term keys for specific sessions.
    fn derive_session_key(&self, context: &[u8]) -> Result<SessionKey, TransecError>;

    /// Get currently loaded key (if any)
    fn get_current_key(&self) -> Option<&TransecKey>;
}

/// Net ID mapper - maps network identifiers to hopset parameters
///
/// # Security Classification
///
/// The mapping between Net IDs and hopsets is **CLASSIFIED** as it reveals
/// network organization and frequency allocation.
///
/// # Implementation Requirements
///
/// - Must validate Net IDs against authorized list
/// - Must provide hopset parameters for authorized nets
/// - Must support net compatibility checking
pub trait NetIdMapper: Send + Sync {
    /// Get hopset parameters for a Net ID
    ///
    /// Returns the frequency hopping parameters associated with this net.
    fn get_hopset(&self, net_id: NetId) -> Result<HopsetParams, NetError>;

    /// Validate Net ID format and membership
    ///
    /// Checks if the Net ID is valid and the user is authorized.
    fn validate_net_id(&self, net_id: NetId) -> bool;

    /// Get list of valid Net IDs for current key
    ///
    /// Returns all nets the current key is authorized for.
    fn get_valid_nets(&self) -> Vec<NetId>;

    /// Check if two Net IDs can communicate
    ///
    /// Returns true if both nets use compatible hopsets.
    fn nets_compatible(&self, net_a: NetId, net_b: NetId) -> bool;

    /// Get the default net for this key
    fn get_default_net(&self) -> Option<NetId>;
}

/// Time synchronization protocol - maintains hop timing
///
/// # Security Classification
///
/// The time synchronization protocol details are **CLASSIFIED** as they
/// could allow an adversary to predict hop timing.
///
/// # Implementation Requirements
///
/// - Must maintain sub-millisecond accuracy
/// - Must support GPS and network time sources
/// - Must handle time uncertainty gracefully
/// - Must generate and process sync bursts
pub trait TimeSyncProtocol: Send + Sync {
    /// Get current SINCGARS time
    fn get_current_time(&self) -> SincgarsTime;

    /// Process received sync burst
    ///
    /// Updates internal time based on received synchronization.
    fn process_sync_burst(&mut self, burst: &SyncBurst) -> Result<(), SyncError>;

    /// Generate sync burst for transmission
    fn generate_sync_burst(&self) -> SyncBurst;

    /// Get time uncertainty
    ///
    /// Returns the uncertainty in timing, used to determine search window.
    fn get_time_uncertainty(&self) -> Duration;

    /// Check if synchronized to net
    fn is_synchronized(&self) -> bool;

    /// Set external time reference
    ///
    /// Updates time from GPS or other external source.
    fn set_external_time(&mut self, time: SincgarsTime, source: TimeSource);

    /// Get the time source currently in use
    fn get_time_source(&self) -> TimeSource;

    /// Advance time by specified duration
    fn advance_time(&mut self, duration: Duration);
}

/// Cryptographic provider - encrypts/decrypts traffic
///
/// # Security Classification
///
/// The cryptographic algorithms are **CLASSIFIED** (Type 1 crypto).
/// Real implementations must use NSA-approved algorithms.
///
/// # Implementation Requirements
///
/// - Must implement Type 1 encryption
/// - Must support crypto sync for late entry
/// - Must properly handle frame counters
/// - Must zeroize crypto state on command
pub trait CryptoProvider: Send + Sync {
    /// Encrypt plaintext for transmission
    fn encrypt(&self, plaintext: &[u8], context: &CryptoContext) -> Result<Vec<u8>, CryptoError>;

    /// Decrypt received ciphertext
    fn decrypt(&self, ciphertext: &[u8], context: &CryptoContext) -> Result<Vec<u8>, CryptoError>;

    /// Generate cryptographic sync vector
    ///
    /// Creates sync information for crypto resynchronization.
    fn generate_sync_vector(&self) -> SyncVector;

    /// Process received sync vector
    ///
    /// Updates crypto state based on received sync.
    fn process_sync_vector(&mut self, sync: &SyncVector) -> Result<(), CryptoError>;

    /// Zeroize all cryptographic state
    fn zeroize(&mut self);

    /// Check if crypto is ready for use
    fn is_ready(&self) -> bool;

    /// Initialize crypto with session key
    fn initialize(&mut self, key: &SessionKey) -> Result<(), CryptoError>;

    /// Get current frame counter
    fn get_frame_counter(&self) -> u64;
}

/// Combined interface for all classified components
///
/// This trait allows passing a single object that implements all
/// classified functionality, if the implementation chooses to
/// combine them.
pub trait ClassifiedModule:
    HoppingAlgorithm + TransecProvider + NetIdMapper + TimeSyncProtocol + CryptoProvider
{
}

// Blanket implementation for types that implement all traits
impl<T> ClassifiedModule for T
where
    T: HoppingAlgorithm + TransecProvider + NetIdMapper + TimeSyncProtocol + CryptoProvider
{
}

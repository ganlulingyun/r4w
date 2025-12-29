//! Trait definitions for HAVEQUICK classified components
//!
//! This module defines the interfaces that must be implemented by
//! authorized parties to provide the classified algorithms for HAVEQUICK.
//!
//! The framework code is unclassified - only the trait implementations
//! contain the actual classified algorithms.

use super::types::*;

/// Hopping algorithm trait - CLASSIFIED IMPLEMENTATION REQUIRED
///
/// The hopping algorithm generates the pseudorandom frequency sequence
/// based on WOD, TOD, and Net ID. The actual algorithm is classified.
pub trait HoppingAlgorithm: Send + Sync {
    /// Initialize the hopper with WOD and Net ID
    fn initialize(&mut self, wod: &WordOfDay, net_id: NetId, tod: TimeOfDay);

    /// Get the channel for the current hop
    fn get_current_channel(&self) -> ChannelNumber;

    /// Advance to the next hop
    fn advance_hop(&mut self);

    /// Resynchronize to a specific time
    fn sync_to_time(&mut self, tod: TimeOfDay);

    /// Get current hop number
    fn current_hop_number(&self) -> u64;

    /// Predict channel for a future hop number
    fn predict_channel(&self, hop_number: u64) -> ChannelNumber;
}

/// Time synchronization protocol trait
///
/// Handles TOD distribution and synchronization between stations.
pub trait TimeSyncProtocol: Send + Sync {
    /// Get current time of day
    fn get_current_tod(&self) -> Option<TimeOfDay>;

    /// Set time from external source (GPS, manual, network)
    fn set_tod(&mut self, tod: TimeOfDay, source: TimeSource);

    /// Check if synchronized
    fn is_synchronized(&self) -> bool;

    /// Get sync status
    fn sync_status(&self) -> SyncStatus;

    /// Advance internal time by given microseconds
    fn advance_time(&mut self, microseconds: u32);

    /// Get time uncertainty in microseconds
    fn get_time_uncertainty(&self) -> u32;
}

/// Net controller trait
///
/// Manages network membership and WOD distribution.
pub trait NetController: Send + Sync {
    /// Load a WOD for a specific net
    fn load_wod(&mut self, net_id: NetId, wod: WordOfDay) -> Result<(), HavequickError>;

    /// Get WOD for a net
    fn get_wod(&self, net_id: NetId) -> Option<&WordOfDay>;

    /// Check if authorized for a net
    fn is_authorized(&self, net_id: NetId) -> bool;

    /// Get all authorized net IDs
    fn get_authorized_nets(&self) -> Vec<NetId>;

    /// Clear all WODs (zeroize)
    fn zeroize(&mut self);
}

/// TRANSEC provider trait (if encryption is used)
///
/// HAVEQUICK itself is not encryption, but radios may use external
/// crypto like KY-58 VINSON. This trait allows integration.
pub trait TransecProvider: Send + Sync {
    /// Check if TRANSEC is available
    fn is_available(&self) -> bool;

    /// Encrypt voice samples (if TRANSEC enabled)
    fn encrypt_voice(&self, samples: &[i16]) -> Vec<i16>;

    /// Decrypt voice samples (if TRANSEC enabled)
    fn decrypt_voice(&self, samples: &[i16]) -> Vec<i16>;

    /// Encrypt data
    fn encrypt_data(&self, data: &[u8]) -> Vec<u8>;

    /// Decrypt data
    fn decrypt_data(&self, data: &[u8]) -> Vec<u8>;
}

/// Voice codec trait
///
/// HAVEQUICK uses AM for voice, but this trait allows for
/// different audio processing chains.
pub trait VoiceCodec: Send + Sync {
    /// Encode audio samples to transmission format
    fn encode(&self, audio: &[f64]) -> Vec<f64>;

    /// Decode received samples to audio
    fn decode(&self, samples: &[f64]) -> Vec<f64>;

    /// Get sample rate in Hz
    fn sample_rate(&self) -> f64;

    /// Get bandwidth in Hz
    fn bandwidth(&self) -> f64;
}

/// Data modem trait
///
/// For ASK data transmission mode.
pub trait DataModem: Send + Sync {
    /// Modulate data bits to samples
    fn modulate(&self, bits: &[bool]) -> Vec<f64>;

    /// Demodulate samples to data bits
    fn demodulate(&self, samples: &[f64]) -> Vec<bool>;

    /// Get data rate in bps
    fn data_rate(&self) -> u32;
}

#[cfg(test)]
mod tests {
    #[allow(unused_imports)]
    use super::*;

    // Traits are tested via their implementations in simulator.rs
}

//! Link-16 Trait Definitions
//!
//! This module defines traits for Link-16 components that have classified
//! implementations. The framework uses these traits to allow swappable
//! implementations for training, simulation, and operational use.

use super::types::*;

/// Frequency hopping pattern generator
///
/// The actual hopping algorithm is COMSEC-controlled. This trait allows
/// simulation implementations for training.
pub trait HoppingPattern: Send + Sync {
    /// Initialize the hopping pattern with crypto variables
    fn initialize(&mut self, crypto_var: &[u8], net_id: u16);

    /// Get the frequency for a given time slot
    fn get_frequency(&self, slot: TimeSlot) -> Frequency;

    /// Get all frequencies for an epoch (16 slots)
    fn get_epoch_frequencies(&self, epoch: u8) -> Vec<Frequency> {
        (0..TimeSlot::SLOTS_PER_EPOCH)
            .filter_map(|slot| TimeSlot::new(epoch, slot))
            .map(|ts| self.get_frequency(ts))
            .collect()
    }

    /// Reset to initial state
    fn reset(&mut self);
}

/// Time synchronization for TDMA
pub trait TimeSync: Send + Sync {
    /// Get current network time
    fn get_network_time(&self) -> NetworkTime;

    /// Set network time from external source (GPS/NTRU)
    fn set_network_time(&mut self, time: NetworkTime);

    /// Check if synchronized to network
    fn is_synchronized(&self) -> bool;

    /// Get time until next slot boundary (microseconds)
    fn time_to_next_slot(&self) -> u64;

    /// Advance time by given microseconds
    fn advance(&mut self, microseconds: u64);
}

/// Message encoder/decoder for J-series messages
pub trait MessageCodec: Send + Sync {
    /// Encode a J-series message to words
    fn encode(&self, msg_type: JSeriesMessage, data: &[u8]) -> Result<Vec<Link16Word>, Link16Error>;

    /// Decode words to message data
    fn decode(&self, words: &[Link16Word]) -> Result<(JSeriesMessage, Vec<u8>), Link16Error>;

    /// Get message type from header word
    fn get_message_type(&self, header: &Link16Word) -> Result<JSeriesMessage, Link16Error>;
}

/// Reed-Solomon error correction
pub trait ErrorCorrection: Send + Sync {
    /// Encode data with Reed-Solomon parity
    fn encode(&self, data: &[u8]) -> Vec<u8>;

    /// Decode and correct errors
    /// Returns (corrected_data, errors_corrected)
    fn decode(&self, data: &[u8]) -> Result<(Vec<u8>, usize), Link16Error>;

    /// Get the error correction capability (t)
    fn correction_capability(&self) -> usize;
}

/// Symbol interleaver for burst error protection
pub trait Interleaver: Send + Sync {
    /// Interleave symbols
    fn interleave(&self, symbols: &[u8]) -> Vec<u8>;

    /// De-interleave symbols
    fn deinterleave(&self, symbols: &[u8]) -> Vec<u8>;

    /// Get interleaver depth
    fn depth(&self) -> usize;
}

/// TRANSEC encryption provider
///
/// Provides transmission security through crypto variables.
/// The actual implementation is classified.
pub trait TransecProvider: Send + Sync {
    /// Load crypto key
    fn load_key(&mut self, key: &[u8]) -> Result<(), Link16Error>;

    /// Encrypt transmission data
    fn encrypt(&self, data: &[u8], slot: TimeSlot) -> Vec<u8>;

    /// Decrypt received data
    fn decrypt(&self, data: &[u8], slot: TimeSlot) -> Result<Vec<u8>, Link16Error>;

    /// Check if crypto is loaded
    fn is_loaded(&self) -> bool;

    /// Get crypto mode
    fn mode(&self) -> CryptoMode;

    /// Zero crypto variables
    fn zeroize(&mut self);
}

/// MSK (Minimum Shift Keying) modulator/demodulator
pub trait MskModem: Send + Sync {
    /// Modulate bits to MSK symbols
    fn modulate(&self, bits: &[bool]) -> Vec<f64>;

    /// Demodulate MSK symbols to bits
    fn demodulate(&self, symbols: &[f64]) -> Vec<bool>;

    /// Get samples per symbol
    fn samples_per_symbol(&self) -> usize;
}

/// Pulse formatting for Link-16 transmission
pub trait PulseFormatter: Send + Sync {
    /// Format data into pulses for transmission
    fn format_pulses(&self, data: &[u8], mode: PulseMode) -> Vec<f64>;

    /// Extract data from received pulses
    fn extract_pulses(&self, pulses: &[f64], mode: PulseMode) -> Vec<u8>;

    /// Get pulses per slot for mode
    fn pulses_per_slot(&self, mode: PulseMode) -> usize {
        match mode {
            PulseMode::Standard => 128,
            PulseMode::Packed2 => 258,
            PulseMode::Packed4 => 516,
        }
    }
}

/// Network participation and slot allocation
pub trait NetworkController: Send + Sync {
    /// Join a network with given parameters
    fn join_network(&mut self, net_id: u16, terminal_id: u8) -> Result<(), Link16Error>;

    /// Leave current network
    fn leave_network(&mut self);

    /// Get assigned time slots for transmission
    fn get_assigned_slots(&self) -> Vec<TimeSlot>;

    /// Check if slot is assigned for transmission
    fn is_tx_slot(&self, slot: TimeSlot) -> bool;

    /// Get active NPGs
    fn get_active_npgs(&self) -> Vec<Npg>;

    /// Subscribe to NPG
    fn subscribe_npg(&mut self, npg: Npg);

    /// Unsubscribe from NPG
    fn unsubscribe_npg(&mut self, npg: Npg);

    /// Get terminal mode
    fn terminal_mode(&self) -> TerminalMode;

    /// Set terminal mode
    fn set_terminal_mode(&mut self, mode: TerminalMode);
}

/// Track database for maintaining situational awareness
pub trait TrackDatabase: Send + Sync {
    /// Add or update a track
    fn update_track(&mut self, stn: SourceTrackNumber, data: &[u8]);

    /// Get track data
    fn get_track(&self, stn: SourceTrackNumber) -> Option<Vec<u8>>;

    /// Remove a track
    fn remove_track(&mut self, stn: SourceTrackNumber);

    /// Get all active tracks
    fn get_all_tracks(&self) -> Vec<SourceTrackNumber>;

    /// Clear all tracks
    fn clear(&mut self);
}

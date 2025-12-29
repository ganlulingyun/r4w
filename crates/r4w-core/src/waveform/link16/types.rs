//! Link-16 Type Definitions
//!
//! This module defines the unclassified types used by Link-16/MIDS/JTIDS.

use std::fmt;

/// Link-16 frequency in L-band (960-1215 MHz)
/// Uses 51 frequencies with 3 MHz spacing
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frequency(pub u8);

impl Frequency {
    /// Base frequency: 969 MHz
    pub const BASE_FREQ_HZ: f64 = 969_000_000.0;
    /// Frequency spacing: 3 MHz
    pub const SPACING_HZ: f64 = 3_000_000.0;
    /// Number of frequencies in the hopping set
    pub const NUM_FREQUENCIES: u8 = 51;

    /// Create a new frequency (0-50)
    pub fn new(index: u8) -> Option<Self> {
        if index < Self::NUM_FREQUENCIES {
            Some(Self(index))
        } else {
            None
        }
    }

    /// Convert to frequency in Hz
    pub fn to_hz(&self) -> f64 {
        Self::BASE_FREQ_HZ + (self.0 as f64) * Self::SPACING_HZ
    }

    /// Convert to frequency in MHz
    pub fn to_mhz(&self) -> f64 {
        self.to_hz() / 1_000_000.0
    }
}

impl fmt::Display for Frequency {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.1} MHz", self.to_mhz())
    }
}

/// Time slot within a Link-16 frame
/// Each frame has 1536 time slots (12.8 minutes)
/// Organized as: 16 slots per epoch, 96 epochs per frame
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimeSlot {
    /// Epoch number (0-95)
    pub epoch: u8,
    /// Slot within epoch (0-15)
    pub slot: u8,
}

impl TimeSlot {
    /// Total slots per frame
    pub const SLOTS_PER_FRAME: u16 = 1536;
    /// Slots per epoch
    pub const SLOTS_PER_EPOCH: u8 = 16;
    /// Epochs per frame
    pub const EPOCHS_PER_FRAME: u8 = 96;
    /// Slot duration in microseconds (7.8125 ms = 7812.5 us)
    pub const SLOT_DURATION_US: f64 = 7812.5;

    /// Create a new time slot
    pub fn new(epoch: u8, slot: u8) -> Option<Self> {
        if epoch < Self::EPOCHS_PER_FRAME && slot < Self::SLOTS_PER_EPOCH {
            Some(Self { epoch, slot })
        } else {
            None
        }
    }

    /// Get absolute slot number (0-1535)
    pub fn absolute(&self) -> u16 {
        (self.epoch as u16) * (Self::SLOTS_PER_EPOCH as u16) + (self.slot as u16)
    }

    /// Create from absolute slot number
    pub fn from_absolute(abs: u16) -> Option<Self> {
        if abs < Self::SLOTS_PER_FRAME {
            Some(Self {
                epoch: (abs / Self::SLOTS_PER_EPOCH as u16) as u8,
                slot: (abs % Self::SLOTS_PER_EPOCH as u16) as u8,
            })
        } else {
            None
        }
    }
}

/// Pulse structure within a time slot
/// Each slot contains 258 pulses in packed-2 mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PulseMode {
    /// Standard single-pulse (128 pulses per slot)
    Standard,
    /// Packed-2 double-pulse (258 pulses per slot)
    Packed2,
    /// Packed-4 quad-pulse (for higher data rates)
    Packed4,
}

impl Default for PulseMode {
    fn default() -> Self {
        Self::Packed2
    }
}

/// Network Participation Group (NPG)
/// Defines which messages a terminal can send/receive
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Npg(pub u8);

impl Npg {
    /// Surveillance NPG
    pub const SURVEILLANCE: Self = Self(1);
    /// Mission Management NPG
    pub const MISSION_MGMT: Self = Self(2);
    /// Air Control NPG
    pub const AIR_CONTROL: Self = Self(3);
    /// Fighter-to-Fighter NPG
    pub const FIGHTER_TO_FIGHTER: Self = Self(7);
    /// Voice NPG A
    pub const VOICE_A: Self = Self(18);
    /// Voice NPG B
    pub const VOICE_B: Self = Self(19);
}

/// Source Track Number (STN) - identifies a track in the network
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SourceTrackNumber {
    /// Source unit ID (5 bits)
    pub source_id: u8,
    /// Track number (10 bits)
    pub track_number: u16,
}

impl SourceTrackNumber {
    /// Create a new STN
    pub fn new(source_id: u8, track_number: u16) -> Self {
        Self {
            source_id: source_id & 0x1F,
            track_number: track_number & 0x3FF,
        }
    }

    /// Pack into 15-bit value
    pub fn pack(&self) -> u16 {
        ((self.source_id as u16) << 10) | self.track_number
    }

    /// Unpack from 15-bit value
    pub fn unpack(value: u16) -> Self {
        Self {
            source_id: ((value >> 10) & 0x1F) as u8,
            track_number: value & 0x3FF,
        }
    }
}

/// J-Series Message Types (unclassified subset)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JSeriesMessage {
    /// J0.0 - Initial Entry
    InitialEntry,
    /// J2.0 - Indirect Interface Unit PPLI
    IndirectPpli,
    /// J2.2 - Air PPLI (Precise Participant Location and Identification)
    AirPpli,
    /// J2.3 - Surface PPLI
    SurfacePpli,
    /// J2.4 - Subsurface PPLI
    SubsurfacePpli,
    /// J2.5 - Land Point PPLI
    LandPointPpli,
    /// J3.0 - Reference Point
    ReferencePoint,
    /// J3.2 - Air Track
    AirTrack,
    /// J3.3 - Surface Track
    SurfaceTrack,
    /// J3.5 - Land Track
    LandTrack,
    /// J7.0 - Track Management
    TrackManagement,
    /// J7.1 - Data Update Request
    DataUpdateRequest,
    /// J12.0 - Mission Assignment
    MissionAssignment,
    /// J28.0 - Free Text
    FreeText,
}

impl JSeriesMessage {
    /// Get the message label (J-number)
    pub fn label(&self) -> &'static str {
        match self {
            Self::InitialEntry => "J0.0",
            Self::IndirectPpli => "J2.0",
            Self::AirPpli => "J2.2",
            Self::SurfacePpli => "J2.3",
            Self::SubsurfacePpli => "J2.4",
            Self::LandPointPpli => "J2.5",
            Self::ReferencePoint => "J3.0",
            Self::AirTrack => "J3.2",
            Self::SurfaceTrack => "J3.3",
            Self::LandTrack => "J3.5",
            Self::TrackManagement => "J7.0",
            Self::DataUpdateRequest => "J7.1",
            Self::MissionAssignment => "J12.0",
            Self::FreeText => "J28.0",
        }
    }

    /// Get word count for message type (standard words)
    pub fn word_count(&self) -> usize {
        match self {
            Self::InitialEntry => 3,
            Self::IndirectPpli | Self::AirPpli | Self::SurfacePpli => 3,
            Self::SubsurfacePpli | Self::LandPointPpli => 3,
            Self::ReferencePoint => 3,
            Self::AirTrack | Self::SurfaceTrack | Self::LandTrack => 3,
            Self::TrackManagement | Self::DataUpdateRequest => 2,
            Self::MissionAssignment => 4,
            Self::FreeText => 6,
        }
    }
}

/// Link-16 word (75 bits)
#[derive(Debug, Clone)]
pub struct Link16Word {
    /// Word data (75 bits, stored in 10 bytes with padding)
    pub data: [u8; 10],
    /// Word type indicator
    pub word_type: WordType,
}

impl Link16Word {
    /// Create a new word with zero data
    pub fn new(word_type: WordType) -> Self {
        Self {
            data: [0u8; 10],
            word_type,
        }
    }
}

/// Word type in Link-16 message
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WordType {
    /// Header word (contains message label)
    Header,
    /// Initial word (first data word)
    Initial,
    /// Extension word
    Extension,
    /// Continuation word
    Continuation,
}

/// Network Time Reference (NTR)
/// GPS-synchronized time used for TDMA
#[derive(Debug, Clone, Copy)]
pub struct NetworkTime {
    /// Epoch within frame (0-95)
    pub epoch: u8,
    /// Time slot within epoch (0-15)
    pub slot: u8,
    /// Pulse within slot
    pub pulse: u16,
    /// Sub-pulse time (nanoseconds)
    pub sub_pulse_ns: u32,
}

impl NetworkTime {
    /// Create from GPS time
    pub fn from_gps_seconds(gps_seconds: f64) -> Self {
        // Frame period is 12.8 minutes = 768 seconds
        let frame_time = gps_seconds % 768.0;
        let slot_time = frame_time / (TimeSlot::SLOT_DURATION_US / 1_000_000.0);
        let absolute_slot = slot_time as u16 % TimeSlot::SLOTS_PER_FRAME;
        let slot_info = TimeSlot::from_absolute(absolute_slot).unwrap_or(TimeSlot { epoch: 0, slot: 0 });

        Self {
            epoch: slot_info.epoch,
            slot: slot_info.slot,
            pulse: 0,
            sub_pulse_ns: 0,
        }
    }

    /// Get current time slot
    pub fn time_slot(&self) -> TimeSlot {
        TimeSlot {
            epoch: self.epoch,
            slot: self.slot,
        }
    }
}

/// Terminal operating mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TerminalMode {
    /// Receive only
    #[default]
    ReceiveOnly,
    /// Active participant (can transmit)
    Active,
    /// Net Time Reference Unit (provides timing)
    Ntru,
    /// Relay mode
    Relay,
}

/// Crypto mode for TRANSEC
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CryptoMode {
    /// No encryption (training/simulation)
    #[default]
    Plain,
    /// Encrypted mode
    Secure,
}

/// Error types for Link-16 operations
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Link16Error {
    /// Invalid frequency index
    InvalidFrequency(u8),
    /// Invalid time slot
    InvalidTimeSlot(u16),
    /// Message encoding error
    EncodingError(String),
    /// Decoding error
    DecodingError(String),
    /// Synchronization error
    SyncError(String),
    /// Crypto key error
    CryptoError(String),
    /// Invalid NPG
    InvalidNpg(u8),
}

impl fmt::Display for Link16Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidFrequency(freq) => write!(f, "Invalid frequency index: {}", freq),
            Self::InvalidTimeSlot(slot) => write!(f, "Invalid time slot: {}", slot),
            Self::EncodingError(msg) => write!(f, "Encoding error: {}", msg),
            Self::DecodingError(msg) => write!(f, "Decoding error: {}", msg),
            Self::SyncError(msg) => write!(f, "Sync error: {}", msg),
            Self::CryptoError(msg) => write!(f, "Crypto error: {}", msg),
            Self::InvalidNpg(npg) => write!(f, "Invalid NPG: {}", npg),
        }
    }
}

impl std::error::Error for Link16Error {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frequency_conversion() {
        let freq = Frequency::new(0).unwrap();
        assert_eq!(freq.to_hz(), 969_000_000.0);

        let freq = Frequency::new(25).unwrap();
        assert_eq!(freq.to_mhz(), 1044.0); // 969 + 25*3 = 1044
    }

    #[test]
    fn test_time_slot() {
        let slot = TimeSlot::new(5, 10).unwrap();
        assert_eq!(slot.absolute(), 5 * 16 + 10);

        let slot2 = TimeSlot::from_absolute(slot.absolute()).unwrap();
        assert_eq!(slot, slot2);
    }

    #[test]
    fn test_source_track_number() {
        let stn = SourceTrackNumber::new(15, 512);
        let packed = stn.pack();
        let unpacked = SourceTrackNumber::unpack(packed);
        assert_eq!(stn.source_id, unpacked.source_id);
        assert_eq!(stn.track_number, unpacked.track_number);
    }
}

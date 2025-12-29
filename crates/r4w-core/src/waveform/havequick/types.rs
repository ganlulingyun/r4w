//! Common types for HAVEQUICK waveform
//!
//! These types are unclassified and represent the data structures used
//! throughout the HAVEQUICK implementation.
//!
//! HAVEQUICK operates in the UHF band (225-400 MHz) with AM modulation
//! for voice and ASK for data.

/// HAVEQUICK channel number (0-6999)
///
/// UHF band: 225.000 MHz to 399.975 MHz, 25 kHz spacing
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ChannelNumber(pub u16);

impl ChannelNumber {
    /// Minimum channel number
    pub const MIN: u16 = 0;
    /// Maximum channel number (7000 channels)
    pub const MAX: u16 = 6999;
    /// Channel spacing in Hz
    pub const SPACING_HZ: f64 = 25_000.0;
    /// Base frequency in Hz (225 MHz)
    pub const BASE_FREQ_HZ: f64 = 225_000_000.0;

    /// Create a new channel number, clamping to valid range
    pub fn new(channel: u16) -> Self {
        Self(channel.min(Self::MAX))
    }

    /// Get the frequency in Hz for this channel
    pub fn to_frequency_hz(&self) -> f64 {
        Self::BASE_FREQ_HZ + (self.0 as f64 * Self::SPACING_HZ)
    }

    /// Create channel from frequency
    pub fn from_frequency_hz(freq: f64) -> Self {
        let channel = ((freq - Self::BASE_FREQ_HZ) / Self::SPACING_HZ).round() as u16;
        Self::new(channel)
    }
}

/// Net ID for HAVEQUICK network (multiple nets can share same WOD)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NetId(pub u8);

impl NetId {
    pub fn new(id: u8) -> Self {
        Self(id)
    }
}

/// HAVEQUICK Time of Day (TOD)
///
/// Typically derived from GPS for accurate synchronization.
/// Resolution to microseconds for precise hop timing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimeOfDay {
    /// Seconds since midnight UTC
    pub seconds: u32,
    /// Microseconds within second
    pub microseconds: u32,
    /// Julian day (day of year, 1-366)
    pub julian_day: u16,
    /// Year
    pub year: u16,
}

impl TimeOfDay {
    pub fn new(year: u16, julian_day: u16, seconds: u32, microseconds: u32) -> Self {
        Self {
            seconds,
            microseconds,
            julian_day,
            year,
        }
    }

    /// Get total microseconds since start of day
    pub fn total_microseconds(&self) -> u64 {
        self.seconds as u64 * 1_000_000 + self.microseconds as u64
    }

    /// Convert to hop number given dwell time in microseconds
    pub fn to_hop_number(&self, dwell_time_us: u32) -> u64 {
        self.total_microseconds() / dwell_time_us as u64
    }
}

/// Word of the Day (WOD)
///
/// The WOD is a TRANSEC variable consisting of six segments of six digits each.
/// The WOD, combined with TOD and Net ID, generates the hopping pattern.
///
/// Format: XXXXXX-XXXXXX-XXXXXX-XXXXXX-XXXXXX-XXXXXX
#[derive(Clone)]
pub struct WordOfDay {
    /// Six segments, each containing 6 digits (0-9)
    segments: [[u8; 6]; 6],
}

impl WordOfDay {
    /// Create a new WOD from 36 digits
    pub fn new(digits: &[u8; 36]) -> Self {
        let mut segments = [[0u8; 6]; 6];
        for (i, segment) in segments.iter_mut().enumerate() {
            for (j, digit) in segment.iter_mut().enumerate() {
                *digit = digits[i * 6 + j] % 10;
            }
        }
        Self { segments }
    }

    /// Create WOD from string format "XXXXXX-XXXXXX-XXXXXX-XXXXXX-XXXXXX-XXXXXX"
    pub fn from_string(s: &str) -> Option<Self> {
        let digits: Vec<u8> = s
            .chars()
            .filter(|c| c.is_ascii_digit())
            .map(|c| c.to_digit(10).unwrap() as u8)
            .collect();

        if digits.len() == 36 {
            let mut arr = [0u8; 36];
            arr.copy_from_slice(&digits);
            Some(Self::new(&arr))
        } else {
            None
        }
    }

    /// Get segment by index (0-5)
    pub fn segment(&self, index: usize) -> Option<&[u8; 6]> {
        self.segments.get(index)
    }

    /// Get all segments as bytes for key derivation
    pub fn as_bytes(&self) -> Vec<u8> {
        self.segments.iter().flatten().copied().collect()
    }
}

impl std::fmt::Debug for WordOfDay {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Don't expose WOD in debug output
        f.debug_struct("WordOfDay")
            .field("segments", &"[REDACTED]")
            .finish()
    }
}

impl Drop for WordOfDay {
    fn drop(&mut self) {
        // Zeroize on drop for security
        for segment in &mut self.segments {
            for digit in segment {
                *digit = 0;
            }
        }
    }
}

/// HAVEQUICK operating mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OperatingMode {
    /// AM Voice mode (standard voice communications)
    AmVoice,
    /// Anti-Jam mode (frequency hopping enabled)
    AntiJam,
    /// Single channel mode (non-hopping)
    SingleChannel,
    /// Tone mode (for testing/alerting)
    Tone,
    /// ADF (Automatic Direction Finding) mode
    Adf,
}

/// Traffic type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrafficType {
    /// Voice traffic (AM modulated)
    Voice,
    /// Data traffic (ASK modulated)
    Data,
    /// Tone signal
    Tone,
}

/// Time source for TOD
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeSource {
    /// GPS-derived time (preferred)
    Gps,
    /// Network-derived time (from master station)
    Network,
    /// Manual operator entry
    Manual,
    /// Free-running internal clock
    Internal,
}

/// Hop rate setting
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HopRate {
    /// Slow hop rate
    Slow,
    /// Medium hop rate
    Medium,
    /// Fast hop rate
    Fast,
}

impl HopRate {
    /// Get dwell time in microseconds
    pub fn dwell_time_us(&self) -> u32 {
        match self {
            // These are approximate values - actual values may vary
            Self::Slow => 100_000,   // 100ms = 10 hops/sec
            Self::Medium => 50_000,  // 50ms = 20 hops/sec
            Self::Fast => 13_300,    // ~13.3ms = 75 hops/sec
        }
    }
}

/// Hopset parameters for HAVEQUICK
#[derive(Debug, Clone)]
pub struct HopsetParams {
    /// Net ID
    pub net_id: NetId,
    /// Hop rate
    pub hop_rate: HopRate,
    /// Number of channels in hopset
    pub num_channels: u16,
}

impl Default for HopsetParams {
    fn default() -> Self {
        Self {
            net_id: NetId(0),
            hop_rate: HopRate::Medium,
            num_channels: 7000,
        }
    }
}

/// Synchronization status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncStatus {
    /// Not synchronized
    NotSynced,
    /// Acquiring sync
    Acquiring,
    /// Synchronized
    Synced,
    /// Lost sync
    LostSync,
}

/// Error types for HAVEQUICK operations
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HavequickError {
    /// No WOD loaded
    NoWod,
    /// No TOD available
    NoTod,
    /// Not synchronized
    NotSynced,
    /// Invalid Net ID
    InvalidNetId,
    /// Invalid WOD format
    InvalidWod,
    /// Time drift exceeds tolerance
    TimeDrift,
    /// Not initialized
    NotInitialized,
    /// Invalid configuration
    InvalidConfiguration(String),
}

impl std::fmt::Display for HavequickError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoWod => write!(f, "No Word of Day loaded"),
            Self::NoTod => write!(f, "No Time of Day available"),
            Self::NotSynced => write!(f, "Not synchronized"),
            Self::InvalidNetId => write!(f, "Invalid Net ID"),
            Self::InvalidWod => write!(f, "Invalid Word of Day format"),
            Self::TimeDrift => write!(f, "Time drift exceeds tolerance"),
            Self::NotInitialized => write!(f, "HAVEQUICK not initialized"),
            Self::InvalidConfiguration(msg) => write!(f, "Invalid configuration: {}", msg),
        }
    }
}

impl std::error::Error for HavequickError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_channel_frequency_conversion() {
        // Channel 0 = 225.000 MHz
        let ch0 = ChannelNumber::new(0);
        assert!((ch0.to_frequency_hz() - 225_000_000.0).abs() < 1.0);

        // Channel 6999 = 399.975 MHz
        let ch_max = ChannelNumber::new(6999);
        assert!((ch_max.to_frequency_hz() - 399_975_000.0).abs() < 1.0);

        // Round trip
        let freq = 300_000_000.0;
        let ch = ChannelNumber::from_frequency_hz(freq);
        assert!((ch.to_frequency_hz() - freq).abs() < 12500.0);
    }

    #[test]
    fn test_time_of_day() {
        let tod = TimeOfDay::new(2024, 100, 43200, 500_000);
        assert_eq!(tod.total_microseconds(), 43200_500_000);

        // 50ms dwell = 20 hops/sec
        let hop = tod.to_hop_number(50_000);
        assert_eq!(hop, 864010);
    }

    #[test]
    fn test_wod_from_string() {
        let wod_str = "123456-789012-345678-901234-567890-123456";
        let wod = WordOfDay::from_string(wod_str).unwrap();
        assert_eq!(wod.segment(0), Some(&[1, 2, 3, 4, 5, 6]));
        assert_eq!(wod.segment(1), Some(&[7, 8, 9, 0, 1, 2]));
    }

    #[test]
    fn test_invalid_wod() {
        // Too short
        assert!(WordOfDay::from_string("12345").is_none());
        // Too long
        assert!(WordOfDay::from_string("1234567890123456789012345678901234567").is_none());
    }
}

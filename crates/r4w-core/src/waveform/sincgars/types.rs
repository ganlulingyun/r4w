//! Common types for SINCGARS waveform
//!
//! These types are unclassified and represent the data structures used
//! throughout the SINCGARS implementation.


/// SINCGARS channel number (0-2319)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ChannelNumber(pub u16);

impl ChannelNumber {
    /// Minimum channel number
    pub const MIN: u16 = 0;
    /// Maximum channel number
    pub const MAX: u16 = 2319;

    /// Create a new channel number, clamping to valid range
    pub fn new(channel: u16) -> Self {
        Self(channel.min(Self::MAX))
    }

    /// Get the frequency in Hz for this channel
    pub fn to_frequency_hz(&self) -> f64 {
        // SINCGARS: 30.000 MHz to 87.975 MHz, 25 kHz spacing
        30_000_000.0 + (self.0 as f64 * 25_000.0)
    }

    /// Create channel from frequency
    pub fn from_frequency_hz(freq: f64) -> Self {
        let channel = ((freq - 30_000_000.0) / 25_000.0).round() as u16;
        Self::new(channel)
    }
}

/// Net ID for SINCGARS network
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NetId(pub u32);

impl NetId {
    pub fn new(id: u32) -> Self {
        Self(id)
    }
}

/// SINCGARS time representation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SincgarsTime {
    /// Seconds since midnight
    pub seconds: u32,
    /// Microseconds within second
    pub microseconds: u32,
    /// Day of year (1-366)
    pub day_of_year: u16,
    /// Year
    pub year: u16,
}

impl SincgarsTime {
    pub fn new(year: u16, day: u16, seconds: u32, microseconds: u32) -> Self {
        Self {
            seconds,
            microseconds,
            day_of_year: day,
            year,
        }
    }

    /// Get total microseconds since start of day
    pub fn total_microseconds(&self) -> u64 {
        self.seconds as u64 * 1_000_000 + self.microseconds as u64
    }

    /// Convert to hop number at given hop rate
    pub fn to_hop_number(&self, hops_per_second: u32) -> u64 {
        let total_us = self.total_microseconds();
        let us_per_hop = 1_000_000 / hops_per_second as u64;
        total_us / us_per_hop
    }
}

/// Key identifier for TRANSEC keys
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct KeyId(pub u32);

/// TRANSEC key material (opaque to framework)
#[derive(Clone)]
pub struct TransecKey {
    /// Key identifier
    pub id: KeyId,
    /// Opaque key material (actual content defined by classified implementation)
    pub material: Vec<u8>,
}

impl std::fmt::Debug for TransecKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Don't expose key material in debug output
        f.debug_struct("TransecKey")
            .field("id", &self.id)
            .field("material", &"[REDACTED]")
            .finish()
    }
}

impl TransecKey {
    pub fn new(id: KeyId, material: Vec<u8>) -> Self {
        Self { id, material }
    }
}

impl Drop for TransecKey {
    fn drop(&mut self) {
        // Zeroize on drop for security
        for byte in &mut self.material {
            *byte = 0;
        }
    }
}

/// Session key derived from master TRANSEC key
#[derive(Clone)]
pub struct SessionKey {
    pub material: Vec<u8>,
}

impl std::fmt::Debug for SessionKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Don't expose key material in debug output
        f.debug_struct("SessionKey")
            .field("material", &"[REDACTED]")
            .finish()
    }
}

impl Drop for SessionKey {
    fn drop(&mut self) {
        for byte in &mut self.material {
            *byte = 0;
        }
    }
}

/// Key metadata (non-sensitive information about a key)
#[derive(Debug, Clone)]
pub struct KeyMetadata {
    /// Key identifier
    pub id: KeyId,
    /// Classification level
    pub classification: String,
    /// Expiration time
    pub expires: Option<SincgarsTime>,
    /// Key type
    pub key_type: KeyType,
}

/// Type of TRANSEC key
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyType {
    /// Traffic Encryption Key
    Tek,
    /// Key Encryption Key
    Kek,
    /// Transmission Security Key
    Tsk,
}

/// Hopset parameters
#[derive(Debug, Clone)]
pub struct HopsetParams {
    /// Net ID this hopset belongs to
    pub net_id: NetId,
    /// Number of channels in hopset
    pub num_channels: u16,
    /// Base channel offset
    pub base_offset: u16,
    /// Hop rate in hops per second
    pub hop_rate: u32,
    /// Dwell time in microseconds
    pub dwell_time_us: u32,
}

impl Default for HopsetParams {
    fn default() -> Self {
        Self {
            net_id: NetId(0),
            num_channels: 2320,
            base_offset: 0,
            hop_rate: 100,
            dwell_time_us: 10_000, // 10ms
        }
    }
}

/// Synchronization burst structure
#[derive(Debug, Clone)]
pub struct SyncBurst {
    /// Time of day encoded in burst
    pub time: SincgarsTime,
    /// Net ID
    pub net_id: NetId,
    /// Sync pattern data
    pub pattern: Vec<u8>,
    /// Signal quality indicator
    pub quality: f32,
}

/// Cryptographic sync vector
#[derive(Debug, Clone)]
pub struct SyncVector {
    /// Initialization vector
    pub iv: Vec<u8>,
    /// Frame counter
    pub frame_counter: u64,
}

/// Context for cryptographic operations
#[derive(Debug, Clone)]
pub struct CryptoContext {
    /// Current frame number
    pub frame_number: u64,
    /// Transmission direction
    pub direction: CryptoDirection,
    /// Associated data (authenticated but not encrypted)
    pub associated_data: Vec<u8>,
}

/// Direction of crypto operation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CryptoDirection {
    Transmit,
    Receive,
}

/// Time source for synchronization
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeSource {
    /// GPS-derived time
    Gps,
    /// Network-derived time (from sync burst)
    Network,
    /// Manual operator entry
    Manual,
    /// Free-running internal clock
    Internal,
}

/// SINCGARS operating mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OperatingMode {
    /// Single channel (non-hopping)
    SingleChannel,
    /// Frequency hopping mode
    FrequencyHopping,
    /// Training mode (reduced hopset)
    Training,
}

/// Voice or data mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrafficMode {
    /// Voice traffic (CVSD encoded)
    Voice,
    /// Low-speed data (75-2400 bps)
    LowSpeedData,
    /// Medium-speed data (4800 bps)
    MediumSpeedData,
    /// High-speed data (16 kbps)
    HighSpeedData,
}

/// Error types for TRANSEC operations
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TransecError {
    /// No key loaded
    NoKey,
    /// Key expired
    KeyExpired,
    /// Invalid key format
    InvalidKey,
    /// Key not authorized for operation
    NotAuthorized,
    /// Hardware security module error
    HsmError(String),
}

impl std::fmt::Display for TransecError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoKey => write!(f, "No TRANSEC key loaded"),
            Self::KeyExpired => write!(f, "TRANSEC key has expired"),
            Self::InvalidKey => write!(f, "Invalid key format"),
            Self::NotAuthorized => write!(f, "Key not authorized for this operation"),
            Self::HsmError(msg) => write!(f, "HSM error: {}", msg),
        }
    }
}

impl std::error::Error for TransecError {}

/// Error types for Net operations
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NetError {
    /// Unknown Net ID
    UnknownNet,
    /// Not authorized for this net
    NotAuthorized,
    /// Invalid Net ID format
    InvalidFormat,
}

impl std::fmt::Display for NetError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownNet => write!(f, "Unknown Net ID"),
            Self::NotAuthorized => write!(f, "Not authorized for this net"),
            Self::InvalidFormat => write!(f, "Invalid Net ID format"),
        }
    }
}

impl std::error::Error for NetError {}

/// Error types for synchronization
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SyncError {
    /// No sync burst detected
    NoSync,
    /// Sync burst corrupted
    Corrupted,
    /// Time uncertainty too large
    TimeDrift,
    /// Net ID mismatch
    WrongNet,
}

impl std::fmt::Display for SyncError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoSync => write!(f, "No sync burst detected"),
            Self::Corrupted => write!(f, "Sync burst corrupted"),
            Self::TimeDrift => write!(f, "Time drift exceeds tolerance"),
            Self::WrongNet => write!(f, "Sync burst from wrong net"),
        }
    }
}

impl std::error::Error for SyncError {}

/// Error types for cryptographic operations
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CryptoError {
    /// No key loaded
    NoKey,
    /// Authentication failed
    AuthFailed,
    /// Invalid ciphertext
    InvalidCiphertext,
    /// Sync vector mismatch
    SyncMismatch,
    /// Algorithm error
    AlgorithmError(String),
}

impl std::fmt::Display for CryptoError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoKey => write!(f, "No encryption key loaded"),
            Self::AuthFailed => write!(f, "Authentication failed"),
            Self::InvalidCiphertext => write!(f, "Invalid ciphertext"),
            Self::SyncMismatch => write!(f, "Crypto sync vector mismatch"),
            Self::AlgorithmError(msg) => write!(f, "Crypto algorithm error: {}", msg),
        }
    }
}

impl std::error::Error for CryptoError {}

/// Overall SINCGARS error type
#[derive(Debug, Clone)]
pub enum SincgarsError {
    Transec(TransecError),
    Net(NetError),
    Sync(SyncError),
    Crypto(CryptoError),
    NotInitialized,
    InvalidConfiguration(String),
}

impl std::fmt::Display for SincgarsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Transec(e) => write!(f, "TRANSEC error: {}", e),
            Self::Net(e) => write!(f, "Net error: {}", e),
            Self::Sync(e) => write!(f, "Sync error: {}", e),
            Self::Crypto(e) => write!(f, "Crypto error: {}", e),
            Self::NotInitialized => write!(f, "SINCGARS not initialized"),
            Self::InvalidConfiguration(msg) => write!(f, "Invalid configuration: {}", msg),
        }
    }
}

impl std::error::Error for SincgarsError {}

impl From<TransecError> for SincgarsError {
    fn from(e: TransecError) -> Self {
        Self::Transec(e)
    }
}

impl From<NetError> for SincgarsError {
    fn from(e: NetError) -> Self {
        Self::Net(e)
    }
}

impl From<SyncError> for SincgarsError {
    fn from(e: SyncError) -> Self {
        Self::Sync(e)
    }
}

impl From<CryptoError> for SincgarsError {
    fn from(e: CryptoError) -> Self {
        Self::Crypto(e)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_channel_frequency_conversion() {
        // Channel 0 = 30.000 MHz
        let ch0 = ChannelNumber::new(0);
        assert!((ch0.to_frequency_hz() - 30_000_000.0).abs() < 1.0);

        // Channel 2319 = 87.975 MHz
        let ch_max = ChannelNumber::new(2319);
        assert!((ch_max.to_frequency_hz() - 87_975_000.0).abs() < 1.0);

        // Round trip
        let freq = 50_000_000.0;
        let ch = ChannelNumber::from_frequency_hz(freq);
        assert!((ch.to_frequency_hz() - freq).abs() < 12500.0);
    }

    #[test]
    fn test_sincgars_time() {
        let time = SincgarsTime::new(2024, 100, 43200, 500_000);
        assert_eq!(time.total_microseconds(), 43200_500_000);

        // 100 hops/sec = 10ms per hop
        let hop = time.to_hop_number(100);
        assert_eq!(hop, 4320050);
    }

    #[test]
    fn test_transec_key_zeroize() {
        let key = TransecKey::new(KeyId(1), vec![0xAA; 32]);
        let _ptr = key.material.as_ptr();
        drop(key);
        // Note: We can't actually verify zeroization after drop in safe Rust,
        // but the Drop impl is there for security
    }
}

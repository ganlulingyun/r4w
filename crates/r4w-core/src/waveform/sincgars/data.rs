//! SINCGARS data mode framing
//!
//! Handles low-speed, medium-speed, and high-speed data modes.
//! The framing structure is unclassified.

use super::types::TrafficMode;

/// SINCGARS data mode configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SincgarsDataMode {
    /// 75 bps (teletype compatible)
    LowSpeed75,
    /// 150 bps
    LowSpeed150,
    /// 300 bps
    LowSpeed300,
    /// 600 bps
    LowSpeed600,
    /// 1200 bps
    LowSpeed1200,
    /// 2400 bps
    LowSpeed2400,
    /// 4800 bps
    MediumSpeed4800,
    /// 16000 bps
    HighSpeed16000,
}

impl SincgarsDataMode {
    /// Get bit rate in bps
    pub fn bit_rate(&self) -> u32 {
        match self {
            Self::LowSpeed75 => 75,
            Self::LowSpeed150 => 150,
            Self::LowSpeed300 => 300,
            Self::LowSpeed600 => 600,
            Self::LowSpeed1200 => 1200,
            Self::LowSpeed2400 => 2400,
            Self::MediumSpeed4800 => 4800,
            Self::HighSpeed16000 => 16000,
        }
    }

    /// Get traffic mode for this data mode
    pub fn traffic_mode(&self) -> TrafficMode {
        match self {
            Self::LowSpeed75 | Self::LowSpeed150 | Self::LowSpeed300
            | Self::LowSpeed600 | Self::LowSpeed1200 | Self::LowSpeed2400 => {
                TrafficMode::LowSpeedData
            }
            Self::MediumSpeed4800 => TrafficMode::MediumSpeedData,
            Self::HighSpeed16000 => TrafficMode::HighSpeedData,
        }
    }

    /// Get frame size in bits for this mode
    pub fn frame_size_bits(&self) -> usize {
        match self {
            Self::LowSpeed75 => 75,
            Self::LowSpeed150 => 150,
            Self::LowSpeed300 => 300,
            Self::LowSpeed600 => 600,
            Self::LowSpeed1200 => 1200,
            Self::LowSpeed2400 => 2400,
            Self::MediumSpeed4800 => 480,
            Self::HighSpeed16000 => 1600,
        }
    }
}

/// Data frame structure
#[derive(Debug, Clone)]
pub struct DataFrame {
    /// Frame sequence number
    pub sequence: u16,
    /// Payload data
    pub payload: Vec<u8>,
    /// CRC-16 checksum
    pub crc: u16,
    /// Frame type
    pub frame_type: FrameType,
}

/// Frame types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameType {
    /// Data frame
    Data,
    /// Acknowledgment
    Ack,
    /// Negative acknowledgment
    Nak,
    /// Synchronization frame
    Sync,
    /// End of transmission
    Eot,
}

/// Data framer for SINCGARS data modes
#[derive(Debug)]
pub struct DataFramer {
    mode: SincgarsDataMode,
    sequence: u16,
    /// FEC enabled
    use_fec: bool,
}

impl DataFramer {
    /// Create new data framer
    pub fn new(mode: SincgarsDataMode) -> Self {
        Self {
            mode,
            sequence: 0,
            use_fec: true,
        }
    }

    /// Set FEC mode
    pub fn with_fec(mut self, enabled: bool) -> Self {
        self.use_fec = enabled;
        self
    }

    /// Get current data mode
    pub fn mode(&self) -> SincgarsDataMode {
        self.mode
    }

    /// Frame data for transmission
    pub fn frame_data(&mut self, data: &[u8]) -> Vec<DataFrame> {
        let payload_size = self.max_payload_size();
        let mut frames = Vec::new();

        for chunk in data.chunks(payload_size) {
            let crc = self.calculate_crc(chunk);

            let frame = DataFrame {
                sequence: self.sequence,
                payload: chunk.to_vec(),
                crc,
                frame_type: FrameType::Data,
            };

            frames.push(frame);
            self.sequence = self.sequence.wrapping_add(1);
        }

        frames
    }

    /// Convert frame to bits for transmission
    pub fn frame_to_bits(&self, frame: &DataFrame) -> Vec<u8> {
        let mut bits = Vec::new();

        // Preamble (sync pattern)
        bits.extend_from_slice(&[0xAA, 0xAA, 0x7E]);

        // Frame type (4 bits) + sequence (12 bits) = 2 bytes
        let header = ((frame.frame_type as u8) << 4) as u16 | (frame.sequence & 0x0FFF);
        bits.push((header >> 8) as u8);
        bits.push((header & 0xFF) as u8);

        // Payload length (1 byte)
        bits.push(frame.payload.len() as u8);

        // Payload
        bits.extend_from_slice(&frame.payload);

        // CRC-16
        bits.push((frame.crc >> 8) as u8);
        bits.push((frame.crc & 0xFF) as u8);

        // Apply FEC if enabled
        if self.use_fec {
            self.apply_fec(&bits)
        } else {
            bits
        }
    }

    /// Parse bits into frame
    pub fn bits_to_frame(&self, bits: &[u8]) -> Result<DataFrame, DataError> {
        // Remove FEC if enabled
        let data = if self.use_fec {
            self.remove_fec(bits)?
        } else {
            bits.to_vec()
        };

        // Check minimum length
        if data.len() < 8 {
            return Err(DataError::TooShort);
        }

        // Check preamble
        if data[0] != 0xAA || data[1] != 0xAA || data[2] != 0x7E {
            return Err(DataError::InvalidPreamble);
        }

        // Parse header
        let header = ((data[3] as u16) << 8) | (data[4] as u16);
        let frame_type = match (header >> 12) & 0x0F {
            0 => FrameType::Data,
            1 => FrameType::Ack,
            2 => FrameType::Nak,
            3 => FrameType::Sync,
            4 => FrameType::Eot,
            _ => return Err(DataError::InvalidFrameType),
        };
        let sequence = (header & 0x0FFF) as u16;

        // Payload length
        let payload_len = data[5] as usize;
        if data.len() < 8 + payload_len {
            return Err(DataError::TooShort);
        }

        // Extract payload
        let payload = data[6..6 + payload_len].to_vec();

        // Extract and verify CRC
        let received_crc = ((data[6 + payload_len] as u16) << 8)
            | (data[7 + payload_len] as u16);
        let calculated_crc = self.calculate_crc(&payload);

        if received_crc != calculated_crc {
            return Err(DataError::CrcMismatch);
        }

        Ok(DataFrame {
            sequence,
            payload,
            crc: received_crc,
            frame_type,
        })
    }

    /// Maximum payload size in bytes
    pub fn max_payload_size(&self) -> usize {
        // Frame overhead: preamble(3) + header(2) + len(1) + crc(2) = 8 bytes
        let frame_bits = self.mode.frame_size_bits();
        let overhead_bits = 8 * 8;
        let payload_bits = frame_bits.saturating_sub(overhead_bits);

        // With FEC, effective payload is halved
        if self.use_fec {
            payload_bits / 16
        } else {
            payload_bits / 8
        }
    }

    /// Calculate CRC-16 (CCITT)
    fn calculate_crc(&self, data: &[u8]) -> u16 {
        let mut crc: u16 = 0xFFFF;

        for byte in data {
            crc ^= (*byte as u16) << 8;
            for _ in 0..8 {
                if crc & 0x8000 != 0 {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }

        crc ^ 0xFFFF
    }

    /// Apply rate-1/2 convolutional FEC
    fn apply_fec(&self, data: &[u8]) -> Vec<u8> {
        // Simple rate-1/2 repetition code for demonstration
        // Real implementation would use proper convolutional coding
        let mut output = Vec::with_capacity(data.len() * 2);

        for &byte in data {
            output.push(byte);
            output.push(byte); // Simple repetition
        }

        output
    }

    /// Remove FEC and correct errors
    fn remove_fec(&self, data: &[u8]) -> Result<Vec<u8>, DataError> {
        if data.len() % 2 != 0 {
            return Err(DataError::InvalidFec);
        }

        // Simple majority voting for repetition code
        let mut output = Vec::with_capacity(data.len() / 2);

        for chunk in data.chunks(2) {
            // In real FEC, we'd do proper decoding
            // For repetition code, just take first byte
            output.push(chunk[0]);
        }

        Ok(output)
    }

    /// Create acknowledgment frame
    pub fn create_ack(&mut self, sequence: u16) -> DataFrame {
        DataFrame {
            sequence,
            payload: vec![],
            crc: 0,
            frame_type: FrameType::Ack,
        }
    }

    /// Create negative acknowledgment frame
    pub fn create_nak(&mut self, sequence: u16) -> DataFrame {
        DataFrame {
            sequence,
            payload: vec![],
            crc: 0,
            frame_type: FrameType::Nak,
        }
    }

    /// Reset framer state
    pub fn reset(&mut self) {
        self.sequence = 0;
    }
}

/// Data framing errors
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DataError {
    /// Frame too short
    TooShort,
    /// Invalid preamble
    InvalidPreamble,
    /// Invalid frame type
    InvalidFrameType,
    /// CRC mismatch
    CrcMismatch,
    /// FEC decoding error
    InvalidFec,
}

impl std::fmt::Display for DataError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TooShort => write!(f, "Frame too short"),
            Self::InvalidPreamble => write!(f, "Invalid preamble"),
            Self::InvalidFrameType => write!(f, "Invalid frame type"),
            Self::CrcMismatch => write!(f, "CRC mismatch"),
            Self::InvalidFec => write!(f, "FEC decoding error"),
        }
    }
}

impl std::error::Error for DataError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_data_modes() {
        assert_eq!(SincgarsDataMode::LowSpeed75.bit_rate(), 75);
        assert_eq!(SincgarsDataMode::HighSpeed16000.bit_rate(), 16000);
    }

    #[test]
    fn test_frame_roundtrip() {
        let mut framer = DataFramer::new(SincgarsDataMode::LowSpeed2400);

        let test_data = b"Hello SINCGARS!";
        let frames = framer.frame_data(test_data);

        assert!(!frames.is_empty());

        for frame in &frames {
            let bits = framer.frame_to_bits(frame);
            let decoded = framer.bits_to_frame(&bits).unwrap();

            assert_eq!(frame.sequence, decoded.sequence);
            assert_eq!(frame.payload, decoded.payload);
        }
    }

    #[test]
    fn test_crc() {
        let framer = DataFramer::new(SincgarsDataMode::LowSpeed2400);

        let data1 = b"test data";
        let data2 = b"test datb"; // Different data

        let crc1 = framer.calculate_crc(data1);
        let crc2 = framer.calculate_crc(data2);

        assert_ne!(crc1, crc2);

        // Same data should give same CRC
        let crc1_again = framer.calculate_crc(data1);
        assert_eq!(crc1, crc1_again);
    }
}

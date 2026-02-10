//! CCSDS Space Packet and Transfer Frame Processing
//!
//! Implements space packet protocol and transfer frame processing per
//! CCSDS 133.0-B-2 (Space Packet Protocol) and CCSDS 132.0-B-3
//! (TM Space Data Link Protocol) for satellite communications.
//!
//! # Features
//!
//! - **Space Packet Protocol**: Encode/decode CCSDS space packets with 6-byte
//!   primary header (version, type flag, APID, sequence control, data length).
//! - **Transfer Frame Processing**: Assemble/disassemble TM transfer frames with
//!   Attached Sync Marker (ASM), frame header, data field, and FECF (CRC-16).
//! - **Virtual Channel Multiplexing**: Route packets to/from multiple virtual
//!   channels within a single physical channel.
//! - **CRC-16 CCITT**: Standard CCSDS Frame Error Control Field calculation.
//! - **Frame Statistics**: Track frame counts, error counts, and packet counts.
//!
//! # Example
//!
//! ```
//! use r4w_core::ccsds_frame_processor::{
//!     CcsdsFrameProcessor, SpacePacket, PacketType,
//! };
//!
//! let mut processor = CcsdsFrameProcessor::new(0x1FF, 1115);
//!
//! // Create and encode a telemetry space packet
//! let packet = SpacePacket {
//!     version: 0,
//!     type_flag: PacketType::Telemetry,
//!     apid: 42,
//!     sequence_flags: 0b11, // standalone
//!     sequence_count: 1,
//!     data: vec![0xDE, 0xAD, 0xBE, 0xEF],
//! };
//!
//! let encoded = CcsdsFrameProcessor::encode_space_packet(&packet);
//! assert_eq!(encoded.len(), 6 + 4); // 6-byte header + 4 data bytes
//!
//! let decoded = CcsdsFrameProcessor::decode_space_packet(&encoded).unwrap();
//! assert_eq!(decoded.apid, 42);
//! assert_eq!(decoded.data, vec![0xDE, 0xAD, 0xBE, 0xEF]);
//!
//! // Wrap in a transfer frame
//! let frame = processor.encode_transfer_frame(0, &encoded);
//! assert!(frame.starts_with(&[0x1A, 0xCF, 0xFC, 0x1D])); // ASM
//!
//! let tf = processor.decode_transfer_frame(&frame).unwrap();
//! assert_eq!(tf.virtual_channel_id, 0);
//! ```

use std::collections::HashMap;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Attached Sync Marker per CCSDS 131.0-B-4
pub const ASM: [u8; 4] = [0x1A, 0xCF, 0xFC, 0x1D];

/// Space Packet primary header length in bytes
pub const SPACE_PACKET_HEADER_LEN: usize = 6;

/// Transfer frame primary header length in bytes (version 1, no secondary header)
pub const TRANSFER_FRAME_HEADER_LEN: usize = 6;

/// FECF (CRC-16) length in bytes
pub const FECF_LEN: usize = 2;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Packet type indicator (bit 4 of first header word).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PacketType {
    /// Telemetry (type = 0)
    Telemetry = 0,
    /// Telecommand (type = 1)
    Telecommand = 1,
}

impl PacketType {
    fn from_bit(bit: u8) -> Self {
        if bit == 0 {
            PacketType::Telemetry
        } else {
            PacketType::Telecommand
        }
    }
}

/// CCSDS Space Packet (CCSDS 133.0-B-2).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SpacePacket {
    /// Packet version number (3 bits, shall be 0)
    pub version: u8,
    /// Packet type: telemetry or telecommand
    pub type_flag: PacketType,
    /// Application Process Identifier (11 bits, 0-2047)
    pub apid: u16,
    /// Sequence flags (2 bits): 0b01=first, 0b00=continuation, 0b10=last, 0b11=standalone
    pub sequence_flags: u8,
    /// Packet sequence count (14 bits, 0-16383)
    pub sequence_count: u16,
    /// Packet data field (user data)
    pub data: Vec<u8>,
}

/// CCSDS TM Transfer Frame (CCSDS 132.0-B-3).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TransferFrame {
    /// Transfer frame version (2 bits, shall be 0)
    pub version: u8,
    /// Spacecraft identifier (10 bits)
    pub spacecraft_id: u16,
    /// Virtual channel identifier (3 bits, 0-7)
    pub virtual_channel_id: u8,
    /// Master/virtual channel frame count (8 bits)
    pub frame_count: u8,
    /// Data field (space packets or idle data)
    pub data_field: Vec<u8>,
    /// Frame Error Control Field (CRC-16-CCITT)
    pub fecf: u16,
}

/// Accumulated statistics for a frame processor.
#[derive(Debug, Clone, Default)]
pub struct FrameStatistics {
    /// Total transfer frames processed
    pub frame_count: u64,
    /// Frames that failed CRC or other validation
    pub error_count: u64,
    /// Total space packets extracted
    pub packet_count: u64,
}

/// Per-virtual-channel state.
#[derive(Debug, Clone, Default)]
struct VirtualChannelState {
    frame_counter: u8,
    packets: Vec<SpacePacket>,
}

// ---------------------------------------------------------------------------
// CRC-16-CCITT
// ---------------------------------------------------------------------------

/// Compute CRC-16-CCITT (polynomial 0x1021, init 0xFFFF) over `data`.
///
/// This is the standard CCSDS FECF algorithm.
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// ---------------------------------------------------------------------------
// CcsdsFrameProcessor
// ---------------------------------------------------------------------------

/// Processor for CCSDS space packet and transfer frame assembly/disassembly.
///
/// Maintains per-virtual-channel state and running statistics.
pub struct CcsdsFrameProcessor {
    /// Spacecraft identifier (10 bits) embedded in transfer frames
    pub spacecraft_id: u16,
    /// Maximum transfer frame data field length (total frame = ASM + header + data + FECF)
    pub frame_data_len: usize,
    /// Running statistics
    pub stats: FrameStatistics,
    /// Per-VC state
    vc_states: HashMap<u8, VirtualChannelState>,
}

impl CcsdsFrameProcessor {
    /// Create a new processor.
    ///
    /// * `spacecraft_id` - 10-bit spacecraft identifier (0-1023).
    /// * `frame_data_len` - maximum length of the data field in a transfer frame.
    pub fn new(spacecraft_id: u16, frame_data_len: usize) -> Self {
        Self {
            spacecraft_id: spacecraft_id & 0x3FF,
            frame_data_len,
            stats: FrameStatistics::default(),
            vc_states: HashMap::new(),
        }
    }

    // ----- Space Packet encode / decode ----------------------------------

    /// Serialize a [`SpacePacket`] into its on-wire byte representation
    /// (6-byte primary header followed by the data field).
    pub fn encode_space_packet(pkt: &SpacePacket) -> Vec<u8> {
        let data_len = pkt.data.len();
        // CCSDS data length field = (number of data bytes) - 1
        let data_length_field = if data_len == 0 { 0u16 } else { (data_len as u16) - 1 };

        let mut buf = Vec::with_capacity(SPACE_PACKET_HEADER_LEN + data_len);

        // Word 0: version(3) | type(1) | sec_hdr_flag(1)=0 | APID(11)
        let w0: u16 = ((pkt.version as u16 & 0x07) << 13)
            | ((pkt.type_flag as u16 & 0x01) << 12)
            | (pkt.apid & 0x7FF);
        buf.push((w0 >> 8) as u8);
        buf.push(w0 as u8);

        // Word 1: seq_flags(2) | seq_count(14)
        let w1: u16 = ((pkt.sequence_flags as u16 & 0x03) << 14)
            | (pkt.sequence_count & 0x3FFF);
        buf.push((w1 >> 8) as u8);
        buf.push(w1 as u8);

        // Word 2: data length - 1
        buf.push((data_length_field >> 8) as u8);
        buf.push(data_length_field as u8);

        buf.extend_from_slice(&pkt.data);
        buf
    }

    /// Parse a byte slice into a [`SpacePacket`].
    ///
    /// Returns `None` if the slice is too short or the length field is
    /// inconsistent with the available data.
    pub fn decode_space_packet(data: &[u8]) -> Option<SpacePacket> {
        if data.len() < SPACE_PACKET_HEADER_LEN {
            return None;
        }

        let w0 = ((data[0] as u16) << 8) | data[1] as u16;
        let w1 = ((data[2] as u16) << 8) | data[3] as u16;
        let w2 = ((data[4] as u16) << 8) | data[5] as u16;

        let version = ((w0 >> 13) & 0x07) as u8;
        let type_flag = PacketType::from_bit(((w0 >> 12) & 0x01) as u8);
        let apid = w0 & 0x7FF;
        let sequence_flags = ((w1 >> 14) & 0x03) as u8;
        let sequence_count = w1 & 0x3FFF;
        let data_len = (w2 as usize) + 1;

        if data.len() < SPACE_PACKET_HEADER_LEN + data_len {
            return None;
        }

        let payload = data[SPACE_PACKET_HEADER_LEN..SPACE_PACKET_HEADER_LEN + data_len].to_vec();

        Some(SpacePacket {
            version,
            type_flag,
            apid,
            sequence_flags,
            sequence_count,
            data: payload,
        })
    }

    // ----- Transfer Frame encode / decode --------------------------------

    /// Assemble a transfer frame carrying `payload` on virtual channel `vc_id`.
    ///
    /// The returned bytes include the 4-byte ASM, 6-byte header, data field
    /// (zero-padded to `frame_data_len`), and 2-byte FECF.
    pub fn encode_transfer_frame(&mut self, vc_id: u8, payload: &[u8]) -> Vec<u8> {
        let vc = self.vc_states.entry(vc_id).or_default();
        let frame_count = vc.frame_counter;
        vc.frame_counter = vc.frame_counter.wrapping_add(1);

        let mut data_field = vec![0u8; self.frame_data_len];
        let copy_len = payload.len().min(self.frame_data_len);
        data_field[..copy_len].copy_from_slice(&payload[..copy_len]);

        // Build header + data for CRC computation
        let mut frame = Vec::with_capacity(
            ASM.len() + TRANSFER_FRAME_HEADER_LEN + self.frame_data_len + FECF_LEN,
        );

        // ASM
        frame.extend_from_slice(&ASM);

        // Header word 0: version(2) | spacecraft_id(10) | vc_id(3) | ocf_flag(1)=0
        let h0: u16 = ((0u16) << 14) // version = 0
            | ((self.spacecraft_id & 0x3FF) << 4)
            | ((vc_id as u16 & 0x07) << 1);
        frame.push((h0 >> 8) as u8);
        frame.push(h0 as u8);

        // Header word 1: master_frame_count(8) | vc_frame_count(8)
        // We use frame_count for both master and VC for simplicity.
        frame.push(frame_count);
        frame.push(frame_count);

        // Header word 2: frame_data_field_status (16 bits)
        // bit 15: secondary header flag = 0
        // bit 14: sync flag = 0
        // bit 13: packet order flag = 0
        // bits 12-11: segment length id = 0b11
        // bits 10-0: first header pointer = 0x0000
        let h2: u16 = 0b0001_1000_0000_0000; // seg_len_id = 11
        frame.push((h2 >> 8) as u8);
        frame.push(h2 as u8);

        // Data field
        frame.extend_from_slice(&data_field);

        // FECF over header + data (everything after ASM)
        let crc = crc16_ccitt(&frame[ASM.len()..]);
        frame.push((crc >> 8) as u8);
        frame.push(crc as u8);

        self.stats.frame_count += 1;

        frame
    }

    /// Parse a transfer frame from `data` (must start with ASM).
    ///
    /// Validates the ASM and CRC. Returns `None` on any failure and
    /// increments the error counter.
    pub fn decode_transfer_frame(&mut self, data: &[u8]) -> Option<TransferFrame> {
        let min_len = ASM.len() + TRANSFER_FRAME_HEADER_LEN + FECF_LEN;
        if data.len() < min_len {
            self.stats.error_count += 1;
            return None;
        }

        // Verify ASM
        if &data[..4] != &ASM {
            self.stats.error_count += 1;
            return None;
        }

        let body = &data[ASM.len()..]; // header + data + fecf

        // Verify CRC over everything except the last 2 bytes
        let frame_without_fecf = &body[..body.len() - FECF_LEN];
        let received_crc =
            ((body[body.len() - 2] as u16) << 8) | body[body.len() - 1] as u16;
        let computed_crc = crc16_ccitt(frame_without_fecf);
        if received_crc != computed_crc {
            self.stats.error_count += 1;
            return None;
        }

        // Parse header
        let h0 = ((body[0] as u16) << 8) | body[1] as u16;
        let version = ((h0 >> 14) & 0x03) as u8;
        let spacecraft_id = (h0 >> 4) & 0x3FF;
        let virtual_channel_id = ((h0 >> 1) & 0x07) as u8;

        let frame_count = body[2]; // master channel frame count

        let data_start = TRANSFER_FRAME_HEADER_LEN;
        let data_end = body.len() - FECF_LEN;
        let data_field = body[data_start..data_end].to_vec();

        self.stats.frame_count += 1;

        Some(TransferFrame {
            version,
            spacecraft_id,
            virtual_channel_id,
            frame_count,
            data_field,
            fecf: received_crc,
        })
    }

    // ----- ASM detection --------------------------------------------------

    /// Scan `stream` for ASM pattern occurrences and return their byte offsets.
    pub fn find_asm_positions(stream: &[u8]) -> Vec<usize> {
        let mut positions = Vec::new();
        if stream.len() < ASM.len() {
            return positions;
        }
        for i in 0..=stream.len() - ASM.len() {
            if stream[i..i + ASM.len()] == ASM {
                positions.push(i);
            }
        }
        positions
    }

    // ----- Virtual Channel mux/demux -------------------------------------

    /// Multiplex a space packet onto the given virtual channel inside a
    /// transfer frame.
    pub fn multiplex(&mut self, vc_id: u8, packet: &SpacePacket) -> Vec<u8> {
        let encoded = Self::encode_space_packet(packet);
        let frame = self.encode_transfer_frame(vc_id, &encoded);
        self.stats.packet_count += 1;

        // Store packet in VC state for demux bookkeeping
        let vc = self.vc_states.entry(vc_id).or_default();
        vc.packets.push(packet.clone());

        frame
    }

    /// Demultiplex a transfer frame: decode the frame, then extract the
    /// space packet from its data field.
    ///
    /// Returns `(TransferFrame, SpacePacket)` on success.
    pub fn demultiplex(
        &mut self,
        data: &[u8],
    ) -> Option<(TransferFrame, SpacePacket)> {
        let tf = self.decode_transfer_frame(data)?;
        let pkt = Self::decode_space_packet(&tf.data_field)?;
        self.stats.packet_count += 1;
        Some((tf, pkt))
    }

    /// Return packets stored for a given virtual channel.
    pub fn get_vc_packets(&self, vc_id: u8) -> &[SpacePacket] {
        match self.vc_states.get(&vc_id) {
            Some(state) => &state.packets,
            None => &[],
        }
    }

    /// Reset all statistics counters.
    pub fn reset_stats(&mut self) {
        self.stats = FrameStatistics::default();
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16_ccitt_known_vectors() {
        // "123456789" is a well-known test vector; CRC-16-CCITT = 0x29B1
        let crc = crc16_ccitt(b"123456789");
        assert_eq!(crc, 0x29B1, "CRC mismatch for standard test vector");
    }

    #[test]
    fn test_crc16_ccitt_empty() {
        let crc = crc16_ccitt(&[]);
        assert_eq!(crc, 0xFFFF, "CRC of empty input should be init value");
    }

    #[test]
    fn test_encode_decode_space_packet_telemetry() {
        let pkt = SpacePacket {
            version: 0,
            type_flag: PacketType::Telemetry,
            apid: 100,
            sequence_flags: 0b11,
            sequence_count: 42,
            data: vec![1, 2, 3, 4, 5],
        };

        let encoded = CcsdsFrameProcessor::encode_space_packet(&pkt);
        assert_eq!(encoded.len(), SPACE_PACKET_HEADER_LEN + 5);

        let decoded = CcsdsFrameProcessor::decode_space_packet(&encoded).unwrap();
        assert_eq!(decoded.version, 0);
        assert_eq!(decoded.type_flag, PacketType::Telemetry);
        assert_eq!(decoded.apid, 100);
        assert_eq!(decoded.sequence_flags, 0b11);
        assert_eq!(decoded.sequence_count, 42);
        assert_eq!(decoded.data, vec![1, 2, 3, 4, 5]);
    }

    #[test]
    fn test_encode_decode_space_packet_telecommand() {
        let pkt = SpacePacket {
            version: 0,
            type_flag: PacketType::Telecommand,
            apid: 2047, // max APID
            sequence_flags: 0b01,
            sequence_count: 16383, // max seq count
            data: vec![0xFF; 10],
        };

        let encoded = CcsdsFrameProcessor::encode_space_packet(&pkt);
        let decoded = CcsdsFrameProcessor::decode_space_packet(&encoded).unwrap();
        assert_eq!(decoded.type_flag, PacketType::Telecommand);
        assert_eq!(decoded.apid, 2047);
        assert_eq!(decoded.sequence_count, 16383);
    }

    #[test]
    fn test_decode_space_packet_too_short() {
        assert!(CcsdsFrameProcessor::decode_space_packet(&[0; 3]).is_none());
    }

    #[test]
    fn test_decode_space_packet_truncated_data() {
        // Header says 10 data bytes but only 5 are present
        let pkt = SpacePacket {
            version: 0,
            type_flag: PacketType::Telemetry,
            apid: 1,
            sequence_flags: 0b11,
            sequence_count: 0,
            data: vec![0; 10],
        };
        let mut encoded = CcsdsFrameProcessor::encode_space_packet(&pkt);
        encoded.truncate(SPACE_PACKET_HEADER_LEN + 5); // chop data
        assert!(CcsdsFrameProcessor::decode_space_packet(&encoded).is_none());
    }

    #[test]
    fn test_encode_decode_transfer_frame() {
        let mut proc = CcsdsFrameProcessor::new(0x1AB, 256);
        let payload = vec![0xCA; 100];
        let frame = proc.encode_transfer_frame(3, &payload);

        // Verify ASM prefix
        assert_eq!(&frame[..4], &ASM);

        // Decode
        let mut proc2 = CcsdsFrameProcessor::new(0x1AB, 256);
        let tf = proc2.decode_transfer_frame(&frame).unwrap();
        assert_eq!(tf.spacecraft_id, 0x1AB);
        assert_eq!(tf.virtual_channel_id, 3);
        assert_eq!(tf.frame_count, 0);
        // First 100 bytes should be our payload, rest zero-padded
        assert_eq!(&tf.data_field[..100], &payload[..]);
        assert!(tf.data_field[100..].iter().all(|&b| b == 0));
    }

    #[test]
    fn test_transfer_frame_crc_tamper() {
        let mut proc = CcsdsFrameProcessor::new(0x100, 64);
        let mut frame = proc.encode_transfer_frame(0, &[0xAA; 32]);

        // Flip a bit in the data area
        frame[10] ^= 0x01;

        let mut proc2 = CcsdsFrameProcessor::new(0x100, 64);
        assert!(
            proc2.decode_transfer_frame(&frame).is_none(),
            "Tampered frame should fail CRC"
        );
        assert_eq!(proc2.stats.error_count, 1);
    }

    #[test]
    fn test_find_asm_positions() {
        let mut stream = vec![0u8; 20];
        // Place ASM at offsets 0 and 12
        stream[0..4].copy_from_slice(&ASM);
        stream[12..16].copy_from_slice(&ASM);
        let positions = CcsdsFrameProcessor::find_asm_positions(&stream);
        assert_eq!(positions, vec![0, 12]);
    }

    #[test]
    fn test_find_asm_positions_empty() {
        assert!(CcsdsFrameProcessor::find_asm_positions(&[]).is_empty());
    }

    #[test]
    fn test_virtual_channel_mux_demux() {
        let mut proc = CcsdsFrameProcessor::new(0x0FF, 256);

        let pkt = SpacePacket {
            version: 0,
            type_flag: PacketType::Telemetry,
            apid: 7,
            sequence_flags: 0b11,
            sequence_count: 99,
            data: vec![10, 20, 30],
        };

        let frame = proc.multiplex(2, &pkt);
        assert_eq!(proc.stats.packet_count, 1);
        assert_eq!(proc.get_vc_packets(2).len(), 1);

        // Demux with a fresh processor
        let mut proc2 = CcsdsFrameProcessor::new(0x0FF, 256);
        let (tf, decoded_pkt) = proc2.demultiplex(&frame).unwrap();
        assert_eq!(tf.virtual_channel_id, 2);
        assert_eq!(decoded_pkt.apid, 7);
        assert_eq!(decoded_pkt.data, vec![10, 20, 30]);
    }

    #[test]
    fn test_frame_counter_wraps() {
        let mut proc = CcsdsFrameProcessor::new(1, 32);
        // Generate 256 frames to wrap the 8-bit counter
        for i in 0..256u32 {
            let frame = proc.encode_transfer_frame(0, &[i as u8]);
            let mut dec = CcsdsFrameProcessor::new(1, 32);
            let tf = dec.decode_transfer_frame(&frame).unwrap();
            assert_eq!(tf.frame_count, (i & 0xFF) as u8);
        }
        // 257th frame should wrap back to 0
        let frame = proc.encode_transfer_frame(0, &[0]);
        let mut dec = CcsdsFrameProcessor::new(1, 32);
        let tf = dec.decode_transfer_frame(&frame).unwrap();
        assert_eq!(tf.frame_count, 0);
    }

    #[test]
    fn test_multiple_virtual_channels() {
        let mut proc = CcsdsFrameProcessor::new(0x200, 128);

        for vc in 0..4u8 {
            let pkt = SpacePacket {
                version: 0,
                type_flag: PacketType::Telemetry,
                apid: vc as u16 * 10,
                sequence_flags: 0b11,
                sequence_count: vc as u16,
                data: vec![vc; 4],
            };
            let frame = proc.multiplex(vc, &pkt);

            let mut dec = CcsdsFrameProcessor::new(0x200, 128);
            let (tf, decoded) = dec.demultiplex(&frame).unwrap();
            assert_eq!(tf.virtual_channel_id, vc);
            assert_eq!(decoded.apid, vc as u16 * 10);
            assert_eq!(decoded.data, vec![vc; 4]);
        }

        assert_eq!(proc.stats.packet_count, 4);
        assert_eq!(proc.stats.frame_count, 4);
    }

    #[test]
    fn test_statistics_reset() {
        let mut proc = CcsdsFrameProcessor::new(1, 64);
        let _ = proc.encode_transfer_frame(0, &[1, 2, 3]);
        assert_eq!(proc.stats.frame_count, 1);
        proc.reset_stats();
        assert_eq!(proc.stats.frame_count, 0);
        assert_eq!(proc.stats.error_count, 0);
        assert_eq!(proc.stats.packet_count, 0);
    }

    #[test]
    fn test_space_packet_single_byte_data() {
        let pkt = SpacePacket {
            version: 0,
            type_flag: PacketType::Telemetry,
            apid: 0,
            sequence_flags: 0b11,
            sequence_count: 0,
            data: vec![0x42],
        };
        let encoded = CcsdsFrameProcessor::encode_space_packet(&pkt);
        // data_length field should be 0 (1 - 1)
        assert_eq!(encoded[4], 0);
        assert_eq!(encoded[5], 0);
        let decoded = CcsdsFrameProcessor::decode_space_packet(&encoded).unwrap();
        assert_eq!(decoded.data, vec![0x42]);
    }
}

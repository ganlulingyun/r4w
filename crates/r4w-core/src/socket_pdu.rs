//! # Socket PDU
//!
//! UDP and TCP PDU source/sink for network-based I/O.
//! Encodes PDUs with a simple length-prefixed framing format
//! for sending/receiving over network sockets. Useful for
//! distributing signal processing across processes or machines.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::socket_pdu::{PduFramer, PduDeframer};
//!
//! let framer = PduFramer::new();
//! let data = vec![0xDE, 0xAD, 0xBE, 0xEF];
//! let frame = framer.frame(&data);
//! assert_eq!(frame.len(), 4 + 4); // 4-byte header + 4 data bytes
//!
//! let mut deframer = PduDeframer::new();
//! let pdus = deframer.feed(&frame);
//! assert_eq!(pdus.len(), 1);
//! assert_eq!(pdus[0], data);
//! ```

/// Maximum PDU size (64 KB).
pub const MAX_PDU_SIZE: usize = 65536;

/// Magic bytes for frame sync (optional).
pub const FRAME_MAGIC: [u8; 2] = [0x52, 0x34]; // "R4"

/// PDU framer: encodes PDUs with length-prefixed framing.
///
/// Frame format: [magic(2)] [length(2, BE)] [payload(N)]
#[derive(Debug, Clone)]
pub struct PduFramer {
    use_magic: bool,
}

impl PduFramer {
    /// Create a new PDU framer.
    pub fn new() -> Self {
        Self { use_magic: true }
    }

    /// Create a framer without magic bytes (simpler format).
    pub fn without_magic() -> Self {
        Self { use_magic: false }
    }

    /// Frame a PDU for transmission.
    pub fn frame(&self, data: &[u8]) -> Vec<u8> {
        let len = data.len().min(MAX_PDU_SIZE) as u16;
        let mut frame = Vec::with_capacity(data.len() + 4);
        if self.use_magic {
            frame.extend_from_slice(&FRAME_MAGIC);
        }
        frame.extend_from_slice(&len.to_be_bytes());
        frame.extend_from_slice(&data[..len as usize]);
        frame
    }

    /// Frame multiple PDUs.
    pub fn frame_batch(&self, pdus: &[Vec<u8>]) -> Vec<u8> {
        let mut output = Vec::new();
        for pdu in pdus {
            output.extend(self.frame(pdu));
        }
        output
    }

    /// Get the header size.
    pub fn header_size(&self) -> usize {
        if self.use_magic { 4 } else { 2 }
    }
}

impl Default for PduFramer {
    fn default() -> Self {
        Self::new()
    }
}

/// PDU deframer: extracts PDUs from a byte stream.
#[derive(Debug, Clone)]
pub struct PduDeframer {
    use_magic: bool,
    buffer: Vec<u8>,
    pdus_extracted: u64,
    bytes_discarded: u64,
}

impl PduDeframer {
    /// Create a new PDU deframer.
    pub fn new() -> Self {
        Self {
            use_magic: true,
            buffer: Vec::new(),
            pdus_extracted: 0,
            bytes_discarded: 0,
        }
    }

    /// Create a deframer without magic bytes.
    pub fn without_magic() -> Self {
        Self {
            use_magic: false,
            buffer: Vec::new(),
            pdus_extracted: 0,
            bytes_discarded: 0,
        }
    }

    /// Feed bytes and extract complete PDUs.
    pub fn feed(&mut self, data: &[u8]) -> Vec<Vec<u8>> {
        self.buffer.extend_from_slice(data);
        let mut pdus = Vec::new();

        loop {
            let header_size = if self.use_magic { 4 } else { 2 };

            if self.buffer.len() < header_size {
                break;
            }

            let offset = if self.use_magic {
                // Find magic bytes.
                match self.find_magic() {
                    Some(pos) => {
                        if pos > 0 {
                            self.bytes_discarded += pos as u64;
                            self.buffer = self.buffer[pos..].to_vec();
                        }
                        0
                    }
                    None => {
                        // Keep last byte (might be start of magic).
                        if self.buffer.len() > 1 {
                            let discard = self.buffer.len() - 1;
                            self.bytes_discarded += discard as u64;
                            self.buffer = self.buffer[discard..].to_vec();
                        }
                        break;
                    }
                }
            } else {
                0
            };

            if self.buffer.len() < offset + header_size {
                break;
            }

            let len_offset = offset + if self.use_magic { 2 } else { 0 };
            let len = u16::from_be_bytes([
                self.buffer[len_offset],
                self.buffer[len_offset + 1],
            ]) as usize;

            if len > MAX_PDU_SIZE {
                // Invalid length, skip magic and try again.
                self.buffer = self.buffer[offset + 1..].to_vec();
                self.bytes_discarded += 1;
                continue;
            }

            let total_frame = header_size + len;
            if self.buffer.len() < offset + total_frame {
                break; // Need more data.
            }

            let pdu_start = offset + header_size;
            let pdu = self.buffer[pdu_start..pdu_start + len].to_vec();
            pdus.push(pdu);
            self.pdus_extracted += 1;
            self.buffer = self.buffer[offset + total_frame..].to_vec();
        }

        pdus
    }

    fn find_magic(&self) -> Option<usize> {
        for i in 0..self.buffer.len().saturating_sub(1) {
            if self.buffer[i] == FRAME_MAGIC[0] && self.buffer[i + 1] == FRAME_MAGIC[1] {
                return Some(i);
            }
        }
        None
    }

    /// Get total PDUs extracted.
    pub fn pdus_extracted(&self) -> u64 {
        self.pdus_extracted
    }

    /// Get bytes discarded (searching for sync).
    pub fn bytes_discarded(&self) -> u64 {
        self.bytes_discarded
    }

    /// Get current buffer size.
    pub fn buffer_len(&self) -> usize {
        self.buffer.len()
    }

    /// Reset the deframer.
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.pdus_extracted = 0;
        self.bytes_discarded = 0;
    }
}

impl Default for PduDeframer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frame_deframe() {
        let framer = PduFramer::new();
        let mut deframer = PduDeframer::new();
        let data = vec![0x01, 0x02, 0x03];
        let frame = framer.frame(&data);
        let pdus = deframer.feed(&frame);
        assert_eq!(pdus.len(), 1);
        assert_eq!(pdus[0], data);
    }

    #[test]
    fn test_multiple_pdus() {
        let framer = PduFramer::new();
        let mut deframer = PduDeframer::new();
        let batch = framer.frame_batch(&[
            vec![0xAA],
            vec![0xBB, 0xCC],
            vec![0xDD, 0xEE, 0xFF],
        ]);
        let pdus = deframer.feed(&batch);
        assert_eq!(pdus.len(), 3);
        assert_eq!(pdus[0], vec![0xAA]);
        assert_eq!(pdus[1], vec![0xBB, 0xCC]);
        assert_eq!(pdus[2], vec![0xDD, 0xEE, 0xFF]);
    }

    #[test]
    fn test_partial_feed() {
        let framer = PduFramer::new();
        let mut deframer = PduDeframer::new();
        let frame = framer.frame(&vec![0x01, 0x02, 0x03]);
        // Feed in two parts.
        let pdus1 = deframer.feed(&frame[..3]);
        assert!(pdus1.is_empty());
        let pdus2 = deframer.feed(&frame[3..]);
        assert_eq!(pdus2.len(), 1);
    }

    #[test]
    fn test_without_magic() {
        let framer = PduFramer::without_magic();
        let mut deframer = PduDeframer::without_magic();
        let data = vec![0xAB, 0xCD];
        let frame = framer.frame(&data);
        assert_eq!(frame.len(), 4); // 2-byte header + 2 data
        let pdus = deframer.feed(&frame);
        assert_eq!(pdus.len(), 1);
        assert_eq!(pdus[0], data);
    }

    #[test]
    fn test_empty_pdu() {
        let framer = PduFramer::new();
        let mut deframer = PduDeframer::new();
        let frame = framer.frame(&[]);
        let pdus = deframer.feed(&frame);
        assert_eq!(pdus.len(), 1);
        assert!(pdus[0].is_empty());
    }

    #[test]
    fn test_garbage_before_frame() {
        let framer = PduFramer::new();
        let mut deframer = PduDeframer::new();
        let frame = framer.frame(&[0x42]);
        let mut data = vec![0xFF, 0xFF, 0xFF]; // Garbage.
        data.extend(frame);
        let pdus = deframer.feed(&data);
        assert_eq!(pdus.len(), 1);
        assert_eq!(pdus[0], vec![0x42]);
        assert!(deframer.bytes_discarded() > 0);
    }

    #[test]
    fn test_header_size() {
        let framer = PduFramer::new();
        assert_eq!(framer.header_size(), 4);
        let framer2 = PduFramer::without_magic();
        assert_eq!(framer2.header_size(), 2);
    }

    #[test]
    fn test_pdus_extracted() {
        let framer = PduFramer::new();
        let mut deframer = PduDeframer::new();
        deframer.feed(&framer.frame(&[1]));
        deframer.feed(&framer.frame(&[2]));
        assert_eq!(deframer.pdus_extracted(), 2);
    }

    #[test]
    fn test_reset() {
        let mut deframer = PduDeframer::new();
        deframer.feed(&[0xFF, 0xFF]);
        assert!(deframer.buffer_len() > 0);
        deframer.reset();
        assert_eq!(deframer.buffer_len(), 0);
        assert_eq!(deframer.pdus_extracted(), 0);
    }

    #[test]
    fn test_large_batch() {
        let framer = PduFramer::new();
        let mut deframer = PduDeframer::new();
        let mut stream = Vec::new();
        for i in 0..100 {
            stream.extend(framer.frame(&vec![i as u8; (i % 10) + 1]));
        }
        let pdus = deframer.feed(&stream);
        assert_eq!(pdus.len(), 100);
    }
}

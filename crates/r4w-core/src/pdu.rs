//! PDU (Protocol Data Unit) Conversion Blocks
//!
//! Blocks for converting between PDU messages and tagged streams,
//! and for inspecting PDU contents. Fundamental for packet-based protocols.
//!
//! ## Blocks
//!
//! - **PduToTaggedStream**: Converts a PDU (byte vector) into a tagged stream with packet_len tags
//! - **TaggedStreamToPdu**: Collects samples between packet_len tags into PDU messages
//! - **MessageDebug**: Captures and displays PDU messages for debugging
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::pdu::{PduToTaggedStream, TaggedStreamToPdu, MessageDebug};
//!
//! let converter = PduToTaggedStream::new("packet_len");
//! let pdus = vec![vec![1u8, 2, 3, 4], vec![5u8, 6]];
//! let (stream, tags) = converter.convert(&pdus);
//! assert_eq!(stream.len(), 6);
//! assert_eq!(tags.len(), 2); // Two packet_len tags
//!
//! let mut collector = TaggedStreamToPdu::new("packet_len");
//! let recovered = collector.process(&stream, &tags);
//! assert_eq!(recovered.len(), 2);
//! assert_eq!(recovered[0], vec![1u8, 2, 3, 4]);
//! ```

use num_complex::Complex64;

/// A PDU message — a variable-length byte vector with optional metadata.
#[derive(Debug, Clone, PartialEq)]
pub struct Pdu {
    /// The raw data bytes.
    pub data: Vec<u8>,
    /// Optional metadata key-value pairs.
    pub metadata: Vec<(String, String)>,
}

impl Pdu {
    pub fn new(data: Vec<u8>) -> Self {
        Self {
            data,
            metadata: Vec::new(),
        }
    }

    pub fn with_metadata(data: Vec<u8>, metadata: Vec<(String, String)>) -> Self {
        Self { data, metadata }
    }

    pub fn len(&self) -> usize {
        self.data.len()
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }
}

/// A tag marking a packet boundary in a tagged stream.
#[derive(Debug, Clone)]
pub struct PacketTag {
    /// Byte offset in the stream where this packet starts.
    pub offset: usize,
    /// Length of the packet in bytes/samples.
    pub length: usize,
}

/// Converts PDU messages into a tagged byte stream.
///
/// Each PDU becomes a contiguous region in the output stream, with a
/// `packet_len` tag at the start of each packet.
#[derive(Debug, Clone)]
pub struct PduToTaggedStream {
    /// Tag key for packet length markers.
    length_tag_key: String,
}

impl PduToTaggedStream {
    pub fn new(length_tag_key: impl Into<String>) -> Self {
        Self {
            length_tag_key: length_tag_key.into(),
        }
    }

    /// Convert a list of PDUs into a contiguous byte stream with packet tags.
    pub fn convert(&self, pdus: &[Vec<u8>]) -> (Vec<u8>, Vec<PacketTag>) {
        let total_len: usize = pdus.iter().map(|p| p.len()).sum();
        let mut stream = Vec::with_capacity(total_len);
        let mut tags = Vec::with_capacity(pdus.len());

        for pdu in pdus {
            tags.push(PacketTag {
                offset: stream.len(),
                length: pdu.len(),
            });
            stream.extend_from_slice(pdu);
        }

        (stream, tags)
    }

    /// Convert PDUs containing complex samples into a tagged IQ stream.
    pub fn convert_complex(&self, pdus: &[Vec<Complex64>]) -> (Vec<Complex64>, Vec<PacketTag>) {
        let total_len: usize = pdus.iter().map(|p| p.len()).sum();
        let mut stream = Vec::with_capacity(total_len);
        let mut tags = Vec::with_capacity(pdus.len());

        for pdu in pdus {
            tags.push(PacketTag {
                offset: stream.len(),
                length: pdu.len(),
            });
            stream.extend_from_slice(pdu);
        }

        (stream, tags)
    }

    /// Convert a single PDU. Returns (stream_slice, tag).
    pub fn convert_single(&self, pdu: &[u8], offset: usize) -> PacketTag {
        PacketTag {
            offset,
            length: pdu.len(),
        }
    }

    pub fn length_tag_key(&self) -> &str {
        &self.length_tag_key
    }
}

/// Collects samples from a tagged stream back into PDU messages.
///
/// Watches for packet_len tags and accumulates that many bytes/samples
/// into each output PDU.
#[derive(Debug, Clone)]
pub struct TaggedStreamToPdu {
    /// Tag key to look for.
    length_tag_key: String,
    /// Partial packet buffer.
    buffer: Vec<u8>,
    /// Expected length of current packet.
    remaining: usize,
}

impl TaggedStreamToPdu {
    pub fn new(length_tag_key: impl Into<String>) -> Self {
        Self {
            length_tag_key: length_tag_key.into(),
            buffer: Vec::new(),
            remaining: 0,
        }
    }

    /// Process a chunk of the tagged stream, returning any complete PDUs.
    pub fn process(&mut self, data: &[u8], tags: &[PacketTag]) -> Vec<Vec<u8>> {
        let mut pdus = Vec::new();

        // Sort tags by offset
        let mut sorted_tags: Vec<&PacketTag> = tags.iter().collect();
        sorted_tags.sort_by_key(|t| t.offset);

        let mut pos = 0;

        for tag in &sorted_tags {
            // If there's data before this tag and we're collecting, flush
            if self.remaining > 0 && pos < tag.offset {
                let take = (tag.offset - pos).min(self.remaining);
                if pos + take <= data.len() {
                    self.buffer.extend_from_slice(&data[pos..pos + take]);
                    self.remaining -= take;
                    pos += take;
                }
                if self.remaining == 0 {
                    pdus.push(std::mem::take(&mut self.buffer));
                }
            }

            // Start new packet
            if tag.offset >= pos && tag.offset < data.len() {
                // Flush any partial buffer
                if !self.buffer.is_empty() {
                    pdus.push(std::mem::take(&mut self.buffer));
                }
                self.remaining = tag.length;
                pos = tag.offset;
            }
        }

        // Consume remaining data for current packet
        if self.remaining > 0 && pos < data.len() {
            let take = (data.len() - pos).min(self.remaining);
            self.buffer.extend_from_slice(&data[pos..pos + take]);
            self.remaining -= take;
            if self.remaining == 0 {
                pdus.push(std::mem::take(&mut self.buffer));
            }
        }

        pdus
    }

    /// Process complex samples from a tagged stream.
    pub fn process_complex(
        &mut self,
        data: &[Complex64],
        tags: &[PacketTag],
    ) -> Vec<Vec<Complex64>> {
        let mut pdus = Vec::new();
        let mut sorted_tags: Vec<&PacketTag> = tags.iter().collect();
        sorted_tags.sort_by_key(|t| t.offset);

        for tag in &sorted_tags {
            if tag.offset + tag.length <= data.len() {
                pdus.push(data[tag.offset..tag.offset + tag.length].to_vec());
            }
        }

        pdus
    }

    pub fn has_partial(&self) -> bool {
        !self.buffer.is_empty()
    }

    pub fn reset(&mut self) {
        self.buffer.clear();
        self.remaining = 0;
    }

    pub fn length_tag_key(&self) -> &str {
        &self.length_tag_key
    }
}

/// Message debugger — captures and displays PDU messages for inspection.
#[derive(Debug, Clone)]
pub struct MessageDebug {
    /// Block name for display.
    name: String,
    /// Captured messages.
    captured: Vec<Pdu>,
    /// Maximum messages to capture (0 = unlimited).
    max_messages: usize,
    /// Display format.
    format: MessageFormat,
}

/// Display format for message debugger.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MessageFormat {
    /// Show hex dump.
    Hex,
    /// Show as UTF-8 text (lossy).
    Text,
    /// Show raw bytes as decimal.
    Decimal,
    /// Show summary only (length).
    Summary,
}

impl MessageDebug {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            captured: Vec::new(),
            max_messages: 0,
            format: MessageFormat::Hex,
        }
    }

    pub fn with_format(name: impl Into<String>, format: MessageFormat) -> Self {
        Self {
            name: name.into(),
            captured: Vec::new(),
            max_messages: 0,
            format,
        }
    }

    /// Capture a PDU message.
    pub fn capture(&mut self, pdu: Pdu) {
        if self.max_messages == 0 || self.captured.len() < self.max_messages {
            self.captured.push(pdu);
        }
    }

    /// Capture raw bytes as a PDU.
    pub fn capture_bytes(&mut self, data: &[u8]) {
        self.capture(Pdu::new(data.to_vec()));
    }

    /// Format a single PDU for display.
    pub fn format_pdu(&self, pdu: &Pdu) -> String {
        match self.format {
            MessageFormat::Hex => {
                let hex: Vec<String> = pdu.data.iter().map(|b| format!("{:02x}", b)).collect();
                format!("[{}: {} bytes] {}", self.name, pdu.len(), hex.join(" "))
            }
            MessageFormat::Text => {
                let text = String::from_utf8_lossy(&pdu.data);
                format!("[{}: {} bytes] \"{}\"", self.name, pdu.len(), text)
            }
            MessageFormat::Decimal => {
                let dec: Vec<String> = pdu.data.iter().map(|b| format!("{}", b)).collect();
                format!("[{}: {} bytes] {}", self.name, pdu.len(), dec.join(", "))
            }
            MessageFormat::Summary => {
                format!("[{}: {} bytes]", self.name, pdu.len())
            }
        }
    }

    /// Get all captured messages.
    pub fn captured(&self) -> &[Pdu] {
        &self.captured
    }

    /// Get a summary of all captured messages.
    pub fn summary(&self) -> String {
        let total_bytes: usize = self.captured.iter().map(|p| p.len()).sum();
        format!(
            "{}: {} messages, {} total bytes",
            self.name,
            self.captured.len(),
            total_bytes
        )
    }

    pub fn reset(&mut self) {
        self.captured.clear();
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn set_max_messages(&mut self, max: usize) {
        self.max_messages = max;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pdu_to_tagged_stream_basic() {
        let converter = PduToTaggedStream::new("packet_len");
        let pdus = vec![vec![1u8, 2, 3, 4], vec![5u8, 6]];
        let (stream, tags) = converter.convert(&pdus);
        assert_eq!(stream, vec![1, 2, 3, 4, 5, 6]);
        assert_eq!(tags.len(), 2);
        assert_eq!(tags[0].offset, 0);
        assert_eq!(tags[0].length, 4);
        assert_eq!(tags[1].offset, 4);
        assert_eq!(tags[1].length, 2);
    }

    #[test]
    fn test_pdu_to_tagged_stream_empty() {
        let converter = PduToTaggedStream::new("packet_len");
        let pdus: Vec<Vec<u8>> = vec![];
        let (stream, tags) = converter.convert(&pdus);
        assert!(stream.is_empty());
        assert!(tags.is_empty());
    }

    #[test]
    fn test_pdu_to_tagged_stream_single() {
        let converter = PduToTaggedStream::new("pkt");
        let pdus = vec![vec![0xDE, 0xAD, 0xBE, 0xEF]];
        let (stream, tags) = converter.convert(&pdus);
        assert_eq!(stream.len(), 4);
        assert_eq!(tags.len(), 1);
        assert_eq!(tags[0].length, 4);
    }

    #[test]
    fn test_pdu_to_tagged_stream_complex() {
        let converter = PduToTaggedStream::new("packet_len");
        let pdus = vec![
            vec![Complex64::new(1.0, 0.0), Complex64::new(0.0, 1.0)],
            vec![Complex64::new(-1.0, 0.0)],
        ];
        let (stream, tags) = converter.convert_complex(&pdus);
        assert_eq!(stream.len(), 3);
        assert_eq!(tags.len(), 2);
        assert_eq!(tags[1].offset, 2);
    }

    #[test]
    fn test_tagged_stream_to_pdu_basic() {
        let mut collector = TaggedStreamToPdu::new("packet_len");
        let data = vec![1u8, 2, 3, 4, 5, 6];
        let tags = vec![
            PacketTag {
                offset: 0,
                length: 4,
            },
            PacketTag {
                offset: 4,
                length: 2,
            },
        ];
        let pdus = collector.process(&data, &tags);
        assert_eq!(pdus.len(), 2);
        assert_eq!(pdus[0], vec![1u8, 2, 3, 4]);
        assert_eq!(pdus[1], vec![5u8, 6]);
    }

    #[test]
    fn test_tagged_stream_to_pdu_complex() {
        let mut collector = TaggedStreamToPdu::new("packet_len");
        let data = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(2.0, 0.0),
            Complex64::new(3.0, 0.0),
        ];
        let tags = vec![
            PacketTag {
                offset: 0,
                length: 2,
            },
            PacketTag {
                offset: 2,
                length: 1,
            },
        ];
        let pdus = collector.process_complex(&data, &tags);
        assert_eq!(pdus.len(), 2);
        assert_eq!(pdus[0].len(), 2);
        assert_eq!(pdus[1].len(), 1);
    }

    #[test]
    fn test_roundtrip_byte_pdus() {
        let converter = PduToTaggedStream::new("packet_len");
        let original = vec![vec![10u8, 20, 30], vec![40u8, 50], vec![60u8]];
        let (stream, tags) = converter.convert(&original);

        let mut collector = TaggedStreamToPdu::new("packet_len");
        let recovered = collector.process(&stream, &tags);

        assert_eq!(recovered.len(), 3);
        assert_eq!(recovered[0], vec![10, 20, 30]);
        assert_eq!(recovered[1], vec![40, 50]);
        assert_eq!(recovered[2], vec![60]);
    }

    #[test]
    fn test_roundtrip_complex_pdus() {
        let converter = PduToTaggedStream::new("pkt");
        let original = vec![
            vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, 4.0)],
            vec![Complex64::new(5.0, 6.0)],
        ];
        let (stream, tags) = converter.convert_complex(&original);

        let mut collector = TaggedStreamToPdu::new("pkt");
        let recovered = collector.process_complex(&stream, &tags);
        assert_eq!(recovered.len(), 2);
        assert_eq!(recovered[0], original[0]);
        assert_eq!(recovered[1], original[1]);
    }

    #[test]
    fn test_message_debug_hex() {
        let mut dbg = MessageDebug::new("test");
        dbg.capture_bytes(&[0xDE, 0xAD, 0xBE, 0xEF]);
        let formatted = dbg.format_pdu(&dbg.captured()[0].clone());
        assert!(formatted.contains("de ad be ef"));
        assert!(formatted.contains("4 bytes"));
    }

    #[test]
    fn test_message_debug_text() {
        let mut dbg = MessageDebug::with_format("test", MessageFormat::Text);
        dbg.capture_bytes(b"Hello");
        let formatted = dbg.format_pdu(&dbg.captured()[0].clone());
        assert!(formatted.contains("Hello"));
    }

    #[test]
    fn test_message_debug_summary() {
        let mut dbg = MessageDebug::new("rx");
        dbg.capture_bytes(&[1, 2, 3]);
        dbg.capture_bytes(&[4, 5]);
        let summary = dbg.summary();
        assert!(summary.contains("2 messages"));
        assert!(summary.contains("5 total bytes"));
    }

    #[test]
    fn test_message_debug_max_messages() {
        let mut dbg = MessageDebug::new("test");
        dbg.set_max_messages(2);
        dbg.capture_bytes(&[1]);
        dbg.capture_bytes(&[2]);
        dbg.capture_bytes(&[3]); // Should be dropped
        assert_eq!(dbg.captured().len(), 2);
    }

    #[test]
    fn test_message_debug_reset() {
        let mut dbg = MessageDebug::new("test");
        dbg.capture_bytes(&[1, 2, 3]);
        assert_eq!(dbg.captured().len(), 1);
        dbg.reset();
        assert!(dbg.captured().is_empty());
    }

    #[test]
    fn test_pdu_metadata() {
        let pdu = Pdu::with_metadata(
            vec![1, 2, 3],
            vec![("source".to_string(), "uart".to_string())],
        );
        assert_eq!(pdu.len(), 3);
        assert_eq!(pdu.metadata.len(), 1);
        assert_eq!(pdu.metadata[0].0, "source");
    }

    #[test]
    fn test_pdu_convert_single() {
        let converter = PduToTaggedStream::new("pkt_len");
        let tag = converter.convert_single(&[1, 2, 3, 4], 100);
        assert_eq!(tag.offset, 100);
        assert_eq!(tag.length, 4);
    }
}

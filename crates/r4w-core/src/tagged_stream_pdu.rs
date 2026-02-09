//! Tagged Stream to PDU — Packet stream bridge
//!
//! Converts between continuous tagged streams (with length tags) and
//! discrete PDU messages. Essential for packet-based protocols.
//! GNU Radio equivalents: `tagged_stream_to_pdu`, `pdu_to_tagged_stream`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::tagged_stream_pdu::{TaggedStreamToPdu, PduToTaggedStream, Pdu, StreamTag};
//!
//! // Convert a tagged stream to PDUs
//! let mut ts2pdu = TaggedStreamToPdu::new("packet_len");
//! let tags = vec![StreamTag::new(0, "packet_len", 5)];
//! let data = vec![1u8, 2, 3, 4, 5, 6, 7, 8];
//! let pdus = ts2pdu.process(&data, &tags);
//! assert_eq!(pdus.len(), 1);
//! assert_eq!(pdus[0].payload, vec![1, 2, 3, 4, 5]);
//!
//! // Convert PDU back to tagged stream
//! let mut pdu2ts = PduToTaggedStream::new("packet_len");
//! let (data, tags) = pdu2ts.process(&pdus[0]);
//! assert_eq!(data.len(), 5);
//! assert_eq!(tags[0].value, 5);
//! ```

use std::collections::HashMap;

/// A stream tag with position, key, and integer value.
#[derive(Debug, Clone, PartialEq)]
pub struct StreamTag {
    /// Offset (sample index) where the tag applies.
    pub offset: usize,
    /// Tag key name.
    pub key: String,
    /// Tag value (integer).
    pub value: usize,
}

impl StreamTag {
    /// Create a new stream tag.
    pub fn new(offset: usize, key: &str, value: usize) -> Self {
        Self {
            offset,
            key: key.to_string(),
            value,
        }
    }
}

/// A Protocol Data Unit with payload and metadata.
#[derive(Debug, Clone, PartialEq)]
pub struct Pdu {
    /// Payload bytes.
    pub payload: Vec<u8>,
    /// Metadata dictionary.
    pub metadata: HashMap<String, String>,
}

impl Pdu {
    /// Create from payload only.
    pub fn new(payload: Vec<u8>) -> Self {
        Self {
            payload,
            metadata: HashMap::new(),
        }
    }

    /// Create with metadata.
    pub fn with_metadata(payload: Vec<u8>, metadata: HashMap<String, String>) -> Self {
        Self { payload, metadata }
    }

    /// Get payload length.
    pub fn len(&self) -> usize {
        self.payload.len()
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.payload.is_empty()
    }

    /// Set a metadata field.
    pub fn set_meta(&mut self, key: &str, value: &str) {
        self.metadata.insert(key.to_string(), value.to_string());
    }

    /// Get a metadata field.
    pub fn get_meta(&self, key: &str) -> Option<&str> {
        self.metadata.get(key).map(|s| s.as_str())
    }
}

/// Converts tagged streams to PDU messages.
///
/// Accumulates samples until a length tag is satisfied, then emits a PDU.
#[derive(Debug, Clone)]
pub struct TaggedStreamToPdu {
    /// Tag key that indicates packet length.
    length_tag_key: String,
    /// Accumulation buffer.
    buffer: Vec<u8>,
    /// Expected packet length (from tag).
    expected_len: Option<usize>,
    /// Total PDUs emitted.
    pdu_count: u64,
}

impl TaggedStreamToPdu {
    /// Create with a length tag key.
    pub fn new(length_tag_key: &str) -> Self {
        Self {
            length_tag_key: length_tag_key.to_string(),
            buffer: Vec::new(),
            expected_len: None,
            pdu_count: 0,
        }
    }

    /// Process a block of data with associated tags.
    ///
    /// Returns completed PDUs. Accumulates partial packets across calls.
    pub fn process(&mut self, data: &[u8], tags: &[StreamTag]) -> Vec<Pdu> {
        let mut pdus = Vec::new();
        let mut pos = 0;

        while pos < data.len() {
            // Check for length tag at current position
            for tag in tags {
                if tag.offset == pos && tag.key == self.length_tag_key {
                    // Start of new packet
                    if !self.buffer.is_empty() && self.expected_len.is_some() {
                        // Emit partial packet from previous tag
                        let pdu = Pdu::new(self.buffer.drain(..).collect());
                        pdus.push(pdu);
                        self.pdu_count += 1;
                    }
                    self.expected_len = Some(tag.value);
                    self.buffer.clear();
                }
            }

            // Accumulate data
            if let Some(expected) = self.expected_len {
                let remaining = expected - self.buffer.len();
                let available = data.len() - pos;
                let take = remaining.min(available);
                self.buffer.extend_from_slice(&data[pos..pos + take]);
                pos += take;

                // Check if packet is complete
                if self.buffer.len() >= expected {
                    let pdu = Pdu::new(self.buffer.drain(..).collect());
                    pdus.push(pdu);
                    self.pdu_count += 1;
                    self.expected_len = None;
                }
            } else {
                // No active tag, skip data
                pos += 1;
            }
        }

        pdus
    }

    /// Get total PDUs emitted.
    pub fn pdu_count(&self) -> u64 {
        self.pdu_count
    }

    /// Get the length tag key.
    pub fn length_tag_key(&self) -> &str {
        &self.length_tag_key
    }

    /// Reset accumulator.
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.expected_len = None;
    }
}

/// Converts PDU messages to tagged streams.
///
/// Each PDU becomes a tagged burst with a length tag at the start.
#[derive(Debug, Clone)]
pub struct PduToTaggedStream {
    /// Tag key for length.
    length_tag_key: String,
    /// Total PDUs processed.
    pdu_count: u64,
}

impl PduToTaggedStream {
    /// Create with a length tag key.
    pub fn new(length_tag_key: &str) -> Self {
        Self {
            length_tag_key: length_tag_key.to_string(),
            pdu_count: 0,
        }
    }

    /// Convert a single PDU to tagged stream data.
    ///
    /// Returns `(data, tags)`.
    pub fn process(&mut self, pdu: &Pdu) -> (Vec<u8>, Vec<StreamTag>) {
        let tag = StreamTag::new(0, &self.length_tag_key, pdu.payload.len());
        self.pdu_count += 1;
        (pdu.payload.clone(), vec![tag])
    }

    /// Convert multiple PDUs to a concatenated tagged stream.
    ///
    /// Returns `(data, tags)` with tags at correct offsets.
    pub fn process_batch(&mut self, pdus: &[Pdu]) -> (Vec<u8>, Vec<StreamTag>) {
        let mut data = Vec::new();
        let mut tags = Vec::new();
        for pdu in pdus {
            let offset = data.len();
            tags.push(StreamTag::new(offset, &self.length_tag_key, pdu.payload.len()));
            data.extend_from_slice(&pdu.payload);
            self.pdu_count += 1;
        }
        (data, tags)
    }

    /// Get total PDUs processed.
    pub fn pdu_count(&self) -> u64 {
        self.pdu_count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pdu_creation() {
        let pdu = Pdu::new(vec![1, 2, 3]);
        assert_eq!(pdu.len(), 3);
        assert!(!pdu.is_empty());
    }

    #[test]
    fn test_pdu_metadata() {
        let mut pdu = Pdu::new(vec![1, 2]);
        pdu.set_meta("type", "data");
        assert_eq!(pdu.get_meta("type"), Some("data"));
        assert_eq!(pdu.get_meta("missing"), None);
    }

    #[test]
    fn test_ts_to_pdu_single_packet() {
        let mut ts2pdu = TaggedStreamToPdu::new("packet_len");
        let tags = vec![StreamTag::new(0, "packet_len", 4)];
        let data = vec![10, 20, 30, 40];
        let pdus = ts2pdu.process(&data, &tags);
        assert_eq!(pdus.len(), 1);
        assert_eq!(pdus[0].payload, vec![10, 20, 30, 40]);
    }

    #[test]
    fn test_ts_to_pdu_multiple_packets() {
        let mut ts2pdu = TaggedStreamToPdu::new("packet_len");
        let tags = vec![
            StreamTag::new(0, "packet_len", 3),
            StreamTag::new(3, "packet_len", 2),
        ];
        let data = vec![1, 2, 3, 4, 5];
        let pdus = ts2pdu.process(&data, &tags);
        assert_eq!(pdus.len(), 2);
        assert_eq!(pdus[0].payload, vec![1, 2, 3]);
        assert_eq!(pdus[1].payload, vec![4, 5]);
    }

    #[test]
    fn test_ts_to_pdu_streaming() {
        let mut ts2pdu = TaggedStreamToPdu::new("packet_len");
        // Tag says 6 bytes, but only 3 available
        let tags = vec![StreamTag::new(0, "packet_len", 6)];
        let pdus1 = ts2pdu.process(&[1, 2, 3], &tags);
        assert!(pdus1.is_empty()); // Not complete yet
        // Continue
        let pdus2 = ts2pdu.process(&[4, 5, 6], &[]);
        assert_eq!(pdus2.len(), 1);
        assert_eq!(pdus2[0].payload, vec![1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn test_ts_to_pdu_no_tags() {
        let mut ts2pdu = TaggedStreamToPdu::new("packet_len");
        let pdus = ts2pdu.process(&[1, 2, 3, 4], &[]);
        assert!(pdus.is_empty()); // No length tag, data is skipped
    }

    #[test]
    fn test_pdu_to_ts_single() {
        let mut pdu2ts = PduToTaggedStream::new("packet_len");
        let pdu = Pdu::new(vec![10, 20, 30]);
        let (data, tags) = pdu2ts.process(&pdu);
        assert_eq!(data, vec![10, 20, 30]);
        assert_eq!(tags.len(), 1);
        assert_eq!(tags[0].key, "packet_len");
        assert_eq!(tags[0].value, 3);
        assert_eq!(tags[0].offset, 0);
    }

    #[test]
    fn test_pdu_to_ts_batch() {
        let mut pdu2ts = PduToTaggedStream::new("packet_len");
        let pdus = vec![
            Pdu::new(vec![1, 2]),
            Pdu::new(vec![3, 4, 5]),
        ];
        let (data, tags) = pdu2ts.process_batch(&pdus);
        assert_eq!(data, vec![1, 2, 3, 4, 5]);
        assert_eq!(tags.len(), 2);
        assert_eq!(tags[0].offset, 0);
        assert_eq!(tags[0].value, 2);
        assert_eq!(tags[1].offset, 2);
        assert_eq!(tags[1].value, 3);
    }

    #[test]
    fn test_roundtrip() {
        let mut ts2pdu = TaggedStreamToPdu::new("pkt_len");
        let mut pdu2ts = PduToTaggedStream::new("pkt_len");

        let original_pdu = Pdu::new(vec![0xAA, 0xBB, 0xCC, 0xDD]);

        // PDU → tagged stream → PDU
        let (data, tags) = pdu2ts.process(&original_pdu);
        let pdus = ts2pdu.process(&data, &tags);
        assert_eq!(pdus.len(), 1);
        assert_eq!(pdus[0].payload, original_pdu.payload);
    }

    #[test]
    fn test_ts_to_pdu_count() {
        let mut ts2pdu = TaggedStreamToPdu::new("packet_len");
        let tags = vec![StreamTag::new(0, "packet_len", 2)];
        ts2pdu.process(&[1, 2], &tags);
        assert_eq!(ts2pdu.pdu_count(), 1);
    }

    #[test]
    fn test_ts_to_pdu_reset() {
        let mut ts2pdu = TaggedStreamToPdu::new("packet_len");
        let tags = vec![StreamTag::new(0, "packet_len", 10)];
        ts2pdu.process(&[1, 2, 3], &tags); // Partial
        ts2pdu.reset();
        // After reset, partial buffer should be gone
        let pdus = ts2pdu.process(&[4, 5, 6, 7, 8, 9, 10], &[]);
        assert!(pdus.is_empty()); // No tag, no output
    }

    #[test]
    fn test_pdu_empty() {
        let pdu = Pdu::new(Vec::new());
        assert!(pdu.is_empty());
        assert_eq!(pdu.len(), 0);
    }

    #[test]
    fn test_stream_tag_creation() {
        let tag = StreamTag::new(42, "my_key", 100);
        assert_eq!(tag.offset, 42);
        assert_eq!(tag.key, "my_key");
        assert_eq!(tag.value, 100);
    }
}

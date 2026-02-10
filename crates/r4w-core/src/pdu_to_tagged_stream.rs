//! # PDU to Tagged Stream Converter
//!
//! Converts Protocol Data Units (PDUs) into tagged streams. This is the reverse
//! operation of `tagged_stream_to_pdu`. Each PDU is emitted as a sequence of
//! samples with a length tag at the start, enabling downstream tagged-stream
//! blocks to process variable-length packets.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::pdu_to_tagged_stream::{PduToTaggedStream, Pdu};
//!
//! let mut converter = PduToTaggedStream::new("packet_len");
//!
//! // Enqueue a PDU
//! let pdu = Pdu::from_bytes(b"Hello");
//! converter.push_pdu(pdu);
//!
//! // Drain as tagged stream
//! let (samples, tags) = converter.drain();
//! assert_eq!(samples.len(), 5);
//! assert_eq!(tags.len(), 1);
//! assert_eq!(tags[0].offset, 0);
//! assert_eq!(tags[0].value, 5);
//! ```

use std::collections::VecDeque;

/// A Protocol Data Unit — a variable-length message with optional metadata.
#[derive(Debug, Clone)]
pub struct Pdu {
    /// Raw payload data.
    pub data: Vec<u8>,
    /// Optional metadata key-value pairs.
    pub metadata: Vec<(String, String)>,
}

impl Pdu {
    /// Create a PDU from raw bytes.
    pub fn from_bytes(data: &[u8]) -> Self {
        Self {
            data: data.to_vec(),
            metadata: Vec::new(),
        }
    }

    /// Create a PDU with metadata.
    pub fn with_metadata(data: &[u8], metadata: Vec<(String, String)>) -> Self {
        Self {
            data: data.to_vec(),
            metadata,
        }
    }

    /// Get the PDU length in bytes.
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Check if the PDU is empty.
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }
}

/// A stream tag marking a position in the output stream.
#[derive(Debug, Clone, PartialEq)]
pub struct StreamTag {
    /// Byte offset in the output stream where this tag applies.
    pub offset: usize,
    /// The tag value (length of the PDU in samples/bytes).
    pub value: usize,
    /// The tag key name.
    pub key: String,
}

/// Converts PDUs into a tagged byte stream.
///
/// PDUs are queued and then drained into a contiguous byte stream with
/// stream tags marking the start and length of each PDU.
#[derive(Debug, Clone)]
pub struct PduToTaggedStream {
    /// Tag key name (e.g., "packet_len").
    tag_key: String,
    /// Queue of pending PDUs.
    pdu_queue: VecDeque<Pdu>,
    /// Total PDUs processed.
    pdus_processed: u64,
    /// Total bytes output.
    bytes_output: u64,
    /// Maximum queue depth (0 = unlimited).
    max_queue_depth: usize,
}

impl PduToTaggedStream {
    /// Create a new converter with the given tag key name.
    pub fn new(tag_key: &str) -> Self {
        Self {
            tag_key: tag_key.to_string(),
            pdu_queue: VecDeque::new(),
            pdus_processed: 0,
            bytes_output: 0,
            max_queue_depth: 0,
        }
    }

    /// Create a converter with a maximum queue depth.
    pub fn with_max_queue(tag_key: &str, max_depth: usize) -> Self {
        Self {
            tag_key: tag_key.to_string(),
            pdu_queue: VecDeque::new(),
            pdus_processed: 0,
            bytes_output: 0,
            max_queue_depth: max_depth,
        }
    }

    /// Push a PDU into the queue. Returns false if queue is full.
    pub fn push_pdu(&mut self, pdu: Pdu) -> bool {
        if self.max_queue_depth > 0 && self.pdu_queue.len() >= self.max_queue_depth {
            return false;
        }
        self.pdu_queue.push_back(pdu);
        true
    }

    /// Push multiple PDUs into the queue.
    pub fn push_pdus(&mut self, pdus: impl IntoIterator<Item = Pdu>) {
        for pdu in pdus {
            self.push_pdu(pdu);
        }
    }

    /// Drain all queued PDUs into a contiguous byte stream with tags.
    pub fn drain(&mut self) -> (Vec<u8>, Vec<StreamTag>) {
        let mut output = Vec::new();
        let mut tags = Vec::new();

        while let Some(pdu) = self.pdu_queue.pop_front() {
            let offset = output.len();
            let len = pdu.data.len();

            tags.push(StreamTag {
                offset,
                value: len,
                key: self.tag_key.clone(),
            });

            output.extend_from_slice(&pdu.data);
            self.pdus_processed += 1;
            self.bytes_output += len as u64;
        }

        (output, tags)
    }

    /// Drain a single PDU (if available) as a tagged stream segment.
    pub fn drain_one(&mut self) -> Option<(Vec<u8>, StreamTag)> {
        let pdu = self.pdu_queue.pop_front()?;
        let len = pdu.data.len();
        let tag = StreamTag {
            offset: 0,
            value: len,
            key: self.tag_key.clone(),
        };
        self.pdus_processed += 1;
        self.bytes_output += len as u64;
        Some((pdu.data, tag))
    }

    /// Drain up to `max_bytes` of data from queued PDUs.
    pub fn drain_limited(&mut self, max_bytes: usize) -> (Vec<u8>, Vec<StreamTag>) {
        let mut output = Vec::new();
        let mut tags = Vec::new();

        while let Some(pdu) = self.pdu_queue.front() {
            if output.len() + pdu.data.len() > max_bytes && !output.is_empty() {
                break;
            }

            let pdu = self.pdu_queue.pop_front().unwrap();
            let offset = output.len();
            let len = pdu.data.len();

            tags.push(StreamTag {
                offset,
                value: len,
                key: self.tag_key.clone(),
            });

            output.extend_from_slice(&pdu.data);
            self.pdus_processed += 1;
            self.bytes_output += len as u64;
        }

        (output, tags)
    }

    /// Get the number of PDUs waiting in the queue.
    pub fn queue_len(&self) -> usize {
        self.pdu_queue.len()
    }

    /// Check if the queue is empty.
    pub fn is_empty(&self) -> bool {
        self.pdu_queue.is_empty()
    }

    /// Get total PDUs processed.
    pub fn pdus_processed(&self) -> u64 {
        self.pdus_processed
    }

    /// Get total bytes output.
    pub fn bytes_output(&self) -> u64 {
        self.bytes_output
    }

    /// Get the tag key name.
    pub fn tag_key(&self) -> &str {
        &self.tag_key
    }

    /// Reset the converter state.
    pub fn reset(&mut self) {
        self.pdu_queue.clear();
        self.pdus_processed = 0;
        self.bytes_output = 0;
    }
}

/// Convert a tagged stream back to PDUs (reverse operation).
pub fn tagged_stream_to_pdus(data: &[u8], tags: &[StreamTag]) -> Vec<Pdu> {
    let mut pdus = Vec::new();

    for tag in tags {
        let start = tag.offset;
        let end = (start + tag.value).min(data.len());
        if start < data.len() {
            pdus.push(Pdu::from_bytes(&data[start..end]));
        }
    }

    pdus
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_single_pdu() {
        let mut conv = PduToTaggedStream::new("packet_len");
        conv.push_pdu(Pdu::from_bytes(b"Hello"));
        let (data, tags) = conv.drain();
        assert_eq!(data, b"Hello");
        assert_eq!(tags.len(), 1);
        assert_eq!(tags[0].offset, 0);
        assert_eq!(tags[0].value, 5);
        assert_eq!(tags[0].key, "packet_len");
    }

    #[test]
    fn test_multiple_pdus() {
        let mut conv = PduToTaggedStream::new("len");
        conv.push_pdu(Pdu::from_bytes(b"AB"));
        conv.push_pdu(Pdu::from_bytes(b"CDE"));
        conv.push_pdu(Pdu::from_bytes(b"F"));

        let (data, tags) = conv.drain();
        assert_eq!(data, b"ABCDEF");
        assert_eq!(tags.len(), 3);
        assert_eq!(tags[0].offset, 0);
        assert_eq!(tags[0].value, 2);
        assert_eq!(tags[1].offset, 2);
        assert_eq!(tags[1].value, 3);
        assert_eq!(tags[2].offset, 5);
        assert_eq!(tags[2].value, 1);
    }

    #[test]
    fn test_empty_queue() {
        let mut conv = PduToTaggedStream::new("len");
        let (data, tags) = conv.drain();
        assert!(data.is_empty());
        assert!(tags.is_empty());
    }

    #[test]
    fn test_drain_one() {
        let mut conv = PduToTaggedStream::new("len");
        conv.push_pdu(Pdu::from_bytes(b"first"));
        conv.push_pdu(Pdu::from_bytes(b"second"));

        let (data, tag) = conv.drain_one().unwrap();
        assert_eq!(data, b"first");
        assert_eq!(tag.value, 5);
        assert_eq!(conv.queue_len(), 1);
    }

    #[test]
    fn test_drain_limited() {
        let mut conv = PduToTaggedStream::new("len");
        conv.push_pdu(Pdu::from_bytes(b"AAAA")); // 4 bytes
        conv.push_pdu(Pdu::from_bytes(b"BBBB")); // 4 bytes
        conv.push_pdu(Pdu::from_bytes(b"CC"));   // 2 bytes

        let (data, tags) = conv.drain_limited(6);
        // First PDU (4) fits, second (4) would exceed 6 with first
        assert_eq!(data, b"AAAA");
        assert_eq!(tags.len(), 1);
        assert_eq!(conv.queue_len(), 2);
    }

    #[test]
    fn test_max_queue_depth() {
        let mut conv = PduToTaggedStream::with_max_queue("len", 2);
        assert!(conv.push_pdu(Pdu::from_bytes(b"A")));
        assert!(conv.push_pdu(Pdu::from_bytes(b"B")));
        assert!(!conv.push_pdu(Pdu::from_bytes(b"C"))); // rejected
        assert_eq!(conv.queue_len(), 2);
    }

    #[test]
    fn test_roundtrip() {
        let mut conv = PduToTaggedStream::new("pkt_len");
        let original = vec![
            Pdu::from_bytes(b"Hello"),
            Pdu::from_bytes(b"World!"),
        ];
        for pdu in &original {
            conv.push_pdu(pdu.clone());
        }
        let (data, tags) = conv.drain();
        let recovered = tagged_stream_to_pdus(&data, &tags);
        assert_eq!(recovered.len(), 2);
        assert_eq!(recovered[0].data, b"Hello");
        assert_eq!(recovered[1].data, b"World!");
    }

    #[test]
    fn test_statistics() {
        let mut conv = PduToTaggedStream::new("len");
        conv.push_pdu(Pdu::from_bytes(b"ABC"));
        conv.push_pdu(Pdu::from_bytes(b"DE"));
        conv.drain();
        assert_eq!(conv.pdus_processed(), 2);
        assert_eq!(conv.bytes_output(), 5);
    }

    #[test]
    fn test_reset() {
        let mut conv = PduToTaggedStream::new("len");
        conv.push_pdu(Pdu::from_bytes(b"data"));
        conv.drain();
        assert_eq!(conv.pdus_processed(), 1);
        conv.reset();
        assert_eq!(conv.pdus_processed(), 0);
        assert_eq!(conv.bytes_output(), 0);
        assert!(conv.is_empty());
    }

    #[test]
    fn test_pdu_with_metadata() {
        let pdu = Pdu::with_metadata(
            b"payload",
            vec![("src".to_string(), "192.168.1.1".to_string())],
        );
        assert_eq!(pdu.len(), 7);
        assert!(!pdu.is_empty());
        assert_eq!(pdu.metadata.len(), 1);
    }
}

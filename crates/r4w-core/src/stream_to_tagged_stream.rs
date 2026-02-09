//! Stream to Tagged Stream — Insert periodic length tags
//!
//! Converts a continuous sample stream into a tagged stream by inserting
//! packet-length tags at regular intervals. Each tag marks the start of
//! a packet of N samples. Used to bridge continuous sources to
//! packet-oriented processing chains.
//! GNU Radio equivalent: `stream_to_tagged_stream`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::stream_to_tagged_stream::StreamToTaggedStream;
//!
//! let mut tagger = StreamToTaggedStream::new(4, "packet_len");
//! let data = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
//! let tags = tagger.process(data.len());
//! assert_eq!(tags.len(), 2); // Two packets of 4
//! assert_eq!(tags[0].offset, 0);
//! assert_eq!(tags[1].offset, 4);
//! ```

/// A tag indicating a packet boundary in a stream.
#[derive(Debug, Clone, PartialEq)]
pub struct StreamTag {
    /// Sample offset where the tag applies.
    pub offset: u64,
    /// Tag key.
    pub key: String,
    /// Tag value (packet length as string).
    pub value: String,
}

/// Converts a continuous stream into a tagged stream with periodic length tags.
#[derive(Debug, Clone)]
pub struct StreamToTaggedStream {
    /// Packet length (samples per packet).
    packet_len: usize,
    /// Tag key for the length tag.
    length_tag_key: String,
    /// Current absolute sample offset.
    offset: u64,
}

impl StreamToTaggedStream {
    /// Create a new tagger with the given packet length and tag key.
    pub fn new(packet_len: usize, length_tag_key: &str) -> Self {
        Self {
            packet_len: packet_len.max(1),
            length_tag_key: length_tag_key.to_string(),
            offset: 0,
        }
    }

    /// Process `num_samples` items and return tags for packet boundaries.
    ///
    /// Tags are placed at every `packet_len` boundary within the input range.
    pub fn process(&mut self, num_samples: usize) -> Vec<StreamTag> {
        let mut tags = Vec::new();
        let start = self.offset;
        let end = start + num_samples as u64;

        // Find the first packet boundary at or after `start`
        let first_boundary = if start % self.packet_len as u64 == 0 {
            start
        } else {
            start + (self.packet_len as u64 - start % self.packet_len as u64)
        };

        let mut boundary = first_boundary;
        while boundary < end {
            tags.push(StreamTag {
                offset: boundary,
                key: self.length_tag_key.clone(),
                value: self.packet_len.to_string(),
            });
            boundary += self.packet_len as u64;
        }

        self.offset = end;
        tags
    }

    /// Reset the internal offset counter.
    pub fn reset(&mut self) {
        self.offset = 0;
    }

    /// Get the current absolute offset.
    pub fn offset(&self) -> u64 {
        self.offset
    }

    /// Get the packet length.
    pub fn packet_len(&self) -> usize {
        self.packet_len
    }

    /// Set a new packet length.
    pub fn set_packet_len(&mut self, len: usize) {
        self.packet_len = len.max(1);
    }

    /// Get the length tag key.
    pub fn length_tag_key(&self) -> &str {
        &self.length_tag_key
    }
}

/// Chop a data slice into packets of `packet_len`, returning (packets, remainder).
///
/// The remainder contains samples that don't fill a complete packet.
pub fn chop_into_packets<T: Clone>(data: &[T], packet_len: usize) -> (Vec<Vec<T>>, Vec<T>) {
    let packet_len = packet_len.max(1);
    let full_packets = data.len() / packet_len;
    let mut packets = Vec::with_capacity(full_packets);

    for i in 0..full_packets {
        let start = i * packet_len;
        packets.push(data[start..start + packet_len].to_vec());
    }

    let remainder = data[full_packets * packet_len..].to_vec();
    (packets, remainder)
}

/// Count how many complete packets fit in `num_samples`.
pub fn count_packets(num_samples: usize, packet_len: usize) -> usize {
    if packet_len == 0 {
        return 0;
    }
    num_samples / packet_len
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_tagging() {
        let mut tagger = StreamToTaggedStream::new(4, "packet_len");
        let tags = tagger.process(8);
        assert_eq!(tags.len(), 2);
        assert_eq!(tags[0].offset, 0);
        assert_eq!(tags[0].value, "4");
        assert_eq!(tags[1].offset, 4);
    }

    #[test]
    fn test_partial_packet() {
        let mut tagger = StreamToTaggedStream::new(4, "pkt");
        let tags = tagger.process(6);
        // Only 1 full packet boundary at offset 0; offset 4 is within range too
        assert_eq!(tags.len(), 2); // 0 and 4 are both < 6
    }

    #[test]
    fn test_continuation() {
        let mut tagger = StreamToTaggedStream::new(4, "len");
        let tags1 = tagger.process(3); // 0..3
        assert_eq!(tags1.len(), 1); // Boundary at 0
        let tags2 = tagger.process(3); // 3..6
        assert_eq!(tags2.len(), 1); // Boundary at 4
        assert_eq!(tags2[0].offset, 4);
    }

    #[test]
    fn test_exact_packet() {
        let mut tagger = StreamToTaggedStream::new(5, "len");
        let tags = tagger.process(5);
        assert_eq!(tags.len(), 1);
        assert_eq!(tags[0].offset, 0);
    }

    #[test]
    fn test_single_sample_packets() {
        let mut tagger = StreamToTaggedStream::new(1, "len");
        let tags = tagger.process(3);
        assert_eq!(tags.len(), 3);
        assert_eq!(tags[0].offset, 0);
        assert_eq!(tags[1].offset, 1);
        assert_eq!(tags[2].offset, 2);
    }

    #[test]
    fn test_zero_samples() {
        let mut tagger = StreamToTaggedStream::new(4, "len");
        let tags = tagger.process(0);
        assert!(tags.is_empty());
    }

    #[test]
    fn test_reset() {
        let mut tagger = StreamToTaggedStream::new(4, "len");
        tagger.process(10);
        assert_eq!(tagger.offset(), 10);
        tagger.reset();
        assert_eq!(tagger.offset(), 0);
    }

    #[test]
    fn test_set_packet_len() {
        let mut tagger = StreamToTaggedStream::new(4, "len");
        assert_eq!(tagger.packet_len(), 4);
        tagger.set_packet_len(8);
        assert_eq!(tagger.packet_len(), 8);
    }

    #[test]
    fn test_chop_into_packets() {
        let data = vec![1, 2, 3, 4, 5, 6, 7];
        let (packets, remainder) = chop_into_packets(&data, 3);
        assert_eq!(packets.len(), 2);
        assert_eq!(packets[0], vec![1, 2, 3]);
        assert_eq!(packets[1], vec![4, 5, 6]);
        assert_eq!(remainder, vec![7]);
    }

    #[test]
    fn test_chop_exact() {
        let data = vec![1.0, 2.0, 3.0, 4.0];
        let (packets, remainder) = chop_into_packets(&data, 2);
        assert_eq!(packets.len(), 2);
        assert!(remainder.is_empty());
    }

    #[test]
    fn test_count_packets() {
        assert_eq!(count_packets(10, 3), 3);
        assert_eq!(count_packets(9, 3), 3);
        assert_eq!(count_packets(0, 5), 0);
        assert_eq!(count_packets(5, 0), 0);
    }

    #[test]
    fn test_large_packet() {
        let mut tagger = StreamToTaggedStream::new(100, "len");
        let tags = tagger.process(50);
        assert_eq!(tags.len(), 1); // Boundary at 0
    }

    #[test]
    fn test_tag_key() {
        let tagger = StreamToTaggedStream::new(4, "my_key");
        assert_eq!(tagger.length_tag_key(), "my_key");
    }
}

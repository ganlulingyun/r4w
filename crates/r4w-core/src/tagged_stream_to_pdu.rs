//! # Tagged Stream to PDU
//!
//! Converts tagged stream blocks into Protocol Data Units (PDUs).
//! The inverse of `pdu_to_tagged_stream`. Segments a continuous stream
//! using length tags to produce discrete PDUs for message-based processing.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::tagged_stream_to_pdu::{TaggedStreamToPdu, StreamTag};
//!
//! let mut converter = TaggedStreamToPdu::new("packet_len");
//!
//! // Feed tagged data: tag says next 4 samples are a packet
//! let tags = vec![StreamTag { offset: 0, key: "packet_len".into(), value: 4 }];
//! let data = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
//! let pdus = converter.process(&data, &tags);
//! assert_eq!(pdus.len(), 1);
//! assert_eq!(pdus[0].data, vec![1.0, 2.0, 3.0, 4.0]);
//! ```

/// A stream tag with offset, key, and integer value.
#[derive(Debug, Clone)]
pub struct StreamTag {
    /// Sample offset in the current block.
    pub offset: usize,
    /// Tag key.
    pub key: String,
    /// Tag value (length in samples).
    pub value: usize,
}

/// A Protocol Data Unit extracted from the stream.
#[derive(Debug, Clone)]
pub struct Pdu {
    /// Sample data.
    pub data: Vec<f64>,
    /// Associated metadata tags.
    pub metadata: Vec<(String, String)>,
}

/// Complex PDU.
#[derive(Debug, Clone)]
pub struct PduComplex {
    /// IQ sample data.
    pub data: Vec<(f64, f64)>,
    /// Associated metadata.
    pub metadata: Vec<(String, String)>,
}

/// Converts tagged streams to PDUs.
#[derive(Debug, Clone)]
pub struct TaggedStreamToPdu {
    /// Length tag key to look for.
    length_tag_key: String,
    /// Internal buffer for partial PDUs.
    buffer: Vec<f64>,
    /// Current expected PDU length (0 = waiting for tag).
    current_len: usize,
    /// Total PDUs produced.
    pdus_produced: u64,
}

impl TaggedStreamToPdu {
    /// Create a new converter with the given length tag key.
    pub fn new(length_tag_key: &str) -> Self {
        Self {
            length_tag_key: length_tag_key.to_string(),
            buffer: Vec::new(),
            current_len: 0,
            pdus_produced: 0,
        }
    }

    /// Process a block of samples with associated tags.
    pub fn process(&mut self, data: &[f64], tags: &[StreamTag]) -> Vec<Pdu> {
        let mut pdus = Vec::new();
        let mut pos = 0;

        // Sort tags by offset.
        let mut sorted_tags: Vec<&StreamTag> = tags
            .iter()
            .filter(|t| t.key == self.length_tag_key)
            .collect();
        sorted_tags.sort_by_key(|t| t.offset);

        for tag in sorted_tags {
            // If we have a partial PDU being assembled, flush it
            // (it's incomplete, but the new tag starts a new PDU).
            if self.current_len > 0 && !self.buffer.is_empty() {
                // Skip incomplete PDU.
                self.buffer.clear();
                self.current_len = 0;
            }

            pos = tag.offset;
            self.current_len = tag.value;

            let available = data.len() - pos;
            let take = available.min(self.current_len);
            self.buffer.extend_from_slice(&data[pos..pos + take]);
            pos += take;

            if self.buffer.len() >= self.current_len {
                let pdu_data = self.buffer[..self.current_len].to_vec();
                pdus.push(Pdu {
                    data: pdu_data,
                    metadata: Vec::new(),
                });
                self.pdus_produced += 1;
                self.buffer.clear();
                self.current_len = 0;
            }
        }

        // If we're in the middle of assembling a PDU (no new tag found),
        // continue filling the buffer.
        if self.current_len > 0 && self.buffer.len() < self.current_len {
            let remaining = self.current_len - self.buffer.len();
            let available = data.len() - pos;
            let take = available.min(remaining);
            self.buffer.extend_from_slice(&data[pos..pos + take]);

            if self.buffer.len() >= self.current_len {
                let pdu_data = self.buffer[..self.current_len].to_vec();
                pdus.push(Pdu {
                    data: pdu_data,
                    metadata: Vec::new(),
                });
                self.pdus_produced += 1;
                self.buffer.clear();
                self.current_len = 0;
            }
        }

        pdus
    }

    /// Process complex IQ samples with tags.
    pub fn process_complex(
        &mut self,
        data: &[(f64, f64)],
        tags: &[StreamTag],
    ) -> Vec<PduComplex> {
        // Convert to real processing (flatten I/Q).
        let real: Vec<f64> = data.iter().flat_map(|&(i, q)| vec![i, q]).collect();
        let adjusted_tags: Vec<StreamTag> = tags
            .iter()
            .filter(|t| t.key == self.length_tag_key)
            .map(|t| StreamTag {
                offset: t.offset * 2,
                key: t.key.clone(),
                value: t.value * 2,
            })
            .collect();

        let pdus = self.process(&real, &adjusted_tags);
        pdus.into_iter()
            .map(|pdu| {
                let iq: Vec<(f64, f64)> = pdu.data.chunks_exact(2).map(|c| (c[0], c[1])).collect();
                PduComplex {
                    data: iq,
                    metadata: pdu.metadata,
                }
            })
            .collect()
    }

    /// Get total PDUs produced.
    pub fn pdus_produced(&self) -> u64 {
        self.pdus_produced
    }

    /// Get the length tag key.
    pub fn length_tag_key(&self) -> &str {
        &self.length_tag_key
    }

    /// Check if there's a partial PDU in the buffer.
    pub fn has_partial(&self) -> bool {
        !self.buffer.is_empty()
    }

    /// Get the current buffer size.
    pub fn buffer_len(&self) -> usize {
        self.buffer.len()
    }

    /// Reset the converter.
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.current_len = 0;
        self.pdus_produced = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_single_pdu() {
        let mut conv = TaggedStreamToPdu::new("len");
        let tags = vec![StreamTag { offset: 0, key: "len".into(), value: 3 }];
        let data = vec![1.0, 2.0, 3.0];
        let pdus = conv.process(&data, &tags);
        assert_eq!(pdus.len(), 1);
        assert_eq!(pdus[0].data, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_multiple_pdus() {
        let mut conv = TaggedStreamToPdu::new("len");
        let tags = vec![
            StreamTag { offset: 0, key: "len".into(), value: 2 },
            StreamTag { offset: 2, key: "len".into(), value: 3 },
        ];
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let pdus = conv.process(&data, &tags);
        assert_eq!(pdus.len(), 2);
        assert_eq!(pdus[0].data, vec![1.0, 2.0]);
        assert_eq!(pdus[1].data, vec![3.0, 4.0, 5.0]);
    }

    #[test]
    fn test_no_tags() {
        let mut conv = TaggedStreamToPdu::new("len");
        let pdus = conv.process(&[1.0, 2.0], &[]);
        assert!(pdus.is_empty());
    }

    #[test]
    fn test_wrong_key() {
        let mut conv = TaggedStreamToPdu::new("len");
        let tags = vec![StreamTag { offset: 0, key: "other".into(), value: 2 }];
        let pdus = conv.process(&[1.0, 2.0], &tags);
        assert!(pdus.is_empty());
    }

    #[test]
    fn test_partial_pdu() {
        let mut conv = TaggedStreamToPdu::new("len");
        let tags = vec![StreamTag { offset: 0, key: "len".into(), value: 5 }];
        // Only 3 samples available, need 5.
        let pdus = conv.process(&[1.0, 2.0, 3.0], &tags);
        assert!(pdus.is_empty());
        assert!(conv.has_partial());
        assert_eq!(conv.buffer_len(), 3);

        // Feed remaining samples (no new tag needed).
        let pdus2 = conv.process(&[4.0, 5.0], &[]);
        assert_eq!(pdus2.len(), 1);
        assert_eq!(pdus2[0].data, vec![1.0, 2.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn test_zero_length() {
        let mut conv = TaggedStreamToPdu::new("len");
        let tags = vec![StreamTag { offset: 0, key: "len".into(), value: 0 }];
        let pdus = conv.process(&[1.0, 2.0], &tags);
        assert_eq!(pdus.len(), 1);
        assert!(pdus[0].data.is_empty());
    }

    #[test]
    fn test_pdus_produced() {
        let mut conv = TaggedStreamToPdu::new("len");
        let tags = vec![
            StreamTag { offset: 0, key: "len".into(), value: 1 },
            StreamTag { offset: 1, key: "len".into(), value: 1 },
        ];
        conv.process(&[1.0, 2.0], &tags);
        assert_eq!(conv.pdus_produced(), 2);
    }

    #[test]
    fn test_reset() {
        let mut conv = TaggedStreamToPdu::new("len");
        let tags = vec![StreamTag { offset: 0, key: "len".into(), value: 10 }];
        conv.process(&[1.0, 2.0], &tags);
        assert!(conv.has_partial());
        conv.reset();
        assert!(!conv.has_partial());
        assert_eq!(conv.pdus_produced(), 0);
    }

    #[test]
    fn test_complex_pdu() {
        let mut conv = TaggedStreamToPdu::new("len");
        let tags = vec![StreamTag { offset: 0, key: "len".into(), value: 2 }];
        let data = vec![(1.0, 2.0), (3.0, 4.0)];
        let pdus = conv.process_complex(&data, &tags);
        assert_eq!(pdus.len(), 1);
        assert_eq!(pdus[0].data, vec![(1.0, 2.0), (3.0, 4.0)]);
    }

    #[test]
    fn test_length_tag_key() {
        let conv = TaggedStreamToPdu::new("packet_len");
        assert_eq!(conv.length_tag_key(), "packet_len");
    }
}

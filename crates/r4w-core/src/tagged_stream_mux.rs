//! # Tagged Stream Multiplexer
//!
//! Combines multiple tagged streams into a single output stream while preserving
//! metadata tags with correctly adjusted offsets. Provides several multiplexing
//! strategies: sequential concatenation, round-robin interleaving, and
//! priority-based ordering.
//!
//! ## Multiplexer Types
//!
//! - **TaggedStreamMux**: Concatenates real-valued streams in input order
//! - **TaggedStreamMuxComplex**: Concatenates complex-valued streams in input order
//! - **RoundRobinMux**: Interleaves fixed-size chunks from each input
//! - **PriorityMux**: Orders inputs by assigned priority (highest first)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::tagged_stream_mux::{
//!     TaggedStreamMux, TaggedStream, StreamTag, TagValue,
//! };
//!
//! let mux = TaggedStreamMux::new(2);
//!
//! let stream_a = TaggedStream {
//!     data: vec![1.0, 2.0, 3.0],
//!     tags: vec![StreamTag {
//!         offset: 0,
//!         key: "burst_start".into(),
//!         value: TagValue::Bool(true),
//!     }],
//! };
//! let stream_b = TaggedStream {
//!     data: vec![4.0, 5.0],
//!     tags: vec![StreamTag {
//!         offset: 0,
//!         key: "packet_len".into(),
//!         value: TagValue::Int(2),
//!     }],
//! };
//!
//! let output = mux.mux(&[stream_a, stream_b]);
//! assert_eq!(output.data, vec![1.0, 2.0, 3.0, 4.0, 5.0]);
//! // stream_b's tag offset is adjusted from 0 to 3 (length of stream_a)
//! assert_eq!(output.tags[1].offset, 3);
//! ```

/// A metadata tag attached to a specific offset within a stream.
#[derive(Debug, Clone, PartialEq)]
pub struct StreamTag {
    /// Sample offset within the stream where this tag applies.
    pub offset: usize,
    /// Tag key identifying the metadata type.
    pub key: String,
    /// Tag value carrying the metadata payload.
    pub value: TagValue,
}

/// Typed value for a stream tag.
#[derive(Debug, Clone, PartialEq)]
pub enum TagValue {
    /// Tag with no associated value (marker).
    None,
    /// Boolean value.
    Bool(bool),
    /// Signed 64-bit integer value.
    Int(i64),
    /// 64-bit floating-point value.
    Float(f64),
    /// String value.
    String(String),
    /// Raw byte buffer.
    Bytes(Vec<u8>),
}

/// A real-valued stream with associated metadata tags.
#[derive(Debug, Clone)]
pub struct TaggedStream {
    /// Sample data.
    pub data: Vec<f64>,
    /// Metadata tags, each referencing a sample offset in `data`.
    pub tags: Vec<StreamTag>,
}

/// A complex-valued stream with associated metadata tags.
#[derive(Debug, Clone)]
pub struct TaggedStreamComplex {
    /// Complex sample data as (real, imaginary) pairs.
    pub data: Vec<(f64, f64)>,
    /// Metadata tags, each referencing a sample offset in `data`.
    pub tags: Vec<StreamTag>,
}

/// Sequential multiplexer for real-valued tagged streams.
///
/// Concatenates inputs in order (input 0, then input 1, ...), adjusting each
/// stream's tag offsets by the cumulative sample count of all preceding streams.
#[derive(Debug, Clone)]
pub struct TaggedStreamMux {
    /// Expected number of input streams.
    num_inputs: usize,
}

impl TaggedStreamMux {
    /// Create a new multiplexer expecting `num_inputs` input streams.
    pub fn new(num_inputs: usize) -> Self {
        Self { num_inputs }
    }

    /// Concatenate all input streams into a single output stream.
    ///
    /// Tag offsets are shifted so they remain correct relative to the output.
    /// If the number of inputs does not match `num_inputs`, only the provided
    /// inputs are processed.
    pub fn mux(&self, inputs: &[TaggedStream]) -> TaggedStream {
        let total_len: usize = inputs.iter().map(|s| s.data.len()).sum();
        let mut data = Vec::with_capacity(total_len);
        let mut tags = Vec::new();
        let mut offset = 0usize;

        for stream in inputs {
            data.extend_from_slice(&stream.data);
            for tag in &stream.tags {
                tags.push(StreamTag {
                    offset: tag.offset + offset,
                    key: tag.key.clone(),
                    value: tag.value.clone(),
                });
            }
            offset += stream.data.len();
        }

        TaggedStream { data, tags }
    }
}

/// Sequential multiplexer for complex-valued tagged streams.
///
/// Operates identically to [`TaggedStreamMux`] but on complex `(f64, f64)` data.
#[derive(Debug, Clone)]
pub struct TaggedStreamMuxComplex {
    /// Expected number of input streams.
    num_inputs: usize,
}

impl TaggedStreamMuxComplex {
    /// Create a new complex multiplexer expecting `num_inputs` input streams.
    pub fn new(num_inputs: usize) -> Self {
        Self { num_inputs }
    }

    /// Concatenate all complex input streams into a single output stream.
    pub fn mux(&self, inputs: &[TaggedStreamComplex]) -> TaggedStreamComplex {
        let total_len: usize = inputs.iter().map(|s| s.data.len()).sum();
        let mut data = Vec::with_capacity(total_len);
        let mut tags = Vec::new();
        let mut offset = 0usize;

        for stream in inputs {
            data.extend_from_slice(&stream.data);
            for tag in &stream.tags {
                tags.push(StreamTag {
                    offset: tag.offset + offset,
                    key: tag.key.clone(),
                    value: tag.value.clone(),
                });
            }
            offset += stream.data.len();
        }

        TaggedStreamComplex { data, tags }
    }
}

/// Round-robin multiplexer for real-valued tagged streams.
///
/// Interleaves fixed-size chunks from each input in a cyclic pattern:
/// chunk from input 0, chunk from input 1, ..., chunk from input 0, ...
///
/// When an input is exhausted, its remaining turns are skipped. Tag offsets
/// are adjusted to reflect each chunk's position in the output.
#[derive(Debug, Clone)]
pub struct RoundRobinMux {
    /// Expected number of input streams.
    num_inputs: usize,
    /// Number of samples taken from each input per turn.
    chunk_size: usize,
}

impl RoundRobinMux {
    /// Create a new round-robin multiplexer.
    ///
    /// `chunk_size` is the number of samples taken from each input per turn.
    pub fn new(num_inputs: usize, chunk_size: usize) -> Self {
        Self {
            num_inputs,
            chunk_size,
        }
    }

    /// Interleave chunks from the input streams into a single output stream.
    pub fn mux(&self, inputs: &[TaggedStream]) -> TaggedStream {
        let total_len: usize = inputs.iter().map(|s| s.data.len()).sum();
        let mut data = Vec::with_capacity(total_len);
        let mut tags = Vec::new();

        // Track how many samples have been consumed from each input.
        let mut cursors: Vec<usize> = vec![0; inputs.len()];
        let mut any_remaining = true;

        while any_remaining {
            any_remaining = false;
            for (i, stream) in inputs.iter().enumerate() {
                let start = cursors[i];
                if start >= stream.data.len() {
                    continue;
                }
                any_remaining = true;
                let end = (start + self.chunk_size).min(stream.data.len());
                let output_offset = data.len();

                data.extend_from_slice(&stream.data[start..end]);

                // Copy tags whose offsets fall within this chunk.
                for tag in &stream.tags {
                    if tag.offset >= start && tag.offset < end {
                        tags.push(StreamTag {
                            offset: output_offset + (tag.offset - start),
                            key: tag.key.clone(),
                            value: tag.value.clone(),
                        });
                    }
                }

                cursors[i] = end;
            }
        }

        TaggedStream { data, tags }
    }
}

/// Priority-based multiplexer for real-valued tagged streams.
///
/// Concatenates inputs ordered by priority (highest priority value first).
/// Inputs with equal priority retain their original order.
#[derive(Debug, Clone)]
pub struct PriorityMux {
    /// Expected number of input streams.
    num_inputs: usize,
    /// Priority for each input (higher = first in output).
    priorities: Vec<u32>,
}

impl PriorityMux {
    /// Create a new priority multiplexer with all priorities initially zero.
    pub fn new(num_inputs: usize) -> Self {
        Self {
            num_inputs,
            priorities: vec![0; num_inputs],
        }
    }

    /// Set the priority for a specific input index.
    ///
    /// Higher values are placed earlier in the output.
    pub fn set_priority(&mut self, input_idx: usize, priority: u32) {
        if input_idx < self.priorities.len() {
            self.priorities[input_idx] = priority;
        }
    }

    /// Concatenate inputs in priority order (highest first) into a single output.
    pub fn mux(&self, inputs: &[TaggedStream]) -> TaggedStream {
        // Build index list sorted by descending priority, stable on original order.
        let mut indices: Vec<usize> = (0..inputs.len()).collect();
        indices.sort_by(|&a, &b| {
            let pa = self.priorities.get(a).copied().unwrap_or(0);
            let pb = self.priorities.get(b).copied().unwrap_or(0);
            pb.cmp(&pa).then(a.cmp(&b))
        });

        let total_len: usize = inputs.iter().map(|s| s.data.len()).sum();
        let mut data = Vec::with_capacity(total_len);
        let mut tags = Vec::new();
        let mut offset = 0usize;

        for &idx in &indices {
            let stream = &inputs[idx];
            data.extend_from_slice(&stream.data);
            for tag in &stream.tags {
                tags.push(StreamTag {
                    offset: tag.offset + offset,
                    key: tag.key.clone(),
                    value: tag.value.clone(),
                });
            }
            offset += stream.data.len();
        }

        TaggedStream { data, tags }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_tag(offset: usize, key: &str, value: TagValue) -> StreamTag {
        StreamTag {
            offset,
            key: key.to_string(),
            value,
        }
    }

    fn make_stream(data: Vec<f64>, tags: Vec<StreamTag>) -> TaggedStream {
        TaggedStream { data, tags }
    }

    #[test]
    fn test_basic_mux() {
        let mux = TaggedStreamMux::new(3);
        let s0 = make_stream(vec![1.0, 2.0], vec![]);
        let s1 = make_stream(vec![3.0, 4.0, 5.0], vec![]);
        let s2 = make_stream(vec![6.0], vec![]);

        let out = mux.mux(&[s0, s1, s2]);
        assert_eq!(out.data, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        assert!(out.tags.is_empty());
    }

    #[test]
    fn test_tag_offset_adjustment() {
        let mux = TaggedStreamMux::new(2);
        let s0 = make_stream(
            vec![1.0, 2.0, 3.0],
            vec![make_tag(0, "start", TagValue::Bool(true))],
        );
        let s1 = make_stream(
            vec![4.0, 5.0],
            vec![
                make_tag(0, "pkt_len", TagValue::Int(2)),
                make_tag(1, "end", TagValue::None),
            ],
        );

        let out = mux.mux(&[s0, s1]);
        assert_eq!(out.data.len(), 5);
        assert_eq!(out.tags.len(), 3);
        // First stream's tag stays at offset 0.
        assert_eq!(out.tags[0].offset, 0);
        assert_eq!(out.tags[0].key, "start");
        // Second stream's tags are shifted by 3 (length of first stream).
        assert_eq!(out.tags[1].offset, 3);
        assert_eq!(out.tags[1].key, "pkt_len");
        assert_eq!(out.tags[2].offset, 4);
        assert_eq!(out.tags[2].key, "end");
    }

    #[test]
    fn test_empty_inputs() {
        let mux = TaggedStreamMux::new(0);
        let out = mux.mux(&[]);
        assert!(out.data.is_empty());
        assert!(out.tags.is_empty());
    }

    #[test]
    fn test_single_input() {
        let mux = TaggedStreamMux::new(1);
        let s = make_stream(
            vec![10.0, 20.0, 30.0],
            vec![make_tag(2, "peak", TagValue::Float(30.0))],
        );

        let out = mux.mux(&[s]);
        assert_eq!(out.data, vec![10.0, 20.0, 30.0]);
        assert_eq!(out.tags.len(), 1);
        assert_eq!(out.tags[0].offset, 2);
        assert_eq!(out.tags[0].value, TagValue::Float(30.0));
    }

    #[test]
    fn test_complex_mux() {
        let mux = TaggedStreamMuxComplex::new(2);
        let s0 = TaggedStreamComplex {
            data: vec![(1.0, 0.0), (0.0, 1.0)],
            tags: vec![make_tag(0, "iq_start", TagValue::None)],
        };
        let s1 = TaggedStreamComplex {
            data: vec![(0.5, -0.5)],
            tags: vec![make_tag(0, "freq", TagValue::Float(1e6))],
        };

        let out = mux.mux(&[s0, s1]);
        assert_eq!(out.data.len(), 3);
        assert_eq!(out.data[0], (1.0, 0.0));
        assert_eq!(out.data[2], (0.5, -0.5));
        assert_eq!(out.tags.len(), 2);
        assert_eq!(out.tags[0].offset, 0);
        assert_eq!(out.tags[1].offset, 2);
        assert_eq!(out.tags[1].key, "freq");
    }

    #[test]
    fn test_round_robin() {
        let mux = RoundRobinMux::new(2, 2);
        let s0 = make_stream(vec![1.0, 2.0, 3.0, 4.0], vec![]);
        let s1 = make_stream(vec![10.0, 20.0, 30.0, 40.0], vec![]);

        let out = mux.mux(&[s0, s1]);
        // chunk_size=2: [1,2] from s0, [10,20] from s1, [3,4] from s0, [30,40] from s1
        assert_eq!(out.data, vec![1.0, 2.0, 10.0, 20.0, 3.0, 4.0, 30.0, 40.0]);
    }

    #[test]
    fn test_round_robin_unequal() {
        let mux = RoundRobinMux::new(2, 3);
        let s0 = make_stream(
            vec![1.0, 2.0, 3.0, 4.0, 5.0],
            vec![make_tag(4, "last", TagValue::Bool(true))],
        );
        let s1 = make_stream(
            vec![10.0, 20.0],
            vec![make_tag(0, "short", TagValue::Int(2))],
        );

        let out = mux.mux(&[s0, s1]);
        // Round 1: s0[0..3] = [1,2,3], s1[0..2] = [10,20]
        // Round 2: s0[3..5] = [4,5], s1 exhausted
        assert_eq!(out.data, vec![1.0, 2.0, 3.0, 10.0, 20.0, 4.0, 5.0]);

        // Tag "short" at s1 offset 0 maps to output offset 3.
        let short_tag = out.tags.iter().find(|t| t.key == "short").unwrap();
        assert_eq!(short_tag.offset, 3);

        // Tag "last" at s0 offset 4 falls in round 2 chunk (s0[3..5]).
        // Output position = 5 + (4 - 3) = 6.
        let last_tag = out.tags.iter().find(|t| t.key == "last").unwrap();
        assert_eq!(last_tag.offset, 6);
    }

    #[test]
    fn test_priority_ordering() {
        let mut mux = PriorityMux::new(3);
        mux.set_priority(0, 1); // low
        mux.set_priority(1, 10); // highest
        mux.set_priority(2, 5); // medium

        let s0 = make_stream(vec![1.0], vec![make_tag(0, "low", TagValue::None)]);
        let s1 = make_stream(vec![2.0], vec![make_tag(0, "high", TagValue::None)]);
        let s2 = make_stream(vec![3.0], vec![make_tag(0, "med", TagValue::None)]);

        let out = mux.mux(&[s0, s1, s2]);
        // Order should be: input 1 (pri=10), input 2 (pri=5), input 0 (pri=1).
        assert_eq!(out.data, vec![2.0, 3.0, 1.0]);
        assert_eq!(out.tags[0].key, "high");
        assert_eq!(out.tags[0].offset, 0);
        assert_eq!(out.tags[1].key, "med");
        assert_eq!(out.tags[1].offset, 1);
        assert_eq!(out.tags[2].key, "low");
        assert_eq!(out.tags[2].offset, 2);
    }

    #[test]
    fn test_mixed_tags() {
        let mux = TaggedStreamMux::new(2);
        let s0 = make_stream(
            vec![1.0, 2.0],
            vec![
                make_tag(0, "name", TagValue::String("alpha".to_string())),
                make_tag(1, "raw", TagValue::Bytes(vec![0xDE, 0xAD])),
            ],
        );
        let s1 = make_stream(
            vec![3.0],
            vec![
                make_tag(0, "count", TagValue::Int(42)),
                make_tag(0, "flag", TagValue::Bool(false)),
                make_tag(0, "level", TagValue::Float(-3.5)),
                make_tag(0, "marker", TagValue::None),
            ],
        );

        let out = mux.mux(&[s0, s1]);
        assert_eq!(out.tags.len(), 6);

        // Verify every tag value variant survives the mux.
        assert_eq!(
            out.tags[0].value,
            TagValue::String("alpha".to_string())
        );
        assert_eq!(
            out.tags[1].value,
            TagValue::Bytes(vec![0xDE, 0xAD])
        );
        assert_eq!(out.tags[2].value, TagValue::Int(42));
        assert_eq!(out.tags[3].value, TagValue::Bool(false));
        assert_eq!(out.tags[4].value, TagValue::Float(-3.5));
        assert_eq!(out.tags[5].value, TagValue::None);

        // s1 tags all shifted by 2.
        for tag in &out.tags[2..] {
            assert_eq!(tag.offset, 2);
        }
    }

    #[test]
    fn test_large_mux() {
        let n_streams = 100;
        let samples_per_stream = 1000;
        let mux = TaggedStreamMux::new(n_streams);

        let inputs: Vec<TaggedStream> = (0..n_streams)
            .map(|i| {
                let data: Vec<f64> = (0..samples_per_stream)
                    .map(|s| (i * samples_per_stream + s) as f64)
                    .collect();
                let tags = vec![make_tag(
                    0,
                    &format!("stream_{}", i),
                    TagValue::Int(i as i64),
                )];
                make_stream(data, tags)
            })
            .collect();

        let out = mux.mux(&inputs);
        assert_eq!(out.data.len(), n_streams * samples_per_stream);
        assert_eq!(out.tags.len(), n_streams);

        // Verify tag offsets: each stream's tag should be at
        // i * samples_per_stream.
        for (i, tag) in out.tags.iter().enumerate() {
            assert_eq!(tag.offset, i * samples_per_stream);
            assert_eq!(tag.key, format!("stream_{}", i));
        }

        // Spot-check data continuity.
        assert_eq!(out.data[0], 0.0);
        assert_eq!(out.data[samples_per_stream], samples_per_stream as f64);
        assert_eq!(
            out.data[out.data.len() - 1],
            (n_streams * samples_per_stream - 1) as f64,
        );
    }
}

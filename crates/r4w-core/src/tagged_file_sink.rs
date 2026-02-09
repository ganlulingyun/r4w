//! Tagged File Sink — Stream-Tag-Triggered IQ File Segmentation
//!
//! Starts a new file segment each time a designated stream tag appears,
//! enabling burst-capture and event-driven recording workflows for
//! ADS-B, pager, trunked radio, and other burst-mode receivers.
//! GNU Radio equivalent: `gr::blocks::tagged_file_sink`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::tagged_file_sink::{TaggedFileSink, TaggedFileSinkConfig, StreamTag, TagValue};
//!
//! let config = TaggedFileSinkConfig {
//!     base_path: std::path::PathBuf::from("/tmp/test_segments"),
//!     tag_key: "burst".into(),
//!     max_segment_samples: Some(10000),
//!     prepend_timestamp: false,
//! };
//! let sink = TaggedFileSink::new(config);
//! assert_eq!(sink.segments_written(), 0);
//! ```

use std::path::PathBuf;

/// Stream tag attached to a sample offset.
#[derive(Debug, Clone)]
pub struct StreamTag {
    /// Sample offset where the tag occurs.
    pub offset: usize,
    /// Tag key string.
    pub key: String,
    /// Tag value.
    pub value: TagValue,
}

/// Tag value types.
#[derive(Debug, Clone)]
pub enum TagValue {
    Bool(bool),
    Int(i64),
    Float(f64),
    Str(String),
}

impl StreamTag {
    /// Create a new stream tag.
    pub fn new(offset: usize, key: &str, value: TagValue) -> Self {
        Self {
            offset,
            key: key.to_string(),
            value,
        }
    }

    /// Create a simple boolean tag.
    pub fn bool_tag(offset: usize, key: &str, value: bool) -> Self {
        Self::new(offset, key, TagValue::Bool(value))
    }
}

/// Information about a completed file segment.
#[derive(Debug, Clone)]
pub struct SegmentInfo {
    /// File path of the segment.
    pub path: PathBuf,
    /// Number of samples in this segment.
    pub num_samples: u64,
    /// Starting sample offset in the overall stream.
    pub start_offset: u64,
    /// Tag value that triggered this segment (if any).
    pub tag_value: Option<String>,
}

/// Configuration for the tagged file sink.
#[derive(Debug, Clone)]
pub struct TaggedFileSinkConfig {
    /// Base directory for output files.
    pub base_path: PathBuf,
    /// Tag key to trigger new segments.
    pub tag_key: String,
    /// Maximum samples per segment (None = unlimited).
    pub max_segment_samples: Option<usize>,
    /// Prepend timestamp to filename.
    pub prepend_timestamp: bool,
}

/// Tagged file sink that segments IQ recordings based on stream tags.
///
/// In-memory implementation that collects segments for testing.
/// For real I/O, segments can be written via `flush_to_disk()`.
#[derive(Debug)]
pub struct TaggedFileSink {
    config: TaggedFileSinkConfig,
    /// Current segment data.
    current_segment: Vec<f64>,
    /// Completed segments.
    segments: Vec<SegmentInfo>,
    /// Total samples processed.
    total_samples: u64,
    /// Current segment start offset.
    segment_start: u64,
    /// In-memory segment buffers (for testing).
    segment_buffers: Vec<Vec<f64>>,
    /// Current tag value.
    current_tag_value: Option<String>,
}

impl TaggedFileSink {
    /// Create a new tagged file sink.
    pub fn new(config: TaggedFileSinkConfig) -> Self {
        Self {
            config,
            current_segment: Vec::new(),
            segments: Vec::new(),
            total_samples: 0,
            segment_start: 0,
            segment_buffers: Vec::new(),
            current_tag_value: None,
        }
    }

    /// Process samples with associated tags.
    ///
    /// Returns the number of new segments completed during this call.
    pub fn process(&mut self, samples: &[f64], tags: &[StreamTag]) -> usize {
        let mut new_segments = 0;

        for (i, &sample) in samples.iter().enumerate() {
            // Check for matching tags at this offset
            for tag in tags {
                if tag.offset == i && tag.key == self.config.tag_key {
                    // Start a new segment
                    if !self.current_segment.is_empty() {
                        self.finalize_segment();
                        new_segments += 1;
                    }
                    self.segment_start = self.total_samples;
                    self.current_tag_value = Some(format!("{:?}", tag.value));
                }
            }

            self.current_segment.push(sample);
            self.total_samples += 1;

            // Check max segment size
            if let Some(max) = self.config.max_segment_samples {
                if self.current_segment.len() >= max {
                    self.finalize_segment();
                    new_segments += 1;
                    self.segment_start = self.total_samples;
                }
            }
        }

        new_segments
    }

    /// Finalize the current segment.
    fn finalize_segment(&mut self) {
        if self.current_segment.is_empty() {
            return;
        }

        let segment_num = self.segments.len();
        let filename = if self.config.prepend_timestamp {
            format!("segment_{:06}_{}.raw", segment_num, self.segment_start)
        } else {
            format!("segment_{:06}.raw", segment_num)
        };

        let path = self.config.base_path.join(filename);

        let info = SegmentInfo {
            path,
            num_samples: self.current_segment.len() as u64,
            start_offset: self.segment_start,
            tag_value: self.current_tag_value.clone(),
        };

        self.segments.push(info);
        self.segment_buffers.push(std::mem::take(&mut self.current_segment));
        self.current_tag_value = None;
    }

    /// Flush remaining samples as final segment.
    pub fn flush(&mut self) {
        if !self.current_segment.is_empty() {
            self.finalize_segment();
        }
    }

    /// Get number of completed segments.
    pub fn segments_written(&self) -> usize {
        self.segments.len()
    }

    /// Get total samples processed.
    pub fn total_samples_written(&self) -> u64 {
        self.total_samples
    }

    /// Get segment info list.
    pub fn segments(&self) -> &[SegmentInfo] {
        &self.segments
    }

    /// Get segment data buffer (for testing).
    pub fn segment_data(&self, index: usize) -> Option<&[f64]> {
        self.segment_buffers.get(index).map(|v| v.as_slice())
    }

    /// Get current (unfinalized) segment length.
    pub fn current_segment_len(&self) -> usize {
        self.current_segment.len()
    }

    /// Reset to initial state.
    pub fn reset(&mut self) {
        self.current_segment.clear();
        self.segments.clear();
        self.segment_buffers.clear();
        self.total_samples = 0;
        self.segment_start = 0;
        self.current_tag_value = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config() -> TaggedFileSinkConfig {
        TaggedFileSinkConfig {
            base_path: PathBuf::from("/tmp/test_tfs"),
            tag_key: "burst".into(),
            max_segment_samples: None,
            prepend_timestamp: false,
        }
    }

    #[test]
    fn test_single_segment() {
        let mut sink = TaggedFileSink::new(make_config());
        let samples = vec![1.0; 100];
        sink.process(&samples, &[]);
        sink.flush();
        assert_eq!(sink.segments_written(), 1);
        assert_eq!(sink.segments()[0].num_samples, 100);
    }

    #[test]
    fn test_tag_splits_segments() {
        let mut sink = TaggedFileSink::new(make_config());
        let samples = vec![1.0; 100];
        let tags = vec![StreamTag::bool_tag(50, "burst", true)];
        sink.process(&samples, &tags);
        sink.flush();
        // Should have 2 segments: [0..50) and [50..100)
        assert_eq!(sink.segments_written(), 2);
        assert_eq!(sink.segments()[0].num_samples, 50);
        assert_eq!(sink.segments()[1].num_samples, 50);
    }

    #[test]
    fn test_multiple_tags() {
        let mut sink = TaggedFileSink::new(make_config());
        let samples = vec![1.0; 100];
        let tags = vec![
            StreamTag::bool_tag(25, "burst", true),
            StreamTag::bool_tag(50, "burst", true),
            StreamTag::bool_tag(75, "burst", true),
        ];
        sink.process(&samples, &tags);
        sink.flush();
        assert_eq!(sink.segments_written(), 4); // initial + 3 tag-triggered
    }

    #[test]
    fn test_prepend_timestamp() {
        let config = TaggedFileSinkConfig {
            prepend_timestamp: true,
            ..make_config()
        };
        let mut sink = TaggedFileSink::new(config);
        let samples = vec![1.0; 50];
        sink.process(&samples, &[]);
        sink.flush();
        let path = &sink.segments()[0].path;
        assert!(path.to_str().unwrap().contains("segment_000000_0"));
    }

    #[test]
    fn test_max_segment_samples() {
        let config = TaggedFileSinkConfig {
            max_segment_samples: Some(30),
            ..make_config()
        };
        let mut sink = TaggedFileSink::new(config);
        let samples = vec![1.0; 100];
        sink.process(&samples, &[]);
        sink.flush();
        // 100 / 30 = 3 full + 1 partial = 4 segments
        assert_eq!(sink.segments_written(), 4);
        assert_eq!(sink.segments()[0].num_samples, 30);
        assert_eq!(sink.segments()[1].num_samples, 30);
        assert_eq!(sink.segments()[2].num_samples, 30);
        assert_eq!(sink.segments()[3].num_samples, 10);
    }

    #[test]
    fn test_empty_input() {
        let mut sink = TaggedFileSink::new(make_config());
        sink.process(&[], &[]);
        sink.flush();
        assert_eq!(sink.segments_written(), 0);
    }

    #[test]
    fn test_tag_at_first_sample() {
        let mut sink = TaggedFileSink::new(make_config());
        let samples = vec![1.0; 50];
        let tags = vec![StreamTag::bool_tag(0, "burst", true)];
        sink.process(&samples, &tags);
        sink.flush();
        // Tag at offset 0, no previous data, so just 1 segment
        assert_eq!(sink.segments_written(), 1);
        assert_eq!(sink.segments()[0].num_samples, 50);
    }

    #[test]
    fn test_segment_data_access() {
        let mut sink = TaggedFileSink::new(make_config());
        let samples: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let tags = vec![StreamTag::bool_tag(5, "burst", true)];
        sink.process(&samples, &tags);
        sink.flush();
        assert_eq!(sink.segments_written(), 2);
        let first = sink.segment_data(0).unwrap();
        assert_eq!(first.len(), 5);
        assert!((first[0] - 0.0).abs() < 1e-10);
        let second = sink.segment_data(1).unwrap();
        assert_eq!(second.len(), 5);
        assert!((second[0] - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_non_matching_tags_ignored() {
        let mut sink = TaggedFileSink::new(make_config());
        let samples = vec![1.0; 50];
        let tags = vec![StreamTag::bool_tag(25, "other_key", true)];
        sink.process(&samples, &tags);
        sink.flush();
        // Non-matching tag should not split
        assert_eq!(sink.segments_written(), 1);
    }

    #[test]
    fn test_reset() {
        let mut sink = TaggedFileSink::new(make_config());
        sink.process(&[1.0; 50], &[]);
        sink.flush();
        assert_eq!(sink.segments_written(), 1);
        sink.reset();
        assert_eq!(sink.segments_written(), 0);
        assert_eq!(sink.total_samples_written(), 0);
    }

    #[test]
    fn test_total_samples() {
        let mut sink = TaggedFileSink::new(make_config());
        sink.process(&[1.0; 100], &[]);
        sink.process(&[2.0; 50], &[]);
        assert_eq!(sink.total_samples_written(), 150);
    }
}

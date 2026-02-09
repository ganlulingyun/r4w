//! Stream Tags — Metadata Propagation System
//!
//! Carries key-value metadata alongside sample streams through processing
//! pipelines. Tags are attached to specific sample offsets and propagate
//! downstream through blocks.
//!
//! ## Common Tag Types
//!
//! | Key              | Type    | Meaning                          |
//! |------------------|---------|----------------------------------|
//! | `rx_time`        | Float   | Absolute reception time          |
//! | `rx_freq`        | Float   | Center frequency (Hz)            |
//! | `rx_rate`        | Float   | Sample rate (Hz)                 |
//! | `burst_start`    | Bool    | Start of burst boundary          |
//! | `burst_end`      | Bool    | End of burst boundary            |
//! | `packet_len`     | Int     | Expected packet length (samples) |
//! | `snr_est`        | Float   | Estimated SNR (dB)               |
//! | `freq_offset`    | Float   | Carrier frequency offset (Hz)    |
//! | `symbol_timing`  | Float   | Symbol timing offset (samples)   |
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::stream_tags::{StreamTag, TagValue, TagStore};
//!
//! let mut tags = TagStore::new();
//!
//! // Add a tag at sample offset 100
//! tags.add(100, "burst_start", TagValue::Bool(true));
//! tags.add(100, "packet_len", TagValue::Int(256));
//! tags.add(356, "burst_end", TagValue::Bool(true));
//!
//! // Query tags in a range
//! let burst_tags = tags.range(90, 110);
//! assert_eq!(burst_tags.len(), 2);
//! ```

use std::collections::BTreeMap;
use std::fmt;

/// Tag value — typed metadata attached to a sample offset.
#[derive(Debug, Clone, PartialEq)]
pub enum TagValue {
    /// Boolean flag (burst_start, burst_end, etc.)
    Bool(bool),
    /// Integer value (packet length, PRN, symbol index, etc.)
    Int(i64),
    /// Floating-point value (frequency, timing, SNR, etc.)
    Float(f64),
    /// String value (modulation name, block ID, etc.)
    String(String),
    /// Byte array (raw data, sync word, etc.)
    Bytes(Vec<u8>),
}

impl fmt::Display for TagValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TagValue::Bool(v) => write!(f, "{v}"),
            TagValue::Int(v) => write!(f, "{v}"),
            TagValue::Float(v) => write!(f, "{v:.6}"),
            TagValue::String(v) => write!(f, "\"{v}\""),
            TagValue::Bytes(v) => write!(f, "[{}B]", v.len()),
        }
    }
}

impl TagValue {
    /// Try to get as bool.
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            TagValue::Bool(v) => Some(*v),
            _ => None,
        }
    }

    /// Try to get as integer.
    pub fn as_int(&self) -> Option<i64> {
        match self {
            TagValue::Int(v) => Some(*v),
            _ => None,
        }
    }

    /// Try to get as float.
    pub fn as_float(&self) -> Option<f64> {
        match self {
            TagValue::Float(v) => Some(*v),
            TagValue::Int(v) => Some(*v as f64),
            _ => None,
        }
    }

    /// Try to get as string.
    pub fn as_str(&self) -> Option<&str> {
        match self {
            TagValue::String(v) => Some(v),
            _ => None,
        }
    }
}

/// A single stream tag: key-value pair at a sample offset.
#[derive(Debug, Clone, PartialEq)]
pub struct StreamTag {
    /// Sample offset where the tag applies.
    pub offset: u64,
    /// Tag key (e.g., "burst_start", "rx_freq").
    pub key: String,
    /// Tag value.
    pub value: TagValue,
    /// Source block that generated the tag (optional).
    pub source: Option<String>,
}

impl StreamTag {
    /// Create a new stream tag.
    pub fn new(offset: u64, key: impl Into<String>, value: TagValue) -> Self {
        Self {
            offset,
            key: key.into(),
            value,
            source: None,
        }
    }

    /// Create a tag with source attribution.
    pub fn with_source(
        offset: u64,
        key: impl Into<String>,
        value: TagValue,
        source: impl Into<String>,
    ) -> Self {
        Self {
            offset,
            key: key.into(),
            value,
            source: Some(source.into()),
        }
    }
}

impl fmt::Display for StreamTag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "@{}: {} = {}", self.offset, self.key, self.value)?;
        if let Some(ref src) = self.source {
            write!(f, " (from {src})")?;
        }
        Ok(())
    }
}

/// Tag store — ordered collection of stream tags.
///
/// Tags are stored sorted by offset for efficient range queries.
#[derive(Debug, Clone, Default)]
pub struct TagStore {
    /// Tags indexed by (offset, sequence) for stable ordering.
    tags: BTreeMap<(u64, u32), StreamTag>,
    /// Sequence counter for stable ordering at same offset.
    seq: u32,
}

impl TagStore {
    /// Create an empty tag store.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a tag at the given sample offset.
    pub fn add(&mut self, offset: u64, key: impl Into<String>, value: TagValue) {
        let tag = StreamTag::new(offset, key, value);
        self.tags.insert((offset, self.seq), tag);
        self.seq += 1;
    }

    /// Add a tag with source attribution.
    pub fn add_with_source(
        &mut self,
        offset: u64,
        key: impl Into<String>,
        value: TagValue,
        source: impl Into<String>,
    ) {
        let tag = StreamTag::with_source(offset, key, value, source);
        self.tags.insert((offset, self.seq), tag);
        self.seq += 1;
    }

    /// Add a pre-built StreamTag.
    pub fn add_tag(&mut self, tag: StreamTag) {
        let offset = tag.offset;
        self.tags.insert((offset, self.seq), tag);
        self.seq += 1;
    }

    /// Get all tags at a specific offset.
    pub fn at(&self, offset: u64) -> Vec<&StreamTag> {
        self.tags
            .range((offset, 0)..=(offset, u32::MAX))
            .map(|(_, tag)| tag)
            .collect()
    }

    /// Get all tags in a range [start, end).
    pub fn range(&self, start: u64, end: u64) -> Vec<&StreamTag> {
        self.tags
            .range((start, 0)..(end, 0))
            .map(|(_, tag)| tag)
            .collect()
    }

    /// Get the first tag with the given key at or after the offset.
    pub fn find(&self, offset: u64, key: &str) -> Option<&StreamTag> {
        self.tags
            .range((offset, 0)..)
            .map(|(_, tag)| tag)
            .find(|tag| tag.key == key)
    }

    /// Get all tags with a specific key.
    pub fn by_key(&self, key: &str) -> Vec<&StreamTag> {
        self.tags
            .values()
            .filter(|tag| tag.key == key)
            .collect()
    }

    /// Get all tags (sorted by offset).
    pub fn all(&self) -> Vec<&StreamTag> {
        self.tags.values().collect()
    }

    /// Number of tags in the store.
    pub fn len(&self) -> usize {
        self.tags.len()
    }

    /// Check if the store is empty.
    pub fn is_empty(&self) -> bool {
        self.tags.is_empty()
    }

    /// Clear all tags.
    pub fn clear(&mut self) {
        self.tags.clear();
        self.seq = 0;
    }

    /// Shift all tag offsets by the given delta.
    ///
    /// Used when consuming samples from the front of a buffer.
    pub fn shift(&mut self, delta: u64) {
        let shifted: Vec<((u64, u32), StreamTag)> = self
            .tags
            .iter()
            .filter_map(|((offset, seq), tag)| {
                if *offset >= delta {
                    let mut new_tag = tag.clone();
                    new_tag.offset -= delta;
                    Some(((new_tag.offset, *seq), new_tag))
                } else {
                    None // Drop tags before the shift point
                }
            })
            .collect();

        self.tags.clear();
        for (key, tag) in shifted {
            self.tags.insert(key, tag);
        }
    }

    /// Merge tags from another store.
    pub fn merge(&mut self, other: &TagStore) {
        for tag in other.all() {
            self.add_tag(tag.clone());
        }
    }

    /// Remove all tags before the given offset.
    pub fn trim_before(&mut self, offset: u64) {
        self.tags.retain(|&(off, _), _| off >= offset);
    }
}

/// Tag propagation policy for processing blocks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TagPropagation {
    /// Pass all tags through (1:1 mapping).
    AllToAll,
    /// Only propagate tags that match the output sample rate.
    OneToOne,
    /// Don't propagate tags (block generates its own).
    None,
}

/// Well-known tag key constants.
pub mod keys {
    /// Absolute reception time (TagValue::Float, seconds).
    pub const RX_TIME: &str = "rx_time";
    /// Center frequency in Hz (TagValue::Float).
    pub const RX_FREQ: &str = "rx_freq";
    /// Sample rate in Hz (TagValue::Float).
    pub const RX_RATE: &str = "rx_rate";
    /// Start of burst marker (TagValue::Bool).
    pub const BURST_START: &str = "burst_start";
    /// End of burst marker (TagValue::Bool).
    pub const BURST_END: &str = "burst_end";
    /// Expected packet length in samples (TagValue::Int).
    pub const PACKET_LEN: &str = "packet_len";
    /// Estimated SNR in dB (TagValue::Float).
    pub const SNR_EST: &str = "snr_est";
    /// Carrier frequency offset in Hz (TagValue::Float).
    pub const FREQ_OFFSET: &str = "freq_offset";
    /// Symbol timing offset in samples (TagValue::Float).
    pub const SYMBOL_TIMING: &str = "symbol_timing";
    /// Modulation type string (TagValue::String).
    pub const MODULATION: &str = "modulation";
    /// Block name that produced the tag (TagValue::String).
    pub const SOURCE_BLOCK: &str = "source_block";
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tag_store_basic() {
        let mut store = TagStore::new();
        store.add(0, "rx_freq", TagValue::Float(915e6));
        store.add(100, "burst_start", TagValue::Bool(true));
        store.add(356, "burst_end", TagValue::Bool(true));

        assert_eq!(store.len(), 3);
    }

    #[test]
    fn test_tag_at_offset() {
        let mut store = TagStore::new();
        store.add(100, "burst_start", TagValue::Bool(true));
        store.add(100, "packet_len", TagValue::Int(256));
        store.add(200, "other", TagValue::Int(42));

        let tags = store.at(100);
        assert_eq!(tags.len(), 2);
        assert!(tags.iter().any(|t| t.key == "burst_start"));
        assert!(tags.iter().any(|t| t.key == "packet_len"));
    }

    #[test]
    fn test_tag_range_query() {
        let mut store = TagStore::new();
        store.add(50, "a", TagValue::Bool(true));
        store.add(100, "b", TagValue::Bool(true));
        store.add(150, "c", TagValue::Bool(true));
        store.add(200, "d", TagValue::Bool(true));

        let range = store.range(90, 160);
        assert_eq!(range.len(), 2);
        assert_eq!(range[0].key, "b");
        assert_eq!(range[1].key, "c");
    }

    #[test]
    fn test_tag_by_key() {
        let mut store = TagStore::new();
        store.add(0, "rx_freq", TagValue::Float(915e6));
        store.add(100, "burst_start", TagValue::Bool(true));
        store.add(1000, "rx_freq", TagValue::Float(920e6));

        let freq_tags = store.by_key("rx_freq");
        assert_eq!(freq_tags.len(), 2);
    }

    #[test]
    fn test_tag_find() {
        let mut store = TagStore::new();
        store.add(50, "burst_start", TagValue::Bool(true));
        store.add(200, "burst_start", TagValue::Bool(true));

        let found = store.find(100, "burst_start");
        assert!(found.is_some());
        assert_eq!(found.unwrap().offset, 200);
    }

    #[test]
    fn test_tag_shift() {
        let mut store = TagStore::new();
        store.add(100, "a", TagValue::Int(1));
        store.add(200, "b", TagValue::Int(2));
        store.add(300, "c", TagValue::Int(3));

        store.shift(150);

        // Tag at 100 should be dropped (before shift)
        assert_eq!(store.len(), 2);
        let all = store.all();
        assert_eq!(all[0].offset, 50);  // 200 - 150
        assert_eq!(all[1].offset, 150); // 300 - 150
    }

    #[test]
    fn test_tag_merge() {
        let mut store1 = TagStore::new();
        store1.add(0, "a", TagValue::Int(1));

        let mut store2 = TagStore::new();
        store2.add(100, "b", TagValue::Int(2));

        store1.merge(&store2);
        assert_eq!(store1.len(), 2);
    }

    #[test]
    fn test_tag_trim() {
        let mut store = TagStore::new();
        store.add(50, "a", TagValue::Int(1));
        store.add(100, "b", TagValue::Int(2));
        store.add(200, "c", TagValue::Int(3));

        store.trim_before(100);
        assert_eq!(store.len(), 2);
    }

    #[test]
    fn test_tag_value_accessors() {
        assert_eq!(TagValue::Bool(true).as_bool(), Some(true));
        assert_eq!(TagValue::Int(42).as_int(), Some(42));
        assert_eq!(TagValue::Float(3.14).as_float(), Some(3.14));
        assert_eq!(TagValue::Int(42).as_float(), Some(42.0));
        assert_eq!(
            TagValue::String("hello".into()).as_str(),
            Some("hello")
        );
        assert_eq!(TagValue::Bool(true).as_int(), None);
    }

    #[test]
    fn test_tag_display() {
        let tag = StreamTag::new(100, "rx_freq", TagValue::Float(915e6));
        let s = format!("{tag}");
        assert!(s.contains("100"));
        assert!(s.contains("rx_freq"));
    }

    #[test]
    fn test_tag_with_source() {
        let tag = StreamTag::with_source(100, "snr_est", TagValue::Float(15.0), "snr_block");
        assert_eq!(tag.source.as_deref(), Some("snr_block"));
    }

    #[test]
    fn test_well_known_keys() {
        assert_eq!(keys::RX_TIME, "rx_time");
        assert_eq!(keys::BURST_START, "burst_start");
        assert_eq!(keys::PACKET_LEN, "packet_len");
    }

    #[test]
    fn test_empty_store() {
        let store = TagStore::new();
        assert!(store.is_empty());
        assert_eq!(store.len(), 0);
        assert!(store.all().is_empty());
    }
}

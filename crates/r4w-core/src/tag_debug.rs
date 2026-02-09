//! Tag Debug and Manipulation Blocks
//!
//! Blocks for inspecting, filtering, and generating stream tags.
//! Complements the `stream_tags` module by providing debug/control blocks.
//!
//! ## Blocks
//!
//! - **TagDebug**: Captures and displays tags for debugging
//! - **TagGate**: Selectively blocks or passes tags
//! - **StreamToTaggedStream**: Inserts packet_len tags at regular intervals
//! - **TagsStrobe**: Injects user-defined tags at regular intervals
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::tag_debug::{TagDebug, StreamToTaggedStream};
//! use r4w_core::stream_tags::{TagStore, TagValue};
//!
//! let mut debug = TagDebug::new("my_debug");
//! let mut tags = TagStore::new();
//! tags.add(0, "freq".to_string(), TagValue::Float(1575.42e6));
//! tags.add(10, "gain".to_string(), TagValue::Float(30.0));
//!
//! let captured = debug.process(&tags, 0, 20);
//! assert_eq!(captured.len(), 2);
//! ```

use crate::stream_tags::{StreamTag, TagStore, TagValue};

/// Tag debugger — captures and inspects tags for debugging.
#[derive(Debug, Clone)]
pub struct TagDebug {
    /// Block name for display
    name: String,
    /// Optional key filter (None = capture all)
    key_filter: Option<String>,
    /// Maximum tags to capture (0 = unlimited)
    display_limit: usize,
    /// Collected tags
    collected: Vec<StreamTag>,
}

impl TagDebug {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            key_filter: None,
            display_limit: 0,
            collected: Vec::new(),
        }
    }

    pub fn with_key_filter(name: impl Into<String>, key: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            key_filter: Some(key.into()),
            display_limit: 0,
            collected: Vec::new(),
        }
    }

    /// Process tags in range [start, end), capturing matching ones.
    pub fn process(&mut self, tags: &TagStore, start: u64, end: u64) -> Vec<StreamTag> {
        let matching: Vec<StreamTag> = tags
            .range(start, end)
            .into_iter()
            .filter(|t| {
                if let Some(ref filter) = self.key_filter {
                    &t.key == filter
                } else {
                    true
                }
            })
            .cloned()
            .collect();

        for tag in &matching {
            if self.display_limit == 0 || self.collected.len() < self.display_limit {
                self.collected.push(tag.clone());
            }
        }
        matching
    }

    /// Get all collected tags.
    pub fn collected(&self) -> &[StreamTag] {
        &self.collected
    }

    /// Format a single tag for display.
    pub fn format_tag(tag: &StreamTag) -> String {
        format!(
            "[offset={}] {} = {}",
            tag.offset,
            tag.key,
            format_tag_value(&tag.value)
        )
    }

    /// Get summary string.
    pub fn summary(&self) -> String {
        let mut keys: Vec<&str> = self
            .collected
            .iter()
            .map(|t| t.key.as_str())
            .collect();
        keys.sort();
        keys.dedup();
        format!(
            "{}: {} tags captured, keys: [{}]",
            self.name,
            self.collected.len(),
            keys.join(", ")
        )
    }

    pub fn reset(&mut self) {
        self.collected.clear();
    }

    pub fn name(&self) -> &str {
        &self.name
    }
}

/// Tag gate — selectively blocks or passes tags.
#[derive(Debug, Clone)]
pub struct TagGate {
    /// Key to gate (None = gate all tags)
    gate_key: Option<String>,
    /// Whether tags pass through (true = pass, false = block)
    enabled: bool,
}

impl TagGate {
    pub fn new() -> Self {
        Self {
            gate_key: None,
            enabled: true,
        }
    }

    pub fn with_key(key: impl Into<String>) -> Self {
        Self {
            gate_key: Some(key.into()),
            enabled: true,
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Filter tags: pass or block based on gate state.
    pub fn filter(&self, tags: &TagStore, start: u64, end: u64) -> TagStore {
        let mut output = TagStore::new();
        for tag in tags.range(start, end) {
            let should_block = match &self.gate_key {
                Some(key) => &tag.key == key && !self.enabled,
                None => !self.enabled,
            };
            if !should_block {
                output.add(tag.offset, tag.key.clone(), tag.value.clone());
            }
        }
        output
    }
}

impl Default for TagGate {
    fn default() -> Self {
        Self::new()
    }
}

/// Converts a plain stream into a tagged stream by inserting packet_len tags.
#[derive(Debug, Clone)]
pub struct StreamToTaggedStream {
    /// Packet length in samples
    packet_len: usize,
    /// Tag key to use
    length_tag_key: String,
}

impl StreamToTaggedStream {
    pub fn new(packet_len: usize) -> Self {
        assert!(packet_len > 0);
        Self {
            packet_len,
            length_tag_key: "packet_len".to_string(),
        }
    }

    pub fn with_key(packet_len: usize, key: impl Into<String>) -> Self {
        assert!(packet_len > 0);
        Self {
            packet_len,
            length_tag_key: key.into(),
        }
    }

    /// Generate tags for a given number of samples starting at offset.
    pub fn generate_tags(&self, num_samples: u64, start_offset: u64) -> TagStore {
        let mut tags = TagStore::new();
        let mut offset = start_offset;
        while offset < start_offset + num_samples {
            tags.add(
                offset,
                self.length_tag_key.clone(),
                TagValue::Int(self.packet_len as i64),
            );
            offset += self.packet_len as u64;
        }
        tags
    }

    pub fn packet_len(&self) -> usize {
        self.packet_len
    }
}

/// Tags strobe — injects user-defined tags at regular intervals.
#[derive(Debug, Clone)]
pub struct TagsStrobe {
    /// Tags to inject (key, value)
    tags: Vec<(String, TagValue)>,
    /// Inject every N samples
    interval_samples: u64,
    /// Current position
    offset: u64,
}

impl TagsStrobe {
    pub fn new(interval_samples: u64) -> Self {
        assert!(interval_samples > 0);
        Self {
            tags: Vec::new(),
            interval_samples,
            offset: 0,
        }
    }

    /// Add a tag to inject at each strobe point.
    pub fn add_tag(&mut self, key: impl Into<String>, value: TagValue) {
        self.tags.push((key.into(), value));
    }

    /// Process N samples and return tags to inject.
    pub fn process(&mut self, num_samples: u64) -> TagStore {
        let mut store = TagStore::new();
        let end = self.offset + num_samples;
        // Find first strobe point at or after current offset
        let mut strobe = (self.offset / self.interval_samples) * self.interval_samples;
        if strobe < self.offset {
            strobe += self.interval_samples;
        }
        while strobe < end {
            for (key, value) in &self.tags {
                store.add(strobe, key.clone(), value.clone());
            }
            strobe += self.interval_samples;
        }
        self.offset = end;
        store
    }

    pub fn reset(&mut self) {
        self.offset = 0;
    }
}

fn format_tag_value(value: &TagValue) -> String {
    match value {
        TagValue::Float(f) => format!("{:.6}", f),
        TagValue::Int(i) => format!("{}", i),
        TagValue::Bool(b) => format!("{}", b),
        TagValue::String(s) => s.clone(),
        TagValue::Bytes(b) => format!("[{} bytes]", b.len()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_tags() -> TagStore {
        let mut tags = TagStore::new();
        tags.add(0, "freq".to_string(), TagValue::Float(1575.42e6));
        tags.add(10, "gain".to_string(), TagValue::Float(30.0));
        tags.add(20, "freq".to_string(), TagValue::Float(1227.60e6));
        tags
    }

    #[test]
    fn test_tag_debug_capture_all() {
        let mut debug = TagDebug::new("test");
        let tags = make_test_tags();
        let captured = debug.process(&tags, 0, 30);
        assert_eq!(captured.len(), 3);
        assert_eq!(debug.collected().len(), 3);
    }

    #[test]
    fn test_tag_debug_key_filter() {
        let mut debug = TagDebug::with_key_filter("test", "freq");
        let tags = make_test_tags();
        let captured = debug.process(&tags, 0, 30);
        assert_eq!(captured.len(), 2); // Only "freq" tags
    }

    #[test]
    fn test_tag_debug_range() {
        let mut debug = TagDebug::new("test");
        let tags = make_test_tags();
        let captured = debug.process(&tags, 5, 15);
        assert_eq!(captured.len(), 1); // Only tag at offset 10
    }

    #[test]
    fn test_tag_debug_summary() {
        let mut debug = TagDebug::new("my_block");
        let tags = make_test_tags();
        debug.process(&tags, 0, 30);
        let summary = debug.summary();
        assert!(summary.contains("my_block"));
        assert!(summary.contains("3 tags"));
    }

    #[test]
    fn test_tag_debug_reset() {
        let mut debug = TagDebug::new("test");
        let tags = make_test_tags();
        debug.process(&tags, 0, 30);
        assert!(!debug.collected().is_empty());
        debug.reset();
        assert!(debug.collected().is_empty());
    }

    #[test]
    fn test_tag_gate_pass_all() {
        let gate = TagGate::new(); // enabled by default
        let tags = make_test_tags();
        let filtered = gate.filter(&tags, 0, 30);
        assert_eq!(filtered.range(0, 30).len(), 3);
    }

    #[test]
    fn test_tag_gate_block_all() {
        let mut gate = TagGate::new();
        gate.set_enabled(false);
        let tags = make_test_tags();
        let filtered = gate.filter(&tags, 0, 30);
        assert_eq!(filtered.range(0, 30).len(), 0);
    }

    #[test]
    fn test_tag_gate_block_specific_key() {
        let mut gate = TagGate::with_key("gain");
        gate.set_enabled(false);
        let tags = make_test_tags();
        let filtered = gate.filter(&tags, 0, 30);
        // Should block "gain" but pass "freq" tags
        let remaining = filtered.range(0, 30);
        assert_eq!(remaining.len(), 2);
        for t in &remaining {
            assert_eq!(t.key, "freq");
        }
    }

    #[test]
    fn test_stream_to_tagged_stream() {
        let stts = StreamToTaggedStream::new(100);
        let tags = stts.generate_tags(500, 0);
        let all = tags.range(0, 500);
        assert_eq!(all.len(), 5); // At 0, 100, 200, 300, 400
        assert_eq!(all[0].offset, 0);
        assert_eq!(all[1].offset, 100);
    }

    #[test]
    fn test_stream_to_tagged_stream_custom_key() {
        let stts = StreamToTaggedStream::with_key(50, "pdu_len");
        let tags = stts.generate_tags(100, 0);
        let all = tags.range(0, 100);
        assert_eq!(all.len(), 2);
        assert_eq!(all[0].key, "pdu_len");
    }

    #[test]
    fn test_tags_strobe() {
        let mut strobe = TagsStrobe::new(10);
        strobe.add_tag("marker", TagValue::Bool(true));
        let tags = strobe.process(25);
        let all = tags.range(0, 25);
        // Strobes at 0, 10, 20
        assert_eq!(all.len(), 3);
    }

    #[test]
    fn test_tags_strobe_multiple_tags() {
        let mut strobe = TagsStrobe::new(100);
        strobe.add_tag("freq", TagValue::Float(1575.42e6));
        strobe.add_tag("power", TagValue::Float(-20.0));
        let tags = strobe.process(100);
        let all = tags.range(0, 100);
        assert_eq!(all.len(), 2); // Both tags at offset 0
    }

    #[test]
    fn test_tags_strobe_streaming() {
        let mut strobe = TagsStrobe::new(10);
        strobe.add_tag("tick", TagValue::Int(1));
        let t1 = strobe.process(15); // Strobes at 0, 10
        let t2 = strobe.process(10); // Strobe at 20
        assert_eq!(t1.range(0, 15).len(), 2);
        assert_eq!(t2.range(15, 25).len(), 1);
    }

    #[test]
    fn test_format_tag() {
        let tag = StreamTag::new(42, "freq", TagValue::Float(1575.42e6));
        let s = TagDebug::format_tag(&tag);
        assert!(s.contains("42"));
        assert!(s.contains("freq"));
    }
}

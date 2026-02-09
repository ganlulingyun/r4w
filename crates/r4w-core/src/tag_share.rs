//! Tag Share — Stream tag propagation and manipulation
//!
//! Utilities for sharing, copying, filtering, and transforming
//! stream tags between processing stages. Manages tag offset
//! adjustment during rate changes and provides tag-based
//! event signaling.
//! GNU Radio equivalent: `tag_share`, tag propagation policies.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::tag_share::{StreamTag, TagStore};
//!
//! let mut store = TagStore::new();
//! store.add(StreamTag::new(0, "burst_start", "true"));
//! store.add(StreamTag::new(100, "burst_end", "true"));
//! let tags = store.get_range(0, 50);
//! assert_eq!(tags.len(), 1);
//! assert_eq!(tags[0].key(), "burst_start");
//! ```

/// A stream tag with position, key, and value.
#[derive(Debug, Clone, PartialEq)]
pub struct StreamTag {
    /// Sample offset where the tag applies.
    offset: u64,
    /// Tag key (identifier).
    key: String,
    /// Tag value.
    value: String,
}

impl StreamTag {
    /// Create a new stream tag.
    pub fn new(offset: u64, key: &str, value: &str) -> Self {
        Self {
            offset,
            key: key.to_string(),
            value: value.to_string(),
        }
    }

    /// Get offset.
    pub fn offset(&self) -> u64 {
        self.offset
    }

    /// Get key.
    pub fn key(&self) -> &str {
        &self.key
    }

    /// Get value.
    pub fn value(&self) -> &str {
        &self.value
    }

    /// Create with adjusted offset (for rate changes).
    pub fn with_offset(&self, new_offset: u64) -> Self {
        Self {
            offset: new_offset,
            key: self.key.clone(),
            value: self.value.clone(),
        }
    }
}

/// Tag storage with range queries and manipulation.
#[derive(Debug, Clone)]
pub struct TagStore {
    tags: Vec<StreamTag>,
}

impl TagStore {
    /// Create an empty tag store.
    pub fn new() -> Self {
        Self { tags: Vec::new() }
    }

    /// Add a tag.
    pub fn add(&mut self, tag: StreamTag) {
        self.tags.push(tag);
    }

    /// Add multiple tags.
    pub fn add_all(&mut self, tags: &[StreamTag]) {
        self.tags.extend_from_slice(tags);
    }

    /// Get all tags in range [start, end).
    pub fn get_range(&self, start: u64, end: u64) -> Vec<&StreamTag> {
        self.tags
            .iter()
            .filter(|t| t.offset >= start && t.offset < end)
            .collect()
    }

    /// Get all tags with a specific key.
    pub fn get_by_key(&self, key: &str) -> Vec<&StreamTag> {
        self.tags.iter().filter(|t| t.key == key).collect()
    }

    /// Get all tags at a specific offset.
    pub fn get_at(&self, offset: u64) -> Vec<&StreamTag> {
        self.tags.iter().filter(|t| t.offset == offset).collect()
    }

    /// Remove all tags before a given offset (garbage collection).
    pub fn prune_before(&mut self, offset: u64) {
        self.tags.retain(|t| t.offset >= offset);
    }

    /// Adjust all tag offsets by a delta.
    pub fn shift_offsets(&mut self, delta: i64) {
        for tag in &mut self.tags {
            if delta >= 0 {
                tag.offset = tag.offset.saturating_add(delta as u64);
            } else {
                tag.offset = tag.offset.saturating_sub((-delta) as u64);
            }
        }
    }

    /// Scale tag offsets by a factor (for rate changes).
    ///
    /// `factor`: output_rate / input_rate
    pub fn scale_offsets(&mut self, factor: f64) {
        for tag in &mut self.tags {
            tag.offset = (tag.offset as f64 * factor) as u64;
        }
    }

    /// Number of tags stored.
    pub fn len(&self) -> usize {
        self.tags.len()
    }

    /// Is the store empty?
    pub fn is_empty(&self) -> bool {
        self.tags.is_empty()
    }

    /// Get all tags (sorted by offset).
    pub fn all_sorted(&self) -> Vec<&StreamTag> {
        let mut tags: Vec<&StreamTag> = self.tags.iter().collect();
        tags.sort_by_key(|t| t.offset);
        tags
    }

    /// Clear all tags.
    pub fn clear(&mut self) {
        self.tags.clear();
    }

    /// Filter tags, keeping only those matching a predicate.
    pub fn filter<F: Fn(&StreamTag) -> bool>(&mut self, predicate: F) {
        self.tags.retain(|t| predicate(t));
    }
}

impl Default for TagStore {
    fn default() -> Self {
        Self::new()
    }
}

/// Propagate tags through a rate change (decimation or interpolation).
///
/// Adjusts offsets: `new_offset = old_offset * output_rate / input_rate`.
pub fn propagate_tags(tags: &[StreamTag], rate_factor: f64) -> Vec<StreamTag> {
    tags.iter()
        .map(|t| {
            let new_offset = (t.offset as f64 * rate_factor) as u64;
            t.with_offset(new_offset)
        })
        .collect()
}

/// Merge two tag streams, sorted by offset.
pub fn merge_tags(a: &[StreamTag], b: &[StreamTag]) -> Vec<StreamTag> {
    let mut merged: Vec<StreamTag> = a.iter().chain(b.iter()).cloned().collect();
    merged.sort_by_key(|t| t.offset);
    merged
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tag_creation() {
        let tag = StreamTag::new(42, "freq", "1000");
        assert_eq!(tag.offset(), 42);
        assert_eq!(tag.key(), "freq");
        assert_eq!(tag.value(), "1000");
    }

    #[test]
    fn test_with_offset() {
        let tag = StreamTag::new(10, "key", "val");
        let adjusted = tag.with_offset(20);
        assert_eq!(adjusted.offset(), 20);
        assert_eq!(adjusted.key(), "key");
    }

    #[test]
    fn test_store_add_and_get() {
        let mut store = TagStore::new();
        store.add(StreamTag::new(0, "start", "1"));
        store.add(StreamTag::new(50, "mid", "2"));
        store.add(StreamTag::new(100, "end", "3"));
        assert_eq!(store.len(), 3);
    }

    #[test]
    fn test_get_range() {
        let mut store = TagStore::new();
        store.add(StreamTag::new(10, "a", "1"));
        store.add(StreamTag::new(20, "b", "2"));
        store.add(StreamTag::new(30, "c", "3"));
        let tags = store.get_range(15, 25);
        assert_eq!(tags.len(), 1);
        assert_eq!(tags[0].key(), "b");
    }

    #[test]
    fn test_get_by_key() {
        let mut store = TagStore::new();
        store.add(StreamTag::new(0, "freq", "100"));
        store.add(StreamTag::new(50, "gain", "20"));
        store.add(StreamTag::new(100, "freq", "200"));
        let freq_tags = store.get_by_key("freq");
        assert_eq!(freq_tags.len(), 2);
    }

    #[test]
    fn test_get_at() {
        let mut store = TagStore::new();
        store.add(StreamTag::new(10, "a", "1"));
        store.add(StreamTag::new(10, "b", "2"));
        store.add(StreamTag::new(20, "c", "3"));
        assert_eq!(store.get_at(10).len(), 2);
        assert_eq!(store.get_at(20).len(), 1);
        assert_eq!(store.get_at(30).len(), 0);
    }

    #[test]
    fn test_prune() {
        let mut store = TagStore::new();
        store.add(StreamTag::new(10, "old", "1"));
        store.add(StreamTag::new(50, "new", "2"));
        store.prune_before(30);
        assert_eq!(store.len(), 1);
        assert_eq!(store.get_by_key("new").len(), 1);
    }

    #[test]
    fn test_shift_offsets() {
        let mut store = TagStore::new();
        store.add(StreamTag::new(10, "a", "1"));
        store.add(StreamTag::new(20, "b", "2"));
        store.shift_offsets(5);
        assert_eq!(store.get_at(15).len(), 1);
        assert_eq!(store.get_at(25).len(), 1);
    }

    #[test]
    fn test_shift_negative() {
        let mut store = TagStore::new();
        store.add(StreamTag::new(10, "a", "1"));
        store.shift_offsets(-3);
        assert_eq!(store.get_at(7).len(), 1);
    }

    #[test]
    fn test_scale_offsets() {
        let mut store = TagStore::new();
        store.add(StreamTag::new(100, "a", "1"));
        store.scale_offsets(0.5); // Decimation by 2
        assert_eq!(store.get_at(50).len(), 1);
    }

    #[test]
    fn test_propagate_tags() {
        let tags = vec![
            StreamTag::new(0, "start", "1"),
            StreamTag::new(100, "end", "1"),
        ];
        let propagated = propagate_tags(&tags, 2.0); // 2x interp
        assert_eq!(propagated[0].offset(), 0);
        assert_eq!(propagated[1].offset(), 200);
    }

    #[test]
    fn test_merge_tags() {
        let a = vec![StreamTag::new(10, "a", "1"), StreamTag::new(30, "c", "3")];
        let b = vec![StreamTag::new(20, "b", "2")];
        let merged = merge_tags(&a, &b);
        assert_eq!(merged.len(), 3);
        assert_eq!(merged[0].offset(), 10);
        assert_eq!(merged[1].offset(), 20);
        assert_eq!(merged[2].offset(), 30);
    }

    #[test]
    fn test_all_sorted() {
        let mut store = TagStore::new();
        store.add(StreamTag::new(30, "c", "3"));
        store.add(StreamTag::new(10, "a", "1"));
        store.add(StreamTag::new(20, "b", "2"));
        let sorted = store.all_sorted();
        assert_eq!(sorted[0].offset(), 10);
        assert_eq!(sorted[1].offset(), 20);
        assert_eq!(sorted[2].offset(), 30);
    }

    #[test]
    fn test_clear() {
        let mut store = TagStore::new();
        store.add(StreamTag::new(0, "a", "1"));
        store.clear();
        assert!(store.is_empty());
    }

    #[test]
    fn test_default() {
        let store = TagStore::default();
        assert!(store.is_empty());
    }
}

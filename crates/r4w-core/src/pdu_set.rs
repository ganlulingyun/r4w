//! # PDU Metadata Set/Modify
//!
//! Set, modify, or delete metadata fields on Protocol Data Units (PDUs).
//! Complements `pdu_filter` (which selects PDUs by metadata) by providing
//! mutation capabilities for PDU metadata in processing pipelines.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::pdu_set::{PduSet, MetaValue, MetaAction};
//!
//! let mut processor = PduSet::new();
//! processor.add_action("channel", MetaAction::Set(MetaValue::Int(5)));
//! processor.add_action("timestamp", MetaAction::Set(MetaValue::Float(1234.5)));
//!
//! let mut meta = Vec::new();
//! let data = vec![0xDE, 0xAD];
//! let result_meta = processor.apply(&data, &meta);
//! assert_eq!(result_meta.len(), 2);
//! ```

use std::collections::HashMap;

/// Metadata value types.
#[derive(Debug, Clone, PartialEq)]
pub enum MetaValue {
    /// Integer value.
    Int(i64),
    /// Floating-point value.
    Float(f64),
    /// String value.
    Str(String),
    /// Boolean value.
    Bool(bool),
    /// Byte vector.
    Bytes(Vec<u8>),
}

impl MetaValue {
    /// Get as integer.
    pub fn as_int(&self) -> Option<i64> {
        match self {
            MetaValue::Int(v) => Some(*v),
            _ => None,
        }
    }

    /// Get as float.
    pub fn as_float(&self) -> Option<f64> {
        match self {
            MetaValue::Float(v) => Some(*v),
            _ => None,
        }
    }

    /// Get as string.
    pub fn as_str(&self) -> Option<&str> {
        match self {
            MetaValue::Str(v) => Some(v),
            _ => None,
        }
    }

    /// Get as bool.
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            MetaValue::Bool(v) => Some(*v),
            _ => None,
        }
    }
}

/// Action to perform on a metadata field.
#[derive(Debug, Clone)]
pub enum MetaAction {
    /// Set to a specific value (overwrite if exists).
    Set(MetaValue),
    /// Delete the field if it exists.
    Delete,
    /// Set only if the field doesn't already exist.
    SetIfMissing(MetaValue),
    /// Increment an integer field by the given amount.
    Increment(i64),
    /// Append to a string field.
    Append(String),
}

/// A PDU with metadata.
#[derive(Debug, Clone)]
pub struct PduMeta {
    /// Field name.
    pub key: String,
    /// Field value.
    pub value: MetaValue,
}

/// PDU metadata set/modify processor.
#[derive(Debug, Clone)]
pub struct PduSet {
    /// Ordered list of actions to apply.
    actions: Vec<(String, MetaAction)>,
    /// Total PDUs processed.
    pdus_processed: u64,
}

impl PduSet {
    /// Create a new PDU set processor.
    pub fn new() -> Self {
        Self {
            actions: Vec::new(),
            pdus_processed: 0,
        }
    }

    /// Add a metadata action.
    pub fn add_action(&mut self, key: &str, action: MetaAction) {
        self.actions.push((key.to_string(), action));
    }

    /// Apply all actions to PDU metadata. Returns updated metadata.
    pub fn apply(&mut self, _data: &[u8], meta: &[PduMeta]) -> Vec<PduMeta> {
        self.pdus_processed += 1;

        // Build a map for easier manipulation.
        let mut map: HashMap<String, MetaValue> = HashMap::new();
        let mut order: Vec<String> = Vec::new();
        for m in meta {
            if !map.contains_key(&m.key) {
                order.push(m.key.clone());
            }
            map.insert(m.key.clone(), m.value.clone());
        }

        for (key, action) in &self.actions {
            match action {
                MetaAction::Set(val) => {
                    if !map.contains_key(key) {
                        order.push(key.clone());
                    }
                    map.insert(key.clone(), val.clone());
                }
                MetaAction::Delete => {
                    map.remove(key);
                    order.retain(|k| k != key);
                }
                MetaAction::SetIfMissing(val) => {
                    if !map.contains_key(key) {
                        order.push(key.clone());
                        map.insert(key.clone(), val.clone());
                    }
                }
                MetaAction::Increment(delta) => {
                    if let Some(MetaValue::Int(v)) = map.get(key) {
                        map.insert(key.clone(), MetaValue::Int(v + delta));
                    } else if !map.contains_key(key) {
                        order.push(key.clone());
                        map.insert(key.clone(), MetaValue::Int(*delta));
                    }
                }
                MetaAction::Append(suffix) => {
                    if let Some(MetaValue::Str(v)) = map.get(key) {
                        let new_val = format!("{}{}", v, suffix);
                        map.insert(key.clone(), MetaValue::Str(new_val));
                    } else if !map.contains_key(key) {
                        order.push(key.clone());
                        map.insert(key.clone(), MetaValue::Str(suffix.clone()));
                    }
                }
            }
        }

        // Reconstruct ordered metadata.
        order
            .into_iter()
            .filter_map(|key| {
                map.remove(&key).map(|value| PduMeta { key, value })
            })
            .collect()
    }

    /// Apply actions and return a full PDU (data + updated metadata).
    pub fn process(&mut self, data: &[u8], meta: &[PduMeta]) -> (Vec<u8>, Vec<PduMeta>) {
        let new_meta = self.apply(data, meta);
        (data.to_vec(), new_meta)
    }

    /// Get total PDUs processed.
    pub fn pdus_processed(&self) -> u64 {
        self.pdus_processed
    }

    /// Get the number of configured actions.
    pub fn num_actions(&self) -> usize {
        self.actions.len()
    }

    /// Clear all actions.
    pub fn clear_actions(&mut self) {
        self.actions.clear();
    }

    /// Reset counters.
    pub fn reset(&mut self) {
        self.pdus_processed = 0;
    }
}

impl Default for PduSet {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_set_field() {
        let mut ps = PduSet::new();
        ps.add_action("freq", MetaAction::Set(MetaValue::Float(915e6)));
        let result = ps.apply(&[], &[]);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].key, "freq");
        assert_eq!(result[0].value.as_float(), Some(915e6));
    }

    #[test]
    fn test_overwrite_field() {
        let mut ps = PduSet::new();
        ps.add_action("ch", MetaAction::Set(MetaValue::Int(7)));
        let existing = vec![PduMeta {
            key: "ch".to_string(),
            value: MetaValue::Int(3),
        }];
        let result = ps.apply(&[], &existing);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].value.as_int(), Some(7));
    }

    #[test]
    fn test_delete_field() {
        let mut ps = PduSet::new();
        ps.add_action("temp", MetaAction::Delete);
        let existing = vec![
            PduMeta { key: "temp".to_string(), value: MetaValue::Float(25.0) },
            PduMeta { key: "id".to_string(), value: MetaValue::Int(42) },
        ];
        let result = ps.apply(&[], &existing);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].key, "id");
    }

    #[test]
    fn test_set_if_missing() {
        let mut ps = PduSet::new();
        ps.add_action("flag", MetaAction::SetIfMissing(MetaValue::Bool(true)));
        // Missing → should set.
        let result1 = ps.apply(&[], &[]);
        assert_eq!(result1.len(), 1);
        assert_eq!(result1[0].value.as_bool(), Some(true));

        // Existing → should not overwrite.
        let existing = vec![PduMeta {
            key: "flag".to_string(),
            value: MetaValue::Bool(false),
        }];
        let result2 = ps.apply(&[], &existing);
        assert_eq!(result2[0].value.as_bool(), Some(false));
    }

    #[test]
    fn test_increment() {
        let mut ps = PduSet::new();
        ps.add_action("count", MetaAction::Increment(1));
        let existing = vec![PduMeta {
            key: "count".to_string(),
            value: MetaValue::Int(5),
        }];
        let result = ps.apply(&[], &existing);
        assert_eq!(result[0].value.as_int(), Some(6));
    }

    #[test]
    fn test_increment_missing() {
        let mut ps = PduSet::new();
        ps.add_action("seq", MetaAction::Increment(10));
        let result = ps.apply(&[], &[]);
        assert_eq!(result[0].value.as_int(), Some(10));
    }

    #[test]
    fn test_append() {
        let mut ps = PduSet::new();
        ps.add_action("tag", MetaAction::Append("_v2".to_string()));
        let existing = vec![PduMeta {
            key: "tag".to_string(),
            value: MetaValue::Str("data".to_string()),
        }];
        let result = ps.apply(&[], &existing);
        assert_eq!(result[0].value.as_str(), Some("data_v2"));
    }

    #[test]
    fn test_multiple_actions() {
        let mut ps = PduSet::new();
        ps.add_action("freq", MetaAction::Set(MetaValue::Float(2.4e9)));
        ps.add_action("power", MetaAction::Set(MetaValue::Float(-30.0)));
        ps.add_action("old", MetaAction::Delete);
        let existing = vec![PduMeta {
            key: "old".to_string(),
            value: MetaValue::Str("stale".to_string()),
        }];
        let result = ps.apply(&[0xDE, 0xAD], &existing);
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn test_process() {
        let mut ps = PduSet::new();
        ps.add_action("ch", MetaAction::Set(MetaValue::Int(1)));
        let (data, meta) = ps.process(&[0x01, 0x02], &[]);
        assert_eq!(data, vec![0x01, 0x02]);
        assert_eq!(meta.len(), 1);
        assert_eq!(ps.pdus_processed(), 1);
    }

    #[test]
    fn test_clear_and_reset() {
        let mut ps = PduSet::new();
        ps.add_action("a", MetaAction::Set(MetaValue::Int(1)));
        ps.add_action("b", MetaAction::Set(MetaValue::Int(2)));
        assert_eq!(ps.num_actions(), 2);
        ps.apply(&[], &[]);
        assert_eq!(ps.pdus_processed(), 1);
        ps.clear_actions();
        assert_eq!(ps.num_actions(), 0);
        ps.reset();
        assert_eq!(ps.pdus_processed(), 0);
    }
}

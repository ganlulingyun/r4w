//! PDU Filter — Filter and route PDUs by metadata
//!
//! Filters Protocol Data Units based on metadata key-value matching.
//! Can accept, reject, or route PDUs to different outputs based on
//! configurable rules. Useful for protocol demuxing, conditional
//! processing, and stream selection.
//! GNU Radio equivalent: `pdu_filter`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::pdu_filter::{PduFilter, FilterRule, PduMeta, MetaValue};
//!
//! let mut filter = PduFilter::new();
//! filter.add_rule(FilterRule::require("type", MetaValue::Str("data".into())));
//!
//! let mut meta = PduMeta::new();
//! meta.insert("type", MetaValue::Str("data".into()));
//! assert!(filter.accept(&meta));
//!
//! let mut meta2 = PduMeta::new();
//! meta2.insert("type", MetaValue::Str("control".into()));
//! assert!(!filter.accept(&meta2));
//! ```

use std::collections::HashMap;

/// Metadata value types for PDU filtering.
#[derive(Debug, Clone, PartialEq)]
pub enum MetaValue {
    /// String value.
    Str(String),
    /// Integer value.
    Int(i64),
    /// Float value.
    Float(f64),
    /// Boolean value.
    Bool(bool),
}

impl MetaValue {
    /// Check if this value matches another (type-aware comparison).
    pub fn matches(&self, other: &MetaValue) -> bool {
        match (self, other) {
            (MetaValue::Str(a), MetaValue::Str(b)) => a == b,
            (MetaValue::Int(a), MetaValue::Int(b)) => a == b,
            (MetaValue::Float(a), MetaValue::Float(b)) => (a - b).abs() < 1e-10,
            (MetaValue::Bool(a), MetaValue::Bool(b)) => a == b,
            (MetaValue::Int(a), MetaValue::Float(b)) => (*a as f64 - b).abs() < 1e-10,
            (MetaValue::Float(a), MetaValue::Int(b)) => (a - *b as f64).abs() < 1e-10,
            _ => false,
        }
    }
}

impl std::fmt::Display for MetaValue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MetaValue::Str(s) => write!(f, "{}", s),
            MetaValue::Int(i) => write!(f, "{}", i),
            MetaValue::Float(v) => write!(f, "{:.6}", v),
            MetaValue::Bool(b) => write!(f, "{}", b),
        }
    }
}

/// PDU metadata (key-value pairs).
#[derive(Debug, Clone)]
pub struct PduMeta {
    entries: HashMap<String, MetaValue>,
}

impl PduMeta {
    /// Create empty metadata.
    pub fn new() -> Self {
        Self {
            entries: HashMap::new(),
        }
    }

    /// Insert a metadata field.
    pub fn insert(&mut self, key: &str, value: MetaValue) {
        self.entries.insert(key.to_string(), value);
    }

    /// Get a metadata field.
    pub fn get(&self, key: &str) -> Option<&MetaValue> {
        self.entries.get(key)
    }

    /// Check if a key exists.
    pub fn contains_key(&self, key: &str) -> bool {
        self.entries.contains_key(key)
    }

    /// Get all keys.
    pub fn keys(&self) -> impl Iterator<Item = &str> {
        self.entries.keys().map(|s| s.as_str())
    }

    /// Number of entries.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

impl Default for PduMeta {
    fn default() -> Self {
        Self::new()
    }
}

/// Filter rule for PDU matching.
#[derive(Debug, Clone)]
pub struct FilterRule {
    /// Key to match.
    pub key: String,
    /// Expected value (None = just check key exists).
    pub value: Option<MetaValue>,
    /// Whether this is a positive (require) or negative (reject) rule.
    pub mode: FilterMode,
}

/// Filter rule mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterMode {
    /// PDU must match this rule.
    Require,
    /// PDU must NOT match this rule.
    Reject,
}

impl FilterRule {
    /// Create a rule requiring key=value.
    pub fn require(key: &str, value: MetaValue) -> Self {
        Self {
            key: key.to_string(),
            value: Some(value),
            mode: FilterMode::Require,
        }
    }

    /// Create a rule requiring key to exist (any value).
    pub fn require_key(key: &str) -> Self {
        Self {
            key: key.to_string(),
            value: None,
            mode: FilterMode::Require,
        }
    }

    /// Create a rule rejecting key=value.
    pub fn reject(key: &str, value: MetaValue) -> Self {
        Self {
            key: key.to_string(),
            value: Some(value),
            mode: FilterMode::Reject,
        }
    }

    /// Create a rule rejecting key existence.
    pub fn reject_key(key: &str) -> Self {
        Self {
            key: key.to_string(),
            value: None,
            mode: FilterMode::Reject,
        }
    }

    /// Check if a PDU matches this rule.
    pub fn matches(&self, meta: &PduMeta) -> bool {
        match (&self.value, meta.get(&self.key)) {
            (Some(expected), Some(actual)) => expected.matches(actual),
            (None, Some(_)) => true,  // Key exists, no value check
            (_, None) => false,       // Key doesn't exist
        }
    }
}

/// PDU filter with configurable rules.
#[derive(Debug, Clone)]
pub struct PduFilter {
    rules: Vec<FilterRule>,
}

impl PduFilter {
    /// Create an empty filter (accepts everything).
    pub fn new() -> Self {
        Self { rules: Vec::new() }
    }

    /// Add a filter rule.
    pub fn add_rule(&mut self, rule: FilterRule) {
        self.rules.push(rule);
    }

    /// Check if a PDU should be accepted.
    ///
    /// Logic: All Require rules must match AND no Reject rules must match.
    pub fn accept(&self, meta: &PduMeta) -> bool {
        for rule in &self.rules {
            let matched = rule.matches(meta);
            match rule.mode {
                FilterMode::Require => {
                    if !matched {
                        return false;
                    }
                }
                FilterMode::Reject => {
                    if matched {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// Filter a batch of PDUs.
    pub fn filter_batch<'a, T>(&self, pdus: &'a [(PduMeta, T)]) -> Vec<&'a (PduMeta, T)> {
        pdus.iter().filter(|(meta, _)| self.accept(meta)).collect()
    }

    /// Get number of rules.
    pub fn num_rules(&self) -> usize {
        self.rules.len()
    }

    /// Clear all rules.
    pub fn clear(&mut self) {
        self.rules.clear();
    }
}

impl Default for PduFilter {
    fn default() -> Self {
        Self::new()
    }
}

/// PDU router: routes PDUs to numbered outputs based on metadata.
#[derive(Debug, Clone)]
pub struct PduRouter {
    routes: Vec<(FilterRule, usize)>,
    default_output: usize,
}

impl PduRouter {
    /// Create a router with given default output index.
    pub fn new(default_output: usize) -> Self {
        Self {
            routes: Vec::new(),
            default_output,
        }
    }

    /// Add a routing rule: if rule matches, send to output `idx`.
    pub fn add_route(&mut self, rule: FilterRule, output_idx: usize) {
        self.routes.push((rule, output_idx));
    }

    /// Route a PDU, returning the output index.
    /// First matching route wins.
    pub fn route(&self, meta: &PduMeta) -> usize {
        for (rule, idx) in &self.routes {
            if rule.matches(meta) {
                return *idx;
            }
        }
        self.default_output
    }

    /// Route a batch, returning (output_index, pdu_index) pairs.
    pub fn route_batch<T>(&self, pdus: &[(PduMeta, T)]) -> Vec<(usize, usize)> {
        pdus.iter()
            .enumerate()
            .map(|(i, (meta, _))| (self.route(meta), i))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_filter_accepts_all() {
        let filter = PduFilter::new();
        let meta = PduMeta::new();
        assert!(filter.accept(&meta));
    }

    #[test]
    fn test_require_string() {
        let mut filter = PduFilter::new();
        filter.add_rule(FilterRule::require("type", MetaValue::Str("data".into())));

        let mut meta = PduMeta::new();
        meta.insert("type", MetaValue::Str("data".into()));
        assert!(filter.accept(&meta));

        let mut meta2 = PduMeta::new();
        meta2.insert("type", MetaValue::Str("control".into()));
        assert!(!filter.accept(&meta2));
    }

    #[test]
    fn test_require_key_exists() {
        let mut filter = PduFilter::new();
        filter.add_rule(FilterRule::require_key("timestamp"));

        let mut meta = PduMeta::new();
        meta.insert("timestamp", MetaValue::Int(12345));
        assert!(filter.accept(&meta));

        let meta2 = PduMeta::new();
        assert!(!filter.accept(&meta2));
    }

    #[test]
    fn test_reject_rule() {
        let mut filter = PduFilter::new();
        filter.add_rule(FilterRule::reject("type", MetaValue::Str("noise".into())));

        let meta = PduMeta::new();
        assert!(filter.accept(&meta)); // No "type" key → not rejected

        let mut meta2 = PduMeta::new();
        meta2.insert("type", MetaValue::Str("noise".into()));
        assert!(!filter.accept(&meta2)); // Matches reject rule

        let mut meta3 = PduMeta::new();
        meta3.insert("type", MetaValue::Str("data".into()));
        assert!(filter.accept(&meta3)); // Different value → not rejected
    }

    #[test]
    fn test_multiple_rules() {
        let mut filter = PduFilter::new();
        filter.add_rule(FilterRule::require("type", MetaValue::Str("data".into())));
        filter.add_rule(FilterRule::require("channel", MetaValue::Int(1)));

        let mut meta = PduMeta::new();
        meta.insert("type", MetaValue::Str("data".into()));
        meta.insert("channel", MetaValue::Int(1));
        assert!(filter.accept(&meta));

        let mut meta2 = PduMeta::new();
        meta2.insert("type", MetaValue::Str("data".into()));
        meta2.insert("channel", MetaValue::Int(2));
        assert!(!filter.accept(&meta2));
    }

    #[test]
    fn test_int_float_comparison() {
        let rule = FilterRule::require("snr", MetaValue::Float(10.0));
        let mut meta = PduMeta::new();
        meta.insert("snr", MetaValue::Int(10));
        assert!(rule.matches(&meta)); // Int 10 matches Float 10.0
    }

    #[test]
    fn test_filter_batch() {
        let mut filter = PduFilter::new();
        filter.add_rule(FilterRule::require("keep", MetaValue::Bool(true)));

        let mut m1 = PduMeta::new();
        m1.insert("keep", MetaValue::Bool(true));
        let mut m2 = PduMeta::new();
        m2.insert("keep", MetaValue::Bool(false));
        let mut m3 = PduMeta::new();
        m3.insert("keep", MetaValue::Bool(true));

        let pdus = vec![(m1, "pdu1"), (m2, "pdu2"), (m3, "pdu3")];
        let accepted = filter.filter_batch(&pdus);
        assert_eq!(accepted.len(), 2);
    }

    #[test]
    fn test_router_basic() {
        let mut router = PduRouter::new(0);
        router.add_route(
            FilterRule::require("type", MetaValue::Str("data".into())),
            1,
        );
        router.add_route(
            FilterRule::require("type", MetaValue::Str("control".into())),
            2,
        );

        let mut m_data = PduMeta::new();
        m_data.insert("type", MetaValue::Str("data".into()));
        assert_eq!(router.route(&m_data), 1);

        let mut m_ctrl = PduMeta::new();
        m_ctrl.insert("type", MetaValue::Str("control".into()));
        assert_eq!(router.route(&m_ctrl), 2);

        let m_unknown = PduMeta::new();
        assert_eq!(router.route(&m_unknown), 0); // default
    }

    #[test]
    fn test_router_first_match_wins() {
        let mut router = PduRouter::new(0);
        router.add_route(FilterRule::require_key("priority"), 1);
        router.add_route(FilterRule::require_key("priority"), 2);

        let mut meta = PduMeta::new();
        meta.insert("priority", MetaValue::Int(5));
        assert_eq!(router.route(&meta), 1); // First rule matches
    }

    #[test]
    fn test_meta_operations() {
        let mut meta = PduMeta::new();
        assert!(meta.is_empty());
        meta.insert("key1", MetaValue::Str("val".into()));
        assert_eq!(meta.len(), 1);
        assert!(meta.contains_key("key1"));
        assert!(!meta.contains_key("key2"));
    }

    #[test]
    fn test_meta_value_display() {
        assert_eq!(format!("{}", MetaValue::Str("hello".into())), "hello");
        assert_eq!(format!("{}", MetaValue::Int(42)), "42");
        assert_eq!(format!("{}", MetaValue::Bool(true)), "true");
    }

    #[test]
    fn test_reject_key() {
        let mut filter = PduFilter::new();
        filter.add_rule(FilterRule::reject_key("error"));

        let meta = PduMeta::new();
        assert!(filter.accept(&meta));

        let mut meta2 = PduMeta::new();
        meta2.insert("error", MetaValue::Str("timeout".into()));
        assert!(!filter.accept(&meta2));
    }

    #[test]
    fn test_clear_rules() {
        let mut filter = PduFilter::new();
        filter.add_rule(FilterRule::require("x", MetaValue::Int(1)));
        assert_eq!(filter.num_rules(), 1);
        filter.clear();
        assert_eq!(filter.num_rules(), 0);
    }
}

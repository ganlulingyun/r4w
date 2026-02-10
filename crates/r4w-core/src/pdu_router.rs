//! # PDU Router
//!
//! Routes PDUs to different outputs based on metadata field values,
//! PDU length, or custom predicates. Essential for protocol demuxing,
//! multi-channel receivers, and conditional processing pipelines.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::pdu_router::{PduRouter, RoutingRule};
//!
//! let mut router = PduRouter::new(3); // 3 output ports
//! router.add_rule(RoutingRule::by_field("type", "data", 0));
//! router.add_rule(RoutingRule::by_field("type", "control", 1));
//! router.set_default_port(2);
//!
//! let pdu = vec![("type".into(), "data".into())];
//! let port = router.route(&pdu, &[]);
//! assert_eq!(port, 0);
//! ```

/// Routing rule for PDU classification.
#[derive(Debug, Clone)]
pub enum RoutingRule {
    /// Route by metadata field value.
    FieldEquals {
        field: String,
        value: String,
        port: usize,
    },
    /// Route by minimum PDU length.
    MinLength {
        min_bytes: usize,
        port: usize,
    },
    /// Route by maximum PDU length.
    MaxLength {
        max_bytes: usize,
        port: usize,
    },
    /// Route by length range.
    LengthRange {
        min_bytes: usize,
        max_bytes: usize,
        port: usize,
    },
    /// Route by byte prefix match.
    PrefixMatch {
        prefix: Vec<u8>,
        port: usize,
    },
}

impl RoutingRule {
    /// Create a field-equals routing rule.
    pub fn by_field(field: &str, value: &str, port: usize) -> Self {
        Self::FieldEquals {
            field: field.to_string(),
            value: value.to_string(),
            port,
        }
    }

    /// Create a minimum length routing rule.
    pub fn min_length(min_bytes: usize, port: usize) -> Self {
        Self::MinLength { min_bytes, port }
    }

    /// Create a length range routing rule.
    pub fn length_range(min_bytes: usize, max_bytes: usize, port: usize) -> Self {
        Self::LengthRange {
            min_bytes,
            max_bytes,
            port,
        }
    }

    /// Create a prefix match routing rule.
    pub fn prefix(prefix: &[u8], port: usize) -> Self {
        Self::PrefixMatch {
            prefix: prefix.to_vec(),
            port,
        }
    }

    /// Check if this rule matches the given metadata and data.
    pub fn matches(&self, metadata: &[(String, String)], data: &[u8]) -> bool {
        match self {
            Self::FieldEquals { field, value, .. } => {
                metadata.iter().any(|(k, v)| k == field && v == value)
            }
            Self::MinLength { min_bytes, .. } => data.len() >= *min_bytes,
            Self::MaxLength { max_bytes, .. } => data.len() <= *max_bytes,
            Self::LengthRange {
                min_bytes,
                max_bytes,
                ..
            } => data.len() >= *min_bytes && data.len() <= *max_bytes,
            Self::PrefixMatch { prefix, .. } => data.starts_with(prefix),
        }
    }

    /// Get the target port for this rule.
    pub fn port(&self) -> usize {
        match self {
            Self::FieldEquals { port, .. }
            | Self::MinLength { port, .. }
            | Self::MaxLength { port, .. }
            | Self::LengthRange { port, .. }
            | Self::PrefixMatch { port, .. } => *port,
        }
    }
}

/// PDU router with configurable rules.
#[derive(Debug, Clone)]
pub struct PduRouter {
    num_ports: usize,
    rules: Vec<RoutingRule>,
    default_port: usize,
    routed_counts: Vec<u64>,
}

impl PduRouter {
    /// Create a new router with the given number of output ports.
    pub fn new(num_ports: usize) -> Self {
        Self {
            num_ports: num_ports.max(1),
            rules: Vec::new(),
            default_port: 0,
            routed_counts: vec![0; num_ports.max(1)],
        }
    }

    /// Add a routing rule (checked in order).
    pub fn add_rule(&mut self, rule: RoutingRule) {
        self.rules.push(rule);
    }

    /// Set the default port for unmatched PDUs.
    pub fn set_default_port(&mut self, port: usize) {
        self.default_port = port.min(self.num_ports - 1);
    }

    /// Route a PDU, returning the output port index.
    pub fn route(&mut self, metadata: &[(String, String)], data: &[u8]) -> usize {
        for rule in &self.rules {
            if rule.matches(metadata, data) {
                let port = rule.port().min(self.num_ports - 1);
                self.routed_counts[port] += 1;
                return port;
            }
        }
        let port = self.default_port;
        self.routed_counts[port] += 1;
        port
    }

    /// Route a batch of PDUs, returning (port, index) pairs.
    pub fn route_batch(
        &mut self,
        pdus: &[(Vec<(String, String)>, Vec<u8>)],
    ) -> Vec<(usize, usize)> {
        pdus.iter()
            .enumerate()
            .map(|(i, (meta, data))| (self.route(meta, data), i))
            .collect()
    }

    /// Get routed counts per port.
    pub fn routed_counts(&self) -> &[u64] {
        &self.routed_counts
    }

    /// Get number of output ports.
    pub fn num_ports(&self) -> usize {
        self.num_ports
    }

    /// Get number of rules.
    pub fn num_rules(&self) -> usize {
        self.rules.len()
    }

    /// Reset counters.
    pub fn reset_counts(&mut self) {
        for c in &mut self.routed_counts {
            *c = 0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_field_routing() {
        let mut router = PduRouter::new(3);
        router.add_rule(RoutingRule::by_field("type", "data", 0));
        router.add_rule(RoutingRule::by_field("type", "control", 1));
        router.set_default_port(2);

        let meta = vec![("type".into(), "data".into())];
        assert_eq!(router.route(&meta, &[]), 0);

        let meta2 = vec![("type".into(), "control".into())];
        assert_eq!(router.route(&meta2, &[]), 1);

        let meta3 = vec![("type".into(), "unknown".into())];
        assert_eq!(router.route(&meta3, &[]), 2);
    }

    #[test]
    fn test_length_routing() {
        let mut router = PduRouter::new(2);
        router.add_rule(RoutingRule::min_length(100, 1));
        router.set_default_port(0);

        assert_eq!(router.route(&[], &vec![0; 50]), 0);
        assert_eq!(router.route(&[], &vec![0; 200]), 1);
    }

    #[test]
    fn test_length_range() {
        let mut router = PduRouter::new(3);
        router.add_rule(RoutingRule::length_range(0, 10, 0));
        router.add_rule(RoutingRule::length_range(11, 100, 1));
        router.set_default_port(2);

        assert_eq!(router.route(&[], &vec![0; 5]), 0);
        assert_eq!(router.route(&[], &vec![0; 50]), 1);
        assert_eq!(router.route(&[], &vec![0; 200]), 2);
    }

    #[test]
    fn test_prefix_routing() {
        let mut router = PduRouter::new(2);
        router.add_rule(RoutingRule::prefix(&[0xAA, 0x55], 1));
        router.set_default_port(0);

        assert_eq!(router.route(&[], &[0xAA, 0x55, 0x01]), 1);
        assert_eq!(router.route(&[], &[0xBB, 0x55, 0x01]), 0);
    }

    #[test]
    fn test_routed_counts() {
        let mut router = PduRouter::new(2);
        router.add_rule(RoutingRule::by_field("ch", "a", 0));
        router.set_default_port(1);

        let meta_a = vec![("ch".into(), "a".into())];
        let meta_b = vec![("ch".into(), "b".into())];
        router.route(&meta_a, &[]);
        router.route(&meta_a, &[]);
        router.route(&meta_b, &[]);

        assert_eq!(router.routed_counts()[0], 2);
        assert_eq!(router.routed_counts()[1], 1);
    }

    #[test]
    fn test_route_batch() {
        let mut router = PduRouter::new(2);
        router.add_rule(RoutingRule::min_length(10, 1));
        router.set_default_port(0);

        let pdus = vec![
            (vec![], vec![0; 5]),
            (vec![], vec![0; 20]),
            (vec![], vec![0; 3]),
        ];
        let result = router.route_batch(&pdus);
        assert_eq!(result[0], (0, 0));
        assert_eq!(result[1], (1, 1));
        assert_eq!(result[2], (0, 2));
    }

    #[test]
    fn test_priority_order() {
        // First matching rule wins.
        let mut router = PduRouter::new(3);
        router.add_rule(RoutingRule::by_field("pri", "high", 0));
        router.add_rule(RoutingRule::by_field("pri", "high", 2)); // Should never match.
        let meta = vec![("pri".into(), "high".into())];
        assert_eq!(router.route(&meta, &[]), 0);
    }

    #[test]
    fn test_reset_counts() {
        let mut router = PduRouter::new(2);
        router.set_default_port(0);
        router.route(&[], &[]);
        router.route(&[], &[]);
        assert_eq!(router.routed_counts()[0], 2);
        router.reset_counts();
        assert_eq!(router.routed_counts()[0], 0);
    }

    #[test]
    fn test_empty_rules() {
        let mut router = PduRouter::new(3);
        router.set_default_port(2);
        assert_eq!(router.route(&[], &[1, 2, 3]), 2);
    }

    #[test]
    fn test_rule_accessors() {
        let rule = RoutingRule::by_field("x", "y", 5);
        assert_eq!(rule.port(), 5);
        assert!(rule.matches(&[("x".into(), "y".into())], &[]));
        assert!(!rule.matches(&[("x".into(), "z".into())], &[]));
    }
}

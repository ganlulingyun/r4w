//! Message Strobe — Periodic message/PDU generation and filtering
//!
//! Generates PDU messages at configurable intervals for protocol testing,
//! heartbeats, and driving packet-based pipelines. Also provides message
//! filtering and counting utilities.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::message_strobe::{MessageStrobe, Message};
//! use std::time::Duration;
//!
//! let msg = Message::new(b"Hello".to_vec());
//! let mut strobe = MessageStrobe::new(msg, Duration::from_millis(100));
//! // Advance time and generate messages
//! let messages = strobe.tick(Duration::from_millis(350));
//! assert_eq!(messages.len(), 3); // 3 periods elapsed
//! ```

use std::time::Duration;
use std::collections::HashMap;

/// A message/PDU with payload and metadata.
#[derive(Debug, Clone, PartialEq)]
pub struct Message {
    /// Payload bytes.
    pub payload: Vec<u8>,
    /// Key-value metadata.
    pub metadata: HashMap<String, String>,
}

impl Message {
    /// Create a message with payload only.
    pub fn new(payload: Vec<u8>) -> Self {
        Self {
            payload,
            metadata: HashMap::new(),
        }
    }

    /// Create with payload and metadata.
    pub fn with_metadata(payload: Vec<u8>, metadata: HashMap<String, String>) -> Self {
        Self { payload, metadata }
    }

    /// Create an empty message.
    pub fn empty() -> Self {
        Self::new(Vec::new())
    }

    /// Get payload length.
    pub fn len(&self) -> usize {
        self.payload.len()
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.payload.is_empty()
    }

    /// Set a metadata key.
    pub fn set_meta(&mut self, key: &str, value: &str) {
        self.metadata.insert(key.to_string(), value.to_string());
    }

    /// Get a metadata value.
    pub fn get_meta(&self, key: &str) -> Option<&str> {
        self.metadata.get(key).map(|s| s.as_str())
    }
}

/// Periodic message generator.
#[derive(Debug, Clone)]
pub struct MessageStrobe {
    /// Message template to emit.
    message: Message,
    /// Emission period.
    period: Duration,
    /// Accumulated time since last emission.
    elapsed: Duration,
    /// Total messages emitted.
    count: u64,
    /// Maximum number of messages (None = unlimited).
    max_count: Option<u64>,
}

impl MessageStrobe {
    /// Create a message strobe.
    pub fn new(message: Message, period: Duration) -> Self {
        Self {
            message,
            period,
            elapsed: Duration::ZERO,
            count: 0,
            max_count: None,
        }
    }

    /// Create with a maximum message count.
    pub fn with_limit(message: Message, period: Duration, max_count: u64) -> Self {
        Self {
            message,
            period,
            elapsed: Duration::ZERO,
            count: 0,
            max_count: Some(max_count),
        }
    }

    /// Advance time and return any messages that should be emitted.
    pub fn tick(&mut self, dt: Duration) -> Vec<Message> {
        let mut messages = Vec::new();
        self.elapsed += dt;
        while self.elapsed >= self.period {
            if let Some(max) = self.max_count {
                if self.count >= max {
                    break;
                }
            }
            let mut msg = self.message.clone();
            msg.set_meta("seq", &self.count.to_string());
            messages.push(msg);
            self.count += 1;
            self.elapsed -= self.period;
        }
        messages
    }

    /// Get total messages emitted.
    pub fn count(&self) -> u64 {
        self.count
    }

    /// Set the message template.
    pub fn set_message(&mut self, message: Message) {
        self.message = message;
    }

    /// Set the period.
    pub fn set_period(&mut self, period: Duration) {
        self.period = period;
    }

    /// Reset the strobe state.
    pub fn reset(&mut self) {
        self.elapsed = Duration::ZERO;
        self.count = 0;
    }
}

/// Message filter — passes or blocks messages by metadata key/value match.
#[derive(Debug, Clone)]
pub struct MessageFilter {
    key: String,
    value: String,
    invert: bool,
}

impl MessageFilter {
    /// Create a filter that passes messages where `key == value`.
    pub fn new(key: &str, value: &str) -> Self {
        Self {
            key: key.to_string(),
            value: value.to_string(),
            invert: false,
        }
    }

    /// Create an inverted filter (blocks matching, passes non-matching).
    pub fn inverted(key: &str, value: &str) -> Self {
        Self {
            key: key.to_string(),
            value: value.to_string(),
            invert: true,
        }
    }

    /// Filter a message. Returns Some if it passes.
    pub fn process(&self, msg: &Message) -> Option<Message> {
        let matches = msg.get_meta(&self.key) == Some(&self.value);
        if matches ^ self.invert {
            Some(msg.clone())
        } else {
            None
        }
    }

    /// Filter a batch of messages.
    pub fn process_batch(&self, messages: &[Message]) -> Vec<Message> {
        messages.iter().filter_map(|m| self.process(m)).collect()
    }
}

/// Message counter — counts PDUs and optionally stops after N.
#[derive(Debug, Clone)]
pub struct MessageCounter {
    count: u64,
    limit: Option<u64>,
}

impl MessageCounter {
    /// Create an unlimited counter.
    pub fn new() -> Self {
        Self { count: 0, limit: None }
    }

    /// Create with a limit.
    pub fn with_limit(limit: u64) -> Self {
        Self { count: 0, limit: Some(limit) }
    }

    /// Count a message. Returns (count, is_at_limit).
    pub fn process(&mut self, _msg: &Message) -> (u64, bool) {
        self.count += 1;
        let at_limit = self.limit.map_or(false, |l| self.count >= l);
        (self.count, at_limit)
    }

    /// Get current count.
    pub fn count(&self) -> u64 {
        self.count
    }

    /// Reset counter.
    pub fn reset(&mut self) {
        self.count = 0;
    }
}

impl Default for MessageCounter {
    fn default() -> Self {
        Self::new()
    }
}

/// Message burst generator — emits N copies of a message at once.
#[derive(Debug, Clone)]
pub struct MessageBurst {
    message: Message,
    count: usize,
}

impl MessageBurst {
    /// Create a burst generator.
    pub fn new(message: Message, count: usize) -> Self {
        Self { message, count }
    }

    /// Generate the burst.
    pub fn generate(&self) -> Vec<Message> {
        (0..self.count).map(|i| {
            let mut msg = self.message.clone();
            msg.set_meta("burst_idx", &i.to_string());
            msg
        }).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_creation() {
        let msg = Message::new(b"test".to_vec());
        assert_eq!(msg.len(), 4);
        assert!(!msg.is_empty());
    }

    #[test]
    fn test_message_metadata() {
        let mut msg = Message::new(b"data".to_vec());
        msg.set_meta("type", "beacon");
        assert_eq!(msg.get_meta("type"), Some("beacon"));
        assert_eq!(msg.get_meta("missing"), None);
    }

    #[test]
    fn test_strobe_generates_messages() {
        let msg = Message::new(b"ping".to_vec());
        let mut strobe = MessageStrobe::new(msg, Duration::from_millis(100));
        let messages = strobe.tick(Duration::from_millis(350));
        assert_eq!(messages.len(), 3);
        assert_eq!(strobe.count(), 3);
    }

    #[test]
    fn test_strobe_no_messages_before_period() {
        let msg = Message::new(b"ping".to_vec());
        let mut strobe = MessageStrobe::new(msg, Duration::from_millis(100));
        let messages = strobe.tick(Duration::from_millis(50));
        assert!(messages.is_empty());
    }

    #[test]
    fn test_strobe_accumulates() {
        let msg = Message::new(b"ping".to_vec());
        let mut strobe = MessageStrobe::new(msg, Duration::from_millis(100));
        let m1 = strobe.tick(Duration::from_millis(60));
        assert!(m1.is_empty());
        let m2 = strobe.tick(Duration::from_millis(60));
        assert_eq!(m2.len(), 1); // 60+60 = 120ms >= 100ms
    }

    #[test]
    fn test_strobe_with_limit() {
        let msg = Message::new(b"ping".to_vec());
        let mut strobe = MessageStrobe::with_limit(msg, Duration::from_millis(10), 5);
        let messages = strobe.tick(Duration::from_millis(1000));
        assert_eq!(messages.len(), 5);
        assert_eq!(strobe.count(), 5);
    }

    #[test]
    fn test_strobe_sequence_numbers() {
        let msg = Message::new(b"test".to_vec());
        let mut strobe = MessageStrobe::new(msg, Duration::from_millis(100));
        let messages = strobe.tick(Duration::from_millis(300));
        assert_eq!(messages[0].get_meta("seq"), Some("0"));
        assert_eq!(messages[1].get_meta("seq"), Some("1"));
        assert_eq!(messages[2].get_meta("seq"), Some("2"));
    }

    #[test]
    fn test_strobe_reset() {
        let msg = Message::new(b"test".to_vec());
        let mut strobe = MessageStrobe::new(msg, Duration::from_millis(100));
        strobe.tick(Duration::from_millis(500));
        assert!(strobe.count() > 0);
        strobe.reset();
        assert_eq!(strobe.count(), 0);
    }

    #[test]
    fn test_filter_passes_matching() {
        let filter = MessageFilter::new("type", "beacon");
        let mut msg = Message::new(b"data".to_vec());
        msg.set_meta("type", "beacon");
        assert!(filter.process(&msg).is_some());
    }

    #[test]
    fn test_filter_blocks_non_matching() {
        let filter = MessageFilter::new("type", "beacon");
        let mut msg = Message::new(b"data".to_vec());
        msg.set_meta("type", "data");
        assert!(filter.process(&msg).is_none());
    }

    #[test]
    fn test_filter_inverted() {
        let filter = MessageFilter::inverted("type", "beacon");
        let mut msg = Message::new(b"data".to_vec());
        msg.set_meta("type", "beacon");
        assert!(filter.process(&msg).is_none()); // Inverted: blocks matching

        let mut msg2 = Message::new(b"data".to_vec());
        msg2.set_meta("type", "data");
        assert!(filter.process(&msg2).is_some()); // Passes non-matching
    }

    #[test]
    fn test_counter() {
        let mut counter = MessageCounter::new();
        let msg = Message::new(b"test".to_vec());
        let (c1, _) = counter.process(&msg);
        let (c2, _) = counter.process(&msg);
        assert_eq!(c1, 1);
        assert_eq!(c2, 2);
    }

    #[test]
    fn test_counter_with_limit() {
        let mut counter = MessageCounter::with_limit(3);
        let msg = Message::new(b"test".to_vec());
        let (_, at_limit) = counter.process(&msg);
        assert!(!at_limit);
        counter.process(&msg);
        let (_, at_limit) = counter.process(&msg);
        assert!(at_limit); // 3rd message hits limit
    }

    #[test]
    fn test_burst_generator() {
        let msg = Message::new(b"burst".to_vec());
        let burst = MessageBurst::new(msg, 5);
        let messages = burst.generate();
        assert_eq!(messages.len(), 5);
        assert_eq!(messages[0].get_meta("burst_idx"), Some("0"));
        assert_eq!(messages[4].get_meta("burst_idx"), Some("4"));
    }

    #[test]
    fn test_filter_batch() {
        let filter = MessageFilter::new("priority", "high");
        let mut m1 = Message::new(b"a".to_vec());
        m1.set_meta("priority", "high");
        let mut m2 = Message::new(b"b".to_vec());
        m2.set_meta("priority", "low");
        let mut m3 = Message::new(b"c".to_vec());
        m3.set_meta("priority", "high");
        let results = filter.process_batch(&[m1, m2, m3]);
        assert_eq!(results.len(), 2);
    }
}

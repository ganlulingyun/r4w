//! # Message Port
//!
//! Asynchronous message passing between DSP blocks. Provides typed
//! message ports for control, status, and metadata exchange outside
//! the sample stream. Similar to GNU Radio's message passing system.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::message_port::{MessagePort, Message};
//!
//! let mut port = MessagePort::new("control");
//! port.send(Message::command("set_freq", "1.0e9"));
//! let msg = port.recv();
//! assert!(msg.is_some());
//! assert_eq!(msg.unwrap().key(), "set_freq");
//! ```

use std::collections::VecDeque;

/// A message with key-value content.
#[derive(Debug, Clone, PartialEq)]
pub struct Message {
    key: String,
    value: MessageValue,
    timestamp: u64,
}

/// Message value types.
#[derive(Debug, Clone, PartialEq)]
pub enum MessageValue {
    /// No value (signal/trigger).
    None,
    /// String value.
    String(String),
    /// Integer value.
    Int(i64),
    /// Float value.
    Float(f64),
    /// Boolean value.
    Bool(bool),
    /// Binary data.
    Bytes(Vec<u8>),
    /// Key-value pairs (dict).
    Dict(Vec<(String, MessageValue)>),
}

impl Message {
    /// Create a command message (key + string value).
    pub fn command(key: &str, value: &str) -> Self {
        Self {
            key: key.to_string(),
            value: MessageValue::String(value.to_string()),
            timestamp: 0,
        }
    }

    /// Create a signal message (key only, no value).
    pub fn signal(key: &str) -> Self {
        Self {
            key: key.to_string(),
            value: MessageValue::None,
            timestamp: 0,
        }
    }

    /// Create a message with float value.
    pub fn float(key: &str, value: f64) -> Self {
        Self {
            key: key.to_string(),
            value: MessageValue::Float(value),
            timestamp: 0,
        }
    }

    /// Create a message with integer value.
    pub fn int(key: &str, value: i64) -> Self {
        Self {
            key: key.to_string(),
            value: MessageValue::Int(value),
            timestamp: 0,
        }
    }

    /// Create with timestamp.
    pub fn with_timestamp(mut self, ts: u64) -> Self {
        self.timestamp = ts;
        self
    }

    /// Get message key.
    pub fn key(&self) -> &str {
        &self.key
    }

    /// Get message value.
    pub fn value(&self) -> &MessageValue {
        &self.value
    }

    /// Get as string if applicable.
    pub fn as_str(&self) -> Option<&str> {
        match &self.value {
            MessageValue::String(s) => Some(s),
            _ => None,
        }
    }

    /// Get as float if applicable.
    pub fn as_float(&self) -> Option<f64> {
        match &self.value {
            MessageValue::Float(f) => Some(*f),
            MessageValue::Int(i) => Some(*i as f64),
            _ => None,
        }
    }

    /// Get as int if applicable.
    pub fn as_int(&self) -> Option<i64> {
        match &self.value {
            MessageValue::Int(i) => Some(*i),
            _ => None,
        }
    }

    /// Get timestamp.
    pub fn timestamp(&self) -> u64 {
        self.timestamp
    }
}

/// Message port for async message passing.
#[derive(Debug, Clone)]
pub struct MessagePort {
    name: String,
    queue: VecDeque<Message>,
    max_queue: usize,
    messages_sent: u64,
    messages_dropped: u64,
}

impl MessagePort {
    /// Create a new message port.
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            queue: VecDeque::new(),
            max_queue: 1024,
            messages_sent: 0,
            messages_dropped: 0,
        }
    }

    /// Create with custom queue capacity.
    pub fn with_capacity(name: &str, max_queue: usize) -> Self {
        Self {
            name: name.to_string(),
            queue: VecDeque::new(),
            max_queue,
            messages_sent: 0,
            messages_dropped: 0,
        }
    }

    /// Send a message.
    pub fn send(&mut self, msg: Message) {
        if self.queue.len() >= self.max_queue {
            self.queue.pop_front();
            self.messages_dropped += 1;
        }
        self.queue.push_back(msg);
        self.messages_sent += 1;
    }

    /// Receive a message (FIFO).
    pub fn recv(&mut self) -> Option<Message> {
        self.queue.pop_front()
    }

    /// Peek at the next message without removing it.
    pub fn peek(&self) -> Option<&Message> {
        self.queue.front()
    }

    /// Get number of pending messages.
    pub fn pending(&self) -> usize {
        self.queue.len()
    }

    /// Check if there are pending messages.
    pub fn has_messages(&self) -> bool {
        !self.queue.is_empty()
    }

    /// Drain all pending messages.
    pub fn drain(&mut self) -> Vec<Message> {
        self.queue.drain(..).collect()
    }

    /// Get port name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get total messages sent.
    pub fn messages_sent(&self) -> u64 {
        self.messages_sent
    }

    /// Get messages dropped due to queue overflow.
    pub fn messages_dropped(&self) -> u64 {
        self.messages_dropped
    }

    /// Clear the queue.
    pub fn clear(&mut self) {
        self.queue.clear();
    }
}

/// Message bus connecting multiple ports.
#[derive(Debug, Clone)]
pub struct MessageBus {
    ports: Vec<(String, String)>, // (from_port, to_port)
    pending: VecDeque<(String, Message)>, // (destination, message)
}

impl MessageBus {
    /// Create a new message bus.
    pub fn new() -> Self {
        Self {
            ports: Vec::new(),
            pending: VecDeque::new(),
        }
    }

    /// Connect two ports (from → to).
    pub fn connect(&mut self, from: &str, to: &str) {
        self.ports.push((from.to_string(), to.to_string()));
    }

    /// Post a message from a named port.
    pub fn post(&mut self, from: &str, msg: Message) {
        for (src, dst) in &self.ports {
            if src == from {
                self.pending.push_back((dst.clone(), msg.clone()));
            }
        }
    }

    /// Collect messages for a named port.
    pub fn collect(&mut self, port_name: &str) -> Vec<Message> {
        let mut result = Vec::new();
        let mut remaining = VecDeque::new();
        for (dst, msg) in self.pending.drain(..) {
            if dst == port_name {
                result.push(msg);
            } else {
                remaining.push_back((dst, msg));
            }
        }
        self.pending = remaining;
        result
    }

    /// Get number of pending messages.
    pub fn pending_count(&self) -> usize {
        self.pending.len()
    }

    /// Get number of connections.
    pub fn num_connections(&self) -> usize {
        self.ports.len()
    }
}

impl Default for MessageBus {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_send_recv() {
        let mut port = MessagePort::new("test");
        port.send(Message::command("freq", "1e9"));
        assert_eq!(port.pending(), 1);
        let msg = port.recv().unwrap();
        assert_eq!(msg.key(), "freq");
        assert_eq!(msg.as_str(), Some("1e9"));
    }

    #[test]
    fn test_signal_message() {
        let msg = Message::signal("trigger");
        assert_eq!(msg.key(), "trigger");
        assert_eq!(*msg.value(), MessageValue::None);
    }

    #[test]
    fn test_float_message() {
        let msg = Message::float("gain", 3.14);
        assert_eq!(msg.as_float(), Some(3.14));
    }

    #[test]
    fn test_int_message() {
        let msg = Message::int("count", 42);
        assert_eq!(msg.as_int(), Some(42));
        // Int can also be read as float.
        assert_eq!(msg.as_float(), Some(42.0));
    }

    #[test]
    fn test_queue_overflow() {
        let mut port = MessagePort::with_capacity("small", 2);
        port.send(Message::signal("a"));
        port.send(Message::signal("b"));
        port.send(Message::signal("c")); // Drops "a".
        assert_eq!(port.pending(), 2);
        assert_eq!(port.messages_dropped(), 1);
        assert_eq!(port.recv().unwrap().key(), "b");
    }

    #[test]
    fn test_drain() {
        let mut port = MessagePort::new("d");
        port.send(Message::signal("x"));
        port.send(Message::signal("y"));
        let msgs = port.drain();
        assert_eq!(msgs.len(), 2);
        assert_eq!(port.pending(), 0);
    }

    #[test]
    fn test_peek() {
        let mut port = MessagePort::new("p");
        port.send(Message::signal("first"));
        assert_eq!(port.peek().unwrap().key(), "first");
        assert_eq!(port.pending(), 1); // Not consumed.
    }

    #[test]
    fn test_message_bus() {
        let mut bus = MessageBus::new();
        bus.connect("tx_ctrl", "modulator_in");
        bus.connect("tx_ctrl", "amplifier_in");
        bus.post("tx_ctrl", Message::float("gain", 2.0));
        assert_eq!(bus.pending_count(), 2);
        let mod_msgs = bus.collect("modulator_in");
        assert_eq!(mod_msgs.len(), 1);
        let amp_msgs = bus.collect("amplifier_in");
        assert_eq!(amp_msgs.len(), 1);
    }

    #[test]
    fn test_message_bus_no_connection() {
        let mut bus = MessageBus::new();
        bus.post("orphan", Message::signal("lost"));
        assert_eq!(bus.pending_count(), 0); // No connections → no pending.
    }

    #[test]
    fn test_timestamp() {
        let msg = Message::float("freq", 1e9).with_timestamp(12345);
        assert_eq!(msg.timestamp(), 12345);
    }
}

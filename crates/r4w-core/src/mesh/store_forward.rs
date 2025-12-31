//! Store and Forward for Offline Nodes
//!
//! This module implements the Meshtastic store-and-forward protocol, allowing
//! messages to be cached when destination nodes are offline and delivered when
//! they come back online.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                    Store & Forward Server                        │
//! │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
//! │  │  MessageDB  │  │  Heartbeat  │  │   Request   │              │
//! │  │   Storage   │  │   Tracker   │  │   Handler   │              │
//! │  └─────────────┘  └─────────────┘  └─────────────┘              │
//! └─────────────────────────────────────────────────────────────────┘
//!                              │
//!                              ▼
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                      Client Nodes                                │
//! │  ┌─────────────┐                    ┌─────────────┐             │
//! │  │   Online    │ ◄────messages────► │   Offline   │             │
//! │  │   Node A    │                    │   Node B    │             │
//! │  └─────────────┘                    └─────────────┘             │
//! └─────────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Features
//!
//! - Message storage for offline destinations
//! - Heartbeat tracking to detect online/offline status
//! - Automatic message delivery when nodes come online
//! - Configurable storage limits and TTL
//! - Request-based retrieval protocol

use super::packet::{NodeId, PacketType};
use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};

/// Store and Forward configuration
#[derive(Debug, Clone)]
pub struct StoreForwardConfig {
    /// Maximum number of messages to store per destination
    pub max_messages_per_dest: usize,
    /// Maximum total messages in storage
    pub max_total_messages: usize,
    /// Message time-to-live
    pub message_ttl: Duration,
    /// Heartbeat timeout (node considered offline after this)
    pub heartbeat_timeout: Duration,
    /// Whether this node is a store-and-forward server
    pub is_server: bool,
    /// Maximum message size to store (bytes)
    pub max_message_size: usize,
}

impl Default for StoreForwardConfig {
    fn default() -> Self {
        Self {
            max_messages_per_dest: 100,
            max_total_messages: 1000,
            message_ttl: Duration::from_secs(7 * 24 * 3600), // 7 days
            heartbeat_timeout: Duration::from_secs(2 * 3600), // 2 hours
            is_server: false,
            max_message_size: 256,
        }
    }
}

/// A stored message waiting for delivery
#[derive(Debug, Clone)]
pub struct StoredMessage {
    /// Message ID for deduplication
    pub message_id: u32,
    /// Source node
    pub source: NodeId,
    /// Destination node
    pub destination: NodeId,
    /// Message payload
    pub payload: Vec<u8>,
    /// Original packet type
    pub packet_type: PacketType,
    /// When the message was stored
    pub stored_at: Instant,
    /// Original timestamp from sender (if any)
    pub original_timestamp: Option<u64>,
    /// Number of delivery attempts
    pub delivery_attempts: u32,
}

impl StoredMessage {
    /// Create a new stored message
    pub fn new(
        message_id: u32,
        source: NodeId,
        destination: NodeId,
        payload: Vec<u8>,
        packet_type: PacketType,
    ) -> Self {
        Self {
            message_id,
            source,
            destination,
            payload,
            packet_type,
            stored_at: Instant::now(),
            original_timestamp: None,
            delivery_attempts: 0,
        }
    }

    /// Check if message has expired
    pub fn is_expired(&self, ttl: Duration) -> bool {
        self.stored_at.elapsed() > ttl
    }

    /// Get message age
    pub fn age(&self) -> Duration {
        self.stored_at.elapsed()
    }
}

/// Node status tracking
#[derive(Debug, Clone)]
pub struct NodeStatus {
    /// Node ID
    pub node_id: NodeId,
    /// Last heard from this node
    pub last_seen: Instant,
    /// Is node currently online
    pub is_online: bool,
    /// Last known RSSI
    pub last_rssi: f32,
    /// Number of stored messages for this node
    pub pending_messages: usize,
}

impl NodeStatus {
    pub fn new(node_id: NodeId) -> Self {
        Self {
            node_id,
            last_seen: Instant::now(),
            is_online: true,
            last_rssi: 0.0,
            pending_messages: 0,
        }
    }

    /// Update last seen timestamp
    pub fn heartbeat(&mut self, rssi: f32) {
        self.last_seen = Instant::now();
        self.is_online = true;
        self.last_rssi = rssi;
    }

    /// Check if node is considered offline
    pub fn check_timeout(&mut self, timeout: Duration) -> bool {
        if self.last_seen.elapsed() > timeout {
            self.is_online = false;
            true
        } else {
            false
        }
    }
}

/// Store and Forward request types (Meshtastic protocol)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StoreForwardRequestType {
    /// Router heartbeat
    RouterHeartbeat = 0,
    /// Client heartbeat
    ClientHeartbeat = 1,
    /// Client history request
    HistoryRequest = 2,
    /// Router text direct message
    RouterTextDirect = 3,
    /// Router text broadcast
    RouterTextBroadcast = 4,
    /// Router ping message
    RouterPing = 5,
    /// Unknown request type
    Unknown = 255,
}

impl From<u8> for StoreForwardRequestType {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::RouterHeartbeat,
            1 => Self::ClientHeartbeat,
            2 => Self::HistoryRequest,
            3 => Self::RouterTextDirect,
            4 => Self::RouterTextBroadcast,
            5 => Self::RouterPing,
            _ => Self::Unknown,
        }
    }
}

/// Store and Forward statistics
#[derive(Debug, Clone, Default)]
pub struct StoreForwardStats {
    /// Total messages stored
    pub messages_stored: u64,
    /// Total messages delivered
    pub messages_delivered: u64,
    /// Total messages expired
    pub messages_expired: u64,
    /// Total messages dropped (limit reached)
    pub messages_dropped: u64,
    /// Current pending messages
    pub pending_count: usize,
    /// Unique destinations tracked
    pub destinations_tracked: usize,
    /// Nodes currently online
    pub nodes_online: usize,
    /// Nodes currently offline
    pub nodes_offline: usize,
}

/// Store and Forward server/client
pub struct StoreForward {
    config: StoreForwardConfig,
    /// Messages waiting for delivery, keyed by destination
    pending_messages: HashMap<NodeId, VecDeque<StoredMessage>>,
    /// Node status tracking
    node_status: HashMap<NodeId, NodeStatus>,
    /// Statistics
    stats: StoreForwardStats,
    /// Our node ID
    our_node_id: NodeId,
    /// Message ID counter for deduplication
    next_message_id: u32,
    /// Recently delivered message IDs (for dedup)
    delivered_ids: VecDeque<u32>,
}

impl StoreForward {
    /// Create a new store-and-forward instance
    pub fn new(our_node_id: NodeId, config: StoreForwardConfig) -> Self {
        Self {
            config,
            pending_messages: HashMap::new(),
            node_status: HashMap::new(),
            stats: StoreForwardStats::default(),
            our_node_id,
            next_message_id: 0,
            delivered_ids: VecDeque::with_capacity(1000),
        }
    }

    /// Create with default configuration
    pub fn new_server(our_node_id: NodeId) -> Self {
        let mut config = StoreForwardConfig::default();
        config.is_server = true;
        Self::new(our_node_id, config)
    }

    /// Create a client (non-server) instance
    pub fn new_client(our_node_id: NodeId) -> Self {
        Self::new(our_node_id, StoreForwardConfig::default())
    }

    /// Check if this instance is a server
    pub fn is_server(&self) -> bool {
        self.config.is_server
    }

    /// Get current statistics
    pub fn stats(&self) -> &StoreForwardStats {
        &self.stats
    }

    /// Store a message for later delivery
    pub fn store_message(
        &mut self,
        source: NodeId,
        destination: NodeId,
        payload: &[u8],
        packet_type: PacketType,
    ) -> Result<u32, StoreForwardError> {
        // Check if we're a server
        if !self.config.is_server {
            return Err(StoreForwardError::NotAServer);
        }

        // Check message size
        if payload.len() > self.config.max_message_size {
            return Err(StoreForwardError::MessageTooLarge);
        }

        // Check total storage limit
        let total = self.total_pending();
        if total >= self.config.max_total_messages {
            self.stats.messages_dropped += 1;
            return Err(StoreForwardError::StorageFull);
        }

        // Get or create destination queue
        let queue = self.pending_messages
            .entry(destination)
            .or_insert_with(VecDeque::new);

        // Check per-destination limit
        if queue.len() >= self.config.max_messages_per_dest {
            // Remove oldest message
            if queue.pop_front().is_some() {
                self.stats.messages_expired += 1;
            }
        }

        // Create and store message
        let message_id = self.next_message_id;
        self.next_message_id = self.next_message_id.wrapping_add(1);

        let message = StoredMessage::new(
            message_id,
            source,
            destination,
            payload.to_vec(),
            packet_type,
        );

        queue.push_back(message);
        self.stats.messages_stored += 1;
        self.update_stats();

        Ok(message_id)
    }

    /// Record a heartbeat from a node
    pub fn heartbeat(&mut self, node_id: NodeId, rssi: f32) {
        let status = self.node_status
            .entry(node_id)
            .or_insert_with(|| NodeStatus::new(node_id));

        let was_offline = !status.is_online;
        status.heartbeat(rssi);

        // If node came back online, check for pending messages
        if was_offline {
            self.update_stats();
        }
    }

    /// Get messages pending for a specific node
    pub fn get_pending_for(&self, destination: NodeId) -> Vec<&StoredMessage> {
        self.pending_messages
            .get(&destination)
            .map(|q| q.iter().collect())
            .unwrap_or_default()
    }

    /// Get number of pending messages for a node
    pub fn pending_count_for(&self, destination: NodeId) -> usize {
        self.pending_messages
            .get(&destination)
            .map(|q| q.len())
            .unwrap_or(0)
    }

    /// Retrieve messages for a node that has come online
    pub fn retrieve_messages(&mut self, destination: NodeId) -> Vec<StoredMessage> {
        // Record delivery in delivered_ids for dedup
        if let Some(queue) = self.pending_messages.remove(&destination) {
            let messages: Vec<_> = queue.into_iter().collect();

            for msg in &messages {
                // Track delivered IDs
                if self.delivered_ids.len() >= 1000 {
                    self.delivered_ids.pop_front();
                }
                self.delivered_ids.push_back(msg.message_id);
                self.stats.messages_delivered += 1;
            }

            self.update_stats();
            messages
        } else {
            Vec::new()
        }
    }

    /// Retrieve a limited number of messages (for history requests)
    pub fn retrieve_history(
        &mut self,
        destination: NodeId,
        max_messages: usize,
        since_timestamp: Option<u64>,
    ) -> Vec<StoredMessage> {
        if let Some(queue) = self.pending_messages.get_mut(&destination) {
            let mut result = Vec::new();
            let mut remaining = VecDeque::new();

            while let Some(msg) = queue.pop_front() {
                if result.len() < max_messages {
                    // Check timestamp filter
                    let include = match (since_timestamp, msg.original_timestamp) {
                        (Some(since), Some(ts)) => ts > since,
                        _ => true,
                    };

                    if include {
                        if self.delivered_ids.len() >= 1000 {
                            self.delivered_ids.pop_front();
                        }
                        self.delivered_ids.push_back(msg.message_id);
                        self.stats.messages_delivered += 1;
                        result.push(msg);
                    } else {
                        remaining.push_back(msg);
                    }
                } else {
                    remaining.push_back(msg);
                }
            }

            *queue = remaining;
            self.update_stats();
            result
        } else {
            Vec::new()
        }
    }

    /// Check if a message was already delivered (deduplication)
    pub fn was_delivered(&self, message_id: u32) -> bool {
        self.delivered_ids.contains(&message_id)
    }

    /// Perform maintenance: expire old messages, check timeouts
    pub fn maintenance(&mut self) {
        let ttl = self.config.message_ttl;
        let timeout = self.config.heartbeat_timeout;

        // Expire old messages
        for queue in self.pending_messages.values_mut() {
            let before = queue.len();
            queue.retain(|msg| !msg.is_expired(ttl));
            self.stats.messages_expired += (before - queue.len()) as u64;
        }

        // Remove empty queues
        self.pending_messages.retain(|_, q| !q.is_empty());

        // Check node timeouts
        for status in self.node_status.values_mut() {
            status.check_timeout(timeout);
        }

        self.update_stats();
    }

    /// Get total pending messages
    fn total_pending(&self) -> usize {
        self.pending_messages.values().map(|q| q.len()).sum()
    }

    /// Update statistics
    fn update_stats(&mut self) {
        self.stats.pending_count = self.total_pending();
        self.stats.destinations_tracked = self.pending_messages.len();
        self.stats.nodes_online = self.node_status.values().filter(|s| s.is_online).count();
        self.stats.nodes_offline = self.node_status.values().filter(|s| !s.is_online).count();
    }

    /// Check if a destination is online
    pub fn is_online(&self, node_id: NodeId) -> bool {
        self.node_status
            .get(&node_id)
            .map(|s| s.is_online)
            .unwrap_or(false)
    }

    /// Get all pending destinations
    pub fn pending_destinations(&self) -> Vec<NodeId> {
        self.pending_messages.keys().copied().collect()
    }

    /// Get node status
    pub fn node_status(&self, node_id: NodeId) -> Option<&NodeStatus> {
        self.node_status.get(&node_id)
    }

    /// Get all node statuses
    pub fn all_node_status(&self) -> Vec<&NodeStatus> {
        self.node_status.values().collect()
    }

    /// Clear all stored messages
    pub fn clear(&mut self) {
        self.pending_messages.clear();
        self.stats.pending_count = 0;
        self.stats.destinations_tracked = 0;
    }
}

/// Store and Forward errors
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StoreForwardError {
    /// This node is not configured as a server
    NotAServer,
    /// Message exceeds maximum size
    MessageTooLarge,
    /// Storage is full
    StorageFull,
    /// Node not found
    NodeNotFound,
    /// Message already delivered
    AlreadyDelivered,
}

impl std::fmt::Display for StoreForwardError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotAServer => write!(f, "Not configured as store-and-forward server"),
            Self::MessageTooLarge => write!(f, "Message exceeds maximum size"),
            Self::StorageFull => write!(f, "Message storage is full"),
            Self::NodeNotFound => write!(f, "Node not found"),
            Self::AlreadyDelivered => write!(f, "Message already delivered"),
        }
    }
}

impl std::error::Error for StoreForwardError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_node_id(n: u32) -> NodeId {
        NodeId::from_u32(n)
    }

    #[test]
    fn test_store_message() {
        let server_id = test_node_id(1);
        let mut sf = StoreForward::new_server(server_id);

        let source = test_node_id(2);
        let dest = test_node_id(3);
        let payload = b"Hello";

        let result = sf.store_message(source, dest, payload, PacketType::Text);
        assert!(result.is_ok());

        assert_eq!(sf.pending_count_for(dest), 1);
        assert_eq!(sf.stats().messages_stored, 1);
    }

    #[test]
    fn test_not_server() {
        let client_id = test_node_id(1);
        let mut sf = StoreForward::new_client(client_id);

        let result = sf.store_message(
            test_node_id(2),
            test_node_id(3),
            b"Hello",
            PacketType::Text,
        );
        assert_eq!(result, Err(StoreForwardError::NotAServer));
    }

    #[test]
    fn test_retrieve_messages() {
        let server_id = test_node_id(1);
        let mut sf = StoreForward::new_server(server_id);

        let source = test_node_id(2);
        let dest = test_node_id(3);

        // Store 3 messages
        for i in 0..3 {
            let msg = format!("Message {}", i);
            sf.store_message(source, dest, msg.as_bytes(), PacketType::Text).unwrap();
        }

        assert_eq!(sf.pending_count_for(dest), 3);

        // Retrieve all
        let messages = sf.retrieve_messages(dest);
        assert_eq!(messages.len(), 3);
        assert_eq!(sf.pending_count_for(dest), 0);
        assert_eq!(sf.stats().messages_delivered, 3);
    }

    #[test]
    fn test_heartbeat_tracking() {
        let server_id = test_node_id(1);
        let mut sf = StoreForward::new_server(server_id);

        let node = test_node_id(2);
        sf.heartbeat(node, -80.0);

        assert!(sf.is_online(node));

        let status = sf.node_status(node).unwrap();
        assert!(status.is_online);
        assert_eq!(status.last_rssi, -80.0);
    }

    #[test]
    fn test_message_size_limit() {
        let server_id = test_node_id(1);
        let mut config = StoreForwardConfig::default();
        config.is_server = true;
        config.max_message_size = 10;
        let mut sf = StoreForward::new(server_id, config);

        let large_payload = vec![0u8; 100];
        let result = sf.store_message(
            test_node_id(2),
            test_node_id(3),
            &large_payload,
            PacketType::Text,
        );
        assert_eq!(result, Err(StoreForwardError::MessageTooLarge));
    }

    #[test]
    fn test_per_destination_limit() {
        let server_id = test_node_id(1);
        let mut config = StoreForwardConfig::default();
        config.is_server = true;
        config.max_messages_per_dest = 3;
        let mut sf = StoreForward::new(server_id, config);

        let dest = test_node_id(3);

        // Store 5 messages (exceeds limit of 3)
        for i in 0..5 {
            let msg = format!("Message {}", i);
            sf.store_message(test_node_id(2), dest, msg.as_bytes(), PacketType::Text).unwrap();
        }

        // Should only have 3 (newest ones)
        assert_eq!(sf.pending_count_for(dest), 3);

        // 2 should have been expired (removed to make room)
        assert_eq!(sf.stats().messages_expired, 2);
    }

    #[test]
    fn test_deduplication() {
        let server_id = test_node_id(1);
        let mut sf = StoreForward::new_server(server_id);

        let dest = test_node_id(3);
        let msg_id = sf.store_message(
            test_node_id(2),
            dest,
            b"Hello",
            PacketType::Text,
        ).unwrap();

        // Before delivery
        assert!(!sf.was_delivered(msg_id));

        // Retrieve
        sf.retrieve_messages(dest);

        // After delivery
        assert!(sf.was_delivered(msg_id));
    }

    #[test]
    fn test_retrieve_history() {
        let server_id = test_node_id(1);
        let mut sf = StoreForward::new_server(server_id);

        let dest = test_node_id(3);

        // Store 10 messages
        for i in 0..10 {
            let msg = format!("Message {}", i);
            sf.store_message(test_node_id(2), dest, msg.as_bytes(), PacketType::Text).unwrap();
        }

        // Retrieve only 3
        let messages = sf.retrieve_history(dest, 3, None);
        assert_eq!(messages.len(), 3);

        // 7 should remain
        assert_eq!(sf.pending_count_for(dest), 7);
    }
}

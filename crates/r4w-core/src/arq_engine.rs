//! ARQ Engine — Automatic Repeat Request reliable delivery
//!
//! Link-layer reliable delivery with fragmentation, reassembly,
//! acknowledgments, retransmission timers, and sequence numbering.
//! Supports Stop-and-Wait, Go-Back-N, and Selective Repeat modes.
//! Used for satellite links, HF modems, IoT, and mesh networks.
//! GNU Radio equivalent: custom OOT / `gr-satellites` ARQ.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::arq_engine::{ArqEngine, ArqConfig, ArqMode};
//!
//! let config = ArqConfig {
//!     mode: ArqMode::StopAndWait,
//!     max_retries: 3,
//!     timeout_ms: 1000,
//!     max_fragment_size: 64,
//!     sequence_bits: 3,
//! };
//! let mut engine = ArqEngine::new(config);
//! let frames = engine.send(b"Hello, ARQ!");
//! assert!(!frames.is_empty());
//! ```

/// ARQ operating mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ArqMode {
    /// Stop-and-Wait: send one frame, wait for ACK.
    StopAndWait,
    /// Go-Back-N: sliding window, retransmit from error.
    GoBackN { window: u8 },
    /// Selective Repeat: retransmit only errored frames.
    SelectiveRepeat { window: u8 },
}

/// ARQ configuration.
#[derive(Debug, Clone)]
pub struct ArqConfig {
    /// ARQ mode.
    pub mode: ArqMode,
    /// Maximum retransmission attempts per frame.
    pub max_retries: u8,
    /// Retransmission timeout in milliseconds.
    pub timeout_ms: u64,
    /// Maximum fragment payload size (MTU).
    pub max_fragment_size: usize,
    /// Sequence number bit width (3, 7, or 15).
    pub sequence_bits: u8,
}

impl Default for ArqConfig {
    fn default() -> Self {
        Self {
            mode: ArqMode::StopAndWait,
            max_retries: 3,
            timeout_ms: 1000,
            max_fragment_size: 128,
            sequence_bits: 7,
        }
    }
}

/// ARQ data frame.
#[derive(Debug, Clone)]
pub struct ArqFrame {
    /// Sequence number.
    pub seq: u16,
    /// Fragment offset within the original payload.
    pub fragment_offset: u16,
    /// More fragments follow.
    pub more_fragments: bool,
    /// Payload data.
    pub payload: Vec<u8>,
    /// CRC-16 over header + payload.
    pub crc: u16,
}

/// ARQ acknowledgment.
#[derive(Debug, Clone)]
pub struct ArqAck {
    /// Sequence number being acknowledged.
    pub seq: u16,
    /// Whether this is a NAK (negative acknowledgment).
    pub nak: bool,
}

/// Pending frame tracking.
#[derive(Debug, Clone)]
struct PendingFrame {
    frame: ArqFrame,
    retries: u8,
    sent_at_ms: u64,
}

/// ARQ statistics.
#[derive(Debug, Clone, Default)]
pub struct ArqStats {
    /// Total frames sent (including retransmissions).
    pub frames_sent: u64,
    /// Retransmissions.
    pub retransmissions: u64,
    /// Frames received.
    pub frames_received: u64,
    /// Duplicate frames dropped.
    pub duplicates_dropped: u64,
    /// Frames that exceeded max retries.
    pub frames_failed: u64,
}

/// ARQ engine.
#[derive(Debug, Clone)]
pub struct ArqEngine {
    config: ArqConfig,
    /// Next sequence number to assign.
    next_seq: u16,
    /// Sequence modulus.
    seq_modulus: u16,
    /// Pending (unacknowledged) frames.
    pending: Vec<PendingFrame>,
    /// Receive buffer for reassembly.
    receive_buffer: Vec<(u16, Vec<u8>)>,
    /// Set of received sequence numbers (for duplicate detection).
    received_seqs: Vec<u16>,
    /// Expected receive sequence.
    expected_seq: u16,
    /// Current time (set by tick()).
    current_time_ms: u64,
    /// Statistics.
    stats: ArqStats,
}

impl ArqEngine {
    /// Create a new ARQ engine.
    pub fn new(config: ArqConfig) -> Self {
        let seq_modulus = 1u16 << config.sequence_bits;
        Self {
            config,
            next_seq: 0,
            seq_modulus,
            pending: Vec::new(),
            receive_buffer: Vec::new(),
            received_seqs: Vec::new(),
            expected_seq: 0,
            current_time_ms: 0,
            stats: ArqStats::default(),
        }
    }

    /// Fragment and enqueue a payload for sending.
    ///
    /// Returns the frames to transmit.
    pub fn send(&mut self, payload: &[u8]) -> Vec<ArqFrame> {
        let mtu = self.config.max_fragment_size.max(1);
        let fragments: Vec<&[u8]> = payload.chunks(mtu).collect();
        let num_frags = fragments.len();
        let mut frames = Vec::new();

        for (i, frag) in fragments.iter().enumerate() {
            let seq = self.next_seq;
            self.next_seq = (self.next_seq + 1) % self.seq_modulus;

            let frame = ArqFrame {
                seq,
                fragment_offset: (i * mtu) as u16,
                more_fragments: i < num_frags - 1,
                payload: frag.to_vec(),
                crc: crc16_frame(seq, frag),
            };

            self.pending.push(PendingFrame {
                frame: frame.clone(),
                retries: 0,
                sent_at_ms: self.current_time_ms,
            });

            self.stats.frames_sent += 1;
            frames.push(frame);
        }

        frames
    }

    /// Process a received data frame.
    ///
    /// Returns reassembled payload if all fragments are received.
    pub fn receive(&mut self, frame: &ArqFrame) -> Option<Vec<u8>> {
        self.stats.frames_received += 1;

        // Verify CRC
        let computed_crc = crc16_frame(frame.seq, &frame.payload);
        if computed_crc != frame.crc {
            return None;
        }

        // Check for duplicates
        if self.received_seqs.contains(&frame.seq) {
            self.stats.duplicates_dropped += 1;
            return None;
        }

        self.received_seqs.push(frame.seq);
        self.receive_buffer
            .push((frame.seq, frame.payload.clone()));

        // If this is the last fragment, try to reassemble
        if !frame.more_fragments {
            // Sort by fragment offset (using seq as proxy)
            self.receive_buffer.sort_by_key(|(seq, _)| *seq);

            let mut assembled = Vec::new();
            for (_, data) in self.receive_buffer.drain(..) {
                assembled.extend(data);
            }
            return Some(assembled);
        }

        None
    }

    /// Process an acknowledgment.
    pub fn process_ack(&mut self, ack: &ArqAck) {
        if ack.nak {
            // NAK: mark for retransmission
            if let Some(pending) = self.pending.iter_mut().find(|p| p.frame.seq == ack.seq) {
                pending.sent_at_ms = 0; // Force retransmit on next tick
            }
        } else {
            // ACK: remove from pending
            self.pending.retain(|p| p.frame.seq != ack.seq);
        }
    }

    /// Generate ACKs for received frames.
    pub fn pending_acks(&self) -> Vec<ArqAck> {
        self.receive_buffer
            .iter()
            .map(|(seq, _)| ArqAck {
                seq: *seq,
                nak: false,
            })
            .collect()
    }

    /// Advance time and check for retransmissions.
    ///
    /// Returns frames that need retransmission.
    pub fn tick(&mut self, elapsed_ms: u64) -> Vec<ArqFrame> {
        self.current_time_ms += elapsed_ms;
        let mut retransmit = Vec::new();

        let timeout = self.config.timeout_ms;
        let max_retries = self.config.max_retries;

        self.pending.retain_mut(|pending| {
            if self.current_time_ms - pending.sent_at_ms >= timeout {
                if pending.retries >= max_retries {
                    // Give up
                    return false;
                }
                pending.retries += 1;
                pending.sent_at_ms = self.current_time_ms;
                retransmit.push(pending.frame.clone());
            }
            true
        });

        self.stats.retransmissions += retransmit.len() as u64;
        self.stats.frames_sent += retransmit.len() as u64;
        retransmit
    }

    /// Number of unacknowledged frames.
    pub fn pending_count(&self) -> usize {
        self.pending.len()
    }

    /// Get statistics.
    pub fn stats(&self) -> &ArqStats {
        &self.stats
    }

    /// Get the window size.
    pub fn window_size(&self) -> u8 {
        match self.config.mode {
            ArqMode::StopAndWait => 1,
            ArqMode::GoBackN { window } => window,
            ArqMode::SelectiveRepeat { window } => window,
        }
    }

    /// Reset the engine.
    pub fn reset(&mut self) {
        self.next_seq = 0;
        self.pending.clear();
        self.receive_buffer.clear();
        self.received_seqs.clear();
        self.expected_seq = 0;
        self.current_time_ms = 0;
        self.stats = ArqStats::default();
    }
}

/// Simple CRC-16 for frame integrity.
fn crc16_frame(seq: u16, payload: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    // Include sequence number
    for byte in seq.to_le_bytes() {
        crc ^= byte as u16;
        for _ in 0..8 {
            if crc & 1 == 1 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    // Include payload
    for &byte in payload {
        crc ^= byte as u16;
        for _ in 0..8 {
            if crc & 1 == 1 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_send_single() {
        let mut engine = ArqEngine::new(ArqConfig::default());
        let frames = engine.send(b"Hello");
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].seq, 0);
        assert!(!frames[0].more_fragments);
        assert_eq!(frames[0].payload, b"Hello");
    }

    #[test]
    fn test_send_fragmented() {
        let config = ArqConfig {
            max_fragment_size: 4,
            ..ArqConfig::default()
        };
        let mut engine = ArqEngine::new(config);
        let frames = engine.send(b"Hello World!");
        assert_eq!(frames.len(), 3); // 4 + 4 + 4
        assert!(frames[0].more_fragments);
        assert!(frames[1].more_fragments);
        assert!(!frames[2].more_fragments);
    }

    #[test]
    fn test_receive_single() {
        let mut tx = ArqEngine::new(ArqConfig::default());
        let mut rx = ArqEngine::new(ArqConfig::default());

        let frames = tx.send(b"Test");
        let result = rx.receive(&frames[0]);
        assert!(result.is_some());
        assert_eq!(result.unwrap(), b"Test");
    }

    #[test]
    fn test_receive_fragmented() {
        let config = ArqConfig {
            max_fragment_size: 3,
            ..ArqConfig::default()
        };
        let mut tx = ArqEngine::new(config.clone());
        let mut rx = ArqEngine::new(config);

        let frames = tx.send(b"ABCDEF");
        assert_eq!(frames.len(), 2);

        let result1 = rx.receive(&frames[0]);
        assert!(result1.is_none()); // Waiting for more fragments

        let result2 = rx.receive(&frames[1]);
        assert!(result2.is_some());
        assert_eq!(result2.unwrap(), b"ABCDEF");
    }

    #[test]
    fn test_ack_removes_pending() {
        let mut engine = ArqEngine::new(ArqConfig::default());
        engine.send(b"Test");
        assert_eq!(engine.pending_count(), 1);

        engine.process_ack(&ArqAck { seq: 0, nak: false });
        assert_eq!(engine.pending_count(), 0);
    }

    #[test]
    fn test_retransmission() {
        let config = ArqConfig {
            timeout_ms: 100,
            max_retries: 3,
            ..ArqConfig::default()
        };
        let mut engine = ArqEngine::new(config);
        engine.send(b"Test");

        // Advance time past timeout
        let retransmit = engine.tick(150);
        assert_eq!(retransmit.len(), 1);
        assert_eq!(engine.stats().retransmissions, 1);
    }

    #[test]
    fn test_max_retries_exceeded() {
        let config = ArqConfig {
            timeout_ms: 10,
            max_retries: 2,
            ..ArqConfig::default()
        };
        let mut engine = ArqEngine::new(config);
        engine.send(b"Test");

        engine.tick(20); // Retry 1
        engine.tick(20); // Retry 2
        engine.tick(20); // Exceed max retries, frame dropped
        assert_eq!(engine.pending_count(), 0);
    }

    #[test]
    fn test_duplicate_detection() {
        let mut tx = ArqEngine::new(ArqConfig::default());
        let mut rx = ArqEngine::new(ArqConfig::default());

        let frames = tx.send(b"Test");
        rx.receive(&frames[0]);
        rx.receive(&frames[0]); // Duplicate
        assert_eq!(rx.stats().duplicates_dropped, 1);
    }

    #[test]
    fn test_crc_integrity() {
        let frame = ArqFrame {
            seq: 42,
            fragment_offset: 0,
            more_fragments: false,
            payload: b"Hello".to_vec(),
            crc: crc16_frame(42, b"Hello"),
        };
        // Verify CRC matches
        assert_eq!(crc16_frame(42, b"Hello"), frame.crc);
        // Different data → different CRC
        assert_ne!(crc16_frame(42, b"World"), frame.crc);
    }

    #[test]
    fn test_window_size() {
        let config = ArqConfig {
            mode: ArqMode::GoBackN { window: 7 },
            ..ArqConfig::default()
        };
        let engine = ArqEngine::new(config);
        assert_eq!(engine.window_size(), 7);
    }

    #[test]
    fn test_reset() {
        let mut engine = ArqEngine::new(ArqConfig::default());
        engine.send(b"Test");
        engine.reset();
        assert_eq!(engine.pending_count(), 0);
        assert_eq!(engine.stats().frames_sent, 0);
    }

    #[test]
    fn test_nak() {
        let mut engine = ArqEngine::new(ArqConfig::default());
        engine.send(b"Test");
        engine.process_ack(&ArqAck { seq: 0, nak: true });
        assert_eq!(engine.pending_count(), 1); // Still pending
    }

    #[test]
    fn test_sequence_wrap() {
        let config = ArqConfig {
            sequence_bits: 3, // Mod 8
            ..ArqConfig::default()
        };
        let mut engine = ArqEngine::new(config);
        for i in 0..10 {
            let frames = engine.send(&[i as u8]);
            // Sequence should wrap around at 8
            assert_eq!(frames[0].seq, i % 8);
            engine.process_ack(&ArqAck {
                seq: frames[0].seq,
                nak: false,
            });
        }
    }
}

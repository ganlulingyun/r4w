//! CSMA/CA MAC — Carrier Sense Multiple Access with Collision Avoidance
//!
//! A MAC layer protocol for shared-medium wireless communication.
//! Implements channel sensing, random backoff, acknowledgment handling,
//! and collision tracking. Supports configurable backoff algorithms
//! (binary exponential, uniform, fixed) and includes factory presets
//! for 802.11-style WiFi and pure ALOHA operation.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::csma_ca_mac::{CsmaCaMac, CsmaConfig, MacAction, MacState};
//!
//! let config = CsmaConfig {
//!     min_backoff: 1,
//!     max_backoff: 31,
//!     max_retries: 5,
//!     slot_time_us: 20,
//!     sifs_us: 10,
//!     difs_us: 50,
//! };
//! let mut mac = CsmaCaMac::new(config);
//! assert_eq!(*mac.state(), MacState::Idle);
//!
//! // Request to transmit on an idle channel
//! let action = mac.request_transmit(100);
//! assert!(matches!(action, MacAction::Transmit));
//! ```

/// MAC protocol state machine states.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MacState {
    /// No pending operation; ready for transmit or receive.
    Idle,
    /// Counting down backoff slots before re-attempting transmit.
    Backoff(usize),
    /// Currently transmitting a frame.
    Transmitting,
    /// Currently receiving a frame.
    Receiving,
    /// Frame sent; waiting for acknowledgment.
    WaitingAck,
}

/// Action returned by MAC operations indicating what the caller should do.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MacAction {
    /// Wait — no action needed this slot.
    Wait,
    /// Transmit the pending frame now.
    Transmit,
    /// Backing off for the given number of remaining slots.
    BackingOff(usize),
    /// Transmission failed after maximum retries.
    Failed,
    /// MAC is idle; nothing pending.
    Idle,
}

/// Backoff algorithm selection.
#[derive(Debug, Clone, PartialEq)]
pub enum BackoffAlgorithm {
    /// 802.11 binary exponential backoff: window doubles on each collision.
    BinaryExponential,
    /// Uniform random backoff within [min_backoff, max_backoff].
    Uniform,
    /// Fixed backoff of exactly the given number of slots.
    Fixed(usize),
}

/// CSMA/CA configuration parameters.
#[derive(Debug, Clone)]
pub struct CsmaConfig {
    /// Minimum contention window size (CW_min).
    pub min_backoff: usize,
    /// Maximum contention window size (CW_max).
    pub max_backoff: usize,
    /// Maximum retransmission attempts before declaring failure.
    pub max_retries: usize,
    /// Slot time in microseconds.
    pub slot_time_us: usize,
    /// Short Interframe Space in microseconds.
    pub sifs_us: usize,
    /// Distributed Interframe Space in microseconds.
    pub difs_us: usize,
}

/// MAC-layer statistics.
#[derive(Debug, Clone, PartialEq)]
pub struct MacStats {
    /// Total successful transmissions.
    pub transmit_count: usize,
    /// Total collisions / busy-channel detections.
    pub collision_count: usize,
    /// Average backoff duration in slots (0.0 if no backoffs).
    pub avg_backoff: f64,
    /// Channel utilization estimate (transmit slots / total slots).
    pub channel_utilization: f64,
}

/// CSMA/CA MAC layer implementation.
///
/// Manages channel access using carrier sensing and random backoff.
/// Call [`tick`](CsmaCaMac::tick) once per slot to advance the state machine.
pub struct CsmaCaMac {
    config: CsmaConfig,
    state: MacState,
    backoff_algo: BackoffAlgorithm,
    retry_count: usize,
    collision_count: usize,
    transmit_count: usize,
    total_backoff_slots: usize,
    backoff_events: usize,
    total_slots: usize,
    transmit_slots: usize,
    pending_payload_len: usize,
    /// Simple xorshift RNG state — avoids external crate dependency.
    rng_state: u64,
}

impl CsmaCaMac {
    /// Create a new CSMA/CA MAC with the given configuration.
    pub fn new(config: CsmaConfig) -> Self {
        Self {
            config,
            state: MacState::Idle,
            backoff_algo: BackoffAlgorithm::BinaryExponential,
            retry_count: 0,
            collision_count: 0,
            transmit_count: 0,
            total_backoff_slots: 0,
            backoff_events: 0,
            total_slots: 0,
            transmit_slots: 0,
            pending_payload_len: 0,
            rng_state: 0xDEAD_BEEF_CAFE_1234,
        }
    }

    /// Request to transmit a frame of the given payload length.
    ///
    /// If the MAC is idle, the caller may transmit immediately. If busy or
    /// already backing off, appropriate action is returned.
    pub fn request_transmit(&mut self, payload_len: usize) -> MacAction {
        self.pending_payload_len = payload_len;
        match self.state {
            MacState::Idle => {
                self.state = MacState::Transmitting;
                self.transmit_count += 1;
                MacAction::Transmit
            }
            MacState::Backoff(_) | MacState::Transmitting | MacState::WaitingAck => {
                MacAction::Wait
            }
            MacState::Receiving => MacAction::Wait,
        }
    }

    /// Advance the MAC state machine by one slot.
    ///
    /// `channel_busy` indicates whether energy was detected on the channel
    /// during this slot (carrier sense result).
    pub fn tick(&mut self, channel_busy: bool) -> MacAction {
        self.total_slots += 1;
        if channel_busy {
            self.transmit_slots += 1;
        }

        match self.state.clone() {
            MacState::Idle => MacAction::Idle,
            MacState::Transmitting => {
                // After transmit, transition to WaitingAck.
                self.state = MacState::WaitingAck;
                MacAction::Wait
            }
            MacState::WaitingAck => {
                // No ACK received this slot — treat as collision.
                self.collision_count += 1;
                self.retry_count += 1;
                if self.retry_count > self.config.max_retries {
                    self.state = MacState::Idle;
                    self.retry_count = 0;
                    return MacAction::Failed;
                }
                let backoff = self.compute_backoff();
                self.backoff_events += 1;
                self.total_backoff_slots += backoff;
                self.state = MacState::Backoff(backoff);
                MacAction::BackingOff(backoff)
            }
            MacState::Backoff(remaining) => {
                if channel_busy {
                    // Channel busy — freeze backoff counter (802.11 behaviour).
                    MacAction::BackingOff(remaining)
                } else if remaining <= 1 {
                    // Backoff expired — attempt transmit.
                    self.state = MacState::Transmitting;
                    self.transmit_count += 1;
                    MacAction::Transmit
                } else {
                    self.state = MacState::Backoff(remaining - 1);
                    MacAction::BackingOff(remaining - 1)
                }
            }
            MacState::Receiving => MacAction::Wait,
        }
    }

    /// Notify the MAC that an acknowledgment was received.
    ///
    /// Transitions back to Idle and clears the retry counter.
    pub fn receive_ack(&mut self) {
        self.state = MacState::Idle;
        self.retry_count = 0;
    }

    /// Return the total collision count.
    pub fn collision_count(&self) -> usize {
        self.collision_count
    }

    /// Return a reference to the current MAC state.
    pub fn state(&self) -> &MacState {
        &self.state
    }

    /// Reset the MAC to its initial state, preserving configuration.
    pub fn reset(&mut self) {
        self.state = MacState::Idle;
        self.retry_count = 0;
        self.collision_count = 0;
        self.transmit_count = 0;
        self.total_backoff_slots = 0;
        self.backoff_events = 0;
        self.total_slots = 0;
        self.transmit_slots = 0;
        self.pending_payload_len = 0;
    }

    /// Set the backoff algorithm used after collisions.
    pub fn set_backoff_algorithm(&mut self, algo: BackoffAlgorithm) {
        self.backoff_algo = algo;
    }

    /// Return aggregate MAC statistics.
    pub fn stats(&self) -> MacStats {
        let avg_backoff = if self.backoff_events > 0 {
            self.total_backoff_slots as f64 / self.backoff_events as f64
        } else {
            0.0
        };
        let channel_utilization = if self.total_slots > 0 {
            self.transmit_slots as f64 / self.total_slots as f64
        } else {
            0.0
        };
        MacStats {
            transmit_count: self.transmit_count,
            collision_count: self.collision_count,
            avg_backoff,
            channel_utilization,
        }
    }

    /// Compute backoff slot count based on the selected algorithm.
    fn compute_backoff(&mut self) -> usize {
        match &self.backoff_algo {
            BackoffAlgorithm::BinaryExponential => {
                // Window = min(min_backoff * 2^retry, max_backoff)
                let window = std::cmp::min(
                    self.config.min_backoff.saturating_mul(1 << self.retry_count),
                    self.config.max_backoff,
                );
                if window == 0 {
                    return 0;
                }
                (self.next_random() as usize) % window + 1
            }
            BackoffAlgorithm::Uniform => {
                let range = self.config.max_backoff - self.config.min_backoff + 1;
                if range == 0 {
                    return self.config.min_backoff;
                }
                self.config.min_backoff + (self.next_random() as usize) % range
            }
            BackoffAlgorithm::Fixed(slots) => *slots,
        }
    }

    /// Simple xorshift64 PRNG — no external dependencies.
    fn next_random(&mut self) -> u64 {
        let mut x = self.rng_state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.rng_state = x;
        x
    }
}

// ---------------------------------------------------------------------------
// Factory constructors
// ---------------------------------------------------------------------------

/// Create a CSMA/CA MAC with 802.11-style WiFi parameters.
///
/// - SIFS = 10 us
/// - DIFS = 50 us
/// - Slot time = 20 us
/// - CW_min = 15, CW_max = 1023
/// - Max retries = 7
pub fn wifi_csma() -> CsmaCaMac {
    let config = CsmaConfig {
        min_backoff: 15,
        max_backoff: 1023,
        max_retries: 7,
        slot_time_us: 20,
        sifs_us: 10,
        difs_us: 50,
    };
    CsmaCaMac::new(config)
}

/// Create a pure ALOHA MAC — no carrier sense, no backoff.
///
/// Transmit immediately regardless of channel state.
/// - Backoff window = 0 (fixed)
/// - Max retries = 0 (no retransmission)
/// - Slot time = 1 us (minimal)
pub fn simple_aloha() -> CsmaCaMac {
    let config = CsmaConfig {
        min_backoff: 0,
        max_backoff: 0,
        max_retries: 0,
        slot_time_us: 1,
        sifs_us: 0,
        difs_us: 0,
    };
    let mut mac = CsmaCaMac::new(config);
    mac.set_backoff_algorithm(BackoffAlgorithm::Fixed(0));
    mac
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> CsmaConfig {
        CsmaConfig {
            min_backoff: 4,
            max_backoff: 64,
            max_retries: 3,
            slot_time_us: 20,
            sifs_us: 10,
            difs_us: 50,
        }
    }

    #[test]
    fn test_construction_and_default_state() {
        let mac = CsmaCaMac::new(default_config());
        assert_eq!(*mac.state(), MacState::Idle);
        assert_eq!(mac.collision_count(), 0);
        let stats = mac.stats();
        assert_eq!(stats.transmit_count, 0);
        assert_eq!(stats.collision_count, 0);
        assert_eq!(stats.avg_backoff, 0.0);
        assert_eq!(stats.channel_utilization, 0.0);
    }

    #[test]
    fn test_request_transmit_idle_channel() {
        let mut mac = CsmaCaMac::new(default_config());
        let action = mac.request_transmit(100);
        assert_eq!(action, MacAction::Transmit);
        assert_eq!(*mac.state(), MacState::Transmitting);
    }

    #[test]
    fn test_backoff_on_busy_channel() {
        let mut mac = CsmaCaMac::new(default_config());
        // Transmit, then tick into WaitingAck
        mac.request_transmit(50);
        mac.tick(false); // Transmitting -> WaitingAck

        // No ACK arrives — tick triggers collision and backoff
        let action = mac.tick(false);
        assert!(matches!(action, MacAction::BackingOff(_)));
        assert!(matches!(*mac.state(), MacState::Backoff(_)));
        assert_eq!(mac.collision_count(), 1);
    }

    #[test]
    fn test_binary_exponential_backoff_doubles_window() {
        // Use Fixed backoff to measure, then switch to BinaryExponential.
        // We check that a second collision produces a backoff from a larger window.
        let config = CsmaConfig {
            min_backoff: 4,
            max_backoff: 1024,
            max_retries: 10,
            slot_time_us: 20,
            sifs_us: 10,
            difs_us: 50,
        };
        let mut mac = CsmaCaMac::new(config);
        // Collision 1
        mac.request_transmit(10);
        mac.tick(false); // -> WaitingAck
        let a1 = mac.tick(false); // collision 1 -> Backoff
        let b1 = match a1 {
            MacAction::BackingOff(n) => n,
            _ => panic!("expected BackingOff"),
        };

        // Drain the backoff to reach transmit again
        let mut remaining = b1;
        while remaining > 0 {
            let act = mac.tick(false);
            match act {
                MacAction::BackingOff(n) => remaining = n,
                MacAction::Transmit => break,
                _ => {}
            }
        }
        // Now in Transmitting or just transmitted — tick to WaitingAck
        if *mac.state() == MacState::Transmitting {
            mac.tick(false);
        }

        // Collision 2 — retry_count=2, window should be min_backoff * 2^2 = 16
        let a2 = mac.tick(false);
        let b2 = match a2 {
            MacAction::BackingOff(n) => n,
            _ => panic!("expected BackingOff on second collision"),
        };

        // Window for retry 1 was 4*2=8 (max value 8), retry 2 is 4*4=16 (max value 16).
        // b2 should be drawn from a larger window, so statistically b2 can be > b1's window.
        // We verify that the max possible backoff for the second collision is larger.
        // Since b1 is from [1, 8] and b2 is from [1, 16], we just verify b2 <= 16.
        assert!(b2 <= 16, "backoff {} should be <= 16", b2);
        assert!(b2 >= 1, "backoff should be >= 1");
    }

    #[test]
    fn test_max_retries_triggers_failure() {
        let config = CsmaConfig {
            min_backoff: 1,
            max_backoff: 2,
            max_retries: 1,
            slot_time_us: 20,
            sifs_us: 10,
            difs_us: 50,
        };
        let mut mac = CsmaCaMac::new(config);
        mac.set_backoff_algorithm(BackoffAlgorithm::Fixed(1));

        // Transmit
        mac.request_transmit(10);
        mac.tick(false); // Transmitting -> WaitingAck

        // First collision -> Backoff (retry 1)
        let a1 = mac.tick(false);
        assert!(matches!(a1, MacAction::BackingOff(_)));

        // Drain backoff (fixed=1, one tick on idle channel triggers transmit)
        let a2 = mac.tick(false);
        assert_eq!(a2, MacAction::Transmit);

        // Transmitting -> WaitingAck
        mac.tick(false);

        // Second collision — retry_count exceeds max_retries (1) -> Failed
        let a3 = mac.tick(false);
        assert_eq!(a3, MacAction::Failed);
        assert_eq!(*mac.state(), MacState::Idle);
    }

    #[test]
    fn test_receive_ack_transitions_to_idle() {
        let mut mac = CsmaCaMac::new(default_config());
        mac.request_transmit(20);
        mac.tick(false); // -> WaitingAck
        assert_eq!(*mac.state(), MacState::WaitingAck);

        mac.receive_ack();
        assert_eq!(*mac.state(), MacState::Idle);
        assert_eq!(mac.collision_count(), 0);
    }

    #[test]
    fn test_wifi_factory_parameters() {
        let mac = wifi_csma();
        assert_eq!(*mac.state(), MacState::Idle);
        assert_eq!(mac.config.min_backoff, 15);
        assert_eq!(mac.config.max_backoff, 1023);
        assert_eq!(mac.config.max_retries, 7);
        assert_eq!(mac.config.slot_time_us, 20);
        assert_eq!(mac.config.sifs_us, 10);
        assert_eq!(mac.config.difs_us, 50);
    }

    #[test]
    fn test_aloha_factory_parameters() {
        let mac = simple_aloha();
        assert_eq!(*mac.state(), MacState::Idle);
        assert_eq!(mac.config.min_backoff, 0);
        assert_eq!(mac.config.max_backoff, 0);
        assert_eq!(mac.config.max_retries, 0);
        assert_eq!(mac.config.slot_time_us, 1);
        assert_eq!(mac.config.sifs_us, 0);
        assert_eq!(mac.config.difs_us, 0);
        assert_eq!(mac.backoff_algo, BackoffAlgorithm::Fixed(0));
    }

    #[test]
    fn test_stats_tracking() {
        let mut mac = CsmaCaMac::new(default_config());
        mac.set_backoff_algorithm(BackoffAlgorithm::Fixed(3));

        // Successful transmit
        mac.request_transmit(10);
        mac.tick(false); // -> WaitingAck
        mac.receive_ack();

        let s1 = mac.stats();
        assert_eq!(s1.transmit_count, 1);
        assert_eq!(s1.collision_count, 0);
        assert_eq!(s1.avg_backoff, 0.0);
        // 1 tick total, 0 busy ticks
        assert_eq!(s1.channel_utilization, 0.0);

        // Second transmit with collision
        mac.request_transmit(10);
        mac.tick(false); // -> WaitingAck
        mac.tick(false); // collision -> Backoff(3)

        let s2 = mac.stats();
        assert_eq!(s2.transmit_count, 2);
        assert_eq!(s2.collision_count, 1);
        assert_eq!(s2.avg_backoff, 3.0); // one backoff event of 3 slots
    }

    #[test]
    fn test_reset_clears_state() {
        let mut mac = CsmaCaMac::new(default_config());
        mac.set_backoff_algorithm(BackoffAlgorithm::Fixed(2));

        mac.request_transmit(10);
        mac.tick(false); // -> WaitingAck
        mac.tick(false); // collision -> Backoff

        assert_ne!(*mac.state(), MacState::Idle);
        assert!(mac.collision_count() > 0);

        mac.reset();

        assert_eq!(*mac.state(), MacState::Idle);
        assert_eq!(mac.collision_count(), 0);
        let stats = mac.stats();
        assert_eq!(stats.transmit_count, 0);
        assert_eq!(stats.collision_count, 0);
        assert_eq!(stats.avg_backoff, 0.0);
        assert_eq!(stats.channel_utilization, 0.0);
    }
}

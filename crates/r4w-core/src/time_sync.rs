//! Multi-Radio Time Synchronization Protocol (MF-043)
//!
//! Provides time synchronization between multiple radios for coordinated
//! frequency hopping, beamforming, and distributed sensing applications.
//!
//! # Protocol Overview
//!
//! The synchronization protocol uses a master-slave architecture with:
//! - **Master**: Broadcasts timing beacons with GPS-derived time
//! - **Slaves**: Receive beacons and discipline local oscillators
//! - **Mesh**: Optional peer-to-peer timing for resilience
//!
//! # Timing Messages
//!
//! - `TimingBeacon`: Periodic broadcast with master time
//! - `TimingRequest`: Slave request for synchronization
//! - `TimingResponse`: Master response with timestamps
//! - `DelayRequest`/`DelayResponse`: RTT measurement for delay compensation
//!
//! # Example
//!
//! ```rust,no_run
//! use r4w_core::time_sync::{TimeSyncMaster, TimeSyncSlave, TimingBeacon};
//!
//! // Master node
//! let mut master = TimeSyncMaster::new(0x01);
//! let beacon = master.generate_beacon();
//!
//! // Slave node receives beacon
//! let mut slave = TimeSyncSlave::new(0x02);
//! slave.process_beacon(&beacon);
//!
//! println!("Slave offset: {} ns", slave.estimated_offset_ns());
//! ```

use std::time::{Duration, Instant};
use crate::gps_time::GpsTime;

/// Maximum allowed clock drift between sync messages (ppm)
#[allow(dead_code)]
const MAX_DRIFT_PPM: f64 = 50.0;

/// Beacon interval for timing broadcast (milliseconds)
const BEACON_INTERVAL_MS: u64 = 100;

/// Number of samples for offset averaging
const OFFSET_FILTER_SIZE: usize = 16;

/// Timing message types
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum TimingMessageType {
    /// Periodic timing beacon from master
    Beacon = 0x01,
    /// Sync request from slave
    SyncRequest = 0x02,
    /// Sync response from master
    SyncResponse = 0x03,
    /// Delay measurement request
    DelayRequest = 0x04,
    /// Delay measurement response
    DelayResponse = 0x05,
    /// Follow-up message with precise timestamp
    FollowUp = 0x06,
}

/// Timing beacon message
#[derive(Debug, Clone)]
pub struct TimingBeacon {
    /// Message type
    pub msg_type: TimingMessageType,
    /// Master node ID
    pub master_id: u8,
    /// Sequence number
    pub sequence: u32,
    /// GPS week
    pub gps_week: u16,
    /// GPS time of week (seconds)
    pub gps_tow_s: f64,
    /// Time uncertainty (nanoseconds)
    pub uncertainty_ns: u32,
    /// Master flags (GPS lock, PPS sync, etc.)
    pub flags: u8,
    /// Reserved for future use
    pub reserved: [u8; 4],
}

impl TimingBeacon {
    /// Create a new timing beacon
    pub fn new(master_id: u8, sequence: u32, gps_time: &GpsTime, uncertainty_ns: u32) -> Self {
        Self {
            msg_type: TimingMessageType::Beacon,
            master_id,
            sequence,
            gps_week: gps_time.week,
            gps_tow_s: gps_time.tow,
            uncertainty_ns,
            flags: 0,
            reserved: [0; 4],
        }
    }

    /// Set GPS lock flag
    pub fn with_gps_lock(mut self, locked: bool) -> Self {
        if locked {
            self.flags |= 0x01;
        }
        self
    }

    /// Set PPS sync flag
    pub fn with_pps_sync(mut self, synced: bool) -> Self {
        if synced {
            self.flags |= 0x02;
        }
        self
    }

    /// Check if GPS is locked
    pub fn gps_locked(&self) -> bool {
        self.flags & 0x01 != 0
    }

    /// Check if PPS is synced
    pub fn pps_synced(&self) -> bool {
        self.flags & 0x02 != 0
    }

    /// Get GPS time from beacon
    pub fn gps_time(&self) -> GpsTime {
        GpsTime::from_week_tow(self.gps_week, self.gps_tow_s)
    }

    /// Serialize to bytes
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(24);
        bytes.push(self.msg_type as u8);
        bytes.push(self.master_id);
        bytes.extend_from_slice(&self.sequence.to_le_bytes());
        bytes.extend_from_slice(&self.gps_week.to_le_bytes());
        bytes.extend_from_slice(&self.gps_tow_s.to_le_bytes());
        bytes.extend_from_slice(&self.uncertainty_ns.to_le_bytes());
        bytes.push(self.flags);
        bytes.extend_from_slice(&self.reserved);
        bytes
    }

    /// Deserialize from bytes
    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < 24 {
            return None;
        }

        Some(Self {
            msg_type: match bytes[0] {
                0x01 => TimingMessageType::Beacon,
                _ => return None,
            },
            master_id: bytes[1],
            sequence: u32::from_le_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]),
            gps_week: u16::from_le_bytes([bytes[6], bytes[7]]),
            gps_tow_s: f64::from_le_bytes([
                bytes[8], bytes[9], bytes[10], bytes[11],
                bytes[12], bytes[13], bytes[14], bytes[15],
            ]),
            uncertainty_ns: u32::from_le_bytes([bytes[16], bytes[17], bytes[18], bytes[19]]),
            flags: bytes[20],
            reserved: [bytes[21], bytes[22], bytes[23], 0],
        })
    }
}

/// Delay measurement request
#[derive(Debug, Clone)]
pub struct DelayRequest {
    /// Requesting node ID
    pub node_id: u8,
    /// Sequence number
    pub sequence: u32,
    /// Transmit timestamp (local time, nanoseconds)
    pub t1_ns: u64,
}

/// Delay measurement response
#[derive(Debug, Clone)]
pub struct DelayResponse {
    /// Responding node ID
    pub node_id: u8,
    /// Sequence number (matches request)
    pub sequence: u32,
    /// Request receive timestamp (responder time)
    pub t2_ns: u64,
    /// Response transmit timestamp (responder time)
    pub t3_ns: u64,
}

/// Time synchronization master
#[derive(Debug)]
pub struct TimeSyncMaster {
    /// Node ID
    node_id: u8,
    /// Sequence counter
    sequence: u32,
    /// Last beacon time
    last_beacon: Option<Instant>,
    /// GPS time source
    gps_time: Option<GpsTime>,
    /// Time uncertainty (nanoseconds)
    uncertainty_ns: u32,
    /// GPS lock status
    gps_locked: bool,
    /// PPS sync status
    pps_synced: bool,
}

impl TimeSyncMaster {
    /// Create a new time sync master
    pub fn new(node_id: u8) -> Self {
        Self {
            node_id,
            sequence: 0,
            last_beacon: None,
            gps_time: None,
            uncertainty_ns: 1_000_000, // 1ms default uncertainty
            gps_locked: false,
            pps_synced: false,
        }
    }

    /// Update GPS time
    pub fn update_gps(&mut self, gps_time: GpsTime, uncertainty_ns: u32) {
        self.gps_time = Some(gps_time);
        self.uncertainty_ns = uncertainty_ns;
        self.gps_locked = true;
    }

    /// Update PPS sync status
    pub fn set_pps_synced(&mut self, synced: bool) {
        self.pps_synced = synced;
    }

    /// Generate a timing beacon
    pub fn generate_beacon(&mut self) -> TimingBeacon {
        let gps = self.gps_time.unwrap_or_else(GpsTime::now);

        let beacon = TimingBeacon::new(
            self.node_id,
            self.sequence,
            &gps,
            self.uncertainty_ns,
        )
        .with_gps_lock(self.gps_locked)
        .with_pps_sync(self.pps_synced);

        self.sequence = self.sequence.wrapping_add(1);
        self.last_beacon = Some(Instant::now());

        beacon
    }

    /// Check if it's time to send a beacon
    pub fn beacon_due(&self) -> bool {
        match self.last_beacon {
            Some(last) => last.elapsed() >= Duration::from_millis(BEACON_INTERVAL_MS),
            None => true,
        }
    }

    /// Get node ID
    pub fn node_id(&self) -> u8 {
        self.node_id
    }

    /// Get current sequence number
    pub fn sequence(&self) -> u32 {
        self.sequence
    }
}

/// Time synchronization slave
#[derive(Debug)]
pub struct TimeSyncSlave {
    /// Node ID
    node_id: u8,
    /// Current master ID (if synced)
    master_id: Option<u8>,
    /// Estimated time offset (nanoseconds, positive = slave ahead)
    offset_ns: i64,
    /// Estimated path delay (nanoseconds)
    delay_ns: u64,
    /// Offset filter (circular buffer)
    offset_filter: [i64; OFFSET_FILTER_SIZE],
    /// Filter index
    filter_index: usize,
    /// Filter count (for averaging)
    filter_count: usize,
    /// Last beacon receive time
    last_beacon_rx: Option<Instant>,
    /// Last beacon sequence
    last_sequence: u32,
    /// Sync state
    state: SyncState,
    /// Local clock at last sync
    local_at_sync: Option<Instant>,
    /// GPS time at last sync
    gps_at_sync: Option<GpsTime>,
}

/// Synchronization state
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SyncState {
    /// Not synchronized
    Unsync,
    /// Acquiring sync
    Acquiring,
    /// Synchronized
    Synced,
    /// Lost sync (holdover)
    Holdover,
}

impl TimeSyncSlave {
    /// Create a new time sync slave
    pub fn new(node_id: u8) -> Self {
        Self {
            node_id,
            master_id: None,
            offset_ns: 0,
            delay_ns: 0,
            offset_filter: [0; OFFSET_FILTER_SIZE],
            filter_index: 0,
            filter_count: 0,
            last_beacon_rx: None,
            last_sequence: 0,
            state: SyncState::Unsync,
            local_at_sync: None,
            gps_at_sync: None,
        }
    }

    /// Process a timing beacon
    pub fn process_beacon(&mut self, beacon: &TimingBeacon) -> bool {
        let now = Instant::now();

        // Check for valid beacon
        if !beacon.gps_locked() {
            return false;
        }

        // Calculate offset (simplified - assumes instantaneous reception)
        let gps_time = beacon.gps_time();
        let local_gps = GpsTime::now();

        // Offset = master time - local time
        let master_ns = (gps_time.week as i64 * 604800 * 1_000_000_000)
            + (gps_time.tow * 1_000_000_000.0) as i64;
        let local_ns = (local_gps.week as i64 * 604800 * 1_000_000_000)
            + (local_gps.tow * 1_000_000_000.0) as i64;

        let raw_offset = master_ns - local_ns;

        // Add to filter
        self.offset_filter[self.filter_index] = raw_offset;
        self.filter_index = (self.filter_index + 1) % OFFSET_FILTER_SIZE;
        self.filter_count = (self.filter_count + 1).min(OFFSET_FILTER_SIZE);

        // Calculate filtered offset
        let sum: i64 = self.offset_filter[..self.filter_count].iter().sum();
        self.offset_ns = sum / self.filter_count as i64;

        // Update state
        self.master_id = Some(beacon.master_id);
        self.last_beacon_rx = Some(now);
        self.last_sequence = beacon.sequence;
        self.local_at_sync = Some(now);
        self.gps_at_sync = Some(gps_time);

        // State machine
        match self.state {
            SyncState::Unsync => {
                if self.filter_count >= 3 {
                    self.state = SyncState::Acquiring;
                }
            }
            SyncState::Acquiring => {
                if self.filter_count >= OFFSET_FILTER_SIZE / 2 {
                    self.state = SyncState::Synced;
                }
            }
            SyncState::Synced => {
                // Stay synced
            }
            SyncState::Holdover => {
                // Transition back to synced
                self.state = SyncState::Synced;
            }
        }

        true
    }

    /// Process delay measurement response
    pub fn process_delay_response(&mut self, t1_ns: u64, response: &DelayResponse, t4_ns: u64) {
        // IEEE 1588 delay calculation
        // delay = ((t4 - t1) - (t3 - t2)) / 2
        let round_trip = t4_ns.saturating_sub(t1_ns);
        let remote_proc = response.t3_ns.saturating_sub(response.t2_ns);
        self.delay_ns = (round_trip.saturating_sub(remote_proc)) / 2;

        // Offset correction with delay
        // offset = ((t2 - t1) + (t3 - t4)) / 2
    }

    /// Get estimated time offset in nanoseconds
    pub fn estimated_offset_ns(&self) -> i64 {
        self.offset_ns
    }

    /// Get estimated path delay in nanoseconds
    pub fn estimated_delay_ns(&self) -> u64 {
        self.delay_ns
    }

    /// Get current sync state
    pub fn state(&self) -> SyncState {
        self.state
    }

    /// Check if synchronized
    pub fn is_synced(&self) -> bool {
        self.state == SyncState::Synced
    }

    /// Get current GPS time estimate
    pub fn estimated_gps_time(&self) -> Option<GpsTime> {
        let (local_at_sync, gps_at_sync) = match (self.local_at_sync, self.gps_at_sync) {
            (Some(l), Some(g)) => (l, g),
            _ => return None,
        };

        let elapsed = Instant::now().duration_since(local_at_sync);
        let gps_elapsed = Duration::from_nanos(elapsed.as_nanos() as u64);

        // Apply offset correction
        let offset_duration = if self.offset_ns >= 0 {
            gps_elapsed + Duration::from_nanos(self.offset_ns as u64)
        } else {
            gps_elapsed.saturating_sub(Duration::from_nanos((-self.offset_ns) as u64))
        };

        Some(gps_at_sync.add_duration(offset_duration))
    }

    /// Check sync health
    pub fn check_sync_health(&mut self) {
        if let Some(last_rx) = self.last_beacon_rx {
            let elapsed = last_rx.elapsed();

            // If no beacon for 1 second, enter holdover
            if elapsed > Duration::from_secs(1) && self.state == SyncState::Synced {
                self.state = SyncState::Holdover;
            }

            // If no beacon for 10 seconds, lose sync
            if elapsed > Duration::from_secs(10) && self.state == SyncState::Holdover {
                self.state = SyncState::Unsync;
                self.master_id = None;
                self.filter_count = 0;
            }
        }
    }

    /// Get master node ID
    pub fn master_id(&self) -> Option<u8> {
        self.master_id
    }

    /// Get node ID
    pub fn node_id(&self) -> u8 {
        self.node_id
    }

    /// Get time since last beacon
    pub fn time_since_beacon(&self) -> Option<Duration> {
        self.last_beacon_rx.map(|t| t.elapsed())
    }
}

/// Time sync network statistics
#[derive(Debug, Default, Clone)]
pub struct TimeSyncStats {
    /// Number of beacons received
    pub beacons_received: u64,
    /// Number of beacons lost (sequence gaps)
    pub beacons_lost: u64,
    /// Current offset (nanoseconds)
    pub offset_ns: i64,
    /// Offset jitter (nanoseconds RMS)
    pub jitter_ns: u64,
    /// Path delay (nanoseconds)
    pub delay_ns: u64,
    /// Sync uptime (seconds)
    pub sync_uptime_s: u64,
}

/// Multi-node time sync coordinator
#[derive(Debug)]
pub struct TimeSyncCoordinator {
    /// Our node ID
    #[allow(dead_code)]
    node_id: u8,
    /// Master node (if we are master)
    master: Option<TimeSyncMaster>,
    /// Slave node (if we are slave)
    slave: Option<TimeSyncSlave>,
    /// Node role
    role: NodeRole,
    /// Statistics
    stats: TimeSyncStats,
}

/// Node role in sync network
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NodeRole {
    /// Master (time source)
    Master,
    /// Slave (time follower)
    Slave,
    /// Grandmaster (network-wide time source)
    Grandmaster,
    /// Boundary (master on one interface, slave on another)
    Boundary,
}

impl TimeSyncCoordinator {
    /// Create coordinator as master
    pub fn new_master(node_id: u8) -> Self {
        Self {
            node_id,
            master: Some(TimeSyncMaster::new(node_id)),
            slave: None,
            role: NodeRole::Master,
            stats: TimeSyncStats::default(),
        }
    }

    /// Create coordinator as slave
    pub fn new_slave(node_id: u8) -> Self {
        Self {
            node_id,
            master: None,
            slave: Some(TimeSyncSlave::new(node_id)),
            role: NodeRole::Slave,
            stats: TimeSyncStats::default(),
        }
    }

    /// Get node role
    pub fn role(&self) -> NodeRole {
        self.role
    }

    /// Generate beacon (if master)
    pub fn generate_beacon(&mut self) -> Option<TimingBeacon> {
        self.master.as_mut().map(|m| m.generate_beacon())
    }

    /// Process received beacon (if slave)
    pub fn process_beacon(&mut self, beacon: &TimingBeacon) -> bool {
        if let Some(ref mut slave) = self.slave {
            if slave.process_beacon(beacon) {
                self.stats.beacons_received += 1;
                self.stats.offset_ns = slave.estimated_offset_ns();
                self.stats.delay_ns = slave.estimated_delay_ns();
                return true;
            }
        }
        false
    }

    /// Update GPS time (if master)
    pub fn update_gps(&mut self, gps_time: GpsTime, uncertainty_ns: u32) {
        if let Some(ref mut master) = self.master {
            master.update_gps(gps_time, uncertainty_ns);
        }
    }

    /// Get current GPS time estimate
    pub fn current_gps_time(&self) -> Option<GpsTime> {
        match &self.slave {
            Some(s) => s.estimated_gps_time(),
            None => self.master.as_ref().and_then(|_| Some(GpsTime::now())),
        }
    }

    /// Get sync statistics
    pub fn stats(&self) -> &TimeSyncStats {
        &self.stats
    }

    /// Check if synchronized
    pub fn is_synced(&self) -> bool {
        match &self.slave {
            Some(s) => s.is_synced(),
            None => self.master.is_some(), // Master is always "synced" if GPS is available
        }
    }

    /// Periodic maintenance
    pub fn tick(&mut self) {
        if let Some(ref mut slave) = self.slave {
            slave.check_sync_health();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_timing_beacon_creation() {
        let gps = GpsTime::from_week_tow(2345, 123456.789);
        let beacon = TimingBeacon::new(0x01, 100, &gps, 1000);

        assert_eq!(beacon.master_id, 0x01);
        assert_eq!(beacon.sequence, 100);
        assert_eq!(beacon.gps_week, 2345);
    }

    #[test]
    fn test_beacon_serialization() {
        let gps = GpsTime::from_week_tow(2345, 123456.789);
        let beacon = TimingBeacon::new(0x01, 100, &gps, 1000)
            .with_gps_lock(true)
            .with_pps_sync(true);

        let bytes = beacon.to_bytes();
        let restored = TimingBeacon::from_bytes(&bytes).unwrap();

        assert_eq!(restored.master_id, beacon.master_id);
        assert_eq!(restored.sequence, beacon.sequence);
        assert!(restored.gps_locked());
        assert!(restored.pps_synced());
    }

    #[test]
    fn test_master_beacon_generation() {
        let mut master = TimeSyncMaster::new(0x01);
        master.update_gps(GpsTime::now(), 100);
        master.set_pps_synced(true);

        let beacon1 = master.generate_beacon();
        let beacon2 = master.generate_beacon();

        assert_eq!(beacon1.sequence + 1, beacon2.sequence);
        assert!(beacon1.gps_locked());
    }

    #[test]
    fn test_slave_sync() {
        let mut master = TimeSyncMaster::new(0x01);
        master.update_gps(GpsTime::now(), 100);

        let mut slave = TimeSyncSlave::new(0x02);
        assert_eq!(slave.state(), SyncState::Unsync);

        // Process several beacons
        for _ in 0..10 {
            let beacon = master.generate_beacon();
            slave.process_beacon(&beacon);
        }

        // Should be acquiring or synced
        assert_ne!(slave.state(), SyncState::Unsync);
    }

    #[test]
    fn test_coordinator_master() {
        let mut coord = TimeSyncCoordinator::new_master(0x01);
        coord.update_gps(GpsTime::now(), 100);

        assert_eq!(coord.role(), NodeRole::Master);
        assert!(coord.is_synced());

        let beacon = coord.generate_beacon();
        assert!(beacon.is_some());
    }

    #[test]
    fn test_coordinator_slave() {
        let mut master_coord = TimeSyncCoordinator::new_master(0x01);
        master_coord.update_gps(GpsTime::now(), 100);

        let mut slave_coord = TimeSyncCoordinator::new_slave(0x02);

        // Exchange beacons
        for _ in 0..10 {
            if let Some(beacon) = master_coord.generate_beacon() {
                slave_coord.process_beacon(&beacon);
            }
        }

        assert!(slave_coord.stats().beacons_received > 0);
    }

    #[test]
    fn test_sync_holdover() {
        let mut slave = TimeSyncSlave::new(0x02);

        // Simulate sync acquisition
        let gps = GpsTime::now();
        let beacon = TimingBeacon::new(0x01, 1, &gps, 100).with_gps_lock(true);

        for _ in 0..20 {
            slave.process_beacon(&beacon);
        }

        assert_eq!(slave.state(), SyncState::Synced);

        // Simulate loss of beacons - would need real time delay to test holdover
    }
}

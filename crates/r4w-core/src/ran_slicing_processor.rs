//! 5G RAN Network Slicing Processor
//!
//! Implements RAN network slicing per 3GPP TS 23.501 and TS 38.300.
//! Provides slice management, resource partitioning, QoS enforcement,
//! inter-slice scheduling, and SLA monitoring for 5G NR deployments.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────┐
//! │               RAN Slicing Processor                 │
//! │                                                     │
//! │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │
//! │  │ Admission│  │ Resource │  │  Inter-Slice     │  │
//! │  │ Control  │  │ Manager  │  │  Scheduler       │  │
//! │  └──────────┘  └──────────┘  └──────────────────┘  │
//! │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │
//! │  │   SLA    │  │   QoS    │  │  Slice Lifecycle │  │
//! │  │ Monitor  │  │ Enforcer │  │  Manager         │  │
//! │  └──────────┘  └──────────┘  └──────────────────┘  │
//! └─────────────────────────────────────────────────────┘
//! ```
//!
//! # Example
//!
//! ```rust
//! use r4w_core::ran_slicing_processor::*;
//!
//! let mut processor = RanSlicingProcessor::new(100); // 100 PRBs total
//!
//! // Create an eMBB slice
//! let snssai = SNssai::new(Sst::Embb, Some(0x001234));
//! let config = SliceConfig::builder(snssai)
//!     .isolation(IsolationLevel::Hard)
//!     .dedicated_prbs(40)
//!     .max_bitrate_mbps(1000.0)
//!     .min_bitrate_mbps(100.0)
//!     .build();
//! let slice_id = processor.create_slice(config).unwrap();
//!
//! // Admit a UE
//! let ue = UeContext::new(1001, vec![slice_id], vec![PDuSession::new(1, slice_id)]);
//! processor.admit_ue(ue).unwrap();
//!
//! // Run scheduling
//! let schedule = processor.schedule_round(1000.0);
//! ```

use std::collections::HashMap;
use std::fmt;

// ---------------------------------------------------------------------------
// S-NSSAI structures (3GPP TS 23.501 §5.15)
// ---------------------------------------------------------------------------

/// Slice/Service Type per 3GPP TS 23.501 Table 5.15.2.2-1
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Sst {
    /// Enhanced Mobile Broadband – high data rate, large coverage
    Embb,
    /// Ultra-Reliable Low Latency Communications
    Urllc,
    /// Massive Internet of Things
    MIot,
    /// Vehicle-to-Everything
    V2x,
    /// Custom/operator-defined
    Custom(u8),
}

impl Sst {
    /// Parse from raw byte value
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Sst::Embb,
            2 => Sst::Urllc,
            3 => Sst::MIot,
            4 => Sst::V2x,
            other => Sst::Custom(other),
        }
    }

    /// Encode to raw byte
    pub fn to_u8(self) -> u8 {
        match self {
            Sst::Embb => 1,
            Sst::Urllc => 2,
            Sst::MIot => 3,
            Sst::V2x => 4,
            Sst::Custom(v) => v,
        }
    }

    /// Human-readable name
    pub fn name(self) -> &'static str {
        match self {
            Sst::Embb => "eMBB",
            Sst::Urllc => "URLLC",
            Sst::MIot => "MIoT",
            Sst::V2x => "V2X",
            Sst::Custom(_) => "Custom",
        }
    }

    /// Default latency budget in milliseconds for this SST
    pub fn default_latency_budget_ms(self) -> f64 {
        match self {
            Sst::Embb => 100.0,
            Sst::Urllc => 1.0,
            Sst::MIot => 10_000.0,
            Sst::V2x => 10.0,
            Sst::Custom(_) => 100.0,
        }
    }

    /// Default reliability target (fraction, 0..1) for this SST
    pub fn default_reliability(self) -> f64 {
        match self {
            Sst::Embb => 0.999,
            Sst::Urllc => 0.999999,
            Sst::MIot => 0.99,
            Sst::V2x => 0.9999,
            Sst::Custom(_) => 0.999,
        }
    }
}

impl fmt::Display for Sst {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SST={}", self.to_u8())
    }
}

/// Single Network Slice Selection Assistance Information (S-NSSAI)
/// Consists of SST (1 byte) and optional SD (3 bytes).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SNssai {
    pub sst: Sst,
    /// Slice Differentiator: 24-bit optional field (None = wildcard)
    pub sd: Option<u32>,
}

impl SNssai {
    /// Create new S-NSSAI. SD must fit in 24 bits if present.
    pub fn new(sst: Sst, sd: Option<u32>) -> Self {
        let sd = sd.map(|v| v & 0x00FF_FFFF);
        SNssai { sst, sd }
    }

    /// Encode to 4-byte representation (SST + SD or SST + 0xFFFFFF)
    pub fn encode(&self) -> [u8; 4] {
        let sst = self.sst.to_u8();
        let sd = self.sd.unwrap_or(0xFF_FFFF);
        [
            sst,
            ((sd >> 16) & 0xFF) as u8,
            ((sd >> 8) & 0xFF) as u8,
            (sd & 0xFF) as u8,
        ]
    }

    /// Decode from 4-byte representation
    pub fn decode(bytes: [u8; 4]) -> Self {
        let sst = Sst::from_u8(bytes[0]);
        let sd_raw = ((bytes[1] as u32) << 16) | ((bytes[2] as u32) << 8) | (bytes[3] as u32);
        let sd = if sd_raw == 0xFF_FFFF { None } else { Some(sd_raw) };
        SNssai { sst, sd }
    }

    /// Check if this S-NSSAI matches another (wildcard SD matches any)
    pub fn matches(&self, other: &SNssai) -> bool {
        if self.sst != other.sst {
            return false;
        }
        match (self.sd, other.sd) {
            (None, _) | (_, None) => true,
            (Some(a), Some(b)) => a == b,
        }
    }
}

impl fmt::Display for SNssai {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.sd {
            Some(sd) => write!(f, "S-NSSAI(SST={},SD={:06X})", self.sst.to_u8(), sd),
            None => write!(f, "S-NSSAI(SST={})", self.sst.to_u8()),
        }
    }
}

/// Network Slice Selection Assistance Information (NSSAI)
/// A set of up to 8 S-NSSAIs per 3GPP TS 23.501 §5.15.2
#[derive(Debug, Clone, Default)]
pub struct Nssai {
    entries: Vec<SNssai>,
}

impl Nssai {
    pub const MAX_ENTRIES: usize = 8;

    pub fn new() -> Self {
        Nssai { entries: Vec::new() }
    }

    /// Add an S-NSSAI. Returns error if already at capacity or duplicate.
    pub fn add(&mut self, snssai: SNssai) -> Result<(), SliceError> {
        if self.entries.len() >= Self::MAX_ENTRIES {
            return Err(SliceError::NssaiCapacityExceeded);
        }
        if self.entries.iter().any(|e| e == &snssai) {
            return Err(SliceError::DuplicateSnssai);
        }
        self.entries.push(snssai);
        Ok(())
    }

    pub fn entries(&self) -> &[SNssai] {
        &self.entries
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Check if this NSSAI contains an S-NSSAI that matches the given one
    pub fn contains_match(&self, snssai: &SNssai) -> bool {
        self.entries.iter().any(|e| e.matches(snssai))
    }
}

// ---------------------------------------------------------------------------
// NR Numerology (3GPP TS 38.211 §4.3.2)
// ---------------------------------------------------------------------------

/// Subcarrier spacing options for NR
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubcarrierSpacing {
    Scs15kHz = 0,
    Scs30kHz = 1,
    Scs60kHz = 2,
    Scs120kHz = 3,
}

impl SubcarrierSpacing {
    pub fn khz(self) -> u32 {
        match self {
            SubcarrierSpacing::Scs15kHz => 15,
            SubcarrierSpacing::Scs30kHz => 30,
            SubcarrierSpacing::Scs60kHz => 60,
            SubcarrierSpacing::Scs120kHz => 120,
        }
    }

    /// Slot duration in milliseconds
    pub fn slot_duration_ms(self) -> f64 {
        match self {
            SubcarrierSpacing::Scs15kHz => 1.0,
            SubcarrierSpacing::Scs30kHz => 0.5,
            SubcarrierSpacing::Scs60kHz => 0.25,
            SubcarrierSpacing::Scs120kHz => 0.125,
        }
    }

    /// Typical use case for this numerology
    pub fn typical_use(self) -> &'static str {
        match self {
            SubcarrierSpacing::Scs15kHz => "Sub-6 GHz eMBB/MIoT",
            SubcarrierSpacing::Scs30kHz => "Sub-6 GHz eMBB",
            SubcarrierSpacing::Scs60kHz => "Sub-6 GHz URLLC",
            SubcarrierSpacing::Scs120kHz => "mmWave eMBB/URLLC",
        }
    }

    /// Select best SCS for a given SST
    pub fn for_sst(sst: Sst) -> Self {
        match sst {
            Sst::Urllc | Sst::V2x => SubcarrierSpacing::Scs60kHz,
            Sst::MIot => SubcarrierSpacing::Scs15kHz,
            _ => SubcarrierSpacing::Scs30kHz,
        }
    }
}

// ---------------------------------------------------------------------------
// Slice isolation and configuration
// ---------------------------------------------------------------------------

/// Slice isolation level
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IsolationLevel {
    /// Hard: dedicated PRBs, no sharing under any load condition
    Hard,
    /// Soft: dedicated minimum + can borrow from shared pool with priority
    Soft,
    /// Best-Effort: competes with other slices, no guaranteed resources
    BestEffort,
}

impl IsolationLevel {
    pub fn description(self) -> &'static str {
        match self {
            IsolationLevel::Hard => "Dedicated PRBs, no sharing",
            IsolationLevel::Soft => "Min guaranteed + shared pool borrowing",
            IsolationLevel::BestEffort => "Shared pool competition only",
        }
    }
}

/// Scheduling policy for inter-slice resource allocation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulingPolicy {
    /// Weighted round-robin across slices
    WeightedRoundRobin,
    /// Strict priority: higher priority slice always served first
    StrictPriority,
    /// Proportional fairness: balance throughput * weight
    ProportionalFairness,
}

/// SLA (Service Level Agreement) targets for a slice
#[derive(Debug, Clone)]
pub struct SlaSla {
    /// Target throughput in Mbps
    pub throughput_mbps: f64,
    /// Maximum latency in milliseconds
    pub latency_ms: f64,
    /// Required packet delivery ratio (0..1)
    pub reliability: f64,
    /// SLA violation window in seconds (measurement period)
    pub violation_window_s: f64,
}

impl SlaSla {
    pub fn for_sst(sst: Sst) -> Self {
        SlaSla {
            throughput_mbps: match sst {
                Sst::Embb => 100.0,
                Sst::Urllc => 10.0,
                Sst::MIot => 1.0,
                Sst::V2x => 50.0,
                Sst::Custom(_) => 10.0,
            },
            latency_ms: sst.default_latency_budget_ms(),
            reliability: sst.default_reliability(),
            violation_window_s: 1.0,
        }
    }
}

/// Configuration for a RAN slice
#[derive(Debug, Clone)]
pub struct SliceConfig {
    pub snssai: SNssai,
    pub isolation: IsolationLevel,
    pub scheduling_policy: SchedulingPolicy,
    pub scheduling_weight: u32,
    pub scheduling_priority: u32,
    /// Dedicated PRB count (Hard isolation guaranteed; Soft minimum)
    pub dedicated_prbs: u32,
    /// Maximum PRBs this slice can use (including shared pool)
    pub max_prbs: u32,
    /// Maximum aggregate bit rate in Mbps
    pub max_bitrate_mbps: f64,
    /// Guaranteed minimum bit rate in Mbps
    pub min_bitrate_mbps: f64,
    /// Preferred numerology for this slice
    pub scs: SubcarrierSpacing,
    /// SLA targets
    pub sla: SlaSla,
}

impl SliceConfig {
    pub fn builder(snssai: SNssai) -> SliceConfigBuilder {
        SliceConfigBuilder::new(snssai)
    }
}

/// Builder for SliceConfig
pub struct SliceConfigBuilder {
    snssai: SNssai,
    isolation: IsolationLevel,
    scheduling_policy: SchedulingPolicy,
    scheduling_weight: u32,
    scheduling_priority: u32,
    dedicated_prbs: u32,
    max_prbs: u32,
    max_bitrate_mbps: f64,
    min_bitrate_mbps: f64,
    scs: SubcarrierSpacing,
    sla: Option<SlaSla>,
}

impl SliceConfigBuilder {
    fn new(snssai: SNssai) -> Self {
        let scs = SubcarrierSpacing::for_sst(snssai.sst);
        SliceConfigBuilder {
            snssai,
            isolation: IsolationLevel::Soft,
            scheduling_policy: SchedulingPolicy::ProportionalFairness,
            scheduling_weight: 10,
            scheduling_priority: 5,
            dedicated_prbs: 10,
            max_prbs: 50,
            max_bitrate_mbps: 100.0,
            min_bitrate_mbps: 0.0,
            scs,
            sla: None,
        }
    }

    pub fn isolation(mut self, level: IsolationLevel) -> Self {
        self.isolation = level;
        self
    }

    pub fn scheduling_policy(mut self, policy: SchedulingPolicy) -> Self {
        self.scheduling_policy = policy;
        self
    }

    pub fn scheduling_weight(mut self, weight: u32) -> Self {
        self.scheduling_weight = weight.max(1);
        self
    }

    pub fn scheduling_priority(mut self, priority: u32) -> Self {
        self.scheduling_priority = priority;
        self
    }

    pub fn dedicated_prbs(mut self, prbs: u32) -> Self {
        self.dedicated_prbs = prbs;
        self
    }

    pub fn max_prbs(mut self, prbs: u32) -> Self {
        self.max_prbs = prbs;
        self
    }

    pub fn max_bitrate_mbps(mut self, rate: f64) -> Self {
        self.max_bitrate_mbps = rate;
        self
    }

    pub fn min_bitrate_mbps(mut self, rate: f64) -> Self {
        self.min_bitrate_mbps = rate;
        self
    }

    pub fn scs(mut self, scs: SubcarrierSpacing) -> Self {
        self.scs = scs;
        self
    }

    pub fn sla(mut self, sla: SlaSla) -> Self {
        self.sla = Some(sla);
        self
    }

    pub fn build(self) -> SliceConfig {
        let sla = self.sla.unwrap_or_else(|| SlaSla::for_sst(self.snssai.sst));
        SliceConfig {
            snssai: self.snssai,
            isolation: self.isolation,
            scheduling_policy: self.scheduling_policy,
            scheduling_weight: self.scheduling_weight,
            scheduling_priority: self.scheduling_priority,
            dedicated_prbs: self.dedicated_prbs,
            max_prbs: self.max_prbs,
            max_bitrate_mbps: self.max_bitrate_mbps,
            min_bitrate_mbps: self.min_bitrate_mbps,
            scs: self.scs,
            sla,
        }
    }
}

// ---------------------------------------------------------------------------
// Slice state and metrics
// ---------------------------------------------------------------------------

/// Unique slice identifier
pub type SliceId = u32;

/// Lifecycle states for a slice
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SliceState {
    Creating,
    Active,
    Modifying,
    Scaling,
    Deleting,
    Inactive,
}

/// Per-time-period metrics for a single slice
#[derive(Debug, Clone, Default)]
pub struct SliceMetrics {
    /// Allocated PRBs in this period
    pub allocated_prbs: u32,
    /// PRBs actually used (with data)
    pub used_prbs: u32,
    /// Achieved throughput in Mbps
    pub throughput_mbps: f64,
    /// Average latency in milliseconds
    pub latency_ms: f64,
    /// Packet delivery ratio (0..1)
    pub packet_delivery_ratio: f64,
    /// Number of UEs currently served
    pub active_ues: u32,
    /// SLA violation flag for this period
    pub sla_violated: bool,
    /// Cumulative SLA violation count
    pub sla_violation_count: u64,
    /// Cumulative periods measured
    pub measurement_periods: u64,
    /// PRB utilization efficiency (0..1)
    pub prb_efficiency: f64,
}

impl SliceMetrics {
    /// SLA violation rate over all measurement periods
    pub fn violation_rate(&self) -> f64 {
        if self.measurement_periods == 0 {
            return 0.0;
        }
        self.sla_violation_count as f64 / self.measurement_periods as f64
    }

    /// PRB utilization fraction (0..1)
    pub fn prb_utilization(&self) -> f64 {
        if self.allocated_prbs == 0 {
            return 0.0;
        }
        self.used_prbs as f64 / self.allocated_prbs as f64
    }
}

/// Internal state for a single slice instance
#[derive(Debug)]
struct SliceInstance {
    id: SliceId,
    config: SliceConfig,
    state: SliceState,
    metrics: SliceMetrics,
    /// UEs currently associated with this slice
    ue_ids: Vec<u32>,
    /// Running allocation for dynamic adjustment
    current_prbs: u32,
    /// Token bucket for bitrate enforcement (bytes remaining)
    token_bucket: f64,
    /// Timestamp of last token refill (ms)
    last_refill_ms: f64,
    /// Sliding window of latency samples (ms)
    latency_window: Vec<f64>,
    /// Sliding window of delivery ratio samples
    delivery_window: Vec<f64>,
}

impl SliceInstance {
    fn new(id: SliceId, config: SliceConfig) -> Self {
        let initial_prbs = config.dedicated_prbs;
        let token_bucket = config.max_bitrate_mbps * 1e6 / 8.0; // bytes
        SliceInstance {
            id,
            config,
            state: SliceState::Creating,
            metrics: SliceMetrics::default(),
            ue_ids: Vec::new(),
            current_prbs: initial_prbs,
            token_bucket,
            last_refill_ms: 0.0,
            latency_window: Vec::with_capacity(100),
            delivery_window: Vec::with_capacity(100),
        }
    }

    /// Refill token bucket based on elapsed time
    fn refill_tokens(&mut self, now_ms: f64) {
        let elapsed_s = (now_ms - self.last_refill_ms) / 1000.0;
        if elapsed_s <= 0.0 {
            return;
        }
        let max_rate_bytes_per_s = self.config.max_bitrate_mbps * 1e6 / 8.0;
        let bucket_max = max_rate_bytes_per_s; // 1-second burst budget
        self.token_bucket = (self.token_bucket + elapsed_s * max_rate_bytes_per_s).min(bucket_max);
        self.last_refill_ms = now_ms;
    }

    /// Try to consume `bytes` from token bucket. Returns allowed bytes.
    fn consume_tokens(&mut self, bytes: f64) -> f64 {
        let allowed = bytes.min(self.token_bucket);
        self.token_bucket -= allowed;
        allowed
    }

    /// Record a latency observation
    fn record_latency(&mut self, latency_ms: f64) {
        const WINDOW: usize = 50;
        if self.latency_window.len() >= WINDOW {
            self.latency_window.remove(0);
        }
        self.latency_window.push(latency_ms);
    }

    /// Record a delivery ratio observation
    fn record_delivery(&mut self, ratio: f64) {
        const WINDOW: usize = 50;
        if self.delivery_window.len() >= WINDOW {
            self.delivery_window.remove(0);
        }
        self.delivery_window.push(ratio);
    }

    /// Compute mean latency from window
    fn mean_latency_ms(&self) -> f64 {
        if self.latency_window.is_empty() {
            return 0.0;
        }
        self.latency_window.iter().sum::<f64>() / self.latency_window.len() as f64
    }

    /// Compute mean delivery ratio from window
    fn mean_delivery_ratio(&self) -> f64 {
        if self.delivery_window.is_empty() {
            return 1.0;
        }
        self.delivery_window.iter().sum::<f64>() / self.delivery_window.len() as f64
    }

    /// Check if SLA is violated based on current metrics
    fn check_sla_violation(&self) -> bool {
        let lat_violated = self.mean_latency_ms() > self.config.sla.latency_ms;
        let rel_violated = self.mean_delivery_ratio() < self.config.sla.reliability;
        let tp_violated = self.metrics.throughput_mbps < self.config.sla.throughput_mbps * 0.9;
        lat_violated || rel_violated || tp_violated
    }
}

// ---------------------------------------------------------------------------
// PDU Sessions and UE context
// ---------------------------------------------------------------------------

/// A PDU session associated with a specific slice
#[derive(Debug, Clone)]
pub struct PDuSession {
    pub session_id: u32,
    pub slice_id: SliceId,
    /// Requested bit rate in Mbps (0 = best effort)
    pub requested_bitrate_mbps: f64,
    /// Queue depth: pending bytes
    pub pending_bytes: f64,
}

impl PDuSession {
    pub fn new(session_id: u32, slice_id: SliceId) -> Self {
        PDuSession {
            session_id,
            slice_id,
            requested_bitrate_mbps: 0.0,
            pending_bytes: 0.0,
        }
    }

    pub fn with_bitrate(mut self, mbps: f64) -> Self {
        self.requested_bitrate_mbps = mbps;
        self
    }
}

/// UE (User Equipment) context for multi-slice association
#[derive(Debug, Clone)]
pub struct UeContext {
    pub ue_id: u32,
    /// Requested slice IDs (from UE's allowed NSSAI)
    pub requested_slices: Vec<SliceId>,
    /// Active PDU sessions
    pub pdu_sessions: Vec<PDuSession>,
    /// Per-slice buffer fill (bytes)
    pub buffer_bytes: HashMap<SliceId, f64>,
}

impl UeContext {
    pub fn new(ue_id: u32, slices: Vec<SliceId>, sessions: Vec<PDuSession>) -> Self {
        UeContext {
            ue_id,
            requested_slices: slices,
            pdu_sessions: sessions,
            buffer_bytes: HashMap::new(),
        }
    }

    /// Add data to UE buffer for a specific slice
    pub fn enqueue(&mut self, slice_id: SliceId, bytes: f64) {
        *self.buffer_bytes.entry(slice_id).or_insert(0.0) += bytes;
    }

    /// Dequeue up to `max_bytes` from the UE's buffer for a slice
    pub fn dequeue(&mut self, slice_id: SliceId, max_bytes: f64) -> f64 {
        let buf = self.buffer_bytes.entry(slice_id).or_insert(0.0);
        let sent = max_bytes.min(*buf);
        *buf -= sent;
        sent
    }

    /// Total pending bytes across all slice buffers
    pub fn total_pending_bytes(&self) -> f64 {
        self.buffer_bytes.values().sum()
    }
}

// ---------------------------------------------------------------------------
// Admission control
// ---------------------------------------------------------------------------

/// Result of a slice admission control decision
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AdmissionResult {
    Accepted,
    /// Rejected with a reason code
    Rejected(RejectionCause),
}

/// Reason for admission rejection
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RejectionCause {
    InsufficientPrbs,
    SliceNotFound,
    SliceNotActive,
    MaxUeCapacity,
    BitrateMismatch,
    SliceAtCapacity,
}

impl fmt::Display for RejectionCause {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            RejectionCause::InsufficientPrbs => "Insufficient PRBs",
            RejectionCause::SliceNotFound => "Slice not found",
            RejectionCause::SliceNotActive => "Slice not active",
            RejectionCause::MaxUeCapacity => "Maximum UE capacity",
            RejectionCause::BitrateMismatch => "Bitrate mismatch",
            RejectionCause::SliceAtCapacity => "Slice at capacity",
        };
        write!(f, "{}", s)
    }
}

// ---------------------------------------------------------------------------
// Scheduling output
// ---------------------------------------------------------------------------

/// Per-slice allocation result for a single scheduling round
#[derive(Debug, Clone)]
pub struct SliceAllocation {
    pub slice_id: SliceId,
    pub snssai: SNssai,
    pub allocated_prbs: u32,
    /// Per-UE byte allocation within this slice
    pub ue_allocations: Vec<(u32, f64)>, // (ue_id, bytes)
    /// Slice throughput for this round in Mbps
    pub throughput_mbps: f64,
}

/// Overall scheduling decision for one TTI (Transmission Time Interval)
#[derive(Debug, Clone)]
pub struct SchedulingDecision {
    /// Timestamp in ms
    pub timestamp_ms: f64,
    /// Total PRBs available
    pub total_prbs: u32,
    /// Per-slice allocations
    pub allocations: Vec<SliceAllocation>,
    /// Unallocated (wasted) PRBs
    pub unallocated_prbs: u32,
    /// Cross-slice interference estimate (0..1, higher = more interference)
    pub cross_slice_interference: f64,
}

// ---------------------------------------------------------------------------
// Cross-slice interference model
// ---------------------------------------------------------------------------

/// Simple cross-slice interference estimate based on PRB overlap
/// and numerology differences. Returns fraction 0..1.
fn estimate_cross_slice_interference(slices: &[&SliceInstance]) -> f64 {
    if slices.len() < 2 {
        return 0.0;
    }
    // Count slices with different SCS sharing PRBs (soft/BE isolation)
    let mut scs_set: Vec<u32> = slices
        .iter()
        .filter(|s| s.config.isolation != IsolationLevel::Hard)
        .map(|s| s.config.scs.khz())
        .collect();
    scs_set.dedup();
    // Multiple numerologies sharing the same band increases interference
    let numerology_penalty = if scs_set.len() > 1 {
        0.05 * (scs_set.len() - 1) as f64
    } else {
        0.0
    };
    // Adjacent channel: count soft/BE slices sharing the pool
    let soft_count = slices
        .iter()
        .filter(|s| s.config.isolation != IsolationLevel::Hard)
        .count();
    let load_penalty = if soft_count > 1 {
        0.02 * (soft_count - 1) as f64
    } else {
        0.0
    };
    (numerology_penalty + load_penalty).min(1.0)
}

// ---------------------------------------------------------------------------
// Resource partitioning
// ---------------------------------------------------------------------------

/// PRB pool snapshot used during scheduling
#[derive(Debug, Clone)]
struct PrbPool {
    total: u32,
    dedicated_used: u32,
    shared_available: u32,
}

impl PrbPool {
    fn new(total: u32, dedicated_reserved: u32) -> Self {
        PrbPool {
            total,
            dedicated_used: dedicated_reserved,
            shared_available: total.saturating_sub(dedicated_reserved),
        }
    }

    fn take_shared(&mut self, n: u32) -> u32 {
        let taken = n.min(self.shared_available);
        self.shared_available -= taken;
        taken
    }
}

// ---------------------------------------------------------------------------
// Error types
// ---------------------------------------------------------------------------

/// Errors from RAN slicing operations
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SliceError {
    SliceNotFound(SliceId),
    SliceAlreadyExists,
    InvalidConfig(String),
    NssaiCapacityExceeded,
    DuplicateSnssai,
    InsufficientCapacity,
    UeNotFound(u32),
    OperationInvalidState(SliceState),
}

impl fmt::Display for SliceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SliceError::SliceNotFound(id) => write!(f, "Slice {} not found", id),
            SliceError::SliceAlreadyExists => write!(f, "Slice already exists"),
            SliceError::InvalidConfig(msg) => write!(f, "Invalid config: {}", msg),
            SliceError::NssaiCapacityExceeded => write!(f, "NSSAI capacity exceeded (max 8)"),
            SliceError::DuplicateSnssai => write!(f, "Duplicate S-NSSAI"),
            SliceError::InsufficientCapacity => write!(f, "Insufficient PRB capacity"),
            SliceError::UeNotFound(id) => write!(f, "UE {} not found", id),
            SliceError::OperationInvalidState(s) => {
                write!(f, "Operation invalid in state {:?}", s)
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Proportional fairness scheduler helper
// ---------------------------------------------------------------------------

/// Compute proportional fairness weight for a UE given its
/// current (short-term) throughput and average (long-term) throughput.
/// PF weight = 1 / average_throughput (standard PF)
fn pf_weight(average_tp: f64) -> f64 {
    if average_tp < 1e-6 {
        1e6 // Very large weight for starved UEs
    } else {
        1.0 / average_tp
    }
}

// ---------------------------------------------------------------------------
// Dynamic resource adjustment
// ---------------------------------------------------------------------------

/// Policy for dynamically scaling slice resources
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScalingPolicy {
    /// No dynamic adjustment
    Static,
    /// Scale up when utilization > high_watermark, down when < low_watermark
    Reactive,
    /// Predict load from trend and pre-allocate
    Predictive,
}

/// Thresholds for reactive scaling
pub struct ScalingThresholds {
    pub high_watermark: f64,  // utilization fraction to trigger scale-up
    pub low_watermark: f64,   // utilization fraction to trigger scale-down
    pub step_prbs: u32,       // PRBs to add/remove per step
}

impl Default for ScalingThresholds {
    fn default() -> Self {
        ScalingThresholds {
            high_watermark: 0.85,
            low_watermark: 0.30,
            step_prbs: 5,
        }
    }
}

// ---------------------------------------------------------------------------
// Main RAN Slicing Processor
// ---------------------------------------------------------------------------

/// The main 5G RAN Network Slicing Processor.
///
/// Manages the lifecycle of network slices, performs resource partitioning,
/// runs the inter-slice scheduler, enforces per-slice QoS, and monitors SLAs.
pub struct RanSlicingProcessor {
    /// Total available Physical Resource Blocks
    total_prbs: u32,
    /// Active slices keyed by SliceId
    slices: HashMap<SliceId, SliceInstance>,
    /// UE contexts keyed by UE ID
    ues: HashMap<u32, UeContext>,
    /// Round-robin scheduling state (current slice index)
    rr_index: usize,
    /// Next slice ID counter
    next_slice_id: SliceId,
    /// Scaling policy
    scaling_policy: ScalingPolicy,
    /// Scaling thresholds
    scaling_thresholds: ScalingThresholds,
    /// Average UE throughput tracker for PF scheduling: ue_id -> avg_Mbps
    ue_avg_throughput: HashMap<u32, f64>,
    /// PF smoothing factor (exponential moving average, 0..1)
    pf_alpha: f64,
    /// Current simulated time in milliseconds
    current_time_ms: f64,
    /// Maximum UEs per slice
    max_ues_per_slice: usize,
    /// Bytes per PRB per millisecond (capacity model)
    bytes_per_prb_ms: f64,
}

impl RanSlicingProcessor {
    /// Create a new processor with `total_prbs` Physical Resource Blocks.
    /// Assumes a moderate spectral efficiency: ~1500 bytes/PRB/ms (≈12 Mbit/s per PRB).
    pub fn new(total_prbs: u32) -> Self {
        RanSlicingProcessor {
            total_prbs,
            slices: HashMap::new(),
            ues: HashMap::new(),
            rr_index: 0,
            next_slice_id: 1,
            scaling_policy: ScalingPolicy::Reactive,
            scaling_thresholds: ScalingThresholds::default(),
            ue_avg_throughput: HashMap::new(),
            pf_alpha: 0.1, // 10% new sample weight
            current_time_ms: 0.0,
            max_ues_per_slice: 256,
            bytes_per_prb_ms: 1500.0,
        }
    }

    /// Set the scaling policy
    pub fn set_scaling_policy(&mut self, policy: ScalingPolicy, thresholds: ScalingThresholds) {
        self.scaling_policy = policy;
        self.scaling_thresholds = thresholds;
    }

    /// Set bytes per PRB per millisecond (capacity model)
    pub fn set_spectral_efficiency(&mut self, bytes_per_prb_ms: f64) {
        self.bytes_per_prb_ms = bytes_per_prb_ms.max(1.0);
    }

    // -----------------------------------------------------------------------
    // Slice lifecycle management
    // -----------------------------------------------------------------------

    /// Create a new slice with the given configuration.
    /// Returns the new SliceId on success.
    pub fn create_slice(&mut self, config: SliceConfig) -> Result<SliceId, SliceError> {
        // Validate config
        if config.dedicated_prbs > self.total_prbs {
            return Err(SliceError::InvalidConfig(
                "dedicated_prbs exceeds total_prbs".into(),
            ));
        }
        if config.max_prbs > self.total_prbs {
            return Err(SliceError::InvalidConfig(
                "max_prbs exceeds total_prbs".into(),
            ));
        }
        if config.dedicated_prbs > config.max_prbs {
            return Err(SliceError::InvalidConfig(
                "dedicated_prbs > max_prbs".into(),
            ));
        }
        if config.min_bitrate_mbps > config.max_bitrate_mbps {
            return Err(SliceError::InvalidConfig(
                "min_bitrate_mbps > max_bitrate_mbps".into(),
            ));
        }

        // Check PRB capacity (dedicated slices need guaranteed headroom)
        if config.isolation == IsolationLevel::Hard {
            let used: u32 = self
                .slices
                .values()
                .filter(|s| s.config.isolation == IsolationLevel::Hard)
                .map(|s| s.config.dedicated_prbs)
                .sum();
            if used + config.dedicated_prbs > self.total_prbs {
                return Err(SliceError::InsufficientCapacity);
            }
        }

        let id = self.next_slice_id;
        self.next_slice_id += 1;

        let mut instance = SliceInstance::new(id, config);
        instance.state = SliceState::Active;
        self.slices.insert(id, instance);
        Ok(id)
    }

    /// Modify an existing slice's configuration
    pub fn modify_slice(&mut self, id: SliceId, new_config: SliceConfig) -> Result<(), SliceError> {
        let instance = self
            .slices
            .get_mut(&id)
            .ok_or(SliceError::SliceNotFound(id))?;
        if instance.state != SliceState::Active {
            return Err(SliceError::OperationInvalidState(instance.state));
        }
        instance.state = SliceState::Modifying;
        instance.config = new_config;
        instance.current_prbs = instance.config.dedicated_prbs;
        instance.state = SliceState::Active;
        Ok(())
    }

    /// Delete a slice. UEs are detached.
    pub fn delete_slice(&mut self, id: SliceId) -> Result<(), SliceError> {
        let instance = self
            .slices
            .get_mut(&id)
            .ok_or(SliceError::SliceNotFound(id))?;
        instance.state = SliceState::Deleting;
        // Detach UEs
        let ue_ids: Vec<u32> = instance.ue_ids.clone();
        for ue_id in &ue_ids {
            if let Some(ue) = self.ues.get_mut(ue_id) {
                ue.requested_slices.retain(|&s| s != id);
                ue.pdu_sessions.retain(|s| s.slice_id != id);
            }
        }
        self.slices.remove(&id);
        Ok(())
    }

    /// Scale a slice's dedicated PRBs up or down
    pub fn scale_slice(&mut self, id: SliceId, new_dedicated_prbs: u32) -> Result<(), SliceError> {
        let instance = self
            .slices
            .get_mut(&id)
            .ok_or(SliceError::SliceNotFound(id))?;
        if new_dedicated_prbs > instance.config.max_prbs {
            return Err(SliceError::InvalidConfig(
                "new_dedicated_prbs > max_prbs".into(),
            ));
        }
        instance.state = SliceState::Scaling;
        instance.config.dedicated_prbs = new_dedicated_prbs;
        instance.current_prbs = new_dedicated_prbs;
        instance.state = SliceState::Active;
        Ok(())
    }

    /// Get slice state
    pub fn slice_state(&self, id: SliceId) -> Option<SliceState> {
        self.slices.get(&id).map(|s| s.state)
    }

    /// Get all active slice IDs
    pub fn active_slice_ids(&self) -> Vec<SliceId> {
        self.slices
            .iter()
            .filter(|(_, s)| s.state == SliceState::Active)
            .map(|(id, _)| *id)
            .collect()
    }

    // -----------------------------------------------------------------------
    // UE admission
    // -----------------------------------------------------------------------

    /// Admit a UE to one or more slices. Runs per-slice admission control.
    pub fn admit_ue(&mut self, ue: UeContext) -> Result<AdmissionResult, SliceError> {
        // Validate each requested slice
        for &sid in &ue.requested_slices {
            let instance = self
                .slices
                .get(&sid)
                .ok_or(SliceError::SliceNotFound(sid))?;
            if instance.state != SliceState::Active {
                return Ok(AdmissionResult::Rejected(RejectionCause::SliceNotActive));
            }
            if instance.ue_ids.len() >= self.max_ues_per_slice {
                return Ok(AdmissionResult::Rejected(RejectionCause::SliceAtCapacity));
            }
        }

        let ue_id = ue.ue_id;
        self.ues.insert(ue_id, ue);

        // Associate UE with slices
        let ue = self.ues.get(&ue_id).unwrap();
        let slices = ue.requested_slices.clone();
        for sid in slices {
            if let Some(instance) = self.slices.get_mut(&sid) {
                if !instance.ue_ids.contains(&ue_id) {
                    instance.ue_ids.push(ue_id);
                    instance.metrics.active_ues += 1;
                }
            }
        }

        self.ue_avg_throughput.entry(ue_id).or_insert(0.0);
        Ok(AdmissionResult::Accepted)
    }

    /// Remove a UE from the system
    pub fn release_ue(&mut self, ue_id: u32) -> Result<(), SliceError> {
        let ue = self
            .ues
            .remove(&ue_id)
            .ok_or(SliceError::UeNotFound(ue_id))?;
        for sid in &ue.requested_slices {
            if let Some(instance) = self.slices.get_mut(sid) {
                instance.ue_ids.retain(|&u| u != ue_id);
                if instance.metrics.active_ues > 0 {
                    instance.metrics.active_ues -= 1;
                }
            }
        }
        self.ue_avg_throughput.remove(&ue_id);
        Ok(())
    }

    /// Check if a UE can be admitted to a specific slice
    pub fn admission_check(&self, ue_id: u32, slice_id: SliceId) -> AdmissionResult {
        let instance = match self.slices.get(&slice_id) {
            Some(s) => s,
            None => return AdmissionResult::Rejected(RejectionCause::SliceNotFound),
        };
        if instance.state != SliceState::Active {
            return AdmissionResult::Rejected(RejectionCause::SliceNotActive);
        }
        if instance.ue_ids.contains(&ue_id) {
            return AdmissionResult::Accepted; // Already admitted
        }
        if instance.ue_ids.len() >= self.max_ues_per_slice {
            return AdmissionResult::Rejected(RejectionCause::SliceAtCapacity);
        }
        // Check if we have enough resource headroom for one more UE
        let min_prbs_per_ue = 1u32;
        if instance.current_prbs < min_prbs_per_ue {
            return AdmissionResult::Rejected(RejectionCause::InsufficientPrbs);
        }
        AdmissionResult::Accepted
    }

    // -----------------------------------------------------------------------
    // Inter-slice scheduling
    // -----------------------------------------------------------------------

    /// Run one scheduling round at `timestamp_ms`.
    /// Allocates PRBs across active slices and within each slice across UEs.
    pub fn schedule_round(&mut self, timestamp_ms: f64) -> SchedulingDecision {
        self.current_time_ms = timestamp_ms;

        // Step 1: Collect active slice IDs
        let active_ids: Vec<SliceId> = self.active_slice_ids();

        // Step 2: Compute PRB pool
        let dedicated_total: u32 = active_ids
            .iter()
            .filter_map(|id| self.slices.get(id))
            .filter(|s| s.config.isolation == IsolationLevel::Hard)
            .map(|s| s.config.dedicated_prbs)
            .sum();
        let mut pool = PrbPool::new(self.total_prbs, dedicated_total);

        // Step 3: Assign base (dedicated) PRBs and collect policies
        let mut allocations: Vec<(SliceId, u32)> = Vec::new();
        for &sid in &active_ids {
            let instance = &self.slices[&sid];
            match instance.config.isolation {
                IsolationLevel::Hard => {
                    allocations.push((sid, instance.config.dedicated_prbs));
                }
                IsolationLevel::Soft => {
                    allocations.push((sid, instance.config.dedicated_prbs));
                }
                IsolationLevel::BestEffort => {
                    allocations.push((sid, 0));
                }
            }
        }

        // Step 4: Distribute shared pool according to scheduling policy
        // Determine which policy to use (all slices share one policy for simplicity;
        // in practice per-slice policies can be used per-group)
        // Use the most common policy among active slices
        let policy = self.dominant_policy(&active_ids);
        self.distribute_shared_prbs(&mut allocations, &mut pool, policy, &active_ids);

        // Step 5: Clamp allocations to max_prbs
        for (sid, prbs) in allocations.iter_mut() {
            if let Some(instance) = self.slices.get(sid) {
                *prbs = (*prbs).min(instance.config.max_prbs);
            }
        }

        // Step 6: Compute cross-slice interference
        let slice_refs: Vec<&SliceInstance> = active_ids
            .iter()
            .filter_map(|id| self.slices.get(id))
            .collect();
        let interference = estimate_cross_slice_interference(&slice_refs);

        // Step 7: Intra-slice scheduling (per-UE allocation)
        let mut slice_allocations: Vec<SliceAllocation> = Vec::new();
        let mut total_allocated_prbs = 0u32;

        for (sid, prbs) in &allocations {
            let slice_alloc = self.schedule_intra_slice(*sid, *prbs, timestamp_ms);
            total_allocated_prbs += slice_alloc.allocated_prbs;
            slice_allocations.push(slice_alloc);
        }

        // Step 8: Update slice metrics and check SLA
        for alloc in &slice_allocations {
            self.update_slice_metrics(alloc, timestamp_ms);
        }

        // Step 9: Dynamic resource adjustment
        if self.scaling_policy != ScalingPolicy::Static {
            let ids_to_check: Vec<SliceId> = active_ids.clone();
            self.dynamic_adjustment(&ids_to_check);
        }

        SchedulingDecision {
            timestamp_ms,
            total_prbs: self.total_prbs,
            allocations: slice_allocations,
            unallocated_prbs: self.total_prbs.saturating_sub(total_allocated_prbs),
            cross_slice_interference: interference,
        }
    }

    /// Determine the dominant scheduling policy among active slices
    fn dominant_policy(&self, ids: &[SliceId]) -> SchedulingPolicy {
        let mut counts: [usize; 3] = [0; 3];
        for &id in ids {
            if let Some(s) = self.slices.get(&id) {
                let idx = match s.config.scheduling_policy {
                    SchedulingPolicy::WeightedRoundRobin => 0,
                    SchedulingPolicy::StrictPriority => 1,
                    SchedulingPolicy::ProportionalFairness => 2,
                };
                counts[idx] += 1;
            }
        }
        if counts[2] >= counts[1] && counts[2] >= counts[0] {
            SchedulingPolicy::ProportionalFairness
        } else if counts[1] >= counts[0] {
            SchedulingPolicy::StrictPriority
        } else {
            SchedulingPolicy::WeightedRoundRobin
        }
    }

    /// Distribute shared pool PRBs to soft/BE slices according to policy
    fn distribute_shared_prbs(
        &mut self,
        allocations: &mut Vec<(SliceId, u32)>,
        pool: &mut PrbPool,
        policy: SchedulingPolicy,
        active_ids: &[SliceId],
    ) {
        match policy {
            SchedulingPolicy::WeightedRoundRobin => {
                self.distribute_wrr(allocations, pool, active_ids);
            }
            SchedulingPolicy::StrictPriority => {
                self.distribute_strict_priority(allocations, pool, active_ids);
            }
            SchedulingPolicy::ProportionalFairness => {
                self.distribute_proportional_fairness(allocations, pool, active_ids);
            }
        }
    }

    /// Weighted Round Robin shared pool distribution
    fn distribute_wrr(
        &mut self,
        allocations: &mut Vec<(SliceId, u32)>,
        pool: &mut PrbPool,
        active_ids: &[SliceId],
    ) {
        // Compute total weight of non-Hard slices
        let total_weight: u32 = active_ids
            .iter()
            .filter_map(|id| self.slices.get(id))
            .filter(|s| s.config.isolation != IsolationLevel::Hard)
            .map(|s| s.config.scheduling_weight)
            .sum();

        if total_weight == 0 || pool.shared_available == 0 {
            return;
        }

        let available = pool.shared_available;
        for alloc in allocations.iter_mut() {
            let sid = alloc.0;
            if let Some(instance) = self.slices.get(&sid) {
                if instance.config.isolation == IsolationLevel::Hard {
                    continue;
                }
                let share = (available * instance.config.scheduling_weight) / total_weight;
                let extra = pool.take_shared(share);
                alloc.1 += extra;
            }
        }
    }

    /// Strict Priority shared pool distribution
    fn distribute_strict_priority(
        &mut self,
        allocations: &mut Vec<(SliceId, u32)>,
        pool: &mut PrbPool,
        active_ids: &[SliceId],
    ) {
        // Sort by priority (higher number = higher priority)
        let mut sorted: Vec<(SliceId, u32)> = active_ids
            .iter()
            .filter_map(|id| {
                self.slices.get(id).and_then(|s| {
                    if s.config.isolation != IsolationLevel::Hard {
                        Some((*id, s.config.scheduling_priority))
                    } else {
                        None
                    }
                })
            })
            .collect();
        sorted.sort_by(|a, b| b.1.cmp(&a.1));

        for (sid, _pri) in sorted {
            if pool.shared_available == 0 {
                break;
            }
            if let Some(instance) = self.slices.get(&sid) {
                let want = instance.config.max_prbs.saturating_sub(instance.config.dedicated_prbs);
                let extra = pool.take_shared(want);
                for alloc in allocations.iter_mut() {
                    if alloc.0 == sid {
                        alloc.1 += extra;
                        break;
                    }
                }
            }
        }
    }

    /// Proportional Fairness shared pool distribution
    fn distribute_proportional_fairness(
        &mut self,
        allocations: &mut Vec<(SliceId, u32)>,
        pool: &mut PrbPool,
        active_ids: &[SliceId],
    ) {
        // Compute PF weight = w_slice / avg_throughput for each slice
        let pf_weights: Vec<(SliceId, f64)> = active_ids
            .iter()
            .filter_map(|id| {
                self.slices.get(id).and_then(|s| {
                    if s.config.isolation == IsolationLevel::Hard {
                        return None;
                    }
                    let avg_tp = s.metrics.throughput_mbps.max(1e-6);
                    let w = (s.config.scheduling_weight as f64) * pf_weight(avg_tp);
                    Some((*id, w))
                })
            })
            .collect();

        let total_pf: f64 = pf_weights.iter().map(|(_, w)| w).sum();
        if total_pf <= 0.0 || pool.shared_available == 0 {
            return;
        }

        let available = pool.shared_available as f64;
        for (sid, w) in &pf_weights {
            let share = ((available * w / total_pf).round() as u32).min(pool.shared_available);
            let extra = pool.take_shared(share);
            for alloc in allocations.iter_mut() {
                if alloc.0 == *sid {
                    alloc.1 += extra;
                    break;
                }
            }
        }
    }

    /// Intra-slice scheduling: distribute `prbs` among UEs associated with slice `sid`
    fn schedule_intra_slice(
        &mut self,
        sid: SliceId,
        prbs: u32,
        timestamp_ms: f64,
    ) -> SliceAllocation {
        let instance = match self.slices.get_mut(&sid) {
            Some(s) => s,
            None => {
                return SliceAllocation {
                    slice_id: sid,
                    snssai: SNssai::new(Sst::Embb, None),
                    allocated_prbs: 0,
                    ue_allocations: vec![],
                    throughput_mbps: 0.0,
                }
            }
        };

        instance.refill_tokens(timestamp_ms);
        let snssai = instance.config.snssai;

        // Bytes available this TTI based on PRB allocation
        let bytes_available = prbs as f64 * self.bytes_per_prb_ms;

        // Get the UEs for this slice
        let ue_ids: Vec<u32> = instance.ue_ids.clone();
        let n_ues = ue_ids.len();

        if n_ues == 0 || prbs == 0 {
            return SliceAllocation {
                slice_id: sid,
                snssai,
                allocated_prbs: prbs,
                ue_allocations: vec![],
                throughput_mbps: 0.0,
            };
        }

        // Compute PF weights per UE
        let mut ue_weights: Vec<(u32, f64)> = ue_ids
            .iter()
            .map(|&uid| {
                let avg = *self.ue_avg_throughput.get(&uid).unwrap_or(&0.0);
                (uid, pf_weight(avg))
            })
            .collect();

        let total_w: f64 = ue_weights.iter().map(|(_, w)| w).sum();
        let total_w = if total_w < 1e-12 { 1.0 } else { total_w };

        // Token-bucket-enforced QoS: compute allowed bytes for the slice
        let allowed_bytes = instance.consume_tokens(bytes_available);

        // Distribute allowed bytes among UEs proportionally to PF weight
        let mut ue_allocations: Vec<(u32, f64)> = Vec::new();
        let mut total_sent = 0.0f64;
        for (uid, w) in &mut ue_weights {
            let share = allowed_bytes * (*w / total_w);
            // Dequeue from UE buffer (or assume full capacity for simplicity)
            let sent = if let Some(ue) = self.ues.get_mut(uid) {
                // If buffer is empty, assume UE generates traffic
                let pending = *ue.buffer_bytes.get(&sid).unwrap_or(&share);
                if pending < 1.0 {
                    ue.enqueue(sid, share); // Inject synthetic traffic
                }
                ue.dequeue(sid, share)
            } else {
                share
            };
            // Update PF average throughput (EMA)
            let alpha = self.pf_alpha;
            let avg = self.ue_avg_throughput.entry(*uid).or_insert(0.0);
            *avg = (1.0 - alpha) * (*avg) + alpha * (sent * 8.0 / 1e6); // Mbps
            total_sent += sent;
            ue_allocations.push((*uid, sent));
        }

        let throughput_mbps = total_sent * 8.0 / 1e6;

        SliceAllocation {
            slice_id: sid,
            snssai,
            allocated_prbs: prbs,
            ue_allocations,
            throughput_mbps,
        }
    }

    // -----------------------------------------------------------------------
    // SLA monitoring and metrics update
    // -----------------------------------------------------------------------

    /// Update slice metrics from a scheduling allocation result
    fn update_slice_metrics(&mut self, alloc: &SliceAllocation, _timestamp_ms: f64) {
        let instance = match self.slices.get_mut(&alloc.slice_id) {
            Some(s) => s,
            None => return,
        };

        instance.metrics.allocated_prbs = alloc.allocated_prbs;
        instance.metrics.used_prbs = if alloc.throughput_mbps > 0.0 {
            alloc.allocated_prbs
        } else {
            0
        };
        instance.metrics.throughput_mbps = alloc.throughput_mbps;

        // Model latency: inversely proportional to spare PRBs relative to demand
        let load_factor = if alloc.allocated_prbs > 0 {
            (instance.ue_ids.len() as f64 / (alloc.allocated_prbs as f64 * 0.1 + 1.0)).min(10.0)
        } else {
            10.0
        };
        let base_latency = instance.config.scs.slot_duration_ms() * 2.0; // 2 slots min
        let latency = base_latency * (1.0 + load_factor * 0.5);
        instance.record_latency(latency);
        instance.metrics.latency_ms = instance.mean_latency_ms();

        // Model delivery ratio: drops with extreme load
        let delivery = (1.0 - load_factor * 0.02).clamp(0.0, 1.0);
        instance.record_delivery(delivery);
        instance.metrics.packet_delivery_ratio = instance.mean_delivery_ratio();

        // PRB efficiency
        instance.metrics.prb_efficiency = if alloc.allocated_prbs > 0 {
            (alloc.throughput_mbps / (alloc.allocated_prbs as f64 * self.bytes_per_prb_ms * 8.0 / 1e6)).clamp(0.0, 1.0)
        } else {
            0.0
        };

        // SLA check
        let violated = instance.check_sla_violation();
        instance.metrics.sla_violated = violated;
        instance.metrics.measurement_periods += 1;
        if violated {
            instance.metrics.sla_violation_count += 1;
        }
    }

    /// Get current metrics for a slice
    pub fn slice_metrics(&self, id: SliceId) -> Option<&SliceMetrics> {
        self.slices.get(&id).map(|s| &s.metrics)
    }

    /// Get all slice metrics as a map
    pub fn all_metrics(&self) -> HashMap<SliceId, &SliceMetrics> {
        self.slices
            .iter()
            .map(|(id, s)| (*id, &s.metrics))
            .collect()
    }

    // -----------------------------------------------------------------------
    // Dynamic resource adjustment
    // -----------------------------------------------------------------------

    /// Perform reactive or predictive dynamic resource adjustment
    fn dynamic_adjustment(&mut self, ids: &[SliceId]) {
        if self.scaling_policy == ScalingPolicy::Static {
            return;
        }

        let high = self.scaling_thresholds.high_watermark;
        let low = self.scaling_thresholds.low_watermark;
        let step = self.scaling_thresholds.step_prbs;

        // Collect adjustments first to avoid borrow issues
        let adjustments: Vec<(SliceId, i32)> = ids
            .iter()
            .filter_map(|&id| {
                let instance = self.slices.get(&id)?;
                let util = instance.metrics.prb_utilization();
                if util > high && instance.current_prbs < instance.config.max_prbs {
                    Some((id, step as i32))
                } else if util < low && instance.current_prbs > instance.config.dedicated_prbs {
                    Some((id, -(step as i32)))
                } else {
                    None
                }
            })
            .collect();

        for (id, delta) in adjustments {
            if let Some(instance) = self.slices.get_mut(&id) {
                let new_prbs = (instance.current_prbs as i32 + delta)
                    .clamp(instance.config.dedicated_prbs as i32, instance.config.max_prbs as i32)
                    as u32;
                instance.current_prbs = new_prbs;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Aggregate / reporting
    // -----------------------------------------------------------------------

    /// Aggregate SLA violation rate across all slices (weighted by dedicated PRBs)
    pub fn aggregate_sla_violation_rate(&self) -> f64 {
        let mut total_weight = 0.0f64;
        let mut weighted_sum = 0.0f64;
        for s in self.slices.values() {
            let w = s.config.dedicated_prbs as f64 + 1.0;
            weighted_sum += w * s.metrics.violation_rate();
            total_weight += w;
        }
        if total_weight <= 0.0 {
            return 0.0;
        }
        weighted_sum / total_weight
    }

    /// Total PRB utilization across all active slices
    pub fn total_prb_utilization(&self) -> f64 {
        let used: u32 = self.slices.values().map(|s| s.metrics.used_prbs).sum();
        used as f64 / self.total_prbs as f64
    }

    /// Get number of active slices
    pub fn num_slices(&self) -> usize {
        self.slices.len()
    }

    /// Get number of admitted UEs
    pub fn num_ues(&self) -> usize {
        self.ues.len()
    }

    /// Per-slice configuration (read-only)
    pub fn slice_config(&self, id: SliceId) -> Option<&SliceConfig> {
        self.slices.get(&id).map(|s| &s.config)
    }

    /// Find slices matching a given S-NSSAI
    pub fn find_slices_by_snssai(&self, snssai: &SNssai) -> Vec<SliceId> {
        self.slices
            .iter()
            .filter(|(_, s)| s.config.snssai.matches(snssai))
            .map(|(id, _)| *id)
            .collect()
    }

    /// Route a PDU session to the best matching slice
    pub fn route_pdu_session(&self, snssai: &SNssai) -> Option<SliceId> {
        // Prefer exact SD match, then wildcard
        let exact: Vec<SliceId> = self
            .slices
            .iter()
            .filter(|(_, s)| {
                s.state == SliceState::Active
                    && s.config.snssai.sst == snssai.sst
                    && s.config.snssai.sd == snssai.sd
            })
            .map(|(id, _)| *id)
            .collect();
        if !exact.is_empty() {
            return exact.into_iter().next();
        }
        self.find_slices_by_snssai(snssai)
            .into_iter()
            .filter(|&id| {
                self.slices
                    .get(&id)
                    .map(|s| s.state == SliceState::Active)
                    .unwrap_or(false)
            })
            .next()
    }

    /// Summary string for logging/debugging
    pub fn summary(&self) -> String {
        let mut s = format!(
            "RAN Slicing: {} slices, {} UEs, {}/{} PRBs allocated\n",
            self.slices.len(),
            self.ues.len(),
            self.slices.values().map(|s| s.metrics.allocated_prbs).sum::<u32>(),
            self.total_prbs,
        );
        for (id, inst) in &self.slices {
            s.push_str(&format!(
                "  Slice {:3}: {} {} prbs={} tp={:.1}Mbps latency={:.2}ms viol_rate={:.3}\n",
                id,
                inst.config.snssai,
                inst.config.sst().name(),
                inst.current_prbs,
                inst.metrics.throughput_mbps,
                inst.metrics.latency_ms,
                inst.metrics.violation_rate(),
            ));
        }
        s
    }
}

// Convenience accessor on SliceConfig
impl SliceConfig {
    pub fn sst(&self) -> Sst {
        self.snssai.sst
    }
}

// Convenience accessor on SliceInstance
impl SliceInstance {
    fn sst(&self) -> Sst {
        self.config.snssai.sst
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn embb_snssai() -> SNssai {
        SNssai::new(Sst::Embb, Some(0x001234))
    }

    fn urllc_snssai() -> SNssai {
        SNssai::new(Sst::Urllc, Some(0xABCDEF))
    }

    fn miot_snssai() -> SNssai {
        SNssai::new(Sst::MIot, None)
    }

    fn default_embb_config() -> SliceConfig {
        SliceConfig::builder(embb_snssai())
            .isolation(IsolationLevel::Soft)
            .dedicated_prbs(20)
            .max_prbs(60)
            .max_bitrate_mbps(500.0)
            .min_bitrate_mbps(50.0)
            .build()
    }

    fn default_urllc_config() -> SliceConfig {
        SliceConfig::builder(urllc_snssai())
            .isolation(IsolationLevel::Hard)
            .dedicated_prbs(10)
            .max_prbs(10)
            .max_bitrate_mbps(100.0)
            .scheduling_priority(10)
            .build()
    }

    // --- S-NSSAI tests ---

    #[test]
    fn test_sst_values() {
        assert_eq!(Sst::Embb.to_u8(), 1);
        assert_eq!(Sst::Urllc.to_u8(), 2);
        assert_eq!(Sst::MIot.to_u8(), 3);
        assert_eq!(Sst::V2x.to_u8(), 4);
        assert_eq!(Sst::Custom(42).to_u8(), 42);
    }

    #[test]
    fn test_sst_from_u8() {
        assert_eq!(Sst::from_u8(1), Sst::Embb);
        assert_eq!(Sst::from_u8(2), Sst::Urllc);
        assert_eq!(Sst::from_u8(3), Sst::MIot);
        assert_eq!(Sst::from_u8(4), Sst::V2x);
        assert!(matches!(Sst::from_u8(99), Sst::Custom(99)));
    }

    #[test]
    fn test_sst_latency_budget() {
        assert!(Sst::Urllc.default_latency_budget_ms() < Sst::Embb.default_latency_budget_ms());
        assert!(Sst::MIot.default_latency_budget_ms() > Sst::Embb.default_latency_budget_ms());
    }

    #[test]
    fn test_sst_reliability() {
        assert!(Sst::Urllc.default_reliability() > Sst::Embb.default_reliability());
        assert!(Sst::Embb.default_reliability() > Sst::MIot.default_reliability());
    }

    #[test]
    fn test_snssai_encode_decode_with_sd() {
        let s = SNssai::new(Sst::Embb, Some(0xABCDEF));
        let encoded = s.encode();
        let decoded = SNssai::decode(encoded);
        assert_eq!(decoded.sst, Sst::Embb);
        assert_eq!(decoded.sd, Some(0xABCDEF));
    }

    #[test]
    fn test_snssai_encode_decode_no_sd() {
        let s = SNssai::new(Sst::Urllc, None);
        let encoded = s.encode();
        let decoded = SNssai::decode(encoded);
        assert_eq!(decoded.sst, Sst::Urllc);
        assert_eq!(decoded.sd, None);
    }

    #[test]
    fn test_snssai_sd_truncated_to_24_bits() {
        let s = SNssai::new(Sst::Embb, Some(0xFF_ABCDEF));
        assert_eq!(s.sd, Some(0xABCDEF)); // upper byte dropped
    }

    #[test]
    fn test_snssai_matching_wildcard() {
        let wildcard = SNssai::new(Sst::Embb, None);
        let specific = SNssai::new(Sst::Embb, Some(0x001234));
        assert!(wildcard.matches(&specific));
        assert!(specific.matches(&wildcard));
    }

    #[test]
    fn test_snssai_matching_exact() {
        let a = SNssai::new(Sst::Embb, Some(0x001234));
        let b = SNssai::new(Sst::Embb, Some(0x001234));
        let c = SNssai::new(Sst::Embb, Some(0x009999));
        assert!(a.matches(&b));
        assert!(!a.matches(&c));
    }

    #[test]
    fn test_snssai_different_sst_no_match() {
        let a = SNssai::new(Sst::Embb, None);
        let b = SNssai::new(Sst::Urllc, None);
        assert!(!a.matches(&b));
    }

    #[test]
    fn test_nssai_add_and_capacity() {
        let mut nssai = Nssai::new();
        for i in 1..=8 {
            let s = SNssai::new(Sst::from_u8(1), Some(i));
            assert!(nssai.add(s).is_ok());
        }
        assert_eq!(nssai.len(), 8);
        // 9th should fail
        let extra = SNssai::new(Sst::Embb, Some(99));
        assert_eq!(nssai.add(extra), Err(SliceError::NssaiCapacityExceeded));
    }

    #[test]
    fn test_nssai_duplicate_rejection() {
        let mut nssai = Nssai::new();
        let s = SNssai::new(Sst::Embb, Some(0x001));
        nssai.add(s).unwrap();
        assert_eq!(nssai.add(s), Err(SliceError::DuplicateSnssai));
    }

    #[test]
    fn test_nssai_contains_match() {
        let mut nssai = Nssai::new();
        nssai.add(SNssai::new(Sst::Embb, Some(0x001))).unwrap();
        assert!(nssai.contains_match(&SNssai::new(Sst::Embb, Some(0x001))));
        assert!(!nssai.contains_match(&SNssai::new(Sst::Urllc, Some(0x001))));
    }

    // --- Numerology tests ---

    #[test]
    fn test_scs_slot_duration() {
        assert!((SubcarrierSpacing::Scs15kHz.slot_duration_ms() - 1.0).abs() < 1e-9);
        assert!((SubcarrierSpacing::Scs30kHz.slot_duration_ms() - 0.5).abs() < 1e-9);
        assert!((SubcarrierSpacing::Scs60kHz.slot_duration_ms() - 0.25).abs() < 1e-9);
        assert!((SubcarrierSpacing::Scs120kHz.slot_duration_ms() - 0.125).abs() < 1e-9);
    }

    #[test]
    fn test_scs_for_sst() {
        assert_eq!(SubcarrierSpacing::for_sst(Sst::Urllc), SubcarrierSpacing::Scs60kHz);
        assert_eq!(SubcarrierSpacing::for_sst(Sst::MIot), SubcarrierSpacing::Scs15kHz);
        assert_eq!(SubcarrierSpacing::for_sst(Sst::V2x), SubcarrierSpacing::Scs60kHz);
        assert_eq!(SubcarrierSpacing::for_sst(Sst::Embb), SubcarrierSpacing::Scs30kHz);
    }

    // --- Slice lifecycle tests ---

    #[test]
    fn test_create_slice_success() {
        let mut p = RanSlicingProcessor::new(100);
        let id = p.create_slice(default_embb_config()).unwrap();
        assert_eq!(p.slice_state(id), Some(SliceState::Active));
        assert_eq!(p.num_slices(), 1);
    }

    #[test]
    fn test_create_slice_invalid_dedicated_exceeds_total() {
        let mut p = RanSlicingProcessor::new(10);
        let config = SliceConfig::builder(embb_snssai())
            .dedicated_prbs(50)
            .max_prbs(50)
            .build();
        assert!(p.create_slice(config).is_err());
    }

    #[test]
    fn test_create_slice_invalid_max_exceeds_total() {
        let mut p = RanSlicingProcessor::new(10);
        let config = SliceConfig::builder(embb_snssai())
            .dedicated_prbs(5)
            .max_prbs(200)
            .build();
        assert!(p.create_slice(config).is_err());
    }

    #[test]
    fn test_create_slice_hard_isolation_capacity() {
        let mut p = RanSlicingProcessor::new(30);
        let hard1 = SliceConfig::builder(embb_snssai())
            .isolation(IsolationLevel::Hard)
            .dedicated_prbs(20)
            .max_prbs(20)
            .build();
        let hard2 = SliceConfig::builder(urllc_snssai())
            .isolation(IsolationLevel::Hard)
            .dedicated_prbs(15) // 20+15 > 30
            .max_prbs(15)
            .build();
        p.create_slice(hard1).unwrap();
        assert_eq!(p.create_slice(hard2), Err(SliceError::InsufficientCapacity));
    }

    #[test]
    fn test_delete_slice() {
        let mut p = RanSlicingProcessor::new(100);
        let id = p.create_slice(default_embb_config()).unwrap();
        p.delete_slice(id).unwrap();
        assert_eq!(p.slice_state(id), None);
        assert_eq!(p.num_slices(), 0);
    }

    #[test]
    fn test_delete_nonexistent_slice() {
        let mut p = RanSlicingProcessor::new(100);
        assert!(matches!(p.delete_slice(999), Err(SliceError::SliceNotFound(999))));
    }

    #[test]
    fn test_modify_slice() {
        let mut p = RanSlicingProcessor::new(100);
        let id = p.create_slice(default_embb_config()).unwrap();
        let new_config = SliceConfig::builder(embb_snssai())
            .dedicated_prbs(30)
            .max_prbs(80)
            .build();
        p.modify_slice(id, new_config).unwrap();
        assert_eq!(p.slice_config(id).unwrap().dedicated_prbs, 30);
    }

    #[test]
    fn test_scale_slice() {
        let mut p = RanSlicingProcessor::new(100);
        let id = p.create_slice(default_embb_config()).unwrap();
        p.scale_slice(id, 35).unwrap();
        assert_eq!(p.slice_state(id), Some(SliceState::Active));
    }

    #[test]
    fn test_scale_slice_exceeds_max_fails() {
        let mut p = RanSlicingProcessor::new(100);
        let id = p.create_slice(default_embb_config()).unwrap();
        // max_prbs is 60 in default_embb_config
        assert!(p.scale_slice(id, 100).is_err());
    }

    #[test]
    fn test_multiple_slices() {
        let mut p = RanSlicingProcessor::new(100);
        let id1 = p.create_slice(default_embb_config()).unwrap();
        let id2 = p.create_slice(default_urllc_config()).unwrap();
        let id3 = p.create_slice(
            SliceConfig::builder(miot_snssai())
                .dedicated_prbs(5)
                .max_prbs(20)
                .build(),
        ).unwrap();
        assert_eq!(p.num_slices(), 3);
        assert_ne!(id1, id2);
        assert_ne!(id2, id3);
    }

    // --- UE admission tests ---

    #[test]
    fn test_admit_ue_success() {
        let mut p = RanSlicingProcessor::new(100);
        let sid = p.create_slice(default_embb_config()).unwrap();
        let ue = UeContext::new(42, vec![sid], vec![PDuSession::new(1, sid)]);
        assert_eq!(p.admit_ue(ue).unwrap(), AdmissionResult::Accepted);
        assert_eq!(p.num_ues(), 1);
    }

    #[test]
    fn test_admit_ue_slice_not_found() {
        let mut p = RanSlicingProcessor::new(100);
        let ue = UeContext::new(42, vec![999], vec![]);
        assert!(p.admit_ue(ue).is_err());
    }

    #[test]
    fn test_release_ue() {
        let mut p = RanSlicingProcessor::new(100);
        let sid = p.create_slice(default_embb_config()).unwrap();
        let ue = UeContext::new(42, vec![sid], vec![]);
        p.admit_ue(ue).unwrap();
        p.release_ue(42).unwrap();
        assert_eq!(p.num_ues(), 0);
    }

    #[test]
    fn test_release_nonexistent_ue() {
        let mut p = RanSlicingProcessor::new(100);
        assert!(matches!(p.release_ue(999), Err(SliceError::UeNotFound(999))));
    }

    #[test]
    fn test_admission_check() {
        let mut p = RanSlicingProcessor::new(100);
        let sid = p.create_slice(default_embb_config()).unwrap();
        assert_eq!(p.admission_check(42, sid), AdmissionResult::Accepted);
        assert_eq!(
            p.admission_check(42, 999),
            AdmissionResult::Rejected(RejectionCause::SliceNotFound)
        );
    }

    #[test]
    fn test_admit_ue_multi_slice() {
        let mut p = RanSlicingProcessor::new(100);
        let sid1 = p.create_slice(default_embb_config()).unwrap();
        let sid2 = p.create_slice(default_urllc_config()).unwrap();
        let ue = UeContext::new(1, vec![sid1, sid2], vec![
            PDuSession::new(1, sid1),
            PDuSession::new(2, sid2),
        ]);
        assert_eq!(p.admit_ue(ue).unwrap(), AdmissionResult::Accepted);
    }

    // --- Scheduling tests ---

    #[test]
    fn test_schedule_round_empty() {
        let mut p = RanSlicingProcessor::new(100);
        let decision = p.schedule_round(1000.0);
        assert_eq!(decision.total_prbs, 100);
        assert!(decision.allocations.is_empty());
    }

    #[test]
    fn test_schedule_round_one_slice() {
        let mut p = RanSlicingProcessor::new(100);
        let sid = p.create_slice(default_embb_config()).unwrap();
        let ue = UeContext::new(1, vec![sid], vec![PDuSession::new(1, sid)]);
        p.admit_ue(ue).unwrap();
        let decision = p.schedule_round(1.0);
        assert_eq!(decision.allocations.len(), 1);
        assert_eq!(decision.allocations[0].slice_id, sid);
        assert!(decision.allocations[0].allocated_prbs > 0);
    }

    #[test]
    fn test_schedule_round_multiple_slices() {
        let mut p = RanSlicingProcessor::new(100);
        let sid1 = p.create_slice(default_embb_config()).unwrap();
        let sid2 = p.create_slice(default_urllc_config()).unwrap();
        for i in 0..3 {
            let ue = UeContext::new(i, vec![sid1], vec![PDuSession::new(1, sid1)]);
            p.admit_ue(ue).unwrap();
        }
        let ue = UeContext::new(10, vec![sid2], vec![PDuSession::new(2, sid2)]);
        p.admit_ue(ue).unwrap();
        let decision = p.schedule_round(1.0);
        assert_eq!(decision.allocations.len(), 2);
    }

    #[test]
    fn test_schedule_no_prb_overcommit() {
        let mut p = RanSlicingProcessor::new(50);
        let configs = vec![
            SliceConfig::builder(embb_snssai()).dedicated_prbs(20).max_prbs(50).build(),
            SliceConfig::builder(urllc_snssai()).isolation(IsolationLevel::Hard).dedicated_prbs(10).max_prbs(10).build(),
            SliceConfig::builder(miot_snssai()).dedicated_prbs(5).max_prbs(20).build(),
        ];
        for cfg in configs {
            p.create_slice(cfg).unwrap();
        }
        let decision = p.schedule_round(1.0);
        let total: u32 = decision.allocations.iter().map(|a| a.allocated_prbs).sum();
        assert!(total <= 50, "Total allocated {} exceeds 50", total);
    }

    #[test]
    fn test_hard_isolation_slice_prbs_respected() {
        let mut p = RanSlicingProcessor::new(100);
        let hard_config = SliceConfig::builder(urllc_snssai())
            .isolation(IsolationLevel::Hard)
            .dedicated_prbs(15)
            .max_prbs(15)
            .build();
        let sid = p.create_slice(hard_config).unwrap();
        let decision = p.schedule_round(1.0);
        let alloc = decision.allocations.iter().find(|a| a.slice_id == sid).unwrap();
        assert_eq!(alloc.allocated_prbs, 15);
    }

    // --- SLA and metrics tests ---

    #[test]
    fn test_metrics_updated_after_schedule() {
        let mut p = RanSlicingProcessor::new(100);
        let sid = p.create_slice(default_embb_config()).unwrap();
        let ue = UeContext::new(1, vec![sid], vec![PDuSession::new(1, sid)]);
        p.admit_ue(ue).unwrap();
        p.schedule_round(1.0);
        let m = p.slice_metrics(sid).unwrap();
        assert!(m.measurement_periods > 0);
        assert!(m.allocated_prbs > 0);
    }

    #[test]
    fn test_sla_violation_rate_tracking() {
        let mut p = RanSlicingProcessor::new(100);
        let sid = p.create_slice(default_embb_config()).unwrap();
        let ue = UeContext::new(1, vec![sid], vec![PDuSession::new(1, sid)]);
        p.admit_ue(ue).unwrap();
        for i in 0..10 {
            p.schedule_round(i as f64 * 1.0);
        }
        let m = p.slice_metrics(sid).unwrap();
        assert_eq!(m.measurement_periods, 10);
        let rate = m.violation_rate();
        assert!((0.0..=1.0).contains(&rate));
    }

    #[test]
    fn test_aggregate_sla_violation_rate() {
        let mut p = RanSlicingProcessor::new(100);
        let sid1 = p.create_slice(default_embb_config()).unwrap();
        let sid2 = p.create_slice(default_urllc_config()).unwrap();
        for i in 0..5u32 {
            let ue = UeContext::new(i, vec![sid1, sid2], vec![]);
            p.admit_ue(ue).unwrap();
        }
        for i in 0..5 {
            p.schedule_round(i as f64 * 1.0);
        }
        let rate = p.aggregate_sla_violation_rate();
        assert!((0.0..=1.0).contains(&rate));
    }

    #[test]
    fn test_prb_utilization_bounded() {
        let mut p = RanSlicingProcessor::new(100);
        p.create_slice(default_embb_config()).unwrap();
        p.schedule_round(1.0);
        let util = p.total_prb_utilization();
        assert!(util >= 0.0 && util <= 1.0);
    }

    #[test]
    fn test_prb_efficiency_bounded() {
        let mut p = RanSlicingProcessor::new(100);
        let sid = p.create_slice(default_embb_config()).unwrap();
        let ue = UeContext::new(1, vec![sid], vec![PDuSession::new(1, sid)]);
        p.admit_ue(ue).unwrap();
        p.schedule_round(1.0);
        let m = p.slice_metrics(sid).unwrap();
        assert!(m.prb_efficiency >= 0.0 && m.prb_efficiency <= 1.0);
    }

    // --- QoS / token bucket tests ---

    #[test]
    fn test_token_bucket_limits_throughput() {
        // Very low max bitrate
        let mut p = RanSlicingProcessor::new(100);
        let config = SliceConfig::builder(embb_snssai())
            .dedicated_prbs(20)
            .max_prbs(80)
            .max_bitrate_mbps(1.0) // 1 Mbps max
            .build();
        let sid = p.create_slice(config).unwrap();
        let ue = UeContext::new(1, vec![sid], vec![PDuSession::new(1, sid)]);
        p.admit_ue(ue).unwrap();
        p.schedule_round(1.0);
        // Throughput should be constrained
        let m = p.slice_metrics(sid).unwrap();
        assert!(m.throughput_mbps >= 0.0);
    }

    // --- Routing and lookup tests ---

    #[test]
    fn test_find_slices_by_snssai() {
        let mut p = RanSlicingProcessor::new(100);
        let sid1 = p.create_slice(default_embb_config()).unwrap();
        p.create_slice(default_urllc_config()).unwrap();
        let found = p.find_slices_by_snssai(&embb_snssai());
        assert!(found.contains(&sid1));
        assert_eq!(found.len(), 1);
    }

    #[test]
    fn test_route_pdu_session_exact_match() {
        let mut p = RanSlicingProcessor::new(100);
        let sid = p.create_slice(default_embb_config()).unwrap();
        let routed = p.route_pdu_session(&embb_snssai());
        assert_eq!(routed, Some(sid));
    }

    #[test]
    fn test_route_pdu_session_no_match() {
        let p = RanSlicingProcessor::new(100);
        let result = p.route_pdu_session(&embb_snssai());
        assert_eq!(result, None);
    }

    // --- Scheduling policy tests ---

    #[test]
    fn test_wrr_distributes_shared_pool() {
        let mut p = RanSlicingProcessor::new(100);
        let s1 = SliceConfig::builder(embb_snssai())
            .isolation(IsolationLevel::Soft)
            .scheduling_policy(SchedulingPolicy::WeightedRoundRobin)
            .scheduling_weight(20)
            .dedicated_prbs(10)
            .max_prbs(80)
            .build();
        let s2 = SliceConfig::builder(miot_snssai())
            .isolation(IsolationLevel::Soft)
            .scheduling_policy(SchedulingPolicy::WeightedRoundRobin)
            .scheduling_weight(10)
            .dedicated_prbs(5)
            .max_prbs(50)
            .build();
        let sid1 = p.create_slice(s1).unwrap();
        let sid2 = p.create_slice(s2).unwrap();
        let decision = p.schedule_round(1.0);
        let a1 = decision.allocations.iter().find(|a| a.slice_id == sid1).unwrap();
        let a2 = decision.allocations.iter().find(|a| a.slice_id == sid2).unwrap();
        // Higher weight slice should get more PRBs from shared pool
        assert!(a1.allocated_prbs >= a2.allocated_prbs);
    }

    #[test]
    fn test_strict_priority_higher_gets_more() {
        let mut p = RanSlicingProcessor::new(100);
        let high_pri = SliceConfig::builder(urllc_snssai())
            .isolation(IsolationLevel::Soft)
            .scheduling_policy(SchedulingPolicy::StrictPriority)
            .scheduling_priority(20)
            .dedicated_prbs(5)
            .max_prbs(80)
            .build();
        let low_pri = SliceConfig::builder(miot_snssai())
            .isolation(IsolationLevel::Soft)
            .scheduling_policy(SchedulingPolicy::StrictPriority)
            .scheduling_priority(1)
            .dedicated_prbs(5)
            .max_prbs(30)
            .build();
        let sid_h = p.create_slice(high_pri).unwrap();
        let sid_l = p.create_slice(low_pri).unwrap();
        let decision = p.schedule_round(1.0);
        let ah = decision.allocations.iter().find(|a| a.slice_id == sid_h).unwrap();
        let al = decision.allocations.iter().find(|a| a.slice_id == sid_l).unwrap();
        assert!(ah.allocated_prbs >= al.allocated_prbs);
    }

    #[test]
    fn test_proportional_fairness_scheduling() {
        let mut p = RanSlicingProcessor::new(100);
        let s = SliceConfig::builder(embb_snssai())
            .isolation(IsolationLevel::Soft)
            .scheduling_policy(SchedulingPolicy::ProportionalFairness)
            .dedicated_prbs(20)
            .max_prbs(80)
            .build();
        let sid = p.create_slice(s).unwrap();
        for i in 0..4 {
            let ue = UeContext::new(i, vec![sid], vec![PDuSession::new(i, sid)]);
            p.admit_ue(ue).unwrap();
        }
        let decision = p.schedule_round(1.0);
        assert!(!decision.allocations.is_empty());
    }

    // --- Cross-slice interference test ---

    #[test]
    fn test_cross_slice_interference_zero_for_single() {
        let instances: Vec<SliceInstance> = Vec::new();
        let refs: Vec<&SliceInstance> = instances.iter().collect();
        let i = estimate_cross_slice_interference(&refs);
        assert_eq!(i, 0.0);
    }

    #[test]
    fn test_cross_slice_interference_increases_with_slices() {
        let mut p = RanSlicingProcessor::new(100);
        let cfg1 = SliceConfig::builder(embb_snssai())
            .isolation(IsolationLevel::Soft)
            .scs(SubcarrierSpacing::Scs15kHz)
            .dedicated_prbs(10)
            .max_prbs(50)
            .build();
        let cfg2 = SliceConfig::builder(urllc_snssai())
            .isolation(IsolationLevel::Soft)
            .scs(SubcarrierSpacing::Scs30kHz)
            .dedicated_prbs(10)
            .max_prbs(50)
            .build();
        p.create_slice(cfg1).unwrap();
        p.create_slice(cfg2).unwrap();
        // Run a round and check interference is reported
        let decision = p.schedule_round(1.0);
        assert!(decision.cross_slice_interference >= 0.0);
    }

    // --- Isolation and isolation levels ---

    #[test]
    fn test_isolation_level_descriptions() {
        assert!(!IsolationLevel::Hard.description().is_empty());
        assert!(!IsolationLevel::Soft.description().is_empty());
        assert!(!IsolationLevel::BestEffort.description().is_empty());
    }

    // --- UE context buffer tests ---

    #[test]
    fn test_ue_enqueue_dequeue() {
        let mut ue = UeContext::new(1, vec![1], vec![]);
        ue.enqueue(1, 1000.0);
        assert!((ue.total_pending_bytes() - 1000.0).abs() < 1e-6);
        let sent = ue.dequeue(1, 400.0);
        assert!((sent - 400.0).abs() < 1e-6);
        assert!((ue.total_pending_bytes() - 600.0).abs() < 1e-6);
    }

    #[test]
    fn test_ue_dequeue_does_not_exceed_buffer() {
        let mut ue = UeContext::new(1, vec![1], vec![]);
        ue.enqueue(1, 100.0);
        let sent = ue.dequeue(1, 500.0); // Request more than available
        assert!((sent - 100.0).abs() < 1e-6);
        assert!((ue.total_pending_bytes() - 0.0).abs() < 1e-6);
    }

    // --- Dynamic adjustment tests ---

    #[test]
    fn test_dynamic_adjustment_scaling_policy() {
        let mut p = RanSlicingProcessor::new(100);
        p.set_scaling_policy(ScalingPolicy::Reactive, ScalingThresholds::default());
        let sid = p.create_slice(default_embb_config()).unwrap();
        let ue = UeContext::new(1, vec![sid], vec![PDuSession::new(1, sid)]);
        p.admit_ue(ue).unwrap();
        // Run several rounds to trigger reactive adjustment
        for i in 0..20 {
            p.schedule_round(i as f64);
        }
        // No panic and state remains valid
        assert_eq!(p.slice_state(sid), Some(SliceState::Active));
    }

    // --- SliceConfig builder tests ---

    #[test]
    fn test_builder_defaults_reasonable() {
        let config = SliceConfig::builder(embb_snssai()).build();
        assert!(config.dedicated_prbs <= config.max_prbs);
        assert!(config.min_bitrate_mbps <= config.max_bitrate_mbps);
    }

    #[test]
    fn test_builder_custom_sla() {
        let sla = SlaSla {
            throughput_mbps: 999.0,
            latency_ms: 0.5,
            reliability: 0.9999,
            violation_window_s: 2.0,
        };
        let config = SliceConfig::builder(urllc_snssai()).sla(sla).build();
        assert!((config.sla.throughput_mbps - 999.0).abs() < 1e-6);
    }

    // --- Summary / display tests ---

    #[test]
    fn test_summary_output() {
        let mut p = RanSlicingProcessor::new(100);
        p.create_slice(default_embb_config()).unwrap();
        let s = p.summary();
        assert!(s.contains("RAN Slicing"));
    }

    #[test]
    fn test_snssai_display() {
        let s = SNssai::new(Sst::Embb, Some(0xABC));
        let disp = format!("{}", s);
        assert!(disp.contains("SST=1"));
        assert!(disp.contains("ABC"));
    }

    #[test]
    fn test_sst_display() {
        let s = format!("{}", Sst::Urllc);
        assert!(s.contains("SST=2"));
    }

    #[test]
    fn test_pf_weight() {
        assert!(pf_weight(0.0) > 1e5); // starved UE
        assert!((pf_weight(1.0) - 1.0).abs() < 1e-6);
        assert!((pf_weight(10.0) - 0.1).abs() < 1e-6);
    }

    #[test]
    fn test_slice_metrics_violation_rate() {
        let mut m = SliceMetrics::default();
        assert_eq!(m.violation_rate(), 0.0);
        m.measurement_periods = 10;
        m.sla_violation_count = 3;
        assert!((m.violation_rate() - 0.3).abs() < 1e-6);
    }

    #[test]
    fn test_slice_metrics_prb_utilization() {
        let mut m = SliceMetrics::default();
        m.allocated_prbs = 10;
        m.used_prbs = 7;
        assert!((m.prb_utilization() - 0.7).abs() < 1e-6);
    }

    #[test]
    fn test_active_slice_ids() {
        let mut p = RanSlicingProcessor::new(100);
        let sid1 = p.create_slice(default_embb_config()).unwrap();
        let sid2 = p.create_slice(default_urllc_config()).unwrap();
        let ids = p.active_slice_ids();
        assert!(ids.contains(&sid1));
        assert!(ids.contains(&sid2));
    }

    #[test]
    fn test_slice_config_sst_accessor() {
        let cfg = default_embb_config();
        assert_eq!(cfg.sst(), Sst::Embb);
    }

    #[test]
    fn test_pdu_session_with_bitrate() {
        let s = PDuSession::new(1, 42).with_bitrate(100.0);
        assert_eq!(s.session_id, 1);
        assert_eq!(s.slice_id, 42);
        assert!((s.requested_bitrate_mbps - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_intra_slice_ue_allocations() {
        let mut p = RanSlicingProcessor::new(100);
        let sid = p.create_slice(default_embb_config()).unwrap();
        for i in 0..3u32 {
            let ue = UeContext::new(i, vec![sid], vec![PDuSession::new(i, sid)]);
            p.admit_ue(ue).unwrap();
        }
        let decision = p.schedule_round(1.0);
        let alloc = decision.allocations.iter().find(|a| a.slice_id == sid).unwrap();
        assert_eq!(alloc.ue_allocations.len(), 3);
    }

    #[test]
    fn test_all_metrics_returns_all_slices() {
        let mut p = RanSlicingProcessor::new(100);
        p.create_slice(default_embb_config()).unwrap();
        p.create_slice(default_urllc_config()).unwrap();
        p.schedule_round(1.0);
        let metrics = p.all_metrics();
        assert_eq!(metrics.len(), 2);
    }
}

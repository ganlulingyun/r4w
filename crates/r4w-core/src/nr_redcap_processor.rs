//! 5G NR RedCap (Reduced Capability) Device Processor
//!
//! Implements the 5G NR RedCap UE processing per 3GPP TS 38.101 / TS 38.306 Release 17.
//!
//! RedCap (Reduced Capability) is a 5G NR feature introduced in Release 17 that
//! targets IoT and wearable devices requiring less complexity than full NR UEs.
//!
//! # Key Reductions vs Full NR UE
//! - Maximum bandwidth: 20 MHz FR1 (vs 100 MHz), 100 MHz FR2 (vs 400 MHz)
//! - MIMO: 1 RX antenna (Release 17), single DL layer
//! - Modulation: max 64-QAM DL/UL (vs 256-QAM DL for normal NR)
//! - Half-duplex FDD operation supported
//! - Extended DRX cycles up to 10.24 s
//! - Relaxed RRM measurements
//!
//! # Target Devices
//! - Wearables (smartwatches, fitness trackers)
//! - Industrial IoT sensors
//! - Video surveillance cameras
//! - Smart metering devices
//!
//! # Standards References
//! - 3GPP TS 38.101-1 Rel-17: FR1 UE radio transmission and reception
//! - 3GPP TS 38.306 Rel-17: NR UE radio access capabilities
//! - 3GPP TR 38.875: Study on reduced capability NR devices

use std::collections::VecDeque;

// ─────────────────────────────────────────────────────────────────
// Constants per 3GPP TS 38.101-1 / TS 38.306 Release 17
// ─────────────────────────────────────────────────────────────────

/// FR1 maximum RedCap bandwidth in Hz (20 MHz, 3GPP TS 38.101-1 Table 5.3.5-1)
pub const FR1_REDCAP_MAX_BW_HZ: u64 = 20_000_000;

/// FR2 maximum RedCap bandwidth in Hz (100 MHz, 3GPP TS 38.101-1 Table 5.3.5-2)
pub const FR2_REDCAP_MAX_BW_HZ: u64 = 100_000_000;

/// Maximum number of DL MIMO layers for Release 17 RedCap
pub const REDCAP_MAX_DL_LAYERS: usize = 1;

/// Maximum number of UL MIMO layers for Release 17 RedCap
pub const REDCAP_MAX_UL_LAYERS: usize = 1;

/// Maximum DL modulation order (64-QAM = order 6)
pub const REDCAP_MAX_DL_MOD_ORDER: u8 = 6;

/// Maximum UL modulation order (64-QAM = order 6)
pub const REDCAP_MAX_UL_MOD_ORDER: u8 = 6;

/// Normal NR UE maximum DL modulation order (256-QAM = order 8)
pub const NORMAL_NR_MAX_DL_MOD_ORDER: u8 = 8;

/// Maximum PRBs for FR1 with 15 kHz SCS at 20 MHz (TS 38.101-1 Table 5.3.2-1)
pub const FR1_20MHZ_15KHZ_MAX_PRBS: u32 = 106;

/// Maximum PRBs for FR1 with 30 kHz SCS at 20 MHz (TS 38.101-1 Table 5.3.2-1)
pub const FR1_20MHZ_30KHZ_MAX_PRBS: u32 = 51;

/// Maximum PRBs for FR2 with 120 kHz SCS at 100 MHz (TS 38.101-1 Table 5.3.2-2)
pub const FR2_100MHZ_120KHZ_MAX_PRBS: u32 = 66;

/// DRX maximum cycle in milliseconds (10.24 s = 10240 ms per TS 38.321)
pub const MAX_DRX_CYCLE_MS: u32 = 10_240;

/// Subframe duration in milliseconds (1 ms per NR numerology)
pub const SUBFRAME_DURATION_MS: f64 = 1.0;

/// NR slot duration at 15 kHz SCS in microseconds (1 ms / 1 slot = 1000 µs)
pub const SLOT_DURATION_15KHZ_US: f64 = 1000.0;

/// NR slot duration at 30 kHz SCS in microseconds (0.5 ms)
pub const SLOT_DURATION_30KHZ_US: f64 = 500.0;

/// NR slot duration at 120 kHz SCS in microseconds (0.125 ms)
pub const SLOT_DURATION_120KHZ_US: f64 = 125.0;

/// Minimum guard period slots for half-duplex FDD (1 slot)
pub const HD_FDD_MIN_GUARD_SLOTS: u32 = 1;

/// OFDM subcarrier spacing 15 kHz in Hz
pub const SCS_15KHZ_HZ: u32 = 15_000;

/// OFDM subcarrier spacing 30 kHz in Hz
pub const SCS_30KHZ_HZ: u32 = 30_000;

/// OFDM subcarrier spacing 60 kHz in Hz
pub const SCS_60KHZ_HZ: u32 = 60_000;

/// OFDM subcarrier spacing 120 kHz in Hz
pub const SCS_120KHZ_HZ: u32 = 120_000;

/// Number of OFDM symbols per slot (normal CP)
pub const SYMBOLS_PER_SLOT: u32 = 14;

/// Subcarriers per PRB
pub const SUBCARRIERS_PER_PRB: u32 = 12;

/// RedCap peak DL data rate approximation (Mbps) for 20 MHz, 64-QAM, 1 layer
/// = 0.948 * (6 bits/sym) * (1 layer) * (100 PRBs * 12 SC) * (12 sym/slot) * (2000 slots/s)
/// ≈ 85 Mbps (approximate, ignoring overheads for simplicity here)
pub const REDCAP_APPROX_PEAK_DL_MBPS: f64 = 85.0;

// ─────────────────────────────────────────────────────────────────
// Enumerations
// ─────────────────────────────────────────────────────────────────

/// Frequency Range (FR) per 3GPP TS 38.101
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrequencyRange {
    /// FR1: Sub-6 GHz (410 MHz – 7125 MHz)
    Fr1,
    /// FR2: mmWave (24250 MHz – 52600 MHz)
    Fr2,
}

/// Subcarrier Spacing (SCS) numerology
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubcarrierSpacing {
    Scs15kHz,
    Scs30kHz,
    Scs60kHz,
    Scs120kHz,
    Scs240kHz,
}

impl SubcarrierSpacing {
    /// Returns spacing in Hz
    pub fn hz(&self) -> u32 {
        match self {
            SubcarrierSpacing::Scs15kHz => 15_000,
            SubcarrierSpacing::Scs30kHz => 30_000,
            SubcarrierSpacing::Scs60kHz => 60_000,
            SubcarrierSpacing::Scs120kHz => 120_000,
            SubcarrierSpacing::Scs240kHz => 240_000,
        }
    }

    /// Returns slot duration in microseconds.
    /// At numerology µ there are 2^µ slots per 1 ms subframe,
    /// so slot_duration = 1000 µs / 2^µ = 1000 / slots_per_subframe.
    pub fn slot_duration_us(&self) -> f64 {
        1000.0 / self.slots_per_subframe() as f64
    }

    /// Returns slots per subframe
    pub fn slots_per_subframe(&self) -> u32 {
        match self {
            SubcarrierSpacing::Scs15kHz => 1,
            SubcarrierSpacing::Scs30kHz => 2,
            SubcarrierSpacing::Scs60kHz => 4,
            SubcarrierSpacing::Scs120kHz => 8,
            SubcarrierSpacing::Scs240kHz => 16,
        }
    }
}

/// DL modulation schemes available to RedCap UE
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Modulation {
    Qpsk,
    Qam16,
    Qam64,
    /// Not available for Release 17 RedCap
    Qam256,
}

impl Modulation {
    /// Returns bits per symbol (modulation order)
    pub fn bits_per_symbol(&self) -> u32 {
        match self {
            Modulation::Qpsk => 2,
            Modulation::Qam16 => 4,
            Modulation::Qam64 => 6,
            Modulation::Qam256 => 8,
        }
    }

    /// Returns modulation order exponent (log2 of modulation cardinality)
    pub fn order(&self) -> u8 {
        match self {
            Modulation::Qpsk => 2,
            Modulation::Qam16 => 4,
            Modulation::Qam64 => 6,
            Modulation::Qam256 => 8,
        }
    }

    /// Returns true if this modulation is allowed for RedCap UE
    pub fn is_redcap_allowed(&self) -> bool {
        matches!(self, Modulation::Qpsk | Modulation::Qam16 | Modulation::Qam64)
    }
}

/// Duplex mode for RedCap UE
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DuplexMode {
    /// Full Duplex FDD (simultaneous TX/RX)
    FullDuplexFdd,
    /// Half Duplex FDD (simplified RF frontend, no simultaneous TX/RX)
    HalfDuplexFdd,
    /// TDD (Time Division Duplex)
    Tdd,
}

/// BWP (Bandwidth Part) type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BwpType {
    InitialDl,
    InitialUl,
    DedicatedDl,
    DedicatedUl,
    /// Separate initial UL BWP for RedCap (avoids blocking normal NR UE access)
    RedcapInitialUl,
}

/// DRX (Discontinuous Reception) state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrxState {
    Active,
    ShortCycle,
    LongCycle,
    Inactive,
}

/// RedCap UE state machine
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UeState {
    /// UE is off / not yet registered
    Idle,
    /// UE performing initial access
    InitialAccess,
    /// UE in RRC_IDLE with DRX
    RrcIdle,
    /// UE in RRC_INACTIVE (power-saving state Rel-16+)
    RrcInactive,
    /// UE in RRC_CONNECTED, receiving data
    RrcConnected,
    /// UE in connected-mode DRX
    ConnectedDrx,
    /// UE performing measurement (RRM)
    Measuring,
}

/// Initial access message type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitialAccessMsg {
    /// Random Access Preamble (Msg1)
    Msg1Preamble,
    /// Random Access Response (Msg2, from gNB)
    Msg2Rar,
    /// RRC Setup Request (Msg3) – carries RedCap indicator
    Msg3SetupRequest,
    /// RRC Setup (Msg4, from gNB)
    Msg4Setup,
}

/// RRM (Radio Resource Management) measurement type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RrmMeasurement {
    /// Reference Signal Received Power
    Rsrp,
    /// Reference Signal Received Quality
    Rsrq,
    /// Signal to Noise and Interference Ratio
    Sinr,
}

// ─────────────────────────────────────────────────────────────────
// Core data structures
// ─────────────────────────────────────────────────────────────────

/// Bandwidth Part (BWP) configuration
#[derive(Debug, Clone)]
pub struct BwpConfig {
    /// BWP identifier (0-4)
    pub bwp_id: u8,
    /// BWP type
    pub bwp_type: BwpType,
    /// Starting PRB index within the carrier
    pub start_prb: u32,
    /// Number of PRBs in this BWP
    pub num_prbs: u32,
    /// Subcarrier spacing
    pub scs: SubcarrierSpacing,
    /// Whether this is the currently active BWP
    pub is_active: bool,
}

impl BwpConfig {
    /// Compute bandwidth in Hz for this BWP
    pub fn bandwidth_hz(&self) -> u64 {
        (self.num_prbs * SUBCARRIERS_PER_PRB) as u64 * self.scs.hz() as u64
    }

    /// Check if this BWP is valid for a RedCap UE in the given frequency range
    pub fn validate_for_redcap(&self, fr: FrequencyRange) -> Result<(), RedCapError> {
        let max_bw = match fr {
            FrequencyRange::Fr1 => FR1_REDCAP_MAX_BW_HZ,
            FrequencyRange::Fr2 => FR2_REDCAP_MAX_BW_HZ,
        };
        if self.bandwidth_hz() > max_bw {
            return Err(RedCapError::BwpTooWide {
                actual_hz: self.bandwidth_hz(),
                max_hz: max_bw,
            });
        }
        // Check PRB limit per SCS
        let max_prbs = match (fr, &self.scs) {
            (FrequencyRange::Fr1, SubcarrierSpacing::Scs15kHz) => FR1_20MHZ_15KHZ_MAX_PRBS,
            (FrequencyRange::Fr1, SubcarrierSpacing::Scs30kHz) => FR1_20MHZ_30KHZ_MAX_PRBS,
            (FrequencyRange::Fr2, SubcarrierSpacing::Scs120kHz) => FR2_100MHZ_120KHZ_MAX_PRBS,
            _ => 275, // default to full NR maximum
        };
        if self.num_prbs > max_prbs {
            return Err(RedCapError::TooManyPrbs {
                actual: self.num_prbs,
                max: max_prbs,
            });
        }
        Ok(())
    }
}

/// DRX configuration per 3GPP TS 38.321
#[derive(Debug, Clone)]
pub struct DrxConfig {
    /// On-duration timer in ms (number of slots UE monitors PDCCH after waking)
    pub on_duration_ms: u32,
    /// DRX inactivity timer in ms
    pub inactivity_timer_ms: u32,
    /// Short DRX cycle length in ms (optional)
    pub short_cycle_ms: Option<u32>,
    /// Long DRX cycle length in ms (max 10240 ms for RedCap)
    pub long_cycle_ms: u32,
    /// Short DRX timer in cycles
    pub short_cycle_timer: Option<u32>,
    /// HARQ RTT timer for DL (ms)
    pub harq_rtt_timer_dl_ms: u32,
    /// HARQ RTT timer for UL (ms)
    pub harq_rtt_timer_ul_ms: u32,
    /// Retransmission timer DL (ms)
    pub retransmission_timer_dl_ms: u32,
    /// Retransmission timer UL (ms)
    pub retransmission_timer_ul_ms: u32,
}

impl DrxConfig {
    /// Create a power-saving RedCap DRX configuration (long DRX = 5.12 s)
    pub fn redcap_power_save() -> Self {
        Self {
            on_duration_ms: 10,
            inactivity_timer_ms: 100,
            short_cycle_ms: Some(640),
            long_cycle_ms: 5_120,
            short_cycle_timer: Some(8),
            harq_rtt_timer_dl_ms: 8,
            harq_rtt_timer_ul_ms: 8,
            retransmission_timer_dl_ms: 16,
            retransmission_timer_ul_ms: 16,
        }
    }

    /// Create a balanced performance/power DRX configuration
    pub fn redcap_balanced() -> Self {
        Self {
            on_duration_ms: 20,
            inactivity_timer_ms: 200,
            short_cycle_ms: Some(320),
            long_cycle_ms: 1_280,
            short_cycle_timer: Some(4),
            harq_rtt_timer_dl_ms: 8,
            harq_rtt_timer_ul_ms: 8,
            retransmission_timer_dl_ms: 16,
            retransmission_timer_ul_ms: 16,
        }
    }

    /// Validate DRX cycle lengths (must be <= MAX_DRX_CYCLE_MS)
    pub fn validate(&self) -> Result<(), RedCapError> {
        if self.long_cycle_ms > MAX_DRX_CYCLE_MS {
            return Err(RedCapError::DrxCycleTooLong {
                actual_ms: self.long_cycle_ms,
                max_ms: MAX_DRX_CYCLE_MS,
            });
        }
        if let Some(short) = self.short_cycle_ms {
            if short >= self.long_cycle_ms {
                return Err(RedCapError::InvalidDrxConfig(
                    "short_cycle_ms must be less than long_cycle_ms".to_string(),
                ));
            }
        }
        Ok(())
    }

    /// Compute average duty cycle ratio (approximation)
    pub fn duty_cycle_ratio(&self) -> f64 {
        let active_ms = self.on_duration_ms as f64;
        let cycle_ms = self.long_cycle_ms as f64;
        (active_ms / cycle_ms).min(1.0)
    }
}

/// Half-Duplex FDD configuration
#[derive(Debug, Clone)]
pub struct HalfDuplexConfig {
    /// Guard period in slots between DL and UL (minimum 1)
    pub guard_period_slots: u32,
    /// Whether the UE is currently in TX mode
    pub in_tx_mode: bool,
    /// SCS used for guard period calculation
    pub scs: SubcarrierSpacing,
}

impl HalfDuplexConfig {
    pub fn new(scs: SubcarrierSpacing) -> Self {
        Self {
            guard_period_slots: HD_FDD_MIN_GUARD_SLOTS,
            in_tx_mode: false,
            scs,
        }
    }

    /// Guard period duration in microseconds
    pub fn guard_duration_us(&self) -> f64 {
        self.guard_period_slots as f64 * self.scs.slot_duration_us()
    }

    /// Switch from DL to UL mode (adds guard period)
    pub fn switch_to_ul(&mut self) {
        self.in_tx_mode = true;
    }

    /// Switch from UL to DL mode (adds guard period)
    pub fn switch_to_dl(&mut self) {
        self.in_tx_mode = false;
    }
}

/// RRM (Radio Resource Management) relaxation configuration
#[derive(Debug, Clone)]
pub struct RrmRelaxConfig {
    /// Relaxed measurement period multiplier (relative to normal NR)
    pub period_multiplier: u32,
    /// Whether SSB-based RRM is enabled (instead of CSI-RS)
    pub ssb_rrm_enabled: bool,
    /// Minimum measurement gap period in ms
    pub min_gap_ms: u32,
    /// Maximum allowed relaxation threshold (dBm) for RSRP to trigger measurement
    pub rsrp_delta_threshold_dbm: f64,
    /// Whether UE is considered stationary/low-mobility
    pub low_mobility: bool,
    /// Whether UE is not-at-cell-edge
    pub not_at_cell_edge: bool,
}

impl RrmRelaxConfig {
    /// Default aggressive relaxation for stationary low-power devices
    pub fn stationary_low_power() -> Self {
        Self {
            period_multiplier: 8,
            ssb_rrm_enabled: true,
            min_gap_ms: 640,
            rsrp_delta_threshold_dbm: 3.0,
            low_mobility: true,
            not_at_cell_edge: true,
        }
    }

    /// Default configuration for mobile devices
    pub fn mobile_device() -> Self {
        Self {
            period_multiplier: 2,
            ssb_rrm_enabled: true,
            min_gap_ms: 80,
            rsrp_delta_threshold_dbm: 6.0,
            low_mobility: false,
            not_at_cell_edge: false,
        }
    }

    /// Check if relaxation criteria are met (3GPP TS 38.133 Section 9.1.x)
    pub fn relaxation_allowed(&self) -> bool {
        self.low_mobility && self.not_at_cell_edge
    }

    /// Effective measurement period in ms
    pub fn effective_period_ms(&self, base_period_ms: u32) -> u32 {
        if self.relaxation_allowed() {
            base_period_ms.saturating_mul(self.period_multiplier)
        } else {
            base_period_ms
        }
    }
}

/// PDCCH monitoring configuration for RedCap
#[derive(Debug, Clone)]
pub struct PdcchMonitorConfig {
    /// Whether PDCCH skipping is enabled
    pub skipping_enabled: bool,
    /// Number of consecutive slots to skip PDCCH monitoring
    pub skip_duration_slots: u32,
    /// Whether to use reduced search space sets
    pub reduced_search_spaces: bool,
    /// Periodicity of PDCCH monitoring in slots
    pub monitoring_periodicity_slots: u32,
    /// Offset within the period for monitoring occasion
    pub monitoring_offset_slots: u32,
}

impl PdcchMonitorConfig {
    /// Default configuration enabling PDCCH skipping for power saving
    pub fn with_skipping(scs: SubcarrierSpacing) -> Self {
        let slots_per_sf = scs.slots_per_subframe();
        Self {
            skipping_enabled: true,
            skip_duration_slots: slots_per_sf * 4, // Skip 4 subframes
            reduced_search_spaces: true,
            monitoring_periodicity_slots: slots_per_sf * 2,
            monitoring_offset_slots: 0,
        }
    }

    /// Standard PDCCH monitoring (no skipping)
    pub fn standard(scs: SubcarrierSpacing) -> Self {
        let slots_per_sf = scs.slots_per_subframe();
        Self {
            skipping_enabled: false,
            skip_duration_slots: 0,
            reduced_search_spaces: false,
            monitoring_periodicity_slots: slots_per_sf,
            monitoring_offset_slots: 0,
        }
    }
}

/// Per-slot resource allocation for RedCap
#[derive(Debug, Clone)]
pub struct ResourceAllocation {
    /// Starting PRB within BWP
    pub start_prb: u32,
    /// Number of allocated PRBs
    pub num_prbs: u32,
    /// Modulation and coding scheme index (0-28 for PDSCH)
    pub mcs_index: u8,
    /// Number of allocated OFDM symbols
    pub num_symbols: u32,
    /// HARQ process ID (0-15)
    pub harq_pid: u8,
    /// New Data Indicator (true = new transmission)
    pub ndi: bool,
    /// Redundancy version (0-3)
    pub rv: u8,
}

/// MCS table entry (simplified, per 3GPP TS 38.214 Table 5.1.3.1-1)
#[derive(Debug, Clone)]
pub struct McsEntry {
    /// MCS index
    pub index: u8,
    /// Modulation order Q_m
    pub mod_order: u32,
    /// Target code rate * 1024
    pub target_code_rate_1024: u32,
    /// Spectral efficiency (bits/RE)
    pub spectral_efficiency: f64,
}

/// Simplified MCS table (64-QAM entries only, max for RedCap)
pub fn redcap_mcs_table() -> Vec<McsEntry> {
    // 3GPP TS 38.214 Table 5.1.3.1-1 (64QAM table), entries 0-27
    let raw: &[(u8, u32, u32)] = &[
        (0, 2, 120),
        (1, 2, 157),
        (2, 2, 193),
        (3, 2, 251),
        (4, 2, 308),
        (5, 2, 379),
        (6, 2, 449),
        (7, 2, 526),
        (8, 2, 602),
        (9, 2, 679),
        (10, 4, 340),
        (11, 4, 378),
        (12, 4, 434),
        (13, 4, 490),
        (14, 4, 553),
        (15, 4, 616),
        (16, 4, 658),
        (17, 6, 438),
        (18, 6, 466),
        (19, 6, 517),
        (20, 6, 567),
        (21, 6, 616),
        (22, 6, 666),
        (23, 6, 719),
        (24, 6, 772),
        (25, 6, 822),
        (26, 6, 873),
        (27, 6, 910),
        // Index 28 reserved (256-QAM) – not valid for RedCap
    ];
    raw.iter()
        .map(|(idx, qm, cr1024)| {
            let se = (*qm as f64) * (*cr1024 as f64) / 1024.0;
            McsEntry {
                index: *idx,
                mod_order: *qm,
                target_code_rate_1024: *cr1024,
                spectral_efficiency: se,
            }
        })
        .collect()
}

/// Compute approximate Transport Block Size (TBS) in bits.
/// Uses simplified formula: TBS ≈ floor(spectral_efficiency * num_re * scaling)
/// where num_re = num_prbs * 12 * num_symbols, scaling accounts for overhead.
pub fn compute_tbs(mcs: &McsEntry, num_prbs: u32, num_symbols: u32) -> u32 {
    // Approximate RE count (ignoring DMRS / reference signal overhead ≈ 12%)
    let num_re = num_prbs * SUBCARRIERS_PER_PRB * num_symbols;
    let oh_factor = 0.88_f64; // ~12% DMRS + control overhead
    let raw_bits = mcs.spectral_efficiency * num_re as f64 * oh_factor;
    // Round down to nearest multiple of 8 (byte-aligned TBS)
    let tbs = (raw_bits as u32) & !7;
    tbs.max(8)
}

/// Half-duplex FDD frame timer
#[derive(Debug, Clone)]
pub struct HdFddTimer {
    /// Current slot index within a radio frame (0-1279 for 30 kHz SCS, 10ms)
    pub current_slot: u32,
    /// Total slots per radio frame
    pub slots_per_frame: u32,
    /// Guard period remaining (slots)
    pub guard_remaining: u32,
    /// In TX (true) or RX (false) half
    pub in_tx_half: bool,
}

impl HdFddTimer {
    pub fn new(scs: SubcarrierSpacing) -> Self {
        let slots_per_frame = scs.slots_per_subframe() * 10; // 10 subframes per frame
        Self {
            current_slot: 0,
            slots_per_frame,
            guard_remaining: 0,
            in_tx_half: false,
        }
    }

    /// Advance by one slot. Returns true if a mode switch occurred.
    pub fn tick(&mut self) -> bool {
        if self.guard_remaining > 0 {
            self.guard_remaining -= 1;
            if self.guard_remaining == 0 {
                self.in_tx_half = !self.in_tx_half;
                return true;
            }
        }
        self.current_slot = (self.current_slot + 1) % self.slots_per_frame;
        false
    }

    /// Request a switch to TX mode. If currently in RX, sets guard period.
    pub fn request_tx(&mut self, guard_slots: u32) -> bool {
        if !self.in_tx_half {
            self.guard_remaining = guard_slots;
            true
        } else {
            false // already in TX
        }
    }

    /// Request a switch to RX mode. If currently in TX, sets guard period.
    pub fn request_rx(&mut self, guard_slots: u32) -> bool {
        if self.in_tx_half {
            self.guard_remaining = guard_slots;
            true
        } else {
            false // already in RX
        }
    }
}

/// HARQ process state
#[derive(Debug, Clone)]
pub struct HarqProcess {
    pub pid: u8,
    pub ndi: bool,
    pub rv: u8,
    pub retx_count: u8,
    pub max_retx: u8,
    pub pending_ack: bool,
    /// Soft buffer (LLR accumulation for Chase combining)
    pub soft_buffer: Vec<f32>,
}

impl HarqProcess {
    pub fn new(pid: u8, buffer_size: usize) -> Self {
        Self {
            pid,
            ndi: false,
            rv: 0,
            retx_count: 0,
            max_retx: 4,
            pending_ack: false,
            soft_buffer: vec![0.0f32; buffer_size],
        }
    }

    pub fn reset(&mut self) {
        self.rv = 0;
        self.retx_count = 0;
        self.pending_ack = false;
        for v in self.soft_buffer.iter_mut() {
            *v = 0.0;
        }
    }

    /// Chase combining: accumulate LLRs
    pub fn combine(&mut self, new_llrs: &[f32]) {
        let len = self.soft_buffer.len().min(new_llrs.len());
        for i in 0..len {
            self.soft_buffer[i] += new_llrs[i];
        }
    }

    pub fn increment_retx(&mut self) -> bool {
        self.retx_count += 1;
        self.retx_count >= self.max_retx
    }
}

/// RedCap UE capability advertisement fields
#[derive(Debug, Clone)]
pub struct RedCapCapabilities {
    /// Frequency range supported
    pub freq_range: FrequencyRange,
    /// Maximum supported bandwidth in Hz
    pub max_bw_hz: u64,
    /// Number of receive antennas
    pub num_rx_antennas: u8,
    /// Number of transmit antennas
    pub num_tx_antennas: u8,
    /// Maximum DL modulation supported
    pub max_dl_modulation: Modulation,
    /// Maximum UL modulation supported
    pub max_ul_modulation: Modulation,
    /// Whether half-duplex FDD is supported
    pub hd_fdd_supported: bool,
    /// RedCap indicator bit for RRC (true = RedCap UE)
    pub is_redcap: bool,
    /// Release version (17 = Rel-17 baseline)
    pub release: u8,
    /// Whether eRedCap (Release 18) features are supported
    pub eredcap: bool,
}

impl RedCapCapabilities {
    /// Create default Rel-17 RedCap capabilities for FR1 wearable
    pub fn fr1_wearable() -> Self {
        Self {
            freq_range: FrequencyRange::Fr1,
            max_bw_hz: FR1_REDCAP_MAX_BW_HZ,
            num_rx_antennas: 1,
            num_tx_antennas: 1,
            max_dl_modulation: Modulation::Qam64,
            max_ul_modulation: Modulation::Qam64,
            hd_fdd_supported: true,
            is_redcap: true,
            release: 17,
            eredcap: false,
        }
    }

    /// Create default Rel-17 RedCap capabilities for FR1 industrial sensor
    pub fn fr1_industrial_sensor() -> Self {
        Self {
            freq_range: FrequencyRange::Fr1,
            max_bw_hz: FR1_REDCAP_MAX_BW_HZ,
            num_rx_antennas: 1,
            num_tx_antennas: 1,
            max_dl_modulation: Modulation::Qam16,
            max_ul_modulation: Modulation::Qpsk,
            hd_fdd_supported: true,
            is_redcap: true,
            release: 17,
            eredcap: false,
        }
    }

    /// Create Rel-17 RedCap capabilities for FR2 video surveillance
    pub fn fr2_video_surveillance() -> Self {
        Self {
            freq_range: FrequencyRange::Fr2,
            max_bw_hz: FR2_REDCAP_MAX_BW_HZ,
            num_rx_antennas: 1,
            num_tx_antennas: 1,
            max_dl_modulation: Modulation::Qam64,
            max_ul_modulation: Modulation::Qam64,
            hd_fdd_supported: false, // TDD at mmWave
            is_redcap: true,
            release: 17,
            eredcap: false,
        }
    }

    /// Validate capabilities against 3GPP Release 17 constraints
    pub fn validate(&self) -> Result<(), RedCapError> {
        // Check max bandwidth
        let max_allowed = match self.freq_range {
            FrequencyRange::Fr1 => FR1_REDCAP_MAX_BW_HZ,
            FrequencyRange::Fr2 => FR2_REDCAP_MAX_BW_HZ,
        };
        if self.max_bw_hz > max_allowed {
            return Err(RedCapError::BwpTooWide {
                actual_hz: self.max_bw_hz,
                max_hz: max_allowed,
            });
        }
        // Check antenna count
        if self.num_rx_antennas > 1 {
            return Err(RedCapError::ExceedsAntennaLimit {
                actual: self.num_rx_antennas,
                max: 1,
            });
        }
        // Check modulation
        if !self.max_dl_modulation.is_redcap_allowed() {
            return Err(RedCapError::ModulationNotAllowed(self.max_dl_modulation));
        }
        if !self.max_ul_modulation.is_redcap_allowed() {
            return Err(RedCapError::ModulationNotAllowed(self.max_ul_modulation));
        }
        Ok(())
    }
}

/// RRC connection request with RedCap identification
#[derive(Debug, Clone)]
pub struct RedCapRrcSetupRequest {
    /// UE identity (truncated RNTI-like value for simulation)
    pub ue_id: u32,
    /// RedCap indicator (set for RedCap UEs)
    pub redcap_indicator: bool,
    /// Establishment cause
    pub establishment_cause: u8,
    /// Requested dedicated BWP index (if any)
    pub requested_bwp: Option<u8>,
}

/// Power consumption model (relative units, normalized to 1.0 = max)
#[derive(Debug, Clone)]
pub struct PowerModel {
    /// Power in ACTIVE state (receiving data)
    pub active_rx_power: f64,
    /// Power in TX state
    pub active_tx_power: f64,
    /// Power in DRX ON state (monitoring PDCCH)
    pub drx_on_power: f64,
    /// Power in DRX sleep state
    pub sleep_power: f64,
    /// Power overhead for half-duplex switching
    pub hd_switching_power: f64,
}

impl PowerModel {
    /// Default RedCap power model (normalized)
    pub fn redcap_default() -> Self {
        Self {
            active_rx_power: 1.0,
            active_tx_power: 1.2,
            drx_on_power: 0.3,
            sleep_power: 0.01,
            hd_switching_power: 0.05,
        }
    }

    /// Estimate average power given DRX config and duty cycle
    pub fn average_power(&self, drx: &DrxConfig) -> f64 {
        let duty = drx.duty_cycle_ratio();
        duty * self.drx_on_power + (1.0 - duty) * self.sleep_power
    }
}

/// SSB-based measurement result
#[derive(Debug, Clone)]
pub struct SsbMeasurement {
    /// SSB index (0-7 for FR1 common)
    pub ssb_index: u8,
    /// RSRP in dBm
    pub rsrp_dbm: f64,
    /// RSRQ in dB
    pub rsrq_db: f64,
    /// SINR in dB
    pub sinr_db: f64,
    /// Timestamp in slots
    pub timestamp_slot: u64,
}

/// RedCap processor error types
#[derive(Debug, Clone, PartialEq)]
pub enum RedCapError {
    BwpTooWide { actual_hz: u64, max_hz: u64 },
    TooManyPrbs { actual: u32, max: u32 },
    ExceedsAntennaLimit { actual: u8, max: u8 },
    ModulationNotAllowed(Modulation),
    DrxCycleTooLong { actual_ms: u32, max_ms: u32 },
    InvalidDrxConfig(String),
    InvalidMcsIndex(u8),
    HarqPidOutOfRange(u8),
    StateTransitionError { from: UeState, to: UeState },
    InvalidGuardPeriod,
    BwpNotFound(u8),
}

impl std::fmt::Display for RedCapError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RedCapError::BwpTooWide { actual_hz, max_hz } => write!(
                f,
                "BWP bandwidth {} Hz exceeds RedCap max {} Hz",
                actual_hz, max_hz
            ),
            RedCapError::TooManyPrbs { actual, max } => {
                write!(f, "BWP has {} PRBs, exceeds RedCap max {}", actual, max)
            }
            RedCapError::ExceedsAntennaLimit { actual, max } => {
                write!(f, "RX antenna count {} exceeds limit {}", actual, max)
            }
            RedCapError::ModulationNotAllowed(m) => {
                write!(f, "Modulation {:?} not allowed for RedCap", m)
            }
            RedCapError::DrxCycleTooLong { actual_ms, max_ms } => write!(
                f,
                "DRX cycle {} ms exceeds max {} ms",
                actual_ms, max_ms
            ),
            RedCapError::InvalidDrxConfig(s) => write!(f, "Invalid DRX config: {}", s),
            RedCapError::InvalidMcsIndex(idx) => write!(f, "Invalid MCS index {}", idx),
            RedCapError::HarqPidOutOfRange(pid) => write!(f, "HARQ PID {} out of range", pid),
            RedCapError::StateTransitionError { from, to } => {
                write!(f, "Invalid state transition {:?} -> {:?}", from, to)
            }
            RedCapError::InvalidGuardPeriod => write!(f, "Invalid half-duplex guard period"),
            RedCapError::BwpNotFound(id) => write!(f, "BWP with ID {} not found", id),
        }
    }
}

// ─────────────────────────────────────────────────────────────────
// Main RedCap Processor
// ─────────────────────────────────────────────────────────────────

/// 5G NR RedCap device processor
///
/// Manages the complete RedCap UE baseband processing including:
/// - BWP management and validation
/// - Half-duplex FDD timing
/// - DRX cycle management
/// - PDCCH monitoring reduction
/// - RRM measurement relaxation
/// - HARQ process management
/// - MCS selection (capped at 64-QAM)
/// - Initial access with RedCap indication
pub struct RedCapProcessor {
    /// UE capabilities
    pub capabilities: RedCapCapabilities,
    /// Active BWP configurations (DL)
    dl_bwps: Vec<BwpConfig>,
    /// Active BWP configurations (UL)
    ul_bwps: Vec<BwpConfig>,
    /// Current active DL BWP index
    active_dl_bwp: u8,
    /// Current active UL BWP index
    active_ul_bwp: u8,
    /// DRX configuration
    drx_config: DrxConfig,
    /// Current DRX state
    drx_state: DrxState,
    /// DRX timer counter (in ms)
    drx_timer_ms: u32,
    /// Half-duplex FDD configuration (if applicable)
    hd_config: Option<HalfDuplexConfig>,
    /// Half-duplex timer
    hd_timer: Option<HdFddTimer>,
    /// PDCCH monitoring configuration
    pdcch_config: PdcchMonitorConfig,
    /// RRM relaxation configuration
    rrm_config: RrmRelaxConfig,
    /// UE state
    state: UeState,
    /// HARQ processes (up to 16)
    harq_procs: Vec<HarqProcess>,
    /// MCS table
    mcs_table: Vec<McsEntry>,
    /// Current system frame number
    sfn: u32,
    /// Current slot within frame
    current_slot: u32,
    /// SCS in use
    scs: SubcarrierSpacing,
    /// Measurement history
    measurements: VecDeque<SsbMeasurement>,
    /// Power model
    power_model: PowerModel,
    /// PDCCH skipping counter
    pdcch_skip_remaining: u32,
    /// Whether RedCap has been indicated to gNB
    redcap_indicated: bool,
    /// Slots per frame
    slots_per_frame: u32,
}

impl RedCapProcessor {
    /// Create a new RedCap processor with given capabilities and SCS
    pub fn new(capabilities: RedCapCapabilities, scs: SubcarrierSpacing) -> Self {
        let slots_per_frame = scs.slots_per_subframe() * 10;
        let mut harq_procs = Vec::new();
        for pid in 0..16u8 {
            harq_procs.push(HarqProcess::new(pid, 8192));
        }
        let hd_config = if capabilities.hd_fdd_supported {
            Some(HalfDuplexConfig::new(scs))
        } else {
            None
        };
        let hd_timer = if capabilities.hd_fdd_supported {
            Some(HdFddTimer::new(scs))
        } else {
            None
        };

        Self {
            capabilities,
            dl_bwps: Vec::new(),
            ul_bwps: Vec::new(),
            active_dl_bwp: 0,
            active_ul_bwp: 0,
            drx_config: DrxConfig::redcap_balanced(),
            drx_state: DrxState::Inactive,
            drx_timer_ms: 0,
            hd_config,
            hd_timer,
            pdcch_config: PdcchMonitorConfig::standard(scs),
            rrm_config: RrmRelaxConfig::mobile_device(),
            state: UeState::Idle,
            harq_procs,
            mcs_table: redcap_mcs_table(),
            sfn: 0,
            current_slot: 0,
            scs,
            measurements: VecDeque::new(),
            power_model: PowerModel::redcap_default(),
            pdcch_skip_remaining: 0,
            redcap_indicated: false,
            slots_per_frame,
        }
    }

    // ── BWP Management ───────────────────────────────────────────

    /// Add a DL BWP configuration
    pub fn add_dl_bwp(&mut self, bwp: BwpConfig) -> Result<(), RedCapError> {
        bwp.validate_for_redcap(self.capabilities.freq_range)?;
        self.dl_bwps.push(bwp);
        Ok(())
    }

    /// Add a UL BWP configuration
    pub fn add_ul_bwp(&mut self, bwp: BwpConfig) -> Result<(), RedCapError> {
        bwp.validate_for_redcap(self.capabilities.freq_range)?;
        self.ul_bwps.push(bwp);
        Ok(())
    }

    /// Switch active DL BWP (triggers re-configuration)
    pub fn switch_dl_bwp(&mut self, bwp_id: u8) -> Result<(), RedCapError> {
        let pos = self
            .dl_bwps
            .iter()
            .position(|b| b.bwp_id == bwp_id)
            .ok_or(RedCapError::BwpNotFound(bwp_id))?;
        // Deactivate old
        if let Some(old) = self.dl_bwps.iter_mut().find(|b| b.is_active) {
            old.is_active = false;
        }
        self.dl_bwps[pos].is_active = true;
        self.active_dl_bwp = bwp_id;
        Ok(())
    }

    /// Switch active UL BWP
    pub fn switch_ul_bwp(&mut self, bwp_id: u8) -> Result<(), RedCapError> {
        let pos = self
            .ul_bwps
            .iter()
            .position(|b| b.bwp_id == bwp_id)
            .ok_or(RedCapError::BwpNotFound(bwp_id))?;
        if let Some(old) = self.ul_bwps.iter_mut().find(|b| b.is_active) {
            old.is_active = false;
        }
        self.ul_bwps[pos].is_active = true;
        self.active_ul_bwp = bwp_id;
        Ok(())
    }

    /// Get active DL BWP reference
    pub fn active_dl_bwp(&self) -> Option<&BwpConfig> {
        self.dl_bwps.iter().find(|b| b.bwp_id == self.active_dl_bwp)
    }

    /// Get active UL BWP reference
    pub fn active_ul_bwp(&self) -> Option<&BwpConfig> {
        self.ul_bwps.iter().find(|b| b.bwp_id == self.active_ul_bwp)
    }

    // ── DRX Management ───────────────────────────────────────────

    /// Configure DRX parameters
    pub fn configure_drx(&mut self, config: DrxConfig) -> Result<(), RedCapError> {
        config.validate()?;
        self.drx_config = config;
        Ok(())
    }

    /// Start DRX operation (enter long cycle)
    pub fn start_drx(&mut self) {
        self.drx_state = DrxState::LongCycle;
        self.drx_timer_ms = self.drx_config.long_cycle_ms;
    }

    /// Tick DRX timer by given ms. Returns new DRX state and whether UE should wake.
    pub fn tick_drx(&mut self, elapsed_ms: u32) -> (DrxState, bool) {
        let should_wake = match self.drx_state {
            DrxState::Active => {
                // In active state, count down inactivity timer
                if self.drx_timer_ms <= elapsed_ms {
                    // Enter short cycle if configured
                    if let Some(short_ms) = self.drx_config.short_cycle_ms {
                        self.drx_state = DrxState::ShortCycle;
                        self.drx_timer_ms = short_ms;
                    } else {
                        self.drx_state = DrxState::LongCycle;
                        self.drx_timer_ms = self.drx_config.long_cycle_ms;
                    }
                    false
                } else {
                    self.drx_timer_ms -= elapsed_ms;
                    false
                }
            }
            DrxState::ShortCycle | DrxState::LongCycle => {
                if self.drx_timer_ms <= elapsed_ms {
                    // Wake up: enter on-duration
                    self.drx_state = DrxState::Active;
                    self.drx_timer_ms = self.drx_config.on_duration_ms;
                    true // signal wake
                } else {
                    self.drx_timer_ms -= elapsed_ms;
                    false
                }
            }
            DrxState::Inactive => false,
        };
        (self.drx_state, should_wake)
    }

    /// Force wake from DRX (e.g., data arrival)
    pub fn wake_from_drx(&mut self) {
        self.drx_state = DrxState::Active;
        self.drx_timer_ms = self.drx_config.inactivity_timer_ms;
    }

    /// Get current DRX duty cycle ratio (approximate)
    pub fn drx_duty_cycle(&self) -> f64 {
        self.drx_config.duty_cycle_ratio()
    }

    // ── Half-Duplex FDD ──────────────────────────────────────────

    /// Configure half-duplex operation
    pub fn configure_half_duplex(&mut self, guard_slots: u32) -> Result<(), RedCapError> {
        if guard_slots < HD_FDD_MIN_GUARD_SLOTS {
            return Err(RedCapError::InvalidGuardPeriod);
        }
        if let Some(ref mut hd) = self.hd_config {
            hd.guard_period_slots = guard_slots;
        }
        Ok(())
    }

    /// Check if currently in TX mode (half-duplex)
    pub fn is_tx_mode(&self) -> bool {
        self.hd_config.as_ref().map(|hd| hd.in_tx_mode).unwrap_or(false)
    }

    /// Request switch to TX mode. Returns guard duration in µs.
    pub fn request_tx_mode(&mut self) -> Option<f64> {
        if let Some(ref mut hd) = self.hd_config {
            if let Some(ref mut timer) = self.hd_timer {
                let guard = hd.guard_period_slots;
                timer.request_tx(guard);
                hd.switch_to_ul();
                return Some(hd.guard_duration_us());
            }
        }
        None
    }

    /// Request switch to RX mode. Returns guard duration in µs.
    pub fn request_rx_mode(&mut self) -> Option<f64> {
        if let Some(ref mut hd) = self.hd_config {
            if let Some(ref mut timer) = self.hd_timer {
                let guard = hd.guard_period_slots;
                timer.request_rx(guard);
                hd.switch_to_dl();
                return Some(hd.guard_duration_us());
            }
        }
        None
    }

    // ── PDCCH Monitoring ─────────────────────────────────────────

    /// Configure PDCCH monitoring (including optional skipping)
    pub fn configure_pdcch_monitoring(&mut self, config: PdcchMonitorConfig) {
        self.pdcch_config = config;
    }

    /// Returns true if PDCCH should be monitored in current slot
    pub fn should_monitor_pdcch(&mut self) -> bool {
        if !self.pdcch_config.skipping_enabled {
            return true;
        }
        if self.pdcch_skip_remaining > 0 {
            self.pdcch_skip_remaining -= 1;
            return false;
        }
        // Check monitoring occasion
        let offset = self.current_slot % self.pdcch_config.monitoring_periodicity_slots;
        if offset == self.pdcch_config.monitoring_offset_slots {
            // Start new skip window
            self.pdcch_skip_remaining = self.pdcch_config.skip_duration_slots.saturating_sub(1);
            true
        } else {
            false
        }
    }

    // ── RRM Measurements ─────────────────────────────────────────

    /// Configure RRM relaxation
    pub fn configure_rrm(&mut self, config: RrmRelaxConfig) {
        self.rrm_config = config;
    }

    /// Record an SSB measurement
    pub fn record_measurement(&mut self, meas: SsbMeasurement) {
        self.measurements.push_back(meas);
        // Keep only last 16 measurements
        while self.measurements.len() > 16 {
            self.measurements.pop_front();
        }
    }

    /// Compute average RSRP from recent measurements
    pub fn average_rsrp_dbm(&self) -> Option<f64> {
        if self.measurements.is_empty() {
            return None;
        }
        let sum: f64 = self.measurements.iter().map(|m| m.rsrp_dbm).sum();
        Some(sum / self.measurements.len() as f64)
    }

    /// Check if RRM relaxation criteria are met
    pub fn rrm_relaxation_allowed(&self) -> bool {
        self.rrm_config.relaxation_allowed()
    }

    /// Get effective RRM measurement period in ms
    pub fn rrm_measurement_period_ms(&self, base_period_ms: u32) -> u32 {
        self.rrm_config.effective_period_ms(base_period_ms)
    }

    // ── Initial Access ───────────────────────────────────────────

    /// Build Msg3 (RRC Setup Request) with RedCap indicator
    pub fn build_msg3(&mut self) -> RedCapRrcSetupRequest {
        self.redcap_indicated = true;
        self.state = UeState::InitialAccess;
        RedCapRrcSetupRequest {
            ue_id: 0x1234, // placeholder
            redcap_indicator: self.capabilities.is_redcap,
            establishment_cause: 0x01, // mo-Signalling
            requested_bwp: Some(self.active_ul_bwp),
        }
    }

    /// Process Msg4 (RRC Setup). Transitions to RRC_CONNECTED.
    pub fn process_msg4(&mut self) -> Result<(), RedCapError> {
        if self.state != UeState::InitialAccess {
            return Err(RedCapError::StateTransitionError {
                from: self.state,
                to: UeState::RrcConnected,
            });
        }
        self.state = UeState::RrcConnected;
        self.drx_state = DrxState::Active;
        self.drx_timer_ms = self.drx_config.inactivity_timer_ms;
        Ok(())
    }

    // ── MCS / Resource Allocation ────────────────────────────────

    /// Get MCS entry by index (validates against RedCap 64-QAM cap)
    pub fn get_mcs(&self, index: u8) -> Result<&McsEntry, RedCapError> {
        if index >= self.mcs_table.len() as u8 {
            return Err(RedCapError::InvalidMcsIndex(index));
        }
        let entry = &self.mcs_table[index as usize];
        // Validate modulation order is within RedCap cap
        if entry.mod_order > REDCAP_MAX_DL_MOD_ORDER as u32 {
            return Err(RedCapError::ModulationNotAllowed(Modulation::Qam256));
        }
        Ok(entry)
    }

    /// Compute TBS for a given resource allocation
    pub fn compute_tbs_for_alloc(&self, alloc: &ResourceAllocation) -> Result<u32, RedCapError> {
        let mcs = self.get_mcs(alloc.mcs_index)?;
        Ok(compute_tbs(mcs, alloc.num_prbs, alloc.num_symbols))
    }

    /// Select best MCS given target modulation and code rate constraints
    pub fn select_mcs(&self, max_mod: Modulation, min_se: f64) -> Option<&McsEntry> {
        let max_order = max_mod.bits_per_symbol();
        self.mcs_table
            .iter()
            .filter(|e| e.mod_order <= max_order && e.spectral_efficiency >= min_se)
            .min_by(|a, b| a.index.cmp(&b.index))
    }

    // ── HARQ Management ──────────────────────────────────────────

    /// Get HARQ process by PID
    pub fn harq_process(&self, pid: u8) -> Result<&HarqProcess, RedCapError> {
        if pid >= 16 {
            return Err(RedCapError::HarqPidOutOfRange(pid));
        }
        Ok(&self.harq_procs[pid as usize])
    }

    /// Get mutable HARQ process
    pub fn harq_process_mut(&mut self, pid: u8) -> Result<&mut HarqProcess, RedCapError> {
        if pid >= 16 {
            return Err(RedCapError::HarqPidOutOfRange(pid));
        }
        Ok(&mut self.harq_procs[pid as usize])
    }

    /// Process received DL LLRs for a given HARQ PID
    pub fn process_dl_harq(&mut self, pid: u8, llrs: &[f32], ndi: bool) -> Result<bool, RedCapError> {
        let proc = self.harq_process_mut(pid)?;
        if ndi != proc.ndi {
            // New data: reset process
            proc.reset();
            proc.ndi = ndi;
        }
        proc.combine(llrs);
        // Simple threshold-based decode check: sum of |LLR| / len > 2.0
        let avg_llr: f32 = proc.soft_buffer.iter().map(|&x| x.abs()).sum::<f32>()
            / proc.soft_buffer.len().max(1) as f32;
        let success = avg_llr > 2.0;
        proc.pending_ack = success;
        Ok(success)
    }

    // ── Timing Advance ───────────────────────────────────────────

    /// Tick by one slot – advances SFN, DRX, PDCCH skip counter, HD timer
    pub fn tick_slot(&mut self) {
        self.current_slot += 1;
        if self.current_slot >= self.slots_per_frame {
            self.current_slot = 0;
            self.sfn = (self.sfn + 1) % 1024;
        }
        // Advance HD FDD timer
        if let Some(ref mut timer) = self.hd_timer {
            timer.tick();
        }
        // Tick DRX by slot duration (approximate as ms)
        let slot_ms = (self.scs.slot_duration_us() / 1000.0).ceil() as u32;
        self.tick_drx(slot_ms.max(1));
    }

    // ── State Machine ────────────────────────────────────────────

    /// Get current UE state
    pub fn state(&self) -> UeState {
        self.state
    }

    /// Transition to RRC IDLE (e.g., after connection release)
    pub fn go_idle(&mut self) {
        self.state = UeState::RrcIdle;
        self.start_drx();
    }

    /// Start initial access procedure
    pub fn start_initial_access(&mut self) -> Result<(), RedCapError> {
        if self.state != UeState::Idle && self.state != UeState::RrcIdle {
            return Err(RedCapError::StateTransitionError {
                from: self.state,
                to: UeState::InitialAccess,
            });
        }
        self.state = UeState::InitialAccess;
        Ok(())
    }

    // ── Peak Data Rate Estimation ────────────────────────────────

    /// Estimate peak DL data rate in Mbps given active BWP and best MCS
    pub fn estimate_peak_dl_rate_mbps(&self) -> f64 {
        let bwp = match self.active_dl_bwp() {
            Some(b) => b,
            None => return 0.0,
        };
        // Best MCS for RedCap: index 27 (64-QAM, CR=0.888)
        let best_mcs = match self.get_mcs(27) {
            Ok(m) => m,
            Err(_) => return 0.0,
        };
        let num_re_per_slot = bwp.num_prbs * SUBCARRIERS_PER_PRB * (SYMBOLS_PER_SLOT - 2); // -2 for DMRS
        let tbs_per_slot = compute_tbs(best_mcs, bwp.num_prbs, SYMBOLS_PER_SLOT - 2);
        let slots_per_second = 1_000_000.0 / self.scs.slot_duration_us();
        let _ = num_re_per_slot; // used implicitly via tbs computation
        (tbs_per_slot as f64 * slots_per_second) / 1_000_000.0
    }

    /// Estimate average power consumption in connected-mode DRX
    pub fn estimate_avg_power(&self) -> f64 {
        self.power_model.average_power(&self.drx_config)
    }
}

// ─────────────────────────────────────────────────────────────────
// Utility functions
// ─────────────────────────────────────────────────────────────────

/// Compute spectral efficiency in bits/s/Hz given bandwidth and data rate
pub fn spectral_efficiency(bandwidth_hz: f64, data_rate_bps: f64) -> f64 {
    if bandwidth_hz <= 0.0 {
        return 0.0;
    }
    data_rate_bps / bandwidth_hz
}

/// Convert MCS index to approximate SNR requirement (dB)
/// Uses a simplified lookup (approximate based on capacity formula)
pub fn mcs_to_snr_db(mcs_index: u8) -> f64 {
    // Approx SNR = 10 * log10(2^se - 1) where se = spectral_efficiency
    let table = redcap_mcs_table();
    if mcs_index as usize >= table.len() {
        return f64::INFINITY;
    }
    let se = table[mcs_index as usize].spectral_efficiency;
    // Shannon approximation (ignoring coding gap)
    10.0 * (f64::exp(se * std::f64::consts::LN_2) - 1.0).log10()
}

/// Compute number of slots for a given duration in ms
pub fn ms_to_slots(duration_ms: f64, scs: SubcarrierSpacing) -> u64 {
    (duration_ms * 1000.0 / scs.slot_duration_us()) as u64
}

/// Compute DRX power saving ratio vs always-on
pub fn drx_power_saving_ratio(drx: &DrxConfig) -> f64 {
    1.0 - drx.duty_cycle_ratio()
}

/// Check if a given bandwidth is valid for a RedCap UE
pub fn is_valid_redcap_bandwidth(bw_hz: u64, fr: FrequencyRange) -> bool {
    let max = match fr {
        FrequencyRange::Fr1 => FR1_REDCAP_MAX_BW_HZ,
        FrequencyRange::Fr2 => FR2_REDCAP_MAX_BW_HZ,
    };
    bw_hz <= max
}

/// Convert PRB count to bandwidth in Hz for a given SCS
pub fn prbs_to_bandwidth_hz(num_prbs: u32, scs: SubcarrierSpacing) -> u64 {
    (num_prbs * SUBCARRIERS_PER_PRB) as u64 * scs.hz() as u64
}

// ─────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: build a valid FR1 RedCap processor at 30 kHz SCS
    fn make_fr1_proc() -> RedCapProcessor {
        RedCapProcessor::new(RedCapCapabilities::fr1_wearable(), SubcarrierSpacing::Scs30kHz)
    }

    // ── Capability validation ──────────────────────────────────────

    #[test]
    fn test_fr1_wearable_capabilities_valid() {
        let caps = RedCapCapabilities::fr1_wearable();
        assert!(caps.validate().is_ok());
    }

    #[test]
    fn test_fr2_video_capabilities_valid() {
        let caps = RedCapCapabilities::fr2_video_surveillance();
        assert!(caps.validate().is_ok());
    }

    #[test]
    fn test_fr1_industrial_sensor_valid() {
        let caps = RedCapCapabilities::fr1_industrial_sensor();
        assert!(caps.validate().is_ok());
    }

    #[test]
    fn test_capabilities_exceed_bandwidth_rejected() {
        let mut caps = RedCapCapabilities::fr1_wearable();
        caps.max_bw_hz = 50_000_000; // 50 MHz > 20 MHz limit
        assert!(matches!(
            caps.validate(),
            Err(RedCapError::BwpTooWide { .. })
        ));
    }

    #[test]
    fn test_capabilities_too_many_antennas_rejected() {
        let mut caps = RedCapCapabilities::fr1_wearable();
        caps.num_rx_antennas = 2;
        assert!(matches!(
            caps.validate(),
            Err(RedCapError::ExceedsAntennaLimit { .. })
        ));
    }

    #[test]
    fn test_capabilities_256qam_rejected() {
        let mut caps = RedCapCapabilities::fr1_wearable();
        caps.max_dl_modulation = Modulation::Qam256;
        assert!(matches!(
            caps.validate(),
            Err(RedCapError::ModulationNotAllowed(_))
        ));
    }

    // ── BWP management ─────────────────────────────────────────────

    #[test]
    fn test_valid_fr1_bwp_15khz_106prbs() {
        let bwp = BwpConfig {
            bwp_id: 0,
            bwp_type: BwpType::InitialDl,
            start_prb: 0,
            num_prbs: 106,
            scs: SubcarrierSpacing::Scs15kHz,
            is_active: true,
        };
        assert!(bwp.validate_for_redcap(FrequencyRange::Fr1).is_ok());
    }

    #[test]
    fn test_valid_fr1_bwp_30khz_51prbs() {
        let bwp = BwpConfig {
            bwp_id: 0,
            bwp_type: BwpType::InitialDl,
            start_prb: 0,
            num_prbs: 51,
            scs: SubcarrierSpacing::Scs30kHz,
            is_active: true,
        };
        assert!(bwp.validate_for_redcap(FrequencyRange::Fr1).is_ok());
    }

    #[test]
    fn test_bwp_too_many_prbs_rejected() {
        let bwp = BwpConfig {
            bwp_id: 0,
            bwp_type: BwpType::InitialDl,
            start_prb: 0,
            num_prbs: 52, // exceeds 51 for 30 kHz SCS
            scs: SubcarrierSpacing::Scs30kHz,
            is_active: true,
        };
        assert!(matches!(
            bwp.validate_for_redcap(FrequencyRange::Fr1),
            Err(RedCapError::TooManyPrbs { .. })
        ));
    }

    #[test]
    fn test_fr2_bwp_valid_120khz() {
        let bwp = BwpConfig {
            bwp_id: 0,
            bwp_type: BwpType::InitialDl,
            start_prb: 0,
            num_prbs: 66,
            scs: SubcarrierSpacing::Scs120kHz,
            is_active: true,
        };
        assert!(bwp.validate_for_redcap(FrequencyRange::Fr2).is_ok());
    }

    #[test]
    fn test_bwp_bandwidth_calculation() {
        let bwp = BwpConfig {
            bwp_id: 0,
            bwp_type: BwpType::InitialDl,
            start_prb: 0,
            num_prbs: 51,
            scs: SubcarrierSpacing::Scs30kHz,
            is_active: true,
        };
        // 51 * 12 * 30000 = 18,360,000 Hz ≈ 18.36 MHz (within 20 MHz)
        let bw = bwp.bandwidth_hz();
        assert!(bw <= FR1_REDCAP_MAX_BW_HZ, "BW {} > 20 MHz", bw);
        assert_eq!(bw, 51 * 12 * 30_000);
    }

    #[test]
    fn test_add_and_switch_dl_bwp() {
        let mut proc = make_fr1_proc();
        let bwp0 = BwpConfig {
            bwp_id: 0,
            bwp_type: BwpType::InitialDl,
            start_prb: 0,
            num_prbs: 51,
            scs: SubcarrierSpacing::Scs30kHz,
            is_active: true,
        };
        let bwp1 = BwpConfig {
            bwp_id: 1,
            bwp_type: BwpType::DedicatedDl,
            start_prb: 10,
            num_prbs: 25,
            scs: SubcarrierSpacing::Scs30kHz,
            is_active: false,
        };
        proc.add_dl_bwp(bwp0).unwrap();
        proc.add_dl_bwp(bwp1).unwrap();
        proc.switch_dl_bwp(1).unwrap();
        assert_eq!(proc.active_dl_bwp, 1);
    }

    #[test]
    fn test_switch_to_nonexistent_bwp_fails() {
        let mut proc = make_fr1_proc();
        let result = proc.switch_dl_bwp(99);
        assert!(matches!(result, Err(RedCapError::BwpNotFound(99))));
    }

    // ── DRX configuration ──────────────────────────────────────────

    #[test]
    fn test_drx_power_save_valid() {
        let drx = DrxConfig::redcap_power_save();
        assert!(drx.validate().is_ok());
    }

    #[test]
    fn test_drx_balanced_valid() {
        let drx = DrxConfig::redcap_balanced();
        assert!(drx.validate().is_ok());
    }

    #[test]
    fn test_drx_cycle_too_long_rejected() {
        let mut drx = DrxConfig::redcap_balanced();
        drx.long_cycle_ms = 20_480; // > 10240 ms
        assert!(matches!(
            drx.validate(),
            Err(RedCapError::DrxCycleTooLong { .. })
        ));
    }

    #[test]
    fn test_drx_short_cycle_not_less_than_long_rejected() {
        let mut drx = DrxConfig::redcap_balanced();
        drx.short_cycle_ms = Some(drx.long_cycle_ms + 100);
        assert!(matches!(drx.validate(), Err(RedCapError::InvalidDrxConfig(_))));
    }

    #[test]
    fn test_drx_duty_cycle() {
        let drx = DrxConfig {
            on_duration_ms: 20,
            long_cycle_ms: 1000,
            ..DrxConfig::redcap_balanced()
        };
        let duty = drx.duty_cycle_ratio();
        assert!((duty - 0.02).abs() < 1e-9);
    }

    #[test]
    fn test_drx_tick_wakes_at_cycle_end() {
        let mut proc = make_fr1_proc();
        let drx = DrxConfig {
            on_duration_ms: 10,
            inactivity_timer_ms: 10,
            short_cycle_ms: None,
            long_cycle_ms: 100,
            short_cycle_timer: None,
            harq_rtt_timer_dl_ms: 8,
            harq_rtt_timer_ul_ms: 8,
            retransmission_timer_dl_ms: 16,
            retransmission_timer_ul_ms: 16,
        };
        proc.configure_drx(drx).unwrap();
        proc.start_drx();
        // Fast-forward past the cycle
        let (_, woke) = proc.tick_drx(101);
        assert!(woke);
    }

    // ── Half-duplex FDD ────────────────────────────────────────────

    #[test]
    fn test_hd_fdd_supported_for_wearable() {
        let proc = make_fr1_proc();
        assert!(proc.capabilities.hd_fdd_supported);
        assert!(proc.hd_config.is_some());
    }

    #[test]
    fn test_hd_fdd_not_supported_for_fr2() {
        let caps = RedCapCapabilities::fr2_video_surveillance();
        let proc = RedCapProcessor::new(caps, SubcarrierSpacing::Scs120kHz);
        assert!(!proc.capabilities.hd_fdd_supported);
        assert!(proc.hd_config.is_none());
    }

    #[test]
    fn test_hd_guard_period_configurable() {
        let mut proc = make_fr1_proc();
        assert!(proc.configure_half_duplex(2).is_ok());
        assert!(proc.configure_half_duplex(0).is_err()); // < minimum
    }

    #[test]
    fn test_hd_fdd_timer_switch() {
        let mut proc = make_fr1_proc();
        let guard_us = proc.request_tx_mode();
        assert!(guard_us.is_some());
        let guard_us2 = proc.request_rx_mode();
        assert!(guard_us2.is_some());
    }

    #[test]
    fn test_hd_timer_tick() {
        let mut timer = HdFddTimer::new(SubcarrierSpacing::Scs30kHz);
        assert_eq!(timer.slots_per_frame, 20); // 2 slots/sf * 10 sf
        timer.request_tx(1);
        let switched = timer.tick(); // guard expires → switches
        assert!(switched);
    }

    // ── PDCCH monitoring ───────────────────────────────────────────

    #[test]
    fn test_pdcch_skipping_reduces_monitoring() {
        let mut proc = make_fr1_proc();
        let config = PdcchMonitorConfig::with_skipping(SubcarrierSpacing::Scs30kHz);
        proc.configure_pdcch_monitoring(config);
        // Count how many slots have monitoring in first 20 slots
        let mut count = 0;
        for _ in 0..20 {
            if proc.should_monitor_pdcch() {
                count += 1;
            }
        }
        assert!(count < 20, "Expected fewer monitoring occasions than 20, got {}", count);
    }

    #[test]
    fn test_pdcch_no_skipping_monitors_all() {
        let mut proc = make_fr1_proc();
        let config = PdcchMonitorConfig::standard(SubcarrierSpacing::Scs30kHz);
        proc.configure_pdcch_monitoring(config);
        let count = (0..10).filter(|_| proc.should_monitor_pdcch()).count();
        assert_eq!(count, 10);
    }

    // ── RRM relaxation ────────────────────────────────────────────

    #[test]
    fn test_rrm_relaxation_for_stationary() {
        let cfg = RrmRelaxConfig::stationary_low_power();
        assert!(cfg.relaxation_allowed());
    }

    #[test]
    fn test_rrm_no_relaxation_for_mobile() {
        let cfg = RrmRelaxConfig::mobile_device();
        assert!(!cfg.relaxation_allowed());
    }

    #[test]
    fn test_rrm_effective_period_multiplied_when_relaxed() {
        let cfg = RrmRelaxConfig::stationary_low_power();
        let base = 80u32;
        let eff = cfg.effective_period_ms(base);
        assert_eq!(eff, base * cfg.period_multiplier);
    }

    #[test]
    fn test_rrm_effective_period_unchanged_when_not_relaxed() {
        let cfg = RrmRelaxConfig::mobile_device();
        let base = 80u32;
        let eff = cfg.effective_period_ms(base);
        assert_eq!(eff, base);
    }

    // ── MCS table ────────────────────────────────────────────────

    #[test]
    fn test_mcs_table_has_correct_entries() {
        let table = redcap_mcs_table();
        assert_eq!(table.len(), 28); // 0-27 for 64-QAM table
    }

    #[test]
    fn test_mcs_table_64qam_max() {
        let table = redcap_mcs_table();
        let max_order = table.iter().map(|e| e.mod_order).max().unwrap();
        assert_eq!(max_order, 6, "Max mod order should be 6 (64-QAM)");
    }

    #[test]
    fn test_get_mcs_valid_index() {
        let proc = make_fr1_proc();
        let mcs = proc.get_mcs(17);
        assert!(mcs.is_ok());
        assert_eq!(mcs.unwrap().mod_order, 6); // 64-QAM starts at index 17
    }

    #[test]
    fn test_get_mcs_invalid_index() {
        let proc = make_fr1_proc();
        assert!(matches!(proc.get_mcs(28), Err(RedCapError::InvalidMcsIndex(28))));
        assert!(matches!(proc.get_mcs(255), Err(RedCapError::InvalidMcsIndex(255))));
    }

    #[test]
    fn test_select_mcs_returns_lowest_valid() {
        let proc = make_fr1_proc();
        let mcs = proc.select_mcs(Modulation::Qam64, 1.0);
        assert!(mcs.is_some());
        assert!(mcs.unwrap().spectral_efficiency >= 1.0);
    }

    // ── TBS computation ───────────────────────────────────────────

    #[test]
    fn test_tbs_positive() {
        let table = redcap_mcs_table();
        let mcs = &table[17]; // 64-QAM
        let tbs = compute_tbs(mcs, 51, 12);
        assert!(tbs > 0);
    }

    #[test]
    fn test_tbs_byte_aligned() {
        let table = redcap_mcs_table();
        let mcs = &table[10]; // 16-QAM
        let tbs = compute_tbs(mcs, 25, 12);
        assert_eq!(tbs % 8, 0);
    }

    #[test]
    fn test_tbs_increases_with_prbs() {
        let table = redcap_mcs_table();
        let mcs = &table[17];
        let tbs_small = compute_tbs(mcs, 10, 12);
        let tbs_large = compute_tbs(mcs, 51, 12);
        assert!(tbs_large > tbs_small);
    }

    // ── HARQ ────────────────────────────────────────────────────

    #[test]
    fn test_harq_processes_exist() {
        let proc = make_fr1_proc();
        assert!(proc.harq_process(0).is_ok());
        assert!(proc.harq_process(15).is_ok());
        assert!(matches!(proc.harq_process(16), Err(RedCapError::HarqPidOutOfRange(16))));
    }

    #[test]
    fn test_harq_process_reset() {
        let mut proc = make_fr1_proc();
        let hp = proc.harq_process_mut(0).unwrap();
        hp.rv = 2;
        hp.retx_count = 3;
        hp.reset();
        assert_eq!(hp.rv, 0);
        assert_eq!(hp.retx_count, 0);
    }

    #[test]
    fn test_harq_soft_combining() {
        let mut proc = make_fr1_proc();
        let llrs1: Vec<f32> = vec![1.5; 100];
        let llrs2: Vec<f32> = vec![1.5; 100];
        // First transmission: ndi=true (new data)
        proc.process_dl_harq(0, &llrs1, true).unwrap();
        // Retransmission: ndi stays true (same transmission block, NDI unchanged)
        proc.process_dl_harq(0, &llrs2, true).unwrap();
        let hp = proc.harq_process(0).unwrap();
        // After combining, buffer values should be doubled (1.5 + 1.5 = 3.0)
        let avg: f32 = hp.soft_buffer[..100].iter().sum::<f32>() / 100.0;
        assert!((avg - 3.0).abs() < 0.1, "avg={}", avg);
    }

    // ── Initial access ────────────────────────────────────────────

    #[test]
    fn test_build_msg3_sets_redcap_indicator() {
        let mut proc = make_fr1_proc();
        proc.start_initial_access().unwrap();
        let msg3 = proc.build_msg3();
        assert!(msg3.redcap_indicator);
    }

    #[test]
    fn test_initial_access_state_transition() {
        let mut proc = make_fr1_proc();
        assert_eq!(proc.state(), UeState::Idle);
        proc.start_initial_access().unwrap();
        assert_eq!(proc.state(), UeState::InitialAccess);
    }

    #[test]
    fn test_process_msg4_transitions_to_connected() {
        let mut proc = make_fr1_proc();
        proc.start_initial_access().unwrap();
        proc.build_msg3();
        assert!(proc.process_msg4().is_ok());
        assert_eq!(proc.state(), UeState::RrcConnected);
    }

    #[test]
    fn test_invalid_state_transition_rejected() {
        let mut proc = make_fr1_proc();
        // Cannot process Msg4 from Idle
        let result = proc.process_msg4();
        assert!(matches!(result, Err(RedCapError::StateTransitionError { .. })));
    }

    // ── Modulation ────────────────────────────────────────────────

    #[test]
    fn test_modulation_bits_per_symbol() {
        assert_eq!(Modulation::Qpsk.bits_per_symbol(), 2);
        assert_eq!(Modulation::Qam16.bits_per_symbol(), 4);
        assert_eq!(Modulation::Qam64.bits_per_symbol(), 6);
        assert_eq!(Modulation::Qam256.bits_per_symbol(), 8);
    }

    #[test]
    fn test_modulation_redcap_allowed() {
        assert!(Modulation::Qpsk.is_redcap_allowed());
        assert!(Modulation::Qam16.is_redcap_allowed());
        assert!(Modulation::Qam64.is_redcap_allowed());
        assert!(!Modulation::Qam256.is_redcap_allowed());
    }

    // ── SCS utilities ────────────────────────────────────────────

    #[test]
    fn test_scs_slot_durations() {
        // Slot duration = 1000 µs / slots_per_subframe
        // 15 kHz (µ=0): 1 slot/subframe → 1000 µs/slot
        let d15 = SubcarrierSpacing::Scs15kHz.slot_duration_us();
        assert!((d15 - 1000.0).abs() < 1e-9, "d15={}", d15);
        // 30 kHz (µ=1): 2 slots/subframe → 500 µs/slot
        let d30 = SubcarrierSpacing::Scs30kHz.slot_duration_us();
        assert!((d30 - 500.0).abs() < 1e-9, "d30={}", d30);
        // 120 kHz (µ=3): 8 slots/subframe → 125 µs/slot
        let d120 = SubcarrierSpacing::Scs120kHz.slot_duration_us();
        assert!((d120 - 125.0).abs() < 1e-9, "d120={}", d120);
    }

    #[test]
    fn test_scs_slots_per_subframe() {
        assert_eq!(SubcarrierSpacing::Scs15kHz.slots_per_subframe(), 1);
        assert_eq!(SubcarrierSpacing::Scs30kHz.slots_per_subframe(), 2);
        assert_eq!(SubcarrierSpacing::Scs60kHz.slots_per_subframe(), 4);
        assert_eq!(SubcarrierSpacing::Scs120kHz.slots_per_subframe(), 8);
    }

    // ── Utility functions ─────────────────────────────────────────

    #[test]
    fn test_is_valid_redcap_bandwidth() {
        assert!(is_valid_redcap_bandwidth(20_000_000, FrequencyRange::Fr1));
        assert!(!is_valid_redcap_bandwidth(25_000_000, FrequencyRange::Fr1));
        assert!(is_valid_redcap_bandwidth(100_000_000, FrequencyRange::Fr2));
        assert!(!is_valid_redcap_bandwidth(200_000_000, FrequencyRange::Fr2));
    }

    #[test]
    fn test_prbs_to_bandwidth() {
        let bw = prbs_to_bandwidth_hz(51, SubcarrierSpacing::Scs30kHz);
        assert_eq!(bw, 51 * 12 * 30_000);
    }

    #[test]
    fn test_ms_to_slots_30khz() {
        // 1 ms = 2 slots at 30 kHz
        let slots = ms_to_slots(1.0, SubcarrierSpacing::Scs30kHz);
        assert_eq!(slots, 2);
    }

    #[test]
    fn test_ms_to_slots_120khz() {
        // 1 ms = 8 slots at 120 kHz
        let slots = ms_to_slots(1.0, SubcarrierSpacing::Scs120kHz);
        assert_eq!(slots, 8);
    }

    #[test]
    fn test_spectral_efficiency_calculation() {
        let se = spectral_efficiency(20_000_000.0, 85_000_000.0);
        assert!((se - 4.25).abs() < 0.01);
    }

    #[test]
    fn test_drx_power_saving_ratio() {
        let drx = DrxConfig {
            on_duration_ms: 10,
            long_cycle_ms: 1000,
            ..DrxConfig::redcap_balanced()
        };
        let ratio = drx_power_saving_ratio(&drx);
        assert!((ratio - 0.99).abs() < 1e-9);
    }

    #[test]
    fn test_mcs_to_snr_db_increases_with_index() {
        let snr0 = mcs_to_snr_db(0);
        let snr10 = mcs_to_snr_db(10);
        let snr27 = mcs_to_snr_db(27);
        assert!(snr10 > snr0, "SNR should increase with MCS index");
        assert!(snr27 > snr10, "SNR should increase with MCS index");
    }

    // ── Peak data rate ────────────────────────────────────────────

    #[test]
    fn test_peak_dl_rate_with_bwp() {
        let mut proc = make_fr1_proc();
        let bwp = BwpConfig {
            bwp_id: 0,
            bwp_type: BwpType::DedicatedDl,
            start_prb: 0,
            num_prbs: 51,
            scs: SubcarrierSpacing::Scs30kHz,
            is_active: true,
        };
        proc.add_dl_bwp(bwp).unwrap();
        proc.active_dl_bwp = 0;
        let rate = proc.estimate_peak_dl_rate_mbps();
        assert!(rate > 0.0, "Expected positive rate, got {}", rate);
        // Should be in reasonable range for 20 MHz, 64-QAM
        assert!(rate < 200.0, "Rate {} Mbps seems too high", rate);
    }

    #[test]
    fn test_peak_dl_rate_without_bwp_is_zero() {
        let proc = make_fr1_proc();
        let rate = proc.estimate_peak_dl_rate_mbps();
        assert_eq!(rate, 0.0);
    }

    // ── Power model ───────────────────────────────────────────────

    #[test]
    fn test_average_power_in_drx() {
        let proc = make_fr1_proc();
        let avg = proc.estimate_avg_power();
        assert!(avg > 0.0);
        assert!(avg <= 1.0, "Average power should be normalized <= 1.0, got {}", avg);
    }

    // ── RRM measurement recording ─────────────────────────────────

    #[test]
    fn test_measurement_recording_and_average() {
        let mut proc = make_fr1_proc();
        for i in 0..5 {
            proc.record_measurement(SsbMeasurement {
                ssb_index: 0,
                rsrp_dbm: -80.0 + i as f64,
                rsrq_db: -10.0,
                sinr_db: 15.0,
                timestamp_slot: i as u64 * 20,
            });
        }
        let avg = proc.average_rsrp_dbm().unwrap();
        assert!((avg - (-78.0)).abs() < 1e-9, "avg={}", avg);
    }

    #[test]
    fn test_measurement_buffer_capped_at_16() {
        let mut proc = make_fr1_proc();
        for i in 0..20 {
            proc.record_measurement(SsbMeasurement {
                ssb_index: 0,
                rsrp_dbm: -80.0,
                rsrq_db: -10.0,
                sinr_db: 15.0,
                timestamp_slot: i,
            });
        }
        assert!(proc.measurements.len() <= 16);
    }

    // ── Slot ticking ──────────────────────────────────────────────

    #[test]
    fn test_slot_tick_advances_sfn() {
        let mut proc = make_fr1_proc();
        let initial_sfn = proc.sfn;
        // Tick enough slots to advance SFN by 1 (20 slots at 30 kHz = 10 ms = 1 frame)
        for _ in 0..20 {
            proc.tick_slot();
        }
        assert_eq!(proc.sfn, (initial_sfn + 1) % 1024);
    }

    #[test]
    fn test_slot_counter_wraps_at_frame_boundary() {
        let mut proc = make_fr1_proc();
        for _ in 0..proc.slots_per_frame {
            proc.tick_slot();
        }
        assert_eq!(proc.current_slot, 0);
    }

    // ── Go idle ───────────────────────────────────────────────────

    #[test]
    fn test_go_idle_starts_drx() {
        let mut proc = make_fr1_proc();
        proc.state = UeState::RrcConnected;
        proc.go_idle();
        assert_eq!(proc.state(), UeState::RrcIdle);
        assert_ne!(proc.drx_state, DrxState::Inactive);
    }
}

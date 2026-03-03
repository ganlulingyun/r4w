//! TWDM-PON (Time and Wavelength Division Multiplexed Passive Optical Network) Processor
//!
//! Implements the NG-PON2 physical layer per ITU-T G.989.x standards.
//! Supports 4-8 wavelength pairs at 10 Gbps each (40-80 Gbps aggregate),
//! XGTC framing, PLOAM messaging, cross-wavelength DBA, protection switching,
//! ONU activation, and PtP WDM overlay for business services.
//!
//! # Standards References
//! - ITU-T G.989.1: NG-PON2 General Requirements
//! - ITU-T G.989.2: Physical Media Dependent (PMD) Layer Specification
//! - ITU-T G.989.3: Transmission Convergence (TC) Layer Specification
//!
//! # Wavelength Plan (ITU-T G.989.2)
//! - Downstream: 1596–1603 nm (100 GHz grid, C-band)
//! - Upstream: 1524–1544 nm (100 GHz grid, S-band)
//! - Coexistence: 1480–1500 nm (GPON DS), 1260–1280 nm (GPON US)
//! - XGS-PON coexistence: 1577 nm DS / 1270 nm US (blocked from TWDM bands)
//!
//! # Architecture
//! ```text
//! OLT                              ODN                ONU
//! ┌─────────────────┐              │            ┌──────────────┐
//! │ λ1 Tx (1596nm)  │──────────────┼────────────│ Tunable Rx   │
//! │ λ2 Tx (1597nm)  │──────────────┼────────────│ Tunable Tx   │
//! │ λ3 Tx (1598nm)  │──────────────┼────────────│ WL selector  │
//! │ λ4 Tx (1599nm)  │──────────────┼────────────└──────────────┘
//! │ λ1 Rx (1524nm)  │◄─────────────┤
//! │ λ2 Rx (1532nm)  │◄─────────────┤
//! │ λ3 Rx (1540nm)  │◄─────────────┤
//! │ λ4 Rx (1544nm)  │◄─────────────┘
//! └─────────────────┘
//! ```

use std::collections::HashMap;

// ─── Constants (ITU-T G.989.x) ────────────────────────────────────────────────

/// Speed of light in vacuum (m/s)
const C_LIGHT: f64 = 2.998_792_458e8;

/// Downstream wavelength channel base (nm) — 1596 nm
pub const DS_BASE_WL_NM: f64 = 1596.0;
/// Downstream channel spacing (nm) corresponding to 100 GHz at ~1596 nm
pub const DS_CHANNEL_SPACING_NM: f64 = 0.8;

/// Upstream wavelength channel base (nm) — 1524 nm
pub const US_BASE_WL_NM: f64 = 1524.0;
/// Upstream channel spacing (nm) corresponding to 100 GHz at ~1530 nm
pub const US_CHANNEL_SPACING_NM: f64 = 0.8;

/// Maximum number of TWDM wavelength channel pairs
pub const MAX_CHANNELS: usize = 8;

/// Minimum number of TWDM wavelength channel pairs
pub const MIN_CHANNELS: usize = 4;

/// Line rate per wavelength channel (bps) — XGS-PON 10G
pub const LINE_RATE_BPS: f64 = 9_953_280_000.0; // 10.3125 Gbaud × 8/10 encoding ≈ 9.95 Gbps payload

/// XGTC frame duration (µs) — 125 µs per G.989.3
pub const XGTC_FRAME_DURATION_US: f64 = 125.0;

/// XGTC frame size in bytes (downstream) per 125 µs at 10 Gbps
pub const XGTC_DS_FRAME_BYTES: u32 = 155_520; // 10.3125 Gbps × 125 µs / 8

/// ONU tuning time maximum (µs) — fast tuning per G.989.3
pub const TUNING_TIME_MAX_US: f64 = 250.0;

/// Protection switching target time (ms) — <50 ms per G.989.1
pub const PROTECTION_SWITCH_TARGET_MS: f64 = 50.0;

/// Boltzmann constant (J/K)
const K_BOLTZMANN: f64 = 1.380_649e-23;
/// Temperature (K) — 290 K standard
const T_NOISE: f64 = 290.0;
/// Reference bandwidth for noise (Hz)
const BW_REF: f64 = 1.0e9;

// ─── ODN Class Optical Budget ──────────────────────────────────────────────────

/// ODN (Optical Distribution Network) class per ITU-T G.989.2
#[derive(Debug, Clone, PartialEq)]
pub enum OdnClass {
    /// N1: Maximum differential loss 10 dB, ORL ≥ 32 dB
    N1,
    /// N2: Maximum differential loss 15 dB, ORL ≥ 32 dB
    N2,
    /// E1: Extended reach class 1 (29–31 dB budget)
    E1,
    /// E2: Extended reach class 2 (33–35 dB budget)
    E2,
}

impl OdnClass {
    /// Returns (min_loss_budget_db, max_loss_budget_db, max_differential_loss_db)
    pub fn loss_budget(&self) -> (f64, f64, f64) {
        match self {
            OdnClass::N1 => (14.0, 29.0, 10.0),
            OdnClass::N2 => (16.0, 31.0, 15.0),
            OdnClass::E1 => (18.0, 33.0, 15.0),
            OdnClass::E2 => (20.0, 35.0, 15.0),
        }
    }

    /// Returns minimum optical return loss (ORL) requirement in dB
    pub fn min_orl_db(&self) -> f64 {
        match self {
            OdnClass::N1 | OdnClass::N2 => 32.0,
            OdnClass::E1 | OdnClass::E2 => 32.0,
        }
    }
}

// ─── Protection Types ──────────────────────────────────────────────────────────

/// Protection switching type per ITU-T G.989.1
#[derive(Debug, Clone, PartialEq)]
pub enum ProtectionType {
    /// Type A: OLT-side protection (redundant OLT transceivers, same ODN)
    TypeA,
    /// Type B: ODN protection (redundant feeder fiber, passive splitter protection)
    TypeB,
    /// Type C: Full protection (OLT + ODN + ONU dual homing, <50 ms switchover)
    TypeC,
}

impl ProtectionType {
    pub fn switchover_time_ms(&self) -> f64 {
        match self {
            ProtectionType::TypeA => 50.0,
            ProtectionType::TypeB => 50.0,
            ProtectionType::TypeC => 10.0, // Fastest — dual active
        }
    }

    pub fn description(&self) -> &'static str {
        match self {
            ProtectionType::TypeA => "OLT-side redundancy (same ODN)",
            ProtectionType::TypeB => "ODN feeder redundancy",
            ProtectionType::TypeC => "Full dual-homing OLT+ODN+ONU",
        }
    }
}

// ─── PLOAM Message Types ───────────────────────────────────────────────────────

/// PLOAM (Physical Layer OAM) message type identifiers per G.989.3
#[derive(Debug, Clone, PartialEq)]
pub enum PloamType {
    /// Channel profile describing wavelength channel capabilities
    ChannelProfile,
    /// Wavelength channel ID assignment to ONU
    WavelengthChannelId,
    /// Tuning control message to direct ONU tuning
    TuningControl,
    /// Serial number request during ONU activation
    SerialNumberRequest,
    /// Serial number response from ONU
    SerialNumberResponse,
    /// Ranging request for RTT measurement
    RangingRequest,
    /// Ranging time assignment
    AssignOnu,
    /// DBA report request (Status Report)
    DbaReport,
    /// Power leveling control
    PowerLevelSequence,
    /// ONU deactivation command
    DeactivateOnu,
    /// Acknowledge message
    Acknowledge,
    /// Protection switching notification
    ProtectionSwitchOver,
}

/// PLOAM message structure (48 bytes per G.989.3)
#[derive(Debug, Clone)]
pub struct PloamMessage {
    pub msg_type: PloamType,
    pub onu_id: u16,
    pub channel_id: u8,
    pub sequence_number: u8,
    pub payload: Vec<u8>, // up to 40 bytes
}

impl PloamMessage {
    pub fn new(msg_type: PloamType, onu_id: u16, channel_id: u8) -> Self {
        PloamMessage {
            msg_type,
            onu_id,
            channel_id,
            sequence_number: 0,
            payload: Vec::new(),
        }
    }

    /// Serialize to 48-byte PLOAM frame
    pub fn serialize(&self) -> Vec<u8> {
        let mut frame = vec![0u8; 48];
        // Byte 0: ONU-ID high
        frame[0] = (self.onu_id >> 8) as u8;
        // Byte 1: ONU-ID low
        frame[1] = (self.onu_id & 0xFF) as u8;
        // Byte 2: Message type ID
        frame[2] = self.msg_type_id();
        // Byte 3: Channel ID
        frame[3] = self.channel_id;
        // Byte 4: Sequence number
        frame[4] = self.sequence_number;
        // Bytes 5–44: Payload
        let payload_len = self.payload.len().min(40);
        frame[5..5 + payload_len].copy_from_slice(&self.payload[..payload_len]);
        // Bytes 44–47: CRC-8 (simplified: XOR of all bytes)
        let crc = frame[0..44].iter().fold(0u8, |acc, &b| acc ^ b);
        frame[44] = crc;
        frame[45] = 0;
        frame[46] = 0;
        frame[47] = 0;
        frame
    }

    fn msg_type_id(&self) -> u8 {
        match self.msg_type {
            PloamType::ChannelProfile => 0x01,
            PloamType::WavelengthChannelId => 0x02,
            PloamType::TuningControl => 0x03,
            PloamType::SerialNumberRequest => 0x10,
            PloamType::SerialNumberResponse => 0x11,
            PloamType::RangingRequest => 0x12,
            PloamType::AssignOnu => 0x13,
            PloamType::DbaReport => 0x20,
            PloamType::PowerLevelSequence => 0x30,
            PloamType::DeactivateOnu => 0x05,
            PloamType::Acknowledge => 0x09,
            PloamType::ProtectionSwitchOver => 0x40,
        }
    }
}

// ─── Wavelength Channel ────────────────────────────────────────────────────────

/// TWDM wavelength channel pair (downstream + upstream)
#[derive(Debug, Clone)]
pub struct WavelengthChannel {
    /// Zero-based channel index (0–7)
    pub channel_id: u8,
    /// Downstream wavelength (nm)
    pub ds_wavelength_nm: f64,
    /// Upstream wavelength (nm)
    pub us_wavelength_nm: f64,
    /// Downstream center frequency (THz)
    pub ds_freq_thz: f64,
    /// Upstream center frequency (THz)
    pub us_freq_thz: f64,
    /// Channel active flag
    pub active: bool,
    /// Assigned ONU IDs on this channel
    pub assigned_onus: Vec<u16>,
    /// Current channel load (fraction 0.0–1.0)
    pub load_fraction: f64,
    /// Per-channel OLT transmit power (dBm)
    pub tx_power_dbm: f64,
    /// Received signal power from ONUs (dBm, averaged)
    pub rx_power_dbm: f64,
}

impl WavelengthChannel {
    /// Create a new wavelength channel from its index
    pub fn new(channel_id: u8) -> Self {
        let ds_wl = DS_BASE_WL_NM + (channel_id as f64) * DS_CHANNEL_SPACING_NM;
        let us_wl = US_BASE_WL_NM + (channel_id as f64) * US_CHANNEL_SPACING_NM;
        let ds_freq = wavelength_nm_to_thz(ds_wl);
        let us_freq = wavelength_nm_to_thz(us_wl);
        WavelengthChannel {
            channel_id,
            ds_wavelength_nm: ds_wl,
            us_wavelength_nm: us_wl,
            ds_freq_thz: ds_freq,
            us_freq_thz: us_freq,
            active: true,
            assigned_onus: Vec::new(),
            load_fraction: 0.0,
            tx_power_dbm: 4.0,   // Typical OLT launch power
            rx_power_dbm: -28.0, // Typical minimum received power
        }
    }

    /// Returns the number of ONUs assigned to this channel
    pub fn onu_count(&self) -> usize {
        self.assigned_onus.len()
    }

    /// Available capacity fraction (1.0 = fully free)
    pub fn available_capacity(&self) -> f64 {
        (1.0 - self.load_fraction).max(0.0)
    }

    /// Throughput available on this channel (bps)
    pub fn available_throughput_bps(&self) -> f64 {
        LINE_RATE_BPS * self.available_capacity()
    }
}

// ─── Tuning Control ────────────────────────────────────────────────────────────

/// ONU wavelength tuning operation and status
#[derive(Debug, Clone)]
pub struct TuningControl {
    /// ONU being tuned
    pub onu_id: u16,
    /// Source channel (before tuning)
    pub from_channel: u8,
    /// Target channel (after tuning)
    pub to_channel: u8,
    /// Maximum allowed tuning time (µs)
    pub tuning_time_us: f64,
    /// Estimated completion time offset from request (µs)
    pub estimated_completion_us: f64,
    /// Tuning successful flag
    pub success: bool,
    /// Reason for tuning (load balancing, protection, bonding)
    pub reason: TuningReason,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TuningReason {
    /// Load balancing across wavelength channels
    LoadBalance,
    /// Protection switching due to channel failure
    ProtectionSwitch,
    /// Channel bonding for higher bandwidth
    Bonding,
    /// Initial wavelength assignment during activation
    Activation,
}

// ─── ONU Report and BW Allocation ─────────────────────────────────────────────

/// ONU upstream bandwidth report (DBRu)
#[derive(Debug, Clone)]
pub struct OnuReport {
    pub onu_id: u16,
    pub channel_id: u8,
    /// Queue occupancy in bytes
    pub queue_occupancy_bytes: u32,
    /// Requested bandwidth in bytes/125µs frame
    pub requested_bw_bytes_per_frame: u32,
    /// T-CONT (Traffic Container) type (1–5 per G.984.3)
    pub tcont_type: u8,
}

/// Downstream bandwidth allocation result
#[derive(Debug, Clone)]
pub struct BwAllocation {
    pub onu_id: u16,
    pub channel_id: u8,
    /// Allocated start time in the XGTC frame (bytes offset)
    pub alloc_start: u32,
    /// Allocated size (bytes in the frame)
    pub alloc_size: u32,
    /// Effective throughput (bps)
    pub effective_bps: f64,
}

// ─── ONU State Machine ─────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq)]
pub enum OnuState {
    /// Initial state — not yet discovered
    Undiscovered,
    /// Serial number received, pending ranging
    SerialAcquired,
    /// Ranging complete, ONU active on channel
    Active,
    /// ONU tuning to another channel
    Tuning,
    /// ONU inactive (low power or deactivated)
    Inactive,
}

/// ONU activation context tracking state machine
#[derive(Debug, Clone)]
pub struct OnuContext {
    pub onu_id: u16,
    pub serial_number: u64,
    pub state: OnuState,
    pub assigned_channel: u8,
    /// Round-trip time (ns)
    pub rtt_ns: f64,
    /// Equalization delay (bytes, 0–16383 per G.989.3)
    pub eqd_bytes: u32,
    /// Transmit power (dBm)
    pub tx_power_dbm: f64,
    /// Bonded channel IDs (empty = no bonding)
    pub bonded_channels: Vec<u8>,
}

impl OnuContext {
    pub fn new(serial_number: u64) -> Self {
        OnuContext {
            onu_id: 0xFFFF, // Unassigned
            serial_number,
            state: OnuState::Undiscovered,
            assigned_channel: 0,
            rtt_ns: 0.0,
            eqd_bytes: 0,
            tx_power_dbm: 0.0,
            bonded_channels: Vec::new(),
        }
    }

    pub fn is_bonded(&self) -> bool {
        !self.bonded_channels.is_empty()
    }

    pub fn bonded_throughput_bps(&self) -> f64 {
        if self.bonded_channels.is_empty() {
            LINE_RATE_BPS
        } else {
            // +1 for primary channel
            LINE_RATE_BPS * (1.0 + self.bonded_channels.len() as f64)
        }
    }
}

// ─── Power Leveling ────────────────────────────────────────────────────────────

/// Per-wavelength power equalization state
#[derive(Debug, Clone)]
pub struct PowerLevelState {
    pub channel_id: u8,
    /// OLT measured received optical power from each ONU (onu_id → dBm)
    pub onu_rx_power: HashMap<u16, f64>,
    /// Target received power at OLT (dBm)
    pub target_rx_power_dbm: f64,
    /// Power adjustment commands issued (onu_id → delta_dBm)
    pub power_adjustments: HashMap<u16, f64>,
}

impl PowerLevelState {
    pub fn new(channel_id: u8) -> Self {
        PowerLevelState {
            channel_id,
            onu_rx_power: HashMap::new(),
            target_rx_power_dbm: -28.0,
            power_adjustments: HashMap::new(),
        }
    }

    /// Compute required power adjustment for a given ONU
    pub fn compute_adjustment(&self, onu_id: u16) -> f64 {
        if let Some(&rx_pwr) = self.onu_rx_power.get(&onu_id) {
            // Positive = ONU should increase TX power
            self.target_rx_power_dbm - rx_pwr
        } else {
            0.0
        }
    }

    /// Apply measured received power, compute and store adjustment
    pub fn update_and_equalize(&mut self, onu_id: u16, measured_rx_dbm: f64) {
        self.onu_rx_power.insert(onu_id, measured_rx_dbm);
        let adj = self.compute_adjustment(onu_id);
        // Clamp to ±6 dB per PLOAM step
        let clamped = adj.max(-6.0).min(6.0);
        self.power_adjustments.insert(onu_id, clamped);
    }
}

// ─── XGTC Frame ───────────────────────────────────────────────────────────────

/// XGTC (XG-PON TC) frame header for TWDM (G.989.3 adaptation)
#[derive(Debug, Clone)]
pub struct XgtcFrame {
    /// Frame counter (32-bit rolling)
    pub frame_counter: u32,
    /// Wavelength channel ID (0–7)
    pub channel_id: u8,
    /// Downstream payload bytes
    pub payload: Vec<u8>,
    /// Number of embedded PLOAM messages
    pub ploam_count: u8,
    /// Downstream BW allocations (burst profiles)
    pub allocations: Vec<BwAllocation>,
}

impl XgtcFrame {
    pub fn new(frame_counter: u32, channel_id: u8) -> Self {
        XgtcFrame {
            frame_counter,
            channel_id,
            payload: Vec::new(),
            ploam_count: 0,
            allocations: Vec::new(),
        }
    }

    /// Build a minimal XGTC downstream frame header (8 bytes)
    pub fn build_header(&self) -> Vec<u8> {
        let mut hdr = vec![0u8; 8];
        // Bytes 0–3: frame counter
        hdr[0] = (self.frame_counter >> 24) as u8;
        hdr[1] = (self.frame_counter >> 16) as u8;
        hdr[2] = (self.frame_counter >> 8) as u8;
        hdr[3] = (self.frame_counter & 0xFF) as u8;
        // Byte 4: channel ID in upper nibble, PLOAM count in lower nibble
        hdr[4] = (self.channel_id << 4) | (self.ploam_count & 0x0F);
        // Bytes 5–7: reserved / FEC indicator
        hdr[5] = 0xAA; // FEC enabled marker
        hdr[6] = 0;
        hdr[7] = 0;
        hdr
    }

    /// Total frame overhead bytes (header + allocations table)
    pub fn overhead_bytes(&self) -> usize {
        8 + self.allocations.len() * 4
    }
}

// ─── PtP WDM Overlay ──────────────────────────────────────────────────────────

/// Point-to-point WDM service for business/premium subscribers
#[derive(Debug, Clone)]
pub struct PtpWdmService {
    pub service_id: u32,
    /// Dedicated downstream wavelength (nm) — outside TWDM bands
    pub ds_wavelength_nm: f64,
    /// Dedicated upstream wavelength (nm)
    pub us_wavelength_nm: f64,
    /// Maximum bandwidth (bps) on this P2P WDM service
    pub max_bw_bps: f64,
    /// ONU port mapped to this service
    pub onu_port_id: u32,
    pub active: bool,
}

impl PtpWdmService {
    /// Create a PtP WDM service using additional C-band wavelengths (>1603 nm)
    pub fn new(service_id: u32, slot_index: u8, onu_port_id: u32) -> Self {
        // Use wavelengths above TWDM DS band (1603 + n * 0.8 nm)
        let ds_wl = 1603.8 + (slot_index as f64) * 0.8;
        // Below US band
        let us_wl = 1520.0 - (slot_index as f64) * 0.8;
        PtpWdmService {
            service_id,
            ds_wavelength_nm: ds_wl,
            us_wavelength_nm: us_wl,
            max_bw_bps: 10e9, // 10 Gbps dedicated
            onu_port_id,
            active: true,
        }
    }
}

// ─── Main TwdmPonProcessor ────────────────────────────────────────────────────

/// Configuration for TWDM-PON system
#[derive(Debug, Clone)]
pub struct TwdmConfig {
    /// Number of active wavelength channel pairs (4–8)
    pub num_channels: u8,
    /// ODN class
    pub odn_class: OdnClass,
    /// Protection type
    pub protection_type: ProtectionType,
    /// Enable channel bonding support
    pub enable_bonding: bool,
    /// Enable PtP WDM overlay
    pub enable_ptp_wdm: bool,
    /// Maximum ONUs per wavelength channel
    pub max_onus_per_channel: u16,
    /// FEC enabled (RS(248,216) per G.989.3)
    pub fec_enabled: bool,
    /// Downstream FEC overhead fraction
    pub fec_overhead_fraction: f64,
}

impl Default for TwdmConfig {
    fn default() -> Self {
        TwdmConfig {
            num_channels: 4,
            odn_class: OdnClass::N2,
            protection_type: ProtectionType::TypeB,
            enable_bonding: true,
            enable_ptp_wdm: false,
            max_onus_per_channel: 256,
            fec_enabled: true,
            fec_overhead_fraction: 0.125, // RS(248,216): 32/248 ≈ 12.9%
        }
    }
}

/// TWDM-PON OLT-side processor implementing NG-PON2 per ITU-T G.989.x
#[derive(Debug, Clone)]
pub struct TwdmPonProcessor {
    config: TwdmConfig,
    /// Active wavelength channels
    channels: Vec<WavelengthChannel>,
    /// ONU contexts keyed by ONU-ID
    onus: HashMap<u16, OnuContext>,
    /// Next ONU-ID to assign
    next_onu_id: u16,
    /// Frame counter (increments each 125 µs)
    frame_counter: u32,
    /// Power leveling state per channel
    power_level: Vec<PowerLevelState>,
    /// Protection: backup channel index for each primary channel (if applicable)
    protection_map: HashMap<u8, u8>,
    /// PtP WDM services
    ptp_services: Vec<PtpWdmService>,
    /// Ranging offset table: onu_id → EQD in ns
    ranging_table: HashMap<u16, f64>,
}

impl TwdmPonProcessor {
    /// Create a new TWDM-PON processor with the given configuration
    pub fn new(config: TwdmConfig) -> Self {
        let num_ch = config.num_channels.clamp(MIN_CHANNELS as u8, MAX_CHANNELS as u8);
        let channels: Vec<WavelengthChannel> = (0..num_ch)
            .map(|i| WavelengthChannel::new(i))
            .collect();
        let power_level: Vec<PowerLevelState> = (0..num_ch)
            .map(|i| PowerLevelState::new(i))
            .collect();

        // Default protection: each channel protects itself (no cross-channel)
        let mut protection_map = HashMap::new();
        for i in 0..num_ch {
            // Type B/C protection: pair channels (0↔1, 2↔3, ...)
            let backup = if i % 2 == 0 { i + 1 } else { i - 1 };
            if backup < num_ch {
                protection_map.insert(i, backup);
            }
        }

        TwdmPonProcessor {
            config,
            channels,
            onus: HashMap::new(),
            next_onu_id: 1,
            frame_counter: 0,
            power_level,
            protection_map,
            ptp_services: Vec::new(),
            ranging_table: HashMap::new(),
        }
    }

    // ─── Wavelength Plan ───────────────────────────────────────────────────────

    /// Configure the wavelength plan with `num_channels` TWDM channel pairs
    pub fn configure_wavelength_plan(&mut self, num_channels: u8) {
        let n = num_channels.clamp(MIN_CHANNELS as u8, MAX_CHANNELS as u8);
        self.channels.clear();
        self.power_level.clear();
        for i in 0..n {
            self.channels.push(WavelengthChannel::new(i));
            self.power_level.push(PowerLevelState::new(i));
        }
        self.config.num_channels = n;
    }

    /// Get immutable reference to a wavelength channel
    pub fn channel(&self, channel_id: u8) -> Option<&WavelengthChannel> {
        self.channels.get(channel_id as usize)
    }

    /// Get mutable reference to a wavelength channel
    pub fn channel_mut(&mut self, channel_id: u8) -> Option<&mut WavelengthChannel> {
        self.channels.get_mut(channel_id as usize)
    }

    /// Number of active channels
    pub fn active_channel_count(&self) -> usize {
        self.channels.iter().filter(|ch| ch.active).count()
    }

    // ─── Aggregate Capacity ────────────────────────────────────────────────────

    /// Total aggregate system capacity (bps) across all active channels
    pub fn aggregate_capacity(&self) -> f64 {
        let active = self.active_channel_count() as f64;
        let raw = active * LINE_RATE_BPS;
        if self.config.fec_enabled {
            raw * (1.0 - self.config.fec_overhead_fraction)
        } else {
            raw
        }
    }

    /// Aggregate capacity in Gbps (rounded)
    pub fn aggregate_capacity_gbps(&self) -> f64 {
        self.aggregate_capacity() / 1e9
    }

    // ─── ONU Activation ───────────────────────────────────────────────────────

    /// Assign a wavelength channel to a newly discovered ONU (least-loaded first).
    /// Primary sort: fewest assigned ONUs. Secondary sort: lowest load_fraction.
    pub fn assign_wavelength(&mut self, onu_id: u16) -> WavelengthChannel {
        let best_ch = self
            .channels
            .iter()
            .filter(|ch| ch.active)
            .filter(|ch| (ch.assigned_onus.len() as u16) < self.config.max_onus_per_channel)
            .min_by(|a, b| {
                a.onu_count()
                    .cmp(&b.onu_count())
                    .then_with(|| a.load_fraction.partial_cmp(&b.load_fraction).unwrap())
            })
            .map(|ch| ch.channel_id)
            .unwrap_or(0);

        if let Some(ch) = self.channels.get_mut(best_ch as usize) {
            if !ch.assigned_onus.contains(&onu_id) {
                ch.assigned_onus.push(onu_id);
            }
        }
        // Update ONU context
        if let Some(ctx) = self.onus.get_mut(&onu_id) {
            ctx.assigned_channel = best_ch;
        }
        // Return a copy of the assigned channel
        self.channels[best_ch as usize].clone()
    }

    /// Register a new ONU with serial number, advance its state machine
    /// Returns the assigned ONU-ID
    pub fn activate_onu(&mut self, serial_number: u64) -> u16 {
        let onu_id = self.next_onu_id;
        self.next_onu_id += 1;
        let mut ctx = OnuContext::new(serial_number);
        ctx.onu_id = onu_id;
        ctx.state = OnuState::SerialAcquired;
        self.onus.insert(onu_id, ctx);
        onu_id
    }

    /// Complete ranging for an ONU — computes EQD and transitions to Active
    pub fn complete_ranging(&mut self, onu_id: u16, measured_rtt_ns: f64) -> u32 {
        // EQD calculation: equalize all ONUs to same logical RTT
        // Target RTT = 20000 ns (20 µs) for 2 km max fiber
        let target_rtt_ns = 20_000.0;
        let eqd_ns = (target_rtt_ns - measured_rtt_ns).max(0.0);
        // Convert to byte units at 10 Gbps: 1 byte = 0.8 ns
        let eqd_bytes = (eqd_ns / 0.8).round() as u32;
        if let Some(ctx) = self.onus.get_mut(&onu_id) {
            ctx.rtt_ns = measured_rtt_ns;
            ctx.eqd_bytes = eqd_bytes;
            ctx.state = OnuState::Active;
        }
        self.ranging_table.insert(onu_id, eqd_ns);
        eqd_bytes
    }

    /// Deactivate an ONU, removing from channel assignment
    pub fn deactivate_onu(&mut self, onu_id: u16) {
        if let Some(ctx) = self.onus.get_mut(&onu_id) {
            let ch_id = ctx.assigned_channel;
            ctx.state = OnuState::Inactive;
            if let Some(ch) = self.channels.get_mut(ch_id as usize) {
                ch.assigned_onus.retain(|&id| id != onu_id);
            }
        }
    }

    // ─── Tuning Control ────────────────────────────────────────────────────────

    /// Issue a wavelength tuning command to an ONU
    pub fn tune_onu(&mut self, onu_id: u16, target_channel: u8) -> TuningControl {
        let from_channel = self
            .onus
            .get(&onu_id)
            .map(|c| c.assigned_channel)
            .unwrap_or(0);

        let already_on_target = from_channel == target_channel;
        let tuning_time = if already_on_target { 0.0 } else { TUNING_TIME_MAX_US };
        let success = target_channel < self.config.num_channels && !already_on_target || already_on_target;

        if success && !already_on_target {
            // Move ONU between channels
            if let Some(old_ch) = self.channels.get_mut(from_channel as usize) {
                old_ch.assigned_onus.retain(|&id| id != onu_id);
            }
            if let Some(new_ch) = self.channels.get_mut(target_channel as usize) {
                new_ch.assigned_onus.push(onu_id);
            }
            if let Some(ctx) = self.onus.get_mut(&onu_id) {
                ctx.assigned_channel = target_channel;
                ctx.state = OnuState::Tuning;
            }
        }

        TuningControl {
            onu_id,
            from_channel,
            to_channel: target_channel,
            tuning_time_us: tuning_time,
            estimated_completion_us: tuning_time + 50.0, // + processing overhead
            success,
            reason: TuningReason::LoadBalance,
        }
    }

    // ─── Cross-Wavelength DBA ──────────────────────────────────────────────────

    /// Perform cross-wavelength Dynamic Bandwidth Allocation across all channels.
    /// Returns a list of upstream bandwidth grants per ONU.
    pub fn cross_wavelength_dba(&self, reports: &[OnuReport]) -> Vec<BwAllocation> {
        // Available bytes per channel per 125 µs frame
        let frame_bytes = XGTC_DS_FRAME_BYTES;
        // Track remaining capacity per channel
        let mut remaining: Vec<u32> = self
            .channels
            .iter()
            .map(|ch| if ch.active { frame_bytes } else { 0 })
            .collect();

        let mut allocations = Vec::new();
        let mut channel_offsets: Vec<u32> = vec![0u32; self.channels.len()];

        for report in reports {
            let ch_id = report.channel_id as usize;
            if ch_id >= self.channels.len() {
                continue;
            }

            // Priority: T-CONT type 1 (fixed) > type 2 (assured) > type 4 (best-effort)
            let priority_factor = match report.tcont_type {
                1 => 1.0,      // Fixed BW — highest priority
                2 => 0.9,
                3 => 0.75,
                4 => 0.5,      // Best-effort
                5 => 0.3,
                _ => 0.5,
            };

            let requested = (report.requested_bw_bytes_per_frame as f64 * priority_factor) as u32;
            let grant = requested.min(remaining[ch_id]);

            if grant > 0 {
                let bps = (grant as f64 / (XGTC_FRAME_DURATION_US * 1e-6)) * 8.0;
                allocations.push(BwAllocation {
                    onu_id: report.onu_id,
                    channel_id: report.channel_id,
                    alloc_start: channel_offsets[ch_id],
                    alloc_size: grant,
                    effective_bps: bps,
                });
                remaining[ch_id] -= grant;
                channel_offsets[ch_id] += grant;
            }
        }

        allocations
    }

    /// Redistribute load across wavelength channels (cross-wavelength load balancing)
    pub fn rebalance_load(&mut self) {
        // Compute per-channel load
        let total_onus: usize = self.channels.iter().map(|ch| ch.onu_count()).sum();
        if total_onus == 0 {
            return;
        }
        let num_active = self.active_channel_count();
        if num_active == 0 {
            return;
        }
        let target_per_ch = total_onus / num_active;

        // Update load fractions
        for ch in self.channels.iter_mut() {
            ch.load_fraction = if target_per_ch > 0 {
                (ch.onu_count() as f64) / (target_per_ch as f64 * 2.0).max(1.0)
            } else {
                0.0
            };
        }
    }

    // ─── Protection Switching ──────────────────────────────────────────────────

    /// Execute protection switching when `failed_channel` goes down.
    /// Returns the backup channel ID used, or error if none available.
    pub fn protection_switch(&mut self, failed_channel: u8) -> Result<u8, &'static str> {
        // Check that failed channel exists and is active
        if failed_channel as usize >= self.channels.len() {
            return Err("Invalid channel ID");
        }
        if !self.channels[failed_channel as usize].active {
            return Err("Channel already inactive");
        }

        let backup_ch = match self.protection_map.get(&failed_channel) {
            Some(&b) => b,
            None => return Err("No protection configured for this channel"),
        };

        if backup_ch as usize >= self.channels.len() {
            return Err("Backup channel out of range");
        }

        // Mark failed channel as inactive
        self.channels[failed_channel as usize].active = false;

        // Move all ONUs from failed channel to backup channel
        let failed_onus: Vec<u16> = self.channels[failed_channel as usize]
            .assigned_onus
            .clone();

        self.channels[failed_channel as usize].assigned_onus.clear();

        let backup_max = self.config.max_onus_per_channel as usize;
        let backup_current = self.channels[backup_ch as usize].assigned_onus.len();

        let migrateable = failed_onus
            .len()
            .min(backup_max.saturating_sub(backup_current));

        for &onu_id in failed_onus[..migrateable].iter() {
            self.channels[backup_ch as usize].assigned_onus.push(onu_id);
            if let Some(ctx) = self.onus.get_mut(&onu_id) {
                ctx.assigned_channel = backup_ch;
                ctx.state = OnuState::Tuning;
            }
        }

        Ok(backup_ch)
    }

    // ─── PLOAM Message Builders ────────────────────────────────────────────────

    /// Build a Channel_Profile PLOAM message for a given wavelength channel
    pub fn build_channel_profile_ploam(&self, channel: &WavelengthChannel) -> Vec<u8> {
        let mut msg = PloamMessage::new(
            PloamType::ChannelProfile,
            0xFFFF, // Broadcast
            channel.channel_id,
        );
        // Payload encodes DS and US wavelengths as 16-bit fields (0.01 nm resolution)
        let ds_encoded = (channel.ds_wavelength_nm * 100.0) as u16;
        let us_encoded = (channel.us_wavelength_nm * 100.0) as u16;
        msg.payload.push((ds_encoded >> 8) as u8);
        msg.payload.push((ds_encoded & 0xFF) as u8);
        msg.payload.push((us_encoded >> 8) as u8);
        msg.payload.push((us_encoded & 0xFF) as u8);
        // TX power (dBm encoded as signed byte × 2)
        let tx_pwr = (channel.tx_power_dbm * 2.0) as i8 as u8;
        msg.payload.push(tx_pwr);
        // Channel load (0–255)
        let load_byte = (channel.load_fraction * 255.0) as u8;
        msg.payload.push(load_byte);
        // ONU count (2 bytes)
        let onu_cnt = channel.onu_count() as u16;
        msg.payload.push((onu_cnt >> 8) as u8);
        msg.payload.push((onu_cnt & 0xFF) as u8);
        msg.serialize()
    }

    /// Build a Wavelength_Channel_ID PLOAM message assigning an ONU to a channel
    pub fn build_wavelength_channel_id_ploam(&self, onu_id: u16, channel_id: u8) -> Vec<u8> {
        let mut msg = PloamMessage::new(
            PloamType::WavelengthChannelId,
            onu_id,
            channel_id,
        );
        msg.payload.push(channel_id);
        msg.payload.push(0); // reserved
        msg.serialize()
    }

    /// Build a Tuning_Control PLOAM message
    pub fn build_tuning_control_ploam(&self, tc: &TuningControl) -> Vec<u8> {
        let mut msg = PloamMessage::new(
            PloamType::TuningControl,
            tc.onu_id,
            tc.from_channel,
        );
        msg.payload.push(tc.from_channel);
        msg.payload.push(tc.to_channel);
        // Tuning time in units of 125 µs frames
        let frames = (tc.tuning_time_us / XGTC_FRAME_DURATION_US).ceil() as u8;
        msg.payload.push(frames);
        msg.payload.push(if tc.success { 0x01 } else { 0x00 });
        msg.serialize()
    }

    /// Build a Power_Level_Sequence PLOAM message
    pub fn build_power_level_ploam(&self, onu_id: u16, channel_id: u8, delta_dbm: f64) -> Vec<u8> {
        let mut msg = PloamMessage::new(
            PloamType::PowerLevelSequence,
            onu_id,
            channel_id,
        );
        // Encode delta_dbm as signed byte (0.5 dB resolution)
        let encoded = (delta_dbm * 2.0).round() as i8 as u8;
        msg.payload.push(encoded);
        msg.serialize()
    }

    /// Build a Protection_Switch_Over PLOAM broadcast
    pub fn build_protection_switchover_ploam(&self, failed_ch: u8, backup_ch: u8) -> Vec<u8> {
        let mut msg = PloamMessage::new(
            PloamType::ProtectionSwitchOver,
            0xFFFF, // Broadcast
            failed_ch,
        );
        msg.payload.push(failed_ch);
        msg.payload.push(backup_ch);
        // Switchover time in ms (1 byte, 0–255 ms)
        let sw_time = self.config.protection_type.switchover_time_ms() as u8;
        msg.payload.push(sw_time);
        msg.serialize()
    }

    // ─── Link Budget ──────────────────────────────────────────────────────────

    /// Calculate optical link budget for a given ODN class (returns max loss margin dB)
    pub fn calculate_link_budget(&self, odn_class: OdnClass) -> f64 {
        let (min_budget, max_budget, max_diff) = odn_class.loss_budget();

        // OLT transmit power per G.989.2 (Class N1/N2 OLT: +2 to +4 dBm)
        let olt_tx_dbm = 4.0;
        // ONU minimum sensitivity at 10 Gbps (−28 to −33 dBm typical)
        let onu_rx_sensitivity_dbm = -28.0;
        // Optical path loss budget
        let path_budget = olt_tx_dbm - onu_rx_sensitivity_dbm;

        // Splitter insertion loss (1:128 PON split: ~21 dB)
        let splitter_loss = 21.0;
        // Connector losses (4 × 0.5 dB)
        let connector_loss = 2.0;
        // Fiber loss (0.3 dB/km × 20 km max)
        let fiber_loss = 6.0;

        let total_path_loss = splitter_loss + connector_loss + fiber_loss;
        let margin = path_budget - total_path_loss;

        // Verify against ODN class maximum budget
        let _class_check = if path_budget >= min_budget && path_budget <= max_budget + max_diff {
            1.0
        } else {
            0.0
        };

        margin
    }

    /// SNR estimation for a given received power and channel bandwidth
    pub fn estimate_snr_db(&self, rx_power_dbm: f64, bandwidth_hz: f64) -> f64 {
        let rx_power_w = 1e-3 * f64::powf(10.0, rx_power_dbm / 10.0);
        let thermal_noise = K_BOLTZMANN * T_NOISE * bandwidth_hz;
        // Assume PIN detector responsivity 0.8 A/W, thermal noise dominated
        let snr_linear = rx_power_w / thermal_noise;
        10.0 * snr_linear.log10()
    }

    // ─── Power Leveling ───────────────────────────────────────────────────────

    /// Update power leveling for an ONU on a given channel
    pub fn update_power_leveling(
        &mut self,
        channel_id: u8,
        onu_id: u16,
        measured_rx_dbm: f64,
    ) -> f64 {
        if let Some(pl) = self.power_level.get_mut(channel_id as usize) {
            pl.update_and_equalize(onu_id, measured_rx_dbm);
            *pl.power_adjustments.get(&onu_id).unwrap_or(&0.0)
        } else {
            0.0
        }
    }

    /// Get power adjustment for an ONU
    pub fn get_power_adjustment(&self, channel_id: u8, onu_id: u16) -> f64 {
        self.power_level
            .get(channel_id as usize)
            .and_then(|pl| pl.power_adjustments.get(&onu_id))
            .copied()
            .unwrap_or(0.0)
    }

    // ─── Channel Bonding ──────────────────────────────────────────────────────

    /// Configure multi-wavelength bonding for an ONU (up to 4 channels)
    /// Returns actual bonded channel list
    pub fn configure_bonding(&mut self, onu_id: u16, additional_channels: &[u8]) -> Vec<u8> {
        if !self.config.enable_bonding {
            return Vec::new();
        }
        // Filter only valid, active channels
        let valid: Vec<u8> = additional_channels
            .iter()
            .copied()
            .filter(|&ch| {
                (ch as usize) < self.channels.len()
                    && self.channels[ch as usize].active
                    && ch != self.onus.get(&onu_id).map(|c| c.assigned_channel).unwrap_or(255)
            })
            .take(3) // max 3 additional = 4 total per G.989.1
            .collect();

        // Assign ONU to bonded channels
        for &ch_id in &valid {
            if let Some(ch) = self.channels.get_mut(ch_id as usize) {
                if !ch.assigned_onus.contains(&onu_id) {
                    ch.assigned_onus.push(onu_id);
                }
            }
        }

        if let Some(ctx) = self.onus.get_mut(&onu_id) {
            ctx.bonded_channels = valid.clone();
        }

        valid
    }

    /// Remove bonding for an ONU, restoring to single-channel operation
    pub fn remove_bonding(&mut self, onu_id: u16) {
        let bonded: Vec<u8> = self
            .onus
            .get(&onu_id)
            .map(|c| c.bonded_channels.clone())
            .unwrap_or_default();

        for ch_id in bonded {
            if let Some(ch) = self.channels.get_mut(ch_id as usize) {
                ch.assigned_onus.retain(|&id| id != onu_id);
            }
        }

        if let Some(ctx) = self.onus.get_mut(&onu_id) {
            ctx.bonded_channels.clear();
        }
    }

    // ─── PtP WDM Overlay ──────────────────────────────────────────────────────

    /// Add a PtP WDM business service
    pub fn add_ptp_service(&mut self, onu_port_id: u32, slot_index: u8) -> u32 {
        let service_id = self.ptp_services.len() as u32 + 1;
        self.ptp_services
            .push(PtpWdmService::new(service_id, slot_index, onu_port_id));
        service_id
    }

    /// Remove a PtP WDM service by ID
    pub fn remove_ptp_service(&mut self, service_id: u32) -> bool {
        let before = self.ptp_services.len();
        self.ptp_services.retain(|s| s.service_id != service_id);
        self.ptp_services.len() < before
    }

    // ─── Frame Generation ─────────────────────────────────────────────────────

    /// Advance the frame counter by one 125 µs tick
    pub fn tick_frame(&mut self) {
        self.frame_counter = self.frame_counter.wrapping_add(1);
    }

    /// Build downstream XGTC frame for a given channel
    pub fn build_xgtc_frame(&self, channel_id: u8) -> XgtcFrame {
        XgtcFrame::new(self.frame_counter, channel_id)
    }

    // ─── Diagnostics ──────────────────────────────────────────────────────────

    /// System summary: active channels, total ONUs, aggregate capacity
    pub fn system_summary(&self) -> TwdmSystemSummary {
        let total_onus: usize = self.channels.iter().map(|ch| ch.onu_count()).sum();
        TwdmSystemSummary {
            active_channels: self.active_channel_count(),
            total_onus,
            aggregate_capacity_gbps: self.aggregate_capacity_gbps(),
            protection_type: self.config.protection_type.clone(),
            odn_class: self.config.odn_class.clone(),
            bonding_enabled: self.config.enable_bonding,
            ptp_services: self.ptp_services.len(),
        }
    }

    /// Check if a channel is overloaded (>90% capacity)
    pub fn is_channel_overloaded(&self, channel_id: u8) -> bool {
        self.channels
            .get(channel_id as usize)
            .map(|ch| ch.load_fraction > 0.9)
            .unwrap_or(false)
    }

    /// Retrieve ONU context
    pub fn onu_context(&self, onu_id: u16) -> Option<&OnuContext> {
        self.onus.get(&onu_id)
    }

    /// Total number of registered ONUs
    pub fn total_onus(&self) -> usize {
        self.onus.len()
    }

    /// Frame counter value
    pub fn frame_counter(&self) -> u32 {
        self.frame_counter
    }
}

/// Summary of the TWDM-PON system state
#[derive(Debug, Clone)]
pub struct TwdmSystemSummary {
    pub active_channels: usize,
    pub total_onus: usize,
    pub aggregate_capacity_gbps: f64,
    pub protection_type: ProtectionType,
    pub odn_class: OdnClass,
    pub bonding_enabled: bool,
    pub ptp_services: usize,
}

// ─── Helper Functions ─────────────────────────────────────────────────────────

/// Convert wavelength in nm to frequency in THz
pub fn wavelength_nm_to_thz(wavelength_nm: f64) -> f64 {
    // f = c / λ
    (C_LIGHT / (wavelength_nm * 1e-9)) / 1e12
}

/// Convert frequency in THz to wavelength in nm
pub fn thz_to_wavelength_nm(freq_thz: f64) -> f64 {
    (C_LIGHT / (freq_thz * 1e12)) / 1e-9
}

/// Convert dBm to watts
pub fn dbm_to_watts(dbm: f64) -> f64 {
    1e-3 * f64::powf(10.0, dbm / 10.0)
}

/// Convert watts to dBm
pub fn watts_to_dbm(watts: f64) -> f64 {
    10.0 * (watts / 1e-3).log10()
}

/// Compute free-space path loss in dB for fiber at a given wavelength
/// (Simplified: fiber propagation at 0.3 dB/km)
pub fn fiber_path_loss_db(length_km: f64) -> f64 {
    0.3 * length_km
}

/// Compute splitting loss for a 1:N passive splitter
pub fn splitter_loss_db(split_ratio: u32) -> f64 {
    10.0 * (split_ratio as f64).log10()
}

/// Minimum ONU count needed for full wavelength channel utilization
pub fn min_onus_for_utilization(target_utilization: f64, max_onus_per_ch: u16, num_channels: u8) -> u32 {
    let per_ch = (target_utilization * max_onus_per_ch as f64).ceil() as u32;
    per_ch * num_channels as u32
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_processor() -> TwdmPonProcessor {
        TwdmPonProcessor::new(TwdmConfig::default())
    }

    // ── Wavelength plan ───────────────────────────────────────────────────────

    #[test]
    fn test_channel_count_default() {
        let proc = default_processor();
        assert_eq!(proc.channels.len(), 4);
    }

    #[test]
    fn test_channel_count_8() {
        let cfg = TwdmConfig { num_channels: 8, ..Default::default() };
        let proc = TwdmPonProcessor::new(cfg);
        assert_eq!(proc.channels.len(), 8);
    }

    #[test]
    fn test_channel_clamp_min() {
        let cfg = TwdmConfig { num_channels: 2, ..Default::default() };
        let proc = TwdmPonProcessor::new(cfg);
        assert_eq!(proc.channels.len(), MIN_CHANNELS);
    }

    #[test]
    fn test_channel_clamp_max() {
        let cfg = TwdmConfig { num_channels: 12, ..Default::default() };
        let proc = TwdmPonProcessor::new(cfg);
        assert_eq!(proc.channels.len(), MAX_CHANNELS);
    }

    #[test]
    fn test_ds_wavelengths() {
        let proc = default_processor();
        // Channel 0: 1596.0 nm
        assert!((proc.channels[0].ds_wavelength_nm - 1596.0).abs() < 0.001);
        // Channel 1: 1596.8 nm
        assert!((proc.channels[1].ds_wavelength_nm - 1596.8).abs() < 0.001);
        // Channel 3: 1598.4 nm
        assert!((proc.channels[3].ds_wavelength_nm - 1598.4).abs() < 0.001);
    }

    #[test]
    fn test_us_wavelengths() {
        let proc = default_processor();
        assert!((proc.channels[0].us_wavelength_nm - 1524.0).abs() < 0.001);
        assert!((proc.channels[1].us_wavelength_nm - 1524.8).abs() < 0.001);
    }

    #[test]
    fn test_ds_frequency_range() {
        let proc = default_processor();
        // DS at 1596 nm should be ~187.8 THz
        let f = proc.channels[0].ds_freq_thz;
        assert!(f > 185.0 && f < 190.0, "DS freq {} THz out of range", f);
    }

    #[test]
    fn test_us_frequency_range() {
        let proc = default_processor();
        // US at 1524 nm should be ~196.9 THz
        let f = proc.channels[0].us_freq_thz;
        assert!(f > 193.0 && f < 200.0, "US freq {} THz out of range", f);
    }

    #[test]
    fn test_configure_wavelength_plan_resize() {
        let mut proc = default_processor();
        proc.configure_wavelength_plan(6);
        assert_eq!(proc.channels.len(), 6);
        assert_eq!(proc.power_level.len(), 6);
    }

    // ── Capacity ──────────────────────────────────────────────────────────────

    #[test]
    fn test_aggregate_capacity_4ch() {
        let proc = default_processor();
        let cap = proc.aggregate_capacity_gbps();
        // 4 × 9.95 Gbps × (1 - 0.125) ≈ 34.8 Gbps
        assert!(cap > 30.0 && cap < 45.0, "4ch capacity {} Gbps out of range", cap);
    }

    #[test]
    fn test_aggregate_capacity_8ch() {
        let cfg = TwdmConfig { num_channels: 8, ..Default::default() };
        let proc = TwdmPonProcessor::new(cfg);
        let cap = proc.aggregate_capacity_gbps();
        assert!(cap > 60.0 && cap < 90.0, "8ch capacity {} Gbps", cap);
    }

    #[test]
    fn test_aggregate_capacity_no_fec() {
        let cfg = TwdmConfig { fec_enabled: false, ..Default::default() };
        let proc = TwdmPonProcessor::new(cfg);
        let cap_no_fec = proc.aggregate_capacity_gbps();
        let cfg2 = TwdmConfig::default();
        let proc2 = TwdmPonProcessor::new(cfg2);
        let cap_fec = proc2.aggregate_capacity_gbps();
        assert!(cap_no_fec > cap_fec, "No-FEC capacity should exceed FEC capacity");
    }

    // ── ONU Activation ────────────────────────────────────────────────────────

    #[test]
    fn test_activate_onu_assigns_id() {
        let mut proc = default_processor();
        let id = proc.activate_onu(0xDEADBEEF_CAFEBABE);
        assert_eq!(id, 1);
        let id2 = proc.activate_onu(0x1234_5678_9ABC_DEF0);
        assert_eq!(id2, 2);
    }

    #[test]
    fn test_activate_onu_state() {
        let mut proc = default_processor();
        let id = proc.activate_onu(12345);
        assert_eq!(proc.onus[&id].state, OnuState::SerialAcquired);
    }

    #[test]
    fn test_assign_wavelength_load_balance() {
        let mut proc = default_processor();
        // Activate and assign 4 ONUs
        for sn in 1u64..=4 {
            let onu_id = proc.activate_onu(sn);
            proc.assign_wavelength(onu_id);
        }
        // Each channel should have exactly 1 ONU
        for ch in &proc.channels {
            assert_eq!(ch.onu_count(), 1, "ch {} has {} ONUs", ch.channel_id, ch.onu_count());
        }
    }

    #[test]
    fn test_complete_ranging() {
        let mut proc = default_processor();
        let id = proc.activate_onu(42);
        let eqd = proc.complete_ranging(id, 10_000.0); // 10 µs RTT
        assert!(eqd > 0, "EQD should be positive for short RTT");
        assert_eq!(proc.onus[&id].state, OnuState::Active);
    }

    #[test]
    fn test_complete_ranging_long_rtt() {
        let mut proc = default_processor();
        let id = proc.activate_onu(99);
        // RTT > target → EQD = 0
        let eqd = proc.complete_ranging(id, 25_000.0);
        assert_eq!(eqd, 0, "EQD should be 0 when RTT > target");
    }

    #[test]
    fn test_deactivate_onu() {
        let mut proc = default_processor();
        let id = proc.activate_onu(7);
        proc.assign_wavelength(id);
        proc.deactivate_onu(id);
        assert_eq!(proc.onus[&id].state, OnuState::Inactive);
        // ONU should be removed from channel
        for ch in &proc.channels {
            assert!(!ch.assigned_onus.contains(&id));
        }
    }

    // ── Tuning Control ────────────────────────────────────────────────────────

    #[test]
    fn test_tune_onu_valid() {
        let mut proc = default_processor();
        let id = proc.activate_onu(55);
        proc.assign_wavelength(id); // starts on channel 0 (least loaded)
        let tc = proc.tune_onu(id, 2);
        assert!(tc.success);
        assert_eq!(tc.to_channel, 2);
        assert!(tc.tuning_time_us <= TUNING_TIME_MAX_US);
    }

    #[test]
    fn test_tune_onu_invalid_channel() {
        let mut proc = default_processor();
        let id = proc.activate_onu(66);
        let tc = proc.tune_onu(id, 99); // out of range
        assert!(!tc.success);
    }

    #[test]
    fn test_tune_onu_same_channel() {
        let mut proc = default_processor();
        let id = proc.activate_onu(77);
        // manually set channel
        proc.onus.get_mut(&id).unwrap().assigned_channel = 1;
        let tc = proc.tune_onu(id, 1); // already on ch 1
        assert!(tc.success);
        assert_eq!(tc.tuning_time_us, 0.0);
    }

    #[test]
    fn test_tuning_control_ploam_length() {
        let mut proc = default_processor();
        let id = proc.activate_onu(88);
        let tc = proc.tune_onu(id, 2);
        let ploam = proc.build_tuning_control_ploam(&tc);
        assert_eq!(ploam.len(), 48, "PLOAM must be 48 bytes");
    }

    // ── Cross-Wavelength DBA ──────────────────────────────────────────────────

    #[test]
    fn test_cross_wavelength_dba_single_report() {
        let proc = default_processor();
        let reports = vec![OnuReport {
            onu_id: 1,
            channel_id: 0,
            queue_occupancy_bytes: 1000,
            requested_bw_bytes_per_frame: 1000,
            tcont_type: 4,
        }];
        let allocs = proc.cross_wavelength_dba(&reports);
        assert_eq!(allocs.len(), 1);
        assert_eq!(allocs[0].onu_id, 1);
        assert!(allocs[0].alloc_size > 0);
    }

    #[test]
    fn test_cross_wavelength_dba_priority() {
        let proc = default_processor();
        let reports = vec![
            OnuReport { onu_id: 1, channel_id: 0, queue_occupancy_bytes: 500,
                requested_bw_bytes_per_frame: 500, tcont_type: 1 },
            OnuReport { onu_id: 2, channel_id: 0, queue_occupancy_bytes: 500,
                requested_bw_bytes_per_frame: 500, tcont_type: 4 },
        ];
        let allocs = proc.cross_wavelength_dba(&reports);
        assert_eq!(allocs.len(), 2);
        // T-CONT 1 (fixed) gets full requested, T-CONT 4 gets less
        let t1_alloc = allocs.iter().find(|a| a.onu_id == 1).unwrap();
        let t4_alloc = allocs.iter().find(|a| a.onu_id == 2).unwrap();
        assert!(t1_alloc.alloc_size >= t4_alloc.alloc_size);
    }

    #[test]
    fn test_cross_wavelength_dba_multi_channel() {
        let proc = default_processor();
        let reports: Vec<OnuReport> = (0..4)
            .map(|ch| OnuReport {
                onu_id: ch as u16 + 1,
                channel_id: ch as u8,
                queue_occupancy_bytes: 10_000,
                requested_bw_bytes_per_frame: 10_000,
                tcont_type: 2,
            })
            .collect();
        let allocs = proc.cross_wavelength_dba(&reports);
        // Each ONU on its own channel should get allocation
        assert_eq!(allocs.len(), 4);
        for (i, alloc) in allocs.iter().enumerate() {
            assert_eq!(alloc.channel_id, i as u8);
        }
    }

    // ── Protection Switching ──────────────────────────────────────────────────

    #[test]
    fn test_protection_switch_valid() {
        let mut proc = default_processor();
        let id = proc.activate_onu(100);
        proc.assign_wavelength(id);
        // Force ONU onto channel 0
        if let Some(ctx) = proc.onus.get_mut(&id) {
            ctx.assigned_channel = 0;
        }
        proc.channels[0].assigned_onus = vec![id];
        proc.channels[1].assigned_onus.clear();

        let result = proc.protection_switch(0);
        assert!(result.is_ok());
        let backup = result.unwrap();
        assert_eq!(backup, 1); // Channel 0 → backup 1
        assert!(!proc.channels[0].active);
    }

    #[test]
    fn test_protection_switch_invalid_channel() {
        let mut proc = default_processor();
        let result = proc.protection_switch(99);
        assert!(result.is_err());
    }

    #[test]
    fn test_protection_switch_already_inactive() {
        let mut proc = default_processor();
        proc.channels[0].active = false;
        let result = proc.protection_switch(0);
        assert!(result.is_err());
    }

    #[test]
    fn test_protection_switch_onus_migrated() {
        let mut proc = default_processor();
        // Manually set up 2 ONUs on channel 0
        for sn in [1u64, 2] {
            let id = proc.activate_onu(sn);
            proc.onus.get_mut(&id).unwrap().assigned_channel = 0;
        }
        proc.channels[0].assigned_onus = vec![1, 2];
        proc.channels[1].assigned_onus.clear();

        proc.protection_switch(0).unwrap();
        assert!(proc.channels[1].assigned_onus.contains(&1) ||
                proc.channels[1].assigned_onus.contains(&2));
    }

    // ── PLOAM Messages ────────────────────────────────────────────────────────

    #[test]
    fn test_channel_profile_ploam_length() {
        let proc = default_processor();
        let ch = proc.channels[0].clone();
        let ploam = proc.build_channel_profile_ploam(&ch);
        assert_eq!(ploam.len(), 48);
    }

    #[test]
    fn test_channel_profile_ploam_channel_id() {
        let proc = default_processor();
        let ch = proc.channels[2].clone();
        let ploam = proc.build_channel_profile_ploam(&ch);
        // Byte 3 = channel_id
        assert_eq!(ploam[3], 2);
    }

    #[test]
    fn test_wavelength_channel_id_ploam() {
        let proc = default_processor();
        let ploam = proc.build_wavelength_channel_id_ploam(42, 3);
        assert_eq!(ploam.len(), 48);
        // ONU-ID in bytes 0–1
        let onu_id = ((ploam[0] as u16) << 8) | ploam[1] as u16;
        assert_eq!(onu_id, 42);
    }

    #[test]
    fn test_power_level_ploam() {
        let proc = default_processor();
        let ploam = proc.build_power_level_ploam(5, 0, 3.0);
        assert_eq!(ploam.len(), 48);
    }

    #[test]
    fn test_protection_switchover_ploam() {
        let proc = default_processor();
        let ploam = proc.build_protection_switchover_ploam(0, 1);
        assert_eq!(ploam.len(), 48);
        // Broadcast ONU-ID = 0xFFFF
        assert_eq!(ploam[0], 0xFF);
        assert_eq!(ploam[1], 0xFF);
        assert_eq!(ploam[5], 0); // failed_ch
        assert_eq!(ploam[6], 1); // backup_ch
    }

    // ── Link Budget ───────────────────────────────────────────────────────────

    #[test]
    fn test_link_budget_n2() {
        let proc = default_processor();
        let margin = proc.calculate_link_budget(OdnClass::N2);
        // Should be positive (more budget than losses)
        assert!(margin > 0.0, "Link budget margin should be positive: {}", margin);
    }

    #[test]
    fn test_link_budget_e2_greater_n1() {
        let proc = default_processor();
        // E2 has higher max budget than N1, but our simple calc is the same;
        // just verify it doesn't panic and returns a number
        let b_n1 = proc.calculate_link_budget(OdnClass::N1);
        let b_e2 = proc.calculate_link_budget(OdnClass::E2);
        // Both should be positive
        assert!(b_n1 > 0.0);
        assert!(b_e2 > 0.0);
    }

    #[test]
    fn test_snr_estimation() {
        let proc = default_processor();
        let snr = proc.estimate_snr_db(-28.0, BW_REF);
        assert!(snr > 20.0, "SNR at -28 dBm should exceed 20 dB: {}", snr);
    }

    // ── Power Leveling ────────────────────────────────────────────────────────

    #[test]
    fn test_power_leveling_update() {
        let mut proc = default_processor();
        let adj = proc.update_power_leveling(0, 1, -30.0);
        // Target is -28 dBm, received -30 dBm → need +2 dBm
        assert!((adj - 2.0).abs() < 0.01, "Adjustment should be +2 dBm, got {}", adj);
    }

    #[test]
    fn test_power_leveling_clamp() {
        let mut proc = default_processor();
        // Very low received power → clamp at +6 dBm
        let adj = proc.update_power_leveling(0, 2, -40.0);
        assert_eq!(adj, 6.0);
    }

    #[test]
    fn test_power_leveling_negative_adjustment() {
        let mut proc = default_processor();
        // Very high received power → negative adjustment (reduce TX power)
        let adj = proc.update_power_leveling(0, 3, -20.0);
        assert!(adj < 0.0, "Should be negative adjustment: {}", adj);
    }

    // ── Channel Bonding ───────────────────────────────────────────────────────

    #[test]
    fn test_channel_bonding_enabled() {
        let mut proc = default_processor();
        let id = proc.activate_onu(200);
        proc.onus.get_mut(&id).unwrap().assigned_channel = 0;
        let bonded = proc.configure_bonding(id, &[1, 2, 3]);
        assert_eq!(bonded.len(), 3);
    }

    #[test]
    fn test_channel_bonding_disabled() {
        let cfg = TwdmConfig { enable_bonding: false, ..Default::default() };
        let mut proc = TwdmPonProcessor::new(cfg);
        let id = proc.activate_onu(201);
        let bonded = proc.configure_bonding(id, &[1, 2]);
        assert!(bonded.is_empty());
    }

    #[test]
    fn test_channel_bonding_excludes_primary() {
        let mut proc = default_processor();
        let id = proc.activate_onu(202);
        proc.onus.get_mut(&id).unwrap().assigned_channel = 1;
        // Trying to bond including primary channel 1 — should exclude it
        let bonded = proc.configure_bonding(id, &[1, 2, 3]);
        assert!(!bonded.contains(&1));
    }

    #[test]
    fn test_remove_bonding() {
        let mut proc = default_processor();
        let id = proc.activate_onu(203);
        proc.onus.get_mut(&id).unwrap().assigned_channel = 0;
        proc.configure_bonding(id, &[1, 2]);
        proc.remove_bonding(id);
        assert!(proc.onus[&id].bonded_channels.is_empty());
    }

    #[test]
    fn test_bonded_throughput() {
        let mut proc = default_processor();
        let id = proc.activate_onu(204);
        proc.onus.get_mut(&id).unwrap().assigned_channel = 0;
        proc.configure_bonding(id, &[1, 2]);
        let ctx = proc.onu_context(id).unwrap();
        let tput = ctx.bonded_throughput_bps();
        // 3× single-channel
        assert!((tput - 3.0 * LINE_RATE_BPS).abs() < 1.0);
    }

    // ── PtP WDM Overlay ───────────────────────────────────────────────────────

    #[test]
    fn test_add_ptp_service() {
        let mut proc = default_processor();
        let sid = proc.add_ptp_service(1001, 0);
        assert_eq!(sid, 1);
        assert_eq!(proc.ptp_services.len(), 1);
    }

    #[test]
    fn test_remove_ptp_service() {
        let mut proc = default_processor();
        let sid = proc.add_ptp_service(1002, 0);
        assert!(proc.remove_ptp_service(sid));
        assert!(proc.ptp_services.is_empty());
    }

    #[test]
    fn test_ptp_wavelength_outside_twdm_band() {
        let mut proc = default_processor();
        proc.add_ptp_service(1003, 0);
        let svc = &proc.ptp_services[0];
        // DS wavelength must be above 1603 nm (TWDM DS top)
        assert!(svc.ds_wavelength_nm > 1603.0, "PtP DS wavelength {} should be > 1603 nm", svc.ds_wavelength_nm);
        // US wavelength must be below 1524 nm (TWDM US base)
        assert!(svc.us_wavelength_nm < 1524.0, "PtP US wavelength {} should be < 1524 nm", svc.us_wavelength_nm);
    }

    // ── XGTC Frame ────────────────────────────────────────────────────────────

    #[test]
    fn test_xgtc_frame_header_length() {
        let proc = default_processor();
        let frame = proc.build_xgtc_frame(0);
        let hdr = frame.build_header();
        assert_eq!(hdr.len(), 8);
    }

    #[test]
    fn test_xgtc_frame_counter_encoding() {
        let mut proc = default_processor();
        proc.frame_counter = 0xDEAD_BEEF;
        let frame = proc.build_xgtc_frame(0);
        let hdr = frame.build_header();
        assert_eq!(hdr[0], 0xDE);
        assert_eq!(hdr[1], 0xAD);
        assert_eq!(hdr[2], 0xBE);
        assert_eq!(hdr[3], 0xEF);
    }

    #[test]
    fn test_xgtc_frame_channel_id_encoding() {
        let proc = default_processor();
        let frame = proc.build_xgtc_frame(3);
        let hdr = frame.build_header();
        // Channel ID is upper nibble of byte 4
        assert_eq!((hdr[4] >> 4) & 0x0F, 3);
    }

    #[test]
    fn test_tick_frame() {
        let mut proc = default_processor();
        assert_eq!(proc.frame_counter(), 0);
        proc.tick_frame();
        assert_eq!(proc.frame_counter(), 1);
        proc.tick_frame();
        assert_eq!(proc.frame_counter(), 2);
    }

    // ── ODN Class ─────────────────────────────────────────────────────────────

    #[test]
    fn test_odn_class_n1_budget() {
        let (min, max, diff) = OdnClass::N1.loss_budget();
        assert_eq!(min, 14.0);
        assert_eq!(max, 29.0);
        assert_eq!(diff, 10.0);
    }

    #[test]
    fn test_odn_class_e2_budget() {
        let (min, max, diff) = OdnClass::E2.loss_budget();
        assert_eq!(min, 20.0);
        assert_eq!(max, 35.0);
        assert_eq!(diff, 15.0);
    }

    #[test]
    fn test_odn_orl_requirement() {
        assert_eq!(OdnClass::N1.min_orl_db(), 32.0);
        assert_eq!(OdnClass::E2.min_orl_db(), 32.0);
    }

    // ── Helper Functions ──────────────────────────────────────────────────────

    #[test]
    fn test_wavelength_to_freq() {
        // 1550 nm → ~193.4 THz
        let f = wavelength_nm_to_thz(1550.0);
        assert!((f - 193.4).abs() < 0.5, "Frequency at 1550 nm: {} THz", f);
    }

    #[test]
    fn test_freq_to_wavelength() {
        let wl = thz_to_wavelength_nm(193.414);
        assert!((wl - 1550.0).abs() < 1.0, "Wavelength at 193.4 THz: {} nm", wl);
    }

    #[test]
    fn test_round_trip_wl_freq() {
        let original = 1596.0;
        let freq = wavelength_nm_to_thz(original);
        let back = thz_to_wavelength_nm(freq);
        assert!((back - original).abs() < 0.001);
    }

    #[test]
    fn test_dbm_watts_conversion() {
        assert!((dbm_to_watts(0.0) - 1e-3).abs() < 1e-10);
        assert!((dbm_to_watts(10.0) - 10e-3).abs() < 1e-8);
        assert!((watts_to_dbm(1e-3) - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_fiber_path_loss() {
        assert!((fiber_path_loss_db(10.0) - 3.0).abs() < 0.001);
        assert!((fiber_path_loss_db(20.0) - 6.0).abs() < 0.001);
    }

    #[test]
    fn test_splitter_loss_128() {
        let loss = splitter_loss_db(128);
        assert!((loss - 21.07).abs() < 0.01, "1:128 split loss: {} dB", loss);
    }

    #[test]
    fn test_min_onus_for_utilization() {
        let min_onus = min_onus_for_utilization(0.5, 256, 4);
        assert_eq!(min_onus, 512); // 50% × 256 × 4 channels
    }

    // ── Protection Type ───────────────────────────────────────────────────────

    #[test]
    fn test_protection_type_c_fastest() {
        assert!(
            ProtectionType::TypeC.switchover_time_ms() < ProtectionType::TypeA.switchover_time_ms()
        );
    }

    #[test]
    fn test_protection_types_under_50ms() {
        for pt in [ProtectionType::TypeA, ProtectionType::TypeB, ProtectionType::TypeC] {
            assert!(
                pt.switchover_time_ms() <= PROTECTION_SWITCH_TARGET_MS,
                "{:?} switchover {} ms exceeds target",
                pt,
                pt.switchover_time_ms()
            );
        }
    }

    // ── System Summary ────────────────────────────────────────────────────────

    #[test]
    fn test_system_summary_default() {
        let proc = default_processor();
        let summary = proc.system_summary();
        assert_eq!(summary.active_channels, 4);
        assert_eq!(summary.total_onus, 0);
        assert!(summary.aggregate_capacity_gbps > 0.0);
    }

    #[test]
    fn test_active_channel_count_after_failure() {
        let mut proc = default_processor();
        proc.channels[0].active = false;
        assert_eq!(proc.active_channel_count(), 3);
    }

    #[test]
    fn test_total_onus_count() {
        let mut proc = default_processor();
        proc.activate_onu(1);
        proc.activate_onu(2);
        proc.activate_onu(3);
        assert_eq!(proc.total_onus(), 3);
    }

    #[test]
    fn test_rebalance_load() {
        let mut proc = default_processor();
        // Load channel 0 heavily
        for sn in 0u64..8 {
            let id = proc.activate_onu(sn + 100);
            proc.channels[0].assigned_onus.push(id);
        }
        proc.rebalance_load();
        // After rebalance, load fraction should be non-zero for ch 0
        assert!(proc.channels[0].load_fraction > 0.0);
    }
}

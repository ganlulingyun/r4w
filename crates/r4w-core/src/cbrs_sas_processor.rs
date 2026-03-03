//! CBRS (Citizens Broadband Radio Service) SAS (Spectrum Access System) Processor
//!
//! Implements FCC Part 96 three-tier spectrum sharing framework for the
//! 3550-3700 MHz CBRS band. Includes CBSD registration, SAS grant management,
//! DPA protection, ESC incumbent detection, channel assignment, and power control.
//!
//! # Band Plan
//! - Total: 3550–3700 MHz (150 MHz)
//! - PAL: 3550–3650 MHz (10 channels × 10 MHz)
//! - GAA: 3550–3700 MHz (shared with PAL + 5 additional channels 3650–3700 MHz)
//! - Incumbent: US Navy radar 3550–3650 MHz
//!
//! # Three-Tier Architecture
//! 1. **Incumbent Access (IA)**: US Navy radar – highest priority, must be protected
//! 2. **Priority Access License (PAL)**: licensed per census tract, up to 7 channels
//! 3. **General Authorized Access (GAA)**: opportunistic, lowest priority
//!
//! # References
//! - FCC Part 96 (47 CFR Part 96)
//! - WINNF-TS-0016 SAS-CBSD interface specification
//! - WINNF-TS-0112 ESC Sensor Network specification

use std::collections::HashMap;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Band start frequency in MHz
pub const BAND_START_MHZ: f64 = 3550.0;
/// Band end frequency in MHz
pub const BAND_END_MHZ: f64 = 3700.0;
/// Channel bandwidth in MHz
pub const CHANNEL_BW_MHZ: f64 = 10.0;
/// Total number of 10 MHz channels in the band
pub const NUM_CHANNELS: usize = 15;
/// Number of PAL channels (3550–3650 MHz)
pub const NUM_PAL_CHANNELS: usize = 10;
/// Number of GAA-only channels (3650–3700 MHz)
pub const NUM_GAA_ONLY_CHANNELS: usize = 5;
/// Category A maximum EIRP (dBm)
pub const CAT_A_MAX_EIRP_DBM: f64 = 30.0;
/// Category B maximum EIRP (dBm)
pub const CAT_B_MAX_EIRP_DBM: f64 = 47.0;
/// Category A maximum conducted power (dBm)
pub const CAT_A_MAX_CONDUCTED_DBM: f64 = 24.0;
/// Category B maximum conducted power (dBm)
pub const CAT_B_MAX_CONDUCTED_DBM: f64 = 30.0;
/// Maximum antenna gain for Category A (dBi)
pub const CAT_A_MAX_ANTENNA_GAIN_DBI: f64 = 6.0;
/// Maximum PAL channels per licensee per census tract
pub const MAX_PAL_CHANNELS_PER_LICENSEE: usize = 7;
/// Heartbeat interval in seconds (minimum)
pub const HEARTBEAT_MIN_SECS: u64 = 60;
/// Heartbeat interval in seconds (maximum)
pub const HEARTBEAT_MAX_SECS: u64 = 300;
/// Default grant duration in seconds
pub const DEFAULT_GRANT_DURATION_SECS: u64 = 300;
/// DPA aggregate interference limit (dBm/10 MHz) for Navy radar
pub const DPA_INTERFERENCE_LIMIT_DBM: f64 = -144.0;
/// ESC detection threshold (dBm/10 MHz)
pub const ESC_DETECTION_THRESHOLD_DBM: f64 = -109.0;
/// Speed of light (m/s)
const SPEED_OF_LIGHT: f64 = 2.998e8;
/// Reference distance for path loss (m)
const D0_M: f64 = 1.0;
/// Earth radius (m)
const EARTH_RADIUS_M: f64 = 6_371_000.0;

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// CBRS three-tier access classification
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AccessTier {
    /// US Navy radar and other incumbents – highest priority
    IncumbentAccess,
    /// Priority Access License – licensed per census tract
    PriorityAccess,
    /// General Authorized Access – opportunistic lowest priority
    GeneralAuthorizedAccess,
}

/// CBSD hardware category per FCC Part 96
#[derive(Debug, Clone, PartialEq, Eq, Copy)]
pub enum CbsdCategory {
    /// Category A: indoor/low-power, max 30 dBm EIRP
    CategoryA,
    /// Category B: outdoor/higher-power, max 47 dBm EIRP
    CategoryB,
}

impl CbsdCategory {
    /// Maximum permitted EIRP in dBm
    pub fn max_eirp_dbm(&self) -> f64 {
        match self {
            CbsdCategory::CategoryA => CAT_A_MAX_EIRP_DBM,
            CbsdCategory::CategoryB => CAT_B_MAX_EIRP_DBM,
        }
    }

    /// Maximum permitted conducted power in dBm
    pub fn max_conducted_dbm(&self) -> f64 {
        match self {
            CbsdCategory::CategoryA => CAT_A_MAX_CONDUCTED_DBM,
            CbsdCategory::CategoryB => CAT_B_MAX_CONDUCTED_DBM,
        }
    }
}

/// Grant state machine states (WINNF-TS-0016 §6.3)
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GrantState {
    /// No grant – CBSD is registered but idle
    Idle,
    /// Grant has been issued but not yet authorized for transmission
    Granted,
    /// CBSD is authorized to transmit
    Authorized,
    /// Grant is suspended (e.g., incumbent detected)
    Suspended,
    /// Grant has been terminated
    Terminated,
}

/// SAS response codes
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SasResponseCode {
    Success,
    VersionMismatch,
    Blacklisted,
    MissingParam,
    InvalidValue,
    UnsupportedSpec,
    RateLimitExceeded,
    InterferenceConstraint,
    GrantConflict,
    Terminated,
    Suspended,
    Unsync,
    RegPending,
    RegGrantConflict,
    CbsdError,
}

/// Channel type for a given 10 MHz slot
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ChannelType {
    /// PAL-protected channel (3550–3650 MHz)
    Pal,
    /// GAA-available channel (3650–3700 MHz, or PAL channel when PAL not active)
    Gaa,
    /// Channel currently restricted due to incumbent presence
    Restricted,
}

/// ESC sensor state
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EscState {
    /// No incumbent detected
    Clear,
    /// Incumbent signal detected above threshold
    IncumbentDetected,
    /// Sensor offline / data unavailable
    Offline,
}

// ---------------------------------------------------------------------------
// Data Structures
// ---------------------------------------------------------------------------

/// Geographic position (latitude, longitude, altitude)
#[derive(Debug, Clone)]
pub struct GeoPosition {
    /// Latitude in degrees WGS-84
    pub latitude_deg: f64,
    /// Longitude in degrees WGS-84
    pub longitude_deg: f64,
    /// Height above mean sea level in meters
    pub height_m: f64,
}

impl GeoPosition {
    pub fn new(lat: f64, lon: f64, height: f64) -> Self {
        GeoPosition { latitude_deg: lat, longitude_deg: lon, height_m: height }
    }

    /// Compute great-circle distance in meters using the haversine formula
    pub fn distance_m(&self, other: &GeoPosition) -> f64 {
        let lat1 = self.latitude_deg.to_radians();
        let lat2 = other.latitude_deg.to_radians();
        let dlat = (other.latitude_deg - self.latitude_deg).to_radians();
        let dlon = (other.longitude_deg - self.longitude_deg).to_radians();
        let a = (dlat / 2.0).sin().powi(2)
            + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
        EARTH_RADIUS_M * c
    }
}

/// CBSD registration information
#[derive(Debug, Clone)]
pub struct CbsdRegistration {
    /// Unique CBSD identifier assigned by SAS
    pub cbsd_id: String,
    /// FCC ID of the device model
    pub fcc_id: String,
    /// Hardware category
    pub category: CbsdCategory,
    /// Geographic location
    pub location: GeoPosition,
    /// Antenna height above ground level in meters
    pub antenna_height_agl_m: f64,
    /// Antenna gain in dBi
    pub antenna_gain_dbi: f64,
    /// Antenna azimuth in degrees (0 = North)
    pub antenna_azimuth_deg: f64,
    /// Antenna beamwidth in degrees (0 = omnidirectional)
    pub antenna_beamwidth_deg: f64,
    /// Indoor deployment flag
    pub indoor_deployment: bool,
    /// Census tract FIPS code (11 digits)
    pub census_tract_fips: String,
    /// Licensee identifier (empty string if GAA-only device)
    pub licensee_id: String,
}

impl CbsdRegistration {
    /// Compute maximum permissible EIRP (dBm) for this CBSD
    pub fn max_eirp_dbm(&self) -> f64 {
        self.category.max_eirp_dbm()
    }

    /// Compute conducted power limit for a given requested EIRP
    pub fn conducted_power_for_eirp(&self, eirp_dbm: f64) -> f64 {
        eirp_dbm - self.antenna_gain_dbi
    }
}

/// A single SAS grant record
#[derive(Debug, Clone)]
pub struct Grant {
    /// Unique grant identifier
    pub grant_id: String,
    /// CBSD this grant belongs to
    pub cbsd_id: String,
    /// Assigned channel index (0-based, maps to 10 MHz blocks from 3550 MHz)
    pub channel_index: usize,
    /// Granted EIRP in dBm
    pub max_eirp_dbm: f64,
    /// Current state
    pub state: GrantState,
    /// Grant issue timestamp (simulated seconds since epoch)
    pub issue_time_secs: u64,
    /// Grant expiry timestamp
    pub expiry_time_secs: u64,
    /// Last heartbeat timestamp
    pub last_heartbeat_secs: u64,
    /// Heartbeat interval in seconds
    pub heartbeat_interval_secs: u64,
    /// Access tier
    pub tier: AccessTier,
}

impl Grant {
    /// Return the center frequency in MHz for this grant's channel
    pub fn center_freq_mhz(&self) -> f64 {
        BAND_START_MHZ + (self.channel_index as f64 + 0.5) * CHANNEL_BW_MHZ
    }

    /// Return the low edge frequency in MHz
    pub fn low_freq_mhz(&self) -> f64 {
        BAND_START_MHZ + self.channel_index as f64 * CHANNEL_BW_MHZ
    }

    /// Return the high edge frequency in MHz
    pub fn high_freq_mhz(&self) -> f64 {
        self.low_freq_mhz() + CHANNEL_BW_MHZ
    }

    /// Check if the grant has expired
    pub fn is_expired(&self, current_time_secs: u64) -> bool {
        current_time_secs >= self.expiry_time_secs
    }

    /// Check if heartbeat is overdue
    pub fn heartbeat_overdue(&self, current_time_secs: u64) -> bool {
        current_time_secs > self.last_heartbeat_secs + self.heartbeat_interval_secs
    }
}

/// PAL license record
#[derive(Debug, Clone)]
pub struct PalLicense {
    /// PAL license identifier
    pub pal_id: String,
    /// Licensee identifier
    pub licensee_id: String,
    /// Census tract FIPS code
    pub census_tract_fips: String,
    /// Assigned channel index
    pub channel_index: usize,
    /// License expiry in simulated seconds
    pub expiry_time_secs: u64,
}

/// Dynamic Protection Area (DPA) record
#[derive(Debug, Clone)]
pub struct DpaRecord {
    /// DPA identifier
    pub dpa_id: String,
    /// Center location of the DPA
    pub center: GeoPosition,
    /// Exclusion zone radius in km (no transmission within)
    pub exclusion_radius_km: f64,
    /// Co-channel interference protection radius in km
    pub protection_radius_km: f64,
    /// Adjacent channel protection radius in km
    pub adj_channel_radius_km: f64,
    /// Aggregate interference limit (dBm/10 MHz)
    pub interference_limit_dbm: f64,
    /// Channels protected by this DPA
    pub protected_channels: Vec<usize>,
    /// Whether this DPA is currently active
    pub active: bool,
}

impl DpaRecord {
    /// Check if a CBSD location is within the exclusion zone
    pub fn in_exclusion_zone(&self, cbsd_pos: &GeoPosition) -> bool {
        let dist_km = self.center.distance_m(cbsd_pos) / 1000.0;
        dist_km <= self.exclusion_radius_km
    }

    /// Check if a CBSD location is within the protection contour
    pub fn in_protection_contour(&self, cbsd_pos: &GeoPosition) -> bool {
        let dist_km = self.center.distance_m(cbsd_pos) / 1000.0;
        dist_km <= self.protection_radius_km
    }
}

/// ESC sensor report
#[derive(Debug, Clone)]
pub struct EscSensorReport {
    /// Sensor identifier
    pub sensor_id: String,
    /// Sensor location
    pub location: GeoPosition,
    /// Current state
    pub state: EscState,
    /// Measured power in dBm/10 MHz (None if sensor offline)
    pub measured_power_dbm: Option<f64>,
    /// Report timestamp
    pub timestamp_secs: u64,
    /// Channels flagged as having incumbent presence
    pub incumbent_channels: Vec<usize>,
}

/// SAS grant request from CBSD
#[derive(Debug, Clone)]
pub struct GrantRequest {
    /// CBSD identifier
    pub cbsd_id: String,
    /// Requested channel index
    pub channel_index: usize,
    /// Requested EIRP in dBm
    pub requested_eirp_dbm: f64,
    /// Requested grant duration in seconds
    pub grant_duration_secs: u64,
}

/// SAS grant response to CBSD
#[derive(Debug, Clone)]
pub struct GrantResponse {
    /// Request echo
    pub cbsd_id: String,
    /// Issued grant (None if denied)
    pub grant: Option<Grant>,
    /// Response code
    pub response_code: SasResponseCode,
    /// Granted EIRP (may differ from requested)
    pub granted_eirp_dbm: Option<f64>,
    /// Granted channel (may differ from requested)
    pub granted_channel: Option<usize>,
}

/// Heartbeat request
#[derive(Debug, Clone)]
pub struct HeartbeatRequest {
    pub cbsd_id: String,
    pub grant_id: String,
    pub timestamp_secs: u64,
    /// Operational state reported by CBSD
    pub operation_state: String,
}

/// Heartbeat response
#[derive(Debug, Clone)]
pub struct HeartbeatResponse {
    pub cbsd_id: String,
    pub grant_id: String,
    pub response_code: SasResponseCode,
    /// Next heartbeat deadline in absolute seconds
    pub next_heartbeat_deadline_secs: u64,
    /// Transmit expire time
    pub transmit_expire_time_secs: u64,
    /// Whether grant is still authorized
    pub authorized: bool,
}

/// Channel descriptor for assignment
#[derive(Debug, Clone)]
pub struct ChannelDescriptor {
    /// Channel index (0-based from 3550 MHz)
    pub index: usize,
    /// Low edge frequency in MHz
    pub low_freq_mhz: f64,
    /// High edge frequency in MHz
    pub high_freq_mhz: f64,
    /// Center frequency in MHz
    pub center_freq_mhz: f64,
    /// Channel type
    pub channel_type: ChannelType,
    /// Number of active grants on this channel
    pub active_grants: usize,
    /// Aggregate interference estimate (dBm)
    pub aggregate_interference_dbm: f64,
    /// Whether this channel is available for assignment
    pub available: bool,
}

impl ChannelDescriptor {
    pub fn new(index: usize) -> Self {
        let low = BAND_START_MHZ + index as f64 * CHANNEL_BW_MHZ;
        let high = low + CHANNEL_BW_MHZ;
        let center = low + CHANNEL_BW_MHZ / 2.0;
        let ch_type = if index < NUM_PAL_CHANNELS {
            ChannelType::Pal
        } else {
            ChannelType::Gaa
        };
        ChannelDescriptor {
            index,
            low_freq_mhz: low,
            high_freq_mhz: high,
            center_freq_mhz: center,
            channel_type: ch_type,
            active_grants: 0,
            aggregate_interference_dbm: f64::NEG_INFINITY,
            available: true,
        }
    }
}

// ---------------------------------------------------------------------------
// Propagation Model
// ---------------------------------------------------------------------------

/// Simplified propagation model combining free-space and terrain factors.
/// Models ITU-R P.1546 / Longley-Rice concepts at a simplified level.
pub struct PropagationModel;

impl PropagationModel {
    /// Free-space path loss in dB at given frequency (MHz) and distance (m)
    pub fn fspl_db(freq_mhz: f64, distance_m: f64) -> f64 {
        if distance_m <= 0.0 {
            return 0.0;
        }
        let freq_hz = freq_mhz * 1e6;
        let lambda = SPEED_OF_LIGHT / freq_hz;
        // Convert: FSPL(dB) = 20*log10(4*pi*d/lambda)
        20.0 * (4.0 * std::f64::consts::PI * distance_m / lambda).log10()
    }

    /// Extended Hata / simplified ITM path loss (dB) for CBRS 3.5 GHz band.
    /// Uses the ITM-like formula:
    ///   PL = A + B*log10(d_km) + C*log10(f_mhz) + terrain_factor
    pub fn path_loss_db(
        freq_mhz: f64,
        distance_m: f64,
        tx_height_m: f64,
        rx_height_m: f64,
        terrain_factor_db: f64,
    ) -> f64 {
        if distance_m <= D0_M {
            return 0.0;
        }
        let d_km = distance_m / 1000.0;
        // Extended Hata-like model for 3.5 GHz
        let a = 69.55 + 26.16 * freq_mhz.log10() - 13.82 * tx_height_m.max(1.0).log10();
        let b = 44.9 - 6.55 * tx_height_m.max(1.0).log10();
        let rx_corr = 3.2 * (11.75 * rx_height_m.max(0.5)).log10().powi(2) - 4.97;
        let pl = a - rx_corr + b * d_km.max(0.001).log10() + terrain_factor_db;
        pl.max(0.0)
    }

    /// Compute received power (dBm) given transmit EIRP and path loss
    pub fn received_power_dbm(tx_eirp_dbm: f64, path_loss_db: f64) -> f64 {
        tx_eirp_dbm - path_loss_db
    }

    /// Compute interference power at DPA from a single CBSD
    pub fn interference_at_dpa_dbm(
        cbsd: &CbsdRegistration,
        eirp_dbm: f64,
        dpa: &DpaRecord,
        freq_mhz: f64,
    ) -> f64 {
        let dist_m = cbsd.location.distance_m(&dpa.center);
        let pl = Self::path_loss_db(freq_mhz, dist_m, cbsd.antenna_height_agl_m, 5.0, 0.0);
        eirp_dbm - pl
    }
}

// ---------------------------------------------------------------------------
// ESC Sensor Network
// ---------------------------------------------------------------------------

/// ESC sensor network – aggregates reports from distributed sensors
pub struct EscSensorNetwork {
    sensors: HashMap<String, EscSensorReport>,
    detection_threshold_dbm: f64,
}

impl EscSensorNetwork {
    pub fn new() -> Self {
        EscSensorNetwork {
            sensors: HashMap::new(),
            detection_threshold_dbm: ESC_DETECTION_THRESHOLD_DBM,
        }
    }

    /// Update a sensor report
    pub fn update_sensor(&mut self, report: EscSensorReport) {
        self.sensors.insert(report.sensor_id.clone(), report);
    }

    /// Returns the set of channels currently flagged as having incumbent presence
    pub fn incumbent_channels(&self, current_time_secs: u64) -> Vec<usize> {
        let max_report_age_secs: u64 = 60;
        let mut flagged: std::collections::HashSet<usize> = std::collections::HashSet::new();
        for report in self.sensors.values() {
            let age = current_time_secs.saturating_sub(report.timestamp_secs);
            if age > max_report_age_secs {
                continue; // stale report
            }
            if report.state == EscState::IncumbentDetected {
                for &ch in &report.incumbent_channels {
                    flagged.insert(ch);
                }
            }
            if let Some(power) = report.measured_power_dbm {
                if power >= self.detection_threshold_dbm {
                    // Flag all PAL channels as protected when above threshold
                    for ch in 0..NUM_PAL_CHANNELS {
                        flagged.insert(ch);
                    }
                }
            }
        }
        let mut result: Vec<usize> = flagged.into_iter().collect();
        result.sort_unstable();
        result
    }

    /// Generate a move list – list of CBSDs that must cease or adjust operation
    pub fn generate_move_list<'a>(
        &self,
        grants: &'a HashMap<String, Grant>,
        current_time_secs: u64,
    ) -> Vec<&'a Grant> {
        let incumbent = self.incumbent_channels(current_time_secs);
        grants
            .values()
            .filter(|g| {
                g.state == GrantState::Authorized && incumbent.contains(&g.channel_index)
            })
            .collect()
    }
}

impl Default for EscSensorNetwork {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// DPA Manager
// ---------------------------------------------------------------------------

/// DPA manager – maintains protection areas and computes aggregate interference
pub struct DpaManager {
    dpas: HashMap<String, DpaRecord>,
}

impl DpaManager {
    pub fn new() -> Self {
        DpaManager { dpas: HashMap::new() }
    }

    /// Register a DPA
    pub fn add_dpa(&mut self, dpa: DpaRecord) {
        self.dpas.insert(dpa.dpa_id.clone(), dpa);
    }

    /// Check if a CBSD at the given location can operate on a channel
    /// given the current set of grants and registered CBSDs.
    pub fn can_operate(
        &self,
        cbsd: &CbsdRegistration,
        channel_index: usize,
        requested_eirp_dbm: f64,
        cbsd_registry: &HashMap<String, CbsdRegistration>,
        grants: &HashMap<String, Grant>,
    ) -> (bool, f64) {
        let freq_mhz =
            BAND_START_MHZ + (channel_index as f64 + 0.5) * CHANNEL_BW_MHZ;

        for dpa in self.dpas.values() {
            if !dpa.active {
                continue;
            }
            if !dpa.protected_channels.contains(&channel_index) {
                continue;
            }

            // Check exclusion zone
            if dpa.in_exclusion_zone(&cbsd.location) {
                return (false, f64::NEG_INFINITY);
            }

            // Compute aggregate interference at DPA from existing grants
            let mut agg_interference_mw: f64 = 0.0;
            for grant in grants.values() {
                if grant.channel_index != channel_index {
                    continue;
                }
                if grant.state != GrantState::Authorized {
                    continue;
                }
                if let Some(other_cbsd) = cbsd_registry.get(&grant.cbsd_id) {
                    let i_dbm = PropagationModel::interference_at_dpa_dbm(
                        other_cbsd,
                        grant.max_eirp_dbm,
                        dpa,
                        freq_mhz,
                    );
                    agg_interference_mw += dbm_to_mw(i_dbm);
                }
            }

            // Check if adding this CBSD would exceed the limit
            let new_i_dbm = PropagationModel::interference_at_dpa_dbm(
                cbsd,
                requested_eirp_dbm,
                dpa,
                freq_mhz,
            );
            let new_agg_mw = agg_interference_mw + dbm_to_mw(new_i_dbm);
            let new_agg_dbm = mw_to_dbm(new_agg_mw);

            if new_agg_dbm > dpa.interference_limit_dbm {
                // Try to compute max allowable EIRP
                let headroom_dbm = dpa.interference_limit_dbm - mw_to_dbm(agg_interference_mw);
                let path_loss = PropagationModel::path_loss_db(
                    freq_mhz,
                    cbsd.location.distance_m(&dpa.center),
                    cbsd.antenna_height_agl_m,
                    5.0,
                    0.0,
                );
                let max_eirp = headroom_dbm + path_loss;
                if max_eirp < -50.0 {
                    return (false, max_eirp);
                }
                let capped = max_eirp.min(cbsd.max_eirp_dbm());
                return (capped > -50.0, capped);
            }
        }
        (true, requested_eirp_dbm)
    }

    /// Compute protection contour for a DPA at a given distance (returns dBm limit)
    pub fn protection_contour_dbm(&self, dpa_id: &str, distance_km: f64) -> Option<f64> {
        let dpa = self.dpas.get(dpa_id)?;
        if distance_km <= dpa.exclusion_radius_km {
            return Some(f64::NEG_INFINITY); // no operation allowed
        }
        if distance_km > dpa.protection_radius_km {
            return Some(dpa.interference_limit_dbm + 20.0); // relaxed zone
        }
        // Linear interpolation within protection zone
        let frac = (distance_km - dpa.exclusion_radius_km)
            / (dpa.protection_radius_km - dpa.exclusion_radius_km);
        Some(dpa.interference_limit_dbm + frac * 20.0)
    }
}

impl Default for DpaManager {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// PAL Manager
// ---------------------------------------------------------------------------

/// PAL license registry and channel assignment logic
pub struct PalManager {
    licenses: HashMap<String, PalLicense>,
}

impl PalManager {
    pub fn new() -> Self {
        PalManager { licenses: HashMap::new() }
    }

    /// Register a PAL license
    pub fn add_license(&mut self, license: PalLicense) {
        self.licenses.insert(license.pal_id.clone(), license);
    }

    /// Return PAL channels assigned to a licensee in a given census tract
    pub fn licensee_channels(&self, licensee_id: &str, fips: &str) -> Vec<usize> {
        self.licenses
            .values()
            .filter(|l| l.licensee_id == licensee_id && l.census_tract_fips == fips)
            .map(|l| l.channel_index)
            .collect()
    }

    /// Check if a CBSD is a PAL holder for the given channel in its tract.
    /// The CBSD must have a non-empty licensee_id matching a PAL license in its tract.
    pub fn is_pal_holder(&self, cbsd: &CbsdRegistration, channel_index: usize) -> bool {
        if cbsd.licensee_id.is_empty() {
            return false;
        }
        self.licenses.values().any(|l| {
            l.census_tract_fips == cbsd.census_tract_fips
                && l.channel_index == channel_index
                && l.licensee_id == cbsd.licensee_id
        })
    }

    /// Return all channels licensed in a given census tract
    pub fn channels_in_tract(&self, fips: &str) -> Vec<usize> {
        self.licenses
            .values()
            .filter(|l| l.census_tract_fips == fips)
            .map(|l| l.channel_index)
            .collect()
    }

    /// Validate that a licensee does not exceed 7 PAL channels per tract
    pub fn validate_channel_limit(&self, licensee_id: &str, fips: &str) -> bool {
        self.licensee_channels(licensee_id, fips).len() <= MAX_PAL_CHANNELS_PER_LICENSEE
    }
}

impl Default for PalManager {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Channel Manager
// ---------------------------------------------------------------------------

/// Channel state and assignment manager
pub struct ChannelManager {
    channels: Vec<ChannelDescriptor>,
    /// Channels blocked by ESC / incumbent
    blocked_channels: Vec<usize>,
}

impl ChannelManager {
    pub fn new() -> Self {
        let channels = (0..NUM_CHANNELS).map(ChannelDescriptor::new).collect();
        ChannelManager { channels, blocked_channels: Vec::new() }
    }

    /// Update blocked channels from ESC reports
    pub fn update_blocked_channels(&mut self, blocked: Vec<usize>) {
        self.blocked_channels = blocked;
        for ch in &mut self.channels {
            let is_blocked = self.blocked_channels.contains(&ch.index);
            if is_blocked {
                ch.channel_type = ChannelType::Restricted;
                ch.available = false;
            } else if ch.index < NUM_PAL_CHANNELS {
                ch.channel_type = ChannelType::Pal;
                ch.available = true;
            } else {
                ch.channel_type = ChannelType::Gaa;
                ch.available = true;
            }
        }
    }

    /// Return available channels for GAA operation
    pub fn available_gaa_channels(&self) -> Vec<usize> {
        self.channels
            .iter()
            .filter(|c| c.available)
            .map(|c| c.index)
            .collect()
    }

    /// Return available PAL channels (not blocked)
    pub fn available_pal_channels(&self) -> Vec<usize> {
        self.channels
            .iter()
            .filter(|c| c.available && c.channel_type == ChannelType::Pal)
            .map(|c| c.index)
            .collect()
    }

    /// Find the least-loaded available channel for a CBSD
    pub fn find_best_channel(&self, tier: &AccessTier, pal_channels: &[usize]) -> Option<usize> {
        match tier {
            AccessTier::PriorityAccess => {
                // PAL CBSD prefers its licensed channels
                self.channels
                    .iter()
                    .filter(|c| c.available && pal_channels.contains(&c.index))
                    .min_by_key(|c| c.active_grants)
                    .map(|c| c.index)
            }
            AccessTier::GeneralAuthorizedAccess => {
                // GAA prefers channels with fewest active grants
                self.channels
                    .iter()
                    .filter(|c| c.available)
                    .min_by_key(|c| c.active_grants)
                    .map(|c| c.index)
            }
            AccessTier::IncumbentAccess => None,
        }
    }

    /// Increment grant count on a channel
    pub fn increment_grants(&mut self, channel_index: usize) {
        if let Some(ch) = self.channels.get_mut(channel_index) {
            ch.active_grants += 1;
        }
    }

    /// Decrement grant count on a channel
    pub fn decrement_grants(&mut self, channel_index: usize) {
        if let Some(ch) = self.channels.get_mut(channel_index) {
            ch.active_grants = ch.active_grants.saturating_sub(1);
        }
    }

    /// Get channel descriptor
    pub fn channel(&self, index: usize) -> Option<&ChannelDescriptor> {
        self.channels.get(index)
    }

    /// Return all channels
    pub fn all_channels(&self) -> &[ChannelDescriptor] {
        &self.channels
    }
}

impl Default for ChannelManager {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// GAA Coexistence Manager
// ---------------------------------------------------------------------------

/// GAA-to-GAA coexistence coordinator using time-division sharing
pub struct GaaCoexistenceManager {
    /// Current time slot (used for time-division sharing)
    current_slot: u64,
    /// Number of time slots per cycle
    slots_per_cycle: u64,
    /// CBSD-to-slot assignment
    cbsd_slot_assignments: HashMap<String, u64>,
}

impl GaaCoexistenceManager {
    pub fn new(slots_per_cycle: u64) -> Self {
        GaaCoexistenceManager {
            current_slot: 0,
            slots_per_cycle,
            cbsd_slot_assignments: HashMap::new(),
        }
    }

    /// Advance to the next time slot
    pub fn advance_slot(&mut self) {
        self.current_slot = (self.current_slot + 1) % self.slots_per_cycle;
    }

    /// Assign a time slot to a CBSD
    pub fn assign_slot(&mut self, cbsd_id: &str) -> u64 {
        let n = self.cbsd_slot_assignments.len() as u64;
        let slot = n % self.slots_per_cycle;
        self.cbsd_slot_assignments.insert(cbsd_id.to_string(), slot);
        slot
    }

    /// Check if a CBSD can transmit in the current slot
    pub fn can_transmit(&self, cbsd_id: &str) -> bool {
        match self.cbsd_slot_assignments.get(cbsd_id) {
            Some(&slot) => slot == self.current_slot,
            None => false,
        }
    }

    /// Remove a CBSD from slot assignment
    pub fn remove_cbsd(&mut self, cbsd_id: &str) {
        self.cbsd_slot_assignments.remove(cbsd_id);
    }

    /// Return current slot
    pub fn current_slot(&self) -> u64 {
        self.current_slot
    }
}

// ---------------------------------------------------------------------------
// Power Controller
// ---------------------------------------------------------------------------

/// CBRS power control module
pub struct PowerController;

impl PowerController {
    /// Compute the maximum permissible EIRP (dBm) for a CBSD on a channel,
    /// considering category limits and DPA constraints.
    pub fn max_eirp(
        cbsd: &CbsdRegistration,
        dpa_constraint_dbm: Option<f64>,
    ) -> f64 {
        let cat_limit = cbsd.max_eirp_dbm();
        match dpa_constraint_dbm {
            Some(dpa_limit) => cat_limit.min(dpa_limit),
            None => cat_limit,
        }
    }

    /// Validate that a requested EIRP does not exceed limits
    pub fn validate_eirp(cbsd: &CbsdRegistration, requested_eirp_dbm: f64) -> bool {
        requested_eirp_dbm <= cbsd.max_eirp_dbm()
    }

    /// Compute maximum conducted power given EIRP and antenna gain
    pub fn max_conducted_power(eirp_dbm: f64, antenna_gain_dbi: f64) -> f64 {
        eirp_dbm - antenna_gain_dbi
    }

    /// Apply power control step: reduce EIRP by step_db
    pub fn reduce_eirp(current_eirp_dbm: f64, step_db: f64) -> f64 {
        current_eirp_dbm - step_db
    }
}

// ---------------------------------------------------------------------------
// Main SAS Processor
// ---------------------------------------------------------------------------

/// Central SAS processor – manages CBSD registration, grant lifecycle,
/// DPA protection, ESC incumbent detection, and channel assignment.
pub struct SasProcessor {
    /// Registered CBSDs
    cbsd_registry: HashMap<String, CbsdRegistration>,
    /// Active grants
    grants: HashMap<String, Grant>,
    /// PAL license manager
    pal_manager: PalManager,
    /// DPA manager
    dpa_manager: DpaManager,
    /// ESC sensor network
    esc_network: EscSensorNetwork,
    /// Channel manager
    channel_manager: ChannelManager,
    /// GAA coexistence manager
    gaa_coexistence: GaaCoexistenceManager,
    /// Grant ID counter
    grant_id_counter: u64,
    /// Current simulated time in seconds
    current_time_secs: u64,
    /// Default heartbeat interval
    default_heartbeat_interval_secs: u64,
    /// Default grant duration
    default_grant_duration_secs: u64,
}

impl SasProcessor {
    /// Create a new SAS processor instance
    pub fn new() -> Self {
        SasProcessor {
            cbsd_registry: HashMap::new(),
            grants: HashMap::new(),
            pal_manager: PalManager::new(),
            dpa_manager: DpaManager::new(),
            esc_network: EscSensorNetwork::new(),
            channel_manager: ChannelManager::new(),
            gaa_coexistence: GaaCoexistenceManager::new(10),
            grant_id_counter: 1,
            current_time_secs: 0,
            default_heartbeat_interval_secs: HEARTBEAT_MIN_SECS,
            default_grant_duration_secs: DEFAULT_GRANT_DURATION_SECS,
        }
    }

    /// Advance the simulated clock
    pub fn advance_time(&mut self, delta_secs: u64) {
        self.current_time_secs += delta_secs;
        self.expire_grants();
    }

    /// Register a CBSD device with the SAS
    pub fn register_cbsd(&mut self, reg: CbsdRegistration) -> Result<String, SasResponseCode> {
        // Validate category
        if reg.category == CbsdCategory::CategoryA && reg.antenna_gain_dbi > CAT_A_MAX_ANTENNA_GAIN_DBI {
            return Err(SasResponseCode::InvalidValue);
        }
        let id = reg.cbsd_id.clone();
        self.cbsd_registry.insert(id.clone(), reg);
        Ok(id)
    }

    /// Deregister a CBSD and terminate all its grants
    pub fn deregister_cbsd(&mut self, cbsd_id: &str) {
        // Terminate all grants for this CBSD
        let grant_ids: Vec<String> = self
            .grants
            .values()
            .filter(|g| g.cbsd_id == cbsd_id)
            .map(|g| g.grant_id.clone())
            .collect();
        for gid in grant_ids {
            self.terminate_grant_internal(&gid);
        }
        self.cbsd_registry.remove(cbsd_id);
        self.gaa_coexistence.remove_cbsd(cbsd_id);
    }

    /// Process a grant request from a CBSD
    pub fn request_grant(&mut self, req: GrantRequest) -> GrantResponse {
        // Verify CBSD is registered
        let cbsd = match self.cbsd_registry.get(&req.cbsd_id) {
            Some(c) => c.clone(),
            None => {
                return GrantResponse {
                    cbsd_id: req.cbsd_id,
                    grant: None,
                    response_code: SasResponseCode::MissingParam,
                    granted_eirp_dbm: None,
                    granted_channel: None,
                };
            }
        };

        // Validate requested EIRP
        if !PowerController::validate_eirp(&cbsd, req.requested_eirp_dbm) {
            return GrantResponse {
                cbsd_id: req.cbsd_id,
                grant: None,
                response_code: SasResponseCode::InvalidValue,
                granted_eirp_dbm: None,
                granted_channel: None,
            };
        }

        // Validate channel is in band
        if req.channel_index >= NUM_CHANNELS {
            return GrantResponse {
                cbsd_id: req.cbsd_id,
                grant: None,
                response_code: SasResponseCode::InvalidValue,
                granted_eirp_dbm: None,
                granted_channel: None,
            };
        }

        // Check ESC / incumbent blocking
        let blocked = self.esc_network.incumbent_channels(self.current_time_secs);
        if blocked.contains(&req.channel_index) {
            return GrantResponse {
                cbsd_id: req.cbsd_id,
                grant: None,
                response_code: SasResponseCode::InterferenceConstraint,
                granted_eirp_dbm: None,
                granted_channel: None,
            };
        }

        // Check channel availability
        let ch = match self.channel_manager.channel(req.channel_index) {
            Some(c) => c.clone(),
            None => {
                return GrantResponse {
                    cbsd_id: req.cbsd_id,
                    grant: None,
                    response_code: SasResponseCode::InvalidValue,
                    granted_eirp_dbm: None,
                    granted_channel: None,
                };
            }
        };

        if !ch.available {
            return GrantResponse {
                cbsd_id: req.cbsd_id,
                grant: None,
                response_code: SasResponseCode::InterferenceConstraint,
                granted_eirp_dbm: None,
                granted_channel: None,
            };
        }

        // Determine access tier
        let is_pal = self.pal_manager.is_pal_holder(&cbsd, req.channel_index);
        let tier = if is_pal {
            AccessTier::PriorityAccess
        } else {
            AccessTier::GeneralAuthorizedAccess
        };

        // For PAL channel requested by GAA, check PAL protection
        if req.channel_index < NUM_PAL_CHANNELS && !is_pal {
            // GAA on PAL channel is allowed only if no PAL is active on that channel
            let pal_active = self.grants.values().any(|g| {
                g.channel_index == req.channel_index
                    && g.tier == AccessTier::PriorityAccess
                    && g.state == GrantState::Authorized
            });
            if pal_active {
                return GrantResponse {
                    cbsd_id: req.cbsd_id,
                    grant: None,
                    response_code: SasResponseCode::InterferenceConstraint,
                    granted_eirp_dbm: None,
                    granted_channel: None,
                };
            }
        }

        // DPA check
        let (can_op, max_eirp_dpa) = self.dpa_manager.can_operate(
            &cbsd,
            req.channel_index,
            req.requested_eirp_dbm,
            &self.cbsd_registry,
            &self.grants,
        );

        if !can_op {
            return GrantResponse {
                cbsd_id: req.cbsd_id,
                grant: None,
                response_code: SasResponseCode::InterferenceConstraint,
                granted_eirp_dbm: None,
                granted_channel: None,
            };
        }

        let granted_eirp = req.requested_eirp_dbm.min(max_eirp_dpa).min(cbsd.max_eirp_dbm());
        let grant_duration = req
            .grant_duration_secs
            .min(self.default_grant_duration_secs);

        // Issue the grant
        let grant_id = format!("GRANT-{:08}", self.grant_id_counter);
        self.grant_id_counter += 1;

        // Assign GAA coexistence slot if needed
        if tier == AccessTier::GeneralAuthorizedAccess {
            self.gaa_coexistence.assign_slot(&req.cbsd_id);
        }

        let grant = Grant {
            grant_id: grant_id.clone(),
            cbsd_id: req.cbsd_id.clone(),
            channel_index: req.channel_index,
            max_eirp_dbm: granted_eirp,
            state: GrantState::Granted,
            issue_time_secs: self.current_time_secs,
            expiry_time_secs: self.current_time_secs + grant_duration,
            last_heartbeat_secs: self.current_time_secs,
            heartbeat_interval_secs: self.default_heartbeat_interval_secs,
            tier,
        };

        self.channel_manager.increment_grants(req.channel_index);
        self.grants.insert(grant_id, grant.clone());

        GrantResponse {
            cbsd_id: req.cbsd_id,
            grant: Some(grant.clone()),
            response_code: SasResponseCode::Success,
            granted_eirp_dbm: Some(granted_eirp),
            granted_channel: Some(req.channel_index),
        }
    }

    /// Authorize a grant (transition Granted → Authorized)
    pub fn authorize_grant(&mut self, grant_id: &str) -> SasResponseCode {
        match self.grants.get_mut(grant_id) {
            Some(g) if g.state == GrantState::Granted => {
                g.state = GrantState::Authorized;
                SasResponseCode::Success
            }
            Some(g) if g.state == GrantState::Authorized => SasResponseCode::Success,
            Some(_) => SasResponseCode::Unsync,
            None => SasResponseCode::MissingParam,
        }
    }

    /// Process a heartbeat from a CBSD
    pub fn process_heartbeat(&mut self, hb: HeartbeatRequest) -> HeartbeatResponse {
        let grant = match self.grants.get_mut(&hb.grant_id) {
            Some(g) if g.cbsd_id == hb.cbsd_id => g,
            _ => {
                return HeartbeatResponse {
                    cbsd_id: hb.cbsd_id,
                    grant_id: hb.grant_id,
                    response_code: SasResponseCode::MissingParam,
                    next_heartbeat_deadline_secs: 0,
                    transmit_expire_time_secs: 0,
                    authorized: false,
                };
            }
        };

        // Check for expiry
        if grant.is_expired(hb.timestamp_secs) {
            grant.state = GrantState::Terminated;
            return HeartbeatResponse {
                cbsd_id: hb.cbsd_id,
                grant_id: hb.grant_id,
                response_code: SasResponseCode::Terminated,
                next_heartbeat_deadline_secs: 0,
                transmit_expire_time_secs: 0,
                authorized: false,
            };
        }

        // Update last heartbeat
        grant.last_heartbeat_secs = hb.timestamp_secs;
        let authorized = grant.state == GrantState::Authorized;
        let next_deadline = hb.timestamp_secs + grant.heartbeat_interval_secs;
        let transmit_expire = grant.expiry_time_secs;

        HeartbeatResponse {
            cbsd_id: hb.cbsd_id,
            grant_id: hb.grant_id,
            response_code: SasResponseCode::Success,
            next_heartbeat_deadline_secs: next_deadline,
            transmit_expire_time_secs: transmit_expire,
            authorized,
        }
    }

    /// Suspend a grant (Authorized → Suspended)
    pub fn suspend_grant(&mut self, grant_id: &str) -> SasResponseCode {
        match self.grants.get_mut(grant_id) {
            Some(g) if g.state == GrantState::Authorized => {
                g.state = GrantState::Suspended;
                SasResponseCode::Suspended
            }
            Some(_) => SasResponseCode::Unsync,
            None => SasResponseCode::MissingParam,
        }
    }

    /// Terminate a grant
    pub fn terminate_grant(&mut self, grant_id: &str) -> SasResponseCode {
        self.terminate_grant_internal(grant_id)
    }

    fn terminate_grant_internal(&mut self, grant_id: &str) -> SasResponseCode {
        match self.grants.get_mut(grant_id) {
            Some(g) => {
                let ch = g.channel_index;
                g.state = GrantState::Terminated;
                self.channel_manager.decrement_grants(ch);
                SasResponseCode::Success
            }
            None => SasResponseCode::MissingParam,
        }
    }

    /// Process ESC sensor report and update channel availability
    pub fn process_esc_report(&mut self, report: EscSensorReport) {
        self.esc_network.update_sensor(report);
        let blocked = self.esc_network.incumbent_channels(self.current_time_secs);
        self.channel_manager.update_blocked_channels(blocked.clone());

        // Suspend all authorized grants on blocked channels
        let to_suspend: Vec<String> = self
            .grants
            .values()
            .filter(|g| {
                g.state == GrantState::Authorized && blocked.contains(&g.channel_index)
            })
            .map(|g| g.grant_id.clone())
            .collect();

        for gid in to_suspend {
            self.suspend_grant(&gid);
        }
    }

    /// Expire grants that have timed out or heartbeat has been missed
    fn expire_grants(&mut self) {
        let now = self.current_time_secs;
        let to_terminate: Vec<String> = self
            .grants
            .values()
            .filter(|g| {
                (g.is_expired(now) || g.heartbeat_overdue(now))
                    && g.state != GrantState::Terminated
            })
            .map(|g| g.grant_id.clone())
            .collect();

        for gid in to_terminate {
            self.terminate_grant_internal(&gid);
        }
    }

    /// Add a PAL license to the registry
    pub fn add_pal_license(&mut self, license: PalLicense) {
        self.pal_manager.add_license(license);
    }

    /// Add a DPA to the protection area registry
    pub fn add_dpa(&mut self, dpa: DpaRecord) {
        self.dpa_manager.add_dpa(dpa);
    }

    /// Query all active grants for a CBSD
    pub fn cbsd_grants(&self, cbsd_id: &str) -> Vec<&Grant> {
        self.grants
            .values()
            .filter(|g| g.cbsd_id == cbsd_id && g.state != GrantState::Terminated)
            .collect()
    }

    /// Return the current channel summary
    pub fn channel_summary(&self) -> &[ChannelDescriptor] {
        self.channel_manager.all_channels()
    }

    /// Return the current simulated time
    pub fn current_time(&self) -> u64 {
        self.current_time_secs
    }

    /// Return number of active grants (not terminated)
    pub fn active_grant_count(&self) -> usize {
        self.grants.values().filter(|g| g.state != GrantState::Terminated).count()
    }

    /// Return all grants
    pub fn all_grants(&self) -> &HashMap<String, Grant> {
        &self.grants
    }

    /// Check if GAA CBSD can transmit in current coexistence slot
    pub fn gaa_can_transmit(&self, cbsd_id: &str) -> bool {
        self.gaa_coexistence.can_transmit(cbsd_id)
    }

    /// Advance GAA coexistence slot
    pub fn advance_gaa_slot(&mut self) {
        self.gaa_coexistence.advance_slot();
    }
}

impl Default for SasProcessor {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Convert dBm to milliwatts
pub fn dbm_to_mw(dbm: f64) -> f64 {
    if dbm == f64::NEG_INFINITY {
        return 0.0;
    }
    10_f64.powf(dbm / 10.0)
}

/// Convert milliwatts to dBm
pub fn mw_to_dbm(mw: f64) -> f64 {
    if mw <= 0.0 {
        return f64::NEG_INFINITY;
    }
    10.0 * mw.log10()
}

/// Compute aggregate interference in dBm from a list of individual dBm values
pub fn aggregate_interference_dbm(individual_dbm: &[f64]) -> f64 {
    let total_mw: f64 = individual_dbm.iter().map(|&d| dbm_to_mw(d)).sum();
    mw_to_dbm(total_mw)
}

/// Channel center frequency in MHz for a given channel index
pub fn channel_center_freq_mhz(channel_index: usize) -> f64 {
    BAND_START_MHZ + (channel_index as f64 + 0.5) * CHANNEL_BW_MHZ
}

/// Returns true if a frequency (MHz) falls in the CBRS band
pub fn in_cbrs_band(freq_mhz: f64) -> bool {
    freq_mhz >= BAND_START_MHZ && freq_mhz <= BAND_END_MHZ
}

/// Returns true if a channel index maps to a PAL channel
pub fn is_pal_channel(channel_index: usize) -> bool {
    channel_index < NUM_PAL_CHANNELS
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_cbsd_a(id: &str, lat: f64, lon: f64) -> CbsdRegistration {
        CbsdRegistration {
            cbsd_id: id.to_string(),
            fcc_id: "FCCTEST001".to_string(),
            category: CbsdCategory::CategoryA,
            location: GeoPosition::new(lat, lon, 0.0),
            antenna_height_agl_m: 5.0,
            antenna_gain_dbi: 2.0,
            antenna_azimuth_deg: 0.0,
            antenna_beamwidth_deg: 360.0,
            indoor_deployment: true,
            census_tract_fips: "06037000000".to_string(),
            licensee_id: String::new(), // GAA by default
        }
    }

    fn make_cbsd_b(id: &str, lat: f64, lon: f64) -> CbsdRegistration {
        CbsdRegistration {
            cbsd_id: id.to_string(),
            fcc_id: "FCCTEST002".to_string(),
            category: CbsdCategory::CategoryB,
            location: GeoPosition::new(lat, lon, 0.0),
            antenna_height_agl_m: 20.0,
            antenna_gain_dbi: 10.0,
            antenna_azimuth_deg: 0.0,
            antenna_beamwidth_deg: 90.0,
            indoor_deployment: false,
            census_tract_fips: "06037000000".to_string(),
            licensee_id: String::new(), // GAA by default
        }
    }

    fn make_cbsd_a_pal(id: &str, lat: f64, lon: f64, licensee_id: &str) -> CbsdRegistration {
        CbsdRegistration {
            cbsd_id: id.to_string(),
            fcc_id: "FCCTEST001".to_string(),
            category: CbsdCategory::CategoryA,
            location: GeoPosition::new(lat, lon, 0.0),
            antenna_height_agl_m: 5.0,
            antenna_gain_dbi: 2.0,
            antenna_azimuth_deg: 0.0,
            antenna_beamwidth_deg: 360.0,
            indoor_deployment: true,
            census_tract_fips: "06037000000".to_string(),
            licensee_id: licensee_id.to_string(),
        }
    }

    fn make_grant_req(cbsd_id: &str, channel: usize, eirp: f64) -> GrantRequest {
        GrantRequest {
            cbsd_id: cbsd_id.to_string(),
            channel_index: channel,
            requested_eirp_dbm: eirp,
            grant_duration_secs: DEFAULT_GRANT_DURATION_SECS,
        }
    }

    // --- Band constants tests ---

    #[test]
    fn test_band_constants() {
        assert_eq!(BAND_START_MHZ, 3550.0);
        assert_eq!(BAND_END_MHZ, 3700.0);
        assert_eq!(CHANNEL_BW_MHZ, 10.0);
        assert_eq!(NUM_CHANNELS, 15);
        assert_eq!(NUM_PAL_CHANNELS, 10);
        assert_eq!(NUM_GAA_ONLY_CHANNELS, 5);
    }

    #[test]
    fn test_eirp_limits() {
        assert_eq!(CAT_A_MAX_EIRP_DBM, 30.0);
        assert_eq!(CAT_B_MAX_EIRP_DBM, 47.0);
    }

    #[test]
    fn test_category_a_eirp() {
        let c = CbsdCategory::CategoryA;
        assert_eq!(c.max_eirp_dbm(), 30.0);
        assert_eq!(c.max_conducted_dbm(), 24.0);
    }

    #[test]
    fn test_category_b_eirp() {
        let c = CbsdCategory::CategoryB;
        assert_eq!(c.max_eirp_dbm(), 47.0);
        assert_eq!(c.max_conducted_dbm(), 30.0);
    }

    // --- Channel frequency mapping ---

    #[test]
    fn test_channel_center_freq() {
        assert!((channel_center_freq_mhz(0) - 3555.0).abs() < 1e-9);
        assert!((channel_center_freq_mhz(9) - 3645.0).abs() < 1e-9);
        assert!((channel_center_freq_mhz(14) - 3695.0).abs() < 1e-9);
    }

    #[test]
    fn test_channel_descriptor_creation() {
        let ch = ChannelDescriptor::new(0);
        assert_eq!(ch.low_freq_mhz, 3550.0);
        assert_eq!(ch.high_freq_mhz, 3560.0);
        assert_eq!(ch.center_freq_mhz, 3555.0);
        assert_eq!(ch.channel_type, ChannelType::Pal);

        let ch14 = ChannelDescriptor::new(14);
        assert_eq!(ch14.channel_type, ChannelType::Gaa);
    }

    #[test]
    fn test_is_pal_channel() {
        assert!(is_pal_channel(0));
        assert!(is_pal_channel(9));
        assert!(!is_pal_channel(10));
        assert!(!is_pal_channel(14));
    }

    #[test]
    fn test_in_cbrs_band() {
        assert!(in_cbrs_band(3550.0));
        assert!(in_cbrs_band(3625.0));
        assert!(in_cbrs_band(3700.0));
        assert!(!in_cbrs_band(3549.9));
        assert!(!in_cbrs_band(3700.1));
    }

    // --- Propagation model ---

    #[test]
    fn test_fspl_increases_with_distance() {
        let pl1 = PropagationModel::fspl_db(3600.0, 1000.0);
        let pl2 = PropagationModel::fspl_db(3600.0, 10000.0);
        assert!(pl2 > pl1, "FSPL should increase with distance");
    }

    #[test]
    fn test_fspl_increases_with_frequency() {
        let pl_low = PropagationModel::fspl_db(3550.0, 5000.0);
        let pl_high = PropagationModel::fspl_db(3700.0, 5000.0);
        assert!(pl_high > pl_low, "FSPL should increase with frequency");
    }

    #[test]
    fn test_path_loss_positive() {
        let pl = PropagationModel::path_loss_db(3600.0, 10000.0, 30.0, 1.5, 0.0);
        assert!(pl > 0.0);
    }

    #[test]
    fn test_path_loss_zero_distance() {
        let pl = PropagationModel::path_loss_db(3600.0, 0.5, 10.0, 1.5, 0.0);
        assert_eq!(pl, 0.0);
    }

    #[test]
    fn test_received_power() {
        let eirp = 30.0_f64;
        let pl = 100.0_f64;
        let rx = PropagationModel::received_power_dbm(eirp, pl);
        assert!((rx - (-70.0)).abs() < 1e-9);
    }

    // --- dBm/mW conversion ---

    #[test]
    fn test_dbm_to_mw() {
        assert!((dbm_to_mw(0.0) - 1.0).abs() < 1e-9);
        assert!((dbm_to_mw(10.0) - 10.0).abs() < 1e-6);
        assert!((dbm_to_mw(30.0) - 1000.0).abs() < 1e-4);
        assert_eq!(dbm_to_mw(f64::NEG_INFINITY), 0.0);
    }

    #[test]
    fn test_mw_to_dbm() {
        assert!((mw_to_dbm(1.0) - 0.0).abs() < 1e-9);
        assert!((mw_to_dbm(10.0) - 10.0).abs() < 1e-9);
        assert!(mw_to_dbm(0.0).is_infinite());
    }

    #[test]
    fn test_aggregate_interference() {
        // Two equal -100 dBm signals => aggregate should be -97 dBm (3 dB more)
        let agg = aggregate_interference_dbm(&[-100.0, -100.0]);
        assert!((agg - (-97.0)).abs() < 0.1);
    }

    // --- GeoPosition ---

    #[test]
    fn test_geo_distance_same_point() {
        let p = GeoPosition::new(34.0, -118.0, 0.0);
        assert!(p.distance_m(&p) < 1.0);
    }

    #[test]
    fn test_geo_distance_one_degree_lat() {
        let p1 = GeoPosition::new(34.0, -118.0, 0.0);
        let p2 = GeoPosition::new(35.0, -118.0, 0.0);
        let d = p1.distance_m(&p2);
        // ~111 km
        assert!(d > 110_000.0 && d < 113_000.0);
    }

    // --- CBSD Registration ---

    #[test]
    fn test_register_cbsd_success() {
        let mut sas = SasProcessor::new();
        let cbsd = make_cbsd_a("CBSD-001", 34.0, -118.0);
        let result = sas.register_cbsd(cbsd);
        assert!(result.is_ok());
    }

    #[test]
    fn test_register_cbsd_cat_a_antenna_gain_too_high() {
        let mut sas = SasProcessor::new();
        let mut cbsd = make_cbsd_a("CBSD-001", 34.0, -118.0);
        cbsd.antenna_gain_dbi = 10.0; // exceeds Cat A limit of 6 dBi
        let result = sas.register_cbsd(cbsd);
        assert_eq!(result, Err(SasResponseCode::InvalidValue));
    }

    #[test]
    fn test_register_cbsd_b_high_gain_allowed() {
        let mut sas = SasProcessor::new();
        let cbsd = make_cbsd_b("CBSD-B-001", 34.0, -118.0);
        let result = sas.register_cbsd(cbsd);
        assert!(result.is_ok());
    }

    // --- Grant lifecycle ---

    #[test]
    fn test_grant_request_success() {
        let mut sas = SasProcessor::new();
        sas.register_cbsd(make_cbsd_a("C1", 34.0, -118.0)).unwrap();
        let resp = sas.request_grant(make_grant_req("C1", 5, 25.0));
        assert_eq!(resp.response_code, SasResponseCode::Success);
        assert!(resp.grant.is_some());
        assert_eq!(resp.granted_channel, Some(5));
    }

    #[test]
    fn test_grant_eirp_capped_at_category_limit() {
        let mut sas = SasProcessor::new();
        sas.register_cbsd(make_cbsd_a("C1", 34.0, -118.0)).unwrap();
        let resp = sas.request_grant(make_grant_req("C1", 0, 35.0)); // exceeds Cat A 30 dBm
        // Should still grant but cap at 30 dBm
        assert_eq!(resp.response_code, SasResponseCode::InvalidValue);
    }

    #[test]
    fn test_grant_unregistered_cbsd() {
        let mut sas = SasProcessor::new();
        let resp = sas.request_grant(make_grant_req("UNKNOWN", 0, 25.0));
        assert_eq!(resp.response_code, SasResponseCode::MissingParam);
    }

    #[test]
    fn test_grant_invalid_channel() {
        let mut sas = SasProcessor::new();
        sas.register_cbsd(make_cbsd_a("C1", 34.0, -118.0)).unwrap();
        let resp = sas.request_grant(make_grant_req("C1", 99, 25.0));
        assert_eq!(resp.response_code, SasResponseCode::InvalidValue);
    }

    #[test]
    fn test_authorize_grant() {
        let mut sas = SasProcessor::new();
        sas.register_cbsd(make_cbsd_a("C1", 34.0, -118.0)).unwrap();
        let resp = sas.request_grant(make_grant_req("C1", 2, 25.0));
        let gid = resp.grant.unwrap().grant_id;
        let code = sas.authorize_grant(&gid);
        assert_eq!(code, SasResponseCode::Success);
        let grant = sas.all_grants().get(&gid).unwrap();
        assert_eq!(grant.state, GrantState::Authorized);
    }

    #[test]
    fn test_heartbeat_success() {
        let mut sas = SasProcessor::new();
        sas.register_cbsd(make_cbsd_a("C1", 34.0, -118.0)).unwrap();
        let resp = sas.request_grant(make_grant_req("C1", 3, 20.0));
        let gid = resp.grant.unwrap().grant_id;
        sas.authorize_grant(&gid);

        let hb = HeartbeatRequest {
            cbsd_id: "C1".to_string(),
            grant_id: gid.clone(),
            timestamp_secs: 30,
            operation_state: "AUTHORIZED".to_string(),
        };
        let hb_resp = sas.process_heartbeat(hb);
        assert_eq!(hb_resp.response_code, SasResponseCode::Success);
        assert!(hb_resp.authorized);
    }

    #[test]
    fn test_heartbeat_expired_grant() {
        let mut sas = SasProcessor::new();
        sas.register_cbsd(make_cbsd_a("C1", 34.0, -118.0)).unwrap();
        let resp = sas.request_grant(make_grant_req("C1", 3, 20.0));
        let gid = resp.grant.unwrap().grant_id;
        sas.authorize_grant(&gid);

        // Send heartbeat well after expiry
        let hb = HeartbeatRequest {
            cbsd_id: "C1".to_string(),
            grant_id: gid,
            timestamp_secs: DEFAULT_GRANT_DURATION_SECS + 100,
            operation_state: "AUTHORIZED".to_string(),
        };
        let hb_resp = sas.process_heartbeat(hb);
        assert_eq!(hb_resp.response_code, SasResponseCode::Terminated);
        assert!(!hb_resp.authorized);
    }

    #[test]
    fn test_suspend_grant() {
        let mut sas = SasProcessor::new();
        sas.register_cbsd(make_cbsd_a("C1", 34.0, -118.0)).unwrap();
        let resp = sas.request_grant(make_grant_req("C1", 1, 25.0));
        let gid = resp.grant.unwrap().grant_id;
        sas.authorize_grant(&gid);
        let code = sas.suspend_grant(&gid);
        assert_eq!(code, SasResponseCode::Suspended);
        assert_eq!(sas.all_grants().get(&gid).unwrap().state, GrantState::Suspended);
    }

    #[test]
    fn test_terminate_grant() {
        let mut sas = SasProcessor::new();
        sas.register_cbsd(make_cbsd_a("C1", 34.0, -118.0)).unwrap();
        let resp = sas.request_grant(make_grant_req("C1", 4, 25.0));
        let gid = resp.grant.unwrap().grant_id;
        sas.authorize_grant(&gid);
        let code = sas.terminate_grant(&gid);
        assert_eq!(code, SasResponseCode::Success);
        assert_eq!(sas.active_grant_count(), 0);
    }

    // --- ESC / Incumbent detection ---

    #[test]
    fn test_esc_blocks_channel() {
        let mut sas = SasProcessor::new();
        sas.register_cbsd(make_cbsd_a("C1", 34.0, -118.0)).unwrap();

        let report = EscSensorReport {
            sensor_id: "ESC-1".to_string(),
            location: GeoPosition::new(34.0, -119.0, 0.0),
            state: EscState::IncumbentDetected,
            measured_power_dbm: Some(-100.0),
            timestamp_secs: 0,
            incumbent_channels: vec![0, 1, 2, 3, 4],
        };
        sas.process_esc_report(report);

        let resp = sas.request_grant(make_grant_req("C1", 2, 25.0));
        assert_eq!(resp.response_code, SasResponseCode::InterferenceConstraint);
    }

    #[test]
    fn test_esc_suspends_authorized_grants() {
        let mut sas = SasProcessor::new();
        sas.register_cbsd(make_cbsd_a("C1", 34.0, -118.0)).unwrap();
        let resp = sas.request_grant(make_grant_req("C1", 0, 25.0));
        let gid = resp.grant.unwrap().grant_id;
        sas.authorize_grant(&gid);

        let report = EscSensorReport {
            sensor_id: "ESC-1".to_string(),
            location: GeoPosition::new(34.0, -119.0, 0.0),
            state: EscState::IncumbentDetected,
            measured_power_dbm: Some(-100.0),
            timestamp_secs: 0,
            incumbent_channels: vec![0],
        };
        sas.process_esc_report(report);
        assert_eq!(sas.all_grants().get(&gid).unwrap().state, GrantState::Suspended);
    }

    #[test]
    fn test_esc_stale_report_ignored() {
        let mut net = EscSensorNetwork::new();
        let report = EscSensorReport {
            sensor_id: "ESC-1".to_string(),
            location: GeoPosition::new(34.0, -119.0, 0.0),
            state: EscState::IncumbentDetected,
            measured_power_dbm: None,
            timestamp_secs: 0,
            incumbent_channels: vec![0, 1, 2],
        };
        net.update_sensor(report);
        // Check at time 120 (stale after 60s)
        let flagged = net.incumbent_channels(120);
        assert!(flagged.is_empty());
    }

    // --- DPA protection ---

    #[test]
    fn test_dpa_exclusion_zone() {
        let dpa = DpaRecord {
            dpa_id: "DPA-NAVY-1".to_string(),
            center: GeoPosition::new(34.0, -118.0, 0.0),
            exclusion_radius_km: 50.0,
            protection_radius_km: 200.0,
            adj_channel_radius_km: 250.0,
            interference_limit_dbm: DPA_INTERFERENCE_LIMIT_DBM,
            protected_channels: (0..10).collect(),
            active: true,
        };
        // CBSD co-located with DPA center (well within exclusion zone)
        let cbsd_pos = GeoPosition::new(34.0, -118.0, 0.0);
        assert!(dpa.in_exclusion_zone(&cbsd_pos));
    }

    #[test]
    fn test_dpa_outside_protection_zone() {
        let dpa = DpaRecord {
            dpa_id: "DPA-NAVY-1".to_string(),
            center: GeoPosition::new(34.0, -118.0, 0.0),
            exclusion_radius_km: 50.0,
            protection_radius_km: 200.0,
            adj_channel_radius_km: 250.0,
            interference_limit_dbm: DPA_INTERFERENCE_LIMIT_DBM,
            protected_channels: (0..10).collect(),
            active: true,
        };
        // CBSD far away (300 km)
        let cbsd_pos = GeoPosition::new(31.3, -118.0, 0.0); // ~300 km south
        assert!(!dpa.in_exclusion_zone(&cbsd_pos));
        assert!(!dpa.in_protection_contour(&cbsd_pos));
    }

    #[test]
    fn test_dpa_protection_contour() {
        let mut mgr = DpaManager::new();
        mgr.add_dpa(DpaRecord {
            dpa_id: "DPA-1".to_string(),
            center: GeoPosition::new(34.0, -118.0, 0.0),
            exclusion_radius_km: 10.0,
            protection_radius_km: 100.0,
            adj_channel_radius_km: 120.0,
            interference_limit_dbm: -144.0,
            protected_channels: vec![0],
            active: true,
        });
        // Inside exclusion
        let lim = mgr.protection_contour_dbm("DPA-1", 5.0);
        assert_eq!(lim, Some(f64::NEG_INFINITY));
        // Outside protection zone
        let lim2 = mgr.protection_contour_dbm("DPA-1", 150.0);
        assert!(lim2.unwrap() > -144.0);
    }

    // --- PAL manager ---

    #[test]
    fn test_pal_license_registration() {
        let mut mgr = PalManager::new();
        mgr.add_license(PalLicense {
            pal_id: "PAL-001".to_string(),
            licensee_id: "LIC-001".to_string(),
            census_tract_fips: "06037000000".to_string(),
            channel_index: 2,
            expiry_time_secs: 1_000_000,
        });
        let channels = mgr.licensee_channels("LIC-001", "06037000000");
        assert_eq!(channels, vec![2]);
    }

    #[test]
    fn test_pal_channel_limit() {
        let mut mgr = PalManager::new();
        for i in 0..7 {
            mgr.add_license(PalLicense {
                pal_id: format!("PAL-{:03}", i),
                licensee_id: "LIC-001".to_string(),
                census_tract_fips: "06037000000".to_string(),
                channel_index: i,
                expiry_time_secs: 1_000_000,
            });
        }
        assert!(mgr.validate_channel_limit("LIC-001", "06037000000"));
        // Add one more to exceed limit
        mgr.add_license(PalLicense {
            pal_id: "PAL-007".to_string(),
            licensee_id: "LIC-001".to_string(),
            census_tract_fips: "06037000000".to_string(),
            channel_index: 7,
            expiry_time_secs: 1_000_000,
        });
        assert!(!mgr.validate_channel_limit("LIC-001", "06037000000"));
    }

    #[test]
    fn test_pal_holder_check() {
        let mut mgr = PalManager::new();
        mgr.add_license(PalLicense {
            pal_id: "PAL-001".to_string(),
            licensee_id: "LIC-001".to_string(),
            census_tract_fips: "06037000000".to_string(),
            channel_index: 3,
            expiry_time_secs: 1_000_000,
        });
        // CBSD with matching licensee_id
        let cbsd_pal = make_cbsd_a_pal("C1", 34.0, -118.0, "LIC-001");
        assert!(mgr.is_pal_holder(&cbsd_pal, 3));
        assert!(!mgr.is_pal_holder(&cbsd_pal, 4));
        // GAA CBSD (no licensee_id) should not be a PAL holder
        let cbsd_gaa = make_cbsd_a("C2", 34.0, -118.0);
        assert!(!mgr.is_pal_holder(&cbsd_gaa, 3));
    }

    // --- Channel manager ---

    #[test]
    fn test_channel_manager_initial_state() {
        let mgr = ChannelManager::new();
        assert_eq!(mgr.all_channels().len(), NUM_CHANNELS);
        let pal = mgr.available_pal_channels();
        assert_eq!(pal.len(), NUM_PAL_CHANNELS);
        let gaa = mgr.available_gaa_channels();
        assert_eq!(gaa.len(), NUM_CHANNELS); // All channels available initially
    }

    #[test]
    fn test_channel_manager_blocking() {
        let mut mgr = ChannelManager::new();
        mgr.update_blocked_channels(vec![0, 1, 2]);
        let pal = mgr.available_pal_channels();
        assert_eq!(pal.len(), NUM_PAL_CHANNELS - 3);
    }

    #[test]
    fn test_channel_manager_grant_counting() {
        let mut mgr = ChannelManager::new();
        mgr.increment_grants(5);
        mgr.increment_grants(5);
        assert_eq!(mgr.channel(5).unwrap().active_grants, 2);
        mgr.decrement_grants(5);
        assert_eq!(mgr.channel(5).unwrap().active_grants, 1);
    }

    // --- Power controller ---

    #[test]
    fn test_power_controller_validate_eirp_cat_a() {
        let cbsd = make_cbsd_a("C1", 34.0, -118.0);
        assert!(PowerController::validate_eirp(&cbsd, 30.0));
        assert!(!PowerController::validate_eirp(&cbsd, 30.1));
    }

    #[test]
    fn test_power_controller_validate_eirp_cat_b() {
        let cbsd = make_cbsd_b("B1", 34.0, -118.0);
        assert!(PowerController::validate_eirp(&cbsd, 47.0));
        assert!(!PowerController::validate_eirp(&cbsd, 47.1));
    }

    #[test]
    fn test_power_controller_max_eirp_dpa_constraint() {
        let cbsd = make_cbsd_b("B1", 34.0, -118.0);
        // DPA limits to 40 dBm
        let max = PowerController::max_eirp(&cbsd, Some(40.0));
        assert_eq!(max, 40.0);
        // No DPA constraint → category limit
        let max2 = PowerController::max_eirp(&cbsd, None);
        assert_eq!(max2, 47.0);
    }

    #[test]
    fn test_power_controller_conducted_power() {
        let p = PowerController::max_conducted_power(30.0, 6.0);
        assert!((p - 24.0).abs() < 1e-9);
    }

    #[test]
    fn test_power_controller_reduce_eirp() {
        let reduced = PowerController::reduce_eirp(30.0, 3.0);
        assert!((reduced - 27.0).abs() < 1e-9);
    }

    // --- GAA coexistence ---

    #[test]
    fn test_gaa_slot_assignment() {
        let mut mgr = GaaCoexistenceManager::new(4);
        let s1 = mgr.assign_slot("C1");
        let s2 = mgr.assign_slot("C2");
        assert_ne!(s1, s2);
    }

    #[test]
    fn test_gaa_can_transmit() {
        let mut mgr = GaaCoexistenceManager::new(4);
        let slot = mgr.assign_slot("C1");
        // Set current slot to match
        for _ in 0..slot {
            mgr.advance_slot();
        }
        assert!(mgr.can_transmit("C1"));
        mgr.advance_slot();
        // After advancing, C1's slot is no longer current
        assert!(!mgr.can_transmit("C1"));
    }

    #[test]
    fn test_gaa_remove_cbsd() {
        let mut mgr = GaaCoexistenceManager::new(4);
        mgr.assign_slot("C1");
        mgr.remove_cbsd("C1");
        assert!(!mgr.can_transmit("C1"));
    }

    // --- SAS processor integration ---

    #[test]
    fn test_sas_grant_count() {
        let mut sas = SasProcessor::new();
        sas.register_cbsd(make_cbsd_a("C1", 34.0, -118.0)).unwrap();
        sas.register_cbsd(make_cbsd_a("C2", 34.1, -118.0)).unwrap();

        sas.request_grant(make_grant_req("C1", 0, 25.0));
        sas.request_grant(make_grant_req("C2", 1, 25.0));
        assert_eq!(sas.active_grant_count(), 2);
    }

    #[test]
    fn test_sas_deregister_terminates_grants() {
        let mut sas = SasProcessor::new();
        sas.register_cbsd(make_cbsd_a("C1", 34.0, -118.0)).unwrap();
        sas.request_grant(make_grant_req("C1", 0, 25.0));
        sas.request_grant(make_grant_req("C1", 1, 25.0));
        assert_eq!(sas.active_grant_count(), 2);

        sas.deregister_cbsd("C1");
        assert_eq!(sas.active_grant_count(), 0);
    }

    #[test]
    fn test_sas_time_advancement_expires_grants() {
        let mut sas = SasProcessor::new();
        sas.register_cbsd(make_cbsd_a("C1", 34.0, -118.0)).unwrap();
        sas.request_grant(make_grant_req("C1", 0, 25.0));
        assert_eq!(sas.active_grant_count(), 1);

        sas.advance_time(DEFAULT_GRANT_DURATION_SECS + 10);
        assert_eq!(sas.active_grant_count(), 0);
    }

    #[test]
    fn test_sas_cbsd_grants_query() {
        let mut sas = SasProcessor::new();
        sas.register_cbsd(make_cbsd_a("C1", 34.0, -118.0)).unwrap();
        sas.request_grant(make_grant_req("C1", 0, 25.0));
        sas.request_grant(make_grant_req("C1", 1, 25.0));
        let grants = sas.cbsd_grants("C1");
        assert_eq!(grants.len(), 2);
    }

    #[test]
    fn test_sas_pal_vs_gaa_tier() {
        let mut sas = SasProcessor::new();
        sas.add_pal_license(PalLicense {
            pal_id: "PAL-001".to_string(),
            licensee_id: "LIC-001".to_string(),
            census_tract_fips: "06037000000".to_string(),
            channel_index: 0,
            expiry_time_secs: 1_000_000,
        });
        // C1 is a PAL licensee device
        sas.register_cbsd(make_cbsd_a_pal("C1", 34.0, -118.0, "LIC-001")).unwrap();
        let resp = sas.request_grant(make_grant_req("C1", 0, 25.0));
        let grant = resp.grant.unwrap();
        assert_eq!(grant.tier, AccessTier::PriorityAccess);

        // C2 has no licensee_id → GAA on PAL channel (allowed while PAL only in Granted state)
        sas.register_cbsd(make_cbsd_a("C2", 34.01, -118.0)).unwrap();
        let resp2 = sas.request_grant(make_grant_req("C2", 0, 25.0));
        assert_eq!(resp2.response_code, SasResponseCode::Success);
        let g2 = resp2.grant.unwrap();
        assert_eq!(g2.tier, AccessTier::GeneralAuthorizedAccess);

        // Authorize PAL grant
        let gid1 = grant.grant_id.clone();
        sas.authorize_grant(&gid1);

        // C3 (GAA) now tries to get a grant on channel 0 – PAL is Authorized → deny
        sas.register_cbsd(make_cbsd_a("C3", 34.02, -118.0)).unwrap();
        let resp3 = sas.request_grant(make_grant_req("C3", 0, 25.0));
        assert_eq!(resp3.response_code, SasResponseCode::InterferenceConstraint);
    }

    #[test]
    fn test_sas_channel_summary_count() {
        let sas = SasProcessor::new();
        assert_eq!(sas.channel_summary().len(), NUM_CHANNELS);
    }

    #[test]
    fn test_grant_center_freq() {
        let grant = Grant {
            grant_id: "G1".to_string(),
            cbsd_id: "C1".to_string(),
            channel_index: 0,
            max_eirp_dbm: 25.0,
            state: GrantState::Granted,
            issue_time_secs: 0,
            expiry_time_secs: 300,
            last_heartbeat_secs: 0,
            heartbeat_interval_secs: 60,
            tier: AccessTier::GeneralAuthorizedAccess,
        };
        assert!((grant.center_freq_mhz() - 3555.0).abs() < 1e-9);
        assert!((grant.low_freq_mhz() - 3550.0).abs() < 1e-9);
        assert!((grant.high_freq_mhz() - 3560.0).abs() < 1e-9);
    }

    #[test]
    fn test_grant_expiry() {
        let grant = Grant {
            grant_id: "G1".to_string(),
            cbsd_id: "C1".to_string(),
            channel_index: 0,
            max_eirp_dbm: 25.0,
            state: GrantState::Granted,
            issue_time_secs: 0,
            expiry_time_secs: 300,
            last_heartbeat_secs: 0,
            heartbeat_interval_secs: 60,
            tier: AccessTier::GeneralAuthorizedAccess,
        };
        assert!(!grant.is_expired(299));
        assert!(grant.is_expired(300));
        assert!(grant.is_expired(400));
    }

    #[test]
    fn test_grant_heartbeat_overdue() {
        let grant = Grant {
            grant_id: "G1".to_string(),
            cbsd_id: "C1".to_string(),
            channel_index: 0,
            max_eirp_dbm: 25.0,
            state: GrantState::Authorized,
            issue_time_secs: 0,
            expiry_time_secs: 3000,
            last_heartbeat_secs: 0,
            heartbeat_interval_secs: 60,
            tier: AccessTier::GeneralAuthorizedAccess,
        };
        assert!(!grant.heartbeat_overdue(60));
        assert!(grant.heartbeat_overdue(61));
    }

    #[test]
    fn test_esc_move_list() {
        let mut net = EscSensorNetwork::new();
        let mut grants: HashMap<String, Grant> = HashMap::new();

        let g = Grant {
            grant_id: "G1".to_string(),
            cbsd_id: "C1".to_string(),
            channel_index: 2,
            max_eirp_dbm: 25.0,
            state: GrantState::Authorized,
            issue_time_secs: 0,
            expiry_time_secs: 3000,
            last_heartbeat_secs: 0,
            heartbeat_interval_secs: 60,
            tier: AccessTier::GeneralAuthorizedAccess,
        };
        grants.insert("G1".to_string(), g);

        let report = EscSensorReport {
            sensor_id: "ESC-1".to_string(),
            location: GeoPosition::new(34.0, -119.0, 0.0),
            state: EscState::IncumbentDetected,
            measured_power_dbm: None,
            timestamp_secs: 0,
            incumbent_channels: vec![2],
        };
        net.update_sensor(report);

        let move_list = net.generate_move_list(&grants, 0);
        assert_eq!(move_list.len(), 1);
        assert_eq!(move_list[0].grant_id, "G1");
    }
}

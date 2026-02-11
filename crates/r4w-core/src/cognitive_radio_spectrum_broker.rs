//! Cognitive Radio Spectrum Broker -- Dynamic Spectrum Access (DSA)
//!
//! This module implements a spectrum broker that manages dynamic spectrum access
//! for cognitive radio systems.  It performs energy-based spectrum sensing,
//! occupancy tracking, channel allocation with priority scheduling, and
//! interference-aware power control for secondary users.
//!
//! ## Key Concepts
//!
//! - **Energy Detection**: Neyman-Pearson threshold for deciding primary user
//!   presence from observed energy.
//! - **Cooperative Sensing**: Fuse local hard decisions (OR / AND / Majority)
//!   from multiple sensing nodes to improve reliability.
//! - **Interference Temperature**: Converts a received power level to an
//!   equivalent temperature `T = P / (k * B)` to compare against regulatory
//!   limits.
//! - **Spectrum Efficiency**: Fraction of channels currently allocated to
//!   secondary users.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cognitive_radio_spectrum_broker::{
//!     SpectrumBrokerConfig, SpectrumBroker,
//! };
//!
//! let config = SpectrumBrokerConfig {
//!     freq_start_mhz: 470.0,
//!     freq_end_mhz: 480.0,
//!     channel_bw_khz: 200.0,
//!     sensing_period_ms: 10.0,
//!     pfa_target: 0.01,
//!     pd_target: 0.9,
//! };
//! let mut broker = SpectrumBroker::new(config);
//!
//! // Report idle energy on channel 0
//! broker.update_sensing(0, -100.0);
//!
//! // Secondary user requests a channel
//! let alloc = broker.request_channel(1, 200.0, 5);
//! assert!(alloc.is_some());
//! ```

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Boltzmann constant (J/K)
pub const BOLTZMANN_K: f64 = 1.380649e-23;

/// Default noise floor assumed for energy detection (dBm).
const DEFAULT_NOISE_FLOOR_DB: f64 = -100.0;

/// Default number of sensing samples used when computing thresholds.
const DEFAULT_NUM_SAMPLES: usize = 1024;

/// Maximum secondary-user transmit power (dBm) assigned to allocations.
const DEFAULT_MAX_TX_POWER_DBM: f64 = 20.0;

/// Default allocation lifetime in arbitrary epoch ticks.
const DEFAULT_ALLOCATION_LIFETIME: u64 = 1000;

/// Energy hysteresis (dB) -- a channel must drop this far below threshold
/// before it transitions back to Idle from OccupiedPrimary.
const HYSTERESIS_DB: f64 = 3.0;

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// State of a single channel as observed by the broker.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChannelState {
    /// No primary or secondary user detected.
    Idle,
    /// A primary (licensed) user occupies the channel.
    OccupiedPrimary,
    /// A secondary user with the given ID occupies the channel.
    OccupiedSecondary(u32),
    /// The channel is currently being sensed (transitional).
    Sensing,
    /// The channel has been permanently removed from consideration.
    Blacklisted,
}

/// Fusion rule for cooperative spectrum sensing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FusionRule {
    /// Decide "occupied" if *any* local sensor decides occupied.
    Or,
    /// Decide "occupied" only if *all* local sensors decide occupied.
    And,
    /// Decide "occupied" if more than half decide occupied.
    Majority,
}

// ---------------------------------------------------------------------------
// Small value types
// ---------------------------------------------------------------------------

/// Status snapshot of a single channel.
#[derive(Debug, Clone)]
pub struct ChannelStatus {
    pub channel_idx: usize,
    pub center_freq_mhz: f64,
    pub state: ChannelState,
    /// Fraction of sensing updates where the channel was detected as occupied.
    pub occupancy_ratio: f64,
    /// Most recent energy measurement (dB).
    pub last_sensed_energy_db: f64,
}

/// A granted spectrum allocation for a secondary user.
#[derive(Debug, Clone)]
pub struct ChannelAllocation {
    pub channel_idx: usize,
    pub center_freq_mhz: f64,
    pub bandwidth_khz: f64,
    pub max_power_dbm: f64,
    /// Epoch tick at which this allocation expires.
    pub expiry_epoch: u64,
}

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

/// Configuration for the spectrum broker.
#[derive(Debug, Clone)]
pub struct SpectrumBrokerConfig {
    /// Lower bound of the managed band (MHz).
    pub freq_start_mhz: f64,
    /// Upper bound of the managed band (MHz).
    pub freq_end_mhz: f64,
    /// Bandwidth of each discrete channel (kHz).
    pub channel_bw_khz: f64,
    /// Sensing cycle period (ms) -- informational.
    pub sensing_period_ms: f64,
    /// Target probability of false alarm (Pfa).
    pub pfa_target: f64,
    /// Target probability of detection (Pd).
    pub pd_target: f64,
}

impl Default for SpectrumBrokerConfig {
    fn default() -> Self {
        Self {
            freq_start_mhz: 470.0,
            freq_end_mhz: 480.0,
            channel_bw_khz: 200.0,
            sensing_period_ms: 10.0,
            pfa_target: 0.01,
            pd_target: 0.9,
        }
    }
}

// ---------------------------------------------------------------------------
// Internal per-channel bookkeeping
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct ChannelRecord {
    state: ChannelState,
    center_freq_mhz: f64,
    /// Running count of sensing updates.
    sense_count: u64,
    /// Running count of updates where energy exceeded threshold.
    occupied_count: u64,
    /// Last reported energy (dB).
    last_energy_db: f64,
    /// User ID if allocated to a secondary user.
    allocated_to: Option<u32>,
    /// Priority of the allocation (higher = more important).
    allocation_priority: u8,
}

// ---------------------------------------------------------------------------
// SpectrumBroker
// ---------------------------------------------------------------------------

/// Central spectrum broker managing a contiguous band divided into
/// equal-width channels.
#[derive(Debug)]
pub struct SpectrumBroker {
    config: SpectrumBrokerConfig,
    channels: Vec<ChannelRecord>,
    /// Energy detection threshold (dB) computed at construction time.
    threshold_db: f64,
    /// Monotonically increasing epoch counter, bumped on each sensing update.
    epoch: u64,
}

impl SpectrumBroker {
    /// Create a new broker.  The managed band is divided into
    /// `floor((freq_end - freq_start) * 1000 / channel_bw_khz)` channels.
    pub fn new(config: SpectrumBrokerConfig) -> Self {
        let band_khz = (config.freq_end_mhz - config.freq_start_mhz) * 1000.0;
        let num_channels = (band_khz / config.channel_bw_khz).floor() as usize;
        assert!(num_channels > 0, "Band too narrow for the channel bandwidth");

        let threshold_db = energy_detector_threshold(
            DEFAULT_NOISE_FLOOR_DB,
            config.pfa_target,
            DEFAULT_NUM_SAMPLES,
        );

        let channels: Vec<ChannelRecord> = (0..num_channels)
            .map(|i| {
                let center = config.freq_start_mhz
                    + (i as f64 + 0.5) * config.channel_bw_khz / 1000.0;
                ChannelRecord {
                    state: ChannelState::Idle,
                    center_freq_mhz: center,
                    sense_count: 0,
                    occupied_count: 0,
                    last_energy_db: DEFAULT_NOISE_FLOOR_DB,
                    allocated_to: None,
                    allocation_priority: 0,
                }
            })
            .collect();

        Self {
            config,
            channels,
            threshold_db,
            epoch: 0,
        }
    }

    /// Return the number of channels managed by the broker.
    pub fn num_channels(&self) -> usize {
        self.channels.len()
    }

    /// Return the current epoch counter.
    pub fn epoch(&self) -> u64 {
        self.epoch
    }

    /// Feed a spectrum sensing result for `channel_idx`.
    ///
    /// If the measured `energy_db` exceeds the detection threshold the channel
    /// is marked `OccupiedPrimary` (any secondary allocation is evicted).
    /// Once the energy drops below `threshold - HYSTERESIS_DB` the channel
    /// returns to `Idle`.
    pub fn update_sensing(&mut self, channel_idx: usize, energy_db: f64) {
        if channel_idx >= self.channels.len() {
            return;
        }
        self.epoch += 1;
        let ch = &mut self.channels[channel_idx];

        // Blacklisted channels are never updated.
        if ch.state == ChannelState::Blacklisted {
            return;
        }

        ch.last_energy_db = energy_db;
        ch.sense_count += 1;

        if energy_db > self.threshold_db {
            ch.occupied_count += 1;
            // Primary user detected -- evict secondary if present.
            if let ChannelState::OccupiedSecondary(_) = ch.state {
                ch.allocated_to = None;
                ch.allocation_priority = 0;
            }
            ch.state = ChannelState::OccupiedPrimary;
        } else if ch.state == ChannelState::OccupiedPrimary
            && energy_db < self.threshold_db - HYSTERESIS_DB
        {
            ch.state = ChannelState::Idle;
        }
        // If the channel is OccupiedSecondary and energy is below threshold,
        // it stays allocated.
    }

    /// Blacklist a channel so it is never allocated.
    pub fn blacklist_channel(&mut self, channel_idx: usize) {
        if let Some(ch) = self.channels.get_mut(channel_idx) {
            ch.state = ChannelState::Blacklisted;
            ch.allocated_to = None;
            ch.allocation_priority = 0;
        }
    }

    /// Request a channel for the secondary user `user_id`.
    ///
    /// The broker searches for the first `Idle` channel whose bandwidth
    /// matches the request.  Higher `priority` values can preempt lower ones.
    /// Returns `None` if no suitable channel is available.
    pub fn request_channel(
        &mut self,
        user_id: u32,
        bandwidth_khz: f64,
        priority: u8,
    ) -> Option<ChannelAllocation> {
        // How many contiguous channels do we need?
        let channels_needed =
            (bandwidth_khz / self.config.channel_bw_khz).ceil() as usize;
        if channels_needed == 0 {
            return None;
        }

        // First pass: look for idle contiguous block.
        let states: Vec<ChannelState> =
            self.channels.iter().map(|c| c.state).collect();
        if let Some(start) = find_contiguous_channels(&states, channels_needed) {
            return self.allocate_block(start, channels_needed, user_id, priority);
        }

        // Second pass: preempt lower-priority secondary users.
        if channels_needed == 1 {
            // Find lowest-priority secondary allocation.
            let mut best: Option<(usize, u8)> = None;
            for (i, ch) in self.channels.iter().enumerate() {
                if let ChannelState::OccupiedSecondary(_) = ch.state {
                    if ch.allocation_priority < priority {
                        match best {
                            None => best = Some((i, ch.allocation_priority)),
                            Some((_, bp)) if ch.allocation_priority < bp => {
                                best = Some((i, ch.allocation_priority));
                            }
                            _ => {}
                        }
                    }
                }
            }
            if let Some((idx, _)) = best {
                // Evict existing secondary user.
                self.channels[idx].state = ChannelState::Idle;
                self.channels[idx].allocated_to = None;
                self.channels[idx].allocation_priority = 0;
                return self.allocate_block(idx, 1, user_id, priority);
            }
        }

        None
    }

    /// Release any channel(s) held by `user_id`.
    pub fn release_channel(&mut self, user_id: u32) {
        for ch in &mut self.channels {
            if ch.allocated_to == Some(user_id) {
                ch.state = ChannelState::Idle;
                ch.allocated_to = None;
                ch.allocation_priority = 0;
            }
        }
    }

    /// Produce a snapshot of all channels.
    pub fn get_occupancy_map(&self) -> Vec<ChannelStatus> {
        self.channels
            .iter()
            .enumerate()
            .map(|(i, ch)| {
                let occupancy_ratio = if ch.sense_count == 0 {
                    0.0
                } else {
                    ch.occupied_count as f64 / ch.sense_count as f64
                };
                ChannelStatus {
                    channel_idx: i,
                    center_freq_mhz: ch.center_freq_mhz,
                    state: ch.state,
                    occupancy_ratio,
                    last_sensed_energy_db: ch.last_energy_db,
                }
            })
            .collect()
    }

    // -- helpers --

    fn allocate_block(
        &mut self,
        start: usize,
        count: usize,
        user_id: u32,
        priority: u8,
    ) -> Option<ChannelAllocation> {
        for i in start..start + count {
            self.channels[i].state = ChannelState::OccupiedSecondary(user_id);
            self.channels[i].allocated_to = Some(user_id);
            self.channels[i].allocation_priority = priority;
        }
        let center = if count == 1 {
            self.channels[start].center_freq_mhz
        } else {
            let lo = self.channels[start].center_freq_mhz
                - self.config.channel_bw_khz / 2000.0;
            let hi = self.channels[start + count - 1].center_freq_mhz
                + self.config.channel_bw_khz / 2000.0;
            (lo + hi) / 2.0
        };
        Some(ChannelAllocation {
            channel_idx: start,
            center_freq_mhz: center,
            bandwidth_khz: count as f64 * self.config.channel_bw_khz,
            max_power_dbm: DEFAULT_MAX_TX_POWER_DBM,
            expiry_epoch: self.epoch + DEFAULT_ALLOCATION_LIFETIME,
        })
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Neyman-Pearson energy detector threshold.
///
/// For an energy detector with `num_samples` observations and a noise-only
/// hypothesis at `noise_power_db`, the threshold is:
///
/// ```text
/// lambda = noise_power_db + 10 * log10(1 + Q_inv(pfa) * sqrt(2 / N))
/// ```
///
/// where `Q_inv(pfa)` is the inverse complementary Gaussian CDF.  We use a
/// rational approximation to the inverse normal here.
pub fn energy_detector_threshold(
    noise_power_db: f64,
    pfa: f64,
    num_samples: usize,
) -> f64 {
    let z = inv_q(pfa); // Q^{-1}(Pfa)
    let n = num_samples as f64;
    // Central-limit approximation to the chi-squared test statistic:
    // threshold factor above noise = z * sqrt(2/N)
    let factor = z * (2.0 / n).sqrt();
    noise_power_db + 10.0 * (1.0 + factor).log10()
}

/// Probability of detection for a given SNR and threshold factor.
///
/// Uses the Gaussian approximation for the non-central chi-squared
/// distribution:
///
/// ```text
/// Pd = Q( (threshold_factor - (1 + gamma)) / ((1 + gamma) * sqrt(2/N)) )
/// ```
///
/// where `gamma` is the linear SNR.
pub fn detection_probability(
    snr_db: f64,
    threshold_factor: f64,
    num_samples: usize,
) -> f64 {
    let gamma = db_to_linear(snr_db); // linear SNR
    let n = num_samples as f64;
    let mean = 1.0 + gamma;
    let std_dev = mean * (2.0 / n).sqrt();
    if std_dev <= 0.0 {
        return if threshold_factor <= mean { 1.0 } else { 0.0 };
    }
    let z = (threshold_factor - mean) / std_dev;
    q_function(z)
}

/// Cooperative sensing fusion.
///
/// Combines a set of local binary decisions (true = "primary detected") into
/// a single global decision according to the given `FusionRule`.
pub fn cooperative_sensing_decision(
    local_decisions: &[bool],
    rule: FusionRule,
) -> bool {
    if local_decisions.is_empty() {
        return false;
    }
    match rule {
        FusionRule::Or => local_decisions.iter().any(|&d| d),
        FusionRule::And => local_decisions.iter().all(|&d| d),
        FusionRule::Majority => {
            let yes = local_decisions.iter().filter(|&&d| d).count();
            yes * 2 > local_decisions.len()
        }
    }
}

/// Spectrum utilization ratio: `allocated / total`.
///
/// Returns 0.0 when `total == 0`.
pub fn spectrum_efficiency(allocated: usize, total: usize) -> f64 {
    if total == 0 {
        return 0.0;
    }
    allocated as f64 / total as f64
}

/// Interference temperature in Kelvin.
///
/// ```text
/// T = P / (k * B)
/// ```
///
/// where `P` is power in Watts (converted from dBm), `k` is Boltzmann's
/// constant, and `B` is bandwidth in Hz.
pub fn interference_temperature(power_dbm: f64, bandwidth_hz: f64) -> f64 {
    let power_w = dbm_to_watts(power_dbm);
    power_w / (BOLTZMANN_K * bandwidth_hz)
}

/// Find the first contiguous run of at least `required` `Idle` channels.
///
/// Returns the starting index, or `None` if no such run exists.
pub fn find_contiguous_channels(
    occupancy: &[ChannelState],
    required: usize,
) -> Option<usize> {
    if required == 0 || occupancy.is_empty() {
        return None;
    }
    let mut run_start = 0;
    let mut run_len = 0;
    for (i, state) in occupancy.iter().enumerate() {
        if *state == ChannelState::Idle {
            if run_len == 0 {
                run_start = i;
            }
            run_len += 1;
            if run_len >= required {
                return Some(run_start);
            }
        } else {
            run_len = 0;
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Internal math helpers
// ---------------------------------------------------------------------------

/// Convert dB to linear power ratio.
fn db_to_linear(db: f64) -> f64 {
    10.0_f64.powf(db / 10.0)
}

/// Convert dBm to Watts.
fn dbm_to_watts(dbm: f64) -> f64 {
    10.0_f64.powf((dbm - 30.0) / 10.0)
}

/// Gaussian Q-function: Q(x) = 0.5 * erfc(x / sqrt(2)).
///
/// Uses a rational approximation to erfc valid for x >= 0.
fn q_function(x: f64) -> f64 {
    0.5 * erfc(x / core::f64::consts::SQRT_2)
}

/// Inverse Q-function: given p = Q(x), return x.
///
/// Uses the rational approximation to the inverse normal (Abramowitz & Stegun
/// 26.2.23).
fn inv_q(p: f64) -> f64 {
    // Q(x) = p  =>  x = Phi^{-1}(1 - p)
    inv_normal_cdf(1.0 - p)
}

/// Complementary error function: erfc(x) = 1 - erf(x).
///
/// Uses Horner-form rational approximation (Abramowitz & Stegun 7.1.26)
/// for x >= 0.  For x < 0, uses erfc(-x) = 2 - erfc(x).
fn erfc(x: f64) -> f64 {
    if x < 0.0 {
        return 2.0 - erfc(-x);
    }
    // Coefficients from A&S 7.1.26 (maximum error ~ 1.5e-7)
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t
        * (0.254829592
            + t * (-0.284496736
                + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    poly * (-x * x).exp()
}

/// Inverse of the standard normal CDF (probit function).
///
/// Rational approximation (Abramowitz & Stegun 26.2.23).
fn inv_normal_cdf(p: f64) -> f64 {
    if p <= 0.0 {
        return f64::NEG_INFINITY;
    }
    if p >= 1.0 {
        return f64::INFINITY;
    }
    if p == 0.5 {
        return 0.0;
    }
    // For p > 0.5 use symmetry.
    if p > 0.5 {
        return -inv_normal_cdf(1.0 - p);
    }
    // A&S 26.2.23 constants
    let t = (-2.0 * p.ln()).sqrt();
    let c0 = 2.515517;
    let c1 = 0.802853;
    let c2 = 0.010328;
    let d1 = 1.432788;
    let d2 = 0.189269;
    let d3 = 0.001308;
    let num = c0 + t * (c1 + t * c2);
    let den = 1.0 + t * (d1 + t * (d2 + t * d3));
    -(t - num / den)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> SpectrumBrokerConfig {
        SpectrumBrokerConfig {
            freq_start_mhz: 470.0,
            freq_end_mhz: 480.0,
            channel_bw_khz: 200.0,
            sensing_period_ms: 10.0,
            pfa_target: 0.01,
            pd_target: 0.9,
        }
    }

    // -- Broker construction ------------------------------------------------

    #[test]
    fn test_broker_channel_count() {
        let broker = SpectrumBroker::new(default_config());
        // (480 - 470) MHz = 10 MHz = 10_000 kHz / 200 kHz = 50 channels
        assert_eq!(broker.num_channels(), 50);
    }

    #[test]
    fn test_broker_initial_state_all_idle() {
        let broker = SpectrumBroker::new(default_config());
        for cs in broker.get_occupancy_map() {
            assert_eq!(cs.state, ChannelState::Idle);
            assert_eq!(cs.occupancy_ratio, 0.0);
        }
    }

    #[test]
    fn test_channel_center_frequencies() {
        let broker = SpectrumBroker::new(default_config());
        let map = broker.get_occupancy_map();
        // First channel: 470.0 + 0.5 * 0.2 = 470.1 MHz
        assert!((map[0].center_freq_mhz - 470.1).abs() < 1e-9);
        // Last channel: 470.0 + 49.5 * 0.2 = 479.9 MHz
        assert!((map[49].center_freq_mhz - 479.9).abs() < 1e-9);
    }

    // -- Sensing updates ----------------------------------------------------

    #[test]
    fn test_sensing_low_energy_stays_idle() {
        let mut broker = SpectrumBroker::new(default_config());
        broker.update_sensing(0, -110.0);
        assert_eq!(broker.get_occupancy_map()[0].state, ChannelState::Idle);
    }

    #[test]
    fn test_sensing_high_energy_marks_occupied() {
        let mut broker = SpectrumBroker::new(default_config());
        // Very high energy, well above any threshold.
        broker.update_sensing(5, -50.0);
        assert_eq!(
            broker.get_occupancy_map()[5].state,
            ChannelState::OccupiedPrimary
        );
    }

    #[test]
    fn test_sensing_hysteresis() {
        let mut broker = SpectrumBroker::new(default_config());
        let thresh = broker.threshold_db;
        // Push above threshold.
        broker.update_sensing(0, thresh + 5.0);
        assert_eq!(
            broker.get_occupancy_map()[0].state,
            ChannelState::OccupiedPrimary
        );
        // Just below threshold but within hysteresis -- should stay occupied.
        broker.update_sensing(0, thresh - 1.0);
        assert_eq!(
            broker.get_occupancy_map()[0].state,
            ChannelState::OccupiedPrimary
        );
        // Well below threshold - hysteresis.
        broker.update_sensing(0, thresh - HYSTERESIS_DB - 1.0);
        assert_eq!(broker.get_occupancy_map()[0].state, ChannelState::Idle);
    }

    #[test]
    fn test_sensing_occupancy_ratio() {
        let mut broker = SpectrumBroker::new(default_config());
        // 2 of 4 sensing updates are above threshold.
        broker.update_sensing(0, -110.0);
        broker.update_sensing(0, -50.0);
        broker.update_sensing(0, -110.0);
        broker.update_sensing(0, -50.0);
        let ratio = broker.get_occupancy_map()[0].occupancy_ratio;
        assert!((ratio - 0.5).abs() < 1e-9);
    }

    #[test]
    fn test_sensing_out_of_range_ignored() {
        let mut broker = SpectrumBroker::new(default_config());
        broker.update_sensing(999, -50.0);
        // Should not panic and epoch still advances only for valid updates.
        assert_eq!(broker.epoch(), 0);
    }

    #[test]
    fn test_sensing_epoch_increments() {
        let mut broker = SpectrumBroker::new(default_config());
        broker.update_sensing(0, -110.0);
        broker.update_sensing(1, -110.0);
        assert_eq!(broker.epoch(), 2);
    }

    // -- Blacklisting -------------------------------------------------------

    #[test]
    fn test_blacklist_channel() {
        let mut broker = SpectrumBroker::new(default_config());
        broker.blacklist_channel(3);
        assert_eq!(
            broker.get_occupancy_map()[3].state,
            ChannelState::Blacklisted
        );
        // Sensing updates should be ignored on blacklisted channels.
        broker.update_sensing(3, -50.0);
        assert_eq!(
            broker.get_occupancy_map()[3].state,
            ChannelState::Blacklisted
        );
    }

    // -- Channel allocation / release ---------------------------------------

    #[test]
    fn test_request_single_channel() {
        let mut broker = SpectrumBroker::new(default_config());
        let alloc = broker.request_channel(1, 200.0, 5).unwrap();
        assert_eq!(alloc.channel_idx, 0);
        assert!((alloc.bandwidth_khz - 200.0).abs() < 1e-9);
        assert_eq!(
            broker.get_occupancy_map()[0].state,
            ChannelState::OccupiedSecondary(1)
        );
    }

    #[test]
    fn test_request_multi_channel() {
        let mut broker = SpectrumBroker::new(default_config());
        // Request 400 kHz -> needs 2 contiguous channels.
        let alloc = broker.request_channel(2, 400.0, 5).unwrap();
        assert_eq!(alloc.channel_idx, 0);
        assert!((alloc.bandwidth_khz - 400.0).abs() < 1e-9);
        assert_eq!(
            broker.get_occupancy_map()[0].state,
            ChannelState::OccupiedSecondary(2)
        );
        assert_eq!(
            broker.get_occupancy_map()[1].state,
            ChannelState::OccupiedSecondary(2)
        );
    }

    #[test]
    fn test_release_channel() {
        let mut broker = SpectrumBroker::new(default_config());
        broker.request_channel(1, 200.0, 5);
        broker.release_channel(1);
        assert_eq!(broker.get_occupancy_map()[0].state, ChannelState::Idle);
    }

    #[test]
    fn test_release_nonexistent_user() {
        let mut broker = SpectrumBroker::new(default_config());
        // Should not panic.
        broker.release_channel(999);
    }

    #[test]
    fn test_allocation_respects_occupied_primary() {
        let mut broker = SpectrumBroker::new(default_config());
        // Mark first 5 channels as occupied.
        for i in 0..5 {
            broker.update_sensing(i, -50.0);
        }
        let alloc = broker.request_channel(1, 200.0, 5).unwrap();
        assert!(alloc.channel_idx >= 5);
    }

    #[test]
    fn test_no_channel_available() {
        let mut broker = SpectrumBroker::new(default_config());
        // Occupy all channels with primary users.
        for i in 0..50 {
            broker.update_sensing(i, -50.0);
        }
        assert!(broker.request_channel(1, 200.0, 5).is_none());
    }

    #[test]
    fn test_primary_evicts_secondary() {
        let mut broker = SpectrumBroker::new(default_config());
        broker.request_channel(1, 200.0, 5);
        assert_eq!(
            broker.get_occupancy_map()[0].state,
            ChannelState::OccupiedSecondary(1)
        );
        // Primary appears on the same channel.
        broker.update_sensing(0, -50.0);
        assert_eq!(
            broker.get_occupancy_map()[0].state,
            ChannelState::OccupiedPrimary
        );
    }

    #[test]
    fn test_priority_preemption() {
        let mut broker = SpectrumBroker::new(default_config());
        // Fill all channels with low-priority user.
        for i in 0..50 {
            // Mark them idle first, then allocate one by one.
            broker.channels[i].state = ChannelState::OccupiedSecondary(100);
            broker.channels[i].allocated_to = Some(100);
            broker.channels[i].allocation_priority = 1;
        }
        // High-priority user preempts.
        let alloc = broker.request_channel(200, 200.0, 10).unwrap();
        assert_eq!(
            broker.get_occupancy_map()[alloc.channel_idx].state,
            ChannelState::OccupiedSecondary(200)
        );
    }

    #[test]
    fn test_priority_cannot_preempt_higher() {
        let mut broker = SpectrumBroker::new(default_config());
        // Fill all with high-priority.
        for i in 0..50 {
            broker.channels[i].state = ChannelState::OccupiedSecondary(100);
            broker.channels[i].allocated_to = Some(100);
            broker.channels[i].allocation_priority = 10;
        }
        // Lower priority cannot preempt.
        assert!(broker.request_channel(200, 200.0, 5).is_none());
    }

    // -- Energy detector threshold ------------------------------------------

    #[test]
    fn test_threshold_increases_with_stricter_pfa() {
        let t1 = energy_detector_threshold(-100.0, 0.1, 1024);
        let t2 = energy_detector_threshold(-100.0, 0.01, 1024);
        let t3 = energy_detector_threshold(-100.0, 0.001, 1024);
        assert!(t1 < t2);
        assert!(t2 < t3);
    }

    #[test]
    fn test_threshold_decreases_with_more_samples() {
        let t1 = energy_detector_threshold(-100.0, 0.01, 64);
        let t2 = energy_detector_threshold(-100.0, 0.01, 1024);
        let t3 = energy_detector_threshold(-100.0, 0.01, 16384);
        assert!(t1 > t2);
        assert!(t2 > t3);
    }

    #[test]
    fn test_threshold_tracks_noise_floor() {
        let t1 = energy_detector_threshold(-100.0, 0.01, 1024);
        let t2 = energy_detector_threshold(-90.0, 0.01, 1024);
        // 10 dB increase in noise floor -> ~10 dB increase in threshold.
        assert!((t2 - t1 - 10.0).abs() < 0.5);
    }

    // -- Detection probability ----------------------------------------------

    #[test]
    fn test_pd_high_snr_close_to_one() {
        let pd = detection_probability(20.0, 1.1, 1024);
        assert!(pd > 0.99);
    }

    #[test]
    fn test_pd_low_snr_close_to_zero() {
        // Very high threshold and very low SNR.
        let pd = detection_probability(-20.0, 10.0, 1024);
        assert!(pd < 0.01);
    }

    #[test]
    fn test_pd_increases_with_snr() {
        let pd1 = detection_probability(0.0, 1.05, 256);
        let pd2 = detection_probability(10.0, 1.05, 256);
        assert!(pd2 > pd1);
    }

    // -- Cooperative sensing ------------------------------------------------

    #[test]
    fn test_fusion_or() {
        assert!(cooperative_sensing_decision(
            &[false, false, true],
            FusionRule::Or
        ));
        assert!(!cooperative_sensing_decision(
            &[false, false, false],
            FusionRule::Or
        ));
    }

    #[test]
    fn test_fusion_and() {
        assert!(cooperative_sensing_decision(
            &[true, true, true],
            FusionRule::And
        ));
        assert!(!cooperative_sensing_decision(
            &[true, true, false],
            FusionRule::And
        ));
    }

    #[test]
    fn test_fusion_majority() {
        assert!(cooperative_sensing_decision(
            &[true, true, false],
            FusionRule::Majority
        ));
        assert!(!cooperative_sensing_decision(
            &[true, false, false],
            FusionRule::Majority
        ));
    }

    #[test]
    fn test_fusion_empty() {
        assert!(!cooperative_sensing_decision(&[], FusionRule::Or));
        assert!(!cooperative_sensing_decision(&[], FusionRule::And));
        assert!(!cooperative_sensing_decision(&[], FusionRule::Majority));
    }

    #[test]
    fn test_fusion_single_element() {
        assert!(cooperative_sensing_decision(&[true], FusionRule::Majority));
        assert!(!cooperative_sensing_decision(
            &[false],
            FusionRule::Majority
        ));
    }

    // -- Spectrum efficiency ------------------------------------------------

    #[test]
    fn test_spectrum_efficiency_basic() {
        assert!((spectrum_efficiency(25, 50) - 0.5).abs() < 1e-9);
        assert!((spectrum_efficiency(0, 50) - 0.0).abs() < 1e-9);
        assert!((spectrum_efficiency(50, 50) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_spectrum_efficiency_zero_total() {
        assert!((spectrum_efficiency(0, 0) - 0.0).abs() < 1e-9);
    }

    // -- Interference temperature -------------------------------------------

    #[test]
    fn test_interference_temperature_positive() {
        let t = interference_temperature(-30.0, 1e6);
        assert!(t > 0.0);
    }

    #[test]
    fn test_interference_temperature_scales_with_power() {
        let t1 = interference_temperature(-30.0, 1e6);
        let t2 = interference_temperature(-20.0, 1e6);
        // 10 dB more power -> 10x higher temperature.
        assert!((t2 / t1 - 10.0).abs() < 0.1);
    }

    #[test]
    fn test_interference_temperature_known_value() {
        // 0 dBm = 1 mW = 0.001 W, BW = 1 MHz
        // T = 0.001 / (1.380649e-23 * 1e6) = ~72.4e12 K
        let t = interference_temperature(0.0, 1e6);
        let expected = 0.001 / (BOLTZMANN_K * 1e6);
        assert!((t - expected).abs() / expected < 1e-6);
    }

    // -- Contiguous channel finder ------------------------------------------

    #[test]
    fn test_find_contiguous_basic() {
        use ChannelState::*;
        let occ = vec![
            OccupiedPrimary,
            Idle,
            Idle,
            Idle,
            OccupiedPrimary,
        ];
        assert_eq!(find_contiguous_channels(&occ, 3), Some(1));
        assert_eq!(find_contiguous_channels(&occ, 4), None);
    }

    #[test]
    fn test_find_contiguous_at_start() {
        use ChannelState::*;
        let occ = vec![Idle, Idle, Idle, OccupiedPrimary];
        assert_eq!(find_contiguous_channels(&occ, 2), Some(0));
    }

    #[test]
    fn test_find_contiguous_at_end() {
        use ChannelState::*;
        let occ = vec![OccupiedPrimary, Idle, Idle, Idle];
        assert_eq!(find_contiguous_channels(&occ, 3), Some(1));
    }

    #[test]
    fn test_find_contiguous_single() {
        use ChannelState::*;
        let occ = vec![OccupiedPrimary, Idle, OccupiedPrimary];
        assert_eq!(find_contiguous_channels(&occ, 1), Some(1));
    }

    #[test]
    fn test_find_contiguous_none_available() {
        use ChannelState::*;
        let occ = vec![OccupiedPrimary; 5];
        assert_eq!(find_contiguous_channels(&occ, 1), None);
    }

    #[test]
    fn test_find_contiguous_zero_required() {
        let occ = vec![ChannelState::Idle; 5];
        assert_eq!(find_contiguous_channels(&occ, 0), None);
    }

    #[test]
    fn test_find_contiguous_empty_slice() {
        assert_eq!(find_contiguous_channels(&[], 1), None);
    }

    // -- Math helpers -------------------------------------------------------

    #[test]
    fn test_erfc_zero() {
        assert!((erfc(0.0) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_erfc_symmetry() {
        // erfc(-x) = 2 - erfc(x)
        let x = 1.5;
        assert!((erfc(-x) - (2.0 - erfc(x))).abs() < 1e-6);
    }

    #[test]
    fn test_q_function_half_at_zero() {
        assert!((q_function(0.0) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_inv_q_roundtrip() {
        // inv_q(Q(x)) ~ x for reasonable values.
        for &x in &[0.5, 1.0, 2.0, 2.5] {
            let p = q_function(x);
            let x_back = inv_q(p);
            assert!(
                (x_back - x).abs() < 0.01,
                "roundtrip failed: x={}, p={}, x_back={}",
                x,
                p,
                x_back
            );
        }
    }

    // -- Default config -----------------------------------------------------

    #[test]
    fn test_default_config() {
        let cfg = SpectrumBrokerConfig::default();
        assert!((cfg.pfa_target - 0.01).abs() < 1e-9);
        assert!((cfg.pd_target - 0.9).abs() < 1e-9);
    }

    // -- Allocation expiry field -------------------------------------------

    #[test]
    fn test_allocation_has_finite_expiry() {
        let mut broker = SpectrumBroker::new(default_config());
        let alloc = broker.request_channel(1, 200.0, 5).unwrap();
        assert!(alloc.expiry_epoch > 0);
        assert!(alloc.expiry_epoch > broker.epoch());
    }

    // -- Max power field ---------------------------------------------------

    #[test]
    fn test_allocation_max_power() {
        let mut broker = SpectrumBroker::new(default_config());
        let alloc = broker.request_channel(1, 200.0, 5).unwrap();
        assert!((alloc.max_power_dbm - DEFAULT_MAX_TX_POWER_DBM).abs() < 1e-9);
    }
}

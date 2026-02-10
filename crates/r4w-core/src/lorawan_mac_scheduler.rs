//! LoRaWAN MAC timing, duty cycle management, and Adaptive Data Rate (ADR).
//!
//! This module implements key LoRaWAN MAC-layer scheduling functions per the
//! LoRaWAN specification (TS001-1.0.4), including:
//!
//! - **Time-on-air** calculation from spreading factor, bandwidth, payload size,
//!   and coding rate.
//! - **Duty cycle tracking** per sub-band with configurable limits (1%, 0.1%, 10%).
//! - **RX window timing** (RECEIVE_DELAY1 = 1 s, RECEIVE_DELAY2 = 2 s).
//! - **Adaptive Data Rate (ADR)** — uses SNR margin to adjust SF and TX power.
//! - **Channel plans** for EU868 and US915 regions.
//! - **Data rate to SF/BW mapping**, maximum payload sizes, and TX power tables.
//! - **Airtime budget** tracking with next-available-slot computation.
//!
//! Complex samples are represented as `(f64, f64)` tuples `(re, im)`.
//!
//! # Example
//!
//! ```
//! use r4w_core::lorawan_mac_scheduler::{LorawanMacScheduler, Region};
//!
//! let mut sched = LorawanMacScheduler::new(Region::EU868);
//!
//! // Calculate time-on-air for SF7, 125 kHz, 20-byte payload, CR 4/5
//! let toa = sched.time_on_air(7, 125_000.0, 20, 1);
//! assert!(toa > 0.0);
//!
//! // Check if we can transmit on sub-band 0 at t = 0.0
//! assert!(sched.can_transmit(0, 0.0));
//!
//! // Record a transmission and verify duty-cycle back-off
//! sched.record_transmission(0, 0.0, toa);
//! let next = sched.next_available_slot(0, 0.0);
//! assert!(next > 0.0);
//! ```

// ---------------------------------------------------------------------------
// Region enum
// ---------------------------------------------------------------------------

/// Supported LoRaWAN regional parameter sets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Region {
    /// EU 863-870 MHz ISM band (duty-cycle limited).
    EU868,
    /// US 902-928 MHz ISM band (dwell-time limited).
    US915,
}

// ---------------------------------------------------------------------------
// Sub-band definition
// ---------------------------------------------------------------------------

/// A sub-band with a frequency range and a duty-cycle limit.
#[derive(Debug, Clone)]
pub struct SubBand {
    /// Lower frequency bound (Hz).
    pub freq_lo: f64,
    /// Upper frequency bound (Hz).
    pub freq_hi: f64,
    /// Duty-cycle limit as a fraction, e.g. 0.01 for 1%.
    pub duty_cycle_limit: f64,
    /// Cumulative airtime used in the current tracking window (seconds).
    pub airtime_used: f64,
    /// Timestamp of the last transmission end (seconds).
    pub last_tx_end: f64,
}

// ---------------------------------------------------------------------------
// Data-rate descriptor
// ---------------------------------------------------------------------------

/// Maps a LoRaWAN data-rate index to physical-layer parameters.
#[derive(Debug, Clone, Copy)]
pub struct DataRateEntry {
    /// Data rate index (0-15).
    pub dr: u8,
    /// Spreading factor.
    pub sf: u8,
    /// Bandwidth in Hz.
    pub bw: f64,
    /// Maximum MAC-layer payload size (bytes) without FOpts.
    pub max_payload: usize,
}

// ---------------------------------------------------------------------------
// TX power table entry
// ---------------------------------------------------------------------------

/// TX power level for a region.
#[derive(Debug, Clone, Copy)]
pub struct TxPowerEntry {
    /// Power index (0 = max).
    pub index: u8,
    /// EIRP in dBm.
    pub eirp_dbm: f64,
}

// ---------------------------------------------------------------------------
// Channel definition
// ---------------------------------------------------------------------------

/// An uplink channel.
#[derive(Debug, Clone, Copy)]
pub struct Channel {
    /// Centre frequency (Hz).
    pub freq: f64,
    /// Minimum data-rate index.
    pub dr_min: u8,
    /// Maximum data-rate index.
    pub dr_max: u8,
    /// Whether the channel is currently enabled.
    pub enabled: bool,
}

// ---------------------------------------------------------------------------
// ADR state
// ---------------------------------------------------------------------------

/// Internal state for the Adaptive Data Rate algorithm.
#[derive(Debug, Clone)]
pub struct AdrState {
    /// Ring buffer of recent uplink SNR observations (dB).
    pub snr_history: Vec<f64>,
    /// Maximum number of observations to keep.
    pub history_len: usize,
    /// Current data-rate index.
    pub current_dr: u8,
    /// Current TX power index (0 = max).
    pub current_tx_power: u8,
    /// Required SNR margin above the demodulation floor (dB).
    pub snr_margin: f64,
}

impl AdrState {
    fn new(initial_dr: u8, initial_tx_power: u8) -> Self {
        Self {
            snr_history: Vec::new(),
            history_len: 20,
            current_dr: initial_dr,
            current_tx_power: initial_tx_power,
            snr_margin: 10.0,
        }
    }
}

// ---------------------------------------------------------------------------
// RX window timing constants
// ---------------------------------------------------------------------------

/// Delay from end of TX to opening of RX1 window (seconds).
pub const RECEIVE_DELAY1: f64 = 1.0;

/// Delay from end of TX to opening of RX2 window (seconds).
pub const RECEIVE_DELAY2: f64 = 2.0;

/// Default RX1 data-rate offset.
pub const DEFAULT_RX1_DR_OFFSET: u8 = 0;

// ---------------------------------------------------------------------------
// LorawanMacScheduler
// ---------------------------------------------------------------------------

/// LoRaWAN MAC-layer scheduler.
///
/// Tracks duty-cycle budgets per sub-band, computes time-on-air, manages RX
/// window timing, and runs the ADR algorithm.
#[derive(Debug, Clone)]
pub struct LorawanMacScheduler {
    /// Active region.
    pub region: Region,
    /// Sub-band definitions with duty-cycle tracking.
    pub sub_bands: Vec<SubBand>,
    /// Data-rate table for the region.
    pub data_rates: Vec<DataRateEntry>,
    /// TX power table for the region.
    pub tx_powers: Vec<TxPowerEntry>,
    /// Channel plan.
    pub channels: Vec<Channel>,
    /// ADR state.
    pub adr: AdrState,
    /// Duration of the tracking window for duty-cycle accounting (seconds).
    pub tracking_window: f64,
    /// Running total airtime across all sub-bands (seconds).
    pub total_airtime: f64,
}

impl LorawanMacScheduler {
    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------

    /// Create a new scheduler for the given region with default parameters.
    pub fn new(region: Region) -> Self {
        let (sub_bands, data_rates, tx_powers, channels) = match region {
            Region::EU868 => (
                Self::eu868_sub_bands(),
                Self::eu868_data_rates(),
                Self::eu868_tx_powers(),
                Self::eu868_channels(),
            ),
            Region::US915 => (
                Self::us915_sub_bands(),
                Self::us915_data_rates(),
                Self::us915_tx_powers(),
                Self::us915_channels(),
            ),
        };

        Self {
            region,
            sub_bands,
            data_rates,
            tx_powers,
            channels,
            adr: AdrState::new(0, 0),
            tracking_window: 3600.0, // 1 hour
            total_airtime: 0.0,
        }
    }

    // ------------------------------------------------------------------
    // Time-on-air calculation  (SX1276-style formula)
    // ------------------------------------------------------------------

    /// Compute the on-air time (seconds) for a LoRa frame.
    ///
    /// # Parameters
    /// - `sf` - Spreading factor (5-12).
    /// - `bw` - Bandwidth in Hz (e.g. 125_000).
    /// - `payload_len` - MAC payload length in bytes.
    /// - `coding_rate` - Coding-rate denominator offset (1-4, for CR 4/5 through 4/8).
    ///
    /// Uses explicit header, CRC-on, low-data-rate-optimise when SF >= 11 and
    /// BW = 125 kHz.
    pub fn time_on_air(&self, sf: u8, bw: f64, payload_len: usize, coding_rate: u8) -> f64 {
        let sf = sf as f64;
        let cr = coding_rate.clamp(1, 4) as f64;
        let de = if sf >= 11.0 && bw <= 125_000.0 { 1.0 } else { 0.0 };
        let n_preamble = 8.0; // standard preamble length

        let t_sym = 2.0_f64.powf(sf) / bw;

        // Number of payload symbols (LoRa modem formula)
        let numerator = 8.0 * payload_len as f64 - 4.0 * sf + 28.0 + 16.0;
        let denom = 4.0 * (sf - 2.0 * de);
        let n_payload = 8.0 + ((numerator / denom).ceil() * (cr + 4.0)).max(0.0);

        let t_preamble = (n_preamble + 4.25) * t_sym;
        let t_payload = n_payload * t_sym;

        t_preamble + t_payload
    }

    // ------------------------------------------------------------------
    // Duty-cycle management
    // ------------------------------------------------------------------

    /// Return `true` if the given sub-band can accept a new transmission at
    /// time `now` (seconds).
    pub fn can_transmit(&self, sub_band_idx: usize, now: f64) -> bool {
        if let Some(sb) = self.sub_bands.get(sub_band_idx) {
            if sb.airtime_used == 0.0 {
                return true;
            }
            let off_time = sb.airtime_used / sb.duty_cycle_limit - sb.airtime_used;
            now >= sb.last_tx_end + off_time
        } else {
            false
        }
    }

    /// Record a completed transmission on a sub-band.
    ///
    /// - `sub_band_idx` - index into `self.sub_bands`.
    /// - `start` - TX start time (seconds).
    /// - `airtime` - duration of the transmission (seconds).
    pub fn record_transmission(&mut self, sub_band_idx: usize, start: f64, airtime: f64) {
        if let Some(sb) = self.sub_bands.get_mut(sub_band_idx) {
            sb.airtime_used += airtime;
            sb.last_tx_end = start + airtime;
        }
        self.total_airtime += airtime;
    }

    /// Earliest time (seconds) at which a new transmission may begin on the
    /// given sub-band.
    pub fn next_available_slot(&self, sub_band_idx: usize, now: f64) -> f64 {
        if let Some(sb) = self.sub_bands.get(sub_band_idx) {
            if sb.airtime_used == 0.0 {
                return now;
            }
            let off_time = sb.airtime_used / sb.duty_cycle_limit - sb.airtime_used;
            let earliest = sb.last_tx_end + off_time;
            if earliest > now { earliest } else { now }
        } else {
            f64::INFINITY
        }
    }

    /// Reset duty-cycle tracking for all sub-bands (e.g. at the start of a
    /// new tracking window).
    pub fn reset_duty_cycle(&mut self) {
        for sb in &mut self.sub_bands {
            sb.airtime_used = 0.0;
            sb.last_tx_end = 0.0;
        }
        self.total_airtime = 0.0;
    }

    /// Return the fraction of duty cycle consumed in sub-band `idx`.
    pub fn duty_cycle_usage(&self, sub_band_idx: usize) -> f64 {
        if let Some(sb) = self.sub_bands.get(sub_band_idx) {
            if self.tracking_window > 0.0 {
                sb.airtime_used / self.tracking_window
            } else {
                0.0
            }
        } else {
            0.0
        }
    }

    // ------------------------------------------------------------------
    // RX window timing
    // ------------------------------------------------------------------

    /// Compute the absolute opening time of the RX1 window.
    ///
    /// `tx_end` is the timestamp (seconds) at which the uplink finished.
    pub fn rx1_open(&self, tx_end: f64) -> f64 {
        tx_end + RECEIVE_DELAY1
    }

    /// Compute the absolute opening time of the RX2 window.
    pub fn rx2_open(&self, tx_end: f64) -> f64 {
        tx_end + RECEIVE_DELAY2
    }

    /// Derive the RX1 data-rate index from the uplink DR and DR offset.
    ///
    /// Clamps to the valid range for the region.
    pub fn rx1_data_rate(&self, uplink_dr: u8, dr_offset: u8) -> u8 {
        let max_dr = self.data_rates.len().saturating_sub(1) as u8;
        if dr_offset > uplink_dr {
            0
        } else {
            (uplink_dr - dr_offset).min(max_dr)
        }
    }

    /// Return the RX2 default data-rate index for the region.
    pub fn rx2_data_rate(&self) -> u8 {
        match self.region {
            Region::EU868 => 0, // SF12/125 kHz
            Region::US915 => 8, // SF12/500 kHz
        }
    }

    /// Return the RX2 default frequency (Hz) for the region.
    pub fn rx2_frequency(&self) -> f64 {
        match self.region {
            Region::EU868 => 869_525_000.0,
            Region::US915 => 923_300_000.0,
        }
    }

    // ------------------------------------------------------------------
    // ADR algorithm
    // ------------------------------------------------------------------

    /// Feed a new uplink SNR observation (dB) into the ADR history.
    pub fn adr_observe_snr(&mut self, snr_db: f64) {
        self.adr.snr_history.push(snr_db);
        if self.adr.snr_history.len() > self.adr.history_len {
            self.adr.snr_history.remove(0);
        }
    }

    /// Run the ADR algorithm and return `(new_dr, new_tx_power_index)`.
    ///
    /// The algorithm:
    /// 1. Compute the maximum SNR over the history window.
    /// 2. Look up the required SNR floor for the current SF.
    /// 3. `margin = max_snr - required_snr - snr_margin`
    /// 4. While margin > 0, first increase DR (lower SF), then reduce TX power.
    pub fn adr_compute(&mut self) -> (u8, u8) {
        if self.adr.snr_history.is_empty() {
            return (self.adr.current_dr, self.adr.current_tx_power);
        }

        let max_snr = self
            .adr
            .snr_history
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);

        // Required demodulation SNR per SF (empirical, from Semtech docs)
        let required_snr = self.required_snr_for_dr(self.adr.current_dr);

        let mut margin = max_snr - required_snr - self.adr.snr_margin;

        let mut dr = self.adr.current_dr;
        let mut tx_power = self.adr.current_tx_power;
        let max_dr = self.max_data_rate();
        let max_tx_idx = self.tx_powers.len().saturating_sub(1) as u8;

        // Step 1: increase DR while margin allows (each DR step ~ 2.5 dB)
        while margin >= 2.5 && dr < max_dr {
            dr += 1;
            margin -= 2.5;
        }

        // Step 2: reduce TX power in 3 dB steps
        while margin >= 3.0 && tx_power < max_tx_idx {
            tx_power += 1;
            margin -= 3.0;
        }

        self.adr.current_dr = dr;
        self.adr.current_tx_power = tx_power;

        (dr, tx_power)
    }

    /// Return the required demodulation SNR (dB) for a given DR index.
    pub fn required_snr_for_dr(&self, dr: u8) -> f64 {
        let sf = self.data_rates.get(dr as usize).map(|e| e.sf).unwrap_or(12);
        Self::required_snr_for_sf(sf)
    }

    /// Required SNR (dB) for a given spreading factor (Semtech empirical values).
    pub fn required_snr_for_sf(sf: u8) -> f64 {
        match sf {
            5 => -2.5,
            6 => -5.0,
            7 => -7.5,
            8 => -10.0,
            9 => -12.5,
            10 => -15.0,
            11 => -17.5,
            12 => -20.0,
            _ => -7.5, // default to SF7
        }
    }

    /// Maximum valid data-rate index for the active region.
    pub fn max_data_rate(&self) -> u8 {
        self.data_rates.len().saturating_sub(1) as u8
    }

    // ------------------------------------------------------------------
    // Channel helpers
    // ------------------------------------------------------------------

    /// Return the list of enabled channel frequencies.
    pub fn enabled_channels(&self) -> Vec<f64> {
        self.channels
            .iter()
            .filter(|c| c.enabled)
            .map(|c| c.freq)
            .collect()
    }

    /// Determine which sub-band index a frequency belongs to.
    pub fn sub_band_for_freq(&self, freq: f64) -> Option<usize> {
        self.sub_bands
            .iter()
            .position(|sb| freq >= sb.freq_lo && freq <= sb.freq_hi)
    }

    /// Maximum payload size (bytes) for a given data-rate index.
    pub fn max_payload_for_dr(&self, dr: u8) -> usize {
        self.data_rates
            .get(dr as usize)
            .map(|e| e.max_payload)
            .unwrap_or(0)
    }

    /// Look up the SF and BW for a data-rate index.
    pub fn dr_to_sf_bw(&self, dr: u8) -> Option<(u8, f64)> {
        self.data_rates.get(dr as usize).map(|e| (e.sf, e.bw))
    }

    /// Look up the TX EIRP (dBm) for a power index.
    pub fn tx_power_dbm(&self, power_index: u8) -> f64 {
        self.tx_powers
            .get(power_index as usize)
            .map(|e| e.eirp_dbm)
            .unwrap_or(0.0)
    }

    // ------------------------------------------------------------------
    // Airtime budget helpers
    // ------------------------------------------------------------------

    /// Total airtime consumed across all sub-bands (seconds).
    pub fn total_airtime_used(&self) -> f64 {
        self.total_airtime
    }

    /// Remaining airtime budget in a sub-band for the tracking window.
    pub fn remaining_airtime(&self, sub_band_idx: usize) -> f64 {
        if let Some(sb) = self.sub_bands.get(sub_band_idx) {
            let max = self.tracking_window * sb.duty_cycle_limit;
            (max - sb.airtime_used).max(0.0)
        } else {
            0.0
        }
    }

    // ------------------------------------------------------------------
    // Complex-sample helper (uses (f64, f64) tuples)
    // ------------------------------------------------------------------

    /// Generate a complex preamble chirp symbol at the given sample rate.
    ///
    /// Returns `Vec<(f64, f64)>` where each element is `(re, im)`.
    /// This produces a single up-chirp across the bandwidth.
    pub fn generate_preamble_chirp(&self, sf: u8, bw: f64, sample_rate: f64) -> Vec<(f64, f64)> {
        let n_samples = 2usize.pow(sf as u32);
        let t_sym = n_samples as f64 / bw;
        let samples_out = (t_sym * sample_rate).round() as usize;

        let mut out = Vec::with_capacity(samples_out);
        for i in 0..samples_out {
            let t = i as f64 / sample_rate;
            let f_inst = -bw / 2.0 + (bw / t_sym) * t;
            let phase = 2.0 * std::f64::consts::PI * f_inst * t;
            out.push((phase.cos(), phase.sin()));
        }
        out
    }

    /// Compute magnitude squared of a complex sample.
    #[inline]
    pub fn mag_sq(sample: (f64, f64)) -> f64 {
        sample.0 * sample.0 + sample.1 * sample.1
    }

    // ------------------------------------------------------------------
    // Region-specific tables (private)
    // ------------------------------------------------------------------

    fn eu868_sub_bands() -> Vec<SubBand> {
        vec![
            // g (868.0-868.6 MHz): 1% duty cycle
            SubBand {
                freq_lo: 868_000_000.0,
                freq_hi: 868_600_000.0,
                duty_cycle_limit: 0.01,
                airtime_used: 0.0,
                last_tx_end: 0.0,
            },
            // g1 (868.7-869.2 MHz): 0.1% duty cycle
            SubBand {
                freq_lo: 868_700_000.0,
                freq_hi: 869_200_000.0,
                duty_cycle_limit: 0.001,
                airtime_used: 0.0,
                last_tx_end: 0.0,
            },
            // g2 (869.4-869.65 MHz): 10% duty cycle
            SubBand {
                freq_lo: 869_400_000.0,
                freq_hi: 869_650_000.0,
                duty_cycle_limit: 0.10,
                airtime_used: 0.0,
                last_tx_end: 0.0,
            },
            // g3 (869.7-870.0 MHz): 1% duty cycle
            SubBand {
                freq_lo: 869_700_000.0,
                freq_hi: 870_000_000.0,
                duty_cycle_limit: 0.01,
                airtime_used: 0.0,
                last_tx_end: 0.0,
            },
        ]
    }

    fn eu868_data_rates() -> Vec<DataRateEntry> {
        vec![
            DataRateEntry { dr: 0, sf: 12, bw: 125_000.0, max_payload: 51 },
            DataRateEntry { dr: 1, sf: 11, bw: 125_000.0, max_payload: 51 },
            DataRateEntry { dr: 2, sf: 10, bw: 125_000.0, max_payload: 51 },
            DataRateEntry { dr: 3, sf: 9, bw: 125_000.0, max_payload: 115 },
            DataRateEntry { dr: 4, sf: 8, bw: 125_000.0, max_payload: 222 },
            DataRateEntry { dr: 5, sf: 7, bw: 125_000.0, max_payload: 222 },
            DataRateEntry { dr: 6, sf: 7, bw: 250_000.0, max_payload: 222 },
        ]
    }

    fn eu868_tx_powers() -> Vec<TxPowerEntry> {
        vec![
            TxPowerEntry { index: 0, eirp_dbm: 16.0 },
            TxPowerEntry { index: 1, eirp_dbm: 14.0 },
            TxPowerEntry { index: 2, eirp_dbm: 12.0 },
            TxPowerEntry { index: 3, eirp_dbm: 10.0 },
            TxPowerEntry { index: 4, eirp_dbm: 8.0 },
            TxPowerEntry { index: 5, eirp_dbm: 6.0 },
            TxPowerEntry { index: 6, eirp_dbm: 4.0 },
            TxPowerEntry { index: 7, eirp_dbm: 2.0 },
        ]
    }

    fn eu868_channels() -> Vec<Channel> {
        vec![
            Channel { freq: 868_100_000.0, dr_min: 0, dr_max: 5, enabled: true },
            Channel { freq: 868_300_000.0, dr_min: 0, dr_max: 5, enabled: true },
            Channel { freq: 868_500_000.0, dr_min: 0, dr_max: 5, enabled: true },
        ]
    }

    fn us915_sub_bands() -> Vec<SubBand> {
        // US915 uses dwell time (400 ms), not duty-cycle limits.
        // Model 8 sub-bands of 8 channels each, all at 100% duty cycle.
        (0..8)
            .map(|i| {
                let lo = 902_300_000.0 + i as f64 * 1_600_000.0;
                SubBand {
                    freq_lo: lo,
                    freq_hi: lo + 1_600_000.0,
                    duty_cycle_limit: 1.0,
                    airtime_used: 0.0,
                    last_tx_end: 0.0,
                }
            })
            .collect()
    }

    fn us915_data_rates() -> Vec<DataRateEntry> {
        vec![
            DataRateEntry { dr: 0, sf: 10, bw: 125_000.0, max_payload: 11 },
            DataRateEntry { dr: 1, sf: 9, bw: 125_000.0, max_payload: 53 },
            DataRateEntry { dr: 2, sf: 8, bw: 125_000.0, max_payload: 125 },
            DataRateEntry { dr: 3, sf: 7, bw: 125_000.0, max_payload: 242 },
            DataRateEntry { dr: 4, sf: 8, bw: 500_000.0, max_payload: 242 },
            DataRateEntry { dr: 5, sf: 7, bw: 500_000.0, max_payload: 242 },
            DataRateEntry { dr: 6, sf: 7, bw: 500_000.0, max_payload: 242 },
            DataRateEntry { dr: 7, sf: 7, bw: 500_000.0, max_payload: 242 },
            // DR8-13: downlink only (500 kHz)
            DataRateEntry { dr: 8, sf: 12, bw: 500_000.0, max_payload: 53 },
            DataRateEntry { dr: 9, sf: 11, bw: 500_000.0, max_payload: 129 },
            DataRateEntry { dr: 10, sf: 10, bw: 500_000.0, max_payload: 242 },
            DataRateEntry { dr: 11, sf: 9, bw: 500_000.0, max_payload: 242 },
            DataRateEntry { dr: 12, sf: 8, bw: 500_000.0, max_payload: 242 },
            DataRateEntry { dr: 13, sf: 7, bw: 500_000.0, max_payload: 242 },
        ]
    }

    fn us915_tx_powers() -> Vec<TxPowerEntry> {
        vec![
            TxPowerEntry { index: 0, eirp_dbm: 30.0 },
            TxPowerEntry { index: 1, eirp_dbm: 28.0 },
            TxPowerEntry { index: 2, eirp_dbm: 26.0 },
            TxPowerEntry { index: 3, eirp_dbm: 24.0 },
            TxPowerEntry { index: 4, eirp_dbm: 22.0 },
            TxPowerEntry { index: 5, eirp_dbm: 20.0 },
            TxPowerEntry { index: 6, eirp_dbm: 18.0 },
            TxPowerEntry { index: 7, eirp_dbm: 16.0 },
            TxPowerEntry { index: 8, eirp_dbm: 14.0 },
            TxPowerEntry { index: 9, eirp_dbm: 12.0 },
            TxPowerEntry { index: 10, eirp_dbm: 10.0 },
        ]
    }

    fn us915_channels() -> Vec<Channel> {
        // 64 x 125 kHz uplink channels + 8 x 500 kHz uplink channels
        let mut chs: Vec<Channel> = (0..64)
            .map(|i| Channel {
                freq: 902_300_000.0 + i as f64 * 200_000.0,
                dr_min: 0,
                dr_max: 3,
                enabled: true,
            })
            .collect();
        for i in 0..8 {
            chs.push(Channel {
                freq: 903_000_000.0 + i as f64 * 1_600_000.0,
                dr_min: 4,
                dr_max: 4,
                enabled: true,
            });
        }
        chs
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -- Construction -------------------------------------------------------

    #[test]
    fn test_new_eu868() {
        let s = LorawanMacScheduler::new(Region::EU868);
        assert_eq!(s.region, Region::EU868);
        assert_eq!(s.sub_bands.len(), 4);
        assert_eq!(s.data_rates.len(), 7);
        assert_eq!(s.tx_powers.len(), 8);
        assert_eq!(s.channels.len(), 3);
    }

    #[test]
    fn test_new_us915() {
        let s = LorawanMacScheduler::new(Region::US915);
        assert_eq!(s.region, Region::US915);
        assert_eq!(s.sub_bands.len(), 8);
        assert_eq!(s.data_rates.len(), 14);
        assert_eq!(s.tx_powers.len(), 11);
        // 64 + 8 = 72 channels
        assert_eq!(s.channels.len(), 72);
    }

    // -- Time on air --------------------------------------------------------

    #[test]
    fn test_time_on_air_sf7_125k() {
        let s = LorawanMacScheduler::new(Region::EU868);
        let toa = s.time_on_air(7, 125_000.0, 20, 1);
        // SF7/125 kHz with 20 bytes should be roughly 50-70 ms
        assert!(toa > 0.040 && toa < 0.100, "toa = {toa}");
    }

    #[test]
    fn test_time_on_air_sf12_125k() {
        let s = LorawanMacScheduler::new(Region::EU868);
        let toa = s.time_on_air(12, 125_000.0, 20, 1);
        // SF12/125 kHz with 20 bytes should be ~1.3-1.8 s
        assert!(toa > 1.0 && toa < 2.5, "toa = {toa}");
    }

    #[test]
    fn test_time_on_air_increases_with_sf() {
        let s = LorawanMacScheduler::new(Region::EU868);
        let toa7 = s.time_on_air(7, 125_000.0, 20, 1);
        let toa12 = s.time_on_air(12, 125_000.0, 20, 1);
        assert!(toa12 > toa7 * 10.0, "SF12 should be much longer than SF7");
    }

    #[test]
    fn test_time_on_air_increases_with_payload() {
        let s = LorawanMacScheduler::new(Region::EU868);
        let small = s.time_on_air(7, 125_000.0, 5, 1);
        let large = s.time_on_air(7, 125_000.0, 200, 1);
        assert!(large > small);
    }

    // -- Duty cycle ---------------------------------------------------------

    #[test]
    fn test_can_transmit_fresh() {
        let s = LorawanMacScheduler::new(Region::EU868);
        assert!(s.can_transmit(0, 0.0));
    }

    #[test]
    fn test_duty_cycle_backoff() {
        let mut s = LorawanMacScheduler::new(Region::EU868);
        let toa = s.time_on_air(7, 125_000.0, 20, 1);
        s.record_transmission(0, 0.0, toa);

        // Immediately after TX the sub-band should be blocked
        assert!(!s.can_transmit(0, toa));

        // After the required off-time it should be available
        let slot = s.next_available_slot(0, toa);
        assert!(s.can_transmit(0, slot));
    }

    #[test]
    fn test_next_available_slot_no_tx() {
        let s = LorawanMacScheduler::new(Region::EU868);
        // With no prior TX, slot should be now
        assert_eq!(s.next_available_slot(0, 5.0), 5.0);
    }

    #[test]
    fn test_reset_duty_cycle() {
        let mut s = LorawanMacScheduler::new(Region::EU868);
        s.record_transmission(0, 0.0, 1.0);
        assert!(s.total_airtime > 0.0);
        s.reset_duty_cycle();
        assert_eq!(s.total_airtime, 0.0);
        assert!(s.can_transmit(0, 0.0));
    }

    #[test]
    fn test_duty_cycle_usage() {
        let mut s = LorawanMacScheduler::new(Region::EU868);
        s.record_transmission(0, 0.0, 36.0); // 36 s in a 3600 s window = 1%
        let usage = s.duty_cycle_usage(0);
        assert!((usage - 0.01).abs() < 1e-9);
    }

    // -- RX windows ---------------------------------------------------------

    #[test]
    fn test_rx_window_timing() {
        let s = LorawanMacScheduler::new(Region::EU868);
        let tx_end = 10.0;
        assert!((s.rx1_open(tx_end) - 11.0).abs() < 1e-9);
        assert!((s.rx2_open(tx_end) - 12.0).abs() < 1e-9);
    }

    #[test]
    fn test_rx1_data_rate_offset() {
        let s = LorawanMacScheduler::new(Region::EU868);
        assert_eq!(s.rx1_data_rate(5, 0), 5);
        assert_eq!(s.rx1_data_rate(5, 3), 2);
        // Offset larger than DR clamps to 0
        assert_eq!(s.rx1_data_rate(2, 5), 0);
    }

    #[test]
    fn test_rx2_defaults() {
        let eu = LorawanMacScheduler::new(Region::EU868);
        assert_eq!(eu.rx2_data_rate(), 0);
        assert!((eu.rx2_frequency() - 869_525_000.0).abs() < 1.0);

        let us = LorawanMacScheduler::new(Region::US915);
        assert_eq!(us.rx2_data_rate(), 8);
        assert!((us.rx2_frequency() - 923_300_000.0).abs() < 1.0);
    }

    // -- ADR ----------------------------------------------------------------

    #[test]
    fn test_adr_no_history() {
        let mut s = LorawanMacScheduler::new(Region::EU868);
        let (dr, pwr) = s.adr_compute();
        assert_eq!(dr, 0);
        assert_eq!(pwr, 0);
    }

    #[test]
    fn test_adr_high_snr_raises_dr() {
        let mut s = LorawanMacScheduler::new(Region::EU868);
        // Feed very high SNR observations
        for _ in 0..20 {
            s.adr_observe_snr(20.0);
        }
        let (dr, _pwr) = s.adr_compute();
        // Should raise DR above 0
        assert!(dr > 0, "ADR should raise DR with high SNR, got DR={dr}");
    }

    #[test]
    fn test_adr_history_length() {
        let mut s = LorawanMacScheduler::new(Region::EU868);
        for i in 0..30 {
            s.adr_observe_snr(i as f64);
        }
        assert_eq!(s.adr.snr_history.len(), 20);
    }

    // -- Channel / data-rate helpers ----------------------------------------

    #[test]
    fn test_enabled_channels() {
        let s = LorawanMacScheduler::new(Region::EU868);
        let chs = s.enabled_channels();
        assert_eq!(chs.len(), 3);
    }

    #[test]
    fn test_sub_band_for_freq() {
        let s = LorawanMacScheduler::new(Region::EU868);
        assert_eq!(s.sub_band_for_freq(868_100_000.0), Some(0));
        assert_eq!(s.sub_band_for_freq(869_500_000.0), Some(2));
        assert_eq!(s.sub_band_for_freq(800_000_000.0), None);
    }

    #[test]
    fn test_max_payload_for_dr() {
        let s = LorawanMacScheduler::new(Region::EU868);
        assert_eq!(s.max_payload_for_dr(0), 51);
        assert_eq!(s.max_payload_for_dr(5), 222);
    }

    #[test]
    fn test_dr_to_sf_bw() {
        let s = LorawanMacScheduler::new(Region::EU868);
        assert_eq!(s.dr_to_sf_bw(0), Some((12, 125_000.0)));
        assert_eq!(s.dr_to_sf_bw(5), Some((7, 125_000.0)));
        assert_eq!(s.dr_to_sf_bw(99), None);
    }

    // -- Complex chirp / airtime budget -------------------------------------

    #[test]
    fn test_generate_preamble_chirp() {
        let s = LorawanMacScheduler::new(Region::EU868);
        let chirp = s.generate_preamble_chirp(7, 125_000.0, 125_000.0);
        assert_eq!(chirp.len(), 128); // 2^7 = 128, at 1x oversampling
        // All samples should have unit magnitude (within rounding)
        for &(re, im) in &chirp {
            let mag = (re * re + im * im).sqrt();
            assert!((mag - 1.0).abs() < 1e-6, "mag = {mag}");
        }
    }

    #[test]
    fn test_remaining_airtime() {
        let mut s = LorawanMacScheduler::new(Region::EU868);
        // Sub-band 0: 1% of 3600 s = 36 s budget
        let budget = s.remaining_airtime(0);
        assert!((budget - 36.0).abs() < 1e-6);
        s.record_transmission(0, 0.0, 10.0);
        let remaining = s.remaining_airtime(0);
        assert!((remaining - 26.0).abs() < 1e-6);
    }

    #[test]
    fn test_tx_power_dbm() {
        let s = LorawanMacScheduler::new(Region::EU868);
        assert!((s.tx_power_dbm(0) - 16.0).abs() < 1e-6);
        assert!((s.tx_power_dbm(7) - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_required_snr_for_sf() {
        assert!((LorawanMacScheduler::required_snr_for_sf(7) - (-7.5)).abs() < 1e-9);
        assert!((LorawanMacScheduler::required_snr_for_sf(12) - (-20.0)).abs() < 1e-9);
    }
}

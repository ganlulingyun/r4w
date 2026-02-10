//! Dynamic Spectrum Manager — tracks spectrum occupancy and optimizes
//! frequency/time/power allocation for multi-user coexistence.
//!
//! The central type is [`DynamicSpectrumManager`], which maintains an
//! [`OccupancyGrid`] (a 2-D time-frequency matrix of observed power levels)
//! and exposes helpers to find vacant channels, allocate resources, and
//! compute coexistence metrics.
//!
//! # Example
//!
//! ```
//! use r4w_core::dynamic_spectrum_manager::{
//!     DynamicSpectrumManager, AllocationRequest,
//! };
//!
//! // Manage 900 MHz ISM band (902–928 MHz), 100 time slots
//! let mut mgr = DynamicSpectrumManager::new(902.0e6, 928.0e6, 26, 100);
//!
//! // Feed a PSD measurement: time slot 0 has energy at band index 3
//! mgr.update_occupancy(0, 3, -60.0);
//!
//! // Ask for a 1 MHz channel
//! let vacant = mgr.find_vacant_channels(1.0e6, -90.0);
//! assert!(!vacant.is_empty());
//!
//! // Allocate a user
//! let req = AllocationRequest {
//!     required_bandwidth: 1.0e6,
//!     max_latency_slots: 10,
//!     min_snr_db: 10.0,
//!     user_id: 42,
//! };
//! let result = mgr.allocate(&req);
//! assert!(result.is_some());
//! ```

use std::fmt;

// ---------------------------------------------------------------------------
// SpectrumBand
// ---------------------------------------------------------------------------

/// Descriptor for a single spectrum band.
#[derive(Debug, Clone, PartialEq)]
pub struct SpectrumBand {
    /// Centre frequency in Hz.
    pub center_freq: f64,
    /// Bandwidth in Hz.
    pub bandwidth: f64,
    /// Regulatory transmit-power ceiling in dBm.
    pub regulatory_limit_dbm: f64,
    /// Priority (lower = higher priority, 0 is highest).
    pub priority: u32,
}

impl SpectrumBand {
    /// Lower edge frequency in Hz.
    pub fn lower_edge(&self) -> f64 {
        self.center_freq - self.bandwidth / 2.0
    }

    /// Upper edge frequency in Hz.
    pub fn upper_edge(&self) -> f64 {
        self.center_freq + self.bandwidth / 2.0
    }
}

// ---------------------------------------------------------------------------
// OccupancyGrid
// ---------------------------------------------------------------------------

/// 2-D time-frequency grid that records observed power levels (in dBm).
///
/// Rows are *time slots* and columns are *frequency bands*.  Each cell
/// stores the measured power spectral density for that slot/band pair.
/// Uninitialised cells default to `f64::NEG_INFINITY` (i.e. no energy).
#[derive(Debug, Clone)]
pub struct OccupancyGrid {
    /// Number of frequency bands (columns).
    pub num_freq_bands: usize,
    /// Number of time slots (rows).
    pub num_time_slots: usize,
    /// Row-major storage: `data[time * num_freq_bands + freq]`.
    data: Vec<f64>,
}

impl OccupancyGrid {
    /// Create a new grid with all cells set to `NEG_INFINITY`.
    pub fn new(num_freq_bands: usize, num_time_slots: usize) -> Self {
        Self {
            num_freq_bands,
            num_time_slots,
            data: vec![f64::NEG_INFINITY; num_freq_bands * num_time_slots],
        }
    }

    /// Read the power level at `(time_slot, freq_band)`.
    pub fn get(&self, time_slot: usize, freq_band: usize) -> f64 {
        self.data[time_slot * self.num_freq_bands + freq_band]
    }

    /// Write a power level at `(time_slot, freq_band)`.
    pub fn set(&mut self, time_slot: usize, freq_band: usize, power_dbm: f64) {
        self.data[time_slot * self.num_freq_bands + freq_band] = power_dbm;
    }

    /// Reset every cell to `NEG_INFINITY`.
    pub fn clear(&mut self) {
        self.data.fill(f64::NEG_INFINITY);
    }
}

// ---------------------------------------------------------------------------
// AllocationRequest / AllocationResult
// ---------------------------------------------------------------------------

/// A request for spectrum resources.
#[derive(Debug, Clone, PartialEq)]
pub struct AllocationRequest {
    /// Minimum required bandwidth in Hz.
    pub required_bandwidth: f64,
    /// Maximum acceptable latency in time-slot units.
    pub max_latency_slots: usize,
    /// Minimum acceptable SNR in dB.
    pub min_snr_db: f64,
    /// Opaque user identifier.
    pub user_id: u64,
}

/// The result of a successful spectrum allocation.
#[derive(Debug, Clone, PartialEq)]
pub struct AllocationResult {
    /// Assigned centre frequency in Hz.
    pub center_freq: f64,
    /// Assigned bandwidth in Hz.
    pub bandwidth: f64,
    /// Recommended transmit power in dBm.
    pub tx_power_dbm: f64,
    /// Assigned time slot index.
    pub time_slot: usize,
    /// User that received the allocation.
    pub user_id: u64,
}

impl fmt::Display for AllocationResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Allocation(user={}, fc={:.0} Hz, bw={:.0} Hz, pwr={:.1} dBm, slot={})",
            self.user_id, self.center_freq, self.bandwidth, self.tx_power_dbm, self.time_slot,
        )
    }
}

// ---------------------------------------------------------------------------
// VacantChannel
// ---------------------------------------------------------------------------

/// A contiguous vacant frequency range discovered by `find_vacant_channels`.
#[derive(Debug, Clone, PartialEq)]
pub struct VacantChannel {
    /// Centre frequency in Hz.
    pub center_freq: f64,
    /// Bandwidth in Hz.
    pub bandwidth: f64,
    /// Index of the first frequency band that composes this channel.
    pub start_band: usize,
    /// Number of contiguous bands in this channel.
    pub num_bands: usize,
}

// ---------------------------------------------------------------------------
// DynamicSpectrumManager
// ---------------------------------------------------------------------------

/// Tracks spectrum occupancy and allocates frequency/time/power resources.
///
/// The manager divides the frequency range `[freq_start, freq_end]` into
/// `num_freq_bands` equal-width bands and the observation window into
/// `num_time_slots` slots.
#[derive(Debug, Clone)]
pub struct DynamicSpectrumManager {
    /// Lower edge of managed spectrum in Hz.
    pub freq_start: f64,
    /// Upper edge of managed spectrum in Hz.
    pub freq_end: f64,
    /// Width of each frequency band in Hz.
    pub band_width: f64,
    /// The occupancy grid.
    pub grid: OccupancyGrid,
    /// Per-band descriptors (length = `num_freq_bands`).
    pub bands: Vec<SpectrumBand>,
    /// Active allocations.
    allocations: Vec<AllocationResult>,
    /// Default occupancy threshold in dBm.  Cells above this are "occupied".
    pub occupancy_threshold_dbm: f64,
    /// Default regulatory power limit in dBm when none is specified per-band.
    pub default_power_limit_dbm: f64,
}

impl DynamicSpectrumManager {
    /// Create a new manager spanning `[freq_start, freq_end]` Hz.
    ///
    /// * `num_freq_bands` – number of frequency bins.
    /// * `num_time_slots` – number of time-slot rows in the occupancy grid.
    pub fn new(
        freq_start: f64,
        freq_end: f64,
        num_freq_bands: usize,
        num_time_slots: usize,
    ) -> Self {
        assert!(freq_end > freq_start, "freq_end must exceed freq_start");
        assert!(num_freq_bands > 0);
        assert!(num_time_slots > 0);

        let total_bw = freq_end - freq_start;
        let band_width = total_bw / num_freq_bands as f64;

        let bands: Vec<SpectrumBand> = (0..num_freq_bands)
            .map(|i| {
                let center = freq_start + band_width * (i as f64 + 0.5);
                SpectrumBand {
                    center_freq: center,
                    bandwidth: band_width,
                    regulatory_limit_dbm: 30.0, // default 1 W
                    priority: 0,
                }
            })
            .collect();

        Self {
            freq_start,
            freq_end,
            band_width,
            grid: OccupancyGrid::new(num_freq_bands, num_time_slots),
            bands,
            allocations: Vec::new(),
            occupancy_threshold_dbm: -90.0,
            default_power_limit_dbm: 30.0,
        }
    }

    // -- Accessors ----------------------------------------------------------

    /// Number of frequency bands.
    pub fn num_freq_bands(&self) -> usize {
        self.grid.num_freq_bands
    }

    /// Number of time slots.
    pub fn num_time_slots(&self) -> usize {
        self.grid.num_time_slots
    }

    /// Current active allocations.
    pub fn allocations(&self) -> &[AllocationResult] {
        &self.allocations
    }

    // -- Occupancy ----------------------------------------------------------

    /// Feed a power spectral density measurement into the grid.
    ///
    /// * `time_slot` – time index (row).
    /// * `freq_band` – frequency index (column).
    /// * `power_dbm` – measured PSD in dBm.
    pub fn update_occupancy(&mut self, time_slot: usize, freq_band: usize, power_dbm: f64) {
        assert!(time_slot < self.grid.num_time_slots);
        assert!(freq_band < self.grid.num_freq_bands);
        self.grid.set(time_slot, freq_band, power_dbm);
    }

    /// Return `true` if the cell is considered occupied.
    fn is_occupied(&self, time_slot: usize, freq_band: usize) -> bool {
        self.grid.get(time_slot, freq_band) > self.occupancy_threshold_dbm
    }

    /// Fraction of *all* grid cells that are above the occupancy threshold.
    pub fn occupancy_ratio(&self) -> f64 {
        let total = self.grid.num_freq_bands * self.grid.num_time_slots;
        if total == 0 {
            return 0.0;
        }
        let occupied = (0..self.grid.num_time_slots)
            .flat_map(|t| (0..self.grid.num_freq_bands).map(move |f| (t, f)))
            .filter(|&(t, f)| self.is_occupied(t, f))
            .count();
        occupied as f64 / total as f64
    }

    // -- Interference temperature -------------------------------------------

    /// Estimated interference temperature for `freq_band` across all time
    /// slots.  Returns the *linear average* of measured powers (in dBm
    /// converted to mW and back).
    ///
    /// Cells with `NEG_INFINITY` (no measurement) are ignored.
    pub fn interference_temperature(&self, freq_band: usize) -> f64 {
        assert!(freq_band < self.grid.num_freq_bands);
        let mut sum_mw = 0.0_f64;
        let mut count = 0usize;
        for t in 0..self.grid.num_time_slots {
            let p = self.grid.get(t, freq_band);
            if p.is_finite() {
                sum_mw += dbm_to_mw(p);
                count += 1;
            }
        }
        if count == 0 {
            f64::NEG_INFINITY
        } else {
            mw_to_dbm(sum_mw / count as f64)
        }
    }

    // -- Coexistence score --------------------------------------------------

    /// A metric in `[0.0, 1.0]` describing how well current allocations
    /// avoid occupied spectrum.  1.0 means no allocation overlaps with
    /// any occupied cell; 0.0 means every allocation overlaps.
    ///
    /// Returns 1.0 if there are no allocations (nothing to interfere with).
    pub fn coexistence_score(&self) -> f64 {
        if self.allocations.is_empty() {
            return 1.0;
        }
        let mut total = 0usize;
        let mut clean = 0usize;
        for alloc in &self.allocations {
            let (band_start, band_end) = self.freq_to_band_range(alloc.center_freq, alloc.bandwidth);
            for b in band_start..band_end {
                total += 1;
                if !self.is_occupied(alloc.time_slot, b) {
                    clean += 1;
                }
            }
        }
        if total == 0 {
            1.0
        } else {
            clean as f64 / total as f64
        }
    }

    // -- Vacant channel discovery -------------------------------------------

    /// Find contiguous vacant frequency ranges that are at least
    /// `min_bandwidth` Hz wide, scanning the *latest* time slot (last row).
    ///
    /// A band is vacant if its power is at or below `threshold_dbm`.
    pub fn find_vacant_channels(
        &self,
        min_bandwidth: f64,
        threshold_dbm: f64,
    ) -> Vec<VacantChannel> {
        let last_slot = self.grid.num_time_slots - 1;
        self.find_vacant_channels_at(min_bandwidth, threshold_dbm, last_slot)
    }

    /// Like [`find_vacant_channels`](Self::find_vacant_channels) but for a
    /// specific time slot.
    pub fn find_vacant_channels_at(
        &self,
        min_bandwidth: f64,
        threshold_dbm: f64,
        time_slot: usize,
    ) -> Vec<VacantChannel> {
        let min_bands = (min_bandwidth / self.band_width).ceil() as usize;
        let mut result = Vec::new();
        let mut run_start: Option<usize> = None;

        for f in 0..self.grid.num_freq_bands {
            let vacant = self.grid.get(time_slot, f) <= threshold_dbm;
            if vacant {
                if run_start.is_none() {
                    run_start = Some(f);
                }
            }
            if !vacant || f == self.grid.num_freq_bands - 1 {
                if let Some(start) = run_start {
                    let end = if vacant { f + 1 } else { f };
                    let len = end - start;
                    if len >= min_bands {
                        let center = self.freq_start
                            + self.band_width * (start as f64 + len as f64 / 2.0);
                        result.push(VacantChannel {
                            center_freq: center,
                            bandwidth: self.band_width * len as f64,
                            start_band: start,
                            num_bands: len,
                        });
                    }
                    run_start = None;
                }
            }
        }
        result
    }

    // -- Allocation ---------------------------------------------------------

    /// Attempt to allocate spectrum resources for the given request.
    ///
    /// Strategy: scan time slots `0..max_latency_slots` and pick the first
    /// vacant channel whose bandwidth satisfies the request.  Power is set to
    /// the band's regulatory limit minus a 3 dB back-off.
    pub fn allocate(&mut self, req: &AllocationRequest) -> Option<AllocationResult> {
        let max_slot = req.max_latency_slots.min(self.grid.num_time_slots);
        for t in 0..max_slot {
            let channels =
                self.find_vacant_channels_at(req.required_bandwidth, self.occupancy_threshold_dbm, t);
            if let Some(ch) = channels.first() {
                // Determine the power limit from the first band in the channel.
                let band_limit = self.bands[ch.start_band].regulatory_limit_dbm;
                let tx_power = band_limit - 3.0; // 3 dB back-off

                let alloc = AllocationResult {
                    center_freq: ch.center_freq,
                    bandwidth: req.required_bandwidth,
                    tx_power_dbm: tx_power,
                    time_slot: t,
                    user_id: req.user_id,
                };
                self.allocations.push(alloc.clone());
                return Some(alloc);
            }
        }
        None
    }

    /// Remove all allocations for `user_id`.
    pub fn deallocate(&mut self, user_id: u64) {
        self.allocations.retain(|a| a.user_id != user_id);
    }

    // -- Helpers ------------------------------------------------------------

    /// Map a centre-frequency + bandwidth pair to a range of band indices.
    fn freq_to_band_range(&self, center_freq: f64, bandwidth: f64) -> (usize, usize) {
        let lower = center_freq - bandwidth / 2.0;
        let upper = center_freq + bandwidth / 2.0;
        let start = ((lower - self.freq_start) / self.band_width).floor().max(0.0) as usize;
        let end = ((upper - self.freq_start) / self.band_width)
            .ceil()
            .min(self.grid.num_freq_bands as f64) as usize;
        (start, end)
    }
}

// ---------------------------------------------------------------------------
// dBm <-> mW helpers (std-only)
// ---------------------------------------------------------------------------

fn dbm_to_mw(dbm: f64) -> f64 {
    10.0_f64.powf(dbm / 10.0)
}

fn mw_to_dbm(mw: f64) -> f64 {
    10.0 * mw.log10()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -- OccupancyGrid basics -----------------------------------------------

    #[test]
    fn test_occupancy_grid_new_defaults_to_neg_inf() {
        let grid = OccupancyGrid::new(4, 3);
        for t in 0..3 {
            for f in 0..4 {
                assert!(grid.get(t, f).is_infinite() && grid.get(t, f) < 0.0);
            }
        }
    }

    #[test]
    fn test_occupancy_grid_set_get() {
        let mut grid = OccupancyGrid::new(5, 2);
        grid.set(1, 3, -42.0);
        assert_eq!(grid.get(1, 3), -42.0);
        assert!(grid.get(0, 3).is_infinite());
    }

    #[test]
    fn test_occupancy_grid_clear() {
        let mut grid = OccupancyGrid::new(3, 3);
        grid.set(0, 0, -10.0);
        grid.clear();
        assert!(grid.get(0, 0).is_infinite());
    }

    // -- SpectrumBand edges -------------------------------------------------

    #[test]
    fn test_spectrum_band_edges() {
        let band = SpectrumBand {
            center_freq: 100.0e6,
            bandwidth: 2.0e6,
            regulatory_limit_dbm: 30.0,
            priority: 0,
        };
        assert!((band.lower_edge() - 99.0e6).abs() < 1.0);
        assert!((band.upper_edge() - 101.0e6).abs() < 1.0);
    }

    // -- DynamicSpectrumManager construction --------------------------------

    #[test]
    fn test_manager_construction() {
        let mgr = DynamicSpectrumManager::new(900.0e6, 928.0e6, 28, 50);
        assert_eq!(mgr.num_freq_bands(), 28);
        assert_eq!(mgr.num_time_slots(), 50);
        assert_eq!(mgr.bands.len(), 28);
        assert!((mgr.band_width - 1.0e6).abs() < 1.0);
    }

    // -- update_occupancy + occupancy_ratio ---------------------------------

    #[test]
    fn test_occupancy_ratio_empty() {
        let mgr = DynamicSpectrumManager::new(100.0, 200.0, 10, 10);
        assert!((mgr.occupancy_ratio() - 0.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_occupancy_ratio_partial() {
        let mut mgr = DynamicSpectrumManager::new(100.0, 200.0, 10, 10);
        // Fill 10 cells above threshold (threshold is -90 dBm)
        for f in 0..10 {
            mgr.update_occupancy(0, f, -50.0);
        }
        // 10 out of 100 cells occupied
        assert!((mgr.occupancy_ratio() - 0.1).abs() < 1e-9);
    }

    // -- find_vacant_channels -----------------------------------------------

    #[test]
    fn test_find_vacant_channels_all_vacant() {
        let mgr = DynamicSpectrumManager::new(0.0, 100.0, 10, 1);
        // band_width = 10, one time slot, no energy => entire band is vacant
        let vacant = mgr.find_vacant_channels(10.0, -90.0);
        assert_eq!(vacant.len(), 1);
        assert_eq!(vacant[0].num_bands, 10);
    }

    #[test]
    fn test_find_vacant_channels_with_occupancy() {
        let mut mgr = DynamicSpectrumManager::new(0.0, 100.0, 10, 1);
        // Occupy band 4 at time slot 0
        mgr.update_occupancy(0, 4, -30.0);
        let vacant = mgr.find_vacant_channels(30.0, -90.0);
        // Should get two segments: bands 0-3 (40 Hz) and bands 5-9 (50 Hz)
        assert_eq!(vacant.len(), 2);
        assert_eq!(vacant[0].num_bands, 4); // bands 0..4
        assert_eq!(vacant[1].num_bands, 5); // bands 5..10
    }

    // -- allocate -----------------------------------------------------------

    #[test]
    fn test_allocate_success() {
        let mut mgr = DynamicSpectrumManager::new(900.0e6, 910.0e6, 10, 10);
        let req = AllocationRequest {
            required_bandwidth: 1.0e6,
            max_latency_slots: 5,
            min_snr_db: 10.0,
            user_id: 1,
        };
        let result = mgr.allocate(&req);
        assert!(result.is_some());
        let r = result.unwrap();
        assert_eq!(r.user_id, 1);
        assert!(r.bandwidth >= 1.0e6);
        assert_eq!(mgr.allocations().len(), 1);
    }

    #[test]
    fn test_allocate_fails_when_full() {
        let mut mgr = DynamicSpectrumManager::new(0.0, 100.0, 10, 1);
        // Fill every band
        for f in 0..10 {
            mgr.update_occupancy(0, f, -10.0);
        }
        let req = AllocationRequest {
            required_bandwidth: 10.0,
            max_latency_slots: 1,
            min_snr_db: 10.0,
            user_id: 7,
        };
        assert!(mgr.allocate(&req).is_none());
    }

    // -- interference_temperature -------------------------------------------

    #[test]
    fn test_interference_temperature() {
        let mut mgr = DynamicSpectrumManager::new(0.0, 100.0, 5, 4);
        // Feed equal power in band 2 across all slots
        for t in 0..4 {
            mgr.update_occupancy(t, 2, -60.0);
        }
        let temp = mgr.interference_temperature(2);
        assert!((temp - (-60.0)).abs() < 0.1);
    }

    #[test]
    fn test_interference_temperature_no_measurements() {
        let mgr = DynamicSpectrumManager::new(0.0, 100.0, 5, 4);
        let temp = mgr.interference_temperature(0);
        assert!(temp.is_infinite() && temp < 0.0);
    }

    // -- coexistence_score --------------------------------------------------

    #[test]
    fn test_coexistence_score_no_allocations() {
        let mgr = DynamicSpectrumManager::new(0.0, 100.0, 10, 5);
        assert!((mgr.coexistence_score() - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_coexistence_score_clean_allocation() {
        let mut mgr = DynamicSpectrumManager::new(0.0, 100.0, 10, 5);
        // No occupancy at all, so any allocation should be fully clean.
        let req = AllocationRequest {
            required_bandwidth: 10.0,
            max_latency_slots: 5,
            min_snr_db: 5.0,
            user_id: 99,
        };
        mgr.allocate(&req);
        assert!((mgr.coexistence_score() - 1.0).abs() < f64::EPSILON);
    }

    // -- deallocate ---------------------------------------------------------

    #[test]
    fn test_deallocate() {
        let mut mgr = DynamicSpectrumManager::new(0.0, 100.0, 10, 5);
        let req = AllocationRequest {
            required_bandwidth: 10.0,
            max_latency_slots: 5,
            min_snr_db: 5.0,
            user_id: 55,
        };
        mgr.allocate(&req);
        assert_eq!(mgr.allocations().len(), 1);
        mgr.deallocate(55);
        assert!(mgr.allocations().is_empty());
    }

    // -- AllocationResult Display -------------------------------------------

    #[test]
    fn test_allocation_result_display() {
        let r = AllocationResult {
            center_freq: 905.0e6,
            bandwidth: 1.0e6,
            tx_power_dbm: 27.0,
            time_slot: 3,
            user_id: 12,
        };
        let s = format!("{}", r);
        assert!(s.contains("user=12"));
        assert!(s.contains("slot=3"));
    }

    // -- dBm conversion round-trip ------------------------------------------

    #[test]
    fn test_dbm_mw_roundtrip() {
        let original = -30.0_f64;
        let mw = dbm_to_mw(original);
        let back = mw_to_dbm(mw);
        assert!((back - original).abs() < 1e-10);
    }
}

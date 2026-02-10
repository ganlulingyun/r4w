//! # RF Environment Mapper
//!
//! Real-time RF environment mapping that combines spectral measurements with
//! spatial and temporal context to build an RF situational awareness picture.
//!
//! This module provides grid-based RF power mapping with temporal decay,
//! frequency band aggregation, occupancy statistics, heat map generation,
//! interference source localization, and spectrum availability queries.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::rf_environment_mapper::{RfEnvironmentMapper, RfMeasurement, BandDefinition};
//!
//! // Create a mapper covering a 100x100 meter area with 10m resolution
//! let mut mapper = RfEnvironmentMapper::new(
//!     0.0, 100.0,  // x range
//!     0.0, 100.0,  // y range
//!     10.0,         // cell size (meters)
//!     60.0,         // temporal half-life (seconds)
//! );
//!
//! // Define a frequency band of interest
//! mapper.add_band(BandDefinition::new("ISM 900", 902.0e6, 928.0e6));
//!
//! // Ingest a measurement
//! let m = RfMeasurement {
//!     frequency: 915.0e6,
//!     power_dbm: -60.0,
//!     location: (25.0, 35.0),
//!     timestamp: 0.0,
//! };
//! mapper.ingest(m);
//!
//! // Query occupancy at a grid cell
//! let stats = mapper.cell_stats(2, 3, "ISM 900", 1.0);
//! assert!(stats.is_some());
//! ```

use std::collections::HashMap;

/// A single RF power measurement with spatial and temporal context.
#[derive(Debug, Clone)]
pub struct RfMeasurement {
    /// Center frequency of the measurement in Hz.
    pub frequency: f64,
    /// Measured power level in dBm.
    pub power_dbm: f64,
    /// Spatial location as (x, y) coordinates (e.g., meters).
    pub location: (f64, f64),
    /// Timestamp in seconds (monotonic).
    pub timestamp: f64,
}

/// Defines a named frequency band for aggregation.
#[derive(Debug, Clone)]
pub struct BandDefinition {
    /// Human-readable name for the band.
    pub name: String,
    /// Lower frequency bound in Hz.
    pub freq_min: f64,
    /// Upper frequency bound in Hz.
    pub freq_max: f64,
}

impl BandDefinition {
    /// Create a new band definition.
    pub fn new(name: &str, freq_min: f64, freq_max: f64) -> Self {
        Self {
            name: name.to_string(),
            freq_min,
            freq_max,
        }
    }

    /// Check whether a frequency falls within this band.
    pub fn contains(&self, freq: f64) -> bool {
        freq >= self.freq_min && freq <= self.freq_max
    }
}

/// Per-cell, per-band occupancy statistics.
#[derive(Debug, Clone)]
pub struct CellStats {
    /// Number of measurements in this cell/band.
    pub measurement_count: usize,
    /// Peak power observed (dBm), after temporal weighting.
    pub peak_power_dbm: f64,
    /// Weighted average power (dBm).
    pub average_power_dbm: f64,
    /// Duty cycle estimate (fraction of measurements above a threshold).
    pub duty_cycle: f64,
    /// Sum of temporal weights (useful for diagnostics).
    pub total_weight: f64,
}

/// A stored measurement inside a grid cell, tagged with its band.
#[derive(Debug, Clone)]
struct StoredMeasurement {
    power_dbm: f64,
    timestamp: f64,
    frequency: f64,
}

/// A 2D power heat map for a given frequency band.
#[derive(Debug, Clone)]
pub struct HeatMap {
    /// Number of columns (x axis).
    pub cols: usize,
    /// Number of rows (y axis).
    pub rows: usize,
    /// Power values in dBm, row-major order. `f64::NEG_INFINITY` means no data.
    pub data: Vec<f64>,
}

impl HeatMap {
    /// Get the power value at grid coordinates (col, row).
    pub fn get(&self, col: usize, row: usize) -> f64 {
        self.data[row * self.cols + col]
    }
}

/// Localized interference source candidate.
#[derive(Debug, Clone)]
pub struct InterferenceSource {
    /// Estimated location (x, y).
    pub location: (f64, f64),
    /// Estimated power level at the source (dBm).
    pub power_dbm: f64,
    /// Grid column index of the peak.
    pub col: usize,
    /// Grid row index of the peak.
    pub row: usize,
}

/// Result of a spectrum availability query.
#[derive(Debug, Clone)]
pub struct SpectrumAvailability {
    /// Name of the band.
    pub band_name: String,
    /// Whether the band appears to be available (below threshold).
    pub available: bool,
    /// Observed average power in dBm (`f64::NEG_INFINITY` if no data).
    pub observed_power_dbm: f64,
}

/// Real-time RF environment mapper.
///
/// Maintains a grid-based spatial map of RF power measurements across
/// configurable frequency bands, with temporal decay of older measurements.
pub struct RfEnvironmentMapper {
    // Grid geometry
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    cell_size: f64,
    cols: usize,
    rows: usize,

    // Temporal decay
    half_life: f64,

    // Band definitions
    bands: Vec<BandDefinition>,

    // Storage: grid_cell_index -> list of measurements
    cells: HashMap<usize, Vec<StoredMeasurement>>,

    // Duty cycle threshold (dBm) – measurements above this count as "occupied"
    duty_cycle_threshold_dbm: f64,
}

impl RfEnvironmentMapper {
    /// Create a new RF environment mapper.
    ///
    /// # Arguments
    /// * `x_min`, `x_max` – spatial extent along x axis
    /// * `y_min`, `y_max` – spatial extent along y axis
    /// * `cell_size` – width/height of each grid cell
    /// * `half_life` – temporal half-life in seconds for exponential decay
    pub fn new(
        x_min: f64,
        x_max: f64,
        y_min: f64,
        y_max: f64,
        cell_size: f64,
        half_life: f64,
    ) -> Self {
        let cols = ((x_max - x_min) / cell_size).ceil() as usize;
        let rows = ((y_max - y_min) / cell_size).ceil() as usize;
        Self {
            x_min,
            x_max,
            y_min,
            y_max,
            cell_size,
            cols,
            rows,
            half_life,
            bands: Vec::new(),
            cells: HashMap::new(),
            duty_cycle_threshold_dbm: -90.0,
        }
    }

    /// Return the number of grid columns.
    pub fn cols(&self) -> usize {
        self.cols
    }

    /// Return the number of grid rows.
    pub fn rows(&self) -> usize {
        self.rows
    }

    /// Set the duty-cycle detection threshold in dBm.
    pub fn set_duty_cycle_threshold(&mut self, threshold_dbm: f64) {
        self.duty_cycle_threshold_dbm = threshold_dbm;
    }

    /// Get the duty-cycle detection threshold in dBm.
    pub fn duty_cycle_threshold(&self) -> f64 {
        self.duty_cycle_threshold_dbm
    }

    /// Add a frequency band definition for aggregation.
    pub fn add_band(&mut self, band: BandDefinition) {
        self.bands.push(band);
    }

    /// Return a reference to the configured bands.
    pub fn bands(&self) -> &[BandDefinition] {
        &self.bands
    }

    /// Convert (x, y) to a grid cell index, returning `None` if out of bounds.
    fn location_to_cell(&self, x: f64, y: f64) -> Option<usize> {
        if x < self.x_min || x >= self.x_max || y < self.y_min || y >= self.y_max {
            return None;
        }
        let col = ((x - self.x_min) / self.cell_size) as usize;
        let row = ((y - self.y_min) / self.cell_size) as usize;
        let col = col.min(self.cols - 1);
        let row = row.min(self.rows - 1);
        Some(row * self.cols + col)
    }

    /// Convert a grid cell index back to (col, row).
    fn cell_to_col_row(&self, idx: usize) -> (usize, usize) {
        let row = idx / self.cols;
        let col = idx % self.cols;
        (col, row)
    }

    /// Convert (col, row) to a cell index.
    fn col_row_to_cell(&self, col: usize, row: usize) -> usize {
        row * self.cols + col
    }

    /// Get the center coordinates of a grid cell.
    pub fn cell_center(&self, col: usize, row: usize) -> (f64, f64) {
        let x = self.x_min + (col as f64 + 0.5) * self.cell_size;
        let y = self.y_min + (row as f64 + 0.5) * self.cell_size;
        (x, y)
    }

    /// Compute the temporal decay weight for a measurement at `meas_time`
    /// evaluated at `current_time`.
    fn temporal_weight(&self, meas_time: f64, current_time: f64) -> f64 {
        if self.half_life <= 0.0 {
            return 1.0;
        }
        let dt = current_time - meas_time;
        if dt < 0.0 {
            return 1.0;
        }
        // w = 2^(-dt / half_life) = exp(-dt * ln2 / half_life)
        let decay = (-dt * std::f64::consts::LN_2 / self.half_life).exp();
        decay
    }

    /// Ingest a new RF measurement into the spatial grid.
    pub fn ingest(&mut self, measurement: RfMeasurement) {
        if let Some(cell_idx) = self.location_to_cell(measurement.location.0, measurement.location.1) {
            let stored = StoredMeasurement {
                power_dbm: measurement.power_dbm,
                timestamp: measurement.timestamp,
                frequency: measurement.frequency,
            };
            self.cells.entry(cell_idx).or_insert_with(Vec::new).push(stored);
        }
    }

    /// Ingest a batch of measurements.
    pub fn ingest_batch(&mut self, measurements: &[RfMeasurement]) {
        for m in measurements {
            self.ingest(m.clone());
        }
    }

    /// Return the total number of stored measurements across all cells.
    pub fn total_measurements(&self) -> usize {
        self.cells.values().map(|v| v.len()).sum()
    }

    /// Prune measurements older than `max_age` seconds relative to `current_time`.
    pub fn prune(&mut self, current_time: f64, max_age: f64) {
        let cutoff = current_time - max_age;
        for measurements in self.cells.values_mut() {
            measurements.retain(|m| m.timestamp >= cutoff);
        }
        self.cells.retain(|_, v| !v.is_empty());
    }

    /// Compute occupancy statistics for a specific cell and band.
    ///
    /// Returns `None` if there are no measurements matching the cell/band, or
    /// if the cell coordinates are out of range.
    pub fn cell_stats(
        &self,
        col: usize,
        row: usize,
        band_name: &str,
        current_time: f64,
    ) -> Option<CellStats> {
        if col >= self.cols || row >= self.rows {
            return None;
        }
        let band = self.bands.iter().find(|b| b.name == band_name)?;
        let cell_idx = self.col_row_to_cell(col, row);
        let measurements = self.cells.get(&cell_idx)?;

        // Filter to measurements in this band
        let band_measurements: Vec<&StoredMeasurement> = measurements
            .iter()
            .filter(|m| band.contains(m.frequency))
            .collect();

        if band_measurements.is_empty() {
            return None;
        }

        let mut peak_power = f64::NEG_INFINITY;
        let mut total_weight = 0.0;
        let mut weighted_power_sum = 0.0; // in linear domain
        let mut above_threshold_weight = 0.0;

        for m in &band_measurements {
            let w = self.temporal_weight(m.timestamp, current_time);
            let power_linear = 10.0_f64.powf(m.power_dbm / 10.0);
            let weighted_power_dbm = m.power_dbm + 10.0 * w.log10();
            if weighted_power_dbm > peak_power {
                peak_power = weighted_power_dbm;
            }
            weighted_power_sum += w * power_linear;
            total_weight += w;
            if m.power_dbm >= self.duty_cycle_threshold_dbm {
                above_threshold_weight += w;
            }
        }

        let avg_linear = if total_weight > 0.0 {
            weighted_power_sum / total_weight
        } else {
            0.0
        };
        let average_power_dbm = if avg_linear > 0.0 {
            10.0 * avg_linear.log10()
        } else {
            f64::NEG_INFINITY
        };

        let duty_cycle = if total_weight > 0.0 {
            above_threshold_weight / total_weight
        } else {
            0.0
        };

        Some(CellStats {
            measurement_count: band_measurements.len(),
            peak_power_dbm: peak_power,
            average_power_dbm,
            duty_cycle,
            total_weight,
        })
    }

    /// Generate a 2D heat map of weighted average power for a given band.
    ///
    /// Cells with no data contain `f64::NEG_INFINITY`.
    pub fn heat_map(&self, band_name: &str, current_time: f64) -> Option<HeatMap> {
        // Verify band exists
        if !self.bands.iter().any(|b| b.name == band_name) {
            return None;
        }

        let mut data = vec![f64::NEG_INFINITY; self.cols * self.rows];

        for row in 0..self.rows {
            for col in 0..self.cols {
                if let Some(stats) = self.cell_stats(col, row, band_name, current_time) {
                    data[row * self.cols + col] = stats.average_power_dbm;
                }
            }
        }

        Some(HeatMap {
            cols: self.cols,
            rows: self.rows,
            data,
        })
    }

    /// Locate potential interference sources using power gradient analysis.
    ///
    /// Finds local maxima in the heat map – cells whose weighted average power
    /// exceeds all of their 8-connected neighbours and is above `min_power_dbm`.
    pub fn locate_interference(
        &self,
        band_name: &str,
        current_time: f64,
        min_power_dbm: f64,
    ) -> Vec<InterferenceSource> {
        let hmap = match self.heat_map(band_name, current_time) {
            Some(h) => h,
            None => return Vec::new(),
        };

        let mut sources = Vec::new();

        for row in 0..self.rows {
            for col in 0..self.cols {
                let val = hmap.get(col, row);
                if val <= min_power_dbm || val == f64::NEG_INFINITY {
                    continue;
                }

                let mut is_peak = true;
                // Check 8-connected neighbours
                for dr in [-1i32, 0, 1] {
                    for dc in [-1i32, 0, 1] {
                        if dr == 0 && dc == 0 {
                            continue;
                        }
                        let nr = row as i32 + dr;
                        let nc = col as i32 + dc;
                        if nr >= 0 && nr < self.rows as i32 && nc >= 0 && nc < self.cols as i32 {
                            let neighbor_val = hmap.get(nc as usize, nr as usize);
                            if neighbor_val > val {
                                is_peak = false;
                                break;
                            }
                        }
                    }
                    if !is_peak {
                        break;
                    }
                }

                if is_peak {
                    let (cx, cy) = self.cell_center(col, row);
                    sources.push(InterferenceSource {
                        location: (cx, cy),
                        power_dbm: val,
                        col,
                        row,
                    });
                }
            }
        }

        // Sort by power descending
        sources.sort_by(|a, b| b.power_dbm.partial_cmp(&a.power_dbm).unwrap_or(std::cmp::Ordering::Equal));
        sources
    }

    /// Query spectrum availability at a location for all defined bands.
    ///
    /// A band is considered "available" if the weighted average power at the
    /// nearest grid cell is below `threshold_dbm`, or if there are no
    /// measurements for that band.
    pub fn spectrum_availability(
        &self,
        x: f64,
        y: f64,
        current_time: f64,
        threshold_dbm: f64,
    ) -> Vec<SpectrumAvailability> {
        let cell_idx = self.location_to_cell(x, y);
        let (col, row) = match cell_idx {
            Some(idx) => self.cell_to_col_row(idx),
            None => {
                // Out of bounds – return all bands as available with no data
                return self
                    .bands
                    .iter()
                    .map(|b| SpectrumAvailability {
                        band_name: b.name.clone(),
                        available: true,
                        observed_power_dbm: f64::NEG_INFINITY,
                    })
                    .collect();
            }
        };

        self.bands
            .iter()
            .map(|band| {
                match self.cell_stats(col, row, &band.name, current_time) {
                    Some(stats) => SpectrumAvailability {
                        band_name: band.name.clone(),
                        available: stats.average_power_dbm < threshold_dbm,
                        observed_power_dbm: stats.average_power_dbm,
                    },
                    None => SpectrumAvailability {
                        band_name: band.name.clone(),
                        available: true,
                        observed_power_dbm: f64::NEG_INFINITY,
                    },
                }
            })
            .collect()
    }

    /// Find unused frequency bands at a given location.
    ///
    /// Returns the names of bands whose average power is below `threshold_dbm`.
    pub fn find_unused_bands(
        &self,
        x: f64,
        y: f64,
        current_time: f64,
        threshold_dbm: f64,
    ) -> Vec<String> {
        self.spectrum_availability(x, y, current_time, threshold_dbm)
            .into_iter()
            .filter(|sa| sa.available)
            .map(|sa| sa.band_name)
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_mapper() -> RfEnvironmentMapper {
        let mut mapper = RfEnvironmentMapper::new(0.0, 100.0, 0.0, 100.0, 10.0, 60.0);
        mapper.add_band(BandDefinition::new("ISM 900", 902.0e6, 928.0e6));
        mapper.add_band(BandDefinition::new("ISM 2400", 2400.0e6, 2483.5e6));
        mapper
    }

    #[test]
    fn test_grid_dimensions() {
        let mapper = make_mapper();
        assert_eq!(mapper.cols(), 10);
        assert_eq!(mapper.rows(), 10);
    }

    #[test]
    fn test_band_definition_contains() {
        let band = BandDefinition::new("Test", 100.0, 200.0);
        assert!(band.contains(100.0));
        assert!(band.contains(150.0));
        assert!(band.contains(200.0));
        assert!(!band.contains(99.9));
        assert!(!band.contains(200.1));
    }

    #[test]
    fn test_ingest_single_measurement() {
        let mut mapper = make_mapper();
        let m = RfMeasurement {
            frequency: 915.0e6,
            power_dbm: -60.0,
            location: (15.0, 25.0),
            timestamp: 0.0,
        };
        mapper.ingest(m);
        assert_eq!(mapper.total_measurements(), 1);
    }

    #[test]
    fn test_ingest_out_of_bounds() {
        let mut mapper = make_mapper();
        let m = RfMeasurement {
            frequency: 915.0e6,
            power_dbm: -60.0,
            location: (150.0, 50.0), // out of bounds
            timestamp: 0.0,
        };
        mapper.ingest(m);
        assert_eq!(mapper.total_measurements(), 0);
    }

    #[test]
    fn test_ingest_batch() {
        let mut mapper = make_mapper();
        let measurements: Vec<RfMeasurement> = (0..5)
            .map(|i| RfMeasurement {
                frequency: 915.0e6,
                power_dbm: -60.0 + i as f64,
                location: (15.0, 25.0),
                timestamp: i as f64,
            })
            .collect();
        mapper.ingest_batch(&measurements);
        assert_eq!(mapper.total_measurements(), 5);
    }

    #[test]
    fn test_cell_stats_basic() {
        let mut mapper = make_mapper();
        mapper.ingest(RfMeasurement {
            frequency: 915.0e6,
            power_dbm: -50.0,
            location: (15.0, 25.0),
            timestamp: 1.0,
        });
        // Cell at col=1, row=2
        let stats = mapper.cell_stats(1, 2, "ISM 900", 1.0).unwrap();
        assert_eq!(stats.measurement_count, 1);
        // At t=1.0 with measurement at t=1.0, weight=1.0, so average == measurement
        assert!((stats.average_power_dbm - (-50.0)).abs() < 0.1);
    }

    #[test]
    fn test_cell_stats_wrong_band() {
        let mut mapper = make_mapper();
        mapper.ingest(RfMeasurement {
            frequency: 915.0e6,
            power_dbm: -50.0,
            location: (15.0, 25.0),
            timestamp: 1.0,
        });
        // Query ISM 2400 band - should find nothing for a 915 MHz measurement
        let stats = mapper.cell_stats(1, 2, "ISM 2400", 1.0);
        assert!(stats.is_none());
    }

    #[test]
    fn test_cell_stats_nonexistent_band() {
        let mapper = make_mapper();
        let stats = mapper.cell_stats(0, 0, "NoSuchBand", 1.0);
        assert!(stats.is_none());
    }

    #[test]
    fn test_cell_stats_out_of_range() {
        let mapper = make_mapper();
        let stats = mapper.cell_stats(999, 999, "ISM 900", 1.0);
        assert!(stats.is_none());
    }

    #[test]
    fn test_temporal_decay() {
        let mut mapper = make_mapper();
        // Insert measurement at t=0
        mapper.ingest(RfMeasurement {
            frequency: 915.0e6,
            power_dbm: -50.0,
            location: (5.0, 5.0),
            timestamp: 0.0,
        });

        // At t=0, weight=1.0
        let stats_t0 = mapper.cell_stats(0, 0, "ISM 900", 0.0).unwrap();
        let power_t0 = stats_t0.average_power_dbm;

        // At t=60 (one half-life), weight=0.5 → weighted average in linear should be halved
        let stats_t60 = mapper.cell_stats(0, 0, "ISM 900", 60.0).unwrap();
        let power_t60 = stats_t60.average_power_dbm;

        // Power should be lower at t=60 because of decay weighting
        // average_power = 10*log10(w * linear) / w = same if only one sample
        // Actually with one sample: avg = w*P_lin / w = P_lin, so average doesn't change
        // But peak_power does change because it's computed as power + 10*log10(w)
        // With one measurement, weighted average (linear) doesn't change since it's
        // (w * P_lin) / w = P_lin. Let me verify:
        assert!((stats_t0.average_power_dbm - stats_t60.average_power_dbm).abs() < 0.01);
        // But total_weight should decay
        assert!(stats_t60.total_weight < stats_t0.total_weight);
        assert!((stats_t60.total_weight - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_temporal_weight_calculation() {
        let mapper = RfEnvironmentMapper::new(0.0, 10.0, 0.0, 10.0, 1.0, 10.0);
        // At same time, weight = 1
        assert!((mapper.temporal_weight(5.0, 5.0) - 1.0).abs() < 1e-10);
        // After one half-life, weight = 0.5
        assert!((mapper.temporal_weight(0.0, 10.0) - 0.5).abs() < 1e-10);
        // After two half-lives, weight = 0.25
        assert!((mapper.temporal_weight(0.0, 20.0) - 0.25).abs() < 1e-10);
        // Future measurement: weight = 1
        assert!((mapper.temporal_weight(10.0, 5.0) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_prune_old_measurements() {
        let mut mapper = make_mapper();
        mapper.ingest(RfMeasurement {
            frequency: 915.0e6,
            power_dbm: -50.0,
            location: (5.0, 5.0),
            timestamp: 0.0,
        });
        mapper.ingest(RfMeasurement {
            frequency: 915.0e6,
            power_dbm: -55.0,
            location: (5.0, 5.0),
            timestamp: 10.0,
        });
        assert_eq!(mapper.total_measurements(), 2);

        // Prune measurements older than 5 seconds relative to t=12
        mapper.prune(12.0, 5.0);
        assert_eq!(mapper.total_measurements(), 1);
    }

    #[test]
    fn test_heat_map_basic() {
        let mut mapper = make_mapper();
        mapper.ingest(RfMeasurement {
            frequency: 915.0e6,
            power_dbm: -40.0,
            location: (5.0, 5.0),
            timestamp: 1.0,
        });
        mapper.ingest(RfMeasurement {
            frequency: 915.0e6,
            power_dbm: -70.0,
            location: (55.0, 55.0),
            timestamp: 1.0,
        });

        let hmap = mapper.heat_map("ISM 900", 1.0).unwrap();
        assert_eq!(hmap.cols, 10);
        assert_eq!(hmap.rows, 10);

        // Cell (0,0) should have data
        let val_00 = hmap.get(0, 0);
        assert!(val_00 > f64::NEG_INFINITY);
        assert!((val_00 - (-40.0)).abs() < 0.1);

        // Cell (5,5) should have data
        let val_55 = hmap.get(5, 5);
        assert!((val_55 - (-70.0)).abs() < 0.1);

        // An empty cell
        let val_99 = hmap.get(9, 9);
        assert!(val_99 == f64::NEG_INFINITY);
    }

    #[test]
    fn test_heat_map_nonexistent_band() {
        let mapper = make_mapper();
        let result = mapper.heat_map("NoSuchBand", 0.0);
        assert!(result.is_none());
    }

    #[test]
    fn test_locate_interference_single_source() {
        let mut mapper = make_mapper();
        // Place a strong signal in one cell surrounded by weaker signals
        mapper.ingest(RfMeasurement {
            frequency: 915.0e6,
            power_dbm: -30.0,
            location: (45.0, 45.0), // cell (4,4)
            timestamp: 1.0,
        });
        // Weaker neighbors
        for (x, y) in &[(35.0, 45.0), (55.0, 45.0), (45.0, 35.0), (45.0, 55.0)] {
            mapper.ingest(RfMeasurement {
                frequency: 915.0e6,
                power_dbm: -60.0,
                location: (*x, *y),
                timestamp: 1.0,
            });
        }

        let sources = mapper.locate_interference("ISM 900", 1.0, -80.0);
        // The strong signal at (4,4) should be detected as a peak
        assert!(!sources.is_empty());
        let strongest = &sources[0];
        assert_eq!(strongest.col, 4);
        assert_eq!(strongest.row, 4);
        assert!(strongest.power_dbm > -40.0);
    }

    #[test]
    fn test_locate_interference_no_sources() {
        let mapper = make_mapper();
        let sources = mapper.locate_interference("ISM 900", 0.0, -80.0);
        assert!(sources.is_empty());
    }

    #[test]
    fn test_spectrum_availability_all_available() {
        let mapper = make_mapper();
        let avail = mapper.spectrum_availability(50.0, 50.0, 0.0, -80.0);
        assert_eq!(avail.len(), 2);
        assert!(avail.iter().all(|a| a.available));
    }

    #[test]
    fn test_spectrum_availability_occupied() {
        let mut mapper = make_mapper();
        mapper.ingest(RfMeasurement {
            frequency: 915.0e6,
            power_dbm: -40.0,
            location: (55.0, 55.0),
            timestamp: 1.0,
        });

        let avail = mapper.spectrum_availability(55.0, 55.0, 1.0, -50.0);
        let ism900 = avail.iter().find(|a| a.band_name == "ISM 900").unwrap();
        assert!(!ism900.available);
        // ISM 2400 should still be available (no data)
        let ism2400 = avail.iter().find(|a| a.band_name == "ISM 2400").unwrap();
        assert!(ism2400.available);
    }

    #[test]
    fn test_find_unused_bands() {
        let mut mapper = make_mapper();
        mapper.ingest(RfMeasurement {
            frequency: 915.0e6,
            power_dbm: -40.0,
            location: (55.0, 55.0),
            timestamp: 1.0,
        });

        let unused = mapper.find_unused_bands(55.0, 55.0, 1.0, -50.0);
        assert_eq!(unused.len(), 1);
        assert_eq!(unused[0], "ISM 2400");
    }

    #[test]
    fn test_spectrum_availability_out_of_bounds() {
        let mapper = make_mapper();
        let avail = mapper.spectrum_availability(999.0, 999.0, 0.0, -80.0);
        assert!(avail.iter().all(|a| a.available));
    }

    #[test]
    fn test_duty_cycle_calculation() {
        let mut mapper = make_mapper();
        mapper.set_duty_cycle_threshold(-55.0);
        assert!((mapper.duty_cycle_threshold() - (-55.0)).abs() < 1e-10);

        // 3 measurements above threshold, 2 below
        for (i, power) in [-50.0, -60.0, -45.0, -70.0, -52.0].iter().enumerate() {
            mapper.ingest(RfMeasurement {
                frequency: 915.0e6,
                power_dbm: *power,
                location: (5.0, 5.0),
                timestamp: i as f64,
            });
        }

        let stats = mapper.cell_stats(0, 0, "ISM 900", 4.0).unwrap();
        assert_eq!(stats.measurement_count, 5);
        // 3 of 5 measurements are above -55 dBm: -50, -45, -52
        // With temporal decay (half_life=60s), weights differ slightly but all
        // measurements are within a few seconds, so weights are ~1.0
        assert!(stats.duty_cycle > 0.5);
        assert!(stats.duty_cycle < 0.7);
    }

    #[test]
    fn test_cell_center_coordinates() {
        let mapper = make_mapper();
        let (cx, cy) = mapper.cell_center(0, 0);
        assert!((cx - 5.0).abs() < 1e-10);
        assert!((cy - 5.0).abs() < 1e-10);

        let (cx, cy) = mapper.cell_center(9, 9);
        assert!((cx - 95.0).abs() < 1e-10);
        assert!((cy - 95.0).abs() < 1e-10);
    }

    #[test]
    fn test_multiple_bands_in_same_cell() {
        let mut mapper = make_mapper();
        mapper.ingest(RfMeasurement {
            frequency: 915.0e6,
            power_dbm: -50.0,
            location: (5.0, 5.0),
            timestamp: 1.0,
        });
        mapper.ingest(RfMeasurement {
            frequency: 2450.0e6,
            power_dbm: -65.0,
            location: (5.0, 5.0),
            timestamp: 1.0,
        });

        let stats_900 = mapper.cell_stats(0, 0, "ISM 900", 1.0).unwrap();
        assert_eq!(stats_900.measurement_count, 1);
        assert!((stats_900.average_power_dbm - (-50.0)).abs() < 0.1);

        let stats_2400 = mapper.cell_stats(0, 0, "ISM 2400", 1.0).unwrap();
        assert_eq!(stats_2400.measurement_count, 1);
        assert!((stats_2400.average_power_dbm - (-65.0)).abs() < 0.1);
    }
}

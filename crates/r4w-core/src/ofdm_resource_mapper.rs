//! OFDM Subcarrier Resource Mapping and Demapping
//!
//! Implements LTE/5G-style OFDM resource element mapping and extraction.
//! A [`ResourceGrid`] holds complex-valued resource elements indexed by
//! subcarrier and OFDM symbol. A [`ResourceMapper`] assigns subcarrier
//! types (data, pilot, guard, DC null, reference) and provides methods to
//! map data symbols into data subcarriers and demap them back out.
//!
//! Factory functions [`lte_resource_mapper`] and [`wifi_resource_mapper`]
//! produce standard allocations for LTE and 802.11a/g systems.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ofdm_resource_mapper::{ResourceGrid, ResourceMapper, wifi_resource_mapper};
//!
//! let mapper = wifi_resource_mapper();
//! let mut grid = ResourceGrid::new(64, 1);
//!
//! // Create 52 data symbols (802.11a/g has 52 data subcarriers)
//! let data: Vec<(f64, f64)> = (0..52).map(|i| (i as f64, 0.0)).collect();
//! mapper.map_data(&data, &mut grid, 0);
//!
//! let recovered = mapper.demap_data(&grid, 0);
//! assert_eq!(recovered.len(), 52);
//! assert_eq!(recovered[0], (0.0, 0.0));
//! ```

/// Complex-valued resource grid for OFDM subcarriers x symbols.
///
/// Elements are stored in row-major order: subcarrier index varies fastest.
#[derive(Debug, Clone)]
pub struct ResourceGrid {
    /// Number of subcarriers (frequency axis).
    pub num_subcarriers: usize,
    /// Number of OFDM symbols (time axis).
    pub num_symbols: usize,
    /// Flat storage: `grid[sym * num_subcarriers + sc]`.
    grid: Vec<(f64, f64)>,
}

impl ResourceGrid {
    /// Create a new resource grid initialised to (0.0, 0.0).
    pub fn new(num_subcarriers: usize, num_symbols: usize) -> Self {
        Self {
            num_subcarriers,
            num_symbols,
            grid: vec![(0.0, 0.0); num_subcarriers * num_symbols],
        }
    }

    /// Set the resource element at subcarrier `sc` and symbol `sym`.
    ///
    /// # Panics
    ///
    /// Panics if `sc >= num_subcarriers` or `sym >= num_symbols`.
    pub fn set(&mut self, sc: usize, sym: usize, val: (f64, f64)) {
        assert!(sc < self.num_subcarriers, "subcarrier index out of range");
        assert!(sym < self.num_symbols, "symbol index out of range");
        self.grid[sym * self.num_subcarriers + sc] = val;
    }

    /// Get the resource element at subcarrier `sc` and symbol `sym`.
    ///
    /// # Panics
    ///
    /// Panics if `sc >= num_subcarriers` or `sym >= num_symbols`.
    pub fn get(&self, sc: usize, sym: usize) -> (f64, f64) {
        assert!(sc < self.num_subcarriers, "subcarrier index out of range");
        assert!(sym < self.num_symbols, "symbol index out of range");
        self.grid[sym * self.num_subcarriers + sc]
    }
}

/// Classification of an OFDM subcarrier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubcarrierType {
    /// Carries user data.
    Data,
    /// Known pilot symbol for channel estimation / tracking.
    Pilot,
    /// Guard band (zero-energy, band edge protection).
    Guard,
    /// DC null subcarrier (centre of spectrum).
    DcNull,
    /// Cell-specific or slot-specific reference signal.
    Reference,
}

/// Maps subcarrier types and performs data mapping / demapping.
///
/// The allocation map is a single vector of [`SubcarrierType`] applied
/// identically to every OFDM symbol unless otherwise configured.
#[derive(Debug, Clone)]
pub struct ResourceMapper {
    /// Number of subcarriers.
    num_subcarriers: usize,
    /// Per-subcarrier type assignment.
    allocation: Vec<SubcarrierType>,
}

impl ResourceMapper {
    /// Create a new mapper with all subcarriers initially set to [`SubcarrierType::Data`].
    pub fn new(num_subcarriers: usize) -> Self {
        Self {
            num_subcarriers,
            allocation: vec![SubcarrierType::Data; num_subcarriers],
        }
    }

    /// Mark the outermost subcarriers as guard bands.
    ///
    /// `left` subcarriers at the low-index end and `right` subcarriers at
    /// the high-index end are set to [`SubcarrierType::Guard`].
    pub fn set_guard_bands(&mut self, left: usize, right: usize) {
        for i in 0..left.min(self.num_subcarriers) {
            self.allocation[i] = SubcarrierType::Guard;
        }
        for i in 0..right.min(self.num_subcarriers) {
            self.allocation[self.num_subcarriers - 1 - i] = SubcarrierType::Guard;
        }
    }

    /// Enable or disable the DC null subcarrier at the centre index
    /// (`num_subcarriers / 2`).
    pub fn set_dc_null(&mut self, enable: bool) {
        let dc = self.num_subcarriers / 2;
        if enable {
            self.allocation[dc] = SubcarrierType::DcNull;
        } else if self.allocation[dc] == SubcarrierType::DcNull {
            self.allocation[dc] = SubcarrierType::Data;
        }
    }

    /// Set specific subcarrier indices as pilots.
    ///
    /// Indices that are out of range are silently ignored.
    pub fn set_pilot_positions(&mut self, positions: &[usize]) {
        for &pos in positions {
            if pos < self.num_subcarriers {
                self.allocation[pos] = SubcarrierType::Pilot;
            }
        }
    }

    /// Return a reference to the full allocation map.
    pub fn allocation(&self) -> &[SubcarrierType] {
        &self.allocation
    }

    /// Return the number of data subcarriers in the current allocation.
    pub fn num_data_subcarriers(&self) -> usize {
        self.allocation.iter().filter(|&&t| t == SubcarrierType::Data).count()
    }

    /// Map data symbols into the data subcarriers of `grid` at OFDM symbol
    /// index `symbol_idx`.
    ///
    /// Data symbols are placed into subcarriers marked [`SubcarrierType::Data`]
    /// in ascending index order. If there are fewer data symbols than data
    /// subcarriers, remaining data subcarriers are left unchanged. Extra data
    /// symbols beyond the number of data subcarriers are ignored.
    pub fn map_data(
        &self,
        data_symbols: &[(f64, f64)],
        grid: &mut ResourceGrid,
        symbol_idx: usize,
    ) {
        let mut data_iter = data_symbols.iter();
        for (sc, &sc_type) in self.allocation.iter().enumerate() {
            if sc_type == SubcarrierType::Data {
                if let Some(&val) = data_iter.next() {
                    grid.set(sc, symbol_idx, val);
                }
            }
        }
    }

    /// Extract data symbols from the data subcarriers of `grid` at OFDM
    /// symbol index `symbol_idx`.
    ///
    /// Returns a vector of `(f64, f64)` values in ascending subcarrier order,
    /// one per [`SubcarrierType::Data`] subcarrier.
    pub fn demap_data(&self, grid: &ResourceGrid, symbol_idx: usize) -> Vec<(f64, f64)> {
        self.allocation
            .iter()
            .enumerate()
            .filter(|(_, &t)| t == SubcarrierType::Data)
            .map(|(sc, _)| grid.get(sc, symbol_idx))
            .collect()
    }
}

/// Create an LTE-style resource mapper.
///
/// LTE uses 12 subcarriers per resource block (RB). The total number of
/// subcarriers is `num_rb * 12`. Guard bands are set to 1 subcarrier on
/// each side, the DC null is enabled at the centre, and pilot (CRS-like)
/// subcarriers are placed every 6th subcarrier starting at index 3
/// (skipping guard-band indices).
pub fn lte_resource_mapper(num_rb: usize) -> ResourceMapper {
    let num_sc = num_rb * 12;
    let mut mapper = ResourceMapper::new(num_sc);

    // Guard bands: 1 on each side
    mapper.set_guard_bands(1, 1);

    // DC null at centre
    mapper.set_dc_null(true);

    // Pilots every 6th subcarrier starting at index 3 (CRS-like pattern),
    // avoiding the guard-band edges.
    let pilots: Vec<usize> = (3..num_sc).step_by(6).collect();
    mapper.set_pilot_positions(&pilots);

    mapper
}

/// Create an IEEE 802.11a/g-style resource mapper.
///
/// 64-point FFT layout (52 data + 4 pilot + 7 guard + 1 DC null = 64):
/// - Subcarriers 0..3 (4 left guard) and 61..63 (3 right guard).
/// - Subcarrier 32 (DC null).
/// - Pilots at subcarriers 11, 25, 39, 53.
/// - Remaining 52 subcarriers carry data.
pub fn wifi_resource_mapper() -> ResourceMapper {
    let mut mapper = ResourceMapper::new(64);

    // Guard bands: 4 left + 3 right = 7 guard subcarriers
    mapper.set_guard_bands(4, 3);

    // DC null at index 32
    mapper.set_dc_null(true);

    // Pilot subcarriers at bins 11, 25, 39, 53
    mapper.set_pilot_positions(&[11, 25, 39, 53]);

    mapper
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_grid_construction_and_get_set() {
        let mut grid = ResourceGrid::new(12, 4);
        assert_eq!(grid.num_subcarriers, 12);
        assert_eq!(grid.num_symbols, 4);

        // Default value is (0, 0)
        assert_eq!(grid.get(0, 0), (0.0, 0.0));
        assert_eq!(grid.get(11, 3), (0.0, 0.0));

        // Set and retrieve
        grid.set(5, 2, (1.0, -1.0));
        assert_eq!(grid.get(5, 2), (1.0, -1.0));

        // Other cells remain zero
        assert_eq!(grid.get(5, 1), (0.0, 0.0));
        assert_eq!(grid.get(4, 2), (0.0, 0.0));
    }

    #[test]
    fn test_guard_band_configuration() {
        let mut mapper = ResourceMapper::new(16);
        mapper.set_guard_bands(3, 2);

        // Left guard: indices 0, 1, 2
        assert_eq!(mapper.allocation()[0], SubcarrierType::Guard);
        assert_eq!(mapper.allocation()[1], SubcarrierType::Guard);
        assert_eq!(mapper.allocation()[2], SubcarrierType::Guard);

        // First data subcarrier
        assert_eq!(mapper.allocation()[3], SubcarrierType::Data);

        // Right guard: indices 14, 15
        assert_eq!(mapper.allocation()[14], SubcarrierType::Guard);
        assert_eq!(mapper.allocation()[15], SubcarrierType::Guard);

        // Last data subcarrier before right guard
        assert_eq!(mapper.allocation()[13], SubcarrierType::Data);
    }

    #[test]
    fn test_dc_null_placement() {
        let mut mapper = ResourceMapper::new(64);
        let dc_idx = 32; // 64 / 2

        // Initially data
        assert_eq!(mapper.allocation()[dc_idx], SubcarrierType::Data);

        // Enable DC null
        mapper.set_dc_null(true);
        assert_eq!(mapper.allocation()[dc_idx], SubcarrierType::DcNull);

        // Disable DC null
        mapper.set_dc_null(false);
        assert_eq!(mapper.allocation()[dc_idx], SubcarrierType::Data);
    }

    #[test]
    fn test_pilot_position_setting() {
        let mut mapper = ResourceMapper::new(32);
        mapper.set_pilot_positions(&[4, 12, 20, 28]);

        assert_eq!(mapper.allocation()[4], SubcarrierType::Pilot);
        assert_eq!(mapper.allocation()[12], SubcarrierType::Pilot);
        assert_eq!(mapper.allocation()[20], SubcarrierType::Pilot);
        assert_eq!(mapper.allocation()[28], SubcarrierType::Pilot);

        // Non-pilot subcarriers are still data
        assert_eq!(mapper.allocation()[0], SubcarrierType::Data);
        assert_eq!(mapper.allocation()[5], SubcarrierType::Data);

        // Out-of-range pilot is silently ignored
        mapper.set_pilot_positions(&[100]);
        // No panic, allocation unchanged
        assert_eq!(mapper.allocation().len(), 32);
    }

    #[test]
    fn test_data_mapping_fills_data_subcarriers_only() {
        let mut mapper = ResourceMapper::new(8);
        mapper.set_guard_bands(1, 1); // indices 0, 7 are guard
        mapper.set_dc_null(true); // index 4 is DC null
        mapper.set_pilot_positions(&[2]); // index 2 is pilot

        // Data subcarriers: 1, 3, 5, 6
        let data = vec![(1.0, 0.1), (3.0, 0.3), (5.0, 0.5), (6.0, 0.6)];
        let mut grid = ResourceGrid::new(8, 1);
        mapper.map_data(&data, &mut grid, 0);

        // Data subcarriers should have the mapped values
        assert_eq!(grid.get(1, 0), (1.0, 0.1));
        assert_eq!(grid.get(3, 0), (3.0, 0.3));
        assert_eq!(grid.get(5, 0), (5.0, 0.5));
        assert_eq!(grid.get(6, 0), (6.0, 0.6));

        // Non-data subcarriers should remain zero
        assert_eq!(grid.get(0, 0), (0.0, 0.0)); // guard
        assert_eq!(grid.get(2, 0), (0.0, 0.0)); // pilot
        assert_eq!(grid.get(4, 0), (0.0, 0.0)); // DC null
        assert_eq!(grid.get(7, 0), (0.0, 0.0)); // guard
    }

    #[test]
    fn test_data_demapping_extracts_correct_symbols() {
        let mut mapper = ResourceMapper::new(8);
        mapper.set_guard_bands(1, 1);
        mapper.set_pilot_positions(&[3]);
        // Data subcarriers: 1, 2, 4, 5, 6

        let mut grid = ResourceGrid::new(8, 1);
        // Fill entire symbol row with distinct values
        for sc in 0..8 {
            grid.set(sc, 0, (sc as f64, sc as f64 * 10.0));
        }

        let extracted = mapper.demap_data(&grid, 0);
        assert_eq!(extracted.len(), 5);
        assert_eq!(extracted[0], (1.0, 10.0)); // sc 1
        assert_eq!(extracted[1], (2.0, 20.0)); // sc 2
        assert_eq!(extracted[2], (4.0, 40.0)); // sc 4
        assert_eq!(extracted[3], (5.0, 50.0)); // sc 5
        assert_eq!(extracted[4], (6.0, 60.0)); // sc 6
    }

    #[test]
    fn test_roundtrip_map_demap_identity() {
        let mut mapper = ResourceMapper::new(16);
        mapper.set_guard_bands(2, 2);
        mapper.set_dc_null(true); // index 8
        mapper.set_pilot_positions(&[4, 12]);

        let num_data = mapper.num_data_subcarriers();
        let data: Vec<(f64, f64)> =
            (0..num_data).map(|i| (i as f64 + 0.5, -(i as f64))).collect();

        let mut grid = ResourceGrid::new(16, 1);
        mapper.map_data(&data, &mut grid, 0);
        let recovered = mapper.demap_data(&grid, 0);

        assert_eq!(recovered.len(), data.len());
        for (orig, recv) in data.iter().zip(recovered.iter()) {
            assert!((orig.0 - recv.0).abs() < 1e-12);
            assert!((orig.1 - recv.1).abs() < 1e-12);
        }
    }

    #[test]
    fn test_lte_factory_correct_subcarrier_count() {
        let mapper = lte_resource_mapper(6); // 6 RBs = 72 subcarriers
        assert_eq!(mapper.allocation().len(), 72);

        // Verify guard bands (1 on each side)
        assert_eq!(mapper.allocation()[0], SubcarrierType::Guard);
        assert_eq!(mapper.allocation()[71], SubcarrierType::Guard);

        // Verify DC null at centre
        assert_eq!(mapper.allocation()[36], SubcarrierType::DcNull);

        // Verify pilots exist every 6th subcarrier starting at 3
        assert_eq!(mapper.allocation()[3], SubcarrierType::Pilot);
        assert_eq!(mapper.allocation()[9], SubcarrierType::Pilot);
        assert_eq!(mapper.allocation()[15], SubcarrierType::Pilot);
    }

    #[test]
    fn test_wifi_factory_correct_layout() {
        let mapper = wifi_resource_mapper();
        assert_eq!(mapper.allocation().len(), 64);

        let num_data = mapper
            .allocation()
            .iter()
            .filter(|&&t| t == SubcarrierType::Data)
            .count();
        let num_pilot = mapper
            .allocation()
            .iter()
            .filter(|&&t| t == SubcarrierType::Pilot)
            .count();

        assert_eq!(num_data, 52, "802.11a/g should have 52 data subcarriers");
        assert_eq!(num_pilot, 4, "802.11a/g should have 4 pilot subcarriers");

        // Pilot positions
        assert_eq!(mapper.allocation()[11], SubcarrierType::Pilot);
        assert_eq!(mapper.allocation()[25], SubcarrierType::Pilot);
        assert_eq!(mapper.allocation()[39], SubcarrierType::Pilot);
        assert_eq!(mapper.allocation()[53], SubcarrierType::Pilot);

        // DC null
        assert_eq!(mapper.allocation()[32], SubcarrierType::DcNull);
    }

    #[test]
    fn test_full_grid_with_mixed_types() {
        // Build a small grid with every subcarrier type represented.
        let mut mapper = ResourceMapper::new(10);
        mapper.set_guard_bands(1, 1);   // indices 0 and 9
        mapper.set_dc_null(true);       // index 5
        mapper.set_pilot_positions(&[3, 7]);

        // Manually set one subcarrier as Reference to exercise that variant.
        // (Direct allocation mutation for test coverage.)
        mapper.allocation[2] = SubcarrierType::Reference;

        // Allocation should be: Guard Data Ref Data Pilot Data DcNull Data Pilot Guard
        // (indices:              0     1    2   3     4      5    6     7    8     9)
        // Wait -- let's re-check: after guard(1,1): 0=Guard, 9=Guard, rest Data
        // set_dc_null: 5 = DcNull
        // set_pilot_positions [3,7]: 3=Pilot, 7=Pilot
        // manual: 2=Reference
        // So: 0=Guard, 1=Data, 2=Ref, 3=Pilot, 4=Data, 5=DcNull, 6=Data, 7=Pilot, 8=Data, 9=Guard
        assert_eq!(mapper.allocation()[0], SubcarrierType::Guard);
        assert_eq!(mapper.allocation()[1], SubcarrierType::Data);
        assert_eq!(mapper.allocation()[2], SubcarrierType::Reference);
        assert_eq!(mapper.allocation()[3], SubcarrierType::Pilot);
        assert_eq!(mapper.allocation()[4], SubcarrierType::Data);
        assert_eq!(mapper.allocation()[5], SubcarrierType::DcNull);
        assert_eq!(mapper.allocation()[6], SubcarrierType::Data);
        assert_eq!(mapper.allocation()[7], SubcarrierType::Pilot);
        assert_eq!(mapper.allocation()[8], SubcarrierType::Data);
        assert_eq!(mapper.allocation()[9], SubcarrierType::Guard);

        // Map 4 data symbols into the 4 data subcarriers
        let data = vec![(10.0, 1.0), (40.0, 4.0), (60.0, 6.0), (80.0, 8.0)];
        let mut grid = ResourceGrid::new(10, 2);
        mapper.map_data(&data, &mut grid, 1);

        // Verify mapped data at symbol 1
        let recovered = mapper.demap_data(&grid, 1);
        assert_eq!(recovered.len(), 4);
        assert_eq!(recovered[0], (10.0, 1.0));
        assert_eq!(recovered[1], (40.0, 4.0));
        assert_eq!(recovered[2], (60.0, 6.0));
        assert_eq!(recovered[3], (80.0, 8.0));

        // Symbol 0 should still be all zeros
        let sym0 = mapper.demap_data(&grid, 0);
        assert!(sym0.iter().all(|&v| v == (0.0, 0.0)));
    }
}

//! # 5G NR Resource Grid Mapper
//!
//! Maps data and reference signals onto 5G NR resource grids per 3GPP TS 38.211.
//!
//! The resource grid is a two-dimensional structure indexed by subcarrier (frequency)
//! and OFDM symbol (time). Each cell is a Resource Element (RE) that can carry
//! data, demodulation reference signals (DMRS), phase-tracking reference signals
//! (PTRS), CSI-RS, guard bands, or DC null.
//!
//! ## Numerology
//!
//! | μ | SCS (kHz) | Slot duration (ms) | Symbols/slot |
//! |---|-----------|--------------------:|-------------:|
//! | 0 |  15       | 1.0                | 14           |
//! | 1 |  30       | 0.5                | 14           |
//! | 2 |  60       | 0.25               | 14           |
//! | 3 | 120       | 0.125              | 14           |
//! | 4 | 240       | 0.0625             | 14           |
//!
//! # Example
//!
//! ```
//! use r4w_core::nr_resource_grid_mapper::{
//!     NrResourceGridMapper, NrConfig, SlotConfig, CyclicPrefixType,
//! };
//!
//! let config = NrConfig {
//!     numerology: 1,       // 30 kHz SCS
//!     num_prbs: 4,         // 4 resource blocks = 48 subcarriers
//!     num_symbols: 14,     // normal slot
//! };
//! let slot = SlotConfig {
//!     slot_number: 0,
//!     frame_number: 0,
//!     cp_type: CyclicPrefixType::Normal,
//! };
//! let mut mapper = NrResourceGridMapper::new(config, slot);
//!
//! // Insert DMRS (Type A, symbols 2 and 3)
//! mapper.insert_dmrs();
//!
//! // Map PDSCH data symbols
//! let data: Vec<(f64, f64)> = (0..100).map(|i| {
//!     let angle = i as f64 * std::f64::consts::FRAC_PI_4;
//!     (angle.cos(), angle.sin())
//! }).collect();
//! let mapped = mapper.map_pdsch(&data);
//!
//! // Retrieve the full grid
//! let grid = mapper.get_grid();
//! assert_eq!(grid.len(), 48);       // 4 PRBs × 12 subcarriers
//! assert_eq!(grid[0].len(), 14);    // 14 symbols per slot
//!
//! // Extract only data REs (skipping DMRS/guard)
//! let extracted = mapper.extract_data();
//! assert_eq!(extracted.len(), mapped);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Kind of content carried by a resource element.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResourceType {
    /// User data (PDSCH / PUSCH).
    Data,
    /// Demodulation reference signal — Type A (front-loaded).
    DmrsTypeA,
    /// Demodulation reference signal — Type B (additional positions).
    DmrsTypeB,
    /// Phase-tracking reference signal.
    PtrsRef,
    /// CSI reference signal.
    CsiRef,
    /// Guard subcarrier (unused).
    Guard,
    /// DC subcarrier null.
    DcNull,
}

/// A single resource element in the grid.
#[derive(Debug, Clone, Copy)]
pub struct ResourceElement {
    /// Subcarrier index (0-based).
    pub subcarrier: usize,
    /// OFDM symbol index within the slot (0-based).
    pub symbol: usize,
    /// Complex value (re, im).
    pub value: (f64, f64),
    /// Type of content this RE carries.
    pub re_type: ResourceType,
}

/// Cyclic prefix type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CyclicPrefixType {
    /// Normal CP — 14 symbols per slot.
    Normal,
    /// Extended CP — 12 symbols per slot (only for μ = 2).
    Extended,
}

/// Per-slot configuration.
#[derive(Debug, Clone, Copy)]
pub struct SlotConfig {
    /// Slot number within the frame (0-based).
    pub slot_number: u16,
    /// System frame number (0–1023).
    pub frame_number: u16,
    /// Cyclic prefix type.
    pub cp_type: CyclicPrefixType,
}

/// Numerology and bandwidth configuration.
#[derive(Debug, Clone, Copy)]
pub struct NrConfig {
    /// Numerology index μ ∈ {0, 1, 2, 3, 4}.
    pub numerology: u8,
    /// Number of physical resource blocks (each PRB = 12 subcarriers).
    pub num_prbs: usize,
    /// Number of OFDM symbols per slot (typically 14 for normal CP).
    pub num_symbols: usize,
}

// ---------------------------------------------------------------------------
// Mapper
// ---------------------------------------------------------------------------

/// Maps data and reference signals onto a 5G NR resource grid.
///
/// The grid is stored as a flat `Vec` of [`ResourceElement`]s indexed by
/// `(subcarrier, symbol)`.  Helper methods populate DMRS, PTRS, and PDSCH
/// data while respecting reserved positions.
pub struct NrResourceGridMapper {
    config: NrConfig,
    slot: SlotConfig,
    /// 2-D grid stored row-major: grid\[subcarrier\]\[symbol\].
    grid: Vec<Vec<ResourceElement>>,
}

impl NrResourceGridMapper {
    // -- construction -------------------------------------------------------

    /// Create a new mapper with all REs initialised to `Guard` at zero.
    pub fn new(config: NrConfig, slot: SlotConfig) -> Self {
        assert!(
            config.numerology <= 4,
            "Numerology must be 0..=4, got {}",
            config.numerology
        );
        assert!(config.num_prbs > 0, "num_prbs must be > 0");
        assert!(config.num_symbols > 0, "num_symbols must be > 0");

        let num_subcarriers = config.num_prbs * 12;
        let mut grid = Vec::with_capacity(num_subcarriers);
        for sc in 0..num_subcarriers {
            let mut row = Vec::with_capacity(config.num_symbols);
            for sym in 0..config.num_symbols {
                row.push(ResourceElement {
                    subcarrier: sc,
                    symbol: sym,
                    value: (0.0, 0.0),
                    re_type: ResourceType::Guard,
                });
            }
            grid.push(row);
        }

        Self { config, slot, grid }
    }

    // -- queries ------------------------------------------------------------

    /// Total number of subcarriers in the grid.
    #[inline]
    pub fn num_subcarriers(&self) -> usize {
        self.config.num_prbs * 12
    }

    /// Subcarrier spacing in kHz derived from the numerology.
    ///
    /// SCS = 15 × 2^μ  kHz.
    #[inline]
    pub fn subcarrier_spacing_khz(&self) -> f64 {
        15.0 * (1u32 << self.config.numerology) as f64
    }

    /// Slot duration in milliseconds.
    ///
    /// T_slot = 1.0 / 2^μ  ms.
    #[inline]
    pub fn slot_duration_ms(&self) -> f64 {
        1.0 / (1u32 << self.config.numerology) as f64
    }

    /// Return a reference to the 2-D grid (subcarriers × symbols).
    pub fn get_grid(&self) -> &Vec<Vec<ResourceElement>> {
        &self.grid
    }

    /// Return the NR configuration.
    pub fn config(&self) -> &NrConfig {
        &self.config
    }

    /// Return the slot configuration.
    pub fn slot_config(&self) -> &SlotConfig {
        &self.slot
    }

    // -- DMRS ---------------------------------------------------------------

    /// Insert demodulation reference signals (DMRS Type A, single-symbol,
    /// configuration type 1).
    ///
    /// Per TS 38.211 Table 7.4.1.1.2-3, the front-loaded DMRS position for
    /// mapping type A with `l_0 = 2` occupies OFDM symbol 2 (and optionally
    /// symbol 3 as additional position).
    ///
    /// Within a DMRS symbol the reference is placed on every other subcarrier
    /// (comb-2) starting at subcarrier offset 0.
    ///
    /// The DMRS sequence is a QPSK-like Gold-sequence approximation seeded
    /// from the slot and frame numbers.
    pub fn insert_dmrs(&mut self) {
        let dmrs_symbols: Vec<usize> = self.dmrs_symbol_indices();
        let num_sc = self.num_subcarriers();

        for &sym in &dmrs_symbols {
            if sym >= self.config.num_symbols {
                continue;
            }
            for sc in (0..num_sc).step_by(2) {
                let (re, im) = self.dmrs_sequence_value(sc, sym);
                self.grid[sc][sym] = ResourceElement {
                    subcarrier: sc,
                    symbol: sym,
                    value: (re, im),
                    re_type: ResourceType::DmrsTypeA,
                };
            }
        }
    }

    /// Symbol indices that carry DMRS (Type A, additional position 1).
    fn dmrs_symbol_indices(&self) -> Vec<usize> {
        // Front-loaded at symbol 2; one additional position at symbol 3.
        vec![2, 3]
    }

    /// Generate a single DMRS QPSK value from a deterministic seed.
    fn dmrs_sequence_value(&self, sc: usize, sym: usize) -> (f64, f64) {
        // Simplified Gold-sequence-like QPSK mapping.
        let seed = (self.slot.frame_number as usize)
            .wrapping_mul(131)
            .wrapping_add(self.slot.slot_number as usize)
            .wrapping_mul(97)
            .wrapping_add(sc)
            .wrapping_mul(53)
            .wrapping_add(sym);
        let phase = ((seed % 4) as f64) * PI / 2.0 + PI / 4.0;
        let scale = 1.0 / std::f64::consts::SQRT_2;
        (phase.cos() * scale, phase.sin() * scale)
    }

    // -- PTRS ---------------------------------------------------------------

    /// Insert phase-tracking reference signals.
    ///
    /// PTRS is inserted on a subset of subcarriers (every `freq_density`-th
    /// subcarrier, default 4) across all non-DMRS data symbols.  The value
    /// is a constant unit-power BPSK symbol (+1/sqrt(2), +1/sqrt(2)).
    pub fn insert_ptrs(&mut self) {
        self.insert_ptrs_with_density(4);
    }

    /// Insert PTRS with a configurable frequency-domain density.
    pub fn insert_ptrs_with_density(&mut self, freq_density: usize) {
        let freq_density = if freq_density == 0 { 4 } else { freq_density };
        let num_sc = self.num_subcarriers();
        let scale = 1.0 / std::f64::consts::SQRT_2;

        for sym in 0..self.config.num_symbols {
            // Skip symbols already occupied by DMRS.
            // Check the first even subcarrier (sc=0) which would be DMRS if present.
            let is_dmrs_sym = self.grid[0][sym].re_type == ResourceType::DmrsTypeA
                || self.grid[0][sym].re_type == ResourceType::DmrsTypeB;
            if is_dmrs_sym {
                continue;
            }
            for sc in (0..num_sc).step_by(freq_density) {
                self.grid[sc][sym] = ResourceElement {
                    subcarrier: sc,
                    symbol: sym,
                    value: (scale, scale),
                    re_type: ResourceType::PtrsRef,
                };
            }
        }
    }

    // -- PDSCH --------------------------------------------------------------

    /// Map data symbols to available (Guard) resource elements in the grid.
    ///
    /// Returns the number of data symbols actually mapped.  If the supplied
    /// slice is shorter than the number of available REs, the remaining REs
    /// stay as `Guard`.  If longer, extra symbols are silently ignored.
    pub fn map_pdsch(&mut self, data: &[(f64, f64)]) -> usize {
        let mut idx = 0;
        for sym in 0..self.config.num_symbols {
            for sc in 0..self.num_subcarriers() {
                if idx >= data.len() {
                    return idx;
                }
                if self.grid[sc][sym].re_type == ResourceType::Guard {
                    self.grid[sc][sym] = ResourceElement {
                        subcarrier: sc,
                        symbol: sym,
                        value: data[idx],
                        re_type: ResourceType::Data,
                    };
                    idx += 1;
                }
            }
        }
        idx
    }

    // -- extraction ---------------------------------------------------------

    /// Extract data resource elements from the grid, in the same order that
    /// [`map_pdsch`](Self::map_pdsch) would have written them.
    pub fn extract_data(&self) -> Vec<(f64, f64)> {
        let mut out = Vec::new();
        for sym in 0..self.config.num_symbols {
            for sc in 0..self.num_subcarriers() {
                if self.grid[sc][sym].re_type == ResourceType::Data {
                    out.push(self.grid[sc][sym].value);
                }
            }
        }
        out
    }

    /// Extract DMRS resource elements from the grid.
    pub fn extract_dmrs(&self) -> Vec<ResourceElement> {
        let mut out = Vec::new();
        for sym in 0..self.config.num_symbols {
            for sc in 0..self.num_subcarriers() {
                let re = &self.grid[sc][sym];
                if re.re_type == ResourceType::DmrsTypeA
                    || re.re_type == ResourceType::DmrsTypeB
                {
                    out.push(*re);
                }
            }
        }
        out
    }

    /// Count how many REs of each [`ResourceType`] are in the grid.
    pub fn resource_counts(&self) -> [(ResourceType, usize); 7] {
        let mut counts = [0usize; 7];
        for sym in 0..self.config.num_symbols {
            for sc in 0..self.num_subcarriers() {
                let idx = match self.grid[sc][sym].re_type {
                    ResourceType::Data => 0,
                    ResourceType::DmrsTypeA => 1,
                    ResourceType::DmrsTypeB => 2,
                    ResourceType::PtrsRef => 3,
                    ResourceType::CsiRef => 4,
                    ResourceType::Guard => 5,
                    ResourceType::DcNull => 6,
                };
                counts[idx] += 1;
            }
        }
        [
            (ResourceType::Data, counts[0]),
            (ResourceType::DmrsTypeA, counts[1]),
            (ResourceType::DmrsTypeB, counts[2]),
            (ResourceType::PtrsRef, counts[3]),
            (ResourceType::CsiRef, counts[4]),
            (ResourceType::Guard, counts[5]),
            (ResourceType::DcNull, counts[6]),
        ]
    }

    /// Set the DC-null subcarrier (centre of the grid).
    pub fn set_dc_null(&mut self) {
        let dc_sc = self.num_subcarriers() / 2;
        for sym in 0..self.config.num_symbols {
            self.grid[dc_sc][sym] = ResourceElement {
                subcarrier: dc_sc,
                symbol: sym,
                value: (0.0, 0.0),
                re_type: ResourceType::DcNull,
            };
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> NrConfig {
        NrConfig {
            numerology: 1,
            num_prbs: 4,
            num_symbols: 14,
        }
    }

    fn default_slot() -> SlotConfig {
        SlotConfig {
            slot_number: 0,
            frame_number: 0,
            cp_type: CyclicPrefixType::Normal,
        }
    }

    // 1. Grid dimensions
    #[test]
    fn test_grid_dimensions() {
        let m = NrResourceGridMapper::new(default_config(), default_slot());
        let grid = m.get_grid();
        assert_eq!(grid.len(), 48, "4 PRBs x 12 = 48 subcarriers");
        for row in grid {
            assert_eq!(row.len(), 14, "14 symbols per slot");
        }
    }

    // 2. Subcarrier spacing
    #[test]
    fn test_subcarrier_spacing() {
        for (mu, expected) in [(0, 15.0), (1, 30.0), (2, 60.0), (3, 120.0), (4, 240.0)] {
            let cfg = NrConfig { numerology: mu, num_prbs: 1, num_symbols: 14 };
            let m = NrResourceGridMapper::new(cfg, default_slot());
            assert!(
                (m.subcarrier_spacing_khz() - expected).abs() < 1e-9,
                "mu={mu}: expected {expected} kHz"
            );
        }
    }

    // 3. Slot duration
    #[test]
    fn test_slot_duration() {
        for (mu, expected) in [(0, 1.0), (1, 0.5), (2, 0.25), (3, 0.125), (4, 0.0625)] {
            let cfg = NrConfig { numerology: mu, num_prbs: 1, num_symbols: 14 };
            let m = NrResourceGridMapper::new(cfg, default_slot());
            assert!(
                (m.slot_duration_ms() - expected).abs() < 1e-12,
                "mu={mu}: expected {expected} ms"
            );
        }
    }

    // 4. DMRS insertion positions and comb-2 spacing
    #[test]
    fn test_dmrs_positions() {
        let mut m = NrResourceGridMapper::new(default_config(), default_slot());
        m.insert_dmrs();

        let dmrs = m.extract_dmrs();
        assert!(!dmrs.is_empty(), "DMRS should be inserted");

        // All DMRS should be on symbols 2 and 3.
        for re in &dmrs {
            assert!(
                re.symbol == 2 || re.symbol == 3,
                "DMRS on unexpected symbol {}",
                re.symbol
            );
            // Comb-2: subcarrier index must be even.
            assert_eq!(re.subcarrier % 2, 0, "DMRS must be on even subcarriers");
        }

        // Expected: 24 even subcarriers x 2 symbols = 48.
        assert_eq!(dmrs.len(), 48);
    }

    // 5. DMRS values have unit-ish power (QPSK)
    #[test]
    fn test_dmrs_power() {
        let mut m = NrResourceGridMapper::new(default_config(), default_slot());
        m.insert_dmrs();
        for re in m.extract_dmrs() {
            let pwr = re.value.0 * re.value.0 + re.value.1 * re.value.1;
            assert!(
                (pwr - 0.5).abs() < 1e-9,
                "DMRS power should be 0.5 (1/sqrt(2) per component), got {pwr}"
            );
        }
    }

    // 6. PDSCH mapping fills only Guard REs
    #[test]
    fn test_pdsch_skips_dmrs() {
        let mut m = NrResourceGridMapper::new(default_config(), default_slot());
        m.insert_dmrs();

        // Feed more data than can fit -- excess is ignored.
        let data: Vec<(f64, f64)> = (0..5000).map(|i| (i as f64, -(i as f64))).collect();
        let mapped = m.map_pdsch(&data);

        // Total REs = 48 x 14 = 672.  DMRS occupies 48.  So data = 624.
        assert_eq!(mapped, 672 - 48);

        // Verify no data RE sits on a DMRS position.
        let grid = m.get_grid();
        for sc in 0..m.num_subcarriers() {
            for sym in 0..m.config().num_symbols {
                let re = &grid[sc][sym];
                if re.re_type == ResourceType::Data {
                    assert!(
                        !(re.symbol == 2 && re.subcarrier % 2 == 0)
                            && !(re.symbol == 3 && re.subcarrier % 2 == 0),
                        "Data placed on DMRS position sc={} sym={}",
                        re.subcarrier,
                        re.symbol
                    );
                }
            }
        }
    }

    // 7. Roundtrip: map then extract data
    #[test]
    fn test_data_roundtrip() {
        let mut m = NrResourceGridMapper::new(default_config(), default_slot());
        m.insert_dmrs();

        let data: Vec<(f64, f64)> = (0..624).map(|i| (i as f64 * 0.01, i as f64 * -0.01)).collect();
        let mapped = m.map_pdsch(&data);
        assert_eq!(mapped, 624);

        let extracted = m.extract_data();
        assert_eq!(extracted.len(), 624);

        for (i, (e, d)) in extracted.iter().zip(data.iter()).enumerate() {
            assert!(
                (e.0 - d.0).abs() < 1e-12 && (e.1 - d.1).abs() < 1e-12,
                "Mismatch at index {i}: extracted={e:?} vs original={d:?}"
            );
        }
    }

    // 8. PTRS insertion
    #[test]
    fn test_ptrs_insertion() {
        let mut m = NrResourceGridMapper::new(default_config(), default_slot());
        m.insert_dmrs();
        m.insert_ptrs();

        let counts = m.resource_counts();
        let ptrs_count = counts.iter().find(|c| c.0 == ResourceType::PtrsRef).unwrap().1;
        assert!(ptrs_count > 0, "PTRS should be present");

        // PTRS must not overlap DMRS symbols.
        let grid = m.get_grid();
        for sc in 0..m.num_subcarriers() {
            for sym in 0..m.config().num_symbols {
                let re = &grid[sc][sym];
                if re.re_type == ResourceType::PtrsRef {
                    assert!(
                        sym != 2 && sym != 3,
                        "PTRS should not be on DMRS symbols"
                    );
                }
            }
        }
    }

    // 9. DC null
    #[test]
    fn test_dc_null() {
        let mut m = NrResourceGridMapper::new(default_config(), default_slot());
        m.set_dc_null();

        let dc_sc = m.num_subcarriers() / 2;
        let grid = m.get_grid();
        for sym in 0..m.config().num_symbols {
            assert_eq!(grid[dc_sc][sym].re_type, ResourceType::DcNull);
            assert_eq!(grid[dc_sc][sym].value, (0.0, 0.0));
        }
    }

    // 10. Resource counts add up
    #[test]
    fn test_resource_counts_sum() {
        let mut m = NrResourceGridMapper::new(default_config(), default_slot());
        m.insert_dmrs();
        m.set_dc_null();

        let data: Vec<(f64, f64)> = (0..2000).map(|_| (1.0, 0.0)).collect();
        m.map_pdsch(&data);

        let counts = m.resource_counts();
        let total: usize = counts.iter().map(|c| c.1).sum();
        assert_eq!(
            total,
            48 * 14,
            "All REs must be accounted for"
        );
    }

    // 11. Empty data mapping returns zero
    #[test]
    fn test_empty_data() {
        let mut m = NrResourceGridMapper::new(default_config(), default_slot());
        let mapped = m.map_pdsch(&[]);
        assert_eq!(mapped, 0);
        let extracted = m.extract_data();
        assert!(extracted.is_empty());
    }

    // 12. Different numerologies give correct subcarrier count
    #[test]
    fn test_prb_scaling() {
        for prbs in [1, 10, 25, 50, 100] {
            let cfg = NrConfig { numerology: 0, num_prbs: prbs, num_symbols: 14 };
            let m = NrResourceGridMapper::new(cfg, default_slot());
            assert_eq!(m.num_subcarriers(), prbs * 12);
            assert_eq!(m.get_grid().len(), prbs * 12);
        }
    }

    // 13. PTRS frequency density parameter
    #[test]
    fn test_ptrs_density() {
        let mut m = NrResourceGridMapper::new(default_config(), default_slot());
        m.insert_ptrs_with_density(12);

        let counts = m.resource_counts();
        let ptrs_count = counts.iter().find(|c| c.0 == ResourceType::PtrsRef).unwrap().1;

        // With density=12, on each of 14 symbols we get 4 PTRS REs (sc 0,12,24,36).
        // No DMRS inserted, so all 14 symbols get PTRS => 4 x 14 = 56.
        assert_eq!(ptrs_count, 4 * 14);
    }

    // 14. Numerology bounds panic
    #[test]
    #[should_panic(expected = "Numerology must be 0..=4")]
    fn test_invalid_numerology() {
        let cfg = NrConfig { numerology: 5, num_prbs: 1, num_symbols: 14 };
        let _ = NrResourceGridMapper::new(cfg, default_slot());
    }
}

//! Index Modulation (IM) mapper for OFDM-IM and related schemes.
//!
//! This module implements index modulation encoding that maps data bits across
//! both subcarrier activation indices and constellation symbols. In OFDM-IM,
//! information is conveyed not only by the symbols on active subcarriers but
//! also by *which* subcarriers are activated.
//!
//! Given `N` total subcarriers and `K` active subcarriers with modulation
//! order `M`:
//!
//! - **Index bits**: `floor(log2(C(N, K)))` bits select which `K` of `N`
//!   subcarriers carry data.
//! - **Symbol bits**: `K * log2(M)` bits are mapped onto the `K` active
//!   subcarriers using standard constellation mapping (BPSK, QPSK, etc.).
//!
//! # Example
//!
//! ```
//! use r4w_core::index_modulation_mapper::{ImConfig, IndexModulationMapper};
//!
//! // 4 subcarriers, 2 active, BPSK
//! let config = ImConfig {
//!     num_subcarriers: 4,
//!     active_subcarriers: 2,
//!     modulation_order: 2,
//! };
//! let mapper = IndexModulationMapper::new(config);
//!
//! assert_eq!(mapper.index_bits(), 2);   // floor(log2(C(4,2))) = floor(log2(6)) = 2
//! assert_eq!(mapper.symbol_bits(), 2);  // 2 * log2(2) = 2
//! assert_eq!(mapper.bits_per_block(), 4);
//!
//! let bits = vec![true, false, true, true]; // 4 bits
//! let block = mapper.map(&bits);
//! assert_eq!(block.active_indices.len(), 2);
//! assert_eq!(block.symbols.len(), 2);
//!
//! let recovered = mapper.demap(&block);
//! assert_eq!(recovered, bits);
//! ```

use std::f64::consts::PI;

/// Configuration for the index modulation mapper.
#[derive(Debug, Clone, PartialEq)]
pub struct ImConfig {
    /// Total number of subcarriers in the block (`N`).
    pub num_subcarriers: usize,
    /// Number of active (data-carrying) subcarriers (`K`).
    pub active_subcarriers: usize,
    /// Constellation order: 2 = BPSK, 4 = QPSK, 8 = 8-PSK, 16 = 16-QAM, etc.
    pub modulation_order: usize,
}

/// Output of the index modulation mapping for one OFDM-IM block.
#[derive(Debug, Clone, PartialEq)]
pub struct ImBlock {
    /// Indices of the active subcarriers (sorted ascending, 0-based).
    pub active_indices: Vec<usize>,
    /// Constellation symbols on the active subcarriers, in order.
    pub symbols: Vec<(f64, f64)>,
}

/// Index Modulation mapper/demapper.
///
/// Encodes data bits into an [`ImBlock`] containing both the subcarrier
/// activation pattern and the constellation symbols, and decodes received
/// blocks back to bits.
#[derive(Debug, Clone)]
pub struct IndexModulationMapper {
    config: ImConfig,
    /// Lookup table: index-bit pattern (as usize) -> sorted active indices.
    index_table: Vec<Vec<usize>>,
    /// Constellation points for the configured modulation order.
    constellation: Vec<(f64, f64)>,
}

// ---------------------------------------------------------------------------
// Helper math
// ---------------------------------------------------------------------------

/// Binomial coefficient C(n, k).
fn binomial(n: usize, k: usize) -> usize {
    if k > n {
        return 0;
    }
    if k == 0 || k == n {
        return 1;
    }
    let k = k.min(n - k); // symmetry
    let mut result: usize = 1;
    for i in 0..k {
        result = result * (n - i) / (i + 1);
    }
    result
}

/// Floor of log2 for a positive integer (returns 0 for input 0 or 1).
fn floor_log2(v: usize) -> usize {
    if v <= 1 {
        return 0;
    }
    (usize::BITS - 1 - v.leading_zeros()) as usize
}

/// Generate all C(n,k) combinations of `k` elements chosen from `0..n`,
/// each combination sorted ascending.
fn combinations(n: usize, k: usize) -> Vec<Vec<usize>> {
    let mut result = Vec::with_capacity(binomial(n, k));
    let mut combo = Vec::with_capacity(k);
    fn recurse(start: usize, n: usize, k: usize, combo: &mut Vec<usize>, result: &mut Vec<Vec<usize>>) {
        if combo.len() == k {
            result.push(combo.clone());
            return;
        }
        let remaining = k - combo.len();
        for i in start..=(n - remaining) {
            combo.push(i);
            recurse(i + 1, n, k, combo, result);
            combo.pop();
        }
    }
    recurse(0, n, k, &mut combo, &mut result);
    result
}

/// Generate PSK/QAM constellation of order `m`.
///
/// - M = 2: BPSK  -> {+1, -1}
/// - M = 4: QPSK  -> unit-circle at 45, 135, 225, 315 degrees
/// - M = 8: 8-PSK -> unit-circle equally spaced
/// - M = 16: 16-QAM -> 4x4 grid, normalized
/// - Other M (power of 2): M-PSK
fn generate_constellation(m: usize) -> Vec<(f64, f64)> {
    assert!(m >= 2 && m.is_power_of_two(), "modulation_order must be a power of 2 >= 2");
    if m == 2 {
        // BPSK: bit 0 -> +1, bit 1 -> -1
        return vec![(1.0, 0.0), (-1.0, 0.0)];
    }
    if m == 16 {
        // 16-QAM: Gray-coded 4x4 grid, average power normalized to 1
        let gray4 = [0usize, 1, 3, 2]; // Gray code for 2 bits
        let levels = [-3.0_f64, -1.0, 1.0, 3.0];
        let norm = (10.0_f64).sqrt(); // sqrt(mean |s|^2) for +/-1,+/-3
        let mut pts = vec![(0.0, 0.0); 16];
        for (gi, &gx) in gray4.iter().enumerate() {
            for (gj, &gy) in gray4.iter().enumerate() {
                let idx = gx * 4 + gy;
                pts[idx] = (levels[gi] / norm, levels[gj] / norm);
            }
        }
        return pts;
    }
    // M-PSK (covers QPSK, 8-PSK, and higher-order PSK)
    (0..m)
        .map(|i| {
            let angle = 2.0 * PI * (i as f64) / (m as f64) + PI / 4.0;
            (angle.cos(), angle.sin())
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Core implementation
// ---------------------------------------------------------------------------

impl IndexModulationMapper {
    /// Create a new mapper from the given configuration.
    ///
    /// # Panics
    ///
    /// Panics if:
    /// - `active_subcarriers` > `num_subcarriers`
    /// - `active_subcarriers` == 0
    /// - `modulation_order` is not a power of two >= 2
    /// - `index_bits()` would be 0 (i.e. C(N,K) < 2)
    pub fn new(config: ImConfig) -> Self {
        assert!(
            config.active_subcarriers > 0 && config.active_subcarriers <= config.num_subcarriers,
            "active_subcarriers must be in 1..=num_subcarriers"
        );
        assert!(
            config.modulation_order >= 2 && config.modulation_order.is_power_of_two(),
            "modulation_order must be a power of 2 >= 2"
        );

        let combos = combinations(config.num_subcarriers, config.active_subcarriers);
        let num_index_bits = floor_log2(combos.len());
        assert!(num_index_bits > 0, "C(N,K) must be >= 2 to carry index bits");

        // Keep only the first 2^index_bits patterns (power-of-two subset).
        let table_size = 1 << num_index_bits;
        let index_table: Vec<Vec<usize>> = combos.into_iter().take(table_size).collect();

        let constellation = generate_constellation(config.modulation_order);

        Self {
            config,
            index_table,
            constellation,
        }
    }

    /// Number of bits conveyed by the subcarrier activation pattern.
    pub fn index_bits(&self) -> usize {
        floor_log2(binomial(self.config.num_subcarriers, self.config.active_subcarriers))
    }

    /// Number of bits conveyed by the constellation symbols on active subcarriers.
    pub fn symbol_bits(&self) -> usize {
        self.config.active_subcarriers * floor_log2(self.config.modulation_order)
    }

    /// Total bits per OFDM-IM block.
    pub fn bits_per_block(&self) -> usize {
        self.index_bits() + self.symbol_bits()
    }

    /// Spectral efficiency in bits per subcarrier per symbol period.
    pub fn spectral_efficiency(&self) -> f64 {
        self.bits_per_block() as f64 / self.config.num_subcarriers as f64
    }

    /// Return a reference to the index lookup table.
    ///
    /// Entry `i` contains the sorted active subcarrier indices for index-bit
    /// pattern `i`.
    pub fn generate_index_table(&self) -> &[Vec<usize>] {
        &self.index_table
    }

    /// Return a reference to the constellation used for symbol mapping.
    pub fn constellation(&self) -> &[(f64, f64)] {
        &self.constellation
    }

    /// Return a reference to the configuration.
    pub fn config(&self) -> &ImConfig {
        &self.config
    }

    // ----- Encoding --------------------------------------------------------

    /// Map a bit slice to an [`ImBlock`].
    ///
    /// The input must have exactly [`bits_per_block()`](Self::bits_per_block) bits.
    ///
    /// The first `index_bits()` bits select the activation pattern; the
    /// remaining `symbol_bits()` bits are split evenly among the `K` active
    /// subcarriers and mapped to constellation symbols.
    pub fn map(&self, bits: &[bool]) -> ImBlock {
        let total = self.bits_per_block();
        assert_eq!(bits.len(), total, "expected {} bits, got {}", total, bits.len());

        let n_idx = self.index_bits();
        let idx_val = bits_to_usize(&bits[..n_idx]);
        let active_indices = self.index_table[idx_val].clone();

        let bits_per_sym = floor_log2(self.config.modulation_order);
        let sym_bits = &bits[n_idx..];
        let symbols: Vec<(f64, f64)> = sym_bits
            .chunks_exact(bits_per_sym)
            .map(|chunk| {
                let sym_idx = bits_to_usize(chunk);
                self.constellation[sym_idx]
            })
            .collect();

        ImBlock {
            active_indices,
            symbols,
        }
    }

    /// Demap an [`ImBlock`] back to bits (maximum-likelihood hard decision).
    ///
    /// The index pattern is identified by exact table lookup; each symbol is
    /// demapped to the nearest constellation point.
    pub fn demap(&self, block: &ImBlock) -> Vec<bool> {
        let mut bits = Vec::with_capacity(self.bits_per_block());

        // --- Recover index bits ---
        let idx_val = self
            .index_table
            .iter()
            .position(|pattern| pattern == &block.active_indices)
            .expect("active_indices not found in index table");
        let n_idx = self.index_bits();
        usize_to_bits(idx_val, n_idx, &mut bits);

        // --- Recover symbol bits ---
        let bits_per_sym = floor_log2(self.config.modulation_order);
        for &sym in &block.symbols {
            let nearest = nearest_constellation_index(&self.constellation, sym);
            usize_to_bits(nearest, bits_per_sym, &mut bits);
        }

        bits
    }

    /// Demap from a full subcarrier vector (length `N`), where inactive
    /// subcarriers are approximately zero.
    ///
    /// This performs a low-complexity two-step detection:
    /// 1. Identify the `K` subcarriers with largest magnitude as active.
    /// 2. Demap the symbols on those subcarriers.
    pub fn demap_from_subcarriers(&self, subcarriers: &[(f64, f64)]) -> Vec<bool> {
        assert_eq!(
            subcarriers.len(),
            self.config.num_subcarriers,
            "expected {} subcarriers",
            self.config.num_subcarriers
        );

        let k = self.config.active_subcarriers;

        // Find K largest-magnitude indices
        let mut indexed_mags: Vec<(usize, f64)> = subcarriers
            .iter()
            .enumerate()
            .map(|(i, &(re, im))| (i, re * re + im * im))
            .collect();
        indexed_mags.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        let mut active: Vec<usize> = indexed_mags[..k].iter().map(|&(i, _)| i).collect();
        active.sort();

        let symbols: Vec<(f64, f64)> = active.iter().map(|&i| subcarriers[i]).collect();

        let block = ImBlock {
            active_indices: active,
            symbols,
        };
        self.demap(&block)
    }

    /// Expand an [`ImBlock`] into a full subcarrier vector of length `N`,
    /// with zeros on inactive subcarriers.
    pub fn to_subcarrier_vector(&self, block: &ImBlock) -> Vec<(f64, f64)> {
        let mut out = vec![(0.0, 0.0); self.config.num_subcarriers];
        for (sym_idx, &sc_idx) in block.active_indices.iter().enumerate() {
            out[sc_idx] = block.symbols[sym_idx];
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Bit / integer helpers
// ---------------------------------------------------------------------------

fn bits_to_usize(bits: &[bool]) -> usize {
    bits.iter().fold(0usize, |acc, &b| (acc << 1) | (b as usize))
}

fn usize_to_bits(val: usize, n: usize, out: &mut Vec<bool>) {
    for i in (0..n).rev() {
        out.push((val >> i) & 1 == 1);
    }
}

fn nearest_constellation_index(constellation: &[(f64, f64)], sym: (f64, f64)) -> usize {
    let mut best = 0;
    let mut best_dist = f64::MAX;
    for (i, &(re, im)) in constellation.iter().enumerate() {
        let d = (sym.0 - re).powi(2) + (sym.1 - im).powi(2);
        if d < best_dist {
            best_dist = d;
            best = i;
        }
    }
    best
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // ---- helpers ----------------------------------------------------------

    fn approx_eq(a: (f64, f64), b: (f64, f64), tol: f64) -> bool {
        (a.0 - b.0).abs() < tol && (a.1 - b.1).abs() < tol
    }

    // ---- tests -----------------------------------------------------------

    #[test]
    fn test_binomial_basic() {
        assert_eq!(binomial(4, 2), 6);
        assert_eq!(binomial(6, 3), 20);
        assert_eq!(binomial(5, 0), 1);
        assert_eq!(binomial(5, 5), 1);
        assert_eq!(binomial(0, 0), 1);
        assert_eq!(binomial(3, 4), 0);
    }

    #[test]
    fn test_floor_log2_values() {
        assert_eq!(floor_log2(1), 0);
        assert_eq!(floor_log2(2), 1);
        assert_eq!(floor_log2(3), 1);
        assert_eq!(floor_log2(4), 2);
        assert_eq!(floor_log2(6), 2);
        assert_eq!(floor_log2(8), 3);
        assert_eq!(floor_log2(16), 4);
    }

    #[test]
    fn test_combinations_count() {
        assert_eq!(combinations(4, 2).len(), 6);
        assert_eq!(combinations(6, 3).len(), 20);
        assert_eq!(combinations(5, 1).len(), 5);
    }

    #[test]
    fn test_bpsk_constellation() {
        let c = generate_constellation(2);
        assert_eq!(c.len(), 2);
        assert!(approx_eq(c[0], (1.0, 0.0), 1e-12));
        assert!(approx_eq(c[1], (-1.0, 0.0), 1e-12));
    }

    #[test]
    fn test_qpsk_constellation() {
        let c = generate_constellation(4);
        assert_eq!(c.len(), 4);
        // All points should be on the unit circle
        for &(re, im) in &c {
            let mag = (re * re + im * im).sqrt();
            assert!((mag - 1.0).abs() < 1e-12, "QPSK point not on unit circle: mag={}", mag);
        }
    }

    #[test]
    fn test_16qam_constellation() {
        let c = generate_constellation(16);
        assert_eq!(c.len(), 16);
        // Average power should be approximately 1.0
        let avg_pwr: f64 = c.iter().map(|&(re, im)| re * re + im * im).sum::<f64>() / 16.0;
        assert!((avg_pwr - 1.0).abs() < 0.01, "16-QAM avg power {} not ~1.0", avg_pwr);
    }

    #[test]
    fn test_bits_per_block_4_2_bpsk() {
        let config = ImConfig {
            num_subcarriers: 4,
            active_subcarriers: 2,
            modulation_order: 2,
        };
        let mapper = IndexModulationMapper::new(config);
        assert_eq!(mapper.index_bits(), 2);  // floor(log2(6)) = 2
        assert_eq!(mapper.symbol_bits(), 2); // 2 * 1
        assert_eq!(mapper.bits_per_block(), 4);
    }

    #[test]
    fn test_bits_per_block_4_2_qpsk() {
        let config = ImConfig {
            num_subcarriers: 4,
            active_subcarriers: 2,
            modulation_order: 4,
        };
        let mapper = IndexModulationMapper::new(config);
        assert_eq!(mapper.index_bits(), 2);  // floor(log2(6)) = 2
        assert_eq!(mapper.symbol_bits(), 4); // 2 * 2
        assert_eq!(mapper.bits_per_block(), 6);
    }

    #[test]
    fn test_map_demap_roundtrip_bpsk() {
        let config = ImConfig {
            num_subcarriers: 4,
            active_subcarriers: 2,
            modulation_order: 2,
        };
        let mapper = IndexModulationMapper::new(config);
        // Exhaustively test all 2^4 = 16 patterns
        for val in 0..(1u32 << 4) {
            let bits: Vec<bool> = (0..4).rev().map(|i| (val >> i) & 1 == 1).collect();
            let block = mapper.map(&bits);
            let recovered = mapper.demap(&block);
            assert_eq!(bits, recovered, "roundtrip failed for val={}", val);
        }
    }

    #[test]
    fn test_map_demap_roundtrip_qpsk() {
        let config = ImConfig {
            num_subcarriers: 4,
            active_subcarriers: 2,
            modulation_order: 4,
        };
        let mapper = IndexModulationMapper::new(config);
        // Test all 2^6 = 64 patterns
        for val in 0..(1u32 << 6) {
            let bits: Vec<bool> = (0..6).rev().map(|i| (val >> i) & 1 == 1).collect();
            let block = mapper.map(&bits);
            let recovered = mapper.demap(&block);
            assert_eq!(bits, recovered, "roundtrip failed for val={}", val);
        }
    }

    #[test]
    fn test_to_subcarrier_vector() {
        let config = ImConfig {
            num_subcarriers: 4,
            active_subcarriers: 2,
            modulation_order: 2,
        };
        let mapper = IndexModulationMapper::new(config);
        let bits = vec![false, false, true, false]; // index pattern 0 -> first combo
        let block = mapper.map(&bits);
        let vec = mapper.to_subcarrier_vector(&block);
        assert_eq!(vec.len(), 4);

        // Inactive subcarriers should be zero
        let mut zero_count = 0;
        for &(re, im) in &vec {
            if re.abs() < 1e-12 && im.abs() < 1e-12 {
                zero_count += 1;
            }
        }
        assert_eq!(zero_count, 2, "expected 2 inactive subcarriers");
    }

    #[test]
    fn test_demap_from_subcarriers() {
        let config = ImConfig {
            num_subcarriers: 4,
            active_subcarriers: 2,
            modulation_order: 2,
        };
        let mapper = IndexModulationMapper::new(config);
        let bits = vec![true, false, true, true];
        let block = mapper.map(&bits);
        let sc = mapper.to_subcarrier_vector(&block);

        let recovered = mapper.demap_from_subcarriers(&sc);
        assert_eq!(bits, recovered);
    }

    #[test]
    fn test_spectral_efficiency() {
        let config = ImConfig {
            num_subcarriers: 4,
            active_subcarriers: 2,
            modulation_order: 2,
        };
        let mapper = IndexModulationMapper::new(config);
        let se = mapper.spectral_efficiency();
        assert!((se - 1.0).abs() < 1e-12, "SE should be 4/4 = 1.0, got {}", se);
    }

    #[test]
    fn test_index_table_size() {
        let config = ImConfig {
            num_subcarriers: 4,
            active_subcarriers: 2,
            modulation_order: 2,
        };
        let mapper = IndexModulationMapper::new(config);
        let table = mapper.generate_index_table();
        // 2^index_bits entries
        assert_eq!(table.len(), 1 << mapper.index_bits());
        // Each entry has K indices
        for entry in table {
            assert_eq!(entry.len(), 2);
        }
    }

    #[test]
    fn test_larger_block_8_3_qpsk() {
        let config = ImConfig {
            num_subcarriers: 8,
            active_subcarriers: 3,
            modulation_order: 4,
        };
        let mapper = IndexModulationMapper::new(config);
        // C(8,3) = 56, floor(log2(56)) = 5
        assert_eq!(mapper.index_bits(), 5);
        // 3 * log2(4) = 6
        assert_eq!(mapper.symbol_bits(), 6);
        assert_eq!(mapper.bits_per_block(), 11);

        // Roundtrip test
        let bits: Vec<bool> = vec![
            true, false, true, true, false, // index bits (5)
            true, true, false, false, true, false, // symbol bits (6)
        ];
        let block = mapper.map(&bits);
        let recovered = mapper.demap(&block);
        assert_eq!(bits, recovered);
    }

    #[test]
    #[should_panic(expected = "active_subcarriers must be in 1..=num_subcarriers")]
    fn test_invalid_active_exceeds_total() {
        let config = ImConfig {
            num_subcarriers: 4,
            active_subcarriers: 5,
            modulation_order: 2,
        };
        IndexModulationMapper::new(config);
    }

    #[test]
    #[should_panic(expected = "modulation_order must be a power of 2 >= 2")]
    fn test_invalid_modulation_order() {
        let config = ImConfig {
            num_subcarriers: 4,
            active_subcarriers: 2,
            modulation_order: 3,
        };
        IndexModulationMapper::new(config);
    }
}

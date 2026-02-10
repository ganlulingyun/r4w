//! Pilot Inserter — OFDM Pilot Tone Insertion and Extraction
//!
//! Places known reference symbols (pilots) at specific subcarrier positions
//! within OFDM symbols for channel estimation and synchronization. Supports
//! comb, block, scattered, and custom pilot patterns.
//!
//! Pilots are complex-valued reference signals whose values are known to both
//! transmitter and receiver. The receiver uses them to estimate the channel
//! frequency response (via interpolation across pilot subcarriers) and to
//! correct phase/frequency offsets.
//!
//! # Pilot Patterns
//!
//! - **Comb**: Pilots at every N-th subcarrier (good for frequency-selective channels)
//! - **Block**: All subcarriers carry pilots every M-th OFDM symbol (good for
//!   time-varying channels)
//! - **Scattered**: Pilots shift in frequency across successive OFDM symbols,
//!   covering the full bandwidth over several symbols (used in DVB-T, LTE)
//! - **Custom**: Arbitrary pilot positions specified by the user
//!
//! # Example
//!
//! ```rust
//! use r4w_core::pilot_inserter::{PilotInserter, PilotPattern};
//!
//! // 16-subcarrier OFDM symbol with comb pilots every 4 subcarriers
//! let pattern = PilotPattern::Comb { spacing: 4 };
//! let inserter = PilotInserter::new(16, pattern, (1.0, 0.0));
//!
//! // Insert data into the non-pilot subcarriers
//! let data: Vec<(f64, f64)> = (0..inserter.num_data_carriers())
//!     .map(|i| (i as f64, 0.0))
//!     .collect();
//! let symbol = inserter.insert(&data);
//! assert_eq!(symbol.len(), 16);
//!
//! // Extract pilots and data back
//! let pilots = inserter.extract_pilots(&symbol);
//! assert!(pilots.iter().all(|&p| p == (1.0, 0.0)));
//! let recovered = inserter.extract_data(&symbol);
//! assert_eq!(recovered, data);
//! ```

/// Pilot placement pattern for OFDM subcarriers.
#[derive(Debug, Clone, PartialEq)]
pub enum PilotPattern {
    /// Pilots at every `spacing`-th subcarrier (indices 0, spacing, 2*spacing, ...).
    Comb { spacing: usize },
    /// All subcarriers carry pilots every `period`-th OFDM symbol.
    /// For the non-pilot symbols, pilots are placed at no positions (data fills all).
    Block { period: usize },
    /// Pilots shift across frequency and time, parameterized by frequency and
    /// time spacing. The pilot positions rotate each OFDM symbol.
    Scattered {
        freq_spacing: usize,
        time_spacing: usize,
    },
    /// User-specified pilot subcarrier indices.
    Custom(Vec<usize>),
}

/// OFDM pilot inserter for fixed pilot patterns (Comb, Block, Custom).
///
/// Manages the mapping between data symbols, pilot symbols, and subcarrier
/// positions within a single OFDM symbol.
#[derive(Debug, Clone)]
pub struct PilotInserter {
    /// FFT size (total number of subcarriers).
    fft_size: usize,
    /// Sorted subcarrier indices that carry pilot symbols.
    pilot_indices: Vec<usize>,
    /// Complex pilot value inserted at each pilot subcarrier.
    pilot_value: (f64, f64),
    /// Sorted subcarrier indices that carry data symbols.
    data_indices: Vec<usize>,
}

impl PilotInserter {
    /// Create a new pilot inserter.
    ///
    /// # Panics
    ///
    /// Panics if `fft_size` is zero, if `spacing` is zero for `Comb` or
    /// `Scattered` patterns, or if any custom index is out of range.
    pub fn new(fft_size: usize, pattern: PilotPattern, pilot_value: (f64, f64)) -> Self {
        assert!(fft_size > 0, "FFT size must be > 0");

        let pilot_indices = match &pattern {
            PilotPattern::Comb { spacing } => {
                assert!(*spacing > 0, "Comb spacing must be > 0");
                (0..fft_size).step_by(*spacing).collect()
            }
            PilotPattern::Block { period: _ } => {
                // Block pattern: in a "pilot symbol" all subcarriers are pilots,
                // but we model the static allocation where no fixed pilots exist
                // in data symbols. The caller manages symbol-level switching.
                // For the inserter, we treat block as having no fixed pilot positions.
                vec![]
            }
            PilotPattern::Scattered {
                freq_spacing,
                time_spacing: _,
            } => {
                assert!(*freq_spacing > 0, "Frequency spacing must be > 0");
                // Initial pilot positions at symbol index 0.
                (0..fft_size).step_by(*freq_spacing).collect()
            }
            PilotPattern::Custom(indices) => {
                let mut sorted = indices.clone();
                sorted.sort_unstable();
                sorted.dedup();
                for &idx in &sorted {
                    assert!(idx < fft_size, "Pilot index {} out of range (fft_size={})", idx, fft_size);
                }
                sorted
            }
        };

        let data_indices: Vec<usize> = (0..fft_size)
            .filter(|i| !pilot_indices.contains(i))
            .collect();

        Self {
            fft_size,
            pilot_indices,
            pilot_value,
            data_indices,
        }
    }

    /// Insert data symbols and pilots into an OFDM symbol.
    ///
    /// `data_symbols` must have exactly `num_data_carriers()` elements.
    /// Returns a vector of length `fft_size` with pilots at their designated
    /// positions, data at data positions, and zeros elsewhere.
    ///
    /// # Panics
    ///
    /// Panics if `data_symbols.len() != self.num_data_carriers()`.
    pub fn insert(&self, data_symbols: &[(f64, f64)]) -> Vec<(f64, f64)> {
        assert_eq!(
            data_symbols.len(),
            self.data_indices.len(),
            "Expected {} data symbols, got {}",
            self.data_indices.len(),
            data_symbols.len()
        );

        let mut symbol = vec![(0.0, 0.0); self.fft_size];

        // Place pilots
        for &idx in &self.pilot_indices {
            symbol[idx] = self.pilot_value;
        }

        // Place data
        for (i, &idx) in self.data_indices.iter().enumerate() {
            symbol[idx] = data_symbols[i];
        }

        symbol
    }

    /// Extract pilot symbols from a received OFDM symbol.
    ///
    /// Returns the complex values at pilot subcarrier positions.
    ///
    /// # Panics
    ///
    /// Panics if `ofdm_symbol.len() != fft_size`.
    pub fn extract_pilots(&self, ofdm_symbol: &[(f64, f64)]) -> Vec<(f64, f64)> {
        assert_eq!(
            ofdm_symbol.len(),
            self.fft_size,
            "OFDM symbol length {} != fft_size {}",
            ofdm_symbol.len(),
            self.fft_size
        );
        self.pilot_indices.iter().map(|&idx| ofdm_symbol[idx]).collect()
    }

    /// Extract data symbols from a received OFDM symbol.
    ///
    /// Returns the complex values at data subcarrier positions.
    ///
    /// # Panics
    ///
    /// Panics if `ofdm_symbol.len() != fft_size`.
    pub fn extract_data(&self, ofdm_symbol: &[(f64, f64)]) -> Vec<(f64, f64)> {
        assert_eq!(
            ofdm_symbol.len(),
            self.fft_size,
            "OFDM symbol length {} != fft_size {}",
            ofdm_symbol.len(),
            self.fft_size
        );
        self.data_indices.iter().map(|&idx| ofdm_symbol[idx]).collect()
    }

    /// Returns the pilot subcarrier indices.
    pub fn pilot_indices(&self) -> &[usize] {
        &self.pilot_indices
    }

    /// Returns the data subcarrier indices.
    pub fn data_indices(&self) -> &[usize] {
        &self.data_indices
    }

    /// Returns the number of subcarriers available for data.
    pub fn num_data_carriers(&self) -> usize {
        self.data_indices.len()
    }
}

/// Scattered pilot inserter with time-varying pilot positions.
///
/// Pilot subcarrier positions rotate each OFDM symbol so that over
/// `time_spacing` symbols, every `freq_spacing`-th subcarrier has been
/// covered by a pilot. This is the pattern used in DVB-T and similar systems.
///
/// The pilot offset for symbol index `n` is `(n % time_spacing) * (freq_spacing / time_spacing)`.
/// Pilot positions for symbol `n`: `offset, offset + freq_spacing, offset + 2*freq_spacing, ...`
#[derive(Debug, Clone)]
pub struct ScatteredPilotInserter {
    /// FFT size (total number of subcarriers).
    fft_size: usize,
    /// Spacing between pilots in frequency domain within a single symbol.
    freq_spacing: usize,
    /// Number of symbols over which the full pilot pattern repeats.
    time_spacing: usize,
    /// Complex pilot value.
    pilot_value: (f64, f64),
    /// Current OFDM symbol index (incremented on each `insert` call).
    symbol_index: usize,
}

impl ScatteredPilotInserter {
    /// Create a new scattered pilot inserter.
    ///
    /// # Panics
    ///
    /// Panics if `fft_size`, `freq_spacing`, or `time_spacing` is zero.
    pub fn new(
        fft_size: usize,
        freq_spacing: usize,
        time_spacing: usize,
        pilot_value: (f64, f64),
    ) -> Self {
        assert!(fft_size > 0, "FFT size must be > 0");
        assert!(freq_spacing > 0, "Frequency spacing must be > 0");
        assert!(time_spacing > 0, "Time spacing must be > 0");

        Self {
            fft_size,
            freq_spacing,
            time_spacing,
            pilot_value,
            symbol_index: 0,
        }
    }

    /// Compute pilot indices for the current symbol index.
    fn current_pilot_indices(&self) -> Vec<usize> {
        let offset = (self.symbol_index % self.time_spacing) * (self.freq_spacing / self.time_spacing);
        (0..self.fft_size)
            .skip(offset)
            .step_by(self.freq_spacing)
            .collect()
    }

    /// Compute data indices for the current symbol index.
    fn current_data_indices(&self) -> Vec<usize> {
        let pilots = self.current_pilot_indices();
        (0..self.fft_size)
            .filter(|i| !pilots.contains(i))
            .collect()
    }

    /// Insert data symbols and pilots into an OFDM symbol, then advance
    /// the symbol index.
    ///
    /// The pilot positions rotate based on the current symbol index.
    /// `data_symbols` must have exactly as many elements as there are
    /// non-pilot subcarriers for the current symbol.
    ///
    /// # Panics
    ///
    /// Panics if `data_symbols` has the wrong length.
    pub fn insert(&mut self, data_symbols: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let pilot_indices = self.current_pilot_indices();
        let data_indices = self.current_data_indices();

        assert_eq!(
            data_symbols.len(),
            data_indices.len(),
            "Expected {} data symbols for symbol index {}, got {}",
            data_indices.len(),
            self.symbol_index,
            data_symbols.len()
        );

        let mut symbol = vec![(0.0, 0.0); self.fft_size];

        for &idx in &pilot_indices {
            symbol[idx] = self.pilot_value;
        }

        for (i, &idx) in data_indices.iter().enumerate() {
            symbol[idx] = data_symbols[i];
        }

        self.symbol_index += 1;
        symbol
    }

    /// Returns the current OFDM symbol index.
    pub fn symbol_index(&self) -> usize {
        self.symbol_index
    }

    /// Returns pilot indices for the current symbol (without advancing).
    pub fn pilot_indices(&self) -> Vec<usize> {
        self.current_pilot_indices()
    }

    /// Returns data indices for the current symbol (without advancing).
    pub fn data_indices(&self) -> Vec<usize> {
        self.current_data_indices()
    }

    /// Returns the number of data carriers for the current symbol.
    pub fn num_data_carriers(&self) -> usize {
        self.current_data_indices().len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_comb_pilot_insert() {
        // 16-point FFT, pilots every 4 subcarriers: 0, 4, 8, 12
        let inserter = PilotInserter::new(16, PilotPattern::Comb { spacing: 4 }, (1.0, 0.0));

        assert_eq!(inserter.pilot_indices(), &[0, 4, 8, 12]);
        assert_eq!(inserter.num_data_carriers(), 12);

        let data: Vec<(f64, f64)> = (0..12).map(|i| (i as f64 + 0.5, 0.0)).collect();
        let symbol = inserter.insert(&data);

        assert_eq!(symbol.len(), 16);
        // Pilots at 0, 4, 8, 12
        assert_eq!(symbol[0], (1.0, 0.0));
        assert_eq!(symbol[4], (1.0, 0.0));
        assert_eq!(symbol[8], (1.0, 0.0));
        assert_eq!(symbol[12], (1.0, 0.0));
        // Data at other positions
        assert_eq!(symbol[1], (0.5, 0.0));  // first data symbol
        assert_eq!(symbol[2], (1.5, 0.0));  // second data symbol
    }

    #[test]
    fn test_block_pilot() {
        // Block pattern: no fixed pilot positions in data symbols
        let inserter = PilotInserter::new(8, PilotPattern::Block { period: 4 }, (1.0, 1.0));

        // Block mode has no fixed pilots, all subcarriers are data
        assert_eq!(inserter.pilot_indices().len(), 0);
        assert_eq!(inserter.num_data_carriers(), 8);

        let data: Vec<(f64, f64)> = (0..8).map(|i| (i as f64, i as f64)).collect();
        let symbol = inserter.insert(&data);
        assert_eq!(symbol.len(), 8);
        assert_eq!(symbol[3], (3.0, 3.0));
    }

    #[test]
    fn test_scattered_pilot() {
        // Scattered: freq_spacing=4, time_spacing=4
        let inserter = PilotInserter::new(
            16,
            PilotPattern::Scattered {
                freq_spacing: 4,
                time_spacing: 4,
            },
            (1.0, 0.0),
        );

        // At symbol 0, pilots at 0, 4, 8, 12 (offset = 0)
        assert_eq!(inserter.pilot_indices(), &[0, 4, 8, 12]);
        assert_eq!(inserter.num_data_carriers(), 12);
    }

    #[test]
    fn test_custom_pattern() {
        let inserter = PilotInserter::new(
            8,
            PilotPattern::Custom(vec![1, 5, 3]),
            (0.5, -0.5),
        );

        // Should be sorted and deduplicated
        assert_eq!(inserter.pilot_indices(), &[1, 3, 5]);
        assert_eq!(inserter.num_data_carriers(), 5);
        assert_eq!(inserter.data_indices(), &[0, 2, 4, 6, 7]);

        let data: Vec<(f64, f64)> = (0..5).map(|i| (i as f64, 0.0)).collect();
        let symbol = inserter.insert(&data);

        assert_eq!(symbol[1], (0.5, -0.5)); // pilot
        assert_eq!(symbol[3], (0.5, -0.5)); // pilot
        assert_eq!(symbol[5], (0.5, -0.5)); // pilot
        assert_eq!(symbol[0], (0.0, 0.0));  // data[0]
        assert_eq!(symbol[2], (1.0, 0.0));  // data[1]
    }

    #[test]
    fn test_extract_pilots() {
        let inserter = PilotInserter::new(8, PilotPattern::Comb { spacing: 2 }, (1.0, 0.0));
        // Pilots at 0, 2, 4, 6

        let data: Vec<(f64, f64)> = vec![(2.0, 3.0), (4.0, 5.0), (6.0, 7.0), (8.0, 9.0)];
        let symbol = inserter.insert(&data);

        let pilots = inserter.extract_pilots(&symbol);
        assert_eq!(pilots.len(), 4);
        assert!(pilots.iter().all(|&p| p == (1.0, 0.0)));
    }

    #[test]
    fn test_extract_data() {
        let inserter = PilotInserter::new(8, PilotPattern::Custom(vec![0, 7]), (1.0, 1.0));
        // Data at 1, 2, 3, 4, 5, 6

        let data: Vec<(f64, f64)> = (0..6).map(|i| (i as f64 * 0.1, i as f64 * 0.2)).collect();
        let symbol = inserter.insert(&data);

        let extracted = inserter.extract_data(&symbol);
        assert_eq!(extracted.len(), 6);
        for (i, &val) in extracted.iter().enumerate() {
            assert_eq!(val, (i as f64 * 0.1, i as f64 * 0.2));
        }
    }

    #[test]
    fn test_roundtrip_data() {
        let inserter = PilotInserter::new(32, PilotPattern::Comb { spacing: 8 }, (1.0, -1.0));
        // Pilots at 0, 8, 16, 24 => 28 data subcarriers

        let data: Vec<(f64, f64)> = (0..28)
            .map(|i| ((i as f64) * 0.3, (i as f64) * -0.1))
            .collect();

        let symbol = inserter.insert(&data);
        let recovered = inserter.extract_data(&symbol);

        assert_eq!(recovered.len(), data.len());
        for (orig, rec) in data.iter().zip(recovered.iter()) {
            assert!((orig.0 - rec.0).abs() < 1e-12);
            assert!((orig.1 - rec.1).abs() < 1e-12);
        }

        // Also verify pilots survived
        let pilots = inserter.extract_pilots(&symbol);
        for p in &pilots {
            assert!((p.0 - 1.0).abs() < 1e-12);
            assert!((p.1 - (-1.0)).abs() < 1e-12);
        }
    }

    #[test]
    fn test_pilot_indices() {
        let inserter = PilotInserter::new(12, PilotPattern::Comb { spacing: 3 }, (1.0, 0.0));
        assert_eq!(inserter.pilot_indices(), &[0, 3, 6, 9]);

        let inserter2 = PilotInserter::new(10, PilotPattern::Custom(vec![9, 0, 5]), (1.0, 0.0));
        assert_eq!(inserter2.pilot_indices(), &[0, 5, 9]);
    }

    #[test]
    fn test_num_data_carriers() {
        // Comb spacing=4 on 16 subcarriers: pilots at 0,4,8,12 => 12 data
        let inserter = PilotInserter::new(16, PilotPattern::Comb { spacing: 4 }, (1.0, 0.0));
        assert_eq!(inserter.num_data_carriers(), 12);

        // Custom with 3 pilots on 8 subcarriers => 5 data
        let inserter2 = PilotInserter::new(8, PilotPattern::Custom(vec![1, 3, 5]), (1.0, 0.0));
        assert_eq!(inserter2.num_data_carriers(), 5);

        // Block pattern: all 8 are data (no fixed pilots)
        let inserter3 = PilotInserter::new(8, PilotPattern::Block { period: 2 }, (1.0, 0.0));
        assert_eq!(inserter3.num_data_carriers(), 8);
    }

    #[test]
    fn test_scattered_rotation() {
        // Scattered inserter: freq_spacing=4, time_spacing=4, fft_size=16
        // Symbol 0: offset = 0 => pilots at 0, 4, 8, 12
        // Symbol 1: offset = 1 => pilots at 1, 5, 9, 13
        // Symbol 2: offset = 2 => pilots at 2, 6, 10, 14
        // Symbol 3: offset = 3 => pilots at 3, 7, 11, 15
        // Symbol 4: offset = 0 => pilots at 0, 4, 8, 12 (wraps)
        let mut inserter = ScatteredPilotInserter::new(16, 4, 4, (1.0, 0.0));

        // Symbol 0: pilots at 0, 4, 8, 12
        assert_eq!(inserter.symbol_index(), 0);
        let pilots_0 = inserter.pilot_indices();
        assert_eq!(pilots_0, vec![0, 4, 8, 12]);

        let data_0: Vec<(f64, f64)> = vec![(1.0, 0.0); inserter.num_data_carriers()];
        let sym_0 = inserter.insert(&data_0);
        assert_eq!(sym_0[0], (1.0, 0.0)); // pilot
        assert_eq!(inserter.symbol_index(), 1);

        // Symbol 1: pilots at 1, 5, 9, 13
        let pilots_1 = inserter.pilot_indices();
        assert_eq!(pilots_1, vec![1, 5, 9, 13]);

        let data_1: Vec<(f64, f64)> = vec![(2.0, 0.0); inserter.num_data_carriers()];
        let sym_1 = inserter.insert(&data_1);
        assert_eq!(sym_1[1], (1.0, 0.0)); // pilot
        assert_eq!(sym_1[0], (2.0, 0.0)); // data
        assert_eq!(inserter.symbol_index(), 2);

        // Symbol 2: pilots at 2, 6, 10, 14
        let pilots_2 = inserter.pilot_indices();
        assert_eq!(pilots_2, vec![2, 6, 10, 14]);
        let data_2: Vec<(f64, f64)> = vec![(3.0, 0.0); inserter.num_data_carriers()];
        let _ = inserter.insert(&data_2);
        assert_eq!(inserter.symbol_index(), 3);

        // Symbol 3: pilots at 3, 7, 11, 15
        let pilots_3 = inserter.pilot_indices();
        assert_eq!(pilots_3, vec![3, 7, 11, 15]);
        let data_3: Vec<(f64, f64)> = vec![(4.0, 0.0); inserter.num_data_carriers()];
        let _ = inserter.insert(&data_3);
        assert_eq!(inserter.symbol_index(), 4);

        // Symbol 4: wraps back to offset 0 => pilots at 0, 4, 8, 12
        let pilots_4 = inserter.pilot_indices();
        assert_eq!(pilots_4, vec![0, 4, 8, 12]);
    }
}

//! Chunks to Symbols (LUT-Based Symbol Mapping)
//!
//! Maps integer symbol indices to constellation points using a look-up table.
//! This is the inverse of symbol slicing — it takes symbol indices and produces
//! the corresponding IQ constellation points.
//!
//! Also includes **Symbols to Soft Bits** for soft-decision output from received
//! constellation points.
//!
//! ## Blocks
//!
//! - **ChunksToSymbols**: Map symbol indices → constellation points (LUT)
//! - **SymbolsToSoftBits**: Compute soft-decision (LLR) values from received symbols
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::chunks_to_symbols::ChunksToSymbols;
//! use num_complex::Complex64;
//!
//! // QPSK constellation
//! let qpsk = vec![
//!     Complex64::new( 1.0,  1.0),  // 0
//!     Complex64::new(-1.0,  1.0),  // 1
//!     Complex64::new(-1.0, -1.0),  // 2
//!     Complex64::new( 1.0, -1.0),  // 3
//! ];
//! let mapper = ChunksToSymbols::new(qpsk);
//! let indices = vec![0, 3, 1, 2];
//! let symbols = mapper.map(&indices);
//! assert_eq!(symbols[0], Complex64::new(1.0, 1.0));
//! assert_eq!(symbols[1], Complex64::new(1.0, -1.0));
//! ```

use num_complex::Complex64;

/// Look-up table based symbol mapper.
#[derive(Debug, Clone)]
pub struct ChunksToSymbols {
    /// Constellation points indexed by symbol value
    table: Vec<Complex64>,
    /// Number of bits per symbol (log2 of table size)
    bits_per_symbol: usize,
}

impl ChunksToSymbols {
    /// Create a new mapper from a constellation table.
    pub fn new(table: Vec<Complex64>) -> Self {
        let size = table.len();
        let bits_per_symbol = if size > 0 {
            (size as f64).log2().ceil() as usize
        } else {
            0
        };
        Self {
            table,
            bits_per_symbol,
        }
    }

    /// Create BPSK mapper: {0 → +1, 1 → -1}.
    pub fn bpsk() -> Self {
        Self::new(vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
        ])
    }

    /// Create QPSK mapper (Gray coded).
    pub fn qpsk() -> Self {
        let s = 1.0 / 2.0_f64.sqrt();
        Self::new(vec![
            Complex64::new(s, s),    // 00
            Complex64::new(-s, s),   // 01
            Complex64::new(-s, -s),  // 11
            Complex64::new(s, -s),   // 10
        ])
    }

    /// Create 8PSK mapper.
    pub fn psk8() -> Self {
        use std::f64::consts::PI;
        let points: Vec<Complex64> = (0..8)
            .map(|i| {
                let angle = 2.0 * PI * i as f64 / 8.0;
                Complex64::new(angle.cos(), angle.sin())
            })
            .collect();
        Self::new(points)
    }

    /// Create 16QAM mapper (Gray coded).
    pub fn qam16() -> Self {
        let scale = 1.0 / 10.0_f64.sqrt(); // Unit average power
        let levels = [-3.0, -1.0, 1.0, 3.0];
        let mut points = Vec::with_capacity(16);
        for &q in &levels {
            for &i in &levels {
                points.push(Complex64::new(i * scale, q * scale));
            }
        }
        Self::new(points)
    }

    /// Map symbol indices to constellation points.
    pub fn map(&self, indices: &[usize]) -> Vec<Complex64> {
        indices
            .iter()
            .map(|&idx| {
                if idx < self.table.len() {
                    self.table[idx]
                } else {
                    Complex64::new(0.0, 0.0)
                }
            })
            .collect()
    }

    /// Map bits to symbols (packing bits_per_symbol bits into each index).
    pub fn map_bits(&self, bits: &[bool]) -> Vec<Complex64> {
        if self.bits_per_symbol == 0 {
            return Vec::new();
        }
        bits.chunks(self.bits_per_symbol)
            .map(|chunk| {
                let mut idx = 0usize;
                for (i, &b) in chunk.iter().enumerate() {
                    if b {
                        idx |= 1 << (self.bits_per_symbol - 1 - i);
                    }
                }
                if idx < self.table.len() {
                    self.table[idx]
                } else {
                    Complex64::new(0.0, 0.0)
                }
            })
            .collect()
    }

    /// Get constellation table.
    pub fn table(&self) -> &[Complex64] {
        &self.table
    }

    /// Get bits per symbol.
    pub fn bits_per_symbol(&self) -> usize {
        self.bits_per_symbol
    }

    /// Get constellation size (M).
    pub fn constellation_size(&self) -> usize {
        self.table.len()
    }
}

/// Soft-bit generator from received constellation points.
///
/// Computes approximate log-likelihood ratios (LLRs) for each bit position
/// based on minimum distance to constellation subsets.
#[derive(Debug, Clone)]
pub struct SymbolsToSoftBits {
    /// Constellation points
    table: Vec<Complex64>,
    /// Bits per symbol
    bits_per_symbol: usize,
    /// Noise variance (σ²) for LLR scaling
    noise_variance: f64,
}

impl SymbolsToSoftBits {
    /// Create from constellation and noise variance.
    pub fn new(table: Vec<Complex64>, noise_variance: f64) -> Self {
        let bits_per_symbol = if table.is_empty() {
            0
        } else {
            (table.len() as f64).log2().ceil() as usize
        };
        Self {
            table,
            bits_per_symbol,
            noise_variance: noise_variance.max(1e-10),
        }
    }

    /// Create for QPSK with given noise variance.
    pub fn qpsk(noise_variance: f64) -> Self {
        let s = 1.0 / 2.0_f64.sqrt();
        Self::new(
            vec![
                Complex64::new(s, s),
                Complex64::new(-s, s),
                Complex64::new(-s, -s),
                Complex64::new(s, -s),
            ],
            noise_variance,
        )
    }

    /// Compute approximate LLR for each bit of each received symbol.
    ///
    /// Returns `bits_per_symbol` soft values per input sample.
    /// Positive = bit is likely 0, Negative = bit is likely 1.
    pub fn demap(&self, received: &[Complex64]) -> Vec<f64> {
        let mut soft_bits = Vec::with_capacity(received.len() * self.bits_per_symbol);

        for &rx in received {
            for bit_pos in 0..self.bits_per_symbol {
                let mask = 1 << (self.bits_per_symbol - 1 - bit_pos);

                // Find minimum distance to constellation points with bit=0 and bit=1
                let mut min_dist_0 = f64::MAX;
                let mut min_dist_1 = f64::MAX;

                for (idx, &point) in self.table.iter().enumerate() {
                    let dist = (rx - point).norm_sqr();
                    if idx & mask == 0 {
                        min_dist_0 = min_dist_0.min(dist);
                    } else {
                        min_dist_1 = min_dist_1.min(dist);
                    }
                }

                // Approximate LLR = (d1² - d0²) / σ²
                let llr = (min_dist_1 - min_dist_0) / self.noise_variance;
                soft_bits.push(llr);
            }
        }

        soft_bits
    }

    /// Set noise variance.
    pub fn set_noise_variance(&mut self, sigma_sq: f64) {
        self.noise_variance = sigma_sq.max(1e-10);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bpsk_map() {
        let mapper = ChunksToSymbols::bpsk();
        let symbols = mapper.map(&[0, 1, 0, 1]);
        assert!((symbols[0].re - 1.0).abs() < 1e-10);
        assert!((symbols[1].re + 1.0).abs() < 1e-10);
        assert_eq!(mapper.bits_per_symbol(), 1);
    }

    #[test]
    fn test_qpsk_map() {
        let mapper = ChunksToSymbols::qpsk();
        assert_eq!(mapper.constellation_size(), 4);
        assert_eq!(mapper.bits_per_symbol(), 2);
        let symbols = mapper.map(&[0, 1, 2, 3]);
        assert_eq!(symbols.len(), 4);
    }

    #[test]
    fn test_map_bits_bpsk() {
        let mapper = ChunksToSymbols::bpsk();
        let bits = vec![false, true, false, true];
        let symbols = mapper.map_bits(&bits);
        assert_eq!(symbols.len(), 4);
        assert!((symbols[0].re - 1.0).abs() < 1e-10);  // 0 → +1
        assert!((symbols[1].re + 1.0).abs() < 1e-10);  // 1 → -1
    }

    #[test]
    fn test_map_bits_qpsk() {
        let mapper = ChunksToSymbols::qpsk();
        let bits = vec![false, false, false, true]; // symbols: 0, 1
        let symbols = mapper.map_bits(&bits);
        assert_eq!(symbols.len(), 2);
    }

    #[test]
    fn test_out_of_range_index() {
        let mapper = ChunksToSymbols::bpsk();
        let symbols = mapper.map(&[0, 1, 99]);
        assert_eq!(symbols[2], Complex64::new(0.0, 0.0));
    }

    #[test]
    fn test_16qam() {
        let mapper = ChunksToSymbols::qam16();
        assert_eq!(mapper.constellation_size(), 16);
        assert_eq!(mapper.bits_per_symbol(), 4);
        let symbols = mapper.map(&[0, 5, 10, 15]);
        assert_eq!(symbols.len(), 4);
    }

    #[test]
    fn test_8psk() {
        let mapper = ChunksToSymbols::psk8();
        assert_eq!(mapper.constellation_size(), 8);
        let symbols = mapper.map(&[0, 1, 2, 3, 4, 5, 6, 7]);
        // All points should have unit magnitude
        for s in &symbols {
            assert!((s.norm() - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_soft_bits_bpsk_noiseless() {
        // BPSK: table = [+1, -1]
        let soft = SymbolsToSoftBits::new(
            vec![Complex64::new(1.0, 0.0), Complex64::new(-1.0, 0.0)],
            0.1,
        );
        // Receive exactly +1 → bit 0 is very likely → positive LLR
        let llrs = soft.demap(&[Complex64::new(1.0, 0.0)]);
        assert_eq!(llrs.len(), 1);
        assert!(llrs[0] > 0.0, "LLR should be positive for received +1");
    }

    #[test]
    fn test_soft_bits_bpsk_negative() {
        let soft = SymbolsToSoftBits::new(
            vec![Complex64::new(1.0, 0.0), Complex64::new(-1.0, 0.0)],
            0.1,
        );
        // Receive -1 → bit 1 is likely → negative LLR
        let llrs = soft.demap(&[Complex64::new(-1.0, 0.0)]);
        assert!(llrs[0] < 0.0, "LLR should be negative for received -1");
    }

    #[test]
    fn test_soft_bits_qpsk() {
        let soft = SymbolsToSoftBits::qpsk(0.1);
        let llrs = soft.demap(&[Complex64::new(0.7, 0.7)]);
        assert_eq!(llrs.len(), 2); // 2 bits per QPSK symbol
    }

    #[test]
    fn test_empty_input() {
        let mapper = ChunksToSymbols::bpsk();
        assert!(mapper.map(&[]).is_empty());
        assert!(mapper.map_bits(&[]).is_empty());
    }
}

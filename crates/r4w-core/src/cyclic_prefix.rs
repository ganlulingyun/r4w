//! Cyclic Prefix — OFDM Guard Interval insertion and removal
//!
//! The cyclic prefix (CP) copies the tail of an OFDM symbol to its front,
//! creating a guard interval that absorbs multipath delay spread. Used in
//! LTE, WiFi (802.11a/g/n/ac), DVB-T, and all modern OFDM systems.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cyclic_prefix::{CyclicPrefixAdder, CyclicPrefixRemover};
//! use num_complex::Complex64;
//!
//! let mut adder = CyclicPrefixAdder::new(64, 16);
//! let symbol: Vec<Complex64> = (0..64).map(|i| Complex64::new(i as f64, 0.0)).collect();
//! let with_cp = adder.add_cp(&symbol);
//! assert_eq!(with_cp.len(), 80); // 64 + 16
//!
//! let mut remover = CyclicPrefixRemover::new(64, 16);
//! let without_cp = remover.remove_cp(&with_cp);
//! assert_eq!(without_cp.len(), 64);
//! assert_eq!(without_cp, symbol);
//! ```

use num_complex::Complex64;

/// Cyclic Prefix Adder — prepends CP to OFDM symbols.
#[derive(Debug, Clone)]
pub struct CyclicPrefixAdder {
    /// FFT size (symbol length without CP).
    fft_size: usize,
    /// CP length in samples.
    cp_length: usize,
}

impl CyclicPrefixAdder {
    pub fn new(fft_size: usize, cp_length: usize) -> Self {
        assert!(cp_length <= fft_size, "CP length must be <= FFT size");
        Self { fft_size, cp_length }
    }

    /// Add cyclic prefix to a single OFDM symbol.
    pub fn add_cp(&self, symbol: &[Complex64]) -> Vec<Complex64> {
        assert_eq!(symbol.len(), self.fft_size);
        let mut output = Vec::with_capacity(self.fft_size + self.cp_length);
        // Copy last cp_length samples to the front
        output.extend_from_slice(&symbol[self.fft_size - self.cp_length..]);
        output.extend_from_slice(symbol);
        output
    }

    /// Add CP to multiple OFDM symbols (concatenated input).
    pub fn add_cp_block(&self, symbols: &[Complex64]) -> Vec<Complex64> {
        let num_symbols = symbols.len() / self.fft_size;
        let mut output = Vec::with_capacity(num_symbols * (self.fft_size + self.cp_length));

        for i in 0..num_symbols {
            let start = i * self.fft_size;
            let end = start + self.fft_size;
            output.extend(self.add_cp(&symbols[start..end]));
        }

        output
    }

    /// Symbol duration with CP as a fraction of symbol rate.
    pub fn overhead_ratio(&self) -> f64 {
        (self.fft_size + self.cp_length) as f64 / self.fft_size as f64
    }

    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    pub fn cp_length(&self) -> usize {
        self.cp_length
    }
}

/// Cyclic Prefix Remover — strips CP from received OFDM symbols.
#[derive(Debug, Clone)]
pub struct CyclicPrefixRemover {
    /// FFT size.
    fft_size: usize,
    /// CP length.
    cp_length: usize,
}

impl CyclicPrefixRemover {
    pub fn new(fft_size: usize, cp_length: usize) -> Self {
        Self { fft_size, cp_length }
    }

    /// Remove CP from a single OFDM symbol (with CP).
    pub fn remove_cp(&self, symbol_with_cp: &[Complex64]) -> Vec<Complex64> {
        let expected_len = self.fft_size + self.cp_length;
        assert_eq!(
            symbol_with_cp.len(),
            expected_len,
            "Expected {} samples, got {}",
            expected_len,
            symbol_with_cp.len()
        );
        symbol_with_cp[self.cp_length..].to_vec()
    }

    /// Remove CP from multiple symbols.
    pub fn remove_cp_block(&self, symbols_with_cp: &[Complex64]) -> Vec<Complex64> {
        let sym_len = self.fft_size + self.cp_length;
        let num_symbols = symbols_with_cp.len() / sym_len;
        let mut output = Vec::with_capacity(num_symbols * self.fft_size);

        for i in 0..num_symbols {
            let start = i * sym_len;
            let end = start + sym_len;
            output.extend(self.remove_cp(&symbols_with_cp[start..end]));
        }

        output
    }
}

/// Cyclic Suffix Adder — appends CS to OFDM symbols (used in some 5G NR configs).
#[derive(Debug, Clone)]
pub struct CyclicSuffixAdder {
    fft_size: usize,
    cs_length: usize,
}

impl CyclicSuffixAdder {
    pub fn new(fft_size: usize, cs_length: usize) -> Self {
        assert!(cs_length <= fft_size);
        Self { fft_size, cs_length }
    }

    /// Add cyclic suffix (copy first cs_length samples to end).
    pub fn add_cs(&self, symbol: &[Complex64]) -> Vec<Complex64> {
        assert_eq!(symbol.len(), self.fft_size);
        let mut output = Vec::with_capacity(self.fft_size + self.cs_length);
        output.extend_from_slice(symbol);
        output.extend_from_slice(&symbol[..self.cs_length]);
        output
    }
}

/// Standard OFDM configurations.
pub struct OfdmCpConfig;

impl OfdmCpConfig {
    /// WiFi 802.11a/g: FFT=64, CP=16 (800ns at 20MHz).
    pub fn wifi() -> (usize, usize) {
        (64, 16)
    }

    /// WiFi 802.11n short GI: FFT=64, CP=8 (400ns at 20MHz).
    pub fn wifi_short_gi() -> (usize, usize) {
        (64, 8)
    }

    /// LTE normal CP, 15kHz subcarrier spacing.
    pub fn lte_normal() -> (usize, usize) {
        (2048, 144)
    }

    /// LTE extended CP, 15kHz subcarrier spacing.
    pub fn lte_extended() -> (usize, usize) {
        (2048, 512)
    }

    /// DVB-T 2K mode, 1/4 guard interval.
    pub fn dvbt_2k_quarter() -> (usize, usize) {
        (2048, 512)
    }

    /// DVB-T 8K mode, 1/8 guard interval.
    pub fn dvbt_8k_eighth() -> (usize, usize) {
        (8192, 1024)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add_cp_basic() {
        let adder = CyclicPrefixAdder::new(8, 2);
        let symbol: Vec<Complex64> = (0..8)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let with_cp = adder.add_cp(&symbol);
        assert_eq!(with_cp.len(), 10);
        // CP should be last 2 samples of symbol (6.0, 7.0) prepended
        assert_eq!(with_cp[0].re, 6.0);
        assert_eq!(with_cp[1].re, 7.0);
        // Followed by original symbol
        assert_eq!(with_cp[2].re, 0.0);
        assert_eq!(with_cp[9].re, 7.0);
    }

    #[test]
    fn test_remove_cp_basic() {
        let remover = CyclicPrefixRemover::new(8, 2);
        let with_cp: Vec<Complex64> = (0..10)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let without_cp = remover.remove_cp(&with_cp);
        assert_eq!(without_cp.len(), 8);
        // Should skip first 2 samples
        assert_eq!(without_cp[0].re, 2.0);
        assert_eq!(without_cp[7].re, 9.0);
    }

    #[test]
    fn test_add_remove_roundtrip() {
        let adder = CyclicPrefixAdder::new(64, 16);
        let remover = CyclicPrefixRemover::new(64, 16);

        let symbol: Vec<Complex64> = (0..64)
            .map(|i| Complex64::from_polar(1.0, i as f64 * 0.1))
            .collect();

        let with_cp = adder.add_cp(&symbol);
        assert_eq!(with_cp.len(), 80);

        let recovered = remover.remove_cp(&with_cp);
        assert_eq!(recovered.len(), 64);

        for (orig, recv) in symbol.iter().zip(recovered.iter()) {
            assert!(
                (orig - recv).norm() < 1e-10,
                "CP roundtrip should be perfect"
            );
        }
    }

    #[test]
    fn test_block_processing() {
        let adder = CyclicPrefixAdder::new(8, 2);
        let remover = CyclicPrefixRemover::new(8, 2);

        // 3 symbols
        let symbols: Vec<Complex64> = (0..24)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();

        let with_cp = adder.add_cp_block(&symbols);
        assert_eq!(with_cp.len(), 30); // 3 * (8+2)

        let recovered = remover.remove_cp_block(&with_cp);
        assert_eq!(recovered.len(), 24);

        for (orig, recv) in symbols.iter().zip(recovered.iter()) {
            assert!((orig - recv).norm() < 1e-10);
        }
    }

    #[test]
    fn test_overhead_ratio() {
        let adder = CyclicPrefixAdder::new(64, 16);
        let ratio = adder.overhead_ratio();
        assert!((ratio - 1.25).abs() < 0.01); // 80/64 = 1.25
    }

    #[test]
    fn test_wifi_config() {
        let (fft, cp) = OfdmCpConfig::wifi();
        assert_eq!(fft, 64);
        assert_eq!(cp, 16);
    }

    #[test]
    fn test_lte_config() {
        let (fft, cp) = OfdmCpConfig::lte_normal();
        assert_eq!(fft, 2048);
        assert_eq!(cp, 144);
    }

    #[test]
    fn test_cyclic_suffix() {
        let adder = CyclicSuffixAdder::new(8, 2);
        let symbol: Vec<Complex64> = (0..8)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let with_cs = adder.add_cs(&symbol);
        assert_eq!(with_cs.len(), 10);
        // CS should be first 2 samples appended at the end
        assert_eq!(with_cs[8].re, 0.0);
        assert_eq!(with_cs[9].re, 1.0);
    }

    #[test]
    fn test_zero_cp() {
        let adder = CyclicPrefixAdder::new(8, 0);
        let symbol = vec![Complex64::new(1.0, 0.0); 8];
        let with_cp = adder.add_cp(&symbol);
        assert_eq!(with_cp.len(), 8); // No CP added
    }

    #[test]
    fn test_cp_is_cyclic() {
        let adder = CyclicPrefixAdder::new(8, 4);
        let symbol: Vec<Complex64> = (0..8)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let with_cp = adder.add_cp(&symbol);

        // CP samples should match the tail of the symbol
        for i in 0..4 {
            assert_eq!(with_cp[i].re, symbol[4 + i].re);
        }
    }
}

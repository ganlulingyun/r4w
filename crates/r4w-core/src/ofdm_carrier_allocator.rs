//! OFDM Carrier Allocator — Subcarrier Mapping and Extraction
//!
//! Maps data symbols, pilot symbols, and sync words onto OFDM subcarriers
//! (TX side) and extracts them back (RX side). Handles null subcarriers
//! (guard bands, DC), pilot insertion with cycling patterns, and data
//! packing across multiple OFDM symbols.
//!
//! GNU Radio equivalent: `ofdm_carrier_allocator_cvc` (TX) and
//! `ofdm_serializer_vcc` (RX).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ofdm_carrier_allocator::{CarrierAllocator, CarrierSerializer, CarrierAllocatorConfig};
//! use num_complex::Complex64;
//!
//! let config = CarrierAllocatorConfig::simple(64, &[1,2,3,4,5,6,7,8], &[10,20], 1.0);
//! let alloc = CarrierAllocator::new(config.clone());
//! let ser = CarrierSerializer::new(config);
//!
//! let data = vec![Complex64::new(1.0, 0.0); 8];
//! let symbol = alloc.allocate(&data);
//! assert_eq!(symbol.len(), 64);
//!
//! let extracted = ser.extract_data(&symbol);
//! assert_eq!(extracted.len(), 8);
//! ```

use num_complex::Complex64;

/// Configuration for OFDM carrier allocation.
#[derive(Debug, Clone)]
pub struct CarrierAllocatorConfig {
    /// FFT size.
    pub fft_size: usize,
    /// Data subcarrier indices (can be negative, mapped via (idx + fft_size) % fft_size).
    pub data_carriers: Vec<i32>,
    /// Pilot subcarrier indices per OFDM symbol (cycling pattern).
    pub pilot_carriers: Vec<Vec<i32>>,
    /// Pilot symbol values per OFDM symbol (cycling pattern, same shape as pilot_carriers).
    pub pilot_symbols: Vec<Vec<Complex64>>,
    /// Optional sync words (full FFT-size vectors inserted before data symbols).
    pub sync_words: Vec<Vec<Complex64>>,
    /// Whether to null the DC subcarrier (index 0).
    pub dc_null: bool,
}

impl CarrierAllocatorConfig {
    /// Create a simple configuration with fixed pilots.
    pub fn simple(
        fft_size: usize,
        data_carriers: &[i32],
        pilot_carriers: &[i32],
        pilot_value: f64,
    ) -> Self {
        let pilot_syms: Vec<Complex64> =
            pilot_carriers.iter().map(|_| Complex64::new(pilot_value, 0.0)).collect();
        Self {
            fft_size,
            data_carriers: data_carriers.to_vec(),
            pilot_carriers: vec![pilot_carriers.to_vec()],
            pilot_symbols: vec![pilot_syms],
            sync_words: vec![],
            dc_null: true,
        }
    }

    /// IEEE 802.11a/g preset: 64-point FFT, 48 data + 4 pilot subcarriers.
    pub fn wifi_80211a() -> Self {
        // Data: -26..-22, -20..-8, -6..-1, 1..6, 8..20, 22..26
        let mut data = Vec::new();
        for k in -26..=-22 {
            data.push(k);
        }
        for k in -20..=-8 {
            data.push(k);
        }
        for k in -6..=-1 {
            data.push(k);
        }
        for k in 1..=6 {
            data.push(k);
        }
        for k in 8..=20 {
            data.push(k);
        }
        for k in 22..=26 {
            data.push(k);
        }

        let pilot_carriers = vec![-21, -7, 7, 21];
        // 802.11a pilot polarity sequence (simplified: all +1 for first symbol)
        let pilot_syms = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
        ];

        Self {
            fft_size: 64,
            data_carriers: data,
            pilot_carriers: vec![pilot_carriers],
            pilot_symbols: vec![pilot_syms],
            sync_words: vec![],
            dc_null: true,
        }
    }

    /// LTE-like resource block allocation.
    pub fn lte_rb(num_rb: usize, fft_size: usize) -> Self {
        let sc_per_rb = 12;
        let total_sc = num_rb * sc_per_rb;
        let half = total_sc as i32 / 2;

        let mut data = Vec::new();
        for k in -half..half {
            if k == 0 {
                continue;
            }
            data.push(k);
        }

        // Pilot every 6 subcarriers
        let mut pilots = Vec::new();
        let mut k = -half + 3;
        while k < half {
            if k != 0 {
                pilots.push(k);
            }
            k += 6;
        }

        let pilot_syms = pilots.iter().map(|_| Complex64::new(1.0, 0.0)).collect();
        // Remove pilot positions from data
        let data: Vec<i32> = data.iter().filter(|d| !pilots.contains(d)).cloned().collect();

        Self {
            fft_size,
            data_carriers: data,
            pilot_carriers: vec![pilots],
            pilot_symbols: vec![pilot_syms],
            sync_words: vec![],
            dc_null: true,
        }
    }

    /// Map a (possibly negative) subcarrier index to an FFT bin.
    fn to_fft_bin(&self, idx: i32) -> usize {
        ((idx as i64 + self.fft_size as i64) % self.fft_size as i64) as usize
    }

    /// Number of data subcarriers per OFDM symbol.
    pub fn num_data_carriers(&self) -> usize {
        self.data_carriers.len()
    }
}

/// TX-side OFDM carrier allocator.
#[derive(Debug, Clone)]
pub struct CarrierAllocator {
    config: CarrierAllocatorConfig,
    symbol_index: usize,
}

impl CarrierAllocator {
    pub fn new(config: CarrierAllocatorConfig) -> Self {
        Self {
            config,
            symbol_index: 0,
        }
    }

    /// Reset internal symbol counter.
    pub fn reset(&mut self) {
        self.symbol_index = 0;
    }

    /// Allocate one OFDM symbol's worth of data onto subcarriers.
    ///
    /// Input length must equal `config.num_data_carriers()`.
    /// Returns a vector of length `fft_size`.
    pub fn allocate(&self, data_symbols: &[Complex64]) -> Vec<Complex64> {
        assert_eq!(
            data_symbols.len(),
            self.config.num_data_carriers(),
            "data length {} != num_data_carriers {}",
            data_symbols.len(),
            self.config.num_data_carriers()
        );

        let mut symbol = vec![Complex64::new(0.0, 0.0); self.config.fft_size];

        // Insert data
        for (i, &idx) in self.config.data_carriers.iter().enumerate() {
            let bin = self.config.to_fft_bin(idx);
            symbol[bin] = data_symbols[i];
        }

        // Insert pilots (cycle through pilot patterns)
        if !self.config.pilot_carriers.is_empty() {
            let pc_idx = self.symbol_index % self.config.pilot_carriers.len();
            let carriers = &self.config.pilot_carriers[pc_idx];
            let symbols = &self.config.pilot_symbols[pc_idx];
            for (i, &idx) in carriers.iter().enumerate() {
                let bin = self.config.to_fft_bin(idx);
                if i < symbols.len() {
                    symbol[bin] = symbols[i];
                }
            }
        }

        // Null DC if configured
        if self.config.dc_null {
            symbol[0] = Complex64::new(0.0, 0.0);
        }

        symbol
    }

    /// Allocate a frame of data across multiple OFDM symbols.
    ///
    /// Any sync words are prepended. Data is split into chunks of
    /// `num_data_carriers`, with zero-padding on the last symbol if needed.
    pub fn allocate_frame(&mut self, data: &[Complex64]) -> Vec<Vec<Complex64>> {
        let mut frame = Vec::new();

        // Insert sync words first
        for sw in &self.config.sync_words {
            frame.push(sw.clone());
        }

        let ndc = self.config.num_data_carriers();
        if ndc == 0 {
            return frame;
        }

        let num_symbols = (data.len() + ndc - 1) / ndc;
        for s in 0..num_symbols {
            let start = s * ndc;
            let end = (start + ndc).min(data.len());
            let mut chunk = data[start..end].to_vec();
            // Zero-pad last symbol if needed
            chunk.resize(ndc, Complex64::new(0.0, 0.0));
            frame.push(self.allocate(&chunk));
            self.symbol_index += 1;
        }

        frame
    }
}

/// RX-side OFDM carrier serializer (extractor).
#[derive(Debug, Clone)]
pub struct CarrierSerializer {
    config: CarrierAllocatorConfig,
}

impl CarrierSerializer {
    pub fn new(config: CarrierAllocatorConfig) -> Self {
        Self { config }
    }

    /// Extract data subcarriers from one OFDM symbol (FFT output).
    pub fn extract_data(&self, fft_output: &[Complex64]) -> Vec<Complex64> {
        assert_eq!(fft_output.len(), self.config.fft_size);
        self.config
            .data_carriers
            .iter()
            .map(|&idx| {
                let bin = self.config.to_fft_bin(idx);
                fft_output[bin]
            })
            .collect()
    }

    /// Extract pilot subcarriers from one OFDM symbol.
    pub fn extract_pilots(&self, fft_output: &[Complex64], symbol_index: usize) -> Vec<Complex64> {
        assert_eq!(fft_output.len(), self.config.fft_size);
        if self.config.pilot_carriers.is_empty() {
            return vec![];
        }
        let pc_idx = symbol_index % self.config.pilot_carriers.len();
        self.config.pilot_carriers[pc_idx]
            .iter()
            .map(|&idx| {
                let bin = self.config.to_fft_bin(idx);
                fft_output[bin]
            })
            .collect()
    }

    /// Extract data from multiple OFDM symbols.
    pub fn extract_frame(&self, symbols: &[Vec<Complex64>]) -> Vec<Complex64> {
        symbols.iter().flat_map(|s| self.extract_data(s)).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_round_trip_identity() {
        let config = CarrierAllocatorConfig::simple(64, &[1, 2, 3, 4, 5, 6, 7, 8], &[10, 20], 1.0);
        let alloc = CarrierAllocator::new(config.clone());
        let ser = CarrierSerializer::new(config);

        let data: Vec<Complex64> = (0..8).map(|i| Complex64::new(i as f64, 0.0)).collect();
        let symbol = alloc.allocate(&data);
        let extracted = ser.extract_data(&symbol);
        assert_eq!(extracted, data);
    }

    #[test]
    fn test_pilot_insertion() {
        let pilots = vec![10, 20];
        let config =
            CarrierAllocatorConfig::simple(64, &[1, 2, 3, 4, 5, 6, 7, 8], &pilots, 1.0);
        let alloc = CarrierAllocator::new(config);
        let data = vec![Complex64::new(0.5, 0.0); 8];
        let symbol = alloc.allocate(&data);
        // Check pilot positions
        assert_eq!(symbol[10], Complex64::new(1.0, 0.0));
        assert_eq!(symbol[20], Complex64::new(1.0, 0.0));
    }

    #[test]
    fn test_null_subcarriers() {
        let config = CarrierAllocatorConfig::simple(64, &[1, 2, 3], &[5], 1.0);
        let alloc = CarrierAllocator::new(config);
        let data = vec![Complex64::new(1.0, 0.0); 3];
        let symbol = alloc.allocate(&data);
        // DC should be null
        assert_eq!(symbol[0], Complex64::new(0.0, 0.0));
        // Guard bands (unused) should be zero
        assert_eq!(symbol[30], Complex64::new(0.0, 0.0));
        assert_eq!(symbol[50], Complex64::new(0.0, 0.0));
    }

    #[test]
    fn test_negative_index_mapping() {
        let config = CarrierAllocatorConfig::simple(64, &[-1, -2, -3, 1, 2, 3], &[], 0.0);
        let alloc = CarrierAllocator::new(config.clone());
        let data: Vec<Complex64> = (0..6).map(|i| Complex64::new(i as f64 + 1.0, 0.0)).collect();
        let symbol = alloc.allocate(&data);
        // -1 maps to bin 63, -2 to 62, -3 to 61
        assert_eq!(symbol[63], Complex64::new(1.0, 0.0));
        assert_eq!(symbol[62], Complex64::new(2.0, 0.0));
        assert_eq!(symbol[61], Complex64::new(3.0, 0.0));
        assert_eq!(symbol[1], Complex64::new(4.0, 0.0));
        assert_eq!(symbol[2], Complex64::new(5.0, 0.0));
        assert_eq!(symbol[3], Complex64::new(6.0, 0.0));
    }

    #[test]
    fn test_multi_symbol_frame() {
        let config = CarrierAllocatorConfig::simple(64, &[1, 2, 3, 4], &[], 0.0);
        let mut alloc = CarrierAllocator::new(config.clone());
        let ser = CarrierSerializer::new(config);

        // 10 data symbols, 4 per OFDM symbol → 3 symbols (last zero-padded)
        let data: Vec<Complex64> = (0..10).map(|i| Complex64::new(i as f64, 0.0)).collect();
        let frame = alloc.allocate_frame(&data);
        assert_eq!(frame.len(), 3);

        let extracted = ser.extract_frame(&frame);
        // First 10 should match, last 2 are zero-padded
        for i in 0..10 {
            assert_eq!(extracted[i], data[i]);
        }
        for i in 10..12 {
            assert_eq!(extracted[i], Complex64::new(0.0, 0.0));
        }
    }

    #[test]
    fn test_wifi_80211a_preset() {
        let config = CarrierAllocatorConfig::wifi_80211a();
        assert_eq!(config.fft_size, 64);
        assert_eq!(config.data_carriers.len(), 48);
        assert_eq!(config.pilot_carriers[0].len(), 4);
    }

    #[test]
    fn test_lte_rb_preset() {
        let config = CarrierAllocatorConfig::lte_rb(6, 128);
        assert_eq!(config.fft_size, 128);
        // 6 RBs = 72 subcarriers, minus DC, minus pilots
        assert!(config.data_carriers.len() > 50);
        assert!(!config.pilot_carriers[0].is_empty());
    }

    #[test]
    fn test_zero_padding_last_symbol() {
        let config = CarrierAllocatorConfig::simple(64, &[1, 2, 3, 4, 5], &[], 0.0);
        let mut alloc = CarrierAllocator::new(config.clone());
        let ser = CarrierSerializer::new(config);

        // 7 data symbols, 5 per OFDM symbol → 2 symbols (last has 2 data + 3 zeros)
        let data: Vec<Complex64> = (0..7).map(|i| Complex64::new(i as f64 + 1.0, 0.0)).collect();
        let frame = alloc.allocate_frame(&data);
        assert_eq!(frame.len(), 2);

        let extracted = ser.extract_frame(&frame);
        assert_eq!(extracted.len(), 10);
        for i in 0..7 {
            assert_eq!(extracted[i], data[i]);
        }
        for i in 7..10 {
            assert_eq!(extracted[i], Complex64::new(0.0, 0.0));
        }
    }

    #[test]
    fn test_sync_word_insertion() {
        let mut config = CarrierAllocatorConfig::simple(64, &[1, 2, 3, 4], &[], 0.0);
        let sync_word: Vec<Complex64> = (0..64).map(|i| Complex64::new(i as f64, 0.0)).collect();
        config.sync_words.push(sync_word.clone());

        let mut alloc = CarrierAllocator::new(config);
        let data = vec![Complex64::new(1.0, 0.0); 4];
        let frame = alloc.allocate_frame(&data);
        assert_eq!(frame.len(), 2); // 1 sync + 1 data
        assert_eq!(frame[0], sync_word);
    }

    #[test]
    fn test_pilot_extraction() {
        let config = CarrierAllocatorConfig::simple(64, &[1, 2, 3], &[10, 20], 1.0);
        let alloc = CarrierAllocator::new(config.clone());
        let ser = CarrierSerializer::new(config);

        let data = vec![Complex64::new(0.5, 0.0); 3];
        let symbol = alloc.allocate(&data);
        let pilots = ser.extract_pilots(&symbol, 0);
        assert_eq!(pilots.len(), 2);
        assert_eq!(pilots[0], Complex64::new(1.0, 0.0));
        assert_eq!(pilots[1], Complex64::new(1.0, 0.0));
    }
}

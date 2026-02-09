//! OFDM Modulator and Demodulator
//!
//! Orthogonal Frequency Division Multiplexing (OFDM) for multi-carrier
//! transmission. Used in Wi-Fi, LTE, DVB-T, and DAB.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ofdm::{OfdmModulator, OfdmConfig};
//! use num_complex::Complex64;
//!
//! let config = OfdmConfig::wifi_like();
//! let mut ofdm_tx = OfdmModulator::new(config.clone());
//!
//! // Map data to subcarriers (QPSK symbols)
//! let v = std::f64::consts::FRAC_1_SQRT_2;
//! let data_symbols: Vec<Complex64> = (0..48).map(|i| {
//!     Complex64::new(if i % 2 == 0 { v } else { -v }, v)
//! }).collect();
//!
//! let ofdm_symbol = ofdm_tx.modulate_symbol(&data_symbols);
//! assert_eq!(ofdm_symbol.len(), 64 + 16); // FFT size + CP length
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// OFDM configuration.
#[derive(Debug, Clone)]
pub struct OfdmConfig {
    /// FFT size (total number of subcarriers). Default: 64
    pub fft_size: usize,
    /// Cyclic prefix length in samples. Default: 16
    pub cp_len: usize,
    /// Number of data subcarriers. Default: 48
    pub num_data_carriers: usize,
    /// Number of pilot subcarriers. Default: 4
    pub num_pilot_carriers: usize,
    /// Pilot subcarrier indices (within FFT). Default: [11, 25, 39, 53] for WiFi-like
    pub pilot_indices: Vec<usize>,
    /// Data subcarrier indices (within FFT)
    pub data_indices: Vec<usize>,
    /// Pilot symbol value. Default: 1+0j
    pub pilot_value: Complex64,
}

impl OfdmConfig {
    /// Wi-Fi-like OFDM configuration (IEEE 802.11a-like).
    /// 64 subcarriers, 48 data, 4 pilots, 16-sample CP.
    pub fn wifi_like() -> Self {
        // Subcarrier layout: [-26..-1, 0, 1..26] mapped to FFT bins
        // DC (0) and guard bands are null
        // Pilots at subcarrier indices: -21, -7, 7, 21
        let pilot_indices = vec![11, 25, 39, 53]; // FFT bin indices for pilots
        let null_indices: Vec<usize> = vec![0, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37]; // DC + guards

        let mut data_indices: Vec<usize> = Vec::new();
        for i in 0..64 {
            if !null_indices.contains(&i) && !pilot_indices.contains(&i) {
                data_indices.push(i);
            }
        }

        Self {
            fft_size: 64,
            cp_len: 16,
            num_data_carriers: data_indices.len(),
            num_pilot_carriers: 4,
            pilot_indices,
            data_indices,
            pilot_value: Complex64::new(1.0, 0.0),
        }
    }

    /// Simple OFDM with minimal guard bands.
    pub fn simple(fft_size: usize, cp_len: usize) -> Self {
        let data_indices: Vec<usize> = (1..fft_size).collect(); // All except DC
        Self {
            fft_size,
            cp_len,
            num_data_carriers: data_indices.len(),
            num_pilot_carriers: 0,
            pilot_indices: vec![],
            data_indices,
            pilot_value: Complex64::new(1.0, 0.0),
        }
    }

    /// DVB-T-like OFDM (2K mode simplified).
    pub fn dvbt_2k() -> Self {
        let fft_size = 2048;
        let cp_len = 512; // 1/4 guard interval
        let pilot_indices: Vec<usize> = (0..142).map(|i| 12 * i + 1).filter(|&i| i < fft_size).collect();
        let null_dc = vec![0, 1024]; // DC and Nyquist
        let mut data_indices = Vec::new();
        for i in 1..fft_size {
            if !pilot_indices.contains(&i) && !null_dc.contains(&i) {
                data_indices.push(i);
            }
        }

        Self {
            fft_size,
            cp_len,
            num_data_carriers: data_indices.len(),
            num_pilot_carriers: pilot_indices.len(),
            pilot_indices,
            data_indices,
            pilot_value: Complex64::new(1.0, 0.0),
        }
    }
}

/// OFDM Modulator (transmitter).
#[derive(Debug, Clone)]
pub struct OfdmModulator {
    config: OfdmConfig,
}

impl OfdmModulator {
    /// Create a new OFDM modulator.
    pub fn new(config: OfdmConfig) -> Self {
        Self { config }
    }

    /// Modulate one OFDM symbol from data subcarrier values.
    ///
    /// `data_symbols` length must equal `num_data_carriers`.
    /// Returns FFT_size + CP_len samples (time domain).
    pub fn modulate_symbol(&self, data_symbols: &[Complex64]) -> Vec<Complex64> {
        assert_eq!(
            data_symbols.len(),
            self.config.num_data_carriers,
            "Data symbols count must match num_data_carriers"
        );

        let n = self.config.fft_size;

        // Build frequency-domain OFDM symbol
        let mut freq = vec![Complex64::new(0.0, 0.0); n];

        // Place data symbols
        for (i, &idx) in self.config.data_indices.iter().enumerate() {
            if i < data_symbols.len() {
                freq[idx] = data_symbols[i];
            }
        }

        // Place pilots
        for &idx in &self.config.pilot_indices {
            freq[idx] = self.config.pilot_value;
        }

        // IFFT to get time-domain samples
        let mut time = self.ifft(&freq);

        // Normalize
        let scale = 1.0 / (n as f64).sqrt();
        for s in time.iter_mut() {
            *s *= scale;
        }

        // Add cyclic prefix
        let cp_len = self.config.cp_len;
        let mut output = Vec::with_capacity(n + cp_len);
        output.extend_from_slice(&time[n - cp_len..]);
        output.extend_from_slice(&time);

        output
    }

    /// Modulate multiple OFDM symbols from a stream of data symbols.
    pub fn modulate(&self, data: &[Complex64]) -> Vec<Complex64> {
        let chunk_size = self.config.num_data_carriers;
        let mut output = Vec::new();

        for chunk in data.chunks(chunk_size) {
            if chunk.len() == chunk_size {
                output.extend(self.modulate_symbol(chunk));
            } else {
                // Pad last symbol with zeros
                let mut padded = chunk.to_vec();
                padded.resize(chunk_size, Complex64::new(0.0, 0.0));
                output.extend(self.modulate_symbol(&padded));
            }
        }

        output
    }

    fn ifft(&self, input: &[Complex64]) -> Vec<Complex64> {
        let n = input.len();
        let mut output = vec![Complex64::new(0.0, 0.0); n];
        for k in 0..n {
            for i in 0..n {
                let angle = 2.0 * PI * k as f64 * i as f64 / n as f64;
                let twiddle = Complex64::new(angle.cos(), angle.sin());
                output[k] += input[i] * twiddle;
            }
            output[k] /= n as f64;
        }
        output
    }
}

/// OFDM Demodulator (receiver).
#[derive(Debug, Clone)]
pub struct OfdmDemodulator {
    config: OfdmConfig,
}

impl OfdmDemodulator {
    /// Create a new OFDM demodulator.
    pub fn new(config: OfdmConfig) -> Self {
        Self { config }
    }

    /// Demodulate one OFDM symbol.
    ///
    /// Input should be FFT_size + CP_len samples.
    /// Returns the data subcarrier values.
    pub fn demodulate_symbol(&self, received: &[Complex64]) -> Vec<Complex64> {
        let n = self.config.fft_size;
        let cp_len = self.config.cp_len;

        assert!(
            received.len() >= n + cp_len,
            "Input too short: {} < {}",
            received.len(),
            n + cp_len
        );

        // Remove cyclic prefix
        let symbol = &received[cp_len..cp_len + n];

        // FFT
        let mut freq = self.fft(symbol);

        // Normalize
        let scale = (n as f64).sqrt();
        for s in freq.iter_mut() {
            *s *= scale;
        }

        // Extract data subcarriers
        self.config
            .data_indices
            .iter()
            .map(|&idx| freq[idx])
            .collect()
    }

    /// Demodulate a stream of OFDM symbols.
    pub fn demodulate(&self, received: &[Complex64]) -> Vec<Complex64> {
        let symbol_len = self.config.fft_size + self.config.cp_len;
        let mut output = Vec::new();

        for chunk in received.chunks(symbol_len) {
            if chunk.len() == symbol_len {
                output.extend(self.demodulate_symbol(chunk));
            }
        }

        output
    }

    /// Extract pilot values from a received OFDM symbol (for channel estimation).
    pub fn extract_pilots(&self, received: &[Complex64]) -> Vec<Complex64> {
        let n = self.config.fft_size;
        let cp_len = self.config.cp_len;

        if received.len() < n + cp_len {
            return vec![];
        }

        let symbol = &received[cp_len..cp_len + n];
        let mut freq = self.fft(symbol);
        let scale = (n as f64).sqrt();
        for s in freq.iter_mut() {
            *s *= scale;
        }

        self.config
            .pilot_indices
            .iter()
            .map(|&idx| freq[idx])
            .collect()
    }

    fn fft(&self, input: &[Complex64]) -> Vec<Complex64> {
        let n = input.len();
        let mut output = vec![Complex64::new(0.0, 0.0); n];
        for k in 0..n {
            for i in 0..n {
                let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
                let twiddle = Complex64::new(angle.cos(), angle.sin());
                output[k] += input[i] * twiddle;
            }
        }
        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ofdm_roundtrip_simple() {
        let config = OfdmConfig::simple(16, 4);
        let tx = OfdmModulator::new(config.clone());
        let rx = OfdmDemodulator::new(config.clone());

        // Create QPSK data for all data subcarriers
        let v = std::f64::consts::FRAC_1_SQRT_2;
        let data: Vec<Complex64> = (0..config.num_data_carriers)
            .map(|i| {
                Complex64::new(
                    if i % 2 == 0 { v } else { -v },
                    if i % 3 == 0 { v } else { -v },
                )
            })
            .collect();

        let transmitted = tx.modulate_symbol(&data);
        assert_eq!(transmitted.len(), 16 + 4);

        let recovered = rx.demodulate_symbol(&transmitted);
        assert_eq!(recovered.len(), data.len());

        // Check roundtrip
        for (orig, recv) in data.iter().zip(recovered.iter()) {
            assert!(
                (orig - recv).norm() < 0.01,
                "OFDM roundtrip failed: orig={orig:.3}, recv={recv:.3}"
            );
        }
    }

    #[test]
    fn test_ofdm_wifi_roundtrip() {
        let config = OfdmConfig::wifi_like();
        let tx = OfdmModulator::new(config.clone());
        let rx = OfdmDemodulator::new(config.clone());

        let v = std::f64::consts::FRAC_1_SQRT_2;
        let data: Vec<Complex64> = (0..config.num_data_carriers)
            .map(|i| Complex64::new(if i % 2 == 0 { v } else { -v }, v))
            .collect();

        let transmitted = tx.modulate_symbol(&data);
        assert_eq!(transmitted.len(), 64 + 16);

        let recovered = rx.demodulate_symbol(&transmitted);
        assert_eq!(recovered.len(), config.num_data_carriers);

        for (orig, recv) in data.iter().zip(recovered.iter()) {
            assert!(
                (orig - recv).norm() < 0.01,
                "WiFi OFDM roundtrip failed"
            );
        }
    }

    #[test]
    fn test_ofdm_multi_symbol() {
        let config = OfdmConfig::simple(8, 2);
        let tx = OfdmModulator::new(config.clone());
        let rx = OfdmDemodulator::new(config.clone());

        let data: Vec<Complex64> = (0..21)
            .map(|i| Complex64::new((i as f64 * 0.1).cos(), (i as f64 * 0.1).sin()))
            .collect();

        let transmitted = tx.modulate(&data);
        let recovered = rx.demodulate(&transmitted);

        // Should recover at least 2 full symbols worth
        assert!(
            recovered.len() >= config.num_data_carriers * 2,
            "Should recover multiple symbols: got {}",
            recovered.len()
        );
    }

    #[test]
    fn test_ofdm_cyclic_prefix() {
        let config = OfdmConfig::simple(8, 2);
        let tx = OfdmModulator::new(config.clone());

        let data: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); config.num_data_carriers];
        let output = tx.modulate_symbol(&data);

        // The CP should be a copy of the last cp_len samples of the OFDM body
        let cp = &output[..config.cp_len];
        let body_end = &output[config.fft_size..config.fft_size + config.cp_len];

        for (a, b) in cp.iter().zip(body_end.iter()) {
            assert!(
                (a - b).norm() < 1e-10,
                "CP should match end of OFDM symbol"
            );
        }
    }

    #[test]
    fn test_ofdm_pilot_extraction() {
        let config = OfdmConfig::wifi_like();
        let tx = OfdmModulator::new(config.clone());
        let rx = OfdmDemodulator::new(config.clone());

        let data: Vec<Complex64> =
            vec![Complex64::new(0.5, 0.5); config.num_data_carriers];
        let transmitted = tx.modulate_symbol(&data);

        let pilots = rx.extract_pilots(&transmitted);
        assert_eq!(pilots.len(), config.num_pilot_carriers);

        // Pilots should be close to the pilot_value
        for (i, p) in pilots.iter().enumerate() {
            assert!(
                (p - config.pilot_value).norm() < 0.01,
                "Pilot {i} should be {:.3}: got {p:.3}",
                config.pilot_value
            );
        }
    }

    #[test]
    fn test_ofdm_config_dimensions() {
        let config = OfdmConfig::wifi_like();
        assert_eq!(config.fft_size, 64);
        assert_eq!(config.cp_len, 16);
        assert_eq!(config.num_pilot_carriers, 4);
        assert_eq!(config.pilot_indices.len(), 4);
        assert!(config.num_data_carriers > 0);
        assert_eq!(
            config.num_data_carriers + config.num_pilot_carriers + 12,
            64,
            "Data + pilots + nulls should equal FFT size"
        );
    }
}

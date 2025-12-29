//! OFDM (Orthogonal Frequency Division Multiplexing)
//!
//! OFDM is a multi-carrier modulation scheme that divides the available
//! bandwidth into many narrow subcarriers. Each subcarrier carries a
//! low-rate data stream using PSK or QAM modulation.
//!
//! ## Key Concepts
//!
//! - **Subcarriers**: Many parallel narrowband channels
//! - **Orthogonality**: Subcarrier spacing = 1/symbol_duration ensures no interference
//! - **FFT/IFFT**: Efficient implementation using Fast Fourier Transform
//! - **Cyclic Prefix**: Guard interval to combat multipath
//!
//! ## Applications
//!
//! - WiFi (802.11a/g/n/ac/ax)
//! - LTE and 5G cellular
//! - DVB-T/T2 digital television
//! - DAB digital radio
//! - DSL broadband
//!
//! ## Mathematical Basis
//!
//! ```text
//! Transmit: s(t) = IFFT{ X[k] }  (frequency domain → time domain)
//! Receive:  X[k] = FFT{ s(t) }   (time domain → frequency domain)
//! ```
//!
//! The IFFT creates a time-domain signal where each subcarrier is
//! a complex exponential at frequency k/N * sample_rate.

use super::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};
use crate::types::IQSample;
use rustfft::{FftPlanner, num_complex::Complex};

/// Unpack bytes to individual bits (MSB first)
fn bytes_to_bits(data: &[u8]) -> Vec<u8> {
    let mut bits = Vec::with_capacity(data.len() * 8);
    for byte in data {
        for i in (0..8).rev() {
            bits.push((byte >> i) & 1);
        }
    }
    bits
}

/// Pack individual bits to bytes (MSB first)
fn bits_to_bytes(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8)
        .map(|chunk| {
            chunk.iter()
                .enumerate()
                .fold(0u8, |acc, (i, &bit)| {
                    acc | ((bit & 1) << (7 - i))
                })
        })
        .collect()
}

/// Subcarrier modulation type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SubcarrierModulation {
    /// BPSK: 1 bit per subcarrier
    Bpsk,
    /// QPSK: 2 bits per subcarrier
    Qpsk,
    /// 16-QAM: 4 bits per subcarrier
    Qam16,
    /// 64-QAM: 6 bits per subcarrier
    Qam64,
}

impl SubcarrierModulation {
    /// Bits per symbol for this modulation
    pub fn bits_per_symbol(&self) -> usize {
        match self {
            Self::Bpsk => 1,
            Self::Qpsk => 2,
            Self::Qam16 => 4,
            Self::Qam64 => 6,
        }
    }

    /// Map bits to constellation point
    pub fn modulate(&self, bits: &[u8]) -> IQSample {
        match self {
            Self::Bpsk => {
                let bit = bits.first().copied().unwrap_or(0) & 1;
                if bit == 0 {
                    IQSample::new(1.0, 0.0)
                } else {
                    IQSample::new(-1.0, 0.0)
                }
            }
            Self::Qpsk => {
                let b0 = bits.first().copied().unwrap_or(0) & 1;
                let b1 = bits.get(1).copied().unwrap_or(0) & 1;
                let scale = 1.0 / 2.0_f64.sqrt();
                let i = if b0 == 0 { scale } else { -scale };
                let q = if b1 == 0 { scale } else { -scale };
                IQSample::new(i, q)
            }
            Self::Qam16 => {
                let b0 = bits.first().copied().unwrap_or(0) & 1;
                let b1 = bits.get(1).copied().unwrap_or(0) & 1;
                let b2 = bits.get(2).copied().unwrap_or(0) & 1;
                let b3 = bits.get(3).copied().unwrap_or(0) & 1;
                // Gray coded 16-QAM
                let i_idx = (b0 << 1) | b1;
                let q_idx = (b2 << 1) | b3;
                let levels = [-3.0, -1.0, 3.0, 1.0]; // Gray coded
                let scale = 1.0 / 10.0_f64.sqrt(); // Normalize power
                IQSample::new(levels[i_idx as usize] * scale, levels[q_idx as usize] * scale)
            }
            Self::Qam64 => {
                let b0 = bits.first().copied().unwrap_or(0) & 1;
                let b1 = bits.get(1).copied().unwrap_or(0) & 1;
                let b2 = bits.get(2).copied().unwrap_or(0) & 1;
                let b3 = bits.get(3).copied().unwrap_or(0) & 1;
                let b4 = bits.get(4).copied().unwrap_or(0) & 1;
                let b5 = bits.get(5).copied().unwrap_or(0) & 1;
                let i_idx = (b0 << 2) | (b1 << 1) | b2;
                let q_idx = (b3 << 2) | (b4 << 1) | b5;
                let levels = [-7.0, -5.0, -1.0, -3.0, 7.0, 5.0, 1.0, 3.0]; // Gray coded
                let scale = 1.0 / 42.0_f64.sqrt(); // Normalize power
                IQSample::new(levels[i_idx as usize] * scale, levels[q_idx as usize] * scale)
            }
        }
    }

    /// Demodulate constellation point to bits
    pub fn demodulate(&self, sample: IQSample) -> Vec<u8> {
        match self {
            Self::Bpsk => {
                vec![if sample.re >= 0.0 { 0 } else { 1 }]
            }
            Self::Qpsk => {
                vec![
                    if sample.re >= 0.0 { 0 } else { 1 },
                    if sample.im >= 0.0 { 0 } else { 1 },
                ]
            }
            Self::Qam16 => {
                // Reverse Gray coding for 16-QAM
                let i_level = if sample.re < -0.4 { 0 }
                    else if sample.re < 0.0 { 1 }
                    else if sample.re < 0.4 { 3 }
                    else { 2 };
                let q_level = if sample.im < -0.4 { 0 }
                    else if sample.im < 0.0 { 1 }
                    else if sample.im < 0.4 { 3 }
                    else { 2 };
                vec![
                    ((i_level >> 1) & 1) as u8,
                    (i_level & 1) as u8,
                    ((q_level >> 1) & 1) as u8,
                    (q_level & 1) as u8,
                ]
            }
            Self::Qam64 => {
                // Simplified 64-QAM demodulation
                let scale = 42.0_f64.sqrt();
                let i_scaled = sample.re * scale;
                let q_scaled = sample.im * scale;

                let i_level = if i_scaled < -6.0 { 0 }
                    else if i_scaled < -4.0 { 1 }
                    else if i_scaled < -2.0 { 3 }
                    else if i_scaled < 0.0 { 2 }
                    else if i_scaled < 2.0 { 6 }
                    else if i_scaled < 4.0 { 7 }
                    else if i_scaled < 6.0 { 5 }
                    else { 4 };
                let q_level = if q_scaled < -6.0 { 0 }
                    else if q_scaled < -4.0 { 1 }
                    else if q_scaled < -2.0 { 3 }
                    else if q_scaled < 0.0 { 2 }
                    else if q_scaled < 2.0 { 6 }
                    else if q_scaled < 4.0 { 7 }
                    else if q_scaled < 6.0 { 5 }
                    else { 4 };
                vec![
                    ((i_level >> 2) & 1) as u8,
                    ((i_level >> 1) & 1) as u8,
                    (i_level & 1) as u8,
                    ((q_level >> 2) & 1) as u8,
                    ((q_level >> 1) & 1) as u8,
                    (q_level & 1) as u8,
                ]
            }
        }
    }
}

/// OFDM modulator/demodulator
#[derive(Debug, Clone)]
pub struct OFDM {
    /// Common waveform parameters
    common: CommonParams,
    /// Number of FFT points (total subcarriers including guard bands)
    fft_size: usize,
    /// Number of data subcarriers (active subcarriers)
    num_data_subcarriers: usize,
    /// Cyclic prefix length (samples)
    cyclic_prefix_len: usize,
    /// Subcarrier modulation type
    subcarrier_mod: SubcarrierModulation,
    /// Subcarrier spacing in Hz
    subcarrier_spacing: f64,
}

impl OFDM {
    /// Create a new OFDM modulator
    ///
    /// # Parameters
    /// - `common`: Common waveform parameters
    /// - `fft_size`: FFT size (e.g., 64, 256, 1024, 2048)
    /// - `num_data_subcarriers`: Number of data-carrying subcarriers
    /// - `cyclic_prefix_ratio`: CP length as fraction of symbol (e.g., 0.25 for 1/4)
    /// - `subcarrier_mod`: Modulation for each subcarrier
    pub fn new(
        common: CommonParams,
        fft_size: usize,
        num_data_subcarriers: usize,
        cyclic_prefix_ratio: f64,
        subcarrier_mod: SubcarrierModulation,
    ) -> Self {
        let cyclic_prefix_len = (fft_size as f64 * cyclic_prefix_ratio) as usize;
        let subcarrier_spacing = common.sample_rate / fft_size as f64;

        Self {
            common,
            fft_size,
            num_data_subcarriers: num_data_subcarriers.min(fft_size - 1),
            cyclic_prefix_len,
            subcarrier_mod,
            subcarrier_spacing,
        }
    }

    /// Create OFDM with WiFi-like parameters (64 subcarriers, 48 data)
    pub fn wifi_like(sample_rate: f64) -> Self {
        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        Self::new(common, 64, 48, 0.25, SubcarrierModulation::Qpsk)
    }

    /// Create OFDM with simple parameters for demonstration
    pub fn simple(sample_rate: f64) -> Self {
        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        Self::new(common, 64, 52, 0.25, SubcarrierModulation::Qpsk)
    }

    /// Get bits per OFDM symbol
    pub fn bits_per_symbol(&self) -> usize {
        self.num_data_subcarriers * self.subcarrier_mod.bits_per_symbol()
    }

    /// Get OFDM symbol duration in seconds
    pub fn symbol_duration(&self) -> f64 {
        (self.fft_size + self.cyclic_prefix_len) as f64 / self.common.sample_rate
    }

    /// Get total bandwidth in Hz
    pub fn bandwidth(&self) -> f64 {
        self.subcarrier_spacing * self.fft_size as f64
    }

    /// Get data rate in bits per second
    pub fn data_rate(&self) -> f64 {
        self.bits_per_symbol() as f64 / self.symbol_duration()
    }

    /// Map data subcarrier index to FFT bin index
    /// Uses centered subcarrier allocation (DC null, guard bands)
    fn data_to_fft_index(&self, data_idx: usize) -> usize {
        let half = self.num_data_subcarriers / 2;
        if data_idx < half {
            // Negative frequencies (upper half of FFT)
            self.fft_size - half + data_idx
        } else {
            // Positive frequencies (lower half of FFT, skip DC)
            data_idx - half + 1
        }
    }

    /// Modulate one OFDM symbol from data subcarrier values
    fn modulate_symbol(&self, subcarriers: &[IQSample]) -> Vec<IQSample> {
        let mut freq_domain: Vec<Complex<f64>> = vec![Complex::new(0.0, 0.0); self.fft_size];

        // Place data on subcarriers
        for (i, &sample) in subcarriers.iter().take(self.num_data_subcarriers).enumerate() {
            let fft_idx = self.data_to_fft_index(i);
            freq_domain[fft_idx] = Complex::new(sample.re, sample.im);
        }

        // IFFT: frequency domain → time domain
        let mut planner = FftPlanner::new();
        let ifft = planner.plan_fft_inverse(self.fft_size);
        ifft.process(&mut freq_domain);

        // Normalize IFFT output
        let scale = 1.0 / (self.fft_size as f64).sqrt();

        // Convert to IQSample and add cyclic prefix
        let time_samples: Vec<IQSample> = freq_domain
            .iter()
            .map(|c| IQSample::new(c.re * scale * self.common.amplitude, c.im * scale * self.common.amplitude))
            .collect();

        // Add cyclic prefix (copy end to beginning)
        let mut symbol = Vec::with_capacity(self.fft_size + self.cyclic_prefix_len);
        symbol.extend_from_slice(&time_samples[self.fft_size - self.cyclic_prefix_len..]);
        symbol.extend_from_slice(&time_samples);

        symbol
    }

    /// Demodulate one OFDM symbol to data subcarrier values
    fn demodulate_symbol(&self, samples: &[IQSample]) -> Vec<IQSample> {
        if samples.len() < self.fft_size + self.cyclic_prefix_len {
            return vec![];
        }

        // Remove cyclic prefix
        let time_samples = &samples[self.cyclic_prefix_len..self.cyclic_prefix_len + self.fft_size];

        // Convert to Complex for FFT
        let mut freq_domain: Vec<Complex<f64>> = time_samples
            .iter()
            .map(|s| Complex::new(s.re, s.im))
            .collect();

        // FFT: time domain → frequency domain
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(self.fft_size);
        fft.process(&mut freq_domain);

        // Normalize and extract data subcarriers
        let scale = 1.0 / (self.fft_size as f64).sqrt() / self.common.amplitude;

        let mut subcarriers = Vec::with_capacity(self.num_data_subcarriers);
        for i in 0..self.num_data_subcarriers {
            let fft_idx = self.data_to_fft_index(i);
            let c = freq_domain[fft_idx];
            subcarriers.push(IQSample::new(c.re * scale, c.im * scale));
        }

        subcarriers
    }
}

impl Waveform for OFDM {
    fn info(&self) -> WaveformInfo {
        // For OFDM, bits_per_symbol can exceed 255 (e.g., 48 subcarriers * 6 bits = 288)
        // Saturate to u8::MAX to indicate "many bits per symbol"
        let bps = self.bits_per_symbol();
        WaveformInfo {
            name: "OFDM",
            full_name: "Orthogonal Frequency Division Multiplexing",
            description: "Multi-carrier modulation using FFT - foundation of WiFi, LTE, 5G",
            complexity: 5,
            bits_per_symbol: bps.min(255) as u8,
            carries_data: true,
            characteristics: &[
                "Multi-carrier (parallel subchannels)",
                "FFT/IFFT-based implementation",
                "Cyclic prefix for multipath immunity",
                "High spectral efficiency",
                "Used in WiFi, LTE, 5G, DVB-T",
            ],
            history: "OFDM was first proposed in the 1960s but became practical with efficient \
                FFT algorithms. Bell Labs patented it in 1970. Digital implementation emerged \
                in the 1980s. OFDM revolutionized wireless: IEEE 802.11a (1999), DVB-T (1997), \
                LTE (2009), and 5G-NR (2018) all use OFDM variants.",
            modern_usage: "OFDM dominates modern wireless: WiFi (802.11a/g/n/ac/ax), all LTE \
                and 5G cellular, digital TV (DVB-T/T2, ATSC 3.0), digital radio (DAB), DSL \
                broadband, and powerline networking. OFDMA variant enables multi-user access. \
                Nearly all high-speed wireless systems use OFDM.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // Convert packed bytes to individual bits
        let bits = bytes_to_bits(data);

        let bits_per_subcarrier = self.subcarrier_mod.bits_per_symbol();
        let bits_per_ofdm_symbol = self.bits_per_symbol();

        let mut samples = Vec::new();

        // Process bits in chunks of bits_per_ofdm_symbol
        for ofdm_symbol_bits in bits.chunks(bits_per_ofdm_symbol) {
            // Map bits to subcarrier constellation points
            let mut subcarriers = Vec::with_capacity(self.num_data_subcarriers);

            for sc_bits in ofdm_symbol_bits.chunks(bits_per_subcarrier) {
                let constellation_point = self.subcarrier_mod.modulate(sc_bits);
                subcarriers.push(constellation_point);
            }

            // Pad with zeros if not enough data
            while subcarriers.len() < self.num_data_subcarriers {
                subcarriers.push(IQSample::new(0.0, 0.0));
            }

            // Generate OFDM symbol
            let symbol_samples = self.modulate_symbol(&subcarriers);
            samples.extend(symbol_samples);
        }

        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let mut result = DemodResult::default();
        let symbol_len = self.fft_size + self.cyclic_prefix_len;

        let mut all_bits = Vec::new();

        // Process each OFDM symbol
        for symbol_samples in samples.chunks(symbol_len) {
            if symbol_samples.len() < symbol_len {
                break;
            }

            // Demodulate to subcarrier values
            let subcarriers = self.demodulate_symbol(symbol_samples);

            // Demodulate each subcarrier to bits, skipping zero-padded subcarriers
            // (subcarriers with very low energy were likely padding)
            let energy_threshold = 0.01; // Skip subcarriers below this energy
            for subcarrier in subcarriers {
                let energy = subcarrier.re * subcarrier.re + subcarrier.im * subcarrier.im;
                if energy < energy_threshold {
                    // This subcarrier was likely zero-padded, stop processing
                    break;
                }
                let bits = self.subcarrier_mod.demodulate(subcarrier);
                all_bits.extend(bits);
            }
        }

        // Convert individual bits back to packed bytes
        result.bits = bits_to_bytes(&all_bits);
        result
    }

    fn samples_per_symbol(&self) -> usize {
        self.fft_size + self.cyclic_prefix_len
    }

    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);

        // Generate constellation for subcarrier modulation
        let constellation = match self.subcarrier_mod {
            SubcarrierModulation::Bpsk => {
                vec![IQSample::new(1.0, 0.0), IQSample::new(-1.0, 0.0)]
            }
            SubcarrierModulation::Qpsk => {
                let s = 1.0 / 2.0_f64.sqrt();
                vec![
                    IQSample::new(s, s),
                    IQSample::new(s, -s),
                    IQSample::new(-s, s),
                    IQSample::new(-s, -s),
                ]
            }
            SubcarrierModulation::Qam16 => {
                let s = 1.0 / 10.0_f64.sqrt();
                let mut points = Vec::with_capacity(16);
                for i in [-3.0, -1.0, 1.0, 3.0] {
                    for q in [-3.0, -1.0, 1.0, 3.0] {
                        points.push(IQSample::new(i * s, q * s));
                    }
                }
                points
            }
            SubcarrierModulation::Qam64 => {
                let s = 1.0 / 42.0_f64.sqrt();
                let mut points = Vec::with_capacity(64);
                for i in [-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0] {
                    for q in [-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0] {
                        points.push(IQSample::new(i * s, q * s));
                    }
                }
                points
            }
        };

        let labels: Vec<String> = (0..constellation.len())
            .map(|i| format!("{:0width$b}", i, width = self.subcarrier_mod.bits_per_symbol()))
            .collect();

        VisualizationData {
            samples,
            constellation,
            constellation_labels: labels,
            spectrum: Vec::new(),
            description: format!(
                "OFDM: {} subcarriers, {:?}, {:.1} kbps",
                self.num_data_subcarriers,
                self.subcarrier_mod,
                self.data_rate() / 1000.0
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ofdm_basic() {
        let ofdm = OFDM::simple(1_000_000.0); // 1 MHz sample rate

        assert_eq!(ofdm.fft_size, 64);
        assert_eq!(ofdm.num_data_subcarriers, 52);
        assert!(ofdm.bits_per_symbol() > 0);
    }

    #[test]
    fn test_ofdm_roundtrip_qpsk() {
        let ofdm = OFDM::new(
            CommonParams {
                sample_rate: 1_000_000.0,
                carrier_freq: 0.0,
                amplitude: 1.0,
            },
            64,
            48,
            0.25,
            SubcarrierModulation::Qpsk,
        );

        // Create test data as packed bytes (enough for one OFDM symbol)
        // bits_per_symbol = 48 subcarriers * 2 bits = 96 bits = 12 bytes
        let data: Vec<u8> = vec![0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
                                  0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];

        let modulated = ofdm.modulate(&data);
        let result = ofdm.demodulate(&modulated);

        // Recovered bytes should match input
        assert_eq!(result.bits.len(), data.len());
        assert_eq!(result.bits, data);
    }

    #[test]
    fn test_ofdm_roundtrip_bpsk() {
        let ofdm = OFDM::new(
            CommonParams {
                sample_rate: 1_000_000.0,
                carrier_freq: 0.0,
                amplitude: 1.0,
            },
            64,
            48,
            0.25,
            SubcarrierModulation::Bpsk,
        );

        // Create test data as packed bytes
        // bits_per_symbol = 48 subcarriers * 1 bit = 48 bits = 6 bytes
        let data: Vec<u8> = vec![0xB2, 0xFC, 0x3D, 0x8A, 0x55, 0xAA];
        let modulated = ofdm.modulate(&data);
        let result = ofdm.demodulate(&modulated);

        assert_eq!(result.bits.len(), data.len());
        assert_eq!(result.bits, data);
    }

    #[test]
    fn test_subcarrier_modulation() {
        // Test QPSK
        let qpsk = SubcarrierModulation::Qpsk;
        let point = qpsk.modulate(&[0, 0]);
        let bits = qpsk.demodulate(point);
        assert_eq!(bits, vec![0, 0]);

        let point = qpsk.modulate(&[1, 1]);
        let bits = qpsk.demodulate(point);
        assert_eq!(bits, vec![1, 1]);
    }

    #[test]
    fn test_ofdm_symbol_duration() {
        let ofdm = OFDM::new(
            CommonParams {
                sample_rate: 20_000_000.0, // 20 MHz like WiFi
                carrier_freq: 0.0,
                amplitude: 1.0,
            },
            64,
            48,
            0.25, // 1/4 CP
            SubcarrierModulation::Qpsk,
        );

        // Symbol duration should be (64 + 16) / 20e6 = 4 μs
        let duration = ofdm.symbol_duration();
        assert!((duration - 4e-6).abs() < 1e-9);
    }

    #[test]
    fn test_wifi_like_params() {
        let ofdm = OFDM::wifi_like(20_000_000.0);

        // WiFi-like: 64 FFT, 48 data subcarriers
        assert_eq!(ofdm.fft_size, 64);
        assert_eq!(ofdm.num_data_subcarriers, 48);

        // Subcarrier spacing should be 20 MHz / 64 = 312.5 kHz
        assert!((ofdm.subcarrier_spacing - 312_500.0).abs() < 1.0);
    }
}

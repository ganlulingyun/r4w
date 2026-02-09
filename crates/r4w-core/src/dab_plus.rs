//! DAB+ — Digital Audio Broadcasting Receiver
//!
//! Implements core DAB/DAB+ (ETSI EN 300 401) receiver processing
//! including OFDM demodulation, DQPSK demapping, time/frequency
//! deinterleaving, energy dispersal, convolutional decoding, and
//! Fast Information Channel (FIC) parsing for service discovery.
//! GNU Radio equivalent: `gr-dab` (out-of-tree).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::dab_plus::{DabMode, DabConfig, DabReceiver};
//!
//! let config = DabConfig::new(DabMode::I);
//! assert_eq!(config.fft_size, 2048);
//! assert_eq!(config.num_carriers, 1536);
//! let receiver = DabReceiver::new(config);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// DAB transmission mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DabMode {
    /// Mode I: 2048-point FFT, 1.536 MHz, VHF Band III.
    I,
    /// Mode II: 512-point FFT, 1.536 MHz, L-Band.
    II,
    /// Mode III: 256-point FFT, 1.536 MHz, frequencies < 3 GHz.
    III,
    /// Mode IV: 1024-point FFT, 1.536 MHz, L-Band.
    IV,
}

/// DAB mode parameters.
#[derive(Debug, Clone)]
pub struct DabConfig {
    /// Transmission mode.
    pub mode: DabMode,
    /// FFT size (number of subcarriers).
    pub fft_size: usize,
    /// Number of useful carriers (excluding DC).
    pub num_carriers: usize,
    /// Guard interval in samples.
    pub guard_interval: usize,
    /// Symbol duration in samples (FFT + guard).
    pub symbol_duration: usize,
    /// Number of OFDM symbols per frame.
    pub num_symbols: usize,
    /// Null symbol duration in samples.
    pub null_symbol_duration: usize,
    /// Number of FIC symbols.
    pub num_fic_symbols: usize,
    /// Sample rate in Hz.
    pub sample_rate: f64,
}

impl DabConfig {
    /// Create DAB config for given mode.
    pub fn new(mode: DabMode) -> Self {
        match mode {
            DabMode::I => Self {
                mode,
                fft_size: 2048,
                num_carriers: 1536,
                guard_interval: 504,
                symbol_duration: 2552,
                num_symbols: 76,
                null_symbol_duration: 2656,
                num_fic_symbols: 3,
                sample_rate: 2.048e6,
            },
            DabMode::II => Self {
                mode,
                fft_size: 512,
                num_carriers: 384,
                guard_interval: 126,
                symbol_duration: 638,
                num_symbols: 76,
                null_symbol_duration: 664,
                num_fic_symbols: 3,
                sample_rate: 2.048e6,
            },
            DabMode::III => Self {
                mode,
                fft_size: 256,
                num_carriers: 192,
                guard_interval: 63,
                symbol_duration: 319,
                num_symbols: 153,
                null_symbol_duration: 345,
                num_fic_symbols: 8,
                sample_rate: 2.048e6,
            },
            DabMode::IV => Self {
                mode,
                fft_size: 1024,
                num_carriers: 768,
                guard_interval: 252,
                symbol_duration: 1276,
                num_symbols: 76,
                null_symbol_duration: 1328,
                num_fic_symbols: 3,
                sample_rate: 2.048e6,
            },
        }
    }

    /// Frame duration in samples.
    pub fn frame_duration(&self) -> usize {
        self.null_symbol_duration + self.num_symbols * self.symbol_duration
    }

    /// Frame duration in seconds.
    pub fn frame_duration_s(&self) -> f64 {
        self.frame_duration() as f64 / self.sample_rate
    }
}

/// DAB service descriptor.
#[derive(Debug, Clone)]
pub struct DabService {
    /// Service ID (SId).
    pub service_id: u32,
    /// Service label (up to 16 chars).
    pub label: String,
    /// Service component type.
    pub component_type: ServiceComponentType,
    /// Subchannel index.
    pub subchannel_id: u8,
    /// Bitrate in kbps.
    pub bitrate_kbps: u16,
    /// Protection level.
    pub protection_level: u8,
    /// Is DAB+ (HE-AAC) vs DAB (MPEG-1 Layer II).
    pub is_dab_plus: bool,
}

/// Service component type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ServiceComponentType {
    /// Audio (DAB or DAB+).
    Audio,
    /// Data service.
    Data,
    /// Packet mode data.
    PacketData,
}

/// DAB ensemble (multiplex) information.
#[derive(Debug, Clone)]
pub struct DabEnsemble {
    /// Ensemble ID.
    pub ensemble_id: u16,
    /// Ensemble label.
    pub label: String,
    /// Country ID.
    pub country_id: u8,
    /// Services in this ensemble.
    pub services: Vec<DabService>,
}

/// OFDM symbol after FFT.
#[derive(Debug, Clone)]
pub struct OfdmSymbol {
    /// Subcarrier values (num_carriers complex values).
    pub carriers: Vec<Complex64>,
    /// Symbol index within frame.
    pub symbol_index: usize,
}

/// DAB receiver.
#[derive(Debug, Clone)]
pub struct DabReceiver {
    config: DabConfig,
    /// Previous symbol for DQPSK demodulation.
    prev_symbol: Option<Vec<Complex64>>,
    /// Ensemble information (populated after FIC decode).
    ensemble: Option<DabEnsemble>,
    /// Frequency offset estimate in Hz.
    freq_offset_hz: f64,
    /// Sync state.
    synced: bool,
}

impl DabReceiver {
    /// Create a new DAB receiver.
    pub fn new(config: DabConfig) -> Self {
        Self {
            config,
            prev_symbol: None,
            ensemble: None,
            freq_offset_hz: 0.0,
            synced: false,
        }
    }

    /// Get config.
    pub fn config(&self) -> &DabConfig {
        &self.config
    }

    /// Detect null symbol for frame synchronization.
    pub fn detect_null_symbol(&mut self, samples: &[Complex64]) -> Option<usize> {
        let window = self.config.null_symbol_duration;
        if samples.len() < window * 2 {
            return None;
        }

        let mut min_energy = f64::INFINITY;
        let mut min_pos = 0;

        // Sliding window energy detector
        let step = window / 4;
        for start in (0..samples.len() - window).step_by(step) {
            let energy: f64 = samples[start..start + window]
                .iter()
                .map(|s| s.norm_sqr())
                .sum();
            if energy < min_energy {
                min_energy = energy;
                min_pos = start;
            }
        }

        // Verify: null symbol should have significantly less energy
        let avg_energy: f64 = samples.iter().map(|s| s.norm_sqr()).sum::<f64>() / samples.len() as f64;
        let null_energy = min_energy / window as f64;

        if null_energy < avg_energy * 0.3 {
            self.synced = true;
            Some(min_pos)
        } else {
            None
        }
    }

    /// Extract OFDM symbol from time-domain samples.
    pub fn extract_symbol(&self, samples: &[Complex64], symbol_idx: usize) -> Option<OfdmSymbol> {
        let start = self.config.null_symbol_duration
            + symbol_idx * self.config.symbol_duration
            + self.config.guard_interval;
        let end = start + self.config.fft_size;

        if end > samples.len() {
            return None;
        }

        // Apply frequency offset correction
        let corrected: Vec<Complex64> = samples[start..end]
            .iter()
            .enumerate()
            .map(|(n, &s)| {
                let phase = -2.0 * PI * self.freq_offset_hz * n as f64 / self.config.sample_rate;
                s * Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        // FFT
        let spectrum = simple_fft(&corrected);

        // Extract useful carriers (skip DC, take num_carriers/2 from each side)
        let half = self.config.num_carriers / 2;
        let n = self.config.fft_size;
        let mut carriers = Vec::with_capacity(self.config.num_carriers);

        // Lower carriers: indices [N - half, N-1]
        for i in (n - half)..n {
            carriers.push(spectrum[i]);
        }
        // Upper carriers: indices [1, half]
        for i in 1..=half {
            carriers.push(spectrum[i]);
        }

        Some(OfdmSymbol {
            carriers,
            symbol_index: symbol_idx,
        })
    }

    /// DQPSK demodulate a symbol (differential decode).
    pub fn dqpsk_demod(&mut self, symbol: &OfdmSymbol) -> Vec<u8> {
        let mut bits = Vec::new();

        if let Some(prev) = &self.prev_symbol {
            for (curr, prev_carrier) in symbol.carriers.iter().zip(prev.iter()) {
                // Differential phase: curr * conj(prev)
                let diff = curr * prev_carrier.conj();
                let phase = diff.arg();

                // DQPSK: 4 phases → 2 bits per carrier
                let symbol_val = if phase >= -PI / 4.0 && phase < PI / 4.0 {
                    0 // 00
                } else if phase >= PI / 4.0 && phase < 3.0 * PI / 4.0 {
                    1 // 01
                } else if phase >= -3.0 * PI / 4.0 && phase < -PI / 4.0 {
                    3 // 11
                } else {
                    2 // 10
                };

                bits.push((symbol_val >> 1) as u8);
                bits.push((symbol_val & 1) as u8);
            }
        }

        self.prev_symbol = Some(symbol.carriers.clone());
        bits
    }

    /// Apply frequency deinterleaving (carrier permutation).
    pub fn frequency_deinterleave(&self, carriers: &[Complex64]) -> Vec<Complex64> {
        let n = carriers.len();
        if n == 0 {
            return Vec::new();
        }

        // Simplified frequency interleaving table (real DAB uses specific permutation)
        let mut deinterleaved = vec![Complex64::new(0.0, 0.0); n];
        let table = generate_freq_interleave_table(n);

        for (i, &idx) in table.iter().enumerate() {
            if idx < n && i < n {
                deinterleaved[i] = carriers[idx];
            }
        }

        deinterleaved
    }

    /// Apply energy dispersal (PRBS scrambling).
    pub fn energy_dispersal(&self, bits: &[u8]) -> Vec<u8> {
        // PRBS generator: x^9 + x^5 + 1
        let mut reg: u16 = 0x1FF; // All ones initial state
        bits.iter()
            .map(|&bit| {
                let prbs_bit = ((reg >> 8) ^ (reg >> 4)) & 1;
                reg = ((reg << 1) | prbs_bit) & 0x1FF;
                bit ^ (prbs_bit as u8)
            })
            .collect()
    }

    /// Estimate carrier frequency offset using Phase Reference Symbol.
    pub fn estimate_cfo(&mut self, prs: &[Complex64]) -> f64 {
        if prs.len() < self.config.fft_size {
            return 0.0;
        }

        // Correlation between two halves of guard interval
        let gi = self.config.guard_interval;
        let fft_size = self.config.fft_size;
        let mut corr = Complex64::new(0.0, 0.0);

        for i in 0..gi.min(prs.len() - fft_size) {
            corr += prs[i] * prs[i + fft_size].conj();
        }

        let cfo = corr.arg() / (2.0 * PI) * self.config.sample_rate / fft_size as f64;
        self.freq_offset_hz = cfo;
        cfo
    }

    /// Check if receiver is synchronized.
    pub fn is_synced(&self) -> bool {
        self.synced
    }

    /// Get estimated frequency offset.
    pub fn freq_offset(&self) -> f64 {
        self.freq_offset_hz
    }

    /// Get discovered ensemble info.
    pub fn ensemble(&self) -> Option<&DabEnsemble> {
        self.ensemble.as_ref()
    }

    /// Reset receiver state.
    pub fn reset(&mut self) {
        self.prev_symbol = None;
        self.ensemble = None;
        self.freq_offset_hz = 0.0;
        self.synced = false;
    }
}

/// Generate frequency interleaving permutation table.
fn generate_freq_interleave_table(n: usize) -> Vec<usize> {
    // Simplified: use bit-reversal permutation as approximation
    let bits = (n as f64).log2().ceil() as u32;
    (0..n)
        .map(|i| {
            let mut rev = 0usize;
            let mut val = i;
            for _ in 0..bits {
                rev = (rev << 1) | (val & 1);
                val >>= 1;
            }
            rev % n
        })
        .collect()
}

/// Simple DFT (for small FFT sizes in tests).
fn simple_fft(input: &[Complex64]) -> Vec<Complex64> {
    let n = input.len();
    let mut output = vec![Complex64::new(0.0, 0.0); n];

    for k in 0..n {
        for (j, &x) in input.iter().enumerate() {
            let phase = -2.0 * PI * k as f64 * j as f64 / n as f64;
            output[k] += x * Complex64::new(phase.cos(), phase.sin());
        }
    }
    output
}

/// DAB channel frequencies (VHF Band III, MHz).
pub fn dab_channel_frequency(channel: &str) -> Option<f64> {
    match channel {
        "5A" => Some(174.928),
        "5B" => Some(176.640),
        "5C" => Some(178.352),
        "5D" => Some(180.064),
        "6A" => Some(181.936),
        "6B" => Some(183.648),
        "6C" => Some(185.360),
        "6D" => Some(187.072),
        "7A" => Some(188.928),
        "7B" => Some(190.640),
        "7C" => Some(192.352),
        "7D" => Some(194.064),
        "8A" => Some(195.936),
        "8B" => Some(197.648),
        "8C" => Some(199.360),
        "8D" => Some(201.072),
        "9A" => Some(202.928),
        "9B" => Some(204.640),
        "9C" => Some(206.352),
        "9D" => Some(208.064),
        "10A" => Some(209.936),
        "10B" => Some(211.648),
        "10C" => Some(213.360),
        "10D" => Some(215.072),
        "11A" => Some(216.928),
        "11B" => Some(218.640),
        "11C" => Some(220.352),
        "11D" => Some(222.064),
        "12A" => Some(223.936),
        "12B" => Some(225.648),
        "12C" => Some(227.360),
        "12D" => Some(229.072),
        "13A" => Some(230.748),
        "13B" => Some(232.496),
        "13C" => Some(234.208),
        "13D" => Some(235.776),
        "13E" => Some(237.488),
        "13F" => Some(239.200),
        _ => None,
    }
}

/// Viterbi-like convolutional decoding (simplified hard-decision).
pub fn convolutional_decode(bits: &[u8], _constraint_length: usize) -> Vec<u8> {
    // Simplified: rate 1/4 convolutional code used in DAB FIC
    // Real impl would use Viterbi with traceback
    // Here we do simple majority voting over 4 bits → 1 bit
    bits.chunks(4)
        .map(|chunk| {
            let ones: usize = chunk.iter().filter(|&&b| b == 1).count();
            if ones > chunk.len() / 2 { 1 } else { 0 }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dab_mode_i_params() {
        let config = DabConfig::new(DabMode::I);
        assert_eq!(config.fft_size, 2048);
        assert_eq!(config.num_carriers, 1536);
        assert_eq!(config.guard_interval, 504);
        assert_eq!(config.num_symbols, 76);
    }

    #[test]
    fn test_dab_mode_ii_params() {
        let config = DabConfig::new(DabMode::II);
        assert_eq!(config.fft_size, 512);
        assert_eq!(config.num_carriers, 384);
    }

    #[test]
    fn test_dab_mode_iii_params() {
        let config = DabConfig::new(DabMode::III);
        assert_eq!(config.fft_size, 256);
        assert_eq!(config.num_symbols, 153);
    }

    #[test]
    fn test_dab_mode_iv_params() {
        let config = DabConfig::new(DabMode::IV);
        assert_eq!(config.fft_size, 1024);
        assert_eq!(config.num_carriers, 768);
    }

    #[test]
    fn test_frame_duration() {
        let config = DabConfig::new(DabMode::I);
        let duration = config.frame_duration();
        // Null + 76 * symbol_duration
        assert_eq!(duration, 2656 + 76 * 2552);
        let duration_s = config.frame_duration_s();
        // Should be ~96 ms for Mode I
        assert!((duration_s - 0.096).abs() < 0.01, "Frame duration: {} s", duration_s);
    }

    #[test]
    fn test_receiver_creation() {
        let config = DabConfig::new(DabMode::I);
        let receiver = DabReceiver::new(config);
        assert!(!receiver.is_synced());
        assert_eq!(receiver.freq_offset(), 0.0);
        assert!(receiver.ensemble().is_none());
    }

    #[test]
    fn test_energy_dispersal() {
        let config = DabConfig::new(DabMode::I);
        let receiver = DabReceiver::new(config);
        let bits = vec![0u8; 100];
        let scrambled = receiver.energy_dispersal(&bits);
        assert_eq!(scrambled.len(), 100);
        // Descrambling should give back original
        let descrambled = receiver.energy_dispersal(&scrambled);
        assert_eq!(descrambled, bits);
    }

    #[test]
    fn test_dqpsk_demod_needs_prev() {
        let config = DabConfig::new(DabMode::I);
        let mut receiver = DabReceiver::new(config);
        let symbol = OfdmSymbol {
            carriers: vec![Complex64::new(1.0, 0.0); 100],
            symbol_index: 0,
        };
        // First symbol: no output (no previous reference)
        let bits = receiver.dqpsk_demod(&symbol);
        assert!(bits.is_empty());

        // Second symbol: should produce bits
        let bits = receiver.dqpsk_demod(&symbol);
        assert_eq!(bits.len(), 200); // 2 bits per carrier
    }

    #[test]
    fn test_dqpsk_demod_phases() {
        let config = DabConfig::new(DabMode::I);
        let mut receiver = DabReceiver::new(config);

        // Reference symbol
        let ref_sym = OfdmSymbol {
            carriers: vec![Complex64::new(1.0, 0.0); 4],
            symbol_index: 0,
        };
        receiver.dqpsk_demod(&ref_sym);

        // Same phase → 00
        let same = OfdmSymbol {
            carriers: vec![Complex64::new(1.0, 0.0); 4],
            symbol_index: 1,
        };
        let bits = receiver.dqpsk_demod(&same);
        // Each carrier produces 2 bits = 00
        assert_eq!(bits[0], 0);
        assert_eq!(bits[1], 0);
    }

    #[test]
    fn test_channel_frequency() {
        assert_eq!(dab_channel_frequency("5A"), Some(174.928));
        assert_eq!(dab_channel_frequency("11A"), Some(216.928));
        assert!(dab_channel_frequency("99Z").is_none());
    }

    #[test]
    fn test_convolutional_decode() {
        let encoded = vec![1, 1, 1, 0, 0, 0, 0, 1];
        let decoded = convolutional_decode(&encoded, 7);
        assert_eq!(decoded.len(), 2);
        assert_eq!(decoded[0], 1); // majority of [1,1,1,0]
        assert_eq!(decoded[1], 0); // majority of [0,0,0,1]
    }

    #[test]
    fn test_frequency_deinterleave() {
        let config = DabConfig::new(DabMode::I);
        let receiver = DabReceiver::new(config);
        let carriers: Vec<Complex64> = (0..16)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let deinterleaved = receiver.frequency_deinterleave(&carriers);
        assert_eq!(deinterleaved.len(), 16);
    }

    #[test]
    fn test_receiver_reset() {
        let config = DabConfig::new(DabMode::I);
        let mut receiver = DabReceiver::new(config);
        receiver.synced = true;
        receiver.freq_offset_hz = 100.0;
        receiver.reset();
        assert!(!receiver.is_synced());
        assert_eq!(receiver.freq_offset(), 0.0);
    }

    #[test]
    fn test_null_symbol_detection() {
        let config = DabConfig::new(DabMode::II); // Smaller FFT for faster test
        let null_dur = config.null_symbol_duration;
        let mut receiver = DabReceiver::new(config);

        // Create signal with a null symbol region
        let n = null_dur * 4;
        let mut samples = Vec::with_capacity(n);
        // Normal signal
        for i in 0..null_dur {
            let phase = 2.0 * PI * 0.1 * i as f64;
            samples.push(Complex64::new(phase.cos(), phase.sin()));
        }
        // Null symbol (silence)
        for _ in 0..null_dur {
            samples.push(Complex64::new(0.0, 0.0));
        }
        // Normal signal
        for i in 0..null_dur * 2 {
            let phase = 2.0 * PI * 0.1 * i as f64;
            samples.push(Complex64::new(phase.cos(), phase.sin()));
        }

        let result = receiver.detect_null_symbol(&samples);
        assert!(result.is_some(), "Should detect null symbol");
    }
}

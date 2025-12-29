//! Frequency Hopping Spread Spectrum (FHSS)
//!
//! FHSS is a spread spectrum technique where the carrier frequency "hops"
//! across a wide bandwidth according to a pseudo-random pattern. This provides:
//!
//! - **Interference Avoidance**: Hops away from narrowband interference
//! - **Covert Communications**: Difficult to detect without knowing hop pattern
//! - **Multiple Access**: Different hop patterns allow multiple users
//! - **Resistance to Jamming**: Jammer must spread energy across all frequencies
//!
//! ## How FHSS Works
//!
//! ```text
//! Time →
//! ┌──────┬──────┬──────┬──────┬──────┬──────┐
//! │      │      │  ●   │      │      │      │ Freq 5
//! ├──────┼──────┼──────┼──────┼──────┼──────┤
//! │      │      │      │      │  ●   │      │ Freq 4
//! ├──────┼──────┼──────┼──────┼──────┼──────┤
//! │  ●   │      │      │      │      │      │ Freq 3
//! ├──────┼──────┼──────┼──────┼──────┼──────┤
//! │      │  ●   │      │      │      │  ●   │ Freq 2
//! ├──────┼──────┼──────┼──────┼──────┼──────┤
//! │      │      │      │  ●   │      │      │ Freq 1
//! └──────┴──────┴──────┴──────┴──────┴──────┘
//!   Hop 1  Hop 2  Hop 3  Hop 4  Hop 5  Hop 6
//! ```
//!
//! ## Processing Gain
//!
//! For FHSS, processing gain comes from the duty cycle at any single frequency:
//! PG = 10 * log10(total_bandwidth / hop_bandwidth)
//!
//! ## LPD/LPI Properties
//!
//! - Short dwell time means signal is only at each frequency briefly
//! - Energy spread across many frequencies
//! - Without hop sequence, receiver sees noise-like signal
//!
//! ## Slow vs Fast Hopping
//!
//! - **Slow FHSS**: Multiple symbols per hop (simpler synchronization)
//! - **Fast FHSS**: Multiple hops per symbol (better interference resistance)

use super::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};
use crate::spreading::lfsr::Lfsr;
use crate::types::IQSample;
use rustfft::{FftPlanner, num_complex::Complex64};
use std::f64::consts::PI;

/// Spectrogram data for FHSS visualization
#[derive(Debug, Clone)]
pub struct FhssSpectrogramData {
    /// Power values at each (time, frequency) bin - [time_idx][freq_idx]
    pub power_grid: Vec<Vec<f64>>,
    /// Hop markers showing (time_seconds, frequency_hz) of each hop
    pub hop_markers: Vec<(f64, f64)>,
    /// Frequency axis labels (Hz)
    pub freq_axis: Vec<f64>,
    /// Time axis labels (seconds)
    pub time_axis: Vec<f64>,
    /// Number of channels
    pub num_channels: usize,
    /// Channel spacing (Hz)
    pub channel_spacing: f64,
    /// Total bandwidth (Hz)
    pub total_bandwidth: f64,
}

/// Hop pattern generation method
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HopPattern {
    /// Pseudo-random using LFSR
    PseudoRandom,
    /// Sequential (for testing)
    Sequential,
}

/// Modulation used at each hop frequency
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HopModulation {
    /// Binary FSK at each hop
    Bfsk { deviation: f64 },
    /// BPSK at each hop
    Bpsk,
    /// QPSK at each hop
    Qpsk,
}

impl HopModulation {
    /// Bits per symbol for the hop modulation
    pub fn bits_per_symbol(&self) -> usize {
        match self {
            Self::Bfsk { .. } | Self::Bpsk => 1,
            Self::Qpsk => 2,
        }
    }
}

/// FHSS configuration
#[derive(Debug, Clone)]
pub struct FhssConfig {
    /// Number of frequency channels
    pub num_channels: usize,
    /// Channel spacing in Hz
    pub channel_spacing: f64,
    /// Hop rate (hops per second)
    pub hop_rate: f64,
    /// Symbols transmitted per hop (dwell time = symbols_per_hop / symbol_rate)
    pub symbols_per_hop: usize,
    /// Symbol rate at each hop frequency (symbols per second)
    pub symbol_rate: f64,
    /// Hop pattern type
    pub hop_pattern: HopPattern,
    /// Modulation at each hop frequency
    pub modulation: HopModulation,
    /// LFSR seed for pseudo-random pattern
    pub seed: u32,
}

impl Default for FhssConfig {
    fn default() -> Self {
        Self {
            num_channels: 50,
            channel_spacing: 25_000.0, // 25 kHz spacing
            hop_rate: 100.0,           // 100 hops/second
            symbols_per_hop: 10,       // 10 symbols per hop
            symbol_rate: 1000.0,       // 1000 symbols/second
            hop_pattern: HopPattern::PseudoRandom,
            modulation: HopModulation::Bfsk { deviation: 5000.0 },
            seed: 0x12345,
        }
    }
}

/// Frequency Hopping Spread Spectrum modulator/demodulator
#[derive(Debug, Clone)]
pub struct FHSS {
    /// Common waveform parameters
    common: CommonParams,
    /// FHSS configuration (public for anti-jam demo access)
    pub config: FhssConfig,
    /// Hop sequence generator
    hop_lfsr: Lfsr,
    /// Current hop index
    current_hop: usize,
    /// Pre-computed hop sequence (one period)
    hop_sequence: Vec<usize>,
}

impl FHSS {
    /// Create a new FHSS modulator
    pub fn new(common: CommonParams, config: FhssConfig) -> Self {
        // Initialize LFSR for hop pattern generation
        // Use degree that covers the number of channels
        let degree = (config.num_channels as f64).log2().ceil() as u8;
        let degree = degree.max(5).min(10);

        // Standard polynomial for the degree
        let polynomial = match degree {
            5 => 0x12,
            6 => 0x21,
            7 => 0x41,
            8 => 0x8E,
            9 => 0x108,
            10 => 0x204,
            _ => 0x12,
        };

        let hop_lfsr = Lfsr::new(degree, polynomial, config.seed);

        // Pre-compute hop sequence
        let mut lfsr_copy = hop_lfsr.clone();
        let period = (1 << degree) - 1;
        let hop_sequence: Vec<usize> = (0..period)
            .map(|_| {
                // Use multiple LFSR clocks to get more bits
                let mut val = 0u32;
                for _ in 0..degree {
                    val = (val << 1) | lfsr_copy.clock() as u32;
                }
                (val as usize) % config.num_channels
            })
            .collect();

        Self {
            common,
            config,
            hop_lfsr,
            current_hop: 0,
            hop_sequence,
        }
    }

    /// Create with default configuration
    pub fn default_config(sample_rate: f64) -> Self {
        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        Self::new(common, FhssConfig::default())
    }

    /// Create with military-style fast hopping
    pub fn fast_hop(sample_rate: f64, num_channels: usize, hop_rate: f64) -> Self {
        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let config = FhssConfig {
            num_channels,
            hop_rate,
            symbols_per_hop: 1, // Fast hopping: 1 symbol per hop
            ..Default::default()
        };
        Self::new(common, config)
    }

    /// Get the total bandwidth (all hop channels)
    pub fn total_bandwidth(&self) -> f64 {
        self.config.num_channels as f64 * self.config.channel_spacing
    }

    /// Get the instantaneous bandwidth (at one hop frequency)
    pub fn hop_bandwidth(&self) -> f64 {
        match self.config.modulation {
            HopModulation::Bfsk { deviation } => 2.0 * deviation + self.config.symbol_rate,
            HopModulation::Bpsk | HopModulation::Qpsk => self.config.symbol_rate,
        }
    }

    /// Get processing gain in dB
    pub fn processing_gain_db(&self) -> f64 {
        10.0 * (self.total_bandwidth() / self.hop_bandwidth()).log10()
    }

    /// Get the dwell time per hop in seconds
    pub fn dwell_time(&self) -> f64 {
        1.0 / self.config.hop_rate
    }

    /// Get samples per hop
    pub fn samples_per_hop(&self) -> usize {
        (self.common.sample_rate * self.dwell_time()) as usize
    }

    /// Get the data rate in bits/second
    pub fn data_rate(&self) -> f64 {
        self.config.symbol_rate * self.config.modulation.bits_per_symbol() as f64
    }

    /// Reset the hop sequence
    pub fn reset(&mut self) {
        self.current_hop = 0;
        self.hop_lfsr = Lfsr::new(
            (self.config.num_channels as f64).log2().ceil() as u8,
            0x41,
            self.config.seed,
        );
    }

    /// Get the next hop channel index
    fn next_hop_channel(&mut self) -> usize {
        match self.config.hop_pattern {
            HopPattern::Sequential => {
                let ch = self.current_hop % self.config.num_channels;
                self.current_hop += 1;
                ch
            }
            HopPattern::PseudoRandom => {
                let ch = self.hop_sequence[self.current_hop % self.hop_sequence.len()];
                self.current_hop += 1;
                ch
            }
        }
    }

    /// Get frequency offset for a channel (relative to center)
    fn channel_to_frequency(&self, channel: usize) -> f64 {
        // Center the channels around 0
        let center_channel = self.config.num_channels as f64 / 2.0;
        (channel as f64 - center_channel) * self.config.channel_spacing
    }

    /// Modulate data at a single hop frequency
    fn modulate_hop(&self, data: &[u8], freq_offset: f64) -> Vec<IQSample> {
        let samples_per_symbol =
            (self.common.sample_rate / self.config.symbol_rate) as usize;
        let mut samples = Vec::with_capacity(data.len() * samples_per_symbol);

        for (symbol_idx, bits) in data.chunks(self.config.modulation.bits_per_symbol()).enumerate()
        {
            match self.config.modulation {
                HopModulation::Bfsk { deviation } => {
                    let bit = bits.first().copied().unwrap_or(0) & 1;
                    let freq = freq_offset + if bit == 0 { deviation } else { -deviation };

                    for i in 0..samples_per_symbol {
                        let t = (symbol_idx * samples_per_symbol + i) as f64
                            / self.common.sample_rate;
                        let phase = 2.0 * PI * freq * t;
                        samples.push(IQSample::new(
                            phase.cos() * self.common.amplitude,
                            phase.sin() * self.common.amplitude,
                        ));
                    }
                }
                HopModulation::Bpsk => {
                    let bit = bits.first().copied().unwrap_or(0) & 1;
                    let symbol_phase = if bit == 0 { 0.0 } else { PI };

                    for i in 0..samples_per_symbol {
                        let t = (symbol_idx * samples_per_symbol + i) as f64
                            / self.common.sample_rate;
                        let phase = 2.0 * PI * freq_offset * t + symbol_phase;
                        samples.push(IQSample::new(
                            phase.cos() * self.common.amplitude,
                            phase.sin() * self.common.amplitude,
                        ));
                    }
                }
                HopModulation::Qpsk => {
                    let b0 = bits.first().copied().unwrap_or(0) & 1;
                    let b1 = bits.get(1).copied().unwrap_or(0) & 1;
                    let symbol_phase = match (b0, b1) {
                        (0, 0) => PI / 4.0,
                        (0, 1) => 3.0 * PI / 4.0,
                        (1, 1) => 5.0 * PI / 4.0,
                        (1, 0) => 7.0 * PI / 4.0,
                        _ => 0.0,
                    };

                    for i in 0..samples_per_symbol {
                        let t = (symbol_idx * samples_per_symbol + i) as f64
                            / self.common.sample_rate;
                        let phase = 2.0 * PI * freq_offset * t + symbol_phase;
                        samples.push(IQSample::new(
                            phase.cos() * self.common.amplitude,
                            phase.sin() * self.common.amplitude,
                        ));
                    }
                }
            }
        }

        samples
    }

    /// Get the hop sequence for visualization
    pub fn get_hop_sequence(&self, num_hops: usize) -> Vec<usize> {
        let mut fhss_copy = self.clone();
        fhss_copy.reset();
        (0..num_hops).map(|_| fhss_copy.next_hop_channel()).collect()
    }

    /// Frequency shift samples to baseband (mix down by freq_offset)
    fn frequency_shift_to_baseband(&self, samples: &[IQSample], freq_offset: f64) -> Vec<IQSample> {
        let omega = -2.0 * PI * freq_offset / self.common.sample_rate;
        samples
            .iter()
            .enumerate()
            .map(|(i, &s)| {
                let phase = omega * i as f64;
                let rotation = IQSample::new(phase.cos(), phase.sin());
                IQSample::new(
                    s.re * rotation.re - s.im * rotation.im,
                    s.re * rotation.im + s.im * rotation.re,
                )
            })
            .collect()
    }

    /// Demodulate BFSK symbols from baseband samples
    fn demodulate_bfsk(&self, samples: &[IQSample], deviation: f64) -> Vec<u8> {
        let samples_per_symbol =
            (self.common.sample_rate / self.config.symbol_rate) as usize;
        if samples_per_symbol == 0 {
            return Vec::new();
        }

        let mut bits = Vec::new();
        let omega_plus = 2.0 * PI * deviation / self.common.sample_rate;
        let omega_minus = 2.0 * PI * (-deviation) / self.common.sample_rate;

        for chunk in samples.chunks(samples_per_symbol) {
            // Correlate with +deviation and -deviation tones
            let mut energy_plus = 0.0;
            let mut energy_minus = 0.0;

            for (i, &s) in chunk.iter().enumerate() {
                // +deviation reference
                let ref_plus = IQSample::new(
                    (omega_plus * i as f64).cos(),
                    (omega_plus * i as f64).sin(),
                );
                let corr_plus = s.re * ref_plus.re + s.im * ref_plus.im;
                energy_plus += corr_plus * corr_plus;

                // -deviation reference
                let ref_minus = IQSample::new(
                    (omega_minus * i as f64).cos(),
                    (omega_minus * i as f64).sin(),
                );
                let corr_minus = s.re * ref_minus.re + s.im * ref_minus.im;
                energy_minus += corr_minus * corr_minus;
            }

            // Bit 0 = +deviation (mark), Bit 1 = -deviation (space)
            bits.push(if energy_plus > energy_minus { 0 } else { 1 });
        }

        bits
    }

    /// Demodulate BPSK symbols from baseband samples
    fn demodulate_bpsk(&self, samples: &[IQSample]) -> Vec<u8> {
        let samples_per_symbol =
            (self.common.sample_rate / self.config.symbol_rate) as usize;
        if samples_per_symbol == 0 {
            return Vec::new();
        }

        let mut bits = Vec::new();

        for chunk in samples.chunks(samples_per_symbol) {
            // Sum the real part (coherent detection)
            let sum_re: f64 = chunk.iter().map(|s| s.re).sum();
            // Positive = 0, Negative = 1
            bits.push(if sum_re >= 0.0 { 0 } else { 1 });
        }

        bits
    }

    /// Demodulate QPSK symbols from baseband samples
    fn demodulate_qpsk(&self, samples: &[IQSample]) -> Vec<u8> {
        let samples_per_symbol =
            (self.common.sample_rate / self.config.symbol_rate) as usize;
        if samples_per_symbol == 0 {
            return Vec::new();
        }

        let mut bits = Vec::new();

        for chunk in samples.chunks(samples_per_symbol) {
            // Average I and Q values
            let avg_i: f64 = chunk.iter().map(|s| s.re).sum::<f64>() / chunk.len() as f64;
            let avg_q: f64 = chunk.iter().map(|s| s.im).sum::<f64>() / chunk.len() as f64;

            // Quadrant detection (Gray coded)
            // Q1 (I>0, Q>0): 00, Q2 (I<0, Q>0): 01, Q3 (I<0, Q<0): 11, Q4 (I>0, Q<0): 10
            let (b0, b1) = match (avg_i >= 0.0, avg_q >= 0.0) {
                (true, true) => (0, 0),   // 45°
                (false, true) => (0, 1),  // 135°
                (false, false) => (1, 1), // 225°
                (true, false) => (1, 0),  // 315°
            };
            bits.push(b0);
            bits.push(b1);
        }

        bits
    }

    /// Demodulate samples at a single hop frequency
    fn demodulate_hop(&self, samples: &[IQSample], freq_offset: f64) -> Vec<u8> {
        // Mix down to baseband
        let baseband = self.frequency_shift_to_baseband(samples, freq_offset);

        // Demodulate based on hop modulation type
        match self.config.modulation {
            HopModulation::Bfsk { deviation } => self.demodulate_bfsk(&baseband, deviation),
            HopModulation::Bpsk => self.demodulate_bpsk(&baseband),
            HopModulation::Qpsk => self.demodulate_qpsk(&baseband),
        }
    }

    /// Generate spectrogram data for visualization
    ///
    /// Creates a time-frequency power display showing the frequency hopping pattern.
    pub fn generate_spectrogram(&self, samples: &[IQSample], fft_size: usize) -> FhssSpectrogramData {
        let fft_size = fft_size.max(64).min(4096);
        let hop_size = fft_size / 2; // 50% overlap

        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(fft_size);

        // Generate Hann window
        let window: Vec<f64> = (0..fft_size)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / fft_size as f64).cos()))
            .collect();

        let mut power_grid = Vec::new();
        let mut time_axis = Vec::new();

        // Sliding window FFT
        let mut offset = 0;
        while offset + fft_size <= samples.len() {
            // Apply window and convert to Complex64
            let mut buffer: Vec<Complex64> = samples[offset..offset + fft_size]
                .iter()
                .zip(window.iter())
                .map(|(s, &w)| Complex64::new(s.re * w, s.im * w))
                .collect();

            // Perform FFT
            fft.process(&mut buffer);

            // Compute power spectrum (shift to center DC)
            let mut power_spectrum: Vec<f64> = vec![0.0; fft_size];
            for i in 0..fft_size {
                let shifted_idx = (i + fft_size / 2) % fft_size;
                let mag = buffer[i].norm();
                power_spectrum[shifted_idx] = 20.0 * (mag.max(1e-10)).log10(); // dB
            }

            power_grid.push(power_spectrum);
            time_axis.push(offset as f64 / self.common.sample_rate);

            offset += hop_size;
        }

        // Generate frequency axis (centered around 0)
        let freq_resolution = self.common.sample_rate / fft_size as f64;
        let freq_axis: Vec<f64> = (0..fft_size)
            .map(|i| (i as f64 - fft_size as f64 / 2.0) * freq_resolution)
            .collect();

        // Generate hop markers
        let mut hop_markers = Vec::new();
        let samples_per_hop = self.samples_per_hop();
        if samples_per_hop > 0 {
            let mut fhss = self.clone();
            fhss.reset();

            let num_hops = samples.len() / samples_per_hop;
            for hop_idx in 0..num_hops {
                let channel = fhss.next_hop_channel();
                let freq = fhss.channel_to_frequency(channel);
                let time = (hop_idx * samples_per_hop) as f64 / self.common.sample_rate;
                hop_markers.push((time, freq));
            }
        }

        FhssSpectrogramData {
            power_grid,
            hop_markers,
            freq_axis,
            time_axis,
            num_channels: self.config.num_channels,
            channel_spacing: self.config.channel_spacing,
            total_bandwidth: self.total_bandwidth(),
        }
    }
}

impl Waveform for FHSS {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "FHSS",
            full_name: "Frequency Hopping Spread Spectrum",
            description: "Carrier frequency hops pseudo-randomly for LPD/LPI",
            complexity: 4,
            bits_per_symbol: self.config.modulation.bits_per_symbol() as u8,
            carries_data: true,
            characteristics: &[
                "Frequency agility (hops across bandwidth)",
                "Interference avoidance",
                "Low Probability of Detection (LPD)",
                "Low Probability of Intercept (LPI)",
                "Multiple access via different hop patterns",
                "Jamming resistance",
            ],
            history: "FHSS was patented in 1942 by Hedy Lamarr and George Antheil for \
                torpedo guidance using piano roll technology. Declassified and commercialized \
                in the 1980s. Used in early Bluetooth (before AFH), military radios like \
                SINCGARS, and combined with DSSS in some systems (hybrid spread spectrum).",
            modern_usage: "Modern Bluetooth uses Adaptive Frequency Hopping (AFH). Military \
                tactical radios (SINCGARS, HAVEQUICK) use FHSS. Some ISM band devices use FHSS \
                for interference avoidance. Often combined with other techniques for enhanced \
                security and reliability.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        let mut samples = Vec::new();
        let bits_per_hop =
            self.config.symbols_per_hop * self.config.modulation.bits_per_symbol();

        // Clone self to track hop state
        let mut fhss = self.clone();
        fhss.reset();

        // Process data in hop-sized chunks
        for hop_data in data.chunks(bits_per_hop) {
            let channel = fhss.next_hop_channel();
            let freq_offset = fhss.channel_to_frequency(channel);

            let hop_samples = fhss.modulate_hop(hop_data, freq_offset);
            samples.extend(hop_samples);

            // Pad to full hop duration if needed
            let expected_samples = fhss.samples_per_hop();
            while samples.len() % expected_samples != 0 && hop_data.len() < bits_per_hop {
                samples.push(IQSample::new(0.0, 0.0));
            }
        }

        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let mut result = DemodResult::default();

        // Add metadata
        result
            .metadata
            .insert("processing_gain_db".to_string(), self.processing_gain_db());
        result
            .metadata
            .insert("total_bandwidth".to_string(), self.total_bandwidth());
        result.metadata.insert("hop_rate".to_string(), self.config.hop_rate);

        let samples_per_hop = self.samples_per_hop();
        if samples_per_hop == 0 || samples.is_empty() {
            return result;
        }

        // Clone self to track hop sequence (synchronized with transmitter)
        let mut fhss = self.clone();
        fhss.reset();

        let mut all_bits = Vec::new();
        let num_hops = samples.len() / samples_per_hop;

        // Process each hop period
        for hop_idx in 0..num_hops {
            let start = hop_idx * samples_per_hop;
            let end = start + samples_per_hop;
            if end > samples.len() {
                break;
            }

            let hop_samples = &samples[start..end];
            let channel = fhss.next_hop_channel();
            let freq_offset = fhss.channel_to_frequency(channel);

            // Demodulate this hop
            let hop_bits = fhss.demodulate_hop(hop_samples, freq_offset);
            all_bits.extend(hop_bits);
        }

        result.bits = all_bits;
        result.metadata.insert("hops_processed".to_string(), num_hops as f64);

        result
    }

    fn samples_per_symbol(&self) -> usize {
        if self.config.symbol_rate <= 0.0 {
            return 1;
        }
        ((self.common.sample_rate / self.config.symbol_rate) as usize).max(1)
    }

    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);

        // No constellation for FHSS (it's frequency-based)
        VisualizationData {
            samples,
            constellation: Vec::new(),
            constellation_labels: Vec::new(),
            spectrum: Vec::new(),
            description: format!(
                "FHSS: {} channels, {:.0} hops/s, {:.1} dB gain, {:?}",
                self.config.num_channels,
                self.config.hop_rate,
                self.processing_gain_db(),
                self.config.modulation
            ),
        }
    }

    fn generate_demo(&self, duration_ms: f64) -> Vec<IQSample> {
        let num_samples = (self.common.sample_rate * duration_ms / 1000.0) as usize;
        let demo_data: Vec<u8> = (0..256).map(|i| (i % 2) as u8).collect();
        let mut samples = self.modulate(&demo_data);
        samples.truncate(num_samples);
        samples
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fhss_basic() {
        let fhss = FHSS::default_config(1_000_000.0);

        assert_eq!(fhss.config.num_channels, 50);
        assert!(fhss.processing_gain_db() > 10.0);
    }

    #[test]
    fn test_fhss_bandwidth() {
        let fhss = FHSS::default_config(1_000_000.0);

        let total_bw = fhss.total_bandwidth();
        let hop_bw = fhss.hop_bandwidth();

        assert!(total_bw > hop_bw * 10.0); // Significant spreading
    }

    #[test]
    fn test_hop_sequence() {
        let fhss = FHSS::default_config(1_000_000.0);
        let hops = fhss.get_hop_sequence(20);

        // All hops should be within channel range
        assert!(hops.iter().all(|&h| h < fhss.config.num_channels));

        // Should have some variation (not all the same)
        let unique: std::collections::HashSet<_> = hops.iter().collect();
        assert!(unique.len() > 1);
    }

    #[test]
    fn test_fhss_modulation() {
        let fhss = FHSS::default_config(1_000_000.0);

        let data: Vec<u8> = vec![1, 0, 1, 1, 0, 0, 1, 0, 1, 0];
        let samples = fhss.modulate(&data);

        assert!(!samples.is_empty());

        // All samples should have reasonable amplitude
        assert!(samples
            .iter()
            .all(|s| (s.re * s.re + s.im * s.im).sqrt() <= 1.5));
    }

    #[test]
    fn test_fast_hop() {
        let fhss = FHSS::fast_hop(1_000_000.0, 100, 1000.0);

        assert_eq!(fhss.config.num_channels, 100);
        assert_eq!(fhss.config.hop_rate, 1000.0);
        assert_eq!(fhss.config.symbols_per_hop, 1);
    }

    #[test]
    fn test_sequential_hopping() {
        let common = CommonParams {
            sample_rate: 1_000_000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let config = FhssConfig {
            num_channels: 10,
            hop_pattern: HopPattern::Sequential,
            ..Default::default()
        };
        let fhss = FHSS::new(common, config);

        let hops = fhss.get_hop_sequence(20);

        // Sequential should go 0,1,2,3,...,9,0,1,2,...
        for (i, &h) in hops.iter().enumerate() {
            assert_eq!(h, i % 10);
        }
    }

    #[test]
    fn test_demodulate_roundtrip_bfsk() {
        let common = CommonParams {
            sample_rate: 100_000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let config = FhssConfig {
            num_channels: 10,
            channel_spacing: 10_000.0,
            hop_rate: 100.0,
            symbols_per_hop: 10,
            symbol_rate: 1000.0,
            hop_pattern: HopPattern::Sequential,
            modulation: HopModulation::Bfsk { deviation: 2000.0 },
            seed: 0x12345,
        };
        let fhss = FHSS::new(common, config);

        // Test data: 100 bits (10 hops x 10 symbols x 1 bit)
        let tx_bits: Vec<u8> = (0..100).map(|i| (i % 2) as u8).collect();

        let samples = fhss.modulate(&tx_bits);
        let result = fhss.demodulate(&samples);

        // Check that we recovered bits
        assert!(!result.bits.is_empty(), "Should recover some bits");

        // Count bit errors (allowing some tolerance)
        let min_len = tx_bits.len().min(result.bits.len());
        let errors: usize = tx_bits[..min_len]
            .iter()
            .zip(result.bits[..min_len].iter())
            .filter(|(&a, &b)| a != b)
            .count();

        let ber = errors as f64 / min_len as f64;
        assert!(ber < 0.1, "BER should be < 10% without noise, got {:.1}%", ber * 100.0);
    }

    #[test]
    fn test_demodulate_roundtrip_bpsk() {
        let common = CommonParams {
            sample_rate: 100_000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let config = FhssConfig {
            num_channels: 5,
            channel_spacing: 10_000.0,
            hop_rate: 50.0,
            symbols_per_hop: 10,
            symbol_rate: 500.0,
            hop_pattern: HopPattern::Sequential,
            modulation: HopModulation::Bpsk,
            seed: 0x12345,
        };
        let fhss = FHSS::new(common, config);

        let tx_bits: Vec<u8> = (0..50).map(|i| (i % 2) as u8).collect();

        let samples = fhss.modulate(&tx_bits);
        let result = fhss.demodulate(&samples);

        assert!(!result.bits.is_empty(), "Should recover some bits");
    }

    #[test]
    fn test_demodulate_metadata() {
        let fhss = FHSS::default_config(1_000_000.0);
        let tx_bits: Vec<u8> = vec![1, 0, 1, 1, 0, 0, 1, 0, 1, 0];

        let samples = fhss.modulate(&tx_bits);
        let result = fhss.demodulate(&samples);

        // Check metadata is present
        assert!(result.metadata.contains_key("processing_gain_db"));
        assert!(result.metadata.contains_key("total_bandwidth"));
        assert!(result.metadata.contains_key("hop_rate"));
    }
}

//! FHSS Anti-Jamming Demonstration Module
//!
//! This module demonstrates how Frequency Hopping Spread Spectrum (FHSS)
//! provides resistance against various types of jamming attacks.
//!
//! ## Jammer Types
//!
//! 1. **Narrowband Jammer**: Transmits on a single frequency, hoping to
//!    interfere with the signal. FHSS defeats this because the signal
//!    only occupies each frequency briefly.
//!
//! 2. **Sweep Jammer**: Sweeps across the frequency band. Can be partially
//!    effective against slow-hopping systems.
//!
//! 3. **Follower Jammer**: Tries to detect the current hop frequency and
//!    jam it. Effectiveness depends on reaction time.
//!
//! ## Processing Gain
//!
//! The key metric is processing gain:
//! ```text
//! PG_db = 10 * log10(total_bandwidth / hop_bandwidth)
//! ```
//!
//! With 50 channels, the jammer must spread its power across ~17 dB more
//! bandwidth to jam all channels simultaneously.

use super::fhss::FHSS;
use super::Waveform;
use crate::types::IQSample;
use std::f64::consts::PI;

/// Simple LFSR-based PRNG for noise generation
/// This avoids depending on rand crate for basic functionality
struct SimplePrng {
    state: u64,
}

impl SimplePrng {
    fn new(seed: u64) -> Self {
        Self { state: if seed == 0 { 0xDEADBEEF } else { seed } }
    }

    fn next_u64(&mut self) -> u64 {
        // xorshift64
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() as f64) / (u64::MAX as f64)
    }

    /// Generate Gaussian noise using Box-Muller transform
    fn next_gaussian(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-10);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

/// Types of jamming attacks
#[derive(Debug, Clone)]
pub enum JammerType {
    /// Narrowband jammer on a single frequency
    Narrowband {
        /// Center frequency of the jammer (Hz, relative to center)
        freq_hz: f64,
        /// Jammer power relative to signal (linear)
        power: f64,
    },
    /// Sweep jammer that moves across the band
    Sweep {
        /// Start frequency (Hz)
        start_freq: f64,
        /// End frequency (Hz)
        end_freq: f64,
        /// Sweep rate (Hz/second)
        sweep_rate: f64,
        /// Jammer power relative to signal (linear)
        power: f64,
    },
    /// Follower jammer that tries to track hops
    Follower {
        /// Reaction time in seconds (how fast the jammer can track)
        reaction_time_sec: f64,
        /// Jammer power relative to signal (linear)
        power: f64,
        /// Jammer bandwidth (Hz)
        bandwidth: f64,
    },
    /// Barrage jammer that spreads power across entire bandwidth
    Barrage {
        /// Total jammer power (distributed across bandwidth)
        power: f64,
    },
}

impl Default for JammerType {
    fn default() -> Self {
        Self::Narrowband {
            freq_hz: 0.0,
            power: 10.0,
        }
    }
}

/// Results of an anti-jam demonstration
#[derive(Debug, Clone)]
pub struct AntiJamResult {
    /// Bit Error Rate without jamming
    pub clean_ber: f64,
    /// Bit Error Rate with jamming
    pub jammed_ber: f64,
    /// Effective processing gain demonstrated (dB)
    pub effective_gain_db: f64,
    /// Theoretical processing gain (dB)
    pub theoretical_gain_db: f64,
    /// Number of hops affected by the jammer
    pub hops_affected: usize,
    /// Total number of hops processed
    pub total_hops: usize,
    /// Percentage of hops affected
    pub percent_affected: f64,
    /// Signal-to-Jammer ratio at the receiver (dB)
    pub sjr_db: f64,
}

/// Anti-jamming demonstration system
#[derive(Debug, Clone)]
pub struct AntiJamDemo {
    /// FHSS system under test
    pub fhss: FHSS,
    /// Jammer configuration
    pub jammer: JammerType,
    /// SNR for AWGN noise (dB)
    pub snr_db: f64,
    /// Random seed for reproducibility
    pub seed: u64,
}

impl AntiJamDemo {
    /// Create a new anti-jam demo with default FHSS and jammer
    pub fn new(sample_rate: f64) -> Self {
        Self {
            fhss: FHSS::default_config(sample_rate),
            jammer: JammerType::default(),
            snr_db: 20.0,
            seed: 42,
        }
    }

    /// Create with custom FHSS configuration
    pub fn with_fhss(fhss: FHSS) -> Self {
        Self {
            fhss,
            jammer: JammerType::default(),
            snr_db: 20.0,
            seed: 42,
        }
    }

    /// Set the jammer type
    pub fn set_jammer(&mut self, jammer: JammerType) {
        self.jammer = jammer;
    }

    /// Set the SNR for background noise
    pub fn set_snr(&mut self, snr_db: f64) {
        self.snr_db = snr_db;
    }

    /// Run the anti-jam demonstration
    pub fn run(&self, tx_bits: &[u8]) -> AntiJamResult {
        let mut rng = SimplePrng::new(self.seed);

        // Modulate the signal
        let clean_samples = self.fhss.modulate(tx_bits);

        // Add AWGN noise
        let noisy_samples = self.add_awgn(&clean_samples, self.snr_db, &mut rng);

        // Demodulate without jamming (baseline)
        let clean_result = self.fhss.demodulate(&noisy_samples);
        let clean_ber = self.calculate_ber(tx_bits, &clean_result.bits);

        // Add jamming
        let jammed_samples = self.apply_jamming(&noisy_samples, &mut rng);

        // Demodulate with jamming
        let jammed_result = self.fhss.demodulate(&jammed_samples);
        let jammed_ber = self.calculate_ber(tx_bits, &jammed_result.bits);

        // Calculate metrics
        let theoretical_gain_db = self.fhss.processing_gain_db();
        let hops_affected = self.count_affected_hops(&clean_samples);
        let total_hops = clean_samples.len() / self.fhss.samples_per_hop();

        // Estimate effective gain from BER improvement
        let effective_gain_db = if jammed_ber > 0.0 && clean_ber < jammed_ber {
            10.0 * ((1.0 - clean_ber) / (1.0 - jammed_ber)).log10()
        } else {
            theoretical_gain_db
        };

        // Calculate SJR
        let sjr_db = self.calculate_sjr();

        AntiJamResult {
            clean_ber,
            jammed_ber,
            effective_gain_db,
            theoretical_gain_db,
            hops_affected,
            total_hops,
            percent_affected: if total_hops > 0 {
                100.0 * hops_affected as f64 / total_hops as f64
            } else {
                0.0
            },
            sjr_db,
        }
    }

    /// Add AWGN noise to samples
    fn add_awgn(&self, samples: &[IQSample], snr_db: f64, rng: &mut SimplePrng) -> Vec<IQSample> {
        if samples.is_empty() {
            return Vec::new();
        }

        // Calculate signal power
        let signal_power: f64 = samples.iter()
            .map(|s| s.re * s.re + s.im * s.im)
            .sum::<f64>() / samples.len() as f64;

        // Calculate noise power from SNR
        let snr_linear = 10.0_f64.powf(snr_db / 10.0);
        let noise_power = signal_power / snr_linear;
        let noise_std = (noise_power / 2.0).sqrt();

        samples.iter().map(|s| {
            let noise_i = rng.next_gaussian() * noise_std;
            let noise_q = rng.next_gaussian() * noise_std;
            IQSample::new(s.re + noise_i, s.im + noise_q)
        }).collect()
    }

    /// Apply jamming to samples
    fn apply_jamming(&self, samples: &[IQSample], rng: &mut SimplePrng) -> Vec<IQSample> {
        let sample_rate = self.fhss.common_params().sample_rate;

        match &self.jammer {
            JammerType::Narrowband { freq_hz, power } => {
                let omega = 2.0 * PI * freq_hz / sample_rate;
                let amplitude = power.sqrt();

                samples.iter().enumerate().map(|(i, s)| {
                    let jam_phase = omega * i as f64;
                    let jam = IQSample::new(
                        amplitude * jam_phase.cos(),
                        amplitude * jam_phase.sin(),
                    );
                    IQSample::new(s.re + jam.re, s.im + jam.im)
                }).collect()
            }

            JammerType::Sweep { start_freq, end_freq, sweep_rate, power } => {
                let amplitude = power.sqrt();
                let _total_time = samples.len() as f64 / sample_rate;
                let freq_range = end_freq - start_freq;

                samples.iter().enumerate().map(|(i, s)| {
                    let t = i as f64 / sample_rate;
                    // Sawtooth sweep
                    let sweep_pos = (t * sweep_rate / freq_range).fract();
                    let freq = start_freq + sweep_pos * freq_range;
                    let jam_phase = 2.0 * PI * freq * t;
                    let jam = IQSample::new(
                        amplitude * jam_phase.cos(),
                        amplitude * jam_phase.sin(),
                    );
                    IQSample::new(s.re + jam.re, s.im + jam.im)
                }).collect()
            }

            JammerType::Follower { reaction_time_sec, power, bandwidth: _ } => {
                let amplitude = power.sqrt();
                let samples_per_hop = self.fhss.samples_per_hop();
                let reaction_samples = (reaction_time_sec * sample_rate) as usize;

                // Get hop sequence
                let mut fhss = self.fhss.clone();
                fhss.reset();
                let num_hops = samples.len() / samples_per_hop;
                let hop_freqs: Vec<f64> = (0..num_hops)
                    .map(|_| {
                        let ch = fhss.get_hop_sequence(1)[0];
                        self.channel_to_frequency(ch)
                    })
                    .collect();

                samples.iter().enumerate().map(|(i, s)| {
                    // Determine which hop we're in
                    let hop_idx = i / samples_per_hop;
                    let sample_in_hop = i % samples_per_hop;

                    // Jammer can only track after reaction time
                    if sample_in_hop >= reaction_samples && hop_idx < hop_freqs.len() {
                        let freq = hop_freqs[hop_idx];
                        let t = i as f64 / sample_rate;
                        let jam_phase = 2.0 * PI * freq * t;
                        let jam = IQSample::new(
                            amplitude * jam_phase.cos(),
                            amplitude * jam_phase.sin(),
                        );
                        IQSample::new(s.re + jam.re, s.im + jam.im)
                    } else {
                        *s
                    }
                }).collect()
            }

            JammerType::Barrage { power } => {
                // Spread power across entire bandwidth as noise
                let total_bw = self.fhss.total_bandwidth();
                let hop_bw = self.fhss.hop_bandwidth();
                let power_per_hz = power / total_bw;
                let noise_power_in_hop = power_per_hz * hop_bw;
                let noise_std = (noise_power_in_hop / 2.0).sqrt();

                samples.iter().map(|s| {
                    let jam_i = rng.next_gaussian() * noise_std;
                    let jam_q = rng.next_gaussian() * noise_std;
                    IQSample::new(s.re + jam_i, s.im + jam_q)
                }).collect()
            }
        }
    }

    /// Count how many hops are affected by the jammer
    fn count_affected_hops(&self, samples: &[IQSample]) -> usize {
        let samples_per_hop = self.fhss.samples_per_hop();
        let num_hops = samples.len() / samples_per_hop;

        let mut fhss = self.fhss.clone();
        fhss.reset();

        let mut affected = 0;
        for _ in 0..num_hops {
            let channel = fhss.get_hop_sequence(1)[0];
            let hop_freq = self.channel_to_frequency(channel);

            if self.jammer_affects_frequency(hop_freq) {
                affected += 1;
            }
        }

        affected
    }

    /// Check if the jammer affects a particular frequency
    fn jammer_affects_frequency(&self, freq: f64) -> bool {
        match &self.jammer {
            JammerType::Narrowband { freq_hz, .. } => {
                // Affects if within hop bandwidth
                (freq - freq_hz).abs() < self.fhss.hop_bandwidth() / 2.0
            }
            JammerType::Sweep { start_freq, end_freq, .. } => {
                // Affects if within sweep range
                freq >= *start_freq && freq <= *end_freq
            }
            JammerType::Follower { .. } => {
                // Follower affects all hops (with delay)
                true
            }
            JammerType::Barrage { .. } => {
                // Barrage affects all frequencies
                true
            }
        }
    }

    /// Calculate Signal-to-Jammer ratio (dB)
    fn calculate_sjr(&self) -> f64 {
        let jammer_power = match &self.jammer {
            JammerType::Narrowband { power, .. } => *power,
            JammerType::Sweep { power, .. } => *power,
            JammerType::Follower { power, .. } => *power,
            JammerType::Barrage { power } => {
                // Effective power in hop bandwidth
                let ratio = self.fhss.hop_bandwidth() / self.fhss.total_bandwidth();
                power * ratio
            }
        };

        // Assume unit signal power
        let signal_power = 1.0;
        10.0 * (signal_power / jammer_power).log10()
    }

    /// Helper to convert channel to frequency
    fn channel_to_frequency(&self, channel: usize) -> f64 {
        let center_channel = self.fhss.config.num_channels as f64 / 2.0;
        (channel as f64 - center_channel) * self.fhss.config.channel_spacing
    }

    /// Calculate Bit Error Rate
    fn calculate_ber(&self, tx_bits: &[u8], rx_bits: &[u8]) -> f64 {
        if tx_bits.is_empty() || rx_bits.is_empty() {
            return 0.0;
        }

        let min_len = tx_bits.len().min(rx_bits.len());
        let errors: usize = tx_bits[..min_len]
            .iter()
            .zip(rx_bits[..min_len].iter())
            .filter(|(&a, &b)| a != b)
            .count();

        errors as f64 / min_len as f64
    }
}

/// Quick helper to create a narrowband jammer at a specific channel
pub fn narrowband_jammer_at_channel(
    fhss: &FHSS,
    channel: usize,
    power: f64,
) -> JammerType {
    let center_channel = fhss.config.num_channels as f64 / 2.0;
    let freq_hz = (channel as f64 - center_channel) * fhss.config.channel_spacing;
    JammerType::Narrowband { freq_hz, power }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_antijam_demo_creation() {
        let demo = AntiJamDemo::new(1_000_000.0);
        assert!(demo.fhss.processing_gain_db() > 10.0);
    }

    #[test]
    fn test_narrowband_jammer() {
        let demo = AntiJamDemo::new(1_000_000.0);

        // Generate test data
        let tx_bits: Vec<u8> = (0..100).map(|i| (i % 2) as u8).collect();
        let result = demo.run(&tx_bits);

        // Narrowband jammer should affect only a small percentage of hops
        assert!(result.percent_affected < 20.0,
            "Narrowband jammer should affect <20% of hops, got {:.1}%",
            result.percent_affected);
    }

    #[test]
    fn test_barrage_jammer() {
        let mut demo = AntiJamDemo::new(1_000_000.0);
        demo.set_jammer(JammerType::Barrage { power: 10.0 });

        let tx_bits: Vec<u8> = (0..50).map(|i| (i % 2) as u8).collect();
        let result = demo.run(&tx_bits);

        // Barrage affects all hops but power is spread thin
        assert_eq!(result.percent_affected, 100.0);
    }

    #[test]
    fn test_processing_gain() {
        let demo = AntiJamDemo::new(1_000_000.0);
        let result = demo.run(&[1, 0, 1, 0, 1, 0, 1, 0]);

        // Should have significant theoretical gain
        assert!(result.theoretical_gain_db > 15.0,
            "Expected >15 dB gain, got {:.1} dB",
            result.theoretical_gain_db);
    }

    #[test]
    fn test_jammer_at_channel() {
        let fhss = FHSS::default_config(1_000_000.0);
        let jammer = narrowband_jammer_at_channel(&fhss, 25, 10.0);

        match jammer {
            JammerType::Narrowband { freq_hz, power } => {
                assert!(freq_hz.abs() < 1.0); // Channel 25 is near center
                assert!((power - 10.0).abs() < 0.1);
            }
            _ => panic!("Expected narrowband jammer"),
        }
    }
}

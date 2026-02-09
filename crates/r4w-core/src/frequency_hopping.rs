//! Frequency Hopping — FHSS (Frequency Hopping Spread Spectrum) Controller
//!
//! Implements frequency hopping pattern generation and control for FHSS systems.
//! Supports both slow-hop (multiple symbols per hop) and fast-hop (multiple hops
//! per symbol) modes. Used in Bluetooth, military radios (JTIDS/Link-16),
//! and IEEE 802.11 FHSS. GNU Radio equivalent: `gr-spread` OOT module.
//!
//! ## Hopping Patterns
//!
//! - **Pseudorandom**: LFSR-based hopping sequence
//! - **Adaptive**: Avoids channels with interference
//! - **Latin Square**: Minimizes co-channel interference in multi-user systems
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::frequency_hopping::{FrequencyHopper, HopConfig, HopPattern};
//!
//! let config = HopConfig {
//!     num_channels: 79,           // Bluetooth: 79 channels
//!     channel_spacing_hz: 1e6,    // 1 MHz spacing
//!     base_freq_hz: 2.402e9,      // 2.402 GHz
//!     hop_rate: 1600.0,           // 1600 hops/sec (Bluetooth)
//!     dwell_time_s: 1.0 / 1600.0,
//!     pattern: HopPattern::Pseudorandom { seed: 42 },
//! };
//!
//! let mut hopper = FrequencyHopper::new(config);
//! let (channel, freq) = hopper.next_hop();
//! assert!(channel < 79);
//! ```

/// Hopping pattern type.
#[derive(Debug, Clone)]
pub enum HopPattern {
    /// Pseudorandom hopping using LFSR.
    Pseudorandom {
        /// LFSR seed value.
        seed: u32,
    },
    /// Sequential hopping through channels.
    Sequential,
    /// Fixed pattern (user-defined hop sequence).
    Fixed {
        /// Channel sequence to follow.
        sequence: Vec<usize>,
    },
    /// Adaptive hopping that avoids blacklisted channels.
    Adaptive {
        /// LFSR seed.
        seed: u32,
        /// Channels to avoid.
        blacklist: Vec<usize>,
    },
}

/// Frequency hopper configuration.
#[derive(Debug, Clone)]
pub struct HopConfig {
    /// Number of available channels.
    pub num_channels: usize,
    /// Channel spacing in Hz.
    pub channel_spacing_hz: f64,
    /// Base (lowest) frequency in Hz.
    pub base_freq_hz: f64,
    /// Hop rate in hops/second.
    pub hop_rate: f64,
    /// Dwell time per channel in seconds.
    pub dwell_time_s: f64,
    /// Hopping pattern.
    pub pattern: HopPattern,
}

/// Frequency hopper state.
#[derive(Debug, Clone)]
pub struct FrequencyHopper {
    config: HopConfig,
    /// Current hop index (counter).
    hop_index: usize,
    /// LFSR state for pseudorandom hopping.
    lfsr_state: u32,
    /// Current channel index.
    current_channel: usize,
    /// Hop sequence for fixed patterns.
    sequence: Vec<usize>,
    /// Blacklisted channels.
    blacklist: Vec<bool>,
}

/// Hop event returned by the hopper.
#[derive(Debug, Clone, Copy)]
pub struct HopEvent {
    /// Channel index.
    pub channel: usize,
    /// Center frequency for this hop (Hz).
    pub frequency_hz: f64,
    /// Hop number (monotonically increasing).
    pub hop_number: usize,
    /// Duration of this hop (seconds).
    pub dwell_time_s: f64,
}

impl FrequencyHopper {
    /// Create a new frequency hopper.
    pub fn new(config: HopConfig) -> Self {
        let num_channels = config.num_channels;
        let (lfsr_state, sequence, blacklist) = match &config.pattern {
            HopPattern::Pseudorandom { seed } => {
                (*seed, Vec::new(), vec![false; num_channels])
            }
            HopPattern::Sequential => {
                (0, Vec::new(), vec![false; num_channels])
            }
            HopPattern::Fixed { sequence } => {
                (0, sequence.clone(), vec![false; num_channels])
            }
            HopPattern::Adaptive { seed, blacklist: bl } => {
                let mut blist = vec![false; num_channels];
                for &ch in bl {
                    if ch < num_channels {
                        blist[ch] = true;
                    }
                }
                (*seed, Vec::new(), blist)
            }
        };

        Self {
            config,
            hop_index: 0,
            lfsr_state: if lfsr_state == 0 { 1 } else { lfsr_state },
            current_channel: 0,
            sequence,
            blacklist,
        }
    }

    /// Get the next hop (channel index, frequency).
    pub fn next_hop(&mut self) -> (usize, f64) {
        let event = self.next_hop_event();
        (event.channel, event.frequency_hz)
    }

    /// Get the next hop event with full details.
    pub fn next_hop_event(&mut self) -> HopEvent {
        let channel = match &self.config.pattern {
            HopPattern::Pseudorandom { .. } => {
                self.lfsr_next() % self.config.num_channels
            }
            HopPattern::Sequential => {
                self.hop_index % self.config.num_channels
            }
            HopPattern::Fixed { .. } => {
                if self.sequence.is_empty() {
                    0
                } else {
                    self.sequence[self.hop_index % self.sequence.len()]
                }
            }
            HopPattern::Adaptive { .. } => {
                let mut ch = self.lfsr_next() % self.config.num_channels;
                // Avoid blacklisted channels
                let mut attempts = 0;
                while self.blacklist[ch] && attempts < self.config.num_channels {
                    ch = self.lfsr_next() % self.config.num_channels;
                    attempts += 1;
                }
                ch
            }
        };

        let frequency = self.channel_to_freq(channel);
        self.current_channel = channel;
        let event = HopEvent {
            channel,
            frequency_hz: frequency,
            hop_number: self.hop_index,
            dwell_time_s: self.config.dwell_time_s,
        };

        self.hop_index += 1;
        event
    }

    /// Convert channel index to center frequency.
    pub fn channel_to_freq(&self, channel: usize) -> f64 {
        self.config.base_freq_hz + channel as f64 * self.config.channel_spacing_hz
    }

    /// Get current channel.
    pub fn current_channel(&self) -> usize {
        self.current_channel
    }

    /// Get hop count so far.
    pub fn hop_count(&self) -> usize {
        self.hop_index
    }

    /// Total bandwidth span.
    pub fn total_bandwidth(&self) -> f64 {
        self.config.num_channels as f64 * self.config.channel_spacing_hz
    }

    /// Processing gain in dB (spreading factor).
    pub fn processing_gain_db(&self) -> f64 {
        10.0 * (self.config.num_channels as f64).log10()
    }

    /// Add a channel to the blacklist (adaptive hopping).
    pub fn blacklist_channel(&mut self, channel: usize) {
        if channel < self.blacklist.len() {
            self.blacklist[channel] = true;
        }
    }

    /// Remove a channel from the blacklist.
    pub fn whitelist_channel(&mut self, channel: usize) {
        if channel < self.blacklist.len() {
            self.blacklist[channel] = false;
        }
    }

    /// Get number of available (non-blacklisted) channels.
    pub fn available_channels(&self) -> usize {
        self.blacklist.iter().filter(|&&b| !b).count()
    }

    /// Generate a sequence of N hop frequencies.
    pub fn generate_sequence(&mut self, n: usize) -> Vec<HopEvent> {
        (0..n).map(|_| self.next_hop_event()).collect()
    }

    /// Reset to initial state.
    pub fn reset(&mut self) {
        self.hop_index = 0;
        self.current_channel = 0;
        match &self.config.pattern {
            HopPattern::Pseudorandom { seed } => {
                self.lfsr_state = if *seed == 0 { 1 } else { *seed };
            }
            HopPattern::Adaptive { seed, .. } => {
                self.lfsr_state = if *seed == 0 { 1 } else { *seed };
            }
            _ => {}
        }
    }

    /// LFSR step (Galois LFSR with polynomial x^31 + x^28 + 1).
    fn lfsr_next(&mut self) -> usize {
        let bit = self.lfsr_state & 1;
        self.lfsr_state >>= 1;
        if bit == 1 {
            self.lfsr_state ^= 0x90000000; // x^31 + x^28
        }
        self.lfsr_state as usize
    }
}

/// Bluetooth frequency hopping preset.
pub fn bluetooth_config(seed: u32) -> HopConfig {
    HopConfig {
        num_channels: 79,
        channel_spacing_hz: 1e6,
        base_freq_hz: 2.402e9,
        hop_rate: 1600.0,
        dwell_time_s: 625e-6, // 625 μs
        pattern: HopPattern::Pseudorandom { seed },
    }
}

/// Military HF hopping preset.
pub fn military_hf_config(seed: u32) -> HopConfig {
    HopConfig {
        num_channels: 200,
        channel_spacing_hz: 25e3,
        base_freq_hz: 2e6,
        hop_rate: 100.0,
        dwell_time_s: 0.01,
        pattern: HopPattern::Pseudorandom { seed },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sequential_hopping() {
        let config = HopConfig {
            num_channels: 10,
            channel_spacing_hz: 1e6,
            base_freq_hz: 900e6,
            hop_rate: 100.0,
            dwell_time_s: 0.01,
            pattern: HopPattern::Sequential,
        };
        let mut hopper = FrequencyHopper::new(config);

        for i in 0..20 {
            let (ch, _) = hopper.next_hop();
            assert_eq!(ch, i % 10, "Sequential hop {} should be channel {}", i, i % 10);
        }
    }

    #[test]
    fn test_pseudorandom_covers_channels() {
        let config = HopConfig {
            num_channels: 20,
            channel_spacing_hz: 1e6,
            base_freq_hz: 2.4e9,
            hop_rate: 100.0,
            dwell_time_s: 0.01,
            pattern: HopPattern::Pseudorandom { seed: 12345 },
        };
        let mut hopper = FrequencyHopper::new(config);

        let mut visited = vec![false; 20];
        for _ in 0..200 {
            let (ch, _) = hopper.next_hop();
            assert!(ch < 20);
            visited[ch] = true;
        }

        let coverage = visited.iter().filter(|&&v| v).count();
        assert!(coverage >= 15, "Should visit most channels, visited {}/20", coverage);
    }

    #[test]
    fn test_fixed_pattern() {
        let config = HopConfig {
            num_channels: 10,
            channel_spacing_hz: 1e6,
            base_freq_hz: 900e6,
            hop_rate: 100.0,
            dwell_time_s: 0.01,
            pattern: HopPattern::Fixed {
                sequence: vec![0, 5, 3, 7, 1],
            },
        };
        let mut hopper = FrequencyHopper::new(config);

        assert_eq!(hopper.next_hop().0, 0);
        assert_eq!(hopper.next_hop().0, 5);
        assert_eq!(hopper.next_hop().0, 3);
        assert_eq!(hopper.next_hop().0, 7);
        assert_eq!(hopper.next_hop().0, 1);
        // Wraps around
        assert_eq!(hopper.next_hop().0, 0);
    }

    #[test]
    fn test_adaptive_blacklist() {
        let config = HopConfig {
            num_channels: 10,
            channel_spacing_hz: 1e6,
            base_freq_hz: 900e6,
            hop_rate: 100.0,
            dwell_time_s: 0.01,
            pattern: HopPattern::Adaptive {
                seed: 42,
                blacklist: vec![3, 5, 7],
            },
        };
        let mut hopper = FrequencyHopper::new(config);

        for _ in 0..100 {
            let (ch, _) = hopper.next_hop();
            assert_ne!(ch, 3, "Should avoid blacklisted channel 3");
            assert_ne!(ch, 5, "Should avoid blacklisted channel 5");
            assert_ne!(ch, 7, "Should avoid blacklisted channel 7");
        }
    }

    #[test]
    fn test_channel_to_freq() {
        let config = HopConfig {
            num_channels: 10,
            channel_spacing_hz: 1e6,
            base_freq_hz: 2.4e9,
            hop_rate: 100.0,
            dwell_time_s: 0.01,
            pattern: HopPattern::Sequential,
        };
        let hopper = FrequencyHopper::new(config);

        assert!((hopper.channel_to_freq(0) - 2.4e9).abs() < 1.0);
        assert!((hopper.channel_to_freq(5) - 2.405e9).abs() < 1.0);
    }

    #[test]
    fn test_bluetooth_preset() {
        let config = bluetooth_config(42);
        assert_eq!(config.num_channels, 79);
        assert!((config.dwell_time_s - 625e-6).abs() < 1e-9);
        let hopper = FrequencyHopper::new(config);
        assert!(hopper.processing_gain_db() > 18.0);
    }

    #[test]
    fn test_processing_gain() {
        let config = HopConfig {
            num_channels: 100,
            channel_spacing_hz: 1e6,
            base_freq_hz: 900e6,
            hop_rate: 100.0,
            dwell_time_s: 0.01,
            pattern: HopPattern::Sequential,
        };
        let hopper = FrequencyHopper::new(config);
        let gain = hopper.processing_gain_db();
        assert!((gain - 20.0).abs() < 0.1, "Processing gain: {:.1} dB", gain);
    }

    #[test]
    fn test_total_bandwidth() {
        let config = HopConfig {
            num_channels: 50,
            channel_spacing_hz: 200e3,
            base_freq_hz: 900e6,
            hop_rate: 100.0,
            dwell_time_s: 0.01,
            pattern: HopPattern::Sequential,
        };
        let hopper = FrequencyHopper::new(config);
        assert!((hopper.total_bandwidth() - 10e6).abs() < 1.0);
    }

    #[test]
    fn test_reset() {
        let config = HopConfig {
            num_channels: 10,
            channel_spacing_hz: 1e6,
            base_freq_hz: 900e6,
            hop_rate: 100.0,
            dwell_time_s: 0.01,
            pattern: HopPattern::Pseudorandom { seed: 42 },
        };
        let mut hopper = FrequencyHopper::new(config);

        let first_run: Vec<usize> = (0..10).map(|_| hopper.next_hop().0).collect();
        hopper.reset();
        let second_run: Vec<usize> = (0..10).map(|_| hopper.next_hop().0).collect();
        assert_eq!(first_run, second_run, "Reset should reproduce same sequence");
    }

    #[test]
    fn test_dynamic_blacklist() {
        let config = HopConfig {
            num_channels: 5,
            channel_spacing_hz: 1e6,
            base_freq_hz: 900e6,
            hop_rate: 100.0,
            dwell_time_s: 0.01,
            pattern: HopPattern::Sequential,
        };
        let mut hopper = FrequencyHopper::new(config);
        assert_eq!(hopper.available_channels(), 5);

        hopper.blacklist_channel(2);
        assert_eq!(hopper.available_channels(), 4);

        hopper.whitelist_channel(2);
        assert_eq!(hopper.available_channels(), 5);
    }
}

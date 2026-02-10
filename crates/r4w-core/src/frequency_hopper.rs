//! Frequency Hopping Spread Spectrum (FHSS) Controller
//!
//! Implements frequency hopping for military, Bluetooth, and anti-jam applications.
//! Provides trait-based hop sequence generation (pseudo-random LFSR, linear, custom list)
//! and a main `FrequencyHopper` controller that mixes input IQ samples with an NCO
//! at the current hop frequency, advancing to the next frequency after each dwell period.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::frequency_hopper::{
//!     FrequencyHopper, PseudoRandomHopper, HopSequence, bluetooth_hopper,
//! };
//!
//! // Create a Bluetooth-style hopper (79 channels, 1 MHz spacing, 2.402 GHz base)
//! let mut hopper = bluetooth_hopper();
//! let freq = hopper.current_frequency();
//! assert!(freq >= 2_402_000_000.0 && freq < 2_481_000_000.0);
//!
//! // Process IQ samples — the hopper mixes them with the current hop frequency
//! let input = vec![(1.0, 0.0); 10];
//! let output = hopper.process(&input);
//! assert_eq!(output.len(), 10);
//!
//! // Or build a custom pseudo-random hopper
//! let seq = PseudoRandomHopper::new(20, 0xBEEF, 25_000.0, 400_000_000.0);
//! let mut fh = FrequencyHopper::new(Box::new(seq), 256);
//! let freq = fh.current_frequency();
//! assert!(freq >= 400_000_000.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Trait
// ---------------------------------------------------------------------------

/// Trait for generating a sequence of hop frequencies.
pub trait HopSequence {
    /// Return the next frequency in the hop pattern (Hz).
    fn next_frequency(&mut self) -> f64;

    /// Reset the sequence to its initial state.
    fn reset(&mut self);

    /// Total number of channels in the hop set.
    fn num_channels(&self) -> usize;
}

// ---------------------------------------------------------------------------
// PseudoRandomHopper
// ---------------------------------------------------------------------------

/// LFSR-based pseudo-random hop sequence.
///
/// Uses a 31-bit Galois LFSR (polynomial x^31 + x^28 + 1) to generate
/// a pseudo-random channel index, then maps it to a frequency via
/// `base_freq + channel * spacing`.
#[derive(Debug, Clone)]
pub struct PseudoRandomHopper {
    num_channels: usize,
    seed: u32,
    channel_spacing_hz: f64,
    base_freq_hz: f64,
    lfsr_state: u32,
}

impl PseudoRandomHopper {
    /// Create a new pseudo-random hopper.
    ///
    /// * `num_channels` - number of frequency channels
    /// * `seed` - LFSR seed (non-zero; zero is replaced with 1)
    /// * `channel_spacing_hz` - spacing between adjacent channels in Hz
    /// * `base_freq_hz` - center frequency of channel 0
    pub fn new(
        num_channels: usize,
        seed: u32,
        channel_spacing_hz: f64,
        base_freq_hz: f64,
    ) -> Self {
        Self {
            num_channels,
            seed,
            channel_spacing_hz,
            base_freq_hz,
            lfsr_state: if seed == 0 { 1 } else { seed },
        }
    }

    /// Advance the LFSR by one step and return the raw state as `usize`.
    fn lfsr_step(&mut self) -> usize {
        let bit = self.lfsr_state & 1;
        self.lfsr_state >>= 1;
        if bit == 1 {
            self.lfsr_state ^= 0x9000_0000; // x^31 + x^28
        }
        self.lfsr_state as usize
    }
}

impl HopSequence for PseudoRandomHopper {
    fn next_frequency(&mut self) -> f64 {
        let channel = self.lfsr_step() % self.num_channels;
        self.base_freq_hz + channel as f64 * self.channel_spacing_hz
    }

    fn reset(&mut self) {
        self.lfsr_state = if self.seed == 0 { 1 } else { self.seed };
    }

    fn num_channels(&self) -> usize {
        self.num_channels
    }
}

// ---------------------------------------------------------------------------
// LinearHopper
// ---------------------------------------------------------------------------

/// Sequential (linear) hop sequence: channel 0, 1, 2, ..., N-1, repeat.
#[derive(Debug, Clone)]
pub struct LinearHopper {
    num_channels: usize,
    channel_spacing_hz: f64,
    base_freq_hz: f64,
    index: usize,
}

impl LinearHopper {
    /// Create a new linear hopper.
    pub fn new(num_channels: usize, channel_spacing_hz: f64, base_freq_hz: f64) -> Self {
        Self {
            num_channels,
            channel_spacing_hz,
            base_freq_hz,
            index: 0,
        }
    }
}

impl HopSequence for LinearHopper {
    fn next_frequency(&mut self) -> f64 {
        let channel = self.index % self.num_channels;
        self.index += 1;
        self.base_freq_hz + channel as f64 * self.channel_spacing_hz
    }

    fn reset(&mut self) {
        self.index = 0;
    }

    fn num_channels(&self) -> usize {
        self.num_channels
    }
}

// ---------------------------------------------------------------------------
// HopPattern (custom list)
// ---------------------------------------------------------------------------

/// A hop sequence built from an explicit list of frequencies.
#[derive(Debug, Clone)]
pub struct HopPattern {
    frequencies: Vec<f64>,
    index: usize,
}

impl HopPattern {
    /// Create a hop sequence from a user-supplied list of frequencies (Hz).
    ///
    /// The sequence cycles through the list repeatedly.
    pub fn from_list(freqs: &[f64]) -> Self {
        Self {
            frequencies: freqs.to_vec(),
            index: 0,
        }
    }
}

impl HopSequence for HopPattern {
    fn next_frequency(&mut self) -> f64 {
        if self.frequencies.is_empty() {
            return 0.0;
        }
        let freq = self.frequencies[self.index % self.frequencies.len()];
        self.index += 1;
        freq
    }

    fn reset(&mut self) {
        self.index = 0;
    }

    fn num_channels(&self) -> usize {
        self.frequencies.len()
    }
}

// ---------------------------------------------------------------------------
// HopStats
// ---------------------------------------------------------------------------

/// Runtime statistics for a frequency hopper.
#[derive(Debug, Clone)]
pub struct HopStats {
    /// Number of distinct channels visited so far.
    pub channels_visited: usize,
    /// Total number of hops executed.
    pub hop_count: usize,
    /// Dwell time in samples.
    pub dwell_time_samples: usize,
}

// ---------------------------------------------------------------------------
// FrequencyHopper (main controller)
// ---------------------------------------------------------------------------

/// Main frequency-hop controller.
///
/// Wraps a [`HopSequence`] and applies the hop frequency to IQ samples
/// by mixing with an internal NCO. The hop frequency advances every
/// `dwell_time_samples` input samples.
pub struct FrequencyHopper {
    sequence: Box<dyn HopSequence>,
    dwell_time_samples: usize,
    /// Samples processed in the current dwell period.
    samples_in_dwell: usize,
    /// Current hop frequency (Hz).
    current_freq: f64,
    /// Total hops executed.
    hop_count: usize,
    /// NCO phase accumulator (radians).
    phase: f64,
    /// Set of visited channel frequencies (for stats).
    visited: std::collections::HashSet<u64>,
}

impl FrequencyHopper {
    /// Create a new frequency hopper.
    ///
    /// * `sequence` - the hop-sequence generator
    /// * `dwell_time_samples` - number of samples per hop before switching
    pub fn new(mut sequence: Box<dyn HopSequence>, dwell_time_samples: usize) -> Self {
        let first_freq = sequence.next_frequency();
        let mut visited = std::collections::HashSet::new();
        visited.insert(first_freq.to_bits());
        Self {
            sequence,
            dwell_time_samples,
            samples_in_dwell: 0,
            current_freq: first_freq,
            hop_count: 0,
            phase: 0.0,
            visited,
        }
    }

    /// Process a block of IQ samples, mixing each with the NCO at the
    /// current hop frequency. The hop frequency advances every
    /// `dwell_time_samples` samples.
    ///
    /// Input and output are `(I, Q)` tuples representing baseband samples.
    /// The NCO produces `exp(j * 2*pi*f*n)` where `f` is the current hop
    /// frequency and `n` is a notional sample-rate-normalised index.
    /// Because the hop frequency itself *is* the target RF frequency offset,
    /// we use a normalised phase increment of `2*pi*f` per sample (i.e. the
    /// caller is assumed to be working at a 1 Hz sample rate equivalent, or
    /// the frequencies in the sequence are already expressed as normalised
    /// frequencies `f/fs`). For realistic RF mixing the hop sequence should
    /// contain normalised offsets (freq_hz / sample_rate).
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut output = Vec::with_capacity(input.len());
        for &(i_in, q_in) in input {
            // Check if we need to hop
            if self.samples_in_dwell >= self.dwell_time_samples && self.dwell_time_samples > 0 {
                self.current_freq = self.sequence.next_frequency();
                self.visited.insert(self.current_freq.to_bits());
                self.hop_count += 1;
                self.samples_in_dwell = 0;
            }

            // NCO mixing: multiply input by exp(j * phase)
            let cos_p = self.phase.cos();
            let sin_p = self.phase.sin();
            let i_out = i_in * cos_p - q_in * sin_p;
            let q_out = i_in * sin_p + q_in * cos_p;
            output.push((i_out, q_out));

            // Advance phase
            self.phase += 2.0 * PI * self.current_freq;
            // Wrap to [-pi, pi]
            while self.phase > PI {
                self.phase -= 2.0 * PI;
            }
            while self.phase < -PI {
                self.phase += 2.0 * PI;
            }

            self.samples_in_dwell += 1;
        }
        output
    }

    /// Return the current hop frequency (Hz).
    pub fn current_frequency(&self) -> f64 {
        self.current_freq
    }

    /// Number of completed hops so far.
    pub fn hop_count(&self) -> usize {
        self.hop_count
    }

    /// Reset the hopper (sequence, counters, NCO phase).
    pub fn reset(&mut self) {
        self.sequence.reset();
        self.current_freq = self.sequence.next_frequency();
        self.visited.clear();
        self.visited.insert(self.current_freq.to_bits());
        self.hop_count = 0;
        self.samples_in_dwell = 0;
        self.phase = 0.0;
    }

    /// Gather runtime statistics.
    pub fn stats(&self) -> HopStats {
        HopStats {
            channels_visited: self.visited.len(),
            hop_count: self.hop_count,
            dwell_time_samples: self.dwell_time_samples,
        }
    }
}

// ---------------------------------------------------------------------------
// Factory helpers
// ---------------------------------------------------------------------------

/// Create a Bluetooth-style frequency hopper.
///
/// 79 channels, 1 MHz spacing, base frequency 2.402 GHz, LFSR seed 1.
/// Dwell time is set to 30 samples (a placeholder; real BT uses 625 us
/// which depends on sample rate).
pub fn bluetooth_hopper() -> FrequencyHopper {
    let seq = PseudoRandomHopper::new(79, 1, 1_000_000.0, 2_402_000_000.0);
    FrequencyHopper::new(Box::new(seq), 30)
}

/// Create a wideband military-style frequency hopper.
///
/// * `num_channels` - number of hop channels
/// * `bandwidth_hz` - total bandwidth to spread across
///
/// Channel spacing is `bandwidth_hz / num_channels`. Base frequency is 30 MHz
/// (HF band). LFSR seed is 0xDEAD. Dwell time is 64 samples.
pub fn military_hopper(num_channels: usize, bandwidth_hz: f64) -> FrequencyHopper {
    let spacing = if num_channels > 0 {
        bandwidth_hz / num_channels as f64
    } else {
        bandwidth_hz
    };
    let seq = PseudoRandomHopper::new(num_channels, 0xDEAD, spacing, 30_000_000.0);
    FrequencyHopper::new(Box::new(seq), 64)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pseudo_random_unique_channels() {
        let mut hopper = PseudoRandomHopper::new(20, 42, 1_000_000.0, 2_400_000_000.0);
        let mut seen = std::collections::HashSet::new();
        for _ in 0..500 {
            let freq = hopper.next_frequency();
            seen.insert(freq.to_bits());
        }
        // With 20 channels and 500 draws, we should visit all 20
        assert_eq!(
            seen.len(),
            20,
            "Expected 20 unique channels, got {}",
            seen.len()
        );
    }

    #[test]
    fn test_linear_hopper_cycles_sequentially() {
        let mut hopper = LinearHopper::new(5, 100.0, 1000.0);
        let expected = [1000.0, 1100.0, 1200.0, 1300.0, 1400.0, 1000.0, 1100.0];
        for &exp in &expected {
            let freq = hopper.next_frequency();
            assert!(
                (freq - exp).abs() < 1e-6,
                "Expected {exp}, got {freq}"
            );
        }
    }

    #[test]
    fn test_frequency_hopper_processes_samples() {
        // Use a zero-frequency hop so the NCO is just multiplying by 1+0j
        let seq = HopPattern::from_list(&[0.0]);
        let mut fh = FrequencyHopper::new(Box::new(seq), 100);

        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 10];
        let output = fh.process(&input);
        assert_eq!(output.len(), 10);
        // With freq=0 the NCO phase stays 0, cos=1 sin=0, output == input
        for (i, &(oi, oq)) in output.iter().enumerate() {
            assert!(
                (oi - 1.0).abs() < 1e-10 && oq.abs() < 1e-10,
                "Sample {i}: expected (1,0), got ({oi},{oq})"
            );
        }
    }

    #[test]
    fn test_dwell_time_triggers_hop() {
        let seq = LinearHopper::new(5, 100.0, 1000.0);
        let dwell = 4;
        let mut fh = FrequencyHopper::new(Box::new(seq), dwell);

        // Initial frequency is channel 0 = 1000 Hz (consumed by constructor)
        assert!((fh.current_frequency() - 1000.0).abs() < 1e-6);
        assert_eq!(fh.hop_count(), 0);

        // Process exactly `dwell` samples — should still be on first hop during
        // these samples, then the *next* sample triggers a hop.
        let input = vec![(1.0, 0.0); dwell];
        let _ = fh.process(&input);
        // After processing `dwell` samples, samples_in_dwell == dwell, but
        // the hop fires on the *next* process call's first sample.
        assert_eq!(fh.hop_count(), 0);

        // One more sample triggers the hop
        let _ = fh.process(&[(1.0, 0.0)]);
        assert_eq!(fh.hop_count(), 1);
        // Next frequency from LinearHopper is channel 1 = 1100 Hz
        assert!(
            (fh.current_frequency() - 1100.0).abs() < 1e-6,
            "After hop, freq should be 1100, got {}",
            fh.current_frequency()
        );
    }

    #[test]
    fn test_custom_hop_pattern_from_list() {
        let freqs = vec![100.0, 200.0, 300.0];
        let mut pat = HopPattern::from_list(&freqs);
        assert_eq!(pat.num_channels(), 3);

        assert!((pat.next_frequency() - 100.0).abs() < 1e-10);
        assert!((pat.next_frequency() - 200.0).abs() < 1e-10);
        assert!((pat.next_frequency() - 300.0).abs() < 1e-10);
        // wraps
        assert!((pat.next_frequency() - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_bluetooth_factory_79_channels() {
        let hopper = bluetooth_hopper();
        // The underlying sequence has 79 channels.
        // current_frequency should be within the Bluetooth band.
        let freq = hopper.current_frequency();
        assert!(
            freq >= 2_402_000_000.0 && freq < 2_402_000_000.0 + 79.0 * 1_000_000.0,
            "Bluetooth freq {freq} out of range"
        );
    }

    #[test]
    fn test_military_factory_wideband() {
        let hopper = military_hopper(200, 5_000_000.0);
        let freq = hopper.current_frequency();
        let spacing = 5_000_000.0 / 200.0;
        assert!(
            freq >= 30_000_000.0 && freq < 30_000_000.0 + 200.0 * spacing,
            "Military freq {freq} out of range"
        );
    }

    #[test]
    fn test_reset_restores_initial_state() {
        let seq = PseudoRandomHopper::new(10, 77, 1_000.0, 0.0);
        let mut fh = FrequencyHopper::new(Box::new(seq), 8);

        // Collect first 5 hop frequencies
        let mut first_run = vec![fh.current_frequency()];
        for _ in 0..4 {
            let _ = fh.process(&vec![(1.0, 0.0); 8 + 1]); // trigger hop
            first_run.push(fh.current_frequency());
        }

        fh.reset();

        let mut second_run = vec![fh.current_frequency()];
        for _ in 0..4 {
            let _ = fh.process(&vec![(1.0, 0.0); 8 + 1]); // trigger hop
            second_run.push(fh.current_frequency());
        }

        assert_eq!(first_run, second_run, "Reset should reproduce the same hop sequence");
    }

    #[test]
    fn test_hop_count_increments() {
        // Process one sample at a time and track hop_count.
        // With dwell_time_samples=D, a hop fires when samples_in_dwell >= D.
        // The first dwell period takes D+1 samples (sid starts at 0, reaches D
        // on the (D+1)-th sample). Each subsequent period takes D samples
        // because the hop resets sid to 0 then the triggering sample increments
        // it to 1.
        let seq = LinearHopper::new(10, 100.0, 0.0);
        let dwell = 4;
        let mut fh = FrequencyHopper::new(Box::new(seq), dwell);

        assert_eq!(fh.hop_count(), 0);

        // First dwell period: need D samples to fill, then 1 more to trigger.
        for _ in 0..dwell {
            let _ = fh.process(&[(0.0, 0.0)]);
        }
        assert_eq!(fh.hop_count(), 0, "no hop yet after {dwell} samples");

        let _ = fh.process(&[(0.0, 0.0)]);
        assert_eq!(fh.hop_count(), 1, "first hop after {dwell}+1 samples");

        // Subsequent periods: D-1 non-hop samples then 1 triggers.
        for _ in 0..(dwell - 1) {
            let _ = fh.process(&[(0.0, 0.0)]);
        }
        assert_eq!(fh.hop_count(), 1, "still 1 hop after D-1 more samples");

        let _ = fh.process(&[(0.0, 0.0)]);
        assert_eq!(fh.hop_count(), 2, "second hop fires");

        for _ in 0..(dwell - 1) {
            let _ = fh.process(&[(0.0, 0.0)]);
        }
        assert_eq!(fh.hop_count(), 2);
        let _ = fh.process(&[(0.0, 0.0)]);
        assert_eq!(fh.hop_count(), 3, "third hop fires");
    }

    #[test]
    fn test_different_seeds_produce_different_sequences() {
        let mut h1 = PseudoRandomHopper::new(50, 1, 1_000.0, 0.0);
        let mut h2 = PseudoRandomHopper::new(50, 9999, 1_000.0, 0.0);

        let seq1: Vec<u64> = (0..20).map(|_| h1.next_frequency().to_bits()).collect();
        let seq2: Vec<u64> = (0..20).map(|_| h2.next_frequency().to_bits()).collect();

        assert_ne!(seq1, seq2, "Different seeds must produce different sequences");
    }
}

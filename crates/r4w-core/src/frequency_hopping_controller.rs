//! Frequency Hopping Spread Spectrum (FHSS) Controller
//!
//! This module implements frequency hopping spread spectrum control including
//! hop sequence generation, timing, and synchronization. It provides:
//!
//! - **Pseudo-random hop sequences** using a 16-bit LFSR
//! - **Hop timing control** with configurable dwell time and guard time
//! - **Frequency synthesizer model** mapping hop indices to RF frequencies
//! - **Slow and fast hopping** modes
//! - **Preamble-based synchronization** for hop alignment
//! - **Anti-jam margin** and **processing gain** estimation
//! - **Hop set optimization** enforcing minimum frequency spacing
//! - **IQ signal generation** with frequency hopping applied
//!
//! # Example
//!
//! ```
//! use r4w_core::frequency_hopping_controller::{
//!     FrequencyHoppingController, HoppingMode, HopTimingConfig,
//! };
//!
//! // Define a hop set of 8 frequencies (in Hz)
//! let hop_set: Vec<f64> = (0..8).map(|i| 900.0e6 + i as f64 * 1.0e6).collect();
//!
//! let timing = HopTimingConfig {
//!     dwell_time_s: 0.010,   // 10 ms per hop
//!     guard_time_s: 0.001,   // 1 ms guard between hops
//!     symbol_duration_s: 0.005, // 5 ms per symbol
//! };
//!
//! let mut ctrl = FrequencyHoppingController::new(
//!     hop_set,
//!     timing,
//!     HoppingMode::Slow { symbols_per_hop: 2 },
//!     42, // LFSR seed
//! );
//!
//! // Generate the first 4 hop frequencies
//! let freqs: Vec<f64> = (0..4).map(|_| ctrl.next_hop_frequency()).collect();
//! assert_eq!(freqs.len(), 4);
//!
//! // Processing gain depends on the number of hop channels
//! let gain_db = FrequencyHoppingController::processing_gain_db(8);
//! assert!((gain_db - 9.03).abs() < 0.1); // 10*log10(8) ~ 9.03
//! ```

use std::f64::consts::PI;

/// Configuration for hop timing parameters.
#[derive(Debug, Clone)]
pub struct HopTimingConfig {
    /// Time spent on each frequency before hopping (seconds).
    pub dwell_time_s: f64,
    /// Guard interval between hops to allow synthesizer settling (seconds).
    pub guard_time_s: f64,
    /// Duration of one data symbol (seconds).
    pub symbol_duration_s: f64,
}

/// Hopping mode: slow (multiple symbols per hop) or fast (multiple hops per symbol).
#[derive(Debug, Clone, PartialEq)]
pub enum HoppingMode {
    /// Multiple symbols are transmitted on a single hop frequency.
    Slow { symbols_per_hop: usize },
    /// Multiple hops occur within a single symbol period.
    Fast { hops_per_symbol: usize },
}

/// Result of a preamble synchronization attempt.
#[derive(Debug, Clone)]
pub struct SyncResult {
    /// Whether synchronization was achieved.
    pub locked: bool,
    /// Sample offset where the preamble was detected.
    pub offset: usize,
    /// Correlation peak value (0.0 to 1.0).
    pub correlation: f64,
}

/// Frequency hopping spread spectrum controller.
///
/// Manages hop sequence generation, timing, frequency mapping,
/// synchronization, and IQ signal generation.
#[derive(Debug, Clone)]
pub struct FrequencyHoppingController {
    /// Set of available hop frequencies (Hz).
    hop_set: Vec<f64>,
    /// Timing configuration.
    timing: HopTimingConfig,
    /// Hopping mode (slow or fast).
    mode: HoppingMode,
    /// LFSR state for pseudo-random sequence generation.
    lfsr_state: u16,
    /// Initial LFSR seed (for resetting).
    lfsr_seed: u16,
    /// Current hop index within the sequence.
    hop_index: usize,
    /// Pre-computed hop sequence (indices into hop_set).
    hop_sequence: Vec<usize>,
    /// Length of the pre-computed sequence.
    sequence_length: usize,
    /// Preamble pattern (hop indices) used for synchronization.
    preamble_pattern: Vec<usize>,
}

impl FrequencyHoppingController {
    /// Creates a new frequency hopping controller.
    ///
    /// # Arguments
    ///
    /// * `hop_set` - Available frequencies in Hz (must have at least 2 entries)
    /// * `timing` - Hop timing configuration
    /// * `mode` - Slow or fast hopping mode
    /// * `seed` - LFSR seed for pseudo-random hop sequence (must be non-zero)
    ///
    /// # Panics
    ///
    /// Panics if `hop_set` has fewer than 2 entries or `seed` is 0.
    pub fn new(
        hop_set: Vec<f64>,
        timing: HopTimingConfig,
        mode: HoppingMode,
        seed: u16,
    ) -> Self {
        assert!(hop_set.len() >= 2, "Hop set must contain at least 2 frequencies");
        assert!(seed != 0, "LFSR seed must be non-zero");

        let n = hop_set.len();
        // Pre-compute a full cycle of hop indices
        let sequence_length = n * 4; // generate enough hops
        let mut controller = Self {
            hop_set,
            timing,
            mode,
            lfsr_state: seed,
            lfsr_seed: seed,
            hop_index: 0,
            hop_sequence: Vec::with_capacity(sequence_length),
            sequence_length,
            preamble_pattern: Vec::new(),
        };

        // Generate the hop sequence
        for _ in 0..sequence_length {
            let idx = controller.lfsr_next() % n;
            controller.hop_sequence.push(idx);
        }

        // Default preamble: first 4 hops of the sequence
        let preamble_len = 4.min(sequence_length);
        controller.preamble_pattern = controller.hop_sequence[..preamble_len].to_vec();

        // Reset LFSR so next_hop starts fresh
        controller.lfsr_state = seed;
        controller.hop_index = 0;

        controller
    }

    /// Advances the LFSR by one step and returns the output value.
    fn lfsr_next(&mut self) -> usize {
        let feedback = (self.lfsr_state ^ (self.lfsr_state >> 2)
            ^ (self.lfsr_state >> 3)
            ^ (self.lfsr_state >> 5))
            & 1;
        self.lfsr_state = (self.lfsr_state >> 1) | (feedback << 15);
        self.lfsr_state as usize
    }

    /// Returns the frequency (Hz) for the next hop in the sequence.
    pub fn next_hop_frequency(&mut self) -> f64 {
        let idx = self.hop_sequence[self.hop_index % self.sequence_length];
        self.hop_index += 1;
        self.hop_set[idx]
    }

    /// Returns the hop set index for the next hop without advancing.
    pub fn peek_hop_index(&self) -> usize {
        self.hop_sequence[self.hop_index % self.sequence_length]
    }

    /// Returns the current hop index counter.
    pub fn current_hop_count(&self) -> usize {
        self.hop_index
    }

    /// Resets the controller to the beginning of the hop sequence.
    pub fn reset(&mut self) {
        self.lfsr_state = self.lfsr_seed;
        self.hop_index = 0;
    }

    /// Returns a reference to the hop set.
    pub fn hop_set(&self) -> &[f64] {
        &self.hop_set
    }

    /// Returns a reference to the timing configuration.
    pub fn timing(&self) -> &HopTimingConfig {
        &self.timing
    }

    /// Returns a reference to the hopping mode.
    pub fn mode(&self) -> &HoppingMode {
        &self.mode
    }

    /// Returns the pre-computed hop sequence (as indices into the hop set).
    pub fn hop_sequence(&self) -> &[usize] {
        &self.hop_sequence
    }

    /// Returns the preamble pattern used for synchronization.
    pub fn preamble_pattern(&self) -> &[usize] {
        &self.preamble_pattern
    }

    /// Sets a custom preamble pattern for synchronization.
    ///
    /// Each element is an index into the hop set.
    pub fn set_preamble_pattern(&mut self, pattern: Vec<usize>) {
        let n = self.hop_set.len();
        for &idx in &pattern {
            assert!(idx < n, "Preamble index {} out of range for hop set of size {}", idx, n);
        }
        self.preamble_pattern = pattern;
    }

    // -----------------------------------------------------------------------
    // Frequency synthesizer model
    // -----------------------------------------------------------------------

    /// Maps a hop index to its corresponding frequency (Hz).
    ///
    /// This models the frequency synthesizer that tunes the radio to the
    /// appropriate channel on each hop.
    pub fn index_to_frequency(&self, hop_idx: usize) -> f64 {
        self.hop_set[hop_idx % self.hop_set.len()]
    }

    /// Returns the channel spacing (Hz) assuming uniform hop set spacing.
    /// Computed as the minimum positive difference between sorted frequencies.
    pub fn channel_spacing(&self) -> f64 {
        if self.hop_set.len() < 2 {
            return 0.0;
        }
        let mut sorted = self.hop_set.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut min_spacing = f64::MAX;
        for i in 1..sorted.len() {
            let diff = sorted[i] - sorted[i - 1];
            if diff > 0.0 && diff < min_spacing {
                min_spacing = diff;
            }
        }
        min_spacing
    }

    /// Returns the total bandwidth spanned by the hop set (Hz).
    pub fn total_bandwidth(&self) -> f64 {
        let min = self.hop_set.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = self.hop_set.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        max - min
    }

    // -----------------------------------------------------------------------
    // Hop timing
    // -----------------------------------------------------------------------

    /// Returns the effective hop rate (hops per second).
    pub fn hop_rate(&self) -> f64 {
        1.0 / (self.timing.dwell_time_s + self.timing.guard_time_s)
    }

    /// Returns the number of symbols transmitted per hop (slow hopping)
    /// or the number of hops per symbol (fast hopping).
    pub fn symbols_per_hop_or_hops_per_symbol(&self) -> (usize, bool) {
        match &self.mode {
            HoppingMode::Slow { symbols_per_hop } => (*symbols_per_hop, true),
            HoppingMode::Fast { hops_per_symbol } => (*hops_per_symbol, false),
        }
    }

    /// Returns the time offset (in seconds) for a given hop number,
    /// accounting for dwell time and guard time.
    pub fn hop_start_time(&self, hop_number: usize) -> f64 {
        hop_number as f64 * (self.timing.dwell_time_s + self.timing.guard_time_s)
    }

    /// Returns the number of samples in one dwell period given a sample rate.
    pub fn dwell_samples(&self, sample_rate: f64) -> usize {
        (self.timing.dwell_time_s * sample_rate).round() as usize
    }

    /// Returns the number of samples in one guard period given a sample rate.
    pub fn guard_samples(&self, sample_rate: f64) -> usize {
        (self.timing.guard_time_s * sample_rate).round() as usize
    }

    // -----------------------------------------------------------------------
    // Synchronization
    // -----------------------------------------------------------------------

    /// Detects the preamble pattern in a sequence of observed hop indices.
    ///
    /// Returns a `SyncResult` indicating whether the preamble was found,
    /// the offset, and a correlation value.
    pub fn detect_preamble(&self, observed_hops: &[usize]) -> SyncResult {
        let pattern = &self.preamble_pattern;
        if pattern.is_empty() || observed_hops.len() < pattern.len() {
            return SyncResult {
                locked: false,
                offset: 0,
                correlation: 0.0,
            };
        }

        let mut best_offset = 0;
        let mut best_corr = 0.0;

        for start in 0..=(observed_hops.len() - pattern.len()) {
            let matches = pattern
                .iter()
                .zip(&observed_hops[start..start + pattern.len()])
                .filter(|(a, b)| a == b)
                .count();
            let corr = matches as f64 / pattern.len() as f64;
            if corr > best_corr {
                best_corr = corr;
                best_offset = start;
            }
        }

        SyncResult {
            locked: best_corr >= 0.75, // 75% match threshold
            offset: best_offset,
            correlation: best_corr,
        }
    }

    /// Generates the preamble as a sequence of frequencies for transmission.
    pub fn preamble_frequencies(&self) -> Vec<f64> {
        self.preamble_pattern
            .iter()
            .map(|&idx| self.hop_set[idx])
            .collect()
    }

    // -----------------------------------------------------------------------
    // Anti-jam and processing gain
    // -----------------------------------------------------------------------

    /// Computes the processing gain in dB for a given number of hop channels.
    ///
    /// Processing gain = 10 * log10(N) where N is the number of channels.
    pub fn processing_gain_db(num_channels: usize) -> f64 {
        assert!(num_channels >= 1, "Must have at least 1 channel");
        10.0 * (num_channels as f64).log10()
    }

    /// Estimates the anti-jam margin in dB.
    ///
    /// AJ margin = processing_gain - system_losses - snr_per_channel
    ///
    /// # Arguments
    ///
    /// * `system_losses_db` - Total system losses in dB
    /// * `snr_per_channel_db` - Required SNR per channel in dB
    pub fn anti_jam_margin_db(&self, system_losses_db: f64, snr_per_channel_db: f64) -> f64 {
        let pg = Self::processing_gain_db(self.hop_set.len());
        pg - system_losses_db - snr_per_channel_db
    }

    /// Estimates the fraction of channels a jammer must jam to deny
    /// communication, assuming uniform hopping.
    ///
    /// Returns a value between 0.0 and 1.0.
    pub fn jammer_fraction_required(&self, jammer_power_to_signal_ratio_db: f64) -> f64 {
        let jsr_linear = 10.0_f64.powf(jammer_power_to_signal_ratio_db / 10.0);
        let n = self.hop_set.len() as f64;
        (jsr_linear / n).min(1.0).max(0.0)
    }

    // -----------------------------------------------------------------------
    // Hop set optimization
    // -----------------------------------------------------------------------

    /// Reorders the hop sequence to enforce a minimum frequency distance
    /// between consecutive hops.
    ///
    /// Returns the number of swaps performed. The algorithm iterates through
    /// the sequence and swaps any hop that violates the minimum distance
    /// with the nearest valid candidate later in the sequence.
    pub fn optimize_min_hop_distance(&mut self, min_distance_hz: f64) -> usize {
        let n = self.hop_sequence.len();
        let mut swaps = 0;

        for i in 1..n {
            let prev_freq = self.hop_set[self.hop_sequence[i - 1]];
            let curr_freq = self.hop_set[self.hop_sequence[i]];

            if (curr_freq - prev_freq).abs() < min_distance_hz {
                // Find the nearest swap candidate that satisfies the constraint
                let mut best_j = None;
                let mut best_dist = f64::MAX;

                for j in (i + 1)..n {
                    let cand_freq = self.hop_set[self.hop_sequence[j]];
                    let dist = (cand_freq - prev_freq).abs();
                    if dist >= min_distance_hz && dist < best_dist {
                        best_dist = dist;
                        best_j = Some(j);
                    }
                }

                if let Some(j) = best_j {
                    self.hop_sequence.swap(i, j);
                    swaps += 1;
                }
            }
        }

        swaps
    }

    /// Validates the hop sequence for minimum distance constraint.
    ///
    /// Returns the number of violations.
    pub fn count_min_distance_violations(&self, min_distance_hz: f64) -> usize {
        let mut violations = 0;
        for i in 1..self.hop_sequence.len() {
            let prev_freq = self.hop_set[self.hop_sequence[i - 1]];
            let curr_freq = self.hop_set[self.hop_sequence[i]];
            if (curr_freq - prev_freq).abs() < min_distance_hz {
                violations += 1;
            }
        }
        violations
    }

    // -----------------------------------------------------------------------
    // IQ signal generation
    // -----------------------------------------------------------------------

    /// Generates IQ samples with frequency hopping applied to a baseband signal.
    ///
    /// Each hop period generates `dwell_samples` of signal at the hop
    /// frequency offset from `center_freq_hz`, followed by `guard_samples`
    /// of zeros (silence during synthesizer settling).
    ///
    /// The baseband signal is a constant-envelope tone at each hop offset.
    ///
    /// # Arguments
    ///
    /// * `num_hops` - Number of hops to generate
    /// * `sample_rate` - Sample rate in Hz
    /// * `center_freq_hz` - Center frequency (hop offsets are relative to this)
    ///
    /// # Returns
    ///
    /// Vector of `(f64, f64)` IQ samples (re, im).
    pub fn generate_hopped_iq(
        &mut self,
        num_hops: usize,
        sample_rate: f64,
        center_freq_hz: f64,
    ) -> Vec<(f64, f64)> {
        let dwell_n = self.dwell_samples(sample_rate);
        let guard_n = self.guard_samples(sample_rate);
        let total_samples = num_hops * (dwell_n + guard_n);
        let mut iq = Vec::with_capacity(total_samples);

        let mut phase = 0.0_f64;

        for _ in 0..num_hops {
            let hop_freq = self.next_hop_frequency();
            let freq_offset = hop_freq - center_freq_hz;
            let phase_inc = 2.0 * PI * freq_offset / sample_rate;

            // Dwell period: generate tone at the hop frequency offset
            for _ in 0..dwell_n {
                let re = phase.cos();
                let im = phase.sin();
                iq.push((re, im));
                phase += phase_inc;
                // Keep phase wrapped to avoid precision loss
                if phase > 2.0 * PI {
                    phase -= 2.0 * PI;
                } else if phase < -2.0 * PI {
                    phase += 2.0 * PI;
                }
            }

            // Guard period: zeros (synthesizer settling)
            for _ in 0..guard_n {
                iq.push((0.0, 0.0));
            }
        }

        iq
    }

    /// Applies frequency hopping to an existing baseband IQ signal.
    ///
    /// Multiplies the input signal by a complex exponential at each hop's
    /// frequency offset, creating the hopped output. Guard intervals are
    /// filled with zeros.
    ///
    /// # Arguments
    ///
    /// * `baseband` - Input baseband IQ samples
    /// * `sample_rate` - Sample rate in Hz
    /// * `center_freq_hz` - Center frequency for offset computation
    ///
    /// # Returns
    ///
    /// Vector of `(f64, f64)` IQ samples with hopping applied.
    pub fn apply_hopping(
        &mut self,
        baseband: &[(f64, f64)],
        sample_rate: f64,
        center_freq_hz: f64,
    ) -> Vec<(f64, f64)> {
        let dwell_n = self.dwell_samples(sample_rate);
        let guard_n = self.guard_samples(sample_rate);
        let hop_len = dwell_n + guard_n;

        if hop_len == 0 {
            return Vec::new();
        }

        let num_hops = (baseband.len() + dwell_n - 1) / dwell_n;
        let mut output = Vec::with_capacity(num_hops * hop_len);
        let mut bb_idx = 0;
        let mut phase = 0.0_f64;

        for _ in 0..num_hops {
            let hop_freq = self.next_hop_frequency();
            let freq_offset = hop_freq - center_freq_hz;
            let phase_inc = 2.0 * PI * freq_offset / sample_rate;

            // Dwell: mix baseband with hop frequency
            for _ in 0..dwell_n {
                if bb_idx < baseband.len() {
                    let (bre, bim) = baseband[bb_idx];
                    let cos_p = phase.cos();
                    let sin_p = phase.sin();
                    // Complex multiply: (bre + j*bim) * (cos + j*sin)
                    let re = bre * cos_p - bim * sin_p;
                    let im = bre * sin_p + bim * cos_p;
                    output.push((re, im));
                    bb_idx += 1;
                } else {
                    output.push((0.0, 0.0));
                }
                phase += phase_inc;
                if phase > 2.0 * PI {
                    phase -= 2.0 * PI;
                } else if phase < -2.0 * PI {
                    phase += 2.0 * PI;
                }
            }

            // Guard: zeros
            for _ in 0..guard_n {
                output.push((0.0, 0.0));
            }
        }

        output
    }

    /// Computes the instantaneous frequency (Hz) of a hopped IQ signal
    /// by measuring the phase difference between consecutive samples.
    ///
    /// Returns a vector of estimated frequencies, one fewer element than input.
    pub fn estimate_instantaneous_frequency(
        iq: &[(f64, f64)],
        sample_rate: f64,
    ) -> Vec<f64> {
        if iq.len() < 2 {
            return Vec::new();
        }

        let mut freqs = Vec::with_capacity(iq.len() - 1);

        for i in 1..iq.len() {
            let (r0, i0) = iq[i - 1];
            let (r1, i1) = iq[i];

            // Conjugate multiply: iq[i] * conj(iq[i-1])
            let re = r1 * r0 + i1 * i0;
            let im = i1 * r0 - r1 * i0;

            let phase_diff = im.atan2(re);
            let freq = phase_diff * sample_rate / (2.0 * PI);
            freqs.push(freq);
        }

        freqs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_hop_set(n: usize) -> Vec<f64> {
        (0..n).map(|i| 900.0e6 + i as f64 * 1.0e6).collect()
    }

    fn default_timing() -> HopTimingConfig {
        HopTimingConfig {
            dwell_time_s: 0.010,
            guard_time_s: 0.001,
            symbol_duration_s: 0.005,
        }
    }

    fn make_controller(n_channels: usize, seed: u16) -> FrequencyHoppingController {
        FrequencyHoppingController::new(
            make_hop_set(n_channels),
            default_timing(),
            HoppingMode::Slow { symbols_per_hop: 2 },
            seed,
        )
    }

    // ------------------------------------------------------------------
    // 1. Construction and basic properties
    // ------------------------------------------------------------------

    #[test]
    fn test_construction() {
        let ctrl = make_controller(8, 42);
        assert_eq!(ctrl.hop_set().len(), 8);
        assert_eq!(ctrl.current_hop_count(), 0);
        assert_eq!(ctrl.preamble_pattern().len(), 4);
    }

    #[test]
    #[should_panic(expected = "at least 2 frequencies")]
    fn test_too_few_channels() {
        FrequencyHoppingController::new(
            vec![900.0e6],
            default_timing(),
            HoppingMode::Slow { symbols_per_hop: 1 },
            1,
        );
    }

    #[test]
    #[should_panic(expected = "LFSR seed must be non-zero")]
    fn test_zero_seed() {
        make_controller(4, 0);
    }

    // ------------------------------------------------------------------
    // 2. LFSR hop sequence generation
    // ------------------------------------------------------------------

    #[test]
    fn test_hop_sequence_is_deterministic() {
        let ctrl1 = make_controller(8, 42);
        let ctrl2 = make_controller(8, 42);
        assert_eq!(ctrl1.hop_sequence(), ctrl2.hop_sequence());
    }

    #[test]
    fn test_different_seeds_different_sequences() {
        let ctrl1 = make_controller(8, 42);
        let ctrl2 = make_controller(8, 99);
        // Extremely unlikely to produce identical sequences
        assert_ne!(ctrl1.hop_sequence(), ctrl2.hop_sequence());
    }

    #[test]
    fn test_hop_indices_in_range() {
        let ctrl = make_controller(8, 42);
        for &idx in ctrl.hop_sequence() {
            assert!(idx < 8, "Hop index {} out of range", idx);
        }
    }

    // ------------------------------------------------------------------
    // 3. Next hop frequency and reset
    // ------------------------------------------------------------------

    #[test]
    fn test_next_hop_frequency_advances() {
        let mut ctrl = make_controller(8, 42);
        let f1 = ctrl.next_hop_frequency();
        assert_eq!(ctrl.current_hop_count(), 1);
        let _f2 = ctrl.next_hop_frequency();
        assert_eq!(ctrl.current_hop_count(), 2);
        // Frequency should be in the hop set
        assert!(f1 >= 900.0e6 && f1 <= 907.0e6);
    }

    #[test]
    fn test_reset_restarts_sequence() {
        let mut ctrl = make_controller(8, 42);
        let f1 = ctrl.next_hop_frequency();
        let f2 = ctrl.next_hop_frequency();
        ctrl.reset();
        assert_eq!(ctrl.current_hop_count(), 0);
        let f1b = ctrl.next_hop_frequency();
        let f2b = ctrl.next_hop_frequency();
        assert_eq!(f1, f1b);
        assert_eq!(f2, f2b);
    }

    // ------------------------------------------------------------------
    // 4. Frequency synthesizer model
    // ------------------------------------------------------------------

    #[test]
    fn test_index_to_frequency() {
        let ctrl = make_controller(4, 1);
        assert_eq!(ctrl.index_to_frequency(0), 900.0e6);
        assert_eq!(ctrl.index_to_frequency(1), 901.0e6);
        assert_eq!(ctrl.index_to_frequency(3), 903.0e6);
        // Wraps around
        assert_eq!(ctrl.index_to_frequency(4), 900.0e6);
    }

    #[test]
    fn test_channel_spacing() {
        let ctrl = make_controller(8, 42);
        let spacing = ctrl.channel_spacing();
        assert!((spacing - 1.0e6).abs() < 1.0, "Expected 1 MHz spacing");
    }

    #[test]
    fn test_total_bandwidth() {
        let ctrl = make_controller(8, 42);
        let bw = ctrl.total_bandwidth();
        assert!((bw - 7.0e6).abs() < 1.0, "Expected 7 MHz total bandwidth");
    }

    // ------------------------------------------------------------------
    // 5. Hop timing
    // ------------------------------------------------------------------

    #[test]
    fn test_hop_rate() {
        let ctrl = make_controller(8, 42);
        let rate = ctrl.hop_rate();
        // dwell=10ms, guard=1ms => hop period=11ms => rate ~ 90.9 hops/s
        assert!((rate - 90.909).abs() < 0.1);
    }

    #[test]
    fn test_dwell_and_guard_samples() {
        let ctrl = make_controller(8, 42);
        let sr = 100_000.0; // 100 kHz
        assert_eq!(ctrl.dwell_samples(sr), 1000); // 10ms * 100kHz
        assert_eq!(ctrl.guard_samples(sr), 100);  // 1ms * 100kHz
    }

    #[test]
    fn test_hop_start_time() {
        let ctrl = make_controller(8, 42);
        assert!((ctrl.hop_start_time(0) - 0.0).abs() < 1e-12);
        assert!((ctrl.hop_start_time(1) - 0.011).abs() < 1e-9);
        assert!((ctrl.hop_start_time(10) - 0.110).abs() < 1e-9);
    }

    // ------------------------------------------------------------------
    // 6. Slow vs fast hopping mode
    // ------------------------------------------------------------------

    #[test]
    fn test_slow_hopping_mode() {
        let ctrl = FrequencyHoppingController::new(
            make_hop_set(8),
            default_timing(),
            HoppingMode::Slow { symbols_per_hop: 4 },
            42,
        );
        let (val, is_slow) = ctrl.symbols_per_hop_or_hops_per_symbol();
        assert!(is_slow);
        assert_eq!(val, 4);
    }

    #[test]
    fn test_fast_hopping_mode() {
        let ctrl = FrequencyHoppingController::new(
            make_hop_set(8),
            default_timing(),
            HoppingMode::Fast { hops_per_symbol: 3 },
            42,
        );
        let (val, is_slow) = ctrl.symbols_per_hop_or_hops_per_symbol();
        assert!(!is_slow);
        assert_eq!(val, 3);
    }

    // ------------------------------------------------------------------
    // 7. Preamble synchronization
    // ------------------------------------------------------------------

    #[test]
    fn test_preamble_detection_perfect_match() {
        let ctrl = make_controller(8, 42);
        let preamble = ctrl.preamble_pattern().to_vec();
        // Observed hops = [junk, preamble, junk]
        let mut observed = vec![7, 7, 7];
        observed.extend_from_slice(&preamble);
        observed.extend_from_slice(&[7, 7]);

        let result = ctrl.detect_preamble(&observed);
        assert!(result.locked);
        assert_eq!(result.offset, 3);
        assert!((result.correlation - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_preamble_detection_no_match() {
        let ctrl = make_controller(16, 42);
        // Create observed sequence that is very different from preamble
        let preamble = ctrl.preamble_pattern().to_vec();
        // Make a sequence of values that are all different from each preamble element
        let mut observed = Vec::new();
        for _ in 0..10 {
            // Find a value not in the preamble
            let mut val = 15;
            for candidate in (0..16).rev() {
                if !preamble.contains(&candidate) {
                    val = candidate;
                    break;
                }
            }
            observed.push(val);
        }

        let result = ctrl.detect_preamble(&observed);
        assert!(!result.locked);
        assert!(result.correlation < 0.75);
    }

    #[test]
    fn test_set_custom_preamble() {
        let mut ctrl = make_controller(8, 42);
        ctrl.set_preamble_pattern(vec![0, 7, 0, 7]);
        assert_eq!(ctrl.preamble_pattern(), &[0, 7, 0, 7]);

        let freqs = ctrl.preamble_frequencies();
        assert_eq!(freqs.len(), 4);
        assert_eq!(freqs[0], 900.0e6);
        assert_eq!(freqs[1], 907.0e6);
    }

    // ------------------------------------------------------------------
    // 8. Processing gain and anti-jam
    // ------------------------------------------------------------------

    #[test]
    fn test_processing_gain() {
        let pg = FrequencyHoppingController::processing_gain_db(8);
        assert!((pg - 9.0309).abs() < 0.01);

        let pg = FrequencyHoppingController::processing_gain_db(100);
        assert!((pg - 20.0).abs() < 0.01);
    }

    #[test]
    fn test_anti_jam_margin() {
        let ctrl = make_controller(100, 42);
        // PG = 20 dB, losses = 3 dB, SNR = 10 dB => margin = 20 - 3 - 10 = 7 dB
        let margin = ctrl.anti_jam_margin_db(3.0, 10.0);
        assert!((margin - 7.0).abs() < 0.01);
    }

    #[test]
    fn test_jammer_fraction() {
        let ctrl = make_controller(100, 42);
        // JSR = 0 dB (equal power) => fraction = 1/100 = 0.01
        let frac = ctrl.jammer_fraction_required(0.0);
        assert!((frac - 0.01).abs() < 0.001);
        // JSR = 20 dB => fraction = 100/100 = 1.0 (capped)
        let frac = ctrl.jammer_fraction_required(20.0);
        assert!((frac - 1.0).abs() < 0.001);
    }

    // ------------------------------------------------------------------
    // 9. Hop set optimization
    // ------------------------------------------------------------------

    #[test]
    fn test_optimize_min_hop_distance() {
        let mut ctrl = make_controller(8, 42);
        let before = ctrl.count_min_distance_violations(2.0e6);
        let swaps = ctrl.optimize_min_hop_distance(2.0e6);
        let after = ctrl.count_min_distance_violations(2.0e6);
        // Optimization should reduce violations (or at least not increase them)
        assert!(after <= before, "Violations increased: {} -> {}", before, after);
        // Some swaps should have occurred if there were violations
        if before > 0 {
            assert!(swaps > 0);
        }
    }

    // ------------------------------------------------------------------
    // 10. IQ signal generation
    // ------------------------------------------------------------------

    #[test]
    fn test_generate_hopped_iq_length() {
        let mut ctrl = make_controller(4, 42);
        let sr = 10_000.0;
        let iq = ctrl.generate_hopped_iq(3, sr, 902.0e6);
        let expected_len = 3 * (ctrl.dwell_samples(sr) + ctrl.guard_samples(sr));
        assert_eq!(iq.len(), expected_len);
    }

    #[test]
    fn test_generate_hopped_iq_guard_is_zero() {
        let mut ctrl = make_controller(4, 42);
        let sr = 10_000.0;
        let iq = ctrl.generate_hopped_iq(1, sr, 902.0e6);
        let dwell_n = ctrl.dwell_samples(sr);
        let guard_n = ctrl.guard_samples(sr);
        // Guard samples should be zeros
        for i in dwell_n..(dwell_n + guard_n) {
            assert_eq!(iq[i], (0.0, 0.0), "Guard sample {} not zero", i);
        }
    }

    #[test]
    fn test_generate_hopped_iq_unit_amplitude() {
        let mut ctrl = make_controller(4, 42);
        let sr = 10_000.0;
        let iq = ctrl.generate_hopped_iq(2, sr, 902.0e6);
        let dwell_n = ctrl.dwell_samples(sr);
        // Dwell samples should have unit amplitude
        for i in 0..dwell_n {
            let (re, im) = iq[i];
            let mag = (re * re + im * im).sqrt();
            assert!(
                (mag - 1.0).abs() < 1e-10,
                "Sample {} magnitude {} != 1.0",
                i,
                mag
            );
        }
    }

    #[test]
    fn test_apply_hopping_to_baseband() {
        let mut ctrl = make_controller(4, 42);
        let sr = 10_000.0;
        let dwell_n = ctrl.dwell_samples(sr);
        // Create a baseband signal of all ones (DC)
        let baseband: Vec<(f64, f64)> = vec![(1.0, 0.0); dwell_n * 2];

        let hopped = ctrl.apply_hopping(&baseband, sr, 902.0e6);
        // Output should be longer than input (guard intervals added)
        assert!(hopped.len() >= baseband.len());
        // Dwell samples should have unit amplitude (baseband=1, mixing preserves amplitude)
        for i in 0..dwell_n {
            let (re, im) = hopped[i];
            let mag = (re * re + im * im).sqrt();
            assert!(
                (mag - 1.0).abs() < 1e-10,
                "Sample {} magnitude {} != 1.0",
                i,
                mag
            );
        }
    }

    #[test]
    fn test_instantaneous_frequency_estimation() {
        // Generate a pure tone and verify frequency estimation
        let sr = 10_000.0;
        let freq = 500.0; // 500 Hz
        let n = 100;
        let iq: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                let phase = 2.0 * PI * freq * t;
                (phase.cos(), phase.sin())
            })
            .collect();

        let est_freqs = FrequencyHoppingController::estimate_instantaneous_frequency(&iq, sr);
        assert_eq!(est_freqs.len(), n - 1);

        // All estimated frequencies should be close to 500 Hz
        for (i, &f) in est_freqs.iter().enumerate() {
            assert!(
                (f - freq).abs() < 1.0,
                "Sample {}: estimated {} Hz, expected {} Hz",
                i,
                f,
                freq
            );
        }
    }

    #[test]
    fn test_peek_does_not_advance() {
        let mut ctrl = make_controller(8, 42);
        let peeked = ctrl.peek_hop_index();
        let peeked2 = ctrl.peek_hop_index();
        assert_eq!(peeked, peeked2);
        assert_eq!(ctrl.current_hop_count(), 0);
        // Now advance and check it changes
        let _ = ctrl.next_hop_frequency();
        assert_eq!(ctrl.current_hop_count(), 1);
    }
}

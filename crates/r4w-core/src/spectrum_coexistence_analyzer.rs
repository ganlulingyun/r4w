//! Spectrum Coexistence Analysis for Cognitive Radio Dynamic Spectrum Access
//!
//! Provides channel-level spectrum occupancy analysis, occupancy tracking over
//! time, and spectrum opportunity discovery for cognitive radio systems. The
//! analyzer splits a wideband IQ capture into channels via DFT, compares
//! per-channel power to a detection threshold, and reports which channels are
//! occupied, the available bandwidth, and interference levels.
//!
//! Companion modules: [`crate::energy_detector`], [`crate::cognitive_engine`],
//! [`crate::spectrum_sensor`].
//!
//! # Example
//!
//! ```rust
//! use r4w_core::spectrum_coexistence_analyzer::{
//!     SpectrumCoexistenceAnalyzer, OccupancyTracker, spectrum_opportunity,
//! };
//!
//! // Create an analyzer: 256-point FFT, 8 channels, 1 MHz sample rate
//! let mut analyzer = SpectrumCoexistenceAnalyzer::new(256, 8, 1_000_000.0);
//! analyzer.set_detection_threshold(-20.0);
//!
//! // Feed low-power noise — expect no occupied channels
//! let noise: Vec<(f64, f64)> = (0..256).map(|_| (0.001, 0.001)).collect();
//! let report = analyzer.analyze(&noise);
//! assert!(report.occupancy_fraction < 0.5);
//!
//! // Track occupancy over time
//! let mut tracker = OccupancyTracker::new(8, 10);
//! tracker.update(&report.occupied_channels);
//! assert_eq!(tracker.most_available(), 0);
//!
//! // Find a contiguous block of 3 idle channels
//! let duty = (0..8).map(|ch| tracker.duty_cycle(ch)).collect::<Vec<_>>();
//! let slot = spectrum_opportunity(&duty, 3);
//! assert!(slot.is_some());
//! ```

use std::collections::VecDeque;
use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// CoexistenceReport
// ---------------------------------------------------------------------------

/// Result of a single spectrum coexistence analysis pass.
#[derive(Debug, Clone)]
pub struct CoexistenceReport {
    /// Per-channel occupancy flag (`true` = occupied).
    pub occupied_channels: Vec<bool>,
    /// Fraction of channels that are occupied (0.0 .. 1.0).
    pub occupancy_fraction: f64,
    /// Per-channel interference level in dB above the detection threshold.
    /// Negative values mean the channel is below the threshold.
    pub interference_level_db: Vec<f64>,
    /// Total bandwidth in Hz that is currently unoccupied.
    pub available_bandwidth_hz: f64,
    /// Per-channel power in dB (10 * log10 of mean linear power per channel).
    pub channel_powers_db: Vec<f64>,
}

// ---------------------------------------------------------------------------
// SpectrumCoexistenceAnalyzer
// ---------------------------------------------------------------------------

/// Wideband spectrum coexistence analyzer.
///
/// Splits the input bandwidth into `num_channels` equal sub-bands, computes
/// per-channel power via DFT, and decides occupancy by comparing against a
/// configurable threshold.
#[derive(Debug, Clone)]
pub struct SpectrumCoexistenceAnalyzer {
    fft_size: usize,
    num_channels: usize,
    sample_rate: f64,
    /// Detection threshold in dB (power above this is "occupied").
    threshold_db: f64,
}

impl SpectrumCoexistenceAnalyzer {
    /// Create a new analyzer.
    ///
    /// * `fft_size` — DFT length (number of frequency bins). Clamped to at
    ///   least `num_channels`.
    /// * `num_channels` — number of equal-width sub-band channels.
    /// * `sample_rate` — sampling rate in Hz (determines bandwidth mapping).
    pub fn new(fft_size: usize, num_channels: usize, sample_rate: f64) -> Self {
        let num_channels = num_channels.max(1);
        let fft_size = fft_size.max(num_channels);
        Self {
            fft_size,
            num_channels,
            sample_rate,
            threshold_db: -30.0, // sensible default
        }
    }

    /// Set the detection threshold in dB.
    ///
    /// Channels whose mean power (in dB) equals or exceeds this value are
    /// marked as occupied.
    pub fn set_detection_threshold(&mut self, threshold_db: f64) {
        self.threshold_db = threshold_db;
    }

    /// Analyze a block of complex IQ samples and produce a coexistence report.
    ///
    /// The input is zero-padded or truncated to `fft_size` samples.
    pub fn analyze(&mut self, samples: &[(f64, f64)]) -> CoexistenceReport {
        let spectrum = power_spectrum(samples, self.fft_size);
        let bins_per_channel = self.fft_size / self.num_channels;
        let channel_bw = self.sample_rate / self.num_channels as f64;

        let mut channel_powers_db = Vec::with_capacity(self.num_channels);
        let mut occupied_channels = Vec::with_capacity(self.num_channels);
        let mut interference_level_db = Vec::with_capacity(self.num_channels);

        for ch in 0..self.num_channels {
            let start = ch * bins_per_channel;
            let end = if ch == self.num_channels - 1 {
                self.fft_size
            } else {
                start + bins_per_channel
            };
            let slice = &spectrum[start..end];
            let mean_power = slice.iter().copied().sum::<f64>() / slice.len() as f64;
            let power_db = linear_to_db(mean_power);

            channel_powers_db.push(power_db);
            let occupied = power_db >= self.threshold_db;
            occupied_channels.push(occupied);
            interference_level_db.push(power_db - self.threshold_db);
        }

        let num_occupied = occupied_channels.iter().filter(|&&o| o).count();
        let occupancy_fraction = num_occupied as f64 / self.num_channels as f64;
        let available_bandwidth_hz =
            (self.num_channels - num_occupied) as f64 * channel_bw;

        CoexistenceReport {
            occupied_channels,
            occupancy_fraction,
            interference_level_db,
            available_bandwidth_hz,
            channel_powers_db,
        }
    }
}

// ---------------------------------------------------------------------------
// ChannelMap
// ---------------------------------------------------------------------------

/// Maps logical channel indices to RF center frequencies and tracks
/// per-channel occupancy.
#[derive(Debug, Clone)]
pub struct ChannelMap {
    center_freq_hz: f64,
    channel_bandwidth_hz: f64,
    num_channels: usize,
    occupied: Vec<bool>,
}

impl ChannelMap {
    /// Create a channel map.
    ///
    /// Channels are laid out symmetrically around `center_freq_hz`:
    ///
    /// ```text
    /// ch0    ch1    ch2   ...   ch(N-1)
    /// |------|------|------|...|------|
    ///             center
    /// ```
    ///
    /// * `center_freq_hz` — center frequency of the overall band.
    /// * `channel_bandwidth_hz` — bandwidth of each channel.
    /// * `num_channels` — number of channels.
    pub fn new(center_freq_hz: f64, channel_bandwidth_hz: f64, num_channels: usize) -> Self {
        let num_channels = num_channels.max(1);
        Self {
            center_freq_hz,
            channel_bandwidth_hz,
            num_channels,
            occupied: vec![false; num_channels],
        }
    }

    /// Return the center frequency of each channel in Hz.
    ///
    /// Channel 0 starts at the lowest frequency edge; the channels span
    /// `num_channels * channel_bandwidth_hz` centered on `center_freq_hz`.
    pub fn channel_frequencies(&self) -> Vec<f64> {
        let total_bw = self.channel_bandwidth_hz * self.num_channels as f64;
        let start = self.center_freq_hz - total_bw / 2.0 + self.channel_bandwidth_hz / 2.0;
        (0..self.num_channels)
            .map(|i| start + i as f64 * self.channel_bandwidth_hz)
            .collect()
    }

    /// Query whether a channel is currently marked as occupied.
    ///
    /// Returns `false` for out-of-range indices.
    pub fn is_occupied(&self, channel: usize) -> bool {
        self.occupied.get(channel).copied().unwrap_or(false)
    }

    /// Update the occupancy vector from an analysis report or external source.
    pub fn set_occupancy(&mut self, occupied: &[bool]) {
        for (dst, src) in self.occupied.iter_mut().zip(occupied.iter()) {
            *dst = *src;
        }
    }
}

// ---------------------------------------------------------------------------
// OccupancyTracker
// ---------------------------------------------------------------------------

/// Tracks channel occupancy over a sliding window of observations, computing
/// per-channel duty cycle (fraction of time occupied).
#[derive(Debug, Clone)]
pub struct OccupancyTracker {
    num_channels: usize,
    /// Ring buffer of observations per channel.  Each inner VecDeque stores
    /// the most recent `averaging_window` boolean observations.
    history: Vec<VecDeque<bool>>,
    averaging_window: usize,
}

impl OccupancyTracker {
    /// Create a new tracker.
    ///
    /// * `num_channels` — number of channels to track.
    /// * `averaging_window` — how many recent observations to keep.
    pub fn new(num_channels: usize, averaging_window: usize) -> Self {
        let num_channels = num_channels.max(1);
        let averaging_window = averaging_window.max(1);
        Self {
            num_channels,
            history: (0..num_channels)
                .map(|_| VecDeque::with_capacity(averaging_window))
                .collect(),
            averaging_window,
        }
    }

    /// Feed one observation vector.  `occupied[ch]` is `true` if channel `ch`
    /// was occupied during this observation period.
    pub fn update(&mut self, occupied: &[bool]) {
        for (ch, deque) in self.history.iter_mut().enumerate() {
            let val = occupied.get(ch).copied().unwrap_or(false);
            if deque.len() == self.averaging_window {
                deque.pop_front();
            }
            deque.push_back(val);
        }
    }

    /// Return the duty cycle (fraction of observations that were occupied) for
    /// a given channel.  Returns 0.0 for out-of-range channels or if no
    /// observations have been recorded.
    pub fn duty_cycle(&self, channel: usize) -> f64 {
        match self.history.get(channel) {
            Some(deque) if !deque.is_empty() => {
                let occupied_count = deque.iter().filter(|&&v| v).count();
                occupied_count as f64 / deque.len() as f64
            }
            _ => 0.0,
        }
    }

    /// Return the index of the channel with the lowest duty cycle (most
    /// available).  Ties are broken in favor of the lowest channel index.
    pub fn most_available(&self) -> usize {
        let mut best_ch = 0;
        let mut best_duty = f64::INFINITY;
        for ch in 0..self.num_channels {
            let d = self.duty_cycle(ch);
            if d < best_duty {
                best_duty = d;
                best_ch = ch;
            }
        }
        best_ch
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Find the first contiguous block of `required_bandwidth` idle channels.
///
/// A channel is considered idle when its occupancy (duty cycle) is below 0.5.
/// Returns the starting channel index of the first such block, or `None` if
/// no block of sufficient width exists.
pub fn spectrum_opportunity(occupancy: &[f64], required_bandwidth: usize) -> Option<usize> {
    if required_bandwidth == 0 || occupancy.is_empty() {
        return None;
    }
    let idle_threshold = 0.5;
    let mut run_start: Option<usize> = None;
    let mut run_len: usize = 0;

    for (i, &occ) in occupancy.iter().enumerate() {
        if occ < idle_threshold {
            if run_start.is_none() {
                run_start = Some(i);
                run_len = 0;
            }
            run_len += 1;
            if run_len >= required_bandwidth {
                return run_start;
            }
        } else {
            run_start = None;
            run_len = 0;
        }
    }
    None
}

/// Compute the interference temperature per channel.
///
/// For each channel, returns the excess power in dB above the noise floor:
///   `channel_power_db - noise_floor_db`.
///
/// A positive value indicates interference or signal presence; a value near
/// zero means the channel is at the thermal noise floor.
pub fn interference_temperature(channel_powers: &[f64], noise_floor_db: f64) -> Vec<f64> {
    channel_powers
        .iter()
        .map(|&p| p - noise_floor_db)
        .collect()
}

// ---------------------------------------------------------------------------
// Factory functions
// ---------------------------------------------------------------------------

/// Factory: create an analyzer configured for Wi-Fi 2.4 GHz coexistence.
///
/// Models the 2.4 GHz ISM band as 13 overlapping 20 MHz channels (per
/// IEEE 802.11b/g/n channel plan).  The FFT size is chosen to give at
/// least one bin per channel.
pub fn wifi_coexistence(sample_rate: f64) -> SpectrumCoexistenceAnalyzer {
    let num_channels = 13;
    // FFT size: at least num_channels, preferably a power-of-two that gives
    // reasonable resolution.
    let min_fft = (sample_rate / 20e6).ceil() as usize; // ~1 bin per 20 MHz
    let fft_size = next_power_of_two(min_fft.max(num_channels));
    let mut analyzer = SpectrumCoexistenceAnalyzer::new(fft_size, num_channels, sample_rate);
    analyzer.set_detection_threshold(-30.0);
    analyzer
}

/// Factory: create an analyzer configured for LTE coexistence.
///
/// Models 10 MHz LTE channels.  Typical deployment: a handful of carriers
/// in a licensed or shared band.  FFT size is chosen similarly.
pub fn lte_coexistence(sample_rate: f64) -> SpectrumCoexistenceAnalyzer {
    let num_channels = 10;
    let min_fft = (sample_rate / 10e6).ceil() as usize;
    let fft_size = next_power_of_two(min_fft.max(num_channels));
    let mut analyzer = SpectrumCoexistenceAnalyzer::new(fft_size, num_channels, sample_rate);
    analyzer.set_detection_threshold(-25.0);
    analyzer
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// O(N^2) DFT producing per-bin normalised power |X[k]|^2 / N^2.
///
/// Matches the convention used in [`crate::energy_detector`].
fn power_spectrum(samples: &[(f64, f64)], fft_size: usize) -> Vec<f64> {
    let n = fft_size;
    let mut spectrum = vec![0.0f64; n];

    for k in 0..n {
        let mut re_sum = 0.0f64;
        let mut im_sum = 0.0f64;
        for (idx, &(si, sq)) in samples.iter().enumerate().take(n) {
            let angle = -2.0 * PI * (k as f64) * (idx as f64) / (n as f64);
            let (sin_a, cos_a) = angle.sin_cos();
            re_sum += si * cos_a - sq * sin_a;
            im_sum += si * sin_a + sq * cos_a;
        }
        spectrum[k] = (re_sum * re_sum + im_sum * im_sum) / (n as f64 * n as f64);
    }

    spectrum
}

/// Convert linear power to dB.  Returns -200.0 for zero/negative input.
fn linear_to_db(p: f64) -> f64 {
    if p <= 0.0 {
        -200.0
    } else {
        10.0 * p.log10()
    }
}

/// Smallest power of two >= n.
fn next_power_of_two(n: usize) -> usize {
    if n == 0 {
        return 1;
    }
    1_usize << (usize::BITS - (n - 1).leading_zeros())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic LCG pseudo-random number generator for tests.
    struct TestRng(u64);
    impl TestRng {
        fn new(seed: u64) -> Self {
            Self(seed)
        }
        fn next_f64(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (self.0 >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
        }
    }

    /// Low-power broadband noise.
    fn noise_samples(n: usize, amplitude: f64) -> Vec<(f64, f64)> {
        let mut rng = TestRng::new(42);
        (0..n)
            .map(|_| (rng.next_f64() * amplitude, rng.next_f64() * amplitude))
            .collect()
    }

    /// Strong single-tone signal at normalised frequency `freq_norm`.
    fn tone_samples(n: usize, amplitude: f64, freq_norm: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let phase = 2.0 * PI * freq_norm * i as f64;
                (phase.cos() * amplitude, phase.sin() * amplitude)
            })
            .collect()
    }

    // ------------------------------------------------------------------
    // 1. Construction
    // ------------------------------------------------------------------
    #[test]
    fn test_construction() {
        let a = SpectrumCoexistenceAnalyzer::new(256, 8, 1_000_000.0);
        assert_eq!(a.fft_size, 256);
        assert_eq!(a.num_channels, 8);
        assert!((a.sample_rate - 1_000_000.0).abs() < 1e-6);

        // num_channels clamped to 1
        let b = SpectrumCoexistenceAnalyzer::new(64, 0, 48000.0);
        assert_eq!(b.num_channels, 1);

        // fft_size clamped to at least num_channels
        let c = SpectrumCoexistenceAnalyzer::new(2, 16, 48000.0);
        assert_eq!(c.fft_size, 16);
    }

    // ------------------------------------------------------------------
    // 2. Analysis detects occupied channels when a strong tone is present
    // ------------------------------------------------------------------
    #[test]
    fn test_analysis_detects_occupied_channels() {
        let fft_size = 128;
        let num_channels = 8;
        let sample_rate = 128_000.0;
        let mut analyzer =
            SpectrumCoexistenceAnalyzer::new(fft_size, num_channels, sample_rate);
        // Set a threshold low enough that a strong tone will exceed it
        analyzer.set_detection_threshold(-60.0);

        // Tone at normalised frequency 0.125 — that is bin 16 of 128, which
        // falls in channel 1 (bins 16..31).
        let signal = tone_samples(fft_size, 1.0, 0.125);
        let report = analyzer.analyze(&signal);

        assert!(
            report.occupied_channels.iter().any(|&o| o),
            "At least one channel should be occupied with a strong tone"
        );
        assert!(
            report.occupancy_fraction > 0.0,
            "Occupancy fraction should be > 0"
        );
    }

    // ------------------------------------------------------------------
    // 3. All-noise produces no occupied channels
    // ------------------------------------------------------------------
    #[test]
    fn test_all_noise_no_occupied() {
        let fft_size = 128;
        let num_channels = 8;
        let sample_rate = 128_000.0;
        let mut analyzer =
            SpectrumCoexistenceAnalyzer::new(fft_size, num_channels, sample_rate);
        // Set threshold high enough that tiny noise will not trigger
        analyzer.set_detection_threshold(0.0);

        let noise = noise_samples(fft_size, 0.0001);
        let report = analyzer.analyze(&noise);

        assert!(
            !report.occupied_channels.iter().any(|&o| o),
            "No channel should be occupied for low-level noise"
        );
        assert!(
            (report.occupancy_fraction - 0.0).abs() < 1e-12,
            "Occupancy fraction should be 0.0"
        );
        assert!(
            (report.available_bandwidth_hz - sample_rate).abs() < 1e-6,
            "All bandwidth should be available"
        );
    }

    // ------------------------------------------------------------------
    // 4. Channel map frequency computation
    // ------------------------------------------------------------------
    #[test]
    fn test_channel_map_frequencies() {
        // 5 channels, each 20 MHz wide, centered on 2.44 GHz
        let map = ChannelMap::new(2.44e9, 20e6, 5);
        let freqs = map.channel_frequencies();

        assert_eq!(freqs.len(), 5);

        // Total bandwidth = 5 * 20 MHz = 100 MHz
        // Lowest channel center = 2.44e9 - 50e6 + 10e6 = 2.40e9
        let expected_start = 2.44e9 - 100e6 / 2.0 + 20e6 / 2.0;
        assert!(
            (freqs[0] - expected_start).abs() < 1.0,
            "First channel center: expected {}, got {}",
            expected_start,
            freqs[0]
        );

        // Spacing should be 20 MHz
        for i in 1..freqs.len() {
            let spacing = freqs[i] - freqs[i - 1];
            assert!(
                (spacing - 20e6).abs() < 1.0,
                "Channel spacing should be 20 MHz, got {}",
                spacing
            );
        }

        // Verify is_occupied defaults to false
        for ch in 0..5 {
            assert!(!map.is_occupied(ch));
        }
        assert!(!map.is_occupied(999)); // out of range
    }

    // ------------------------------------------------------------------
    // 5. Occupancy tracker duty cycle
    // ------------------------------------------------------------------
    #[test]
    fn test_occupancy_tracker_duty_cycle() {
        let mut tracker = OccupancyTracker::new(4, 10);

        // Feed 10 observations: channel 0 is always occupied, channel 1 is
        // occupied half the time, channel 2 never, channel 3 always.
        for i in 0..10 {
            let obs = vec![true, i % 2 == 0, false, true];
            tracker.update(&obs);
        }

        assert!((tracker.duty_cycle(0) - 1.0).abs() < 1e-12, "ch0 always on");
        assert!(
            (tracker.duty_cycle(1) - 0.5).abs() < 1e-12,
            "ch1 half: got {}",
            tracker.duty_cycle(1)
        );
        assert!((tracker.duty_cycle(2) - 0.0).abs() < 1e-12, "ch2 never on");
        assert!((tracker.duty_cycle(3) - 1.0).abs() < 1e-12, "ch3 always on");

        // Out-of-range channel
        assert!((tracker.duty_cycle(99) - 0.0).abs() < 1e-12);
    }

    // ------------------------------------------------------------------
    // 6. Most available channel selection
    // ------------------------------------------------------------------
    #[test]
    fn test_most_available_channel() {
        let mut tracker = OccupancyTracker::new(4, 10);

        // ch0: 80% occupied, ch1: 60%, ch2: 10%, ch3: 40%
        for i in 0..10 {
            tracker.update(&[i < 8, i < 6, i < 1, i < 4]);
        }

        assert_eq!(
            tracker.most_available(),
            2,
            "Channel 2 has lowest occupancy"
        );
    }

    // ------------------------------------------------------------------
    // 7. Spectrum opportunity finding
    // ------------------------------------------------------------------
    #[test]
    fn test_spectrum_opportunity() {
        // Occupancy: [0.8, 0.9, 0.1, 0.2, 0.05, 0.7, 0.0, 0.0]
        let occ = [0.8, 0.9, 0.1, 0.2, 0.05, 0.7, 0.0, 0.0];

        // Need 3 contiguous idle channels => channels 2,3,4
        assert_eq!(spectrum_opportunity(&occ, 3), Some(2));

        // Need 4 contiguous idle => no block of 4 exists
        assert_eq!(spectrum_opportunity(&occ, 4), None);

        // Need 2 contiguous idle => first pair is channels 2,3
        assert_eq!(spectrum_opportunity(&occ, 2), Some(2));

        // Need 1 => first idle channel is index 2
        assert_eq!(spectrum_opportunity(&occ, 1), Some(2));

        // Edge: required_bandwidth = 0 => None
        assert_eq!(spectrum_opportunity(&occ, 0), None);

        // Edge: empty occupancy
        assert_eq!(spectrum_opportunity(&[], 1), None);
    }

    // ------------------------------------------------------------------
    // 8. Interference temperature computation
    // ------------------------------------------------------------------
    #[test]
    fn test_interference_temperature() {
        let powers = vec![-40.0, -50.0, -30.0, -60.0];
        let noise_floor = -50.0;
        let temps = interference_temperature(&powers, noise_floor);

        assert_eq!(temps.len(), 4);
        assert!((temps[0] - 10.0).abs() < 1e-12); // -40 - (-50) = 10
        assert!((temps[1] - 0.0).abs() < 1e-12);  // at noise floor
        assert!((temps[2] - 20.0).abs() < 1e-12);  // -30 - (-50) = 20
        assert!((temps[3] - (-10.0)).abs() < 1e-12); // below noise floor
    }

    // ------------------------------------------------------------------
    // 9. WiFi factory
    // ------------------------------------------------------------------
    #[test]
    fn test_wifi_factory() {
        let analyzer = wifi_coexistence(260e6);
        assert_eq!(analyzer.num_channels, 13);
        assert!(
            analyzer.fft_size >= 13,
            "FFT size should be >= 13, got {}",
            analyzer.fft_size
        );
        assert!(
            analyzer.fft_size.is_power_of_two(),
            "FFT size should be power of two, got {}",
            analyzer.fft_size
        );
        assert!(
            (analyzer.threshold_db - (-30.0)).abs() < 1e-12,
            "WiFi threshold should be -30 dB"
        );
    }

    // ------------------------------------------------------------------
    // 10. LTE factory
    // ------------------------------------------------------------------
    #[test]
    fn test_lte_factory() {
        let analyzer = lte_coexistence(100e6);
        assert_eq!(analyzer.num_channels, 10);
        assert!(
            analyzer.fft_size >= 10,
            "FFT size should be >= 10, got {}",
            analyzer.fft_size
        );
        assert!(
            analyzer.fft_size.is_power_of_two(),
            "FFT size should be power of two, got {}",
            analyzer.fft_size
        );
        assert!(
            (analyzer.threshold_db - (-25.0)).abs() < 1e-12,
            "LTE threshold should be -25 dB"
        );
    }
}

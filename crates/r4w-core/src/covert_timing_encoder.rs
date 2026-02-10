//! Covert timing channel encoder/decoder.
//!
//! Encodes data into inter-packet timing gaps for covert communication below
//! noise-floor detection thresholds. The sender modulates the inter-arrival time
//! of otherwise-normal packets: a "0" bit is conveyed by a timing gap close to
//! the base interval, while a "1" bit uses a slightly different gap. An observer
//! sees only ordinary traffic whose timing jitter appears natural.
//!
//! # Features
//!
//! * **Binary or M-ary modulation** — choose 2 timing levels (1 bit/symbol) or
//!   up to 256 levels for higher throughput.
//! * **Jitter injection** — configurable pseudo-random jitter masks the covert
//!   modulation.
//! * **Preamble synchronization** — a known timing pattern lets the receiver
//!   lock onto the covert stream.
//! * **Cover traffic mixing** — blend covert timing into a background traffic
//!   pattern to reduce detectability.
//! * **Analytic helpers** — channel capacity, detection probability, and BER
//!   estimates.
//!
//! # Example
//!
//! ```
//! use r4w_core::covert_timing_encoder::{CovertTimingEncoder, TimingConfig};
//!
//! let config = TimingConfig {
//!     base_interval_us: 10_000.0,
//!     bit_0_offset_us: -500.0,
//!     bit_1_offset_us: 500.0,
//!     jitter_std_us: 50.0,
//! };
//!
//! let encoder = CovertTimingEncoder::new(config.clone(), None);
//!
//! // Encode a short bit sequence.
//! let bits = vec![true, false, true, true, false];
//! let encoded = encoder.encode_timing(&bits);
//! assert_eq!(encoded.intervals_us.len(), bits.len());
//!
//! // Decode (jitter is small relative to separation, so round-trip succeeds).
//! let decoded = encoder.decode_timing(&encoded);
//! assert_eq!(decoded, bits);
//! ```

use std::f64::consts::{E, PI};

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Parameters that define the covert timing channel.
#[derive(Debug, Clone)]
pub struct TimingConfig {
    /// Nominal inter-packet interval in microseconds.
    pub base_interval_us: f64,
    /// Offset applied for a "0" bit (added to base interval).
    pub bit_0_offset_us: f64,
    /// Offset applied for a "1" bit (added to base interval).
    pub bit_1_offset_us: f64,
    /// Standard deviation of the injected timing jitter (microseconds).
    pub jitter_std_us: f64,
}

impl Default for TimingConfig {
    fn default() -> Self {
        Self {
            base_interval_us: 10_000.0,
            bit_0_offset_us: -500.0,
            bit_1_offset_us: 500.0,
            jitter_std_us: 50.0,
        }
    }
}

// ---------------------------------------------------------------------------
// M-ary configuration
// ---------------------------------------------------------------------------

/// M-ary modulation configuration.
///
/// When `None` is supplied to [`CovertTimingEncoder::new`] the encoder defaults
/// to binary (M = 2).
#[derive(Debug, Clone)]
pub struct MaryConfig {
    /// Number of timing levels (must be a power of two, 2..=256).
    pub m: usize,
    /// Per-level offsets from the base interval (length must equal `m`).
    pub level_offsets_us: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Encoded output
// ---------------------------------------------------------------------------

/// A sequence of inter-arrival times produced by the encoder.
#[derive(Debug, Clone)]
pub struct EncodedTiming {
    /// Inter-arrival times in microseconds.
    pub intervals_us: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Timing channel capacity
// ---------------------------------------------------------------------------

/// Covert timing channel capacity estimate.
#[derive(Debug, Clone)]
pub struct TimingChannel {
    /// Estimated capacity in bits per timing interval.
    pub bits_per_interval: f64,
    /// Timing resolution used in the estimate (microseconds).
    pub resolution_us: f64,
    /// Jitter standard deviation used in the estimate (microseconds).
    pub jitter_us: f64,
}

impl TimingChannel {
    /// Estimate the channel capacity for a binary timing channel.
    ///
    /// Uses a simplified Gaussian-channel capacity formula:
    /// C = 0.5 * log2(1 + (delta / sigma)^2)
    /// where delta is half the distance between the two timing levels and sigma
    /// is the jitter standard deviation.
    pub fn estimate(config: &TimingConfig) -> Self {
        let delta = (config.bit_1_offset_us - config.bit_0_offset_us).abs() / 2.0;
        let sigma = config.jitter_std_us.max(1e-12);
        let snr = (delta / sigma).powi(2);
        let bits_per_interval = 0.5 * (1.0 + snr).log2();
        Self {
            bits_per_interval,
            resolution_us: delta * 2.0,
            jitter_us: sigma,
        }
    }

    /// Estimate the channel capacity for an M-ary timing channel.
    pub fn estimate_mary(config: &TimingConfig, mary: &MaryConfig) -> Self {
        let sigma = config.jitter_std_us.max(1e-12);
        let min_off = mary
            .level_offsets_us
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let max_off = mary
            .level_offsets_us
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let span = (max_off - min_off).abs();
        let snr = (span / (2.0 * sigma)).powi(2);
        let bits_per_interval = 0.5 * (1.0 + snr).log2();
        Self {
            bits_per_interval,
            resolution_us: span,
            jitter_us: sigma,
        }
    }
}

// ---------------------------------------------------------------------------
// Simple deterministic PRNG (xorshift64)
// ---------------------------------------------------------------------------

/// Minimal xorshift64 PRNG so we avoid external crate dependencies.
#[derive(Debug, Clone)]
struct Xorshift64 {
    state: u64,
}

impl Xorshift64 {
    fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 { 1 } else { seed },
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    /// Return a value in [0, 1).
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / ((1u64 << 53) as f64)
    }

    /// Approximate Gaussian via Box-Muller.
    fn next_gaussian(&mut self, mean: f64, std_dev: f64) -> f64 {
        let u1 = self.next_f64().max(1e-15);
        let u2 = self.next_f64();
        let z = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos();
        mean + z * std_dev
    }
}

// ---------------------------------------------------------------------------
// Preamble
// ---------------------------------------------------------------------------

/// Default preamble pattern (8 alternating bits).
const DEFAULT_PREAMBLE: [bool; 8] = [true, false, true, false, true, false, true, false];

// ---------------------------------------------------------------------------
// Encoder / Decoder
// ---------------------------------------------------------------------------

/// Covert timing encoder and decoder.
///
/// Encodes bits into inter-packet timing gaps and decodes them back using
/// threshold detection.
#[derive(Debug, Clone)]
pub struct CovertTimingEncoder {
    config: TimingConfig,
    mary: Option<MaryConfig>,
    rng_seed: u64,
}

impl CovertTimingEncoder {
    /// Create a new encoder.
    ///
    /// Pass `Some(MaryConfig { .. })` for M-ary modulation or `None` for binary.
    pub fn new(config: TimingConfig, mary: Option<MaryConfig>) -> Self {
        Self {
            config,
            mary,
            rng_seed: 0xDEAD_BEEF_CAFE_1234,
        }
    }

    /// Change the PRNG seed used for jitter injection.
    pub fn set_seed(&mut self, seed: u64) {
        self.rng_seed = seed;
    }

    // -- encoding ----------------------------------------------------------

    /// Encode a slice of bits into inter-arrival times.
    ///
    /// For binary mode each bit maps to `base + offset + jitter`.
    /// For M-ary mode, bits are grouped into chunks of `log2(M)` and each
    /// group selects a level.
    pub fn encode_timing(&self, bits: &[bool]) -> EncodedTiming {
        let mut rng = Xorshift64::new(self.rng_seed);

        let intervals = if let Some(ref mary) = self.mary {
            self.encode_mary(bits, mary, &mut rng)
        } else {
            self.encode_binary(bits, &mut rng)
        };

        EncodedTiming {
            intervals_us: intervals,
        }
    }

    fn encode_binary(&self, bits: &[bool], rng: &mut Xorshift64) -> Vec<f64> {
        bits.iter()
            .map(|&b| {
                let offset = if b {
                    self.config.bit_1_offset_us
                } else {
                    self.config.bit_0_offset_us
                };
                let jitter = rng.next_gaussian(0.0, self.config.jitter_std_us);
                self.config.base_interval_us + offset + jitter
            })
            .collect()
    }

    fn encode_mary(&self, bits: &[bool], mary: &MaryConfig, rng: &mut Xorshift64) -> Vec<f64> {
        let bits_per_symbol = (mary.m as f64).log2() as usize;
        let mut intervals = Vec::new();
        let mut i = 0;
        while i < bits.len() {
            let end = (i + bits_per_symbol).min(bits.len());
            let mut symbol: usize = 0;
            for (j, &b) in bits[i..end].iter().enumerate() {
                if b {
                    symbol |= 1 << (bits_per_symbol - 1 - j);
                }
            }
            let offset = mary.level_offsets_us[symbol.min(mary.m - 1)];
            let jitter = rng.next_gaussian(0.0, self.config.jitter_std_us);
            intervals.push(self.config.base_interval_us + offset + jitter);
            i = end;
        }
        intervals
    }

    // -- decoding ----------------------------------------------------------

    /// Decode inter-arrival times back to bits using threshold detection.
    pub fn decode_timing(&self, encoded: &EncodedTiming) -> Vec<bool> {
        if let Some(ref mary) = self.mary {
            self.decode_mary(&encoded.intervals_us, mary)
        } else {
            self.decode_binary(&encoded.intervals_us)
        }
    }

    fn decode_binary(&self, intervals: &[f64]) -> Vec<bool> {
        let threshold = self.config.base_interval_us
            + (self.config.bit_0_offset_us + self.config.bit_1_offset_us) / 2.0;
        intervals.iter().map(|&t| t >= threshold).collect()
    }

    fn decode_mary(&self, intervals: &[f64], mary: &MaryConfig) -> Vec<bool> {
        let bits_per_symbol = (mary.m as f64).log2() as usize;
        let mut bits = Vec::new();
        for &t in intervals {
            let residual = t - self.config.base_interval_us;
            // Nearest-level detection.
            let symbol = mary
                .level_offsets_us
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    let da = (residual - **a).abs();
                    let db = (residual - **b).abs();
                    da.partial_cmp(&db).unwrap()
                })
                .map(|(idx, _)| idx)
                .unwrap_or(0);
            for j in 0..bits_per_symbol {
                bits.push((symbol >> (bits_per_symbol - 1 - j)) & 1 == 1);
            }
        }
        bits
    }

    // -- preamble ----------------------------------------------------------

    /// Encode a preamble followed by data bits.
    pub fn encode_with_preamble(&self, bits: &[bool]) -> EncodedTiming {
        let mut full = DEFAULT_PREAMBLE.to_vec();
        full.extend_from_slice(bits);
        self.encode_timing(&full)
    }

    /// Detect the preamble in a stream of inter-arrival times and return the
    /// data bits that follow it.
    ///
    /// Returns `None` if no preamble is found.
    pub fn decode_with_preamble(&self, encoded: &EncodedTiming) -> Option<Vec<bool>> {
        let all_bits = self.decode_timing(encoded);
        let preamble = DEFAULT_PREAMBLE.to_vec();
        if all_bits.len() < preamble.len() {
            return None;
        }
        for start in 0..=(all_bits.len() - preamble.len()) {
            if all_bits[start..start + preamble.len()] == preamble[..] {
                return Some(all_bits[start + preamble.len()..].to_vec());
            }
        }
        None
    }

    // -- cover traffic -----------------------------------------------------

    /// Mix covert-encoded intervals with cover traffic.
    ///
    /// Cover traffic consists of intervals drawn from a Gaussian distribution
    /// centred at the base interval. The covert intervals are interleaved at
    /// positions indicated by `covert_positions` (sorted indices into the
    /// output vector).
    pub fn add_cover_traffic(
        &self,
        covert: &EncodedTiming,
        total_packets: usize,
        covert_positions: &[usize],
    ) -> EncodedTiming {
        assert!(total_packets >= covert.intervals_us.len());
        assert_eq!(covert_positions.len(), covert.intervals_us.len());

        let mut rng = Xorshift64::new(self.rng_seed.wrapping_add(0x1234));
        let mut output = Vec::with_capacity(total_packets);

        let mut ci = 0;
        for i in 0..total_packets {
            if ci < covert_positions.len() && covert_positions[ci] == i {
                output.push(covert.intervals_us[ci]);
                ci += 1;
            } else {
                let cover = rng.next_gaussian(
                    self.config.base_interval_us,
                    (self.config.bit_1_offset_us - self.config.bit_0_offset_us).abs() / 2.0,
                );
                output.push(cover);
            }
        }

        EncodedTiming {
            intervals_us: output,
        }
    }

    // -- analytic helpers --------------------------------------------------

    /// Estimate the probability that a passive observer detects the covert
    /// channel using a Kolmogorov-Smirnov-style metric.
    ///
    /// This is a simplified model: the observer compares the empirical timing
    /// distribution against a Gaussian baseline. Returns a value in \[0, 1\].
    pub fn detection_probability(&self) -> f64 {
        let separation = (self.config.bit_1_offset_us - self.config.bit_0_offset_us).abs();
        let sigma = self.config.jitter_std_us.max(1e-12);
        let x = separation / (2.0 * sigma);
        // Logistic approximation centred at x=3.
        let p_detect = 1.0 / (1.0 + E.powf(-(x - 3.0)));
        p_detect.clamp(0.0, 1.0)
    }

    /// Estimate the bit-error rate (BER) for the binary timing channel given
    /// the configured jitter.
    ///
    /// BER = Q(delta / (2 * sigma)) where delta is the separation between the
    /// two levels and sigma is the jitter standard deviation.
    pub fn bit_error_rate(&self) -> f64 {
        let separation = (self.config.bit_1_offset_us - self.config.bit_0_offset_us).abs();
        let sigma = self.config.jitter_std_us.max(1e-12);
        let x = separation / (2.0 * sigma);
        q_function(x)
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &TimingConfig {
        &self.config
    }

    /// Return a reference to the M-ary configuration, if any.
    pub fn mary_config(&self) -> Option<&MaryConfig> {
        self.mary.as_ref()
    }
}

// ---------------------------------------------------------------------------
// Q-function (tail probability of standard normal)
// ---------------------------------------------------------------------------

/// Approximation of the Q-function: Q(x) = 0.5 * erfc(x / sqrt(2)).
///
/// Uses the Abramowitz and Stegun rational approximation for erfc.
fn q_function(x: f64) -> f64 {
    if x < 0.0 {
        return 1.0 - q_function(-x);
    }
    let t = 1.0 / (1.0 + 0.3275911 * x / std::f64::consts::SQRT_2);
    let poly = t
        * (0.254829592
            + t * (-0.284496736
                + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    let erfc_val = poly * (-x * x / 2.0).exp();
    0.5 * erfc_val
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> TimingConfig {
        TimingConfig {
            base_interval_us: 10_000.0,
            bit_0_offset_us: -500.0,
            bit_1_offset_us: 500.0,
            jitter_std_us: 50.0,
        }
    }

    // 1. Basic binary encode / decode round-trip.
    #[test]
    fn test_binary_roundtrip() {
        let enc = CovertTimingEncoder::new(default_config(), None);
        let bits = vec![true, false, true, true, false, false, true];
        let encoded = enc.encode_timing(&bits);
        let decoded = enc.decode_timing(&encoded);
        assert_eq!(decoded, bits);
    }

    // 2. Empty input produces empty output.
    #[test]
    fn test_empty_bits() {
        let enc = CovertTimingEncoder::new(default_config(), None);
        let encoded = enc.encode_timing(&[]);
        assert!(encoded.intervals_us.is_empty());
        let decoded = enc.decode_timing(&encoded);
        assert!(decoded.is_empty());
    }

    // 3. Intervals are centred around expected values.
    #[test]
    fn test_interval_ranges() {
        let cfg = default_config();
        let enc = CovertTimingEncoder::new(cfg.clone(), None);
        let bits = vec![false; 200];
        let encoded = enc.encode_timing(&bits);

        let mean: f64 =
            encoded.intervals_us.iter().sum::<f64>() / encoded.intervals_us.len() as f64;
        let expected = cfg.base_interval_us + cfg.bit_0_offset_us;
        let tolerance = 3.0 * cfg.jitter_std_us / (200.0_f64).sqrt();
        assert!(
            (mean - expected).abs() < tolerance,
            "mean {mean} vs expected {expected}, tol {tolerance}"
        );
    }

    // 4. Jitter = 0 gives exact round-trip.
    #[test]
    fn test_zero_jitter_exact() {
        let mut cfg = default_config();
        cfg.jitter_std_us = 0.0;
        let enc = CovertTimingEncoder::new(cfg.clone(), None);
        let bits: Vec<bool> = (0..50).map(|i| i % 3 == 0).collect();
        let encoded = enc.encode_timing(&bits);

        for (i, &b) in bits.iter().enumerate() {
            let expected = cfg.base_interval_us
                + if b {
                    cfg.bit_1_offset_us
                } else {
                    cfg.bit_0_offset_us
                };
            assert!(
                (encoded.intervals_us[i] - expected).abs() < 1e-9,
                "interval[{i}] = {} vs expected {expected}",
                encoded.intervals_us[i]
            );
        }

        let decoded = enc.decode_timing(&encoded);
        assert_eq!(decoded, bits);
    }

    // 5. M-ary (4-ary) encode / decode round-trip.
    #[test]
    fn test_mary_roundtrip() {
        let cfg = TimingConfig {
            base_interval_us: 10_000.0,
            bit_0_offset_us: -1500.0,
            bit_1_offset_us: 1500.0,
            jitter_std_us: 30.0,
        };
        let mary = MaryConfig {
            m: 4,
            level_offsets_us: vec![-1500.0, -500.0, 500.0, 1500.0],
        };
        let enc = CovertTimingEncoder::new(cfg, Some(mary));
        // 8 bits -> 4 symbols of 2 bits each.
        let bits = vec![false, false, false, true, true, false, true, true];
        let encoded = enc.encode_timing(&bits);
        assert_eq!(encoded.intervals_us.len(), 4);
        let decoded = enc.decode_timing(&encoded);
        assert_eq!(decoded, bits);
    }

    // 6. Preamble encode / decode.
    #[test]
    fn test_preamble_sync() {
        let mut cfg = default_config();
        cfg.jitter_std_us = 10.0;
        let enc = CovertTimingEncoder::new(cfg, None);
        let data = vec![true, true, false, false, true];
        let encoded = enc.encode_with_preamble(&data);
        assert_eq!(encoded.intervals_us.len(), 13);
        let recovered = enc.decode_with_preamble(&encoded);
        assert_eq!(recovered, Some(data));
    }

    // 7. Cover traffic preserves covert intervals.
    #[test]
    fn test_cover_traffic() {
        let enc = CovertTimingEncoder::new(default_config(), None);
        let bits = vec![true, false, true];
        let covert = enc.encode_timing(&bits);
        let positions = vec![1, 5, 9];
        let mixed = enc.add_cover_traffic(&covert, 12, &positions);
        assert_eq!(mixed.intervals_us.len(), 12);

        for (i, &pos) in positions.iter().enumerate() {
            assert!(
                (mixed.intervals_us[pos] - covert.intervals_us[i]).abs() < 1e-12,
                "covert interval mismatch at position {pos}"
            );
        }
    }

    // 8. Detection probability is low when jitter is large.
    #[test]
    fn test_detection_probability_low() {
        let cfg = TimingConfig {
            base_interval_us: 10_000.0,
            bit_0_offset_us: -10.0,
            bit_1_offset_us: 10.0,
            jitter_std_us: 500.0,
        };
        let enc = CovertTimingEncoder::new(cfg, None);
        let p = enc.detection_probability();
        assert!(p < 0.1, "detection probability should be low, got {p}");
    }

    // 9. Detection probability is high when jitter is small.
    #[test]
    fn test_detection_probability_high() {
        let cfg = TimingConfig {
            base_interval_us: 10_000.0,
            bit_0_offset_us: -5000.0,
            bit_1_offset_us: 5000.0,
            jitter_std_us: 10.0,
        };
        let enc = CovertTimingEncoder::new(cfg, None);
        let p = enc.detection_probability();
        assert!(p > 0.9, "detection probability should be high, got {p}");
    }

    // 10. BER estimate is near zero for large separation / small jitter.
    #[test]
    fn test_ber_low() {
        let cfg = TimingConfig {
            base_interval_us: 10_000.0,
            bit_0_offset_us: -1000.0,
            bit_1_offset_us: 1000.0,
            jitter_std_us: 50.0,
        };
        let enc = CovertTimingEncoder::new(cfg, None);
        let ber = enc.bit_error_rate();
        assert!(ber < 1e-6, "BER should be very low, got {ber}");
    }

    // 11. BER is high when jitter dominates.
    #[test]
    fn test_ber_high() {
        let cfg = TimingConfig {
            base_interval_us: 10_000.0,
            bit_0_offset_us: -1.0,
            bit_1_offset_us: 1.0,
            jitter_std_us: 500.0,
        };
        let enc = CovertTimingEncoder::new(cfg, None);
        let ber = enc.bit_error_rate();
        assert!(ber > 0.4, "BER should be close to 0.5, got {ber}");
    }

    // 12. Channel capacity estimate.
    #[test]
    fn test_channel_capacity() {
        let cfg = default_config();
        let tc = TimingChannel::estimate(&cfg);
        // delta=500, sigma=50 -> SNR=100 -> C = 0.5*log2(101) ~ 3.33
        assert!(
            tc.bits_per_interval > 3.0,
            "capacity too low: {}",
            tc.bits_per_interval
        );
        assert!(
            tc.bits_per_interval < 4.0,
            "capacity too high: {}",
            tc.bits_per_interval
        );
    }

    // 13. Seed changes jitter pattern.
    #[test]
    fn test_different_seeds() {
        let cfg = default_config();
        let mut enc1 = CovertTimingEncoder::new(cfg.clone(), None);
        enc1.set_seed(1);
        let mut enc2 = CovertTimingEncoder::new(cfg, None);
        enc2.set_seed(2);

        let bits = vec![true; 20];
        let e1 = enc1.encode_timing(&bits);
        let e2 = enc2.encode_timing(&bits);
        let differ = e1
            .intervals_us
            .iter()
            .zip(e2.intervals_us.iter())
            .any(|(a, b)| (a - b).abs() > 1e-9);
        assert!(differ, "different seeds should produce different jitter");
    }
}

//! Adaptive narrowband interference removal for wideband signals.
//!
//! This module implements frequency-domain notching and eigenvalue-based detection
//! to identify and excise narrowband interferers from wideband IQ sample streams.
//! The approach performs an FFT on the input, estimates the noise floor, detects
//! spectral peaks that exceed a configurable threshold, and zeros (or interpolates)
//! the affected bins before transforming back to the time domain.
//!
//! # Algorithm Overview
//!
//! 1. Compute the power spectrum via FFT.
//! 2. Estimate the noise floor using a robust median-based estimator.
//! 3. Detect bins whose power exceeds the noise floor by the configured threshold.
//! 4. Group adjacent bins into interferer regions.
//! 5. Apply spectral masking (zeroing or interpolation) to excise the interferers.
//! 6. Inverse-FFT back to the time domain.
//!
//! # Example
//!
//! ```
//! use r4w_core::interference_excision::{ExcisionConfig, InterferenceExciser};
//!
//! let config = ExcisionConfig {
//!     fft_size: 64,
//!     detection_threshold_db: 15.0,
//!     notch_bandwidth_bins: 3,
//!     max_interferers: 4,
//! };
//! let mut exciser = InterferenceExciser::new(config);
//!
//! // Create a wideband signal (noise) with a strong CW interferer at bin 16
//! let n = 64;
//! let mut samples: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let phase = 2.0 * std::f64::consts::PI * (i as f64) * 16.0 / (n as f64);
//!         (phase.cos() * 10.0, phase.sin() * 10.0)
//!     })
//!     .collect();
//!
//! // Add some low-level noise
//! for s in samples.iter_mut() {
//!     s.0 += 0.01 * (s.0 * 37.0).sin();
//!     s.1 += 0.01 * (s.1 * 53.0).cos();
//! }
//!
//! let result = exciser.adaptive_excise(&samples);
//! assert!(result.len() == n);
//!
//! // The strong interferer should be suppressed
//! let interferers = exciser.detect_interferers(&samples);
//! assert!(!interferers.is_empty());
//! ```

use std::f64::consts::PI;

// ─── Complex arithmetic helpers ────────────────────────────────────────────

type C64 = (f64, f64);

#[inline]
fn c_add(a: C64, b: C64) -> C64 {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: C64, b: C64) -> C64 {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: C64, b: C64) -> C64 {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_scale(a: C64, s: f64) -> C64 {
    (a.0 * s, a.1 * s)
}

#[inline]
fn c_mag_sq(a: C64) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

// ─── Radix-2 DIT FFT (in-place) ───────────────────────────────────────────

fn bit_reverse(x: usize, log2n: u32) -> usize {
    let mut result = 0usize;
    let mut val = x;
    for _ in 0..log2n {
        result = (result << 1) | (val & 1);
        val >>= 1;
    }
    result
}

/// In-place radix-2 DIT FFT. `inverse` selects IFFT (with 1/N scaling).
fn fft_in_place(buf: &mut [C64], inverse: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT size must be power of two");
    let log2n = n.trailing_zeros();

    // Bit-reversal permutation
    for i in 0..n {
        let j = bit_reverse(i, log2n);
        if i < j {
            buf.swap(i, j);
        }
    }

    // Butterfly stages
    let mut size = 2usize;
    while size <= n {
        let half = size / 2;
        let sign = if inverse { 1.0 } else { -1.0 };
        let angle_step = sign * 2.0 * PI / (size as f64);
        for k in (0..n).step_by(size) {
            for j in 0..half {
                let angle = angle_step * (j as f64);
                let twiddle = (angle.cos(), angle.sin());
                let u = buf[k + j];
                let t = c_mul(twiddle, buf[k + j + half]);
                buf[k + j] = c_add(u, t);
                buf[k + j + half] = c_sub(u, t);
            }
        }
        size <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / (n as f64);
        for sample in buf.iter_mut() {
            *sample = c_scale(*sample, inv_n);
        }
    }
}

// ─── Public types ──────────────────────────────────────────────────────────

/// Classification of a detected interferer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterfererType {
    /// Continuous-wave (single-bin or very narrow) interferer.
    Cw,
    /// Wideband interferer spanning multiple bins.
    Wideband,
}

/// Information about a single detected interferer.
#[derive(Debug, Clone)]
pub struct InterfererInfo {
    /// Centre frequency bin (0-based, up to fft_size-1).
    pub frequency_bin: usize,
    /// Peak power in dB (relative to noise floor).
    pub power_db: f64,
    /// Number of bins the interferer spans.
    pub bandwidth_bins: usize,
    /// Classification.
    pub interferer_type: InterfererType,
}

/// Configuration for the interference exciser.
#[derive(Debug, Clone)]
pub struct ExcisionConfig {
    /// FFT size (must be a power of two).
    pub fft_size: usize,
    /// Detection threshold in dB above the estimated noise floor.
    pub detection_threshold_db: f64,
    /// Number of bins to notch on *each side* of a detected peak.
    pub notch_bandwidth_bins: usize,
    /// Maximum number of interferers to excise per block.
    pub max_interferers: usize,
}

impl Default for ExcisionConfig {
    fn default() -> Self {
        Self {
            fft_size: 1024,
            detection_threshold_db: 20.0,
            notch_bandwidth_bins: 2,
            max_interferers: 8,
        }
    }
}

/// Result of an excision operation, including diagnostics.
#[derive(Debug, Clone)]
pub struct ExcisionResult {
    /// The cleaned IQ samples (same length as input).
    pub samples: Vec<C64>,
    /// Interferers that were detected.
    pub interferers: Vec<InterfererInfo>,
    /// Estimated SIR in dB *before* excision.
    pub sir_before_db: f64,
    /// Estimated SIR in dB *after* excision.
    pub sir_after_db: f64,
    /// Estimated noise floor power in dB.
    pub noise_floor_db: f64,
}

/// Adaptive narrowband interference exciser.
///
/// Operates on blocks of IQ samples equal to `fft_size`. Input vectors that
/// are longer than `fft_size` are processed in consecutive non-overlapping
/// blocks; a remainder shorter than `fft_size` is zero-padded internally and
/// truncated on output.
#[derive(Debug, Clone)]
pub struct InterferenceExciser {
    config: ExcisionConfig,
    /// Running noise floor estimate (dB), updated each block.
    noise_floor_db: f64,
    /// Total blocks processed.
    blocks_processed: u64,
}

impl InterferenceExciser {
    /// Create a new exciser with the given configuration.
    ///
    /// # Panics
    ///
    /// Panics if `fft_size` is not a power of two or is zero.
    pub fn new(config: ExcisionConfig) -> Self {
        assert!(
            config.fft_size.is_power_of_two() && config.fft_size > 0,
            "fft_size must be a positive power of two"
        );
        Self {
            config,
            noise_floor_db: f64::NEG_INFINITY,
            blocks_processed: 0,
        }
    }

    /// Reset internal state (noise floor estimate, block counter).
    pub fn reset(&mut self) {
        self.noise_floor_db = f64::NEG_INFINITY;
        self.blocks_processed = 0;
    }

    /// Return the current configuration.
    pub fn config(&self) -> &ExcisionConfig {
        &self.config
    }

    /// Return the current noise-floor estimate in dB.
    pub fn noise_floor_db(&self) -> f64 {
        self.noise_floor_db
    }

    // ── Internal helpers ───────────────────────────────────────────────

    /// Compute power spectrum (linear) from a frequency-domain buffer.
    fn power_spectrum(freq: &[C64]) -> Vec<f64> {
        freq.iter().map(|&s| c_mag_sq(s)).collect()
    }

    /// Estimate the noise floor from a power spectrum using the median.
    /// Returns power in linear scale.
    fn estimate_noise_floor_linear(power: &[f64]) -> f64 {
        let mut sorted: Vec<f64> = power.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mid = sorted.len() / 2;
        if sorted.len() % 2 == 0 && sorted.len() >= 2 {
            (sorted[mid - 1] + sorted[mid]) / 2.0
        } else {
            sorted[mid]
        }
    }

    /// Convert linear power to dB, clamping to a minimum of -200 dB.
    fn to_db(linear: f64) -> f64 {
        if linear <= 0.0 {
            -200.0
        } else {
            10.0 * linear.log10()
        }
    }

    /// Convert dB to linear power.
    fn from_db(db: f64) -> f64 {
        10.0_f64.powf(db / 10.0)
    }

    /// Detect interferers in one FFT block's power spectrum.
    fn detect_in_spectrum(&self, power: &[f64], noise_floor_linear: f64) -> Vec<InterfererInfo> {
        let n = power.len();
        let threshold_linear = noise_floor_linear * Self::from_db(self.config.detection_threshold_db);

        // Find all bins above threshold
        let mut above: Vec<bool> = power.iter().map(|&p| p > threshold_linear).collect();

        // Group contiguous bins into interferer regions
        let mut interferers = Vec::new();
        let mut i = 0;
        while i < n && interferers.len() < self.config.max_interferers {
            if above[i] {
                let start = i;
                let mut peak_bin = i;
                let mut peak_power = power[i];
                while i < n && above[i] {
                    if power[i] > peak_power {
                        peak_power = power[i];
                        peak_bin = i;
                    }
                    i += 1;
                }
                let span = i - start;
                let nf_db = Self::to_db(noise_floor_linear);
                let power_db = Self::to_db(peak_power) - nf_db;
                let interferer_type = if span <= 2 {
                    InterfererType::Cw
                } else {
                    InterfererType::Wideband
                };
                interferers.push(InterfererInfo {
                    frequency_bin: peak_bin,
                    power_db,
                    bandwidth_bins: span,
                    interferer_type,
                });
                // Mark grouped bins to avoid double-counting
                for b in start..i {
                    above[b] = false;
                }
            } else {
                i += 1;
            }
        }

        // Sort by power descending, keep top max_interferers
        interferers.sort_by(|a, b| b.power_db.partial_cmp(&a.power_db).unwrap_or(std::cmp::Ordering::Equal));
        interferers.truncate(self.config.max_interferers);
        interferers
    }

    /// Apply spectral notching: interpolate bins around each detected interferer
    /// for smooth transitions.
    fn apply_notch(freq: &mut [C64], interferers: &[InterfererInfo], guard_bins: usize) {
        let n = freq.len();
        for info in interferers {
            let centre = info.frequency_bin;
            let half_span = info.bandwidth_bins / 2 + guard_bins;
            let lo = if centre >= half_span { centre - half_span } else { 0 };
            let hi = (centre + half_span + 1).min(n);

            // Grab edge values for interpolation
            let left_val = if lo > 0 { freq[lo - 1] } else { (0.0, 0.0) };
            let right_val = if hi < n { freq[hi] } else { (0.0, 0.0) };
            let span = hi - lo;

            for (j, bin) in freq[lo..hi].iter_mut().enumerate() {
                if span > 1 {
                    // Linear interpolation across the notch
                    let t = (j as f64) / ((span - 1) as f64);
                    *bin = c_add(c_scale(left_val, 1.0 - t), c_scale(right_val, t));
                } else {
                    *bin = (0.0, 0.0);
                }
            }
        }
    }

    /// Estimate SIR (dB) given a power spectrum, noise floor (linear), and
    /// detected interferers.
    fn estimate_sir(power: &[f64], noise_floor_linear: f64, interferers: &[InterfererInfo], guard_bins: usize) -> f64 {
        let n = power.len();
        let mut signal_power = 0.0;
        let mut interference_power = 0.0;

        // Build a mask of interference bins
        let mut is_interference = vec![false; n];
        for info in interferers {
            let centre = info.frequency_bin;
            let half_span = info.bandwidth_bins / 2 + guard_bins;
            let lo = if centre >= half_span { centre - half_span } else { 0 };
            let hi = (centre + half_span + 1).min(n);
            for bin in lo..hi {
                is_interference[bin] = true;
            }
        }

        for (i, &p) in power.iter().enumerate() {
            if is_interference[i] {
                // Interference power above noise floor
                interference_power += (p - noise_floor_linear).max(0.0);
            } else {
                signal_power += p;
            }
        }

        if interference_power <= 0.0 {
            return 100.0; // effectively no interference
        }
        Self::to_db(signal_power / interference_power)
    }

    // ── Public API ─────────────────────────────────────────────────────

    /// Detect interferers in the given IQ samples without modifying them.
    ///
    /// Operates on the first `fft_size` samples (zero-pads if shorter).
    pub fn detect_interferers(&self, samples: &[C64]) -> Vec<InterfererInfo> {
        let n = self.config.fft_size;
        let mut buf = vec![(0.0, 0.0); n];
        let copy_len = samples.len().min(n);
        buf[..copy_len].copy_from_slice(&samples[..copy_len]);

        fft_in_place(&mut buf, false);
        let power = Self::power_spectrum(&buf);
        let noise_floor = Self::estimate_noise_floor_linear(&power);
        self.detect_in_spectrum(&power, noise_floor)
    }

    /// Excise the specified interferers from the given IQ samples.
    ///
    /// This does **not** run detection — use the interferers list you provide.
    /// Processes in `fft_size` blocks; zero-pads the last block if needed.
    pub fn excise(&mut self, samples: &[C64], interferers: &[InterfererInfo]) -> Vec<C64> {
        let n = self.config.fft_size;
        let total = samples.len();
        let mut output = Vec::with_capacity(total);

        let mut offset = 0;
        while offset < total {
            let remaining = total - offset;
            let block_len = remaining.min(n);
            let mut buf = vec![(0.0, 0.0); n];
            buf[..block_len].copy_from_slice(&samples[offset..offset + block_len]);

            fft_in_place(&mut buf, false);
            Self::apply_notch(&mut buf, interferers, self.config.notch_bandwidth_bins);
            fft_in_place(&mut buf, true);

            // Keep only valid samples (no zero-pad artefacts)
            output.extend_from_slice(&buf[..block_len]);
            offset += block_len;
        }

        output
    }

    /// Detect interferers and excise them in one step, returning cleaned samples.
    pub fn adaptive_excise(&mut self, samples: &[C64]) -> Vec<C64> {
        let result = self.adaptive_excise_with_info(samples);
        result.samples
    }

    /// Like [`adaptive_excise`](Self::adaptive_excise) but returns an
    /// [`ExcisionResult`] with SIR estimates and detected interferers.
    pub fn adaptive_excise_with_info(&mut self, samples: &[C64]) -> ExcisionResult {
        let n = self.config.fft_size;
        let total = samples.len();
        let mut output = Vec::with_capacity(total);
        let mut all_interferers: Vec<InterfererInfo> = Vec::new();
        let mut total_sir_before = 0.0_f64;
        let mut total_sir_after = 0.0_f64;
        let mut block_count = 0u64;
        let mut last_noise_floor_db = self.noise_floor_db;

        let mut offset = 0;
        while offset < total {
            let remaining = total - offset;
            let block_len = remaining.min(n);
            let mut buf = vec![(0.0, 0.0); n];
            buf[..block_len].copy_from_slice(&samples[offset..offset + block_len]);

            // Forward FFT
            fft_in_place(&mut buf, false);
            let power_before = Self::power_spectrum(&buf);
            let noise_floor_linear = Self::estimate_noise_floor_linear(&power_before);
            let nf_db = Self::to_db(noise_floor_linear);

            // Update running noise floor (exponential moving average)
            if self.noise_floor_db.is_finite() {
                self.noise_floor_db = 0.8 * self.noise_floor_db + 0.2 * nf_db;
            } else {
                self.noise_floor_db = nf_db;
            }
            last_noise_floor_db = self.noise_floor_db;

            // Detect
            let interferers = self.detect_in_spectrum(&power_before, noise_floor_linear);
            let sir_before = Self::estimate_sir(
                &power_before,
                noise_floor_linear,
                &interferers,
                self.config.notch_bandwidth_bins,
            );

            // Excise
            Self::apply_notch(&mut buf, &interferers, self.config.notch_bandwidth_bins);

            // Measure SIR after
            let power_after = Self::power_spectrum(&buf);
            let sir_after = Self::estimate_sir(
                &power_after,
                noise_floor_linear,
                &interferers,
                self.config.notch_bandwidth_bins,
            );

            // Inverse FFT
            fft_in_place(&mut buf, true);
            output.extend_from_slice(&buf[..block_len]);

            total_sir_before += sir_before;
            total_sir_after += sir_after;
            all_interferers.extend(interferers);
            block_count += 1;
            self.blocks_processed += 1;

            offset += block_len;
        }

        let avg = |v: f64| if block_count > 0 { v / block_count as f64 } else { 0.0 };

        ExcisionResult {
            samples: output,
            interferers: all_interferers,
            sir_before_db: avg(total_sir_before),
            sir_after_db: avg(total_sir_after),
            noise_floor_db: last_noise_floor_db,
        }
    }
}

// ─── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> ExcisionConfig {
        ExcisionConfig {
            fft_size: 64,
            detection_threshold_db: 15.0,
            notch_bandwidth_bins: 1,
            max_interferers: 4,
        }
    }

    /// Generate a pure tone at the given bin index.
    fn tone(n: usize, bin: usize, amplitude: f64) -> Vec<C64> {
        (0..n)
            .map(|i| {
                let phase = 2.0 * PI * (i as f64) * (bin as f64) / (n as f64);
                (amplitude * phase.cos(), amplitude * phase.sin())
            })
            .collect()
    }

    /// Generate white-ish pseudo-noise (deterministic).
    fn pseudo_noise(n: usize, amplitude: f64) -> Vec<C64> {
        // Simple LCG for determinism
        let mut state: u64 = 0xDEAD_BEEF;
        (0..n)
            .map(|_| {
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
                let r1 = ((state >> 33) as f64) / (u32::MAX as f64) - 0.5;
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
                let r2 = ((state >> 33) as f64) / (u32::MAX as f64) - 0.5;
                (amplitude * r1, amplitude * r2)
            })
            .collect()
    }

    #[test]
    fn test_config_default() {
        let cfg = ExcisionConfig::default();
        assert_eq!(cfg.fft_size, 1024);
        assert_eq!(cfg.detection_threshold_db, 20.0);
        assert_eq!(cfg.notch_bandwidth_bins, 2);
        assert_eq!(cfg.max_interferers, 8);
    }

    #[test]
    fn test_detect_single_cw_interferer() {
        let cfg = default_config();
        let exciser = InterferenceExciser::new(cfg.clone());

        // Strong tone at bin 10 plus noise
        let n = cfg.fft_size;
        let mut samples = pseudo_noise(n, 0.1);
        let cw = tone(n, 10, 20.0);
        for i in 0..n {
            samples[i] = c_add(samples[i], cw[i]);
        }

        let interferers = exciser.detect_interferers(&samples);
        assert!(!interferers.is_empty(), "Should detect at least one interferer");
        // The strongest interferer should be near bin 10
        assert_eq!(interferers[0].frequency_bin, 10);
        assert_eq!(interferers[0].interferer_type, InterfererType::Cw);
    }

    #[test]
    fn test_detect_multiple_interferers() {
        let cfg = ExcisionConfig {
            fft_size: 128,
            detection_threshold_db: 15.0,
            notch_bandwidth_bins: 1,
            max_interferers: 8,
        };
        let exciser = InterferenceExciser::new(cfg.clone());
        let n = cfg.fft_size;

        let mut samples = pseudo_noise(n, 0.1);
        // Two tones at bins 20 and 50
        let t1 = tone(n, 20, 15.0);
        let t2 = tone(n, 50, 12.0);
        for i in 0..n {
            samples[i] = c_add(c_add(samples[i], t1[i]), t2[i]);
        }

        let interferers = exciser.detect_interferers(&samples);
        assert!(
            interferers.len() >= 2,
            "Should detect at least 2 interferers, found {}",
            interferers.len()
        );

        let bins: Vec<usize> = interferers.iter().map(|inf| inf.frequency_bin).collect();
        assert!(bins.contains(&20), "Should find interferer at bin 20");
        assert!(bins.contains(&50), "Should find interferer at bin 50");
    }

    #[test]
    fn test_no_interferers_in_noise() {
        // Use a high threshold to avoid false detections from LCG spectral unevenness
        let cfg = ExcisionConfig {
            fft_size: 64,
            detection_threshold_db: 25.0,
            notch_bandwidth_bins: 1,
            max_interferers: 4,
        };
        let exciser = InterferenceExciser::new(cfg.clone());
        let samples = pseudo_noise(cfg.fft_size, 1.0);
        let interferers = exciser.detect_interferers(&samples);
        assert!(
            interferers.is_empty(),
            "Uniform noise should not trigger detections, found {}",
            interferers.len()
        );
    }

    #[test]
    fn test_excise_removes_interferer_power() {
        let cfg = default_config();
        let mut exciser = InterferenceExciser::new(cfg.clone());
        let n = cfg.fft_size;

        let mut samples = pseudo_noise(n, 0.1);
        let cw = tone(n, 16, 30.0);
        for i in 0..n {
            samples[i] = c_add(samples[i], cw[i]);
        }

        let before_power: f64 = samples.iter().map(|&s| c_mag_sq(s)).sum();
        let cleaned = exciser.adaptive_excise(&samples);
        let after_power: f64 = cleaned.iter().map(|&s| c_mag_sq(s)).sum();

        // The cleaned signal should have significantly less power
        assert!(
            after_power < before_power * 0.5,
            "Power should drop after excision: before={before_power:.1}, after={after_power:.1}"
        );
    }

    #[test]
    fn test_adaptive_excise_with_info() {
        let cfg = default_config();
        let mut exciser = InterferenceExciser::new(cfg.clone());
        let n = cfg.fft_size;

        let mut samples = pseudo_noise(n, 0.1);
        let cw = tone(n, 8, 25.0);
        for i in 0..n {
            samples[i] = c_add(samples[i], cw[i]);
        }

        let result = exciser.adaptive_excise_with_info(&samples);
        assert_eq!(result.samples.len(), n);
        assert!(!result.interferers.is_empty());
        assert!(
            result.sir_after_db > result.sir_before_db,
            "SIR should improve: before={:.1} dB, after={:.1} dB",
            result.sir_before_db,
            result.sir_after_db
        );
        assert!(result.noise_floor_db.is_finite());
    }

    #[test]
    fn test_noise_floor_estimation_updates() {
        let cfg = default_config();
        let mut exciser = InterferenceExciser::new(cfg.clone());
        assert!(exciser.noise_floor_db().is_infinite());

        let samples = pseudo_noise(cfg.fft_size, 1.0);
        let _ = exciser.adaptive_excise(&samples);
        let nf1 = exciser.noise_floor_db();
        assert!(nf1.is_finite(), "Noise floor should be updated");

        // Process another block — EMA should nudge the value
        let _ = exciser.adaptive_excise(&samples);
        let nf2 = exciser.noise_floor_db();
        assert!(nf2.is_finite());
    }

    #[test]
    fn test_reset() {
        let cfg = default_config();
        let mut exciser = InterferenceExciser::new(cfg.clone());
        let samples = pseudo_noise(cfg.fft_size, 1.0);
        let _ = exciser.adaptive_excise(&samples);
        assert!(exciser.noise_floor_db().is_finite());

        exciser.reset();
        assert!(exciser.noise_floor_db().is_infinite());
        assert_eq!(exciser.blocks_processed, 0);
    }

    #[test]
    fn test_zero_pad_short_input() {
        let cfg = default_config(); // fft_size = 64
        let mut exciser = InterferenceExciser::new(cfg);

        // Provide fewer than fft_size samples
        let samples = pseudo_noise(30, 1.0);
        let out = exciser.adaptive_excise(&samples);
        assert_eq!(out.len(), 30, "Output length should match input length");
    }

    #[test]
    fn test_multi_block_processing() {
        let cfg = default_config(); // fft_size = 64
        let mut exciser = InterferenceExciser::new(cfg.clone());

        // 3 full blocks
        let n = cfg.fft_size * 3;
        let mut samples = pseudo_noise(n, 0.1);
        let cw = tone(n, 10, 20.0);
        for i in 0..n {
            samples[i] = c_add(samples[i], cw[i]);
        }

        let out = exciser.adaptive_excise(&samples);
        assert_eq!(out.len(), n, "Output length should equal input length");
    }

    #[test]
    fn test_excise_preserves_signal_without_interferer() {
        // Use a high threshold so noise alone does not trigger any excision
        let cfg = ExcisionConfig {
            fft_size: 64,
            detection_threshold_db: 25.0,
            notch_bandwidth_bins: 1,
            max_interferers: 4,
        };
        let mut exciser = InterferenceExciser::new(cfg.clone());
        let samples = pseudo_noise(cfg.fft_size, 1.0);

        let cleaned = exciser.adaptive_excise(&samples);
        assert_eq!(cleaned.len(), samples.len());

        // Without interferers the signal should be mostly unchanged
        let mut max_err = 0.0_f64;
        for (a, b) in samples.iter().zip(cleaned.iter()) {
            let err = c_mag_sq(c_sub(*a, *b)).sqrt();
            if err > max_err {
                max_err = err;
            }
        }
        // Allow small numerical noise from FFT round-trip
        assert!(
            max_err < 1e-10,
            "Signal should be preserved, max error = {max_err}"
        );
    }

    #[test]
    fn test_max_interferers_limit() {
        let cfg = ExcisionConfig {
            fft_size: 256,
            detection_threshold_db: 10.0,
            notch_bandwidth_bins: 1,
            max_interferers: 2,
        };
        let exciser = InterferenceExciser::new(cfg.clone());
        let n = cfg.fft_size;

        let mut samples = pseudo_noise(n, 0.01);
        // Insert 5 strong tones
        for &bin in &[10, 40, 80, 120, 200] {
            let t = tone(n, bin, 20.0);
            for i in 0..n {
                samples[i] = c_add(samples[i], t[i]);
            }
        }

        let interferers = exciser.detect_interferers(&samples);
        assert!(
            interferers.len() <= 2,
            "Should respect max_interferers=2, found {}",
            interferers.len()
        );
    }

    #[test]
    fn test_wideband_interferer_classification() {
        // Create a signal with energy spread across several adjacent bins
        let cfg = ExcisionConfig {
            fft_size: 128,
            detection_threshold_db: 15.0,
            notch_bandwidth_bins: 1,
            max_interferers: 4,
        };
        let exciser = InterferenceExciser::new(cfg.clone());
        let n = cfg.fft_size;

        let mut samples = pseudo_noise(n, 0.05);
        // Add tones at adjacent bins 30,31,32 to simulate wideband
        for &bin in &[30, 31, 32] {
            let t = tone(n, bin, 15.0);
            for i in 0..n {
                samples[i] = c_add(samples[i], t[i]);
            }
        }

        let interferers = exciser.detect_interferers(&samples);
        assert!(!interferers.is_empty());
        // The grouped interferer should span >=3 bins -> Wideband
        let wb = interferers
            .iter()
            .find(|inf| inf.bandwidth_bins >= 3);
        assert!(
            wb.is_some(),
            "Should detect a wideband interferer spanning >=3 bins"
        );
        if let Some(wb_info) = wb {
            assert_eq!(wb_info.interferer_type, InterfererType::Wideband);
        }
    }

    #[test]
    fn test_fft_roundtrip_identity() {
        // Verify our FFT implementation: FFT then IFFT should be identity
        let n = 64;
        let original = pseudo_noise(n, 1.0);
        let mut buf = original.clone();
        fft_in_place(&mut buf, false);
        fft_in_place(&mut buf, true);
        for (a, b) in original.iter().zip(buf.iter()) {
            let err = c_mag_sq(c_sub(*a, *b)).sqrt();
            assert!(err < 1e-12, "FFT roundtrip error too large: {err}");
        }
    }
}

//! Channel Sounding Processor -- Impulse Response Measurement and Channel Characterization
//!
//! This module provides channel sounding and impulse response measurement for
//! wireless channel characterization. It estimates the channel impulse response (CIR)
//! via cross-correlation of a known sounding sequence with the received signal, then
//! derives channel statistics such as RMS delay spread, coherence bandwidth, power
//! delay profile, and Rician K-factor.
//!
//! Uses only `(f64, f64)` tuples for complex I/Q samples (no external crate dependencies).
//!
//! # Example
//!
//! ```rust
//! use r4w_core::channel_sounding_processor::{
//!     ChannelSounder, SoundingSequence, generate_sounding_sequence,
//!     compute_channel_stats, power_delay_profile, wideband_sounder,
//! };
//!
//! // Generate a Zadoff-Chu sounding sequence
//! let seq = generate_sounding_sequence(SoundingSequence::ZadoffChu(7), 64, 1e6);
//! let sounder = ChannelSounder::new(seq.clone(), 1e6);
//!
//! // Simulate a single-path channel (identity)
//! let cir_mag = sounder.estimate_cir(&seq);
//! assert!(cir_mag[0] > 0.5);
//!
//! // Compute channel statistics
//! let stats = compute_channel_stats(&cir_mag, 1e6, -20.0);
//! assert!(stats.rms_delay_spread_s >= 0.0);
//!
//! // Power delay profile
//! let pdp = power_delay_profile(&cir_mag);
//! let max_val = pdp.iter().cloned().fold(0.0_f64, f64::max);
//! assert!((max_val - 1.0).abs() < 1e-9);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers (using (f64, f64) tuples)
// ---------------------------------------------------------------------------

#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

#[inline]
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_mag(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

#[inline]
fn c_mag_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

// ---------------------------------------------------------------------------
// ChannelStats
// ---------------------------------------------------------------------------

/// Channel statistics derived from a measured channel impulse response.
#[derive(Debug, Clone)]
pub struct ChannelStats {
    /// RMS delay spread in seconds.
    pub rms_delay_spread_s: f64,
    /// Mean excess delay in seconds.
    pub mean_excess_delay_s: f64,
    /// Maximum excess delay in seconds (relative to first path above threshold).
    pub max_excess_delay_s: f64,
    /// Coherence bandwidth in Hz (approx 1 / (5 * rms_delay_spread)).
    pub coherence_bandwidth_hz: f64,
    /// Number of significant multipath components above the threshold.
    pub num_paths: usize,
    /// Rician K-factor in dB (strongest path power vs scattered power).
    pub k_factor_db: f64,
}

// ---------------------------------------------------------------------------
// ChannelSounder
// ---------------------------------------------------------------------------

/// Correlation-based channel sounder using a known sounding sequence.
#[derive(Debug, Clone)]
pub struct ChannelSounder {
    /// The reference sounding sequence (complex I/Q samples).
    sounding_sequence: Vec<(f64, f64)>,
    /// Sample rate in Hz.
    sample_rate: f64,
    /// Energy of the sounding sequence (for normalization).
    seq_energy: f64,
}

impl ChannelSounder {
    /// Create a new channel sounder from a known sounding sequence.
    ///
    /// * `sounding_sequence` - The reference complex I/Q sounding sequence.
    /// * `sample_rate` - Sample rate in Hz.
    pub fn new(sounding_sequence: Vec<(f64, f64)>, sample_rate: f64) -> Self {
        let seq_energy: f64 = sounding_sequence.iter().map(|s| c_mag_sq(*s)).sum();
        Self {
            sounding_sequence,
            sample_rate,
            seq_energy,
        }
    }

    /// Estimate the channel impulse response magnitude via cross-correlation.
    ///
    /// Returns a vector of |h(tau)| values, one per sample delay offset.
    pub fn estimate_cir(&self, received: &[(f64, f64)]) -> Vec<f64> {
        self.estimate_cir_complex(received)
            .iter()
            .map(|c| c_mag(*c))
            .collect()
    }

    /// Estimate the complex channel impulse response via cross-correlation.
    ///
    /// Returns a vector of complex (I, Q) correlation values, one per sample delay offset.
    pub fn estimate_cir_complex(&self, received: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let ref_len = self.sounding_sequence.len();
        if received.len() < ref_len || ref_len == 0 {
            return vec![];
        }

        let num_lags = received.len() - ref_len + 1;
        let norm = if self.seq_energy > 0.0 {
            self.seq_energy
        } else {
            1.0
        };

        let mut cir = Vec::with_capacity(num_lags);
        for lag in 0..num_lags {
            let mut acc = (0.0, 0.0);
            for k in 0..ref_len {
                acc = c_add(acc, c_mul(received[lag + k], c_conj(self.sounding_sequence[k])));
            }
            cir.push((acc.0 / norm, acc.1 / norm));
        }
        cir
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Compute comprehensive channel statistics from a CIR magnitude profile.
///
/// * `cir_magnitude` - Channel impulse response magnitude values (one per sample).
/// * `sample_rate` - Sample rate in Hz.
/// * `threshold_db` - Threshold in dB below the peak for counting significant paths.
pub fn compute_channel_stats(cir_magnitude: &[f64], sample_rate: f64, threshold_db: f64) -> ChannelStats {
    if cir_magnitude.is_empty() || sample_rate <= 0.0 {
        return ChannelStats {
            rms_delay_spread_s: 0.0,
            mean_excess_delay_s: 0.0,
            max_excess_delay_s: 0.0,
            coherence_bandwidth_hz: f64::INFINITY,
            num_paths: 0,
            k_factor_db: f64::NEG_INFINITY,
        };
    }

    let peak = cir_magnitude.iter().cloned().fold(0.0_f64, f64::max);
    if peak <= 0.0 {
        return ChannelStats {
            rms_delay_spread_s: 0.0,
            mean_excess_delay_s: 0.0,
            max_excess_delay_s: 0.0,
            coherence_bandwidth_hz: f64::INFINITY,
            num_paths: 0,
            k_factor_db: f64::NEG_INFINITY,
        };
    }

    // Power profile
    let power: Vec<f64> = cir_magnitude.iter().map(|m| m * m).collect();

    // Threshold in linear power
    let peak_power = peak * peak;
    let lin_threshold = peak_power * 10.0_f64.powf(threshold_db / 10.0);

    // Count significant paths
    let num_paths = power.iter().filter(|&&p| p >= lin_threshold).count();

    // Find first and last path indices above threshold
    let first_path = power.iter().position(|&p| p >= lin_threshold).unwrap_or(0);
    let last_path = power.iter().rposition(|&p| p >= lin_threshold).unwrap_or(0);

    let max_excess_delay_s = (last_path - first_path) as f64 / sample_rate;

    // Delay spread (using power-weighted delays, only paths above threshold)
    let rms_ds = rms_delay_spread(cir_magnitude, sample_rate);
    let mean_delay = mean_excess_delay(cir_magnitude, sample_rate);
    let coh_bw = coherence_bandwidth(rms_ds);
    let k_db = k_factor_estimate(cir_magnitude);

    ChannelStats {
        rms_delay_spread_s: rms_ds,
        mean_excess_delay_s: mean_delay,
        max_excess_delay_s,
        coherence_bandwidth_hz: coh_bw,
        num_paths,
        k_factor_db: k_db,
    }
}

/// Compute the RMS delay spread from a CIR magnitude profile.
///
/// RMS delay spread = sqrt(E[tau^2] - E[tau]^2), weighted by power |h(tau)|^2.
///
/// * `cir` - CIR magnitude values (one per sample).
/// * `sample_rate` - Sample rate in Hz.
pub fn rms_delay_spread(cir: &[f64], sample_rate: f64) -> f64 {
    if cir.is_empty() || sample_rate <= 0.0 {
        return 0.0;
    }

    let power: Vec<f64> = cir.iter().map(|m| m * m).collect();
    let total_power: f64 = power.iter().sum();
    if total_power <= 0.0 {
        return 0.0;
    }

    let dt = 1.0 / sample_rate;

    // E[tau] = sum(P_k * tau_k) / sum(P_k)
    let mean_tau: f64 = power
        .iter()
        .enumerate()
        .map(|(i, &p)| p * (i as f64 * dt))
        .sum::<f64>()
        / total_power;

    // E[tau^2] = sum(P_k * tau_k^2) / sum(P_k)
    let mean_tau_sq: f64 = power
        .iter()
        .enumerate()
        .map(|(i, &p)| {
            let tau = i as f64 * dt;
            p * tau * tau
        })
        .sum::<f64>()
        / total_power;

    let variance = mean_tau_sq - mean_tau * mean_tau;
    if variance > 0.0 {
        variance.sqrt()
    } else {
        0.0
    }
}

/// Compute the mean excess delay from a CIR magnitude profile.
fn mean_excess_delay(cir: &[f64], sample_rate: f64) -> f64 {
    if cir.is_empty() || sample_rate <= 0.0 {
        return 0.0;
    }
    let power: Vec<f64> = cir.iter().map(|m| m * m).collect();
    let total_power: f64 = power.iter().sum();
    if total_power <= 0.0 {
        return 0.0;
    }
    let dt = 1.0 / sample_rate;
    power
        .iter()
        .enumerate()
        .map(|(i, &p)| p * (i as f64 * dt))
        .sum::<f64>()
        / total_power
}

/// Compute the coherence bandwidth from RMS delay spread.
///
/// Uses the approximation: B_c ≈ 1 / (5 * sigma_tau).
pub fn coherence_bandwidth(delay_spread: f64) -> f64 {
    if delay_spread > 0.0 {
        1.0 / (5.0 * delay_spread)
    } else {
        f64::INFINITY
    }
}

/// Compute the normalized power delay profile from a CIR magnitude.
///
/// Returns |h(tau)|^2 normalized so the peak value is 1.0.
pub fn power_delay_profile(cir: &[f64]) -> Vec<f64> {
    if cir.is_empty() {
        return vec![];
    }

    let power: Vec<f64> = cir.iter().map(|m| m * m).collect();
    let max_power = power.iter().cloned().fold(0.0_f64, f64::max);
    if max_power <= 0.0 {
        return vec![0.0; cir.len()];
    }

    power.iter().map(|&p| p / max_power).collect()
}

/// Estimate the Rician K-factor from a CIR magnitude profile.
///
/// K = strongest_path_power / (total_power - strongest_path_power), returned in dB.
pub fn k_factor_estimate(cir: &[f64]) -> f64 {
    if cir.is_empty() {
        return f64::NEG_INFINITY;
    }

    let power: Vec<f64> = cir.iter().map(|m| m * m).collect();
    let total_power: f64 = power.iter().sum();
    let max_power = power.iter().cloned().fold(0.0_f64, f64::max);

    let scattered = total_power - max_power;
    if scattered > 0.0 {
        10.0 * (max_power / scattered).log10()
    } else if max_power > 0.0 {
        // Pure LOS, no scattering
        f64::INFINITY
    } else {
        f64::NEG_INFINITY
    }
}

// ---------------------------------------------------------------------------
// Sounding Sequence Generation
// ---------------------------------------------------------------------------

/// Type of sounding sequence to generate.
#[derive(Debug, Clone)]
pub enum SoundingSequence {
    /// Zadoff-Chu sequence with the given root index.
    ZadoffChu(usize),
    /// Pseudorandom noise sequence from an LFSR of the given order.
    PnSequence(usize),
    /// Linear FM chirp spanning the given bandwidth in Hz.
    Chirp(f64),
}

/// Generate a sounding sequence of the specified type and length.
///
/// * `seq_type` - Type of sounding sequence.
/// * `length` - Number of complex samples.
/// * `sample_rate` - Sample rate in Hz (used for chirp bandwidth mapping).
pub fn generate_sounding_sequence(
    seq_type: SoundingSequence,
    length: usize,
    sample_rate: f64,
) -> Vec<(f64, f64)> {
    match seq_type {
        SoundingSequence::ZadoffChu(root) => generate_zadoff_chu(root, length),
        SoundingSequence::PnSequence(order) => generate_pn_sequence(order, length),
        SoundingSequence::Chirp(bandwidth) => generate_chirp(bandwidth, length, sample_rate),
    }
}

/// Generate a Zadoff-Chu sequence of given root and length.
///
/// x_u(n) = exp(-j * pi * u * n * (n+1) / N) for odd N
/// x_u(n) = exp(-j * pi * u * n^2 / N) for even N
fn generate_zadoff_chu(root: usize, length: usize) -> Vec<(f64, f64)> {
    let n = length;
    let u = root as f64;
    (0..n)
        .map(|k| {
            let phase = if n % 2 == 1 {
                -PI * u * (k as f64) * (k as f64 + 1.0) / n as f64
            } else {
                -PI * u * (k as f64) * (k as f64) / n as f64
            };
            (phase.cos(), phase.sin())
        })
        .collect()
}

/// Generate a PN (pseudorandom noise) sequence from an LFSR.
fn generate_pn_sequence(order: usize, length: usize) -> Vec<(f64, f64)> {
    let order = order.max(2).min(16);
    // Primitive polynomial taps for LFSR
    let taps: u32 = match order {
        2 => 0b11,
        3 => 0b110,
        4 => 0b1100,
        5 => 0b10100,
        6 => 0b110000,
        7 => 0b1100000,
        8 => 0b10111000,
        9 => 0b100010000,
        10 => 0b1001000000,
        11 => 0b10100000000,
        12 => 0b111000001000,
        13 => 0b1110010000000,
        14 => 0b11100000000010,
        15 => 0b110000000000000,
        16 => 0b1101000000001000,
        _ => 0b110,
    };

    let mut reg: u32 = 1;
    let mut seq = Vec::with_capacity(length);

    for _ in 0..length {
        let bit = reg & 1;
        // Map 0 -> -1, 1 -> +1
        let val = if bit == 1 { 1.0 } else { -1.0 };
        seq.push((val, 0.0));

        let feedback = (reg & taps).count_ones() & 1;
        reg = (reg >> 1) | (feedback << (order as u32 - 1));
    }

    seq
}

/// Generate a linear FM chirp sequence.
fn generate_chirp(bandwidth: f64, length: usize, sample_rate: f64) -> Vec<(f64, f64)> {
    let duration = length as f64 / sample_rate;
    let chirp_rate = bandwidth / duration;
    (0..length)
        .map(|i| {
            let t = i as f64 / sample_rate;
            let phase = 2.0 * PI * (chirp_rate / 2.0) * t * t;
            (phase.cos(), phase.sin())
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

/// Create a wideband channel sounder configured for a given bandwidth.
///
/// Uses a Zadoff-Chu sequence with root index 7 and length chosen to cover
/// the bandwidth. The sequence length is the smallest odd number >= bandwidth / 1000
/// (at least 63).
///
/// * `bandwidth_hz` - Measurement bandwidth in Hz.
/// * `sample_rate` - Sample rate in Hz.
pub fn wideband_sounder(bandwidth_hz: f64, sample_rate: f64) -> ChannelSounder {
    // Choose a sequence length proportional to bandwidth, minimum 63
    let mut seq_len = (bandwidth_hz / 1000.0).ceil() as usize;
    if seq_len < 63 {
        seq_len = 63;
    }
    // Make it odd for cleaner Zadoff-Chu properties
    if seq_len % 2 == 0 {
        seq_len += 1;
    }

    let seq = generate_zadoff_chu(7, seq_len);
    ChannelSounder::new(seq, sample_rate)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: create a single-path received signal (identity channel).
    fn single_path_received(seq: &[(f64, f64)]) -> Vec<(f64, f64)> {
        seq.to_vec()
    }

    /// Helper: create a two-path received signal with a delayed attenuated copy.
    fn two_path_received(seq: &[(f64, f64)], delay: usize, attenuation: f64) -> Vec<(f64, f64)> {
        let total_len = seq.len() + delay;
        let mut rx = vec![(0.0, 0.0); total_len];
        // Direct path
        for (i, &s) in seq.iter().enumerate() {
            rx[i] = c_add(rx[i], s);
        }
        // Delayed path
        for (i, &s) in seq.iter().enumerate() {
            let idx = i + delay;
            if idx < total_len {
                rx[idx] = c_add(rx[idx], (s.0 * attenuation, s.1 * attenuation));
            }
        }
        rx
    }

    #[test]
    fn test_cir_single_path() {
        let seq = generate_sounding_sequence(SoundingSequence::ZadoffChu(7), 63, 1e6);
        let sounder = ChannelSounder::new(seq.clone(), 1e6);
        let rx = single_path_received(&seq);
        let cir = sounder.estimate_cir(&rx);

        // Only one lag (received == reference), so length should be 1
        assert_eq!(cir.len(), 1);
        // The single correlation value should be close to 1.0 (normalized)
        assert!(
            (cir[0] - 1.0).abs() < 1e-9,
            "Single-path CIR peak should be ~1.0, got {}",
            cir[0]
        );
    }

    #[test]
    fn test_cir_two_path() {
        let seq = generate_sounding_sequence(SoundingSequence::ZadoffChu(7), 63, 1e6);
        let sounder = ChannelSounder::new(seq.clone(), 1e6);
        let delay = 5;
        let rx = two_path_received(&seq, delay, 0.5);
        let cir = sounder.estimate_cir(&rx);

        // Should have delay+1 lags
        assert!(cir.len() >= delay + 1, "CIR should cover at least delay+1 lags");

        // Peak at lag 0 (direct path, should be close to 1.0)
        assert!(
            cir[0] > 0.9,
            "Direct path should have strong correlation, got {}",
            cir[0]
        );
        // Secondary peak at lag = delay (attenuated path, should be ~0.5)
        assert!(
            cir[delay] > 0.3,
            "Delayed path should show up at lag {}, got {}",
            delay,
            cir[delay]
        );
        // The direct path should be stronger than the delayed path
        assert!(
            cir[0] > cir[delay],
            "Direct path ({}) should be stronger than delayed path ({})",
            cir[0],
            cir[delay]
        );
    }

    #[test]
    fn test_delay_spread_single_path() {
        // Single-path: CIR has a single peak, delay spread should be ~0
        let mut cir_mag = vec![0.0; 64];
        cir_mag[0] = 1.0;
        let ds = rms_delay_spread(&cir_mag, 1e6);
        assert!(
            ds < 1e-10,
            "Single-path delay spread should be near zero, got {}",
            ds
        );
    }

    #[test]
    fn test_delay_spread_multipath() {
        // Two equal-power paths separated by 10 samples at 1 MHz -> 10 us apart
        let mut cir_mag = vec![0.0; 64];
        cir_mag[0] = 1.0;
        cir_mag[10] = 1.0;
        let sample_rate = 1e6;
        let ds = rms_delay_spread(&cir_mag, sample_rate);

        // With two equal-power paths at tau=0 and tau=10us:
        // E[tau] = 5us, E[tau^2] = 50us^2, sigma = sqrt(50 - 25) = 5us
        let expected = 5e-6; // 5 microseconds
        assert!(
            (ds - expected).abs() < 1e-9,
            "RMS delay spread should be ~5 us, got {} s",
            ds
        );
    }

    #[test]
    fn test_coherence_bandwidth() {
        let ds = 5e-6; // 5 us RMS delay spread
        let bw = coherence_bandwidth(ds);
        // B_c = 1 / (5 * 5e-6) = 40 kHz
        let expected = 40_000.0;
        assert!(
            (bw - expected).abs() < 1.0,
            "Coherence bandwidth should be ~40 kHz, got {} Hz",
            bw
        );

        // Zero delay spread -> infinite coherence bandwidth
        let bw_zero = coherence_bandwidth(0.0);
        assert!(bw_zero.is_infinite(), "Zero delay spread should give infinite coherence BW");
    }

    #[test]
    fn test_power_delay_profile_normalization() {
        let cir_mag = vec![1.0, 0.5, 0.25, 0.1];
        let pdp = power_delay_profile(&cir_mag);

        // Peak of PDP should be 1.0
        let max_val = pdp.iter().cloned().fold(0.0_f64, f64::max);
        assert!(
            (max_val - 1.0).abs() < 1e-12,
            "PDP peak should be normalized to 1.0, got {}",
            max_val
        );

        // Second tap: 0.5^2 / 1.0^2 = 0.25
        assert!(
            (pdp[1] - 0.25).abs() < 1e-12,
            "PDP[1] should be 0.25, got {}",
            pdp[1]
        );

        // Third tap: 0.25^2 / 1.0^2 = 0.0625
        assert!(
            (pdp[2] - 0.0625).abs() < 1e-12,
            "PDP[2] should be 0.0625, got {}",
            pdp[2]
        );
    }

    #[test]
    fn test_k_factor_strong_los() {
        // Strong LOS: one dominant path, tiny scattered power
        let mut cir_mag = vec![0.0; 32];
        cir_mag[0] = 10.0;
        cir_mag[5] = 0.1;
        cir_mag[10] = 0.05;

        let k_db = k_factor_estimate(&cir_mag);

        // LOS power = 100, scattered = 0.01 + 0.0025 = 0.0125
        // K = 100 / 0.0125 = 8000 => ~39 dB
        assert!(
            k_db > 30.0,
            "K-factor should be large for strong LOS, got {} dB",
            k_db
        );
    }

    #[test]
    fn test_channel_stats_computation() {
        let mut cir_mag = vec![0.0; 64];
        cir_mag[0] = 1.0;
        cir_mag[5] = 0.7;
        cir_mag[15] = 0.3;
        let sample_rate = 1e6;

        let stats = compute_channel_stats(&cir_mag, sample_rate, -20.0);

        assert!(stats.rms_delay_spread_s > 0.0, "Should have non-zero delay spread");
        assert!(stats.mean_excess_delay_s > 0.0, "Should have non-zero mean delay");
        assert!(stats.max_excess_delay_s > 0.0, "Should have non-zero max excess delay");
        assert!(
            stats.coherence_bandwidth_hz > 0.0 && stats.coherence_bandwidth_hz.is_finite(),
            "Should have finite coherence bandwidth"
        );
        assert!(stats.num_paths >= 2, "Should detect at least 2 significant paths, got {}", stats.num_paths);
        assert!(
            stats.k_factor_db.is_finite(),
            "K-factor should be finite for multipath channel"
        );
    }

    #[test]
    fn test_sounding_sequence_generation() {
        let sample_rate = 1e6;
        let length = 127;

        // Zadoff-Chu: all samples should have unit magnitude
        let zc = generate_sounding_sequence(SoundingSequence::ZadoffChu(5), length, sample_rate);
        assert_eq!(zc.len(), length);
        for (i, &s) in zc.iter().enumerate() {
            let mag = c_mag(s);
            assert!(
                (mag - 1.0).abs() < 1e-9,
                "ZC sample {} magnitude should be 1.0, got {}",
                i,
                mag
            );
        }

        // PN sequence: all samples should be +/-1 (real only)
        let pn = generate_sounding_sequence(SoundingSequence::PnSequence(7), length, sample_rate);
        assert_eq!(pn.len(), length);
        for (i, &s) in pn.iter().enumerate() {
            assert!(
                (s.0.abs() - 1.0).abs() < 1e-12 && s.1.abs() < 1e-12,
                "PN sample {} should be (+/-1, 0), got ({}, {})",
                i,
                s.0,
                s.1
            );
        }

        // Chirp: all samples should have unit magnitude
        let chirp = generate_sounding_sequence(SoundingSequence::Chirp(100e3), length, sample_rate);
        assert_eq!(chirp.len(), length);
        for (i, &s) in chirp.iter().enumerate() {
            let mag = c_mag(s);
            assert!(
                (mag - 1.0).abs() < 1e-9,
                "Chirp sample {} magnitude should be 1.0, got {}",
                i,
                mag
            );
        }
    }

    #[test]
    fn test_wideband_sounder_factory() {
        let sounder = wideband_sounder(1e6, 2e6);

        // Sequence length should be at least 63 and odd
        let seq_len = sounder.sounding_sequence.len();
        assert!(seq_len >= 63, "Sequence should be at least 63 samples, got {}", seq_len);
        assert!(seq_len % 2 == 1, "Sequence length should be odd, got {}", seq_len);

        // Should be able to estimate CIR through an identity channel
        let rx = sounder.sounding_sequence.clone();
        let cir = sounder.estimate_cir(&rx);
        assert_eq!(cir.len(), 1);
        assert!(
            (cir[0] - 1.0).abs() < 1e-9,
            "Identity channel CIR should be ~1.0, got {}",
            cir[0]
        );

        // Factory with small bandwidth should still produce min 63 samples
        let small = wideband_sounder(10e3, 44100.0);
        assert!(
            small.sounding_sequence.len() >= 63,
            "Minimum sequence length should be 63"
        );
    }
}

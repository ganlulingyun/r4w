//! # NOMA (Non-Orthogonal Multiple Access) Encoder and SIC Decoder
//!
//! This module implements power-domain NOMA for multiplexing 2–4 users onto
//! a single time-frequency resource.  The transmitter uses **superposition
//! coding** — each user's signal is scaled by the square-root of its allocated
//! power fraction and the signals are summed.  The receiver applies
//! **successive interference cancellation (SIC)**: it decodes the strongest
//! user first, reconstructs and subtracts its contribution, then decodes the
//! next user, and so on.
//!
//! All complex numbers are represented as `(f64, f64)` tuples `(re, im)`.
//! Only the standard library is used.
//!
//! # Example
//!
//! ```
//! use r4w_core::noma_decoder::{NomaEncoder, NomaDecoder, UserConfig, Modulation};
//!
//! // Two users with BPSK modulation
//! let users = vec![
//!     UserConfig { user_id: 0, power_fraction: 0.8, modulation: Modulation::Bpsk },
//!     UserConfig { user_id: 1, power_fraction: 0.2, modulation: Modulation::Bpsk },
//! ];
//!
//! // Each user sends 4 BPSK symbols (±1)
//! let user_symbols: Vec<Vec<(f64, f64)>> = vec![
//!     vec![(1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (-1.0, 0.0)],
//!     vec![(1.0, 0.0), (1.0, 0.0), (-1.0, 0.0), (-1.0, 0.0)],
//! ];
//!
//! let encoder = NomaEncoder::new(users.clone());
//! let composite = encoder.encode(&user_symbols);
//! assert_eq!(composite.len(), 4);
//!
//! // Decode with SIC (no noise → perfect recovery)
//! let channel_gains = vec![1.0_f64, 0.5];
//! let decoder = NomaDecoder::new(users, channel_gains);
//! let result = decoder.decode_sic(&composite, 4);
//! assert_eq!(result.decoded_symbols.len(), 2);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Modulation scheme used by a NOMA user.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Modulation {
    /// Binary Phase Shift Keying (1 bit/symbol).
    Bpsk,
    /// Quadrature Phase Shift Keying (2 bits/symbol).
    Qpsk,
}

/// Per-user configuration for the NOMA system.
#[derive(Debug, Clone)]
pub struct UserConfig {
    /// Unique identifier for the user (index).
    pub user_id: usize,
    /// Fraction of the total transmit power allocated to this user (0.0–1.0).
    pub power_fraction: f64,
    /// Modulation scheme.
    pub modulation: Modulation,
}

/// Result of SIC decoding.
#[derive(Debug, Clone)]
pub struct SicResult {
    /// Decoded symbols per user, in the SIC decoding order (strongest first).
    pub decoded_symbols: Vec<Vec<(f64, f64)>>,
    /// User IDs in the SIC decoding order.
    pub user_order: Vec<usize>,
    /// Bit-error rate estimate per user (set to 0.0; caller should compare
    /// against ground truth externally).
    pub per_user_ber: Vec<f64>,
    /// Residual interference power after all users have been cancelled.
    pub residual_interference: f64,
}

// ---------------------------------------------------------------------------
// Encoder
// ---------------------------------------------------------------------------

/// Superposition-coded NOMA encoder.
///
/// Scales each user's symbol stream by `sqrt(power_fraction)` and sums them
/// to produce a composite signal.
#[derive(Debug, Clone)]
pub struct NomaEncoder {
    users: Vec<UserConfig>,
}

impl NomaEncoder {
    /// Create a new encoder.  Panics if `users` is empty or has more than 4
    /// entries, or if power fractions do not sum to approximately 1.
    pub fn new(users: Vec<UserConfig>) -> Self {
        assert!(!users.is_empty(), "At least one user is required");
        assert!(users.len() <= 4, "At most 4 simultaneous users supported");
        let total: f64 = users.iter().map(|u| u.power_fraction).sum();
        assert!(
            (total - 1.0).abs() < 1e-6,
            "Power fractions must sum to 1.0, got {total}"
        );
        Self { users }
    }

    /// Superimpose user signals.
    ///
    /// `user_symbols[i]` is the symbol sequence for `self.users[i]`.
    /// All sequences must have the same length.  Returns the composite
    /// signal vector.
    pub fn encode(&self, user_symbols: &[Vec<(f64, f64)>]) -> Vec<(f64, f64)> {
        assert_eq!(
            user_symbols.len(),
            self.users.len(),
            "Must provide symbols for every user"
        );
        let n = user_symbols[0].len();
        for (i, s) in user_symbols.iter().enumerate() {
            assert_eq!(
                s.len(),
                n,
                "User {i} symbol length ({}) differs from user 0 ({n})",
                s.len()
            );
        }

        let mut composite = vec![(0.0_f64, 0.0_f64); n];
        for (u_idx, cfg) in self.users.iter().enumerate() {
            let amp = cfg.power_fraction.sqrt();
            for (k, sym) in user_symbols[u_idx].iter().enumerate() {
                composite[k].0 += amp * sym.0;
                composite[k].1 += amp * sym.1;
            }
        }
        composite
    }
}

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------

/// SIC-based NOMA decoder.
///
/// Users are decoded in order of decreasing *effective received power*
/// (channel gain × allocated power).  After decoding each user the
/// reconstructed signal is subtracted from the composite.
#[derive(Debug, Clone)]
pub struct NomaDecoder {
    users: Vec<UserConfig>,
    /// Channel gain magnitude per user (linear, not dB).
    channel_gains: Vec<f64>,
}

impl NomaDecoder {
    /// Create a new decoder.
    ///
    /// `channel_gains[i]` corresponds to `users[i]`.
    pub fn new(users: Vec<UserConfig>, channel_gains: Vec<f64>) -> Self {
        assert_eq!(users.len(), channel_gains.len());
        assert!(!users.is_empty() && users.len() <= 4);
        Self {
            users,
            channel_gains,
        }
    }

    /// Perform successive interference cancellation.
    ///
    /// `num_symbols` is the number of symbols each user transmitted.
    pub fn decode_sic(&self, received: &[(f64, f64)], num_symbols: usize) -> SicResult {
        assert!(received.len() >= num_symbols);

        // Determine decoding order: strongest effective power first.
        let order = order_users(&self.users, &self.channel_gains);

        let mut residual: Vec<(f64, f64)> = received[..num_symbols].to_vec();
        let mut decoded_symbols: Vec<Vec<(f64, f64)>> = Vec::with_capacity(order.len());
        let mut user_order: Vec<usize> = Vec::with_capacity(order.len());

        for &idx in &order {
            let cfg = &self.users[idx];
            let gain = self.channel_gains[idx];
            let amp = cfg.power_fraction.sqrt() * gain;

            // Slice decision on the residual.
            let decoded: Vec<(f64, f64)> = residual
                .iter()
                .map(|s| {
                    // Scale received sample to unit constellation
                    let scaled = if amp.abs() > 1e-15 {
                        (s.0 / amp, s.1 / amp)
                    } else {
                        *s
                    };
                    hard_decide(scaled, cfg.modulation)
                })
                .collect();

            // Reconstruct and subtract.
            for (k, sym) in decoded.iter().enumerate() {
                residual[k].0 -= amp * sym.0;
                residual[k].1 -= amp * sym.1;
            }

            decoded_symbols.push(decoded);
            user_order.push(cfg.user_id);
        }

        // Residual interference power.
        let residual_interference: f64 =
            residual.iter().map(|s| s.0 * s.0 + s.1 * s.1).sum::<f64>() / num_symbols as f64;

        let per_user_ber = vec![0.0; order.len()];

        SicResult {
            decoded_symbols,
            user_order,
            per_user_ber,
            residual_interference,
        }
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Order users by effective received power (strongest first).
///
/// Effective power = `channel_gain[i]^2 * power_fraction[i]`.
pub fn order_users(users: &[UserConfig], channel_gains: &[f64]) -> Vec<usize> {
    let mut indices: Vec<usize> = (0..users.len()).collect();
    indices.sort_by(|&a, &b| {
        let pa = channel_gains[a].powi(2) * users[a].power_fraction;
        let pb = channel_gains[b].powi(2) * users[b].power_fraction;
        pb.partial_cmp(&pa).unwrap_or(std::cmp::Ordering::Equal)
    });
    indices
}

/// Compute optimal power fractions for `n` users using a simple
/// gain-inversive allocation.
///
/// Users with *weaker* channels receive *more* power.  `channel_gains` are
/// linear magnitudes.  Returns a vector of power fractions summing to 1.
pub fn power_allocation(channel_gains: &[f64]) -> Vec<f64> {
    assert!(!channel_gains.is_empty() && channel_gains.len() <= 4);
    // Inverse-gain allocation: weight_i = 1 / |h_i|^2
    let inv_gains: Vec<f64> = channel_gains
        .iter()
        .map(|&g| {
            let g2 = g * g;
            if g2 > 1e-30 { 1.0 / g2 } else { 1e30 }
        })
        .collect();
    let total: f64 = inv_gains.iter().sum();
    inv_gains.iter().map(|&w| w / total).collect()
}

/// Compute per-user SINR accounting for inter-user interference under SIC.
///
/// Under standard NOMA SIC, a user decoded at position *k* sees interference
/// only from users decoded *after* it (those with weaker effective power).
/// `noise_power` is the additive noise variance (sigma squared).
///
/// Returns SINR in **linear** scale for each user in SIC decoding order.
pub fn sinr_per_user(
    users: &[UserConfig],
    channel_gains: &[f64],
    noise_power: f64,
) -> Vec<(usize, f64)> {
    let order = order_users(users, channel_gains);
    let mut results = Vec::with_capacity(order.len());

    for (pos, &idx) in order.iter().enumerate() {
        let signal = channel_gains[idx].powi(2) * users[idx].power_fraction;

        // Interference from users decoded later (weaker).
        let interference: f64 = order[pos + 1..]
            .iter()
            .map(|&j| channel_gains[j].powi(2) * users[j].power_fraction)
            .sum();

        let sinr = signal / (interference + noise_power);
        results.push((users[idx].user_id, sinr));
    }

    results
}

/// Compute the achievable rate region boundary for a 2-user NOMA system.
///
/// Returns a vector of `(R1, R2)` pairs (bits/s/Hz) tracing the boundary.
/// `snr` is the total transmit SNR (linear) and `h` = `[h1, h2]` are
/// channel gain magnitudes.  Assumes `|h1| >= |h2|` (user 1 is the strong
/// user).
pub fn capacity_region(h: [f64; 2], snr: f64, num_points: usize) -> Vec<(f64, f64)> {
    let mut points = Vec::with_capacity(num_points);

    for i in 0..num_points {
        // alpha = fraction of power to user 2 (the weak user)
        let alpha = i as f64 / (num_points - 1).max(1) as f64;

        // User 1 (strong) is decoded second under SIC (after removing user 2).
        // R1 = log2(1 + |h1|^2 * (1-alpha) * SNR)
        let r1 = (1.0 + h[0].powi(2) * (1.0 - alpha) * snr).ln() / 2.0_f64.ln();

        // User 2 (weak) is decoded first, treating user 1 as noise.
        // R2 = log2(1 + |h2|^2 * alpha * SNR / (|h2|^2 * (1-alpha) * SNR + 1))
        let num = h[1].powi(2) * alpha * snr;
        let den = h[1].powi(2) * (1.0 - alpha) * snr + 1.0;
        let r2 = (1.0 + num / den).ln() / 2.0_f64.ln();

        points.push((r1, r2));
    }

    points
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Hard-decision slicer for BPSK or QPSK constellation.
fn hard_decide(sample: (f64, f64), modulation: Modulation) -> (f64, f64) {
    match modulation {
        Modulation::Bpsk => {
            // Constellation: +1, -1 on real axis.
            let re = if sample.0 >= 0.0 { 1.0 } else { -1.0 };
            (re, 0.0)
        }
        Modulation::Qpsk => {
            // Constellation: (+-1/sqrt2, +-1/sqrt2)
            let v = 1.0 / 2.0_f64.sqrt();
            let re = if sample.0 >= 0.0 { v } else { -v };
            let im = if sample.1 >= 0.0 { v } else { -v };
            (re, im)
        }
    }
}

/// Add AWGN to a signal (deterministic seeded PRNG for reproducibility).
#[cfg(test)]
fn add_awgn(signal: &[(f64, f64)], noise_std: f64, seed: u64) -> Vec<(f64, f64)> {
    // Simple xorshift64 PRNG + Box-Muller for Gaussian samples.
    let mut state = seed ^ 0xDEAD_BEEF_CAFE_BABE;
    let mut next_u64 = move || -> u64 {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    let mut next_f64 = || -> f64 { (next_u64() as f64) / (u64::MAX as f64) };

    signal
        .iter()
        .map(|s| {
            // Box-Muller
            let u1 = next_f64().max(1e-15);
            let u2 = next_f64();
            let r = (-2.0 * u1.ln()).sqrt();
            let theta = 2.0 * PI * u2;
            let n_re = r * theta.cos() * noise_std;
            let n_im = r * theta.sin() * noise_std;
            (s.0 + n_re, s.1 + n_im)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: two BPSK users with default power split.
    fn two_user_bpsk() -> (Vec<UserConfig>, Vec<Vec<(f64, f64)>>) {
        let users = vec![
            UserConfig { user_id: 0, power_fraction: 0.8, modulation: Modulation::Bpsk },
            UserConfig { user_id: 1, power_fraction: 0.2, modulation: Modulation::Bpsk },
        ];
        let symbols = vec![
            vec![(1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (-1.0, 0.0)],
            vec![(1.0, 0.0), (1.0, 0.0), (-1.0, 0.0), (-1.0, 0.0)],
        ];
        (users, symbols)
    }

    #[test]
    fn test_encode_two_bpsk_users() {
        let (users, symbols) = two_user_bpsk();
        let encoder = NomaEncoder::new(users);
        let composite = encoder.encode(&symbols);

        // First symbol: sqrt(0.8)*1 + sqrt(0.2)*1
        let expected_0 = 0.8_f64.sqrt() + 0.2_f64.sqrt();
        assert!((composite[0].0 - expected_0).abs() < 1e-12);
        assert!(composite[0].1.abs() < 1e-12);

        // Second symbol: sqrt(0.8)*(-1) + sqrt(0.2)*(1)
        let expected_1 = -0.8_f64.sqrt() + 0.2_f64.sqrt();
        assert!((composite[1].0 - expected_1).abs() < 1e-12);
    }

    #[test]
    fn test_encode_length_preserved() {
        let (users, symbols) = two_user_bpsk();
        let encoder = NomaEncoder::new(users);
        let composite = encoder.encode(&symbols);
        assert_eq!(composite.len(), 4);
    }

    #[test]
    fn test_sic_decode_noiseless() {
        let (users, symbols) = two_user_bpsk();
        let encoder = NomaEncoder::new(users.clone());
        let composite = encoder.encode(&symbols);

        // Equal channel gains - SIC order determined by power fraction.
        let channel_gains = vec![1.0, 1.0];
        let decoder = NomaDecoder::new(users, channel_gains);
        let result = decoder.decode_sic(&composite, 4);

        // User 0 has higher power so is decoded first.
        assert_eq!(result.user_order[0], 0);
        assert_eq!(result.user_order[1], 1);

        // Verify decoded symbols match originals.
        for k in 0..4 {
            assert!(
                (result.decoded_symbols[0][k].0 - symbols[0][k].0).abs() < 1e-9,
                "User 0 symbol {k} mismatch"
            );
            assert!(
                (result.decoded_symbols[1][k].0 - symbols[1][k].0).abs() < 1e-9,
                "User 1 symbol {k} mismatch"
            );
        }
    }

    #[test]
    fn test_residual_interference_noiseless() {
        let (users, symbols) = two_user_bpsk();
        let encoder = NomaEncoder::new(users.clone());
        let composite = encoder.encode(&symbols);

        let decoder = NomaDecoder::new(users, vec![1.0, 1.0]);
        let result = decoder.decode_sic(&composite, 4);
        assert!(
            result.residual_interference < 1e-20,
            "Residual should be ~0 in noiseless case, got {}",
            result.residual_interference
        );
    }

    #[test]
    fn test_order_users_by_effective_power() {
        let users = vec![
            UserConfig { user_id: 0, power_fraction: 0.8, modulation: Modulation::Bpsk },
            UserConfig { user_id: 1, power_fraction: 0.2, modulation: Modulation::Bpsk },
        ];
        // User 1 has much higher channel gain -> stronger effective power.
        let gains = vec![0.5, 3.0];
        let order = order_users(&users, &gains);
        // Effective: user0 = 0.25*0.8 = 0.2, user1 = 9*0.2 = 1.8
        assert_eq!(order, vec![1, 0]);
    }

    #[test]
    fn test_power_allocation_weaker_gets_more() {
        let gains = vec![2.0, 0.5];
        let alloc = power_allocation(&gains);
        // Weaker channel (user 1, gain 0.5) should get more power.
        assert!(alloc[1] > alloc[0], "Weaker user should get more power");
        assert!((alloc[0] + alloc[1] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_power_allocation_three_users() {
        let gains = vec![3.0, 1.0, 0.5];
        let alloc = power_allocation(&gains);
        assert!((alloc.iter().sum::<f64>() - 1.0).abs() < 1e-12);
        // Weakest channel gets the most power.
        assert!(alloc[2] > alloc[1]);
        assert!(alloc[1] > alloc[0]);
    }

    #[test]
    fn test_sinr_per_user_two_users() {
        let users = vec![
            UserConfig { user_id: 0, power_fraction: 0.8, modulation: Modulation::Bpsk },
            UserConfig { user_id: 1, power_fraction: 0.2, modulation: Modulation::Bpsk },
        ];
        let gains = vec![1.0, 1.0];
        let noise = 0.01;
        let sinrs = sinr_per_user(&users, &gains, noise);

        // User 0 decoded first (strongest), sees user 1 as interference.
        assert_eq!(sinrs[0].0, 0); // user_id 0
        let expected_sinr0 = 0.8 / (0.2 + 0.01);
        assert!((sinrs[0].1 - expected_sinr0).abs() < 1e-9);

        // User 1 decoded last, no remaining interference.
        assert_eq!(sinrs[1].0, 1);
        let expected_sinr1 = 0.2 / 0.01;
        assert!((sinrs[1].1 - expected_sinr1).abs() < 1e-9);
    }

    #[test]
    fn test_capacity_region_boundary() {
        let h = [2.0, 1.0];
        let snr = 10.0;
        let points = capacity_region(h, snr, 50);
        assert_eq!(points.len(), 50);

        // At alpha = 0 (all power to user 1): R2 = 0, R1 = max.
        let (r1_max, r2_min) = points[0];
        assert!(r2_min.abs() < 1e-12, "R2 should be 0 when alpha=0");
        assert!(r1_max > 0.0);

        // At alpha = 1 (all power to user 2): R1 = 0, R2 = max.
        let (r1_min, r2_max) = points[49];
        assert!(r1_min.abs() < 1e-12, "R1 should be 0 when alpha=1");
        assert!(r2_max > 0.0);

        // All rates should be non-negative.
        for (r1, r2) in &points {
            assert!(*r1 >= -1e-12 && *r2 >= -1e-12);
        }
    }

    #[test]
    fn test_qpsk_encode_decode_noiseless() {
        let v = 1.0 / 2.0_f64.sqrt();
        let users = vec![
            UserConfig { user_id: 0, power_fraction: 0.7, modulation: Modulation::Qpsk },
            UserConfig { user_id: 1, power_fraction: 0.3, modulation: Modulation::Qpsk },
        ];
        let symbols = vec![
            vec![(v, v), (-v, v), (-v, -v), (v, -v)],
            vec![(v, -v), (v, v), (-v, v), (-v, -v)],
        ];

        let encoder = NomaEncoder::new(users.clone());
        let composite = encoder.encode(&symbols);

        let decoder = NomaDecoder::new(users, vec![1.0, 1.0]);
        let result = decoder.decode_sic(&composite, 4);

        // User 0 decoded first.
        for k in 0..4 {
            assert!(
                (result.decoded_symbols[0][k].0 - symbols[0][k].0).abs() < 1e-9,
                "QPSK user 0 re mismatch at {k}"
            );
            assert!(
                (result.decoded_symbols[0][k].1 - symbols[0][k].1).abs() < 1e-9,
                "QPSK user 0 im mismatch at {k}"
            );
        }
        // User 1 after cancellation.
        for k in 0..4 {
            assert!(
                (result.decoded_symbols[1][k].0 - symbols[1][k].0).abs() < 1e-9,
                "QPSK user 1 re mismatch at {k}"
            );
            assert!(
                (result.decoded_symbols[1][k].1 - symbols[1][k].1).abs() < 1e-9,
                "QPSK user 1 im mismatch at {k}"
            );
        }
    }

    #[test]
    fn test_four_user_noiseless() {
        // Use well-separated power fractions so SIC can resolve all users.
        // The strongest user must have enough margin over the sum of weaker
        // users to allow correct hard decisions at every cancellation stage.
        let users = vec![
            UserConfig { user_id: 0, power_fraction: 0.50, modulation: Modulation::Bpsk },
            UserConfig { user_id: 1, power_fraction: 0.25, modulation: Modulation::Bpsk },
            UserConfig { user_id: 2, power_fraction: 0.15, modulation: Modulation::Bpsk },
            UserConfig { user_id: 3, power_fraction: 0.10, modulation: Modulation::Bpsk },
        ];
        let symbols = vec![
            vec![(1.0, 0.0), (-1.0, 0.0)],
            vec![(1.0, 0.0), (1.0, 0.0)],
            vec![(1.0, 0.0), (-1.0, 0.0)],
            vec![(1.0, 0.0), (1.0, 0.0)],
        ];

        let encoder = NomaEncoder::new(users.clone());
        let composite = encoder.encode(&symbols);

        let decoder = NomaDecoder::new(users.clone(), vec![1.0, 1.0, 1.0, 1.0]);
        let result = decoder.decode_sic(&composite, 2);

        assert_eq!(result.decoded_symbols.len(), 4);
        assert!(
            result.residual_interference < 1e-20,
            "Residual should be ~0, got {}",
            result.residual_interference
        );

        // Verify each user's symbols match (order may differ from input).
        for (pos, &uid) in result.user_order.iter().enumerate() {
            for k in 0..2 {
                assert!(
                    (result.decoded_symbols[pos][k].0 - symbols[uid][k].0).abs() < 1e-9,
                    "4-user: user {uid} symbol {k} mismatch"
                );
            }
        }
    }

    #[test]
    fn test_sic_with_awgn_low_noise() {
        let (users, symbols) = two_user_bpsk();
        let encoder = NomaEncoder::new(users.clone());
        let composite = encoder.encode(&symbols);

        // Add very low noise - should still decode correctly.
        let noisy = add_awgn(&composite, 0.01, 42);

        let decoder = NomaDecoder::new(users, vec![1.0, 1.0]);
        let result = decoder.decode_sic(&noisy, 4);

        // At SNR ~ 40 dB, BPSK should decode perfectly.
        for k in 0..4 {
            assert!(
                (result.decoded_symbols[0][k].0 - symbols[0][k].0).abs() < 1e-6,
                "Low-noise: user 0 symbol {k} wrong"
            );
            assert!(
                (result.decoded_symbols[1][k].0 - symbols[1][k].0).abs() < 1e-6,
                "Low-noise: user 1 symbol {k} wrong"
            );
        }
    }

    #[test]
    fn test_hard_decide_bpsk() {
        assert_eq!(hard_decide((0.5, 0.3), Modulation::Bpsk), (1.0, 0.0));
        assert_eq!(hard_decide((-0.1, -2.0), Modulation::Bpsk), (-1.0, 0.0));
        assert_eq!(hard_decide((0.0, 0.0), Modulation::Bpsk), (1.0, 0.0)); // boundary
    }

    #[test]
    fn test_hard_decide_qpsk() {
        let v = 1.0 / 2.0_f64.sqrt();
        assert_eq!(hard_decide((0.3, 0.3), Modulation::Qpsk), (v, v));
        assert_eq!(hard_decide((-0.1, 0.9), Modulation::Qpsk), (-v, v));
        assert_eq!(hard_decide((-0.5, -0.5), Modulation::Qpsk), (-v, -v));
        assert_eq!(hard_decide((0.8, -0.2), Modulation::Qpsk), (v, -v));
    }

    #[test]
    #[should_panic(expected = "At most 4")]
    fn test_encoder_rejects_five_users() {
        let users: Vec<UserConfig> = (0..5)
            .map(|i| UserConfig {
                user_id: i,
                power_fraction: 0.2,
                modulation: Modulation::Bpsk,
            })
            .collect();
        let _ = NomaEncoder::new(users);
    }

    #[test]
    #[should_panic(expected = "sum to 1.0")]
    fn test_encoder_rejects_bad_power_sum() {
        let users = vec![
            UserConfig { user_id: 0, power_fraction: 0.5, modulation: Modulation::Bpsk },
            UserConfig { user_id: 1, power_fraction: 0.3, modulation: Modulation::Bpsk },
        ];
        let _ = NomaEncoder::new(users);
    }
}

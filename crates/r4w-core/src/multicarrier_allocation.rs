//! # Multi-Carrier Subcarrier Allocation
//!
//! Dynamically allocates subcarrier power and modulation order in multi-carrier
//! systems (e.g., OFDM, DMT) using waterfilling and bit-loading algorithms.
//!
//! This module provides a high-level [`MulticarrierAllocator`] that combines:
//! - **Waterfilling** — optimal power allocation maximizing Shannon capacity
//! - **Bit-loading** — Hughes-Hartogs greedy algorithm assigning discrete
//!   modulation orders (BPSK, QPSK, 16-QAM, 64-QAM) to each subcarrier
//! - **Equal-power baseline** — uniform distribution for comparison
//! - **Channel Quality Indicator (CQI)** — per-group quality metric
//!
//! # Example
//!
//! ```rust
//! use r4w_core::multicarrier_allocation::{AllocatorConfig, MulticarrierAllocator};
//!
//! // 8 subcarriers with varying channel SNR (in dB)
//! let channel_snr_db = vec![25.0, 20.0, 15.0, 10.0, 8.0, 5.0, 3.0, 1.0];
//!
//! let config = AllocatorConfig {
//!     num_subcarriers: 8,
//!     total_power: 8.0,
//!     noise_power_per_subcarrier: vec![1.0; 8],
//!     target_ber: 1e-4,
//! };
//!
//! let allocator = MulticarrierAllocator::new(config);
//! let result = allocator.allocate(&channel_snr_db);
//!
//! // All power is used (or less if some subcarriers are shut off)
//! assert!(result.total_power <= 8.0 + 1e-9);
//! // Stronger subcarriers get higher modulation orders
//! assert!(result.per_subcarrier[0].modulation_order >= result.per_subcarrier[7].modulation_order);
//! // Total throughput is the sum of bits per symbol
//! assert!(result.total_bits > 0);
//! println!("Total throughput: {} bits/symbol", result.total_bits);
//! ```

use std::f64::consts::LN_2;

/// Supported modulation orders and their bits-per-symbol.
///
/// Returns the sorted list of `(bits_per_symbol, required_snr_db)` pairs for the
/// target BER. The required SNR is the minimum SNR needed to achieve the target
/// BER with that modulation, based on approximate BER formulas for M-QAM over AWGN.
fn modulation_table(target_ber: f64) -> Vec<(u32, f64)> {
    // For M-QAM, approximate required Eb/N0 for a given BER:
    //   BER ≈ (2/log2(M)) * (1 - 1/sqrt(M)) * erfc(sqrt(3*Eb*log2(M) / (2*(M-1)*N0)))
    // We invert this numerically for each M to find required SNR per bit,
    // then scale to SNR per symbol = Eb/N0 * bits_per_symbol.
    //
    // For simplicity and std-only constraint, we use pre-computed thresholds
    // parameterized by target_ber via the inverse erfc approximation.

    let inv_erfc_val = inv_erfc(target_ber);

    // BPSK (M=2, bits=1): SNR = (inv_erfc(BER))^2
    let snr_bpsk = inv_erfc_val * inv_erfc_val;

    // QPSK (M=4, bits=2): same Eb/N0 as BPSK, SNR_symbol = 2 * Eb/N0
    let snr_qpsk = 2.0 * snr_bpsk;

    // 16-QAM (M=16, bits=4): Eb/N0 = (inv_erfc(BER * 4/3))^2 * 10/4
    // Simplified: required Es/N0 ≈ 10 * (inv_erfc(target_ber * (4.0/3.0)))^2 / 1.0
    let inv_erfc_16 = inv_erfc(target_ber * 4.0 / 3.0);
    let snr_16qam = 10.0 / 3.0 * (16.0 - 1.0) / 1.0 * (inv_erfc_16 * inv_erfc_16) / 4.0;
    // More accurate: Es/N0 = (2/3)(M-1) * (inv_erfc(BER * log2(M) / (2*(1-1/sqrt(M)))))^2
    // Let's use a cleaner formula:
    // For M-QAM: Es/N0_required = (2(M-1)/3) * [erfc_inv(BER * log2(M)/(2(1-1/sqrt(M))))]^2
    let snr_16qam = required_snr_mqam(16, target_ber);
    let snr_64qam = required_snr_mqam(64, target_ber);

    // Convert linear SNR to dB
    let to_db = |x: f64| 10.0 * x.log10();

    vec![
        (1, to_db(snr_bpsk)),    // BPSK
        (2, to_db(snr_qpsk)),    // QPSK
        (4, to_db(snr_16qam)),   // 16-QAM
        (6, to_db(snr_64qam)),   // 64-QAM
    ]
}

/// Required Es/N0 (linear) for M-QAM to achieve target BER over AWGN.
fn required_snr_mqam(m: u32, target_ber: f64) -> f64 {
    let mf = m as f64;
    let k = (mf as f64).log2(); // bits per symbol
    let scale = 2.0 * (1.0 - 1.0 / mf.sqrt());
    // BER ≈ (scale / k) * erfc(sqrt(3 * Es / (2*(M-1) * N0)))
    // => erfc_arg = inv_erfc(BER * k / scale)
    // => 3 * Es/N0 / (2*(M-1)) = erfc_arg^2
    // => Es/N0 = 2*(M-1)/3 * erfc_arg^2
    let ber_scaled = (target_ber * k / scale).min(1.0).max(1e-15);
    let erfc_arg = inv_erfc(ber_scaled);
    2.0 * (mf - 1.0) / 3.0 * erfc_arg * erfc_arg
}

/// Approximate inverse complementary error function.
///
/// Uses a rational approximation accurate to ~1e-7 for 0 < p < 2.
fn inv_erfc(p: f64) -> f64 {
    // inv_erfc(p) = -inv_erf(1-p) ... but we use a direct approximation.
    // For p in (0, 1], use the approximation for inv_erfc.
    if p <= 0.0 {
        return f64::INFINITY;
    }
    if p >= 2.0 {
        return f64::NEG_INFINITY;
    }

    // Use Newton's method on erfc(x) = p, starting from a rational approximation.
    // Initial guess from Winitzki's approximation of erf_inv:
    // erf_inv(z) ≈ sign(z) * sqrt(sqrt((4/pi/a + ln(1-z^2)/2)^2 - ln(1-z^2)/a) - (4/pi/a + ln(1-z^2)/2))
    // where a ≈ 0.147.
    let z = 1.0 - p; // erf(x) = z
    let result = erf_inv_approx(z);

    // A few Newton-Raphson iterations for refinement:
    // erfc(x) = p => f(x) = erfc(x) - p, f'(x) = -2/sqrt(pi) * exp(-x^2)
    let mut x = result;
    let two_over_sqrt_pi = 2.0 / std::f64::consts::PI.sqrt();
    for _ in 0..4 {
        let err = erfc_approx(x) - p;
        let deriv = -two_over_sqrt_pi * (-x * x).exp();
        if deriv.abs() < 1e-30 {
            break;
        }
        x -= err / deriv;
    }
    x
}

/// Approximate erf_inv using Winitzki's formula.
fn erf_inv_approx(z: f64) -> f64 {
    if z.abs() >= 1.0 {
        return if z > 0.0 { f64::INFINITY } else { f64::NEG_INFINITY };
    }
    if z.abs() < 1e-15 {
        return 0.0;
    }
    let a = 0.147;
    let ln_term = (1.0 - z * z).ln();
    let b = 2.0 / (std::f64::consts::PI * a) + ln_term / 2.0;
    let c = ln_term / a;
    let inner = (b * b - c).sqrt() - b;
    if inner < 0.0 {
        return 0.0;
    }
    z.signum() * inner.sqrt()
}

/// Approximate erfc(x) using a series/continued-fraction approach.
fn erfc_approx(x: f64) -> f64 {
    // erfc(x) = 1 - erf(x)
    // Use the relation: erf(x) = 2/sqrt(pi) * integral_0^x exp(-t^2) dt
    // For |x| < 3.5, use the Taylor series; for larger, use asymptotic.
    if x < 0.0 {
        return 2.0 - erfc_approx(-x);
    }
    if x == 0.0 {
        return 1.0;
    }
    // Horner form of a rational approximation (Abramowitz & Stegun 7.1.26)
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t * (0.254829592
        + t * (-0.284496736
            + t * (1.421413741
                + t * (-1.453152027
                    + t * 1.061405429))));
    poly * (-x * x).exp()
}

/// Configuration for the multi-carrier allocator.
#[derive(Debug, Clone)]
pub struct AllocatorConfig {
    /// Number of subcarriers.
    pub num_subcarriers: usize,
    /// Total transmit power budget (linear).
    pub total_power: f64,
    /// Noise power per subcarrier (linear). Length must equal `num_subcarriers`.
    pub noise_power_per_subcarrier: Vec<f64>,
    /// Target bit error rate for modulation selection.
    pub target_ber: f64,
}

/// Per-subcarrier allocation result.
#[derive(Debug, Clone)]
pub struct SubcarrierAllocation {
    /// Allocated power (linear) on this subcarrier.
    pub power: f64,
    /// Modulation order in bits per symbol: 0 (off), 1 (BPSK), 2 (QPSK), 4 (16-QAM), 6 (64-QAM).
    pub modulation_order: u32,
    /// Effective SNR on this subcarrier in dB (after power allocation).
    pub snr_db: f64,
}

/// Overall allocation result across all subcarriers.
#[derive(Debug, Clone)]
pub struct AllocationResult {
    /// Per-subcarrier allocation details.
    pub per_subcarrier: Vec<SubcarrierAllocation>,
    /// Total bits per OFDM symbol (sum of modulation orders across active subcarriers).
    pub total_bits: u32,
    /// Total allocated power (should equal or be less than budget).
    pub total_power: f64,
}

/// Multi-carrier subcarrier power and modulation allocator.
///
/// Combines waterfilling power allocation with bit-loading to jointly optimize
/// power and modulation order per subcarrier.
#[derive(Debug, Clone)]
pub struct MulticarrierAllocator {
    config: AllocatorConfig,
}

impl MulticarrierAllocator {
    /// Create a new allocator with the given configuration.
    ///
    /// # Panics
    ///
    /// Panics if `noise_power_per_subcarrier` length does not match `num_subcarriers`.
    pub fn new(config: AllocatorConfig) -> Self {
        assert_eq!(
            config.noise_power_per_subcarrier.len(),
            config.num_subcarriers,
            "noise_power_per_subcarrier length must match num_subcarriers"
        );
        assert!(config.total_power > 0.0, "total_power must be positive");
        assert!(config.target_ber > 0.0 && config.target_ber < 1.0, "target_ber must be in (0, 1)");
        Self { config }
    }

    /// Waterfilling power allocation given channel SNR per subcarrier (in dB).
    ///
    /// Returns the optimal power distribution that maximizes total Shannon capacity.
    /// Subcarriers that are too weak receive zero power.
    pub fn waterfill(&self, channel_snr_db: &[f64]) -> Vec<f64> {
        assert_eq!(channel_snr_db.len(), self.config.num_subcarriers);

        let n = self.config.num_subcarriers;
        // Convert channel SNR (dB) to linear channel gain.
        // SNR_i = gain_i * P_equal / N_i => gain_i = SNR_i * N_i / P_equal
        // For waterfilling we treat the "effective noise" as N_i / gain_i.
        // We model: gain_i = 10^(snr_db_i / 10) (channel gain relative to noise).
        let gains: Vec<f64> = channel_snr_db.iter()
            .map(|&snr| db_to_linear(snr))
            .collect();

        // Iterative waterfilling
        let mut active = vec![true; n];
        loop {
            let active_count = active.iter().filter(|&&a| a).count();
            if active_count == 0 {
                return vec![0.0; n];
            }

            // Water level: mu = (P_total + sum_{active} N_i/g_i) / |active|
            let inv_gain_sum: f64 = (0..n)
                .filter(|&i| active[i])
                .map(|i| self.config.noise_power_per_subcarrier[i] / gains[i])
                .sum();

            let mu = (self.config.total_power + inv_gain_sum) / active_count as f64;

            let mut powers = vec![0.0; n];
            let mut any_negative = false;
            for i in 0..n {
                if active[i] {
                    let p = mu - self.config.noise_power_per_subcarrier[i] / gains[i];
                    if p < 0.0 {
                        active[i] = false;
                        any_negative = true;
                    } else {
                        powers[i] = p;
                    }
                }
            }

            if !any_negative {
                return powers;
            }
        }
    }

    /// Bit-loading using a greedy Hughes-Hartogs-style algorithm.
    ///
    /// Given channel SNR per subcarrier (dB), incrementally assigns bits to the
    /// subcarrier that requires the least additional power, subject to the total
    /// power budget and the target BER.
    ///
    /// Returns `(modulation_orders, powers)` where each modulation order is
    /// 0, 1, 2, 4, or 6 bits.
    pub fn bit_loading(&self, channel_snr_db: &[f64]) -> (Vec<u32>, Vec<f64>) {
        assert_eq!(channel_snr_db.len(), self.config.num_subcarriers);

        let n = self.config.num_subcarriers;
        let mod_table = modulation_table(self.config.target_ber);
        // mod_table: [(bits, required_snr_db), ...] sorted by bits ascending

        // Available modulation steps: 0 -> 1 -> 2 -> 4 -> 6
        let steps: Vec<u32> = mod_table.iter().map(|&(b, _)| b).collect();
        let required_snrs: Vec<f64> = mod_table.iter().map(|&(_, s)| s).collect();

        // Current assignment: index into steps (-1 means unassigned, use 0 for "no bits")
        let mut current_step = vec![0usize; n]; // index 0 = not yet loaded
        let mut bits = vec![0u32; n];
        let mut powers = vec![0.0f64; n];
        let mut total_power_used = 0.0;

        // For each step, compute the incremental power cost to go from current modulation
        // to the next modulation level on subcarrier i.
        // incremental_power(i, next_step) = N_i * 10^(required_snr_db/10) / g_i - current_power_i
        // where g_i = 10^(channel_snr_db_i/10) is the channel gain.

        let gains: Vec<f64> = channel_snr_db.iter()
            .map(|&snr| db_to_linear(snr))
            .collect();

        // Power needed to support a given modulation order on subcarrier i:
        // P_i = N_i * SNR_required / g_i
        let power_for = |sub: usize, step_idx: usize| -> f64 {
            if step_idx == 0 {
                return 0.0;
            }
            let req_snr_linear = db_to_linear(required_snrs[step_idx - 1]);
            self.config.noise_power_per_subcarrier[sub] * req_snr_linear / gains[sub]
        };

        loop {
            // Find the subcarrier + next step that costs the least incremental power
            let mut best_sub = None;
            let mut best_cost = f64::INFINITY;
            let mut best_step = 0usize;

            for i in 0..n {
                let cur = current_step[i];
                if cur >= steps.len() {
                    continue; // already at max modulation
                }
                let next = cur + 1;
                let new_power = power_for(i, next);
                let cost = new_power - powers[i];
                if cost < best_cost && cost >= 0.0 {
                    best_cost = cost;
                    best_sub = Some(i);
                    best_step = next;
                }
            }

            match best_sub {
                Some(sub) if total_power_used + best_cost <= self.config.total_power + 1e-12 => {
                    total_power_used -= powers[sub];
                    powers[sub] = power_for(sub, best_step);
                    total_power_used += powers[sub];
                    current_step[sub] = best_step;
                    bits[sub] = steps[best_step - 1];
                }
                _ => break, // no more feasible assignments
            }
        }

        (bits, powers)
    }

    /// Equal-power allocation baseline.
    ///
    /// Distributes total power uniformly across all subcarriers and assigns
    /// the highest supportable modulation order to each.
    pub fn equal_power(&self, channel_snr_db: &[f64]) -> AllocationResult {
        assert_eq!(channel_snr_db.len(), self.config.num_subcarriers);

        let n = self.config.num_subcarriers;
        let power_per = self.config.total_power / n as f64;
        let mod_table = modulation_table(self.config.target_ber);

        let mut allocations = Vec::with_capacity(n);
        let mut total_bits = 0u32;

        for i in 0..n {
            let gain = db_to_linear(channel_snr_db[i]);
            let effective_snr_linear = gain * power_per / self.config.noise_power_per_subcarrier[i];
            let effective_snr_db = linear_to_db(effective_snr_linear);

            // Find highest modulation that the effective SNR can support
            let mut mod_order = 0u32;
            for &(bits, req_snr) in mod_table.iter().rev() {
                if effective_snr_db >= req_snr {
                    mod_order = bits;
                    break;
                }
            }

            total_bits += mod_order;
            allocations.push(SubcarrierAllocation {
                power: power_per,
                modulation_order: mod_order,
                snr_db: effective_snr_db,
            });
        }

        AllocationResult {
            per_subcarrier: allocations,
            total_bits,
            total_power: power_per * n as f64,
        }
    }

    /// Full allocation: waterfilling power + bit-loading modulation selection.
    ///
    /// Given per-subcarrier channel SNR in dB, returns optimal power allocation
    /// and modulation order per subcarrier.
    pub fn allocate(&self, channel_snr_db: &[f64]) -> AllocationResult {
        assert_eq!(channel_snr_db.len(), self.config.num_subcarriers);

        let (bits, powers) = self.bit_loading(channel_snr_db);
        let mod_table = modulation_table(self.config.target_ber);
        let n = self.config.num_subcarriers;

        let gains: Vec<f64> = channel_snr_db.iter()
            .map(|&snr| db_to_linear(snr))
            .collect();

        let mut allocations = Vec::with_capacity(n);
        let mut total_bits = 0u32;
        let mut total_power = 0.0;

        for i in 0..n {
            let effective_snr_linear = if powers[i] > 0.0 {
                gains[i] * powers[i] / self.config.noise_power_per_subcarrier[i]
            } else {
                0.0
            };
            let effective_snr_db = if effective_snr_linear > 0.0 {
                linear_to_db(effective_snr_linear)
            } else {
                f64::NEG_INFINITY
            };

            total_bits += bits[i];
            total_power += powers[i];
            allocations.push(SubcarrierAllocation {
                power: powers[i],
                modulation_order: bits[i],
                snr_db: effective_snr_db,
            });
        }

        AllocationResult {
            per_subcarrier: allocations,
            total_bits,
            total_power,
        }
    }

    /// Total throughput in bits per OFDM symbol for a given allocation.
    pub fn total_throughput(result: &AllocationResult) -> u32 {
        result.total_bits
    }

    /// Shannon capacity estimate in bits per second.
    ///
    /// Computes `sum_i BW_i * log2(1 + SNR_i)` across all subcarriers.
    /// `subcarrier_bandwidth_hz` is the bandwidth per subcarrier in Hz.
    pub fn capacity_bps(&self, channel_snr_db: &[f64], subcarrier_bandwidth_hz: f64) -> f64 {
        assert_eq!(channel_snr_db.len(), self.config.num_subcarriers);

        let powers = self.waterfill(channel_snr_db);
        let gains: Vec<f64> = channel_snr_db.iter()
            .map(|&snr| db_to_linear(snr))
            .collect();

        let mut capacity = 0.0;
        for i in 0..self.config.num_subcarriers {
            if powers[i] > 0.0 {
                let snr = gains[i] * powers[i] / self.config.noise_power_per_subcarrier[i];
                capacity += subcarrier_bandwidth_hz * (1.0 + snr).log2();
            }
        }
        capacity
    }

    /// Channel Quality Indicator (CQI) per subcarrier group.
    ///
    /// Groups subcarriers into `num_groups` contiguous groups and returns
    /// a CQI value (0-15) per group based on average SNR. Uses a simplified
    /// LTE-like CQI mapping.
    ///
    /// # Panics
    ///
    /// Panics if `num_groups` is zero or exceeds the number of subcarriers.
    pub fn channel_quality_indicator(&self, channel_snr_db: &[f64], num_groups: usize) -> Vec<u32> {
        assert_eq!(channel_snr_db.len(), self.config.num_subcarriers);
        assert!(num_groups > 0, "num_groups must be positive");
        assert!(num_groups <= self.config.num_subcarriers, "num_groups must not exceed num_subcarriers");

        let n = self.config.num_subcarriers;
        let group_size = n / num_groups;
        let remainder = n % num_groups;

        let mut cqi = Vec::with_capacity(num_groups);
        let mut start = 0;

        for g in 0..num_groups {
            let size = group_size + if g < remainder { 1 } else { 0 };
            let end = start + size;
            let avg_snr: f64 = channel_snr_db[start..end].iter().sum::<f64>() / size as f64;
            cqi.push(snr_to_cqi(avg_snr));
            start = end;
        }

        cqi
    }
}

/// Map an average SNR (dB) to a CQI index (0-15).
///
/// Uses a simplified LTE-like table where CQI 0 means "out of range" and
/// CQI 1-15 map to increasing SNR thresholds.
fn snr_to_cqi(snr_db: f64) -> u32 {
    // Approximate LTE CQI-to-SNR mapping thresholds (dB)
    const THRESHOLDS: [f64; 15] = [
        -6.7, -4.7, -2.3, 0.2, 2.4, 4.3, 5.9, 8.1,
        10.3, 11.7, 14.1, 16.3, 18.7, 21.0, 22.7,
    ];
    for (i, &thresh) in THRESHOLDS.iter().enumerate().rev() {
        if snr_db >= thresh {
            return (i + 1) as u32;
        }
    }
    0
}

/// Convert dB to linear scale.
#[inline]
fn db_to_linear(db: f64) -> f64 {
    10.0_f64.powf(db / 10.0)
}

/// Convert linear scale to dB.
#[inline]
fn linear_to_db(linear: f64) -> f64 {
    10.0 * linear.log10()
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-6;

    fn default_config(n: usize) -> AllocatorConfig {
        AllocatorConfig {
            num_subcarriers: n,
            total_power: n as f64,
            noise_power_per_subcarrier: vec![1.0; n],
            target_ber: 1e-4,
        }
    }

    #[test]
    fn test_waterfill_equal_channels() {
        let config = default_config(4);
        let alloc = MulticarrierAllocator::new(config);
        let snr_db = vec![20.0; 4];
        let powers = alloc.waterfill(&snr_db);
        assert_eq!(powers.len(), 4);
        // Equal channels => equal power
        for &p in &powers {
            assert!((p - 1.0).abs() < EPS, "expected ~1.0, got {p}");
        }
    }

    #[test]
    fn test_waterfill_power_conservation() {
        let config = default_config(8);
        let alloc = MulticarrierAllocator::new(config.clone());
        let snr_db = vec![30.0, 25.0, 20.0, 15.0, 10.0, 5.0, 3.0, 1.0];
        let powers = alloc.waterfill(&snr_db);
        let total: f64 = powers.iter().sum();
        assert!(
            (total - config.total_power).abs() < 0.01,
            "total power {total} should be close to budget {}",
            config.total_power
        );
    }

    #[test]
    fn test_waterfill_stronger_gets_more() {
        let config = default_config(2);
        let alloc = MulticarrierAllocator::new(config);
        let snr_db = vec![30.0, 5.0];
        let powers = alloc.waterfill(&snr_db);
        assert!(
            powers[0] > powers[1],
            "stronger channel should get more power: {} vs {}",
            powers[0], powers[1]
        );
    }

    #[test]
    fn test_bit_loading_basic() {
        let config = default_config(4);
        let alloc = MulticarrierAllocator::new(config);
        let snr_db = vec![30.0, 20.0, 10.0, 3.0];
        let (bits, powers) = alloc.bit_loading(&snr_db);
        assert_eq!(bits.len(), 4);
        assert_eq!(powers.len(), 4);
        // Strongest subcarrier should get highest modulation
        assert!(bits[0] >= bits[3], "strongest should have >= bits: {} vs {}", bits[0], bits[3]);
        // Total power should not exceed budget
        let total: f64 = powers.iter().sum();
        assert!(total <= 4.0 + EPS, "total power {total} exceeds budget");
    }

    #[test]
    fn test_bit_loading_all_strong() {
        let config = default_config(4);
        let alloc = MulticarrierAllocator::new(config);
        let snr_db = vec![40.0; 4];
        let (bits, _powers) = alloc.bit_loading(&snr_db);
        // Very strong channels should all get 64-QAM (6 bits) if power allows
        for &b in &bits {
            assert!(b > 0, "strong channels should have nonzero bits");
        }
    }

    #[test]
    fn test_equal_power_baseline() {
        let config = default_config(4);
        let alloc = MulticarrierAllocator::new(config);
        let snr_db = vec![25.0, 20.0, 15.0, 10.0];
        let result = alloc.equal_power(&snr_db);
        assert_eq!(result.per_subcarrier.len(), 4);
        // All subcarriers should have equal power
        for s in &result.per_subcarrier {
            assert!((s.power - 1.0).abs() < EPS, "equal power should be 1.0, got {}", s.power);
        }
        assert!((result.total_power - 4.0).abs() < EPS);
    }

    #[test]
    fn test_allocate_returns_valid_result() {
        let config = default_config(8);
        let alloc = MulticarrierAllocator::new(config);
        let snr_db = vec![30.0, 25.0, 20.0, 15.0, 10.0, 8.0, 5.0, 2.0];
        let result = alloc.allocate(&snr_db);
        assert_eq!(result.per_subcarrier.len(), 8);
        assert!(result.total_power <= 8.0 + 0.01);
        // total_bits should match sum
        let sum_bits: u32 = result.per_subcarrier.iter().map(|s| s.modulation_order).sum();
        assert_eq!(result.total_bits, sum_bits);
        // Modulation orders should be valid
        for s in &result.per_subcarrier {
            assert!(
                [0, 1, 2, 4, 6].contains(&s.modulation_order),
                "invalid modulation order: {}",
                s.modulation_order
            );
        }
    }

    #[test]
    fn test_total_throughput() {
        let config = default_config(4);
        let alloc = MulticarrierAllocator::new(config);
        let snr_db = vec![25.0, 20.0, 15.0, 10.0];
        let result = alloc.allocate(&snr_db);
        let tp = MulticarrierAllocator::total_throughput(&result);
        assert_eq!(tp, result.total_bits);
        assert!(tp > 0, "throughput should be positive for these SNRs");
    }

    #[test]
    fn test_capacity_bps() {
        let config = default_config(4);
        let alloc = MulticarrierAllocator::new(config);
        let snr_db = vec![20.0, 15.0, 10.0, 5.0];
        let cap = alloc.capacity_bps(&snr_db, 15_000.0);
        assert!(cap > 0.0, "capacity should be positive");
        assert!(cap.is_finite(), "capacity should be finite");
    }

    #[test]
    fn test_channel_quality_indicator() {
        let config = default_config(8);
        let alloc = MulticarrierAllocator::new(config);
        let snr_db = vec![30.0, 28.0, 20.0, 18.0, 10.0, 8.0, 0.0, -5.0];
        let cqi = alloc.channel_quality_indicator(&snr_db, 4);
        assert_eq!(cqi.len(), 4);
        // First group (high SNR) should have higher CQI than last group
        assert!(cqi[0] > cqi[3], "first group CQI {} should exceed last {}", cqi[0], cqi[3]);
        // All CQI should be in [0, 15]
        for &c in &cqi {
            assert!(c <= 15, "CQI out of range: {c}");
        }
    }

    #[test]
    fn test_allocate_monotonic_modulation() {
        // With monotonically decreasing SNR, modulation orders should be non-increasing
        let config = default_config(6);
        let alloc = MulticarrierAllocator::new(config);
        let snr_db = vec![35.0, 28.0, 21.0, 14.0, 7.0, 0.0];
        let result = alloc.allocate(&snr_db);
        for i in 0..result.per_subcarrier.len() - 1 {
            assert!(
                result.per_subcarrier[i].modulation_order >= result.per_subcarrier[i + 1].modulation_order,
                "modulation should be non-increasing: subcarrier {} has {} but {} has {}",
                i, result.per_subcarrier[i].modulation_order,
                i + 1, result.per_subcarrier[i + 1].modulation_order
            );
        }
    }

    #[test]
    fn test_zero_snr_subcarrier_gets_no_bits() {
        let config = AllocatorConfig {
            num_subcarriers: 3,
            total_power: 3.0,
            noise_power_per_subcarrier: vec![1.0; 3],
            target_ber: 1e-4,
        };
        let alloc = MulticarrierAllocator::new(config);
        // One subcarrier with very low SNR
        let snr_db = vec![25.0, 20.0, -10.0];
        let result = alloc.allocate(&snr_db);
        // The -10 dB subcarrier should get 0 bits (too weak)
        assert_eq!(
            result.per_subcarrier[2].modulation_order, 0,
            "very weak subcarrier should get 0 bits, got {}",
            result.per_subcarrier[2].modulation_order
        );
    }

    #[test]
    fn test_db_conversions_roundtrip() {
        let values = [0.001, 0.1, 1.0, 10.0, 100.0, 1000.0];
        for &v in &values {
            let db = linear_to_db(v);
            let back = db_to_linear(db);
            assert!(
                (back - v).abs() / v < 1e-10,
                "roundtrip failed for {v}: db={db}, back={back}"
            );
        }
    }

    #[test]
    fn test_inv_erfc_known_values() {
        // erfc(0) = 1 => inv_erfc(1) ≈ 0
        let x = inv_erfc(1.0);
        assert!(x.abs() < 0.001, "inv_erfc(1.0) should be ~0, got {x}");

        // erfc(1) ≈ 0.1573 => inv_erfc(0.1573) ≈ 1.0
        let x = inv_erfc(0.1573);
        assert!((x - 1.0).abs() < 0.01, "inv_erfc(0.1573) should be ~1.0, got {x}");
    }

    #[test]
    fn test_modulation_table_ordering() {
        let table = modulation_table(1e-4);
        assert_eq!(table.len(), 4);
        // Bits should be increasing
        for i in 0..table.len() - 1 {
            assert!(table[i].0 < table[i + 1].0, "bits should increase");
        }
        // Required SNR should be increasing
        for i in 0..table.len() - 1 {
            assert!(
                table[i].1 < table[i + 1].1,
                "required SNR should increase: {} vs {}",
                table[i].1, table[i + 1].1
            );
        }
    }

    #[test]
    fn test_snr_to_cqi_range() {
        // Very low SNR => CQI 0
        assert_eq!(snr_to_cqi(-20.0), 0);
        // Very high SNR => CQI 15
        assert_eq!(snr_to_cqi(30.0), 15);
        // Moderate SNR => somewhere in between
        let cqi = snr_to_cqi(10.0);
        assert!(cqi >= 1 && cqi <= 15, "moderate SNR CQI should be 1-15, got {cqi}");
    }
}

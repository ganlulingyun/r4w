//! # Water-Filling Power Allocation
//!
//! Implements the classic water-filling algorithm for optimal power allocation
//! across parallel channels, commonly used in MIMO eigenmodes, OFDM subcarriers,
//! and DSL tone loading.
//!
//! The water-filling solution maximizes total capacity under a sum-power constraint
//! by allocating more power to stronger channels and potentially shutting off weak
//! ones entirely. The algorithm iteratively computes a water level and removes
//! channels whose optimal allocation would be negative.
//!
//! Also provides:
//! - **Mercury/water-filling** with a coding gap for practical (non-ideal) coding
//! - **Inverse water-filling** for source coding (rate-distortion), which allocates
//!   more power to *weaker* channels
//! - A [`WaterfillingAllocator`] builder with minimum per-channel power constraints
//!
//! # Example
//!
//! ```rust
//! use r4w_core::waterfilling::{waterfill, capacity_with_allocation, equal_power};
//!
//! let gains = [10.0, 5.0, 1.0];
//! let total_power = 6.0;
//! let noise = 1.0;
//!
//! let powers = waterfill(&gains, total_power, noise);
//! let cap = capacity_with_allocation(&gains, &powers, noise);
//!
//! // Water-filling always achieves at least as much capacity as equal power
//! let eq = equal_power(gains.len(), total_power);
//! let cap_eq = capacity_with_allocation(&gains, &eq, noise);
//! assert!(cap >= cap_eq - 1e-12);
//! ```

/// Compute the optimal water-filling power allocation.
///
/// Given a set of parallel channels with gains `channel_gains`, a total power
/// budget `total_power`, and per-channel noise power `noise_power`, returns the
/// power allocated to each channel that maximizes the sum capacity
/// `sum_i log2(1 + g_i * p_i / N)`.
///
/// Channels whose optimal allocation would be negative (i.e., too weak to be
/// worth using) receive zero power. The algorithm iterates until only channels
/// with positive allocation remain.
///
/// # Panics
///
/// Panics if `total_power` or `noise_power` is negative, or if `noise_power` is zero.
pub fn waterfill(channel_gains: &[f64], total_power: f64, noise_power: f64) -> Vec<f64> {
    assert!(total_power >= 0.0, "total_power must be non-negative");
    assert!(noise_power > 0.0, "noise_power must be positive");

    if channel_gains.is_empty() {
        return vec![];
    }

    let n = channel_gains.len();
    // active[i] = true means channel i is still in the allocation set
    let mut active = vec![true; n];

    loop {
        let active_count = active.iter().filter(|&&a| a).count();
        if active_count == 0 {
            return vec![0.0; n];
        }

        // Water level: mu = (P_total + sum_{active} N/g_i) / |active|
        let inv_gain_sum: f64 = (0..n)
            .filter(|&i| active[i])
            .map(|i| noise_power / channel_gains[i])
            .sum();

        let mu = (total_power + inv_gain_sum) / active_count as f64;

        // Compute per-channel allocation: p_i = mu - N/g_i
        let mut powers = vec![0.0; n];
        let mut any_negative = false;
        for i in 0..n {
            if active[i] {
                let p = mu - noise_power / channel_gains[i];
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
        // Otherwise, loop again with the reduced active set
    }
}

/// Compute the water level (mu) for a set of channels.
///
/// The water level is the constant such that `p_i = max(0, mu - N/g_i)` and
/// `sum p_i = P_total`. This function returns the water level *before* removing
/// channels with negative allocation (i.e., using all channels as active).
/// For the iterative solution that removes zero-allocation channels, use
/// [`waterfill`] and derive mu from the result.
pub fn water_level(channel_gains: &[f64], total_power: f64, noise_power: f64) -> f64 {
    assert!(noise_power > 0.0, "noise_power must be positive");

    if channel_gains.is_empty() {
        return 0.0;
    }

    let n = channel_gains.len();
    let inv_gain_sum: f64 = channel_gains.iter().map(|&g| noise_power / g).sum();
    (total_power + inv_gain_sum) / n as f64
}

/// Compute the total capacity achieved by a given power allocation.
///
/// Returns `sum_i log2(1 + g_i * p_i / N)` in bits per channel use.
///
/// # Panics
///
/// Panics if `channel_gains` and `powers` have different lengths.
pub fn capacity_with_allocation(channel_gains: &[f64], powers: &[f64], noise_power: f64) -> f64 {
    assert_eq!(
        channel_gains.len(),
        powers.len(),
        "channel_gains and powers must have the same length"
    );

    channel_gains
        .iter()
        .zip(powers.iter())
        .map(|(&g, &p)| {
            if p <= 0.0 {
                0.0
            } else {
                (1.0 + g * p / noise_power).log2()
            }
        })
        .sum()
}

/// Equal power allocation baseline.
///
/// Distributes `total_power` uniformly across `num_channels` channels.
/// This is the simplest allocation strategy and serves as a lower bound
/// on capacity compared to water-filling.
pub fn equal_power(num_channels: usize, total_power: f64) -> Vec<f64> {
    if num_channels == 0 {
        return vec![];
    }
    vec![total_power / num_channels as f64; num_channels]
}

/// Mercury/water-filling with a coding gap.
///
/// In practical coded systems, the achievable rate is reduced by a gap factor
/// `Gamma` (in dB) relative to capacity. The effective SNR becomes
/// `g_i * p_i / (N * Gamma)` and the water level shifts accordingly.
///
/// A typical gap for a well-designed code is 0 dB (ideal) to ~9.8 dB
/// (uncoded QAM at 1e-6 BER). Turbo/LDPC codes achieve gaps of ~0.5-1.5 dB.
///
/// # Arguments
///
/// * `channel_gains` - per-channel gains (linear, not dB)
/// * `total_power` - total power budget
/// * `noise_power` - noise power per channel
/// * `gap_db` - coding gap in dB (0 = ideal Shannon)
pub fn mercury_waterfill(
    channel_gains: &[f64],
    total_power: f64,
    noise_power: f64,
    gap_db: f64,
) -> Vec<f64> {
    assert!(total_power >= 0.0, "total_power must be non-negative");
    assert!(noise_power > 0.0, "noise_power must be positive");

    if channel_gains.is_empty() {
        return vec![];
    }

    // Convert gap from dB to linear
    let gamma = 10.0_f64.powf(gap_db / 10.0);

    // Effective noise = N * Gamma per channel
    let effective_noise = noise_power * gamma;

    let n = channel_gains.len();
    let mut active = vec![true; n];

    loop {
        let active_count = active.iter().filter(|&&a| a).count();
        if active_count == 0 {
            return vec![0.0; n];
        }

        let inv_gain_sum: f64 = (0..n)
            .filter(|&i| active[i])
            .map(|i| effective_noise / channel_gains[i])
            .sum();

        let mu = (total_power + inv_gain_sum) / active_count as f64;

        let mut powers = vec![0.0; n];
        let mut any_negative = false;
        for i in 0..n {
            if active[i] {
                let p = mu - effective_noise / channel_gains[i];
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

/// Inverse water-filling for source coding (rate-distortion theory).
///
/// In source coding over parallel Gaussian channels, distortion is minimized
/// by allocating *more* power (representation rate) to *weaker* channels.
/// The allocation is `p_i = max(0, N/g_i - mu)`, where `mu` is chosen so
/// that `sum p_i = total_power`.
///
/// This is the dual of the channel-coding water-filling problem.
pub fn inverse_waterfill(
    channel_gains: &[f64],
    total_power: f64,
    noise_power: f64,
) -> Vec<f64> {
    assert!(total_power >= 0.0, "total_power must be non-negative");
    assert!(noise_power > 0.0, "noise_power must be positive");

    if channel_gains.is_empty() {
        return vec![];
    }

    let n = channel_gains.len();
    let mut active = vec![true; n];

    loop {
        let active_count = active.iter().filter(|&&a| a).count();
        if active_count == 0 {
            return vec![0.0; n];
        }

        // For inverse water-filling the water level is below the channel levels.
        // p_i = N/g_i - mu, sum p_i = P
        // sum (N/g_i - mu) = P  =>  mu = (sum N/g_i - P) / |active|
        let inv_gain_sum: f64 = (0..n)
            .filter(|&i| active[i])
            .map(|i| noise_power / channel_gains[i])
            .sum();

        let mu = (inv_gain_sum - total_power) / active_count as f64;

        let mut powers = vec![0.0; n];
        let mut any_negative = false;
        for i in 0..n {
            if active[i] {
                let p = noise_power / channel_gains[i] - mu;
                if p < 0.0 {
                    // This channel is too strong; it needs no allocation
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

/// Configurable water-filling allocator with minimum per-channel power constraints.
///
/// # Example
///
/// ```rust
/// use r4w_core::waterfilling::WaterfillingAllocator;
///
/// let mut alloc = WaterfillingAllocator::new(10.0, 1.0);
/// alloc.set_min_power(0.5);
///
/// let gains = [8.0, 4.0, 2.0, 0.5];
/// let (powers, capacity) = alloc.allocate_with_capacity(&gains);
///
/// assert!((powers.iter().sum::<f64>() - 10.0).abs() < 1e-9);
/// assert!(capacity > 0.0);
/// ```
pub struct WaterfillingAllocator {
    /// Total power budget across all channels.
    pub total_power: f64,
    /// Noise power per channel.
    pub noise_power: f64,
    /// Minimum power to allocate to any active channel.
    /// Channels that would receive less than this are shut off.
    pub min_power_per_channel: f64,
}

impl WaterfillingAllocator {
    /// Create a new allocator with the given power budget and noise power.
    pub fn new(total_power: f64, noise_power: f64) -> Self {
        Self {
            total_power,
            noise_power,
            min_power_per_channel: 0.0,
        }
    }

    /// Set the minimum power per active channel.
    ///
    /// Any channel whose water-filling allocation falls below this threshold
    /// will be shut off (zero power). The freed power is redistributed among
    /// the remaining active channels.
    pub fn set_min_power(&mut self, min: f64) {
        self.min_power_per_channel = min;
    }

    /// Allocate power across channels, returning the per-channel powers.
    pub fn allocate(&self, channel_gains: &[f64]) -> Vec<f64> {
        if channel_gains.is_empty() {
            return vec![];
        }

        let n = channel_gains.len();
        let mut active = vec![true; n];

        loop {
            let active_count = active.iter().filter(|&&a| a).count();
            if active_count == 0 {
                return vec![0.0; n];
            }

            let inv_gain_sum: f64 = (0..n)
                .filter(|&i| active[i])
                .map(|i| self.noise_power / channel_gains[i])
                .sum();

            let mu = (self.total_power + inv_gain_sum) / active_count as f64;

            let mut powers = vec![0.0; n];
            let mut any_below_min = false;
            for i in 0..n {
                if active[i] {
                    let p = mu - self.noise_power / channel_gains[i];
                    if p < self.min_power_per_channel {
                        active[i] = false;
                        any_below_min = true;
                    } else {
                        powers[i] = p;
                    }
                }
            }

            if !any_below_min {
                return powers;
            }
        }
    }

    /// Allocate power and also return the total achieved capacity.
    ///
    /// Returns `(powers, capacity)` where capacity is in bits per channel use.
    pub fn allocate_with_capacity(&self, channel_gains: &[f64]) -> (Vec<f64>, f64) {
        let powers = self.allocate(channel_gains);
        let cap = capacity_with_allocation(channel_gains, &powers, self.noise_power);
        (powers, cap)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    #[test]
    fn test_equal_channels() {
        // All channels identical => equal allocation regardless of algorithm
        let gains = [5.0, 5.0, 5.0, 5.0];
        let powers = waterfill(&gains, 8.0, 1.0);
        assert_eq!(powers.len(), 4);
        for &p in &powers {
            assert!((p - 2.0).abs() < EPS, "expected 2.0, got {p}");
        }
    }

    #[test]
    fn test_different_channels() {
        // Stronger channel gets more power
        let gains = [10.0, 1.0];
        let powers = waterfill(&gains, 4.0, 1.0);
        assert_eq!(powers.len(), 2);
        let total: f64 = powers.iter().sum();
        assert!(
            (total - 4.0).abs() < EPS,
            "total power should be 4.0, got {total}"
        );
        // Stronger channel (gain=10) should get more power than weaker (gain=1)
        assert!(
            powers[0] > powers[1],
            "stronger channel should get more power: {:.4} vs {:.4}",
            powers[0],
            powers[1]
        );
    }

    #[test]
    fn test_weak_channel_cutoff() {
        // One very weak channel should be shut off
        let gains = [10.0, 5.0, 0.01];
        let powers = waterfill(&gains, 2.0, 1.0);
        assert_eq!(powers.len(), 3);
        // Channel with gain 0.01 needs N/g = 100 just to reach the water level,
        // which is far above what is available. It should be shut off.
        assert!(
            powers[2] < EPS,
            "very weak channel should be shut off, got {:.6}",
            powers[2]
        );
        let total: f64 = powers.iter().sum();
        assert!(
            (total - 2.0).abs() < EPS,
            "total power should be 2.0, got {total}"
        );
    }

    #[test]
    fn test_water_level() {
        let gains = [4.0, 2.0, 1.0];
        let mu = water_level(&gains, 6.0, 1.0);
        // mu = (6 + 1/4 + 1/2 + 1/1) / 3 = (6 + 1.75) / 3 = 7.75 / 3
        let expected = 7.75 / 3.0;
        assert!(
            (mu - expected).abs() < EPS,
            "water level: expected {expected:.6}, got {mu:.6}"
        );
    }

    #[test]
    fn test_capacity_optimal() {
        // Water-filling capacity should be non-negative and finite
        let gains = [8.0, 4.0, 2.0, 1.0];
        let powers = waterfill(&gains, 10.0, 1.0);
        let cap = capacity_with_allocation(&gains, &powers, 1.0);
        assert!(cap > 0.0, "capacity should be positive");
        assert!(cap.is_finite(), "capacity should be finite");
    }

    #[test]
    fn test_equal_vs_waterfill() {
        // Water-filling always achieves >= capacity than equal power
        let gains = [10.0, 5.0, 2.0, 0.5];
        let total = 8.0;
        let noise = 1.0;

        let wf_powers = waterfill(&gains, total, noise);
        let eq_powers = equal_power(gains.len(), total);

        let cap_wf = capacity_with_allocation(&gains, &wf_powers, noise);
        let cap_eq = capacity_with_allocation(&gains, &eq_powers, noise);

        assert!(
            cap_wf >= cap_eq - EPS,
            "water-filling ({cap_wf:.6}) should beat equal power ({cap_eq:.6})"
        );
    }

    #[test]
    fn test_mercury() {
        let gains = [10.0, 5.0, 2.0, 1.0];
        let total = 8.0;
        let noise = 1.0;

        // With gap = 0 dB, mercury should equal standard water-filling
        let wf = waterfill(&gains, total, noise);
        let mwf_0db = mercury_waterfill(&gains, total, noise, 0.0);
        for (a, b) in wf.iter().zip(mwf_0db.iter()) {
            assert!(
                (a - b).abs() < EPS,
                "0 dB gap should match standard: {a:.6} vs {b:.6}"
            );
        }

        // With gap > 0, more channels may be shut off (less total capacity)
        let mwf_3db = mercury_waterfill(&gains, total, noise, 3.0);
        let cap_0 = capacity_with_allocation(&gains, &mwf_0db, noise);
        let cap_3 = capacity_with_allocation(&gains, &mwf_3db, noise);
        // The 3 dB gap allocation uses same total power but differently
        let total_3: f64 = mwf_3db.iter().sum();
        assert!(
            (total_3 - total).abs() < EPS,
            "mercury should use all power: {total_3:.6}"
        );
        // 3 dB gap should produce a different allocation (unless channels are all identical)
        // but still a valid one (we check power sums to total)
        assert!(cap_0.is_finite());
        assert!(cap_3.is_finite());
    }

    #[test]
    fn test_allocator() {
        let mut alloc = WaterfillingAllocator::new(10.0, 1.0);
        alloc.set_min_power(0.5);

        let gains = [8.0, 4.0, 2.0, 0.1];
        let (powers, cap) = alloc.allocate_with_capacity(&gains);

        // Very weak channel (gain=0.1) should be shut off with min_power=0.5
        assert!(
            powers[3] < EPS,
            "weak channel should be off, got {:.6}",
            powers[3]
        );

        // Total power should still sum to budget (among active channels)
        let total: f64 = powers.iter().sum();
        assert!(
            (total - 10.0).abs() < EPS,
            "total power: expected 10.0, got {total:.6}"
        );
        assert!(cap > 0.0);
    }

    #[test]
    fn test_inverse() {
        // Inverse water-filling allocates more to weaker channels
        let gains = [10.0, 1.0];
        let powers = inverse_waterfill(&gains, 4.0, 1.0);
        assert_eq!(powers.len(), 2);
        let total: f64 = powers.iter().sum();
        assert!(
            (total - 4.0).abs() < EPS,
            "total power should be 4.0, got {total}"
        );
        // Weaker channel (gain=1) should get MORE power in inverse WF
        assert!(
            powers[1] > powers[0],
            "weaker channel should get more power: {:.4} vs {:.4}",
            powers[1],
            powers[0]
        );
    }

    #[test]
    fn test_single_channel() {
        // Single channel gets all the power
        let gains = [5.0];
        let powers = waterfill(&gains, 3.0, 1.0);
        assert_eq!(powers.len(), 1);
        assert!(
            (powers[0] - 3.0).abs() < EPS,
            "single channel should get all power"
        );

        // Capacity of single channel
        let cap = capacity_with_allocation(&gains, &powers, 1.0);
        let expected = (1.0_f64 + 5.0 * 3.0 / 1.0).log2(); // log2(16) = 4
        assert!(
            (cap - expected).abs() < EPS,
            "expected {expected:.6}, got {cap:.6}"
        );
    }
}

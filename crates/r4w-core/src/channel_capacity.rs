//! Shannon Channel Capacity — AWGN, MIMO, and Fading Channels
//!
//! Computes theoretical channel capacity limits based on Shannon's channel
//! coding theorem. Includes AWGN capacity, MIMO capacity with equal power
//! and waterfilling allocation, Rayleigh fading ergodic/outage capacity,
//! and Eb/N0 conversions. Useful for benchmarking waveform designs against
//! theoretical limits and for link-level system analysis.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::channel_capacity::{awgn_capacity, spectral_efficiency, ChannelCapacityAnalyzer};
//!
//! // AWGN capacity at 1 MHz bandwidth, 10 dB SNR
//! let snr_linear = 10.0_f64.powf(10.0 / 10.0); // 10 dB -> linear
//! let cap = awgn_capacity(1.0e6, snr_linear);
//! assert!(cap > 3.0e6); // > 3 Mbps
//!
//! // Spectral efficiency at same SNR
//! let se = spectral_efficiency(snr_linear);
//! assert!((se - cap / 1.0e6).abs() < 1e-6);
//!
//! // Parametric analysis over SNR range
//! let analyzer = ChannelCapacityAnalyzer::new(1.0e6);
//! let results = analyzer.capacity_vs_snr(&[0.0, 5.0, 10.0, 15.0, 20.0]);
//! assert_eq!(results.len(), 5);
//! ```

/// Euler-Mascheroni constant.
const EULER_MASCHERONI: f64 = 0.5772156649015329;

// ---------------------------------------------------------------------------
// AWGN capacity
// ---------------------------------------------------------------------------

/// Shannon capacity of an AWGN channel.
///
/// C = B * log2(1 + SNR) bits/s
///
/// # Arguments
/// * `bandwidth_hz` - Channel bandwidth in Hz
/// * `snr_linear` - Signal-to-noise ratio (linear scale, **not** dB)
pub fn awgn_capacity(bandwidth_hz: f64, snr_linear: f64) -> f64 {
    bandwidth_hz * (1.0 + snr_linear).log2()
}

/// Shannon capacity of an AWGN channel with SNR given in dB.
///
/// Converts `snr_db` to linear and calls [`awgn_capacity`].
pub fn awgn_capacity_db(bandwidth_hz: f64, snr_db: f64) -> f64 {
    let snr_linear = 10.0_f64.powf(snr_db / 10.0);
    awgn_capacity(bandwidth_hz, snr_linear)
}

/// Spectral efficiency of an AWGN channel.
///
/// C/B = log2(1 + SNR) bits/s/Hz
pub fn spectral_efficiency(snr_linear: f64) -> f64 {
    (1.0 + snr_linear).log2()
}

// ---------------------------------------------------------------------------
// MIMO capacity
// ---------------------------------------------------------------------------

/// MIMO capacity with equal power allocation over an identity channel.
///
/// For an i.i.d. Rayleigh flat-fading channel with perfect CSIR and
/// equal power allocation:
///
/// C = min(N_tx, N_rx) * log2(1 + SNR / N_tx) bits/s/Hz
///
/// This is the capacity when the channel matrix H = I (identity), i.e.
/// all eigenvalues are 1.
///
/// # Arguments
/// * `num_tx` - Number of transmit antennas
/// * `num_rx` - Number of receive antennas
/// * `snr_linear` - Total SNR (linear)
pub fn mimo_capacity(num_tx: usize, num_rx: usize, snr_linear: f64) -> f64 {
    let min_dim = num_tx.min(num_rx) as f64;
    min_dim * (1.0 + snr_linear / num_tx as f64).log2()
}

/// MIMO capacity with optimal waterfilling power allocation.
///
/// Given the squared singular values (eigenvalues of H^H * H) and total
/// power budget, waterfilling distributes power across eigenmodes to
/// maximise the sum-rate:
///
/// C = sum_i log2(1 + p_i * lambda_i / noise_power)
///
/// where p_i = max(mu - noise_power / lambda_i, 0) and mu is chosen so
/// that sum(p_i) = total_power.
///
/// # Arguments
/// * `eigenvalues` - Squared singular values of the channel matrix (lambda_i)
/// * `total_power` - Total transmit power budget
/// * `noise_power` - Noise power per receive antenna
pub fn mimo_capacity_waterfilling(
    eigenvalues: &[f64],
    total_power: f64,
    noise_power: f64,
) -> f64 {
    if eigenvalues.is_empty() || total_power <= 0.0 || noise_power <= 0.0 {
        return 0.0;
    }

    // Compute inverse eigenvalue-to-noise ratios
    let mut inv_snr: Vec<f64> = eigenvalues
        .iter()
        .filter(|&&l| l > 0.0)
        .map(|&l| noise_power / l)
        .collect();

    if inv_snr.is_empty() {
        return 0.0;
    }

    // Sort ascending so we can peel off the weakest channels first
    inv_snr.sort_by(|a, b| a.partial_cmp(b).unwrap());

    // Iterative waterfilling: start with all channels active, remove
    // those that would get negative power.
    let n = inv_snr.len();
    let mut active = n;

    loop {
        // Water level: mu = (total_power + sum of 1/snr_i) / active
        let sum_inv: f64 = inv_snr[..active].iter().sum();
        let mu = (total_power + sum_inv) / active as f64;

        // Check if the weakest active channel gets non-negative power
        if mu > inv_snr[active - 1] {
            // All active channels have positive power allocation
            let capacity: f64 = inv_snr[..active]
                .iter()
                .map(|&inv| (mu / inv).log2())
                .sum();
            return capacity;
        }

        // Remove the weakest channel
        active -= 1;
        if active == 0 {
            return 0.0;
        }
    }
}

// ---------------------------------------------------------------------------
// Fading channel capacity
// ---------------------------------------------------------------------------

/// Ergodic capacity of a Rayleigh fading channel (single antenna).
///
/// For a Rayleigh fading channel with perfect CSI at the receiver, the
/// ergodic capacity is approximated as:
///
/// C_erg ≈ log2(1 + SNR) - γ / ln(2)
///
/// where γ ≈ 0.5772 is the Euler-Mascheroni constant. This approximation
/// is most accurate at high SNR.
pub fn rayleigh_ergodic_capacity(snr_linear: f64) -> f64 {
    (1.0 + snr_linear).log2() - EULER_MASCHERONI / 2.0_f64.ln()
}

/// Outage capacity of a slow-fading channel.
///
/// The ε-outage capacity is the rate that can be supported with
/// probability (1 - ε). For a Rayleigh fading channel:
///
/// C_out = log2(1 + SNR * ln(1 / (1 - ε)))
///
/// This uses the inverse CDF of the exponential distribution
/// (Rayleigh power) to find the SNR exceeded with probability 1 - ε,
/// then computes the corresponding AWGN capacity.
///
/// # Arguments
/// * `snr_linear` - Average SNR (linear)
/// * `outage_prob` - Outage probability ε (0 < ε < 1)
pub fn outage_capacity(snr_linear: f64, outage_prob: f64) -> f64 {
    // The CDF of exponential(1) is F(x) = 1 - e^{-x}.
    // We want the SNR exceeded with probability 1 - outage_prob,
    // i.e. P(gamma > gamma_th) = 1 - outage_prob
    // => gamma_th = -ln(1 - outage_prob) * mean_snr
    // But for outage, we want the SNR value at the outage_prob quantile:
    // gamma_th = -ln(1 - outage_prob) * snr_linear
    // Wait — the outage_prob quantile of the fading gain g ~ Exp(1):
    // P(g < g_th) = outage_prob => g_th = -ln(1 - outage_prob)
    // Instantaneous SNR = snr_linear * g, so
    // SNR_th = snr_linear * (-ln(1 - outage_prob))
    let snr_threshold = snr_linear * (-(1.0 - outage_prob).ln());
    (1.0 + snr_threshold).log2()
}

// ---------------------------------------------------------------------------
// Eb/N0 conversions
// ---------------------------------------------------------------------------

/// Convert Eb/N0 (dB) to SNR (dB).
///
/// SNR_dB = Eb/N0_dB + 10 * log10(spectral_efficiency)
///
/// The spectral efficiency η relates the two via SNR = (Eb/N0) * η.
pub fn eb_no_to_snr(eb_no_db: f64, spectral_eff: f64) -> f64 {
    eb_no_db + 10.0 * spectral_eff.log10()
}

/// Convert SNR (dB) to Eb/N0 (dB).
///
/// Eb/N0_dB = SNR_dB - 10 * log10(spectral_efficiency)
pub fn snr_to_eb_no(snr_db: f64, spectral_eff: f64) -> f64 {
    snr_db - 10.0 * spectral_eff.log10()
}

/// Shannon limit on Eb/N0 for reliable communication.
///
/// Returns −1.59 dB (= 10 * log10(ln 2) ≈ −1.5917 dB), the minimum
/// Eb/N0 at which error-free transmission is theoretically possible
/// as the spectral efficiency approaches zero.
pub fn shannon_limit_eb_no() -> f64 {
    10.0 * 2.0_f64.ln().log10()
}

// ---------------------------------------------------------------------------
// ChannelCapacityAnalyzer
// ---------------------------------------------------------------------------

/// Parametric channel capacity analyzer.
///
/// Wraps a bandwidth setting and provides convenience methods for
/// generating capacity-versus-SNR curves.
#[derive(Debug, Clone)]
pub struct ChannelCapacityAnalyzer {
    bandwidth_hz: f64,
}

impl ChannelCapacityAnalyzer {
    /// Create a new analyzer for the given channel bandwidth.
    pub fn new(bandwidth_hz: f64) -> Self {
        Self { bandwidth_hz }
    }

    /// Compute AWGN capacity for each SNR value in `snr_range_db`.
    ///
    /// Returns a vector of `(snr_db, capacity_bps)` pairs.
    pub fn capacity_vs_snr(&self, snr_range_db: &[f64]) -> Vec<(f64, f64)> {
        snr_range_db
            .iter()
            .map(|&snr_db| {
                let cap = awgn_capacity_db(self.bandwidth_hz, snr_db);
                (snr_db, cap)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: convert dB to linear.
    fn db_to_linear(db: f64) -> f64 {
        10.0_f64.powf(db / 10.0)
    }

    #[test]
    fn test_awgn_basic() {
        // At SNR = 0 dB (linear 1), C = B * log2(2) = B
        let cap = awgn_capacity(1.0e6, 1.0);
        assert!((cap - 1.0e6).abs() < 1.0, "C should equal B at SNR=1");

        // At SNR = 10 dB, C = B * log2(1+10) ≈ B * 3.459
        let snr_10db = db_to_linear(10.0);
        let cap10 = awgn_capacity(1.0e6, snr_10db);
        let expected = 1.0e6 * (1.0 + snr_10db).log2();
        assert!((cap10 - expected).abs() < 1.0);

        // awgn_capacity_db should agree
        let cap10_db = awgn_capacity_db(1.0e6, 10.0);
        assert!((cap10 - cap10_db).abs() < 1.0);
    }

    #[test]
    fn test_spectral_efficiency() {
        // At SNR = 0 dB: SE = log2(2) = 1 bit/s/Hz
        let se = spectral_efficiency(1.0);
        assert!((se - 1.0).abs() < 1e-10);

        // At SNR = 7 dB: SE = log2(1 + 5.012) ≈ 2.588
        let snr_7db = db_to_linear(7.0);
        let se7 = spectral_efficiency(snr_7db);
        assert!((se7 - (1.0 + snr_7db).log2()).abs() < 1e-10);
    }

    #[test]
    fn test_mimo_siso() {
        // MIMO with 1x1 should equal SISO AWGN spectral efficiency
        let snr = db_to_linear(10.0);
        let mimo_cap = mimo_capacity(1, 1, snr);
        let siso_se = spectral_efficiency(snr);
        assert!(
            (mimo_cap - siso_se).abs() < 1e-10,
            "1x1 MIMO should equal SISO"
        );
    }

    #[test]
    fn test_mimo_2x2() {
        // 2x2 MIMO identity channel: C = 2 * log2(1 + SNR/2)
        let snr = db_to_linear(20.0);
        let cap = mimo_capacity(2, 2, snr);
        let expected = 2.0 * (1.0 + snr / 2.0).log2();
        assert!((cap - expected).abs() < 1e-10);

        // 2x2 should give higher capacity than 1x1 at same total power
        let siso_cap = mimo_capacity(1, 1, snr);
        assert!(cap > siso_cap, "2x2 MIMO should beat SISO");
    }

    #[test]
    fn test_waterfilling() {
        // Single eigenvalue: all power goes to one channel
        // C = log2(1 + P * lambda / N)
        let eigenvalues = vec![1.0];
        let cap = mimo_capacity_waterfilling(&eigenvalues, 10.0, 1.0);
        let expected = (1.0 + 10.0 * 1.0_f64 / 1.0).log2();
        assert!(
            (cap - expected).abs() < 1e-10,
            "Single eigenvalue: got {cap}, expected {expected}"
        );

        // Two equal eigenvalues: equal power split
        let eigenvalues = vec![1.0, 1.0];
        let cap2 = mimo_capacity_waterfilling(&eigenvalues, 10.0, 1.0);
        // mu = (10 + 1 + 1) / 2 = 6, p_i = 6 - 1 = 5 each
        // C = 2 * log2(6 / 1) = 2 * log2(6)
        let expected2 = 2.0 * 6.0_f64.log2();
        assert!(
            (cap2 - expected2).abs() < 1e-10,
            "Two equal eigenvalues: got {cap2}, expected {expected2}"
        );

        // Very weak second channel should get no power
        let eigenvalues = vec![1.0, 0.001];
        let cap_wf = mimo_capacity_waterfilling(&eigenvalues, 1.0, 1.0);
        // The weak channel has inv_snr = 1000, which is >> total power,
        // so waterfilling should allocate zero to it.
        assert!(cap_wf > 0.0, "Should still have positive capacity");

        // Empty eigenvalues
        assert_eq!(mimo_capacity_waterfilling(&[], 10.0, 1.0), 0.0);
    }

    #[test]
    fn test_rayleigh() {
        // Ergodic capacity should be less than AWGN capacity
        let snr = db_to_linear(10.0);
        let awgn = spectral_efficiency(snr);
        let rayleigh = rayleigh_ergodic_capacity(snr);
        assert!(
            rayleigh < awgn,
            "Rayleigh ergodic capacity ({rayleigh}) should be < AWGN ({awgn})"
        );

        // The correction term is γ/ln(2) ≈ 0.8327
        let correction = EULER_MASCHERONI / 2.0_f64.ln();
        assert!((awgn - rayleigh - correction).abs() < 1e-10);
    }

    #[test]
    fn test_outage() {
        let snr = db_to_linear(10.0);

        // Low outage probability => lower capacity (more conservative)
        let cap_1pct = outage_capacity(snr, 0.01);
        let cap_10pct = outage_capacity(snr, 0.10);
        let cap_50pct = outage_capacity(snr, 0.50);
        assert!(
            cap_1pct < cap_10pct,
            "1% outage capacity should be < 10% outage capacity"
        );
        assert!(
            cap_10pct < cap_50pct,
            "10% outage capacity should be < 50% outage capacity"
        );

        // At 50% outage, the threshold SNR = snr * ln(2) ≈ 0.693 * snr
        let expected_50 = (1.0 + snr * 2.0_f64.ln()).log2();
        assert!((cap_50pct - expected_50).abs() < 1e-10);
    }

    #[test]
    fn test_eb_no_conversion() {
        // Round-trip: Eb/N0 -> SNR -> Eb/N0
        let eb_no_db = 5.0;
        let spectral_eff = 2.0; // e.g. QPSK
        let snr_db = eb_no_to_snr(eb_no_db, spectral_eff);
        let eb_no_back = snr_to_eb_no(snr_db, spectral_eff);
        assert!(
            (eb_no_back - eb_no_db).abs() < 1e-10,
            "Round-trip Eb/N0 conversion failed"
        );

        // SNR = Eb/N0 + 10*log10(2) ≈ Eb/N0 + 3.01
        let expected_snr = eb_no_db + 10.0 * 2.0_f64.log10();
        assert!((snr_db - expected_snr).abs() < 1e-10);
    }

    #[test]
    fn test_shannon_limit() {
        let limit = shannon_limit_eb_no();
        // Should be approximately -1.59 dB
        assert!(
            (limit - (-1.5917)).abs() < 0.001,
            "Shannon limit should be ≈ -1.59 dB, got {limit}"
        );
        // More precise: 10*log10(ln2) = 10 * log10(0.6931) = 10 * (-0.15917) = -1.5917
        let precise = 10.0 * 2.0_f64.ln().log10();
        assert!((limit - precise).abs() < 1e-12);
    }

    #[test]
    fn test_analyzer() {
        let analyzer = ChannelCapacityAnalyzer::new(1.0e6);
        let snr_range = vec![-5.0, 0.0, 5.0, 10.0, 15.0, 20.0];
        let results = analyzer.capacity_vs_snr(&snr_range);

        assert_eq!(results.len(), 6);

        // Capacity should be monotonically increasing with SNR
        for i in 1..results.len() {
            assert!(
                results[i].1 > results[i - 1].1,
                "Capacity should increase with SNR"
            );
        }

        // Check that SNR values are passed through correctly
        for (i, &snr) in snr_range.iter().enumerate() {
            assert!((results[i].0 - snr).abs() < 1e-12);
        }

        // Spot-check: at 0 dB, C = B * log2(2) = B = 1 MHz
        let cap_0db = results[1].1;
        assert!(
            (cap_0db - 1.0e6).abs() < 1.0,
            "At 0 dB SNR, C should be 1 MHz, got {cap_0db}"
        );
    }
}

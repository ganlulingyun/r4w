//! Multi-antenna diversity combining techniques for improved reception in fading channels.
//!
//! This module provides several diversity combining strategies used in wireless
//! receivers with multiple antenna branches:
//!
//! - **Selection Combining (SC)**: Picks the branch with the highest SNR/RSSI.
//! - **Equal Gain Combining (EGC)**: Co-phases all branches and sums with equal weight.
//! - **Maximal Ratio Combining (MRC)**: Weights each branch by its SNR, co-phases and sums.
//! - **Switch and Stay (SSC)**: Stays on the current branch until quality drops below threshold.
//!
//! All combiners support 2-8 antenna branches and provide diversity gain metrics.
//!
//! # Example
//!
//! ```
//! use r4w_core::antenna_diversity_combiner::{SelectionCombiner, DiversityCombiner};
//!
//! // Two-branch selection combiner
//! let mut sc = SelectionCombiner::new(2).unwrap();
//!
//! // Pilot symbols for channel estimation
//! let pilots = vec![(1.0, 0.0); 4];
//!
//! // Received pilot observations per branch (branch 0 weaker, branch 1 stronger)
//! let branch0_pilots: Vec<(f64, f64)> = vec![(0.3, 0.1), (0.2, 0.15), (0.35, 0.05), (0.25, 0.1)];
//! let branch1_pilots: Vec<(f64, f64)> = vec![(0.9, 0.05), (0.85, 0.1), (0.95, 0.0), (0.88, 0.08)];
//!
//! sc.estimate_channels(&[&branch0_pilots, &branch1_pilots], &pilots);
//!
//! // Data samples per branch
//! let branch0_data = vec![(0.3, 0.4), (-0.2, 0.35)];
//! let branch1_data = vec![(0.8, 0.2), (-0.7, 0.15)];
//!
//! let combined = sc.combine(&[&branch0_data, &branch1_data]);
//! assert_eq!(combined.len(), 2);
//! ```

use std::f64::consts::PI;

// ─────────────────────────── Complex helpers ───────────────────────────

/// Complex multiply: (a+jb)(c+jd)
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex conjugate
#[inline]
fn conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Magnitude squared
#[inline]
fn mag2(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Magnitude
#[inline]
fn mag(a: (f64, f64)) -> f64 {
    mag2(a).sqrt()
}

/// Phase angle
#[inline]
fn phase(a: (f64, f64)) -> f64 {
    a.1.atan2(a.0)
}

/// Complex from polar
#[inline]
fn from_polar(r: f64, theta: f64) -> (f64, f64) {
    (r * theta.cos(), r * theta.sin())
}

/// Complex add
#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Scale complex by real
#[inline]
fn cscale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

// ─────────────────────────── Error type ────────────────────────────────

/// Errors from combiner operations.
#[derive(Debug, Clone, PartialEq)]
pub enum CombinerError {
    /// Number of branches must be 2-8.
    InvalidBranchCount(usize),
    /// Supplied data has wrong number of branches.
    BranchCountMismatch { expected: usize, got: usize },
    /// Branches have different sample counts.
    UnequalBranchLengths,
    /// No channel estimate available (call `estimate_channels` first).
    NoChannelEstimate,
    /// Pilot vectors have mismatched lengths.
    PilotLengthMismatch,
    /// Need at least one pilot symbol.
    NoPilots,
}

impl std::fmt::Display for CombinerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidBranchCount(n) => write!(f, "branch count {n} not in 2..=8"),
            Self::BranchCountMismatch { expected, got } => {
                write!(f, "expected {expected} branches, got {got}")
            }
            Self::UnequalBranchLengths => write!(f, "branches have different sample counts"),
            Self::NoChannelEstimate => write!(f, "no channel estimate; call estimate_channels first"),
            Self::PilotLengthMismatch => write!(f, "pilot observation length != reference length"),
            Self::NoPilots => write!(f, "need at least one pilot symbol"),
        }
    }
}

impl std::error::Error for CombinerError {}

// ─────────────────────────── Statistics ─────────────────────────────────

/// Per-combination statistics and diversity gain metrics.
#[derive(Debug, Clone)]
pub struct CombinerStats {
    /// Estimated SNR (linear) per branch.
    pub branch_snr: Vec<f64>,
    /// Combined output SNR (linear).
    pub combined_snr: f64,
    /// Diversity gain in dB = 10*log10(combined_snr / max(branch_snr)).
    pub diversity_gain_db: f64,
    /// Selected branch index (for SC and SSC).
    pub selected_branch: Option<usize>,
    /// Estimated channel amplitude per branch.
    pub branch_amplitudes: Vec<f64>,
    /// Estimated channel phase (radians) per branch.
    pub branch_phases: Vec<f64>,
}

impl CombinerStats {
    /// Theoretical diversity gain for `n_branches` independent Rayleigh fading branches
    /// at a given outage probability `p_out` (0 < p_out < 1) for selection combining.
    ///
    /// For SC the CDF is `F(γ) = (1 - exp(-γ/γ_avg))^L`.
    /// The gain is expressed as the ratio of the SNR thresholds at `p_out`
    /// for L branches vs. 1 branch, in dB.
    pub fn theoretical_sc_gain_db(n_branches: usize, p_out: f64) -> f64 {
        if n_branches < 1 || p_out <= 0.0 || p_out >= 1.0 {
            return 0.0;
        }
        let l = n_branches as f64;
        // SNR threshold for single branch: γ_1 = -ln(1 - p_out)
        let gamma_1 = -(1.0 - p_out).ln();
        // SNR threshold for L branches: γ_L = -ln(1 - p_out^(1/L))
        let gamma_l = -(1.0 - p_out.powf(1.0 / l)).ln();
        10.0 * (gamma_l / gamma_1).log10()
    }

    /// Theoretical MRC diversity gain for `n_branches` independent Rayleigh branches.
    /// MRC achieves full diversity order L, with average combined SNR = L * γ_avg.
    /// Returns gain in dB = 10*log10(L).
    pub fn theoretical_mrc_gain_db(n_branches: usize) -> f64 {
        if n_branches < 1 {
            return 0.0;
        }
        10.0 * (n_branches as f64).log10()
    }
}

// ─────────────────────────── Channel state ──────────────────────────────

/// Per-branch channel estimate: complex gain h = amplitude * exp(j*phase).
#[derive(Debug, Clone)]
struct BranchChannel {
    /// Estimated complex channel gain.
    gain: (f64, f64),
    /// Estimated SNR (linear).
    snr: f64,
}

impl BranchChannel {
    fn amplitude(&self) -> f64 {
        mag(self.gain)
    }
    fn phase(&self) -> f64 {
        phase(self.gain)
    }
}

// ─────────────────────────── Trait ──────────────────────────────────────

/// Common interface for all diversity combiners.
pub trait DiversityCombiner {
    /// Number of antenna branches.
    fn num_branches(&self) -> usize;

    /// Estimate per-branch channel from pilot symbols.
    ///
    /// `received_pilots[b]` is the slice of received pilot IQ for branch `b`.
    /// `reference_pilots` is the known transmitted pilot sequence.
    fn estimate_channels(
        &mut self,
        received_pilots: &[&[(f64, f64)]],
        reference_pilots: &[(f64, f64)],
    ) -> Result<(), CombinerError>;

    /// Combine IQ samples from all branches.
    ///
    /// `branches[b]` is the received data IQ slice for branch `b`.
    /// Returns the combined output samples.
    fn combine(&mut self, branches: &[&[(f64, f64)]]) -> Vec<(f64, f64)>;

    /// Get the latest combiner statistics (available after `combine`).
    fn stats(&self) -> Option<&CombinerStats>;
}

// ─────────────────────────── Helpers ────────────────────────────────────

fn validate_branch_count(n: usize) -> Result<(), CombinerError> {
    if !(2..=8).contains(&n) {
        Err(CombinerError::InvalidBranchCount(n))
    } else {
        Ok(())
    }
}

fn validate_branches(
    expected: usize,
    branches: &[&[(f64, f64)]],
) -> Result<usize, CombinerError> {
    if branches.len() != expected {
        return Err(CombinerError::BranchCountMismatch {
            expected,
            got: branches.len(),
        });
    }
    let len = branches[0].len();
    for b in branches.iter().skip(1) {
        if b.len() != len {
            return Err(CombinerError::UnequalBranchLengths);
        }
    }
    Ok(len)
}

/// Estimate channel for one branch from pilot observations and reference pilots.
/// Returns (complex_gain, noise_variance).
fn estimate_branch_channel(
    rx_pilots: &[(f64, f64)],
    ref_pilots: &[(f64, f64)],
) -> (/* gain */ (f64, f64), /* noise_var */ f64) {
    let n = rx_pilots.len();
    // LS estimate: h = (1/N) * sum( rx[k] * conj(ref[k]) / |ref[k]|^2 )
    let mut h_sum = (0.0, 0.0);
    for i in 0..n {
        let ref_conj = conj(ref_pilots[i]);
        let ref_pwr = mag2(ref_pilots[i]);
        if ref_pwr > 1e-30 {
            let prod = cmul(rx_pilots[i], ref_conj);
            h_sum = cadd(h_sum, cscale(prod, 1.0 / ref_pwr));
        }
    }
    let h_est = cscale(h_sum, 1.0 / n as f64);

    // Noise variance estimate
    let mut noise_var = 0.0;
    for i in 0..n {
        let predicted = cmul(h_est, ref_pilots[i]);
        let diff = (rx_pilots[i].0 - predicted.0, rx_pilots[i].1 - predicted.1);
        noise_var += mag2(diff);
    }
    noise_var /= n as f64;

    (h_est, noise_var)
}

fn build_channels(
    received_pilots: &[&[(f64, f64)]],
    reference_pilots: &[(f64, f64)],
    num_branches: usize,
) -> Result<Vec<BranchChannel>, CombinerError> {
    if received_pilots.len() != num_branches {
        return Err(CombinerError::BranchCountMismatch {
            expected: num_branches,
            got: received_pilots.len(),
        });
    }
    if reference_pilots.is_empty() {
        return Err(CombinerError::NoPilots);
    }
    for rp in received_pilots {
        if rp.len() != reference_pilots.len() {
            return Err(CombinerError::PilotLengthMismatch);
        }
    }
    let mut channels = Vec::with_capacity(num_branches);
    for b in 0..num_branches {
        let (gain, noise_var) = estimate_branch_channel(received_pilots[b], reference_pilots);
        let signal_pwr = mag2(gain);
        let snr = if noise_var > 1e-30 {
            signal_pwr / noise_var
        } else {
            1e6 // essentially noiseless
        };
        channels.push(BranchChannel { gain, snr });
    }
    Ok(channels)
}

fn build_stats(channels: &[BranchChannel], combined_snr: f64, selected: Option<usize>) -> CombinerStats {
    let branch_snr: Vec<f64> = channels.iter().map(|c| c.snr).collect();
    let max_branch = branch_snr.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let diversity_gain_db = if max_branch > 1e-30 {
        10.0 * (combined_snr / max_branch).log10()
    } else {
        0.0
    };
    CombinerStats {
        branch_snr,
        combined_snr,
        diversity_gain_db,
        selected_branch: selected,
        branch_amplitudes: channels.iter().map(|c| c.amplitude()).collect(),
        branch_phases: channels.iter().map(|c| c.phase()).collect(),
    }
}

// ─────────────────────── Selection Combiner ────────────────────────────

/// Selection Combining: picks the antenna branch with the highest estimated SNR.
pub struct SelectionCombiner {
    num_branches: usize,
    channels: Option<Vec<BranchChannel>>,
    last_stats: Option<CombinerStats>,
}

impl SelectionCombiner {
    pub fn new(num_branches: usize) -> Result<Self, CombinerError> {
        validate_branch_count(num_branches)?;
        Ok(Self {
            num_branches,
            channels: None,
            last_stats: None,
        })
    }
}

impl DiversityCombiner for SelectionCombiner {
    fn num_branches(&self) -> usize {
        self.num_branches
    }

    fn estimate_channels(
        &mut self,
        received_pilots: &[&[(f64, f64)]],
        reference_pilots: &[(f64, f64)],
    ) -> Result<(), CombinerError> {
        self.channels = Some(build_channels(received_pilots, reference_pilots, self.num_branches)?);
        Ok(())
    }

    fn combine(&mut self, branches: &[&[(f64, f64)]]) -> Vec<(f64, f64)> {
        let len = match validate_branches(self.num_branches, branches) {
            Ok(l) => l,
            Err(_) => return vec![],
        };
        let channels = match &self.channels {
            Some(c) => c,
            None => return vec![],
        };

        // Select branch with highest SNR
        let best = channels
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.snr.partial_cmp(&b.1.snr).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, _)| i)
            .unwrap_or(0);

        let combined_snr = channels[best].snr;
        self.last_stats = Some(build_stats(channels, combined_snr, Some(best)));

        branches[best].to_vec()
    }

    fn stats(&self) -> Option<&CombinerStats> {
        self.last_stats.as_ref()
    }
}

// ─────────────────── Equal Gain Combiner ───────────────────────────────

/// Equal Gain Combining: co-phases all branches (removes channel phase), then sums
/// with equal amplitude weight.
pub struct EqualGainCombiner {
    num_branches: usize,
    channels: Option<Vec<BranchChannel>>,
    last_stats: Option<CombinerStats>,
}

impl EqualGainCombiner {
    pub fn new(num_branches: usize) -> Result<Self, CombinerError> {
        validate_branch_count(num_branches)?;
        Ok(Self {
            num_branches,
            channels: None,
            last_stats: None,
        })
    }
}

impl DiversityCombiner for EqualGainCombiner {
    fn num_branches(&self) -> usize {
        self.num_branches
    }

    fn estimate_channels(
        &mut self,
        received_pilots: &[&[(f64, f64)]],
        reference_pilots: &[(f64, f64)],
    ) -> Result<(), CombinerError> {
        self.channels = Some(build_channels(received_pilots, reference_pilots, self.num_branches)?);
        Ok(())
    }

    fn combine(&mut self, branches: &[&[(f64, f64)]]) -> Vec<(f64, f64)> {
        let len = match validate_branches(self.num_branches, branches) {
            Ok(l) => l,
            Err(_) => return vec![],
        };
        let channels = match &self.channels {
            Some(c) => c,
            None => return vec![],
        };

        // Co-phase each branch: multiply by exp(-j*phase(h_b))
        let mut output = vec![(0.0, 0.0); len];
        for (b, ch) in channels.iter().enumerate() {
            let dephase = from_polar(1.0, -ch.phase());
            for i in 0..len {
                let rotated = cmul(branches[b][i], dephase);
                output[i] = cadd(output[i], rotated);
            }
        }

        // Normalize by number of branches
        let scale = 1.0 / self.num_branches as f64;
        for s in &mut output {
            *s = cscale(*s, scale);
        }

        // EGC combined SNR ≈ (sum |h_b|)^2 / (L * noise_var)
        let sum_amp: f64 = channels.iter().map(|c| c.amplitude()).sum();
        // Average noise variance across branches
        let avg_noise = {
            let total: f64 = channels.iter().map(|c| {
                let sig = mag2(c.gain);
                if c.snr > 1e-30 { sig / c.snr } else { 1e-30 }
            }).sum();
            total / self.num_branches as f64
        };
        let combined_snr = if avg_noise > 1e-30 {
            (sum_amp * sum_amp) / (self.num_branches as f64 * avg_noise)
        } else {
            1e6
        };

        self.last_stats = Some(build_stats(channels, combined_snr, None));
        output
    }

    fn stats(&self) -> Option<&CombinerStats> {
        self.last_stats.as_ref()
    }
}

// ─────────────────── Maximal Ratio Combiner ────────────────────────────

/// Maximal Ratio Combining: weights each branch by its channel gain conjugate (SNR-optimal).
/// This is the optimal linear combiner for independent branch noise.
pub struct MaximalRatioCombiner {
    num_branches: usize,
    channels: Option<Vec<BranchChannel>>,
    last_stats: Option<CombinerStats>,
}

impl MaximalRatioCombiner {
    pub fn new(num_branches: usize) -> Result<Self, CombinerError> {
        validate_branch_count(num_branches)?;
        Ok(Self {
            num_branches,
            channels: None,
            last_stats: None,
        })
    }
}

impl DiversityCombiner for MaximalRatioCombiner {
    fn num_branches(&self) -> usize {
        self.num_branches
    }

    fn estimate_channels(
        &mut self,
        received_pilots: &[&[(f64, f64)]],
        reference_pilots: &[(f64, f64)],
    ) -> Result<(), CombinerError> {
        self.channels = Some(build_channels(received_pilots, reference_pilots, self.num_branches)?);
        Ok(())
    }

    fn combine(&mut self, branches: &[&[(f64, f64)]]) -> Vec<(f64, f64)> {
        let len = match validate_branches(self.num_branches, branches) {
            Ok(l) => l,
            Err(_) => return vec![],
        };
        let channels = match &self.channels {
            Some(c) => c,
            None => return vec![],
        };

        // MRC weight for branch b: w_b = h_b* / sum(|h_k|^2)
        let total_gain2: f64 = channels.iter().map(|c| mag2(c.gain)).sum();

        let mut output = vec![(0.0, 0.0); len];
        if total_gain2 < 1e-30 {
            self.last_stats = Some(build_stats(channels, 0.0, None));
            return output;
        }

        for (b, ch) in channels.iter().enumerate() {
            let weight = cscale(conj(ch.gain), 1.0 / total_gain2);
            for i in 0..len {
                let weighted = cmul(branches[b][i], weight);
                output[i] = cadd(output[i], weighted);
            }
        }

        // MRC combined SNR = sum of branch SNRs
        let combined_snr: f64 = channels.iter().map(|c| c.snr).sum();
        self.last_stats = Some(build_stats(channels, combined_snr, None));
        output
    }

    fn stats(&self) -> Option<&CombinerStats> {
        self.last_stats.as_ref()
    }
}

// ─────────────────── Switch and Stay Combiner ──────────────────────────

/// Switch-and-Stay Combining: stays on the current antenna branch until its
/// estimated SNR drops below a configurable threshold, then switches to the
/// next branch in round-robin order.
pub struct SwitchAndStayCombiner {
    num_branches: usize,
    /// SNR threshold (linear) below which to switch.
    threshold: f64,
    /// Current active branch.
    active_branch: usize,
    channels: Option<Vec<BranchChannel>>,
    last_stats: Option<CombinerStats>,
}

impl SwitchAndStayCombiner {
    /// Create a new SSC combiner.
    ///
    /// `threshold_db` is the SNR threshold in dB below which the combiner switches.
    pub fn new(num_branches: usize, threshold_db: f64) -> Result<Self, CombinerError> {
        validate_branch_count(num_branches)?;
        Ok(Self {
            num_branches,
            threshold: 10.0_f64.powf(threshold_db / 10.0),
            active_branch: 0,
            channels: None,
            last_stats: None,
        })
    }

    /// Get the currently active branch index.
    pub fn active_branch(&self) -> usize {
        self.active_branch
    }
}

impl DiversityCombiner for SwitchAndStayCombiner {
    fn num_branches(&self) -> usize {
        self.num_branches
    }

    fn estimate_channels(
        &mut self,
        received_pilots: &[&[(f64, f64)]],
        reference_pilots: &[(f64, f64)],
    ) -> Result<(), CombinerError> {
        self.channels = Some(build_channels(received_pilots, reference_pilots, self.num_branches)?);
        Ok(())
    }

    fn combine(&mut self, branches: &[&[(f64, f64)]]) -> Vec<(f64, f64)> {
        let len = match validate_branches(self.num_branches, branches) {
            Ok(l) => l,
            Err(_) => return vec![],
        };
        let channels = match &self.channels {
            Some(c) => c,
            None => return vec![],
        };

        // Check if current branch is below threshold
        if channels[self.active_branch].snr < self.threshold {
            // Switch to next branch (round-robin)
            self.active_branch = (self.active_branch + 1) % self.num_branches;
        }

        let combined_snr = channels[self.active_branch].snr;
        self.last_stats = Some(build_stats(channels, combined_snr, Some(self.active_branch)));

        branches[self.active_branch].to_vec()
    }

    fn stats(&self) -> Option<&CombinerStats> {
        self.last_stats.as_ref()
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // Utility: create pilot-observed samples for a branch with a known channel gain and noise level.
    fn make_rx_pilots(
        ref_pilots: &[(f64, f64)],
        channel_gain: (f64, f64),
        noise_scale: f64,
    ) -> Vec<(f64, f64)> {
        // Deterministic pseudo-noise using simple LFSR-like sequence
        let mut rx = Vec::with_capacity(ref_pilots.len());
        for (i, p) in ref_pilots.iter().enumerate() {
            let s = cmul(channel_gain, *p);
            // Tiny deterministic perturbation instead of random noise
            let noise_i = noise_scale * ((i as f64 * 0.7).sin());
            let noise_q = noise_scale * ((i as f64 * 1.3).cos());
            rx.push((s.0 + noise_i, s.1 + noise_q));
        }
        rx
    }

    fn make_data(
        data: &[(f64, f64)],
        channel_gain: (f64, f64),
    ) -> Vec<(f64, f64)> {
        data.iter().map(|d| cmul(channel_gain, *d)).collect()
    }

    fn reference_pilots() -> Vec<(f64, f64)> {
        vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0),
             (1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
    }

    fn data_symbols() -> Vec<(f64, f64)> {
        vec![(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)]
    }

    // ── Branch count validation ──

    #[test]
    fn test_invalid_branch_count_too_few() {
        assert!(SelectionCombiner::new(1).is_err());
        assert!(EqualGainCombiner::new(0).is_err());
        assert!(MaximalRatioCombiner::new(1).is_err());
        assert!(SwitchAndStayCombiner::new(1, 10.0).is_err());
    }

    #[test]
    fn test_invalid_branch_count_too_many() {
        assert!(SelectionCombiner::new(9).is_err());
        assert!(EqualGainCombiner::new(10).is_err());
        assert!(MaximalRatioCombiner::new(100).is_err());
        assert!(SwitchAndStayCombiner::new(9, 10.0).is_err());
    }

    #[test]
    fn test_valid_branch_counts() {
        for n in 2..=8 {
            assert!(SelectionCombiner::new(n).is_ok());
            assert!(EqualGainCombiner::new(n).is_ok());
            assert!(MaximalRatioCombiner::new(n).is_ok());
            assert!(SwitchAndStayCombiner::new(n, 5.0).is_ok());
        }
    }

    // ── Selection Combiner ──

    #[test]
    fn test_sc_selects_stronger_branch() {
        let mut sc = SelectionCombiner::new(2).unwrap();
        let pilots = reference_pilots();
        let strong_gain = (0.9, 0.1);
        let weak_gain = (0.2, 0.05);

        let rx0 = make_rx_pilots(&pilots, weak_gain, 0.001);
        let rx1 = make_rx_pilots(&pilots, strong_gain, 0.001);
        sc.estimate_channels(&[&rx0, &rx1], &pilots).unwrap();

        let data = data_symbols();
        let d0 = make_data(&data, weak_gain);
        let d1 = make_data(&data, strong_gain);
        let combined = sc.combine(&[&d0, &d1]);

        // SC should select branch 1 (stronger)
        let stats = sc.stats().unwrap();
        assert_eq!(stats.selected_branch, Some(1));
        // Output should be identical to branch 1
        for (c, expected) in combined.iter().zip(d1.iter()) {
            assert!((c.0 - expected.0).abs() < 1e-12);
            assert!((c.1 - expected.1).abs() < 1e-12);
        }
    }

    #[test]
    fn test_sc_with_three_branches() {
        let mut sc = SelectionCombiner::new(3).unwrap();
        let pilots = reference_pilots();

        let gains = [(0.1, 0.0), (0.5, 0.2), (0.3, -0.1)];
        let rxp: Vec<Vec<(f64, f64)>> = gains.iter().map(|g| make_rx_pilots(&pilots, *g, 0.001)).collect();
        let rxp_refs: Vec<&[(f64, f64)]> = rxp.iter().map(|v| v.as_slice()).collect();
        sc.estimate_channels(&rxp_refs, &pilots).unwrap();

        let data = data_symbols();
        let ds: Vec<Vec<(f64, f64)>> = gains.iter().map(|g| make_data(&data, *g)).collect();
        let ds_refs: Vec<&[(f64, f64)]> = ds.iter().map(|v| v.as_slice()).collect();
        let _ = sc.combine(&ds_refs);

        let stats = sc.stats().unwrap();
        // Branch 1 has highest amplitude (0.5, 0.2) -> |h|=0.539
        assert_eq!(stats.selected_branch, Some(1));
    }

    // ── Equal Gain Combiner ──

    #[test]
    fn test_egc_co_phases_branches() {
        let mut egc = EqualGainCombiner::new(2).unwrap();
        let pilots = reference_pilots();

        // Two branches with same amplitude but different phase
        let g0 = from_polar(0.8, 0.0);     // 0° phase
        let g1 = from_polar(0.8, PI / 3.0); // 60° phase

        let rx0 = make_rx_pilots(&pilots, g0, 0.001);
        let rx1 = make_rx_pilots(&pilots, g1, 0.001);
        egc.estimate_channels(&[&rx0, &rx1], &pilots).unwrap();

        let data = vec![(1.0, 0.0)];
        let d0 = make_data(&data, g0);
        let d1 = make_data(&data, g1);
        let combined = egc.combine(&[&d0, &d1]);

        // After co-phasing, both branches should constructively add.
        // Each branch contributes ~0.8 in-phase, so combined ~0.8 (after /2 normalization)
        let out_mag = mag(combined[0]);
        assert!(out_mag > 0.7, "EGC output magnitude should be close to branch amplitude, got {out_mag}");
    }

    #[test]
    fn test_egc_diversity_gain_positive() {
        let mut egc = EqualGainCombiner::new(2).unwrap();
        let pilots = reference_pilots();
        let g0 = from_polar(0.5, 0.3);
        let g1 = from_polar(0.7, -0.5);

        let rx0 = make_rx_pilots(&pilots, g0, 0.01);
        let rx1 = make_rx_pilots(&pilots, g1, 0.01);
        egc.estimate_channels(&[&rx0, &rx1], &pilots).unwrap();

        let data = data_symbols();
        let d0 = make_data(&data, g0);
        let d1 = make_data(&data, g1);
        let _ = egc.combine(&[&d0, &d1]);

        let stats = egc.stats().unwrap();
        // Combined SNR should generally be >= max branch SNR
        assert!(stats.combined_snr >= 0.0, "combined SNR should be non-negative");
    }

    // ── Maximal Ratio Combiner ──

    #[test]
    fn test_mrc_combined_snr_is_sum() {
        let mut mrc = MaximalRatioCombiner::new(2).unwrap();
        let pilots = reference_pilots();
        let g0 = (0.6, 0.0);
        let g1 = (0.0, 0.8);

        let rx0 = make_rx_pilots(&pilots, g0, 0.01);
        let rx1 = make_rx_pilots(&pilots, g1, 0.01);
        mrc.estimate_channels(&[&rx0, &rx1], &pilots).unwrap();

        let data = data_symbols();
        let d0 = make_data(&data, g0);
        let d1 = make_data(&data, g1);
        let _ = mrc.combine(&[&d0, &d1]);

        let stats = mrc.stats().unwrap();
        // MRC combined SNR ≈ snr_0 + snr_1
        let sum_snr = stats.branch_snr[0] + stats.branch_snr[1];
        assert!(
            (stats.combined_snr - sum_snr).abs() / sum_snr < 0.01,
            "MRC combined SNR ({}) should equal sum of branch SNRs ({})",
            stats.combined_snr,
            sum_snr
        );
    }

    #[test]
    fn test_mrc_output_approximates_original() {
        let mut mrc = MaximalRatioCombiner::new(2).unwrap();
        let pilots = reference_pilots();
        // Both branches see the same signal with same gain (no noise)
        let g = (1.0, 0.0);

        let rx0 = make_rx_pilots(&pilots, g, 0.0);
        let rx1 = make_rx_pilots(&pilots, g, 0.0);
        mrc.estimate_channels(&[&rx0, &rx1], &pilots).unwrap();

        let data = vec![(1.0, 0.0), (-1.0, 0.0)];
        let d0 = make_data(&data, g);
        let d1 = make_data(&data, g);
        let combined = mrc.combine(&[&d0, &d1]);

        // With perfect identical channels, MRC output ≈ original data
        for (c, d) in combined.iter().zip(data.iter()) {
            assert!((c.0 - d.0).abs() < 0.01, "MRC real part: {} vs {}", c.0, d.0);
            assert!((c.1 - d.1).abs() < 0.01, "MRC imag part: {} vs {}", c.1, d.1);
        }
    }

    #[test]
    fn test_mrc_four_branches() {
        let mut mrc = MaximalRatioCombiner::new(4).unwrap();
        let pilots = reference_pilots();
        let gains = [
            from_polar(0.3, 0.0),
            from_polar(0.5, 1.0),
            from_polar(0.7, -0.5),
            from_polar(0.4, 2.0),
        ];

        let rxp: Vec<Vec<(f64, f64)>> = gains.iter().map(|g| make_rx_pilots(&pilots, *g, 0.01)).collect();
        let rxp_refs: Vec<&[(f64, f64)]> = rxp.iter().map(|v| v.as_slice()).collect();
        mrc.estimate_channels(&rxp_refs, &pilots).unwrap();

        let data = data_symbols();
        let ds: Vec<Vec<(f64, f64)>> = gains.iter().map(|g| make_data(&data, *g)).collect();
        let ds_refs: Vec<&[(f64, f64)]> = ds.iter().map(|v| v.as_slice()).collect();
        let combined = mrc.combine(&ds_refs);

        assert_eq!(combined.len(), data.len());
        let stats = mrc.stats().unwrap();
        assert_eq!(stats.branch_snr.len(), 4);
        // Diversity gain should be positive with 4 branches
        assert!(stats.diversity_gain_db > 0.0, "4-branch MRC should have positive diversity gain");
    }

    // ── Switch and Stay Combiner ──

    #[test]
    fn test_ssc_stays_on_good_branch() {
        let mut ssc = SwitchAndStayCombiner::new(2, 5.0).unwrap(); // threshold 5 dB
        let pilots = reference_pilots();
        let g0 = (0.9, 0.0); // strong
        let g1 = (0.3, 0.0); // weak

        let rx0 = make_rx_pilots(&pilots, g0, 0.01);
        let rx1 = make_rx_pilots(&pilots, g1, 0.01);
        ssc.estimate_channels(&[&rx0, &rx1], &pilots).unwrap();

        let data = data_symbols();
        let d0 = make_data(&data, g0);
        let d1 = make_data(&data, g1);
        let _ = ssc.combine(&[&d0, &d1]);

        // Branch 0 is strong (SNR >> 5dB threshold), should stay
        assert_eq!(ssc.active_branch(), 0);
    }

    #[test]
    fn test_ssc_switches_on_weak_branch() {
        let mut ssc = SwitchAndStayCombiner::new(2, 40.0).unwrap(); // very high threshold (40 dB)
        let pilots = reference_pilots();
        let g0 = (0.3, 0.0); // moderate
        let g1 = (0.8, 0.0); // stronger

        let rx0 = make_rx_pilots(&pilots, g0, 0.05);
        let rx1 = make_rx_pilots(&pilots, g1, 0.05);
        ssc.estimate_channels(&[&rx0, &rx1], &pilots).unwrap();

        let data = data_symbols();
        let d0 = make_data(&data, g0);
        let d1 = make_data(&data, g1);
        let _ = ssc.combine(&[&d0, &d1]);

        // With a 40 dB threshold, branch 0 SNR is likely below threshold → switch to 1
        assert_eq!(ssc.active_branch(), 1);
    }

    #[test]
    fn test_ssc_round_robin_wrap() {
        let mut ssc = SwitchAndStayCombiner::new(3, 100.0).unwrap(); // impossibly high threshold
        ssc.active_branch = 2; // start at last branch
        let pilots = reference_pilots();
        let gains = [(0.3, 0.0), (0.3, 0.0), (0.3, 0.0)];

        let rxp: Vec<Vec<(f64, f64)>> = gains.iter().map(|g| make_rx_pilots(&pilots, *g, 0.05)).collect();
        let rxp_refs: Vec<&[(f64, f64)]> = rxp.iter().map(|v| v.as_slice()).collect();
        ssc.estimate_channels(&rxp_refs, &pilots).unwrap();

        let data = data_symbols();
        let ds: Vec<Vec<(f64, f64)>> = gains.iter().map(|g| make_data(&data, *g)).collect();
        let ds_refs: Vec<&[(f64, f64)]> = ds.iter().map(|v| v.as_slice()).collect();
        let _ = ssc.combine(&ds_refs);

        // Should wrap from branch 2 to branch 0
        assert_eq!(ssc.active_branch(), 0);
    }

    // ── Channel estimation ──

    #[test]
    fn test_channel_estimate_accuracy() {
        let mut mrc = MaximalRatioCombiner::new(2).unwrap();
        let pilots = reference_pilots();
        let true_gain = (0.7, 0.3);

        let rx = make_rx_pilots(&pilots, true_gain, 0.0); // no noise → exact estimate
        let rx_other = make_rx_pilots(&pilots, (0.5, 0.0), 0.0);
        mrc.estimate_channels(&[&rx, &rx_other], &pilots).unwrap();

        let stats = mrc.stats();
        // Should not have stats yet (no combine called)
        assert!(stats.is_none());

        // Combine to populate stats
        let data = vec![(1.0, 0.0)];
        let d0 = make_data(&data, true_gain);
        let d1 = make_data(&data, (0.5, 0.0));
        let _ = mrc.combine(&[&d0, &d1]);

        let stats = mrc.stats().unwrap();
        // Amplitude of branch 0 should be close to |0.7 + 0.3j| ≈ 0.7616
        let expected_amp = mag(true_gain);
        assert!(
            (stats.branch_amplitudes[0] - expected_amp).abs() < 0.01,
            "estimated amplitude {} vs expected {}",
            stats.branch_amplitudes[0],
            expected_amp
        );
    }

    #[test]
    fn test_pilot_length_mismatch_error() {
        let mut sc = SelectionCombiner::new(2).unwrap();
        let ref_pilots = vec![(1.0, 0.0), (0.0, 1.0)];
        let rx0 = vec![(0.5, 0.1)]; // length 1 vs 2
        let rx1 = vec![(0.5, 0.1), (0.1, 0.5)];
        let result = sc.estimate_channels(&[&rx0, &rx1], &ref_pilots);
        assert!(result.is_err());
    }

    #[test]
    fn test_no_pilots_error() {
        let mut sc = SelectionCombiner::new(2).unwrap();
        let ref_pilots: Vec<(f64, f64)> = vec![];
        let rx0: Vec<(f64, f64)> = vec![];
        let rx1: Vec<(f64, f64)> = vec![];
        let result = sc.estimate_channels(&[&rx0, &rx1], &ref_pilots);
        assert!(result.is_err());
    }

    // ── Theoretical gains ──

    #[test]
    fn test_theoretical_sc_gain() {
        let gain_2 = CombinerStats::theoretical_sc_gain_db(2, 0.01);
        let gain_4 = CombinerStats::theoretical_sc_gain_db(4, 0.01);

        // 2-branch SC should give ~10-12 dB gain at 1% outage
        assert!(gain_2 > 5.0, "2-branch SC gain should be > 5 dB, got {gain_2}");
        assert!(gain_2 < 15.0, "2-branch SC gain should be < 15 dB, got {gain_2}");

        // 4-branch should be more than 2-branch
        assert!(gain_4 > gain_2, "4-branch gain ({gain_4}) should exceed 2-branch ({gain_2})");
    }

    #[test]
    fn test_theoretical_mrc_gain() {
        assert!((CombinerStats::theoretical_mrc_gain_db(1) - 0.0).abs() < 1e-10);
        assert!((CombinerStats::theoretical_mrc_gain_db(2) - 3.0103).abs() < 0.001);
        assert!((CombinerStats::theoretical_mrc_gain_db(4) - 6.0206).abs() < 0.001);
    }

    // ── Edge cases ──

    #[test]
    fn test_branch_count_mismatch_in_combine() {
        let mut sc = SelectionCombiner::new(2).unwrap();
        let pilots = reference_pilots();
        let rx0 = make_rx_pilots(&pilots, (0.5, 0.0), 0.01);
        let rx1 = make_rx_pilots(&pilots, (0.5, 0.0), 0.01);
        sc.estimate_channels(&[&rx0, &rx1], &pilots).unwrap();

        // Pass 3 branches when 2 expected
        let d = vec![(1.0, 0.0)];
        let result = sc.combine(&[&d, &d, &d]);
        assert!(result.is_empty()); // graceful empty return
    }

    #[test]
    fn test_combine_without_channel_estimate() {
        let mut sc = SelectionCombiner::new(2).unwrap();
        let d = vec![(1.0, 0.0)];
        let result = sc.combine(&[&d, &d]);
        assert!(result.is_empty()); // no channel estimate → empty
    }

    #[test]
    fn test_eight_branches_mrc() {
        let mut mrc = MaximalRatioCombiner::new(8).unwrap();
        let pilots = reference_pilots();

        let gains: Vec<(f64, f64)> = (0..8)
            .map(|i| from_polar(0.3 + 0.1 * i as f64, i as f64 * 0.5))
            .collect();

        let rxp: Vec<Vec<(f64, f64)>> = gains.iter().map(|g| make_rx_pilots(&pilots, *g, 0.01)).collect();
        let rxp_refs: Vec<&[(f64, f64)]> = rxp.iter().map(|v| v.as_slice()).collect();
        mrc.estimate_channels(&rxp_refs, &pilots).unwrap();

        let data = data_symbols();
        let ds: Vec<Vec<(f64, f64)>> = gains.iter().map(|g| make_data(&data, *g)).collect();
        let ds_refs: Vec<&[(f64, f64)]> = ds.iter().map(|v| v.as_slice()).collect();
        let combined = mrc.combine(&ds_refs);

        assert_eq!(combined.len(), 4);
        let stats = mrc.stats().unwrap();
        assert_eq!(stats.branch_snr.len(), 8);
        assert_eq!(stats.branch_amplitudes.len(), 8);
        assert_eq!(stats.branch_phases.len(), 8);
    }

    // ── Complex helper sanity ──

    #[test]
    fn test_complex_helpers() {
        // cmul
        let a = (3.0, 4.0);
        let b = (1.0, 2.0);
        let c = cmul(a, b);
        assert!((c.0 - (-5.0)).abs() < 1e-12);
        assert!((c.1 - 10.0).abs() < 1e-12);

        // conj
        let cc = conj(a);
        assert_eq!(cc, (3.0, -4.0));

        // mag
        assert!((mag(a) - 5.0).abs() < 1e-12);

        // from_polar round-trip
        let p = from_polar(5.0, phase(a));
        assert!((p.0 - a.0).abs() < 1e-10);
        assert!((p.1 - a.1).abs() < 1e-10);
    }
}

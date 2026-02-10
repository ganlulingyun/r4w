//! Adaptive Modulation and Coding (AMC) link adaptation engine for wireless systems.
//!
//! This module provides a [`LinkAdaptationEngine`] that dynamically selects the best
//! modulation and coding scheme (MCS) based on channel conditions (SNR, BLER). It
//! supports multiple adaptation algorithms including SNR-threshold, outer-loop BLER
//! adjustment, and mutual-information based selection, with hysteresis to prevent
//! rapid MCS oscillation.
//!
//! # Example
//!
//! ```
//! use r4w_core::link_adaptation_engine::{
//!     LinkAdaptationEngine, ModCodScheme, AdaptationAlgorithm,
//! };
//!
//! // Build a small MCS table
//! let table = vec![
//!     ModCodScheme::new(2, 0.5, 1.0, 3.0),   // QPSK 1/2
//!     ModCodScheme::new(4, 0.5, 2.0, 10.0),   // 16QAM 1/2
//!     ModCodScheme::new(6, 0.75, 4.5, 18.0),  // 64QAM 3/4
//! ];
//!
//! let mut engine = LinkAdaptationEngine::new(table);
//! engine.set_algorithm(AdaptationAlgorithm::SnrThreshold { margin_db: 2.0 });
//!
//! // Report a good SNR measurement
//! engine.update_snr(15.0);
//!
//! let decision = engine.recommend();
//! assert_eq!(decision.recommended_mcs, 1); // 16QAM 1/2 (requires 10 dB + 2 dB margin)
//! assert!(decision.predicted_throughput > 0.0);
//!
//! // Apply the recommendation
//! engine.apply(&decision);
//! assert_eq!(engine.state().current_mcs_index, 1);
//! ```

use std::fmt;

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------

/// A modulation-and-coding scheme descriptor.
#[derive(Debug, Clone, PartialEq)]
pub struct ModCodScheme {
    /// Modulation order (e.g. 2 = QPSK bits-per-symbol, 4 = 16-QAM, 6 = 64-QAM).
    pub modulation_order: u8,
    /// FEC code rate in (0, 1].
    pub code_rate: f64,
    /// Spectral efficiency in bits/s/Hz.
    pub spectral_efficiency: f64,
    /// Minimum required SNR (dB) for acceptable BLER at this MCS.
    pub required_snr_db: f64,
}

impl ModCodScheme {
    /// Create a new MCS entry.
    pub fn new(
        modulation_order: u8,
        code_rate: f64,
        spectral_efficiency: f64,
        required_snr_db: f64,
    ) -> Self {
        Self {
            modulation_order,
            code_rate,
            spectral_efficiency,
            required_snr_db,
        }
    }
}

impl fmt::Display for ModCodScheme {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mod_name = match self.modulation_order {
            1 => "BPSK",
            2 => "QPSK",
            4 => "16QAM",
            6 => "64QAM",
            8 => "256QAM",
            _ => "MOD",
        };
        write!(
            f,
            "{} R={:.2} (SE={:.2} b/s/Hz, req {:.1} dB)",
            mod_name, self.code_rate, self.spectral_efficiency, self.required_snr_db
        )
    }
}

/// Algorithm used for link adaptation decisions.
#[derive(Debug, Clone, PartialEq)]
pub enum AdaptationAlgorithm {
    /// Pick highest MCS whose required_snr < measured_snr - margin.
    SnrThreshold {
        /// Safety margin in dB subtracted from measured SNR.
        margin_db: f64,
    },
    /// Adjust effective SNR thresholds based on measured BLER, targeting a BLER
    /// setpoint (typically ~10 %).
    OuterLoopBler {
        /// Target BLER (0.0 .. 1.0), e.g. 0.10 for 10 %.
        target_bler: f64,
        /// Step size (dB) for threshold adjustments.
        step_db: f64,
    },
    /// Select MCS maximising estimated mutual information.
    MutualInformation,
}

impl Default for AdaptationAlgorithm {
    fn default() -> Self {
        AdaptationAlgorithm::SnrThreshold { margin_db: 3.0 }
    }
}

/// Snapshot of the current link state maintained by the engine.
#[derive(Debug, Clone, PartialEq)]
pub struct LinkState {
    /// Index into the MCS table for the currently active scheme.
    pub current_mcs_index: usize,
    /// Latest smoothed SNR estimate (dB).
    pub snr_estimate: f64,
    /// Latest BLER estimate (0.0 .. 1.0).
    pub bler_estimate: f64,
    /// Estimated throughput (bits per second) at the current MCS and SNR.
    pub throughput_bps: f64,
}

/// Result of a link-adaptation recommendation.
#[derive(Debug, Clone, PartialEq)]
pub struct AdaptationDecision {
    /// Recommended MCS table index.
    pub recommended_mcs: usize,
    /// Human-readable reason for the decision.
    pub reason: String,
    /// Predicted throughput if this MCS is adopted (bits/s).
    pub predicted_throughput: f64,
}

// ---------------------------------------------------------------------------
// Engine
// ---------------------------------------------------------------------------

/// Main adaptive modulation and coding (AMC) controller.
///
/// Maintains a table of [`ModCodScheme`]s sorted by increasing spectral
/// efficiency, tracks channel quality through SNR and BLER updates, and
/// recommends the best MCS for current conditions.
pub struct LinkAdaptationEngine {
    mcs_table: Vec<ModCodScheme>,
    algorithm: AdaptationAlgorithm,
    state: LinkState,
    /// Per-MCS effective SNR thresholds (used by outer-loop BLER).
    effective_thresholds: Vec<f64>,
    /// Hysteresis in dB: MCS upgrade requires this much extra margin beyond
    /// the threshold, while downgrade is immediate.
    hysteresis_db: f64,
    /// Exponential moving-average coefficient for SNR smoothing (0..1).
    snr_alpha: f64,
    /// Bandwidth in Hz assumed for throughput computation.
    bandwidth_hz: f64,
    /// Whether at least one SNR sample has been received.
    snr_initialised: bool,
    /// Running block-error counts for BLER estimation.
    total_blocks: u64,
    total_errors: u64,
}

impl LinkAdaptationEngine {
    // -- Construction -------------------------------------------------------

    /// Create a new engine with the given MCS table.
    ///
    /// The table is sorted internally by `required_snr_db` (ascending). The
    /// engine starts at MCS index 0 (lowest rate) with an SNR estimate of 0 dB.
    pub fn new(mut mcs_table: Vec<ModCodScheme>) -> Self {
        assert!(!mcs_table.is_empty(), "MCS table must not be empty");
        // Sort by required SNR so index 0 is the most robust scheme.
        mcs_table.sort_by(|a, b| {
            a.required_snr_db
                .partial_cmp(&b.required_snr_db)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let effective_thresholds: Vec<f64> =
            mcs_table.iter().map(|m| m.required_snr_db).collect();
        Self {
            mcs_table,
            algorithm: AdaptationAlgorithm::default(),
            state: LinkState {
                current_mcs_index: 0,
                snr_estimate: 0.0,
                bler_estimate: 0.0,
                throughput_bps: 0.0,
            },
            effective_thresholds,
            hysteresis_db: 1.0,
            snr_alpha: 0.3,
            bandwidth_hz: 1.0e6, // default 1 MHz
            snr_initialised: false,
            total_blocks: 0,
            total_errors: 0,
        }
    }

    /// Return an LTE-like MCS table (QPSK 1/3 through 64-QAM 4/5).
    pub fn lte_table() -> Vec<ModCodScheme> {
        vec![
            ModCodScheme::new(2, 1.0 / 3.0, 0.66, -1.0),
            ModCodScheme::new(2, 0.5, 1.0, 2.0),
            ModCodScheme::new(2, 2.0 / 3.0, 1.33, 4.5),
            ModCodScheme::new(2, 3.0 / 4.0, 1.5, 6.0),
            ModCodScheme::new(4, 1.0 / 3.0, 1.33, 7.5),
            ModCodScheme::new(4, 0.5, 2.0, 10.0),
            ModCodScheme::new(4, 2.0 / 3.0, 2.66, 12.5),
            ModCodScheme::new(4, 3.0 / 4.0, 3.0, 14.0),
            ModCodScheme::new(6, 1.0 / 2.0, 3.0, 16.0),
            ModCodScheme::new(6, 2.0 / 3.0, 4.0, 18.5),
            ModCodScheme::new(6, 3.0 / 4.0, 4.5, 20.0),
            ModCodScheme::new(6, 4.0 / 5.0, 4.8, 22.0),
        ]
    }

    /// Return a WiFi-like MCS table (MCS 0 through MCS 9).
    pub fn wifi_table() -> Vec<ModCodScheme> {
        vec![
            ModCodScheme::new(1, 0.5, 0.5, -2.0),     // MCS0 BPSK 1/2
            ModCodScheme::new(2, 0.5, 1.0, 2.0),       // MCS1 QPSK 1/2
            ModCodScheme::new(2, 3.0 / 4.0, 1.5, 5.0), // MCS2 QPSK 3/4
            ModCodScheme::new(4, 0.5, 2.0, 9.0),       // MCS3 16QAM 1/2
            ModCodScheme::new(4, 3.0 / 4.0, 3.0, 12.0),// MCS4 16QAM 3/4
            ModCodScheme::new(6, 2.0 / 3.0, 4.0, 16.0),// MCS5 64QAM 2/3
            ModCodScheme::new(6, 3.0 / 4.0, 4.5, 19.0),// MCS6 64QAM 3/4
            ModCodScheme::new(6, 5.0 / 6.0, 5.0, 21.0),// MCS7 64QAM 5/6
            ModCodScheme::new(8, 3.0 / 4.0, 6.0, 24.0),// MCS8 256QAM 3/4
            ModCodScheme::new(8, 5.0 / 6.0, 6.67, 27.0),// MCS9 256QAM 5/6
        ]
    }

    // -- Configuration ------------------------------------------------------

    /// Set the adaptation algorithm.
    pub fn set_algorithm(&mut self, algorithm: AdaptationAlgorithm) {
        self.algorithm = algorithm;
    }

    /// Set the hysteresis (dB) to prevent rapid MCS oscillation.
    ///
    /// An upgrade requires the SNR to exceed the threshold by `hysteresis_db`,
    /// while a downgrade triggers as soon as the threshold is violated.
    pub fn set_hysteresis(&mut self, db: f64) {
        self.hysteresis_db = db.abs();
    }

    /// Set the exponential-moving-average coefficient for SNR smoothing.
    ///
    /// `alpha` in `(0, 1]`; higher values give more weight to new samples.
    pub fn set_snr_alpha(&mut self, alpha: f64) {
        self.snr_alpha = alpha.clamp(0.01, 1.0);
    }

    /// Set the bandwidth (Hz) used for throughput estimation.
    pub fn set_bandwidth(&mut self, bw_hz: f64) {
        self.bandwidth_hz = bw_hz.max(1.0);
    }

    // -- Updates ------------------------------------------------------------

    /// Feed a new instantaneous SNR measurement (dB) into the engine.
    ///
    /// The value is smoothed with an exponential moving average.
    pub fn update_snr(&mut self, snr_db: f64) {
        if !self.snr_initialised {
            self.state.snr_estimate = snr_db;
            self.snr_initialised = true;
        } else {
            self.state.snr_estimate =
                self.snr_alpha * snr_db + (1.0 - self.snr_alpha) * self.state.snr_estimate;
        }
        // Recompute throughput for the current MCS.
        self.state.throughput_bps =
            self.throughput_estimate(self.state.current_mcs_index, self.state.snr_estimate);
    }

    /// Update BLER statistics with a batch of block results.
    ///
    /// `block_errors` out of `total_blocks` were received in error.
    pub fn update_bler(&mut self, block_errors: u32, total_blocks: u32) {
        if total_blocks == 0 {
            return;
        }
        self.total_errors += block_errors as u64;
        self.total_blocks += total_blocks as u64;
        if self.total_blocks > 0 {
            self.state.bler_estimate = self.total_errors as f64 / self.total_blocks as f64;
        }

        // Outer-loop threshold adjustment.
        if let AdaptationAlgorithm::OuterLoopBler {
            target_bler,
            step_db,
        } = &self.algorithm
        {
            let target = *target_bler;
            let step = *step_db;
            let idx = self.state.current_mcs_index;
            if self.state.bler_estimate > target {
                // Too many errors -> raise the effective threshold (be more conservative).
                self.effective_thresholds[idx] += step;
            } else {
                // BLER below target -> lower threshold (be more aggressive).
                self.effective_thresholds[idx] -= step * target / (1.0 - target);
            }
        }
    }

    // -- Recommendation -----------------------------------------------------

    /// Recommend the best MCS for the current channel state.
    pub fn recommend(&self) -> AdaptationDecision {
        match &self.algorithm {
            AdaptationAlgorithm::SnrThreshold { margin_db } => {
                self.recommend_snr_threshold(*margin_db)
            }
            AdaptationAlgorithm::OuterLoopBler { .. } => self.recommend_outer_loop(),
            AdaptationAlgorithm::MutualInformation => self.recommend_mutual_info(),
        }
    }

    /// Apply a previously obtained [`AdaptationDecision`], switching the active MCS.
    pub fn apply(&mut self, decision: &AdaptationDecision) {
        let idx = decision.recommended_mcs.min(self.mcs_table.len() - 1);
        self.state.current_mcs_index = idx;
        self.state.throughput_bps =
            self.throughput_estimate(idx, self.state.snr_estimate);
    }

    /// Estimate throughput (bits/s) for a given MCS index and SNR.
    ///
    /// Uses a simple sigmoid BLER model: at or above the required SNR the
    /// throughput is `spectral_efficiency * bandwidth * (1 - BLER)`.
    pub fn throughput_estimate(&self, mcs_index: usize, snr_db: f64) -> f64 {
        if mcs_index >= self.mcs_table.len() {
            return 0.0;
        }
        let mcs = &self.mcs_table[mcs_index];
        let bler = Self::sigmoid_bler(snr_db, mcs.required_snr_db);
        mcs.spectral_efficiency * self.bandwidth_hz * (1.0 - bler)
    }

    /// Access the current link state.
    pub fn state(&self) -> &LinkState {
        &self.state
    }

    /// Access the MCS table.
    pub fn mcs_table(&self) -> &[ModCodScheme] {
        &self.mcs_table
    }

    // -- Private helpers ----------------------------------------------------

    /// SNR-threshold recommendation.
    fn recommend_snr_threshold(&self, margin_db: f64) -> AdaptationDecision {
        let available_snr = self.state.snr_estimate - margin_db;
        let current = self.state.current_mcs_index;

        // Find the highest MCS whose threshold we can meet (with hysteresis).
        let mut best = 0usize;
        for (i, mcs) in self.mcs_table.iter().enumerate() {
            let threshold = if i > current {
                // Upgrading: require extra hysteresis margin.
                mcs.required_snr_db + self.hysteresis_db
            } else {
                mcs.required_snr_db
            };
            if available_snr >= threshold {
                best = i;
            }
        }

        let predicted = self.throughput_estimate(best, self.state.snr_estimate);
        let reason = if best > current {
            format!(
                "Upgrade to MCS {}: SNR {:.1} dB exceeds threshold {:.1}+{:.1} dB",
                best,
                self.state.snr_estimate,
                self.mcs_table[best].required_snr_db,
                margin_db + self.hysteresis_db,
            )
        } else if best < current {
            format!(
                "Downgrade to MCS {}: SNR {:.1} dB below threshold {:.1}+{:.1} dB",
                best,
                self.state.snr_estimate,
                self.mcs_table[current].required_snr_db,
                margin_db,
            )
        } else {
            format!(
                "Hold MCS {}: SNR {:.1} dB adequate",
                best, self.state.snr_estimate
            )
        };

        AdaptationDecision {
            recommended_mcs: best,
            reason,
            predicted_throughput: predicted,
        }
    }

    /// Outer-loop BLER recommendation (uses adjusted effective thresholds).
    fn recommend_outer_loop(&self) -> AdaptationDecision {
        let current = self.state.current_mcs_index;
        let mut best = 0usize;

        for (i, threshold) in self.effective_thresholds.iter().enumerate() {
            let effective = if i > current {
                *threshold + self.hysteresis_db
            } else {
                *threshold
            };
            if self.state.snr_estimate >= effective {
                best = i;
            }
        }

        let predicted = self.throughput_estimate(best, self.state.snr_estimate);
        let reason = if best != current {
            format!(
                "BLER-adjusted: MCS {} -> {} (BLER={:.3}, effective threshold={:.1} dB)",
                current, best, self.state.bler_estimate, self.effective_thresholds[best]
            )
        } else {
            format!(
                "Hold MCS {} (BLER={:.3}, threshold={:.1} dB)",
                best, self.state.bler_estimate, self.effective_thresholds[best]
            )
        };

        AdaptationDecision {
            recommended_mcs: best,
            reason,
            predicted_throughput: predicted,
        }
    }

    /// Mutual-information based recommendation.
    ///
    /// Approximates capacity with `log2(1 + SNR_linear)` and picks the MCS
    /// whose spectral efficiency is closest to but does not exceed that
    /// capacity.
    fn recommend_mutual_info(&self) -> AdaptationDecision {
        let snr_linear = 10.0_f64.powf(self.state.snr_estimate / 10.0);
        let capacity = (1.0 + snr_linear).log2();
        let current = self.state.current_mcs_index;

        let mut best = 0usize;
        for (i, mcs) in self.mcs_table.iter().enumerate() {
            if mcs.spectral_efficiency <= capacity {
                best = i;
            }
        }

        let predicted = self.throughput_estimate(best, self.state.snr_estimate);
        let reason = format!(
            "MI-based: capacity={:.2} b/s/Hz, selected MCS {} (SE={:.2})",
            capacity, best, self.mcs_table[best].spectral_efficiency
        );

        AdaptationDecision {
            recommended_mcs: best,
            reason,
            predicted_throughput: predicted,
        }
    }

    /// Simple sigmoid-shaped BLER model.
    ///
    /// Returns a value in `[0, 1]` where 0 means no errors and 1 means all
    /// blocks are in error. The transition is centred at `required_snr_db` with
    /// a slope of ~1 dB per decade of BLER change.
    fn sigmoid_bler(snr_db: f64, required_snr_db: f64) -> f64 {
        let x = snr_db - required_snr_db;
        // Steepness factor: ~1.5 gives a reasonable transition width (~6 dB
        // from BLER 0.9 to 0.1).
        1.0 / (1.0 + (1.5 * x).exp())
    }
}

impl fmt::Debug for LinkAdaptationEngine {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LinkAdaptationEngine")
            .field("algorithm", &self.algorithm)
            .field("state", &self.state)
            .field("mcs_table_len", &self.mcs_table.len())
            .field("hysteresis_db", &self.hysteresis_db)
            .finish()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_table() -> Vec<ModCodScheme> {
        vec![
            ModCodScheme::new(2, 0.5, 1.0, 3.0),   // MCS 0: QPSK 1/2
            ModCodScheme::new(4, 0.5, 2.0, 10.0),   // MCS 1: 16QAM 1/2
            ModCodScheme::new(6, 0.75, 4.5, 18.0),  // MCS 2: 64QAM 3/4
        ]
    }

    #[test]
    fn test_new_starts_at_lowest_mcs() {
        let engine = LinkAdaptationEngine::new(simple_table());
        assert_eq!(engine.state().current_mcs_index, 0);
        assert_eq!(engine.state().snr_estimate, 0.0);
    }

    #[test]
    fn test_mcs_table_sorted_by_snr() {
        let table = vec![
            ModCodScheme::new(6, 0.75, 4.5, 18.0),
            ModCodScheme::new(2, 0.5, 1.0, 3.0),
            ModCodScheme::new(4, 0.5, 2.0, 10.0),
        ];
        let engine = LinkAdaptationEngine::new(table);
        assert!(
            engine.mcs_table()[0].required_snr_db
                < engine.mcs_table()[1].required_snr_db
        );
        assert!(
            engine.mcs_table()[1].required_snr_db
                < engine.mcs_table()[2].required_snr_db
        );
    }

    #[test]
    fn test_snr_threshold_selects_correct_mcs() {
        let mut engine = LinkAdaptationEngine::new(simple_table());
        engine.set_algorithm(AdaptationAlgorithm::SnrThreshold { margin_db: 0.0 });
        engine.set_hysteresis(0.0);

        // SNR well above MCS 1 threshold but below MCS 2
        engine.update_snr(15.0);
        let d = engine.recommend();
        assert_eq!(d.recommended_mcs, 1);
    }

    #[test]
    fn test_snr_threshold_margin() {
        let mut engine = LinkAdaptationEngine::new(simple_table());
        engine.set_algorithm(AdaptationAlgorithm::SnrThreshold { margin_db: 3.0 });
        engine.set_hysteresis(0.0);

        // SNR = 12 dB, MCS 1 requires 10+3=13 dB -> should stay at MCS 0
        engine.update_snr(12.0);
        let d = engine.recommend();
        assert_eq!(d.recommended_mcs, 0);

        // SNR = 14 dB, 14-3=11 >= 10 -> MCS 1
        // Push multiple times to overcome EMA smoothing
        engine.update_snr(14.0);
        engine.update_snr(14.0);
        engine.update_snr(14.0);
        engine.update_snr(14.0);
        engine.update_snr(14.0);
        let d = engine.recommend();
        assert_eq!(d.recommended_mcs, 1);
    }

    #[test]
    fn test_hysteresis_prevents_upgrade_oscillation() {
        let mut engine = LinkAdaptationEngine::new(simple_table());
        engine.set_algorithm(AdaptationAlgorithm::SnrThreshold { margin_db: 0.0 });
        engine.set_hysteresis(2.0);
        engine.set_snr_alpha(1.0); // no smoothing

        // At SNR = 10.5 dB with hysteresis 2 dB, upgrade from MCS0 to MCS1
        // requires 10 + 2 = 12 dB, so should stay at MCS 0.
        engine.update_snr(10.5);
        let d = engine.recommend();
        assert_eq!(d.recommended_mcs, 0);

        // SNR = 13 dB exceeds 12 dB threshold -> upgrade
        engine.update_snr(13.0);
        let d = engine.recommend();
        assert_eq!(d.recommended_mcs, 1);
    }

    #[test]
    fn test_downgrade_is_immediate() {
        let mut engine = LinkAdaptationEngine::new(simple_table());
        engine.set_algorithm(AdaptationAlgorithm::SnrThreshold { margin_db: 0.0 });
        engine.set_hysteresis(2.0);
        engine.set_snr_alpha(1.0);

        // Set up at MCS 1
        engine.update_snr(15.0);
        let d = engine.recommend();
        engine.apply(&d);
        assert_eq!(engine.state().current_mcs_index, 1);

        // Drop SNR below MCS 1 threshold (10 dB) - downgrade is immediate (no hysteresis)
        engine.update_snr(9.0);
        let d = engine.recommend();
        assert_eq!(d.recommended_mcs, 0);
    }

    #[test]
    fn test_apply_updates_state() {
        let mut engine = LinkAdaptationEngine::new(simple_table());
        engine.set_snr_alpha(1.0);
        engine.update_snr(15.0);

        let decision = AdaptationDecision {
            recommended_mcs: 1,
            reason: "test".to_string(),
            predicted_throughput: 1e6,
        };
        engine.apply(&decision);
        assert_eq!(engine.state().current_mcs_index, 1);
        assert!(engine.state().throughput_bps > 0.0);
    }

    #[test]
    fn test_apply_clamps_to_table_size() {
        let mut engine = LinkAdaptationEngine::new(simple_table());
        let decision = AdaptationDecision {
            recommended_mcs: 999,
            reason: "overflow test".to_string(),
            predicted_throughput: 0.0,
        };
        engine.apply(&decision);
        assert_eq!(engine.state().current_mcs_index, 2); // clamped to last
    }

    #[test]
    fn test_update_bler() {
        let mut engine = LinkAdaptationEngine::new(simple_table());
        engine.update_bler(5, 100);
        assert!((engine.state().bler_estimate - 0.05).abs() < 1e-9);

        engine.update_bler(10, 100);
        // cumulative: 15 errors / 200 blocks = 0.075
        assert!((engine.state().bler_estimate - 0.075).abs() < 1e-9);
    }

    #[test]
    fn test_outer_loop_raises_threshold_on_high_bler() {
        let mut engine = LinkAdaptationEngine::new(simple_table());
        engine.set_algorithm(AdaptationAlgorithm::OuterLoopBler {
            target_bler: 0.10,
            step_db: 0.5,
        });

        let original = engine.effective_thresholds[0];
        // Report a high BLER (above target).
        engine.update_bler(20, 100);
        assert!(engine.effective_thresholds[0] > original);
    }

    #[test]
    fn test_outer_loop_lowers_threshold_on_low_bler() {
        let mut engine = LinkAdaptationEngine::new(simple_table());
        engine.set_algorithm(AdaptationAlgorithm::OuterLoopBler {
            target_bler: 0.10,
            step_db: 0.5,
        });

        let original = engine.effective_thresholds[0];
        // Report a low BLER (below target).
        engine.update_bler(1, 100);
        assert!(engine.effective_thresholds[0] < original);
    }

    #[test]
    fn test_mutual_information_selects_below_capacity() {
        let mut engine = LinkAdaptationEngine::new(simple_table());
        engine.set_algorithm(AdaptationAlgorithm::MutualInformation);
        engine.set_snr_alpha(1.0);

        // SNR = 10 dB -> SNR_lin = 10 -> capacity = log2(11) ~ 3.46 b/s/Hz
        // MCS 0 SE=1.0, MCS 1 SE=2.0, MCS 2 SE=4.5
        // Highest SE <= 3.46 is MCS 1 (SE=2.0)
        engine.update_snr(10.0);
        let d = engine.recommend();
        assert_eq!(d.recommended_mcs, 1);
    }

    #[test]
    fn test_throughput_estimate_at_high_snr() {
        let engine = LinkAdaptationEngine::new(simple_table());
        // At very high SNR, BLER -> 0, throughput -> SE * BW
        let tp = engine.throughput_estimate(0, 30.0);
        // MCS 0: SE=1.0, BW=1MHz -> ~1e6 bps
        assert!((tp - 1.0e6).abs() < 1.0e4);
    }

    #[test]
    fn test_throughput_estimate_at_low_snr() {
        let engine = LinkAdaptationEngine::new(simple_table());
        // At very low SNR (well below threshold), BLER -> 1, throughput -> 0
        let tp = engine.throughput_estimate(2, -20.0);
        assert!(tp < 1.0e3); // effectively zero
    }

    #[test]
    fn test_throughput_out_of_range_index() {
        let engine = LinkAdaptationEngine::new(simple_table());
        assert_eq!(engine.throughput_estimate(999, 10.0), 0.0);
    }

    #[test]
    fn test_lte_table_has_12_entries() {
        let table = LinkAdaptationEngine::lte_table();
        assert_eq!(table.len(), 12);
        // Verify they are already in ascending SNR order.
        for i in 1..table.len() {
            assert!(table[i].required_snr_db >= table[i - 1].required_snr_db);
        }
    }

    #[test]
    fn test_wifi_table_has_10_entries() {
        let table = LinkAdaptationEngine::wifi_table();
        assert_eq!(table.len(), 10);
        for i in 1..table.len() {
            assert!(table[i].required_snr_db >= table[i - 1].required_snr_db);
        }
    }

    #[test]
    fn test_snr_smoothing() {
        let mut engine = LinkAdaptationEngine::new(simple_table());
        engine.set_snr_alpha(0.5);

        engine.update_snr(10.0); // first sample -> directly 10.0
        assert!((engine.state().snr_estimate - 10.0).abs() < 1e-9);

        engine.update_snr(20.0); // 0.5*20 + 0.5*10 = 15.0
        assert!((engine.state().snr_estimate - 15.0).abs() < 1e-9);

        engine.update_snr(20.0); // 0.5*20 + 0.5*15 = 17.5
        assert!((engine.state().snr_estimate - 17.5).abs() < 1e-9);
    }

    #[test]
    fn test_decision_reason_contains_text() {
        let mut engine = LinkAdaptationEngine::new(simple_table());
        engine.set_algorithm(AdaptationAlgorithm::SnrThreshold { margin_db: 0.0 });
        engine.set_hysteresis(0.0);
        engine.set_snr_alpha(1.0);

        engine.update_snr(5.0);
        let d = engine.recommend();
        assert!(!d.reason.is_empty());
        // Should say "Hold" since we are at MCS 0 and SNR supports MCS 0
        assert!(d.reason.contains("Hold") || d.reason.contains("MCS"));
    }

    #[test]
    fn test_set_bandwidth_affects_throughput() {
        let mut engine = LinkAdaptationEngine::new(simple_table());
        engine.set_snr_alpha(1.0);
        engine.update_snr(30.0);

        let tp1 = engine.throughput_estimate(0, 30.0); // BW = 1 MHz default

        engine.set_bandwidth(20.0e6);
        let tp20 = engine.throughput_estimate(0, 30.0);

        assert!((tp20 / tp1 - 20.0).abs() < 0.5);
    }

    #[test]
    fn test_modcodscheme_display() {
        let mcs = ModCodScheme::new(2, 0.5, 1.0, 3.0);
        let s = format!("{}", mcs);
        assert!(s.contains("QPSK"));
        assert!(s.contains("0.50"));
    }

    #[test]
    fn test_full_adaptation_cycle() {
        // Simulate a full cycle: start low, ramp SNR up, then drop.
        let mut engine = LinkAdaptationEngine::new(simple_table());
        engine.set_algorithm(AdaptationAlgorithm::SnrThreshold { margin_db: 1.0 });
        engine.set_hysteresis(0.0);
        engine.set_snr_alpha(1.0);

        // Low SNR -> MCS 0
        engine.update_snr(0.0);
        let d = engine.recommend();
        engine.apply(&d);
        assert_eq!(engine.state().current_mcs_index, 0);

        // Medium SNR -> MCS 1
        engine.update_snr(15.0);
        let d = engine.recommend();
        engine.apply(&d);
        assert_eq!(engine.state().current_mcs_index, 1);

        // High SNR -> MCS 2
        engine.update_snr(25.0);
        let d = engine.recommend();
        engine.apply(&d);
        assert_eq!(engine.state().current_mcs_index, 2);

        // Drop SNR -> should fall back
        engine.update_snr(5.0);
        let d = engine.recommend();
        engine.apply(&d);
        assert_eq!(engine.state().current_mcs_index, 0);
    }
}

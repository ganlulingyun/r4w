//! # Adaptive Modulation and Coding (Link Adaptation)
//!
//! Closed-loop link adaptation engine that dynamically selects the optimal
//! modulation scheme and FEC code rate based on channel quality feedback.
//! Implements DVB-S2 ACM, LTE/5G CQI-to-MCS mapping, and Wi-Fi rate adaptation.
//!
//! GNU Radio equivalent: `gr-dtv/dvb_s2_modcod`, `gr-adapt` OOT, `gr-lte` CQI/MCS.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::adaptive_modcod::{AdaptiveModCod, ChannelQualityReport};
//!
//! let mut engine = AdaptiveModCod::with_dvb_s2_table();
//! let report = ChannelQualityReport {
//!     snr_db: 12.0,
//!     ber_estimate: 1e-5,
//!     mutual_information: 0.0,
//!     timestamp_ms: 0,
//! };
//! let decision = engine.update(&report);
//! assert!(decision.entry.spectral_efficiency > 0.0);
//! ```

/// A modulation + code-rate pair with performance thresholds.
#[derive(Debug, Clone)]
pub struct ModCodEntry {
    /// Index in the modcod table.
    pub index: u8,
    /// Modulation type.
    pub modulation: ModulationType,
    /// Code rate.
    pub code_rate: CodeRate,
    /// Spectral efficiency in bits/s/Hz.
    pub spectral_efficiency: f64,
    /// Minimum required SNR (dB) for target BER.
    pub snr_threshold_db: f64,
    /// Target BER for this mode.
    pub target_ber: f64,
}

/// Supported modulation types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModulationType {
    Bpsk,
    Qpsk,
    Psk8,
    Qam16,
    Qam64,
    Qam256,
}

impl ModulationType {
    /// Bits per symbol for this modulation.
    pub fn bits_per_symbol(&self) -> usize {
        match self {
            ModulationType::Bpsk => 1,
            ModulationType::Qpsk => 2,
            ModulationType::Psk8 => 3,
            ModulationType::Qam16 => 4,
            ModulationType::Qam64 => 6,
            ModulationType::Qam256 => 8,
        }
    }
}

/// Code rate as a fraction k/n.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CodeRate {
    pub k: u8,
    pub n: u8,
}

impl CodeRate {
    pub fn new(k: u8, n: u8) -> Self {
        Self { k, n }
    }

    pub fn rate(&self) -> f64 {
        self.k as f64 / self.n as f64
    }
}

/// Channel quality report from receiver feedback.
#[derive(Debug, Clone)]
pub struct ChannelQualityReport {
    /// Estimated SNR in dB.
    pub snr_db: f64,
    /// Estimated BER.
    pub ber_estimate: f64,
    /// Mutual information estimate.
    pub mutual_information: f64,
    /// Timestamp in milliseconds.
    pub timestamp_ms: u64,
}

/// Link adaptation configuration.
#[derive(Debug, Clone)]
pub struct LinkAdaptConfig {
    /// ModCod table (must be sorted by ascending snr_threshold_db).
    pub modcod_table: Vec<ModCodEntry>,
    /// Hysteresis margin in dB before switching up.
    pub hysteresis_db: f64,
    /// Number of reports to average over.
    pub averaging_window: usize,
    /// Safety margin below threshold in dB.
    pub backoff_db: f64,
    /// Adaptation strategy.
    pub strategy: AdaptationStrategy,
}

/// Adaptation strategy.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AdaptationStrategy {
    /// Maximize throughput for given channel.
    MaxThroughput,
    /// Maximize reliability (always stay below max allowed modcod).
    MaxReliability,
    /// Target a specific spectral efficiency.
    TargetEfficiency(f64),
}

/// Decision output from the adaptation engine.
#[derive(Debug, Clone)]
pub struct ModCodDecision {
    /// Selected modcod entry.
    pub entry: ModCodEntry,
    /// Whether the mode changed from previous decision.
    pub changed: bool,
    /// Reason for the decision.
    pub reason: DecisionReason,
    /// Averaged SNR used for the decision.
    pub averaged_snr_db: f64,
}

/// Reason for a modcod decision.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DecisionReason {
    Initial,
    SnrImproved,
    SnrDegraded,
    HysteresisHold,
    BerExceeded,
}

/// Adaptive Modulation and Coding engine.
pub struct AdaptiveModCod {
    config: LinkAdaptConfig,
    current_index: usize,
    snr_history: Vec<f64>,
    initialized: bool,
}

impl AdaptiveModCod {
    /// Create a new engine with the given configuration.
    /// The modcod table is sorted by ascending snr_threshold_db.
    pub fn new(mut config: LinkAdaptConfig) -> Self {
        config.modcod_table.sort_by(|a, b| a.snr_threshold_db.partial_cmp(&b.snr_threshold_db).unwrap());
        Self {
            config,
            current_index: 0,
            snr_history: Vec::new(),
            initialized: false,
        }
    }

    /// Create with DVB-S2 modcod table (28 entries per ETSI EN 302 307-1).
    pub fn with_dvb_s2_table() -> Self {
        let table = dvb_s2_modcod_table();
        Self::new(LinkAdaptConfig {
            modcod_table: table,
            hysteresis_db: 1.0,
            averaging_window: 10,
            backoff_db: 0.5,
            strategy: AdaptationStrategy::MaxThroughput,
        })
    }

    /// Create with LTE CQI-to-MCS table (15 entries).
    pub fn with_lte_cqi_table() -> Self {
        let table = lte_cqi_table();
        Self::new(LinkAdaptConfig {
            modcod_table: table,
            hysteresis_db: 1.0,
            averaging_window: 5,
            backoff_db: 0.0,
            strategy: AdaptationStrategy::MaxThroughput,
        })
    }

    /// Create with Wi-Fi MCS table (10 entries for 802.11n/ac).
    pub fn with_wifi_mcs_table() -> Self {
        let table = wifi_mcs_table();
        Self::new(LinkAdaptConfig {
            modcod_table: table,
            hysteresis_db: 2.0,
            averaging_window: 8,
            backoff_db: 1.0,
            strategy: AdaptationStrategy::MaxThroughput,
        })
    }

    /// Update the engine with a new channel quality report and get a decision.
    pub fn update(&mut self, report: &ChannelQualityReport) -> ModCodDecision {
        // Add to history
        self.snr_history.push(report.snr_db);
        if self.snr_history.len() > self.config.averaging_window {
            self.snr_history.remove(0);
        }

        let avg_snr = self.averaged_snr();

        if !self.initialized {
            self.initialized = true;
            let idx = self.select_index_for_snr(avg_snr);
            self.current_index = idx;
            return ModCodDecision {
                entry: self.config.modcod_table[idx].clone(),
                changed: true,
                reason: DecisionReason::Initial,
                averaged_snr_db: avg_snr,
            };
        }

        let new_idx = match self.config.strategy {
            AdaptationStrategy::MaxThroughput => {
                self.apply_hysteresis(avg_snr)
            }
            AdaptationStrategy::MaxReliability => {
                // Pick one step below the max-throughput choice
                let throughput_idx = self.select_index_for_snr(avg_snr);
                if throughput_idx > 0 { throughput_idx - 1 } else { 0 }
            }
            AdaptationStrategy::TargetEfficiency(target) => {
                self.select_index_for_efficiency(target, avg_snr)
            }
        };

        let changed = new_idx != self.current_index;
        let reason = if !changed {
            DecisionReason::HysteresisHold
        } else if new_idx > self.current_index {
            DecisionReason::SnrImproved
        } else {
            DecisionReason::SnrDegraded
        };

        self.current_index = new_idx;

        ModCodDecision {
            entry: self.config.modcod_table[new_idx].clone(),
            changed,
            reason,
            averaged_snr_db: avg_snr,
        }
    }

    /// Get the current modcod entry.
    pub fn current_modcod(&self) -> &ModCodEntry {
        &self.config.modcod_table[self.current_index]
    }

    /// Get current spectral efficiency.
    pub fn spectral_efficiency(&self) -> f64 {
        self.config.modcod_table[self.current_index].spectral_efficiency
    }

    /// Select the best modcod for a given SNR (stateless query).
    pub fn select_for_snr(&self, snr_db: f64) -> &ModCodEntry {
        let idx = self.select_index_for_snr(snr_db);
        &self.config.modcod_table[idx]
    }

    /// Get the modcod table.
    pub fn modcod_table(&self) -> &[ModCodEntry] {
        &self.config.modcod_table
    }

    /// Reset the engine state.
    pub fn reset(&mut self) {
        self.current_index = 0;
        self.snr_history.clear();
        self.initialized = false;
    }

    fn averaged_snr(&self) -> f64 {
        if self.snr_history.is_empty() {
            return -100.0;
        }
        // Exponential moving average
        let alpha = 2.0 / (self.snr_history.len() as f64 + 1.0);
        let mut avg = self.snr_history[0];
        for &s in &self.snr_history[1..] {
            avg = alpha * s + (1.0 - alpha) * avg;
        }
        avg
    }

    fn select_index_for_snr(&self, snr_db: f64) -> usize {
        let effective_snr = snr_db - self.config.backoff_db;
        let mut best = 0;
        for (i, entry) in self.config.modcod_table.iter().enumerate() {
            if effective_snr >= entry.snr_threshold_db {
                best = i;
            }
        }
        best
    }

    fn select_index_for_efficiency(&self, target: f64, snr_db: f64) -> usize {
        let effective_snr = snr_db - self.config.backoff_db;
        let mut best = 0;
        let mut best_diff = f64::MAX;
        for (i, entry) in self.config.modcod_table.iter().enumerate() {
            if effective_snr >= entry.snr_threshold_db {
                let diff = (entry.spectral_efficiency - target).abs();
                if diff < best_diff {
                    best_diff = diff;
                    best = i;
                }
            }
        }
        best
    }

    fn apply_hysteresis(&self, avg_snr: f64) -> usize {
        let effective_snr = avg_snr - self.config.backoff_db;
        let current = &self.config.modcod_table[self.current_index];

        // Check if we should switch down (immediate, smaller hysteresis)
        if effective_snr < current.snr_threshold_db - self.config.hysteresis_db / 2.0 {
            // Find the highest mode we qualify for
            return self.select_index_for_snr(avg_snr);
        }

        // Check if we should switch up (need full hysteresis margin)
        if self.current_index + 1 < self.config.modcod_table.len() {
            let next = &self.config.modcod_table[self.current_index + 1];
            if effective_snr >= next.snr_threshold_db + self.config.hysteresis_db {
                // Find the highest mode with hysteresis
                let mut best = self.current_index;
                for i in (self.current_index + 1)..self.config.modcod_table.len() {
                    if effective_snr >= self.config.modcod_table[i].snr_threshold_db + self.config.hysteresis_db {
                        best = i;
                    }
                }
                return best;
            }
        }

        self.current_index
    }
}

/// DVB-S2 modcod table (28 entries, ETSI EN 302 307-1).
pub fn dvb_s2_modcod_table() -> Vec<ModCodEntry> {
    vec![
        ModCodEntry { index: 1, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(1, 4), spectral_efficiency: 0.49, snr_threshold_db: -2.35, target_ber: 1e-5 },
        ModCodEntry { index: 2, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(1, 3), spectral_efficiency: 0.66, snr_threshold_db: -1.24, target_ber: 1e-5 },
        ModCodEntry { index: 3, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(2, 5), spectral_efficiency: 0.79, snr_threshold_db: -0.30, target_ber: 1e-5 },
        ModCodEntry { index: 4, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(1, 2), spectral_efficiency: 0.99, snr_threshold_db: 1.00, target_ber: 1e-5 },
        ModCodEntry { index: 5, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(3, 5), spectral_efficiency: 1.19, snr_threshold_db: 2.23, target_ber: 1e-5 },
        ModCodEntry { index: 6, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(2, 3), spectral_efficiency: 1.32, snr_threshold_db: 3.10, target_ber: 1e-5 },
        ModCodEntry { index: 7, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(3, 4), spectral_efficiency: 1.49, snr_threshold_db: 4.03, target_ber: 1e-5 },
        ModCodEntry { index: 8, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(4, 5), spectral_efficiency: 1.59, snr_threshold_db: 4.68, target_ber: 1e-5 },
        ModCodEntry { index: 9, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(5, 6), spectral_efficiency: 1.65, snr_threshold_db: 5.18, target_ber: 1e-5 },
        ModCodEntry { index: 10, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(8, 9), spectral_efficiency: 1.77, snr_threshold_db: 6.20, target_ber: 1e-5 },
        ModCodEntry { index: 11, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(9, 10), spectral_efficiency: 1.79, snr_threshold_db: 6.42, target_ber: 1e-5 },
        ModCodEntry { index: 12, modulation: ModulationType::Psk8, code_rate: CodeRate::new(3, 5), spectral_efficiency: 1.78, snr_threshold_db: 5.50, target_ber: 1e-5 },
        ModCodEntry { index: 13, modulation: ModulationType::Psk8, code_rate: CodeRate::new(2, 3), spectral_efficiency: 1.98, snr_threshold_db: 6.62, target_ber: 1e-5 },
        ModCodEntry { index: 14, modulation: ModulationType::Psk8, code_rate: CodeRate::new(3, 4), spectral_efficiency: 2.23, snr_threshold_db: 7.91, target_ber: 1e-5 },
        ModCodEntry { index: 15, modulation: ModulationType::Psk8, code_rate: CodeRate::new(5, 6), spectral_efficiency: 2.48, snr_threshold_db: 9.35, target_ber: 1e-5 },
        ModCodEntry { index: 16, modulation: ModulationType::Psk8, code_rate: CodeRate::new(8, 9), spectral_efficiency: 2.65, snr_threshold_db: 10.69, target_ber: 1e-5 },
        ModCodEntry { index: 17, modulation: ModulationType::Psk8, code_rate: CodeRate::new(9, 10), spectral_efficiency: 2.68, snr_threshold_db: 10.98, target_ber: 1e-5 },
        ModCodEntry { index: 18, modulation: ModulationType::Qam16, code_rate: CodeRate::new(2, 3), spectral_efficiency: 2.64, snr_threshold_db: 10.21, target_ber: 1e-5 },
        ModCodEntry { index: 19, modulation: ModulationType::Qam16, code_rate: CodeRate::new(3, 4), spectral_efficiency: 2.97, snr_threshold_db: 11.03, target_ber: 1e-5 },
        ModCodEntry { index: 20, modulation: ModulationType::Qam16, code_rate: CodeRate::new(4, 5), spectral_efficiency: 3.17, snr_threshold_db: 11.61, target_ber: 1e-5 },
        ModCodEntry { index: 21, modulation: ModulationType::Qam16, code_rate: CodeRate::new(5, 6), spectral_efficiency: 3.30, snr_threshold_db: 12.03, target_ber: 1e-5 },
        ModCodEntry { index: 22, modulation: ModulationType::Qam16, code_rate: CodeRate::new(8, 9), spectral_efficiency: 3.52, snr_threshold_db: 13.13, target_ber: 1e-5 },
        ModCodEntry { index: 23, modulation: ModulationType::Qam16, code_rate: CodeRate::new(9, 10), spectral_efficiency: 3.57, snr_threshold_db: 13.28, target_ber: 1e-5 },
        ModCodEntry { index: 24, modulation: ModulationType::Qam64, code_rate: CodeRate::new(2, 3), spectral_efficiency: 3.95, snr_threshold_db: 13.64, target_ber: 1e-5 },
        ModCodEntry { index: 25, modulation: ModulationType::Qam64, code_rate: CodeRate::new(3, 4), spectral_efficiency: 4.44, snr_threshold_db: 14.28, target_ber: 1e-5 },
        ModCodEntry { index: 26, modulation: ModulationType::Qam64, code_rate: CodeRate::new(4, 5), spectral_efficiency: 4.74, snr_threshold_db: 15.10, target_ber: 1e-5 },
        ModCodEntry { index: 27, modulation: ModulationType::Qam64, code_rate: CodeRate::new(5, 6), spectral_efficiency: 4.94, snr_threshold_db: 15.56, target_ber: 1e-5 },
        ModCodEntry { index: 28, modulation: ModulationType::Qam64, code_rate: CodeRate::new(9, 10), spectral_efficiency: 5.33, snr_threshold_db: 16.05, target_ber: 1e-5 },
    ]
}

/// LTE CQI table (15 entries, 3GPP TS 36.213 Table 7.2.3-1).
pub fn lte_cqi_table() -> Vec<ModCodEntry> {
    vec![
        ModCodEntry { index: 1, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(1, 13), spectral_efficiency: 0.1523, snr_threshold_db: -6.7, target_ber: 1e-4 },
        ModCodEntry { index: 2, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(1, 8), spectral_efficiency: 0.2344, snr_threshold_db: -4.7, target_ber: 1e-4 },
        ModCodEntry { index: 3, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(3, 16), spectral_efficiency: 0.3770, snr_threshold_db: -2.3, target_ber: 1e-4 },
        ModCodEntry { index: 4, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(1, 4), spectral_efficiency: 0.6016, snr_threshold_db: 0.2, target_ber: 1e-4 },
        ModCodEntry { index: 5, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(1, 3), spectral_efficiency: 0.8770, snr_threshold_db: 2.4, target_ber: 1e-4 },
        ModCodEntry { index: 6, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(1, 2), spectral_efficiency: 1.1758, snr_threshold_db: 4.7, target_ber: 1e-4 },
        ModCodEntry { index: 7, modulation: ModulationType::Qam16, code_rate: CodeRate::new(1, 3), spectral_efficiency: 1.4766, snr_threshold_db: 6.7, target_ber: 1e-4 },
        ModCodEntry { index: 8, modulation: ModulationType::Qam16, code_rate: CodeRate::new(1, 2), spectral_efficiency: 1.9141, snr_threshold_db: 8.7, target_ber: 1e-4 },
        ModCodEntry { index: 9, modulation: ModulationType::Qam16, code_rate: CodeRate::new(2, 3), spectral_efficiency: 2.4063, snr_threshold_db: 10.7, target_ber: 1e-4 },
        ModCodEntry { index: 10, modulation: ModulationType::Qam64, code_rate: CodeRate::new(1, 2), spectral_efficiency: 2.7305, snr_threshold_db: 12.2, target_ber: 1e-4 },
        ModCodEntry { index: 11, modulation: ModulationType::Qam64, code_rate: CodeRate::new(1, 2), spectral_efficiency: 3.3223, snr_threshold_db: 14.1, target_ber: 1e-4 },
        ModCodEntry { index: 12, modulation: ModulationType::Qam64, code_rate: CodeRate::new(2, 3), spectral_efficiency: 3.9023, snr_threshold_db: 15.8, target_ber: 1e-4 },
        ModCodEntry { index: 13, modulation: ModulationType::Qam64, code_rate: CodeRate::new(3, 4), spectral_efficiency: 4.5234, snr_threshold_db: 17.8, target_ber: 1e-4 },
        ModCodEntry { index: 14, modulation: ModulationType::Qam64, code_rate: CodeRate::new(5, 6), spectral_efficiency: 5.1152, snr_threshold_db: 19.8, target_ber: 1e-4 },
        ModCodEntry { index: 15, modulation: ModulationType::Qam64, code_rate: CodeRate::new(9, 10), spectral_efficiency: 5.5547, snr_threshold_db: 21.8, target_ber: 1e-4 },
    ]
}

/// Wi-Fi MCS table (10 entries, 802.11n/ac single stream).
pub fn wifi_mcs_table() -> Vec<ModCodEntry> {
    vec![
        ModCodEntry { index: 0, modulation: ModulationType::Bpsk, code_rate: CodeRate::new(1, 2), spectral_efficiency: 0.5, snr_threshold_db: 2.0, target_ber: 1e-5 },
        ModCodEntry { index: 1, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(1, 2), spectral_efficiency: 1.0, snr_threshold_db: 5.0, target_ber: 1e-5 },
        ModCodEntry { index: 2, modulation: ModulationType::Qpsk, code_rate: CodeRate::new(3, 4), spectral_efficiency: 1.5, snr_threshold_db: 9.0, target_ber: 1e-5 },
        ModCodEntry { index: 3, modulation: ModulationType::Qam16, code_rate: CodeRate::new(1, 2), spectral_efficiency: 2.0, snr_threshold_db: 11.0, target_ber: 1e-5 },
        ModCodEntry { index: 4, modulation: ModulationType::Qam16, code_rate: CodeRate::new(3, 4), spectral_efficiency: 3.0, snr_threshold_db: 15.0, target_ber: 1e-5 },
        ModCodEntry { index: 5, modulation: ModulationType::Qam64, code_rate: CodeRate::new(2, 3), spectral_efficiency: 4.0, snr_threshold_db: 18.0, target_ber: 1e-5 },
        ModCodEntry { index: 6, modulation: ModulationType::Qam64, code_rate: CodeRate::new(3, 4), spectral_efficiency: 4.5, snr_threshold_db: 20.0, target_ber: 1e-5 },
        ModCodEntry { index: 7, modulation: ModulationType::Qam64, code_rate: CodeRate::new(5, 6), spectral_efficiency: 5.0, snr_threshold_db: 25.0, target_ber: 1e-5 },
        ModCodEntry { index: 8, modulation: ModulationType::Qam256, code_rate: CodeRate::new(3, 4), spectral_efficiency: 6.0, snr_threshold_db: 29.0, target_ber: 1e-5 },
        ModCodEntry { index: 9, modulation: ModulationType::Qam256, code_rate: CodeRate::new(5, 6), spectral_efficiency: 6.67, snr_threshold_db: 33.0, target_ber: 1e-5 },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dvb_s2_table_construction() {
        let engine = AdaptiveModCod::with_dvb_s2_table();
        let table = engine.modcod_table();
        assert_eq!(table.len(), 28);
        // First entry should be lowest SNR threshold (QPSK 1/4)
        assert_eq!(table[0].modulation, ModulationType::Qpsk);
        assert!((table[0].snr_threshold_db - (-2.35)).abs() < 0.01);
        // Last entry should be highest SNR threshold (64QAM 9/10)
        assert!((table[27].snr_threshold_db - 16.05).abs() < 0.01);
        // Verify sorted by ascending snr_threshold_db
        for w in table.windows(2) {
            assert!(w[1].snr_threshold_db >= w[0].snr_threshold_db,
                "Table not sorted: {} >= {}", w[1].snr_threshold_db, w[0].snr_threshold_db);
        }
    }

    #[test]
    fn test_low_snr_selects_robust_mode() {
        let engine = AdaptiveModCod::with_dvb_s2_table();
        let entry = engine.select_for_snr(0.0);
        // SNR=0 with backoff should select QPSK 1/4 or 1/3
        assert_eq!(entry.modulation, ModulationType::Qpsk);
        assert!(entry.spectral_efficiency <= 1.0);
    }

    #[test]
    fn test_high_snr_selects_efficient_mode() {
        let engine = AdaptiveModCod::with_dvb_s2_table();
        let entry = engine.select_for_snr(20.0);
        assert_eq!(entry.modulation, ModulationType::Qam64);
        assert!(entry.spectral_efficiency > 5.0);
    }

    #[test]
    fn test_hysteresis_prevents_oscillation() {
        let mut engine = AdaptiveModCod::with_dvb_s2_table();
        engine.config.hysteresis_db = 2.0;

        // Initialize at a specific mode
        let report_init = ChannelQualityReport {
            snr_db: 8.0, ber_estimate: 0.0, mutual_information: 0.0, timestamp_ms: 0,
        };
        engine.update(&report_init);
        let initial_idx = engine.current_index;

        // Oscillate near the threshold
        let threshold = engine.config.modcod_table[initial_idx].snr_threshold_db + engine.config.backoff_db;
        for i in 0..20 {
            let snr = if i % 2 == 0 { threshold - 0.3 } else { threshold + 0.3 };
            let report = ChannelQualityReport {
                snr_db: snr, ber_estimate: 0.0, mutual_information: 0.0, timestamp_ms: i * 100,
            };
            let decision = engine.update(&report);
            // With 2 dB hysteresis, 0.6 dB oscillation should not cause changes
            if i > 2 {
                assert!(!decision.changed, "Mode changed at iteration {}", i);
            }
        }
    }

    #[test]
    fn test_upward_transition_with_hysteresis() {
        let mut engine = AdaptiveModCod::with_dvb_s2_table();
        engine.config.hysteresis_db = 1.0;

        // Start at low SNR
        for _ in 0..5 {
            engine.update(&ChannelQualityReport {
                snr_db: 0.0, ber_estimate: 0.0, mutual_information: 0.0, timestamp_ms: 0,
            });
        }
        let low_idx = engine.current_index;

        // Sustained high SNR well above next threshold
        let mut found_improved = false;
        for i in 0..20 {
            let decision = engine.update(&ChannelQualityReport {
                snr_db: 15.0, ber_estimate: 0.0, mutual_information: 0.0, timestamp_ms: (i + 5) * 100,
            });
            if decision.changed && decision.reason == DecisionReason::SnrImproved {
                found_improved = true;
                break;
            }
        }
        assert!(found_improved, "Should have switched up");
        assert!(engine.current_index > low_idx);
    }

    #[test]
    fn test_downward_transition_on_degradation() {
        let mut engine = AdaptiveModCod::with_dvb_s2_table();
        engine.config.hysteresis_db = 1.0;

        // Start at high SNR
        for _ in 0..10 {
            engine.update(&ChannelQualityReport {
                snr_db: 15.0, ber_estimate: 0.0, mutual_information: 0.0, timestamp_ms: 0,
            });
        }
        let high_idx = engine.current_index;

        // Sudden drop
        let mut found_degraded = false;
        for i in 0..10 {
            let decision = engine.update(&ChannelQualityReport {
                snr_db: 2.0, ber_estimate: 0.0, mutual_information: 0.0, timestamp_ms: (i + 10) * 100,
            });
            if decision.changed && decision.reason == DecisionReason::SnrDegraded {
                found_degraded = true;
                break;
            }
        }
        assert!(found_degraded, "Should have switched down");
        assert!(engine.current_index < high_idx);
    }

    #[test]
    fn test_averaging_smooths_noise() {
        let mut engine = AdaptiveModCod::with_dvb_s2_table();
        engine.config.averaging_window = 10;
        engine.config.hysteresis_db = 1.0;

        // Feed noisy reports around 8 dB
        for i in 0..10 {
            let noise = if i % 2 == 0 { 3.0 } else { -3.0 };
            engine.update(&ChannelQualityReport {
                snr_db: 8.0 + noise, ber_estimate: 0.0, mutual_information: 0.0, timestamp_ms: i * 100,
            });
        }
        let avg = engine.averaged_snr();
        assert!((avg - 8.0).abs() < 2.0, "Average SNR {} too far from 8.0", avg);
    }

    #[test]
    fn test_max_reliability_strategy() {
        let mut engine = AdaptiveModCod::with_dvb_s2_table();
        engine.config.strategy = AdaptationStrategy::MaxReliability;

        let throughput_engine = AdaptiveModCod::with_dvb_s2_table();
        let throughput_entry = throughput_engine.select_for_snr(10.0);

        let decision = engine.update(&ChannelQualityReport {
            snr_db: 10.0, ber_estimate: 0.0, mutual_information: 0.0, timestamp_ms: 0,
        });
        // MaxReliability should pick a less efficient mode
        assert!(decision.entry.spectral_efficiency <= throughput_entry.spectral_efficiency);
    }

    #[test]
    fn test_reset_returns_to_initial() {
        let mut engine = AdaptiveModCod::with_dvb_s2_table();

        for i in 0..10 {
            engine.update(&ChannelQualityReport {
                snr_db: 15.0, ber_estimate: 0.0, mutual_information: 0.0, timestamp_ms: i * 100,
            });
        }
        assert!(engine.current_index > 0);

        engine.reset();
        assert_eq!(engine.current_index, 0);
        assert!(engine.snr_history.is_empty());
        assert!(!engine.initialized);
    }

    #[test]
    fn test_lte_cqi_table() {
        let engine = AdaptiveModCod::with_lte_cqi_table();
        let table = engine.modcod_table();
        assert_eq!(table.len(), 15);
        // CQI 1: QPSK ~0.15 efficiency
        assert_eq!(table[0].modulation, ModulationType::Qpsk);
        assert!((table[0].spectral_efficiency - 0.1523).abs() < 0.01);
        // CQI 15: 64QAM ~5.55 efficiency
        assert_eq!(table[14].modulation, ModulationType::Qam64);
        assert!((table[14].spectral_efficiency - 5.5547).abs() < 0.01);
    }
}

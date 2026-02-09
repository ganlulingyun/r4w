//! # Cognitive Radio Decision Engine
//!
//! Dynamic spectrum access decision engine combining spectrum sensing results,
//! policy databases, and learning algorithms for real-time spectrum access
//! decisions. Implements the OODA loop (Observe-Orient-Decide-Act) for
//! cognitive radio per IEEE 802.22.
//!
//! GNU Radio equivalent: `gr-cognitiva` OOT, `gr-ieee802-22` OOT, DARPA SC2.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cognitive_engine::*;
//!
//! let config = CognitiveEngineConfig {
//!     spectrum_pool: vec![
//!         SpectrumBand { id: 0, center_freq_hz: 470e6, bandwidth_hz: 6e6,
//!                         priority: BandPriority::Secondary,
//!                         regulatory_status: RegulatoryStatus::TvWhiteSpace },
//!     ],
//!     sensing_period_ms: 100,
//!     min_vacancy_s: 1.0,
//!     pu_detection_threshold_db: 10.0,
//!     handoff_margin_db: 3.0,
//!     learning_rate: 0.1,
//!     strategy: AccessStrategy::Greedy,
//! };
//! let mut engine = CognitiveEngine::new(config);
//! let decision = engine.decide();
//! ```

use std::collections::HashMap;

/// A radio frequency band/channel.
#[derive(Debug, Clone)]
pub struct SpectrumBand {
    /// Unique band identifier.
    pub id: u32,
    /// Center frequency in Hz.
    pub center_freq_hz: f64,
    /// Bandwidth in Hz.
    pub bandwidth_hz: f64,
    /// Priority level.
    pub priority: BandPriority,
    /// Regulatory status.
    pub regulatory_status: RegulatoryStatus,
}

/// Band priority level.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BandPriority {
    Primary,
    Secondary,
    Opportunistic,
}

/// Regulatory status of a band.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RegulatoryStatus {
    Licensed,
    Unlicensed,
    TvWhiteSpace,
    CbrsGaa,
    CbrsPal,
}

/// Spectrum occupancy observation.
#[derive(Debug, Clone)]
pub struct OccupancyObservation {
    /// Band identifier.
    pub band_id: u32,
    /// Whether the band is currently occupied.
    pub occupied: bool,
    /// Measured SNR in dB.
    pub snr_db: f64,
    /// Interference power in dBm.
    pub interference_power_dbm: f64,
    /// Timestamp in milliseconds.
    pub timestamp_ms: u64,
}

/// Channel quality score for selection.
#[derive(Debug, Clone)]
pub struct ChannelScore {
    /// Band identifier.
    pub band_id: u32,
    /// Fraction of time available (0..1).
    pub availability: f64,
    /// Average SNR in dB.
    pub average_snr_db: f64,
    /// Expected vacancy duration in seconds.
    pub vacancy_duration_s: f64,
    /// Interference level in dBm.
    pub interference_level_dbm: f64,
    /// Weighted composite score.
    pub composite_score: f64,
}

/// Cognitive engine configuration.
#[derive(Debug, Clone)]
pub struct CognitiveEngineConfig {
    /// Available spectrum bands.
    pub spectrum_pool: Vec<SpectrumBand>,
    /// Sensing period in milliseconds.
    pub sensing_period_ms: u64,
    /// Minimum vacancy time before using a band.
    pub min_vacancy_s: f64,
    /// Primary user detection threshold in dB.
    pub pu_detection_threshold_db: f64,
    /// Handoff margin in dB.
    pub handoff_margin_db: f64,
    /// Learning rate for adaptive algorithms.
    pub learning_rate: f64,
    /// Channel access strategy.
    pub strategy: AccessStrategy,
}

/// Channel access strategy.
#[derive(Debug, Clone, Copy)]
pub enum AccessStrategy {
    /// Always pick the channel with highest composite score.
    Greedy,
    /// Explore with probability epsilon.
    EpsilonGreedy { epsilon: f64 },
    /// Upper Confidence Bound for channel selection.
    Ucb1,
    /// Thompson Sampling with Beta prior.
    ThompsonSampling,
}

/// Spectrum access decision.
#[derive(Debug, Clone)]
pub struct AccessDecision {
    /// Action to take.
    pub action: AccessAction,
    /// Selected band (if any).
    pub selected_band: Option<SpectrumBand>,
    /// Recommended TX power in dBm.
    pub tx_power_dbm: f64,
    /// Reason for the decision.
    pub reason: String,
}

/// Access action types.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccessAction {
    Transmit,
    Handoff,
    Backoff,
    Vacate,
}

/// The cognitive engine.
pub struct CognitiveEngine {
    config: CognitiveEngineConfig,
    occupancy_history: HashMap<u32, Vec<OccupancyObservation>>,
    channel_scores: Vec<ChannelScore>,
    current_band: Option<u32>,
    ucb_counts: HashMap<u32, u64>,
    ucb_rewards: HashMap<u32, f64>,
    total_decisions: u64,
    prng_state: u64,
}

impl CognitiveEngine {
    /// Create a new cognitive engine.
    pub fn new(config: CognitiveEngineConfig) -> Self {
        let band_ids: Vec<u32> = config.spectrum_pool.iter().map(|b| b.id).collect();
        let mut ucb_counts = HashMap::new();
        let mut ucb_rewards = HashMap::new();
        for &id in &band_ids {
            ucb_counts.insert(id, 0);
            ucb_rewards.insert(id, 0.0);
        }

        Self {
            config,
            occupancy_history: HashMap::new(),
            channel_scores: Vec::new(),
            current_band: None,
            ucb_counts,
            ucb_rewards,
            total_decisions: 0,
            prng_state: 42,
        }
    }

    /// Observe spectrum occupancy.
    pub fn observe(&mut self, observations: &[OccupancyObservation]) {
        for obs in observations {
            self.occupancy_history
                .entry(obs.band_id)
                .or_insert_with(Vec::new)
                .push(obs.clone());
        }
        self.update_scores();
    }

    /// Make a spectrum access decision.
    pub fn decide(&mut self) -> AccessDecision {
        self.total_decisions += 1;

        if self.channel_scores.is_empty() {
            self.update_scores();
        }

        // Find available channels (clone to avoid borrow conflict with &mut self)
        let available: Vec<ChannelScore> = self.channel_scores.iter()
            .filter(|s| s.availability > 0.3)
            .cloned()
            .collect();

        if available.is_empty() {
            return AccessDecision {
                action: AccessAction::Backoff,
                selected_band: None,
                tx_power_dbm: -100.0,
                reason: "All bands occupied".into(),
            };
        }

        let selected_id = match self.config.strategy {
            AccessStrategy::Greedy => {
                available.iter()
                    .max_by(|a, b| a.composite_score.partial_cmp(&b.composite_score).unwrap())
                    .unwrap().band_id
            }
            AccessStrategy::EpsilonGreedy { epsilon } => {
                let r = self.next_random_f64();
                if r < epsilon {
                    // Explore: random channel
                    let idx = (self.next_random_f64() * available.len() as f64) as usize;
                    available[idx.min(available.len() - 1)].band_id
                } else {
                    // Exploit: best channel
                    available.iter()
                        .max_by(|a, b| a.composite_score.partial_cmp(&b.composite_score).unwrap())
                        .unwrap().band_id
                }
            }
            AccessStrategy::Ucb1 => {
                self.ucb1_select(&available)
            }
            AccessStrategy::ThompsonSampling => {
                self.thompson_select(&available)
            }
        };

        let band = self.config.spectrum_pool.iter()
            .find(|b| b.id == selected_id)
            .cloned();

        let action = if self.current_band.is_some() && self.current_band != Some(selected_id) {
            AccessAction::Handoff
        } else {
            AccessAction::Transmit
        };

        // Update UCB counters
        *self.ucb_counts.entry(selected_id).or_insert(0) += 1;
        let score = self.channel_scores.iter()
            .find(|s| s.band_id == selected_id)
            .map(|s| s.composite_score)
            .unwrap_or(0.0);
        let count = *self.ucb_counts.get(&selected_id).unwrap_or(&1) as f64;
        let old_reward = *self.ucb_rewards.get(&selected_id).unwrap_or(&0.0);
        self.ucb_rewards.insert(selected_id, old_reward + (score - old_reward) / count);

        self.current_band = Some(selected_id);

        AccessDecision {
            action,
            selected_band: band,
            tx_power_dbm: 20.0,
            reason: format!("Selected band {} with score {:.2}", selected_id, score),
        }
    }

    /// Get current channel scores.
    pub fn channel_scores(&self) -> &[ChannelScore] {
        &self.channel_scores
    }

    /// Get the best channel by composite score.
    pub fn best_channel(&self) -> Option<&ChannelScore> {
        self.channel_scores.iter()
            .max_by(|a, b| a.composite_score.partial_cmp(&b.composite_score).unwrap())
    }

    /// Trigger a handoff to the next best band.
    pub fn trigger_handoff(&mut self, reason: &str) -> AccessDecision {
        self.current_band = None;
        let mut decision = self.decide();
        decision.action = AccessAction::Handoff;
        decision.reason = format!("Handoff: {}", reason);
        decision
    }

    /// Primary user detected on current band.
    pub fn pu_detected_on_current(&mut self) -> AccessDecision {
        let band_id = self.current_band.take();
        AccessDecision {
            action: AccessAction::Vacate,
            selected_band: None,
            tx_power_dbm: -100.0,
            reason: format!("PU detected on band {:?}, vacating", band_id),
        }
    }

    /// Estimate vacancy probability for a band.
    pub fn vacancy_probability(&self, band_id: u32) -> f64 {
        let history = match self.occupancy_history.get(&band_id) {
            Some(h) => h,
            None => return 0.5, // no data, assume 50%
        };

        if history.is_empty() { return 0.5; }

        let vacant_count = history.iter().filter(|o| !o.occupied).count();
        vacant_count as f64 / history.len() as f64
    }

    /// Compute overall spectrum utilization.
    pub fn spectrum_utilization(&self) -> f64 {
        if self.config.spectrum_pool.is_empty() { return 0.0; }

        let mut total_bw_time = 0.0;
        let mut occupied_bw_time = 0.0;

        for band in &self.config.spectrum_pool {
            let history = self.occupancy_history.get(&band.id);
            let n_observations = history.map(|h| h.len()).unwrap_or(0);
            if n_observations == 0 { continue; }
            let n_occupied = history.unwrap().iter().filter(|o| o.occupied).count();

            total_bw_time += band.bandwidth_hz * n_observations as f64;
            occupied_bw_time += band.bandwidth_hz * n_occupied as f64;
        }

        if total_bw_time == 0.0 { return 0.0; }
        occupied_bw_time / total_bw_time
    }

    /// Reset the engine.
    pub fn reset(&mut self) {
        self.occupancy_history.clear();
        self.channel_scores.clear();
        self.current_band = None;
        for v in self.ucb_counts.values_mut() { *v = 0; }
        for v in self.ucb_rewards.values_mut() { *v = 0.0; }
        self.total_decisions = 0;
    }

    /// Get total number of decisions made.
    pub fn total_decisions(&self) -> u64 {
        self.total_decisions
    }

    fn update_scores(&mut self) {
        self.channel_scores.clear();

        for band in &self.config.spectrum_pool {
            let history = self.occupancy_history.get(&band.id);
            let (availability, avg_snr, avg_interference) = match history {
                Some(obs) if !obs.is_empty() => {
                    let n = obs.len() as f64;
                    let vacant = obs.iter().filter(|o| !o.occupied).count() as f64;
                    let snr_sum: f64 = obs.iter().map(|o| o.snr_db).sum();
                    let int_sum: f64 = obs.iter().map(|o| o.interference_power_dbm).sum();
                    (vacant / n, snr_sum / n, int_sum / n)
                }
                _ => (0.5, 0.0, -100.0), // defaults
            };

            // Estimate vacancy duration from consecutive vacant observations
            let vacancy_dur = availability * 10.0; // simplified model

            // Composite score
            let w_a = 0.4;
            let w_s = 0.3;
            let w_v = 0.2;
            let w_i = 0.1;

            let norm_snr = (avg_snr / 30.0).clamp(0.0, 1.0);
            let norm_vacancy = (vacancy_dur / 60.0).clamp(0.0, 1.0);
            let norm_interference = (1.0 - (avg_interference + 100.0) / 100.0).clamp(0.0, 1.0);

            let composite = w_a * availability + w_s * norm_snr
                + w_v * norm_vacancy - w_i * (1.0 - norm_interference);

            self.channel_scores.push(ChannelScore {
                band_id: band.id,
                availability,
                average_snr_db: avg_snr,
                vacancy_duration_s: vacancy_dur,
                interference_level_dbm: avg_interference,
                composite_score: composite,
            });
        }
    }

    fn ucb1_select(&self, available: &[ChannelScore]) -> u32 {
        let n = self.total_decisions.max(1) as f64;

        // First, explore unvisited channels
        for score in available {
            let count = *self.ucb_counts.get(&score.band_id).unwrap_or(&0);
            if count == 0 {
                return score.band_id;
            }
        }

        // UCB1: select by avg_reward + sqrt(2 * ln(N) / n_i)
        let mut best_id = available[0].band_id;
        let mut best_ucb = f64::NEG_INFINITY;

        for score in available {
            let count = *self.ucb_counts.get(&score.band_id).unwrap_or(&1) as f64;
            let avg_reward = *self.ucb_rewards.get(&score.band_id).unwrap_or(&0.0);
            let ucb = avg_reward + (2.0 * n.ln() / count).sqrt();

            if ucb > best_ucb {
                best_ucb = ucb;
                best_id = score.band_id;
            }
        }
        best_id
    }

    fn thompson_select(&mut self, available: &[ChannelScore]) -> u32 {
        // Thompson Sampling with Beta(alpha, beta)
        let mut best_id = available[0].band_id;
        let mut best_sample = f64::NEG_INFINITY;

        for score in available {
            let count = *self.ucb_counts.get(&score.band_id).unwrap_or(&0) as f64;
            let reward = *self.ucb_rewards.get(&score.band_id).unwrap_or(&0.5);
            let alpha = reward * count + 1.0;
            let beta = (1.0 - reward) * count + 1.0;

            // Approximate Beta sample using mean + noise
            let mean = alpha / (alpha + beta);
            let variance = (alpha * beta) / ((alpha + beta).powi(2) * (alpha + beta + 1.0));
            let sample = mean + (self.next_random_f64() - 0.5) * 2.0 * variance.sqrt();

            if sample > best_sample {
                best_sample = sample;
                best_id = score.band_id;
            }
        }
        best_id
    }

    fn next_random_f64(&mut self) -> f64 {
        self.prng_state ^= self.prng_state << 13;
        self.prng_state ^= self.prng_state >> 7;
        self.prng_state ^= self.prng_state << 17;
        (self.prng_state as f64) / (u64::MAX as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config(strategy: AccessStrategy) -> CognitiveEngineConfig {
        CognitiveEngineConfig {
            spectrum_pool: vec![
                SpectrumBand { id: 0, center_freq_hz: 470e6, bandwidth_hz: 6e6,
                    priority: BandPriority::Secondary,
                    regulatory_status: RegulatoryStatus::TvWhiteSpace },
                SpectrumBand { id: 1, center_freq_hz: 476e6, bandwidth_hz: 6e6,
                    priority: BandPriority::Secondary,
                    regulatory_status: RegulatoryStatus::TvWhiteSpace },
                SpectrumBand { id: 2, center_freq_hz: 482e6, bandwidth_hz: 6e6,
                    priority: BandPriority::Secondary,
                    regulatory_status: RegulatoryStatus::TvWhiteSpace },
            ],
            sensing_period_ms: 100,
            min_vacancy_s: 1.0,
            pu_detection_threshold_db: 10.0,
            handoff_margin_db: 3.0,
            learning_rate: 0.1,
            strategy,
        }
    }

    #[test]
    fn test_greedy_selects_best_channel() {
        let mut engine = CognitiveEngine::new(make_config(AccessStrategy::Greedy));

        // Band 1 is clearly best
        engine.observe(&[
            OccupancyObservation { band_id: 0, occupied: true, snr_db: 5.0, interference_power_dbm: -60.0, timestamp_ms: 0 },
            OccupancyObservation { band_id: 1, occupied: false, snr_db: 25.0, interference_power_dbm: -90.0, timestamp_ms: 0 },
            OccupancyObservation { band_id: 2, occupied: false, snr_db: 10.0, interference_power_dbm: -70.0, timestamp_ms: 0 },
        ]);

        let decision = engine.decide();
        assert_eq!(decision.action, AccessAction::Transmit);
        assert_eq!(decision.selected_band.unwrap().id, 1);
    }

    #[test]
    fn test_pu_detection_triggers_vacate() {
        let mut engine = CognitiveEngine::new(make_config(AccessStrategy::Greedy));
        engine.observe(&[
            OccupancyObservation { band_id: 0, occupied: false, snr_db: 20.0, interference_power_dbm: -90.0, timestamp_ms: 0 },
        ]);
        engine.decide(); // Start on some band
        assert!(engine.current_band.is_some());

        let decision = engine.pu_detected_on_current();
        assert_eq!(decision.action, AccessAction::Vacate);
        assert!(engine.current_band.is_none());
    }

    #[test]
    fn test_handoff_to_next_best() {
        let mut engine = CognitiveEngine::new(make_config(AccessStrategy::Greedy));
        engine.observe(&[
            OccupancyObservation { band_id: 0, occupied: false, snr_db: 20.0, interference_power_dbm: -90.0, timestamp_ms: 0 },
            OccupancyObservation { band_id: 1, occupied: false, snr_db: 15.0, interference_power_dbm: -85.0, timestamp_ms: 0 },
        ]);

        engine.decide(); // Initial selection
        engine.pu_detected_on_current(); // Vacate

        let decision = engine.decide();
        assert!(decision.selected_band.is_some());
    }

    #[test]
    fn test_ucb1_explores_all_channels() {
        let mut engine = CognitiveEngine::new(make_config(AccessStrategy::Ucb1));
        engine.observe(&[
            OccupancyObservation { band_id: 0, occupied: false, snr_db: 15.0, interference_power_dbm: -80.0, timestamp_ms: 0 },
            OccupancyObservation { band_id: 1, occupied: false, snr_db: 15.0, interference_power_dbm: -80.0, timestamp_ms: 0 },
            OccupancyObservation { band_id: 2, occupied: false, snr_db: 15.0, interference_power_dbm: -80.0, timestamp_ms: 0 },
        ]);

        let mut selected_ids = std::collections::HashSet::new();
        for _ in 0..3 {
            let decision = engine.decide();
            if let Some(band) = &decision.selected_band {
                selected_ids.insert(band.id);
            }
        }
        // UCB1 should explore all 3 bands first
        assert_eq!(selected_ids.len(), 3, "UCB1 should explore all bands first");
    }

    #[test]
    fn test_ucb1_converges() {
        let mut engine = CognitiveEngine::new(make_config(AccessStrategy::Ucb1));

        // Band 1 is consistently best
        for _ in 0..100 {
            engine.observe(&[
                OccupancyObservation { band_id: 0, occupied: false, snr_db: 5.0, interference_power_dbm: -60.0, timestamp_ms: 0 },
                OccupancyObservation { band_id: 1, occupied: false, snr_db: 25.0, interference_power_dbm: -95.0, timestamp_ms: 0 },
                OccupancyObservation { band_id: 2, occupied: false, snr_db: 10.0, interference_power_dbm: -70.0, timestamp_ms: 0 },
            ]);
            engine.decide();
        }

        // Check that band 1 was selected most often in the last 50
        let count_1 = *engine.ucb_counts.get(&1).unwrap_or(&0);
        let total: u64 = engine.ucb_counts.values().sum();
        assert!(count_1 as f64 / total as f64 > 0.5,
            "Band 1 selected {}% of the time", count_1 as f64 / total as f64 * 100.0);
    }

    #[test]
    fn test_epsilon_greedy_exploration() {
        let mut engine = CognitiveEngine::new(make_config(
            AccessStrategy::EpsilonGreedy { epsilon: 0.5 }
        ));

        for _ in 0..5 {
            engine.observe(&[
                OccupancyObservation { band_id: 0, occupied: false, snr_db: 25.0, interference_power_dbm: -95.0, timestamp_ms: 0 },
                OccupancyObservation { band_id: 1, occupied: false, snr_db: 5.0, interference_power_dbm: -60.0, timestamp_ms: 0 },
                OccupancyObservation { band_id: 2, occupied: false, snr_db: 5.0, interference_power_dbm: -60.0, timestamp_ms: 0 },
            ]);
        }

        let mut counts = HashMap::new();
        for _ in 0..1000 {
            let decision = engine.decide();
            if let Some(band) = decision.selected_band {
                *counts.entry(band.id).or_insert(0u32) += 1;
            }
        }

        // With epsilon=0.5, non-best channels should be selected significantly
        let non_best: u32 = counts.iter()
            .filter(|(&id, _)| id != 0)
            .map(|(_, &c)| c)
            .sum();
        assert!(non_best > 100, "Non-best channels selected only {} times out of 1000", non_best);
    }

    #[test]
    fn test_vacancy_probability() {
        let mut engine = CognitiveEngine::new(make_config(AccessStrategy::Greedy));
        engine.observe(&[
            OccupancyObservation { band_id: 0, occupied: true, snr_db: 0.0, interference_power_dbm: -50.0, timestamp_ms: 0 },
            OccupancyObservation { band_id: 0, occupied: true, snr_db: 0.0, interference_power_dbm: -50.0, timestamp_ms: 100 },
            OccupancyObservation { band_id: 0, occupied: false, snr_db: 10.0, interference_power_dbm: -80.0, timestamp_ms: 200 },
            OccupancyObservation { band_id: 0, occupied: false, snr_db: 10.0, interference_power_dbm: -80.0, timestamp_ms: 300 },
        ]);

        let prob = engine.vacancy_probability(0);
        assert!((prob - 0.5).abs() < 0.01, "Vacancy prob {} != 0.5", prob);
    }

    #[test]
    fn test_all_bands_occupied_backoff() {
        let mut engine = CognitiveEngine::new(make_config(AccessStrategy::Greedy));
        engine.observe(&[
            OccupancyObservation { band_id: 0, occupied: true, snr_db: 0.0, interference_power_dbm: -30.0, timestamp_ms: 0 },
            OccupancyObservation { band_id: 1, occupied: true, snr_db: 0.0, interference_power_dbm: -30.0, timestamp_ms: 0 },
            OccupancyObservation { band_id: 2, occupied: true, snr_db: 0.0, interference_power_dbm: -30.0, timestamp_ms: 0 },
        ]);

        let decision = engine.decide();
        assert_eq!(decision.action, AccessAction::Backoff);
        assert!(decision.selected_band.is_none());
    }

    #[test]
    fn test_spectrum_utilization() {
        let config = CognitiveEngineConfig {
            spectrum_pool: vec![
                SpectrumBand { id: 0, center_freq_hz: 470e6, bandwidth_hz: 10e6,
                    priority: BandPriority::Secondary,
                    regulatory_status: RegulatoryStatus::TvWhiteSpace },
                SpectrumBand { id: 1, center_freq_hz: 480e6, bandwidth_hz: 5e6,
                    priority: BandPriority::Secondary,
                    regulatory_status: RegulatoryStatus::TvWhiteSpace },
                SpectrumBand { id: 2, center_freq_hz: 490e6, bandwidth_hz: 10e6,
                    priority: BandPriority::Secondary,
                    regulatory_status: RegulatoryStatus::TvWhiteSpace },
            ],
            sensing_period_ms: 100,
            min_vacancy_s: 1.0,
            pu_detection_threshold_db: 10.0,
            handoff_margin_db: 3.0,
            learning_rate: 0.1,
            strategy: AccessStrategy::Greedy,
        };
        let mut engine = CognitiveEngine::new(config);

        // Band 0: 50% occupied (1/2), Band 1: 100% (1/1), Band 2: 0% (0/2)
        engine.observe(&[
            OccupancyObservation { band_id: 0, occupied: true, snr_db: 0.0, interference_power_dbm: -50.0, timestamp_ms: 0 },
            OccupancyObservation { band_id: 0, occupied: false, snr_db: 10.0, interference_power_dbm: -80.0, timestamp_ms: 100 },
            OccupancyObservation { band_id: 1, occupied: true, snr_db: 0.0, interference_power_dbm: -40.0, timestamp_ms: 0 },
            OccupancyObservation { band_id: 2, occupied: false, snr_db: 15.0, interference_power_dbm: -90.0, timestamp_ms: 0 },
            OccupancyObservation { band_id: 2, occupied: false, snr_db: 15.0, interference_power_dbm: -90.0, timestamp_ms: 100 },
        ]);

        let util = engine.spectrum_utilization();
        // total_bw_time = 10e6*2 + 5e6*1 + 10e6*2 = 45e6
        // occupied_bw_time = 10e6*1 + 5e6*1 + 10e6*0 = 15e6
        // util = 15/45 = 0.333...
        assert!((util - 1.0 / 3.0).abs() < 0.01,
            "Utilization {} != 0.333", util);
    }

    #[test]
    fn test_reset_clears_history() {
        let mut engine = CognitiveEngine::new(make_config(AccessStrategy::Greedy));
        engine.observe(&[
            OccupancyObservation { band_id: 0, occupied: false, snr_db: 20.0, interference_power_dbm: -90.0, timestamp_ms: 0 },
        ]);
        engine.decide();
        engine.decide();
        assert!(engine.total_decisions > 0);

        engine.reset();
        assert_eq!(engine.total_decisions(), 0);
        assert!(engine.channel_scores().is_empty());
    }
}

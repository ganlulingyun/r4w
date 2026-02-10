//! Reinforcement learning-based spectrum sensing and opportunistic access for cognitive radio.
//!
//! This module provides a [`SpectrumLearner`] that uses Q-learning with epsilon-greedy or
//! UCB1 action selection to discover and exploit vacant spectrum channels. It maintains
//! per-channel occupancy models, a Q-table for state-action values, and throughput history
//! for performance tracking.
//!
//! # Example
//!
//! ```
//! use r4w_core::cognitive_radio_spectrum_learner::*;
//!
//! let config = LearnerConfig {
//!     num_channels: 4,
//!     learning_rate: 0.1,
//!     discount_factor: 0.9,
//!     exploration_rate: 0.3,
//! };
//! let mut learner = SpectrumLearner::new(config);
//!
//! // Feed sensing results for channel 0
//! learner.observe(0, false, 15.0, 0.0);
//! // Select an action using epsilon-greedy
//! let action = learner.select_action(SelectionPolicy::EpsilonGreedy);
//! // Provide a reward and update Q-table
//! learner.update(&action, 1.0);
//!
//! // Check throughput history
//! assert_eq!(learner.throughput_history().len(), 1);
//! ```

use std::collections::HashMap;
use std::fmt;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration parameters for the [`SpectrumLearner`].
#[derive(Debug, Clone)]
pub struct LearnerConfig {
    /// Number of channels to manage.
    pub num_channels: usize,
    /// Q-learning learning rate (alpha, 0..=1).
    pub learning_rate: f64,
    /// Q-learning discount factor (gamma, 0..=1).
    pub discount_factor: f64,
    /// Epsilon for epsilon-greedy exploration (0..=1).
    pub exploration_rate: f64,
}

impl Default for LearnerConfig {
    fn default() -> Self {
        Self {
            num_channels: 8,
            learning_rate: 0.1,
            discount_factor: 0.95,
            exploration_rate: 0.1,
        }
    }
}

// ---------------------------------------------------------------------------
// Channel state
// ---------------------------------------------------------------------------

/// Snapshot of a single channel's state.
#[derive(Debug, Clone)]
pub struct ChannelState {
    /// Centre frequency (Hz) - informational.
    pub frequency: f64,
    /// Whether the channel is currently occupied by a primary user.
    pub occupied: bool,
    /// Most recent measured SNR in dB.
    pub snr_db: f64,
    /// Timestamp (seconds) of the last sensing event.
    pub last_sensed_time: f64,
}

impl ChannelState {
    fn new(frequency: f64) -> Self {
        Self {
            frequency,
            occupied: false,
            snr_db: 0.0,
            last_sensed_time: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Actions
// ---------------------------------------------------------------------------

/// Possible actions the cognitive radio can take.
#[derive(Debug, Clone, PartialEq)]
pub enum Action {
    /// Sense the given channel index.
    Sense(usize),
    /// Transmit on the given channel index at the specified power (dBm).
    Transmit(usize, f64),
    /// Do nothing this time-step.
    Wait,
}

impl fmt::Display for Action {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Action::Sense(ch) => write!(f, "Sense({})", ch),
            Action::Transmit(ch, pwr) => write!(f, "Transmit({}, {:.1} dBm)", ch, pwr),
            Action::Wait => write!(f, "Wait"),
        }
    }
}

// ---------------------------------------------------------------------------
// Selection policy
// ---------------------------------------------------------------------------

/// Strategy used to pick the next action.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SelectionPolicy {
    /// Epsilon-greedy: explore with probability `exploration_rate`, otherwise exploit.
    EpsilonGreedy,
    /// Upper Confidence Bound 1 - balances exploration/exploitation analytically.
    Ucb1,
}

// ---------------------------------------------------------------------------
// Occupancy model
// ---------------------------------------------------------------------------

/// Per-channel occupancy probability tracker built from historical observations.
#[derive(Debug, Clone)]
pub struct OccupancyModel {
    /// Number of times each channel was observed as occupied.
    occupied_count: Vec<u64>,
    /// Total observations per channel.
    total_count: Vec<u64>,
}

impl OccupancyModel {
    fn new(num_channels: usize) -> Self {
        Self {
            occupied_count: vec![0; num_channels],
            total_count: vec![0; num_channels],
        }
    }

    /// Record an observation for `channel`.
    fn record(&mut self, channel: usize, occupied: bool) {
        if channel < self.total_count.len() {
            self.total_count[channel] += 1;
            if occupied {
                self.occupied_count[channel] += 1;
            }
        }
    }

    /// Return the estimated occupancy probability for `channel` (0.0 - 1.0).
    pub fn occupancy_probability(&self, channel: usize) -> f64 {
        if channel >= self.total_count.len() || self.total_count[channel] == 0 {
            return 0.5; // uninformative prior
        }
        self.occupied_count[channel] as f64 / self.total_count[channel] as f64
    }

    /// Return the estimated vacancy probability for `channel` (1 - occupancy).
    pub fn vacancy_probability(&self, channel: usize) -> f64 {
        1.0 - self.occupancy_probability(channel)
    }

    fn reset(&mut self) {
        self.occupied_count.iter_mut().for_each(|v| *v = 0);
        self.total_count.iter_mut().for_each(|v| *v = 0);
    }
}

// ---------------------------------------------------------------------------
// Q-table key
// ---------------------------------------------------------------------------

/// Compact state representation for Q-table indexing.
///
/// We discretise each channel's status to (occupied, snr_bucket) and combine
/// the resulting per-channel tuples into a single `Vec`-based key.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct StateKey(Vec<(bool, i32)>);

fn snr_bucket(snr_db: f64) -> i32 {
    // Quantise to 5 dB buckets
    (snr_db / 5.0).round() as i32
}

// ---------------------------------------------------------------------------
// Simple PRNG (xorshift64) - avoids external crate dependency
// ---------------------------------------------------------------------------

struct Xorshift64 {
    state: u64,
}

impl Xorshift64 {
    fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 { 1 } else { seed },
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    /// Uniform f64 in [0, 1).
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / ((1u64 << 53) as f64)
    }

    /// Uniform usize in [0, n).
    fn next_usize(&mut self, n: usize) -> usize {
        (self.next_f64() * n as f64) as usize % n
    }
}

// ---------------------------------------------------------------------------
// Action indexing helpers
// ---------------------------------------------------------------------------

/// Map an [`Action`] to a flat integer index for the Q-table.
///
/// Layout: `[Sense(0)..Sense(N-1), Transmit(0)..Transmit(N-1), Wait]`
fn action_index(action: &Action, num_channels: usize) -> usize {
    match action {
        Action::Sense(ch) => *ch,
        Action::Transmit(ch, _) => num_channels + *ch,
        Action::Wait => 2 * num_channels,
    }
}

fn num_actions(num_channels: usize) -> usize {
    2 * num_channels + 1
}

fn index_to_action(idx: usize, num_channels: usize) -> Action {
    if idx < num_channels {
        Action::Sense(idx)
    } else if idx < 2 * num_channels {
        Action::Transmit(idx - num_channels, 0.0)
    } else {
        Action::Wait
    }
}

// ---------------------------------------------------------------------------
// Saved Q-table type
// ---------------------------------------------------------------------------

/// Serialisable Q-table representation: maps (state, action_index) to value.
pub type SavedQTable = Vec<(Vec<(bool, i32)>, Vec<f64>)>;

// ---------------------------------------------------------------------------
// SpectrumLearner
// ---------------------------------------------------------------------------

/// Reinforcement learning agent for opportunistic spectrum access.
///
/// Maintains per-channel state, an occupancy model, a Q-table, and throughput
/// tracking. The learner observes sensing results, selects actions, and learns
/// from reward feedback.
pub struct SpectrumLearner {
    config: LearnerConfig,
    channels: Vec<ChannelState>,
    occupancy_model: OccupancyModel,
    q_table: HashMap<StateKey, Vec<f64>>,
    action_counts: HashMap<StateKey, Vec<u64>>,
    total_steps: u64,
    throughput_log: Vec<f64>,
    rng: Xorshift64,
}

impl SpectrumLearner {
    /// Create a new learner. Channel frequencies default to `i * 1e6` Hz.
    pub fn new(config: LearnerConfig) -> Self {
        let n = config.num_channels;
        let channels = (0..n)
            .map(|i| ChannelState::new(i as f64 * 1_000_000.0))
            .collect();
        Self {
            occupancy_model: OccupancyModel::new(n),
            q_table: HashMap::new(),
            action_counts: HashMap::new(),
            total_steps: 0,
            throughput_log: Vec::new(),
            rng: Xorshift64::new(42),
            channels,
            config,
        }
    }

    /// Create a learner with explicit per-channel centre frequencies.
    pub fn with_frequencies(config: LearnerConfig, frequencies: &[f64]) -> Self {
        assert_eq!(
            frequencies.len(),
            config.num_channels,
            "frequency list length must equal num_channels"
        );
        let channels = frequencies
            .iter()
            .map(|&f| ChannelState::new(f))
            .collect();
        let n = config.num_channels;
        Self {
            occupancy_model: OccupancyModel::new(n),
            q_table: HashMap::new(),
            action_counts: HashMap::new(),
            total_steps: 0,
            throughput_log: Vec::new(),
            rng: Xorshift64::new(42),
            channels,
            config,
        }
    }

    // -- Observation --------------------------------------------------------

    /// Feed a sensing result for a single channel.
    pub fn observe(&mut self, channel: usize, occupied: bool, snr_db: f64, time: f64) {
        assert!(channel < self.config.num_channels, "channel index out of range");
        self.channels[channel].occupied = occupied;
        self.channels[channel].snr_db = snr_db;
        self.channels[channel].last_sensed_time = time;
        self.occupancy_model.record(channel, occupied);
    }

    /// Batch observe: supply a slice of `(channel, occupied, snr_db)` tuples
    /// all at the same `time`.
    pub fn observe_batch(&mut self, observations: &[(usize, bool, f64)], time: f64) {
        for &(ch, occ, snr) in observations {
            self.observe(ch, occ, snr, time);
        }
    }

    // -- State key ----------------------------------------------------------

    fn state_key(&self) -> StateKey {
        StateKey(
            self.channels
                .iter()
                .map(|c| (c.occupied, snr_bucket(c.snr_db)))
                .collect(),
        )
    }

    fn q_row(&mut self) -> &mut Vec<f64> {
        let key = self.state_key();
        let n = num_actions(self.config.num_channels);
        self.q_table.entry(key).or_insert_with(|| vec![0.0; n])
    }

    fn q_row_ref(&self) -> Vec<f64> {
        let key = self.state_key();
        let n = num_actions(self.config.num_channels);
        self.q_table.get(&key).cloned().unwrap_or_else(|| vec![0.0; n])
    }

    fn count_row(&mut self) -> &mut Vec<u64> {
        let key = self.state_key();
        let n = num_actions(self.config.num_channels);
        self.action_counts
            .entry(key)
            .or_insert_with(|| vec![0; n])
    }

    // -- Action selection ---------------------------------------------------

    /// Choose an action according to `policy`.
    pub fn select_action(&mut self, policy: SelectionPolicy) -> Action {
        let n = self.config.num_channels;
        match policy {
            SelectionPolicy::EpsilonGreedy => self.select_epsilon_greedy(n),
            SelectionPolicy::Ucb1 => self.select_ucb1(n),
        }
    }

    fn select_epsilon_greedy(&mut self, n: usize) -> Action {
        if self.rng.next_f64() < self.config.exploration_rate {
            // Explore: random action
            let idx = self.rng.next_usize(num_actions(n));
            index_to_action(idx, n)
        } else {
            // Exploit: best Q-value
            let q = self.q_row_ref();
            let best = q
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
                .map(|(i, _)| i)
                .unwrap_or(num_actions(n) - 1);
            index_to_action(best, n)
        }
    }

    fn select_ucb1(&mut self, n: usize) -> Action {
        let q = self.q_row_ref();
        let counts = {
            let key = self.state_key();
            let na = num_actions(n);
            self.action_counts
                .get(&key)
                .cloned()
                .unwrap_or_else(|| vec![0; na])
        };

        let total: u64 = counts.iter().sum::<u64>().max(1);
        let ln_total = (total as f64).ln();

        let best = q
            .iter()
            .zip(counts.iter())
            .enumerate()
            .map(|(i, (&qv, &c))| {
                if c == 0 {
                    (i, f64::MAX) // unvisited - infinite bonus
                } else {
                    let bonus = (2.0 * ln_total / c as f64).sqrt();
                    (i, qv + bonus)
                }
            })
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap().then(b.0.cmp(&a.0)))
            .map(|(i, _)| i)
            .unwrap_or(num_actions(n) - 1);

        index_to_action(best, n)
    }

    // -- Update -------------------------------------------------------------

    /// Perform a Q-learning update given the action that was taken and the
    /// observed reward. Also logs the reward as throughput and increments
    /// action counts.
    pub fn update(&mut self, action: &Action, reward: f64) {
        let n = self.config.num_channels;
        let aidx = action_index(action, n);
        let alpha = self.config.learning_rate;
        let gamma = self.config.discount_factor;

        // Current Q-value
        let old_q = {
            let row = self.q_row();
            row[aidx]
        };

        // Max future Q (from current state - on-policy simplification)
        let max_future = self.q_row_ref().iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        // TD update
        let new_q = old_q + alpha * (reward + gamma * max_future - old_q);
        self.q_row()[aidx] = new_q;

        // Bookkeeping
        self.count_row()[aidx] += 1;
        self.total_steps += 1;
        self.throughput_log.push(reward);
    }

    // -- Prediction ---------------------------------------------------------

    /// Predict which channels are likely to be vacant at a future `time`,
    /// returning channel indices sorted by vacancy probability (highest first).
    pub fn predict_vacancy(&self, _future_time: f64) -> Vec<(usize, f64)> {
        let mut predictions: Vec<(usize, f64)> = (0..self.config.num_channels)
            .map(|ch| (ch, self.occupancy_model.vacancy_probability(ch)))
            .collect();
        predictions.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        predictions
    }

    // -- Accessors ----------------------------------------------------------

    /// Return a reference to the per-channel occupancy model.
    pub fn occupancy_model(&self) -> &OccupancyModel {
        &self.occupancy_model
    }

    /// Return a snapshot of the current channel states.
    pub fn channel_states(&self) -> &[ChannelState] {
        &self.channels
    }

    /// Return a reference to the throughput history (one entry per `update` call).
    pub fn throughput_history(&self) -> &[f64] {
        &self.throughput_log
    }

    /// Expose the learned Q-values as a flat table.
    pub fn get_q_table(&self) -> SavedQTable {
        self.q_table
            .iter()
            .map(|(k, v)| (k.0.clone(), v.clone()))
            .collect()
    }

    /// Load a previously saved Q-table.
    pub fn load_q_table(&mut self, saved: &SavedQTable) {
        self.q_table.clear();
        for (state_vec, values) in saved {
            self.q_table
                .insert(StateKey(state_vec.clone()), values.clone());
        }
    }

    /// Total number of learning steps completed.
    pub fn total_steps(&self) -> u64 {
        self.total_steps
    }

    /// Return the current configuration.
    pub fn config(&self) -> &LearnerConfig {
        &self.config
    }

    /// Reset all learned state (Q-table, occupancy model, throughput log).
    pub fn reset(&mut self) {
        self.q_table.clear();
        self.action_counts.clear();
        self.occupancy_model.reset();
        self.throughput_log.clear();
        self.total_steps = 0;
        for ch in &mut self.channels {
            ch.occupied = false;
            ch.snr_db = 0.0;
            ch.last_sensed_time = 0.0;
        }
    }

    /// Set the exploration rate (epsilon) dynamically.
    pub fn set_exploration_rate(&mut self, epsilon: f64) {
        self.config.exploration_rate = epsilon.clamp(0.0, 1.0);
    }

    /// Seed the internal PRNG for reproducible results.
    pub fn set_seed(&mut self, seed: u64) {
        self.rng = Xorshift64::new(seed);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config(n: usize) -> LearnerConfig {
        LearnerConfig {
            num_channels: n,
            learning_rate: 0.1,
            discount_factor: 0.9,
            exploration_rate: 0.0, // greedy by default for deterministic tests
        }
    }

    #[test]
    fn test_new_learner_has_correct_channel_count() {
        let learner = SpectrumLearner::new(default_config(4));
        assert_eq!(learner.channel_states().len(), 4);
    }

    #[test]
    fn test_observe_updates_channel_state() {
        let mut learner = SpectrumLearner::new(default_config(3));
        learner.observe(1, true, 12.5, 1.0);
        let ch = &learner.channel_states()[1];
        assert!(ch.occupied);
        assert!((ch.snr_db - 12.5).abs() < 1e-9);
        assert!((ch.last_sensed_time - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_observe_batch() {
        let mut learner = SpectrumLearner::new(default_config(4));
        learner.observe_batch(&[(0, false, 10.0), (2, true, 5.0), (3, false, 20.0)], 2.0);
        assert!(!learner.channel_states()[0].occupied);
        assert!(learner.channel_states()[2].occupied);
        assert!(!learner.channel_states()[3].occupied);
    }

    #[test]
    fn test_occupancy_model_tracks_probability() {
        let mut learner = SpectrumLearner::new(default_config(2));
        // Channel 0: occupied 3 out of 4 times
        learner.observe(0, true, 5.0, 0.0);
        learner.observe(0, true, 5.0, 1.0);
        learner.observe(0, true, 5.0, 2.0);
        learner.observe(0, false, 5.0, 3.0);
        let prob = learner.occupancy_model().occupancy_probability(0);
        assert!((prob - 0.75).abs() < 1e-9);
    }

    #[test]
    fn test_vacancy_prediction_sorted_by_probability() {
        let mut learner = SpectrumLearner::new(default_config(3));
        // Channel 0: always occupied, channel 1: never occupied, channel 2: half
        learner.observe(0, true, 5.0, 0.0);
        learner.observe(0, true, 5.0, 1.0);
        learner.observe(1, false, 10.0, 0.0);
        learner.observe(1, false, 10.0, 1.0);
        learner.observe(2, true, 7.0, 0.0);
        learner.observe(2, false, 7.0, 1.0);

        let preds = learner.predict_vacancy(5.0);
        // Channel 1 (vacancy=1.0) should be first
        assert_eq!(preds[0].0, 1);
        assert!((preds[0].1 - 1.0).abs() < 1e-9);
        // Channel 0 (vacancy=0.0) should be last
        assert_eq!(preds[2].0, 0);
        assert!((preds[2].1 - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_greedy_action_selection_exploits_best_q() {
        let mut learner = SpectrumLearner::new(default_config(2));
        learner.observe(0, false, 10.0, 0.0);
        learner.observe(1, true, 5.0, 0.0);

        // Manually inject a Q-value so Transmit(0) is best
        let key = learner.state_key();
        let n = learner.config.num_channels;
        let row = learner
            .q_table
            .entry(key)
            .or_insert_with(|| vec![0.0; num_actions(n)]);
        row[action_index(&Action::Transmit(0, 0.0), n)] = 10.0;

        // Greedy selection should pick Transmit(0)
        let action = learner.select_action(SelectionPolicy::EpsilonGreedy);
        assert_eq!(action, Action::Transmit(0, 0.0));
    }

    #[test]
    fn test_update_modifies_q_table() {
        let mut learner = SpectrumLearner::new(default_config(2));
        learner.observe(0, false, 10.0, 0.0);

        let action = Action::Sense(0);
        learner.update(&action, 5.0);

        let q = learner.get_q_table();
        // Q-table should have at least one entry with non-zero value
        assert!(!q.is_empty());
        let has_nonzero = q.iter().any(|(_, vals)| vals.iter().any(|&v| v != 0.0));
        assert!(has_nonzero, "Q-table should have non-zero entries after update");
    }

    #[test]
    fn test_throughput_history_grows_with_updates() {
        let mut learner = SpectrumLearner::new(default_config(2));
        learner.observe(0, false, 10.0, 0.0);
        assert!(learner.throughput_history().is_empty());

        learner.update(&Action::Sense(0), 1.0);
        learner.update(&Action::Transmit(0, 10.0), 3.0);
        learner.update(&Action::Wait, 0.0);
        assert_eq!(learner.throughput_history().len(), 3);
        assert!((learner.throughput_history()[1] - 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_reset_clears_all_state() {
        let mut learner = SpectrumLearner::new(default_config(3));
        learner.observe(0, true, 15.0, 1.0);
        learner.update(&Action::Sense(0), 2.0);
        assert!(!learner.get_q_table().is_empty());

        learner.reset();
        assert!(learner.get_q_table().is_empty());
        assert!(learner.throughput_history().is_empty());
        assert_eq!(learner.total_steps(), 0);
        // Occupancy model reset to uninformative prior
        assert!((learner.occupancy_model().occupancy_probability(0) - 0.5).abs() < 1e-9);
    }

    #[test]
    fn test_save_and_load_q_table() {
        let mut learner = SpectrumLearner::new(default_config(2));
        learner.observe(0, false, 10.0, 0.0);
        learner.update(&Action::Transmit(0, 5.0), 4.0);

        let saved = learner.get_q_table();
        assert!(!saved.is_empty());

        // Reset and reload
        learner.reset();
        assert!(learner.get_q_table().is_empty());

        learner.load_q_table(&saved);
        let reloaded = learner.get_q_table();
        assert_eq!(reloaded.len(), saved.len());
    }

    #[test]
    fn test_ucb1_picks_unvisited_actions_first() {
        let mut learner = SpectrumLearner::new(default_config(2));
        learner.observe(0, false, 10.0, 0.0);

        // With no prior visits every action has infinite UCB - the first
        // (index 0 = Sense(0)) should be selected.
        let action = learner.select_action(SelectionPolicy::Ucb1);
        assert_eq!(action, Action::Sense(0));
    }

    #[test]
    fn test_action_display() {
        assert_eq!(format!("{}", Action::Sense(2)), "Sense(2)");
        assert_eq!(format!("{}", Action::Transmit(1, 10.0)), "Transmit(1, 10.0 dBm)");
        assert_eq!(format!("{}", Action::Wait), "Wait");
    }

    #[test]
    fn test_with_frequencies_constructor() {
        let config = LearnerConfig {
            num_channels: 3,
            ..Default::default()
        };
        let freqs = [900e6, 915e6, 928e6];
        let learner = SpectrumLearner::with_frequencies(config, &freqs);
        assert!((learner.channel_states()[0].frequency - 900e6).abs() < 1e-3);
        assert!((learner.channel_states()[2].frequency - 928e6).abs() < 1e-3);
    }

    #[test]
    fn test_set_exploration_rate_clamped() {
        let mut learner = SpectrumLearner::new(default_config(2));
        learner.set_exploration_rate(1.5);
        assert!((learner.config().exploration_rate - 1.0).abs() < 1e-9);
        learner.set_exploration_rate(-0.5);
        assert!((learner.config().exploration_rate - 0.0).abs() < 1e-9);
    }
}

//! Burst Gating Controller
//!
//! Conditional gating/muting of sample streams based on burst detection events,
//! with configurable pre/post-burst padding and gap bridging. The controller
//! operates as a state machine (Idle → PreBurst → Active → PostBurst) and can
//! either pass samples during bursts (default) or mute them.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::burst_gating_controller::{BurstGatingController, GatingConfig, GateMode};
//!
//! let config = GatingConfig {
//!     pre_burst_samples: 0,
//!     post_burst_samples: 0,
//!     min_burst_length: 1,
//!     max_gap: 0,
//!     mode: GateMode::PassDuringBurst,
//! };
//! let mut controller = BurstGatingController::new(config);
//!
//! // Process samples: first 3 are idle, then a burst of 4
//! let samples = vec![0.0_f64; 3];
//! let out = controller.process(&samples, false);
//! assert!(out.iter().all(|&s| s == 0.0));
//!
//! let burst = vec![1.0_f64; 4];
//! let out = controller.process(&burst, true);
//! // Burst samples pass through
//! assert_eq!(out, vec![1.0, 1.0, 1.0, 1.0]);
//!
//! // End the burst so it gets counted
//! let _ = controller.process(&[0.0], false);
//! assert_eq!(controller.statistics().burst_count, 1);
//! ```

use std::collections::VecDeque;
use std::fmt;

/// Gating mode: determines whether bursts are passed through or muted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateMode {
    /// Pass samples during a burst, zero-fill otherwise (default).
    PassDuringBurst,
    /// Mute (zero) samples during a burst, pass otherwise.
    MuteDuringBurst,
}

impl Default for GateMode {
    fn default() -> Self {
        Self::PassDuringBurst
    }
}

/// Configuration for the burst gating controller.
#[derive(Debug, Clone)]
pub struct GatingConfig {
    /// Number of samples to include before the burst trigger.
    pub pre_burst_samples: usize,
    /// Number of samples to include after the burst ends.
    pub post_burst_samples: usize,
    /// Minimum number of active samples required for a burst to count.
    /// Bursts shorter than this are suppressed.
    pub min_burst_length: usize,
    /// Maximum gap (in samples) between bursts that will be bridged.
    /// If the gap between two bursts is less than or equal to `max_gap`,
    /// the gate stays open through the gap.
    pub max_gap: usize,
    /// Gate mode: pass-during-burst or mute-during-burst.
    pub mode: GateMode,
}

impl Default for GatingConfig {
    fn default() -> Self {
        Self {
            pre_burst_samples: 0,
            post_burst_samples: 0,
            min_burst_length: 1,
            max_gap: 0,
            mode: GateMode::PassDuringBurst,
        }
    }
}

/// State machine states for the gating controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateState {
    /// No burst detected; gate is closed (or open, depending on mode).
    Idle,
    /// Pre-burst padding region: gate is opening.
    PreBurst,
    /// Burst is actively detected.
    Active,
    /// Post-burst padding region: gate is closing.
    PostBurst,
}

impl fmt::Display for GateState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GateState::Idle => write!(f, "Idle"),
            GateState::PreBurst => write!(f, "PreBurst"),
            GateState::Active => write!(f, "Active"),
            GateState::PostBurst => write!(f, "PostBurst"),
        }
    }
}

/// Event emitted on gate transitions.
#[derive(Debug, Clone, PartialEq)]
pub struct GateEvent {
    /// Sample index (absolute, since creation/reset) when the event occurred.
    pub index: u64,
    /// The kind of transition.
    pub kind: GateEventKind,
    /// Length of the burst in samples (populated on `GateClose`; 0 on `GateOpen`).
    pub burst_length: u64,
}

/// Kind of gate transition event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateEventKind {
    /// Gate opened (burst detected).
    GateOpen,
    /// Gate closed (burst ended, including post-burst padding).
    GateClose,
}

/// Accumulated statistics for gating operations.
#[derive(Debug, Clone, PartialEq)]
pub struct GatingStatistics {
    /// Number of complete bursts detected.
    pub burst_count: u64,
    /// Total number of samples that were gated (passed through in PassDuringBurst,
    /// or muted in MuteDuringBurst).
    pub total_gated_samples: u64,
    /// Total number of samples processed.
    pub total_samples: u64,
    /// Average burst length in samples (0.0 if no bursts).
    pub average_burst_length: f64,
}

/// Burst Gating Controller.
///
/// Implements a state machine that gates (passes or mutes) sample streams
/// based on an external `burst_active` flag. Supports pre/post-burst
/// padding, gap bridging, minimum burst length filtering, and transition
/// event logging.
pub struct BurstGatingController {
    config: GatingConfig,
    state: GateState,
    /// Ring buffer for pre-burst lookback.
    pre_buffer: VecDeque<f64>,
    /// Samples accumulated in the current burst (for min_burst_length check).
    current_burst_length: u64,
    /// Counter for post-burst countdown.
    post_remaining: usize,
    /// Counter for gap bridging: how many idle samples since last burst.
    gap_counter: usize,
    /// Absolute sample index since creation/reset.
    sample_index: u64,
    /// Index where the current gate opened.
    gate_open_index: u64,
    /// Total burst lengths (sum) for average computation.
    total_burst_length_sum: u64,

    // Statistics
    burst_count: u64,
    total_gated_samples: u64,
    total_samples: u64,

    /// Accumulated events log.
    events: Vec<GateEvent>,
}

impl BurstGatingController {
    /// Create a new controller with the given configuration.
    pub fn new(config: GatingConfig) -> Self {
        let pre_cap = config.pre_burst_samples;
        Self {
            config,
            state: GateState::Idle,
            pre_buffer: VecDeque::with_capacity(pre_cap),
            current_burst_length: 0,
            post_remaining: 0,
            gap_counter: 0,
            sample_index: 0,
            gate_open_index: 0,
            total_burst_length_sum: 0,
            burst_count: 0,
            total_gated_samples: 0,
            total_samples: 0,
            events: Vec::new(),
        }
    }

    /// Return the current state of the controller.
    pub fn state(&self) -> GateState {
        self.state
    }

    /// Return collected gate transition events.
    pub fn events(&self) -> &[GateEvent] {
        &self.events
    }

    /// Return accumulated statistics.
    pub fn statistics(&self) -> GatingStatistics {
        GatingStatistics {
            burst_count: self.burst_count,
            total_gated_samples: self.total_gated_samples,
            total_samples: self.total_samples,
            average_burst_length: if self.burst_count == 0 {
                0.0
            } else {
                self.total_burst_length_sum as f64 / self.burst_count as f64
            },
        }
    }

    /// Reset the controller to its initial state, clearing all statistics
    /// and events.
    pub fn reset(&mut self) {
        self.state = GateState::Idle;
        self.pre_buffer.clear();
        self.current_burst_length = 0;
        self.post_remaining = 0;
        self.gap_counter = 0;
        self.sample_index = 0;
        self.gate_open_index = 0;
        self.total_burst_length_sum = 0;
        self.burst_count = 0;
        self.total_gated_samples = 0;
        self.total_samples = 0;
        self.events.clear();
    }

    /// Process a block of samples with a uniform `burst_active` flag.
    ///
    /// Returns gated output samples (same length as input). In
    /// `PassDuringBurst` mode, samples are passed through when the gate is
    /// open and zeroed when closed. In `MuteDuringBurst` mode, the opposite
    /// applies.
    pub fn process(&mut self, samples: &[f64], burst_active: bool) -> Vec<f64> {
        let mut output = Vec::with_capacity(samples.len());
        for &sample in samples {
            let out = self.process_one(sample, burst_active);
            output.push(out);
        }
        output
    }

    /// Process a single sample. Returns the gated output value.
    fn process_one(&mut self, sample: f64, burst_active: bool) -> f64 {
        self.total_samples += 1;
        let idx = self.sample_index;
        self.sample_index += 1;

        // Determine whether the gate is open after this sample.
        let gate_open = self.advance_state(sample, burst_active, idx);

        let pass = match self.config.mode {
            GateMode::PassDuringBurst => gate_open,
            GateMode::MuteDuringBurst => !gate_open,
        };

        if pass {
            self.total_gated_samples += 1;
            sample
        } else {
            0.0
        }
    }

    /// Core state-machine advance. Returns true if the gate is considered open.
    fn advance_state(&mut self, sample: f64, burst_active: bool, idx: u64) -> bool {
        match self.state {
            GateState::Idle => {
                if burst_active {
                    // Transition: Idle -> Active (possibly via PreBurst).
                    self.state = GateState::Active;
                    self.current_burst_length = 1;
                    self.gate_open_index =
                        idx.saturating_sub(self.config.pre_burst_samples as u64);
                    self.events.push(GateEvent {
                        index: self.gate_open_index,
                        kind: GateEventKind::GateOpen,
                        burst_length: 0,
                    });
                    // Flush pre-buffer (those samples are already emitted as zeros
                    // in previous calls; the pre-burst padding only affects future
                    // extract_bursts calls). Mark gate open now.
                    true
                } else {
                    // Stay idle; maintain pre-buffer for potential future burst.
                    if self.config.pre_burst_samples > 0 {
                        if self.pre_buffer.len() >= self.config.pre_burst_samples {
                            self.pre_buffer.pop_front();
                        }
                        self.pre_buffer.push_back(sample);
                    }
                    false
                }
            }
            GateState::PreBurst => {
                // PreBurst is used internally by extract_bursts; in the
                // process() path we jump directly to Active.
                // This branch handles it gracefully if it's ever entered.
                self.state = GateState::Active;
                self.current_burst_length = 1;
                true
            }
            GateState::Active => {
                if burst_active {
                    self.current_burst_length += 1;
                    self.gap_counter = 0;
                    true
                } else {
                    // Burst signal dropped. Check for gap bridging.
                    self.gap_counter += 1;
                    if self.gap_counter <= self.config.max_gap {
                        // Bridge the gap: stay Active.
                        self.current_burst_length += 1;
                        true
                    } else {
                        // Gap exceeded: begin post-burst countdown.
                        if self.config.post_burst_samples > 0 {
                            self.state = GateState::PostBurst;
                            self.post_remaining = self.config.post_burst_samples - 1;
                        } else {
                            self.finish_burst(idx);
                        }
                        true
                    }
                }
            }
            GateState::PostBurst => {
                if burst_active {
                    // New burst during post-burst: re-enter Active.
                    self.state = GateState::Active;
                    self.current_burst_length += 1;
                    self.gap_counter = 0;
                    self.post_remaining = 0;
                    true
                } else if self.post_remaining > 0 {
                    self.post_remaining -= 1;
                    true
                } else {
                    // Post-burst done: close gate.
                    self.finish_burst(idx);
                    false
                }
            }
        }
    }

    /// Finalize the current burst: emit close event, update statistics.
    fn finish_burst(&mut self, idx: u64) {
        let burst_len = self.current_burst_length;
        if burst_len >= self.config.min_burst_length as u64 {
            self.burst_count += 1;
            self.total_burst_length_sum += burst_len;
        }
        self.events.push(GateEvent {
            index: idx,
            kind: GateEventKind::GateClose,
            burst_length: burst_len,
        });
        self.current_burst_length = 0;
        self.gap_counter = 0;
        self.state = GateState::Idle;
        self.pre_buffer.clear();
    }

    /// Extract burst segments from a pre-recorded stream of samples with
    /// per-sample `burst_active` flags.
    ///
    /// Returns a `Vec` of burst segments, where each segment includes
    /// pre-burst and post-burst padding (clamped to stream boundaries).
    pub fn extract_bursts(
        config: &GatingConfig,
        samples: &[f64],
        burst_flags: &[bool],
    ) -> Vec<BurstSegment> {
        assert_eq!(
            samples.len(),
            burst_flags.len(),
            "samples and burst_flags must have the same length"
        );

        // First pass: find raw burst regions (start, end) with gap bridging.
        let mut regions: Vec<(usize, usize)> = Vec::new();
        let mut in_burst = false;
        let mut start = 0usize;
        let mut gap = 0usize;

        for (i, &active) in burst_flags.iter().enumerate() {
            if active {
                if !in_burst {
                    in_burst = true;
                    start = i;
                    gap = 0;
                } else {
                    gap = 0;
                }
            } else if in_burst {
                gap += 1;
                if gap > config.max_gap {
                    // End this region at the sample before the gap exceeded.
                    let end = i - gap;
                    regions.push((start, end));
                    in_burst = false;
                    gap = 0;
                }
            }
        }
        if in_burst {
            // Close final region. Exclude trailing gap samples.
            let end = if gap > 0 {
                samples.len() - 1 - gap
            } else {
                samples.len() - 1
            };
            regions.push((start, end));
        }

        // Second pass: apply min_burst_length filter, pre/post padding, and extract.
        let mut segments = Vec::new();
        for (raw_start, raw_end) in regions {
            let burst_len = raw_end - raw_start + 1;
            if burst_len < config.min_burst_length {
                continue;
            }
            let padded_start = raw_start.saturating_sub(config.pre_burst_samples);
            let padded_end = (raw_end + config.post_burst_samples).min(samples.len() - 1);

            segments.push(BurstSegment {
                start_index: padded_start,
                end_index: padded_end,
                burst_start: raw_start,
                burst_end: raw_end,
                samples: samples[padded_start..=padded_end].to_vec(),
            });
        }

        segments
    }
}

/// A burst segment extracted from a continuous stream.
#[derive(Debug, Clone, PartialEq)]
pub struct BurstSegment {
    /// Start index in the original stream (including pre-burst padding).
    pub start_index: usize,
    /// End index in the original stream (including post-burst padding).
    pub end_index: usize,
    /// Start index of the actual burst (without padding).
    pub burst_start: usize,
    /// End index of the actual burst (without padding).
    pub burst_end: usize,
    /// Extracted samples.
    pub samples: Vec<f64>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_idle_produces_zeros() {
        let config = GatingConfig::default();
        let mut ctrl = BurstGatingController::new(config);
        let out = ctrl.process(&[1.0, 2.0, 3.0], false);
        assert_eq!(out, vec![0.0, 0.0, 0.0]);
        assert_eq!(ctrl.state(), GateState::Idle);
    }

    #[test]
    fn test_burst_passes_through() {
        let config = GatingConfig::default();
        let mut ctrl = BurstGatingController::new(config);
        let out = ctrl.process(&[1.0, 2.0, 3.0], true);
        assert_eq!(out, vec![1.0, 2.0, 3.0]);
        assert_eq!(ctrl.state(), GateState::Active);
    }

    #[test]
    fn test_mute_during_burst_mode() {
        let config = GatingConfig {
            mode: GateMode::MuteDuringBurst,
            ..Default::default()
        };
        let mut ctrl = BurstGatingController::new(config);
        // Idle: samples pass through in MuteDuringBurst mode.
        let out = ctrl.process(&[1.0, 2.0], false);
        assert_eq!(out, vec![1.0, 2.0]);
        // Active: samples are muted.
        let out = ctrl.process(&[3.0, 4.0], true);
        assert_eq!(out, vec![0.0, 0.0]);
    }

    #[test]
    fn test_post_burst_padding() {
        let config = GatingConfig {
            post_burst_samples: 3,
            ..Default::default()
        };
        let mut ctrl = BurstGatingController::new(config);
        // Burst active.
        let _ = ctrl.process(&[1.0, 2.0], true);
        assert_eq!(ctrl.state(), GateState::Active);
        // Burst ends, enters PostBurst.
        let out = ctrl.process(&[10.0, 20.0, 30.0, 40.0, 50.0], false);
        // First 3 should pass (post-burst padding), then gate closes, then 0.
        // Sample 1: Active->PostBurst (gap_counter=1 > max_gap=0), post_remaining=2, gate open -> 10.0
        // Sample 2: PostBurst, post_remaining=1, gate open -> 20.0
        // Sample 3: PostBurst, post_remaining=0, gate open -> 30.0
        // Sample 4: PostBurst done, finish_burst, gate closed -> 0.0
        // Sample 5: Idle, gate closed -> 0.0
        assert_eq!(out, vec![10.0, 20.0, 30.0, 0.0, 0.0]);
    }

    #[test]
    fn test_gap_bridging() {
        let config = GatingConfig {
            max_gap: 3,
            ..Default::default()
        };
        let mut ctrl = BurstGatingController::new(config);
        // Start a burst.
        let _ = ctrl.process(&[1.0], true);
        // Gap of 3 samples (within max_gap).
        let out = ctrl.process(&[0.5, 0.5, 0.5], false);
        // Gate stays open, samples pass through.
        assert_eq!(out, vec![0.5, 0.5, 0.5]);
        assert_eq!(ctrl.state(), GateState::Active);
        // Resume burst.
        let out = ctrl.process(&[2.0], true);
        assert_eq!(out, vec![2.0]);
    }

    #[test]
    fn test_gap_exceeded_closes_gate() {
        let config = GatingConfig {
            max_gap: 2,
            ..Default::default()
        };
        let mut ctrl = BurstGatingController::new(config);
        let _ = ctrl.process(&[1.0], true);
        // Gap of 3 > max_gap of 2: gate should close.
        let out = ctrl.process(&[0.5, 0.5, 0.5], false);
        // gap=1 -> bridged (open), gap=2 -> bridged (open),
        // gap=3 -> exceeds -> transition to Idle, but still returns true for this sample
        // because the post_burst_samples=0 finish_burst transitions happen and the
        // sample at the transition point is still gated open.
        assert_eq!(out[0], 0.5); // gap=1, bridged
        assert_eq!(out[1], 0.5); // gap=2, bridged
        // gap=3 exceeds: finish_burst called, returns true (gate open for this last sample)
        assert_eq!(out[2], 0.5);
    }

    #[test]
    fn test_statistics_burst_count() {
        let config = GatingConfig::default();
        let mut ctrl = BurstGatingController::new(config);
        // First burst.
        let _ = ctrl.process(&[1.0, 1.0, 1.0], true);
        let _ = ctrl.process(&[0.0], false);
        let stats = ctrl.statistics();
        assert_eq!(stats.burst_count, 1);
        // Second burst.
        let _ = ctrl.process(&[2.0, 2.0], true);
        let _ = ctrl.process(&[0.0], false);
        let stats = ctrl.statistics();
        assert_eq!(stats.burst_count, 2);
        assert_eq!(stats.total_samples, 7);
    }

    #[test]
    fn test_statistics_average_burst_length() {
        let config = GatingConfig::default();
        let mut ctrl = BurstGatingController::new(config);
        // Burst of length 4.
        let _ = ctrl.process(&[1.0; 4], true);
        let _ = ctrl.process(&[0.0], false);
        // Burst of length 6.
        let _ = ctrl.process(&[2.0; 6], true);
        let _ = ctrl.process(&[0.0], false);
        let stats = ctrl.statistics();
        assert_eq!(stats.burst_count, 2);
        assert!((stats.average_burst_length - 5.0).abs() < 1e-9);
    }

    #[test]
    fn test_events_log() {
        let config = GatingConfig::default();
        let mut ctrl = BurstGatingController::new(config);
        let _ = ctrl.process(&[1.0, 1.0], true);
        let _ = ctrl.process(&[0.0], false);
        let events = ctrl.events();
        assert_eq!(events.len(), 2);
        assert_eq!(events[0].kind, GateEventKind::GateOpen);
        assert_eq!(events[1].kind, GateEventKind::GateClose);
        assert_eq!(events[1].burst_length, 2);
    }

    #[test]
    fn test_reset() {
        let config = GatingConfig::default();
        let mut ctrl = BurstGatingController::new(config);
        let _ = ctrl.process(&[1.0, 1.0], true);
        let _ = ctrl.process(&[0.0], false);
        assert_eq!(ctrl.statistics().burst_count, 1);
        ctrl.reset();
        assert_eq!(ctrl.statistics().burst_count, 0);
        assert_eq!(ctrl.statistics().total_samples, 0);
        assert!(ctrl.events().is_empty());
        assert_eq!(ctrl.state(), GateState::Idle);
    }

    #[test]
    fn test_extract_bursts_basic() {
        let config = GatingConfig {
            pre_burst_samples: 2,
            post_burst_samples: 2,
            min_burst_length: 1,
            max_gap: 0,
            mode: GateMode::PassDuringBurst,
        };
        let samples: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let mut flags = vec![false; 20];
        // Burst from index 5..=9
        for i in 5..=9 {
            flags[i] = true;
        }
        let bursts = BurstGatingController::extract_bursts(&config, &samples, &flags);
        assert_eq!(bursts.len(), 1);
        let seg = &bursts[0];
        // Pre-padding: 5-2=3, post-padding: 9+2=11
        assert_eq!(seg.start_index, 3);
        assert_eq!(seg.end_index, 11);
        assert_eq!(seg.burst_start, 5);
        assert_eq!(seg.burst_end, 9);
        assert_eq!(
            seg.samples,
            vec![3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0]
        );
    }

    #[test]
    fn test_extract_bursts_min_length_filter() {
        let config = GatingConfig {
            min_burst_length: 5,
            ..Default::default()
        };
        let samples = vec![1.0; 10];
        let mut flags = vec![false; 10];
        // Short burst of 3 samples — should be filtered out.
        flags[2] = true;
        flags[3] = true;
        flags[4] = true;
        let bursts = BurstGatingController::extract_bursts(&config, &samples, &flags);
        assert!(bursts.is_empty());
    }

    #[test]
    fn test_extract_bursts_gap_bridging() {
        let config = GatingConfig {
            max_gap: 2,
            min_burst_length: 1,
            ..Default::default()
        };
        let samples = vec![1.0; 15];
        let mut flags = vec![false; 15];
        // Burst 1: indices 2..=4
        for i in 2..=4 {
            flags[i] = true;
        }
        // Gap of 2 at indices 5,6 — should be bridged.
        // Burst 2: indices 7..=9
        for i in 7..=9 {
            flags[i] = true;
        }
        let bursts = BurstGatingController::extract_bursts(&config, &samples, &flags);
        // Should be merged into one segment because gap <= max_gap.
        assert_eq!(bursts.len(), 1);
        assert_eq!(bursts[0].burst_start, 2);
        assert_eq!(bursts[0].burst_end, 9);
    }

    #[test]
    fn test_post_burst_reactivation() {
        let config = GatingConfig {
            post_burst_samples: 5,
            ..Default::default()
        };
        let mut ctrl = BurstGatingController::new(config);
        // Start burst.
        let _ = ctrl.process(&[1.0], true);
        // End burst, enter PostBurst.
        let _ = ctrl.process(&[2.0], false);
        assert_eq!(ctrl.state(), GateState::PostBurst);
        // Re-activate during post-burst.
        let out = ctrl.process(&[3.0], true);
        assert_eq!(out, vec![3.0]);
        assert_eq!(ctrl.state(), GateState::Active);
    }

    #[test]
    fn test_gate_state_display() {
        assert_eq!(format!("{}", GateState::Idle), "Idle");
        assert_eq!(format!("{}", GateState::PreBurst), "PreBurst");
        assert_eq!(format!("{}", GateState::Active), "Active");
        assert_eq!(format!("{}", GateState::PostBurst), "PostBurst");
    }
}

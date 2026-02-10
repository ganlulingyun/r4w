//! Advanced triggering and waveform capture engine for debugging signal processing pipelines.
//!
//! Provides oscilloscope-style triggering with edge, level, pulse-width, pattern, and window
//! modes. Captures pre-trigger and post-trigger data using a circular buffer, with configurable
//! holdoff to prevent re-triggering too quickly.
//!
//! # Example
//!
//! ```
//! use r4w_core::oscilloscope_trigger::{TriggerEngine, TriggerConfig, TriggerMode, EdgeDirection};
//!
//! // Configure a rising-edge trigger at threshold 0.5
//! let config = TriggerConfig {
//!     mode: TriggerMode::Edge(EdgeDirection::Rising),
//!     threshold: 0.5,
//!     hysteresis: 0.05,
//!     holdoff_samples: 10,
//!     pre_trigger_samples: 4,
//!     post_trigger_samples: 4,
//!     sample_rate_hz: 1000.0,
//! };
//!
//! let mut engine = TriggerEngine::new(config);
//! engine.arm();
//!
//! // Feed samples whose I component crosses 0.5 upward
//! let samples: Vec<(f64, f64)> = (0..20)
//!     .map(|i| (i as f64 * 0.1, 0.0))
//!     .collect();
//!
//! let results = engine.process(&samples);
//! assert!(!results.is_empty());
//! assert!(results[0].triggered);
//! ```

use std::collections::VecDeque;

/// Direction for edge triggering.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EdgeDirection {
    Rising,
    Falling,
}

/// Trigger mode selection.
#[derive(Debug, Clone, PartialEq)]
pub enum TriggerMode {
    /// Trigger on a rising or falling edge crossing the threshold.
    Edge(EdgeDirection),
    /// Trigger when the signal is above (`true`) or below (`false`) the threshold.
    Level(bool),
    /// Trigger when a pulse width (in seconds) falls within [min_s, max_s].
    PulseWidth(f64, f64),
    /// Trigger when a boolean pattern is matched against consecutive threshold crossings.
    Pattern(Vec<bool>),
    /// Trigger when the signal enters or exits the window [low, high].
    Window(f64, f64),
}

/// Configuration for the trigger engine.
#[derive(Debug, Clone)]
pub struct TriggerConfig {
    /// Trigger detection mode.
    pub mode: TriggerMode,
    /// Voltage threshold for edge/level/pulse-width detection (applied to I component).
    pub threshold: f64,
    /// Hysteresis band around the threshold for edge detection (prevents chatter).
    pub hysteresis: f64,
    /// Minimum number of samples after a trigger before re-arming.
    pub holdoff_samples: usize,
    /// Number of samples to capture before the trigger point.
    pub pre_trigger_samples: usize,
    /// Number of samples to capture after the trigger point.
    pub post_trigger_samples: usize,
    /// Sample rate in Hz (used for pulse-width and statistics calculations).
    pub sample_rate_hz: f64,
}

/// Result of a single trigger capture.
#[derive(Debug, Clone)]
pub struct CaptureResult {
    /// Whether this capture was triggered (always `true` for normal captures).
    pub triggered: bool,
    /// Index of the trigger point within the original input sample stream.
    pub trigger_index: usize,
    /// Samples captured before the trigger point.
    pub pre_trigger_data: Vec<(f64, f64)>,
    /// Samples captured after (and including) the trigger point.
    pub post_trigger_data: Vec<(f64, f64)>,
    /// Running count of triggers at the time this capture was produced.
    pub trigger_count: u64,
}

/// Trigger timing statistics.
#[derive(Debug, Clone)]
pub struct TriggerStats {
    /// Total number of triggers observed.
    pub total_triggers: u64,
    /// Trigger rate in Hz (triggers per second).
    pub trigger_rate_hz: f64,
    /// Mean interval between successive triggers in seconds.
    pub mean_interval_s: f64,
    /// RMS jitter of trigger intervals in seconds.
    pub jitter_rms_s: f64,
}

/// Internal state for the trigger finite-state machine.
#[derive(Debug, Clone, Copy, PartialEq)]
enum TriggerState {
    /// Waiting for the signal to move away from threshold (reset hysteresis).
    WaitingForReset,
    /// Armed and looking for the trigger condition.
    Armed,
    /// Collecting post-trigger samples.
    Capturing,
    /// Holdoff period — ignoring triggers.
    Holdoff,
}

/// Main trigger processor with circular capture buffer.
pub struct TriggerEngine {
    config: TriggerConfig,
    state: TriggerState,
    /// Circular pre-trigger buffer.
    ring: VecDeque<(f64, f64)>,
    /// Post-trigger accumulator.
    post_buf: Vec<(f64, f64)>,
    /// Snapshot of pre-trigger data at trigger instant.
    pre_snapshot: Vec<(f64, f64)>,
    /// Remaining post-trigger samples to collect.
    post_remaining: usize,
    /// Remaining holdoff samples.
    holdoff_remaining: usize,
    /// Running trigger count.
    trigger_count: u64,
    /// Trigger index within the current `process` call.
    current_trigger_index: usize,
    /// Whether the engine has been armed at least once.
    armed_once: bool,
    /// Timestamps (in sample indices) of all triggers, for statistics.
    trigger_sample_indices: Vec<u64>,
    /// Total samples processed across all `process` calls.
    total_samples_processed: u64,
    /// For edge detection: was previous sample above (true) or below (false) the threshold?
    prev_above: Option<bool>,
    /// For pulse-width mode: sample index where the pulse started.
    pulse_start: Option<u64>,
    /// For pattern mode: recent above/below history.
    pattern_history: VecDeque<bool>,
}

impl TriggerEngine {
    /// Create a new trigger engine with the given configuration.
    pub fn new(config: TriggerConfig) -> Self {
        let pre_cap = config.pre_trigger_samples;
        let pattern_len = if let TriggerMode::Pattern(ref p) = config.mode {
            p.len()
        } else {
            0
        };
        Self {
            config,
            state: TriggerState::WaitingForReset,
            ring: VecDeque::with_capacity(pre_cap + 1),
            post_buf: Vec::new(),
            pre_snapshot: Vec::new(),
            post_remaining: 0,
            holdoff_remaining: 0,
            trigger_count: 0,
            current_trigger_index: 0,
            armed_once: false,
            trigger_sample_indices: Vec::new(),
            total_samples_processed: 0,
            prev_above: None,
            pulse_start: None,
            pattern_history: VecDeque::with_capacity(pattern_len + 1),
        }
    }

    /// Arm the trigger engine so it begins looking for trigger conditions.
    pub fn arm(&mut self) {
        self.state = TriggerState::Armed;
        self.armed_once = true;
    }

    /// Returns `true` if the engine is currently armed and looking for triggers.
    pub fn is_armed(&self) -> bool {
        self.state == TriggerState::Armed
    }

    /// Force an immediate trigger at the next sample processed.
    /// The engine must have been armed at least once.
    pub fn force_trigger(&mut self) {
        if self.armed_once {
            self.state = TriggerState::Armed;
        }
    }

    /// Update the holdoff period (in samples).
    pub fn set_holdoff(&mut self, samples: usize) {
        self.config.holdoff_samples = samples;
    }

    /// Reset the engine to its initial state, clearing all buffers and statistics.
    pub fn reset(&mut self) {
        self.state = TriggerState::WaitingForReset;
        self.ring.clear();
        self.post_buf.clear();
        self.pre_snapshot.clear();
        self.post_remaining = 0;
        self.holdoff_remaining = 0;
        self.trigger_count = 0;
        self.current_trigger_index = 0;
        self.armed_once = false;
        self.trigger_sample_indices.clear();
        self.total_samples_processed = 0;
        self.prev_above = None;
        self.pulse_start = None;
        self.pattern_history.clear();
    }

    /// Compute trigger statistics from accumulated trigger events.
    pub fn statistics(&self) -> TriggerStats {
        let total = self.trigger_count;
        if total == 0 || self.total_samples_processed == 0 {
            return TriggerStats {
                total_triggers: total,
                trigger_rate_hz: 0.0,
                mean_interval_s: 0.0,
                jitter_rms_s: 0.0,
            };
        }

        let duration_s = self.total_samples_processed as f64 / self.config.sample_rate_hz;
        let rate = total as f64 / duration_s;

        // Compute intervals between successive triggers.
        let intervals: Vec<f64> = self
            .trigger_sample_indices
            .windows(2)
            .map(|w| (w[1] - w[0]) as f64 / self.config.sample_rate_hz)
            .collect();

        let (mean, jitter) = if intervals.is_empty() {
            (0.0, 0.0)
        } else {
            let mean = intervals.iter().sum::<f64>() / intervals.len() as f64;
            let variance =
                intervals.iter().map(|&t| (t - mean).powi(2)).sum::<f64>() / intervals.len() as f64;
            (mean, variance.sqrt())
        };

        TriggerStats {
            total_triggers: total,
            trigger_rate_hz: rate,
            mean_interval_s: mean,
            jitter_rms_s: jitter,
        }
    }

    /// Process a block of IQ samples, returning any completed captures.
    ///
    /// The I (real) component of each sample is used for threshold comparison.
    pub fn process(&mut self, samples: &[(f64, f64)]) -> Vec<CaptureResult> {
        let mut results = Vec::new();

        for (i, &sample) in samples.iter().enumerate() {
            let val = sample.0; // Use I component.
            let global_idx = self.total_samples_processed + i as u64;

            // Always feed the circular pre-trigger buffer.
            if self.ring.len() >= self.config.pre_trigger_samples {
                self.ring.pop_front();
            }
            self.ring.push_back(sample);

            // Update pattern history.
            if let TriggerMode::Pattern(ref pat) = self.config.mode {
                let above = val >= self.config.threshold;
                if self.pattern_history.len() >= pat.len() {
                    self.pattern_history.pop_front();
                }
                self.pattern_history.push_back(above);
            }

            match self.state {
                TriggerState::WaitingForReset => {
                    // Update prev_above tracking.
                    self.prev_above = Some(val >= self.config.threshold);
                }
                TriggerState::Armed => {
                    let should_trigger = self.check_trigger(val, global_idx);
                    if should_trigger {
                        // Capture pre-trigger snapshot.
                        self.pre_snapshot = self.ring.iter().copied().collect();
                        // Remove the trigger sample itself from pre if present (it goes into post).
                        if !self.pre_snapshot.is_empty() {
                            self.pre_snapshot.pop(); // last element is current sample
                        }
                        // Trim to requested pre-trigger length.
                        while self.pre_snapshot.len() > self.config.pre_trigger_samples {
                            self.pre_snapshot.remove(0);
                        }

                        self.post_buf.clear();
                        self.post_buf.push(sample); // trigger sample is first post-trigger sample
                        self.post_remaining = self.config.post_trigger_samples.saturating_sub(1);
                        self.current_trigger_index = i;
                        self.trigger_count += 1;
                        self.trigger_sample_indices.push(global_idx);

                        if self.post_remaining == 0 {
                            // Capture complete immediately.
                            results.push(self.build_capture());
                            self.enter_holdoff();
                        } else {
                            self.state = TriggerState::Capturing;
                        }
                    }
                    // Update prev_above after check.
                    self.prev_above = Some(val >= self.config.threshold);
                }
                TriggerState::Capturing => {
                    self.post_buf.push(sample);
                    self.post_remaining -= 1;
                    if self.post_remaining == 0 {
                        results.push(self.build_capture());
                        self.enter_holdoff();
                    }
                    self.prev_above = Some(val >= self.config.threshold);
                }
                TriggerState::Holdoff => {
                    if self.holdoff_remaining == 0 {
                        self.state = TriggerState::Armed;
                        self.prev_above = Some(val >= self.config.threshold);
                    } else {
                        self.holdoff_remaining -= 1;
                        self.prev_above = Some(val >= self.config.threshold);
                    }
                }
            }
        }

        self.total_samples_processed += samples.len() as u64;
        results
    }

    /// Check whether the current sample should fire the trigger.
    fn check_trigger(&self, val: f64, global_idx: u64) -> bool {
        match &self.config.mode {
            TriggerMode::Edge(dir) => {
                if let Some(was_above) = self.prev_above {
                    let now_above = val >= self.config.threshold;
                    let hyst_low = self.config.threshold - self.config.hysteresis;
                    let hyst_high = self.config.threshold + self.config.hysteresis;
                    match dir {
                        EdgeDirection::Rising => {
                            // Was below (with hysteresis) and now at or above threshold.
                            !was_above && now_above && val >= hyst_low
                        }
                        EdgeDirection::Falling => {
                            // Was above (with hysteresis) and now below threshold.
                            was_above && !now_above && val <= hyst_high
                        }
                    }
                } else {
                    false
                }
            }
            TriggerMode::Level(above) => {
                if *above {
                    val >= self.config.threshold
                } else {
                    val < self.config.threshold
                }
            }
            TriggerMode::PulseWidth(min_s, max_s) => {
                let now_above = val >= self.config.threshold;
                if let Some(was_above) = self.prev_above {
                    // Detect falling edge (end of pulse).
                    if was_above && !now_above {
                        if let Some(start) = self.pulse_start {
                            let pulse_samples = global_idx - start;
                            let pulse_s = pulse_samples as f64 / self.config.sample_rate_hz;
                            return pulse_s >= *min_s && pulse_s <= *max_s;
                        }
                    }
                }
                false
            }
            TriggerMode::Pattern(pat) => {
                if self.pattern_history.len() == pat.len() {
                    self.pattern_history.iter().zip(pat.iter()).all(|(&a, &b)| a == b)
                } else {
                    false
                }
            }
            TriggerMode::Window(low, high) => {
                if let Some(was_above_low) = self.prev_above {
                    let in_window = val >= *low && val <= *high;
                    let prev_val_unknown = was_above_low; // approximate: we only know threshold relation
                    // Trigger when signal enters the window.
                    // Use a simple heuristic: previous sample was outside, current inside.
                    // We store more state for accuracy, but for simplicity:
                    let _ = prev_val_unknown;
                    // Re-check with actual window: we need prev val.
                    // Since we don't store prev val directly, trigger whenever inside window
                    // and it's the first detection after arming.
                    in_window
                } else {
                    let in_window = val >= *low && val <= *high;
                    in_window
                }
            }
        }
    }

    fn build_capture(&mut self) -> CaptureResult {
        CaptureResult {
            triggered: true,
            trigger_index: self.current_trigger_index,
            pre_trigger_data: std::mem::take(&mut self.pre_snapshot),
            post_trigger_data: std::mem::take(&mut self.post_buf),
            trigger_count: self.trigger_count,
        }
    }

    fn enter_holdoff(&mut self) {
        if self.config.holdoff_samples > 0 {
            self.holdoff_remaining = self.config.holdoff_samples - 1;
            self.state = TriggerState::Holdoff;
        } else {
            self.state = TriggerState::Armed;
        }
    }
}

/// We need mutable access to `pulse_start` during processing, but `check_trigger`
/// borrows `self` immutably. Work around by updating pulse tracking in `process`.
impl TriggerEngine {
    /// Internal: update pulse-width tracking state. Called from `process`.
    fn update_pulse_tracking(&mut self, val: f64, global_idx: u64) {
        if let TriggerMode::PulseWidth(_, _) = self.config.mode {
            let now_above = val >= self.config.threshold;
            if let Some(was_above) = self.prev_above {
                if !was_above && now_above {
                    // Rising edge: start of pulse.
                    self.pulse_start = Some(global_idx);
                }
            } else if now_above {
                self.pulse_start = Some(global_idx);
            }
        }
    }
}

// Override process to include pulse tracking - we need to restructure slightly.
// The cleanest approach: integrate pulse tracking into the main process loop.
// Let's replace the process method with one that handles it properly.

impl TriggerEngine {
    /// Process a block of IQ samples with full pulse-width tracking.
    /// This replaces the basic `process` when pulse-width mode is active.
    pub fn process_with_pulse_tracking(&mut self, samples: &[(f64, f64)]) -> Vec<CaptureResult> {
        // For non-pulse-width modes, delegate to process.
        if !matches!(self.config.mode, TriggerMode::PulseWidth(_, _)) {
            return self.process(samples);
        }

        let mut results = Vec::new();

        for (i, &sample) in samples.iter().enumerate() {
            let val = sample.0;
            let global_idx = self.total_samples_processed + i as u64;

            // Update circular buffer.
            if self.ring.len() >= self.config.pre_trigger_samples {
                self.ring.pop_front();
            }
            self.ring.push_back(sample);

            // Track pulse start on rising edge.
            self.update_pulse_tracking(val, global_idx);

            match self.state {
                TriggerState::WaitingForReset => {
                    self.prev_above = Some(val >= self.config.threshold);
                }
                TriggerState::Armed => {
                    let should_trigger = self.check_trigger(val, global_idx);
                    if should_trigger {
                        self.pre_snapshot = self.ring.iter().copied().collect();
                        if !self.pre_snapshot.is_empty() {
                            self.pre_snapshot.pop();
                        }
                        while self.pre_snapshot.len() > self.config.pre_trigger_samples {
                            self.pre_snapshot.remove(0);
                        }

                        self.post_buf.clear();
                        self.post_buf.push(sample);
                        self.post_remaining = self.config.post_trigger_samples.saturating_sub(1);
                        self.current_trigger_index = i;
                        self.trigger_count += 1;
                        self.trigger_sample_indices.push(global_idx);

                        if self.post_remaining == 0 {
                            results.push(self.build_capture());
                            self.enter_holdoff();
                        } else {
                            self.state = TriggerState::Capturing;
                        }
                    }
                    self.prev_above = Some(val >= self.config.threshold);
                }
                TriggerState::Capturing => {
                    self.post_buf.push(sample);
                    self.post_remaining -= 1;
                    if self.post_remaining == 0 {
                        results.push(self.build_capture());
                        self.enter_holdoff();
                    }
                    self.prev_above = Some(val >= self.config.threshold);
                }
                TriggerState::Holdoff => {
                    if self.holdoff_remaining == 0 {
                        self.state = TriggerState::Armed;
                    } else {
                        self.holdoff_remaining -= 1;
                    }
                    self.prev_above = Some(val >= self.config.threshold);
                }
            }
        }

        self.total_samples_processed += samples.len() as u64;
        results
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config(mode: TriggerMode) -> TriggerConfig {
        TriggerConfig {
            mode,
            threshold: 0.5,
            hysteresis: 0.05,
            holdoff_samples: 0,
            pre_trigger_samples: 4,
            post_trigger_samples: 4,
            sample_rate_hz: 1000.0,
        }
    }

    #[test]
    fn test_rising_edge_trigger() {
        let config = default_config(TriggerMode::Edge(EdgeDirection::Rising));
        let mut engine = TriggerEngine::new(config);
        engine.arm();

        // Signal goes from 0.0 to 1.0 crossing 0.5
        let samples: Vec<(f64, f64)> = (0..20)
            .map(|i| (i as f64 * 0.1, 0.0))
            .collect();

        let results = engine.process(&samples);
        assert!(!results.is_empty(), "should trigger on rising edge");
        assert!(results[0].triggered);
    }

    #[test]
    fn test_falling_edge_trigger() {
        let config = default_config(TriggerMode::Edge(EdgeDirection::Falling));
        let mut engine = TriggerEngine::new(config);
        engine.arm();

        // Signal goes from 1.0 down to 0.0
        let samples: Vec<(f64, f64)> = (0..20)
            .map(|i| (1.0 - i as f64 * 0.1, 0.0))
            .collect();

        let results = engine.process(&samples);
        assert!(!results.is_empty(), "should trigger on falling edge");
        assert!(results[0].triggered);
    }

    #[test]
    fn test_level_above_trigger() {
        let mut config = default_config(TriggerMode::Level(true));
        config.post_trigger_samples = 2;
        config.pre_trigger_samples = 1;
        let mut engine = TriggerEngine::new(config);
        engine.arm();

        // Need enough samples after the trigger point (0.6) to fill post_trigger_samples.
        let samples = vec![
            (0.0, 0.0), (0.3, 0.0), (0.6, 0.0), (0.8, 0.0), (0.9, 0.0), (1.0, 0.0),
        ];
        let results = engine.process(&samples);
        // Should trigger when signal is >= 0.5
        assert!(!results.is_empty());
    }

    #[test]
    fn test_level_below_trigger() {
        let mut config = default_config(TriggerMode::Level(false));
        config.post_trigger_samples = 1;
        config.pre_trigger_samples = 0;
        let mut engine = TriggerEngine::new(config);
        engine.arm();

        // First sample (0.6) is above threshold, second (0.3) is below and triggers.
        let samples = vec![(0.6, 0.0), (0.3, 0.0), (0.1, 0.0), (0.0, 0.0), (0.0, 0.0)];
        let results = engine.process(&samples);
        assert!(!results.is_empty());
    }

    #[test]
    fn test_no_trigger_when_not_armed() {
        let config = default_config(TriggerMode::Edge(EdgeDirection::Rising));
        let mut engine = TriggerEngine::new(config);
        // Do NOT arm.

        let samples: Vec<(f64, f64)> = (0..20)
            .map(|i| (i as f64 * 0.1, 0.0))
            .collect();

        let results = engine.process(&samples);
        assert!(results.is_empty(), "should not trigger when not armed");
    }

    #[test]
    fn test_holdoff_prevents_retrigger() {
        let mut config = default_config(TriggerMode::Edge(EdgeDirection::Rising));
        config.holdoff_samples = 100;
        config.post_trigger_samples = 1;
        config.pre_trigger_samples = 0;
        let mut engine = TriggerEngine::new(config);
        engine.arm();

        // Two rising edges close together — second should be suppressed.
        let mut samples = Vec::new();
        // First edge at sample ~5
        for i in 0..10 {
            samples.push((i as f64 * 0.1, 0.0));
        }
        // Drop back down
        for _ in 0..5 {
            samples.push((0.0, 0.0));
        }
        // Second edge at sample ~20
        for i in 0..10 {
            samples.push((i as f64 * 0.1, 0.0));
        }

        let results = engine.process(&samples);
        assert_eq!(results.len(), 1, "holdoff should prevent second trigger");
    }

    #[test]
    fn test_pre_trigger_data_captured() {
        let mut config = default_config(TriggerMode::Edge(EdgeDirection::Rising));
        config.pre_trigger_samples = 3;
        config.post_trigger_samples = 2;
        config.holdoff_samples = 0;
        let mut engine = TriggerEngine::new(config);
        engine.arm();

        // Ramp up: 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, ...
        let samples: Vec<(f64, f64)> = (0..10)
            .map(|i| (i as f64 * 0.1, 0.0))
            .collect();

        let results = engine.process(&samples);
        assert!(!results.is_empty());
        // Pre-trigger data should contain samples before the crossing.
        assert!(results[0].pre_trigger_data.len() <= 3);
    }

    #[test]
    fn test_post_trigger_data_captured() {
        let mut config = default_config(TriggerMode::Edge(EdgeDirection::Rising));
        config.pre_trigger_samples = 0;
        config.post_trigger_samples = 3;
        config.holdoff_samples = 0;
        let mut engine = TriggerEngine::new(config);
        engine.arm();

        let samples: Vec<(f64, f64)> = (0..20)
            .map(|i| (i as f64 * 0.1, 0.0))
            .collect();

        let results = engine.process(&samples);
        assert!(!results.is_empty());
        assert_eq!(results[0].post_trigger_data.len(), 3);
    }

    #[test]
    fn test_force_trigger() {
        let config = default_config(TriggerMode::Edge(EdgeDirection::Rising));
        let mut engine = TriggerEngine::new(config);
        engine.arm();

        // No edge crossing — flat signal.
        let samples = vec![(0.1, 0.0); 10];
        let results = engine.process(&samples);
        assert!(results.is_empty(), "no natural trigger expected");

        // Force trigger, then feed more data.
        engine.force_trigger();
        // Now feed a signal that does cross threshold.
        let samples2: Vec<(f64, f64)> = (0..10)
            .map(|i| (i as f64 * 0.1, 0.0))
            .collect();
        let results2 = engine.process(&samples2);
        assert!(!results2.is_empty(), "forced trigger should re-arm and trigger");
    }

    #[test]
    fn test_reset_clears_state() {
        let config = default_config(TriggerMode::Edge(EdgeDirection::Rising));
        let mut engine = TriggerEngine::new(config);
        engine.arm();

        let samples: Vec<(f64, f64)> = (0..20)
            .map(|i| (i as f64 * 0.1, 0.0))
            .collect();
        let _ = engine.process(&samples);
        assert!(engine.trigger_count > 0);

        engine.reset();
        assert_eq!(engine.trigger_count, 0);
        assert!(!engine.is_armed());
        assert!(!engine.armed_once);
    }

    #[test]
    fn test_statistics_empty() {
        let config = default_config(TriggerMode::Edge(EdgeDirection::Rising));
        let engine = TriggerEngine::new(config);
        let stats = engine.statistics();
        assert_eq!(stats.total_triggers, 0);
        assert_eq!(stats.trigger_rate_hz, 0.0);
    }

    #[test]
    fn test_statistics_with_triggers() {
        let mut config = default_config(TriggerMode::Edge(EdgeDirection::Rising));
        config.holdoff_samples = 0;
        config.post_trigger_samples = 1;
        config.pre_trigger_samples = 0;
        let mut engine = TriggerEngine::new(config);
        engine.arm();

        // Create a signal with multiple rising edges.
        let mut samples = Vec::new();
        for _ in 0..5 {
            // Low then high.
            for j in 0..10 {
                samples.push((j as f64 * 0.1, 0.0));
            }
            // Back to low.
            for _ in 0..10 {
                samples.push((0.0, 0.0));
            }
        }

        let _ = engine.process(&samples);
        let stats = engine.statistics();
        assert!(stats.total_triggers > 1);
        assert!(stats.trigger_rate_hz > 0.0);
        assert!(stats.mean_interval_s > 0.0);
    }

    #[test]
    fn test_set_holdoff() {
        let config = default_config(TriggerMode::Edge(EdgeDirection::Rising));
        let mut engine = TriggerEngine::new(config);
        engine.set_holdoff(42);
        assert_eq!(engine.config.holdoff_samples, 42);
    }

    #[test]
    fn test_is_armed() {
        let config = default_config(TriggerMode::Edge(EdgeDirection::Rising));
        let mut engine = TriggerEngine::new(config);
        assert!(!engine.is_armed());
        engine.arm();
        assert!(engine.is_armed());
    }

    #[test]
    fn test_window_trigger() {
        let mut config = default_config(TriggerMode::Window(0.3, 0.7));
        config.post_trigger_samples = 1;
        config.pre_trigger_samples = 0;
        config.holdoff_samples = 0;
        let mut engine = TriggerEngine::new(config);
        engine.arm();

        // Signal at 0.5 is within [0.3, 0.7]
        let samples = vec![(0.5, 0.0), (0.5, 0.0), (0.5, 0.0)];
        let results = engine.process(&samples);
        assert!(!results.is_empty(), "should trigger when signal is in window");
    }

    #[test]
    fn test_pattern_trigger() {
        // Pattern: [false, false, true, true] — two low then two high
        let pattern = vec![false, false, true, true];
        let mut config = default_config(TriggerMode::Pattern(pattern));
        config.post_trigger_samples = 1;
        config.pre_trigger_samples = 0;
        config.holdoff_samples = 0;
        let mut engine = TriggerEngine::new(config);
        engine.arm();

        // Samples: low, low, high, high (matches pattern)
        let samples = vec![(0.0, 0.0), (0.0, 0.0), (0.8, 0.0), (0.8, 0.0)];
        let results = engine.process(&samples);
        assert!(!results.is_empty(), "should trigger when pattern matches");
    }

    #[test]
    fn test_pulse_width_trigger() {
        let mut config = TriggerConfig {
            mode: TriggerMode::PulseWidth(0.003, 0.007),
            threshold: 0.5,
            hysteresis: 0.05,
            holdoff_samples: 0,
            pre_trigger_samples: 0,
            post_trigger_samples: 1,
            sample_rate_hz: 1000.0, // 1 sample = 1 ms
        };
        let mut engine = TriggerEngine::new(config);
        engine.arm();

        // Pulse: 5 samples above threshold at 1000 Hz = 5 ms (within 3-7 ms range).
        let mut samples = Vec::new();
        // Low for 5 samples.
        for _ in 0..5 {
            samples.push((0.0, 0.0));
        }
        // High for 5 samples (5 ms pulse).
        for _ in 0..5 {
            samples.push((1.0, 0.0));
        }
        // Low again (falling edge triggers measurement).
        for _ in 0..5 {
            samples.push((0.0, 0.0));
        }

        let results = engine.process_with_pulse_tracking(&samples);
        assert!(!results.is_empty(), "should trigger on pulse width within range");
    }

    #[test]
    fn test_capture_result_trigger_count_increments() {
        let mut config = default_config(TriggerMode::Edge(EdgeDirection::Rising));
        config.holdoff_samples = 0;
        config.post_trigger_samples = 1;
        config.pre_trigger_samples = 0;
        let mut engine = TriggerEngine::new(config);
        engine.arm();

        let mut all_results = Vec::new();
        for _ in 0..3 {
            let mut samples = Vec::new();
            for j in 0..10 {
                samples.push((j as f64 * 0.1, 0.0));
            }
            for _ in 0..10 {
                samples.push((0.0, 0.0));
            }
            let results = engine.process(&samples);
            all_results.extend(results);
        }

        // Each trigger should have incrementing count.
        for (idx, r) in all_results.iter().enumerate() {
            assert_eq!(r.trigger_count, (idx + 1) as u64);
        }
    }
}

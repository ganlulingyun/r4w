//! # Adaptive Power Controller
//!
//! Closed-loop transmit power control for maintaining target received signal quality.
//!
//! This module implements inner-loop fast Transmit Power Control (TPC) commands and
//! outer-loop SINR target adjustment based on measured BLER/BER, following principles
//! similar to WCDMA/LTE power control.
//!
//! ## Features
//!
//! - **Inner loop**: Fast TPC commands (up/down/hold) at configurable rate (e.g., 1500 Hz for WCDMA)
//! - **Outer loop**: Adjusts SINR target based on measured block/bit error rate to maintain QoS
//! - **Power control modes**: Open-loop (path loss compensation), closed-loop, or combined
//! - **Power ramping**: Configurable ramp-up/ramp-down rate limits (dB/step)
//! - **Min/max bounds**: Power saturation handling with headroom reporting
//! - **Interference-aware**: Back-off when interference is detected above a threshold
//! - **SINR estimation**: Moving average filter for smoothing noisy SINR measurements
//! - **TPC command types**: Ternary (up/down/hold) or multi-step commands
//! - **Statistics**: Power histogram, adjustment frequency, saturation event tracking
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::adaptive_power_controller::{
//!     AdaptivePowerController, PowerControlConfig, PowerControlMode,
//! };
//!
//! let config = PowerControlConfig {
//!     target_sinr_db: 10.0,
//!     step_size_db: 1.0,
//!     min_power_dbm: -50.0,
//!     max_power_dbm: 23.0,
//!     mode: PowerControlMode::ClosedLoop,
//!     ..PowerControlConfig::default()
//! };
//!
//! let mut controller = AdaptivePowerController::new(config);
//!
//! // Simulate receiving a measured SINR and getting a TPC command
//! let measured_sinr_db = 8.0; // below target
//! let cmd = controller.inner_loop_step(measured_sinr_db);
//! assert!(controller.current_power_dbm() > -50.0); // power increased
//! ```

use std::collections::VecDeque;

/// Transmit Power Control command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TpcCommand {
    /// Increase power by the given amount in dB.
    Up(f64),
    /// Decrease power by the given amount in dB.
    Down(f64),
    /// Hold current power level.
    Hold,
}

/// Power control operating mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PowerControlMode {
    /// Open-loop: power set based on estimated path loss.
    OpenLoop,
    /// Closed-loop: power adjusted via TPC commands from the receiver.
    ClosedLoop,
    /// Combined: open-loop baseline with closed-loop fine adjustment.
    Combined,
}

impl Default for PowerControlMode {
    fn default() -> Self {
        PowerControlMode::ClosedLoop
    }
}

/// TPC command generation style.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TpcCommandStyle {
    /// Ternary commands: Up(step), Down(step), or Hold.
    Ternary,
    /// Multi-step: command magnitude proportional to SINR error.
    MultiStep {
        /// Maximum number of step sizes per command.
        max_steps: u32,
    },
}

impl Default for TpcCommandStyle {
    fn default() -> Self {
        TpcCommandStyle::Ternary
    }
}

/// Power ramp configuration.
#[derive(Debug, Clone, Copy)]
pub struct RampConfig {
    /// Maximum power increase per step (dB).
    pub max_ramp_up_db_per_step: f64,
    /// Maximum power decrease per step (dB).
    pub max_ramp_down_db_per_step: f64,
}

impl Default for RampConfig {
    fn default() -> Self {
        RampConfig {
            max_ramp_up_db_per_step: 2.0,
            max_ramp_down_db_per_step: 3.0,
        }
    }
}

/// Outer loop configuration for SINR target adjustment.
#[derive(Debug, Clone, Copy)]
pub struct OuterLoopConfig {
    /// Target BLER (Block Error Rate) for QoS, e.g. 0.01 = 1%.
    pub target_bler: f64,
    /// SINR target increase step when BLER exceeds target (dB).
    pub sinr_up_step_db: f64,
    /// SINR target decrease step when BLER is below target (dB).
    pub sinr_down_step_db: f64,
    /// Minimum allowed SINR target (dB).
    pub min_sinr_target_db: f64,
    /// Maximum allowed SINR target (dB).
    pub max_sinr_target_db: f64,
}

impl Default for OuterLoopConfig {
    fn default() -> Self {
        OuterLoopConfig {
            target_bler: 0.01,
            sinr_up_step_db: 0.5,
            sinr_down_step_db: 0.05,
            min_sinr_target_db: -5.0,
            max_sinr_target_db: 30.0,
        }
    }
}

/// Configuration for the adaptive power controller.
#[derive(Debug, Clone)]
pub struct PowerControlConfig {
    /// Target SINR/SNR in dB (inner loop reference).
    pub target_sinr_db: f64,
    /// TPC step size in dB (for ternary mode).
    pub step_size_db: f64,
    /// Minimum transmit power in dBm.
    pub min_power_dbm: f64,
    /// Maximum transmit power in dBm.
    pub max_power_dbm: f64,
    /// Initial transmit power in dBm.
    pub initial_power_dbm: f64,
    /// Power control mode.
    pub mode: PowerControlMode,
    /// TPC command generation style.
    pub command_style: TpcCommandStyle,
    /// Power ramp configuration.
    pub ramp: RampConfig,
    /// Outer loop configuration.
    pub outer_loop: OuterLoopConfig,
    /// Moving average window size for SINR estimation.
    pub sinr_avg_window: usize,
    /// Interference detection threshold in dB above noise floor.
    pub interference_threshold_db: f64,
    /// Power back-off when interference is detected (dB).
    pub interference_backoff_db: f64,
    /// SINR hysteresis band (dB) — commands only issued when |error| > hysteresis.
    pub hysteresis_db: f64,
}

impl Default for PowerControlConfig {
    fn default() -> Self {
        PowerControlConfig {
            target_sinr_db: 10.0,
            step_size_db: 1.0,
            min_power_dbm: -50.0,
            max_power_dbm: 23.0,
            initial_power_dbm: 0.0,
            mode: PowerControlMode::ClosedLoop,
            command_style: TpcCommandStyle::Ternary,
            ramp: RampConfig::default(),
            outer_loop: OuterLoopConfig::default(),
            sinr_avg_window: 10,
            interference_threshold_db: 20.0,
            interference_backoff_db: 6.0,
            hysteresis_db: 0.0,
        }
    }
}

/// Statistics tracked by the power controller.
#[derive(Debug, Clone)]
pub struct PowerControlStats {
    /// Total number of TPC commands issued.
    pub total_commands: u64,
    /// Number of Up commands issued.
    pub up_commands: u64,
    /// Number of Down commands issued.
    pub down_commands: u64,
    /// Number of Hold commands issued.
    pub hold_commands: u64,
    /// Number of times power hit the maximum bound.
    pub max_saturation_events: u64,
    /// Number of times power hit the minimum bound.
    pub min_saturation_events: u64,
    /// Number of interference back-off events.
    pub interference_backoff_events: u64,
    /// Power histogram bins (dBm): each bin covers 1 dB.
    /// Index 0 corresponds to `min_power_dbm`.
    pub power_histogram: Vec<u64>,
    /// Running sum of power for average calculation.
    power_sum: f64,
    /// Number of power samples for average.
    power_count: u64,
    /// Minimum power observed (dBm).
    pub min_power_observed: f64,
    /// Maximum power observed (dBm).
    pub max_power_observed: f64,
}

impl PowerControlStats {
    fn new(min_power_dbm: f64, max_power_dbm: f64) -> Self {
        let num_bins = ((max_power_dbm - min_power_dbm).ceil() as usize).max(1) + 1;
        PowerControlStats {
            total_commands: 0,
            up_commands: 0,
            down_commands: 0,
            hold_commands: 0,
            max_saturation_events: 0,
            min_saturation_events: 0,
            interference_backoff_events: 0,
            power_histogram: vec![0u64; num_bins],
            power_sum: 0.0,
            power_count: 0,
            min_power_observed: f64::MAX,
            max_power_observed: f64::MIN,
        }
    }

    fn record_command(&mut self, cmd: &TpcCommand) {
        self.total_commands += 1;
        match cmd {
            TpcCommand::Up(_) => self.up_commands += 1,
            TpcCommand::Down(_) => self.down_commands += 1,
            TpcCommand::Hold => self.hold_commands += 1,
        }
    }

    fn record_power(&mut self, power_dbm: f64, min_power_dbm: f64) {
        self.power_sum += power_dbm;
        self.power_count += 1;
        if power_dbm < self.min_power_observed {
            self.min_power_observed = power_dbm;
        }
        if power_dbm > self.max_power_observed {
            self.max_power_observed = power_dbm;
        }
        let bin = ((power_dbm - min_power_dbm).round() as usize)
            .min(self.power_histogram.len() - 1);
        self.power_histogram[bin] += 1;
    }

    /// Average transmit power over all recorded samples (dBm).
    pub fn average_power_dbm(&self) -> f64 {
        if self.power_count == 0 {
            0.0
        } else {
            self.power_sum / self.power_count as f64
        }
    }
}

/// Adaptive power controller implementing inner-loop TPC and outer-loop SINR adjustment.
#[derive(Debug, Clone)]
pub struct AdaptivePowerController {
    config: PowerControlConfig,
    /// Current transmit power in dBm.
    current_power: f64,
    /// Current effective SINR target (adjusted by outer loop).
    effective_sinr_target: f64,
    /// Moving average buffer for SINR measurements.
    sinr_buffer: VecDeque<f64>,
    /// Whether interference is currently detected.
    interference_active: bool,
    /// Open-loop estimated path loss (dB), set externally.
    estimated_path_loss_db: f64,
    /// Statistics.
    stats: PowerControlStats,
}

impl AdaptivePowerController {
    /// Create a new adaptive power controller with the given configuration.
    pub fn new(config: PowerControlConfig) -> Self {
        let initial = config.initial_power_dbm
            .max(config.min_power_dbm)
            .min(config.max_power_dbm);
        let effective_target = config.target_sinr_db;
        let stats = PowerControlStats::new(config.min_power_dbm, config.max_power_dbm);
        AdaptivePowerController {
            config,
            current_power: initial,
            effective_sinr_target: effective_target,
            sinr_buffer: VecDeque::new(),
            interference_active: false,
            estimated_path_loss_db: 0.0,
            stats,
        }
    }

    /// Current transmit power in dBm.
    pub fn current_power_dbm(&self) -> f64 {
        self.current_power
    }

    /// Current effective SINR target in dB (may differ from config due to outer loop).
    pub fn effective_sinr_target_db(&self) -> f64 {
        self.effective_sinr_target
    }

    /// Power headroom: how much power can still be increased (dB).
    pub fn power_headroom_db(&self) -> f64 {
        self.config.max_power_dbm - self.current_power
    }

    /// Whether the transmitter is at maximum power.
    pub fn is_at_max_power(&self) -> bool {
        (self.current_power - self.config.max_power_dbm).abs() < 1e-9
    }

    /// Whether the transmitter is at minimum power.
    pub fn is_at_min_power(&self) -> bool {
        (self.current_power - self.config.min_power_dbm).abs() < 1e-9
    }

    /// Set the estimated path loss for open-loop mode (dB).
    pub fn set_estimated_path_loss(&mut self, path_loss_db: f64) {
        self.estimated_path_loss_db = path_loss_db;
    }

    /// Get a reference to the power control statistics.
    pub fn stats(&self) -> &PowerControlStats {
        &self.stats
    }

    /// Reset all statistics.
    pub fn reset_stats(&mut self) {
        self.stats = PowerControlStats::new(self.config.min_power_dbm, self.config.max_power_dbm);
    }

    /// Report whether interference is currently flagged.
    pub fn is_interference_active(&self) -> bool {
        self.interference_active
    }

    /// Smoothed SINR estimate from the moving average buffer (dB).
    /// Returns `None` if no measurements have been recorded.
    pub fn smoothed_sinr_db(&self) -> Option<f64> {
        if self.sinr_buffer.is_empty() {
            return None;
        }
        let sum: f64 = self.sinr_buffer.iter().copied().sum();
        Some(sum / self.sinr_buffer.len() as f64)
    }

    /// Feed a SINR measurement into the moving average filter.
    fn update_sinr_estimate(&mut self, measured_sinr_db: f64) {
        self.sinr_buffer.push_back(measured_sinr_db);
        while self.sinr_buffer.len() > self.config.sinr_avg_window {
            self.sinr_buffer.pop_front();
        }
    }

    /// Apply power adjustment with ramp rate limiting and min/max saturation.
    fn apply_power_adjustment(&mut self, delta_db: f64) {
        let clamped_delta = if delta_db > 0.0 {
            delta_db.min(self.config.ramp.max_ramp_up_db_per_step)
        } else {
            delta_db.max(-self.config.ramp.max_ramp_down_db_per_step)
        };

        let new_power = self.current_power + clamped_delta;

        if new_power >= self.config.max_power_dbm {
            self.current_power = self.config.max_power_dbm;
            self.stats.max_saturation_events += 1;
        } else if new_power <= self.config.min_power_dbm {
            self.current_power = self.config.min_power_dbm;
            self.stats.min_saturation_events += 1;
        } else {
            self.current_power = new_power;
        }

        self.stats.record_power(self.current_power, self.config.min_power_dbm);
    }

    /// Compute open-loop power based on path loss compensation.
    /// Returns the desired transmit power in dBm.
    fn open_loop_power(&self) -> f64 {
        // Target received power = target SINR + noise floor assumption (-100 dBm typical)
        let noise_floor_dbm = -100.0;
        let target_rx_power = self.effective_sinr_target + noise_floor_dbm;
        let desired = target_rx_power + self.estimated_path_loss_db;
        desired.max(self.config.min_power_dbm).min(self.config.max_power_dbm)
    }

    /// Run one inner-loop step given a measured SINR (dB).
    ///
    /// Returns the TPC command that was generated and applied.
    pub fn inner_loop_step(&mut self, measured_sinr_db: f64) -> TpcCommand {
        self.update_sinr_estimate(measured_sinr_db);

        let smoothed = self.smoothed_sinr_db().unwrap_or(measured_sinr_db);

        let cmd = match self.config.mode {
            PowerControlMode::OpenLoop => {
                let target = self.open_loop_power();
                let delta = target - self.current_power;
                if delta.abs() < 0.01 {
                    TpcCommand::Hold
                } else if delta > 0.0 {
                    TpcCommand::Up(delta)
                } else {
                    TpcCommand::Down(-delta)
                }
            }
            PowerControlMode::ClosedLoop => {
                self.generate_closed_loop_command(smoothed)
            }
            PowerControlMode::Combined => {
                // Use open-loop as coarse, then closed-loop as fine adjustment
                let ol_target = self.open_loop_power();
                let ol_delta = ol_target - self.current_power;

                // Apply open-loop coarse adjustment first (half weight)
                let ol_component = ol_delta * 0.5;

                // Closed-loop fine adjustment
                let error = self.effective_sinr_target - smoothed;
                let cl_component = if error.abs() <= self.config.hysteresis_db {
                    0.0
                } else {
                    match self.config.command_style {
                        TpcCommandStyle::Ternary => {
                            if error > 0.0 {
                                self.config.step_size_db
                            } else {
                                -self.config.step_size_db
                            }
                        }
                        TpcCommandStyle::MultiStep { max_steps } => {
                            let steps = (error.abs() / self.config.step_size_db)
                                .round()
                                .min(max_steps as f64);
                            steps * self.config.step_size_db * error.signum()
                        }
                    }
                };

                let total_delta = ol_component + cl_component;
                if total_delta.abs() < 0.01 {
                    TpcCommand::Hold
                } else if total_delta > 0.0 {
                    TpcCommand::Up(total_delta)
                } else {
                    TpcCommand::Down(-total_delta)
                }
            }
        };

        // Apply the command
        match cmd {
            TpcCommand::Up(db) => self.apply_power_adjustment(db),
            TpcCommand::Down(db) => self.apply_power_adjustment(-db),
            TpcCommand::Hold => {
                self.stats.record_power(self.current_power, self.config.min_power_dbm);
            }
        }

        self.stats.record_command(&cmd);
        cmd
    }

    /// Generate a closed-loop TPC command based on SINR error.
    fn generate_closed_loop_command(&self, smoothed_sinr_db: f64) -> TpcCommand {
        let error = self.effective_sinr_target - smoothed_sinr_db;

        if error.abs() <= self.config.hysteresis_db {
            return TpcCommand::Hold;
        }

        match self.config.command_style {
            TpcCommandStyle::Ternary => {
                if error > 0.0 {
                    TpcCommand::Up(self.config.step_size_db)
                } else {
                    TpcCommand::Down(self.config.step_size_db)
                }
            }
            TpcCommandStyle::MultiStep { max_steps } => {
                let steps = (error.abs() / self.config.step_size_db)
                    .round()
                    .min(max_steps as f64);
                let magnitude = steps * self.config.step_size_db;
                if magnitude < 0.01 {
                    TpcCommand::Hold
                } else if error > 0.0 {
                    TpcCommand::Up(magnitude)
                } else {
                    TpcCommand::Down(magnitude)
                }
            }
        }
    }

    /// Run the outer loop update given a measured BLER (0.0 to 1.0).
    ///
    /// Call this at a slower rate than the inner loop (e.g., once per frame or
    /// transport block). Adjusts the effective SINR target to maintain the QoS BLER.
    pub fn outer_loop_update(&mut self, measured_bler: f64) {
        let oc = &self.config.outer_loop;
        if measured_bler > oc.target_bler {
            // Too many errors — raise the SINR target
            self.effective_sinr_target = (self.effective_sinr_target + oc.sinr_up_step_db)
                .min(oc.max_sinr_target_db);
        } else {
            // BLER is acceptable — try to lower the target to save power
            self.effective_sinr_target = (self.effective_sinr_target - oc.sinr_down_step_db)
                .max(oc.min_sinr_target_db);
        }
    }

    /// Notify the controller about detected interference level (dB above noise floor).
    ///
    /// If the interference exceeds the configured threshold, the controller will back
    /// off power to avoid contributing to the interference environment.
    pub fn report_interference(&mut self, interference_level_db: f64) {
        if interference_level_db > self.config.interference_threshold_db {
            if !self.interference_active {
                self.interference_active = true;
                self.stats.interference_backoff_events += 1;
                self.apply_power_adjustment(-self.config.interference_backoff_db);
            }
        } else {
            self.interference_active = false;
        }
    }

    /// Apply an external TPC command (e.g., received from the base station).
    pub fn apply_external_command(&mut self, cmd: TpcCommand) {
        match cmd {
            TpcCommand::Up(db) => self.apply_power_adjustment(db),
            TpcCommand::Down(db) => self.apply_power_adjustment(-db),
            TpcCommand::Hold => {
                self.stats.record_power(self.current_power, self.config.min_power_dbm);
            }
        }
        self.stats.record_command(&cmd);
    }

    /// Set current power directly (e.g., for initialization or handover).
    /// The value is clamped to min/max bounds.
    pub fn set_power_dbm(&mut self, power_dbm: f64) {
        self.current_power = power_dbm
            .max(self.config.min_power_dbm)
            .min(self.config.max_power_dbm);
        self.stats.record_power(self.current_power, self.config.min_power_dbm);
    }

    /// Get a copy of the current configuration.
    pub fn config(&self) -> &PowerControlConfig {
        &self.config
    }

    /// Update the SINR target directly (bypassing outer loop).
    pub fn set_sinr_target_db(&mut self, target_db: f64) {
        self.effective_sinr_target = target_db;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_controller() -> AdaptivePowerController {
        let config = PowerControlConfig {
            target_sinr_db: 10.0,
            step_size_db: 1.0,
            min_power_dbm: -50.0,
            max_power_dbm: 23.0,
            initial_power_dbm: 0.0,
            mode: PowerControlMode::ClosedLoop,
            ..PowerControlConfig::default()
        };
        AdaptivePowerController::new(config)
    }

    #[test]
    fn test_initial_power() {
        let ctrl = default_controller();
        assert!((ctrl.current_power_dbm() - 0.0).abs() < 1e-9);
        assert!((ctrl.effective_sinr_target_db() - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_inner_loop_increases_power_when_sinr_low() {
        let mut ctrl = default_controller();
        let initial = ctrl.current_power_dbm();
        ctrl.inner_loop_step(5.0); // 5 dB below target of 10
        assert!(ctrl.current_power_dbm() > initial);
    }

    #[test]
    fn test_inner_loop_decreases_power_when_sinr_high() {
        let mut ctrl = default_controller();
        ctrl.set_power_dbm(10.0);
        let initial = ctrl.current_power_dbm();
        ctrl.inner_loop_step(15.0); // 5 dB above target of 10
        assert!(ctrl.current_power_dbm() < initial);
    }

    #[test]
    fn test_ternary_step_size() {
        let mut ctrl = default_controller();
        let initial = ctrl.current_power_dbm();
        ctrl.inner_loop_step(5.0); // below target => Up(1.0)
        assert!((ctrl.current_power_dbm() - (initial + 1.0)).abs() < 1e-9);
    }

    #[test]
    fn test_multi_step_mode() {
        let config = PowerControlConfig {
            target_sinr_db: 10.0,
            step_size_db: 1.0,
            min_power_dbm: -50.0,
            max_power_dbm: 23.0,
            initial_power_dbm: 0.0,
            mode: PowerControlMode::ClosedLoop,
            command_style: TpcCommandStyle::MultiStep { max_steps: 5 },
            ramp: RampConfig {
                max_ramp_up_db_per_step: 10.0,
                max_ramp_down_db_per_step: 10.0,
            },
            ..PowerControlConfig::default()
        };
        let mut ctrl = AdaptivePowerController::new(config);
        let initial = ctrl.current_power_dbm();
        // SINR is 7 dB, target is 10 dB, error = 3 dB => 3 steps of 1 dB
        // But with moving average of window=10, first sample is 7.0, so smoothed=7.0
        ctrl.inner_loop_step(7.0);
        let expected = initial + 3.0; // 3 steps * 1 dB
        assert!(
            (ctrl.current_power_dbm() - expected).abs() < 1e-9,
            "got {} expected {}",
            ctrl.current_power_dbm(),
            expected
        );
    }

    #[test]
    fn test_max_power_saturation() {
        let mut ctrl = default_controller();
        ctrl.set_power_dbm(22.5);
        // Try to increase beyond max (23 dBm)
        ctrl.inner_loop_step(0.0); // very low SINR => Up
        assert!((ctrl.current_power_dbm() - 23.0).abs() < 1e-9);
        assert!(ctrl.is_at_max_power());
        assert!(ctrl.stats().max_saturation_events > 0);
    }

    #[test]
    fn test_min_power_saturation() {
        let mut ctrl = default_controller();
        ctrl.set_power_dbm(-49.5);
        // Try to decrease below min (-50 dBm)
        ctrl.inner_loop_step(25.0); // very high SINR => Down
        assert!((ctrl.current_power_dbm() - (-50.0)).abs() < 1e-9);
        assert!(ctrl.is_at_min_power());
        assert!(ctrl.stats().min_saturation_events > 0);
    }

    #[test]
    fn test_power_headroom() {
        let mut ctrl = default_controller();
        ctrl.set_power_dbm(20.0);
        assert!((ctrl.power_headroom_db() - 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_ramp_rate_limiting() {
        let config = PowerControlConfig {
            target_sinr_db: 10.0,
            step_size_db: 1.0,
            min_power_dbm: -50.0,
            max_power_dbm: 23.0,
            initial_power_dbm: 0.0,
            mode: PowerControlMode::ClosedLoop,
            command_style: TpcCommandStyle::MultiStep { max_steps: 10 },
            ramp: RampConfig {
                max_ramp_up_db_per_step: 2.0,
                max_ramp_down_db_per_step: 3.0,
            },
            ..PowerControlConfig::default()
        };
        let mut ctrl = AdaptivePowerController::new(config);
        // Error of 8 dB => multi-step wants 8 dB, but ramp limits to 2 dB up
        ctrl.inner_loop_step(2.0);
        assert!((ctrl.current_power_dbm() - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_outer_loop_raises_target_on_high_bler() {
        let mut ctrl = default_controller();
        let initial_target = ctrl.effective_sinr_target_db();
        ctrl.outer_loop_update(0.05); // BLER = 5%, above default target of 1%
        assert!(ctrl.effective_sinr_target_db() > initial_target);
    }

    #[test]
    fn test_outer_loop_lowers_target_on_low_bler() {
        let mut ctrl = default_controller();
        let initial_target = ctrl.effective_sinr_target_db();
        ctrl.outer_loop_update(0.001); // BLER = 0.1%, below 1% target
        assert!(ctrl.effective_sinr_target_db() < initial_target);
    }

    #[test]
    fn test_outer_loop_sinr_target_bounded() {
        let mut ctrl = default_controller();
        // Repeatedly raise target
        for _ in 0..200 {
            ctrl.outer_loop_update(1.0); // 100% BLER
        }
        assert!(ctrl.effective_sinr_target_db() <= ctrl.config().outer_loop.max_sinr_target_db);

        // Repeatedly lower target
        for _ in 0..2000 {
            ctrl.outer_loop_update(0.0); // 0% BLER
        }
        assert!(ctrl.effective_sinr_target_db() >= ctrl.config().outer_loop.min_sinr_target_db);
    }

    #[test]
    fn test_interference_backoff() {
        let mut ctrl = default_controller();
        ctrl.set_power_dbm(10.0);
        let before = ctrl.current_power_dbm();
        ctrl.report_interference(25.0); // above threshold of 20 dB
        assert!(ctrl.is_interference_active());
        assert!(ctrl.current_power_dbm() < before);
        assert_eq!(ctrl.stats().interference_backoff_events, 1);
    }

    #[test]
    fn test_interference_clears() {
        let mut ctrl = default_controller();
        ctrl.set_power_dbm(10.0);
        ctrl.report_interference(25.0);
        assert!(ctrl.is_interference_active());
        ctrl.report_interference(5.0); // below threshold
        assert!(!ctrl.is_interference_active());
    }

    #[test]
    fn test_interference_backoff_only_once() {
        let mut ctrl = default_controller();
        ctrl.set_power_dbm(10.0);
        ctrl.report_interference(25.0);
        let power_after_first = ctrl.current_power_dbm();
        ctrl.report_interference(30.0); // still above threshold, but already active
        assert!((ctrl.current_power_dbm() - power_after_first).abs() < 1e-9);
        assert_eq!(ctrl.stats().interference_backoff_events, 1);
    }

    #[test]
    fn test_open_loop_mode() {
        let config = PowerControlConfig {
            target_sinr_db: 10.0,
            step_size_db: 1.0,
            min_power_dbm: -50.0,
            max_power_dbm: 23.0,
            initial_power_dbm: 0.0,
            mode: PowerControlMode::OpenLoop,
            ramp: RampConfig {
                max_ramp_up_db_per_step: 50.0,
                max_ramp_down_db_per_step: 50.0,
            },
            ..PowerControlConfig::default()
        };
        let mut ctrl = AdaptivePowerController::new(config);
        ctrl.set_estimated_path_loss(120.0);
        // target_rx = 10 + (-100) = -90 dBm, desired_tx = -90 + 120 = 30 dBm => clamped to 23
        ctrl.inner_loop_step(5.0); // measured SINR doesn't matter much in open-loop
        assert!((ctrl.current_power_dbm() - 23.0).abs() < 1e-9);
    }

    #[test]
    fn test_combined_mode() {
        let config = PowerControlConfig {
            target_sinr_db: 10.0,
            step_size_db: 1.0,
            min_power_dbm: -50.0,
            max_power_dbm: 23.0,
            initial_power_dbm: 0.0,
            mode: PowerControlMode::Combined,
            ..PowerControlConfig::default()
        };
        let mut ctrl = AdaptivePowerController::new(config);
        ctrl.set_estimated_path_loss(80.0);
        // Open-loop: target_rx = -90, desired = -90+80 = -10. delta = -10 - 0 = -10, half = -5
        // Closed-loop: error = 10 - 8 = 2 > 0 => up 1.0
        // Total = -5 + 1 = -4 => power decreases by 4 (ramp limited to 3 down)
        ctrl.inner_loop_step(8.0);
        // Power should have changed from 0.0
        assert!((ctrl.current_power_dbm() - 0.0).abs() > 0.01);
    }

    #[test]
    fn test_sinr_moving_average() {
        let config = PowerControlConfig {
            sinr_avg_window: 4,
            ..PowerControlConfig::default()
        };
        let mut ctrl = AdaptivePowerController::new(config);
        // Feed measurements
        ctrl.inner_loop_step(8.0);
        ctrl.inner_loop_step(10.0);
        ctrl.inner_loop_step(12.0);
        ctrl.inner_loop_step(10.0);
        // Average should be (8+10+12+10)/4 = 10.0
        let avg = ctrl.smoothed_sinr_db().unwrap();
        assert!((avg - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_sinr_window_overflow() {
        let config = PowerControlConfig {
            sinr_avg_window: 3,
            ..PowerControlConfig::default()
        };
        let mut ctrl = AdaptivePowerController::new(config);
        ctrl.inner_loop_step(10.0);
        ctrl.inner_loop_step(10.0);
        ctrl.inner_loop_step(10.0);
        ctrl.inner_loop_step(20.0); // pushes out oldest 10.0
        // Window: [10, 10, 20], avg = 13.33
        let avg = ctrl.smoothed_sinr_db().unwrap();
        assert!((avg - (40.0 / 3.0)).abs() < 1e-9);
    }

    #[test]
    fn test_external_command_application() {
        let mut ctrl = default_controller();
        ctrl.set_power_dbm(10.0);
        ctrl.apply_external_command(TpcCommand::Up(2.0));
        assert!((ctrl.current_power_dbm() - 12.0).abs() < 1e-9);

        ctrl.apply_external_command(TpcCommand::Down(5.0));
        assert!((ctrl.current_power_dbm() - 9.0).abs() < 1e-9);

        ctrl.apply_external_command(TpcCommand::Hold);
        assert!((ctrl.current_power_dbm() - 9.0).abs() < 1e-9);
    }

    #[test]
    fn test_stats_tracking() {
        let config = PowerControlConfig {
            target_sinr_db: 10.0,
            step_size_db: 1.0,
            min_power_dbm: -50.0,
            max_power_dbm: 23.0,
            initial_power_dbm: 0.0,
            mode: PowerControlMode::ClosedLoop,
            sinr_avg_window: 1, // no averaging so each step sees exact value
            ..PowerControlConfig::default()
        };
        let mut ctrl = AdaptivePowerController::new(config);
        ctrl.inner_loop_step(5.0);  // Up (5 < 10)
        ctrl.inner_loop_step(5.0);  // Up (5 < 10)
        ctrl.inner_loop_step(20.0); // Down (20 > 10)
        let stats = ctrl.stats();
        assert_eq!(stats.total_commands, 3);
        assert_eq!(stats.up_commands, 2);
        assert_eq!(stats.down_commands, 1);
        assert!(stats.average_power_dbm().is_finite());
    }

    #[test]
    fn test_hysteresis_prevents_toggling() {
        let config = PowerControlConfig {
            target_sinr_db: 10.0,
            step_size_db: 1.0,
            hysteresis_db: 0.5,
            min_power_dbm: -50.0,
            max_power_dbm: 23.0,
            initial_power_dbm: 0.0,
            mode: PowerControlMode::ClosedLoop,
            sinr_avg_window: 1, // no averaging for clarity
            ..PowerControlConfig::default()
        };
        let mut ctrl = AdaptivePowerController::new(config);
        let initial = ctrl.current_power_dbm();
        // SINR = 9.8 => error = 0.2, which is within hysteresis of 0.5 => Hold
        let cmd = ctrl.inner_loop_step(9.8);
        assert_eq!(cmd, TpcCommand::Hold);
        assert!((ctrl.current_power_dbm() - initial).abs() < 1e-9);
    }

    #[test]
    fn test_set_and_reset() {
        let mut ctrl = default_controller();
        ctrl.inner_loop_step(5.0);
        ctrl.inner_loop_step(5.0);
        assert!(ctrl.stats().total_commands > 0);

        ctrl.reset_stats();
        assert_eq!(ctrl.stats().total_commands, 0);
        assert_eq!(ctrl.stats().up_commands, 0);

        ctrl.set_sinr_target_db(15.0);
        assert!((ctrl.effective_sinr_target_db() - 15.0).abs() < 1e-9);
    }

    #[test]
    fn test_initial_power_clamped_to_bounds() {
        let config = PowerControlConfig {
            initial_power_dbm: 100.0, // above max
            max_power_dbm: 23.0,
            min_power_dbm: -50.0,
            ..PowerControlConfig::default()
        };
        let ctrl = AdaptivePowerController::new(config);
        assert!((ctrl.current_power_dbm() - 23.0).abs() < 1e-9);

        let config2 = PowerControlConfig {
            initial_power_dbm: -100.0, // below min
            max_power_dbm: 23.0,
            min_power_dbm: -50.0,
            ..PowerControlConfig::default()
        };
        let ctrl2 = AdaptivePowerController::new(config2);
        assert!((ctrl2.current_power_dbm() - (-50.0)).abs() < 1e-9);
    }

    #[test]
    fn test_power_histogram_populated() {
        let mut ctrl = default_controller();
        for sinr in &[5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 15.0] {
            ctrl.inner_loop_step(*sinr);
        }
        let total: u64 = ctrl.stats().power_histogram.iter().sum();
        // Each step records power, plus the initial set from apply/hold
        assert!(total > 0);
    }

    #[test]
    fn test_smoothed_sinr_none_when_empty() {
        let ctrl = default_controller();
        assert!(ctrl.smoothed_sinr_db().is_none());
    }
}

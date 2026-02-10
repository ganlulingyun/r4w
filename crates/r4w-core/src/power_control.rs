//! Transmit Power Control Algorithms
//!
//! Implements open-loop, closed-loop, and fractional power control algorithms
//! used in cellular and wireless communications systems. Power control is
//! essential for managing interference, conserving battery life, and meeting
//! regulatory constraints on transmit power.
//!
//! ## Algorithms
//!
//! - [`OpenLoopPowerControl`] — Determines TX power from estimated path loss
//!   and interference, without feedback from the receiver.
//! - [`ClosedLoopPowerControl`] — Adjusts TX power in discrete steps based on
//!   measured SINR feedback (TPC commands), as used in WCDMA/LTE.
//! - [`FractionalPowerControl`] — LTE-style fractional path loss compensation
//!   where only a fraction (alpha) of the path loss is compensated.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::power_control::{ClosedLoopPowerControl, dbm_to_watts};
//!
//! let mut pc = ClosedLoopPowerControl::new(10.0, 1.0, 23.0, -40.0);
//! // Measured SINR is below target — power increases
//! let power = pc.update(5.0, 10.0);
//! assert!(power > 10.0);
//! assert!(dbm_to_watts(power) > 0.0);
//! ```

/// Convert power in dBm to watts.
///
/// Formula: `W = 10^((dBm - 30) / 10)`
pub fn dbm_to_watts(dbm: f64) -> f64 {
    10.0_f64.powf((dbm - 30.0) / 10.0)
}

/// Convert power in watts to dBm.
///
/// Formula: `dBm = 10 * log10(W) + 30`
pub fn watts_to_dbm(watts: f64) -> f64 {
    10.0 * watts.log10() + 30.0
}

// ---------------------------------------------------------------------------
// Open-Loop Power Control
// ---------------------------------------------------------------------------

/// Open-loop transmit power control.
///
/// Computes the required TX power from the estimated downlink path loss and
/// the current interference level, without any feedback from the receiver.
/// The result is clamped to `[min_power_dbm, max_power_dbm]`.
///
/// **Formula:** `P_tx = clamp(target_sinr + interference + path_loss, min, max)`
#[derive(Debug, Clone)]
pub struct OpenLoopPowerControl {
    /// Estimated downlink path loss in dB (set via [`compute_power`]).
    pub path_loss_db: f64,
    /// Target SINR at the receiver (dB).
    pub target_sinr_db: f64,
    /// Maximum allowed transmit power (dBm).
    pub max_power_dbm: f64,
    /// Minimum allowed transmit power (dBm).
    pub min_power_dbm: f64,
}

impl OpenLoopPowerControl {
    /// Create a new open-loop power controller.
    ///
    /// `path_loss_db` is initialised to 0.0 and updated on each call to
    /// [`compute_power`].
    pub fn new(target_sinr_db: f64, max_power_dbm: f64, min_power_dbm: f64) -> Self {
        Self {
            path_loss_db: 0.0,
            target_sinr_db,
            max_power_dbm,
            min_power_dbm,
        }
    }

    /// Compute the required TX power given the current path loss and
    /// interference level.
    ///
    /// Returns the transmit power in dBm, clamped to the configured range.
    pub fn compute_power(&mut self, path_loss_db: f64, interference_dbm: f64) -> f64 {
        self.path_loss_db = path_loss_db;
        let power = self.target_sinr_db + interference_dbm + path_loss_db;
        power.clamp(self.min_power_dbm, self.max_power_dbm)
    }
}

// ---------------------------------------------------------------------------
// Closed-Loop Power Control
// ---------------------------------------------------------------------------

/// Statistics gathered by [`ClosedLoopPowerControl`].
#[derive(Debug, Clone, Copy)]
pub struct PowerControlStats {
    /// Average transmit power over all updates (dBm).
    pub avg_power_dbm: f64,
    /// Maximum transmit power observed (dBm).
    pub max_power_dbm: f64,
    /// Minimum transmit power observed (dBm).
    pub min_power_dbm: f64,
    /// Total number of TPC updates processed.
    pub num_updates: u64,
}

/// Closed-loop transmit power control.
///
/// At each slot the receiver measures the SINR and sends a Transmit Power
/// Control (TPC) command: if the measured SINR is below the target the
/// transmitter increases power by `step_db`; if above, it decreases by
/// `step_db`. The result is clamped to `[min_power_dbm, max_power_dbm]`.
///
/// This models the inner-loop power control used in WCDMA (1 dB steps at
/// 1500 Hz) and similar closed-loop schemes.
#[derive(Debug, Clone)]
pub struct ClosedLoopPowerControl {
    /// Current transmit power (dBm).
    current_power_dbm: f64,
    /// Power adjustment step size (dB, positive).
    step_db: f64,
    /// Maximum allowed transmit power (dBm).
    max_power_dbm: f64,
    /// Minimum allowed transmit power (dBm).
    min_power_dbm: f64,
    /// Target SINR (dB) — stored for reference; the target is also passed
    /// per-update so it can change over time.
    target_sinr_db: f64,
    // Running stats
    power_sum: f64,
    power_max: f64,
    power_min: f64,
    num_updates: u64,
}

impl ClosedLoopPowerControl {
    /// Create a new closed-loop power controller.
    ///
    /// * `initial_power_dbm` — starting TX power (dBm)
    /// * `step_db` — magnitude of each TPC adjustment (dB)
    /// * `max_power_dbm` — upper power limit (dBm)
    /// * `min_power_dbm` — lower power limit (dBm)
    pub fn new(
        initial_power_dbm: f64,
        step_db: f64,
        max_power_dbm: f64,
        min_power_dbm: f64,
    ) -> Self {
        Self {
            current_power_dbm: initial_power_dbm,
            step_db,
            max_power_dbm,
            min_power_dbm,
            target_sinr_db: 0.0,
            power_sum: 0.0,
            power_max: f64::NEG_INFINITY,
            power_min: f64::INFINITY,
            num_updates: 0,
        }
    }

    /// Process one TPC update.
    ///
    /// If `measured_sinr_db < target_sinr_db` the power is increased by
    /// `step_db`; otherwise it is decreased by `step_db`. The result is
    /// clamped to the configured range.
    ///
    /// Returns the new transmit power in dBm.
    pub fn update(&mut self, measured_sinr_db: f64, target_sinr_db: f64) -> f64 {
        self.target_sinr_db = target_sinr_db;
        if measured_sinr_db < target_sinr_db {
            self.current_power_dbm += self.step_db;
        } else {
            self.current_power_dbm -= self.step_db;
        }
        self.current_power_dbm = self.current_power_dbm.clamp(self.min_power_dbm, self.max_power_dbm);

        // Update stats
        self.num_updates += 1;
        self.power_sum += self.current_power_dbm;
        if self.current_power_dbm > self.power_max {
            self.power_max = self.current_power_dbm;
        }
        if self.current_power_dbm < self.power_min {
            self.power_min = self.current_power_dbm;
        }

        self.current_power_dbm
    }

    /// Get the current transmit power (dBm).
    pub fn power(&self) -> f64 {
        self.current_power_dbm
    }

    /// Return accumulated power control statistics.
    ///
    /// If no updates have been processed yet, `avg_power_dbm` is 0.0 and
    /// `max_power_dbm` / `min_power_dbm` reflect the initial state (NEG_INFINITY
    /// and INFINITY respectively).
    pub fn stats(&self) -> PowerControlStats {
        let avg = if self.num_updates > 0 {
            self.power_sum / self.num_updates as f64
        } else {
            0.0
        };
        PowerControlStats {
            avg_power_dbm: avg,
            max_power_dbm: self.power_max,
            min_power_dbm: self.power_min,
            num_updates: self.num_updates,
        }
    }
}

// ---------------------------------------------------------------------------
// Fractional Power Control (LTE uplink)
// ---------------------------------------------------------------------------

/// LTE-style fractional power control.
///
/// Only a fraction `alpha` (0.0..=1.0) of the estimated path loss is
/// compensated, which reduces inter-cell interference for cell-edge users
/// compared to full compensation.
///
/// **Formula:** `P_tx = min(P_max, P0 + alpha * PL)`
///
/// Where:
/// * `P0` — base power level (dBm), set by the eNodeB
/// * `alpha` — path loss compensation factor (0 = no compensation, 1 = full)
/// * `PL` — estimated downlink path loss (dB)
/// * `P_max` — maximum UE transmit power (dBm, typically 23 dBm)
#[derive(Debug, Clone)]
pub struct FractionalPowerControl {
    /// Base power level set by the network (dBm).
    p0: f64,
    /// Path loss compensation factor (0.0..=1.0).
    alpha: f64,
    /// Maximum allowed transmit power (dBm).
    max_power_dbm: f64,
}

impl FractionalPowerControl {
    /// Create a new fractional power controller.
    ///
    /// * `p0` — nominal power level (dBm)
    /// * `alpha` — path loss compensation factor (0.0 to 1.0)
    /// * `max_power_dbm` — upper power limit (dBm)
    pub fn new(p0: f64, alpha: f64, max_power_dbm: f64) -> Self {
        Self {
            p0,
            alpha: alpha.clamp(0.0, 1.0),
            max_power_dbm,
        }
    }

    /// Compute the transmit power given the estimated path loss.
    ///
    /// Returns `min(max_power_dbm, p0 + alpha * path_loss_db)` in dBm.
    pub fn compute_power(&self, path_loss_db: f64) -> f64 {
        let power = self.p0 + self.alpha * path_loss_db;
        power.min(self.max_power_dbm)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_open_loop_basic() {
        let mut pc = OpenLoopPowerControl::new(10.0, 30.0, -10.0);
        // target_sinr(10) + interference(-90) + path_loss(100) = 20 dBm
        let power = pc.compute_power(100.0, -90.0);
        assert!((power - 20.0).abs() < 1e-9);
        assert!((pc.path_loss_db - 100.0).abs() < 1e-9);
    }

    #[test]
    fn test_open_loop_clamped() {
        let mut pc = OpenLoopPowerControl::new(10.0, 23.0, -10.0);
        // target_sinr(10) + interference(-50) + path_loss(120) = 80 dBm, clamped to 23
        let power = pc.compute_power(120.0, -50.0);
        assert!((power - 23.0).abs() < 1e-9);

        // Very low required power, clamped to min
        let power = pc.compute_power(10.0, -100.0);
        // 10 + (-100) + 10 = -80, clamped to -10
        assert!((power - (-10.0)).abs() < 1e-9);
    }

    #[test]
    fn test_closed_loop_increase() {
        let mut pc = ClosedLoopPowerControl::new(10.0, 1.0, 23.0, -40.0);
        // measured SINR (5) < target (10) → power should increase by 1 dB
        let power = pc.update(5.0, 10.0);
        assert!((power - 11.0).abs() < 1e-9);
    }

    #[test]
    fn test_closed_loop_decrease() {
        let mut pc = ClosedLoopPowerControl::new(10.0, 1.0, 23.0, -40.0);
        // measured SINR (15) >= target (10) → power should decrease by 1 dB
        let power = pc.update(15.0, 10.0);
        assert!((power - 9.0).abs() < 1e-9);
    }

    #[test]
    fn test_closed_loop_limits() {
        let mut pc = ClosedLoopPowerControl::new(22.0, 2.0, 23.0, -40.0);
        // Increase: 22 + 2 = 24, clamped to 23
        let power = pc.update(0.0, 10.0);
        assert!((power - 23.0).abs() < 1e-9);

        // Keep pushing — should stay at 23
        let power = pc.update(0.0, 10.0);
        assert!((power - 23.0).abs() < 1e-9);

        // Now test lower limit
        let mut pc2 = ClosedLoopPowerControl::new(-39.0, 2.0, 23.0, -40.0);
        // Decrease: -39 - 2 = -41, clamped to -40
        let power = pc2.update(20.0, 10.0);
        assert!((power - (-40.0)).abs() < 1e-9);
    }

    #[test]
    fn test_fractional_pc() {
        let pc = FractionalPowerControl::new(-70.0, 0.8, 23.0);
        // P = -70 + 0.8 * 130 = -70 + 104 = 34 → clamped to 23
        let power = pc.compute_power(130.0);
        assert!((power - 23.0).abs() < 1e-9);

        // P = -70 + 0.8 * 80 = -70 + 64 = -6 → below max, unclamped
        let power = pc.compute_power(80.0);
        assert!((power - (-6.0)).abs() < 1e-9);
    }

    #[test]
    fn test_dbm_watts_conversion() {
        // 0 dBm = 1 mW = 0.001 W
        assert!((dbm_to_watts(0.0) - 0.001).abs() < 1e-9);
        // 30 dBm = 1 W
        assert!((dbm_to_watts(30.0) - 1.0).abs() < 1e-9);
        // 20 dBm = 0.1 W
        assert!((dbm_to_watts(20.0) - 0.1).abs() < 1e-6);

        // Round-trip
        let dbm_val = 17.5;
        let watts = dbm_to_watts(dbm_val);
        let dbm_back = watts_to_dbm(watts);
        assert!((dbm_back - dbm_val).abs() < 1e-9);
    }

    #[test]
    fn test_stats() {
        let mut pc = ClosedLoopPowerControl::new(10.0, 1.0, 23.0, -40.0);
        // 3 increases: 11, 12, 13
        pc.update(0.0, 10.0);
        pc.update(0.0, 10.0);
        pc.update(0.0, 10.0);

        let stats = pc.stats();
        assert_eq!(stats.num_updates, 3);
        assert!((stats.max_power_dbm - 13.0).abs() < 1e-9);
        assert!((stats.min_power_dbm - 11.0).abs() < 1e-9);
        // avg = (11 + 12 + 13) / 3 = 12
        assert!((stats.avg_power_dbm - 12.0).abs() < 1e-9);
    }

    #[test]
    fn test_initial_power() {
        let pc = ClosedLoopPowerControl::new(15.0, 1.0, 23.0, -40.0);
        assert!((pc.power() - 15.0).abs() < 1e-9);

        let pc2 = ClosedLoopPowerControl::new(-5.0, 0.5, 20.0, -30.0);
        assert!((pc2.power() - (-5.0)).abs() < 1e-9);
    }

    #[test]
    fn test_empty_updates() {
        let pc = ClosedLoopPowerControl::new(10.0, 1.0, 23.0, -40.0);
        let stats = pc.stats();
        assert_eq!(stats.num_updates, 0);
        assert!((stats.avg_power_dbm - 0.0).abs() < 1e-9);
    }
}

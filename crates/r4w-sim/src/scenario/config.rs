//! Scenario configuration
//!
//! Defines the parameters for a generic multi-emitter IQ scenario.

use serde::{Deserialize, Serialize};

/// Configuration for the scenario engine
// trace:FR-039 | ai:claude
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScenarioConfig {
    /// Scenario duration in seconds
    pub duration_s: f64,
    /// Output sample rate in Hz
    pub sample_rate: f64,
    /// Center frequency in Hz (for baseband conversion)
    pub center_frequency_hz: f64,
    /// Processing block size in samples
    pub block_size: usize,
    /// Receiver noise floor in dBW/Hz (thermal noise density)
    pub noise_floor_dbw_hz: f64,
    /// Random seed for reproducibility
    pub seed: u64,
}

impl Default for ScenarioConfig {
    fn default() -> Self {
        Self {
            duration_s: 0.001,           // 1 ms
            sample_rate: 2_046_000.0,    // 2.046 MHz (2x GPS L1 chipping rate)
            center_frequency_hz: 1_575_420_000.0, // GPS L1
            block_size: 2046,            // 1 code period at 2.046 MHz
            noise_floor_dbw_hz: -204.0,  // ~kTB at 290K
            seed: 42,
        }
    }
}

impl ScenarioConfig {
    /// Total number of samples for the full scenario
    pub fn total_samples(&self) -> usize {
        (self.duration_s * self.sample_rate).ceil() as usize
    }

    /// Number of blocks needed
    pub fn num_blocks(&self) -> usize {
        let total = self.total_samples();
        (total + self.block_size - 1) / self.block_size
    }

    /// Noise power in linear scale for the configured bandwidth
    pub fn noise_power_linear(&self) -> f64 {
        // N = N0 * BW where N0 = 10^(noise_floor_dbw_hz/10)
        let n0_linear = 10.0_f64.powf(self.noise_floor_dbw_hz / 10.0);
        n0_linear * self.sample_rate
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let cfg = ScenarioConfig::default();
        assert_eq!(cfg.total_samples(), 2046);
        assert_eq!(cfg.num_blocks(), 1);
    }

    #[test]
    fn test_noise_power() {
        let cfg = ScenarioConfig::default();
        let noise = cfg.noise_power_linear();
        // Should be a very small but positive number
        assert!(noise > 0.0);
        assert!(noise < 1e-10);
    }
}

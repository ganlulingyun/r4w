//! # Noise Figure Calculator
//!
//! Cascaded noise figure and noise temperature analysis for RF receiver chains
//! using the Friis equation. Computes system-level noise performance from
//! individual stage specifications.
//!
//! ## Friis Equation
//!
//! F_total = F1 + (F2-1)/G1 + (F3-1)/(G1*G2) + ...
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::noise_figure::{RfStage, CascadedAnalysis};
//!
//! let chain = vec![
//!     RfStage::new("LNA", 1.5, 20.0),     // NF=1.5dB, Gain=20dB
//!     RfStage::new("Mixer", 8.0, -6.0),    // NF=8dB, Loss=6dB
//!     RfStage::new("IF Amp", 3.0, 30.0),   // NF=3dB, Gain=30dB
//! ];
//!
//! let result = CascadedAnalysis::analyze(&chain);
//! assert!(result.total_nf_db < 2.0);  // System NF dominated by LNA
//! assert!(result.total_gain_db > 40.0);
//! ```

/// An RF stage with noise figure and gain.
#[derive(Debug, Clone)]
pub struct RfStage {
    /// Stage name.
    pub name: String,
    /// Noise figure in dB.
    pub nf_db: f64,
    /// Gain in dB (negative for loss).
    pub gain_db: f64,
}

impl RfStage {
    /// Create a new RF stage.
    pub fn new(name: &str, nf_db: f64, gain_db: f64) -> Self {
        Self {
            name: name.to_string(),
            nf_db,
            gain_db,
        }
    }

    /// Create a passive stage (NF = |loss| in dB).
    pub fn passive(name: &str, loss_db: f64) -> Self {
        Self {
            name: name.to_string(),
            nf_db: loss_db.abs(),
            gain_db: -loss_db.abs(),
        }
    }

    /// Get the noise factor (linear, not dB).
    pub fn noise_factor(&self) -> f64 {
        db_to_linear(self.nf_db)
    }

    /// Get the gain (linear, not dB).
    pub fn gain_linear(&self) -> f64 {
        db_to_linear(self.gain_db)
    }

    /// Get the equivalent noise temperature (Kelvin) at T0=290K.
    pub fn noise_temperature(&self) -> f64 {
        290.0 * (self.noise_factor() - 1.0)
    }
}

/// Result of a cascaded noise figure analysis.
#[derive(Debug, Clone)]
pub struct CascadedResult {
    /// Total system noise figure in dB.
    pub total_nf_db: f64,
    /// Total system noise factor (linear).
    pub total_noise_factor: f64,
    /// Total system gain in dB.
    pub total_gain_db: f64,
    /// System noise temperature (Kelvin).
    pub noise_temperature_k: f64,
    /// Per-stage cumulative NF (dB) at each point in the chain.
    pub cumulative_nf_db: Vec<f64>,
    /// Per-stage cumulative gain (dB).
    pub cumulative_gain_db: Vec<f64>,
}

/// Cascaded noise figure analysis using Friis equation.
pub struct CascadedAnalysis;

impl CascadedAnalysis {
    /// Analyze a chain of RF stages.
    pub fn analyze(stages: &[RfStage]) -> CascadedResult {
        if stages.is_empty() {
            return CascadedResult {
                total_nf_db: 0.0,
                total_noise_factor: 1.0,
                total_gain_db: 0.0,
                noise_temperature_k: 0.0,
                cumulative_nf_db: Vec::new(),
                cumulative_gain_db: Vec::new(),
            };
        }

        let mut f_total = stages[0].noise_factor();
        let mut g_product = stages[0].gain_linear();
        let mut total_gain_db = stages[0].gain_db;
        let mut cumulative_nf_db = vec![stages[0].nf_db];
        let mut cumulative_gain_db = vec![stages[0].gain_db];

        for stage in &stages[1..] {
            let f_i = stage.noise_factor();
            f_total += (f_i - 1.0) / g_product;
            g_product *= stage.gain_linear();
            total_gain_db += stage.gain_db;
            cumulative_nf_db.push(linear_to_db(f_total));
            cumulative_gain_db.push(total_gain_db);
        }

        CascadedResult {
            total_nf_db: linear_to_db(f_total),
            total_noise_factor: f_total,
            total_gain_db,
            noise_temperature_k: 290.0 * (f_total - 1.0),
            cumulative_nf_db,
            cumulative_gain_db,
        }
    }

    /// Compute sensitivity for a given bandwidth and minimum SNR.
    /// Returns minimum detectable signal (MDS) in dBm.
    pub fn sensitivity_dbm(nf_db: f64, bandwidth_hz: f64, min_snr_db: f64) -> f64 {
        // MDS = -174 + 10*log10(BW) + NF + SNR_min
        -174.0 + 10.0 * bandwidth_hz.log10() + nf_db + min_snr_db
    }

    /// Compute noise power in dBm for a given NF and bandwidth.
    pub fn noise_power_dbm(nf_db: f64, bandwidth_hz: f64) -> f64 {
        // P_noise = kTB * F = -174 + 10*log10(BW) + NF
        -174.0 + 10.0 * bandwidth_hz.log10() + nf_db
    }

    /// Convert noise temperature to noise figure.
    pub fn temperature_to_nf_db(temp_k: f64) -> f64 {
        linear_to_db(1.0 + temp_k / 290.0)
    }

    /// Convert noise figure to noise temperature.
    pub fn nf_db_to_temperature(nf_db: f64) -> f64 {
        290.0 * (db_to_linear(nf_db) - 1.0)
    }
}

fn db_to_linear(db: f64) -> f64 {
    10.0_f64.powf(db / 10.0)
}

fn linear_to_db(linear: f64) -> f64 {
    10.0 * linear.log10()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_single_stage() {
        let stages = vec![RfStage::new("LNA", 2.0, 20.0)];
        let result = CascadedAnalysis::analyze(&stages);
        assert!((result.total_nf_db - 2.0).abs() < 1e-6);
        assert!((result.total_gain_db - 20.0).abs() < 1e-6);
    }

    #[test]
    fn test_two_stage_friis() {
        // LNA: NF=1dB, Gain=20dB; Mixer: NF=10dB, Gain=-6dB
        let stages = vec![
            RfStage::new("LNA", 1.0, 20.0),
            RfStage::new("Mixer", 10.0, -6.0),
        ];
        let result = CascadedAnalysis::analyze(&stages);
        // F_total = F1 + (F2-1)/G1 = 1.259 + (10-1)/100 = 1.259 + 0.09 = 1.349
        // NF_total ≈ 1.3 dB
        assert!(result.total_nf_db < 1.5);
        assert!(result.total_nf_db > 1.0);
        assert!((result.total_gain_db - 14.0).abs() < 1e-6);
    }

    #[test]
    fn test_three_stage_receiver() {
        let chain = vec![
            RfStage::new("LNA", 1.5, 20.0),
            RfStage::new("Mixer", 8.0, -6.0),
            RfStage::new("IF Amp", 3.0, 30.0),
        ];
        let result = CascadedAnalysis::analyze(&chain);
        // System NF dominated by LNA.
        assert!(result.total_nf_db < 2.5);
        assert!((result.total_gain_db - 44.0).abs() < 1e-6);
        assert_eq!(result.cumulative_nf_db.len(), 3);
        assert_eq!(result.cumulative_gain_db.len(), 3);
    }

    #[test]
    fn test_passive_stage() {
        let cable = RfStage::passive("Cable", 3.0);
        assert!((cable.nf_db - 3.0).abs() < 1e-10);
        assert!((cable.gain_db - (-3.0)).abs() < 1e-10);
    }

    #[test]
    fn test_noise_temperature() {
        let stage = RfStage::new("LNA", 0.5, 20.0);
        let temp = stage.noise_temperature();
        assert!(temp > 0.0);
        assert!(temp < 50.0); // 0.5dB NF → about 35K
    }

    #[test]
    fn test_sensitivity() {
        // Typical receiver: NF=6dB, BW=200kHz, SNR_min=10dB
        let mds = CascadedAnalysis::sensitivity_dbm(6.0, 200_000.0, 10.0);
        // -174 + 53 + 6 + 10 = -105 dBm
        assert!((mds - (-105.0)).abs() < 1.0);
    }

    #[test]
    fn test_noise_power() {
        let noise = CascadedAnalysis::noise_power_dbm(3.0, 1_000_000.0);
        // -174 + 60 + 3 = -111 dBm
        assert!((noise - (-111.0)).abs() < 0.1);
    }

    #[test]
    fn test_temperature_conversion_roundtrip() {
        let nf = 3.0;
        let temp = CascadedAnalysis::nf_db_to_temperature(nf);
        let nf_back = CascadedAnalysis::temperature_to_nf_db(temp);
        assert!((nf - nf_back).abs() < 1e-6);
    }

    #[test]
    fn test_empty_chain() {
        let result = CascadedAnalysis::analyze(&[]);
        assert!((result.total_nf_db - 0.0).abs() < 1e-10);
        assert!((result.total_gain_db - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_cumulative_nf_increases() {
        let chain = vec![
            RfStage::new("LNA", 1.0, 15.0),
            RfStage::new("Filter", 2.0, -2.0),
            RfStage::new("Mixer", 8.0, 5.0),
        ];
        let result = CascadedAnalysis::analyze(&chain);
        // Cumulative NF should monotonically increase.
        for i in 1..result.cumulative_nf_db.len() {
            assert!(
                result.cumulative_nf_db[i] >= result.cumulative_nf_db[i - 1],
                "Cumulative NF should increase"
            );
        }
    }
}

//! Link Budget Calculator — RF System Design
//!
//! Computes end-to-end RF link budgets including transmit power, antenna
//! gains, free-space path loss (FSPL), atmospheric attenuation, receiver
//! noise figure, and resulting SNR/C/N0 margin. Also models cascaded
//! receiver chains using Friis's noise figure formula.
//!
//! No direct GNU Radio equivalent — bridges the gap between signal
//! processing blocks and real system deployment.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::link_budget::LinkBudget;
//!
//! let result = LinkBudget::new()
//!     .tx_power_dbm(30.0)        // 1 W
//!     .tx_antenna_gain_dbi(2.0)
//!     .rx_antenna_gain_dbi(2.0)
//!     .frequency_hz(915e6)       // 915 MHz ISM
//!     .distance_m(10_000.0)      // 10 km
//!     .rx_noise_figure_db(6.0)
//!     .bandwidth_hz(125_000.0)   // 125 kHz (LoRa)
//!     .compute();
//!
//! assert!(result.snr_db > 0.0);
//! ```

use std::f64::consts::PI;

/// Speed of light (m/s).
const C: f64 = 299_792_458.0;

/// Link budget result.
#[derive(Debug, Clone)]
pub struct LinkBudgetResult {
    /// Effective Isotropic Radiated Power (dBm).
    pub eirp_dbm: f64,
    /// Free-space path loss (dB, positive value).
    pub fspl_db: f64,
    /// Received power (dBm).
    pub received_power_dbm: f64,
    /// Noise floor at receiver (dBm).
    pub noise_floor_dbm: f64,
    /// Signal-to-Noise Ratio (dB).
    pub snr_db: f64,
    /// Carrier-to-Noise density ratio (dB-Hz).
    pub cn0_dbhz: f64,
    /// Link margin above required SNR (dB). NaN if no required_snr set.
    pub margin_db: f64,
}

/// Link budget builder.
#[derive(Debug, Clone)]
pub struct LinkBudget {
    tx_power_dbm: f64,
    tx_antenna_gain_dbi: f64,
    rx_antenna_gain_dbi: f64,
    frequency_hz: f64,
    distance_m: f64,
    cable_loss_db: f64,
    atmospheric_loss_db: f64,
    rx_noise_figure_db: f64,
    bandwidth_hz: f64,
    temperature_k: f64,
    required_snr_db: Option<f64>,
}

impl LinkBudget {
    pub fn new() -> Self {
        Self {
            tx_power_dbm: 0.0,
            tx_antenna_gain_dbi: 0.0,
            rx_antenna_gain_dbi: 0.0,
            frequency_hz: 1e9,
            distance_m: 1000.0,
            cable_loss_db: 0.0,
            atmospheric_loss_db: 0.0,
            rx_noise_figure_db: 0.0,
            bandwidth_hz: 1e6,
            temperature_k: 290.0,
            required_snr_db: None,
        }
    }

    pub fn tx_power_dbm(mut self, power: f64) -> Self {
        self.tx_power_dbm = power;
        self
    }

    pub fn tx_antenna_gain_dbi(mut self, gain: f64) -> Self {
        self.tx_antenna_gain_dbi = gain;
        self
    }

    pub fn rx_antenna_gain_dbi(mut self, gain: f64) -> Self {
        self.rx_antenna_gain_dbi = gain;
        self
    }

    pub fn frequency_hz(mut self, freq: f64) -> Self {
        self.frequency_hz = freq;
        self
    }

    pub fn distance_m(mut self, dist: f64) -> Self {
        self.distance_m = dist;
        self
    }

    pub fn cable_loss_db(mut self, loss: f64) -> Self {
        self.cable_loss_db = loss;
        self
    }

    pub fn atmospheric_loss_db(mut self, loss: f64) -> Self {
        self.atmospheric_loss_db = loss;
        self
    }

    pub fn rx_noise_figure_db(mut self, nf: f64) -> Self {
        self.rx_noise_figure_db = nf;
        self
    }

    pub fn bandwidth_hz(mut self, bw: f64) -> Self {
        self.bandwidth_hz = bw;
        self
    }

    pub fn temperature_k(mut self, temp: f64) -> Self {
        self.temperature_k = temp;
        self
    }

    pub fn required_snr_db(mut self, snr: f64) -> Self {
        self.required_snr_db = Some(snr);
        self
    }

    /// Compute the full link budget.
    pub fn compute(&self) -> LinkBudgetResult {
        let eirp_dbm = self.tx_power_dbm + self.tx_antenna_gain_dbi - self.cable_loss_db;
        let fspl = free_space_path_loss_db(self.frequency_hz, self.distance_m);
        let received_power_dbm =
            eirp_dbm - fspl - self.atmospheric_loss_db + self.rx_antenna_gain_dbi;

        let noise_floor = thermal_noise_floor_dbm(self.bandwidth_hz, self.temperature_k)
            + self.rx_noise_figure_db;

        let snr_db = received_power_dbm - noise_floor;
        let cn0_dbhz = received_power_dbm - noise_floor + 10.0 * self.bandwidth_hz.log10();

        let margin_db = match self.required_snr_db {
            Some(req) => snr_db - req,
            None => f64::NAN,
        };

        LinkBudgetResult {
            eirp_dbm,
            fspl_db: fspl,
            received_power_dbm,
            noise_floor_dbm: noise_floor,
            snr_db,
            cn0_dbhz,
            margin_db,
        }
    }
}

impl Default for LinkBudget {
    fn default() -> Self {
        Self::new()
    }
}

/// Free-space path loss in dB.
///
/// FSPL = 20*log10(4*pi*d*f/c)
///
/// Returns 0.0 for zero distance.
pub fn free_space_path_loss_db(freq_hz: f64, distance_m: f64) -> f64 {
    if distance_m <= 0.0 || freq_hz <= 0.0 {
        return 0.0;
    }
    20.0 * (4.0 * PI * distance_m * freq_hz / C).log10()
}

/// Thermal noise floor in dBm.
///
/// N = 10*log10(k*T*B) + 30  (convert from dBW to dBm)
/// At 290K: N ≈ -174 + 10*log10(BW_hz) dBm
pub fn thermal_noise_floor_dbm(bandwidth_hz: f64, temperature_k: f64) -> f64 {
    const K_BOLTZMANN: f64 = 1.380649e-23; // J/K
    if bandwidth_hz <= 0.0 {
        return f64::NEG_INFINITY;
    }
    10.0 * (K_BOLTZMANN * temperature_k * bandwidth_hz).log10() + 30.0
}

/// Compute maximum communication range given link parameters.
///
/// Inverts the FSPL formula: d = c/(4*pi*f) * 10^((EIRP - P_rx_min - L_extra) / 20)
pub fn max_range_m(
    eirp_dbm: f64,
    rx_sensitivity_dbm: f64,
    freq_hz: f64,
    extra_losses_db: f64,
) -> f64 {
    if freq_hz <= 0.0 {
        return 0.0;
    }
    let max_fspl = eirp_dbm - rx_sensitivity_dbm - extra_losses_db;
    if max_fspl <= 0.0 {
        return 0.0;
    }
    (C / (4.0 * PI * freq_hz)) * 10.0_f64.powf(max_fspl / 20.0)
}

/// Cascaded receiver chain noise figure calculator (Friis formula).
#[derive(Debug, Clone)]
pub struct CascadedReceiver {
    /// (gain_db, noise_figure_db) for each stage.
    stages: Vec<(f64, f64)>,
}

impl CascadedReceiver {
    pub fn new() -> Self {
        Self { stages: Vec::new() }
    }

    /// Add a receiver stage with gain and noise figure (both in dB).
    pub fn add_stage(mut self, gain_db: f64, noise_figure_db: f64) -> Self {
        self.stages.push((gain_db, noise_figure_db));
        self
    }

    /// Compute the cascaded (total system) noise figure in dB.
    ///
    /// Friis formula: F_total = F_1 + (F_2-1)/G_1 + (F_3-1)/(G_1*G_2) + ...
    pub fn total_noise_figure_db(&self) -> f64 {
        if self.stages.is_empty() {
            return 0.0;
        }

        // Convert to linear scale
        let mut f_total_linear = db_to_linear(self.stages[0].1); // F_1
        let mut cumulative_gain_linear = db_to_linear(self.stages[0].0); // G_1

        for i in 1..self.stages.len() {
            let f_i = db_to_linear(self.stages[i].1);
            f_total_linear += (f_i - 1.0) / cumulative_gain_linear;
            cumulative_gain_linear *= db_to_linear(self.stages[i].0);
        }

        linear_to_db(f_total_linear)
    }

    /// Compute total gain in dB.
    pub fn total_gain_db(&self) -> f64 {
        self.stages.iter().map(|(g, _)| g).sum()
    }
}

impl Default for CascadedReceiver {
    fn default() -> Self {
        Self::new()
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
    fn test_fspl_known_values() {
        // 1 GHz at 1 km ≈ 92.45 dB
        let fspl = free_space_path_loss_db(1e9, 1000.0);
        assert!(
            (fspl - 92.45).abs() < 0.1,
            "FSPL at 1GHz/1km = {fspl:.2} dB, expected ~92.45"
        );

        // 2.4 GHz at 100 m ≈ 80.0 dB
        let fspl2 = free_space_path_loss_db(2.4e9, 100.0);
        assert!(
            (fspl2 - 80.04).abs() < 0.2,
            "FSPL at 2.4GHz/100m = {fspl2:.2} dB, expected ~80.0"
        );
    }

    #[test]
    fn test_thermal_noise_floor() {
        // 1 MHz BW at 290K ≈ -114 dBm
        let nf = thermal_noise_floor_dbm(1e6, 290.0);
        assert!(
            (nf - (-114.0)).abs() < 0.1,
            "noise floor = {nf:.1} dBm, expected ~-114"
        );
    }

    #[test]
    fn test_complete_link_budget() {
        let result = LinkBudget::new()
            .tx_power_dbm(30.0) // 1 W
            .tx_antenna_gain_dbi(0.0)
            .rx_antenna_gain_dbi(0.0)
            .frequency_hz(915e6)
            .distance_m(10_000.0)
            .rx_noise_figure_db(10.0)
            .bandwidth_hz(125_000.0)
            .compute();

        // EIRP = 30 dBm
        assert!((result.eirp_dbm - 30.0).abs() < 0.01);
        // FSPL at 915 MHz / 10 km ≈ 111.7 dB
        assert!(
            (result.fspl_db - 111.7).abs() < 0.5,
            "FSPL = {:.1}",
            result.fspl_db
        );
        // Verify SNR is reasonable
        assert!(result.snr_db.is_finite());
    }

    #[test]
    fn test_friis_cascaded_nf() {
        // LNA: gain 20 dB, NF 1 dB
        // Mixer: gain -6 dB, NF 10 dB
        // IF amp: gain 30 dB, NF 5 dB
        let rx = CascadedReceiver::new()
            .add_stage(20.0, 1.0)
            .add_stage(-6.0, 10.0)
            .add_stage(30.0, 5.0);

        let nf = rx.total_noise_figure_db();
        // F1 = 1.259, G1 = 100 (20 dB)
        // F2 = 10.0, (F2-1)/G1 = 0.090
        // G1*G2 = 100 * 0.251 = 25.1
        // F3 = 3.162, (F3-1)/25.1 = 0.086
        // F_total = 1.259 + 0.090 + 0.086 = 1.435 → 1.57 dB
        assert!(
            (nf - 1.57).abs() < 0.15,
            "cascaded NF = {nf:.2} dB, expected ~1.57"
        );
    }

    #[test]
    fn test_friis_single_stage() {
        let rx = CascadedReceiver::new().add_stage(20.0, 3.0);
        let nf = rx.total_noise_figure_db();
        assert!(
            (nf - 3.0).abs() < 0.01,
            "single stage NF = {nf:.2}, expected 3.0"
        );
    }

    #[test]
    fn test_max_range() {
        // 14 dBm EIRP, -137 dBm sensitivity, 868 MHz
        let range = max_range_m(14.0, -137.0, 868e6, 0.0);
        assert!(
            range > 10_000.0,
            "LoRa SF12 range should be > 10 km, got {range:.0} m"
        );
    }

    #[test]
    fn test_gps_link_budget() {
        // GPS SV EIRP ≈ 26.8 dBW = 56.8 dBm (L1 C/A), 1575.42 MHz, 20200 km orbit
        let result = LinkBudget::new()
            .tx_power_dbm(56.8)
            .tx_antenna_gain_dbi(0.0) // Included in EIRP
            .rx_antenna_gain_dbi(3.0) // Patch antenna
            .frequency_hz(1575.42e6)
            .distance_m(20_200_000.0)
            .rx_noise_figure_db(2.0)
            .bandwidth_hz(2.046e6) // GPS C/A bandwidth
            .compute();

        // C/N0 should be ~40-55 dB-Hz
        assert!(
            result.cn0_dbhz > 35.0 && result.cn0_dbhz < 60.0,
            "GPS C/N0 = {:.1} dB-Hz, expected 35-60",
            result.cn0_dbhz
        );
    }

    #[test]
    fn test_zero_distance() {
        let fspl = free_space_path_loss_db(1e9, 0.0);
        assert_eq!(fspl, 0.0, "zero distance should give 0 FSPL");
        assert!(!fspl.is_nan());
    }

    #[test]
    fn test_margin_calculation() {
        let result = LinkBudget::new()
            .tx_power_dbm(30.0)
            .frequency_hz(915e6)
            .distance_m(1000.0)
            .rx_noise_figure_db(6.0)
            .bandwidth_hz(125_000.0)
            .required_snr_db(10.0)
            .compute();

        assert!(result.margin_db.is_finite());
        assert!(
            (result.margin_db - (result.snr_db - 10.0)).abs() < 0.01,
            "margin should be SNR - required_SNR"
        );
    }

    #[test]
    fn test_total_gain() {
        let rx = CascadedReceiver::new()
            .add_stage(20.0, 1.0)
            .add_stage(-6.0, 10.0)
            .add_stage(30.0, 5.0);
        let gain = rx.total_gain_db();
        assert!(
            (gain - 44.0).abs() < 0.01,
            "total gain = {gain}, expected 44 dB"
        );
    }
}

//! LPI/LPD Metrics and Analysis
//!
//! This module provides tools for analyzing and quantifying the Low Probability
//! of Intercept (LPI) and Low Probability of Detection (LPD) characteristics
//! of spread spectrum waveforms.
//!
//! ## Key Metrics
//!
//! - **Processing Gain**: How much the signal is spread (dB)
//! - **Power Spectral Density**: Signal power per Hz
//! - **Margin Below Noise Floor**: How far below thermal noise
//! - **Detection Probability**: Likelihood of intercept
//!
//! ## Thermal Noise Floor
//!
//! The fundamental noise floor at temperature T:
//! ```text
//! N₀ = kTB where k = 1.38e-23 J/K (Boltzmann's constant)
//!
//! At room temperature (290K):
//! N₀ = -174 dBm/Hz
//!
//! With receiver noise figure NF:
//! Effective noise floor = -174 + NF dBm/Hz
//! ```


/// Boltzmann's constant (J/K)
pub const BOLTZMANN_K: f64 = 1.380649e-23;

/// Room temperature in Kelvin
pub const ROOM_TEMP_K: f64 = 290.0;

/// Thermal noise floor at room temperature (dBm/Hz)
pub const THERMAL_NOISE_FLOOR_DBM_HZ: f64 = -174.0;

/// Calculate processing gain in dB
///
/// Processing gain = 10 * log10(spread_bandwidth / data_bandwidth)
///                 = 10 * log10(chip_rate / symbol_rate)
///                 = 10 * log10(chips_per_symbol)
pub fn processing_gain_db(chips_per_symbol: usize) -> f64 {
    10.0 * (chips_per_symbol as f64).log10()
}

/// Calculate processing gain from bandwidth ratio
pub fn processing_gain_from_bandwidth(spread_bw: f64, data_bw: f64) -> f64 {
    10.0 * (spread_bw / data_bw).log10()
}

/// Convert power in Watts to dBm
pub fn watts_to_dbm(watts: f64) -> f64 {
    10.0 * (watts * 1000.0).log10()
}

/// Convert dBm to Watts
pub fn dbm_to_watts(dbm: f64) -> f64 {
    10.0_f64.powf((dbm - 30.0) / 10.0)
}

/// Convert dBm to dBW
pub fn dbm_to_dbw(dbm: f64) -> f64 {
    dbm - 30.0
}

/// Calculate power spectral density (dBm/Hz) from total power and bandwidth
///
/// PSD = P_total - 10*log10(BW)
pub fn power_spectral_density(total_power_dbm: f64, bandwidth_hz: f64) -> f64 {
    total_power_dbm - 10.0 * bandwidth_hz.log10()
}

/// Calculate total power from PSD and bandwidth
///
/// P_total = PSD + 10*log10(BW)
pub fn power_from_psd(psd_dbm_hz: f64, bandwidth_hz: f64) -> f64 {
    psd_dbm_hz + 10.0 * bandwidth_hz.log10()
}

/// Calculate noise floor for a given bandwidth and noise figure
///
/// Returns noise power in dBm
pub fn noise_floor_dbm(bandwidth_hz: f64, noise_figure_db: f64) -> f64 {
    THERMAL_NOISE_FLOOR_DBM_HZ + 10.0 * bandwidth_hz.log10() + noise_figure_db
}

/// Calculate margin below noise floor
///
/// Positive value means signal is below noise floor (harder to detect)
pub fn margin_below_noise(signal_psd_dbm_hz: f64, noise_figure_db: f64) -> f64 {
    let effective_noise_floor = THERMAL_NOISE_FLOOR_DBM_HZ + noise_figure_db;
    effective_noise_floor - signal_psd_dbm_hz
}

/// LPI/LPD analysis result
#[derive(Debug, Clone)]
pub struct LpiAnalysis {
    /// Processing gain in dB
    pub processing_gain_db: f64,
    /// Spread bandwidth in Hz
    pub spread_bandwidth_hz: f64,
    /// Data bandwidth in Hz
    pub data_bandwidth_hz: f64,
    /// Transmit power in dBm
    pub tx_power_dbm: f64,
    /// Power spectral density in dBm/Hz
    pub psd_dbm_hz: f64,
    /// Margin below thermal noise floor (positive = below)
    pub margin_below_noise_db: f64,
    /// Equivalent isotropically radiated power (dBm)
    pub eirp_dbm: Option<f64>,
    /// Detection range estimate (km) for reference receiver
    pub detection_range_km: Option<f64>,
}

impl LpiAnalysis {
    /// Create a new LPI analysis
    pub fn new(
        chips_per_symbol: usize,
        chip_rate_hz: f64,
        symbol_rate_hz: f64,
        tx_power_dbm: f64,
    ) -> Self {
        let processing_gain_db = processing_gain_db(chips_per_symbol);
        let spread_bandwidth_hz = chip_rate_hz;
        let data_bandwidth_hz = symbol_rate_hz;
        let psd_dbm_hz = power_spectral_density(tx_power_dbm, spread_bandwidth_hz);
        let margin_below_noise_db = margin_below_noise(psd_dbm_hz, 10.0); // Assume 10 dB NF

        Self {
            processing_gain_db,
            spread_bandwidth_hz,
            data_bandwidth_hz,
            tx_power_dbm,
            psd_dbm_hz,
            margin_below_noise_db,
            eirp_dbm: None,
            detection_range_km: None,
        }
    }

    /// Check if signal is below noise floor
    pub fn is_below_noise_floor(&self) -> bool {
        self.margin_below_noise_db > 0.0
    }

    /// Get LPI rating (qualitative)
    pub fn lpi_rating(&self) -> &'static str {
        if self.margin_below_noise_db > 20.0 {
            "Excellent"
        } else if self.margin_below_noise_db > 10.0 {
            "Very Good"
        } else if self.margin_below_noise_db > 0.0 {
            "Good"
        } else if self.margin_below_noise_db > -10.0 {
            "Moderate"
        } else {
            "Poor"
        }
    }
}

/// Analyze DSSS waveform for LPI properties
pub fn analyze_dsss(
    chips_per_symbol: usize,
    chip_rate_hz: f64,
    tx_power_dbm: f64,
) -> LpiAnalysis {
    let symbol_rate_hz = chip_rate_hz / chips_per_symbol as f64;
    LpiAnalysis::new(chips_per_symbol, chip_rate_hz, symbol_rate_hz, tx_power_dbm)
}

/// Analyze FHSS waveform for LPI properties
pub fn analyze_fhss(
    num_channels: usize,
    hop_bandwidth_hz: f64,
    tx_power_dbm: f64,
) -> LpiAnalysis {
    // For FHSS, processing gain comes from frequency agility
    // Approximate as if spreading across all channels
    let total_bandwidth = num_channels as f64 * hop_bandwidth_hz;
    let processing_gain = processing_gain_from_bandwidth(total_bandwidth, hop_bandwidth_hz);
    let psd_dbm_hz = power_spectral_density(tx_power_dbm, total_bandwidth);
    let margin = margin_below_noise(psd_dbm_hz, 10.0);

    LpiAnalysis {
        processing_gain_db: processing_gain,
        spread_bandwidth_hz: total_bandwidth,
        data_bandwidth_hz: hop_bandwidth_hz,
        tx_power_dbm,
        psd_dbm_hz,
        margin_below_noise_db: margin,
        eirp_dbm: None,
        detection_range_km: None,
    }
}

/// Calculate required processing gain to be below noise floor
///
/// Given TX power and desired margin below noise, calculate required chips/symbol
pub fn required_processing_gain(
    tx_power_dbm: f64,
    data_bandwidth_hz: f64,
    desired_margin_db: f64,
    noise_figure_db: f64,
) -> f64 {
    // PSD = P_tx - 10*log10(BW_spread)
    // We want PSD < noise_floor - margin
    // noise_floor = -174 + NF
    // So: P_tx - 10*log10(BW_spread) < -174 + NF - margin
    // 10*log10(BW_spread) > P_tx + 174 - NF + margin
    // BW_spread > 10^((P_tx + 174 - NF + margin)/10)

    let required_spread_bw_log =
        (tx_power_dbm + 174.0 - noise_figure_db + desired_margin_db) / 10.0;
    let required_spread_bw = 10.0_f64.powf(required_spread_bw_log);

    // Processing gain = spread_bw / data_bw
    10.0 * (required_spread_bw / data_bandwidth_hz).log10()
}

/// Estimate detection probability based on energy detection
///
/// Simplified model for educational purposes
pub fn detection_probability(
    signal_psd_dbm_hz: f64,
    noise_figure_db: f64,
    integration_time_s: f64,
    _false_alarm_rate: f64,
) -> f64 {
    let effective_noise = THERMAL_NOISE_FLOOR_DBM_HZ + noise_figure_db;
    let snr_per_hz = signal_psd_dbm_hz - effective_noise;

    // Time-bandwidth product gives integration gain
    // This is a simplified model
    let integration_gain_db = 10.0 * integration_time_s.log10();
    let effective_snr = snr_per_hz + integration_gain_db;

    // Approximate detection probability using error function
    // Pd ≈ Q(Q^-1(Pfa) - sqrt(2 * SNR_linear))
    let snr_linear = 10.0_f64.powf(effective_snr / 10.0);

    // Simplified: sigmoid-like function
    let x = (snr_linear - 1.0) / 0.5;
    1.0 / (1.0 + (-x).exp())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_processing_gain() {
        assert!((processing_gain_db(127) - 21.03).abs() < 0.1);
        assert!((processing_gain_db(1023) - 30.1).abs() < 0.1);
    }

    #[test]
    fn test_power_conversions() {
        assert!((watts_to_dbm(1.0) - 30.0).abs() < 0.01);
        assert!((watts_to_dbm(0.001) - 0.0).abs() < 0.01);
        assert!((dbm_to_watts(30.0) - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_psd_calculation() {
        // 1W into 1MHz should be -30 dBm/Hz
        let psd = power_spectral_density(30.0, 1_000_000.0);
        assert!((psd - (-30.0)).abs() < 0.1);
    }

    #[test]
    fn test_noise_floor() {
        // 1 Hz bandwidth, 0 dB NF
        let nf = noise_floor_dbm(1.0, 0.0);
        assert!((nf - (-174.0)).abs() < 0.1);

        // 1 MHz bandwidth, 10 dB NF
        let nf = noise_floor_dbm(1_000_000.0, 10.0);
        // -174 + 60 + 10 = -104 dBm
        assert!((nf - (-104.0)).abs() < 0.1);
    }

    #[test]
    fn test_dsss_analysis() {
        // 127 chips/symbol, 1 Mchip/s, 0 dBm TX
        let analysis = analyze_dsss(127, 1_000_000.0, 0.0);

        assert!((analysis.processing_gain_db - 21.0).abs() < 1.0);
        assert!(analysis.spread_bandwidth_hz == 1_000_000.0);
    }

    #[test]
    fn test_lpi_rating() {
        // To be below noise floor, we need VERY wide bandwidth and low power
        // Example: 1023 chips, 100 MHz chip rate, -30 dBm TX power
        // PSD = -30 - 10*log10(100e6) = -30 - 80 = -110 dBm/Hz
        // Noise floor (10 dB NF) = -174 + 10 = -164 dBm/Hz
        // Still above noise floor by 54 dB
        //
        // To be truly below noise floor at 1W (30 dBm), we'd need:
        // 30 - 10*log10(BW) < -164
        // BW > 10^((30+164)/10) = 10^19.4 Hz (not practical!)
        //
        // So for educational purposes, test with very low power:
        // -80 dBm TX, 100 MHz spread: PSD = -80 - 80 = -160 dBm/Hz
        // Still 4 dB above -164 dBm/Hz floor

        let analysis = analyze_dsss(1023, 100_000_000.0, -90.0);
        println!(
            "PSD: {} dBm/Hz, Margin: {} dB, Rating: {}",
            analysis.psd_dbm_hz, analysis.margin_below_noise_db, analysis.lpi_rating()
        );

        // With -90 dBm TX and 100 MHz spread:
        // PSD = -90 - 80 = -170 dBm/Hz, which is 6 dB below -164 dBm/Hz floor
        assert!(analysis.is_below_noise_floor());
        assert_eq!(analysis.lpi_rating(), "Good");
    }
}

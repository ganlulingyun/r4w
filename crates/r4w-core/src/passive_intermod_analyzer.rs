//! # Passive Intermodulation (PIM) Analyzer
//!
//! Analyzes Passive Intermodulation products in RF systems, particularly for
//! cellular base station antennas and tower-mounted equipment per IEC 62037.
//!
//! PIM arises when two or more high-power carriers pass through passive
//! components with nonlinear junctions (corroded connectors, dissimilar metal
//! contacts, ferromagnetic materials). The resulting intermodulation products
//! can fall into receive bands, degrading sensitivity and raising the effective
//! noise floor.
//!
//! ## Intermodulation Frequencies
//!
//! For two carriers at frequencies f1 and f2, intermodulation products appear at:
//!
//!     f_IM = m * f1 + n * f2
//!
//! where the *order* is |m| + |n|. Only odd-order products (3rd, 5th, 7th ...)
//! typically fall near the original carriers and inside receiver bands.
//!
//! ## IEC 62037 Compliance
//!
//! The IEC 62037 series defines measurement methods and pass/fail limits for
//! PIM in RF connectors, cables, and passive devices. This module applies the
//! standard limit thresholds for 3rd through 9th order products.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::passive_intermod_analyzer::{
//!     PimConfig, PimAnalyzer, enumerate_pim_products, pim_in_band,
//!     lte_band_rx_range, compute_im_frequency, iec62037_limit_dbc,
//! };
//!
//! // Two LTE Band 1 downlink carriers
//! let config = PimConfig {
//!     carrier_freq_1_mhz: 2110.0,
//!     carrier_freq_2_mhz: 2140.0,
//!     carrier_power_dbm: 43.0,  // 20 W per carrier
//!     noise_floor_dbm: -130.0,
//! };
//!
//! // Enumerate 3rd-order products
//! let products = enumerate_pim_products(2110.0, 2140.0, 3);
//! for p in &products {
//!     if let Some((rx_low, rx_high)) = lte_band_rx_range(1) {
//!         if pim_in_band(p.frequency_mhz, rx_low, rx_high) {
//!             println!("PIM order {} at {:.1} MHz falls in Band 1 UL!", p.order, p.frequency_mhz);
//!         }
//!     }
//! }
//! ```

use std::f64::consts::PI;

/// Speed of light in metres per second.
const SPEED_OF_LIGHT_M_S: f64 = 299_792_458.0;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for a two-tone PIM test.
#[derive(Debug, Clone)]
pub struct PimConfig {
    /// First carrier frequency in MHz.
    pub carrier_freq_1_mhz: f64,
    /// Second carrier frequency in MHz.
    pub carrier_freq_2_mhz: f64,
    /// Carrier power per tone in dBm (default 43.0 dBm = 20 W).
    pub carrier_power_dbm: f64,
    /// System noise floor in dBm (default -130.0 dBm).
    pub noise_floor_dbm: f64,
}

impl Default for PimConfig {
    fn default() -> Self {
        Self {
            carrier_freq_1_mhz: 1930.0,
            carrier_freq_2_mhz: 1990.0,
            carrier_power_dbm: 43.0,
            noise_floor_dbm: -130.0,
        }
    }
}

// ---------------------------------------------------------------------------
// PIM product descriptor
// ---------------------------------------------------------------------------

/// A single intermodulation product.
#[derive(Debug, Clone, PartialEq)]
pub struct PimProduct {
    /// Intermodulation order (|m| + |n|).
    pub order: usize,
    /// Resulting frequency in MHz.
    pub frequency_mhz: f64,
    /// Power level in dBm (only meaningful after spectrum analysis).
    pub power_dbm: f64,
    /// Coefficient on f1.
    pub m_coefficient: i32,
    /// Coefficient on f2.
    pub n_coefficient: i32,
}

// ---------------------------------------------------------------------------
// Analysis result
// ---------------------------------------------------------------------------

/// Result of a PIM analysis run.
#[derive(Debug, Clone)]
pub struct PimResult {
    /// All detected / enumerated PIM products.
    pub products: Vec<PimProduct>,
    /// Order of the worst (highest-power) PIM product.
    pub worst_pim_order: usize,
    /// Power of the worst PIM product in dBm.
    pub worst_pim_dbm: f64,
    /// Worst PIM level relative to carrier power in dBc.
    pub pim_dbc: f64,
    /// Whether all products meet IEC 62037 limits.
    pub pass: bool,
}

// ---------------------------------------------------------------------------
// Analyzer
// ---------------------------------------------------------------------------

/// PIM spectrum analyzer.
///
/// Given a measured spectrum (power in dBm vs frequency in MHz), identifies
/// intermodulation products up to a configurable maximum order and evaluates
/// them against IEC 62037 pass/fail thresholds.
#[derive(Debug, Clone)]
pub struct PimAnalyzer {
    config: PimConfig,
    /// Maximum intermodulation order to search for.
    pub max_order: usize,
    /// Frequency tolerance for matching predicted IM frequencies to spectrum
    /// bins, in MHz.
    pub freq_tolerance_mhz: f64,
}

impl PimAnalyzer {
    /// Create a new analyzer with default max_order = 9 and 0.5 MHz tolerance.
    pub fn new(config: PimConfig) -> Self {
        Self {
            config,
            max_order: 9,
            freq_tolerance_mhz: 0.5,
        }
    }

    /// Analyze a measured spectrum for PIM products.
    ///
    /// `spectrum` contains power values in dBm, one per frequency bin.
    /// `freq_axis_mhz` contains the corresponding centre frequency of each bin.
    /// Both slices must have the same length.
    pub fn analyze(&self, spectrum: &[f64], freq_axis_mhz: &[f64]) -> PimResult {
        assert_eq!(
            spectrum.len(),
            freq_axis_mhz.len(),
            "spectrum and freq_axis_mhz must have the same length"
        );

        let predicted = enumerate_pim_products(
            self.config.carrier_freq_1_mhz,
            self.config.carrier_freq_2_mhz,
            self.max_order,
        );

        let mut detected: Vec<PimProduct> = Vec::new();

        for mut prod in predicted {
            // Find the closest spectrum bin to the predicted frequency.
            if let Some(power) = self.find_peak_near(
                prod.frequency_mhz,
                spectrum,
                freq_axis_mhz,
            ) {
                // Only report products above the noise floor.
                if power > self.config.noise_floor_dbm {
                    prod.power_dbm = power;
                    detected.push(prod);
                }
            }
        }

        // Determine worst product.
        let (worst_order, worst_dbm) = detected
            .iter()
            .max_by(|a, b| a.power_dbm.partial_cmp(&b.power_dbm).unwrap())
            .map(|p| (p.order, p.power_dbm))
            .unwrap_or((0, self.config.noise_floor_dbm));

        let pim_dbc = worst_dbm - self.config.carrier_power_dbm;

        // Check IEC 62037 pass/fail for each detected product.
        let pass = detected.iter().all(|p| {
            let limit = iec62037_limit_dbc(p.order);
            (p.power_dbm - self.config.carrier_power_dbm) <= limit
        });

        PimResult {
            products: detected,
            worst_pim_order: worst_order,
            worst_pim_dbm: worst_dbm,
            pim_dbc,
            pass,
        }
    }

    /// Find the peak power near `target_freq_mhz` within `freq_tolerance_mhz`.
    fn find_peak_near(
        &self,
        target_freq_mhz: f64,
        spectrum: &[f64],
        freq_axis_mhz: &[f64],
    ) -> Option<f64> {
        let mut best_power: Option<f64> = None;

        for (i, &freq) in freq_axis_mhz.iter().enumerate() {
            if (freq - target_freq_mhz).abs() <= self.freq_tolerance_mhz {
                let power = spectrum[i];
                best_power = Some(match best_power {
                    Some(prev) => if power > prev { power } else { prev },
                    None => power,
                });
            }
        }

        best_power
    }
}

// ---------------------------------------------------------------------------
// Pure functions
// ---------------------------------------------------------------------------

/// Compute the intermodulation frequency for coefficients m, n.
///
///     f_IM = m * f1 + n * f2
pub fn compute_im_frequency(f1: f64, f2: f64, m: i32, n: i32) -> f64 {
    (m as f64) * f1 + (n as f64) * f2
}

/// Compute the intermodulation order for coefficients m, n.
///
///     order = |m| + |n|
pub fn compute_im_order(m: i32, n: i32) -> usize {
    (m.unsigned_abs() + n.unsigned_abs()) as usize
}

/// Enumerate all intermodulation products of two carriers up to `max_order`.
///
/// Returns products for all coefficient pairs (m, n) where |m| + |n| <= max_order
/// and |m| + |n| >= 2, with the resulting frequency being positive.
/// Products at the fundamental frequencies (order 1) are excluded.
pub fn enumerate_pim_products(f1: f64, f2: f64, max_order: usize) -> Vec<PimProduct> {
    let mut products = Vec::new();
    let max_o = max_order as i32;

    for m in -max_o..=max_o {
        for n in -max_o..=max_o {
            let order = compute_im_order(m, n);
            if order < 2 || order > max_order {
                continue;
            }
            let freq = compute_im_frequency(f1, f2, m, n);
            if freq <= 0.0 {
                continue;
            }
            products.push(PimProduct {
                order,
                frequency_mhz: freq,
                power_dbm: f64::NEG_INFINITY,
                m_coefficient: m,
                n_coefficient: n,
            });
        }
    }

    // Sort by order, then frequency.
    products.sort_by(|a, b| {
        a.order
            .cmp(&b.order)
            .then_with(|| a.frequency_mhz.partial_cmp(&b.frequency_mhz).unwrap())
    });

    products
}

/// Check whether a PIM product frequency falls within a receive band.
pub fn pim_in_band(product_freq: f64, rx_low_mhz: f64, rx_high_mhz: f64) -> bool {
    product_freq >= rx_low_mhz && product_freq <= rx_high_mhz
}

/// IEC 62037 PIM limit in dBc for a given intermodulation order.
///
/// The standard specifies limits relative to the carrier power. Typical
/// acceptance thresholds (two 20 W carriers at +43 dBm each):
///
/// | Order | Limit (dBc) | Absolute (dBm) |
/// |-------|-------------|----------------|
/// |   3   |   -150      |   -107         |
/// |   5   |   -160      |   -117         |
/// |   7   |   -165      |   -122         |
/// |   9   |   -170      |   -127         |
///
/// Orders above 9 return -175 dBc as a conservative default.
pub fn iec62037_limit_dbc(order: usize) -> f64 {
    match order {
        3 => -150.0,
        5 => -160.0,
        7 => -165.0,
        9 => -170.0,
        _ if order > 9 => -175.0,
        // Even orders or order < 3 get a generous limit since they are
        // rarely problematic in passive devices.
        _ => -140.0,
    }
}

/// Estimate theoretical PIM power for a given carrier power and order.
///
/// A simplified model:
///     P_IM(dBm) = pim_coefficient_dbm + order * carrier_power_dbm
///
/// In practice `pim_coefficient_dbm` depends on the device under test and is
/// determined empirically. A typical value for a good connector is around
/// -200 to -250 dBm referenced to the fundamental intercept.
///
/// For a quick approximation with two equal-power carriers:
///     P_IM(dBm) = pim_coefficient_dbm + order * P_carrier(dBm)
///
/// This model follows the classic power-law relationship where the IM product
/// power increases by `order` dB for each 1 dB increase in carrier power.
pub fn estimate_pim_power(
    carrier_power_dbm: f64,
    order: usize,
    pim_coefficient_dbm: f64,
) -> f64 {
    pim_coefficient_dbm + (order as f64) * carrier_power_dbm
}

/// Convert a round-trip delay (in nanoseconds) to a one-way distance in metres.
///
/// Used for PIM distance-to-fault measurements where a swept or pulsed signal
/// identifies the location of a PIM source on a transmission line or antenna
/// system.
///
///     distance = (delay_ns * c) / (2 * 1e9)
///
/// This assumes free-space propagation velocity. For coaxial cable, multiply
/// the result by the cable's velocity factor (typically 0.81-0.88).
pub fn distance_to_pim_source(delay_ns: f64) -> f64 {
    (delay_ns * SPEED_OF_LIGHT_M_S) / (2.0 * 1e9)
}

/// Return the uplink (receive at base station) frequency range for common
/// LTE / 5G-NR FDD bands, in MHz.
///
/// Returns `Some((low_mhz, high_mhz))` for the *uplink* (UE transmit,
/// base station receive) band, since PIM from downlink carriers falling
/// into the uplink band is the primary concern.
///
/// Only a representative subset of bands is included.
pub fn lte_band_rx_range(band: u32) -> Option<(f64, f64)> {
    match band {
        // Band : UL low  - UL high  (MHz)
        1 => Some((1920.0, 1980.0)),
        2 => Some((1850.0, 1910.0)),
        3 => Some((1710.0, 1785.0)),
        4 => Some((1710.0, 1755.0)),
        5 => Some((824.0, 849.0)),
        7 => Some((2500.0, 2570.0)),
        8 => Some((880.0, 915.0)),
        12 => Some((699.0, 716.0)),
        13 => Some((777.0, 787.0)),
        14 => Some((788.0, 798.0)),
        17 => Some((704.0, 716.0)),
        20 => Some((832.0, 862.0)),
        25 => Some((1850.0, 1915.0)),
        26 => Some((814.0, 849.0)),
        28 => Some((703.0, 748.0)),
        30 => Some((2305.0, 2315.0)),
        41 => Some((2496.0, 2690.0)), // TDD - listed for completeness
        66 => Some((1710.0, 1780.0)),
        71 => Some((663.0, 698.0)),
        _ => None,
    }
}

/// Generate a two-tone test signal as a vector of (I, Q) sample pairs.
///
/// Produces a baseband-equivalent IQ signal containing two tones at the
/// specified frequencies, each at the given power level.
///
/// # Arguments
///
/// * `f1_mhz` - First tone frequency in MHz.
/// * `f2_mhz` - Second tone frequency in MHz.
/// * `power_dbm` - Power per tone in dBm (into 50 ohms).
/// * `fs_mhz` - Sample rate in MHz.
/// * `duration_us` - Signal duration in microseconds.
///
/// # Returns
///
/// Vector of (I, Q) sample pairs representing the composite two-tone signal.
pub fn generate_two_tone_signal(
    f1_mhz: f64,
    f2_mhz: f64,
    power_dbm: f64,
    fs_mhz: f64,
    duration_us: f64,
) -> Vec<(f64, f64)> {
    let num_samples = (fs_mhz * duration_us).round() as usize;
    if num_samples == 0 {
        return Vec::new();
    }

    // Convert dBm to voltage amplitude (into 50 ohms).
    // P = V^2 / (2 * R)  =>  V = sqrt(2 * R * P_watts)
    // P_watts = 10^((power_dbm - 30) / 10)
    let p_watts = dbm_to_watts(power_dbm);
    let amplitude = (2.0 * 50.0 * p_watts).sqrt();

    let dt = 1.0 / (fs_mhz * 1e6); // seconds per sample
    let omega1 = 2.0 * PI * f1_mhz * 1e6;
    let omega2 = 2.0 * PI * f2_mhz * 1e6;

    let mut signal = Vec::with_capacity(num_samples);

    for k in 0..num_samples {
        let t = k as f64 * dt;
        let i = amplitude * (omega1 * t).cos() + amplitude * (omega2 * t).cos();
        let q = amplitude * (omega1 * t).sin() + amplitude * (omega2 * t).sin();
        signal.push((i, q));
    }

    signal
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Convert dBm to watts.
fn dbm_to_watts(dbm: f64) -> f64 {
    10.0_f64.powf((dbm - 30.0) / 10.0)
}

/// Convert watts to dBm.
#[allow(dead_code)]
fn watts_to_dbm(watts: f64) -> f64 {
    10.0 * watts.log10() + 30.0
}

/// Convert linear ratio to dB.
#[allow(dead_code)]
fn linear_to_db(linear: f64) -> f64 {
    10.0 * linear.log10()
}

/// Convert dB to linear ratio.
#[allow(dead_code)]
fn db_to_linear(db: f64) -> f64 {
    10.0_f64.powf(db / 10.0)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-9;
    const FREQ_EPS: f64 = 1e-6;

    // -----------------------------------------------------------------------
    // IM frequency calculation
    // -----------------------------------------------------------------------

    #[test]
    fn test_im_frequency_third_order_lower() {
        // 2*f1 - f2
        let freq = compute_im_frequency(1930.0, 1990.0, 2, -1);
        assert!((freq - 1870.0).abs() < FREQ_EPS, "got {freq}");
    }

    #[test]
    fn test_im_frequency_third_order_upper() {
        // 2*f2 - f1
        let freq = compute_im_frequency(1930.0, 1990.0, -1, 2);
        assert!((freq - 2050.0).abs() < FREQ_EPS, "got {freq}");
    }

    #[test]
    fn test_im_frequency_fifth_order() {
        // 3*f1 - 2*f2
        let freq = compute_im_frequency(1930.0, 1990.0, 3, -2);
        let expected = 3.0 * 1930.0 - 2.0 * 1990.0; // 1810
        assert!((freq - expected).abs() < FREQ_EPS, "got {freq}");
    }

    #[test]
    fn test_im_frequency_sum_products() {
        // f1 + f2
        let freq = compute_im_frequency(100.0, 200.0, 1, 1);
        assert!((freq - 300.0).abs() < FREQ_EPS);
    }

    #[test]
    fn test_im_frequency_negative_result() {
        // -2*f1 + f2 when f2 < 2*f1 can be negative
        let freq = compute_im_frequency(1000.0, 500.0, -2, 1);
        assert!(freq < 0.0, "expected negative, got {freq}");
    }

    // -----------------------------------------------------------------------
    // Order computation
    // -----------------------------------------------------------------------

    #[test]
    fn test_order_third() {
        assert_eq!(compute_im_order(2, -1), 3);
        assert_eq!(compute_im_order(-1, 2), 3);
    }

    #[test]
    fn test_order_fifth() {
        assert_eq!(compute_im_order(3, -2), 5);
        assert_eq!(compute_im_order(-2, 3), 5);
    }

    #[test]
    fn test_order_seventh() {
        assert_eq!(compute_im_order(4, -3), 7);
    }

    #[test]
    fn test_order_second() {
        assert_eq!(compute_im_order(1, 1), 2);
        assert_eq!(compute_im_order(1, -1), 2);
    }

    #[test]
    fn test_order_zero_coefficients() {
        assert_eq!(compute_im_order(0, 0), 0);
        assert_eq!(compute_im_order(3, 0), 3);
        assert_eq!(compute_im_order(0, -5), 5);
    }

    // -----------------------------------------------------------------------
    // Product enumeration
    // -----------------------------------------------------------------------

    #[test]
    fn test_enumerate_pim_products_third_order() {
        let products = enumerate_pim_products(1930.0, 1990.0, 3);
        // Should contain the classic IM3 pair: 2f1-f2 and 2f2-f1
        let has_lower = products
            .iter()
            .any(|p| p.order == 3 && (p.frequency_mhz - 1870.0).abs() < FREQ_EPS);
        let has_upper = products
            .iter()
            .any(|p| p.order == 3 && (p.frequency_mhz - 2050.0).abs() < FREQ_EPS);
        assert!(has_lower, "missing 2f1-f2 at 1870 MHz");
        assert!(has_upper, "missing 2f2-f1 at 2050 MHz");
    }

    #[test]
    fn test_enumerate_pim_products_no_fundamentals() {
        let products = enumerate_pim_products(900.0, 1000.0, 5);
        // No product should be order 1 (pure fundamental)
        assert!(
            products.iter().all(|p| p.order >= 2),
            "fundamentals should be excluded"
        );
    }

    #[test]
    fn test_enumerate_pim_products_positive_freq_only() {
        let products = enumerate_pim_products(100.0, 200.0, 7);
        assert!(
            products.iter().all(|p| p.frequency_mhz > 0.0),
            "all products should have positive frequency"
        );
    }

    #[test]
    fn test_enumerate_pim_products_sorted() {
        let products = enumerate_pim_products(850.0, 900.0, 5);
        for window in products.windows(2) {
            assert!(
                window[0].order <= window[1].order,
                "products should be sorted by order"
            );
            if window[0].order == window[1].order {
                assert!(
                    window[0].frequency_mhz <= window[1].frequency_mhz,
                    "within same order, products should be sorted by frequency"
                );
            }
        }
    }

    #[test]
    fn test_enumerate_pim_products_max_order_respected() {
        let products = enumerate_pim_products(900.0, 950.0, 5);
        assert!(
            products.iter().all(|p| p.order <= 5),
            "no product should exceed max_order"
        );
    }

    // -----------------------------------------------------------------------
    // In-band check
    // -----------------------------------------------------------------------

    #[test]
    fn test_pim_in_band_inside() {
        assert!(pim_in_band(1950.0, 1920.0, 1980.0));
    }

    #[test]
    fn test_pim_in_band_at_edges() {
        assert!(pim_in_band(1920.0, 1920.0, 1980.0));
        assert!(pim_in_band(1980.0, 1920.0, 1980.0));
    }

    #[test]
    fn test_pim_in_band_outside() {
        assert!(!pim_in_band(1919.9, 1920.0, 1980.0));
        assert!(!pim_in_band(1980.1, 1920.0, 1980.0));
    }

    // -----------------------------------------------------------------------
    // IEC 62037 limits
    // -----------------------------------------------------------------------

    #[test]
    fn test_iec62037_limit_third_order() {
        assert!((iec62037_limit_dbc(3) - (-150.0)).abs() < EPSILON);
    }

    #[test]
    fn test_iec62037_limit_fifth_order() {
        assert!((iec62037_limit_dbc(5) - (-160.0)).abs() < EPSILON);
    }

    #[test]
    fn test_iec62037_limit_seventh_order() {
        assert!((iec62037_limit_dbc(7) - (-165.0)).abs() < EPSILON);
    }

    #[test]
    fn test_iec62037_limit_ninth_order() {
        assert!((iec62037_limit_dbc(9) - (-170.0)).abs() < EPSILON);
    }

    #[test]
    fn test_iec62037_limit_higher_order() {
        assert!((iec62037_limit_dbc(11) - (-175.0)).abs() < EPSILON);
        assert!((iec62037_limit_dbc(13) - (-175.0)).abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // PIM power estimation
    // -----------------------------------------------------------------------

    #[test]
    fn test_estimate_pim_power_third_order() {
        // P_IM = coeff + 3 * 43 = -230 + 129 = -101 dBm
        let p = estimate_pim_power(43.0, 3, -230.0);
        assert!((p - (-101.0)).abs() < EPSILON, "got {p}");
    }

    #[test]
    fn test_estimate_pim_power_fifth_order() {
        let p = estimate_pim_power(43.0, 5, -350.0);
        let expected = -350.0 + 5.0 * 43.0; // -350 + 215 = -135
        assert!((p - expected).abs() < EPSILON, "got {p}");
    }

    #[test]
    fn test_estimate_pim_power_scales_with_carrier() {
        let p1 = estimate_pim_power(40.0, 3, -230.0);
        let p2 = estimate_pim_power(43.0, 3, -230.0);
        // 3 dB increase in carrier should yield 3*3 = 9 dB increase in IM3
        assert!(
            ((p2 - p1) - 9.0).abs() < EPSILON,
            "IM3 should scale 3:1 with carrier power"
        );
    }

    // -----------------------------------------------------------------------
    // Distance from delay
    // -----------------------------------------------------------------------

    #[test]
    fn test_distance_to_pim_source_zero_delay() {
        assert!((distance_to_pim_source(0.0)).abs() < EPSILON);
    }

    #[test]
    fn test_distance_to_pim_source_known_distance() {
        // 10 ns round-trip => 10e-9 * c / 2 ~ 1.499 m
        let d = distance_to_pim_source(10.0);
        let expected = 10.0e-9 * SPEED_OF_LIGHT_M_S / 2.0;
        assert!((d - expected).abs() < 1e-3, "got {d}, expected {expected}");
    }

    #[test]
    fn test_distance_to_pim_source_large_delay() {
        // 1000 ns => ~150 m
        let d = distance_to_pim_source(1000.0);
        assert!((d - 149.896229).abs() < 0.001, "got {d}");
    }

    // -----------------------------------------------------------------------
    // LTE bands
    // -----------------------------------------------------------------------

    #[test]
    fn test_lte_band_1_rx_range() {
        let (low, high) = lte_band_rx_range(1).expect("Band 1 should be defined");
        assert!((low - 1920.0).abs() < FREQ_EPS);
        assert!((high - 1980.0).abs() < FREQ_EPS);
    }

    #[test]
    fn test_lte_band_7_rx_range() {
        let (low, high) = lte_band_rx_range(7).expect("Band 7 should be defined");
        assert!((low - 2500.0).abs() < FREQ_EPS);
        assert!((high - 2570.0).abs() < FREQ_EPS);
    }

    #[test]
    fn test_lte_band_unknown() {
        assert!(lte_band_rx_range(99).is_none());
    }

    #[test]
    fn test_lte_band_71_rx_range() {
        let (low, high) = lte_band_rx_range(71).expect("Band 71 should be defined");
        assert!((low - 663.0).abs() < FREQ_EPS);
        assert!((high - 698.0).abs() < FREQ_EPS);
    }

    // -----------------------------------------------------------------------
    // Two-tone signal generation
    // -----------------------------------------------------------------------

    #[test]
    fn test_two_tone_signal_length() {
        let sig = generate_two_tone_signal(1.0, 2.0, 0.0, 10.0, 100.0);
        // 10 MHz * 100 us = 1000 samples
        assert_eq!(sig.len(), 1000);
    }

    #[test]
    fn test_two_tone_signal_not_all_zero() {
        let sig = generate_two_tone_signal(1.0, 2.0, 10.0, 20.0, 50.0);
        let max_i = sig.iter().map(|(i, _)| i.abs()).fold(0.0_f64, f64::max);
        assert!(max_i > 0.0, "signal should not be all zeros");
    }

    #[test]
    fn test_two_tone_signal_zero_duration() {
        let sig = generate_two_tone_signal(1.0, 2.0, 10.0, 20.0, 0.0);
        assert!(sig.is_empty());
    }

    #[test]
    fn test_two_tone_signal_power_scaling() {
        // Higher power => larger amplitude
        let sig_low = generate_two_tone_signal(1.0, 2.0, 0.0, 20.0, 10.0);
        let sig_high = generate_two_tone_signal(1.0, 2.0, 20.0, 20.0, 10.0);
        let rms_low: f64 = (sig_low.iter().map(|(i, _)| i * i).sum::<f64>()
            / sig_low.len() as f64)
            .sqrt();
        let rms_high: f64 = (sig_high.iter().map(|(i, _)| i * i).sum::<f64>()
            / sig_high.len() as f64)
            .sqrt();
        assert!(
            rms_high > rms_low * 5.0,
            "20 dBm signal should be much larger than 0 dBm"
        );
    }

    // -----------------------------------------------------------------------
    // Analyzer
    // -----------------------------------------------------------------------

    #[test]
    fn test_analyzer_clean_spectrum_passes() {
        let config = PimConfig {
            carrier_freq_1_mhz: 1930.0,
            carrier_freq_2_mhz: 1990.0,
            carrier_power_dbm: 43.0,
            noise_floor_dbm: -130.0,
        };
        let analyzer = PimAnalyzer::new(config);

        // Build a flat noise-floor spectrum from 1800 to 2100 MHz
        let n = 3001;
        let freq_axis: Vec<f64> = (0..n).map(|i| 1800.0 + (i as f64) * 0.1).collect();
        let spectrum: Vec<f64> = vec![-130.0; n];

        let result = analyzer.analyze(&spectrum, &freq_axis);
        assert!(result.pass, "clean spectrum should pass");
        assert!(result.products.is_empty(), "no products above noise floor");
    }

    #[test]
    fn test_analyzer_detects_im3_product() {
        let config = PimConfig {
            carrier_freq_1_mhz: 1930.0,
            carrier_freq_2_mhz: 1990.0,
            carrier_power_dbm: 43.0,
            noise_floor_dbm: -130.0,
        };
        let analyzer = PimAnalyzer::new(config);

        // IM3 lower = 2*1930 - 1990 = 1870 MHz
        let n = 3001;
        let freq_axis: Vec<f64> = (0..n).map(|i| 1800.0 + (i as f64) * 0.1).collect();
        let mut spectrum: Vec<f64> = vec![-140.0; n];

        // Inject a PIM product at 1870 MHz (bin index = (1870-1800)/0.1 = 700)
        spectrum[700] = -100.0; // -100 dBm, which is -143 dBc (above -150 dBc limit)

        let result = analyzer.analyze(&spectrum, &freq_axis);

        // Should detect at least one product
        assert!(
            !result.products.is_empty(),
            "should detect the injected PIM product"
        );

        let im3 = result
            .products
            .iter()
            .find(|p| (p.frequency_mhz - 1870.0).abs() < 1.0 && p.order == 3);
        assert!(im3.is_some(), "should find IM3 product at 1870 MHz");
        assert!(
            (im3.unwrap().power_dbm - (-100.0)).abs() < EPSILON,
            "power should be -100 dBm"
        );
    }

    #[test]
    fn test_analyzer_fail_on_excessive_pim() {
        let config = PimConfig {
            carrier_freq_1_mhz: 1930.0,
            carrier_freq_2_mhz: 1990.0,
            carrier_power_dbm: 43.0,
            noise_floor_dbm: -130.0,
        };
        let analyzer = PimAnalyzer::new(config);

        let n = 3001;
        let freq_axis: Vec<f64> = (0..n).map(|i| 1800.0 + (i as f64) * 0.1).collect();
        let mut spectrum: Vec<f64> = vec![-140.0; n];

        // Inject a very strong IM3 at 1870 MHz: -80 dBm = -123 dBc
        // IEC 62037 IM3 limit is -150 dBc, so this should fail hard
        spectrum[700] = -80.0;

        let result = analyzer.analyze(&spectrum, &freq_axis);
        assert!(!result.pass, "excessive PIM should fail IEC 62037");
        assert!(result.pim_dbc > -150.0, "PIM level should exceed the limit");
    }

    // -----------------------------------------------------------------------
    // PimConfig default
    // -----------------------------------------------------------------------

    #[test]
    fn test_pim_config_default() {
        let config = PimConfig::default();
        assert!((config.carrier_power_dbm - 43.0).abs() < EPSILON);
        assert!((config.noise_floor_dbm - (-130.0)).abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_im_frequency_zero_coefficients() {
        assert!((compute_im_frequency(100.0, 200.0, 0, 0)).abs() < FREQ_EPS);
    }

    #[test]
    fn test_enumerate_pim_products_order_two() {
        let products = enumerate_pim_products(100.0, 200.0, 2);
        // Should have some second-order products
        assert!(
            products.iter().any(|p| p.order == 2),
            "should include second-order products"
        );
        // f1 + f2 = 300 should be present
        assert!(
            products
                .iter()
                .any(|p| p.order == 2 && (p.frequency_mhz - 300.0).abs() < FREQ_EPS),
            "should include f1+f2 = 300 MHz"
        );
    }

    #[test]
    fn test_distance_to_pim_source_negative_delay() {
        // Negative delay is unphysical but should not panic
        let d = distance_to_pim_source(-5.0);
        assert!(d < 0.0, "negative delay yields negative distance");
    }

    #[test]
    fn test_pim_in_band_degenerate_range() {
        // rx_low == rx_high (single point)
        assert!(pim_in_band(100.0, 100.0, 100.0));
        assert!(!pim_in_band(100.1, 100.0, 100.0));
    }

    #[test]
    fn test_iec62037_limit_even_orders() {
        // Even orders get the lenient -140 dBc default
        assert!((iec62037_limit_dbc(2) - (-140.0)).abs() < EPSILON);
        assert!((iec62037_limit_dbc(4) - (-140.0)).abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Helper function tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_dbm_to_watts() {
        // 30 dBm = 1 W
        assert!((dbm_to_watts(30.0) - 1.0).abs() < 1e-9);
        // 0 dBm = 1 mW
        assert!((dbm_to_watts(0.0) - 0.001).abs() < 1e-9);
        // 43 dBm ~ 20 W
        assert!((dbm_to_watts(43.0) - 19.953).abs() < 0.01);
    }

    #[test]
    fn test_watts_to_dbm_roundtrip() {
        for &dbm in &[-10.0, 0.0, 10.0, 30.0, 43.0] {
            let w = dbm_to_watts(dbm);
            let back = watts_to_dbm(w);
            assert!(
                (back - dbm).abs() < 1e-9,
                "roundtrip failed for {dbm} dBm"
            );
        }
    }

    // -----------------------------------------------------------------------
    // Band 3 PIM scenario (common real-world case)
    // -----------------------------------------------------------------------

    #[test]
    fn test_band3_im3_falls_in_uplink() {
        // Band 3 DL: 1805-1880 MHz, UL: 1710-1785 MHz
        // Two DL carriers: f1=1805, f2=1880
        // IM3 lower: 2*1805 - 1880 = 1730 MHz => inside UL band
        let im3_freq = compute_im_frequency(1805.0, 1880.0, 2, -1);
        assert!(
            (im3_freq - 1730.0).abs() < FREQ_EPS,
            "IM3 should be at 1730 MHz"
        );
        let (rx_low, rx_high) = lte_band_rx_range(3).unwrap();
        assert!(
            pim_in_band(im3_freq, rx_low, rx_high),
            "IM3 at 1730 MHz should fall in Band 3 UL (1710-1785)"
        );
    }
}

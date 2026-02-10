//! RF Propagation Loss Models for link budget analysis and coverage planning.
//!
//! This module implements several widely-used RF propagation models including
//! Free-Space Path Loss (FSPL), Okumura-Hata, COST-231, Two-Ray ground
//! reflection, Log-Distance, and ITU-R P.1238 indoor models.
//!
//! # Example
//!
//! ```
//! use r4w_core::rf_propagation_model::{
//!     PropagationModel, Environment, PropagationResult,
//!     free_space_path_loss, hata_model, coverage_radius, received_power,
//! };
//!
//! // Free-space path loss at 1 km, 900 MHz
//! let result = free_space_path_loss(1000.0, 900e6);
//! assert!((result.path_loss_db - 91.53).abs() < 0.1);
//!
//! // Okumura-Hata urban model
//! let result = hata_model(5000.0, 900e6, 30.0, 1.5, Environment::Urban);
//! assert!(result.path_loss_db > 100.0);
//!
//! // Coverage radius for a given TX power and receiver sensitivity
//! let radius = coverage_radius(
//!     &PropagationModel::FreeSpace,
//!     900e6,
//!     30.0,   // TX power dBm
//!     -100.0, // RX sensitivity dBm
//!     0.0,    // TX antenna gain dBi
//!     0.0,    // RX antenna gain dBi
//! );
//! assert!(radius.is_some());
//! assert!(radius.unwrap() > 1000.0);
//!
//! // Received power calculation
//! let rx_power = received_power(20.0, 91.5, 2.0, 0.0);
//! assert!((rx_power - (-69.5)).abs() < 0.01);
//! ```

use std::f64::consts::PI;

/// Speed of light in m/s.
const C: f64 = 299_792_458.0;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Propagation model selector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PropagationModel {
    /// Free-space path loss (Friis).
    FreeSpace,
    /// Okumura-Hata model (150-1500 MHz).
    Hata(Environment),
    /// COST-231 Hata extension (1500-2000 MHz).
    Cost231(Environment),
    /// ITU-R P.1238 indoor model.
    ItuRP1238 {
        /// Number of floors penetrated.
        floors: u32,
    },
    /// Two-ray ground reflection model.
    TwoRay {
        /// TX antenna height in metres.
        tx_height_m: f64,
        /// RX antenna height in metres.
        rx_height_m: f64,
    },
    /// Log-distance path loss model.
    LogDistance {
        /// Path-loss exponent.
        n: f64,
        /// Reference distance in metres.
        d0: f64,
        /// Shadow-fading standard deviation in dB (set to 0.0 for deterministic).
        sigma_db: f64,
    },
}

/// Propagation environment type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Environment {
    Urban,
    Suburban,
    Rural,
    OpenArea,
    Indoor,
}

/// Result of a propagation-loss calculation.
#[derive(Debug, Clone, Copy)]
pub struct PropagationResult {
    /// Predicted path loss in dB (positive value).
    pub path_loss_db: f64,
    /// Distance used for the calculation in metres.
    pub distance_m: f64,
    /// Frequency used for the calculation in Hz.
    pub frequency_hz: f64,
}

// ---------------------------------------------------------------------------
// Core model functions
// ---------------------------------------------------------------------------

/// Free-space path loss (Friis equation).
///
/// FSPL = 20 * log10(4 * pi * d * f / c)
///
/// # Arguments
/// * `distance_m` - Distance in metres (must be > 0).
/// * `frequency_hz` - Carrier frequency in Hz (must be > 0).
///
/// # Panics
/// Panics if `distance_m` or `frequency_hz` is not positive.
pub fn free_space_path_loss(distance_m: f64, frequency_hz: f64) -> PropagationResult {
    assert!(distance_m > 0.0, "distance must be positive");
    assert!(frequency_hz > 0.0, "frequency must be positive");

    let path_loss_db = 20.0 * (4.0 * PI * distance_m * frequency_hz / C).log10();

    PropagationResult {
        path_loss_db,
        distance_m,
        frequency_hz,
    }
}

/// Okumura-Hata model for macro-cell path-loss prediction.
///
/// Valid frequency range: 150-1500 MHz.
/// Valid distance range: 1-20 km.
/// Valid base-station height: 30-200 m.
/// Valid mobile height: 1-10 m.
///
/// # Arguments
/// * `distance_m` - Distance in metres.
/// * `frequency_hz` - Carrier frequency in Hz.
/// * `h_bs_m` - Base-station antenna height in metres.
/// * `h_ms_m` - Mobile-station antenna height in metres.
/// * `env` - Propagation environment.
pub fn hata_model(
    distance_m: f64,
    frequency_hz: f64,
    h_bs_m: f64,
    h_ms_m: f64,
    env: Environment,
) -> PropagationResult {
    let f_mhz = frequency_hz / 1e6;
    let d_km = distance_m / 1000.0;

    assert!(
        (150.0..=1500.0).contains(&f_mhz),
        "Hata model requires 150-1500 MHz, got {} MHz",
        f_mhz
    );
    assert!(d_km > 0.0, "distance must be positive");

    // Small-city mobile antenna correction factor (default)
    let a_hms = (1.1 * f_mhz.log10() - 0.7) * h_ms_m
        - (1.56 * f_mhz.log10() - 0.8);

    // Urban path loss (base formula)
    let l_urban = 69.55
        + 26.16 * f_mhz.log10()
        - 13.82 * h_bs_m.log10()
        - a_hms
        + (44.9 - 6.55 * h_bs_m.log10()) * d_km.log10();

    let path_loss_db = match env {
        Environment::Urban | Environment::Indoor => l_urban,
        Environment::Suburban => {
            l_urban - 2.0 * (f_mhz / 28.0).log10().powi(2) - 5.4
        }
        Environment::Rural | Environment::OpenArea => {
            l_urban - 4.78 * f_mhz.log10().powi(2) + 18.33 * f_mhz.log10() - 40.94
        }
    };

    PropagationResult {
        path_loss_db,
        distance_m,
        frequency_hz,
    }
}

/// COST-231 Hata extension for 1500-2000 MHz.
///
/// Extends the Okumura-Hata model to higher frequencies used by PCS/DCS.
///
/// # Arguments
/// Same as [`hata_model`], but valid from 1500-2000 MHz.
pub fn cost231_model(
    distance_m: f64,
    frequency_hz: f64,
    h_bs_m: f64,
    h_ms_m: f64,
    env: Environment,
) -> PropagationResult {
    let f_mhz = frequency_hz / 1e6;
    let d_km = distance_m / 1000.0;

    assert!(
        (1500.0..=2000.0).contains(&f_mhz),
        "COST-231 model requires 1500-2000 MHz, got {} MHz",
        f_mhz
    );
    assert!(d_km > 0.0, "distance must be positive");

    // Mobile antenna correction (small/medium city)
    let a_hms = (1.1 * f_mhz.log10() - 0.7) * h_ms_m
        - (1.56 * f_mhz.log10() - 0.8);

    // Metropolitan correction factor
    let c_m = match env {
        Environment::Urban => 3.0, // metropolitan / dense urban
        _ => 0.0,                  // suburban, rural, etc.
    };

    let path_loss_db = 46.3
        + 33.9 * f_mhz.log10()
        - 13.82 * h_bs_m.log10()
        - a_hms
        + (44.9 - 6.55 * h_bs_m.log10()) * d_km.log10()
        + c_m;

    PropagationResult {
        path_loss_db,
        distance_m,
        frequency_hz,
    }
}

/// Two-ray ground-reflection model.
///
/// For short distances (< crossover) the model falls back to FSPL.
/// Beyond the crossover distance the path loss increases at 40 dB/decade.
///
/// # Arguments
/// * `distance_m` - Distance in metres.
/// * `frequency_hz` - Carrier frequency in Hz.
/// * `h_tx_m` - Transmitter antenna height in metres.
/// * `h_rx_m` - Receiver antenna height in metres.
pub fn two_ray_model(
    distance_m: f64,
    frequency_hz: f64,
    h_tx_m: f64,
    h_rx_m: f64,
) -> PropagationResult {
    assert!(distance_m > 0.0, "distance must be positive");
    assert!(frequency_hz > 0.0, "frequency must be positive");
    assert!(h_tx_m > 0.0, "TX height must be positive");
    assert!(h_rx_m > 0.0, "RX height must be positive");

    let lambda = C / frequency_hz;
    // Crossover distance: beyond this the two-ray model dominates
    let d_crossover = 4.0 * h_tx_m * h_rx_m / lambda;

    let path_loss_db = if distance_m < d_crossover {
        // Use FSPL for short range
        20.0 * (4.0 * PI * distance_m * frequency_hz / C).log10()
    } else {
        // Two-ray: PL = 40*log10(d) - 20*log10(ht) - 20*log10(hr)
        40.0 * distance_m.log10() - 20.0 * h_tx_m.log10() - 20.0 * h_rx_m.log10()
    };

    PropagationResult {
        path_loss_db,
        distance_m,
        frequency_hz,
    }
}

/// Log-distance path-loss model.
///
/// PL(d) = PL(d0) + 10 * n * log10(d / d0) + X_sigma
///
/// where PL(d0) is the free-space loss at the reference distance d0.
///
/// # Arguments
/// * `distance_m` - Distance in metres.
/// * `frequency_hz` - Carrier frequency in Hz.
/// * `n` - Path-loss exponent (2 = free space, 2.7-3.5 urban, 4-6 indoor).
/// * `d0_m` - Reference distance in metres (commonly 1 m or 100 m).
/// * `x_sigma_db` - Shadow-fading margin in dB (deterministic; set 0 if none).
pub fn log_distance_model(
    distance_m: f64,
    frequency_hz: f64,
    n: f64,
    d0_m: f64,
    x_sigma_db: f64,
) -> PropagationResult {
    assert!(distance_m > 0.0, "distance must be positive");
    assert!(frequency_hz > 0.0, "frequency must be positive");
    assert!(d0_m > 0.0, "reference distance must be positive");

    // PL at reference distance (free-space)
    let pl_d0 = 20.0 * (4.0 * PI * d0_m * frequency_hz / C).log10();

    let path_loss_db = pl_d0 + 10.0 * n * (distance_m / d0_m).log10() + x_sigma_db;

    PropagationResult {
        path_loss_db,
        distance_m,
        frequency_hz,
    }
}

/// ITU-R P.1238 indoor propagation model.
///
/// L = 20 * log10(f_MHz) + N * log10(d) + Lf(n_floors) - 28
///
/// where N is the distance power-loss coefficient and Lf is the floor
/// penetration loss factor.
///
/// # Arguments
/// * `distance_m` - Distance in metres (> 1 m).
/// * `frequency_hz` - Carrier frequency in Hz.
/// * `floors` - Number of floors between TX and RX (0 = same floor).
pub fn itu_indoor_model(
    distance_m: f64,
    frequency_hz: f64,
    floors: u32,
) -> PropagationResult {
    assert!(distance_m > 0.0, "distance must be positive");
    assert!(frequency_hz > 0.0, "frequency must be positive");

    let f_mhz = frequency_hz / 1e6;

    // Distance power-loss coefficient (office environment typical)
    let n_coeff: f64 = 30.0;

    // Floor penetration loss (approximate: 15 + 4*(n-1) for n >= 1)
    let lf: f64 = if floors == 0 {
        0.0
    } else {
        15.0 + 4.0 * (floors as f64 - 1.0)
    };

    let path_loss_db = 20.0 * f_mhz.log10() + n_coeff * distance_m.log10() + lf - 28.0;

    PropagationResult {
        path_loss_db,
        distance_m,
        frequency_hz,
    }
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Compute path loss using any [`PropagationModel`] variant.
///
/// This is a convenience dispatcher that calls the appropriate model function.
pub fn path_loss(model: &PropagationModel, distance_m: f64, frequency_hz: f64) -> PropagationResult {
    match *model {
        PropagationModel::FreeSpace => free_space_path_loss(distance_m, frequency_hz),
        PropagationModel::Hata(env) => {
            // Default antenna heights for convenience dispatch
            hata_model(distance_m, frequency_hz, 30.0, 1.5, env)
        }
        PropagationModel::Cost231(env) => {
            cost231_model(distance_m, frequency_hz, 30.0, 1.5, env)
        }
        PropagationModel::ItuRP1238 { floors } => {
            itu_indoor_model(distance_m, frequency_hz, floors)
        }
        PropagationModel::TwoRay {
            tx_height_m,
            rx_height_m,
        } => two_ray_model(distance_m, frequency_hz, tx_height_m, rx_height_m),
        PropagationModel::LogDistance { n, d0, sigma_db } => {
            log_distance_model(distance_m, frequency_hz, n, d0, sigma_db)
        }
    }
}

/// Compute received power in dBm.
///
/// P_rx = P_tx - PL + G_tx + G_rx
///
/// # Arguments
/// * `tx_power_dbm` - Transmit power in dBm.
/// * `path_loss_db` - Path loss in dB (positive).
/// * `tx_gain_dbi` - Transmit antenna gain in dBi.
/// * `rx_gain_dbi` - Receive antenna gain in dBi.
pub fn received_power(
    tx_power_dbm: f64,
    path_loss_db: f64,
    tx_gain_dbi: f64,
    rx_gain_dbi: f64,
) -> f64 {
    tx_power_dbm - path_loss_db + tx_gain_dbi + rx_gain_dbi
}

/// Estimate the maximum coverage radius for a given link budget.
///
/// Uses binary search to find the distance at which received power equals
/// the receiver sensitivity threshold.
///
/// # Arguments
/// * `model` - Propagation model to use.
/// * `frequency_hz` - Carrier frequency in Hz.
/// * `tx_power_dbm` - Transmit power in dBm.
/// * `rx_sensitivity_dbm` - Receiver sensitivity in dBm.
/// * `tx_gain_dbi` - Transmit antenna gain in dBi.
/// * `rx_gain_dbi` - Receive antenna gain in dBi.
///
/// # Returns
/// `Some(radius_m)` if a valid radius is found, `None` if the budget is
/// infeasible even at 1 m.
pub fn coverage_radius(
    model: &PropagationModel,
    frequency_hz: f64,
    tx_power_dbm: f64,
    rx_sensitivity_dbm: f64,
    tx_gain_dbi: f64,
    rx_gain_dbi: f64,
) -> Option<f64> {
    let link_margin = tx_power_dbm + tx_gain_dbi + rx_gain_dbi - rx_sensitivity_dbm;

    // Check if even at 1 m the budget is infeasible
    let pl_1m = path_loss(model, 1.0, frequency_hz).path_loss_db;
    if link_margin < pl_1m {
        return None;
    }

    // Binary search between 1 m and 1 000 km
    let mut lo: f64 = 1.0;
    let mut hi: f64 = 1_000_000.0;

    for _ in 0..100 {
        let mid = (lo + hi) / 2.0;
        let pl = path_loss(model, mid, frequency_hz).path_loss_db;
        if pl < link_margin {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    Some((lo + hi) / 2.0)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 0.5; // dB tolerance for model comparisons

    #[test]
    fn test_fspl_1km_900mhz() {
        // Well-known value: FSPL at 1 km, 900 MHz ~ 91.53 dB
        let r = free_space_path_loss(1000.0, 900e6);
        assert!(
            (r.path_loss_db - 91.53).abs() < TOLERANCE,
            "FSPL={:.2} dB, expected ~91.53",
            r.path_loss_db
        );
        assert_eq!(r.distance_m, 1000.0);
        assert_eq!(r.frequency_hz, 900e6);
    }

    #[test]
    fn test_fspl_inverse_square() {
        // Doubling distance should add ~6 dB
        let r1 = free_space_path_loss(1000.0, 2.4e9);
        let r2 = free_space_path_loss(2000.0, 2.4e9);
        let delta = r2.path_loss_db - r1.path_loss_db;
        assert!(
            (delta - 6.02).abs() < 0.1,
            "Expected ~6.02 dB increase, got {:.2}",
            delta
        );
    }

    #[test]
    fn test_fspl_frequency_dependence() {
        // Doubling frequency should add ~6 dB
        let r1 = free_space_path_loss(1000.0, 1e9);
        let r2 = free_space_path_loss(1000.0, 2e9);
        let delta = r2.path_loss_db - r1.path_loss_db;
        assert!(
            (delta - 6.02).abs() < 0.1,
            "Expected ~6.02 dB increase, got {:.2}",
            delta
        );
    }

    #[test]
    #[should_panic(expected = "distance must be positive")]
    fn test_fspl_zero_distance_panics() {
        free_space_path_loss(0.0, 900e6);
    }

    #[test]
    fn test_hata_urban_greater_than_suburban() {
        let urban = hata_model(5000.0, 900e6, 30.0, 1.5, Environment::Urban);
        let suburban = hata_model(5000.0, 900e6, 30.0, 1.5, Environment::Suburban);
        assert!(
            urban.path_loss_db > suburban.path_loss_db,
            "Urban loss ({:.1}) should exceed suburban ({:.1})",
            urban.path_loss_db,
            suburban.path_loss_db
        );
    }

    #[test]
    fn test_hata_suburban_greater_than_rural() {
        let suburban = hata_model(5000.0, 900e6, 30.0, 1.5, Environment::Suburban);
        let rural = hata_model(5000.0, 900e6, 30.0, 1.5, Environment::Rural);
        assert!(
            suburban.path_loss_db > rural.path_loss_db,
            "Suburban loss ({:.1}) should exceed rural ({:.1})",
            suburban.path_loss_db,
            rural.path_loss_db
        );
    }

    #[test]
    #[should_panic(expected = "Hata model requires 150-1500 MHz")]
    fn test_hata_frequency_out_of_range() {
        hata_model(1000.0, 2.4e9, 30.0, 1.5, Environment::Urban);
    }

    #[test]
    fn test_cost231_metropolitan() {
        let r = cost231_model(5000.0, 1800e6, 30.0, 1.5, Environment::Urban);
        // COST-231 at 1800 MHz, 5 km should give a plausible macro-cell loss
        assert!(
            r.path_loss_db > 120.0 && r.path_loss_db < 200.0,
            "COST-231 loss = {:.1} dB, expected 120-200",
            r.path_loss_db
        );
    }

    #[test]
    #[should_panic(expected = "COST-231 model requires 1500-2000 MHz")]
    fn test_cost231_frequency_out_of_range() {
        cost231_model(1000.0, 900e6, 30.0, 1.5, Environment::Urban);
    }

    #[test]
    fn test_two_ray_long_range() {
        // At very long range, two-ray should give ~40 dB/decade slope
        let r1 = two_ray_model(10_000.0, 900e6, 50.0, 2.0);
        let r2 = two_ray_model(100_000.0, 900e6, 50.0, 2.0);
        let slope = (r2.path_loss_db - r1.path_loss_db) / (100_000.0_f64 / 10_000.0).log10();
        assert!(
            (slope - 40.0).abs() < 1.0,
            "Two-ray slope = {:.1} dB/decade, expected ~40",
            slope
        );
    }

    #[test]
    fn test_two_ray_short_range_matches_fspl() {
        // At short range the two-ray model should fall back to FSPL
        let two_ray = two_ray_model(10.0, 900e6, 50.0, 2.0);
        let fspl = free_space_path_loss(10.0, 900e6);
        assert!(
            (two_ray.path_loss_db - fspl.path_loss_db).abs() < 0.01,
            "Short-range two-ray ({:.2}) should match FSPL ({:.2})",
            two_ray.path_loss_db,
            fspl.path_loss_db
        );
    }

    #[test]
    fn test_log_distance_exponent_2_matches_fspl() {
        // With n=2.0, d0=1m, sigma=0 should match FSPL
        let ld = log_distance_model(1000.0, 900e6, 2.0, 1.0, 0.0);
        let fspl = free_space_path_loss(1000.0, 900e6);
        assert!(
            (ld.path_loss_db - fspl.path_loss_db).abs() < 0.01,
            "Log-dist(n=2) = {:.2}, FSPL = {:.2}",
            ld.path_loss_db,
            fspl.path_loss_db
        );
    }

    #[test]
    fn test_log_distance_higher_exponent() {
        // Higher exponent should yield greater loss
        let ld2 = log_distance_model(1000.0, 900e6, 2.0, 1.0, 0.0);
        let ld4 = log_distance_model(1000.0, 900e6, 4.0, 1.0, 0.0);
        assert!(
            ld4.path_loss_db > ld2.path_loss_db,
            "n=4 loss ({:.1}) should exceed n=2 ({:.1})",
            ld4.path_loss_db,
            ld2.path_loss_db
        );
    }

    #[test]
    fn test_itu_indoor_floor_penetration() {
        // More floors should yield more loss
        let r0 = itu_indoor_model(50.0, 2.4e9, 0);
        let r2 = itu_indoor_model(50.0, 2.4e9, 2);
        assert!(
            r2.path_loss_db > r0.path_loss_db,
            "2-floor loss ({:.1}) should exceed same-floor ({:.1})",
            r2.path_loss_db,
            r0.path_loss_db
        );
    }

    #[test]
    fn test_received_power() {
        // Simple arithmetic check: Prx = 30 - 100 + 5 + 3 = -62 dBm
        let prx = received_power(30.0, 100.0, 5.0, 3.0);
        assert!(
            (prx - (-62.0)).abs() < 1e-10,
            "Received power = {}, expected -62",
            prx
        );
    }

    #[test]
    fn test_coverage_radius_returns_some() {
        let radius = coverage_radius(
            &PropagationModel::FreeSpace,
            900e6,
            30.0,
            -100.0,
            0.0,
            0.0,
        );
        assert!(radius.is_some(), "Should find a coverage radius");
        let r = radius.unwrap();
        // 30 dBm TX, -100 dBm sensitivity -> 130 dB budget
        // FSPL of 130 dB at 900 MHz ~ ~85 km
        assert!(r > 10_000.0, "Expected radius > 10 km, got {:.0} m", r);
    }

    #[test]
    fn test_coverage_radius_infeasible() {
        // Very low TX power and poor sensitivity should be infeasible
        let radius = coverage_radius(
            &PropagationModel::FreeSpace,
            900e6,
            -50.0,  // very low TX
            0.0,    // bad sensitivity
            0.0,
            0.0,
        );
        assert!(
            radius.is_none(),
            "Should return None for infeasible link budget"
        );
    }

    #[test]
    fn test_path_loss_dispatcher() {
        // Dispatcher should give same result as direct call
        let direct = free_space_path_loss(500.0, 2.4e9);
        let dispatched = path_loss(&PropagationModel::FreeSpace, 500.0, 2.4e9);
        assert!(
            (direct.path_loss_db - dispatched.path_loss_db).abs() < 1e-10,
            "Dispatcher mismatch: direct={:.2}, dispatched={:.2}",
            direct.path_loss_db,
            dispatched.path_loss_db
        );
    }

    #[test]
    fn test_log_distance_shadow_fading_offset() {
        // Adding shadow fading margin should increase loss by exactly that amount
        let base = log_distance_model(1000.0, 900e6, 3.0, 1.0, 0.0);
        let faded = log_distance_model(1000.0, 900e6, 3.0, 1.0, 8.0);
        assert!(
            (faded.path_loss_db - base.path_loss_db - 8.0).abs() < 1e-10,
            "Shadow fading offset should be exactly 8 dB"
        );
    }

    #[test]
    fn test_hata_increasing_distance() {
        let r1 = hata_model(1000.0, 900e6, 30.0, 1.5, Environment::Urban);
        let r2 = hata_model(10_000.0, 900e6, 30.0, 1.5, Environment::Urban);
        assert!(
            r2.path_loss_db > r1.path_loss_db,
            "Loss should increase with distance"
        );
    }
}

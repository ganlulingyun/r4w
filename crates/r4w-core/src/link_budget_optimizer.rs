//! Link Budget Optimizer — End-to-End Link Parameter Optimization
//!
//! Selects the best combination of modulation, coding rate, and transmit power
//! to maximise throughput while meeting target BER and constraint limits.
//! Uses a library of predefined ModCod options (BPSK 1/2 through 64-QAM 3/4)
//! and evaluates each against the current link conditions.
//!
//! No external crates are required — all calculations use the standard library.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::link_budget_optimizer::{
//!     LinkBudgetOptimizer, LinkParams, Constraints,
//! };
//!
//! let params = LinkParams {
//!     tx_power_dbm: 30.0,
//!     path_loss_db: 100.0,
//!     noise_figure_db: 6.0,
//!     tx_antenna_gain_dbi: 2.0,
//!     rx_antenna_gain_dbi: 2.0,
//!     bandwidth_hz: 1_000_000.0,
//! };
//!
//! let constraints = Constraints {
//!     max_tx_power_dbm: 36.0,
//!     min_ber: 1e-6,
//!     min_throughput_bps: 100_000.0,
//!     max_latency_ms: None,
//! };
//!
//! let optimizer = LinkBudgetOptimizer::new(params, constraints);
//! let result = optimizer.optimize();
//!
//! assert!(result.is_some());
//! let r = result.unwrap();
//! assert!(r.expected_ber <= 1e-6);
//! assert!(r.throughput_bps >= 100_000.0);
//! ```

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Physical-layer link parameters.
#[derive(Debug, Clone)]
pub struct LinkParams {
    /// Transmit power (dBm).
    pub tx_power_dbm: f64,
    /// Total path loss (dB, positive value).
    pub path_loss_db: f64,
    /// Receiver noise figure (dB).
    pub noise_figure_db: f64,
    /// Transmit antenna gain (dBi).
    pub tx_antenna_gain_dbi: f64,
    /// Receive antenna gain (dBi).
    pub rx_antenna_gain_dbi: f64,
    /// Channel bandwidth (Hz).
    pub bandwidth_hz: f64,
}

/// Modulation and coding option.
#[derive(Debug, Clone, PartialEq)]
pub struct ModCodOption {
    /// Human-readable label, e.g. "QPSK 3/4".
    pub name: &'static str,
    /// Modulation family.
    pub modulation: ModulationType,
    /// Code rate as a fraction (e.g. 0.5 for rate 1/2).
    pub coding_rate: f64,
    /// Spectral efficiency (bits/s/Hz) = log2(M) * coding_rate.
    pub spectral_efficiency: f64,
    /// Minimum Eb/N0 (dB) needed to achieve a BER of ~1e-5 with this modcod.
    pub required_snr_db: f64,
}

/// Modulation families.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModulationType {
    Bpsk,
    Qpsk,
    Qam16,
    Qam64,
}

/// Optimization constraints.
#[derive(Debug, Clone)]
pub struct Constraints {
    /// Maximum allowed transmit power (dBm).
    pub max_tx_power_dbm: f64,
    /// Minimum acceptable BER (i.e. the BER must be **at most** this value).
    pub min_ber: f64,
    /// Minimum required throughput (bps).
    pub min_throughput_bps: f64,
    /// Optional maximum latency (ms).  Not enforced by the optimizer today but
    /// carried for future use and for callers to inspect.
    pub max_latency_ms: Option<f64>,
}

/// Result of the optimisation pass.
#[derive(Debug, Clone)]
pub struct OptimizationResult {
    /// The selected modulation-and-coding option.
    pub selected_modcod: ModCodOption,
    /// Transmit power that will be used (dBm).
    pub tx_power_dbm: f64,
    /// Expected BER at the operating point.
    pub expected_ber: f64,
    /// Achievable throughput (bps).
    pub throughput_bps: f64,
    /// Link margin (dB) — how much SNR headroom remains.
    pub margin_db: f64,
}

// ---------------------------------------------------------------------------
// Optimizer
// ---------------------------------------------------------------------------

/// Optimises end-to-end link parameters.
#[derive(Debug, Clone)]
pub struct LinkBudgetOptimizer {
    params: LinkParams,
    constraints: Constraints,
}

impl LinkBudgetOptimizer {
    /// Create a new optimizer with the given link parameters and constraints.
    pub fn new(params: LinkParams, constraints: Constraints) -> Self {
        Self { params, constraints }
    }

    // -- catalogue ----------------------------------------------------------

    /// Return the built-in set of modulation/coding options, ordered from
    /// lowest to highest spectral efficiency.
    pub fn available_modcods() -> Vec<ModCodOption> {
        vec![
            ModCodOption {
                name: "BPSK 1/2",
                modulation: ModulationType::Bpsk,
                coding_rate: 0.5,
                spectral_efficiency: 0.5,   // 1 * 0.5
                required_snr_db: 4.0,
            },
            ModCodOption {
                name: "BPSK 3/4",
                modulation: ModulationType::Bpsk,
                coding_rate: 0.75,
                spectral_efficiency: 0.75,
                required_snr_db: 5.5,
            },
            ModCodOption {
                name: "QPSK 1/2",
                modulation: ModulationType::Qpsk,
                coding_rate: 0.5,
                spectral_efficiency: 1.0,   // 2 * 0.5
                required_snr_db: 6.5,
            },
            ModCodOption {
                name: "QPSK 3/4",
                modulation: ModulationType::Qpsk,
                coding_rate: 0.75,
                spectral_efficiency: 1.5,
                required_snr_db: 8.5,
            },
            ModCodOption {
                name: "16-QAM 1/2",
                modulation: ModulationType::Qam16,
                coding_rate: 0.5,
                spectral_efficiency: 2.0,   // 4 * 0.5
                required_snr_db: 12.0,
            },
            ModCodOption {
                name: "16-QAM 3/4",
                modulation: ModulationType::Qam16,
                coding_rate: 0.75,
                spectral_efficiency: 3.0,
                required_snr_db: 15.0,
            },
            ModCodOption {
                name: "64-QAM 1/2",
                modulation: ModulationType::Qam64,
                coding_rate: 0.5,
                spectral_efficiency: 3.0,   // 6 * 0.5
                required_snr_db: 17.0,
            },
            ModCodOption {
                name: "64-QAM 2/3",
                modulation: ModulationType::Qam64,
                coding_rate: 2.0 / 3.0,
                spectral_efficiency: 4.0,
                required_snr_db: 19.5,
            },
            ModCodOption {
                name: "64-QAM 3/4",
                modulation: ModulationType::Qam64,
                coding_rate: 0.75,
                spectral_efficiency: 4.5,
                required_snr_db: 21.0,
            },
        ]
    }

    // -- core helpers -------------------------------------------------------

    /// Receive SNR (dB) for a given TX power.
    fn receive_snr_db(&self, tx_power_dbm: f64) -> f64 {
        // Thermal noise floor = -174 dBm/Hz + 10*log10(BW) + NF
        let noise_floor_dbm =
            -174.0 + 10.0 * log10(self.params.bandwidth_hz) + self.params.noise_figure_db;
        let rx_power_dbm = tx_power_dbm
            + self.params.tx_antenna_gain_dbi
            + self.params.rx_antenna_gain_dbi
            - self.params.path_loss_db;
        rx_power_dbm - noise_floor_dbm
    }

    /// Approximate BER for a modcod at a given SNR (dB).
    ///
    /// Uses classic uncoded AWGN BER formulae scaled by an approximation for
    /// the coding gain.  The coding gain model is intentionally simple —
    /// `coding_gain = -10*log10(coding_rate)` — which is adequate for
    /// comparative optimisation across modcods.
    fn estimate_ber(modcod: &ModCodOption, snr_db: f64) -> f64 {
        // Effective Eb/N0 accounting for spectral efficiency and coding gain.
        // Eb/N0 = SNR_dB - 10*log10(spectral_efficiency) + coding_gain
        let coding_gain_db = -10.0 * log10(modcod.coding_rate);
        let eb_n0_db = snr_db - 10.0 * log10(modcod.spectral_efficiency) + coding_gain_db;
        let eb_n0_linear = db_to_linear(eb_n0_db);

        match modcod.modulation {
            ModulationType::Bpsk | ModulationType::Qpsk => {
                // BER = Q(sqrt(2*Eb/N0)) = 0.5*erfc(sqrt(Eb/N0))
                0.5 * erfc(eb_n0_linear.sqrt())
            }
            ModulationType::Qam16 => {
                // BER ~ (3/8)*erfc(sqrt(2*Eb/N0 / 5))
                0.375 * erfc((2.0 * eb_n0_linear / 5.0).sqrt())
            }
            ModulationType::Qam64 => {
                // BER ~ (7/24)*erfc(sqrt(Eb/N0 / 7))
                (7.0 / 24.0) * erfc((eb_n0_linear / 7.0).sqrt())
            }
        }
    }

    // -- public API ---------------------------------------------------------

    /// Compute the link margin (dB) for a specific modcod at the current TX
    /// power.  Positive margin means the link closes; negative means it does
    /// not.
    pub fn compute_margin(&self, modcod: &ModCodOption) -> f64 {
        let snr = self.receive_snr_db(self.params.tx_power_dbm);
        snr - modcod.required_snr_db
    }

    /// Given a measured receive SNR (dB), select the highest-throughput modcod
    /// whose required SNR (plus an implementation margin) is met.
    ///
    /// `impl_margin_db` is a safety margin subtracted from the available SNR
    /// before comparison (typically 2-3 dB).
    pub fn adaptive_selection(
        measured_snr_db: f64,
        impl_margin_db: f64,
    ) -> Option<ModCodOption> {
        let effective = measured_snr_db - impl_margin_db;
        let modcods = Self::available_modcods();
        // Walk from highest spectral efficiency downward.
        modcods
            .into_iter()
            .rev()
            .find(|mc| effective >= mc.required_snr_db)
    }

    /// Calculate the minimum TX power (dBm) required to achieve the target BER
    /// with the given modcod.
    pub fn power_control(&self, modcod: &ModCodOption, target_ber: f64) -> f64 {
        // Binary search for the minimum TX power.
        let mut lo = -30.0_f64; // -30 dBm
        let mut hi = self.constraints.max_tx_power_dbm;
        for _ in 0..100 {
            let mid = (lo + hi) / 2.0;
            let snr = self.receive_snr_db(mid);
            let ber = Self::estimate_ber(modcod, snr);
            if ber <= target_ber {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        hi
    }

    /// Run the full optimisation: find the modcod and power level that
    /// maximises throughput while respecting all constraints.
    ///
    /// Returns `None` if no modcod can satisfy the constraints.
    pub fn optimize(&self) -> Option<OptimizationResult> {
        let modcods = Self::available_modcods();
        let mut best: Option<OptimizationResult> = None;

        for mc in &modcods {
            // Throughput for this modcod.
            let throughput = mc.spectral_efficiency * self.params.bandwidth_hz;
            if throughput < self.constraints.min_throughput_bps {
                continue;
            }

            // Minimum power to hit target BER.
            let needed_power = self.power_control(mc, self.constraints.min_ber);
            if needed_power > self.constraints.max_tx_power_dbm {
                continue; // exceeds power budget
            }

            // Use the higher of (needed power, configured TX power) so we
            // don't understate the margin when the configured power already
            // exceeds the minimum.
            let use_power = needed_power.max(
                self.params.tx_power_dbm.min(self.constraints.max_tx_power_dbm),
            );
            let snr = self.receive_snr_db(use_power);
            let ber = Self::estimate_ber(mc, snr);
            let margin = snr - mc.required_snr_db;

            if ber > self.constraints.min_ber {
                continue;
            }

            let candidate = OptimizationResult {
                selected_modcod: mc.clone(),
                tx_power_dbm: use_power,
                expected_ber: ber,
                throughput_bps: throughput,
                margin_db: margin,
            };

            // Prefer highest throughput; break ties on lower power.
            let dominated = match &best {
                Some(b) => {
                    candidate.throughput_bps > b.throughput_bps
                        || (candidate.throughput_bps == b.throughput_bps
                            && candidate.tx_power_dbm < b.tx_power_dbm)
                }
                None => true,
            };
            if dominated {
                best = Some(candidate);
            }
        }

        best
    }
}

// ---------------------------------------------------------------------------
// Maths helpers (std-only)
// ---------------------------------------------------------------------------

/// Base-10 logarithm.
#[inline]
fn log10(x: f64) -> f64 {
    x.log10()
}

/// Convert dB to linear scale.
#[inline]
fn db_to_linear(db: f64) -> f64 {
    10.0_f64.powf(db / 10.0)
}

/// Complementary error function approximation (Abramowitz & Stegun 7.1.26).
///
/// Maximum relative error < 1.5e-7 for x >= 0.
fn erfc(x: f64) -> f64 {
    if x < 0.0 {
        return 2.0 - erfc(-x);
    }
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t
        * (0.254829592
            + t * (-0.284496736
                + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    poly * (-x * x).exp()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: default link params for a reasonable UHF link.
    fn default_params() -> LinkParams {
        LinkParams {
            tx_power_dbm: 30.0,
            path_loss_db: 100.0,
            noise_figure_db: 6.0,
            tx_antenna_gain_dbi: 2.0,
            rx_antenna_gain_dbi: 2.0,
            bandwidth_hz: 1_000_000.0,
        }
    }

    fn default_constraints() -> Constraints {
        Constraints {
            max_tx_power_dbm: 36.0,
            min_ber: 1e-5,
            min_throughput_bps: 0.0,
            max_latency_ms: None,
        }
    }

    #[test]
    fn test_available_modcods_count() {
        let mcs = LinkBudgetOptimizer::available_modcods();
        assert!(mcs.len() >= 9, "expected at least 9 modcods, got {}", mcs.len());
    }

    #[test]
    fn test_modcods_sorted_by_spectral_efficiency() {
        let mcs = LinkBudgetOptimizer::available_modcods();
        for w in mcs.windows(2) {
            assert!(
                w[0].spectral_efficiency <= w[1].spectral_efficiency,
                "{} ({}) should be <= {} ({})",
                w[0].name,
                w[0].spectral_efficiency,
                w[1].name,
                w[1].spectral_efficiency
            );
        }
    }

    #[test]
    fn test_receive_snr_positive_margin() {
        let opt = LinkBudgetOptimizer::new(default_params(), default_constraints());
        let snr = opt.receive_snr_db(30.0);
        // 30 + 2 + 2 - 100 = -66 dBm received.
        // Noise floor = -174 + 60 + 6 = -108 dBm.
        // SNR = -66 - (-108) = 42 dB.
        assert!((snr - 42.0).abs() < 0.1, "expected ~42 dB, got {snr}");
    }

    #[test]
    fn test_estimate_ber_bpsk_high_snr() {
        let mc = &LinkBudgetOptimizer::available_modcods()[0]; // BPSK 1/2
        let ber = LinkBudgetOptimizer::estimate_ber(mc, 20.0);
        assert!(ber < 1e-10, "BPSK at 20 dB SNR should have negligible BER, got {ber}");
    }

    #[test]
    fn test_estimate_ber_decreases_with_snr() {
        let mc = &LinkBudgetOptimizer::available_modcods()[2]; // QPSK 1/2
        let ber_low = LinkBudgetOptimizer::estimate_ber(mc, 5.0);
        let ber_high = LinkBudgetOptimizer::estimate_ber(mc, 15.0);
        assert!(
            ber_high < ber_low,
            "BER should decrease with increasing SNR: {ber_low} vs {ber_high}"
        );
    }

    #[test]
    fn test_compute_margin_positive() {
        let opt = LinkBudgetOptimizer::new(default_params(), default_constraints());
        let mc = &LinkBudgetOptimizer::available_modcods()[0]; // BPSK 1/2, req 4 dB
        let margin = opt.compute_margin(mc);
        // SNR ~ 42, required 4 -> margin ~ 38
        assert!(margin > 30.0, "expected large positive margin, got {margin}");
    }

    #[test]
    fn test_compute_margin_negative_for_weak_link() {
        let mut params = default_params();
        params.path_loss_db = 160.0; // very weak
        let opt = LinkBudgetOptimizer::new(params, default_constraints());
        let mc = &LinkBudgetOptimizer::available_modcods()[8]; // 64-QAM 3/4
        let margin = opt.compute_margin(mc);
        assert!(margin < 0.0, "expected negative margin for weak link, got {margin}");
    }

    #[test]
    fn test_adaptive_selection_picks_highest_feasible() {
        // With 25 dB measured SNR and 2 dB margin -> effective 23 dB.
        // 64-QAM 3/4 requires 21 dB -> should be selected.
        let mc = LinkBudgetOptimizer::adaptive_selection(25.0, 2.0);
        assert!(mc.is_some());
        let mc = mc.unwrap();
        assert_eq!(mc.name, "64-QAM 3/4");
    }

    #[test]
    fn test_adaptive_selection_none_when_snr_too_low() {
        let mc = LinkBudgetOptimizer::adaptive_selection(-5.0, 2.0);
        assert!(mc.is_none(), "should return None when SNR is too low");
    }

    #[test]
    fn test_power_control_returns_feasible_power() {
        let opt = LinkBudgetOptimizer::new(default_params(), default_constraints());
        let mc = &LinkBudgetOptimizer::available_modcods()[0]; // BPSK 1/2
        let pwr = opt.power_control(mc, 1e-5);
        // For a generous link, minimum power should be well below max.
        assert!(
            pwr < opt.constraints.max_tx_power_dbm,
            "expected power < max, got {pwr}"
        );
        // Verify the BER at that power is indeed acceptable.
        let snr = opt.receive_snr_db(pwr);
        let ber = LinkBudgetOptimizer::estimate_ber(mc, snr);
        assert!(ber <= 1e-5 + 1e-12, "BER at computed power should meet target, got {ber}");
    }

    #[test]
    fn test_optimize_selects_highest_throughput() {
        let opt = LinkBudgetOptimizer::new(default_params(), default_constraints());
        let result = opt.optimize();
        assert!(result.is_some(), "optimisation should succeed for a good link");
        let r = result.unwrap();
        // With 42 dB SNR and no throughput floor, should pick the highest
        // spectral efficiency modcod (64-QAM 3/4).
        assert_eq!(r.selected_modcod.name, "64-QAM 3/4");
        assert!(r.expected_ber <= 1e-5);
        assert!(r.margin_db > 0.0);
    }

    #[test]
    fn test_optimize_returns_none_when_infeasible() {
        let mut params = default_params();
        params.path_loss_db = 180.0; // impossibly bad link
        let constraints = Constraints {
            max_tx_power_dbm: 20.0,
            min_ber: 1e-6,
            min_throughput_bps: 0.0,
            max_latency_ms: None,
        };
        let opt = LinkBudgetOptimizer::new(params, constraints);
        assert!(opt.optimize().is_none());
    }

    #[test]
    fn test_optimize_respects_throughput_floor() {
        let opt = LinkBudgetOptimizer::new(
            default_params(),
            Constraints {
                max_tx_power_dbm: 36.0,
                min_ber: 1e-5,
                min_throughput_bps: 2_500_000.0, // > QPSK 3/4 throughput
                max_latency_ms: None,
            },
        );
        let result = opt.optimize();
        assert!(result.is_some());
        let r = result.unwrap();
        assert!(
            r.throughput_bps >= 2_500_000.0,
            "throughput {} should be >= 2.5 Mbps",
            r.throughput_bps
        );
    }

    #[test]
    fn test_erfc_symmetry() {
        // erfc(-x) = 2 - erfc(x)
        let x = 1.5;
        let diff = erfc(-x) - (2.0 - erfc(x));
        assert!(diff.abs() < 1e-12, "erfc symmetry violated: {diff}");
    }

    #[test]
    fn test_erfc_boundary_values() {
        assert!((erfc(0.0) - 1.0).abs() < 1e-6, "erfc(0) should be ~1");
        assert!(erfc(5.0) < 1e-10, "erfc(5) should be tiny");
    }
}

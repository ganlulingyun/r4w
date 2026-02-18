//! # Coulometry Charge Analyzer
//!
//! Coulometric analysis for measuring charge transfer in electrochemical cells,
//! battery cycling, electroplating, and Karl Fischer titration via precise
//! current-time integration (Q = integral I dt).
//!
//! ## Key Components
//!
//! - **Chronoamperogram** - Current vs time data with trapezoidal integration and Cottrell analysis
//! - **BatteryCycler** - Charge/discharge cycle analysis with coulombic/energy efficiency
//! - **GalvanostaticProfile** - Voltage vs capacity curves with dQ/dV and dV/dQ analysis
//! - **FaradayCalculator** - Faraday's law of electrolysis for mass/charge calculations
//! - **KarlFischerTitrator** - Coulometric Karl Fischer water determination
//! - **RateCapabilityAnalyzer** - C-rate performance with Peukert's law and Ragone plots
//! - **ImpedanceFromPulse** - Pulse resistance measurement (ohmic + polarization)
//! - **CoulometricTitrator** - Controlled-current coulometric titration
//! - **ChargeIntegrator** - High-precision charge measurement (trapezoidal, Simpson)
//! - **CoulometrySimulator** - Synthetic data generation for testing
//!
//! ## Physics
//!
//! - Faraday's law: m = QM/(zF), F = 96485.33 C/mol
//! - Cottrell equation: I(t) = nFAD^(1/2) C / (pi t)^(1/2)
//! - Karl Fischer: 1 mg H2O = 10.71 C
//! - Peukert's law: Cp = I^k * t
//! - Coulombic efficiency: CE = Q_discharge / Q_charge

use std::f64::consts::PI;

/// Faraday constant in C/mol
const FARADAY_CONSTANT: f64 = 96485.33212;

/// Karl Fischer equivalence: coulombs per milligram of water
const KF_COULOMBS_PER_MG_WATER: f64 = 10.71;

/// Seconds per hour
const SECONDS_PER_HOUR: f64 = 3600.0;

// ---------------------------------------------------------------------------
// Chronoamperogram
// ---------------------------------------------------------------------------

/// Current vs time data I(t) from a chronoamperometric experiment.
#[derive(Debug, Clone)]
pub struct Chronoamperogram {
    /// Time values in seconds
    pub time_s: Vec<f64>,
    /// Current values in amperes
    pub current_a: Vec<f64>,
}

/// Result of Cottrell analysis.
#[derive(Debug, Clone)]
pub struct CottrellResult {
    /// Theoretical prefactor nFAD^(1/2)C / pi^(1/2) in A*s^(1/2)
    pub prefactor: f64,
    /// Fitted slope from I vs 1/sqrt(t) linear regression
    pub fitted_slope: f64,
    /// R-squared goodness of fit
    pub r_squared: f64,
    /// Number of electrons transferred (estimated from fitted slope)
    pub n_electrons_estimated: f64,
}

impl Chronoamperogram {
    /// Create a new chronoamperogram from time and current arrays.
    ///
    /// # Panics
    /// Panics if lengths differ or arrays are empty.
    pub fn new(time_s: Vec<f64>, current_a: Vec<f64>) -> Self {
        assert!(
            !time_s.is_empty(),
            "time_s must not be empty"
        );
        assert_eq!(
            time_s.len(),
            current_a.len(),
            "time_s and current_a must have the same length"
        );
        Self { time_s, current_a }
    }

    /// Total charge Q = integral I(t) dt via trapezoidal integration, in coulombs.
    pub fn total_charge_c(&self) -> f64 {
        integrate_trapezoidal(&self.time_s, &self.current_a)
    }

    /// Total charge in milliamp-hours.
    pub fn total_charge_mah(&self) -> f64 {
        // Q [C] = Q [A*s]; 1 mAh = 3.6 C
        self.total_charge_c() * 1000.0 / SECONDS_PER_HOUR
    }

    /// Peak absolute current in amperes.
    pub fn peak_current(&self) -> f64 {
        self.current_a
            .iter()
            .map(|i| i.abs())
            .fold(0.0_f64, f64::max)
    }

    /// Average current magnitude in amperes.
    pub fn average_current(&self) -> f64 {
        if self.current_a.is_empty() {
            return 0.0;
        }
        let sum: f64 = self.current_a.iter().map(|i| i.abs()).sum();
        sum / self.current_a.len() as f64
    }

    /// Cottrell analysis: fit I(t) to the Cottrell equation I = nFAD^(1/2)C/(pi*t)^(1/2).
    ///
    /// Parameters:
    /// - `area_cm2`: electrode area in cm^2
    /// - `d_cm2_per_s`: diffusion coefficient in cm^2/s
    /// - `c_mol_per_cm3`: bulk concentration in mol/cm^3
    ///
    /// Returns a `CottrellResult` with theoretical prefactor, fitted slope, R^2, and
    /// estimated number of electrons.
    pub fn cottrell_analysis(
        &self,
        area_cm2: f64,
        d_cm2_per_s: f64,
        c_mol_per_cm3: f64,
    ) -> CottrellResult {
        // Cottrell: I(t) = n * F * A * sqrt(D) * C / sqrt(pi * t)
        //         = [n * F * A * sqrt(D) * C / sqrt(pi)] * 1/sqrt(t)
        // So if we plot I vs x = 1/sqrt(t), slope = n*F*A*sqrt(D)*C/sqrt(pi)

        // Build x = 1/sqrt(t), y = |I| for t > 0
        let mut xs = Vec::new();
        let mut ys = Vec::new();
        for (i, &t) in self.time_s.iter().enumerate() {
            if t > 0.0 {
                xs.push(1.0 / t.sqrt());
                ys.push(self.current_a[i].abs());
            }
        }

        let (slope, _intercept, r_sq) = linear_regression(&xs, &ys);

        // Theoretical prefactor (per electron): F * A * sqrt(D) * C / sqrt(pi)
        let per_electron = FARADAY_CONSTANT * area_cm2 * d_cm2_per_s.sqrt() * c_mol_per_cm3
            / PI.sqrt();

        let n_est = if per_electron.abs() > 1e-30 {
            slope / per_electron
        } else {
            0.0
        };

        CottrellResult {
            prefactor: per_electron,
            fitted_slope: slope,
            r_squared: r_sq,
            n_electrons_estimated: n_est,
        }
    }
}

// ---------------------------------------------------------------------------
// BatteryCycler
// ---------------------------------------------------------------------------

/// Data for a single charge/discharge cycle.
#[derive(Debug, Clone)]
pub struct CycleData {
    pub cycle_number: usize,
    pub charge_capacity_mah: f64,
    pub discharge_capacity_mah: f64,
    pub charge_energy_wh: f64,
    pub discharge_energy_wh: f64,
}

/// Battery cycle analysis.
#[derive(Debug, Clone)]
pub struct BatteryCycler {
    pub cycles: Vec<CycleData>,
}

impl BatteryCycler {
    /// Create a new battery cycler from cycle data.
    pub fn new(cycles: Vec<CycleData>) -> Self {
        Self { cycles }
    }

    /// Coulombic efficiency CE = Q_discharge / Q_charge * 100 (percent).
    pub fn coulombic_efficiency(cycle: &CycleData) -> f64 {
        if cycle.charge_capacity_mah.abs() < 1e-30 {
            return 0.0;
        }
        (cycle.discharge_capacity_mah / cycle.charge_capacity_mah) * 100.0
    }

    /// Energy efficiency EE = E_discharge / E_charge * 100 (percent).
    pub fn energy_efficiency(cycle: &CycleData) -> f64 {
        if cycle.charge_energy_wh.abs() < 1e-30 {
            return 0.0;
        }
        (cycle.discharge_energy_wh / cycle.charge_energy_wh) * 100.0
    }

    /// Capacity retention relative to the first cycle (percent).
    pub fn capacity_retention(cycles: &[CycleData]) -> Vec<f64> {
        if cycles.is_empty() {
            return Vec::new();
        }
        let initial = cycles[0].discharge_capacity_mah;
        if initial.abs() < 1e-30 {
            return vec![0.0; cycles.len()];
        }
        cycles
            .iter()
            .map(|c| (c.discharge_capacity_mah / initial) * 100.0)
            .collect()
    }

    /// Capacity fade rate in percent per cycle, from linear fit of retention vs cycle number.
    pub fn capacity_fade_rate(cycles: &[CycleData]) -> f64 {
        if cycles.len() < 2 {
            return 0.0;
        }
        let retention = Self::capacity_retention(cycles);
        let xs: Vec<f64> = cycles.iter().map(|c| c.cycle_number as f64).collect();
        let (slope, _intercept, _r_sq) = linear_regression(&xs, &retention);
        // slope is change in retention (%) per cycle; fade rate is the negative of that
        -slope
    }

    /// Predict how many cycles until capacity drops to `threshold_percent` of initial.
    /// Uses linear extrapolation of capacity retention.
    pub fn cycle_life_prediction(cycles: &[CycleData], threshold_percent: f64) -> usize {
        if cycles.len() < 2 {
            return 0;
        }
        let retention = Self::capacity_retention(cycles);
        let xs: Vec<f64> = cycles.iter().map(|c| c.cycle_number as f64).collect();
        let (slope, intercept, _) = linear_regression(&xs, &retention);

        if slope.abs() < 1e-30 || slope > 0.0 {
            // No fade or gaining capacity - infinite life
            return usize::MAX;
        }
        // threshold = slope * n + intercept => n = (threshold - intercept) / slope
        let n = (threshold_percent - intercept) / slope;
        if n < 0.0 {
            0
        } else {
            n.ceil() as usize
        }
    }
}

// ---------------------------------------------------------------------------
// GalvanostaticProfile
// ---------------------------------------------------------------------------

/// Voltage vs capacity curve from a galvanostatic experiment.
#[derive(Debug, Clone)]
pub struct GalvanostaticProfile {
    /// Capacity values in mAh
    pub capacity_mah: Vec<f64>,
    /// Voltage values in V
    pub voltage_v: Vec<f64>,
}

impl GalvanostaticProfile {
    /// Create a new galvanostatic profile.
    ///
    /// # Panics
    /// Panics if lengths differ or arrays are empty.
    pub fn new(capacity_mah: Vec<f64>, voltage_v: Vec<f64>) -> Self {
        assert!(!capacity_mah.is_empty(), "capacity_mah must not be empty");
        assert_eq!(
            capacity_mah.len(),
            voltage_v.len(),
            "capacity_mah and voltage_v must have the same length"
        );
        Self {
            capacity_mah,
            voltage_v,
        }
    }

    /// Specific capacity in mAh/g vs voltage for a given electrode mass.
    pub fn specific_capacity(&self, mass_g: f64) -> Vec<(f64, f64)> {
        assert!(mass_g > 0.0, "mass_g must be positive");
        self.capacity_mah
            .iter()
            .zip(self.voltage_v.iter())
            .map(|(&q, &v)| (q / mass_g, v))
            .collect()
    }

    /// Differential capacity dQ/dV vs V for identifying phase transitions.
    /// Uses central differences for interior points, forward/backward at boundaries.
    pub fn differential_capacity(&self) -> Vec<(f64, f64)> {
        let n = self.capacity_mah.len();
        if n < 2 {
            return Vec::new();
        }
        let mut result = Vec::with_capacity(n);
        for i in 0..n {
            let dq_dv = if i == 0 {
                let dv = self.voltage_v[1] - self.voltage_v[0];
                if dv.abs() < 1e-30 {
                    0.0
                } else {
                    (self.capacity_mah[1] - self.capacity_mah[0]) / dv
                }
            } else if i == n - 1 {
                let dv = self.voltage_v[n - 1] - self.voltage_v[n - 2];
                if dv.abs() < 1e-30 {
                    0.0
                } else {
                    (self.capacity_mah[n - 1] - self.capacity_mah[n - 2]) / dv
                }
            } else {
                let dv = self.voltage_v[i + 1] - self.voltage_v[i - 1];
                if dv.abs() < 1e-30 {
                    0.0
                } else {
                    (self.capacity_mah[i + 1] - self.capacity_mah[i - 1]) / dv
                }
            };
            result.push((self.voltage_v[i], dq_dv));
        }
        result
    }

    /// Differential voltage dV/dQ vs Q for plateau identification.
    pub fn differential_voltage(&self) -> Vec<(f64, f64)> {
        let n = self.capacity_mah.len();
        if n < 2 {
            return Vec::new();
        }
        let mut result = Vec::with_capacity(n);
        for i in 0..n {
            let dv_dq = if i == 0 {
                let dq = self.capacity_mah[1] - self.capacity_mah[0];
                if dq.abs() < 1e-30 {
                    0.0
                } else {
                    (self.voltage_v[1] - self.voltage_v[0]) / dq
                }
            } else if i == n - 1 {
                let dq = self.capacity_mah[n - 1] - self.capacity_mah[n - 2];
                if dq.abs() < 1e-30 {
                    0.0
                } else {
                    (self.voltage_v[n - 1] - self.voltage_v[n - 2]) / dq
                }
            } else {
                let dq = self.capacity_mah[i + 1] - self.capacity_mah[i - 1];
                if dq.abs() < 1e-30 {
                    0.0
                } else {
                    (self.voltage_v[i + 1] - self.voltage_v[i - 1]) / dq
                }
            };
            result.push((self.capacity_mah[i], dv_dq));
        }
        result
    }

    /// Voltage at 50% of total capacity (midpoint voltage).
    pub fn mid_voltage(&self) -> f64 {
        if self.capacity_mah.is_empty() {
            return 0.0;
        }
        let max_q = self.capacity_mah.last().copied().unwrap_or(0.0);
        let min_q = self.capacity_mah.first().copied().unwrap_or(0.0);
        let target = (min_q + max_q) / 2.0;

        // Find two points bracketing 50% capacity
        for i in 0..self.capacity_mah.len() - 1 {
            let q0 = self.capacity_mah[i];
            let q1 = self.capacity_mah[i + 1];
            if (q0 <= target && q1 >= target) || (q0 >= target && q1 <= target) {
                let dq = q1 - q0;
                if dq.abs() < 1e-30 {
                    return self.voltage_v[i];
                }
                let frac = (target - q0) / dq;
                return self.voltage_v[i] + frac * (self.voltage_v[i + 1] - self.voltage_v[i]);
            }
        }
        // Fallback: return midpoint voltage
        self.voltage_v[self.voltage_v.len() / 2]
    }

    /// Voltage hysteresis between charge and discharge curves.
    /// Computed as the average absolute voltage difference at matching capacity points.
    pub fn voltage_hysteresis(
        charge: &GalvanostaticProfile,
        discharge: &GalvanostaticProfile,
    ) -> f64 {
        if charge.capacity_mah.is_empty() || discharge.capacity_mah.is_empty() {
            return 0.0;
        }

        // Interpolate discharge voltage at each charge capacity point
        let mut total_diff = 0.0;
        let mut count = 0usize;

        for (i, &q) in charge.capacity_mah.iter().enumerate() {
            if let Some(v_dis) =
                interpolate_at(&discharge.capacity_mah, &discharge.voltage_v, q)
            {
                total_diff += (charge.voltage_v[i] - v_dis).abs();
                count += 1;
            }
        }

        if count == 0 {
            0.0
        } else {
            total_diff / count as f64
        }
    }
}

// ---------------------------------------------------------------------------
// FaradayCalculator
// ---------------------------------------------------------------------------

/// Faraday's law calculations for electrolysis.
pub struct FaradayCalculator;

impl FaradayCalculator {
    /// Faraday constant F = 96485.33 C/mol.
    pub fn faraday_constant() -> f64 {
        FARADAY_CONSTANT
    }

    /// Mass deposited by electrolysis: m = Q * M / (z * F).
    ///
    /// - `charge_c`: charge in coulombs
    /// - `molar_mass`: molar mass in g/mol
    /// - `z_electrons`: number of electrons transferred per ion
    pub fn mass_deposited(charge_c: f64, molar_mass: f64, z_electrons: u32) -> f64 {
        assert!(z_electrons > 0, "z_electrons must be positive");
        charge_c * molar_mass / (z_electrons as f64 * FARADAY_CONSTANT)
    }

    /// Charge needed to deposit a given mass: Q = m * z * F / M.
    pub fn charge_for_mass(mass_g: f64, molar_mass: f64, z_electrons: u32) -> f64 {
        assert!(z_electrons > 0, "z_electrons must be positive");
        assert!(molar_mass > 0.0, "molar_mass must be positive");
        mass_g * z_electrons as f64 * FARADAY_CONSTANT / molar_mass
    }

    /// Current efficiency = (actual mass / theoretical mass) * 100%.
    pub fn current_efficiency(actual_mass: f64, theoretical_mass: f64) -> f64 {
        if theoretical_mass.abs() < 1e-30 {
            return 0.0;
        }
        (actual_mass / theoretical_mass) * 100.0
    }

    /// Moles from charge: n = Q / (z * F).
    pub fn moles_from_charge(charge_c: f64, z_electrons: u32) -> f64 {
        assert!(z_electrons > 0, "z_electrons must be positive");
        charge_c / (z_electrons as f64 * FARADAY_CONSTANT)
    }
}

// ---------------------------------------------------------------------------
// KarlFischerTitrator
// ---------------------------------------------------------------------------

/// Coulometric Karl Fischer water determination.
pub struct KarlFischerTitrator;

impl KarlFischerTitrator {
    /// Water content in ppm from charge consumed.
    ///
    /// - `charge_mc`: charge in millicoulombs
    /// - `sample_mass_g`: sample mass in grams
    ///
    /// 1 mg H2O = 10.71 C => water_mg = charge_mc / (10.71 * 1000) [mc to C]
    /// ppm = water_mg * 1e6 / (sample_mass_g * 1000)  [g to mg]
    pub fn water_content_ppm(charge_mc: f64, sample_mass_g: f64) -> f64 {
        assert!(sample_mass_g > 0.0, "sample_mass_g must be positive");
        // charge_mc millicoulombs -> coulombs: charge_mc / 1000
        // water_mg = charge_C / KF_COULOMBS_PER_MG_WATER
        let charge_c = charge_mc / 1000.0;
        let water_mg = charge_c / KF_COULOMBS_PER_MG_WATER;
        // ppm = (water_mg / 1000) / sample_mass_g * 1e6
        //     = water_mg * 1000 / sample_mass_g
        (water_mg / 1000.0) / sample_mass_g * 1e6
    }

    /// Water mass in micrograms from charge in millicoulombs.
    pub fn water_mass_ug(charge_mc: f64) -> f64 {
        let charge_c = charge_mc / 1000.0;
        let water_mg = charge_c / KF_COULOMBS_PER_MG_WATER;
        water_mg * 1000.0 // mg to ug
    }

    /// Drift-corrected charge: subtract background moisture drift.
    ///
    /// - `raw_charge`: raw charge in millicoulombs
    /// - `drift_rate_ug_per_min`: drift rate in ug/min
    /// - `time_min`: elapsed time in minutes
    pub fn drift_correction(
        raw_charge: f64,
        drift_rate_ug_per_min: f64,
        time_min: f64,
    ) -> f64 {
        // drift water mass in ug
        let drift_water_ug = drift_rate_ug_per_min * time_min;
        // convert drift water to charge: water_mg = charge_C / 10.71
        // => charge_mc = water_mg * 10.71 * 1000 = water_ug * 10.71 / 1000 * 1000 = water_ug * 10.71
        // Wait: 1 mg H2O = 10.71 C => 1 ug H2O = 10.71/1000 C = 10.71 mC / 1000
        // charge_mc = water_ug / 1000 * 10.71 * 1000 = water_ug * 10.71
        // Actually: water_mg = charge_C / 10.71; charge_C = water_mg * 10.71
        // charge_mc = charge_C * 1000 = water_mg * 10710
        // water_ug = water_mg * 1000 => water_mg = water_ug / 1000
        // charge_mc_drift = (water_ug / 1000) * 10.71 * 1000 = water_ug * 10.71
        let drift_charge_mc = drift_water_ug * KF_COULOMBS_PER_MG_WATER;
        raw_charge - drift_charge_mc
    }

    /// Biamperometric endpoint detection: find the time/index where current
    /// drops below the threshold, indicating the titration endpoint.
    ///
    /// Returns the index of the first sample where current drops to or below `threshold_ua`.
    /// If current never drops below, returns the last index.
    pub fn endpoint_detection(current: &[f64], threshold_ua: f64) -> f64 {
        // Look for the point where current transitions below threshold
        // Typically in KF, the generator current drops when all water is consumed
        for (i, &c) in current.iter().enumerate() {
            if c <= threshold_ua {
                return i as f64;
            }
        }
        current.len().saturating_sub(1) as f64
    }
}

// ---------------------------------------------------------------------------
// RateCapabilityAnalyzer
// ---------------------------------------------------------------------------

/// Result of rate capability analysis.
#[derive(Debug, Clone)]
pub struct RateCapabilityResult {
    /// Capacity at 1C rate in mAh (interpolated if needed)
    pub capacity_at_1c: f64,
    /// Capacity retention at each C-rate: (c_rate, percent_of_1c)
    pub capacity_retention_at_rates: Vec<(f64, f64)>,
    /// Ragone data: (specific_energy, specific_power) pairs
    pub ragone_data: Vec<(f64, f64)>,
}

/// C-rate performance analysis.
pub struct RateCapabilityAnalyzer;

impl RateCapabilityAnalyzer {
    /// Analyze rate capability from C-rate vs capacity data.
    ///
    /// - `c_rates`: array of C-rate values (e.g., 0.1, 0.5, 1.0, 2.0, 5.0)
    /// - `capacities_mah`: measured capacity at each C-rate
    pub fn analyze(c_rates: &[f64], capacities_mah: &[f64]) -> RateCapabilityResult {
        assert_eq!(
            c_rates.len(),
            capacities_mah.len(),
            "c_rates and capacities must have the same length"
        );
        assert!(!c_rates.is_empty(), "must have at least one data point");

        // Find capacity at 1C (interpolate if needed)
        let cap_1c = interpolate_at(c_rates, capacities_mah, 1.0).unwrap_or(capacities_mah[0]);

        // Capacity retention relative to interpolated 1C value
        let retention: Vec<(f64, f64)> = c_rates
            .iter()
            .zip(capacities_mah.iter())
            .map(|(&rate, &cap)| {
                let ret = if cap_1c.abs() > 1e-30 {
                    (cap / cap_1c) * 100.0
                } else {
                    0.0
                };
                (rate, ret)
            })
            .collect();

        // Ragone plot: energy vs power (normalized per 1C capacity)
        // Energy ~ capacity * nominal_voltage (use capacity as proxy for Wh/kg)
        // Power ~ C_rate * nominal_voltage (use c_rate * capacity as proxy)
        let ragone: Vec<(f64, f64)> = c_rates
            .iter()
            .zip(capacities_mah.iter())
            .map(|(&rate, &cap)| {
                let energy = cap; // proxy for specific energy
                let power = cap * rate; // proxy for specific power
                (energy, power)
            })
            .collect();

        RateCapabilityResult {
            capacity_at_1c: cap_1c,
            capacity_retention_at_rates: retention,
            ragone_data: ragone,
        }
    }

    /// Peukert's equation: t = Cp / I^k where Cp = I_ref^k * t_ref.
    ///
    /// Returns predicted discharge time at current `i`.
    /// - `i_ref`: reference current (A)
    /// - `t_ref`: reference discharge time (hours)
    /// - `i`: target current (A)
    /// - `k`: Peukert exponent (typically 1.1-1.4 for lead-acid, ~1.05 for Li-ion)
    pub fn peukert_equation(i_ref: f64, t_ref: f64, i: f64, k: f64) -> f64 {
        assert!(i > 0.0, "current must be positive");
        let cp = i_ref.powf(k) * t_ref;
        cp / i.powf(k)
    }

    /// Build a Ragone plot from energy and power density data.
    /// Returns a sorted list of (energy_wh_per_kg, power_w_per_kg) tuples.
    pub fn ragone_plot(
        energies_wh_per_kg: &[f64],
        powers_w_per_kg: &[f64],
    ) -> Vec<(f64, f64)> {
        assert_eq!(
            energies_wh_per_kg.len(),
            powers_w_per_kg.len(),
            "energy and power arrays must have the same length"
        );
        let mut data: Vec<(f64, f64)> = energies_wh_per_kg
            .iter()
            .zip(powers_w_per_kg.iter())
            .map(|(&e, &p)| (e, p))
            .collect();
        data.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        data
    }
}

// ---------------------------------------------------------------------------
// ImpedanceFromPulse
// ---------------------------------------------------------------------------

/// Result of pulse resistance analysis.
#[derive(Debug, Clone)]
pub struct PulseResult {
    /// Ohmic (instantaneous) resistance in ohms
    pub r_ohmic: f64,
    /// Polarization (slow) resistance in ohms
    pub r_polarization: f64,
    /// Total resistance = r_ohmic + r_polarization
    pub r_total: f64,
}

/// Pulse resistance measurement from galvanostatic intermittent pulses.
pub struct ImpedanceFromPulse;

impl ImpedanceFromPulse {
    /// DC resistance from voltage step: R_dc = |delta_V| / |I|.
    pub fn dc_resistance(v_before: f64, v_after: f64, current: f64) -> f64 {
        if current.abs() < 1e-30 {
            return 0.0;
        }
        (v_before - v_after).abs() / current.abs()
    }

    /// Analyze a voltage pulse to extract ohmic and polarization resistance.
    ///
    /// - `time`: time values during the pulse (seconds)
    /// - `voltage`: voltage values during the pulse (volts)
    /// - `current`: applied current magnitude (amps, positive)
    ///
    /// The ohmic resistance is from the initial (instantaneous) voltage drop,
    /// and polarization is from the slower drift until steady state.
    pub fn pulse_analysis(time: &[f64], voltage: &[f64], current: f64) -> PulseResult {
        assert_eq!(time.len(), voltage.len());
        assert!(time.len() >= 2, "need at least 2 points");

        let v_initial = voltage[0]; // voltage before pulse
        let v_first = voltage[1]; // voltage right after pulse onset (ohmic drop)
        let v_final = voltage[voltage.len() - 1]; // voltage at end of pulse (steady state)

        let current_abs = current.abs().max(1e-30);

        let r_ohmic = (v_initial - v_first).abs() / current_abs;
        let r_total = (v_initial - v_final).abs() / current_abs;
        let r_polarization = r_total - r_ohmic;

        PulseResult {
            r_ohmic,
            r_polarization: r_polarization.max(0.0),
            r_total,
        }
    }

    /// Area-specific resistance: ASR = R * A in ohm*cm^2.
    pub fn area_specific_resistance(r_ohm: f64, area_cm2: f64) -> f64 {
        r_ohm * area_cm2
    }
}

// ---------------------------------------------------------------------------
// CoulometricTitrator
// ---------------------------------------------------------------------------

/// Controlled-current coulometric titration analysis.
pub struct CoulometricTitrator;

impl CoulometricTitrator {
    /// Find equivalence point from the second derivative maximum of the titration curve.
    ///
    /// - `volume_or_time`: x-axis values (volume or time)
    /// - `signal`: y-axis values (pH or mV)
    ///
    /// Returns the x-value at the equivalence point (maximum of |d(signal)/dx|).
    pub fn equivalence_point(volume_or_time: &[f64], signal: &[f64]) -> f64 {
        assert_eq!(volume_or_time.len(), signal.len());
        let n = signal.len();
        if n < 3 {
            return volume_or_time[n / 2];
        }

        // Compute first derivative |dS/dx|
        let mut max_deriv = 0.0_f64;
        let mut ep_x = volume_or_time[0];

        for i in 1..n - 1 {
            let dx = volume_or_time[i + 1] - volume_or_time[i - 1];
            if dx.abs() < 1e-30 {
                continue;
            }
            let ds = (signal[i + 1] - signal[i - 1]) / dx;
            if ds.abs() > max_deriv {
                max_deriv = ds.abs();
                ep_x = volume_or_time[i];
            }
        }

        ep_x
    }

    /// Concentration from coulometric titration: C = Q / (z * F * V).
    ///
    /// - `charge_c`: charge in coulombs
    /// - `volume_l`: solution volume in liters
    /// - `z`: number of electrons per equivalent
    pub fn concentration_from_charge(charge_c: f64, volume_l: f64, z: u32) -> f64 {
        assert!(z > 0, "z must be positive");
        assert!(volume_l > 0.0, "volume must be positive");
        charge_c / (z as f64 * FARADAY_CONSTANT * volume_l)
    }

    /// Return the titration curve as (charge_mc, signal) pairs.
    pub fn titration_curve(charge_mc: &[f64], ph_or_mv: &[f64]) -> Vec<(f64, f64)> {
        assert_eq!(charge_mc.len(), ph_or_mv.len());
        charge_mc
            .iter()
            .zip(ph_or_mv.iter())
            .map(|(&q, &s)| (q, s))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// ChargeIntegrator
// ---------------------------------------------------------------------------

/// High-precision charge measurement via numerical integration.
pub struct ChargeIntegrator;

impl ChargeIntegrator {
    /// Trapezoidal integration: Q = sum_{i=0}^{N-2} (t[i+1]-t[i]) * (I[i]+I[i+1]) / 2.
    pub fn integrate_trapezoidal(time: &[f64], current: &[f64]) -> f64 {
        integrate_trapezoidal(time, current)
    }

    /// Simpson's 1/3 rule integration for more precise results.
    /// Falls back to trapezoidal for the last interval if N is even.
    pub fn integrate_simpson(time: &[f64], current: &[f64]) -> f64 {
        integrate_simpson(time, current)
    }

    /// Cumulative charge Q(t): running integral at each time point.
    pub fn cumulative_charge(time: &[f64], current: &[f64]) -> Vec<f64> {
        assert_eq!(time.len(), current.len());
        let n = time.len();
        let mut result = Vec::with_capacity(n);
        let mut q = 0.0;
        result.push(q);
        for i in 1..n {
            let dt = time[i] - time[i - 1];
            q += dt * (current[i] + current[i - 1]) / 2.0;
            result.push(q);
        }
        result
    }

    /// Noise floor charge: estimated charge error from current noise.
    ///
    /// - `current_noise_rms`: RMS current noise in amperes
    /// - `duration_s`: measurement duration in seconds
    /// - `sample_rate`: sampling rate in Hz
    ///
    /// Integration of white noise: sigma_Q = sigma_I * dt * sqrt(N) = sigma_I * sqrt(T/fs)
    pub fn noise_floor_charge(
        current_noise_rms: f64,
        duration_s: f64,
        sample_rate: f64,
    ) -> f64 {
        assert!(sample_rate > 0.0, "sample_rate must be positive");
        let n_samples = duration_s * sample_rate;
        let dt = 1.0 / sample_rate;
        // RMS of integrated noise: sigma_Q = sigma_I * dt * sqrt(N)
        current_noise_rms * dt * n_samples.sqrt()
    }
}

// ---------------------------------------------------------------------------
// CoulometrySimulator
// ---------------------------------------------------------------------------

/// Generate synthetic coulometric data for testing.
pub struct CoulometrySimulator;

impl CoulometrySimulator {
    /// Simulate a Cottrell-type chronoamperogram: I(t) = n*F*A*sqrt(D)*C / sqrt(pi*t).
    ///
    /// - `n`: number of electrons
    /// - `area_cm2`: electrode area in cm^2
    /// - `d`: diffusion coefficient in cm^2/s
    /// - `c`: concentration in mol/cm^3
    /// - `duration`: total time in seconds
    ///
    /// Returns a chronoamperogram with 1000 evenly spaced time points (starting from 0.001s).
    pub fn simulate_cottrell(
        n: u32,
        area_cm2: f64,
        d: f64,
        c: f64,
        duration: f64,
    ) -> Chronoamperogram {
        let num_points = 1000;
        let t_start = 0.001; // avoid t=0 singularity
        let dt = (duration - t_start) / (num_points - 1) as f64;

        let prefactor =
            n as f64 * FARADAY_CONSTANT * area_cm2 * d.sqrt() * c / PI.sqrt();

        let mut time_s = Vec::with_capacity(num_points);
        let mut current_a = Vec::with_capacity(num_points);

        for i in 0..num_points {
            let t = t_start + i as f64 * dt;
            time_s.push(t);
            current_a.push(prefactor / t.sqrt());
        }

        Chronoamperogram { time_s, current_a }
    }

    /// Simulate a battery charge/discharge cycle.
    ///
    /// - `capacity_mah`: nominal capacity in mAh
    /// - `voltage_range`: (V_min, V_max)
    /// - `num_points`: number of data points per half-cycle
    ///
    /// Returns (charge_profile, discharge_profile).
    pub fn simulate_battery_cycle(
        capacity_mah: f64,
        voltage_range: (f64, f64),
        num_points: usize,
    ) -> (GalvanostaticProfile, GalvanostaticProfile) {
        let (v_min, v_max) = voltage_range;
        let dq = capacity_mah / (num_points - 1) as f64;

        // Charge: voltage rises from v_min to v_max with a slight S-curve
        let mut charge_q = Vec::with_capacity(num_points);
        let mut charge_v = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let q = i as f64 * dq;
            let frac = i as f64 / (num_points - 1) as f64;
            // Sigmoid-like voltage rise
            let v = v_min + (v_max - v_min) * (3.0 * frac * frac - 2.0 * frac * frac * frac);
            charge_q.push(q);
            charge_v.push(v);
        }

        // Discharge: voltage drops from v_max to v_min with a slight plateau
        let mut discharge_q = Vec::with_capacity(num_points);
        let mut discharge_v = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let q = i as f64 * dq;
            let frac = i as f64 / (num_points - 1) as f64;
            let v = v_max - (v_max - v_min) * (3.0 * frac * frac - 2.0 * frac * frac * frac);
            discharge_q.push(q);
            discharge_v.push(v);
        }

        (
            GalvanostaticProfile::new(charge_q, charge_v),
            GalvanostaticProfile::new(discharge_q, discharge_v),
        )
    }

    /// Simulate battery cycling with capacity fade.
    ///
    /// - `initial_capacity`: initial discharge capacity in mAh
    /// - `fade_per_cycle`: capacity loss per cycle in mAh
    /// - `num_cycles`: number of cycles to simulate
    pub fn simulate_cycling(
        initial_capacity: f64,
        fade_per_cycle: f64,
        num_cycles: usize,
    ) -> Vec<CycleData> {
        let mut cycles = Vec::with_capacity(num_cycles);
        for i in 0..num_cycles {
            let discharge_cap = (initial_capacity - fade_per_cycle * i as f64).max(0.0);
            // Charge capacity is slightly higher (CE < 100%)
            let charge_cap = discharge_cap * 1.01;
            // Energy based on nominal voltage ~3.7V
            let nom_v = 3.7;
            cycles.push(CycleData {
                cycle_number: i + 1,
                charge_capacity_mah: charge_cap,
                discharge_capacity_mah: discharge_cap,
                charge_energy_wh: charge_cap * nom_v / 1000.0,
                discharge_energy_wh: discharge_cap * nom_v / 1000.0,
            });
        }
        cycles
    }

    /// Add Gaussian noise to a chronoamperogram.
    ///
    /// Uses a simple LCG PRNG to generate pseudo-random noise.
    pub fn add_noise(data: &Chronoamperogram, noise_a: f64) -> Chronoamperogram {
        let mut rng_state: u64 = 0xDEAD_BEEF_CAFE_1234;
        let mut current_a = Vec::with_capacity(data.current_a.len());
        for &i in &data.current_a {
            // Box-Muller transform using LCG
            let u1 = lcg_uniform(&mut rng_state);
            let u2 = lcg_uniform(&mut rng_state);
            let z = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos();
            current_a.push(i + noise_a * z);
        }
        Chronoamperogram {
            time_s: data.time_s.clone(),
            current_a,
        }
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Trapezoidal integration of y over x.
fn integrate_trapezoidal(x: &[f64], y: &[f64]) -> f64 {
    assert_eq!(x.len(), y.len());
    let mut sum = 0.0;
    for i in 0..x.len().saturating_sub(1) {
        let dx = x[i + 1] - x[i];
        sum += dx * (y[i] + y[i + 1]) / 2.0;
    }
    sum
}

/// Simpson's 1/3 rule integration.
/// For N points (N-1 intervals), applies Simpson's rule on pairs of intervals,
/// with a final trapezoidal step if (N-1) is odd.
fn integrate_simpson(x: &[f64], y: &[f64]) -> f64 {
    assert_eq!(x.len(), y.len());
    let n = x.len();
    if n < 2 {
        return 0.0;
    }
    if n == 2 {
        return integrate_trapezoidal(x, y);
    }

    let mut sum = 0.0;
    let mut i = 0;

    // Apply Simpson's 1/3 rule on pairs of intervals
    while i + 2 < n {
        let h0 = x[i + 1] - x[i];
        let h1 = x[i + 2] - x[i + 1];
        // For non-uniform spacing, use composite formula
        let h = x[i + 2] - x[i];
        if (h0 - h1).abs() < 1e-12 * h.abs() {
            // Uniform spacing: standard Simpson's
            sum += h / 6.0 * (y[i] + 4.0 * y[i + 1] + y[i + 2]);
        } else {
            // Non-uniform Simpson's
            let alpha = (2.0 * h0 - h1) / (6.0 * h0 / h);
            let beta = h * h * h / (6.0 * h0 * h1);
            let gamma = (2.0 * h1 - h0) / (6.0 * h1 / h);
            sum += alpha * y[i] + beta * y[i + 1] + gamma * y[i + 2];
        }
        i += 2;
    }

    // If there's a leftover interval, use trapezoidal rule
    if i + 1 < n {
        let dx = x[i + 1] - x[i];
        sum += dx * (y[i] + y[i + 1]) / 2.0;
    }

    sum
}

/// Linear regression: y = slope * x + intercept.
/// Returns (slope, intercept, r_squared).
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64, f64) {
    let n = x.len() as f64;
    if n < 2.0 {
        return (0.0, y.first().copied().unwrap_or(0.0), 0.0);
    }

    let sum_x: f64 = x.iter().sum();
    let sum_y: f64 = y.iter().sum();
    let sum_xy: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * yi).sum();
    let sum_xx: f64 = x.iter().map(|xi| xi * xi).sum();

    let denom = n * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (0.0, sum_y / n, 0.0);
    }

    let slope = (n * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n;

    // R-squared
    let y_mean = sum_y / n;
    let ss_tot: f64 = y.iter().map(|yi| (yi - y_mean) * (yi - y_mean)).sum();
    let ss_res: f64 = x
        .iter()
        .zip(y.iter())
        .map(|(xi, yi)| {
            let pred = slope * xi + intercept;
            (yi - pred) * (yi - pred)
        })
        .sum();

    let r_sq = if ss_tot.abs() < 1e-30 {
        1.0
    } else {
        1.0 - ss_res / ss_tot
    };

    (slope, intercept, r_sq)
}

/// Linear interpolation of y at target_x given sorted x, y arrays.
fn interpolate_at(x: &[f64], y: &[f64], target_x: f64) -> Option<f64> {
    if x.is_empty() {
        return None;
    }
    if x.len() == 1 {
        return Some(y[0]);
    }

    // Clamp to range
    if target_x <= x[0] {
        return Some(y[0]);
    }
    if target_x >= x[x.len() - 1] {
        return Some(y[y.len() - 1]);
    }

    // Find bracketing interval
    for i in 0..x.len() - 1 {
        if (x[i] <= target_x && x[i + 1] >= target_x)
            || (x[i] >= target_x && x[i + 1] <= target_x)
        {
            let dx = x[i + 1] - x[i];
            if dx.abs() < 1e-30 {
                return Some(y[i]);
            }
            let frac = (target_x - x[i]) / dx;
            return Some(y[i] + frac * (y[i + 1] - y[i]));
        }
    }

    Some(y[y.len() - 1])
}

/// Simple LCG PRNG returning a uniform value in (0, 1).
fn lcg_uniform(state: &mut u64) -> f64 {
    *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    // Map upper bits to [0,1)
    let val = (*state >> 11) as f64 / (1u64 << 53) as f64;
    // Clamp away from 0 for log safety
    val.max(1e-15)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    // ===== Chronoamperogram tests =====

    #[test]
    fn test_chronoamperogram_new() {
        let ca = Chronoamperogram::new(vec![0.0, 1.0, 2.0], vec![1.0, 0.5, 0.25]);
        assert_eq!(ca.time_s.len(), 3);
        assert_eq!(ca.current_a.len(), 3);
    }

    #[test]
    #[should_panic]
    fn test_chronoamperogram_new_empty() {
        Chronoamperogram::new(vec![], vec![]);
    }

    #[test]
    #[should_panic]
    fn test_chronoamperogram_new_mismatch() {
        Chronoamperogram::new(vec![0.0, 1.0], vec![1.0]);
    }

    #[test]
    fn test_total_charge_constant_current() {
        // 1 A for 10 s = 10 C
        let ca = Chronoamperogram::new(vec![0.0, 10.0], vec![1.0, 1.0]);
        assert!((ca.total_charge_c() - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_total_charge_linear_ramp() {
        // Current ramps from 0 to 2 A over 10 s: Q = 0.5 * 10 * 2 = 10 C
        let ca = Chronoamperogram::new(vec![0.0, 10.0], vec![0.0, 2.0]);
        assert!((ca.total_charge_c() - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_total_charge_mah() {
        // 3.6 C = 1 mAh
        let ca = Chronoamperogram::new(vec![0.0, 3.6], vec![1.0, 1.0]);
        assert!((ca.total_charge_mah() - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_peak_current() {
        let ca = Chronoamperogram::new(vec![0.0, 1.0, 2.0], vec![0.5, -2.0, 1.0]);
        assert!((ca.peak_current() - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_average_current() {
        let ca = Chronoamperogram::new(vec![0.0, 1.0, 2.0], vec![1.0, 2.0, 3.0]);
        assert!((ca.average_current() - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_average_current_negative() {
        let ca = Chronoamperogram::new(vec![0.0, 1.0], vec![-1.0, -3.0]);
        assert!((ca.average_current() - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_cottrell_analysis() {
        // Simulate Cottrell data and fit back
        let n = 1;
        let area = 1.0;
        let d = 1e-5;
        let c = 1e-6;
        let ca = CoulometrySimulator::simulate_cottrell(n, area, d, c, 10.0);
        let result = ca.cottrell_analysis(area, d, c);
        // n_electrons_estimated should be close to 1
        assert!((result.n_electrons_estimated - 1.0).abs() < 0.05);
        assert!(result.r_squared > 0.99);
    }

    #[test]
    fn test_cottrell_prefactor() {
        let ca = Chronoamperogram::new(vec![1.0], vec![1.0]);
        let result = ca.cottrell_analysis(1.0, 1e-5, 1e-6);
        let expected = FARADAY_CONSTANT * 1.0 * (1e-5_f64).sqrt() * 1e-6 / PI.sqrt();
        assert!((result.prefactor - expected).abs() < 1e-10);
    }

    // ===== BatteryCycler tests =====

    #[test]
    fn test_coulombic_efficiency_100() {
        let cycle = CycleData {
            cycle_number: 1,
            charge_capacity_mah: 100.0,
            discharge_capacity_mah: 100.0,
            charge_energy_wh: 0.37,
            discharge_energy_wh: 0.37,
        };
        assert!((BatteryCycler::coulombic_efficiency(&cycle) - 100.0).abs() < EPSILON);
    }

    #[test]
    fn test_coulombic_efficiency_partial() {
        let cycle = CycleData {
            cycle_number: 1,
            charge_capacity_mah: 100.0,
            discharge_capacity_mah: 95.0,
            charge_energy_wh: 0.37,
            discharge_energy_wh: 0.35,
        };
        assert!((BatteryCycler::coulombic_efficiency(&cycle) - 95.0).abs() < EPSILON);
    }

    #[test]
    fn test_coulombic_efficiency_zero_charge() {
        let cycle = CycleData {
            cycle_number: 1,
            charge_capacity_mah: 0.0,
            discharge_capacity_mah: 0.0,
            charge_energy_wh: 0.0,
            discharge_energy_wh: 0.0,
        };
        assert!((BatteryCycler::coulombic_efficiency(&cycle)).abs() < EPSILON);
    }

    #[test]
    fn test_energy_efficiency() {
        let cycle = CycleData {
            cycle_number: 1,
            charge_capacity_mah: 100.0,
            discharge_capacity_mah: 100.0,
            charge_energy_wh: 0.40,
            discharge_energy_wh: 0.36,
        };
        assert!((BatteryCycler::energy_efficiency(&cycle) - 90.0).abs() < EPSILON);
    }

    #[test]
    fn test_energy_efficiency_zero() {
        let cycle = CycleData {
            cycle_number: 1,
            charge_capacity_mah: 100.0,
            discharge_capacity_mah: 0.0,
            charge_energy_wh: 0.0,
            discharge_energy_wh: 0.0,
        };
        assert!((BatteryCycler::energy_efficiency(&cycle)).abs() < EPSILON);
    }

    #[test]
    fn test_capacity_retention() {
        let cycles = vec![
            CycleData { cycle_number: 1, charge_capacity_mah: 110.0, discharge_capacity_mah: 100.0, charge_energy_wh: 0.4, discharge_energy_wh: 0.37 },
            CycleData { cycle_number: 2, charge_capacity_mah: 108.0, discharge_capacity_mah: 95.0, charge_energy_wh: 0.4, discharge_energy_wh: 0.35 },
            CycleData { cycle_number: 3, charge_capacity_mah: 106.0, discharge_capacity_mah: 90.0, charge_energy_wh: 0.39, discharge_energy_wh: 0.33 },
        ];
        let ret = BatteryCycler::capacity_retention(&cycles);
        assert!((ret[0] - 100.0).abs() < EPSILON);
        assert!((ret[1] - 95.0).abs() < EPSILON);
        assert!((ret[2] - 90.0).abs() < EPSILON);
    }

    #[test]
    fn test_capacity_retention_empty() {
        let ret = BatteryCycler::capacity_retention(&[]);
        assert!(ret.is_empty());
    }

    #[test]
    fn test_capacity_fade_rate() {
        let cycles = CoulometrySimulator::simulate_cycling(100.0, 0.5, 10);
        let fade = BatteryCycler::capacity_fade_rate(&cycles);
        // 0.5 mAh/cycle out of 100 mAh = ~0.5%/cycle
        assert!((fade - 0.5).abs() < 0.1);
    }

    #[test]
    fn test_capacity_fade_rate_one_cycle() {
        let cycles = vec![CycleData {
            cycle_number: 1,
            charge_capacity_mah: 100.0,
            discharge_capacity_mah: 100.0,
            charge_energy_wh: 0.37,
            discharge_energy_wh: 0.37,
        }];
        assert!((BatteryCycler::capacity_fade_rate(&cycles)).abs() < EPSILON);
    }

    #[test]
    fn test_cycle_life_prediction() {
        // 100 mAh, losing 0.5 mAh/cycle => 80% at ~40 cycles
        let cycles = CoulometrySimulator::simulate_cycling(100.0, 0.5, 20);
        let life = BatteryCycler::cycle_life_prediction(&cycles, 80.0);
        assert!(life > 30 && life < 50, "predicted life = {}", life);
    }

    #[test]
    fn test_cycle_life_prediction_no_fade() {
        let cycles = CoulometrySimulator::simulate_cycling(100.0, 0.0, 10);
        let life = BatteryCycler::cycle_life_prediction(&cycles, 80.0);
        assert_eq!(life, usize::MAX);
    }

    // ===== GalvanostaticProfile tests =====

    #[test]
    fn test_galvanostatic_profile_new() {
        let gp = GalvanostaticProfile::new(vec![0.0, 50.0, 100.0], vec![3.0, 3.5, 4.2]);
        assert_eq!(gp.capacity_mah.len(), 3);
    }

    #[test]
    #[should_panic]
    fn test_galvanostatic_profile_empty() {
        GalvanostaticProfile::new(vec![], vec![]);
    }

    #[test]
    fn test_specific_capacity() {
        let gp = GalvanostaticProfile::new(vec![0.0, 50.0, 100.0], vec![3.0, 3.5, 4.2]);
        let sc = gp.specific_capacity(0.5);
        assert!((sc[0].0 - 0.0).abs() < EPSILON); // 0/0.5
        assert!((sc[1].0 - 100.0).abs() < EPSILON); // 50/0.5
        assert!((sc[2].0 - 200.0).abs() < EPSILON); // 100/0.5
    }

    #[test]
    fn test_differential_capacity() {
        // Linear V vs Q: V = 3.0 + 0.012*Q => dQ/dV = 1/0.012 ~ 83.33
        let q = vec![0.0, 50.0, 100.0];
        let v = vec![3.0, 3.6, 4.2];
        let gp = GalvanostaticProfile::new(q, v);
        let dc = gp.differential_capacity();
        // Central point: dQ/dV = (100-0)/(4.2-3.0) = 83.33
        assert!((dc[1].1 - 83.333333).abs() < 0.01);
    }

    #[test]
    fn test_differential_voltage() {
        let q = vec![0.0, 50.0, 100.0];
        let v = vec![3.0, 3.6, 4.2];
        let gp = GalvanostaticProfile::new(q, v);
        let dv = gp.differential_voltage();
        // Central: dV/dQ = (4.2-3.0)/(100-0) = 0.012
        assert!((dv[1].1 - 0.012).abs() < 0.001);
    }

    #[test]
    fn test_mid_voltage() {
        let gp = GalvanostaticProfile::new(vec![0.0, 50.0, 100.0], vec![3.0, 3.6, 4.2]);
        let mid_v = gp.mid_voltage();
        assert!((mid_v - 3.6).abs() < 0.01);
    }

    #[test]
    fn test_mid_voltage_interpolated() {
        let gp = GalvanostaticProfile::new(
            vec![0.0, 25.0, 75.0, 100.0],
            vec![3.0, 3.3, 3.9, 4.2],
        );
        let mid_v = gp.mid_voltage();
        assert!(mid_v > 3.3 && mid_v < 3.9, "mid_v = {}", mid_v);
    }

    #[test]
    fn test_voltage_hysteresis() {
        let charge = GalvanostaticProfile::new(
            vec![0.0, 50.0, 100.0],
            vec![3.5, 3.8, 4.2],
        );
        let discharge = GalvanostaticProfile::new(
            vec![0.0, 50.0, 100.0],
            vec![3.3, 3.6, 4.0],
        );
        let hyst = GalvanostaticProfile::voltage_hysteresis(&charge, &discharge);
        assert!((hyst - 0.2).abs() < 0.01);
    }

    #[test]
    fn test_voltage_hysteresis_empty() {
        let charge = GalvanostaticProfile::new(vec![0.0], vec![3.5]);
        let discharge = GalvanostaticProfile::new(vec![0.0], vec![3.3]);
        let hyst = GalvanostaticProfile::voltage_hysteresis(&charge, &discharge);
        assert!((hyst - 0.2).abs() < 0.01);
    }

    // ===== FaradayCalculator tests =====

    #[test]
    fn test_faraday_constant() {
        assert!((FaradayCalculator::faraday_constant() - 96485.33212).abs() < 0.001);
    }

    #[test]
    fn test_mass_deposited_copper() {
        // Cu2+ + 2e- -> Cu, M=63.546
        // Q=96485 C, z=2 => m = 96485*63.546/(2*96485) = 31.773 g
        let m = FaradayCalculator::mass_deposited(96485.33212, 63.546, 2);
        assert!((m - 31.773).abs() < 0.01);
    }

    #[test]
    fn test_mass_deposited_silver() {
        // Ag+ + e- -> Ag, M=107.868
        // Q=96485 C, z=1 => m = 107.868 g
        let m = FaradayCalculator::mass_deposited(FARADAY_CONSTANT, 107.868, 1);
        assert!((m - 107.868).abs() < 0.01);
    }

    #[test]
    fn test_charge_for_mass_roundtrip() {
        let m = 10.0; // g
        let mm = 63.546;
        let z = 2;
        let q = FaradayCalculator::charge_for_mass(m, mm, z);
        let m2 = FaradayCalculator::mass_deposited(q, mm, z);
        assert!((m - m2).abs() < 1e-8);
    }

    #[test]
    fn test_current_efficiency() {
        assert!((FaradayCalculator::current_efficiency(9.5, 10.0) - 95.0).abs() < EPSILON);
    }

    #[test]
    fn test_current_efficiency_zero() {
        assert!((FaradayCalculator::current_efficiency(1.0, 0.0)).abs() < EPSILON);
    }

    #[test]
    fn test_moles_from_charge() {
        // Q = F, z = 1 => n = 1 mol
        let n = FaradayCalculator::moles_from_charge(FARADAY_CONSTANT, 1);
        assert!((n - 1.0).abs() < 1e-8);
    }

    #[test]
    fn test_moles_from_charge_z2() {
        // Q = F, z = 2 => n = 0.5 mol
        let n = FaradayCalculator::moles_from_charge(FARADAY_CONSTANT, 2);
        assert!((n - 0.5).abs() < 1e-8);
    }

    #[test]
    #[should_panic]
    fn test_mass_deposited_z0() {
        FaradayCalculator::mass_deposited(100.0, 63.546, 0);
    }

    // ===== KarlFischerTitrator tests =====

    #[test]
    fn test_kf_water_mass_ug() {
        // 10.71 C = 1 mg = 1000 ug => 10710 mC = 1000 ug
        let ug = KarlFischerTitrator::water_mass_ug(10710.0);
        assert!((ug - 1000.0).abs() < 0.1);
    }

    #[test]
    fn test_kf_water_content_ppm() {
        // 10.71 mC -> 0.001 mg H2O = 1 ug
        // sample = 1g => ppm = 1 ug / 1g * 1e6 / 1e6 ... let's compute:
        // charge_mc = 10.71 => charge_C = 0.01071
        // water_mg = 0.01071 / 10.71 = 0.001 mg
        // ppm = (0.001/1000) / 1.0 * 1e6 = 0.001
        let ppm = KarlFischerTitrator::water_content_ppm(10.71, 1.0);
        assert!((ppm - 1.0).abs() < 0.01, "ppm = {}", ppm);
    }

    #[test]
    fn test_kf_water_content_ppm_large() {
        // 10710 mC = 10.71 C => 1 mg H2O
        // sample = 10g => ppm = (1mg/1000)/(10g) * 1e6 = 100 ppm
        let ppm = KarlFischerTitrator::water_content_ppm(10710.0, 10.0);
        assert!((ppm - 100.0).abs() < 0.1, "ppm = {}", ppm);
    }

    #[test]
    fn test_kf_drift_correction() {
        // raw = 100 mc, drift = 2 ug/min, time = 5 min => drift = 10 ug
        // drift_charge_mc = 10 * 10.71 = 107.1 mc
        let corrected = KarlFischerTitrator::drift_correction(200.0, 2.0, 5.0);
        assert!((corrected - (200.0 - 107.1)).abs() < 0.1, "corrected = {}", corrected);
    }

    #[test]
    fn test_kf_endpoint_detection() {
        let current = vec![100.0, 80.0, 60.0, 20.0, 5.0, 2.0, 1.0];
        let ep = KarlFischerTitrator::endpoint_detection(&current, 10.0);
        // 20 > 10, but 5 <= 10, so first crossing is at index 4
        assert!((ep - 4.0).abs() < EPSILON);
    }

    #[test]
    fn test_kf_endpoint_never_below() {
        let current = vec![100.0, 80.0, 60.0];
        let ep = KarlFischerTitrator::endpoint_detection(&current, 10.0);
        assert!((ep - 2.0).abs() < EPSILON); // last index
    }

    // ===== RateCapabilityAnalyzer tests =====

    #[test]
    fn test_rate_capability_analyze() {
        let c_rates = vec![0.1, 0.5, 1.0, 2.0, 5.0];
        let caps = vec![100.0, 98.0, 95.0, 85.0, 60.0];
        let result = RateCapabilityAnalyzer::analyze(&c_rates, &caps);
        assert!((result.capacity_at_1c - 95.0).abs() < 0.1);
        assert_eq!(result.capacity_retention_at_rates.len(), 5);
    }

    #[test]
    fn test_rate_capability_retention() {
        let c_rates = vec![0.5, 1.0, 2.0];
        let caps = vec![100.0, 95.0, 80.0];
        let result = RateCapabilityAnalyzer::analyze(&c_rates, &caps);
        // At 2C: 80/95 * 100 = 84.2%
        assert!((result.capacity_retention_at_rates[2].1 - 84.21).abs() < 0.1);
    }

    #[test]
    fn test_peukert_equation() {
        // At reference: 1A for 10h. Peukert k=1.2.
        // Cp = 1^1.2 * 10 = 10
        // At 2A: t = 10 / 2^1.2 = 10/2.2974 = 4.353 h
        let t = RateCapabilityAnalyzer::peukert_equation(1.0, 10.0, 2.0, 1.2);
        assert!((t - 4.353).abs() < 0.01, "t = {}", t);
    }

    #[test]
    fn test_peukert_same_current() {
        let t = RateCapabilityAnalyzer::peukert_equation(1.0, 10.0, 1.0, 1.2);
        assert!((t - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_ragone_plot() {
        let energies = vec![200.0, 150.0, 100.0, 50.0];
        let powers = vec![100.0, 500.0, 1000.0, 5000.0];
        let rp = RateCapabilityAnalyzer::ragone_plot(&energies, &powers);
        // Should be sorted by energy
        assert!((rp[0].0 - 50.0).abs() < EPSILON);
        assert!((rp[3].0 - 200.0).abs() < EPSILON);
    }

    // ===== ImpedanceFromPulse tests =====

    #[test]
    fn test_dc_resistance() {
        let r = ImpedanceFromPulse::dc_resistance(4.0, 3.8, 1.0);
        assert!((r - 0.2).abs() < EPSILON);
    }

    #[test]
    fn test_dc_resistance_zero_current() {
        let r = ImpedanceFromPulse::dc_resistance(4.0, 3.8, 0.0);
        assert!((r - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_pulse_analysis() {
        // V_initial=4.0, V_after_ohmic=3.9, V_steady=3.8, I=1.0
        let time = vec![0.0, 0.001, 0.1, 1.0, 10.0];
        let voltage = vec![4.0, 3.9, 3.85, 3.82, 3.8];
        let result = ImpedanceFromPulse::pulse_analysis(&time, &voltage, 1.0);
        assert!((result.r_ohmic - 0.1).abs() < EPSILON);
        assert!((result.r_total - 0.2).abs() < EPSILON);
        assert!((result.r_polarization - 0.1).abs() < EPSILON);
    }

    #[test]
    fn test_area_specific_resistance() {
        let asr = ImpedanceFromPulse::area_specific_resistance(0.05, 10.0);
        assert!((asr - 0.5).abs() < EPSILON);
    }

    // ===== CoulometricTitrator tests =====

    #[test]
    fn test_equivalence_point() {
        // Sharp S-curve: large derivative at midpoint
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        let y = vec![2.0, 2.5, 3.0, 3.5, 5.0, 9.0, 11.0, 11.5, 12.0, 12.3, 12.5];
        let ep = CoulometricTitrator::equivalence_point(&x, &y);
        // Steepest slope around x=5 (jump from 5 to 11)
        assert!((ep - 5.0).abs() < 1.5, "ep = {}", ep);
    }

    #[test]
    fn test_concentration_from_charge() {
        // Q = z*F*C*V => C = Q/(z*F*V)
        // Q = 96485.33, z=1, V=1L => C = 1.0 mol/L
        let c = CoulometricTitrator::concentration_from_charge(FARADAY_CONSTANT, 1.0, 1);
        assert!((c - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_concentration_from_charge_dilute() {
        let c = CoulometricTitrator::concentration_from_charge(96.48533, 1.0, 1);
        assert!((c - 0.001).abs() < 1e-6);
    }

    #[test]
    fn test_titration_curve() {
        let q = vec![0.0, 10.0, 20.0];
        let ph = vec![3.0, 7.0, 11.0];
        let curve = CoulometricTitrator::titration_curve(&q, &ph);
        assert_eq!(curve.len(), 3);
        assert!((curve[1].0 - 10.0).abs() < EPSILON);
        assert!((curve[1].1 - 7.0).abs() < EPSILON);
    }

    // ===== ChargeIntegrator tests =====

    #[test]
    fn test_integrate_trapezoidal_constant() {
        let t = vec![0.0, 1.0, 2.0, 3.0];
        let i = vec![2.0, 2.0, 2.0, 2.0];
        let q = ChargeIntegrator::integrate_trapezoidal(&t, &i);
        assert!((q - 6.0).abs() < EPSILON);
    }

    #[test]
    fn test_integrate_trapezoidal_linear() {
        let t = vec![0.0, 1.0, 2.0];
        let i = vec![0.0, 1.0, 2.0];
        let q = ChargeIntegrator::integrate_trapezoidal(&t, &i);
        // Integral of x from 0 to 2 = 2
        assert!((q - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_integrate_simpson_quadratic() {
        // Simpson's rule is exact for polynomials up to degree 3
        // Integral of x^2 from 0 to 2 = 8/3 = 2.6667
        let t = vec![0.0, 1.0, 2.0];
        let i = vec![0.0, 1.0, 4.0];
        let q = ChargeIntegrator::integrate_simpson(&t, &i);
        assert!((q - 8.0 / 3.0).abs() < 0.01, "q = {}", q);
    }

    #[test]
    fn test_integrate_simpson_constant() {
        let t = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let i = vec![5.0, 5.0, 5.0, 5.0, 5.0];
        let q = ChargeIntegrator::integrate_simpson(&t, &i);
        assert!((q - 20.0).abs() < EPSILON);
    }

    #[test]
    fn test_cumulative_charge() {
        let t = vec![0.0, 1.0, 2.0, 3.0];
        let i = vec![1.0, 1.0, 1.0, 1.0];
        let cum = ChargeIntegrator::cumulative_charge(&t, &i);
        assert_eq!(cum.len(), 4);
        assert!((cum[0] - 0.0).abs() < EPSILON);
        assert!((cum[1] - 1.0).abs() < EPSILON);
        assert!((cum[2] - 2.0).abs() < EPSILON);
        assert!((cum[3] - 3.0).abs() < EPSILON);
    }

    #[test]
    fn test_cumulative_charge_ramp() {
        let t = vec![0.0, 1.0, 2.0];
        let i = vec![0.0, 1.0, 2.0];
        let cum = ChargeIntegrator::cumulative_charge(&t, &i);
        assert!((cum[0] - 0.0).abs() < EPSILON);
        assert!((cum[1] - 0.5).abs() < EPSILON); // trapezoid: (0+1)/2 * 1
        assert!((cum[2] - 2.0).abs() < EPSILON); // + (1+2)/2 * 1
    }

    #[test]
    fn test_noise_floor_charge() {
        // sigma_I = 1e-6 A, T = 1s, fs = 1000 Hz
        // N = 1000, dt = 0.001
        // sigma_Q = 1e-6 * 0.001 * sqrt(1000) = 1e-6 * 0.001 * 31.62 = 3.162e-8
        let nf = ChargeIntegrator::noise_floor_charge(1e-6, 1.0, 1000.0);
        assert!((nf - 3.162e-8).abs() < 1e-10, "nf = {}", nf);
    }

    // ===== CoulometrySimulator tests =====

    #[test]
    fn test_simulate_cottrell() {
        let ca = CoulometrySimulator::simulate_cottrell(1, 1.0, 1e-5, 1e-6, 10.0);
        assert_eq!(ca.time_s.len(), 1000);
        assert_eq!(ca.current_a.len(), 1000);
        // Current should be positive and decreasing
        assert!(ca.current_a[0] > ca.current_a[999]);
        assert!(ca.current_a[0] > 0.0);
    }

    #[test]
    fn test_simulate_cottrell_magnitude() {
        let ca = CoulometrySimulator::simulate_cottrell(1, 1.0, 1e-5, 1e-6, 10.0);
        // At t=1s: I = F * 1.0 * sqrt(1e-5) * 1e-6 / sqrt(pi*1)
        let expected = FARADAY_CONSTANT * 1.0 * (1e-5_f64).sqrt() * 1e-6 / (PI * 1.0_f64).sqrt();
        // Find the point closest to t=1s
        let idx = ca.time_s.iter().position(|&t| t >= 1.0).unwrap();
        let actual = ca.current_a[idx];
        let t = ca.time_s[idx];
        let expected_at_t = FARADAY_CONSTANT * 1.0 * (1e-5_f64).sqrt() * 1e-6 / (PI * t).sqrt();
        assert!((actual - expected_at_t).abs() / expected_at_t < 0.001);
        let _ = expected; // suppress warning
    }

    #[test]
    fn test_simulate_battery_cycle() {
        let (charge, discharge) =
            CoulometrySimulator::simulate_battery_cycle(100.0, (3.0, 4.2), 50);
        assert_eq!(charge.capacity_mah.len(), 50);
        assert_eq!(discharge.voltage_v.len(), 50);
        // Charge voltage should increase
        assert!(charge.voltage_v[49] > charge.voltage_v[0]);
        // Discharge voltage should decrease
        assert!(discharge.voltage_v[49] < discharge.voltage_v[0]);
    }

    #[test]
    fn test_simulate_battery_cycle_endpoints() {
        let (charge, discharge) =
            CoulometrySimulator::simulate_battery_cycle(100.0, (3.0, 4.2), 100);
        assert!((charge.voltage_v[0] - 3.0).abs() < EPSILON);
        assert!((charge.voltage_v[99] - 4.2).abs() < EPSILON);
        assert!((discharge.voltage_v[0] - 4.2).abs() < EPSILON);
        assert!((discharge.voltage_v[99] - 3.0).abs() < EPSILON);
    }

    #[test]
    fn test_simulate_cycling() {
        let cycles = CoulometrySimulator::simulate_cycling(100.0, 0.5, 10);
        assert_eq!(cycles.len(), 10);
        assert_eq!(cycles[0].cycle_number, 1);
        assert!((cycles[0].discharge_capacity_mah - 100.0).abs() < EPSILON);
        assert!((cycles[9].discharge_capacity_mah - 95.5).abs() < EPSILON);
    }

    #[test]
    fn test_simulate_cycling_no_fade() {
        let cycles = CoulometrySimulator::simulate_cycling(100.0, 0.0, 5);
        for c in &cycles {
            assert!((c.discharge_capacity_mah - 100.0).abs() < EPSILON);
        }
    }

    #[test]
    fn test_simulate_cycling_floor_zero() {
        let cycles = CoulometrySimulator::simulate_cycling(10.0, 5.0, 5);
        assert!((cycles[2].discharge_capacity_mah - 0.0).abs() < EPSILON);
        assert!((cycles[4].discharge_capacity_mah - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_add_noise() {
        let ca = CoulometrySimulator::simulate_cottrell(1, 1.0, 1e-5, 1e-6, 10.0);
        let noisy = CoulometrySimulator::add_noise(&ca, 1e-9);
        assert_eq!(noisy.time_s.len(), ca.time_s.len());
        // Times should be unchanged
        assert!((noisy.time_s[0] - ca.time_s[0]).abs() < EPSILON);
        // Currents should be different but close
        let diff: f64 = noisy
            .current_a
            .iter()
            .zip(ca.current_a.iter())
            .map(|(a, b)| (a - b).abs())
            .sum::<f64>()
            / ca.current_a.len() as f64;
        assert!(diff < 1e-7, "mean diff = {}", diff);
    }

    // ===== Helper function tests =====

    #[test]
    fn test_linear_regression_perfect() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![2.0, 5.0, 8.0, 11.0, 14.0];
        let (slope, intercept, r_sq) = linear_regression(&x, &y);
        assert!((slope - 3.0).abs() < EPSILON);
        assert!((intercept - 2.0).abs() < EPSILON);
        assert!((r_sq - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_linear_regression_flat() {
        let x = vec![0.0, 1.0, 2.0];
        let y = vec![5.0, 5.0, 5.0];
        let (slope, intercept, r_sq) = linear_regression(&x, &y);
        assert!((slope).abs() < EPSILON);
        assert!((intercept - 5.0).abs() < EPSILON);
        assert!((r_sq - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_interpolate_at_exact() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![10.0, 20.0, 30.0, 40.0];
        assert!((interpolate_at(&x, &y, 1.0).unwrap() - 20.0).abs() < EPSILON);
    }

    #[test]
    fn test_interpolate_at_between() {
        let x = vec![0.0, 1.0, 2.0];
        let y = vec![0.0, 10.0, 20.0];
        assert!((interpolate_at(&x, &y, 0.5).unwrap() - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_interpolate_at_clamp() {
        let x = vec![0.0, 1.0];
        let y = vec![5.0, 10.0];
        assert!((interpolate_at(&x, &y, -1.0).unwrap() - 5.0).abs() < EPSILON);
        assert!((interpolate_at(&x, &y, 2.0).unwrap() - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_interpolate_at_empty() {
        assert!(interpolate_at(&[], &[], 1.0).is_none());
    }

    #[test]
    fn test_integrate_trapezoidal_single_point() {
        let t = vec![0.0];
        let i = vec![1.0];
        assert!((integrate_trapezoidal(&t, &i)).abs() < EPSILON);
    }

    #[test]
    fn test_simpson_vs_trapezoidal_accuracy() {
        // For a quadratic, Simpson should be more accurate than trapezoidal
        // f(x) = x^2, integral from 0 to 4 = 64/3 = 21.333
        let n = 5; // odd number of points
        let t: Vec<f64> = (0..n).map(|i| i as f64).collect();
        let y: Vec<f64> = t.iter().map(|&x| x * x).collect();
        let q_trap = integrate_trapezoidal(&t, &y);
        let q_simp = integrate_simpson(&t, &y);
        let exact = 64.0 / 3.0;
        assert!(
            (q_simp - exact).abs() <= (q_trap - exact).abs(),
            "Simpson {} vs Trapezoidal {} for exact {}",
            q_simp,
            q_trap,
            exact
        );
    }

    // ===== Round-trip and integration tests =====

    #[test]
    fn test_faraday_roundtrip_moles() {
        // Deposit 2 moles of Cu2+ (z=2) then convert back
        let n_moles = 2.0;
        let z = 2;
        let q = n_moles * z as f64 * FARADAY_CONSTANT;
        let n_back = FaradayCalculator::moles_from_charge(q, z);
        assert!((n_back - n_moles).abs() < 1e-8);
    }

    #[test]
    fn test_full_battery_analysis_pipeline() {
        // Simulate -> analyze -> predict
        let cycles = CoulometrySimulator::simulate_cycling(100.0, 0.2, 50);
        let cycler = BatteryCycler::new(cycles.clone());

        // Check CE
        let ce = BatteryCycler::coulombic_efficiency(&cycler.cycles[0]);
        assert!(ce > 98.0 && ce < 100.0);

        // Check fade
        let fade = BatteryCycler::capacity_fade_rate(&cycler.cycles);
        assert!(fade > 0.15 && fade < 0.25, "fade = {}", fade);

        // Predict life to 80%
        let life = BatteryCycler::cycle_life_prediction(&cycler.cycles, 80.0);
        assert!(life > 80 && life < 120, "life = {}", life);
    }

    #[test]
    fn test_charge_discharge_hysteresis_consistency() {
        let (charge, discharge) =
            CoulometrySimulator::simulate_battery_cycle(100.0, (3.0, 4.2), 100);
        let hyst = GalvanostaticProfile::voltage_hysteresis(&charge, &discharge);
        // Simulated profiles have some built-in hysteresis from the S-curve
        assert!(hyst >= 0.0, "hysteresis should be non-negative: {}", hyst);
    }

    #[test]
    fn test_kf_water_mass_consistency() {
        // Check that water_mass_ug and water_content_ppm are consistent
        let charge_mc = 1000.0;
        let sample_mass_g = 5.0;
        let water_ug = KarlFischerTitrator::water_mass_ug(charge_mc);
        let ppm = KarlFischerTitrator::water_content_ppm(charge_mc, sample_mass_g);
        // ppm = water_ug / (sample_mass_g * 1e6) * 1e6 = water_ug / sample_mass_g
        // Actually: ppm = (water_mg/1000) / sample_mass_g * 1e6
        //         = water_ug / 1e6 / sample_mass_g * 1e6
        //         = water_ug / sample_mass_g
        let expected_ppm = water_ug / (sample_mass_g * 1e6) * 1e6;
        assert!((ppm - expected_ppm).abs() < 0.01, "ppm={}, expected={}", ppm, expected_ppm);
    }

    #[test]
    fn test_rate_capability_ragone_sorted() {
        let energies = vec![50.0, 200.0, 100.0, 150.0];
        let powers = vec![5000.0, 100.0, 1000.0, 500.0];
        let rp = RateCapabilityAnalyzer::ragone_plot(&energies, &powers);
        // Should be sorted by energy ascending
        for i in 0..rp.len() - 1 {
            assert!(rp[i].0 <= rp[i + 1].0);
        }
    }

    #[test]
    fn test_pulse_analysis_no_polarization() {
        // If voltage drops instantly and stays constant -> no polarization
        let time = vec![0.0, 0.001, 1.0, 10.0];
        let voltage = vec![4.0, 3.8, 3.8, 3.8];
        let result = ImpedanceFromPulse::pulse_analysis(&time, &voltage, 1.0);
        assert!((result.r_ohmic - 0.2).abs() < EPSILON);
        assert!((result.r_polarization - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_simpson_single_interval() {
        let t = vec![0.0, 1.0];
        let i = vec![1.0, 3.0];
        let q = ChargeIntegrator::integrate_simpson(&t, &i);
        assert!((q - 2.0).abs() < EPSILON); // falls back to trapezoidal
    }
}

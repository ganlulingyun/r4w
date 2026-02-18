// trace:FR-ZETA | ai:claude
//! # Zeta Potential Analyzer
//!
//! Electrophoretic light scattering (ELS) data analysis for measuring zeta potential
//! of colloidal particles. Includes electrophoretic mobility calculation, DLVO stability
//! analysis, and particle charge characterization.
//!
//! ## Physics Background
//!
//! - **Electrophoretic mobility**: µ = v/E (velocity per field strength)
//! - **Smoluchowski**: ζ = µη/ε (thin double layer, κa >> 1)
//! - **Hückel**: ζ = 3µη/(2ε) (thick double layer, κa << 1)
//! - **Debye length**: κ⁻¹ = √(ε₀εᵣkT/(2NAe²I)) ≈ 0.304/√I nm (water, 25°C)
//! - **DLVO**: V_total = V_attraction + V_repulsion
//! - **IEP**: pH where ζ = 0 mV; stability requires |ζ| > 25–30 mV

use std::f64::consts::PI;

// ─── Physical constants ────────────────────────────────────────────────────────

/// Boltzmann constant (J/K)
const K_B: f64 = 1.380649e-23;
/// Avogadro's number (1/mol)
const N_A: f64 = 6.02214076e23;
/// Elementary charge (C)
const E_CHARGE: f64 = 1.602176634e-19;
/// Vacuum permittivity (F/m)
const EPSILON_0: f64 = 8.8541878128e-12;

// ─── Enums ─────────────────────────────────────────────────────────────────────

/// Colloidal stability classification based on DLVO energy barrier.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StabilityClass {
    /// Energy barrier > 15 kT – kinetically stable
    Stable,
    /// Energy barrier 5–15 kT – marginal stability
    Marginal,
    /// Energy barrier < 5 kT – rapid coagulation expected
    Unstable,
}

/// Surface charge sign inferred from zeta potential.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChargeSign {
    Positive,
    Negative,
    Neutral,
}

/// Sample conductivity status for measurement quality.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConductivityStatus {
    /// < 0.1 mS/cm – may be unreliable
    Low,
    /// 0.1–5 mS/cm – optimal range
    Optimal,
    /// 5–20 mS/cm – acceptable but electrode effects possible
    High,
    /// > 20 mS/cm – Joule heating likely, measurement unreliable
    TooHigh,
}

// ─── 1. Electrophoretic Mobility ───────────────────────────────────────────────

/// Electrophoretic mobility measurement from frequency shift or velocity data.
pub struct ElectrophoreticMobility {
    frequency_shifts_hz: Vec<f64>,
    field_v_per_m: f64,
}

impl ElectrophoreticMobility {
    /// Create from measured frequency shifts and applied electric field.
    pub fn new(frequency_shifts_hz: Vec<f64>, field_v_per_m: f64) -> Self {
        Self {
            frequency_shifts_hz,
            field_v_per_m,
        }
    }

    /// Mean electrophoretic mobility in µm·cm/(V·s).
    ///
    /// Uses µ = Δf / (E × q_scattering) where q_scattering is approximated
    /// from a typical 173° backscatter geometry at 633 nm laser wavelength.
    pub fn mobility_um_cm_per_vs(&self) -> f64 {
        if self.frequency_shifts_hz.is_empty() || self.field_v_per_m.abs() < 1e-30 {
            return 0.0;
        }
        // Scattering vector magnitude for 173° backscatter, λ = 633 nm, n = 1.33 (water)
        let lambda = 633e-9; // m
        let n_medium = 1.33;
        let theta = 173.0_f64.to_radians();
        let q_scattering = 4.0 * PI * n_medium * (theta / 2.0).sin() / lambda;

        let mean_shift: f64 =
            self.frequency_shifts_hz.iter().sum::<f64>() / self.frequency_shifts_hz.len() as f64;
        // mobility in m²/(V·s), then convert to µm·cm/(V·s): multiply by 1e8
        let mobility_si = mean_shift / (self.field_v_per_m * q_scattering);
        mobility_si * 1e8
    }

    /// Compute mobility directly from velocity and field.
    ///
    /// µ = v / E  where v is in µm/s and E in V/cm, result in µm·cm/(V·s).
    pub fn from_velocity(velocity_um_per_s: f64, field_v_per_cm: f64) -> f64 {
        if field_v_per_cm.abs() < 1e-30 {
            return 0.0;
        }
        velocity_um_per_s / field_v_per_cm
    }

    /// Compute mobility distribution from a set of frequency shifts.
    ///
    /// Returns Vec of (mobility in µm·cm/(V·s), normalized weight).
    pub fn distribution(shifts: &[f64], field: f64) -> Vec<(f64, f64)> {
        if shifts.is_empty() || field.abs() < 1e-30 {
            return Vec::new();
        }
        let lambda = 633e-9;
        let n_medium = 1.33;
        let theta = 173.0_f64.to_radians();
        let q_scattering = 4.0 * PI * n_medium * (theta / 2.0).sin() / lambda;

        let mobilities: Vec<f64> = shifts
            .iter()
            .map(|&s| (s / (field * q_scattering)) * 1e8)
            .collect();

        // Simple histogram-like distribution: bin and normalize
        let n = mobilities.len() as f64;
        let weight = 1.0 / n;
        mobilities.into_iter().map(|m| (m, weight)).collect()
    }
}

// ─── 2. Zeta Potential Calculator ──────────────────────────────────────────────

/// Convert electrophoretic mobility to zeta potential using various models.
pub struct ZetaPotentialCalculator;

impl ZetaPotentialCalculator {
    /// Smoluchowski equation: ζ = µη/ε   (κa >> 1, thin double layer).
    ///
    /// * `mobility` – µ in m²/(V·s)
    /// * `viscosity` – η in Pa·s
    /// * `permittivity` – ε = ε₀εᵣ in F/m
    ///
    /// Returns zeta potential in volts.
    pub fn smoluchowski(mobility: f64, viscosity: f64, permittivity: f64) -> f64 {
        if permittivity.abs() < 1e-40 {
            return 0.0;
        }
        mobility * viscosity / permittivity
    }

    /// Hückel equation: ζ = 3µη/(2ε)   (κa << 1, thick double layer).
    pub fn huckel(mobility: f64, viscosity: f64, permittivity: f64) -> f64 {
        if permittivity.abs() < 1e-40 {
            return 0.0;
        }
        3.0 * mobility * viscosity / (2.0 * permittivity)
    }

    /// Henry equation: ζ = 3µη/(2ε·f(κa)).
    pub fn henry(mobility: f64, viscosity: f64, permittivity: f64, kappa_a: f64) -> f64 {
        let f_ka = Self::henry_function(kappa_a);
        if (permittivity * f_ka).abs() < 1e-40 {
            return 0.0;
        }
        3.0 * mobility * viscosity / (2.0 * permittivity * f_ka)
    }

    /// Henry function f(κa) interpolation between Hückel (1.0) and Smoluchowski (1.5).
    ///
    /// Uses Ohshima's approximation for convenience.
    pub fn henry_function(kappa_a: f64) -> f64 {
        Self::ohshima_approximation(kappa_a)
    }

    /// Ohshima's approximation for Henry's function:
    ///
    /// f(κa) ≈ 1 + 1 / [2(1 + 2.5/(κa(1 + 2e^{-κa})))]
    pub fn ohshima_approximation(kappa_a: f64) -> f64 {
        if kappa_a < 1e-12 {
            return 1.0; // Hückel limit
        }
        let inner = kappa_a * (1.0 + 2.0 * (-kappa_a).exp());
        let denom = 2.0 * (1.0 + 2.5 / inner);
        1.0 + 1.0 / denom
    }
}

// ─── 3. Debye Length ───────────────────────────────────────────────────────────

/// Electric double layer thickness (Debye length) calculations.
pub struct DebyeLength;

impl DebyeLength {
    /// Calculate Debye length κ⁻¹ in metres.
    ///
    /// κ⁻¹ = √(ε₀ εᵣ k_B T / (2 N_A e² I))
    ///
    /// * `ionic_strength_mol_per_l` – I in mol/L
    /// * `temperature_k` – T in kelvin
    /// * `permittivity_rel` – εᵣ (relative permittivity, dimensionless)
    ///
    /// Returns Debye length in nanometres.
    pub fn calculate(ionic_strength_mol_per_l: f64, temperature_k: f64, permittivity_rel: f64) -> f64 {
        if ionic_strength_mol_per_l <= 0.0 {
            return f64::INFINITY;
        }
        // Convert mol/L to mol/m³ (× 1000)
        let i_m3 = ionic_strength_mol_per_l * 1000.0;
        let numerator = EPSILON_0 * permittivity_rel * K_B * temperature_k;
        let denominator = 2.0 * N_A * E_CHARGE * E_CHARGE * i_m3;
        let kappa_inv_m = (numerator / denominator).sqrt();
        kappa_inv_m * 1e9 // convert to nm
    }

    /// Approximate Debye length from conductivity (S/m).
    ///
    /// Empirical relation for aqueous solutions at 25 °C:
    /// κ⁻¹ ≈ 0.304 / √I   and   I ≈ conductivity / 0.013
    ///
    /// Returns Debye length in nm.
    pub fn from_conductivity(conductivity_s_per_m: f64) -> f64 {
        if conductivity_s_per_m <= 0.0 {
            return f64::INFINITY;
        }
        // conductivity in S/m; typical factor for monovalent salt
        // I (mol/L) ≈ conductivity(S/m) / 1.3   (rough)
        let ionic_strength = conductivity_s_per_m / 1.3;
        if ionic_strength <= 0.0 {
            return f64::INFINITY;
        }
        0.304 / ionic_strength.sqrt()
    }

    /// Compute κa = particle_radius / Debye_length.
    pub fn kappa_a(debye_length_nm: f64, particle_radius_nm: f64) -> f64 {
        if debye_length_nm <= 0.0 || debye_length_nm.is_infinite() {
            return 0.0;
        }
        particle_radius_nm / debye_length_nm
    }

    /// Ionic strength from ion concentrations: I = ½ Σ cᵢ zᵢ².
    ///
    /// `concentrations` – slice of (concentration_mol_per_L, charge_valence).
    pub fn ionic_strength_from_concentration(concentrations: &[(f64, i32)]) -> f64 {
        let sum: f64 = concentrations
            .iter()
            .map(|&(c, z)| c * (z as f64) * (z as f64))
            .sum();
        0.5 * sum
    }
}

// ─── 4. DLVO Theory ────────────────────────────────────────────────────────────

/// Derjaguin-Landau-Verwey-Overbeek (DLVO) colloidal stability analysis.
pub struct DlvoTheory;

impl DlvoTheory {
    /// Van der Waals attraction between two identical spheres (Derjaguin approx.).
    ///
    /// V_A = -A R / (12 d)   for d << R.
    ///
    /// * `hamaker` – Hamaker constant (J), typically 1e-21 to 1e-19
    /// * `distance_nm` – surface-to-surface distance (nm)
    /// * `radius_nm` – particle radius (nm)
    ///
    /// Returns energy in joules (negative = attractive).
    pub fn van_der_waals_attraction(hamaker: f64, distance_nm: f64, radius_nm: f64) -> f64 {
        if distance_nm <= 0.0 {
            return f64::NEG_INFINITY;
        }
        let d_m = distance_nm * 1e-9;
        let r_m = radius_nm * 1e-9;
        -hamaker * r_m / (12.0 * d_m)
    }

    /// Electrostatic double-layer repulsion between two identical spheres.
    ///
    /// V_R = 2π ε R ψ₀² ln(1 + e^{-κd})
    ///
    /// Uses constant potential approximation.
    ///
    /// * `zeta_mv` – zeta potential in millivolts
    /// * `radius_nm` – particle radius (nm)
    /// * `kappa` – inverse Debye length (1/nm)
    /// * `distance_nm` – surface separation (nm)
    /// * `permittivity` – ε = ε₀ εᵣ (F/m)
    ///
    /// Returns energy in joules (positive = repulsive).
    pub fn electrostatic_repulsion(
        zeta_mv: f64,
        radius_nm: f64,
        kappa: f64,
        distance_nm: f64,
        permittivity: f64,
    ) -> f64 {
        let psi0 = zeta_mv * 1e-3; // V
        let r_m = radius_nm * 1e-9;
        let d_m = distance_nm * 1e-9;
        let kappa_m = kappa * 1e9; // convert from 1/nm to 1/m
        let exponent = -kappa_m * d_m;
        // Clamp to avoid overflow
        let exp_val = if exponent < -500.0 { 0.0 } else { exponent.exp() };
        2.0 * PI * permittivity * r_m * psi0 * psi0 * (1.0 + exp_val).ln()
    }

    /// Total DLVO interaction energy at multiple distances.
    ///
    /// Returns Vec of (distance_nm, V_total in joules).
    pub fn total_interaction(
        hamaker: f64,
        zeta_mv: f64,
        radius_nm: f64,
        kappa: f64,
        distances: &[f64],
        permittivity: f64,
    ) -> Vec<(f64, f64)> {
        distances
            .iter()
            .map(|&d| {
                let va = Self::van_der_waals_attraction(hamaker, d, radius_nm);
                let vr =
                    Self::electrostatic_repulsion(zeta_mv, radius_nm, kappa, d, permittivity);
                (d, va + vr)
            })
            .collect()
    }

    /// Find the energy barrier (maximum) in a DLVO interaction curve.
    ///
    /// Returns the maximum V_total in joules, or 0.0 if monotonically attractive.
    pub fn energy_barrier(interaction_curve: &[(f64, f64)]) -> f64 {
        interaction_curve
            .iter()
            .map(|&(_, v)| v)
            .fold(f64::NEG_INFINITY, f64::max)
            .max(0.0)
    }

    /// Predict colloidal stability from the DLVO energy barrier in kT units.
    pub fn predict_stability(energy_barrier_kt: f64) -> StabilityClass {
        if energy_barrier_kt > 15.0 {
            StabilityClass::Stable
        } else if energy_barrier_kt >= 5.0 {
            StabilityClass::Marginal
        } else {
            StabilityClass::Unstable
        }
    }
}

// ─── 5. Stability Analyzer ─────────────────────────────────────────────────────

/// Stability ratio and critical coagulation concentration analysis.
pub struct StabilityAnalyzer;

impl StabilityAnalyzer {
    /// Estimate critical coagulation concentration (CCC) from zeta potential vs
    /// concentration data.  CCC is the concentration where |ζ| first drops below
    /// a threshold (~25 mV or where the curve flattens).
    ///
    /// Finds the concentration where |ζ| crosses below 25 mV by linear interpolation.
    pub fn critical_coagulation_concentration(
        zeta_values: &[f64],
        concentrations: &[f64],
    ) -> f64 {
        let threshold = 25.0; // mV
        if zeta_values.len() != concentrations.len() || zeta_values.is_empty() {
            return 0.0;
        }
        // Walk through sorted by concentration (assume already sorted ascending)
        for i in 1..zeta_values.len() {
            let z_prev = zeta_values[i - 1].abs();
            let z_curr = zeta_values[i].abs();
            if z_prev >= threshold && z_curr < threshold {
                // Linear interpolation
                let frac = (z_prev - threshold) / (z_prev - z_curr);
                return concentrations[i - 1]
                    + frac * (concentrations[i] - concentrations[i - 1]);
            }
        }
        // If never crossed, return last concentration or 0
        if zeta_values.last().map(|z| z.abs() < threshold).unwrap_or(false) {
            *concentrations.first().unwrap_or(&0.0)
        } else {
            *concentrations.last().unwrap_or(&0.0)
        }
    }

    /// Schulze-Hardy rule: CCC ∝ z⁻⁶.
    ///
    /// Returns the relative CCC for valence `z` compared to z=1.
    pub fn schulze_hardy_rule(valence: i32) -> f64 {
        let z = valence.abs() as f64;
        if z < 1e-12 {
            return f64::INFINITY;
        }
        1.0 / z.powi(6)
    }

    /// Stability ratio W = k_fast / k_slow.
    ///
    /// W = 1 for fast (diffusion-limited) aggregation.
    pub fn stability_ratio(fast_rate: f64, measured_rate: f64) -> f64 {
        if measured_rate.abs() < 1e-30 {
            return f64::INFINITY;
        }
        fast_rate / measured_rate
    }

    /// Fuchs stability ratio approximation:
    ///
    /// W ≈ (1/(2κa)) exp(V_max / kT)
    pub fn fuchs_stability_ratio(kappa_a: f64, energy_barrier_kt: f64) -> f64 {
        if kappa_a <= 0.0 {
            return f64::INFINITY;
        }
        (1.0 / (2.0 * kappa_a)) * energy_barrier_kt.exp()
    }
}

// ─── 6. pH Titration ───────────────────────────────────────────────────────────

/// Zeta potential versus pH analysis.
pub struct PhTitration {
    ph_values: Vec<f64>,
    zeta_mv: Vec<f64>,
}

impl PhTitration {
    /// Create from matched pH and zeta-potential vectors.
    pub fn new(ph_values: Vec<f64>, zeta_mv: Vec<f64>) -> Self {
        assert_eq!(ph_values.len(), zeta_mv.len(), "pH and zeta vectors must have equal length");
        Self { ph_values, zeta_mv }
    }

    /// Isoelectric point (IEP): pH where ζ = 0 mV.
    ///
    /// Uses linear interpolation between the two points that straddle zero.
    pub fn isoelectric_point(&self) -> f64 {
        for i in 1..self.zeta_mv.len() {
            let z_prev = self.zeta_mv[i - 1];
            let z_curr = self.zeta_mv[i];
            if (z_prev >= 0.0 && z_curr <= 0.0) || (z_prev <= 0.0 && z_curr >= 0.0) {
                let dz = z_prev - z_curr;
                if dz.abs() < 1e-30 {
                    return (self.ph_values[i - 1] + self.ph_values[i]) / 2.0;
                }
                let frac = z_prev / dz;
                return self.ph_values[i - 1]
                    + frac * (self.ph_values[i] - self.ph_values[i - 1]);
            }
        }
        f64::NAN // no crossing found
    }

    /// Point of zero charge (PZC) – same as IEP for simple oxides.
    pub fn point_of_zero_charge(&self) -> f64 {
        self.isoelectric_point()
    }

    /// pH range where |ζ| exceeds threshold (mV), indicating stability.
    ///
    /// Returns (pH_low, pH_high) or (NaN, NaN) if never exceeded.
    pub fn stable_ph_range(&self, threshold_mv: f64) -> (f64, f64) {
        let mut low = f64::NAN;
        let mut high = f64::NAN;
        for i in 0..self.ph_values.len() {
            if self.zeta_mv[i].abs() >= threshold_mv {
                if low.is_nan() {
                    low = self.ph_values[i];
                }
                high = self.ph_values[i];
            }
        }
        (low, high)
    }

    /// Surface charge sign at the given pH, interpolated from the titration curve.
    pub fn surface_charge_sign(&self, ph: f64) -> ChargeSign {
        let zeta = interpolate_linear(&self.ph_values, &self.zeta_mv, ph);
        if zeta > 1.0 {
            ChargeSign::Positive
        } else if zeta < -1.0 {
            ChargeSign::Negative
        } else {
            ChargeSign::Neutral
        }
    }
}

// ─── 7. Salt Titration ─────────────────────────────────────────────────────────

/// Zeta potential versus ionic strength analysis.
pub struct SaltTitration {
    ionic_strengths: Vec<f64>,
    zeta_mv: Vec<f64>,
}

impl SaltTitration {
    pub fn new(ionic_strengths: Vec<f64>, zeta_mv: Vec<f64>) -> Self {
        assert_eq!(
            ionic_strengths.len(),
            zeta_mv.len(),
            "ionic strength and zeta vectors must match"
        );
        Self {
            ionic_strengths,
            zeta_mv,
        }
    }

    /// Charge screening rate: dζ / d(log₁₀ I), estimated by linear regression.
    pub fn charge_screening_rate(&self) -> f64 {
        if self.ionic_strengths.len() < 2 {
            return 0.0;
        }
        let log_is: Vec<f64> = self
            .ionic_strengths
            .iter()
            .filter(|&&x| x > 0.0)
            .map(|x| x.log10())
            .collect();
        let n = log_is.len().min(self.zeta_mv.len());
        if n < 2 {
            return 0.0;
        }
        let zetas = &self.zeta_mv[..n];
        linear_regression_slope(&log_is[..n], zetas)
    }

    /// Extrapolate zeta potential to zero ionic strength (surface potential estimate).
    ///
    /// Uses linear regression on log(I) vs ζ and evaluates at I → 0 (log I → -∞ capped).
    /// In practice, extrapolates to the y-intercept of ζ vs I.
    pub fn extrapolate_to_zero_salt(&self) -> f64 {
        if self.ionic_strengths.len() < 2 {
            return self.zeta_mv.first().copied().unwrap_or(0.0);
        }
        // Simple linear extrapolation: fit ζ = a + b*I, return a
        let (intercept, _slope) =
            linear_regression(&self.ionic_strengths, &self.zeta_mv);
        intercept
    }

    /// Compression of double layer: change in zeta when ionic strength changes.
    ///
    /// Returns Δζ / Δ(log I).
    pub fn compression_of_double_layer(
        is1: f64,
        is2: f64,
        zeta1: f64,
        zeta2: f64,
    ) -> f64 {
        let d_log_i = is2.log10() - is1.log10();
        if d_log_i.abs() < 1e-30 {
            return 0.0;
        }
        (zeta2 - zeta1) / d_log_i
    }
}

// ─── 8. Particle Charge Calculator ─────────────────────────────────────────────

/// Surface charge density calculations.
pub struct ParticleChargeCalculator;

impl ParticleChargeCalculator {
    /// Linearized Poisson-Boltzmann surface charge density:
    ///
    /// σ = ε κ ζ      (valid for low ζ, < ~25 mV)
    ///
    /// * `zeta_mv` – zeta potential in mV
    /// * `kappa` – inverse Debye length in 1/m
    /// * `permittivity` – ε = ε₀ εᵣ in F/m
    ///
    /// Returns σ in C/m².
    pub fn surface_charge_density(zeta_mv: f64, kappa: f64, permittivity: f64) -> f64 {
        let zeta_v = zeta_mv * 1e-3;
        permittivity * kappa * zeta_v
    }

    /// Gouy-Chapman exact surface charge density (symmetric 1:1 electrolyte):
    ///
    /// σ = √(8 ε ε₀ k_B T n₀) sinh(e ζ / (2 k_B T))
    ///
    /// * `zeta_mv` – zeta potential in mV
    /// * `ionic_strength` – in mol/L
    /// * `temperature_k` – in kelvin
    ///
    /// Returns σ in C/m².
    pub fn gouy_chapman(zeta_mv: f64, ionic_strength: f64, temperature_k: f64) -> f64 {
        let zeta_v = zeta_mv * 1e-3;
        let eps = EPSILON_0 * 78.5; // water at 25°C
        let n0 = ionic_strength * 1000.0 * N_A; // number density (1/m³)
        let thermal = K_B * temperature_k;
        let prefactor = (8.0 * eps * thermal * n0).sqrt();
        let argument = E_CHARGE * zeta_v / (2.0 * thermal);
        prefactor * argument.sinh()
    }

    /// Total number of elementary charges on a spherical particle.
    ///
    /// N = σ × 4πR² / e
    pub fn number_of_charges(sigma_c_per_m2: f64, radius_nm: f64) -> f64 {
        let r_m = radius_nm * 1e-9;
        let area = 4.0 * PI * r_m * r_m;
        (sigma_c_per_m2 * area / E_CHARGE).abs()
    }

    /// Effective charge after Manning condensation.
    ///
    /// q_eff = q_structural / ξ   when ξ > 1, else q_structural.
    pub fn effective_charge(structural_charge: f64, manning_parameter: f64) -> f64 {
        if manning_parameter > 1.0 {
            structural_charge / manning_parameter
        } else {
            structural_charge
        }
    }
}

// ─── 9. Zeta Simulator ────────────────────────────────────────────────────────

/// Generate synthetic electrophoretic light scattering data for testing.
pub struct ZetaSimulator;

impl ZetaSimulator {
    /// Simulate a mobility distribution from a population of particles with
    /// Gaussian-distributed zeta potentials.
    ///
    /// Returns a Vec of mobility values in µm·cm/(V·s).
    pub fn simulate_mobility_distribution(
        mean_zeta_mv: f64,
        std_mv: f64,
        num_particles: usize,
        viscosity: f64,
        permittivity: f64,
    ) -> Vec<f64> {
        let mut rng = SimpleRng::new(42);
        let mut mobilities = Vec::with_capacity(num_particles);
        for _ in 0..num_particles {
            let zeta_mv = mean_zeta_mv + std_mv * rng.next_gaussian();
            let zeta_v = zeta_mv * 1e-3;
            // Smoluchowski: µ = εζ/η  (SI: m²/(V·s)), convert to µm·cm/(V·s) × 1e8
            let mu_si = permittivity * zeta_v / viscosity;
            mobilities.push(mu_si * 1e8);
        }
        mobilities
    }

    /// Simulate a pH titration curve.
    ///
    /// Uses a sigmoid model: ζ = max_zeta × tanh((IEP - pH) / 2).
    ///
    /// Returns (ph_values, zeta_values_mv).
    pub fn simulate_ph_titration(
        iep: f64,
        max_zeta: f64,
        num_points: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        let ph_start = 2.0;
        let ph_end = 12.0;
        let step = if num_points > 1 {
            (ph_end - ph_start) / (num_points - 1) as f64
        } else {
            0.0
        };
        let mut ph = Vec::with_capacity(num_points);
        let mut zeta = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let p = ph_start + i as f64 * step;
            let z = max_zeta * ((iep - p) / 2.0).tanh();
            ph.push(p);
            zeta.push(z);
        }
        (ph, zeta)
    }

    /// Simulate a DLVO interaction curve.
    ///
    /// Returns Vec of (distance_nm, V_total in joules).
    pub fn simulate_dlvo_curve(
        hamaker: f64,
        zeta_mv: f64,
        radius_nm: f64,
        kappa: f64,
    ) -> Vec<(f64, f64)> {
        let permittivity = EPSILON_0 * 78.5;
        let distances: Vec<f64> = (1..=200).map(|i| i as f64 * 0.5).collect();
        DlvoTheory::total_interaction(hamaker, zeta_mv, radius_nm, kappa, &distances, permittivity)
    }

    /// Add Gaussian noise to a data vector.
    pub fn add_noise(data: &[f64], noise_std: f64) -> Vec<f64> {
        let mut rng = SimpleRng::new(123);
        data.iter()
            .map(|&x| x + noise_std * rng.next_gaussian())
            .collect()
    }
}

// ─── 10. Quality Metrics ───────────────────────────────────────────────────────

/// Measurement quality assessment.
pub struct QualityMetrics;

impl QualityMetrics {
    /// Phase plot quality: R² of a linear fit to phase vs frequency.
    ///
    /// Good quality: R² > 0.95.
    pub fn phase_plot_quality(phase: &[f64], frequency: &[f64]) -> f64 {
        if phase.len() < 2 || phase.len() != frequency.len() {
            return 0.0;
        }
        r_squared(frequency, phase)
    }

    /// Check if photon count rate is above minimum acceptable.
    pub fn count_rate_check(count_rate: f64, min_acceptable: f64) -> bool {
        count_rate >= min_acceptable
    }

    /// Classify sample conductivity status.
    pub fn conductivity_check(conductivity_ms_per_cm: f64) -> ConductivityStatus {
        if conductivity_ms_per_cm < 0.1 {
            ConductivityStatus::Low
        } else if conductivity_ms_per_cm <= 5.0 {
            ConductivityStatus::Optimal
        } else if conductivity_ms_per_cm <= 20.0 {
            ConductivityStatus::High
        } else {
            ConductivityStatus::TooHigh
        }
    }

    /// Repeatability: mean and standard deviation of repeated measurements.
    pub fn repeatability(measurements: &[f64]) -> (f64, f64) {
        if measurements.is_empty() {
            return (0.0, 0.0);
        }
        let n = measurements.len() as f64;
        let mean = measurements.iter().sum::<f64>() / n;
        if measurements.len() == 1 {
            return (mean, 0.0);
        }
        let variance = measurements.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0);
        (mean, variance.sqrt())
    }
}

// ─── Helper Utilities ──────────────────────────────────────────────────────────

/// Simple deterministic PRNG (xorshift64).
struct SimpleRng {
    state: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 { 1 } else { seed },
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Box-Muller transform for Gaussian samples.
    fn next_gaussian(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-15);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

/// Linear interpolation on sorted (x, y) data.
fn interpolate_linear(xs: &[f64], ys: &[f64], x: f64) -> f64 {
    if xs.is_empty() {
        return 0.0;
    }
    if x <= xs[0] {
        return ys[0];
    }
    if x >= xs[xs.len() - 1] {
        return ys[ys.len() - 1];
    }
    for i in 1..xs.len() {
        if x <= xs[i] {
            let t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
            return ys[i - 1] + t * (ys[i] - ys[i - 1]);
        }
    }
    *ys.last().unwrap()
}

/// Simple linear regression: returns (intercept, slope).
fn linear_regression(xs: &[f64], ys: &[f64]) -> (f64, f64) {
    let n = xs.len().min(ys.len()) as f64;
    if n < 2.0 {
        return (ys.first().copied().unwrap_or(0.0), 0.0);
    }
    let sx: f64 = xs.iter().take(n as usize).sum();
    let sy: f64 = ys.iter().take(n as usize).sum();
    let sxx: f64 = xs.iter().take(n as usize).map(|x| x * x).sum();
    let sxy: f64 = xs
        .iter()
        .zip(ys.iter())
        .take(n as usize)
        .map(|(x, y)| x * y)
        .sum();
    let denom = n * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return (sy / n, 0.0);
    }
    let slope = (n * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / n;
    (intercept, slope)
}

/// Slope from simple linear regression.
fn linear_regression_slope(xs: &[f64], ys: &[f64]) -> f64 {
    linear_regression(xs, ys).1
}

/// Coefficient of determination R².
fn r_squared(xs: &[f64], ys: &[f64]) -> f64 {
    let n = xs.len().min(ys.len());
    if n < 2 {
        return 0.0;
    }
    let (intercept, slope) = linear_regression(xs, ys);
    let y_mean = ys.iter().take(n).sum::<f64>() / n as f64;
    let ss_tot: f64 = ys.iter().take(n).map(|y| (y - y_mean).powi(2)).sum();
    let ss_res: f64 = xs
        .iter()
        .zip(ys.iter())
        .take(n)
        .map(|(x, y)| {
            let y_pred = intercept + slope * x;
            (y - y_pred).powi(2)
        })
        .sum();
    if ss_tot.abs() < 1e-30 {
        return 1.0; // constant data
    }
    1.0 - ss_res / ss_tot
}

// ─── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const WATER_VISCOSITY: f64 = 8.9e-4; // Pa·s at 25°C
    const WATER_PERMITTIVITY_REL: f64 = 78.5;
    const WATER_PERMITTIVITY: f64 = EPSILON_0 * WATER_PERMITTIVITY_REL;
    const ROOM_TEMP: f64 = 298.15; // K

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ── ElectrophoreticMobility ──

    #[test]
    fn test_mobility_from_velocity() {
        // 100 µm/s at 10 V/cm → 10 µm·cm/(V·s)
        let mu = ElectrophoreticMobility::from_velocity(100.0, 10.0);
        assert!(approx_eq(mu, 10.0, 1e-10));
    }

    #[test]
    fn test_mobility_from_velocity_zero_field() {
        assert_eq!(ElectrophoreticMobility::from_velocity(100.0, 0.0), 0.0);
    }

    #[test]
    fn test_mobility_empty_shifts() {
        let em = ElectrophoreticMobility::new(vec![], 1000.0);
        assert_eq!(em.mobility_um_cm_per_vs(), 0.0);
    }

    #[test]
    fn test_mobility_zero_field() {
        let em = ElectrophoreticMobility::new(vec![100.0], 0.0);
        assert_eq!(em.mobility_um_cm_per_vs(), 0.0);
    }

    #[test]
    fn test_mobility_positive_shift() {
        let em = ElectrophoreticMobility::new(vec![500.0], 5000.0);
        let mu = em.mobility_um_cm_per_vs();
        // Should produce a finite, nonzero positive value
        assert!(mu.is_finite());
        assert!(mu != 0.0);
    }

    #[test]
    fn test_mobility_distribution_returns_correct_length() {
        let shifts = vec![100.0, 200.0, 300.0];
        let dist = ElectrophoreticMobility::distribution(&shifts, 5000.0);
        assert_eq!(dist.len(), 3);
    }

    #[test]
    fn test_mobility_distribution_weights_sum_to_one() {
        let shifts = vec![100.0, 200.0, 300.0, 400.0];
        let dist = ElectrophoreticMobility::distribution(&shifts, 5000.0);
        let total: f64 = dist.iter().map(|(_, w)| w).sum();
        assert!(approx_eq(total, 1.0, 1e-10));
    }

    #[test]
    fn test_mobility_distribution_empty() {
        let dist = ElectrophoreticMobility::distribution(&[], 5000.0);
        assert!(dist.is_empty());
    }

    // ── ZetaPotentialCalculator ──

    #[test]
    fn test_smoluchowski_basic() {
        // µ = 3e-8 m²/(V·s), typical for ~-40 mV in water
        let mu = 3.0e-8;
        let zeta = ZetaPotentialCalculator::smoluchowski(mu, WATER_VISCOSITY, WATER_PERMITTIVITY);
        // ζ = µη/ε ≈ 3e-8 × 8.9e-4 / 6.95e-10 ≈ 0.0384 V ≈ 38.4 mV
        let zeta_mv = zeta * 1e3;
        assert!(zeta_mv > 30.0 && zeta_mv < 50.0, "Smoluchowski zeta = {zeta_mv} mV");
    }

    #[test]
    fn test_smoluchowski_zero_permittivity() {
        assert_eq!(ZetaPotentialCalculator::smoluchowski(1e-8, 1e-3, 0.0), 0.0);
    }

    #[test]
    fn test_huckel_vs_smoluchowski() {
        let mu = 3.0e-8;
        let smol = ZetaPotentialCalculator::smoluchowski(mu, WATER_VISCOSITY, WATER_PERMITTIVITY);
        let huck = ZetaPotentialCalculator::huckel(mu, WATER_VISCOSITY, WATER_PERMITTIVITY);
        // Hückel gives 1.5× Smoluchowski
        assert!(approx_eq(huck / smol, 1.5, 1e-10));
    }

    #[test]
    fn test_henry_large_kappa_a() {
        let mu = 3.0e-8;
        // For large κa, Henry → Smoluchowski
        let smol = ZetaPotentialCalculator::smoluchowski(mu, WATER_VISCOSITY, WATER_PERMITTIVITY);
        let henry = ZetaPotentialCalculator::henry(mu, WATER_VISCOSITY, WATER_PERMITTIVITY, 1000.0);
        // Should be close to Smoluchowski
        let ratio = henry / smol;
        assert!(ratio > 0.9 && ratio < 1.15, "Henry/Smoluchowski ratio = {ratio}");
    }

    #[test]
    fn test_henry_small_kappa_a() {
        let mu = 3.0e-8;
        // For small κa, Henry → Hückel
        let huck = ZetaPotentialCalculator::huckel(mu, WATER_VISCOSITY, WATER_PERMITTIVITY);
        let henry = ZetaPotentialCalculator::henry(mu, WATER_VISCOSITY, WATER_PERMITTIVITY, 0.001);
        let ratio = henry / huck;
        assert!(ratio > 0.95 && ratio < 1.05, "Henry/Hückel ratio = {ratio}");
    }

    #[test]
    fn test_henry_function_limits() {
        // κa → 0: f(κa) → 1.0
        let f_low = ZetaPotentialCalculator::henry_function(0.0);
        assert!(approx_eq(f_low, 1.0, 0.01));

        // κa → ∞: f(κa) → 1.5
        let f_high = ZetaPotentialCalculator::henry_function(10000.0);
        assert!(f_high > 1.45 && f_high <= 1.5, "f(κa→∞) = {f_high}");
    }

    #[test]
    fn test_ohshima_monotonic() {
        let mut prev = ZetaPotentialCalculator::ohshima_approximation(0.0);
        for i in 1..=100 {
            let ka = i as f64 * 0.5;
            let f = ZetaPotentialCalculator::ohshima_approximation(ka);
            assert!(f >= prev - 1e-12, "f(κa) should be monotonically increasing");
            prev = f;
        }
    }

    #[test]
    fn test_ohshima_range() {
        for i in 0..=1000 {
            let ka = i as f64 * 0.1;
            let f = ZetaPotentialCalculator::ohshima_approximation(ka);
            assert!(f >= 1.0 - 1e-10 && f <= 1.5 + 1e-10, "f(κa={ka}) = {f} out of [1,1.5]");
        }
    }

    // ── DebyeLength ──

    #[test]
    fn test_debye_length_10mm_nacl() {
        // 10 mM NaCl at 25°C: κ⁻¹ ≈ 3.04 nm
        let dl = DebyeLength::calculate(0.01, ROOM_TEMP, WATER_PERMITTIVITY_REL);
        assert!(dl > 2.5 && dl < 3.5, "Debye length for 10mM = {dl} nm");
    }

    #[test]
    fn test_debye_length_100mm_nacl() {
        // 100 mM: κ⁻¹ ≈ 0.96 nm
        let dl = DebyeLength::calculate(0.1, ROOM_TEMP, WATER_PERMITTIVITY_REL);
        assert!(dl > 0.7 && dl < 1.3, "Debye length for 100mM = {dl} nm");
    }

    #[test]
    fn test_debye_length_zero_ionic_strength() {
        let dl = DebyeLength::calculate(0.0, ROOM_TEMP, WATER_PERMITTIVITY_REL);
        assert!(dl.is_infinite());
    }

    #[test]
    fn test_debye_length_from_conductivity() {
        let dl = DebyeLength::from_conductivity(0.1);
        assert!(dl > 0.0 && dl.is_finite());
    }

    #[test]
    fn test_debye_length_from_conductivity_zero() {
        assert!(DebyeLength::from_conductivity(0.0).is_infinite());
    }

    #[test]
    fn test_kappa_a() {
        // Debye = 10 nm, radius = 100 nm → κa = 10
        assert!(approx_eq(DebyeLength::kappa_a(10.0, 100.0), 10.0, 1e-10));
    }

    #[test]
    fn test_kappa_a_infinite_debye() {
        assert_eq!(DebyeLength::kappa_a(f64::INFINITY, 100.0), 0.0);
    }

    #[test]
    fn test_ionic_strength_nacl() {
        // NaCl 0.1 M: I = 0.5*(0.1*1 + 0.1*1) = 0.1
        let is = DebyeLength::ionic_strength_from_concentration(&[(0.1, 1), (0.1, -1)]);
        assert!(approx_eq(is, 0.1, 1e-10));
    }

    #[test]
    fn test_ionic_strength_cacl2() {
        // CaCl₂ 0.1 M: I = 0.5*(0.1*4 + 0.2*1) = 0.3
        let is = DebyeLength::ionic_strength_from_concentration(&[(0.1, 2), (0.2, -1)]);
        assert!(approx_eq(is, 0.3, 1e-10));
    }

    // ── DlvoTheory ──

    #[test]
    fn test_vdw_attraction_negative() {
        let va = DlvoTheory::van_der_waals_attraction(1e-20, 5.0, 100.0);
        assert!(va < 0.0, "VdW should be attractive (negative)");
    }

    #[test]
    fn test_vdw_attraction_increases_with_closer_distance() {
        let va_far = DlvoTheory::van_der_waals_attraction(1e-20, 10.0, 100.0);
        let va_close = DlvoTheory::van_der_waals_attraction(1e-20, 5.0, 100.0);
        assert!(va_close < va_far, "|V_A(5nm)| > |V_A(10nm)|");
    }

    #[test]
    fn test_vdw_zero_distance() {
        let va = DlvoTheory::van_der_waals_attraction(1e-20, 0.0, 100.0);
        assert!(va.is_infinite() && va < 0.0);
    }

    #[test]
    fn test_electrostatic_repulsion_positive() {
        let vr = DlvoTheory::electrostatic_repulsion(-30.0, 100.0, 0.1, 5.0, WATER_PERMITTIVITY);
        assert!(vr > 0.0, "Electrostatic repulsion should be positive, got {vr}");
    }

    #[test]
    fn test_electrostatic_decays_with_distance() {
        let vr_close =
            DlvoTheory::electrostatic_repulsion(-30.0, 100.0, 0.1, 2.0, WATER_PERMITTIVITY);
        let vr_far =
            DlvoTheory::electrostatic_repulsion(-30.0, 100.0, 0.1, 50.0, WATER_PERMITTIVITY);
        assert!(vr_close > vr_far);
    }

    #[test]
    fn test_total_interaction_length() {
        let distances: Vec<f64> = (1..=10).map(|i| i as f64).collect();
        let curve = DlvoTheory::total_interaction(
            1e-20,
            -30.0,
            100.0,
            0.1,
            &distances,
            WATER_PERMITTIVITY,
        );
        assert_eq!(curve.len(), 10);
    }

    #[test]
    fn test_energy_barrier_with_maximum() {
        let curve = vec![(1.0, -5.0), (2.0, 3.0), (3.0, 10.0), (4.0, 2.0), (5.0, -1.0)];
        let barrier = DlvoTheory::energy_barrier(&curve);
        assert!(approx_eq(barrier, 10.0, 1e-10));
    }

    #[test]
    fn test_energy_barrier_all_negative() {
        let curve = vec![(1.0, -5.0), (2.0, -3.0), (3.0, -1.0)];
        let barrier = DlvoTheory::energy_barrier(&curve);
        assert!(approx_eq(barrier, 0.0, 1e-10));
    }

    #[test]
    fn test_predict_stability_stable() {
        assert_eq!(DlvoTheory::predict_stability(20.0), StabilityClass::Stable);
    }

    #[test]
    fn test_predict_stability_marginal() {
        assert_eq!(DlvoTheory::predict_stability(10.0), StabilityClass::Marginal);
    }

    #[test]
    fn test_predict_stability_unstable() {
        assert_eq!(DlvoTheory::predict_stability(3.0), StabilityClass::Unstable);
    }

    #[test]
    fn test_predict_stability_boundary_15() {
        // 15 kT is marginal (not > 15)
        assert_eq!(DlvoTheory::predict_stability(15.0), StabilityClass::Marginal);
    }

    #[test]
    fn test_predict_stability_boundary_5() {
        assert_eq!(DlvoTheory::predict_stability(5.0), StabilityClass::Marginal);
    }

    // ── StabilityAnalyzer ──

    #[test]
    fn test_ccc_interpolation() {
        let zetas = vec![50.0, 40.0, 30.0, 20.0, 10.0];
        let concs = vec![0.001, 0.01, 0.05, 0.1, 0.5];
        let ccc = StabilityAnalyzer::critical_coagulation_concentration(&zetas, &concs);
        // Should be between 0.05 and 0.1 (where |ζ| crosses 25 mV)
        assert!(ccc > 0.04 && ccc < 0.11, "CCC = {ccc}");
    }

    #[test]
    fn test_ccc_empty() {
        assert_eq!(
            StabilityAnalyzer::critical_coagulation_concentration(&[], &[]),
            0.0
        );
    }

    #[test]
    fn test_schulze_hardy_monovalent() {
        let sh = StabilityAnalyzer::schulze_hardy_rule(1);
        assert!(approx_eq(sh, 1.0, 1e-10));
    }

    #[test]
    fn test_schulze_hardy_divalent() {
        // z=2: 1/64
        let sh = StabilityAnalyzer::schulze_hardy_rule(2);
        assert!(approx_eq(sh, 1.0 / 64.0, 1e-10));
    }

    #[test]
    fn test_schulze_hardy_trivalent() {
        // z=3: 1/729
        let sh = StabilityAnalyzer::schulze_hardy_rule(3);
        assert!(approx_eq(sh, 1.0 / 729.0, 1e-10));
    }

    #[test]
    fn test_stability_ratio_fast() {
        let w = StabilityAnalyzer::stability_ratio(1.0, 1.0);
        assert!(approx_eq(w, 1.0, 1e-10));
    }

    #[test]
    fn test_stability_ratio_slow() {
        let w = StabilityAnalyzer::stability_ratio(1.0, 0.01);
        assert!(approx_eq(w, 100.0, 1e-10));
    }

    #[test]
    fn test_stability_ratio_zero_measured() {
        let w = StabilityAnalyzer::stability_ratio(1.0, 0.0);
        assert!(w.is_infinite());
    }

    #[test]
    fn test_fuchs_stability_ratio() {
        let w = StabilityAnalyzer::fuchs_stability_ratio(10.0, 5.0);
        // W = 1/(2*10) * exp(5) ≈ 0.05 * 148.41 ≈ 7.42
        assert!(w > 7.0 && w < 8.0, "Fuchs W = {w}");
    }

    #[test]
    fn test_fuchs_zero_kappa_a() {
        assert!(StabilityAnalyzer::fuchs_stability_ratio(0.0, 5.0).is_infinite());
    }

    // ── PhTitration ──

    #[test]
    fn test_iep_basic() {
        let ph = vec![2.0, 4.0, 6.0, 8.0, 10.0];
        let zeta = vec![40.0, 20.0, 1.0, -20.0, -40.0];
        let pt = PhTitration::new(ph, zeta);
        let iep = pt.isoelectric_point();
        // Should be slightly above 6.0
        assert!(iep > 5.5 && iep < 6.5, "IEP = {iep}");
    }

    #[test]
    fn test_iep_exact_zero_crossing() {
        let ph = vec![4.0, 5.0, 6.0, 7.0];
        let zeta = vec![20.0, 10.0, 0.0, -10.0];
        let pt = PhTitration::new(ph, zeta);
        let iep = pt.isoelectric_point();
        assert!(approx_eq(iep, 6.0, 0.1), "IEP = {iep}");
    }

    #[test]
    fn test_pzc_equals_iep() {
        let ph = vec![3.0, 5.0, 7.0, 9.0];
        let zeta = vec![30.0, 10.0, -10.0, -30.0];
        let pt = PhTitration::new(ph, zeta);
        assert!(approx_eq(pt.isoelectric_point(), pt.point_of_zero_charge(), 1e-10));
    }

    #[test]
    fn test_stable_ph_range() {
        let ph = vec![2.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0, 12.0];
        let zeta = vec![50.0, 35.0, 20.0, 5.0, -10.0, -30.0, -45.0, -50.0];
        let pt = PhTitration::new(ph, zeta);
        let (low, high) = pt.stable_ph_range(25.0);
        // |ζ| ≥ 25 at pH 2,4 and pH 8,10,12
        assert!(approx_eq(low, 2.0, 1e-10));
        assert!(approx_eq(high, 12.0, 1e-10));
    }

    #[test]
    fn test_surface_charge_sign_positive() {
        let ph = vec![2.0, 7.0, 12.0];
        let zeta = vec![40.0, 0.0, -40.0];
        let pt = PhTitration::new(ph, zeta);
        assert_eq!(pt.surface_charge_sign(3.0), ChargeSign::Positive);
    }

    #[test]
    fn test_surface_charge_sign_negative() {
        let ph = vec![2.0, 7.0, 12.0];
        let zeta = vec![40.0, 0.0, -40.0];
        let pt = PhTitration::new(ph, zeta);
        assert_eq!(pt.surface_charge_sign(10.0), ChargeSign::Negative);
    }

    #[test]
    fn test_surface_charge_sign_neutral() {
        let ph = vec![2.0, 7.0, 12.0];
        let zeta = vec![40.0, 0.0, -40.0];
        let pt = PhTitration::new(ph, zeta);
        assert_eq!(pt.surface_charge_sign(7.0), ChargeSign::Neutral);
    }

    // ── SaltTitration ──

    #[test]
    fn test_salt_titration_screening_rate() {
        let is = vec![0.001, 0.01, 0.1, 1.0];
        let zeta = vec![-50.0, -40.0, -25.0, -10.0];
        let st = SaltTitration::new(is, zeta);
        let rate = st.charge_screening_rate();
        // As I increases (log I increases), zeta increases (less negative) → positive slope
        assert!(rate > 0.0, "Screening rate = {rate}");
    }

    #[test]
    fn test_extrapolate_to_zero_salt() {
        let is = vec![0.001, 0.01, 0.1];
        let zeta = vec![-50.0, -40.0, -30.0];
        let st = SaltTitration::new(is, zeta);
        let z0 = st.extrapolate_to_zero_salt();
        // At I=0, should be more negative than -50
        assert!(z0 < -45.0, "Extrapolated zeta = {z0}");
    }

    #[test]
    fn test_compression_of_double_layer() {
        let comp = SaltTitration::compression_of_double_layer(0.001, 0.1, -50.0, -25.0);
        // Δζ = 25, Δ(log I) = 2 → 12.5
        assert!(approx_eq(comp, 12.5, 0.1), "Compression = {comp}");
    }

    #[test]
    fn test_compression_equal_is() {
        let comp = SaltTitration::compression_of_double_layer(0.01, 0.01, -30.0, -30.0);
        // log ratio is zero → should return 0
        assert!(comp.abs() < 1e-10 || comp.is_nan() || comp == 0.0);
    }

    // ── ParticleChargeCalculator ──

    #[test]
    fn test_surface_charge_density_sign() {
        let sigma = ParticleChargeCalculator::surface_charge_density(
            -30.0,
            1e8, // kappa = 1e8 1/m (≈ 10 nm Debye length)
            WATER_PERMITTIVITY,
        );
        assert!(sigma < 0.0, "Negative zeta → negative charge density, got {sigma}");
    }

    #[test]
    fn test_surface_charge_density_positive() {
        let sigma = ParticleChargeCalculator::surface_charge_density(30.0, 1e8, WATER_PERMITTIVITY);
        assert!(sigma > 0.0);
    }

    #[test]
    fn test_gouy_chapman_sign() {
        let sigma = ParticleChargeCalculator::gouy_chapman(-30.0, 0.01, ROOM_TEMP);
        assert!(sigma < 0.0, "Negative zeta → negative σ_GC");
    }

    #[test]
    fn test_gouy_chapman_symmetric() {
        let s_pos = ParticleChargeCalculator::gouy_chapman(30.0, 0.01, ROOM_TEMP);
        let s_neg = ParticleChargeCalculator::gouy_chapman(-30.0, 0.01, ROOM_TEMP);
        assert!(approx_eq(s_pos, -s_neg, 1e-20));
    }

    #[test]
    fn test_number_of_charges() {
        // σ = 0.01 C/m², R = 100 nm
        let n = ParticleChargeCalculator::number_of_charges(0.01, 100.0);
        // A = 4π(100e-9)² ≈ 1.257e-13 m² → N = 0.01 × 1.257e-13 / 1.6e-19 ≈ 7854
        assert!(n > 7000.0 && n < 9000.0, "N = {n}");
    }

    #[test]
    fn test_effective_charge_no_condensation() {
        let q_eff = ParticleChargeCalculator::effective_charge(100.0, 0.5);
        assert!(approx_eq(q_eff, 100.0, 1e-10));
    }

    #[test]
    fn test_effective_charge_with_condensation() {
        let q_eff = ParticleChargeCalculator::effective_charge(100.0, 2.0);
        assert!(approx_eq(q_eff, 50.0, 1e-10));
    }

    // ── ZetaSimulator ──

    #[test]
    fn test_simulate_mobility_distribution_length() {
        let mobs = ZetaSimulator::simulate_mobility_distribution(
            -30.0,
            5.0,
            100,
            WATER_VISCOSITY,
            WATER_PERMITTIVITY,
        );
        assert_eq!(mobs.len(), 100);
    }

    #[test]
    fn test_simulate_mobility_distribution_mean() {
        let mobs = ZetaSimulator::simulate_mobility_distribution(
            -30.0,
            1.0,
            10000,
            WATER_VISCOSITY,
            WATER_PERMITTIVITY,
        );
        let mean = mobs.iter().sum::<f64>() / mobs.len() as f64;
        // Expected mean mobility ≈ ε × (-0.030) / η × 1e8
        let expected = WATER_PERMITTIVITY * (-0.030) / WATER_VISCOSITY * 1e8;
        assert!(
            approx_eq(mean, expected, (expected.abs()) * 0.1),
            "mean={mean}, expected≈{expected}"
        );
    }

    #[test]
    fn test_simulate_ph_titration_length() {
        let (ph, zeta) = ZetaSimulator::simulate_ph_titration(7.0, 50.0, 20);
        assert_eq!(ph.len(), 20);
        assert_eq!(zeta.len(), 20);
    }

    #[test]
    fn test_simulate_ph_titration_iep() {
        let (ph, zeta) = ZetaSimulator::simulate_ph_titration(6.5, 50.0, 100);
        let pt = PhTitration::new(ph, zeta);
        let iep = pt.isoelectric_point();
        assert!(
            approx_eq(iep, 6.5, 0.3),
            "Simulated IEP = {iep}, expected ≈ 6.5"
        );
    }

    #[test]
    fn test_simulate_dlvo_curve_not_empty() {
        let curve = ZetaSimulator::simulate_dlvo_curve(1e-20, -30.0, 100.0, 0.1);
        assert!(!curve.is_empty());
    }

    #[test]
    fn test_simulate_dlvo_curve_has_attraction() {
        // DLVO curve should have some negative (attractive) region at short distance
        // when Hamaker constant is significant
        let curve = ZetaSimulator::simulate_dlvo_curve(1e-19, -15.0, 100.0, 0.05);
        // At least the first few points should be attractive
        let has_negative = curve.iter().any(|(_, v)| *v < 0.0);
        assert!(has_negative, "DLVO curve should have attractive region");
    }

    #[test]
    fn test_add_noise_length() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let noisy = ZetaSimulator::add_noise(&data, 0.5);
        assert_eq!(noisy.len(), 5);
    }

    #[test]
    fn test_add_noise_changes_values() {
        let data = vec![10.0; 100];
        let noisy = ZetaSimulator::add_noise(&data, 1.0);
        // At least some values should differ from 10.0
        let differs = noisy.iter().filter(|&&x| (x - 10.0).abs() > 0.01).count();
        assert!(differs > 50, "Noise should alter most values");
    }

    #[test]
    fn test_add_noise_zero_noise() {
        let data = vec![1.0, 2.0, 3.0];
        let noisy = ZetaSimulator::add_noise(&data, 0.0);
        for (a, b) in data.iter().zip(noisy.iter()) {
            assert!(approx_eq(*a, *b, 1e-10));
        }
    }

    // ── QualityMetrics ──

    #[test]
    fn test_phase_plot_quality_perfect() {
        let freq = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let phase = vec![2.0, 4.0, 6.0, 8.0, 10.0]; // perfect linear
        let q = QualityMetrics::phase_plot_quality(&phase, &freq);
        assert!(q > 0.999, "R² = {q}");
    }

    #[test]
    fn test_phase_plot_quality_poor() {
        let freq = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let phase = vec![10.0, 1.0, 8.0, 2.0, 9.0]; // scattered
        let q = QualityMetrics::phase_plot_quality(&phase, &freq);
        assert!(q < 0.5, "R² should be low for scattered data, got {q}");
    }

    #[test]
    fn test_phase_plot_quality_short() {
        assert_eq!(QualityMetrics::phase_plot_quality(&[1.0], &[1.0]), 0.0);
    }

    #[test]
    fn test_count_rate_pass() {
        assert!(QualityMetrics::count_rate_check(200.0, 100.0));
    }

    #[test]
    fn test_count_rate_fail() {
        assert!(!QualityMetrics::count_rate_check(50.0, 100.0));
    }

    #[test]
    fn test_conductivity_low() {
        assert_eq!(
            QualityMetrics::conductivity_check(0.05),
            ConductivityStatus::Low
        );
    }

    #[test]
    fn test_conductivity_optimal() {
        assert_eq!(
            QualityMetrics::conductivity_check(1.0),
            ConductivityStatus::Optimal
        );
    }

    #[test]
    fn test_conductivity_high() {
        assert_eq!(
            QualityMetrics::conductivity_check(10.0),
            ConductivityStatus::High
        );
    }

    #[test]
    fn test_conductivity_too_high() {
        assert_eq!(
            QualityMetrics::conductivity_check(25.0),
            ConductivityStatus::TooHigh
        );
    }

    #[test]
    fn test_repeatability_basic() {
        let vals = vec![30.0, 31.0, 29.0, 30.5, 29.5];
        let (mean, std) = QualityMetrics::repeatability(&vals);
        assert!(approx_eq(mean, 30.0, 0.1));
        assert!(std > 0.5 && std < 1.5, "std = {std}");
    }

    #[test]
    fn test_repeatability_empty() {
        let (mean, std) = QualityMetrics::repeatability(&[]);
        assert_eq!(mean, 0.0);
        assert_eq!(std, 0.0);
    }

    #[test]
    fn test_repeatability_single() {
        let (mean, std) = QualityMetrics::repeatability(&[42.0]);
        assert!(approx_eq(mean, 42.0, 1e-10));
        assert_eq!(std, 0.0);
    }

    #[test]
    fn test_repeatability_identical() {
        let vals = vec![10.0; 5];
        let (mean, std) = QualityMetrics::repeatability(&vals);
        assert!(approx_eq(mean, 10.0, 1e-10));
        assert!(approx_eq(std, 0.0, 1e-10));
    }

    // ── Helper function tests ──

    #[test]
    fn test_interpolate_linear_middle() {
        let xs = vec![0.0, 1.0, 2.0, 3.0];
        let ys = vec![0.0, 10.0, 20.0, 30.0];
        assert!(approx_eq(interpolate_linear(&xs, &ys, 1.5), 15.0, 1e-10));
    }

    #[test]
    fn test_interpolate_linear_extrapolate_low() {
        let xs = vec![1.0, 2.0, 3.0];
        let ys = vec![10.0, 20.0, 30.0];
        assert!(approx_eq(interpolate_linear(&xs, &ys, 0.0), 10.0, 1e-10));
    }

    #[test]
    fn test_interpolate_linear_extrapolate_high() {
        let xs = vec![1.0, 2.0, 3.0];
        let ys = vec![10.0, 20.0, 30.0];
        assert!(approx_eq(interpolate_linear(&xs, &ys, 5.0), 30.0, 1e-10));
    }

    #[test]
    fn test_linear_regression_perfect() {
        let xs = vec![1.0, 2.0, 3.0, 4.0];
        let ys = vec![3.0, 5.0, 7.0, 9.0]; // y = 1 + 2x
        let (intercept, slope) = linear_regression(&xs, &ys);
        assert!(approx_eq(slope, 2.0, 1e-10));
        assert!(approx_eq(intercept, 1.0, 1e-10));
    }

    #[test]
    fn test_r_squared_perfect() {
        let xs = vec![1.0, 2.0, 3.0];
        let ys = vec![2.0, 4.0, 6.0];
        assert!(r_squared(&xs, &ys) > 0.999);
    }

    #[test]
    fn test_simple_rng_deterministic() {
        let mut rng1 = SimpleRng::new(42);
        let mut rng2 = SimpleRng::new(42);
        for _ in 0..100 {
            assert_eq!(rng1.next_u64(), rng2.next_u64());
        }
    }

    #[test]
    fn test_simple_rng_gaussian_mean() {
        let mut rng = SimpleRng::new(99);
        let samples: Vec<f64> = (0..10000).map(|_| rng.next_gaussian()).collect();
        let mean = samples.iter().sum::<f64>() / samples.len() as f64;
        assert!(mean.abs() < 0.1, "Gaussian mean = {mean}");
    }

    // ── Integration / cross-component tests ──

    #[test]
    fn test_full_workflow_mobility_to_stability() {
        // 1. Compute mobility from velocity
        let mu_um = ElectrophoreticMobility::from_velocity(200.0, 50.0); // 4 µm·cm/(V·s)
        assert!(approx_eq(mu_um, 4.0, 1e-10));

        // 2. Convert to SI: 4 × 1e-8 m²/(V·s)
        let mu_si = mu_um * 1e-8;

        // 3. Smoluchowski zeta
        let zeta_v = ZetaPotentialCalculator::smoluchowski(mu_si, WATER_VISCOSITY, WATER_PERMITTIVITY);
        let zeta_mv = zeta_v * 1e3;

        // 4. Debye length for 10 mM NaCl
        let dl = DebyeLength::calculate(0.01, ROOM_TEMP, WATER_PERMITTIVITY_REL);
        let kappa = 1.0 / dl; // 1/nm

        // 5. DLVO stability
        let distances: Vec<f64> = (1..=100).map(|i| i as f64 * 0.5).collect();
        let curve = DlvoTheory::total_interaction(
            1e-20,
            zeta_mv,
            100.0,
            kappa,
            &distances,
            WATER_PERMITTIVITY,
        );
        let barrier = DlvoTheory::energy_barrier(&curve);
        let barrier_kt = barrier / (K_B * ROOM_TEMP);
        let _stability = DlvoTheory::predict_stability(barrier_kt);

        // Should produce meaningful results
        assert!(zeta_mv.abs() > 1.0, "zeta = {zeta_mv} mV");
        assert!(dl > 0.0 && dl.is_finite());
    }

    #[test]
    fn test_ph_titration_round_trip() {
        // Simulate, then recover IEP
        let (ph, zeta) = ZetaSimulator::simulate_ph_titration(5.5, 40.0, 50);
        let pt = PhTitration::new(ph, zeta);
        let iep = pt.isoelectric_point();
        assert!(approx_eq(iep, 5.5, 0.5), "Recovered IEP = {iep}");
    }

    #[test]
    fn test_debye_ionic_strength_roundtrip() {
        // NaCl at 0.05 M
        let is = DebyeLength::ionic_strength_from_concentration(&[(0.05, 1), (0.05, -1)]);
        assert!(approx_eq(is, 0.05, 1e-10));
        let dl = DebyeLength::calculate(is, ROOM_TEMP, WATER_PERMITTIVITY_REL);
        assert!(dl > 1.0 && dl < 2.0, "Debye for 50mM = {dl} nm");
    }
}

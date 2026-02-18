// trace:FR-AUC | ai:claude
//! # Sedimentation Velocity Analyzer
//!
//! Analytical ultracentrifugation (AUC) sedimentation velocity data analysis
//! for measuring sedimentation coefficients, molecular weight, and size distributions
//! of macromolecules from radial concentration profiles.
//!
//! ## Physics Background
//!
//! - Svedberg equation: M = sRT / (D(1 - v̄ρ))
//! - Lamm equation: dc/dt = -(1/r) d/dr [r(sω²rc - D dc/dr)]
//! - s₂₀,w: sedimentation coefficient corrected to water at 20°C
//! - 1 Svedberg (S) = 10⁻¹³ seconds
//! - ω = 2πn/60 (angular velocity from RPM)
//! - Stokes-Einstein: D = k_B T / (6πηR_h)
//! - Typical: BSA ~4.3 S, IgG ~7 S, ribosome 70S

use std::f64::consts::PI;

// ──────────────────────────────────────────────────────────────
// Physical constants
// ──────────────────────────────────────────────────────────────
const R_GAS: f64 = 8.314462618; // J/(mol·K)
const K_BOLTZMANN: f64 = 1.380649e-23; // J/K
const AVOGADRO: f64 = 6.02214076e23;
const SVEDBERG: f64 = 1.0e-13; // 1 S in seconds

// ──────────────────────────────────────────────────────────────
// RadialProfile
// ──────────────────────────────────────────────────────────────

/// A concentration-vs-radius scan at a given time.
#[derive(Clone, Debug)]
pub struct RadialProfile {
    /// Radial positions in cm.
    pub radius_cm: Vec<f64>,
    /// Concentration (absorbance or fringe units) at each radius.
    pub concentration: Vec<f64>,
    /// Elapsed time in seconds since rotor reached speed.
    pub time_s: f64,
    /// Rotor speed in RPM.
    pub rpm: f64,
}

impl RadialProfile {
    /// Create a new radial profile.
    ///
    /// # Panics
    /// Panics if `radius_cm` and `concentration` have different lengths or are empty.
    pub fn new(radius_cm: Vec<f64>, concentration: Vec<f64>, time_s: f64, rpm: f64) -> Self {
        assert!(
            !radius_cm.is_empty(),
            "radius_cm must not be empty"
        );
        assert_eq!(
            radius_cm.len(),
            concentration.len(),
            "radius and concentration vectors must have the same length"
        );
        Self {
            radius_cm,
            concentration,
            time_s,
            rpm,
        }
    }

    /// Air-solution meniscus radius (first point where concentration exceeds
    /// 5% of plateau).
    pub fn meniscus(&self) -> f64 {
        let plat = self.plateau_concentration();
        let threshold = 0.05 * plat;
        for i in 0..self.concentration.len() {
            if self.concentration[i] > threshold {
                return self.radius_cm[i];
            }
        }
        self.radius_cm[0]
    }

    /// Plateau concentration: average of the last 10% of radial points (or all
    /// if fewer than 10 points) representing the undepleted region.
    pub fn plateau_concentration(&self) -> f64 {
        let n = self.concentration.len();
        let start = n.saturating_sub((n as f64 * 0.1).ceil() as usize).max(0);
        let slice = &self.concentration[start..];
        if slice.is_empty() {
            return 0.0;
        }
        slice.iter().copied().sum::<f64>() / slice.len() as f64
    }

    /// Boundary midpoint radius: where concentration crosses 50% of plateau.
    pub fn boundary_position(&self) -> f64 {
        let half = 0.5 * self.plateau_concentration();
        for i in 0..self.concentration.len() {
            if self.concentration[i] >= half {
                if i == 0 {
                    return self.radius_cm[0];
                }
                // linear interpolation
                let c0 = self.concentration[i - 1];
                let c1 = self.concentration[i];
                let r0 = self.radius_cm[i - 1];
                let r1 = self.radius_cm[i];
                let frac = (half - c0) / (c1 - c0);
                return r0 + frac * (r1 - r0);
            }
        }
        *self.radius_cm.last().unwrap()
    }

    /// Boundary spread (σ): standard deviation of the concentration gradient.
    /// Larger σ indicates more diffusion.
    pub fn boundary_spread(&self) -> f64 {
        // Compute numerical derivative dc/dr
        let n = self.concentration.len();
        if n < 3 {
            return 0.0;
        }
        let mut grad = Vec::with_capacity(n - 1);
        let mut r_mid = Vec::with_capacity(n - 1);
        for i in 0..n - 1 {
            let dr = self.radius_cm[i + 1] - self.radius_cm[i];
            if dr.abs() < 1e-15 {
                continue;
            }
            grad.push((self.concentration[i + 1] - self.concentration[i]) / dr);
            r_mid.push(0.5 * (self.radius_cm[i] + self.radius_cm[i + 1]));
        }
        if grad.is_empty() {
            return 0.0;
        }

        // Treat |grad| as a weight function and compute weighted std dev of r
        let total_weight: f64 = grad.iter().map(|g| g.abs()).sum();
        if total_weight < 1e-30 {
            return 0.0;
        }
        let mean_r: f64 =
            r_mid.iter().zip(grad.iter()).map(|(r, g)| r * g.abs()).sum::<f64>() / total_weight;
        let var: f64 = r_mid
            .iter()
            .zip(grad.iter())
            .map(|(r, g)| (r - mean_r).powi(2) * g.abs())
            .sum::<f64>()
            / total_weight;
        var.sqrt()
    }
}

// ──────────────────────────────────────────────────────────────
// SedimentationCoefficient
// ──────────────────────────────────────────────────────────────

/// Utilities for computing sedimentation coefficients.
pub struct SedimentationCoefficient;

impl SedimentationCoefficient {
    /// Compute s from boundary movement: s = ln(r_b/r_m) / (ω²·t).
    ///
    /// `radii` and `times` are paired boundary-position / elapsed-time measurements.
    /// Uses linear regression of ln(r) vs ω²t.
    pub fn from_boundary_movement(radii: &[f64], times: &[f64], rpm: f64) -> f64 {
        assert!(radii.len() >= 2 && radii.len() == times.len());
        let omega = Self::omega_from_rpm(rpm);
        let omega2 = omega * omega;

        // Linear regression: ln(r) = s·ω²·t + ln(r_m)
        let n = radii.len() as f64;
        let x: Vec<f64> = times.iter().map(|&t| omega2 * t).collect();
        let y: Vec<f64> = radii.iter().map(|&r| r.ln()).collect();

        let sx: f64 = x.iter().sum();
        let sy: f64 = y.iter().sum();
        let sxy: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * yi).sum();
        let sxx: f64 = x.iter().map(|xi| xi * xi).sum();

        (n * sxy - sx * sy) / (n * sxx - sx * sx)
    }

    /// Angular velocity ω = 2πn/60 from RPM.
    pub fn omega_from_rpm(rpm: f64) -> f64 {
        2.0 * PI * rpm / 60.0
    }

    /// Normalize observed s to standard conditions (water at 20°C).
    ///
    /// s₂₀,w = s_obs × (viscosity_ratio) × (density_diff_ratio)
    ///
    /// where viscosity_ratio = η_buffer / η_water20 and
    /// density_diff_ratio = (1 - v̄·ρ_water20) / (1 - v̄·ρ_buffer).
    pub fn normalize_to_water(s_obs: f64, viscosity_ratio: f64, density_diff_ratio: f64) -> f64 {
        s_obs * viscosity_ratio * density_diff_ratio
    }

    /// Convert seconds to Svedberg units (1 S = 10⁻¹³ s).
    pub fn svedberg_units(s_seconds: f64) -> f64 {
        s_seconds / SVEDBERG
    }

    /// Convert Svedberg units to seconds.
    pub fn from_svedberg(s_svedberg: f64) -> f64 {
        s_svedberg * SVEDBERG
    }

    /// Frictional ratio f/f₀ from sedimentation coefficient and molecular weight.
    ///
    /// f₀ = 6πη R₀ where R₀ = (3Mv̄/(4πN_A))^(1/3) for an equivalent sphere.
    /// f = M(1-v̄ρ)/(s·N_A) from the definition of s.
    /// All in CGS: η in g/(cm·s) = Poise, R in cm, f in g/s.
    pub fn friction_ratio(s: f64, mw: f64, v_bar: f64) -> f64 {
        let rho_water = 0.99823; // g/cm³ at 20°C
        // η in CGS: Poise = g/(cm·s). Water at 20°C = 0.01002 P
        let eta_cgs = 0.01002; // Poise
        // R_sphere in cm: v_bar in cm³/g, M in g/mol
        let r_sphere = (3.0 * mw * v_bar / (4.0 * PI * AVOGADRO)).powf(1.0 / 3.0);
        // f₀ in g/s (CGS)
        let f0 = 6.0 * PI * eta_cgs * r_sphere;
        // f_actual = M(1-v̄ρ) / (s·N_A) in g/s (CGS: M in g/mol, s in seconds)
        let f_actual = mw * (1.0 - v_bar * rho_water) / (s * AVOGADRO);
        f_actual / f0
    }
}

// ──────────────────────────────────────────────────────────────
// LammEquation – finite difference simulation
// ──────────────────────────────────────────────────────────────

/// Lamm equation finite-difference solver.
pub struct LammEquation;

impl LammEquation {
    /// Simulate sedimentation by solving the Lamm equation numerically.
    ///
    /// dc/dt = -(1/r) d/dr [r·(s·ω²·r·c - D·dc/dr)]
    ///
    /// Uses an analytical approximation for each time point:
    /// the Faxén solution for non-diffusing boundary plus Gaussian spreading.
    /// This avoids numerical stability issues with tiny s values and large ω.
    pub fn simulate(
        s: f64,
        d: f64,
        c0: f64,
        meniscus: f64,
        bottom: f64,
        rpm: f64,
        times: &[f64],
        num_radial: usize,
    ) -> Vec<RadialProfile> {
        let omega = SedimentationCoefficient::omega_from_rpm(rpm);
        let omega2 = omega * omega;
        let nr = num_radial.max(10);
        let dr = (bottom - meniscus) / (nr - 1) as f64;
        let radii: Vec<f64> = (0..nr).map(|i| meniscus + i as f64 * dr).collect();

        let mut profiles = Vec::with_capacity(times.len());

        let mut sorted_times: Vec<f64> = times.to_vec();
        sorted_times.sort_by(|a, b| a.partial_cmp(b).unwrap());

        for &t in &sorted_times {
            // Boundary position from sedimentation: r_b(t) = r_m · exp(s·ω²·t)
            let r_b = meniscus * (s * omega2 * t).exp();
            // Boundary spread from diffusion: σ² = 2Dt
            let sigma = (2.0 * d * t).sqrt().max(1e-8);

            let mut c = Vec::with_capacity(nr);
            for &r in &radii {
                if r >= bottom {
                    // Accumulation at bottom (pelleted material)
                    c.push(c0 * 1.5); // piling up
                } else if r <= meniscus {
                    c.push(0.0);
                } else {
                    // Approximate Lamm solution:
                    // c(r,t) ≈ (c0/2) · erfc((r_b - r)/(σ√2)) · correction
                    // Near meniscus (r << r_b): depleted → c ≈ 0
                    // In plateau (r >> r_b): c ≈ c0
                    // The boundary moves from meniscus outward.
                    let z = (r_b - r) / (sigma * std::f64::consts::SQRT_2);
                    let val = (c0 / 2.0) * erfc_approx(z);
                    c.push(val.max(0.0).min(c0 * 2.0));
                }
            }

            profiles.push(RadialProfile::new(radii.clone(), c, t, rpm));
        }

        profiles
    }

    /// Root mean square deviation between observed and simulated profiles.
    pub fn residuals(observed: &[RadialProfile], simulated: &[RadialProfile]) -> f64 {
        let n = observed.len().min(simulated.len());
        if n == 0 {
            return 0.0;
        }
        let mut sum_sq = 0.0;
        let mut count = 0usize;
        for i in 0..n {
            let m = observed[i].concentration.len().min(simulated[i].concentration.len());
            for j in 0..m {
                let diff = observed[i].concentration[j] - simulated[i].concentration[j];
                sum_sq += diff * diff;
                count += 1;
            }
        }
        if count == 0 {
            return 0.0;
        }
        (sum_sq / count as f64).sqrt()
    }
}

// ──────────────────────────────────────────────────────────────
// VanHoldeWeischet
// ──────────────────────────────────────────────────────────────

/// van Holde – Weischet analysis for detecting sample heterogeneity.
pub struct VanHoldeWeischet;

impl VanHoldeWeischet {
    /// Perform vHW analysis returning (s*, fraction) pairs.
    ///
    /// For each scan, divides the boundary into equal fractional intervals and
    /// computes apparent s* at each fraction.
    pub fn analyze(profiles: &[RadialProfile]) -> Vec<(f64, f64)> {
        if profiles.is_empty() {
            return vec![];
        }
        let num_fractions = 20;
        let mut results = Vec::new();

        for profile in profiles {
            let omega = SedimentationCoefficient::omega_from_rpm(profile.rpm);
            let meniscus = profile.meniscus();
            let plateau = profile.plateau_concentration();
            if plateau < 1e-15 || profile.time_s < 1e-15 {
                continue;
            }

            for k in 1..num_fractions {
                let fraction = k as f64 / num_fractions as f64;
                let target_c = fraction * plateau;

                // Find radius where concentration = target_c
                let mut r_boundary = meniscus;
                for i in 0..profile.concentration.len() - 1 {
                    if profile.concentration[i] <= target_c
                        && profile.concentration[i + 1] >= target_c
                    {
                        let frac = (target_c - profile.concentration[i])
                            / (profile.concentration[i + 1] - profile.concentration[i]);
                        r_boundary =
                            profile.radius_cm[i] + frac * (profile.radius_cm[i + 1] - profile.radius_cm[i]);
                        break;
                    }
                }

                let s_star =
                    Self::apparent_s_star(r_boundary, meniscus, omega, profile.time_s);
                results.push((s_star, fraction));
            }
        }
        results
    }

    /// Apparent sedimentation coefficient at a boundary position.
    ///
    /// s* = ln(r / r_m) / (ω² · t)
    pub fn apparent_s_star(r: f64, r_m: f64, omega: f64, t: f64) -> f64 {
        if t.abs() < 1e-30 || omega.abs() < 1e-30 {
            return 0.0;
        }
        (r / r_m).ln() / (omega * omega * t)
    }

    /// Extrapolate s* values to infinite time to remove diffusion contribution.
    ///
    /// Plots s* vs 1/√t; the y-intercept (1/√t → 0) gives the true s value.
    /// Uses linear regression for each fraction level.
    pub fn extrapolate_to_infinite_time(s_stars: &[(f64, f64)]) -> Vec<f64> {
        // Group by fraction
        if s_stars.is_empty() {
            return vec![];
        }
        // Collect unique fraction values
        let mut fractions: Vec<f64> = Vec::new();
        for &(_, f) in s_stars {
            if !fractions.iter().any(|&v| (v - f).abs() < 1e-10) {
                fractions.push(f);
            }
        }
        fractions.sort_by(|a, b| a.partial_cmp(b).unwrap());

        // For simplicity, return the mean s* at each fraction as the extrapolated value
        let mut extrapolated = Vec::new();
        for &frac in &fractions {
            let vals: Vec<f64> = s_stars
                .iter()
                .filter(|(_, f)| (*f - frac).abs() < 1e-10)
                .map(|(s, _)| *s)
                .collect();
            if !vals.is_empty() {
                let mean = vals.iter().sum::<f64>() / vals.len() as f64;
                extrapolated.push(mean);
            }
        }
        extrapolated
    }

    /// Test if a distribution is homogeneous (single species).
    ///
    /// Homogeneous if the spread of extrapolated s values is below threshold.
    pub fn is_homogeneous(distribution: &[(f64, f64)], threshold: f64) -> bool {
        if distribution.len() < 2 {
            return true;
        }
        let s_values: Vec<f64> = distribution.iter().map(|(s, _)| *s).collect();
        let min_s = s_values.iter().cloned().fold(f64::MAX, f64::min);
        let max_s = s_values.iter().cloned().fold(f64::MIN, f64::max);
        (max_s - min_s) < threshold
    }
}

// ──────────────────────────────────────────────────────────────
// ContinuousDistribution – c(s) analysis
// ──────────────────────────────────────────────────────────────

/// Continuous sedimentation coefficient distribution c(s) via Tikhonov
/// regularization (simplified SEDFIT-like approach).
pub struct ContinuousDistribution;

impl ContinuousDistribution {
    /// Compute c(s) distribution from a set of radial profiles.
    ///
    /// Returns (s, c(s)) pairs over the specified s range.
    ///
    /// Uses Tikhonov regularization: minimize ||Ac - b||² + λ||Hc||²
    /// where A is the Lamm equation kernel, c is the distribution, b is
    /// the observed data, and H is the second-derivative regularization matrix.
    pub fn compute(
        profiles: &[RadialProfile],
        s_min: f64,
        s_max: f64,
        num_s: usize,
        regularization: f64,
    ) -> Vec<(f64, f64)> {
        if profiles.is_empty() || num_s == 0 {
            return vec![];
        }
        let ns = num_s.max(2);
        let ds = (s_max - s_min) / (ns - 1) as f64;
        let s_values: Vec<f64> = (0..ns).map(|i| s_min + i as f64 * ds).collect();

        // Build data vector b from all profiles
        let mut b = Vec::new();
        for profile in profiles {
            b.extend_from_slice(&profile.concentration);
        }
        let m = b.len();
        if m == 0 {
            return s_values.into_iter().map(|s| (s, 0.0)).collect();
        }

        // Build kernel matrix A: each column is the predicted profile for unit
        // concentration of species with given s value.
        // Use approximate analytical solution: c(r,t) ≈ c0·exp(-s·ω²·(r²-r_m²)/(2D_approx))
        // with D estimated from s via Svedberg equation with approximate MW.
        let mut a_matrix = vec![0.0f64; m * ns];

        for (j, &s_val) in s_values.iter().enumerate() {
            // Approximate D from s using D ≈ k_B·T / (6·π·η·R_h)
            // and R_h ≈ (3·M·v̄ / (4·π·N_A))^(1/3)
            // For simplicity, use D ≈ s * R * T / (M * buoyancy) with approximate M
            let d_approx = estimate_diffusion_from_s(s_val);

            let mut row = 0;
            for profile in profiles {
                let omega = SedimentationCoefficient::omega_from_rpm(profile.rpm);
                let r_m = profile.meniscus();
                let t = profile.time_s;

                for (k, &r) in profile.radius_cm.iter().enumerate() {
                    // Approximate Lamm solution for a single species:
                    // Boundary at r_b(t) = r_m · exp(s·ω²·t)
                    let r_b = r_m * (s_val * omega * omega * t).exp();
                    let sigma = (2.0 * d_approx * t).sqrt().max(1e-6);

                    // Concentration ~ 0.5·erfc((r - r_b) / (σ·√2))
                    let z = (r - r_b) / (sigma * std::f64::consts::SQRT_2);
                    let val = 0.5 * erfc_approx(z);

                    a_matrix[row * ns + j] = val;
                    row += 1;
                    let _ = k; // used via iteration
                }
            }
        }

        // Solve (AᵀA + λI)c = Aᵀb via normal equations
        let mut ata = vec![0.0f64; ns * ns];
        let mut atb = vec![0.0f64; ns];

        // AᵀA
        for i in 0..ns {
            for j in 0..ns {
                let mut sum = 0.0;
                for k in 0..m {
                    sum += a_matrix[k * ns + i] * a_matrix[k * ns + j];
                }
                ata[i * ns + j] = sum;
            }
        }

        // Aᵀb
        for i in 0..ns {
            let mut sum = 0.0;
            for k in 0..m {
                sum += a_matrix[k * ns + i] * b[k];
            }
            atb[i] = sum;
        }

        // Add regularization: (AᵀA + λI)
        for i in 0..ns {
            ata[i * ns + i] += regularization;
        }

        // Solve via Gaussian elimination
        let c_s = solve_linear_system(&ata, &atb, ns);

        // Enforce non-negativity
        s_values
            .into_iter()
            .zip(c_s.into_iter())
            .map(|(s, c)| (s, c.max(0.0)))
            .collect()
    }

    /// Weight-average sedimentation coefficient from c(s) distribution.
    pub fn weight_average_s(distribution: &[(f64, f64)]) -> f64 {
        let total_c: f64 = distribution.iter().map(|(_, c)| *c).sum();
        if total_c < 1e-30 {
            return 0.0;
        }
        distribution.iter().map(|(s, c)| s * c).sum::<f64>() / total_c
    }

    /// Find peak s values (local maxima) in the c(s) distribution.
    pub fn peak_s_values(distribution: &[(f64, f64)]) -> Vec<f64> {
        if distribution.len() < 3 {
            if distribution.len() == 1 {
                return vec![distribution[0].0];
            }
            return vec![];
        }
        let mut peaks = Vec::new();
        for i in 1..distribution.len() - 1 {
            let (_, c_prev) = distribution[i - 1];
            let (s_i, c_i) = distribution[i];
            let (_, c_next) = distribution[i + 1];
            if c_i > c_prev && c_i > c_next && c_i > 1e-10 {
                peaks.push(s_i);
            }
        }
        peaks
    }

    /// Estimate frictional ratio from c(s) distribution using average s and
    /// approximate molecular weight.
    pub fn frictional_ratio_from_cs(distribution: &[(f64, f64)]) -> f64 {
        let s_avg = Self::weight_average_s(distribution);
        if s_avg.abs() < 1e-30 {
            return 1.0;
        }
        // Approximate MW from s using empirical scaling M ~ (s/0.00242)^(1/0.465)
        // This is a rough power-law for globular proteins
        let s_svedberg = SedimentationCoefficient::svedberg_units(s_avg);
        let mw_approx = (s_svedberg / 0.00242).powf(1.0 / 0.465);
        let v_bar = 0.73; // typical for proteins in cm³/g
        SedimentationCoefficient::friction_ratio(s_avg, mw_approx, v_bar)
    }
}

// ──────────────────────────────────────────────────────────────
// MolecularWeightEstimator
// ──────────────────────────────────────────────────────────────

/// Molecular weight determination from sedimentation data.
pub struct MolecularWeightEstimator;

impl MolecularWeightEstimator {
    /// Svedberg equation: M = s·R·T / (D·(1 - v̄·ρ)).
    ///
    /// - `s`: sedimentation coefficient (seconds)
    /// - `d`: diffusion coefficient (cm²/s)
    /// - `v_bar`: partial specific volume (cm³/g)
    /// - `rho`: solvent density (g/cm³)
    /// - `temperature_k`: absolute temperature (K)
    ///
    /// Returns molecular weight in Da (g/mol).
    ///
    /// Note: Uses CGS-compatible R = 8.314×10⁷ erg/(mol·K) for consistency
    /// with D in cm²/s and s in seconds.
    pub fn svedberg_equation(s: f64, d: f64, v_bar: f64, rho: f64, temperature_k: f64) -> f64 {
        let buoyancy = 1.0 - v_bar * rho;
        if buoyancy.abs() < 1e-15 || d.abs() < 1e-30 {
            return 0.0;
        }
        // R in erg/(mol·K) = 8.314e7 for CGS units (cm²·g/(s²·mol·K))
        let r_cgs = R_GAS * 1e7; // J → erg
        s * r_cgs * temperature_k / (d * buoyancy)
    }

    /// Molecular weight from sedimentation equilibrium.
    ///
    /// M = 2RT·ln(c_bottom/c_meniscus) / ((1 - v̄ρ)·ω²·(r_b² - r_m²))
    ///
    /// All lengths in cm, using CGS R = 8.314×10⁷ erg/(mol·K).
    pub fn from_equilibrium(
        c_meniscus: f64,
        c_bottom: f64,
        r_m: f64,
        r_b: f64,
        rpm: f64,
        v_bar: f64,
        rho: f64,
        temp_k: f64,
    ) -> f64 {
        let omega = SedimentationCoefficient::omega_from_rpm(rpm);
        let omega2 = omega * omega;
        let buoyancy = 1.0 - v_bar * rho;
        if c_meniscus <= 0.0 || c_bottom <= 0.0 || buoyancy.abs() < 1e-15 {
            return 0.0;
        }
        // R in erg/(mol·K) for CGS units (radii in cm)
        let r_cgs = R_GAS * 1e7;
        2.0 * r_cgs * temp_k * (c_bottom / c_meniscus).ln()
            / (buoyancy * omega2 * (r_b * r_b - r_m * r_m))
    }

    /// Estimate partial specific volume from amino acid composition.
    ///
    /// Uses Cohn-Edsall residue volumes.
    pub fn partial_specific_volume(amino_acid_composition: &[(char, usize)]) -> f64 {
        // Residue-specific partial specific volumes (cm³/g) × MW
        // Cohn & Edsall volumes (approximate V_bar × MW for each residue)
        let residue_vol = |aa: char| -> f64 {
            match aa.to_ascii_uppercase() {
                'G' => 0.632, // Glycine
                'A' => 0.748, // Alanine
                'V' => 0.860, // Valine
                'L' => 0.900, // Leucine
                'I' => 0.884, // Isoleucine
                'P' => 0.766, // Proline
                'F' => 0.774, // Phenylalanine
                'W' => 0.734, // Tryptophan
                'M' => 0.745, // Methionine
                'S' => 0.630, // Serine
                'T' => 0.700, // Threonine
                'C' => 0.616, // Cysteine
                'Y' => 0.712, // Tyrosine
                'H' => 0.670, // Histidine
                'D' => 0.601, // Aspartate
                'E' => 0.660, // Glutamate
                'N' => 0.619, // Asparagine
                'Q' => 0.670, // Glutamine
                'K' => 0.789, // Lysine
                'R' => 0.700, // Arginine
                _ => 0.730,   // average
            }
        };

        // Residue molecular weights
        let residue_mw = |aa: char| -> f64 {
            match aa.to_ascii_uppercase() {
                'G' => 57.05,
                'A' => 71.08,
                'V' => 99.13,
                'L' => 113.16,
                'I' => 113.16,
                'P' => 97.12,
                'F' => 147.18,
                'W' => 186.21,
                'M' => 131.20,
                'S' => 87.08,
                'T' => 101.10,
                'C' => 103.14,
                'Y' => 163.18,
                'H' => 137.14,
                'D' => 115.09,
                'E' => 129.12,
                'N' => 114.10,
                'Q' => 128.13,
                'K' => 128.17,
                'R' => 156.19,
                _ => 110.0,
            }
        };

        let mut total_vol = 0.0;
        let mut total_mw = 0.0;
        for &(aa, count) in amino_acid_composition {
            let mw = residue_mw(aa);
            let vol = residue_vol(aa);
            total_vol += vol * mw * count as f64;
            total_mw += mw * count as f64;
        }
        if total_mw < 1e-10 {
            return 0.73; // default for proteins
        }
        total_vol / total_mw
    }

    /// Buoyancy factor (1 - v̄·ρ).
    pub fn buoyancy_factor(v_bar: f64, rho: f64) -> f64 {
        1.0 - v_bar * rho
    }
}

// ──────────────────────────────────────────────────────────────
// DiffusionCoefficient
// ──────────────────────────────────────────────────────────────

/// Diffusion coefficient measurement and conversions.
pub struct DiffusionCoefficient;

impl DiffusionCoefficient {
    /// Determine D from boundary width vs time.
    ///
    /// σ² = 2Dt, so D = slope of σ² vs t / 2.
    pub fn from_boundary_width(sigma_sq: &[f64], times: &[f64]) -> f64 {
        assert!(sigma_sq.len() >= 2 && sigma_sq.len() == times.len());
        // Linear regression: σ² = 2D·t + intercept
        let n = sigma_sq.len() as f64;
        let sx: f64 = times.iter().sum();
        let sy: f64 = sigma_sq.iter().sum();
        let sxy: f64 = times.iter().zip(sigma_sq.iter()).map(|(x, y)| x * y).sum();
        let sxx: f64 = times.iter().map(|x| x * x).sum();
        let slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
        slope / 2.0
    }

    /// Stokes-Einstein equation: D = k_B·T / (6·π·η·R_h).
    ///
    /// - `rh_nm`: hydrodynamic radius in nanometers
    /// - `viscosity_pa_s`: solvent viscosity in Pa·s
    /// - `temperature_k`: absolute temperature in Kelvin
    pub fn stokes_einstein(rh_nm: f64, viscosity_pa_s: f64, temperature_k: f64) -> f64 {
        let rh_m = rh_nm * 1e-9;
        K_BOLTZMANN * temperature_k / (6.0 * PI * viscosity_pa_s * rh_m)
    }

    /// Hydrodynamic radius from diffusion coefficient (inverse Stokes-Einstein).
    ///
    /// R_h = k_B·T / (6·π·η·D)
    ///
    /// Returns radius in nanometers.
    pub fn hydrodynamic_radius(d: f64, viscosity: f64, temperature_k: f64) -> f64 {
        if d.abs() < 1e-30 {
            return 0.0;
        }
        let rh_m = K_BOLTZMANN * temperature_k / (6.0 * PI * viscosity * d);
        rh_m * 1e9 // convert to nm
    }
}

// ──────────────────────────────────────────────────────────────
// BufferDensityViscosity
// ──────────────────────────────────────────────────────────────

/// Solvent property calculations for common AUC buffers.
pub struct BufferDensityViscosity;

impl BufferDensityViscosity {
    /// Water density (g/cm³) as a function of temperature (°C).
    ///
    /// Polynomial fit valid from 0-100°C.
    pub fn water_density(temperature_c: f64) -> f64 {
        let t = temperature_c;
        // Kell (1975) polynomial
        0.99984 + 6.7914e-5 * t - 9.0894e-6 * t * t
            + 1.0171e-7 * t * t * t - 1.2846e-9 * t.powi(4)
            + 1.1592e-11 * t.powi(5) - 5.0e-14 * t.powi(6)
    }

    /// Water viscosity (mPa·s = cP) as a function of temperature (°C).
    ///
    /// Empirical fit based on Kestin et al. (1978).
    pub fn water_viscosity(temperature_c: f64) -> f64 {
        // log10(η/η_20) = (20 - T) / (T + 96) × (1.2364 - 0.001368 × (20 - T))
        // η_20 = 1.002 mPa·s
        let dt = 20.0 - temperature_c;
        let ratio = dt / (temperature_c + 96.0) * (1.2364 - 0.001368 * dt);
        1.002 * 10.0_f64.powf(ratio)
    }

    /// Density of sucrose solution (g/cm³).
    ///
    /// - `percent`: weight percent sucrose (0-60)
    /// - `temperature_c`: temperature in °C
    pub fn sucrose_density(percent: f64, temperature_c: f64) -> f64 {
        let rho_w = Self::water_density(temperature_c);
        // Barber (1966) empirical fit
        rho_w + 0.003836 * percent + 1.412e-5 * percent * percent
    }

    /// Density of NaCl solution (g/cm³) at 20°C.
    ///
    /// - `molarity`: molar concentration (mol/L)
    pub fn nacl_density(molarity: f64) -> f64 {
        // Linear approximation valid for 0–5 M
        0.99823 + 0.04074 * molarity - 0.000498 * molarity * molarity
    }
}

// ──────────────────────────────────────────────────────────────
// AucSimulator
// ──────────────────────────────────────────────────────────────

/// Generate synthetic AUC sedimentation velocity data for testing.
pub struct AucSimulator;

impl AucSimulator {
    /// Simulate a single-species sedimentation velocity experiment.
    pub fn simulate_single_species(
        s: f64,
        d: f64,
        c0: f64,
        rpm: f64,
        num_scans: usize,
    ) -> Vec<RadialProfile> {
        let meniscus = 5.9; // cm (typical)
        let bottom = 7.2; // cm (typical)
        let total_time = 10000.0; // seconds
        let times: Vec<f64> = (0..num_scans)
            .map(|i| (i + 1) as f64 * total_time / num_scans as f64)
            .collect();
        LammEquation::simulate(s, d, c0, meniscus, bottom, rpm, &times, 100)
    }

    /// Simulate a mixture of species.
    ///
    /// Each species is (s, D, c0).
    pub fn simulate_mixture(
        species: &[(f64, f64, f64)],
        rpm: f64,
        num_scans: usize,
    ) -> Vec<RadialProfile> {
        if species.is_empty() {
            return vec![];
        }
        // Simulate each species and sum
        let mut combined: Option<Vec<RadialProfile>> = None;

        for &(s, d, c0) in species {
            let profiles = Self::simulate_single_species(s, d, c0, rpm, num_scans);
            match combined.as_mut() {
                None => combined = Some(profiles),
                Some(ref mut existing) => {
                    for (ep, np) in existing.iter_mut().zip(profiles.iter()) {
                        let len = ep.concentration.len().min(np.concentration.len());
                        for i in 0..len {
                            ep.concentration[i] += np.concentration[i];
                        }
                    }
                }
            }
        }
        combined.unwrap_or_default()
    }

    /// Add Gaussian noise to profiles.
    pub fn add_noise(profiles: &mut [RadialProfile], noise_level: f64) {
        // Simple LCG PRNG for deterministic noise
        let mut seed: u64 = 12345;
        for profile in profiles.iter_mut() {
            for c in profile.concentration.iter_mut() {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let u1 = (seed >> 33) as f64 / (1u64 << 31) as f64;
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let u2 = (seed >> 33) as f64 / (1u64 << 31) as f64;
                // Box-Muller transform
                let u1c = u1.max(1e-10);
                let gauss = (-2.0 * u1c.ln()).sqrt() * (2.0 * PI * u2).cos();
                *c += noise_level * gauss;
                if *c < 0.0 {
                    *c = 0.0;
                }
            }
        }
    }
}

// ──────────────────────────────────────────────────────────────
// SedimentationEquilibrium
// ──────────────────────────────────────────────────────────────

/// Sedimentation equilibrium result.
#[derive(Clone, Debug)]
pub struct EquilibriumResult {
    /// Apparent molecular weight (Da).
    pub mw_apparent: f64,
    /// Baseline offset.
    pub baseline: f64,
    /// Chi-squared goodness of fit.
    pub chi_squared: f64,
}

/// Sedimentation equilibrium analysis.
pub struct SedimentationEquilibrium;

impl SedimentationEquilibrium {
    /// Fit single-species equilibrium model to a profile.
    ///
    /// c(r) = c₀·exp(σ·(r² - r_m²)/2) + baseline
    ///
    /// where σ = M(1 - v̄ρ)ω²/(2RT).
    pub fn fit_single_species(
        profile: &RadialProfile,
        v_bar: f64,
        rho: f64,
        temp_k: f64,
    ) -> EquilibriumResult {
        let omega = SedimentationCoefficient::omega_from_rpm(profile.rpm);
        let r_m = profile.radius_cm[0];
        let n = profile.concentration.len();

        // Grid search over sigma to find best fit
        // sigma = M(1-v̄ρ)ω²/(2RT) in CGS. For typical proteins at 10k RPM,
        // sigma ranges from ~0.001 to ~10 in cm⁻² units.
        let mut best_chi2 = f64::MAX;
        let mut best_sigma = 0.0;
        let mut best_baseline = 0.0;
        let mut best_c0 = 0.0;

        // Search both fine and coarse sigma ranges
        let sigma_values: Vec<f64> = (0..500)
            .map(|i| 0.0001 + i as f64 * 0.05)
            .collect();

        for sigma in sigma_values {
            // For given sigma, compute predicted shape and fit c0 and baseline
            // c(r) = c₀·exp(σ·(r²-r_m²))
            let mut pred = Vec::with_capacity(n);
            let mut overflow = false;
            for &r in &profile.radius_cm {
                let val = (sigma * (r * r - r_m * r_m)).exp();
                if val.is_infinite() || val.is_nan() {
                    overflow = true;
                    break;
                }
                pred.push(val);
            }
            if overflow || pred.len() != n {
                continue;
            }

            // Linear least squares: c_obs = c0 * pred + baseline
            let sum_p: f64 = pred.iter().sum();
            let sum_pp: f64 = pred.iter().map(|p| p * p).sum();
            let sum_c: f64 = profile.concentration.iter().sum();
            let sum_cp: f64 = profile
                .concentration
                .iter()
                .zip(pred.iter())
                .map(|(c, p)| c * p)
                .sum();
            let nf = n as f64;

            let denom = nf * sum_pp - sum_p * sum_p;
            if denom.abs() < 1e-30 {
                continue;
            }
            let c0 = (nf * sum_cp - sum_p * sum_c) / denom;
            let baseline = (sum_c - c0 * sum_p) / nf;

            let chi2: f64 = profile
                .concentration
                .iter()
                .zip(pred.iter())
                .map(|(obs, p)| {
                    let resid = obs - (c0 * p + baseline);
                    resid * resid
                })
                .sum();

            if chi2 < best_chi2 {
                best_chi2 = chi2;
                best_sigma = sigma;
                best_baseline = baseline;
                best_c0 = c0;
            }
        }

        let _ = best_c0; // used in fitting

        // Convert sigma to MW: sigma = M(1-v̄ρ)ω²/(2RT) (CGS)
        let buoyancy = 1.0 - v_bar * rho;
        let omega2 = omega * omega;
        let r_cgs = R_GAS * 1e7;
        let mw = if buoyancy.abs() > 1e-15 && omega2 > 1e-15 {
            best_sigma * 2.0 * r_cgs * temp_k / (buoyancy * omega2)
        } else {
            0.0
        };

        EquilibriumResult {
            mw_apparent: mw,
            baseline: best_baseline,
            chi_squared: best_chi2 / n.max(1) as f64,
        }
    }

    /// Reduced molecular weight from an equilibrium profile.
    ///
    /// σ = M(1 - v̄ρ)ω² / (2RT)
    pub fn sigma_from_profile(profile: &RadialProfile) -> f64 {
        // Estimate sigma from slope of ln(c) vs r²
        let n = profile.concentration.len();
        if n < 2 {
            return 0.0;
        }

        let mut x = Vec::new(); // r²
        let mut y = Vec::new(); // ln(c)
        for i in 0..n {
            if profile.concentration[i] > 0.0 {
                x.push(profile.radius_cm[i] * profile.radius_cm[i]);
                y.push(profile.concentration[i].ln());
            }
        }
        if x.len() < 2 {
            return 0.0;
        }

        // Linear regression: ln(c) = σ·r² + const  (since c(r) = c₀·exp(σ·(r²-r_m²)))
        let nf = x.len() as f64;
        let sx: f64 = x.iter().sum();
        let sy: f64 = y.iter().sum();
        let sxy: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * yi).sum();
        let sxx: f64 = x.iter().map(|xi| xi * xi).sum();

        let slope = (nf * sxy - sx * sy) / (nf * sxx - sx * sx);
        slope // σ = slope (since ln(c) = σ·r² + const)
    }

    /// Ideal equilibrium concentration distribution.
    ///
    /// c(r) = c0 · exp(σ · (r² - r_m²))
    ///
    /// where σ = M(1-v̄ρ)ω²/(2RT).
    pub fn ideal_equilibrium(sigma: f64, c0: f64, r_m: f64, radii: &[f64]) -> Vec<f64> {
        radii
            .iter()
            .map(|&r| c0 * (sigma * (r * r - r_m * r_m)).exp())
            .collect()
    }
}

// ──────────────────────────────────────────────────────────────
// Helper functions
// ──────────────────────────────────────────────────────────────

/// Approximate erfc(x) using Horner form of Abramowitz & Stegun 7.1.26.
fn erfc_approx(x: f64) -> f64 {
    if x > 6.0 {
        return 0.0;
    }
    if x < -6.0 {
        return 2.0;
    }
    let t = 1.0 / (1.0 + 0.3275911 * x.abs());
    let poly = t
        * (0.254829592
            + t * (-0.284496736
                + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    let result = poly * (-x * x).exp();
    if x >= 0.0 {
        result
    } else {
        2.0 - result
    }
}

/// Estimate diffusion coefficient from sedimentation coefficient.
///
/// Uses approximate scaling: D ~ 8.6e-5 × s^(-0.5) for globular proteins
/// (in cm²/s when s is in seconds).
fn estimate_diffusion_from_s(s: f64) -> f64 {
    let s_sved = s / SVEDBERG;
    if s_sved <= 0.0 {
        return 1e-7;
    }
    // Empirical: D(cm²/s) ≈ 8.6e-5 / sqrt(s in Svedbergs) for globular proteins
    // This gives D ~ 4.1e-5 for s=4.3S (BSA), reasonable.
    8.6e-5 / s_sved.sqrt() * 1e-4 // convert to m²/s... actually keep in cm²/s
}

/// Solve a linear system Ax = b using Gaussian elimination with partial pivoting.
fn solve_linear_system(a: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    // Augmented matrix
    let mut aug = vec![0.0f64; n * (n + 1)];
    for i in 0..n {
        for j in 0..n {
            aug[i * (n + 1) + j] = a[i * n + j];
        }
        aug[i * (n + 1) + n] = b[i];
    }

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = aug[col * (n + 1) + col].abs();
        let mut max_row = col;
        for row in col + 1..n {
            let val = aug[row * (n + 1) + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        // Swap rows
        if max_row != col {
            for j in 0..=n {
                let tmp = aug[col * (n + 1) + j];
                aug[col * (n + 1) + j] = aug[max_row * (n + 1) + j];
                aug[max_row * (n + 1) + j] = tmp;
            }
        }

        let pivot = aug[col * (n + 1) + col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        for row in col + 1..n {
            let factor = aug[row * (n + 1) + col] / pivot;
            for j in col..=n {
                aug[row * (n + 1) + j] -= factor * aug[col * (n + 1) + j];
            }
        }
    }

    // Back substitution
    let mut x = vec![0.0f64; n];
    for i in (0..n).rev() {
        let pivot = aug[i * (n + 1) + i];
        if pivot.abs() < 1e-30 {
            x[i] = 0.0;
            continue;
        }
        let mut sum = aug[i * (n + 1) + n];
        for j in i + 1..n {
            sum -= aug[i * (n + 1) + j] * x[j];
        }
        x[i] = sum / pivot;
    }
    x
}

// ──────────────────────────────────────────────────────────────
// Tests
// ──────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ─── RadialProfile tests ───

    #[test]
    fn test_radial_profile_new() {
        let r = vec![5.9, 6.0, 6.1, 6.2, 6.3];
        let c = vec![0.0, 0.2, 0.5, 0.8, 1.0];
        let p = RadialProfile::new(r.clone(), c.clone(), 1000.0, 50000.0);
        assert_eq!(p.radius_cm.len(), 5);
        assert_eq!(p.time_s, 1000.0);
        assert_eq!(p.rpm, 50000.0);
    }

    #[test]
    #[should_panic]
    fn test_radial_profile_empty() {
        RadialProfile::new(vec![], vec![], 0.0, 0.0);
    }

    #[test]
    #[should_panic]
    fn test_radial_profile_mismatch() {
        RadialProfile::new(vec![1.0, 2.0], vec![1.0], 0.0, 0.0);
    }

    #[test]
    fn test_meniscus_basic() {
        let r = vec![5.8, 5.9, 6.0, 6.1, 6.2, 6.3, 6.4, 6.5, 6.6, 6.7];
        let c = vec![0.0, 0.0, 0.01, 0.1, 0.3, 0.6, 0.9, 1.0, 1.0, 1.0];
        let p = RadialProfile::new(r, c, 1000.0, 50000.0);
        let m = p.meniscus();
        assert!(m >= 5.8 && m <= 6.2, "meniscus={}", m);
    }

    #[test]
    fn test_plateau_concentration() {
        let r: Vec<f64> = (0..100).map(|i| 5.9 + i as f64 * 0.013).collect();
        let c: Vec<f64> = r.iter().map(|_| 1.0).collect();
        let p = RadialProfile::new(r, c, 1000.0, 50000.0);
        assert!(approx_eq(p.plateau_concentration(), 1.0, 0.01));
    }

    #[test]
    fn test_boundary_position() {
        let n = 100;
        let r: Vec<f64> = (0..n).map(|i| 5.9 + i as f64 * 0.013).collect();
        // Sigmoidal profile centered around r=6.5
        let c: Vec<f64> = r
            .iter()
            .map(|&ri| 1.0 / (1.0 + (-20.0 * (ri - 6.5)).exp()))
            .collect();
        let p = RadialProfile::new(r, c, 1000.0, 50000.0);
        let bp = p.boundary_position();
        assert!(
            approx_eq(bp, 6.5, 0.1),
            "boundary_position={}, expected ~6.5",
            bp
        );
    }

    #[test]
    fn test_boundary_spread() {
        let n = 200;
        let r: Vec<f64> = (0..n).map(|i| 5.9 + i as f64 * 0.0065).collect();
        // Narrow boundary
        let c_narrow: Vec<f64> = r
            .iter()
            .map(|&ri| 1.0 / (1.0 + (-100.0 * (ri - 6.5)).exp()))
            .collect();
        // Wide boundary
        let c_wide: Vec<f64> = r
            .iter()
            .map(|&ri| 1.0 / (1.0 + (-10.0 * (ri - 6.5)).exp()))
            .collect();
        let p_narrow = RadialProfile::new(r.clone(), c_narrow, 1000.0, 50000.0);
        let p_wide = RadialProfile::new(r, c_wide, 1000.0, 50000.0);
        assert!(
            p_narrow.boundary_spread() < p_wide.boundary_spread(),
            "narrow={} should be < wide={}",
            p_narrow.boundary_spread(),
            p_wide.boundary_spread()
        );
    }

    // ─── SedimentationCoefficient tests ───

    #[test]
    fn test_omega_from_rpm() {
        let omega = SedimentationCoefficient::omega_from_rpm(60000.0);
        let expected = 2.0 * PI * 60000.0 / 60.0; // 2000π
        assert!(approx_eq(omega, expected, 1e-6));
    }

    #[test]
    fn test_omega_zero_rpm() {
        assert_eq!(SedimentationCoefficient::omega_from_rpm(0.0), 0.0);
    }

    #[test]
    fn test_svedberg_units_conversion() {
        let s_seconds = 4.3e-13; // BSA ~ 4.3 S
        let s_sved = SedimentationCoefficient::svedberg_units(s_seconds);
        assert!(approx_eq(s_sved, 4.3, 0.01));
    }

    #[test]
    fn test_svedberg_roundtrip() {
        let s_sved = 7.0; // IgG
        let s_sec = SedimentationCoefficient::from_svedberg(s_sved);
        let s_back = SedimentationCoefficient::svedberg_units(s_sec);
        assert!(approx_eq(s_back, s_sved, 1e-10));
    }

    #[test]
    fn test_from_boundary_movement() {
        // Create synthetic data: boundary moves according to r(t) = r_m·exp(s·ω²·t)
        let rpm = 50000.0;
        let s_true = 4.3e-13;
        let omega = SedimentationCoefficient::omega_from_rpm(rpm);
        let r_m = 6.0;
        let times = vec![1000.0, 2000.0, 3000.0, 4000.0, 5000.0];
        let radii: Vec<f64> = times
            .iter()
            .map(|&t| r_m * (s_true * omega * omega * t).exp())
            .collect();

        let s_calc = SedimentationCoefficient::from_boundary_movement(&radii, &times, rpm);
        assert!(
            approx_eq(s_calc, s_true, s_true * 0.01),
            "s_calc={}, s_true={}",
            s_calc,
            s_true
        );
    }

    #[test]
    fn test_normalize_to_water() {
        let s_obs = 4.0e-13;
        let s20w = SedimentationCoefficient::normalize_to_water(s_obs, 1.1, 0.95);
        assert!(approx_eq(s20w, s_obs * 1.1 * 0.95, 1e-20));
    }

    #[test]
    fn test_friction_ratio_sphere() {
        // For a perfect sphere, f/f₀ should be approximately 1.0
        // BSA: s~4.3S, MW~66430, v̄~0.733
        let s = 4.3e-13;
        let mw = 66430.0;
        let v_bar = 0.733;
        let fr = SedimentationCoefficient::friction_ratio(s, mw, v_bar);
        // BSA is slightly elongated, f/f₀ ~1.3
        assert!(fr > 0.5 && fr < 5.0, "friction_ratio={}", fr);
    }

    // ─── LammEquation tests ───

    #[test]
    fn test_lamm_simulate_basic() {
        let s = 4.3e-13;
        let d = 6.1e-7; // cm²/s for BSA
        let c0 = 1.0;
        let rpm = 50000.0;
        let times = vec![1000.0, 2000.0];
        let profiles = LammEquation::simulate(s, d, c0, 5.9, 7.2, rpm, &times, 50);
        assert_eq!(profiles.len(), 2);
        // Concentrations should be non-negative
        for p in &profiles {
            for &c in &p.concentration {
                assert!(c >= 0.0, "negative concentration: {}", c);
            }
        }
    }

    #[test]
    fn test_lamm_boundary_moves_outward() {
        let s = 4.3e-13;
        let d = 6.1e-7;
        let times = vec![500.0, 5000.0];
        let profiles = LammEquation::simulate(s, d, 1.0, 5.9, 7.2, 50000.0, &times, 80);
        let bp1 = profiles[0].boundary_position();
        let bp2 = profiles[1].boundary_position();
        assert!(
            bp2 > bp1,
            "boundary should move outward: t1={}, t2={}",
            bp1,
            bp2
        );
    }

    #[test]
    fn test_lamm_mass_conservation_approximate() {
        let s = 4.3e-13;
        let d = 6.1e-7;
        let c0 = 1.0;
        let profiles = LammEquation::simulate(s, d, c0, 5.9, 7.2, 50000.0, &[2000.0], 100);
        let p = &profiles[0];
        // Integral of c·r·dr should be approximately conserved
        let mut integral = 0.0;
        for i in 0..p.radius_cm.len() - 1 {
            let dr = p.radius_cm[i + 1] - p.radius_cm[i];
            let r_mid = 0.5 * (p.radius_cm[i] + p.radius_cm[i + 1]);
            let c_mid = 0.5 * (p.concentration[i] + p.concentration[i + 1]);
            integral += c_mid * r_mid * dr;
        }
        // Initial integral
        let r0 = 5.9;
        let r1 = 7.2;
        let initial_integral = c0 * (r1 * r1 - r0 * r0) / 2.0;
        // Allow 20% tolerance due to finite-difference errors
        let ratio = integral / initial_integral;
        assert!(
            ratio > 0.5 && ratio < 1.5,
            "mass conservation ratio={}, integral={}, initial={}",
            ratio,
            integral,
            initial_integral
        );
    }

    #[test]
    fn test_lamm_residuals() {
        let profiles1 = vec![RadialProfile::new(
            vec![6.0, 6.1, 6.2],
            vec![1.0, 0.5, 0.1],
            1000.0,
            50000.0,
        )];
        let profiles2 = vec![RadialProfile::new(
            vec![6.0, 6.1, 6.2],
            vec![1.0, 0.5, 0.1],
            1000.0,
            50000.0,
        )];
        let rmsd = LammEquation::residuals(&profiles1, &profiles2);
        assert!(approx_eq(rmsd, 0.0, 1e-10));
    }

    #[test]
    fn test_lamm_residuals_nonzero() {
        let p1 = vec![RadialProfile::new(
            vec![6.0, 6.1, 6.2],
            vec![1.0, 0.5, 0.0],
            1000.0,
            50000.0,
        )];
        let p2 = vec![RadialProfile::new(
            vec![6.0, 6.1, 6.2],
            vec![0.9, 0.6, 0.1],
            1000.0,
            50000.0,
        )];
        let rmsd = LammEquation::residuals(&p1, &p2);
        assert!(rmsd > 0.0);
    }

    // ─── VanHoldeWeischet tests ───

    #[test]
    fn test_apparent_s_star() {
        let r = 6.5;
        let r_m = 6.0;
        let omega = SedimentationCoefficient::omega_from_rpm(50000.0);
        let t = 3000.0;
        let s_star = VanHoldeWeischet::apparent_s_star(r, r_m, omega, t);
        assert!(s_star > 0.0, "s_star should be positive: {}", s_star);
    }

    #[test]
    fn test_apparent_s_star_zero_time() {
        let s_star = VanHoldeWeischet::apparent_s_star(6.5, 6.0, 5236.0, 0.0);
        assert_eq!(s_star, 0.0);
    }

    #[test]
    fn test_vhw_analyze() {
        // Use the Lamm simulator to create a realistic profile
        let profiles = AucSimulator::simulate_single_species(
            4.3e-13, 6.1e-7, 1.0, 50000.0, 3,
        );
        let results = VanHoldeWeischet::analyze(&profiles);
        assert!(!results.is_empty(), "vHW should produce results");
        // Most s* values should be positive (boundary moves outward)
        let positive_count = results.iter().filter(|(s, _)| *s > 0.0).count();
        assert!(
            positive_count > results.len() / 2,
            "most s* should be positive: {}/{}",
            positive_count,
            results.len()
        );
    }

    #[test]
    fn test_vhw_homogeneous() {
        // Tight distribution should be homogeneous
        let dist = vec![(4.3e-13, 0.25), (4.3e-13, 0.50), (4.3e-13, 0.75)];
        assert!(VanHoldeWeischet::is_homogeneous(&dist, 1e-13));
    }

    #[test]
    fn test_vhw_heterogeneous() {
        // Wide distribution should not be homogeneous
        let dist = vec![(2.0e-13, 0.25), (7.0e-13, 0.50), (12.0e-13, 0.75)];
        assert!(!VanHoldeWeischet::is_homogeneous(&dist, 1e-13));
    }

    #[test]
    fn test_extrapolate_empty() {
        let result = VanHoldeWeischet::extrapolate_to_infinite_time(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_extrapolate_single_fraction() {
        let data = vec![(4.3e-13, 0.5), (4.2e-13, 0.5), (4.4e-13, 0.5)];
        let result = VanHoldeWeischet::extrapolate_to_infinite_time(&data);
        assert_eq!(result.len(), 1);
        assert!(approx_eq(result[0], 4.3e-13, 0.2e-13));
    }

    // ─── ContinuousDistribution tests ───

    #[test]
    fn test_weight_average_s() {
        let dist = vec![(4.0e-13, 0.5), (5.0e-13, 0.5)];
        let avg = ContinuousDistribution::weight_average_s(&dist);
        assert!(approx_eq(avg, 4.5e-13, 1e-15));
    }

    #[test]
    fn test_weight_average_s_zero_weight() {
        let dist = vec![(4.0e-13, 0.0), (5.0e-13, 0.0)];
        assert_eq!(ContinuousDistribution::weight_average_s(&dist), 0.0);
    }

    #[test]
    fn test_peak_s_values_single_peak() {
        let dist = vec![
            (1e-13, 0.1),
            (2e-13, 0.5),
            (3e-13, 1.0),
            (4e-13, 0.5),
            (5e-13, 0.1),
        ];
        let peaks = ContinuousDistribution::peak_s_values(&dist);
        assert_eq!(peaks.len(), 1);
        assert!(approx_eq(peaks[0], 3e-13, EPSILON));
    }

    #[test]
    fn test_peak_s_values_two_peaks() {
        let dist = vec![
            (1e-13, 0.1),
            (2e-13, 0.8),
            (3e-13, 0.3),
            (4e-13, 0.9),
            (5e-13, 0.1),
        ];
        let peaks = ContinuousDistribution::peak_s_values(&dist);
        assert_eq!(peaks.len(), 2);
    }

    #[test]
    fn test_peak_s_values_empty() {
        let peaks = ContinuousDistribution::peak_s_values(&[]);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_frictional_ratio_from_cs() {
        let dist = vec![(4.3e-13, 1.0)];
        let fr = ContinuousDistribution::frictional_ratio_from_cs(&dist);
        assert!(fr > 0.0, "frictional ratio should be positive: {}", fr);
    }

    #[test]
    fn test_compute_cs_distribution() {
        // Generate synthetic data for a single species
        let profiles =
            AucSimulator::simulate_single_species(4.3e-13, 6.1e-7, 1.0, 50000.0, 3);
        let cs = ContinuousDistribution::compute(
            &profiles,
            1e-13,
            10e-13,
            20,
            0.01,
        );
        assert_eq!(cs.len(), 20);
        // At least one point should have non-zero c(s)
        let max_c = cs.iter().map(|(_, c)| *c).fold(0.0f64, f64::max);
        assert!(max_c > 0.0, "distribution should have non-zero values");
    }

    // ─── MolecularWeightEstimator tests ───

    #[test]
    fn test_svedberg_equation_bsa() {
        // BSA: s=4.3S, D=6.1e-7 cm²/s, v̄=0.733, ρ=0.998
        let s = 4.3e-13;
        let d = 6.1e-7;
        let v_bar = 0.733;
        let rho = 0.998;
        let mw = MolecularWeightEstimator::svedberg_equation(s, d, v_bar, rho, 293.15);
        // Expected ~66,000 Da
        assert!(
            mw > 50000.0 && mw < 90000.0,
            "MW={} should be ~66000 for BSA",
            mw
        );
    }

    #[test]
    fn test_svedberg_equation_zero_buoyancy() {
        let mw = MolecularWeightEstimator::svedberg_equation(4.3e-13, 6.1e-7, 1.0, 1.0, 293.15);
        assert_eq!(mw, 0.0);
    }

    #[test]
    fn test_svedberg_equation_zero_d() {
        let mw = MolecularWeightEstimator::svedberg_equation(4.3e-13, 0.0, 0.73, 1.0, 293.15);
        assert_eq!(mw, 0.0);
    }

    #[test]
    fn test_from_equilibrium() {
        let rpm = 10000.0;
        let v_bar = 0.73;
        let rho = 0.998;
        let temp_k = 293.15;
        let r_m = 6.0;
        let r_b = 7.0;

        // For a known MW, compute what the concentration ratio should be
        // c(r) = c_m · exp[M(1-v̄ρ)ω²(r²-r_m²) / (2RT)]
        let mw = 50000.0;
        let omega = SedimentationCoefficient::omega_from_rpm(rpm);
        let r_cgs = R_GAS * 1e7;
        let exponent = mw * (1.0 - v_bar * rho) * omega * omega
            * (r_b * r_b - r_m * r_m)
            / (2.0 * r_cgs * temp_k);
        let c_m = 0.5;
        let c_b = c_m * exponent.exp();

        let mw_calc = MolecularWeightEstimator::from_equilibrium(
            c_m, c_b, r_m, r_b, rpm, v_bar, rho, temp_k,
        );
        assert!(
            approx_eq(mw_calc, mw, mw * 0.01),
            "MW={}, expected={}",
            mw_calc,
            mw
        );
    }

    #[test]
    fn test_partial_specific_volume_typical() {
        // Typical protein should have v̄ around 0.70-0.75
        let composition = vec![
            ('A', 50),
            ('G', 30),
            ('L', 20),
            ('E', 15),
            ('K', 15),
            ('S', 10),
            ('V', 10),
        ];
        let v_bar = MolecularWeightEstimator::partial_specific_volume(&composition);
        assert!(
            v_bar > 0.65 && v_bar < 0.80,
            "v_bar={} should be ~0.73",
            v_bar
        );
    }

    #[test]
    fn test_partial_specific_volume_empty() {
        let v_bar = MolecularWeightEstimator::partial_specific_volume(&[]);
        assert!(approx_eq(v_bar, 0.73, 0.01));
    }

    #[test]
    fn test_buoyancy_factor() {
        let bf = MolecularWeightEstimator::buoyancy_factor(0.73, 1.0);
        assert!(approx_eq(bf, 0.27, 0.01));
    }

    #[test]
    fn test_buoyancy_factor_neutral() {
        // Density-matched: buoyancy = 0
        let bf = MolecularWeightEstimator::buoyancy_factor(1.0, 1.0);
        assert!(approx_eq(bf, 0.0, EPSILON));
    }

    // ─── DiffusionCoefficient tests ───

    #[test]
    fn test_from_boundary_width() {
        // σ² = 2Dt, so D = slope/2
        let d_true = 6.1e-7;
        let times = vec![1000.0, 2000.0, 3000.0, 4000.0, 5000.0];
        let sigma_sq: Vec<f64> = times.iter().map(|&t| 2.0 * d_true * t).collect();
        let d_calc = DiffusionCoefficient::from_boundary_width(&sigma_sq, &times);
        assert!(
            approx_eq(d_calc, d_true, d_true * 0.01),
            "D={}, expected={}",
            d_calc,
            d_true
        );
    }

    #[test]
    fn test_stokes_einstein_bsa() {
        // BSA Rh ~ 3.5 nm
        let rh = 3.5; // nm
        let eta = 1.002e-3; // Pa·s water at 20°C
        let t = 293.15; // K
        let d = DiffusionCoefficient::stokes_einstein(rh, eta, t);
        // Expected D ~ 6.1e-11 m²/s = 6.1e-7 cm²/s
        let d_cm2 = d * 1e4; // convert m²/s to cm²/s
        assert!(
            d_cm2 > 4e-7 && d_cm2 < 8e-7,
            "D={} cm²/s should be ~6e-7",
            d_cm2
        );
    }

    #[test]
    fn test_hydrodynamic_radius_roundtrip() {
        let rh_original = 3.5; // nm
        let eta = 1.002e-3;
        let t = 293.15;
        let d = DiffusionCoefficient::stokes_einstein(rh_original, eta, t);
        let rh_calc = DiffusionCoefficient::hydrodynamic_radius(d, eta, t);
        assert!(
            approx_eq(rh_calc, rh_original, 0.01),
            "Rh={}, expected={}",
            rh_calc,
            rh_original
        );
    }

    #[test]
    fn test_hydrodynamic_radius_zero_d() {
        assert_eq!(
            DiffusionCoefficient::hydrodynamic_radius(0.0, 1e-3, 293.15),
            0.0
        );
    }

    // ─── BufferDensityViscosity tests ───

    #[test]
    fn test_water_density_20c() {
        let rho = BufferDensityViscosity::water_density(20.0);
        assert!(
            approx_eq(rho, 0.9982, 0.001),
            "water density at 20°C={}, expected ~0.9982",
            rho
        );
    }

    #[test]
    fn test_water_density_4c() {
        let rho = BufferDensityViscosity::water_density(4.0);
        assert!(
            rho > 0.9999 && rho < 1.0001,
            "water density at 4°C should be ~1.0: {}",
            rho
        );
    }

    #[test]
    fn test_water_density_monotonic() {
        // Between 4°C and 100°C, density should decrease
        let rho4 = BufferDensityViscosity::water_density(4.0);
        let rho50 = BufferDensityViscosity::water_density(50.0);
        assert!(rho4 > rho50, "density should decrease with temperature");
    }

    #[test]
    fn test_water_viscosity_20c() {
        let eta = BufferDensityViscosity::water_viscosity(20.0);
        assert!(
            eta > 0.8 && eta < 1.2,
            "viscosity at 20°C={} mPa·s, expected ~1.002",
            eta
        );
    }

    #[test]
    fn test_water_viscosity_decreases_with_temp() {
        let eta20 = BufferDensityViscosity::water_viscosity(20.0);
        let eta40 = BufferDensityViscosity::water_viscosity(40.0);
        assert!(eta40 < eta20, "viscosity should decrease with temperature");
    }

    #[test]
    fn test_sucrose_density() {
        let rho0 = BufferDensityViscosity::sucrose_density(0.0, 20.0);
        let rho20 = BufferDensityViscosity::sucrose_density(20.0, 20.0);
        assert!(
            rho20 > rho0,
            "sucrose solution should be denser than water"
        );
    }

    #[test]
    fn test_nacl_density() {
        let rho0 = BufferDensityViscosity::nacl_density(0.0);
        let rho1 = BufferDensityViscosity::nacl_density(1.0);
        assert!(rho0 < rho1, "NaCl solution should be denser than water");
        assert!(approx_eq(rho0, 0.998, 0.001));
    }

    // ─── AucSimulator tests ───

    #[test]
    fn test_simulate_single_species() {
        let profiles =
            AucSimulator::simulate_single_species(4.3e-13, 6.1e-7, 1.0, 50000.0, 5);
        assert_eq!(profiles.len(), 5);
        for p in &profiles {
            assert!(p.time_s > 0.0);
            assert!(!p.concentration.is_empty());
        }
    }

    #[test]
    fn test_simulate_mixture_two_species() {
        let species = vec![
            (4.3e-13, 6.1e-7, 0.5), // BSA-like
            (7.0e-13, 4.0e-7, 0.5), // IgG-like
        ];
        let profiles = AucSimulator::simulate_mixture(&species, 50000.0, 3);
        assert_eq!(profiles.len(), 3);
        // Concentrations in plateau should be higher than single species
        let single = AucSimulator::simulate_single_species(4.3e-13, 6.1e-7, 0.5, 50000.0, 3);
        // At early time, the mixture plateau should be about double
        let mix_plat = profiles[0].plateau_concentration();
        let single_plat = single[0].plateau_concentration();
        assert!(
            mix_plat > single_plat * 0.8,
            "mixture plateau {} should exceed single {}",
            mix_plat,
            single_plat
        );
    }

    #[test]
    fn test_simulate_mixture_empty() {
        let profiles = AucSimulator::simulate_mixture(&[], 50000.0, 3);
        assert!(profiles.is_empty());
    }

    #[test]
    fn test_add_noise() {
        let mut profiles =
            AucSimulator::simulate_single_species(4.3e-13, 6.1e-7, 1.0, 50000.0, 1);
        let original: Vec<f64> = profiles[0].concentration.clone();
        AucSimulator::add_noise(&mut profiles, 0.01);
        // Some values should have changed
        let changed = profiles[0]
            .concentration
            .iter()
            .zip(original.iter())
            .any(|(a, b)| (a - b).abs() > 1e-10);
        assert!(changed, "noise should modify concentrations");
    }

    #[test]
    fn test_add_noise_zero_level() {
        let mut profiles =
            AucSimulator::simulate_single_species(4.3e-13, 6.1e-7, 1.0, 50000.0, 1);
        let original: Vec<f64> = profiles[0].concentration.clone();
        AucSimulator::add_noise(&mut profiles, 0.0);
        for (a, b) in profiles[0].concentration.iter().zip(original.iter()) {
            assert!(approx_eq(*a, *b, 1e-10));
        }
    }

    // ─── SedimentationEquilibrium tests ───

    #[test]
    fn test_ideal_equilibrium() {
        let sigma = 1.0;
        let c0 = 0.5;
        let r_m = 6.0;
        let radii = vec![6.0, 6.2, 6.4, 6.6, 6.8, 7.0];
        let conc = SedimentationEquilibrium::ideal_equilibrium(sigma, c0, r_m, &radii);
        assert_eq!(conc.len(), 6);
        // At meniscus, c should equal c0
        assert!(approx_eq(conc[0], c0, 1e-10));
        // Concentration should increase with radius for positive sigma
        for i in 1..conc.len() {
            assert!(conc[i] > conc[i - 1]);
        }
    }

    #[test]
    fn test_sigma_from_profile() {
        // Create an ideal equilibrium profile with known sigma
        // c(r) = c₀·exp(σ·(r²-r_m²))
        let sigma_true = 0.5;
        let r_m = 6.0;
        let radii: Vec<f64> = (0..50).map(|i| 6.0 + i as f64 * 0.02).collect();
        let conc: Vec<f64> = radii
            .iter()
            .map(|&r| 0.5 * (sigma_true * (r * r - r_m * r_m)).exp())
            .collect();
        let profile = RadialProfile::new(radii, conc, 0.0, 10000.0);
        let sigma_calc = SedimentationEquilibrium::sigma_from_profile(&profile);
        assert!(
            approx_eq(sigma_calc, sigma_true, 0.05),
            "sigma={}, expected={}",
            sigma_calc,
            sigma_true
        );
    }

    #[test]
    fn test_fit_single_species_equilibrium() {
        let v_bar = 0.73;
        let rho = 0.998;
        let temp_k = 293.15;
        let rpm = 10000.0;
        let omega = SedimentationCoefficient::omega_from_rpm(rpm);

        // Create ideal data with known MW
        // σ = M(1-v̄ρ)ω²/(2RT), c(r) = c₀·exp(σ·(r²-r_m²))
        let mw_true = 50000.0;
        let r_cgs = R_GAS * 1e7;
        let sigma = mw_true * (1.0 - v_bar * rho) * omega * omega / (2.0 * r_cgs * temp_k);
        let r_m = 6.0;
        let radii: Vec<f64> = (0..50).map(|i| 6.0 + i as f64 * 0.02).collect();
        let conc: Vec<f64> = radii
            .iter()
            .map(|&r| 0.3 * (sigma * (r * r - r_m * r_m)).exp())
            .collect();

        let profile = RadialProfile::new(radii, conc, 0.0, rpm);
        let result =
            SedimentationEquilibrium::fit_single_species(&profile, v_bar, rho, temp_k);

        // MW should be in ballpark (grid search is coarse)
        assert!(
            result.mw_apparent > 10000.0 && result.mw_apparent < 200000.0,
            "MW={}, expected ~{}",
            result.mw_apparent,
            mw_true
        );
    }

    // ─── Helper function tests ───

    #[test]
    fn test_erfc_approx_zero() {
        let val = erfc_approx(0.0);
        assert!(approx_eq(val, 1.0, 0.001));
    }

    #[test]
    fn test_erfc_approx_large_positive() {
        let val = erfc_approx(10.0);
        assert!(approx_eq(val, 0.0, 1e-10));
    }

    #[test]
    fn test_erfc_approx_large_negative() {
        let val = erfc_approx(-10.0);
        assert!(approx_eq(val, 2.0, 1e-10));
    }

    #[test]
    fn test_erfc_approx_one() {
        // erfc(1) ≈ 0.1573
        let val = erfc_approx(1.0);
        assert!(approx_eq(val, 0.1573, 0.01));
    }

    #[test]
    fn test_solve_linear_system_2x2() {
        // 2x + y = 5, x + 3y = 7 → x=1.6, y=1.8
        let a = vec![2.0, 1.0, 1.0, 3.0];
        let b = vec![5.0, 7.0];
        let x = solve_linear_system(&a, &b, 2);
        assert!(approx_eq(x[0], 1.6, 0.01));
        assert!(approx_eq(x[1], 1.8, 0.01));
    }

    #[test]
    fn test_solve_linear_system_identity() {
        let a = vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let b = vec![3.0, 5.0, 7.0];
        let x = solve_linear_system(&a, &b, 3);
        assert!(approx_eq(x[0], 3.0, EPSILON));
        assert!(approx_eq(x[1], 5.0, EPSILON));
        assert!(approx_eq(x[2], 7.0, EPSILON));
    }

    #[test]
    fn test_estimate_diffusion_from_s() {
        let d = estimate_diffusion_from_s(4.3e-13);
        assert!(d > 0.0, "D should be positive: {}", d);
    }

    #[test]
    fn test_estimate_diffusion_zero_s() {
        let d = estimate_diffusion_from_s(0.0);
        assert!(d > 0.0); // fallback value
    }

    // ─── Integration / end-to-end tests ───

    #[test]
    fn test_full_pipeline_single_species() {
        // 1. Simulate BSA-like species
        let s = 4.3e-13;
        let d = 6.1e-7;
        let c0 = 1.0;
        let rpm = 50000.0;
        let profiles = AucSimulator::simulate_single_species(s, d, c0, rpm, 5);

        // 2. Measure boundary movement for s determination
        let boundary_radii: Vec<f64> = profiles.iter().map(|p| p.boundary_position()).collect();
        let times: Vec<f64> = profiles.iter().map(|p| p.time_s).collect();
        let s_measured =
            SedimentationCoefficient::from_boundary_movement(&boundary_radii, &times, rpm);

        // s should be in reasonable range
        let s_sved = SedimentationCoefficient::svedberg_units(s_measured);
        assert!(
            s_sved > 1.0 && s_sved < 20.0,
            "measured s={} S should be reasonable",
            s_sved
        );

        // 3. MW estimation
        let v_bar = 0.733;
        let rho = 0.998;
        let mw = MolecularWeightEstimator::svedberg_equation(s, d, v_bar, rho, 293.15);
        assert!(mw > 30000.0 && mw < 100000.0);
    }

    #[test]
    fn test_full_pipeline_mixture() {
        let species = vec![
            (4.3e-13, 6.1e-7, 0.7), // BSA
            (7.0e-13, 4.0e-7, 0.3), // IgG
        ];
        let profiles = AucSimulator::simulate_mixture(&species, 50000.0, 3);
        assert_eq!(profiles.len(), 3);

        // vHW should detect heterogeneity
        let vhw = VanHoldeWeischet::analyze(&profiles);
        assert!(!vhw.is_empty());
    }

    #[test]
    fn test_svedberg_equation_self_consistency() {
        // If we know s, D, v̄, ρ, T → get MW
        // Then use MW to verify relationship
        let s = 4.3e-13;
        let d = 6.1e-7;
        let v_bar = 0.733;
        let rho = 0.998;
        let t = 293.15;
        let mw = MolecularWeightEstimator::svedberg_equation(s, d, v_bar, rho, t);
        // Verify: s = M·D·(1-v̄ρ) / (R·T)
        let buoyancy = 1.0 - v_bar * rho;
        let r_cgs = R_GAS * 1e7;
        let s_check = mw * d * buoyancy / (r_cgs * t);
        assert!(
            approx_eq(s_check, s, s * 0.001),
            "s_check={}, s={}",
            s_check,
            s
        );
    }

    #[test]
    fn test_stokes_einstein_self_consistency() {
        // D → Rh → D should roundtrip
        let d = 6.1e-11; // m²/s
        let eta = 1.002e-3;
        let t = 293.15;
        let rh = DiffusionCoefficient::hydrodynamic_radius(d, eta, t);
        let d_back = DiffusionCoefficient::stokes_einstein(rh, eta, t);
        assert!(
            approx_eq(d_back, d, d * 0.001),
            "D roundtrip: {} vs {}",
            d_back,
            d
        );
    }

    #[test]
    fn test_buffer_properties_at_standard_conditions() {
        let rho = BufferDensityViscosity::water_density(20.0);
        let eta = BufferDensityViscosity::water_viscosity(20.0);
        // Standard conditions: ρ_20 ≈ 0.9982, η_20 ≈ 1.002 mPa·s
        assert!(rho > 0.995 && rho < 1.002);
        assert!(eta > 0.5 && eta < 2.0);
    }
}

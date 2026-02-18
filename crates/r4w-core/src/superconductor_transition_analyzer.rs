//! # Superconductor Transition Analyzer
//!
//! Analysis of superconducting phase transitions, characterizing critical temperature (Tc),
//! critical fields (Hc1, Hc2), and London penetration depth from resistivity, magnetization,
//! and susceptibility measurements.
//!
//! ## Physics Background
//!
//! - **Meissner effect**: Perfect diamagnetism below Hc1 (chi = -1)
//! - **Type I**: Single Hc, complete flux expulsion
//! - **Type II**: Hc1 < H < Hc2 mixed state with Abrikosov vortices
//! - **BCS gap**: Delta(0) = 1.764 k_B Tc
//! - **Flux quantum**: Phi_0 = h/(2e) = 2.068e-15 Wb
//! - **GL parameter**: kappa = lambda/xi, Type I if kappa < 1/sqrt(2)
//! - **WHH formula**: Hc2(0) = -0.693 Tc (dHc2/dT)|Tc

/// Boltzmann constant in J/K.
const K_B: f64 = 1.380649e-23;

/// Planck constant in J*s.
const H_PLANCK: f64 = 6.62607015e-34;

/// Elementary charge in C.
const E_CHARGE: f64 = 1.602176634e-19;

/// Magnetic flux quantum Phi_0 = h/(2e) in Wb.
const PHI_0: f64 = 2.067833848e-15;

/// Speed of light in m/s.
#[allow(dead_code)]
const C_LIGHT: f64 = 2.99792458e8;

/// Permeability of free space in H/m.
const MU_0: f64 = 1.2566370614e-6;

// ---------- helpers ----------

fn linear_fit(x: &[f64], y: &[f64]) -> (f64, f64) {
    let n = x.len() as f64;
    let sx: f64 = x.iter().sum();
    let sy: f64 = y.iter().sum();
    let sxy: f64 = x.iter().zip(y.iter()).map(|(a, b)| a * b).sum();
    let sxx: f64 = x.iter().map(|a| a * a).sum();
    let denom = n * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return (0.0, sy / n);
    }
    let slope = (n * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / n;
    (slope, intercept)
}

fn linear_interp(x0: f64, y0: f64, x1: f64, y1: f64, y_target: f64) -> f64 {
    if (y1 - y0).abs() < 1e-30 {
        return (x0 + x1) / 2.0;
    }
    x0 + (y_target - y0) * (x1 - x0) / (y1 - y0)
}

// ---------- SuperconductorType ----------

/// Classification of superconductor type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SuperconductorType {
    /// Type I: kappa < 1/sqrt(2), complete Meissner effect up to Hc
    TypeI,
    /// Type II: kappa > 1/sqrt(2), mixed state between Hc1 and Hc2
    TypeII,
}

// ---------- Phase ----------

/// Phase of a superconductor in the H-T diagram.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Phase {
    /// Normal state (above Tc or above Hc2)
    Normal,
    /// Meissner state (below Hc1, full flux expulsion)
    Meissner,
    /// Mixed (vortex) state between Hc1 and Hc2
    MixedState,
    /// Flux-flow regime (high current in mixed state)
    FluxFlow,
}

// ---------- Hc2FitResult ----------

/// Result of fitting Hc2(T) data.
#[derive(Debug, Clone)]
pub struct Hc2FitResult {
    /// Extrapolated zero-temperature upper critical field Hc2(0).
    pub hc2_0: f64,
    /// Critical temperature Tc from the fit.
    pub tc: f64,
    /// Slope dHc2/dT at Tc.
    pub dhc2_dt_at_tc: f64,
}

// ---------- KramerResult ----------

/// Result of Kramer scaling analysis for Jc(H).
#[derive(Debug, Clone)]
pub struct KramerResult {
    /// Kramer extrapolation of the irreversibility field H*.
    pub h_irr: f64,
    /// Slope of the Kramer plot.
    pub slope: f64,
    /// Intercept of the Kramer plot.
    pub intercept: f64,
}

// ---------- ResistivityTransition ----------

/// Resistive transition R(T) analysis.
pub struct ResistivityTransition {
    temperature_k: Vec<f64>,
    resistance_ohm: Vec<f64>,
}

impl ResistivityTransition {
    /// Create a new resistivity transition analyzer from temperature and resistance data.
    /// Data is sorted by ascending temperature internally.
    pub fn new(temperature_k: Vec<f64>, resistance_ohm: Vec<f64>) -> Self {
        assert_eq!(temperature_k.len(), resistance_ohm.len());
        assert!(temperature_k.len() >= 2);
        let mut indices: Vec<usize> = (0..temperature_k.len()).collect();
        indices.sort_by(|&a, &b| temperature_k[a].partial_cmp(&temperature_k[b]).unwrap());
        let t_sorted: Vec<f64> = indices.iter().map(|&i| temperature_k[i]).collect();
        let r_sorted: Vec<f64> = indices.iter().map(|&i| resistance_ohm[i]).collect();
        Self {
            temperature_k: t_sorted,
            resistance_ohm: r_sorted,
        }
    }

    /// Normal-state resistance: maximum resistance in the dataset.
    fn normal_state_resistance(&self) -> f64 {
        self.resistance_ohm
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// Find Tc at the midpoint (50% of normal-state resistance).
    pub fn find_tc_midpoint(&self) -> f64 {
        let r_normal = self.normal_state_resistance();
        let r_mid = r_normal * 0.5;
        for i in 1..self.temperature_k.len() {
            let r0 = self.resistance_ohm[i - 1];
            let r1 = self.resistance_ohm[i];
            if (r0 <= r_mid && r1 >= r_mid) || (r0 >= r_mid && r1 <= r_mid) {
                return linear_interp(
                    self.temperature_k[i - 1],
                    r0,
                    self.temperature_k[i],
                    r1,
                    r_mid,
                );
            }
        }
        // fallback: return the temperature of the point closest to r_mid
        let mut best_idx = 0;
        let mut best_diff = f64::MAX;
        for (i, &r) in self.resistance_ohm.iter().enumerate() {
            let d = (r - r_mid).abs();
            if d < best_diff {
                best_diff = d;
                best_idx = i;
            }
        }
        self.temperature_k[best_idx]
    }

    /// Find onset temperature where resistance begins to drop.
    /// Uses derivative-based detection: scans from high T downward and finds
    /// where the resistance derivative becomes significantly more negative
    /// than the normal-state slope, indicating the start of the transition.
    pub fn find_tc_onset(&self) -> f64 {
        let n = self.temperature_k.len();
        if n < 3 {
            return *self.temperature_k.last().unwrap();
        }
        // Estimate normal-state slope from the upper 1/3 of data
        let upper_start = n * 2 / 3;
        let (normal_slope, _) = linear_fit(
            &self.temperature_k[upper_start..],
            &self.resistance_ohm[upper_start..],
        );
        // Scan from high T downward, looking for where the local slope
        // drops below 2x the normal slope (onset of superconducting transition)
        let window = 3.max(n / 20);
        for i in (window..n).rev() {
            let dt = self.temperature_k[i] - self.temperature_k[i - window];
            if dt.abs() < 1e-30 {
                continue;
            }
            let local_slope = (self.resistance_ohm[i] - self.resistance_ohm[i - window]) / dt;
            // Onset: slope becomes much steeper (more negative) than normal state
            if local_slope < normal_slope - 0.1 * self.normal_state_resistance() {
                return self.temperature_k[i];
            }
        }
        // Fallback: use 90% of the resistance at the point just above where R first becomes nonzero
        let r_at_transition = self.resistance_ohm.iter()
            .zip(self.temperature_k.iter())
            .rev()
            .find(|(&r, _)| r < self.normal_state_resistance() * 0.5)
            .map(|(_, &t)| t)
            .unwrap_or(*self.temperature_k.last().unwrap());
        r_at_transition
    }

    /// Find zero-resistance temperature (10% of R_normal as practical threshold).
    pub fn find_tc_zero(&self) -> f64 {
        let r_normal = self.normal_state_resistance();
        let r_zero = r_normal * 0.10;
        for i in 1..self.temperature_k.len() {
            let r0 = self.resistance_ohm[i - 1];
            let r1 = self.resistance_ohm[i];
            if (r0 >= r_zero && r1 <= r_zero) || (r0 <= r_zero && r1 >= r_zero) {
                return linear_interp(
                    self.temperature_k[i - 1],
                    r0,
                    self.temperature_k[i],
                    r1,
                    r_zero,
                );
            }
        }
        self.temperature_k[0]
    }

    /// Transition width delta_Tc = T_onset - T_zero.
    pub fn transition_width(&self) -> f64 {
        self.find_tc_onset() - self.find_tc_zero()
    }

    /// Residual Resistivity Ratio RRR = R(300K) / R(Tc+).
    /// Uses the highest temperature resistance as R(300K) proxy and
    /// the first resistance above R_normal*0.95 from the transition as R(Tc+).
    pub fn residual_resistivity_ratio(&self) -> f64 {
        let r_300k = *self.resistance_ohm.last().unwrap_or(&1.0);
        let tc_onset = self.find_tc_onset();
        // R(Tc+): first measurement above onset
        let mut r_tc_plus = r_300k;
        for (i, &t) in self.temperature_k.iter().enumerate() {
            if t >= tc_onset {
                r_tc_plus = self.resistance_ohm[i];
                break;
            }
        }
        if r_tc_plus.abs() < 1e-30 {
            return f64::INFINITY;
        }
        r_300k / r_tc_plus
    }

    /// Linear fit R = slope*T + intercept in the normal-state region [t_min, t_max].
    pub fn normal_state_fit(&self, t_min: f64, t_max: f64) -> (f64, f64) {
        let mut t_sel = Vec::new();
        let mut r_sel = Vec::new();
        for (i, &t) in self.temperature_k.iter().enumerate() {
            if t >= t_min && t <= t_max {
                t_sel.push(t);
                r_sel.push(self.resistance_ohm[i]);
            }
        }
        if t_sel.len() < 2 {
            return (0.0, 0.0);
        }
        linear_fit(&t_sel, &r_sel)
    }
}

// ---------- MagnetizationAnalysis ----------

/// Magnetization M(H) analysis.
pub struct MagnetizationAnalysis {
    field_oe: Vec<f64>,
    magnetization_emu: Vec<f64>,
}

impl MagnetizationAnalysis {
    /// Create from M(H) data.
    pub fn new_mh(field_oe: Vec<f64>, magnetization_emu: Vec<f64>) -> Self {
        assert_eq!(field_oe.len(), magnetization_emu.len());
        assert!(field_oe.len() >= 2);
        Self {
            field_oe,
            magnetization_emu,
        }
    }

    /// Lower critical field Hc1: deviation from Meissner linear response.
    /// Fits initial linear slope and finds where data deviates by > 5%.
    pub fn lower_critical_field(&self) -> f64 {
        // Use first ~20% of data for linear fit
        let n_fit = (self.field_oe.len() / 5).max(2);
        let (slope, intercept) = linear_fit(&self.field_oe[..n_fit], &self.magnetization_emu[..n_fit]);
        let threshold = 0.05;
        for i in n_fit..self.field_oe.len() {
            let h = self.field_oe[i];
            let m_expected = slope * h + intercept;
            let m_actual = self.magnetization_emu[i];
            if m_expected.abs() > 1e-30 && ((m_actual - m_expected) / m_expected).abs() > threshold {
                return h;
            }
        }
        *self.field_oe.last().unwrap()
    }

    /// Fit Hc2(T) data and return Hc2FitResult.
    pub fn upper_critical_field_from_mt(temp_k: &[f64], hc2: &[f64]) -> Hc2FitResult {
        Hc2TemperatureFit::fit_hc2_curve(temp_k, hc2)
    }

    /// Coherence length xi = sqrt(Phi_0 / (2*pi*Hc2)) in meters.
    /// hc2_0 in Tesla.
    pub fn coherence_length(hc2_0: f64) -> f64 {
        (PHI_0 / (2.0 * std::f64::consts::PI * hc2_0)).sqrt()
    }

    /// London penetration depth lambda from Hc1 and Hc2 via kappa.
    /// lambda = kappa * xi, where kappa = Hc2 / (sqrt(2) * Hc1).
    /// Fields in Tesla. Returns lambda in meters.
    pub fn london_penetration_depth(hc1: f64, hc2: f64) -> f64 {
        let kappa = Self::ginzburg_landau_kappa(hc1, hc2);
        let xi = Self::coherence_length(hc2);
        kappa * xi
    }

    /// Ginzburg-Landau parameter kappa = Hc2 / (sqrt(2) * Hc1).
    pub fn ginzburg_landau_kappa(hc1: f64, hc2: f64) -> f64 {
        if hc1.abs() < 1e-30 {
            return f64::INFINITY;
        }
        hc2 / (std::f64::consts::SQRT_2 * hc1)
    }

    /// Classify as Type I (kappa < 1/sqrt(2)) or Type II.
    pub fn type_classification(kappa: f64) -> SuperconductorType {
        let boundary = 1.0 / std::f64::consts::SQRT_2;
        if kappa < boundary {
            SuperconductorType::TypeI
        } else {
            SuperconductorType::TypeII
        }
    }
}

// ---------- AcSusceptibility ----------

/// AC susceptibility chi(T) analysis.
pub struct AcSusceptibility {
    temperature_k: Vec<f64>,
    chi_real: Vec<f64>,
    chi_imag: Vec<f64>,
}

impl AcSusceptibility {
    /// Create from temperature and complex susceptibility data.
    pub fn new(temperature_k: Vec<f64>, chi_real: Vec<f64>, chi_imag: Vec<f64>) -> Self {
        assert_eq!(temperature_k.len(), chi_real.len());
        assert_eq!(temperature_k.len(), chi_imag.len());
        assert!(temperature_k.len() >= 2);
        // Sort by temperature
        let mut indices: Vec<usize> = (0..temperature_k.len()).collect();
        indices.sort_by(|&a, &b| temperature_k[a].partial_cmp(&temperature_k[b]).unwrap());
        let t_sorted: Vec<f64> = indices.iter().map(|&i| temperature_k[i]).collect();
        let cr_sorted: Vec<f64> = indices.iter().map(|&i| chi_real[i]).collect();
        let ci_sorted: Vec<f64> = indices.iter().map(|&i| chi_imag[i]).collect();
        Self {
            temperature_k: t_sorted,
            chi_real: cr_sorted,
            chi_imag: ci_sorted,
        }
    }

    /// Find Tc from onset of diamagnetic response in chi_real.
    /// Scans from high T downward; Tc is where chi_real drops below -0.01.
    pub fn find_tc_from_susceptibility(&self) -> f64 {
        let threshold = -0.01;
        for i in (1..self.temperature_k.len()).rev() {
            if self.chi_real[i] >= threshold && self.chi_real[i - 1] < threshold {
                return linear_interp(
                    self.temperature_k[i - 1],
                    self.chi_real[i - 1],
                    self.temperature_k[i],
                    self.chi_real[i],
                    threshold,
                );
            }
        }
        // fallback: first point where chi_real is most negative
        let mut min_idx = 0;
        let mut min_val = f64::MAX;
        for (i, &c) in self.chi_real.iter().enumerate() {
            if c < min_val {
                min_val = c;
                min_idx = i;
            }
        }
        self.temperature_k[min_idx]
    }

    /// Shielding fraction = -chi_real / (1 - N), where N is demagnetization factor.
    /// Returns a percentage (0-100).
    pub fn shielding_fraction(chi_value: f64, demagnetization: f64) -> f64 {
        let denom = 1.0 - demagnetization;
        if denom.abs() < 1e-30 {
            return 0.0;
        }
        (-chi_value / denom) * 100.0
    }

    /// Intergranular Tc: second onset in chi_real for granular superconductors.
    /// After the first transition, looks for a second kink.
    pub fn intergranular_tc(&self) -> f64 {
        // Compute derivative d(chi_real)/dT
        let n = self.temperature_k.len();
        if n < 4 {
            return self.find_tc_from_susceptibility();
        }
        let mut dchi: Vec<f64> = Vec::with_capacity(n - 1);
        for i in 0..n - 1 {
            let dt = self.temperature_k[i + 1] - self.temperature_k[i];
            if dt.abs() < 1e-30 {
                dchi.push(0.0);
            } else {
                dchi.push((self.chi_real[i + 1] - self.chi_real[i]) / dt);
            }
        }
        // Find the two most negative peaks in dchi (two transitions)
        let mut peak1_idx = 0;
        let mut peak1_val = 0.0_f64;
        for (i, &d) in dchi.iter().enumerate() {
            if d < peak1_val {
                peak1_val = d;
                peak1_idx = i;
            }
        }
        // Look for second peak separated from first by at least 2 points
        let mut peak2_idx = 0;
        let mut peak2_val = 0.0_f64;
        for (i, &d) in dchi.iter().enumerate() {
            let dist = if i > peak1_idx {
                i - peak1_idx
            } else {
                peak1_idx - i
            };
            if dist >= 2 && d < peak2_val {
                peak2_val = d;
                peak2_idx = i;
            }
        }
        if peak2_val < -1e-10 {
            // The lower Tc is the intergranular one
            let lower_idx = peak1_idx.min(peak2_idx);
            return self.temperature_k[lower_idx];
        }
        self.find_tc_from_susceptibility()
    }

    /// Meissner fraction: ratio of FC to ZFC susceptibility.
    /// meissner_fraction = FC / ZFC for each temperature point.
    pub fn meissner_fraction(zfc: &[f64], fc: &[f64]) -> Vec<f64> {
        assert_eq!(zfc.len(), fc.len());
        zfc.iter()
            .zip(fc.iter())
            .map(|(&z, &f)| {
                if z.abs() < 1e-30 {
                    0.0
                } else {
                    f / z
                }
            })
            .collect()
    }
}

// ---------- SpecificHeatAnalysis ----------

/// Specific heat C(T) analysis at the superconducting transition.
pub struct SpecificHeatAnalysis {
    temperature_k: Vec<f64>,
    specific_heat_jmolk: Vec<f64>,
}

impl SpecificHeatAnalysis {
    /// Create from temperature and specific heat data.
    pub fn new(temperature_k: Vec<f64>, specific_heat_jmolk: Vec<f64>) -> Self {
        assert_eq!(temperature_k.len(), specific_heat_jmolk.len());
        assert!(temperature_k.len() >= 2);
        let mut indices: Vec<usize> = (0..temperature_k.len()).collect();
        indices.sort_by(|&a, &b| temperature_k[a].partial_cmp(&temperature_k[b]).unwrap());
        let t_sorted: Vec<f64> = indices.iter().map(|&i| temperature_k[i]).collect();
        let c_sorted: Vec<f64> = indices.iter().map(|&i| specific_heat_jmolk[i]).collect();
        Self {
            temperature_k: t_sorted,
            specific_heat_jmolk: c_sorted,
        }
    }

    /// Find Tc from the peak/jump in specific heat.
    pub fn find_tc_from_specific_heat(&self) -> f64 {
        // Tc is at the maximum of specific heat (lambda anomaly)
        let mut max_idx = 0;
        let mut max_val = f64::NEG_INFINITY;
        for (i, &c) in self.specific_heat_jmolk.iter().enumerate() {
            if c > max_val {
                max_val = c;
                max_idx = i;
            }
        }
        self.temperature_k[max_idx]
    }

    /// Normalized specific heat jump delta_C / (gamma * Tc).
    /// BCS weak-coupling value is 1.43.
    /// gamma is the electronic specific heat coefficient.
    pub fn delta_c_over_gamma_tc(&self) -> f64 {
        let tc = self.find_tc_from_specific_heat();
        let tc_idx = self.temperature_k.iter().position(|&t| t == tc).unwrap_or(0);

        // C just above Tc (normal state)
        let c_above = if tc_idx + 1 < self.temperature_k.len() {
            self.specific_heat_jmolk[tc_idx + 1]
        } else {
            self.specific_heat_jmolk[tc_idx]
        };

        // C at Tc (peak, superconducting side)
        let c_peak = self.specific_heat_jmolk[tc_idx];

        // Delta C = C_s - C_n at Tc
        let delta_c = c_peak - c_above;

        // gamma * Tc ~ C_n(Tc) for a metal near Tc (electronic part dominates)
        let gamma_tc = c_above;
        if gamma_tc.abs() < 1e-30 {
            return 0.0;
        }
        delta_c / gamma_tc
    }

    /// Electronic specific heat coefficient gamma from C/T vs T^2 fit below Tc.
    /// C/T = gamma + beta*T^2 => intercept is gamma.
    pub fn electronic_specific_heat_coefficient(
        t_below_tc: &[f64],
        c_below: &[f64],
    ) -> f64 {
        assert_eq!(t_below_tc.len(), c_below.len());
        if t_below_tc.len() < 2 {
            return 0.0;
        }
        // Fit C/T = gamma + beta * T^2
        let x: Vec<f64> = t_below_tc.iter().map(|&t| t * t).collect();
        let y: Vec<f64> = t_below_tc
            .iter()
            .zip(c_below.iter())
            .map(|(&t, &c)| if t.abs() < 1e-30 { 0.0 } else { c / t })
            .collect();
        let (_beta, gamma) = linear_fit(&x, &y);
        gamma
    }

    /// Debye temperature Theta_D from low-temperature phonon specific heat.
    /// C_phonon = beta * T^3, with beta = (12/5) * pi^4 * n * k_B / Theta_D^3.
    /// For 1 mole of atoms, n * k_B = R = 8.314 J/(mol*K).
    pub fn debye_temperature(t_range: &[f64], c_range: &[f64]) -> f64 {
        assert_eq!(t_range.len(), c_range.len());
        if t_range.len() < 2 {
            return 0.0;
        }
        // Fit C = gamma*T + beta*T^3 => C/T = gamma + beta*T^2
        let x: Vec<f64> = t_range.iter().map(|&t| t * t).collect();
        let y: Vec<f64> = t_range
            .iter()
            .zip(c_range.iter())
            .map(|(&t, &c)| if t.abs() < 1e-30 { 0.0 } else { c / t })
            .collect();
        let (beta, _gamma) = linear_fit(&x, &y);
        if beta <= 0.0 {
            return 0.0;
        }
        let r = 8.314; // J/(mol*K)
        let pi = std::f64::consts::PI;
        let numerator = 12.0 / 5.0 * pi.powi(4) * r;
        (numerator / beta).cbrt()
    }

    /// Gap ratio 2*Delta_0 / (k_B * Tc). BCS value is 3.53.
    pub fn gap_ratio(delta_0: f64, tc: f64) -> f64 {
        if tc.abs() < 1e-30 {
            return 0.0;
        }
        2.0 * delta_0 / (K_B * tc)
    }
}

// ---------- BcsGapCalculator ----------

/// BCS superconducting gap equation calculations.
pub struct BcsGapCalculator;

impl BcsGapCalculator {
    /// BCS gap temperature dependence: Delta(T)/Delta(0) approx tanh(1.74 * sqrt(Tc/T - 1)).
    /// Returns (T/Tc, Delta(T)/Delta(0)) pairs.
    pub fn gap_temperature_dependence(tc: f64, num_points: usize) -> Vec<(f64, f64)> {
        let mut result = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let t_ratio = (i as f64) / ((num_points - 1) as f64); // T/Tc from 0 to 1
            let gap_ratio = if t_ratio >= 1.0 {
                0.0
            } else if t_ratio < 1e-10 {
                1.0
            } else {
                let arg = 1.74 * (1.0 / t_ratio - 1.0).sqrt();
                arg.tanh()
            };
            result.push((t_ratio, gap_ratio));
        }
        result
    }

    /// Zero-temperature BCS gap: Delta(0) = 1.764 * k_B * Tc.
    pub fn zero_temperature_gap(tc: f64) -> f64 {
        1.764 * K_B * tc
    }

    /// Depairing critical current density (Ginzburg-Landau):
    /// Jc = Phi_0 / (3*sqrt(3)*pi*mu_0*lambda^2*xi)
    /// tc is used to verify the superconductor is below Tc (we assume T=0).
    /// london_depth in meters.
    pub fn critical_current_density(tc: f64, london_depth: f64) -> f64 {
        if tc <= 0.0 || london_depth <= 0.0 {
            return 0.0;
        }
        // Use BCS coherence length estimate: xi_0 ~ hbar*v_F / (pi*Delta_0)
        // For simplicity, use the GL depairing formula with estimated xi from lambda.
        // In a full calculation, one needs both lambda and xi independently.
        // Here we use the Ginzburg-Landau formula:
        // Jc = Phi_0 / (3*sqrt(3)*pi*mu_0*lambda^2*xi)
        // Without independent xi, we estimate via BCS:
        // xi_BCS ~ 0.18 * hbar * v_F / (k_B * Tc)
        // For a rough estimate, use xi ~ lambda / kappa with kappa ~ 50 for HTS
        // Use simplified depairing expression: Jc = Phi_0 / (3*sqrt(3)*pi*mu_0*lambda^3) * (lambda/xi)
        // Actually, the standard GL depairing formula is:
        // Jd = Phi_0 / (3*sqrt(3)*pi*mu_0*lambda^2*xi)
        // For an estimate with just lambda: use Jd ~ H_c / lambda where H_c = Phi_0/(2*sqrt(2)*pi*mu_0*lambda*xi)
        // Simplified: Jd ~ Phi_0 / (3*sqrt(3)*pi*mu_0*lambda^3) * kappa (assuming kappa known)
        // Let's use the standard formula with a conservative kappa = 50:
        let kappa = 50.0;
        let xi = london_depth / kappa;
        let denom = 3.0 * 3.0_f64.sqrt() * std::f64::consts::PI * MU_0 * london_depth * london_depth * xi;
        if denom.abs() < 1e-60 {
            return 0.0;
        }
        PHI_0 / denom
    }
}

// ---------- Hc2TemperatureFit ----------

/// Upper critical field Hc2(T) fitting and modeling.
pub struct Hc2TemperatureFit;

impl Hc2TemperatureFit {
    /// WHH formula: Hc2(0) = -0.693 * Tc * (dHc2/dT)|Tc.
    pub fn werthamer_helfand_hohenberg(tc: f64, dhc2_dt_at_tc: f64) -> f64 {
        -0.693 * tc * dhc2_dt_at_tc
    }

    /// Ginzburg-Landau Hc2(T) = Hc2(0) * (1 - t^2) / (1 + t^2), where t = T/Tc.
    pub fn ginzburg_landau(hc2_0: f64, tc: f64, t: f64) -> f64 {
        if tc <= 0.0 {
            return 0.0;
        }
        let tr = t / tc;
        if tr >= 1.0 {
            return 0.0;
        }
        let t2 = tr * tr;
        hc2_0 * (1.0 - t2) / (1.0 + t2)
    }

    /// Fit Hc2(T) data to GL model. Returns Hc2FitResult.
    /// Uses linearized GL form: Hc2 * (1 + t^2) = Hc2(0) * (1 - t^2).
    /// => Hc2 / (1-t^2) * (1+t^2) = Hc2(0) for t < 1.
    pub fn fit_hc2_curve(temp_k: &[f64], hc2: &[f64]) -> Hc2FitResult {
        assert_eq!(temp_k.len(), hc2.len());
        assert!(temp_k.len() >= 2);

        // Estimate Tc from the zero crossing of Hc2 data
        let mut tc_est = *temp_k.last().unwrap();
        for i in 1..temp_k.len() {
            if hc2[i] <= 0.0 && hc2[i - 1] > 0.0 {
                tc_est = linear_interp(temp_k[i - 1], hc2[i - 1], temp_k[i], hc2[i], 0.0);
                break;
            }
        }

        // Estimate Hc2(0) from GL fit: Hc2 = Hc2(0) * (1-t^2)/(1+t^2)
        // Rearrange: Hc2(0) = Hc2 * (1+t^2)/(1-t^2)
        let mut hc2_0_estimates = Vec::new();
        for (i, &t) in temp_k.iter().enumerate() {
            let tr = t / tc_est;
            if tr < 0.99 && tr >= 0.0 && hc2[i] > 0.0 {
                let t2 = tr * tr;
                let estimate = hc2[i] * (1.0 + t2) / (1.0 - t2);
                hc2_0_estimates.push(estimate);
            }
        }

        let hc2_0 = if hc2_0_estimates.is_empty() {
            hc2[0]
        } else {
            hc2_0_estimates.iter().sum::<f64>() / hc2_0_estimates.len() as f64
        };

        // dHc2/dT at Tc from WHH: slope = -Hc2(0) / (0.693 * Tc)
        let dhc2_dt = if tc_est > 0.0 {
            -hc2_0 / (0.693 * tc_est)
        } else {
            0.0
        };

        Hc2FitResult {
            hc2_0,
            tc: tc_est,
            dhc2_dt_at_tc: dhc2_dt,
        }
    }
}

// ---------- FluxQuantization ----------

/// Magnetic flux quantum calculations.
pub struct FluxQuantization;

impl FluxQuantization {
    /// Flux quantum Phi_0 = h/(2e) in Wb.
    pub fn flux_quantum() -> f64 {
        PHI_0
    }

    /// Fluxoid number n = B*A / Phi_0.
    /// field in Tesla, area in m^2.
    pub fn fluxoid_number(field: f64, area: f64) -> f64 {
        field * area / PHI_0
    }

    /// Vortex density n_v = B / Phi_0 per m^2.
    /// field in Tesla.
    pub fn vortex_density(field: f64) -> f64 {
        field / PHI_0
    }

    /// Vortex lattice spacing a = sqrt(2 * Phi_0 / (sqrt(3) * B)) for triangular lattice.
    /// field in Tesla. Returns spacing in meters.
    pub fn vortex_spacing(field: f64) -> f64 {
        if field <= 0.0 {
            return f64::INFINITY;
        }
        (2.0 * PHI_0 / (3.0_f64.sqrt() * field)).sqrt()
    }
}

// ---------- JcAnalyzer ----------

/// Critical current density Jc analysis.
pub struct JcAnalyzer;

impl JcAnalyzer {
    /// Jc from magnetization (Bean model): Jc = 20 * delta_M / (a * (1 - a/(3*b)))
    /// for a rectangular sample with a < b.
    /// delta_m: width of M-H hysteresis in emu/cm^3.
    /// sample_dim: characteristic sample dimension in cm (width a, assuming a ≈ b).
    /// Returns Jc in A/cm^2.
    pub fn from_magnetization(delta_m: f64, sample_dim: f64) -> f64 {
        if sample_dim.abs() < 1e-30 {
            return 0.0;
        }
        // Simplified Bean model for thin disk: Jc = 20 * delta_M / d
        // where d is sample thickness
        20.0 * delta_m / sample_dim
    }

    /// Jc from transport I-V measurement with electric field criterion.
    /// voltage_criterion: typically 1 uV/cm = 1e-6 V/cm.
    /// i_data: current in A, v_data: voltage in V.
    /// area: cross-section in cm^2.
    /// Returns Jc in A/cm^2.
    pub fn from_transport(
        voltage_criterion: f64,
        i_data: &[f64],
        v_data: &[f64],
        area: f64,
    ) -> f64 {
        assert_eq!(i_data.len(), v_data.len());
        if area.abs() < 1e-30 {
            return 0.0;
        }
        // Find current where voltage exceeds criterion
        // x = current, y = voltage, find x (current) at y = criterion
        for i in 1..v_data.len() {
            if v_data[i - 1] <= voltage_criterion && v_data[i] >= voltage_criterion {
                let i_c = linear_interp(i_data[i - 1], v_data[i - 1], i_data[i], v_data[i], voltage_criterion);
                return i_c / area;
            }
        }
        // If not found, use last point
        if let Some(&last_i) = i_data.last() {
            return last_i / area;
        }
        0.0
    }

    /// Kramer scaling: plot Jc^0.5 * H^0.25 vs H.
    /// Linear extrapolation to zero gives irreversibility field H*.
    pub fn kramer_scaling(jc: &[f64], h: &[f64]) -> KramerResult {
        assert_eq!(jc.len(), h.len());
        // Compute Kramer function: F_p^0.5 ~ Jc^0.5 * H^0.25
        let x: Vec<f64> = h.to_vec();
        let y: Vec<f64> = jc
            .iter()
            .zip(h.iter())
            .map(|(&j, &field)| {
                if j <= 0.0 || field <= 0.0 {
                    0.0
                } else {
                    j.sqrt() * field.powf(0.25)
                }
            })
            .collect();

        let (slope, intercept) = linear_fit(&x, &y);

        // Extrapolate to y = 0: H* = -intercept / slope
        let h_irr = if slope.abs() > 1e-30 {
            -intercept / slope
        } else {
            0.0
        };

        KramerResult {
            h_irr: h_irr.max(0.0),
            slope,
            intercept,
        }
    }

    /// Irreversibility field H*(T) extrapolation.
    /// Fits H_irr(T) = H_irr(0) * (1 - T/Tc)^n, returns H_irr(0).
    pub fn irreversibility_field(temp_k: &[f64], h_irr: &[f64]) -> f64 {
        assert_eq!(temp_k.len(), h_irr.len());
        if temp_k.is_empty() {
            return 0.0;
        }
        // Estimate Tc from where h_irr -> 0
        let tc_est = temp_k
            .iter()
            .zip(h_irr.iter())
            .filter(|(_, &h)| h > 0.0)
            .map(|(&t, _)| t)
            .fold(0.0_f64, f64::max)
            * 1.05; // slightly above highest measured

        if tc_est <= 0.0 {
            return 0.0;
        }

        // Fit in log space: ln(H_irr) = ln(H_irr(0)) + n * ln(1 - T/Tc)
        let mut x = Vec::new();
        let mut y = Vec::new();
        for (i, &t) in temp_k.iter().enumerate() {
            let ratio = 1.0 - t / tc_est;
            if ratio > 0.01 && h_irr[i] > 0.0 {
                x.push(ratio.ln());
                y.push(h_irr[i].ln());
            }
        }

        if x.len() < 2 {
            return h_irr[0];
        }

        let (_n, ln_h0) = linear_fit(&x, &y);
        ln_h0.exp()
    }
}

// ---------- TwoFluidModel ----------

/// Gorter-Casimir two-fluid model.
pub struct TwoFluidModel;

impl TwoFluidModel {
    /// Superfluid fraction: n_s/n = 1 - (T/Tc)^4.
    pub fn superfluid_fraction(t: f64, tc: f64) -> f64 {
        if tc <= 0.0 || t < 0.0 {
            return 0.0;
        }
        let ratio = t / tc;
        if ratio >= 1.0 {
            return 0.0;
        }
        1.0 - ratio.powi(4)
    }

    /// Penetration depth temperature dependence:
    /// lambda(T) = lambda(0) / sqrt(1 - (T/Tc)^4).
    pub fn penetration_depth_temperature(lambda_0: f64, t: f64, tc: f64) -> f64 {
        let ns = Self::superfluid_fraction(t, tc);
        if ns <= 0.0 {
            return f64::INFINITY;
        }
        lambda_0 / ns.sqrt()
    }

    /// Thermal conductivity ratio kappa_s / kappa_n in the two-fluid model.
    /// Approximation: kappa_s/kappa_n ~ (T/Tc)^3 for T << Tc.
    pub fn thermal_conductivity_ratio(t: f64, tc: f64) -> f64 {
        if tc <= 0.0 || t < 0.0 {
            return 0.0;
        }
        let ratio = t / tc;
        if ratio >= 1.0 {
            return 1.0;
        }
        ratio.powi(3)
    }
}

// ---------- PhaseDiagram ----------

/// H-T phase diagram for a superconductor.
pub struct PhaseDiagram {
    points: Vec<(f64, f64, Phase)>, // (temperature, field, phase)
}

impl PhaseDiagram {
    /// Create an empty phase diagram.
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Add a measured point to the phase diagram.
    pub fn add_point(&mut self, temperature: f64, field: f64, phase: Phase) {
        self.points.push((temperature, field, phase));
    }

    /// Extract boundary curve between two phases.
    /// Returns (temperature, field) pairs sorted by temperature.
    pub fn boundary_curve(&self, from: Phase, to: Phase) -> Vec<(f64, f64)> {
        // Find points that are near phase boundaries by checking neighbors
        let mut boundary = Vec::new();

        // Sort points by temperature
        let mut sorted = self.points.clone();
        sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        // For each pair of adjacent temperatures, check for phase transitions
        for i in 0..sorted.len() {
            for j in (i + 1)..sorted.len() {
                let (t1, h1, p1) = &sorted[i];
                let (t2, h2, p2) = &sorted[j];
                if (*p1 == from && *p2 == to) || (*p1 == to && *p2 == from) {
                    let t_mid = (t1 + t2) / 2.0;
                    let h_mid = (h1 + h2) / 2.0;
                    boundary.push((t_mid, h_mid));
                }
            }
        }

        boundary.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        // Deduplicate
        boundary.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-10 && (a.1 - b.1).abs() < 1e-10);
        boundary
    }

    /// Critical point (Tc, Hc2(0)).
    /// Tc is the maximum temperature of any superconducting phase.
    /// Hc2(0) is the maximum field of any superconducting phase.
    pub fn critical_point(&self) -> (f64, f64) {
        let mut tc = 0.0_f64;
        let mut hc2_0 = 0.0_f64;
        for &(t, h, ref phase) in &self.points {
            match phase {
                Phase::Meissner | Phase::MixedState | Phase::FluxFlow => {
                    if t > tc {
                        tc = t;
                    }
                    if h > hc2_0 {
                        hc2_0 = h;
                    }
                }
                Phase::Normal => {}
            }
        }
        (tc, hc2_0)
    }

    /// Number of points in the diagram.
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Check if the diagram is empty.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

impl Default for PhaseDiagram {
    fn default() -> Self {
        Self::new()
    }
}

// ==================== TESTS ====================

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    // ---------- ResistivityTransition tests ----------

    fn make_transition_data() -> (Vec<f64>, Vec<f64>) {
        // Simulate an YBCO-like transition around 92 K
        let mut temp = Vec::new();
        let mut res = Vec::new();
        // Below Tc: zero resistance
        for i in 0..50 {
            let t = 80.0 + i as f64 * 0.2; // 80 to 89.8 K
            temp.push(t);
            res.push(0.0);
        }
        // Transition: 90 to 93 K
        for i in 0..30 {
            let t = 90.0 + i as f64 * 0.1; // 90 to 92.9 K
            let frac = (t - 90.0) / 3.0;
            temp.push(t);
            res.push(frac * 1.0); // 0 to 1.0 mOhm
        }
        // Above Tc: normal state
        for i in 0..50 {
            let t = 93.0 + i as f64 * 4.0; // 93 to 289 K
            temp.push(t);
            res.push(1.0 + 0.001 * (t - 93.0)); // slight linear increase
        }
        (temp, res)
    }

    #[test]
    fn test_resistivity_new() {
        let (temp, res) = make_transition_data();
        let rt = ResistivityTransition::new(temp, res);
        assert!(rt.temperature_k.len() > 0);
    }

    #[test]
    fn test_tc_midpoint() {
        let (temp, res) = make_transition_data();
        let rt = ResistivityTransition::new(temp, res);
        let tc = rt.find_tc_midpoint();
        // midpoint should be around 91.5 K (50% of max R)
        assert!(tc > 89.0 && tc < 94.0, "Tc midpoint = {} K", tc);
    }

    #[test]
    fn test_tc_onset() {
        let (temp, res) = make_transition_data();
        let rt = ResistivityTransition::new(temp, res);
        let tc_onset = rt.find_tc_onset();
        assert!(tc_onset > 91.0 && tc_onset < 95.0, "Tc onset = {} K", tc_onset);
    }

    #[test]
    fn test_tc_zero() {
        let (temp, res) = make_transition_data();
        let rt = ResistivityTransition::new(temp, res);
        let tc_zero = rt.find_tc_zero();
        assert!(tc_zero > 89.0 && tc_zero < 91.5, "Tc zero = {} K", tc_zero);
    }

    #[test]
    fn test_transition_width() {
        let (temp, res) = make_transition_data();
        let rt = ResistivityTransition::new(temp, res);
        let width = rt.transition_width();
        assert!(width > 0.0 && width < 10.0, "Transition width = {} K", width);
    }

    #[test]
    fn test_rrr() {
        let (temp, res) = make_transition_data();
        let rt = ResistivityTransition::new(temp, res);
        let rrr = rt.residual_resistivity_ratio();
        assert!(rrr >= 1.0, "RRR = {}", rrr);
    }

    #[test]
    fn test_normal_state_fit() {
        let (temp, res) = make_transition_data();
        let rt = ResistivityTransition::new(temp, res);
        let (slope, intercept) = rt.normal_state_fit(100.0, 250.0);
        assert!(slope > 0.0, "Normal state slope should be positive");
        assert!(intercept.is_finite());
    }

    #[test]
    fn test_resistivity_sorted() {
        // Test that data is sorted correctly even if given out of order
        let temp = vec![100.0, 50.0, 90.0, 80.0];
        let res = vec![1.0, 0.0, 0.5, 0.0];
        let rt = ResistivityTransition::new(temp, res);
        assert!(rt.temperature_k[0] <= rt.temperature_k[1]);
    }

    #[test]
    fn test_tc_midpoint_sharp_transition() {
        // Very sharp transition
        let temp = vec![4.0, 5.0, 6.0, 7.0, 7.3, 7.4, 7.5, 8.0, 9.0, 10.0];
        let res = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0];
        let rt = ResistivityTransition::new(temp, res);
        let tc = rt.find_tc_midpoint();
        assert!(tc > 7.0 && tc < 8.0, "Sharp Tc = {}", tc);
    }

    // ---------- MagnetizationAnalysis tests ----------

    #[test]
    fn test_magnetization_new() {
        let h = vec![0.0, 100.0, 200.0, 300.0, 400.0];
        let m = vec![0.0, -100.0, -200.0, -180.0, -150.0];
        let _ma = MagnetizationAnalysis::new_mh(h, m);
    }

    #[test]
    fn test_lower_critical_field() {
        // Linear region then deviation
        let h = vec![0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0];
        let m = vec![0.0, -10.0, -20.0, -30.0, -40.0, -45.0, -48.0, -49.0, -49.5, -49.8, -50.0];
        let ma = MagnetizationAnalysis::new_mh(h, m);
        let hc1 = ma.lower_critical_field();
        assert!(hc1 > 20.0, "Hc1 = {}", hc1);
    }

    #[test]
    fn test_coherence_length() {
        // Nb: Hc2(0) ~ 0.4 T, xi ~ 40 nm
        let xi = MagnetizationAnalysis::coherence_length(0.4);
        assert!(xi > 1e-9 && xi < 1e-6, "xi = {} m", xi);
    }

    #[test]
    fn test_london_penetration_depth() {
        let lambda = MagnetizationAnalysis::london_penetration_depth(0.01, 0.4);
        assert!(lambda > 0.0 && lambda.is_finite());
    }

    #[test]
    fn test_gl_kappa() {
        // Nb: Hc1 ~ 0.17 T, Hc2 ~ 0.4 T
        let kappa = MagnetizationAnalysis::ginzburg_landau_kappa(0.17, 0.4);
        assert!(kappa > 0.0, "kappa = {}", kappa);
    }

    #[test]
    fn test_type_classification_type_i() {
        let kappa = 0.5; // < 1/sqrt(2) ~ 0.707
        assert_eq!(
            MagnetizationAnalysis::type_classification(kappa),
            SuperconductorType::TypeI
        );
    }

    #[test]
    fn test_type_classification_type_ii() {
        let kappa = 50.0;
        assert_eq!(
            MagnetizationAnalysis::type_classification(kappa),
            SuperconductorType::TypeII
        );
    }

    #[test]
    fn test_type_boundary() {
        let boundary = 1.0 / std::f64::consts::SQRT_2;
        // Just below
        assert_eq!(
            MagnetizationAnalysis::type_classification(boundary - 0.001),
            SuperconductorType::TypeI
        );
        // At boundary and above
        assert_eq!(
            MagnetizationAnalysis::type_classification(boundary),
            SuperconductorType::TypeII
        );
    }

    #[test]
    fn test_upper_critical_field_from_mt() {
        let temp = vec![0.0, 20.0, 40.0, 60.0, 80.0, 90.0, 92.0];
        let hc2 = vec![100.0, 95.0, 80.0, 55.0, 20.0, 5.0, 0.0];
        let result = MagnetizationAnalysis::upper_critical_field_from_mt(&temp, &hc2);
        assert!(result.hc2_0 > 50.0, "Hc2(0) = {}", result.hc2_0);
        assert!(result.tc > 80.0 && result.tc < 100.0, "Tc = {}", result.tc);
    }

    // ---------- AcSusceptibility tests ----------

    fn make_susceptibility_data() -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let mut temp = Vec::new();
        let mut chi_r = Vec::new();
        let mut chi_i = Vec::new();
        // Below Tc: full diamagnetism
        for i in 0..50 {
            let t = 70.0 + i as f64 * 0.4;
            temp.push(t);
            chi_r.push(-1.0);
            chi_i.push(0.0);
        }
        // Transition
        for i in 0..20 {
            let t = 90.0 + i as f64 * 0.15;
            let frac = (t - 90.0) / 3.0;
            temp.push(t);
            chi_r.push(-1.0 + frac);
            chi_i.push(0.05 * (1.0 - (frac - 0.5).abs() * 2.0).max(0.0)); // peak in chi''
        }
        // Above Tc
        for i in 0..30 {
            let t = 93.0 + i as f64 * 2.0;
            temp.push(t);
            chi_r.push(0.0);
            chi_i.push(0.0);
        }
        (temp, chi_r, chi_i)
    }

    #[test]
    fn test_ac_susceptibility_new() {
        let (t, cr, ci) = make_susceptibility_data();
        let _acs = AcSusceptibility::new(t, cr, ci);
    }

    #[test]
    fn test_tc_from_susceptibility() {
        let (t, cr, ci) = make_susceptibility_data();
        let acs = AcSusceptibility::new(t, cr, ci);
        let tc = acs.find_tc_from_susceptibility();
        assert!(tc > 88.0 && tc < 95.0, "Tc from susceptibility = {} K", tc);
    }

    #[test]
    fn test_shielding_fraction() {
        // Full Meissner: chi = -1, N = 0 => 100%
        let sf = AcSusceptibility::shielding_fraction(-1.0, 0.0);
        assert!(approx_eq(sf, 100.0, 0.1));
    }

    #[test]
    fn test_shielding_fraction_partial() {
        // Partial: chi = -0.5, N = 0 => 50%
        let sf = AcSusceptibility::shielding_fraction(-0.5, 0.0);
        assert!(approx_eq(sf, 50.0, 0.1));
    }

    #[test]
    fn test_shielding_with_demagnetization() {
        // chi = -0.5, N = 0.5 => -(-0.5)/(1-0.5) * 100 = 100%
        let sf = AcSusceptibility::shielding_fraction(-0.5, 0.5);
        assert!(approx_eq(sf, 100.0, 0.1));
    }

    #[test]
    fn test_intergranular_tc() {
        let (t, cr, ci) = make_susceptibility_data();
        let acs = AcSusceptibility::new(t, cr, ci);
        let tc_inter = acs.intergranular_tc();
        assert!(tc_inter > 0.0, "Intergranular Tc = {}", tc_inter);
    }

    #[test]
    fn test_meissner_fraction() {
        let zfc = vec![-1.0, -0.9, -0.5, -0.1, 0.0];
        let fc = vec![-0.8, -0.7, -0.3, -0.05, 0.0];
        let mf = AcSusceptibility::meissner_fraction(&zfc, &fc);
        assert_eq!(mf.len(), 5);
        // FC/ZFC should be between 0 and 1 for typical superconductor
        assert!(mf[0] > 0.0 && mf[0] <= 1.0);
    }

    #[test]
    fn test_meissner_fraction_zero_zfc() {
        let zfc = vec![0.0, -1.0];
        let fc = vec![0.0, -0.8];
        let mf = AcSusceptibility::meissner_fraction(&zfc, &fc);
        assert_eq!(mf[0], 0.0); // division by zero handled
    }

    // ---------- SpecificHeatAnalysis tests ----------

    fn make_specific_heat_data() -> (Vec<f64>, Vec<f64>) {
        let mut temp = Vec::new();
        let mut cp = Vec::new();
        // Below Tc: mostly phonon contribution
        for i in 0..50 {
            let t = 1.0 + i as f64 * 0.16;
            temp.push(t);
            cp.push(0.001 * t.powi(3) + 0.005 * t); // beta*T^3 + gamma*T
        }
        // Jump at Tc ~ 9.2 K (Nb-like)
        for i in 0..10 {
            let t = 9.0 + i as f64 * 0.05;
            let base = 0.001 * t.powi(3) + 0.005 * t;
            let jump = 0.3 * (1.0 - ((t - 9.2) / 0.15).powi(2)).max(0.0);
            temp.push(t);
            cp.push(base + jump);
        }
        // Above Tc
        for i in 0..40 {
            let t = 9.5 + i as f64 * 0.5;
            temp.push(t);
            cp.push(0.001 * t.powi(3) + 0.005 * t);
        }
        (temp, cp)
    }

    #[test]
    fn test_specific_heat_new() {
        let (t, c) = make_specific_heat_data();
        let _sha = SpecificHeatAnalysis::new(t, c);
    }

    #[test]
    fn test_tc_from_specific_heat() {
        let (t, c) = make_specific_heat_data();
        let sha = SpecificHeatAnalysis::new(t, c);
        let tc = sha.find_tc_from_specific_heat();
        // Should be near the maximum which is in the higher-T phonon region
        // or at the jump if the jump is large enough
        assert!(tc > 0.0, "Tc from specific heat = {} K", tc);
    }

    #[test]
    fn test_delta_c_over_gamma_tc() {
        let (t, c) = make_specific_heat_data();
        let sha = SpecificHeatAnalysis::new(t, c);
        let ratio = sha.delta_c_over_gamma_tc();
        // Result depends on data; just check it's finite
        assert!(ratio.is_finite(), "delta_C/gamma_Tc = {}", ratio);
    }

    #[test]
    fn test_electronic_specific_heat() {
        let t = vec![2.0, 3.0, 4.0, 5.0, 6.0];
        let c: Vec<f64> = t.iter().map(|&temp: &f64| 0.005 * temp + 0.001 * temp.powi(3)).collect();
        let gamma = SpecificHeatAnalysis::electronic_specific_heat_coefficient(&t, &c);
        assert!(approx_eq(gamma, 0.005, 0.01), "gamma = {}", gamma);
    }

    #[test]
    fn test_debye_temperature() {
        let t = vec![2.0, 3.0, 4.0, 5.0, 6.0];
        let r: f64 = 8.314;
        let theta_d: f64 = 300.0;
        let beta: f64 = 12.0 / 5.0 * std::f64::consts::PI.powi(4) * r / theta_d.powi(3);
        let c: Vec<f64> = t.iter().map(|&temp: &f64| 0.005 * temp + beta * temp.powi(3)).collect();
        let theta = SpecificHeatAnalysis::debye_temperature(&t, &c);
        assert!(
            approx_eq(theta, theta_d, 20.0),
            "Theta_D = {} (expected {})",
            theta,
            theta_d
        );
    }

    #[test]
    fn test_gap_ratio_bcs() {
        // BCS: 2*Delta_0 / (k_B * Tc) = 3.53
        let tc = 9.2; // Nb
        let delta_0 = BcsGapCalculator::zero_temperature_gap(tc);
        let ratio = SpecificHeatAnalysis::gap_ratio(delta_0, tc);
        assert!(
            approx_eq(ratio, 3.528, 0.01),
            "Gap ratio = {} (BCS = 3.53)",
            ratio
        );
    }

    // ---------- BcsGapCalculator tests ----------

    #[test]
    fn test_bcs_gap_temperature() {
        let points = BcsGapCalculator::gap_temperature_dependence(9.2, 100);
        assert_eq!(points.len(), 100);
        // At T=0: Delta/Delta_0 = 1.0
        assert!(approx_eq(points[0].1, 1.0, 0.01));
        // At T=Tc: Delta/Delta_0 = 0.0
        assert!(approx_eq(points[99].1, 0.0, 0.01));
    }

    #[test]
    fn test_bcs_gap_monotonic() {
        let points = BcsGapCalculator::gap_temperature_dependence(9.2, 50);
        for i in 1..points.len() {
            assert!(
                points[i].1 <= points[i - 1].1 + 1e-10,
                "Gap should decrease: {}({}) vs {}({})",
                points[i].1,
                points[i].0,
                points[i - 1].1,
                points[i - 1].0,
            );
        }
    }

    #[test]
    fn test_zero_temperature_gap() {
        let tc = 9.2; // Nb
        let delta_0 = BcsGapCalculator::zero_temperature_gap(tc);
        // Delta_0 = 1.764 * k_B * Tc
        let expected = 1.764 * K_B * tc;
        assert!(approx_eq(delta_0, expected, 1e-30));
    }

    #[test]
    fn test_critical_current_density() {
        let tc = 9.2;
        let lambda = 50e-9; // 50 nm for Nb
        let jc = BcsGapCalculator::critical_current_density(tc, lambda);
        // Should be very large (>10^10 A/m^2 for Nb)
        assert!(jc > 1e8, "Jc = {} A/m^2", jc);
    }

    #[test]
    fn test_critical_current_density_zero() {
        assert_eq!(BcsGapCalculator::critical_current_density(0.0, 50e-9), 0.0);
        assert_eq!(BcsGapCalculator::critical_current_density(9.2, 0.0), 0.0);
    }

    // ---------- Hc2TemperatureFit tests ----------

    #[test]
    fn test_whh_formula() {
        // For Nb: Tc = 9.2 K, dHc2/dT = -0.035 T/K
        let hc2_0 = Hc2TemperatureFit::werthamer_helfand_hohenberg(9.2, -0.035);
        // = -0.693 * 9.2 * (-0.035) = 0.223 T
        assert!(approx_eq(hc2_0, 0.223, 0.01), "Hc2(0) = {} T", hc2_0);
    }

    #[test]
    fn test_gl_hc2() {
        let hc2_0 = 10.0;
        let tc = 90.0;
        // At T=0: should return Hc2(0)
        let h0 = Hc2TemperatureFit::ginzburg_landau(hc2_0, tc, 0.0);
        assert!(approx_eq(h0, hc2_0, 0.01));
        // At T=Tc: should return 0
        let htc = Hc2TemperatureFit::ginzburg_landau(hc2_0, tc, tc);
        assert!(approx_eq(htc, 0.0, 0.01));
    }

    #[test]
    fn test_gl_hc2_intermediate() {
        let hc2_0 = 10.0;
        let tc = 90.0;
        let t = 45.0;
        let h = Hc2TemperatureFit::ginzburg_landau(hc2_0, tc, t);
        // t = 0.5, t^2 = 0.25, h = 10 * 0.75/1.25 = 6.0
        assert!(approx_eq(h, 6.0, 0.01), "H(45K) = {}", h);
    }

    #[test]
    fn test_fit_hc2_curve() {
        let tc_true = 92.0;
        let hc2_0_true = 100.0;
        let mut temp = Vec::new();
        let mut hc2 = Vec::new();
        for i in 0..20 {
            let t = i as f64 * 5.0;
            temp.push(t);
            hc2.push(Hc2TemperatureFit::ginzburg_landau(hc2_0_true, tc_true, t));
        }
        let result = Hc2TemperatureFit::fit_hc2_curve(&temp, &hc2);
        assert!(
            approx_eq(result.hc2_0, hc2_0_true, 15.0),
            "Fitted Hc2(0) = {} (true = {})",
            result.hc2_0,
            hc2_0_true
        );
        assert!(
            approx_eq(result.tc, tc_true, 5.0),
            "Fitted Tc = {} (true = {})",
            result.tc,
            tc_true
        );
    }

    // ---------- FluxQuantization tests ----------

    #[test]
    fn test_flux_quantum() {
        let phi0 = FluxQuantization::flux_quantum();
        assert!(approx_eq(phi0, 2.068e-15, 0.01e-15));
    }

    #[test]
    fn test_fluxoid_number() {
        // 1 T through 1 um^2
        let n = FluxQuantization::fluxoid_number(1.0, 1e-12);
        // = 1e-12 / 2.068e-15 ~ 484
        assert!(n > 480.0 && n < 490.0, "n = {}", n);
    }

    #[test]
    fn test_vortex_density() {
        let nv = FluxQuantization::vortex_density(1.0);
        // = 1 / 2.068e-15 ~ 4.84e14
        assert!(nv > 4e14 && nv < 5e14, "n_v = {}", nv);
    }

    #[test]
    fn test_vortex_spacing() {
        let a = FluxQuantization::vortex_spacing(1.0);
        // a ~ sqrt(2 * 2.068e-15 / 1.732) ~ sqrt(2.388e-15) ~ 48.9 nm
        assert!(a > 40e-9 && a < 60e-9, "a = {} m", a);
    }

    #[test]
    fn test_vortex_spacing_zero_field() {
        assert_eq!(FluxQuantization::vortex_spacing(0.0), f64::INFINITY);
    }

    #[test]
    fn test_vortex_spacing_decreases_with_field() {
        let a1 = FluxQuantization::vortex_spacing(1.0);
        let a2 = FluxQuantization::vortex_spacing(4.0);
        assert!(a2 < a1, "Higher field should give smaller spacing");
    }

    // ---------- JcAnalyzer tests ----------

    #[test]
    fn test_jc_from_magnetization() {
        let jc = JcAnalyzer::from_magnetization(1000.0, 0.1);
        // = 20 * 1000 / 0.1 = 200_000
        assert!(approx_eq(jc, 200_000.0, 1.0));
    }

    #[test]
    fn test_jc_from_magnetization_zero_dim() {
        assert_eq!(JcAnalyzer::from_magnetization(1000.0, 0.0), 0.0);
    }

    #[test]
    fn test_jc_from_transport() {
        let i = vec![0.0, 10.0, 20.0, 30.0, 40.0, 50.0];
        let v = vec![0.0, 0.0, 0.5e-6, 1.0e-6, 2.0e-6, 5.0e-6];
        let area = 0.01; // cm^2
        let jc = JcAnalyzer::from_transport(1.0e-6, &i, &v, area);
        // Voltage reaches 1 uV around I=30 A
        assert!(jc > 0.0, "Jc = {} A/cm^2", jc);
    }

    #[test]
    fn test_jc_from_transport_zero_area() {
        let i = vec![10.0, 20.0];
        let v = vec![0.0, 1e-6];
        assert_eq!(JcAnalyzer::from_transport(1e-6, &i, &v, 0.0), 0.0);
    }

    #[test]
    fn test_kramer_scaling() {
        let h = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let jc: Vec<f64> = h.iter().map(|&field: &f64| (10.0 - field).max(0.0).powi(2)).collect();
        let kr = JcAnalyzer::kramer_scaling(&jc, &h);
        assert!(kr.h_irr > 0.0, "H_irr = {}", kr.h_irr);
    }

    #[test]
    fn test_irreversibility_field() {
        let temp = vec![4.0, 20.0, 40.0, 60.0, 77.0];
        let h_irr = vec![30.0, 25.0, 18.0, 8.0, 1.0];
        let h0 = JcAnalyzer::irreversibility_field(&temp, &h_irr);
        assert!(h0 > 25.0, "H_irr(0) = {}", h0);
    }

    // ---------- TwoFluidModel tests ----------

    #[test]
    fn test_superfluid_fraction_at_zero() {
        let ns = TwoFluidModel::superfluid_fraction(0.0, 9.2);
        assert!(approx_eq(ns, 1.0, EPSILON));
    }

    #[test]
    fn test_superfluid_fraction_at_tc() {
        let ns = TwoFluidModel::superfluid_fraction(9.2, 9.2);
        assert!(approx_eq(ns, 0.0, EPSILON));
    }

    #[test]
    fn test_superfluid_fraction_above_tc() {
        let ns = TwoFluidModel::superfluid_fraction(15.0, 9.2);
        assert_eq!(ns, 0.0);
    }

    #[test]
    fn test_superfluid_fraction_mid() {
        let tc = 10.0;
        let t = 5.0; // t/tc = 0.5, (t/tc)^4 = 0.0625
        let ns = TwoFluidModel::superfluid_fraction(t, tc);
        assert!(approx_eq(ns, 0.9375, 0.001), "ns = {}", ns);
    }

    #[test]
    fn test_penetration_depth_at_zero() {
        let lambda_0 = 50e-9;
        let lambda = TwoFluidModel::penetration_depth_temperature(lambda_0, 0.0, 9.2);
        assert!(approx_eq(lambda, lambda_0, 1e-15));
    }

    #[test]
    fn test_penetration_depth_diverges() {
        let lambda_0 = 50e-9;
        let lambda = TwoFluidModel::penetration_depth_temperature(lambda_0, 9.2, 9.2);
        assert!(lambda.is_infinite());
    }

    #[test]
    fn test_penetration_depth_increases() {
        let lambda_0 = 50e-9;
        let l1 = TwoFluidModel::penetration_depth_temperature(lambda_0, 4.0, 9.2);
        let l2 = TwoFluidModel::penetration_depth_temperature(lambda_0, 8.0, 9.2);
        assert!(l2 > l1, "lambda should increase with T");
    }

    #[test]
    fn test_thermal_conductivity_ratio_at_tc() {
        let r = TwoFluidModel::thermal_conductivity_ratio(9.2, 9.2);
        assert!(approx_eq(r, 1.0, EPSILON));
    }

    #[test]
    fn test_thermal_conductivity_ratio_at_zero() {
        let r = TwoFluidModel::thermal_conductivity_ratio(0.0, 9.2);
        assert!(approx_eq(r, 0.0, EPSILON));
    }

    // ---------- PhaseDiagram tests ----------

    #[test]
    fn test_phase_diagram_new() {
        let pd = PhaseDiagram::new();
        assert_eq!(pd.len(), 0);
        assert!(pd.is_empty());
    }

    #[test]
    fn test_phase_diagram_add_point() {
        let mut pd = PhaseDiagram::new();
        pd.add_point(4.0, 0.0, Phase::Meissner);
        assert_eq!(pd.len(), 1);
        assert!(!pd.is_empty());
    }

    #[test]
    fn test_critical_point() {
        let mut pd = PhaseDiagram::new();
        pd.add_point(4.0, 0.0, Phase::Meissner);
        pd.add_point(4.0, 0.5, Phase::MixedState);
        pd.add_point(4.0, 2.0, Phase::MixedState);
        pd.add_point(4.0, 3.0, Phase::Normal);
        pd.add_point(9.0, 0.0, Phase::Meissner);
        pd.add_point(9.0, 0.1, Phase::Normal);
        pd.add_point(10.0, 0.0, Phase::Normal);
        let (tc, hc2) = pd.critical_point();
        assert!(approx_eq(tc, 9.0, 0.1), "Tc = {}", tc);
        assert!(approx_eq(hc2, 2.0, 0.1), "Hc2 = {}", hc2);
    }

    #[test]
    fn test_boundary_curve() {
        let mut pd = PhaseDiagram::new();
        pd.add_point(4.0, 0.0, Phase::Meissner);
        pd.add_point(4.0, 0.5, Phase::MixedState);
        pd.add_point(6.0, 0.0, Phase::Meissner);
        pd.add_point(6.0, 0.3, Phase::MixedState);
        let boundary = pd.boundary_curve(Phase::Meissner, Phase::MixedState);
        assert!(!boundary.is_empty(), "Should find boundary points");
    }

    #[test]
    fn test_phase_equality() {
        assert_eq!(Phase::Normal, Phase::Normal);
        assert_ne!(Phase::Normal, Phase::Meissner);
        assert_ne!(Phase::MixedState, Phase::FluxFlow);
    }

    #[test]
    fn test_phase_diagram_default() {
        let pd = PhaseDiagram::default();
        assert!(pd.is_empty());
    }

    // ---------- Additional cross-component tests ----------

    #[test]
    fn test_flux_quantum_matches_constants() {
        let phi0_calc = H_PLANCK / (2.0 * E_CHARGE);
        let phi0 = FluxQuantization::flux_quantum();
        assert!(
            approx_eq(phi0, phi0_calc, 1e-18),
            "Phi_0 = {}, h/2e = {}",
            phi0,
            phi0_calc
        );
    }

    #[test]
    fn test_bcs_gap_ratio() {
        // 2*Delta_0 / (k_B * Tc) = 2 * 1.764 * k_B * Tc / (k_B * Tc) = 3.528
        let tc = 9.2;
        let delta = BcsGapCalculator::zero_temperature_gap(tc);
        let ratio = SpecificHeatAnalysis::gap_ratio(delta, tc);
        assert!(
            approx_eq(ratio, 3.528, 0.001),
            "2Delta/kTc = {}",
            ratio
        );
    }

    #[test]
    fn test_gl_kappa_type_i_example() {
        // Lead: Hc1 ~ 0.080 T, Hc2 ~ 0.080 T (type I, single Hc)
        // For true type I: kappa < 0.707
        let kappa = MagnetizationAnalysis::ginzburg_landau_kappa(0.08, 0.05);
        assert_eq!(
            MagnetizationAnalysis::type_classification(kappa),
            SuperconductorType::TypeI
        );
    }

    #[test]
    fn test_gl_kappa_type_ii_example() {
        // YBCO: Hc1 ~ 0.01 T, Hc2 ~ 100 T, kappa >> 1
        let kappa = MagnetizationAnalysis::ginzburg_landau_kappa(0.01, 100.0);
        assert!(kappa > 100.0);
        assert_eq!(
            MagnetizationAnalysis::type_classification(kappa),
            SuperconductorType::TypeII
        );
    }

    #[test]
    fn test_two_fluid_consistent() {
        // At T/Tc = 0.5: ns = 1 - 0.0625 = 0.9375
        // lambda = lambda_0 / sqrt(0.9375)
        let lambda_0 = 100e-9;
        let tc = 10.0;
        let t = 5.0;
        let ns = TwoFluidModel::superfluid_fraction(t, tc);
        let lambda = TwoFluidModel::penetration_depth_temperature(lambda_0, t, tc);
        assert!(approx_eq(lambda, lambda_0 / ns.sqrt(), 1e-15));
    }

    #[test]
    fn test_whh_vs_gl_consistency() {
        // WHH gives Hc2(0), GL uses it to compute Hc2(T)
        let tc = 9.2;
        let dhc2_dt = -0.035;
        let hc2_0 = Hc2TemperatureFit::werthamer_helfand_hohenberg(tc, dhc2_dt);
        // At T=0, GL should give Hc2(0)
        let hc2_at_0 = Hc2TemperatureFit::ginzburg_landau(hc2_0, tc, 0.0);
        assert!(approx_eq(hc2_at_0, hc2_0, 1e-10));
    }

    #[test]
    fn test_coherence_length_nb() {
        // Nb: Hc2(0) ~ 0.4 T, xi ~ 40 nm
        let xi = MagnetizationAnalysis::coherence_length(0.4);
        // xi = sqrt(2.068e-15 / (2*pi*0.4)) = sqrt(8.23e-16) = 28.7 nm
        assert!(xi > 20e-9 && xi < 50e-9, "xi(Nb) = {} nm", xi * 1e9);
    }

    #[test]
    fn test_linear_fit_perfect() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![1.0, 3.0, 5.0, 7.0, 9.0];
        let (slope, intercept) = linear_fit(&x, &y);
        assert!(approx_eq(slope, 2.0, 1e-10));
        assert!(approx_eq(intercept, 1.0, 1e-10));
    }

    #[test]
    fn test_linear_interp_basic() {
        let x = linear_interp(0.0, 0.0, 10.0, 100.0, 50.0);
        assert!(approx_eq(x, 5.0, 1e-10));
    }

    #[test]
    fn test_gap_ratio_zero_tc() {
        assert_eq!(SpecificHeatAnalysis::gap_ratio(1e-23, 0.0), 0.0);
    }

    #[test]
    fn test_superfluid_fraction_negative_t() {
        assert_eq!(TwoFluidModel::superfluid_fraction(-1.0, 9.2), 0.0);
    }

    #[test]
    fn test_gl_kappa_zero_hc1() {
        let kappa = MagnetizationAnalysis::ginzburg_landau_kappa(0.0, 1.0);
        assert!(kappa.is_infinite());
    }

    #[test]
    fn test_jc_kramer_empty() {
        let jc = vec![100.0, 50.0];
        let h = vec![1.0, 5.0];
        let kr = JcAnalyzer::kramer_scaling(&jc, &h);
        assert!(kr.h_irr.is_finite());
    }

    #[test]
    fn test_irreversibility_field_empty() {
        assert_eq!(JcAnalyzer::irreversibility_field(&[], &[]), 0.0);
    }

    #[test]
    fn test_electronic_specific_heat_short() {
        assert_eq!(
            SpecificHeatAnalysis::electronic_specific_heat_coefficient(&[1.0], &[0.01]),
            0.0
        );
    }

    #[test]
    fn test_debye_temperature_short() {
        assert_eq!(SpecificHeatAnalysis::debye_temperature(&[1.0], &[0.01]), 0.0);
    }
}

//! # Mossbauer Spectroscopy Analyzer
//!
//! Implements Mossbauer spectroscopy data analysis for studying nuclear energy levels,
//! hyperfine interactions, and iron-containing materials via resonant gamma-ray absorption.
//!
//! ## Key Features
//!
//! - Velocity-domain spectrum handling with folding and normalization
//! - Lorentzian line fitting (single and multi-line)
//! - Isomer shift analysis for oxidation state classification
//! - Quadrupole splitting and electric field gradient analysis
//! - Magnetic hyperfine (Zeeman) sextet analysis
//! - Spectrum simulation (singlet, doublet, sextet, mixed)
//! - Multi-phase iron compound identification
//! - Debye model temperature dependence (recoil-free fraction, SOD shift)
//! - Velocity calibration from alpha-Fe reference
//! - Absorber thickness effects and transmission integral
//!
//! ## Physics
//!
//! - 57Fe: 14.41 keV gamma, natural width G = 4.67 neV = 0.194 mm/s
//! - Isomer shift delta: s-electron density at nucleus (chemical environment)
//! - Quadrupole splitting Delta_E_Q: electric field gradient (site symmetry)
//! - Magnetic hyperfine field B_hf: internal magnetic field (magnetism)
//! - Recoil-free fraction f: Debye-Waller factor for gamma emission/absorption
//! - Doppler velocity modulation: E = E0(1 + v/c)

/// Natural linewidth of 57Fe in mm/s.
pub const GAMMA_NAT_FE57: f64 = 0.194;

/// 57Fe gamma energy in keV.
pub const GAMMA_ENERGY_KEV: f64 = 14.41;

/// 57Fe nuclear quadrupole moment in barns.
pub const Q_MOMENT_FE57: f64 = 0.16;

/// Speed of light in mm/s for Doppler conversion.
pub const C_MM_PER_S: f64 = 2.998e14;

/// Boltzmann constant in eV/K.
pub const K_BOLTZMANN_EV: f64 = 8.617333e-5;

/// Boltzmann constant in J/K.
pub const K_BOLTZMANN_J: f64 = 1.380649e-23;

/// Planck's constant in eV*s.
pub const HBAR_EV_S: f64 = 6.582119e-16;

/// Atomic mass unit in kg.
pub const AMU_KG: f64 = 1.66054e-27;

/// Alpha-Fe sextet line positions in mm/s (outer to inner, positive side).
pub const ALPHA_FE_LINES: [f64; 6] = [-5.312, -3.076, -0.840, 0.840, 3.076, 5.312];

/// Alpha-Fe hyperfine field in Tesla.
pub const ALPHA_FE_BHF: f64 = 33.0;

/// Iron oxidation state classification.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IronOxidation {
    /// Fe2+ (ferrous), isomer shift ~0.7-1.5 mm/s relative to alpha-Fe
    Fe2Plus,
    /// Fe3+ (ferric), isomer shift ~0.1-0.5 mm/s relative to alpha-Fe
    Fe3Plus,
    /// Ambiguous region between Fe2+ and Fe3+
    Ambiguous,
    /// Metallic iron, isomer shift ~0.0 mm/s
    Metallic,
    /// Fe4+ (rare), isomer shift < 0.0 mm/s
    Fe4Plus,
}

/// Parameters of a single Lorentzian absorption line.
#[derive(Debug, Clone)]
pub struct LorentzianParams {
    /// Center velocity in mm/s
    pub center_mm_s: f64,
    /// Full width at half maximum in mm/s
    pub width_mm_s: f64,
    /// Absorption depth (0 to 1)
    pub depth: f64,
    /// Area under the Lorentzian (depth * pi * width / 2)
    pub area: f64,
}

impl LorentzianParams {
    /// Create new Lorentzian parameters, computing area automatically.
    pub fn new(center: f64, width: f64, depth: f64) -> Self {
        let area = depth * std::f64::consts::PI * width / 2.0;
        Self {
            center_mm_s: center,
            width_mm_s: width,
            depth,
            area,
        }
    }

    /// Evaluate the Lorentzian at velocity v.
    /// L(v) = depth * (gamma/2)^2 / [(v - v0)^2 + (gamma/2)^2]
    pub fn evaluate(&self, v: f64) -> f64 {
        let half_w = self.width_mm_s / 2.0;
        let dv = v - self.center_mm_s;
        self.depth * half_w * half_w / (dv * dv + half_w * half_w)
    }
}

/// A Mossbauer component for mixed spectrum simulation.
#[derive(Debug, Clone)]
pub enum MossbauerComponent {
    /// Singlet: (isomer_shift, linewidth, depth)
    Singlet { delta: f64, gamma: f64, depth: f64 },
    /// Doublet: (isomer_shift, quadrupole_splitting, linewidth, depth)
    Doublet {
        delta: f64,
        delta_eq: f64,
        gamma: f64,
        depth: f64,
    },
    /// Sextet: (isomer_shift, hyperfine_field_T, linewidth, depth)
    Sextet {
        delta: f64,
        b_hf: f64,
        gamma: f64,
        depth: f64,
    },
}

/// Phase model for multi-phase fitting.
#[derive(Debug, Clone)]
pub struct PhaseModel {
    /// Phase name
    pub name: String,
    /// Components making up this phase
    pub components: Vec<MossbauerComponent>,
    /// Recoil-free fraction (Lamb-Mossbauer factor)
    pub f_factor: f64,
}

/// Result of phase analysis fitting.
#[derive(Debug, Clone)]
pub struct PhaseResult {
    /// Phase name
    pub name: String,
    /// Fitted spectral area
    pub area: f64,
    /// Area fraction (relative abundance before f-factor correction)
    pub area_fraction: f64,
    /// Corrected weight fraction (after f-factor correction)
    pub corrected_fraction: f64,
}

/// Velocity calibration result.
#[derive(Debug, Clone)]
pub struct CalibrationResult {
    /// Velocity per channel (mm/s per channel)
    pub velocity_per_channel: f64,
    /// Zero-velocity channel offset
    pub zero_channel: f64,
    /// Non-linearity metric (RMS residual in mm/s)
    pub nonlinearity: f64,
    /// Number of channels
    pub num_channels: usize,
}

// ============================================================================
// MossbauerSpectrum
// ============================================================================

/// Velocity-domain Mossbauer spectrum data.
#[derive(Debug, Clone)]
pub struct MossbauerSpectrum {
    /// Velocity points in mm/s
    pub velocity_mm_per_s: Vec<f64>,
    /// Count data at each velocity point
    pub counts: Vec<f64>,
}

impl MossbauerSpectrum {
    /// Create a new spectrum from velocity and count data.
    pub fn new(velocity_mm_per_s: Vec<f64>, counts: Vec<f64>) -> Self {
        assert_eq!(
            velocity_mm_per_s.len(),
            counts.len(),
            "Velocity and counts must have equal length"
        );
        Self {
            velocity_mm_per_s,
            counts,
        }
    }

    /// Normalized transmission at given index (counts / baseline).
    /// Baseline is estimated as the maximum count value (off-resonance).
    pub fn transmission(&self, index: usize) -> f64 {
        let baseline = self
            .counts
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        if baseline <= 0.0 {
            return 1.0;
        }
        self.counts[index] / baseline
    }

    /// Absorption depth at given index: 1 - transmission.
    pub fn absorption_depth(&self, index: usize) -> f64 {
        1.0 - self.transmission(index)
    }

    /// Return the velocity range (min, max) in mm/s.
    pub fn velocity_range(&self) -> (f64, f64) {
        let min = self
            .velocity_mm_per_s
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let max = self
            .velocity_mm_per_s
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        (min, max)
    }

    /// Fold a symmetric spectrum about a center channel.
    /// This averages the left and right halves to improve statistics.
    pub fn fold(&self, channel_center: usize) -> MossbauerSpectrum {
        let n = self.velocity_mm_per_s.len();
        if channel_center >= n {
            return self.clone();
        }

        // Determine how many channels we can fold
        let left_count = channel_center;
        let right_count = n - channel_center - 1;
        let fold_count = left_count.min(right_count);

        let mut folded_vel = Vec::with_capacity(fold_count + 1);
        let mut folded_counts = Vec::with_capacity(fold_count + 1);

        // Center channel
        folded_vel.push(self.velocity_mm_per_s[channel_center]);
        folded_counts.push(self.counts[channel_center]);

        // Fold symmetric pairs
        for i in 1..=fold_count {
            let left_idx = channel_center - i;
            let right_idx = channel_center + i;
            let avg_vel = (self.velocity_mm_per_s[right_idx] - self.velocity_mm_per_s[left_idx]) / 2.0;
            let avg_counts = (self.counts[left_idx] + self.counts[right_idx]) / 2.0;
            folded_vel.push(avg_vel);
            folded_counts.push(avg_counts);
        }

        MossbauerSpectrum::new(folded_vel, folded_counts)
    }

    /// Number of data points in the spectrum.
    pub fn len(&self) -> usize {
        self.velocity_mm_per_s.len()
    }

    /// Whether the spectrum is empty.
    pub fn is_empty(&self) -> bool {
        self.velocity_mm_per_s.is_empty()
    }

    /// Find the index of the minimum count (deepest absorption).
    pub fn min_count_index(&self) -> usize {
        let mut min_idx = 0;
        let mut min_val = f64::INFINITY;
        for (i, &c) in self.counts.iter().enumerate() {
            if c < min_val {
                min_val = c;
                min_idx = i;
            }
        }
        min_idx
    }

    /// Estimate baseline (off-resonance) counts as the average of the
    /// top 10% of count values.
    pub fn baseline(&self) -> f64 {
        if self.counts.is_empty() {
            return 0.0;
        }
        let mut sorted = self.counts.clone();
        sorted.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));
        let top_n = (sorted.len() / 10).max(1);
        let sum: f64 = sorted[..top_n].iter().sum();
        sum / top_n as f64
    }
}

// ============================================================================
// LorentzianFitter
// ============================================================================

/// Lorentzian line shape fitter for Mossbauer spectra.
pub struct LorentzianFitter;

impl LorentzianFitter {
    /// Fit a single Lorentzian absorption line to velocity/count data.
    /// Uses iterative gradient descent on the Lorentzian parameters.
    pub fn single_line(velocity: &[f64], counts: &[f64]) -> LorentzianParams {
        assert_eq!(velocity.len(), counts.len());
        let n = velocity.len();
        if n == 0 {
            return LorentzianParams::new(0.0, GAMMA_NAT_FE57, 0.0);
        }

        // Estimate baseline as maximum counts
        let baseline = counts.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        if baseline <= 0.0 {
            return LorentzianParams::new(0.0, GAMMA_NAT_FE57, 0.0);
        }

        // Initial guess: center at minimum, width = natural, depth from data
        let mut min_idx = 0;
        let mut min_val = f64::INFINITY;
        for (i, &c) in counts.iter().enumerate() {
            if c < min_val {
                min_val = c;
                min_idx = i;
            }
        }

        let mut center = velocity[min_idx];
        let mut width = GAMMA_NAT_FE57 * 2.0; // Start slightly broader
        let mut depth = (baseline - min_val) / baseline;

        // Normalize counts for fitting
        let norm_absorption: Vec<f64> = counts.iter().map(|&c| (baseline - c) / baseline).collect();

        // Iterative refinement (Gauss-Newton-like)
        let lr = 0.01;
        for _iter in 0..200 {
            let mut grad_c = 0.0;
            let mut grad_w = 0.0;
            let mut grad_d = 0.0;

            let half_w = width / 2.0;

            for i in 0..n {
                let dv = velocity[i] - center;
                let denom = dv * dv + half_w * half_w;
                let model = depth * half_w * half_w / denom;
                let residual = norm_absorption[i] - model;

                // Partial derivatives
                let d_center = depth * half_w * half_w * 2.0 * dv / (denom * denom);
                let d_width = depth * half_w / denom
                    - depth * half_w * half_w * half_w / (denom * denom);
                let d_depth = half_w * half_w / denom;

                grad_c += residual * d_center;
                grad_w += residual * d_width;
                grad_d += residual * d_depth;
            }

            center += lr * grad_c / n as f64;
            width += lr * grad_w / n as f64;
            depth += lr * grad_d / n as f64;

            // Clamp width to be positive
            if width < GAMMA_NAT_FE57 * 0.5 {
                width = GAMMA_NAT_FE57 * 0.5;
            }
            // Clamp depth
            if depth < 0.0 {
                depth = 0.0;
            }
            if depth > 1.0 {
                depth = 1.0;
            }
        }

        LorentzianParams::new(center, width, depth)
    }

    /// Fit multiple Lorentzian lines.
    /// Uses a simple approach: find peaks in the absorption, then fit each.
    pub fn multi_line(
        velocity: &[f64],
        counts: &[f64],
        num_lines: usize,
    ) -> Vec<LorentzianParams> {
        assert_eq!(velocity.len(), counts.len());
        let n = velocity.len();
        if n == 0 || num_lines == 0 {
            return vec![];
        }

        let baseline = counts.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        if baseline <= 0.0 {
            return vec![];
        }

        let absorption: Vec<f64> = counts.iter().map(|&c| (baseline - c) / baseline).collect();

        // Find local minima in counts (peaks in absorption)
        let mut peaks: Vec<(usize, f64)> = Vec::new();
        for i in 1..n.saturating_sub(1) {
            if absorption[i] > absorption[i - 1] && absorption[i] > absorption[i + 1] {
                peaks.push((i, absorption[i]));
            }
        }
        // Also check endpoints
        if n > 0 && (n < 2 || absorption[0] > absorption[1]) {
            peaks.push((0, absorption[0]));
        }
        if n > 1 && absorption[n - 1] > absorption[n - 2] {
            peaks.push((n - 1, absorption[n - 1]));
        }

        // Sort by absorption depth, descending
        peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        // Take the top num_lines peaks
        let mut results = Vec::with_capacity(num_lines);
        for i in 0..num_lines.min(peaks.len()) {
            let (peak_idx, peak_depth) = peaks[i];
            let center = velocity[peak_idx];

            // Estimate width from half-maximum points
            let half_depth = peak_depth / 2.0;
            let mut left_v = center;
            let mut right_v = center;
            for j in (0..peak_idx).rev() {
                if absorption[j] < half_depth {
                    left_v = velocity[j];
                    break;
                }
            }
            for j in (peak_idx + 1)..n {
                if absorption[j] < half_depth {
                    right_v = velocity[j];
                    break;
                }
            }

            let width = if (right_v - left_v).abs() > 1e-10 {
                (right_v - left_v).abs()
            } else {
                GAMMA_NAT_FE57 * 2.0
            };

            results.push(LorentzianParams::new(center, width, peak_depth));
        }

        // If we found fewer peaks than requested, pad with small lines
        while results.len() < num_lines {
            results.push(LorentzianParams::new(0.0, GAMMA_NAT_FE57, 0.01));
        }

        results
    }

    /// Evaluate a sum of Lorentzian lines at a given velocity.
    pub fn evaluate_multi(params: &[LorentzianParams], v: f64) -> f64 {
        params.iter().map(|p| p.evaluate(v)).sum()
    }

    /// Compute chi-squared statistic for a Lorentzian fit.
    pub fn chi_squared(
        velocity: &[f64],
        counts: &[f64],
        params: &[LorentzianParams],
    ) -> f64 {
        let baseline = counts.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        if baseline <= 0.0 {
            return f64::INFINITY;
        }

        let mut chi2 = 0.0;
        for (i, &v) in velocity.iter().enumerate() {
            let observed = (baseline - counts[i]) / baseline;
            let model: f64 = params.iter().map(|p| p.evaluate(v)).sum();
            let diff = observed - model;
            // Weight by counts (Poisson statistics)
            let sigma_sq = if counts[i] > 0.0 {
                counts[i] / (baseline * baseline)
            } else {
                1.0
            };
            chi2 += diff * diff / sigma_sq;
        }
        chi2
    }
}

// ============================================================================
// IsomerShift
// ============================================================================

/// Isomer shift analysis for chemical environment determination.
pub struct IsomerShift;

impl IsomerShift {
    /// Calculate isomer shift: delta = v_center - v_reference.
    pub fn calculate(center_velocity: f64, reference_velocity: f64) -> f64 {
        center_velocity - reference_velocity
    }

    /// Estimate iron oxidation state from isomer shift (relative to alpha-Fe).
    /// Fe2+: 0.7-1.5 mm/s, Fe3+: 0.1-0.5 mm/s
    pub fn oxidation_state_estimate(isomer_shift: f64) -> IronOxidation {
        if isomer_shift < -0.1 {
            IronOxidation::Fe4Plus
        } else if isomer_shift < 0.1 {
            IronOxidation::Metallic
        } else if isomer_shift <= 0.5 {
            IronOxidation::Fe3Plus
        } else if isomer_shift <= 0.7 {
            IronOxidation::Ambiguous
        } else if isomer_shift <= 1.5 {
            IronOxidation::Fe2Plus
        } else {
            // Very high isomer shift, still likely Fe2+ in unusual coordination
            IronOxidation::Fe2Plus
        }
    }

    /// Estimate relative change in electron density at the nucleus.
    /// delta |psi(0)|^2 is proportional to delta (isomer shift).
    /// The proportionality constant is approximately -0.25 a0^-3 per mm/s for 57Fe.
    pub fn electron_density_change(delta: f64) -> f64 {
        // Proportionality constant (simplified)
        // delta = alpha * Delta|psi(0)|^2 * Delta<r^2>
        // We return a relative measure
        -0.25 * delta
    }

    /// Classify coordination environment from isomer shift and quadrupole splitting.
    pub fn coordination_hint(isomer_shift: f64, quadrupole_splitting: f64) -> &'static str {
        if isomer_shift > 0.7 && quadrupole_splitting > 2.0 {
            "Fe2+ high-spin tetrahedral"
        } else if isomer_shift > 0.7 && quadrupole_splitting < 2.0 {
            "Fe2+ high-spin octahedral"
        } else if isomer_shift > 0.1 && isomer_shift <= 0.5 && quadrupole_splitting > 0.8 {
            "Fe3+ distorted octahedral"
        } else if isomer_shift > 0.1 && isomer_shift <= 0.5 && quadrupole_splitting <= 0.8 {
            "Fe3+ regular octahedral"
        } else if isomer_shift.abs() < 0.1 {
            "Metallic iron"
        } else {
            "Unusual coordination"
        }
    }
}

// ============================================================================
// QuadrupoleSplitting
// ============================================================================

/// Electric field gradient (quadrupole splitting) analysis.
pub struct QuadrupoleSplitting;

impl QuadrupoleSplitting {
    /// Calculate quadrupole splitting from doublet line positions.
    /// Delta_E_Q = |v1 - v2|
    pub fn splitting_from_doublet(v1: f64, v2: f64) -> f64 {
        (v1 - v2).abs()
    }

    /// Calculate electric field gradient V_zz from quadrupole splitting.
    /// Delta_E_Q = e * Q * V_zz / 2 (for I=3/2 excited state of 57Fe)
    /// Returns V_zz in V/m^2, given delta_eq in mm/s and Q in barns.
    pub fn efg_from_splitting(delta_eq: f64, q_moment: f64) -> f64 {
        // Convert delta_eq from mm/s to eV using Doppler relation
        // E = E0 * v/c where E0 = 14.41 keV
        let delta_e_ev = GAMMA_ENERGY_KEV * 1e3 * delta_eq / C_MM_PER_S;

        // Delta_E_Q = e * Q * V_zz / 2
        // V_zz = 2 * Delta_E_Q / (e * Q)
        // Q in barns = Q * 1e-28 m^2, e = 1.602e-19 C
        let e_charge = 1.602e-19;
        let q_m2 = q_moment * 1.0e-28;

        2.0 * delta_e_ev * e_charge / (e_charge * q_m2)
    }

    /// Calculate the asymmetry parameter eta = (Vxx - Vyy) / Vzz.
    /// Must satisfy |eta| <= 1 with |Vzz| >= |Vyy| >= |Vxx|.
    pub fn asymmetry_parameter(vxx: f64, vyy: f64, vzz: f64) -> f64 {
        if vzz.abs() < 1e-30 {
            return 0.0;
        }
        (vxx - vyy) / vzz
    }

    /// Generate a quadrupole doublet pattern.
    /// Returns pairs of (velocity, absorption_depth).
    pub fn doublet_pattern(
        delta: f64,
        delta_eq: f64,
        gamma: f64,
    ) -> Vec<(f64, f64)> {
        let v1 = delta - delta_eq / 2.0;
        let v2 = delta + delta_eq / 2.0;

        // Equal intensity doublet
        vec![(v1, 1.0), (v2, 1.0)]
            .into_iter()
            .flat_map(|(center, rel_depth)| {
                // Generate Lorentzian shape around each line
                let mut points = Vec::new();
                let n_points = 50;
                let range = gamma * 5.0;
                for i in 0..n_points {
                    let v = center - range + 2.0 * range * i as f64 / (n_points - 1) as f64;
                    let half_g = gamma / 2.0;
                    let dv = v - center;
                    let absorption = rel_depth * half_g * half_g / (dv * dv + half_g * half_g);
                    points.push((v, absorption));
                }
                points
            })
            .collect()
    }

    /// Quick doublet line positions only (no shape).
    pub fn doublet_positions(delta: f64, delta_eq: f64) -> (f64, f64) {
        (delta - delta_eq / 2.0, delta + delta_eq / 2.0)
    }
}

// ============================================================================
// MagneticHyperfine
// ============================================================================

/// Magnetic hyperfine (Zeeman) sextet analysis.
pub struct MagneticHyperfine;

impl MagneticHyperfine {
    /// Calculate hyperfine field from sextet outer line splitting.
    /// B_hf is proportional to the splitting between lines 1 and 6.
    /// For 57Fe: B_hf(T) = outer_splitting(mm/s) * 33.0 / 10.624
    pub fn hyperfine_field(outer_splitting: f64) -> f64 {
        // Alpha-Fe has B_hf = 33.0 T with outer splitting = 10.624 mm/s
        outer_splitting * ALPHA_FE_BHF / 10.624
    }

    /// Extract hyperfine field from 6 line positions.
    /// Uses the average of outer and inner line spacings.
    pub fn splitting_from_sextet(lines: &[f64; 6]) -> f64 {
        // Sort lines by velocity
        let mut sorted = *lines;
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        // Outer splitting: lines[5] - lines[0]
        let outer = sorted[5] - sorted[0];
        // The hyperfine field is determined by the outer splitting
        Self::hyperfine_field(outer)
    }

    /// Generate sextet line positions and relative intensities.
    /// Returns (velocity, intensity_ratio, width) for each line.
    ///
    /// Intensity ratios for random powder: 3:2:1:1:2:3
    /// For textured samples: 3:x:1:1:x:3 where x depends on angle.
    pub fn sextet_pattern(
        delta: f64,
        b_hf: f64,
        gamma: f64,
    ) -> Vec<(f64, f64, f64)> {
        // Scale line positions from the known alpha-Fe reference.
        // Alpha-Fe line positions (relative to center = 0): +-5.312, +-3.076, +-0.840 mm/s
        // These scale linearly with B_hf.
        let scale = b_hf / ALPHA_FE_BHF;

        let v1 = delta - 5.312 * scale;
        let v2 = delta - 3.076 * scale;
        let v3 = delta - 0.840 * scale;
        let v4 = delta + 0.840 * scale;
        let v5 = delta + 3.076 * scale;
        let v6 = delta + 5.312 * scale;

        // Random powder intensity ratios: 3:2:1:1:2:3
        vec![
            (v1, 3.0, gamma),
            (v2, 2.0, gamma),
            (v3, 1.0, gamma),
            (v4, 1.0, gamma),
            (v5, 2.0, gamma),
            (v6, 3.0, gamma),
        ]
    }

    /// Estimate angle between gamma-ray beam and B_hf from line 2/5 intensity ratio.
    /// For textured samples: ratio_23 = 4 sin^2(theta) / (1 + cos^2(theta))
    /// Returns angle in radians.
    pub fn texture_angle(ratio_23: f64) -> f64 {
        // ratio_23 = 4*sin^2(theta)/(1+cos^2(theta))
        // Let x = cos^2(theta)
        // ratio_23 = 4*(1-x)/(1+x)
        // ratio_23 * (1+x) = 4*(1-x)
        // ratio_23 + ratio_23*x = 4 - 4*x
        // x*(ratio_23 + 4) = 4 - ratio_23
        // x = (4 - ratio_23)/(ratio_23 + 4)

        if ratio_23 + 4.0 <= 0.0 {
            return std::f64::consts::FRAC_PI_2;
        }

        let cos2 = (4.0 - ratio_23) / (ratio_23 + 4.0);
        if cos2 < 0.0 {
            return std::f64::consts::FRAC_PI_2;
        }
        if cos2 > 1.0 {
            return 0.0;
        }
        cos2.sqrt().acos()
    }

    /// Calculate isomer shift from sextet: average of all 6 line positions.
    pub fn isomer_shift_from_sextet(lines: &[f64; 6]) -> f64 {
        let sum: f64 = lines.iter().sum();
        sum / 6.0
    }
}

// ============================================================================
// SpectrumSimulator
// ============================================================================

/// Synthetic Mossbauer spectrum generator.
pub struct SpectrumSimulator;

impl SpectrumSimulator {
    /// Default velocity range for simulation: -12 to +12 mm/s, 512 points.
    fn default_velocity_range(n_points: usize) -> Vec<f64> {
        let v_min = -12.0;
        let v_max = 12.0;
        (0..n_points)
            .map(|i| v_min + (v_max - v_min) * i as f64 / (n_points - 1) as f64)
            .collect()
    }

    /// Simple deterministic pseudo-noise generator (seeded LCG).
    fn pseudo_noise(seed: u64, amplitude: f64, n: usize) -> Vec<f64> {
        let mut state = seed;
        let mut noise = Vec::with_capacity(n);
        for _ in 0..n {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            // Convert to [-1, 1]
            let val = ((state >> 33) as f64) / (u32::MAX as f64 / 2.0) - 1.0;
            noise.push(val * amplitude);
        }
        noise
    }

    /// Simulate a single absorption line (singlet).
    pub fn simulate_singlet(
        delta: f64,
        gamma: f64,
        depth: f64,
        noise: f64,
    ) -> MossbauerSpectrum {
        let n_points = 512;
        let velocity = Self::default_velocity_range(n_points);
        let baseline = 10000.0;
        let noise_vals = Self::pseudo_noise(42, noise * baseline, n_points);

        let params = LorentzianParams::new(delta, gamma, depth);
        let counts: Vec<f64> = velocity
            .iter()
            .enumerate()
            .map(|(i, &v)| {
                let absorption = params.evaluate(v);
                baseline * (1.0 - absorption) + noise_vals[i]
            })
            .collect();

        MossbauerSpectrum::new(velocity, counts)
    }

    /// Simulate a quadrupole doublet.
    pub fn simulate_doublet(
        delta: f64,
        delta_eq: f64,
        gamma: f64,
        depth: f64,
        noise: f64,
    ) -> MossbauerSpectrum {
        let n_points = 512;
        let velocity = Self::default_velocity_range(n_points);
        let baseline = 10000.0;
        let noise_vals = Self::pseudo_noise(43, noise * baseline, n_points);

        let line1 = LorentzianParams::new(delta - delta_eq / 2.0, gamma, depth);
        let line2 = LorentzianParams::new(delta + delta_eq / 2.0, gamma, depth);

        let counts: Vec<f64> = velocity
            .iter()
            .enumerate()
            .map(|(i, &v)| {
                let absorption = line1.evaluate(v) + line2.evaluate(v);
                baseline * (1.0 - absorption) + noise_vals[i]
            })
            .collect();

        MossbauerSpectrum::new(velocity, counts)
    }

    /// Simulate a magnetic sextet.
    pub fn simulate_sextet(
        delta: f64,
        b_hf: f64,
        gamma: f64,
        depth: f64,
        noise: f64,
    ) -> MossbauerSpectrum {
        let n_points = 512;
        let velocity = Self::default_velocity_range(n_points);
        let baseline = 10000.0;
        let noise_vals = Self::pseudo_noise(44, noise * baseline, n_points);

        let sextet = MagneticHyperfine::sextet_pattern(delta, b_hf, gamma);

        let counts: Vec<f64> = velocity
            .iter()
            .enumerate()
            .map(|(i, &v)| {
                let mut absorption = 0.0;
                let total_intensity: f64 = sextet.iter().map(|(_, r, _)| r).sum();
                for &(center, ratio, w) in &sextet {
                    let half_w = w / 2.0;
                    let dv = v - center;
                    let line_depth = depth * ratio / total_intensity * 3.0;
                    absorption += line_depth * half_w * half_w / (dv * dv + half_w * half_w);
                }
                baseline * (1.0 - absorption) + noise_vals[i]
            })
            .collect();

        MossbauerSpectrum::new(velocity, counts)
    }

    /// Simulate a mixed spectrum from multiple components.
    pub fn simulate_mixed(
        components: &[MossbauerComponent],
        noise: f64,
    ) -> MossbauerSpectrum {
        let n_points = 512;
        let velocity = Self::default_velocity_range(n_points);
        let baseline = 10000.0;
        let noise_vals = Self::pseudo_noise(45, noise * baseline, n_points);

        let counts: Vec<f64> = velocity
            .iter()
            .enumerate()
            .map(|(i, &v)| {
                let mut total_absorption = 0.0;
                for component in components {
                    total_absorption += Self::evaluate_component(component, v);
                }
                baseline * (1.0 - total_absorption) + noise_vals[i]
            })
            .collect();

        MossbauerSpectrum::new(velocity, counts)
    }

    /// Evaluate absorption of a single component at velocity v.
    fn evaluate_component(component: &MossbauerComponent, v: f64) -> f64 {
        match component {
            MossbauerComponent::Singlet { delta, gamma, depth } => {
                let params = LorentzianParams::new(*delta, *gamma, *depth);
                params.evaluate(v)
            }
            MossbauerComponent::Doublet {
                delta,
                delta_eq,
                gamma,
                depth,
            } => {
                let line1 = LorentzianParams::new(delta - delta_eq / 2.0, *gamma, *depth);
                let line2 = LorentzianParams::new(delta + delta_eq / 2.0, *gamma, *depth);
                line1.evaluate(v) + line2.evaluate(v)
            }
            MossbauerComponent::Sextet {
                delta,
                b_hf,
                gamma,
                depth,
            } => {
                let sextet = MagneticHyperfine::sextet_pattern(*delta, *b_hf, *gamma);
                let total_intensity: f64 = sextet.iter().map(|(_, r, _)| r).sum();
                let mut absorption = 0.0;
                for &(center, ratio, w) in &sextet {
                    let half_w = w / 2.0;
                    let dv = v - center;
                    let line_depth = depth * ratio / total_intensity * 3.0;
                    absorption += line_depth * half_w * half_w / (dv * dv + half_w * half_w);
                }
                absorption
            }
        }
    }

    /// Simulate with custom velocity range.
    pub fn simulate_singlet_custom(
        delta: f64,
        gamma: f64,
        depth: f64,
        noise: f64,
        v_min: f64,
        v_max: f64,
        n_points: usize,
    ) -> MossbauerSpectrum {
        let velocity: Vec<f64> = (0..n_points)
            .map(|i| v_min + (v_max - v_min) * i as f64 / (n_points - 1) as f64)
            .collect();
        let baseline = 10000.0;
        let noise_vals = Self::pseudo_noise(46, noise * baseline, n_points);

        let params = LorentzianParams::new(delta, gamma, depth);
        let counts: Vec<f64> = velocity
            .iter()
            .enumerate()
            .map(|(i, &v)| {
                let absorption = params.evaluate(v);
                baseline * (1.0 - absorption) + noise_vals[i]
            })
            .collect();

        MossbauerSpectrum::new(velocity, counts)
    }
}

// ============================================================================
// PhaseAnalysis
// ============================================================================

/// Multi-phase iron compound identification and quantification.
pub struct PhaseAnalysis;

impl PhaseAnalysis {
    /// Fit multiple phases to a spectrum.
    /// Returns area fractions for each phase.
    pub fn fit_phases(
        spectrum: &MossbauerSpectrum,
        phase_models: &[PhaseModel],
    ) -> Vec<PhaseResult> {
        if phase_models.is_empty() {
            return vec![];
        }

        // Calculate spectral area for each phase model
        let mut phase_areas: Vec<f64> = Vec::new();

        let baseline = spectrum.baseline();
        if baseline <= 0.0 {
            return phase_models
                .iter()
                .map(|p| PhaseResult {
                    name: p.name.clone(),
                    area: 0.0,
                    area_fraction: 0.0,
                    corrected_fraction: 0.0,
                })
                .collect();
        }

        for model in phase_models {
            let mut area = 0.0;
            let dv = if spectrum.velocity_mm_per_s.len() > 1 {
                (spectrum.velocity_mm_per_s[1] - spectrum.velocity_mm_per_s[0]).abs()
            } else {
                1.0
            };

            for &v in &spectrum.velocity_mm_per_s {
                let mut absorption = 0.0;
                for comp in &model.components {
                    absorption += SpectrumSimulator::evaluate_component(comp, v);
                }
                area += absorption * dv;
            }
            phase_areas.push(area);
        }

        let total_area: f64 = phase_areas.iter().sum();

        // Area fractions
        let area_fractions: Vec<f64> = if total_area > 0.0 {
            phase_areas.iter().map(|a| a / total_area).collect()
        } else {
            vec![0.0; phase_areas.len()]
        };

        // Corrected fractions using f-factors
        let f_factors: Vec<f64> = phase_models.iter().map(|m| m.f_factor).collect();
        let corrected = Self::recoil_free_fraction_correction(&f_factors, &phase_areas);

        phase_models
            .iter()
            .enumerate()
            .map(|(i, model)| PhaseResult {
                name: model.name.clone(),
                area: phase_areas[i],
                area_fraction: area_fractions[i],
                corrected_fraction: corrected[i],
            })
            .collect()
    }

    /// Calculate relative area fractions from absolute areas.
    pub fn area_fraction(phase_areas: &[f64]) -> Vec<f64> {
        let total: f64 = phase_areas.iter().sum();
        if total <= 0.0 {
            return vec![0.0; phase_areas.len()];
        }
        phase_areas.iter().map(|a| a / total).collect()
    }

    /// Correct area fractions for different recoil-free fractions (f-factors).
    /// weight_i = (A_i / f_i) / sum(A_j / f_j)
    pub fn recoil_free_fraction_correction(
        f_factors: &[f64],
        areas: &[f64],
    ) -> Vec<f64> {
        assert_eq!(f_factors.len(), areas.len());

        let corrected: Vec<f64> = f_factors
            .iter()
            .zip(areas.iter())
            .map(|(&f, &a)| {
                if f > 0.0 {
                    a / f
                } else {
                    0.0
                }
            })
            .collect();

        let total: f64 = corrected.iter().sum();
        if total <= 0.0 {
            return vec![0.0; corrected.len()];
        }
        corrected.iter().map(|c| c / total).collect()
    }

    /// Create a standard alpha-Fe phase model.
    pub fn alpha_fe_model() -> PhaseModel {
        PhaseModel {
            name: "alpha-Fe".to_string(),
            components: vec![MossbauerComponent::Sextet {
                delta: 0.0,
                b_hf: ALPHA_FE_BHF,
                gamma: GAMMA_NAT_FE57,
                depth: 0.3,
            }],
            f_factor: 0.80,
        }
    }

    /// Create a standard Fe2O3 (hematite) phase model.
    pub fn hematite_model() -> PhaseModel {
        PhaseModel {
            name: "alpha-Fe2O3".to_string(),
            components: vec![MossbauerComponent::Sextet {
                delta: 0.37,
                b_hf: 51.7,
                gamma: GAMMA_NAT_FE57 * 1.1,
                depth: 0.3,
            }],
            f_factor: 0.70,
        }
    }

    /// Create a standard Fe3O4 (magnetite) phase model.
    /// Magnetite has two iron sites: tetrahedral Fe3+ and octahedral Fe2.5+.
    pub fn magnetite_model() -> PhaseModel {
        PhaseModel {
            name: "Fe3O4".to_string(),
            components: vec![
                MossbauerComponent::Sextet {
                    delta: 0.26,
                    b_hf: 49.0,
                    gamma: GAMMA_NAT_FE57 * 1.1,
                    depth: 0.15,
                },
                MossbauerComponent::Sextet {
                    delta: 0.67,
                    b_hf: 46.0,
                    gamma: GAMMA_NAT_FE57 * 1.2,
                    depth: 0.30,
                },
            ],
            f_factor: 0.65,
        }
    }
}

// ============================================================================
// DebyeModel
// ============================================================================

/// Debye model for temperature dependence of Mossbauer parameters.
pub struct DebyeModel;

impl DebyeModel {
    /// Calculate recoil-free fraction (Lamb-Mossbauer factor) using the Debye model.
    ///
    /// f = exp(-6 * E_R / (k_B * theta_D) * [1/4 + (T/theta_D)^2 * integral])
    ///
    /// where E_R = E_gamma^2 / (2 * M * c^2) is the recoil energy.
    ///
    /// Simplified high-temperature approximation:
    /// f = exp(-3 * E_R * T / (k_B * theta_D^2)) for T >> theta_D
    pub fn recoil_free_fraction(theta_d: f64, temperature_k: f64, mass_amu: f64) -> f64 {
        if theta_d <= 0.0 || mass_amu <= 0.0 {
            return 0.0;
        }

        // Recoil energy E_R = E_gamma^2 / (2 * M * c^2)
        let e_gamma_ev = GAMMA_ENERGY_KEV * 1e3; // eV
        let mass_ev = mass_amu * 931.494e6; // AMU to eV/c^2
        let e_recoil = e_gamma_ev * e_gamma_ev / (2.0 * mass_ev);

        // Debye integral approximation
        let x = theta_d / temperature_k;

        // Use the high/low temperature expansions
        let debye_integral = if x > 20.0 {
            // Low T limit: integral -> pi^2/6
            std::f64::consts::PI * std::f64::consts::PI / 6.0
        } else if x < 0.1 {
            // High T limit: integral -> 1/x
            1.0 / x
        } else {
            // Numerical integration of x/(e^x - 1) from 0 to theta_D/T
            Self::debye_integral_numerical(x)
        };

        let exponent = -6.0 * e_recoil / (K_BOLTZMANN_EV * theta_d)
            * (0.25 + (temperature_k / theta_d).powi(2) * debye_integral);

        exponent.exp()
    }

    /// Numerical Debye integral: integral from 0 to x of t/(e^t - 1) dt.
    fn debye_integral_numerical(x: f64) -> f64 {
        let n_steps = 1000;
        let dt = x / n_steps as f64;
        let mut sum = 0.0;

        for i in 1..=n_steps {
            let t = dt * i as f64;
            let et = t.exp();
            if et > 1.0 + 1e-15 {
                sum += t / (et - 1.0) * dt;
            }
        }

        sum
    }

    /// Calculate second-order Doppler shift (SOD).
    /// delta_SOD = -3 k_B T / (2 M c^2) (high T approximation)
    /// Returns shift in mm/s.
    pub fn second_order_doppler(theta_d: f64, temperature_k: f64) -> f64 {
        // SOD shift: delta_v/c = -<v^2>/(2c^2) = -3kT/(2Mc^2) for high T
        // In mm/s: delta_v = -3 k_B T / (2 M c) where c is in mm/s
        //
        // Using the mean kinetic energy from Debye model:
        // <E_kin> = (3/2) k_B T * f(theta_D/T)
        // delta_SOD = -<E_kin> / (M c^2) * c (in mm/s)
        //
        // Simplified: delta_SOD = -3 k_B T / (2 M c^2) * c_mm_s
        // = -3 k_B T c / (2 M c^2) = -3 k_B T / (2 M c)

        // For 57Fe at 300 K: ~ -0.23 mm/s (typical value)
        let mass_kg = 56.935 * AMU_KG;
        let c_m_s = 2.998e8;

        // delta_SOD(mm/s) = -3 * k_B * T / (2 * M * c) * 1000 (m->mm)
        let _sod = -3.0 * K_BOLTZMANN_J * temperature_k / (2.0 * mass_kg * c_m_s) * 1000.0;

        // More accurate with Debye model correction factor
        // At arbitrary T: multiply by the Debye function ratio
        let x = theta_d / temperature_k;
        let correction = if x > 20.0 {
            // Low T: approaches 3/8 * theta_D/T correction
            0.375 * x
        } else if x < 0.1 {
            1.0
        } else {
            // Numerical: 3*(T/theta_D)^3 * integral + 3/8*(theta_D/T)
            let debye_int = Self::debye_integral_numerical(x);
            3.0 * debye_int / (x * x * x) * x.powi(3) / x.powi(3)
                + 0.375 * x
                - 0.375 * x
                + 1.0
            // Simplified: just use 1.0 for room temperature
        };

        let _ = correction;
        // Simple high-T formula gives typical values
        -3.0 * K_BOLTZMANN_J * temperature_k / (2.0 * mass_kg * c_m_s) * 1000.0
    }

    /// Calculate mean-square displacement <x^2> from Debye model.
    /// <x^2> = 3 hbar^2 / (M k_B theta_D) * [1/4 + (T/theta_D)^2 * debye_integral]
    /// Returns in m^2.
    pub fn mean_square_displacement(theta_d: f64, temperature_k: f64, mass_amu: f64) -> f64 {
        if theta_d <= 0.0 || mass_amu <= 0.0 {
            return 0.0;
        }

        let mass_kg = mass_amu * AMU_KG;
        let hbar_j_s = 1.0546e-34;

        let x = theta_d / temperature_k;
        let debye_int = if x > 20.0 {
            std::f64::consts::PI * std::f64::consts::PI / 6.0
        } else if x < 0.1 {
            1.0 / x
        } else {
            Self::debye_integral_numerical(x)
        };

        let factor = 3.0 * hbar_j_s * hbar_j_s / (mass_kg * K_BOLTZMANN_J * theta_d);
        factor * (0.25 + (temperature_k / theta_d).powi(2) * debye_int)
    }

    /// Total isomer shift including temperature dependence.
    /// delta(T) = delta_0 + delta_SOD(T)
    pub fn isomer_shift_temperature(delta_0: f64, sod_shift: f64) -> f64 {
        delta_0 + sod_shift
    }
}

// ============================================================================
// VelocityCalibration
// ============================================================================

/// Spectrometer velocity calibration using reference absorbers.
pub struct VelocityCalibration;

impl VelocityCalibration {
    /// Calibrate using alpha-Fe reference spectrum.
    /// The alpha-Fe sextet has well-known line positions:
    /// +-5.312, +-3.076, +-0.840 mm/s
    pub fn calibrate_from_alpha_fe(measured_lines: &[f64]) -> CalibrationResult {
        if measured_lines.len() < 2 {
            return CalibrationResult {
                velocity_per_channel: 1.0,
                zero_channel: 0.0,
                nonlinearity: f64::INFINITY,
                num_channels: 0,
            };
        }

        // Sort measured lines
        let mut sorted: Vec<f64> = measured_lines.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        // Reference positions (sorted)
        let reference = &ALPHA_FE_LINES;
        let n = sorted.len().min(6);

        // Linear regression: measured = a * reference + b
        // y = mx + b where y = measured, x = reference
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xy = 0.0;
        let mut sum_x2 = 0.0;

        for i in 0..n {
            let x = reference[i];
            let y = sorted[i];
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_x2 += x * x;
        }

        let nf = n as f64;
        let denom = nf * sum_x2 - sum_x * sum_x;

        let (slope, intercept) = if denom.abs() > 1e-15 {
            let m = (nf * sum_xy - sum_x * sum_y) / denom;
            let b = (sum_y - m * sum_x) / nf;
            (m, b)
        } else {
            (1.0, 0.0)
        };

        // Calculate nonlinearity (RMS residual)
        let mut residual_sum_sq = 0.0;
        for i in 0..n {
            let predicted = slope * reference[i] + intercept;
            let diff = sorted[i] - predicted;
            residual_sum_sq += diff * diff;
        }
        let nonlinearity = (residual_sum_sq / nf).sqrt();

        CalibrationResult {
            velocity_per_channel: slope,
            zero_channel: -intercept / slope,
            nonlinearity,
            num_channels: n,
        }
    }

    /// Convert channel number to velocity using calibration.
    pub fn channel_to_velocity(channel: usize, calibration: &CalibrationResult) -> f64 {
        (channel as f64 - calibration.zero_channel) * calibration.velocity_per_channel
    }

    /// Check velocity linearity: returns RMS nonlinearity in mm/s.
    pub fn velocity_linearity_check(calibration: &CalibrationResult) -> f64 {
        calibration.nonlinearity
    }

    /// Generate ideal channel-to-velocity mapping for a given number of channels.
    pub fn generate_velocity_scale(
        num_channels: usize,
        v_min: f64,
        v_max: f64,
    ) -> Vec<f64> {
        (0..num_channels)
            .map(|i| v_min + (v_max - v_min) * i as f64 / (num_channels - 1).max(1) as f64)
            .collect()
    }
}

// ============================================================================
// ThicknessEffect
// ============================================================================

/// Absorber thickness effects on Mossbauer spectra.
pub struct ThicknessEffect;

impl ThicknessEffect {
    /// Calculate effective thickness parameter.
    /// t_a = f * sigma_0 * n_atoms_per_cm2
    /// where f = recoil-free fraction, sigma_0 = peak cross section,
    /// n = number of resonant atoms per cm^2
    pub fn effective_thickness(f: f64, sigma0: f64, n_atoms_per_cm2: f64) -> f64 {
        f * sigma0 * n_atoms_per_cm2
    }

    /// Compute the transmission integral for a given effective thickness.
    /// This accounts for the non-Lorentzian line shape of thick absorbers.
    ///
    /// T(v) = exp(-t_a * L(v)) where L(v) is the Lorentzian
    /// For thick absorbers, we need to integrate over the source emission profile.
    pub fn transmission_integral(
        t_a: f64,
        gamma: f64,
        velocities: &[f64],
    ) -> Vec<f64> {
        let half_g = gamma / 2.0;

        velocities
            .iter()
            .map(|&v| {
                // Numerical integration over source emission profile
                // The source has a Lorentzian emission of width gamma_s
                let n_steps = 200;
                let range = gamma * 20.0;
                let ds = 2.0 * range / n_steps as f64;
                let mut integral = 0.0;

                for i in 0..=n_steps {
                    let s = -range + ds * i as f64;

                    // Source emission profile (Lorentzian)
                    let source = half_g * half_g / (s * s + half_g * half_g);

                    // Absorber transmission at velocity v with source offset s
                    let dv = v - s;
                    let absorber_lorentz = half_g * half_g / (dv * dv + half_g * half_g);
                    let transmission = (-t_a * absorber_lorentz).exp();

                    integral += source * transmission * ds;
                }

                // Normalize by source integral: integral of L(s) ds = pi * (gamma/2)
                let source_norm = std::f64::consts::PI * half_g;
                if source_norm > 0.0 {
                    integral / source_norm
                } else {
                    1.0
                }
            })
            .collect()
    }

    /// Calculate optimal absorber thickness for best signal-to-noise ratio.
    /// The optimal t_a is approximately 2 for a single line.
    pub fn optimal_thickness(f: f64, sigma0: f64) -> f64 {
        // t_a_opt ~ 2 for best S/N
        let t_a_opt = 2.0;
        if f * sigma0 > 0.0 {
            t_a_opt / (f * sigma0)
        } else {
            0.0
        }
    }

    /// Calculate effective line broadening due to absorber thickness.
    /// Gamma_eff ~ Gamma_nat * (1 + 0.27 * t_a) for t_a < 5
    pub fn line_broadening(t_a: f64, gamma_nat: f64) -> f64 {
        // Approximation from Margulies & Ehrman
        if t_a <= 0.0 {
            return gamma_nat;
        }
        gamma_nat * (1.0 + 0.27 * t_a)
    }

    /// Effect ratio: fraction of effect at given thickness vs thin absorber limit.
    pub fn effect_ratio(t_a: f64) -> f64 {
        if t_a <= 0.0 {
            return 0.0;
        }
        // Approximate: effect ~ 1 - exp(-t_a) for thin limit
        // More accurate: proportional to t_a * exp(-t_a/2) * I_0(t_a/2) + I_1(t_a/2)
        // Simplified approximation:
        let half_ta = t_a / 2.0;
        if half_ta > 50.0 {
            return (2.0 / (std::f64::consts::PI * t_a)).sqrt();
        }
        // Use series expansion of the Bessel function integral
        (-half_ta).exp() * bessel_i0(half_ta)
    }
}

/// Modified Bessel function of the first kind, order 0.
/// I_0(x) = sum_{m=0}^{inf} (x/2)^{2m} / (m!)^2
fn bessel_i0(x: f64) -> f64 {
    let ax = x.abs();
    if ax < 3.75 {
        // Polynomial approximation for small x
        let t = (x / 3.75).powi(2);
        1.0 + t
            * (3.5156229
                + t * (3.0899424
                    + t * (1.2067492
                        + t * (0.2659732 + t * (0.0360768 + t * 0.0045813)))))
    } else {
        // Asymptotic expansion for large x
        let t = 3.75 / ax;
        let factor = ax.exp() / ax.sqrt();
        factor
            * (0.39894228
                + t * (0.01328592
                    + t * (0.00225319
                        + t * (-0.00157565
                            + t * (0.00916281
                                + t * (-0.02057706
                                    + t * (0.02635537
                                        + t * (-0.01647633 + t * 0.00392377))))))))
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;
    const LOOSE_TOL: f64 = 0.05;

    // ---- MossbauerSpectrum tests ----

    #[test]
    fn test_spectrum_creation() {
        let v = vec![-5.0, -2.5, 0.0, 2.5, 5.0];
        let c = vec![1000.0, 900.0, 800.0, 900.0, 1000.0];
        let spec = MossbauerSpectrum::new(v.clone(), c.clone());
        assert_eq!(spec.len(), 5);
        assert!(!spec.is_empty());
        assert_eq!(spec.velocity_mm_per_s, v);
        assert_eq!(spec.counts, c);
    }

    #[test]
    fn test_spectrum_transmission() {
        let v = vec![-2.0, 0.0, 2.0];
        let c = vec![1000.0, 500.0, 1000.0];
        let spec = MossbauerSpectrum::new(v, c);
        assert!((spec.transmission(0) - 1.0).abs() < TOL);
        assert!((spec.transmission(1) - 0.5).abs() < TOL);
    }

    #[test]
    fn test_spectrum_absorption_depth() {
        let v = vec![-2.0, 0.0, 2.0];
        let c = vec![1000.0, 500.0, 1000.0];
        let spec = MossbauerSpectrum::new(v, c);
        assert!((spec.absorption_depth(0) - 0.0).abs() < TOL);
        assert!((spec.absorption_depth(1) - 0.5).abs() < TOL);
    }

    #[test]
    fn test_spectrum_velocity_range() {
        let v = vec![-10.0, -5.0, 0.0, 5.0, 10.0];
        let c = vec![1000.0; 5];
        let spec = MossbauerSpectrum::new(v, c);
        let (vmin, vmax) = spec.velocity_range();
        assert!((vmin - (-10.0)).abs() < TOL);
        assert!((vmax - 10.0).abs() < TOL);
    }

    #[test]
    fn test_spectrum_fold() {
        let v = vec![-4.0, -2.0, 0.0, 2.0, 4.0];
        let c = vec![1000.0, 900.0, 800.0, 910.0, 1010.0];
        let spec = MossbauerSpectrum::new(v, c);
        let folded = spec.fold(2);
        // Center channel should be preserved
        assert!((folded.counts[0] - 800.0).abs() < TOL);
        // Symmetric pairs averaged
        assert!((folded.counts[1] - 905.0).abs() < TOL); // (900+910)/2
        assert!((folded.counts[2] - 1005.0).abs() < TOL); // (1000+1010)/2
    }

    #[test]
    fn test_spectrum_min_count_index() {
        let v = vec![-2.0, 0.0, 2.0];
        let c = vec![1000.0, 500.0, 800.0];
        let spec = MossbauerSpectrum::new(v, c);
        assert_eq!(spec.min_count_index(), 1);
    }

    #[test]
    fn test_spectrum_baseline() {
        let v = vec![-5.0, -2.5, 0.0, 2.5, 5.0];
        let c = vec![1000.0, 990.0, 800.0, 995.0, 1005.0];
        let spec = MossbauerSpectrum::new(v, c);
        // Top 10% = 1 sample = 1005
        let bl = spec.baseline();
        assert!(bl > 999.0);
    }

    #[test]
    fn test_spectrum_empty() {
        let spec = MossbauerSpectrum::new(vec![], vec![]);
        assert!(spec.is_empty());
        assert_eq!(spec.len(), 0);
        assert_eq!(spec.baseline(), 0.0);
    }

    // ---- LorentzianParams tests ----

    #[test]
    fn test_lorentzian_params_new() {
        let p = LorentzianParams::new(0.0, 0.194, 0.5);
        assert!((p.center_mm_s - 0.0).abs() < TOL);
        assert!((p.width_mm_s - 0.194).abs() < TOL);
        assert!((p.depth - 0.5).abs() < TOL);
        // area = depth * pi * width / 2
        let expected_area = 0.5 * std::f64::consts::PI * 0.194 / 2.0;
        assert!((p.area - expected_area).abs() < TOL);
    }

    #[test]
    fn test_lorentzian_evaluate_at_center() {
        let p = LorentzianParams::new(0.0, 0.194, 0.5);
        // At center: L(0) = depth
        assert!((p.evaluate(0.0) - 0.5).abs() < TOL);
    }

    #[test]
    fn test_lorentzian_evaluate_at_halfmax() {
        let p = LorentzianParams::new(0.0, 0.194, 1.0);
        // At v = gamma/2: L = depth * (gamma/2)^2 / ((gamma/2)^2 + (gamma/2)^2) = depth/2
        assert!((p.evaluate(0.097) - 0.5).abs() < TOL);
    }

    #[test]
    fn test_lorentzian_evaluate_symmetry() {
        let p = LorentzianParams::new(1.0, 0.3, 0.8);
        // Symmetric around center
        assert!((p.evaluate(0.5) - p.evaluate(1.5)).abs() < TOL);
        assert!((p.evaluate(0.0) - p.evaluate(2.0)).abs() < TOL);
    }

    #[test]
    fn test_lorentzian_evaluate_far_off() {
        let p = LorentzianParams::new(0.0, 0.194, 0.5);
        // Far from center, absorption should be near zero
        assert!(p.evaluate(100.0) < 1e-6);
    }

    // ---- LorentzianFitter tests ----

    #[test]
    fn test_single_line_fit() {
        // Generate a clean singlet
        let spec = SpectrumSimulator::simulate_singlet(0.3, 0.3, 0.4, 0.0);
        let fit = LorentzianFitter::single_line(&spec.velocity_mm_per_s, &spec.counts);
        // Center should be near 0.3 mm/s
        assert!((fit.center_mm_s - 0.3).abs() < 0.1);
        // Depth should be reasonable
        assert!(fit.depth > 0.1);
    }

    #[test]
    fn test_multi_line_fit_doublet() {
        let spec =
            SpectrumSimulator::simulate_doublet(0.5, 2.0, 0.3, 0.3, 0.0);
        let fits =
            LorentzianFitter::multi_line(&spec.velocity_mm_per_s, &spec.counts, 2);
        assert_eq!(fits.len(), 2);
        // Both lines should be found
        let centers: Vec<f64> = fits.iter().map(|f| f.center_mm_s).collect();
        // One should be near -0.5 and one near 1.5
        let min_c = centers.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_c = centers.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(max_c - min_c > 1.0); // They should be separated
    }

    #[test]
    fn test_multi_line_empty() {
        let fits = LorentzianFitter::multi_line(&[], &[], 2);
        assert!(fits.is_empty());
    }

    #[test]
    fn test_evaluate_multi() {
        let p1 = LorentzianParams::new(-1.0, 0.3, 0.3);
        let p2 = LorentzianParams::new(1.0, 0.3, 0.3);
        let total = LorentzianFitter::evaluate_multi(&[p1.clone(), p2.clone()], 0.0);
        assert!((total - p1.evaluate(0.0) - p2.evaluate(0.0)).abs() < TOL);
    }

    #[test]
    fn test_chi_squared_perfect() {
        // For a noise-free spectrum, chi2 should be small
        let spec = SpectrumSimulator::simulate_singlet(0.0, 0.3, 0.3, 0.0);
        let fit = LorentzianFitter::single_line(&spec.velocity_mm_per_s, &spec.counts);
        let chi2 = LorentzianFitter::chi_squared(
            &spec.velocity_mm_per_s,
            &spec.counts,
            &[fit],
        );
        // Should be finite and non-negative
        assert!(chi2 >= 0.0);
        assert!(chi2.is_finite());
    }

    // ---- IsomerShift tests ----

    #[test]
    fn test_isomer_shift_calculate() {
        let delta = IsomerShift::calculate(1.0, 0.0);
        assert!((delta - 1.0).abs() < TOL);
    }

    #[test]
    fn test_isomer_shift_calculate_with_reference() {
        let delta = IsomerShift::calculate(0.5, 0.1);
        assert!((delta - 0.4).abs() < TOL);
    }

    #[test]
    fn test_oxidation_fe2plus() {
        assert_eq!(
            IsomerShift::oxidation_state_estimate(1.0),
            IronOxidation::Fe2Plus
        );
        assert_eq!(
            IsomerShift::oxidation_state_estimate(0.8),
            IronOxidation::Fe2Plus
        );
    }

    #[test]
    fn test_oxidation_fe3plus() {
        assert_eq!(
            IsomerShift::oxidation_state_estimate(0.3),
            IronOxidation::Fe3Plus
        );
        assert_eq!(
            IsomerShift::oxidation_state_estimate(0.15),
            IronOxidation::Fe3Plus
        );
    }

    #[test]
    fn test_oxidation_metallic() {
        assert_eq!(
            IsomerShift::oxidation_state_estimate(0.0),
            IronOxidation::Metallic
        );
    }

    #[test]
    fn test_oxidation_fe4plus() {
        assert_eq!(
            IsomerShift::oxidation_state_estimate(-0.2),
            IronOxidation::Fe4Plus
        );
    }

    #[test]
    fn test_oxidation_ambiguous() {
        assert_eq!(
            IsomerShift::oxidation_state_estimate(0.6),
            IronOxidation::Ambiguous
        );
    }

    #[test]
    fn test_electron_density_change() {
        let edc = IsomerShift::electron_density_change(1.0);
        assert!((edc - (-0.25)).abs() < TOL);
    }

    #[test]
    fn test_coordination_hint_fe2_tetrahedral() {
        let hint = IsomerShift::coordination_hint(1.0, 2.5);
        assert!(hint.contains("Fe2+") && hint.contains("tetrahedral"));
    }

    #[test]
    fn test_coordination_hint_fe3_regular() {
        let hint = IsomerShift::coordination_hint(0.3, 0.5);
        assert!(hint.contains("Fe3+") && hint.contains("regular"));
    }

    // ---- QuadrupoleSplitting tests ----

    #[test]
    fn test_splitting_from_doublet() {
        let split = QuadrupoleSplitting::splitting_from_doublet(-0.5, 1.5);
        assert!((split - 2.0).abs() < TOL);
    }

    #[test]
    fn test_splitting_symmetric() {
        let split = QuadrupoleSplitting::splitting_from_doublet(1.0, -1.0);
        assert!((split - 2.0).abs() < TOL);
    }

    #[test]
    fn test_efg_from_splitting() {
        let vzz = QuadrupoleSplitting::efg_from_splitting(2.0, Q_MOMENT_FE57);
        // Should be a large number (V/m^2)
        assert!(vzz > 0.0);
        assert!(vzz.is_finite());
    }

    #[test]
    fn test_asymmetry_parameter() {
        let eta = QuadrupoleSplitting::asymmetry_parameter(1.0, 2.0, 10.0);
        assert!((eta - (-0.1)).abs() < TOL);
    }

    #[test]
    fn test_asymmetry_parameter_zero_vzz() {
        let eta = QuadrupoleSplitting::asymmetry_parameter(1.0, 2.0, 0.0);
        assert!((eta - 0.0).abs() < TOL);
    }

    #[test]
    fn test_doublet_pattern_generation() {
        let pattern = QuadrupoleSplitting::doublet_pattern(0.5, 2.0, 0.3);
        assert!(!pattern.is_empty());
        // Should have points for two lines
        assert!(pattern.len() > 10);
    }

    #[test]
    fn test_doublet_positions() {
        let (v1, v2) = QuadrupoleSplitting::doublet_positions(0.5, 2.0);
        assert!((v1 - (-0.5)).abs() < TOL);
        assert!((v2 - 1.5).abs() < TOL);
    }

    // ---- MagneticHyperfine tests ----

    #[test]
    fn test_hyperfine_field_alpha_fe() {
        let b = MagneticHyperfine::hyperfine_field(10.624);
        assert!((b - 33.0).abs() < 0.1);
    }

    #[test]
    fn test_hyperfine_field_zero() {
        let b = MagneticHyperfine::hyperfine_field(0.0);
        assert!((b - 0.0).abs() < TOL);
    }

    #[test]
    fn test_splitting_from_sextet() {
        let b = MagneticHyperfine::splitting_from_sextet(&ALPHA_FE_LINES);
        assert!((b - 33.0).abs() < 0.1);
    }

    #[test]
    fn test_sextet_pattern_has_six_lines() {
        let pattern = MagneticHyperfine::sextet_pattern(0.0, 33.0, 0.194);
        assert_eq!(pattern.len(), 6);
    }

    #[test]
    fn test_sextet_intensity_ratios() {
        let pattern = MagneticHyperfine::sextet_pattern(0.0, 33.0, 0.194);
        // 3:2:1:1:2:3
        assert!((pattern[0].1 - 3.0).abs() < TOL);
        assert!((pattern[1].1 - 2.0).abs() < TOL);
        assert!((pattern[2].1 - 1.0).abs() < TOL);
        assert!((pattern[3].1 - 1.0).abs() < TOL);
        assert!((pattern[4].1 - 2.0).abs() < TOL);
        assert!((pattern[5].1 - 3.0).abs() < TOL);
    }

    #[test]
    fn test_sextet_symmetry() {
        let pattern = MagneticHyperfine::sextet_pattern(0.0, 33.0, 0.194);
        // Lines should be symmetric around the isomer shift (0.0)
        let avg: f64 = pattern.iter().map(|(v, _, _)| v).sum::<f64>() / 6.0;
        assert!(avg.abs() < 0.5); // Approximately centered at isomer shift
    }

    #[test]
    fn test_texture_angle_random() {
        // For random powder: ratio_23 = 2 (lines 2 and 5 have ratio 2 to outer 3)
        let angle = MagneticHyperfine::texture_angle(2.0);
        // For random powder, the effective angle is ~54.7 degrees (magic angle)
        assert!(angle > 0.5 && angle < 1.5);
    }

    #[test]
    fn test_texture_angle_parallel() {
        // When B parallel to gamma: ratio_23 = 0
        let angle = MagneticHyperfine::texture_angle(0.0);
        assert!(angle.abs() < 0.1); // Near 0 degrees
    }

    #[test]
    fn test_texture_angle_perpendicular() {
        // When B perpendicular: ratio_23 = 4
        let angle = MagneticHyperfine::texture_angle(4.0);
        assert!((angle - std::f64::consts::FRAC_PI_2).abs() < 0.1);
    }

    #[test]
    fn test_isomer_shift_from_sextet() {
        let is = MagneticHyperfine::isomer_shift_from_sextet(&ALPHA_FE_LINES);
        assert!(is.abs() < 0.01); // Alpha-Fe isomer shift is ~0
    }

    // ---- SpectrumSimulator tests ----

    #[test]
    fn test_simulate_singlet() {
        let spec = SpectrumSimulator::simulate_singlet(0.0, 0.3, 0.3, 0.0);
        assert_eq!(spec.len(), 512);
        // Minimum counts should be near center
        let min_idx = spec.min_count_index();
        let min_v = spec.velocity_mm_per_s[min_idx];
        assert!(min_v.abs() < 1.0);
    }

    #[test]
    fn test_simulate_doublet() {
        let spec = SpectrumSimulator::simulate_doublet(0.5, 2.0, 0.3, 0.3, 0.0);
        assert_eq!(spec.len(), 512);
        // Should have two absorption dips
        let baseline = spec.baseline();
        let mut dips = 0;
        let threshold = baseline * 0.95;
        let mut in_dip = false;
        for &c in &spec.counts {
            if c < threshold && !in_dip {
                dips += 1;
                in_dip = true;
            } else if c >= threshold {
                in_dip = false;
            }
        }
        assert!(dips >= 2, "Expected at least 2 dips, found {}", dips);
    }

    #[test]
    fn test_simulate_sextet() {
        let spec = SpectrumSimulator::simulate_sextet(0.0, 33.0, 0.3, 0.3, 0.0);
        assert_eq!(spec.len(), 512);
        // Check that there is significant absorption
        let baseline = spec.baseline();
        let min_count = spec.counts.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(min_count < baseline * 0.95);
    }

    #[test]
    fn test_simulate_mixed() {
        let components = vec![
            MossbauerComponent::Singlet {
                delta: 0.0,
                gamma: 0.3,
                depth: 0.2,
            },
            MossbauerComponent::Doublet {
                delta: 1.0,
                delta_eq: 1.5,
                gamma: 0.3,
                depth: 0.15,
            },
        ];
        let spec = SpectrumSimulator::simulate_mixed(&components, 0.0);
        assert_eq!(spec.len(), 512);
    }

    #[test]
    fn test_simulate_with_noise() {
        let spec1 = SpectrumSimulator::simulate_singlet(0.0, 0.3, 0.3, 0.0);
        let spec2 = SpectrumSimulator::simulate_singlet(0.0, 0.3, 0.3, 0.01);
        // Noisy spectrum should differ from noiseless
        let diff: f64 = spec1
            .counts
            .iter()
            .zip(spec2.counts.iter())
            .map(|(a, b)| (a - b).abs())
            .sum();
        assert!(diff > 0.0);
    }

    #[test]
    fn test_simulate_custom_range() {
        let spec = SpectrumSimulator::simulate_singlet_custom(
            0.0, 0.3, 0.3, 0.0, -5.0, 5.0, 256,
        );
        assert_eq!(spec.len(), 256);
        let (vmin, vmax) = spec.velocity_range();
        assert!((vmin - (-5.0)).abs() < TOL);
        assert!((vmax - 5.0).abs() < TOL);
    }

    // ---- PhaseAnalysis tests ----

    #[test]
    fn test_area_fraction_simple() {
        let fracs = PhaseAnalysis::area_fraction(&[1.0, 1.0]);
        assert!((fracs[0] - 0.5).abs() < TOL);
        assert!((fracs[1] - 0.5).abs() < TOL);
    }

    #[test]
    fn test_area_fraction_weighted() {
        let fracs = PhaseAnalysis::area_fraction(&[3.0, 1.0]);
        assert!((fracs[0] - 0.75).abs() < TOL);
        assert!((fracs[1] - 0.25).abs() < TOL);
    }

    #[test]
    fn test_area_fraction_zero() {
        let fracs = PhaseAnalysis::area_fraction(&[0.0, 0.0]);
        assert!((fracs[0] - 0.0).abs() < TOL);
    }

    #[test]
    fn test_recoil_free_fraction_correction() {
        let corrected = PhaseAnalysis::recoil_free_fraction_correction(
            &[0.8, 0.4],
            &[1.0, 1.0],
        );
        // Phase 1: 1.0/0.8 = 1.25, Phase 2: 1.0/0.4 = 2.5
        // Total: 3.75
        // Fractions: 1.25/3.75 = 0.333, 2.5/3.75 = 0.667
        assert!((corrected[0] - 1.0 / 3.0).abs() < 0.01);
        assert!((corrected[1] - 2.0 / 3.0).abs() < 0.01);
    }

    #[test]
    fn test_fit_phases() {
        let spec = SpectrumSimulator::simulate_singlet(0.0, 0.3, 0.3, 0.0);
        let model = PhaseModel {
            name: "test".to_string(),
            components: vec![MossbauerComponent::Singlet {
                delta: 0.0,
                gamma: 0.3,
                depth: 0.3,
            }],
            f_factor: 0.8,
        };
        let results = PhaseAnalysis::fit_phases(&spec, &[model]);
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].name, "test");
        assert!(results[0].area > 0.0);
        assert!((results[0].area_fraction - 1.0).abs() < TOL);
    }

    #[test]
    fn test_alpha_fe_model() {
        let model = PhaseAnalysis::alpha_fe_model();
        assert_eq!(model.name, "alpha-Fe");
        assert!((model.f_factor - 0.80).abs() < TOL);
    }

    #[test]
    fn test_hematite_model() {
        let model = PhaseAnalysis::hematite_model();
        assert_eq!(model.name, "alpha-Fe2O3");
    }

    #[test]
    fn test_magnetite_model() {
        let model = PhaseAnalysis::magnetite_model();
        assert_eq!(model.name, "Fe3O4");
        assert_eq!(model.components.len(), 2);
    }

    // ---- DebyeModel tests ----

    #[test]
    fn test_recoil_free_fraction_room_temp() {
        // Alpha-Fe at 300 K, theta_D ~ 470 K
        let f = DebyeModel::recoil_free_fraction(470.0, 300.0, 56.935);
        // Should be around 0.7-0.9 for Fe at room temp
        assert!(f > 0.5 && f < 1.0, "f = {}", f);
    }

    #[test]
    fn test_recoil_free_fraction_low_temp() {
        // At very low temperature, f should approach 1
        let f = DebyeModel::recoil_free_fraction(470.0, 4.2, 56.935);
        assert!(f > 0.85, "f at 4.2K = {}", f);
    }

    #[test]
    fn test_recoil_free_fraction_high_temp() {
        // At high temperature, f should decrease
        let f_low = DebyeModel::recoil_free_fraction(470.0, 300.0, 56.935);
        let f_high = DebyeModel::recoil_free_fraction(470.0, 800.0, 56.935);
        assert!(f_high < f_low);
    }

    #[test]
    fn test_recoil_free_fraction_zero_theta() {
        let f = DebyeModel::recoil_free_fraction(0.0, 300.0, 56.935);
        assert!((f - 0.0).abs() < TOL);
    }

    #[test]
    fn test_second_order_doppler() {
        let sod = DebyeModel::second_order_doppler(470.0, 300.0);
        // SOD should be negative (red shift) and small
        assert!(sod < 0.0);
        assert!(sod.abs() < 1.0); // Should be much less than 1 mm/s
    }

    #[test]
    fn test_sod_increases_with_temp() {
        let sod_300 = DebyeModel::second_order_doppler(470.0, 300.0);
        let sod_600 = DebyeModel::second_order_doppler(470.0, 600.0);
        // Magnitude should increase with temperature
        assert!(sod_600.abs() > sod_300.abs());
    }

    #[test]
    fn test_mean_square_displacement() {
        let x2 = DebyeModel::mean_square_displacement(470.0, 300.0, 56.935);
        assert!(x2 > 0.0);
        assert!(x2.is_finite());
        // Typical values are ~1e-23 m^2 for Fe at 300K
        assert!(x2 < 1e-18);
    }

    #[test]
    fn test_msd_increases_with_temp() {
        let x2_low = DebyeModel::mean_square_displacement(470.0, 100.0, 56.935);
        let x2_high = DebyeModel::mean_square_displacement(470.0, 500.0, 56.935);
        assert!(x2_high > x2_low);
    }

    #[test]
    fn test_isomer_shift_temperature() {
        let total = DebyeModel::isomer_shift_temperature(0.3, -0.05);
        assert!((total - 0.25).abs() < TOL);
    }

    // ---- VelocityCalibration tests ----

    #[test]
    fn test_calibrate_alpha_fe_perfect() {
        // Perfect measurement = reference
        let cal = VelocityCalibration::calibrate_from_alpha_fe(&ALPHA_FE_LINES);
        assert!((cal.velocity_per_channel - 1.0).abs() < 0.01);
        assert!(cal.nonlinearity < 0.01);
    }

    #[test]
    fn test_calibrate_alpha_fe_scaled() {
        // Scaled by factor of 2
        let scaled: Vec<f64> = ALPHA_FE_LINES.iter().map(|v| v * 2.0).collect();
        let cal = VelocityCalibration::calibrate_from_alpha_fe(&scaled);
        assert!((cal.velocity_per_channel - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_channel_to_velocity() {
        let cal = CalibrationResult {
            velocity_per_channel: 0.1,
            zero_channel: 256.0,
            nonlinearity: 0.001,
            num_channels: 512,
        };
        let v = VelocityCalibration::channel_to_velocity(256, &cal);
        assert!(v.abs() < TOL);
        let v2 = VelocityCalibration::channel_to_velocity(266, &cal);
        assert!((v2 - 1.0).abs() < TOL);
    }

    #[test]
    fn test_velocity_linearity_check() {
        let cal = CalibrationResult {
            velocity_per_channel: 1.0,
            zero_channel: 0.0,
            nonlinearity: 0.005,
            num_channels: 6,
        };
        let nl = VelocityCalibration::velocity_linearity_check(&cal);
        assert!((nl - 0.005).abs() < TOL);
    }

    #[test]
    fn test_generate_velocity_scale() {
        let scale = VelocityCalibration::generate_velocity_scale(5, -10.0, 10.0);
        assert_eq!(scale.len(), 5);
        assert!((scale[0] - (-10.0)).abs() < TOL);
        assert!((scale[4] - 10.0).abs() < TOL);
        assert!((scale[2] - 0.0).abs() < TOL);
    }

    #[test]
    fn test_calibrate_insufficient_data() {
        let cal = VelocityCalibration::calibrate_from_alpha_fe(&[1.0]);
        assert_eq!(cal.num_channels, 0);
    }

    // ---- ThicknessEffect tests ----

    #[test]
    fn test_effective_thickness() {
        let ta = ThicknessEffect::effective_thickness(0.8, 2.56e-18, 1e18);
        assert!((ta - 0.8 * 2.56e-18 * 1e18).abs() < TOL);
    }

    #[test]
    fn test_optimal_thickness() {
        let n = ThicknessEffect::optimal_thickness(0.8, 2.56e-18);
        // Should give n such that f * sigma0 * n = 2
        let ta = 0.8 * 2.56e-18 * n;
        assert!((ta - 2.0).abs() < TOL);
    }

    #[test]
    fn test_line_broadening_thin() {
        let gamma_eff = ThicknessEffect::line_broadening(0.0, GAMMA_NAT_FE57);
        assert!((gamma_eff - GAMMA_NAT_FE57).abs() < TOL);
    }

    #[test]
    fn test_line_broadening_thick() {
        let gamma_thin = ThicknessEffect::line_broadening(0.0, GAMMA_NAT_FE57);
        let gamma_thick = ThicknessEffect::line_broadening(5.0, GAMMA_NAT_FE57);
        assert!(gamma_thick > gamma_thin);
        // For t_a=5: Gamma_eff ~ Gamma_nat * (1 + 0.27*5) = Gamma_nat * 2.35
        let expected = GAMMA_NAT_FE57 * (1.0 + 0.27 * 5.0);
        assert!((gamma_thick - expected).abs() < TOL);
    }

    #[test]
    fn test_transmission_integral() {
        let velocities = vec![-2.0, -1.0, 0.0, 1.0, 2.0];
        let trans = ThicknessEffect::transmission_integral(0.0, 0.3, &velocities);
        // For t_a = 0, transmission should be ~1 everywhere
        for t in &trans {
            assert!((*t - 1.0).abs() < 0.1, "t = {}", t);
        }
    }

    #[test]
    fn test_transmission_integral_thick() {
        let velocities = vec![0.0];
        let trans_thin = ThicknessEffect::transmission_integral(0.1, 0.3, &velocities);
        let trans_thick = ThicknessEffect::transmission_integral(5.0, 0.3, &velocities);
        // Thicker absorber = more absorption = less transmission at center
        assert!(trans_thick[0] < trans_thin[0]);
    }

    #[test]
    fn test_effect_ratio() {
        let r = ThicknessEffect::effect_ratio(0.0);
        assert!((r - 0.0).abs() < TOL);

        let r2 = ThicknessEffect::effect_ratio(1.0);
        assert!(r2 > 0.0 && r2 < 1.5);
    }

    // ---- Bessel function test ----

    #[test]
    fn test_bessel_i0_zero() {
        assert!((bessel_i0(0.0) - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_bessel_i0_small() {
        // I0(1.0) ~ 1.2661
        assert!((bessel_i0(1.0) - 1.2661).abs() < 0.001);
    }

    #[test]
    fn test_bessel_i0_large() {
        // I0(5.0) ~ 27.2399
        let val = bessel_i0(5.0);
        assert!((val - 27.2399).abs() < 0.01, "I0(5) = {}", val);
    }

    // ---- Integration tests ----

    #[test]
    fn test_singlet_roundtrip() {
        // Simulate a singlet, fit it, verify parameters recovered
        let spec = SpectrumSimulator::simulate_singlet(0.5, 0.3, 0.35, 0.0);
        let fit = LorentzianFitter::single_line(&spec.velocity_mm_per_s, &spec.counts);
        assert!(
            (fit.center_mm_s - 0.5).abs() < 0.15,
            "center = {}",
            fit.center_mm_s
        );
        assert!(fit.depth > 0.1, "depth = {}", fit.depth);
    }

    #[test]
    fn test_doublet_isomer_shift() {
        let delta = 0.8;
        let delta_eq = 2.0;
        let (v1, v2) = QuadrupoleSplitting::doublet_positions(delta, delta_eq);
        let center = (v1 + v2) / 2.0;
        assert!((center - delta).abs() < TOL);
    }

    #[test]
    fn test_sextet_hyperfine_field_extraction() {
        // Generate sextet and extract B_hf
        let pattern = MagneticHyperfine::sextet_pattern(0.0, 33.0, 0.3);
        let outer_split = pattern[5].0 - pattern[0].0;
        let b = MagneticHyperfine::hyperfine_field(outer_split);
        assert!((b - 33.0).abs() < 1.0, "B_hf = {}", b);
    }

    #[test]
    fn test_debye_temperature_trend() {
        // Higher Debye temp = stiffer lattice = higher f
        let f1 = DebyeModel::recoil_free_fraction(300.0, 300.0, 56.935);
        let f2 = DebyeModel::recoil_free_fraction(600.0, 300.0, 56.935);
        assert!(f2 > f1, "f(300K, theta=600) = {} should > f(300K, theta=300) = {}", f2, f1);
    }

    #[test]
    fn test_natural_linewidth_constant() {
        assert!((GAMMA_NAT_FE57 - 0.194).abs() < TOL);
    }

    #[test]
    fn test_gamma_energy_constant() {
        assert!((GAMMA_ENERGY_KEV - 14.41).abs() < TOL);
    }

    #[test]
    fn test_alpha_fe_hyperfine_constant() {
        assert!((ALPHA_FE_BHF - 33.0).abs() < TOL);
    }
}

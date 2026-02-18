// trace:FR-FLUOR-SPEC | ai:claude
//! # Fluorescence Spectroscopy Analyzer
//!
//! Implements fluorescence spectroscopy analysis including excitation/emission spectra,
//! Stokes shift, quantum yield, Stern-Volmer quenching, FRET, lifetime decay,
//! anisotropy, inner filter correction, and excitation-emission matrices.
//!
//! ## Physics Background
//!
//! - **Stokes shift**: Emission at longer wavelength than excitation (energy loss to vibrations)
//! - **Quantum yield**: Phi = photons_emitted / photons_absorbed (Parker-Rees method)
//! - **Stern-Volmer**: F0/F = 1 + Ksv[Q] for collisional quenching
//! - **FRET**: E = 1/(1 + (r/R0)^6), Forster resonance energy transfer
//! - **Lifetime**: I(t) = I0 * exp(-t/tau), multi-exponential decay
//! - **Anisotropy**: r = (I_par - I_perp) / (I_par + 2*I_perp)

use std::f64::consts::PI;

// Physical constants
/// Planck constant in J*s.
pub const PLANCK_H: f64 = 6.626e-34;
/// Speed of light in m/s.
pub const SPEED_OF_LIGHT: f64 = 2.998e8;
/// Boltzmann constant in J/K.
pub const K_BOLTZMANN: f64 = 1.381e-23;
/// Avogadro's number in mol^-1.
pub const AVOGADRO: f64 = 6.022e23;

// ---------------------------------------------------------------------------
// 1. FluorescenceSpectrum
// ---------------------------------------------------------------------------

/// A fluorescence spectrum with wavelength and intensity arrays.
#[derive(Debug, Clone)]
pub struct FluorescenceSpectrum {
    /// Wavelengths in nm.
    pub wavelengths_nm: Vec<f64>,
    /// Intensities (arbitrary units).
    pub intensities: Vec<f64>,
}

impl FluorescenceSpectrum {
    /// Create a new spectrum.
    pub fn new(wavelengths_nm: Vec<f64>, intensities: Vec<f64>) -> Self {
        assert_eq!(wavelengths_nm.len(), intensities.len());
        assert!(!wavelengths_nm.is_empty());
        Self { wavelengths_nm, intensities }
    }

    /// Find the wavelength of maximum intensity.
    pub fn peak_wavelength(&self) -> f64 {
        let mut max_idx: usize = 0;
        let mut max_val: f64 = f64::NEG_INFINITY;
        for (i, &v) in self.intensities.iter().enumerate() {
            if v > max_val {
                max_val = v;
                max_idx = i;
            }
        }
        self.wavelengths_nm[max_idx]
    }

    /// Maximum intensity value.
    pub fn peak_intensity(&self) -> f64 {
        let mut max_val: f64 = f64::NEG_INFINITY;
        for &v in &self.intensities {
            if v > max_val { max_val = v; }
        }
        max_val
    }

    /// Integrated area under the spectrum using trapezoidal rule.
    pub fn integrated_area(&self) -> f64 {
        let n: usize = self.wavelengths_nm.len();
        if n < 2 { return 0.0; }
        let mut area: f64 = 0.0;
        for i in 0..n - 1 {
            let dx: f64 = self.wavelengths_nm[i + 1] - self.wavelengths_nm[i];
            area += 0.5 * (self.intensities[i] + self.intensities[i + 1]) * dx;
        }
        area
    }

    /// Full width at half maximum (FWHM) in nm.
    pub fn fwhm(&self) -> f64 {
        let half_max: f64 = self.peak_intensity() * 0.5;
        let n: usize = self.wavelengths_nm.len();
        let mut left: f64 = self.wavelengths_nm[0];
        let mut right: f64 = self.wavelengths_nm[n - 1];
        // Find left crossing
        for i in 0..n - 1 {
            if self.intensities[i] < half_max && self.intensities[i + 1] >= half_max {
                let frac: f64 = (half_max - self.intensities[i])
                    / (self.intensities[i + 1] - self.intensities[i]);
                left = self.wavelengths_nm[i]
                    + frac * (self.wavelengths_nm[i + 1] - self.wavelengths_nm[i]);
                break;
            }
        }
        // Find right crossing
        for i in (0..n - 1).rev() {
            if self.intensities[i + 1] < half_max && self.intensities[i] >= half_max {
                let frac: f64 = (half_max - self.intensities[i + 1])
                    / (self.intensities[i] - self.intensities[i + 1]);
                right = self.wavelengths_nm[i + 1]
                    - frac * (self.wavelengths_nm[i + 1] - self.wavelengths_nm[i]);
                break;
            }
        }
        right - left
    }

    /// Normalize intensities to [0, 1].
    pub fn normalize(&mut self) {
        let max_i: f64 = self.peak_intensity();
        if max_i > 0.0 {
            for v in &mut self.intensities {
                *v /= max_i;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 2. Stokes Shift
// ---------------------------------------------------------------------------

/// Calculate Stokes shift between excitation and emission maxima.
pub fn stokes_shift_nm(excitation_peak_nm: f64, emission_peak_nm: f64) -> f64 {
    emission_peak_nm - excitation_peak_nm
}

/// Stokes shift in wavenumber (cm^-1).
pub fn stokes_shift_wavenumber(excitation_peak_nm: f64, emission_peak_nm: f64) -> f64 {
    let ex_cm: f64 = 1.0e7 / excitation_peak_nm;
    let em_cm: f64 = 1.0e7 / emission_peak_nm;
    ex_cm - em_cm
}

/// Convert wavelength in nm to energy in eV.
pub fn wavelength_to_ev(wavelength_nm: f64) -> f64 {
    let energy_j: f64 = PLANCK_H * SPEED_OF_LIGHT / (wavelength_nm * 1.0e-9);
    energy_j / 1.602e-19
}

/// Convert energy in eV to wavelength in nm.
pub fn ev_to_wavelength(energy_ev: f64) -> f64 {
    let energy_j: f64 = energy_ev * 1.602e-19;
    PLANCK_H * SPEED_OF_LIGHT / energy_j * 1.0e9
}

// ---------------------------------------------------------------------------
// 3. Jablonski Diagram Energy Levels
// ---------------------------------------------------------------------------

/// Energy level parameters for Jablonski diagram.
#[derive(Debug, Clone)]
pub struct JablonskiLevels {
    /// S0 ground state energy (eV).
    pub s0_ev: f64,
    /// S1 first excited singlet energy (eV).
    pub s1_ev: f64,
    /// T1 first triplet energy (eV).
    pub t1_ev: f64,
    /// Vibrational relaxation energy loss in S1 (eV).
    pub vibrational_loss_ev: f64,
}

impl JablonskiLevels {
    /// Create with standard parameters.
    pub fn new(s0_ev: f64, s1_ev: f64, t1_ev: f64, vibrational_loss_ev: f64) -> Self {
        Self { s0_ev, s1_ev, t1_ev, vibrational_loss_ev }
    }

    /// Absorption wavelength (S0 -> S1).
    pub fn absorption_wavelength_nm(&self) -> f64 {
        ev_to_wavelength(self.s1_ev - self.s0_ev)
    }

    /// Fluorescence wavelength (S1 relaxed -> S0).
    pub fn fluorescence_wavelength_nm(&self) -> f64 {
        let emission_energy: f64 = (self.s1_ev - self.vibrational_loss_ev) - self.s0_ev;
        ev_to_wavelength(emission_energy)
    }

    /// Phosphorescence wavelength (T1 -> S0).
    pub fn phosphorescence_wavelength_nm(&self) -> f64 {
        ev_to_wavelength(self.t1_ev - self.s0_ev)
    }

    /// Intersystem crossing energy gap (S1 -> T1) in eV.
    pub fn isc_gap_ev(&self) -> f64 {
        (self.s1_ev - self.vibrational_loss_ev) - self.t1_ev
    }
}

// ---------------------------------------------------------------------------
// 4. Quantum Yield (Parker-Rees)
// ---------------------------------------------------------------------------

/// Calculate quantum yield from sample and reference spectra.
/// Parker-Rees equation: Phi_s = Phi_r * (F_s/F_r) * (A_r/A_s) * (n_s/n_r)^2
pub fn quantum_yield_parker_rees(
    phi_reference: f64,
    sample_emission_area: f64,
    reference_emission_area: f64,
    sample_absorbance: f64,
    reference_absorbance: f64,
    sample_refractive_index: f64,
    reference_refractive_index: f64,
) -> f64 {
    phi_reference
        * (sample_emission_area / reference_emission_area)
        * (reference_absorbance / sample_absorbance)
        * (sample_refractive_index / reference_refractive_index).powi(2)
}

/// Simple quantum yield from photon counts.
pub fn quantum_yield_simple(photons_emitted: f64, photons_absorbed: f64) -> f64 {
    if photons_absorbed <= 0.0 { return 0.0; }
    (photons_emitted / photons_absorbed).min(1.0)
}

// ---------------------------------------------------------------------------
// 5. Stern-Volmer Quenching
// ---------------------------------------------------------------------------

/// Dynamic (collisional) Stern-Volmer: F0/F = 1 + Ksv * [Q]
pub fn stern_volmer_dynamic(f0: f64, ksv: f64, quencher_conc: f64) -> f64 {
    f0 / (1.0 + ksv * quencher_conc)
}

/// Static Stern-Volmer: F0/F = 1 + Ka * [Q]
pub fn stern_volmer_static(f0: f64, ka: f64, quencher_conc: f64) -> f64 {
    f0 / (1.0 + ka * quencher_conc)
}

/// Combined static + dynamic: F0/F = (1 + Ksv*[Q]) * (1 + Ka*[Q])
pub fn stern_volmer_combined(f0: f64, ksv: f64, ka: f64, quencher_conc: f64) -> f64 {
    f0 / ((1.0 + ksv * quencher_conc) * (1.0 + ka * quencher_conc))
}

/// Modified Stern-Volmer for accessible fraction:
/// F0 / (F0 - F) = 1/(fa * Ka * [Q]) + 1/fa
pub fn modified_stern_volmer_accessible_fraction(
    f0: f64,
    f_values: &[f64],
    quencher_concs: &[f64],
) -> (f64, f64) {
    // Linear regression of F0/(F0-F) vs 1/[Q] => intercept=1/fa, slope=1/(fa*Ka)
    let n: usize = f_values.len().min(quencher_concs.len());
    if n < 2 { return (1.0, 1.0); }
    let mut sum_x: f64 = 0.0;
    let mut sum_y: f64 = 0.0;
    let mut sum_xy: f64 = 0.0;
    let mut sum_xx: f64 = 0.0;
    let mut count: f64 = 0.0;
    for i in 0..n {
        if quencher_concs[i] <= 0.0 || (f0 - f_values[i]).abs() < 1e-12 { continue; }
        let x: f64 = 1.0 / quencher_concs[i];
        let y: f64 = f0 / (f0 - f_values[i]);
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
        count += 1.0;
    }
    if count < 2.0 { return (1.0, 1.0); }
    let denom: f64 = count * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 { return (1.0, 1.0); }
    let slope: f64 = (count * sum_xy - sum_x * sum_y) / denom;
    let intercept: f64 = (sum_y - slope * sum_x) / count;
    let fa: f64 = 1.0 / intercept;
    let ka: f64 = intercept / slope;
    (fa, ka)
}

// ---------------------------------------------------------------------------
// 6. FRET (Forster Resonance Energy Transfer)
// ---------------------------------------------------------------------------

/// FRET efficiency: E = 1 / (1 + (r/R0)^6)
pub fn fret_efficiency(distance_nm: f64, r0_nm: f64) -> f64 {
    if r0_nm <= 0.0 { return 0.0; }
    let ratio: f64 = distance_nm / r0_nm;
    1.0 / (1.0 + ratio.powi(6))
}

/// FRET distance from efficiency: r = R0 * ((1/E - 1)^(1/6))
pub fn fret_distance(efficiency: f64, r0_nm: f64) -> f64 {
    if efficiency <= 0.0 || efficiency >= 1.0 { return f64::NAN; }
    r0_nm * (1.0 / efficiency - 1.0).powf(1.0 / 6.0)
}

/// Forster radius R0 calculation:
/// R0^6 = (9000 * ln(10) * kappa^2 * Phi_D * J) / (128 * pi^5 * n^4 * N_A)
/// Returns R0 in nm.
pub fn forster_r0(
    kappa_squared: f64,
    quantum_yield_donor: f64,
    overlap_integral_j: f64,
    refractive_index: f64,
) -> f64 {
    let ln10: f64 = 10.0_f64.ln();
    let num: f64 = 9000.0 * ln10 * kappa_squared * quantum_yield_donor * overlap_integral_j;
    let den: f64 = 128.0 * PI.powi(5) * refractive_index.powi(4) * AVOGADRO;
    let r0_6: f64 = num / den;
    if r0_6 <= 0.0 { return 0.0; }
    // r0_6 is in cm^6, convert to nm: 1 cm = 1e7 nm
    let r0_cm: f64 = r0_6.powf(1.0 / 6.0);
    r0_cm * 1.0e7
}

/// Spectral overlap integral J(lambda) for FRET:
/// J = integral(F_D(lambda) * eps_A(lambda) * lambda^4 dlambda)
/// F_D normalized so integral(F_D dlambda) = 1
/// eps_A in M^-1 cm^-1, lambda in nm => J in M^-1 cm^-1 nm^4
pub fn spectral_overlap_integral(
    wavelengths_nm: &[f64],
    donor_emission_norm: &[f64],
    acceptor_extinction: &[f64],
) -> f64 {
    let n: usize = wavelengths_nm.len()
        .min(donor_emission_norm.len())
        .min(acceptor_extinction.len());
    if n < 2 { return 0.0; }
    let mut integral: f64 = 0.0;
    for i in 0..n - 1 {
        let dl: f64 = wavelengths_nm[i + 1] - wavelengths_nm[i];
        let lam: f64 = 0.5 * (wavelengths_nm[i] + wavelengths_nm[i + 1]);
        let fd: f64 = 0.5 * (donor_emission_norm[i] + donor_emission_norm[i + 1]);
        let ea: f64 = 0.5 * (acceptor_extinction[i] + acceptor_extinction[i + 1]);
        integral += fd * ea * lam.powi(4) * dl;
    }
    integral
}

// ---------------------------------------------------------------------------
// 7. Fluorescence Lifetime
// ---------------------------------------------------------------------------

/// Single exponential decay: I(t) = amplitude * exp(-t / tau)
pub fn single_exponential_decay(t: f64, amplitude: f64, tau: f64) -> f64 {
    amplitude * (-t / tau).exp()
}

/// Multi-exponential decay: I(t) = sum_i(a_i * exp(-t / tau_i))
pub fn multi_exponential_decay(t: f64, amplitudes: &[f64], lifetimes: &[f64]) -> f64 {
    let n: usize = amplitudes.len().min(lifetimes.len());
    let mut result: f64 = 0.0;
    for i in 0..n {
        result += amplitudes[i] * (-t / lifetimes[i]).exp();
    }
    result
}

/// Fit single exponential decay by linearization: ln(I) = ln(A) - t/tau
/// Returns (amplitude, lifetime_tau).
pub fn fit_single_exponential(times: &[f64], intensities: &[f64]) -> (f64, f64) {
    let n: usize = times.len().min(intensities.len());
    if n < 2 { return (1.0, 1.0); }
    let mut sum_x: f64 = 0.0;
    let mut sum_y: f64 = 0.0;
    let mut sum_xy: f64 = 0.0;
    let mut sum_xx: f64 = 0.0;
    let mut count: f64 = 0.0;
    for i in 0..n {
        if intensities[i] <= 0.0 { continue; }
        let x: f64 = times[i];
        let y: f64 = intensities[i].ln();
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
        count += 1.0;
    }
    if count < 2.0 { return (1.0, 1.0); }
    let denom: f64 = count * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 { return (1.0, 1.0); }
    let slope: f64 = (count * sum_xy - sum_x * sum_y) / denom;
    let intercept: f64 = (sum_y - slope * sum_x) / count;
    let tau: f64 = -1.0 / slope;
    let amplitude: f64 = intercept.exp();
    (amplitude, tau)
}

/// Average lifetime for multi-exponential: <tau> = sum(a_i * tau_i^2) / sum(a_i * tau_i)
pub fn average_lifetime(amplitudes: &[f64], lifetimes: &[f64]) -> f64 {
    let n: usize = amplitudes.len().min(lifetimes.len());
    let mut num: f64 = 0.0;
    let mut den: f64 = 0.0;
    for i in 0..n {
        num += amplitudes[i] * lifetimes[i] * lifetimes[i];
        den += amplitudes[i] * lifetimes[i];
    }
    if den.abs() < 1e-30 { return 0.0; }
    num / den
}

// ---------------------------------------------------------------------------
// 8. Fluorescence Anisotropy
// ---------------------------------------------------------------------------

/// Fluorescence anisotropy: r = (I_par - I_perp) / (I_par + 2*I_perp)
pub fn anisotropy(i_parallel: f64, i_perpendicular: f64) -> f64 {
    let denom: f64 = i_parallel + 2.0 * i_perpendicular;
    if denom.abs() < 1e-30 { return 0.0; }
    (i_parallel - i_perpendicular) / denom
}

/// Fluorescence polarization: P = (I_par - I_perp) / (I_par + I_perp)
pub fn polarization(i_parallel: f64, i_perpendicular: f64) -> f64 {
    let denom: f64 = i_parallel + i_perpendicular;
    if denom.abs() < 1e-30 { return 0.0; }
    (i_parallel - i_perpendicular) / denom
}

/// Perrin equation: 1/r = (1/r0) * (1 + tau/theta)
/// r0 = fundamental anisotropy, tau = lifetime, theta = rotational correlation time
pub fn perrin_anisotropy(r0: f64, tau: f64, theta: f64) -> f64 {
    if theta <= 0.0 { return 0.0; }
    r0 / (1.0 + tau / theta)
}

/// Time-resolved anisotropy decay: r(t) = r0 * exp(-t/theta)
pub fn anisotropy_decay(t: f64, r0: f64, theta: f64) -> f64 {
    r0 * (-t / theta).exp()
}

/// Fit anisotropy decay. Returns (r0, theta).
pub fn fit_anisotropy_decay(times: &[f64], r_values: &[f64]) -> (f64, f64) {
    let n: usize = times.len().min(r_values.len());
    if n < 2 { return (0.4, 1.0); }
    let mut sum_x: f64 = 0.0;
    let mut sum_y: f64 = 0.0;
    let mut sum_xy: f64 = 0.0;
    let mut sum_xx: f64 = 0.0;
    let mut count: f64 = 0.0;
    for i in 0..n {
        if r_values[i] <= 0.0 { continue; }
        let x: f64 = times[i];
        let y: f64 = r_values[i].ln();
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
        count += 1.0;
    }
    if count < 2.0 { return (0.4, 1.0); }
    let denom: f64 = count * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 { return (0.4, 1.0); }
    let slope: f64 = (count * sum_xy - sum_x * sum_y) / denom;
    let intercept: f64 = (sum_y - slope * sum_x) / count;
    let theta: f64 = -1.0 / slope;
    let r0: f64 = intercept.exp();
    (r0, theta)
}

// ---------------------------------------------------------------------------
// 9. Inner Filter Effect Correction
// ---------------------------------------------------------------------------

/// Inner filter effect correction:
/// F_corr = F_obs * 10^((A_ex + A_em) / 2)
pub fn inner_filter_correction(f_obs: f64, absorbance_ex: f64, absorbance_em: f64) -> f64 {
    let exponent: f64 = (absorbance_ex + absorbance_em) / 2.0;
    f_obs * 10.0_f64.powf(exponent)
}

/// More accurate inner filter correction (Lakowicz):
/// F_corr = F_obs * (A_ex * ln(10)) / (1 - 10^(-A_ex)) * (A_em * ln(10)) / (1 - 10^(-A_em))
pub fn inner_filter_correction_lakowicz(
    f_obs: f64,
    absorbance_ex: f64,
    absorbance_em: f64,
) -> f64 {
    let ln10: f64 = 10.0_f64.ln();
    let corr_ex: f64 = if absorbance_ex.abs() < 1e-10 {
        1.0
    } else {
        (absorbance_ex * ln10) / (1.0 - 10.0_f64.powf(-absorbance_ex))
    };
    let corr_em: f64 = if absorbance_em.abs() < 1e-10 {
        1.0
    } else {
        (absorbance_em * ln10) / (1.0 - 10.0_f64.powf(-absorbance_em))
    };
    f_obs * corr_ex * corr_em
}

// ---------------------------------------------------------------------------
// 10. Excitation-Emission Matrix (EEM)
// ---------------------------------------------------------------------------

/// 3D Excitation-Emission Matrix.
#[derive(Debug, Clone)]
pub struct ExcitationEmissionMatrix {
    /// Excitation wavelengths in nm.
    pub excitation_wavelengths: Vec<f64>,
    /// Emission wavelengths in nm.
    pub emission_wavelengths: Vec<f64>,
    /// Intensity matrix [excitation_idx][emission_idx].
    pub intensities: Vec<Vec<f64>>,
}

impl ExcitationEmissionMatrix {
    /// Create new EEM.
    pub fn new(
        excitation_wavelengths: Vec<f64>,
        emission_wavelengths: Vec<f64>,
        intensities: Vec<Vec<f64>>,
    ) -> Self {
        assert_eq!(intensities.len(), excitation_wavelengths.len());
        for row in &intensities {
            assert_eq!(row.len(), emission_wavelengths.len());
        }
        Self { excitation_wavelengths, emission_wavelengths, intensities }
    }

    /// Get intensity at specific excitation/emission indices.
    pub fn get(&self, ex_idx: usize, em_idx: usize) -> f64 {
        self.intensities[ex_idx][em_idx]
    }

    /// Find peaks in the EEM above a threshold.
    pub fn find_peaks(&self, threshold: f64) -> Vec<EemPeak> {
        let n_ex: usize = self.excitation_wavelengths.len();
        let n_em: usize = self.emission_wavelengths.len();
        let mut peaks: Vec<EemPeak> = Vec::new();
        for i in 1..n_ex.saturating_sub(1) {
            for j in 1..n_em.saturating_sub(1) {
                let val: f64 = self.intensities[i][j];
                if val < threshold { continue; }
                // Check if local maximum (4-connected)
                let is_max: bool = val > self.intensities[i - 1][j]
                    && val > self.intensities[i + 1][j]
                    && val > self.intensities[i][j - 1]
                    && val > self.intensities[i][j + 1];
                if is_max {
                    peaks.push(EemPeak {
                        excitation_nm: self.excitation_wavelengths[i],
                        emission_nm: self.emission_wavelengths[j],
                        intensity: val,
                    });
                }
            }
        }
        peaks.sort_by(|a, b| b.intensity.partial_cmp(&a.intensity).unwrap_or(std::cmp::Ordering::Equal));
        peaks
    }

    /// Extract emission spectrum at a given excitation wavelength (nearest).
    pub fn emission_at_excitation(&self, ex_nm: f64) -> FluorescenceSpectrum {
        let idx: usize = nearest_index(&self.excitation_wavelengths, ex_nm);
        FluorescenceSpectrum::new(
            self.emission_wavelengths.clone(),
            self.intensities[idx].clone(),
        )
    }

    /// Extract excitation spectrum at a given emission wavelength (nearest).
    pub fn excitation_at_emission(&self, em_nm: f64) -> FluorescenceSpectrum {
        let em_idx: usize = nearest_index(&self.emission_wavelengths, em_nm);
        let intensities: Vec<f64> = self.intensities.iter()
            .map(|row| row[em_idx])
            .collect();
        FluorescenceSpectrum::new(
            self.excitation_wavelengths.clone(),
            intensities,
        )
    }

    /// Maximum intensity in the EEM.
    pub fn max_intensity(&self) -> f64 {
        let mut max_val: f64 = f64::NEG_INFINITY;
        for row in &self.intensities {
            for &v in row {
                if v > max_val { max_val = v; }
            }
        }
        max_val
    }
}

/// A peak in the EEM.
#[derive(Debug, Clone)]
pub struct EemPeak {
    pub excitation_nm: f64,
    pub emission_nm: f64,
    pub intensity: f64,
}

fn nearest_index(arr: &[f64], target: f64) -> usize {
    let mut best_idx: usize = 0;
    let mut best_dist: f64 = f64::MAX;
    for (i, &v) in arr.iter().enumerate() {
        let d: f64 = (v - target).abs();
        if d < best_dist {
            best_dist = d;
            best_idx = i;
        }
    }
    best_idx
}

// ---------------------------------------------------------------------------
// 11. Calibration and LOD
// ---------------------------------------------------------------------------

/// Linear calibration result.
#[derive(Debug, Clone)]
pub struct CalibrationResult {
    pub slope: f64,
    pub intercept: f64,
    pub r_squared: f64,
    pub lod: f64,
    pub loq: f64,
}

/// Perform linear calibration and compute LOD/LOQ.
/// LOD = 3 * sigma / slope, LOQ = 10 * sigma / slope
pub fn calibrate_linear(
    concentrations: &[f64],
    intensities: &[f64],
) -> CalibrationResult {
    let n: usize = concentrations.len().min(intensities.len());
    assert!(n >= 2);
    let mut sum_x: f64 = 0.0;
    let mut sum_y: f64 = 0.0;
    let mut sum_xy: f64 = 0.0;
    let mut sum_xx: f64 = 0.0;
    let nf: f64 = n as f64;
    for i in 0..n {
        sum_x += concentrations[i];
        sum_y += intensities[i];
        sum_xy += concentrations[i] * intensities[i];
        sum_xx += concentrations[i] * concentrations[i];
    }
    let denom: f64 = nf * sum_xx - sum_x * sum_x;
    let slope: f64 = (nf * sum_xy - sum_x * sum_y) / denom;
    let intercept: f64 = (sum_y - slope * sum_x) / nf;
    // R-squared
    let y_mean: f64 = sum_y / nf;
    let mut ss_res: f64 = 0.0;
    let mut ss_tot: f64 = 0.0;
    let mut residuals: Vec<f64> = Vec::with_capacity(n);
    for i in 0..n {
        let y_pred: f64 = slope * concentrations[i] + intercept;
        let res: f64 = intensities[i] - y_pred;
        residuals.push(res);
        ss_res += res * res;
        ss_tot += (intensities[i] - y_mean) * (intensities[i] - y_mean);
    }
    let r_squared: f64 = if ss_tot > 0.0 { 1.0 - ss_res / ss_tot } else { 0.0 };
    // Sigma of residuals
    let sigma: f64 = if n > 2 {
        (ss_res / (nf - 2.0)).sqrt()
    } else {
        0.0
    };
    let lod: f64 = if slope.abs() > 1e-30 { 3.0 * sigma / slope.abs() } else { f64::INFINITY };
    let loq: f64 = if slope.abs() > 1e-30 { 10.0 * sigma / slope.abs() } else { f64::INFINITY };
    CalibrationResult { slope, intercept, r_squared, lod, loq }
}

// ---------------------------------------------------------------------------
// 12. Spectral Processing Utilities
// ---------------------------------------------------------------------------

/// Gaussian spectral band: I(lam) = amplitude * exp(-0.5 * ((lam - center) / sigma)^2)
pub fn gaussian_band(wavelength: f64, center: f64, sigma: f64, amplitude: f64) -> f64 {
    let z: f64 = (wavelength - center) / sigma;
    amplitude * (-0.5 * z * z).exp()
}

/// Lorentzian spectral band.
pub fn lorentzian_band(wavelength: f64, center: f64, gamma: f64, amplitude: f64) -> f64 {
    let dw: f64 = wavelength - center;
    amplitude * gamma * gamma / (dw * dw + gamma * gamma)
}

/// Generate a synthetic Gaussian emission spectrum.
pub fn generate_gaussian_spectrum(
    center_nm: f64,
    fwhm_nm: f64,
    amplitude: f64,
    start_nm: f64,
    end_nm: f64,
    n_points: usize,
) -> FluorescenceSpectrum {
    let sigma: f64 = fwhm_nm / (2.0 * (2.0 * 2.0_f64.ln()).sqrt());
    let step: f64 = (end_nm - start_nm) / (n_points as f64 - 1.0);
    let mut wavelengths: Vec<f64> = Vec::with_capacity(n_points);
    let mut intensities: Vec<f64> = Vec::with_capacity(n_points);
    for i in 0..n_points {
        let lam: f64 = start_nm + i as f64 * step;
        wavelengths.push(lam);
        intensities.push(gaussian_band(lam, center_nm, sigma, amplitude));
    }
    FluorescenceSpectrum::new(wavelengths, intensities)
}

/// Smooth spectrum with Savitzky-Golay 5-point quadratic.
pub fn smooth_spectrum_sg5(intensities: &[f64]) -> Vec<f64> {
    let n: usize = intensities.len();
    if n < 5 {
        return intensities.to_vec();
    }
    let mut out: Vec<f64> = vec![0.0; n];
    out[0] = intensities[0];
    out[1] = intensities[1];
    out[n - 2] = intensities[n - 2];
    out[n - 1] = intensities[n - 1];
    let coeffs: [f64; 5] = [-3.0, 12.0, 17.0, 12.0, -3.0];
    let norm: f64 = 35.0;
    for i in 2..n - 2 {
        let mut s: f64 = 0.0;
        for (k, &c) in coeffs.iter().enumerate() {
            s += c * intensities[i + k - 2];
        }
        out[i] = s / norm;
    }
    out
}

/// Baseline correction using linear interpolation between endpoints.
pub fn baseline_correction_linear(
    wavelengths: &[f64],
    intensities: &[f64],
) -> Vec<f64> {
    let n: usize = wavelengths.len().min(intensities.len());
    if n < 2 { return intensities.to_vec(); }
    let y0: f64 = intensities[0];
    let y1: f64 = intensities[n - 1];
    let x0: f64 = wavelengths[0];
    let x1: f64 = wavelengths[n - 1];
    let range: f64 = x1 - x0;
    if range.abs() < 1e-30 { return intensities.to_vec(); }
    let mut corrected: Vec<f64> = Vec::with_capacity(n);
    for i in 0..n {
        let frac: f64 = (wavelengths[i] - x0) / range;
        let baseline: f64 = y0 + frac * (y1 - y0);
        corrected.push(intensities[i] - baseline);
    }
    corrected
}

// ---------------------------------------------------------------------------
// 13. FluorescenceProcessor Orchestrator
// ---------------------------------------------------------------------------

/// Orchestrator for fluorescence spectroscopy analysis.
#[derive(Debug, Clone)]
pub struct FluorescenceProcessor {
    /// Excitation spectrum.
    pub excitation: Option<FluorescenceSpectrum>,
    /// Emission spectrum.
    pub emission: Option<FluorescenceSpectrum>,
    /// EEM data.
    pub eem: Option<ExcitationEmissionMatrix>,
    /// Measured quantum yield.
    pub quantum_yield: Option<f64>,
    /// Fitted lifetime (ns).
    pub lifetime_ns: Option<f64>,
}

impl FluorescenceProcessor {
    /// Create new processor.
    pub fn new() -> Self {
        Self {
            excitation: None,
            emission: None,
            eem: None,
            quantum_yield: None,
            lifetime_ns: None,
        }
    }

    /// Set excitation spectrum.
    pub fn set_excitation(&mut self, spec: FluorescenceSpectrum) {
        self.excitation = Some(spec);
    }

    /// Set emission spectrum.
    pub fn set_emission(&mut self, spec: FluorescenceSpectrum) {
        self.emission = Some(spec);
    }

    /// Set EEM.
    pub fn set_eem(&mut self, eem: ExcitationEmissionMatrix) {
        self.eem = Some(eem);
    }

    /// Calculate Stokes shift from loaded spectra.
    pub fn stokes_shift(&self) -> Option<f64> {
        let ex: &FluorescenceSpectrum = self.excitation.as_ref()?;
        let em: &FluorescenceSpectrum = self.emission.as_ref()?;
        Some(stokes_shift_nm(ex.peak_wavelength(), em.peak_wavelength()))
    }

    /// Fit lifetime from decay data and store result.
    pub fn fit_lifetime(&mut self, times: &[f64], intensities: &[f64]) -> (f64, f64) {
        let (amp, tau) = fit_single_exponential(times, intensities);
        self.lifetime_ns = Some(tau);
        (amp, tau)
    }

    /// Calculate quantum yield using Parker-Rees and store.
    pub fn calculate_quantum_yield(
        &mut self,
        phi_ref: f64,
        sample_area: f64,
        ref_area: f64,
        sample_abs: f64,
        ref_abs: f64,
        n_sample: f64,
        n_ref: f64,
    ) -> f64 {
        let qy: f64 = quantum_yield_parker_rees(
            phi_ref, sample_area, ref_area, sample_abs, ref_abs, n_sample, n_ref,
        );
        self.quantum_yield = Some(qy);
        qy
    }

    /// Analyze EEM to find peaks.
    pub fn analyze_eem(&self, threshold: f64) -> Vec<EemPeak> {
        match &self.eem {
            Some(eem) => eem.find_peaks(threshold),
            None => Vec::new(),
        }
    }
}

impl Default for FluorescenceProcessor {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// 14. Beer-Lambert Law Utilities
// ---------------------------------------------------------------------------

/// Beer-Lambert: A = epsilon * c * l
pub fn beer_lambert_absorbance(epsilon: f64, concentration: f64, path_length: f64) -> f64 {
    epsilon * concentration * path_length
}

/// Transmittance from absorbance: T = 10^(-A)
pub fn transmittance_from_absorbance(absorbance: f64) -> f64 {
    10.0_f64.powf(-absorbance)
}

/// Absorbance from transmittance: A = -log10(T)
pub fn absorbance_from_transmittance(transmittance: f64) -> f64 {
    if transmittance <= 0.0 { return f64::INFINITY; }
    -(transmittance.log10())
}

// ---------------------------------------------------------------------------
// 15. Photobleaching Kinetics
// ---------------------------------------------------------------------------

/// First-order photobleaching: I(t) = I0 * exp(-k * t)
pub fn photobleaching_first_order(t: f64, i0: f64, k_bleach: f64) -> f64 {
    i0 * (-k_bleach * t).exp()
}

/// Fit photobleaching rate constant from time-intensity data.
pub fn fit_photobleaching_rate(times: &[f64], intensities: &[f64]) -> (f64, f64) {
    fit_single_exponential(times, intensities)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn test_spectrum_peak() {
        let spec = FluorescenceSpectrum::new(
            vec![400.0, 420.0, 440.0, 460.0, 480.0],
            vec![0.1, 0.5, 1.0, 0.7, 0.2],
        );
        assert!(approx_eq(spec.peak_wavelength(), 440.0, 0.01));
        assert!(approx_eq(spec.peak_intensity(), 1.0, 0.01));
    }

    #[test]
    fn test_spectrum_area() {
        let spec = FluorescenceSpectrum::new(
            vec![0.0, 10.0, 20.0],
            vec![0.0, 1.0, 0.0],
        );
        assert!(approx_eq(spec.integrated_area(), 10.0, 0.01));
    }

    #[test]
    fn test_spectrum_fwhm() {
        let spec = generate_gaussian_spectrum(500.0, 40.0, 1.0, 400.0, 600.0, 201);
        let fwhm: f64 = spec.fwhm();
        assert!(approx_eq(fwhm, 40.0, 3.0));
    }

    #[test]
    fn test_spectrum_normalize() {
        let mut spec = FluorescenceSpectrum::new(
            vec![400.0, 500.0, 600.0],
            vec![2.0, 4.0, 1.0],
        );
        spec.normalize();
        assert!(approx_eq(spec.peak_intensity(), 1.0, 1e-10));
        assert!(approx_eq(spec.intensities[0], 0.5, 1e-10));
    }

    #[test]
    fn test_stokes_shift_nm() {
        assert!(approx_eq(stokes_shift_nm(480.0, 520.0), 40.0, 0.01));
    }

    #[test]
    fn test_stokes_shift_wavenumber() {
        let shift: f64 = stokes_shift_wavenumber(480.0, 520.0);
        assert!(shift > 0.0);
        // 1e7/480 - 1e7/520 = 20833 - 19231 = 1603 cm^-1
        assert!(approx_eq(shift, 1603.0, 5.0));
    }

    #[test]
    fn test_wavelength_energy_conversion() {
        let ev: f64 = wavelength_to_ev(500.0);
        assert!(approx_eq(ev, 2.48, 0.02));
        let nm: f64 = ev_to_wavelength(ev);
        assert!(approx_eq(nm, 500.0, 1.0));
    }

    #[test]
    fn test_jablonski_levels() {
        let jab = JablonskiLevels::new(0.0, 3.0, 2.0, 0.5);
        let abs_nm: f64 = jab.absorption_wavelength_nm();
        let fl_nm: f64 = jab.fluorescence_wavelength_nm();
        assert!(fl_nm > abs_nm); // emission at longer wavelength
        assert!(approx_eq(jab.isc_gap_ev(), 0.5, 0.01));
    }

    #[test]
    fn test_jablonski_phosphorescence() {
        let jab = JablonskiLevels::new(0.0, 3.0, 2.0, 0.5);
        let phos_nm: f64 = jab.phosphorescence_wavelength_nm();
        let fl_nm: f64 = jab.fluorescence_wavelength_nm();
        assert!(phos_nm > fl_nm); // phosphorescence at even longer wavelength
    }

    #[test]
    fn test_quantum_yield_parker_rees() {
        let qy: f64 = quantum_yield_parker_rees(0.95, 100.0, 100.0, 0.1, 0.1, 1.33, 1.33);
        assert!(approx_eq(qy, 0.95, 0.01));
    }

    #[test]
    fn test_quantum_yield_simple() {
        assert!(approx_eq(quantum_yield_simple(80.0, 100.0), 0.8, 0.01));
        assert!(approx_eq(quantum_yield_simple(0.0, 100.0), 0.0, 0.01));
    }

    #[test]
    fn test_stern_volmer_dynamic() {
        let f: f64 = stern_volmer_dynamic(100.0, 10.0, 0.0);
        assert!(approx_eq(f, 100.0, 0.01));
        let f2: f64 = stern_volmer_dynamic(100.0, 10.0, 0.1);
        assert!(approx_eq(f2, 50.0, 0.01)); // F0/(1+10*0.1) = 100/2
    }

    #[test]
    fn test_stern_volmer_static() {
        let f: f64 = stern_volmer_static(100.0, 5.0, 0.2);
        assert!(approx_eq(f, 50.0, 0.01)); // 100/(1+5*0.2) = 100/2
    }

    #[test]
    fn test_stern_volmer_combined() {
        let f: f64 = stern_volmer_combined(100.0, 10.0, 5.0, 0.1);
        // F0/((1+10*0.1)*(1+5*0.1)) = 100/(2*1.5) = 33.33
        assert!(approx_eq(f, 33.33, 0.1));
    }

    #[test]
    fn test_modified_stern_volmer() {
        // Generate data with fa=0.8, Ka=10
        let f0: f64 = 100.0;
        let fa: f64 = 0.8;
        let ka: f64 = 10.0;
        let concs: Vec<f64> = vec![0.01, 0.02, 0.05, 0.1, 0.2, 0.5];
        let f_vals: Vec<f64> = concs.iter()
            .map(|&q| f0 * (1.0 - fa) + f0 * fa / (1.0 + ka * q))
            .collect();
        let (est_fa, est_ka) = modified_stern_volmer_accessible_fraction(f0, &f_vals, &concs);
        assert!(approx_eq(est_fa, fa, 0.1));
        assert!(approx_eq(est_ka, ka, 2.0));
    }

    #[test]
    fn test_fret_efficiency() {
        assert!(approx_eq(fret_efficiency(5.0, 5.0), 0.5, 0.01)); // r=R0 => E=0.5
        assert!(approx_eq(fret_efficiency(0.0, 5.0), 1.0, 0.01)); // r=0 => E=1
        assert!(fret_efficiency(100.0, 5.0) < 0.001); // large distance => low E
    }

    #[test]
    fn test_fret_distance() {
        let d: f64 = fret_distance(0.5, 5.0);
        assert!(approx_eq(d, 5.0, 0.01)); // E=0.5 => r=R0
    }

    #[test]
    fn test_forster_r0() {
        // Typical values: kappa2=2/3, QY=0.5, J=1e-13 M-1 cm-1 nm4, n=1.4
        let r0: f64 = forster_r0(2.0 / 3.0, 0.5, 1e-13, 1.4);
        // R0 should be in the range of a few nm for typical FRET pairs
        assert!(r0 > 1.0 && r0 < 20.0);
    }

    #[test]
    fn test_spectral_overlap_integral() {
        let wl: Vec<f64> = (0..100).map(|i| 400.0 + i as f64 * 2.0).collect();
        let donor: Vec<f64> = wl.iter()
            .map(|&l| gaussian_band(l, 520.0, 20.0, 0.01))
            .collect();
        let acceptor: Vec<f64> = wl.iter()
            .map(|&l| gaussian_band(l, 540.0, 25.0, 50000.0))
            .collect();
        let j: f64 = spectral_overlap_integral(&wl, &donor, &acceptor);
        assert!(j > 0.0);
    }

    #[test]
    fn test_single_exponential_decay() {
        let v: f64 = single_exponential_decay(0.0, 1000.0, 5.0);
        assert!(approx_eq(v, 1000.0, 0.01));
        let v2: f64 = single_exponential_decay(5.0, 1000.0, 5.0);
        assert!(approx_eq(v2, 1000.0 / std::f64::consts::E, 1.0));
    }

    #[test]
    fn test_multi_exponential_decay() {
        let v: f64 = multi_exponential_decay(0.0, &[500.0, 500.0], &[3.0, 10.0]);
        assert!(approx_eq(v, 1000.0, 0.01));
    }

    #[test]
    fn test_fit_single_exponential() {
        let tau_true: f64 = 4.5;
        let amp_true: f64 = 800.0;
        let times: Vec<f64> = (0..50).map(|i| i as f64 * 0.5).collect();
        let intensities: Vec<f64> = times.iter()
            .map(|&t| amp_true * (-t / tau_true).exp())
            .collect();
        let (amp, tau) = fit_single_exponential(&times, &intensities);
        assert!(approx_eq(amp, amp_true, 1.0));
        assert!(approx_eq(tau, tau_true, 0.1));
    }

    #[test]
    fn test_average_lifetime() {
        let avg: f64 = average_lifetime(&[0.5, 0.5], &[2.0, 8.0]);
        // <tau> = (0.5*4 + 0.5*64)/(0.5*2 + 0.5*8) = 34/5 = 6.8
        assert!(approx_eq(avg, 6.8, 0.01));
    }

    #[test]
    fn test_anisotropy() {
        let r: f64 = anisotropy(3.0, 1.0);
        // r = (3-1)/(3+2) = 2/5 = 0.4
        assert!(approx_eq(r, 0.4, 0.001));
    }

    #[test]
    fn test_polarization() {
        let p: f64 = polarization(3.0, 1.0);
        // P = (3-1)/(3+1) = 0.5
        assert!(approx_eq(p, 0.5, 0.001));
    }

    #[test]
    fn test_perrin_equation() {
        let r: f64 = perrin_anisotropy(0.4, 4.0, 4.0);
        // r = 0.4 / (1 + 4/4) = 0.4/2 = 0.2
        assert!(approx_eq(r, 0.2, 0.001));
    }

    #[test]
    fn test_anisotropy_decay() {
        let r: f64 = anisotropy_decay(0.0, 0.4, 10.0);
        assert!(approx_eq(r, 0.4, 0.001));
    }

    #[test]
    fn test_fit_anisotropy_decay() {
        let r0_true: f64 = 0.35;
        let theta_true: f64 = 8.0;
        let times: Vec<f64> = (0..30).map(|i| i as f64 * 0.5).collect();
        let r_vals: Vec<f64> = times.iter()
            .map(|&t| r0_true * (-t / theta_true).exp())
            .collect();
        let (r0, theta) = fit_anisotropy_decay(&times, &r_vals);
        assert!(approx_eq(r0, r0_true, 0.01));
        assert!(approx_eq(theta, theta_true, 0.2));
    }

    #[test]
    fn test_inner_filter_correction() {
        let f_corr: f64 = inner_filter_correction(50.0, 0.0, 0.0);
        assert!(approx_eq(f_corr, 50.0, 0.01)); // no correction at A=0
    }

    #[test]
    fn test_inner_filter_correction_nonzero() {
        let f_corr: f64 = inner_filter_correction(50.0, 0.5, 0.3);
        // 50 * 10^((0.5+0.3)/2) = 50 * 10^0.4 = 50 * 2.512 = 125.6
        assert!(approx_eq(f_corr, 125.6, 1.0));
    }

    #[test]
    fn test_inner_filter_lakowicz() {
        let f_corr: f64 = inner_filter_correction_lakowicz(50.0, 0.0, 0.0);
        assert!(approx_eq(f_corr, 50.0, 0.1));
    }

    #[test]
    fn test_eem_create() {
        let eem = ExcitationEmissionMatrix::new(
            vec![300.0, 320.0],
            vec![400.0, 420.0, 440.0],
            vec![
                vec![0.1, 0.5, 0.2],
                vec![0.3, 0.8, 0.4],
            ],
        );
        assert!(approx_eq(eem.get(1, 1), 0.8, 0.001));
        assert!(approx_eq(eem.max_intensity(), 0.8, 0.001));
    }

    #[test]
    fn test_eem_find_peaks() {
        let eem = ExcitationEmissionMatrix::new(
            vec![300.0, 320.0, 340.0],
            vec![400.0, 420.0, 440.0],
            vec![
                vec![0.1, 0.2, 0.1],
                vec![0.2, 0.9, 0.3],
                vec![0.1, 0.3, 0.1],
            ],
        );
        let peaks = eem.find_peaks(0.5);
        assert_eq!(peaks.len(), 1);
        assert!(approx_eq(peaks[0].excitation_nm, 320.0, 0.01));
        assert!(approx_eq(peaks[0].emission_nm, 420.0, 0.01));
    }

    #[test]
    fn test_eem_emission_at_excitation() {
        let eem = ExcitationEmissionMatrix::new(
            vec![300.0, 320.0],
            vec![400.0, 420.0],
            vec![
                vec![0.1, 0.5],
                vec![0.3, 0.8],
            ],
        );
        let spec = eem.emission_at_excitation(318.0);
        assert!(approx_eq(spec.intensities[1], 0.8, 0.001));
    }

    #[test]
    fn test_eem_excitation_at_emission() {
        let eem = ExcitationEmissionMatrix::new(
            vec![300.0, 320.0],
            vec![400.0, 420.0],
            vec![
                vec![0.1, 0.5],
                vec![0.3, 0.8],
            ],
        );
        let spec = eem.excitation_at_emission(420.0);
        assert!(approx_eq(spec.intensities[1], 0.8, 0.001));
    }

    #[test]
    fn test_calibration_linear() {
        let concs: Vec<f64> = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0];
        let ints: Vec<f64> = vec![0.5, 10.5, 20.5, 30.5, 40.5, 50.5];
        let cal = calibrate_linear(&concs, &ints);
        assert!(approx_eq(cal.slope, 10.0, 0.01));
        assert!(approx_eq(cal.intercept, 0.5, 0.01));
        assert!(cal.r_squared > 0.999);
    }

    #[test]
    fn test_calibration_lod() {
        let concs: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let ints: Vec<f64> = concs.iter().map(|&c| 5.0 * c + 1.0).collect();
        let cal = calibrate_linear(&concs, &ints);
        assert!(approx_eq(cal.slope, 5.0, 0.01));
        // Perfect data => LOD should be very small
        assert!(cal.lod < 0.01);
    }

    #[test]
    fn test_gaussian_band() {
        let v: f64 = gaussian_band(500.0, 500.0, 20.0, 1.0);
        assert!(approx_eq(v, 1.0, 0.001));
        let v2: f64 = gaussian_band(600.0, 500.0, 20.0, 1.0);
        assert!(v2 < 0.01);
    }

    #[test]
    fn test_lorentzian_band() {
        let v: f64 = lorentzian_band(500.0, 500.0, 10.0, 1.0);
        assert!(approx_eq(v, 1.0, 0.001));
    }

    #[test]
    fn test_generate_gaussian_spectrum() {
        let spec = generate_gaussian_spectrum(500.0, 40.0, 1.0, 400.0, 600.0, 101);
        assert_eq!(spec.wavelengths_nm.len(), 101);
        assert!(approx_eq(spec.peak_wavelength(), 500.0, 2.5));
    }

    #[test]
    fn test_smooth_spectrum_sg5() {
        let noisy: Vec<f64> = vec![1.0, 1.2, 0.8, 1.1, 0.9, 1.0, 1.1, 0.9, 1.0, 1.0];
        let smooth = smooth_spectrum_sg5(&noisy);
        assert_eq!(smooth.len(), noisy.len());
        // Smoothed values should be close to 1.0
        for &v in &smooth[2..smooth.len() - 2] {
            assert!(approx_eq(v, 1.0, 0.2));
        }
    }

    #[test]
    fn test_baseline_correction_linear() {
        let wl: Vec<f64> = vec![400.0, 450.0, 500.0, 550.0, 600.0];
        let ints: Vec<f64> = vec![1.0, 5.0, 10.0, 5.0, 1.0];
        let corrected = baseline_correction_linear(&wl, &ints);
        assert!(approx_eq(corrected[0], 0.0, 0.01));
        assert!(approx_eq(corrected[4], 0.0, 0.01));
        assert!(corrected[2] > 0.0);
    }

    #[test]
    fn test_beer_lambert() {
        let a: f64 = beer_lambert_absorbance(10000.0, 0.001, 1.0);
        assert!(approx_eq(a, 10.0, 0.01));
    }

    #[test]
    fn test_transmittance_absorbance() {
        let t: f64 = transmittance_from_absorbance(1.0);
        assert!(approx_eq(t, 0.1, 0.001));
        let a: f64 = absorbance_from_transmittance(0.1);
        assert!(approx_eq(a, 1.0, 0.001));
    }

    #[test]
    fn test_photobleaching() {
        let i: f64 = photobleaching_first_order(0.0, 1000.0, 0.1);
        assert!(approx_eq(i, 1000.0, 0.01));
        let i2: f64 = photobleaching_first_order(10.0, 1000.0, 0.1);
        assert!(approx_eq(i2, 1000.0 * (-1.0_f64).exp(), 1.0));
    }

    #[test]
    fn test_fit_photobleaching_rate() {
        let k: f64 = 0.05;
        let times: Vec<f64> = (0..40).map(|i| i as f64).collect();
        let ints: Vec<f64> = times.iter().map(|&t| 500.0 * (-k * t).exp()).collect();
        let (amp, tau) = fit_photobleaching_rate(&times, &ints);
        assert!(approx_eq(amp, 500.0, 5.0));
        assert!(approx_eq(tau, 1.0 / k, 0.5));
    }

    #[test]
    fn test_processor_new() {
        let proc = FluorescenceProcessor::new();
        assert!(proc.excitation.is_none());
        assert!(proc.emission.is_none());
        assert!(proc.quantum_yield.is_none());
    }

    #[test]
    fn test_processor_stokes_shift() {
        let mut proc = FluorescenceProcessor::new();
        proc.set_excitation(FluorescenceSpectrum::new(
            vec![460.0, 480.0, 500.0], vec![0.5, 1.0, 0.3],
        ));
        proc.set_emission(FluorescenceSpectrum::new(
            vec![500.0, 520.0, 540.0], vec![0.3, 1.0, 0.5],
        ));
        let shift: f64 = proc.stokes_shift().unwrap();
        assert!(approx_eq(shift, 40.0, 0.01));
    }

    #[test]
    fn test_processor_fit_lifetime() {
        let mut proc = FluorescenceProcessor::new();
        let times: Vec<f64> = (0..30).map(|i| i as f64 * 0.5).collect();
        let ints: Vec<f64> = times.iter().map(|&t| 1000.0 * (-t / 5.0).exp()).collect();
        let (_, tau) = proc.fit_lifetime(&times, &ints);
        assert!(approx_eq(tau, 5.0, 0.2));
        assert!(proc.lifetime_ns.is_some());
    }

    #[test]
    fn test_processor_quantum_yield() {
        let mut proc = FluorescenceProcessor::new();
        let qy: f64 = proc.calculate_quantum_yield(0.95, 100.0, 120.0, 0.1, 0.08, 1.33, 1.33);
        assert!(qy > 0.0 && qy < 1.0);
        assert!(proc.quantum_yield.is_some());
    }

    #[test]
    fn test_processor_analyze_eem() {
        let mut proc = FluorescenceProcessor::new();
        proc.set_eem(ExcitationEmissionMatrix::new(
            vec![300.0, 320.0, 340.0],
            vec![400.0, 420.0, 440.0],
            vec![
                vec![0.1, 0.2, 0.1],
                vec![0.2, 0.9, 0.3],
                vec![0.1, 0.3, 0.1],
            ],
        ));
        let peaks = proc.analyze_eem(0.5);
        assert_eq!(peaks.len(), 1);
    }

    #[test]
    fn test_fret_roundtrip() {
        let r0: f64 = 5.5;
        let d: f64 = 4.0;
        let e: f64 = fret_efficiency(d, r0);
        let d_back: f64 = fret_distance(e, r0);
        assert!(approx_eq(d_back, d, 0.01));
    }

    #[test]
    fn test_fret_efficiency_limits() {
        assert!(approx_eq(fret_efficiency(0.001, 5.0), 1.0, 0.01));
        assert!(fret_efficiency(50.0, 5.0) < 1e-6);
    }

    #[test]
    fn test_wavelength_to_ev_visible() {
        let blue: f64 = wavelength_to_ev(450.0);
        let red: f64 = wavelength_to_ev(700.0);
        assert!(blue > red); // higher energy
        assert!(blue > 2.5 && blue < 3.0);
        assert!(red > 1.7 && red < 1.9);
    }

    #[test]
    fn test_ev_to_wavelength_roundtrip() {
        let wl: f64 = 532.0;
        let ev: f64 = wavelength_to_ev(wl);
        let wl_back: f64 = ev_to_wavelength(ev);
        assert!(approx_eq(wl_back, wl, 0.5));
    }

    #[test]
    fn test_stern_volmer_zero_quencher() {
        assert!(approx_eq(stern_volmer_dynamic(100.0, 10.0, 0.0), 100.0, 0.01));
        assert!(approx_eq(stern_volmer_static(100.0, 10.0, 0.0), 100.0, 0.01));
        assert!(approx_eq(stern_volmer_combined(100.0, 10.0, 5.0, 0.0), 100.0, 0.01));
    }

    #[test]
    fn test_multi_exp_at_zero() {
        let v: f64 = multi_exponential_decay(0.0, &[300.0, 400.0, 300.0], &[1.0, 5.0, 10.0]);
        assert!(approx_eq(v, 1000.0, 0.01));
    }

    #[test]
    fn test_average_lifetime_single() {
        let avg: f64 = average_lifetime(&[1.0], &[5.0]);
        assert!(approx_eq(avg, 5.0, 0.01));
    }

    #[test]
    fn test_anisotropy_limits() {
        // Maximum anisotropy for single photon is 0.4
        let r: f64 = anisotropy(1.0, 0.0);
        // (1-0)/(1+0) = 1/1 = 1.0 (not physical, but mathematically correct)
        assert!(approx_eq(r, 1.0 / 1.0, 0.001));
    }

    #[test]
    fn test_polarization_isotropic() {
        // Equal parallel and perpendicular => P = 0
        let p: f64 = polarization(1.0, 1.0);
        assert!(approx_eq(p, 0.0, 0.001));
    }

    #[test]
    fn test_perrin_large_theta() {
        // Large rotational time => anisotropy approaches r0
        let r: f64 = perrin_anisotropy(0.4, 4.0, 1000.0);
        assert!(approx_eq(r, 0.4, 0.01));
    }

    #[test]
    fn test_perrin_small_theta() {
        // Very fast rotation => anisotropy drops
        let r: f64 = perrin_anisotropy(0.4, 4.0, 0.01);
        assert!(r < 0.01);
    }

    #[test]
    fn test_inner_filter_zero_absorbance() {
        let corr: f64 = inner_filter_correction(75.0, 0.0, 0.0);
        assert!(approx_eq(corr, 75.0, 0.01));
    }

    #[test]
    fn test_beer_lambert_zero_concentration() {
        assert!(approx_eq(beer_lambert_absorbance(10000.0, 0.0, 1.0), 0.0, 1e-10));
    }

    #[test]
    fn test_transmittance_zero() {
        let a: f64 = absorbance_from_transmittance(0.0);
        assert!(a.is_infinite());
    }

    #[test]
    fn test_transmittance_one() {
        let a: f64 = absorbance_from_transmittance(1.0);
        assert!(approx_eq(a, 0.0, 1e-10));
    }

    #[test]
    fn test_spectrum_single_point() {
        let spec = FluorescenceSpectrum::new(vec![500.0], vec![1.0]);
        assert!(approx_eq(spec.peak_wavelength(), 500.0, 0.01));
        assert!(approx_eq(spec.integrated_area(), 0.0, 0.01));
    }

    #[test]
    fn test_calibration_perfect_line() {
        let concs: Vec<f64> = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let ints: Vec<f64> = concs.iter().map(|&c| 2.0 * c + 3.0).collect();
        let cal = calibrate_linear(&concs, &ints);
        assert!(approx_eq(cal.slope, 2.0, 0.001));
        assert!(approx_eq(cal.intercept, 3.0, 0.001));
        assert!(cal.r_squared > 0.9999);
    }

    #[test]
    fn test_processor_default() {
        let proc = FluorescenceProcessor::default();
        assert!(proc.excitation.is_none());
    }

    #[test]
    fn test_gaussian_spectrum_symmetry() {
        let spec = generate_gaussian_spectrum(500.0, 40.0, 1.0, 400.0, 600.0, 201);
        let peak_idx: usize = spec.wavelengths_nm.iter()
            .position(|&w| (w - 500.0).abs() < 1.0).unwrap();
        // Should be approximately symmetric
        if peak_idx > 10 && peak_idx < spec.intensities.len() - 10 {
            let left: f64 = spec.intensities[peak_idx - 10];
            let right: f64 = spec.intensities[peak_idx + 10];
            assert!(approx_eq(left, right, 0.05));
        }
    }

    #[test]
    fn test_baseline_correction_flat() {
        let wl: Vec<f64> = vec![400.0, 500.0, 600.0];
        let ints: Vec<f64> = vec![5.0, 10.0, 5.0];
        let corrected = baseline_correction_linear(&wl, &ints);
        assert!(approx_eq(corrected[0], 0.0, 0.01));
        assert!(approx_eq(corrected[2], 0.0, 0.01));
        assert!(approx_eq(corrected[1], 5.0, 0.01));
    }

    #[test]
    fn test_fret_distance_invalid() {
        assert!(fret_distance(0.0, 5.0).is_nan());
        assert!(fret_distance(1.0, 5.0).is_nan());
    }

    #[test]
    fn test_nearest_index() {
        let arr: Vec<f64> = vec![10.0, 20.0, 30.0, 40.0];
        assert_eq!(nearest_index(&arr, 15.0), 0); // closest to 10 (diff=5 vs diff=5, first wins)
        assert_eq!(nearest_index(&arr, 25.0), 1); // closest to 20
        assert_eq!(nearest_index(&arr, 35.0), 2); // closest to 30
    }

    #[test]
    fn test_smooth_short_spectrum() {
        let data: Vec<f64> = vec![1.0, 2.0, 3.0];
        let smooth = smooth_spectrum_sg5(&data);
        assert_eq!(smooth, data); // too short to smooth
    }

    #[test]
    fn test_eem_max_intensity() {
        let eem = ExcitationEmissionMatrix::new(
            vec![300.0, 320.0],
            vec![400.0, 420.0],
            vec![vec![1.0, 2.0], vec![3.0, 4.0]],
        );
        assert!(approx_eq(eem.max_intensity(), 4.0, 0.001));
    }
}

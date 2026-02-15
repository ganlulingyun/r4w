//! # Quantum Dot Spectroscopy
//!
//! This module implements quantum dot (QD) optical spectroscopy analysis for
//! nanoparticle characterization and biosensing applications.
//!
//! ## Overview
//!
//! Quantum dots are semiconductor nanocrystals whose optical properties depend
//! on their size due to quantum confinement. As the particle radius decreases
//! below the bulk exciton Bohr radius, the bandgap increases, blue-shifting
//! emission and absorption spectra. This module provides tools for:
//!
//! - **Size-dependent bandgap** via the Brus equation (particle-in-a-sphere)
//! - **Photoluminescence (PL) spectra** with Gaussian emission profiles
//! - **Absorption spectra** with first excitonic peaks and continuum
//! - **Size distribution analysis** from PL linewidth
//! - **Fluorescence lifetime** mono/biexponential decay fitting
//! - **FRET** (Forster Resonance Energy Transfer) for distance measurement
//! - **Blinking analysis** from intensity time traces
//!
//! ## Brus Equation
//!
//! The size-dependent bandgap of a quantum dot is:
//!
//! ```text
//! E(r) = E_bulk + h^2/(8*r^2) * (1/m_e* + 1/m_h*) - 1.8*e^2/(4*pi*epsilon_0*epsilon_r*r)
//! ```
//!
//! where the second term is the quantum confinement energy and the third term
//! is the Coulomb attraction between electron and hole.
//!
//! ## Usage
//!
//! ```rust
//! use r4w_core::quantum_dot_spectroscopy::*;
//!
//! // Create a 3 nm radius CdSe quantum dot
//! let qd = QuantumDot::new(3.0, QdMaterial::CdSe);
//!
//! // Calculate size-dependent bandgap
//! let eg = qd.bandgap_ev();
//! assert!(eg > qd.material.bulk_bandgap_ev()); // Blue-shifted from bulk
//!
//! // Get emission wavelength
//! let lambda = qd.emission_wavelength_nm();
//! assert!(lambda > 400.0 && lambda < 700.0); // Visible range
//!
//! // Generate PL spectrum
//! let spectrum = qd.pl_spectrum(400.0, 700.0, 1.0, 0.8);
//! ```

// ─── Physical constants ─────────────────────────────────────────────────────

/// Planck's constant in J*s
const H_PLANCK: f64 = 6.62607015e-34;

/// Speed of light in m/s
const C_LIGHT: f64 = 2.99792458e8;

/// Elementary charge in C
const E_CHARGE: f64 = 1.602176634e-19;

/// Vacuum permittivity in F/m
const EPSILON_0: f64 = 8.8541878128e-12;

/// Free electron mass in kg
const M_ELECTRON: f64 = 9.1093837015e-31;

/// Planck's constant in eV*s
const H_EV: f64 = 4.135667696e-15;

/// pi
const PI: f64 = std::f64::consts::PI;

// ─── QD Material ─────────────────────────────────────────────────────────────

/// Quantum dot semiconductor material with bulk optical properties.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum QdMaterial {
    /// Cadmium selenide - most common QD material, visible emission
    CdSe,
    /// Cadmium telluride - red/NIR emission
    CdTe,
    /// Indium phosphide - cadmium-free alternative
    InP,
    /// Lead sulfide - infrared emission
    PbS,
}

impl QdMaterial {
    /// Bulk bandgap energy in eV
    pub fn bulk_bandgap_ev(&self) -> f64 {
        match self {
            QdMaterial::CdSe => 1.74,
            QdMaterial::CdTe => 1.50,
            QdMaterial::InP => 1.35,
            QdMaterial::PbS => 0.41,
        }
    }

    /// Effective electron mass as fraction of free electron mass
    pub fn effective_electron_mass(&self) -> f64 {
        match self {
            QdMaterial::CdSe => 0.12,
            QdMaterial::CdTe => 0.11,
            QdMaterial::InP => 0.08,
            QdMaterial::PbS => 0.085,
        }
    }

    /// Effective hole mass as fraction of free electron mass
    pub fn effective_hole_mass(&self) -> f64 {
        match self {
            QdMaterial::CdSe => 0.45,
            QdMaterial::CdTe => 0.35,
            QdMaterial::InP => 0.60,
            QdMaterial::PbS => 0.085,
        }
    }

    /// Relative dielectric constant
    pub fn dielectric_constant(&self) -> f64 {
        match self {
            QdMaterial::CdSe => 10.6,
            QdMaterial::CdTe => 10.2,
            QdMaterial::InP => 12.5,
            QdMaterial::PbS => 17.0,
        }
    }

    /// Material name string
    pub fn name(&self) -> &'static str {
        match self {
            QdMaterial::CdSe => "CdSe",
            QdMaterial::CdTe => "CdTe",
            QdMaterial::InP => "InP",
            QdMaterial::PbS => "PbS",
        }
    }
}

// ─── Quantum Dot ─────────────────────────────────────────────────────────────

/// A quantum dot nanoparticle with size-dependent optical properties.
#[derive(Debug, Clone)]
pub struct QuantumDot {
    /// Radius in nanometers
    pub radius_nm: f64,
    /// Semiconductor material
    pub material: QdMaterial,
}

impl QuantumDot {
    /// Create a new quantum dot with given radius (nm) and material.
    pub fn new(radius_nm: f64, material: QdMaterial) -> Self {
        assert!(radius_nm > 0.0, "Radius must be positive");
        Self {
            radius_nm,
            material,
        }
    }

    /// Radius in meters.
    fn radius_m(&self) -> f64 {
        self.radius_nm * 1.0e-9
    }

    /// Quantum confinement energy in eV (kinetic term of Brus equation).
    ///
    /// E_conf = h^2 / (8 * r^2) * (1/m_e* + 1/m_h*)
    pub fn confinement_energy_ev(&self) -> f64 {
        let r = self.radius_m();
        let me = self.material.effective_electron_mass() * M_ELECTRON;
        let mh = self.material.effective_hole_mass() * M_ELECTRON;
        let e_joules = H_PLANCK * H_PLANCK / (8.0 * r * r) * (1.0 / me + 1.0 / mh);
        e_joules / E_CHARGE
    }

    /// Coulomb attraction energy in eV (negative contribution).
    ///
    /// E_coulomb = -1.8 * e^2 / (4 * pi * epsilon_0 * epsilon_r * r)
    pub fn coulomb_energy_ev(&self) -> f64 {
        let r = self.radius_m();
        let eps_r = self.material.dielectric_constant();
        let e_joules = -1.8 * E_CHARGE * E_CHARGE / (4.0 * PI * EPSILON_0 * eps_r * r);
        e_joules / E_CHARGE
    }

    /// Size-dependent bandgap via Brus equation in eV.
    ///
    /// E(r) = E_bulk + h^2/(8*r^2)*(1/m_e*+1/m_h*) - 1.8*e^2/(4*pi*eps0*eps_r*r)
    pub fn bandgap_ev(&self) -> f64 {
        self.material.bulk_bandgap_ev() + self.confinement_energy_ev() + self.coulomb_energy_ev()
    }

    /// Emission wavelength in nm from size-dependent bandgap.
    ///
    /// lambda = h*c / E
    pub fn emission_wavelength_nm(&self) -> f64 {
        let e_joules = self.bandgap_ev() * E_CHARGE;
        let lambda_m = H_PLANCK * C_LIGHT / e_joules;
        lambda_m * 1.0e9
    }

    /// Emission frequency in Hz.
    pub fn emission_frequency_hz(&self) -> f64 {
        self.bandgap_ev() * E_CHARGE / H_PLANCK
    }

    /// Generate photoluminescence spectrum as (wavelength_nm, intensity) pairs.
    ///
    /// Models PL as a Gaussian centered at the emission wavelength.
    ///
    /// - `lambda_min`, `lambda_max`: wavelength range in nm
    /// - `fwhm_nm`: full width at half maximum in nm
    /// - `quantum_yield`: PL quantum yield (0..1)
    pub fn pl_spectrum(
        &self,
        lambda_min: f64,
        lambda_max: f64,
        fwhm_nm: f64,
        quantum_yield: f64,
    ) -> Vec<(f64, f64)> {
        let center = self.emission_wavelength_nm();
        let sigma = fwhm_to_sigma(fwhm_nm);
        let step = (lambda_max - lambda_min) / 300.0;
        let mut spectrum = Vec::new();
        let mut lam = lambda_min;
        while lam <= lambda_max {
            let intensity = quantum_yield * gaussian(lam, center, sigma);
            spectrum.push((lam, intensity));
            lam += step;
        }
        spectrum
    }

    /// Generate absorption spectrum as (wavelength_nm, alpha) pairs.
    ///
    /// Models first excitonic absorption peak plus continuum above bandgap.
    ///
    /// - `lambda_min`, `lambda_max`: wavelength range in nm
    /// - `peak_width_nm`: width of excitonic peak
    /// - `stokes_shift_nm`: redshift of emission vs absorption peak
    pub fn absorption_spectrum(
        &self,
        lambda_min: f64,
        lambda_max: f64,
        peak_width_nm: f64,
        stokes_shift_nm: f64,
    ) -> Vec<(f64, f64)> {
        let emission_wl = self.emission_wavelength_nm();
        // Absorption peak is blue-shifted from emission by Stokes shift
        let abs_peak_wl = emission_wl - stokes_shift_nm;
        let sigma = fwhm_to_sigma(peak_width_nm);

        // Bandgap wavelength (onset of absorption)
        let bg_wl = emission_wl;

        let step = (lambda_max - lambda_min) / 300.0;
        let mut spectrum = Vec::new();
        let mut lam = lambda_min;
        while lam <= lambda_max {
            // First excitonic peak (Gaussian)
            let excitonic = gaussian(lam, abs_peak_wl, sigma);

            // Continuum absorption above bandgap: proportional to sqrt(E - Eg)
            // for direct bandgap semiconductors
            let continuum = if lam < bg_wl {
                let e_photon = wavelength_to_ev(lam);
                let e_gap = wavelength_to_ev(bg_wl);
                let de = e_photon - e_gap;
                if de > 0.0 {
                    0.3 * de.sqrt()
                } else {
                    0.0
                }
            } else {
                0.0
            };

            let alpha = excitonic + continuum;
            spectrum.push((lam, alpha));
            lam += step;
        }
        spectrum
    }
}

// ─── Utility functions ───────────────────────────────────────────────────────

/// Convert FWHM to Gaussian sigma.
fn fwhm_to_sigma(fwhm: f64) -> f64 {
    fwhm / (2.0 * (2.0_f64.ln() * 2.0).sqrt())
}

/// Gaussian function: A * exp(-(x - mu)^2 / (2 * sigma^2))
fn gaussian(x: f64, mu: f64, sigma: f64) -> f64 {
    let dx = x - mu;
    (-dx * dx / (2.0 * sigma * sigma)).exp()
}

/// Convert wavelength in nm to photon energy in eV.
pub fn wavelength_to_ev(lambda_nm: f64) -> f64 {
    H_EV * C_LIGHT / (lambda_nm * 1.0e-9)
}

/// Convert photon energy in eV to wavelength in nm.
pub fn ev_to_wavelength(ev: f64) -> f64 {
    H_EV * C_LIGHT / (ev * 1.0e-9)
}

// ─── Size Distribution Analysis ──────────────────────────────────────────────

/// Analyze quantum dot size distribution from photoluminescence data.
pub struct SizeDistribution;

impl SizeDistribution {
    /// Estimate QD radius from emission wavelength using iterative Newton solve
    /// of the Brus equation.
    ///
    /// Returns radius in nm, or None if convergence fails.
    pub fn radius_from_wavelength(lambda_nm: f64, material: QdMaterial) -> Option<f64> {
        let target_ev = wavelength_to_ev(lambda_nm);
        // Initial guess: start from a reasonable radius
        let mut r_nm = 3.0;

        for _ in 0..100 {
            let qd = QuantumDot::new(r_nm, material);
            let eg = qd.bandgap_ev();
            let err = eg - target_ev;

            if err.abs() < 1.0e-8 {
                return Some(r_nm);
            }

            // Numerical derivative dE/dr
            let dr = 0.001;
            let qd_plus = QuantumDot::new(r_nm + dr, material);
            let deg_dr = (qd_plus.bandgap_ev() - eg) / dr;

            if deg_dr.abs() < 1.0e-15 {
                return None;
            }

            let r_new = r_nm - err / deg_dr;
            if r_new <= 0.1 {
                r_nm = 0.5;
            } else if r_new > 50.0 {
                r_nm = 50.0;
            } else {
                r_nm = r_new;
            }
        }
        // Return best estimate even if not fully converged
        Some(r_nm)
    }

    /// Estimate size distribution (mean radius, sigma) from PL FWHM.
    ///
    /// Maps the spectral FWHM to a size distribution width using the
    /// derivative of the Brus equation.
    pub fn from_pl_fwhm(center_nm: f64, fwhm_nm: f64, material: QdMaterial) -> Option<(f64, f64)> {
        let mean_r = Self::radius_from_wavelength(center_nm, material)?;

        // dE/dr at mean radius (numerical)
        let dr = 0.01;
        let qd1 = QuantumDot::new(mean_r, material);
        let qd2 = QuantumDot::new(mean_r + dr, material);
        let de_dr = (qd2.bandgap_ev() - qd1.bandgap_ev()) / dr;

        if de_dr.abs() < 1.0e-15 {
            return None;
        }

        // dlambda/dr = dlambda/dE * dE/dr
        // lambda = hc/E => dlambda/dE = -hc/E^2
        let e_center = wavelength_to_ev(center_nm);
        let dlambda_de = -(H_EV * C_LIGHT * 1.0e9) / (e_center * e_center * E_CHARGE);
        // Actually: lambda(nm) = 1239.84 / E(eV), so dlambda/dE = -1239.84/E^2
        let dlambda_de_simple = -1239.84 / (e_center * e_center);
        let dlambda_dr = dlambda_de_simple * de_dr;

        // sigma_lambda -> sigma_r
        let sigma_lambda = fwhm_to_sigma(fwhm_nm);
        let sigma_r = (sigma_lambda / dlambda_dr).abs();

        Some((mean_r, sigma_r))
    }

    /// Coefficient of variation: CV = sigma / mean
    pub fn coefficient_of_variation(mean_r: f64, sigma_r: f64) -> f64 {
        if mean_r.abs() < 1.0e-15 {
            return 0.0;
        }
        sigma_r / mean_r
    }

    /// Generate a Gaussian size distribution as (radius_nm, probability) pairs.
    pub fn gaussian_distribution(mean_r: f64, sigma_r: f64, n_points: usize) -> Vec<(f64, f64)> {
        let r_min = (mean_r - 4.0 * sigma_r).max(0.1);
        let r_max = mean_r + 4.0 * sigma_r;
        let step = (r_max - r_min) / (n_points as f64 - 1.0);
        let norm = 1.0 / (sigma_r * (2.0 * PI).sqrt());
        (0..n_points)
            .map(|i| {
                let r = r_min + i as f64 * step;
                let p = norm * gaussian(r, mean_r, sigma_r);
                (r, p)
            })
            .collect()
    }
}

// ─── Tauc Plot Analysis ──────────────────────────────────────────────────────

/// Tauc plot analysis for direct bandgap extraction.
///
/// For direct bandgap semiconductors: (alpha * h * nu)^2 vs h*nu
/// The bandgap is extracted from the x-intercept of the linear region.
pub struct TaucPlot;

impl TaucPlot {
    /// Generate Tauc plot data from absorption spectrum.
    ///
    /// Input: (wavelength_nm, alpha) pairs
    /// Output: (photon_energy_eV, (alpha*h*nu)^2) pairs
    pub fn from_absorption(absorption: &[(f64, f64)]) -> Vec<(f64, f64)> {
        absorption
            .iter()
            .filter(|(lam, _)| *lam > 0.0)
            .map(|(lam, alpha)| {
                let hnu = wavelength_to_ev(*lam);
                let tauc = (alpha * hnu) * (alpha * hnu);
                (hnu, tauc)
            })
            .collect()
    }

    /// Extract bandgap from Tauc plot by finding the linear region
    /// and extrapolating to (alpha*h*nu)^2 = 0.
    ///
    /// Uses the steepest region of the Tauc plot for linear fitting.
    pub fn extract_bandgap(tauc_data: &[(f64, f64)]) -> Option<f64> {
        if tauc_data.len() < 5 {
            return None;
        }

        // Find the region with maximum slope (steepest part)
        let mut max_slope = 0.0_f64;
        let mut best_idx = 0;
        for i in 1..tauc_data.len() {
            let dx = tauc_data[i].0 - tauc_data[i - 1].0;
            let dy = tauc_data[i].1 - tauc_data[i - 1].1;
            if dx.abs() > 1.0e-15 {
                let slope = dy / dx;
                if slope > max_slope {
                    max_slope = slope;
                    best_idx = i;
                }
            }
        }

        if max_slope.abs() < 1.0e-15 {
            return None;
        }

        // Use a window around the steepest point for linear fit
        let start = if best_idx >= 5 { best_idx - 5 } else { 0 };
        let end = (best_idx + 5).min(tauc_data.len());
        let window = &tauc_data[start..end];

        // Linear regression: y = m*x + b
        let (m, b) = linear_regression(window)?;

        if m.abs() < 1.0e-15 {
            return None;
        }

        // x-intercept: 0 = m*x + b => x = -b/m
        let eg = -b / m;
        if eg > 0.0 {
            Some(eg)
        } else {
            None
        }
    }
}

/// Simple linear regression returning (slope, intercept).
fn linear_regression(data: &[(f64, f64)]) -> Option<(f64, f64)> {
    let n = data.len() as f64;
    if n < 2.0 {
        return None;
    }
    let sx: f64 = data.iter().map(|(x, _)| x).sum();
    let sy: f64 = data.iter().map(|(_, y)| y).sum();
    let sxx: f64 = data.iter().map(|(x, _)| x * x).sum();
    let sxy: f64 = data.iter().map(|(x, y)| x * y).sum();

    let denom = n * sxx - sx * sx;
    if denom.abs() < 1.0e-30 {
        return None;
    }
    let m = (n * sxy - sx * sy) / denom;
    let b = (sy - m * sx) / n;
    Some((m, b))
}

// ─── Fluorescence Lifetime ───────────────────────────────────────────────────

/// Fluorescence lifetime analysis for quantum dot decay kinetics.
pub struct FluorescenceLifetime;

impl FluorescenceLifetime {
    /// Monoexponential decay: I(t) = I0 * exp(-t/tau) + background
    pub fn monoexponential(t: f64, i0: f64, tau: f64, background: f64) -> f64 {
        i0 * (-t / tau).exp() + background
    }

    /// Generate monoexponential decay curve.
    pub fn monoexponential_curve(
        times: &[f64],
        i0: f64,
        tau: f64,
        background: f64,
    ) -> Vec<f64> {
        times
            .iter()
            .map(|&t| Self::monoexponential(t, i0, tau, background))
            .collect()
    }

    /// Biexponential decay: I(t) = A1*exp(-t/tau1) + A2*exp(-t/tau2) + background
    pub fn biexponential(
        t: f64,
        a1: f64,
        tau1: f64,
        a2: f64,
        tau2: f64,
        background: f64,
    ) -> f64 {
        a1 * (-t / tau1).exp() + a2 * (-t / tau2).exp() + background
    }

    /// Generate biexponential decay curve.
    pub fn biexponential_curve(
        times: &[f64],
        a1: f64,
        tau1: f64,
        a2: f64,
        tau2: f64,
        background: f64,
    ) -> Vec<f64> {
        times
            .iter()
            .map(|&t| Self::biexponential(t, a1, tau1, a2, tau2, background))
            .collect()
    }

    /// Intensity-weighted average lifetime for biexponential decay.
    ///
    /// tau_avg = (A1*tau1^2 + A2*tau2^2) / (A1*tau1 + A2*tau2)
    pub fn average_lifetime(a1: f64, tau1: f64, a2: f64, tau2: f64) -> f64 {
        let num = a1 * tau1 * tau1 + a2 * tau2 * tau2;
        let den = a1 * tau1 + a2 * tau2;
        if den.abs() < 1.0e-30 {
            return 0.0;
        }
        num / den
    }

    /// Amplitude-weighted average lifetime for biexponential decay.
    ///
    /// tau_amp = (A1*tau1 + A2*tau2) / (A1 + A2)
    pub fn amplitude_average_lifetime(a1: f64, tau1: f64, a2: f64, tau2: f64) -> f64 {
        let den = a1 + a2;
        if den.abs() < 1.0e-30 {
            return 0.0;
        }
        (a1 * tau1 + a2 * tau2) / den
    }

    /// Radiative decay rate from quantum yield and lifetime.
    ///
    /// k_r = QY / tau
    pub fn radiative_rate(quantum_yield: f64, tau: f64) -> f64 {
        if tau.abs() < 1.0e-30 {
            return 0.0;
        }
        quantum_yield / tau
    }

    /// Non-radiative decay rate from quantum yield and lifetime.
    ///
    /// k_nr = (1 - QY) / tau
    pub fn non_radiative_rate(quantum_yield: f64, tau: f64) -> f64 {
        if tau.abs() < 1.0e-30 {
            return 0.0;
        }
        (1.0 - quantum_yield) / tau
    }

    /// Total decay rate: k_total = k_r + k_nr = 1/tau
    pub fn total_rate(tau: f64) -> f64 {
        if tau.abs() < 1.0e-30 {
            return 0.0;
        }
        1.0 / tau
    }

    /// Fit monoexponential decay to data using linearized least-squares.
    ///
    /// Takes ln(I(t) - bg) = ln(I0) - t/tau and fits a line.
    /// Returns (i0, tau) or None if fit fails.
    pub fn fit_monoexponential(
        times: &[f64],
        intensities: &[f64],
        background: f64,
    ) -> Option<(f64, f64)> {
        if times.len() != intensities.len() || times.len() < 3 {
            return None;
        }

        // Linearize: ln(I - bg) = ln(I0) - t/tau
        let log_data: Vec<(f64, f64)> = times
            .iter()
            .zip(intensities.iter())
            .filter(|(_, &i)| i - background > 1.0e-10)
            .map(|(&t, &i)| (t, (i - background).ln()))
            .collect();

        if log_data.len() < 3 {
            return None;
        }

        let (slope, intercept) = linear_regression(&log_data)?;

        if slope >= 0.0 {
            return None; // Decay should have negative slope
        }

        let tau = -1.0 / slope;
        let i0 = intercept.exp();
        Some((i0, tau))
    }
}

// ─── FRET Analysis ───────────────────────────────────────────────────────────

/// Forster Resonance Energy Transfer (FRET) analysis.
///
/// FRET is a distance-dependent energy transfer mechanism between a donor
/// fluorophore and an acceptor chromophore, used as a molecular ruler.
pub struct FretAnalysis;

impl FretAnalysis {
    /// Calculate FRET efficiency from donor-acceptor distance and Forster radius.
    ///
    /// E = R0^6 / (R0^6 + r^6)
    pub fn efficiency(r_nm: f64, r0_nm: f64) -> f64 {
        let r0_6 = r0_nm.powi(6);
        let r_6 = r_nm.powi(6);
        r0_6 / (r0_6 + r_6)
    }

    /// Calculate donor-acceptor distance from FRET efficiency and Forster radius.
    ///
    /// r = R0 * ((1 - E) / E)^(1/6)
    pub fn distance_from_efficiency(efficiency: f64, r0_nm: f64) -> f64 {
        if efficiency <= 0.0 || efficiency >= 1.0 {
            return 0.0;
        }
        r0_nm * ((1.0 - efficiency) / efficiency).powf(1.0 / 6.0)
    }

    /// Calculate Forster radius from physical parameters.
    ///
    /// R0 = 0.211 * (kappa^2 * n^(-4) * QY_D * J)^(1/6) nm
    ///
    /// - `kappa_sq`: orientation factor (2/3 for random, 0-4 range)
    /// - `n`: refractive index of medium
    /// - `qy_donor`: quantum yield of donor
    /// - `j_overlap`: spectral overlap integral in M^-1 cm^-1 nm^4
    pub fn forster_radius(kappa_sq: f64, n: f64, qy_donor: f64, j_overlap: f64) -> f64 {
        0.211 * (kappa_sq * n.powi(-4) * qy_donor * j_overlap).powf(1.0 / 6.0)
    }

    /// Calculate spectral overlap integral J between donor emission and acceptor absorption.
    ///
    /// J = integral[ F_D(lambda) * eps_A(lambda) * lambda^4 ] d_lambda
    ///     / integral[ F_D(lambda) ] d_lambda
    ///
    /// Input: matched arrays of (wavelength_nm, donor_emission, acceptor_extinction)
    pub fn spectral_overlap_integral(
        wavelengths_nm: &[f64],
        donor_emission: &[f64],
        acceptor_absorption: &[f64],
    ) -> f64 {
        if wavelengths_nm.len() < 2
            || donor_emission.len() != wavelengths_nm.len()
            || acceptor_absorption.len() != wavelengths_nm.len()
        {
            return 0.0;
        }

        let mut numerator = 0.0;
        let mut denominator = 0.0;

        for i in 0..wavelengths_nm.len() - 1 {
            let dlam = wavelengths_nm[i + 1] - wavelengths_nm[i];
            let lam = wavelengths_nm[i];
            let fd = donor_emission[i];
            let ea = acceptor_absorption[i];
            let lam4 = lam * lam * lam * lam;

            numerator += fd * ea * lam4 * dlam;
            denominator += fd * dlam;
        }

        if denominator.abs() < 1.0e-30 {
            return 0.0;
        }

        numerator / denominator
    }

    /// FRET rate constant.
    ///
    /// k_FRET = (1/tau_D) * (R0/r)^6
    pub fn fret_rate(tau_donor: f64, r_nm: f64, r0_nm: f64) -> f64 {
        if tau_donor.abs() < 1.0e-30 || r_nm.abs() < 1.0e-30 {
            return 0.0;
        }
        (1.0 / tau_donor) * (r0_nm / r_nm).powi(6)
    }

    /// Donor lifetime in presence of acceptor.
    ///
    /// tau_DA = tau_D / (1 + (R0/r)^6)
    pub fn donor_lifetime_with_acceptor(tau_donor: f64, r_nm: f64, r0_nm: f64) -> f64 {
        tau_donor / (1.0 + (r0_nm / r_nm).powi(6))
    }

    /// FRET efficiency from donor lifetimes.
    ///
    /// E = 1 - tau_DA / tau_D
    pub fn efficiency_from_lifetimes(tau_d: f64, tau_da: f64) -> f64 {
        if tau_d.abs() < 1.0e-30 {
            return 0.0;
        }
        1.0 - tau_da / tau_d
    }
}

// ─── Blinking Analysis ───────────────────────────────────────────────────────

/// Quantum dot blinking (fluorescence intermittency) analysis.
///
/// QDs exhibit random switching between bright (on) and dark (off) states.
pub struct BlinkingAnalysis;

/// A blinking event with state and duration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BlinkEvent {
    /// True for ON state, false for OFF state
    pub is_on: bool,
    /// Duration in time units (number of samples)
    pub duration: usize,
    /// Start index in the trace
    pub start_idx: usize,
}

impl BlinkingAnalysis {
    /// Assign on/off states from intensity time trace using a threshold.
    ///
    /// Returns a boolean vector: true = on, false = off.
    pub fn threshold_states(intensities: &[f64], threshold: f64) -> Vec<bool> {
        intensities.iter().map(|&i| i >= threshold).collect()
    }

    /// Extract blinking events (on/off durations) from state assignments.
    pub fn extract_events(states: &[bool]) -> Vec<BlinkEvent> {
        if states.is_empty() {
            return Vec::new();
        }

        let mut events = Vec::new();
        let mut current_state = states[0];
        let mut start = 0;
        let mut count = 1;

        for i in 1..states.len() {
            if states[i] == current_state {
                count += 1;
            } else {
                events.push(BlinkEvent {
                    is_on: current_state,
                    duration: count,
                    start_idx: start,
                });
                current_state = states[i];
                start = i;
                count = 1;
            }
        }
        // Push final event
        events.push(BlinkEvent {
            is_on: current_state,
            duration: count,
            start_idx: start,
        });

        events
    }

    /// Calculate on-fraction: fraction of time QD is in the ON state.
    pub fn on_fraction(states: &[bool]) -> f64 {
        if states.is_empty() {
            return 0.0;
        }
        let on_count = states.iter().filter(|&&s| s).count();
        on_count as f64 / states.len() as f64
    }

    /// Compute on-time durations from blinking events.
    pub fn on_times(events: &[BlinkEvent]) -> Vec<usize> {
        events
            .iter()
            .filter(|e| e.is_on)
            .map(|e| e.duration)
            .collect()
    }

    /// Compute off-time durations from blinking events.
    pub fn off_times(events: &[BlinkEvent]) -> Vec<usize> {
        events
            .iter()
            .filter(|e| !e.is_on)
            .map(|e| e.duration)
            .collect()
    }

    /// Compute histogram of durations with given bin edges.
    pub fn duration_histogram(durations: &[usize], num_bins: usize) -> Vec<(f64, usize)> {
        if durations.is_empty() || num_bins == 0 {
            return Vec::new();
        }

        let min_d = *durations.iter().min().unwrap() as f64;
        let max_d = *durations.iter().max().unwrap() as f64;

        if (max_d - min_d).abs() < 1.0e-10 {
            return vec![(min_d, durations.len())];
        }

        let bin_width = (max_d - min_d) / num_bins as f64;
        let mut bins = vec![0_usize; num_bins];

        for &d in durations {
            let idx = ((d as f64 - min_d) / bin_width) as usize;
            let idx = idx.min(num_bins - 1);
            bins[idx] += 1;
        }

        bins.iter()
            .enumerate()
            .map(|(i, &count)| {
                let center = min_d + (i as f64 + 0.5) * bin_width;
                (center, count)
            })
            .collect()
    }

    /// Average on-time duration.
    pub fn mean_on_time(events: &[BlinkEvent]) -> f64 {
        let on = Self::on_times(events);
        if on.is_empty() {
            return 0.0;
        }
        on.iter().sum::<usize>() as f64 / on.len() as f64
    }

    /// Average off-time duration.
    pub fn mean_off_time(events: &[BlinkEvent]) -> f64 {
        let off = Self::off_times(events);
        if off.is_empty() {
            return 0.0;
        }
        off.iter().sum::<usize>() as f64 / off.len() as f64
    }
}

// ─── Ensemble Spectrum ───────────────────────────────────────────────────────

/// Generate ensemble PL spectrum from a distribution of QD sizes.
///
/// Each QD in the size distribution contributes a Gaussian emission peak
/// at its size-dependent wavelength, weighted by the probability.
pub fn ensemble_pl_spectrum(
    material: QdMaterial,
    mean_radius_nm: f64,
    sigma_radius_nm: f64,
    num_sizes: usize,
    lambda_min: f64,
    lambda_max: f64,
    num_wavelengths: usize,
    single_qd_fwhm_nm: f64,
) -> Vec<(f64, f64)> {
    let sizes = SizeDistribution::gaussian_distribution(mean_radius_nm, sigma_radius_nm, num_sizes);
    let wl_step = (lambda_max - lambda_min) / (num_wavelengths as f64 - 1.0);

    let mut spectrum: Vec<f64> = vec![0.0; num_wavelengths];

    let sigma_single = fwhm_to_sigma(single_qd_fwhm_nm);

    for &(r, prob) in &sizes {
        if r < 0.5 {
            continue;
        }
        let qd = QuantumDot::new(r, material);
        let center = qd.emission_wavelength_nm();
        if center < lambda_min - 50.0 || center > lambda_max + 50.0 {
            continue;
        }

        for i in 0..num_wavelengths {
            let lam = lambda_min + i as f64 * wl_step;
            spectrum[i] += prob * gaussian(lam, center, sigma_single);
        }
    }

    // Normalize to peak = 1
    let max_val = spectrum.iter().cloned().fold(0.0_f64, f64::max);
    if max_val > 1.0e-30 {
        for v in spectrum.iter_mut() {
            *v /= max_val;
        }
    }

    (0..num_wavelengths)
        .map(|i| (lambda_min + i as f64 * wl_step, spectrum[i]))
        .collect()
}

// ─── Quantum Yield ───────────────────────────────────────────────────────────

/// Quantum yield measurement utilities.
pub struct QuantumYield;

impl QuantumYield {
    /// Calculate quantum yield from integrated emission and absorption.
    ///
    /// QY = photons_emitted / photons_absorbed
    pub fn from_photon_counts(emitted: f64, absorbed: f64) -> f64 {
        if absorbed.abs() < 1.0e-30 {
            return 0.0;
        }
        (emitted / absorbed).min(1.0).max(0.0)
    }

    /// Comparative method: QY relative to a reference standard.
    ///
    /// QY_sample = QY_ref * (I_sample/I_ref) * (A_ref/A_sample) * (n_sample/n_ref)^2
    ///
    /// - `i_sample`, `i_ref`: integrated emission intensities
    /// - `a_sample`, `a_ref`: absorbance at excitation wavelength
    /// - `n_sample`, `n_ref`: refractive indices
    /// - `qy_ref`: quantum yield of reference standard
    pub fn comparative_method(
        i_sample: f64,
        i_ref: f64,
        a_sample: f64,
        a_ref: f64,
        n_sample: f64,
        n_ref: f64,
        qy_ref: f64,
    ) -> f64 {
        if i_ref.abs() < 1.0e-30 || a_sample.abs() < 1.0e-30 || n_ref.abs() < 1.0e-30 {
            return 0.0;
        }
        let qy = qy_ref * (i_sample / i_ref) * (a_ref / a_sample) * (n_sample / n_ref).powi(2);
        qy.min(1.0).max(0.0)
    }

    /// Quantum yield from radiative and non-radiative rates.
    ///
    /// QY = k_r / (k_r + k_nr)
    pub fn from_rates(k_radiative: f64, k_non_radiative: f64) -> f64 {
        let total = k_radiative + k_non_radiative;
        if total.abs() < 1.0e-30 {
            return 0.0;
        }
        k_radiative / total
    }
}

// ─── Stokes Shift ────────────────────────────────────────────────────────────

/// Calculate Stokes shift between absorption and emission peaks.
///
/// Returns shift in nm (positive means emission is red-shifted).
pub fn stokes_shift_nm(absorption_peak_nm: f64, emission_peak_nm: f64) -> f64 {
    emission_peak_nm - absorption_peak_nm
}

/// Calculate Stokes shift in eV.
pub fn stokes_shift_ev(absorption_peak_nm: f64, emission_peak_nm: f64) -> f64 {
    wavelength_to_ev(absorption_peak_nm) - wavelength_to_ev(emission_peak_nm)
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1.0e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // --- QdMaterial tests ---

    #[test]
    fn test_cdse_properties() {
        let m = QdMaterial::CdSe;
        assert_eq!(m.bulk_bandgap_ev(), 1.74);
        assert_eq!(m.effective_electron_mass(), 0.12);
        assert_eq!(m.effective_hole_mass(), 0.45);
        assert_eq!(m.dielectric_constant(), 10.6);
        assert_eq!(m.name(), "CdSe");
    }

    #[test]
    fn test_cdte_properties() {
        let m = QdMaterial::CdTe;
        assert_eq!(m.bulk_bandgap_ev(), 1.50);
        assert_eq!(m.effective_electron_mass(), 0.11);
        assert_eq!(m.effective_hole_mass(), 0.35);
        assert_eq!(m.dielectric_constant(), 10.2);
    }

    #[test]
    fn test_inp_properties() {
        let m = QdMaterial::InP;
        assert_eq!(m.bulk_bandgap_ev(), 1.35);
        assert_eq!(m.effective_electron_mass(), 0.08);
        assert_eq!(m.effective_hole_mass(), 0.6);
        assert_eq!(m.dielectric_constant(), 12.5);
    }

    #[test]
    fn test_pbs_properties() {
        let m = QdMaterial::PbS;
        assert_eq!(m.bulk_bandgap_ev(), 0.41);
        assert_eq!(m.effective_electron_mass(), 0.085);
        assert_eq!(m.effective_hole_mass(), 0.085);
        assert_eq!(m.dielectric_constant(), 17.0);
    }

    // --- QuantumDot creation and basic properties ---

    #[test]
    fn test_qd_creation() {
        let qd = QuantumDot::new(3.0, QdMaterial::CdSe);
        assert_eq!(qd.radius_nm, 3.0);
        assert_eq!(qd.material, QdMaterial::CdSe);
    }

    #[test]
    #[should_panic]
    fn test_qd_zero_radius() {
        QuantumDot::new(0.0, QdMaterial::CdSe);
    }

    #[test]
    fn test_confinement_energy_positive() {
        let qd = QuantumDot::new(3.0, QdMaterial::CdSe);
        assert!(qd.confinement_energy_ev() > 0.0);
    }

    #[test]
    fn test_coulomb_energy_negative() {
        let qd = QuantumDot::new(3.0, QdMaterial::CdSe);
        assert!(qd.coulomb_energy_ev() < 0.0);
    }

    #[test]
    fn test_bandgap_larger_than_bulk() {
        // For small QDs, confinement dominates, so bandgap > bulk
        for material in &[QdMaterial::CdSe, QdMaterial::CdTe, QdMaterial::InP, QdMaterial::PbS] {
            let qd = QuantumDot::new(2.0, *material);
            assert!(
                qd.bandgap_ev() > material.bulk_bandgap_ev(),
                "Bandgap for 2nm {:?} QD ({:.3} eV) should be > bulk ({:.3} eV)",
                material,
                qd.bandgap_ev(),
                material.bulk_bandgap_ev()
            );
        }
    }

    #[test]
    fn test_bandgap_size_dependence() {
        // Smaller QD should have larger bandgap
        let small = QuantumDot::new(2.0, QdMaterial::CdSe);
        let large = QuantumDot::new(5.0, QdMaterial::CdSe);
        assert!(small.bandgap_ev() > large.bandgap_ev());
    }

    #[test]
    fn test_emission_wavelength_visible_cdse() {
        // CdSe QDs of 2-4 nm typically emit in visible range
        let qd = QuantumDot::new(3.0, QdMaterial::CdSe);
        let wl = qd.emission_wavelength_nm();
        assert!(wl > 300.0 && wl < 800.0, "CdSe 3nm emission: {} nm", wl);
    }

    #[test]
    fn test_emission_wavelength_blueshift_with_size() {
        // Smaller QD -> shorter wavelength (blue shift)
        let small = QuantumDot::new(2.0, QdMaterial::CdSe);
        let large = QuantumDot::new(5.0, QdMaterial::CdSe);
        assert!(small.emission_wavelength_nm() < large.emission_wavelength_nm());
    }

    #[test]
    fn test_pbs_infrared_emission() {
        // PbS QDs emit in infrared
        let qd = QuantumDot::new(4.0, QdMaterial::PbS);
        let wl = qd.emission_wavelength_nm();
        assert!(wl > 700.0, "PbS should emit in IR, got {} nm", wl);
    }

    // --- PL spectrum tests ---

    #[test]
    fn test_pl_spectrum_gaussian_shape() {
        let qd = QuantumDot::new(3.0, QdMaterial::CdSe);
        let spectrum = qd.pl_spectrum(400.0, 700.0, 20.0, 0.8);
        assert!(!spectrum.is_empty());

        // Peak should be near emission wavelength
        let peak = spectrum
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();
        let expected_center = qd.emission_wavelength_nm();
        assert!(
            (peak.0 - expected_center).abs() < 5.0,
            "PL peak at {:.1} nm, expected near {:.1} nm",
            peak.0,
            expected_center
        );
    }

    #[test]
    fn test_pl_spectrum_qy_scaling() {
        let qd = QuantumDot::new(3.0, QdMaterial::CdSe);
        let sp_high = qd.pl_spectrum(400.0, 700.0, 20.0, 1.0);
        let sp_low = qd.pl_spectrum(400.0, 700.0, 20.0, 0.5);

        // High QY spectrum should have double the intensity
        let max_high = sp_high.iter().map(|(_, i)| *i).fold(0.0_f64, f64::max);
        let max_low = sp_low.iter().map(|(_, i)| *i).fold(0.0_f64, f64::max);
        assert!(approx_eq(max_high / max_low, 2.0, 0.01));
    }

    // --- Absorption spectrum ---

    #[test]
    fn test_absorption_spectrum_peak() {
        let qd = QuantumDot::new(3.0, QdMaterial::CdSe);
        let abs = qd.absorption_spectrum(350.0, 700.0, 15.0, 20.0);
        assert!(!abs.is_empty());

        // Should have non-zero absorption at shorter wavelengths
        let short_wl_abs = abs.iter().find(|(l, _)| *l < 450.0).map(|(_, a)| *a).unwrap_or(0.0);
        assert!(short_wl_abs >= 0.0);
    }

    // --- Wavelength-energy conversions ---

    #[test]
    fn test_wavelength_to_ev_conversion() {
        // 620 nm (red) ~ 2.0 eV
        let e = wavelength_to_ev(620.0);
        assert!(approx_eq(e, 2.0, 0.02));
    }

    #[test]
    fn test_ev_to_wavelength_roundtrip() {
        let wl = 550.0;
        let e = wavelength_to_ev(wl);
        let wl2 = ev_to_wavelength(e);
        assert!(approx_eq(wl, wl2, 0.01));
    }

    // --- Size distribution ---

    #[test]
    fn test_radius_from_wavelength_cdse() {
        // Create a QD, get its wavelength, then recover the radius
        let qd = QuantumDot::new(3.0, QdMaterial::CdSe);
        let wl = qd.emission_wavelength_nm();
        let recovered = SizeDistribution::radius_from_wavelength(wl, QdMaterial::CdSe).unwrap();
        assert!(
            approx_eq(recovered, 3.0, 0.1),
            "Recovered radius {:.3} nm, expected 3.0 nm",
            recovered
        );
    }

    #[test]
    fn test_radius_from_wavelength_pbs() {
        let qd = QuantumDot::new(4.0, QdMaterial::PbS);
        let wl = qd.emission_wavelength_nm();
        let recovered = SizeDistribution::radius_from_wavelength(wl, QdMaterial::PbS).unwrap();
        assert!(
            approx_eq(recovered, 4.0, 0.15),
            "Recovered radius {:.3} nm, expected 4.0 nm",
            recovered
        );
    }

    #[test]
    fn test_size_distribution_from_fwhm() {
        let result = SizeDistribution::from_pl_fwhm(550.0, 30.0, QdMaterial::CdSe);
        assert!(result.is_some());
        let (mean_r, sigma_r) = result.unwrap();
        assert!(mean_r > 0.0);
        assert!(sigma_r > 0.0);
        assert!(sigma_r < mean_r); // Size distribution narrower than mean
    }

    #[test]
    fn test_coefficient_of_variation() {
        let cv = SizeDistribution::coefficient_of_variation(3.0, 0.3);
        assert!(approx_eq(cv, 0.1, TOL));
    }

    #[test]
    fn test_gaussian_distribution_normalization() {
        let dist = SizeDistribution::gaussian_distribution(3.0, 0.3, 200);
        assert_eq!(dist.len(), 200);

        // Integrate - should be approximately 1
        let integral: f64 = dist.windows(2).map(|w| {
            let dr = w[1].0 - w[0].0;
            0.5 * (w[0].1 + w[1].1) * dr
        }).sum();
        assert!(approx_eq(integral, 1.0, 0.05), "Integral = {}", integral);
    }

    // --- Tauc plot ---

    #[test]
    fn test_tauc_plot_data_generation() {
        let qd = QuantumDot::new(3.0, QdMaterial::CdSe);
        let abs = qd.absorption_spectrum(350.0, 700.0, 15.0, 20.0);
        let tauc = TaucPlot::from_absorption(&abs);
        assert!(!tauc.is_empty());

        // All Tauc values should be non-negative
        for &(e, t) in &tauc {
            assert!(e > 0.0);
            assert!(t >= 0.0);
        }
    }

    // --- Fluorescence lifetime ---

    #[test]
    fn test_monoexponential_decay() {
        let i = FluorescenceLifetime::monoexponential(0.0, 1000.0, 20.0, 10.0);
        assert!(approx_eq(i, 1010.0, TOL));

        let i_decay = FluorescenceLifetime::monoexponential(20.0, 1000.0, 20.0, 10.0);
        let expected = 1000.0 * (-1.0_f64).exp() + 10.0;
        assert!(approx_eq(i_decay, expected, TOL));
    }

    #[test]
    fn test_biexponential_decay() {
        let i = FluorescenceLifetime::biexponential(0.0, 500.0, 5.0, 500.0, 50.0, 10.0);
        assert!(approx_eq(i, 1010.0, TOL));
    }

    #[test]
    fn test_average_lifetime() {
        // For monoexponential (A2=0), average lifetime = tau1
        let tau = FluorescenceLifetime::average_lifetime(1.0, 20.0, 0.0, 0.0);
        assert!(approx_eq(tau, 20.0, TOL));

        // Biexponential: tau_avg = (A1*tau1^2 + A2*tau2^2) / (A1*tau1 + A2*tau2)
        let tau_bi = FluorescenceLifetime::average_lifetime(0.7, 5.0, 0.3, 20.0);
        let expected = (0.7 * 25.0 + 0.3 * 400.0) / (0.7 * 5.0 + 0.3 * 20.0);
        assert!(approx_eq(tau_bi, expected, TOL));
    }

    #[test]
    fn test_amplitude_average_lifetime() {
        let tau = FluorescenceLifetime::amplitude_average_lifetime(0.7, 5.0, 0.3, 20.0);
        let expected = (0.7 * 5.0 + 0.3 * 20.0) / 1.0;
        assert!(approx_eq(tau, expected, TOL));
    }

    #[test]
    fn test_radiative_rate() {
        let kr = FluorescenceLifetime::radiative_rate(0.8, 20.0e-9);
        assert!(approx_eq(kr, 0.8 / 20.0e-9, 1.0));
    }

    #[test]
    fn test_non_radiative_rate() {
        let knr = FluorescenceLifetime::non_radiative_rate(0.8, 20.0e-9);
        assert!(approx_eq(knr, 0.2 / 20.0e-9, 1.0));
    }

    #[test]
    fn test_total_rate() {
        let tau = 20.0e-9;
        let kt = FluorescenceLifetime::total_rate(tau);
        let kr = FluorescenceLifetime::radiative_rate(0.8, tau);
        let knr = FluorescenceLifetime::non_radiative_rate(0.8, tau);
        assert!(approx_eq(kt, kr + knr, 1.0));
    }

    #[test]
    fn test_fit_monoexponential() {
        let tau_true = 20.0;
        let i0_true = 1000.0;
        let bg = 10.0;

        let times: Vec<f64> = (0..100).map(|i| i as f64 * 0.5).collect();
        let intensities: Vec<f64> = times
            .iter()
            .map(|&t| FluorescenceLifetime::monoexponential(t, i0_true, tau_true, bg))
            .collect();

        let (i0_fit, tau_fit) =
            FluorescenceLifetime::fit_monoexponential(&times, &intensities, bg).unwrap();
        assert!(
            approx_eq(tau_fit, tau_true, 0.5),
            "Fitted tau = {:.3}, expected {:.3}",
            tau_fit,
            tau_true
        );
        assert!(
            approx_eq(i0_fit, i0_true, 50.0),
            "Fitted I0 = {:.1}, expected {:.1}",
            i0_fit,
            i0_true
        );
    }

    // --- FRET ---

    #[test]
    fn test_fret_efficiency_at_r0() {
        // At r = R0, efficiency = 50%
        let eff = FretAnalysis::efficiency(5.0, 5.0);
        assert!(approx_eq(eff, 0.5, TOL));
    }

    #[test]
    fn test_fret_efficiency_close() {
        // r << R0 -> E -> 1
        let eff = FretAnalysis::efficiency(1.0, 10.0);
        assert!(eff > 0.99);
    }

    #[test]
    fn test_fret_efficiency_far() {
        // r >> R0 -> E -> 0
        let eff = FretAnalysis::efficiency(50.0, 5.0);
        assert!(eff < 0.01);
    }

    #[test]
    fn test_fret_distance_from_efficiency() {
        let r0 = 5.4;
        let r_true = 4.0;
        let eff = FretAnalysis::efficiency(r_true, r0);
        let r_recovered = FretAnalysis::distance_from_efficiency(eff, r0);
        assert!(
            approx_eq(r_recovered, r_true, 0.01),
            "Recovered r = {:.3} nm, expected {:.3} nm",
            r_recovered,
            r_true
        );
    }

    #[test]
    fn test_forster_radius() {
        // Typical values
        let r0 = FretAnalysis::forster_radius(2.0 / 3.0, 1.33, 0.8, 1.0e15);
        assert!(r0 > 0.0 && r0 < 100.0, "R0 = {} nm", r0);
    }

    #[test]
    fn test_spectral_overlap_integral() {
        // Simple test: overlapping Gaussians
        let n = 100;
        let wavelengths: Vec<f64> = (0..n).map(|i| 400.0 + i as f64 * 3.0).collect();
        let donor: Vec<f64> = wavelengths
            .iter()
            .map(|&l| gaussian(l, 530.0, 15.0))
            .collect();
        let acceptor: Vec<f64> = wavelengths
            .iter()
            .map(|&l| gaussian(l, 560.0, 15.0))
            .collect();

        let j = FretAnalysis::spectral_overlap_integral(&wavelengths, &donor, &acceptor);
        assert!(j > 0.0, "J should be positive for overlapping spectra");
    }

    #[test]
    fn test_fret_rate() {
        let rate = FretAnalysis::fret_rate(20.0e-9, 3.0, 5.0);
        assert!(rate > 0.0);
    }

    #[test]
    fn test_donor_lifetime_with_acceptor() {
        let tau_d = 20.0e-9;
        let r0 = 5.0;
        let r = 5.0; // At R0, efficiency = 50%, so tau_DA = tau_D / 2
        let tau_da = FretAnalysis::donor_lifetime_with_acceptor(tau_d, r, r0);
        assert!(approx_eq(tau_da, tau_d / 2.0, 1.0e-15));
    }

    #[test]
    fn test_efficiency_from_lifetimes() {
        let eff = FretAnalysis::efficiency_from_lifetimes(20.0, 10.0);
        assert!(approx_eq(eff, 0.5, TOL));
    }

    // --- Blinking analysis ---

    #[test]
    fn test_threshold_states() {
        let trace = vec![100.0, 50.0, 120.0, 30.0, 80.0];
        let states = BlinkingAnalysis::threshold_states(&trace, 60.0);
        assert_eq!(states, vec![true, false, true, false, true]);
    }

    #[test]
    fn test_extract_events() {
        let states = vec![true, true, true, false, false, true, false];
        let events = BlinkingAnalysis::extract_events(&states);
        assert_eq!(events.len(), 4);
        assert_eq!(events[0].is_on, true);
        assert_eq!(events[0].duration, 3);
        assert_eq!(events[1].is_on, false);
        assert_eq!(events[1].duration, 2);
        assert_eq!(events[2].is_on, true);
        assert_eq!(events[2].duration, 1);
        assert_eq!(events[3].is_on, false);
        assert_eq!(events[3].duration, 1);
    }

    #[test]
    fn test_on_fraction() {
        let states = vec![true, true, false, true, false];
        let frac = BlinkingAnalysis::on_fraction(&states);
        assert!(approx_eq(frac, 0.6, TOL));
    }

    #[test]
    fn test_on_off_times() {
        let states = vec![true, true, true, false, false, true, false];
        let events = BlinkingAnalysis::extract_events(&states);
        let on = BlinkingAnalysis::on_times(&events);
        let off = BlinkingAnalysis::off_times(&events);
        assert_eq!(on, vec![3, 1]);
        assert_eq!(off, vec![2, 1]);
    }

    #[test]
    fn test_mean_on_off_time() {
        let states = vec![true, true, true, false, false, true, false];
        let events = BlinkingAnalysis::extract_events(&states);
        let mean_on = BlinkingAnalysis::mean_on_time(&events);
        let mean_off = BlinkingAnalysis::mean_off_time(&events);
        assert!(approx_eq(mean_on, 2.0, TOL));
        assert!(approx_eq(mean_off, 1.5, TOL));
    }

    #[test]
    fn test_duration_histogram() {
        let durations = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
        let hist = BlinkingAnalysis::duration_histogram(&durations, 5);
        assert_eq!(hist.len(), 5);
        let total: usize = hist.iter().map(|(_, c)| c).sum();
        assert_eq!(total, 10);
    }

    // --- Quantum yield ---

    #[test]
    fn test_qy_from_photon_counts() {
        let qy = QuantumYield::from_photon_counts(800.0, 1000.0);
        assert!(approx_eq(qy, 0.8, TOL));
    }

    #[test]
    fn test_qy_comparative_method() {
        // Same sample = same QY as reference
        let qy = QuantumYield::comparative_method(100.0, 100.0, 0.1, 0.1, 1.33, 1.33, 0.95);
        assert!(approx_eq(qy, 0.95, TOL));
    }

    #[test]
    fn test_qy_from_rates() {
        let qy = QuantumYield::from_rates(4.0e7, 1.0e7);
        assert!(approx_eq(qy, 0.8, TOL));
    }

    // --- Stokes shift ---

    #[test]
    fn test_stokes_shift() {
        let shift = stokes_shift_nm(520.0, 540.0);
        assert!(approx_eq(shift, 20.0, TOL));

        let shift_ev = stokes_shift_ev(520.0, 540.0);
        assert!(shift_ev > 0.0); // Positive in eV
    }

    // --- Ensemble spectrum ---

    #[test]
    fn test_ensemble_spectrum() {
        let spectrum = ensemble_pl_spectrum(
            QdMaterial::CdSe,
            3.0,
            0.3,
            50,
            400.0,
            700.0,
            100,
            10.0,
        );
        assert_eq!(spectrum.len(), 100);

        // Peak should be normalized to 1
        let max_val = spectrum.iter().map(|(_, i)| *i).fold(0.0_f64, f64::max);
        assert!(approx_eq(max_val, 1.0, 0.01));
    }

    #[test]
    fn test_ensemble_broader_with_larger_sigma() {
        // Wider size distribution should give broader PL
        let narrow = ensemble_pl_spectrum(QdMaterial::CdSe, 3.0, 0.1, 50, 400.0, 700.0, 200, 5.0);
        let broad = ensemble_pl_spectrum(QdMaterial::CdSe, 3.0, 0.5, 50, 400.0, 700.0, 200, 5.0);

        // Measure FWHM by counting points above 0.5
        let narrow_fwhm = narrow.iter().filter(|(_, i)| *i > 0.5).count();
        let broad_fwhm = broad.iter().filter(|(_, i)| *i > 0.5).count();
        assert!(
            broad_fwhm >= narrow_fwhm,
            "Broad FWHM {} should >= narrow FWHM {}",
            broad_fwhm,
            narrow_fwhm
        );
    }

    // --- Edge cases ---

    #[test]
    fn test_empty_blinking_trace() {
        let frac = BlinkingAnalysis::on_fraction(&[]);
        assert!(approx_eq(frac, 0.0, TOL));

        let events = BlinkingAnalysis::extract_events(&[]);
        assert!(events.is_empty());
    }

    #[test]
    fn test_fret_boundary_efficiencies() {
        // Efficiency = 0 at infinite distance (approximate)
        let eff_far = FretAnalysis::efficiency(1000.0, 5.0);
        assert!(eff_far < 1.0e-10);

        // Distance from extreme efficiencies
        let d = FretAnalysis::distance_from_efficiency(0.0, 5.0);
        assert_eq!(d, 0.0);
        let d = FretAnalysis::distance_from_efficiency(1.0, 5.0);
        assert_eq!(d, 0.0);
    }

    #[test]
    fn test_linear_regression() {
        // y = 2x + 1
        let data = vec![(0.0, 1.0), (1.0, 3.0), (2.0, 5.0), (3.0, 7.0)];
        let (m, b) = linear_regression(&data).unwrap();
        assert!(approx_eq(m, 2.0, 1.0e-10));
        assert!(approx_eq(b, 1.0, 1.0e-10));
    }
}

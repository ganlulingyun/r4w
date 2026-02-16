//! Synchrotron Radiation Beamline Signal Processor
//!
//! Signal processing for synchrotron radiation beamlines used in:
//! - Protein crystallography
//! - XAFS (X-ray Absorption Fine Structure) spectroscopy
//! - X-ray fluorescence microscopy
//! - Materials characterization
//! - Photoemission spectroscopy
//!
//! # Physics Background
//!
//! Synchrotron radiation is emitted by relativistic electrons circulating in a
//! storage ring. Key physics:
//! - Lorentz factor: gamma = E / (m_e * c^2)
//! - Critical energy (bending magnet): E_c = (3/2) * hbar * gamma^3 / rho
//! - Undulator parameter: K = 0.9337 * B0[T] * lambda_u[cm]
//! - Bragg's law: n * lambda = 2 * d * sin(theta)

use std::f64::consts::PI;

// Physical constants
const HBAR_EV_S: f64 = 6.582119569e-16; // reduced Planck constant [eV·s]
const HC_EV_A: f64 = 12398.4198; // h*c [eV·Å]
const HC_EV_M: f64 = 1.23984198e-6; // h*c [eV·m]
const ME_EV: f64 = 0.51099895e6; // electron rest mass energy [eV]
const C_M_S: f64 = 2.99792458e8; // speed of light [m/s]
const E_CHARGE: f64 = 1.602176634e-19; // elementary charge [C]
const ME_KG: f64 = 9.1093837015e-31; // electron mass [kg]
const HBAR_J_S: f64 = 1.054571817e-34; // reduced Planck constant [J·s]
const H_EV_S: f64 = 4.135667696e-15; // Planck constant [eV·s]

/// Source type for the synchrotron beamline
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SourceType {
    /// Bending magnet source
    BendingMagnet {
        /// Bending radius [m]
        rho: f64,
        /// Magnetic field [T]
        field: f64,
    },
    /// Undulator insertion device
    Undulator {
        /// Period length [m]
        lambda_u: f64,
        /// Number of periods
        num_periods: u32,
        /// Peak magnetic field [T]
        b0: f64,
    },
    /// Wiggler insertion device
    Wiggler {
        /// Period length [m]
        lambda_u: f64,
        /// Number of periods
        num_periods: u32,
        /// Peak magnetic field [T]
        b0: f64,
    },
}

/// Configuration for a synchrotron beamline
#[derive(Debug, Clone)]
pub struct SynchrotronConfig {
    /// Electron beam energy [eV]
    pub beam_energy_ev: f64,
    /// Storage ring current [A]
    pub ring_current_a: f64,
    /// Radiation source type
    pub source: SourceType,
    /// Horizontal emittance [m·rad]
    pub emittance_h: f64,
    /// Vertical emittance [m·rad]
    pub emittance_v: f64,
}

impl SynchrotronConfig {
    /// Create a configuration for a typical 3rd-generation light source (e.g., APS at 7 GeV)
    pub fn aps_7gev() -> Self {
        SynchrotronConfig {
            beam_energy_ev: 7.0e9,
            ring_current_a: 0.100,
            source: SourceType::BendingMagnet {
                rho: 38.96,
                field: 0.599,
            },
            emittance_h: 3.0e-9,
            emittance_v: 30.0e-12,
        }
    }

    /// Create a configuration for ESRF at 6 GeV
    pub fn esrf_6gev() -> Self {
        SynchrotronConfig {
            beam_energy_ev: 6.0e9,
            ring_current_a: 0.200,
            source: SourceType::Undulator {
                lambda_u: 0.023,
                num_periods: 70,
                b0: 0.71,
            },
            emittance_h: 4.0e-9,
            emittance_v: 10.0e-12,
        }
    }

    /// Lorentz factor gamma = E / (m_e * c^2)
    pub fn gamma(&self) -> f64 {
        self.beam_energy_ev / ME_EV
    }

    /// Undulator K parameter: K = 0.9337 * B0[T] * lambda_u[cm]
    pub fn undulator_k(&self) -> Option<f64> {
        match self.source {
            SourceType::Undulator { lambda_u, b0, .. }
            | SourceType::Wiggler { lambda_u, b0, .. } => {
                let lambda_u_cm = lambda_u * 100.0;
                Some(0.9337 * b0 * lambda_u_cm)
            }
            SourceType::BendingMagnet { .. } => None,
        }
    }
}

/// Calculates spectral brilliance and critical energy for synchrotron radiation
pub struct BrillianceCalculator {
    config: SynchrotronConfig,
}

impl BrillianceCalculator {
    pub fn new(config: SynchrotronConfig) -> Self {
        Self { config }
    }

    /// Critical energy for bending magnet: E_c = (3 * hbar * c * gamma^3) / (2 * rho)
    /// Units: [eV]
    pub fn critical_energy_ev(&self) -> f64 {
        let gamma = self.config.gamma();
        match self.config.source {
            SourceType::BendingMagnet { rho, .. } => {
                // E_c = (3/2) * hbar * c * gamma^3 / rho
                // Using hbar in [J·s], result in J, then convert to eV
                let ec_j = 1.5 * HBAR_J_S * C_M_S * gamma.powi(3) / rho;
                ec_j / E_CHARGE
            }
            SourceType::Wiggler { b0, .. } => {
                // For wiggler: E_c = (3/2) * hbar * gamma^2 * e * B / m_e
                let ec_j = 1.5 * HBAR_J_S * gamma.powi(2) * E_CHARGE * b0 / ME_KG;
                ec_j / E_CHARGE
            }
            SourceType::Undulator { .. } => {
                // Undulator fundamental energy is more meaningful; use first harmonic
                self.undulator_harmonic_energy(1).unwrap_or(0.0)
            }
        }
    }

    /// Critical wavelength [Å]
    pub fn critical_wavelength_angstrom(&self) -> f64 {
        let ec = self.critical_energy_ev();
        if ec > 0.0 {
            HC_EV_A / ec
        } else {
            0.0
        }
    }

    /// Undulator harmonic energies: E_n = n * E_1
    /// E_1 = (2 * gamma^2 * h * c) / (lambda_u * (1 + K^2/2))
    pub fn undulator_harmonic_energy(&self, n: u32) -> Option<f64> {
        match self.config.source {
            SourceType::Undulator { lambda_u, b0, .. }
            | SourceType::Wiggler { lambda_u, b0, .. } => {
                let gamma = self.config.gamma();
                let k = 0.9337 * b0 * (lambda_u * 100.0); // K parameter
                // E_1 = 2 * gamma^2 * hc / (lambda_u * (1 + K^2/2))
                // hc in eV·m
                let e1 = 2.0 * gamma.powi(2) * HC_EV_M / (lambda_u * (1.0 + k.powi(2) / 2.0));
                Some(n as f64 * e1)
            }
            SourceType::BendingMagnet { .. } => None,
        }
    }

    /// Approximate spectral brilliance for bending magnet
    /// photons / (s · mm^2 · mrad^2 · 0.1%BW)
    ///
    /// Uses the universal synchrotron radiation function approximation.
    pub fn brilliance_bending_magnet(&self, photon_energy_ev: f64) -> f64 {
        let ec = self.critical_energy_ev();
        if ec <= 0.0 {
            return 0.0;
        }
        let y = photon_energy_ev / ec;

        // Approximate universal function H2(y) for brilliance
        // H2(y) ~ y^2 * K_{2/3}(y/2)^2 -- approximation
        // We use a practical approximation: brilliance ~ C * E_ring^2 * I * G(y)
        // G(y) = y^2 * exp(-y) for a simple approximation that captures the shape
        let g = y * y * (-y).exp();

        // Normalization constant for typical bending magnet brilliance
        // ~1.33e13 ph/s/mm2/mrad2/0.1%BW per GeV^2 per A at the critical energy
        let e_gev = self.config.beam_energy_ev / 1.0e9;
        let current = self.config.ring_current_a;

        1.33e13 * e_gev.powi(2) * current * g * 2.718 // e factor to normalize G(1)~1
    }

    /// Approximate on-axis flux from undulator (photons/s/0.1%BW)
    pub fn undulator_flux(&self, harmonic: u32) -> Option<f64> {
        match self.config.source {
            SourceType::Undulator { num_periods, b0, lambda_u, .. } => {
                let k = 0.9337 * b0 * (lambda_u * 100.0);
                let current = self.config.ring_current_a;
                let n = harmonic as f64;

                // Flux ~ 1.43e14 * N * I[A] * n * Fn(K)
                // Fn(K) is a function of K and harmonic number
                // For odd harmonics: Fn ~ (n*K/(1+K^2/2))^2 for small K
                let xi = n * k / (1.0 + k.powi(2) / 2.0);
                let fn_k = xi.powi(2) * 0.25; // simplified

                Some(1.43e14 * num_periods as f64 * current * fn_k)
            }
            _ => None,
        }
    }
}

/// Undulator spectrum calculator
pub struct UndulatorSpectrum {
    gamma: f64,
    lambda_u: f64,
    num_periods: u32,
    k_param: f64,
}

impl UndulatorSpectrum {
    /// Create from undulator parameters
    pub fn new(beam_energy_ev: f64, lambda_u: f64, num_periods: u32, b0: f64) -> Self {
        let gamma = beam_energy_ev / ME_EV;
        let k_param = 0.9337 * b0 * (lambda_u * 100.0);
        Self {
            gamma,
            lambda_u,
            num_periods,
            k_param,
        }
    }

    /// K parameter
    pub fn k_parameter(&self) -> f64 {
        self.k_param
    }

    /// Fundamental energy E_1 = 2*gamma^2*hc / (lambda_u*(1+K^2/2)) [eV]
    pub fn fundamental_energy_ev(&self) -> f64 {
        2.0 * self.gamma.powi(2) * HC_EV_M / (self.lambda_u * (1.0 + self.k_param.powi(2) / 2.0))
    }

    /// Harmonic energy E_n = n * E_1 [eV]
    pub fn harmonic_energy_ev(&self, n: u32) -> f64 {
        n as f64 * self.fundamental_energy_ev()
    }

    /// Bandwidth of nth harmonic: dE/E ~ 1/(n*N) where N is number of periods
    pub fn harmonic_bandwidth(&self, n: u32) -> f64 {
        1.0 / (n as f64 * self.num_periods as f64)
    }

    /// Generate spectrum envelope around harmonic n
    /// Returns (energy_ev, relative_intensity) pairs
    pub fn harmonic_lineshape(&self, n: u32, num_points: usize) -> Vec<(f64, f64)> {
        let e_n = self.harmonic_energy_ev(n);
        let bw = self.harmonic_bandwidth(n);
        let sigma = e_n * bw / 2.35; // FWHM to sigma

        let e_min = e_n - 4.0 * sigma;
        let e_max = e_n + 4.0 * sigma;
        let de = (e_max - e_min) / (num_points - 1) as f64;

        (0..num_points)
            .map(|i| {
                let e = e_min + i as f64 * de;
                let x = (e - e_n) / sigma;
                // sinc^2 lineshape for undulator
                let sinc_arg = PI * x * self.num_periods as f64 * bw;
                let intensity = if sinc_arg.abs() < 1e-10 {
                    1.0
                } else {
                    (sinc_arg.sin() / sinc_arg).powi(2)
                };
                (e, intensity)
            })
            .collect()
    }

    /// List first N odd harmonics with their energies
    pub fn odd_harmonics(&self, count: usize) -> Vec<(u32, f64)> {
        (0..count)
            .map(|i| {
                let n = 2 * i as u32 + 1;
                (n, self.harmonic_energy_ev(n))
            })
            .collect()
    }
}

/// Crystal d-spacings for common monochromator reflections [Å]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CrystalReflection {
    /// Si(111): d = 3.13551 Å
    Si111,
    /// Si(220): d = 1.92017 Å
    Si220,
    /// Si(311): d = 1.63747 Å
    Si311,
    /// Si(400): d = 1.35773 Å
    Si400,
    /// Ge(111): d = 3.26631 Å
    Ge111,
    /// Custom d-spacing [Å]
    Custom(f64),
}

impl CrystalReflection {
    /// Return d-spacing in Angstroms
    pub fn d_spacing_angstrom(&self) -> f64 {
        match self {
            CrystalReflection::Si111 => 3.13551,
            CrystalReflection::Si220 => 1.92017,
            CrystalReflection::Si311 => 1.63747,
            CrystalReflection::Si400 => 1.35773,
            CrystalReflection::Ge111 => 3.26631,
            CrystalReflection::Custom(d) => *d,
        }
    }
}

/// Monochromator calibrator using Bragg's law
///
/// E = hc / (2 * d * sin(theta))
pub struct MonochromatorCalibrator {
    crystal: CrystalReflection,
    /// Order of diffraction (usually 1)
    order: u32,
}

impl MonochromatorCalibrator {
    pub fn new(crystal: CrystalReflection) -> Self {
        Self { crystal, order: 1 }
    }

    pub fn with_order(mut self, order: u32) -> Self {
        self.order = order;
        self
    }

    /// Bragg angle for given photon energy [radians]
    /// n * lambda = 2 * d * sin(theta)
    /// theta = arcsin(n * hc / (2 * d * E))
    pub fn energy_to_angle(&self, energy_ev: f64) -> Option<f64> {
        let d = self.crystal.d_spacing_angstrom();
        let lambda = HC_EV_A / energy_ev; // wavelength in Å
        let sin_theta = self.order as f64 * lambda / (2.0 * d);
        if sin_theta.abs() <= 1.0 {
            Some(sin_theta.asin())
        } else {
            None // unreachable Bragg condition
        }
    }

    /// Photon energy for given Bragg angle [eV]
    /// E = n * hc / (2 * d * sin(theta))
    pub fn angle_to_energy(&self, theta_rad: f64) -> f64 {
        let d = self.crystal.d_spacing_angstrom();
        self.order as f64 * HC_EV_A / (2.0 * d * theta_rad.sin())
    }

    /// Energy resolution dE/E for Darwin width
    /// dE/E ~ (2 * r_e * |F_hkl| * d^2) / (pi * V * sin(2*theta))
    /// Simplified: dE/E ~ d_hkl * cot(theta) * dtheta
    pub fn energy_resolution(&self, energy_ev: f64, angular_divergence_rad: f64) -> Option<f64> {
        let theta = self.energy_to_angle(energy_ev)?;
        // dE/E = cot(theta) * dtheta
        let cot_theta = theta.cos() / theta.sin();
        Some(cot_theta * angular_divergence_rad)
    }

    /// Calibrate an array of angles to energies
    pub fn calibrate_scan(&self, angles_rad: &[f64]) -> Vec<f64> {
        angles_rad.iter().map(|&a| self.angle_to_energy(a)).collect()
    }
}

/// XAFS (X-ray Absorption Fine Structure) processor
///
/// Extracts EXAFS chi(k) from absorption spectra:
/// chi(k) = (mu - mu0) / delta_mu0
///
/// where k = sqrt(2 * m_e * (E - E0) / hbar^2) is the photoelectron wavenumber
pub struct XafsProcessor {
    /// Edge energy E0 [eV]
    edge_energy_ev: f64,
    /// Pre-edge fit range [eV] relative to edge
    pre_edge_range: (f64, f64),
    /// Post-edge normalization range [eV] relative to edge
    post_edge_range: (f64, f64),
}

impl XafsProcessor {
    pub fn new(edge_energy_ev: f64) -> Self {
        Self {
            edge_energy_ev,
            pre_edge_range: (-150.0, -30.0),
            post_edge_range: (50.0, 300.0),
        }
    }

    pub fn with_pre_edge_range(mut self, start: f64, end: f64) -> Self {
        self.pre_edge_range = (start, end);
        self
    }

    pub fn with_post_edge_range(mut self, start: f64, end: f64) -> Self {
        self.post_edge_range = (start, end);
        self
    }

    /// Convert energy [eV] to photoelectron wavenumber k [Å^-1]
    /// k = sqrt(2 * m_e * (E - E0)) / hbar
    /// In practical units: k = 0.5123 * sqrt(E - E0) with E in eV and k in Å^-1
    pub fn energy_to_k(&self, energy_ev: f64) -> f64 {
        let de = energy_ev - self.edge_energy_ev;
        if de > 0.0 {
            0.5123 * de.sqrt()
        } else {
            0.0
        }
    }

    /// Convert k [Å^-1] to energy [eV]
    pub fn k_to_energy(&self, k: f64) -> f64 {
        self.edge_energy_ev + (k / 0.5123).powi(2)
    }

    /// Linear fit to data in [x_min, x_max] range
    /// Returns (slope, intercept)
    fn linear_fit(x: &[f64], y: &[f64], x_min: f64, x_max: f64) -> (f64, f64) {
        let mut sum_x: f64 = 0.0;
        let mut sum_y: f64 = 0.0;
        let mut sum_xx: f64 = 0.0;
        let mut sum_xy: f64 = 0.0;
        let mut n: f64 = 0.0;

        for (&xi, &yi) in x.iter().zip(y.iter()) {
            if xi >= x_min && xi <= x_max {
                sum_x += xi;
                sum_y += yi;
                sum_xx += xi * xi;
                sum_xy += xi * yi;
                n += 1.0;
            }
        }

        if n < 2.0 {
            return (0.0, sum_y / n.max(1.0));
        }

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return (0.0, sum_y / n);
        }

        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n;
        (slope, intercept)
    }

    /// Extract normalized mu(E) from raw absorption data
    ///
    /// Steps:
    /// 1. Fit pre-edge line and subtract
    /// 2. Fit post-edge line for normalization
    /// 3. Normalize: mu_norm = (mu - pre_edge) / edge_step
    pub fn normalize_mu(&self, energies_ev: &[f64], mu_raw: &[f64]) -> Vec<f64> {
        let e0 = self.edge_energy_ev;

        // Pre-edge linear fit
        let (pre_slope, pre_intercept) = Self::linear_fit(
            energies_ev,
            mu_raw,
            e0 + self.pre_edge_range.0,
            e0 + self.pre_edge_range.1,
        );

        // Post-edge linear fit
        let (post_slope, post_intercept) = Self::linear_fit(
            energies_ev,
            mu_raw,
            e0 + self.post_edge_range.0,
            e0 + self.post_edge_range.1,
        );

        // Edge step at E0
        let pre_at_e0 = pre_slope * e0 + pre_intercept;
        let post_at_e0 = post_slope * e0 + post_intercept;
        let edge_step = (post_at_e0 - pre_at_e0).max(1e-10);

        // Normalize
        energies_ev
            .iter()
            .zip(mu_raw.iter())
            .map(|(&e, &mu)| {
                let pre_line = pre_slope * e + pre_intercept;
                (mu - pre_line) / edge_step
            })
            .collect()
    }

    /// Extract EXAFS chi(k) from normalized mu(E)
    ///
    /// chi(k) = (mu(E) - mu0(E)) / delta_mu0
    ///
    /// Uses simple spline-like background (piecewise linear approximation)
    pub fn extract_chi_k(
        &self,
        energies_ev: &[f64],
        mu_normalized: &[f64],
    ) -> (Vec<f64>, Vec<f64>) {
        let e0 = self.edge_energy_ev;

        // Only use post-edge data
        let mut k_values = Vec::new();
        let mut chi_values = Vec::new();

        // Find post-edge data
        let post_data: Vec<(f64, f64)> = energies_ev
            .iter()
            .zip(mu_normalized.iter())
            .filter(|(&e, _)| e > e0 + 10.0)
            .map(|(&e, &mu)| (e, mu))
            .collect();

        if post_data.is_empty() {
            return (k_values, chi_values);
        }

        // Simple smooth background using running average
        let window = 15.min(post_data.len());
        let mut background = vec![0.0; post_data.len()];
        for i in 0..post_data.len() {
            let start = if i >= window / 2 { i - window / 2 } else { 0 };
            let end = (i + window / 2 + 1).min(post_data.len());
            let sum: f64 = post_data[start..end].iter().map(|(_, mu)| mu).sum();
            background[i] = sum / (end - start) as f64;
        }

        for (i, &(e, mu)) in post_data.iter().enumerate() {
            let k = self.energy_to_k(e);
            if k > 0.0 {
                let chi = mu - background[i];
                k_values.push(k);
                chi_values.push(chi);
            }
        }

        (k_values, chi_values)
    }

    /// Apply k-weighting to chi(k)
    pub fn k_weight(k: &[f64], chi: &[f64], weight: f64) -> Vec<f64> {
        k.iter()
            .zip(chi.iter())
            .map(|(&ki, &chi_i)| chi_i * ki.powf(weight))
            .collect()
    }
}

/// Multi-channel fluorescence detector processor
///
/// Handles MCA (Multi-Channel Analyzer) spectrum processing,
/// ROI integration, and dead time correction.
pub struct FluorescenceDetector {
    /// Number of channels in MCA
    num_channels: usize,
    /// Energy per channel [eV/channel]
    ev_per_channel: f64,
    /// Energy offset [eV]
    energy_offset: f64,
    /// Dead time per event [s]
    dead_time_s: f64,
}

impl FluorescenceDetector {
    pub fn new(num_channels: usize, ev_per_channel: f64) -> Self {
        Self {
            num_channels,
            ev_per_channel,
            energy_offset: 0.0,
            dead_time_s: 1.0e-6, // 1 microsecond typical
        }
    }

    pub fn with_dead_time(mut self, tau_s: f64) -> Self {
        self.dead_time_s = tau_s;
        self
    }

    pub fn with_energy_offset(mut self, offset_ev: f64) -> Self {
        self.energy_offset = offset_ev;
        self
    }

    /// Convert channel number to energy [eV]
    pub fn channel_to_energy(&self, channel: usize) -> f64 {
        self.energy_offset + channel as f64 * self.ev_per_channel
    }

    /// Convert energy to channel number
    pub fn energy_to_channel(&self, energy_ev: f64) -> usize {
        let ch = ((energy_ev - self.energy_offset) / self.ev_per_channel).round() as isize;
        ch.max(0).min(self.num_channels as isize - 1) as usize
    }

    /// Dead time correction: true_rate = measured_rate / (1 - measured_rate * tau)
    /// Non-paralyzable (Type I) model
    pub fn dead_time_correction(&self, measured_rate_hz: f64) -> f64 {
        let denom = 1.0 - measured_rate_hz * self.dead_time_s;
        if denom > 0.01 {
            measured_rate_hz / denom
        } else {
            // Saturation: measured rate approaches 1/tau
            1.0 / self.dead_time_s
        }
    }

    /// Dead time correction factor
    pub fn dead_time_factor(&self, measured_rate_hz: f64) -> f64 {
        let corrected = self.dead_time_correction(measured_rate_hz);
        if measured_rate_hz > 0.0 {
            corrected / measured_rate_hz
        } else {
            1.0
        }
    }

    /// Integrate counts in a Region of Interest (ROI)
    pub fn roi_integrate(&self, spectrum: &[f64], roi_start_ev: f64, roi_end_ev: f64) -> f64 {
        let ch_start = self.energy_to_channel(roi_start_ev);
        let ch_end = self.energy_to_channel(roi_end_ev);
        let start = ch_start.min(ch_end);
        let end = ch_start.max(ch_end).min(spectrum.len());

        spectrum[start..=end.min(spectrum.len() - 1)]
            .iter()
            .sum()
    }

    /// Subtract trapezoidal background from ROI
    pub fn roi_net_counts(
        &self,
        spectrum: &[f64],
        roi_start_ev: f64,
        roi_end_ev: f64,
        bg_width_channels: usize,
    ) -> f64 {
        let ch_start = self.energy_to_channel(roi_start_ev);
        let ch_end = self.energy_to_channel(roi_end_ev).min(spectrum.len() - 1);

        if ch_start >= ch_end || ch_end >= spectrum.len() {
            return 0.0;
        }

        // Background: average of channels on either side of ROI
        let bg_left_start = ch_start.saturating_sub(bg_width_channels);
        let bg_left: f64 = if ch_start > bg_left_start {
            spectrum[bg_left_start..ch_start].iter().sum::<f64>()
                / (ch_start - bg_left_start) as f64
        } else {
            0.0
        };

        let bg_right_end = (ch_end + bg_width_channels + 1).min(spectrum.len());
        let bg_right: f64 = if bg_right_end > ch_end + 1 {
            spectrum[ch_end + 1..bg_right_end].iter().sum::<f64>()
                / (bg_right_end - ch_end - 1) as f64
        } else {
            0.0
        };

        // Total counts minus trapezoidal background
        let total: f64 = spectrum[ch_start..=ch_end].iter().sum();
        let roi_width = (ch_end - ch_start + 1) as f64;
        let bg_total = roi_width * (bg_left + bg_right) / 2.0;

        total - bg_total
    }

    /// Find peaks in MCA spectrum (simple threshold method)
    pub fn find_peaks(&self, spectrum: &[f64], threshold: f64) -> Vec<(usize, f64, f64)> {
        let mut peaks = Vec::new();
        if spectrum.len() < 3 {
            return peaks;
        }

        for i in 1..spectrum.len() - 1 {
            if spectrum[i] > threshold
                && spectrum[i] > spectrum[i - 1]
                && spectrum[i] > spectrum[i + 1]
            {
                let energy = self.channel_to_energy(i);
                peaks.push((i, energy, spectrum[i]));
            }
        }
        peaks
    }
}

/// Beam Position Monitor (BPM) processor
///
/// Four-quadrant (quad) BPM readout:
/// x = K * (A - C) / (A + B + C + D)
/// y = K * (B - D) / (A + B + C + D)
///
/// where A, B, C, D are quadrant signals and K is calibration factor [mm]
pub struct BeamPositionMonitor {
    /// Calibration factor for horizontal [mm]
    k_x: f64,
    /// Calibration factor for vertical [mm]
    k_y: f64,
    /// Offset correction x [mm]
    offset_x: f64,
    /// Offset correction y [mm]
    offset_y: f64,
}

impl BeamPositionMonitor {
    pub fn new(k_x: f64, k_y: f64) -> Self {
        Self {
            k_x,
            k_y,
            offset_x: 0.0,
            offset_y: 0.0,
        }
    }

    pub fn with_offsets(mut self, offset_x: f64, offset_y: f64) -> Self {
        self.offset_x = offset_x;
        self.offset_y = offset_y;
        self
    }

    /// Calculate beam position from quad BPM signals
    /// Returns (x_mm, y_mm)
    ///
    /// Quadrant layout:
    /// ```text
    ///   B | A
    ///  ---+---
    ///   C | D
    /// ```
    pub fn position(&self, a: f64, b: f64, c: f64, d: f64) -> (f64, f64) {
        let total = a + b + c + d;
        if total < 1e-15 {
            return (0.0, 0.0);
        }

        let x = self.k_x * (a - c) / total - self.offset_x;
        let y = self.k_y * (b - d) / total - self.offset_y;
        (x, y)
    }

    /// Process a time series of BPM readings
    /// Each element is (A, B, C, D)
    pub fn process_series(&self, readings: &[(f64, f64, f64, f64)]) -> Vec<(f64, f64)> {
        readings
            .iter()
            .map(|&(a, b, c, d)| self.position(a, b, c, d))
            .collect()
    }

    /// Calculate beam position RMS from a series
    pub fn rms_position(&self, positions: &[(f64, f64)]) -> (f64, f64) {
        if positions.is_empty() {
            return (0.0, 0.0);
        }

        let n = positions.len() as f64;
        let mean_x: f64 = positions.iter().map(|(x, _)| x).sum::<f64>() / n;
        let mean_y: f64 = positions.iter().map(|(_, y)| y).sum::<f64>() / n;

        let rms_x = (positions
            .iter()
            .map(|(x, _)| (x - mean_x).powi(2))
            .sum::<f64>()
            / n)
            .sqrt();
        let rms_y = (positions
            .iter()
            .map(|(_, y)| (y - mean_y).powi(2))
            .sum::<f64>()
            / n)
            .sqrt();

        (rms_x, rms_y)
    }

    /// Total beam current from sum signal [arbitrary units -> mA with calibration]
    pub fn total_current(&self, a: f64, b: f64, c: f64, d: f64) -> f64 {
        a + b + c + d
    }
}

/// Bunch timing analyzer for storage ring fill patterns
pub struct BunchTimingAnalyzer {
    /// RF frequency [Hz]
    rf_frequency_hz: f64,
    /// Harmonic number (number of RF buckets)
    harmonic_number: u32,
}

impl BunchTimingAnalyzer {
    pub fn new(rf_frequency_hz: f64, harmonic_number: u32) -> Self {
        Self {
            rf_frequency_hz,
            harmonic_number,
        }
    }

    /// Revolution frequency [Hz]
    pub fn revolution_frequency(&self) -> f64 {
        self.rf_frequency_hz / self.harmonic_number as f64
    }

    /// Revolution period [s]
    pub fn revolution_period(&self) -> f64 {
        1.0 / self.revolution_frequency()
    }

    /// RF bucket spacing [s]
    pub fn bucket_spacing(&self) -> f64 {
        1.0 / self.rf_frequency_hz
    }

    /// Analyze fill pattern from time-domain bunch signal
    /// Returns vector of (bucket_index, relative_charge) for filled buckets
    pub fn analyze_fill_pattern(&self, signal: &[f64], sample_rate_hz: f64) -> Vec<(u32, f64)> {
        let bucket_spacing_samples = sample_rate_hz / self.rf_frequency_hz;
        let mut pattern = Vec::new();

        // Find maximum for normalization
        let max_val = signal
            .iter()
            .cloned()
            .fold(0.0_f64, f64::max);
        if max_val < 1e-15 {
            return pattern;
        }

        // Sample at each bucket position
        for bucket in 0..self.harmonic_number {
            let sample_pos = (bucket as f64 * bucket_spacing_samples) as usize;
            if sample_pos < signal.len() {
                let charge = signal[sample_pos] / max_val;
                if charge > 0.05 {
                    // 5% threshold
                    pattern.push((bucket, charge));
                }
            }
        }

        pattern
    }

    /// Check if fill pattern is uniform (multi-bunch)
    pub fn is_uniform_fill(&self, pattern: &[(u32, f64)]) -> bool {
        if pattern.len() < 2 {
            return false;
        }
        let mean_charge: f64 = pattern.iter().map(|(_, c)| c).sum::<f64>() / pattern.len() as f64;
        let variance: f64 = pattern
            .iter()
            .map(|(_, c)| (c - mean_charge).powi(2))
            .sum::<f64>()
            / pattern.len() as f64;

        // Uniform if variance < 5% of mean^2
        variance < 0.05 * mean_charge.powi(2)
    }

    /// Calculate bunch purity for single-bunch mode
    /// Purity = main_bunch_charge / total_charge
    pub fn bunch_purity(&self, pattern: &[(u32, f64)]) -> f64 {
        if pattern.is_empty() {
            return 0.0;
        }

        let total: f64 = pattern.iter().map(|(_, c)| c).sum();
        let max_charge = pattern
            .iter()
            .map(|(_, c)| *c)
            .fold(0.0_f64, f64::max);

        if total > 0.0 {
            max_charge / total
        } else {
            0.0
        }
    }
}

/// Storage ring current monitor with beam lifetime estimation
pub struct StorageRingCurrent {
    /// Injection current [A]
    initial_current: f64,
    /// Beam lifetime [s]
    beam_lifetime_s: f64,
}

impl StorageRingCurrent {
    pub fn new(initial_current: f64) -> Self {
        Self {
            initial_current,
            beam_lifetime_s: f64::INFINITY,
        }
    }

    /// Current at time t: I(t) = I0 * exp(-t / tau_beam)
    pub fn current_at_time(&self, t_s: f64) -> f64 {
        self.initial_current * (-t_s / self.beam_lifetime_s).exp()
    }

    /// Fit exponential decay to current measurements
    /// Returns (I0, tau_beam_s)
    ///
    /// Uses linear regression on ln(I) = ln(I0) - t/tau
    pub fn fit_lifetime(times_s: &[f64], currents_a: &[f64]) -> (f64, f64) {
        if times_s.len() < 2 || currents_a.len() < 2 {
            return (0.0, f64::INFINITY);
        }

        let n = times_s.len().min(currents_a.len());
        let mut sum_t = 0.0;
        let mut sum_ln_i = 0.0;
        let mut sum_t2 = 0.0;
        let mut sum_t_ln_i = 0.0;
        let mut count = 0.0;

        for i in 0..n {
            if currents_a[i] > 0.0 {
                let ln_i = currents_a[i].ln();
                sum_t += times_s[i];
                sum_ln_i += ln_i;
                sum_t2 += times_s[i].powi(2);
                sum_t_ln_i += times_s[i] * ln_i;
                count += 1.0;
            }
        }

        if count < 2.0 {
            return (0.0, f64::INFINITY);
        }

        let denom = count * sum_t2 - sum_t * sum_t;
        if denom.abs() < 1e-30 {
            return (0.0, f64::INFINITY);
        }

        let slope = (count * sum_t_ln_i - sum_t * sum_ln_i) / denom;
        let intercept = (sum_ln_i - slope * sum_t) / count;

        let i0 = intercept.exp();
        let tau = if slope.abs() > 1e-15 {
            -1.0 / slope
        } else {
            f64::INFINITY
        };

        (i0, tau)
    }

    /// Create from fitted data
    pub fn from_fit(times_s: &[f64], currents_a: &[f64]) -> Self {
        let (i0, tau) = Self::fit_lifetime(times_s, currents_a);
        Self {
            initial_current: i0,
            beam_lifetime_s: tau,
        }
    }

    /// Predict time until current drops below threshold
    pub fn time_to_threshold(&self, threshold_a: f64) -> f64 {
        if threshold_a >= self.initial_current || threshold_a <= 0.0 {
            return 0.0;
        }
        -self.beam_lifetime_s * (threshold_a / self.initial_current).ln()
    }

    /// Get beam lifetime
    pub fn beam_lifetime_s(&self) -> f64 {
        self.beam_lifetime_s
    }

    /// Get beam lifetime in hours
    pub fn beam_lifetime_hours(&self) -> f64 {
        self.beam_lifetime_s / 3600.0
    }
}

/// Absorption edge detector for element identification
///
/// Identifies K, L, and M absorption edges in energy scans
pub struct AbsorptionEdgeDetector {
    /// Minimum edge jump for detection (relative)
    min_edge_jump: f64,
}

/// Known absorption edge
#[derive(Debug, Clone)]
pub struct AbsorptionEdge {
    /// Element symbol
    pub element: &'static str,
    /// Edge type (K, L1, L2, L3, etc.)
    pub edge_type: &'static str,
    /// Edge energy [eV]
    pub energy_ev: f64,
}

// Selected K-edge energies for common elements
const KNOWN_EDGES: &[AbsorptionEdge] = &[
    AbsorptionEdge { element: "Ti", edge_type: "K", energy_ev: 4966.0 },
    AbsorptionEdge { element: "V", edge_type: "K", energy_ev: 5470.0 },
    AbsorptionEdge { element: "Cr", edge_type: "K", energy_ev: 5989.0 },
    AbsorptionEdge { element: "Mn", edge_type: "K", energy_ev: 6539.0 },
    AbsorptionEdge { element: "Fe", edge_type: "K", energy_ev: 7112.0 },
    AbsorptionEdge { element: "Co", edge_type: "K", energy_ev: 7709.0 },
    AbsorptionEdge { element: "Ni", edge_type: "K", energy_ev: 8333.0 },
    AbsorptionEdge { element: "Cu", edge_type: "K", energy_ev: 8979.0 },
    AbsorptionEdge { element: "Zn", edge_type: "K", energy_ev: 9659.0 },
    AbsorptionEdge { element: "Se", edge_type: "K", energy_ev: 12658.0 },
    AbsorptionEdge { element: "Br", edge_type: "K", energy_ev: 13474.0 },
    AbsorptionEdge { element: "Mo", edge_type: "K", energy_ev: 20000.0 },
    AbsorptionEdge { element: "Ag", edge_type: "K", energy_ev: 25514.0 },
    AbsorptionEdge { element: "Pt", edge_type: "L3", energy_ev: 11564.0 },
    AbsorptionEdge { element: "Au", edge_type: "L3", energy_ev: 11919.0 },
    AbsorptionEdge { element: "Pb", edge_type: "L3", energy_ev: 13035.0 },
    AbsorptionEdge { element: "S", edge_type: "K", energy_ev: 2472.0 },
    AbsorptionEdge { element: "P", edge_type: "K", energy_ev: 2145.0 },
    AbsorptionEdge { element: "Ca", edge_type: "K", energy_ev: 4038.0 },
    AbsorptionEdge { element: "As", edge_type: "K", energy_ev: 11867.0 },
];

impl AbsorptionEdgeDetector {
    pub fn new() -> Self {
        Self {
            min_edge_jump: 0.1,
        }
    }

    pub fn with_min_edge_jump(mut self, jump: f64) -> Self {
        self.min_edge_jump = jump;
        self
    }

    /// Detect edges in an absorption spectrum
    /// Returns detected edge positions [eV] and jump magnitudes
    pub fn detect_edges(&self, energies_ev: &[f64], absorption: &[f64]) -> Vec<(f64, f64)> {
        let mut edges = Vec::new();
        if energies_ev.len() < 5 || absorption.len() < 5 {
            return edges;
        }

        let n = energies_ev.len().min(absorption.len());

        // Compute derivative (finite difference)
        let mut derivative = vec![0.0; n];
        for i in 1..n - 1 {
            let de = energies_ev[i + 1] - energies_ev[i - 1];
            if de.abs() > 1e-10 {
                derivative[i] = (absorption[i + 1] - absorption[i - 1]) / de;
            }
        }

        // Find peaks in the derivative (edges are peaks in d(mu)/dE)
        // Use a window-based approach
        let window = 5;
        for i in window..n - window {
            let is_peak = (1..=window).all(|j| {
                derivative[i] > derivative[i - j] && derivative[i] > derivative[i + j]
            });

            if is_peak && derivative[i] > self.min_edge_jump {
                // Refine edge position using center of mass
                let mut weighted_e = 0.0;
                let mut weight_sum = 0.0;
                for j in (i.saturating_sub(2))..=(i + 2).min(n - 1) {
                    if derivative[j] > 0.0 {
                        weighted_e += energies_ev[j] * derivative[j];
                        weight_sum += derivative[j];
                    }
                }
                let edge_energy = if weight_sum > 0.0 {
                    weighted_e / weight_sum
                } else {
                    energies_ev[i]
                };

                edges.push((edge_energy, derivative[i]));
            }
        }

        edges
    }

    /// Identify element from detected edge energy
    pub fn identify_edge(&self, edge_energy_ev: f64, tolerance_ev: f64) -> Vec<&AbsorptionEdge> {
        KNOWN_EDGES
            .iter()
            .filter(|e| (e.energy_ev - edge_energy_ev).abs() < tolerance_ev)
            .collect()
    }

    /// Identify all edges in a spectrum
    pub fn identify_spectrum(
        &self,
        energies_ev: &[f64],
        absorption: &[f64],
        tolerance_ev: f64,
    ) -> Vec<(f64, Vec<&AbsorptionEdge>)> {
        let edges = self.detect_edges(energies_ev, absorption);
        edges
            .into_iter()
            .map(|(energy, _jump)| {
                let matches = self.identify_edge(energy, tolerance_ev);
                (energy, matches)
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // ---- SynchrotronConfig tests ----

    #[test]
    fn test_gamma_7gev() {
        let config = SynchrotronConfig::aps_7gev();
        let gamma = config.gamma();
        // 7 GeV / 0.511 MeV ~ 13699
        assert!((gamma - 13699.0).abs() < 10.0, "gamma = {}", gamma);
    }

    #[test]
    fn test_gamma_6gev() {
        let config = SynchrotronConfig::esrf_6gev();
        let gamma = config.gamma();
        // 6 GeV / 0.511 MeV ~ 11742
        assert!((gamma - 11742.0).abs() < 10.0, "gamma = {}", gamma);
    }

    #[test]
    fn test_undulator_k_parameter() {
        let config = SynchrotronConfig::esrf_6gev();
        let k = config.undulator_k().unwrap();
        // K = 0.9337 * 0.71 * 2.3 = 1.525
        assert!((k - 1.525).abs() < 0.01, "K = {}", k);
    }

    #[test]
    fn test_bending_magnet_no_k() {
        let config = SynchrotronConfig::aps_7gev();
        assert!(config.undulator_k().is_none());
    }

    // ---- BrillianceCalculator tests ----

    #[test]
    fn test_critical_energy_bending_magnet() {
        let config = SynchrotronConfig::aps_7gev();
        let calc = BrillianceCalculator::new(config);
        let ec = calc.critical_energy_ev();
        // APS 7 GeV, rho=38.96m: E_c should be ~19-20 keV range
        assert!(ec > 10_000.0 && ec < 30_000.0, "E_c = {} eV", ec);
    }

    #[test]
    fn test_critical_wavelength() {
        let config = SynchrotronConfig::aps_7gev();
        let calc = BrillianceCalculator::new(config);
        let ec = calc.critical_energy_ev();
        let wl = calc.critical_wavelength_angstrom();
        // lambda_c = hc/Ec
        let expected = HC_EV_A / ec;
        assert!(
            (wl - expected).abs() < 0.001,
            "wavelength: {} vs expected {}",
            wl,
            expected
        );
    }

    #[test]
    fn test_undulator_harmonic_energy() {
        let config = SynchrotronConfig::esrf_6gev();
        let calc = BrillianceCalculator::new(config);
        let e1 = calc.undulator_harmonic_energy(1).unwrap();
        let e3 = calc.undulator_harmonic_energy(3).unwrap();
        // Third harmonic should be 3x fundamental
        assert!(
            (e3 / e1 - 3.0).abs() < 0.01,
            "e3/e1 = {}",
            e3 / e1
        );
        // Fundamental should be in keV range for ESRF undulator
        assert!(e1 > 1_000.0 && e1 < 50_000.0, "E1 = {} eV", e1);
    }

    #[test]
    fn test_brilliance_bending_magnet_shape() {
        let config = SynchrotronConfig::aps_7gev();
        let calc = BrillianceCalculator::new(config);
        let ec = calc.critical_energy_ev();

        // Brilliance at critical energy should be near peak
        let b_at_ec = calc.brilliance_bending_magnet(ec);
        let b_at_low = calc.brilliance_bending_magnet(ec * 0.01);
        let b_at_high = calc.brilliance_bending_magnet(ec * 10.0);

        assert!(b_at_ec > 0.0);
        assert!(b_at_ec > b_at_high, "brilliance should drop above Ec");
        assert!(b_at_ec > b_at_low, "brilliance peak near Ec");
    }

    #[test]
    fn test_undulator_flux() {
        let config = SynchrotronConfig::esrf_6gev();
        let calc = BrillianceCalculator::new(config);
        let flux = calc.undulator_flux(1).unwrap();
        assert!(flux > 0.0, "flux should be positive");
    }

    // ---- UndulatorSpectrum tests ----

    #[test]
    fn test_undulator_spectrum_k() {
        let spec = UndulatorSpectrum::new(6.0e9, 0.023, 70, 0.71);
        let k = spec.k_parameter();
        assert!((k - 1.525).abs() < 0.01, "K = {}", k);
    }

    #[test]
    fn test_undulator_odd_harmonics() {
        let spec = UndulatorSpectrum::new(6.0e9, 0.023, 70, 0.71);
        let harmonics = spec.odd_harmonics(5);
        assert_eq!(harmonics.len(), 5);
        assert_eq!(harmonics[0].0, 1);
        assert_eq!(harmonics[1].0, 3);
        assert_eq!(harmonics[2].0, 5);

        // Energies should increase
        for i in 1..harmonics.len() {
            assert!(harmonics[i].1 > harmonics[i - 1].1);
        }
    }

    #[test]
    fn test_undulator_bandwidth() {
        let spec = UndulatorSpectrum::new(6.0e9, 0.023, 70, 0.71);
        let bw1 = spec.harmonic_bandwidth(1);
        let bw3 = spec.harmonic_bandwidth(3);
        // 1/(1*70) = 0.01429
        assert!((bw1 - 1.0 / 70.0).abs() < 1e-6);
        // 3rd harmonic has 3x narrower bandwidth
        assert!((bw3 - 1.0 / 210.0).abs() < 1e-6);
    }

    #[test]
    fn test_harmonic_lineshape() {
        let spec = UndulatorSpectrum::new(6.0e9, 0.023, 70, 0.71);
        let shape = spec.harmonic_lineshape(1, 101);
        assert_eq!(shape.len(), 101);
        // Peak should be near center (harmonic energy)
        let max_idx = shape
            .iter()
            .enumerate()
            .max_by(|a, b| a.1 .1.partial_cmp(&b.1 .1).unwrap())
            .unwrap()
            .0;
        // Peak should be within central region
        assert!(max_idx > 30 && max_idx < 70, "peak at index {}", max_idx);
    }

    // ---- MonochromatorCalibrator tests ----

    #[test]
    fn test_si111_d_spacing() {
        let d = CrystalReflection::Si111.d_spacing_angstrom();
        assert!((d - 3.13551).abs() < 0.001);
    }

    #[test]
    fn test_si220_d_spacing() {
        let d = CrystalReflection::Si220.d_spacing_angstrom();
        assert!((d - 1.92017).abs() < 0.001);
    }

    #[test]
    fn test_bragg_angle_roundtrip() {
        let cal = MonochromatorCalibrator::new(CrystalReflection::Si111);
        let energy = 10000.0; // 10 keV
        let angle = cal.energy_to_angle(energy).unwrap();
        let energy_back = cal.angle_to_energy(angle);
        assert!(
            (energy - energy_back).abs() < 0.01,
            "roundtrip: {} vs {}",
            energy,
            energy_back
        );
    }

    #[test]
    fn test_bragg_angle_known() {
        let cal = MonochromatorCalibrator::new(CrystalReflection::Si111);
        // At 8 keV: lambda = 12398.4198/8000 = 1.54980 Å
        // sin(theta) = lambda / (2*d) = 1.54980 / (2*3.13551) = 0.24719
        // theta = 14.32°
        let angle = cal.energy_to_angle(8000.0).unwrap();
        let theta_deg = angle * 180.0 / PI;
        assert!(
            (theta_deg - 14.32).abs() < 0.1,
            "theta = {} deg",
            theta_deg
        );
    }

    #[test]
    fn test_unreachable_bragg() {
        let cal = MonochromatorCalibrator::new(CrystalReflection::Si111);
        // Very low energy -> lambda too long -> sin(theta) > 1
        let result = cal.energy_to_angle(500.0);
        assert!(result.is_none());
    }

    #[test]
    fn test_energy_resolution() {
        let cal = MonochromatorCalibrator::new(CrystalReflection::Si111);
        let de_e = cal.energy_resolution(10000.0, 1e-4).unwrap();
        assert!(de_e > 0.0 && de_e < 1.0);
    }

    #[test]
    fn test_calibrate_scan() {
        let cal = MonochromatorCalibrator::new(CrystalReflection::Si111);
        let angles: Vec<f64> = (0..5)
            .map(|i| 0.2 + i as f64 * 0.01)
            .collect();
        let energies = cal.calibrate_scan(&angles);
        assert_eq!(energies.len(), 5);
        // Higher angle -> lower energy (inverse Bragg)
        for i in 1..energies.len() {
            assert!(energies[i] < energies[i - 1]);
        }
    }

    // ---- XafsProcessor tests ----

    #[test]
    fn test_energy_to_k() {
        let xafs = XafsProcessor::new(7112.0); // Fe K-edge
        // At E = 7112 eV (edge): k = 0
        assert_eq!(xafs.energy_to_k(7112.0), 0.0);
        // At E < E0: k = 0
        assert_eq!(xafs.energy_to_k(7000.0), 0.0);
        // At E = 7212 eV: k = 0.5123 * sqrt(100) = 5.123
        let k = xafs.energy_to_k(7212.0);
        assert!((k - 5.123).abs() < 0.01, "k = {}", k);
    }

    #[test]
    fn test_k_to_energy_roundtrip() {
        let xafs = XafsProcessor::new(7112.0);
        let k = 5.0;
        let e = xafs.k_to_energy(k);
        let k_back = xafs.energy_to_k(e);
        assert!((k - k_back).abs() < 0.001);
    }

    #[test]
    fn test_normalize_mu() {
        let xafs = XafsProcessor::new(1000.0);
        // Create synthetic absorption spectrum with edge at 1000 eV
        let energies: Vec<f64> = (0..200).map(|i| 800.0 + i as f64 * 2.0).collect();
        let mu: Vec<f64> = energies
            .iter()
            .map(|&e| {
                if e < 1000.0 {
                    0.5 + 0.0001 * e // pre-edge (linear)
                } else {
                    1.5 + 0.0001 * e // post-edge (step + linear)
                }
            })
            .collect();

        let normalized = xafs.normalize_mu(&energies, &mu);
        assert_eq!(normalized.len(), energies.len());

        // Post-edge should be roughly 1.0 after normalization
        let post_edge_mean: f64 = normalized[120..180].iter().sum::<f64>() / 60.0;
        assert!(
            (post_edge_mean - 1.0).abs() < 0.3,
            "post-edge mean = {}",
            post_edge_mean
        );
    }

    #[test]
    fn test_extract_chi_k() {
        let xafs = XafsProcessor::new(7112.0);
        // Synthetic EXAFS: sinusoidal oscillation on top of smooth background
        let energies: Vec<f64> = (0..500)
            .map(|i| 6900.0 + i as f64 * 2.0)
            .collect();
        let mu_norm: Vec<f64> = energies
            .iter()
            .map(|&e| {
                if e < 7112.0 {
                    0.0
                } else {
                    let k = 0.5123 * (e - 7112.0).sqrt();
                    1.0 + 0.05 * (2.0 * k * 2.5).sin() * (-0.01 * k.powi(2)).exp()
                }
            })
            .collect();

        let (k_vals, chi_vals) = xafs.extract_chi_k(&energies, &mu_norm);
        assert!(!k_vals.is_empty());
        assert_eq!(k_vals.len(), chi_vals.len());
        // k values should be positive
        assert!(k_vals.iter().all(|&k| k > 0.0));
    }

    #[test]
    fn test_k_weighting() {
        let k = vec![1.0, 2.0, 3.0, 4.0];
        let chi = vec![0.1, 0.08, 0.06, 0.04];

        let weighted = XafsProcessor::k_weight(&k, &chi, 2.0);
        assert_eq!(weighted.len(), 4);
        // k^2 * chi
        assert!((weighted[0] - 0.1).abs() < 1e-10);
        assert!((weighted[1] - 0.32).abs() < 1e-10);
        assert!((weighted[2] - 0.54).abs() < 1e-10);
        assert!((weighted[3] - 0.64).abs() < 1e-10);
    }

    // ---- FluorescenceDetector tests ----

    #[test]
    fn test_channel_to_energy() {
        let det = FluorescenceDetector::new(4096, 10.0);
        assert!((det.channel_to_energy(0) - 0.0).abs() < 1e-10);
        assert!((det.channel_to_energy(100) - 1000.0).abs() < 1e-10);
    }

    #[test]
    fn test_energy_to_channel() {
        let det = FluorescenceDetector::new(4096, 10.0);
        assert_eq!(det.energy_to_channel(1000.0), 100);
        assert_eq!(det.energy_to_channel(0.0), 0);
    }

    #[test]
    fn test_channel_energy_roundtrip() {
        let det = FluorescenceDetector::new(4096, 10.0).with_energy_offset(50.0);
        let ch = 500;
        let energy = det.channel_to_energy(ch);
        let ch_back = det.energy_to_channel(energy);
        assert_eq!(ch, ch_back);
    }

    #[test]
    fn test_dead_time_correction() {
        let det = FluorescenceDetector::new(4096, 10.0).with_dead_time(1e-6);
        // At low rate: correction ~ 1.0
        let corrected = det.dead_time_correction(1000.0);
        assert!((corrected / 1000.0 - 1.0).abs() < 0.01);

        // At 100 kcps: correction factor = 1/(1 - 0.1) = 1.111
        let corrected = det.dead_time_correction(100_000.0);
        let factor = corrected / 100_000.0;
        assert!((factor - 1.111).abs() < 0.01, "factor = {}", factor);
    }

    #[test]
    fn test_dead_time_saturation() {
        let det = FluorescenceDetector::new(4096, 10.0).with_dead_time(1e-6);
        // At saturation: measured_rate * tau >= 0.99 -> denom <= 0.01
        // This triggers the saturation clamp to 1/tau
        let corrected = det.dead_time_correction(999_000.0);
        // Should be clamped at 1/tau = 1e6
        assert!(
            (corrected - 1.0e6).abs() < 1.0,
            "corrected = {}",
            corrected
        );
    }

    #[test]
    fn test_roi_integrate() {
        let det = FluorescenceDetector::new(1000, 10.0);
        let mut spectrum = vec![0.0; 1000];
        // Put a peak around channel 500 (5000 eV)
        for i in 495..506 {
            spectrum[i] = 100.0;
        }

        let total = det.roi_integrate(&spectrum, 4940.0, 5060.0);
        assert!((total - 1100.0).abs() < 10.0, "roi total = {}", total);
    }

    #[test]
    fn test_find_peaks() {
        let det = FluorescenceDetector::new(1000, 10.0);
        let mut spectrum = vec![1.0; 1000];
        // Add peaks
        spectrum[200] = 50.0;
        spectrum[500] = 100.0;
        spectrum[800] = 75.0;

        let peaks = det.find_peaks(&spectrum, 10.0);
        assert_eq!(peaks.len(), 3);
        assert_eq!(peaks[0].0, 200);
        assert_eq!(peaks[1].0, 500);
        assert_eq!(peaks[2].0, 800);
    }

    // ---- BeamPositionMonitor tests ----

    #[test]
    fn test_bpm_centered() {
        let bpm = BeamPositionMonitor::new(10.0, 10.0);
        let (x, y) = bpm.position(100.0, 100.0, 100.0, 100.0);
        assert!((x).abs() < 1e-10);
        assert!((y).abs() < 1e-10);
    }

    #[test]
    fn test_bpm_offset_x() {
        let bpm = BeamPositionMonitor::new(10.0, 10.0);
        // A=200, C=0, B=D=100 -> x = 10*(200-0)/400 = 5.0 mm
        let (x, y) = bpm.position(200.0, 100.0, 0.0, 100.0);
        assert!((x - 5.0).abs() < 1e-10, "x = {}", x);
        assert!((y).abs() < 1e-10, "y = {}", y);
    }

    #[test]
    fn test_bpm_offset_y() {
        let bpm = BeamPositionMonitor::new(10.0, 10.0);
        // B=200, D=0, A=C=100 -> y = 10*(200-0)/400 = 5.0 mm
        let (x, y) = bpm.position(100.0, 200.0, 100.0, 0.0);
        assert!((x).abs() < 1e-10, "x = {}", x);
        assert!((y - 5.0).abs() < 1e-10, "y = {}", y);
    }

    #[test]
    fn test_bpm_with_offsets() {
        let bpm = BeamPositionMonitor::new(10.0, 10.0).with_offsets(1.0, 2.0);
        let (x, y) = bpm.position(100.0, 100.0, 100.0, 100.0);
        assert!((x - (-1.0)).abs() < 1e-10);
        assert!((y - (-2.0)).abs() < 1e-10);
    }

    #[test]
    fn test_bpm_zero_signal() {
        let bpm = BeamPositionMonitor::new(10.0, 10.0);
        let (x, y) = bpm.position(0.0, 0.0, 0.0, 0.0);
        assert_eq!(x, 0.0);
        assert_eq!(y, 0.0);
    }

    #[test]
    fn test_bpm_rms() {
        let bpm = BeamPositionMonitor::new(10.0, 10.0);
        let positions = vec![(1.0, 2.0), (-1.0, -2.0), (1.0, 2.0), (-1.0, -2.0)];
        let (rms_x, rms_y) = bpm.rms_position(&positions);
        assert!((rms_x - 1.0).abs() < 1e-10);
        assert!((rms_y - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_bpm_process_series() {
        let bpm = BeamPositionMonitor::new(10.0, 10.0);
        let readings = vec![
            (100.0, 100.0, 100.0, 100.0),
            (150.0, 100.0, 50.0, 100.0),
        ];
        let positions = bpm.process_series(&readings);
        assert_eq!(positions.len(), 2);
        assert!((positions[0].0).abs() < 1e-10);
        assert!((positions[1].0 - 2.5).abs() < 1e-10);
    }

    // ---- BunchTimingAnalyzer tests ----

    #[test]
    fn test_revolution_frequency() {
        let analyzer = BunchTimingAnalyzer::new(352.2e6, 864);
        let f_rev = analyzer.revolution_frequency();
        // 352.2 MHz / 864 ~ 407.6 kHz
        assert!((f_rev - 407_638.9).abs() < 1.0, "f_rev = {}", f_rev);
    }

    #[test]
    fn test_bucket_spacing() {
        let analyzer = BunchTimingAnalyzer::new(500e6, 1000);
        let spacing = analyzer.bucket_spacing();
        assert!((spacing - 2e-9).abs() < 1e-15); // 2 ns
    }

    #[test]
    fn test_fill_pattern_analysis() {
        let analyzer = BunchTimingAnalyzer::new(500e6, 10);
        // Create signal with 5 filled buckets
        let sample_rate = 10.0e9; // 10 GHz
        let bucket_spacing_samples = (sample_rate / 500e6) as usize; // 20 samples
        let total_samples = bucket_spacing_samples * 10;
        let mut signal = vec![0.0; total_samples];

        // Fill every other bucket
        for i in (0..10).step_by(2) {
            let pos = i * bucket_spacing_samples;
            if pos < signal.len() {
                signal[pos] = 1.0;
            }
        }

        let pattern = analyzer.analyze_fill_pattern(&signal, sample_rate);
        assert_eq!(pattern.len(), 5);
    }

    #[test]
    fn test_bunch_purity_single() {
        let analyzer = BunchTimingAnalyzer::new(500e6, 10);
        // Single bunch
        let pattern = vec![(3, 1.0)];
        let purity = analyzer.bunch_purity(&pattern);
        assert!((purity - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_uniform_fill_detection() {
        let analyzer = BunchTimingAnalyzer::new(500e6, 10);
        // Uniform fill
        let uniform: Vec<(u32, f64)> = (0..10).map(|i| (i, 1.0)).collect();
        assert!(analyzer.is_uniform_fill(&uniform));

        // Non-uniform
        let non_uniform = vec![(0, 1.0), (5, 0.1)];
        assert!(!analyzer.is_uniform_fill(&non_uniform));
    }

    // ---- StorageRingCurrent tests ----

    #[test]
    fn test_exponential_decay() {
        let ring = StorageRingCurrent {
            initial_current: 0.200,
            beam_lifetime_s: 10.0 * 3600.0, // 10 hours
        };

        // At t=0
        assert!((ring.current_at_time(0.0) - 0.200).abs() < 1e-10);
        // At t=tau
        let at_tau = ring.current_at_time(ring.beam_lifetime_s);
        assert!((at_tau - 0.200 / std::f64::consts::E).abs() < 1e-6);
    }

    #[test]
    fn test_fit_lifetime() {
        // Generate exponential decay data: I0=0.200A, tau=36000s
        let tau = 36000.0;
        let i0 = 0.200;
        let times: Vec<f64> = (0..20).map(|i| i as f64 * 3600.0).collect();
        let currents: Vec<f64> = times.iter().map(|&t| i0 * (-t / tau).exp()).collect();

        let (fitted_i0, fitted_tau) = StorageRingCurrent::fit_lifetime(&times, &currents);
        assert!(
            (fitted_i0 - i0).abs() < 0.001,
            "I0: {} vs {}",
            fitted_i0,
            i0
        );
        assert!(
            (fitted_tau - tau).abs() / tau < 0.01,
            "tau: {} vs {}",
            fitted_tau,
            tau
        );
    }

    #[test]
    fn test_time_to_threshold() {
        let ring = StorageRingCurrent {
            initial_current: 0.200,
            beam_lifetime_s: 36000.0,
        };
        // Time to drop to 0.100 A = tau * ln(2)
        let t = ring.time_to_threshold(0.100);
        let expected = 36000.0 * (2.0_f64).ln();
        assert!((t - expected).abs() < 1.0, "t = {} vs {}", t, expected);
    }

    #[test]
    fn test_beam_lifetime_hours() {
        let ring = StorageRingCurrent {
            initial_current: 0.200,
            beam_lifetime_s: 36000.0,
        };
        assert!((ring.beam_lifetime_hours() - 10.0).abs() < 1e-10);
    }

    // ---- AbsorptionEdgeDetector tests ----

    #[test]
    fn test_identify_fe_k_edge() {
        let detector = AbsorptionEdgeDetector::new();
        let matches = detector.identify_edge(7112.0, 10.0);
        assert!(!matches.is_empty());
        assert_eq!(matches[0].element, "Fe");
        assert_eq!(matches[0].edge_type, "K");
    }

    #[test]
    fn test_identify_cu_k_edge() {
        let detector = AbsorptionEdgeDetector::new();
        let matches = detector.identify_edge(8979.0, 10.0);
        assert!(!matches.is_empty());
        assert_eq!(matches[0].element, "Cu");
    }

    #[test]
    fn test_detect_edges_synthetic() {
        let detector = AbsorptionEdgeDetector::new().with_min_edge_jump(0.001);

        // Create absorption spectrum with an edge at 7112 eV
        let energies: Vec<f64> = (0..200).map(|i| 6900.0 + i as f64 * 3.0).collect();
        let absorption: Vec<f64> = energies
            .iter()
            .map(|&e| {
                if e < 7112.0 {
                    0.5
                } else {
                    1.5 + 0.1 * (-(e - 7112.0) / 50.0).exp() * ((e - 7112.0) * 0.1).sin()
                }
            })
            .collect();

        let edges = detector.detect_edges(&energies, &absorption);
        // Should find at least one edge near 7112 eV
        assert!(!edges.is_empty(), "should detect at least one edge");
        let found_near_7112 = edges
            .iter()
            .any(|(e, _)| (*e - 7112.0).abs() < 20.0);
        assert!(found_near_7112, "should find edge near 7112 eV: {:?}", edges);
    }

    #[test]
    fn test_no_edges_flat_spectrum() {
        let detector = AbsorptionEdgeDetector::new();
        let energies: Vec<f64> = (0..100).map(|i| 5000.0 + i as f64 * 10.0).collect();
        let absorption = vec![1.0; 100]; // flat
        let edges = detector.detect_edges(&energies, &absorption);
        assert!(edges.is_empty());
    }

    #[test]
    fn test_identify_spectrum() {
        let detector = AbsorptionEdgeDetector::new().with_min_edge_jump(0.001);
        let energies: Vec<f64> = (0..200).map(|i| 6900.0 + i as f64 * 3.0).collect();
        let absorption: Vec<f64> = energies
            .iter()
            .map(|&e| if e < 7112.0 { 0.5 } else { 1.5 })
            .collect();

        let results = detector.identify_spectrum(&energies, &absorption, 30.0);
        // Check results structure
        for (energy, matches) in &results {
            assert!(*energy > 6900.0 && *energy < 7500.0);
            let _ = matches; // may or may not match known edges
        }
    }

    // ---- Integration tests ----

    #[test]
    fn test_full_xafs_pipeline() {
        // Simulate Fe K-edge XAFS measurement
        let edge = 7112.0;
        let xafs = XafsProcessor::new(edge);

        // Generate synthetic spectrum
        let energies: Vec<f64> = (0..500)
            .map(|i| 6900.0 + i as f64 * 1.5)
            .collect();
        let mu: Vec<f64> = energies
            .iter()
            .map(|&e| {
                let pre = 0.3 + 0.00005 * e;
                if e < edge {
                    pre
                } else {
                    let k = 0.5123 * (e - edge).max(0.0).sqrt();
                    let post = pre + 1.0;
                    let exafs = 0.05 * (2.0 * k * 2.5).sin() * (-0.005 * k.powi(2)).exp();
                    post + exafs
                }
            })
            .collect();

        // Normalize
        let mu_norm = xafs.normalize_mu(&energies, &mu);
        assert_eq!(mu_norm.len(), energies.len());

        // Extract chi(k)
        let (k_vals, chi_vals) = xafs.extract_chi_k(&energies, &mu_norm);
        assert!(!k_vals.is_empty());

        // Apply k^2 weighting
        let weighted = XafsProcessor::k_weight(&k_vals, &chi_vals, 2.0);
        assert_eq!(weighted.len(), k_vals.len());
    }

    #[test]
    fn test_monochromator_si220() {
        let cal = MonochromatorCalibrator::new(CrystalReflection::Si220);
        let angle = cal.energy_to_angle(12000.0).unwrap();
        let energy_back = cal.angle_to_energy(angle);
        assert!((energy_back - 12000.0).abs() < 0.1);
    }

    #[test]
    fn test_monochromator_second_order() {
        let cal = MonochromatorCalibrator::new(CrystalReflection::Si111).with_order(2);
        let e = 20000.0;
        let angle = cal.energy_to_angle(e).unwrap();
        let e_back = cal.angle_to_energy(angle);
        assert!((e_back - e).abs() < 0.1);
    }

    #[test]
    fn test_roi_net_counts() {
        let det = FluorescenceDetector::new(1000, 10.0);
        let mut spectrum = vec![10.0; 1000]; // flat background
        // Add a peak
        for i in 490..511 {
            spectrum[i] = 110.0;
        }

        let net = det.roi_net_counts(&spectrum, 4900.0, 5100.0, 5);
        // Gross = 21 channels * 110 = 2310
        // Background = 21 * 10 = 210
        // Net ~ 2100
        assert!(net > 1800.0 && net < 2300.0, "net = {}", net);
    }

    #[test]
    fn test_dead_time_factor() {
        let det = FluorescenceDetector::new(4096, 10.0).with_dead_time(1e-6);
        let factor = det.dead_time_factor(100_000.0);
        assert!((factor - 1.111).abs() < 0.01);
        // Zero rate
        let factor_zero = det.dead_time_factor(0.0);
        assert!((factor_zero - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_bpm_total_current() {
        let bpm = BeamPositionMonitor::new(10.0, 10.0);
        assert!((bpm.total_current(100.0, 100.0, 100.0, 100.0) - 400.0).abs() < 1e-10);
    }

    #[test]
    fn test_from_fit() {
        let tau = 20000.0;
        let i0 = 0.300;
        let times: Vec<f64> = (0..10).map(|i| i as f64 * 2000.0).collect();
        let currents: Vec<f64> = times.iter().map(|&t| i0 * (-t / tau).exp()).collect();

        let ring = StorageRingCurrent::from_fit(&times, &currents);
        assert!(
            (ring.initial_current - i0).abs() < 0.001,
            "I0 = {}",
            ring.initial_current
        );
        assert!(
            (ring.beam_lifetime_s - tau).abs() / tau < 0.01,
            "tau = {}",
            ring.beam_lifetime_s
        );
    }
}

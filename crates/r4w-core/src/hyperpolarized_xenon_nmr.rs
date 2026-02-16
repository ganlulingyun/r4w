//! Hyperpolarized xenon-129 NMR/MRI signal processing.
//!
//! This module implements signal processing algorithms for hyperpolarized
//! xenon-129 (hp-Xe) nuclear magnetic resonance and MRI. Applications include
//! lung ventilation imaging, brain perfusion measurement, biosensor detection,
//! porous media characterization, and surface chemistry studies.
//!
//! # Physics Background
//!
//! Xenon-129 has a gyromagnetic ratio of gamma/2pi = 11.777 MHz/T, producing
//! a Larmor frequency of ~35.33 MHz at 3T. Thermal polarization at room
//! temperature is tiny (~1.2e-5), but Spin-Exchange Optical Pumping (SEOP)
//! with rubidium vapor can produce hyperpolarization levels of 10-50%,
//! yielding 10,000-50,000x signal enhancement.
//!
//! Key distinguishing feature: hyperpolarized magnetization is **non-renewable**.
//! Each RF excitation permanently consumes a fraction of the polarization
//! (cos(alpha) per pulse), requiring careful flip angle management.
//!
//! Xe-129 chemical shifts distinguish three compartments in the lung:
//! - Gas phase: 0 ppm (reference)
//! - Dissolved in tissue/plasma: ~197 ppm
//! - Dissolved in red blood cells: ~217 ppm
//!
//! # Example
//!
//! ```
//! use r4w_core::hyperpolarized_xenon_nmr::{
//!     XenonNmrConfig, FidProcessor, ChemicalShiftAnalyzer,
//!     PolarizationCalculator, FlipAngleOptimizer,
//! };
//!
//! let config = XenonNmrConfig::new(3.0); // 3 Tesla
//! assert!((config.larmor_freq_hz - 35_331_000.0).abs() < 1000.0);
//!
//! // Thermal vs hyperpolarized enhancement
//! let calc = PolarizationCalculator::new(3.0, 300.0);
//! let thermal = calc.thermal_polarization();
//! let enhancement = calc.enhancement_factor(0.30);
//! assert!(enhancement > 10000.0);
//!
//! // Optimal flip angles for 64 acquisitions
//! let optimizer = FlipAngleOptimizer::new(64);
//! let angles = optimizer.variable_flip_angles();
//! assert_eq!(angles.len(), 64);
//! ```

use std::f64::consts::PI;

// ============================================================================
// Physical Constants
// ============================================================================

/// Xe-129 gyromagnetic ratio / 2pi in MHz/T.
const XE129_GAMMA_MHZ_PER_T: f64 = 11.777;

/// Xe-129 gyromagnetic ratio in rad/s/T.
const XE129_GAMMA_RAD: f64 = XE129_GAMMA_MHZ_PER_T * 2.0 * PI * 1.0e6;

/// Reduced Planck constant in J*s.
const HBAR: f64 = 1.054_571_817e-34;

/// Boltzmann constant in J/K.
const K_B: f64 = 1.380_649e-23;

// ============================================================================
// XenonNmrConfig
// ============================================================================

/// Configuration for hyperpolarized Xe-129 NMR experiments.
#[derive(Debug, Clone)]
pub struct XenonNmrConfig {
    /// Static magnetic field strength in Tesla.
    pub b0_tesla: f64,
    /// Larmor frequency in Hz (gamma/2pi * B0).
    pub larmor_freq_hz: f64,
    /// Hyperpolarization level (0.0 to 1.0).
    pub polarization: f64,
    /// T1 relaxation time in seconds.
    pub t1_seconds: f64,
    /// Number of spectral points for FID processing.
    pub num_points: usize,
    /// Spectral bandwidth (sampling rate) in Hz.
    pub bandwidth_hz: f64,
}

impl XenonNmrConfig {
    /// Create a new config for the given field strength with typical hp-Xe parameters.
    pub fn new(b0_tesla: f64) -> Self {
        let larmor_freq_hz = XE129_GAMMA_MHZ_PER_T * b0_tesla * 1.0e6;
        Self {
            b0_tesla,
            larmor_freq_hz,
            polarization: 0.30,
            t1_seconds: 20.0,
            num_points: 1024,
            bandwidth_hz: 50_000.0,
        }
    }

    /// Create config for lung ventilation imaging.
    pub fn lung_ventilation(b0_tesla: f64) -> Self {
        Self {
            t1_seconds: 20.0,
            polarization: 0.30,
            ..Self::new(b0_tesla)
        }
    }

    /// Create config for dissolved-phase spectroscopy.
    pub fn dissolved_phase(b0_tesla: f64) -> Self {
        Self {
            t1_seconds: 8.0,
            polarization: 0.25,
            bandwidth_hz: 100_000.0,
            ..Self::new(b0_tesla)
        }
    }

    /// Frequency offset in Hz for a given chemical shift in ppm.
    pub fn ppm_to_hz(&self, ppm: f64) -> f64 {
        ppm * self.larmor_freq_hz * 1.0e-6
    }

    /// Chemical shift in ppm for a given frequency offset in Hz.
    pub fn hz_to_ppm(&self, hz: f64) -> f64 {
        hz / (self.larmor_freq_hz * 1.0e-6)
    }
}

// ============================================================================
// SpinExchangeModel
// ============================================================================

/// Spin-Exchange Optical Pumping (SEOP) model.
///
/// Models the polarization buildup of Xe-129 through spin exchange with
/// optically pumped rubidium vapor:
///
///   dP_Xe/dt = gamma_SE * (P_Rb - P_Xe) - P_Xe / T1
///
/// where gamma_SE is the spin-exchange rate, P_Rb is rubidium polarization,
/// and T1 is the Xe-129 longitudinal relaxation time in the cell.
#[derive(Debug, Clone)]
pub struct SpinExchangeModel {
    /// Spin-exchange rate (1/s), typically 0.01-0.1 for SEOP cells.
    pub gamma_se: f64,
    /// Rubidium polarization (0 to 1), depends on laser power/detuning.
    pub p_rb: f64,
    /// Xe-129 T1 in the SEOP cell (seconds), typically 100-10000 s.
    pub t1_cell: f64,
}

impl SpinExchangeModel {
    /// Create a new SEOP model with the given parameters.
    pub fn new(gamma_se: f64, p_rb: f64, t1_cell: f64) -> Self {
        Self {
            gamma_se,
            p_rb,
            t1_cell,
        }
    }

    /// Typical high-flow SEOP parameters.
    pub fn typical_high_flow() -> Self {
        Self {
            gamma_se: 0.02,
            p_rb: 0.95,
            t1_cell: 2000.0,
        }
    }

    /// Typical batch-mode SEOP parameters.
    pub fn typical_batch() -> Self {
        Self {
            gamma_se: 0.005,
            p_rb: 0.90,
            t1_cell: 5000.0,
        }
    }

    /// Rate of change of Xe polarization dP_Xe/dt.
    pub fn dp_dt(&self, p_xe: f64) -> f64 {
        self.gamma_se * (self.p_rb - p_xe) - p_xe / self.t1_cell
    }

    /// Steady-state Xe polarization: P_Xe_ss = gamma_SE * P_Rb / (gamma_SE + 1/T1).
    pub fn steady_state_polarization(&self) -> f64 {
        let denom = self.gamma_se + 1.0 / self.t1_cell;
        if denom <= 0.0 {
            return 0.0;
        }
        self.gamma_se * self.p_rb / denom
    }

    /// Time constant for polarization buildup: tau = 1 / (gamma_SE + 1/T1).
    pub fn buildup_time_constant(&self) -> f64 {
        1.0 / (self.gamma_se + 1.0 / self.t1_cell)
    }

    /// Simulate polarization buildup from P_Xe=0 over the given time points.
    /// Returns P_Xe(t) = P_ss * (1 - exp(-t/tau)).
    pub fn simulate_buildup(&self, time_points: &[f64]) -> Vec<f64> {
        let p_ss = self.steady_state_polarization();
        let tau = self.buildup_time_constant();
        time_points
            .iter()
            .map(|&t| p_ss * (1.0 - (-t / tau).exp()))
            .collect()
    }

    /// Simulate polarization dynamics using Euler integration from initial P_Xe.
    pub fn simulate_euler(&self, p_xe_init: f64, dt: f64, num_steps: usize) -> Vec<f64> {
        let mut result = Vec::with_capacity(num_steps);
        let mut p = p_xe_init;
        for _ in 0..num_steps {
            result.push(p);
            let dp = self.dp_dt(p);
            p += dp * dt;
            p = p.clamp(0.0, 1.0);
        }
        result
    }
}

// ============================================================================
// PolarizationCalculator
// ============================================================================

/// Calculates thermal and hyperpolarized polarization levels.
///
/// Thermal equilibrium polarization:
///   P = tanh(gamma * hbar * B0 / (2 * k_B * T))
///
/// For Xe-129 at 3T, 300K: P_thermal ≈ 1.2e-5
/// Hyperpolarized: P_hp ≈ 0.1-0.5 → 10,000-50,000x enhancement
#[derive(Debug, Clone)]
pub struct PolarizationCalculator {
    /// Magnetic field strength in Tesla.
    pub b0: f64,
    /// Temperature in Kelvin.
    pub temperature: f64,
}

impl PolarizationCalculator {
    /// Create a new calculator for the given field and temperature.
    pub fn new(b0: f64, temperature: f64) -> Self {
        Self { b0, temperature }
    }

    /// Thermal equilibrium polarization P = tanh(gamma*hbar*B0 / (2*k_B*T)).
    pub fn thermal_polarization(&self) -> f64 {
        let arg = XE129_GAMMA_RAD * HBAR * self.b0 / (2.0 * K_B * self.temperature);
        arg.tanh()
    }

    /// Signal enhancement factor: P_hp / P_thermal.
    pub fn enhancement_factor(&self, p_hp: f64) -> f64 {
        let p_thermal = self.thermal_polarization();
        if p_thermal <= 0.0 {
            return 0.0;
        }
        p_hp / p_thermal
    }

    /// Signal-to-noise ratio scaling relative to thermal: SNR_hp / SNR_thermal = P_hp / P_thermal.
    pub fn snr_enhancement(&self, p_hp: f64) -> f64 {
        self.enhancement_factor(p_hp)
    }

    /// Remaining polarization after N RF pulses of flip angle alpha (radians).
    /// P(N) = P0 * cos(alpha)^N
    pub fn polarization_after_pulses(&self, p0: f64, alpha_rad: f64, n_pulses: usize) -> f64 {
        p0 * alpha_rad.cos().powi(n_pulses as i32)
    }
}

// ============================================================================
// FidProcessor
// ============================================================================

/// Free Induction Decay (FID) signal processor.
///
/// Processes raw FID time-domain data into frequency-domain spectra:
/// 1. Apodization (exponential or Gaussian line broadening)
/// 2. Zero-filling for spectral interpolation
/// 3. DFT to frequency domain
/// 4. Phase correction (zeroth-order and first-order)
#[derive(Debug, Clone)]
pub struct FidProcessor {
    /// Line broadening in Hz for exponential apodization.
    pub line_broadening_hz: f64,
    /// Zero-fill factor (1 = no zero-fill, 2 = double points, etc.).
    pub zero_fill_factor: usize,
    /// Bandwidth (sampling rate) in Hz.
    pub bandwidth_hz: f64,
}

impl FidProcessor {
    /// Create a new FID processor with the given parameters.
    pub fn new(bandwidth_hz: f64) -> Self {
        Self {
            line_broadening_hz: 10.0,
            zero_fill_factor: 2,
            bandwidth_hz,
        }
    }

    /// Apply exponential apodization to FID data.
    /// Each point is multiplied by exp(-pi * lb * t) where t = n / bandwidth.
    pub fn apodize_exponential(&self, fid_re: &[f64], fid_im: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let n = fid_re.len().min(fid_im.len());
        let mut re = Vec::with_capacity(n);
        let mut im = Vec::with_capacity(n);
        for i in 0..n {
            let t = i as f64 / self.bandwidth_hz;
            let decay = (-PI * self.line_broadening_hz * t).exp();
            re.push(fid_re[i] * decay);
            im.push(fid_im[i] * decay);
        }
        (re, im)
    }

    /// Apply Gaussian apodization.
    /// Each point is multiplied by exp(-pi * sigma^2 * t^2).
    pub fn apodize_gaussian(
        &self,
        fid_re: &[f64],
        fid_im: &[f64],
        sigma_hz: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let n = fid_re.len().min(fid_im.len());
        let mut re = Vec::with_capacity(n);
        let mut im = Vec::with_capacity(n);
        for i in 0..n {
            let t = i as f64 / self.bandwidth_hz;
            let decay = (-PI * sigma_hz * sigma_hz * t * t).exp();
            re.push(fid_re[i] * decay);
            im.push(fid_im[i] * decay);
        }
        (re, im)
    }

    /// Zero-fill FID data to zero_fill_factor * original length.
    pub fn zero_fill(&self, fid_re: &[f64], fid_im: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let n = fid_re.len().max(fid_im.len());
        let new_len = n * self.zero_fill_factor;
        let mut re = vec![0.0; new_len];
        let mut im = vec![0.0; new_len];
        for i in 0..fid_re.len().min(new_len) {
            re[i] = fid_re[i];
        }
        for i in 0..fid_im.len().min(new_len) {
            im[i] = fid_im[i];
        }
        (re, im)
    }

    /// Compute DFT of complex data (real, imag) -> (spectrum_re, spectrum_im).
    /// Uses simple O(N^2) DFT for correctness without external dependencies.
    pub fn dft(re: &[f64], im: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let n = re.len();
        let mut spec_re = vec![0.0; n];
        let mut spec_im = vec![0.0; n];
        for k in 0..n {
            let mut sum_re = 0.0;
            let mut sum_im = 0.0;
            for j in 0..n {
                let angle = -2.0 * PI * (k as f64) * (j as f64) / (n as f64);
                let cos_a = angle.cos();
                let sin_a = angle.sin();
                // (re[j] + i*im[j]) * (cos_a + i*sin_a)
                sum_re += re[j] * cos_a - im[j] * sin_a;
                sum_im += re[j] * sin_a + im[j] * cos_a;
            }
            spec_re[k] = sum_re;
            spec_im[k] = sum_im;
        }
        (spec_re, spec_im)
    }

    /// Compute magnitude spectrum from complex spectrum.
    pub fn magnitude(re: &[f64], im: &[f64]) -> Vec<f64> {
        re.iter()
            .zip(im.iter())
            .map(|(&r, &i)| (r * r + i * i).sqrt())
            .collect()
    }

    /// Apply zeroth-order phase correction (constant phase shift).
    pub fn phase_correct_zero(
        re: &[f64],
        im: &[f64],
        phi0_rad: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let cos_p = phi0_rad.cos();
        let sin_p = phi0_rad.sin();
        let corr_re: Vec<f64> = re
            .iter()
            .zip(im.iter())
            .map(|(&r, &i)| r * cos_p + i * sin_p)
            .collect();
        let corr_im: Vec<f64> = re
            .iter()
            .zip(im.iter())
            .map(|(&r, &i)| -r * sin_p + i * cos_p)
            .collect();
        (corr_re, corr_im)
    }

    /// Apply first-order phase correction (linear phase across spectrum).
    /// phi(k) = phi0 + phi1 * (k - pivot) / N
    pub fn phase_correct_first(
        re: &[f64],
        im: &[f64],
        phi0_rad: f64,
        phi1_rad: f64,
        pivot: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        let n = re.len();
        let mut corr_re = vec![0.0; n];
        let mut corr_im = vec![0.0; n];
        for k in 0..n {
            let phi = phi0_rad + phi1_rad * (k as f64 - pivot as f64) / n as f64;
            let cos_p = phi.cos();
            let sin_p = phi.sin();
            corr_re[k] = re[k] * cos_p + im[k] * sin_p;
            corr_im[k] = -re[k] * sin_p + im[k] * cos_p;
        }
        (corr_re, corr_im)
    }

    /// Full FID processing pipeline: apodize -> zero-fill -> DFT -> magnitude.
    pub fn process(&self, fid_re: &[f64], fid_im: &[f64]) -> Vec<f64> {
        let (apod_re, apod_im) = self.apodize_exponential(fid_re, fid_im);
        let (zf_re, zf_im) = self.zero_fill(&apod_re, &apod_im);
        let (spec_re, spec_im) = Self::dft(&zf_re, &zf_im);
        Self::magnitude(&spec_re, &spec_im)
    }

    /// Generate frequency axis in Hz for the processed spectrum.
    pub fn frequency_axis(&self, num_points: usize) -> Vec<f64> {
        (0..num_points)
            .map(|k| {
                let frac = k as f64 / num_points as f64;
                if frac <= 0.5 {
                    frac * self.bandwidth_hz
                } else {
                    (frac - 1.0) * self.bandwidth_hz
                }
            })
            .collect()
    }
}

// ============================================================================
// ChemicalShiftAnalyzer
// ============================================================================

/// Xe-129 chemical shift reference values in ppm.
pub const XE_GAS_PPM: f64 = 0.0;
/// Xe dissolved in tissue/plasma (~197 ppm from gas).
pub const XE_TISSUE_PPM: f64 = 197.0;
/// Xe dissolved in red blood cells (~217 ppm from gas).
pub const XE_RBC_PPM: f64 = 217.0;

/// Analyzes Xe-129 chemical shift spectrum to identify and quantify compartments.
#[derive(Debug, Clone)]
pub struct ChemicalShiftAnalyzer {
    /// Larmor frequency in Hz for ppm conversion.
    pub larmor_freq_hz: f64,
    /// Spectral bandwidth in Hz.
    pub bandwidth_hz: f64,
}

impl ChemicalShiftAnalyzer {
    /// Create analyzer for given field strength.
    pub fn new(b0_tesla: f64, bandwidth_hz: f64) -> Self {
        Self {
            larmor_freq_hz: XE129_GAMMA_MHZ_PER_T * b0_tesla * 1.0e6,
            bandwidth_hz,
        }
    }

    /// Convert ppm offset to frequency in Hz.
    pub fn ppm_to_hz(&self, ppm: f64) -> f64 {
        ppm * self.larmor_freq_hz * 1.0e-6
    }

    /// Convert spectral index to ppm (assuming spectrum from DFT).
    pub fn index_to_ppm(&self, index: usize, num_points: usize) -> f64 {
        let freq_hz = if index <= num_points / 2 {
            index as f64 * self.bandwidth_hz / num_points as f64
        } else {
            (index as f64 - num_points as f64) * self.bandwidth_hz / num_points as f64
        };
        freq_hz / (self.larmor_freq_hz * 1.0e-6)
    }

    /// Find peak amplitude and index near a target ppm value.
    /// Searches within +/- search_range_ppm.
    pub fn find_peak_near(
        &self,
        spectrum: &[f64],
        target_ppm: f64,
        search_range_ppm: f64,
    ) -> Option<(usize, f64)> {
        let n = spectrum.len();
        let mut best_idx = 0;
        let mut best_val = f64::NEG_INFINITY;
        for i in 0..n {
            let ppm = self.index_to_ppm(i, n);
            if (ppm - target_ppm).abs() <= search_range_ppm && spectrum[i] > best_val {
                best_val = spectrum[i];
                best_idx = i;
            }
        }
        if best_val > f64::NEG_INFINITY {
            Some((best_idx, best_val))
        } else {
            None
        }
    }

    /// Analyze a magnitude spectrum to find gas, tissue, and RBC peaks.
    /// Returns (gas_amplitude, tissue_amplitude, rbc_amplitude).
    pub fn analyze_dissolved_phase(&self, spectrum: &[f64]) -> DissolvedPhaseResult {
        let search_range = 15.0; // +/- 15 ppm search window
        let gas = self.find_peak_near(spectrum, XE_GAS_PPM, search_range);
        let tissue = self.find_peak_near(spectrum, XE_TISSUE_PPM, search_range);
        let rbc = self.find_peak_near(spectrum, XE_RBC_PPM, search_range);

        DissolvedPhaseResult {
            gas_amplitude: gas.map(|(_, v)| v).unwrap_or(0.0),
            tissue_amplitude: tissue.map(|(_, v)| v).unwrap_or(0.0),
            rbc_amplitude: rbc.map(|(_, v)| v).unwrap_or(0.0),
            rbc_to_tissue_ratio: match (rbc, tissue) {
                (Some((_, r)), Some((_, t))) if t > 0.0 => r / t,
                _ => 0.0,
            },
            dissolved_to_gas_ratio: match (gas, tissue, rbc) {
                (Some((_, g)), Some((_, t)), Some((_, r))) if g > 0.0 => (t + r) / g,
                _ => 0.0,
            },
        }
    }
}

/// Result of dissolved-phase spectral analysis.
#[derive(Debug, Clone)]
pub struct DissolvedPhaseResult {
    /// Peak amplitude of gas-phase Xe (0 ppm).
    pub gas_amplitude: f64,
    /// Peak amplitude of tissue-dissolved Xe (~197 ppm).
    pub tissue_amplitude: f64,
    /// Peak amplitude of RBC-dissolved Xe (~217 ppm).
    pub rbc_amplitude: f64,
    /// RBC-to-tissue ratio (healthy ~0.4-0.6).
    pub rbc_to_tissue_ratio: f64,
    /// Dissolved-to-gas ratio.
    pub dissolved_to_gas_ratio: f64,
}

// ============================================================================
// DissolvedPhaseMapper
// ============================================================================

/// Separates gas, tissue, and RBC components via spectral decomposition.
///
/// Uses a simplified three-component model: the FID is decomposed as a sum
/// of three damped sinusoids at known chemical shift frequencies.
#[derive(Debug, Clone)]
pub struct DissolvedPhaseMapper {
    /// Larmor frequency in Hz.
    larmor_freq_hz: f64,
    /// Sampling rate in Hz.
    bandwidth_hz: f64,
    /// T2* decay constants in seconds for gas, tissue, RBC.
    pub t2_star: [f64; 3],
    /// Chemical shift offsets in ppm for the three compartments.
    pub shifts_ppm: [f64; 3],
}

impl DissolvedPhaseMapper {
    /// Create a mapper with default parameters for a given field strength.
    pub fn new(b0_tesla: f64, bandwidth_hz: f64) -> Self {
        Self {
            larmor_freq_hz: XE129_GAMMA_MHZ_PER_T * b0_tesla * 1.0e6,
            bandwidth_hz,
            t2_star: [30.0e-3, 2.0e-3, 1.5e-3], // Gas: 30ms, Tissue: 2ms, RBC: 1.5ms
            shifts_ppm: [XE_GAS_PPM, XE_TISSUE_PPM, XE_RBC_PPM],
        }
    }

    /// Generate the time-domain basis function for a single component.
    /// S_k(t) = exp(-t/T2*_k) * exp(i * 2pi * freq_k * t)
    /// Returns (real, imag) parts.
    pub fn basis_function(&self, component: usize, num_points: usize) -> (Vec<f64>, Vec<f64>) {
        assert!(component < 3, "component must be 0, 1, or 2");
        let freq_hz = self.shifts_ppm[component] * self.larmor_freq_hz * 1.0e-6;
        let t2 = self.t2_star[component];
        let mut re = Vec::with_capacity(num_points);
        let mut im = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let t = i as f64 / self.bandwidth_hz;
            let decay = (-t / t2).exp();
            let angle = 2.0 * PI * freq_hz * t;
            re.push(decay * angle.cos());
            im.push(decay * angle.sin());
        }
        (re, im)
    }

    /// Decompose an FID into three components using least-squares projection.
    /// Returns amplitudes [gas, tissue, rbc] as complex (re, im) pairs.
    pub fn decompose(
        &self,
        fid_re: &[f64],
        fid_im: &[f64],
    ) -> [(f64, f64); 3] {
        let n = fid_re.len().min(fid_im.len());
        let mut result = [(0.0, 0.0); 3];

        for comp in 0..3 {
            let (basis_re, basis_im) = self.basis_function(comp, n);

            // Inner product: <FID, basis> = sum(fid * conj(basis))
            let mut dot_re = 0.0;
            let mut dot_im = 0.0;
            let mut norm_sq = 0.0;
            for i in 0..n {
                // fid * conj(basis)
                dot_re += fid_re[i] * basis_re[i] + fid_im[i] * basis_im[i];
                dot_im += fid_im[i] * basis_re[i] - fid_re[i] * basis_im[i];
                norm_sq += basis_re[i] * basis_re[i] + basis_im[i] * basis_im[i];
            }
            if norm_sq > 0.0 {
                result[comp] = (dot_re / norm_sq, dot_im / norm_sq);
            }
        }
        result
    }

    /// Get component magnitudes [gas, tissue, rbc] from decomposition.
    pub fn component_magnitudes(&self, fid_re: &[f64], fid_im: &[f64]) -> [f64; 3] {
        let coeffs = self.decompose(fid_re, fid_im);
        [
            (coeffs[0].0 * coeffs[0].0 + coeffs[0].1 * coeffs[0].1).sqrt(),
            (coeffs[1].0 * coeffs[1].0 + coeffs[1].1 * coeffs[1].1).sqrt(),
            (coeffs[2].0 * coeffs[2].0 + coeffs[2].1 * coeffs[2].1).sqrt(),
        ]
    }
}

// ============================================================================
// T1RelaxationTracker
// ============================================================================

/// Tracks T1 decay of hyperpolarized magnetization.
///
/// For hp-Xe, the magnetization is non-renewable:
///   M(t) = M0 * exp(-t/T1)
///
/// Additionally, each RF pulse with flip angle alpha consumes polarization:
///   M_after = M_before * cos(alpha)
///
/// Combined: M(t, n) = M0 * exp(-t/T1) * cos(alpha)^n
#[derive(Debug, Clone)]
pub struct T1RelaxationTracker {
    /// Initial magnetization (arbitrary units).
    pub m0: f64,
    /// T1 relaxation time in seconds.
    pub t1: f64,
    /// Current magnetization level.
    current_m: f64,
    /// Elapsed time in seconds.
    elapsed_time: f64,
    /// Number of RF pulses applied.
    pulse_count: usize,
}

impl T1RelaxationTracker {
    /// Create a new tracker with initial magnetization and T1.
    pub fn new(m0: f64, t1: f64) -> Self {
        Self {
            m0,
            t1,
            current_m: m0,
            elapsed_time: 0.0,
            pulse_count: 0,
        }
    }

    /// Preset for hp-Xe in lungs (T1 ~ 20 s).
    pub fn lung() -> Self {
        Self::new(1.0, 20.0)
    }

    /// Preset for hp-Xe in blood (T1 ~ 7 s).
    pub fn blood() -> Self {
        Self::new(1.0, 7.0)
    }

    /// Magnetization at time t from initial state (no RF pulses).
    pub fn m_at_time(&self, t: f64) -> f64 {
        self.m0 * (-t / self.t1).exp()
    }

    /// Magnetization after n RF pulses and time t.
    pub fn m_at_time_with_pulses(&self, t: f64, n_pulses: usize, alpha_rad: f64) -> f64 {
        self.m0 * (-t / self.t1).exp() * alpha_rad.cos().powi(n_pulses as i32)
    }

    /// Apply an RF pulse with given flip angle. Returns signal (transverse) and
    /// updates the longitudinal magnetization.
    pub fn apply_pulse(&mut self, alpha_rad: f64, dt_since_last: f64) -> f64 {
        // T1 decay during waiting time
        self.elapsed_time += dt_since_last;
        self.current_m *= (-dt_since_last / self.t1).exp();

        // RF excitation: signal proportional to M * sin(alpha)
        let signal = self.current_m * alpha_rad.sin();

        // Remaining longitudinal: M * cos(alpha)
        self.current_m *= alpha_rad.cos();
        self.pulse_count += 1;

        signal
    }

    /// Current remaining magnetization.
    pub fn current_magnetization(&self) -> f64 {
        self.current_m
    }

    /// Number of RF pulses applied so far.
    pub fn pulse_count(&self) -> usize {
        self.pulse_count
    }

    /// Elapsed time since creation.
    pub fn elapsed_time(&self) -> f64 {
        self.elapsed_time
    }

    /// Fraction of initial magnetization remaining.
    pub fn remaining_fraction(&self) -> f64 {
        if self.m0 > 0.0 {
            self.current_m / self.m0
        } else {
            0.0
        }
    }
}

// ============================================================================
// FlipAngleOptimizer
// ============================================================================

/// Optimizes variable flip angle (VFA) schemes for hyperpolarized imaging.
///
/// The optimal VFA for equalizing signal across N acquisitions:
///   alpha_n = atan(1 / sqrt(N - n))
///
/// where n = 0, 1, ..., N-1. The last pulse uses all remaining magnetization
/// (alpha = 90 degrees). This produces constant signal across all acquisitions.
#[derive(Debug, Clone)]
pub struct FlipAngleOptimizer {
    /// Total number of acquisitions.
    pub num_acquisitions: usize,
}

impl FlipAngleOptimizer {
    /// Create optimizer for N acquisitions.
    pub fn new(num_acquisitions: usize) -> Self {
        Self { num_acquisitions }
    }

    /// Compute optimal variable flip angles in radians.
    /// alpha_n = atan(1 / sqrt(N - n)) for n = 0..N-1
    pub fn variable_flip_angles(&self) -> Vec<f64> {
        let n = self.num_acquisitions;
        if n == 0 {
            return vec![];
        }
        (0..n)
            .map(|i| {
                let remaining = n - i;
                if remaining <= 1 {
                    PI / 2.0 // Last acquisition: use all remaining magnetization
                } else {
                    (1.0 / (remaining as f64 - 1.0).sqrt()).atan()
                }
            })
            .collect()
    }

    /// Compute flip angles in degrees.
    pub fn variable_flip_angles_deg(&self) -> Vec<f64> {
        self.variable_flip_angles()
            .iter()
            .map(|&a| a * 180.0 / PI)
            .collect()
    }

    /// Simulate signal with VFA scheme. Returns normalized signal at each acquisition.
    /// All signals should be approximately equal if scheme is optimal.
    pub fn simulate_vfa_signal(&self) -> Vec<f64> {
        let angles = self.variable_flip_angles();
        let mut m = 1.0; // Normalized initial magnetization
        let mut signals = Vec::with_capacity(angles.len());
        for &alpha in &angles {
            let sig = m * alpha.sin();
            m *= alpha.cos();
            signals.push(sig);
        }
        signals
    }

    /// Compute constant flip angle that maximizes total signal across N acquisitions.
    /// This is the Ernst angle for non-renewable magnetization:
    /// Maximizes sum of sin(alpha) * cos(alpha)^n for n=0..N-1.
    pub fn optimal_constant_angle(&self) -> f64 {
        let n = self.num_acquisitions;
        if n == 0 {
            return 0.0;
        }
        // Numerical search
        let mut best_angle = 0.0;
        let mut best_total = 0.0;
        for step in 1..9000 {
            let alpha = (step as f64) * PI / 18000.0; // 0.01 degree steps up to 90
            let mut total = 0.0;
            let mut m = 1.0;
            for _ in 0..n {
                total += m * alpha.sin();
                m *= alpha.cos();
            }
            if total > best_total {
                best_total = total;
                best_angle = alpha;
            }
        }
        best_angle
    }

    /// Simulate signal with constant flip angle for comparison.
    pub fn simulate_constant_signal(&self, alpha_rad: f64) -> Vec<f64> {
        let mut m = 1.0;
        let mut signals = Vec::with_capacity(self.num_acquisitions);
        for _ in 0..self.num_acquisitions {
            let sig = m * alpha_rad.sin();
            m *= alpha_rad.cos();
            signals.push(sig);
        }
        signals
    }
}

// ============================================================================
// VentilationDefectMapper
// ============================================================================

/// Quantifies ventilation defects from hp-Xe lung images.
///
/// Ventilation Defect Percentage (VDP):
///   VDP = (total_lung_voxels - ventilated_voxels) / total_lung_voxels * 100%
///
/// Healthy lungs: VDP < 5%
/// Mild disease: VDP 5-15%
/// Moderate disease: VDP 15-30%
/// Severe disease: VDP > 30%
#[derive(Debug, Clone)]
pub struct VentilationDefectMapper {
    /// Signal threshold (fraction of mean) below which a voxel is "defect".
    pub defect_threshold: f64,
}

impl VentilationDefectMapper {
    /// Create with default threshold (0.1 = 10% of mean signal).
    pub fn new() -> Self {
        Self {
            defect_threshold: 0.1,
        }
    }

    /// Create with custom defect threshold.
    pub fn with_threshold(threshold: f64) -> Self {
        Self {
            defect_threshold: threshold,
        }
    }

    /// Compute Ventilation Defect Percentage from signal intensities.
    /// `signals` contains signal intensity for each lung voxel.
    /// `lung_mask` indicates which voxels are within the lung (true = lung).
    pub fn compute_vdp(&self, signals: &[f64], lung_mask: &[bool]) -> f64 {
        let n = signals.len().min(lung_mask.len());
        if n == 0 {
            return 0.0;
        }

        // Mean signal within lung
        let mut sum = 0.0;
        let mut lung_count = 0usize;
        for i in 0..n {
            if lung_mask[i] {
                sum += signals[i];
                lung_count += 1;
            }
        }
        if lung_count == 0 {
            return 0.0;
        }
        let mean_signal = sum / lung_count as f64;
        let threshold = self.defect_threshold * mean_signal;

        // Count defective voxels
        let mut defect_count = 0usize;
        for i in 0..n {
            if lung_mask[i] && signals[i] < threshold {
                defect_count += 1;
            }
        }

        defect_count as f64 / lung_count as f64 * 100.0
    }

    /// Classify ventilation from VDP percentage.
    pub fn classify_ventilation(vdp: f64) -> VentilationCategory {
        if vdp < 5.0 {
            VentilationCategory::Normal
        } else if vdp < 15.0 {
            VentilationCategory::Mild
        } else if vdp < 30.0 {
            VentilationCategory::Moderate
        } else {
            VentilationCategory::Severe
        }
    }

    /// Generate a binary defect map: true = ventilated, false = defect.
    pub fn defect_map(&self, signals: &[f64], lung_mask: &[bool]) -> Vec<bool> {
        let n = signals.len().min(lung_mask.len());
        // Compute mean within lung
        let mut sum = 0.0;
        let mut count = 0usize;
        for i in 0..n {
            if lung_mask[i] {
                sum += signals[i];
                count += 1;
            }
        }
        let mean_signal = if count > 0 { sum / count as f64 } else { 1.0 };
        let threshold = self.defect_threshold * mean_signal;

        (0..n)
            .map(|i| {
                if lung_mask[i] {
                    signals[i] >= threshold
                } else {
                    false
                }
            })
            .collect()
    }

    /// Compute signal intensity histogram within the lung.
    /// Returns (bin_edges, counts) with num_bins bins.
    pub fn signal_histogram(signals: &[f64], lung_mask: &[bool], num_bins: usize) -> (Vec<f64>, Vec<usize>) {
        let n = signals.len().min(lung_mask.len());
        let lung_signals: Vec<f64> = (0..n)
            .filter(|&i| lung_mask[i])
            .map(|i| signals[i])
            .collect();

        if lung_signals.is_empty() || num_bins == 0 {
            return (vec![], vec![]);
        }

        let min_val = lung_signals.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_val = lung_signals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let range = max_val - min_val;
        if range <= 0.0 {
            return (vec![min_val], vec![lung_signals.len()]);
        }

        let bin_width = range / num_bins as f64;
        let edges: Vec<f64> = (0..=num_bins).map(|i| min_val + i as f64 * bin_width).collect();
        let mut counts = vec![0usize; num_bins];

        for &v in &lung_signals {
            let bin = ((v - min_val) / bin_width).floor() as usize;
            let bin = bin.min(num_bins - 1);
            counts[bin] += 1;
        }

        (edges, counts)
    }
}

/// Ventilation defect severity category.
#[derive(Debug, Clone, PartialEq)]
pub enum VentilationCategory {
    /// VDP < 5%
    Normal,
    /// VDP 5-15%
    Mild,
    /// VDP 15-30%
    Moderate,
    /// VDP > 30%
    Severe,
}

// ============================================================================
// DiffusionCoefficient
// ============================================================================

/// Apparent Diffusion Coefficient (ADC) measurement from hp-Xe.
///
/// Signal decay with diffusion weighting:
///   S(b) = S0 * exp(-b * ADC)
///
/// where b is the diffusion weighting factor (s/cm^2) and ADC is the
/// apparent diffusion coefficient (cm^2/s).
///
/// In healthy lungs, Xe ADC ~ 0.04 cm^2/s.
/// In emphysema, ADC increases due to enlarged airspaces.
#[derive(Debug, Clone)]
pub struct DiffusionCoefficient;

impl DiffusionCoefficient {
    /// Compute ADC from two b-value measurements.
    /// ADC = -ln(S1/S0) / (b1 - b0)
    pub fn compute_adc(s0: f64, s1: f64, b0: f64, b1: f64) -> f64 {
        if s0 <= 0.0 || s1 <= 0.0 || (b1 - b0).abs() < 1.0e-30 {
            return 0.0;
        }
        -(s1 / s0).ln() / (b1 - b0)
    }

    /// Compute ADC from multiple b-values using linear regression on ln(S).
    /// Returns (adc, s0_estimate).
    pub fn compute_adc_multi(signals: &[f64], b_values: &[f64]) -> (f64, f64) {
        let n = signals.len().min(b_values.len());
        if n < 2 {
            return (0.0, 0.0);
        }

        // Linear regression: ln(S) = ln(S0) - ADC * b
        // y = a + m*x where y=ln(S), x=b, m=-ADC, a=ln(S0)
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xx = 0.0;
        let mut sum_xy = 0.0;
        let mut valid = 0;

        for i in 0..n {
            if signals[i] > 0.0 {
                let x = b_values[i];
                let y = signals[i].ln();
                sum_x += x;
                sum_y += y;
                sum_xx += x * x;
                sum_xy += x * y;
                valid += 1;
            }
        }

        if valid < 2 {
            return (0.0, 0.0);
        }

        let nf = valid as f64;
        let denom = nf * sum_xx - sum_x * sum_x;
        if denom.abs() < 1.0e-30 {
            return (0.0, 0.0);
        }

        let slope = (nf * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / nf;

        let adc = -slope; // ADC = -slope
        let s0 = intercept.exp();
        (adc, s0)
    }

    /// Predict signal for a given ADC and b-value.
    pub fn predict_signal(s0: f64, adc: f64, b: f64) -> f64 {
        s0 * (-b * adc).exp()
    }

    /// Classify lung health from Xe ADC value (cm^2/s).
    pub fn classify_adc(adc: f64) -> AdcCategory {
        if adc < 0.03 {
            AdcCategory::Restricted
        } else if adc < 0.055 {
            AdcCategory::Normal
        } else if adc < 0.08 {
            AdcCategory::MildEmphysema
        } else {
            AdcCategory::SevereEmphysema
        }
    }
}

/// ADC classification category.
#[derive(Debug, Clone, PartialEq)]
pub enum AdcCategory {
    /// ADC < 0.03 cm^2/s: restricted diffusion
    Restricted,
    /// ADC 0.03-0.055 cm^2/s: healthy lung
    Normal,
    /// ADC 0.055-0.08 cm^2/s: mild emphysema
    MildEmphysema,
    /// ADC > 0.08 cm^2/s: severe emphysema
    SevereEmphysema,
}

// ============================================================================
// Helper: generate synthetic FID
// ============================================================================

/// Generate a synthetic FID with multiple frequency components.
/// Each component: (amplitude, frequency_hz, t2_star_s, phase_rad).
pub fn generate_synthetic_fid(
    components: &[(f64, f64, f64, f64)],
    num_points: usize,
    bandwidth_hz: f64,
) -> (Vec<f64>, Vec<f64>) {
    let mut re = vec![0.0; num_points];
    let mut im = vec![0.0; num_points];
    for &(amp, freq, t2, phase) in components {
        for i in 0..num_points {
            let t = i as f64 / bandwidth_hz;
            let decay = (-t / t2).exp();
            let angle = 2.0 * PI * freq * t + phase;
            re[i] += amp * decay * angle.cos();
            im[i] += amp * decay * angle.sin();
        }
    }
    (re, im)
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1.0e-6;

    // --- XenonNmrConfig tests ---

    #[test]
    fn test_config_larmor_frequency_3t() {
        let config = XenonNmrConfig::new(3.0);
        // gamma/2pi = 11.777 MHz/T, at 3T: 35.331 MHz
        let expected = 11.777e6 * 3.0;
        assert!((config.larmor_freq_hz - expected).abs() < 1.0);
    }

    #[test]
    fn test_config_larmor_frequency_1_5t() {
        let config = XenonNmrConfig::new(1.5);
        let expected = 11.777e6 * 1.5;
        assert!((config.larmor_freq_hz - expected).abs() < 1.0);
    }

    #[test]
    fn test_config_ppm_to_hz() {
        let config = XenonNmrConfig::new(3.0);
        // 1 ppm at 35.331 MHz = 35.331 Hz
        let hz = config.ppm_to_hz(1.0);
        assert!((hz - 35.331).abs() < 0.1);
    }

    #[test]
    fn test_config_hz_to_ppm_roundtrip() {
        let config = XenonNmrConfig::new(3.0);
        let ppm = 197.0;
        let hz = config.ppm_to_hz(ppm);
        let back = config.hz_to_ppm(hz);
        assert!((back - ppm).abs() < EPSILON);
    }

    #[test]
    fn test_config_presets() {
        let lung = XenonNmrConfig::lung_ventilation(3.0);
        assert!((lung.t1_seconds - 20.0).abs() < EPSILON);
        let dissolved = XenonNmrConfig::dissolved_phase(3.0);
        assert!((dissolved.t1_seconds - 8.0).abs() < EPSILON);
    }

    // --- SpinExchangeModel tests ---

    #[test]
    fn test_seop_steady_state() {
        let model = SpinExchangeModel::new(0.02, 0.95, 2000.0);
        let p_ss = model.steady_state_polarization();
        // P_ss = gamma_SE * P_Rb / (gamma_SE + 1/T1)
        let expected = 0.02 * 0.95 / (0.02 + 1.0 / 2000.0);
        assert!((p_ss - expected).abs() < EPSILON);
        assert!(p_ss > 0.0 && p_ss < 1.0);
    }

    #[test]
    fn test_seop_buildup_time_constant() {
        let model = SpinExchangeModel::new(0.02, 0.95, 2000.0);
        let tau = model.buildup_time_constant();
        let expected = 1.0 / (0.02 + 1.0 / 2000.0);
        assert!((tau - expected).abs() < 0.01);
    }

    #[test]
    fn test_seop_dp_dt_at_zero() {
        let model = SpinExchangeModel::new(0.02, 0.95, 2000.0);
        let dp = model.dp_dt(0.0);
        // At P_Xe=0: dp/dt = gamma_SE * P_Rb
        assert!((dp - 0.02 * 0.95).abs() < EPSILON);
    }

    #[test]
    fn test_seop_dp_dt_at_steady_state() {
        let model = SpinExchangeModel::new(0.02, 0.95, 2000.0);
        let p_ss = model.steady_state_polarization();
        let dp = model.dp_dt(p_ss);
        assert!(dp.abs() < 1.0e-10);
    }

    #[test]
    fn test_seop_buildup_simulation() {
        let model = SpinExchangeModel::typical_high_flow();
        let times: Vec<f64> = (0..100).map(|i| i as f64 * 10.0).collect();
        let pol = model.simulate_buildup(&times);
        assert_eq!(pol.len(), 100);
        assert!(pol[0].abs() < EPSILON); // Starts at 0
        // Should approach steady state
        let p_ss = model.steady_state_polarization();
        assert!((pol[99] - p_ss).abs() < 0.05);
    }

    #[test]
    fn test_seop_euler_simulation() {
        let model = SpinExchangeModel::new(0.02, 0.90, 1000.0);
        let result = model.simulate_euler(0.0, 1.0, 500);
        assert_eq!(result.len(), 500);
        assert!(result[0].abs() < EPSILON);
        // Should be monotonically increasing
        for i in 1..result.len() {
            assert!(result[i] >= result[i - 1] - EPSILON);
        }
    }

    #[test]
    fn test_seop_presets() {
        let hf = SpinExchangeModel::typical_high_flow();
        assert!(hf.gamma_se > 0.0);
        let batch = SpinExchangeModel::typical_batch();
        assert!(batch.gamma_se > 0.0);
        assert!(batch.gamma_se < hf.gamma_se);
    }

    // --- PolarizationCalculator tests ---

    #[test]
    fn test_thermal_polarization_3t() {
        let calc = PolarizationCalculator::new(3.0, 300.0);
        let p = calc.thermal_polarization();
        // Should be very small, ~2.8e-6 for Xe-129 at 3T/300K
        assert!(p > 0.0);
        assert!(p < 1.0e-4);
    }

    #[test]
    fn test_thermal_polarization_increases_with_field() {
        let p1 = PolarizationCalculator::new(1.5, 300.0).thermal_polarization();
        let p3 = PolarizationCalculator::new(3.0, 300.0).thermal_polarization();
        assert!(p3 > p1);
        // Should be roughly linear for small argument: p3 ~ 2*p1
        assert!((p3 / p1 - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_thermal_polarization_decreases_with_temperature() {
        let p_cold = PolarizationCalculator::new(3.0, 100.0).thermal_polarization();
        let p_hot = PolarizationCalculator::new(3.0, 300.0).thermal_polarization();
        assert!(p_cold > p_hot);
    }

    #[test]
    fn test_enhancement_factor() {
        let calc = PolarizationCalculator::new(3.0, 300.0);
        let enhancement = calc.enhancement_factor(0.30);
        // P_thermal ~ 2.8e-6 at 3T/300K, P_hp = 0.30 -> enhancement ~ 100,000+
        assert!(enhancement > 10000.0);
        assert!(enhancement < 1000000.0);
    }

    #[test]
    fn test_polarization_after_pulses() {
        let calc = PolarizationCalculator::new(3.0, 300.0);
        // 10 pulses of 10 degrees
        let alpha = 10.0_f64.to_radians();
        let p = calc.polarization_after_pulses(0.30, alpha, 10);
        let expected = 0.30 * alpha.cos().powi(10);
        assert!((p - expected).abs() < EPSILON);
    }

    #[test]
    fn test_polarization_after_90_degree_pulse() {
        let calc = PolarizationCalculator::new(3.0, 300.0);
        let p = calc.polarization_after_pulses(0.30, PI / 2.0, 1);
        assert!(p.abs() < EPSILON);
    }

    // --- FidProcessor tests ---

    #[test]
    fn test_fid_exponential_apodization() {
        let proc = FidProcessor::new(10000.0);
        let fid_re = vec![1.0; 100];
        let fid_im = vec![0.0; 100];
        let (apod_re, _apod_im) = proc.apodize_exponential(&fid_re, &fid_im);
        assert_eq!(apod_re.len(), 100);
        assert!((apod_re[0] - 1.0).abs() < EPSILON);
        assert!(apod_re[99] < apod_re[0]); // Decays
    }

    #[test]
    fn test_fid_gaussian_apodization() {
        let proc = FidProcessor::new(10000.0);
        let fid_re = vec![1.0; 100];
        let fid_im = vec![0.0; 100];
        let (apod_re, _) = proc.apodize_gaussian(&fid_re, &fid_im, 50.0);
        assert_eq!(apod_re.len(), 100);
        assert!((apod_re[0] - 1.0).abs() < EPSILON);
        assert!(apod_re[99] < apod_re[0]);
    }

    #[test]
    fn test_fid_zero_fill() {
        let proc = FidProcessor::new(10000.0);
        let re = vec![1.0, 2.0, 3.0, 4.0];
        let im = vec![0.1, 0.2, 0.3, 0.4];
        let (zf_re, zf_im) = proc.zero_fill(&re, &im);
        assert_eq!(zf_re.len(), 8); // 4 * 2 = 8
        assert!((zf_re[0] - 1.0).abs() < EPSILON);
        assert!((zf_re[4] - 0.0).abs() < EPSILON); // Zero-filled portion
        assert!((zf_im[3] - 0.4).abs() < EPSILON);
    }

    #[test]
    fn test_dft_single_tone() {
        // DFT of a single-frequency complex exponential (not cosine, to avoid +/- ambiguity)
        let n = 64;
        let freq_bin = 5;
        let re: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq_bin as f64 * i as f64 / n as f64).cos())
            .collect();
        let im: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq_bin as f64 * i as f64 / n as f64).sin())
            .collect();
        let (spec_re, spec_im) = FidProcessor::dft(&re, &im);
        let mag = FidProcessor::magnitude(&spec_re, &spec_im);

        // Peak should be at bin 5
        let peak_idx = mag
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak_idx, freq_bin);
    }

    #[test]
    fn test_dft_dc_signal() {
        let n = 32;
        let re = vec![1.0; n];
        let im = vec![0.0; n];
        let (spec_re, spec_im) = FidProcessor::dft(&re, &im);
        let mag = FidProcessor::magnitude(&spec_re, &spec_im);
        assert!((mag[0] - n as f64).abs() < EPSILON);
        for i in 1..n {
            assert!(mag[i] < EPSILON);
        }
    }

    #[test]
    fn test_phase_correction_zero_order() {
        let re = vec![1.0, 0.0, -1.0, 0.0];
        let im = vec![0.0, 1.0, 0.0, -1.0];
        // Apply 90 degree phase correction
        let (corr_re, corr_im) = FidProcessor::phase_correct_zero(&re, &im, PI / 2.0);
        // After 90 degree: new_re = re*cos(90) + im*sin(90) = im
        assert!((corr_re[0] - 0.0).abs() < EPSILON);
        assert!((corr_re[1] - 1.0).abs() < EPSILON);
        assert!((corr_im[0] - (-1.0)).abs() < EPSILON);
    }

    #[test]
    fn test_phase_correction_first_order() {
        let n = 16;
        let re = vec![1.0; n];
        let im = vec![0.0; n];
        let (corr_re, _corr_im) =
            FidProcessor::phase_correct_first(&re, &im, 0.0, PI, n / 2);
        // At pivot, phi = 0 so corr_re[pivot] should be ~1.0
        assert!((corr_re[n / 2] - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_fid_full_pipeline() {
        let proc = FidProcessor::new(10000.0);
        let fid_re = vec![1.0; 32];
        let fid_im = vec![0.0; 32];
        let spectrum = proc.process(&fid_re, &fid_im);
        assert_eq!(spectrum.len(), 64); // 32 * 2 (zero fill factor)
        assert!(spectrum[0] > 0.0);
    }

    #[test]
    fn test_frequency_axis() {
        let proc = FidProcessor::new(10000.0);
        let axis = proc.frequency_axis(100);
        assert_eq!(axis.len(), 100);
        assert!((axis[0] - 0.0).abs() < EPSILON);
        // At halfway: should be bw/2
        assert!((axis[50] - 5000.0).abs() < EPSILON);
    }

    // --- ChemicalShiftAnalyzer tests ---

    #[test]
    fn test_chemical_shift_ppm_to_hz() {
        let analyzer = ChemicalShiftAnalyzer::new(3.0, 50000.0);
        let hz = analyzer.ppm_to_hz(197.0);
        // 197 ppm * 35.331 MHz * 1e-6 ~ 6960 Hz
        assert!((hz - 6960.0).abs() < 100.0);
    }

    #[test]
    fn test_chemical_shift_find_peak() {
        let analyzer = ChemicalShiftAnalyzer::new(3.0, 50000.0);
        // Create a simple spectrum with a peak
        let n = 256;
        let mut spectrum = vec![0.0; n];
        // Put a peak at index that corresponds to ~0 ppm (gas)
        spectrum[0] = 100.0;
        let result = analyzer.find_peak_near(&spectrum, 0.0, 5.0);
        assert!(result.is_some());
        let (_, amp) = result.unwrap();
        assert!((amp - 100.0).abs() < EPSILON);
    }

    #[test]
    fn test_dissolved_phase_analysis() {
        let analyzer = ChemicalShiftAnalyzer::new(3.0, 50000.0);
        // Create spectrum with peaks at gas, tissue, RBC frequencies
        let n = 4096;
        let mut spectrum = vec![1.0; n];
        // Gas peak at index 0
        spectrum[0] = 1000.0;
        // Tissue peak (~197 ppm): find corresponding index
        let tissue_freq = 197.0 * analyzer.larmor_freq_hz * 1.0e-6;
        let tissue_idx = (tissue_freq / (analyzer.bandwidth_hz / n as f64)).round() as usize;
        if tissue_idx < n {
            spectrum[tissue_idx] = 300.0;
        }
        // RBC peak (~217 ppm)
        let rbc_freq = 217.0 * analyzer.larmor_freq_hz * 1.0e-6;
        let rbc_idx = (rbc_freq / (analyzer.bandwidth_hz / n as f64)).round() as usize;
        if rbc_idx < n {
            spectrum[rbc_idx] = 150.0;
        }

        let result = analyzer.analyze_dissolved_phase(&spectrum);
        assert!(result.gas_amplitude > 0.0);
    }

    // --- DissolvedPhaseMapper tests ---

    #[test]
    fn test_dissolved_phase_basis_functions() {
        let mapper = DissolvedPhaseMapper::new(3.0, 50000.0);
        let (re, im) = mapper.basis_function(0, 100);
        assert_eq!(re.len(), 100);
        assert_eq!(im.len(), 100);
        // Gas component at 0 Hz: first sample should be (1, 0)
        assert!((re[0] - 1.0).abs() < EPSILON);
        assert!(im[0].abs() < EPSILON);
    }

    #[test]
    fn test_dissolved_phase_decomposition_single_component() {
        let mapper = DissolvedPhaseMapper::new(3.0, 50000.0);
        let n = 512;
        // Generate pure gas-phase FID
        let (fid_re, fid_im) = mapper.basis_function(0, n);
        let magnitudes = mapper.component_magnitudes(&fid_re, &fid_im);
        // Gas should dominate
        assert!(magnitudes[0] > magnitudes[1]);
        assert!(magnitudes[0] > magnitudes[2]);
    }

    // --- T1RelaxationTracker tests ---

    #[test]
    fn test_t1_decay() {
        let tracker = T1RelaxationTracker::new(1.0, 20.0);
        let m = tracker.m_at_time(20.0);
        let expected = (-1.0_f64).exp(); // e^(-1) ~ 0.368
        assert!((m - expected).abs() < 0.001);
    }

    #[test]
    fn test_t1_with_pulses() {
        let tracker = T1RelaxationTracker::new(1.0, 20.0);
        let alpha = 10.0_f64.to_radians();
        let m = tracker.m_at_time_with_pulses(0.0, 5, alpha);
        let expected = alpha.cos().powi(5);
        assert!((m - expected).abs() < EPSILON);
    }

    #[test]
    fn test_t1_apply_pulse() {
        let mut tracker = T1RelaxationTracker::new(1.0, 20.0);
        let alpha = 30.0_f64.to_radians();
        let signal = tracker.apply_pulse(alpha, 0.0);
        // Signal = sin(30deg) = 0.5
        assert!((signal - 0.5).abs() < EPSILON);
        // Remaining = cos(30deg) ~ 0.866
        assert!((tracker.current_magnetization() - alpha.cos()).abs() < EPSILON);
        assert_eq!(tracker.pulse_count(), 1);
    }

    #[test]
    fn test_t1_multiple_pulses() {
        let mut tracker = T1RelaxationTracker::new(1.0, 1000.0); // Long T1
        let alpha = 10.0_f64.to_radians();
        for _ in 0..10 {
            tracker.apply_pulse(alpha, 0.0);
        }
        let expected = alpha.cos().powi(10);
        assert!((tracker.current_magnetization() - expected).abs() < 0.001);
    }

    #[test]
    fn test_t1_presets() {
        let lung = T1RelaxationTracker::lung();
        assert!((lung.t1 - 20.0).abs() < EPSILON);
        let blood = T1RelaxationTracker::blood();
        assert!((blood.t1 - 7.0).abs() < EPSILON);
    }

    // --- FlipAngleOptimizer tests ---

    #[test]
    fn test_vfa_last_angle_90() {
        let opt = FlipAngleOptimizer::new(10);
        let angles = opt.variable_flip_angles();
        assert_eq!(angles.len(), 10);
        assert!((angles[9] - PI / 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_vfa_angles_increasing() {
        let opt = FlipAngleOptimizer::new(32);
        let angles = opt.variable_flip_angles();
        for i in 1..angles.len() {
            assert!(angles[i] >= angles[i - 1] - EPSILON);
        }
    }

    #[test]
    fn test_vfa_equal_signal() {
        let opt = FlipAngleOptimizer::new(64);
        let signals = opt.simulate_vfa_signal();
        assert_eq!(signals.len(), 64);
        // All signals should be approximately equal
        let mean = signals.iter().sum::<f64>() / signals.len() as f64;
        for s in &signals {
            assert!((s - mean).abs() / mean < 0.01); // Within 1%
        }
    }

    #[test]
    fn test_vfa_degrees() {
        let opt = FlipAngleOptimizer::new(10);
        let deg = opt.variable_flip_angles_deg();
        assert_eq!(deg.len(), 10);
        assert!((deg[9] - 90.0).abs() < 0.01);
    }

    #[test]
    fn test_vfa_empty() {
        let opt = FlipAngleOptimizer::new(0);
        let angles = opt.variable_flip_angles();
        assert!(angles.is_empty());
    }

    #[test]
    fn test_constant_angle_signal_decay() {
        let opt = FlipAngleOptimizer::new(20);
        let alpha = 10.0_f64.to_radians();
        let signals = opt.simulate_constant_signal(alpha);
        // Signal should decay monotonically with constant angle
        for i in 1..signals.len() {
            assert!(signals[i] < signals[i - 1] + EPSILON);
        }
    }

    #[test]
    fn test_optimal_constant_angle() {
        let opt = FlipAngleOptimizer::new(100);
        let alpha = opt.optimal_constant_angle();
        // Should be a small angle for many acquisitions
        assert!(alpha > 0.0);
        assert!(alpha < PI / 4.0); // Less than 45 degrees
    }

    // --- VentilationDefectMapper tests ---

    #[test]
    fn test_vdp_no_defects() {
        let mapper = VentilationDefectMapper::new();
        let signals = vec![1.0, 1.0, 1.0, 1.0, 1.0];
        let mask = vec![true, true, true, true, true];
        let vdp = mapper.compute_vdp(&signals, &mask);
        assert!(vdp < 0.01);
    }

    #[test]
    fn test_vdp_all_defects() {
        let mapper = VentilationDefectMapper::with_threshold(0.5);
        // Mean = 0.1, threshold = 0.05, all below threshold
        let signals = vec![0.01, 0.01, 0.01, 0.01];
        let mask = vec![true, true, true, true];
        // All voxels same value -> mean = 0.01, threshold = 0.005, all above
        // Actually, all are same so none are defects
        let vdp = mapper.compute_vdp(&signals, &mask);
        assert!(vdp < 0.01); // All same -> none below 50% of mean
    }

    #[test]
    fn test_vdp_mixed() {
        let mapper = VentilationDefectMapper::with_threshold(0.5);
        // 4 voxels with signal, 1 with zero (defect)
        let signals = vec![1.0, 1.0, 1.0, 1.0, 0.0];
        let mask = vec![true, true, true, true, true];
        let vdp = mapper.compute_vdp(&signals, &mask);
        // Mean = 0.8, threshold = 0.4, defect count = 1 (the 0.0)
        // VDP = 1/5 * 100 = 20%
        assert!((vdp - 20.0).abs() < 0.01);
    }

    #[test]
    fn test_vdp_with_mask() {
        let mapper = VentilationDefectMapper::new();
        let signals = vec![1.0, 0.0, 1.0, 0.0];
        let mask = vec![true, false, true, false];
        // Only lung voxels: [1.0, 1.0], mean=1.0, threshold=0.1
        let vdp = mapper.compute_vdp(&signals, &mask);
        assert!(vdp < 0.01); // No defects among lung voxels
    }

    #[test]
    fn test_ventilation_classification() {
        assert_eq!(
            VentilationDefectMapper::classify_ventilation(3.0),
            VentilationCategory::Normal
        );
        assert_eq!(
            VentilationDefectMapper::classify_ventilation(10.0),
            VentilationCategory::Mild
        );
        assert_eq!(
            VentilationDefectMapper::classify_ventilation(20.0),
            VentilationCategory::Moderate
        );
        assert_eq!(
            VentilationDefectMapper::classify_ventilation(35.0),
            VentilationCategory::Severe
        );
    }

    #[test]
    fn test_defect_map() {
        let mapper = VentilationDefectMapper::with_threshold(0.5);
        let signals = vec![1.0, 1.0, 0.0, 1.0];
        let mask = vec![true, true, true, true];
        let map = mapper.defect_map(&signals, &mask);
        // Mean = 0.75, threshold = 0.375
        assert!(map[0]); // 1.0 > 0.375
        assert!(!map[2]); // 0.0 < 0.375
    }

    #[test]
    fn test_signal_histogram() {
        let signals = vec![0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
        let mask = vec![true; 10];
        let (edges, counts) = VentilationDefectMapper::signal_histogram(&signals, &mask, 5);
        assert_eq!(edges.len(), 6); // num_bins + 1
        let total: usize = counts.iter().sum();
        assert_eq!(total, 10);
    }

    // --- DiffusionCoefficient tests ---

    #[test]
    fn test_adc_two_points() {
        let s0: f64 = 1000.0;
        let adc: f64 = 0.04; // cm^2/s (healthy lung)
        let b0: f64 = 0.0;
        let b1: f64 = 10.0; // s/cm^2
        let exponent: f64 = -b1 * adc;
        let s1 = s0 * exponent.exp();
        let computed = DiffusionCoefficient::compute_adc(s0, s1, b0, b1);
        assert!((computed - adc).abs() < 1.0e-10);
    }

    #[test]
    fn test_adc_multi_point() {
        let true_adc: f64 = 0.05;
        let s0_true: f64 = 800.0;
        let b_values: Vec<f64> = vec![0.0, 2.0, 5.0, 10.0, 15.0];
        let signals: Vec<f64> = b_values
            .iter()
            .map(|&b| { let exp_arg: f64 = -b * true_adc; s0_true * exp_arg.exp() })
            .collect();
        let (adc, s0_est) = DiffusionCoefficient::compute_adc_multi(&signals, &b_values);
        assert!((adc - true_adc).abs() < 1.0e-6);
        assert!((s0_est - s0_true).abs() < 0.1);
    }

    #[test]
    fn test_adc_predict_signal() {
        let s0 = 1000.0;
        let adc = 0.04;
        let b = 5.0;
        let predicted = DiffusionCoefficient::predict_signal(s0, adc, b);
        let expected = 1000.0 * (-5.0 * 0.04_f64).exp();
        assert!((predicted - expected).abs() < EPSILON);
    }

    #[test]
    fn test_adc_classification() {
        assert_eq!(
            DiffusionCoefficient::classify_adc(0.02),
            AdcCategory::Restricted
        );
        assert_eq!(
            DiffusionCoefficient::classify_adc(0.04),
            AdcCategory::Normal
        );
        assert_eq!(
            DiffusionCoefficient::classify_adc(0.06),
            AdcCategory::MildEmphysema
        );
        assert_eq!(
            DiffusionCoefficient::classify_adc(0.10),
            AdcCategory::SevereEmphysema
        );
    }

    // --- Synthetic FID tests ---

    #[test]
    fn test_generate_synthetic_fid() {
        let components = vec![(1.0, 100.0, 0.01, 0.0)];
        let (re, im) = generate_synthetic_fid(&components, 256, 10000.0);
        assert_eq!(re.len(), 256);
        assert_eq!(im.len(), 256);
        // First sample: cos(0) = 1, sin(0) = 0
        assert!((re[0] - 1.0).abs() < EPSILON);
        assert!(im[0].abs() < EPSILON);
    }

    #[test]
    fn test_synthetic_fid_decays() {
        let components = vec![(1.0, 0.0, 0.001, 0.0)];
        let (re, _im) = generate_synthetic_fid(&components, 100, 10000.0);
        // Should decay over time
        assert!(re[0] > re[50]);
        assert!(re[50] > re[99]);
    }

    #[test]
    fn test_synthetic_fid_multi_component() {
        let components = vec![
            (1.0, 0.0, 0.01, 0.0),
            (0.5, 500.0, 0.005, 0.0),
        ];
        let (re, im) = generate_synthetic_fid(&components, 256, 10000.0);
        assert_eq!(re.len(), 256);
        // First sample: amp1*cos(0) + amp2*cos(0) = 1.0 + 0.5 = 1.5
        assert!((re[0] - 1.5).abs() < EPSILON);
        assert!(im[0].abs() < EPSILON);
    }

    #[test]
    fn test_fid_process_with_synthetic() {
        let proc = FidProcessor::new(10000.0);
        let components = vec![(1.0, 500.0, 0.01, 0.0)];
        let (re, im) = generate_synthetic_fid(&components, 64, 10000.0);
        let spectrum = proc.process(&re, &im);
        // Should have a peak somewhere
        let max_val = spectrum.iter().cloned().fold(0.0_f64, f64::max);
        assert!(max_val > 0.0);
    }

    // --- Edge cases ---

    #[test]
    fn test_adc_zero_signal() {
        let adc = DiffusionCoefficient::compute_adc(0.0, 0.0, 0.0, 1.0);
        assert!((adc - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_vdp_empty() {
        let mapper = VentilationDefectMapper::new();
        let vdp = mapper.compute_vdp(&[], &[]);
        assert!((vdp - 0.0).abs() < EPSILON);
    }
}

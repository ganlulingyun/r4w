//! Piezoelectric Energy Harvester — Vibration-to-Electrical Energy Conversion
//!
//! Signal processing for piezoelectric energy harvesting from mechanical vibrations.
//! Models cantilever beam dynamics, electromechanical coupling, rectifier circuits,
//! energy storage, and broadband harvesting arrays.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::piezoelectric_energy_harvester::{PiezoConfig, CantileverBeam, PowerEstimator};
//!
//! let config = PiezoConfig::pzt_5a();
//! let beam = CantileverBeam::unimorph(&config, 0.01); // 1% damping ratio
//! let f_n = beam.natural_frequency_hz();
//! let estimator = PowerEstimator::new(&config, &beam);
//! let p_max = estimator.max_power_at_resonance(1.0); // 1 m/s^2 acceleration
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// PiezoConfig — material and geometry parameters
// ---------------------------------------------------------------------------

/// Piezoelectric material and geometry configuration.
#[derive(Debug, Clone)]
pub struct PiezoConfig {
    /// Piezoelectric strain coefficient d31 (C/N).
    pub d31: f64,
    /// Relative permittivity epsilon_33^T (F/m).
    pub epsilon_33: f64,
    /// Elastic modulus (Young's modulus) in Pa.
    pub elastic_modulus_pa: f64,
    /// Material density in kg/m^3.
    pub density_kgm3: f64,
    /// Layer thickness in meters.
    pub thickness_m: f64,
    /// Beam length in meters.
    pub length_m: f64,
    /// Beam width in meters.
    pub width_m: f64,
}

impl PiezoConfig {
    /// PZT-5A (lead zirconate titanate, Navy Type II).
    pub fn pzt_5a() -> Self {
        Self {
            d31: -190e-12,
            epsilon_33: 1700.0 * 8.854e-12,
            elastic_modulus_pa: 66e9,
            density_kgm3: 7750.0,
            thickness_m: 0.5e-3,
            length_m: 30e-3,
            width_m: 10e-3,
        }
    }

    /// PZT-5H (lead zirconate titanate, Navy Type VI — higher coupling).
    pub fn pzt_5h() -> Self {
        Self {
            d31: -274e-12,
            epsilon_33: 3400.0 * 8.854e-12,
            elastic_modulus_pa: 62e9,
            density_kgm3: 7500.0,
            thickness_m: 0.5e-3,
            length_m: 30e-3,
            width_m: 10e-3,
        }
    }

    /// PVDF (polyvinylidene fluoride — flexible polymer piezo).
    pub fn pvdf() -> Self {
        Self {
            d31: -33e-12,
            epsilon_33: 12.0 * 8.854e-12,
            elastic_modulus_pa: 2.5e9,
            density_kgm3: 1780.0,
            thickness_m: 0.1e-3,
            length_m: 40e-3,
            width_m: 15e-3,
        }
    }

    /// AlN (aluminum nitride — MEMS compatible).
    pub fn aln() -> Self {
        Self {
            d31: -2.0e-12,
            epsilon_33: 10.5 * 8.854e-12,
            elastic_modulus_pa: 345e9,
            density_kgm3: 3260.0,
            thickness_m: 1e-6,
            length_m: 1e-3,
            width_m: 0.5e-3,
        }
    }

    /// Cross-sectional area (m^2).
    pub fn cross_section_area(&self) -> f64 {
        self.width_m * self.thickness_m
    }

    /// Volume of piezo layer (m^3).
    pub fn volume(&self) -> f64 {
        self.length_m * self.width_m * self.thickness_m
    }

    /// Electromechanical coupling coefficient k_e^2 = d31^2 * E / epsilon_33.
    pub fn coupling_k_squared(&self) -> f64 {
        self.d31 * self.d31 * self.elastic_modulus_pa / self.epsilon_33
    }

    /// Clamped (blocked) capacitance C_p = epsilon_33 * A / t.
    pub fn capacitance(&self) -> f64 {
        self.epsilon_33 * self.length_m * self.width_m / self.thickness_m
    }
}

// ---------------------------------------------------------------------------
// Cantilever beam model
// ---------------------------------------------------------------------------

/// Beam configuration: unimorph (one active layer) or bimorph (two active layers).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BeamType {
    Unimorph,
    Bimorph,
}

/// Cantilever beam lumped-parameter model.
#[derive(Debug, Clone)]
pub struct CantileverBeam {
    /// Effective mass (kg).
    pub mass_eff: f64,
    /// Effective stiffness (N/m).
    pub stiffness: f64,
    /// Damping coefficient (N*s/m).
    pub damping: f64,
    /// Damping ratio (zeta).
    pub damping_ratio: f64,
    /// Natural frequency (rad/s).
    pub omega_n: f64,
    /// Beam type.
    pub beam_type: BeamType,
    /// Transformer coupling factor alpha = d31 * E * w * t_p / L.
    pub alpha: f64,
    /// Clamped capacitance (F).
    pub capacitance: f64,
}

impl CantileverBeam {
    /// First mode eigenvalue for clamped-free beam.
    const LAMBDA_1: f64 = 1.8751;

    /// Create unimorph cantilever beam model.
    pub fn unimorph(config: &PiezoConfig, damping_ratio: f64) -> Self {
        Self::new(config, damping_ratio, BeamType::Unimorph)
    }

    /// Create bimorph cantilever beam model.
    pub fn bimorph(config: &PiezoConfig, damping_ratio: f64) -> Self {
        Self::new(config, damping_ratio, BeamType::Bimorph)
    }

    /// Create cantilever beam model from config and damping ratio.
    pub fn new(config: &PiezoConfig, damping_ratio: f64, beam_type: BeamType) -> Self {
        let l = config.length_m;
        let w = config.width_m;
        let t = config.thickness_m;
        let e = config.elastic_modulus_pa;
        let rho = config.density_kgm3;

        // Second moment of area for rectangular cross-section
        let thickness_eff = match beam_type {
            BeamType::Unimorph => t,
            BeamType::Bimorph => 2.0 * t,
        };
        let i_moment = w * thickness_eff.powi(3) / 12.0;
        let area = w * thickness_eff;
        let mass_per_length = rho * area;

        // Natural frequency: f_n = (lambda_1^2 / (2*pi*L^2)) * sqrt(E*I / (rho*A))
        let omega_n = Self::LAMBDA_1.powi(2) / l.powi(2)
            * (e * i_moment / mass_per_length).sqrt();

        // Lumped effective mass (approx 0.2427 * total mass for first mode)
        let mass_eff = 0.2427 * mass_per_length * l;

        // Stiffness from omega_n^2 = k/m
        let stiffness = mass_eff * omega_n * omega_n;

        // Damping coefficient c = 2 * zeta * m * omega_n
        let damping = 2.0 * damping_ratio * mass_eff * omega_n;

        // Electromechanical coupling factor: alpha = d31 * E * w * t_p / L
        let alpha = config.d31.abs() * e * w * t / l;

        let capacitance = config.capacitance();

        Self {
            mass_eff,
            stiffness,
            damping,
            damping_ratio,
            omega_n,
            beam_type,
            alpha,
            capacitance,
        }
    }

    /// Natural frequency in Hz.
    pub fn natural_frequency_hz(&self) -> f64 {
        self.omega_n / (2.0 * PI)
    }

    /// Quality factor Q = 1/(2*zeta).
    pub fn quality_factor(&self) -> f64 {
        1.0 / (2.0 * self.damping_ratio)
    }

    /// 3 dB bandwidth = f_n / Q.
    pub fn bandwidth_3db(&self) -> f64 {
        self.natural_frequency_hz() / self.quality_factor()
    }

    /// Figure of merit: FoM = k_e^2 * Q.
    pub fn figure_of_merit(&self, config: &PiezoConfig) -> f64 {
        config.coupling_k_squared() * self.quality_factor()
    }
}

// ---------------------------------------------------------------------------
// Equivalent circuit model
// ---------------------------------------------------------------------------

/// Equivalent circuit (Butterworth-Van Dyke) parameters.
#[derive(Debug, Clone)]
pub struct EquivalentCircuit {
    /// Motional inductance L_m (H) — represents mass.
    pub l_m: f64,
    /// Motional capacitance C_m (F) — represents compliance.
    pub c_m: f64,
    /// Motional resistance R_m (Ohm) — represents damping.
    pub r_m: f64,
    /// Piezo current source factor alpha (C/m or A/(m/s)).
    pub alpha: f64,
    /// Clamped capacitance C_p (F).
    pub c_p: f64,
}

impl EquivalentCircuit {
    /// Derive equivalent circuit from cantilever beam model.
    pub fn from_beam(beam: &CantileverBeam) -> Self {
        let alpha = beam.alpha;
        let alpha_sq = alpha * alpha;

        // Mechanical → electrical analogy through transformer ratio
        // L_m = m / alpha^2, C_m = alpha^2 / k, R_m = c / alpha^2
        let l_m = beam.mass_eff / alpha_sq;
        let c_m = alpha_sq / beam.stiffness;
        let r_m = beam.damping / alpha_sq;

        Self {
            l_m,
            c_m,
            r_m,
            alpha,
            c_p: beam.capacitance,
        }
    }

    /// Resonance frequency of the motional branch (Hz).
    pub fn resonance_hz(&self) -> f64 {
        1.0 / (2.0 * PI * (self.l_m * self.c_m).sqrt())
    }

    /// Impedance magnitude at a given frequency (Ohm).
    pub fn impedance_magnitude(&self, freq_hz: f64) -> f64 {
        let omega = 2.0 * PI * freq_hz;
        // Motional branch: Z_m = R_m + j*(omega*L_m - 1/(omega*C_m))
        let x_m = omega * self.l_m - 1.0 / (omega * self.c_m);
        let z_m_re = self.r_m;
        let z_m_im = x_m;
        let z_m_mag_sq = z_m_re * z_m_re + z_m_im * z_m_im;

        // Static capacitance branch: Z_p = 1/(j*omega*C_p) = -j/(omega*C_p)
        let b_p = omega * self.c_p; // susceptance

        // Parallel combination: 1/Z_total = 1/Z_m + j*omega*C_p
        let y_re = z_m_re / z_m_mag_sq;
        let y_im = -z_m_im / z_m_mag_sq + b_p;

        1.0 / (y_re * y_re + y_im * y_im).sqrt()
    }
}

// ---------------------------------------------------------------------------
// Frequency response
// ---------------------------------------------------------------------------

/// Frequency response of the harvester.
pub struct FrequencyResponse;

impl FrequencyResponse {
    /// Displacement magnitude at angular frequency omega for base excitation F0.
    /// X(omega) = F0/m / sqrt((omega_n^2 - omega^2)^2 + (2*zeta*omega_n*omega)^2)
    pub fn displacement(beam: &CantileverBeam, omega: f64, f0: f64) -> f64 {
        let on = beam.omega_n;
        let z = beam.damping_ratio;
        let num = f0 / beam.mass_eff;
        let d1 = on * on - omega * omega;
        let d2 = 2.0 * z * on * omega;
        num / (d1 * d1 + d2 * d2).sqrt()
    }

    /// Voltage magnitude across open-circuit terminals at frequency omega.
    /// V(omega) = alpha * X(omega) / C_p at open circuit.
    pub fn open_circuit_voltage(beam: &CantileverBeam, omega: f64, f0: f64) -> f64 {
        let x = Self::displacement(beam, omega, f0);
        // For harmonic excitation, V_oc = alpha * omega * X / (omega * C_p)
        // simplifies for the peak displacement to alpha * X / C_p
        // but more precisely for AC: V = alpha * j*omega*X / (j*omega*C_p + 1/R_L)
        // At open circuit (R_L -> inf): V_oc = alpha * X / C_p
        beam.alpha * x / beam.capacitance
    }

    /// Power delivered to load resistance R_L at frequency omega.
    pub fn power_at_load(beam: &CantileverBeam, omega: f64, f0: f64, r_load: f64) -> f64 {
        let x = Self::displacement(beam, omega, f0);
        let v_dot = omega * x; // velocity amplitude
        let i_p = beam.alpha * v_dot; // current source magnitude

        // Power across R_L in parallel with C_p:
        // P = 0.5 * |I_p|^2 * R_L / (1 + (omega*C_p*R_L)^2)
        let wc = omega * beam.capacitance * r_load;
        0.5 * i_p * i_p * r_load / (1.0 + wc * wc)
    }

    /// Compute frequency response sweep: returns (freq_hz, displacement, voltage, power).
    pub fn sweep(
        beam: &CantileverBeam,
        f_start: f64,
        f_end: f64,
        n_points: usize,
        f0: f64,
        r_load: f64,
    ) -> Vec<(f64, f64, f64, f64)> {
        let mut result = Vec::with_capacity(n_points);
        for i in 0..n_points {
            let frac = i as f64 / (n_points - 1).max(1) as f64;
            let freq = f_start + frac * (f_end - f_start);
            let omega = 2.0 * PI * freq;
            let x = Self::displacement(beam, omega, f0);
            let v = Self::open_circuit_voltage(beam, omega, f0);
            let p = Self::power_at_load(beam, omega, f0, r_load);
            result.push((freq, x, v, p));
        }
        result
    }
}

// ---------------------------------------------------------------------------
// Power output estimation
// ---------------------------------------------------------------------------

/// Power output estimator for piezoelectric harvester.
#[derive(Debug, Clone)]
pub struct PowerEstimator {
    /// Transformer coupling factor.
    pub alpha: f64,
    /// Clamped capacitance (F).
    pub capacitance: f64,
    /// Natural frequency (rad/s).
    pub omega_n: f64,
    /// Damping ratio.
    pub damping_ratio: f64,
    /// Effective mass (kg).
    pub mass_eff: f64,
    /// Volume of piezo element (m^3).
    pub volume: f64,
}

impl PowerEstimator {
    /// Create power estimator from config and beam.
    pub fn new(config: &PiezoConfig, beam: &CantileverBeam) -> Self {
        Self {
            alpha: beam.alpha,
            capacitance: beam.capacitance,
            omega_n: beam.omega_n,
            damping_ratio: beam.damping_ratio,
            mass_eff: beam.mass_eff,
            volume: config.volume(),
        }
    }

    /// Optimal load resistance for maximum power at resonance: R_opt = 1/(omega_n * C_p).
    pub fn optimal_load_resistance(&self) -> f64 {
        1.0 / (self.omega_n * self.capacitance)
    }

    /// Peak displacement at resonance for given base acceleration (m/s^2).
    /// X_max = a_base / (2 * zeta * omega_n^2)
    pub fn peak_displacement(&self, accel_ms2: f64) -> f64 {
        let f0 = self.mass_eff * accel_ms2;
        f0 / (2.0 * self.damping_ratio * self.mass_eff * self.omega_n * self.omega_n)
    }

    /// Peak velocity at resonance: v_max = omega_n * X_max.
    pub fn peak_velocity(&self, accel_ms2: f64) -> f64 {
        self.omega_n * self.peak_displacement(accel_ms2)
    }

    /// Open circuit voltage: V_oc = alpha * X_max / C_p.
    pub fn open_circuit_voltage(&self, accel_ms2: f64) -> f64 {
        self.alpha * self.peak_displacement(accel_ms2) / self.capacitance
    }

    /// Short circuit current: I_sc = alpha * v_dot_max.
    pub fn short_circuit_current(&self, accel_ms2: f64) -> f64 {
        self.alpha * self.peak_velocity(accel_ms2)
    }

    /// Maximum power at resonance with optimal load.
    /// P_max = V_oc^2 / (4 * R_opt)
    pub fn max_power_at_resonance(&self, accel_ms2: f64) -> f64 {
        let v_oc = self.open_circuit_voltage(accel_ms2);
        let r_opt = self.optimal_load_resistance();
        v_oc * v_oc / (4.0 * r_opt)
    }

    /// Power density in W/m^3.
    pub fn power_density(&self, accel_ms2: f64) -> f64 {
        self.max_power_at_resonance(accel_ms2) / self.volume
    }

    /// Power at an arbitrary frequency and load resistance.
    pub fn power_at_frequency(&self, accel_ms2: f64, freq_hz: f64, r_load: f64) -> f64 {
        let omega = 2.0 * PI * freq_hz;
        let f0 = self.mass_eff * accel_ms2;
        let on = self.omega_n;
        let z = self.damping_ratio;

        // Displacement
        let d1 = on * on - omega * omega;
        let d2 = 2.0 * z * on * omega;
        let x = f0 / self.mass_eff / (d1 * d1 + d2 * d2).sqrt();

        // Velocity amplitude
        let v_dot = omega * x;
        let i_p = self.alpha * v_dot;

        // Power across R_L || C_p
        let wc = omega * self.capacitance * r_load;
        0.5 * i_p * i_p * r_load / (1.0 + wc * wc)
    }
}

// ---------------------------------------------------------------------------
// Rectifier circuit models
// ---------------------------------------------------------------------------

/// Rectifier topology for AC-DC conversion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RectifierType {
    /// Standard full-bridge rectifier.
    FullBridge,
    /// Voltage doubler (Greinacher / Villard).
    VoltageDoubler,
    /// Synchronized Switch Harvesting on Inductor.
    Sshi,
}

/// Rectifier circuit model.
#[derive(Debug, Clone)]
pub struct RectifierCircuit {
    /// Rectifier topology.
    pub topology: RectifierType,
    /// Diode forward voltage drop (V).
    pub v_diode: f64,
    /// Quality factor (for SSHI boost calculation).
    pub q_factor: f64,
}

impl RectifierCircuit {
    /// Standard bridge rectifier with Schottky diodes (V_d ~ 0.3V).
    pub fn full_bridge() -> Self {
        Self {
            topology: RectifierType::FullBridge,
            v_diode: 0.3,
            q_factor: 1.0,
        }
    }

    /// Voltage doubler with Schottky diodes.
    pub fn voltage_doubler() -> Self {
        Self {
            topology: RectifierType::VoltageDoubler,
            v_diode: 0.3,
            q_factor: 1.0,
        }
    }

    /// SSHI rectifier with specified Q factor.
    pub fn sshi(q_factor: f64) -> Self {
        Self {
            topology: RectifierType::Sshi,
            v_diode: 0.3,
            q_factor,
        }
    }

    /// DC output voltage from peak AC voltage.
    pub fn dc_voltage(&self, v_peak: f64) -> f64 {
        match self.topology {
            RectifierType::FullBridge => {
                (v_peak - 2.0 * self.v_diode).max(0.0)
            }
            RectifierType::VoltageDoubler => {
                (2.0 * (v_peak - self.v_diode)).max(0.0)
            }
            RectifierType::Sshi => {
                // SSHI effectively boosts the voltage
                let boost = 2.0 * self.q_factor / PI;
                (v_peak * boost - 2.0 * self.v_diode).max(0.0)
            }
        }
    }

    /// SSHI power boost factor relative to standard rectifier: 2*Q/pi.
    pub fn sshi_boost_factor(&self) -> f64 {
        2.0 * self.q_factor / PI
    }

    /// Rectifier efficiency: P_dc / P_ac.
    pub fn efficiency(&self, v_peak: f64) -> f64 {
        if v_peak <= 0.0 {
            return 0.0;
        }
        let v_dc = self.dc_voltage(v_peak);
        let diode_loss_factor = match self.topology {
            RectifierType::FullBridge => 2.0 * self.v_diode / v_peak,
            RectifierType::VoltageDoubler => self.v_diode / v_peak,
            RectifierType::Sshi => 2.0 * self.v_diode / (v_peak * self.sshi_boost_factor()),
        };
        let eff = (1.0 - diode_loss_factor).max(0.0);
        // Clamp to physical range
        eff.min(1.0)
    }

    /// DC power output given AC power and peak voltage.
    pub fn dc_power(&self, p_ac: f64, v_peak: f64) -> f64 {
        p_ac * self.efficiency(v_peak)
    }
}

// ---------------------------------------------------------------------------
// Energy storage model
// ---------------------------------------------------------------------------

/// Capacitor-based energy storage model.
#[derive(Debug, Clone)]
pub struct EnergyStorage {
    /// Storage capacitance (F).
    pub capacitance: f64,
    /// Series resistance (Ohm) — ESR + source resistance.
    pub resistance: f64,
    /// Current voltage on capacitor (V).
    pub voltage: f64,
}

impl EnergyStorage {
    /// Create energy storage with given capacitance and resistance.
    pub fn new(capacitance: f64, resistance: f64) -> Self {
        Self {
            capacitance,
            resistance,
            voltage: 0.0,
        }
    }

    /// Capacitor voltage at time t given constant source voltage.
    /// V_cap(t) = V_source * (1 - exp(-t/(R*C)))
    pub fn voltage_at_time(&self, v_source: f64, time_s: f64) -> f64 {
        let tau = self.resistance * self.capacitance;
        if tau <= 0.0 {
            return v_source;
        }
        v_source * (1.0 - (-time_s / tau).exp())
    }

    /// RC time constant (seconds).
    pub fn time_constant(&self) -> f64 {
        self.resistance * self.capacitance
    }

    /// Energy stored at a given voltage: E = 0.5 * C * V^2.
    pub fn energy_at_voltage(&self, voltage: f64) -> f64 {
        0.5 * self.capacitance * voltage * voltage
    }

    /// Energy currently stored.
    pub fn current_energy(&self) -> f64 {
        self.energy_at_voltage(self.voltage)
    }

    /// Time to charge from 0V to threshold voltage.
    /// t = -R*C * ln(1 - V_threshold / V_source)
    pub fn time_to_threshold(&self, v_source: f64, v_threshold: f64) -> Option<f64> {
        if v_threshold >= v_source || v_source <= 0.0 || v_threshold <= 0.0 {
            return None;
        }
        let tau = self.resistance * self.capacitance;
        Some(-tau * (1.0 - v_threshold / v_source).ln())
    }

    /// Simulate charging over time steps, returns voltage trace.
    pub fn simulate_charging(
        &mut self,
        v_source: f64,
        dt: f64,
        n_steps: usize,
    ) -> Vec<f64> {
        let mut trace = Vec::with_capacity(n_steps);
        let tau = self.resistance * self.capacitance;
        let alpha = if tau > 0.0 { (-dt / tau).exp() } else { 0.0 };

        for _ in 0..n_steps {
            self.voltage = v_source + (self.voltage - v_source) * alpha;
            trace.push(self.voltage);
        }
        trace
    }
}

// ---------------------------------------------------------------------------
// Vibration source characterization
// ---------------------------------------------------------------------------

/// Vibration source analysis utilities.
pub struct VibrationAnalyzer;

impl VibrationAnalyzer {
    /// RMS acceleration from time series (m/s^2).
    pub fn rms_acceleration(samples: &[f64]) -> f64 {
        if samples.is_empty() {
            return 0.0;
        }
        let sum_sq: f64 = samples.iter().map(|&x| x * x).sum();
        (sum_sq / samples.len() as f64).sqrt()
    }

    /// Peak acceleration from time series.
    pub fn peak_acceleration(samples: &[f64]) -> f64 {
        samples
            .iter()
            .map(|x| x.abs())
            .fold(0.0_f64, f64::max)
    }

    /// Crest factor = peak / RMS.
    pub fn crest_factor(samples: &[f64]) -> f64 {
        let rms = Self::rms_acceleration(samples);
        if rms == 0.0 {
            return 0.0;
        }
        Self::peak_acceleration(samples) / rms
    }

    /// Power spectral density via periodogram (single-sided).
    /// Returns (frequency_bins_hz, psd_values).
    pub fn psd(samples: &[f64], sample_rate: f64) -> (Vec<f64>, Vec<f64>) {
        let n = samples.len();
        if n == 0 {
            return (vec![], vec![]);
        }

        // Zero-pad to next power of 2
        let nfft = n.next_power_of_two();
        let mut re = vec![0.0; nfft];
        let mut im = vec![0.0; nfft];

        // Apply Hann window
        for i in 0..n {
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / (n - 1).max(1) as f64).cos());
            re[i] = samples[i] * w;
        }

        // DFT (simple radix-2 Cooley-Tukey)
        fft_in_place(&mut re, &mut im);

        // Single-sided PSD
        let n_out = nfft / 2 + 1;
        let df = sample_rate / nfft as f64;
        let scale = 2.0 / (sample_rate * n as f64);

        let mut freqs = Vec::with_capacity(n_out);
        let mut psd = Vec::with_capacity(n_out);

        for i in 0..n_out {
            freqs.push(i as f64 * df);
            let mag_sq = re[i] * re[i] + im[i] * im[i];
            let p = mag_sq * scale;
            // DC and Nyquist are not doubled
            let p = if i == 0 || i == nfft / 2 { p / 2.0 } else { p };
            psd.push(p);
        }

        (freqs, psd)
    }

    /// Dominant frequency from PSD (excluding DC).
    pub fn dominant_frequency(samples: &[f64], sample_rate: f64) -> f64 {
        let (freqs, psd) = Self::psd(samples, sample_rate);
        if freqs.len() < 2 {
            return 0.0;
        }

        // Skip DC (index 0)
        let mut max_idx = 1;
        let mut max_val = psd[1];
        for i in 2..psd.len() {
            if psd[i] > max_val {
                max_val = psd[i];
                max_idx = i;
            }
        }
        freqs[max_idx]
    }
}

// ---------------------------------------------------------------------------
// Broadband harvesting — multi-resonant array
// ---------------------------------------------------------------------------

/// Multi-resonant harvesting array for broadband vibration.
#[derive(Debug, Clone)]
pub struct BroadbandArray {
    /// Individual beam resonant frequencies (Hz).
    pub frequencies: Vec<f64>,
    /// Individual beam bandwidths (Hz).
    pub bandwidths: Vec<f64>,
    /// Individual beam max power outputs (W) at 1 m/s^2.
    pub max_powers: Vec<f64>,
}

impl BroadbandArray {
    /// Design a multi-resonant array spanning [f_low, f_high] with N beams.
    pub fn design(
        config: &PiezoConfig,
        damping_ratio: f64,
        f_low: f64,
        f_high: f64,
        n_beams: usize,
    ) -> Self {
        let mut frequencies = Vec::with_capacity(n_beams);
        let mut bandwidths = Vec::with_capacity(n_beams);
        let mut max_powers = Vec::with_capacity(n_beams);

        for i in 0..n_beams {
            let frac = if n_beams > 1 {
                i as f64 / (n_beams - 1) as f64
            } else {
                0.5
            };
            let target_freq = f_low + frac * (f_high - f_low);

            // Adjust beam length for target frequency:
            // f_n = (lambda_1^2/(2*pi*L^2)) * sqrt(E*I/(rho*A))
            // L = lambda_1 * (E*I/(rho*A))^(1/4) / (2*pi*f_n)^(1/2)
            let t = config.thickness_m;
            let w = config.width_m;
            let e = config.elastic_modulus_pa;
            let rho = config.density_kgm3;

            let i_moment = w * t * t * t / 12.0;
            let area = w * t;
            let ei_rho_a = e * i_moment / (rho * area);

            let omega_target = 2.0 * PI * target_freq;
            let l_needed = CantileverBeam::LAMBDA_1
                * (ei_rho_a.sqrt() / omega_target).sqrt();

            let mut cfg = config.clone();
            cfg.length_m = l_needed;

            let beam = CantileverBeam::unimorph(&cfg, damping_ratio);
            let estimator = PowerEstimator::new(&cfg, &beam);

            frequencies.push(beam.natural_frequency_hz());
            bandwidths.push(beam.bandwidth_3db());
            max_powers.push(estimator.max_power_at_resonance(1.0));
        }

        Self {
            frequencies,
            bandwidths,
            max_powers,
        }
    }

    /// Number of beams in the array.
    pub fn num_beams(&self) -> usize {
        self.frequencies.len()
    }

    /// Total bandwidth coverage (sum of individual bandwidths).
    pub fn total_bandwidth(&self) -> f64 {
        self.bandwidths.iter().sum()
    }

    /// Bandwidth enhancement factor relative to single beam.
    pub fn bandwidth_enhancement(&self) -> f64 {
        if self.bandwidths.is_empty() {
            return 0.0;
        }
        let single_bw = self.bandwidths[0];
        if single_bw <= 0.0 {
            return 0.0;
        }
        self.total_bandwidth() / single_bw
    }

    /// Total power from array at a given frequency and acceleration.
    pub fn total_power_at_freq(
        &self,
        config: &PiezoConfig,
        damping_ratio: f64,
        freq_hz: f64,
        accel_ms2: f64,
    ) -> f64 {
        let mut total = 0.0;

        for &f_n in &self.frequencies {
            // Calculate power contribution from each beam
            let omega = 2.0 * PI * freq_hz;
            let omega_n = 2.0 * PI * f_n;
            let z = damping_ratio;

            // Frequency response scaling
            let d1 = omega_n * omega_n - omega * omega;
            let d2 = 2.0 * z * omega_n * omega;
            let h_sq = 1.0 / (d1 * d1 + d2 * d2);
            let h_peak = 1.0 / (2.0 * z * omega_n * omega_n);
            let h_peak_sq = h_peak * h_peak;

            // Scale max power by frequency response
            let idx = self
                .frequencies
                .iter()
                .position(|&f| (f - f_n).abs() < 1e-6)
                .unwrap_or(0);
            let p_max = self.max_powers[idx] * accel_ms2 * accel_ms2;
            total += p_max * h_sq / h_peak_sq;
        }

        total
    }
}

// ---------------------------------------------------------------------------
// Simple FFT (radix-2 Cooley-Tukey, in-place)
// ---------------------------------------------------------------------------

fn fft_in_place(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    assert!(n.is_power_of_two(), "FFT length must be power of 2");
    assert_eq!(re.len(), im.len());

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 0..n {
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Butterfly stages
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = -2.0 * PI / len as f64;
        for i in (0..n).step_by(len) {
            for k in 0..half {
                let w_re = (angle * k as f64).cos();
                let w_im = (angle * k as f64).sin();
                let idx1 = i + k;
                let idx2 = i + k + half;

                let t_re = w_re * re[idx2] - w_im * im[idx2];
                let t_im = w_re * im[idx2] + w_im * re[idx2];

                re[idx2] = re[idx1] - t_re;
                im[idx2] = im[idx1] - t_im;
                re[idx1] += t_re;
                im[idx1] += t_im;
            }
        }
        len <<= 1;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-10;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn relative_eq(a: f64, b: f64, rel_tol: f64) -> bool {
        if a == 0.0 && b == 0.0 {
            return true;
        }
        let max_ab = a.abs().max(b.abs());
        (a - b).abs() / max_ab < rel_tol
    }

    // --- PiezoConfig tests ---

    #[test]
    fn test_pzt_5a_preset() {
        let c = PiezoConfig::pzt_5a();
        assert!(c.d31 < 0.0, "d31 should be negative for PZT");
        assert!(c.d31.abs() > 100e-12 && c.d31.abs() < 300e-12);
        assert!(c.elastic_modulus_pa > 50e9 && c.elastic_modulus_pa < 80e9);
        assert!(c.density_kgm3 > 7000.0 && c.density_kgm3 < 8000.0);
    }

    #[test]
    fn test_pzt_5h_preset() {
        let c = PiezoConfig::pzt_5h();
        assert!(c.d31.abs() > PiezoConfig::pzt_5a().d31.abs(),
            "PZT-5H should have higher d31 than PZT-5A");
    }

    #[test]
    fn test_pvdf_preset() {
        let c = PiezoConfig::pvdf();
        assert!(c.d31.abs() < 50e-12, "PVDF has low d31");
        assert!(c.elastic_modulus_pa < 5e9, "PVDF is a soft polymer");
        assert!(c.density_kgm3 < 2000.0, "PVDF density is low");
    }

    #[test]
    fn test_aln_preset() {
        let c = PiezoConfig::aln();
        assert!(c.d31.abs() < 5e-12, "AlN has very small d31");
        assert!(c.elastic_modulus_pa > 300e9, "AlN is very stiff");
        assert!(c.thickness_m < 10e-6, "AlN is MEMS-scale thin");
    }

    #[test]
    fn test_cross_section_area() {
        let c = PiezoConfig::pzt_5a();
        let a = c.cross_section_area();
        assert!(approx_eq(a, c.width_m * c.thickness_m, EPSILON));
    }

    #[test]
    fn test_volume() {
        let c = PiezoConfig::pzt_5a();
        let v = c.volume();
        assert!(approx_eq(v, c.length_m * c.width_m * c.thickness_m, EPSILON));
    }

    #[test]
    fn test_coupling_coefficient() {
        let c = PiezoConfig::pzt_5a();
        let k2 = c.coupling_k_squared();
        assert!(k2 > 0.0, "k^2 must be positive");
        assert!(k2 < 1.0, "k^2 must be less than 1 for realistic materials");
        // PZT-5A k31^2 is typically around 0.10-0.15
        assert!(k2 > 0.05 && k2 < 0.30, "k^2 = {} out of expected range", k2);
    }

    #[test]
    fn test_capacitance() {
        let c = PiezoConfig::pzt_5a();
        let cap = c.capacitance();
        assert!(cap > 0.0);
        // C = epsilon * L * W / t
        let expected = c.epsilon_33 * c.length_m * c.width_m / c.thickness_m;
        assert!(approx_eq(cap, expected, EPSILON));
    }

    // --- CantileverBeam tests ---

    #[test]
    fn test_unimorph_natural_frequency() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let f_n = beam.natural_frequency_hz();
        // Typical PZT-5A 30mm cantilever: several hundred Hz
        assert!(f_n > 50.0 && f_n < 5000.0,
            "Natural frequency {} Hz out of expected range", f_n);
    }

    #[test]
    fn test_bimorph_higher_frequency() {
        let c = PiezoConfig::pzt_5a();
        let uni = CantileverBeam::unimorph(&c, 0.01);
        let bi = CantileverBeam::bimorph(&c, 0.01);
        // Bimorph (double thickness) should have higher natural frequency
        assert!(bi.natural_frequency_hz() > uni.natural_frequency_hz());
    }

    #[test]
    fn test_quality_factor() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let q = beam.quality_factor();
        assert!(approx_eq(q, 50.0, 1e-6), "Q = 1/(2*0.01) = 50");
    }

    #[test]
    fn test_bandwidth_3db() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let bw = beam.bandwidth_3db();
        let expected = beam.natural_frequency_hz() / beam.quality_factor();
        assert!(approx_eq(bw, expected, 1e-6));
    }

    #[test]
    fn test_figure_of_merit() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let fom = beam.figure_of_merit(&c);
        let expected = c.coupling_k_squared() * beam.quality_factor();
        assert!(approx_eq(fom, expected, 1e-10));
        assert!(fom > 0.0);
    }

    #[test]
    fn test_beam_mass_positive() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.02);
        assert!(beam.mass_eff > 0.0);
        assert!(beam.stiffness > 0.0);
        assert!(beam.damping > 0.0);
    }

    #[test]
    fn test_beam_damping_ratio_preserved() {
        let c = PiezoConfig::pzt_5a();
        let zeta = 0.035;
        let beam = CantileverBeam::unimorph(&c, zeta);
        assert!(approx_eq(beam.damping_ratio, zeta, EPSILON));
    }

    // --- EquivalentCircuit tests ---

    #[test]
    fn test_equivalent_circuit_resonance() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let circuit = EquivalentCircuit::from_beam(&beam);

        let f_circuit = circuit.resonance_hz();
        let f_beam = beam.natural_frequency_hz();
        // Circuit resonance should match beam natural frequency
        assert!(relative_eq(f_circuit, f_beam, 0.01),
            "Circuit: {} Hz, Beam: {} Hz", f_circuit, f_beam);
    }

    #[test]
    fn test_equivalent_circuit_parameters_positive() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let circuit = EquivalentCircuit::from_beam(&beam);
        assert!(circuit.l_m > 0.0);
        assert!(circuit.c_m > 0.0);
        assert!(circuit.r_m > 0.0);
        assert!(circuit.c_p > 0.0);
    }

    #[test]
    fn test_impedance_at_resonance() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let circuit = EquivalentCircuit::from_beam(&beam);

        let f_r = circuit.resonance_hz();
        let z_at_res = circuit.impedance_magnitude(f_r);
        let z_off = circuit.impedance_magnitude(f_r * 0.5);

        // At resonance, impedance should be at a minimum (motional branch becomes resistive)
        assert!(z_at_res < z_off,
            "Z at resonance ({}) should be less than off-resonance ({})",
            z_at_res, z_off);
    }

    // --- PowerEstimator tests ---

    #[test]
    fn test_optimal_load_resistance() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let est = PowerEstimator::new(&c, &beam);

        let r_opt = est.optimal_load_resistance();
        let expected = 1.0 / (beam.omega_n * beam.capacitance);
        assert!(approx_eq(r_opt, expected, 1e-6));
        assert!(r_opt > 0.0);
    }

    #[test]
    fn test_open_circuit_voltage() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let est = PowerEstimator::new(&c, &beam);

        let v_oc = est.open_circuit_voltage(1.0);
        assert!(v_oc > 0.0, "Open circuit voltage should be positive");
    }

    #[test]
    fn test_short_circuit_current() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let est = PowerEstimator::new(&c, &beam);

        let i_sc = est.short_circuit_current(1.0);
        assert!(i_sc > 0.0, "Short circuit current should be positive");
    }

    #[test]
    fn test_max_power_positive() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let est = PowerEstimator::new(&c, &beam);

        let p = est.max_power_at_resonance(1.0);
        assert!(p > 0.0, "Max power should be positive");
    }

    #[test]
    fn test_power_scales_with_acceleration_squared() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let est = PowerEstimator::new(&c, &beam);

        let p1 = est.max_power_at_resonance(1.0);
        let p2 = est.max_power_at_resonance(2.0);
        // Power proportional to acceleration^2
        assert!(relative_eq(p2 / p1, 4.0, 0.01),
            "Power ratio: {} (expected 4.0)", p2 / p1);
    }

    #[test]
    fn test_power_density() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let est = PowerEstimator::new(&c, &beam);

        let pd = est.power_density(1.0);
        let expected = est.max_power_at_resonance(1.0) / c.volume();
        assert!(approx_eq(pd, expected, 1e-15));
    }

    #[test]
    fn test_power_peaks_at_resonance() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let est = PowerEstimator::new(&c, &beam);
        let r_opt = est.optimal_load_resistance();
        let f_n = beam.natural_frequency_hz();

        let p_res = est.power_at_frequency(1.0, f_n, r_opt);
        let p_off = est.power_at_frequency(1.0, f_n * 0.5, r_opt);
        assert!(p_res > p_off,
            "Power at resonance ({}) should exceed off-resonance ({})", p_res, p_off);
    }

    // --- FrequencyResponse tests ---

    #[test]
    fn test_displacement_peaks_at_resonance() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);

        let x_res = FrequencyResponse::displacement(&beam, beam.omega_n, 1.0);
        let x_off = FrequencyResponse::displacement(&beam, beam.omega_n * 0.5, 1.0);
        assert!(x_res > x_off);
    }

    #[test]
    fn test_frequency_sweep_correct_length() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let sweep = FrequencyResponse::sweep(&beam, 10.0, 1000.0, 100, 1.0, 10000.0);
        assert_eq!(sweep.len(), 100);
    }

    #[test]
    fn test_frequency_sweep_monotonic_frequency() {
        let c = PiezoConfig::pzt_5a();
        let beam = CantileverBeam::unimorph(&c, 0.01);
        let sweep = FrequencyResponse::sweep(&beam, 10.0, 1000.0, 50, 1.0, 10000.0);
        for i in 1..sweep.len() {
            assert!(sweep[i].0 > sweep[i - 1].0);
        }
    }

    // --- RectifierCircuit tests ---

    #[test]
    fn test_full_bridge_dc_voltage() {
        let rect = RectifierCircuit::full_bridge();
        let v_dc = rect.dc_voltage(5.0);
        // V_dc = V_peak - 2*V_diode = 5.0 - 0.6 = 4.4
        assert!(approx_eq(v_dc, 4.4, 1e-10));
    }

    #[test]
    fn test_full_bridge_low_voltage_clamp() {
        let rect = RectifierCircuit::full_bridge();
        let v_dc = rect.dc_voltage(0.5);
        // V_peak (0.5) < 2*V_diode (0.6), should clamp to 0
        assert!(approx_eq(v_dc, 0.0, 1e-10));
    }

    #[test]
    fn test_voltage_doubler() {
        let rect = RectifierCircuit::voltage_doubler();
        let v_dc = rect.dc_voltage(5.0);
        // V_dc = 2*(V_peak - V_diode) = 2*(5.0 - 0.3) = 9.4
        assert!(approx_eq(v_dc, 9.4, 1e-10));
    }

    #[test]
    fn test_sshi_boost() {
        let rect = RectifierCircuit::sshi(50.0);
        let boost = rect.sshi_boost_factor();
        let expected = 2.0 * 50.0 / PI;
        assert!(approx_eq(boost, expected, 1e-10));
        assert!(boost > 30.0, "SSHI with Q=50 should give significant boost");
    }

    #[test]
    fn test_rectifier_efficiency() {
        let rect = RectifierCircuit::full_bridge();
        let eff = rect.efficiency(10.0);
        assert!(eff > 0.0 && eff < 1.0);
        // 1 - 2*0.3/10 = 0.94
        assert!(approx_eq(eff, 0.94, 1e-10));
    }

    #[test]
    fn test_rectifier_efficiency_zero_input() {
        let rect = RectifierCircuit::full_bridge();
        assert!(approx_eq(rect.efficiency(0.0), 0.0, EPSILON));
    }

    #[test]
    fn test_dc_power() {
        let rect = RectifierCircuit::full_bridge();
        let p_dc = rect.dc_power(1.0, 10.0);
        assert!(approx_eq(p_dc, 0.94, 1e-10));
    }

    // --- EnergyStorage tests ---

    #[test]
    fn test_voltage_at_time_zero() {
        let es = EnergyStorage::new(100e-6, 1000.0);
        let v = es.voltage_at_time(3.3, 0.0);
        assert!(approx_eq(v, 0.0, 1e-10));
    }

    #[test]
    fn test_voltage_at_time_large() {
        let es = EnergyStorage::new(100e-6, 1000.0);
        let v = es.voltage_at_time(3.3, 100.0); // well beyond 5*RC
        assert!(approx_eq(v, 3.3, 1e-3));
    }

    #[test]
    fn test_time_constant() {
        let es = EnergyStorage::new(100e-6, 1000.0);
        let tau = es.time_constant();
        assert!(approx_eq(tau, 0.1, 1e-10));
    }

    #[test]
    fn test_energy_stored() {
        let es = EnergyStorage::new(100e-6, 1000.0);
        let e = es.energy_at_voltage(3.3);
        // E = 0.5 * 100e-6 * 3.3^2 = 5.445e-4
        assert!(approx_eq(e, 0.5 * 100e-6 * 3.3 * 3.3, 1e-10));
    }

    #[test]
    fn test_time_to_threshold() {
        let es = EnergyStorage::new(100e-6, 1000.0);
        let t = es.time_to_threshold(5.0, 3.3).unwrap();
        assert!(t > 0.0);
        // Check: V(t) should equal threshold
        let v = es.voltage_at_time(5.0, t);
        assert!(approx_eq(v, 3.3, 1e-6));
    }

    #[test]
    fn test_time_to_threshold_impossible() {
        let es = EnergyStorage::new(100e-6, 1000.0);
        assert!(es.time_to_threshold(3.0, 5.0).is_none());
    }

    #[test]
    fn test_simulate_charging() {
        let mut es = EnergyStorage::new(100e-6, 1000.0);
        let trace = es.simulate_charging(3.3, 0.001, 1000);
        assert_eq!(trace.len(), 1000);
        // Should monotonically increase
        for i in 1..trace.len() {
            assert!(trace[i] >= trace[i - 1]);
        }
        // Should approach 3.3V at the end (10 time constants)
        assert!(trace.last().unwrap().abs() > 3.0);
    }

    // --- VibrationAnalyzer tests ---

    #[test]
    fn test_rms_acceleration() {
        let samples: Vec<f64> = (0..1000)
            .map(|i| (2.0 * PI * 100.0 * i as f64 / 1000.0).sin())
            .collect();
        let rms = VibrationAnalyzer::rms_acceleration(&samples);
        // RMS of sinusoid = 1/sqrt(2) ~ 0.707
        assert!(relative_eq(rms, 1.0 / 2.0_f64.sqrt(), 0.02));
    }

    #[test]
    fn test_rms_empty() {
        assert!(approx_eq(VibrationAnalyzer::rms_acceleration(&[]), 0.0, EPSILON));
    }

    #[test]
    fn test_peak_acceleration() {
        let samples = vec![0.5, -1.2, 0.8, 1.0, -0.3];
        let peak = VibrationAnalyzer::peak_acceleration(&samples);
        assert!(approx_eq(peak, 1.2, EPSILON));
    }

    #[test]
    fn test_crest_factor_sine() {
        let samples: Vec<f64> = (0..10000)
            .map(|i| (2.0 * PI * 50.0 * i as f64 / 10000.0).sin())
            .collect();
        let cf = VibrationAnalyzer::crest_factor(&samples);
        // Crest factor of sine = sqrt(2) ~ 1.414
        assert!(relative_eq(cf, 2.0_f64.sqrt(), 0.02));
    }

    #[test]
    fn test_dominant_frequency() {
        let fs = 1000.0;
        let f_target = 120.0;
        let n = 1024;
        let samples: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f_target * i as f64 / fs).sin())
            .collect();
        let f_dom = VibrationAnalyzer::dominant_frequency(&samples, fs);
        // Should be close to 120 Hz (within one FFT bin)
        let df = fs / n as f64;
        assert!(
            (f_dom - f_target).abs() < 2.0 * df,
            "Dominant freq {} Hz not close to {} Hz (bin width {})",
            f_dom, f_target, df
        );
    }

    #[test]
    fn test_psd_length() {
        let samples = vec![0.0; 256];
        let (freqs, psd) = VibrationAnalyzer::psd(&samples, 1000.0);
        assert_eq!(freqs.len(), psd.len());
        assert_eq!(freqs.len(), 129); // 256/2 + 1
    }

    // --- BroadbandArray tests ---

    #[test]
    fn test_broadband_array_design() {
        let c = PiezoConfig::pzt_5a();
        let array = BroadbandArray::design(&c, 0.01, 50.0, 200.0, 5);
        assert_eq!(array.num_beams(), 5);
        assert_eq!(array.frequencies.len(), 5);
    }

    #[test]
    fn test_broadband_array_spans_range() {
        let c = PiezoConfig::pzt_5a();
        let array = BroadbandArray::design(&c, 0.01, 50.0, 200.0, 5);
        // First and last beams should be near the range boundaries
        assert!(array.frequencies[0] < 80.0,
            "First beam freq {} should be near 50 Hz", array.frequencies[0]);
        assert!(*array.frequencies.last().unwrap() > 150.0,
            "Last beam freq {} should be near 200 Hz",
            array.frequencies.last().unwrap());
    }

    #[test]
    fn test_bandwidth_enhancement() {
        let c = PiezoConfig::pzt_5a();
        let array = BroadbandArray::design(&c, 0.01, 50.0, 200.0, 5);
        let enh = array.bandwidth_enhancement();
        // N beams give roughly N*x bandwidth enhancement (varies with beam length scaling)
        assert!(enh > 1.0,
            "Bandwidth enhancement {} should be > 1.0 for multi-beam array", enh);
    }

    #[test]
    fn test_total_power_positive() {
        let c = PiezoConfig::pzt_5a();
        let array = BroadbandArray::design(&c, 0.01, 50.0, 200.0, 3);
        let p = array.total_power_at_freq(&c, 0.01, 100.0, 1.0);
        assert!(p > 0.0, "Total array power should be positive");
    }

    // --- FFT helper test ---

    #[test]
    fn test_fft_impulse() {
        // FFT of impulse [1, 0, 0, 0] = [1, 1, 1, 1]
        let mut re = vec![1.0, 0.0, 0.0, 0.0];
        let mut im = vec![0.0, 0.0, 0.0, 0.0];
        fft_in_place(&mut re, &mut im);
        for i in 0..4 {
            assert!(approx_eq(re[i], 1.0, 1e-10), "re[{}] = {}", i, re[i]);
            assert!(approx_eq(im[i], 0.0, 1e-10), "im[{}] = {}", i, im[i]);
        }
    }

    #[test]
    fn test_fft_dc() {
        // FFT of [1, 1, 1, 1] = [4, 0, 0, 0]
        let mut re = vec![1.0, 1.0, 1.0, 1.0];
        let mut im = vec![0.0, 0.0, 0.0, 0.0];
        fft_in_place(&mut re, &mut im);
        assert!(approx_eq(re[0], 4.0, 1e-10));
        for i in 1..4 {
            assert!(approx_eq(re[i], 0.0, 1e-10), "re[{}] = {}", i, re[i]);
            assert!(approx_eq(im[i], 0.0, 1e-10), "im[{}] = {}", i, im[i]);
        }
    }
}

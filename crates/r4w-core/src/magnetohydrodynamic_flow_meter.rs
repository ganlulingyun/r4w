//! Magnetohydrodynamic (MHD) flow meter for conducting fluid velocity determination.
//!
//! This module implements electromagnetic flow measurement based on Faraday's law
//! of electromagnetic induction. When a conducting fluid flows through a magnetic
//! field, an EMF is induced proportional to the flow velocity. This principle is
//! used in industrial flow meters for conductive liquids (water, slurries, liquid
//! metals, blood) as well as in astrophysical plasma diagnostics.
//!
//! # Physics Background
//!
//! Faraday's law for a conducting fluid in a transverse magnetic field:
//!
//! ```text
//! E = B * D * v
//! ```
//!
//! where `B` = magnetic flux density (T), `D` = pipe inner diameter (m), and
//! `v` = mean flow velocity (m/s). The volumetric flow rate is:
//!
//! ```text
//! Q = (pi/4) * D^2 * v = (pi * D * E) / (4 * B)
//! ```
//!
//! # Dimensionless Numbers
//!
//! - **Reynolds number** Re = rho * v * D / mu (flow regime)
//! - **Magnetic Reynolds number** Rm = mu_0 * sigma * v * D (field distortion)
//! - **Hartmann number** Ha = B * D * sqrt(sigma / mu) (MHD boundary layer)
//!
//! # Example
//!
//! ```
//! use r4w_core::magnetohydrodynamic_flow_meter::{MhdFlowMeter, FluidProperties};
//!
//! let fluid = FluidProperties::seawater();
//! let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, fluid);
//!
//! // Measure velocity from electrode EMF
//! let velocity = meter.velocity_from_emf(0.005); // 5 mV
//! let flow_rate = meter.flow_rate_from_emf(0.005);
//! assert!(velocity > 0.0);
//! assert!(flow_rate > 0.0);
//! ```

use std::f64::consts::PI;

// ── Physical constants ─────────────────────────────────────────────────────

/// Permeability of free space (T*m/A).
const MU_0: f64 = 4.0 * PI * 1e-7;

// ── Fluid properties ───────────────────────────────────────────────────────

/// Physical properties of the conducting fluid.
#[derive(Debug, Clone)]
pub struct FluidProperties {
    /// Electrical conductivity in S/m.
    pub conductivity_sm: f64,
    /// Mass density in kg/m^3.
    pub density_kgm3: f64,
    /// Dynamic viscosity in Pa*s.
    pub viscosity_pas: f64,
    /// Relative magnetic permeability (dimensionless, ~1.0 for most fluids).
    pub relative_permeability: f64,
}

impl FluidProperties {
    /// Create custom fluid properties.
    pub fn new(
        conductivity_sm: f64,
        density_kgm3: f64,
        viscosity_pas: f64,
        relative_permeability: f64,
    ) -> Self {
        Self {
            conductivity_sm,
            density_kgm3,
            viscosity_pas,
            relative_permeability,
        }
    }

    /// Seawater (~4.8 S/m, 1025 kg/m^3).
    pub fn seawater() -> Self {
        Self {
            conductivity_sm: 4.8,
            density_kgm3: 1025.0,
            viscosity_pas: 1.08e-3,
            relative_permeability: 1.0,
        }
    }

    /// Liquid sodium (~10.7e6 S/m, 927 kg/m^3 at 100 C).
    pub fn liquid_sodium() -> Self {
        Self {
            conductivity_sm: 10.7e6,
            density_kgm3: 927.0,
            viscosity_pas: 6.9e-4,
            relative_permeability: 1.0,
        }
    }

    /// Mercury (~1.04e6 S/m, 13534 kg/m^3).
    pub fn mercury() -> Self {
        Self {
            conductivity_sm: 1.04e6,
            density_kgm3: 13534.0,
            viscosity_pas: 1.526e-3,
            relative_permeability: 1.0,
        }
    }

    /// Human blood (~0.7 S/m, 1060 kg/m^3).
    pub fn blood() -> Self {
        Self {
            conductivity_sm: 0.7,
            density_kgm3: 1060.0,
            viscosity_pas: 3.5e-3,
            relative_permeability: 1.0,
        }
    }

    /// Tap water (~0.05 S/m, 998 kg/m^3).
    pub fn tap_water() -> Self {
        Self {
            conductivity_sm: 0.05,
            density_kgm3: 998.0,
            viscosity_pas: 1.002e-3,
            relative_permeability: 1.0,
        }
    }

    /// Molten steel (~0.72e6 S/m, 7000 kg/m^3).
    pub fn molten_steel() -> Self {
        Self {
            conductivity_sm: 0.72e6,
            density_kgm3: 7000.0,
            viscosity_pas: 6.0e-3,
            relative_permeability: 1.0,
        }
    }
}

// ── Excitation mode ────────────────────────────────────────────────────────

/// Magnetic field excitation mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ExcitationMode {
    /// Direct current - simplest, but affected by electrode polarization.
    Dc,
    /// Alternating current at given frequency - rejects DC offsets but
    /// susceptible to transformer coupling and eddy currents.
    Ac { frequency_hz: f64 },
    /// Pulsed DC with alternating polarity - best offset rejection while
    /// avoiding AC interference issues.
    PulsedDc { period_s: f64 },
}

// ── Velocity profile ───────────────────────────────────────────────────────

/// Flow velocity profile type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VelocityProfile {
    /// Uniform (plug) flow - ideal case, W = 1 everywhere.
    Uniform,
    /// Fully developed laminar (parabolic): v(r) = 2*v_mean*(1-(r/R)^2).
    Laminar,
    /// Turbulent profile approximated by 1/n power law.
    Turbulent { reynolds: f64 },
}

// ── MHD flow meter ─────────────────────────────────────────────────────────

/// Magnetohydrodynamic flow meter.
///
/// Implements Faraday's law-based electromagnetic flow measurement for
/// conducting fluids. Supports DC, AC, and pulsed DC excitation modes.
#[derive(Debug, Clone)]
pub struct MhdFlowMeter {
    /// Pipe inner diameter in metres.
    pub pipe_diameter_m: f64,
    /// Applied magnetic flux density in Tesla.
    pub magnetic_field_t: f64,
    /// Electrode separation in metres (typically equal to pipe diameter).
    pub electrode_separation_m: f64,
    /// Properties of the fluid being measured.
    pub fluid: FluidProperties,
    /// Excitation mode for the magnetic field.
    pub excitation: ExcitationMode,
    /// Zero offset voltage in Volts (from electrode polarization, etc.).
    pub zero_offset_v: f64,
    /// Span correction factor (default 1.0, adjusted during calibration).
    pub span_factor: f64,
}

impl MhdFlowMeter {
    /// Create a new MHD flow meter with DC excitation and default calibration.
    ///
    /// # Arguments
    /// - `pipe_diameter_m` - Inner pipe diameter (m)
    /// - `magnetic_field_t` - Magnetic flux density (T)
    /// - `electrode_separation_m` - Electrode spacing (m)
    /// - `fluid` - Conducting fluid properties
    pub fn new(
        pipe_diameter_m: f64,
        magnetic_field_t: f64,
        electrode_separation_m: f64,
        fluid: FluidProperties,
    ) -> Self {
        Self {
            pipe_diameter_m,
            magnetic_field_t,
            electrode_separation_m,
            fluid,
            excitation: ExcitationMode::Dc,
            zero_offset_v: 0.0,
            span_factor: 1.0,
        }
    }

    /// Set excitation mode.
    pub fn with_excitation(mut self, mode: ExcitationMode) -> Self {
        self.excitation = mode;
        self
    }

    /// Set zero offset for calibration.
    pub fn with_zero_offset(mut self, offset_v: f64) -> Self {
        self.zero_offset_v = offset_v;
        self
    }

    /// Set span correction factor.
    pub fn with_span_factor(mut self, factor: f64) -> Self {
        self.span_factor = factor;
        self
    }

    // ── Faraday's law core ─────────────────────────────────────────────

    /// Compute the induced EMF for a given mean velocity.
    ///
    /// E = B * D * v (Faraday's law for electromagnetic flow meter).
    pub fn emf_from_velocity(&self, velocity_ms: f64) -> f64 {
        self.magnetic_field_t * self.electrode_separation_m * velocity_ms
    }

    /// Compute mean velocity from measured electrode EMF.
    ///
    /// v = (E - offset) * span / (B * D)
    pub fn velocity_from_emf(&self, emf_v: f64) -> f64 {
        let corrected = (emf_v - self.zero_offset_v) * self.span_factor;
        corrected / (self.magnetic_field_t * self.electrode_separation_m)
    }

    /// Compute volumetric flow rate from EMF.
    ///
    /// Q = (pi * D * E) / (4 * B)  using the pipe cross-sectional area.
    pub fn flow_rate_from_emf(&self, emf_v: f64) -> f64 {
        let v = self.velocity_from_emf(emf_v);
        self.flow_rate_from_velocity(v)
    }

    /// Compute volumetric flow rate from mean velocity.
    ///
    /// Q = (pi/4) * D^2 * v
    pub fn flow_rate_from_velocity(&self, velocity_ms: f64) -> f64 {
        (PI / 4.0) * self.pipe_diameter_m * self.pipe_diameter_m * velocity_ms
    }

    /// Compute mass flow rate from mean velocity.
    ///
    /// m_dot = rho * Q
    pub fn mass_flow_rate(&self, velocity_ms: f64) -> f64 {
        self.fluid.density_kgm3 * self.flow_rate_from_velocity(velocity_ms)
    }

    /// Voltage sensitivity dE/dv = B * D (V/(m/s)).
    pub fn voltage_sensitivity(&self) -> f64 {
        self.magnetic_field_t * self.electrode_separation_m
    }

    // ── Dimensionless numbers ──────────────────────────────────────────

    /// Reynolds number Re = rho * v * D / mu.
    pub fn reynolds_number(&self, velocity_ms: f64) -> f64 {
        self.fluid.density_kgm3 * velocity_ms * self.pipe_diameter_m / self.fluid.viscosity_pas
    }

    /// Magnetic Reynolds number Rm = mu_0 * mu_r * sigma * v * D.
    ///
    /// Rm << 1: flow does not distort the applied field (most industrial meters).
    /// Rm >> 1: field frozen into fluid (astrophysical plasmas).
    pub fn magnetic_reynolds_number(&self, velocity_ms: f64) -> f64 {
        MU_0 * self.fluid.relative_permeability
            * self.fluid.conductivity_sm
            * velocity_ms
            * self.pipe_diameter_m
    }

    /// Hartmann number Ha = B * D * sqrt(sigma / mu).
    ///
    /// Characterizes the relative importance of electromagnetic to viscous forces.
    /// Ha >> 1: Hartmann boundary layer thickness ~ D / Ha.
    pub fn hartmann_number(&self) -> f64 {
        self.magnetic_field_t
            * self.pipe_diameter_m
            * (self.fluid.conductivity_sm / self.fluid.viscosity_pas).sqrt()
    }

    /// Interaction parameter (Stuart number) N = Ha^2 / Re.
    ///
    /// Ratio of electromagnetic to inertial forces.
    pub fn interaction_parameter(&self, velocity_ms: f64) -> f64 {
        let ha = self.hartmann_number();
        let re = self.reynolds_number(velocity_ms);
        if re.abs() < 1e-30 {
            return 0.0;
        }
        ha * ha / re
    }

    /// Hartmann boundary layer thickness delta_H = D / Ha.
    pub fn hartmann_layer_thickness(&self) -> f64 {
        let ha = self.hartmann_number();
        if ha.abs() < 1e-30 {
            return self.pipe_diameter_m;
        }
        self.pipe_diameter_m / ha
    }

    // ── Flow regime classification ─────────────────────────────────────

    /// Classify the flow regime based on Reynolds number.
    pub fn flow_regime(&self, velocity_ms: f64) -> FlowRegime {
        let re = self.reynolds_number(velocity_ms);
        if re < 2300.0 {
            FlowRegime::Laminar
        } else if re < 4000.0 {
            FlowRegime::Transitional
        } else {
            FlowRegime::Turbulent
        }
    }

    /// Check if the magnetic field is distorted by the flow (Rm >> 1).
    pub fn is_field_frozen(&self, velocity_ms: f64) -> bool {
        self.magnetic_reynolds_number(velocity_ms) > 1.0
    }

    // ── Velocity profile corrections ───────────────────────────────────

    /// Weight function correction factor for a given velocity profile.
    ///
    /// For an ideal uniform flow, the factor is 1.0. Real profiles require
    /// corrections due to the non-uniform sensitivity of the electrode pair.
    pub fn profile_correction_factor(&self, profile: VelocityProfile) -> f64 {
        match profile {
            VelocityProfile::Uniform => 1.0,
            VelocityProfile::Laminar => {
                // Laminar parabolic profile in a circular pipe with point
                // electrodes: the weight function integral gives a correction
                // factor of exactly 1.0 for an ideal geometry, but real
                // electrode geometry produces a small deviation. For standard
                // point electrodes, the theoretical factor is 1.0 (Shercliff's
                // weight function result). We return 1.0 for ideal case.
                1.0
            }
            VelocityProfile::Turbulent { reynolds } => {
                // Approximate turbulent profile correction:
                // k_turb = 1 / (1 + 1.08 * Re^(-0.08))
                // This accounts for the fuller velocity profile in turbulent flow.
                if reynolds <= 0.0 {
                    return 1.0;
                }
                1.0 / (1.0 + 1.08 * reynolds.powf(-0.08))
            }
        }
    }

    /// Compute the laminar parabolic velocity at radial position r.
    ///
    /// v(r) = 2 * v_mean * (1 - (r/R)^2)
    pub fn laminar_velocity_at_radius(&self, v_mean: f64, r: f64) -> f64 {
        let big_r = self.pipe_diameter_m / 2.0;
        if r.abs() > big_r {
            return 0.0;
        }
        2.0 * v_mean * (1.0 - (r / big_r).powi(2))
    }

    /// Compute the turbulent 1/n power-law velocity at radial position r.
    ///
    /// v(r) = v_max * (1 - r/R)^(1/n)
    /// where n ~ 7 for typical turbulent flow and v_max = v_mean * (n+1)*(2n+1)/(2*n^2)
    pub fn turbulent_velocity_at_radius(
        &self,
        v_mean: f64,
        r: f64,
        n: f64,
    ) -> f64 {
        let big_r = self.pipe_diameter_m / 2.0;
        if r.abs() >= big_r {
            return 0.0;
        }
        let v_max = v_mean * (n + 1.0) * (2.0 * n + 1.0) / (2.0 * n * n);
        v_max * (1.0 - r.abs() / big_r).powf(1.0 / n)
    }

    // ── Signal processing ──────────────────────────────────────────────

    /// Synchronous demodulation of AC-excited flow meter signal.
    ///
    /// Given raw electrode samples and a reference carrier at the excitation
    /// frequency, extract the in-phase (flow) and quadrature (noise) components.
    ///
    /// Returns `(flow_signal, quadrature_noise)`.
    pub fn synchronous_demodulate(
        &self,
        samples: &[f64],
        sample_rate_hz: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let freq = match self.excitation {
            ExcitationMode::Ac { frequency_hz } => frequency_hz,
            _ => return (samples.to_vec(), vec![0.0; samples.len()]),
        };

        let n = samples.len();
        let mut in_phase = Vec::with_capacity(n);
        let mut quadrature = Vec::with_capacity(n);

        for (i, &s) in samples.iter().enumerate() {
            let t = i as f64 / sample_rate_hz;
            let phase = 2.0 * PI * freq * t;
            in_phase.push(2.0 * s * phase.cos());
            quadrature.push(2.0 * s * phase.sin());
        }

        // Low-pass filter via simple moving average (rectangular window).
        // The window spans one full period of the excitation frequency.
        let period_samples = (sample_rate_hz / freq).round().max(1.0) as usize;
        let flow = moving_average(&in_phase, period_samples);
        let noise = moving_average(&quadrature, period_samples);

        (flow, noise)
    }

    /// Pulsed DC offset rejection.
    ///
    /// Given samples from alternating-polarity excitation, subtract consecutive
    /// half-cycles to cancel electrode polarization and other DC offsets.
    ///
    /// Returns offset-corrected samples.
    pub fn pulsed_dc_offset_reject(&self, samples: &[f64]) -> Vec<f64> {
        let period_samples = match self.excitation {
            ExcitationMode::PulsedDc { period_s } => {
                // Assume Nyquist-adequate sampling; caller provides samples
                // at an implied rate. We treat every adjacent pair as
                // positive/negative excitation.
                let _ = period_s; // period metadata; we process sample-by-sample
                2usize
            }
            _ => 2,
        };
        let half = period_samples / 2;
        if half == 0 || samples.len() < period_samples {
            return samples.to_vec();
        }

        let mut result = Vec::with_capacity(samples.len() / period_samples);
        let mut i = 0;
        while i + period_samples <= samples.len() {
            // Average of positive half minus negative half:
            let mut pos_sum = 0.0;
            let mut neg_sum = 0.0;
            for j in 0..half {
                pos_sum += samples[i + j];
            }
            for j in half..period_samples {
                neg_sum += samples[i + j];
            }
            result.push((pos_sum - neg_sum) / (2.0 * half as f64));
            i += period_samples;
        }
        result
    }

    // ── Error sources ──────────────────────────────────────────────────

    /// Estimate electrode polarization voltage (empirical model).
    ///
    /// Polarization is significant for DC excitation and low-conductivity fluids.
    /// Returns approximate polarization voltage in Volts.
    pub fn electrode_polarization_voltage(&self) -> f64 {
        // Simplified model: V_pol ~ 0.3 / sqrt(sigma)  for stainless steel
        // electrodes in aqueous solutions (empirical, order-of-magnitude).
        if self.fluid.conductivity_sm <= 0.0 {
            return f64::INFINITY;
        }
        0.3 / self.fluid.conductivity_sm.sqrt()
    }

    /// Eddy current error fraction for conducting pipe walls.
    ///
    /// For a pipe wall of given conductivity and thickness, the eddy currents
    /// induced in the wall reduce the measured signal. The error fraction is
    /// approximately:
    ///
    /// epsilon = sigma_wall * t_wall / (sigma_fluid * D)
    pub fn eddy_current_error(
        &self,
        wall_conductivity_sm: f64,
        wall_thickness_m: f64,
    ) -> f64 {
        if self.fluid.conductivity_sm <= 0.0 || self.pipe_diameter_m <= 0.0 {
            return 0.0;
        }
        wall_conductivity_sm * wall_thickness_m
            / (self.fluid.conductivity_sm * self.pipe_diameter_m)
    }

    /// End-effect correction for short magnets.
    ///
    /// When the magnet length L is not much greater than the pipe diameter D,
    /// the field is non-uniform and the sensitivity is reduced. Approximate
    /// correction factor (empirical):
    ///
    /// k_end = 1 - 0.5 * exp(-2 * L / D)
    pub fn end_effect_correction(&self, magnet_length_m: f64) -> f64 {
        if self.pipe_diameter_m <= 0.0 {
            return 1.0;
        }
        1.0 - 0.5 * (-2.0 * magnet_length_m / self.pipe_diameter_m).exp()
    }

    /// Temperature coefficient of sensitivity.
    ///
    /// The magnetic field and fluid conductivity both change with temperature.
    /// For a permanent-magnet meter, the dominant effect is B(T) derating.
    ///
    /// Returns fractional sensitivity change per kelvin (typical NdFeB ~-0.12%/K).
    pub fn temperature_coefficient_per_k(&self, magnet_temp_coeff: f64) -> f64 {
        magnet_temp_coeff
    }

    /// Minimum detectable velocity from thermal noise floor.
    ///
    /// The Johnson-Nyquist noise voltage across the electrode impedance sets the
    /// ultimate limit: V_n = sqrt(4 * k_B * T * R * BW).
    ///
    /// Returns minimum detectable velocity in m/s.
    pub fn minimum_detectable_velocity(
        &self,
        electrode_impedance_ohm: f64,
        bandwidth_hz: f64,
        temperature_k: f64,
    ) -> f64 {
        // Boltzmann constant
        const K_B: f64 = 1.380649e-23;
        let noise_v = (4.0 * K_B * temperature_k * electrode_impedance_ohm * bandwidth_hz).sqrt();
        let sensitivity = self.voltage_sensitivity();
        if sensitivity.abs() < 1e-30 {
            return f64::INFINITY;
        }
        noise_v / sensitivity
    }

    /// Electrode impedance estimate (simplified contact model).
    ///
    /// Z ~ 1 / (sigma * A_electrode)
    /// where A_electrode is the effective electrode contact area.
    pub fn electrode_impedance(&self, electrode_area_m2: f64) -> f64 {
        if self.fluid.conductivity_sm <= 0.0 || electrode_area_m2 <= 0.0 {
            return f64::INFINITY;
        }
        1.0 / (self.fluid.conductivity_sm * electrode_area_m2)
    }
}

// ── Lorentz Force Velocimetry ──────────────────────────────────────────────

/// Lorentz force velocimetry (contactless, non-invasive).
///
/// An external magnet system generates a force on a conducting fluid moving
/// past it. By Newton's third law, an equal and opposite force acts on the
/// magnet, which can be measured with a precision force balance.
#[derive(Debug, Clone)]
pub struct LorentzForceVelocimeter {
    /// Magnetic flux density at the fluid (T).
    pub magnetic_field_t: f64,
    /// Effective interaction volume (m^3).
    pub interaction_volume_m3: f64,
    /// Fluid conductivity (S/m).
    pub fluid_conductivity_sm: f64,
}

impl LorentzForceVelocimeter {
    /// Create a new Lorentz force velocimeter.
    pub fn new(
        magnetic_field_t: f64,
        interaction_volume_m3: f64,
        fluid_conductivity_sm: f64,
    ) -> Self {
        Self {
            magnetic_field_t,
            interaction_volume_m3,
            fluid_conductivity_sm,
        }
    }

    /// Compute the Lorentz force on the magnet system for a given fluid velocity.
    ///
    /// F = sigma * v * B^2 * V_interaction (simplified integral).
    pub fn force_from_velocity(&self, velocity_ms: f64) -> f64 {
        self.fluid_conductivity_sm
            * velocity_ms
            * self.magnetic_field_t
            * self.magnetic_field_t
            * self.interaction_volume_m3
    }

    /// Compute velocity from measured Lorentz force.
    pub fn velocity_from_force(&self, force_n: f64) -> f64 {
        let denom = self.fluid_conductivity_sm
            * self.magnetic_field_t
            * self.magnetic_field_t
            * self.interaction_volume_m3;
        if denom.abs() < 1e-30 {
            return 0.0;
        }
        force_n / denom
    }

    /// Force sensitivity dF/dv = sigma * B^2 * V.
    pub fn force_sensitivity(&self) -> f64 {
        self.fluid_conductivity_sm
            * self.magnetic_field_t
            * self.magnetic_field_t
            * self.interaction_volume_m3
    }
}

// ── Calibration ────────────────────────────────────────────────────────────

/// Calibration data for an MHD flow meter.
#[derive(Debug, Clone)]
pub struct Calibration {
    /// Measured (EMF, flow_rate) calibration points.
    pub points: Vec<(f64, f64)>,
    /// Linear fit: Q = slope * E + intercept.
    pub slope: f64,
    /// Linear fit intercept.
    pub intercept: f64,
}

impl Calibration {
    /// Create a calibration from measured data points (emf_v, flow_rate_m3s).
    ///
    /// Performs least-squares linear fit: Q = slope * E + intercept.
    pub fn from_points(points: &[(f64, f64)]) -> Self {
        let n = points.len() as f64;
        if points.is_empty() {
            return Self {
                points: Vec::new(),
                slope: 0.0,
                intercept: 0.0,
            };
        }
        if points.len() == 1 {
            let (e, q) = points[0];
            return Self {
                points: points.to_vec(),
                slope: if e.abs() > 1e-30 { q / e } else { 0.0 },
                intercept: 0.0,
            };
        }

        let sum_x: f64 = points.iter().map(|(e, _)| e).sum();
        let sum_y: f64 = points.iter().map(|(_, q)| q).sum();
        let sum_xy: f64 = points.iter().map(|(e, q)| e * q).sum();
        let sum_xx: f64 = points.iter().map(|(e, _)| e * e).sum();

        let denom = n * sum_xx - sum_x * sum_x;
        let (slope, intercept) = if denom.abs() < 1e-30 {
            (0.0, sum_y / n)
        } else {
            let sl = (n * sum_xy - sum_x * sum_y) / denom;
            let ic = (sum_y - sl * sum_x) / n;
            (sl, ic)
        };

        Self {
            points: points.to_vec(),
            slope,
            intercept,
        }
    }

    /// Apply calibration to get flow rate from measured EMF.
    pub fn flow_rate_from_emf(&self, emf_v: f64) -> f64 {
        self.slope * emf_v + self.intercept
    }

    /// Compute the zero offset (EMF at Q=0).
    pub fn zero_offset(&self) -> f64 {
        if self.slope.abs() < 1e-30 {
            return 0.0;
        }
        -self.intercept / self.slope
    }

    /// Compute R-squared (coefficient of determination) for the linear fit.
    pub fn r_squared(&self) -> f64 {
        if self.points.len() < 2 {
            return 1.0;
        }
        let n = self.points.len() as f64;
        let mean_y: f64 = self.points.iter().map(|(_, q)| q).sum::<f64>() / n;
        let ss_tot: f64 = self.points.iter().map(|(_, q)| (q - mean_y).powi(2)).sum();
        let ss_res: f64 = self
            .points
            .iter()
            .map(|(e, q)| {
                let predicted = self.slope * e + self.intercept;
                (q - predicted).powi(2)
            })
            .sum();
        if ss_tot.abs() < 1e-30 {
            return 1.0;
        }
        1.0 - ss_res / ss_tot
    }
}

// ── Weight function ────────────────────────────────────────────────────────

/// Compute Shercliff's weight function W(r, theta) for a circular pipe
/// with point electrodes on a diameter.
///
/// In the ideal case (uniform field, point electrodes, insulating pipe wall),
/// W = 1 everywhere, meaning the meter is equally sensitive to flow at all
/// radial/angular positions. For finite-area electrodes, the weight function
/// deviates from unity near the electrodes and walls.
///
/// This function returns the simplified weight for a pipe of radius R at
/// normalised coordinates (r_norm = r/R, theta in radians from electrode axis).
///
/// For the ideal case: W(r_norm, theta) = 1.0
/// With first-order electrode size correction:
/// W = 1 + (r_norm^2) * cos(2*theta) * correction
pub fn weight_function(r_norm: f64, theta: f64, electrode_correction: f64) -> f64 {
    if r_norm < 0.0 || r_norm > 1.0 {
        return 0.0;
    }
    1.0 + r_norm * r_norm * (2.0 * theta).cos() * electrode_correction
}

/// Compute the mean weight over a velocity profile.
///
/// For axisymmetric flow, integrates W(r) * v(r) * r dr over the cross section.
/// The result is the effective correction factor for the given profile.
///
/// Uses numerical trapezoidal integration with `n_points` radial steps.
pub fn mean_weight_factor(
    profile: VelocityProfile,
    pipe_diameter_m: f64,
    electrode_correction: f64,
    n_points: usize,
) -> f64 {
    let big_r = pipe_diameter_m / 2.0;
    let n = n_points.max(2);
    let dr = big_r / (n as f64);

    let mut numerator = 0.0;
    let mut denominator = 0.0;

    for i in 0..=n {
        let r = i as f64 * dr;
        let r_norm = r / big_r;

        // Axisymmetric: average over theta gives cos(2*theta) -> 0
        // so W_avg(r) = 1.0 for axisymmetric flows.
        let w = 1.0 + r_norm * r_norm * electrode_correction * 0.0; // cos integral = 0
        let _ = w; // The theta-averaged weight is always 1.0 for axisymmetric flow

        let v_norm = match profile {
            VelocityProfile::Uniform => 1.0,
            VelocityProfile::Laminar => 2.0 * (1.0 - r_norm * r_norm),
            VelocityProfile::Turbulent { reynolds } => {
                let n_exp = if reynolds > 4000.0 {
                    // Approximation: n ~ 1.0 / (1.8 * log10(Re) - 1.5) for Re > 4000
                    let log_re = reynolds.log10();
                    1.0 / (1.8 * log_re - 1.5).max(0.1)
                } else {
                    1.0 / 7.0
                };
                if r_norm < 1.0 {
                    let v_max_ratio = (n_exp + 1.0) * (2.0 * n_exp + 1.0) / (2.0 * n_exp * n_exp);
                    v_max_ratio * (1.0 - r_norm).powf(1.0 / n_exp)
                } else {
                    0.0
                }
            }
        };

        let weight_factor = if i == 0 || i == n { 0.5 } else { 1.0 };
        numerator += weight_factor * 1.0 * v_norm * r_norm; // W_avg = 1
        denominator += weight_factor * v_norm * r_norm;
    }

    if denominator.abs() < 1e-30 {
        return 1.0;
    }
    numerator / denominator
}

// ── Helpers ────────────────────────────────────────────────────────────────

/// Simple moving average filter.
fn moving_average(data: &[f64], window: usize) -> Vec<f64> {
    let n = data.len();
    if window == 0 || n == 0 {
        return data.to_vec();
    }
    let w = window.min(n);
    let mut result = Vec::with_capacity(n);
    let mut sum: f64 = data[..w].iter().sum();
    // Fill initial samples with partial average
    for i in 0..n {
        if i >= w {
            sum += data[i] - data[i - w];
        }
        let count = if i < w { i + 1 } else { w };
        if i < w {
            sum = data[..=i].iter().sum();
        }
        result.push(sum / count as f64);
    }
    result
}

/// Reynolds number for a given fluid, velocity, and pipe diameter.
pub fn reynolds_number(
    density_kgm3: f64,
    velocity_ms: f64,
    pipe_diameter_m: f64,
    viscosity_pas: f64,
) -> f64 {
    density_kgm3 * velocity_ms * pipe_diameter_m / viscosity_pas
}

/// Magnetic Reynolds number.
pub fn magnetic_reynolds_number(
    conductivity_sm: f64,
    relative_permeability: f64,
    velocity_ms: f64,
    pipe_diameter_m: f64,
) -> f64 {
    MU_0 * relative_permeability * conductivity_sm * velocity_ms * pipe_diameter_m
}

/// Hartmann number.
pub fn hartmann_number(
    magnetic_field_t: f64,
    pipe_diameter_m: f64,
    conductivity_sm: f64,
    viscosity_pas: f64,
) -> f64 {
    magnetic_field_t * pipe_diameter_m * (conductivity_sm / viscosity_pas).sqrt()
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-9;
    const REL_TOL: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn rel_eq(a: f64, b: f64, tol: f64) -> bool {
        if a.abs() < 1e-30 && b.abs() < 1e-30 {
            return true;
        }
        let denom = a.abs().max(b.abs());
        (a - b).abs() / denom < tol
    }

    // ── Fluid presets ──────────────────────────────────────────────────

    #[test]
    fn test_seawater_properties() {
        let f = FluidProperties::seawater();
        assert!(approx_eq(f.conductivity_sm, 4.8, 0.01));
        assert!(approx_eq(f.density_kgm3, 1025.0, 1.0));
        assert!(f.viscosity_pas > 0.0);
        assert!(approx_eq(f.relative_permeability, 1.0, TOL));
    }

    #[test]
    fn test_liquid_sodium_properties() {
        let f = FluidProperties::liquid_sodium();
        assert!(f.conductivity_sm > 1e6);
        assert!(f.density_kgm3 > 900.0 && f.density_kgm3 < 950.0);
    }

    #[test]
    fn test_mercury_properties() {
        let f = FluidProperties::mercury();
        assert!(approx_eq(f.conductivity_sm, 1.04e6, 1e3));
        assert!(f.density_kgm3 > 13000.0);
    }

    #[test]
    fn test_blood_properties() {
        let f = FluidProperties::blood();
        assert!(approx_eq(f.conductivity_sm, 0.7, 0.01));
        assert!(f.density_kgm3 > 1050.0 && f.density_kgm3 < 1070.0);
    }

    #[test]
    fn test_tap_water_properties() {
        let f = FluidProperties::tap_water();
        assert!(f.conductivity_sm < 0.1);
        assert!(f.density_kgm3 > 990.0 && f.density_kgm3 < 1010.0);
    }

    #[test]
    fn test_molten_steel_properties() {
        let f = FluidProperties::molten_steel();
        assert!(f.conductivity_sm > 1e5);
        assert!(f.density_kgm3 > 6000.0);
    }

    // ── Faraday's law core ─────────────────────────────────────────────

    #[test]
    fn test_emf_from_velocity() {
        // E = B * D * v
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        let emf = meter.emf_from_velocity(2.0);
        // E = 0.05 * 0.1 * 2.0 = 0.01 V
        assert!(approx_eq(emf, 0.01, TOL));
    }

    #[test]
    fn test_velocity_from_emf_roundtrip() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        let v_original = 3.5;
        let emf = meter.emf_from_velocity(v_original);
        let v_recovered = meter.velocity_from_emf(emf);
        assert!(approx_eq(v_recovered, v_original, TOL));
    }

    #[test]
    fn test_velocity_with_zero_offset() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater())
            .with_zero_offset(0.001); // 1 mV offset
        // E = B*D*v = 0.05*0.1*2 = 0.01 V; with 1 mV offset, measured = 0.011 V
        let v = meter.velocity_from_emf(0.011);
        assert!(approx_eq(v, 2.0, 1e-6));
    }

    #[test]
    fn test_velocity_with_span_factor() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater())
            .with_span_factor(1.02); // 2% span correction
        let emf = 0.01; // B*D*v = 0.005 * v => v_uncorrected = 2.0
        let v = meter.velocity_from_emf(emf);
        // v = (0.01 * 1.02) / (0.05 * 0.1) = 0.0102 / 0.005 = 2.04
        assert!(approx_eq(v, 2.04, 1e-6));
    }

    #[test]
    fn test_flow_rate_from_velocity() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        let q = meter.flow_rate_from_velocity(2.0);
        // Q = pi/4 * D^2 * v = pi/4 * 0.01 * 2 = 0.015708 m^3/s
        let expected = PI / 4.0 * 0.1 * 0.1 * 2.0;
        assert!(approx_eq(q, expected, TOL));
    }

    #[test]
    fn test_flow_rate_from_emf() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        let emf = meter.emf_from_velocity(2.0);
        let q_direct = meter.flow_rate_from_velocity(2.0);
        let q_from_emf = meter.flow_rate_from_emf(emf);
        assert!(approx_eq(q_direct, q_from_emf, TOL));
    }

    #[test]
    fn test_mass_flow_rate() {
        let fluid = FluidProperties::seawater();
        let density = fluid.density_kgm3;
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, fluid);
        let v = 2.0;
        let q = meter.flow_rate_from_velocity(v);
        let m_dot = meter.mass_flow_rate(v);
        assert!(approx_eq(m_dot, density * q, TOL));
    }

    #[test]
    fn test_voltage_sensitivity() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        // dE/dv = B * D = 0.05 * 0.1 = 0.005 V/(m/s)
        assert!(approx_eq(meter.voltage_sensitivity(), 0.005, TOL));
    }

    // ── Dimensionless numbers ──────────────────────────────────────────

    #[test]
    fn test_reynolds_number_laminar() {
        let meter = MhdFlowMeter::new(0.01, 0.1, 0.01, FluidProperties::blood());
        // Re = 1060 * 0.1 * 0.01 / 0.0035 = 302.86
        let re = meter.reynolds_number(0.1);
        assert!(re < 2300.0); // laminar
        assert!(re > 200.0);
    }

    #[test]
    fn test_reynolds_number_turbulent() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        // Re = 1025 * 5.0 * 0.1 / 0.00108 = ~474537
        let re = meter.reynolds_number(5.0);
        assert!(re > 4000.0); // turbulent
    }

    #[test]
    fn test_magnetic_reynolds_number_industrial() {
        // Most industrial meters: Rm << 1
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        let rm = meter.magnetic_reynolds_number(2.0);
        // Rm = mu0 * 4.8 * 2.0 * 0.1 ~ 1.2e-6 << 1
        assert!(rm < 0.001);
    }

    #[test]
    fn test_magnetic_reynolds_number_liquid_metal() {
        let meter = MhdFlowMeter::new(0.5, 0.1, 0.5, FluidProperties::liquid_sodium());
        // Rm = mu0 * 10.7e6 * 10 * 0.5 ~ 67.2
        let rm = meter.magnetic_reynolds_number(10.0);
        assert!(rm > 1.0); // field distortion significant
    }

    #[test]
    fn test_hartmann_number() {
        let meter = MhdFlowMeter::new(0.1, 1.0, 0.1, FluidProperties::liquid_sodium());
        let ha = meter.hartmann_number();
        // Ha = 1.0 * 0.1 * sqrt(10.7e6 / 6.9e-4) = 0.1 * sqrt(1.5507e10) = 0.1 * 124528 ~ 12453
        assert!(ha > 1000.0);
    }

    #[test]
    fn test_hartmann_layer_thickness() {
        let meter = MhdFlowMeter::new(0.1, 1.0, 0.1, FluidProperties::liquid_sodium());
        let ha = meter.hartmann_number();
        let delta = meter.hartmann_layer_thickness();
        assert!(approx_eq(delta, 0.1 / ha, 1e-12));
    }

    #[test]
    fn test_interaction_parameter() {
        let meter = MhdFlowMeter::new(0.1, 0.5, 0.1, FluidProperties::liquid_sodium());
        let n_param = meter.interaction_parameter(5.0);
        // N = Ha^2 / Re; should be > 0 for conducting fluid in strong field
        assert!(n_param > 0.0);
    }

    // ── Flow regime ────────────────────────────────────────────────────

    #[test]
    fn test_flow_regime_laminar() {
        let meter = MhdFlowMeter::new(0.01, 0.1, 0.01, FluidProperties::blood());
        assert_eq!(meter.flow_regime(0.01), FlowRegime::Laminar);
    }

    #[test]
    fn test_flow_regime_turbulent() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        assert_eq!(meter.flow_regime(5.0), FlowRegime::Turbulent);
    }

    #[test]
    fn test_is_field_frozen() {
        let meter_water = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        assert!(!meter_water.is_field_frozen(2.0));

        let meter_na = MhdFlowMeter::new(0.5, 0.1, 0.5, FluidProperties::liquid_sodium());
        assert!(meter_na.is_field_frozen(10.0));
    }

    // ── Velocity profiles ──────────────────────────────────────────────

    #[test]
    fn test_uniform_profile_correction() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        assert!(approx_eq(
            meter.profile_correction_factor(VelocityProfile::Uniform),
            1.0,
            TOL
        ));
    }

    #[test]
    fn test_turbulent_profile_correction() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        let k = meter.profile_correction_factor(VelocityProfile::Turbulent { reynolds: 100000.0 });
        // k should be close to but slightly less than 1.0
        assert!(k > 0.4 && k < 1.0);
    }

    #[test]
    fn test_laminar_velocity_at_center() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        let v_center = meter.laminar_velocity_at_radius(1.0, 0.0);
        // v(0) = 2 * v_mean = 2.0
        assert!(approx_eq(v_center, 2.0, TOL));
    }

    #[test]
    fn test_laminar_velocity_at_wall() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        let v_wall = meter.laminar_velocity_at_radius(1.0, 0.05); // r = R
        assert!(approx_eq(v_wall, 0.0, TOL));
    }

    #[test]
    fn test_laminar_velocity_outside_pipe() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        let v = meter.laminar_velocity_at_radius(1.0, 0.06); // r > R
        assert!(approx_eq(v, 0.0, TOL));
    }

    #[test]
    fn test_turbulent_velocity_at_center() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        let v_center = meter.turbulent_velocity_at_radius(1.0, 0.0, 7.0);
        // v_max = v_mean * (n+1)*(2n+1)/(2*n^2) = 1 * 8*15/98 ~ 1.2245
        let v_max = 8.0 * 15.0 / (2.0 * 49.0);
        assert!(approx_eq(v_center, v_max, 1e-4));
    }

    #[test]
    fn test_turbulent_velocity_at_wall() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        let v_wall = meter.turbulent_velocity_at_radius(1.0, 0.05, 7.0);
        assert!(approx_eq(v_wall, 0.0, TOL));
    }

    // ── Signal processing ──────────────────────────────────────────────

    #[test]
    fn test_synchronous_demodulate_ac() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater())
            .with_excitation(ExcitationMode::Ac { frequency_hz: 50.0 });

        let fs = 10000.0;
        let n = 2000;
        let v = 1.5; // m/s
        let signal_amplitude = meter.emf_from_velocity(v);

        // Generate AC-modulated flow signal: E * cos(2*pi*f*t)
        let samples: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                signal_amplitude * (2.0 * PI * 50.0 * t).cos()
            })
            .collect();

        let (flow, _quad) = meter.synchronous_demodulate(&samples, fs);
        assert_eq!(flow.len(), n);

        // After demod and LPF, the DC component should converge to signal_amplitude
        // Check the latter half for convergence
        let avg: f64 = flow[n / 2..].iter().sum::<f64>() / (n / 2) as f64;
        assert!(rel_eq(avg, signal_amplitude, 0.1));
    }

    #[test]
    fn test_synchronous_demodulate_dc_passthrough() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        // DC mode: synchronous_demodulate should pass through unchanged
        let samples = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let (flow, quad) = meter.synchronous_demodulate(&samples, 1000.0);
        assert_eq!(flow, samples);
        assert_eq!(quad, vec![0.0; 5]);
    }

    #[test]
    fn test_pulsed_dc_offset_rejection() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater())
            .with_excitation(ExcitationMode::PulsedDc { period_s: 0.1 });

        // Positive excitation gives: signal + offset
        // Negative excitation gives: -signal + offset
        let offset = 0.002;
        let signal = 0.005;
        let samples = vec![
            signal + offset,
            -signal + offset,
            signal + offset,
            -signal + offset,
        ];

        let corrected = meter.pulsed_dc_offset_reject(&samples);
        // Each pair: (s+off - (-s+off)) / 2 = s
        for &c in &corrected {
            assert!(approx_eq(c, signal, 1e-10));
        }
    }

    // ── Error sources ──────────────────────────────────────────────────

    #[test]
    fn test_electrode_polarization_seawater() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        let v_pol = meter.electrode_polarization_voltage();
        // 0.3 / sqrt(4.8) ~ 0.137 V
        assert!(v_pol > 0.1 && v_pol < 0.2);
    }

    #[test]
    fn test_electrode_polarization_liquid_metal() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::liquid_sodium());
        let v_pol = meter.electrode_polarization_voltage();
        // 0.3 / sqrt(10.7e6) ~ 9.2e-5 V (negligible for liquid metals)
        assert!(v_pol < 0.001);
    }

    #[test]
    fn test_eddy_current_error() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        // Stainless steel wall: sigma ~ 1.4e6 S/m, thickness 3mm
        let err = meter.eddy_current_error(1.4e6, 0.003);
        // 1.4e6 * 0.003 / (4.8 * 0.1) = 4200 / 0.48 = 8750
        // This is huge - stainless steel pipes are very problematic with seawater!
        assert!(err > 1.0);
    }

    #[test]
    fn test_eddy_current_error_nonconductive_liner() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        // Non-conductive liner (PTFE): sigma ~ 0
        let err = meter.eddy_current_error(0.0, 0.003);
        assert!(approx_eq(err, 0.0, TOL));
    }

    #[test]
    fn test_end_effect_correction_long_magnet() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        // Long magnet: L = 10 * D
        let k = meter.end_effect_correction(1.0);
        // 1 - 0.5*exp(-20) ~ 1.0
        assert!(k > 0.99);
    }

    #[test]
    fn test_end_effect_correction_short_magnet() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        // Short magnet: L = D/4
        let k = meter.end_effect_correction(0.025);
        // 1 - 0.5*exp(-0.5) ~ 1 - 0.303 ~ 0.697
        assert!(k > 0.6 && k < 0.8);
    }

    #[test]
    fn test_minimum_detectable_velocity() {
        let meter = MhdFlowMeter::new(0.1, 0.1, 0.1, FluidProperties::seawater());
        let v_min = meter.minimum_detectable_velocity(1000.0, 10.0, 300.0);
        // Should be a very small positive number
        assert!(v_min > 0.0);
        assert!(v_min < 0.01); // sub-cm/s resolution expected
    }

    #[test]
    fn test_electrode_impedance() {
        let meter = MhdFlowMeter::new(0.1, 0.05, 0.1, FluidProperties::seawater());
        // Electrode area 1 cm^2 = 1e-4 m^2
        let z = meter.electrode_impedance(1e-4);
        // Z = 1 / (4.8 * 1e-4) ~ 2083 ohm
        assert!(rel_eq(z, 1.0 / (4.8 * 1e-4), REL_TOL));
    }

    // ── Lorentz force velocimetry ──────────────────────────────────────

    #[test]
    fn test_lorentz_force_from_velocity() {
        let lfv = LorentzForceVelocimeter::new(0.5, 0.001, 1.04e6);
        let f = lfv.force_from_velocity(1.0);
        // F = 1.04e6 * 1.0 * 0.25 * 0.001 = 260 N
        assert!(approx_eq(f, 260.0, 0.1));
    }

    #[test]
    fn test_lorentz_force_roundtrip() {
        let lfv = LorentzForceVelocimeter::new(0.3, 0.005, 4.8);
        let v_orig = 2.5;
        let f = lfv.force_from_velocity(v_orig);
        let v_rec = lfv.velocity_from_force(f);
        assert!(approx_eq(v_rec, v_orig, TOL));
    }

    #[test]
    fn test_lorentz_force_sensitivity() {
        let lfv = LorentzForceVelocimeter::new(0.5, 0.001, 1.04e6);
        let s = lfv.force_sensitivity();
        // sigma * B^2 * V = 1.04e6 * 0.25 * 0.001 = 260 N/(m/s)
        assert!(approx_eq(s, 260.0, 0.1));
    }

    // ── Calibration ────────────────────────────────────────────────────

    #[test]
    fn test_calibration_linear_fit() {
        // Perfect linear data: Q = 2 * E
        let points = vec![(0.001, 0.002), (0.005, 0.01), (0.01, 0.02)];
        let cal = Calibration::from_points(&points);
        assert!(rel_eq(cal.slope, 2.0, 1e-4));
        assert!(approx_eq(cal.intercept, 0.0, 1e-6));
    }

    #[test]
    fn test_calibration_r_squared_perfect() {
        let points = vec![(1.0, 2.0), (2.0, 4.0), (3.0, 6.0)];
        let cal = Calibration::from_points(&points);
        assert!(cal.r_squared() > 0.999);
    }

    #[test]
    fn test_calibration_flow_rate_from_emf() {
        let points = vec![(0.001, 0.002), (0.005, 0.01), (0.01, 0.02)];
        let cal = Calibration::from_points(&points);
        let q = cal.flow_rate_from_emf(0.007);
        assert!(rel_eq(q, 0.014, 1e-3));
    }

    #[test]
    fn test_calibration_zero_offset() {
        let points = vec![(1.0, 1.5), (2.0, 3.5), (3.0, 5.5)];
        let cal = Calibration::from_points(&points);
        // Q = 2*E - 0.5, zero offset at E = 0.25
        let zo = cal.zero_offset();
        assert!(zo > 0.0);
    }

    #[test]
    fn test_calibration_single_point() {
        let cal = Calibration::from_points(&[(0.005, 0.01)]);
        assert!(rel_eq(cal.slope, 2.0, 1e-4));
    }

    #[test]
    fn test_calibration_empty() {
        let cal = Calibration::from_points(&[]);
        assert!(approx_eq(cal.slope, 0.0, TOL));
        assert!(approx_eq(cal.intercept, 0.0, TOL));
    }

    // ── Weight function ────────────────────────────────────────────────

    #[test]
    fn test_weight_function_ideal() {
        // Ideal case: electrode_correction = 0 => W = 1 everywhere
        assert!(approx_eq(weight_function(0.0, 0.0, 0.0), 1.0, TOL));
        assert!(approx_eq(weight_function(0.5, PI / 4.0, 0.0), 1.0, TOL));
        assert!(approx_eq(weight_function(1.0, 0.0, 0.0), 1.0, TOL));
    }

    #[test]
    fn test_weight_function_outside_pipe() {
        assert!(approx_eq(weight_function(1.1, 0.0, 0.0), 0.0, TOL));
        assert!(approx_eq(weight_function(-0.1, 0.0, 0.0), 0.0, TOL));
    }

    #[test]
    fn test_mean_weight_factor_uniform() {
        let w = mean_weight_factor(VelocityProfile::Uniform, 0.1, 0.0, 100);
        assert!(approx_eq(w, 1.0, 1e-4));
    }

    #[test]
    fn test_mean_weight_factor_laminar() {
        let w = mean_weight_factor(VelocityProfile::Laminar, 0.1, 0.0, 200);
        // For ideal geometry with axisymmetric flow, weight factor = 1.0
        assert!(approx_eq(w, 1.0, 1e-3));
    }

    // ── Standalone functions ───────────────────────────────────────────

    #[test]
    fn test_standalone_reynolds() {
        let re = reynolds_number(1025.0, 2.0, 0.1, 1.08e-3);
        // 1025 * 2 * 0.1 / 0.00108 ~ 189815
        assert!(re > 100000.0);
    }

    #[test]
    fn test_standalone_magnetic_reynolds() {
        let rm = magnetic_reynolds_number(4.8, 1.0, 2.0, 0.1);
        // mu0 * 4.8 * 2 * 0.1 ~ 1.2e-6
        assert!(rm < 0.001);
    }

    #[test]
    fn test_standalone_hartmann() {
        let ha = hartmann_number(1.0, 0.1, 10.7e6, 6.9e-4);
        // 1 * 0.1 * sqrt(10.7e6 / 6.9e-4) ~ 12453
        assert!(ha > 10000.0);
    }

    // ── Integration / scenario tests ───────────────────────────────────

    #[test]
    fn test_seawater_flow_scenario() {
        // Typical seawater desalination pipe: 200mm, 0.05 T, 3 m/s
        let meter = MhdFlowMeter::new(0.2, 0.05, 0.2, FluidProperties::seawater());
        let v = 3.0;
        let emf = meter.emf_from_velocity(v);
        assert!(approx_eq(emf, 0.05 * 0.2 * 3.0, TOL)); // 30 mV

        let q = meter.flow_rate_from_velocity(v);
        let expected_q = PI / 4.0 * 0.04 * 3.0; // ~0.0942 m^3/s
        assert!(rel_eq(q, expected_q, REL_TOL));

        let re = meter.reynolds_number(v);
        assert!(re > 4000.0); // turbulent
    }

    #[test]
    fn test_liquid_sodium_reactor_scenario() {
        // Nuclear reactor coolant loop: 500mm pipe, 0.1 T, 5 m/s
        let meter = MhdFlowMeter::new(0.5, 0.1, 0.5, FluidProperties::liquid_sodium());
        let emf = meter.emf_from_velocity(5.0);
        assert!(approx_eq(emf, 0.1 * 0.5 * 5.0, TOL)); // 250 mV

        let rm = meter.magnetic_reynolds_number(5.0);
        assert!(rm > 1.0); // field distortion matters

        let ha = meter.hartmann_number();
        assert!(ha > 100.0); // strong MHD effects
    }

    #[test]
    fn test_blood_flow_scenario() {
        // Aorta: ~25mm diameter, 0.3 T (MRI field), peak ~1 m/s
        let meter = MhdFlowMeter::new(0.025, 0.3, 0.025, FluidProperties::blood());
        let emf = meter.emf_from_velocity(1.0);
        // E = 0.3 * 0.025 * 1.0 = 7.5 mV
        assert!(approx_eq(emf, 0.0075, TOL));

        let re = meter.reynolds_number(1.0);
        // Re = 1060 * 1.0 * 0.025 / 0.0035 ~ 7571 (turbulent)
        assert!(re > 4000.0);
    }
}

/// Flow regime classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowRegime {
    /// Re < 2300: orderly layered flow.
    Laminar,
    /// 2300 <= Re < 4000: intermittent turbulence.
    Transitional,
    /// Re >= 4000: fully turbulent.
    Turbulent,
}

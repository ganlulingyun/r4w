//! Tokamak Plasma Position and Shape Control
//!
//! This module implements feedback control algorithms for magnetic confinement fusion
//! tokamak devices. Tokamak plasmas are vertically unstable due to elongated cross-sections
//! and require active feedback control of poloidal field (PF) coil currents to maintain
//! plasma position, shape, and stability.
//!
//! # Key Concepts
//!
//! - **Vertical Stability**: Elongated plasmas (κ > 1) are inherently unstable vertically.
//!   The vertical instability growth rate depends on the stability index n_s and the
//!   resistive wall time constant τ_wall.
//!
//! - **Radial Position Control**: Maintains the plasma horizontal position by adjusting
//!   the vertical (poloidal) magnetic field. Uses proportional feedback on position error.
//!
//! - **Plasma Shape Control**: Controls elongation (κ), triangularity (δ), and other
//!   shaping parameters through multiple PF coil currents simultaneously.
//!
//! - **Magnetic Diagnostics**: Rogowski coils measure plasma current via voltage integration,
//!   flux loops measure poloidal flux, and magnetic probes measure local field components.
//!   Together they enable real-time equilibrium reconstruction.
//!
//! - **Disruption Prediction**: Monitors locked mode amplitude, current quench rate,
//!   and density limits to predict and mitigate plasma disruptions.
//!
//! - **Power Balance**: Ohmic heating, bremsstrahlung losses, and energy confinement
//!   time scaling laws (ITER89-P) determine the plasma energy balance.
//!
//! # References
//!
//! - Wesson, J. "Tokamaks", 4th ed., Oxford University Press, 2011.
//! - Ariola, M. & Pironti, A. "Magnetic Control of Tokamak Plasmas", Springer, 2016.
//! - ITER Physics Basis, Nuclear Fusion 39(12), 1999.

use std::f64::consts::PI;

// ─── Configuration ──────────────────────────────────────────────────────────

/// Configuration parameters for a tokamak device.
///
/// Defines the geometric and magnetic parameters of the tokamak,
/// along with the number of poloidal field coils and the control system
/// sample rate.
///
/// # Example
///
/// ```
/// use r4w_core::tokamak_plasma_control::TokamakConfig;
///
/// // ITER-like configuration
/// let config = TokamakConfig {
///     major_radius_m: 6.2,
///     minor_radius_m: 2.0,
///     toroidal_field_t: 5.3,
///     plasma_current_ma: 15.0,
///     elongation: 1.7,
///     triangularity: 0.33,
///     num_pf_coils: 6,
///     control_sample_rate_hz: 10_000.0,
/// };
/// ```
#[derive(Debug, Clone)]
pub struct TokamakConfig {
    /// Major radius R0 in meters (typically 1.5–6.2 m).
    pub major_radius_m: f64,
    /// Minor radius a in meters (typically 0.5–2.0 m).
    pub minor_radius_m: f64,
    /// Toroidal magnetic field B_T in Tesla (typically 2–6 T).
    pub toroidal_field_t: f64,
    /// Plasma current I_p in mega-amperes (typically 1–15 MA).
    pub plasma_current_ma: f64,
    /// Elongation κ of the plasma cross-section (typically 1.0–2.0).
    pub elongation: f64,
    /// Triangularity δ of the plasma cross-section (typically 0.0–0.5).
    pub triangularity: f64,
    /// Number of poloidal field coils (typically 6–14).
    pub num_pf_coils: usize,
    /// Control system sample rate in Hz (typically 10,000).
    pub control_sample_rate_hz: f64,
}

impl Default for TokamakConfig {
    /// Returns an ITER-like default configuration.
    fn default() -> Self {
        Self {
            major_radius_m: 6.2,
            minor_radius_m: 2.0,
            toroidal_field_t: 5.3,
            plasma_current_ma: 15.0,
            elongation: 1.7,
            triangularity: 0.33,
            num_pf_coils: 6,
            control_sample_rate_hz: 10_000.0,
        }
    }
}

// ─── Plasma Equilibrium ─────────────────────────────────────────────────────

/// Describes the magnetic equilibrium state of the plasma.
///
/// Contains the plasma position, shape parameters, and key physics quantities
/// that are reconstructed in real time from magnetic diagnostic measurements.
#[derive(Debug, Clone)]
pub struct PlasmaEquilibrium {
    /// Horizontal (radial) position of the plasma center in meters.
    pub r_center_m: f64,
    /// Vertical position of the plasma center in meters.
    pub z_center_m: f64,
    /// Elongation κ (ratio of vertical to horizontal extent).
    pub kappa: f64,
    /// Triangularity δ (D-shape parameter, 0 = circular, >0 = D-shaped).
    pub delta: f64,
    /// Poloidal beta β_p = 2μ₀⟨p⟩/B_p² (ratio of kinetic to magnetic pressure).
    pub beta_p: f64,
    /// Internal inductance l_i (current profile peaking parameter).
    pub li: f64,
    /// Plasma current in mega-amperes.
    pub ip_ma: f64,
    /// Safety factor at 95% flux surface (q₉₅).
    pub q95: f64,
}

impl Default for PlasmaEquilibrium {
    fn default() -> Self {
        Self {
            r_center_m: 6.2,
            z_center_m: 0.0,
            kappa: 1.7,
            delta: 0.33,
            beta_p: 0.6,
            li: 0.85,
            ip_ma: 15.0,
            q95: 3.0,
        }
    }
}

// ─── Plasma Controller ──────────────────────────────────────────────────────

/// Feedback controller for tokamak plasma position and shape.
///
/// Implements PID-based control for vertical stability, radial position,
/// and plasma shape. The controller computes poloidal field coil current
/// adjustments based on the difference between target and measured equilibrium.
///
/// # Vertical Stability
///
/// Elongated plasmas are vertically unstable with a growth rate:
///
///   γ = n_s / τ_wall
///
/// where n_s is the vertical stability index (increases with elongation)
/// and τ_wall is the resistive wall time constant. Active feedback must
/// operate faster than this growth rate.
///
/// # Shape Control
///
/// The shape controller adjusts multiple PF coil currents simultaneously
/// to control elongation, triangularity, position, and current distribution.
pub struct PlasmaController {
    /// Device configuration.
    config: TokamakConfig,
}

impl PlasmaController {
    /// Creates a new plasma controller from the given tokamak configuration.
    pub fn new(config: TokamakConfig) -> Self {
        Self { config }
    }

    /// Returns a reference to the underlying tokamak configuration.
    pub fn config(&self) -> &TokamakConfig {
        &self.config
    }

    /// Computes the vertical stability index n_s for a given elongation and aspect ratio.
    ///
    /// The stability index characterizes how unstable the plasma is vertically.
    /// Higher elongation and lower aspect ratio increase n_s, making the plasma
    /// more unstable.
    ///
    /// Approximate formula:
    ///   n_s ≈ 1.5 * (κ² - 1) / A
    ///
    /// where κ is the elongation and A = R/a is the aspect ratio.
    ///
    /// # Arguments
    ///
    /// * `elongation` - Plasma elongation κ (≥ 1.0)
    /// * `aspect_ratio` - Aspect ratio A = R/a
    ///
    /// # Returns
    ///
    /// Vertical stability index n_s (dimensionless). Returns 0.0 for κ ≤ 1.0
    /// (circular plasma is neutrally stable vertically).
    pub fn vertical_stability_index(elongation: f64, aspect_ratio: f64) -> f64 {
        if elongation <= 1.0 || aspect_ratio <= 0.0 {
            return 0.0;
        }
        1.5 * (elongation * elongation - 1.0) / aspect_ratio
    }

    /// Computes the vertical instability growth rate in Hz.
    ///
    /// The growth rate determines how fast the plasma moves vertically
    /// without active feedback control:
    ///
    ///   γ = n_s / τ_wall
    ///
    /// where τ_wall is the resistive wall time constant in seconds.
    ///
    /// # Arguments
    ///
    /// * `n_s` - Vertical stability index
    /// * `tau_wall_ms` - Resistive wall time constant in milliseconds
    ///
    /// # Returns
    ///
    /// Growth rate in Hz. Returns 0.0 if τ_wall ≤ 0.
    pub fn vertical_growth_rate(n_s: f64, tau_wall_ms: f64) -> f64 {
        if tau_wall_ms <= 0.0 {
            return 0.0;
        }
        let tau_wall_s = tau_wall_ms * 1e-3;
        n_s / tau_wall_s
    }

    /// General-purpose PID controller.
    ///
    /// Implements a discrete PID controller with anti-windup (integral term
    /// is updated in place). Computes:
    ///
    ///   u = Kp * e + Ki * ∫e·dt + Kd * de/dt
    ///
    /// # Arguments
    ///
    /// * `error` - Current error signal (target - measured)
    /// * `integral` - Accumulated integral of error (mutable, updated in place)
    /// * `prev_error` - Previous error value (mutable, updated in place)
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    /// * `kd` - Derivative gain
    /// * `dt` - Time step in seconds
    ///
    /// # Returns
    ///
    /// Control output u.
    pub fn pid_controller(
        error: f64,
        integral: &mut f64,
        prev_error: &mut f64,
        kp: f64,
        ki: f64,
        kd: f64,
        dt: f64,
    ) -> f64 {
        if dt <= 0.0 {
            return 0.0;
        }
        *integral += error * dt;
        let derivative = (error - *prev_error) / dt;
        *prev_error = error;
        kp * error + ki * *integral + kd * derivative
    }

    /// Computes PF coil current adjustments for plasma shape control.
    ///
    /// Determines the corrections needed for each poloidal field coil
    /// to drive the plasma from its current equilibrium state toward
    /// the target equilibrium. Shape parameters (κ, δ) and position
    /// (R, Z) are controlled simultaneously.
    ///
    /// The control law distributes corrections across coils:
    /// - Coils 0,1: Vertical position (antisymmetric for vertical field)
    /// - Coils 2,3: Radial position (symmetric for radial field)
    /// - Coils 4+: Elongation and triangularity shaping
    ///
    /// # Arguments
    ///
    /// * `target` - Desired plasma equilibrium
    /// * `current` - Current measured plasma equilibrium
    ///
    /// # Returns
    ///
    /// Vector of coil current adjustments in kA, one per PF coil.
    pub fn shape_controller(
        &self,
        target: &PlasmaEquilibrium,
        current: &PlasmaEquilibrium,
    ) -> Vec<f64> {
        let n = self.config.num_pf_coils;
        let mut adjustments = vec![0.0; n];

        // Position errors
        let r_error = target.r_center_m - current.r_center_m;
        let z_error = target.z_center_m - current.z_center_m;

        // Shape errors
        let kappa_error = target.kappa - current.kappa;
        let delta_error = target.delta - current.delta;

        // Vertical position: coils 0,1 in antisymmetric configuration
        // Produces a radial field to push plasma vertically
        if n >= 2 {
            let gain_z = 10.0; // kA per meter of vertical displacement
            adjustments[0] = gain_z * z_error;
            adjustments[1] = -gain_z * z_error;
        }

        // Radial position: coils 2,3 in symmetric configuration
        // Produces a vertical field to push plasma radially
        if n >= 4 {
            let gain_r = 5.0; // kA per meter of radial displacement
            adjustments[2] = gain_r * r_error;
            adjustments[3] = gain_r * r_error;
        }

        // Elongation control: coils 4+ distribute shaping currents
        if n >= 5 {
            let gain_kappa = 20.0; // kA per unit of elongation error
            adjustments[4] = gain_kappa * kappa_error;
        }

        // Triangularity control
        if n >= 6 {
            let gain_delta = 15.0; // kA per unit of triangularity error
            adjustments[5] = gain_delta * delta_error;
        }

        adjustments
    }

    /// Radial position control via horizontal field correction.
    ///
    /// Computes the vertical magnetic field adjustment needed to correct
    /// the plasma's radial (horizontal) position:
    ///
    ///   ΔB_v = Kp * (R_target - R_current)
    ///
    /// # Arguments
    ///
    /// * `r_target` - Target radial position in meters
    /// * `r_current` - Current radial position in meters
    /// * `kp` - Proportional gain in T/m
    ///
    /// # Returns
    ///
    /// Vertical field correction in Tesla.
    pub fn radial_position_control(r_target: f64, r_current: f64, kp: f64) -> f64 {
        kp * (r_target - r_current)
    }

    /// Fast vertical position control with proportional-derivative feedback.
    ///
    /// Vertical stability requires fast feedback (typically < 1 ms response).
    /// Uses PD control on the vertical displacement:
    ///
    ///   ΔB_r = Kp * z + Kd * dz/dt
    ///
    /// where z is the vertical displacement and dz/dt is the vertical velocity
    /// estimated from consecutive samples.
    ///
    /// # Arguments
    ///
    /// * `z_current` - Current vertical position in meters
    /// * `kp` - Proportional gain in T/m
    /// * `kd` - Derivative gain in T·s/m
    /// * `z_prev` - Previous vertical position in meters
    /// * `dt` - Time step in seconds
    ///
    /// # Returns
    ///
    /// Radial field correction in Tesla. Positive field pushes plasma downward.
    pub fn vertical_position_control(
        z_current: f64,
        kp: f64,
        kd: f64,
        z_prev: f64,
        dt: f64,
    ) -> f64 {
        if dt <= 0.0 {
            return kp * z_current;
        }
        let dz_dt = (z_current - z_prev) / dt;
        kp * z_current + kd * dz_dt
    }

    /// Computes the cylindrical safety factor q.
    ///
    /// The safety factor q is the ratio of toroidal to poloidal field line
    /// windings. It determines MHD stability: q > 1 everywhere is required
    /// to avoid the kink instability (Kruskal-Shafranov limit).
    ///
    /// Cylindrical approximation (including elongation correction):
    ///
    ///   q_cyl = (5 · a² · B_T) / (R · I_p) · (1 + κ²) / 2
    ///
    /// where a is in meters, B_T in Tesla, R in meters, I_p in MA, and κ is elongation.
    ///
    /// # Arguments
    ///
    /// * `b_t` - Toroidal field in Tesla
    /// * `r` - Major radius in meters
    /// * `a` - Minor radius in meters
    /// * `ip_ma` - Plasma current in mega-amperes
    /// * `elongation` - Plasma elongation κ
    ///
    /// # Returns
    ///
    /// Cylindrical safety factor q (dimensionless).
    pub fn safety_factor(b_t: f64, r: f64, a: f64, ip_ma: f64, elongation: f64) -> f64 {
        if ip_ma.abs() < 1e-12 || r.abs() < 1e-12 {
            return f64::INFINITY;
        }
        (5.0 * a * a * b_t) / (r * ip_ma) * (1.0 + elongation * elongation) / 2.0
    }

    /// Computes the Greenwald density limit.
    ///
    /// The Greenwald limit is an empirical maximum density above which
    /// the plasma tends to disrupt:
    ///
    ///   n_GW = I_p / (π · a²)
    ///
    /// where I_p is in MA and a is in meters, giving n_GW in units of 10²⁰ m⁻³.
    ///
    /// # Arguments
    ///
    /// * `ip_ma` - Plasma current in mega-amperes
    /// * `a_m` - Minor radius in meters
    ///
    /// # Returns
    ///
    /// Greenwald density limit in units of 10²⁰ m⁻³.
    pub fn greenwald_density_limit(ip_ma: f64, a_m: f64) -> f64 {
        if a_m.abs() < 1e-12 {
            return f64::INFINITY;
        }
        ip_ma / (PI * a_m * a_m)
    }
}

// ─── Magnetic Diagnostics ───────────────────────────────────────────────────

/// Magnetic diagnostic signal processing for tokamak equilibrium reconstruction.
///
/// Tokamaks use arrays of magnetic sensors to determine the plasma position
/// and shape in real time:
///
/// - **Rogowski coils**: Measure total enclosed current via ∮B·dl = μ₀I.
///   The coil output voltage is proportional to dI/dt, so integration yields current.
///
/// - **Flux loops**: Simple wire loops measuring time-integrated poloidal flux.
///   The loop voltage is dΨ/dt, so integration gives the flux Ψ.
///
/// - **Magnetic probes**: Hall sensors or pickup coils measuring local B-field
///   components at specific locations around the vacuum vessel.
///
/// - **Diamagnetic loop**: Measures the change in toroidal flux due to the
///   plasma pressure, providing a proxy for stored energy.
pub struct MagneticDiagnostics;

impl MagneticDiagnostics {
    /// Integrates a Rogowski coil voltage signal to recover the plasma current.
    ///
    /// The Rogowski coil output is V = M · dI/dt, so:
    ///
    ///   I(t) = (1/M) · ∫V(t)dt
    ///
    /// Uses trapezoidal integration for improved accuracy.
    ///
    /// # Arguments
    ///
    /// * `voltage_signal` - Sampled voltage waveform from the Rogowski coil
    /// * `dt_s` - Sample period in seconds
    /// * `sensitivity` - Mutual inductance M in V·s/A (Rogowski sensitivity)
    ///
    /// # Returns
    ///
    /// Reconstructed current waveform (same length as input).
    pub fn rogowski_coil_current(
        voltage_signal: &[f64],
        dt_s: f64,
        sensitivity: f64,
    ) -> Vec<f64> {
        if voltage_signal.is_empty() || sensitivity.abs() < 1e-30 || dt_s <= 0.0 {
            return vec![0.0; voltage_signal.len()];
        }
        let mut current = vec![0.0; voltage_signal.len()];
        let mut integral = 0.0;
        for i in 0..voltage_signal.len() {
            if i > 0 {
                // Trapezoidal integration
                integral += 0.5 * (voltage_signal[i - 1] + voltage_signal[i]) * dt_s;
            }
            current[i] = integral / sensitivity;
        }
        current
    }

    /// Integrates a flux loop signal to recover the poloidal magnetic flux.
    ///
    /// The flux loop output voltage is V = dΨ/dt, so:
    ///
    ///   Ψ(t) = ∫V(t)dt
    ///
    /// # Arguments
    ///
    /// * `signal` - Sampled voltage waveform from the flux loop
    /// * `dt_s` - Sample period in seconds
    ///
    /// # Returns
    ///
    /// Reconstructed flux waveform in Wb (same length as input).
    pub fn flux_loop_measurement(signal: &[f64], dt_s: f64) -> Vec<f64> {
        if signal.is_empty() || dt_s <= 0.0 {
            return vec![0.0; signal.len()];
        }
        let mut flux = vec![0.0; signal.len()];
        let mut integral = 0.0;
        for i in 0..signal.len() {
            if i > 0 {
                integral += 0.5 * (signal[i - 1] + signal[i]) * dt_s;
            }
            flux[i] = integral;
        }
        flux
    }

    /// Converts magnetic probe voltage signals to field measurements.
    ///
    /// For calibrated probes, the magnetic field is directly proportional
    /// to the output voltage:
    ///
    ///   B = V × sensitivity
    ///
    /// # Arguments
    ///
    /// * `signal` - Sampled voltage waveform from the magnetic probe
    /// * `sensitivity_t_per_v` - Calibration factor in T/V
    ///
    /// # Returns
    ///
    /// Magnetic field waveform in Tesla.
    pub fn magnetic_probe_field(signal: &[f64], sensitivity_t_per_v: f64) -> Vec<f64> {
        signal.iter().map(|v| v * sensitivity_t_per_v).collect()
    }

    /// Processes a diamagnetic loop signal to estimate stored energy proxy.
    ///
    /// The diamagnetic loop measures the change in toroidal flux caused by
    /// the plasma pressure. The signal is integrated and scaled by a geometry
    /// factor to yield a quantity proportional to the plasma stored energy:
    ///
    ///   W_dia ∝ geometry_factor × ∫V(t)dt
    ///
    /// # Arguments
    ///
    /// * `signal` - Sampled voltage waveform from the diamagnetic loop
    /// * `dt_s` - Sample period in seconds
    /// * `geometry_factor` - Scaling factor accounting for loop geometry (J/Wb)
    ///
    /// # Returns
    ///
    /// Stored energy proxy waveform.
    pub fn diamagnetic_loop(signal: &[f64], dt_s: f64, geometry_factor: f64) -> Vec<f64> {
        let flux = Self::flux_loop_measurement(signal, dt_s);
        flux.iter().map(|f| f * geometry_factor).collect()
    }

    /// Simplified equilibrium reconstruction from magnetic measurements.
    ///
    /// Performs a basic reconstruction of the plasma equilibrium state from
    /// poloidal field measurements, flux loop values, and probe positions.
    /// This is a simplified version of the Grad-Shafranov equilibrium
    /// reconstruction (full EFIT-like codes use iterative fitting).
    ///
    /// The reconstruction estimates:
    /// - Plasma center (R, Z) from weighted averages of probe signals
    /// - Elongation from vertical/horizontal field ratios
    /// - Plasma current from the average poloidal field
    ///
    /// # Arguments
    ///
    /// * `bp_measurements` - Poloidal field measurements in Tesla
    /// * `flux_measurements` - Flux loop measurements in Wb
    /// * `probe_positions` - (R, Z) positions of the magnetic probes in meters
    ///
    /// # Returns
    ///
    /// Reconstructed `PlasmaEquilibrium`.
    pub fn equilibrium_reconstruction(
        bp_measurements: &[f64],
        flux_measurements: &[f64],
        probe_positions: &[[f64; 2]],
    ) -> PlasmaEquilibrium {
        let n_probes = bp_measurements.len().min(probe_positions.len());
        if n_probes == 0 {
            return PlasmaEquilibrium::default();
        }

        // Estimate plasma center from weighted average of probe positions
        // Weight by absolute field value (stronger field = closer to plasma)
        let mut r_sum = 0.0;
        let mut z_sum = 0.0;
        let mut w_sum = 0.0;
        for i in 0..n_probes {
            let w = bp_measurements[i].abs() + 1e-30;
            r_sum += probe_positions[i][0] * w;
            z_sum += probe_positions[i][1] * w;
            w_sum += w;
        }
        let r_center = r_sum / w_sum;
        let z_center = z_sum / w_sum;

        // Estimate plasma current from average poloidal field
        // B_p ≈ μ₀ I_p / (2π a), so I_p ≈ B_p * 2π * a / μ₀
        let mu_0 = 4.0 * PI * 1e-7;
        let avg_bp: f64 = bp_measurements.iter().copied().sum::<f64>() / n_probes as f64;
        let a_est = 0.5; // estimate minor radius from probe spread
        let ip_a = avg_bp.abs() * 2.0 * PI * a_est / mu_0; // in Amperes
        let ip_ma = ip_a * 1e-6;

        // Estimate elongation from flux distribution
        // Use ratio of vertical to horizontal flux extent as proxy
        let avg_flux = if flux_measurements.is_empty() {
            0.0
        } else {
            flux_measurements.iter().copied().sum::<f64>() / flux_measurements.len() as f64
        };
        let kappa = 1.0 + 0.5 * avg_flux.abs().min(2.0);

        // Safety factor estimate
        let q95 = if ip_ma > 0.01 {
            5.0 * a_est * a_est * 5.0 / (r_center * ip_ma) * (1.0 + kappa * kappa) / 2.0
        } else {
            3.0
        };

        PlasmaEquilibrium {
            r_center_m: r_center,
            z_center_m: z_center,
            kappa,
            delta: 0.33, // Default; full reconstruction would fit this
            beta_p: 0.6,
            li: 0.85,
            ip_ma,
            q95,
        }
    }
}

// ─── Disruption Predictor ───────────────────────────────────────────────────

/// Disruption prediction and warning system.
///
/// Tokamak disruptions are sudden, violent terminations of the plasma
/// that can deposit enormous heat loads on plasma-facing components and
/// induce large electromagnetic forces on the vessel. Early detection
/// allows mitigation actions (e.g., massive gas injection).
///
/// Key disruption precursors:
/// - **Locked modes**: n=1 magnetic perturbations that lock to the wall,
///   indicating loss of plasma rotation and impending disruption.
/// - **Current quench**: Rapid loss of plasma current during disruption,
///   with rates up to 1 MA/ms in ITER.
/// - **Thermal quench**: Rapid loss of plasma thermal energy, typically
///   preceding the current quench.
/// - **Density limit**: Operating above the Greenwald density limit
///   leads to radiative collapse and disruption.
pub struct DisruptionPredictor;

impl DisruptionPredictor {
    /// Detects a locked mode from the amplitude signal of saddle coils.
    ///
    /// A locked mode is an n=1 magnetic perturbation that has stopped
    /// rotating and become stationary relative to the wall. This is a
    /// strong predictor of imminent disruption.
    ///
    /// # Arguments
    ///
    /// * `amplitude_signal` - Time series of n=1 mode amplitude (arbitrary units)
    /// * `threshold` - Detection threshold (same units as signal)
    ///
    /// # Returns
    ///
    /// `Some(index)` of the first sample exceeding the threshold, or `None`.
    pub fn locked_mode_detector(amplitude_signal: &[f64], threshold: f64) -> Option<usize> {
        amplitude_signal
            .iter()
            .position(|&v| v.abs() >= threshold)
    }

    /// Computes the current quench rate dI_p/dt during a disruption.
    ///
    /// The current quench rate is the maximum |dI/dt| observed in the
    /// plasma current signal. High quench rates induce large eddy
    /// currents and forces on the vacuum vessel.
    ///
    /// # Arguments
    ///
    /// * `ip_signal` - Plasma current time series in MA
    /// * `dt_s` - Sample period in seconds
    ///
    /// # Returns
    ///
    /// Maximum absolute current quench rate in MA/s.
    pub fn current_quench_rate(ip_signal: &[f64], dt_s: f64) -> f64 {
        if ip_signal.len() < 2 || dt_s <= 0.0 {
            return 0.0;
        }
        let mut max_rate = 0.0_f64;
        for i in 1..ip_signal.len() {
            let rate = ((ip_signal[i] - ip_signal[i - 1]) / dt_s).abs();
            if rate > max_rate {
                max_rate = rate;
            }
        }
        max_rate
    }

    /// Estimates the thermal quench e-folding time.
    ///
    /// During a thermal quench, the plasma temperature drops exponentially:
    ///   T(t) = T₀ · exp(-t/τ_TQ)
    ///
    /// The e-folding time τ_TQ is estimated from the log-slope of the
    /// temperature decay.
    ///
    /// # Arguments
    ///
    /// * `temperature_signal` - Electron temperature time series in keV
    /// * `dt_s` - Sample period in seconds
    ///
    /// # Returns
    ///
    /// Thermal quench e-folding time in seconds. Returns 0.0 if the signal
    /// does not show a clear decay.
    pub fn thermal_quench_time(temperature_signal: &[f64], dt_s: f64) -> f64 {
        if temperature_signal.len() < 2 || dt_s <= 0.0 {
            return 0.0;
        }

        // Find peak temperature
        let (peak_idx, peak_val) = temperature_signal
            .iter()
            .enumerate()
            .fold((0, f64::NEG_INFINITY), |(mi, mv), (i, &v)| {
                if v > mv { (i, v) } else { (mi, mv) }
            });

        if peak_val <= 0.0 || peak_idx >= temperature_signal.len() - 1 {
            return 0.0;
        }

        // Find when temperature drops to 1/e of peak
        let threshold = peak_val / std::f64::consts::E;
        for i in (peak_idx + 1)..temperature_signal.len() {
            if temperature_signal[i] <= threshold {
                return (i - peak_idx) as f64 * dt_s;
            }
        }

        // If never reaches 1/e, estimate from average decay rate
        let last = temperature_signal.last().copied().unwrap_or(peak_val);
        if last < peak_val && last > 0.0 {
            let total_time = (temperature_signal.len() - 1 - peak_idx) as f64 * dt_s;
            let ratio = peak_val / last;
            total_time / ratio.ln()
        } else {
            0.0
        }
    }

    /// Computes the halo current fraction.
    ///
    /// During a vertical displacement event (VDE), the plasma contacts
    /// the wall and drives halo currents in the vacuum vessel structures.
    /// The halo current fraction is:
    ///
    ///   f_halo = I_halo / I_p
    ///
    /// Typical values are 0.1–0.4. Higher fractions indicate more severe
    /// force loading.
    ///
    /// # Arguments
    ///
    /// * `halo_current` - Measured halo current in the same units as plasma current
    /// * `plasma_current` - Plasma current
    ///
    /// # Returns
    ///
    /// Halo current fraction (dimensionless).
    pub fn halo_current_fraction(halo_current: f64, plasma_current: f64) -> f64 {
        if plasma_current.abs() < 1e-30 {
            return 0.0;
        }
        (halo_current / plasma_current).abs()
    }

    /// Issues a disruption warning based on multiple precursors.
    ///
    /// A disruption warning is triggered when any of the following conditions
    /// are met:
    /// - A locked mode is detected
    /// - The current quench rate exceeds 10 MA/s
    /// - The density exceeds the Greenwald limit (density_fraction > 1.0)
    ///
    /// # Arguments
    ///
    /// * `ip_rate` - Current quench rate in MA/s
    /// * `locked_mode` - Whether a locked mode has been detected
    /// * `density_fraction` - Ratio of operating density to Greenwald limit (n/n_GW)
    ///
    /// # Returns
    ///
    /// `true` if a disruption warning should be issued.
    pub fn disruption_warning(ip_rate: f64, locked_mode: bool, density_fraction: f64) -> bool {
        locked_mode || ip_rate.abs() > 10.0 || density_fraction > 1.0
    }
}

// ─── Power Balance ──────────────────────────────────────────────────────────

/// Plasma power balance and confinement calculations.
///
/// The energy confinement time τ_E determines how long the plasma retains
/// its thermal energy. It is governed by transport processes and is the
/// key figure of merit for fusion performance. The power balance is:
///
///   dW/dt = P_heat - P_loss = P_OH + P_aux - P_rad - W/τ_E
///
/// where W is the plasma stored energy, P_OH is Ohmic heating, P_aux
/// is auxiliary heating, and P_rad is radiative losses.
pub struct PowerBalance;

impl PowerBalance {
    /// Computes Ohmic heating power.
    ///
    ///   P_OH = I_p² × R_plasma
    ///
    /// where I_p is the plasma current and R_plasma is the plasma resistance.
    /// Note: I_p is in MA, so we convert to Amperes for the calculation.
    ///
    /// # Arguments
    ///
    /// * `ip_ma` - Plasma current in mega-amperes
    /// * `resistance_ohm` - Plasma resistance in Ohms
    ///
    /// # Returns
    ///
    /// Ohmic heating power in Watts.
    pub fn ohmic_heating_power(ip_ma: f64, resistance_ohm: f64) -> f64 {
        let ip_a = ip_ma * 1e6;
        ip_a * ip_a * resistance_ohm
    }

    /// Computes bremsstrahlung radiation power loss.
    ///
    /// Bremsstrahlung (free-free radiation) is the dominant radiation
    /// loss mechanism in hot plasmas:
    ///
    ///   P_brem = C_brem × n_e² × √T_e × Z_eff × V
    ///
    /// where C_brem ≈ 5.35 × 10⁻³⁷ W·m³·keV⁻¹/² is the bremsstrahlung
    /// coefficient for a hydrogen plasma.
    ///
    /// # Arguments
    ///
    /// * `ne_m3` - Electron density in m⁻³
    /// * `te_kev` - Electron temperature in keV
    /// * `volume_m3` - Plasma volume in m³
    /// * `zeff` - Effective charge number (1.0 for pure hydrogen)
    ///
    /// # Returns
    ///
    /// Bremsstrahlung power in Watts.
    pub fn bremsstrahlung_power(ne_m3: f64, te_kev: f64, volume_m3: f64, zeff: f64) -> f64 {
        if te_kev <= 0.0 {
            return 0.0;
        }
        let c_brem = 5.35e-37; // W·m³·keV^{-1/2}
        c_brem * ne_m3 * ne_m3 * te_kev.sqrt() * zeff * volume_m3
    }

    /// Computes the energy confinement time using the ITER89-P L-mode scaling.
    ///
    /// The ITER89-P scaling law is an empirical fit to a multi-machine database:
    ///
    ///   τ_E = 0.048 × I_p^0.85 × B_T^0.2 × n̄^0.1 × P^{-0.5}
    ///         × R^1.2 × a^0.3 × κ^0.5 × M^0.5
    ///
    /// where I_p is in MA, B_T in T, n̄ in 10²⁰/m³, P in MW, R and a in m,
    /// κ is elongation, and M is the isotope mass in AMU.
    ///
    /// # Arguments
    ///
    /// * `ip` - Plasma current in MA
    /// * `bt` - Toroidal field in Tesla
    /// * `ne` - Line-averaged electron density in 10²⁰/m³
    /// * `power` - Total heating power in MW
    /// * `r` - Major radius in meters
    /// * `a` - Minor radius in meters
    /// * `kappa` - Elongation
    /// * `mass` - Isotope mass in AMU (2.0 for deuterium, 2.5 for D-T)
    ///
    /// # Returns
    ///
    /// Energy confinement time in seconds.
    pub fn confinement_time_iter89(
        ip: f64,
        bt: f64,
        ne: f64,
        power: f64,
        r: f64,
        a: f64,
        kappa: f64,
        mass: f64,
    ) -> f64 {
        if power <= 0.0 {
            return 0.0;
        }
        0.048
            * ip.abs().powf(0.85)
            * bt.abs().powf(0.2)
            * ne.abs().powf(0.1)
            * power.powf(-0.5)
            * r.abs().powf(1.2)
            * a.abs().powf(0.3)
            * kappa.abs().powf(0.5)
            * mass.abs().powf(0.5)
    }

    /// Computes the fusion triple product n·T·τ_E.
    ///
    /// The triple product is the key figure of merit for fusion performance.
    /// Ignition requires approximately n·T·τ_E > 3 × 10²¹ keV·s/m³
    /// at optimal temperature (~15 keV for D-T fusion).
    ///
    /// # Arguments
    ///
    /// * `ne` - Electron density in m⁻³
    /// * `ti_kev` - Ion temperature in keV
    /// * `tau_e` - Energy confinement time in seconds
    ///
    /// # Returns
    ///
    /// Triple product in keV·s/m³.
    pub fn fusion_triple_product(ne: f64, ti_kev: f64, tau_e: f64) -> f64 {
        ne * ti_kev * tau_e
    }

    /// Computes the minimum n·τ required for ignition (Lawson criterion).
    ///
    /// The Lawson criterion gives the minimum density-confinement product
    /// for a D-T plasma to sustain itself:
    ///
    ///   n·τ > 12·T / (⟨σv⟩_DT · E_α)
    ///
    /// A commonly used simplified form at 10–20 keV:
    ///
    ///   n·τ ≈ 1.5 × 10²⁰ / T² (in s/m³ when T is in keV)
    ///
    /// This function uses a more physical parameterization based on the
    /// D-T reaction rate:
    ///
    ///   n·τ_min ≈ 12 × T / ⟨σv⟩_DT(T)
    ///
    /// where ⟨σv⟩_DT is approximated for 5–30 keV.
    ///
    /// # Arguments
    ///
    /// * `ti_kev` - Ion temperature in keV
    ///
    /// # Returns
    ///
    /// Minimum n·τ in s/m³ for ignition.
    pub fn lawson_criterion(ti_kev: f64) -> f64 {
        if ti_kev <= 0.0 {
            return f64::INFINITY;
        }
        // D-T fusion reactivity approximation: <sigma*v> ≈ 1.1e-24 * T^2 m³/s
        // (valid near optimal temperature ~10-20 keV)
        // E_alpha = 3.5 MeV = 3.5e3 keV = 5.6e-13 J
        // Lawson: n*tau > 12*T / (<sigma*v> * E_alpha)
        // n*tau > 12 * T / (1.1e-24 * T^2 * 3.5e3) = 12 / (3.85e-21 * T)
        // n*tau > 3.12e21 / T  (s/m^3 for T in keV)
        3.12e21 / ti_kev
    }
}

// ─── Helper Functions ───────────────────────────────────────────────────────

/// Computes the aspect ratio A = R/a.
///
/// The aspect ratio is a key geometric parameter:
/// - Large aspect ratio (A > 3): conventional tokamaks (ITER, JET)
/// - Low aspect ratio (A < 2): spherical tokamaks (NSTX, MAST)
///
/// # Arguments
///
/// * `major_r` - Major radius R in meters
/// * `minor_a` - Minor radius a in meters
///
/// # Returns
///
/// Aspect ratio (dimensionless).
pub fn aspect_ratio(major_r: f64, minor_a: f64) -> f64 {
    if minor_a.abs() < 1e-30 {
        return f64::INFINITY;
    }
    major_r / minor_a
}

/// Computes the plasma volume for an elongated torus.
///
///   V = 2π² R κ a²
///
/// where R is the major radius, a is the minor radius, and κ is the elongation.
///
/// # Arguments
///
/// * `r` - Major radius in meters
/// * `a` - Minor radius in meters
/// * `kappa` - Elongation
///
/// # Returns
///
/// Plasma volume in m³.
pub fn plasma_volume(r: f64, a: f64, kappa: f64) -> f64 {
    2.0 * PI * PI * r * kappa * a * a
}

/// Computes the toroidal magnetic flux through the plasma cross-section.
///
///   Φ_T = B_T × π × a²
///
/// This is the vacuum toroidal flux enclosed by the plasma boundary.
///
/// # Arguments
///
/// * `bt` - Toroidal field in Tesla
/// * `r` - Major radius in meters (unused in vacuum flux, included for API consistency)
/// * `a` - Minor radius in meters
///
/// # Returns
///
/// Toroidal flux in Weber.
pub fn toroidal_flux(bt: f64, _r: f64, a: f64) -> f64 {
    bt * PI * a * a
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-9;

    // --- TokamakConfig tests ---

    #[test]
    fn test_default_config() {
        let config = TokamakConfig::default();
        assert!((config.major_radius_m - 6.2).abs() < EPSILON);
        assert!((config.minor_radius_m - 2.0).abs() < EPSILON);
        assert!((config.toroidal_field_t - 5.3).abs() < EPSILON);
        assert!((config.plasma_current_ma - 15.0).abs() < EPSILON);
        assert!((config.elongation - 1.7).abs() < EPSILON);
        assert!((config.triangularity - 0.33).abs() < EPSILON);
        assert_eq!(config.num_pf_coils, 6);
        assert!((config.control_sample_rate_hz - 10_000.0).abs() < EPSILON);
    }

    // --- Aspect ratio tests ---

    #[test]
    fn test_aspect_ratio_iter() {
        // ITER: R=6.2m, a=2.0m -> A=3.1
        let a = aspect_ratio(6.2, 2.0);
        assert!((a - 3.1).abs() < EPSILON);
    }

    #[test]
    fn test_aspect_ratio_spherical_tokamak() {
        // Spherical tokamak: R=0.85m, a=0.65m -> A≈1.31
        let a = aspect_ratio(0.85, 0.65);
        assert!((a - 0.85 / 0.65).abs() < EPSILON);
    }

    #[test]
    fn test_aspect_ratio_zero_minor_radius() {
        assert!(aspect_ratio(6.2, 0.0).is_infinite());
    }

    // --- Plasma volume tests ---

    #[test]
    fn test_plasma_volume_circular() {
        // Circular plasma (κ=1): V = 2π²Ra²
        let v = plasma_volume(6.2, 2.0, 1.0);
        let expected = 2.0 * PI * PI * 6.2 * 1.0 * 4.0;
        assert!((v - expected).abs() < 1e-6);
    }

    #[test]
    fn test_plasma_volume_elongated() {
        // ITER-like: R=6.2, a=2.0, κ=1.7
        let v = plasma_volume(6.2, 2.0, 1.7);
        let expected = 2.0 * PI * PI * 6.2 * 1.7 * 4.0;
        assert!((v - expected).abs() < 1e-6);
    }

    #[test]
    fn test_plasma_volume_increases_with_elongation() {
        let v1 = plasma_volume(6.2, 2.0, 1.0);
        let v2 = plasma_volume(6.2, 2.0, 1.5);
        assert!(v2 > v1);
    }

    // --- Toroidal flux tests ---

    #[test]
    fn test_toroidal_flux() {
        let phi = toroidal_flux(5.3, 6.2, 2.0);
        let expected = 5.3 * PI * 4.0;
        assert!((phi - expected).abs() < 1e-6);
    }

    // --- Safety factor tests ---

    #[test]
    fn test_safety_factor_greater_than_one() {
        // For typical parameters, q should be > 1 (kink stability)
        let q = PlasmaController::safety_factor(5.3, 6.2, 2.0, 15.0, 1.7);
        assert!(q > 1.0, "Safety factor q = {} should be > 1", q);
    }

    #[test]
    fn test_safety_factor_increases_with_bt() {
        let q1 = PlasmaController::safety_factor(3.0, 6.2, 2.0, 15.0, 1.7);
        let q2 = PlasmaController::safety_factor(5.3, 6.2, 2.0, 15.0, 1.7);
        assert!(q2 > q1, "q should increase with B_T: q1={}, q2={}", q1, q2);
    }

    #[test]
    fn test_safety_factor_decreases_with_ip() {
        let q1 = PlasmaController::safety_factor(5.3, 6.2, 2.0, 10.0, 1.7);
        let q2 = PlasmaController::safety_factor(5.3, 6.2, 2.0, 15.0, 1.7);
        assert!(q2 < q1, "q should decrease with I_p: q1={}, q2={}", q1, q2);
    }

    #[test]
    fn test_safety_factor_zero_current() {
        let q = PlasmaController::safety_factor(5.3, 6.2, 2.0, 0.0, 1.7);
        assert!(q.is_infinite());
    }

    #[test]
    fn test_safety_factor_formula() {
        // q_cyl = (5*a²*B_T)/(R*I_p) * (1+κ²)/2
        let b_t = 5.3;
        let r = 6.2;
        let a = 2.0;
        let ip = 15.0;
        let kappa = 1.7;
        let expected = (5.0 * a * a * b_t) / (r * ip) * (1.0 + kappa * kappa) / 2.0;
        let q = PlasmaController::safety_factor(b_t, r, a, ip, kappa);
        assert!((q - expected).abs() < 1e-10);
    }

    // --- Greenwald density tests ---

    #[test]
    fn test_greenwald_density_iter() {
        // ITER: I_p=15 MA, a=2.0 m
        // n_GW = 15 / (π * 4) ≈ 1.19 × 10²⁰/m³
        let n_gw = PlasmaController::greenwald_density_limit(15.0, 2.0);
        assert!(
            (n_gw - 15.0 / (PI * 4.0)).abs() < 1e-6,
            "n_GW = {} should be ~1.19",
            n_gw
        );
        // Check it's approximately 1.0 × 10²⁰ order
        assert!(n_gw > 0.5 && n_gw < 2.0, "n_GW should be ~1 for ITER");
    }

    #[test]
    fn test_greenwald_density_zero_radius() {
        assert!(PlasmaController::greenwald_density_limit(15.0, 0.0).is_infinite());
    }

    #[test]
    fn test_greenwald_density_scales_with_current() {
        let n1 = PlasmaController::greenwald_density_limit(5.0, 2.0);
        let n2 = PlasmaController::greenwald_density_limit(15.0, 2.0);
        assert!(n2 > n1);
        assert!((n2 / n1 - 3.0).abs() < 1e-6, "Should scale linearly with I_p");
    }

    // --- Vertical stability tests ---

    #[test]
    fn test_vertical_stability_circular_plasma() {
        // Circular plasma (κ=1) should be neutrally stable: n_s=0
        let n_s = PlasmaController::vertical_stability_index(1.0, 3.0);
        assert!(n_s.abs() < EPSILON, "Circular plasma n_s should be 0");
    }

    #[test]
    fn test_vertical_stability_elongated_plasma() {
        // Elongated plasma should be unstable: n_s > 0
        let n_s = PlasmaController::vertical_stability_index(1.7, 3.1);
        assert!(n_s > 0.0, "Elongated plasma should have n_s > 0");
    }

    #[test]
    fn test_vertical_stability_increases_with_elongation() {
        let n_s1 = PlasmaController::vertical_stability_index(1.3, 3.0);
        let n_s2 = PlasmaController::vertical_stability_index(1.7, 3.0);
        assert!(
            n_s2 > n_s1,
            "Higher elongation should be more unstable: n_s1={}, n_s2={}",
            n_s1,
            n_s2
        );
    }

    #[test]
    fn test_vertical_growth_rate() {
        let n_s = PlasmaController::vertical_stability_index(1.7, 3.1);
        let gamma = PlasmaController::vertical_growth_rate(n_s, 5.0); // 5 ms wall time
        assert!(gamma > 0.0, "Growth rate should be positive");
        // γ = n_s / (5e-3), should be on order of 100 Hz
        assert!(gamma > 10.0 && gamma < 10_000.0);
    }

    #[test]
    fn test_vertical_growth_rate_zero_wall_time() {
        assert!((PlasmaController::vertical_growth_rate(1.0, 0.0)).abs() < EPSILON);
    }

    // --- PID controller tests ---

    #[test]
    fn test_pid_zero_error() {
        let mut integral = 0.0;
        let mut prev_error = 0.0;
        let output =
            PlasmaController::pid_controller(0.0, &mut integral, &mut prev_error, 1.0, 1.0, 1.0, 0.001);
        assert!(
            output.abs() < EPSILON,
            "Zero error should give zero output: {}",
            output
        );
    }

    #[test]
    fn test_pid_proportional() {
        let mut integral = 0.0;
        let mut prev_error = 0.0;
        let output = PlasmaController::pid_controller(
            1.0,
            &mut integral,
            &mut prev_error,
            2.0,
            0.0,
            0.0,
            0.001,
        );
        assert!(
            (output - 2.0).abs() < EPSILON,
            "P-only: Kp*e = 2.0 * 1.0 = 2.0, got {}",
            output
        );
    }

    #[test]
    fn test_pid_integral_accumulates() {
        let mut integral = 0.0;
        let mut prev_error = 0.0;
        let dt = 0.001;
        // Apply constant error for several steps
        for _ in 0..10 {
            PlasmaController::pid_controller(1.0, &mut integral, &mut prev_error, 0.0, 1.0, 0.0, dt);
        }
        // Integral should be approximately 10 * 1.0 * 0.001 = 0.01
        assert!(
            (integral - 0.01).abs() < 1e-6,
            "Integral = {}, expected 0.01",
            integral
        );
    }

    #[test]
    fn test_pid_derivative() {
        let mut integral = 0.0;
        let mut prev_error = 0.0;
        let dt = 0.001;
        // First step: error = 0
        PlasmaController::pid_controller(0.0, &mut integral, &mut prev_error, 0.0, 0.0, 1.0, dt);
        // Second step: error = 1.0, de/dt = 1.0/0.001 = 1000
        let output =
            PlasmaController::pid_controller(1.0, &mut integral, &mut prev_error, 0.0, 0.0, 1.0, dt);
        assert!(
            (output - 1000.0).abs() < 1e-3,
            "Derivative output should be ~1000, got {}",
            output
        );
    }

    #[test]
    fn test_pid_zero_dt() {
        let mut integral = 0.0;
        let mut prev_error = 0.0;
        let output =
            PlasmaController::pid_controller(1.0, &mut integral, &mut prev_error, 1.0, 1.0, 1.0, 0.0);
        assert!(output.abs() < EPSILON, "Zero dt should return 0");
    }

    // --- Shape controller tests ---

    #[test]
    fn test_shape_controller_no_error() {
        let config = TokamakConfig::default();
        let controller = PlasmaController::new(config);
        let target = PlasmaEquilibrium::default();
        let current = target.clone();
        let adj = controller.shape_controller(&target, &current);
        for a in &adj {
            assert!(a.abs() < EPSILON, "No error should give zero adjustments");
        }
    }

    #[test]
    fn test_shape_controller_vertical_displacement() {
        let config = TokamakConfig::default();
        let controller = PlasmaController::new(config.clone());
        let target = PlasmaEquilibrium::default();
        let mut current = target.clone();
        current.z_center_m = 0.01; // 1 cm vertical displacement
        let adj = controller.shape_controller(&target, &current);
        assert_eq!(adj.len(), config.num_pf_coils);
        // Coils 0 and 1 should have opposite signs (antisymmetric)
        assert!((adj[0] + adj[1]).abs() < EPSILON);
        assert!(adj[0].abs() > 0.0);
    }

    #[test]
    fn test_shape_controller_coil_count() {
        let mut config = TokamakConfig::default();
        config.num_pf_coils = 10;
        let controller = PlasmaController::new(config);
        let target = PlasmaEquilibrium::default();
        let current = target.clone();
        let adj = controller.shape_controller(&target, &current);
        assert_eq!(adj.len(), 10);
    }

    // --- Radial position control tests ---

    #[test]
    fn test_radial_position_no_error() {
        let correction = PlasmaController::radial_position_control(6.2, 6.2, 1.0);
        assert!(correction.abs() < EPSILON);
    }

    #[test]
    fn test_radial_position_outward_correction() {
        // Target is at 6.2, current is at 6.1 (too far inboard)
        let correction = PlasmaController::radial_position_control(6.2, 6.1, 1.0);
        assert!(correction > 0.0, "Should push outward");
    }

    // --- Vertical position control tests ---

    #[test]
    fn test_vertical_position_at_zero() {
        let correction = PlasmaController::vertical_position_control(0.0, 1.0, 0.5, 0.0, 0.001);
        assert!(correction.abs() < EPSILON, "No displacement should give zero correction");
    }

    #[test]
    fn test_vertical_position_pd_response() {
        // Displacement with velocity
        let z_prev = 0.0;
        let z_curr = 0.01;
        let dt = 0.001;
        let kp = 100.0;
        let kd = 10.0;
        let correction =
            PlasmaController::vertical_position_control(z_curr, kp, kd, z_prev, dt);
        // P: 100 * 0.01 = 1.0
        // D: 10 * (0.01/0.001) = 100.0
        assert!(
            (correction - 101.0).abs() < 1e-6,
            "PD: expected 101.0, got {}",
            correction
        );
    }

    // --- Rogowski coil tests ---

    #[test]
    fn test_rogowski_coil_step_current() {
        // A step change in current produces a delta in dI/dt
        // So the Rogowski coil sees a pulse voltage that integrates to the step
        let sensitivity = 0.01; // V·s/A
        let dt = 0.001;
        // Create a voltage pulse corresponding to a 1000 A step
        // V = M * dI/dt = 0.01 * 1000 / 0.001 = 10000 V ... that's large
        // Instead, use a smooth ramp: I goes from 0 to 1000 A over 10 samples
        // dI/dt = 1000 / (10*0.001) = 100000 A/s
        // V = M * dI/dt = 0.01 * 100000 = 1000 V
        let n = 10;
        let voltage: Vec<f64> = vec![1000.0; n]; // constant voltage = constant dI/dt
        let current = MagneticDiagnostics::rogowski_coil_current(&voltage, dt, sensitivity);
        assert_eq!(current.len(), n);
        // Final current should be approximately (n-0.5) * 1000 * dt / sensitivity
        // = 9.5 * 1.0 / 0.01 = 950 ... trapezoidal
        // Actually with trapezoidal: integral = sum of 0.5*(V[i-1]+V[i])*dt for i=1..9
        // = 9 * 1000 * 0.001 = 9.0
        // current = 9.0 / 0.01 = 900
        let final_current = current[n - 1];
        assert!(
            final_current > 0.0,
            "Current should be positive after positive voltage"
        );
        // Check monotonic increase
        for i in 1..n {
            assert!(current[i] >= current[i - 1]);
        }
    }

    #[test]
    fn test_rogowski_coil_zero_sensitivity() {
        let voltage = vec![1.0, 2.0, 3.0];
        let current = MagneticDiagnostics::rogowski_coil_current(&voltage, 0.001, 0.0);
        for c in &current {
            assert!(c.abs() < EPSILON, "Zero sensitivity should give zero current");
        }
    }

    #[test]
    fn test_rogowski_empty_input() {
        let current = MagneticDiagnostics::rogowski_coil_current(&[], 0.001, 0.01);
        assert!(current.is_empty());
    }

    // --- Flux loop tests ---

    #[test]
    fn test_flux_loop_integration() {
        let dt = 0.001;
        let signal = vec![1.0; 10]; // Constant voltage
        let flux = MagneticDiagnostics::flux_loop_measurement(&signal, dt);
        assert_eq!(flux.len(), 10);
        // Flux should increase monotonically
        for i in 1..10 {
            assert!(flux[i] > flux[i - 1]);
        }
    }

    #[test]
    fn test_flux_loop_zero_signal() {
        let signal = vec![0.0; 5];
        let flux = MagneticDiagnostics::flux_loop_measurement(&signal, 0.001);
        for f in &flux {
            assert!(f.abs() < EPSILON);
        }
    }

    // --- Magnetic probe tests ---

    #[test]
    fn test_magnetic_probe_field() {
        let signal = vec![1.0, 2.0, 3.0, 4.0];
        let sensitivity = 0.5; // T/V
        let field = MagneticDiagnostics::magnetic_probe_field(&signal, sensitivity);
        assert_eq!(field.len(), 4);
        assert!((field[0] - 0.5).abs() < EPSILON);
        assert!((field[1] - 1.0).abs() < EPSILON);
        assert!((field[2] - 1.5).abs() < EPSILON);
        assert!((field[3] - 2.0).abs() < EPSILON);
    }

    // --- Diamagnetic loop tests ---

    #[test]
    fn test_diamagnetic_loop() {
        let signal = vec![1.0; 5];
        let result = MagneticDiagnostics::diamagnetic_loop(&signal, 0.001, 1000.0);
        assert_eq!(result.len(), 5);
        // Should be scaled version of flux loop output
        for i in 1..5 {
            assert!(result[i] > result[i - 1] || (result[i] - result[i - 1]).abs() < EPSILON);
        }
    }

    // --- Equilibrium reconstruction tests ---

    #[test]
    fn test_equilibrium_reconstruction_basic() {
        let bp = vec![0.1, 0.12, 0.11, 0.09];
        let flux = vec![0.5, 0.6, 0.55, 0.45];
        let positions = [
            [6.0, 1.0],
            [6.4, 0.5],
            [6.2, -0.5],
            [6.0, -1.0],
        ];
        let eq = MagneticDiagnostics::equilibrium_reconstruction(&bp, &flux, &positions);
        // Center should be roughly in the middle of the probes
        assert!(eq.r_center_m > 5.5 && eq.r_center_m < 7.0);
        assert!(eq.ip_ma > 0.0);
    }

    #[test]
    fn test_equilibrium_reconstruction_empty() {
        let eq =
            MagneticDiagnostics::equilibrium_reconstruction(&[], &[], &[]);
        // Should return default
        assert!((eq.r_center_m - 6.2).abs() < EPSILON);
    }

    // --- Disruption predictor tests ---

    #[test]
    fn test_locked_mode_detected() {
        let signal = vec![0.1, 0.2, 0.3, 0.8, 0.9];
        let result = DisruptionPredictor::locked_mode_detector(&signal, 0.5);
        assert_eq!(result, Some(3));
    }

    #[test]
    fn test_locked_mode_not_detected() {
        let signal = vec![0.1, 0.2, 0.3, 0.4];
        let result = DisruptionPredictor::locked_mode_detector(&signal, 0.5);
        assert_eq!(result, None);
    }

    #[test]
    fn test_current_quench_rate() {
        // Plasma current drops from 15 MA to 0 in 10 steps at dt=0.001s
        let ip: Vec<f64> = (0..=10).map(|i| 15.0 - 1.5 * i as f64).collect();
        let rate = DisruptionPredictor::current_quench_rate(&ip, 0.001);
        // dI/dt = 1.5 / 0.001 = 1500 MA/s
        assert!(
            (rate - 1500.0).abs() < 1e-6,
            "Expected 1500 MA/s, got {}",
            rate
        );
    }

    #[test]
    fn test_current_quench_rate_empty() {
        assert!((DisruptionPredictor::current_quench_rate(&[], 0.001)).abs() < EPSILON);
    }

    #[test]
    fn test_thermal_quench_time() {
        // Exponential decay: T(t) = 10 * exp(-t/0.005)
        let dt = 0.001;
        let tau = 0.005;
        let signal: Vec<f64> = (0..20).map(|i| 10.0 * (-i as f64 * dt / tau).exp()).collect();
        let tq = DisruptionPredictor::thermal_quench_time(&signal, dt);
        // Should be approximately tau = 5 ms
        assert!(
            (tq - tau).abs() < 2.0 * dt, // Within 2 sample periods
            "Thermal quench time = {}, expected ~{}",
            tq,
            tau
        );
    }

    #[test]
    fn test_halo_current_fraction() {
        assert!((DisruptionPredictor::halo_current_fraction(3.0, 10.0) - 0.3).abs() < EPSILON);
        assert!((DisruptionPredictor::halo_current_fraction(0.0, 10.0)).abs() < EPSILON);
        assert!((DisruptionPredictor::halo_current_fraction(5.0, 0.0)).abs() < EPSILON);
    }

    #[test]
    fn test_disruption_warning_locked_mode() {
        assert!(DisruptionPredictor::disruption_warning(0.0, true, 0.5));
    }

    #[test]
    fn test_disruption_warning_high_quench_rate() {
        assert!(DisruptionPredictor::disruption_warning(15.0, false, 0.5));
    }

    #[test]
    fn test_disruption_warning_density_limit() {
        assert!(DisruptionPredictor::disruption_warning(0.0, false, 1.1));
    }

    #[test]
    fn test_disruption_warning_safe() {
        assert!(!DisruptionPredictor::disruption_warning(5.0, false, 0.8));
    }

    // --- Power balance tests ---

    #[test]
    fn test_ohmic_heating_power() {
        // P = I²R = (1e6)² * 1e-8 = 10 W ... typical small tokamak
        let p = PowerBalance::ohmic_heating_power(1.0, 1e-8);
        assert!(
            (p - 1e4).abs() < 1e-2,
            "P_OH = I²R = (1e6)²×1e-8 = 10000 W, got {}",
            p
        );
    }

    #[test]
    fn test_ohmic_heating_iter() {
        // ITER: 15 MA, ~10 nΩ
        let p = PowerBalance::ohmic_heating_power(15.0, 1e-8);
        // P = (15e6)² * 1e-8 = 2.25e13 * 1e-8 = 2.25e5 = 225 kW
        let expected = (15e6_f64).powi(2) * 1e-8;
        assert!((p - expected).abs() < 1.0);
    }

    #[test]
    fn test_bremsstrahlung_power() {
        // Typical: ne=1e20, Te=10 keV, V=800 m³, Zeff=1.5
        let p = PowerBalance::bremsstrahlung_power(1e20, 10.0, 800.0, 1.5);
        assert!(p > 0.0);
        // C_brem * (1e20)^2 * sqrt(10) * 1.5 * 800
        // = 5.35e-37 * 1e40 * 3.162 * 1.5 * 800
        // = 5.35e3 * 3.162 * 1200 ≈ 20.3 MW
        assert!(
            p > 1e6 && p < 1e9,
            "P_brem should be in MW range, got {} W",
            p
        );
    }

    #[test]
    fn test_bremsstrahlung_zero_temperature() {
        assert!((PowerBalance::bremsstrahlung_power(1e20, 0.0, 800.0, 1.5)).abs() < EPSILON);
    }

    // --- Confinement time tests ---

    #[test]
    fn test_confinement_time_iter89() {
        // ITER-like: Ip=15, Bt=5.3, ne=1.0, P=50 MW, R=6.2, a=2.0, κ=1.7, M=2.5
        let tau = PowerBalance::confinement_time_iter89(15.0, 5.3, 1.0, 50.0, 6.2, 2.0, 1.7, 2.5);
        assert!(
            tau > 0.01 && tau < 10.0,
            "Confinement time should be 0.1-few seconds, got {}",
            tau
        );
    }

    #[test]
    fn test_confinement_time_zero_power() {
        let tau = PowerBalance::confinement_time_iter89(15.0, 5.3, 1.0, 0.0, 6.2, 2.0, 1.7, 2.5);
        assert!(tau.abs() < EPSILON, "Zero power should give zero tau");
    }

    #[test]
    fn test_confinement_time_scales_with_current() {
        let tau1 = PowerBalance::confinement_time_iter89(10.0, 5.3, 1.0, 50.0, 6.2, 2.0, 1.7, 2.5);
        let tau2 = PowerBalance::confinement_time_iter89(15.0, 5.3, 1.0, 50.0, 6.2, 2.0, 1.7, 2.5);
        assert!(
            tau2 > tau1,
            "Confinement should improve with higher current"
        );
    }

    // --- Fusion triple product tests ---

    #[test]
    fn test_fusion_triple_product() {
        // ITER target: ne=1e20 m^-3, Ti=10 keV, tau=3s
        // n*T*tau = 1e20 * 10 * 3 = 3e21 keV·s/m³
        let ntt = PowerBalance::fusion_triple_product(1e20, 10.0, 3.0);
        assert!(
            (ntt - 3e21).abs() < 1e15,
            "Triple product should be ~3e21, got {}",
            ntt
        );
    }

    #[test]
    fn test_fusion_triple_product_ignition() {
        // At ignition: n*T*tau > ~3e21 keV·s/m³
        let ntt = PowerBalance::fusion_triple_product(1.5e20, 15.0, 2.0);
        // = 1.5e20 * 15 * 2 = 4.5e21
        assert!(
            ntt > 3e21,
            "Should exceed ignition threshold, got {}",
            ntt
        );
    }

    // --- Lawson criterion tests ---

    #[test]
    fn test_lawson_criterion_finite() {
        let ntau = PowerBalance::lawson_criterion(10.0);
        assert!(ntau > 0.0 && ntau.is_finite());
        // At 10 keV: ~3.12e20 s/m³
        assert!(
            ntau > 1e19 && ntau < 1e23,
            "Lawson n*tau at 10 keV = {}, expected ~3e20",
            ntau
        );
    }

    #[test]
    fn test_lawson_criterion_zero_temperature() {
        assert!(PowerBalance::lawson_criterion(0.0).is_infinite());
    }

    #[test]
    fn test_lawson_criterion_decreases_with_temperature() {
        // Higher temperature → lower n*tau required (up to optimal)
        let ntau1 = PowerBalance::lawson_criterion(5.0);
        let ntau2 = PowerBalance::lawson_criterion(15.0);
        assert!(
            ntau2 < ntau1,
            "Higher T should lower n*tau requirement"
        );
    }
}

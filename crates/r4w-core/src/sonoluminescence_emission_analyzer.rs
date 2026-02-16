//! Sonoluminescence Emission Analyzer — Signal Processing for Acoustic Cavitation Light Emission
//!
//! Implements signal processing for sonoluminescence (SL) — light emission from
//! collapsing acoustic cavitation bubbles. Applications include sonochemistry,
//! medical ultrasound, plasma physics, extreme state research, and fusion studies.
//!
//! ## Physics Background
//!
//! Sonoluminescence occurs when an acoustically driven bubble undergoes violent
//! inertial collapse. During the final stage of compression, the bubble interior
//! reaches extreme temperatures (10,000–50,000 K for single-bubble SL), producing
//! a brief flash of broadband light with duration < 100 ps.
//!
//! ## Rayleigh-Plesset Equation
//!
//! Spherical bubble dynamics are governed by:
//!
//! ```text
//! R·R'' + (3/2)·R'^2 = (1/ρ)·[P_gas·(R0/R)^(3γ) - P0 - P_ac·sin(ωt) - 4μ·R'/R - 2σ/R]
//! ```
//!
//! where R is the bubble radius, R0 is the equilibrium radius, ρ is liquid density,
//! P_gas is the initial gas pressure, γ is the polytropic exponent, P0 is ambient
//! pressure, P_ac is acoustic pressure amplitude, μ is dynamic viscosity, and σ is
//! surface tension.
//!
//! ## Processing Chain
//!
//! ```text
//! Hydrophone Signal ── Pressure Calibration ── Cavitation Detection
//!                                                      │
//! Acoustic Drive ── Rayleigh-Plesset Solver ── Bubble Radius R(t)
//!                                                      │
//!                                              Collapse Timing ── Emission Analysis
//!                                                      │               │
//!                                              Flash Duration    Spectrum Fitting
//!                                                      │               │
//!                                              Stability Analysis  Temperature Est.
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::sonoluminescence_emission_analyzer::{
//!     SonoluminescenceConfig, RayleighPlessetSolver, EmissionTimingAnalyzer,
//! };
//!
//! // Configure for typical SBSL experiment in water at 25 kHz
//! let config = SonoluminescenceConfig::sbsl_water_25khz();
//! let mut solver = RayleighPlessetSolver::new(&config);
//!
//! // Simulate bubble dynamics over one acoustic cycle
//! let dt = 1e-9; // 1 ns timestep
//! let n_steps = (1.0 / config.driving_freq_hz / dt) as usize;
//! let trajectory = solver.simulate(n_steps, dt);
//!
//! // Find collapse events
//! let analyzer = EmissionTimingAnalyzer::new(config.driving_freq_hz);
//! let collapses = analyzer.find_collapses(&trajectory);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Speed of light in vacuum (m/s).
const C_LIGHT: f64 = 299_792_458.0;

/// Planck constant (J·s).
const H_PLANCK: f64 = 6.626_070_15e-34;

/// Boltzmann constant (J/K).
const K_BOLTZMANN: f64 = 1.380_649e-23;

/// Standard atmospheric pressure (Pa).
const P_ATM: f64 = 101_325.0;

/// Speed of sound in water at 20 °C (m/s).
const C_SOUND_WATER: f64 = 1482.0;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Gas type inside the cavitation bubble.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GasType {
    /// Argon — noble gas, γ = 5/3. Best for single-bubble SL.
    Argon,
    /// Air — N₂/O₂ mix, γ ≈ 1.4.
    Air,
    /// Helium — noble gas, γ = 5/3, lighter.
    Helium,
    /// Xenon — noble gas, γ = 5/3, heavier, brighter SL.
    Xenon,
    /// Custom polytropic exponent.
    Custom(f64),
}

impl GasType {
    /// Polytropic exponent γ for the gas.
    pub fn gamma(&self) -> f64 {
        match self {
            GasType::Argon | GasType::Helium | GasType::Xenon => 5.0 / 3.0,
            GasType::Air => 1.4,
            GasType::Custom(g) => *g,
        }
    }
}

/// Configuration for a sonoluminescence experiment.
#[derive(Debug, Clone)]
pub struct SonoluminescenceConfig {
    /// Acoustic driving frequency (Hz). Typical SBSL: 20–40 kHz.
    pub driving_freq_hz: f64,
    /// Acoustic pressure amplitude (Pa). Typical SBSL: 1.2–1.5 atm.
    pub pressure_amplitude_pa: f64,
    /// Equilibrium bubble radius (m). Typical SBSL: 4–5 µm.
    pub equilibrium_radius_m: f64,
    /// Liquid density (kg/m³). Water ≈ 998.
    pub liquid_density: f64,
    /// Dynamic viscosity of liquid (Pa·s). Water at 20 °C ≈ 1.002e-3.
    pub liquid_viscosity: f64,
    /// Surface tension (N/m). Water ≈ 0.0728.
    pub surface_tension: f64,
    /// Static / ambient pressure (Pa). Typically 1 atm.
    pub ambient_pressure_pa: f64,
    /// Gas type (determines polytropic exponent).
    pub gas_type: GasType,
    /// Speed of sound in liquid (m/s).
    pub sound_speed: f64,
}

impl SonoluminescenceConfig {
    /// Typical single-bubble SL experiment: water, argon, 25 kHz.
    pub fn sbsl_water_25khz() -> Self {
        Self {
            driving_freq_hz: 25_000.0,
            pressure_amplitude_pa: 1.3 * P_ATM,
            equilibrium_radius_m: 4.5e-6,
            liquid_density: 998.0,
            liquid_viscosity: 1.002e-3,
            surface_tension: 0.0728,
            ambient_pressure_pa: P_ATM,
            gas_type: GasType::Argon,
            sound_speed: C_SOUND_WATER,
        }
    }

    /// Multi-bubble SL experiment: water, air, 20 kHz.
    pub fn mbsl_water_20khz() -> Self {
        Self {
            driving_freq_hz: 20_000.0,
            pressure_amplitude_pa: 1.5 * P_ATM,
            equilibrium_radius_m: 5.0e-6,
            liquid_density: 998.0,
            liquid_viscosity: 1.002e-3,
            surface_tension: 0.0728,
            ambient_pressure_pa: P_ATM,
            gas_type: GasType::Air,
            sound_speed: C_SOUND_WATER,
        }
    }

    /// Angular frequency ω = 2πf.
    pub fn omega(&self) -> f64 {
        2.0 * PI * self.driving_freq_hz
    }

    /// Acoustic period T = 1/f.
    pub fn acoustic_period_s(&self) -> f64 {
        1.0 / self.driving_freq_hz
    }

    /// Initial gas pressure inside the bubble via Laplace: P_gas = P0 + 2σ/R0.
    pub fn initial_gas_pressure(&self) -> f64 {
        self.ambient_pressure_pa + 2.0 * self.surface_tension / self.equilibrium_radius_m
    }
}

// ---------------------------------------------------------------------------
// Rayleigh-Plesset ODE Solver
// ---------------------------------------------------------------------------

/// State of the bubble: [R, R'].
#[derive(Debug, Clone, Copy)]
pub struct BubbleState {
    /// Bubble radius (m).
    pub radius: f64,
    /// Radial velocity dR/dt (m/s).
    pub velocity: f64,
    /// Current time (s).
    pub time: f64,
}

/// A single point on the bubble trajectory.
#[derive(Debug, Clone, Copy)]
pub struct TrajectoryPoint {
    /// Time (s).
    pub time: f64,
    /// Bubble radius (m).
    pub radius: f64,
    /// Radial velocity (m/s).
    pub velocity: f64,
    /// Radial acceleration (m/s²).
    pub acceleration: f64,
    /// Compression ratio R0/R.
    pub compression_ratio: f64,
}

/// Rayleigh-Plesset equation solver for spherical bubble dynamics.
///
/// Solves the ODE:
///   R·R'' + (3/2)·R'² = (1/ρ)·[P_gas·(R0/R)^(3γ) - P0 - P_ac·sin(ωt) - 4μ·R'/R - 2σ/R]
///
/// Uses 4th-order Runge-Kutta (RK4) integration.
pub struct RayleighPlessetSolver {
    /// Configuration parameters.
    config: SonoluminescenceConfig,
    /// Current bubble state.
    state: BubbleState,
    /// γ (polytropic exponent).
    gamma: f64,
    /// Initial gas pressure P_gas0.
    p_gas0: f64,
}

impl RayleighPlessetSolver {
    /// Create a new solver from configuration.
    ///
    /// Initializes the bubble at its equilibrium radius with zero velocity.
    pub fn new(config: &SonoluminescenceConfig) -> Self {
        let gamma = config.gas_type.gamma();
        let p_gas0 = config.initial_gas_pressure();
        Self {
            config: config.clone(),
            state: BubbleState {
                radius: config.equilibrium_radius_m,
                velocity: 0.0,
                time: 0.0,
            },
            gamma,
            p_gas0,
        }
    }

    /// Reset the solver to equilibrium initial conditions.
    pub fn reset(&mut self) {
        self.state = BubbleState {
            radius: self.config.equilibrium_radius_m,
            velocity: 0.0,
            time: 0.0,
        };
    }

    /// Compute R'' from the Rayleigh-Plesset equation.
    ///
    /// R'' = (1/R)·[(1/ρ)·(P_gas·(R0/R)^(3γ) - P0 - P_ac·sin(ωt) - 4μ·R'/R - 2σ/R) - (3/2)·R'²]
    fn compute_acceleration(&self, t: f64, r: f64, rdot: f64) -> f64 {
        let c = &self.config;
        let r0 = c.equilibrium_radius_m;
        let rho = c.liquid_density;
        let omega = c.omega();

        // Gas pressure (adiabatic compression)
        let ratio = r0 / r;
        let p_gas = self.p_gas0 * ratio.powf(3.0 * self.gamma);

        // Acoustic driving pressure
        let p_acoustic = c.pressure_amplitude_pa * (omega * t).sin();

        // Viscous damping
        let viscous = 4.0 * c.liquid_viscosity * rdot / r;

        // Surface tension (Laplace pressure)
        let surface = 2.0 * c.surface_tension / r;

        // Net pressure
        let p_net = p_gas - c.ambient_pressure_pa - p_acoustic - viscous - surface;

        // Rayleigh-Plesset: R·R'' + (3/2)R'² = p_net/ρ
        // => R'' = (p_net/ρ - (3/2)R'²) / R
        (p_net / rho - 1.5 * rdot * rdot) / r
    }

    /// Advance one RK4 step.
    fn rk4_step(&self, t: f64, r: f64, rdot: f64, dt: f64) -> (f64, f64) {
        // State: y = [r, rdot], y' = [rdot, rddot]
        let k1_r = rdot;
        let k1_v = self.compute_acceleration(t, r, rdot);

        let k2_r = rdot + 0.5 * dt * k1_v;
        let k2_v = self.compute_acceleration(t + 0.5 * dt, r + 0.5 * dt * k1_r, k2_r);

        let k3_r = rdot + 0.5 * dt * k2_v;
        let k3_v = self.compute_acceleration(t + 0.5 * dt, r + 0.5 * dt * k2_r, k3_r);

        let k4_r = rdot + dt * k3_v;
        let k4_v = self.compute_acceleration(t + dt, r + dt * k3_r, k4_r);

        let r_new = r + (dt / 6.0) * (k1_r + 2.0 * k2_r + 2.0 * k3_r + k4_r);
        let v_new = rdot + (dt / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v);

        (r_new.max(1e-12), v_new) // clamp radius to avoid singularity
    }

    /// Step the solver forward by dt seconds.
    pub fn step(&mut self, dt: f64) -> BubbleState {
        let (r_new, v_new) = self.rk4_step(
            self.state.time,
            self.state.radius,
            self.state.velocity,
            dt,
        );
        self.state.time += dt;
        self.state.radius = r_new;
        self.state.velocity = v_new;
        self.state
    }

    /// Simulate for `n_steps` at timestep `dt`, returning full trajectory.
    pub fn simulate(&mut self, n_steps: usize, dt: f64) -> Vec<TrajectoryPoint> {
        let mut trajectory = Vec::with_capacity(n_steps + 1);
        let r0 = self.config.equilibrium_radius_m;

        // Record initial state
        let acc = self.compute_acceleration(
            self.state.time,
            self.state.radius,
            self.state.velocity,
        );
        trajectory.push(TrajectoryPoint {
            time: self.state.time,
            radius: self.state.radius,
            velocity: self.state.velocity,
            acceleration: acc,
            compression_ratio: r0 / self.state.radius,
        });

        for _ in 0..n_steps {
            let st = self.step(dt);
            let acc = self.compute_acceleration(st.time, st.radius, st.velocity);
            trajectory.push(TrajectoryPoint {
                time: st.time,
                radius: st.radius,
                velocity: st.velocity,
                acceleration: acc,
                compression_ratio: r0 / st.radius,
            });
        }

        trajectory
    }

    /// Current bubble state.
    pub fn state(&self) -> BubbleState {
        self.state
    }
}

// ---------------------------------------------------------------------------
// Bubble Radius Tracker
// ---------------------------------------------------------------------------

/// Statistics from a bubble radius trajectory.
#[derive(Debug, Clone)]
pub struct BubbleRadiusStats {
    /// Maximum radius reached during expansion (m).
    pub max_radius: f64,
    /// Minimum radius reached during collapse (m).
    pub min_radius: f64,
    /// Maximum compression ratio R0/R_min.
    pub max_compression_ratio: f64,
    /// Maximum collapse speed |dR/dt| (m/s).
    pub max_collapse_speed: f64,
    /// Mach number of collapse: |dR/dt|_max / c_sound.
    pub collapse_mach_number: f64,
    /// Time of maximum expansion (s).
    pub time_of_max_radius: f64,
    /// Time of minimum radius / maximum compression (s).
    pub time_of_min_radius: f64,
}

/// Track bubble radius oscillation and extract collapse dynamics.
pub struct BubbleRadiusTracker {
    sound_speed: f64,
    equilibrium_radius: f64,
}

impl BubbleRadiusTracker {
    /// Create a new tracker.
    pub fn new(config: &SonoluminescenceConfig) -> Self {
        Self {
            sound_speed: config.sound_speed,
            equilibrium_radius: config.equilibrium_radius_m,
        }
    }

    /// Analyze a trajectory for collapse dynamics.
    pub fn analyze(&self, trajectory: &[TrajectoryPoint]) -> BubbleRadiusStats {
        let mut max_r = f64::MIN;
        let mut min_r = f64::MAX;
        let mut max_speed = 0.0_f64;
        let mut t_max = 0.0;
        let mut t_min = 0.0;

        for pt in trajectory {
            if pt.radius > max_r {
                max_r = pt.radius;
                t_max = pt.time;
            }
            if pt.radius < min_r {
                min_r = pt.radius;
                t_min = pt.time;
            }
            // Collapse speed: negative velocity magnitude during contraction
            let speed = pt.velocity.abs();
            if speed > max_speed {
                max_speed = speed;
            }
        }

        BubbleRadiusStats {
            max_radius: max_r,
            min_radius: min_r,
            max_compression_ratio: self.equilibrium_radius / min_r,
            max_collapse_speed: max_speed,
            collapse_mach_number: max_speed / self.sound_speed,
            time_of_max_radius: t_max,
            time_of_min_radius: t_min,
        }
    }

    /// Find indices of local minima in the radius trajectory (collapse events).
    pub fn find_collapse_indices(&self, trajectory: &[TrajectoryPoint]) -> Vec<usize> {
        if trajectory.len() < 3 {
            return vec![];
        }
        let mut minima = Vec::new();
        for i in 1..trajectory.len() - 1 {
            if trajectory[i].radius < trajectory[i - 1].radius
                && trajectory[i].radius < trajectory[i + 1].radius
                && trajectory[i].radius < self.equilibrium_radius * 0.95
            {
                minima.push(i);
            }
        }
        minima
    }
}

// ---------------------------------------------------------------------------
// Emission Timing Analyzer
// ---------------------------------------------------------------------------

/// A detected collapse event with timing information.
#[derive(Debug, Clone, Copy)]
pub struct CollapseEvent {
    /// Index in trajectory.
    pub index: usize,
    /// Time of collapse (s).
    pub time: f64,
    /// Minimum radius at collapse (m).
    pub min_radius: f64,
    /// Phase of acoustic cycle at collapse (radians, 0 to 2π).
    pub acoustic_phase: f64,
    /// Collapse speed |dR/dt| at minimum (m/s).
    pub collapse_speed: f64,
}

/// Analyze emission timing relative to acoustic driving cycle.
pub struct EmissionTimingAnalyzer {
    driving_freq_hz: f64,
}

impl EmissionTimingAnalyzer {
    /// Create a new emission timing analyzer.
    pub fn new(driving_freq_hz: f64) -> Self {
        Self { driving_freq_hz }
    }

    /// Find collapse events in a trajectory.
    pub fn find_collapses(&self, trajectory: &[TrajectoryPoint]) -> Vec<CollapseEvent> {
        if trajectory.len() < 3 {
            return vec![];
        }

        let omega = 2.0 * PI * self.driving_freq_hz;
        let mut events = Vec::new();

        for i in 1..trajectory.len() - 1 {
            // Local minimum in radius
            if trajectory[i].radius < trajectory[i - 1].radius
                && trajectory[i].radius < trajectory[i + 1].radius
            {
                let phase = (omega * trajectory[i].time) % (2.0 * PI);
                events.push(CollapseEvent {
                    index: i,
                    time: trajectory[i].time,
                    min_radius: trajectory[i].radius,
                    acoustic_phase: phase,
                    collapse_speed: trajectory[i].velocity.abs(),
                });
            }
        }

        events
    }

    /// Compute jitter of collapse timing across multiple cycles.
    ///
    /// Returns (mean_phase, phase_std_dev) in radians.
    pub fn timing_jitter(&self, events: &[CollapseEvent]) -> (f64, f64) {
        if events.is_empty() {
            return (0.0, 0.0);
        }

        let n = events.len() as f64;
        let phases: Vec<f64> = events.iter().map(|e| e.acoustic_phase).collect();

        // Circular mean (use sin/cos to handle wraparound)
        let sin_mean: f64 = phases.iter().map(|p| p.sin()).sum::<f64>() / n;
        let cos_mean: f64 = phases.iter().map(|p| p.cos()).sum::<f64>() / n;
        let mean_phase = sin_mean.atan2(cos_mean);
        let mean_phase = if mean_phase < 0.0 {
            mean_phase + 2.0 * PI
        } else {
            mean_phase
        };

        // Circular standard deviation
        let r_bar = (sin_mean * sin_mean + cos_mean * cos_mean).sqrt();
        let circ_std = (-2.0 * r_bar.ln()).sqrt().min(PI); // cap at π

        (mean_phase, circ_std)
    }

    /// Mean inter-collapse interval and its standard deviation (seconds).
    pub fn inter_collapse_stats(&self, events: &[CollapseEvent]) -> (f64, f64) {
        if events.len() < 2 {
            return (0.0, 0.0);
        }
        let intervals: Vec<f64> = events
            .windows(2)
            .map(|w| w[1].time - w[0].time)
            .collect();
        let n = intervals.len() as f64;
        let mean = intervals.iter().sum::<f64>() / n;
        let var = intervals.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n;
        (mean, var.sqrt())
    }
}

// ---------------------------------------------------------------------------
// Spectrum Analyzer — Blackbody Fitting
// ---------------------------------------------------------------------------

/// Result of blackbody spectrum fitting.
#[derive(Debug, Clone)]
pub struct BlackbodyFitResult {
    /// Estimated temperature (K).
    pub temperature_k: f64,
    /// Scaling factor (amplitude).
    pub amplitude: f64,
    /// Residual sum of squares.
    pub residual: f64,
}

/// Analyze emission spectrum and fit blackbody models.
pub struct SpectrumAnalyzer;

impl SpectrumAnalyzer {
    /// Planck's law: spectral radiance B(λ, T).
    ///
    /// I(λ) = (2hc²/λ⁵) / (exp(hc/(λ·k_B·T)) - 1)
    ///
    /// λ in meters, T in Kelvin. Returns W·sr⁻¹·m⁻³.
    pub fn planck_spectral_radiance(wavelength_m: f64, temperature_k: f64) -> f64 {
        let hc = H_PLANCK * C_LIGHT;
        let numerator = 2.0 * hc * C_LIGHT / wavelength_m.powi(5);
        let exponent = hc / (wavelength_m * K_BOLTZMANN * temperature_k);
        // Prevent overflow for large exponents
        if exponent > 500.0 {
            return 0.0;
        }
        numerator / (exponent.exp() - 1.0)
    }

    /// Wien's displacement law: peak wavelength for temperature T.
    ///
    /// λ_max = b / T, where b = 2.897_771_955e-3 m·K.
    pub fn wien_peak_wavelength(temperature_k: f64) -> f64 {
        2.897_771_955e-3 / temperature_k
    }

    /// Estimate temperature from peak wavelength using Wien's law.
    pub fn temperature_from_peak_wavelength(peak_wavelength_m: f64) -> f64 {
        2.897_771_955e-3 / peak_wavelength_m
    }

    /// Fit a blackbody spectrum to measured data.
    ///
    /// `wavelengths_m`: wavelength values in meters.
    /// `intensities`: measured spectral intensity values.
    /// `temp_range`: (T_min, T_max) in Kelvin to search.
    /// `n_steps`: number of temperature steps to try.
    ///
    /// Uses least-squares grid search with amplitude scaling.
    pub fn fit_blackbody(
        wavelengths_m: &[f64],
        intensities: &[f64],
        temp_range: (f64, f64),
        n_steps: usize,
    ) -> BlackbodyFitResult {
        assert_eq!(wavelengths_m.len(), intensities.len());
        assert!(wavelengths_m.len() >= 2);

        let mut best_temp = temp_range.0;
        let mut best_amp = 1.0;
        let mut best_residual = f64::MAX;

        let dt = (temp_range.1 - temp_range.0) / n_steps as f64;

        for i in 0..=n_steps {
            let t = temp_range.0 + i as f64 * dt;

            // Compute model spectrum
            let model: Vec<f64> = wavelengths_m
                .iter()
                .map(|&lam| Self::planck_spectral_radiance(lam, t))
                .collect();

            // Optimal amplitude: a = Σ(m·d) / Σ(m²)
            let dot_md: f64 = model.iter().zip(intensities).map(|(m, d)| m * d).sum();
            let dot_mm: f64 = model.iter().map(|m| m * m).sum();

            if dot_mm < 1e-300 {
                continue;
            }

            let amp = dot_md / dot_mm;
            if amp <= 0.0 {
                continue;
            }

            // Residual
            let residual: f64 = model
                .iter()
                .zip(intensities)
                .map(|(m, d)| (amp * m - d).powi(2))
                .sum();

            if residual < best_residual {
                best_residual = residual;
                best_temp = t;
                best_amp = amp;
            }
        }

        BlackbodyFitResult {
            temperature_k: best_temp,
            amplitude: best_amp,
            residual: best_residual,
        }
    }

    /// Generate a synthetic blackbody spectrum for given temperature.
    pub fn generate_blackbody_spectrum(
        temperature_k: f64,
        wavelengths_m: &[f64],
    ) -> Vec<f64> {
        wavelengths_m
            .iter()
            .map(|&lam| Self::planck_spectral_radiance(lam, temperature_k))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Flash Duration Measurer
// ---------------------------------------------------------------------------

/// Result of flash duration analysis.
#[derive(Debug, Clone)]
pub struct FlashDurationResult {
    /// Full-width at half-maximum (s).
    pub fwhm_s: f64,
    /// Peak intensity (arb. units).
    pub peak_intensity: f64,
    /// Time of peak (s).
    pub peak_time_s: f64,
    /// Rise time (10% to 90% of peak, seconds).
    pub rise_time_s: f64,
    /// Fall time (90% to 10% of peak, seconds).
    pub fall_time_s: f64,
    /// Total flash energy (integrated intensity, arb. units·s).
    pub total_energy: f64,
}

/// Measure temporal profile of sonoluminescence flash.
pub struct FlashDurationMeasurer;

impl FlashDurationMeasurer {
    /// Analyze a time-resolved intensity profile.
    ///
    /// `times_s`: time values in seconds.
    /// `intensities`: corresponding intensity values.
    pub fn analyze(times_s: &[f64], intensities: &[f64]) -> FlashDurationResult {
        assert_eq!(times_s.len(), intensities.len());
        assert!(!times_s.is_empty());

        // Find peak
        let (peak_idx, &peak_val) = intensities
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        let peak_time = times_s[peak_idx];

        let half_max = peak_val / 2.0;

        // FWHM: find half-max crossings on each side
        let left_time = Self::find_crossing(times_s, intensities, half_max, 0, peak_idx, true);
        let right_time =
            Self::find_crossing(times_s, intensities, half_max, peak_idx, intensities.len() - 1, false);
        let fwhm = right_time - left_time;

        // Rise time (10% to 90%)
        let t10 = Self::find_crossing(times_s, intensities, peak_val * 0.1, 0, peak_idx, true);
        let t90 = Self::find_crossing(times_s, intensities, peak_val * 0.9, 0, peak_idx, true);
        let rise_time = t90 - t10;

        // Fall time (90% to 10%)
        let t90_fall =
            Self::find_crossing(times_s, intensities, peak_val * 0.9, peak_idx, intensities.len() - 1, false);
        let t10_fall =
            Self::find_crossing(times_s, intensities, peak_val * 0.1, peak_idx, intensities.len() - 1, false);
        let fall_time = t10_fall - t90_fall;

        // Total energy via trapezoidal integration
        let energy = Self::trapz(times_s, intensities);

        FlashDurationResult {
            fwhm_s: fwhm.max(0.0),
            peak_intensity: peak_val,
            peak_time_s: peak_time,
            rise_time_s: rise_time.max(0.0),
            fall_time_s: fall_time.max(0.0),
            total_energy: energy,
        }
    }

    /// Find linear-interpolated crossing of threshold between two indices.
    fn find_crossing(
        times: &[f64],
        values: &[f64],
        threshold: f64,
        start: usize,
        end: usize,
        rising: bool,
    ) -> f64 {
        if rising {
            for i in start..end {
                if values[i] <= threshold && values[i + 1] >= threshold {
                    let frac = (threshold - values[i]) / (values[i + 1] - values[i]);
                    return times[i] + frac * (times[i + 1] - times[i]);
                }
            }
        } else {
            for i in start..end {
                if values[i] >= threshold && values[i + 1] <= threshold {
                    let frac = (values[i] - threshold) / (values[i] - values[i + 1]);
                    return times[i] + frac * (times[i + 1] - times[i]);
                }
            }
        }
        // Fallback: return boundary time
        if rising { times[start] } else { times[end] }
    }

    /// Trapezoidal integration.
    fn trapz(x: &[f64], y: &[f64]) -> f64 {
        let mut sum = 0.0;
        for i in 0..x.len() - 1 {
            sum += 0.5 * (y[i] + y[i + 1]) * (x[i + 1] - x[i]);
        }
        sum
    }

    /// Generate a Gaussian flash profile for simulation.
    ///
    /// Returns (times, intensities) centered at `t_center` with given FWHM.
    pub fn generate_gaussian_flash(
        t_center: f64,
        fwhm: f64,
        amplitude: f64,
        n_points: usize,
        window_fwhm_multiples: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let sigma = fwhm / (2.0 * (2.0 * 2.0_f64.ln()).sqrt());
        let half_window = window_fwhm_multiples * fwhm;
        let dt = 2.0 * half_window / (n_points - 1) as f64;

        let mut times = Vec::with_capacity(n_points);
        let mut intensities = Vec::with_capacity(n_points);

        for i in 0..n_points {
            let t = t_center - half_window + i as f64 * dt;
            let val = amplitude * (-0.5 * ((t - t_center) / sigma).powi(2)).exp();
            times.push(t);
            intensities.push(val);
        }

        (times, intensities)
    }
}

// ---------------------------------------------------------------------------
// Acoustic Pressure Calibrator
// ---------------------------------------------------------------------------

/// Acoustic pressure calibrator: convert hydrophone voltage to pressure.
pub struct AcousticPressureCalibrator {
    /// Hydrophone sensitivity (V/Pa). Typical: ~1e-6 to ~1e-4 V/Pa.
    sensitivity_v_per_pa: f64,
    /// Preamplifier gain (linear, dimensionless).
    preamp_gain: f64,
}

impl AcousticPressureCalibrator {
    /// Create a new calibrator.
    ///
    /// `sensitivity_v_per_pa`: hydrophone receive sensitivity (V/Pa).
    /// `preamp_gain`: preamplifier gain (linear ratio, 1.0 = no gain).
    pub fn new(sensitivity_v_per_pa: f64, preamp_gain: f64) -> Self {
        Self {
            sensitivity_v_per_pa,
            preamp_gain,
        }
    }

    /// Convert voltage signal to pressure (Pa).
    ///
    /// P = V / (M · G), where M = sensitivity, G = preamp gain.
    pub fn voltage_to_pressure(&self, voltage: f64) -> f64 {
        voltage / (self.sensitivity_v_per_pa * self.preamp_gain)
    }

    /// Convert pressure to voltage (inverse calibration).
    pub fn pressure_to_voltage(&self, pressure_pa: f64) -> f64 {
        pressure_pa * self.sensitivity_v_per_pa * self.preamp_gain
    }

    /// Calibrate a signal buffer (voltage → pressure).
    pub fn calibrate_signal(&self, voltages: &[f64]) -> Vec<f64> {
        voltages
            .iter()
            .map(|&v| self.voltage_to_pressure(v))
            .collect()
    }

    /// Compute RMS pressure from voltage signal.
    pub fn rms_pressure(&self, voltages: &[f64]) -> f64 {
        let rms_v = (voltages.iter().map(|v| v * v).sum::<f64>() / voltages.len() as f64).sqrt();
        self.voltage_to_pressure(rms_v)
    }

    /// Convert RMS pressure to sound pressure level (dB re 1 µPa).
    pub fn pressure_to_spl_db(pressure_pa: f64) -> f64 {
        let p_ref = 1e-6; // 1 µPa reference for underwater acoustics
        20.0 * (pressure_pa / p_ref).log10()
    }
}

// ---------------------------------------------------------------------------
// Bubble Stability Analyzer
// ---------------------------------------------------------------------------

/// Result of bubble stability analysis.
#[derive(Debug, Clone)]
pub struct StabilityResult {
    /// Blake threshold pressure (Pa).
    pub blake_threshold_pa: f64,
    /// Whether the bubble is above the Blake threshold (will grow unboundedly).
    pub above_blake_threshold: bool,
    /// Resonance frequency of the bubble (Hz) — Minnaert frequency.
    pub resonance_freq_hz: f64,
    /// Ratio of driving frequency to resonance frequency.
    pub frequency_ratio: f64,
    /// Whether in the stable SL parameter region (heuristic).
    pub in_stable_sl_region: bool,
}

/// Analyze bubble stability conditions and parameter space.
pub struct BubbleStabilityAnalyzer;

impl BubbleStabilityAnalyzer {
    /// Blake threshold: minimum acoustic pressure for unbounded bubble growth.
    ///
    /// P_B = P_static + (4/3) · σ / (R0 · √3)
    ///
    /// More precisely for vapor-free case:
    /// P_B = P0 + (4σ/(3R0)) · (4σ / (3·P0·R0))^(1/2) ... simplified here.
    pub fn blake_threshold(config: &SonoluminescenceConfig) -> f64 {
        let p0 = config.ambient_pressure_pa;
        let sigma = config.surface_tension;
        let r0 = config.equilibrium_radius_m;

        // Blake threshold: P_B = P0 + (4/3) · σ/(R0 · √3)
        // More complete form including gas pressure inside:
        // P_Blake = P0 + (4*sigma) / (3 * R0 * sqrt(3))
        // For the critical case with surface tension:
        p0 + 4.0 * sigma / (3.0 * r0 * 3.0_f64.sqrt())
    }

    /// Minnaert resonance frequency for a bubble of radius R0.
    ///
    /// f_M = (1/(2πR0)) · √(3γP0/ρ)
    pub fn minnaert_frequency(config: &SonoluminescenceConfig) -> f64 {
        let gamma = config.gas_type.gamma();
        let p0 = config.ambient_pressure_pa;
        let rho = config.liquid_density;
        let r0 = config.equilibrium_radius_m;

        (1.0 / (2.0 * PI * r0)) * (3.0 * gamma * p0 / rho).sqrt()
    }

    /// Full stability analysis.
    pub fn analyze(config: &SonoluminescenceConfig) -> StabilityResult {
        let blake = Self::blake_threshold(config);
        let minnaert = Self::minnaert_frequency(config);
        let freq_ratio = config.driving_freq_hz / minnaert;

        // Heuristic for stable SL region:
        // - Driving pressure above Blake threshold (cavitation occurs)
        // - But not too far above (violent multi-bubble regime)
        // - Driving frequency well below resonance (parametric regime)
        let above_blake = config.pressure_amplitude_pa > blake;
        let pressure_ratio = config.pressure_amplitude_pa / P_ATM;
        let in_stable = above_blake && pressure_ratio < 1.6 && freq_ratio < 0.5;

        StabilityResult {
            blake_threshold_pa: blake,
            above_blake_threshold: above_blake,
            resonance_freq_hz: minnaert,
            frequency_ratio: freq_ratio,
            in_stable_sl_region: in_stable,
        }
    }
}

// ---------------------------------------------------------------------------
// Multi-Bubble Analyzer (MBSL)
// ---------------------------------------------------------------------------

/// A detected multi-bubble event.
#[derive(Debug, Clone, Copy)]
pub struct MultibubbleEvent {
    /// Index in the signal.
    pub index: usize,
    /// Time (s).
    pub time: f64,
    /// Peak amplitude.
    pub peak_amplitude: f64,
}

/// Analyze multi-bubble sonoluminescence (MBSL) signals.
pub struct MultibubbleAnalyzer {
    /// Detection threshold (fraction of max amplitude).
    threshold_fraction: f64,
    /// Minimum separation between events (samples).
    min_separation: usize,
}

impl MultibubbleAnalyzer {
    /// Create a new multi-bubble analyzer.
    ///
    /// `threshold_fraction`: detection threshold as fraction of signal max (0 to 1).
    /// `min_separation`: minimum separation between events in samples.
    pub fn new(threshold_fraction: f64, min_separation: usize) -> Self {
        Self {
            threshold_fraction,
            min_separation,
        }
    }

    /// Detect bubble events in a time series of light intensity.
    ///
    /// `times_s`: time values.
    /// `intensities`: measured intensity values.
    pub fn detect_events(
        &self,
        times_s: &[f64],
        intensities: &[f64],
    ) -> Vec<MultibubbleEvent> {
        assert_eq!(times_s.len(), intensities.len());
        if intensities.is_empty() {
            return vec![];
        }

        let max_val = intensities
            .iter()
            .cloned()
            .fold(f64::MIN, f64::max);
        let threshold = self.threshold_fraction * max_val;

        let mut events = Vec::new();
        let mut last_event_idx: Option<usize> = None;

        for i in 1..intensities.len() - 1 {
            // Local maximum above threshold
            if intensities[i] > threshold
                && intensities[i] >= intensities[i - 1]
                && intensities[i] >= intensities[i + 1]
            {
                // Check minimum separation
                if let Some(last) = last_event_idx {
                    if i - last < self.min_separation {
                        continue;
                    }
                }
                events.push(MultibubbleEvent {
                    index: i,
                    time: times_s[i],
                    peak_amplitude: intensities[i],
                });
                last_event_idx = Some(i);
            }
        }

        events
    }

    /// Compute event rate (events per second).
    pub fn event_rate(&self, events: &[MultibubbleEvent]) -> f64 {
        if events.len() < 2 {
            return 0.0;
        }
        let duration = events.last().unwrap().time - events.first().unwrap().time;
        if duration <= 0.0 {
            return 0.0;
        }
        (events.len() - 1) as f64 / duration
    }

    /// Amplitude distribution statistics: (mean, std_dev, min, max).
    pub fn amplitude_stats(events: &[MultibubbleEvent]) -> (f64, f64, f64, f64) {
        if events.is_empty() {
            return (0.0, 0.0, 0.0, 0.0);
        }
        let amps: Vec<f64> = events.iter().map(|e| e.peak_amplitude).collect();
        let n = amps.len() as f64;
        let mean = amps.iter().sum::<f64>() / n;
        let var = amps.iter().map(|a| (a - mean).powi(2)).sum::<f64>() / n;
        let min = amps.iter().cloned().fold(f64::MAX, f64::min);
        let max = amps.iter().cloned().fold(f64::MIN, f64::max);
        (mean, var.sqrt(), min, max)
    }

    /// Inter-event interval histogram.
    ///
    /// Returns `(bin_edges, counts)` with `n_bins` bins.
    pub fn interval_histogram(
        events: &[MultibubbleEvent],
        n_bins: usize,
    ) -> (Vec<f64>, Vec<usize>) {
        if events.len() < 2 || n_bins == 0 {
            return (vec![], vec![]);
        }

        let intervals: Vec<f64> = events
            .windows(2)
            .map(|w| w[1].time - w[0].time)
            .collect();

        let min_i = intervals.iter().cloned().fold(f64::MAX, f64::min);
        let max_i = intervals.iter().cloned().fold(f64::MIN, f64::max);
        let range = max_i - min_i;

        if range <= 0.0 {
            return (vec![min_i, max_i], vec![intervals.len()]);
        }

        let bin_width = range / n_bins as f64;
        let mut counts = vec![0usize; n_bins];
        let mut edges = Vec::with_capacity(n_bins + 1);

        for i in 0..=n_bins {
            edges.push(min_i + i as f64 * bin_width);
        }

        for &ival in &intervals {
            let bin = ((ival - min_i) / bin_width).floor() as usize;
            let bin = bin.min(n_bins - 1);
            counts[bin] += 1;
        }

        (edges, counts)
    }
}

// ---------------------------------------------------------------------------
// Cavitation Threshold Detector
// ---------------------------------------------------------------------------

/// Result of cavitation detection.
#[derive(Debug, Clone)]
pub struct CavitationDetectionResult {
    /// Whether cavitation was detected.
    pub cavitation_detected: bool,
    /// Broadband noise level increase (dB above baseline).
    pub broadband_noise_db: f64,
    /// Subharmonic level (dB) at f_drive/2.
    pub subharmonic_level_db: f64,
    /// Ultraharmonic level (dB) at 3·f_drive/2.
    pub ultraharmonic_level_db: f64,
}

/// Detect onset of cavitation from hydrophone spectral data.
///
/// Indicators:
/// - Broadband noise increase (inertial cavitation)
/// - Subharmonic emission at f/2 (stable cavitation)
/// - Ultraharmonic emission at 3f/2, 5f/2, ... (stable cavitation)
pub struct CavitationThresholdDetector {
    /// Driving frequency (Hz).
    driving_freq_hz: f64,
    /// Broadband noise threshold (dB above baseline).
    broadband_threshold_db: f64,
    /// Subharmonic threshold (dB above noise floor).
    subharmonic_threshold_db: f64,
}

impl CavitationThresholdDetector {
    /// Create a new cavitation detector.
    ///
    /// `driving_freq_hz`: acoustic driving frequency.
    /// `broadband_threshold_db`: threshold for broadband noise detection.
    /// `subharmonic_threshold_db`: threshold for subharmonic detection.
    pub fn new(
        driving_freq_hz: f64,
        broadband_threshold_db: f64,
        subharmonic_threshold_db: f64,
    ) -> Self {
        Self {
            driving_freq_hz,
            broadband_threshold_db,
            subharmonic_threshold_db,
        }
    }

    /// Detect cavitation from a power spectrum.
    ///
    /// `freq_bins`: frequency values (Hz).
    /// `power_db`: power spectral density in dB.
    /// `baseline_db`: baseline (no-cavitation) power spectrum in dB.
    pub fn detect(
        &self,
        freq_bins: &[f64],
        power_db: &[f64],
        baseline_db: &[f64],
    ) -> CavitationDetectionResult {
        assert_eq!(freq_bins.len(), power_db.len());
        assert_eq!(freq_bins.len(), baseline_db.len());

        // Broadband noise: average power increase excluding harmonics
        let f_drive = self.driving_freq_hz;
        let mut broadband_sum = 0.0;
        let mut broadband_count = 0;

        for i in 0..freq_bins.len() {
            let f = freq_bins[i];
            // Skip bins near harmonics of driving frequency
            let nearest_harmonic = (f / f_drive).round() * f_drive;
            if (f - nearest_harmonic).abs() > f_drive * 0.1 {
                broadband_sum += power_db[i] - baseline_db[i];
                broadband_count += 1;
            }
        }

        let broadband_noise_db = if broadband_count > 0 {
            broadband_sum / broadband_count as f64
        } else {
            0.0
        };

        // Subharmonic at f/2
        let subharmonic_level = Self::level_at_frequency(
            freq_bins,
            power_db,
            baseline_db,
            f_drive / 2.0,
        );

        // Ultraharmonic at 3f/2
        let ultraharmonic_level = Self::level_at_frequency(
            freq_bins,
            power_db,
            baseline_db,
            1.5 * f_drive,
        );

        let cavitation_detected = broadband_noise_db > self.broadband_threshold_db
            || subharmonic_level > self.subharmonic_threshold_db;

        CavitationDetectionResult {
            cavitation_detected,
            broadband_noise_db,
            subharmonic_level_db: subharmonic_level,
            ultraharmonic_level_db: ultraharmonic_level,
        }
    }

    /// Find the power level at a specific frequency relative to baseline.
    fn level_at_frequency(
        freq_bins: &[f64],
        power_db: &[f64],
        baseline_db: &[f64],
        target_freq: f64,
    ) -> f64 {
        // Find nearest bin
        let idx = freq_bins
            .iter()
            .enumerate()
            .min_by(|a, b| {
                (a.1 - target_freq)
                    .abs()
                    .partial_cmp(&(b.1 - target_freq).abs())
                    .unwrap()
            })
            .map(|(i, _)| i)
            .unwrap_or(0);

        power_db[idx] - baseline_db[idx]
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // --- Configuration tests ---

    #[test]
    fn test_config_sbsl_defaults() {
        let c = SonoluminescenceConfig::sbsl_water_25khz();
        assert_eq!(c.driving_freq_hz, 25_000.0);
        assert!((c.pressure_amplitude_pa - 1.3 * P_ATM).abs() < 1.0);
        assert_eq!(c.equilibrium_radius_m, 4.5e-6);
        assert_eq!(c.liquid_density, 998.0);
    }

    #[test]
    fn test_config_mbsl_defaults() {
        let c = SonoluminescenceConfig::mbsl_water_20khz();
        assert_eq!(c.driving_freq_hz, 20_000.0);
        assert!((c.pressure_amplitude_pa - 1.5 * P_ATM).abs() < 1.0);
        assert!(matches!(c.gas_type, GasType::Air));
    }

    #[test]
    fn test_config_omega() {
        let c = SonoluminescenceConfig::sbsl_water_25khz();
        let expected = 2.0 * PI * 25_000.0;
        assert!((c.omega() - expected).abs() < 1e-6);
    }

    #[test]
    fn test_config_acoustic_period() {
        let c = SonoluminescenceConfig::sbsl_water_25khz();
        assert!((c.acoustic_period_s() - 4e-5).abs() < 1e-10);
    }

    #[test]
    fn test_config_initial_gas_pressure() {
        let c = SonoluminescenceConfig::sbsl_water_25khz();
        let p_gas = c.initial_gas_pressure();
        // P_gas = P0 + 2σ/R0 = 101325 + 2*0.0728/4.5e-6 ≈ 101325 + 32356 ≈ 133681
        assert!(p_gas > P_ATM);
        assert!(p_gas < 2.0 * P_ATM);
    }

    // --- Gas type tests ---

    #[test]
    fn test_gas_type_gamma() {
        assert!((GasType::Argon.gamma() - 5.0 / 3.0).abs() < 1e-10);
        assert!((GasType::Air.gamma() - 1.4).abs() < 1e-10);
        assert!((GasType::Helium.gamma() - 5.0 / 3.0).abs() < 1e-10);
        assert!((GasType::Custom(1.3).gamma() - 1.3).abs() < 1e-10);
    }

    // --- Rayleigh-Plesset solver tests ---

    #[test]
    fn test_solver_initial_state() {
        let config = SonoluminescenceConfig::sbsl_water_25khz();
        let solver = RayleighPlessetSolver::new(&config);
        let state = solver.state();
        assert!((state.radius - config.equilibrium_radius_m).abs() < 1e-15);
        assert!((state.velocity).abs() < 1e-15);
        assert!((state.time).abs() < 1e-15);
    }

    #[test]
    fn test_solver_step_changes_state() {
        let config = SonoluminescenceConfig::sbsl_water_25khz();
        let mut solver = RayleighPlessetSolver::new(&config);
        let initial = solver.state();
        solver.step(1e-9);
        let after = solver.state();
        // Time should advance
        assert!(after.time > initial.time);
        // State should change (acoustic driving is present)
        assert!(after.radius != initial.radius || after.velocity != initial.velocity);
    }

    #[test]
    fn test_solver_reset() {
        let config = SonoluminescenceConfig::sbsl_water_25khz();
        let mut solver = RayleighPlessetSolver::new(&config);
        solver.step(1e-9);
        solver.step(1e-9);
        solver.reset();
        let state = solver.state();
        assert!((state.radius - config.equilibrium_radius_m).abs() < 1e-15);
        assert!((state.time).abs() < 1e-15);
    }

    #[test]
    fn test_solver_simulate_returns_correct_length() {
        let config = SonoluminescenceConfig::sbsl_water_25khz();
        let mut solver = RayleighPlessetSolver::new(&config);
        let traj = solver.simulate(100, 1e-9);
        assert_eq!(traj.len(), 101); // initial + 100 steps
    }

    #[test]
    fn test_solver_bubble_expands_and_collapses() {
        // Over one acoustic cycle, the bubble should expand and collapse
        let config = SonoluminescenceConfig::sbsl_water_25khz();
        let mut solver = RayleighPlessetSolver::new(&config);
        let dt = 1e-8; // 10 ns
        let n_steps = (config.acoustic_period_s() / dt) as usize;
        let traj = solver.simulate(n_steps, dt);

        let r0 = config.equilibrium_radius_m;
        let max_r = traj.iter().map(|p| p.radius).fold(f64::MIN, f64::max);
        let min_r = traj.iter().map(|p| p.radius).fold(f64::MAX, f64::min);

        // Bubble should expand beyond equilibrium
        assert!(max_r > r0, "Max radius {} should exceed R0 {}", max_r, r0);
        // Bubble should compress below equilibrium
        assert!(min_r < r0, "Min radius {} should be less than R0 {}", min_r, r0);
    }

    #[test]
    fn test_solver_compression_ratio() {
        let config = SonoluminescenceConfig::sbsl_water_25khz();
        let mut solver = RayleighPlessetSolver::new(&config);
        let dt = 1e-8;
        let n_steps = (config.acoustic_period_s() / dt) as usize;
        let traj = solver.simulate(n_steps, dt);

        // Compression ratio should be > 1 at collapse
        let max_cr = traj
            .iter()
            .map(|p| p.compression_ratio)
            .fold(f64::MIN, f64::max);
        assert!(max_cr > 1.0);
    }

    // --- BubbleRadiusTracker tests ---

    #[test]
    fn test_radius_tracker_analyze() {
        let config = SonoluminescenceConfig::sbsl_water_25khz();
        let mut solver = RayleighPlessetSolver::new(&config);
        let dt = 1e-8;
        let n_steps = (config.acoustic_period_s() / dt) as usize;
        let traj = solver.simulate(n_steps, dt);

        let tracker = BubbleRadiusTracker::new(&config);
        let stats = tracker.analyze(&traj);

        assert!(stats.max_radius > config.equilibrium_radius_m);
        assert!(stats.min_radius < config.equilibrium_radius_m);
        assert!(stats.max_compression_ratio > 1.0);
        assert!(stats.max_collapse_speed > 0.0);
    }

    #[test]
    fn test_radius_tracker_collapse_indices() {
        let config = SonoluminescenceConfig::sbsl_water_25khz();
        let mut solver = RayleighPlessetSolver::new(&config);
        let dt = 1e-8;
        // Simulate two cycles
        let n_steps = (2.0 * config.acoustic_period_s() / dt) as usize;
        let traj = solver.simulate(n_steps, dt);

        let tracker = BubbleRadiusTracker::new(&config);
        let collapses = tracker.find_collapse_indices(&traj);

        // Should find at least one collapse per cycle
        assert!(!collapses.is_empty(), "Should detect at least one collapse");
    }

    #[test]
    fn test_radius_tracker_mach_number() {
        let config = SonoluminescenceConfig::sbsl_water_25khz();
        let mut solver = RayleighPlessetSolver::new(&config);
        let dt = 1e-8;
        let n_steps = (config.acoustic_period_s() / dt) as usize;
        let traj = solver.simulate(n_steps, dt);

        let tracker = BubbleRadiusTracker::new(&config);
        let stats = tracker.analyze(&traj);

        // Mach number should be positive
        assert!(stats.collapse_mach_number > 0.0);
        // Should be speed/c_sound
        assert!(
            (stats.collapse_mach_number - stats.max_collapse_speed / config.sound_speed).abs()
                < 1e-10
        );
    }

    // --- Emission Timing tests ---

    #[test]
    fn test_emission_timing_find_collapses() {
        let config = SonoluminescenceConfig::sbsl_water_25khz();
        let mut solver = RayleighPlessetSolver::new(&config);
        let dt = 1e-8;
        let n_steps = (2.0 * config.acoustic_period_s() / dt) as usize;
        let traj = solver.simulate(n_steps, dt);

        let analyzer = EmissionTimingAnalyzer::new(config.driving_freq_hz);
        let events = analyzer.find_collapses(&traj);

        assert!(!events.is_empty());
        // Phase should be in [0, 2π)
        for e in &events {
            assert!(e.acoustic_phase >= 0.0);
            assert!(e.acoustic_phase < 2.0 * PI + 0.01);
        }
    }

    #[test]
    fn test_emission_timing_jitter() {
        // Create fake events with consistent phase
        let events = vec![
            CollapseEvent {
                index: 0,
                time: 0.0,
                min_radius: 1e-7,
                acoustic_phase: 3.0,
                collapse_speed: 100.0,
            },
            CollapseEvent {
                index: 1,
                time: 4e-5,
                min_radius: 1e-7,
                acoustic_phase: 3.01,
                collapse_speed: 100.0,
            },
            CollapseEvent {
                index: 2,
                time: 8e-5,
                min_radius: 1e-7,
                acoustic_phase: 2.99,
                collapse_speed: 100.0,
            },
        ];

        let analyzer = EmissionTimingAnalyzer::new(25_000.0);
        let (mean, std) = analyzer.timing_jitter(&events);

        // Mean should be near 3.0 rad
        assert!((mean - 3.0).abs() < 0.1);
        // Jitter should be small
        assert!(std < 0.1);
    }

    #[test]
    fn test_emission_timing_inter_collapse_stats() {
        let events = vec![
            CollapseEvent {
                index: 0,
                time: 0.0,
                min_radius: 1e-7,
                acoustic_phase: 3.0,
                collapse_speed: 100.0,
            },
            CollapseEvent {
                index: 1,
                time: 4e-5,
                min_radius: 1e-7,
                acoustic_phase: 3.0,
                collapse_speed: 100.0,
            },
            CollapseEvent {
                index: 2,
                time: 8e-5,
                min_radius: 1e-7,
                acoustic_phase: 3.0,
                collapse_speed: 100.0,
            },
        ];

        let analyzer = EmissionTimingAnalyzer::new(25_000.0);
        let (mean, std) = analyzer.inter_collapse_stats(&events);

        assert!((mean - 4e-5).abs() < 1e-10);
        assert!(std < 1e-10); // uniform intervals => zero std
    }

    // --- Spectrum Analyzer tests ---

    #[test]
    fn test_planck_radiance_positive() {
        let val = SpectrumAnalyzer::planck_spectral_radiance(500e-9, 10_000.0);
        assert!(val > 0.0);
    }

    #[test]
    fn test_planck_radiance_increases_with_temperature() {
        let lam = 400e-9; // 400 nm (visible)
        let i1 = SpectrumAnalyzer::planck_spectral_radiance(lam, 5_000.0);
        let i2 = SpectrumAnalyzer::planck_spectral_radiance(lam, 10_000.0);
        assert!(i2 > i1, "Higher T should give higher visible-light intensity");
    }

    #[test]
    fn test_wien_peak_wavelength() {
        // Sun at ~5778 K => peak at ~502 nm
        let peak = SpectrumAnalyzer::wien_peak_wavelength(5778.0);
        assert!((peak - 501.5e-9).abs() < 5e-9);
    }

    #[test]
    fn test_temperature_from_peak_roundtrip() {
        let t_original = 15_000.0;
        let peak = SpectrumAnalyzer::wien_peak_wavelength(t_original);
        let t_recovered = SpectrumAnalyzer::temperature_from_peak_wavelength(peak);
        assert!((t_recovered - t_original).abs() < 1.0);
    }

    #[test]
    fn test_blackbody_fit() {
        // Generate synthetic spectrum at 20000 K and recover the temperature
        let temp = 20_000.0;
        let wavelengths: Vec<f64> = (200..=700)
            .step_by(10)
            .map(|nm| nm as f64 * 1e-9)
            .collect();
        let intensities = SpectrumAnalyzer::generate_blackbody_spectrum(temp, &wavelengths);

        let result =
            SpectrumAnalyzer::fit_blackbody(&wavelengths, &intensities, (5_000.0, 40_000.0), 1000);

        assert!(
            (result.temperature_k - temp).abs() < 200.0,
            "Fit T={} should be near true T={}",
            result.temperature_k,
            temp
        );
    }

    #[test]
    fn test_planck_radiance_very_high_exponent() {
        // Very long wavelength, low T => huge exponent, should return 0 gracefully
        let val = SpectrumAnalyzer::planck_spectral_radiance(10e-6, 10.0);
        assert!(val.is_finite());
    }

    // --- Flash Duration tests ---

    #[test]
    fn test_flash_duration_gaussian() {
        let fwhm = 100e-12; // 100 ps
        let (times, intensities) =
            FlashDurationMeasurer::generate_gaussian_flash(0.0, fwhm, 1.0, 1001, 5.0);

        let result = FlashDurationMeasurer::analyze(&times, &intensities);

        // FWHM should be close to 100 ps
        assert!(
            (result.fwhm_s - fwhm).abs() < fwhm * 0.1,
            "FWHM={:.2e} should be near {:.2e}",
            result.fwhm_s,
            fwhm
        );
        // Peak should be near 1.0
        assert!((result.peak_intensity - 1.0).abs() < 0.01);
        // Peak time near 0
        assert!(result.peak_time_s.abs() < fwhm);
    }

    #[test]
    fn test_flash_duration_energy() {
        let fwhm = 100e-12;
        let amplitude = 2.0;
        let (times, intensities) =
            FlashDurationMeasurer::generate_gaussian_flash(0.0, fwhm, amplitude, 1001, 5.0);

        let result = FlashDurationMeasurer::analyze(&times, &intensities);

        // Gaussian integral: A * sigma * sqrt(2pi)
        // sigma = fwhm / (2*sqrt(2*ln2))
        let sigma = fwhm / (2.0 * (2.0 * 2.0_f64.ln()).sqrt());
        let expected_energy = amplitude * sigma * (2.0 * PI).sqrt();
        assert!(
            (result.total_energy - expected_energy).abs() / expected_energy < 0.05,
            "Energy={:.2e} should be near {:.2e}",
            result.total_energy,
            expected_energy
        );
    }

    #[test]
    fn test_flash_rise_fall_time() {
        let fwhm = 100e-12;
        let (times, intensities) =
            FlashDurationMeasurer::generate_gaussian_flash(0.0, fwhm, 1.0, 2001, 5.0);

        let result = FlashDurationMeasurer::analyze(&times, &intensities);

        // For Gaussian, rise and fall should be symmetric and roughly equal
        assert!(result.rise_time_s > 0.0);
        assert!(result.fall_time_s > 0.0);
        assert!(
            (result.rise_time_s - result.fall_time_s).abs() / result.rise_time_s < 0.1,
            "Rise={:.2e} Fall={:.2e} should be similar for Gaussian",
            result.rise_time_s,
            result.fall_time_s
        );
    }

    // --- Acoustic Pressure Calibrator tests ---

    #[test]
    fn test_calibrator_voltage_to_pressure() {
        let cal = AcousticPressureCalibrator::new(1e-5, 10.0); // 10 µV/Pa, gain 10
        let pressure = cal.voltage_to_pressure(0.01); // 10 mV
        // P = 0.01 / (1e-5 * 10) = 100 Pa
        assert!((pressure - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_calibrator_roundtrip() {
        let cal = AcousticPressureCalibrator::new(5e-5, 20.0);
        let p_orig = 50_000.0;
        let v = cal.pressure_to_voltage(p_orig);
        let p_back = cal.voltage_to_pressure(v);
        assert!((p_back - p_orig).abs() < 1e-6);
    }

    #[test]
    fn test_calibrator_rms_pressure() {
        let cal = AcousticPressureCalibrator::new(1e-5, 1.0);
        // Sinusoidal voltage with amplitude 0.01 V => RMS = 0.01/sqrt(2)
        let n = 10_000;
        let voltages: Vec<f64> = (0..n)
            .map(|i| 0.01 * (2.0 * PI * i as f64 / n as f64).sin())
            .collect();
        let rms_p = cal.rms_pressure(&voltages);
        let expected_rms_v = 0.01 / 2.0_f64.sqrt();
        let expected_rms_p = expected_rms_v / 1e-5;
        assert!(
            (rms_p - expected_rms_p).abs() / expected_rms_p < 0.01,
            "RMS pressure={:.1} should be near {:.1}",
            rms_p,
            expected_rms_p
        );
    }

    #[test]
    fn test_spl_conversion() {
        // 1 Pa => 120 dB re 1 µPa
        let spl = AcousticPressureCalibrator::pressure_to_spl_db(1.0);
        assert!((spl - 120.0).abs() < 0.1);
    }

    // --- Bubble Stability tests ---

    #[test]
    fn test_blake_threshold() {
        let config = SonoluminescenceConfig::sbsl_water_25khz();
        let blake = BubbleStabilityAnalyzer::blake_threshold(&config);
        // Blake threshold should be somewhat above ambient
        assert!(blake > P_ATM);
    }

    #[test]
    fn test_minnaert_frequency() {
        let config = SonoluminescenceConfig::sbsl_water_25khz();
        let f_m = BubbleStabilityAnalyzer::minnaert_frequency(&config);
        // For R0 = 4.5 µm in water, resonance should be in the MHz range
        assert!(f_m > 100_000.0, "Minnaert freq={:.0} Hz should be > 100 kHz", f_m);
    }

    #[test]
    fn test_stability_analysis() {
        let config = SonoluminescenceConfig::sbsl_water_25khz();
        let result = BubbleStabilityAnalyzer::analyze(&config);
        // Driving freq should be well below resonance for SBSL
        assert!(result.frequency_ratio < 1.0);
        assert!(result.resonance_freq_hz > config.driving_freq_hz);
    }

    // --- MultibubbleAnalyzer tests ---

    #[test]
    fn test_multibubble_detect_events() {
        let n = 1000;
        let dt = 1e-6;
        let times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        // Create signal with 3 peaks
        let mut intensities = vec![0.0; n];
        for &peak_idx in &[200, 500, 800] {
            for j in 0..20 {
                let idx = peak_idx - 10 + j;
                if idx < n {
                    let x = (j as f64 - 10.0) / 5.0;
                    intensities[idx] = 1.0 * (-0.5 * x * x).exp();
                }
            }
        }

        let analyzer = MultibubbleAnalyzer::new(0.3, 50);
        let events = analyzer.detect_events(&times, &intensities);

        assert_eq!(events.len(), 3, "Should detect 3 events");
    }

    #[test]
    fn test_multibubble_event_rate() {
        let events = vec![
            MultibubbleEvent { index: 0, time: 0.0, peak_amplitude: 1.0 },
            MultibubbleEvent { index: 1, time: 1e-4, peak_amplitude: 1.0 },
            MultibubbleEvent { index: 2, time: 2e-4, peak_amplitude: 1.0 },
            MultibubbleEvent { index: 3, time: 3e-4, peak_amplitude: 1.0 },
        ];

        let analyzer = MultibubbleAnalyzer::new(0.5, 10);
        let rate = analyzer.event_rate(&events);

        // 3 intervals in 3e-4 s => 10000 events/s
        assert!((rate - 10_000.0).abs() < 1.0);
    }

    #[test]
    fn test_multibubble_amplitude_stats() {
        let events = vec![
            MultibubbleEvent { index: 0, time: 0.0, peak_amplitude: 1.0 },
            MultibubbleEvent { index: 1, time: 1e-4, peak_amplitude: 2.0 },
            MultibubbleEvent { index: 2, time: 2e-4, peak_amplitude: 3.0 },
        ];
        let (mean, std, min, max) = MultibubbleAnalyzer::amplitude_stats(&events);
        assert!((mean - 2.0).abs() < 1e-10);
        assert!((min - 1.0).abs() < 1e-10);
        assert!((max - 3.0).abs() < 1e-10);
        assert!(std > 0.0);
    }

    #[test]
    fn test_multibubble_interval_histogram() {
        let events = vec![
            MultibubbleEvent { index: 0, time: 0.0, peak_amplitude: 1.0 },
            MultibubbleEvent { index: 1, time: 1e-4, peak_amplitude: 1.0 },
            MultibubbleEvent { index: 2, time: 2e-4, peak_amplitude: 1.0 },
            MultibubbleEvent { index: 3, time: 3e-4, peak_amplitude: 1.0 },
        ];

        let (edges, counts) = MultibubbleAnalyzer::interval_histogram(&events, 5);
        assert_eq!(edges.len(), 6); // n_bins + 1
        assert_eq!(counts.len(), 5);
        let total: usize = counts.iter().sum();
        assert_eq!(total, 3); // 3 intervals
    }

    // --- Cavitation Threshold Detector tests ---

    #[test]
    fn test_cavitation_detector_no_cavitation() {
        let detector = CavitationThresholdDetector::new(25_000.0, 3.0, 6.0);

        let freq_bins: Vec<f64> = (0..100).map(|i| i as f64 * 500.0).collect();
        let power_db = vec![-80.0; 100];
        let baseline_db = vec![-80.0; 100];

        let result = detector.detect(&freq_bins, &power_db, &baseline_db);
        assert!(!result.cavitation_detected);
    }

    #[test]
    fn test_cavitation_detector_broadband_noise() {
        let detector = CavitationThresholdDetector::new(25_000.0, 3.0, 6.0);

        let freq_bins: Vec<f64> = (0..100).map(|i| i as f64 * 500.0).collect();
        let baseline_db = vec![-80.0; 100];
        // Elevated broadband noise
        let power_db = vec![-75.0; 100];

        let result = detector.detect(&freq_bins, &power_db, &baseline_db);
        assert!(result.cavitation_detected);
        assert!(result.broadband_noise_db > 3.0);
    }

    #[test]
    fn test_cavitation_detector_subharmonic() {
        let detector = CavitationThresholdDetector::new(25_000.0, 3.0, 6.0);
        let df = 500.0;
        let n_bins = 100;
        let freq_bins: Vec<f64> = (0..n_bins).map(|i| i as f64 * df).collect();
        let baseline_db = vec![-80.0; n_bins];
        let mut power_db = vec![-80.0; n_bins];

        // Add strong subharmonic at 12500 Hz (bin 25)
        let sub_bin = (12_500.0 / df).round() as usize;
        power_db[sub_bin] = -70.0; // 10 dB above baseline

        let result = detector.detect(&freq_bins, &power_db, &baseline_db);
        assert!(result.subharmonic_level_db > 6.0);
    }

    #[test]
    fn test_cavitation_detector_ultraharmonic() {
        let detector = CavitationThresholdDetector::new(25_000.0, 3.0, 6.0);
        let df = 500.0;
        let n_bins = 100;
        let freq_bins: Vec<f64> = (0..n_bins).map(|i| i as f64 * df).collect();
        let baseline_db = vec![-80.0; n_bins];
        let mut power_db = vec![-80.0; n_bins];

        // Add ultraharmonic at 37500 Hz (3f/2)
        let ultra_bin = (37_500.0 / df).round() as usize;
        if ultra_bin < n_bins {
            power_db[ultra_bin] = -65.0;
        }

        let result = detector.detect(&freq_bins, &power_db, &baseline_db);
        assert!(result.ultraharmonic_level_db > 0.0);
    }

    // --- Integration / end-to-end test ---

    #[test]
    fn test_full_sbsl_simulation_pipeline() {
        // End-to-end: config -> solver -> tracker -> timing -> stability
        let config = SonoluminescenceConfig::sbsl_water_25khz();
        let mut solver = RayleighPlessetSolver::new(&config);

        let dt = 1e-8;
        let n_cycles = 2;
        let n_steps = (n_cycles as f64 * config.acoustic_period_s() / dt) as usize;
        let trajectory = solver.simulate(n_steps, dt);

        // Tracker
        let tracker = BubbleRadiusTracker::new(&config);
        let stats = tracker.analyze(&trajectory);
        assert!(stats.max_compression_ratio > 1.0);

        // Timing
        let timing = EmissionTimingAnalyzer::new(config.driving_freq_hz);
        let collapses = timing.find_collapses(&trajectory);
        assert!(!collapses.is_empty());

        // Stability
        let stability = BubbleStabilityAnalyzer::analyze(&config);
        assert!(stability.resonance_freq_hz > 0.0);
    }

    #[test]
    fn test_calibrator_signal_batch() {
        let cal = AcousticPressureCalibrator::new(1e-5, 1.0);
        let voltages = vec![0.001, 0.002, 0.003, 0.004];
        let pressures = cal.calibrate_signal(&voltages);
        assert_eq!(pressures.len(), 4);
        assert!((pressures[0] - 100.0).abs() < 1e-6);
        assert!((pressures[3] - 400.0).abs() < 1e-6);
    }
}

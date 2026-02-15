//! # Acoustic Levitation Controller
//!
//! Ultrasonic standing wave control for acoustic levitation. Acoustic levitation
//! uses high-intensity ultrasound (typically 40 kHz) to create standing wave
//! pressure nodes where small objects can be suspended against gravity.
//!
//! ## Background
//!
//! When an ultrasonic transducer faces a flat reflector, a standing wave forms
//! between them. At pressure nodes (where acoustic pressure is zero), small
//! particles experience a restoring force that can counteract gravity. The
//! Gor'kov potential describes the acoustic radiation force on a small sphere
//! in a standing wave field.
//!
//! ## Applications
//!
//! - **Containerless processing**: Melt and solidify materials without container contamination
//! - **Materials science**: Study crystal growth and phase transitions in microgravity-like conditions
//! - **Pharmaceutical research**: Evaporate solvent from drug droplets while suspended
//! - **Phased array levitation**: Dynamically move and manipulate levitated objects
//!
//! ## Key Equations
//!
//! - Wavelength: λ = c / f
//! - Wavenumber: k = 2πf / c
//! - Standing wave pressure: p(z) = P₀ sin(kz) (with reflector at z = 0)
//! - Node positions: z_n = nλ/2 for n = 1, 2, 3, ...
//! - Gor'kov potential: U = (2π a³ / 3) [f₁ <p²>/(ρc²) − (3/2) f₂ ρ <v²>]
//! - Acoustic radiation force: F = −∇U
//! - Levitation criterion: F_acoustic > m·g

use std::f64::consts::PI;

// ──────────────────────────────────────────────────────────
// Helper functions
// ──────────────────────────────────────────────────────────

/// Compute the wavenumber k = 2πf / c.
///
/// # Arguments
/// * `frequency` - Frequency in Hz
/// * `sound_speed` - Speed of sound in m/s
///
/// # Returns
/// Wavenumber in rad/m
pub fn wavenumber(frequency: f64, sound_speed: f64) -> f64 {
    2.0 * PI * frequency / sound_speed
}

/// Convert Sound Pressure Level (dB re 20 µPa) to pressure in Pascals.
///
/// SPL = 20·log10(p / p_ref), so p = p_ref · 10^(SPL/20).
///
/// # Arguments
/// * `spl_db` - Sound Pressure Level in dB re 20 µPa
///
/// # Returns
/// Pressure in Pascals
pub fn spl_to_pressure(spl_db: f64) -> f64 {
    let p_ref = 20.0e-6; // 20 µPa reference
    p_ref * 10.0_f64.powf(spl_db / 20.0)
}

/// Convert pressure in Pascals to Sound Pressure Level (dB re 20 µPa).
///
/// SPL = 20·log10(p / p_ref).
///
/// # Arguments
/// * `pressure_pa` - Pressure in Pascals
///
/// # Returns
/// SPL in dB re 20 µPa
pub fn pressure_to_spl(pressure_pa: f64) -> f64 {
    let p_ref = 20.0e-6;
    20.0 * (pressure_pa / p_ref).log10()
}

// ──────────────────────────────────────────────────────────
// LevitationConfig
// ──────────────────────────────────────────────────────────

/// Configuration for an acoustic levitation system.
///
/// Describes the transducer array geometry, operating frequency, and medium
/// properties needed to compute the acoustic field.
#[derive(Clone, Debug)]
pub struct LevitationConfig {
    /// Operating frequency in Hz (typically 40,000 Hz for air levitation).
    pub frequency_hz: f64,
    /// Speed of sound in the medium in m/s (default 343.0 for air at 20 °C).
    pub sound_speed_m_s: f64,
    /// Number of transducer elements.
    pub num_transducers: usize,
    /// 3-D positions of each transducer [x, y, z] in metres.
    pub transducer_positions: Vec<[f64; 3]>,
    /// Radius of each transducer element in metres (default 0.005 = 5 mm).
    pub transducer_radius_m: f64,
    /// Density of the propagation medium in kg/m³ (default 1.225 for air at sea level).
    pub medium_density_kg_m3: f64,
}

impl Default for LevitationConfig {
    fn default() -> Self {
        Self {
            frequency_hz: 40_000.0,
            sound_speed_m_s: 343.0,
            num_transducers: 1,
            transducer_positions: vec![[0.0, 0.0, 0.0]],
            transducer_radius_m: 0.005,
            medium_density_kg_m3: 1.225,
        }
    }
}

// ──────────────────────────────────────────────────────────
// LevitationController
// ──────────────────────────────────────────────────────────

/// Core levitation controller that computes acoustic pressure fields,
/// Gor'kov potentials, and acoustic radiation forces for a given
/// transducer array configuration.
pub struct LevitationController {
    config: LevitationConfig,
}

impl LevitationController {
    /// Create a new controller from the given configuration.
    pub fn new(config: LevitationConfig) -> Self {
        Self { config }
    }

    /// Wavelength λ = c / f in metres.
    pub fn wavelength(&self) -> f64 {
        self.config.sound_speed_m_s / self.config.frequency_hz
    }

    /// Compute the complex acoustic pressure at `position` due to superposition
    /// of all transducers with given `amplitudes` and `phases` (radians).
    ///
    /// Each transducer is modelled as a point source radiating spherical waves:
    ///   p_i = A_i / r_i · exp(j(kr_i + φ_i))
    ///
    /// Returns (real part, imaginary part) of the total complex pressure.
    pub fn pressure_field(
        &self,
        position: [f64; 3],
        amplitudes: &[f64],
        phases: &[f64],
    ) -> (f64, f64) {
        let k = wavenumber(self.config.frequency_hz, self.config.sound_speed_m_s);
        let mut re = 0.0;
        let mut im = 0.0;

        for (i, pos) in self.config.transducer_positions.iter().enumerate() {
            let dx = position[0] - pos[0];
            let dy = position[1] - pos[1];
            let dz = position[2] - pos[2];
            let r = (dx * dx + dy * dy + dz * dz).sqrt();
            if r < 1.0e-15 {
                continue; // avoid singularity at the source
            }

            let amp = amplitudes.get(i).copied().unwrap_or(1.0);
            let phase = phases.get(i).copied().unwrap_or(0.0);
            let phasor = k * r + phase;
            re += (amp / r) * phasor.cos();
            im += (amp / r) * phasor.sin();
        }

        (re, im)
    }

    /// Compute z-coordinates of pressure nodes in a 1-D standing wave
    /// between a transducer at z = 0 and a reflector at `reflector_distance`.
    ///
    /// Pressure nodes occur at z = n·λ/2 for n = 1, 2, ... as long as z < reflector_distance.
    /// (With a hard reflector at z = reflector_distance the pressure has an antinode there.)
    pub fn standing_wave_nodes(&self, reflector_distance: f64) -> Vec<f64> {
        let half_lambda = self.wavelength() / 2.0;
        let mut nodes = Vec::new();
        let mut n = 1;
        loop {
            let z = n as f64 * half_lambda;
            if z >= reflector_distance {
                break;
            }
            nodes.push(z);
            n += 1;
        }
        nodes
    }

    /// Compute z-coordinates of pressure antinodes in a 1-D standing wave
    /// between a transducer at z = 0 and a reflector at `reflector_distance`.
    ///
    /// Antinodes occur at z = (2n−1)·λ/4 for n = 1, 2, ...
    pub fn standing_wave_antinodes(&self, reflector_distance: f64) -> Vec<f64> {
        let quarter_lambda = self.wavelength() / 4.0;
        let mut antinodes = Vec::new();
        let mut n = 1;
        loop {
            let z = (2 * n - 1) as f64 * quarter_lambda;
            if z >= reflector_distance {
                break;
            }
            antinodes.push(z);
            n += 1;
        }
        antinodes
    }

    /// Compute the Gor'kov acoustic radiation potential U at `position`.
    ///
    /// U = (2π a³ / 3) · [ f₁ · <p²> / (ρ₀ c²) − (3/2) · f₂ · ρ₀ · <v²> ]
    ///
    /// where f₁ and f₂ are the monopole and dipole scattering coefficients
    /// depending on particle and medium properties, and <p²>, <v²> are the
    /// time-averaged squared pressure and velocity at the position.
    ///
    /// The velocity is estimated from the pressure gradient via finite differences.
    ///
    /// # Arguments
    /// * `position` - [x, y, z] in metres
    /// * `amplitudes` - Transducer amplitudes
    /// * `phases` - Transducer phases in radians
    /// * `particle_radius` - Radius of the levitated sphere in metres
    /// * `particle_density` - Density of the particle in kg/m³
    pub fn gorkov_potential(
        &self,
        position: [f64; 3],
        amplitudes: &[f64],
        phases: &[f64],
        particle_radius: f64,
        particle_density: f64,
    ) -> f64 {
        let rho0 = self.config.medium_density_kg_m3;
        let c = self.config.sound_speed_m_s;
        let a = particle_radius;

        // Compressibility of medium (air is approximately ideal gas)
        let kappa_0 = 1.0 / (rho0 * c * c);
        // Assume particle is much less compressible (solid sphere)
        let kappa_p = kappa_0 * 0.01; // effectively incompressible

        let f1 = 1.0 - kappa_p / kappa_0;
        let f2 = 2.0 * (particle_density - rho0) / (2.0 * particle_density + rho0);

        // Time-averaged squared pressure at the point
        let (pr, pi) = self.pressure_field(position, amplitudes, phases);
        let p_sq = pr * pr + pi * pi;

        // Estimate <v²> from pressure gradient (finite differences)
        let delta = self.wavelength() * 0.001; // small step
        let mut v_sq = 0.0;
        for axis in 0..3 {
            let mut pos_plus = position;
            let mut pos_minus = position;
            pos_plus[axis] += delta;
            pos_minus[axis] -= delta;

            let (pr_plus, pi_plus) = self.pressure_field(pos_plus, amplitudes, phases);
            let (pr_minus, pi_minus) = self.pressure_field(pos_minus, amplitudes, phases);

            let dp_re = (pr_plus - pr_minus) / (2.0 * delta);
            let dp_im = (pi_plus - pi_minus) / (2.0 * delta);

            // |grad p|^2 component, then v = grad_p / (j ω ρ₀) → |v|^2 = |grad_p|^2 / (ω ρ₀)^2
            v_sq += dp_re * dp_re + dp_im * dp_im;
        }
        let omega = 2.0 * PI * self.config.frequency_hz;
        v_sq /= (omega * rho0) * (omega * rho0);

        // Gor'kov potential
        let volume_factor = 2.0 * PI * a * a * a / 3.0;
        volume_factor * (f1 * p_sq / (rho0 * c * c) - 1.5 * f2 * rho0 * v_sq)
    }

    /// Compute the acoustic radiation force F = −∇U at `position`.
    ///
    /// Returns [Fx, Fy, Fz] in Newtons computed via central finite differences
    /// of the Gor'kov potential.
    pub fn acoustic_radiation_force(
        &self,
        position: [f64; 3],
        amplitudes: &[f64],
        phases: &[f64],
        particle_radius: f64,
        particle_density: f64,
    ) -> [f64; 3] {
        let delta = self.wavelength() * 0.001;
        let mut force = [0.0; 3];

        for axis in 0..3 {
            let mut pos_plus = position;
            let mut pos_minus = position;
            pos_plus[axis] += delta;
            pos_minus[axis] -= delta;

            let u_plus = self.gorkov_potential(
                pos_plus,
                amplitudes,
                phases,
                particle_radius,
                particle_density,
            );
            let u_minus = self.gorkov_potential(
                pos_minus,
                amplitudes,
                phases,
                particle_radius,
                particle_density,
            );

            force[axis] = -(u_plus - u_minus) / (2.0 * delta);
        }

        force
    }

    /// Check whether the acoustic force in the z-direction exceeds gravity.
    ///
    /// Levitation occurs when |F_acoustic_z| > m·g.
    ///
    /// # Arguments
    /// * `particle_mass_kg` - Mass of the particle
    /// * `acoustic_force_z` - Upward acoustic force component (positive = upward)
    pub fn levitation_criterion(&self, particle_mass_kg: f64, acoustic_force_z: f64) -> bool {
        let g = 9.80665; // standard gravity m/s²
        acoustic_force_z.abs() > particle_mass_kg * g
    }

    /// Compute optimal transducer phases to focus the acoustic field at `focus_point`.
    ///
    /// Each transducer's phase is set so that all contributions arrive in-phase
    /// at the focus: φ_i = −k·r_i (mod 2π), where r_i is the distance from
    /// transducer i to the focus point.
    pub fn optimal_phases_for_focus(&self, focus_point: [f64; 3]) -> Vec<f64> {
        let k = wavenumber(self.config.frequency_hz, self.config.sound_speed_m_s);
        self.config
            .transducer_positions
            .iter()
            .map(|pos| {
                let dx = focus_point[0] - pos[0];
                let dy = focus_point[1] - pos[1];
                let dz = focus_point[2] - pos[2];
                let r = (dx * dx + dy * dy + dz * dz).sqrt();
                // Negative so all waves arrive in phase at the focus
                let phase = -(k * r) % (2.0 * PI);
                // Normalize to [0, 2π)
                if phase < 0.0 {
                    phase + 2.0 * PI
                } else {
                    phase
                }
            })
            .collect()
    }
}

// ──────────────────────────────────────────────────────────
// StandingWaveAnalyzer
// ──────────────────────────────────────────────────────────

/// Analysis utilities for 1-D standing wave fields in a resonant cavity.
///
/// Models the pressure and velocity distributions for a plane standing wave
/// between a transducer and a reflector.
pub struct StandingWaveAnalyzer {
    wavelength: f64,
}

impl StandingWaveAnalyzer {
    /// Create a new analyzer for the given wavelength (metres).
    pub fn new(wavelength: f64) -> Self {
        Self { wavelength }
    }

    /// Compute the pressure amplitude profile |sin(kz)| along the z-axis
    /// for a standing wave with a reflector at `reflector_z`.
    ///
    /// The reflector is a pressure antinode (hard boundary), so:
    ///   p(z) = amplitude · |sin(k · (reflector_z − z))|
    pub fn pressure_profile_1d(
        &self,
        z_positions: &[f64],
        reflector_z: f64,
        amplitude: f64,
    ) -> Vec<f64> {
        let k = 2.0 * PI / self.wavelength;
        z_positions
            .iter()
            .map(|&z| amplitude * (k * (reflector_z - z)).sin().abs())
            .collect()
    }

    /// Compute the velocity amplitude profile |cos(kz)| along the z-axis.
    ///
    /// Velocity nodes are at antinodes of pressure and vice versa:
    ///   v(z) ∝ amplitude · |cos(k · (reflector_z − z))|
    pub fn velocity_profile_1d(
        &self,
        z_positions: &[f64],
        reflector_z: f64,
        amplitude: f64,
    ) -> Vec<f64> {
        let k = 2.0 * PI / self.wavelength;
        z_positions
            .iter()
            .map(|&z| amplitude * (k * (reflector_z - z)).cos().abs())
            .collect()
    }

    /// Distance between adjacent pressure nodes: λ/2.
    pub fn node_spacing(&self) -> f64 {
        self.wavelength / 2.0
    }

    /// Maximum particle radius that can be trapped in a single pressure node.
    ///
    /// Empirically the particle must be smaller than about λ/4 for stable
    /// single-node trapping.
    pub fn max_particle_radius_for_node(wavelength: f64) -> f64 {
        wavelength / 4.0
    }

    /// Quality factor of a resonant cavity: Q = f₀ / Δf.
    ///
    /// # Arguments
    /// * `resonance_freq` - Centre frequency of the resonance in Hz
    /// * `bandwidth` - 3 dB bandwidth of the resonance in Hz
    pub fn quality_factor(resonance_freq: f64, bandwidth: f64) -> f64 {
        resonance_freq / bandwidth
    }

    /// Resonance frequencies of a cavity of length L with hard boundaries:
    ///   f_n = n·c / (2L)  for n = 1, 2, ..., num_modes.
    pub fn resonance_frequencies(
        cavity_length: f64,
        sound_speed: f64,
        num_modes: usize,
    ) -> Vec<f64> {
        (1..=num_modes)
            .map(|n| n as f64 * sound_speed / (2.0 * cavity_length))
            .collect()
    }
}

// ──────────────────────────────────────────────────────────
// TransducerModel
// ──────────────────────────────────────────────────────────

/// Acoustic model of a circular piston transducer.
///
/// The far-field directivity of a circular piston of radius `a` radiating
/// at wavenumber `k` is:
///
///   D(θ) = 2·J₁(ka sin θ) / (ka sin θ)
///
/// where J₁ is the Bessel function of the first kind, order 1.
pub struct TransducerModel {
    frequency: f64,
    radius: f64,
}

impl TransducerModel {
    /// Create a transducer model.
    ///
    /// # Arguments
    /// * `frequency` - Operating frequency in Hz
    /// * `radius` - Transducer radius in metres
    pub fn new(frequency: f64, radius: f64) -> Self {
        Self { frequency, radius }
    }

    /// Far-field directivity of a circular piston at angle `angle_rad` from the axis.
    ///
    /// D(θ) = 2·J₁(ka sin θ) / (ka sin θ)
    ///
    /// On-axis (θ = 0) the directivity is 1.0.
    pub fn directivity(&self, angle_rad: f64) -> f64 {
        let k = wavenumber(self.frequency, 343.0);
        let x = k * self.radius * angle_rad.sin();
        if x.abs() < 1.0e-12 {
            return 1.0; // limit as x→0 is 1
        }
        let j1 = bessel_j1(x);
        (2.0 * j1 / x).abs()
    }

    /// Near-field distance (Fresnel distance) for a circular aperture.
    ///
    /// N = D² / (4λ) = a² · f / c  (using diameter D = 2a).
    ///
    /// Beyond this distance the field is approximately a plane wave.
    pub fn near_field_distance(radius: f64, wavelength: f64) -> f64 {
        let diameter = 2.0 * radius;
        diameter * diameter / (4.0 * wavelength)
    }

    /// Sound Pressure Level at distance `distance_m` from a source
    /// radiating `power_w` watts isotropically.
    ///
    /// SPL = 10·log10(P / (4π r²)) + 120 + 10·log10(ρ c)
    ///
    /// Simplified: assuming ρc ≈ 413 for air, intensity I = P/(4πr²),
    /// p_rms = √(I·ρc), SPL = 20·log10(p_rms / 20e-6).
    pub fn sound_pressure_level(power_w: f64, distance_m: f64) -> f64 {
        let rho_c = 1.225 * 343.0; // air impedance ≈ 420 Pa·s/m
        let intensity = power_w / (4.0 * PI * distance_m * distance_m);
        let p_rms = (intensity * rho_c).sqrt();
        pressure_to_spl(p_rms)
    }

    /// Half-power (−3 dB) beam width in degrees.
    ///
    /// For a circular piston the first null occurs at sin(θ) ≈ 1.22 λ/D.
    /// The −3 dB beam width is approximately:
    ///   θ_3dB ≈ 2 · arcsin(0.514 λ / D)  (in radians, converted to degrees).
    ///
    /// For small arguments this simplifies to ≈ 58.9 λ / D degrees.
    pub fn beam_width_deg(radius: f64, wavelength: f64) -> f64 {
        let diameter = 2.0 * radius;
        let sin_half = 0.514 * wavelength / diameter;
        // Clamp for large λ/D ratios
        if sin_half >= 1.0 {
            return 180.0;
        }
        2.0 * sin_half.asin().to_degrees()
    }
}

// ──────────────────────────────────────────────────────────
// PhasedArrayController
// ──────────────────────────────────────────────────────────

/// Controller for a phased transducer array used in acoustic levitation.
///
/// Supports beam steering, twin-trap creation, vortex traps with orbital
/// angular momentum, and smooth particle trajectory generation.
pub struct PhasedArrayController {
    num_elements: usize,
    element_spacing: f64,
    frequency: f64,
    sound_speed: f64,
}

impl PhasedArrayController {
    /// Create a new phased array controller.
    ///
    /// # Arguments
    /// * `num_elements` - Number of transducer elements in the linear array
    /// * `element_spacing` - Centre-to-centre spacing in metres
    /// * `frequency` - Operating frequency in Hz
    /// * `sound_speed` - Speed of sound in m/s
    pub fn new(
        num_elements: usize,
        element_spacing: f64,
        frequency: f64,
        sound_speed: f64,
    ) -> Self {
        Self {
            num_elements,
            element_spacing,
            frequency,
            sound_speed,
        }
    }

    /// Compute the phase delays for beam steering to `angle_deg` from broadside.
    ///
    /// For a uniform linear array along the x-axis:
    ///   φ_n = −n · k · d · sin(θ)
    ///
    /// where d is the element spacing and θ is the steering angle.
    pub fn beam_steer_phases(&self, angle_deg: f64) -> Vec<f64> {
        let k = wavenumber(self.frequency, self.sound_speed);
        let sin_theta = angle_deg.to_radians().sin();
        (0..self.num_elements)
            .map(|n| -(n as f64) * k * self.element_spacing * sin_theta)
            .collect()
    }

    /// Create a twin-trap (two opposing foci) for stable 3-D particle trapping.
    ///
    /// The twin trap uses two focal points slightly above and below `position`
    /// (separated by λ/2). The upper focus has inverted phase (π shift)
    /// creating a pressure node at the target position.
    ///
    /// Returns (amplitudes, phases) for each element. Elements are assumed
    /// on the z-axis at even spacing starting from z = 0.
    pub fn create_twin_trap(&self, position: [f64; 3]) -> (Vec<f64>, Vec<f64>) {
        let k = wavenumber(self.frequency, self.sound_speed);
        let wavelength = self.sound_speed / self.frequency;
        let offset = wavelength / 4.0;

        // Two foci: one above, one below the target
        let focus_above = [position[0], position[1], position[2] + offset];
        let focus_below = [position[0], position[1], position[2] - offset];

        let mut amplitudes = vec![1.0; self.num_elements];
        let mut phases = vec![0.0; self.num_elements];

        for n in 0..self.num_elements {
            // Assume elements along z-axis for simplicity
            let elem_pos = [0.0, 0.0, n as f64 * self.element_spacing];

            let r_above = dist3(elem_pos, focus_above);
            let r_below = dist3(elem_pos, focus_below);

            // Phase to focus at both points (superposition with π shift on upper)
            let phase_below = -(k * r_below) % (2.0 * PI);
            let phase_above = (-(k * r_above) + PI) % (2.0 * PI);

            // Average the two contributions
            phases[n] = (phase_below + phase_above) / 2.0;
            amplitudes[n] = 1.0;
        }

        (amplitudes, phases)
    }

    /// Create a vortex trap with the specified topological charge (orbital
    /// angular momentum order).
    ///
    /// A vortex trap adds an azimuthal phase gradient of `m·φ_azimuth`
    /// around the array, where `m` is the topological charge. This creates
    /// a ring-shaped intensity pattern suitable for trapping asymmetric
    /// particles.
    ///
    /// Returns phase array for each element.
    pub fn create_vortex_trap(&self, position: [f64; 3], topological_charge: i32) -> Vec<f64> {
        let k = wavenumber(self.frequency, self.sound_speed);

        (0..self.num_elements)
            .map(|n| {
                // Assume elements in a ring: distribute azimuthally
                let azimuth = 2.0 * PI * (n as f64) / (self.num_elements as f64);
                let elem_x = self.element_spacing * (self.num_elements as f64 / 2.0) * azimuth.cos();
                let elem_y = self.element_spacing * (self.num_elements as f64 / 2.0) * azimuth.sin();
                let elem_pos = [elem_x, elem_y, 0.0];

                let r = dist3(elem_pos, position);
                let focus_phase = -(k * r);
                let vortex_phase = topological_charge as f64 * azimuth;
                (focus_phase + vortex_phase) % (2.0 * PI)
            })
            .collect()
    }

    /// Generate a trajectory of phase arrays to smoothly move a particle
    /// from its current position (encoded in `current_phases`) to `target_pos`.
    ///
    /// Uses linear interpolation between the current phases and the phases
    /// needed for the target position over `steps` intermediate states.
    ///
    /// Returns a vector of phase arrays, one per step.
    pub fn move_particle(
        &self,
        current_phases: &[f64],
        target_pos: [f64; 3],
        steps: usize,
    ) -> Vec<Vec<f64>> {
        let k = wavenumber(self.frequency, self.sound_speed);

        // Compute target phases (focus at target_pos)
        let target_phases: Vec<f64> = (0..self.num_elements)
            .map(|n| {
                let elem_pos = [0.0, 0.0, n as f64 * self.element_spacing];
                let r = dist3(elem_pos, target_pos);
                let phase = -(k * r) % (2.0 * PI);
                if phase < 0.0 { phase + 2.0 * PI } else { phase }
            })
            .collect();

        let steps = steps.max(1);
        (0..steps)
            .map(|step| {
                let t = (step + 1) as f64 / steps as f64;
                current_phases
                    .iter()
                    .zip(target_phases.iter())
                    .map(|(&c, &tgt)| {
                        // Interpolate via shortest arc on the unit circle
                        let mut diff = tgt - c;
                        if diff > PI {
                            diff -= 2.0 * PI;
                        } else if diff < -PI {
                            diff += 2.0 * PI;
                        }
                        let mut phase = c + t * diff;
                        // Normalize to [0, 2π)
                        phase = phase % (2.0 * PI);
                        if phase < 0.0 { phase + 2.0 * PI } else { phase }
                    })
                    .collect()
            })
            .collect()
    }
}

// ──────────────────────────────────────────────────────────
// ParticleProperties
// ──────────────────────────────────────────────────────────

/// Utility methods for computing acoustic levitation properties of particles.
pub struct ParticleProperties;

impl ParticleProperties {
    /// Estimate the maximum levitable particle diameter in millimetres
    /// at a given frequency and Sound Pressure Level.
    ///
    /// Based on Bond number analysis: the acoustic radiation pressure
    /// must overcome gravity. For a sphere of density ρ_p in air:
    ///   d_max ∝ sqrt(p_acoustic / (ρ_p · g))
    ///
    /// This uses a simplified scaling: at 40 kHz / 160 dB SPL,
    /// objects up to ~5 mm can be levitated.
    pub fn max_levitable_diameter_mm(frequency_hz: f64, spl_db: f64) -> f64 {
        // Reference: 40 kHz, 160 dB → ~5 mm for water droplets
        let p = spl_to_pressure(spl_db);
        // Scaling: diameter ∝ sqrt(pressure) / sqrt(frequency)
        // Reference point: p_ref at 160 dB, f_ref = 40 kHz, d_ref = 5 mm
        let p_ref = spl_to_pressure(160.0);
        let f_ref = 40_000.0;
        5.0 * (p / p_ref).sqrt() * (f_ref / frequency_hz).sqrt()
    }

    /// Acoustic contrast factor Φ that determines whether a particle is
    /// attracted to pressure nodes (Φ > 0) or antinodes (Φ < 0).
    ///
    /// Φ = f₁/3 + f₂/2
    ///
    /// where:
    ///   f₁ = 1 − κ_p/κ₀  (monopole coefficient)
    ///   f₂ = 2(ρ_p − ρ₀) / (2ρ_p + ρ₀)  (dipole coefficient)
    ///
    /// # Arguments
    /// * `particle_density` - Density of particle (kg/m³)
    /// * `medium_density` - Density of medium (kg/m³)
    /// * `particle_compressibility` - Compressibility of particle (1/Pa)
    /// * `medium_compressibility` - Compressibility of medium (1/Pa)
    pub fn acoustic_contrast_factor(
        particle_density: f64,
        medium_density: f64,
        particle_compressibility: f64,
        medium_compressibility: f64,
    ) -> f64 {
        let f1 = 1.0 - particle_compressibility / medium_compressibility;
        let f2 = 2.0 * (particle_density - medium_density)
            / (2.0 * particle_density + medium_density);
        f1 / 3.0 + f2 / 2.0
    }

    /// Terminal settling velocity of a sphere under Stokes drag.
    ///
    /// v = 2 a² (ρ_p − ρ_m) g / (9 μ)
    ///
    /// Valid for low Reynolds numbers (Re < 1).
    ///
    /// # Arguments
    /// * `particle_radius` - Radius of the particle in metres
    /// * `particle_density` - Density of the particle in kg/m³
    /// * `medium_density` - Density of the medium in kg/m³
    /// * `viscosity` - Dynamic viscosity of the medium in Pa·s
    pub fn settling_velocity(
        particle_radius: f64,
        particle_density: f64,
        medium_density: f64,
        viscosity: f64,
    ) -> f64 {
        let g = 9.80665;
        2.0 * particle_radius * particle_radius * (particle_density - medium_density) * g
            / (9.0 * viscosity)
    }
}

// ──────────────────────────────────────────────────────────
// Internal helpers
// ──────────────────────────────────────────────────────────

/// Euclidean distance between two 3-D points.
fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Approximation of the Bessel function of the first kind, order 1, J₁(x).
///
/// Uses a polynomial approximation valid for all x, based on Abramowitz & Stegun.
fn bessel_j1(x: f64) -> f64 {
    let ax = x.abs();
    if ax < 8.0 {
        // Polynomial approximation for |x| < 8
        let y = x * x;
        let num = x
            * (72362614232.0
                + y * (-7895059235.0
                    + y * (242396853.1
                        + y * (-2972611.439
                            + y * (15704.4826 + y * (-30.16036606))))));
        let den = 144725228442.0
            + y * (2300535178.0
                + y * (18583304.74
                    + y * (99447.43394 + y * (376.9991397 + y * 1.0))));
        num / den
    } else {
        // Asymptotic expansion for |x| >= 8
        let z = 8.0 / ax;
        let y = z * z;
        let xx = ax - 2.356194491; // ax - 3π/4

        let p = 1.0
            + y * (0.183105e-2
                + y * (-0.3516396496e-4
                    + y * (0.2457520174e-5 + y * (-0.240337019e-6))));
        let q = 0.04687499995
            + y * (-0.2002690873e-3
                + y * (0.8449199096e-5
                    + y * (-0.88228987e-6 + y * 0.105787412e-6)));

        let ans = (0.636619772 / ax).sqrt() * (xx.cos() * p - z * xx.sin() * q);
        if x < 0.0 { -ans } else { ans }
    }
}

// ──────────────────────────────────────────────────────────
// Tests
// ──────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOLERANCE: f64 = 1.0e-6;

    fn default_controller() -> LevitationController {
        LevitationController::new(LevitationConfig::default())
    }

    // --- Wavenumber ---

    #[test]
    fn test_wavenumber_40khz() {
        let k = wavenumber(40_000.0, 343.0);
        let expected = 2.0 * PI * 40_000.0 / 343.0;
        assert!((k - expected).abs() < TOLERANCE, "k={k}, expected={expected}");
    }

    #[test]
    fn test_wavenumber_1khz() {
        let k = wavenumber(1_000.0, 343.0);
        let expected = 2.0 * PI * 1_000.0 / 343.0;
        assert!((k - expected).abs() < TOLERANCE);
    }

    // --- Wavelength ---

    #[test]
    fn test_wavelength_40khz_in_air() {
        let ctrl = default_controller();
        let lambda = ctrl.wavelength();
        // λ = 343 / 40000 = 0.008575 m ≈ 8.575 mm
        assert!(
            (lambda - 0.008575).abs() < 1.0e-6,
            "wavelength={lambda}, expected ≈ 0.008575"
        );
    }

    #[test]
    fn test_wavelength_in_water() {
        let config = LevitationConfig {
            sound_speed_m_s: 1500.0,
            frequency_hz: 40_000.0,
            ..Default::default()
        };
        let ctrl = LevitationController::new(config);
        let lambda = ctrl.wavelength();
        assert!((lambda - 0.0375).abs() < 1.0e-6);
    }

    // --- SPL / Pressure conversion ---

    #[test]
    fn test_spl_to_pressure_and_back() {
        let spl = 120.0; // 20 Pa
        let p = spl_to_pressure(spl);
        assert!((p - 20.0).abs() < 0.01, "p={p}, expected ≈ 20 Pa");
        let spl_back = pressure_to_spl(p);
        assert!(
            (spl_back - spl).abs() < 0.001,
            "spl_back={spl_back}, expected={spl}"
        );
    }

    #[test]
    fn test_spl_to_pressure_94db() {
        // 94 dB SPL = 1 Pa
        let p = spl_to_pressure(94.0);
        assert!((p - 1.0).abs() < 0.01, "p={p}, expected ≈ 1 Pa");
    }

    #[test]
    fn test_pressure_to_spl_roundtrip() {
        for spl in [60.0, 80.0, 100.0, 140.0, 160.0] {
            let p = spl_to_pressure(spl);
            let spl2 = pressure_to_spl(p);
            assert!(
                (spl2 - spl).abs() < 0.01,
                "Roundtrip failed for SPL={spl}: got {spl2}"
            );
        }
    }

    // --- Standing wave nodes and antinodes ---

    #[test]
    fn test_standing_wave_nodes() {
        let ctrl = default_controller();
        let lambda = ctrl.wavelength();
        let reflector = lambda * 3.0; // 3 wavelengths distance
        let nodes = ctrl.standing_wave_nodes(reflector);
        // Nodes at λ/2, λ, 3λ/2, 2λ, 5λ/2
        assert_eq!(nodes.len(), 5, "Expected 5 nodes, got {}", nodes.len());
        for (i, node) in nodes.iter().enumerate() {
            let expected = (i + 1) as f64 * lambda / 2.0;
            assert!(
                (node - expected).abs() < 1.0e-10,
                "Node {i}: {node} vs expected {expected}"
            );
        }
    }

    #[test]
    fn test_standing_wave_antinodes() {
        let ctrl = default_controller();
        let lambda = ctrl.wavelength();
        let reflector = lambda * 2.0;
        let antinodes = ctrl.standing_wave_antinodes(reflector);
        // Antinodes at λ/4, 3λ/4, 5λ/4, 7λ/4
        assert_eq!(antinodes.len(), 4);
        for (i, an) in antinodes.iter().enumerate() {
            let expected = (2 * (i + 1) - 1) as f64 * lambda / 4.0;
            assert!(
                (an - expected).abs() < 1.0e-10,
                "Antinode {i}: {an} vs expected {expected}"
            );
        }
    }

    #[test]
    fn test_node_spacing() {
        let ctrl = default_controller();
        let lambda = ctrl.wavelength();
        let analyzer = StandingWaveAnalyzer::new(lambda);
        let spacing = analyzer.node_spacing();
        assert!(
            (spacing - lambda / 2.0).abs() < 1.0e-12,
            "Node spacing {spacing} vs λ/2 = {}",
            lambda / 2.0
        );
    }

    // --- Pressure and velocity profiles ---

    #[test]
    fn test_pressure_zero_at_nodes() {
        let ctrl = default_controller();
        let lambda = ctrl.wavelength();
        let reflector = lambda * 4.0;
        let nodes = ctrl.standing_wave_nodes(reflector);

        let analyzer = StandingWaveAnalyzer::new(lambda);
        let pressures = analyzer.pressure_profile_1d(&nodes, reflector, 1.0);

        for (i, &p) in pressures.iter().enumerate() {
            assert!(
                p.abs() < 1.0e-10,
                "Pressure at node {i} = {p}, expected ≈ 0"
            );
        }
    }

    #[test]
    fn test_pressure_max_at_antinodes() {
        let ctrl = default_controller();
        let lambda = ctrl.wavelength();
        let reflector = lambda * 4.0;
        let antinodes = ctrl.standing_wave_antinodes(reflector);

        let analyzer = StandingWaveAnalyzer::new(lambda);
        let pressures = analyzer.pressure_profile_1d(&antinodes, reflector, 1.0);

        for (i, &p) in pressures.iter().enumerate() {
            assert!(
                (p - 1.0).abs() < 1.0e-10,
                "Pressure at antinode {i} = {p}, expected ≈ 1.0"
            );
        }
    }

    #[test]
    fn test_velocity_max_at_nodes() {
        let ctrl = default_controller();
        let lambda = ctrl.wavelength();
        let reflector = lambda * 4.0;
        let nodes = ctrl.standing_wave_nodes(reflector);

        let analyzer = StandingWaveAnalyzer::new(lambda);
        let velocities = analyzer.velocity_profile_1d(&nodes, reflector, 1.0);

        for (i, &v) in velocities.iter().enumerate() {
            assert!(
                (v - 1.0).abs() < 1.0e-10,
                "Velocity at node {i} = {v}, expected ≈ 1.0"
            );
        }
    }

    #[test]
    fn test_velocity_zero_at_antinodes() {
        let ctrl = default_controller();
        let lambda = ctrl.wavelength();
        let reflector = lambda * 4.0;
        let antinodes = ctrl.standing_wave_antinodes(reflector);

        let analyzer = StandingWaveAnalyzer::new(lambda);
        let velocities = analyzer.velocity_profile_1d(&antinodes, reflector, 1.0);

        for (i, &v) in velocities.iter().enumerate() {
            assert!(
                v.abs() < 1.0e-10,
                "Velocity at antinode {i} = {v}, expected ≈ 0"
            );
        }
    }

    // --- Resonance frequencies ---

    #[test]
    fn test_resonance_frequencies_integer_multiples() {
        let freqs = StandingWaveAnalyzer::resonance_frequencies(0.1, 343.0, 5);
        assert_eq!(freqs.len(), 5);
        let f1 = 343.0 / (2.0 * 0.1); // 1715 Hz
        for (i, &f) in freqs.iter().enumerate() {
            let expected = (i + 1) as f64 * f1;
            assert!(
                (f - expected).abs() < 0.01,
                "Mode {}: f={f}, expected={expected}",
                i + 1
            );
        }
    }

    #[test]
    fn test_resonance_fundamental() {
        let freqs = StandingWaveAnalyzer::resonance_frequencies(0.5, 343.0, 1);
        // f₁ = 343 / (2 * 0.5) = 343 Hz
        assert!((freqs[0] - 343.0).abs() < 0.01);
    }

    // --- Quality factor ---

    #[test]
    fn test_quality_factor() {
        let q = StandingWaveAnalyzer::quality_factor(40_000.0, 200.0);
        assert!((q - 200.0).abs() < 0.01);
    }

    // --- Directivity ---

    #[test]
    fn test_directivity_on_axis() {
        let tm = TransducerModel::new(40_000.0, 0.005);
        let d = tm.directivity(0.0);
        assert!(
            (d - 1.0).abs() < 1.0e-6,
            "On-axis directivity = {d}, expected 1.0"
        );
    }

    #[test]
    fn test_directivity_decreases_off_axis() {
        let tm = TransducerModel::new(40_000.0, 0.005);
        let d_0 = tm.directivity(0.0);
        let d_10 = tm.directivity(10.0_f64.to_radians());
        let d_30 = tm.directivity(30.0_f64.to_radians());
        assert!(d_0 > d_10, "d(0) should be > d(10deg)");
        assert!(d_10 > d_30, "d(10deg) should be > d(30deg)");
    }

    // --- Near field distance ---

    #[test]
    fn test_near_field_distance() {
        let lambda = 343.0 / 40_000.0;
        let nfd = TransducerModel::near_field_distance(0.005, lambda);
        let d = 2.0 * 0.005;
        let expected = d * d / (4.0 * lambda);
        assert!(
            (nfd - expected).abs() < 1.0e-8,
            "NF distance {nfd} vs expected {expected}"
        );
    }

    #[test]
    fn test_near_field_increases_with_aperture() {
        let lambda = 343.0 / 40_000.0;
        let nf_small = TransducerModel::near_field_distance(0.005, lambda);
        let nf_large = TransducerModel::near_field_distance(0.010, lambda);
        assert!(
            nf_large > nf_small,
            "Larger aperture should give larger NF distance"
        );
        // Should scale as D² → 4x ratio
        let ratio = nf_large / nf_small;
        assert!((ratio - 4.0).abs() < 0.1, "ratio={ratio}, expected ≈ 4");
    }

    // --- Beam width ---

    #[test]
    fn test_beam_width_decreases_with_larger_aperture() {
        let lambda = 343.0 / 40_000.0;
        let bw_small = TransducerModel::beam_width_deg(0.005, lambda);
        let bw_large = TransducerModel::beam_width_deg(0.010, lambda);
        assert!(
            bw_large < bw_small,
            "Larger aperture should give narrower beam: bw_small={bw_small}, bw_large={bw_large}"
        );
    }

    // --- SPL ---

    #[test]
    fn test_spl_decreases_with_distance() {
        let spl_1m = TransducerModel::sound_pressure_level(1.0, 1.0);
        let spl_2m = TransducerModel::sound_pressure_level(1.0, 2.0);
        assert!(spl_1m > spl_2m, "SPL should decrease with distance");
        // Inverse square law: 6 dB drop per doubling
        let diff = spl_1m - spl_2m;
        assert!(
            (diff - 6.02).abs() < 0.1,
            "SPL drop={diff} dB, expected ≈ 6 dB"
        );
    }

    // --- Phase focusing ---

    #[test]
    fn test_optimal_phases_equidistant() {
        // All transducers equidistant from focus → all phases equal
        let config = LevitationConfig {
            num_transducers: 4,
            transducer_positions: vec![
                [0.01, 0.0, 0.0],
                [0.0, 0.01, 0.0],
                [-0.01, 0.0, 0.0],
                [0.0, -0.01, 0.0],
            ],
            ..Default::default()
        };
        let ctrl = LevitationController::new(config);
        let phases = ctrl.optimal_phases_for_focus([0.0, 0.0, 0.0]);
        // All should be equal since all are equidistant from the origin
        let first = phases[0];
        for (i, &ph) in phases.iter().enumerate() {
            assert!(
                (ph - first).abs() < 1.0e-10,
                "Phase[{i}]={ph} differs from phase[0]={first}"
            );
        }
    }

    #[test]
    fn test_optimal_phases_different_distances() {
        let config = LevitationConfig {
            num_transducers: 2,
            transducer_positions: vec![[0.0, 0.0, 0.0], [0.0, 0.0, 0.01]],
            ..Default::default()
        };
        let ctrl = LevitationController::new(config);
        let phases = ctrl.optimal_phases_for_focus([0.0, 0.0, 0.05]);
        // Transducers at different distances → different phases
        assert!(
            (phases[0] - phases[1]).abs() > 1.0e-6,
            "Phases should differ for non-equidistant transducers"
        );
    }

    // --- Levitation criterion ---

    #[test]
    fn test_levitation_criterion_sufficient_force() {
        let ctrl = default_controller();
        // 1 mg particle, 0.01 N force (way more than needed)
        assert!(ctrl.levitation_criterion(1.0e-6, 0.01));
    }

    #[test]
    fn test_levitation_criterion_insufficient_force() {
        let ctrl = default_controller();
        // 1 kg particle, tiny force
        assert!(!ctrl.levitation_criterion(1.0, 1.0e-6));
    }

    #[test]
    fn test_heavier_objects_need_more_force() {
        let ctrl = default_controller();
        let light_mass = 1.0e-6;
        let heavy_mass = 1.0e-3;
        let force = 1.0e-4; // 0.1 mN
        let can_lift_light = ctrl.levitation_criterion(light_mass, force);
        let can_lift_heavy = ctrl.levitation_criterion(heavy_mass, force);
        assert!(can_lift_light, "Should be able to lift light particle");
        assert!(!can_lift_heavy, "Should NOT be able to lift heavy particle");
    }

    // --- Acoustic contrast factor ---

    #[test]
    fn test_acoustic_contrast_factor_dense_particle() {
        // Dense particle (steel in air): should be positive (attracted to nodes)
        let rho_p = 7800.0; // steel
        let rho_m = 1.225; // air
        let kappa_m = 1.0 / (rho_m * 343.0 * 343.0); // air compressibility
        let kappa_p = kappa_m * 0.0001; // steel is much less compressible

        let phi = ParticleProperties::acoustic_contrast_factor(rho_p, rho_m, kappa_p, kappa_m);
        assert!(
            phi > 0.0,
            "Dense particles should have positive contrast factor, got {phi}"
        );
    }

    #[test]
    fn test_acoustic_contrast_factor_equal_properties() {
        // Particle same as medium → Φ = 0
        let rho = 1000.0;
        let kappa = 1.0e-9;
        let phi = ParticleProperties::acoustic_contrast_factor(rho, rho, kappa, kappa);
        assert!(phi.abs() < 1.0e-10, "Same properties should give Φ ≈ 0, got {phi}");
    }

    // --- Settling velocity ---

    #[test]
    fn test_settling_velocity_positive() {
        // Dense particle in air should settle downward
        let v = ParticleProperties::settling_velocity(
            0.001,  // 1 mm radius
            2500.0, // glass
            1.225,  // air
            1.81e-5, // air viscosity
        );
        assert!(v > 0.0, "Dense particle should have positive settling velocity");
    }

    #[test]
    fn test_settling_velocity_scales_with_radius_squared() {
        let v1 = ParticleProperties::settling_velocity(0.001, 2500.0, 1.225, 1.81e-5);
        let v2 = ParticleProperties::settling_velocity(0.002, 2500.0, 1.225, 1.81e-5);
        let ratio = v2 / v1;
        assert!(
            (ratio - 4.0).abs() < 0.01,
            "Velocity should scale as r²: ratio = {ratio}"
        );
    }

    // --- Max particle radius for node ---

    #[test]
    fn test_max_particle_radius() {
        let lambda = 343.0 / 40_000.0;
        let r_max = StandingWaveAnalyzer::max_particle_radius_for_node(lambda);
        assert!(
            (r_max - lambda / 4.0).abs() < 1.0e-10,
            "Max radius {r_max} vs λ/4 = {}",
            lambda / 4.0
        );
    }

    // --- Beam steering ---

    #[test]
    fn test_beam_steer_broadside() {
        let pac = PhasedArrayController::new(8, 0.004, 40_000.0, 343.0);
        let phases = pac.beam_steer_phases(0.0);
        // Broadside: all phases = 0
        for (i, &p) in phases.iter().enumerate() {
            assert!(
                p.abs() < 1.0e-12,
                "Phase[{i}]={p}, expected 0 for broadside"
            );
        }
    }

    #[test]
    fn test_beam_steer_non_zero() {
        let pac = PhasedArrayController::new(8, 0.004, 40_000.0, 343.0);
        let phases = pac.beam_steer_phases(30.0);
        // Should have progressive phase shift
        assert!(phases[1].abs() > 1.0e-6, "Should have non-zero phase shift");
        // Linear progression
        let step = phases[1] - phases[0];
        for i in 2..phases.len() {
            let expected = phases[0] + (i as f64) * step;
            assert!(
                (phases[i] - expected).abs() < 1.0e-10,
                "Non-linear phase progression at element {i}"
            );
        }
    }

    // --- Move particle ---

    #[test]
    fn test_move_particle_trajectory_length() {
        let pac = PhasedArrayController::new(4, 0.004, 40_000.0, 343.0);
        let start = vec![0.0; 4];
        let trajectory = pac.move_particle(&start, [0.0, 0.0, 0.05], 10);
        assert_eq!(trajectory.len(), 10);
        for step_phases in &trajectory {
            assert_eq!(step_phases.len(), 4);
        }
    }

    // --- Vortex trap ---

    #[test]
    fn test_vortex_trap_phases_count() {
        let pac = PhasedArrayController::new(16, 0.004, 40_000.0, 343.0);
        let phases = pac.create_vortex_trap([0.0, 0.0, 0.05], 1);
        assert_eq!(phases.len(), 16);
    }

    // --- Twin trap ---

    #[test]
    fn test_twin_trap_outputs() {
        let pac = PhasedArrayController::new(8, 0.004, 40_000.0, 343.0);
        let (amps, phases) = pac.create_twin_trap([0.0, 0.0, 0.03]);
        assert_eq!(amps.len(), 8);
        assert_eq!(phases.len(), 8);
        for &a in &amps {
            assert!((a - 1.0).abs() < 1.0e-12, "All amplitudes should be 1.0");
        }
    }

    // --- Max levitable diameter ---

    #[test]
    fn test_max_levitable_diameter_reference() {
        let d = ParticleProperties::max_levitable_diameter_mm(40_000.0, 160.0);
        assert!(
            (d - 5.0).abs() < 0.1,
            "Reference: 40 kHz/160 dB → ~5 mm, got {d}"
        );
    }

    #[test]
    fn test_max_levitable_diameter_scales_with_spl() {
        let d_low = ParticleProperties::max_levitable_diameter_mm(40_000.0, 140.0);
        let d_high = ParticleProperties::max_levitable_diameter_mm(40_000.0, 160.0);
        assert!(
            d_high > d_low,
            "Higher SPL should allow larger particle: d_low={d_low}, d_high={d_high}"
        );
    }

    // --- Pressure field superposition ---

    #[test]
    fn test_pressure_field_single_source() {
        let config = LevitationConfig {
            num_transducers: 1,
            transducer_positions: vec![[0.0, 0.0, 0.0]],
            ..Default::default()
        };
        let ctrl = LevitationController::new(config);
        let (re, im) = ctrl.pressure_field([0.0, 0.0, 0.01], &[1.0], &[0.0]);
        let mag = (re * re + im * im).sqrt();
        assert!(mag > 0.0, "Should have non-zero pressure");
    }

    // --- Gor'kov potential ---

    #[test]
    fn test_gorkov_potential_finite() {
        let config = LevitationConfig {
            num_transducers: 1,
            transducer_positions: vec![[0.0, 0.0, 0.0]],
            ..Default::default()
        };
        let ctrl = LevitationController::new(config);
        let u = ctrl.gorkov_potential(
            [0.0, 0.0, 0.01],
            &[1.0],
            &[0.0],
            0.0005,
            2500.0,
        );
        assert!(u.is_finite(), "Gor'kov potential should be finite");
    }

    // --- Acoustic radiation force ---

    #[test]
    fn test_acoustic_radiation_force_has_components() {
        let config = LevitationConfig {
            num_transducers: 1,
            transducer_positions: vec![[0.0, 0.0, 0.0]],
            ..Default::default()
        };
        let ctrl = LevitationController::new(config);
        let f = ctrl.acoustic_radiation_force(
            [0.001, 0.001, 0.01],
            &[1.0],
            &[0.0],
            0.0005,
            2500.0,
        );
        // Force should be finite
        for c in &f {
            assert!(c.is_finite(), "Force component should be finite");
        }
    }

    // --- Bessel J1 sanity ---

    #[test]
    fn test_bessel_j1_at_zero() {
        assert!(bessel_j1(0.0).abs() < 1.0e-10, "J1(0) = 0");
    }

    #[test]
    fn test_bessel_j1_known_value() {
        // J1(1.0) ≈ 0.44005
        let j = bessel_j1(1.0);
        assert!(
            (j - 0.44005).abs() < 0.001,
            "J1(1.0) = {j}, expected ≈ 0.44005"
        );
    }
}

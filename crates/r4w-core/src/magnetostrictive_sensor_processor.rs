//! Magnetostrictive sensor signal processing for position, force, and torque measurement.
//!
//! Implements time-of-flight waveguide position sensing, Villari effect force/torque
//! measurement, Wiedemann effect torsional wave generation, signal conditioning,
//! hysteresis modeling (simplified Jiles-Atherton), and magnetomechanical coupling analysis.

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Material properties
// ---------------------------------------------------------------------------

/// Physical properties of a magnetostrictive material.
#[derive(Debug, Clone)]
pub struct MaterialProperties {
    /// Saturation magnetostriction in parts per million (ppm).
    pub saturation_magnetostriction_ppm: f64,
    /// Young's modulus in Pascals.
    pub youngs_modulus_pa: f64,
    /// Density in kg/m^3.
    pub density_kgm3: f64,
    /// Relative magnetic permeability (dimensionless).
    pub permeability_relative: f64,
    /// Curie temperature in Kelvin.
    pub curie_temperature_k: f64,
}

impl MaterialProperties {
    /// Terfenol-D: giant magnetostrictive alloy (Tb_{0.3}Dy_{0.7}Fe_2).
    pub fn terfenol_d() -> Self {
        Self {
            saturation_magnetostriction_ppm: 1600.0,
            youngs_modulus_pa: 25.0e9,
            density_kgm3: 9250.0,
            permeability_relative: 10.0,
            curie_temperature_k: 653.0,
        }
    }

    /// Galfenol: iron-gallium alloy (Fe_{1-x}Ga_x).
    pub fn galfenol() -> Self {
        Self {
            saturation_magnetostriction_ppm: 400.0,
            youngs_modulus_pa: 60.0e9,
            density_kgm3: 7800.0,
            permeability_relative: 75.0,
            curie_temperature_k: 973.0,
        }
    }

    /// Nickel: common negative magnetostrictive material.
    pub fn nickel() -> Self {
        Self {
            saturation_magnetostriction_ppm: -34.0,
            youngs_modulus_pa: 200.0e9,
            density_kgm3: 8900.0,
            permeability_relative: 600.0,
            curie_temperature_k: 627.0,
        }
    }

    /// Iron: weakly negative magnetostrictive material.
    pub fn iron() -> Self {
        Self {
            saturation_magnetostriction_ppm: -7.0,
            youngs_modulus_pa: 211.0e9,
            density_kgm3: 7874.0,
            permeability_relative: 5000.0,
            curie_temperature_k: 1043.0,
        }
    }

    /// Shear modulus G = E / (2*(1+nu)).  Assume Poisson ratio nu ≈ 0.3.
    pub fn shear_modulus(&self) -> f64 {
        self.youngs_modulus_pa / (2.0 * (1.0 + 0.3))
    }

    /// Torsional wave velocity v = sqrt(G / rho).
    pub fn torsional_wave_velocity(&self) -> f64 {
        (self.shear_modulus() / self.density_kgm3).sqrt()
    }

    /// Longitudinal wave velocity v_l = sqrt(E / rho).
    pub fn longitudinal_wave_velocity(&self) -> f64 {
        (self.youngs_modulus_pa / self.density_kgm3).sqrt()
    }
}

// ---------------------------------------------------------------------------
// Time-of-flight position sensor
// ---------------------------------------------------------------------------

/// Configuration for a magnetostrictive waveguide position sensor.
#[derive(Debug, Clone)]
pub struct WaveguideConfig {
    /// Torsional wave speed in m/s (≈2800 for nickel-iron waveguide).
    pub wave_speed_ms: f64,
    /// Sampling rate of the digitiser in Hz.
    pub sample_rate_hz: f64,
    /// Total waveguide length in metres.
    pub waveguide_length_m: f64,
    /// Dead zone near the transmitter in metres.
    pub dead_zone_m: f64,
    /// Temperature coefficient of wave speed (1/K).
    pub temp_coeff: f64,
    /// Reference temperature in Kelvin.
    pub ref_temperature_k: f64,
}

impl Default for WaveguideConfig {
    fn default() -> Self {
        Self {
            wave_speed_ms: 2800.0,
            sample_rate_hz: 10.0e6,
            waveguide_length_m: 1.0,
            dead_zone_m: 0.02,
            temp_coeff: 1.5e-4,
            ref_temperature_k: 293.15,
        }
    }
}

impl WaveguideConfig {
    /// Effective wave speed at a given temperature.
    pub fn wave_speed_at_temp(&self, temperature_k: f64) -> f64 {
        self.wave_speed_ms * (1.0 + self.temp_coeff * (temperature_k - self.ref_temperature_k))
    }

    /// Position from round-trip time-of-flight: pos = v * t / 2.
    pub fn position_from_tof(&self, tof_seconds: f64) -> f64 {
        self.wave_speed_ms * tof_seconds / 2.0
    }

    /// Position with temperature compensation.
    pub fn position_from_tof_compensated(&self, tof_seconds: f64, temperature_k: f64) -> f64 {
        let v = self.wave_speed_at_temp(temperature_k);
        v * tof_seconds / 2.0
    }

    /// Position resolution from timing resolution: delta_x = v * delta_t / 2.
    pub fn resolution(&self) -> f64 {
        self.wave_speed_ms / (2.0 * self.sample_rate_hz)
    }

    /// Maximum measurable position (limited by waveguide length minus dead zone).
    pub fn max_position(&self) -> f64 {
        self.waveguide_length_m - self.dead_zone_m
    }
}

// ---------------------------------------------------------------------------
// Time-of-flight processing
// ---------------------------------------------------------------------------

/// Detect threshold crossings in a signal, returning sample indices where the
/// signal first exceeds `threshold` (rising edge, with minimum gap).
pub fn threshold_crossings(signal: &[f64], threshold: f64, min_gap_samples: usize) -> Vec<usize> {
    let mut crossings = Vec::new();
    let mut last_crossing: Option<usize> = None;
    let mut was_below = true;

    for (i, &s) in signal.iter().enumerate() {
        if was_below && s >= threshold {
            if let Some(lc) = last_crossing {
                if i - lc < min_gap_samples {
                    was_below = false;
                    continue;
                }
            }
            crossings.push(i);
            last_crossing = Some(i);
            was_below = false;
        } else if s < threshold {
            was_below = true;
        }
    }
    crossings
}

/// Find zero-crossings (positive-going) with linear interpolation for sub-sample accuracy.
pub fn zero_crossings_interpolated(signal: &[f64]) -> Vec<f64> {
    let mut crossings = Vec::new();
    for i in 0..signal.len().saturating_sub(1) {
        if signal[i] <= 0.0 && signal[i + 1] > 0.0 {
            // Linear interpolation: t = i + (-signal[i]) / (signal[i+1] - signal[i])
            let denom = signal[i + 1] - signal[i];
            if denom.abs() > 1e-30 {
                let frac = -signal[i] / denom;
                crossings.push(i as f64 + frac);
            }
        }
    }
    crossings
}

/// Parabolic interpolation around a peak to get sub-sample arrival time.
/// Given three consecutive samples y[k-1], y[k], y[k+1] where y[k] is the peak,
/// returns the fractional offset from k.
pub fn parabolic_interpolation(y_prev: f64, y_peak: f64, y_next: f64) -> f64 {
    let denom = 2.0 * (2.0 * y_peak - y_prev - y_next);
    if denom.abs() < 1e-30 {
        return 0.0;
    }
    (y_prev - y_next) / denom
}

/// Find the index of the maximum absolute value in a signal.
pub fn find_peak_index(signal: &[f64]) -> Option<usize> {
    if signal.is_empty() {
        return None;
    }
    let mut best_idx = 0;
    let mut best_val = signal[0].abs();
    for (i, &s) in signal.iter().enumerate().skip(1) {
        let a = s.abs();
        if a > best_val {
            best_val = a;
            best_idx = i;
        }
    }
    Some(best_idx)
}

/// Compute envelope of a signal using simple absolute-value smoothing (moving average of |x|).
pub fn envelope_detect(signal: &[f64], window_size: usize) -> Vec<f64> {
    let n = signal.len();
    if n == 0 || window_size == 0 {
        return vec![];
    }
    let w = window_size.min(n);
    let mut env = vec![0.0; n];
    let mut sum: f64 = signal[..w].iter().map(|x| x.abs()).sum();
    let half = w / 2;
    // Centre the first window
    if half < n {
        env[half] = sum / w as f64;
    }
    for i in 1..n {
        let centre = i + half;
        if centre >= n {
            break;
        }
        let add_idx = i + w - 1;
        if add_idx < n {
            sum += signal[add_idx].abs();
        }
        sum -= signal[i - 1].abs();
        env[centre] = sum / w as f64;
    }
    // Fill edges with nearest computed value
    if let Some(first_nonzero) = env.iter().position(|&v| v > 0.0) {
        for i in 0..first_nonzero {
            env[i] = env[first_nonzero];
        }
    }
    if let Some(last_nonzero) = env.iter().rposition(|&v| v > 0.0) {
        for i in (last_nonzero + 1)..n {
            env[i] = env[last_nonzero];
        }
    }
    env
}

/// Coherent averaging of multiple pulse acquisitions to improve SNR.
pub fn coherent_average(pulses: &[Vec<f64>]) -> Vec<f64> {
    if pulses.is_empty() {
        return vec![];
    }
    let n = pulses[0].len();
    let count = pulses.len() as f64;
    let mut avg = vec![0.0; n];
    for pulse in pulses {
        let len = pulse.len().min(n);
        for i in 0..len {
            avg[i] += pulse[i];
        }
    }
    for v in &mut avg {
        *v /= count;
    }
    avg
}

// ---------------------------------------------------------------------------
// Multi-magnet position detection
// ---------------------------------------------------------------------------

/// Detect multiple magnet positions from a torsional wave signal.
/// Returns positions in metres, filtered by dead zone and waveguide length.
pub fn detect_multi_magnet_positions(
    signal: &[f64],
    config: &WaveguideConfig,
    threshold: f64,
    min_separation_m: f64,
) -> Vec<f64> {
    let min_gap_samples =
        (min_separation_m * 2.0 / config.wave_speed_ms * config.sample_rate_hz) as usize;
    let crossings = threshold_crossings(signal, threshold, min_gap_samples.max(1));

    let mut positions = Vec::new();
    for &idx in &crossings {
        let tof = idx as f64 / config.sample_rate_hz;
        let pos = config.position_from_tof(tof);
        if pos >= config.dead_zone_m && pos <= config.max_position() {
            positions.push(pos);
        }
    }
    positions
}

// ---------------------------------------------------------------------------
// Velocity / acceleration from position history
// ---------------------------------------------------------------------------

/// Compute velocity from position samples using central differences.
/// Returns one fewer element than input if using forward differences,
/// or N-2 elements for central differences.
pub fn velocity_from_position(positions: &[f64], dt: f64) -> Vec<f64> {
    if positions.len() < 2 {
        return vec![];
    }
    if positions.len() == 2 {
        return vec![(positions[1] - positions[0]) / dt];
    }
    // Central differences for interior points
    let n = positions.len();
    let mut vel = Vec::with_capacity(n);
    // Forward difference for first point
    vel.push((positions[1] - positions[0]) / dt);
    for i in 1..n - 1 {
        vel.push((positions[i + 1] - positions[i - 1]) / (2.0 * dt));
    }
    // Backward difference for last point
    vel.push((positions[n - 1] - positions[n - 2]) / dt);
    vel
}

/// Compute acceleration from position samples using second central differences.
pub fn acceleration_from_position(positions: &[f64], dt: f64) -> Vec<f64> {
    if positions.len() < 3 {
        return vec![];
    }
    let n = positions.len();
    let dt2 = dt * dt;
    let mut acc = Vec::with_capacity(n - 2);
    for i in 1..n - 1 {
        acc.push((positions[i + 1] - 2.0 * positions[i] + positions[i - 1]) / dt2);
    }
    acc
}

// ---------------------------------------------------------------------------
// Villari effect (inverse magnetostriction) – force and torque sensing
// ---------------------------------------------------------------------------

/// Compute the relative permeability change due to applied stress (Villari effect).
/// Delta_mu / mu ≈ k_villari * sigma, where k_villari depends on material.
/// Returns the fractional change.
pub fn villari_permeability_change(
    stress_pa: f64,
    saturation_magnetostriction_ppm: f64,
    youngs_modulus_pa: f64,
) -> f64 {
    // Simplified linearised model: Δμ/μ ≈ 3λ_s σ / (μ₀ M_s²) ≈ k * σ
    // We use the approximation k ≈ 3 * lambda_s / E for small stress
    let lambda_s = saturation_magnetostriction_ppm * 1e-6;
    3.0 * lambda_s * stress_pa / youngs_modulus_pa
}

/// Compute force from stress and cross-sectional area: F = σ * A.
pub fn force_from_stress(stress_pa: f64, cross_section_area_m2: f64) -> f64 {
    stress_pa * cross_section_area_m2
}

/// Compute stress from measured inductance change and material properties.
/// sigma ≈ (ΔL/L) * E / (3 * lambda_s)
pub fn stress_from_inductance_change(
    delta_l_over_l: f64,
    youngs_modulus_pa: f64,
    saturation_magnetostriction_ppm: f64,
) -> f64 {
    let lambda_s = saturation_magnetostriction_ppm * 1e-6;
    if lambda_s.abs() < 1e-15 {
        return 0.0;
    }
    delta_l_over_l * youngs_modulus_pa / (3.0 * lambda_s)
}

/// Torque from stress and polar section modulus: T = σ * Z_p.
pub fn torque_from_stress(stress_pa: f64, polar_section_modulus_m3: f64) -> f64 {
    stress_pa * polar_section_modulus_m3
}

/// Polar section modulus for a solid circular shaft: Z_p = π d³ / 16.
pub fn polar_section_modulus_solid(diameter_m: f64) -> f64 {
    PI * diameter_m.powi(3) / 16.0
}

// ---------------------------------------------------------------------------
// Wiedemann effect
// ---------------------------------------------------------------------------

/// Wiedemann effect: torsional strain from combined axial (H_a) and
/// circumferential (H_c) magnetic fields.
/// Strain ≈ (3/2) λ_s sin(2θ) where θ = atan(H_c / H_a).
pub fn wiedemann_torsional_strain(
    h_axial: f64,
    h_circumferential: f64,
    saturation_magnetostriction_ppm: f64,
) -> f64 {
    let lambda_s = saturation_magnetostriction_ppm * 1e-6;
    let theta = h_circumferential.atan2(h_axial);
    1.5 * lambda_s * (2.0 * theta).sin()
}

/// Torsional wave velocity: v = sqrt(G / rho).
pub fn torsional_wave_velocity(shear_modulus_pa: f64, density_kgm3: f64) -> f64 {
    (shear_modulus_pa / density_kgm3).sqrt()
}

// ---------------------------------------------------------------------------
// Signal conditioning: simple bandpass filter
// ---------------------------------------------------------------------------

/// Simple second-order IIR bandpass filter (biquad) for torsional wave frequency content.
/// Centre frequency `fc`, quality factor `q`, at sample rate `fs`.
pub struct BandpassFilter {
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
    x1: f64,
    x2: f64,
    y1: f64,
    y2: f64,
}

impl BandpassFilter {
    /// Create a new bandpass filter.
    pub fn new(fc: f64, q: f64, fs: f64) -> Self {
        let w0 = 2.0 * PI * fc / fs;
        let alpha = w0.sin() / (2.0 * q);
        let a0 = 1.0 + alpha;
        Self {
            b0: alpha / a0,
            b1: 0.0,
            b2: -alpha / a0,
            a1: -2.0 * w0.cos() / a0,
            a2: (1.0 - alpha) / a0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// Process a single sample.
    pub fn process(&mut self, x: f64) -> f64 {
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }

    /// Filter an entire slice.
    pub fn filter(&mut self, signal: &[f64]) -> Vec<f64> {
        signal.iter().map(|&x| self.process(x)).collect()
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Temperature compensation
// ---------------------------------------------------------------------------

/// Correct position measurement for temperature-induced wave speed change.
/// Returns corrected position.
pub fn temperature_compensate_position(
    measured_position: f64,
    temperature_k: f64,
    config: &WaveguideConfig,
) -> f64 {
    let v_actual = config.wave_speed_at_temp(temperature_k);
    measured_position * v_actual / config.wave_speed_ms
}

// ---------------------------------------------------------------------------
// Error analysis
// ---------------------------------------------------------------------------

/// Position uncertainty from timing jitter: σ_x = v * σ_t / 2.
pub fn position_uncertainty_from_jitter(wave_speed: f64, timing_jitter_s: f64) -> f64 {
    wave_speed * timing_jitter_s / 2.0
}

/// Position error from temperature uncertainty.
/// δx = x * α * δT.
pub fn position_error_from_temperature(
    position: f64,
    temp_coeff: f64,
    temperature_uncertainty_k: f64,
) -> f64 {
    position * temp_coeff * temperature_uncertainty_k
}

/// Total position uncertainty (RSS of jitter and temperature contributions).
pub fn total_position_uncertainty(
    wave_speed: f64,
    timing_jitter_s: f64,
    position: f64,
    temp_coeff: f64,
    temperature_uncertainty_k: f64,
) -> f64 {
    let sigma_jitter = position_uncertainty_from_jitter(wave_speed, timing_jitter_s);
    let sigma_temp = position_error_from_temperature(position, temp_coeff, temperature_uncertainty_k);
    (sigma_jitter * sigma_jitter + sigma_temp * sigma_temp).sqrt()
}

/// Repeatability from noise floor: σ_repeat = v / (2 * SNR_linear * fs).
pub fn repeatability_from_snr(wave_speed: f64, snr_linear: f64, sample_rate: f64) -> f64 {
    wave_speed / (2.0 * snr_linear * sample_rate)
}

/// Non-linearity error (simplified model): integral non-linearity = k * x^2.
pub fn nonlinearity_error(position: f64, nonlinearity_coeff: f64) -> f64 {
    nonlinearity_coeff * position * position
}

// ---------------------------------------------------------------------------
// Magnetomechanical coupling
// ---------------------------------------------------------------------------

/// Magnetomechanical coupling coefficient squared:
/// k² = d² / (s * μ)
/// where d is piezomagnetic constant (m/A), s is compliance (1/Pa), μ is permeability (H/m).
pub fn coupling_coefficient_squared(
    piezomagnetic_d: f64,
    compliance: f64,
    permeability: f64,
) -> f64 {
    if compliance.abs() < 1e-30 || permeability.abs() < 1e-30 {
        return 0.0;
    }
    (piezomagnetic_d * piezomagnetic_d) / (compliance * permeability)
}

/// Coupling coefficient k = sqrt(k²).
pub fn coupling_coefficient(piezomagnetic_d: f64, compliance: f64, permeability: f64) -> f64 {
    coupling_coefficient_squared(piezomagnetic_d, compliance, permeability).sqrt()
}

/// Energy conversion efficiency from coupling: η ≈ k².
pub fn energy_conversion_efficiency(
    piezomagnetic_d: f64,
    compliance: f64,
    permeability: f64,
) -> f64 {
    coupling_coefficient_squared(piezomagnetic_d, compliance, permeability)
}

// ---------------------------------------------------------------------------
// Hysteresis modelling – simplified Jiles-Atherton
// ---------------------------------------------------------------------------

/// Langevin function: L(x) = coth(x) - 1/x.
/// For small x, uses Taylor expansion to avoid numerical issues.
fn langevin(x: f64) -> f64 {
    if x.abs() < 1e-4 {
        x / 3.0 - x.powi(3) / 45.0
    } else if x.abs() > 20.0 {
        // For large |x|, coth(x) → sign(x), so L(x) → sign(x) - 1/x
        x.signum() - 1.0 / x
    } else {
        // coth(x) = (e^x + e^-x) / (e^x - e^-x)
        let ex = x.exp();
        let emx = (-x).exp();
        (ex + emx) / (ex - emx) - 1.0 / x
    }
}

/// Parameters for simplified Jiles-Atherton hysteresis model.
#[derive(Debug, Clone)]
pub struct JilesAthertonParams {
    /// Saturation magnetisation (A/m).
    pub m_s: f64,
    /// Domain density parameter (A/m).
    pub a: f64,
    /// Domain coupling factor (dimensionless).
    pub alpha: f64,
    /// Pinning coefficient (A/m).
    pub k: f64,
    /// Reversibility coefficient (0..1).
    pub c: f64,
}

impl Default for JilesAthertonParams {
    fn default() -> Self {
        // Typical soft magnetic material parameters
        Self {
            m_s: 1.6e6,
            a: 1100.0,
            alpha: 1.6e-3,
            k: 400.0,
            c: 0.2,
        }
    }
}

/// Anhysteretic magnetisation M_an at effective field H_eff.
/// M_an = M_s * L(H_eff / a) = M_s * (coth(H_eff/a) - a/H_eff).
pub fn anhysteretic_magnetization(h_eff: f64, params: &JilesAthertonParams) -> f64 {
    if params.a.abs() < 1e-30 {
        return 0.0;
    }
    params.m_s * langevin(h_eff / params.a)
}

/// Generate a simplified B-H loop by stepping through the Jiles-Atherton model.
/// Returns (H_values, B_values) for one full cycle of applied field amplitude `h_max`
/// with `num_steps` per half cycle.
pub fn generate_bh_loop(
    params: &JilesAthertonParams,
    h_max: f64,
    num_steps: usize,
) -> (Vec<f64>, Vec<f64>) {
    let mu0 = 4.0 * PI * 1e-7;
    let total = 4 * num_steps;
    let mut h_vec = Vec::with_capacity(total);
    let mut b_vec = Vec::with_capacity(total);

    // Generate H trajectory: 0 → h_max → -h_max → h_max
    for i in 0..num_steps {
        h_vec.push(h_max * i as f64 / num_steps as f64);
    }
    for i in 0..2 * num_steps {
        h_vec.push(h_max - 2.0 * h_max * i as f64 / (2 * num_steps) as f64);
    }
    for i in 0..num_steps {
        h_vec.push(-h_max + 2.0 * h_max * i as f64 / (2 * num_steps) as f64);
    }

    let mut m = 0.0;
    for i in 0..h_vec.len() {
        let h = h_vec[i];
        let h_eff = h + params.alpha * m;
        let m_an = anhysteretic_magnetization(h_eff, params);

        // Simplified forward Euler step
        let delta_h = if i > 0 { h - h_vec[i - 1] } else { 0.0 };
        let delta_sign = if delta_h >= 0.0 { 1.0 } else { -1.0 };

        let m_an_minus_m = m_an - m;
        let denom = params.k * delta_sign - params.alpha * m_an_minus_m;
        if denom.abs() > 1e-30 {
            let dm_irr = m_an_minus_m / denom;
            let dm_rev = params.c * (m_an - m);
            let dm = (1.0 - params.c) * dm_irr + dm_rev;
            // Clamp to avoid runaway
            let dm_clamped = dm.max(-params.m_s * 0.1).min(params.m_s * 0.1);
            m += dm_clamped * delta_h;
        }
        // Clamp magnetisation
        m = m.max(-params.m_s).min(params.m_s);

        let b = mu0 * (h + m);
        b_vec.push(b);
    }

    (h_vec, b_vec)
}

// ---------------------------------------------------------------------------
// Frequency response / resonance
// ---------------------------------------------------------------------------

/// Mechanical resonance frequency of a waveguide (rod) of length L with free-free BCs:
/// f_n = n * v / (2 * L).
pub fn resonance_frequency(harmonic: usize, wave_speed: f64, length: f64) -> f64 {
    if length <= 0.0 {
        return 0.0;
    }
    harmonic as f64 * wave_speed / (2.0 * length)
}

/// Transfer function magnitude of a simple second-order mechanical resonator.
/// |H(f)| = 1 / sqrt((1 - (f/f0)²)² + (f/(Q*f0))²)
pub fn mechanical_transfer_function(freq: f64, resonance_freq: f64, q_factor: f64) -> f64 {
    if resonance_freq <= 0.0 {
        return 0.0;
    }
    let r = freq / resonance_freq;
    let r2 = r * r;
    1.0 / ((1.0 - r2).powi(2) + (r / q_factor).powi(2)).sqrt()
}

/// Bandwidth from Q factor: BW = f0 / Q.
pub fn bandwidth_from_q(resonance_freq: f64, q_factor: f64) -> f64 {
    if q_factor <= 0.0 {
        return f64::INFINITY;
    }
    resonance_freq / q_factor
}

/// Bandwidth limitation from wave propagation: max signal bandwidth ≈ v / (2 * δx_min).
pub fn propagation_bandwidth_limit(wave_speed: f64, min_resolution: f64) -> f64 {
    if min_resolution <= 0.0 {
        return f64::INFINITY;
    }
    wave_speed / (2.0 * min_resolution)
}

// ---------------------------------------------------------------------------
// Magnetostrictive position sensor (stateful processor)
// ---------------------------------------------------------------------------

/// Stateful magnetostrictive position sensor processor.
pub struct PositionSensor {
    config: WaveguideConfig,
    /// History of measured positions for velocity/acceleration computation.
    position_history: Vec<f64>,
    /// Maximum number of history entries.
    max_history: usize,
    /// Last measured temperature.
    temperature_k: f64,
}

impl PositionSensor {
    /// Create a new position sensor with given configuration.
    pub fn new(config: WaveguideConfig) -> Self {
        Self {
            temperature_k: config.ref_temperature_k,
            config,
            position_history: Vec::new(),
            max_history: 1024,
        }
    }

    /// Set current temperature for compensation.
    pub fn set_temperature(&mut self, temp_k: f64) {
        self.temperature_k = temp_k;
    }

    /// Process a torsional wave signal and return detected positions.
    pub fn process(&mut self, signal: &[f64], threshold: f64) -> Vec<f64> {
        let min_sep = 0.01; // 1 cm minimum separation
        let positions = detect_multi_magnet_positions(signal, &self.config, threshold, min_sep);

        // Apply temperature compensation
        let compensated: Vec<f64> = positions
            .iter()
            .map(|&p| temperature_compensate_position(p, self.temperature_k, &self.config))
            .collect();

        // Store first position in history (primary magnet)
        if let Some(&pos) = compensated.first() {
            self.position_history.push(pos);
            if self.position_history.len() > self.max_history {
                self.position_history.remove(0);
            }
        }

        compensated
    }

    /// Get velocity estimate from position history.
    pub fn velocity(&self, sample_period: f64) -> Option<f64> {
        let n = self.position_history.len();
        if n < 2 {
            return None;
        }
        let vel = velocity_from_position(&self.position_history, sample_period);
        vel.last().copied()
    }

    /// Get acceleration estimate from position history.
    pub fn acceleration(&self, sample_period: f64) -> Option<f64> {
        let n = self.position_history.len();
        if n < 3 {
            return None;
        }
        let acc = acceleration_from_position(&self.position_history, sample_period);
        acc.last().copied()
    }

    /// Current position (most recent measurement).
    pub fn current_position(&self) -> Option<f64> {
        self.position_history.last().copied()
    }

    /// Position history.
    pub fn history(&self) -> &[f64] {
        &self.position_history
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.position_history.clear();
    }
}

// ---------------------------------------------------------------------------
// Force/Torque sensor
// ---------------------------------------------------------------------------

/// Stateful force/torque sensor based on Villari effect.
pub struct ForceTorqueSensor {
    material: MaterialProperties,
    /// Cross-sectional area of the sensing element (m²).
    cross_section_area: f64,
    /// Diameter of sensing shaft for torque (m).
    shaft_diameter: f64,
}

impl ForceTorqueSensor {
    pub fn new(material: MaterialProperties, cross_section_area: f64, shaft_diameter: f64) -> Self {
        Self {
            material,
            cross_section_area,
            shaft_diameter,
        }
    }

    /// Estimate stress from a measured relative inductance change ΔL/L.
    pub fn stress_from_measurement(&self, delta_l_over_l: f64) -> f64 {
        stress_from_inductance_change(
            delta_l_over_l,
            self.material.youngs_modulus_pa,
            self.material.saturation_magnetostriction_ppm,
        )
    }

    /// Estimate force from inductance change.
    pub fn force_from_measurement(&self, delta_l_over_l: f64) -> f64 {
        let stress = self.stress_from_measurement(delta_l_over_l);
        force_from_stress(stress, self.cross_section_area)
    }

    /// Estimate torque from inductance change.
    pub fn torque_from_measurement(&self, delta_l_over_l: f64) -> f64 {
        let stress = self.stress_from_measurement(delta_l_over_l);
        let z_p = polar_section_modulus_solid(self.shaft_diameter);
        torque_from_stress(stress, z_p)
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-6;
    const EPS_COARSE: f64 = 1e-3;

    // --- Material presets ---

    #[test]
    fn test_terfenol_d_properties() {
        let m = MaterialProperties::terfenol_d();
        assert!((m.saturation_magnetostriction_ppm - 1600.0).abs() < EPS);
        assert!((m.youngs_modulus_pa - 25.0e9).abs() < 1.0);
        assert!((m.density_kgm3 - 9250.0).abs() < EPS);
        assert!((m.curie_temperature_k - 653.0).abs() < EPS);
    }

    #[test]
    fn test_galfenol_properties() {
        let m = MaterialProperties::galfenol();
        assert!((m.saturation_magnetostriction_ppm - 400.0).abs() < EPS);
        assert!((m.youngs_modulus_pa - 60.0e9).abs() < 1.0);
    }

    #[test]
    fn test_nickel_properties() {
        let m = MaterialProperties::nickel();
        assert!((m.saturation_magnetostriction_ppm - (-34.0)).abs() < EPS);
        assert!((m.permeability_relative - 600.0).abs() < EPS);
    }

    #[test]
    fn test_iron_properties() {
        let m = MaterialProperties::iron();
        assert!((m.saturation_magnetostriction_ppm - (-7.0)).abs() < EPS);
        assert!((m.permeability_relative - 5000.0).abs() < EPS);
        assert!((m.curie_temperature_k - 1043.0).abs() < EPS);
    }

    #[test]
    fn test_torsional_wave_velocity() {
        let m = MaterialProperties::nickel();
        let v = m.torsional_wave_velocity();
        // G = 200e9 / 2.6 ≈ 76.92e9, v = sqrt(76.92e9 / 8900) ≈ 2940 m/s
        assert!(v > 2500.0 && v < 3500.0, "v = {}", v);
    }

    #[test]
    fn test_longitudinal_wave_velocity() {
        let m = MaterialProperties::iron();
        let v = m.longitudinal_wave_velocity();
        // sqrt(211e9 / 7874) ≈ 5176 m/s
        assert!(v > 4500.0 && v < 6000.0, "v = {}", v);
    }

    // --- Waveguide config ---

    #[test]
    fn test_waveguide_default_config() {
        let c = WaveguideConfig::default();
        assert!((c.wave_speed_ms - 2800.0).abs() < EPS);
        assert!((c.dead_zone_m - 0.02).abs() < EPS);
    }

    #[test]
    fn test_position_from_tof() {
        let c = WaveguideConfig::default();
        // 100 microseconds round-trip → position = 2800 * 100e-6 / 2 = 0.14 m
        let pos = c.position_from_tof(100e-6);
        assert!((pos - 0.14).abs() < EPS);
    }

    #[test]
    fn test_position_resolution() {
        let c = WaveguideConfig::default();
        // resolution = 2800 / (2 * 10e6) = 0.00014 m = 0.14 mm
        let res = c.resolution();
        assert!((res - 0.00014).abs() < 1e-8);
    }

    #[test]
    fn test_wave_speed_at_temperature() {
        let c = WaveguideConfig::default();
        // At 10K above reference: v = 2800 * (1 + 1.5e-4 * 10) = 2800 * 1.0015 = 2804.2
        let v = c.wave_speed_at_temp(c.ref_temperature_k + 10.0);
        assert!((v - 2804.2).abs() < 0.1);
    }

    #[test]
    fn test_position_from_tof_compensated() {
        let c = WaveguideConfig::default();
        let pos_ref = c.position_from_tof(100e-6);
        let pos_comp = c.position_from_tof_compensated(100e-6, c.ref_temperature_k);
        assert!((pos_ref - pos_comp).abs() < EPS);
    }

    #[test]
    fn test_max_position() {
        let c = WaveguideConfig::default();
        assert!((c.max_position() - 0.98).abs() < EPS);
    }

    // --- Threshold crossing detection ---

    #[test]
    fn test_threshold_crossings_basic() {
        let signal = vec![0.0, 0.1, 0.5, 0.8, 0.3, 0.1, 0.0, 0.2, 0.9, 0.4];
        let crossings = threshold_crossings(&signal, 0.5, 1);
        assert_eq!(crossings.len(), 2);
        assert_eq!(crossings[0], 2);
        assert_eq!(crossings[1], 8);
    }

    #[test]
    fn test_threshold_crossings_min_gap() {
        let signal = vec![0.0, 0.6, 0.0, 0.6, 0.0];
        let crossings = threshold_crossings(&signal, 0.5, 5);
        // Second crossing at index 3 is within gap of 5 from index 1
        assert_eq!(crossings.len(), 1);
    }

    // --- Zero crossing detection ---

    #[test]
    fn test_zero_crossings_interpolated() {
        let signal = vec![-1.0, 1.0];
        let zc = zero_crossings_interpolated(&signal);
        assert_eq!(zc.len(), 1);
        assert!((zc[0] - 0.5).abs() < EPS);
    }

    #[test]
    fn test_zero_crossings_sine() {
        // sin at 1 kHz, sampled at 100 kHz
        let n = 200;
        let fs = 100000.0;
        let f = 1000.0;
        let signal: Vec<f64> = (0..n).map(|i| (2.0 * PI * f * i as f64 / fs).sin()).collect();
        let zc = zero_crossings_interpolated(&signal);
        // Expect 2 positive zero crossings per cycle, ~2 cycles = ~4 crossings (approximately)
        assert!(zc.len() >= 2 && zc.len() <= 6, "crossings: {}", zc.len());
    }

    // --- Parabolic interpolation ---

    #[test]
    fn test_parabolic_interpolation_centered() {
        // Symmetric: peak is exactly at centre
        let offset = parabolic_interpolation(0.5, 1.0, 0.5);
        assert!(offset.abs() < EPS);
    }

    #[test]
    fn test_parabolic_interpolation_offset() {
        // Asymmetric: y_next > y_prev means true peak is towards next sample → negative offset
        let offset = parabolic_interpolation(0.3, 1.0, 0.8);
        assert!(offset < 0.0); // True peak between k and k+1
        // Reverse asymmetry: y_prev > y_next → positive offset
        let offset2 = parabolic_interpolation(0.8, 1.0, 0.3);
        assert!(offset2 > 0.0);
    }

    // --- Envelope detection ---

    #[test]
    fn test_envelope_detect() {
        let signal = vec![0.0, 1.0, -1.0, 1.0, -1.0, 0.5, -0.5, 0.0];
        let env = envelope_detect(&signal, 3);
        assert_eq!(env.len(), signal.len());
        // Envelope should be non-negative
        for &e in &env {
            assert!(e >= 0.0);
        }
    }

    // --- Coherent averaging ---

    #[test]
    fn test_coherent_average() {
        let p1 = vec![1.0, 2.0, 3.0];
        let p2 = vec![3.0, 4.0, 5.0];
        let avg = coherent_average(&[p1, p2]);
        assert!((avg[0] - 2.0).abs() < EPS);
        assert!((avg[1] - 3.0).abs() < EPS);
        assert!((avg[2] - 4.0).abs() < EPS);
    }

    #[test]
    fn test_coherent_average_empty() {
        let avg = coherent_average(&[]);
        assert!(avg.is_empty());
    }

    // --- Multi-magnet detection ---

    #[test]
    fn test_multi_magnet_positions() {
        let config = WaveguideConfig::default();
        // Create a signal with two "pulses" at known sample indices
        let n = 10000;
        let mut signal = vec![0.0; n];
        // Magnet 1 at position 0.1 m → TOF = 2*0.1/2800 = 71.4 us → sample = 714
        let idx1 = (0.1 * 2.0 / config.wave_speed_ms * config.sample_rate_hz) as usize;
        // Magnet 2 at position 0.5 m → sample = 3571
        let idx2 = (0.5 * 2.0 / config.wave_speed_ms * config.sample_rate_hz) as usize;
        for i in idx1..idx1 + 10 {
            if i < n {
                signal[i] = 1.0;
            }
        }
        for i in idx2..idx2 + 10 {
            if i < n {
                signal[i] = 1.0;
            }
        }
        let positions = detect_multi_magnet_positions(&signal, &config, 0.5, 0.05);
        assert_eq!(positions.len(), 2, "positions: {:?}", positions);
        assert!((positions[0] - 0.1).abs() < 0.005);
        assert!((positions[1] - 0.5).abs() < 0.005);
    }

    // --- Velocity and acceleration ---

    #[test]
    fn test_velocity_from_position_linear() {
        // Linear motion: x = 2*t
        let positions = vec![0.0, 2.0, 4.0, 6.0, 8.0];
        let dt = 1.0;
        let vel = velocity_from_position(&positions, dt);
        assert_eq!(vel.len(), 5);
        for &v in &vel {
            assert!((v - 2.0).abs() < EPS);
        }
    }

    #[test]
    fn test_acceleration_from_position_constant() {
        // Constant acceleration: x = 0.5*a*t^2 with a=2
        let dt = 1.0;
        let positions: Vec<f64> = (0..5).map(|i| (i as f64).powi(2)).collect();
        let acc = acceleration_from_position(&positions, dt);
        // All accelerations should be 2.0
        for &a in &acc {
            assert!((a - 2.0).abs() < EPS);
        }
    }

    #[test]
    fn test_velocity_two_points() {
        let v = velocity_from_position(&[1.0, 3.0], 0.5);
        assert_eq!(v.len(), 1);
        assert!((v[0] - 4.0).abs() < EPS);
    }

    #[test]
    fn test_acceleration_too_few_points() {
        assert!(acceleration_from_position(&[1.0, 2.0], 1.0).is_empty());
    }

    // --- Villari effect ---

    #[test]
    fn test_villari_permeability_change() {
        // Terfenol-D: lambda_s = 1600 ppm, E = 25 GPa, stress = 10 MPa
        let delta = villari_permeability_change(10.0e6, 1600.0, 25.0e9);
        // 3 * (1600e-6) * 10e6 / 25e9 = 3 * 1.6e-3 * 10e6 / 25e9 = 1.92e-6
        assert!((delta - 1.92e-6).abs() < 1e-10);
    }

    #[test]
    fn test_force_from_stress() {
        let f = force_from_stress(100e6, 1e-4); // 100 MPa, 1 cm²
        assert!((f - 10000.0).abs() < EPS);
    }

    #[test]
    fn test_stress_from_inductance_change() {
        // Round-trip: stress → ΔL/L → stress
        let sigma = 50.0e6;
        let lambda_s = 1600.0; // ppm
        let e = 25.0e9;
        let dl = villari_permeability_change(sigma, lambda_s, e);
        let sigma_recovered = stress_from_inductance_change(dl, e, lambda_s);
        assert!((sigma_recovered - sigma).abs() / sigma < 1e-6);
    }

    #[test]
    fn test_torque_from_stress() {
        let z_p = polar_section_modulus_solid(0.02); // 20 mm diameter
        let t = torque_from_stress(100e6, z_p);
        // Z_p = pi * 0.02^3 / 16 ≈ 1.571e-6 m³
        // T = 100e6 * 1.571e-6 ≈ 157.1 Nm
        assert!(t > 100.0 && t < 200.0, "torque = {} Nm", t);
    }

    #[test]
    fn test_polar_section_modulus() {
        let z = polar_section_modulus_solid(0.1); // 100 mm diameter
        let expected = PI * 0.1_f64.powi(3) / 16.0;
        assert!((z - expected).abs() < 1e-10);
    }

    // --- Wiedemann effect ---

    #[test]
    fn test_wiedemann_torsional_strain() {
        let strain = wiedemann_torsional_strain(1000.0, 1000.0, 1600.0);
        // theta = 45°, sin(90°) = 1, strain = 1.5 * 1.6e-3 * 1 = 2.4e-3
        assert!((strain - 2.4e-3).abs() < 1e-6);
    }

    #[test]
    fn test_wiedemann_zero_circumferential() {
        let strain = wiedemann_torsional_strain(1000.0, 0.0, 1600.0);
        // theta = 0, sin(0) = 0
        assert!(strain.abs() < 1e-10);
    }

    // --- Bandpass filter ---

    #[test]
    fn test_bandpass_filter_passband() {
        let fs = 1e6;
        let fc = 200e3;
        let mut filt = BandpassFilter::new(fc, 5.0, fs);
        // Feed a tone at the centre frequency
        let n = 2000;
        let input: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * fc * i as f64 / fs).sin())
            .collect();
        let output = filt.filter(&input);
        // After settling, output should have significant amplitude
        let rms: f64 = output[n / 2..].iter().map(|x| x * x).sum::<f64>()
            / (n / 2) as f64;
        assert!(rms.sqrt() > 0.1, "RMS = {}", rms.sqrt());
    }

    #[test]
    fn test_bandpass_filter_stopband() {
        let fs = 1e6;
        let fc = 200e3;
        let mut filt = BandpassFilter::new(fc, 5.0, fs);
        // Feed a tone far from centre
        let f_stop = 10e3;
        let n = 2000;
        let input: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f_stop * i as f64 / fs).sin())
            .collect();
        let output = filt.filter(&input);
        let rms: f64 = output[n / 2..].iter().map(|x| x * x).sum::<f64>()
            / (n / 2) as f64;
        // Output should be heavily attenuated
        assert!(rms.sqrt() < 0.1, "RMS = {}", rms.sqrt());
    }

    // --- Temperature compensation ---

    #[test]
    fn test_temperature_compensate_position() {
        let config = WaveguideConfig::default();
        let pos = 0.5;
        // At reference temperature, no change
        let comp = temperature_compensate_position(pos, config.ref_temperature_k, &config);
        assert!((comp - pos).abs() < EPS);
    }

    // --- Error analysis ---

    #[test]
    fn test_position_uncertainty_from_jitter() {
        let sigma = position_uncertainty_from_jitter(2800.0, 1e-9);
        // 2800 * 1e-9 / 2 = 1.4e-6 m = 1.4 um
        assert!((sigma - 1.4e-6).abs() < 1e-10);
    }

    #[test]
    fn test_position_error_from_temperature() {
        let err = position_error_from_temperature(0.5, 1.5e-4, 10.0);
        // 0.5 * 1.5e-4 * 10 = 7.5e-4 m
        assert!((err - 7.5e-4).abs() < 1e-8);
    }

    #[test]
    fn test_total_position_uncertainty() {
        let total = total_position_uncertainty(2800.0, 1e-9, 0.5, 1.5e-4, 10.0);
        let j = position_uncertainty_from_jitter(2800.0, 1e-9);
        let t = position_error_from_temperature(0.5, 1.5e-4, 10.0);
        let expected = (j * j + t * t).sqrt();
        assert!((total - expected).abs() < 1e-10);
    }

    #[test]
    fn test_repeatability_from_snr() {
        let r = repeatability_from_snr(2800.0, 100.0, 10e6);
        // 2800 / (2 * 100 * 10e6) = 1.4e-6
        assert!((r - 1.4e-6).abs() < 1e-10);
    }

    #[test]
    fn test_nonlinearity_error() {
        let err = nonlinearity_error(0.5, 1e-3);
        assert!((err - 2.5e-4).abs() < 1e-10);
    }

    // --- Coupling coefficient ---

    #[test]
    fn test_coupling_coefficient() {
        // Typical Terfenol-D: d ≈ 1.5e-8 m/A, s = 1/E = 4e-11 1/Pa, mu ≈ 10 * mu0
        let d = 1.5e-8;
        let s = 1.0 / 25.0e9;
        let mu = 10.0 * 4.0 * PI * 1e-7;
        let k2 = coupling_coefficient_squared(d, s, mu);
        let k = coupling_coefficient(d, s, mu);
        assert!((k * k - k2).abs() < 1e-10);
        assert!(k > 0.0 && k < 1.0, "k = {}", k);
    }

    // --- Hysteresis ---

    #[test]
    fn test_langevin_small_x() {
        // For small x, L(x) ≈ x/3
        let l = langevin(1e-5);
        assert!((l - 1e-5 / 3.0).abs() < 1e-12);
    }

    #[test]
    fn test_langevin_large_x() {
        // For large x, L(x) → 1 - 1/x
        let l = langevin(100.0);
        // L(100) = 1 - 1/100 = 0.99
        assert!((l - 0.99).abs() < 1e-6);
        // Very large: practically 1
        let l2 = langevin(1e6);
        assert!((l2 - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_anhysteretic_magnetization() {
        let params = JilesAthertonParams::default();
        // At H_eff = 0, M_an should be 0
        let m0 = anhysteretic_magnetization(0.0, &params);
        assert!(m0.abs() < 1.0);
        // At large positive H_eff, M_an → M_s
        let m_large = anhysteretic_magnetization(1e6, &params);
        assert!((m_large - params.m_s).abs() / params.m_s < 0.01);
    }

    #[test]
    fn test_bh_loop_not_empty() {
        let params = JilesAthertonParams::default();
        let (h, b) = generate_bh_loop(&params, 5000.0, 50);
        assert!(!h.is_empty());
        assert_eq!(h.len(), b.len());
        // B should have both positive and negative values
        let has_pos = b.iter().any(|&v| v > 0.0);
        let has_neg = b.iter().any(|&v| v < 0.0);
        assert!(has_pos);
        assert!(has_neg);
    }

    // --- Frequency response ---

    #[test]
    fn test_resonance_frequency() {
        let f1 = resonance_frequency(1, 2800.0, 0.5);
        // f1 = 1 * 2800 / (2 * 0.5) = 2800 Hz
        assert!((f1 - 2800.0).abs() < EPS);
    }

    #[test]
    fn test_mechanical_transfer_at_resonance() {
        // At resonance, |H| = Q (for lightly damped system)
        let h = mechanical_transfer_function(1000.0, 1000.0, 50.0);
        assert!((h - 50.0).abs() < 1.0);
    }

    #[test]
    fn test_mechanical_transfer_dc() {
        // At DC (f=0), |H| = 1
        let h = mechanical_transfer_function(0.0, 1000.0, 50.0);
        assert!((h - 1.0).abs() < EPS);
    }

    #[test]
    fn test_bandwidth_from_q() {
        let bw = bandwidth_from_q(1000.0, 50.0);
        assert!((bw - 20.0).abs() < EPS);
    }

    #[test]
    fn test_propagation_bandwidth_limit() {
        let bw = propagation_bandwidth_limit(2800.0, 0.001);
        // 2800 / (2 * 0.001) = 1.4 MHz
        assert!((bw - 1.4e6).abs() < 1.0);
    }

    // --- Position sensor (stateful) ---

    #[test]
    fn test_position_sensor_basic() {
        let config = WaveguideConfig::default();
        let mut sensor = PositionSensor::new(config);
        assert!(sensor.current_position().is_none());
        assert!(sensor.history().is_empty());
    }

    #[test]
    fn test_position_sensor_process() {
        let config = WaveguideConfig::default();
        let mut sensor = PositionSensor::new(config.clone());

        // Create signal with pulse at position 0.3 m
        let n = 10000;
        let mut signal = vec![0.0; n];
        let idx = (0.3 * 2.0 / config.wave_speed_ms * config.sample_rate_hz) as usize;
        for i in idx..idx + 20 {
            if i < n {
                signal[i] = 1.0;
            }
        }
        let positions = sensor.process(&signal, 0.5);
        assert!(!positions.is_empty());
        assert!((positions[0] - 0.3).abs() < 0.01, "pos = {}", positions[0]);
    }

    #[test]
    fn test_position_sensor_velocity() {
        let config = WaveguideConfig::default();
        let mut sensor = PositionSensor::new(config);
        // Manually push positions representing constant velocity
        sensor.position_history = vec![0.0, 0.01, 0.02, 0.03, 0.04];
        let v = sensor.velocity(0.001);
        assert!(v.is_some());
        assert!((v.unwrap() - 10.0).abs() < EPS); // 0.01 / 0.001 = 10 m/s
    }

    #[test]
    fn test_position_sensor_reset() {
        let config = WaveguideConfig::default();
        let mut sensor = PositionSensor::new(config);
        sensor.position_history = vec![1.0, 2.0, 3.0];
        sensor.reset();
        assert!(sensor.history().is_empty());
    }

    // --- Force/Torque sensor ---

    #[test]
    fn test_force_torque_sensor_force() {
        let mat = MaterialProperties::terfenol_d();
        let area = 1e-4; // 1 cm²
        let sensor = ForceTorqueSensor::new(mat, area, 0.02);
        // Apply a known ΔL/L corresponding to a known stress
        let sigma = 10.0e6; // 10 MPa
        let dl = villari_permeability_change(sigma, 1600.0, 25.0e9);
        let f = sensor.force_from_measurement(dl);
        let expected = sigma * area;
        assert!((f - expected).abs() / expected < 1e-6);
    }

    #[test]
    fn test_force_torque_sensor_torque() {
        let mat = MaterialProperties::galfenol();
        let sensor = ForceTorqueSensor::new(mat, 1e-4, 0.02);
        let sigma = 50.0e6;
        let dl = villari_permeability_change(sigma, 400.0, 60.0e9);
        let torque = sensor.torque_from_measurement(dl);
        let z_p = polar_section_modulus_solid(0.02);
        let expected = sigma * z_p;
        assert!((torque - expected).abs() / expected < 1e-6);
    }

    // --- Find peak index ---

    #[test]
    fn test_find_peak_index() {
        let signal = vec![0.1, 0.5, 0.3, 0.9, 0.2];
        assert_eq!(find_peak_index(&signal), Some(3));
    }

    #[test]
    fn test_find_peak_index_negative() {
        let signal = vec![0.1, -2.0, 0.5];
        assert_eq!(find_peak_index(&signal), Some(1));
    }

    #[test]
    fn test_find_peak_index_empty() {
        let signal: Vec<f64> = vec![];
        assert_eq!(find_peak_index(&signal), None);
    }

    // --- Torsional wave velocity free function ---

    #[test]
    fn test_torsional_wave_velocity_fn() {
        let v = torsional_wave_velocity(80.0e9, 8000.0);
        // sqrt(80e9 / 8000) = sqrt(10e6) ≈ 3162 m/s
        assert!((v - 3162.27766).abs() < 0.01);
    }
}

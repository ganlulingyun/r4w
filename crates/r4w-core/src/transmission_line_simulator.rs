//! RF Transmission Line Simulator
//!
//! Models RF transmission lines computing S-parameters, impedance,
//! reflection coefficients, and time-domain reflectometry responses.
//!
//! Complex numbers are represented as `(f64, f64)` tuples of `(real, imaginary)`.
//!
//! # Example
//!
//! ```
//! use r4w_core::transmission_line_simulator::{TransmissionLine, LineType};
//!
//! // 50-ohm coaxial cable, 1 metre long, velocity factor 0.66
//! let line = TransmissionLine::new(LineType::Coaxial, 50.0, 1.0, 0.66, 0.02);
//!
//! // Reflection coefficient into a 75-ohm load
//! let gamma = line.reflection_coefficient((75.0, 0.0));
//! let mag = (gamma.0 * gamma.0 + gamma.1 * gamma.1).sqrt();
//! assert!(mag < 1.0, "reflection magnitude must be < 1 for passive load");
//!
//! // VSWR from that reflection
//! let vswr = line.vswr((75.0, 0.0));
//! assert!(vswr >= 1.0);
//!
//! // S-parameters at 1 GHz
//! let (s11, s21) = line.s_parameters(1.0e9, (75.0, 0.0));
//! // S21 magnitude should be less than unity due to loss
//! let s21_mag = (s21.0 * s21.0 + s21.1 * s21.1).sqrt();
//! assert!(s21_mag <= 1.0);
//! ```

use std::f64::consts::PI;

// ── Complex arithmetic helpers ────────────────────────────────────────

/// Complex type alias: `(real, imaginary)`.
pub type Cplx = (f64, f64);

#[inline]
fn c_add(a: Cplx, b: Cplx) -> Cplx {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: Cplx, b: Cplx) -> Cplx {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: Cplx, b: Cplx) -> Cplx {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_div(a: Cplx, b: Cplx) -> Cplx {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom == 0.0 {
        return (f64::INFINITY, 0.0);
    }
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

#[inline]
fn c_mag(a: Cplx) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

#[inline]
fn c_scale(s: f64, a: Cplx) -> Cplx {
    (s * a.0, s * a.1)
}

/// Complex exponential: e^(a + jb) = e^a * (cos b + j sin b).
#[inline]
fn c_exp(z: Cplx) -> Cplx {
    let r = z.0.exp();
    (r * z.1.cos(), r * z.1.sin())
}

/// Complex hyperbolic tangent: tanh(z) = (e^(2z) - 1) / (e^(2z) + 1).
#[inline]
fn c_tanh(z: Cplx) -> Cplx {
    let e2z = c_exp(c_scale(2.0, z));
    c_div(c_sub(e2z, (1.0, 0.0)), c_add(e2z, (1.0, 0.0)))
}

// ── Line type enumeration ─────────────────────────────────────────────

/// Physical type of the transmission line.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LineType {
    /// Coaxial cable (e.g., RG-58, RG-213, LMR-400).
    Coaxial,
    /// Microstrip on a PCB substrate.
    Microstrip,
    /// Stripline (embedded in dielectric).
    Stripline,
    /// Generic / user-specified parameters.
    Generic,
}

// ── Transmission line model ───────────────────────────────────────────

/// A lossy transmission line segment.
///
/// Models a uniform transmission line with a given characteristic impedance,
/// physical length, velocity factor, and attenuation constant.
#[derive(Debug, Clone)]
pub struct TransmissionLine {
    /// Physical type of the line.
    pub line_type: LineType,
    /// Characteristic impedance Z0 in ohms (real, resistive).
    pub z0: f64,
    /// Physical length in metres.
    pub length: f64,
    /// Velocity factor (0.0 .. 1.0], ratio of phase velocity to speed of light.
    pub velocity_factor: f64,
    /// Attenuation constant in nepers per metre at the reference frequency.
    /// A typical low-loss coax might be 0.01-0.05 Np/m at 1 GHz.
    pub alpha_per_metre: f64,
}

/// Speed of light in vacuum (m/s).
const C0: f64 = 299_792_458.0;

impl TransmissionLine {
    /// Create a new transmission line.
    ///
    /// # Arguments
    /// * `line_type` - physical construction type
    /// * `z0` - characteristic impedance (ohms)
    /// * `length` - physical length (metres)
    /// * `velocity_factor` - 0 < vf <= 1
    /// * `alpha_per_metre` - attenuation constant (Np/m)
    pub fn new(
        line_type: LineType,
        z0: f64,
        length: f64,
        velocity_factor: f64,
        alpha_per_metre: f64,
    ) -> Self {
        assert!(z0 > 0.0, "characteristic impedance must be positive");
        assert!(length >= 0.0, "length must be non-negative");
        assert!(
            velocity_factor > 0.0 && velocity_factor <= 1.0,
            "velocity factor must be in (0, 1]"
        );
        assert!(alpha_per_metre >= 0.0, "attenuation must be non-negative");
        Self {
            line_type,
            z0,
            length,
            velocity_factor,
            alpha_per_metre,
        }
    }

    /// Phase velocity of the line in m/s.
    #[inline]
    pub fn phase_velocity(&self) -> f64 {
        C0 * self.velocity_factor
    }

    /// One-way propagation delay through the line in seconds.
    #[inline]
    pub fn propagation_delay(&self) -> f64 {
        self.length / self.phase_velocity()
    }

    /// Propagation constant gamma = alpha + j*beta at a given frequency (Hz).
    ///
    /// * alpha (attenuation) is in Np/m.
    /// * beta (phase constant) = 2*pi*f / v_p in rad/m.
    pub fn propagation_constant(&self, freq_hz: f64) -> Cplx {
        let beta = 2.0 * PI * freq_hz / self.phase_velocity();
        (self.alpha_per_metre, beta)
    }

    /// Electrical length of the line in degrees at a given frequency.
    ///
    /// theta = beta * l  (converted from radians to degrees).
    pub fn electrical_length_deg(&self, freq_hz: f64) -> f64 {
        let beta = 2.0 * PI * freq_hz / self.phase_velocity();
        (beta * self.length).to_degrees()
    }

    /// Voltage reflection coefficient Gamma = (Z_L - Z0) / (Z_L + Z0).
    ///
    /// `z_load` is the complex load impedance.
    pub fn reflection_coefficient(&self, z_load: Cplx) -> Cplx {
        let z0: Cplx = (self.z0, 0.0);
        c_div(c_sub(z_load, z0), c_add(z_load, z0))
    }

    /// VSWR (Voltage Standing Wave Ratio) for a given load impedance.
    ///
    /// VSWR = (1 + |Gamma|) / (1 - |Gamma|).
    pub fn vswr(&self, z_load: Cplx) -> f64 {
        let gamma_mag = c_mag(self.reflection_coefficient(z_load));
        // Clamp to avoid division by zero for a total reflection
        let gamma_mag = gamma_mag.min(0.999_999);
        (1.0 + gamma_mag) / (1.0 - gamma_mag)
    }

    /// Return loss in dB for a given load impedance.
    ///
    /// RL = -20 * log10(|Gamma|). Returns positive infinity for a perfectly matched load.
    pub fn return_loss_db(&self, z_load: Cplx) -> f64 {
        let gamma_mag = c_mag(self.reflection_coefficient(z_load));
        if gamma_mag == 0.0 {
            return f64::INFINITY;
        }
        -20.0 * gamma_mag.log10()
    }

    /// Input impedance looking into the line terminated by `z_load`.
    ///
    /// Z_in = Z0 * (Z_L + Z0 * tanh(gamma*l)) / (Z0 + Z_L * tanh(gamma*l))
    pub fn input_impedance(&self, freq_hz: f64, z_load: Cplx) -> Cplx {
        let gamma = self.propagation_constant(freq_hz);
        let gamma_l = c_mul(gamma, (self.length, 0.0));
        let tanh_gl = c_tanh(gamma_l);
        let z0: Cplx = (self.z0, 0.0);

        let numerator = c_add(z_load, c_mul(z0, tanh_gl));
        let denominator = c_add(z0, c_mul(z_load, tanh_gl));

        c_mul(z0, c_div(numerator, denominator))
    }

    /// Two-port S-parameters (S11, S21) at a given frequency and load impedance.
    ///
    /// Uses the transfer-matrix approach for a lossy line:
    /// * S11 = reflection at the input looking through the line
    /// * S21 = transmitted signal accounting for loss and phase shift
    ///
    /// The reference impedance is taken to be Z0 of the line itself.
    pub fn s_parameters(&self, freq_hz: f64, z_load: Cplx) -> (Cplx, Cplx) {
        let z_in = self.input_impedance(freq_hz, z_load);
        let z0: Cplx = (self.z0, 0.0);

        // S11 = (Z_in - Z0) / (Z_in + Z0)
        let s11 = c_div(c_sub(z_in, z0), c_add(z_in, z0));

        // S21: transmission through the line
        // For a line of propagation constant gamma and length l:
        //   S21 = e^{-gamma*l} * (1 - Gamma_L^2) / (1 - Gamma_L^2 * e^{-2*gamma*l})
        // where Gamma_L is the load reflection coefficient.
        let gamma = self.propagation_constant(freq_hz);
        let gamma_l = c_mul(gamma, (self.length, 0.0));
        let exp_neg_gl = c_exp(c_scale(-1.0, gamma_l));
        let exp_neg_2gl = c_exp(c_scale(-2.0, gamma_l));

        let gamma_l_load = self.reflection_coefficient(z_load);
        let gamma_l_sq = c_mul(gamma_l_load, gamma_l_load);

        let numerator = c_mul(exp_neg_gl, c_sub((1.0, 0.0), gamma_l_sq));
        let denominator = c_sub((1.0, 0.0), c_mul(gamma_l_sq, exp_neg_2gl));

        let s21 = c_div(numerator, denominator);

        (s11, s21)
    }

    /// Time-domain reflectometry (TDR) impulse response.
    ///
    /// Returns a vector of `(time_seconds, amplitude)` pairs representing the
    /// reflected signal at the input of the line when a unit impulse is sent.
    ///
    /// Models the initial reflection at the source/line interface (if source
    /// impedance differs from Z0), the round-trip reflection from the load,
    /// and successive bounces with exponential decay due to line loss.
    ///
    /// * `z_source` - source impedance (complex)
    /// * `z_load`   - load impedance (complex)
    /// * `num_bounces` - number of round-trip reflections to compute
    pub fn tdr_response(
        &self,
        z_source: Cplx,
        z_load: Cplx,
        num_bounces: usize,
    ) -> Vec<(f64, f64)> {
        let round_trip = 2.0 * self.propagation_delay();
        let round_trip_loss = (-2.0 * self.alpha_per_metre * self.length).exp();

        // Reflection coefficients at each end
        let gamma_source = {
            let z0: Cplx = (self.z0, 0.0);
            c_div(c_sub(z_source, z0), c_add(z_source, z0))
        };
        let gamma_load = self.reflection_coefficient(z_load);

        let gamma_s_real = gamma_source.0; // use real part for sign-preserving TDR
        let gamma_l_real = gamma_load.0;

        let mut response = Vec::with_capacity(num_bounces + 1);

        // Initial reflection at source-line junction (t = 0)
        response.push((0.0, gamma_s_real));

        // Subsequent bounces
        let mut amplitude = (1.0 - gamma_s_real * gamma_s_real) * gamma_l_real;
        for n in 0..num_bounces {
            let t = round_trip * (n as f64 + 1.0);
            let loss = round_trip_loss.powi((n as i32) + 1);
            let bounce_factor = if n == 0 {
                1.0
            } else {
                (gamma_s_real * gamma_l_real).powi(n as i32)
            };
            response.push((t, amplitude * loss * bounce_factor));
        }

        response
    }

    /// Convert a complex impedance to normalised Smith chart coordinates.
    ///
    /// Returns `(u, v)` where:
    ///   Gamma = (z_norm - 1) / (z_norm + 1)
    ///   u = Re(Gamma), v = Im(Gamma)
    ///
    /// The point lies within the unit circle on the Smith chart.
    pub fn smith_chart_coords(&self, z: Cplx) -> (f64, f64) {
        let z_norm = c_div(z, (self.z0, 0.0));
        let gamma = c_div(c_sub(z_norm, (1.0, 0.0)), c_add(z_norm, (1.0, 0.0)));
        (gamma.0, gamma.1)
    }
}

// ── Tests ─────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-9;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn c_approx_eq(a: Cplx, b: Cplx, tol: f64) -> bool {
        approx_eq(a.0, b.0, tol) && approx_eq(a.1, b.1, tol)
    }

    #[test]
    fn test_matched_load_zero_reflection() {
        let line = TransmissionLine::new(LineType::Coaxial, 50.0, 1.0, 0.66, 0.0);
        let gamma = line.reflection_coefficient((50.0, 0.0));
        assert!(c_approx_eq(gamma, (0.0, 0.0), TOL), "matched load should have zero reflection");
    }

    #[test]
    fn test_open_circuit_reflection() {
        let line = TransmissionLine::new(LineType::Generic, 50.0, 1.0, 1.0, 0.0);
        // Open circuit: Z_L -> infinity, Gamma -> +1
        let gamma = line.reflection_coefficient((1e15, 0.0));
        assert!(
            approx_eq(c_mag(gamma), 1.0, 1e-6),
            "open circuit should have |Gamma| ~ 1, got {}",
            c_mag(gamma)
        );
    }

    #[test]
    fn test_short_circuit_reflection() {
        let line = TransmissionLine::new(LineType::Generic, 50.0, 1.0, 1.0, 0.0);
        // Short circuit: Z_L = 0, Gamma = -1
        let gamma = line.reflection_coefficient((0.0, 0.0));
        assert!(
            c_approx_eq(gamma, (-1.0, 0.0), TOL),
            "short circuit should have Gamma = -1, got {:?}",
            gamma
        );
    }

    #[test]
    fn test_vswr_matched() {
        let line = TransmissionLine::new(LineType::Coaxial, 50.0, 1.0, 0.66, 0.0);
        let v = line.vswr((50.0, 0.0));
        assert!(
            approx_eq(v, 1.0, 1e-6),
            "matched load VSWR should be 1.0, got {}",
            v
        );
    }

    #[test]
    fn test_vswr_75_ohm_load() {
        let line = TransmissionLine::new(LineType::Coaxial, 50.0, 1.0, 0.66, 0.0);
        let v = line.vswr((75.0, 0.0));
        // Gamma = (75-50)/(75+50) = 25/125 = 0.2
        // VSWR = (1+0.2)/(1-0.2) = 1.5
        assert!(
            approx_eq(v, 1.5, 1e-9),
            "VSWR for 75 ohm load on 50 ohm line should be 1.5, got {}",
            v
        );
    }

    #[test]
    fn test_return_loss_matched() {
        let line = TransmissionLine::new(LineType::Coaxial, 50.0, 1.0, 0.66, 0.0);
        let rl = line.return_loss_db((50.0, 0.0));
        assert!(
            rl.is_infinite() && rl > 0.0,
            "matched load return loss should be +inf, got {}",
            rl
        );
    }

    #[test]
    fn test_return_loss_75_ohm() {
        let line = TransmissionLine::new(LineType::Coaxial, 50.0, 1.0, 0.66, 0.0);
        let rl = line.return_loss_db((75.0, 0.0));
        // |Gamma| = 0.2 -> RL = -20*log10(0.2) ~ 13.979 dB
        let expected = -20.0 * 0.2_f64.log10();
        assert!(
            approx_eq(rl, expected, 1e-6),
            "return loss should be {:.3} dB, got {:.3}",
            expected,
            rl
        );
    }

    #[test]
    fn test_input_impedance_quarter_wave_lossless() {
        // A lossless quarter-wave transformer: Z_in = Z0^2 / Z_L
        let z0 = 50.0;
        let freq = 1.0e9;
        let vf = 1.0;
        let wavelength = C0 * vf / freq;
        let quarter_wave = wavelength / 4.0;

        let line = TransmissionLine::new(LineType::Generic, z0, quarter_wave, vf, 0.0);
        let z_load: Cplx = (100.0, 0.0);
        let z_in = line.input_impedance(freq, z_load);

        // Expected: Z0^2 / Z_L = 2500 / 100 = 25 ohm
        assert!(
            approx_eq(z_in.0, 25.0, 1e-4),
            "quarter-wave transformer Z_in real should be 25 ohm, got {:.6}",
            z_in.0
        );
        assert!(
            approx_eq(z_in.1, 0.0, 1e-4),
            "quarter-wave transformer Z_in imag should be ~0, got {:.6}",
            z_in.1
        );
    }

    #[test]
    fn test_input_impedance_half_wave_lossless() {
        // A lossless half-wave line: Z_in = Z_L (impedance repeats)
        let z0 = 50.0;
        let freq = 1.0e9;
        let vf = 1.0;
        let wavelength = C0 * vf / freq;
        let half_wave = wavelength / 2.0;

        let line = TransmissionLine::new(LineType::Generic, z0, half_wave, vf, 0.0);
        let z_load: Cplx = (75.0, 30.0);
        let z_in = line.input_impedance(freq, z_load);

        assert!(
            c_approx_eq(z_in, z_load, 1e-3),
            "half-wave line Z_in should equal Z_L, got {:?} expected {:?}",
            z_in,
            z_load
        );
    }

    #[test]
    fn test_electrical_length() {
        let freq = 1.0e9;
        let vf = 0.66;
        let wavelength = C0 * vf / freq;

        // One full wavelength -> 360 degrees
        let line = TransmissionLine::new(LineType::Coaxial, 50.0, wavelength, vf, 0.0);
        let deg = line.electrical_length_deg(freq);
        assert!(
            approx_eq(deg, 360.0, 1e-4),
            "one wavelength should be 360 degrees, got {}",
            deg
        );
    }

    #[test]
    fn test_s_parameters_matched_lossless() {
        // Matched, lossless line: S11 ~ 0, |S21| ~ 1
        let line = TransmissionLine::new(LineType::Generic, 50.0, 1.0, 1.0, 0.0);
        let z_load: Cplx = (50.0, 0.0);
        let (s11, s21) = line.s_parameters(1.0e9, z_load);

        assert!(
            c_mag(s11) < 1e-9,
            "matched lossless S11 magnitude should be ~0, got {}",
            c_mag(s11)
        );
        assert!(
            approx_eq(c_mag(s21), 1.0, 1e-9),
            "matched lossless |S21| should be ~1, got {}",
            c_mag(s21)
        );
    }

    #[test]
    fn test_s21_lossy_line() {
        // A lossy line should have |S21| < 1 even for a matched load
        let line = TransmissionLine::new(LineType::Coaxial, 50.0, 1.0, 0.66, 0.05);
        let z_load: Cplx = (50.0, 0.0);
        let (_s11, s21) = line.s_parameters(1.0e9, z_load);

        let s21_mag = c_mag(s21);
        let expected = (-0.05_f64 * 1.0).exp(); // e^{-alpha*l}
        assert!(
            approx_eq(s21_mag, expected, 1e-6),
            "|S21| for lossy matched line should be {:.6}, got {:.6}",
            expected,
            s21_mag
        );
    }

    #[test]
    fn test_smith_chart_center() {
        // Z = Z0 -> normalised = 1+j0 -> Gamma = 0 -> center of Smith chart
        let line = TransmissionLine::new(LineType::Generic, 50.0, 1.0, 1.0, 0.0);
        let (u, v) = line.smith_chart_coords((50.0, 0.0));
        assert!(
            approx_eq(u, 0.0, TOL) && approx_eq(v, 0.0, TOL),
            "matched impedance should map to Smith chart centre, got ({}, {})",
            u,
            v
        );
    }

    #[test]
    fn test_smith_chart_open_short() {
        let line = TransmissionLine::new(LineType::Generic, 50.0, 1.0, 1.0, 0.0);

        // Open circuit (large Z) -> right edge (1, 0)
        let (u, _v) = line.smith_chart_coords((1e15, 0.0));
        assert!(
            approx_eq(u, 1.0, 1e-6),
            "open circuit should be at u~1, got {}",
            u
        );

        // Short circuit (Z=0) -> left edge (-1, 0)
        let (u, v) = line.smith_chart_coords((0.0, 0.0));
        assert!(
            approx_eq(u, -1.0, TOL) && approx_eq(v, 0.0, TOL),
            "short circuit should be at (-1, 0), got ({}, {})",
            u,
            v
        );
    }

    #[test]
    fn test_tdr_response_matched() {
        // Matched source and load: initial reflection = 0, load bounce = 0
        let line = TransmissionLine::new(LineType::Coaxial, 50.0, 1.0, 0.66, 0.0);
        let resp = line.tdr_response((50.0, 0.0), (50.0, 0.0), 3);

        // First entry is source reflection (should be 0 for matched source)
        assert!(
            approx_eq(resp[0].1, 0.0, TOL),
            "matched source reflection should be 0, got {}",
            resp[0].1
        );
        // All subsequent bounces should also be 0
        for (i, (_t, amp)) in resp.iter().enumerate().skip(1) {
            assert!(
                approx_eq(*amp, 0.0, TOL),
                "bounce {} amplitude should be 0, got {}",
                i,
                amp
            );
        }
    }

    #[test]
    fn test_tdr_response_timing() {
        let vf = 0.66;
        let length = 10.0;
        let line = TransmissionLine::new(LineType::Coaxial, 50.0, length, vf, 0.0);
        let resp = line.tdr_response((50.0, 0.0), (75.0, 0.0), 3);

        let expected_rt = 2.0 * length / (C0 * vf);
        // Second entry should be at one round-trip time
        assert!(
            approx_eq(resp[1].0, expected_rt, 1e-15),
            "first load bounce should be at t={:.12e}, got {:.12e}",
            expected_rt,
            resp[1].0
        );
    }

    #[test]
    fn test_propagation_delay() {
        let line = TransmissionLine::new(LineType::Coaxial, 50.0, 100.0, 0.66, 0.0);
        let delay = line.propagation_delay();
        let expected = 100.0 / (C0 * 0.66);
        assert!(
            approx_eq(delay, expected, 1e-15),
            "propagation delay should be {:.12e}, got {:.12e}",
            expected,
            delay
        );
    }

    #[test]
    fn test_propagation_constant() {
        let line = TransmissionLine::new(LineType::Generic, 50.0, 1.0, 1.0, 0.1);
        let gamma = line.propagation_constant(1.0e9);
        // alpha = 0.1
        assert!(approx_eq(gamma.0, 0.1, TOL), "alpha should be 0.1, got {}", gamma.0);
        // beta = 2*pi * 1e9 / c
        let expected_beta = 2.0 * PI * 1.0e9 / C0;
        assert!(
            approx_eq(gamma.1, expected_beta, 1e-3),
            "beta should be {:.3}, got {:.3}",
            expected_beta,
            gamma.1
        );
    }

    #[test]
    fn test_line_type_enum() {
        let line = TransmissionLine::new(LineType::Microstrip, 75.0, 0.05, 0.55, 0.1);
        assert_eq!(line.line_type, LineType::Microstrip);
        assert_eq!(line.z0, 75.0);

        let line2 = TransmissionLine::new(LineType::Stripline, 100.0, 0.1, 0.5, 0.05);
        assert_eq!(line2.line_type, LineType::Stripline);
    }
}

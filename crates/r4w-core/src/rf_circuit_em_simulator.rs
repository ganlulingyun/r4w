//! RF Circuit Electromagnetic Simulator
//!
//! Simulates RF circuits using transmission line theory and S-parameter analysis.
//! Provides S-parameter computation for basic elements (resistor, capacitor, inductor,
//! transmission line), cascade (ABCD matrix) chain analysis, impedance matching network
//! design (L-network, Pi-network, T-network), Smith chart calculations, VSWR/return loss
//! computation, stability factor (K and mu), noise figure cascade (Friis formula), and
//! frequency response sweeps.
//!
//! All complex numbers are represented as `(f64, f64)` tuples where the first element
//! is the real part and the second is the imaginary part.
//!
//! # Example
//!
//! ```
//! use r4w_core::rf_circuit_em_simulator::*;
//!
//! // Compute S-parameters for a 50-ohm series resistor in a 50-ohm system
//! let z = (50.0, 0.0); // 50 ohm resistor (pure real)
//! let s = s_params_series_impedance(z, 50.0);
//!
//! // s11 should be non-zero (impedance mismatch effect)
//! let s11_mag = complex_mag(s.s11);
//! assert!(s11_mag > 0.0);
//!
//! // Compute VSWR from reflection coefficient magnitude
//! let gamma_mag = complex_mag(s.s11);
//! let standing_wave_ratio = vswr(gamma_mag);
//! assert!(standing_wave_ratio >= 1.0);
//! ```

use std::f64::consts::PI;

// ─── Complex arithmetic helpers ─────────────────────────────────────────────

/// Add two complex numbers.
pub fn complex_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Subtract complex `b` from complex `a`.
pub fn complex_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Multiply two complex numbers.
pub fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Divide complex `a` by complex `b`.
///
/// # Panics
///
/// Panics if `b` is zero.
pub fn complex_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = b.0 * b.0 + b.1 * b.1;
    assert!(denom > 0.0, "Division by zero complex number");
    (
        (a.0 * b.0 + a.1 * b.1) / denom,
        (a.1 * b.0 - a.0 * b.1) / denom,
    )
}

/// Magnitude of a complex number.
pub fn complex_mag(z: (f64, f64)) -> f64 {
    (z.0 * z.0 + z.1 * z.1).sqrt()
}

/// Conjugate of a complex number.
pub fn complex_conj(z: (f64, f64)) -> (f64, f64) {
    (z.0, -z.1)
}

/// Scale a complex number by a real scalar.
pub fn complex_scale(z: (f64, f64), s: f64) -> (f64, f64) {
    (z.0 * s, z.1 * s)
}

/// Negate a complex number.
pub fn complex_neg(z: (f64, f64)) -> (f64, f64) {
    (-z.0, -z.1)
}

/// Complex number from polar form (magnitude, angle in radians).
pub fn complex_from_polar(mag: f64, angle: f64) -> (f64, f64) {
    (mag * angle.cos(), mag * angle.sin())
}

/// Determinant of a 2x2 complex matrix [[a,b],[c,d]] = ad - bc.
fn complex_det_2x2(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> (f64, f64) {
    complex_sub(complex_mul(a, d), complex_mul(b, c))
}

// ─── Core types ─────────────────────────────────────────────────────────────

/// Two-port S-parameter matrix.
///
/// Each element is a complex number represented as `(f64, f64)`.
#[derive(Debug, Clone, Copy)]
pub struct SParams {
    /// Input reflection coefficient.
    pub s11: (f64, f64),
    /// Reverse transmission coefficient.
    pub s12: (f64, f64),
    /// Forward transmission coefficient.
    pub s21: (f64, f64),
    /// Output reflection coefficient.
    pub s22: (f64, f64),
}

/// Two-port ABCD (transmission) matrix.
///
/// Used for cascading two-port networks via matrix multiplication.
/// Each element is a complex number represented as `(f64, f64)`.
#[derive(Debug, Clone, Copy)]
pub struct AbcdMatrix {
    /// Voltage ratio (dimensionless).
    pub a: (f64, f64),
    /// Transfer impedance (ohms).
    pub b: (f64, f64),
    /// Transfer admittance (siemens).
    pub c: (f64, f64),
    /// Current ratio (dimensionless).
    pub d: (f64, f64),
}

/// Main RF circuit simulator.
///
/// Holds a reference impedance `z0` and provides methods for building and
/// analyzing RF circuits.
#[derive(Debug, Clone)]
pub struct RfCircuitSim {
    /// Reference impedance in ohms (typically 50.0).
    pub z0: f64,
    /// Cascade of ABCD stages.
    pub stages: Vec<AbcdMatrix>,
}

impl RfCircuitSim {
    /// Create a new simulator with the given reference impedance.
    pub fn new(z0: f64) -> Self {
        Self {
            z0,
            stages: Vec::new(),
        }
    }

    /// Add a two-port stage (as an ABCD matrix) to the cascade.
    pub fn add_stage(&mut self, abcd: AbcdMatrix) {
        self.stages.push(abcd);
    }

    /// Compute the overall ABCD matrix for all cascaded stages.
    pub fn cascade(&self) -> AbcdMatrix {
        cascade_abcd(&self.stages)
    }

    /// Compute the overall S-parameters for all cascaded stages.
    pub fn s_params(&self) -> SParams {
        let abcd = self.cascade();
        abcd_to_s(&abcd, self.z0)
    }

    /// Perform a frequency sweep, computing S-parameters at each frequency.
    ///
    /// The `circuit_fn` closure receives a frequency in Hz and returns the
    /// S-parameters at that frequency.
    pub fn sweep(&self, circuit_fn: &dyn Fn(f64) -> SParams, freqs: &[f64]) -> Vec<SParams> {
        frequency_sweep(circuit_fn, freqs)
    }
}

// ─── S-parameter computation for basic elements ────────────────────────────

/// Compute S-parameters for a series impedance element.
///
/// Models a two-port network with impedance `z` in series between the ports,
/// referenced to characteristic impedance `z0`.
///
/// The S-parameter matrix for a series impedance Z in a Z0 system is:
/// - S11 = S22 = Z / (Z + 2*Z0)
/// - S12 = S21 = 2*Z0 / (Z + 2*Z0)
pub fn s_params_series_impedance(z: (f64, f64), z0: f64) -> SParams {
    let two_z0 = (2.0 * z0, 0.0);
    let denom = complex_add(z, two_z0);
    let s11 = complex_div(z, denom);
    let s21 = complex_div(two_z0, denom);
    SParams {
        s11,
        s12: s21,
        s21,
        s22: s11,
    }
}

/// Compute S-parameters for a shunt admittance element.
///
/// Models a two-port network with admittance `y` connected between the
/// signal line and ground, referenced to characteristic impedance `z0`.
///
/// The S-parameter matrix for a shunt admittance Y in a Z0 system is:
/// - S11 = S22 = -Y / (Y + 2*Y0) where Y0 = 1/Z0
/// - S12 = S21 = 2*Y0 / (Y + 2*Y0)
pub fn s_params_shunt_admittance(y: (f64, f64), z0: f64) -> SParams {
    let y0 = 1.0 / z0;
    let two_y0 = (2.0 * y0, 0.0);
    let denom = complex_add(y, two_y0);
    let s11 = complex_neg(complex_div(y, denom));
    let s21 = complex_div(two_y0, denom);
    SParams {
        s11,
        s12: s21,
        s21,
        s22: s11,
    }
}

/// Compute S-parameters for a transmission line segment.
///
/// Models a lossless transmission line with characteristic impedance `z_line`,
/// electrical length `electrical_length_rad` (in radians), referenced to `z0`.
///
/// Uses the ABCD matrix representation of a transmission line:
/// - A = cos(θ)
/// - B = j * Z_line * sin(θ)
/// - C = j * sin(θ) / Z_line
/// - D = cos(θ)
///
/// Then converts to S-parameters.
pub fn s_params_transmission_line(z_line: f64, electrical_length_rad: f64, z0: f64) -> SParams {
    let cos_theta = electrical_length_rad.cos();
    let sin_theta = electrical_length_rad.sin();

    let abcd = AbcdMatrix {
        a: (cos_theta, 0.0),
        b: (0.0, z_line * sin_theta),
        c: (0.0, sin_theta / z_line),
        d: (cos_theta, 0.0),
    };

    abcd_to_s(&abcd, z0)
}

// ─── S-parameter / ABCD conversions ────────────────────────────────────────

/// Convert S-parameters to an ABCD matrix.
///
/// Uses the standard conversion formulas referenced to impedance `z0`:
/// - A = ((1+S11)(1-S22) + S12*S21) / (2*S21)
/// - B = Z0 * ((1+S11)(1+S22) - S12*S21) / (2*S21)
/// - C = (1/Z0) * ((1-S11)(1-S22) - S12*S21) / (2*S21)
/// - D = ((1-S11)(1+S22) + S12*S21) / (2*S21)
pub fn s_to_abcd(s: &SParams, z0: f64) -> AbcdMatrix {
    let one = (1.0, 0.0);
    let two_s21 = complex_scale(s.s21, 2.0);
    let s12_s21 = complex_mul(s.s12, s.s21);

    let one_plus_s11 = complex_add(one, s.s11);
    let one_minus_s11 = complex_sub(one, s.s11);
    let one_plus_s22 = complex_add(one, s.s22);
    let one_minus_s22 = complex_sub(one, s.s22);

    let a_num = complex_add(complex_mul(one_plus_s11, one_minus_s22), s12_s21);
    let a = complex_div(a_num, two_s21);

    let b_num = complex_sub(complex_mul(one_plus_s11, one_plus_s22), s12_s21);
    let b = complex_scale(complex_div(b_num, two_s21), z0);

    let c_num = complex_sub(complex_mul(one_minus_s11, one_minus_s22), s12_s21);
    let c = complex_scale(complex_div(c_num, two_s21), 1.0 / z0);

    let d_num = complex_add(complex_mul(one_minus_s11, one_plus_s22), s12_s21);
    let d = complex_div(d_num, two_s21);

    AbcdMatrix { a, b, c, d }
}

/// Convert an ABCD matrix to S-parameters.
///
/// Uses the standard conversion formulas referenced to impedance `z0`:
/// - S11 = (A + B/Z0 - C*Z0 - D) / (A + B/Z0 + C*Z0 + D)
/// - S12 = 2*(AD - BC) / (A + B/Z0 + C*Z0 + D)
/// - S21 = 2 / (A + B/Z0 + C*Z0 + D)
/// - S22 = (-A + B/Z0 - C*Z0 + D) / (A + B/Z0 + C*Z0 + D)
pub fn abcd_to_s(abcd: &AbcdMatrix, z0: f64) -> SParams {
    let b_over_z0 = complex_scale(abcd.b, 1.0 / z0);
    let c_times_z0 = complex_scale(abcd.c, z0);

    // Common denominator: A + B/Z0 + C*Z0 + D
    let denom = complex_add(
        complex_add(abcd.a, b_over_z0),
        complex_add(c_times_z0, abcd.d),
    );

    // S11 = (A + B/Z0 - C*Z0 - D) / denom
    let s11_num = complex_sub(
        complex_add(abcd.a, b_over_z0),
        complex_add(c_times_z0, abcd.d),
    );
    let s11 = complex_div(s11_num, denom);

    // S12 = 2*(AD - BC) / denom
    let det = complex_det_2x2(abcd.a, abcd.b, abcd.c, abcd.d);
    let s12 = complex_div(complex_scale(det, 2.0), denom);

    // S21 = 2 / denom
    let s21 = complex_div((2.0, 0.0), denom);

    // S22 = (-A + B/Z0 - C*Z0 + D) / denom
    let s22_num = complex_add(
        complex_sub(b_over_z0, abcd.a),
        complex_sub(abcd.d, c_times_z0),
    );
    let s22 = complex_div(s22_num, denom);

    SParams { s11, s12, s21, s22 }
}

/// Cascade (chain-multiply) a sequence of ABCD matrices.
///
/// Returns the identity matrix if the slice is empty. For a single element,
/// returns that element. For multiple elements, multiplies them left to right.
pub fn cascade_abcd(stages: &[AbcdMatrix]) -> AbcdMatrix {
    if stages.is_empty() {
        return AbcdMatrix {
            a: (1.0, 0.0),
            b: (0.0, 0.0),
            c: (0.0, 0.0),
            d: (1.0, 0.0),
        };
    }

    let mut result = stages[0];
    for stage in &stages[1..] {
        result = abcd_multiply(&result, stage);
    }
    result
}

/// Multiply two ABCD matrices.
fn abcd_multiply(m1: &AbcdMatrix, m2: &AbcdMatrix) -> AbcdMatrix {
    AbcdMatrix {
        a: complex_add(complex_mul(m1.a, m2.a), complex_mul(m1.b, m2.c)),
        b: complex_add(complex_mul(m1.a, m2.b), complex_mul(m1.b, m2.d)),
        c: complex_add(complex_mul(m1.c, m2.a), complex_mul(m1.d, m2.c)),
        d: complex_add(complex_mul(m1.c, m2.b), complex_mul(m1.d, m2.d)),
    }
}

// ─── Smith chart / impedance calculations ──────────────────────────────────

/// Compute the reflection coefficient (Gamma) from impedance and reference impedance.
///
/// Γ = (Z - Z0) / (Z + Z0)
pub fn impedance_to_reflection(z: (f64, f64), z0: f64) -> (f64, f64) {
    let z0_c = (z0, 0.0);
    complex_div(complex_sub(z, z0_c), complex_add(z, z0_c))
}

/// Compute impedance from reflection coefficient and reference impedance.
///
/// Z = Z0 * (1 + Γ) / (1 - Γ)
pub fn reflection_to_impedance(gamma: (f64, f64), z0: f64) -> (f64, f64) {
    let one = (1.0, 0.0);
    complex_scale(
        complex_div(complex_add(one, gamma), complex_sub(one, gamma)),
        z0,
    )
}

/// Compute VSWR (Voltage Standing Wave Ratio) from the magnitude of Gamma.
///
/// VSWR = (1 + |Γ|) / (1 - |Γ|)
///
/// Returns `f64::INFINITY` if `gamma_mag >= 1.0`.
pub fn vswr(gamma_mag: f64) -> f64 {
    if gamma_mag >= 1.0 {
        return f64::INFINITY;
    }
    (1.0 + gamma_mag) / (1.0 - gamma_mag)
}

/// Compute return loss in dB from the magnitude of Gamma.
///
/// Return Loss = -20 * log10(|Γ|)
///
/// Returns `f64::INFINITY` if `gamma_mag` is zero (perfect match).
pub fn return_loss_db(gamma_mag: f64) -> f64 {
    if gamma_mag <= 0.0 {
        return f64::INFINITY;
    }
    -20.0 * gamma_mag.log10()
}

// ─── Impedance matching ────────────────────────────────────────────────────

/// Design an L-network to match `z_source` to `z_load` at frequency `freq_hz`.
///
/// Returns `(l_henries, c_farads)` for the matching network. The L-network
/// consists of a series inductor and a shunt capacitor (or vice versa).
///
/// This implementation uses the real parts of source and load impedances
/// to compute the Q factor, then derives the required L and C values.
///
/// # Panics
///
/// Panics if either impedance has a non-positive real part.
pub fn design_l_network(z_source: (f64, f64), z_load: (f64, f64), freq_hz: f64) -> (f64, f64) {
    let r_s = z_source.0;
    let r_l = z_load.0;
    assert!(r_s > 0.0 && r_l > 0.0, "Impedances must have positive real parts");

    let omega = 2.0 * PI * freq_hz;

    // Ensure r_high > r_low for Q calculation
    let (r_high, r_low) = if r_s > r_l { (r_s, r_l) } else { (r_l, r_s) };

    // Q factor for the match
    let q = ((r_high / r_low) - 1.0).sqrt();

    // Series element reactance (inductor)
    let x_series = q * r_low;
    // Shunt element reactance (capacitor)
    let x_shunt = r_high / q;

    // L = X_series / omega
    let l = x_series / omega;
    // C = 1 / (omega * X_shunt)
    let c = 1.0 / (omega * x_shunt);

    (l, c)
}

/// Design a Pi-network matching circuit.
///
/// Returns `(c1_farads, l_henries, c2_farads)` for a Pi-topology matching
/// network between `r_source` and `r_load` at the given frequency and
/// desired quality factor `q`.
///
/// The Pi-network provides more bandwidth control than the L-network
/// through the additional Q parameter.
pub fn design_pi_network(r_source: f64, r_load: f64, freq_hz: f64, q: f64) -> (f64, f64, f64) {
    let omega = 2.0 * PI * freq_hz;

    // Virtual resistance at the center node
    let r_virtual = r_source.min(r_load) / (1.0 + q * q);

    // First shunt capacitor (across source)
    let q1 = ((r_source / r_virtual) - 1.0).sqrt();
    let x_c1 = r_source / q1;
    let c1 = 1.0 / (omega * x_c1);

    // Second shunt capacitor (across load)
    let q2 = ((r_load / r_virtual) - 1.0).sqrt();
    let x_c2 = r_load / q2;
    let c2 = 1.0 / (omega * x_c2);

    // Series inductor
    let x_l = r_virtual * (q1 + q2);
    let l = x_l / omega;

    (c1, l, c2)
}

/// Design a T-network matching circuit.
///
/// Returns `(l1_henries, c_farads, l2_henries)` for a T-topology matching
/// network between `r_source` and `r_load` at the given frequency and
/// desired quality factor `q`.
pub fn design_t_network(r_source: f64, r_load: f64, freq_hz: f64, q: f64) -> (f64, f64, f64) {
    let omega = 2.0 * PI * freq_hz;

    // Virtual resistance at the center node
    let r_virtual = r_source.max(r_load) * (1.0 + q * q);

    // First series inductor
    let q1 = ((r_virtual / r_source) - 1.0).sqrt();
    let x_l1 = q1 * r_source;
    let l1 = x_l1 / omega;

    // Second series inductor
    let q2 = ((r_virtual / r_load) - 1.0).sqrt();
    let x_l2 = q2 * r_load;
    let l2 = x_l2 / omega;

    // Shunt capacitor
    let x_c = r_virtual / (q1 + q2);
    let c = 1.0 / (omega * x_c);

    (l1, c, l2)
}

// ─── Stability analysis ────────────────────────────────────────────────────

/// Compute the Rollett stability factor K.
///
/// K = (1 - |S11|² - |S22|² + |Δ|²) / (2 * |S12 * S21|)
///
/// where Δ = S11*S22 - S12*S21 (determinant of the S-matrix).
///
/// The device is unconditionally stable if K > 1 and |Δ| < 1.
pub fn stability_k(s: &SParams) -> f64 {
    let s11_mag_sq = s.s11.0 * s.s11.0 + s.s11.1 * s.s11.1;
    let s22_mag_sq = s.s22.0 * s.s22.0 + s.s22.1 * s.s22.1;
    let s12_s21 = complex_mul(s.s12, s.s21);
    let delta = complex_sub(complex_mul(s.s11, s.s22), s12_s21);
    let delta_mag_sq = delta.0 * delta.0 + delta.1 * delta.1;
    let s12_s21_mag = complex_mag(s12_s21);

    if s12_s21_mag == 0.0 {
        return f64::INFINITY; // Unilateral device, unconditionally stable
    }

    (1.0 - s11_mag_sq - s22_mag_sq + delta_mag_sq) / (2.0 * s12_s21_mag)
}

/// Compute the Edwards-Sinsky stability factor mu.
///
/// μ = (1 - |S11|²) / (|S22 - Δ*S11*| + |S12*S21|)
///
/// The device is unconditionally stable if μ > 1. This is a single-parameter
/// stability test (unlike K which also requires |Δ| < 1).
pub fn stability_mu(s: &SParams) -> f64 {
    let s11_mag_sq = s.s11.0 * s.s11.0 + s.s11.1 * s.s11.1;
    let delta = complex_sub(complex_mul(s.s11, s.s22), complex_mul(s.s12, s.s21));
    let s11_conj = complex_conj(s.s11);
    let delta_s11_conj = complex_mul(delta, s11_conj);
    let diff = complex_sub(s.s22, delta_s11_conj);
    let s12_s21_mag = complex_mag(complex_mul(s.s12, s.s21));

    let denom = complex_mag(diff) + s12_s21_mag;
    if denom == 0.0 {
        return f64::INFINITY;
    }

    (1.0 - s11_mag_sq) / denom
}

// ─── Noise figure ──────────────────────────────────────────────────────────

/// Compute the cascaded noise figure using Friis' formula.
///
/// Each stage is specified as `(nf_linear, gain_linear)` where `nf_linear`
/// is the noise factor (not in dB) and `gain_linear` is the power gain
/// (not in dB).
///
/// F_total = F1 + (F2 - 1)/G1 + (F3 - 1)/(G1*G2) + ...
///
/// Returns the total noise factor (linear). Convert to dB with `10*log10(result)`.
///
/// # Panics
///
/// Panics if `stages` is empty.
pub fn noise_figure_cascade(stages: &[(f64, f64)]) -> f64 {
    assert!(!stages.is_empty(), "At least one stage required");

    let mut total_nf = stages[0].0;
    let mut cumulative_gain = stages[0].1;

    for stage in &stages[1..] {
        let (nf, gain) = *stage;
        total_nf += (nf - 1.0) / cumulative_gain;
        cumulative_gain *= gain;
    }

    total_nf
}

// ─── Frequency sweep ───────────────────────────────────────────────────────

/// Perform a frequency sweep, computing S-parameters at each frequency.
///
/// The `circuit` closure receives a frequency in Hz and returns the
/// S-parameters at that frequency.
pub fn frequency_sweep(circuit: &dyn Fn(f64) -> SParams, freqs: &[f64]) -> Vec<SParams> {
    freqs.iter().map(|&f| circuit(f)).collect()
}

// ─── Component impedance helpers ───────────────────────────────────────────

/// Impedance of a resistor: Z = R (purely real).
pub fn impedance_resistor(r: f64) -> (f64, f64) {
    (r, 0.0)
}

/// Impedance of an inductor at frequency `freq_hz`: Z = jωL.
pub fn impedance_inductor(l: f64, freq_hz: f64) -> (f64, f64) {
    (0.0, 2.0 * PI * freq_hz * l)
}

/// Impedance of a capacitor at frequency `freq_hz`: Z = 1/(jωC) = -j/(ωC).
pub fn impedance_capacitor(c: f64, freq_hz: f64) -> (f64, f64) {
    let omega = 2.0 * PI * freq_hz;
    (0.0, -1.0 / (omega * c))
}

/// Admittance of a component (reciprocal of impedance).
pub fn admittance(z: (f64, f64)) -> (f64, f64) {
    complex_div((1.0, 0.0), z)
}

// ─── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-10;
    const LOOSE_EPSILON: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    fn complex_approx_eq(a: (f64, f64), b: (f64, f64), eps: f64) -> bool {
        approx_eq(a.0, b.0, eps) && approx_eq(a.1, b.1, eps)
    }

    #[test]
    fn test_complex_arithmetic_basics() {
        assert_eq!(complex_add((1.0, 2.0), (3.0, 4.0)), (4.0, 6.0));
        assert_eq!(complex_sub((5.0, 3.0), (2.0, 1.0)), (3.0, 2.0));
        // (1+2i)*(3+4i) = 3+4i+6i+8i^2 = -5+10i
        assert_eq!(complex_mul((1.0, 2.0), (3.0, 4.0)), (-5.0, 10.0));
        // (1+0i)/(1+0i) = 1
        assert_eq!(complex_div((1.0, 0.0), (1.0, 0.0)), (1.0, 0.0));
        assert!(approx_eq(complex_mag((3.0, 4.0)), 5.0, EPSILON));
    }

    #[test]
    fn test_complex_div_and_conj() {
        // (1+i)/(1-i) = (1+i)^2/2 = 2i/2 = i
        let result = complex_div((1.0, 1.0), (1.0, -1.0));
        assert!(complex_approx_eq(result, (0.0, 1.0), EPSILON));

        assert_eq!(complex_conj((3.0, -4.0)), (3.0, 4.0));
    }

    #[test]
    fn test_complex_scale_neg_polar() {
        assert_eq!(complex_scale((2.0, 3.0), 2.0), (4.0, 6.0));
        assert_eq!(complex_neg((1.0, -2.0)), (-1.0, 2.0));
        let p = complex_from_polar(1.0, PI / 2.0);
        assert!(complex_approx_eq(p, (0.0, 1.0), EPSILON));
    }

    #[test]
    fn test_series_impedance_matched() {
        // Zero impedance series element => perfect through
        let s = s_params_series_impedance((0.0, 0.0), 50.0);
        assert!(complex_approx_eq(s.s11, (0.0, 0.0), EPSILON));
        assert!(complex_approx_eq(s.s21, (1.0, 0.0), EPSILON));
    }

    #[test]
    fn test_series_impedance_50_ohm() {
        // 50-ohm series resistor in 50-ohm system
        let s = s_params_series_impedance((50.0, 0.0), 50.0);
        // S11 = 50/(50+100) = 1/3
        assert!(approx_eq(s.s11.0, 1.0 / 3.0, EPSILON));
        assert!(approx_eq(s.s11.1, 0.0, EPSILON));
        // S21 = 100/150 = 2/3
        assert!(approx_eq(s.s21.0, 2.0 / 3.0, EPSILON));
        assert!(approx_eq(s.s21.1, 0.0, EPSILON));
    }

    #[test]
    fn test_shunt_admittance_zero() {
        // Zero admittance (open circuit) => perfect through
        let s = s_params_shunt_admittance((0.0, 0.0), 50.0);
        assert!(complex_approx_eq(s.s11, (0.0, 0.0), EPSILON));
        assert!(complex_approx_eq(s.s21, (1.0, 0.0), EPSILON));
    }

    #[test]
    fn test_shunt_admittance_finite() {
        // Shunt 0.02 S (50 ohm to ground) in 50-ohm system
        // Y0 = 0.02 S, Y = 0.02 S
        // S11 = -0.02 / (0.02 + 0.04) = -1/3
        let s = s_params_shunt_admittance((0.02, 0.0), 50.0);
        assert!(approx_eq(s.s11.0, -1.0 / 3.0, EPSILON));
        // S21 = 0.04 / 0.06 = 2/3
        assert!(approx_eq(s.s21.0, 2.0 / 3.0, EPSILON));
    }

    #[test]
    fn test_transmission_line_matched() {
        // TL with Z_line = Z0 => S11 = 0 at any length
        let s = s_params_transmission_line(50.0, PI / 4.0, 50.0);
        assert!(approx_eq(complex_mag(s.s11), 0.0, LOOSE_EPSILON));
        assert!(approx_eq(complex_mag(s.s21), 1.0, LOOSE_EPSILON));
    }

    #[test]
    fn test_transmission_line_quarter_wave() {
        // Quarter-wave transformer: 75-ohm line, 90 degrees, 50-ohm system
        let s = s_params_transmission_line(75.0, PI / 2.0, 50.0);
        // S21 magnitude should be < 1 (some reflection)
        let s21_mag = complex_mag(s.s21);
        assert!(s21_mag > 0.0 && s21_mag <= 1.0);
        // S11 should be non-zero (mismatch)
        let s11_mag = complex_mag(s.s11);
        assert!(s11_mag > 0.0);
    }

    #[test]
    fn test_s_to_abcd_roundtrip() {
        // Create some S-params, convert to ABCD and back
        let original = s_params_series_impedance((25.0, 10.0), 50.0);
        let abcd = s_to_abcd(&original, 50.0);
        let recovered = abcd_to_s(&abcd, 50.0);

        assert!(complex_approx_eq(original.s11, recovered.s11, LOOSE_EPSILON));
        assert!(complex_approx_eq(original.s12, recovered.s12, LOOSE_EPSILON));
        assert!(complex_approx_eq(original.s21, recovered.s21, LOOSE_EPSILON));
        assert!(complex_approx_eq(original.s22, recovered.s22, LOOSE_EPSILON));
    }

    #[test]
    fn test_abcd_identity_cascade() {
        // Cascading identity should give identity
        let identity = AbcdMatrix {
            a: (1.0, 0.0),
            b: (0.0, 0.0),
            c: (0.0, 0.0),
            d: (1.0, 0.0),
        };
        let result = cascade_abcd(&[identity, identity]);
        assert!(complex_approx_eq(result.a, (1.0, 0.0), EPSILON));
        assert!(complex_approx_eq(result.b, (0.0, 0.0), EPSILON));
        assert!(complex_approx_eq(result.c, (0.0, 0.0), EPSILON));
        assert!(complex_approx_eq(result.d, (1.0, 0.0), EPSILON));
    }

    #[test]
    fn test_cascade_empty() {
        // Empty cascade returns identity
        let result = cascade_abcd(&[]);
        assert!(complex_approx_eq(result.a, (1.0, 0.0), EPSILON));
        assert!(complex_approx_eq(result.d, (1.0, 0.0), EPSILON));
    }

    #[test]
    fn test_impedance_to_reflection_matched() {
        // Z = Z0 => Gamma = 0
        let gamma = impedance_to_reflection((50.0, 0.0), 50.0);
        assert!(complex_approx_eq(gamma, (0.0, 0.0), EPSILON));
    }

    #[test]
    fn test_impedance_to_reflection_open() {
        // Z = infinity => Gamma = 1 (approximate with large impedance)
        let gamma = impedance_to_reflection((1e12, 0.0), 50.0);
        assert!(approx_eq(gamma.0, 1.0, 1e-6));
    }

    #[test]
    fn test_impedance_to_reflection_short() {
        // Z = 0 => Gamma = -1
        let gamma = impedance_to_reflection((0.0, 0.0), 50.0);
        assert!(complex_approx_eq(gamma, (-1.0, 0.0), EPSILON));
    }

    #[test]
    fn test_reflection_to_impedance_roundtrip() {
        let z_original = (75.0, 25.0);
        let gamma = impedance_to_reflection(z_original, 50.0);
        let z_recovered = reflection_to_impedance(gamma, 50.0);
        assert!(complex_approx_eq(z_original, z_recovered, LOOSE_EPSILON));
    }

    #[test]
    fn test_vswr_calculations() {
        // Perfect match: Gamma = 0 => VSWR = 1
        assert!(approx_eq(vswr(0.0), 1.0, EPSILON));
        // Gamma = 0.5 => VSWR = 3
        assert!(approx_eq(vswr(0.5), 3.0, EPSILON));
        // Gamma = 1 => VSWR = infinity
        assert!(vswr(1.0).is_infinite());
    }

    #[test]
    fn test_return_loss_db() {
        // Gamma = 1 => return loss = 0 dB
        assert!(approx_eq(return_loss_db(1.0), 0.0, EPSILON));
        // Gamma = 0.1 => return loss = 20 dB
        assert!(approx_eq(return_loss_db(0.1), 20.0, LOOSE_EPSILON));
        // Perfect match => infinite return loss
        assert!(return_loss_db(0.0).is_infinite());
    }

    #[test]
    fn test_design_l_network() {
        // Match 50 ohm to 200 ohm at 1 GHz
        let (l, c) = design_l_network((50.0, 0.0), (200.0, 0.0), 1e9);
        assert!(l > 0.0, "Inductance must be positive");
        assert!(c > 0.0, "Capacitance must be positive");
        // L should be in nH range, C in pF range for 1 GHz
        assert!(l < 1e-6, "L should be small at GHz frequencies");
        assert!(c < 1e-9, "C should be small at GHz frequencies");
    }

    #[test]
    fn test_design_pi_network() {
        let (c1, l, c2) = design_pi_network(50.0, 75.0, 1e9, 5.0);
        assert!(c1 > 0.0);
        assert!(l > 0.0);
        assert!(c2 > 0.0);
    }

    #[test]
    fn test_design_t_network() {
        let (l1, c, l2) = design_t_network(50.0, 75.0, 1e9, 5.0);
        assert!(l1 > 0.0);
        assert!(c > 0.0);
        assert!(l2 > 0.0);
    }

    #[test]
    fn test_stability_k_passive() {
        // A passive reciprocal network (e.g., attenuator) should have K >= 1
        let s = s_params_series_impedance((100.0, 0.0), 50.0);
        let k = stability_k(&s);
        assert!(k >= 1.0, "Passive network should be unconditionally stable, K={}", k);
    }

    #[test]
    fn test_stability_mu() {
        // For the same passive network, mu should also indicate stability
        let s = s_params_series_impedance((100.0, 0.0), 50.0);
        let mu = stability_mu(&s);
        assert!(mu > 0.0, "mu should be positive for passive network, mu={}", mu);
    }

    #[test]
    fn test_noise_figure_single_stage() {
        // Single stage with NF = 3 dB (factor 2.0), gain = 10 dB (10x)
        let nf = noise_figure_cascade(&[(2.0, 10.0)]);
        assert!(approx_eq(nf, 2.0, EPSILON));
    }

    #[test]
    fn test_noise_figure_two_stage_friis() {
        // Stage 1: NF = 2 (3 dB), Gain = 10 (10 dB)
        // Stage 2: NF = 10 (10 dB), Gain = 100 (20 dB)
        // F_total = 2 + (10-1)/10 = 2 + 0.9 = 2.9
        let nf = noise_figure_cascade(&[(2.0, 10.0), (10.0, 100.0)]);
        assert!(approx_eq(nf, 2.9, EPSILON));
    }

    #[test]
    fn test_frequency_sweep() {
        // Sweep a series inductor across frequencies
        let l = 10e-9; // 10 nH
        let z0 = 50.0;
        let freqs: Vec<f64> = (1..=5).map(|i| i as f64 * 1e9).collect();

        let results = frequency_sweep(
            &|f| {
                let z = impedance_inductor(l, f);
                s_params_series_impedance(z, z0)
            },
            &freqs,
        );

        assert_eq!(results.len(), 5);
        // S11 magnitude should increase with frequency (more mismatch)
        let mag_first = complex_mag(results[0].s11);
        let mag_last = complex_mag(results[4].s11);
        assert!(mag_last > mag_first, "Higher freq should have more reflection for series inductor");
    }

    #[test]
    fn test_component_impedances() {
        // Resistor
        assert_eq!(impedance_resistor(100.0), (100.0, 0.0));

        // Inductor at 1 GHz, 10 nH => Z = j*2*pi*1e9*10e-9 = j*62.83...
        let z_l = impedance_inductor(10e-9, 1e9);
        assert!(approx_eq(z_l.0, 0.0, EPSILON));
        assert!(approx_eq(z_l.1, 2.0 * PI * 1e9 * 10e-9, LOOSE_EPSILON));

        // Capacitor at 1 GHz, 1 pF => Z = -j/(2*pi*1e9*1e-12)
        let z_c = impedance_capacitor(1e-12, 1e9);
        assert!(approx_eq(z_c.0, 0.0, EPSILON));
        assert!(z_c.1 < 0.0, "Capacitor impedance should be negative imaginary");
    }

    #[test]
    fn test_admittance() {
        // Y = 1/Z for a 50-ohm resistor
        let y = admittance((50.0, 0.0));
        assert!(approx_eq(y.0, 0.02, EPSILON));
        assert!(approx_eq(y.1, 0.0, EPSILON));
    }

    #[test]
    fn test_rf_circuit_sim_cascade() {
        let mut sim = RfCircuitSim::new(50.0);

        // Add two series 25-ohm resistors (should equal one 50-ohm resistor)
        let s1 = s_params_series_impedance((25.0, 0.0), 50.0);
        let abcd1 = s_to_abcd(&s1, 50.0);
        sim.add_stage(abcd1);
        sim.add_stage(abcd1);

        let result = sim.s_params();
        // Two 25-ohm resistors in series = 50-ohm series impedance
        let expected = s_params_series_impedance((50.0, 0.0), 50.0);
        assert!(complex_approx_eq(result.s11, expected.s11, LOOSE_EPSILON));
        assert!(complex_approx_eq(result.s21, expected.s21, LOOSE_EPSILON));
    }

    #[test]
    fn test_s_params_reciprocity_and_symmetry() {
        // For a symmetric, reciprocal network: S12 = S21 and S11 = S22
        let s = s_params_series_impedance((30.0, 15.0), 50.0);
        assert!(complex_approx_eq(s.s12, s.s21, EPSILON));
        assert!(complex_approx_eq(s.s11, s.s22, EPSILON));
    }
}

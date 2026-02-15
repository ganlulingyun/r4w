//! # Quantum Decoherence Characterizer
//!
//! This module implements quantum decoherence characterization for qubit systems,
//! measuring T1/T2 relaxation times and dephasing processes.
//!
//! ## Background
//!
//! Quantum decoherence is the process by which a quantum system loses its quantum
//! properties due to interaction with its environment. The key timescales are:
//!
//! - **T1** (energy relaxation): Time for the excited state |1> to decay to |0>.
//!   Also called longitudinal relaxation or spin-lattice relaxation.
//! - **T2** (Hahn echo / transverse coherence): Time for phase coherence to decay,
//!   measured with a spin-echo (refocusing) pulse. Removes static inhomogeneities.
//! - **T2*** (Ramsey / inhomogeneous dephasing): Includes both intrinsic dephasing
//!   and static field inhomogeneities. Always T2* <= T2 <= 2*T1.
//!
//! ## Components
//!
//! - [`BlochVector`] -- State on the Bloch sphere (x, y, z).
//! - [`DecoherenceCharacterizer`] -- Main struct for T1/T2/T2* characterization.
//! - [`DynamicalDecoupling`] -- CPMG, XY-4 pulse sequences for extending coherence.
//! - [`NoiseSpectroscopy`] -- Reconstruct noise PSD from dynamical decoupling data.
//! - [`BlochSimulator`] -- Bloch equation integration with relaxation.
//! - [`LindbladSimulator`] -- Simplified Lindblad master equation for density matrices.
//! - [`GateFidelity`] -- Gate fidelity estimation from T1/T2.
//! - [`RandomizedBenchmarking`] -- Simulated RB sequence fidelity decay.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::quantum_decoherence_characterizer::*;
//!
//! let char = DecoherenceCharacterizer::new(50.0, 30.0, 10.0);
//! assert!((char.t1_us() - 50.0).abs() < 1e-10);
//! assert!((char.t2_us() - 30.0).abs() < 1e-10);
//! assert!((char.t2_star_us() - 10.0).abs() < 1e-10);
//!
//! // Generate T1 decay data
//! let (times, probs) = char.t1_decay(0.0, 200.0, 50);
//! assert_eq!(times.len(), 50);
//! assert!(probs[0] > probs[49]); // Decays over time
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// BlochVector
// ---------------------------------------------------------------------------

/// A point on the Bloch sphere representing a qubit state.
///
/// The Bloch sphere maps pure qubit states to the unit sphere:
/// - |0> = (0, 0, +1) (north pole)
/// - |1> = (0, 0, -1) (south pole)
/// - |+> = (+1, 0, 0)
/// - |-> = (-1, 0, 0)
/// - |+i> = (0, +1, 0)
/// - |-i> = (0, -1, 0)
#[derive(Debug, Clone, Copy)]
pub struct BlochVector {
    /// X component (Re(<sigma_x>))
    pub x: f64,
    /// Y component (Re(<sigma_y>))
    pub y: f64,
    /// Z component (Re(<sigma_z>))
    pub z: f64,
}

impl BlochVector {
    /// Create a new Bloch vector.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Create from spherical angles on the Bloch sphere.
    /// theta: polar angle from +z axis (0 = |0>, pi = |1>)
    /// phi: azimuthal angle in xy-plane from +x axis
    pub fn from_angles(theta: f64, phi: f64) -> Self {
        Self {
            x: theta.sin() * phi.cos(),
            y: theta.sin() * phi.sin(),
            z: theta.cos(),
        }
    }

    /// Create the |0> state (north pole).
    pub fn zero_state() -> Self {
        Self { x: 0.0, y: 0.0, z: 1.0 }
    }

    /// Create the |1> state (south pole).
    pub fn one_state() -> Self {
        Self { x: 0.0, y: 0.0, z: -1.0 }
    }

    /// Create the |+> state (equator, +x).
    pub fn plus_state() -> Self {
        Self { x: 1.0, y: 0.0, z: 0.0 }
    }

    /// Length of the Bloch vector (1.0 for pure states, <1 for mixed).
    pub fn length(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Transverse component magnitude sqrt(x^2 + y^2).
    pub fn transverse(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    /// Normalize to unit length.
    pub fn normalize(&self) -> Self {
        let len = self.length();
        if len < 1e-15 {
            return *self;
        }
        Self {
            x: self.x / len,
            y: self.y / len,
            z: self.z / len,
        }
    }

    /// Dot product with another Bloch vector.
    pub fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Cross product with another Bloch vector.
    pub fn cross(&self, other: &Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }
}

// ---------------------------------------------------------------------------
// 2x2 Density Matrix (for Lindblad)
// ---------------------------------------------------------------------------

/// 2x2 density matrix for a single qubit, stored as (rho_00, rho_01, rho_10, rho_11).
/// rho_01 and rho_10 are complex, stored as (real, imag).
#[derive(Debug, Clone, Copy)]
pub struct DensityMatrix2x2 {
    /// rho_00 (real, diagonal)
    pub rho_00: f64,
    /// rho_01 (real part)
    pub rho_01_re: f64,
    /// rho_01 (imaginary part)
    pub rho_01_im: f64,
    /// rho_10 (real part) = rho_01_re for Hermitian
    pub rho_10_re: f64,
    /// rho_10 (imaginary part) = -rho_01_im for Hermitian
    pub rho_10_im: f64,
    /// rho_11 (real, diagonal)
    pub rho_11: f64,
}

impl DensityMatrix2x2 {
    /// Create from a pure state |0>.
    pub fn zero_state() -> Self {
        Self {
            rho_00: 1.0, rho_01_re: 0.0, rho_01_im: 0.0,
            rho_10_re: 0.0, rho_10_im: 0.0, rho_11: 0.0,
        }
    }

    /// Create from a pure state |1>.
    pub fn one_state() -> Self {
        Self {
            rho_00: 0.0, rho_01_re: 0.0, rho_01_im: 0.0,
            rho_10_re: 0.0, rho_10_im: 0.0, rho_11: 1.0,
        }
    }

    /// Create the |+> state = (|0> + |1>)/sqrt(2).
    pub fn plus_state() -> Self {
        Self {
            rho_00: 0.5, rho_01_re: 0.5, rho_01_im: 0.0,
            rho_10_re: 0.5, rho_10_im: 0.0, rho_11: 0.5,
        }
    }

    /// Create a maximally mixed state I/2.
    pub fn maximally_mixed() -> Self {
        Self {
            rho_00: 0.5, rho_01_re: 0.0, rho_01_im: 0.0,
            rho_10_re: 0.0, rho_10_im: 0.0, rho_11: 0.5,
        }
    }

    /// Trace of the density matrix (should be 1.0).
    pub fn trace(&self) -> f64 {
        self.rho_00 + self.rho_11
    }

    /// Purity Tr(rho^2). 1.0 for pure states, 0.5 for maximally mixed.
    pub fn purity(&self) -> f64 {
        self.rho_00 * self.rho_00 + self.rho_11 * self.rho_11
            + 2.0 * (self.rho_01_re * self.rho_01_re + self.rho_01_im * self.rho_01_im)
    }

    /// Convert to Bloch vector representation.
    pub fn to_bloch(&self) -> BlochVector {
        BlochVector {
            x: 2.0 * self.rho_01_re,
            y: 2.0 * self.rho_01_im,
            z: self.rho_00 - self.rho_11,
        }
    }

    /// Create from a Bloch vector.
    pub fn from_bloch(bv: &BlochVector) -> Self {
        Self {
            rho_00: 0.5 * (1.0 + bv.z),
            rho_01_re: 0.5 * bv.x,
            rho_01_im: 0.5 * bv.y,
            rho_10_re: 0.5 * bv.x,
            rho_10_im: -0.5 * bv.y,
            rho_11: 0.5 * (1.0 - bv.z),
        }
    }

    /// Fidelity with another density matrix (simplified for 2x2).
    /// F = Tr(rho * sigma) + 2*sqrt(det(rho)*det(sigma)) for qubit states.
    /// For pure target state |psi>, F = <psi|rho|psi>.
    pub fn fidelity_with_pure_zero(&self) -> f64 {
        self.rho_00
    }

    /// Fidelity with |1> state.
    pub fn fidelity_with_pure_one(&self) -> f64 {
        self.rho_11
    }

    /// Fidelity with |+> state.
    pub fn fidelity_with_plus(&self) -> f64 {
        0.5 * (self.rho_00 + self.rho_11) + self.rho_01_re
    }
}

// ---------------------------------------------------------------------------
// DecoherenceCharacterizer
// ---------------------------------------------------------------------------

/// Main struct for characterizing qubit decoherence.
///
/// Stores the fundamental relaxation timescales and provides methods
/// to generate synthetic measurement data, simulate Bloch dynamics,
/// and estimate gate fidelities.
pub struct DecoherenceCharacterizer {
    /// Energy relaxation time (T1) in microseconds.
    t1: f64,
    /// Transverse coherence time from Hahn echo (T2) in microseconds.
    t2: f64,
    /// Inhomogeneous dephasing time (T2*) in microseconds.
    t2_star: f64,
}

impl DecoherenceCharacterizer {
    /// Create a new characterizer with given T1, T2, T2* in microseconds.
    ///
    /// Enforces the fundamental constraint T2* <= T2 <= 2*T1.
    /// If T2 > 2*T1, it is clamped to 2*T1.
    /// If T2* > T2, it is clamped to T2.
    pub fn new(t1_us: f64, t2_us: f64, t2_star_us: f64) -> Self {
        assert!(t1_us > 0.0, "T1 must be positive");
        let t2_clamped = t2_us.min(2.0 * t1_us);
        let t2_star_clamped = t2_star_us.min(t2_clamped);
        Self {
            t1: t1_us,
            t2: t2_clamped,
            t2_star: t2_star_clamped,
        }
    }

    /// Get T1 in microseconds.
    pub fn t1_us(&self) -> f64 {
        self.t1
    }

    /// Get T2 in microseconds.
    pub fn t2_us(&self) -> f64 {
        self.t2
    }

    /// Get T2* in microseconds.
    pub fn t2_star_us(&self) -> f64 {
        self.t2_star
    }

    /// Pure dephasing time T_phi, defined by 1/T2 = 1/(2*T1) + 1/T_phi.
    pub fn t_phi_us(&self) -> f64 {
        let rate = 1.0 / self.t2 - 1.0 / (2.0 * self.t1);
        if rate <= 0.0 {
            f64::INFINITY
        } else {
            1.0 / rate
        }
    }

    // -----------------------------------------------------------------------
    // T1 Measurement
    // -----------------------------------------------------------------------

    /// Generate ideal T1 decay curve (no noise).
    ///
    /// Prepares |1>, waits time tau, measures P(|1>).
    /// P(|1>) = exp(-tau / T1)
    ///
    /// Returns (times, probabilities) with `n_points` equally spaced in [tau_min, tau_max].
    pub fn t1_decay(&self, tau_min: f64, tau_max: f64, n_points: usize) -> (Vec<f64>, Vec<f64>) {
        let mut times = Vec::with_capacity(n_points);
        let mut probs = Vec::with_capacity(n_points);
        for i in 0..n_points {
            let tau = if n_points <= 1 {
                tau_min
            } else {
                tau_min + (tau_max - tau_min) * (i as f64) / ((n_points - 1) as f64)
            };
            times.push(tau);
            probs.push((-tau / self.t1).exp());
        }
        (times, probs)
    }

    /// Generate T1 decay data with synthetic noise (deterministic seeded PRNG).
    ///
    /// Model: P(|1>) = A * exp(-tau/T1) + B + noise
    /// A = amplitude, B = baseline offset.
    pub fn t1_decay_noisy(
        &self,
        tau_min: f64,
        tau_max: f64,
        n_points: usize,
        amplitude: f64,
        baseline: f64,
        noise_sigma: f64,
        seed: u64,
    ) -> (Vec<f64>, Vec<f64>) {
        let (times, ideal) = self.t1_decay(tau_min, tau_max, n_points);
        let mut prng = SimplePrng::new(seed);
        let probs = ideal
            .iter()
            .map(|&p| {
                let noisy = amplitude * p + baseline + noise_sigma * prng.gaussian();
                noisy.clamp(0.0, 1.0)
            })
            .collect();
        (times, probs)
    }

    /// Fit T1 from decay data using linearized least squares.
    ///
    /// Model: P = A * exp(-tau/T1). We fit ln(P) = ln(A) - tau/T1.
    /// Only positive P values are used. Returns estimated T1 in the same time units as input.
    pub fn fit_t1(times: &[f64], probs: &[f64]) -> f64 {
        let mut sum_t = 0.0;
        let mut sum_lnp = 0.0;
        let mut sum_t_lnp = 0.0;
        let mut sum_t2 = 0.0;
        let mut n = 0.0;

        for (&t, &p) in times.iter().zip(probs.iter()) {
            if p > 1e-10 {
                let lnp = p.ln();
                sum_t += t;
                sum_lnp += lnp;
                sum_t_lnp += t * lnp;
                sum_t2 += t * t;
                n += 1.0;
            }
        }

        if n < 2.0 {
            return f64::NAN; // fallback: insufficient data
        }

        // Linear regression: lnp = a + b*t => b = -1/T1
        let denom = n * sum_t2 - sum_t * sum_t;
        if denom.abs() < 1e-15 {
            return f64::NAN;
        }
        let slope = (n * sum_t_lnp - sum_t * sum_lnp) / denom;
        if slope >= 0.0 {
            return f64::INFINITY; // No decay detected
        }
        -1.0 / slope
    }

    // -----------------------------------------------------------------------
    // T2* (Ramsey) Measurement
    // -----------------------------------------------------------------------

    /// Generate Ramsey (T2*) fringe data.
    ///
    /// After pi/2 - free evolution tau - pi/2:
    /// Signal = A * exp(-tau / T2*) * cos(delta_omega * tau) + B
    ///
    /// delta_omega is the detuning frequency in rad/us.
    pub fn ramsey_fringe(
        &self,
        tau_min: f64,
        tau_max: f64,
        n_points: usize,
        delta_omega: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let mut times = Vec::with_capacity(n_points);
        let mut signal = Vec::with_capacity(n_points);
        for i in 0..n_points {
            let tau = if n_points <= 1 {
                tau_min
            } else {
                tau_min + (tau_max - tau_min) * (i as f64) / ((n_points - 1) as f64)
            };
            times.push(tau);
            let envelope = (-tau / self.t2_star).exp();
            let oscillation = (delta_omega * tau).cos();
            signal.push(envelope * oscillation);
        }
        (times, signal)
    }

    // -----------------------------------------------------------------------
    // T2 (Hahn Echo) Measurement
    // -----------------------------------------------------------------------

    /// Generate Hahn echo (T2) decay data.
    ///
    /// pi/2 - tau/2 - pi (refocusing) - tau/2 - pi/2:
    /// Signal = A * exp(-tau / T2)
    ///
    /// The pi pulse refocuses static dephasing, so only intrinsic T2 remains.
    pub fn hahn_echo_decay(
        &self,
        tau_min: f64,
        tau_max: f64,
        n_points: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        let mut times = Vec::with_capacity(n_points);
        let mut signal = Vec::with_capacity(n_points);
        for i in 0..n_points {
            let tau = if n_points <= 1 {
                tau_min
            } else {
                tau_min + (tau_max - tau_min) * (i as f64) / ((n_points - 1) as f64)
            };
            times.push(tau);
            signal.push((-tau / self.t2).exp());
        }
        (times, signal)
    }

    // -----------------------------------------------------------------------
    // Gate Fidelity
    // -----------------------------------------------------------------------

    /// First-order gate fidelity estimate.
    ///
    /// F_gate ~ 1 - t_gate/T1 - t_gate/T2
    pub fn gate_fidelity(&self, t_gate_us: f64) -> f64 {
        let f = 1.0 - t_gate_us / self.t1 - t_gate_us / self.t2;
        f.max(0.0)
    }

    /// Average gate fidelity for single-qubit gate (more precise).
    ///
    /// For a depolarizing channel with error rate p:
    /// F_avg = 1 - (2/3) * (1 - exp(-t_gate/T1)) - (2/3) * (1 - exp(-t_gate/T2))
    pub fn gate_fidelity_precise(&self, t_gate_us: f64) -> f64 {
        let p_relax = 1.0 - (-t_gate_us / self.t1).exp();
        let p_dephase = 1.0 - (-t_gate_us / self.t2).exp();
        let f = 1.0 - (2.0 / 3.0) * p_relax - (1.0 / 3.0) * p_dephase;
        f.max(0.0).min(1.0)
    }
}

// ---------------------------------------------------------------------------
// Dynamical Decoupling
// ---------------------------------------------------------------------------

/// Type of dynamical decoupling sequence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DdSequence {
    /// Carr-Purcell-Meiboom-Gill: N equally-spaced pi pulses along Y.
    Cpmg,
    /// XY-4: alternating X and Y pi pulses for robustness to pulse errors.
    Xy4,
}

/// Dynamical decoupling calculator.
///
/// Computes effective T2 and filter functions for DD sequences.
pub struct DynamicalDecoupling {
    /// Base T2 without DD (in us).
    t2_base: f64,
    /// Noise spectral exponent (1/f^alpha noise).
    noise_alpha: f64,
}

impl DynamicalDecoupling {
    /// Create a new DD calculator.
    ///
    /// * `t2_base` - intrinsic T2 in microseconds
    /// * `noise_alpha` - noise PSD exponent (1.0 for 1/f, 0.0 for white)
    pub fn new(t2_base: f64, noise_alpha: f64) -> Self {
        Self { t2_base, noise_alpha }
    }

    /// Compute pi-pulse times for a CPMG sequence.
    ///
    /// N pi-pulses equally spaced in total evolution time tau.
    /// Pulse times at tau/(2N), 3*tau/(2N), ..., (2N-1)*tau/(2N).
    pub fn cpmg_pulse_times(n_pulses: usize, tau: f64) -> Vec<f64> {
        let mut times = Vec::with_capacity(n_pulses);
        for k in 0..n_pulses {
            times.push(tau * (2 * k + 1) as f64 / (2 * n_pulses) as f64);
        }
        times
    }

    /// Compute pi-pulse times for XY-4 sequence.
    ///
    /// XY-4 repeats the pattern [X, Y, X, Y] with equal spacing.
    /// Returns (times, axes) where axes alternate X(true) / Y(false).
    pub fn xy4_pulse_times(n_repeats: usize, tau: f64) -> (Vec<f64>, Vec<bool>) {
        let n_pulses = 4 * n_repeats;
        let mut times = Vec::with_capacity(n_pulses);
        let mut axes = Vec::with_capacity(n_pulses);
        for k in 0..n_pulses {
            times.push(tau * (2 * k + 1) as f64 / (2 * n_pulses) as f64);
            // Pattern: X, Y, X, Y
            axes.push(k % 2 == 0); // true = X, false = Y
        }
        (times, axes)
    }

    /// Compute the filter function F(omega) for a DD sequence.
    ///
    /// F(omega) = |1 + sum_k (-1)^(k+1) * 2 * exp(i*omega*t_k) + (-1)^N * exp(i*omega*tau)|^2
    ///
    /// Simplified for evenly-spaced pulses (CPMG-like).
    pub fn filter_function(pulse_times: &[f64], tau: f64, omega: f64) -> f64 {
        // The modulation function y(t) switches sign at each pi-pulse.
        // F(omega) = |omega * integral_0^tau y(t) exp(i*omega*t) dt|^2
        // For piecewise-constant y(t) = +/-1:
        // F(omega) = |sum of segments|^2

        let n = pulse_times.len();
        let mut boundaries = Vec::with_capacity(n + 2);
        boundaries.push(0.0);
        for &t in pulse_times {
            boundaries.push(t);
        }
        boundaries.push(tau);

        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        let mut sign = 1.0_f64;

        for i in 0..boundaries.len() - 1 {
            let t_start = boundaries[i];
            let t_end = boundaries[i + 1];

            if omega.abs() < 1e-12 {
                // F(0) limit
                sum_re += sign * (t_end - t_start);
            } else {
                // Integral of sign * exp(i*omega*t) dt from t_start to t_end
                // = sign * [exp(i*omega*t_end) - exp(i*omega*t_start)] / (i*omega)
                let e_end_re = (omega * t_end).cos();
                let e_end_im = (omega * t_end).sin();
                let e_start_re = (omega * t_start).cos();
                let e_start_im = (omega * t_start).sin();
                let diff_re = e_end_re - e_start_re;
                let diff_im = e_end_im - e_start_im;
                // Divide by i*omega: (a+ib)/(i*omega) = (b - ia)/omega
                sum_re += sign * diff_im / omega;
                sum_im += sign * (-diff_re) / omega;
            }
            sign = -sign;
        }

        sum_re * sum_re + sum_im * sum_im
    }

    /// Effective T2 with N DD pulses for 1/f^alpha noise.
    ///
    /// For 1/f^p noise: T2_eff ~ T2 * (N * tau_0 / T2)^(1/(p+1))
    /// where tau_0 is the free evolution time and N is the number of pulses.
    ///
    /// This is an approximation valid for large N.
    pub fn effective_t2(&self, n_pulses: usize, tau_total: f64) -> f64 {
        if n_pulses == 0 {
            return self.t2_base;
        }
        let p = self.noise_alpha;
        let ratio = (n_pulses as f64) * tau_total / self.t2_base;
        self.t2_base * ratio.powf(1.0 / (p + 1.0))
    }

    /// Coherence decay under CPMG with N pulses at total time tau.
    ///
    /// Signal = exp(-(tau / T2_eff)^(1 + alpha)) for stretched exponential.
    /// Simplified: Signal = exp(-tau / T2_eff) for moderate N.
    pub fn cpmg_coherence(&self, n_pulses: usize, tau: f64) -> f64 {
        let t2_eff = self.effective_t2(n_pulses, tau);
        (-tau / t2_eff).exp()
    }
}

// ---------------------------------------------------------------------------
// Noise Spectroscopy
// ---------------------------------------------------------------------------

/// Noise spectroscopy: reconstruct noise PSD from DD measurements.
pub struct NoiseSpectroscopy;

impl NoiseSpectroscopy {
    /// Estimate noise PSD at frequency f from CPMG decay data.
    ///
    /// Using the relation: decay_rate(N) ~ integral S(f) * F_N(f) df
    /// For CPMG with N pulses, the filter peaks at f_N = N / (2 * tau).
    /// S(f_N) ~ -ln(coherence) / tau
    ///
    /// Returns Vec<(frequency, spectral_density)>.
    pub fn reconstruct_psd(
        n_pulses_list: &[usize],
        tau: f64,
        coherences: &[f64],
    ) -> Vec<(f64, f64)> {
        let mut psd = Vec::with_capacity(n_pulses_list.len());
        for (&n, &c) in n_pulses_list.iter().zip(coherences.iter()) {
            if n == 0 || c <= 0.0 || c >= 1.0 {
                continue;
            }
            let freq = n as f64 / (2.0 * tau);
            let decay_rate = -c.ln() / tau;
            // S(f) ~ decay_rate / (pi * N)
            let s_f = decay_rate / (PI * n as f64);
            psd.push((freq, s_f));
        }
        psd
    }

    /// Fit 1/f^alpha noise model: S(f) = A / f^alpha.
    ///
    /// Uses log-log linear regression.
    /// Returns (A, alpha).
    pub fn fit_one_over_f(psd: &[(f64, f64)]) -> (f64, f64) {
        if psd.len() < 2 {
            return (1.0, 1.0);
        }

        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xy = 0.0;
        let mut sum_x2 = 0.0;
        let n = psd.len() as f64;

        for &(f, s) in psd {
            if f > 0.0 && s > 0.0 {
                let lf = f.ln();
                let ls = s.ln();
                sum_x += lf;
                sum_y += ls;
                sum_xy += lf * ls;
                sum_x2 += lf * lf;
            }
        }

        let denom = n * sum_x2 - sum_x * sum_x;
        if denom.abs() < 1e-15 {
            return (1.0, 1.0);
        }

        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n;

        let alpha = -slope;
        let a = intercept.exp();
        (a, alpha)
    }

    /// Estimate white noise floor from high-frequency PSD values.
    ///
    /// Takes the mean of the highest-frequency quartile of PSD data.
    pub fn white_noise_floor(psd: &[(f64, f64)]) -> f64 {
        if psd.is_empty() {
            return 0.0;
        }
        let mut sorted: Vec<(f64, f64)> = psd.to_vec();
        sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        let start = sorted.len() * 3 / 4;
        let slice = &sorted[start..];
        if slice.is_empty() {
            return sorted.last().unwrap().1;
        }
        slice.iter().map(|&(_, s)| s).sum::<f64>() / slice.len() as f64
    }
}

// ---------------------------------------------------------------------------
// Bloch Equation Simulator
// ---------------------------------------------------------------------------

/// Bloch equation simulator with relaxation.
///
/// Integrates the Bloch equations:
/// dMx/dt = gamma * (My*Bz - Mz*By) - Mx/T2
/// dMy/dt = gamma * (Mz*Bx - Mx*Bz) - My/T2
/// dMz/dt = gamma * (Mx*By - My*Bx) - (Mz - M0)/T1
pub struct BlochSimulator {
    /// T1 relaxation time (arbitrary units matching dt).
    t1: f64,
    /// T2 relaxation time.
    t2: f64,
    /// Equilibrium magnetization (usually 1.0 for |0>).
    m0_z: f64,
    /// Gyromagnetic ratio (for precession).
    gamma: f64,
}

impl BlochSimulator {
    /// Create a new Bloch simulator.
    pub fn new(t1: f64, t2: f64) -> Self {
        Self {
            t1,
            t2,
            m0_z: 1.0,
            gamma: 2.0 * PI, // normalized so B=1 gives omega = 2*pi
        }
    }

    /// Set the gyromagnetic ratio.
    pub fn with_gamma(mut self, gamma: f64) -> Self {
        self.gamma = gamma;
        self
    }

    /// Euler integration step.
    ///
    /// B = (bx, by, bz) is the magnetic field vector.
    pub fn step(&self, m: &BlochVector, bx: f64, by: f64, bz: f64, dt: f64) -> BlochVector {
        // Precession: dM/dt = gamma * (M x B)
        let precession = BlochVector::new(
            self.gamma * (m.y * bz - m.z * by),
            self.gamma * (m.z * bx - m.x * bz),
            self.gamma * (m.x * by - m.y * bx),
        );

        // Relaxation
        let relax_x = -m.x / self.t2;
        let relax_y = -m.y / self.t2;
        let relax_z = -(m.z - self.m0_z) / self.t1;

        BlochVector {
            x: m.x + (precession.x + relax_x) * dt,
            y: m.y + (precession.y + relax_y) * dt,
            z: m.z + (precession.z + relax_z) * dt,
        }
    }

    /// Simulate free evolution (no applied field) for time duration.
    pub fn free_evolution(&self, initial: &BlochVector, duration: f64, n_steps: usize) -> BlochVector {
        let dt = duration / n_steps as f64;
        let mut m = *initial;
        for _ in 0..n_steps {
            m = self.step(&m, 0.0, 0.0, 0.0, dt);
        }
        m
    }

    /// Simulate evolution in a static B0 field along z for time duration.
    pub fn evolve_in_field(
        &self,
        initial: &BlochVector,
        b0: f64,
        duration: f64,
        n_steps: usize,
    ) -> BlochVector {
        let dt = duration / n_steps as f64;
        let mut m = *initial;
        for _ in 0..n_steps {
            m = self.step(&m, 0.0, 0.0, b0, dt);
        }
        m
    }

    /// Record trajectory during free evolution.
    pub fn record_free_evolution(
        &self,
        initial: &BlochVector,
        duration: f64,
        n_steps: usize,
    ) -> Vec<BlochVector> {
        let dt = duration / n_steps as f64;
        let mut trajectory = Vec::with_capacity(n_steps + 1);
        let mut m = *initial;
        trajectory.push(m);
        for _ in 0..n_steps {
            m = self.step(&m, 0.0, 0.0, 0.0, dt);
            trajectory.push(m);
        }
        trajectory
    }
}

// ---------------------------------------------------------------------------
// Lindblad Master Equation (simplified 2x2)
// ---------------------------------------------------------------------------

/// Simplified Lindblad master equation simulator for a single qubit.
///
/// drho/dt = -i[H, rho] + sum_k (L_k rho L_k^dag - 0.5 {L_k^dag L_k, rho})
///
/// Channels:
/// - Amplitude damping (T1 process): L = sqrt(gamma_1) * |0><1|
/// - Phase damping (pure dephasing T_phi): L = sqrt(gamma_phi/2) * sigma_z
///
/// Rates: gamma_1 = 1/T1, gamma_phi = 1/T_phi
/// 1/T2 = 1/(2*T1) + 1/T_phi
pub struct LindbladSimulator {
    /// Amplitude damping rate (1/T1).
    gamma_1: f64,
    /// Pure dephasing rate (1/T_phi).
    gamma_phi: f64,
    /// Hamiltonian frequency (omega_q / 2 for qubit splitting).
    omega: f64,
}

impl LindbladSimulator {
    /// Create from T1 and T2 in microseconds.
    ///
    /// Computes gamma_phi from 1/T2 = 1/(2*T1) + 1/T_phi.
    pub fn from_t1_t2(t1: f64, t2: f64, omega: f64) -> Self {
        let gamma_1 = 1.0 / t1;
        let gamma_phi_rate = 1.0 / t2 - 1.0 / (2.0 * t1);
        let gamma_phi = if gamma_phi_rate > 0.0 { gamma_phi_rate } else { 0.0 };
        Self { gamma_1, gamma_phi, omega }
    }

    /// Euler integration step for the density matrix.
    pub fn step(&self, rho: &DensityMatrix2x2, dt: f64) -> DensityMatrix2x2 {
        // Hamiltonian evolution: H = (omega/2) * sigma_z
        // -i[H, rho]:
        // d(rho_01)/dt += -i * omega * rho_01
        // d(rho_10)/dt += +i * omega * rho_10
        // Diagonals unchanged by Hamiltonian.

        // Amplitude damping: L = sqrt(gamma_1) |0><1|
        // L rho L^dag = gamma_1 * rho_11 * |0><0|
        // L^dag L = gamma_1 * |1><1|
        // {L^dag L, rho}/2 = gamma_1/2 * (|1><1| rho + rho |1><1|)

        // Phase damping: L = sqrt(gamma_phi/2) sigma_z
        // L rho L^dag = (gamma_phi/2) sigma_z rho sigma_z
        // L^dag L = (gamma_phi/2) I
        // Effects: dephases off-diagonal elements at rate gamma_phi

        // Combined derivatives:
        // d(rho_00) = gamma_1 * rho_11
        // d(rho_11) = -gamma_1 * rho_11
        // d(rho_01) = -i*omega*rho_01 - (gamma_1/2 + gamma_phi)*rho_01
        // d(rho_10) = +i*omega*rho_10 - (gamma_1/2 + gamma_phi)*rho_10

        let off_diag_rate = self.gamma_1 / 2.0 + self.gamma_phi;

        let d_rho_00 = self.gamma_1 * rho.rho_11;
        let d_rho_11 = -self.gamma_1 * rho.rho_11;

        // d(rho_01) = (-off_diag_rate - i*omega) * rho_01
        let d_rho_01_re = -off_diag_rate * rho.rho_01_re + self.omega * rho.rho_01_im;
        let d_rho_01_im = -off_diag_rate * rho.rho_01_im - self.omega * rho.rho_01_re;

        let d_rho_10_re = -off_diag_rate * rho.rho_10_re - self.omega * rho.rho_10_im;
        let d_rho_10_im = -off_diag_rate * rho.rho_10_im + self.omega * rho.rho_10_re;

        DensityMatrix2x2 {
            rho_00: rho.rho_00 + d_rho_00 * dt,
            rho_01_re: rho.rho_01_re + d_rho_01_re * dt,
            rho_01_im: rho.rho_01_im + d_rho_01_im * dt,
            rho_10_re: rho.rho_10_re + d_rho_10_re * dt,
            rho_10_im: rho.rho_10_im + d_rho_10_im * dt,
            rho_11: rho.rho_11 + d_rho_11 * dt,
        }
    }

    /// Simulate evolution for a given duration.
    pub fn evolve(&self, initial: &DensityMatrix2x2, duration: f64, n_steps: usize) -> DensityMatrix2x2 {
        let dt = duration / n_steps as f64;
        let mut rho = *initial;
        for _ in 0..n_steps {
            rho = self.step(&rho, dt);
        }
        rho
    }

    /// Analytical T1 decay: rho_11(t) = rho_11(0) * exp(-gamma_1 * t).
    pub fn analytical_t1_decay(&self, rho_11_0: f64, t: f64) -> f64 {
        rho_11_0 * (-self.gamma_1 * t).exp()
    }

    /// Analytical T2 decay: |rho_01(t)| = |rho_01(0)| * exp(-t/T2).
    pub fn analytical_t2_decay(&self, rho_01_mag_0: f64, t: f64) -> f64 {
        let t2_rate = self.gamma_1 / 2.0 + self.gamma_phi;
        rho_01_mag_0 * (-t2_rate * t).exp()
    }
}

// ---------------------------------------------------------------------------
// Gate Fidelity
// ---------------------------------------------------------------------------

/// Gate fidelity calculations.
pub struct GateFidelity;

impl GateFidelity {
    /// First-order gate infidelity.
    ///
    /// r ~ t_gate/T1 + t_gate/T2
    pub fn infidelity_first_order(t_gate: f64, t1: f64, t2: f64) -> f64 {
        t_gate / t1 + t_gate / t2
    }

    /// Average gate fidelity for single-qubit depolarizing channel.
    ///
    /// F = (1 + p) / 2 where p is the depolarizing parameter.
    pub fn depolarizing_fidelity(p: f64) -> f64 {
        (1.0 + p) / 2.0
    }

    /// Depolarizing parameter from T1 and T2.
    ///
    /// p ~ (1/3)(exp(-t/T1) + 2*exp(-t/T2)) for a single qubit.
    pub fn depolarizing_parameter(t_gate: f64, t1: f64, t2: f64) -> f64 {
        let p = ((-t_gate / t1).exp() + 2.0 * (-t_gate / t2).exp()) / 3.0;
        p.clamp(0.0, 1.0)
    }

    /// Number of gates before reaching target error rate.
    ///
    /// N_gates ~ target_error / error_per_gate
    pub fn gates_before_error(error_per_gate: f64, target_error: f64) -> f64 {
        if error_per_gate <= 0.0 {
            return f64::INFINITY;
        }
        target_error / error_per_gate
    }
}

// ---------------------------------------------------------------------------
// Randomized Benchmarking
// ---------------------------------------------------------------------------

/// Randomized benchmarking simulation.
///
/// Models fidelity decay: F = A * p^m + B
/// where m is the number of Clifford gates, p is the depolarizing parameter.
pub struct RandomizedBenchmarking {
    /// Depolarizing parameter per Clifford gate.
    p: f64,
    /// SPAM amplitude (ideally 0.5 for d=2).
    a: f64,
    /// SPAM offset (ideally 0.5 for d=2).
    b: f64,
}

impl RandomizedBenchmarking {
    /// Create from depolarizing parameter.
    pub fn new(p: f64) -> Self {
        Self { p, a: 0.5, b: 0.5 }
    }

    /// Create from T1, T2, and gate time.
    pub fn from_relaxation(t1: f64, t2: f64, t_gate: f64) -> Self {
        let p = GateFidelity::depolarizing_parameter(t_gate, t1, t2);
        Self { p, a: 0.5, b: 0.5 }
    }

    /// Set SPAM parameters.
    pub fn with_spam(mut self, a: f64, b: f64) -> Self {
        self.a = a;
        self.b = b;
        self
    }

    /// Sequence fidelity at m Clifford gates.
    ///
    /// F(m) = A * p^m + B
    pub fn fidelity(&self, m: usize) -> f64 {
        self.a * self.p.powi(m as i32) + self.b
    }

    /// Error per Clifford gate.
    ///
    /// r = (1 - p)(d - 1)/d where d = 2 for single qubit.
    pub fn error_per_clifford(&self) -> f64 {
        (1.0 - self.p) * (2.0 - 1.0) / 2.0
    }

    /// Generate RB decay curve.
    ///
    /// Returns (sequence_lengths, fidelities).
    pub fn decay_curve(&self, max_length: usize, n_points: usize) -> (Vec<usize>, Vec<f64>) {
        let mut lengths = Vec::with_capacity(n_points);
        let mut fidelities = Vec::with_capacity(n_points);
        for i in 0..n_points {
            let m = if n_points <= 1 {
                0
            } else {
                (max_length * i) / (n_points - 1)
            };
            lengths.push(m);
            fidelities.push(self.fidelity(m));
        }
        (lengths, fidelities)
    }

    /// Fit RB data to extract p and error per Clifford.
    ///
    /// Uses logarithmic linear regression on F(m) - B_est.
    /// Returns (p, error_per_clifford).
    pub fn fit_rb_data(lengths: &[usize], fidelities: &[f64]) -> (f64, f64) {
        if lengths.len() < 2 {
            return (1.0, 0.0);
        }

        // Estimate B as the final fidelity value (asymptote)
        let b_est = 0.5;

        let mut sum_m = 0.0;
        let mut sum_lnf = 0.0;
        let mut sum_m_lnf = 0.0;
        let mut sum_m2 = 0.0;
        let mut n = 0.0;

        for (&m, &f) in lengths.iter().zip(fidelities.iter()) {
            let corrected = f - b_est;
            if corrected > 1e-10 {
                let lnf = corrected.ln();
                let mf = m as f64;
                sum_m += mf;
                sum_lnf += lnf;
                sum_m_lnf += mf * lnf;
                sum_m2 += mf * mf;
                n += 1.0;
            }
        }

        if n < 2.0 {
            return (1.0, 0.0);
        }

        let slope = (n * sum_m_lnf - sum_m * sum_lnf) / (n * sum_m2 - sum_m * sum_m);
        let p = slope.exp().clamp(0.0, 1.0);
        let epc = (1.0 - p) / 2.0; // (d-1)/d = 1/2 for d=2
        (p, epc)
    }
}

// ---------------------------------------------------------------------------
// Simple PRNG (for deterministic noise generation)
// ---------------------------------------------------------------------------

/// Simple xorshift64 PRNG for deterministic noise.
struct SimplePrng {
    state: u64,
}

impl SimplePrng {
    fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 { 0x12345678_9ABCDEF0 } else { seed },
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Box-Muller transform for Gaussian random numbers.
    fn gaussian(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-15);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-6;

    // --- BlochVector tests ---

    #[test]
    fn test_bloch_zero_state() {
        let bv = BlochVector::zero_state();
        assert!((bv.x).abs() < EPSILON);
        assert!((bv.y).abs() < EPSILON);
        assert!((bv.z - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_bloch_one_state() {
        let bv = BlochVector::one_state();
        assert!((bv.z + 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_bloch_plus_state() {
        let bv = BlochVector::plus_state();
        assert!((bv.x - 1.0).abs() < EPSILON);
        assert!((bv.y).abs() < EPSILON);
        assert!((bv.z).abs() < EPSILON);
    }

    #[test]
    fn test_bloch_from_angles_north_pole() {
        let bv = BlochVector::from_angles(0.0, 0.0);
        assert!((bv.z - 1.0).abs() < EPSILON);
        assert!(bv.x.abs() < EPSILON);
    }

    #[test]
    fn test_bloch_from_angles_south_pole() {
        let bv = BlochVector::from_angles(PI, 0.0);
        assert!((bv.z + 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_bloch_from_angles_equator() {
        let bv = BlochVector::from_angles(PI / 2.0, 0.0);
        assert!((bv.x - 1.0).abs() < EPSILON);
        assert!(bv.z.abs() < EPSILON);
    }

    #[test]
    fn test_bloch_length_pure_state() {
        let bv = BlochVector::from_angles(PI / 3.0, PI / 4.0);
        assert!((bv.length() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_bloch_length_mixed_state() {
        let bv = BlochVector::new(0.3, 0.0, 0.0);
        assert!((bv.length() - 0.3).abs() < EPSILON);
    }

    #[test]
    fn test_bloch_normalize() {
        let bv = BlochVector::new(3.0, 4.0, 0.0).normalize();
        assert!((bv.length() - 1.0).abs() < EPSILON);
        assert!((bv.x - 0.6).abs() < EPSILON);
        assert!((bv.y - 0.8).abs() < EPSILON);
    }

    #[test]
    fn test_bloch_transverse() {
        let bv = BlochVector::new(0.6, 0.8, 0.0);
        assert!((bv.transverse() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_bloch_dot_product() {
        let a = BlochVector::new(1.0, 0.0, 0.0);
        let b = BlochVector::new(0.0, 1.0, 0.0);
        assert!(a.dot(&b).abs() < EPSILON);

        let c = BlochVector::new(1.0, 0.0, 0.0);
        assert!((a.dot(&c) - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_bloch_cross_product() {
        let x = BlochVector::new(1.0, 0.0, 0.0);
        let y = BlochVector::new(0.0, 1.0, 0.0);
        let z = x.cross(&y);
        assert!((z.z - 1.0).abs() < EPSILON);
    }

    // --- DecoherenceCharacterizer tests ---

    #[test]
    fn test_characterizer_creation() {
        let dc = DecoherenceCharacterizer::new(50.0, 30.0, 10.0);
        assert!((dc.t1_us() - 50.0).abs() < EPSILON);
        assert!((dc.t2_us() - 30.0).abs() < EPSILON);
        assert!((dc.t2_star_us() - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_t2_clamped_to_2t1() {
        let dc = DecoherenceCharacterizer::new(10.0, 50.0, 5.0);
        assert!((dc.t2_us() - 20.0).abs() < EPSILON); // clamped to 2*T1
    }

    #[test]
    fn test_t2_star_clamped_to_t2() {
        let dc = DecoherenceCharacterizer::new(50.0, 30.0, 100.0);
        assert!((dc.t2_star_us() - 30.0).abs() < EPSILON); // clamped to T2
    }

    #[test]
    fn test_t_phi_calculation() {
        // 1/T2 = 1/(2*T1) + 1/T_phi
        // T1=50, T2=20: 1/20 = 1/100 + 1/T_phi => 1/T_phi = 0.04 => T_phi=25
        let dc = DecoherenceCharacterizer::new(50.0, 20.0, 10.0);
        assert!((dc.t_phi_us() - 25.0).abs() < 0.01);
    }

    #[test]
    fn test_t_phi_infinite_when_t2_equals_2t1() {
        let dc = DecoherenceCharacterizer::new(50.0, 100.0, 50.0);
        assert!(dc.t_phi_us().is_infinite());
    }

    #[test]
    fn test_t1_decay_initial() {
        let dc = DecoherenceCharacterizer::new(50.0, 30.0, 10.0);
        let (times, probs) = dc.t1_decay(0.0, 200.0, 100);
        assert_eq!(times.len(), 100);
        // At t=0, P(|1>) = exp(0) = 1.0
        assert!((probs[0] - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_t1_decay_monotonic() {
        let dc = DecoherenceCharacterizer::new(50.0, 30.0, 10.0);
        let (_times, probs) = dc.t1_decay(0.0, 200.0, 50);
        for i in 1..probs.len() {
            assert!(probs[i] <= probs[i - 1] + EPSILON);
        }
    }

    #[test]
    fn test_t1_decay_at_t1() {
        let dc = DecoherenceCharacterizer::new(50.0, 30.0, 10.0);
        let (times, probs) = dc.t1_decay(0.0, 50.0, 2);
        // At tau = T1, P = exp(-1) ~ 0.3679
        assert!((times[1] - 50.0).abs() < EPSILON);
        assert!((probs[1] - (-1.0_f64).exp()).abs() < EPSILON);
    }

    #[test]
    fn test_t1_decay_noisy() {
        let dc = DecoherenceCharacterizer::new(50.0, 30.0, 10.0);
        let (times, probs) = dc.t1_decay_noisy(0.0, 200.0, 50, 0.9, 0.05, 0.01, 42);
        assert_eq!(times.len(), 50);
        assert_eq!(probs.len(), 50);
        // All probabilities should be between 0 and 1
        for &p in &probs {
            assert!(p >= 0.0 && p <= 1.0);
        }
    }

    #[test]
    fn test_fit_t1() {
        let dc = DecoherenceCharacterizer::new(50.0, 30.0, 10.0);
        let (times, probs) = dc.t1_decay(0.0, 200.0, 100);
        let t1_fit = DecoherenceCharacterizer::fit_t1(&times, &probs);
        assert!((t1_fit - 50.0).abs() < 1.0); // Should be close to true T1
    }

    #[test]
    fn test_ramsey_fringe_envelope() {
        let dc = DecoherenceCharacterizer::new(50.0, 30.0, 10.0);
        let (_times, signal) = dc.ramsey_fringe(0.0, 100.0, 200, 0.0);
        // With zero detuning, cos(0) = 1, so signal = exp(-t/T2*)
        assert!((signal[0] - 1.0).abs() < EPSILON);
        // Should decay
        assert!(signal[199] < signal[0]);
    }

    #[test]
    fn test_ramsey_fringe_oscillation() {
        let dc = DecoherenceCharacterizer::new(50.0, 30.0, 10.0);
        let (_times, signal) = dc.ramsey_fringe(0.0, 100.0, 1000, 2.0 * PI);
        // With detuning, signal should oscillate (have both positive and negative values)
        let has_positive = signal.iter().any(|&s| s > 0.1);
        let has_negative = signal.iter().any(|&s| s < -0.1);
        assert!(has_positive);
        assert!(has_negative);
    }

    #[test]
    fn test_hahn_echo_decay() {
        let dc = DecoherenceCharacterizer::new(50.0, 30.0, 10.0);
        let (times, signal) = dc.hahn_echo_decay(0.0, 100.0, 50);
        assert_eq!(times.len(), 50);
        // At tau = T2 = 30, signal = exp(-1) ~ 0.368
        // Find the closest point to tau = 30
        let idx = (30.0_f64 / 100.0 * 49.0).round() as usize;
        let expected = (-times[idx] / 30.0).exp();
        assert!((signal[idx] - expected).abs() < EPSILON);
    }

    #[test]
    fn test_hahn_echo_longer_than_ramsey() {
        let dc = DecoherenceCharacterizer::new(50.0, 30.0, 10.0);
        let tau = 20.0;
        // Compare at fixed time: Hahn echo should show more coherence than Ramsey
        let (_, hahn) = dc.hahn_echo_decay(tau, tau, 1);
        let (_, ramsey) = dc.ramsey_fringe(tau, tau, 1, 0.0); // zero detuning for fair comparison
        assert!(hahn[0] > ramsey[0]); // T2 > T2* so Hahn echo decays slower
    }

    // --- DynamicalDecoupling tests ---

    #[test]
    fn test_cpmg_pulse_times() {
        let times = DynamicalDecoupling::cpmg_pulse_times(4, 100.0);
        assert_eq!(times.len(), 4);
        // Pulses at 12.5, 37.5, 62.5, 87.5
        assert!((times[0] - 12.5).abs() < EPSILON);
        assert!((times[1] - 37.5).abs() < EPSILON);
        assert!((times[2] - 62.5).abs() < EPSILON);
        assert!((times[3] - 87.5).abs() < EPSILON);
    }

    #[test]
    fn test_xy4_pulse_times() {
        let (times, axes) = DynamicalDecoupling::xy4_pulse_times(1, 100.0);
        assert_eq!(times.len(), 4);
        assert_eq!(axes.len(), 4);
        // Axes alternate: X, Y, X, Y
        assert!(axes[0]);  // X
        assert!(!axes[1]); // Y
        assert!(axes[2]);  // X
        assert!(!axes[3]); // Y
    }

    #[test]
    fn test_filter_function_single_pulse() {
        let pulse_times = DynamicalDecoupling::cpmg_pulse_times(1, 10.0);
        // At the passband frequency, filter function should be large
        let f_peak = DynamicalDecoupling::filter_function(&pulse_times, 10.0, PI / 10.0);
        let f_zero = DynamicalDecoupling::filter_function(&pulse_times, 10.0, 0.0);
        // Filter function at omega=0 should be zero for odd N (modulation averages out)
        assert!(f_zero.abs() < 1e-3);
        assert!(f_peak > 0.0);
    }

    #[test]
    fn test_effective_t2_increases_with_n() {
        let dd = DynamicalDecoupling::new(30.0, 1.0);
        let t2_1 = dd.effective_t2(1, 30.0);
        let t2_4 = dd.effective_t2(4, 30.0);
        let t2_16 = dd.effective_t2(16, 30.0);
        assert!(t2_4 > t2_1);
        assert!(t2_16 > t2_4);
    }

    #[test]
    fn test_cpmg_coherence_decays() {
        let dd = DynamicalDecoupling::new(30.0, 1.0);
        let c_short = dd.cpmg_coherence(4, 10.0);
        let c_long = dd.cpmg_coherence(4, 100.0);
        assert!(c_short > c_long);
    }

    // --- NoiseSpectroscopy tests ---

    #[test]
    fn test_reconstruct_psd() {
        let n_list = vec![1, 2, 4, 8, 16];
        let tau = 100.0;
        let coherences = vec![0.9, 0.85, 0.8, 0.75, 0.7];
        let psd = NoiseSpectroscopy::reconstruct_psd(&n_list, tau, &coherences);
        assert_eq!(psd.len(), 5);
        // Frequencies should increase
        for i in 1..psd.len() {
            assert!(psd[i].0 > psd[i - 1].0);
        }
    }

    #[test]
    fn test_fit_one_over_f() {
        // Generate synthetic 1/f noise PSD: S(f) = 2.0 / f^1.0
        let psd: Vec<(f64, f64)> = (1..=10)
            .map(|i| {
                let f = i as f64;
                (f, 2.0 / f)
            })
            .collect();
        let (a, alpha) = NoiseSpectroscopy::fit_one_over_f(&psd);
        assert!((alpha - 1.0).abs() < 0.1);
        assert!((a - 2.0).abs() < 0.5);
    }

    #[test]
    fn test_white_noise_floor() {
        let psd = vec![(1.0, 10.0), (2.0, 5.0), (3.0, 3.0), (4.0, 2.0)];
        let floor = NoiseSpectroscopy::white_noise_floor(&psd);
        // Highest-frequency quartile is just the last point: 2.0
        assert!((floor - 2.0).abs() < EPSILON);
    }

    // --- BlochSimulator tests ---

    #[test]
    fn test_bloch_t1_relaxation() {
        let sim = BlochSimulator::new(50.0, 30.0);
        let initial = BlochVector::one_state(); // z = -1
        let final_state = sim.free_evolution(&initial, 500.0, 10000);
        // After 10*T1, should be close to equilibrium (z ~ +1)
        assert!(final_state.z > 0.9);
    }

    #[test]
    fn test_bloch_t2_relaxation() {
        let sim = BlochSimulator::new(100.0, 20.0);
        let initial = BlochVector::plus_state(); // x = 1, y = 0, z = 0
        let final_state = sim.free_evolution(&initial, 200.0, 10000);
        // After 10*T2, transverse components should be ~ 0
        assert!(final_state.transverse() < 0.1);
    }

    #[test]
    fn test_bloch_precession() {
        let sim = BlochSimulator::new(1e6, 1e6); // Very long relaxation
        let initial = BlochVector::plus_state(); // x = 1
        // Evolve in B0 field for half a revolution
        let b0 = 1.0;
        let period = 1.0; // omega = gamma * B0 = 2*pi * 1 => period = 1
        let final_state = sim.evolve_in_field(&initial, b0, period / 2.0, 5000);
        // After half period, x should be near -1 (rotated by pi)
        assert!((final_state.x + 1.0).abs() < 0.1);
    }

    #[test]
    fn test_bloch_trajectory_length() {
        let sim = BlochSimulator::new(50.0, 30.0);
        let initial = BlochVector::plus_state();
        let traj = sim.record_free_evolution(&initial, 100.0, 100);
        assert_eq!(traj.len(), 101); // initial + 100 steps
    }

    // --- LindbladSimulator tests ---

    #[test]
    fn test_lindblad_t1_decay() {
        let sim = LindbladSimulator::from_t1_t2(50.0, 30.0, 0.0);
        let initial = DensityMatrix2x2::one_state();
        let final_rho = sim.evolve(&initial, 500.0, 50000);
        // After 10*T1, should be mostly in |0>
        assert!(final_rho.rho_00 > 0.99);
        assert!(final_rho.rho_11 < 0.01);
    }

    #[test]
    fn test_lindblad_t2_dephasing() {
        let sim = LindbladSimulator::from_t1_t2(100.0, 20.0, 0.0);
        let initial = DensityMatrix2x2::plus_state();
        let final_rho = sim.evolve(&initial, 200.0, 50000);
        // After 10*T2, off-diagonal should be ~ 0
        let coherence = (final_rho.rho_01_re * final_rho.rho_01_re
            + final_rho.rho_01_im * final_rho.rho_01_im)
            .sqrt();
        assert!(coherence < 0.01);
    }

    #[test]
    fn test_lindblad_trace_preservation() {
        let sim = LindbladSimulator::from_t1_t2(50.0, 30.0, 1.0);
        let initial = DensityMatrix2x2::plus_state();
        let final_rho = sim.evolve(&initial, 50.0, 10000);
        assert!((final_rho.trace() - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_lindblad_purity_decreases() {
        let sim = LindbladSimulator::from_t1_t2(50.0, 30.0, 0.0);
        let initial = DensityMatrix2x2::plus_state();
        assert!((initial.purity() - 1.0).abs() < EPSILON); // Pure state
        let mid = sim.evolve(&initial, 25.0, 5000);
        assert!(mid.purity() < 1.0); // Mixed after some decoherence
    }

    #[test]
    fn test_lindblad_analytical_t1() {
        let sim = LindbladSimulator::from_t1_t2(50.0, 30.0, 0.0);
        let t = 25.0;
        let analytical = sim.analytical_t1_decay(1.0, t);
        let expected = (-t / 50.0).exp();
        assert!((analytical - expected).abs() < EPSILON);
    }

    #[test]
    fn test_lindblad_analytical_t2() {
        let sim = LindbladSimulator::from_t1_t2(50.0, 30.0, 0.0);
        let t = 15.0;
        let analytical = sim.analytical_t2_decay(0.5, t);
        let expected = 0.5 * (-t / 30.0).exp();
        assert!((analytical - expected).abs() < EPSILON);
    }

    // --- DensityMatrix tests ---

    #[test]
    fn test_density_matrix_trace() {
        let rho = DensityMatrix2x2::plus_state();
        assert!((rho.trace() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_density_matrix_purity_pure() {
        let rho = DensityMatrix2x2::zero_state();
        assert!((rho.purity() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_density_matrix_purity_mixed() {
        let rho = DensityMatrix2x2::maximally_mixed();
        assert!((rho.purity() - 0.5).abs() < EPSILON);
    }

    #[test]
    fn test_density_matrix_bloch_roundtrip() {
        let bv = BlochVector::from_angles(PI / 3.0, PI / 5.0);
        let rho = DensityMatrix2x2::from_bloch(&bv);
        let bv2 = rho.to_bloch();
        assert!((bv.x - bv2.x).abs() < EPSILON);
        assert!((bv.y - bv2.y).abs() < EPSILON);
        assert!((bv.z - bv2.z).abs() < EPSILON);
    }

    #[test]
    fn test_density_matrix_fidelity_zero() {
        let rho = DensityMatrix2x2::zero_state();
        assert!((rho.fidelity_with_pure_zero() - 1.0).abs() < EPSILON);
        assert!(rho.fidelity_with_pure_one().abs() < EPSILON);
    }

    #[test]
    fn test_density_matrix_fidelity_plus() {
        let rho = DensityMatrix2x2::plus_state();
        assert!((rho.fidelity_with_plus() - 1.0).abs() < EPSILON);
    }

    // --- GateFidelity tests ---

    #[test]
    fn test_gate_infidelity_first_order() {
        let r = GateFidelity::infidelity_first_order(0.05, 50.0, 30.0);
        let expected = 0.05 / 50.0 + 0.05 / 30.0;
        assert!((r - expected).abs() < EPSILON);
    }

    #[test]
    fn test_depolarizing_fidelity() {
        assert!((GateFidelity::depolarizing_fidelity(1.0) - 1.0).abs() < EPSILON);
        assert!((GateFidelity::depolarizing_fidelity(0.0) - 0.5).abs() < EPSILON);
    }

    #[test]
    fn test_depolarizing_parameter() {
        let p = GateFidelity::depolarizing_parameter(0.0, 50.0, 30.0);
        assert!((p - 1.0).abs() < EPSILON); // No time elapsed => p = 1
    }

    #[test]
    fn test_gates_before_error() {
        let n = GateFidelity::gates_before_error(0.001, 0.1);
        assert!((n - 100.0).abs() < EPSILON);
    }

    // --- RandomizedBenchmarking tests ---

    #[test]
    fn test_rb_fidelity_at_zero() {
        let rb = RandomizedBenchmarking::new(0.99);
        // F(0) = A * p^0 + B = A + B = 0.5 + 0.5 = 1.0
        assert!((rb.fidelity(0) - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_rb_fidelity_decays() {
        let rb = RandomizedBenchmarking::new(0.99);
        let f0 = rb.fidelity(0);
        let f100 = rb.fidelity(100);
        let f1000 = rb.fidelity(1000);
        assert!(f0 > f100);
        assert!(f100 > f1000);
    }

    #[test]
    fn test_rb_error_per_clifford() {
        let rb = RandomizedBenchmarking::new(0.99);
        let epc = rb.error_per_clifford();
        // r = (1-p)(d-1)/d = 0.01 * 1/2 = 0.005
        assert!((epc - 0.005).abs() < EPSILON);
    }

    #[test]
    fn test_rb_decay_curve() {
        let rb = RandomizedBenchmarking::new(0.99);
        let (lengths, fidelities) = rb.decay_curve(1000, 20);
        assert_eq!(lengths.len(), 20);
        assert_eq!(fidelities.len(), 20);
        // Monotonically decreasing
        for i in 1..fidelities.len() {
            assert!(fidelities[i] <= fidelities[i - 1] + EPSILON);
        }
    }

    #[test]
    fn test_rb_from_relaxation() {
        let rb = RandomizedBenchmarking::from_relaxation(50.0, 30.0, 0.05);
        let epc = rb.error_per_clifford();
        assert!(epc > 0.0);
        assert!(epc < 0.01); // Should be small for short gates
    }

    #[test]
    fn test_rb_fit() {
        let rb = RandomizedBenchmarking::new(0.98);
        let (lengths, fidelities) = rb.decay_curve(500, 50);
        let (p_fit, epc_fit) = RandomizedBenchmarking::fit_rb_data(&lengths, &fidelities);
        assert!((p_fit - 0.98).abs() < 0.02);
        assert!((epc_fit - 0.01).abs() < 0.005);
    }

    #[test]
    fn test_rb_with_spam() {
        let rb = RandomizedBenchmarking::new(0.99).with_spam(0.48, 0.49);
        let f0 = rb.fidelity(0);
        assert!((f0 - (0.48 + 0.49)).abs() < EPSILON);
    }

    // --- Gate fidelity from characterizer ---

    #[test]
    fn test_gate_fidelity_short_gate() {
        let dc = DecoherenceCharacterizer::new(50.0, 30.0, 10.0);
        let f = dc.gate_fidelity(0.05);
        // Should be close to 1.0 for very short gate times
        assert!(f > 0.99);
    }

    #[test]
    fn test_gate_fidelity_precise_bounds() {
        let dc = DecoherenceCharacterizer::new(50.0, 30.0, 10.0);
        let f = dc.gate_fidelity_precise(0.05);
        assert!(f > 0.0 && f <= 1.0);
    }
}

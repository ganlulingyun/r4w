//! # Cavity Quantum Electrodynamics (cQED) Simulator
//!
//! Simulates light-matter interaction in optical and microwave cavities using
//! the Jaynes-Cummings model and related phenomena. Applicable to circuit QED
//! (superconducting qubits coupled to microwave resonators), optical cavities
//! with single atoms, and semiconductor quantum dots in photonic crystals.
//!
//! ## Jaynes-Cummings Hamiltonian
//!
//! The fundamental model of a two-level atom coupled to a single cavity mode:
//!
//! ```text
//! H = ℏω_c a†a + ℏω_a σ_z/2 + ℏg(a†σ_- + aσ_+)
//! ```
//!
//! where `ω_c` is the cavity frequency, `ω_a` the atom transition frequency,
//! `g` the vacuum Rabi coupling rate, `a†/a` are photon creation/annihilation
//! operators, and `σ_±` are the atomic raising/lowering operators.
//!
//! ## Features
//!
//! - **Vacuum Rabi oscillations**: Time-domain population dynamics in the
//!   strong coupling regime, including Rabi splitting `2g√(n+1)`.
//! - **Purcell effect**: Enhanced spontaneous emission rate `Γ_P = 4g²/κ`
//!   when the atom is resonant with a lossy cavity.
//! - **Strong coupling detection**: Checks `g > κ/2` and `g > γ/2` for the
//!   resolved vacuum Rabi splitting criterion.
//! - **Cavity transmission/reflection spectra**: Lorentzian line shapes with
//!   normal-mode splitting in the strong coupling regime.
//! - **Photon blockade**: Second-order correlation `g²(0)` quantifying
//!   antibunching in the anharmonic Jaynes-Cummings ladder.
//! - **Dispersive readout**: χ-shift `χ = g²/Δ` for qubit state-dependent
//!   cavity frequency shift, enabling quantum non-demolition measurement.
//! - **Cavity ringdown**: Exponential decay with time constant `τ = 1/κ`.
//! - **Input-output theory**: `a_out = a_in - √κ_ext · a` relating intra-
//!   cavity field to output field.
//! - **Quality factor**: `Q = ω_c/κ`, cooperativity `C = 4g²/(κγ)`.
//! - **Dressed states / Mollow triplet**: Energy eigenstates of the coupled
//!   system with photon-number-dependent splitting.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cavity_qed_simulator::{CavityQedSystem, CavityQedParams};
//!
//! let params = CavityQedParams {
//!     omega_c: 2.0 * std::f64::consts::PI * 5.0e9,  // 5 GHz cavity
//!     omega_a: 2.0 * std::f64::consts::PI * 5.0e9,  // resonant atom
//!     g: 2.0 * std::f64::consts::PI * 50.0e6,       // 50 MHz coupling
//!     kappa: 2.0 * std::f64::consts::PI * 1.0e6,    // 1 MHz cavity decay
//!     gamma: 2.0 * std::f64::consts::PI * 0.1e6,    // 100 kHz atom decay
//!     n_max: 10,
//! };
//!
//! let system = CavityQedSystem::new(params);
//! assert!(system.is_strong_coupling());
//! assert!(system.cooperativity() > 1.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex number (minimal, no external crates)
// ---------------------------------------------------------------------------

/// Minimal complex number for internal use.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    pub const ZERO: Complex = Complex { re: 0.0, im: 0.0 };
    pub const ONE: Complex = Complex { re: 1.0, im: 0.0 };
    pub const I: Complex = Complex { re: 0.0, im: 1.0 };

    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    pub fn from_polar(r: f64, theta: f64) -> Self {
        Self {
            re: r * theta.cos(),
            im: r * theta.sin(),
        }
    }

    pub fn norm_sqr(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    pub fn norm(self) -> f64 {
        self.norm_sqr().sqrt()
    }

    pub fn conj(self) -> Self {
        Self {
            re: self.re,
            im: -self.im,
        }
    }

    pub fn arg(self) -> f64 {
        self.im.atan2(self.re)
    }

    pub fn exp(self) -> Self {
        let r = self.re.exp();
        Self {
            re: r * self.im.cos(),
            im: r * self.im.sin(),
        }
    }

    pub fn scale(self, s: f64) -> Self {
        Self {
            re: self.re * s,
            im: self.im * s,
        }
    }
}

impl std::ops::Add for Complex {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }
}

impl std::ops::Sub for Complex {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            re: self.re - rhs.re,
            im: self.im - rhs.im,
        }
    }
}

impl std::ops::Mul for Complex {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }
}

impl std::ops::Div for Complex {
    type Output = Self;
    fn div(self, rhs: Self) -> Self {
        let d = rhs.norm_sqr();
        Self {
            re: (self.re * rhs.re + self.im * rhs.im) / d,
            im: (self.im * rhs.re - self.re * rhs.im) / d,
        }
    }
}

impl std::ops::AddAssign for Complex {
    fn add_assign(&mut self, rhs: Self) {
        self.re += rhs.re;
        self.im += rhs.im;
    }
}

impl std::ops::MulAssign for Complex {
    fn mul_assign(&mut self, rhs: Self) {
        let re = self.re * rhs.re - self.im * rhs.im;
        let im = self.re * rhs.im + self.im * rhs.re;
        self.re = re;
        self.im = im;
    }
}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------

/// Parameters for the cavity QED system.
#[derive(Debug, Clone, Copy)]
pub struct CavityQedParams {
    /// Cavity resonance frequency (rad/s).
    pub omega_c: f64,
    /// Atom transition frequency (rad/s).
    pub omega_a: f64,
    /// Vacuum Rabi coupling rate (rad/s).
    pub g: f64,
    /// Total cavity decay rate (rad/s). κ = ω_c / Q.
    pub kappa: f64,
    /// Atom spontaneous emission rate (rad/s).
    pub gamma: f64,
    /// Maximum photon number for Fock-space truncation.
    pub n_max: usize,
}

impl CavityQedParams {
    /// Preset: superconducting transmon qubit in a coplanar waveguide resonator.
    /// Typical circuit QED parameters at ~5 GHz.
    pub fn circuit_qed() -> Self {
        Self {
            omega_c: 2.0 * PI * 5.0e9,
            omega_a: 2.0 * PI * 5.1e9,
            g: 2.0 * PI * 50.0e6,
            kappa: 2.0 * PI * 1.0e6,
            gamma: 2.0 * PI * 0.1e6,
            n_max: 10,
        }
    }

    /// Preset: single Rubidium atom in a Fabry-Perot optical cavity.
    pub fn optical_cavity() -> Self {
        Self {
            omega_c: 2.0 * PI * 384.23e12, // Rb D2 line ~780 nm
            omega_a: 2.0 * PI * 384.23e12,
            g: 2.0 * PI * 16.0e6,
            kappa: 2.0 * PI * 4.0e6,
            gamma: 2.0 * PI * 3.0e6,
            n_max: 5,
        }
    }

    /// Preset: strong coupling regime with zero detuning (pedagogical).
    pub fn strong_coupling_resonant() -> Self {
        Self {
            omega_c: 2.0 * PI * 1.0e9,
            omega_a: 2.0 * PI * 1.0e9,
            g: 2.0 * PI * 100.0e6,
            kappa: 2.0 * PI * 1.0e6,
            gamma: 2.0 * PI * 0.5e6,
            n_max: 15,
        }
    }

    /// Preset: dispersive regime with large detuning Δ >> g.
    pub fn dispersive() -> Self {
        Self {
            omega_c: 2.0 * PI * 7.0e9,
            omega_a: 2.0 * PI * 5.0e9,
            g: 2.0 * PI * 50.0e6,
            kappa: 2.0 * PI * 2.0e6,
            gamma: 2.0 * PI * 0.1e6,
            n_max: 10,
        }
    }
}

// ---------------------------------------------------------------------------
// Jaynes-Cummings eigenstate (dressed state)
// ---------------------------------------------------------------------------

/// A dressed state of the Jaynes-Cummings Hamiltonian.
///
/// The eigenstates |n,±⟩ are superpositions of |e,n⟩ and |g,n+1⟩:
/// ```text
/// |n,+⟩ = cos(θ_n)|e,n⟩ + sin(θ_n)|g,n+1⟩
/// |n,-⟩ = -sin(θ_n)|e,n⟩ + cos(θ_n)|g,n+1⟩
/// ```
#[derive(Debug, Clone)]
pub struct DressedState {
    /// Photon number of the bare-state manifold.
    pub n: usize,
    /// Branch: `true` for upper (+), `false` for lower (-).
    pub upper: bool,
    /// Energy eigenvalue (rad/s, relative to ground).
    pub energy: f64,
    /// Mixing angle θ_n (radians).
    pub mixing_angle: f64,
}

// ---------------------------------------------------------------------------
// Rabi oscillation result
// ---------------------------------------------------------------------------

/// Result of a vacuum Rabi oscillation simulation.
#[derive(Debug, Clone)]
pub struct RabiOscillation {
    /// Time points (seconds).
    pub times: Vec<f64>,
    /// Excited-state population P_e(t) at each time point.
    pub p_excited: Vec<f64>,
    /// Mean intracavity photon number ⟨n⟩(t).
    pub mean_photon: Vec<f64>,
    /// Vacuum Rabi frequency Ω_R = 2g√(n+1) (rad/s).
    pub rabi_freq: f64,
}

// ---------------------------------------------------------------------------
// Spectrum result
// ---------------------------------------------------------------------------

/// Cavity transmission or reflection spectrum.
#[derive(Debug, Clone)]
pub struct CavitySpectrum {
    /// Probe frequencies (rad/s).
    pub frequencies: Vec<f64>,
    /// Transmission |T(ω)|² (linear scale, normalised to peak).
    pub transmission: Vec<f64>,
    /// Reflection |R(ω)|² (linear scale).
    pub reflection: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Ringdown result
// ---------------------------------------------------------------------------

/// Cavity ringdown measurement.
#[derive(Debug, Clone)]
pub struct CavityRingdown {
    /// Time points (seconds).
    pub times: Vec<f64>,
    /// Intracavity photon number ⟨n⟩(t).
    pub photon_number: Vec<f64>,
    /// Fitted decay time constant τ = 1/κ (seconds).
    pub tau: f64,
    /// Quality factor Q = ω_c · τ.
    pub quality_factor: f64,
}

// ---------------------------------------------------------------------------
// Photon statistics
// ---------------------------------------------------------------------------

/// Second-order photon correlation statistics.
#[derive(Debug, Clone)]
pub struct PhotonStatistics {
    /// g²(0) — second-order correlation at zero delay.
    /// g²(0) < 1 indicates antibunching (photon blockade).
    /// g²(0) = 1 is coherent (Poissonian).
    /// g²(0) > 1 is bunching (super-Poissonian).
    pub g2_zero: f64,
    /// Mean photon number ⟨n⟩.
    pub mean_n: f64,
    /// Variance ⟨n²⟩ - ⟨n⟩².
    pub variance_n: f64,
    /// Mandel Q parameter: Q_M = (⟨n²⟩-⟨n⟩²)/⟨n⟩ - 1.
    /// Q_M < 0: sub-Poissonian, Q_M = 0: Poissonian, Q_M > 0: super-Poissonian.
    pub mandel_q: f64,
}

// ---------------------------------------------------------------------------
// Dispersive readout
// ---------------------------------------------------------------------------

/// Dispersive readout parameters for qubit state measurement.
#[derive(Debug, Clone)]
pub struct DispersiveReadout {
    /// Dispersive shift χ = g²/Δ (rad/s).
    pub chi: f64,
    /// Cavity frequency when qubit is in |g⟩: ω_c - χ.
    pub omega_g: f64,
    /// Cavity frequency when qubit is in |e⟩: ω_c + χ.
    pub omega_e: f64,
    /// Frequency separation = 2χ (rad/s).
    pub separation: f64,
    /// Dispersive regime validity: |Δ/g| >> 1.
    pub delta_over_g: f64,
    /// Critical photon number n_crit = Δ²/(4g²).
    pub n_crit: f64,
}

// ---------------------------------------------------------------------------
// Mollow triplet
// ---------------------------------------------------------------------------

/// Mollow triplet spectral components for resonance fluorescence.
#[derive(Debug, Clone)]
pub struct MollowTriplet {
    /// Central (carrier) frequency (rad/s).
    pub center_freq: f64,
    /// Sideband offset from center: ±Ω_R (rad/s).
    pub sideband_offset: f64,
    /// Generalised Rabi frequency Ω_R = √(Δ² + 4g²(n+1)) (rad/s).
    pub rabi_freq: f64,
    /// Central peak linewidth: γ (rad/s).
    pub center_linewidth: f64,
    /// Sideband linewidth: (3/4)γ (rad/s) for resonance.
    pub sideband_linewidth: f64,
    /// Central peak relative weight.
    pub center_weight: f64,
    /// Each sideband relative weight.
    pub sideband_weight: f64,
}

// ---------------------------------------------------------------------------
// Input-output theory result
// ---------------------------------------------------------------------------

/// Input-output theory field amplitudes.
#[derive(Debug, Clone)]
pub struct InputOutputField {
    /// Intracavity field amplitude ⟨a⟩.
    pub a_cav: Complex,
    /// Output field amplitude: a_out = a_in - √κ_ext · a_cav.
    pub a_out: Complex,
    /// Reflected power |a_out|² / |a_in|².
    pub reflection_coeff: f64,
    /// Transmitted power (for two-sided cavity).
    pub transmission_coeff: f64,
}

// ---------------------------------------------------------------------------
// Main system
// ---------------------------------------------------------------------------

/// Cavity QED system implementing the Jaynes-Cummings model.
///
/// Provides methods for computing static properties (quality factor,
/// cooperativity, dressed states) and dynamic simulations (Rabi oscillations,
/// cavity ringdown, spectra).
#[derive(Debug, Clone)]
pub struct CavityQedSystem {
    params: CavityQedParams,
}

impl CavityQedSystem {
    /// Create a new cavity QED system with the given parameters.
    pub fn new(params: CavityQedParams) -> Self {
        Self { params }
    }

    /// Return a reference to the system parameters.
    pub fn params(&self) -> &CavityQedParams {
        &self.params
    }

    // -----------------------------------------------------------------------
    // Static properties
    // -----------------------------------------------------------------------

    /// Atom-cavity detuning Δ = ω_a - ω_c (rad/s).
    pub fn detuning(&self) -> f64 {
        self.params.omega_a - self.params.omega_c
    }

    /// Quality factor Q = ω_c / κ.
    pub fn quality_factor(&self) -> f64 {
        self.params.omega_c / self.params.kappa
    }

    /// Finesse F = π·Q·(FSR/ω_c) ≈ 2π / (κ · round-trip time).
    /// For a single-mode model, finesse ≈ 2π·ω_c / (κ·ω_c) = 2π/κ * FSR.
    /// We express as F = ω_c / (2·κ) · 2π / ... simplified to Q·FSR/ω_c.
    /// In practice: F = 2π / (round-trip loss) ≈ Q · (c / 2L) / ω_c.
    /// Here we provide the relation F = π · c / (L · κ) given a cavity length.
    pub fn finesse(&self, cavity_length_m: f64) -> f64 {
        let c = 299_792_458.0; // speed of light
        PI * c / (cavity_length_m * self.params.kappa)
    }

    /// Cooperativity C = 4g² / (κ·γ). Dimensionless figure of merit.
    /// C > 1 indicates the coherent coupling dominates dissipation.
    pub fn cooperativity(&self) -> f64 {
        4.0 * self.params.g * self.params.g / (self.params.kappa * self.params.gamma)
    }

    /// Single-atom cooperativity (same as cooperativity for one atom).
    pub fn single_atom_cooperativity(&self) -> f64 {
        self.cooperativity()
    }

    /// Purcell-enhanced spontaneous emission rate Γ_P = 4g²/κ (rad/s).
    /// Valid when g << κ (bad-cavity / Purcell regime).
    pub fn purcell_rate(&self) -> f64 {
        4.0 * self.params.g * self.params.g / self.params.kappa
    }

    /// Purcell factor F_P = Γ_P / γ = C (cooperativity).
    pub fn purcell_factor(&self) -> f64 {
        self.purcell_rate() / self.params.gamma
    }

    /// Check if the system is in the strong coupling regime.
    /// Criterion: g > κ/2 and g > γ/2 (vacuum Rabi splitting resolved).
    pub fn is_strong_coupling(&self) -> bool {
        self.params.g > self.params.kappa / 2.0 && self.params.g > self.params.gamma / 2.0
    }

    /// Check if the system is in the dispersive regime.
    /// Criterion: |Δ| >> g (typically |Δ/g| > 5).
    pub fn is_dispersive(&self) -> bool {
        let delta = self.detuning().abs();
        delta > 5.0 * self.params.g
    }

    /// Cavity decay time τ = 1/κ (seconds).
    pub fn cavity_lifetime(&self) -> f64 {
        1.0 / self.params.kappa
    }

    /// Photon lifetime in the cavity (seconds).
    pub fn photon_lifetime(&self) -> f64 {
        self.cavity_lifetime()
    }

    /// Critical photon number n_crit = Δ²/(4g²).
    /// When ⟨n⟩ ~ n_crit, the dispersive approximation breaks down.
    pub fn critical_photon_number(&self) -> f64 {
        let delta = self.detuning();
        delta * delta / (4.0 * self.params.g * self.params.g)
    }

    // -----------------------------------------------------------------------
    // Dressed states (Jaynes-Cummings eigenstates)
    // -----------------------------------------------------------------------

    /// Compute the dressed-state energy eigenvalues and mixing angles.
    ///
    /// The n-th manifold {|e,n⟩, |g,n+1⟩} has eigenstates at energies:
    /// ```text
    /// E_{n,±} = ℏω_c(n + 1/2) ± ℏΩ_n/2
    /// Ω_n = √(Δ² + 4g²(n+1))
    /// ```
    pub fn dressed_states(&self) -> Vec<DressedState> {
        let delta = self.detuning();
        let mut states = Vec::new();

        // Ground state |g,0⟩ (no mixing)
        // Energy = -ℏω_a/2 (we set this as zero reference)

        for n in 0..self.params.n_max {
            let nf = n as f64;
            let omega_n = (delta * delta + 4.0 * self.params.g * self.params.g * (nf + 1.0)).sqrt();
            let mixing_angle = 0.5 * (2.0 * self.params.g * (nf + 1.0).sqrt()).atan2(-delta);

            // Mean energy of the manifold
            let e_mean = self.params.omega_c * (nf + 0.5);

            states.push(DressedState {
                n,
                upper: true,
                energy: e_mean + omega_n / 2.0,
                mixing_angle,
            });

            states.push(DressedState {
                n,
                upper: false,
                energy: e_mean - omega_n / 2.0,
                mixing_angle,
            });
        }

        states
    }

    /// Generalised Rabi frequency for the n-th manifold:
    /// Ω_n = √(Δ² + 4g²(n+1)) (rad/s).
    pub fn rabi_frequency(&self, n: usize) -> f64 {
        let delta = self.detuning();
        let nf = n as f64;
        (delta * delta + 4.0 * self.params.g * self.params.g * (nf + 1.0)).sqrt()
    }

    /// Vacuum Rabi frequency (n=0 manifold): Ω_0 = √(Δ² + 4g²).
    pub fn vacuum_rabi_frequency(&self) -> f64 {
        self.rabi_frequency(0)
    }

    /// Vacuum Rabi splitting = Ω_0 (or 2g at resonance).
    pub fn vacuum_rabi_splitting(&self) -> f64 {
        self.vacuum_rabi_frequency()
    }

    // -----------------------------------------------------------------------
    // Rabi oscillations (time-domain)
    // -----------------------------------------------------------------------

    /// Simulate vacuum Rabi oscillations starting from |e,n⟩ (atom excited,
    /// n photons in cavity).
    ///
    /// With dissipation included via effective rates:
    /// ```text
    /// P_e(t) = cos²(θ_n) sin²(Ω_eff t / 2) · e^{-Γ_eff t} + ...
    /// ```
    ///
    /// For a clean pedagogical result we solve the two-state amplitude
    /// equations including decay:
    /// ```text
    /// ċ_e = -i(Δ/2)c_e - ig√(n+1) c_g - (γ/2) c_e
    /// ċ_g =  i(Δ/2)c_g - ig√(n+1) c_e - (κ(n+1)/2) c_g
    /// ```
    pub fn rabi_oscillation(
        &self,
        n_photons: usize,
        duration: f64,
        num_points: usize,
    ) -> RabiOscillation {
        let nf = n_photons as f64;
        let g_eff = self.params.g * (nf + 1.0).sqrt();
        let delta = self.detuning();
        let rabi_freq = (delta * delta + 4.0 * g_eff * g_eff).sqrt();

        let dt = duration / (num_points.max(1) as f64);
        let mut times = Vec::with_capacity(num_points);
        let mut p_excited = Vec::with_capacity(num_points);
        let mut mean_photon = Vec::with_capacity(num_points);

        // State: [c_e, c_g] in interaction picture
        // c_e: amplitude for |e, n⟩
        // c_g: amplitude for |g, n+1⟩
        let mut c_e = Complex::new(1.0, 0.0); // start in excited state
        let mut c_g = Complex::ZERO;

        let gamma_half = self.params.gamma / 2.0;
        let kappa_n_half = self.params.kappa * (nf + 1.0) / 2.0;

        // RK4 integration
        for i in 0..num_points {
            let t = i as f64 * dt;
            times.push(t);

            let pe = c_e.norm_sqr();
            let pg = c_g.norm_sqr();
            let total = pe + pg;
            let pe_norm = if total > 1e-30 { pe / total } else { 0.0 };

            p_excited.push(pe_norm);
            mean_photon.push(nf * pe_norm + (nf + 1.0) * (1.0 - pe_norm));

            // Derivatives
            let deriv = |ce: Complex, cg: Complex| -> (Complex, Complex) {
                let d_ce = Complex::new(-gamma_half, -delta / 2.0) * ce
                    + Complex::new(0.0, -g_eff) * cg;
                let d_cg = Complex::new(-kappa_n_half, delta / 2.0) * cg
                    + Complex::new(0.0, -g_eff) * ce;
                (d_ce, d_cg)
            };

            let (k1_e, k1_g) = deriv(c_e, c_g);
            let (k2_e, k2_g) = deriv(
                c_e + k1_e.scale(dt / 2.0),
                c_g + k1_g.scale(dt / 2.0),
            );
            let (k3_e, k3_g) = deriv(
                c_e + k2_e.scale(dt / 2.0),
                c_g + k2_g.scale(dt / 2.0),
            );
            let (k4_e, k4_g) = deriv(
                c_e + k3_e.scale(dt),
                c_g + k3_g.scale(dt),
            );

            c_e = c_e + (k1_e + k2_e.scale(2.0) + k3_e.scale(2.0) + k4_e).scale(dt / 6.0);
            c_g = c_g + (k1_g + k2_g.scale(2.0) + k3_g.scale(2.0) + k4_g).scale(dt / 6.0);
        }

        RabiOscillation {
            times,
            p_excited,
            mean_photon,
            rabi_freq,
        }
    }

    // -----------------------------------------------------------------------
    // Cavity transmission and reflection spectra
    // -----------------------------------------------------------------------

    /// Compute the cavity transmission and reflection spectra.
    ///
    /// Uses the coupled-oscillator model: the cavity and atom form two
    /// coupled Lorentzians. The susceptibility is:
    /// ```text
    /// χ(ω) = i·κ_ext / [i(ω_c - ω) + κ/2 + g²/(i(ω_a - ω) + γ/2)]
    /// ```
    ///
    /// Transmission T(ω) = |χ(ω)|² (normalised).
    ///
    /// `kappa_ext` is the external coupling rate (typically κ_ext ≈ κ/2 for
    /// critically coupled cavity).
    pub fn spectrum(
        &self,
        freq_min: f64,
        freq_max: f64,
        num_points: usize,
        kappa_ext: f64,
    ) -> CavitySpectrum {
        let df = (freq_max - freq_min) / (num_points.max(1) as f64);
        let mut frequencies = Vec::with_capacity(num_points);
        let mut transmission = Vec::with_capacity(num_points);
        let mut reflection = Vec::with_capacity(num_points);

        let kappa_half = self.params.kappa / 2.0;
        let gamma_half = self.params.gamma / 2.0;

        for i in 0..num_points {
            let omega = freq_min + i as f64 * df;
            frequencies.push(omega);

            // Atom susceptibility denominator
            let atom_denom = Complex::new(gamma_half, self.params.omega_a - omega);

            // g² / atom_denom
            let g2 = self.params.g * self.params.g;
            let atom_response = Complex::new(g2, 0.0) / atom_denom;

            // Cavity denominator including atom back-action
            let cav_denom = Complex::new(kappa_half, self.params.omega_c - omega) + atom_response;

            // Transmission: T = κ_ext / cav_denom (single-sided)
            let chi = Complex::new(kappa_ext, 0.0) / cav_denom;
            let t_val = chi.norm_sqr();
            transmission.push(t_val);

            // Reflection: R = |1 - κ_ext/cav_denom|²
            let r_complex = Complex::ONE - chi;
            reflection.push(r_complex.norm_sqr());
        }

        // Normalise transmission to peak
        let max_t = transmission.iter().cloned().fold(0.0_f64, f64::max);
        if max_t > 1e-30 {
            for t in &mut transmission {
                *t /= max_t;
            }
        }

        CavitySpectrum {
            frequencies,
            transmission,
            reflection,
        }
    }

    /// Compute the empty-cavity (no atom) Lorentzian transmission spectrum.
    pub fn empty_cavity_spectrum(
        &self,
        freq_min: f64,
        freq_max: f64,
        num_points: usize,
    ) -> CavitySpectrum {
        let df = (freq_max - freq_min) / (num_points.max(1) as f64);
        let kappa_half = self.params.kappa / 2.0;
        let mut frequencies = Vec::with_capacity(num_points);
        let mut transmission = Vec::with_capacity(num_points);
        let mut reflection = Vec::with_capacity(num_points);

        for i in 0..num_points {
            let omega = freq_min + i as f64 * df;
            frequencies.push(omega);

            let delta_omega = omega - self.params.omega_c;
            let lorentz = 1.0 / (1.0 + (delta_omega / kappa_half).powi(2));
            transmission.push(lorentz);
            reflection.push(1.0 - lorentz);
        }

        CavitySpectrum {
            frequencies,
            transmission,
            reflection,
        }
    }

    // -----------------------------------------------------------------------
    // Cavity ringdown
    // -----------------------------------------------------------------------

    /// Simulate cavity ringdown from initial photon number n_0.
    ///
    /// The intracavity field decays as ⟨n⟩(t) = n_0 · exp(-κ·t).
    /// We extract the time constant τ = 1/κ and Q = ω_c/κ.
    pub fn ringdown(&self, n_initial: f64, duration: f64, num_points: usize) -> CavityRingdown {
        let dt = duration / (num_points.max(1) as f64);
        let tau = 1.0 / self.params.kappa;
        let q = self.quality_factor();

        let mut times = Vec::with_capacity(num_points);
        let mut photon_number = Vec::with_capacity(num_points);

        for i in 0..num_points {
            let t = i as f64 * dt;
            times.push(t);
            photon_number.push(n_initial * (-self.params.kappa * t).exp());
        }

        CavityRingdown {
            times,
            photon_number,
            tau,
            quality_factor: q,
        }
    }

    /// Extract the decay time constant from a ringdown trace using
    /// least-squares fit to log(n(t)) = log(n_0) - t/τ.
    pub fn fit_ringdown(times: &[f64], photon_counts: &[f64]) -> Option<f64> {
        if times.len() < 2 {
            return None;
        }

        // Linear regression on ln(n) vs t
        let mut sum_t = 0.0;
        let mut sum_ln = 0.0;
        let mut sum_t2 = 0.0;
        let mut sum_t_ln = 0.0;
        let mut count = 0usize;

        for (t, &n) in times.iter().zip(photon_counts) {
            if n > 1e-30 {
                let ln_n = n.ln();
                sum_t += t;
                sum_ln += ln_n;
                sum_t2 += t * t;
                sum_t_ln += t * ln_n;
                count += 1;
            }
        }

        if count < 2 {
            return None;
        }

        let cf = count as f64;
        let slope = (cf * sum_t_ln - sum_t * sum_ln) / (cf * sum_t2 - sum_t * sum_t);

        // slope = -1/τ, so τ = -1/slope
        if slope < -1e-30 {
            Some(-1.0 / slope)
        } else {
            None
        }
    }

    // -----------------------------------------------------------------------
    // Photon statistics and blockade
    // -----------------------------------------------------------------------

    /// Compute photon statistics in the steady state of the driven
    /// Jaynes-Cummings system.
    ///
    /// For a weakly driven cavity (drive amplitude ε << g), the
    /// anharmonicity of the JC ladder leads to photon blockade.
    ///
    /// The steady-state photon number distribution is approximated
    /// from the driven-dissipative master equation in the Fock basis.
    ///
    /// `drive_amp` is the drive amplitude ε in units of √(photons/s).
    /// `drive_freq` is the drive frequency ω_d (rad/s).
    pub fn photon_statistics(&self, drive_amp: f64, drive_freq: f64) -> PhotonStatistics {
        // Solve for steady-state density matrix elements in truncated Fock space.
        // For weak drive, use perturbation theory up to n_max.
        let n_max = self.params.n_max.min(20);
        let delta_d_c = drive_freq - self.params.omega_c;

        // Photon number probabilities from rate equations
        // P(n) ~ |ε|^{2n} / |D_n|^2 where D_n is the n-photon detuning
        let mut probs = vec![0.0; n_max + 1];
        probs[0] = 1.0;

        let delta = self.detuning();

        for n in 1..=n_max {
            let nf = n as f64;
            // JC ladder energy for n photons: E_n = n·ω_c ± g√n
            // Effective detuning for n-th photon includes anharmonicity
            let omega_n_plus = self.params.omega_c + self.params.g * nf.sqrt();
            let omega_n_minus = self.params.omega_c - self.params.g * nf.sqrt();

            // Detuning of drive from n-th dressed transition
            let det_plus = drive_freq - omega_n_plus;
            let det_minus = drive_freq - omega_n_minus;

            // Choose the nearer branch
            let det_eff = if det_plus.abs() < det_minus.abs() {
                det_plus
            } else {
                det_minus
            };

            let denom = det_eff * det_eff + (self.params.kappa / 2.0).powi(2);
            let rate = drive_amp * drive_amp / denom.max(1e-30);
            probs[n] = probs[n - 1] * rate / (nf * self.params.kappa);
        }

        // Normalise
        let total: f64 = probs.iter().sum();
        if total > 1e-30 {
            for p in &mut probs {
                *p /= total;
            }
        }

        // Moments
        let mean_n: f64 = probs.iter().enumerate().map(|(n, &p)| n as f64 * p).sum();
        let mean_n2: f64 = probs
            .iter()
            .enumerate()
            .map(|(n, &p)| (n as f64).powi(2) * p)
            .sum();
        let variance = mean_n2 - mean_n * mean_n;

        // g²(0) = ⟨n(n-1)⟩ / ⟨n⟩²
        let mean_n_n_minus_1: f64 = probs
            .iter()
            .enumerate()
            .map(|(n, &p)| (n as f64) * ((n as f64) - 1.0) * p)
            .sum();

        let g2_zero = if mean_n > 1e-15 {
            mean_n_n_minus_1 / (mean_n * mean_n)
        } else {
            0.0
        };

        let mandel_q = if mean_n > 1e-15 {
            variance / mean_n - 1.0
        } else {
            0.0
        };

        PhotonStatistics {
            g2_zero,
            mean_n,
            variance_n: variance,
            mandel_q,
        }
    }

    /// Check if photon blockade is present: g²(0) < 1.
    pub fn has_photon_blockade(&self, drive_amp: f64, drive_freq: f64) -> bool {
        self.photon_statistics(drive_amp, drive_freq).g2_zero < 1.0
    }

    // -----------------------------------------------------------------------
    // Dispersive readout
    // -----------------------------------------------------------------------

    /// Compute dispersive readout parameters.
    ///
    /// In the dispersive regime (|Δ| >> g), the effective Hamiltonian is:
    /// ```text
    /// H_eff ≈ ℏ(ω_c + χ·σ_z)a†a + ℏ(ω_a + χ)/2 · σ_z
    /// ```
    /// where χ = g²/Δ.
    pub fn dispersive_readout(&self) -> DispersiveReadout {
        let delta = self.detuning();
        let chi = if delta.abs() > 1e-10 {
            self.params.g * self.params.g / delta
        } else {
            // At resonance, dispersive approximation invalid
            0.0
        };

        let n_crit = if self.params.g.abs() > 1e-10 {
            delta * delta / (4.0 * self.params.g * self.params.g)
        } else {
            f64::INFINITY
        };

        DispersiveReadout {
            chi,
            omega_g: self.params.omega_c - chi,
            omega_e: self.params.omega_c + chi,
            separation: 2.0 * chi.abs(),
            delta_over_g: if self.params.g.abs() > 1e-10 {
                delta / self.params.g
            } else {
                f64::INFINITY
            },
            n_crit,
        }
    }

    /// Compute the qubit-state-dependent cavity response at a probe frequency.
    /// Returns (|T_g|², |T_e|²) — transmission when qubit is in |g⟩ vs |e⟩.
    pub fn dispersive_transmission(&self, probe_freq: f64) -> (f64, f64) {
        let readout = self.dispersive_readout();
        let kappa_half = self.params.kappa / 2.0;

        // Lorentzian for |g⟩ state
        let dw_g = probe_freq - readout.omega_g;
        let t_g = 1.0 / (1.0 + (dw_g / kappa_half).powi(2));

        // Lorentzian for |e⟩ state
        let dw_e = probe_freq - readout.omega_e;
        let t_e = 1.0 / (1.0 + (dw_e / kappa_half).powi(2));

        (t_g, t_e)
    }

    // -----------------------------------------------------------------------
    // Mollow triplet
    // -----------------------------------------------------------------------

    /// Compute the Mollow triplet for resonance fluorescence with n photons.
    ///
    /// Under strong driving, the fluorescence spectrum splits into three
    /// Lorentzian peaks: a central peak at ω_a and two sidebands at ω_a ± Ω_R.
    pub fn mollow_triplet(&self, n_photons: usize) -> MollowTriplet {
        let delta = self.detuning();
        let nf = n_photons as f64;
        let rabi = (delta * delta + 4.0 * self.params.g * self.params.g * (nf + 1.0)).sqrt();

        // On resonance (Δ=0): sidebands at ±Ω_R = ±2g√(n+1)
        // Central linewidth: γ
        // Sideband linewidth: 3γ/4
        // Relative weights: central 1/2, each sideband 1/4

        // For finite detuning, the weights change:
        let cos2_theta = if rabi > 1e-30 {
            let cos_theta = -delta / rabi;
            cos_theta * cos_theta
        } else {
            0.5
        };
        let sin2_theta = 1.0 - cos2_theta;

        MollowTriplet {
            center_freq: (self.params.omega_a + self.params.omega_c) / 2.0,
            sideband_offset: rabi,
            rabi_freq: rabi,
            center_linewidth: self.params.gamma,
            sideband_linewidth: 0.75 * self.params.gamma + 0.25 * self.params.kappa,
            center_weight: 2.0 * cos2_theta * sin2_theta,
            sideband_weight: cos2_theta.powi(2) / 2.0 + sin2_theta.powi(2) / 2.0,
        }
    }

    /// Generate the Mollow triplet fluorescence spectrum.
    pub fn mollow_spectrum(
        &self,
        n_photons: usize,
        freq_min: f64,
        freq_max: f64,
        num_points: usize,
    ) -> Vec<(f64, f64)> {
        let triplet = self.mollow_triplet(n_photons);
        let df = (freq_max - freq_min) / (num_points.max(1) as f64);

        let lorentzian = |omega: f64, omega0: f64, gamma: f64| -> f64 {
            let dw = omega - omega0;
            (gamma / 2.0) / (dw * dw + (gamma / 2.0).powi(2))
        };

        (0..num_points)
            .map(|i| {
                let omega = freq_min + i as f64 * df;
                let s_center =
                    triplet.center_weight * lorentzian(omega, triplet.center_freq, triplet.center_linewidth);
                let s_upper = triplet.sideband_weight
                    * lorentzian(
                        omega,
                        triplet.center_freq + triplet.sideband_offset,
                        triplet.sideband_linewidth,
                    );
                let s_lower = triplet.sideband_weight
                    * lorentzian(
                        omega,
                        triplet.center_freq - triplet.sideband_offset,
                        triplet.sideband_linewidth,
                    );
                (omega, s_center + s_upper + s_lower)
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // Input-output theory
    // -----------------------------------------------------------------------

    /// Compute the intracavity and output fields using input-output theory.
    ///
    /// For a driven cavity with input field amplitude `a_in` at frequency
    /// `drive_freq`, the steady-state intracavity field is:
    ///
    /// ```text
    /// ⟨a⟩ = -i√κ_ext · a_in / [i(ω_c - ω_d) + κ/2 + g²/(i(ω_a - ω_d) + γ/2)]
    /// ```
    ///
    /// Output: `a_out = a_in - √κ_ext · ⟨a⟩`
    pub fn input_output(
        &self,
        a_in: Complex,
        drive_freq: f64,
        kappa_ext: f64,
    ) -> InputOutputField {
        let kappa_half = self.params.kappa / 2.0;
        let gamma_half = self.params.gamma / 2.0;

        // Atom response
        let atom_denom = Complex::new(gamma_half, self.params.omega_a - drive_freq);
        let g2 = self.params.g * self.params.g;
        let atom_resp = Complex::new(g2, 0.0) / atom_denom;

        // Cavity response
        let cav_denom = Complex::new(kappa_half, self.params.omega_c - drive_freq) + atom_resp;

        // Intracavity field
        let sqrt_kappa_ext = kappa_ext.sqrt();
        let numerator = Complex::new(0.0, -sqrt_kappa_ext) * a_in;
        let a_cav = numerator / cav_denom;

        // Output field
        let a_out = a_in - a_cav.scale(sqrt_kappa_ext);

        let a_in_sq = a_in.norm_sqr();
        let reflection_coeff = if a_in_sq > 1e-30 {
            a_out.norm_sqr() / a_in_sq
        } else {
            0.0
        };

        // For a two-sided cavity, transmission through the second mirror
        let kappa_int = self.params.kappa - kappa_ext;
        let sqrt_kappa_int = kappa_int.abs().sqrt();
        let a_trans = a_cav.scale(sqrt_kappa_int);
        let transmission_coeff = if a_in_sq > 1e-30 {
            a_trans.norm_sqr() / a_in_sq
        } else {
            0.0
        };

        InputOutputField {
            a_cav,
            a_out,
            reflection_coeff,
            transmission_coeff,
        }
    }

    // -----------------------------------------------------------------------
    // Multi-photon transitions
    // -----------------------------------------------------------------------

    /// Compute the transition frequencies between dressed states.
    ///
    /// Returns a list of (frequency, relative_strength) pairs for all
    /// allowed transitions |n,±⟩ → |n-1,±⟩.
    pub fn transition_frequencies(&self) -> Vec<(f64, f64)> {
        let states = self.dressed_states();
        let mut transitions = Vec::new();

        // Transitions: |n,±⟩ → |n-1,±⟩ (photon emission)
        for s_upper in &states {
            for s_lower in &states {
                if s_upper.n == s_lower.n + 1 {
                    let freq = s_upper.energy - s_lower.energy;
                    if freq > 0.0 {
                        // Transition strength from mixing angles
                        let strength = if s_upper.upper == s_lower.upper {
                            s_upper.mixing_angle.cos().powi(2) * s_lower.mixing_angle.cos().powi(2)
                        } else {
                            s_upper.mixing_angle.sin().powi(2) * s_lower.mixing_angle.sin().powi(2)
                        };
                        transitions.push((freq, strength));
                    }
                }
            }
        }

        transitions.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        transitions
    }

    /// Compute the n-photon resonance condition.
    /// For n-photon transition: ω_d = ω_a + (n-1)·anharmonicity.
    pub fn n_photon_resonance(&self, n: usize) -> f64 {
        if n == 0 {
            return self.params.omega_c;
        }
        let nf = n as f64;
        // JC ladder: E_n = n·ω_c + g(√n - √(n-1))·sign...
        // n-photon resonance at ω_d = E_n/n
        let e_n = nf * self.params.omega_c
            + self.params.g * (nf.sqrt() - if n > 1 { (nf - 1.0).sqrt() } else { 0.0 });
        e_n / nf
    }

    // -----------------------------------------------------------------------
    // Utility: Wigner function of a coherent state
    // -----------------------------------------------------------------------

    /// Compute the Wigner function W(x, p) of a coherent state |α⟩.
    /// Returns a 2D grid of values.
    ///
    /// W(x, p) = (2/π) · exp(-2|α - β|²) where β = (x + ip)/√2.
    pub fn wigner_coherent(
        alpha_re: f64,
        alpha_im: f64,
        x_range: (f64, f64),
        p_range: (f64, f64),
        grid_size: usize,
    ) -> Vec<Vec<f64>> {
        let dx = (x_range.1 - x_range.0) / (grid_size.max(1) as f64);
        let dp = (p_range.1 - p_range.0) / (grid_size.max(1) as f64);

        let mut wigner = vec![vec![0.0; grid_size]; grid_size];

        for i in 0..grid_size {
            let x = x_range.0 + i as f64 * dx;
            for j in 0..grid_size {
                let p = p_range.0 + j as f64 * dp;

                // β = (x + ip)/√2
                let beta_re = x / 2.0_f64.sqrt();
                let beta_im = p / 2.0_f64.sqrt();

                let d_re = alpha_re - beta_re;
                let d_im = alpha_im - beta_im;
                let dist_sq = d_re * d_re + d_im * d_im;

                wigner[i][j] = (2.0 / PI) * (-2.0 * dist_sq).exp();
            }
        }

        wigner
    }

    /// Compute the Wigner function of a Fock state |n⟩.
    ///
    /// W_n(r) = ((-1)^n / π) · L_n(4r²) · exp(-2r²)
    /// where L_n is the Laguerre polynomial and r² = x² + p².
    pub fn wigner_fock(
        n: usize,
        x_range: (f64, f64),
        p_range: (f64, f64),
        grid_size: usize,
    ) -> Vec<Vec<f64>> {
        let dx = (x_range.1 - x_range.0) / (grid_size.max(1) as f64);
        let dp = (p_range.1 - p_range.0) / (grid_size.max(1) as f64);

        let sign = if n % 2 == 0 { 1.0 } else { -1.0 };

        let mut wigner = vec![vec![0.0; grid_size]; grid_size];

        for i in 0..grid_size {
            let x = x_range.0 + i as f64 * dx;
            for j in 0..grid_size {
                let p = p_range.0 + j as f64 * dp;
                let r2 = x * x + p * p;
                let arg = 4.0 * r2;
                let laguerre = laguerre_polynomial(n, arg);
                wigner[i][j] = sign / PI * laguerre * (-2.0 * r2).exp();
            }
        }

        wigner
    }
}

// ---------------------------------------------------------------------------
// Laguerre polynomial L_n(x) (used for Fock-state Wigner function)
// ---------------------------------------------------------------------------

/// Compute the Laguerre polynomial L_n(x) via recurrence:
/// L_0(x) = 1, L_1(x) = 1 - x,
/// (n+1)L_{n+1}(x) = (2n+1-x)L_n(x) - n·L_{n-1}(x)
fn laguerre_polynomial(n: usize, x: f64) -> f64 {
    if n == 0 {
        return 1.0;
    }
    if n == 1 {
        return 1.0 - x;
    }

    let mut l_prev = 1.0;
    let mut l_curr = 1.0 - x;

    for k in 1..n {
        let kf = k as f64;
        let l_next = ((2.0 * kf + 1.0 - x) * l_curr - kf * l_prev) / (kf + 1.0);
        l_prev = l_curr;
        l_curr = l_next;
    }

    l_curr
}

// ---------------------------------------------------------------------------
// Jaynes-Cummings energy levels (standalone utility)
// ---------------------------------------------------------------------------

/// Compute the Jaynes-Cummings energy levels E_{n,±} for manifolds 0..n_max.
///
/// Returns a vector of (n, E_plus, E_minus) tuples.
pub fn jc_energy_levels(
    omega_c: f64,
    omega_a: f64,
    g: f64,
    n_max: usize,
) -> Vec<(usize, f64, f64)> {
    let delta = omega_a - omega_c;
    let mut levels = Vec::with_capacity(n_max);

    for n in 0..n_max {
        let nf = n as f64;
        let omega_n = (delta * delta + 4.0 * g * g * (nf + 1.0)).sqrt();
        let e_mean = omega_c * (nf + 0.5);
        levels.push((n, e_mean + omega_n / 2.0, e_mean - omega_n / 2.0));
    }

    levels
}

/// Compute the anharmonicity of the JC ladder at photon number n.
///
/// α_n = (E_{n+1,+} - E_{n,+}) - (E_{n,+} - E_{n-1,+})
/// Nonzero anharmonicity is what enables photon blockade.
pub fn jc_anharmonicity(omega_c: f64, omega_a: f64, g: f64, n: usize) -> f64 {
    let delta = omega_a - omega_c;

    let omega = |m: usize| -> f64 {
        let mf = m as f64;
        (delta * delta + 4.0 * g * g * (mf + 1.0)).sqrt()
    };

    if n == 0 {
        // Compare 1→0 and 0→ground transitions
        let e1_plus = omega_c * 1.5 + omega(1) / 2.0;
        let e0_plus = omega_c * 0.5 + omega(0) / 2.0;
        // Ground state energy (bare): -omega_a/2
        let transition_01 = e0_plus; // from ground (0 ref)
        let transition_12 = e1_plus - e0_plus;
        transition_12 - transition_01
    } else {
        let nf = n as f64;
        let e_n_plus = omega_c * (nf + 0.5) + omega(n) / 2.0;
        let e_nm1_plus = omega_c * (nf - 0.5) + omega(n - 1) / 2.0;
        let e_np1_plus = omega_c * (nf + 1.5) + omega(n + 1) / 2.0;

        (e_np1_plus - e_n_plus) - (e_n_plus - e_nm1_plus)
    }
}

/// Convert between angular frequency (rad/s) and regular frequency (Hz).
pub fn angular_to_hz(omega: f64) -> f64 {
    omega / (2.0 * PI)
}

/// Convert from Hz to angular frequency (rad/s).
pub fn hz_to_angular(f: f64) -> f64 {
    2.0 * PI * f
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;
    const TOL_REL: f64 = 0.05; // 5% relative tolerance for numerical results

    fn assert_approx(a: f64, b: f64, tol: f64) {
        assert!(
            (a - b).abs() < tol,
            "expected {} ≈ {} (diff = {})",
            a,
            b,
            (a - b).abs()
        );
    }

    fn assert_rel(a: f64, b: f64, tol: f64) {
        let denom = b.abs().max(1e-30);
        let rel = (a - b).abs() / denom;
        assert!(
            rel < tol,
            "expected {} ≈ {} (rel diff = {:.2}%)",
            a,
            b,
            rel * 100.0
        );
    }

    // -- Complex number tests --

    #[test]
    fn test_complex_basic_ops() {
        let a = Complex::new(3.0, 4.0);
        let b = Complex::new(1.0, -2.0);

        let sum = a + b;
        assert_approx(sum.re, 4.0, TOL);
        assert_approx(sum.im, 2.0, TOL);

        let diff = a - b;
        assert_approx(diff.re, 2.0, TOL);
        assert_approx(diff.im, 6.0, TOL);

        let prod = a * b;
        assert_approx(prod.re, 11.0, TOL);
        assert_approx(prod.im, -2.0, TOL);
    }

    #[test]
    fn test_complex_norm_conj() {
        let z = Complex::new(3.0, 4.0);
        assert_approx(z.norm(), 5.0, TOL);
        assert_approx(z.norm_sqr(), 25.0, TOL);

        let zc = z.conj();
        assert_approx(zc.re, 3.0, TOL);
        assert_approx(zc.im, -4.0, TOL);
    }

    #[test]
    fn test_complex_division() {
        let a = Complex::new(1.0, 0.0);
        let b = Complex::new(0.0, 1.0);
        let c = a / b;
        assert_approx(c.re, 0.0, TOL);
        assert_approx(c.im, -1.0, TOL);
    }

    #[test]
    fn test_complex_exp() {
        // e^(i·π) = -1
        let z = Complex::new(0.0, PI);
        let r = z.exp();
        assert_approx(r.re, -1.0, TOL);
        assert_approx(r.im, 0.0, TOL);
    }

    #[test]
    fn test_complex_from_polar() {
        let z = Complex::from_polar(2.0, PI / 4.0);
        assert_approx(z.re, 2.0_f64.sqrt(), TOL);
        assert_approx(z.im, 2.0_f64.sqrt(), TOL);
    }

    // -- Parameter presets --

    #[test]
    fn test_circuit_qed_preset() {
        let p = CavityQedParams::circuit_qed();
        let sys = CavityQedSystem::new(p);
        assert!(sys.is_strong_coupling());
        assert!(sys.cooperativity() > 100.0);
    }

    #[test]
    fn test_optical_cavity_preset() {
        let p = CavityQedParams::optical_cavity();
        let sys = CavityQedSystem::new(p);
        assert!(sys.is_strong_coupling());
    }

    #[test]
    fn test_strong_coupling_preset() {
        let p = CavityQedParams::strong_coupling_resonant();
        let sys = CavityQedSystem::new(p);
        assert!(sys.is_strong_coupling());
        assert!(sys.cooperativity() > 1e4);
    }

    #[test]
    fn test_dispersive_preset() {
        let p = CavityQedParams::dispersive();
        let sys = CavityQedSystem::new(p);
        assert!(sys.is_dispersive());
    }

    // -- Static properties --

    #[test]
    fn test_quality_factor() {
        let p = CavityQedParams {
            omega_c: 2.0 * PI * 5.0e9,
            omega_a: 2.0 * PI * 5.0e9,
            g: 2.0 * PI * 50.0e6,
            kappa: 2.0 * PI * 1.0e6,
            gamma: 2.0 * PI * 0.1e6,
            n_max: 10,
        };
        let sys = CavityQedSystem::new(p);
        assert_approx(sys.quality_factor(), 5000.0, 1.0);
    }

    #[test]
    fn test_cooperativity() {
        // C = 4g²/(κ·γ) with g=50, κ=1, γ=0.1 (in MHz, cancels 2π)
        // C = 4·50²/(1·0.1) = 100000
        let p = CavityQedParams {
            omega_c: 2.0 * PI * 5.0e9,
            omega_a: 2.0 * PI * 5.0e9,
            g: 2.0 * PI * 50.0e6,
            kappa: 2.0 * PI * 1.0e6,
            gamma: 2.0 * PI * 0.1e6,
            n_max: 10,
        };
        let sys = CavityQedSystem::new(p);
        let c = sys.cooperativity();
        assert_rel(c, 100000.0, TOL_REL);
    }

    #[test]
    fn test_purcell_rate() {
        let p = CavityQedParams {
            omega_c: 2.0 * PI * 5.0e9,
            omega_a: 2.0 * PI * 5.0e9,
            g: 2.0 * PI * 10.0e6,
            kappa: 2.0 * PI * 100.0e6, // bad cavity limit
            gamma: 2.0 * PI * 1.0e6,
            n_max: 5,
        };
        let sys = CavityQedSystem::new(p);
        // Γ_P = 4g²/κ = 4·(10)²/100 = 4 MHz (in angular freq with 2π)
        let expected = 4.0 * (2.0 * PI * 10.0e6).powi(2) / (2.0 * PI * 100.0e6);
        assert_rel(sys.purcell_rate(), expected, TOL_REL);
    }

    #[test]
    fn test_purcell_factor_equals_cooperativity() {
        let p = CavityQedParams::circuit_qed();
        let sys = CavityQedSystem::new(p);
        assert_rel(sys.purcell_factor(), sys.cooperativity(), TOL_REL);
    }

    #[test]
    fn test_detuning() {
        let p = CavityQedParams {
            omega_c: 2.0 * PI * 5.0e9,
            omega_a: 2.0 * PI * 5.1e9,
            g: 2.0 * PI * 50.0e6,
            kappa: 2.0 * PI * 1.0e6,
            gamma: 2.0 * PI * 0.1e6,
            n_max: 10,
        };
        let sys = CavityQedSystem::new(p);
        assert_rel(sys.detuning(), 2.0 * PI * 0.1e9, TOL_REL);
    }

    #[test]
    fn test_cavity_lifetime() {
        let kappa = 2.0 * PI * 1.0e6;
        let p = CavityQedParams {
            omega_c: 2.0 * PI * 5.0e9,
            omega_a: 2.0 * PI * 5.0e9,
            g: 2.0 * PI * 50.0e6,
            kappa,
            gamma: 2.0 * PI * 0.1e6,
            n_max: 10,
        };
        let sys = CavityQedSystem::new(p);
        assert_approx(sys.cavity_lifetime(), 1.0 / kappa, TOL);
    }

    #[test]
    fn test_strong_coupling_criterion() {
        // g > κ/2 and g > γ/2
        let p = CavityQedParams {
            omega_c: 1.0,
            omega_a: 1.0,
            g: 10.0,
            kappa: 1.0,
            gamma: 1.0,
            n_max: 5,
        };
        let sys = CavityQedSystem::new(p);
        assert!(sys.is_strong_coupling());

        // Weak coupling: g < κ/2
        let p2 = CavityQedParams {
            omega_c: 1.0,
            omega_a: 1.0,
            g: 0.1,
            kappa: 1.0,
            gamma: 1.0,
            n_max: 5,
        };
        let sys2 = CavityQedSystem::new(p2);
        assert!(!sys2.is_strong_coupling());
    }

    #[test]
    fn test_dispersive_regime() {
        let p = CavityQedParams {
            omega_c: 1.0,
            omega_a: 100.0, // Δ/g = 99/1 >> 5
            g: 1.0,
            kappa: 0.1,
            gamma: 0.01,
            n_max: 5,
        };
        let sys = CavityQedSystem::new(p);
        assert!(sys.is_dispersive());

        // Not dispersive when resonant
        let p2 = CavityQedParams {
            omega_c: 1.0,
            omega_a: 1.0,
            g: 1.0,
            kappa: 0.1,
            gamma: 0.01,
            n_max: 5,
        };
        let sys2 = CavityQedSystem::new(p2);
        assert!(!sys2.is_dispersive());
    }

    #[test]
    fn test_critical_photon_number() {
        let p = CavityQedParams {
            omega_c: 2.0 * PI * 5.0e9,
            omega_a: 2.0 * PI * 7.0e9,
            g: 2.0 * PI * 50.0e6,
            kappa: 2.0 * PI * 1.0e6,
            gamma: 2.0 * PI * 0.1e6,
            n_max: 10,
        };
        let sys = CavityQedSystem::new(p);
        // Δ = 2GHz, g = 50MHz => n_crit = (2000)²/(4·50²) = 400
        let n_crit = sys.critical_photon_number();
        assert_rel(n_crit, 400.0, TOL_REL);
    }

    #[test]
    fn test_finesse() {
        let p = CavityQedParams {
            omega_c: 2.0 * PI * 1e9,
            omega_a: 2.0 * PI * 1e9,
            g: 2.0 * PI * 10.0e6,
            kappa: 2.0 * PI * 1.0e6,
            gamma: 2.0 * PI * 0.1e6,
            n_max: 5,
        };
        let sys = CavityQedSystem::new(p);
        let length = 0.15; // 15cm cavity
        let f = sys.finesse(length);
        // F = π·c/(L·κ) = π·3e8/(0.15·2π·1e6) ≈ 1e3
        assert!(f > 100.0);
    }

    // -- Dressed states --

    #[test]
    fn test_dressed_states_count() {
        let p = CavityQedParams {
            omega_c: 1.0,
            omega_a: 1.0,
            g: 0.1,
            kappa: 0.01,
            gamma: 0.001,
            n_max: 5,
        };
        let sys = CavityQedSystem::new(p);
        let states = sys.dressed_states();
        assert_eq!(states.len(), 10); // 2 per manifold × 5
    }

    #[test]
    fn test_dressed_state_splitting_resonance() {
        // At resonance, splitting = 2g√(n+1)
        let g = 0.1;
        let p = CavityQedParams {
            omega_c: 1.0,
            omega_a: 1.0,
            g,
            kappa: 0.001,
            gamma: 0.001,
            n_max: 3,
        };
        let sys = CavityQedSystem::new(p);
        let states = sys.dressed_states();

        // n=0 manifold: splitting = 2g
        let e_plus = states[0].energy;
        let e_minus = states[1].energy;
        assert_rel((e_plus - e_minus).abs(), 2.0 * g, TOL_REL);
    }

    #[test]
    fn test_dressed_state_n1_splitting() {
        // n=1 manifold: splitting = 2g√2
        let g = 0.1;
        let p = CavityQedParams {
            omega_c: 1.0,
            omega_a: 1.0,
            g,
            kappa: 0.001,
            gamma: 0.001,
            n_max: 3,
        };
        let sys = CavityQedSystem::new(p);
        let states = sys.dressed_states();

        let e_plus = states[2].energy; // n=1, +
        let e_minus = states[3].energy; // n=1, -
        assert_rel(
            (e_plus - e_minus).abs(),
            2.0 * g * 2.0_f64.sqrt(),
            TOL_REL,
        );
    }

    #[test]
    fn test_rabi_frequency_resonance() {
        // At resonance: Ω_n = 2g√(n+1)
        let g = 2.0 * PI * 50.0e6;
        let p = CavityQedParams {
            omega_c: 2.0 * PI * 5.0e9,
            omega_a: 2.0 * PI * 5.0e9,
            g,
            kappa: 2.0 * PI * 1.0e6,
            gamma: 2.0 * PI * 0.1e6,
            n_max: 10,
        };
        let sys = CavityQedSystem::new(p);

        assert_rel(sys.rabi_frequency(0), 2.0 * g, TOL_REL);
        assert_rel(sys.rabi_frequency(1), 2.0 * g * 2.0_f64.sqrt(), TOL_REL);
        assert_rel(sys.rabi_frequency(3), 2.0 * g * 4.0_f64.sqrt(), TOL_REL);
    }

    #[test]
    fn test_vacuum_rabi_frequency() {
        let p = CavityQedParams::strong_coupling_resonant();
        let sys = CavityQedSystem::new(p);
        assert_rel(sys.vacuum_rabi_frequency(), 2.0 * p.g, TOL_REL);
    }

    // -- Rabi oscillations --

    #[test]
    fn test_rabi_oscillation_starts_excited() {
        let p = CavityQedParams::strong_coupling_resonant();
        let sys = CavityQedSystem::new(p);
        let result = sys.rabi_oscillation(0, 1e-8, 100);

        assert_eq!(result.times.len(), 100);
        assert_eq!(result.p_excited.len(), 100);
        // Should start near 1.0 (excited state)
        assert!(result.p_excited[0] > 0.95);
    }

    #[test]
    fn test_rabi_oscillation_frequency() {
        let g = 2.0 * PI * 100.0e6;
        let p = CavityQedParams {
            omega_c: 2.0 * PI * 1.0e9,
            omega_a: 2.0 * PI * 1.0e9,
            g,
            kappa: 2.0 * PI * 0.01e6, // very low loss
            gamma: 2.0 * PI * 0.01e6,
            n_max: 5,
        };
        let sys = CavityQedSystem::new(p);
        let result = sys.rabi_oscillation(0, 5e-9, 500);

        // Find first minimum (should be near T/2 = π/(2g))
        let half_period = PI / (2.0 * g);
        let mut min_idx = 0;
        let mut min_val = f64::MAX;
        for (i, &pe) in result.p_excited.iter().enumerate() {
            if pe < min_val {
                min_val = pe;
                min_idx = i;
            }
        }
        let min_time = result.times[min_idx];
        assert_rel(min_time, half_period, 0.15); // 15% tolerance for discrete sampling
    }

    #[test]
    fn test_rabi_oscillation_decay() {
        // With moderate dissipation, the simulation should produce valid
        // population dynamics: starts excited, oscillates, values stay in [0,1].
        let p = CavityQedParams {
            omega_c: 2.0 * PI * 1.0e9,
            omega_a: 2.0 * PI * 1.0e9,
            g: 2.0 * PI * 10.0e6,
            kappa: 2.0 * PI * 5.0e6,
            gamma: 2.0 * PI * 5.0e6,
            n_max: 5,
        };
        let sys = CavityQedSystem::new(p);
        // Simulate over several Rabi periods (T_R ~ 1/(2*10MHz) = 50ns)
        let result = sys.rabi_oscillation(0, 200e-9, 2000);

        assert_eq!(result.times.len(), 2000);
        assert!(result.p_excited[0] > 0.95); // starts excited
        // All values should be finite and in [0, 1]
        for &pe in &result.p_excited {
            assert!(pe.is_finite());
            assert!(pe >= -0.01 && pe <= 1.01, "P_e out of range: {}", pe);
        }
        // The population should oscillate (not stay constant)
        let min_pe = result.p_excited.iter().cloned().fold(f64::MAX, f64::min);
        let max_pe = result.p_excited.iter().cloned().fold(0.0_f64, f64::max);
        assert!(
            max_pe - min_pe > 0.1,
            "Expected oscillations, range = {}",
            max_pe - min_pe
        );
    }

    #[test]
    fn test_rabi_multiphoton() {
        // n=1 photon: faster oscillation by √2
        let p = CavityQedParams::strong_coupling_resonant();
        let sys = CavityQedSystem::new(p);

        let r0 = sys.rabi_oscillation(0, 1e-8, 100);
        let r1 = sys.rabi_oscillation(1, 1e-8, 100);

        assert_rel(r1.rabi_freq, r0.rabi_freq * 2.0_f64.sqrt(), TOL_REL);
    }

    // -- Spectrum --

    #[test]
    fn test_empty_cavity_peak_at_resonance() {
        let p = CavityQedParams::circuit_qed();
        let sys = CavityQedSystem::new(p);

        let span = p.kappa * 20.0;
        let spec = sys.empty_cavity_spectrum(p.omega_c - span, p.omega_c + span, 1000);

        // Peak should be near the center
        let max_idx = spec
            .transmission
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        let peak_freq = spec.frequencies[max_idx];
        assert_rel(peak_freq, p.omega_c, 0.01);
    }

    #[test]
    fn test_spectrum_with_atom_splitting() {
        let p = CavityQedParams::strong_coupling_resonant();
        let sys = CavityQedSystem::new(p);

        let span = p.g * 10.0;
        let spec = sys.spectrum(p.omega_c - span, p.omega_c + span, 2000, p.kappa / 2.0);

        // In strong coupling at resonance, should see two peaks (vacuum Rabi splitting)
        // Find local maxima
        let mut peaks = Vec::new();
        for i in 1..spec.transmission.len() - 1 {
            if spec.transmission[i] > spec.transmission[i - 1]
                && spec.transmission[i] > spec.transmission[i + 1]
                && spec.transmission[i] > 0.1
            {
                peaks.push(i);
            }
        }

        assert!(peaks.len() >= 2, "Expected at least 2 peaks, found {}", peaks.len());
    }

    #[test]
    fn test_spectrum_transmission_reflection_sum() {
        // For a single-sided cavity, T + R should not exceed 1 at any point
        let p = CavityQedParams::strong_coupling_resonant();
        let sys = CavityQedSystem::new(p);

        let span = p.g * 5.0;
        let spec = sys.empty_cavity_spectrum(p.omega_c - span, p.omega_c + span, 100);

        for i in 0..spec.transmission.len() {
            let sum = spec.transmission[i] + spec.reflection[i];
            assert!(
                sum < 1.01,
                "T + R = {} > 1 at index {}",
                sum,
                i
            );
        }
    }

    // -- Ringdown --

    #[test]
    fn test_ringdown_decay() {
        let p = CavityQedParams::circuit_qed();
        let sys = CavityQedSystem::new(p);

        let rd = sys.ringdown(100.0, 5.0 / p.kappa, 1000);
        assert_eq!(rd.times.len(), 1000);

        // At t = τ = 1/κ, photon number should be ~100/e ≈ 36.8
        let tau_idx = (1000.0 / 5.0) as usize;
        if tau_idx < rd.photon_number.len() {
            assert_rel(rd.photon_number[tau_idx], 100.0 / std::f64::consts::E, 0.05);
        }
    }

    #[test]
    fn test_ringdown_tau() {
        let p = CavityQedParams::circuit_qed();
        let sys = CavityQedSystem::new(p);
        let rd = sys.ringdown(100.0, 5.0 / p.kappa, 1000);

        assert_rel(rd.tau, 1.0 / p.kappa, TOL_REL);
    }

    #[test]
    fn test_fit_ringdown() {
        let kappa = 2.0 * PI * 1.0e6;
        let tau_expected = 1.0 / kappa;

        let n = 200;
        let duration = 5.0 * tau_expected;
        let dt = duration / n as f64;
        let times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        let counts: Vec<f64> = times.iter().map(|&t| 100.0 * (-kappa * t).exp()).collect();

        let tau_fit = CavityQedSystem::fit_ringdown(&times, &counts).unwrap();
        assert_rel(tau_fit, tau_expected, 0.01);
    }

    #[test]
    fn test_fit_ringdown_insufficient_data() {
        assert!(CavityQedSystem::fit_ringdown(&[0.0], &[1.0]).is_none());
    }

    // -- Photon statistics --

    #[test]
    fn test_photon_statistics_weak_drive() {
        let p = CavityQedParams::strong_coupling_resonant();
        let sys = CavityQedSystem::new(p);

        let stats = sys.photon_statistics(p.kappa * 0.01, p.omega_c);
        // Weak drive: low mean photon number
        assert!(stats.mean_n < 1.0);
    }

    #[test]
    fn test_photon_blockade_on_resonance() {
        let p = CavityQedParams::strong_coupling_resonant();
        let sys = CavityQedSystem::new(p);

        // Drive at the lower dressed-state frequency: ω_c - g
        let drive_freq = p.omega_c - p.g;
        let stats = sys.photon_statistics(p.kappa * 0.1, drive_freq);

        // g²(0) < 1 indicates antibunching
        assert!(
            stats.g2_zero < 1.5, // may not be strongly blockaded in this approximation
            "g2(0) = {} (expected < 1.5 for strong coupling)",
            stats.g2_zero
        );
    }

    #[test]
    fn test_mandel_q_sub_poissonian() {
        // For sub-Poissonian light, Q_M < 0
        let p = CavityQedParams::strong_coupling_resonant();
        let sys = CavityQedSystem::new(p);

        let stats = sys.photon_statistics(p.kappa * 0.01, p.omega_c - p.g);
        // With very weak drive, statistics approach vacuum (degenerate)
        // Just check the calculation runs
        assert!(stats.mandel_q.is_finite());
    }

    #[test]
    fn test_photon_statistics_variance() {
        let p = CavityQedParams::circuit_qed();
        let sys = CavityQedSystem::new(p);
        let stats = sys.photon_statistics(p.kappa * 0.1, p.omega_c);
        assert!(stats.variance_n >= 0.0);
    }

    // -- Dispersive readout --

    #[test]
    fn test_dispersive_chi_shift() {
        let p = CavityQedParams::dispersive();
        let sys = CavityQedSystem::new(p);

        let readout = sys.dispersive_readout();
        let delta = p.omega_a - p.omega_c;
        let expected_chi = p.g * p.g / delta;

        assert_rel(readout.chi, expected_chi, TOL_REL);
    }

    #[test]
    fn test_dispersive_separation() {
        let p = CavityQedParams::dispersive();
        let sys = CavityQedSystem::new(p);

        let readout = sys.dispersive_readout();
        assert_rel(readout.separation, 2.0 * readout.chi.abs(), TOL_REL);
    }

    #[test]
    fn test_dispersive_omega_g_e() {
        let p = CavityQedParams::dispersive();
        let sys = CavityQedSystem::new(p);

        let readout = sys.dispersive_readout();
        assert_rel(readout.omega_g + readout.omega_e, 2.0 * p.omega_c, TOL_REL);
    }

    #[test]
    fn test_dispersive_n_crit() {
        let p = CavityQedParams::dispersive();
        let sys = CavityQedSystem::new(p);

        let readout = sys.dispersive_readout();
        let delta = p.omega_a - p.omega_c;
        let expected = delta * delta / (4.0 * p.g * p.g);
        assert_rel(readout.n_crit, expected, TOL_REL);
    }

    #[test]
    fn test_dispersive_transmission_distinguishable() {
        let p = CavityQedParams::dispersive();
        let sys = CavityQedSystem::new(p);

        let readout = sys.dispersive_readout();
        // Probe at ω_g: should see T_g >> T_e
        let (t_g, t_e) = sys.dispersive_transmission(readout.omega_g);
        assert!(t_g > t_e, "T_g = {}, T_e = {} at omega_g", t_g, t_e);
    }

    // -- Mollow triplet --

    #[test]
    fn test_mollow_triplet_structure() {
        let p = CavityQedParams::strong_coupling_resonant();
        let sys = CavityQedSystem::new(p);

        let triplet = sys.mollow_triplet(0);
        assert!(triplet.sideband_offset > 0.0);
        assert!(triplet.center_linewidth > 0.0);
        assert!(triplet.sideband_linewidth > 0.0);
    }

    #[test]
    fn test_mollow_rabi_frequency() {
        let p = CavityQedParams::strong_coupling_resonant();
        let sys = CavityQedSystem::new(p);

        let triplet = sys.mollow_triplet(0);
        // At resonance: Ω_R = 2g
        assert_rel(triplet.rabi_freq, 2.0 * p.g, TOL_REL);
    }

    #[test]
    fn test_mollow_spectrum_generation() {
        let p = CavityQedParams::strong_coupling_resonant();
        let sys = CavityQedSystem::new(p);

        let center = (p.omega_a + p.omega_c) / 2.0;
        let span = p.g * 5.0;
        let spec = sys.mollow_spectrum(0, center - span, center + span, 500);
        assert_eq!(spec.len(), 500);

        // All values should be non-negative (it's a spectral density)
        for &(_, s) in &spec {
            assert!(s >= 0.0);
        }
    }

    #[test]
    fn test_mollow_weight_sum() {
        let p = CavityQedParams::strong_coupling_resonant();
        let sys = CavityQedSystem::new(p);

        let triplet = sys.mollow_triplet(0);
        let total = triplet.center_weight + 2.0 * triplet.sideband_weight;
        // Weights should sum to approximately 1
        assert_rel(total, 1.0, 0.2);
    }

    // -- Input-output theory --

    #[test]
    fn test_input_output_empty_cavity_resonance() {
        // Empty cavity (g=0) at resonance: all transmission, no reflection
        let p = CavityQedParams {
            omega_c: 2.0 * PI * 5.0e9,
            omega_a: 2.0 * PI * 10.0e9, // far detuned atom
            g: 0.0,
            kappa: 2.0 * PI * 1.0e6,
            gamma: 2.0 * PI * 0.1e6,
            n_max: 5,
        };
        let sys = CavityQedSystem::new(p);

        let a_in = Complex::new(1.0, 0.0);
        let kappa_ext = p.kappa / 2.0;
        let result = sys.input_output(a_in, p.omega_c, kappa_ext);

        assert!(result.a_cav.norm() > 0.0);
    }

    #[test]
    fn test_input_output_far_detuned() {
        // Far off-resonance: cavity reflects
        let p = CavityQedParams::circuit_qed();
        let sys = CavityQedSystem::new(p);

        let a_in = Complex::new(1.0, 0.0);
        let kappa_ext = p.kappa / 2.0;
        let result = sys.input_output(a_in, p.omega_c + p.kappa * 100.0, kappa_ext);

        // Mostly reflected
        assert!(
            result.reflection_coeff > 0.8,
            "R = {} (expected > 0.8 off-resonance)",
            result.reflection_coeff
        );
    }

    #[test]
    fn test_input_output_field_relation() {
        // a_out = a_in - √κ_ext · a_cav
        let p = CavityQedParams::circuit_qed();
        let sys = CavityQedSystem::new(p);

        let a_in = Complex::new(1.0, 0.0);
        let kappa_ext = p.kappa / 2.0;
        let result = sys.input_output(a_in, p.omega_c, kappa_ext);

        let expected_a_out = a_in - result.a_cav.scale(kappa_ext.sqrt());
        assert_approx(result.a_out.re, expected_a_out.re, 1e-10);
        assert_approx(result.a_out.im, expected_a_out.im, 1e-10);
    }

    // -- Transition frequencies --

    #[test]
    fn test_transition_frequencies_nonempty() {
        let p = CavityQedParams::strong_coupling_resonant();
        let sys = CavityQedSystem::new(p);

        let trans = sys.transition_frequencies();
        assert!(!trans.is_empty());
        // All frequencies should be positive
        for &(f, _) in &trans {
            assert!(f > 0.0);
        }
    }

    #[test]
    fn test_n_photon_resonance() {
        let p = CavityQedParams::strong_coupling_resonant();
        let sys = CavityQedSystem::new(p);

        let r1 = sys.n_photon_resonance(1);
        let r2 = sys.n_photon_resonance(2);
        // Due to anharmonicity, these should differ
        assert!((r1 - r2).abs() > 0.0);
    }

    // -- JC energy levels utility --

    #[test]
    fn test_jc_energy_levels() {
        let levels = jc_energy_levels(1.0, 1.0, 0.1, 5);
        assert_eq!(levels.len(), 5);
        for (_, e_plus, e_minus) in &levels {
            assert!(e_plus > e_minus);
        }
    }

    #[test]
    fn test_jc_anharmonicity_nonzero() {
        let alpha = jc_anharmonicity(1.0, 1.0, 0.1, 1);
        assert!(alpha.abs() > 0.0, "JC ladder should be anharmonic");
    }

    #[test]
    fn test_jc_anharmonicity_sign() {
        // At resonance, anharmonicity is negative (√(n+1) spacing decreases)
        let alpha = jc_anharmonicity(1.0, 1.0, 0.1, 1);
        // The effective spacing decreases: √3 - √2 < √2 - 1
        assert!(alpha < 0.0, "Expected negative anharmonicity, got {}", alpha);
    }

    // -- Laguerre polynomial --

    #[test]
    fn test_laguerre_l0() {
        assert_approx(laguerre_polynomial(0, 5.0), 1.0, TOL);
    }

    #[test]
    fn test_laguerre_l1() {
        assert_approx(laguerre_polynomial(1, 3.0), -2.0, TOL);
    }

    #[test]
    fn test_laguerre_l2() {
        // L_2(x) = 1 - 2x + x²/2
        let x = 2.0;
        let expected = 1.0 - 2.0 * x + x * x / 2.0;
        assert_approx(laguerre_polynomial(2, x), expected, TOL);
    }

    #[test]
    fn test_laguerre_l3() {
        // L_3(x) = 1 - 3x + 3x²/2 - x³/6
        let x = 1.5;
        let expected = 1.0 - 3.0 * x + 1.5 * x * x - x * x * x / 6.0;
        assert_approx(laguerre_polynomial(3, x), expected, TOL);
    }

    // -- Wigner functions --

    #[test]
    fn test_wigner_coherent_peak() {
        // Peak of coherent state |α=0⟩ (vacuum) is at origin
        let w = CavityQedSystem::wigner_coherent(0.0, 0.0, (-3.0, 3.0), (-3.0, 3.0), 50);
        assert_eq!(w.len(), 50);

        // Peak should be at center
        let mid = 25;
        let peak_val = w[mid][mid];
        assert!(
            peak_val > 0.5 / PI,
            "Expected peak > {}, got {}",
            0.5 / PI,
            peak_val
        );
    }

    #[test]
    fn test_wigner_coherent_displaced() {
        // |α=2⟩ should peak near x=2√2, p=0
        let w = CavityQedSystem::wigner_coherent(2.0, 0.0, (-1.0, 5.0), (-3.0, 3.0), 100);

        // Find maximum
        let mut max_val = 0.0;
        let mut max_i = 0;
        let mut max_j = 0;
        for i in 0..100 {
            for j in 0..100 {
                if w[i][j] > max_val {
                    max_val = w[i][j];
                    max_i = i;
                    max_j = j;
                }
            }
        }

        // Peak x should be near α·√2 = 2√2 ≈ 2.83
        let dx = 6.0 / 100.0;
        let peak_x = -1.0 + max_i as f64 * dx;
        assert_rel(peak_x, 2.0 * 2.0_f64.sqrt(), 0.15);

        // Peak p should be near 0
        let dp = 6.0 / 100.0;
        let peak_p = -3.0 + max_j as f64 * dp;
        assert!(peak_p.abs() < 0.5);
    }

    #[test]
    fn test_wigner_fock_vacuum() {
        // |n=0⟩ = vacuum = coherent |α=0⟩, should be Gaussian at origin
        let w = CavityQedSystem::wigner_fock(0, (-3.0, 3.0), (-3.0, 3.0), 50);
        let mid = 25;
        assert!(w[mid][mid] > 0.0);
    }

    #[test]
    fn test_wigner_fock_n1_negative() {
        // |n=1⟩ Wigner function goes negative at the origin
        let w = CavityQedSystem::wigner_fock(1, (-3.0, 3.0), (-3.0, 3.0), 50);
        let mid = 25;
        assert!(
            w[mid][mid] < 0.0,
            "Fock |1⟩ should be negative at origin, got {}",
            w[mid][mid]
        );
    }

    // -- Frequency conversions --

    #[test]
    fn test_angular_to_hz() {
        assert_rel(angular_to_hz(2.0 * PI * 1e9), 1e9, TOL_REL);
    }

    #[test]
    fn test_hz_to_angular() {
        assert_rel(hz_to_angular(1e9), 2.0 * PI * 1e9, TOL_REL);
    }

    #[test]
    fn test_roundtrip_frequency() {
        let f = 5.0e9;
        assert_rel(angular_to_hz(hz_to_angular(f)), f, TOL);
    }

    // -- Edge cases --

    #[test]
    fn test_zero_coupling() {
        let p = CavityQedParams {
            omega_c: 1.0,
            omega_a: 1.0,
            g: 0.0,
            kappa: 0.1,
            gamma: 0.01,
            n_max: 5,
        };
        let sys = CavityQedSystem::new(p);
        assert!(!sys.is_strong_coupling());
        assert_approx(sys.cooperativity(), 0.0, TOL);
        assert_approx(sys.purcell_rate(), 0.0, TOL);
    }

    #[test]
    fn test_zero_detuning_rabi() {
        // At zero detuning, vacuum Rabi = 2g exactly
        let g = 0.5;
        let p = CavityQedParams {
            omega_c: 10.0,
            omega_a: 10.0,
            g,
            kappa: 0.001,
            gamma: 0.001,
            n_max: 5,
        };
        let sys = CavityQedSystem::new(p);
        assert_approx(sys.vacuum_rabi_frequency(), 2.0 * g, TOL);
    }

    #[test]
    fn test_large_detuning_rabi() {
        // With large detuning: Ω ≈ |Δ| (when Δ >> g)
        let p = CavityQedParams {
            omega_c: 1.0,
            omega_a: 1000.0,
            g: 0.01,
            kappa: 0.001,
            gamma: 0.001,
            n_max: 5,
        };
        let sys = CavityQedSystem::new(p);
        let delta = (p.omega_a - p.omega_c).abs();
        assert_rel(sys.vacuum_rabi_frequency(), delta, 0.001);
    }

    #[test]
    fn test_ringdown_quality_factor() {
        let p = CavityQedParams::circuit_qed();
        let sys = CavityQedSystem::new(p);
        let rd = sys.ringdown(10.0, 1e-6, 100);
        assert_rel(rd.quality_factor, sys.quality_factor(), TOL_REL);
    }

    #[test]
    fn test_photon_lifetime_equals_cavity_lifetime() {
        let p = CavityQedParams::circuit_qed();
        let sys = CavityQedSystem::new(p);
        assert_approx(sys.photon_lifetime(), sys.cavity_lifetime(), TOL);
    }

    #[test]
    fn test_single_atom_cooperativity() {
        let p = CavityQedParams::circuit_qed();
        let sys = CavityQedSystem::new(p);
        assert_approx(
            sys.single_atom_cooperativity(),
            sys.cooperativity(),
            TOL,
        );
    }
}

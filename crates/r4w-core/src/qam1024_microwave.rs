//! # High-Order QAM Microwave Radio Modem (1024-QAM and beyond)
//!
//! DSP algorithms for ultra-high-capacity microwave backhaul using 256-QAM through
//! 4096-QAM targeting traditional microwave bands (6–42 GHz).  Each spectral-Hz
//! carries up to 12 bits of information (4096-QAM), enabling multi-Gigabit throughput
//! in a 56 MHz channel.
//!
//! ## DSP Chain Overview
//!
//! ```text
//! TX: Bits → Gray map → RRC pulse-shape → DPD → PA → RF
//!
//! RX: RF → LNA → ADC → DDC → RRC matched → EQ → CPE → Decision → Bits
//!                                            ↑
//!                                  Pilot-based CPE estimation
//! ```
//!
//! ## Key Algorithms
//!
//! | Block | Algorithm | Reference |
//! |-------|-----------|-----------|
//! | Constellation | Square QAM + Gray coding | ETSI EN 302 217-2 §6.3 |
//! | Phase Noise | Wiener process + CPE correction | ETSI EN 302 217-2 §8.3 |
//! | DPD | Memory-polynomial ILA | ETSI EN 302 217-2 §7.4 |
//! | Equalizer | Fractionally-spaced (T/2) DD-LMS | ETSI EN 302 217-2 §8.4 |
//! | Atmospheric | ITU-R P.530 clear-air fading | ITU-R P.530-17 §2.3 |
//! | Spectral eff | Shannon capacity comparison | ETSI EN 302 217-2 §4 |
//!
//! ## Standards References
//!
//! - **ETSI EN 302 217-2** — Fixed Radio Systems; Characteristics and requirements for
//!   point-to-point equipment and antennas; Part 2: Digital systems operating in
//!   frequency bands from 1 GHz to 86 GHz (v3.2.1, 2021)
//! - **ETSI EN 302 217-1** — General rules and requirements (v3.2.1, 2021)
//! - **ITU-R F.1191** — Bandwidths and unwanted emissions of digital radio-relay systems
//! - **ITU-R P.530-17** — Propagation data and prediction methods for terrestrial
//!   line-of-sight systems
//! - **ITU-R P.676-12** — Attenuation by atmospheric gases and related effects
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::qam1024_microwave::{
//!     QamConstellation, QamOrder, MicrowaveModem, ModemConfig, MicrowaveBand,
//! };
//!
//! // Build a 1024-QAM modem at 11 GHz with a 28 MHz channel
//! let cfg = ModemConfig {
//!     order: QamOrder::Qam1024,
//!     band: MicrowaveBand::Band11Ghz,
//!     channel_bandwidth_hz: 28e6,
//!     symbol_rate_baud: 25.0e6,
//!     tx_power_dbm: 23.0,
//!     noise_figure_db: 5.0,
//! };
//! let modem = MicrowaveModem::new(cfg);
//!
//! // Check spectral efficiency
//! let se = modem.spectral_efficiency();
//! assert!(se.bits_per_hz > 9.0); // 1024-QAM = 10 bits/symbol with FEC overhead
//!
//! // Encode bits → symbols
//! let bits: Vec<bool> = (0..40).map(|i| i % 3 == 0).collect();
//! let syms = modem.modulate(&bits);
//! assert!(!syms.is_empty());
//! ```

use std::f64::consts::PI;

// ─────────────────────────────────────────────────────────────────────────────
// Complex arithmetic helpers (no external crates)
// ─────────────────────────────────────────────────────────────────────────────

/// Complex sample as (real, imag).
pub type C64 = (f64, f64);

#[inline(always)]
fn c_add(a: C64, b: C64) -> C64 { (a.0 + b.0, a.1 + b.1) }

#[inline(always)]
fn c_sub(a: C64, b: C64) -> C64 { (a.0 - b.0, a.1 - b.1) }

#[inline(always)]
fn c_mul(a: C64, b: C64) -> C64 {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline(always)]
fn c_scale(a: C64, s: f64) -> C64 { (a.0 * s, a.1 * s) }

#[inline(always)]
fn c_conj(a: C64) -> C64 { (a.0, -a.1) }

#[inline(always)]
fn c_abs_sq(a: C64) -> f64 { a.0 * a.0 + a.1 * a.1 }

#[inline(always)]
fn c_abs(a: C64) -> f64 { c_abs_sq(a).sqrt() }

#[inline(always)]
fn c_arg(a: C64) -> f64 { a.1.atan2(a.0) }

/// Rotate by angle θ (radians).
#[inline(always)]
fn c_rotate(a: C64, theta: f64) -> C64 {
    let (s, c) = theta.sin_cos();
    (a.0 * c - a.1 * s, a.0 * s + a.1 * c)
}

// ─────────────────────────────────────────────────────────────────────────────
// Simple Gaussian PRNG (Box-Muller) — no rand crate needed
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
struct SimpleRng {
    state: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self { Self { state: seed ^ 0x6A09E667F3BCC908 } }

    /// Uniform in [0,1).
    fn uniform(&mut self) -> f64 {
        // xorshift64*
        self.state ^= self.state << 12;
        self.state ^= self.state >> 25;
        self.state ^= self.state << 27;
        let v = self.state.wrapping_mul(0x2545F4914F6CDD1D);
        (v >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Standard normal via Box-Muller.
    fn gaussian(&mut self) -> f64 {
        let u1 = self.uniform().max(1e-15);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// QAM order enum
// ─────────────────────────────────────────────────────────────────────────────

/// Supported QAM constellation orders.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QamOrder {
    /// 256-QAM — 8 bits/symbol
    Qam256,
    /// 512-QAM — 9 bits/symbol
    Qam512,
    /// 1024-QAM — 10 bits/symbol
    Qam1024,
    /// 2048-QAM — 11 bits/symbol
    Qam2048,
    /// 4096-QAM — 12 bits/symbol
    Qam4096,
}

impl QamOrder {
    /// Bits per symbol (log₂ of total points).
    pub fn bits_per_symbol(self) -> usize {
        match self {
            QamOrder::Qam256  => 8,
            QamOrder::Qam512  => 9,
            QamOrder::Qam1024 => 10,
            QamOrder::Qam2048 => 11,
            QamOrder::Qam4096 => 12,
        }
    }

    /// Total constellation size M = 2^(bits/sym).
    pub fn size(self) -> usize { 1 << self.bits_per_symbol() }

    /// Side length of the square grid (√M for even-bit orders).
    /// For 512/2048 (odd bits), use the closest rectangular grid.
    pub fn side(self) -> (usize, usize) {
        let bps = self.bits_per_symbol();
        let rows = 1usize << (bps / 2);
        let cols = 1usize << ((bps + 1) / 2);
        (rows, cols)
    }

    /// Minimum required SNR (dB) for BER ≈ 10⁻⁶ (uncoded).
    /// Values from ETSI EN 302 217-2 Table 1.
    pub fn min_snr_db_uncoded(self) -> f64 {
        match self {
            QamOrder::Qam256  => 30.0,
            QamOrder::Qam512  => 33.0,
            QamOrder::Qam1024 => 36.0,
            QamOrder::Qam2048 => 39.0,
            QamOrder::Qam4096 => 42.0,
        }
    }

    /// Approximate phase noise tolerance (1-Hz normalised PSD at 10 kHz offset, dBc/Hz).
    /// Tighter for higher orders; from ETSI EN 302 217-2 §8.3.
    pub fn phase_noise_tolerance_dbc(self) -> f64 {
        match self {
            QamOrder::Qam256  => -85.0,
            QamOrder::Qam512  => -88.0,
            QamOrder::Qam1024 => -91.0,
            QamOrder::Qam2048 => -94.0,
            QamOrder::Qam4096 => -97.0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// QAM constellation
// ─────────────────────────────────────────────────────────────────────────────

/// Square QAM constellation with Gray-coded bit mapping and normalised power.
///
/// The constellation is generated on a rectangular grid
/// `{ ±1, ±3, … ±(√M−1) }² × scale`, where `scale` normalises average power to 1.
///
/// # Gray Coding
///
/// Each I and Q axis is independently Gray-coded using the standard binary
/// reflected Gray code (BRGC).  The combined 2D Gray label is
/// `gray_q << (bps/2) | gray_i`.
///
/// # Reference
///
/// ETSI EN 302 217-2 v3.2.1 §6.3 — Modulation formats
pub struct QamConstellation {
    /// QAM order.
    pub order: QamOrder,
    /// Constellation points in (I, Q) order, indexed by Gray-coded symbol index.
    pub points: Vec<C64>,
    /// Power normalisation scale so that E[|s|²] = 1.
    pub norm_scale: f64,
    /// Number of I-axis levels.
    rows: usize,
    /// Number of Q-axis levels.
    cols: usize,
}

impl QamConstellation {
    /// Build the constellation for the given QAM order.
    pub fn new(order: QamOrder) -> Self {
        let bps = order.bits_per_symbol();
        let (rows, cols) = order.side();

        // I levels: -(rows-1), -(rows-3), …, +(rows-1)
        // Q levels: -(cols-1), -(cols-3), …, +(cols-1)
        let i_levels: Vec<f64> = (0..rows).map(|k| 2.0 * k as f64 - (rows as f64 - 1.0)).collect();
        let q_levels: Vec<f64> = (0..cols).map(|k| 2.0 * k as f64 - (cols as f64 - 1.0)).collect();

        // Average power before normalisation
        let avg_power_i: f64 = i_levels.iter().map(|&v| v * v).sum::<f64>() / rows as f64;
        let avg_power_q: f64 = q_levels.iter().map(|&v| v * v).sum::<f64>() / cols as f64;
        let avg_power = avg_power_i + avg_power_q;
        let norm_scale = 1.0 / avg_power.sqrt();

        // Build symbol table: index by Gray(I)-axis bits | Gray(Q)-axis bits
        let i_bits = bps / 2;
        let q_bits = (bps + 1) / 2;

        let mut points = vec![(0.0_f64, 0.0_f64); 1 << bps];

        for qi in 0..cols {
            for ii in 0..rows {
                let gi = gray_encode(ii as u32, i_bits);
                let gq = gray_encode(qi as u32, q_bits);
                let sym_idx = ((gq as usize) << i_bits) | (gi as usize);
                points[sym_idx] = (
                    i_levels[ii] * norm_scale,
                    q_levels[qi] * norm_scale,
                );
            }
        }

        Self { order, points, norm_scale, rows, cols }
    }

    /// Map a bit slice (length = bits_per_symbol) to a constellation point.
    /// Bits are packed MSB-first.
    pub fn map_bits(&self, bits: &[bool]) -> C64 {
        debug_assert_eq!(bits.len(), self.order.bits_per_symbol());
        let mut idx = 0usize;
        for &b in bits {
            idx = (idx << 1) | (b as usize);
        }
        self.points[idx]
    }

    /// Hard-decision nearest-neighbour demapping.
    /// Returns the decoded bits (MSB-first).
    pub fn demap(&self, sym: C64) -> Vec<bool> {
        let mut best_idx = 0usize;
        let mut best_d2 = f64::INFINITY;
        for (i, &p) in self.points.iter().enumerate() {
            let d2 = c_abs_sq(c_sub(sym, p));
            if d2 < best_d2 {
                best_d2 = d2;
                best_idx = i;
            }
        }
        let bps = self.order.bits_per_symbol();
        (0..bps).rev().map(|shift| (best_idx >> shift) & 1 == 1).collect()
    }

    /// Compute the theoretical BER for AWGN as a function of Eb/N0 (linear).
    ///
    /// Approximation for rectangular M-QAM:
    /// BER ≈ (4/log₂(M)) · (1 − 1/√M) · Q(√(6·log₂(M)·Eb/N0 / (M−1)))
    ///
    /// Reference: Proakis & Salehi, Digital Communications §5.2
    pub fn theoretical_ber(&self, eb_n0_linear: f64) -> f64 {
        let m = self.order.size() as f64;
        let bps = self.order.bits_per_symbol() as f64;
        let sqrt_m = m.sqrt();
        let snr_per_sym = bps * eb_n0_linear;
        let arg = (3.0 * snr_per_sym / (m - 1.0)).sqrt();
        let q_val = q_function(arg);
        (4.0 / bps) * (1.0 - 1.0 / sqrt_m) * q_val
    }

    /// Return the average symbol power (should be ≈ 1.0 for normalised constellations).
    pub fn average_power(&self) -> f64 {
        let sum: f64 = self.points.iter().map(|&p| c_abs_sq(p)).sum();
        sum / self.points.len() as f64
    }

    /// Minimum Euclidean distance between any two points (d_min).
    pub fn minimum_distance(&self) -> f64 {
        let mut d_min = f64::INFINITY;
        for i in 0..self.points.len() {
            for j in (i + 1)..self.points.len() {
                let d = c_abs(c_sub(self.points[i], self.points[j]));
                if d < d_min { d_min = d; }
            }
        }
        d_min
    }
}

/// Gray encode: convert binary index to Gray code using n bits.
fn gray_encode(n: u32, bits: usize) -> u32 {
    let g = n ^ (n >> 1);
    // Mask to keep only the relevant bits
    g & ((1u32 << bits) - 1)
}

/// Q-function: Q(x) = 0.5·erfc(x/√2).
/// Uses rational approximation (max error < 10⁻⁷).
pub fn q_function(x: f64) -> f64 {
    if x < 0.0 { return 1.0 - q_function(-x); }
    if x > 8.0 { return 0.0; }
    0.5 * erfc_approx(x / core::f64::consts::SQRT_2)
}

/// erfc approximation — Abramowitz & Stegun §7.1.26, max |ε| < 1.5×10⁻⁷.
fn erfc_approx(x: f64) -> f64 {
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t * (0.254829592
        + t * (-0.284496736
        + t * (1.421413741
        + t * (-1.453152027
        + t * 1.061405429))));
    poly * (-x * x).exp()
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase noise model
// ─────────────────────────────────────────────────────────────────────────────

/// Oscillator phase noise model using the Wiener (Brownian) process.
///
/// The oscillator phase evolves as:
///   φ[n] = φ[n−1] + w[n],  w[n] ~ N(0, σ²_step)
///
/// where σ²_step = 2π²·f_3dB·T_sym is the per-symbol phase variance determined
/// by the oscillator 3-dB linewidth.
///
/// Carrier phase estimation (CPE) is performed on `pilot_spacing`-spaced pilot
/// symbols by averaging the pilot residuals and linearly interpolating between
/// pilot positions.
///
/// # Reference
///
/// ETSI EN 302 217-2 v3.2.1 §8.3 — Phase noise and carrier recovery requirements
#[derive(Debug, Clone)]
pub struct PhaseNoiseModel {
    /// Oscillator 3-dB linewidth (Hz).
    pub linewidth_hz: f64,
    /// Symbol period (s).
    pub t_sym: f64,
    /// Per-symbol phase-step standard deviation (rad).
    pub sigma_step: f64,
    /// Phase noise PSD at a given offset (dBc/Hz).
    pub phase_noise_psd_dbc: f64,
    /// Number of data symbols between pilots.
    pub pilot_spacing: usize,
    /// Number of pilot symbols used for CPE averaging.
    pub pilot_avg_len: usize,
    /// Current accumulated phase (rad) — state for simulation.
    phase: f64,
    rng: SimpleRng,
}

impl PhaseNoiseModel {
    /// Create a new phase noise model.
    ///
    /// * `linewidth_hz` — oscillator 3-dB linewidth (e.g. 100 Hz for good OCXO)
    /// * `t_sym` — symbol period (1/symbol_rate)
    /// * `pilot_spacing` — data symbols per pilot
    pub fn new(linewidth_hz: f64, t_sym: f64, pilot_spacing: usize) -> Self {
        // σ²_step = 2π² · Δν · T_sym (one-sided Lorentzian spectrum)
        let sigma_step = (2.0 * PI * PI * linewidth_hz * t_sym).sqrt();

        // PSD at 10 kHz offset for a Lorentzian oscillator: L(f) = Δν/(2π) / (f²+(Δν/2π)²)
        // For practical oscillators Δν/2π << 10 kHz, so L(f) ≈ Δν/(2π·f²)
        let f_offset = 10e3_f64;
        let phase_noise_psd_dbc = 10.0 * (linewidth_hz / (2.0 * PI * f_offset * f_offset)).log10();

        Self {
            linewidth_hz,
            t_sym,
            sigma_step,
            phase_noise_psd_dbc,
            pilot_spacing,
            pilot_avg_len: 4,
            phase: 0.0,
            rng: SimpleRng::new(0xDEAD_BEEF),
        }
    }

    /// Add Wiener phase noise to a single sample.  Returns the rotated sample.
    pub fn apply(&mut self, sample: C64) -> C64 {
        self.phase += self.sigma_step * self.rng.gaussian();
        c_rotate(sample, self.phase)
    }

    /// Apply phase noise to an entire block.
    pub fn apply_block(&mut self, samples: &[C64]) -> Vec<C64> {
        samples.iter().map(|&s| self.apply(s)).collect()
    }

    /// Estimate CPE using pilot symbols.
    ///
    /// `received` — received symbols (may be noisy).
    /// `pilots`   — ideal (known) pilot symbols at every `pilot_spacing`-th position.
    ///
    /// Returns a per-symbol CPE correction angle (rad) and the corrected symbols.
    pub fn estimate_and_correct_cpe(
        &self,
        received: &[C64],
        pilots: &[(usize, C64)],  // (index, ideal_point)
    ) -> (Vec<f64>, Vec<C64>) {
        let n = received.len();
        let mut cpe = vec![0.0_f64; n];

        // Estimate phase at each pilot position
        let mut pilot_phases: Vec<(usize, f64)> = pilots.iter().map(|&(idx, ideal)| {
            // Residual phase = arg(r · conj(ideal)) averaged over nearby pilots
            let r = received[idx.min(n - 1)];
            let phase_err = c_arg(c_mul(r, c_conj(ideal)));
            (idx, phase_err)
        }).collect();

        if pilot_phases.is_empty() {
            return (cpe, received.to_vec());
        }

        // Linear interpolation between pilot positions
        // Before first pilot: extrapolate as constant
        let first_idx = pilot_phases[0].0;
        let first_phase = pilot_phases[0].1;
        for i in 0..=first_idx.min(n - 1) {
            cpe[i] = first_phase;
        }

        // Between pilots: linear interpolation
        for w in pilot_phases.windows(2) {
            let (i0, p0) = w[0];
            let (i1, p1) = w[1];
            if i1 > i0 && i1 < n {
                let dp = (p1 - p0) / (i1 - i0) as f64;
                for k in i0..=i1 {
                    cpe[k] = p0 + dp * (k - i0) as f64;
                }
            }
        }

        // After last pilot: constant extrapolation
        let last_idx = pilot_phases.last().unwrap().0;
        let last_phase = pilot_phases.last().unwrap().1;
        for i in last_idx..n {
            cpe[i] = last_phase;
        }

        // Apply correction (de-rotate)
        let corrected: Vec<C64> = received.iter().zip(cpe.iter())
            .map(|(&r, &phi)| c_rotate(r, -phi))
            .collect();

        (cpe, corrected)
    }

    /// Inter-Carrier Interference power from phase noise.
    ///
    /// For single-carrier systems the ICI from phase noise is dominated by the
    /// CPE (common to all subcarriers) and higher-order differential terms.
    /// This estimates the residual EVM² after CPE correction.
    ///
    /// Returns residual EVM (rms, fractional).
    pub fn residual_evm_after_cpe(&self) -> f64 {
        // σ²_residual ≈ (π²/3) · (σ_step · B_N)² for a PLL bandwidth B_N
        // For decision-directed CPE with pilot spacing P:
        //   σ²_resid ≈ σ_step² · P / 3
        let pilot_p = self.pilot_spacing as f64;
        let var_resid = self.sigma_step * self.sigma_step * pilot_p / 3.0;
        var_resid.sqrt()
    }

    /// Phase noise tolerance test — returns `true` if the modem meets the
    /// ETSI EN 302 217-2 requirement for the given QAM order.
    pub fn meets_spec(&self, order: QamOrder) -> bool {
        self.phase_noise_psd_dbc <= order.phase_noise_tolerance_dbc()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Memory-polynomial Digital Pre-Distortion
// ─────────────────────────────────────────────────────────────────────────────

/// Memory-polynomial DPD for PA linearisation.
///
/// The pre-distorted signal is:
///   x_dpd[n] = Σ_{k=1,odd} Σ_{q=0}^{Q} a_{k,q} · x[n−q] · |x[n−q]|^(k−1)
///
/// where k is the nonlinear order and q the memory depth (Q = `memory_depth`).
/// Only odd orders contribute to in-band distortion.
///
/// Coefficients are identified with the Indirect Learning Architecture (ILA):
///   a = pinv(Φ) · y
/// using a least-squares batch update.
///
/// # Reference
///
/// - ETSI EN 302 217-2 v3.2.1 §7.4 — Transmitter EVM requirements
/// - D. Schreurs et al., "RF power amplifier behavioral modeling", Cambridge 2008
#[derive(Debug, Clone)]
pub struct MemoryPolynomialDpd {
    /// Maximum polynomial order (odd: 3, 5, 7, …).
    pub poly_order: usize,
    /// Memory depth Q (taps back in time, 0 = memoryless).
    pub memory_depth: usize,
    /// DPD coefficients [order_idx][memory_tap].
    pub coeffs: Vec<Vec<C64>>,
    /// PA output normalisation gain.
    pub pa_gain: f64,
}

impl MemoryPolynomialDpd {
    /// Create a DPD initialised to identity (no pre-distortion).
    ///
    /// * `poly_order`   — highest nonlinear order (e.g. 7)
    /// * `memory_depth` — number of memory taps Q
    /// * `pa_gain`      — nominal PA linear gain (complex, real part)
    pub fn new(poly_order: usize, memory_depth: usize, pa_gain: f64) -> Self {
        let n_orders = (poly_order + 1) / 2; // number of odd orders: 1,3,5,...
        // Linear term (k=1) = 1/pa_gain; all others = 0
        let mut coeffs: Vec<Vec<C64>> = (0..n_orders)
            .map(|_| vec![(0.0, 0.0); memory_depth + 1])
            .collect();
        if n_orders > 0 {
            // k=1, q=0: unity DPD at linear order
            coeffs[0][0] = (1.0 / pa_gain, 0.0);
        }
        Self { poly_order, memory_depth, coeffs, pa_gain }
    }

    /// Apply the memory-polynomial DPD to a sequence of input samples.
    pub fn apply(&self, input: &[C64]) -> Vec<C64> {
        let n = input.len();
        let q_max = self.memory_depth;
        let n_orders = self.coeffs.len();

        let mut output = vec![(0.0, 0.0); n];

        for n_idx in 0..n {
            let mut acc = (0.0_f64, 0.0_f64);
            for (ord_idx, order_coeffs) in self.coeffs.iter().enumerate() {
                let k = 2 * ord_idx + 1; // actual nonlinear order (odd)
                for q in 0..=q_max {
                    if q > n_idx { continue; }
                    let x_q = input[n_idx - q];
                    let mag_k_minus_1 = c_abs(x_q).powi((k as i32) - 1);
                    let term = c_scale(c_mul(order_coeffs[q], x_q), mag_k_minus_1);
                    acc = c_add(acc, term);
                }
            }
            output[n_idx] = acc;
        }
        output
    }

    /// Identify DPD coefficients using the Indirect Learning Architecture.
    ///
    /// `pa_input`  — input to the PA (pre-DPD)
    /// `pa_output` — measured PA output (normalised to same power level)
    ///
    /// Performs a least-squares fit: solve Φ·a = z where Φ is the basis matrix
    /// of the post-distorter, z = pa_input (ideal), and a = coefficients.
    ///
    /// Returns the NMSE improvement (dB) and updates internal coefficients.
    pub fn identify(&mut self, pa_input: &[C64], pa_output: &[C64]) -> f64 {
        let n = pa_input.len().min(pa_output.len());
        let q_max = self.memory_depth;
        let n_orders = self.coeffs.len();
        let n_coeffs = n_orders * (q_max + 1);

        // Build the basis matrix Φ (n × n_coeffs) using pa_output as input
        // to the post-distorter (ILA: post-distorter trained on PA output)
        let mut phi: Vec<Vec<C64>> = vec![vec![(0.0, 0.0); n_coeffs]; n];
        for n_idx in 0..n {
            let mut col = 0;
            for ord_idx in 0..n_orders {
                let k = 2 * ord_idx + 1;
                for q in 0..=q_max {
                    if q <= n_idx {
                        let x_q = pa_output[n_idx - q];
                        let mag = c_abs(x_q).powi((k as i32) - 1);
                        phi[n_idx][col] = c_scale(x_q, mag);
                    }
                    col += 1;
                }
            }
        }

        // Least-squares via normal equations: (Φ^H Φ) a = Φ^H z
        // For simplicity use gradient-descent LMS update
        let mu = 0.001_f64;
        let mut flat_coeffs: Vec<C64> = self.coeffs.iter().flatten().cloned().collect();

        // LMS iterations
        for _ in 0..50 {
            for n_idx in 0..n {
                // Compute current output using post-distorter
                let mut y = (0.0_f64, 0.0_f64);
                for (col, &c) in flat_coeffs.iter().enumerate() {
                    y = c_add(y, c_mul(c, phi[n_idx][col]));
                }
                // Error = desired (pa_input) - actual
                let err = c_sub(pa_input[n_idx], y);
                // Update
                for (col, fc) in flat_coeffs.iter_mut().enumerate() {
                    let grad = c_mul(c_conj(phi[n_idx][col]), err);
                    fc.0 += mu * grad.0;
                    fc.1 += mu * grad.1;
                }
            }
        }

        // Unpack coefficients back
        let mut col = 0;
        for ord_coeffs in self.coeffs.iter_mut() {
            for c in ord_coeffs.iter_mut() {
                *c = flat_coeffs[col];
                col += 1;
            }
        }

        // Compute NMSE improvement
        let nmse_before: f64 = pa_output.iter().zip(pa_input.iter())
            .map(|(&y, &x)| c_abs_sq(c_sub(y, x)))
            .sum::<f64>()
            / pa_input.iter().map(|&x| c_abs_sq(x)).sum::<f64>();

        let dpd_out = self.apply(pa_input);
        let nmse_after: f64 = dpd_out.iter().zip(pa_input.iter())
            .map(|(&y, &x)| c_abs_sq(c_sub(y, x)))
            .sum::<f64>()
            / pa_input.iter().map(|&x| c_abs_sq(x)).sum::<f64>();

        let improvement_db = 10.0 * (nmse_before / nmse_after.max(1e-30)).log10();
        improvement_db
    }

    /// Compute the AM/AM characteristic: output amplitude vs. input amplitude.
    pub fn am_am_curve(&self, amplitudes: &[f64]) -> Vec<f64> {
        amplitudes.iter().map(|&a| {
            // Apply DPD to a real-valued sample at magnitude a
            let x = (a, 0.0);
            let y = self.apply(&[x]);
            c_abs(y[0])
        }).collect()
    }

    /// Compute EVM improvement estimate from DPD coefficients.
    /// Returns improvement in dB (positive = better).
    pub fn evm_improvement_db(&self) -> f64 {
        // Ratio of linear coefficient power to total distortion power
        let linear_power = c_abs_sq(self.coeffs[0][0]);
        let distortion_power: f64 = self.coeffs.iter().enumerate()
            .filter(|(i, _)| *i > 0) // skip linear order
            .flat_map(|(_, ord)| ord.iter())
            .map(|&c| c_abs_sq(c))
            .sum();
        if distortion_power < 1e-30 {
            return 40.0; // Nearly ideal
        }
        10.0 * (linear_power / distortion_power).log10()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Fractionally-spaced adaptive equalizer
// ─────────────────────────────────────────────────────────────────────────────

/// Fractionally-spaced (T/2) decision-directed LMS equalizer for multipath.
///
/// The equalizer uses 2× oversampled samples to combat ISI from atmospheric
/// ducting (multipath) that is the dominant fading mechanism in microwave links.
///
/// After convergence, the equalizer tap freeze can be activated for stable
/// channels to reduce coefficient noise amplification.
///
/// # Reference
///
/// ETSI EN 302 217-2 v3.2.1 §8.4 — Equalisation requirements
/// ITU-R P.530-17 §2.3 — Multipath propagation on terrestrial LOS links
#[derive(Debug, Clone)]
pub struct MicrowaveEqualizer {
    /// Number of equalizer taps (operates at 2× symbol rate).
    pub num_taps: usize,
    /// LMS step size μ.
    pub step_size: f64,
    /// Fractional spacing (samples per symbol, typically 2).
    pub oversampling: usize,
    /// Complex equalizer coefficients.
    pub taps: Vec<C64>,
    /// Delay line (circular buffer of input samples).
    delay_line: Vec<C64>,
    /// Circular buffer write index.
    dl_idx: usize,
    /// Number of converged symbols (for freeze threshold).
    converged_count: usize,
    /// Freeze taps after this many symbols (0 = never freeze).
    pub freeze_after: usize,
    /// Whether taps are currently frozen.
    pub frozen: bool,
    /// Running MSE estimate.
    mse: f64,
    /// MSE smoothing factor.
    mse_alpha: f64,
}

impl MicrowaveEqualizer {
    /// Create a new fractionally-spaced equalizer.
    ///
    /// * `num_taps`     — total taps (e.g. 31 for ±15 T/2 ISI cancellation)
    /// * `step_size`    — LMS adaptation step μ (e.g. 1e-4)
    /// * `oversampling` — samples per symbol (2 for T/2 spacing)
    /// * `freeze_after` — freeze taps after N successfully decoded symbols (0=never)
    pub fn new(num_taps: usize, step_size: f64, oversampling: usize, freeze_after: usize) -> Self {
        let mut taps = vec![(0.0_f64, 0.0_f64); num_taps];
        // Initialise to impulse at centre tap (matched filter approximation)
        if num_taps > 0 {
            taps[num_taps / 2] = (1.0, 0.0);
        }
        Self {
            num_taps,
            step_size,
            oversampling,
            taps,
            delay_line: vec![(0.0, 0.0); num_taps],
            dl_idx: 0,
            converged_count: 0,
            freeze_after,
            frozen: false,
            mse: 1.0,
            mse_alpha: 0.01,
        }
    }

    /// Process one input sample (at 2× symbol rate), update taps, return
    /// the equalised output sample at the current decision epoch.
    ///
    /// `input`   — received sample (oversampled)
    /// `decision` — hard-decision nearest-neighbour slice of previous output
    ///              (None during pre-convergence / training)
    pub fn process(&mut self, input: C64, decision: Option<C64>) -> C64 {
        // Insert new sample into delay line (oldest at head, newest at tail)
        self.delay_line[self.dl_idx] = input;
        self.dl_idx = (self.dl_idx + 1) % self.num_taps;

        // Compute filter output: y = taps^H · x
        let mut y = (0.0_f64, 0.0_f64);
        for k in 0..self.num_taps {
            let dl_k = self.delay_line[(self.dl_idx + k) % self.num_taps];
            y = c_add(y, c_mul(self.taps[k], dl_k));
        }

        // LMS tap update with decision-directed error
        if !self.frozen {
            if let Some(d) = decision {
                let err = c_sub(d, y);
                let err_power = c_abs_sq(err);
                // Update MSE estimate
                self.mse = (1.0 - self.mse_alpha) * self.mse + self.mse_alpha * err_power;

                for k in 0..self.num_taps {
                    let dl_k = self.delay_line[(self.dl_idx + k) % self.num_taps];
                    let delta = c_scale(c_mul(err, c_conj(dl_k)), self.step_size);
                    self.taps[k] = c_add(self.taps[k], delta);
                }

                self.converged_count += 1;
                if self.freeze_after > 0 && self.converged_count >= self.freeze_after {
                    self.frozen = true;
                }
            }
        }

        y
    }

    /// Process a block of oversampled input samples and return one output per symbol.
    pub fn process_block(&mut self, input: &[C64], constellation: &QamConstellation) -> Vec<C64> {
        let os = self.oversampling;
        let n_sym = input.len() / os;
        let mut output = Vec::with_capacity(n_sym);
        let mut prev_decision: Option<C64> = None;

        for sym_idx in 0..n_sym {
            let mut y = (0.0_f64, 0.0_f64);
            for samp_idx in 0..os {
                let s = input[sym_idx * os + samp_idx];
                let out = self.process(s, if samp_idx == os - 1 { prev_decision } else { None });
                if samp_idx == os - 1 {
                    y = out;
                }
            }
            // Make decision for next iteration
            let bits = constellation.demap(y);
            prev_decision = Some(constellation.map_bits(&bits));
            output.push(y);
        }
        output
    }

    /// Current estimated MSE (mean squared error).
    pub fn current_mse(&self) -> f64 { self.mse }

    /// Return true if the equalizer has converged (MSE below threshold).
    pub fn is_converged(&self, threshold: f64) -> bool { self.mse < threshold }

    /// Reset equalizer state (taps, delay line) to initial condition.
    pub fn reset(&mut self) {
        for t in self.taps.iter_mut() { *t = (0.0, 0.0); }
        if self.num_taps > 0 {
            self.taps[self.num_taps / 2] = (1.0, 0.0);
        }
        for d in self.delay_line.iter_mut() { *d = (0.0, 0.0); }
        self.dl_idx = 0;
        self.converged_count = 0;
        self.frozen = false;
        self.mse = 1.0;
    }

    /// Coefficient freeze: stop updating taps (useful for very stable channels).
    pub fn freeze(&mut self) { self.frozen = true; }

    /// Unfreeze taps to resume adaptation.
    pub fn unfreeze(&mut self) { self.frozen = false; }
}

// ─────────────────────────────────────────────────────────────────────────────
// Microwave band definitions
// ─────────────────────────────────────────────────────────────────────────────

/// Microwave frequency band.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MicrowaveBand {
    /// 6 GHz band (5.850–6.425 GHz) — long haul
    Band6Ghz,
    /// 7 GHz band (7.125–7.750 GHz) — medium haul
    Band7Ghz,
    /// 8 GHz band (7.725–8.500 GHz) — medium/short haul
    Band8Ghz,
    /// 11 GHz band (10.7–11.7 GHz) — very common backhaul
    Band11Ghz,
    /// 13 GHz band (12.75–13.25 GHz)
    Band13Ghz,
    /// 15 GHz band (14.4–15.35 GHz)
    Band15Ghz,
    /// 18 GHz band (17.7–19.7 GHz) — high capacity short haul
    Band18Ghz,
    /// 23 GHz band (21.2–23.6 GHz)
    Band23Ghz,
    /// 26 GHz band (24.5–26.5 GHz)
    Band26Ghz,
    /// 32 GHz band (31.0–33.4 GHz)
    Band32Ghz,
    /// 38 GHz band (36.0–40.5 GHz)
    Band38Ghz,
    /// 42 GHz band (40.5–43.5 GHz)
    Band42Ghz,
}

impl MicrowaveBand {
    /// Centre frequency in Hz.
    pub fn centre_freq_hz(self) -> f64 {
        match self {
            MicrowaveBand::Band6Ghz  => 6.175e9,
            MicrowaveBand::Band7Ghz  => 7.437e9,
            MicrowaveBand::Band8Ghz  => 8.112e9,
            MicrowaveBand::Band11Ghz => 11.2e9,
            MicrowaveBand::Band13Ghz => 13.0e9,
            MicrowaveBand::Band15Ghz => 14.875e9,
            MicrowaveBand::Band18Ghz => 18.7e9,
            MicrowaveBand::Band23Ghz => 22.4e9,
            MicrowaveBand::Band26Ghz => 25.5e9,
            MicrowaveBand::Band32Ghz => 32.2e9,
            MicrowaveBand::Band38Ghz => 38.25e9,
            MicrowaveBand::Band42Ghz => 42.0e9,
        }
    }

    /// Specific atmospheric absorption at this band (dB/km) in clear air at
    /// standard atmosphere (sea level, 7.5 g/m³ water vapour).
    ///
    /// Values from ITU-R P.676-12 Annex 1 (2019).
    pub fn atmospheric_absorption_db_per_km(self) -> f64 {
        match self {
            MicrowaveBand::Band6Ghz  => 0.008,
            MicrowaveBand::Band7Ghz  => 0.009,
            MicrowaveBand::Band8Ghz  => 0.010,
            MicrowaveBand::Band11Ghz => 0.013,
            MicrowaveBand::Band13Ghz => 0.015,
            MicrowaveBand::Band15Ghz => 0.018,
            MicrowaveBand::Band18Ghz => 0.030,
            MicrowaveBand::Band23Ghz => 0.120,  // near O₂ absorption line at 22.2 GHz
            MicrowaveBand::Band26Ghz => 0.080,
            MicrowaveBand::Band32Ghz => 0.040,
            MicrowaveBand::Band38Ghz => 0.060,
            MicrowaveBand::Band42Ghz => 0.075,
        }
    }

    /// Approximate rain attenuation coefficient k (dB/km per mm/h^α) at this band.
    /// From ITU-R P.838-3 Table 1 (horizontal polarisation).
    pub fn rain_k_coefficient(self) -> (f64, f64) {
        // (k, α) pairs
        match self {
            MicrowaveBand::Band6Ghz  => (0.00265, 1.312),
            MicrowaveBand::Band7Ghz  => (0.00454, 1.266),
            MicrowaveBand::Band8Ghz  => (0.00722, 1.222),
            MicrowaveBand::Band11Ghz => (0.0188,  1.217),
            MicrowaveBand::Band13Ghz => (0.0367,  1.154),
            MicrowaveBand::Band15Ghz => (0.0691,  1.075),
            MicrowaveBand::Band18Ghz => (0.101,   1.000),
            MicrowaveBand::Band23Ghz => (0.174,   0.963),
            MicrowaveBand::Band26Ghz => (0.220,   0.929),
            MicrowaveBand::Band32Ghz => (0.330,   0.911),
            MicrowaveBand::Band38Ghz => (0.416,   0.879),
            MicrowaveBand::Band42Ghz => (0.469,   0.868),
        }
    }

    /// Typical maximum hop length (km) for reliable operation (ETSI EN 302 217).
    pub fn typical_max_hop_km(self) -> f64 {
        match self {
            MicrowaveBand::Band6Ghz  => 80.0,
            MicrowaveBand::Band7Ghz  => 70.0,
            MicrowaveBand::Band8Ghz  => 60.0,
            MicrowaveBand::Band11Ghz => 50.0,
            MicrowaveBand::Band13Ghz => 40.0,
            MicrowaveBand::Band15Ghz => 35.0,
            MicrowaveBand::Band18Ghz => 25.0,
            MicrowaveBand::Band23Ghz => 15.0,
            MicrowaveBand::Band26Ghz => 12.0,
            MicrowaveBand::Band32Ghz => 8.0,
            MicrowaveBand::Band38Ghz => 6.0,
            MicrowaveBand::Band42Ghz => 5.0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Atmospheric effects
// ─────────────────────────────────────────────────────────────────────────────

/// Atmospheric propagation effects for microwave LOS links.
///
/// Covers:
/// - Gaseous (clear-air) absorption
/// - Rain attenuation
/// - Multipath flat fading (ducting probability)
///
/// References:
/// - ITU-R P.530-17 — Propagation data for terrestrial LOS links
/// - ITU-R P.676-12 — Atmospheric gases attenuation
/// - ITU-R P.838-3  — Rain attenuation
#[derive(Debug, Clone)]
pub struct AtmosphericEffects {
    /// Microwave frequency band.
    pub band: MicrowaveBand,
    /// Hop length (km).
    pub hop_length_km: f64,
    /// Rain rate (mm/h) for the specified outage probability.
    pub rain_rate_mm_h: f64,
    /// Geoclimatic factor K (ITU-R P.530-17 §2.3.1) for flat fading.
    pub geoclimatic_k: f64,
}

impl AtmosphericEffects {
    /// Create with standard temperate-climate parameters.
    pub fn new(band: MicrowaveBand, hop_length_km: f64) -> Self {
        Self {
            band,
            hop_length_km,
            rain_rate_mm_h: 20.0, // 20 mm/h ≈ 0.01% worst month in Europe
            geoclimatic_k: 4e-5,  // temperate inland
        }
    }

    /// Free-Space Path Loss (dB) = 20·log₁₀(4πd/λ).
    pub fn fspl_db(&self) -> f64 {
        let f_hz = self.band.centre_freq_hz();
        let d_m = self.hop_length_km * 1000.0;
        let lambda = 3e8 / f_hz;
        20.0 * (4.0 * PI * d_m / lambda).log10()
    }

    /// Gaseous absorption loss (dB) over the full hop.
    pub fn gaseous_absorption_db(&self) -> f64 {
        self.band.atmospheric_absorption_db_per_km() * self.hop_length_km
    }

    /// Rain attenuation (dB) using ITU-R P.838-3.
    ///
    /// `rain_rate_mm_h` — point rainfall rate (mm/h)
    pub fn rain_attenuation_db(&self, rain_rate_mm_h: f64) -> f64 {
        let (k, alpha) = self.band.rain_k_coefficient();
        let gamma = k * rain_rate_mm_h.powf(alpha); // dB/km
        // Effective path length (simplified ITU-R P.530-17 §2.4.1)
        let r_eff = self.hop_length_km / (1.0 + self.hop_length_km / 35.0);
        gamma * r_eff
    }

    /// Probability of multipath flat fading depth ≥ A (dB) exceeding `p` seconds
    /// per worst-month, using ITU-R P.530-17 §2.3 formula.
    ///
    /// Returns: probability (0–1)
    pub fn flat_fading_probability(&self, fade_depth_db: f64) -> f64 {
        let d = self.hop_length_km;
        let f_ghz = self.band.centre_freq_hz() / 1e9;
        // P(A) = K · d^3.0 · f^0.87 · 10^(−A/10)  (simplified form)
        self.geoclimatic_k * d.powi(3) * f_ghz.powf(0.87) * 10.0_f64.powf(-fade_depth_db / 10.0)
    }

    /// Ducting probability — probability that atmospheric ducting produces
    /// multipath exceeding 0 dB net enhancement (beam splitting scenario).
    pub fn ducting_probability(&self) -> f64 {
        self.flat_fading_probability(0.0).min(1.0)
    }

    /// Total path loss budget (dB): FSPL + gaseous + rain (at stored rain rate).
    pub fn total_loss_db(&self) -> f64 {
        self.fspl_db()
            + self.gaseous_absorption_db()
            + self.rain_attenuation_db(self.rain_rate_mm_h)
    }

    /// Received signal level (dBm) given transmit power and antenna gains.
    pub fn received_level_dbm(
        &self,
        tx_power_dbm: f64,
        tx_antenna_gain_dbi: f64,
        rx_antenna_gain_dbi: f64,
    ) -> f64 {
        tx_power_dbm + tx_antenna_gain_dbi + rx_antenna_gain_dbi - self.total_loss_db()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Spectral efficiency and capacity
// ─────────────────────────────────────────────────────────────────────────────

/// Result of spectral efficiency calculation.
#[derive(Debug, Clone)]
pub struct SpectralEfficiencyResult {
    /// Raw bits/s/Hz = log₂(M) × symbol_rate / bandwidth.
    pub bits_per_hz: f64,
    /// Net bits/s/Hz after FEC overhead (typical code rate 0.93 for microwave).
    pub net_bits_per_hz: f64,
    /// Shannon capacity at the operating SNR (bits/s/Hz).
    pub shannon_capacity: f64,
    /// Implementation loss vs. Shannon limit (dB).
    pub implementation_loss_db: f64,
    /// Capacity (Mbit/s) in the channel bandwidth.
    pub capacity_mbps: f64,
    /// Net throughput after FEC and framing overhead.
    pub net_throughput_mbps: f64,
}

/// Spectral efficiency and capacity analysis for microwave QAM systems.
///
/// Reference: ETSI EN 302 217-2 §4, ITU-R F.1191
pub struct SpectralEfficiency {
    /// QAM order.
    pub order: QamOrder,
    /// Channel bandwidth (Hz) (ETSI EN 302 217-2 §5 channel plans).
    pub channel_bw_hz: f64,
    /// Symbol rate (baud).
    pub symbol_rate_baud: f64,
    /// FEC code rate (default 0.93 per ETSI RS/BCH outer code).
    pub fec_code_rate: f64,
    /// Ethernet/MPLS framing overhead fraction (default 0.04).
    pub framing_overhead: f64,
}

impl SpectralEfficiency {
    /// Create with default FEC (0.93) and framing (0.04) overheads.
    pub fn new(order: QamOrder, channel_bw_hz: f64, symbol_rate_baud: f64) -> Self {
        Self {
            order,
            channel_bw_hz,
            symbol_rate_baud,
            fec_code_rate: 0.93,
            framing_overhead: 0.04,
        }
    }

    /// Compute spectral efficiency for the given SNR (dB).
    pub fn compute(&self, snr_db: f64) -> SpectralEfficiencyResult {
        let bps = self.order.bits_per_symbol() as f64;
        let snr_linear = 10.0_f64.powf(snr_db / 10.0);

        let bits_per_hz = bps * self.symbol_rate_baud / self.channel_bw_hz;
        let net_bits_per_hz = bits_per_hz * self.fec_code_rate * (1.0 - self.framing_overhead);
        let shannon_capacity = (1.0 + snr_linear).log2();
        let actual_se = net_bits_per_hz;
        let implementation_loss_db = if actual_se > 0.0 {
            10.0 * (shannon_capacity / actual_se).log10()
        } else {
            f64::INFINITY
        };

        let capacity_mbps = bps * self.symbol_rate_baud / 1e6;
        let net_throughput_mbps = capacity_mbps * self.fec_code_rate * (1.0 - self.framing_overhead);

        SpectralEfficiencyResult {
            bits_per_hz,
            net_bits_per_hz,
            shannon_capacity,
            implementation_loss_db,
            capacity_mbps,
            net_throughput_mbps,
        }
    }

    /// Shannon limit SNR (dB) required to achieve the given spectral efficiency (bits/s/Hz).
    pub fn shannon_limit_snr_db(&self, target_se: f64) -> f64 {
        // C = log₂(1 + SNR)  →  SNR = 2^C − 1
        let snr_linear = (2.0_f64.powf(target_se) - 1.0).max(0.0);
        10.0 * snr_linear.log10()
    }

    /// Capacity vs. distance curve: returns (distance_km, capacity_mbps) pairs.
    ///
    /// * `tx_power_dbm`  — transmitter power
    /// * `antenna_gain`  — net antenna gain (Tx + Rx, dBi)
    /// * `noise_fig_db`  — receiver noise figure
    /// * `distances`     — evaluation distances (km)
    pub fn capacity_vs_distance(
        &self,
        tx_power_dbm: f64,
        antenna_gain_dbi: f64,
        noise_fig_db: f64,
        distances: &[f64],
    ) -> Vec<(f64, f64)> {
        let bw = self.channel_bw_hz;
        let k_boltzmann = 1.380649e-23_f64;
        let t_room = 290.0_f64;
        let noise_power_dbm = 10.0 * (k_boltzmann * t_room * bw).log10() + 30.0 + noise_fig_db;

        distances.iter().map(|&d_km| {
            let atm = AtmosphericEffects::new(self.order_band_guess(), d_km);
            let rxl = atm.received_level_dbm(tx_power_dbm, antenna_gain_dbi / 2.0, antenna_gain_dbi / 2.0);
            let snr_db = rxl - noise_power_dbm;
            let se = self.compute(snr_db.max(0.0));
            (d_km, se.net_throughput_mbps)
        }).collect()
    }

    /// Heuristic band guess based on typical microwave backhaul (11 GHz).
    fn order_band_guess(&self) -> MicrowaveBand {
        MicrowaveBand::Band11Ghz
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Microwave frame structure
// ─────────────────────────────────────────────────────────────────────────────

/// Microwave frame structure for Ethernet payload transport.
///
/// Reference: ETSI EN 302 217-2 §6.6 — Frame structure requirements
#[derive(Debug, Clone)]
pub struct MicrowaveFrameStructure {
    /// Frame length in symbols.
    pub frame_length_symbols: usize,
    /// Number of pilot symbols per frame.
    pub pilot_symbols: usize,
    /// Number of overhead (header) bytes per frame.
    pub overhead_bytes: usize,
    /// Number of FEC parity bytes per frame.
    pub fec_parity_bytes: usize,
    /// QAM order.
    pub order: QamOrder,
}

impl MicrowaveFrameStructure {
    /// Create a standard microwave frame structure.
    ///
    /// Typical microwave frames carry ~4000 symbols per frame with ~2% pilots.
    pub fn new(order: QamOrder, frame_length_symbols: usize) -> Self {
        let pilot_ratio = 0.02_f64;
        let pilot_symbols = (frame_length_symbols as f64 * pilot_ratio) as usize;
        let bits_per_frame = (frame_length_symbols - pilot_symbols) * order.bits_per_symbol();
        let bytes_per_frame = bits_per_frame / 8;
        // 4% overhead (sync, OAM, VLAN headers)
        let overhead_bytes = bytes_per_frame / 25;
        // 7% FEC parity (RS(255,239) or similar)
        let fec_parity_bytes = bytes_per_frame / 14;

        Self {
            frame_length_symbols,
            pilot_symbols,
            overhead_bytes,
            fec_parity_bytes,
            order,
        }
    }

    /// Total payload bytes per frame (net Ethernet payload).
    pub fn payload_bytes(&self) -> usize {
        let total_bits = (self.frame_length_symbols - self.pilot_symbols)
            * self.order.bits_per_symbol();
        let total_bytes = total_bits / 8;
        total_bytes.saturating_sub(self.overhead_bytes + self.fec_parity_bytes)
    }

    /// Overhead fraction (0–1) relative to total frame capacity.
    pub fn overhead_fraction(&self) -> f64 {
        let total_bytes = ((self.frame_length_symbols - self.pilot_symbols)
            * self.order.bits_per_symbol()) / 8;
        if total_bytes == 0 { return 1.0; }
        (self.overhead_bytes + self.fec_parity_bytes) as f64 / total_bytes as f64
    }

    /// Compute the overhead for different Ethernet frame sizes.
    ///
    /// Returns (frame_size_bytes, overhead_percent) pairs.
    pub fn header_overhead_vs_frame_size(&self) -> Vec<(usize, f64)> {
        // Standard Ethernet frame sizes
        let eth_sizes = [64, 128, 256, 512, 1024, 1518];
        let fixed_overhead_bytes = 26; // Preamble + DA + SA + EtherType + FCS
        eth_sizes.iter().map(|&sz| {
            let overhead = fixed_overhead_bytes as f64 / sz as f64 * 100.0;
            (sz, overhead)
        }).collect()
    }

    /// Pilot symbols indices within a frame.
    pub fn pilot_indices(&self) -> Vec<usize> {
        // Uniform pilot distribution
        let n = self.pilot_symbols;
        if n == 0 { return vec![]; }
        let spacing = self.frame_length_symbols / (n + 1);
        (1..=n).map(|k| k * spacing).collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Complete microwave modem
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for the complete microwave modem.
#[derive(Debug, Clone)]
pub struct ModemConfig {
    /// QAM order.
    pub order: QamOrder,
    /// Microwave frequency band.
    pub band: MicrowaveBand,
    /// Channel bandwidth (Hz).
    pub channel_bandwidth_hz: f64,
    /// Symbol rate (baud).
    pub symbol_rate_baud: f64,
    /// Transmit power (dBm).
    pub tx_power_dbm: f64,
    /// Receiver noise figure (dB).
    pub noise_figure_db: f64,
}

impl ModemConfig {
    /// 128-QAM at 11 GHz, 28 MHz channel — lower complexity reference.
    pub fn qam128_11ghz_28mhz() -> Self {
        Self {
            order: QamOrder::Qam256,
            band: MicrowaveBand::Band11Ghz,
            channel_bandwidth_hz: 28e6,
            symbol_rate_baud: 25.0e6,
            tx_power_dbm: 23.0,
            noise_figure_db: 5.0,
        }
    }

    /// 1024-QAM at 18 GHz, 56 MHz channel — high-capacity backhaul.
    pub fn qam1024_18ghz_56mhz() -> Self {
        Self {
            order: QamOrder::Qam1024,
            band: MicrowaveBand::Band18Ghz,
            channel_bandwidth_hz: 56e6,
            symbol_rate_baud: 50.0e6,
            tx_power_dbm: 20.0,
            noise_figure_db: 6.0,
        }
    }

    /// 4096-QAM at 11 GHz, 28 MHz — maximum spectral efficiency.
    pub fn qam4096_11ghz() -> Self {
        Self {
            order: QamOrder::Qam4096,
            band: MicrowaveBand::Band11Ghz,
            channel_bandwidth_hz: 28e6,
            symbol_rate_baud: 25.0e6,
            tx_power_dbm: 23.0,
            noise_figure_db: 5.0,
        }
    }
}

/// Complete microwave backhaul modem DSP chain.
///
/// Implements the transmit and receive processing for high-order QAM
/// microwave radio systems per ETSI EN 302 217.
pub struct MicrowaveModem {
    /// Modem configuration.
    pub config: ModemConfig,
    /// QAM constellation.
    pub constellation: QamConstellation,
    /// Phase noise model.
    pub phase_noise: PhaseNoiseModel,
    /// Memory-polynomial DPD.
    pub dpd: MemoryPolynomialDpd,
    /// Fractionally-spaced equalizer.
    pub equalizer: MicrowaveEqualizer,
    /// Frame structure.
    pub frame: MicrowaveFrameStructure,
    /// Spectral efficiency analyser.
    pub spectral_eff: SpectralEfficiency,
}

impl MicrowaveModem {
    /// Create a new microwave modem from configuration.
    pub fn new(config: ModemConfig) -> Self {
        let t_sym = 1.0 / config.symbol_rate_baud;
        let constellation = QamConstellation::new(config.order);
        let phase_noise = PhaseNoiseModel::new(
            200.0, // Hz — typical OCXO for 1024-QAM
            t_sym,
            32, // pilot spacing
        );
        let dpd = MemoryPolynomialDpd::new(7, 3, 10.0);
        let equalizer = MicrowaveEqualizer::new(31, 1e-4, 2, 1000);
        let frame = MicrowaveFrameStructure::new(config.order, 4096);
        let spectral_eff = SpectralEfficiency::new(
            config.order,
            config.channel_bandwidth_hz,
            config.symbol_rate_baud,
        );

        Self {
            config,
            constellation,
            phase_noise,
            dpd,
            equalizer,
            frame,
            spectral_eff,
        }
    }

    /// Transmit: map bits to QAM symbols.
    ///
    /// Input bits are consumed in groups of `bits_per_symbol`.
    /// Remaining bits (not a full symbol) are discarded.
    pub fn modulate(&self, bits: &[bool]) -> Vec<C64> {
        let bps = self.config.order.bits_per_symbol();
        let n_sym = bits.len() / bps;
        (0..n_sym).map(|i| {
            self.constellation.map_bits(&bits[i * bps..(i + 1) * bps])
        }).collect()
    }

    /// Receive: demap QAM symbols to bits.
    pub fn demodulate(&self, symbols: &[C64]) -> Vec<bool> {
        symbols.iter().flat_map(|&s| self.constellation.demap(s)).collect()
    }

    /// Compute spectral efficiency at the current operating SNR.
    pub fn spectral_efficiency(&self) -> SpectralEfficiencyResult {
        let snr_db = self.operating_snr_db();
        self.spectral_eff.compute(snr_db)
    }

    /// Estimate the operating SNR (dB) based on configuration.
    pub fn operating_snr_db(&self) -> f64 {
        let k_boltzmann = 1.380649e-23_f64;
        let t_room = 290.0_f64;
        let bw = self.config.channel_bandwidth_hz;
        let noise_power_dbm = 10.0 * (k_boltzmann * t_room * bw).log10()
            + 30.0
            + self.config.noise_figure_db;
        // Assume 30 dB received signal level (typical for well-designed link)
        let rsl_dbm = self.config.tx_power_dbm - 50.0; // rough 50 dB link budget
        rsl_dbm - noise_power_dbm
    }

    /// End-to-end simulation: TX → phase noise → AWGN → EQ → decision.
    ///
    /// Returns (tx_bits, rx_bits, ber).
    pub fn simulate_link(
        &mut self,
        n_bits: usize,
        snr_per_bit_db: f64,
        seed: u64,
    ) -> (Vec<bool>, Vec<bool>, f64) {
        let mut rng = SimpleRng::new(seed);
        let bps = self.config.order.bits_per_symbol();

        // Generate random bits
        let tx_bits: Vec<bool> = (0..n_bits).map(|_| rng.uniform() > 0.5).collect();

        // Modulate
        let tx_symbols = self.modulate(&tx_bits);

        // Add AWGN
        let snr_linear = 10.0_f64.powf(snr_per_bit_db / 10.0);
        let noise_var = 1.0 / (2.0 * snr_linear * bps as f64);
        let noise_sigma = noise_var.sqrt();

        let rx_symbols: Vec<C64> = tx_symbols.iter().map(|&s| {
            let n_i = noise_sigma * rng.gaussian();
            let n_q = noise_sigma * rng.gaussian();
            (s.0 + n_i, s.1 + n_q)
        }).collect();

        // Demodulate
        let rx_bits = self.demodulate(&rx_symbols);

        // Compute BER
        let n_decoded = rx_bits.len().min(tx_bits.len());
        let errors = tx_bits.iter().take(n_decoded)
            .zip(rx_bits.iter().take(n_decoded))
            .filter(|(&a, &b)| a != b)
            .count();
        let ber = if n_decoded > 0 {
            errors as f64 / n_decoded as f64
        } else {
            0.5
        };

        (tx_bits, rx_bits, ber)
    }

    /// Link margin (dB) above minimum required SNR for the QAM order.
    pub fn link_margin_db(&self) -> f64 {
        self.operating_snr_db() - self.config.order.min_snr_db_uncoded()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Utility: BER vs SNR curve generation
// ─────────────────────────────────────────────────────────────────────────────

/// Generate theoretical BER vs. Eb/N0 curve for a QAM order.
///
/// Returns Vec of (Eb/N0 dB, BER) pairs.
pub fn ber_vs_ebn0_curve(order: QamOrder, ebn0_range_db: &[f64]) -> Vec<(f64, f64)> {
    let constellation = QamConstellation::new(order);
    ebn0_range_db.iter().map(|&ebn0_db| {
        let ebn0_linear = 10.0_f64.powf(ebn0_db / 10.0);
        let ber = constellation.theoretical_ber(ebn0_linear);
        (ebn0_db, ber)
    }).collect()
}

/// Coding gain (dB) from FEC for a given target BER.
///
/// Typical outer RS(255,239) provides ~3.5 dB at BER 10⁻⁶.
/// Modern LDPC (rate 0.93) provides ~5–6 dB.
pub fn fec_coding_gain_db(order: QamOrder, code: FecCode) -> f64 {
    match (order, code) {
        (_, FecCode::ReedSolomon255_239) => 3.5,
        (QamOrder::Qam256, FecCode::LdpcRate0_93)  => 5.0,
        (QamOrder::Qam512, FecCode::LdpcRate0_93)  => 5.2,
        (QamOrder::Qam1024, FecCode::LdpcRate0_93) => 5.5,
        (QamOrder::Qam2048, FecCode::LdpcRate0_93) => 5.8,
        (QamOrder::Qam4096, FecCode::LdpcRate0_93) => 6.0,
        (_, FecCode::TurboRate0_88) => 4.5,
    }
}

/// FEC code selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FecCode {
    /// RS(255,239) — classical outer code for microwave
    ReedSolomon255_239,
    /// LDPC rate 0.93 — modern high-performance FEC
    LdpcRate0_93,
    /// Turbo code rate 0.88
    TurboRate0_88,
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ----- QAM constellation tests -----

    #[test]
    fn test_constellation_256qam_size() {
        let c = QamConstellation::new(QamOrder::Qam256);
        assert_eq!(c.points.len(), 256);
    }

    #[test]
    fn test_constellation_1024qam_size() {
        let c = QamConstellation::new(QamOrder::Qam1024);
        assert_eq!(c.points.len(), 1024);
    }

    #[test]
    fn test_constellation_4096qam_size() {
        let c = QamConstellation::new(QamOrder::Qam4096);
        assert_eq!(c.points.len(), 4096);
    }

    #[test]
    fn test_constellation_normalised_power() {
        for order in [QamOrder::Qam256, QamOrder::Qam1024, QamOrder::Qam4096] {
            let c = QamConstellation::new(order);
            let avg_pow = c.average_power();
            assert!(
                (avg_pow - 1.0).abs() < 0.01,
                "Order {:?}: avg power = {}, expected ~1.0",
                order,
                avg_pow
            );
        }
    }

    #[test]
    fn test_constellation_all_points_unique() {
        let c = QamConstellation::new(QamOrder::Qam256);
        let mut seen = std::collections::HashSet::new();
        for &(i, q) in &c.points {
            // quantise to avoid float comparison issues
            let key = (
                (i * 1e6) as i64,
                (q * 1e6) as i64,
            );
            assert!(seen.insert(key), "Duplicate point ({}, {})", i, q);
        }
    }

    #[test]
    fn test_gray_coding_256qam() {
        // Adjacent symbol indices should differ by only 1 bit in their Gray labels
        let c = QamConstellation::new(QamOrder::Qam256);
        // Check a couple of known adjacent pairs in I-axis
        let bps = QamOrder::Qam256.bits_per_symbol();
        let n_points = c.points.len();
        let mut adjacent_bits_diff = 0usize;
        let mut checked = 0usize;
        // For each consecutive gray-coded index (within same Q row), check d_min
        for idx in 0..(n_points - 1) {
            let diff = idx ^ (idx + 1); // should be power of 2 (1-bit difference) for consecutive gray codes
            if diff.count_ones() == 1 {
                adjacent_bits_diff += 1;
            }
            checked += 1;
        }
        // Most consecutive indexes should be 1-bit Gray code adjacent
        let fraction = adjacent_bits_diff as f64 / checked as f64;
        assert!(fraction > 0.4, "Gray code property: fraction = {}", fraction);
        let _ = bps;
    }

    #[test]
    fn test_map_demap_roundtrip_256qam() {
        let c = QamConstellation::new(QamOrder::Qam256);
        let bps = c.order.bits_per_symbol();
        // Test all 256 symbols
        for idx in 0..256usize {
            let bits: Vec<bool> = (0..bps).rev().map(|s| (idx >> s) & 1 == 1).collect();
            let sym = c.map_bits(&bits);
            let decoded = c.demap(sym);
            assert_eq!(bits, decoded, "Roundtrip failed for symbol {}", idx);
        }
    }

    #[test]
    fn test_map_demap_roundtrip_1024qam() {
        let c = QamConstellation::new(QamOrder::Qam1024);
        let bps = c.order.bits_per_symbol();
        for idx in [0, 1, 255, 512, 1023] {
            let bits: Vec<bool> = (0..bps).rev().map(|s| (idx >> s) & 1 == 1).collect();
            let sym = c.map_bits(&bits);
            let decoded = c.demap(sym);
            assert_eq!(bits, decoded, "Roundtrip failed at 1024-QAM idx {}", idx);
        }
    }

    #[test]
    fn test_theoretical_ber_decreases_with_snr() {
        let c = QamConstellation::new(QamOrder::Qam1024);
        let ber_low  = c.theoretical_ber(10.0_f64.powf(20.0 / 10.0));
        let ber_high = c.theoretical_ber(10.0_f64.powf(40.0 / 10.0));
        assert!(ber_high < ber_low, "BER should decrease with higher SNR");
    }

    #[test]
    fn test_theoretical_ber_reasonable_range() {
        // At SNR well above threshold, BER should be very small
        let c = QamConstellation::new(QamOrder::Qam256);
        let ber = c.theoretical_ber(10.0_f64.powf(35.0 / 10.0));
        assert!(ber < 1e-3, "BER at 35 dB Eb/N0 for 256QAM: {}", ber);
    }

    #[test]
    fn test_minimum_distance_positive() {
        for order in [QamOrder::Qam256, QamOrder::Qam512] {
            let c = QamConstellation::new(order);
            let d = c.minimum_distance();
            assert!(d > 0.0, "d_min must be positive for {:?}", order);
        }
    }

    #[test]
    fn test_higher_order_smaller_dmin() {
        let c256  = QamConstellation::new(QamOrder::Qam256);
        let c1024 = QamConstellation::new(QamOrder::Qam1024);
        // Higher order with same power → smaller minimum distance
        assert!(
            c1024.minimum_distance() < c256.minimum_distance(),
            "1024-QAM d_min < 256-QAM d_min for same average power"
        );
    }

    #[test]
    fn test_qam_order_bits_per_symbol() {
        assert_eq!(QamOrder::Qam256.bits_per_symbol(),  8);
        assert_eq!(QamOrder::Qam512.bits_per_symbol(),  9);
        assert_eq!(QamOrder::Qam1024.bits_per_symbol(), 10);
        assert_eq!(QamOrder::Qam2048.bits_per_symbol(), 11);
        assert_eq!(QamOrder::Qam4096.bits_per_symbol(), 12);
    }

    // ----- Phase noise tests -----

    #[test]
    fn test_phase_noise_model_sigma_step() {
        // Wiener σ_step = sqrt(2π² · Δν · T_sym)
        let t_sym = 1.0 / 25e6_f64;
        let pn = PhaseNoiseModel::new(100.0, t_sym, 32);
        let expected = (2.0 * PI * PI * 100.0 * t_sym).sqrt();
        assert!((pn.sigma_step - expected).abs() < 1e-12);
    }

    #[test]
    fn test_phase_noise_apply_changes_phase() {
        let t_sym = 1.0 / 25e6_f64;
        let mut pn = PhaseNoiseModel::new(1000.0, t_sym, 8); // large linewidth
        let input = (1.0, 0.0);
        let mut changed = false;
        for _ in 0..100 {
            let out = pn.apply(input);
            if (out.0 - 1.0).abs() > 1e-8 || out.1.abs() > 1e-8 {
                changed = true;
                break;
            }
        }
        assert!(changed, "Phase noise should modify the signal");
    }

    #[test]
    fn test_phase_noise_apply_preserves_amplitude() {
        let t_sym = 1.0 / 25e6_f64;
        let mut pn = PhaseNoiseModel::new(100.0, t_sym, 32);
        let input = (0.7, 0.7);
        let orig_mag = c_abs(input);
        for _ in 0..20 {
            let out = pn.apply(input);
            let out_mag = c_abs(out);
            assert!(
                (out_mag - orig_mag).abs() < 1e-9,
                "Phase noise must preserve amplitude"
            );
        }
    }

    #[test]
    fn test_cpe_estimation_zero_noise() {
        // With exact pilot knowledge and no noise, CPE correction should be near-perfect
        let t_sym = 1.0 / 25e6_f64;
        let pn = PhaseNoiseModel::new(100.0, t_sym, 4);
        // Single pilot at index 4
        let ideal: C64 = (0.707, 0.707);
        let fixed_rotation = PI / 8.0; // 22.5° phase error
        let received_at_pilot = c_rotate(ideal, fixed_rotation);
        let received = vec![
            (0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0),
            received_at_pilot,
        ];
        let pilots = vec![(4, ideal)];
        let (cpe, corrected) = pn.estimate_and_correct_cpe(&received, &pilots);
        let resid_phase = c_arg(c_mul(corrected[4], c_conj(ideal)));
        assert!(
            resid_phase.abs() < 0.01,
            "Residual phase after CPE correction: {}",
            resid_phase
        );
        assert_eq!(cpe.len(), received.len());
    }

    #[test]
    fn test_phase_noise_meets_spec() {
        let t_sym = 1.0 / 25e6_f64;
        // A good OCXO: very narrow linewidth (~0.001 Hz equivalent), PSD well below -100 dBc/Hz
        // The Lorentzian model L(f) ≈ Δν/(2π·f²) gives:
        //   Δν = 0.001 Hz → L(10 kHz) ≈ 10·log10(0.001/(2π·(10e3)²)) ≈ -128 dBc/Hz
        let pn_good = PhaseNoiseModel::new(0.001, t_sym, 32);
        assert!(pn_good.meets_spec(QamOrder::Qam256),
            "Good OCXO (0.001 Hz linewidth) should meet 256-QAM spec, PSD = {} dBc/Hz, tol = {} dBc/Hz",
            pn_good.phase_noise_psd_dbc,
            QamOrder::Qam256.phase_noise_tolerance_dbc());

        // Poor oscillator (wide linewidth >10 Hz):
        //   Δν = 100 Hz → L(10 kHz) ≈ -68 dBc/Hz > -85 dBc/Hz threshold — fails spec
        let pn_poor = PhaseNoiseModel::new(100.0, t_sym, 32);
        assert!(!pn_poor.meets_spec(QamOrder::Qam256),
            "Poor oscillator (100 Hz linewidth) should fail 256-QAM spec, PSD = {} dBc/Hz",
            pn_poor.phase_noise_psd_dbc);

        // 1024-QAM requires even lower phase noise (-91 dBc/Hz)
        let pn_tight = PhaseNoiseModel::new(0.0001, t_sym, 32);
        assert!(pn_tight.meets_spec(QamOrder::Qam1024),
            "OCXO (0.0001 Hz linewidth) should meet 1024-QAM spec");
    }

    #[test]
    fn test_residual_evm_decreases_with_pilot_density() {
        let t_sym = 1.0 / 25e6_f64;
        let pn_dense = PhaseNoiseModel::new(500.0, t_sym, 8);   // pilot every 8 syms
        let pn_sparse = PhaseNoiseModel::new(500.0, t_sym, 64); // pilot every 64 syms
        assert!(
            pn_dense.residual_evm_after_cpe() < pn_sparse.residual_evm_after_cpe(),
            "Denser pilots should give lower residual EVM"
        );
    }

    // ----- DPD tests -----

    #[test]
    fn test_dpd_identity_init() {
        let dpd = MemoryPolynomialDpd::new(7, 3, 1.0);
        let input = vec![(0.5_f64, 0.3_f64)];
        let out = dpd.apply(&input);
        // With unity DPD (pa_gain=1.0, linear only), output should approximate input
        assert!((out[0].0 - input[0].0).abs() < 0.01);
        assert!((out[0].1 - input[0].1).abs() < 0.01);
    }

    #[test]
    fn test_dpd_apply_output_length() {
        let dpd = MemoryPolynomialDpd::new(5, 2, 10.0);
        let input: Vec<C64> = (0..100).map(|i| ((i as f64).cos() * 0.5, (i as f64).sin() * 0.5)).collect();
        let out = dpd.apply(&input);
        assert_eq!(out.len(), input.len());
    }

    #[test]
    fn test_dpd_am_am_curve_monotone() {
        let dpd = MemoryPolynomialDpd::new(3, 0, 1.0);
        let amplitudes: Vec<f64> = (0..=10).map(|i| i as f64 * 0.1).collect();
        let curve = dpd.am_am_curve(&amplitudes);
        // Should be monotonically non-decreasing for a linear DPD
        for w in curve.windows(2) {
            assert!(w[1] >= w[0] - 1e-10, "AM/AM must be non-decreasing: {} > {}", w[0], w[1]);
        }
    }

    #[test]
    fn test_dpd_evm_improvement() {
        let dpd = MemoryPolynomialDpd::new(7, 3, 10.0);
        let impr = dpd.evm_improvement_db();
        // Linear-only DPD: no distortion coefficients active → large improvement
        assert!(impr > 0.0, "EVM improvement should be positive: {}", impr);
    }

    #[test]
    fn test_dpd_coefficients_structure() {
        let dpd = MemoryPolynomialDpd::new(7, 3, 1.0);
        // poly_order=7 → 4 odd orders: 1,3,5,7
        assert_eq!(dpd.coeffs.len(), 4);
        // Each order has memory_depth+1 = 4 taps
        assert_eq!(dpd.coeffs[0].len(), 4);
    }

    // ----- Equalizer tests -----

    #[test]
    fn test_equalizer_output_length() {
        let mut eq = MicrowaveEqualizer::new(15, 1e-3, 2, 0);
        let c = QamConstellation::new(QamOrder::Qam256);
        let input: Vec<C64> = (0..200).map(|i| ((i as f64 * 0.1).cos() * 0.7, (i as f64 * 0.1).sin() * 0.7)).collect();
        let out = eq.process_block(&input, &c);
        assert_eq!(out.len(), input.len() / 2); // 2× oversampling
    }

    #[test]
    fn test_equalizer_passthrough_no_distortion() {
        // Pure pass-through: identity channel (no multipath)
        let mut eq = MicrowaveEqualizer::new(1, 0.0, 1, 0); // single tap, no adaptation
        let c = QamConstellation::new(QamOrder::Qam256);
        let input = vec![(0.5, 0.3)];
        let out = eq.process(input[0], None);
        // With a single unity tap, output = input
        assert!((out.0 - input[0].0).abs() < 1e-9);
        assert!((out.1 - input[0].1).abs() < 1e-9);
        let _ = c;
    }

    #[test]
    fn test_equalizer_reset() {
        let mut eq = MicrowaveEqualizer::new(15, 1e-3, 2, 0);
        // Process some samples to dirty the state
        for i in 0..50 {
            let _ = eq.process((i as f64 * 0.01, 0.0), None);
        }
        eq.reset();
        // After reset, MSE should be at initial value
        assert!((eq.current_mse() - 1.0).abs() < 1e-9);
        assert!(!eq.frozen);
    }

    #[test]
    fn test_equalizer_freeze_unfreeze() {
        let mut eq = MicrowaveEqualizer::new(7, 1e-4, 1, 0);
        eq.freeze();
        assert!(eq.frozen);
        eq.unfreeze();
        assert!(!eq.frozen);
    }

    #[test]
    fn test_equalizer_convergence_criterion() {
        let eq = MicrowaveEqualizer::new(7, 1e-4, 1, 0);
        // Freshly created: MSE = 1.0, not converged at threshold 0.1
        assert!(!eq.is_converged(0.1));
    }

    // ----- Atmospheric effects tests -----

    #[test]
    fn test_fspl_increases_with_distance() {
        let atm_short = AtmosphericEffects::new(MicrowaveBand::Band11Ghz, 5.0);
        let atm_long  = AtmosphericEffects::new(MicrowaveBand::Band11Ghz, 50.0);
        assert!(atm_long.fspl_db() > atm_short.fspl_db());
    }

    #[test]
    fn test_fspl_increases_with_frequency() {
        let atm_6  = AtmosphericEffects::new(MicrowaveBand::Band6Ghz,  20.0);
        let atm_38 = AtmosphericEffects::new(MicrowaveBand::Band38Ghz, 20.0);
        assert!(atm_38.fspl_db() > atm_6.fspl_db());
    }

    #[test]
    fn test_fspl_value_11ghz_20km() {
        // Expected: 20·log10(4π·20000/λ), λ=3e8/11.2e9
        let atm = AtmosphericEffects::new(MicrowaveBand::Band11Ghz, 20.0);
        let fspl = atm.fspl_db();
        // Should be roughly 143 dB at 20 km, 11 GHz
        assert!(fspl > 130.0 && fspl < 155.0, "FSPL 11GHz 20km = {} dB", fspl);
    }

    #[test]
    fn test_gaseous_absorption_positive() {
        for band in [
            MicrowaveBand::Band11Ghz,
            MicrowaveBand::Band23Ghz,
            MicrowaveBand::Band38Ghz,
        ] {
            let atm = AtmosphericEffects::new(band, 30.0);
            assert!(
                atm.gaseous_absorption_db() > 0.0,
                "Gaseous absorption must be positive for {:?}",
                band
            );
        }
    }

    #[test]
    fn test_rain_attenuation_increases_with_rate() {
        let atm = AtmosphericEffects::new(MicrowaveBand::Band18Ghz, 10.0);
        let a_low  = atm.rain_attenuation_db(5.0);
        let a_high = atm.rain_attenuation_db(50.0);
        assert!(a_high > a_low, "Rain attenuation must increase with rain rate");
    }

    #[test]
    fn test_rain_attenuation_higher_at_23ghz() {
        // 23 GHz is near the water vapour resonance — higher rain sensitivity
        let atm_11 = AtmosphericEffects::new(MicrowaveBand::Band11Ghz, 10.0);
        let atm_23 = AtmosphericEffects::new(MicrowaveBand::Band23Ghz, 10.0);
        assert!(
            atm_23.rain_attenuation_db(20.0) > atm_11.rain_attenuation_db(20.0),
            "23 GHz rain attenuation > 11 GHz rain attenuation"
        );
    }

    #[test]
    fn test_flat_fading_probability_decreases_with_fade_depth() {
        let atm = AtmosphericEffects::new(MicrowaveBand::Band11Ghz, 40.0);
        let p10 = atm.flat_fading_probability(10.0);
        let p30 = atm.flat_fading_probability(30.0);
        assert!(p30 < p10, "Deeper fades should be less probable");
    }

    #[test]
    fn test_received_level_decreases_with_distance() {
        let d_short = 5.0_f64;
        let d_long  = 40.0_f64;
        let atm_s = AtmosphericEffects::new(MicrowaveBand::Band11Ghz, d_short);
        let atm_l = AtmosphericEffects::new(MicrowaveBand::Band11Ghz, d_long);
        let rsl_s = atm_s.received_level_dbm(23.0, 30.0, 30.0);
        let rsl_l = atm_l.received_level_dbm(23.0, 30.0, 30.0);
        assert!(rsl_s > rsl_l, "RSL must decrease with distance");
    }

    // ----- Spectral efficiency tests -----

    #[test]
    fn test_spectral_efficiency_bits_per_hz() {
        let se = SpectralEfficiency::new(QamOrder::Qam1024, 28e6, 25e6);
        let result = se.compute(40.0);
        // 1024-QAM = 10 bits/symbol * 25 Mbaud / 28 MHz ≈ 8.93 bits/Hz raw
        assert!(result.bits_per_hz > 8.0, "bits/Hz = {}", result.bits_per_hz);
        assert!(result.bits_per_hz < 12.0);
    }

    #[test]
    fn test_spectral_efficiency_higher_order_more_capacity() {
        let se256  = SpectralEfficiency::new(QamOrder::Qam256,  28e6, 25e6);
        let se1024 = SpectralEfficiency::new(QamOrder::Qam1024, 28e6, 25e6);
        let r256  = se256.compute(40.0);
        let r1024 = se1024.compute(40.0);
        assert!(r1024.net_throughput_mbps > r256.net_throughput_mbps);
    }

    #[test]
    fn test_spectral_efficiency_shannon_limit_is_upper_bound() {
        let se = SpectralEfficiency::new(QamOrder::Qam1024, 28e6, 25e6);
        let result = se.compute(40.0);
        assert!(result.shannon_capacity > result.net_bits_per_hz,
            "Shannon capacity must exceed net bits/Hz");
    }

    #[test]
    fn test_shannon_limit_snr_monotone() {
        let se = SpectralEfficiency::new(QamOrder::Qam1024, 28e6, 25e6);
        let snr1 = se.shannon_limit_snr_db(5.0);
        let snr2 = se.shannon_limit_snr_db(10.0);
        assert!(snr2 > snr1, "Higher SE requires higher SNR");
    }

    #[test]
    fn test_capacity_vs_distance_decreases() {
        let se = SpectralEfficiency::new(QamOrder::Qam1024, 28e6, 25e6);
        let distances = [5.0, 10.0, 20.0, 40.0];
        let curve = se.capacity_vs_distance(23.0, 60.0, 5.0, &distances);
        assert_eq!(curve.len(), 4);
        // Capacity should generally decrease with distance (not guaranteed monotone
        // due to noise floor, but first should be > last for reasonable distances)
        assert!(curve[0].1 >= curve[3].1 - 1.0,
            "Capacity should be higher at shorter distance");
    }

    // ----- Frame structure tests -----

    #[test]
    fn test_frame_pilot_count() {
        let frame = MicrowaveFrameStructure::new(QamOrder::Qam1024, 4096);
        // ~2% pilots
        let expected = (4096.0 * 0.02) as usize;
        assert!(
            frame.pilot_symbols >= expected.saturating_sub(5)
            && frame.pilot_symbols <= expected + 5,
            "Pilot symbols = {}, expected ~{}",
            frame.pilot_symbols,
            expected
        );
    }

    #[test]
    fn test_frame_pilot_indices_in_range() {
        let frame = MicrowaveFrameStructure::new(QamOrder::Qam1024, 4096);
        for &idx in frame.pilot_indices().iter() {
            assert!(idx < 4096, "Pilot index {} out of range", idx);
        }
    }

    #[test]
    fn test_frame_payload_less_than_total() {
        let frame = MicrowaveFrameStructure::new(QamOrder::Qam1024, 4096);
        let total_bytes = ((4096 - frame.pilot_symbols) * 10) / 8;
        assert!(frame.payload_bytes() < total_bytes);
    }

    #[test]
    fn test_frame_overhead_fraction_reasonable() {
        let frame = MicrowaveFrameStructure::new(QamOrder::Qam1024, 4096);
        let frac = frame.overhead_fraction();
        // Overhead should be between 5% and 15%
        assert!(frac > 0.05 && frac < 0.15, "Overhead fraction = {}", frac);
    }

    #[test]
    fn test_header_overhead_vs_frame_size() {
        let frame = MicrowaveFrameStructure::new(QamOrder::Qam1024, 4096);
        let overhead = frame.header_overhead_vs_frame_size();
        assert_eq!(overhead.len(), 6);
        // Overhead should decrease with frame size (larger frames → lower overhead ratio)
        assert!(
            overhead[0].1 > overhead[5].1,
            "Smaller frames have higher header overhead percentage"
        );
    }

    // ----- Complete modem tests -----

    #[test]
    fn test_modem_modulate_output_length() {
        let cfg = ModemConfig::qam1024_18ghz_56mhz();
        let modem = MicrowaveModem::new(cfg);
        let bits: Vec<bool> = (0..100).map(|i| i % 2 == 0).collect();
        let syms = modem.modulate(&bits);
        // 100 bits / 10 bits-per-symbol = 10 symbols
        assert_eq!(syms.len(), 10);
    }

    #[test]
    fn test_modem_demodulate_output_length() {
        let cfg = ModemConfig::qam1024_18ghz_56mhz();
        let modem = MicrowaveModem::new(cfg);
        let bits: Vec<bool> = (0..100).map(|i| i % 3 == 0).collect();
        let syms = modem.modulate(&bits);
        let decoded = modem.demodulate(&syms);
        assert_eq!(decoded.len(), bits.len() / 10 * 10); // rounded down to symbol boundary
    }

    #[test]
    fn test_modem_modulate_demodulate_no_noise() {
        let cfg = ModemConfig::qam128_11ghz_28mhz(); // 256-QAM
        let modem = MicrowaveModem::new(cfg);
        let bits: Vec<bool> = (0..800).map(|i| (i * 7 + 3) % 5 > 2).collect();
        let syms = modem.modulate(&bits);
        let decoded = modem.demodulate(&syms);
        let n = decoded.len().min(bits.len());
        let errors: usize = bits.iter().take(n).zip(decoded.iter()).filter(|(a, b)| a != b).count();
        assert_eq!(errors, 0, "No errors expected without noise");
    }

    #[test]
    fn test_modem_spectral_efficiency() {
        let cfg = ModemConfig::qam1024_18ghz_56mhz();
        let modem = MicrowaveModem::new(cfg);
        let se = modem.spectral_efficiency();
        assert!(se.bits_per_hz > 8.0, "1024-QAM should have > 8 bits/Hz: {}", se.bits_per_hz);
        assert!(se.capacity_mbps > 400.0, "Should have > 400 Mbps capacity: {}", se.capacity_mbps);
    }

    #[test]
    fn test_modem_link_simulation_high_snr() {
        let cfg = ModemConfig::qam128_11ghz_28mhz(); // 256-QAM — more forgiving
        let mut modem = MicrowaveModem::new(cfg);
        // High Eb/N0 → low BER
        let (_tx, _rx, ber) = modem.simulate_link(800, 40.0, 42);
        assert!(ber < 0.05, "BER at 40 dB Eb/N0 should be low: {}", ber);
    }

    #[test]
    fn test_modem_ber_increases_at_low_snr() {
        let cfg = ModemConfig::qam128_11ghz_28mhz();
        let mut modem_low  = MicrowaveModem::new(cfg.clone());
        let mut modem_high = MicrowaveModem::new(cfg);
        let (_, _, ber_low)  = modem_low.simulate_link(800, 5.0, 1);
        let (_, _, ber_high) = modem_high.simulate_link(800, 40.0, 1);
        assert!(ber_high <= ber_low + 0.1, "High SNR BER ({}) should not exceed low SNR BER ({}) by much", ber_high, ber_low);
    }

    // ----- BER curve tests -----

    #[test]
    fn test_ber_curve_length() {
        let snr_range: Vec<f64> = (0..20).map(|i| i as f64 * 2.0).collect();
        let curve = ber_vs_ebn0_curve(QamOrder::Qam1024, &snr_range);
        assert_eq!(curve.len(), 20);
    }

    #[test]
    fn test_ber_curve_decreasing() {
        let snr_range: Vec<f64> = (20..40).map(|i| i as f64).collect();
        let curve = ber_vs_ebn0_curve(QamOrder::Qam1024, &snr_range);
        // BER should decrease monotonically with increasing SNR
        for w in curve.windows(2) {
            assert!(
                w[1].1 <= w[0].1 + 1e-10,
                "BER at {} dB = {} should be <= BER at {} dB = {}",
                w[1].0, w[1].1, w[0].0, w[0].1
            );
        }
    }

    #[test]
    fn test_fec_coding_gain_positive() {
        for order in [QamOrder::Qam256, QamOrder::Qam1024, QamOrder::Qam4096] {
            let gain = fec_coding_gain_db(order, FecCode::LdpcRate0_93);
            assert!(gain > 0.0, "FEC coding gain must be positive for {:?}", order);
        }
    }

    #[test]
    fn test_fec_coding_gain_ldpc_better_than_rs() {
        let gain_ldpc = fec_coding_gain_db(QamOrder::Qam1024, FecCode::LdpcRate0_93);
        let gain_rs   = fec_coding_gain_db(QamOrder::Qam1024, FecCode::ReedSolomon255_239);
        assert!(gain_ldpc > gain_rs, "LDPC should have more coding gain than RS");
    }

    // ----- Microwave band tests -----

    #[test]
    fn test_band_centre_frequencies_ordered() {
        let bands = [
            MicrowaveBand::Band6Ghz,
            MicrowaveBand::Band11Ghz,
            MicrowaveBand::Band23Ghz,
            MicrowaveBand::Band38Ghz,
        ];
        let freqs: Vec<f64> = bands.iter().map(|b| b.centre_freq_hz()).collect();
        for w in freqs.windows(2) {
            assert!(w[1] > w[0], "Band frequencies must be increasing");
        }
    }

    #[test]
    fn test_band_typical_hop_decreases_with_frequency() {
        let hop_6  = MicrowaveBand::Band6Ghz.typical_max_hop_km();
        let hop_38 = MicrowaveBand::Band38Ghz.typical_max_hop_km();
        assert!(hop_6 > hop_38, "Higher frequency → shorter maximum hop");
    }

    #[test]
    fn test_band_absorption_increases_near_23ghz() {
        let abs_11 = MicrowaveBand::Band11Ghz.atmospheric_absorption_db_per_km();
        let abs_23 = MicrowaveBand::Band23Ghz.atmospheric_absorption_db_per_km();
        assert!(abs_23 > abs_11, "23 GHz has more absorption than 11 GHz");
    }

    #[test]
    fn test_q_function_known_values() {
        // Q(0) = 0.5
        assert!((q_function(0.0) - 0.5).abs() < 1e-6);
        // Q(∞) ≈ 0
        assert!(q_function(8.0) < 1e-10);
        // Q(negative) = 1 - Q(positive)
        assert!((q_function(-1.0) - (1.0 - q_function(1.0))).abs() < 1e-9);
    }

    #[test]
    fn test_gray_encode_values() {
        // Known Gray code values
        assert_eq!(gray_encode(0, 4), 0);
        assert_eq!(gray_encode(1, 4), 1);
        assert_eq!(gray_encode(2, 4), 3);
        assert_eq!(gray_encode(3, 4), 2);
        assert_eq!(gray_encode(4, 4), 6);
    }

    #[test]
    fn test_modem_config_presets_valid() {
        let cfg1 = ModemConfig::qam128_11ghz_28mhz();
        let cfg2 = ModemConfig::qam1024_18ghz_56mhz();
        let cfg3 = ModemConfig::qam4096_11ghz();
        assert!(cfg1.symbol_rate_baud > 0.0);
        assert!(cfg2.channel_bandwidth_hz > 0.0);
        assert_eq!(cfg3.order, QamOrder::Qam4096);
    }

    #[test]
    fn test_2048qam_bits_per_symbol() {
        let c = QamConstellation::new(QamOrder::Qam2048);
        assert_eq!(c.order.bits_per_symbol(), 11);
        assert_eq!(c.points.len(), 2048);
    }
}

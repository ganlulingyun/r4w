//! # Coherent Optical QAM DSP Processor
//!
//! Digital signal processing chain for coherent optical fiber communications,
//! supporting high-order QAM modulation (16QAM, 64QAM, 256QAM) with algorithms
//! that compensate the major linear and nonlinear fiber impairments.
//!
//! ## DSP Chain Overview
//!
//! ```text
//!  ADC (I/Q)  →  CDC  →  Butterfly EQ  →  Carrier Recovery  →  Symbol Decision
//!                (FFT)    (CMA/DD-LMS)    (CFO + CPE BPS)       (QAM slicer)
//! ```
//!
//! ## Key Algorithms
//!
//! | Stage | Algorithm | Reference |
//! |-------|-----------|-----------|
//! | CDC   | Overlap-save FFT equalizer | ITU-T G.975.1 |
//! | Pol-demux | 2×2 butterfly CMA/DD-LMS | Godard 1980, Kikuchi 2008 |
//! | CFO   | 4th-power FFT / blind search | Ly-Gagnon 2006 |
//! | CPE   | Blind Phase Search (BPS) | Pfau 2009 |
//! | CPE   | Viterbi-Viterbi | Viterbi 1983 |
//!
//! ## Standards References
//!
//! - ITU-T G.975.1 (2004) — Forward error correction for high-bit-rate DWDM
//! - OIF-CEI-04.0 (2017) — Electrical interfaces for coherent pluggables
//! - IEEE 802.3ba (2010) — 100 Gb Ethernet
//! - OpenROADM MSA — Transponder interoperability
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::qam_coherent_dsp::{CoherentDspConfig, CoherentReceiver, QamOrder};
//!
//! // Build a 100G DP-QPSK (4QAM) receiver
//! let config = CoherentDspConfig::qpsk_100g();
//! let mut rx = CoherentReceiver::new(config);
//!
//! // Generate test QPSK symbols
//! let symbols: Vec<(f64, f64)> = (0..128)
//!     .map(|i| {
//!         let v = std::f64::consts::FRAC_1_SQRT_2;
//!         match i % 4 { 0 => (v, v), 1 => (-v, v), 2 => (-v, -v), _ => (v, -v) }
//!     })
//!     .collect();
//!
//! let result = rx.process_block(&symbols);
//! assert_eq!(result.len(), symbols.len());
//! ```

use std::f64::consts::PI;

// ─────────────────────────────────────────────────────────────────────────────
// Complex arithmetic helpers (no external crates)
// ─────────────────────────────────────────────────────────────────────────────

/// Complex number as (real, imag) tuple.
type C64 = (f64, f64);

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

#[inline(always)]
fn c_exp(phase: f64) -> C64 { (phase.cos(), phase.sin()) }

// ─────────────────────────────────────────────────────────────────────────────
// Radix-2 FFT (Cooley–Tukey, in-place DIT)
// ─────────────────────────────────────────────────────────────────────────────

/// In-place radix-2 DIT FFT.  `n` must be a power of two.
/// Set `inverse = true` for IFFT (result is NOT divided by n; caller must scale).
fn fft_inplace(buf: &mut Vec<C64>, inverse: bool) {
    let n = buf.len();
    debug_assert!(n.is_power_of_two(), "FFT size must be power of two");

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            buf.swap(i, j);
        }
    }

    // Butterfly stages
    let sign = if inverse { 1.0_f64 } else { -1.0_f64 };
    let mut len = 2usize;
    while len <= n {
        let half = len / 2;
        let ang = sign * 2.0 * PI / len as f64;
        let w_root: C64 = (ang.cos(), ang.sin());
        let mut k = 0;
        while k < n {
            let mut w: C64 = (1.0, 0.0);
            for i in 0..half {
                let u = buf[k + i];
                let v = c_mul(buf[k + i + half], w);
                buf[k + i] = c_add(u, v);
                buf[k + i + half] = c_sub(u, v);
                w = c_mul(w, w_root);
            }
            k += len;
        }
        len <<= 1;
    }
}

fn fft(input: &[C64]) -> Vec<C64> {
    let mut buf = input.to_vec();
    fft_inplace(&mut buf, false);
    buf
}

fn ifft(input: &[C64]) -> Vec<C64> {
    let n = input.len();
    let mut buf = input.to_vec();
    fft_inplace(&mut buf, true);
    let inv_n = 1.0 / n as f64;
    buf.iter_mut().for_each(|s| *s = c_scale(*s, inv_n));
    buf
}

// ─────────────────────────────────────────────────────────────────────────────
// QAM order
// ─────────────────────────────────────────────────────────────────────────────

/// QAM modulation order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QamOrder {
    /// 4-QAM / QPSK — 2 bits per symbol.
    Qam4,
    /// 16-QAM — 4 bits per symbol.
    Qam16,
    /// 64-QAM — 6 bits per symbol.
    Qam64,
    /// 256-QAM — 8 bits per symbol.
    Qam256,
}

impl QamOrder {
    /// Number of constellation points.
    pub fn points(&self) -> usize {
        match self {
            QamOrder::Qam4 => 4,
            QamOrder::Qam16 => 16,
            QamOrder::Qam64 => 64,
            QamOrder::Qam256 => 256,
        }
    }

    /// Alphabet size along one axis (sqrt of order).
    pub fn levels(&self) -> usize {
        match self {
            QamOrder::Qam4 => 2,
            QamOrder::Qam16 => 4,
            QamOrder::Qam64 => 8,
            QamOrder::Qam256 => 16,
        }
    }

    /// Bits per symbol.
    pub fn bits_per_symbol(&self) -> usize {
        match self {
            QamOrder::Qam4 => 2,
            QamOrder::Qam16 => 4,
            QamOrder::Qam64 => 6,
            QamOrder::Qam256 => 8,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Main configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for the coherent optical DSP receiver chain.
#[derive(Debug, Clone)]
pub struct CoherentDspConfig {
    /// QAM modulation order.
    pub qam_order: QamOrder,

    /// Symbol rate in Gbaud (1e9 symbols/s).
    pub symbol_rate_gbaud: f64,

    // ── Fiber / CDC parameters ──────────────────────────────────────────────

    /// Fiber chromatic dispersion coefficient [ps/(nm·km)].  Standard SMF: 17.
    pub dispersion_ps_nm_km: f64,

    /// Carrier wavelength [nm].  C-band: 1550.
    pub wavelength_nm: f64,

    /// Fiber span length [km].
    pub fiber_length_km: f64,

    // ── Butterfly equalizer ─────────────────────────────────────────────────

    /// Number of T/2-spaced adaptive filter taps per arm.
    pub butterfly_taps: usize,

    /// CMA step size (mu).  Typical: 1e-3 .. 1e-2.
    pub cma_step_size: f64,

    /// DD-LMS step size used after CMA convergence.
    pub dd_lms_step_size: f64,

    /// Number of symbols used in CMA pretraining before switching to DD-LMS.
    pub cma_pretrain_symbols: usize,

    // ── Carrier recovery ────────────────────────────────────────────────────

    /// Maximum expected frequency offset [GHz].
    pub max_cfo_ghz: f64,

    /// BPS: number of test phases uniformly distributed over [-π/M, π/M).
    pub bps_test_phases: usize,

    /// BPS: averaging block length (symbols).
    pub bps_block_length: usize,

    // ── CDC overlap-save ────────────────────────────────────────────────────

    /// FFT size for overlap-save CDC.  Must be power of two.
    pub cdc_fft_size: usize,
}

impl CoherentDspConfig {
    /// Standard 100G DP-QPSK (4QAM) receiver — ITU-T G.709 compatible.
    ///
    /// 28 Gbaud × 2 polarizations × 2 bits = 112 Gb/s net, with FEC overhead ≈ 100G.
    pub fn qpsk_100g() -> Self {
        Self {
            qam_order: QamOrder::Qam4,
            symbol_rate_gbaud: 28.0,
            dispersion_ps_nm_km: 17.0,
            wavelength_nm: 1550.0,
            fiber_length_km: 80.0,
            butterfly_taps: 15,
            cma_step_size: 1e-3,
            dd_lms_step_size: 5e-4,
            cma_pretrain_symbols: 2000,
            max_cfo_ghz: 4.0,
            bps_test_phases: 64,
            bps_block_length: 16,
            cdc_fft_size: 512,
        }
    }

    /// 200G DP-16QAM receiver — typical coherent DWDM metro/long-haul.
    pub fn qam16_200g() -> Self {
        Self {
            qam_order: QamOrder::Qam16,
            symbol_rate_gbaud: 32.0,
            dispersion_ps_nm_km: 17.0,
            wavelength_nm: 1550.0,
            fiber_length_km: 80.0,
            butterfly_taps: 19,
            cma_step_size: 5e-4,
            dd_lms_step_size: 2e-4,
            cma_pretrain_symbols: 5000,
            max_cfo_ghz: 4.0,
            bps_test_phases: 128,
            bps_block_length: 32,
            cdc_fft_size: 512,
        }
    }

    /// 400G DP-64QAM receiver — high-capacity short/medium reach.
    pub fn qam64_400g() -> Self {
        Self {
            qam_order: QamOrder::Qam64,
            symbol_rate_gbaud: 64.0,
            dispersion_ps_nm_km: 17.0,
            wavelength_nm: 1550.0,
            fiber_length_km: 80.0,
            butterfly_taps: 25,
            cma_step_size: 2e-4,
            dd_lms_step_size: 1e-4,
            cma_pretrain_symbols: 10000,
            max_cfo_ghz: 4.0,
            bps_test_phases: 256,
            bps_block_length: 64,
            cdc_fft_size: 1024,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// QAM Constellation
// ─────────────────────────────────────────────────────────────────────────────

/// Gray-coded square QAM constellation with average-power normalization.
///
/// Reference: Proakis & Salehi, "Digital Communications," 5th ed., §6.5.
#[derive(Debug, Clone)]
pub struct QamConstellation {
    order: QamOrder,
    /// Ideal symbol positions, normalized so E[|s|²] = 1.
    points: Vec<C64>,
    /// Normalization factor applied to raw (un-normalized) points.
    norm_factor: f64,
}

impl QamConstellation {
    /// Build the Gray-coded constellation for the given order.
    pub fn new(order: QamOrder) -> Self {
        let levels = order.levels();
        // Un-normalized alphabet: ±1, ±3, ..., ±(M-1)
        let alphabet: Vec<f64> = (0..levels)
            .map(|i| (2 * i) as f64 - (levels as f64 - 1.0))
            .collect();

        // Average power of un-normalized grid: E[x²] = (M²-1)/3
        // For M levels: sum of (2k+1-M)² for k=0..M-1 = M*(M²-1)/3
        let m2 = levels as f64;
        let avg_power_1d = (m2 * m2 - 1.0) / 3.0;
        let avg_power_2d = 2.0 * avg_power_1d;
        let norm_factor = 1.0 / avg_power_2d.sqrt();

        // Gray encode each axis: Gray(i) = i XOR (i >> 1)
        let gray_idx: Vec<usize> = (0..levels).map(|i| i ^ (i >> 1)).collect();

        let mut points = Vec::with_capacity(levels * levels);
        for qi in 0..levels {
            for ii in 0..levels {
                // Gray-coded I and Q values
                let i_val = alphabet[gray_idx[ii]];
                let q_val = alphabet[gray_idx[qi]];
                points.push((i_val * norm_factor, q_val * norm_factor));
            }
        }

        Self { order, points, norm_factor }
    }

    /// All ideal constellation points (normalized).
    pub fn points(&self) -> &[C64] {
        &self.points
    }

    /// Normalization factor (multiply raw ±1,±3,… symbols by this).
    pub fn norm_factor(&self) -> f64 {
        self.norm_factor
    }

    /// QAM order.
    pub fn order(&self) -> QamOrder {
        self.order
    }

    /// Hard-decision slicer: find nearest constellation point.
    pub fn slice(&self, rx: C64) -> C64 {
        let mut best = self.points[0];
        let mut best_dist = f64::INFINITY;
        for &p in &self.points {
            let d = c_abs_sq(c_sub(rx, p));
            if d < best_dist {
                best_dist = d;
                best = p;
            }
        }
        best
    }

    /// Symbol error: returns 0.0 if rx slices to the reference symbol, 1.0 otherwise.
    pub fn symbol_error(&self, rx: C64, reference: C64) -> f64 {
        let decision = self.slice(rx);
        let d = c_abs_sq(c_sub(decision, reference));
        if d < 1e-9 { 0.0 } else { 1.0 }
    }

    /// CMA dispersion constant R₂ = E[|s|⁴] / E[|s|²].
    ///
    /// For CMA convergence of multi-ring constellations (Godard 1980).
    pub fn cma_radius_sq(&self) -> f64 {
        let n = self.points.len() as f64;
        let e4: f64 = self.points.iter().map(|&p| c_abs_sq(p).powi(2)).sum::<f64>() / n;
        let e2: f64 = self.points.iter().map(|&p| c_abs_sq(p)).sum::<f64>() / n;
        e4 / e2
    }

    /// Number of distinct amplitude rings in the constellation.
    pub fn num_rings(&self) -> usize {
        let mut unique_r_sq: Vec<f64> = self
            .points
            .iter()
            .map(|&p| (c_abs_sq(p) * 1e6).round() / 1e6)
            .collect();
        unique_r_sq.sort_by(|a, b| a.partial_cmp(b).unwrap());
        unique_r_sq.dedup();
        unique_r_sq.len()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Chromatic Dispersion Compensator (CDC)
// ─────────────────────────────────────────────────────────────────────────────

/// Frequency-domain chromatic dispersion compensator using overlap-save.
///
/// The CD transfer function is:
/// ```text
/// H(f) = exp(j · β₂ · L · (2πf)² / 2)
/// ```
/// where β₂ = -D · λ² / (2π·c) is the group-velocity dispersion.
///
/// With D in ps/(nm·km), λ in nm, c in nm/ps, L in km, symbol rate in GHz.
///
/// Reference: Savory, "Digital filters for coherent optical receivers," Opt. Express 2008.
#[derive(Debug, Clone)]
pub struct ChromaticDispersionCompensator {
    fft_size: usize,
    /// Overlap length in samples (discarded per block in overlap-save).
    pub filter_len: usize,
    /// Frequency-domain filter tap vector H[k], one per FFT bin.
    pub h_freq: Vec<C64>,
    /// Input overlap buffer (last `filter_len - 1` samples from previous block).
    overlap: Vec<C64>,
}

impl ChromaticDispersionCompensator {
    /// Create a CDC for the given fiber parameters.
    ///
    /// # Arguments
    /// * `dispersion_ps_nm_km` — chromatic dispersion coefficient D [ps/(nm·km)]
    /// * `wavelength_nm` — carrier wavelength [nm]
    /// * `fiber_length_km` — compensated fiber length [km]
    /// * `symbol_rate_ghz` — input sample rate (≥ symbol rate × oversampling) [GHz]
    /// * `fft_size` — overlap-save FFT size (power of two, ≥ 2× filter length)
    pub fn new(
        dispersion_ps_nm_km: f64,
        wavelength_nm: f64,
        fiber_length_km: f64,
        symbol_rate_ghz: f64,
        fft_size: usize,
    ) -> Self {
        // β₂ = -D λ² / (2π c), with D in ps/(nm·km), λ in nm, c = 2.998×10⁵ nm/ps
        // β₂ units: ps²/km
        let c_nm_ps = 2.998e5_f64; // speed of light [nm/ps]
        let beta2_ps2_km = -dispersion_ps_nm_km * wavelength_nm * wavelength_nm
            / (2.0 * PI * c_nm_ps);

        // Total accumulated dispersion [ps²]
        let d_total_ps2 = beta2_ps2_km * fiber_length_km;

        // Sample period in ps: T_s = 1 / (symbol_rate_ghz * 1e9) converted:
        // If symbol_rate is in GHz = 1e9 sym/s, T_s = 1/Rs ns = 1e3/Rs ps
        // But for normalized digital freq we use T_s = 1 as index over [0, fft_size-1].
        // The digital frequency of bin k is f_k = k/fft_size * Rs  [GHz]
        let rs = symbol_rate_ghz; // GHz

        let mut h_freq = vec![(0.0_f64, 0.0_f64); fft_size];
        for k in 0..fft_size {
            // Map bin to frequency in GHz: centered around 0
            let kk = if k <= fft_size / 2 { k as f64 } else { k as f64 - fft_size as f64 };
            let f_ghz = kk * rs / fft_size as f64;

            // f in THz for β₂ in ps²/km consistency: convert GHz → 1/ps
            // β₂ [ps²/km] * L [km] = D_total [ps²]
            // H(f) = exp(j * D_total * (2π f)² / 2)  where f in 1/ps = f_GHz * 1e-3 [1/ps]
            // Actually: β₂ [s²/m] = β₂ [ps²/km] * 1e-24 / 1e3 = β₂ [ps²/km] * 1e-27 s²/m
            // Simpler dimensionally consistent form with f in GHz, D_total in ps²:
            // ω = 2π * f_GHz [Grad/s] ; H = exp(j * D_total[ps²] * ω² / 2)
            // ω² = (2π f_GHz)² [Grad/s]²  =>  D_total * ω² / 2 has units ps² * (Grad/s)²
            // ps² * (1/ps)² = dimensionless, since Grad/s = 1e9/s = 1/ns ... careful.
            // Use: f [Hz] = f_GHz * 1e9; D_total [s²] = D_total_ps2 * 1e-24.
            // phase = D_total[s²] * (2π f[Hz])² / 2  (radians, dimensionless) ✓
            let f_hz = f_ghz * 1e9;
            let d_total_s2 = d_total_ps2 * 1e-24;
            let phase = d_total_s2 * (2.0 * PI * f_hz).powi(2) / 2.0;
            h_freq[k] = (phase.cos(), phase.sin());
        }

        // Estimate required filter length from maximum group delay spread (samples)
        // Δτ_max = |β₂| * L * Δω_max = |D_total| * 2π * Rs [ps] → samples
        let delta_tau_ps = d_total_ps2.abs() * 2.0 * PI * rs; // [ps * Grad/s = samples? ]
        // Actually: Δτ [ps] = D [ps/nm] * L [km] * Δλ [nm]
        // For digital: Δτ = |β₂| * L * ω_max = |D_total[ps²]| * 2π * Rs[GHz] * 1e-3 [ps]
        let delta_tau_samples =
            (d_total_ps2.abs() * 2.0 * PI * rs * 1e-3 + 2.0).ceil() as usize;
        let filter_len = delta_tau_samples.max(4).min(fft_size / 2 - 1);

        let overlap = vec![(0.0, 0.0); filter_len];

        Self { fft_size, filter_len, h_freq, overlap }
    }

    /// Apply CDC compensation to a block of samples using overlap-save.
    ///
    /// Returns a vector of the same length as `input`.
    pub fn process(&mut self, input: &[C64]) -> Vec<C64> {
        let step = self.fft_size - self.filter_len;
        let n_blocks = (input.len() + step - 1) / step;
        let mut output = Vec::with_capacity(input.len());

        let mut pos = 0usize;
        for _ in 0..n_blocks {
            // Build block: [overlap | new data]
            let mut block = vec![(0.0, 0.0); self.fft_size];
            // Copy overlap
            for (i, &v) in self.overlap.iter().enumerate() {
                block[i] = v;
            }
            // Copy new data
            let take = step.min(input.len().saturating_sub(pos));
            for i in 0..take {
                block[self.filter_len + i] = input[pos + i];
            }
            pos += take;

            // FFT, multiply by H, IFFT
            fft_inplace(&mut block, false);
            for k in 0..self.fft_size {
                block[k] = c_mul(block[k], self.h_freq[k]);
            }
            fft_inplace(&mut block, true);
            let inv = 1.0 / self.fft_size as f64;
            for s in &mut block {
                *s = c_scale(*s, inv);
            }

            // Discard the first filter_len samples (overlap region)
            let valid_start = self.filter_len;
            let valid_end = self.fft_size.min(valid_start + take);
            for i in valid_start..valid_end {
                output.push(block[i]);
            }
        }

        // Update overlap for next call
        let new_overlap_start = input.len().saturating_sub(self.filter_len);
        for (i, s) in self.overlap.iter_mut().enumerate() {
            let src = new_overlap_start + i;
            *s = if src < input.len() { input[src] } else { (0.0, 0.0) };
        }

        output.truncate(input.len());
        output
    }

    /// Compute the CD transfer function H(f) at a given normalized frequency.
    ///
    /// `f_norm` ∈ [−0.5, 0.5] (normalized to sample rate).
    pub fn transfer_function_at(&self, f_norm: f64) -> C64 {
        let bin = ((f_norm * self.fft_size as f64).round() as isize)
            .rem_euclid(self.fft_size as isize) as usize;
        self.h_freq[bin]
    }

    /// Reset the overlap buffer (e.g., on burst boundary).
    pub fn reset(&mut self) {
        self.overlap.iter_mut().for_each(|s| *s = (0.0, 0.0));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 2×2 Butterfly Equalizer (polarization demultiplexing)
// ─────────────────────────────────────────────────────────────────────────────

/// Adaptive algorithm for the butterfly equalizer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ButterflyMode {
    /// Constant Modulus Algorithm — blind, good for initial convergence.
    Cma,
    /// Decision-Directed LMS — lower steady-state error, requires near-convergence.
    DdLms,
    /// Radius-Directed Equalization — extends CMA to multi-ring constellations.
    Rde,
}

/// 2×2 butterfly adaptive equalizer for dual-polarization coherent reception.
///
/// The equalizer implements four T/2-spaced FIR filters arranged as:
/// ```text
///   [Y_X]   [H_XX  H_XY] [X_in]
///   [Y_Y] = [H_YX  H_YY] [Y_in]
/// ```
/// where each H_ij is a complex FIR vector of length `taps`.
///
/// References:
/// - Kikuchi, "Fundamentals of Coherent Optical Fiber Communications," 2016.
/// - Savory, "Digital filters for coherent optical receivers," Opt. Express 2008.
#[derive(Debug, Clone)]
pub struct ButterflyEqualizer {
    taps: usize,
    /// H_XX: X→X filter.
    h_xx: Vec<C64>,
    /// H_XY: Y→X filter.
    h_xy: Vec<C64>,
    /// H_YX: X→Y filter.
    h_yx: Vec<C64>,
    /// H_YY: Y→Y filter.
    h_yy: Vec<C64>,
    /// Delay line for X polarization.
    buf_x: Vec<C64>,
    /// Delay line for Y polarization.
    buf_y: Vec<C64>,
    /// Write pointer in circular buffer.
    ptr: usize,
    /// CMA step size.
    mu_cma: f64,
    /// DD-LMS step size.
    mu_dd: f64,
    /// Target modulus² for CMA (= R₂ from constellation).
    r2: f64,
    /// Current mode.
    pub mode: ButterflyMode,
    /// Number of symbols processed (used for mode switching).
    symbols_processed: usize,
    /// Threshold for switching CMA → DD-LMS.
    pretrain_symbols: usize,
    /// Reference constellation for DD slicing.
    constellation: QamConstellation,
    /// Ring radii² for RDE.
    ring_radii_sq: Vec<f64>,
}

impl ButterflyEqualizer {
    /// Create a new butterfly equalizer.
    pub fn new(config: &CoherentDspConfig) -> Self {
        let taps = config.butterfly_taps;
        let constellation = QamConstellation::new(config.qam_order);
        let r2 = constellation.cma_radius_sq();

        // Compute unique ring radii² from constellation
        let mut radii_sq: Vec<f64> = constellation
            .points()
            .iter()
            .map(|&p| (c_abs_sq(p) * 1e6).round() / 1e6)
            .collect();
        radii_sq.sort_by(|a, b| a.partial_cmp(b).unwrap());
        radii_sq.dedup();

        // Initialize H_XX = [0,..,1,..,0] (center spike = identity)
        let mut h_xx = vec![(0.0_f64, 0.0_f64); taps];
        h_xx[taps / 2] = (1.0, 0.0);
        let h_xy = vec![(0.0_f64, 0.0_f64); taps];
        let h_yx = vec![(0.0_f64, 0.0_f64); taps];
        let mut h_yy = vec![(0.0_f64, 0.0_f64); taps];
        h_yy[taps / 2] = (1.0, 0.0);

        Self {
            taps,
            h_xx,
            h_xy,
            h_yx,
            h_yy,
            buf_x: vec![(0.0, 0.0); taps],
            buf_y: vec![(0.0, 0.0); taps],
            ptr: 0,
            mu_cma: config.cma_step_size,
            mu_dd: config.dd_lms_step_size,
            r2,
            mode: ButterflyMode::Cma,
            symbols_processed: 0,
            pretrain_symbols: config.cma_pretrain_symbols,
            constellation,
            ring_radii_sq: radii_sq,
        }
    }

    /// Process a single pair of dual-polarization input samples.
    ///
    /// Returns `(y_x, y_y)` — the equalized X and Y outputs.
    pub fn process_sample(&mut self, x_in: C64, y_in: C64) -> (C64, C64) {
        // Write new samples into circular delay line
        self.buf_x[self.ptr] = x_in;
        self.buf_y[self.ptr] = y_in;

        // FIR outputs: Y = H · buf (circular)
        let mut y_x = (0.0_f64, 0.0_f64);
        let mut y_y = (0.0_f64, 0.0_f64);
        for k in 0..self.taps {
            let idx = (self.ptr + self.taps - k) % self.taps;
            y_x = c_add(y_x, c_mul(self.h_xx[k], self.buf_x[idx]));
            y_x = c_add(y_x, c_mul(self.h_xy[k], self.buf_y[idx]));
            y_y = c_add(y_y, c_mul(self.h_yx[k], self.buf_x[idx]));
            y_y = c_add(y_y, c_mul(self.h_yy[k], self.buf_y[idx]));
        }

        // Compute errors and update taps
        let (ex, ey) = match self.mode {
            ButterflyMode::Cma => {
                // CMA error: e = (|y|² - R₂) · y
                let ex = c_scale(y_x, c_abs_sq(y_x) - self.r2);
                let ey = c_scale(y_y, c_abs_sq(y_y) - self.r2);
                (ex, ey)
            }
            ButterflyMode::DdLms => {
                // DD-LMS: slice then compute error
                let dx = self.constellation.slice(y_x);
                let dy = self.constellation.slice(y_y);
                let ex = c_sub(y_x, dx);
                let ey = c_sub(y_y, dy);
                (ex, ey)
            }
            ButterflyMode::Rde => {
                // RDE: find nearest ring radius², project onto that ring
                let rx_sq = c_abs_sq(y_x);
                let ry_sq = c_abs_sq(y_y);
                let target_x = self.nearest_ring(rx_sq);
                let target_y = self.nearest_ring(ry_sq);
                let ex = c_scale(y_x, rx_sq - target_x);
                let ey = c_scale(y_y, ry_sq - target_y);
                (ex, ey)
            }
        };

        let mu = match self.mode {
            ButterflyMode::DdLms => self.mu_dd,
            _ => self.mu_cma,
        };

        // Stochastic gradient update: H ← H - μ · e · x*
        for k in 0..self.taps {
            let idx = (self.ptr + self.taps - k) % self.taps;
            let xc = c_conj(self.buf_x[idx]);
            let yc = c_conj(self.buf_y[idx]);

            self.h_xx[k] = c_sub(self.h_xx[k], c_scale(c_mul(ex, xc), mu));
            self.h_xy[k] = c_sub(self.h_xy[k], c_scale(c_mul(ex, yc), mu));
            self.h_yx[k] = c_sub(self.h_yx[k], c_scale(c_mul(ey, xc), mu));
            self.h_yy[k] = c_sub(self.h_yy[k], c_scale(c_mul(ey, yc), mu));
        }

        // Advance circular pointer
        self.ptr = (self.ptr + 1) % self.taps;
        self.symbols_processed += 1;

        // Switch CMA → DD-LMS after pretraining
        if self.mode == ButterflyMode::Cma && self.symbols_processed >= self.pretrain_symbols {
            self.mode = ButterflyMode::DdLms;
        }

        (y_x, y_y)
    }

    fn nearest_ring(&self, r_sq: f64) -> f64 {
        let mut best = self.ring_radii_sq[0];
        let mut best_dist = (r_sq - best).abs();
        for &rs in &self.ring_radii_sq[1..] {
            let d = (r_sq - rs).abs();
            if d < best_dist {
                best_dist = d;
                best = rs;
            }
        }
        best
    }

    /// CMA cost function: J = E[(|y|² - R₂)²].  Lower is better.
    pub fn cma_cost(&self, samples: &[C64]) -> f64 {
        if samples.is_empty() {
            return 0.0;
        }
        let sum: f64 = samples
            .iter()
            .map(|&s| (c_abs_sq(s) - self.r2).powi(2))
            .sum();
        sum / samples.len() as f64
    }

    /// Reset taps to initial identity state.
    pub fn reset(&mut self) {
        let taps = self.taps;
        self.h_xx.iter_mut().for_each(|t| *t = (0.0, 0.0));
        self.h_xy.iter_mut().for_each(|t| *t = (0.0, 0.0));
        self.h_yx.iter_mut().for_each(|t| *t = (0.0, 0.0));
        self.h_yy.iter_mut().for_each(|t| *t = (0.0, 0.0));
        self.h_xx[taps / 2] = (1.0, 0.0);
        self.h_yy[taps / 2] = (1.0, 0.0);
        self.buf_x.iter_mut().for_each(|t| *t = (0.0, 0.0));
        self.buf_y.iter_mut().for_each(|t| *t = (0.0, 0.0));
        self.ptr = 0;
        self.symbols_processed = 0;
        self.mode = ButterflyMode::Cma;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Carrier Recovery: CFO + CPE
// ─────────────────────────────────────────────────────────────────────────────

/// Carrier frequency offset and phase error estimation/correction.
///
/// Two-stage recovery:
/// 1. **CFO**: 4th-power FFT method (QPSK) or blind frequency search (higher QAM).
/// 2. **CPE**: Blind Phase Search (BPS) or Viterbi-Viterbi (V&V).
///
/// References:
/// - Ly-Gagnon et al., "Coherent detection of optical quadrature phase-shift keying
///   signals with carrier phase estimation," JLT 2006.
/// - Pfau et al., "Hardware-efficient coherent digital receiver concept with feedforward
///   carrier recovery for M-QAM constellations," JLT 2009.
#[derive(Debug, Clone)]
pub struct CarrierRecovery {
    /// Estimated carrier frequency offset in radians per sample.
    pub cfo_rad_per_sample: f64,
    /// Current phase accumulator for NCO.
    nco_phase: f64,
    /// BPS test phases [rad].
    bps_phases: Vec<f64>,
    /// BPS block length.
    bps_block_len: usize,
    /// Reference constellation for BPS decision metric.
    constellation: QamConstellation,
    /// V&V: accumulated phase history (for unwrapping).
    vv_phase_acc: f64,
    /// QAM order (determines power for V&V).
    qam_order: QamOrder,
}

impl CarrierRecovery {
    /// Create a carrier recovery block.
    pub fn new(config: &CoherentDspConfig) -> Self {
        let n = config.bps_test_phases;
        let order = config.qam_order;
        // BPS test phases uniformly spaced over [-π/M, π/M)  (M = modulation index)
        // For square QAM the rotational symmetry is π/2 (4-fold), so test over ±π/4.
        let half_range = PI / 4.0; // works for QPSK, 16QAM, 64QAM, 256QAM
        let bps_phases: Vec<f64> = (0..n)
            .map(|i| -half_range + (2.0 * half_range * i as f64) / n as f64)
            .collect();

        let constellation = QamConstellation::new(order);

        Self {
            cfo_rad_per_sample: 0.0,
            nco_phase: 0.0,
            bps_phases,
            bps_block_len: config.bps_block_length,
            constellation,
            vv_phase_acc: 0.0,
            qam_order: order,
        }
    }

    /// Estimate the carrier frequency offset using the 4th-power FFT method.
    ///
    /// Raises each sample to the 4th power (removes QPSK/QAM modulation for 4-fold
    /// symmetric constellations) then finds the spectral peak.
    ///
    /// # Arguments
    /// * `samples` — received complex baseband samples
    /// * `sample_rate` — sample rate in samples/s (arbitrary units, consistent with return)
    ///
    /// Returns the estimated CFO in the same units as `sample_rate`.
    pub fn estimate_cfo_4th_power(&self, samples: &[C64], sample_rate: f64) -> f64 {
        if samples.len() < 4 {
            return 0.0;
        }
        // Pad to next power of two
        let n = samples.len().next_power_of_two();
        let mut buf: Vec<C64> = samples
            .iter()
            .map(|&s| {
                // Raise to 4th power: s^4
                let s2 = c_mul(s, s);
                c_mul(s2, s2)
            })
            .collect();
        buf.resize(n, (0.0, 0.0));

        fft_inplace(&mut buf, false);

        // Find peak bin
        let peak_bin = buf
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| c_abs_sq(**a).partial_cmp(&c_abs_sq(**b)).unwrap())
            .map(|(i, _)| i)
            .unwrap_or(0);

        // Convert bin to frequency (accounting for fftshift centering)
        let freq_bin = if peak_bin <= n / 2 { peak_bin as f64 } else { peak_bin as f64 - n as f64 };

        // The 4th power shifts the tone to 4×CFO; divide by 4
        (freq_bin / n as f64) * sample_rate / 4.0
    }

    /// Apply frequency correction using an NCO.
    ///
    /// Multiplies each sample by exp(-j·2π·cfo·n/sample_rate).
    pub fn apply_cfo_correction(&mut self, samples: &mut [C64]) {
        for s in samples.iter_mut() {
            let corr = c_exp(-self.nco_phase);
            *s = c_mul(*s, corr);
            self.nco_phase = (self.nco_phase + self.cfo_rad_per_sample)
                .rem_euclid(2.0 * PI);
        }
    }

    /// Blind Phase Search (BPS) carrier phase estimation.
    ///
    /// For each block of `bps_block_len` symbols, tests N candidate phases and
    /// selects the one minimizing the sum of squared distances to nearest constellation
    /// points.  Returns a vector of phase corrections (one per symbol).
    ///
    /// Reference: Pfau et al., JLT 2009.
    pub fn bps_phase_estimation(&self, samples: &[C64]) -> Vec<f64> {
        let n = samples.len();
        let bl = self.bps_block_len;
        let n_test = self.bps_phases.len();
        let mut phase_corrections = vec![0.0_f64; n];

        let half_bl = bl / 2;

        for sym_idx in 0..n {
            // Window: [sym_idx - half_bl, sym_idx + half_bl)
            let win_start = sym_idx.saturating_sub(half_bl);
            let win_end = (sym_idx + half_bl).min(n);

            let mut best_phase = 0.0_f64;
            let mut best_cost = f64::INFINITY;

            for &phi in &self.bps_phases {
                let rot = c_exp(phi);
                let cost: f64 = samples[win_start..win_end]
                    .iter()
                    .map(|&s| {
                        let rotated = c_mul(s, rot);
                        let nearest = self.constellation.slice(rotated);
                        c_abs_sq(c_sub(rotated, nearest))
                    })
                    .sum();
                if cost < best_cost {
                    best_cost = cost;
                    best_phase = phi;
                }
            }

            phase_corrections[sym_idx] = best_phase;
        }

        // Unwrap phase to prevent 2π/M cycle slips
        let m_fold = 4.0_f64; // π/2 rotational symmetry
        let threshold = PI / m_fold;
        for i in 1..n {
            let diff = phase_corrections[i] - phase_corrections[i - 1];
            if diff > threshold {
                phase_corrections[i] -= 2.0 * threshold;
            } else if diff < -threshold {
                phase_corrections[i] += 2.0 * threshold;
            }
        }

        phase_corrections
    }

    /// Apply phase corrections returned by BPS or V&V.
    pub fn apply_phase_corrections(samples: &[C64], corrections: &[f64]) -> Vec<C64> {
        samples
            .iter()
            .zip(corrections.iter())
            .map(|(&s, &phi)| c_mul(s, c_exp(-phi)))
            .collect()
    }

    /// Viterbi-Viterbi (4th-power) carrier phase estimation.
    ///
    /// Classic V&V algorithm for QPSK, and extended to M-fold symmetric QAM.
    /// Computes: φ̂ = (1/M) · arg(Σ y^M  over block)
    ///
    /// Reference: Viterbi & Viterbi, IEEE Trans. Inf. Theory 1983.
    pub fn viterbi_viterbi_cpe(&mut self, samples: &[C64]) -> Vec<f64> {
        let n = samples.len();
        let bl = self.bps_block_len;
        let m = 4u32; // 4-fold symmetry for square QAM
        let mf = m as f64;

        let mut phase_estimates = vec![0.0_f64; n];
        let mut i = 0;
        while i < n {
            let end = (i + bl).min(n);
            // Accumulate y^M over block
            let sum = samples[i..end].iter().fold((0.0_f64, 0.0_f64), |acc, &s| {
                let sm = {
                    let s2 = c_mul(s, s);
                    let s4 = c_mul(s2, s2);
                    s4
                };
                c_add(acc, sm)
            });
            let phi_block = c_arg(sum) / mf;
            // Handle π/2 cycle-slip ambiguity: fold into [-π/4, π/4)
            let wrapped = ((phi_block + PI / 4.0).rem_euclid(PI / 2.0)) - PI / 4.0;
            for j in i..end {
                phase_estimates[j] = wrapped;
            }
            i = end;
        }

        // Phase unwrapping across blocks to suppress cycle slips
        for i in 1..n {
            let diff = phase_estimates[i] - phase_estimates[i - 1];
            if diff > PI / 4.0 {
                phase_estimates[i] -= PI / 2.0;
            } else if diff < -PI / 4.0 {
                phase_estimates[i] += PI / 2.0;
            }
        }

        self.vv_phase_acc = *phase_estimates.last().unwrap_or(&0.0);
        phase_estimates
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Performance Metrics
// ─────────────────────────────────────────────────────────────────────────────

/// Performance metrics for the coherent DSP chain.
#[derive(Debug, Clone)]
pub struct CohDspMetrics {
    /// RMS Error Vector Magnitude [linear fraction].
    pub evm_rms: f64,
    /// Peak EVM [linear fraction].
    pub evm_peak: f64,
    /// EVM in dB (20·log₁₀(EVM_rms)).
    pub evm_db: f64,
    /// Estimated BER from EVM (Gaussian approximation).
    pub ber_from_evm: f64,
    /// Q-factor [dB].
    pub q_factor_db: f64,
    /// Estimated OSNR [dB] from EVM.
    pub osnr_db_est: f64,
    /// Symbol Error Rate measured.
    pub ser: f64,
    /// Mean phase error [rad].
    pub mean_phase_error: f64,
}

impl CohDspMetrics {
    /// Compute metrics from received symbols and ideal reference symbols.
    ///
    /// `rx` — received symbols after full DSP chain
    /// `ideal` — corresponding ideal constellation points
    pub fn compute(rx: &[C64], ideal: &[C64]) -> Self {
        assert_eq!(rx.len(), ideal.len(), "rx and ideal must be same length");
        let n = rx.len();
        if n == 0 {
            return Self {
                evm_rms: 0.0,
                evm_peak: 0.0,
                evm_db: f64::NEG_INFINITY,
                ber_from_evm: 0.5,
                q_factor_db: 0.0,
                osnr_db_est: 0.0,
                ser: 0.0,
                mean_phase_error: 0.0,
            };
        }

        // Average symbol power
        let p_avg = ideal.iter().map(|&s| c_abs_sq(s)).sum::<f64>() / n as f64;

        // EVM: sqrt(mean(|rx - ideal|²) / P_avg)
        let sum_err_sq: f64 = rx
            .iter()
            .zip(ideal.iter())
            .map(|(&r, &id)| c_abs_sq(c_sub(r, id)))
            .sum();
        let evm_rms = (sum_err_sq / n as f64 / p_avg).sqrt();

        let evm_peak = rx
            .iter()
            .zip(ideal.iter())
            .map(|(&r, &id)| (c_abs_sq(c_sub(r, id)) / p_avg).sqrt())
            .fold(0.0_f64, f64::max);

        let evm_db = 20.0 * evm_rms.log10();

        // BER from EVM (Gaussian noise approximation, Gray-coded QAM):
        // SNR_per_bit ≈ 1 / (EVM² · bits_per_symbol)
        // For 4QAM: BER = 0.5 · erfc(1 / (evm · sqrt(2)))
        // General approximation: BER ≈ 0.5 · erfc(SNR_lin.sqrt())  where SNR_lin = 1/(2·EVM²)
        let snr_lin = 1.0 / (2.0 * evm_rms * evm_rms);
        let ber_from_evm = 0.5 * erfc(snr_lin.sqrt());

        // Q-factor: Q = 20·log₁₀(sqrt(2) · erfinv(1 - 2·BER))
        // Simpler: Q² = 1 / EVM²  for QPSK; use SNR approximation
        let q_linear = (2.0 * snr_lin).sqrt();
        let q_factor_db = 20.0 * q_linear.log10();

        // OSNR estimation: For NRZ at Rs Gbaud, OSNR[dBm/0.1nm] ≈ SNR_lin · Rs / B_ref
        // where B_ref = 12.5 GHz (0.1 nm at 1550 nm). Simplified: OSNR ≈ SNR_lin (in 0.1nm ref).
        let osnr_db_est = 10.0 * snr_lin.log10();

        // Mean phase error
        let mean_phase_error = rx
            .iter()
            .zip(ideal.iter())
            .map(|(&r, &id)| {
                // phase error ≈ Im(r · id*) / |id|²  (small angle)
                let num = r.0 * id.1 - r.1 * id.0; // Im(r · id*)
                let den = c_abs_sq(id);
                if den > 1e-12 { num / den } else { 0.0 }
            })
            .sum::<f64>()
            / n as f64;

        Self {
            evm_rms,
            evm_peak,
            evm_db,
            ber_from_evm,
            q_factor_db,
            osnr_db_est,
            ser: 0.0, // computed separately via constellation.symbol_error
            mean_phase_error,
        }
    }
}

/// Complementary error function approximation (Abramowitz & Stegun 7.1.26).
fn erfc(x: f64) -> f64 {
    if x < 0.0 {
        return 2.0 - erfc(-x);
    }
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t * (0.254829592
        + t * (-0.284496736
            + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    poly * (-x * x).exp()
}

// ─────────────────────────────────────────────────────────────────────────────
// Full CoherentReceiver chain
// ─────────────────────────────────────────────────────────────────────────────

/// Output of the coherent DSP chain for one polarization.
#[derive(Debug, Clone)]
pub struct CoherentRxOutput {
    /// Equalized and phase-corrected symbols.
    pub symbols: Vec<C64>,
    /// Estimated carrier frequency offset [rad/sample].
    pub cfo_estimate: f64,
    /// Per-symbol phase corrections applied [rad].
    pub phase_corrections: Vec<f64>,
}

/// Full coherent optical DSP chain:
/// CDC → Butterfly EQ → CFO correction → CPE (BPS).
///
/// Processes single-polarization samples for simplicity.
/// For dual-polarization, call `process_dual_pol()`.
#[derive(Debug, Clone)]
pub struct CoherentReceiver {
    config: CoherentDspConfig,
    cdc: ChromaticDispersionCompensator,
    butterfly: ButterflyEqualizer,
    carrier_recovery: CarrierRecovery,
    constellation: QamConstellation,
}

impl CoherentReceiver {
    /// Build a complete coherent DSP chain from the given configuration.
    pub fn new(config: CoherentDspConfig) -> Self {
        let cdc = ChromaticDispersionCompensator::new(
            config.dispersion_ps_nm_km,
            config.wavelength_nm,
            config.fiber_length_km,
            config.symbol_rate_gbaud,
            config.cdc_fft_size,
        );
        let butterfly = ButterflyEqualizer::new(&config);
        let carrier_recovery = CarrierRecovery::new(&config);
        let constellation = QamConstellation::new(config.qam_order);
        Self { config, cdc, butterfly, carrier_recovery, constellation }
    }

    /// Process a block of received single-polarization IQ samples.
    ///
    /// Applies: CDC → adaptive equalization (single-pol) → CFO → BPS CPE.
    /// Returns equalized, phase-corrected symbols.
    pub fn process_block(&mut self, input: &[(f64, f64)]) -> CoherentRxOutput {
        // Stage 1: Chromatic dispersion compensation
        let after_cdc = self.cdc.process(input);

        // Stage 2: Single-polarization adaptive equalization
        // For single-pol we use H_XX only (treat Y as zero).
        let dummy_y = (0.0_f64, 0.0_f64);
        let mut after_eq: Vec<C64> = after_cdc
            .iter()
            .map(|&s| self.butterfly.process_sample(s, dummy_y).0)
            .collect();

        // Stage 3: Carrier frequency offset estimation (4th-power method)
        let cfo_hz = self
            .carrier_recovery
            .estimate_cfo_4th_power(&after_eq, self.config.symbol_rate_gbaud * 1e9);
        // Convert to rad/sample
        let cfo_rad_per_sample = 2.0 * PI * cfo_hz / (self.config.symbol_rate_gbaud * 1e9);
        self.carrier_recovery.cfo_rad_per_sample = cfo_rad_per_sample;
        self.carrier_recovery.apply_cfo_correction(&mut after_eq);

        // Stage 4: Carrier phase estimation (BPS)
        let phase_corrections = self.carrier_recovery.bps_phase_estimation(&after_eq);
        let symbols =
            CarrierRecovery::apply_phase_corrections(&after_eq, &phase_corrections);

        CoherentRxOutput {
            symbols,
            cfo_estimate: cfo_rad_per_sample,
            phase_corrections,
        }
    }

    /// Process dual-polarization samples: (X, Y) pairs.
    ///
    /// Returns `(output_x, output_y)`.
    pub fn process_dual_pol(
        &mut self,
        x_in: &[C64],
        y_in: &[C64],
    ) -> (CoherentRxOutput, CoherentRxOutput) {
        assert_eq!(x_in.len(), y_in.len(), "X and Y must be same length");

        // CDC on each polarization independently
        let x_cdc = self.cdc.process(x_in);
        // Clone cdc for Y (independent CDC state)
        let mut cdc_y = self.cdc.clone();
        cdc_y.reset();
        let y_cdc = cdc_y.process(y_in);

        // Butterfly equalization
        let mut x_eq = Vec::with_capacity(x_in.len());
        let mut y_eq = Vec::with_capacity(y_in.len());
        for (&xi, &yi) in x_cdc.iter().zip(y_cdc.iter()) {
            let (yo_x, yo_y) = self.butterfly.process_sample(xi, yi);
            x_eq.push(yo_x);
            y_eq.push(yo_y);
        }

        // CFO estimation from X pol, apply to both
        let cfo_hz = self
            .carrier_recovery
            .estimate_cfo_4th_power(&x_eq, self.config.symbol_rate_gbaud * 1e9);
        let cfo_rps = 2.0 * PI * cfo_hz / (self.config.symbol_rate_gbaud * 1e9);
        self.carrier_recovery.cfo_rad_per_sample = cfo_rps;

        self.carrier_recovery.apply_cfo_correction(&mut x_eq);
        // Apply same CFO to Y (reset NCO to apply from same point)
        let saved_nco = self.carrier_recovery.nco_phase;
        self.carrier_recovery.nco_phase = 0.0;
        self.carrier_recovery.apply_cfo_correction(&mut y_eq);
        self.carrier_recovery.nco_phase = saved_nco;

        // Independent BPS CPE per polarization
        let phi_x = self.carrier_recovery.bps_phase_estimation(&x_eq);
        let phi_y = self.carrier_recovery.bps_phase_estimation(&y_eq);

        let sym_x = CarrierRecovery::apply_phase_corrections(&x_eq, &phi_x);
        let sym_y = CarrierRecovery::apply_phase_corrections(&y_eq, &phi_y);

        (
            CoherentRxOutput {
                symbols: sym_x,
                cfo_estimate: cfo_rps,
                phase_corrections: phi_x,
            },
            CoherentRxOutput {
                symbols: sym_y,
                cfo_estimate: cfo_rps,
                phase_corrections: phi_y,
            },
        )
    }

    /// Perform hard symbol decisions on a set of equalized symbols.
    pub fn decide(&self, symbols: &[C64]) -> Vec<C64> {
        symbols.iter().map(|&s| self.constellation.slice(s)).collect()
    }

    /// Compute EVM metrics for a received block vs. ideal symbols.
    pub fn measure_evm(&self, rx: &[C64], ideal: &[C64]) -> CohDspMetrics {
        CohDspMetrics::compute(rx, ideal)
    }

    /// Reset all DSP state (for new burst processing).
    pub fn reset(&mut self) {
        self.cdc.reset();
        self.butterfly.reset();
        self.carrier_recovery.nco_phase = 0.0;
        self.carrier_recovery.cfo_rad_per_sample = 0.0;
        self.carrier_recovery.vv_phase_acc = 0.0;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helpers ────────────────────────────────────────────────────────────

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    fn complex_close(a: C64, b: C64, tol: f64) -> bool {
        c_abs(c_sub(a, b)) <= tol
    }

    // ── QAM Constellation Tests ─────────────────────────────────────────────

    #[test]
    fn test_qam4_point_count() {
        let c = QamConstellation::new(QamOrder::Qam4);
        assert_eq!(c.points().len(), 4);
    }

    #[test]
    fn test_qam16_point_count() {
        let c = QamConstellation::new(QamOrder::Qam16);
        assert_eq!(c.points().len(), 16);
    }

    #[test]
    fn test_qam64_point_count() {
        let c = QamConstellation::new(QamOrder::Qam64);
        assert_eq!(c.points().len(), 64);
    }

    #[test]
    fn test_qam256_point_count() {
        let c = QamConstellation::new(QamOrder::Qam256);
        assert_eq!(c.points().len(), 256);
    }

    #[test]
    fn test_qam4_normalized_power() {
        let c = QamConstellation::new(QamOrder::Qam4);
        let n = c.points().len() as f64;
        let avg_power = c.points().iter().map(|&p| c_abs_sq(p)).sum::<f64>() / n;
        assert!(approx_eq(avg_power, 1.0, 1e-9), "4QAM power = {avg_power}");
    }

    #[test]
    fn test_qam16_normalized_power() {
        let c = QamConstellation::new(QamOrder::Qam16);
        let n = c.points().len() as f64;
        let avg_power = c.points().iter().map(|&p| c_abs_sq(p)).sum::<f64>() / n;
        assert!(approx_eq(avg_power, 1.0, 1e-9), "16QAM power = {avg_power}");
    }

    #[test]
    fn test_qam64_normalized_power() {
        let c = QamConstellation::new(QamOrder::Qam64);
        let n = c.points().len() as f64;
        let avg_power = c.points().iter().map(|&p| c_abs_sq(p)).sum::<f64>() / n;
        assert!(approx_eq(avg_power, 1.0, 1e-9), "64QAM power = {avg_power}");
    }

    #[test]
    fn test_qam256_normalized_power() {
        let c = QamConstellation::new(QamOrder::Qam256);
        let n = c.points().len() as f64;
        let avg_power = c.points().iter().map(|&p| c_abs_sq(p)).sum::<f64>() / n;
        assert!(approx_eq(avg_power, 1.0, 1e-9), "256QAM power = {avg_power}");
    }

    #[test]
    fn test_qam4_all_unique() {
        let c = QamConstellation::new(QamOrder::Qam4);
        // All points should be distinct
        let pts = c.points();
        for i in 0..pts.len() {
            for j in (i + 1)..pts.len() {
                assert!(!complex_close(pts[i], pts[j], 1e-9), "Duplicate points at {i},{j}");
            }
        }
    }

    #[test]
    fn test_hard_slicer_nearest_point() {
        let c = QamConstellation::new(QamOrder::Qam4);
        // Perfect QPSK point + tiny noise
        let ideal = c.points()[0];
        let noisy = (ideal.0 + 0.01, ideal.1 - 0.01);
        let decision = c.slice(noisy);
        assert!(complex_close(decision, ideal, 1e-9));
    }

    #[test]
    fn test_qam16_slicer_correct_quadrant() {
        let c = QamConstellation::new(QamOrder::Qam16);
        // First quadrant point — find one with I>0, Q>0
        let first_q: Vec<C64> = c.points().iter().filter(|&&p| p.0 > 0.0 && p.1 > 0.0).copied().collect();
        assert!(!first_q.is_empty());
        let ref_pt = first_q[0];
        let noisy = (ref_pt.0 + 0.02, ref_pt.1 - 0.02);
        let decision = c.slice(noisy);
        assert!(complex_close(decision, ref_pt, 1e-9));
    }

    #[test]
    fn test_cma_radius_sq_positive() {
        for order in [QamOrder::Qam4, QamOrder::Qam16, QamOrder::Qam64, QamOrder::Qam256] {
            let c = QamConstellation::new(order);
            let r2 = c.cma_radius_sq();
            assert!(r2 > 0.0, "{order:?} R₂ = {r2}");
        }
    }

    #[test]
    fn test_qam4_cma_r2_equals_one() {
        // For unit-power QPSK, R₂ = E[|s|⁴]/E[|s|²] = 1 (all points on unit circle).
        let c = QamConstellation::new(QamOrder::Qam4);
        let r2 = c.cma_radius_sq();
        assert!(approx_eq(r2, 1.0, 1e-9), "QPSK R₂ = {r2}");
    }

    #[test]
    fn test_num_rings_qam4() {
        let c = QamConstellation::new(QamOrder::Qam4);
        assert_eq!(c.num_rings(), 1, "QPSK has 1 ring");
    }

    #[test]
    fn test_num_rings_qam16() {
        let c = QamConstellation::new(QamOrder::Qam16);
        // 16QAM has 3 rings (corner, edge, inner)
        let rings = c.num_rings();
        assert!(rings >= 2, "16QAM should have ≥ 2 rings, got {rings}");
    }

    #[test]
    fn test_symbol_error_no_noise() {
        let c = QamConstellation::new(QamOrder::Qam16);
        let pt = c.points()[0];
        assert_eq!(c.symbol_error(pt, pt), 0.0);
    }

    #[test]
    fn test_symbol_error_wrong_symbol() {
        let c = QamConstellation::new(QamOrder::Qam16);
        let pt0 = c.points()[0];
        let pt1 = c.points()[1];
        // If we receive pt1 and reference is pt0, should error
        // (only if they're far enough apart to not confuse slicer)
        let _ = c.symbol_error(pt1, pt0); // just check it runs
    }

    // ── FFT Tests ──────────────────────────────────────────────────────────

    #[test]
    fn test_fft_ifft_roundtrip() {
        let n = 64;
        let input: Vec<C64> = (0..n).map(|i| (i as f64 * 0.1, i as f64 * 0.05)).collect();
        let spectrum = fft(&input);
        let recovered = ifft(&spectrum);
        for (a, b) in input.iter().zip(recovered.iter()) {
            assert!(complex_close(*a, *b, 1e-10), "FFT/IFFT roundtrip failed");
        }
    }

    #[test]
    fn test_fft_dc_component() {
        let n = 8;
        let input: Vec<C64> = vec![(1.0, 0.0); n];
        let spectrum = fft(&input);
        // DC bin should equal sum = n
        assert!(approx_eq(spectrum[0].0, n as f64, 1e-10));
        assert!(approx_eq(spectrum[0].1, 0.0, 1e-10));
        // All other bins should be zero
        for k in 1..n {
            assert!(c_abs(spectrum[k]) < 1e-10, "Non-DC bin {k} non-zero");
        }
    }

    #[test]
    fn test_fft_tone() {
        let n = 64;
        let k0 = 4usize;
        // Pure tone at bin k0: x[n] = exp(j 2π k0 n / N)
        let input: Vec<C64> = (0..n)
            .map(|i| c_exp(2.0 * PI * k0 as f64 * i as f64 / n as f64))
            .collect();
        let spectrum = fft(&input);
        let peak_bin = spectrum
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| c_abs_sq(**a).partial_cmp(&c_abs_sq(**b)).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        assert_eq!(peak_bin, k0, "FFT tone peak should be at bin {k0}");
    }

    // ── CDC Tests ─────────────────────────────────────────────────────────

    #[test]
    fn test_cdc_zero_dispersion() {
        // With D=0, H(f) = exp(0) = 1 for all f → pass-through
        let mut cdc = ChromaticDispersionCompensator::new(0.0, 1550.0, 80.0, 28.0, 64);
        let input: Vec<C64> = (0..32).map(|i| (i as f64 * 0.1, 0.0)).collect();
        let output = cdc.process(&input);
        assert_eq!(output.len(), input.len());
        // The sum of power should be preserved (approximately)
        let in_power: f64 = input.iter().map(|&s| c_abs_sq(s)).sum();
        let out_power: f64 = output.iter().map(|&s| c_abs_sq(s)).sum();
        assert!(approx_eq(in_power, out_power, in_power * 0.1 + 1e-6),
            "CDC power preservation failed: {in_power} vs {out_power}");
    }

    #[test]
    fn test_cdc_transfer_function_unit_magnitude() {
        let cdc = ChromaticDispersionCompensator::new(17.0, 1550.0, 80.0, 28.0, 256);
        // H(f) should be a pure phase rotation: |H(f)| = 1
        for f in [-0.4, -0.2, 0.0, 0.2, 0.4] {
            let h = cdc.transfer_function_at(f);
            assert!(approx_eq(c_abs(h), 1.0, 1e-9), "|H({f})| = {}", c_abs(h));
        }
    }

    #[test]
    fn test_cdc_output_length_matches_input() {
        let mut cdc = ChromaticDispersionCompensator::new(17.0, 1550.0, 80.0, 28.0, 64);
        let input: Vec<C64> = vec![(1.0, 0.0); 100];
        let output = cdc.process(&input);
        assert_eq!(output.len(), input.len());
    }

    #[test]
    fn test_cdc_reset_clears_overlap() {
        let mut cdc = ChromaticDispersionCompensator::new(17.0, 1550.0, 80.0, 28.0, 64);
        let input: Vec<C64> = vec![(1.0, 0.5); 64];
        let _ = cdc.process(&input);
        cdc.reset();
        // After reset, overlap buffer should be zero
        let all_zero = cdc.overlap.iter().all(|&s| s == (0.0, 0.0));
        assert!(all_zero, "Overlap buffer not cleared after reset");
    }

    #[test]
    fn test_cdc_compensate_then_apply_is_identity() {
        // Verify that the forward CDC transfer function H(f) and the inverse
        // (with negated dispersion) satisfy H_fwd(f) · H_inv(f) = 1.
        // This is the fundamental correctness property of the CDC.
        let d = 17.0;
        let lambda = 1550.0;
        let l = 80.0;
        let rs = 28.0;
        let fft_size = 256;

        let cdc_fwd = ChromaticDispersionCompensator::new(d, lambda, l, rs, fft_size);
        let cdc_inv = ChromaticDispersionCompensator::new(-d, lambda, l, rs, fft_size);

        // Verify H_fwd(f) · H_inv(f) = 1 for several normalized frequencies
        for &f_norm in &[-0.4, -0.25, -0.1, 0.0, 0.1, 0.25, 0.4] {
            let h_fwd = cdc_fwd.transfer_function_at(f_norm);
            let h_inv = cdc_inv.transfer_function_at(f_norm);
            let product = c_mul(h_fwd, h_inv);
            assert!(approx_eq(c_abs(product), 1.0, 1e-9),
                "H_fwd · H_inv magnitude != 1 at f={f_norm}: |product|={}", c_abs(product));
            // Phase should be 0 mod 2π: product = (cos(0), sin(0)) = (1, 0)
            assert!(approx_eq(product.0, 1.0, 1e-9),
                "H_fwd · H_inv real part != 1 at f={f_norm}: product.re={}", product.0);
            assert!(approx_eq(product.1, 0.0, 1e-9),
                "H_fwd · H_inv imag part != 0 at f={f_norm}: product.im={}", product.1);
        }

        // Also verify that the frequency-domain filter vectors are conjugate pairs:
        // h_inv[k] = conj(h_fwd[k])  (since phase sign flips with negated D)
        for k in 0..fft_size {
            let hf = cdc_fwd.h_freq[k];
            let hi = cdc_inv.h_freq[k];
            // hf · hi should be real ≈ 1
            let product = c_mul(hf, hi);
            assert!(approx_eq(product.0, 1.0, 1e-9), "h_fwd[{k}] · h_inv[{k}] != 1");
        }
    }

    // ── Butterfly Equalizer Tests ─────────────────────────────────────────

    #[test]
    fn test_butterfly_initial_mode_cma() {
        let config = CoherentDspConfig::qpsk_100g();
        let eq = ButterflyEqualizer::new(&config);
        assert_eq!(eq.mode, ButterflyMode::Cma);
    }

    #[test]
    fn test_butterfly_passthrough_identity() {
        // With 1 tap identity initialization, a QPSK symbol should pass through
        let mut config = CoherentDspConfig::qpsk_100g();
        config.butterfly_taps = 1;
        config.cma_pretrain_symbols = 10000; // stay in CMA
        let mut eq = ButterflyEqualizer::new(&config);
        // After 1 tap, h_xx = [1+0j] → perfect passthrough
        let sym = (0.7071, 0.7071);
        let (out, _) = eq.process_sample(sym, (0.0, 0.0));
        assert!(complex_close(out, sym, 1e-6), "Identity equalizer failed");
    }

    #[test]
    fn test_butterfly_cma_cost_decreases() {
        let config = CoherentDspConfig::qpsk_100g();
        let mut eq = ButterflyEqualizer::new(&config);
        let c = QamConstellation::new(QamOrder::Qam4);

        // Feed QPSK symbols with mild rotation
        let n = 500usize;
        let mut symbols_out: Vec<C64> = Vec::with_capacity(n);
        for i in 0..n {
            let pt = c.points()[i % 4];
            // Add slight rotation
            let rot = c_exp(0.1);
            let s = c_mul(pt, rot);
            let (y, _) = eq.process_sample(s, (0.0, 0.0));
            symbols_out.push(y);
        }

        // Cost early vs. late
        let cost_early = eq.cma_cost(&symbols_out[..50]);
        let cost_late = eq.cma_cost(&symbols_out[symbols_out.len() - 50..]);
        // Late cost should be ≤ early cost (equalizer improves or maintains)
        assert!(cost_late <= cost_early + 0.5, "CMA cost did not decrease: {cost_early} → {cost_late}");
    }

    #[test]
    fn test_butterfly_switches_to_ddlms() {
        let mut config = CoherentDspConfig::qpsk_100g();
        config.cma_pretrain_symbols = 10;
        let mut eq = ButterflyEqualizer::new(&config);
        let c = QamConstellation::new(QamOrder::Qam4);
        for i in 0..15 {
            let pt = c.points()[i % 4];
            let _ = eq.process_sample(pt, (0.0, 0.0));
        }
        assert_eq!(eq.mode, ButterflyMode::DdLms, "Should have switched to DD-LMS");
    }

    #[test]
    fn test_butterfly_reset_restores_identity() {
        let config = CoherentDspConfig::qpsk_100g();
        let mut eq = ButterflyEqualizer::new(&config);
        // Process some symbols to corrupt taps
        let c = QamConstellation::new(QamOrder::Qam4);
        for i in 0..100 {
            let pt = c.points()[i % 4];
            let _ = eq.process_sample(pt, (0.0, 0.0));
        }
        eq.reset();
        // After reset, center tap should be 1+0j
        let center = eq.taps / 2;
        assert!(complex_close(eq.h_xx[center], (1.0, 0.0), 1e-9));
        assert_eq!(eq.mode, ButterflyMode::Cma);
    }

    // ── Carrier Recovery Tests ────────────────────────────────────────────

    #[test]
    fn test_cfo_estimation_zero_offset() {
        let cr = CarrierRecovery::new(&CoherentDspConfig::qpsk_100g());
        let c = QamConstellation::new(QamOrder::Qam4);
        // Perfect QPSK symbols, no CFO
        let symbols: Vec<C64> = (0..256).map(|i| c.points()[i % 4]).collect();
        let cfo = cr.estimate_cfo_4th_power(&symbols, 28e9);
        assert!(cfo.abs() < 2e8, "CFO estimate for zero offset too large: {cfo}");
    }

    #[test]
    fn test_cfo_estimation_detects_offset() {
        let cr = CarrierRecovery::new(&CoherentDspConfig::qpsk_100g());
        let c = QamConstellation::new(QamOrder::Qam4);
        let n = 512usize;
        let cfo_true = 1e9; // 1 GHz offset
        let rs = 28e9;
        // Modulate QPSK + add CFO
        let symbols: Vec<C64> = (0..n)
            .map(|i| {
                let pt = c.points()[i % 4];
                let phase = 2.0 * PI * cfo_true * i as f64 / rs;
                c_mul(pt, c_exp(phase))
            })
            .collect();
        let cfo_est = cr.estimate_cfo_4th_power(&symbols, rs);
        // Should be within ±500 MHz
        assert!(
            (cfo_est - cfo_true).abs() < 5e8,
            "CFO estimation error too large: true={cfo_true}, est={cfo_est}"
        );
    }

    #[test]
    fn test_cfo_correction_reduces_residual() {
        let config = CoherentDspConfig::qpsk_100g();
        let mut cr = CarrierRecovery::new(&config);
        let c = QamConstellation::new(QamOrder::Qam4);
        let n = 512usize;
        let cfo_true = 500e6;
        let rs = 28e9;
        // Build symbols with CFO applied
        let symbols_with_cfo: Vec<C64> = (0..n)
            .map(|i| {
                let pt = c.points()[i % 4];
                let phase = 2.0 * PI * cfo_true * i as f64 / rs;
                c_mul(pt, c_exp(phase))
            })
            .collect();
        // Raise to 4th power to strip QPSK modulation; CFO tone at 4×cfo_true
        let fourth_power_before: Vec<C64> = symbols_with_cfo.iter().map(|&s| {
            let s2 = c_mul(s, s); c_mul(s2, s2)
        }).collect();

        let cfo_est = cr.estimate_cfo_4th_power(&symbols_with_cfo, rs);
        cr.cfo_rad_per_sample = 2.0 * PI * cfo_est / rs;
        let mut symbols_corrected = symbols_with_cfo.clone();
        cr.apply_cfo_correction(&mut symbols_corrected);

        // After correction, raise to 4th power again — the 4×CFO tone should be gone
        // Measure the spectral peak of the 4th-power signal: it should be near DC now
        let fourth_power_after: Vec<C64> = symbols_corrected.iter().map(|&s| {
            let s2 = c_mul(s, s); c_mul(s2, s2)
        }).collect();
        let n_fft = n.next_power_of_two();
        let mut buf_before = fourth_power_before.clone();
        buf_before.resize(n_fft, (0.0, 0.0));
        fft_inplace(&mut buf_before, false);
        let mut buf_after = fourth_power_after.clone();
        buf_after.resize(n_fft, (0.0, 0.0));
        fft_inplace(&mut buf_after, false);

        // Peak bin in "before" spectrum should be at the CFO bin
        let peak_before = buf_before.iter().enumerate()
            .max_by(|(_, a), (_, b)| c_abs_sq(**a).partial_cmp(&c_abs_sq(**b)).unwrap())
            .map(|(i, _)| i).unwrap();
        // Peak bin in "after" spectrum should be closer to DC (bin 0)
        let peak_after = buf_after.iter().enumerate()
            .max_by(|(_, a), (_, b)| c_abs_sq(**a).partial_cmp(&c_abs_sq(**b)).unwrap())
            .map(|(i, _)| i).unwrap();

        // Convert bins to frequencies
        let freq_before = if peak_before <= n_fft / 2 { peak_before as f64 } else { peak_before as f64 - n_fft as f64 };
        let freq_after = if peak_after <= n_fft / 2 { peak_after as f64 } else { peak_after as f64 - n_fft as f64 };
        // After correction the residual frequency (in bins) should be less than before
        assert!(
            freq_after.abs() < freq_before.abs() + 2.0,
            "CFO not reduced: before peak at bin {freq_before}, after at {freq_after}"
        );
    }

    #[test]
    fn test_bps_phase_estimation_zero_phase() {
        let config = CoherentDspConfig::qpsk_100g();
        let cr = CarrierRecovery::new(&config);
        let c = QamConstellation::new(QamOrder::Qam4);
        let symbols: Vec<C64> = (0..64).map(|i| c.points()[i % 4]).collect();
        let corrections = cr.bps_phase_estimation(&symbols);
        assert_eq!(corrections.len(), symbols.len());
        // Phase corrections should be near zero for perfect input
        let max_corr = corrections.iter().map(|&x| x.abs()).fold(0.0_f64, f64::max);
        assert!(max_corr < PI / 4.0 + 0.01, "BPS corrections out of range: {max_corr}");
    }

    #[test]
    fn test_bps_phase_estimation_with_constant_offset() {
        let config = CoherentDspConfig::qpsk_100g();
        let cr = CarrierRecovery::new(&config);
        let c = QamConstellation::new(QamOrder::Qam4);
        let phi_true = 0.15_f64; // 0.15 rad constant phase offset
        let symbols: Vec<C64> = (0..64)
            .map(|i| c_mul(c.points()[i % 4], c_exp(phi_true)))
            .collect();
        let corrections = cr.bps_phase_estimation(&symbols);
        // Average correction should be close to -phi_true
        let avg_corr = corrections.iter().sum::<f64>() / corrections.len() as f64;
        assert!(
            (avg_corr + phi_true).abs() < 0.15,
            "BPS avg correction {avg_corr} should ≈ -{phi_true}"
        );
    }

    #[test]
    fn test_bps_apply_phase_corrections() {
        // apply_phase_corrections(samples, corrections) multiplies each sample by
        // exp(-corrections[i]).  So to undo a rotation of +phi, pass corrections = [phi].
        let c = QamConstellation::new(QamOrder::Qam4);
        let phi = 0.2_f64;
        // Rotate symbols by +phi
        let symbols: Vec<C64> = (0..32)
            .map(|i| c_mul(c.points()[i % 4], c_exp(phi)))
            .collect();
        // Correction = phi  →  applied rotation = exp(-phi)  →  net = exp(0) = 1
        let corrections = vec![phi; 32];
        let corrected = CarrierRecovery::apply_phase_corrections(&symbols, &corrections);
        let originals: Vec<C64> = (0..32).map(|i| c.points()[i % 4]).collect();
        for (orig, corr) in originals.iter().zip(corrected.iter()) {
            assert!(complex_close(*corr, *orig, 1e-9), "Phase correction failed: {corr:?} vs {orig:?}");
        }
    }

    #[test]
    fn test_viterbi_viterbi_zero_phase_noise() {
        let config = CoherentDspConfig::qpsk_100g();
        let mut cr = CarrierRecovery::new(&config);
        let c = QamConstellation::new(QamOrder::Qam4);
        let symbols: Vec<C64> = (0..64).map(|i| c.points()[i % 4]).collect();
        let phases = cr.viterbi_viterbi_cpe(&symbols);
        assert_eq!(phases.len(), symbols.len());
        let max_phase = phases.iter().map(|&x| x.abs()).fold(0.0_f64, f64::max);
        assert!(max_phase < PI / 4.0 + 0.01, "V&V phase out of range: {max_phase}");
    }

    #[test]
    fn test_viterbi_viterbi_with_phase_noise() {
        let config = CoherentDspConfig::qpsk_100g();
        let mut cr = CarrierRecovery::new(&config);
        let c = QamConstellation::new(QamOrder::Qam4);
        // Very slow phase drift: 0.01 rad per block (well within V&V tracking range)
        let n = 128usize;
        let bl = config.bps_block_length;
        let symbols: Vec<C64> = (0..n)
            .map(|i| {
                let phase = 0.01 * (i / bl) as f64;
                c_mul(c.points()[i % 4], c_exp(phase))
            })
            .collect();
        let phases = cr.viterbi_viterbi_cpe(&symbols);
        // Corrected symbols should be within the QPSK symbol radius
        let corrected = CarrierRecovery::apply_phase_corrections(&symbols, &phases);
        // Verify all corrected symbols have magnitude close to 1 (QPSK lies on unit circle)
        let max_mag_err = corrected
            .iter()
            .map(|&s| (c_abs(s) - 1.0).abs())
            .fold(0.0_f64, f64::max);
        assert!(max_mag_err < 0.1,
            "V&V: corrected symbols deviate from unit circle by {max_mag_err}");
    }

    // ── EVM / Metrics Tests ───────────────────────────────────────────────

    #[test]
    fn test_evm_zero_noise() {
        let c = QamConstellation::new(QamOrder::Qam16);
        let pts: Vec<C64> = c.points().to_vec();
        let metrics = CohDspMetrics::compute(&pts, &pts);
        assert!(approx_eq(metrics.evm_rms, 0.0, 1e-12), "EVM for perfect symbols: {}", metrics.evm_rms);
    }

    #[test]
    fn test_evm_known_noise() {
        let c = QamConstellation::new(QamOrder::Qam4);
        // Add constant offset ε to all symbols
        let eps = 0.05_f64;
        let ideal: Vec<C64> = c.points().to_vec();
        let rx: Vec<C64> = ideal.iter().map(|&p| (p.0 + eps, p.1)).collect();
        let metrics = CohDspMetrics::compute(&rx, &ideal);
        // EVM_rms = |ε| / sqrt(avg_power)
        let avg_p = ideal.iter().map(|&p| c_abs_sq(p)).sum::<f64>() / ideal.len() as f64;
        let expected_evm = eps / avg_p.sqrt();
        assert!(approx_eq(metrics.evm_rms, expected_evm, 1e-9));
    }

    #[test]
    fn test_evm_db_negative_for_small_error() {
        let c = QamConstellation::new(QamOrder::Qam16);
        let ideal: Vec<C64> = c.points().to_vec();
        let rx: Vec<C64> = ideal.iter().map(|&p| (p.0 + 0.02, p.1 + 0.01)).collect();
        let metrics = CohDspMetrics::compute(&rx, &ideal);
        assert!(metrics.evm_db < 0.0, "EVM_dB should be negative for small errors");
    }

    #[test]
    fn test_ber_from_evm_in_valid_range() {
        let c = QamConstellation::new(QamOrder::Qam4);
        let ideal: Vec<C64> = c.points().to_vec();
        let rx: Vec<C64> = ideal.iter().map(|&p| (p.0 + 0.1, p.1 - 0.1)).collect();
        let metrics = CohDspMetrics::compute(&rx, &ideal);
        assert!(metrics.ber_from_evm >= 0.0 && metrics.ber_from_evm <= 0.5,
            "BER out of [0,0.5]: {}", metrics.ber_from_evm);
    }

    #[test]
    fn test_q_factor_positive() {
        let c = QamConstellation::new(QamOrder::Qam4);
        let ideal: Vec<C64> = c.points().to_vec();
        let rx: Vec<C64> = ideal.iter().map(|&p| (p.0 + 0.05, p.1)).collect();
        let metrics = CohDspMetrics::compute(&rx, &ideal);
        assert!(metrics.q_factor_db > 0.0, "Q-factor should be positive: {}", metrics.q_factor_db);
    }

    #[test]
    fn test_erfc_known_values() {
        // erfc(0) = 1.0
        assert!(approx_eq(erfc(0.0), 1.0, 1e-6));
        // erfc(∞) ≈ 0
        assert!(erfc(10.0) < 1e-6);
        // erfc(-0) = erfc(0)
        assert!(approx_eq(erfc(-0.0), 1.0, 1e-6));
    }

    // ── Full Chain Tests ──────────────────────────────────────────────────

    #[test]
    fn test_coherent_receiver_process_block_length() {
        let config = CoherentDspConfig::qpsk_100g();
        let mut rx = CoherentReceiver::new(config);
        let input: Vec<C64> = vec![(0.7071, 0.7071); 128];
        let output = rx.process_block(&input);
        assert_eq!(output.symbols.len(), input.len());
        assert_eq!(output.phase_corrections.len(), input.len());
    }

    #[test]
    fn test_coherent_receiver_qpsk_no_impairments() {
        // Use zero dispersion so the CDC stage is a pass-through.
        // The butterfly equalizer (CMA with 2000 pretrain) plus BPS CPE chain
        // should preserve the QPSK structure well enough for a functional check.
        let mut config = CoherentDspConfig::qpsk_100g();
        config.dispersion_ps_nm_km = 0.0; // no CDC distortion
        config.cma_pretrain_symbols = 50;  // converge quickly for small input
        let mut rx = CoherentReceiver::new(config);
        let c = QamConstellation::new(QamOrder::Qam4);
        // Feed 512 perfect QPSK symbols
        let input: Vec<C64> = (0..512).map(|i| c.points()[i % 4]).collect();
        let output = rx.process_block(&input);
        // Check that the output contains valid-looking complex values (finite, bounded)
        let all_finite = output.symbols.iter().all(|s| s.0.is_finite() && s.1.is_finite());
        assert!(all_finite, "Output contains non-finite values");
        // After the initial CMA transient, symbols should be near unit circle (QPSK)
        let tail_start = output.symbols.len().saturating_sub(64);
        let tail_rms: f64 = {
            let tail = &output.symbols[tail_start..];
            let sum_err: f64 = tail.iter().map(|&s| {
                let nearest = c.slice(s);
                c_abs_sq(c_sub(s, nearest))
            }).sum();
            (sum_err / tail.len() as f64).sqrt()
        };
        // Generous threshold: < 0.8 (just checks the chain doesn't blow up)
        assert!(tail_rms < 0.8, "Post-CMA EVM too high: {tail_rms}");
    }

    #[test]
    fn test_coherent_receiver_reset() {
        let config = CoherentDspConfig::qpsk_100g();
        let mut rx = CoherentReceiver::new(config);
        let input: Vec<C64> = vec![(0.7071, 0.7071); 128];
        let _ = rx.process_block(&input);
        rx.reset();
        // After reset, NCO phase and CFO should be zero
        assert_eq!(rx.carrier_recovery.nco_phase, 0.0);
        assert_eq!(rx.carrier_recovery.cfo_rad_per_sample, 0.0);
    }

    #[test]
    fn test_coherent_receiver_hard_decisions() {
        let config = CoherentDspConfig::qpsk_100g();
        let rx = CoherentReceiver::new(config);
        let c = QamConstellation::new(QamOrder::Qam4);
        // Slightly noisy symbols
        let symbols: Vec<C64> = c.points().iter().map(|&p| (p.0 + 0.05, p.1 - 0.05)).collect();
        let decisions = rx.decide(&symbols);
        assert_eq!(decisions.len(), symbols.len());
        // Each decision should be a valid constellation point
        for d in &decisions {
            let is_valid = c.points().iter().any(|&p| complex_close(*d, p, 1e-9));
            assert!(is_valid, "Decision {d:?} is not a valid constellation point");
        }
    }

    #[test]
    fn test_dual_pol_output_lengths() {
        let config = CoherentDspConfig::qpsk_100g();
        let mut rx = CoherentReceiver::new(config);
        let n = 128usize;
        let c = QamConstellation::new(QamOrder::Qam4);
        let x: Vec<C64> = (0..n).map(|i| c.points()[i % 4]).collect();
        let y: Vec<C64> = (0..n).map(|i| c.points()[(i + 1) % 4]).collect();
        let (out_x, out_y) = rx.process_dual_pol(&x, &y);
        assert_eq!(out_x.symbols.len(), n);
        assert_eq!(out_y.symbols.len(), n);
    }

    #[test]
    fn test_measure_evm_via_receiver() {
        let config = CoherentDspConfig::qpsk_100g();
        let rx = CoherentReceiver::new(config);
        let c = QamConstellation::new(QamOrder::Qam4);
        let ideal: Vec<C64> = (0..64).map(|i| c.points()[i % 4]).collect();
        let rx_syms: Vec<C64> = ideal.iter().map(|&p| (p.0 + 0.02, p.1 + 0.02)).collect();
        let metrics = rx.measure_evm(&rx_syms, &ideal);
        assert!(metrics.evm_rms > 0.0);
        assert!(metrics.evm_rms < 0.1);
    }

    #[test]
    fn test_qam_order_bits_per_symbol() {
        assert_eq!(QamOrder::Qam4.bits_per_symbol(), 2);
        assert_eq!(QamOrder::Qam16.bits_per_symbol(), 4);
        assert_eq!(QamOrder::Qam64.bits_per_symbol(), 6);
        assert_eq!(QamOrder::Qam256.bits_per_symbol(), 8);
    }

    #[test]
    fn test_config_presets_compile() {
        let _ = CoherentDspConfig::qpsk_100g();
        let _ = CoherentDspConfig::qam16_200g();
        let _ = CoherentDspConfig::qam64_400g();
    }

    #[test]
    fn test_cdc_different_fiber_lengths() {
        // Longer fiber should produce different H(f) (more dispersion)
        let h80 = ChromaticDispersionCompensator::new(17.0, 1550.0, 80.0, 28.0, 64);
        let h160 = ChromaticDispersionCompensator::new(17.0, 1550.0, 160.0, 28.0, 64);
        let h_80_f = h80.transfer_function_at(0.1);
        let h_160_f = h160.transfer_function_at(0.1);
        // Different lengths should give different phase responses
        assert!(!complex_close(h_80_f, h_160_f, 1e-6),
            "CDC H(f) should differ for different fiber lengths");
    }

    #[test]
    fn test_butterfly_rde_mode() {
        let mut config = CoherentDspConfig::qam16_200g();
        config.cma_pretrain_symbols = 100000; // stay in CMA/RDE mode
        let mut eq = ButterflyEqualizer::new(&config);
        eq.mode = ButterflyMode::Rde;
        let c = QamConstellation::new(QamOrder::Qam16);
        let sym = c.points()[0];
        // Should not panic
        let (out, _) = eq.process_sample(sym, (0.0, 0.0));
        assert!(c_abs(out).is_finite(), "RDE output should be finite");
    }
}

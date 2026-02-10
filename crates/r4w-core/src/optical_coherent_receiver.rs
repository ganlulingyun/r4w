//! Coherent optical signal detection and recovery for fiber-optic and free-space optical
//! communications.
//!
//! This module implements the digital signal processing chain for coherent optical receivers,
//! including 90-degree optical hybrid mixing, balanced photodetection, polarization
//! demultiplexing using the Constant Modulus Algorithm (CMA), carrier frequency offset
//! estimation via 4th-power spectral analysis, carrier phase recovery (Viterbi-Viterbi
//! and blind phase search), and adaptive equalization for chromatic dispersion compensation.
//!
//! All complex numbers are represented as `(f64, f64)` tuples where the first element is
//! the real (in-phase) component and the second is the imaginary (quadrature) component.
//!
//! # Example
//!
//! ```
//! use r4w_core::optical_coherent_receiver::{
//!     optical_hybrid_mix, balanced_photodetect, OpticalCoherentReceiver,
//! };
//!
//! // Create a simple QPSK signal and local oscillator
//! let signal = vec![(1.0_f64, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
//! let lo = vec![(1.0_f64, 0.0); 4];
//!
//! // 90-degree optical hybrid mixing
//! let (i_ch, q_ch) = optical_hybrid_mix(&signal, &lo);
//! assert_eq!(i_ch.0.len(), 4);
//! assert_eq!(q_ch.0.len(), 4);
//!
//! // Balanced photodetection
//! let detected_i = balanced_photodetect(&i_ch.0, &i_ch.1);
//! assert_eq!(detected_i.len(), 4);
//!
//! // Create a receiver instance
//! let rx = OpticalCoherentReceiver::new(28e9, 4, 100e3);
//! assert_eq!(rx.modulation_order(), 4);
//! ```

use std::f64::consts::PI;

// ─── Complex arithmetic helpers ─────────────────────────────────────────────

/// Multiply two complex numbers represented as `(f64, f64)` tuples.
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Conjugate of a complex number.
fn c_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Magnitude squared of a complex number.
fn c_abs2(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Magnitude of a complex number.
fn c_abs(a: (f64, f64)) -> f64 {
    c_abs2(a).sqrt()
}

/// Add two complex numbers.
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Subtract two complex numbers.
fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Scale a complex number by a real scalar.
fn c_scale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

/// Complex exponential: e^{j*theta}.
fn c_exp_j(theta: f64) -> (f64, f64) {
    (theta.cos(), theta.sin())
}

/// Phase angle of a complex number.
fn c_arg(a: (f64, f64)) -> f64 {
    a.1.atan2(a.0)
}

/// Raise a complex number to an integer power.
fn c_pow_int(mut base: (f64, f64), mut exp: u32) -> (f64, f64) {
    let mut result = (1.0, 0.0);
    while exp > 0 {
        if exp & 1 == 1 {
            result = c_mul(result, base);
        }
        base = c_mul(base, base);
        exp >>= 1;
    }
    result
}

// ─── Simple DFT (no external FFT crate) ────────────────────────────────────

/// Compute the Discrete Fourier Transform of a complex signal.
fn dft(x: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = x.len();
    let mut out = vec![(0.0, 0.0); n];
    for k in 0..n {
        let mut sum = (0.0, 0.0);
        for (i, &xi) in x.iter().enumerate() {
            let angle = -2.0 * PI * (k as f64) * (i as f64) / (n as f64);
            sum = c_add(sum, c_mul(xi, c_exp_j(angle)));
        }
        out[k] = sum;
    }
    out
}

/// Compute the Inverse Discrete Fourier Transform of a complex spectrum.
fn idft(x: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = x.len();
    let mut out = vec![(0.0, 0.0); n];
    for k in 0..n {
        let mut sum = (0.0, 0.0);
        for (i, &xi) in x.iter().enumerate() {
            let angle = 2.0 * PI * (k as f64) * (i as f64) / (n as f64);
            sum = c_add(sum, c_mul(xi, c_exp_j(angle)));
        }
        out[k] = c_scale(sum, 1.0 / n as f64);
    }
    out
}

// ─── Public API ─────────────────────────────────────────────────────────────

/// Perform 90-degree optical hybrid mixing of the incoming signal with a local
/// oscillator (LO).
///
/// Returns a tuple of `(i_channel, q_channel)` where each channel is a pair of
/// `(positive_arm, negative_arm)` vectors suitable for balanced photodetection.
///
/// The hybrid implements the standard four-port coupler:
/// - I+: signal + LO
/// - I-: signal - LO
/// - Q+: signal + j*LO
/// - Q-: signal - j*LO
///
/// # Panics
///
/// Panics if `signal` and `lo` have different lengths.
pub fn optical_hybrid_mix(
    signal: &[(f64, f64)],
    lo: &[(f64, f64)],
) -> (
    (Vec<(f64, f64)>, Vec<(f64, f64)>),
    (Vec<(f64, f64)>, Vec<(f64, f64)>),
) {
    assert_eq!(
        signal.len(),
        lo.len(),
        "signal and LO must have the same length"
    );
    let n = signal.len();

    let mut i_pos = Vec::with_capacity(n);
    let mut i_neg = Vec::with_capacity(n);
    let mut q_pos = Vec::with_capacity(n);
    let mut q_neg = Vec::with_capacity(n);

    let j = (0.0, 1.0);

    for i in 0..n {
        let s = signal[i];
        let l = lo[i];

        // I channel: signal ± LO
        i_pos.push(c_add(s, l));
        i_neg.push(c_sub(s, l));

        // Q channel: signal ± j*LO
        let jl = c_mul(j, l);
        q_pos.push(c_add(s, jl));
        q_neg.push(c_sub(s, jl));
    }

    ((i_pos, i_neg), (q_pos, q_neg))
}

/// Perform balanced photodetection on a pair of positive and negative optical signals.
///
/// The balanced detector computes `|pos|^2 - |neg|^2` for each sample, which
/// removes the DC bias and common-mode noise present in direct detection.
///
/// # Panics
///
/// Panics if `pos` and `neg` have different lengths.
pub fn balanced_photodetect(pos: &[(f64, f64)], neg: &[(f64, f64)]) -> Vec<f64> {
    assert_eq!(
        pos.len(),
        neg.len(),
        "positive and negative arms must have the same length"
    );
    pos.iter()
        .zip(neg.iter())
        .map(|(&p, &n)| c_abs2(p) - c_abs2(n))
        .collect()
}

/// Configuration and state for a coherent optical receiver.
///
/// This struct encapsulates the key system parameters needed for the various
/// DSP recovery algorithms: symbol rate, modulation order (e.g. 4 for QPSK),
/// and laser linewidth for phase noise modelling.
pub struct OpticalCoherentReceiver {
    symbol_rate: f64,
    modulation_order: u32,
    linewidth_hz: f64,
}

impl OpticalCoherentReceiver {
    /// Create a new coherent optical receiver configuration.
    ///
    /// - `symbol_rate` -- Symbol rate in symbols per second (e.g. 28e9 for 28 GBaud).
    /// - `modulation_order` -- Constellation order (e.g. 4 for QPSK, 16 for 16-QAM).
    /// - `linewidth_hz` -- Combined laser linewidth in Hz (transmitter + LO).
    pub fn new(symbol_rate: f64, modulation_order: u32, linewidth_hz: f64) -> Self {
        Self {
            symbol_rate,
            modulation_order,
            linewidth_hz,
        }
    }

    /// Return the configured symbol rate.
    pub fn symbol_rate(&self) -> f64 {
        self.symbol_rate
    }

    /// Return the configured modulation order.
    pub fn modulation_order(&self) -> u32 {
        self.modulation_order
    }

    /// Return the configured linewidth.
    pub fn linewidth_hz(&self) -> f64 {
        self.linewidth_hz
    }

    /// Run the full coherent receive chain on I and Q detected samples.
    ///
    /// Steps performed:
    /// 1. Combine I/Q into complex samples
    /// 2. Chromatic dispersion compensation (if `dispersion_params` is `Some`)
    /// 3. Carrier frequency offset estimation and correction
    /// 4. Carrier phase recovery via Viterbi-Viterbi
    ///
    /// The optional `dispersion_params` tuple contains
    /// `(dispersion_ps_nm, length_km, wavelength_nm)`.
    ///
    /// Returns the recovered complex symbols.
    pub fn process(
        &self,
        i_samples: &[f64],
        q_samples: &[f64],
        dispersion_params: Option<(f64, f64, f64)>,
    ) -> Vec<(f64, f64)> {
        assert_eq!(i_samples.len(), q_samples.len());
        let mut samples: Vec<(f64, f64)> = i_samples
            .iter()
            .zip(q_samples.iter())
            .map(|(&i, &q)| (i, q))
            .collect();

        // CD compensation
        if let Some((disp, length, wavelength)) = dispersion_params {
            samples = chromatic_dispersion_compensate(&samples, disp, length, wavelength);
        }

        // CFO estimation and correction
        let cfo = estimate_frequency_offset(&samples);
        let n = samples.len() as f64;
        for (k, s) in samples.iter_mut().enumerate() {
            let phase = -2.0 * PI * cfo * (k as f64) / n;
            *s = c_mul(*s, c_exp_j(phase));
        }

        // CPE via Viterbi-Viterbi
        viterbi_viterbi_cpe(&samples, self.modulation_order)
    }
}

/// Polarization demultiplexing using the Constant Modulus Algorithm (CMA) butterfly
/// filter structure.
///
/// Implements a 2x2 MIMO adaptive equalizer where four FIR filters (hxx, hxy, hyx, hyy)
/// are updated to minimize `(|y|^2 - 1)^2` on each output. This separates arbitrarily
/// mixed X and Y polarization tributaries.
///
/// - `x_pol` -- samples from the X polarization receiver
/// - `y_pol` -- samples from the Y polarization receiver
/// - `taps` -- number of equalizer taps (should be odd, typically 7-21)
///
/// Returns `(x_out, y_out)` -- the demultiplexed polarization tributaries.
///
/// # Panics
///
/// Panics if `x_pol` and `y_pol` have different lengths or if `taps` is zero.
pub fn polarization_demux_cma(
    x_pol: &[(f64, f64)],
    y_pol: &[(f64, f64)],
    taps: usize,
) -> (Vec<(f64, f64)>, Vec<(f64, f64)>) {
    assert_eq!(x_pol.len(), y_pol.len(), "polarizations must be equal length");
    assert!(taps > 0, "taps must be positive");

    let n = x_pol.len();
    if n < taps {
        return (vec![], vec![]);
    }

    let mu = 1e-3; // CMA step size

    // Initialize butterfly filters: hxx = delta, others = 0
    let mut hxx = vec![(0.0, 0.0); taps];
    let mut hxy = vec![(0.0, 0.0); taps];
    let mut hyx = vec![(0.0, 0.0); taps];
    let mut hyy = vec![(0.0, 0.0); taps];

    // Center tap initialization
    let center = taps / 2;
    hxx[center] = (1.0, 0.0);
    hyy[center] = (1.0, 0.0);

    let out_len = n - taps + 1;
    let mut x_out = Vec::with_capacity(out_len);
    let mut y_out = Vec::with_capacity(out_len);

    for k in 0..out_len {
        // Build input vectors (reversed for convolution)
        let x_in: Vec<(f64, f64)> = (0..taps).map(|i| x_pol[k + taps - 1 - i]).collect();
        let y_in: Vec<(f64, f64)> = (0..taps).map(|i| y_pol[k + taps - 1 - i]).collect();

        // Compute outputs
        let mut ox = (0.0, 0.0);
        let mut oy = (0.0, 0.0);
        for i in 0..taps {
            ox = c_add(ox, c_add(c_mul(hxx[i], x_in[i]), c_mul(hxy[i], y_in[i])));
            oy = c_add(oy, c_add(c_mul(hyx[i], x_in[i]), c_mul(hyy[i], y_in[i])));
        }

        x_out.push(ox);
        y_out.push(oy);

        // CMA error: e = (|y|^2 - R) * y, where R = 1 (unit modulus target)
        let ex = c_scale(ox, c_abs2(ox) - 1.0);
        let ey = c_scale(oy, c_abs2(oy) - 1.0);

        // Update taps: h -= mu * e * conj(input)
        for i in 0..taps {
            let cx = c_conj(x_in[i]);
            let cy = c_conj(y_in[i]);

            hxx[i] = c_sub(hxx[i], c_scale(c_mul(ex, cx), mu));
            hxy[i] = c_sub(hxy[i], c_scale(c_mul(ex, cy), mu));
            hyx[i] = c_sub(hyx[i], c_scale(c_mul(ey, cx), mu));
            hyy[i] = c_sub(hyy[i], c_scale(c_mul(ey, cy), mu));
        }
    }

    (x_out, y_out)
}

/// Estimate the carrier frequency offset (CFO) from coherent receiver samples using the
/// 4th-power spectral method.
///
/// Raises each sample to the 4th power (suitable for QPSK) to remove modulation, then
/// finds the peak in the DFT of the raised signal. The returned value is in normalized
/// frequency units (cycles per sample), ranging from approximately -0.5 to 0.5.
///
/// For an absolute frequency offset in Hz, multiply by the sample rate.
pub fn estimate_frequency_offset(samples: &[(f64, f64)]) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }

    // Raise to 4th power to strip QPSK modulation
    let raised: Vec<(f64, f64)> = samples.iter().map(|&s| c_pow_int(s, 4)).collect();

    // DFT
    let spectrum = dft(&raised);
    let n = spectrum.len();

    // Find peak bin
    let mut max_mag = 0.0;
    let mut peak_bin = 0usize;
    for (k, &s) in spectrum.iter().enumerate() {
        let mag = c_abs2(s);
        if mag > max_mag {
            max_mag = mag;
            peak_bin = k;
        }
    }

    // Convert bin to normalized frequency, accounting for negative frequencies
    let freq_bin = if peak_bin > n / 2 {
        peak_bin as f64 - n as f64
    } else {
        peak_bin as f64
    };

    // Divide by 4 because we raised to the 4th power
    freq_bin / (n as f64 * 4.0)
}

/// Carrier phase estimation (CPE) using the Viterbi-Viterbi algorithm.
///
/// Raises each symbol to the `order`-th power (e.g. 4 for QPSK) to remove the
/// modulation, averages the phase over a sliding window, and rotates each symbol
/// to compensate for the estimated phase error.
///
/// - `samples` -- input complex symbols
/// - `order` -- modulation order (e.g. 4 for QPSK)
///
/// Returns phase-corrected symbols.
pub fn viterbi_viterbi_cpe(samples: &[(f64, f64)], order: u32) -> Vec<(f64, f64)> {
    if samples.is_empty() {
        return vec![];
    }

    let n = samples.len();
    let window = 9.min(n); // Averaging window size

    // Raise to M-th power
    let raised: Vec<(f64, f64)> = samples.iter().map(|&s| c_pow_int(s, order)).collect();

    // Sliding-window average of raised symbols
    let half_w = window / 2;
    let mut corrected = Vec::with_capacity(n);

    for k in 0..n {
        let start = if k >= half_w { k - half_w } else { 0 };
        let end = (k + half_w + 1).min(n);
        let count = end - start;

        let mut sum = (0.0, 0.0);
        for i in start..end {
            sum = c_add(sum, raised[i]);
        }
        let avg = c_scale(sum, 1.0 / count as f64);

        // Phase estimate (divide by order to unwrap)
        let phase_est = c_arg(avg) / order as f64;

        // Correct the sample
        corrected.push(c_mul(samples[k], c_exp_j(-phase_est)));
    }

    corrected
}

/// Carrier phase recovery using the Blind Phase Search (BPS) algorithm.
///
/// Tests a set of candidate phase rotations on each symbol and selects the one
/// that minimizes the distance to the nearest constellation point. This method
/// is more robust than Viterbi-Viterbi for higher-order modulations (e.g. 16-QAM).
///
/// - `samples` -- input complex symbols
/// - `test_phases` -- number of candidate phases to test (e.g. 32 or 64)
///
/// Returns phase-corrected symbols.
pub fn blind_phase_search(samples: &[(f64, f64)], test_phases: usize) -> Vec<(f64, f64)> {
    if samples.is_empty() || test_phases == 0 {
        return samples.to_vec();
    }

    let window = 9.min(samples.len());
    let half_w = window / 2;
    let n = samples.len();

    // Generate test phase angles in [0, pi/2) for QPSK symmetry
    let phases: Vec<f64> = (0..test_phases)
        .map(|i| (i as f64) * PI / (2.0 * test_phases as f64))
        .collect();

    // QPSK constellation points
    let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
    let constellation: Vec<(f64, f64)> = vec![
        (inv_sqrt2, inv_sqrt2),
        (-inv_sqrt2, inv_sqrt2),
        (-inv_sqrt2, -inv_sqrt2),
        (inv_sqrt2, -inv_sqrt2),
    ];

    // Distance to nearest constellation point
    let min_dist = |s: (f64, f64)| -> f64 {
        constellation
            .iter()
            .map(|&c| c_abs2(c_sub(s, c)))
            .fold(f64::MAX, f64::min)
    };

    let mut corrected = Vec::with_capacity(n);

    for k in 0..n {
        let start = if k >= half_w { k - half_w } else { 0 };
        let end = (k + half_w + 1).min(n);

        // For each test phase, compute total distance over the window
        let mut best_phase = 0.0;
        let mut best_cost = f64::MAX;

        for &phi in &phases {
            let rot = c_exp_j(phi);
            let mut cost = 0.0;
            for i in start..end {
                let rotated = c_mul(samples[i], rot);
                cost += min_dist(rotated);
            }
            if cost < best_cost {
                best_cost = cost;
                best_phase = phi;
            }
        }

        corrected.push(c_mul(samples[k], c_exp_j(best_phase)));
    }

    corrected
}

/// Compensate for chromatic dispersion in a single-mode fiber channel.
///
/// Applies an all-pass frequency-domain filter that inverts the quadratic phase
/// accumulated over the fiber link. This is the most common first step in coherent
/// receiver DSP.
///
/// - `samples` -- input complex samples (at the receiver sample rate)
/// - `dispersion_ps_nm` -- fiber dispersion parameter in ps/(nm*km), typically 17 for SMF-28
/// - `length_km` -- fiber length in km
/// - `wavelength_nm` -- carrier wavelength in nm (e.g. 1550)
///
/// Returns dispersion-compensated complex samples.
pub fn chromatic_dispersion_compensate(
    samples: &[(f64, f64)],
    dispersion_ps_nm: f64,
    length_km: f64,
    wavelength_nm: f64,
) -> Vec<(f64, f64)> {
    if samples.is_empty() {
        return vec![];
    }

    let n = samples.len();

    // Convert to SI units
    // D in ps/(nm*km) = 1e-12 s / (1e-9 m * 1e3 m) = 1e-6 s/m^2
    let d = dispersion_ps_nm * 1e-6;
    let l = length_km * 1e3; // km -> m
    let lambda = wavelength_nm * 1e-9; // nm -> m
    let c = 3e8; // speed of light m/s

    // beta2 = -D * lambda^2 / (2 * pi * c)
    let beta2 = -d * lambda * lambda / (2.0 * PI * c);

    // Total accumulated dispersion
    let total_beta2_l = beta2 * l;

    // Forward DFT
    let spectrum = dft(samples);

    // Apply frequency-domain CD compensation filter
    let mut compensated_spectrum = Vec::with_capacity(n);
    for k in 0..n {
        // Normalized frequency
        let freq = if k <= n / 2 {
            k as f64 / n as f64
        } else {
            (k as f64 - n as f64) / n as f64
        };
        // The transfer function of CD is H(f) = exp(j * beta2 * L * (2*pi*f)^2 / 2)
        // Compensation is the conjugate: exp(-j * beta2 * L * (2*pi*f)^2 / 2)
        let omega = 2.0 * PI * freq;
        let phase = -0.5 * total_beta2_l * omega * omega;
        compensated_spectrum.push(c_mul(spectrum[k], c_exp_j(phase)));
    }

    // Inverse DFT
    idft(&compensated_spectrum)
}

/// Count bit errors between transmitted and received bit sequences.
///
/// Returns `(error_count, total_bits)`. The comparison stops at the length of the
/// shorter sequence.
///
/// # Example
///
/// ```
/// use r4w_core::optical_coherent_receiver::ber_count;
/// let tx = vec![true, false, true, true, false];
/// let rx = vec![true, true,  true, false, false];
/// let (errors, total) = ber_count(&tx, &rx);
/// assert_eq!(errors, 2);
/// assert_eq!(total, 5);
/// ```
pub fn ber_count(tx_bits: &[bool], rx_bits: &[bool]) -> (usize, usize) {
    let total = tx_bits.len().min(rx_bits.len());
    let errors = tx_bits
        .iter()
        .zip(rx_bits.iter())
        .filter(|(&t, &r)| t != r)
        .count();
    (errors, total)
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-9;

    fn assert_close(a: f64, b: f64, tolerance: f64) {
        assert!(
            (a - b).abs() < tolerance,
            "expected {b}, got {a}, diff = {}",
            (a - b).abs()
        );
    }

    fn assert_c_close(a: (f64, f64), b: (f64, f64), tolerance: f64) {
        assert_close(a.0, b.0, tolerance);
        assert_close(a.1, b.1, tolerance);
    }

    // ── 1. optical_hybrid_mix basic ─────────────────────────────────────

    #[test]
    fn test_optical_hybrid_mix_unit_signal_and_lo() {
        let signal = vec![(1.0, 0.0)];
        let lo = vec![(1.0, 0.0)];
        let ((i_pos, i_neg), (q_pos, q_neg)) = optical_hybrid_mix(&signal, &lo);

        // I+: (1,0)+(1,0) = (2,0)
        assert_c_close(i_pos[0], (2.0, 0.0), TOL);
        // I-: (1,0)-(1,0) = (0,0)
        assert_c_close(i_neg[0], (0.0, 0.0), TOL);
        // Q+: (1,0)+(0,1)*(1,0) = (1,0)+(0,1) = (1,1)
        assert_c_close(q_pos[0], (1.0, 1.0), TOL);
        // Q-: (1,0)-(0,1)*(1,0) = (1,0)-(0,1) = (1,-1)
        assert_c_close(q_neg[0], (1.0, -1.0), TOL);
    }

    // ── 2. optical_hybrid_mix length preservation ───────────────────────

    #[test]
    fn test_optical_hybrid_mix_preserves_length() {
        let n = 16;
        let signal: Vec<(f64, f64)> = (0..n).map(|i| (i as f64, 0.0)).collect();
        let lo = vec![(1.0, 0.0); n];
        let ((i_pos, _), (q_pos, _)) = optical_hybrid_mix(&signal, &lo);
        assert_eq!(i_pos.len(), n);
        assert_eq!(q_pos.len(), n);
    }

    // ── 3. balanced_photodetect DC removal ──────────────────────────────

    #[test]
    fn test_balanced_photodetect_removes_dc() {
        // If pos and neg have the same power, output should be zero
        let pos = vec![(1.0, 0.0); 4];
        let neg = vec![(1.0, 0.0); 4];
        let result = balanced_photodetect(&pos, &neg);
        for &v in &result {
            assert_close(v, 0.0, TOL);
        }
    }

    // ── 4. balanced_photodetect known values ────────────────────────────

    #[test]
    fn test_balanced_photodetect_known_values() {
        let pos = vec![(2.0, 0.0), (0.0, 3.0)];
        let neg = vec![(1.0, 0.0), (0.0, 1.0)];
        let result = balanced_photodetect(&pos, &neg);
        // |2|^2 - |1|^2 = 4 - 1 = 3
        assert_close(result[0], 3.0, TOL);
        // |3j|^2 - |j|^2 = 9 - 1 = 8
        assert_close(result[1], 8.0, TOL);
    }

    // ── 5. coherent receiver construction ───────────────────────────────

    #[test]
    fn test_receiver_construction() {
        let rx = OpticalCoherentReceiver::new(28e9, 4, 100e3);
        assert_close(rx.symbol_rate(), 28e9, TOL);
        assert_eq!(rx.modulation_order(), 4);
        assert_close(rx.linewidth_hz(), 100e3, TOL);
    }

    // ── 6. estimate_frequency_offset zero offset ────────────────────────

    #[test]
    fn test_estimate_frequency_offset_zero() {
        // Pure QPSK signal with no CFO should give ~0 offset
        let n = 64;
        let qpsk: Vec<(f64, f64)> = (0..n)
            .map(|i| match i % 4 {
                0 => (1.0, 1.0),
                1 => (-1.0, 1.0),
                2 => (-1.0, -1.0),
                _ => (1.0, -1.0),
            })
            .collect();
        let offset = estimate_frequency_offset(&qpsk);
        assert!(offset.abs() < 0.01, "expected ~0, got {offset}");
    }

    // ── 7. estimate_frequency_offset non-zero ───────────────────────────

    #[test]
    fn test_estimate_frequency_offset_nonzero() {
        let n = 64;
        let cfo = 0.05; // normalized frequency offset
        // Apply CFO to constant-amplitude signal
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|k| c_exp_j(2.0 * PI * cfo * k as f64))
            .collect();
        let estimated = estimate_frequency_offset(&samples);
        // Should be close to the applied CFO
        assert!(
            (estimated - cfo).abs() < 0.02,
            "expected ~{cfo}, got {estimated}"
        );
    }

    // ── 8. estimate_frequency_offset empty input ────────────────────────

    #[test]
    fn test_estimate_frequency_offset_empty() {
        assert_close(estimate_frequency_offset(&[]), 0.0, TOL);
    }

    // ── 9. viterbi_viterbi_cpe constant phase ───────────────────────────

    #[test]
    fn test_viterbi_cpe_constant_phase() {
        // QPSK symbols with a fixed phase offset; VV should remove the offset
        // such that the relative spacing between symbols is preserved.
        let phase_offset = 0.1;
        let n = 32;
        let base_phases = [PI / 4.0, 3.0 * PI / 4.0, -3.0 * PI / 4.0, -PI / 4.0];
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| c_exp_j(base_phases[i % 4] + phase_offset))
            .collect();

        let corrected = viterbi_viterbi_cpe(&samples, 4);
        assert_eq!(corrected.len(), n);

        // After VV correction, all symbols should have near-unit amplitude
        for (i, &s) in corrected.iter().enumerate() {
            let mag = c_abs(s);
            assert!(
                (mag - 1.0).abs() < 0.05,
                "sample {i}: magnitude {mag} far from 1.0"
            );
        }

        // The phase differences between consecutive symbols with the same
        // input pattern should be consistent (i.e. the CPE removed the
        // common offset even if there is a pi/2 ambiguity).
        // Check that symbols 0 and 4 (same constellation point) have ~same phase.
        for k in 0..(n / 4 - 1) {
            let p0 = c_arg(corrected[k * 4]);
            let p1 = c_arg(corrected[(k + 1) * 4]);
            let mut diff = (p0 - p1).abs();
            if diff > PI {
                diff = 2.0 * PI - diff;
            }
            assert!(
                diff < 0.15,
                "symbols {} and {}: phase diff = {diff:.4}",
                k * 4,
                (k + 1) * 4
            );
        }

        // Check that the phase separation between symbol 0 and symbol 1
        // is close to pi/2 (the QPSK spacing)
        let p0 = c_arg(corrected[0]);
        let p1 = c_arg(corrected[1]);
        let mut sep = (p1 - p0).abs();
        if sep > PI {
            sep = 2.0 * PI - sep;
        }
        assert!(
            (sep - PI / 2.0).abs() < 0.15,
            "symbol 0-1 phase separation {sep:.4} not close to pi/2"
        );
    }

    // ── 10. viterbi_viterbi_cpe empty ───────────────────────────────────

    #[test]
    fn test_viterbi_cpe_empty() {
        assert!(viterbi_viterbi_cpe(&[], 4).is_empty());
    }

    // ── 11. blind_phase_search basic ────────────────────────────────────

    #[test]
    fn test_blind_phase_search_no_rotation() {
        // Ideal QPSK symbols (no phase error) should pass through almost unchanged
        let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
        let samples = vec![
            (inv_sqrt2, inv_sqrt2),
            (-inv_sqrt2, inv_sqrt2),
            (-inv_sqrt2, -inv_sqrt2),
            (inv_sqrt2, -inv_sqrt2),
        ];
        let result = blind_phase_search(&samples, 32);
        assert_eq!(result.len(), 4);
        // The best phase should be 0 (or very close), so symbols should be ~unchanged
        for (i, &s) in result.iter().enumerate() {
            let mag = c_abs(s);
            assert!(
                (mag - 1.0).abs() < 0.01,
                "sample {i}: magnitude {mag} != 1.0"
            );
        }
    }

    // ── 12. blind_phase_search empty ────────────────────────────────────

    #[test]
    fn test_blind_phase_search_empty() {
        assert!(blind_phase_search(&[], 32).is_empty());
    }

    // ── 13. blind_phase_search zero test phases ─────────────────────────

    #[test]
    fn test_blind_phase_search_zero_phases() {
        let samples = vec![(1.0, 0.0)];
        let result = blind_phase_search(&samples, 0);
        assert_eq!(result.len(), 1);
    }

    // ── 14. chromatic_dispersion_compensate roundtrip ────────────────────

    #[test]
    fn test_cd_compensate_roundtrip() {
        // Applying CD then compensating should return close to original
        let n = 32;
        let original: Vec<(f64, f64)> =
            (0..n).map(|i| c_exp_j(PI / 4.0 * (i % 4) as f64)).collect();

        let d = 17.0; // ps/(nm*km)
        let l = 80.0; // km
        let lambda = 1550.0; // nm

        // Forward dispersion
        let dispersed = apply_cd_forward(&original, d, l, lambda);
        // Compensate
        let recovered = chromatic_dispersion_compensate(&dispersed, d, l, lambda);

        // Check recovery (accounting for DFT numerical noise)
        for i in 0..n {
            assert_c_close(recovered[i], original[i], 1e-6);
        }
    }

    /// Helper: apply chromatic dispersion in the forward direction.
    fn apply_cd_forward(
        samples: &[(f64, f64)],
        dispersion_ps_nm: f64,
        length_km: f64,
        wavelength_nm: f64,
    ) -> Vec<(f64, f64)> {
        let n = samples.len();
        let d = dispersion_ps_nm * 1e-6;
        let l = length_km * 1e3;
        let lambda = wavelength_nm * 1e-9;
        let c = 3e8;
        let beta2 = -d * lambda * lambda / (2.0 * PI * c);
        let total_beta2_l = beta2 * l;

        let spectrum = dft(samples);
        let mut dispersed_spectrum = Vec::with_capacity(n);
        for k in 0..n {
            let freq = if k <= n / 2 {
                k as f64 / n as f64
            } else {
                (k as f64 - n as f64) / n as f64
            };
            let omega = 2.0 * PI * freq;
            // Forward: +j * beta2*L * omega^2 / 2
            let phase = 0.5 * total_beta2_l * omega * omega;
            dispersed_spectrum.push(c_mul(spectrum[k], c_exp_j(phase)));
        }
        idft(&dispersed_spectrum)
    }

    // ── 15. chromatic_dispersion_compensate empty ────────────────────────

    #[test]
    fn test_cd_compensate_empty() {
        assert!(chromatic_dispersion_compensate(&[], 17.0, 80.0, 1550.0).is_empty());
    }

    // ── 16. ber_count basic ─────────────────────────────────────────────

    #[test]
    fn test_ber_count_no_errors() {
        let bits = vec![true, false, true, false];
        let (errors, total) = ber_count(&bits, &bits);
        assert_eq!(errors, 0);
        assert_eq!(total, 4);
    }

    // ── 17. ber_count with errors ───────────────────────────────────────

    #[test]
    fn test_ber_count_with_errors() {
        let tx = vec![true, false, true, true, false];
        let rx = vec![true, true, true, false, false];
        let (errors, total) = ber_count(&tx, &rx);
        assert_eq!(errors, 2);
        assert_eq!(total, 5);
    }

    // ── 18. ber_count different lengths ─────────────────────────────────

    #[test]
    fn test_ber_count_different_lengths() {
        let tx = vec![true, false, true];
        let rx = vec![true, false];
        let (errors, total) = ber_count(&tx, &rx);
        assert_eq!(errors, 0);
        assert_eq!(total, 2);
    }

    // ── 19. polarization_demux_cma basic convergence ────────────────────

    #[test]
    fn test_polarization_demux_cma_identity() {
        // If X and Y polarizations are already separated (unit modulus),
        // the CMA should preserve them approximately
        let n = 200;
        let x_pol: Vec<(f64, f64)> = (0..n)
            .map(|i| c_exp_j(PI / 4.0 + PI / 2.0 * (i % 4) as f64))
            .collect();
        let y_pol: Vec<(f64, f64)> = (0..n)
            .map(|i| c_exp_j(3.0 * PI / 4.0 + PI / 2.0 * (i % 4) as f64))
            .collect();

        let (x_out, y_out) = polarization_demux_cma(&x_pol, &y_pol, 5);

        // Output should exist
        assert!(!x_out.is_empty());
        assert!(!y_out.is_empty());

        // After convergence, output should have approximately unit modulus
        let start = x_out.len() / 2; // skip convergence transient
        for &s in &x_out[start..] {
            let mag = c_abs(s);
            assert!(
                (mag - 1.0).abs() < 0.3,
                "X output magnitude {mag} far from 1.0"
            );
        }
    }

    // ── 20. polarization_demux_cma short input ──────────────────────────

    #[test]
    fn test_polarization_demux_cma_short_input() {
        // Input shorter than taps should return empty
        let x = vec![(1.0, 0.0); 3];
        let y = vec![(0.0, 1.0); 3];
        let (xo, yo) = polarization_demux_cma(&x, &y, 5);
        assert!(xo.is_empty());
        assert!(yo.is_empty());
    }

    // ── 21. full receiver process chain ─────────────────────────────────

    #[test]
    fn test_receiver_process_chain() {
        let rx = OpticalCoherentReceiver::new(28e9, 4, 100e3);
        let n = 32;

        // Generate clean QPSK I/Q
        let i_samples: Vec<f64> = (0..n)
            .map(|k| match k % 4 {
                0 => 1.0,
                1 => -1.0,
                2 => -1.0,
                _ => 1.0,
            })
            .collect();
        let q_samples: Vec<f64> = (0..n)
            .map(|k| match k % 4 {
                0 => 1.0,
                1 => 1.0,
                2 => -1.0,
                _ => -1.0,
            })
            .collect();

        let result = rx.process(&i_samples, &q_samples, None);
        assert_eq!(result.len(), n);

        // All symbols should still have approximately sqrt(2) magnitude
        for (i, &s) in result.iter().enumerate() {
            let mag = c_abs(s);
            assert!(
                (mag - 2.0_f64.sqrt()).abs() < 0.3,
                "sample {i}: magnitude {mag} far from sqrt(2)"
            );
        }
    }

    // ── 22. DFT/IDFT roundtrip ──────────────────────────────────────────

    #[test]
    fn test_dft_idft_roundtrip() {
        let original: Vec<(f64, f64)> =
            vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        let spectrum = dft(&original);
        let recovered = idft(&spectrum);
        for i in 0..original.len() {
            assert_c_close(recovered[i], original[i], 1e-10);
        }
    }

    // ── 23. complex arithmetic helpers ──────────────────────────────────

    #[test]
    fn test_complex_arithmetic() {
        // Multiplication: (1+2j)(3+4j) = 3+4j+6j+8j^2 = -5+10j
        assert_c_close(c_mul((1.0, 2.0), (3.0, 4.0)), (-5.0, 10.0), TOL);

        // Conjugate
        assert_c_close(c_conj((3.0, 4.0)), (3.0, -4.0), TOL);

        // Magnitude squared
        assert_close(c_abs2((3.0, 4.0)), 25.0, TOL);

        // Magnitude
        assert_close(c_abs((3.0, 4.0)), 5.0, TOL);

        // c_exp_j(0) = (1, 0)
        assert_c_close(c_exp_j(0.0), (1.0, 0.0), TOL);

        // c_exp_j(pi/2) = (0, 1)
        assert_c_close(c_exp_j(PI / 2.0), (0.0, 1.0), TOL);
    }

    // ── 24. optical hybrid with complex LO ──────────────────────────────

    #[test]
    fn test_optical_hybrid_complex_lo() {
        let signal = vec![(1.0, 1.0)];
        let lo = vec![(0.0, 1.0)]; // j
        let ((i_pos, i_neg), (q_pos, q_neg)) = optical_hybrid_mix(&signal, &lo);

        // I+: (1,1)+(0,1) = (1,2)
        assert_c_close(i_pos[0], (1.0, 2.0), TOL);
        // I-: (1,1)-(0,1) = (1,0)
        assert_c_close(i_neg[0], (1.0, 0.0), TOL);
        // j*LO = j*j = -1 => (-1,0)
        // Q+: (1,1)+(-1,0) = (0,1)
        assert_c_close(q_pos[0], (0.0, 1.0), TOL);
        // Q-: (1,1)-(-1,0) = (2,1)
        assert_c_close(q_neg[0], (2.0, 1.0), TOL);
    }
}

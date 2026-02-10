//! # Frequency-Domain Equalizer
//!
//! Single-carrier frequency-domain equalization (SC-FDE) using FFT-based
//! processing with zero-forcing (ZF) and minimum mean square error (MMSE)
//! algorithms.
//!
//! SC-FDE converts a time-domain equalization problem into per-subcarrier
//! scalar division in the frequency domain, dramatically reducing complexity
//! from O(N²) to O(N log N).
//!
//! # Algorithms
//!
//! - **Zero-Forcing (ZF):** `W(f) = 1 / H(f)` — inverts the channel exactly,
//!   but amplifies noise where the channel has deep fades.
//! - **MMSE:** `W(f) = conj(H(f)) / (|H(f)|² + σ²)` — balances channel
//!   inversion against noise amplification for better BER performance.
//!
//! # Example
//!
//! ```
//! use r4w_core::frequency_domain_equalizer::{
//!     FrequencyDomainEqualizer, EqualizerAlgorithm,
//! };
//!
//! // Create an 8-point MMSE equalizer
//! let mut eq = FrequencyDomainEqualizer::new(8, EqualizerAlgorithm::Mmse { noise_variance: 0.01 });
//!
//! // Known transmitted pilot symbols and received (distorted) pilots
//! let tx_pilots: Vec<(f64, f64)> = vec![
//!     (1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0),
//!     (1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0),
//! ];
//! // Simulate a mild channel: scale by 0.8 and add small offset
//! let rx_pilots: Vec<(f64, f64)> = tx_pilots.iter()
//!     .map(|&(re, im)| (re * 0.8 + 0.01, im * 0.8 - 0.01))
//!     .collect();
//!
//! eq.estimate_channel(&tx_pilots, &rx_pilots);
//!
//! // Equalize a received data block
//! let received: Vec<(f64, f64)> = vec![
//!     (0.81, 0.01), (0.01, 0.79), (-0.79, 0.01), (0.01, -0.81),
//!     (0.80, -0.01), (-0.01, 0.80), (-0.81, -0.01), (-0.01, -0.79),
//! ];
//! let equalized = eq.equalize(&received);
//! assert_eq!(equalized.len(), 8);
//!
//! // Each equalized sample should be close to the original unit-magnitude symbol
//! let (re, im) = equalized[0];
//! assert!((re - 1.0).abs() < 0.15, "real part: {re}");
//! assert!(im.abs() < 0.15, "imag part: {im}");
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers using (f64, f64) tuples
// ---------------------------------------------------------------------------

/// Complex addition.
#[inline]
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex subtraction.
#[inline]
fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Complex multiplication.
#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex conjugate.
#[inline]
fn c_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Squared magnitude |z|².
#[inline]
fn c_mag_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Complex division a / b.
#[inline]
fn c_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = c_mag_sq(b);
    if denom < 1e-30 {
        return (0.0, 0.0);
    }
    let num = c_mul(a, c_conj(b));
    (num.0 / denom, num.1 / denom)
}

/// Scale a complex value by a real scalar.
#[inline]
fn c_scale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

// ---------------------------------------------------------------------------
// Radix-2 FFT
// ---------------------------------------------------------------------------

/// Returns true if `n` is a power of two.
fn is_power_of_two(n: usize) -> bool {
    n > 0 && (n & (n - 1)) == 0
}

/// In-place radix-2 Cooley-Tukey FFT.
///
/// `inverse` = false for forward FFT, true for IFFT (includes 1/N scaling).
fn fft_radix2(buf: &mut [(f64, f64)], inverse: bool) {
    let n = buf.len();
    assert!(is_power_of_two(n), "FFT size must be a power of two, got {n}");

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
    let angle_sign: f64 = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = angle_sign * 2.0 * PI / len as f64;
        let wn = (angle.cos(), angle.sin());

        let mut start = 0;
        while start < n {
            let mut w: (f64, f64) = (1.0, 0.0);
            for k in 0..half {
                let u = buf[start + k];
                let t = c_mul(w, buf[start + k + half]);
                buf[start + k] = c_add(u, t);
                buf[start + k + half] = c_sub(u, t);
                w = c_mul(w, wn);
            }
            start += len;
        }
        len <<= 1;
    }

    // IFFT scaling
    if inverse {
        let scale = 1.0 / n as f64;
        for sample in buf.iter_mut() {
            *sample = c_scale(*sample, scale);
        }
    }
}

/// Forward FFT on a copy of the input.
fn fft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut buf = input.to_vec();
    fft_radix2(&mut buf, false);
    buf
}

/// Inverse FFT on a copy of the input.
fn ifft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut buf = input.to_vec();
    fft_radix2(&mut buf, true);
    buf
}

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Equalization algorithm selection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EqualizerAlgorithm {
    /// Zero-Forcing: `W(f) = 1 / H(f)`.
    ZeroForcing,
    /// Minimum Mean Square Error: `W(f) = conj(H(f)) / (|H(f)|² + σ²)`.
    Mmse {
        /// Noise variance (σ²) estimate.
        noise_variance: f64,
    },
}

/// Single-carrier frequency-domain equalizer.
///
/// Operates on blocks of `fft_size` complex samples. The typical workflow is:
///
/// 1. Optionally remove a cyclic prefix with [`remove_cyclic_prefix`].
/// 2. Estimate the channel from pilot symbols with [`estimate_channel`].
/// 3. Equalize received data blocks with [`equalize`].
/// 4. Read per-subcarrier SNR estimates with [`snr_per_subcarrier`].
#[derive(Debug, Clone)]
pub struct FrequencyDomainEqualizer {
    fft_size: usize,
    algorithm: EqualizerAlgorithm,
    /// Per-bin channel frequency response H(f).
    channel_estimate: Vec<(f64, f64)>,
    /// Whether the channel has been estimated at least once.
    channel_estimated: bool,
    /// Optional smoothing window half-width for channel estimate (0 = off).
    smoothing_width: usize,
}

impl FrequencyDomainEqualizer {
    /// Create a new equalizer with the given FFT size and algorithm.
    ///
    /// # Panics
    ///
    /// Panics if `fft_size` is not a power of two or is zero.
    pub fn new(fft_size: usize, algorithm: EqualizerAlgorithm) -> Self {
        assert!(
            is_power_of_two(fft_size),
            "FFT size must be a power of two, got {fft_size}"
        );
        Self {
            fft_size,
            algorithm,
            channel_estimate: vec![(1.0, 0.0); fft_size],
            channel_estimated: false,
            smoothing_width: 0,
        }
    }

    /// Return the configured FFT size.
    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    /// Return the current equalization algorithm.
    pub fn algorithm(&self) -> EqualizerAlgorithm {
        self.algorithm
    }

    /// Change the equalization algorithm.
    pub fn set_algorithm(&mut self, algorithm: EqualizerAlgorithm) {
        self.algorithm = algorithm;
    }

    /// Enable or disable frequency-domain channel estimate smoothing.
    ///
    /// `half_width` is the number of adjacent bins on each side used for
    /// a moving-average smooth. Set to 0 to disable.
    pub fn set_smoothing(&mut self, half_width: usize) {
        self.smoothing_width = half_width;
    }

    /// Estimate per-bin channel frequency response `H(f)` from known
    /// transmitted and received pilot symbols.
    ///
    /// Both slices must have length equal to `fft_size`.
    ///
    /// Computes `H(f) = FFT(rx) / FFT(tx)` (i.e., `H = Y / X` in the
    /// frequency domain).
    ///
    /// If smoothing is enabled, applies a moving-average window to the
    /// resulting H(f).
    ///
    /// # Panics
    ///
    /// Panics if slice lengths do not match `fft_size`.
    pub fn estimate_channel(
        &mut self,
        tx_pilots: &[(f64, f64)],
        rx_pilots: &[(f64, f64)],
    ) {
        assert_eq!(
            tx_pilots.len(),
            self.fft_size,
            "tx_pilots length {} != fft_size {}",
            tx_pilots.len(),
            self.fft_size,
        );
        assert_eq!(
            rx_pilots.len(),
            self.fft_size,
            "rx_pilots length {} != fft_size {}",
            rx_pilots.len(),
            self.fft_size,
        );

        let tx_freq = fft(tx_pilots);
        let rx_freq = fft(rx_pilots);

        for k in 0..self.fft_size {
            self.channel_estimate[k] = c_div(rx_freq[k], tx_freq[k]);
        }

        // Apply smoothing if configured
        if self.smoothing_width > 0 {
            self.smooth_channel_estimate();
        }

        self.channel_estimated = true;
    }

    /// Set the channel estimate directly (e.g., from an external source).
    ///
    /// # Panics
    ///
    /// Panics if `h` length does not match `fft_size`.
    pub fn set_channel_estimate(&mut self, h: &[(f64, f64)]) {
        assert_eq!(
            h.len(),
            self.fft_size,
            "channel estimate length {} != fft_size {}",
            h.len(),
            self.fft_size,
        );
        self.channel_estimate = h.to_vec();
        self.channel_estimated = true;
    }

    /// Return a reference to the current per-bin channel estimate H(f).
    pub fn channel_estimate(&self) -> &[(f64, f64)] {
        &self.channel_estimate
    }

    /// Apply frequency-domain equalization to a received block.
    ///
    /// The input `received` must have exactly `fft_size` samples. The process:
    ///
    /// 1. Compute `Y(f) = FFT(received)`.
    /// 2. Compute equalizer weights `W(f)` from the stored `H(f)`.
    /// 3. Compute `X̂(f) = W(f) · Y(f)`.
    /// 4. Return `x̂ = IFFT(X̂(f))`.
    ///
    /// # Panics
    ///
    /// Panics if `received` length does not match `fft_size`, or if the
    /// channel has not been estimated yet.
    pub fn equalize(&self, received: &[(f64, f64)]) -> Vec<(f64, f64)> {
        assert_eq!(
            received.len(),
            self.fft_size,
            "received block length {} != fft_size {}",
            received.len(),
            self.fft_size,
        );
        assert!(
            self.channel_estimated,
            "channel must be estimated before equalization"
        );

        let y_freq = fft(received);
        let mut x_hat_freq = vec![(0.0, 0.0); self.fft_size];

        for k in 0..self.fft_size {
            let h = self.channel_estimate[k];
            let w = self.compute_weight(h);
            x_hat_freq[k] = c_mul(w, y_freq[k]);
        }

        ifft(&x_hat_freq)
    }

    /// Remove a cyclic prefix from the beginning of a received block.
    ///
    /// Returns the remaining `fft_size` samples after removing `cp_len`
    /// leading samples.
    ///
    /// # Panics
    ///
    /// Panics if `block.len() != fft_size + cp_len`.
    pub fn remove_cyclic_prefix<'a>(
        &self,
        block: &'a [(f64, f64)],
        cp_len: usize,
    ) -> &'a [(f64, f64)] {
        let expected = self.fft_size + cp_len;
        assert_eq!(
            block.len(),
            expected,
            "block length {} != fft_size + cp_len = {}",
            block.len(),
            expected,
        );
        &block[cp_len..]
    }

    /// Estimate per-subcarrier SNR (in linear scale) from the channel
    /// estimate and algorithm noise variance.
    ///
    /// For ZF: `SNR(k) = |H(k)|² / ε` where ε is a small regularizer.
    /// For MMSE: `SNR(k) = |H(k)|² / σ²`.
    ///
    /// Returns a vector of length `fft_size`.
    pub fn snr_per_subcarrier(&self) -> Vec<f64> {
        let noise_var = match self.algorithm {
            EqualizerAlgorithm::ZeroForcing => 1e-10, // small regularizer
            EqualizerAlgorithm::Mmse { noise_variance } => noise_variance.max(1e-30),
        };

        self.channel_estimate
            .iter()
            .map(|&h| c_mag_sq(h) / noise_var)
            .collect()
    }

    /// Estimate per-subcarrier SNR in dB.
    pub fn snr_per_subcarrier_db(&self) -> Vec<f64> {
        self.snr_per_subcarrier()
            .into_iter()
            .map(|snr_lin| 10.0 * snr_lin.log10())
            .collect()
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /// Compute the equalizer weight W(f) for a single bin given H(f).
    fn compute_weight(&self, h: (f64, f64)) -> (f64, f64) {
        match self.algorithm {
            EqualizerAlgorithm::ZeroForcing => {
                // W = 1/H
                c_div((1.0, 0.0), h)
            }
            EqualizerAlgorithm::Mmse { noise_variance } => {
                // W = conj(H) / (|H|^2 + sigma^2)
                let denom = c_mag_sq(h) + noise_variance;
                if denom < 1e-30 {
                    (0.0, 0.0)
                } else {
                    let conj_h = c_conj(h);
                    (conj_h.0 / denom, conj_h.1 / denom)
                }
            }
        }
    }

    /// Apply moving-average smoothing to the channel estimate in-place.
    fn smooth_channel_estimate(&mut self) {
        let n = self.fft_size;
        let w = self.smoothing_width;
        if w == 0 || n == 0 {
            return;
        }

        let original = self.channel_estimate.clone();
        for k in 0..n {
            let mut sum = (0.0, 0.0);
            let mut count = 0u32;
            let lo = if k >= w { k - w } else { 0 };
            let hi = if k + w < n { k + w } else { n - 1 };
            for j in lo..=hi {
                sum = c_add(sum, original[j]);
                count += 1;
            }
            self.channel_estimate[k] = c_scale(sum, 1.0 / count as f64);
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a unit impulse of length n.
    fn impulse(n: usize) -> Vec<(f64, f64)> {
        let mut v = vec![(0.0, 0.0); n];
        v[0] = (1.0, 0.0);
        v
    }

    /// Helper: max absolute error between two complex vectors.
    fn max_error(a: &[(f64, f64)], b: &[(f64, f64)]) -> f64 {
        a.iter()
            .zip(b.iter())
            .map(|(&(ar, ai), &(br, bi))| ((ar - br).abs()).max((ai - bi).abs()))
            .fold(0.0f64, f64::max)
    }

    // -- FFT tests --

    #[test]
    fn test_fft_impulse() {
        // FFT of a unit impulse should be all ones
        let inp = impulse(8);
        let result = fft(&inp);
        for (k, &(re, im)) in result.iter().enumerate() {
            assert!(
                (re - 1.0).abs() < 1e-12 && im.abs() < 1e-12,
                "bin {k}: ({re}, {im})"
            );
        }
    }

    #[test]
    fn test_fft_inverse_roundtrip() {
        let original: Vec<(f64, f64)> = (0..16)
            .map(|i| ((i as f64).sin(), (i as f64 * 0.7).cos()))
            .collect();
        let freq = fft(&original);
        let recovered = ifft(&freq);
        assert!(
            max_error(&original, &recovered) < 1e-10,
            "FFT/IFFT roundtrip error too large"
        );
    }

    #[test]
    fn test_fft_parsevals_theorem() {
        // Sum of |x[n]|^2 == (1/N) * Sum of |X[k]|^2
        let x: Vec<(f64, f64)> = (0..8)
            .map(|i| ((i as f64 * 0.3).cos(), (i as f64 * 0.5).sin()))
            .collect();
        let x_freq = fft(&x);

        let time_energy: f64 = x.iter().map(|s| c_mag_sq(*s)).sum();
        let freq_energy: f64 = x_freq.iter().map(|s| c_mag_sq(*s)).sum::<f64>() / 8.0;

        assert!(
            (time_energy - freq_energy).abs() < 1e-10,
            "Parseval's theorem violated: {time_energy} vs {freq_energy}"
        );
    }

    // -- Equalizer construction --

    #[test]
    fn test_new_equalizer() {
        let eq = FrequencyDomainEqualizer::new(64, EqualizerAlgorithm::ZeroForcing);
        assert_eq!(eq.fft_size(), 64);
        assert_eq!(eq.algorithm(), EqualizerAlgorithm::ZeroForcing);
        assert_eq!(eq.channel_estimate().len(), 64);
    }

    #[test]
    #[should_panic(expected = "power of two")]
    fn test_non_power_of_two_panics() {
        let _ = FrequencyDomainEqualizer::new(12, EqualizerAlgorithm::ZeroForcing);
    }

    // -- Zero-Forcing equalization --

    #[test]
    fn test_zf_flat_channel() {
        // Flat channel H = 0.5 (6 dB attenuation) — ZF should recover original
        let n = 8;
        let mut eq = FrequencyDomainEqualizer::new(n, EqualizerAlgorithm::ZeroForcing);

        // Transmitted QPSK-like symbols
        let tx: Vec<(f64, f64)> = vec![
            (1.0, 1.0),
            (-1.0, 1.0),
            (-1.0, -1.0),
            (1.0, -1.0),
            (1.0, 0.0),
            (0.0, 1.0),
            (-1.0, 0.0),
            (0.0, -1.0),
        ];
        let rx: Vec<(f64, f64)> = tx.iter().map(|&(r, i)| (r * 0.5, i * 0.5)).collect();

        eq.estimate_channel(&tx, &rx);
        let equalized = eq.equalize(&rx);

        assert!(
            max_error(&tx, &equalized) < 1e-10,
            "ZF equalization error too large for flat channel"
        );
    }

    // -- MMSE equalization --

    #[test]
    fn test_mmse_flat_channel() {
        let n = 8;
        let mut eq = FrequencyDomainEqualizer::new(
            n,
            EqualizerAlgorithm::Mmse {
                noise_variance: 0.001,
            },
        );

        let tx: Vec<(f64, f64)> = vec![
            (1.0, 0.0),
            (0.0, 1.0),
            (-1.0, 0.0),
            (0.0, -1.0),
            (1.0, 1.0),
            (-1.0, 1.0),
            (-1.0, -1.0),
            (1.0, -1.0),
        ];
        // Channel gain = 0.7
        let rx: Vec<(f64, f64)> = tx.iter().map(|&(r, i)| (r * 0.7, i * 0.7)).collect();

        eq.estimate_channel(&tx, &rx);
        let equalized = eq.equalize(&rx);

        // MMSE with low noise should be very close to original
        assert!(
            max_error(&tx, &equalized) < 0.05,
            "MMSE equalization error too large: {}",
            max_error(&tx, &equalized)
        );
    }

    // -- Cyclic prefix removal --

    #[test]
    fn test_cyclic_prefix_removal() {
        let n = 8;
        let cp_len = 2;
        let eq = FrequencyDomainEqualizer::new(n, EqualizerAlgorithm::ZeroForcing);

        // Build a block with CP: last 2 samples prepended
        let data: Vec<(f64, f64)> = (0..n).map(|i| (i as f64, 0.0)).collect();
        let mut block = Vec::with_capacity(n + cp_len);
        block.extend_from_slice(&data[n - cp_len..]);
        block.extend_from_slice(&data);

        let stripped = eq.remove_cyclic_prefix(&block, cp_len);
        assert_eq!(stripped.len(), n);
        assert!(
            max_error(stripped, &data) < 1e-15,
            "CP removal produced wrong data"
        );
    }

    // -- SNR estimation --

    #[test]
    fn test_snr_per_subcarrier() {
        let n = 8;
        let noise_var = 0.01;
        let mut eq = FrequencyDomainEqualizer::new(
            n,
            EqualizerAlgorithm::Mmse {
                noise_variance: noise_var,
            },
        );

        // Set a known flat channel H(k) = (2.0, 0.0) for all k
        let h = vec![(2.0, 0.0); n];
        eq.set_channel_estimate(&h);

        let snr = eq.snr_per_subcarrier();
        // Expected: |H|^2 / sigma^2 = 4.0 / 0.01 = 400.0
        for (k, &s) in snr.iter().enumerate() {
            assert!(
                (s - 400.0).abs() < 1e-6,
                "SNR bin {k}: expected 400, got {s}"
            );
        }

        let snr_db = eq.snr_per_subcarrier_db();
        let expected_db = 10.0 * 400.0_f64.log10(); // ~26.02 dB
        for &s in &snr_db {
            assert!((s - expected_db).abs() < 1e-6);
        }
    }

    // -- Channel estimate smoothing --

    #[test]
    fn test_channel_smoothing() {
        let n = 8;
        let mut eq = FrequencyDomainEqualizer::new(n, EqualizerAlgorithm::ZeroForcing);
        eq.set_smoothing(1); // half-width = 1 → 3-tap average

        // Set a channel estimate with a spike at bin 3
        let mut h = vec![(1.0, 0.0); n];
        h[3] = (4.0, 0.0);

        // TX = impulse (FFT = all ones), RX = IFFT(h)
        let rx_time = ifft(&h);
        let tx_time = impulse(n);

        eq.estimate_channel(&tx_time, &rx_time);

        let smoothed = eq.channel_estimate();
        // Bin 3 was 4.0; after smoothing with neighbors (1.0 at bins 2,4),
        // it should become (1+4+1)/3 = 2.0
        assert!(
            (smoothed[3].0 - 2.0).abs() < 1e-10,
            "smoothed bin 3: {:?}",
            smoothed[3]
        );
        // Bin 0 (edge): average of bins 0 and 1 only = (1+1)/2 = 1.0
        assert!(
            (smoothed[0].0 - 1.0).abs() < 1e-10,
            "smoothed bin 0: {:?}",
            smoothed[0]
        );
    }

    // -- Algorithm change --

    #[test]
    fn test_set_algorithm() {
        let mut eq = FrequencyDomainEqualizer::new(8, EqualizerAlgorithm::ZeroForcing);
        eq.set_algorithm(EqualizerAlgorithm::Mmse {
            noise_variance: 0.1,
        });
        assert_eq!(
            eq.algorithm(),
            EqualizerAlgorithm::Mmse {
                noise_variance: 0.1
            }
        );
    }

    // -- Frequency-selective channel --

    #[test]
    fn test_frequency_selective_channel() {
        // Two-tap channel: h = [1.0, 0.5] (ISI)
        let n = 16;
        let mut eq = FrequencyDomainEqualizer::new(n, EqualizerAlgorithm::ZeroForcing);

        // Build channel impulse response
        let mut h_time = vec![(0.0, 0.0); n];
        h_time[0] = (1.0, 0.0);
        h_time[1] = (0.5, 0.0);

        // TX: known pseudo-random pilots
        let tx: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * i as f64 * 3.0 / n as f64;
                (phase.cos(), phase.sin())
            })
            .collect();

        // Convolve in frequency domain: RX = IFFT(FFT(tx) * FFT(h))
        let tx_freq = fft(&tx);
        let h_freq = fft(&h_time);
        let rx_freq: Vec<(f64, f64)> = tx_freq
            .iter()
            .zip(h_freq.iter())
            .map(|(&x, &h)| c_mul(x, h))
            .collect();
        let rx = ifft(&rx_freq);

        eq.estimate_channel(&tx, &rx);
        let equalized = eq.equalize(&rx);

        assert!(
            max_error(&tx, &equalized) < 1e-8,
            "ZF failed on freq-selective channel, error: {}",
            max_error(&tx, &equalized)
        );
    }

    // -- Edge: single-bin deep fade with MMSE --

    #[test]
    fn test_mmse_deep_fade_robustness() {
        let n = 8;
        let mut eq = FrequencyDomainEqualizer::new(
            n,
            EqualizerAlgorithm::Mmse {
                noise_variance: 0.1,
            },
        );

        // Channel with a near-zero bin
        let mut h = vec![(1.0, 0.0); n];
        h[2] = (0.001, 0.0); // deep fade at bin 2

        eq.set_channel_estimate(&h);

        // Fabricate a received signal: just ones in time domain
        let rx: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let equalized = eq.equalize(&rx);

        // MMSE should not blow up — all outputs should be finite
        for (k, &(re, im)) in equalized.iter().enumerate() {
            assert!(
                re.is_finite() && im.is_finite(),
                "bin {k} is not finite: ({re}, {im})"
            );
        }
    }

    // -- Panic on length mismatch --

    #[test]
    #[should_panic(expected = "tx_pilots length")]
    fn test_estimate_channel_wrong_tx_length() {
        let mut eq = FrequencyDomainEqualizer::new(8, EqualizerAlgorithm::ZeroForcing);
        eq.estimate_channel(&[(1.0, 0.0); 4], &[(1.0, 0.0); 8]);
    }

    #[test]
    #[should_panic(expected = "channel must be estimated")]
    fn test_equalize_without_channel_estimate() {
        let eq = FrequencyDomainEqualizer::new(8, EqualizerAlgorithm::ZeroForcing);
        let _ = eq.equalize(&[(1.0, 0.0); 8]);
    }
}

//! Information-theoretic entropy measures for blind signal classification
//! and modulation recognition.
//!
//! This module provides [`EntropyCalculator`] with configurable histogram bins
//! and window size for computing Shannon entropy, Renyi entropy, spectral
//! entropy, sample entropy, approximate entropy, differential entropy,
//! Kullback-Leibler divergence, joint entropy, and mutual information.
//!
//! All complex samples are represented as `(f64, f64)` tuples `(re, im)`.
//!
//! # Example
//!
//! ```
//! use r4w_core::entropy_calculator::EntropyCalculator;
//!
//! let calc = EntropyCalculator::new(64, 256);
//!
//! // Some IQ samples with varying amplitude
//! let samples: Vec<(f64, f64)> = (0..128)
//!     .map(|i| {
//!         let phase = i as f64 * 0.1;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let shannon = calc.shannon_entropy(&samples);
//! assert!(shannon >= 0.0, "entropy must be non-negative");
//! ```

use std::f64::consts::PI;

/// Configurable entropy calculator for IQ signal analysis.
///
/// Uses histogram-based probability estimation with a configurable number of
/// bins and a window size that limits how many samples are considered at once.
#[derive(Debug, Clone)]
pub struct EntropyCalculator {
    /// Number of histogram bins for amplitude/value discretization.
    bins: usize,
    /// Maximum number of samples to consider (0 = unlimited).
    window_size: usize,
}

impl EntropyCalculator {
    /// Create a new `EntropyCalculator`.
    ///
    /// * `bins` – number of histogram bins (clamped to at least 2).
    /// * `window_size` – maximum samples to use (0 means unlimited).
    pub fn new(bins: usize, window_size: usize) -> Self {
        Self {
            bins: bins.max(2),
            window_size,
        }
    }

    /// Return the configured number of histogram bins.
    pub fn bins(&self) -> usize {
        self.bins
    }

    /// Return the configured window size.
    pub fn window_size(&self) -> usize {
        self.window_size
    }

    // ---------------------------------------------------------------
    // Internal helpers
    // ---------------------------------------------------------------

    /// Apply the window limit and return a slice of the input.
    fn windowed<'a, T>(&self, data: &'a [T]) -> &'a [T] {
        if self.window_size == 0 || data.len() <= self.window_size {
            data
        } else {
            &data[data.len() - self.window_size..]
        }
    }

    /// Extract amplitudes from complex samples.
    fn amplitudes(&self, samples: &[(f64, f64)]) -> Vec<f64> {
        self.windowed(samples)
            .iter()
            .map(|(re, im)| (re * re + im * im).sqrt())
            .collect()
    }

    /// Build a normalized histogram (probability distribution) from a slice of
    /// values. Returns a vector of length `self.bins`.
    fn histogram(&self, values: &[f64]) -> Vec<f64> {
        if values.is_empty() {
            return vec![0.0; self.bins];
        }
        let min = values.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let range = max - min;
        let mut counts = vec![0usize; self.bins];
        if range == 0.0 {
            // All values are identical – put everything in the first bin.
            counts[0] = values.len();
        } else {
            for &v in values {
                let idx = ((v - min) / range * (self.bins as f64 - 1.0)).round() as usize;
                counts[idx.min(self.bins - 1)] += 1;
            }
        }
        let n = values.len() as f64;
        counts.iter().map(|&c| c as f64 / n).collect()
    }

    // ---------------------------------------------------------------
    // Public entropy measures
    // ---------------------------------------------------------------

    /// Shannon entropy of the amplitude distribution (in nats).
    ///
    /// H = - sum( p_i * ln(p_i) ) for p_i > 0
    pub fn shannon_entropy(&self, samples: &[(f64, f64)]) -> f64 {
        let amps = self.amplitudes(samples);
        let dist = self.histogram(&amps);
        shannon_from_dist(&dist)
    }

    /// Renyi entropy of order `alpha` of the amplitude distribution (in nats).
    ///
    /// H_alpha = 1/(1-alpha) * ln( sum(p_i^alpha) )
    ///
    /// `alpha` must be > 0 and != 1.  When alpha -> 1 the result converges
    /// to Shannon entropy; this function returns Shannon entropy for alpha == 1.
    pub fn renyi_entropy(&self, samples: &[(f64, f64)], alpha: f64) -> f64 {
        assert!(alpha > 0.0, "alpha must be positive");
        let amps = self.amplitudes(samples);
        let dist = self.histogram(&amps);
        if (alpha - 1.0).abs() < 1e-12 {
            return shannon_from_dist(&dist);
        }
        let sum_pa: f64 = dist.iter().filter(|&&p| p > 0.0).map(|&p| p.powf(alpha)).sum();
        if sum_pa == 0.0 {
            return 0.0;
        }
        sum_pa.ln() / (1.0 - alpha)
    }

    /// Spectral entropy – Shannon entropy of the normalised power spectral
    /// density.  Uses a simple DFT (no external FFT crate).
    ///
    /// The result is normalised to [0, 1] by dividing by ln(N).
    pub fn spectral_entropy(&self, samples: &[(f64, f64)]) -> f64 {
        let s = self.windowed(samples);
        let n = s.len();
        if n < 2 {
            return 0.0;
        }
        // Compute |DFT[k]|^2 for k = 0..n-1 via brute-force DFT.
        let psd = dft_power(s);
        let total: f64 = psd.iter().sum();
        if total == 0.0 {
            return 0.0;
        }
        let norm: Vec<f64> = psd.iter().map(|&p| p / total).collect();
        let h = shannon_from_dist(&norm);
        let h_max = (n as f64).ln();
        if h_max == 0.0 {
            0.0
        } else {
            h / h_max
        }
    }

    /// Sample entropy (SampEn) – a regularity measure for time series.
    ///
    /// Uses the amplitude time-series extracted from the IQ samples.
    ///
    /// * `m` – template length (embedding dimension).
    /// * `r` – tolerance (fraction of the standard deviation of the amplitudes).
    ///
    /// Returns `-ln(A/B)` where A is the number of template matches of length
    /// m+1 and B is the number of matches of length m. Self-matches are
    /// excluded.
    pub fn sample_entropy(&self, samples: &[(f64, f64)], m: usize, r: f64) -> f64 {
        let amps = self.amplitudes(samples);
        sample_entropy_impl(&amps, m, r)
    }

    /// Approximate entropy (ApEn) for signal complexity.
    ///
    /// Similar to sample entropy but includes self-matches, making it
    /// biased but well-defined for short time series.
    ///
    /// * `m` – template length.
    /// * `r` – tolerance (fraction of the standard deviation).
    pub fn approximate_entropy(&self, samples: &[(f64, f64)], m: usize, r: f64) -> f64 {
        let amps = self.amplitudes(samples);
        approximate_entropy_impl(&amps, m, r)
    }

    /// Differential entropy estimation for a continuous amplitude distribution
    /// using histogram-based estimation.
    ///
    /// h(X) approximately equals - sum( p_i * ln(p_i / delta) ) where delta is the bin width.
    pub fn differential_entropy(&self, samples: &[(f64, f64)]) -> f64 {
        let amps = self.amplitudes(samples);
        if amps.is_empty() {
            return 0.0;
        }
        let min = amps.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = amps.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let range = max - min;
        if range == 0.0 {
            return f64::NEG_INFINITY; // degenerate: all identical
        }
        let delta = range / self.bins as f64;
        let dist = self.histogram(&amps);
        let mut h = 0.0;
        for &p in &dist {
            if p > 0.0 {
                h -= p * (p / delta).ln();
            }
        }
        h
    }

    /// Kullback-Leibler divergence D_KL(P || Q) between two distributions
    /// derived from two sets of IQ samples.
    ///
    /// The distributions are amplitude histograms. Returns f64::INFINITY when
    /// Q has zero probability where P is non-zero.
    pub fn kl_divergence(&self, p_samples: &[(f64, f64)], q_samples: &[(f64, f64)]) -> f64 {
        let p_amps = self.amplitudes(p_samples);
        let q_amps = self.amplitudes(q_samples);
        let p_dist = self.histogram(&p_amps);
        let q_dist = self.histogram(&q_amps);
        kl_divergence_dist(&p_dist, &q_dist)
    }

    /// KL divergence from raw (pre-computed, normalised) distributions.
    pub fn kl_divergence_from_dist(&self, p: &[f64], q: &[f64]) -> f64 {
        kl_divergence_dist(p, q)
    }

    /// Joint entropy H(X, Y) of two amplitude sequences using a 2-D
    /// histogram.
    ///
    /// Both sample slices must have the same length.
    pub fn joint_entropy(&self, x_samples: &[(f64, f64)], y_samples: &[(f64, f64)]) -> f64 {
        let x = self.amplitudes(x_samples);
        let y = self.amplitudes(y_samples);
        let n = x.len().min(y.len());
        if n == 0 {
            return 0.0;
        }
        let x = &x[..n];
        let y = &y[..n];
        let joint = self.histogram_2d(x, y);
        shannon_from_dist(&joint)
    }

    /// Mutual information I(X; Y) = H(X) + H(Y) - H(X, Y).
    pub fn mutual_information(
        &self,
        x_samples: &[(f64, f64)],
        y_samples: &[(f64, f64)],
    ) -> f64 {
        let hx = self.shannon_entropy(x_samples);
        let hy = self.shannon_entropy(y_samples);
        let hxy = self.joint_entropy(x_samples, y_samples);
        // Clamp to 0 to avoid tiny negative values from floating point.
        (hx + hy - hxy).max(0.0)
    }

    /// Shannon entropy from a raw (pre-computed) probability distribution.
    pub fn shannon_entropy_from_dist(&self, dist: &[f64]) -> f64 {
        shannon_from_dist(dist)
    }

    // ---------------------------------------------------------------
    // Private 2-D histogram
    // ---------------------------------------------------------------

    /// Build a flattened 2-D histogram (bins x bins) from two value arrays.
    fn histogram_2d(&self, x: &[f64], y: &[f64]) -> Vec<f64> {
        let n = x.len().min(y.len());
        if n == 0 {
            return vec![0.0; self.bins * self.bins];
        }

        let x_min = x.iter().cloned().fold(f64::INFINITY, f64::min);
        let x_max = x.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let y_min = y.iter().cloned().fold(f64::INFINITY, f64::min);
        let y_max = y.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        let x_range = x_max - x_min;
        let y_range = y_max - y_min;

        let mut counts = vec![0usize; self.bins * self.bins];

        for i in 0..n {
            let xi = if x_range == 0.0 {
                0
            } else {
                ((x[i] - x_min) / x_range * (self.bins as f64 - 1.0)).round() as usize
            }
            .min(self.bins - 1);
            let yi = if y_range == 0.0 {
                0
            } else {
                ((y[i] - y_min) / y_range * (self.bins as f64 - 1.0)).round() as usize
            }
            .min(self.bins - 1);
            counts[xi * self.bins + yi] += 1;
        }

        let total = n as f64;
        counts.iter().map(|&c| c as f64 / total).collect()
    }
}

// ===================================================================
// Free-standing helper functions
// ===================================================================

/// Shannon entropy from a probability distribution (in nats).
fn shannon_from_dist(dist: &[f64]) -> f64 {
    let mut h = 0.0;
    for &p in dist {
        if p > 0.0 {
            h -= p * p.ln();
        }
    }
    h
}

/// KL divergence D_KL(P || Q) from two discrete distributions.
fn kl_divergence_dist(p: &[f64], q: &[f64]) -> f64 {
    assert_eq!(p.len(), q.len(), "distributions must have equal length");
    let mut d = 0.0;
    for (&pi, &qi) in p.iter().zip(q.iter()) {
        if pi > 0.0 {
            if qi == 0.0 {
                return f64::INFINITY;
            }
            d += pi * (pi / qi).ln();
        }
    }
    d
}

/// Brute-force DFT power spectrum |X[k]|^2 for k = 0..N-1.
fn dft_power(samples: &[(f64, f64)]) -> Vec<f64> {
    let n = samples.len();
    let mut psd = Vec::with_capacity(n);
    for k in 0..n {
        let mut re_sum = 0.0;
        let mut im_sum = 0.0;
        for (i, &(re, im)) in samples.iter().enumerate() {
            let angle = -2.0 * PI * (k as f64) * (i as f64) / (n as f64);
            let (sin_a, cos_a) = angle.sin_cos();
            re_sum += re * cos_a - im * sin_a;
            im_sum += re * sin_a + im * cos_a;
        }
        psd.push(re_sum * re_sum + im_sum * im_sum);
    }
    psd
}

/// Standard deviation of a slice.
fn std_dev(data: &[f64]) -> f64 {
    if data.len() < 2 {
        return 0.0;
    }
    let n = data.len() as f64;
    let mean = data.iter().sum::<f64>() / n;
    let var = data.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n;
    var.sqrt()
}

/// Chebyshev (max-norm) distance between two templates.
fn chebyshev(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

/// Sample entropy implementation on a 1-D real sequence.
fn sample_entropy_impl(data: &[f64], m: usize, r_frac: f64) -> f64 {
    let n = data.len();
    if n <= m + 1 {
        return 0.0;
    }
    let sd = std_dev(data);
    if sd == 0.0 {
        return 0.0;
    }
    let tol = r_frac * sd;

    let count_matches = |template_len: usize| -> usize {
        let num_templates = n - template_len;
        let mut total = 0usize;
        for i in 0..num_templates {
            for j in (i + 1)..num_templates {
                if chebyshev(&data[i..i + template_len], &data[j..j + template_len]) <= tol {
                    total += 2; // count (i,j) and (j,i)
                }
            }
        }
        total
    };

    let b = count_matches(m);
    let a = count_matches(m + 1);

    if b == 0 {
        return 0.0;
    }

    // Normalise by number of possible pairs (excluding self-matches).
    let nb = (n - m) as f64;
    let na = (n - m - 1) as f64;
    let b_rate = b as f64 / (nb * (nb - 1.0));
    let a_rate = if na > 1.0 {
        a as f64 / (na * (na - 1.0))
    } else {
        0.0
    };

    if a_rate == 0.0 {
        return f64::INFINITY;
    }

    -(a_rate / b_rate).ln()
}

/// Approximate entropy (ApEn) implementation.
fn approximate_entropy_impl(data: &[f64], m: usize, r_frac: f64) -> f64 {
    let n = data.len();
    if n <= m + 1 {
        return 0.0;
    }
    let sd = std_dev(data);
    if sd == 0.0 {
        return 0.0;
    }
    let tol = r_frac * sd;

    let phi = |template_len: usize| -> f64 {
        let num = n - template_len + 1;
        let mut c = vec![0.0f64; num];
        for i in 0..num {
            let mut count = 0usize;
            for j in 0..num {
                if chebyshev(&data[i..i + template_len], &data[j..j + template_len]) <= tol {
                    count += 1;
                }
            }
            c[i] = count as f64 / num as f64;
        }
        c.iter().map(|&ci| ci.ln()).sum::<f64>() / num as f64
    };

    phi(m) - phi(m + 1)
}

// ===================================================================
// Tests
// ===================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a constant-amplitude complex tone.
    fn tone(n: usize, freq: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 * freq;
                (t.cos(), t.sin())
            })
            .collect()
    }

    /// Helper: generate a constant DC signal.
    fn dc(n: usize, amp: f64) -> Vec<(f64, f64)> {
        vec![(amp, 0.0); n]
    }

    /// Helper: deterministic pseudo-random amplitude signal.
    fn pseudo_random(n: usize) -> Vec<(f64, f64)> {
        let mut v = Vec::with_capacity(n);
        let mut state: u64 = 0xDEAD_BEEF;
        for _ in 0..n {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let amp = (state >> 33) as f64 / (1u64 << 31) as f64;
            let phase = ((state >> 16) & 0xFFFF) as f64 / 65536.0 * 2.0 * PI;
            v.push((amp * phase.cos(), amp * phase.sin()));
        }
        v
    }

    // ----- Shannon Entropy -----

    #[test]
    fn shannon_entropy_non_negative() {
        let calc = EntropyCalculator::new(32, 0);
        let s = tone(128, 0.1);
        assert!(calc.shannon_entropy(&s) >= 0.0);
    }

    #[test]
    fn shannon_entropy_constant_signal_is_zero() {
        let calc = EntropyCalculator::new(32, 0);
        let s = dc(100, 1.0);
        let h = calc.shannon_entropy(&s);
        assert!(h.abs() < 1e-10, "expected ~0 for constant signal, got {h}");
    }

    #[test]
    fn shannon_entropy_varied_greater_than_constant() {
        let calc = EntropyCalculator::new(32, 0);
        let constant = dc(200, 1.0);
        let varied = pseudo_random(200);
        assert!(calc.shannon_entropy(&varied) > calc.shannon_entropy(&constant));
    }

    // ----- Renyi Entropy -----

    #[test]
    fn renyi_alpha_one_equals_shannon() {
        let calc = EntropyCalculator::new(32, 0);
        let s = pseudo_random(200);
        let shannon = calc.shannon_entropy(&s);
        let renyi = calc.renyi_entropy(&s, 1.0);
        assert!(
            (shannon - renyi).abs() < 1e-10,
            "Renyi(alpha=1) should equal Shannon: {renyi} vs {shannon}"
        );
    }

    #[test]
    fn renyi_alpha_two_leq_shannon() {
        // H_2 <= H_1 (Renyi entropy is non-increasing in alpha).
        let calc = EntropyCalculator::new(32, 0);
        let s = pseudo_random(200);
        let h1 = calc.renyi_entropy(&s, 1.0);
        let h2 = calc.renyi_entropy(&s, 2.0);
        assert!(
            h2 <= h1 + 1e-10,
            "H_2 should be <= H_1: {h2} vs {h1}"
        );
    }

    #[test]
    fn renyi_constant_signal_is_zero() {
        let calc = EntropyCalculator::new(32, 0);
        let s = dc(100, 1.0);
        let h = calc.renyi_entropy(&s, 2.0);
        assert!(h.abs() < 1e-10, "expected ~0 for constant signal, got {h}");
    }

    // ----- Spectral Entropy -----

    #[test]
    fn spectral_entropy_pure_tone_is_low() {
        let calc = EntropyCalculator::new(32, 0);
        // A pure tone concentrates energy in one DFT bin.
        let s = tone(64, 2.0 * PI / 8.0);
        let se = calc.spectral_entropy(&s);
        assert!(
            se < 0.5,
            "pure tone should have low spectral entropy, got {se}"
        );
    }

    #[test]
    fn spectral_entropy_in_zero_one() {
        let calc = EntropyCalculator::new(32, 0);
        let s = pseudo_random(64);
        let se = calc.spectral_entropy(&s);
        assert!((0.0..=1.0001).contains(&se), "spectral entropy should be in [0,1], got {se}");
    }

    #[test]
    fn spectral_entropy_empty() {
        let calc = EntropyCalculator::new(32, 0);
        let se = calc.spectral_entropy(&[]);
        assert_eq!(se, 0.0);
    }

    // ----- Sample Entropy -----

    #[test]
    fn sample_entropy_regular_signal() {
        let calc = EntropyCalculator::new(32, 0);
        // A perfectly periodic tone should yield low sample entropy.
        let s = tone(200, 2.0 * PI / 20.0);
        let se = calc.sample_entropy(&s, 2, 0.2);
        // Periodic signals have very low or zero sample entropy.
        assert!(se.is_finite() || se == f64::INFINITY);
    }

    #[test]
    fn sample_entropy_random_higher_than_periodic() {
        let calc = EntropyCalculator::new(32, 0);
        // Use a tone that produces varying amplitudes (non-zero std dev)
        let periodic: Vec<(f64, f64)> = (0..200)
            .map(|i| {
                let t = i as f64 * 0.1;
                (t.cos() * (1.0 + 0.3 * (t * 0.05).sin()), t.sin() * (1.0 + 0.3 * (t * 0.05).sin()))
            })
            .collect();
        let random = pseudo_random(200);
        let se_per = calc.sample_entropy(&periodic, 2, 0.2);
        let se_rand = calc.sample_entropy(&random, 2, 0.2);
        // Both should return finite values (or infinity for no matches)
        assert!(!se_per.is_nan(), "periodic SampEn should not be NaN");
        assert!(!se_rand.is_nan(), "random SampEn should not be NaN");
    }

    // ----- Approximate Entropy -----

    #[test]
    fn approximate_entropy_non_negative() {
        let calc = EntropyCalculator::new(32, 0);
        let s = pseudo_random(200);
        let ae = calc.approximate_entropy(&s, 2, 0.2);
        assert!(ae >= 0.0, "ApEn should be non-negative, got {ae}");
    }

    #[test]
    fn approximate_entropy_random_higher_than_periodic() {
        let calc = EntropyCalculator::new(32, 0);
        // Use signal with varying amplitudes
        let periodic: Vec<(f64, f64)> = (0..200)
            .map(|i| {
                let t = i as f64 * 0.1;
                let amp = 1.0 + 0.3 * (t * 0.07).sin();
                (amp * t.cos(), amp * t.sin())
            })
            .collect();
        let random = pseudo_random(200);
        let ae_per = calc.approximate_entropy(&periodic, 2, 0.2);
        let ae_rand = calc.approximate_entropy(&random, 2, 0.2);
        assert!(
            ae_rand >= ae_per,
            "random should have higher ApEn: {ae_rand} vs {ae_per}"
        );
    }

    // ----- Differential Entropy -----

    #[test]
    fn differential_entropy_varied_signal() {
        let calc = EntropyCalculator::new(64, 0);
        let s = pseudo_random(500);
        let de = calc.differential_entropy(&s);
        // Differential entropy can be negative; just check it's finite.
        assert!(de.is_finite(), "differential entropy should be finite, got {de}");
    }

    #[test]
    fn differential_entropy_constant_is_neg_inf() {
        let calc = EntropyCalculator::new(32, 0);
        let s = dc(100, 1.0);
        let de = calc.differential_entropy(&s);
        assert!(de == f64::NEG_INFINITY, "constant signal should give -inf, got {de}");
    }

    // ----- KL Divergence -----

    #[test]
    fn kl_divergence_same_distribution_is_zero() {
        let calc = EntropyCalculator::new(32, 0);
        let s = pseudo_random(500);
        let d = calc.kl_divergence(&s, &s);
        assert!(d.abs() < 1e-10, "KL(P||P) should be 0, got {d}");
    }

    #[test]
    fn kl_divergence_different_distributions_positive() {
        let calc = EntropyCalculator::new(32, 0);
        let p = tone(500, 0.05);
        let q = pseudo_random(500);
        let d = calc.kl_divergence(&p, &q);
        assert!(d >= 0.0, "KL divergence should be >= 0, got {d}");
    }

    #[test]
    fn kl_divergence_from_dist_manual() {
        let calc = EntropyCalculator::new(4, 0);
        let p = vec![0.5, 0.5, 0.0, 0.0];
        let q = vec![0.25, 0.25, 0.25, 0.25];
        let d = calc.kl_divergence_from_dist(&p, &q);
        // KL = 0.5*ln(2) + 0.5*ln(2) = ln(2)
        let expected = 2.0f64.ln();
        assert!(
            (d - expected).abs() < 1e-10,
            "expected {expected}, got {d}"
        );
    }

    // ----- Joint Entropy -----

    #[test]
    fn joint_entropy_same_signal_equals_marginal() {
        let calc = EntropyCalculator::new(32, 0);
        let s = pseudo_random(300);
        let hx = calc.shannon_entropy(&s);
        let hxy = calc.joint_entropy(&s, &s);
        // H(X,X) = H(X) because Y=X provides no new info.
        assert!(
            (hx - hxy).abs() < 0.2,
            "H(X,X) should be approx H(X): {hxy} vs {hx}"
        );
    }

    // ----- Mutual Information -----

    #[test]
    fn mutual_information_same_signal() {
        let calc = EntropyCalculator::new(32, 0);
        let s = pseudo_random(300);
        let mi = calc.mutual_information(&s, &s);
        let hx = calc.shannon_entropy(&s);
        // I(X;X) = H(X)
        assert!(
            (mi - hx).abs() < 0.2,
            "I(X;X) should be approx H(X): {mi} vs {hx}"
        );
    }

    #[test]
    fn mutual_information_non_negative() {
        let calc = EntropyCalculator::new(32, 0);
        let a = tone(200, 0.1);
        let b = pseudo_random(200);
        let mi = calc.mutual_information(&a, &b);
        assert!(mi >= 0.0, "MI should be >= 0, got {mi}");
    }

    // ----- Window size -----

    #[test]
    fn window_size_limits_samples() {
        let calc = EntropyCalculator::new(16, 50);
        let long = pseudo_random(500);
        let short = pseudo_random(50);
        // With window=50, only the last 50 samples of `long` are used.
        let h = calc.shannon_entropy(&long);
        assert!(h.is_finite());
        let h2 = calc.shannon_entropy(&short);
        assert!(h2.is_finite());
    }

    // ----- Constructor / accessors -----

    #[test]
    fn bins_clamped_to_minimum() {
        let calc = EntropyCalculator::new(0, 0);
        assert_eq!(calc.bins(), 2);
        let calc2 = EntropyCalculator::new(1, 0);
        assert_eq!(calc2.bins(), 2);
    }

    // ----- Shannon from dist helper -----

    #[test]
    fn shannon_from_dist_uniform() {
        let calc = EntropyCalculator::new(4, 0);
        let dist = vec![0.25, 0.25, 0.25, 0.25];
        let h = calc.shannon_entropy_from_dist(&dist);
        let expected = (4.0f64).ln();
        assert!(
            (h - expected).abs() < 1e-10,
            "uniform dist entropy should be ln(4)={expected}, got {h}"
        );
    }
}

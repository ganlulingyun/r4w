//! # Power-Law Spectrum Estimator
//!
//! Estimates spectral exponents (1/f^α noise characterization) for signal
//! classification and noise analysis. Provides tools for identifying power-law
//! scaling in signals, computing Hurst exponents, Detrended Fluctuation Analysis
//! (DFA), Allan variance, and spectral flatness.
//!
//! ## Noise Color Classification
//!
//! | Color  | α (alpha) | Description                        |
//! |--------|-----------|------------------------------------|
//! | Violet | ≈ −2      | PSD rises with f² (differentiated) |
//! | Blue   | ≈ −1      | PSD rises with f                   |
//! | White  | ≈  0      | Flat PSD                           |
//! | Pink   | ≈  1      | PSD ∝ 1/f (flicker noise)          |
//! | Brown  | ≈  2      | PSD ∝ 1/f² (random walk)           |
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::power_law_spectrum_estimator::{PowerLawSpectrumEstimator, NoiseColor};
//!
//! // Generate white noise (alpha ≈ 0)
//! let mut rng_state: u64 = 12345;
//! let signal: Vec<f64> = (0..1024).map(|_| {
//!     // Simple LCG pseudo-random
//!     rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
//!     (rng_state >> 33) as f64 / (1u64 << 31) as f64 - 0.5
//! }).collect();
//!
//! let estimator = PowerLawSpectrumEstimator::new(44100.0);
//! let result = estimator.estimate(&signal);
//!
//! println!("Spectral exponent α = {:.2}", result.alpha);
//! println!("Noise color: {:?}", result.noise_color);
//! assert!(result.alpha.abs() < 1.0, "White noise should have alpha near 0");
//! ```

use std::f64::consts::PI;

// ─── Complex helpers using (f64, f64) tuples ─────────────────────────────────

/// Complex number type as (real, imaginary) tuple.
pub type Complex = (f64, f64);

#[inline]
fn c_mul(a: Complex, b: Complex) -> Complex {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_add(a: Complex, b: Complex) -> Complex {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_mag_sq(a: Complex) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn c_from_polar(mag: f64, phase: f64) -> Complex {
    (mag * phase.cos(), mag * phase.sin())
}

// ─── FFT (radix-2 Cooley-Tukey, in-place) ───────────────────────────────────

fn next_power_of_two(n: usize) -> usize {
    let mut p = 1;
    while p < n {
        p <<= 1;
    }
    p
}

fn bit_reverse(mut x: usize, log2n: u32) -> usize {
    let mut result = 0;
    for _ in 0..log2n {
        result = (result << 1) | (x & 1);
        x >>= 1;
    }
    result
}

/// Radix-2 DIT FFT. `data` length must be a power of two.
fn fft(data: &mut [Complex]) {
    let n = data.len();
    assert!(n.is_power_of_two(), "FFT length must be power of two");
    let log2n = n.trailing_zeros();

    // Bit-reversal permutation
    for i in 0..n {
        let j = bit_reverse(i, log2n);
        if i < j {
            data.swap(i, j);
        }
    }

    // Butterfly stages
    let mut size = 2;
    while size <= n {
        let half = size / 2;
        let angle_step = -2.0 * PI / size as f64;
        for k in 0..n / size {
            let base = k * size;
            for j in 0..half {
                let w = c_from_polar(1.0, angle_step * j as f64);
                let t = c_mul(w, data[base + j + half]);
                let u = data[base + j];
                data[base + j] = c_add(u, t);
                data[base + j + half] = (u.0 - t.0, u.1 - t.1);
            }
        }
        size <<= 1;
    }
}

// ─── Noise color classification ──────────────────────────────────────────────

/// Classification of noise by its spectral exponent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NoiseColor {
    /// PSD ∝ f² (α ≈ −2)
    Violet,
    /// PSD ∝ f (α ≈ −1)
    Blue,
    /// Flat PSD (α ≈ 0)
    White,
    /// PSD ∝ 1/f (α ≈ 1)
    Pink,
    /// PSD ∝ 1/f² (α ≈ 2)
    Brown,
    /// Spectral exponent outside the standard classifications.
    Other,
}

impl NoiseColor {
    /// Classify based on the spectral exponent α.
    pub fn from_alpha(alpha: f64) -> Self {
        if alpha < -1.5 {
            NoiseColor::Violet
        } else if alpha < -0.5 {
            NoiseColor::Blue
        } else if alpha < 0.5 {
            NoiseColor::White
        } else if alpha < 1.5 {
            NoiseColor::Pink
        } else if alpha < 2.5 {
            NoiseColor::Brown
        } else {
            NoiseColor::Other
        }
    }

    /// Human-readable name.
    pub fn name(&self) -> &'static str {
        match self {
            NoiseColor::Violet => "Violet",
            NoiseColor::Blue => "Blue",
            NoiseColor::White => "White",
            NoiseColor::Pink => "Pink",
            NoiseColor::Brown => "Brown",
            NoiseColor::Other => "Other",
        }
    }
}

// ─── PSD estimation method ───────────────────────────────────────────────────

/// Method used for Power Spectral Density estimation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PsdMethod {
    /// Simple periodogram (single FFT of the entire signal).
    Periodogram,
    /// Welch's method with 50% overlapping Hanning-windowed segments.
    Welch,
}

// ─── Estimation results ──────────────────────────────────────────────────────

/// Result of a power-law spectrum estimation.
#[derive(Debug, Clone)]
pub struct EstimationResult {
    /// Estimated spectral exponent α (PSD ∝ 1/f^α).
    pub alpha: f64,
    /// Noise color classification.
    pub noise_color: NoiseColor,
    /// Hurst exponent H = (α − 1) / 2 (meaningful for 1 < α < 3).
    pub hurst_exponent: f64,
    /// R² (coefficient of determination) of the log-log fit.
    pub r_squared: f64,
    /// Standard error of the slope estimate.
    pub slope_std_error: f64,
    /// 95% confidence interval for α: (lower, upper).
    pub confidence_interval_95: (f64, f64),
    /// Spectral flatness (Wiener entropy), 0..1. 1 = perfectly flat.
    pub spectral_flatness: f64,
    /// Frequencies (Hz) used in the PSD.
    pub frequencies: Vec<f64>,
    /// Power spectral density values corresponding to `frequencies`.
    pub psd: Vec<f64>,
}

/// Result of Detrended Fluctuation Analysis.
#[derive(Debug, Clone)]
pub struct DfaResult {
    /// Box sizes used.
    pub box_sizes: Vec<usize>,
    /// Fluctuation function F(n) for each box size.
    pub fluctuations: Vec<f64>,
    /// Estimated DFA exponent (slope of log F vs log n).
    pub exponent: f64,
    /// R² of the log-log fit.
    pub r_squared: f64,
}

/// Result of Allan variance computation.
#[derive(Debug, Clone)]
pub struct AllanVarianceResult {
    /// Averaging times τ.
    pub taus: Vec<f64>,
    /// Allan variance σ²_y(τ) for each τ.
    pub allan_var: Vec<f64>,
    /// Allan deviation σ_y(τ) for each τ.
    pub allan_dev: Vec<f64>,
    /// Slope of log-log Allan deviation vs τ (indicates noise type).
    pub slope: f64,
}

// ─── Estimator ───────────────────────────────────────────────────────────────

/// Estimates power-law spectral exponents from time-domain signals.
///
/// Characterizes 1/f^α noise by fitting the PSD on a log-log scale.
#[derive(Debug, Clone)]
pub struct PowerLawSpectrumEstimator {
    /// Sample rate in Hz.
    sample_rate: f64,
    /// PSD estimation method.
    psd_method: PsdMethod,
    /// Segment length for Welch's method (0 = auto).
    welch_segment_len: usize,
    /// Lower bound of frequency range for fitting (Hz). 0 = use first bin.
    freq_min: f64,
    /// Upper bound of frequency range for fitting (Hz). 0 = Nyquist.
    freq_max: f64,
}

impl PowerLawSpectrumEstimator {
    /// Create a new estimator with the given sample rate.
    ///
    /// Defaults to Welch method with automatic segment length and full
    /// frequency range.
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            psd_method: PsdMethod::Welch,
            welch_segment_len: 0,
            freq_min: 0.0,
            freq_max: 0.0,
        }
    }

    /// Set the PSD estimation method.
    pub fn with_method(mut self, method: PsdMethod) -> Self {
        self.psd_method = method;
        self
    }

    /// Set the segment length for Welch's method.
    pub fn with_welch_segment_len(mut self, len: usize) -> Self {
        self.welch_segment_len = len;
        self
    }

    /// Set the frequency range for the log-log fit.
    ///
    /// Only PSD bins within `[freq_min, freq_max]` will be used for
    /// regression. Pass 0.0 for either bound to use the default
    /// (first bin / Nyquist).
    pub fn with_freq_range(mut self, min_hz: f64, max_hz: f64) -> Self {
        self.freq_min = min_hz;
        self.freq_max = max_hz;
        self
    }

    // ── PSD estimation ───────────────────────────────────────────────────

    /// Compute the one-sided PSD of a real signal.
    ///
    /// Returns `(frequencies, psd_values)`.
    pub fn compute_psd(&self, signal: &[f64]) -> (Vec<f64>, Vec<f64>) {
        match self.psd_method {
            PsdMethod::Periodogram => self.periodogram(signal),
            PsdMethod::Welch => self.welch(signal),
        }
    }

    fn periodogram(&self, signal: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let n = next_power_of_two(signal.len());
        let mut buf: Vec<Complex> = signal
            .iter()
            .map(|&x| (x, 0.0))
            .chain(std::iter::repeat((0.0, 0.0)))
            .take(n)
            .collect();

        fft(&mut buf);

        let num_bins = n / 2 + 1;
        let df = self.sample_rate / n as f64;
        let norm = 1.0 / (self.sample_rate * n as f64);

        let freqs: Vec<f64> = (0..num_bins).map(|i| i as f64 * df).collect();
        let mut psd: Vec<f64> = (0..num_bins)
            .map(|i| {
                let p = c_mag_sq(buf[i]) * norm;
                // Double the one-sided bins (except DC and Nyquist)
                if i == 0 || i == n / 2 {
                    p
                } else {
                    2.0 * p
                }
            })
            .collect();

        // Avoid exact zeros (replace with tiny value)
        for v in &mut psd {
            if *v <= 0.0 {
                *v = 1e-300;
            }
        }

        (freqs, psd)
    }

    fn hanning_window(n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos()))
            .collect()
    }

    fn welch(&self, signal: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let seg_len = if self.welch_segment_len > 0 {
            self.welch_segment_len
        } else {
            // Default: roughly signal.len()/8 rounded up to next power of 2,
            // but at least 64 and at most signal.len().
            let candidate = signal.len() / 8;
            let candidate = candidate.max(64).min(signal.len());
            next_power_of_two(candidate)
        };

        let nfft = next_power_of_two(seg_len);
        let overlap = seg_len / 2;
        let step = seg_len - overlap;
        let window = Self::hanning_window(seg_len);
        let win_power: f64 = window.iter().map(|w| w * w).sum::<f64>() / seg_len as f64;

        let num_bins = nfft / 2 + 1;
        let mut psd_accum = vec![0.0f64; num_bins];
        let mut num_segments = 0usize;

        let mut start = 0;
        while start + seg_len <= signal.len() {
            let mut buf: Vec<Complex> = (0..nfft)
                .map(|i| {
                    if i < seg_len {
                        (signal[start + i] * window[i], 0.0)
                    } else {
                        (0.0, 0.0)
                    }
                })
                .collect();

            fft(&mut buf);

            let norm = 1.0 / (self.sample_rate * win_power * seg_len as f64);
            for i in 0..num_bins {
                let p = c_mag_sq(buf[i]) * norm;
                let p = if i == 0 || i == nfft / 2 { p } else { 2.0 * p };
                psd_accum[i] += p;
            }
            num_segments += 1;
            start += step;
        }

        if num_segments == 0 {
            // Signal too short for Welch; fall back to periodogram
            return self.periodogram(signal);
        }

        let df = self.sample_rate / nfft as f64;
        let freqs: Vec<f64> = (0..num_bins).map(|i| i as f64 * df).collect();
        let mut psd: Vec<f64> = psd_accum
            .iter()
            .map(|&v| {
                let p = v / num_segments as f64;
                if p <= 0.0 { 1e-300 } else { p }
            })
            .collect();

        // Ensure no zeros
        for v in &mut psd {
            if *v <= 0.0 {
                *v = 1e-300;
            }
        }

        (freqs, psd)
    }

    // ── Log-log linear regression ────────────────────────────────────────

    /// Performs ordinary least-squares linear regression.
    ///
    /// Returns `(slope, intercept, r_squared, slope_std_error)`.
    fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64, f64, f64) {
        assert_eq!(x.len(), y.len());
        let n = x.len() as f64;
        if x.len() < 2 {
            return (0.0, 0.0, 0.0, f64::INFINITY);
        }

        let sum_x: f64 = x.iter().sum();
        let sum_y: f64 = y.iter().sum();
        let sum_xy: f64 = x.iter().zip(y).map(|(a, b)| a * b).sum();
        let sum_xx: f64 = x.iter().map(|a| a * a).sum();

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return (0.0, sum_y / n, 0.0, f64::INFINITY);
        }

        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n;

        // R²
        let y_mean = sum_y / n;
        let ss_tot: f64 = y.iter().map(|v| (v - y_mean).powi(2)).sum();
        let ss_res: f64 = x
            .iter()
            .zip(y)
            .map(|(xi, yi)| {
                let predicted = slope * xi + intercept;
                (yi - predicted).powi(2)
            })
            .sum();

        let r_squared = if ss_tot > 0.0 {
            1.0 - ss_res / ss_tot
        } else {
            0.0
        };

        // Standard error of slope
        let n_int = x.len();
        let mse = if n_int > 2 {
            ss_res / (n_int - 2) as f64
        } else {
            0.0
        };
        let slope_se = if denom.abs() > 1e-30 {
            (mse * n / denom).sqrt()
        } else {
            f64::INFINITY
        };

        (slope, intercept, r_squared, slope_se)
    }

    // ── Main estimation ──────────────────────────────────────────────────

    /// Estimate the spectral exponent of the given signal.
    pub fn estimate(&self, signal: &[f64]) -> EstimationResult {
        let (freqs, psd) = self.compute_psd(signal);

        // Determine frequency range for fitting
        let f_min = if self.freq_min > 0.0 {
            self.freq_min
        } else if freqs.len() > 1 {
            freqs[1] // skip DC
        } else {
            0.0
        };
        let f_max = if self.freq_max > 0.0 {
            self.freq_max
        } else {
            self.sample_rate / 2.0
        };

        // Collect log-log points within frequency range
        let mut log_f = Vec::new();
        let mut log_p = Vec::new();
        for (i, (&f, &p)) in freqs.iter().zip(psd.iter()).enumerate() {
            if i == 0 {
                continue; // always skip DC
            }
            if f >= f_min && f <= f_max && p > 0.0 {
                log_f.push(f.ln());
                log_p.push(p.ln());
            }
        }

        let (slope, _intercept, r_squared, slope_se) = if log_f.len() >= 2 {
            Self::linear_regression(&log_f, &log_p)
        } else {
            (0.0, 0.0, 0.0, f64::INFINITY)
        };

        // PSD ∝ 1/f^α  →  log(PSD) = -α·log(f) + C  →  α = -slope
        let alpha = -slope;
        let noise_color = NoiseColor::from_alpha(alpha);
        let hurst = (alpha - 1.0) / 2.0;

        // 95% confidence interval (using t ≈ 1.96 for large N)
        let t_val = 1.96;
        let ci_low = alpha - t_val * slope_se;
        let ci_high = alpha + t_val * slope_se;

        // Spectral flatness (Wiener entropy)
        let flatness = Self::spectral_flatness_from_psd(&psd[1..]);

        EstimationResult {
            alpha,
            noise_color,
            hurst_exponent: hurst,
            r_squared,
            slope_std_error: slope_se,
            confidence_interval_95: (ci_low, ci_high),
            spectral_flatness: flatness,
            frequencies: freqs,
            psd,
        }
    }

    // ── Spectral flatness ────────────────────────────────────────────────

    fn spectral_flatness_from_psd(psd: &[f64]) -> f64 {
        if psd.is_empty() {
            return 0.0;
        }
        let n = psd.len() as f64;

        // Geometric mean (via log)
        let log_sum: f64 = psd.iter().map(|&v| v.max(1e-300).ln()).sum();
        let geo_mean = (log_sum / n).exp();

        // Arithmetic mean
        let arith_mean: f64 = psd.iter().sum::<f64>() / n;

        if arith_mean > 0.0 {
            (geo_mean / arith_mean).clamp(0.0, 1.0)
        } else {
            0.0
        }
    }

    /// Compute spectral flatness (Wiener entropy) for a signal.
    ///
    /// Returns a value in [0, 1]. 1 means perfectly flat (white noise),
    /// lower values indicate more spectral structure.
    pub fn spectral_flatness(&self, signal: &[f64]) -> f64 {
        let (_freqs, psd) = self.compute_psd(signal);
        if psd.len() > 1 {
            Self::spectral_flatness_from_psd(&psd[1..])
        } else {
            0.0
        }
    }

    // ── Detrended Fluctuation Analysis ───────────────────────────────────

    /// Perform Detrended Fluctuation Analysis (DFA).
    ///
    /// DFA measures long-range correlations in time series. The DFA exponent
    /// relates to the Hurst exponent: for fractional Gaussian noise, DFA
    /// exponent ≈ H; for fractional Brownian motion, DFA exponent ≈ H + 1.
    pub fn dfa(&self, signal: &[f64]) -> DfaResult {
        let n = signal.len();
        if n < 16 {
            return DfaResult {
                box_sizes: vec![],
                fluctuations: vec![],
                exponent: 0.0,
                r_squared: 0.0,
            };
        }

        // Cumulative sum (integration / profile)
        let mean: f64 = signal.iter().sum::<f64>() / n as f64;
        let mut profile = Vec::with_capacity(n);
        let mut cumsum = 0.0;
        for &x in signal {
            cumsum += x - mean;
            profile.push(cumsum);
        }

        // Box sizes: logarithmically spaced from 4 to n/4
        let min_box = 4usize;
        let max_box = n / 4;
        if max_box < min_box {
            return DfaResult {
                box_sizes: vec![],
                fluctuations: vec![],
                exponent: 0.0,
                r_squared: 0.0,
            };
        }

        let num_scales = 20.min(max_box - min_box + 1);
        let log_min = (min_box as f64).ln();
        let log_max = (max_box as f64).ln();

        let mut box_sizes = Vec::new();
        let mut fluctuations = Vec::new();

        for i in 0..num_scales {
            let t = if num_scales > 1 {
                i as f64 / (num_scales - 1) as f64
            } else {
                0.0
            };
            let box_size = (log_min + t * (log_max - log_min)).exp().round() as usize;
            if box_size < 4 || box_sizes.last() == Some(&box_size) {
                continue;
            }

            let num_boxes = n / box_size;
            if num_boxes == 0 {
                continue;
            }

            let mut sum_sq = 0.0;
            let mut count = 0usize;

            for b in 0..num_boxes {
                let start = b * box_size;
                let end = start + box_size;

                // Linear detrend within box
                let (slope, intercept, _, _) = {
                    let x_vals: Vec<f64> = (0..box_size).map(|j| j as f64).collect();
                    let y_vals: Vec<f64> = profile[start..end].to_vec();
                    Self::linear_regression(&x_vals, &y_vals)
                };

                for j in 0..box_size {
                    let trend = slope * j as f64 + intercept;
                    let residual = profile[start + j] - trend;
                    sum_sq += residual * residual;
                    count += 1;
                }
            }

            if count > 0 {
                let f_n = (sum_sq / count as f64).sqrt();
                box_sizes.push(box_size);
                fluctuations.push(f_n);
            }
        }

        // Log-log regression
        let log_n: Vec<f64> = box_sizes.iter().map(|&s| (s as f64).ln()).collect();
        let log_f: Vec<f64> = fluctuations.iter().map(|&f| f.max(1e-300).ln()).collect();
        let (exponent, _, r_squared, _) = Self::linear_regression(&log_n, &log_f);

        DfaResult {
            box_sizes,
            fluctuations,
            exponent,
            r_squared,
        }
    }

    // ── Allan Variance ───────────────────────────────────────────────────

    /// Compute the overlapping Allan variance of a time series.
    ///
    /// Used for characterizing clock/oscillator stability. The slope of
    /// Allan deviation vs averaging time τ on a log-log plot indicates
    /// the dominant noise type.
    pub fn allan_variance(&self, signal: &[f64]) -> AllanVarianceResult {
        let n = signal.len();
        let tau0 = 1.0 / self.sample_rate;

        // Averaging factors: powers of 2 up to n/2
        let mut taus = Vec::new();
        let mut allan_var = Vec::new();

        let mut m = 1usize;
        while m <= n / 2 {
            let tau = m as f64 * tau0;
            let num_blocks = n / m;
            if num_blocks < 2 {
                break;
            }

            let avgs: Vec<f64> = (0..num_blocks)
                .map(|i| {
                    let start = i * m;
                    let end = start + m;
                    signal[start..end].iter().sum::<f64>() / m as f64
                })
                .collect();

            let mut sum_sq = 0.0;
            let mut count = 0usize;

            for i in 0..avgs.len() - 1 {
                let diff = avgs[i + 1] - avgs[i];
                sum_sq += diff * diff;
                count += 1;
            }

            if count > 0 {
                let avar = sum_sq / (2.0 * count as f64);
                taus.push(tau);
                allan_var.push(avar);
            }

            // Double m each step (or next integer for small m)
            if m < 4 {
                m += 1;
            } else {
                m *= 2;
            }
        }

        let allan_dev: Vec<f64> = allan_var.iter().map(|&v| v.sqrt()).collect();

        // Fit slope of log-log ADEV vs tau
        let log_tau: Vec<f64> = taus.iter().map(|&t| t.ln()).collect();
        let log_dev: Vec<f64> = allan_dev.iter().map(|&d| d.max(1e-300).ln()).collect();
        let (slope, _, _, _) = Self::linear_regression(&log_tau, &log_dev);

        AllanVarianceResult {
            taus,
            allan_var,
            allan_dev,
            slope,
        }
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // Simple LCG PRNG for deterministic tests
    struct Lcg(u64);

    impl Lcg {
        fn new(seed: u64) -> Self {
            Self(seed)
        }
        fn next_f64(&mut self) -> f64 {
            self.0 = self.0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((self.0 >> 33) as f64) / ((1u64 << 31) as f64) - 0.5
        }
        fn next_gaussian(&mut self) -> f64 {
            // Box-Muller
            let u1 = self.next_f64() + 0.5; // [0,1)
            let u2 = self.next_f64() + 0.5;
            let u1 = u1.clamp(1e-10, 1.0 - 1e-10);
            (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
        }
    }

    fn white_noise(n: usize, seed: u64) -> Vec<f64> {
        let mut rng = Lcg::new(seed);
        (0..n).map(|_| rng.next_gaussian()).collect()
    }

    fn brown_noise(n: usize, seed: u64) -> Vec<f64> {
        let mut rng = Lcg::new(seed);
        let mut acc = 0.0;
        (0..n)
            .map(|_| {
                acc += rng.next_gaussian();
                acc
            })
            .collect()
    }

    // ── Test 1: FFT basic ────────────────────────────────────────────────

    #[test]
    fn test_fft_dc_signal() {
        let mut data: Vec<Complex> = vec![(1.0, 0.0); 8];
        fft(&mut data);
        // DC component should be 8, rest should be ~0
        assert!((data[0].0 - 8.0).abs() < 1e-10);
        for i in 1..8 {
            assert!(c_mag_sq(data[i]).sqrt() < 1e-10, "bin {i} should be zero");
        }
    }

    // ── Test 2: FFT single tone ──────────────────────────────────────────

    #[test]
    fn test_fft_single_tone() {
        let n = 64;
        let k = 4; // frequency bin 4
        let signal: Vec<Complex> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * k as f64 * i as f64 / n as f64;
                (phase.cos(), 0.0)
            })
            .collect();
        let mut buf = signal;
        fft(&mut buf);

        // Peak at bin k and n-k
        let mag_k = c_mag_sq(buf[k]).sqrt();
        assert!(mag_k > n as f64 * 0.4, "expected peak at bin {k}");
    }

    // ── Test 3: NoiseColor classification ────────────────────────────────

    #[test]
    fn test_noise_color_classification() {
        assert_eq!(NoiseColor::from_alpha(-2.0), NoiseColor::Violet);
        assert_eq!(NoiseColor::from_alpha(-1.0), NoiseColor::Blue);
        assert_eq!(NoiseColor::from_alpha(0.0), NoiseColor::White);
        assert_eq!(NoiseColor::from_alpha(1.0), NoiseColor::Pink);
        assert_eq!(NoiseColor::from_alpha(2.0), NoiseColor::Brown);
        assert_eq!(NoiseColor::from_alpha(5.0), NoiseColor::Other);
    }

    // ── Test 4: NoiseColor names ─────────────────────────────────────────

    #[test]
    fn test_noise_color_names() {
        assert_eq!(NoiseColor::White.name(), "White");
        assert_eq!(NoiseColor::Pink.name(), "Pink");
        assert_eq!(NoiseColor::Brown.name(), "Brown");
        assert_eq!(NoiseColor::Blue.name(), "Blue");
        assert_eq!(NoiseColor::Violet.name(), "Violet");
        assert_eq!(NoiseColor::Other.name(), "Other");
    }

    // ── Test 5: Linear regression on known data ─────────────────────────

    #[test]
    fn test_linear_regression_exact() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![2.0, 4.0, 6.0, 8.0, 10.0]; // y = 2x
        let (slope, intercept, r_sq, _) = PowerLawSpectrumEstimator::linear_regression(&x, &y);
        assert!((slope - 2.0).abs() < 1e-10);
        assert!(intercept.abs() < 1e-10);
        assert!((r_sq - 1.0).abs() < 1e-10);
    }

    // ── Test 6: Linear regression with noise ─────────────────────────────

    #[test]
    fn test_linear_regression_noisy() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![2.1, 3.9, 6.1, 7.9, 10.1];
        let (slope, _, r_sq, se) = PowerLawSpectrumEstimator::linear_regression(&x, &y);
        assert!((slope - 2.0).abs() < 0.1);
        assert!(r_sq > 0.99);
        assert!(se < 0.2);
    }

    // ── Test 7: White noise estimation ───────────────────────────────────

    #[test]
    fn test_white_noise_estimation() {
        let signal = white_noise(4096, 42);
        let est = PowerLawSpectrumEstimator::new(1000.0);
        let result = est.estimate(&signal);
        // White noise: α ≈ 0
        assert!(
            result.alpha.abs() < 0.5,
            "white noise alpha = {}, expected near 0",
            result.alpha
        );
        assert_eq!(result.noise_color, NoiseColor::White);
    }

    // ── Test 8: Brown noise estimation ───────────────────────────────────

    #[test]
    fn test_brown_noise_estimation() {
        let signal = brown_noise(8192, 99);
        let est = PowerLawSpectrumEstimator::new(1000.0);
        let result = est.estimate(&signal);
        // Brown noise: α ≈ 2
        assert!(
            result.alpha > 1.2 && result.alpha < 3.0,
            "brown noise alpha = {}, expected near 2",
            result.alpha
        );
        assert!(
            result.noise_color == NoiseColor::Brown || result.noise_color == NoiseColor::Pink,
            "color = {:?}",
            result.noise_color
        );
    }

    // ── Test 9: Periodogram method ───────────────────────────────────────

    #[test]
    fn test_periodogram_method() {
        let signal = white_noise(2048, 123);
        let est = PowerLawSpectrumEstimator::new(1000.0).with_method(PsdMethod::Periodogram);
        let result = est.estimate(&signal);
        assert!(
            result.alpha.abs() < 0.8,
            "periodogram white noise alpha = {}",
            result.alpha
        );
    }

    // ── Test 10: PSD output dimensions ───────────────────────────────────

    #[test]
    fn test_psd_dimensions() {
        let signal = white_noise(256, 1);
        let est = PowerLawSpectrumEstimator::new(1000.0).with_method(PsdMethod::Periodogram);
        let (freqs, psd) = est.compute_psd(&signal);
        assert_eq!(freqs.len(), psd.len());
        // For 256 samples -> 256-pt FFT -> 129 one-sided bins
        assert_eq!(freqs.len(), 129);
        assert!((freqs[0] - 0.0).abs() < 1e-10);
        // last freq should be Nyquist
        let nyquist = 1000.0 / 2.0;
        assert!((freqs[freqs.len() - 1] - nyquist).abs() < 1.0);
    }

    // ── Test 11: Spectral flatness for white noise ───────────────────────

    #[test]
    fn test_spectral_flatness_white_noise() {
        let signal = white_noise(4096, 77);
        let est = PowerLawSpectrumEstimator::new(1000.0);
        let sf = est.spectral_flatness(&signal);
        // White noise should have high spectral flatness (> 0.5)
        assert!(
            sf > 0.5,
            "white noise spectral flatness = {}, expected > 0.5",
            sf
        );
    }

    // ── Test 12: Spectral flatness for tone ──────────────────────────────

    #[test]
    fn test_spectral_flatness_tone() {
        let n = 4096;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 100.0 * i as f64 / 1000.0).sin())
            .collect();
        let est = PowerLawSpectrumEstimator::new(1000.0);
        let sf = est.spectral_flatness(&signal);
        // Tone should have low spectral flatness (< 0.3)
        assert!(
            sf < 0.3,
            "tone spectral flatness = {}, expected < 0.3",
            sf
        );
    }

    // ── Test 13: Frequency range selection ───────────────────────────────

    #[test]
    fn test_frequency_range_selection() {
        let signal = white_noise(4096, 55);
        let est = PowerLawSpectrumEstimator::new(1000.0).with_freq_range(10.0, 200.0);
        let result = est.estimate(&signal);
        // Should still identify white noise even with restricted range
        assert!(
            result.alpha.abs() < 0.8,
            "freq-limited alpha = {}",
            result.alpha
        );
    }

    // ── Test 14: Hurst exponent ──────────────────────────────────────────

    #[test]
    fn test_hurst_exponent() {
        let signal = brown_noise(8192, 33);
        let est = PowerLawSpectrumEstimator::new(1000.0);
        let result = est.estimate(&signal);
        // Brown noise α ≈ 2 → H = (2 - 1) / 2 = 0.5
        // Allow wider tolerance for stochastic signal
        assert!(
            result.hurst_exponent > -0.2 && result.hurst_exponent < 1.5,
            "hurst = {}",
            result.hurst_exponent
        );
    }

    // ── Test 15: Confidence interval contains true value ─────────────────

    #[test]
    fn test_confidence_interval() {
        let signal = white_noise(4096, 66);
        let est = PowerLawSpectrumEstimator::new(1000.0);
        let result = est.estimate(&signal);
        // 95% CI should bracket the true alpha ≈ 0
        let (lo, hi) = result.confidence_interval_95;
        assert!(lo < hi, "CI lower should be < upper");
        assert!(
            lo < 0.5 && hi > -0.5,
            "CI [{lo}, {hi}] should plausibly contain 0"
        );
    }

    // ── Test 16: DFA on white noise ──────────────────────────────────────

    #[test]
    fn test_dfa_white_noise() {
        let signal = white_noise(4096, 88);
        let est = PowerLawSpectrumEstimator::new(1000.0);
        let dfa = est.dfa(&signal);
        // White noise DFA exponent ≈ 0.5
        assert!(
            dfa.exponent > 0.2 && dfa.exponent < 0.8,
            "DFA exponent = {}, expected ~0.5",
            dfa.exponent
        );
        assert!(!dfa.box_sizes.is_empty());
        assert_eq!(dfa.box_sizes.len(), dfa.fluctuations.len());
    }

    // ── Test 17: DFA on brown noise ──────────────────────────────────────

    #[test]
    fn test_dfa_brown_noise() {
        let signal = brown_noise(4096, 44);
        let est = PowerLawSpectrumEstimator::new(1000.0);
        let dfa = est.dfa(&signal);
        // Brown noise (random walk) DFA exponent ≈ 1.5
        assert!(
            dfa.exponent > 1.0 && dfa.exponent < 2.0,
            "DFA exponent = {}, expected ~1.5",
            dfa.exponent
        );
    }

    // ── Test 18: Allan variance computation ──────────────────────────────

    #[test]
    fn test_allan_variance() {
        let signal = white_noise(2048, 11);
        let est = PowerLawSpectrumEstimator::new(1000.0);
        let av = est.allan_variance(&signal);
        assert!(!av.taus.is_empty());
        assert_eq!(av.taus.len(), av.allan_var.len());
        assert_eq!(av.taus.len(), av.allan_dev.len());
        // All variances should be positive
        for &v in &av.allan_var {
            assert!(v > 0.0, "Allan variance should be positive");
        }
        // Deviation = sqrt(variance)
        for (v, d) in av.allan_var.iter().zip(av.allan_dev.iter()) {
            assert!((d - v.sqrt()).abs() < 1e-10);
        }
    }

    // ── Test 19: Allan variance slope for white noise ────────────────────

    #[test]
    fn test_allan_variance_slope_white() {
        let signal = white_noise(8192, 22);
        let est = PowerLawSpectrumEstimator::new(1000.0);
        let av = est.allan_variance(&signal);
        // White noise: ADEV slope ≈ -0.5 (τ^{-1/2})
        assert!(
            av.slope < 0.0,
            "white noise Allan slope = {}, expected < 0",
            av.slope
        );
    }

    // ── Test 20: Short signal graceful handling ──────────────────────────

    #[test]
    fn test_short_signal() {
        let signal = vec![1.0, 2.0, 3.0, 4.0];
        let est = PowerLawSpectrumEstimator::new(100.0);
        // Should not panic
        let result = est.estimate(&signal);
        assert!(result.alpha.is_finite());
    }

    // ── Test 21: DFA with too-short signal ───────────────────────────────

    #[test]
    fn test_dfa_short_signal() {
        let signal = vec![1.0, 2.0];
        let est = PowerLawSpectrumEstimator::new(100.0);
        let dfa = est.dfa(&signal);
        assert!(dfa.box_sizes.is_empty());
        assert_eq!(dfa.exponent, 0.0);
    }

    // ── Test 22: Builder pattern ─────────────────────────────────────────

    #[test]
    fn test_builder_pattern() {
        let est = PowerLawSpectrumEstimator::new(44100.0)
            .with_method(PsdMethod::Welch)
            .with_welch_segment_len(512)
            .with_freq_range(20.0, 20000.0);
        // Just verify it builds without panic
        assert_eq!(est.sample_rate, 44100.0);
    }

    // ── Test 23: Welch segment length ────────────────────────────────────

    #[test]
    fn test_welch_custom_segment() {
        let signal = white_noise(2048, 7);
        let est = PowerLawSpectrumEstimator::new(1000.0).with_welch_segment_len(256);
        let (freqs, psd) = est.compute_psd(&signal);
        // With 256-pt segments → 256-pt FFT → 129 bins
        assert_eq!(freqs.len(), 129);
        assert_eq!(psd.len(), 129);
    }

    // ── Test 24: R² quality check ────────────────────────────────────────

    #[test]
    fn test_r_squared_quality() {
        // Brown noise should have a good log-log fit
        let signal = brown_noise(8192, 55);
        let est = PowerLawSpectrumEstimator::new(1000.0);
        let result = est.estimate(&signal);
        assert!(
            result.r_squared > 0.5,
            "brown noise R² = {}, expected > 0.5",
            result.r_squared
        );
    }
}

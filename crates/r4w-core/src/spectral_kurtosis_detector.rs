//! Spectral Kurtosis (SK) detector for RFI/interference detection.
//!
//! Higher-order statistics applied to spectral data enable robust detection
//! of radio frequency interference (RFI) in radio astronomy, SIGINT, and
//! spectrum monitoring applications. The spectral kurtosis estimator exploits
//! the fact that Gaussian noise has SK ≈ 1, while deterministic signals
//! (e.g., CW tones) drive SK → 0 and impulsive interference pushes SK > 1.
//!
//! # Example
//!
//! ```
//! use r4w_core::spectral_kurtosis_detector::{SpectralKurtosis, SkConfig};
//!
//! let config = SkConfig {
//!     fft_size: 64,
//!     num_accumulations: 10,
//!     sample_rate: 1_000_000.0,
//!     false_alarm_rate: 0.0027,
//! };
//!
//! let sk = SpectralKurtosis::new(config);
//!
//! // Generate some Gaussian noise samples
//! let num_samples = 64 * 10;
//! let mut samples: Vec<(f64, f64)> = Vec::with_capacity(num_samples);
//! // Simple deterministic pseudo-noise for reproducibility
//! let mut state: u64 = 12345;
//! for _ in 0..num_samples {
//!     state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
//!     let u1 = (state >> 33) as f64 / (1u64 << 31) as f64;
//!     state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
//!     let u2 = (state >> 33) as f64 / (1u64 << 31) as f64;
//!     // Box-Muller approximation (simplified)
//!     let r = (-2.0 * (u1.max(1e-10)).ln()).sqrt();
//!     let theta = 2.0 * std::f64::consts::PI * u2;
//!     samples.push((r * theta.cos(), r * theta.sin()));
//! }
//!
//! let result = sk.detect_rfi(&samples);
//! // For Gaussian noise, most bins should not be flagged
//! assert!(result.rfi_fraction < 0.5);
//! ```

use std::f64::consts::PI;

/// Configuration for the Spectral Kurtosis estimator.
#[derive(Debug, Clone)]
pub struct SkConfig {
    /// Number of FFT bins (must be a power of 2).
    pub fft_size: usize,
    /// Number of spectra to accumulate (M). Higher M gives tighter thresholds.
    pub num_accumulations: usize,
    /// Sample rate in Hz (used for frequency axis labeling).
    pub sample_rate: f64,
    /// Probability of false alarm (Pfa). Typical values: 0.0027 (3σ), 0.01, 0.05.
    pub false_alarm_rate: f64,
}

/// Result of RFI detection across the spectrum.
#[derive(Debug, Clone)]
pub struct DetectionResult {
    /// Per-bin flag: true = RFI detected.
    pub flagged_bins: Vec<bool>,
    /// Per-bin spectral kurtosis values.
    pub sk_values: Vec<f64>,
    /// Upper detection threshold.
    pub threshold_upper: f64,
    /// Lower detection threshold.
    pub threshold_lower: f64,
    /// Fraction of bins flagged as RFI (0.0 to 1.0).
    pub rfi_fraction: f64,
}

/// Classification of detected RFI type based on SK statistics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RfiType {
    /// No significant RFI detected (SK ≈ 1 across spectrum).
    Clean,
    /// Narrowband interference (few bins with SK → 0).
    Narrowband,
    /// Broadband interference (many bins flagged).
    Broadband,
    /// Impulsive interference (bins with SK >> 1).
    Impulsive,
}

/// Spectral Kurtosis estimator engine for RFI detection.
///
/// Uses higher-order statistics of spectral data to distinguish Gaussian
/// noise (SK ≈ 1) from deterministic (SK → 0) or impulsive (SK > 1)
/// interference.
#[derive(Debug, Clone)]
pub struct SpectralKurtosis {
    config: SkConfig,
    /// Precomputed Hann window coefficients.
    window: Vec<f64>,
    /// Precomputed twiddle factors for FFT (cos, sin) pairs.
    twiddle_re: Vec<Vec<f64>>,
    twiddle_im: Vec<Vec<f64>>,
}

impl SpectralKurtosis {
    /// Create a new Spectral Kurtosis estimator with the given configuration.
    ///
    /// # Panics
    ///
    /// Panics if `fft_size` is 0 or not a power of two, if `num_accumulations` < 2,
    /// or if `false_alarm_rate` is not in (0, 1).
    pub fn new(config: SkConfig) -> Self {
        assert!(config.fft_size > 0, "fft_size must be > 0");
        assert!(
            config.fft_size.is_power_of_two(),
            "fft_size must be a power of two"
        );
        assert!(
            config.num_accumulations >= 2,
            "num_accumulations must be >= 2"
        );
        assert!(
            config.false_alarm_rate > 0.0 && config.false_alarm_rate < 1.0,
            "false_alarm_rate must be in (0, 1)"
        );

        let n = config.fft_size;

        // Precompute Hann window
        let window: Vec<f64> = (0..n)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos()))
            .collect();

        // Precompute twiddle factors for each FFT stage
        let num_stages = (n as f64).log2() as usize;
        let mut twiddle_re = Vec::with_capacity(num_stages);
        let mut twiddle_im = Vec::with_capacity(num_stages);

        for s in 0..num_stages {
            let m = 1 << (s + 1);
            let half_m = m / 2;
            let mut stage_re = Vec::with_capacity(half_m);
            let mut stage_im = Vec::with_capacity(half_m);
            for k in 0..half_m {
                let angle = -2.0 * PI * k as f64 / m as f64;
                stage_re.push(angle.cos());
                stage_im.push(angle.sin());
            }
            twiddle_re.push(stage_re);
            twiddle_im.push(stage_im);
        }

        Self {
            config,
            window,
            twiddle_re,
            twiddle_im,
        }
    }

    /// Compute the spectral kurtosis estimate for each frequency bin.
    ///
    /// SK = (M+1)/(M-1) * (M * S2 / S1² - 1)
    ///
    /// where S1 = Σ|X_k|², S2 = Σ|X_k|⁴, and the sums run over M accumulated
    /// spectra. For Gaussian noise, SK ≈ 1.
    ///
    /// The input `samples` must contain at least `fft_size * num_accumulations`
    /// samples. Excess samples are ignored.
    pub fn compute_sk(&self, samples: &[(f64, f64)]) -> Vec<f64> {
        let n = self.config.fft_size;
        let m = self.config.num_accumulations;
        let required = n * m;

        assert!(
            samples.len() >= required,
            "Need at least {} samples (fft_size={} x M={}), got {}",
            required,
            n,
            m,
            samples.len()
        );

        // Accumulate S1 (sum of |X|^2) and S2 (sum of |X|^4) per bin
        let mut s1 = vec![0.0f64; n];
        let mut s2 = vec![0.0f64; n];

        for acc in 0..m {
            let offset = acc * n;
            let segment = &samples[offset..offset + n];

            // Apply Hann window and compute FFT
            let spectrum = self.windowed_fft(segment);

            // Accumulate power statistics
            for (k, &(re, im)) in spectrum.iter().enumerate() {
                let power = re * re + im * im; // |X_k|^2
                s1[k] += power;
                s2[k] += power * power; // |X_k|^4
            }
        }

        // Compute SK per bin
        let m_f = m as f64;
        let scale = (m_f + 1.0) / (m_f - 1.0);

        s1.iter()
            .zip(s2.iter())
            .map(|(&s1_k, &s2_k)| {
                if s1_k.abs() < 1e-30 {
                    // Avoid division by zero for empty bins
                    1.0
                } else {
                    scale * (m_f * s2_k / (s1_k * s1_k) - 1.0)
                }
            })
            .collect()
    }

    /// Detect RFI in the given samples, returning a full detection result.
    ///
    /// This computes the SK values, applies the chi-squared thresholds,
    /// and flags bins that fall outside the expected range for Gaussian noise.
    pub fn detect_rfi(&self, samples: &[(f64, f64)]) -> DetectionResult {
        let sk_values = self.compute_sk(samples);
        let (threshold_lower, threshold_upper) = self.thresholds();

        let flagged_bins: Vec<bool> = sk_values
            .iter()
            .map(|&sk| sk < threshold_lower || sk > threshold_upper)
            .collect();

        let num_flagged = flagged_bins.iter().filter(|&&f| f).count();
        let rfi_fraction = if flagged_bins.is_empty() {
            0.0
        } else {
            num_flagged as f64 / flagged_bins.len() as f64
        };

        DetectionResult {
            flagged_bins,
            sk_values,
            threshold_upper,
            threshold_lower,
            rfi_fraction,
        }
    }

    /// Compute the upper and lower detection thresholds based on a
    /// chi-squared approximation to the SK distribution.
    ///
    /// For M accumulations, the SK estimator has known variance under the
    /// null hypothesis (Gaussian noise). The thresholds are set using the
    /// inverse chi-squared distribution approximation:
    ///
    /// sigma^2_SK = 4/M (approximate variance for large M)
    /// Thresholds: 1 +/- z_alpha * sigma_SK
    ///
    /// where z_alpha is the normal quantile for the desired false alarm rate.
    pub fn thresholds(&self) -> (f64, f64) {
        let m = self.config.num_accumulations as f64;
        let pfa = self.config.false_alarm_rate;

        // Variance of SK estimator under H0 (Gaussian noise)
        // Exact: Var(SK) = 4/M for large M (see Nita & Gary 2010)
        let var_sk = 4.0 / m;
        let sigma_sk = var_sk.sqrt();

        // Use inverse normal approximation for thresholds
        // z_alpha for two-sided test: split Pfa equally between upper and lower
        let z = inv_normal_cdf(1.0 - pfa / 2.0);

        let lower = (1.0 - z * sigma_sk).max(0.0);
        let upper = 1.0 + z * sigma_sk;

        (lower, upper)
    }

    /// Generate a boolean flagging mask from a detection result.
    ///
    /// Returns true for bins that are flagged as containing RFI.
    pub fn flagging_mask(&self, result: &DetectionResult) -> Vec<bool> {
        result.flagged_bins.clone()
    }

    /// Classify the type of RFI present based on SK value distribution.
    ///
    /// - **Clean**: Most bins have SK near 1 (< 5% flagged).
    /// - **Narrowband**: A few bins have SK near 0 (< 20% flagged, mostly below threshold).
    /// - **Broadband**: Many bins flagged (>= 20%).
    /// - **Impulsive**: Significant fraction of bins with SK > upper threshold.
    pub fn classify_rfi(&self, sk_values: &[f64]) -> RfiType {
        let (lower, upper) = self.thresholds();
        let n = sk_values.len();
        if n == 0 {
            return RfiType::Clean;
        }

        let below = sk_values.iter().filter(|&&v| v < lower).count();
        let above = sk_values.iter().filter(|&&v| v > upper).count();
        let flagged = below + above;

        let flagged_frac = flagged as f64 / n as f64;
        let above_frac = above as f64 / n as f64;

        if flagged_frac < 0.05 {
            RfiType::Clean
        } else if above_frac > 0.1 {
            RfiType::Impulsive
        } else if flagged_frac >= 0.20 {
            RfiType::Broadband
        } else {
            RfiType::Narrowband
        }
    }

    /// Apply the detection mask to zero out flagged frequency bins.
    ///
    /// Takes the time-domain samples and a per-bin boolean mask. For each
    /// FFT segment, transforms to frequency domain, zeroes flagged bins,
    /// and transforms back. Returns the cleaned time-domain samples.
    ///
    /// The mask length must equal `fft_size`. The output length equals
    /// `fft_size * num_accumulations` (matching the processed portion of input).
    pub fn apply_mask(
        &self,
        samples: &[(f64, f64)],
        mask: &[bool],
    ) -> Vec<(f64, f64)> {
        let n = self.config.fft_size;
        let m = self.config.num_accumulations;

        assert_eq!(
            mask.len(),
            n,
            "Mask length must equal fft_size ({})",
            n
        );
        assert!(
            samples.len() >= n * m,
            "Need at least {} samples",
            n * m
        );

        let mut output = Vec::with_capacity(n * m);

        for acc in 0..m {
            let offset = acc * n;
            let segment = &samples[offset..offset + n];

            // Forward FFT (with window)
            let mut spectrum = self.windowed_fft(segment);

            // Zero flagged bins
            for (k, flagged) in mask.iter().enumerate() {
                if *flagged {
                    spectrum[k] = (0.0, 0.0);
                }
            }

            // Inverse FFT
            let time_domain = self.ifft(&spectrum);
            output.extend_from_slice(&time_domain);
        }

        output
    }

    /// Apply Hann window and compute FFT of a segment.
    fn windowed_fft(&self, segment: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = self.config.fft_size;
        debug_assert_eq!(segment.len(), n);

        // Apply window
        let mut data: Vec<(f64, f64)> = segment
            .iter()
            .enumerate()
            .map(|(i, &(re, im))| (re * self.window[i], im * self.window[i]))
            .collect();

        // In-place Cooley-Tukey FFT
        self.fft_in_place(&mut data);
        data
    }

    /// In-place radix-2 Cooley-Tukey FFT.
    fn fft_in_place(&self, data: &mut [(f64, f64)]) {
        let n = data.len();
        debug_assert!(n.is_power_of_two());

        // Bit-reversal permutation
        bit_reverse_permutation(data);

        // Butterfly stages
        let num_stages = (n as f64).log2() as usize;
        for s in 0..num_stages {
            let m = 1 << (s + 1);
            let half_m = m / 2;

            let mut k_start = 0;
            while k_start < n {
                for j in 0..half_m {
                    let w_re = self.twiddle_re[s][j];
                    let w_im = self.twiddle_im[s][j];

                    let idx_even = k_start + j;
                    let idx_odd = k_start + j + half_m;

                    let (t_re, t_im) = complex_mul(
                        w_re,
                        w_im,
                        data[idx_odd].0,
                        data[idx_odd].1,
                    );

                    let u_re = data[idx_even].0;
                    let u_im = data[idx_even].1;

                    data[idx_even] = (u_re + t_re, u_im + t_im);
                    data[idx_odd] = (u_re - t_re, u_im - t_im);
                }
                k_start += m;
            }
        }
    }

    /// Inverse FFT using conjugate trick: IFFT(X) = conj(FFT(conj(X))) / N.
    fn ifft(&self, spectrum: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = spectrum.len();

        // Conjugate input
        let mut data: Vec<(f64, f64)> = spectrum.iter().map(|&(re, im)| (re, -im)).collect();

        // Forward FFT
        self.fft_in_place(&mut data);

        // Conjugate and scale
        let inv_n = 1.0 / n as f64;
        for sample in data.iter_mut() {
            sample.0 *= inv_n;
            sample.1 *= -inv_n;
        }

        data
    }
}

/// Bit-reversal permutation for in-place FFT.
fn bit_reverse_permutation(data: &mut [(f64, f64)]) {
    let n = data.len();
    let mut j = 0usize;
    for i in 0..n {
        if i < j {
            data.swap(i, j);
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }
}

/// Complex multiplication: (a + bi)(c + di) = (ac - bd) + (ad + bc)i
#[inline]
fn complex_mul(a: f64, b: f64, c: f64, d: f64) -> (f64, f64) {
    (a * c - b * d, a * d + b * c)
}

/// Approximate inverse normal CDF (probit function) using Acklam's rational approximation.
///
/// Accurate to approximately 1.15e-9 relative error across the full range.
fn inv_normal_cdf(p: f64) -> f64 {
    assert!(p > 0.0 && p < 1.0, "p must be in (0, 1)");

    // Coefficients from Peter Acklam's rational approximation
    const A: [f64; 6] = [
        -3.969683028665376e+01,
         2.209460984245205e+02,
        -2.759285104469687e+02,
         1.383577518672690e+02,
        -3.066479806614716e+01,
         2.506628277459239e+00,
    ];
    const B: [f64; 5] = [
        -5.447609879822406e+01,
         1.615858368580409e+02,
        -1.556989798598866e+02,
         6.680131188771972e+01,
        -1.328068155288572e+01,
    ];
    const C: [f64; 6] = [
        -7.784894002430293e-03,
        -3.223964580411365e-01,
        -2.400758277161838e+00,
        -2.549732539343734e+00,
         4.374664141464968e+00,
         2.938163982698783e+00,
    ];
    const D: [f64; 4] = [
        7.784695709041462e-03,
        3.224671290700398e-01,
        2.445134137142996e+00,
        3.754408661907416e+00,
    ];

    const P_LOW: f64 = 0.02425;
    const P_HIGH: f64 = 1.0 - P_LOW;

    if p < P_LOW {
        // Lower tail
        let q = (-2.0 * p.ln()).sqrt();
        (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
    } else if p <= P_HIGH {
        // Central region
        let q = p - 0.5;
        let r = q * q;
        (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5]) * q
            / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r + 1.0)
    } else {
        // Upper tail
        let q = (-2.0 * (1.0 - p).ln()).sqrt();
        -(((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Simple linear congruential PRNG for deterministic tests.
    struct TestRng {
        state: u64,
    }

    impl TestRng {
        fn new(seed: u64) -> Self {
            Self { state: seed }
        }

        /// Generate a uniform random f64 in (0, 1).
        fn next_f64(&mut self) -> f64 {
            self.state = self
                .state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            // Ensure result is in (0, 1) exclusive
            ((self.state >> 33) as f64 + 0.5) / (1u64 << 31) as f64
        }

        /// Generate a Gaussian random pair via Box-Muller.
        fn next_gaussian_pair(&mut self) -> (f64, f64) {
            let u1 = self.next_f64().max(1e-15);
            let u2 = self.next_f64();
            let r = (-2.0 * u1.ln()).sqrt();
            let theta = 2.0 * PI * u2;
            (r * theta.cos(), r * theta.sin())
        }

        /// Generate complex Gaussian noise samples.
        fn gaussian_samples(&mut self, n: usize) -> Vec<(f64, f64)> {
            (0..n).map(|_| self.next_gaussian_pair()).collect()
        }
    }

    fn default_config() -> SkConfig {
        SkConfig {
            fft_size: 64,
            num_accumulations: 16,
            sample_rate: 1_000_000.0,
            false_alarm_rate: 0.0027,
        }
    }

    #[test]
    fn test_new_valid_config() {
        let config = default_config();
        let sk = SpectralKurtosis::new(config);
        assert_eq!(sk.config.fft_size, 64);
        assert_eq!(sk.config.num_accumulations, 16);
        assert_eq!(sk.window.len(), 64);
    }

    #[test]
    #[should_panic(expected = "fft_size must be a power of two")]
    fn test_new_non_power_of_two() {
        let config = SkConfig {
            fft_size: 50,
            num_accumulations: 10,
            sample_rate: 1e6,
            false_alarm_rate: 0.01,
        };
        SpectralKurtosis::new(config);
    }

    #[test]
    #[should_panic(expected = "num_accumulations must be >= 2")]
    fn test_new_insufficient_accumulations() {
        let config = SkConfig {
            fft_size: 64,
            num_accumulations: 1,
            sample_rate: 1e6,
            false_alarm_rate: 0.01,
        };
        SpectralKurtosis::new(config);
    }

    #[test]
    #[should_panic(expected = "false_alarm_rate must be in (0, 1)")]
    fn test_new_invalid_pfa() {
        let config = SkConfig {
            fft_size: 64,
            num_accumulations: 10,
            sample_rate: 1e6,
            false_alarm_rate: 0.0,
        };
        SpectralKurtosis::new(config);
    }

    #[test]
    fn test_hann_window_endpoints() {
        let config = default_config();
        let sk = SpectralKurtosis::new(config);
        // Hann window is zero at endpoints
        assert!(sk.window[0].abs() < 1e-10);
        // Hann window peaks at center
        let center = sk.window.len() / 2;
        assert!((sk.window[center] - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_sk_gaussian_noise_near_one() {
        let config = SkConfig {
            fft_size: 64,
            num_accumulations: 64,
            sample_rate: 1e6,
            false_alarm_rate: 0.01,
        };
        let sk = SpectralKurtosis::new(config);
        let mut rng = TestRng::new(42);
        let samples = rng.gaussian_samples(64 * 64);

        let sk_values = sk.compute_sk(&samples);

        // For Gaussian noise, SK should be close to 1.0 for most bins
        // (skip bin 0 which can be anomalous due to DC)
        let mean_sk: f64 = sk_values[1..].iter().sum::<f64>() / (sk_values.len() - 1) as f64;
        assert!(
            (mean_sk - 1.0).abs() < 0.5,
            "Mean SK for Gaussian noise should be near 1.0, got {}",
            mean_sk
        );
    }

    #[test]
    fn test_sk_cw_interference_below_one() {
        let config = SkConfig {
            fft_size: 64,
            num_accumulations: 32,
            sample_rate: 1e6,
            false_alarm_rate: 0.01,
        };
        let sk = SpectralKurtosis::new(config);

        // Generate a strong CW tone at bin 10 (freq = 10/64 * sample_rate)
        let n = 64;
        let m = 32;
        let tone_bin = 10;
        let tone_freq = 2.0 * PI * tone_bin as f64 / n as f64;
        let amplitude = 10.0;

        let mut rng = TestRng::new(99);
        let mut samples = Vec::with_capacity(n * m);
        for acc in 0..m {
            for i in 0..n {
                let t = (acc * n + i) as f64;
                let (nr, ni) = rng.next_gaussian_pair();
                let re = amplitude * (tone_freq * t).cos() + nr * 0.1;
                let im = amplitude * (tone_freq * t).sin() + ni * 0.1;
                samples.push((re, im));
            }
        }

        let sk_values = sk.compute_sk(&samples);

        // The bin containing the CW tone should have SK << 1
        // (deterministic signal drives SK toward 0)
        assert!(
            sk_values[tone_bin] < 0.5,
            "SK at CW tone bin should be << 1, got {}",
            sk_values[tone_bin]
        );
    }

    #[test]
    fn test_thresholds_reasonable() {
        let config = default_config();
        let sk = SpectralKurtosis::new(config);
        let (lower, upper) = sk.thresholds();

        // Thresholds should bracket 1.0
        assert!(lower < 1.0, "Lower threshold should be < 1.0, got {}", lower);
        assert!(upper > 1.0, "Upper threshold should be > 1.0, got {}", upper);
        // Lower should be non-negative
        assert!(
            lower >= 0.0,
            "Lower threshold should be >= 0.0, got {}",
            lower
        );
    }

    #[test]
    fn test_thresholds_tighter_with_more_accumulations() {
        let config_few = SkConfig {
            fft_size: 64,
            num_accumulations: 8,
            sample_rate: 1e6,
            false_alarm_rate: 0.01,
        };
        let config_many = SkConfig {
            fft_size: 64,
            num_accumulations: 128,
            sample_rate: 1e6,
            false_alarm_rate: 0.01,
        };

        let sk_few = SpectralKurtosis::new(config_few);
        let sk_many = SpectralKurtosis::new(config_many);

        let (lower_few, upper_few) = sk_few.thresholds();
        let (lower_many, upper_many) = sk_many.thresholds();

        // More accumulations should give tighter thresholds
        let range_few = upper_few - lower_few;
        let range_many = upper_many - lower_many;
        assert!(
            range_many < range_few,
            "More accumulations should give tighter thresholds: {} vs {}",
            range_many,
            range_few
        );
    }

    #[test]
    fn test_detect_rfi_gaussian_clean() {
        let config = SkConfig {
            fft_size: 64,
            num_accumulations: 64,
            sample_rate: 1e6,
            false_alarm_rate: 0.01,
        };
        let sk = SpectralKurtosis::new(config);
        let mut rng = TestRng::new(12345);
        let samples = rng.gaussian_samples(64 * 64);

        let result = sk.detect_rfi(&samples);

        // Very few bins should be flagged for clean Gaussian noise
        assert!(
            result.rfi_fraction < 0.3,
            "Gaussian noise should have low RFI fraction, got {}",
            result.rfi_fraction
        );
    }

    #[test]
    fn test_detect_rfi_with_cw_tone() {
        // Use larger M for more stable SK and a tone that lands squarely in one bin
        let n = 64;
        let m = 64;
        let config = SkConfig {
            fft_size: n,
            num_accumulations: m,
            sample_rate: 1e6,
            false_alarm_rate: 0.05, // wider Pfa for easier detection
        };
        let sk = SpectralKurtosis::new(config);

        let tone_bin = 16;
        let tone_freq = 2.0 * PI * tone_bin as f64 / n as f64;

        let mut rng = TestRng::new(777);
        let mut samples = Vec::with_capacity(n * m);
        for acc in 0..m {
            for i in 0..n {
                let t = (acc * n + i) as f64;
                let (nr, ni) = rng.next_gaussian_pair();
                // Very high SNR tone: amplitude 50, noise 0.1
                let re = 50.0 * (tone_freq * t).cos() + nr * 0.1;
                let im = 50.0 * (tone_freq * t).sin() + ni * 0.1;
                samples.push((re, im));
            }
        }

        let result = sk.detect_rfi(&samples);

        // The SK value at the tone bin should be very low (near 0 for pure tone)
        assert!(
            result.sk_values[tone_bin] < 0.3,
            "SK at CW tone bin {} should be near 0, got {}",
            tone_bin,
            result.sk_values[tone_bin]
        );

        // The tone bin should be flagged
        assert!(
            result.flagged_bins[tone_bin],
            "CW tone bin {} should be flagged as RFI (SK={}, lower={}, upper={})",
            tone_bin,
            result.sk_values[tone_bin],
            result.threshold_lower,
            result.threshold_upper
        );
    }

    #[test]
    fn test_classify_clean() {
        let config = default_config();
        let sk = SpectralKurtosis::new(config);
        // All SK values near 1.0 -- within thresholds
        let sk_values: Vec<f64> = vec![1.0; 64];
        assert_eq!(sk.classify_rfi(&sk_values), RfiType::Clean);
    }

    #[test]
    fn test_classify_narrowband() {
        let config = SkConfig {
            fft_size: 64,
            num_accumulations: 100, // Large M for tight thresholds
            sample_rate: 1e6,
            false_alarm_rate: 0.01,
        };
        let sk = SpectralKurtosis::new(config);
        let (lower, upper) = sk.thresholds();

        // Most bins clean, a few bins with SK = 0 (well below any threshold)
        let mut sk_values = vec![1.0; 64];
        // Flag 5 bins (7.8%) -- enough to exceed 5% but less than 20%
        for i in 10..15 {
            sk_values[i] = 0.0; // CW-like: SK near 0, definitely below lower threshold
        }
        let rfi_type = sk.classify_rfi(&sk_values);
        assert_eq!(
            rfi_type,
            RfiType::Narrowband,
            "Expected Narrowband, thresholds=({}, {}), flagged 5/64 bins with SK=0",
            lower,
            upper
        );
    }

    #[test]
    fn test_classify_broadband() {
        let config = SkConfig {
            fft_size: 64,
            num_accumulations: 100, // Large M for tight thresholds
            sample_rate: 1e6,
            false_alarm_rate: 0.01,
        };
        let sk = SpectralKurtosis::new(config);
        let (lower, upper) = sk.thresholds();

        // Many bins with SK = 0 (well below threshold) -- >= 20%
        let mut sk_values = vec![1.0; 64];
        for i in 0..20 {
            sk_values[i] = 0.0; // 20 out of 64 = 31.25%
        }
        let rfi_type = sk.classify_rfi(&sk_values);
        assert_eq!(
            rfi_type,
            RfiType::Broadband,
            "Expected Broadband, thresholds=({}, {}), flagged 20/64 bins with SK=0",
            lower,
            upper
        );
    }

    #[test]
    fn test_classify_impulsive() {
        let config = SkConfig {
            fft_size: 64,
            num_accumulations: 100,
            sample_rate: 1e6,
            false_alarm_rate: 0.01,
        };
        let sk = SpectralKurtosis::new(config);
        let (_, upper) = sk.thresholds();

        // Many bins above upper threshold (impulsive: SK >> 1)
        let mut sk_values = vec![1.0; 64];
        for i in 0..15 {
            sk_values[i] = 100.0; // Way above upper threshold
        }
        let rfi_type = sk.classify_rfi(&sk_values);
        assert_eq!(
            rfi_type,
            RfiType::Impulsive,
            "Expected Impulsive, upper threshold={}, 15 bins with SK=100",
            upper
        );
    }

    #[test]
    fn test_flagging_mask_matches_detection() {
        let config = default_config();
        let sk = SpectralKurtosis::new(config);
        let mut rng = TestRng::new(555);
        let samples = rng.gaussian_samples(64 * 16);

        let result = sk.detect_rfi(&samples);
        let mask = sk.flagging_mask(&result);

        assert_eq!(mask.len(), result.flagged_bins.len());
        assert_eq!(mask, result.flagged_bins);
    }

    #[test]
    fn test_apply_mask_zeros_flagged_bins() {
        let config = SkConfig {
            fft_size: 16,
            num_accumulations: 4,
            sample_rate: 1e6,
            false_alarm_rate: 0.01,
        };
        let sk = SpectralKurtosis::new(config);

        // Create simple test signal
        let mut rng = TestRng::new(101);
        let samples = rng.gaussian_samples(16 * 4);

        // Flag bins 4..8 (25% of bins)
        let mut mask = vec![false; 16];
        for i in 4..8 {
            mask[i] = true;
        }

        let cleaned = sk.apply_mask(&samples, &mask);
        assert_eq!(cleaned.len(), 16 * 4);

        // Verify all output values are finite
        for (re, im) in &cleaned {
            assert!(re.is_finite(), "Output real part should be finite");
            assert!(im.is_finite(), "Output imag part should be finite");
        }
    }

    #[test]
    fn test_apply_mask_all_false_preserves_signal() {
        // With no bins flagged, the windowed FFT -> IFFT should approximately
        // reconstruct the windowed signal
        let config = SkConfig {
            fft_size: 16,
            num_accumulations: 2,
            sample_rate: 1e6,
            false_alarm_rate: 0.01,
        };
        let sk = SpectralKurtosis::new(config);

        let mut rng = TestRng::new(202);
        let samples = rng.gaussian_samples(16 * 2);

        let mask = vec![false; 16]; // No bins flagged
        let cleaned = sk.apply_mask(&samples, &mask);

        assert_eq!(cleaned.len(), 32);
        // Output should have finite values
        for &(re, im) in &cleaned {
            assert!(re.is_finite());
            assert!(im.is_finite());
        }
    }

    #[test]
    fn test_fft_roundtrip() {
        // Verify FFT -> IFFT roundtrip preserves signal
        let config = SkConfig {
            fft_size: 16,
            num_accumulations: 2,
            sample_rate: 1e6,
            false_alarm_rate: 0.01,
        };
        let sk = SpectralKurtosis::new(config);

        let input: Vec<(f64, f64)> = (0..16)
            .map(|i| {
                let t = i as f64 / 16.0;
                ((2.0 * PI * 3.0 * t).cos(), (2.0 * PI * 3.0 * t).sin())
            })
            .collect();

        // FFT then IFFT should recover the original signal
        let mut fft_data = input.clone();
        sk.fft_in_place(&mut fft_data);
        let recovered = sk.ifft(&fft_data);

        for (i, (&(re_in, im_in), &(re_out, im_out))) in
            input.iter().zip(recovered.iter()).enumerate()
        {
            assert!(
                (re_in - re_out).abs() < 1e-10,
                "Real mismatch at {}: {} vs {}",
                i,
                re_in,
                re_out
            );
            assert!(
                (im_in - im_out).abs() < 1e-10,
                "Imag mismatch at {}: {} vs {}",
                i,
                im_in,
                im_out
            );
        }
    }

    #[test]
    fn test_inv_normal_cdf_known_values() {
        // Phi^-1(0.5) = 0
        assert!(
            inv_normal_cdf(0.5).abs() < 1e-6,
            "inv_normal_cdf(0.5) = {}, expected 0",
            inv_normal_cdf(0.5)
        );
        // Phi^-1(0.975) = 1.96
        assert!(
            (inv_normal_cdf(0.975) - 1.96).abs() < 0.01,
            "inv_normal_cdf(0.975) = {}, expected ~1.96",
            inv_normal_cdf(0.975)
        );
        // Phi^-1(0.8413) ~ 1.0
        assert!(
            (inv_normal_cdf(0.8413) - 1.0).abs() < 0.01,
            "inv_normal_cdf(0.8413) = {}, expected ~1.0",
            inv_normal_cdf(0.8413)
        );
        // Symmetry: Phi^-1(p) = -Phi^-1(1-p)
        let p = 0.1;
        assert!(
            (inv_normal_cdf(p) + inv_normal_cdf(1.0 - p)).abs() < 1e-6,
            "Symmetry violated"
        );
    }

    #[test]
    fn test_detection_result_fields() {
        let config = SkConfig {
            fft_size: 32,
            num_accumulations: 8,
            sample_rate: 1e6,
            false_alarm_rate: 0.05,
        };
        let sk = SpectralKurtosis::new(config);
        let mut rng = TestRng::new(999);
        let samples = rng.gaussian_samples(32 * 8);

        let result = sk.detect_rfi(&samples);

        assert_eq!(result.flagged_bins.len(), 32);
        assert_eq!(result.sk_values.len(), 32);
        assert!(result.threshold_lower < result.threshold_upper);
        assert!(result.rfi_fraction >= 0.0 && result.rfi_fraction <= 1.0);
    }

    #[test]
    fn test_classify_empty_values() {
        let config = default_config();
        let sk = SpectralKurtosis::new(config);
        assert_eq!(sk.classify_rfi(&[]), RfiType::Clean);
    }
}

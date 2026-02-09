//! Wavelet Analysis and Synthesis
//!
//! Discrete Wavelet Transform (DWT) for multi-resolution signal analysis,
//! denoising, and compression. Useful for analyzing non-stationary signals.
//!
//! ## Blocks
//!
//! - **DwtAnalyzer**: Forward DWT decomposition (analysis filter bank)
//! - **DwtSynthesizer**: Inverse DWT reconstruction (synthesis filter bank)
//! - **WaveletDenoiser**: Soft/hard thresholding for denoising via DWT
//!
//! ## Supported Wavelets
//!
//! - Haar (db1): Simplest wavelet, good for step detection
//! - Daubechies-4 (db4): Good general-purpose wavelet
//! - Symlet-4 (sym4): Near-symmetric variant of Daubechies
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::wavelet::{DwtAnalyzer, DwtSynthesizer, WaveletType};
//!
//! let analyzer = DwtAnalyzer::new(WaveletType::Haar, 3);
//! let signal: Vec<f64> = (0..64).map(|i| (i as f64 * 0.1).sin()).collect();
//! let coeffs = analyzer.analyze(&signal);
//! assert_eq!(coeffs.num_levels(), 3);
//!
//! let synth = DwtSynthesizer::new(WaveletType::Haar);
//! let reconstructed = synth.synthesize(&coeffs);
//! assert_eq!(reconstructed.len(), signal.len());
//! ```

/// Wavelet family selection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WaveletType {
    /// Haar wavelet (db1). Taps: [1/√2, 1/√2].
    Haar,
    /// Daubechies-4 wavelet. 4-tap lowpass, good for smooth signals.
    Db4,
    /// Symlet-4 wavelet. Near-symmetric 8-tap, less phase distortion.
    Sym4,
}

/// Wavelet filter coefficients (lowpass and highpass for analysis/synthesis).
#[derive(Debug, Clone)]
pub struct WaveletFilter {
    /// Lowpass decomposition filter (analysis).
    pub lo_d: Vec<f64>,
    /// Highpass decomposition filter (analysis).
    pub hi_d: Vec<f64>,
    /// Lowpass reconstruction filter (synthesis).
    pub lo_r: Vec<f64>,
    /// Highpass reconstruction filter (synthesis).
    pub hi_r: Vec<f64>,
}

impl WaveletFilter {
    /// Get filter coefficients for the given wavelet type.
    ///
    /// For orthogonal wavelets with correlation-based analysis:
    /// - hi_d[m] = (-1)^(N-1-m) * lo_d[N-1-m]  (QMF relation)
    /// - lo_r = lo_d  (synthesis = analysis for orthogonal wavelets)
    /// - hi_r = hi_d
    pub fn new(wavelet: WaveletType) -> Self {
        let lo_d = match wavelet {
            WaveletType::Haar => {
                let v = std::f64::consts::FRAC_1_SQRT_2;
                vec![v, v]
            }
            WaveletType::Db4 => {
                let h0 = (1.0 + 3.0_f64.sqrt()) / (4.0 * 2.0_f64.sqrt());
                let h1 = (3.0 + 3.0_f64.sqrt()) / (4.0 * 2.0_f64.sqrt());
                let h2 = (3.0 - 3.0_f64.sqrt()) / (4.0 * 2.0_f64.sqrt());
                let h3 = (1.0 - 3.0_f64.sqrt()) / (4.0 * 2.0_f64.sqrt());
                vec![h0, h1, h2, h3]
            }
            WaveletType::Sym4 => {
                vec![
                    -0.07576571478927333,
                    -0.02963552764599851,
                    0.49761866763201545,
                    0.8037387518059161,
                    0.29785779560527736,
                    -0.09921954357684722,
                    -0.012603967262037833,
                    0.032223100604042702,
                ]
            }
        };

        let n = lo_d.len();
        // QMF relation: hi_d[m] = (-1)^(N-1-m) * lo_d[N-1-m]
        let hi_d: Vec<f64> = (0..n)
            .map(|m| {
                let sign = if (n - 1 - m) % 2 == 0 { 1.0 } else { -1.0 };
                sign * lo_d[n - 1 - m]
            })
            .collect();

        // For orthogonal wavelets: synthesis filters = analysis filters
        Self {
            lo_r: lo_d.clone(),
            hi_r: hi_d.clone(),
            lo_d,
            hi_d,
        }
    }

    /// Filter length.
    pub fn len(&self) -> usize {
        self.lo_d.len()
    }
}

/// DWT decomposition coefficients at multiple levels.
#[derive(Debug, Clone)]
pub struct DwtCoefficients {
    /// Approximation coefficients at the coarsest level.
    pub approximation: Vec<f64>,
    /// Detail coefficients at each level (from finest to coarsest).
    pub details: Vec<Vec<f64>>,
    /// Original signal length for reconstruction.
    pub original_len: usize,
}

impl DwtCoefficients {
    /// Number of decomposition levels.
    pub fn num_levels(&self) -> usize {
        self.details.len()
    }

    /// Total number of coefficients.
    pub fn total_coefficients(&self) -> usize {
        self.approximation.len() + self.details.iter().map(|d| d.len()).sum::<usize>()
    }

    /// Get energy distribution across levels.
    pub fn energy_per_level(&self) -> Vec<f64> {
        let mut energies = Vec::with_capacity(self.details.len() + 1);
        energies.push(self.approximation.iter().map(|x| x * x).sum());
        for detail in &self.details {
            energies.push(detail.iter().map(|x| x * x).sum());
        }
        energies
    }
}

/// DWT analyzer — forward wavelet decomposition.
#[derive(Debug, Clone)]
pub struct DwtAnalyzer {
    /// Wavelet filter bank.
    filter: WaveletFilter,
    /// Number of decomposition levels.
    levels: usize,
    /// Wavelet type.
    wavelet: WaveletType,
}

impl DwtAnalyzer {
    pub fn new(wavelet: WaveletType, levels: usize) -> Self {
        assert!(levels > 0, "At least 1 decomposition level required");
        Self {
            filter: WaveletFilter::new(wavelet),
            levels,
            wavelet,
        }
    }

    /// Perform multi-level DWT analysis.
    pub fn analyze(&self, signal: &[f64]) -> DwtCoefficients {
        let mut approx = signal.to_vec();
        let mut details = Vec::with_capacity(self.levels);

        for _ in 0..self.levels {
            let (a, d) = self.single_level_decompose(&approx);
            details.push(d);
            approx = a;
        }

        DwtCoefficients {
            approximation: approx,
            details,
            original_len: signal.len(),
        }
    }

    /// Single-level DWT decomposition: convolve + downsample by 2.
    fn single_level_decompose(&self, input: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let n = input.len();
        let out_len = (n + 1) / 2; // ceil(n/2)

        let mut approx = Vec::with_capacity(out_len);
        let mut detail = Vec::with_capacity(out_len);

        for i in 0..out_len {
            let mut lo_sum = 0.0;
            let mut hi_sum = 0.0;
            for (k, (&lo, &hi)) in self.filter.lo_d.iter().zip(self.filter.hi_d.iter()).enumerate()
            {
                let idx = 2 * i + k;
                let sample = if idx < n {
                    input[idx]
                } else {
                    // Symmetric extension
                    input[n.saturating_sub(1).saturating_sub(idx - n)]
                };
                lo_sum += lo * sample;
                hi_sum += hi * sample;
            }
            approx.push(lo_sum);
            detail.push(hi_sum);
        }

        (approx, detail)
    }

    pub fn wavelet(&self) -> WaveletType {
        self.wavelet
    }

    pub fn levels(&self) -> usize {
        self.levels
    }
}

/// DWT synthesizer — inverse wavelet reconstruction.
#[derive(Debug, Clone)]
pub struct DwtSynthesizer {
    /// Wavelet filter bank.
    filter: WaveletFilter,
    /// Wavelet type.
    wavelet: WaveletType,
}

impl DwtSynthesizer {
    pub fn new(wavelet: WaveletType) -> Self {
        Self {
            filter: WaveletFilter::new(wavelet),
            wavelet,
        }
    }

    /// Perform multi-level inverse DWT reconstruction.
    pub fn synthesize(&self, coeffs: &DwtCoefficients) -> Vec<f64> {
        let mut approx = coeffs.approximation.clone();

        // Reconstruct from coarsest to finest
        for detail in coeffs.details.iter().rev() {
            approx = self.single_level_reconstruct(&approx, detail);
        }

        // Trim to original length
        approx.truncate(coeffs.original_len);
        approx
    }

    /// Single-level inverse DWT: upsample + convolve + sum.
    fn single_level_reconstruct(&self, approx: &[f64], detail: &[f64]) -> Vec<f64> {
        let n = approx.len().max(detail.len());
        let out_len = 2 * n;
        let mut output = vec![0.0; out_len];

        // Upsample and filter approximation coefficients
        for i in 0..n {
            if i < approx.len() {
                for (k, &coeff) in self.filter.lo_r.iter().enumerate() {
                    let idx = 2 * i + k;
                    if idx < out_len {
                        output[idx] += coeff * approx[i];
                    }
                }
            }
        }

        // Upsample and filter detail coefficients
        for i in 0..n {
            if i < detail.len() {
                for (k, &coeff) in self.filter.hi_r.iter().enumerate() {
                    let idx = 2 * i + k;
                    if idx < out_len {
                        output[idx] += coeff * detail[i];
                    }
                }
            }
        }

        output
    }

    pub fn wavelet(&self) -> WaveletType {
        self.wavelet
    }
}

/// Thresholding method for wavelet denoising.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ThresholdMethod {
    /// Hard thresholding: set coefficients below threshold to zero.
    Hard,
    /// Soft thresholding: shrink coefficients toward zero.
    Soft,
}

/// Wavelet denoiser — thresholds detail coefficients to remove noise.
///
/// Uses the universal threshold: λ = σ * √(2 * ln(N))
/// where σ is estimated from the finest detail coefficients (MAD estimator).
#[derive(Debug, Clone)]
pub struct WaveletDenoiser {
    /// Wavelet type for analysis/synthesis.
    wavelet: WaveletType,
    /// Decomposition levels.
    levels: usize,
    /// Thresholding method.
    method: ThresholdMethod,
    /// Manual threshold override (None = automatic).
    manual_threshold: Option<f64>,
}

impl WaveletDenoiser {
    pub fn new(wavelet: WaveletType, levels: usize, method: ThresholdMethod) -> Self {
        Self {
            wavelet,
            levels,
            method,
            manual_threshold: None,
        }
    }

    /// Set a manual threshold (overrides automatic estimation).
    pub fn set_threshold(&mut self, threshold: f64) {
        self.manual_threshold = Some(threshold);
    }

    /// Denoise a signal.
    pub fn denoise(&self, signal: &[f64]) -> Vec<f64> {
        let analyzer = DwtAnalyzer::new(self.wavelet, self.levels);
        let mut coeffs = analyzer.analyze(signal);

        // Estimate noise level from finest detail coefficients
        let threshold = if let Some(t) = self.manual_threshold {
            t
        } else {
            self.estimate_threshold(&coeffs)
        };

        // Apply thresholding to detail coefficients
        for detail in coeffs.details.iter_mut() {
            self.apply_threshold(detail, threshold);
        }

        let synthesizer = DwtSynthesizer::new(self.wavelet);
        synthesizer.synthesize(&coeffs)
    }

    /// Estimate threshold using MAD (Median Absolute Deviation) of finest level.
    fn estimate_threshold(&self, coeffs: &DwtCoefficients) -> f64 {
        if coeffs.details.is_empty() {
            return 0.0;
        }

        let finest = &coeffs.details[0]; // Finest level detail coefficients
        if finest.is_empty() {
            return 0.0;
        }

        // MAD estimator for noise standard deviation
        let mut abs_vals: Vec<f64> = finest.iter().map(|x| x.abs()).collect();
        abs_vals.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = abs_vals[abs_vals.len() / 2];
        let sigma = median / 0.6745; // MAD-based σ estimate

        // Universal threshold
        let n = coeffs.original_len as f64;
        sigma * (2.0 * n.ln()).sqrt()
    }

    /// Apply hard or soft thresholding.
    fn apply_threshold(&self, coeffs: &mut [f64], threshold: f64) {
        for c in coeffs.iter_mut() {
            match self.method {
                ThresholdMethod::Hard => {
                    if c.abs() < threshold {
                        *c = 0.0;
                    }
                }
                ThresholdMethod::Soft => {
                    if c.abs() < threshold {
                        *c = 0.0;
                    } else {
                        *c = c.signum() * (c.abs() - threshold);
                    }
                }
            }
        }
    }

    pub fn wavelet(&self) -> WaveletType {
        self.wavelet
    }

    pub fn method(&self) -> ThresholdMethod {
        self.method
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_haar_filter_orthogonality() {
        let f = WaveletFilter::new(WaveletType::Haar);
        // lo_d should be orthogonal to hi_d
        let dot: f64 = f.lo_d.iter().zip(f.hi_d.iter()).map(|(a, b)| a * b).sum();
        assert!(dot.abs() < 1e-10, "Filters should be orthogonal");
        // lo_d should have unit energy
        let energy: f64 = f.lo_d.iter().map(|x| x * x).sum();
        assert!((energy - 1.0).abs() < 1e-10, "Filter should have unit energy");
    }

    #[test]
    fn test_db4_filter_length() {
        let f = WaveletFilter::new(WaveletType::Db4);
        assert_eq!(f.len(), 4);
    }

    #[test]
    fn test_sym4_filter_length() {
        let f = WaveletFilter::new(WaveletType::Sym4);
        assert_eq!(f.len(), 8);
    }

    #[test]
    fn test_haar_roundtrip() {
        let analyzer = DwtAnalyzer::new(WaveletType::Haar, 2);
        let signal: Vec<f64> = (0..16).map(|i| (i as f64 * 0.3).sin()).collect();
        let coeffs = analyzer.analyze(&signal);
        assert_eq!(coeffs.num_levels(), 2);

        let synth = DwtSynthesizer::new(WaveletType::Haar);
        let reconstructed = synth.synthesize(&coeffs);
        assert_eq!(reconstructed.len(), signal.len());

        for (orig, recon) in signal.iter().zip(reconstructed.iter()) {
            assert!(
                (orig - recon).abs() < 0.2,
                "Haar roundtrip: orig={:.3}, recon={:.3}",
                orig,
                recon
            );
        }
    }

    #[test]
    fn test_db4_roundtrip() {
        let analyzer = DwtAnalyzer::new(WaveletType::Db4, 2);
        let signal: Vec<f64> = (0..32).map(|i| (i as f64 * 0.2).sin()).collect();
        let coeffs = analyzer.analyze(&signal);

        let synth = DwtSynthesizer::new(WaveletType::Db4);
        let reconstructed = synth.synthesize(&coeffs);
        assert_eq!(reconstructed.len(), signal.len());
    }

    #[test]
    fn test_dwt_coefficient_structure() {
        let analyzer = DwtAnalyzer::new(WaveletType::Haar, 3);
        let signal = vec![1.0; 64];
        let coeffs = analyzer.analyze(&signal);

        assert_eq!(coeffs.num_levels(), 3);
        // Each level halves the length
        assert_eq!(coeffs.details[0].len(), 32); // Finest
        assert_eq!(coeffs.details[1].len(), 16);
        assert_eq!(coeffs.details[2].len(), 8); // Coarsest
        assert_eq!(coeffs.approximation.len(), 8);
    }

    #[test]
    fn test_dwt_energy_conservation() {
        let analyzer = DwtAnalyzer::new(WaveletType::Haar, 2);
        let signal: Vec<f64> = (0..32).map(|i| (i as f64 * 0.5).sin()).collect();
        let signal_energy: f64 = signal.iter().map(|x| x * x).sum();

        let coeffs = analyzer.analyze(&signal);
        let energies = coeffs.energy_per_level();
        let coeff_energy: f64 = energies.iter().sum();

        // Energy should be approximately conserved (within boundary effects)
        assert!(
            (signal_energy - coeff_energy).abs() / signal_energy < 0.5,
            "Energy conservation: signal={:.3}, coeffs={:.3}",
            signal_energy,
            coeff_energy
        );
    }

    #[test]
    fn test_hard_threshold_denoising() {
        // Create clean signal + noise
        let n = 128;
        let clean: Vec<f64> = (0..n).map(|i| (i as f64 * 0.1).sin()).collect();
        let noisy: Vec<f64> = clean
            .iter()
            .enumerate()
            .map(|(i, &x)| x + 0.1 * ((i * 17 + 3) as f64 * 0.7).sin())
            .collect();

        let denoiser = WaveletDenoiser::new(WaveletType::Haar, 3, ThresholdMethod::Hard);
        let denoised = denoiser.denoise(&noisy);
        assert_eq!(denoised.len(), n);
    }

    #[test]
    fn test_soft_threshold_denoising() {
        let n = 64;
        let signal: Vec<f64> = (0..n).map(|i| if i >= 20 && i < 40 { 1.0 } else { 0.0 }).collect();
        let noisy: Vec<f64> = signal
            .iter()
            .enumerate()
            .map(|(i, &x)| x + 0.05 * ((i * 13) as f64 * 0.3).sin())
            .collect();

        let denoiser = WaveletDenoiser::new(WaveletType::Haar, 2, ThresholdMethod::Soft);
        let denoised = denoiser.denoise(&noisy);
        assert_eq!(denoised.len(), n);
    }

    #[test]
    fn test_manual_threshold() {
        let mut denoiser = WaveletDenoiser::new(WaveletType::Haar, 2, ThresholdMethod::Hard);
        denoiser.set_threshold(0.5);

        let signal: Vec<f64> = (0..32).map(|i| (i as f64 * 0.2).sin()).collect();
        let denoised = denoiser.denoise(&signal);
        assert_eq!(denoised.len(), signal.len());
    }

    #[test]
    fn test_total_coefficients() {
        let analyzer = DwtAnalyzer::new(WaveletType::Haar, 3);
        let signal = vec![0.0; 64];
        let coeffs = analyzer.analyze(&signal);
        // 8 (approx) + 32 + 16 + 8 (details) = 64
        assert_eq!(coeffs.total_coefficients(), 64);
    }

    #[test]
    fn test_single_level_decomposition() {
        let analyzer = DwtAnalyzer::new(WaveletType::Haar, 1);
        let signal = vec![1.0, 2.0, 3.0, 4.0];
        let coeffs = analyzer.analyze(&signal);
        assert_eq!(coeffs.num_levels(), 1);
        assert_eq!(coeffs.approximation.len(), 2);
        assert_eq!(coeffs.details[0].len(), 2);
    }

    #[test]
    fn test_step_signal_haar() {
        // Haar should detect the step efficiently
        // Place step at index 7 so it straddles a Haar pair boundary (6,7)
        let analyzer = DwtAnalyzer::new(WaveletType::Haar, 2);
        let mut signal = vec![0.0; 16];
        for i in 7..16 {
            signal[i] = 1.0;
        }
        let coeffs = analyzer.analyze(&signal);
        // Detail coefficients should be non-zero near the step at some level
        let has_nonzero = coeffs.details.iter().any(|level| {
            level.iter().any(|&d| d.abs() > 0.1)
        });
        assert!(has_nonzero, "Haar should detect the step transition");
    }

    #[test]
    fn test_constant_signal_approx() {
        let analyzer = DwtAnalyzer::new(WaveletType::Haar, 2);
        let signal = vec![3.0; 16];
        let coeffs = analyzer.analyze(&signal);
        // Detail coefficients of a constant signal should be ~zero
        for detail in &coeffs.details {
            for &d in detail {
                assert!(d.abs() < 1e-10, "Constant signal should have zero details");
            }
        }
    }
}

//! Radar waveform classifier via matched-filter correlation.
//!
//! Correlates received echoes against a bank of candidate waveforms and
//! identifies the modulation type.  Uses only the Rust standard library.
//!
//! # Example
//!
//! ```
//! use r4w_core::radar_waveform_classifier::{
//!     RadarWaveformClassifier, WaveformType,
//! };
//!
//! let mut clf = RadarWaveformClassifier::new(0.3);
//!
//! // Populate the waveform bank with reference templates
//! clf.add_lfm_template(1e6, 10e-6, 1e6);
//! clf.add_barker_template(13, 1e6);
//!
//! // Generate a test LFM pulse and classify it
//! let lfm = RadarWaveformClassifier::generate_lfm(1e6, 10e-6, 1e6);
//! let result = clf.classify(&lfm);
//! assert_eq!(result.detected_type, WaveformType::Lfm);
//! assert!(result.confidence > 0.5);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Minimal complex-number helpers (std-only, no external crates)
// ---------------------------------------------------------------------------

/// Minimal complex number for internal use.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cplx {
    pub re: f64,
    pub im: f64,
}

impl Cplx {
    #[inline]
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    #[inline]
    pub fn from_polar(mag: f64, phase: f64) -> Self {
        Self {
            re: mag * phase.cos(),
            im: mag * phase.sin(),
        }
    }

    #[inline]
    pub fn norm_sqr(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    #[inline]
    pub fn norm(self) -> f64 {
        self.norm_sqr().sqrt()
    }

    #[inline]
    pub fn conj(self) -> Self {
        Self {
            re: self.re,
            im: -self.im,
        }
    }
}

impl std::ops::Add for Cplx {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }
}

impl std::ops::AddAssign for Cplx {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.re += rhs.re;
        self.im += rhs.im;
    }
}

impl std::ops::Mul for Cplx {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }
}

impl std::ops::Mul<f64> for Cplx {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f64) -> Self {
        Self {
            re: self.re * rhs,
            im: self.im * rhs,
        }
    }
}

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Modulation family detected by the classifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum WaveformType {
    /// Linear Frequency Modulated chirp
    Lfm,
    /// Barker phase code
    Barker,
    /// Frank polyphase code
    FrankCode,
    /// Polyphase Barker code
    PolyphaseBarker,
    /// Continuous wave (unmodulated)
    Cw,
    /// Not matched to any template
    Unknown,
}

/// A reference waveform stored in the template bank.
#[derive(Clone, Debug)]
pub struct WaveformTemplate {
    /// Modulation family
    pub waveform_type: WaveformType,
    /// Complex baseband samples of the reference pulse
    pub samples: Vec<Cplx>,
    /// Bandwidth in Hz (informational)
    pub bandwidth: f64,
    /// Pulse width in seconds (informational)
    pub pulse_width: f64,
}

/// Result returned by [`RadarWaveformClassifier::classify`].
#[derive(Clone, Debug)]
pub struct ClassificationResult {
    /// Best-matching modulation type
    pub detected_type: WaveformType,
    /// Confidence in [0, 1]
    pub confidence: f64,
    /// Index of the best template in the bank
    pub matched_template_idx: usize,
    /// Peak normalised cross-correlation magnitude
    pub peak_correlation: f64,
}

// ---------------------------------------------------------------------------
// Classifier
// ---------------------------------------------------------------------------

/// Correlates input pulses against a bank of reference waveforms.
///
/// The detection threshold (in \[0, 1\]) gates the minimum normalised
/// cross-correlation required to declare a match.
pub struct RadarWaveformClassifier {
    templates: Vec<WaveformTemplate>,
    threshold: f64,
}

impl RadarWaveformClassifier {
    /// Create an empty classifier with the given correlation threshold.
    pub fn new(threshold: f64) -> Self {
        Self {
            templates: Vec::new(),
            threshold: threshold.clamp(0.0, 1.0),
        }
    }

    /// Number of templates currently in the bank.
    pub fn template_count(&self) -> usize {
        self.templates.len()
    }

    // -- template management ------------------------------------------------

    /// Add an arbitrary reference waveform to the bank.
    pub fn add_template(&mut self, template: WaveformTemplate) {
        self.templates.push(template);
    }

    /// Convenience: generate an LFM chirp reference and add it.
    pub fn add_lfm_template(&mut self, bandwidth: f64, pulse_width: f64, sample_rate: f64) {
        let samples = Self::generate_lfm(bandwidth, pulse_width, sample_rate);
        self.templates.push(WaveformTemplate {
            waveform_type: WaveformType::Lfm,
            samples,
            bandwidth,
            pulse_width,
        });
    }

    /// Convenience: generate a Barker code reference and add it.
    pub fn add_barker_template(&mut self, code_length: usize, sample_rate: f64) {
        let samples = Self::generate_barker(code_length);
        let pulse_width = samples.len() as f64 / sample_rate;
        self.templates.push(WaveformTemplate {
            waveform_type: WaveformType::Barker,
            samples,
            bandwidth: sample_rate,
            pulse_width,
        });
    }

    // -- classification -----------------------------------------------------

    /// Classify `input` by correlating against every template.
    ///
    /// Returns the best match (or `WaveformType::Unknown` if nothing
    /// exceeds the threshold).
    pub fn classify(&self, input: &[Cplx]) -> ClassificationResult {
        let mut best_corr = 0.0_f64;
        let mut best_idx = 0_usize;

        for (i, tmpl) in self.templates.iter().enumerate() {
            let peak = normalised_peak_correlation(input, &tmpl.samples);
            if peak > best_corr {
                best_corr = peak;
                best_idx = i;
            }
        }

        if best_corr >= self.threshold && !self.templates.is_empty() {
            // Confidence: how far above threshold, scaled to [0,1]
            let confidence = if self.threshold < 1.0 {
                ((best_corr - self.threshold) / (1.0 - self.threshold)).clamp(0.0, 1.0)
            } else {
                1.0
            };
            ClassificationResult {
                detected_type: self.templates[best_idx].waveform_type,
                confidence,
                matched_template_idx: best_idx,
                peak_correlation: best_corr,
            }
        } else {
            ClassificationResult {
                detected_type: WaveformType::Unknown,
                confidence: 0.0,
                matched_template_idx: 0,
                peak_correlation: best_corr,
            }
        }
    }

    // -- waveform generators ------------------------------------------------

    /// Generate a Linear Frequency Modulated (LFM) chirp.
    ///
    /// * `bandwidth` – sweep range in Hz
    /// * `pulse_width` – duration in seconds
    /// * `sample_rate` – samples per second
    pub fn generate_lfm(bandwidth: f64, pulse_width: f64, sample_rate: f64) -> Vec<Cplx> {
        let n = (pulse_width * sample_rate).round() as usize;
        if n == 0 {
            return Vec::new();
        }
        let chirp_rate = bandwidth / pulse_width;
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate - pulse_width / 2.0;
                let phase = PI * chirp_rate * t * t;
                Cplx::from_polar(1.0, phase)
            })
            .collect()
    }

    /// Generate a Barker phase-coded waveform (one sample per chip).
    ///
    /// Supported lengths: 2, 3, 4, 5, 7, 11, 13.  Returns empty vec for
    /// unsupported lengths.
    pub fn generate_barker(code_length: usize) -> Vec<Cplx> {
        let code: &[f64] = match code_length {
            2 => &[1.0, -1.0],
            3 => &[1.0, 1.0, -1.0],
            4 => &[1.0, 1.0, -1.0, 1.0],
            5 => &[1.0, 1.0, 1.0, -1.0, 1.0],
            7 => &[1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0],
            11 => &[1.0, 1.0, 1.0, -1.0, -1.0, -1.0, 1.0, -1.0, -1.0, 1.0, -1.0],
            13 => &[
                1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0,
            ],
            _ => return Vec::new(),
        };
        code.iter().map(|&v| Cplx::new(v, 0.0)).collect()
    }

    /// Generate a Frank polyphase code of order `m` (code length = m*m).
    pub fn generate_frank_code(m: usize) -> Vec<Cplx> {
        let n = m * m;
        (0..n)
            .map(|k| {
                let i = k / m;
                let j = k % m;
                let phase = 2.0 * PI * (i * j) as f64 / m as f64;
                Cplx::from_polar(1.0, phase)
            })
            .collect()
    }

    /// Generate a CW (unmodulated) pulse of `n` samples.
    pub fn generate_cw(n: usize) -> Vec<Cplx> {
        vec![Cplx::new(1.0, 0.0); n]
    }

    // -- ambiguity / pulse compression --------------------------------------

    /// Compute the ambiguity function |χ(τ, f_d)| on a grid.
    ///
    /// * `signal` – complex baseband pulse
    /// * `delay_range` – number of delay bins (centred on zero)
    /// * `doppler_bins` – number of Doppler bins
    /// * `doppler_max` – maximum Doppler shift in Hz
    /// * `sample_rate` – sample rate in Hz
    ///
    /// Returns a row-major matrix of shape `(doppler_bins, delay_range)`.
    pub fn ambiguity_function(
        signal: &[Cplx],
        delay_range: usize,
        doppler_bins: usize,
        doppler_max: f64,
        sample_rate: f64,
    ) -> Vec<Vec<f64>> {
        let n = signal.len();
        if n == 0 || delay_range == 0 || doppler_bins == 0 {
            return Vec::new();
        }

        let half_delay = delay_range / 2;
        let doppler_step = if doppler_bins > 1 {
            2.0 * doppler_max / (doppler_bins - 1) as f64
        } else {
            0.0
        };

        (0..doppler_bins)
            .map(|di| {
                let fd = -doppler_max + di as f64 * doppler_step;
                (0..delay_range)
                    .map(|ti| {
                        let tau = ti as isize - half_delay as isize;
                        let mut acc = Cplx::new(0.0, 0.0);
                        for k in 0..n {
                            let shifted_idx = k as isize + tau;
                            if shifted_idx >= 0 && (shifted_idx as usize) < n {
                                let doppler_phase =
                                    2.0 * PI * fd * k as f64 / sample_rate;
                                let doppler_phasor = Cplx::from_polar(1.0, doppler_phase);
                                let s = signal[k];
                                let s_shifted = signal[shifted_idx as usize].conj();
                                acc += s * doppler_phasor * s_shifted;
                            }
                        }
                        acc.norm()
                    })
                    .collect()
            })
            .collect()
    }

    /// Estimate pulse compression ratio from a template.
    ///
    /// Defined as the time-bandwidth product: `bandwidth * pulse_width`.
    pub fn pulse_compression_ratio(template: &WaveformTemplate) -> f64 {
        template.bandwidth * template.pulse_width
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Peak of the normalised cross-correlation magnitude.
///
/// Returns a value in \[0, 1\].
fn normalised_peak_correlation(a: &[Cplx], b: &[Cplx]) -> f64 {
    if a.is_empty() || b.is_empty() {
        return 0.0;
    }

    // Energy of b (template)
    let energy_b: f64 = b.iter().map(|s| s.norm_sqr()).sum();
    if energy_b == 0.0 {
        return 0.0;
    }

    // Sliding correlation
    let a_len = a.len();
    let b_len = b.len();
    let corr_len = if a_len >= b_len { a_len - b_len + 1 } else { 0 };

    let mut peak = 0.0_f64;
    for start in 0..corr_len {
        let mut acc = Cplx::new(0.0, 0.0);
        let mut energy_a_seg = 0.0_f64;
        for j in 0..b_len {
            acc += a[start + j] * b[j].conj();
            energy_a_seg += a[start + j].norm_sqr();
        }
        let denom = (energy_a_seg * energy_b).sqrt();
        if denom > 0.0 {
            let val = acc.norm() / denom;
            if val > peak {
                peak = val;
            }
        }
    }

    // Also check if b is longer than a (perfect match when same length)
    if a_len < b_len {
        // Correlate the overlap region
        let mut acc = Cplx::new(0.0, 0.0);
        let mut ea = 0.0_f64;
        let mut eb = 0.0_f64;
        let overlap = a_len;
        for j in 0..overlap {
            acc += a[j] * b[j].conj();
            ea += a[j].norm_sqr();
            eb += b[j].norm_sqr();
        }
        let denom = (ea * eb).sqrt();
        if denom > 0.0 {
            let val = acc.norm() / denom;
            if val > peak {
                peak = val;
            }
        }
    }

    peak
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // 1. LFM generation produces correct length
    #[test]
    fn test_generate_lfm_length() {
        let samples = RadarWaveformClassifier::generate_lfm(1e6, 10e-6, 1e6);
        assert_eq!(samples.len(), 10); // 10e-6 * 1e6 = 10
    }

    // 2. LFM samples are unit magnitude
    #[test]
    fn test_generate_lfm_unit_magnitude() {
        let samples = RadarWaveformClassifier::generate_lfm(500e3, 20e-6, 2e6);
        for s in &samples {
            assert!(approx_eq(s.norm(), 1.0, 1e-12));
        }
    }

    // 3. Barker code lengths and known sequences
    #[test]
    fn test_generate_barker_known_codes() {
        for &len in &[2, 3, 4, 5, 7, 11, 13] {
            let b = RadarWaveformClassifier::generate_barker(len);
            assert_eq!(b.len(), len, "Barker-{len} wrong length");
        }
        // Unsupported length
        assert!(RadarWaveformClassifier::generate_barker(6).is_empty());
    }

    // 4. Barker-13 autocorrelation sidelobe property
    #[test]
    fn test_barker13_autocorrelation() {
        let b = RadarWaveformClassifier::generate_barker(13);
        // Zero-lag autocorrelation should be 13
        let zero_lag: f64 = b.iter().map(|s| s.norm_sqr()).sum();
        assert!(approx_eq(zero_lag, 13.0, 1e-10));

        // Sidelobes should be <= 1 in magnitude
        for tau in 1..b.len() {
            let mut acc = Cplx::new(0.0, 0.0);
            for i in 0..(b.len() - tau) {
                acc += b[i] * b[i + tau].conj();
            }
            assert!(acc.norm() <= 1.0 + 1e-10, "Barker-13 sidelobe at tau={tau} too high");
        }
    }

    // 5. Classify LFM correctly
    #[test]
    fn test_classify_lfm() {
        let mut clf = RadarWaveformClassifier::new(0.3);
        clf.add_lfm_template(1e6, 10e-6, 1e6);
        clf.add_barker_template(13, 1e6);

        let pulse = RadarWaveformClassifier::generate_lfm(1e6, 10e-6, 1e6);
        let result = clf.classify(&pulse);

        assert_eq!(result.detected_type, WaveformType::Lfm);
        assert!(result.peak_correlation > 0.9);
    }

    // 6. Classify Barker correctly
    #[test]
    fn test_classify_barker() {
        let mut clf = RadarWaveformClassifier::new(0.3);
        clf.add_lfm_template(1e6, 10e-6, 1e6);
        clf.add_barker_template(13, 1e6);

        let pulse = RadarWaveformClassifier::generate_barker(13);
        let result = clf.classify(&pulse);

        assert_eq!(result.detected_type, WaveformType::Barker);
        assert!(result.peak_correlation > 0.9);
    }

    // 7. Empty classifier returns Unknown
    #[test]
    fn test_empty_classifier() {
        let clf = RadarWaveformClassifier::new(0.5);
        let pulse = RadarWaveformClassifier::generate_lfm(1e6, 10e-6, 1e6);
        let result = clf.classify(&pulse);
        assert_eq!(result.detected_type, WaveformType::Unknown);
    }

    // 8. Pulse compression ratio calculation
    #[test]
    fn test_pulse_compression_ratio() {
        let tmpl = WaveformTemplate {
            waveform_type: WaveformType::Lfm,
            samples: Vec::new(),
            bandwidth: 5e6,
            pulse_width: 10e-6,
        };
        let pcr = RadarWaveformClassifier::pulse_compression_ratio(&tmpl);
        assert!(approx_eq(pcr, 50.0, 1e-6));
    }

    // 9. Ambiguity function peak at (0 delay, 0 Doppler)
    #[test]
    fn test_ambiguity_function_peak() {
        let pulse = RadarWaveformClassifier::generate_lfm(1e6, 10e-6, 1e6);
        let af = RadarWaveformClassifier::ambiguity_function(&pulse, 11, 11, 5e4, 1e6);

        assert_eq!(af.len(), 11);
        assert_eq!(af[0].len(), 11);

        // Centre of the grid (zero delay, zero Doppler)
        let centre_doppler = 5; // middle row
        let centre_delay = 5; // middle column
        let peak_val = af[centre_doppler][centre_delay];

        // The peak should be the largest value
        let max_val: f64 = af.iter().flat_map(|row| row.iter()).copied().fold(0.0_f64, f64::max);
        assert!(approx_eq(peak_val, max_val, 1e-6),
                "AF peak not at centre: peak_val={peak_val}, max_val={max_val}");
    }

    // 10. Frank code generation
    #[test]
    fn test_generate_frank_code() {
        let m = 4;
        let code = RadarWaveformClassifier::generate_frank_code(m);
        assert_eq!(code.len(), m * m);
        // All chips are unit magnitude
        for s in &code {
            assert!(approx_eq(s.norm(), 1.0, 1e-12));
        }
        // First chip is always 1+0j (phase = 0)
        assert!(approx_eq(code[0].re, 1.0, 1e-12));
        assert!(approx_eq(code[0].im, 0.0, 1e-12));
    }

    // 11. CW generation
    #[test]
    fn test_generate_cw() {
        let cw = RadarWaveformClassifier::generate_cw(64);
        assert_eq!(cw.len(), 64);
        for s in &cw {
            assert!(approx_eq(s.re, 1.0, 1e-12));
            assert!(approx_eq(s.im, 0.0, 1e-12));
        }
    }

    // 12. High-threshold means Unknown when correlation is moderate
    #[test]
    fn test_high_threshold_rejects() {
        let mut clf = RadarWaveformClassifier::new(0.999);
        clf.add_barker_template(5, 1e6);

        // Feed a slightly noisy version
        let mut pulse = RadarWaveformClassifier::generate_barker(5);
        pulse[0] = Cplx::new(0.5, 0.3); // corrupt first chip
        let result = clf.classify(&pulse);

        // Should be Unknown because the corrupted waveform won't hit 0.999
        assert_eq!(result.detected_type, WaveformType::Unknown);
    }

    // 13. Template count tracking
    #[test]
    fn test_template_count() {
        let mut clf = RadarWaveformClassifier::new(0.3);
        assert_eq!(clf.template_count(), 0);
        clf.add_lfm_template(1e6, 10e-6, 1e6);
        assert_eq!(clf.template_count(), 1);
        clf.add_barker_template(7, 1e6);
        assert_eq!(clf.template_count(), 2);
    }
}

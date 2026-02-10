//! Comprehensive modulation quality analysis (EVM, MER, ACPR, spectral regrowth).
//!
//! The [`VectorSignalAnalyzer`] provides measurement capabilities commonly found in
//! bench-top vector signal analyzers: Error Vector Magnitude (EVM), Modulation Error
//! Ratio (MER), Adjacent Channel Power Ratio (ACPR), IQ imbalance, and more.
//!
//! # Example
//!
//! ```
//! use r4w_core::vector_signal_analyzer::{VectorSignalAnalyzer, ModulationType};
//!
//! // Create a VSA configured for QPSK
//! let mut vsa = VectorSignalAnalyzer::new(ModulationType::Qpsk);
//!
//! // Generate ideal QPSK symbols (normalized 1/sqrt(2))
//! let s = std::f64::consts::FRAC_1_SQRT_2;
//! let samples = vec![
//!     (s, s), (-s, s), (-s, -s), (s, -s),
//!     (s, s), (-s, s), (-s, -s), (s, -s),
//! ];
//!
//! // Measure EVM — ideal symbols should give ~0% EVM
//! let evm = vsa.measure_evm(&samples);
//! assert!(evm.evm_rms_percent < 1.0, "Ideal QPSK should have near-zero EVM");
//!
//! // Full analysis report
//! let report = vsa.analyze(&samples);
//! assert!(report.mer_db > 40.0, "MER should be very high for ideal symbols");
//! ```

use std::f64::consts::{FRAC_1_SQRT_2, PI};

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Supported modulation types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModulationType {
    Bpsk,
    Qpsk,
    Eightpsk,
    Qam16,
    Qam64,
    Qam256,
}

/// Error Vector Magnitude metrics.
#[derive(Debug, Clone)]
pub struct EvmMetrics {
    /// RMS EVM expressed as a percentage of the reference constellation magnitude.
    pub evm_rms_percent: f64,
    /// Peak EVM expressed as a percentage.
    pub evm_peak_percent: f64,
    /// RMS EVM in dB (20·log10(evm_rms_linear)).
    pub evm_rms_db: f64,
    /// Per-symbol EVM percentages.
    pub evm_per_symbol: Vec<f64>,
}

/// Adjacent Channel Power Ratio metrics.
#[derive(Debug, Clone)]
pub struct AcprMetrics {
    /// Power ratio of the lower adjacent channel relative to the main channel (dB).
    pub lower_acpr_db: f64,
    /// Power ratio of the upper adjacent channel relative to the main channel (dB).
    pub upper_acpr_db: f64,
    /// Absolute power in the main channel (dBm, assuming 50 Ω).
    pub main_channel_power_dbm: f64,
}

/// Full VSA measurement report.
#[derive(Debug, Clone)]
pub struct VsaReport {
    pub evm: EvmMetrics,
    pub acpr: AcprMetrics,
    /// Modulation Error Ratio in dB.
    pub mer_db: f64,
    /// IQ origin offset in dB.
    pub iq_offset_db: f64,
    /// Amplitude imbalance between I and Q rails in dB.
    pub amplitude_imbalance_db: f64,
    /// Phase imbalance between I and Q rails in degrees.
    pub phase_imbalance_deg: f64,
    /// Estimated carrier frequency offset in Hz (requires known sample rate).
    pub carrier_freq_offset_hz: f64,
}

// ---------------------------------------------------------------------------
// VectorSignalAnalyzer
// ---------------------------------------------------------------------------

/// Main VSA engine.
///
/// Construct with [`VectorSignalAnalyzer::new`], then call individual
/// measurement methods or [`VectorSignalAnalyzer::analyze`] for a full report.
pub struct VectorSignalAnalyzer {
    mod_type: ModulationType,
    /// Reference constellation points for the configured modulation.
    reference: Vec<(f64, f64)>,
    /// Sample rate in Hz — used for CFO estimation and ACPR.  Defaults to 1.0.
    sample_rate: f64,
}

impl VectorSignalAnalyzer {
    /// Create a new VSA for the given modulation type.
    pub fn new(mod_type: ModulationType) -> Self {
        let reference = generate_constellation(mod_type);
        Self {
            mod_type,
            reference,
            sample_rate: 1.0,
        }
    }

    /// Set the sample rate (Hz) for frequency-domain measurements.
    pub fn set_sample_rate(&mut self, fs: f64) {
        self.sample_rate = fs;
    }

    // -- Full report --------------------------------------------------------

    /// Run all measurements and return a comprehensive [`VsaReport`].
    pub fn analyze(&mut self, samples: &[(f64, f64)]) -> VsaReport {
        let evm = self.measure_evm(samples);
        let mer_db = self.measure_mer(samples);
        let (amplitude_imbalance_db, phase_imbalance_deg) = self.measure_iq_imbalance(samples);

        // DC offset → IQ origin offset
        let iq_offset_db = self.measure_iq_offset(samples);

        // CFO estimation via mean phase rotation between consecutive symbols
        let carrier_freq_offset_hz = self.estimate_cfo(samples);

        // ACPR with sensible defaults (channel_bw = 1.0, offset = 1.0 in
        // normalised-frequency units when sample_rate == 1.0).
        let acpr = self.measure_acpr(samples, 0.5, 1.0);

        VsaReport {
            evm,
            acpr,
            mer_db,
            iq_offset_db,
            amplitude_imbalance_db,
            phase_imbalance_deg,
            carrier_freq_offset_hz,
        }
    }

    // -- Individual measurements --------------------------------------------

    /// Compute EVM metrics for the given IQ samples.
    pub fn measure_evm(&self, samples: &[(f64, f64)]) -> EvmMetrics {
        if samples.is_empty() {
            return EvmMetrics {
                evm_rms_percent: 0.0,
                evm_peak_percent: 0.0,
                evm_rms_db: f64::NEG_INFINITY,
                evm_per_symbol: vec![],
            };
        }

        // Reference power (mean power of constellation points)
        let ref_power = self.reference_power();

        let mut sum_sq = 0.0_f64;
        let mut peak_sq = 0.0_f64;
        let mut per_symbol = Vec::with_capacity(samples.len());

        for &sample in samples {
            let nearest = self.nearest_constellation_point(sample);
            let ei = sample.0 - nearest.0;
            let eq = sample.1 - nearest.1;
            let err_sq = ei * ei + eq * eq;
            sum_sq += err_sq;
            if err_sq > peak_sq {
                peak_sq = err_sq;
            }
            let evm_pct = (err_sq / ref_power).sqrt() * 100.0;
            per_symbol.push(evm_pct);
        }

        let rms_linear = (sum_sq / samples.len() as f64 / ref_power).sqrt();
        let peak_linear = (peak_sq / ref_power).sqrt();

        EvmMetrics {
            evm_rms_percent: rms_linear * 100.0,
            evm_peak_percent: peak_linear * 100.0,
            evm_rms_db: 20.0 * rms_linear.max(1e-30).log10(),
            evm_per_symbol: per_symbol,
        }
    }

    /// Compute ACPR by integrating FFT power in main and adjacent channels.
    ///
    /// * `channel_bw` — bandwidth of the main channel in Hz.
    /// * `offset` — centre-to-centre spacing between main and adjacent channels in Hz.
    ///
    /// FFT bins are mapped to frequencies via [`Self::sample_rate`].
    pub fn measure_acpr(
        &self,
        samples: &[(f64, f64)],
        channel_bw: f64,
        offset: f64,
    ) -> AcprMetrics {
        let n = samples.len().next_power_of_two().max(64);
        let spectrum = fft_power(samples, n);

        let bin_hz = self.sample_rate / n as f64;

        // Helpers — integrate power in a frequency band [lo, hi) Hz (centred at DC).
        let integrate = |lo_hz: f64, hi_hz: f64| -> f64 {
            let mut pwr = 0.0;
            for (i, &p) in spectrum.iter().enumerate() {
                // Map bin index to frequency: 0..N/2 → 0..fs/2, N/2..N → -fs/2..0
                let freq = if i <= n / 2 {
                    i as f64 * bin_hz
                } else {
                    (i as f64 - n as f64) * bin_hz
                };
                if freq >= lo_hz && freq < hi_hz {
                    pwr += p;
                }
            }
            pwr
        };

        let half_bw = channel_bw / 2.0;
        let main_pwr = integrate(-half_bw, half_bw).max(1e-30);
        let lower_pwr = integrate(-offset - half_bw, -offset + half_bw).max(1e-30);
        let upper_pwr = integrate(offset - half_bw, offset + half_bw).max(1e-30);

        let main_dbm = 10.0 * main_pwr.log10() + 30.0; // W → dBm

        AcprMetrics {
            lower_acpr_db: 10.0 * (lower_pwr / main_pwr).log10(),
            upper_acpr_db: 10.0 * (upper_pwr / main_pwr).log10(),
            main_channel_power_dbm: main_dbm,
        }
    }

    /// Measure IQ imbalance: returns `(amplitude_imbalance_db, phase_imbalance_deg)`.
    ///
    /// Amplitude imbalance is `20·log10(rms_I / rms_Q)`.
    /// Phase imbalance is the deviation from 90° between I and Q.
    pub fn measure_iq_imbalance(&self, samples: &[(f64, f64)]) -> (f64, f64) {
        if samples.is_empty() {
            return (0.0, 0.0);
        }

        let n = samples.len() as f64;

        // RMS of I and Q rails
        let rms_i = (samples.iter().map(|s| s.0 * s.0).sum::<f64>() / n).sqrt();
        let rms_q = (samples.iter().map(|s| s.1 * s.1).sum::<f64>() / n).sqrt();

        let amp_db = if rms_q > 1e-30 {
            20.0 * (rms_i / rms_q).log10()
        } else {
            0.0
        };

        // Phase imbalance via correlation: E[I·Q] = 0.5·A²·sin(φ) for a
        // signal with perfect amplitude balance.  We approximate φ in degrees.
        let cross = samples.iter().map(|s| s.0 * s.1).sum::<f64>() / n;
        let power = (rms_i * rms_q).max(1e-30);
        // sin(phase_err) ≈ cross / power
        let phase_err_rad = (cross / power).clamp(-1.0, 1.0).asin();
        let phase_deg = phase_err_rad * 180.0 / PI;

        (amp_db, phase_deg)
    }

    /// Compute Modulation Error Ratio (MER) in dB.
    ///
    /// MER = 10·log10(Σ|ref|² / Σ|err|²) where err = sample − nearest ref.
    pub fn measure_mer(&self, samples: &[(f64, f64)]) -> f64 {
        if samples.is_empty() {
            return 0.0;
        }

        let mut ref_power = 0.0_f64;
        let mut err_power = 0.0_f64;

        for &sample in samples {
            let nearest = self.nearest_constellation_point(sample);
            ref_power += nearest.0 * nearest.0 + nearest.1 * nearest.1;
            let ei = sample.0 - nearest.0;
            let eq = sample.1 - nearest.1;
            err_power += ei * ei + eq * eq;
        }

        if err_power < 1e-30 {
            return 100.0; // effectively infinite MER, cap at 100 dB
        }

        10.0 * (ref_power / err_power).log10()
    }

    // -- Internal helpers ---------------------------------------------------

    /// Find the nearest constellation point to `sample`.
    fn nearest_constellation_point(&self, sample: (f64, f64)) -> (f64, f64) {
        let mut best = self.reference[0];
        let mut best_dist = f64::MAX;
        for &pt in &self.reference {
            let di = sample.0 - pt.0;
            let dq = sample.1 - pt.1;
            let d = di * di + dq * dq;
            if d < best_dist {
                best_dist = d;
                best = pt;
            }
        }
        best
    }

    /// Mean power of the reference constellation.
    fn reference_power(&self) -> f64 {
        let sum: f64 = self
            .reference
            .iter()
            .map(|p| p.0 * p.0 + p.1 * p.1)
            .sum();
        (sum / self.reference.len() as f64).max(1e-30)
    }

    /// IQ origin offset in dB: 10·log10(|mean|² / mean_power).
    fn measure_iq_offset(&self, samples: &[(f64, f64)]) -> f64 {
        if samples.is_empty() {
            return f64::NEG_INFINITY;
        }
        let n = samples.len() as f64;
        let mean_i = samples.iter().map(|s| s.0).sum::<f64>() / n;
        let mean_q = samples.iter().map(|s| s.1).sum::<f64>() / n;
        let dc_power = mean_i * mean_i + mean_q * mean_q;
        let sig_power = (samples.iter().map(|s| s.0 * s.0 + s.1 * s.1).sum::<f64>() / n).max(1e-30);
        10.0 * (dc_power / sig_power).max(1e-30).log10()
    }

    /// Estimate carrier frequency offset from mean inter-symbol phase rotation.
    fn estimate_cfo(&self, samples: &[(f64, f64)]) -> f64 {
        if samples.len() < 2 {
            return 0.0;
        }

        // Accumulate angle difference between consecutive symbols
        let mut angle_sum = 0.0_f64;
        let mut count = 0usize;
        for i in 1..samples.len() {
            // conjugate multiply: s[i] * conj(s[i-1])
            let (ri, ii) = samples[i];
            let (rp, ip) = samples[i - 1];
            let re = ri * rp + ii * ip;
            let im = ii * rp - ri * ip;
            angle_sum += im.atan2(re);
            count += 1;
        }

        if count == 0 {
            return 0.0;
        }

        let mean_angle = angle_sum / count as f64;
        // CFO = mean_angle / (2π) * sample_rate
        mean_angle / (2.0 * PI) * self.sample_rate
    }
}

// ---------------------------------------------------------------------------
// Constellation generation
// ---------------------------------------------------------------------------

/// Generate normalised reference constellation points for a modulation type.
fn generate_constellation(mod_type: ModulationType) -> Vec<(f64, f64)> {
    match mod_type {
        ModulationType::Bpsk => vec![(1.0, 0.0), (-1.0, 0.0)],
        ModulationType::Qpsk => {
            let s = FRAC_1_SQRT_2;
            vec![(s, s), (-s, s), (-s, -s), (s, -s)]
        }
        ModulationType::Eightpsk => (0..8)
            .map(|k| {
                let angle = 2.0 * PI * k as f64 / 8.0;
                (angle.cos(), angle.sin())
            })
            .collect(),
        ModulationType::Qam16 => generate_qam(4),
        ModulationType::Qam64 => generate_qam(8),
        ModulationType::Qam256 => generate_qam(16),
    }
}

/// Generate a square QAM constellation with `side × side` points, normalised to
/// unit average power.
fn generate_qam(side: usize) -> Vec<(f64, f64)> {
    let mut pts = Vec::with_capacity(side * side);
    let half = (side as f64 - 1.0) / 2.0;
    for i in 0..side {
        for q in 0..side {
            pts.push((i as f64 - half, q as f64 - half));
        }
    }
    // Normalise to unit average power
    let avg_power: f64 = pts.iter().map(|p| p.0 * p.0 + p.1 * p.1).sum::<f64>() / pts.len() as f64;
    let scale = 1.0 / avg_power.sqrt();
    for pt in &mut pts {
        pt.0 *= scale;
        pt.1 *= scale;
    }
    pts
}

// ---------------------------------------------------------------------------
// Minimal radix-2 DIT FFT (std-only, no external crate)
// ---------------------------------------------------------------------------

/// Compute power spectrum (magnitude²) via in-place radix-2 FFT.
/// Input is zero-padded to `n` (must be a power of two).
fn fft_power(samples: &[(f64, f64)], n: usize) -> Vec<f64> {
    debug_assert!(n.is_power_of_two());

    // Copy input into work buffer, zero-pad
    let mut re = vec![0.0_f64; n];
    let mut im = vec![0.0_f64; n];
    for (i, &(r, j)) in samples.iter().enumerate().take(n) {
        re[i] = r;
        im[i] = j;
    }

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
            re.swap(i, j);
            im.swap(i, j);
        }
    }

    // Cooley-Tukey butterflies
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = -2.0 * PI / len as f64;
        let wn_re = angle.cos();
        let wn_im = angle.sin();
        let mut start = 0;
        while start < n {
            let mut w_re = 1.0;
            let mut w_im = 0.0;
            for k in 0..half {
                let a = start + k;
                let b = a + half;
                let tr = w_re * re[b] - w_im * im[b];
                let ti = w_re * im[b] + w_im * re[b];
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
                let new_w_re = w_re * wn_re - w_im * wn_im;
                let new_w_im = w_re * wn_im + w_im * wn_re;
                w_re = new_w_re;
                w_im = new_w_im;
            }
            start += len;
        }
        len <<= 1;
    }

    // Power spectrum (unnormalised)
    re.iter()
        .zip(im.iter())
        .map(|(r, i)| r * r + i * i)
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_1_SQRT_2;

    const EPS: f64 = 1e-6;

    // -- constellation generation -------------------------------------------

    #[test]
    fn test_bpsk_constellation() {
        let pts = generate_constellation(ModulationType::Bpsk);
        assert_eq!(pts.len(), 2);
        assert!((pts[0].0 - 1.0).abs() < EPS);
        assert!((pts[1].0 + 1.0).abs() < EPS);
    }

    #[test]
    fn test_qpsk_constellation() {
        let pts = generate_constellation(ModulationType::Qpsk);
        assert_eq!(pts.len(), 4);
        // All points should have unit magnitude
        for p in &pts {
            let mag = (p.0 * p.0 + p.1 * p.1).sqrt();
            assert!((mag - 1.0).abs() < 1e-10, "QPSK point magnitude should be 1.0, got {}", mag);
        }
    }

    #[test]
    fn test_8psk_constellation() {
        let pts = generate_constellation(ModulationType::Eightpsk);
        assert_eq!(pts.len(), 8);
        for p in &pts {
            let mag = (p.0 * p.0 + p.1 * p.1).sqrt();
            assert!((mag - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_qam16_constellation() {
        let pts = generate_constellation(ModulationType::Qam16);
        assert_eq!(pts.len(), 16);
        // Average power should be ~1.0 after normalisation
        let avg: f64 = pts.iter().map(|p| p.0 * p.0 + p.1 * p.1).sum::<f64>() / pts.len() as f64;
        assert!((avg - 1.0).abs() < 0.01, "QAM16 avg power {} != 1.0", avg);
    }

    #[test]
    fn test_qam64_constellation() {
        let pts = generate_constellation(ModulationType::Qam64);
        assert_eq!(pts.len(), 64);
        let avg: f64 = pts.iter().map(|p| p.0 * p.0 + p.1 * p.1).sum::<f64>() / pts.len() as f64;
        assert!((avg - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_qam256_constellation() {
        let pts = generate_constellation(ModulationType::Qam256);
        assert_eq!(pts.len(), 256);
        let avg: f64 = pts.iter().map(|p| p.0 * p.0 + p.1 * p.1).sum::<f64>() / pts.len() as f64;
        assert!((avg - 1.0).abs() < 0.01);
    }

    // -- EVM ----------------------------------------------------------------

    #[test]
    fn test_evm_ideal_qpsk() {
        let vsa = VectorSignalAnalyzer::new(ModulationType::Qpsk);
        let s = FRAC_1_SQRT_2;
        let samples = vec![(s, s), (-s, s), (-s, -s), (s, -s)];
        let evm = vsa.measure_evm(&samples);
        assert!(evm.evm_rms_percent < 1e-10, "Ideal QPSK EVM should be ~0%");
        assert!(evm.evm_peak_percent < 1e-10);
        assert_eq!(evm.evm_per_symbol.len(), 4);
    }

    #[test]
    fn test_evm_noisy_bpsk() {
        let vsa = VectorSignalAnalyzer::new(ModulationType::Bpsk);
        // Add known offset to BPSK symbols
        let noise = 0.05;
        let samples = vec![
            (1.0 + noise, noise),
            (-1.0 - noise, -noise),
            (1.0 + noise, noise),
        ];
        let evm = vsa.measure_evm(&samples);
        assert!(evm.evm_rms_percent > 0.0, "Noisy symbols should have non-zero EVM");
        assert!(evm.evm_rms_percent < 20.0, "Small noise should give small EVM");
    }

    #[test]
    fn test_evm_empty_input() {
        let vsa = VectorSignalAnalyzer::new(ModulationType::Bpsk);
        let evm = vsa.measure_evm(&[]);
        assert_eq!(evm.evm_rms_percent, 0.0);
        assert!(evm.evm_per_symbol.is_empty());
    }

    // -- MER ----------------------------------------------------------------

    #[test]
    fn test_mer_ideal_qpsk() {
        let vsa = VectorSignalAnalyzer::new(ModulationType::Qpsk);
        let s = FRAC_1_SQRT_2;
        let samples = vec![(s, s), (-s, s), (-s, -s), (s, -s)];
        let mer = vsa.measure_mer(&samples);
        assert!(mer >= 99.0, "Ideal QPSK MER should be capped at 100 dB, got {}", mer);
    }

    #[test]
    fn test_mer_noisy() {
        let vsa = VectorSignalAnalyzer::new(ModulationType::Bpsk);
        let samples = vec![(1.05, 0.02), (-0.97, -0.03), (1.01, 0.01)];
        let mer = vsa.measure_mer(&samples);
        assert!(mer > 10.0 && mer < 100.0, "MER should be moderate, got {}", mer);
    }

    // -- IQ imbalance -------------------------------------------------------

    #[test]
    fn test_iq_imbalance_balanced() {
        let vsa = VectorSignalAnalyzer::new(ModulationType::Qpsk);
        let s = FRAC_1_SQRT_2;
        let samples = vec![(s, s), (-s, s), (-s, -s), (s, -s)];
        let (amp_db, phase_deg) = vsa.measure_iq_imbalance(&samples);
        assert!(amp_db.abs() < 0.1, "Balanced QPSK should have ~0 dB imbalance, got {}", amp_db);
        assert!(phase_deg.abs() < 1.0, "Balanced QPSK should have ~0° phase error, got {}", phase_deg);
    }

    #[test]
    fn test_iq_imbalance_amplitude() {
        let vsa = VectorSignalAnalyzer::new(ModulationType::Bpsk);
        // I rail at 1.0, Q rail at 0.5 → amplitude imbalance
        let samples = vec![(1.0, 0.5), (-1.0, -0.5), (1.0, 0.5), (-1.0, -0.5)];
        let (amp_db, _) = vsa.measure_iq_imbalance(&samples);
        // 20*log10(1.0/0.5) = 6.02 dB
        assert!((amp_db - 6.02).abs() < 0.1, "Expected ~6 dB imbalance, got {}", amp_db);
    }

    // -- ACPR ---------------------------------------------------------------

    #[test]
    fn test_acpr_tone() {
        // A single DC tone should have all power in the main channel
        let mut vsa = VectorSignalAnalyzer::new(ModulationType::Bpsk);
        vsa.set_sample_rate(100.0);
        let samples: Vec<(f64, f64)> = (0..256).map(|_| (1.0, 0.0)).collect();
        let acpr = vsa.measure_acpr(&samples, 10.0, 20.0);
        // Adjacent channels should be well below main
        assert!(acpr.lower_acpr_db < -20.0, "Lower ACPR should be very negative, got {}", acpr.lower_acpr_db);
        assert!(acpr.upper_acpr_db < -20.0, "Upper ACPR should be very negative, got {}", acpr.upper_acpr_db);
    }

    // -- Full report --------------------------------------------------------

    #[test]
    fn test_analyze_report() {
        let mut vsa = VectorSignalAnalyzer::new(ModulationType::Bpsk);
        // All same symbol → zero EVM, zero CFO
        let samples: Vec<(f64, f64)> = vec![(1.0, 0.0); 64];
        let report = vsa.analyze(&samples);
        assert!(report.evm.evm_rms_percent < 1e-6);
        assert!(report.mer_db >= 99.0);
        assert!(report.carrier_freq_offset_hz.abs() < 1e-10,
            "CFO should be ~0, got {}", report.carrier_freq_offset_hz);
    }

    // -- FFT ----------------------------------------------------------------

    #[test]
    fn test_fft_power_dc() {
        // DC signal → all power in bin 0
        let samples: Vec<(f64, f64)> = vec![(1.0, 0.0); 16];
        let pwr = fft_power(&samples, 16);
        assert!((pwr[0] - 256.0).abs() < 1e-6, "DC bin should be N²=256, got {}", pwr[0]);
        for &p in &pwr[1..] {
            assert!(p < 1e-10, "Non-DC bin should be ~0");
        }
    }
}

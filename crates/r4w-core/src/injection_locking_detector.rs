//! Detection and mitigation of injection-locked oscillator interference in receivers.
//!
//! Injection locking occurs when an external signal couples into a local oscillator,
//! pulling its frequency and degrading phase noise performance. This module detects
//! such coupling and provides mitigation strategies.
//!
//! # Example
//!
//! ```
//! use r4w_core::injection_locking_detector::{InjectionLockDetector, DetectorConfig};
//!
//! let config = DetectorConfig {
//!     sample_rate: 1_000_000.0,
//!     fft_size: 1024,
//!     phase_noise_threshold_dbc: -80.0,
//!     freq_pulling_threshold_hz: 50.0,
//! };
//! let detector = InjectionLockDetector::new(config);
//!
//! // Generate a simple test signal (tone at DC with slight offset)
//! let n = 1024;
//! let signal: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / 1_000_000.0;
//!         let phase = 2.0 * std::f64::consts::PI * 100.0 * t;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let status = detector.detect(&signal, 0.0);
//! assert!(status.confidence >= 0.0 && status.confidence <= 1.0);
//! ```

use std::f64::consts::PI;

/// Configuration for the injection lock detector.
#[derive(Debug, Clone)]
pub struct DetectorConfig {
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// FFT size for spectral analysis.
    pub fft_size: usize,
    /// Phase noise threshold in dBc below which injection locking is suspected.
    pub phase_noise_threshold_dbc: f64,
    /// Frequency pulling threshold in Hz above which locking is detected.
    pub freq_pulling_threshold_hz: f64,
}

/// Status result from injection lock detection.
#[derive(Debug, Clone)]
pub struct LockStatus {
    /// Whether injection locking is detected.
    pub is_locked: bool,
    /// Estimated frequency pulling in Hz.
    pub pulling_freq_hz: f64,
    /// Estimated lock range in Hz.
    pub lock_range_hz: f64,
    /// Excess phase noise in dB above expected floor.
    pub phase_noise_excess_db: f64,
    /// Detection confidence from 0.0 to 1.0.
    pub confidence: f64,
}

/// Countermeasure strategy for injection locking.
#[derive(Debug, Clone, PartialEq)]
pub enum Countermeasure {
    /// No mitigation needed.
    None,
    /// Apply an adaptive notch filter at the injection frequency.
    AdaptiveNotch,
    /// Apply frequency dithering to break lock.
    FreqDithering,
    /// Alert that physical isolation is insufficient.
    IsolationAlert,
}

/// Detector for injection-locked oscillator interference.
///
/// Uses phase noise analysis and instantaneous frequency estimation to identify
/// parasitic oscillator coupling in received signals.
pub struct InjectionLockDetector {
    config: DetectorConfig,
}

impl InjectionLockDetector {
    /// Create a new detector with the given configuration.
    pub fn new(config: DetectorConfig) -> Self {
        Self { config }
    }

    /// Detect injection locking in a signal relative to a nominal frequency.
    ///
    /// Analyzes the signal for signs of oscillator pulling and excess phase noise
    /// that indicate injection locking.
    pub fn detect(&self, signal: &[(f64, f64)], nominal_freq: f64) -> LockStatus {
        if signal.len() < 2 {
            return LockStatus {
                is_locked: false,
                pulling_freq_hz: 0.0,
                lock_range_hz: 0.0,
                phase_noise_excess_db: 0.0,
                confidence: 0.0,
            };
        }

        let pulling = self.frequency_pulling(signal, nominal_freq);
        let phase_noise = self.phase_noise_spectrum(signal);
        let excess_db = self.compute_phase_noise_excess(&phase_noise);

        let pulling_detected = pulling.abs() > self.config.freq_pulling_threshold_hz;
        let noise_detected = excess_db > self.config.phase_noise_threshold_dbc.abs();

        // Confidence based on how far pulling and noise exceed thresholds
        let pulling_ratio = if self.config.freq_pulling_threshold_hz > 0.0 {
            (pulling.abs() / self.config.freq_pulling_threshold_hz).min(2.0) / 2.0
        } else {
            0.0
        };
        let noise_ratio = if self.config.phase_noise_threshold_dbc.abs() > 0.0 {
            (excess_db / self.config.phase_noise_threshold_dbc.abs()).min(2.0) / 2.0
        } else {
            0.0
        };
        let confidence = (pulling_ratio * 0.6 + noise_ratio * 0.4).clamp(0.0, 1.0);

        let is_locked = pulling_detected || noise_detected;

        LockStatus {
            is_locked,
            pulling_freq_hz: pulling,
            lock_range_hz: pulling.abs() * 2.0,
            phase_noise_excess_db: excess_db,
            confidence,
        }
    }

    /// Compute the phase noise spectrum of a signal.
    ///
    /// Returns a vector of (offset_hz, dBc/Hz) pairs representing the
    /// single-sideband phase noise power spectral density.
    pub fn phase_noise_spectrum(&self, signal: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = self.config.fft_size.min(signal.len());
        if n == 0 {
            return vec![];
        }

        // Extract instantaneous phase
        let phases: Vec<f64> = signal[..n]
            .iter()
            .map(|(i, q)| q.atan2(*i))
            .collect();

        // Remove linear phase trend (carrier)
        let detrended = detrend_phase(&phases);

        // Compute FFT of phase deviation via DFT
        let spectrum = dft(&detrended.iter().map(|&p| (p, 0.0)).collect::<Vec<_>>());

        // Convert to dBc/Hz
        let freq_res = self.config.sample_rate / n as f64;
        let carrier_power = signal[..n]
            .iter()
            .map(|(i, q)| i * i + q * q)
            .sum::<f64>()
            / n as f64;

        let carrier_power_db = if carrier_power > 0.0 {
            10.0 * carrier_power.log10()
        } else {
            -200.0
        };

        let mut result = Vec::with_capacity(n / 2);
        for k in 1..n / 2 {
            let offset_hz = k as f64 * freq_res;
            let mag_sq = spectrum[k].0 * spectrum[k].0 + spectrum[k].1 * spectrum[k].1;
            let psd = if mag_sq > 0.0 {
                10.0 * (mag_sq / (n as f64 * n as f64)).log10() - carrier_power_db
                    - 10.0 * freq_res.log10()
            } else {
                -200.0
            };
            result.push((offset_hz, psd));
        }

        result
    }

    /// Estimate the injection lock range using Adler's equation.
    ///
    /// `delta_f = f0 / (2*Q) * sqrt(P_inj / P_osc)`
    ///
    /// # Arguments
    /// * `q_factor` - Quality factor of the oscillator
    /// * `p_inj_dbm` - Injected signal power in dBm
    /// * `p_osc_dbm` - Oscillator power in dBm
    ///
    /// # Returns
    /// Lock range in Hz
    pub fn lock_range_estimate(&self, q_factor: f64, p_inj_dbm: f64, p_osc_dbm: f64) -> f64 {
        if q_factor <= 0.0 {
            return 0.0;
        }
        // Convert dBm to linear power ratio
        let p_ratio = 10.0_f64.powf((p_inj_dbm - p_osc_dbm) / 10.0);
        if p_ratio < 0.0 {
            return 0.0;
        }
        // Use center of Nyquist band as f0 proxy
        let f0 = self.config.sample_rate / 2.0;
        f0 / (2.0 * q_factor) * p_ratio.sqrt()
    }

    /// Apply mitigation to a signal based on detected lock status.
    ///
    /// Returns the mitigated signal. The mitigation strategy is chosen based on
    /// the lock status severity.
    pub fn mitigate(
        &self,
        signal: &[(f64, f64)],
        status: &LockStatus,
    ) -> Vec<(f64, f64)> {
        let countermeasure = self.recommend_countermeasure(status);
        match countermeasure {
            Countermeasure::None => signal.to_vec(),
            Countermeasure::AdaptiveNotch => {
                self.apply_adaptive_notch(signal, status.pulling_freq_hz)
            }
            Countermeasure::FreqDithering => self.apply_freq_dithering(signal),
            Countermeasure::IsolationAlert => {
                // Cannot fix in DSP; return signal as-is but status carries the alert
                signal.to_vec()
            }
        }
    }

    /// Estimate frequency pulling by measuring the deviation of instantaneous
    /// frequency from the nominal frequency.
    ///
    /// Returns the average frequency deviation in Hz.
    pub fn frequency_pulling(
        &self,
        signal: &[(f64, f64)],
        nominal_freq: f64,
    ) -> f64 {
        if signal.len() < 2 {
            return 0.0;
        }

        // Instantaneous frequency via phase difference
        let inst_freqs = instantaneous_frequency(signal, self.config.sample_rate);
        if inst_freqs.is_empty() {
            return 0.0;
        }

        let mean_freq: f64 = inst_freqs.iter().sum::<f64>() / inst_freqs.len() as f64;
        mean_freq - nominal_freq
    }

    /// Recommend a countermeasure based on lock status.
    fn recommend_countermeasure(&self, status: &LockStatus) -> Countermeasure {
        if !status.is_locked {
            return Countermeasure::None;
        }
        if status.confidence > 0.8 && status.phase_noise_excess_db > 20.0 {
            Countermeasure::IsolationAlert
        } else if status.pulling_freq_hz.abs() > self.config.freq_pulling_threshold_hz {
            Countermeasure::AdaptiveNotch
        } else {
            Countermeasure::FreqDithering
        }
    }

    /// Apply an adaptive IIR notch filter at the detected injection frequency.
    fn apply_adaptive_notch(&self, signal: &[(f64, f64)], notch_freq: f64) -> Vec<(f64, f64)> {
        if signal.is_empty() || self.config.sample_rate <= 0.0 {
            return signal.to_vec();
        }

        // Normalized frequency
        let w0 = 2.0 * PI * notch_freq.abs() / self.config.sample_rate;

        // Notch bandwidth parameter (narrow notch)
        let r = 0.95;

        // IIR notch: H(z) = (1 - 2*cos(w0)*z^-1 + z^-2) / (1 - 2*r*cos(w0)*z^-1 + r^2*z^-2)
        let cos_w0 = w0.cos();
        let b0 = 1.0;
        let b1 = -2.0 * cos_w0;
        let b2 = 1.0;
        let a1 = -2.0 * r * cos_w0;
        let a2 = r * r;

        let mut output = Vec::with_capacity(signal.len());
        // State for I and Q channels independently
        let mut xi1 = 0.0_f64;
        let mut xi2 = 0.0_f64;
        let mut yi1 = 0.0_f64;
        let mut yi2 = 0.0_f64;
        let mut xq1 = 0.0_f64;
        let mut xq2 = 0.0_f64;
        let mut yq1 = 0.0_f64;
        let mut yq2 = 0.0_f64;

        for &(si, sq) in signal {
            let yi = b0 * si + b1 * xi1 + b2 * xi2 - a1 * yi1 - a2 * yi2;
            xi2 = xi1;
            xi1 = si;
            yi2 = yi1;
            yi1 = yi;

            let yq = b0 * sq + b1 * xq1 + b2 * xq2 - a1 * yq1 - a2 * yq2;
            xq2 = xq1;
            xq1 = sq;
            yq2 = yq1;
            yq1 = yq;

            output.push((yi, yq));
        }

        output
    }

    /// Apply frequency dithering to break injection lock.
    fn apply_freq_dithering(&self, signal: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if signal.is_empty() {
            return vec![];
        }

        // Apply a small pseudo-random phase modulation to break coherence
        let dither_bw = 100.0; // Hz dither bandwidth
        let mut output = Vec::with_capacity(signal.len());

        for (i, &(si, sq)) in signal.iter().enumerate() {
            let t = i as f64 / self.config.sample_rate;
            // Triangular dither pattern
            let dither_phase = dither_bw * 2.0 * PI * t
                * (2.0 * PI * 37.0 * t).sin(); // 37 Hz modulation rate
            let cos_d = dither_phase.cos();
            let sin_d = dither_phase.sin();
            output.push((
                si * cos_d - sq * sin_d,
                si * sin_d + sq * cos_d,
            ));
        }

        output
    }

    /// Compute excess phase noise relative to a flat noise floor estimate.
    fn compute_phase_noise_excess(&self, spectrum: &[(f64, f64)]) -> f64 {
        if spectrum.is_empty() {
            return 0.0;
        }

        let values: Vec<f64> = spectrum.iter().map(|(_, db)| *db).collect();

        // Estimate noise floor as median of the spectrum
        let mut sorted = values.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let floor = sorted[sorted.len() / 2];

        // Peak value
        let peak = values
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);

        // Excess is how much the peak exceeds the floor
        (peak - floor).max(0.0)
    }
}

/// Compute instantaneous frequency from IQ samples via phase differencing.
fn instantaneous_frequency(signal: &[(f64, f64)], sample_rate: f64) -> Vec<f64> {
    if signal.len() < 2 {
        return vec![];
    }

    let mut freqs = Vec::with_capacity(signal.len() - 1);
    for i in 1..signal.len() {
        let (i0, q0) = signal[i - 1];
        let (i1, q1) = signal[i];

        // Conjugate multiply: s[n] * conj(s[n-1])
        let re = i1 * i0 + q1 * q0;
        let im = q1 * i0 - i1 * q0;

        let dphi = im.atan2(re);
        let freq = dphi * sample_rate / (2.0 * PI);
        freqs.push(freq);
    }

    freqs
}

/// Remove linear phase trend (carrier) from phase sequence.
fn detrend_phase(phases: &[f64]) -> Vec<f64> {
    let n = phases.len();
    if n < 2 {
        return phases.to_vec();
    }

    // Unwrap phase first
    let mut unwrapped = vec![phases[0]];
    for i in 1..n {
        let mut diff = phases[i] - phases[i - 1];
        while diff > PI {
            diff -= 2.0 * PI;
        }
        while diff < -PI {
            diff += 2.0 * PI;
        }
        unwrapped.push(unwrapped[i - 1] + diff);
    }

    // Linear regression: y = a + b*x
    let n_f = n as f64;
    let sum_x: f64 = (0..n).map(|i| i as f64).sum();
    let sum_y: f64 = unwrapped.iter().sum();
    let sum_xy: f64 = (0..n).map(|i| i as f64 * unwrapped[i]).sum();
    let sum_x2: f64 = (0..n).map(|i| (i as f64) * (i as f64)).sum();

    let denom = n_f * sum_x2 - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return unwrapped;
    }

    let b = (n_f * sum_xy - sum_x * sum_y) / denom;
    let a = (sum_y - b * sum_x) / n_f;

    unwrapped
        .iter()
        .enumerate()
        .map(|(i, &p)| p - (a + b * i as f64))
        .collect()
}

/// Simple DFT (not FFT) for small sizes. Used for phase noise analysis.
fn dft(signal: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = signal.len();
    let mut result = vec![(0.0, 0.0); n];

    for k in 0..n {
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &(sr, si)) in signal.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            re += sr * cos_a - si * sin_a;
            im += sr * sin_a + si * cos_a;
        }
        result[k] = (re, im);
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> DetectorConfig {
        DetectorConfig {
            sample_rate: 1_000_000.0,
            fft_size: 256,
            phase_noise_threshold_dbc: -80.0,
            freq_pulling_threshold_hz: 50.0,
        }
    }

    fn generate_tone(freq: f64, sample_rate: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * freq * t;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    #[test]
    fn test_new_detector() {
        let config = default_config();
        let detector = InjectionLockDetector::new(config.clone());
        assert_eq!(detector.config.sample_rate, 1_000_000.0);
        assert_eq!(detector.config.fft_size, 256);
    }

    #[test]
    fn test_detect_clean_signal_no_lock() {
        let config = default_config();
        let detector = InjectionLockDetector::new(config);
        let signal = generate_tone(1000.0, 1_000_000.0, 1024);
        let status = detector.detect(&signal, 1000.0);
        // Clean tone at nominal freq should not show significant pulling
        assert!(
            status.pulling_freq_hz.abs() < 100.0,
            "pulling={} should be small",
            status.pulling_freq_hz
        );
    }

    #[test]
    fn test_detect_pulled_signal() {
        let config = DetectorConfig {
            sample_rate: 1_000_000.0,
            fft_size: 256,
            phase_noise_threshold_dbc: -80.0,
            freq_pulling_threshold_hz: 50.0,
        };
        let detector = InjectionLockDetector::new(config);
        // Signal at 1200 Hz but nominal is 1000 Hz -> 200 Hz pulling
        let signal = generate_tone(1200.0, 1_000_000.0, 4096);
        let status = detector.detect(&signal, 1000.0);
        assert!(
            (status.pulling_freq_hz - 200.0).abs() < 20.0,
            "Expected ~200 Hz pulling, got {}",
            status.pulling_freq_hz
        );
        assert!(status.is_locked);
    }

    #[test]
    fn test_detect_empty_signal() {
        let detector = InjectionLockDetector::new(default_config());
        let status = detector.detect(&[], 1000.0);
        assert!(!status.is_locked);
        assert_eq!(status.confidence, 0.0);
    }

    #[test]
    fn test_detect_single_sample() {
        let detector = InjectionLockDetector::new(default_config());
        let status = detector.detect(&[(1.0, 0.0)], 1000.0);
        assert!(!status.is_locked);
        assert_eq!(status.pulling_freq_hz, 0.0);
    }

    #[test]
    fn test_frequency_pulling_zero_offset() {
        let detector = InjectionLockDetector::new(default_config());
        let signal = generate_tone(5000.0, 1_000_000.0, 4096);
        let pulling = detector.frequency_pulling(&signal, 5000.0);
        assert!(
            pulling.abs() < 10.0,
            "Expected near-zero pulling, got {}",
            pulling
        );
    }

    #[test]
    fn test_frequency_pulling_known_offset() {
        let detector = InjectionLockDetector::new(default_config());
        let signal = generate_tone(5500.0, 1_000_000.0, 4096);
        let pulling = detector.frequency_pulling(&signal, 5000.0);
        assert!(
            (pulling - 500.0).abs() < 20.0,
            "Expected ~500 Hz pulling, got {}",
            pulling
        );
    }

    #[test]
    fn test_frequency_pulling_negative() {
        let detector = InjectionLockDetector::new(default_config());
        let signal = generate_tone(4500.0, 1_000_000.0, 4096);
        let pulling = detector.frequency_pulling(&signal, 5000.0);
        assert!(
            (pulling - (-500.0)).abs() < 20.0,
            "Expected ~-500 Hz pulling, got {}",
            pulling
        );
    }

    #[test]
    fn test_lock_range_estimate_adler() {
        let config = DetectorConfig {
            sample_rate: 10_000_000.0, // 10 MHz -> f0 = 5 MHz
            fft_size: 256,
            phase_noise_threshold_dbc: -80.0,
            freq_pulling_threshold_hz: 50.0,
        };
        let detector = InjectionLockDetector::new(config);
        // Q=100, equal power => delta_f = f0/(2*Q) * 1.0 = 5e6/200 = 25000 Hz
        let range = detector.lock_range_estimate(100.0, 0.0, 0.0);
        assert!(
            (range - 25000.0).abs() < 1.0,
            "Expected 25000 Hz, got {}",
            range
        );
    }

    #[test]
    fn test_lock_range_estimate_power_difference() {
        let config = DetectorConfig {
            sample_rate: 10_000_000.0,
            fft_size: 256,
            phase_noise_threshold_dbc: -80.0,
            freq_pulling_threshold_hz: 50.0,
        };
        let detector = InjectionLockDetector::new(config);
        // P_inj 20 dB below P_osc => ratio = 0.01 => sqrt = 0.1
        // delta_f = 5e6/(2*100) * 0.1 = 2500 Hz
        let range = detector.lock_range_estimate(100.0, -20.0, 0.0);
        assert!(
            (range - 2500.0).abs() < 1.0,
            "Expected 2500 Hz, got {}",
            range
        );
    }

    #[test]
    fn test_lock_range_zero_q() {
        let detector = InjectionLockDetector::new(default_config());
        let range = detector.lock_range_estimate(0.0, 0.0, 0.0);
        assert_eq!(range, 0.0);
    }

    #[test]
    fn test_phase_noise_spectrum_returns_correct_length() {
        let detector = InjectionLockDetector::new(default_config());
        let signal = generate_tone(1000.0, 1_000_000.0, 512);
        let pn = detector.phase_noise_spectrum(&signal);
        // fft_size=256, so we expect 256/2 - 1 = 127 bins
        assert_eq!(pn.len(), 127);
    }

    #[test]
    fn test_phase_noise_spectrum_offsets_positive() {
        let detector = InjectionLockDetector::new(default_config());
        let signal = generate_tone(1000.0, 1_000_000.0, 512);
        let pn = detector.phase_noise_spectrum(&signal);
        for (offset, _) in &pn {
            assert!(*offset > 0.0, "Offset should be positive, got {}", offset);
        }
    }

    #[test]
    fn test_phase_noise_spectrum_empty_signal() {
        let detector = InjectionLockDetector::new(default_config());
        let pn = detector.phase_noise_spectrum(&[]);
        assert!(pn.is_empty());
    }

    #[test]
    fn test_mitigate_no_lock() {
        let detector = InjectionLockDetector::new(default_config());
        let signal = generate_tone(1000.0, 1_000_000.0, 256);
        let status = LockStatus {
            is_locked: false,
            pulling_freq_hz: 0.0,
            lock_range_hz: 0.0,
            phase_noise_excess_db: 0.0,
            confidence: 0.0,
        };
        let mitigated = detector.mitigate(&signal, &status);
        // No lock -> signal returned unchanged
        assert_eq!(mitigated.len(), signal.len());
        for (orig, mit) in signal.iter().zip(mitigated.iter()) {
            assert!((orig.0 - mit.0).abs() < 1e-12);
            assert!((orig.1 - mit.1).abs() < 1e-12);
        }
    }

    #[test]
    fn test_mitigate_adaptive_notch() {
        let detector = InjectionLockDetector::new(default_config());
        let signal = generate_tone(1000.0, 1_000_000.0, 512);
        let status = LockStatus {
            is_locked: true,
            pulling_freq_hz: 200.0,
            lock_range_hz: 400.0,
            phase_noise_excess_db: 5.0,
            confidence: 0.5,
        };
        let mitigated = detector.mitigate(&signal, &status);
        assert_eq!(mitigated.len(), signal.len());
        // Notch filter should modify the signal
        let differs = signal
            .iter()
            .zip(mitigated.iter())
            .any(|(a, b)| (a.0 - b.0).abs() > 1e-10 || (a.1 - b.1).abs() > 1e-10);
        assert!(differs, "Notch filter should modify the signal");
    }

    #[test]
    fn test_mitigate_preserves_length() {
        let detector = InjectionLockDetector::new(default_config());
        let signal = generate_tone(1000.0, 1_000_000.0, 1024);
        let status = LockStatus {
            is_locked: true,
            pulling_freq_hz: 500.0,
            lock_range_hz: 1000.0,
            phase_noise_excess_db: 30.0,
            confidence: 0.9,
        };
        let mitigated = detector.mitigate(&signal, &status);
        assert_eq!(mitigated.len(), signal.len());
    }

    #[test]
    fn test_countermeasure_selection() {
        let detector = InjectionLockDetector::new(default_config());

        // No lock
        let status_none = LockStatus {
            is_locked: false,
            pulling_freq_hz: 0.0,
            lock_range_hz: 0.0,
            phase_noise_excess_db: 0.0,
            confidence: 0.0,
        };
        assert_eq!(
            detector.recommend_countermeasure(&status_none),
            Countermeasure::None
        );

        // Severe lock -> IsolationAlert
        let status_severe = LockStatus {
            is_locked: true,
            pulling_freq_hz: 1000.0,
            lock_range_hz: 2000.0,
            phase_noise_excess_db: 30.0,
            confidence: 0.9,
        };
        assert_eq!(
            detector.recommend_countermeasure(&status_severe),
            Countermeasure::IsolationAlert
        );

        // Moderate pulling -> AdaptiveNotch
        let status_moderate = LockStatus {
            is_locked: true,
            pulling_freq_hz: 200.0,
            lock_range_hz: 400.0,
            phase_noise_excess_db: 5.0,
            confidence: 0.5,
        };
        assert_eq!(
            detector.recommend_countermeasure(&status_moderate),
            Countermeasure::AdaptiveNotch
        );
    }

    #[test]
    fn test_confidence_range() {
        let detector = InjectionLockDetector::new(default_config());
        let signal = generate_tone(1000.0, 1_000_000.0, 512);

        for offset in [0.0, 10.0, 100.0, 500.0, 5000.0] {
            let status = detector.detect(&signal, 1000.0 - offset);
            assert!(
                status.confidence >= 0.0 && status.confidence <= 1.0,
                "Confidence {} out of range for offset {}",
                status.confidence,
                offset
            );
        }
    }

    #[test]
    fn test_instantaneous_frequency_dc() {
        // DC signal (0 Hz) should give ~0 Hz instantaneous frequency
        let signal: Vec<(f64, f64)> = vec![(1.0, 0.0); 100];
        let freqs = instantaneous_frequency(&signal, 1_000_000.0);
        assert_eq!(freqs.len(), 99);
        for f in &freqs {
            assert!(f.abs() < 1e-6, "DC signal should have 0 Hz inst freq, got {}", f);
        }
    }

    #[test]
    fn test_instantaneous_frequency_known_tone() {
        let fs = 1_000_000.0;
        let freq = 10_000.0;
        let signal = generate_tone(freq, fs, 1000);
        let freqs = instantaneous_frequency(&signal, fs);
        let mean: f64 = freqs.iter().sum::<f64>() / freqs.len() as f64;
        assert!(
            (mean - freq).abs() < 5.0,
            "Expected inst freq ~{}, got {}",
            freq,
            mean
        );
    }

    #[test]
    fn test_detrend_phase_removes_linear() {
        // Linear phase ramp should be removed
        let phases: Vec<f64> = (0..100).map(|i| 0.01 * i as f64).collect();
        let detrended = detrend_phase(&phases);
        let max_residual = detrended
            .iter()
            .map(|x| x.abs())
            .fold(0.0_f64, f64::max);
        assert!(
            max_residual < 1e-10,
            "Detrended linear phase should be ~0, max={}",
            max_residual
        );
    }

    #[test]
    fn test_dft_single_tone() {
        let n = 64;
        let k0 = 5; // tone at bin 5
        let signal: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * k0 as f64 * i as f64 / n as f64;
                (phase.cos(), phase.sin())
            })
            .collect();
        let spectrum = dft(&signal);

        // Bin k0 should have magnitude ~N, others ~0
        let mag_k0 = (spectrum[k0].0.powi(2) + spectrum[k0].1.powi(2)).sqrt();
        assert!(
            (mag_k0 - n as f64).abs() < 1e-6,
            "Expected mag={}, got {}",
            n,
            mag_k0
        );

        // Check that other bins are near zero
        for (k, &(re, im)) in spectrum.iter().enumerate() {
            if k != k0 {
                let mag = (re * re + im * im).sqrt();
                assert!(
                    mag < 1e-6,
                    "Bin {} should be ~0, got mag={}",
                    k,
                    mag
                );
            }
        }
    }
}

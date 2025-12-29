//! Synchronization for LoRa Reception
//!
//! This module handles the critical task of synchronizing to an incoming
//! LoRa signal before demodulation can begin.
//!
//! ## Synchronization Challenges
//!
//! When receiving a LoRa signal, we need to determine:
//!
//! 1. **Timing**: Where does the packet start? (which sample?)
//! 2. **Frequency Offset**: How far off is the carrier from nominal? (CFO)
//! 3. **Spreading Factor**: What SF is being used? (if unknown)
//!
//! ## Preamble Structure
//!
//! LoRa uses a distinctive preamble for synchronization:
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │ N Upchirps │ Sync Words │ 2.25 Downchirps │ Header+Payload  │
//! └─────────────────────────────────────────────────────────────┘
//!      (8)         (2)           (2.25)
//!
//! The pattern of upchirps followed by downchirps creates a
//! distinctive signature that's easy to detect.
//! ```
//!
//! ## CFO Estimation
//!
//! When we mix the received signal with a reference downchirp during
//! the upchirp portion of the preamble:
//!
//! ```text
//! f_tone = CFO - B*τ/T    (during upchirps)
//! f_tone = CFO + B*τ/T    (during downchirps)
//!
//! Therefore:
//! CFO = (f_up + f_down) / 2
//! τ = T/B * (f_down - f_up) / 2
//! ```

use crate::chirp::ChirpGenerator;
use crate::fft_utils::FftProcessor;
use crate::params::LoRaParams;
use crate::types::{Complex, DspError, DspResult, IQSample};

use std::f64::consts::PI;

/// Preamble detector state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectorState {
    /// Searching for preamble
    Searching,
    /// Detected potential preamble, confirming
    Confirming,
    /// Preamble confirmed, synchronizing
    Synchronizing,
    /// Synchronized, ready for payload
    Synchronized,
}

/// Result of preamble detection
#[derive(Debug, Clone)]
pub struct DetectionResult {
    /// Whether a valid preamble was detected
    pub detected: bool,
    /// Sample offset to start of packet
    pub timing_offset: usize,
    /// Estimated carrier frequency offset in Hz
    pub cfo_hz: f64,
    /// Estimated timing offset within a symbol (fractional)
    pub fractional_timing: f64,
    /// Detected spreading factor (if auto-detect enabled)
    pub spreading_factor: Option<u8>,
    /// Signal strength estimate
    pub rssi: f64,
    /// SNR estimate in dB
    pub snr_db: f64,
}

/// Preamble detector for LoRa
///
/// Continuously processes samples looking for a valid LoRa preamble.
#[derive(Debug)]
pub struct PreambleDetector {
    /// LoRa parameters
    params: LoRaParams,
    /// Chirp generator for reference chirps
    chirp_gen: ChirpGenerator,
    /// FFT processor
    fft: FftProcessor,
    /// Current detector state
    state: DetectorState,
    /// History of detected peak positions
    peak_history: Vec<usize>,
    /// Number of consistent peaks needed to confirm
    confirm_threshold: usize,
    /// Detection power threshold
    power_threshold: f64,
}

impl PreambleDetector {
    /// Create a new preamble detector
    pub fn new(params: LoRaParams) -> Self {
        let chirp_gen = ChirpGenerator::new(params.clone());
        let fft_size = params.samples_per_symbol();
        let fft = FftProcessor::new(fft_size);

        Self {
            params,
            chirp_gen,
            fft,
            state: DetectorState::Searching,
            peak_history: Vec::with_capacity(8),
            confirm_threshold: 3,
            power_threshold: 0.1,
        }
    }

    /// Reset the detector state
    pub fn reset(&mut self) {
        self.state = DetectorState::Searching;
        self.peak_history.clear();
    }

    /// Get current state
    pub fn state(&self) -> DetectorState {
        self.state
    }

    /// Process a window of samples looking for preamble
    ///
    /// Returns Some(DetectionResult) if preamble is found, None otherwise.
    pub fn process(&mut self, samples: &[IQSample]) -> Option<DetectionResult> {
        let n = self.params.samples_per_symbol();

        if samples.len() < n {
            return None;
        }

        // Mix with downchirp and FFT
        let downchirp = self.chirp_gen.base_downchirp();

        let mixed: Vec<Complex> = samples[..n]
            .iter()
            .zip(downchirp.iter())
            .map(|(&s, &d)| s * d.conj())
            .collect();

        let spectrum = self.fft.fft(&mixed);
        let (peak_bin, peak_mag, _) = FftProcessor::find_peak(&spectrum);

        // Calculate average power for SNR estimation
        let avg_power: f64 = spectrum.iter().map(|c| c.norm_sqr()).sum::<f64>() / spectrum.len() as f64;
        let peak_power = peak_mag * peak_mag;

        // Check if peak is strong enough
        if peak_power < self.power_threshold * avg_power {
            self.peak_history.clear();
            self.state = DetectorState::Searching;
            return None;
        }

        // Track peak position
        self.peak_history.push(peak_bin);

        // Keep only recent history
        if self.peak_history.len() > 8 {
            self.peak_history.remove(0);
        }

        // Check for consistent peaks (preamble detection)
        if self.peak_history.len() >= self.confirm_threshold {
            let recent: Vec<usize> = self.peak_history.iter().rev().take(self.confirm_threshold).copied().collect();

            // Check if all recent peaks are at the same position (base upchirp)
            let consistent = recent.iter().all(|&p| (p as i64 - recent[0] as i64).abs() <= 1);

            if consistent {
                self.state = DetectorState::Synchronized;

                let snr_db = if avg_power > 0.0 {
                    10.0 * (peak_power / avg_power).log10()
                } else {
                    0.0
                };

                return Some(DetectionResult {
                    detected: true,
                    timing_offset: 0, // Would need more sophisticated calculation
                    cfo_hz: self.estimate_cfo_from_peak(peak_bin),
                    fractional_timing: 0.0,
                    spreading_factor: Some(self.params.sf.value()),
                    rssi: 10.0 * peak_power.log10(),
                    snr_db,
                });
            }
        }

        self.state = DetectorState::Confirming;
        None
    }

    /// Estimate CFO from FFT peak position
    fn estimate_cfo_from_peak(&self, peak_bin: usize) -> f64 {
        let n = self.params.chips_per_symbol();
        let bw = self.params.bw.hz();

        // Peak bin offset from center indicates frequency offset
        let bin_offset = if peak_bin > n / 2 {
            peak_bin as i64 - n as i64
        } else {
            peak_bin as i64
        };

        bin_offset as f64 * bw / n as f64
    }
}

/// Full synchronizer with CFO correction
#[derive(Debug)]
pub struct Synchronizer {
    /// LoRa parameters
    params: LoRaParams,
    /// Chirp generator
    chirp_gen: ChirpGenerator,
    /// FFT processor
    fft: FftProcessor,
    /// FFT processor for fine sync (oversampled)
    #[allow(dead_code)]
    fft_fine: FftProcessor,
    /// Oversampling factor for fine sync
    #[allow(dead_code)]
    fine_osf: usize,
}

impl Synchronizer {
    /// Create a new synchronizer
    pub fn new(params: LoRaParams) -> Self {
        let chirp_gen = ChirpGenerator::new(params.clone());
        let fft_size = params.samples_per_symbol();
        let fft = FftProcessor::new(fft_size);

        // Fine sync uses 4x oversampling for sub-sample accuracy
        let fine_osf = 4;
        let fft_fine = FftProcessor::new(fft_size * fine_osf);

        Self {
            params,
            chirp_gen,
            fft,
            fft_fine,
            fine_osf,
        }
    }

    /// Perform full synchronization on a signal with detected preamble
    ///
    /// Returns (CFO in Hz, timing offset in samples, sync quality)
    pub fn synchronize(&mut self, samples: &[IQSample]) -> DspResult<SyncResult> {
        let n = self.params.samples_per_symbol();
        let preamble_len = self.params.preamble_length;

        // Need at least preamble + 2.25 downchirps
        let min_samples = n * (preamble_len + 4);
        if samples.len() < min_samples {
            return Err(DspError::BufferTooShort {
                expected: min_samples,
                actual: samples.len(),
            });
        }

        // Analyze upchirp portion (first few symbols)
        let upchirp_samples = &samples[..n * 3];
        let (up_freq, up_mag) = self.analyze_upchirps(upchirp_samples)?;

        // Find downchirp portion (after sync words)
        // Typically at position (preamble_len + 2) symbols
        let down_start = n * (preamble_len + 2);
        if down_start + n * 2 > samples.len() {
            return Err(DspError::SyncError("Not enough samples for downchirp analysis".into()));
        }

        let downchirp_samples = &samples[down_start..down_start + n * 2];
        let (down_freq, down_mag) = self.analyze_downchirps(downchirp_samples)?;

        // Calculate CFO and timing from upchirp/downchirp frequencies
        // CFO = (f_up + f_down) / 2
        // timing = T/B * (f_down - f_up) / 2
        let bw = self.params.bw.hz();
        let k = self.params.chips_per_symbol() as f64;

        let cfo_hz = (up_freq + down_freq) / 2.0;
        let timing_offset_samples = (down_freq - up_freq) / (2.0 * bw) * k;

        let quality = (up_mag + down_mag) / 2.0;

        Ok(SyncResult {
            cfo_hz,
            timing_offset_samples,
            quality,
            preamble_end: down_start + (n * 9 / 4), // After 2.25 downchirps
        })
    }

    /// Analyze upchirp portion to get frequency estimate
    fn analyze_upchirps(&mut self, samples: &[IQSample]) -> DspResult<(f64, f64)> {
        let n = self.params.samples_per_symbol();
        let downchirp = self.chirp_gen.base_downchirp();

        // Mix with downchirp
        let mixed: Vec<Complex> = samples[..n]
            .iter()
            .zip(downchirp.iter())
            .map(|(&s, &d)| s * d.conj())
            .collect();

        let spectrum = self.fft.fft(&mixed);
        let (peak_idx, peak_mag) = FftProcessor::find_peak_interpolated(&spectrum);

        // Convert bin to frequency
        let k = self.params.chips_per_symbol() as f64;
        let bw = self.params.bw.hz();
        let freq = (peak_idx - k / 2.0) * bw / k;

        Ok((freq, peak_mag))
    }

    /// Analyze downchirp portion to get frequency estimate
    fn analyze_downchirps(&mut self, samples: &[IQSample]) -> DspResult<(f64, f64)> {
        let n = self.params.samples_per_symbol();
        let upchirp = self.chirp_gen.base_upchirp();

        // Mix with upchirp (conjugate of downchirp analysis)
        let mixed: Vec<Complex> = samples[..n]
            .iter()
            .zip(upchirp.iter())
            .map(|(&s, &u)| s * u.conj())
            .collect();

        let spectrum = self.fft.fft(&mixed);
        let (peak_idx, peak_mag) = FftProcessor::find_peak_interpolated(&spectrum);

        // Convert bin to frequency (sign flipped for downchirp)
        let k = self.params.chips_per_symbol() as f64;
        let bw = self.params.bw.hz();
        let freq = -(peak_idx - k / 2.0) * bw / k;

        Ok((freq, peak_mag))
    }

    /// Apply CFO correction to samples
    pub fn correct_cfo(&self, samples: &mut [IQSample], cfo_hz: f64) {
        let sample_rate = self.params.sample_rate;

        for (i, sample) in samples.iter_mut().enumerate() {
            let t = i as f64 / sample_rate;
            let correction = Complex::new(
                (2.0 * PI * cfo_hz * t).cos(),
                -(2.0 * PI * cfo_hz * t).sin(),
            );
            *sample *= correction;
        }
    }
}

/// Result of synchronization
#[derive(Debug, Clone)]
pub struct SyncResult {
    /// Carrier frequency offset in Hz
    pub cfo_hz: f64,
    /// Timing offset in samples (fractional)
    pub timing_offset_samples: f64,
    /// Synchronization quality metric
    pub quality: f64,
    /// Sample index where preamble ends (payload starts)
    pub preamble_end: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_preamble_detector_creation() {
        let params = LoRaParams::builder()
            .spreading_factor(7)
            .bandwidth(125_000)
            .build();

        let detector = PreambleDetector::new(params);
        assert_eq!(detector.state(), DetectorState::Searching);
    }

    #[test]
    fn test_synchronizer_creation() {
        let params = LoRaParams::builder()
            .spreading_factor(7)
            .bandwidth(125_000)
            .build();

        let _sync = Synchronizer::new(params);
        // Just test creation doesn't panic
    }

    #[test]
    fn test_cfo_correction() {
        let params = LoRaParams::builder()
            .spreading_factor(7)
            .bandwidth(125_000)
            .build();

        let sync = Synchronizer::new(params.clone());

        // Create a simple test signal
        let mut samples: Vec<IQSample> = (0..128)
            .map(|_i| Complex::new(1.0, 0.0))
            .collect();

        // Apply CFO correction
        sync.correct_cfo(&mut samples, 1000.0);

        // Samples should now have varying phase
        assert!((samples[0].im - 0.0).abs() < 0.01);
        // Later samples should have different phase
        assert!(samples[100].im.abs() > 0.1);
    }
}

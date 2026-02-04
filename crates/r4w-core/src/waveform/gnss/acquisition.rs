//! GNSS Signal Acquisition (PCPS - Parallel Code Phase Search)
//!
//! FFT-based acquisition searches a 2D space of code phase and Doppler
//! frequency to find GNSS signals. For each Doppler hypothesis, the input
//! signal is mixed down, correlated with the local code replica using FFT,
//! and the peak-to-noise ratio is checked against a threshold.
//!
//! ## Algorithm
//!
//! ```text
//! For each Doppler bin fd:
//!   1. Mix input with exp(-j*2π*fd*t) to remove carrier
//!   2. Compute FFT of mixed signal
//!   3. Multiply by conjugate FFT of local code
//!   4. IFFT to get correlation at all code phases
//!   5. Find peak magnitude
//!
//! ┌─────────────────────────────────────────┐
//! │         2D Acquisition Grid              │
//! │ Doppler ▲                                │
//! │ +5kHz   │ · · · · · * · · · · · · ·     │
//! │         │ · · · · · · · · · · · · ·     │
//! │  0 Hz   │ · · · · · · · · · · · · ·     │
//! │         │ · · · · · · · · · · · · ·     │
//! │ -5kHz   │ · · · · · · · · · · · · ·     │
//! │         └───────────────────────────→    │
//! │           0       Code Phase      1022  │
//! └─────────────────────────────────────────┘
//! ```

use crate::fft_utils::FftProcessor;
use crate::types::IQSample;
use rustfft::num_complex::Complex64;
use std::f64::consts::PI;

use super::types::AcquisitionResult;

/// PCPS (Parallel Code Phase Search) acquisition engine
#[derive(Debug)]
pub struct PcpsAcquisition {
    /// FFT size (must be >= code length)
    fft_size: usize,
    /// Code length in chips
    code_length: usize,
    /// Doppler search range in Hz (±doppler_max)
    doppler_max_hz: f64,
    /// Doppler search step in Hz
    doppler_step_hz: f64,
    /// Detection threshold (peak-to-noise ratio)
    threshold: f64,
    /// Sample rate in samples/second
    sample_rate: f64,
    /// Number of coherent integration periods (code periods to average)
    coherent_periods: usize,
}

impl PcpsAcquisition {
    /// Create a new PCPS acquisition engine
    ///
    /// # Arguments
    /// * `code_length` - Number of chips in the spreading code
    /// * `sample_rate` - Sample rate in Hz (should be integer multiple of chipping rate)
    pub fn new(code_length: usize, sample_rate: f64) -> Self {
        let fft_size = code_length.next_power_of_two();
        Self {
            fft_size,
            code_length,
            doppler_max_hz: 5000.0,
            doppler_step_hz: 500.0,
            threshold: 2.5,
            sample_rate,
            coherent_periods: 1,
        }
    }

    /// Set the Doppler search range
    pub fn with_doppler_range(mut self, max_hz: f64, step_hz: f64) -> Self {
        self.doppler_max_hz = max_hz;
        self.doppler_step_hz = step_hz;
        self
    }

    /// Set the detection threshold
    pub fn with_threshold(mut self, threshold: f64) -> Self {
        self.threshold = threshold;
        self
    }

    /// Set number of coherent integration periods
    pub fn with_coherent_periods(mut self, periods: usize) -> Self {
        self.coherent_periods = periods.max(1);
        self
    }

    /// Acquire a signal using PCPS
    ///
    /// # Arguments
    /// * `input` - Input I/Q samples (at least one code period worth)
    /// * `code` - Local code replica (+1/-1 values)
    /// * `prn` - PRN number for the result
    ///
    /// # Returns
    /// Acquisition result with code phase, Doppler, and detection metric
    pub fn acquire(&self, input: &[IQSample], code: &[i8], prn: u8) -> AcquisitionResult {
        let mut fft = FftProcessor::new(self.fft_size);
        let samples_per_code = self.code_length;

        // Pre-compute FFT of local code replica (conjugate)
        let mut code_fft: Vec<Complex64> = code.iter()
            .map(|&c| Complex64::new(c as f64, 0.0))
            .collect();
        code_fft.resize(self.fft_size, Complex64::new(0.0, 0.0));
        fft.fft_inplace(&mut code_fft);
        // Conjugate for correlation
        for c in code_fft.iter_mut() {
            *c = c.conj();
        }

        let mut best_peak = 0.0_f64;
        let mut best_code_phase = 0_usize;
        let mut best_doppler = 0.0_f64;
        let mut noise_floor = 0.0_f64;
        let mut total_bins = 0_usize;

        // Doppler search
        let num_doppler_bins = (2.0 * self.doppler_max_hz / self.doppler_step_hz) as i32 + 1;
        let doppler_start = -self.doppler_max_hz;

        for d in 0..num_doppler_bins {
            let doppler = doppler_start + d as f64 * self.doppler_step_hz;

            // Mix input with carrier to remove Doppler
            let mut mixed: Vec<Complex64> = Vec::with_capacity(self.fft_size);
            for (i, &sample) in input.iter().take(samples_per_code).enumerate() {
                let t = i as f64 / self.sample_rate;
                let phase = -2.0 * PI * doppler * t;
                let carrier = Complex64::new(phase.cos(), phase.sin());
                mixed.push(sample * carrier);
            }
            mixed.resize(self.fft_size, Complex64::new(0.0, 0.0));

            // FFT of mixed signal
            fft.fft_inplace(&mut mixed);

            // Multiply by conjugate code FFT (circular correlation)
            for i in 0..self.fft_size {
                mixed[i] *= code_fft[i];
            }

            // IFFT to get correlation at all code phases
            fft.ifft_inplace(&mut mixed);

            // Find peak in this Doppler bin
            for (phase, corr) in mixed.iter().enumerate().take(self.code_length) {
                let mag = corr.norm_sqr();
                total_bins += 1;
                noise_floor += mag;

                if mag > best_peak {
                    best_peak = mag;
                    best_code_phase = phase;
                    best_doppler = doppler;
                }
            }
        }

        // Calculate noise floor (excluding the peak bin)
        noise_floor = (noise_floor - best_peak) / (total_bins - 1).max(1) as f64;
        let peak_metric = if noise_floor > 0.0 {
            best_peak / noise_floor
        } else {
            best_peak
        };

        let detected = peak_metric > self.threshold;

        // Estimate C/N0 if detected
        let cn0_estimate = if detected {
            // Simplified C/N0 estimation from acquisition metric
            let code_period = self.code_length as f64 / self.sample_rate;
            Some(10.0 * (peak_metric / code_period).log10())
        } else {
            None
        };

        AcquisitionResult {
            prn,
            detected,
            code_phase: best_code_phase as f64,
            doppler_hz: best_doppler,
            peak_metric,
            threshold: self.threshold,
            cn0_estimate,
        }
    }

    /// Perform acquisition across a grid and return the full 2D correlation surface
    /// Useful for visualization of the acquisition search space
    pub fn acquire_grid(&self, input: &[IQSample], code: &[i8]) -> AcquisitionGrid {
        let mut fft = FftProcessor::new(self.fft_size);
        let samples_per_code = self.code_length;

        // Pre-compute FFT of local code replica (conjugate)
        let mut code_fft: Vec<Complex64> = code.iter()
            .map(|&c| Complex64::new(c as f64, 0.0))
            .collect();
        code_fft.resize(self.fft_size, Complex64::new(0.0, 0.0));
        fft.fft_inplace(&mut code_fft);
        for c in code_fft.iter_mut() {
            *c = c.conj();
        }

        let num_doppler_bins = (2.0 * self.doppler_max_hz / self.doppler_step_hz) as usize + 1;
        let doppler_start = -self.doppler_max_hz;

        let mut doppler_bins = Vec::with_capacity(num_doppler_bins);
        let mut grid = Vec::with_capacity(num_doppler_bins);

        for d in 0..num_doppler_bins {
            let doppler = doppler_start + d as f64 * self.doppler_step_hz;
            doppler_bins.push(doppler);

            let mut mixed: Vec<Complex64> = Vec::with_capacity(self.fft_size);
            for (i, &sample) in input.iter().take(samples_per_code).enumerate() {
                let t = i as f64 / self.sample_rate;
                let phase = -2.0 * PI * doppler * t;
                let carrier = Complex64::new(phase.cos(), phase.sin());
                mixed.push(sample * carrier);
            }
            mixed.resize(self.fft_size, Complex64::new(0.0, 0.0));

            fft.fft_inplace(&mut mixed);
            for i in 0..self.fft_size {
                mixed[i] *= code_fft[i];
            }
            fft.ifft_inplace(&mut mixed);

            let row: Vec<f64> = mixed.iter().take(self.code_length)
                .map(|c| c.norm_sqr())
                .collect();
            grid.push(row);
        }

        AcquisitionGrid {
            doppler_bins,
            code_phases: (0..self.code_length).map(|i| i as f64).collect(),
            correlation_power: grid,
        }
    }

    /// Get the FFT size being used
    pub fn fft_size(&self) -> usize {
        self.fft_size
    }
}

/// 2D acquisition grid result for visualization
#[derive(Debug, Clone)]
pub struct AcquisitionGrid {
    /// Doppler frequency bins (Hz)
    pub doppler_bins: Vec<f64>,
    /// Code phase bins (chips)
    pub code_phases: Vec<f64>,
    /// Correlation power [doppler][code_phase]
    pub correlation_power: Vec<Vec<f64>>,
}

impl AcquisitionGrid {
    /// Find the peak in the grid
    pub fn find_peak(&self) -> (f64, f64, f64) {
        let mut best_power = 0.0_f64;
        let mut best_doppler = 0.0;
        let mut best_phase = 0.0;

        for (d, row) in self.correlation_power.iter().enumerate() {
            for (p, &power) in row.iter().enumerate() {
                if power > best_power {
                    best_power = power;
                    best_doppler = self.doppler_bins[d];
                    best_phase = self.code_phases[p];
                }
            }
        }
        (best_doppler, best_phase, best_power)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::waveform::gnss::prn::GpsCaCodeGenerator;
    use crate::spreading::PnSequence;

    #[test]
    fn test_acquisition_no_noise() {
        let prn = 1;
        let code_length = 1023;
        let sample_rate = 1023.0; // 1 sample per chip

        // Generate code
        let mut gen = GpsCaCodeGenerator::new(prn);
        let code = gen.generate_code();

        // Generate a clean signal with known code phase and Doppler
        let code_phase_true = 100;
        let doppler_true = 1000.0; // Hz

        let mut signal: Vec<IQSample> = Vec::with_capacity(code_length);
        for i in 0..code_length {
            let chip_idx = (i + code_length - code_phase_true) % code_length;
            let chip_val = code[chip_idx] as f64;
            let t = i as f64 / sample_rate;
            let phase = 2.0 * PI * doppler_true * t;
            signal.push(Complex64::new(
                chip_val * phase.cos(),
                chip_val * phase.sin(),
            ));
        }

        let acq = PcpsAcquisition::new(code_length, sample_rate)
            .with_doppler_range(5000.0, 500.0)
            .with_threshold(2.0);

        let result = acq.acquire(&signal, &code, prn);

        assert!(result.detected, "Signal should be detected");
        assert_eq!(result.code_phase as usize, code_phase_true,
            "Code phase {} should match true phase {}", result.code_phase, code_phase_true);
        assert!((result.doppler_hz - doppler_true).abs() <= 500.0,
            "Doppler {} should be within 500 Hz of {}", result.doppler_hz, doppler_true);
    }

    #[test]
    fn test_acquisition_wrong_prn() {
        let code_length = 1023;
        let sample_rate = 1023.0;

        // Generate signal with PRN 1
        let mut gen1 = GpsCaCodeGenerator::new(1);
        let code1 = gen1.generate_code();
        let signal: Vec<IQSample> = code1.iter()
            .map(|&c| Complex64::new(c as f64, 0.0))
            .collect();

        // Try to acquire with PRN 7
        let mut gen7 = GpsCaCodeGenerator::new(7);
        let code7 = gen7.generate_code();

        let acq = PcpsAcquisition::new(code_length, sample_rate)
            .with_threshold(2.5);
        let result = acq.acquire(&signal, &code7, 7);

        // Should not detect (wrong PRN)
        assert!(!result.detected || result.peak_metric < 10.0,
            "Wrong PRN should have low peak metric, got {}", result.peak_metric);
    }

    #[test]
    fn test_acquisition_grid() {
        let code_length = 1023;
        let sample_rate = 1023.0;

        let mut gen = GpsCaCodeGenerator::new(1);
        let code = gen.generate_code();
        let signal: Vec<IQSample> = code.iter()
            .map(|&c| Complex64::new(c as f64, 0.0))
            .collect();

        let acq = PcpsAcquisition::new(code_length, sample_rate)
            .with_doppler_range(2000.0, 500.0);
        let grid = acq.acquire_grid(&signal, &code);

        let (doppler, phase, _power) = grid.find_peak();
        assert!((doppler).abs() <= 500.0, "Zero Doppler signal should peak near 0 Hz");
        assert!((phase).abs() <= 1.0 || (phase - 1023.0).abs() <= 1.0,
            "Zero phase signal should peak near 0 chips");
    }
}

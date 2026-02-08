//! Core Filter Traits
//!
//! Defines the fundamental traits for digital filters in R4W.
//!
//! ## Architecture
//!
//! The filter system is built around a hierarchy of traits:
//!
//! ```text
//! Filter (core trait)
//!    ├── FirFilterOps (FIR-specific operations)
//!    ├── IirFilterOps (IIR-specific operations)  [future]
//!    └── FrequencyResponse (frequency domain analysis)
//!
//! StatefulFilter (filters with internal state)
//!    └── Extends Filter with reset capability
//! ```
//!
//! ## Usage
//!
//! All filters implement the core `Filter` trait, enabling polymorphic usage:
//!
//! ```rust,ignore
//! use r4w_core::filters::{Filter, FirFilter};
//!
//! fn apply_filter(filter: &mut dyn Filter, samples: &mut [Complex64]) {
//!     filter.process_inplace(samples);
//! }
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Core trait for all digital filters.
///
/// This is the fundamental trait that all filters must implement.
/// It provides complex sample processing which is the most general case.
pub trait Filter: Send + Sync {
    /// Process a single complex sample through the filter.
    ///
    /// This is the core filtering operation. Filters maintain internal
    /// state (delay lines, accumulators) that persist between calls.
    fn process(&mut self, input: Complex64) -> Complex64;

    /// Process a block of samples, returning filtered output.
    ///
    /// Default implementation calls `process()` for each sample.
    /// Implementations may override for better performance.
    fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Process samples in place.
    ///
    /// Default implementation calls `process()` for each sample.
    /// Implementations may override for better performance.
    fn process_inplace(&mut self, samples: &mut [Complex64]) {
        for s in samples.iter_mut() {
            *s = self.process(*s);
        }
    }

    /// Reset filter state (clear delay lines, accumulators).
    ///
    /// Call this when starting to process a new signal stream
    /// to avoid artifacts from previous data.
    fn reset(&mut self);

    /// Get the group delay in samples.
    ///
    /// For linear-phase FIR filters, this is (N-1)/2 where N is the
    /// number of taps. For IIR filters, this varies with frequency.
    fn group_delay(&self) -> f64;

    /// Get the filter order.
    ///
    /// For FIR filters: number of taps - 1.
    /// For IIR filters: max(len(b), len(a)) - 1.
    fn order(&self) -> usize;
}

/// Trait for filters that can process real-valued samples efficiently.
///
/// Some filters (especially those with real coefficients) can process
/// real samples more efficiently than converting to/from complex.
pub trait RealFilter: Filter {
    /// Process a single real-valued sample.
    fn process_real(&mut self, input: f64) -> f64;

    /// Process a block of real samples.
    fn process_real_block(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&s| self.process_real(s)).collect()
    }

    /// Process real samples in place.
    fn process_real_inplace(&mut self, samples: &mut [f64]) {
        for s in samples.iter_mut() {
            *s = self.process_real(*s);
        }
    }
}

/// Trait for FIR filter-specific operations.
///
/// FIR filters have finite impulse response and are characterized
/// by their tap coefficients (impulse response).
pub trait FirFilterOps: Filter {
    /// Get the filter coefficients (impulse response).
    fn coefficients(&self) -> &[f64];

    /// Get the number of taps.
    fn num_taps(&self) -> usize {
        self.coefficients().len()
    }

    /// Check if the filter has linear phase (symmetric coefficients).
    fn is_linear_phase(&self) -> bool {
        let coeffs = self.coefficients();
        let n = coeffs.len();
        for i in 0..n / 2 {
            if (coeffs[i] - coeffs[n - 1 - i]).abs() > 1e-10 {
                return false;
            }
        }
        true
    }
}

/// Trait for frequency response analysis.
///
/// Provides methods to compute the magnitude and phase response
/// of a filter at specific frequencies.
pub trait FrequencyResponse {
    /// Compute the magnitude response at a given frequency.
    ///
    /// # Arguments
    /// * `freq_hz` - Frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    ///
    /// # Returns
    /// Magnitude response (linear scale, not dB)
    fn magnitude_response(&self, freq_hz: f64, sample_rate: f64) -> f64;

    /// Compute the magnitude response in decibels.
    fn magnitude_response_db(&self, freq_hz: f64, sample_rate: f64) -> f64 {
        20.0 * self.magnitude_response(freq_hz, sample_rate).log10()
    }

    /// Compute the phase response at a given frequency.
    ///
    /// # Arguments
    /// * `freq_hz` - Frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    ///
    /// # Returns
    /// Phase response in radians
    fn phase_response(&self, freq_hz: f64, sample_rate: f64) -> f64;

    /// Compute magnitude response at multiple frequencies.
    fn frequency_response(&self, freqs_hz: &[f64], sample_rate: f64) -> Vec<(f64, f64)> {
        freqs_hz
            .iter()
            .map(|&f| (self.magnitude_response(f, sample_rate), self.phase_response(f, sample_rate)))
            .collect()
    }
}

/// Implement FrequencyResponse for any type that implements FirFilterOps.
///
/// Uses the DFT of the filter coefficients to compute the response.
impl<T: FirFilterOps + ?Sized> FrequencyResponse for T {
    fn magnitude_response(&self, freq_hz: f64, sample_rate: f64) -> f64 {
        let coeffs = self.coefficients();
        let omega = 2.0 * PI * freq_hz / sample_rate;

        // H(e^jω) = Σ h[n] * e^(-jωn)
        let mut real = 0.0;
        let mut imag = 0.0;
        for (n, &h) in coeffs.iter().enumerate() {
            let phase = omega * n as f64;
            real += h * phase.cos();
            imag -= h * phase.sin();
        }

        (real * real + imag * imag).sqrt()
    }

    fn phase_response(&self, freq_hz: f64, sample_rate: f64) -> f64 {
        let coeffs = self.coefficients();
        let omega = 2.0 * PI * freq_hz / sample_rate;

        let mut real = 0.0;
        let mut imag = 0.0;
        for (n, &h) in coeffs.iter().enumerate() {
            let phase = omega * n as f64;
            real += h * phase.cos();
            imag -= h * phase.sin();
        }

        imag.atan2(real)
    }
}

/// Filter type classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterType {
    /// Lowpass filter - passes low frequencies, attenuates high
    Lowpass,
    /// Highpass filter - passes high frequencies, attenuates low
    Highpass,
    /// Bandpass filter - passes a band of frequencies
    Bandpass,
    /// Bandstop (notch) filter - attenuates a band of frequencies
    Bandstop,
    /// Allpass filter - passes all frequencies, only affects phase
    Allpass,
}

/// Filter response type (for IIR design).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterResponse {
    /// Butterworth - maximally flat passband
    Butterworth,
    /// Chebyshev Type I - equiripple passband
    ChebyshevI,
    /// Chebyshev Type II - equiripple stopband
    ChebyshevII,
    /// Elliptic (Cauer) - equiripple both bands
    Elliptic,
    /// Bessel - maximally flat group delay
    Bessel,
}

#[cfg(test)]
mod tests {
    use super::*;

    // Mock filter for testing traits
    struct MockFirFilter {
        coeffs: Vec<f64>,
        delay_line: Vec<Complex64>,
        delay_idx: usize,
    }

    impl MockFirFilter {
        fn new(coeffs: Vec<f64>) -> Self {
            let len = coeffs.len();
            Self {
                coeffs,
                delay_line: vec![Complex64::new(0.0, 0.0); len],
                delay_idx: 0,
            }
        }
    }

    impl Filter for MockFirFilter {
        fn process(&mut self, input: Complex64) -> Complex64 {
            self.delay_line[self.delay_idx] = input;
            let mut output = Complex64::new(0.0, 0.0);
            let len = self.coeffs.len();
            for i in 0..len {
                let delay_pos = (self.delay_idx + len - i) % len;
                output += self.delay_line[delay_pos] * self.coeffs[i];
            }
            self.delay_idx = (self.delay_idx + 1) % len;
            output
        }

        fn reset(&mut self) {
            for s in self.delay_line.iter_mut() {
                *s = Complex64::new(0.0, 0.0);
            }
            self.delay_idx = 0;
        }

        fn group_delay(&self) -> f64 {
            (self.coeffs.len() - 1) as f64 / 2.0
        }

        fn order(&self) -> usize {
            self.coeffs.len() - 1
        }
    }

    impl FirFilterOps for MockFirFilter {
        fn coefficients(&self) -> &[f64] {
            &self.coeffs
        }
    }

    #[test]
    fn test_filter_trait() {
        let mut filter = MockFirFilter::new(vec![0.25, 0.5, 0.25]);

        let input = Complex64::new(1.0, 0.0);
        let _ = filter.process(input);
        let _ = filter.process(input);
        let output = filter.process(input);

        // After 3 samples, should be fully settled
        assert!((output.re - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_fir_filter_ops() {
        let filter = MockFirFilter::new(vec![0.25, 0.5, 0.25]);

        assert_eq!(filter.num_taps(), 3);
        assert!(filter.is_linear_phase());
        assert_eq!(filter.order(), 2);
    }

    #[test]
    fn test_frequency_response() {
        let filter = MockFirFilter::new(vec![0.25, 0.5, 0.25]);

        // DC response should be unity (sum of coeffs)
        let dc_mag = filter.magnitude_response(0.0, 1000.0);
        assert!((dc_mag - 1.0).abs() < 0.01);

        // High frequency should be attenuated
        let hf_mag = filter.magnitude_response(400.0, 1000.0);
        assert!(hf_mag < dc_mag);
    }

    #[test]
    fn test_linear_phase_detection() {
        // Symmetric coefficients = linear phase
        let symmetric = MockFirFilter::new(vec![0.1, 0.2, 0.4, 0.2, 0.1]);
        assert!(symmetric.is_linear_phase());

        // Asymmetric coefficients = not linear phase
        let asymmetric = MockFirFilter::new(vec![0.1, 0.2, 0.4, 0.3]);
        assert!(!asymmetric.is_linear_phase());
    }
}

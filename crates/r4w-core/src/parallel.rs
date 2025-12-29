//! Parallel Processing Module
//!
//! This module provides parallel implementations of DSP operations using Rayon.
//! Enable with the `parallel` feature flag.
//!
//! ## Usage
//!
//! ```toml
//! [dependencies]
//! r4w-core = { version = "0.1", features = ["parallel"] }
//! ```
//!
//! ## Performance Considerations
//!
//! Parallelization adds overhead, so it's most beneficial for:
//! - Batch processing of multiple packets/symbols
//! - Large FFT computations (spectrogram, waterfall)
//! - Channel simulation with many samples
//!
//! For single symbol operations, sequential code is often faster.

use rayon::prelude::*;
use rustfft::num_complex::Complex64;

use crate::chirp::ChirpGenerator;
use crate::demodulation::SymbolResult;
use crate::fft_utils::{FftProcessor, Spectrogram};
use crate::params::LoRaParams;
use crate::types::IQSample;
use crate::waveform::{CommonParams, DemodResult, WaveformFactory};

/// Parallel batch modulator for processing multiple messages
pub struct ParallelModulator {
    params: LoRaParams,
}

impl ParallelModulator {
    /// Create a new parallel modulator
    pub fn new(params: LoRaParams) -> Self {
        Self { params }
    }

    /// Modulate multiple payloads in parallel
    ///
    /// Each payload is processed by a separate thread.
    /// Returns a vector of I/Q sample vectors, one per payload.
    pub fn modulate_batch(&self, payloads: &[&[u8]]) -> Vec<Vec<IQSample>> {
        payloads
            .par_iter()
            .map(|payload| {
                let mut modulator = crate::modulation::Modulator::new(self.params.clone());
                modulator.modulate(payload)
            })
            .collect()
    }

    /// Modulate multiple symbols in parallel
    ///
    /// Useful for generating chirps for many symbols at once.
    pub fn modulate_symbols_parallel(&self, symbols: &[u16]) -> Vec<IQSample> {
        let chirp_gen = ChirpGenerator::new(self.params.clone());
        let samples_per_symbol = self.params.samples_per_symbol();

        // Pre-allocate output
        let total_samples = symbols.len() * samples_per_symbol;
        let mut result = vec![Complex64::new(0.0, 0.0); total_samples];

        // Process symbols in parallel chunks
        result
            .par_chunks_mut(samples_per_symbol)
            .zip(symbols.par_iter())
            .for_each(|(chunk, &symbol)| {
                let chirp = chirp_gen.generate_symbol_chirp_fast(symbol);
                chunk.copy_from_slice(&chirp);
            });

        result
    }
}

/// Parallel batch demodulator for processing multiple signals
pub struct ParallelDemodulator {
    params: LoRaParams,
}

impl ParallelDemodulator {
    /// Create a new parallel demodulator
    pub fn new(params: LoRaParams) -> Self {
        Self { params }
    }

    /// Demodulate multiple I/Q sample buffers in parallel
    pub fn demodulate_batch(&self, signals: &[&[IQSample]]) -> Vec<Vec<SymbolResult>> {
        let samples_per_symbol = self.params.samples_per_symbol();
        signals
            .par_iter()
            .map(|samples| {
                let num_symbols = samples.len() / samples_per_symbol;
                let mut demodulator = crate::demodulation::Demodulator::new(self.params.clone());
                demodulator.demodulate_symbols(samples, num_symbols)
            })
            .collect()
    }

    /// Demodulate symbols from a single buffer in parallel
    ///
    /// Splits the buffer into symbol-sized chunks and processes them concurrently.
    pub fn demodulate_symbols_parallel(&self, samples: &[IQSample]) -> Vec<u16> {
        let samples_per_symbol = self.params.samples_per_symbol();
        let chirp_gen = ChirpGenerator::new(self.params.clone());
        let downchirp = chirp_gen.base_downchirp();

        samples
            .par_chunks_exact(samples_per_symbol)
            .map(|symbol_samples| {
                // Dechirp: multiply by downchirp
                let mut dechirped: Vec<Complex64> = symbol_samples
                    .iter()
                    .zip(downchirp.iter())
                    .map(|(&s, &d)| s * d)
                    .collect();

                // FFT
                let mut fft = FftProcessor::new(samples_per_symbol);
                fft.fft_inplace(&mut dechirped);

                // Find peak
                let (peak_bin, _, _) = FftProcessor::find_peak(&dechirped);
                peak_bin as u16
            })
            .collect()
    }
}

/// Compute spectrogram in parallel
///
/// Each FFT frame is computed by a separate thread.
pub fn parallel_spectrogram(
    signal: &[IQSample],
    fft_size: usize,
    hop_size: usize,
    sample_rate: f64,
) -> Spectrogram {
    // Generate Hann window
    let window: Vec<f64> = (0..fft_size)
        .map(|i| 0.5 * (1.0 - (2.0 * std::f64::consts::PI * i as f64 / fft_size as f64).cos()))
        .collect();

    // Calculate frame positions
    let num_frames = (signal.len().saturating_sub(fft_size)) / hop_size + 1;
    let frame_positions: Vec<usize> = (0..num_frames).map(|i| i * hop_size).collect();

    // Process frames in parallel
    let power_db: Vec<Vec<f64>> = frame_positions
        .par_iter()
        .filter_map(|&pos| {
            if pos + fft_size <= signal.len() {
                let mut fft = FftProcessor::new(fft_size);

                // Extract and window the frame
                let mut frame: Vec<Complex64> = signal[pos..pos + fft_size]
                    .iter()
                    .enumerate()
                    .map(|(i, &s)| s * window[i])
                    .collect();

                // Compute FFT
                fft.fft_inplace(&mut frame);

                // Compute power spectrum in dB and FFT shift
                let power = FftProcessor::power_spectrum_db(&frame);
                Some(FftProcessor::fft_shift(&power))
            } else {
                None
            }
        })
        .collect();

    // Compute times
    let times: Vec<f64> = frame_positions
        .iter()
        .take(power_db.len())
        .map(|&pos| (pos + fft_size / 2) as f64 / sample_rate)
        .collect();

    // Compute frequency axis
    let freq_resolution = sample_rate / fft_size as f64;
    let frequencies: Vec<f64> = (0..fft_size)
        .map(|i| {
            let idx = if i < fft_size / 2 {
                i as f64
            } else {
                (i as i64 - fft_size as i64) as f64
            };
            idx * freq_resolution
        })
        .collect();
    let frequencies = FftProcessor::fft_shift(&frequencies);

    Spectrogram {
        times,
        frequencies,
        power_db,
        fft_size,
        hop_size,
        sample_rate,
    }
}

/// Parallel waveform batch processor
pub struct ParallelWaveformProcessor {
    sample_rate: f64,
}

impl ParallelWaveformProcessor {
    /// Create a new parallel waveform processor
    pub fn new(common_params: CommonParams) -> Self {
        Self {
            sample_rate: common_params.sample_rate,
        }
    }

    /// Create from sample rate directly
    pub fn with_sample_rate(sample_rate: f64) -> Self {
        Self { sample_rate }
    }

    /// Modulate multiple bit sequences with the same waveform type in parallel
    pub fn modulate_batch(&self, waveform_name: &str, bit_sequences: &[Vec<u8>]) -> Vec<Vec<IQSample>> {
        let sample_rate = self.sample_rate;
        bit_sequences
            .par_iter()
            .filter_map(|bits| {
                WaveformFactory::create(waveform_name, sample_rate)
                    .map(|waveform| waveform.modulate(bits))
            })
            .collect()
    }

    /// Demodulate multiple sample buffers with the same waveform type in parallel
    pub fn demodulate_batch(&self, waveform_name: &str, sample_buffers: &[Vec<IQSample>]) -> Vec<DemodResult> {
        let sample_rate = self.sample_rate;
        sample_buffers
            .par_iter()
            .filter_map(|samples| {
                WaveformFactory::create(waveform_name, sample_rate)
                    .map(|waveform| waveform.demodulate(samples))
            })
            .collect()
    }

    /// Test multiple waveforms on the same data in parallel
    pub fn compare_waveforms(&self, waveform_names: &[&str], bits: &[u8]) -> Vec<(String, Vec<IQSample>, DemodResult)> {
        let sample_rate = self.sample_rate;
        waveform_names
            .par_iter()
            .filter_map(|&name| {
                WaveformFactory::create(name, sample_rate).map(|waveform| {
                    let samples = waveform.modulate(bits);
                    let result = waveform.demodulate(&samples);
                    (name.to_string(), samples, result)
                })
            })
            .collect()
    }
}

/// Parallel channel simulation
pub mod channel {
    use super::*;
    use rand::SeedableRng;
    use rand_distr::{Distribution, Normal};

    /// Add AWGN noise to multiple signals in parallel
    pub fn add_awgn_batch(
        signals: &[Vec<IQSample>],
        snr_db: f64,
    ) -> Vec<Vec<IQSample>> {
        signals
            .par_iter()
            .enumerate()
            .map(|(idx, signal)| {
                add_awgn_single(signal, snr_db, idx as u64)
            })
            .collect()
    }

    fn add_awgn_single(signal: &[IQSample], snr_db: f64, seed: u64) -> Vec<IQSample> {
        // Calculate signal power
        let signal_power: f64 = signal.iter().map(|s| s.norm_sqr()).sum::<f64>() / signal.len() as f64;

        // Calculate noise power from SNR
        let snr_linear = 10.0_f64.powf(snr_db / 10.0);
        let noise_power = signal_power / snr_linear;
        let noise_std = (noise_power / 2.0).sqrt();

        // Generate noise
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        let normal = Normal::new(0.0, noise_std).unwrap();

        signal
            .iter()
            .map(|&s| {
                let noise_re = normal.sample(&mut rng);
                let noise_im = normal.sample(&mut rng);
                s + Complex64::new(noise_re, noise_im)
            })
            .collect()
    }

    /// Simulate multipath channel on multiple signals in parallel
    pub fn apply_multipath_batch(
        signals: &[Vec<IQSample>],
        delays_samples: &[usize],
        amplitudes: &[f64],
    ) -> Vec<Vec<IQSample>> {
        signals
            .par_iter()
            .map(|signal| apply_multipath_single(signal, delays_samples, amplitudes))
            .collect()
    }

    fn apply_multipath_single(
        signal: &[IQSample],
        delays_samples: &[usize],
        amplitudes: &[f64],
    ) -> Vec<IQSample> {
        let max_delay = delays_samples.iter().max().copied().unwrap_or(0);
        let mut output = vec![Complex64::new(0.0, 0.0); signal.len() + max_delay];

        for (&delay, &amp) in delays_samples.iter().zip(amplitudes.iter()) {
            for (i, &s) in signal.iter().enumerate() {
                output[i + delay] += s * amp;
            }
        }

        output
    }
}

/// Utility functions for parallel operations
pub mod utils {
    use super::*;

    /// Compute magnitudes of complex samples in parallel
    pub fn parallel_magnitude(samples: &[IQSample]) -> Vec<f64> {
        samples.par_iter().map(|s| s.norm()).collect()
    }

    /// Compute power (magnitude squared) in parallel
    pub fn parallel_power(samples: &[IQSample]) -> Vec<f64> {
        samples.par_iter().map(|s| s.norm_sqr()).collect()
    }

    /// Compute phase angles in parallel
    pub fn parallel_phase(samples: &[IQSample]) -> Vec<f64> {
        samples.par_iter().map(|s| s.arg()).collect()
    }

    /// Element-wise complex multiply in parallel
    pub fn parallel_complex_multiply(a: &[IQSample], b: &[IQSample]) -> Vec<IQSample> {
        a.par_iter()
            .zip(b.par_iter())
            .map(|(&x, &y)| x * y)
            .collect()
    }

    /// Find peak in parallel (for very large spectra)
    pub fn parallel_find_peak(spectrum: &[IQSample]) -> (usize, f64) {
        spectrum
            .par_iter()
            .enumerate()
            .map(|(i, s)| (i, s.norm()))
            .reduce(
                || (0, 0.0),
                |a, b| if b.1 > a.1 { b } else { a },
            )
    }

    /// Parallel correlation computation
    pub fn parallel_correlate(signal: &[IQSample], reference: &[IQSample]) -> Vec<f64> {
        let ref_len = reference.len();
        let output_len = signal.len().saturating_sub(ref_len) + 1;

        (0..output_len)
            .into_par_iter()
            .map(|offset| {
                signal[offset..offset + ref_len]
                    .iter()
                    .zip(reference.iter())
                    .map(|(&s, &r)| (s * r.conj()).re)
                    .sum()
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parallel_modulator() {
        let params = LoRaParams::builder()
            .spreading_factor(7)
            .bandwidth(125_000)
            .build();

        let modulator = ParallelModulator::new(params);
        let payloads: Vec<&[u8]> = vec![b"Hello", b"World", b"Test!"];

        let results = modulator.modulate_batch(&payloads);
        assert_eq!(results.len(), 3);
        assert!(!results[0].is_empty());
    }

    #[test]
    fn test_parallel_demodulator() {
        let params = LoRaParams::builder()
            .spreading_factor(7)
            .bandwidth(125_000)
            .build();

        // Generate test samples
        let mut modulator = crate::modulation::Modulator::new(params.clone());
        let samples = modulator.symbols_only(b"Hi");

        let demodulator = ParallelDemodulator::new(params);
        let symbols = demodulator.demodulate_symbols_parallel(&samples);
        assert!(!symbols.is_empty());
    }

    #[test]
    fn test_parallel_spectrogram() {
        let sample_rate = 1000.0;
        let signal: Vec<IQSample> = (0..2048)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * std::f64::consts::PI * 100.0 * t;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let spec = parallel_spectrogram(&signal, 256, 128, sample_rate);
        assert!(!spec.times.is_empty());
        assert!(!spec.frequencies.is_empty());
        assert!(!spec.power_db.is_empty());
    }

    #[test]
    fn test_parallel_awgn() {
        let signal: Vec<IQSample> = (0..1000)
            .map(|_| Complex64::new(1.0, 0.0))
            .collect();

        let signals = vec![signal.clone(), signal.clone(), signal];
        let noisy = channel::add_awgn_batch(&signals, 20.0);

        assert_eq!(noisy.len(), 3);
        // Each output should be different due to different seeds
        assert_ne!(noisy[0][0], noisy[1][0]);
    }

    #[test]
    fn test_parallel_magnitude() {
        let samples: Vec<IQSample> = vec![
            Complex64::new(3.0, 4.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 1.0),
        ];

        let mags = utils::parallel_magnitude(&samples);
        assert!((mags[0] - 5.0).abs() < 1e-10);
        assert!((mags[1] - 1.0).abs() < 1e-10);
        assert!((mags[2] - 1.0).abs() < 1e-10);
    }
}

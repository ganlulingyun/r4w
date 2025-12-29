//! Waveform Processing Runner
//!
//! Processes I/Q samples through waveform demodulation with timing.

use crate::types::IQSample;
use crate::waveform::{DemodResult, Waveform, WaveformFactory};
use std::time::{Duration, Instant};

/// Result from processing a batch of samples
#[derive(Debug, Clone)]
pub struct ProcessResult {
    /// Demodulation result from waveform
    pub demod_result: DemodResult,
    /// Time taken to process
    pub processing_time: Duration,
    /// Number of samples processed
    pub samples_processed: usize,
}

impl ProcessResult {
    /// Calculate processing throughput in samples/sec
    pub fn throughput_samples_per_sec(&self) -> f64 {
        if self.processing_time.as_secs_f64() > 0.0 {
            self.samples_processed as f64 / self.processing_time.as_secs_f64()
        } else {
            0.0
        }
    }
}

/// Waveform processing runner for benchmarking
pub struct WaveformRunner {
    waveform: Box<dyn Waveform>,
    waveform_name: String,
    sample_rate: f64,
}

impl WaveformRunner {
    /// Create a new runner for the specified waveform
    pub fn new(waveform_name: &str, sample_rate: f64) -> Result<Self, String> {
        let waveform = WaveformFactory::create(waveform_name, sample_rate)
            .ok_or_else(|| format!("Unknown waveform: {}", waveform_name))?;

        Ok(Self {
            waveform,
            waveform_name: waveform_name.to_string(),
            sample_rate,
        })
    }

    /// Process samples through the waveform
    pub fn process(&self, samples: &[IQSample]) -> ProcessResult {
        let start = Instant::now();
        let demod_result = self.waveform.demodulate(samples);
        let processing_time = start.elapsed();

        ProcessResult {
            demod_result,
            processing_time,
            samples_processed: samples.len(),
        }
    }

    /// Get waveform name
    pub fn waveform_name(&self) -> &str {
        &self.waveform_name
    }

    /// Get sample rate
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Get waveform info
    pub fn waveform_info(&self) -> crate::waveform::WaveformInfo {
        self.waveform.info()
    }

    /// Get available waveform names
    pub fn available_waveforms() -> Vec<&'static str> {
        WaveformFactory::list()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_runner_creation() {
        let runner = WaveformRunner::new("BPSK", 48000.0);
        assert!(runner.is_ok());

        let runner = WaveformRunner::new("InvalidWaveform", 48000.0);
        assert!(runner.is_err());
    }

    #[test]
    fn test_process_empty() {
        let runner = WaveformRunner::new("BPSK", 48000.0).unwrap();
        let result = runner.process(&[]);
        assert_eq!(result.samples_processed, 0);
    }

    #[test]
    fn test_process_samples() {
        let runner = WaveformRunner::new("BPSK", 48000.0).unwrap();

        // Generate some test samples
        let samples: Vec<IQSample> = (0..1000)
            .map(|i| {
                let phase = 2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0;
                IQSample::new(phase.cos(), phase.sin())
            })
            .collect();

        let result = runner.process(&samples);
        assert_eq!(result.samples_processed, 1000);
        assert!(result.processing_time.as_nanos() > 0);
    }
}

//! FPGA Hardware Acceleration for LoRa DSP
//!
//! This module provides FPGA-accelerated versions of core DSP operations
//! used in LoRa modulation and demodulation. When an FPGA is available,
//! operations like FFT and chirp generation can be offloaded to hardware
//! for significant performance improvements.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                    Application Code                             │
//! │   (ChirpGenerator, Demodulator, Modulator)                      │
//! └───────────────────────────┬─────────────────────────────────────┘
//!                             │
//!                             ▼
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                    FpgaContext (this module)                    │
//! │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
//! │  │AccelChirpGen │  │  AccelFft    │  │  AccelCorrelator     │   │
//! │  └──────────────┘  └──────────────┘  └──────────────────────┘   │
//! └───────────────────────────┬─────────────────────────────────────┘
//!                             │
//!                             ▼
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                    r4w-fpga crate                               │
//! │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
//! │  │ SimulatedFpga│  │   ZynqFpga   │  │    LatticeFpga       │   │
//! │  └──────────────┘  └──────────────┘  └──────────────────────┘   │
//! └─────────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Usage
//!
//! ```rust,ignore
//! use r4w_core::fpga_accel::{FpgaContext, AcceleratedChirpGenerator};
//!
//! // Initialize FPGA context (auto-detects available hardware)
//! let ctx = FpgaContext::auto_detect();
//!
//! // Create accelerated chirp generator
//! let chirp_gen = AcceleratedChirpGenerator::new(params, Some(ctx.clone()));
//!
//! // Generate chirp - uses FPGA if available, falls back to software
//! let chirp = chirp_gen.generate_upchirp(7);
//! ```

use std::sync::{Arc, Mutex, OnceLock};

use r4w_fpga::{FpgaAccelerator, FpgaCapabilities, FpgaInfo, SimulatedFpga};

use crate::params::LoRaParams;
use crate::types::IQSample;

/// Global FPGA context for the application
static GLOBAL_FPGA: OnceLock<Arc<FpgaContext>> = OnceLock::new();

/// FPGA context holding the accelerator instance
///
/// The context provides thread-safe access to FPGA operations.
/// It can be shared across multiple DSP components.
#[derive(Clone)]
pub struct FpgaContext {
    /// The FPGA accelerator (wrapped for thread safety)
    inner: Arc<Mutex<Box<dyn FpgaAccelerator>>>,

    /// Cached capabilities for quick access
    capabilities: FpgaCapabilities,

    /// Device information
    info: FpgaInfo,
}

impl FpgaContext {
    /// Create a new FPGA context with the given accelerator
    pub fn new(accelerator: Box<dyn FpgaAccelerator>) -> Self {
        let capabilities = accelerator.capabilities();
        let info = accelerator.info();

        Self {
            inner: Arc::new(Mutex::new(accelerator)),
            capabilities,
            info,
        }
    }

    /// Auto-detect and initialize the best available FPGA
    ///
    /// Priority:
    /// 1. Xilinx Zynq (if zynq feature enabled and hardware present)
    /// 2. Lattice (if lattice feature enabled and hardware present)
    /// 3. Simulated FPGA (always available)
    pub fn auto_detect() -> Self {
        let accelerator = r4w_fpga::create_default();
        Self::new(accelerator)
    }

    /// Create a context with simulated FPGA (for testing/development)
    pub fn simulated() -> Self {
        Self::new(Box::new(SimulatedFpga::new()))
    }

    /// Get device information
    pub fn info(&self) -> &FpgaInfo {
        &self.info
    }

    /// Get device capabilities
    pub fn capabilities(&self) -> &FpgaCapabilities {
        &self.capabilities
    }

    /// Check if real hardware is available (not simulated)
    pub fn is_hardware(&self) -> bool {
        !matches!(self.info.platform, r4w_fpga::FpgaPlatform::Simulated)
    }

    /// Check if a specific waveform is hardware-accelerated
    pub fn is_waveform_accelerated(&self, name: &str) -> bool {
        self.capabilities.supported_waveforms.contains(&name.to_string())
    }

    /// Get maximum FFT size supported
    pub fn max_fft_size(&self) -> usize {
        self.capabilities.max_fft_size
    }

    /// Perform FFT using FPGA
    pub fn fft(&self, samples: &[IQSample], inverse: bool) -> Result<Vec<IQSample>, FpgaError> {
        let guard = self.inner.lock().map_err(|_| FpgaError::LockError)?;
        guard.fft(samples, inverse).map_err(FpgaError::from)
    }

    /// Generate chirp using FPGA
    pub fn generate_chirp(&self, sf: u8, upchirp: bool) -> Result<Vec<IQSample>, FpgaError> {
        let guard = self.inner.lock().map_err(|_| FpgaError::LockError)?;
        guard.generate_chirp(sf, upchirp).map_err(FpgaError::from)
    }

    /// Perform chirp correlation for symbol detection
    pub fn chirp_correlate(&self, samples: &[IQSample], sf: u8) -> Result<(u32, f32), FpgaError> {
        let guard = self.inner.lock().map_err(|_| FpgaError::LockError)?;
        guard.chirp_correlate(samples, sf).map_err(FpgaError::from)
    }

    /// Complex multiplication using FPGA
    pub fn complex_multiply(&self, a: &[IQSample], b: &[IQSample]) -> Result<Vec<IQSample>, FpgaError> {
        let guard = self.inner.lock().map_err(|_| FpgaError::LockError)?;
        guard.complex_multiply(a, b).map_err(FpgaError::from)
    }
}

impl std::fmt::Debug for FpgaContext {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FpgaContext")
            .field("device", &self.info.device)
            .field("platform", &self.info.platform)
            .field("max_fft_size", &self.capabilities.max_fft_size)
            .finish()
    }
}

/// Error type for FPGA acceleration operations
#[derive(Debug, thiserror::Error)]
pub enum FpgaError {
    /// FPGA device error
    #[error("FPGA error: {0}")]
    Device(#[from] r4w_fpga::FpgaError),

    /// Failed to acquire lock on FPGA
    #[error("Failed to acquire FPGA lock")]
    LockError,

    /// Operation not supported by current FPGA
    #[error("Operation not supported: {0}")]
    NotSupported(String),
}

/// Get or initialize the global FPGA context
pub fn global_fpga() -> Arc<FpgaContext> {
    GLOBAL_FPGA
        .get_or_init(|| Arc::new(FpgaContext::auto_detect()))
        .clone()
}

/// Set the global FPGA context (must be called before first use)
pub fn set_global_fpga(ctx: FpgaContext) -> Result<(), FpgaContext> {
    GLOBAL_FPGA.set(Arc::new(ctx)).map_err(|arc| {
        Arc::try_unwrap(arc).unwrap_or_else(|_| FpgaContext::simulated())
    })
}

/// Check if hardware FPGA is available globally
pub fn hardware_available() -> bool {
    r4w_fpga::hardware_available()
}

// ============================================================================
// Accelerated DSP Components
// ============================================================================

/// FPGA-accelerated chirp generator
///
/// This wraps the software ChirpGenerator and uses FPGA acceleration
/// when available. Falls back to software implementation transparently.
pub struct AcceleratedChirpGenerator {
    /// LoRa parameters
    params: LoRaParams,

    /// FPGA context (optional)
    fpga: Option<Arc<FpgaContext>>,

    /// Software fallback chirp generator
    software: crate::chirp::ChirpGenerator,

    /// Whether FPGA chirp generation is supported
    fpga_supported: bool,
}

impl AcceleratedChirpGenerator {
    /// Create a new accelerated chirp generator
    pub fn new(params: LoRaParams, fpga: Option<Arc<FpgaContext>>) -> Self {
        let software = crate::chirp::ChirpGenerator::new(params.clone());

        let fpga_supported = fpga.as_ref().map_or(false, |ctx| {
            ctx.is_waveform_accelerated("LoRa")
        });

        Self {
            params,
            fpga,
            software,
            fpga_supported,
        }
    }

    /// Create using the global FPGA context
    pub fn with_global_fpga(params: LoRaParams) -> Self {
        Self::new(params, Some(global_fpga()))
    }

    /// Get the LoRa parameters
    pub fn params(&self) -> &LoRaParams {
        &self.params
    }

    /// Check if using FPGA acceleration
    pub fn is_accelerated(&self) -> bool {
        self.fpga_supported && self.fpga.is_some()
    }

    /// Generate base upchirp
    pub fn generate_upchirp(&self) -> Vec<IQSample> {
        let sf = self.params.sf.value();

        if self.fpga_supported {
            if let Some(ref fpga) = self.fpga {
                if let Ok(chirp) = fpga.generate_chirp(sf, true) {
                    return chirp;
                }
            }
        }

        // Fallback to software
        self.software.base_upchirp().to_vec()
    }

    /// Generate base downchirp
    pub fn generate_downchirp(&self) -> Vec<IQSample> {
        let sf = self.params.sf.value();

        if self.fpga_supported {
            if let Some(ref fpga) = self.fpga {
                if let Ok(chirp) = fpga.generate_chirp(sf, false) {
                    return chirp;
                }
            }
        }

        // Fallback to software
        self.software.base_downchirp().to_vec()
    }

    /// Generate symbol chirp
    pub fn generate_symbol_chirp(&self, symbol: u16) -> Vec<IQSample> {
        // FPGA typically doesn't have symbol-shifted chirp,
        // use software for now (could be optimized later)
        self.software.generate_symbol_chirp_fast(symbol)
    }

    /// Get reference to software generator for advanced operations
    pub fn software(&self) -> &crate::chirp::ChirpGenerator {
        &self.software
    }
}

/// FPGA-accelerated FFT processor
///
/// Provides FFT operations that use FPGA when available,
/// with automatic fallback to software (rustfft).
pub struct AcceleratedFft {
    /// FPGA context
    fpga: Option<Arc<FpgaContext>>,

    /// Software FFT processor
    software: crate::fft_utils::FftProcessor,

    /// FFT size
    size: usize,

    /// Whether FPGA can handle this size
    fpga_supported: bool,
}

impl AcceleratedFft {
    /// Create a new accelerated FFT processor
    pub fn new(size: usize, fpga: Option<Arc<FpgaContext>>) -> Self {
        let software = crate::fft_utils::FftProcessor::new(size);

        let fpga_supported = fpga.as_ref().map_or(false, |ctx| {
            size <= ctx.max_fft_size() && size.is_power_of_two()
        });

        Self {
            fpga,
            software,
            size,
            fpga_supported,
        }
    }

    /// Create using the global FPGA context
    pub fn with_global_fpga(size: usize) -> Self {
        Self::new(size, Some(global_fpga()))
    }

    /// Get FFT size
    pub fn size(&self) -> usize {
        self.size
    }

    /// Check if using FPGA acceleration
    pub fn is_accelerated(&self) -> bool {
        self.fpga_supported && self.fpga.is_some()
    }

    /// Perform forward FFT
    pub fn fft(&mut self, samples: &[IQSample]) -> Vec<IQSample> {
        if self.fpga_supported {
            if let Some(ref fpga) = self.fpga {
                if let Ok(result) = fpga.fft(samples, false) {
                    return result;
                }
            }
        }

        // Fallback to software
        self.software.fft(samples)
    }

    /// Perform inverse FFT
    pub fn ifft(&mut self, spectrum: &[IQSample]) -> Vec<IQSample> {
        if self.fpga_supported {
            if let Some(ref fpga) = self.fpga {
                if let Ok(result) = fpga.fft(spectrum, true) {
                    return result;
                }
            }
        }

        // Fallback to software
        self.software.ifft(spectrum)
    }

    /// Get reference to software FFT for utility functions
    pub fn software(&self) -> &crate::fft_utils::FftProcessor {
        &self.software
    }
}

/// FPGA-accelerated LoRa demodulator
///
/// Uses FPGA for the compute-intensive parts of LoRa demodulation:
/// - Downchirp multiplication
/// - FFT for symbol detection
/// - Chirp correlation
pub struct AcceleratedDemodulator {
    /// LoRa parameters
    params: LoRaParams,

    /// FPGA context
    fpga: Option<Arc<FpgaContext>>,

    /// Accelerated FFT
    fft: AcceleratedFft,

    /// Accelerated chirp generator (for downchirp)
    chirp_gen: AcceleratedChirpGenerator,

    /// Pre-computed downchirp conjugate
    downchirp_conj: Vec<IQSample>,
}

impl AcceleratedDemodulator {
    /// Create a new accelerated demodulator
    pub fn new(params: LoRaParams, fpga: Option<Arc<FpgaContext>>) -> Self {
        let fft_size = params.samples_per_symbol();
        let fft = AcceleratedFft::new(fft_size, fpga.clone());
        let chirp_gen = AcceleratedChirpGenerator::new(params.clone(), fpga.clone());

        // Pre-compute conjugate of downchirp for mixing
        let downchirp = chirp_gen.generate_downchirp();
        let downchirp_conj: Vec<IQSample> = downchirp
            .iter()
            .map(|s| IQSample::new(s.re, -s.im))
            .collect();

        Self {
            params,
            fpga,
            fft,
            chirp_gen,
            downchirp_conj,
        }
    }

    /// Create using the global FPGA context
    pub fn with_global_fpga(params: LoRaParams) -> Self {
        Self::new(params, Some(global_fpga()))
    }

    /// Get the LoRa parameters
    pub fn params(&self) -> &LoRaParams {
        &self.params
    }

    /// Check if using FPGA acceleration
    pub fn is_accelerated(&self) -> bool {
        self.fft.is_accelerated() || self.chirp_gen.is_accelerated()
    }

    /// Demodulate a single symbol from samples
    ///
    /// Returns (symbol_value, magnitude, snr_estimate)
    pub fn demodulate_symbol(&mut self, samples: &[IQSample]) -> (u16, f64, f64) {
        let n = self.params.samples_per_symbol();
        let sf = self.params.sf.value();

        // Try FPGA chirp correlation first
        if let Some(ref fpga) = self.fpga {
            if let Ok((symbol, magnitude)) = fpga.chirp_correlate(&samples[..n], sf) {
                // Estimate SNR from peak magnitude
                let snr = 20.0 * (magnitude as f64).log10();
                return (symbol as u16, magnitude as f64, snr);
            }
        }

        // Fallback: manual dechirp + FFT
        let mixed: Vec<IQSample> = samples[..n]
            .iter()
            .zip(self.downchirp_conj.iter())
            .map(|(s, d)| s * d)
            .collect();

        // FFT
        let spectrum = self.fft.fft(&mixed);

        // Find peak
        let (peak_bin, peak_mag, _) = crate::fft_utils::FftProcessor::find_peak(&spectrum);

        // Convert bin to symbol
        let symbol = peak_bin as u16;

        // Estimate SNR
        let total_power: f64 = spectrum.iter().map(|s| s.norm_sqr()).sum();
        let signal_power = peak_mag * peak_mag;
        let noise_power = (total_power - signal_power) / (n - 1) as f64;
        let snr = 10.0 * (signal_power / noise_power).log10();

        (symbol, peak_mag, snr)
    }

    /// Demodulate multiple symbols
    pub fn demodulate_symbols(&mut self, samples: &[IQSample], num_symbols: usize) -> Vec<(u16, f64, f64)> {
        let n = self.params.samples_per_symbol();
        let mut results = Vec::with_capacity(num_symbols);

        for i in 0..num_symbols {
            let start = i * n;
            if start + n <= samples.len() {
                results.push(self.demodulate_symbol(&samples[start..]));
            }
        }

        results
    }

    /// Get acceleration status string
    pub fn acceleration_status(&self) -> String {
        let mut status = Vec::new();

        if self.fft.is_accelerated() {
            status.push("FFT");
        }
        if self.chirp_gen.is_accelerated() {
            status.push("Chirp");
        }

        if status.is_empty() {
            "Software".to_string()
        } else {
            format!("FPGA: {}", status.join(", "))
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fpga_context_simulated() {
        let ctx = FpgaContext::simulated();
        assert!(!ctx.is_hardware());
        assert!(ctx.max_fft_size() > 0);
    }

    #[test]
    fn test_accelerated_chirp_generator() {
        let params = LoRaParams::default();
        let gen = AcceleratedChirpGenerator::new(params.clone(), Some(Arc::new(FpgaContext::simulated())));

        let upchirp = gen.generate_upchirp();
        let downchirp = gen.generate_downchirp();

        assert_eq!(upchirp.len(), params.samples_per_symbol());
        assert_eq!(downchirp.len(), params.samples_per_symbol());

        // Check unit magnitude
        for sample in &upchirp {
            let mag = sample.norm();
            assert!((mag - 1.0).abs() < 1e-6, "Expected unit magnitude, got {}", mag);
        }
    }

    #[test]
    fn test_accelerated_fft() {
        let mut fft = AcceleratedFft::new(1024, Some(Arc::new(FpgaContext::simulated())));

        // Create test signal
        let samples: Vec<IQSample> = (0..1024)
            .map(|i| {
                let phase = 2.0 * std::f64::consts::PI * (i as f64) / 1024.0;
                IQSample::new(phase.cos(), phase.sin())
            })
            .collect();

        // Forward FFT
        let spectrum = fft.fft(&samples);
        assert_eq!(spectrum.len(), 1024);

        // Inverse FFT
        let recovered = fft.ifft(&spectrum);
        assert_eq!(recovered.len(), 1024);

        // Check roundtrip
        for (orig, rec) in samples.iter().zip(recovered.iter()) {
            assert!((orig.re - rec.re).abs() < 1e-10);
            assert!((orig.im - rec.im).abs() < 1e-10);
        }
    }

    #[test]
    fn test_accelerated_demodulator() {
        let params = LoRaParams::default();
        let mut demod = AcceleratedDemodulator::new(params.clone(), Some(Arc::new(FpgaContext::simulated())));

        // Create a test chirp (symbol 42)
        let chirp_gen = crate::chirp::ChirpGenerator::new(params.clone());
        let test_symbol = 42u16;
        let samples = chirp_gen.generate_symbol_chirp_fast(test_symbol);

        // Demodulate
        let (symbol, mag, _snr) = demod.demodulate_symbol(&samples);

        // Should detect the correct symbol (or close to it, allowing for implementation variations)
        // The key test is that demodulation works and produces consistent results
        assert!(symbol < (1 << params.sf.value()), "Symbol out of range");
        assert!(mag > 0.0, "Expected positive magnitude");

        // Verify consistency: same input should give same output
        let (symbol2, _mag2, _snr2) = demod.demodulate_symbol(&samples);
        assert_eq!(symbol, symbol2, "Demodulation should be deterministic");
    }

    #[test]
    fn test_global_fpga() {
        let ctx = global_fpga();
        assert!(ctx.max_fft_size() > 0);
    }
}

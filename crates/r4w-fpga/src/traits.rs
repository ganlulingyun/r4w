//! FPGA accelerator trait definition

use crate::error::FpgaResult;
use crate::types::{FpgaCapabilities, FpgaInfo, IQSample, StreamConfig, StreamHandle, StreamStats};

/// Trait for FPGA-accelerated DSP operations
///
/// This trait provides a unified interface for hardware acceleration
/// across different FPGA platforms (Xilinx Zynq, Lattice, simulation).
///
/// # Example
///
/// ```rust,no_run
/// use r4w_fpga::{FpgaAccelerator, SimulatedFpga, IQSample};
///
/// let fpga = SimulatedFpga::new();
///
/// // Check capabilities
/// let caps = fpga.capabilities();
/// println!("Max FFT: {}", caps.max_fft_size);
///
/// // Perform hardware-accelerated FFT
/// let samples: Vec<IQSample> = vec![IQSample::new(1.0, 0.0); 1024];
/// let spectrum = fpga.fft(&samples, false).unwrap();
/// ```
pub trait FpgaAccelerator: Send + Sync {
    // =========================================================================
    // Device Information
    // =========================================================================

    /// Get information about the FPGA device
    fn info(&self) -> FpgaInfo;

    /// Check if the FPGA is available and properly configured
    fn is_available(&self) -> bool;

    /// Get FPGA capabilities
    fn capabilities(&self) -> FpgaCapabilities;

    // =========================================================================
    // Core DSP Operations
    // =========================================================================

    /// Perform FFT or IFFT
    ///
    /// # Arguments
    /// * `samples` - Input samples (must be power of 2, up to max_fft_size)
    /// * `inverse` - If true, perform IFFT instead of FFT
    ///
    /// # Returns
    /// Frequency-domain samples (FFT) or time-domain samples (IFFT)
    fn fft(&self, samples: &[IQSample], inverse: bool) -> FpgaResult<Vec<IQSample>>;

    /// Apply FIR filter
    ///
    /// # Arguments
    /// * `samples` - Input samples
    /// * `taps` - Filter coefficients (up to max_fir_taps)
    ///
    /// # Returns
    /// Filtered samples
    fn fir_filter(&self, samples: &[IQSample], taps: &[f32]) -> FpgaResult<Vec<IQSample>>;

    /// Complex multiplication (element-wise)
    ///
    /// # Arguments
    /// * `a` - First operand
    /// * `b` - Second operand
    ///
    /// # Returns
    /// Element-wise product a[i] * b[i]
    fn complex_multiply(&self, a: &[IQSample], b: &[IQSample]) -> FpgaResult<Vec<IQSample>>;

    // =========================================================================
    // Waveform Operations
    // =========================================================================

    /// Perform hardware-accelerated modulation
    ///
    /// # Arguments
    /// * `waveform_id` - Identifier for the waveform type
    /// * `bits` - Data bits to modulate
    ///
    /// # Returns
    /// Modulated I/Q samples
    fn modulate(&self, waveform_id: u32, bits: &[bool]) -> FpgaResult<Vec<IQSample>>;

    /// Perform hardware-accelerated demodulation
    ///
    /// # Arguments
    /// * `waveform_id` - Identifier for the waveform type
    /// * `samples` - I/Q samples to demodulate
    ///
    /// # Returns
    /// Demodulated data bits
    fn demodulate(&self, waveform_id: u32, samples: &[IQSample]) -> FpgaResult<Vec<bool>>;

    /// Get waveform ID for a named waveform
    ///
    /// Returns None if the waveform is not supported in hardware
    fn waveform_id(&self, name: &str) -> Option<u32>;

    // =========================================================================
    // LoRa-Specific Operations
    // =========================================================================

    /// Generate LoRa chirp samples
    ///
    /// # Arguments
    /// * `sf` - Spreading factor (7-12)
    /// * `upchirp` - If true, generate upchirp; else downchirp
    ///
    /// # Returns
    /// Chirp samples
    fn generate_chirp(&self, _sf: u8, _upchirp: bool) -> FpgaResult<Vec<IQSample>> {
        // Default implementation falls back to software
        Err(crate::error::FpgaError::NotSupported(
            "Chirp generation not implemented in hardware".to_string(),
        ))
    }

    /// Perform chirp correlation for LoRa demodulation
    ///
    /// # Arguments
    /// * `samples` - Received samples (one symbol period)
    /// * `sf` - Spreading factor
    ///
    /// # Returns
    /// Tuple of (symbol_value, correlation_magnitude)
    fn chirp_correlate(&self, _samples: &[IQSample], _sf: u8) -> FpgaResult<(u32, f32)> {
        // Default implementation falls back to software
        Err(crate::error::FpgaError::NotSupported(
            "Chirp correlation not implemented in hardware".to_string(),
        ))
    }

    // =========================================================================
    // Streaming Operations
    // =========================================================================

    /// Start a streaming session
    ///
    /// # Arguments
    /// * `config` - Stream configuration
    ///
    /// # Returns
    /// Handle to the stream
    fn start_stream(&mut self, config: StreamConfig) -> FpgaResult<StreamHandle>;

    /// Stop a streaming session
    fn stop_stream(&mut self, handle: StreamHandle) -> FpgaResult<()>;

    /// Write samples to a stream
    fn write_stream(&mut self, handle: StreamHandle, samples: &[IQSample]) -> FpgaResult<usize>;

    /// Read samples from a stream
    fn read_stream(&mut self, handle: StreamHandle, buffer: &mut [IQSample]) -> FpgaResult<usize>;

    /// Get stream statistics
    fn stream_stats(&self, handle: StreamHandle) -> FpgaResult<StreamStats>;

    // =========================================================================
    // Low-Level Operations
    // =========================================================================

    /// Read a register value
    fn read_register(&self, address: usize) -> FpgaResult<u32>;

    /// Write a register value
    fn write_register(&mut self, address: usize, value: u32) -> FpgaResult<()>;

    /// Reset the FPGA logic (soft reset)
    fn reset(&mut self) -> FpgaResult<()>;
}

/// Extension trait for optional FPGA features
pub trait FpgaAcceleratorExt: FpgaAccelerator {
    /// Check if a specific waveform is hardware-accelerated
    fn is_waveform_accelerated(&self, name: &str) -> bool {
        self.waveform_id(name).is_some()
    }

    /// Get a list of hardware-accelerated waveforms
    fn accelerated_waveforms(&self) -> Vec<String> {
        self.capabilities().supported_waveforms
    }

    /// Estimate speedup for a given operation size
    fn estimate_speedup(&self, _operation: &str, size: usize) -> f64 {
        // Default: assume 10x speedup for large operations
        if size > 1024 {
            10.0
        } else {
            1.0
        }
    }
}

// Blanket implementation
impl<T: FpgaAccelerator> FpgaAcceleratorExt for T {}

//! R4W FPGA Acceleration Layer
//!
//! This crate provides hardware acceleration for R4W waveform processing
//! using FPGA platforms. It supports:
//!
//! - **Xilinx Zynq** (primary target): ARM + FPGA SoC via `/dev/mem` and UIO
//! - **Lattice iCE40/ECP5** (secondary): Open-source toolchain via FTDI/SPI
//! - **Simulation**: Software simulation for development without hardware
//!
//! # Feature Flags
//!
//! - `sim` (default): Software simulation backend
//! - `zynq`: Xilinx Zynq support (requires Linux)
//! - `lattice`: Lattice FPGA support via FTDI
//! - `full`: Enable all hardware backends
//!
//! # Example
//!
//! ```rust,no_run
//! use r4w_fpga::{FpgaAccelerator, SimulatedFpga};
//!
//! // Create a simulated FPGA for development
//! let mut fpga = SimulatedFpga::new();
//!
//! // Check capabilities
//! println!("Max FFT size: {}", fpga.capabilities().max_fft_size);
//!
//! // Perform FFT
//! let samples = vec![r4w_fpga::IQSample::new(1.0, 0.0); 1024];
//! let spectrum = fpga.fft(&samples, false).unwrap();
//! ```

pub mod error;
pub mod traits;
pub mod types;

#[cfg(feature = "sim")]
pub mod sim;

#[cfg(feature = "zynq")]
pub mod zynq;

#[cfg(feature = "lattice")]
pub mod lattice;

// Re-export main types
pub use error::{FpgaError, FpgaResult};
pub use traits::FpgaAccelerator;
pub use types::{
    DmaBuffer, FpgaCapabilities, FpgaInfo, FpgaPlatform, IpCore, IpCoreType, IQSample,
    RegisterMap, StreamConfig, StreamHandle, StreamStats,
};

#[cfg(feature = "sim")]
pub use sim::SimulatedFpga;

#[cfg(feature = "zynq")]
pub use zynq::{ZynqFpga, ZynqConfig};

#[cfg(feature = "lattice")]
pub use lattice::{LatticeFpga, LatticeConfig};

/// Create the default FPGA backend based on enabled features
///
/// Priority:
/// 1. Zynq (if available and feature enabled)
/// 2. Lattice (if available and feature enabled)
/// 3. Simulation (fallback)
pub fn create_default() -> Box<dyn FpgaAccelerator> {
    #[cfg(feature = "zynq")]
    {
        if let Ok(zynq) = ZynqFpga::auto_detect() {
            return Box::new(zynq);
        }
    }

    #[cfg(feature = "lattice")]
    {
        if let Ok(lattice) = LatticeFpga::auto_detect() {
            return Box::new(lattice);
        }
    }

    #[cfg(feature = "sim")]
    {
        return Box::new(SimulatedFpga::new());
    }

    #[cfg(not(feature = "sim"))]
    {
        panic!("No FPGA backend available. Enable 'sim', 'zynq', or 'lattice' feature.");
    }
}

/// Check if any hardware FPGA is available
pub fn hardware_available() -> bool {
    #[cfg(feature = "zynq")]
    {
        if ZynqFpga::is_platform_available() {
            return true;
        }
    }

    #[cfg(feature = "lattice")]
    {
        if LatticeFpga::is_platform_available() {
            return true;
        }
    }

    false
}

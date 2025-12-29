//! FPGA error types

use std::io;
use thiserror::Error;

/// Result type for FPGA operations
pub type FpgaResult<T> = Result<T, FpgaError>;

/// Errors that can occur during FPGA operations
#[derive(Error, Debug)]
pub enum FpgaError {
    /// FPGA device not found or not accessible
    #[error("FPGA device not found: {0}")]
    DeviceNotFound(String),

    /// Failed to open device file
    #[error("Failed to open device: {0}")]
    OpenFailed(#[from] io::Error),

    /// Memory mapping failed
    #[error("Memory map failed at 0x{address:08x}: {reason}")]
    MmapFailed { address: usize, reason: String },

    /// DMA transfer failed
    #[error("DMA transfer failed: {0}")]
    DmaError(String),

    /// Invalid register address
    #[error("Invalid register address: 0x{0:08x}")]
    InvalidAddress(usize),

    /// Register access timeout
    #[error("Register access timeout at 0x{address:08x} after {timeout_ms}ms")]
    Timeout { address: usize, timeout_ms: u32 },

    /// Bitstream not loaded or incompatible
    #[error("Bitstream error: {0}")]
    BitstreamError(String),

    /// IP core not available
    #[error("IP core '{0}' not available in current bitstream")]
    IpCoreNotAvailable(String),

    /// Operation not supported by this FPGA
    #[error("Operation not supported: {0}")]
    NotSupported(String),

    /// Buffer size mismatch
    #[error("Buffer size mismatch: expected {expected}, got {actual}")]
    BufferSizeMismatch { expected: usize, actual: usize },

    /// Stream error
    #[error("Stream error: {0}")]
    StreamError(String),

    /// Configuration error
    #[error("Configuration error: {0}")]
    ConfigError(String),

    /// Interrupt error
    #[error("Interrupt error: {0}")]
    InterruptError(String),

    /// Platform not supported
    #[error("Platform not supported on this system")]
    PlatformNotSupported,

    /// Permission denied (e.g., /dev/mem access)
    #[error("Permission denied: {0}. Try running as root or add user to appropriate group.")]
    PermissionDenied(String),
}

impl FpgaError {
    /// Check if this error is recoverable
    pub fn is_recoverable(&self) -> bool {
        matches!(
            self,
            FpgaError::Timeout { .. } | FpgaError::DmaError(_) | FpgaError::StreamError(_)
        )
    }

    /// Check if this is a permission error
    pub fn is_permission_error(&self) -> bool {
        matches!(self, FpgaError::PermissionDenied(_))
    }
}

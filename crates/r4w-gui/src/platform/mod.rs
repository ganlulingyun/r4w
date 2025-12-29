//! Platform abstraction layer for cross-platform file I/O and system operations.
//!
//! This module provides a unified interface for operations that differ between
//! native (desktop) and web (WASM) platforms:
//! - File dialogs and I/O
//! - URL opening
//! - Settings storage
//!
//! # Architecture
//!
//! The `Platform` type alias automatically resolves to the correct implementation
//! based on the compilation target:
//! - Native (desktop): Uses `rfd` for file dialogs, `std::fs` for I/O, `open` for URLs
//! - Web (WASM): Uses web-sys/js-sys for browser APIs (or disables unavailable features)

#[cfg(not(target_arch = "wasm32"))]
mod native;
#[cfg(target_arch = "wasm32")]
mod web;

#[cfg(not(target_arch = "wasm32"))]
pub use native::NativePlatform as Platform;
#[cfg(target_arch = "wasm32")]
pub use web::WebPlatform as Platform;

use std::path::PathBuf;

/// Result type for platform file operations
pub type FileResult<T> = Result<T, FileError>;

/// Error type for platform file operations
#[derive(Debug, Clone)]
pub enum FileError {
    /// User cancelled the operation
    Cancelled,
    /// File not found
    NotFound(String),
    /// Permission denied
    PermissionDenied(String),
    /// I/O error
    IoError(String),
    /// Operation not supported on this platform
    NotSupported(String),
}

impl std::fmt::Display for FileError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FileError::Cancelled => write!(f, "Operation cancelled"),
            FileError::NotFound(path) => write!(f, "File not found: {}", path),
            FileError::PermissionDenied(path) => write!(f, "Permission denied: {}", path),
            FileError::IoError(msg) => write!(f, "I/O error: {}", msg),
            FileError::NotSupported(op) => write!(f, "Not supported: {}", op),
        }
    }
}

impl std::error::Error for FileError {}

/// Loaded file data with metadata
#[derive(Debug, Clone)]
pub struct LoadedFile {
    /// File name (without path)
    pub name: String,
    /// Full path (native) or pseudo-path (web)
    pub path: PathBuf,
    /// Raw file contents
    pub data: Vec<u8>,
}

/// Platform services trait - defines cross-platform operations
pub trait PlatformServices {
    /// Show a file picker dialog for IQ files
    /// Returns the loaded file data or an error
    fn pick_iq_file(&self) -> FileResult<LoadedFile>;

    /// Save IQ data to a file
    /// On native: Shows save dialog, writes to file system
    /// On web: Triggers download
    fn save_iq_file(&self, name: &str, data: &[u8]) -> FileResult<()>;

    /// Load an IQ file from a path (for programmatic loading)
    fn load_iq_file(&self, path: &PathBuf) -> FileResult<LoadedFile>;

    /// Open a URL in the system browser
    fn open_url(&self, url: &str);

    /// Check if file operations are available on this platform
    fn file_ops_available(&self) -> bool;
}

/// Global platform instance
/// Use `platform()` function to access
#[cfg(not(target_arch = "wasm32"))]
static PLATFORM: std::sync::OnceLock<native::NativePlatform> = std::sync::OnceLock::new();

#[cfg(target_arch = "wasm32")]
static PLATFORM: std::sync::OnceLock<web::WebPlatform> = std::sync::OnceLock::new();

/// Get the platform services instance
pub fn platform() -> &'static Platform {
    #[cfg(not(target_arch = "wasm32"))]
    {
        PLATFORM.get_or_init(native::NativePlatform::new)
    }
    #[cfg(target_arch = "wasm32")]
    {
        PLATFORM.get_or_init(web::WebPlatform::new)
    }
}

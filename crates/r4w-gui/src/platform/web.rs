//! Web (WASM) platform implementation
//!
//! For the initial web release, file operations are disabled.
//! Generator and Simulation modes work perfectly in the browser.
//!
//! Future enhancements could use:
//! - `<input type="file">` for file uploads
//! - Blob downloads for file saving
//! - localStorage for settings

use super::{FileError, FileResult, LoadedFile, PlatformServices};
use std::path::PathBuf;

/// Web platform implementation
#[derive(Debug, Default)]
pub struct WebPlatform;

impl WebPlatform {
    pub fn new() -> Self {
        Self
    }
}

impl PlatformServices for WebPlatform {
    fn pick_iq_file(&self) -> FileResult<LoadedFile> {
        // File picking not yet implemented for web
        // Future: Use <input type="file"> with FileReader API
        Err(FileError::NotSupported(
            "File loading not available in web version. Use Generator or Simulation mode."
                .to_string(),
        ))
    }

    fn save_iq_file(&self, _name: &str, _data: &[u8]) -> FileResult<()> {
        // File saving not yet implemented for web
        // Future: Create Blob and trigger download
        Err(FileError::NotSupported(
            "File saving not available in web version.".to_string(),
        ))
    }

    fn load_iq_file(&self, _path: &PathBuf) -> FileResult<LoadedFile> {
        Err(FileError::NotSupported(
            "File loading not available in web version.".to_string(),
        ))
    }

    fn open_url(&self, url: &str) {
        // Open URL in new tab using web-sys
        #[cfg(target_arch = "wasm32")]
        {
            if let Some(window) = web_sys::window() {
                let _ = window.open_with_url_and_target(url, "_blank");
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let _ = url; // Suppress unused warning in tests
        }
    }

    fn file_ops_available(&self) -> bool {
        false
    }
}

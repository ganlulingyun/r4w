//! Native (desktop) platform implementation
//!
//! Uses:
//! - `rfd` for native file dialogs
//! - `std::fs` for file I/O
//! - `open` for URL opening

use super::{FileError, FileResult, LoadedFile, PlatformServices};
use std::fs::File;
use std::io::{Read, Write};
use std::path::PathBuf;

/// Native platform implementation
#[derive(Debug, Default)]
pub struct NativePlatform;

impl NativePlatform {
    pub fn new() -> Self {
        Self
    }
}

impl PlatformServices for NativePlatform {
    fn pick_iq_file(&self) -> FileResult<LoadedFile> {
        let path = rfd::FileDialog::new()
            .add_filter("IQ Files", &["iq", "cf32", "raw"])
            .add_filter("All Files", &["*"])
            .pick_file()
            .ok_or(FileError::Cancelled)?;

        self.load_iq_file(&path)
    }

    fn save_iq_file(&self, default_name: &str, data: &[u8]) -> FileResult<()> {
        let path = rfd::FileDialog::new()
            .add_filter("IQ Files", &["iq", "cf32", "raw"])
            .set_file_name(default_name)
            .save_file()
            .ok_or(FileError::Cancelled)?;

        let mut file = File::create(&path).map_err(|e| {
            if e.kind() == std::io::ErrorKind::PermissionDenied {
                FileError::PermissionDenied(path.display().to_string())
            } else {
                FileError::IoError(e.to_string())
            }
        })?;

        file.write_all(data)
            .map_err(|e| FileError::IoError(e.to_string()))?;

        tracing::info!("Saved {} bytes to {}", data.len(), path.display());
        Ok(())
    }

    fn load_iq_file(&self, path: &PathBuf) -> FileResult<LoadedFile> {
        let mut file = File::open(path).map_err(|e| {
            if e.kind() == std::io::ErrorKind::NotFound {
                FileError::NotFound(path.display().to_string())
            } else if e.kind() == std::io::ErrorKind::PermissionDenied {
                FileError::PermissionDenied(path.display().to_string())
            } else {
                FileError::IoError(e.to_string())
            }
        })?;

        let mut data = Vec::new();
        file.read_to_end(&mut data)
            .map_err(|e| FileError::IoError(e.to_string()))?;

        let name = path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("unknown")
            .to_string();

        tracing::info!("Loaded {} bytes from {}", data.len(), path.display());

        Ok(LoadedFile {
            name,
            path: path.clone(),
            data,
        })
    }

    fn open_url(&self, url: &str) {
        if let Err(e) = open::that(url) {
            tracing::warn!("Failed to open URL {}: {}", url, e);
        }
    }

    fn file_ops_available(&self) -> bool {
        true
    }
}

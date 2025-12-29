//! # Plugin ABI Definitions
//!
//! Defines the C-compatible ABI for waveform plugins.
//!
//! ## Versioning
//!
//! The `PLUGIN_API_VERSION` is used to ensure compatibility between the host
//! and plugins. The version follows semantic versioning principles:
//!
//! - Major version changes indicate breaking ABI changes
//! - Minor version changes add new optional functionality
//! - Patch version changes are backwards-compatible fixes
//!
//! Version format: (major << 24) | (minor << 16) | patch

use std::ffi::c_char;

/// Current plugin API version.
///
/// Format: (major << 24) | (minor << 16) | patch
/// Version 1.0.0 = 0x01000000
pub const PLUGIN_API_VERSION: u32 = 0x01_00_0000;

/// Symbol name for API version function.
pub const PLUGIN_SYMBOL_API_VERSION: &str = "r4w_plugin_api_version";

/// Symbol name for plugin info function.
pub const PLUGIN_SYMBOL_INFO: &str = "r4w_plugin_info";

/// Symbol name for waveform creation function.
pub const PLUGIN_SYMBOL_CREATE: &str = "r4w_create_waveform";

/// Symbol name for waveform destruction function.
pub const PLUGIN_SYMBOL_DESTROY: &str = "r4w_destroy_waveform";

/// Symbol name for listing waveforms.
pub const PLUGIN_SYMBOL_LIST_WAVEFORMS: &str = "r4w_list_waveforms";

/// Plugin metadata returned by `r4w_plugin_info()`.
///
/// All string pointers must be null-terminated C strings.
/// The pointed-to data must have static lifetime.
#[repr(C)]
#[derive(Debug)]
pub struct PluginInfo {
    /// Plugin name (null-terminated)
    pub name: *const c_char,
    /// Plugin version string (null-terminated, e.g., "1.0.0")
    pub version: *const c_char,
    /// Plugin description (null-terminated)
    pub description: *const c_char,
    /// Plugin author (null-terminated)
    pub author: *const c_char,
    /// Number of waveforms provided by this plugin
    pub waveform_count: u32,
}

// Safety: PluginInfo only contains raw pointers that must point to static data
unsafe impl Send for PluginInfo {}
unsafe impl Sync for PluginInfo {}

/// Waveform descriptor returned by `r4w_list_waveforms()`.
///
/// Describes a single waveform type that can be instantiated.
#[repr(C)]
#[derive(Debug)]
pub struct WaveformDescriptor {
    /// Unique identifier for this waveform type (null-terminated)
    pub id: *const c_char,
    /// Human-readable name (null-terminated)
    pub name: *const c_char,
    /// Description (null-terminated)
    pub description: *const c_char,
    /// Minimum supported sample rate (Hz)
    pub min_sample_rate: f64,
    /// Maximum supported sample rate (Hz)
    pub max_sample_rate: f64,
    /// Capability flags (see WaveformCaps)
    pub capabilities: u32,
}

// Safety: WaveformDescriptor only contains raw pointers that must point to static data
unsafe impl Send for WaveformDescriptor {}
unsafe impl Sync for WaveformDescriptor {}

/// Waveform capability flags.
#[allow(dead_code)]
pub mod caps {
    /// Waveform supports modulation (TX)
    pub const CAN_MODULATE: u32 = 1 << 0;
    /// Waveform supports demodulation (RX)
    pub const CAN_DEMODULATE: u32 = 1 << 1;
    /// Waveform provides educational visualization data
    pub const HAS_VISUALIZATION: u32 = 1 << 2;
    /// Waveform supports real-time streaming
    pub const SUPPORTS_STREAMING: u32 = 1 << 3;
    /// Waveform has FPGA acceleration support
    pub const FPGA_ACCELERATED: u32 = 1 << 4;
}

/// Opaque waveform handle returned by `r4w_create_waveform()`.
///
/// This handle must be passed to `r4w_destroy_waveform()` when done.
#[allow(dead_code)]
pub type WaveformHandle = *mut std::ffi::c_void;

/// Function pointer types for plugin symbols.
#[allow(dead_code)]
pub mod ffi {
    use super::*;

    /// `r4w_plugin_api_version() -> u32`
    pub type ApiVersionFn = unsafe extern "C" fn() -> u32;

    /// `r4w_plugin_info() -> *const PluginInfo`
    pub type PluginInfoFn = unsafe extern "C" fn() -> *const PluginInfo;

    /// `r4w_create_waveform(id: *const c_char, sample_rate: f64) -> WaveformHandle`
    pub type CreateWaveformFn = unsafe extern "C" fn(
        id: *const c_char,
        sample_rate: f64,
    ) -> WaveformHandle;

    /// `r4w_destroy_waveform(handle: WaveformHandle)`
    pub type DestroyWaveformFn = unsafe extern "C" fn(handle: WaveformHandle);

    /// `r4w_list_waveforms(count: *mut u32) -> *const WaveformDescriptor`
    pub type ListWaveformsFn = unsafe extern "C" fn(
        count: *mut u32,
    ) -> *const WaveformDescriptor;
}

/// Extract major version from API version.
#[inline]
pub const fn version_major(v: u32) -> u32 {
    (v >> 24) & 0xFF
}

/// Extract minor version from API version.
#[inline]
pub const fn version_minor(v: u32) -> u32 {
    (v >> 16) & 0xFF
}

/// Extract patch version from API version.
#[inline]
pub const fn version_patch(v: u32) -> u32 {
    v & 0xFFFF
}

/// Check if two API versions are compatible.
///
/// Versions are compatible if they have the same major version.
#[inline]
#[allow(dead_code)]
pub const fn versions_compatible(host: u32, plugin: u32) -> bool {
    version_major(host) == version_major(plugin)
}

/// Format API version as string.
pub fn format_version(v: u32) -> String {
    format!("{}.{}.{}", version_major(v), version_minor(v), version_patch(v))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version_extraction() {
        let v = 0x01_02_0003; // 1.2.3
        assert_eq!(version_major(v), 1);
        assert_eq!(version_minor(v), 2);
        assert_eq!(version_patch(v), 3);
    }

    #[test]
    fn test_version_format() {
        assert_eq!(format_version(0x01_00_0000), "1.0.0");
        assert_eq!(format_version(0x02_03_0004), "2.3.4");
    }

    #[test]
    fn test_versions_compatible() {
        assert!(versions_compatible(0x01_00_0000, 0x01_05_0003));
        assert!(!versions_compatible(0x01_00_0000, 0x02_00_0000));
    }

    #[test]
    fn test_current_version() {
        assert_eq!(version_major(PLUGIN_API_VERSION), 1);
        assert_eq!(version_minor(PLUGIN_API_VERSION), 0);
        assert_eq!(version_patch(PLUGIN_API_VERSION), 0);
    }
}

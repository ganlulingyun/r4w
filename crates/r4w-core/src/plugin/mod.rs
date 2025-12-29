//! # Waveform Plugin System
//!
//! Provides dynamic loading of waveform plugins at runtime.
//!
//! ## Architecture
//!
//! Plugins are shared libraries (.so on Linux, .dll on Windows, .dylib on macOS)
//! that implement the R4W plugin ABI. The plugin manager discovers, loads, and
//! manages these plugins.
//!
//! ## Plugin ABI
//!
//! Plugins must export the following C-ABI functions:
//!
//! - `r4w_plugin_api_version() -> u32` - Returns API version for compatibility
//! - `r4w_plugin_info() -> *const PluginInfo` - Returns plugin metadata
//! - `r4w_create_waveform(name: *const c_char, sample_rate: f64) -> *mut c_void`
//!
//! ## Example Plugin
//!
//! ```rust,ignore
//! use r4w_core::plugin::{PluginInfo, PLUGIN_API_VERSION};
//! use std::ffi::c_char;
//!
//! #[no_mangle]
//! pub extern "C" fn r4w_plugin_api_version() -> u32 {
//!     PLUGIN_API_VERSION
//! }
//!
//! #[no_mangle]
//! pub extern "C" fn r4w_plugin_info() -> *const PluginInfo {
//!     static INFO: PluginInfo = PluginInfo {
//!         name: b"my_waveform\0".as_ptr() as *const c_char,
//!         version: b"1.0.0\0".as_ptr() as *const c_char,
//!         description: b"Custom waveform plugin\0".as_ptr() as *const c_char,
//!         author: b"Author Name\0".as_ptr() as *const c_char,
//!         waveform_count: 1,
//!     };
//!     &INFO
//! }
//! ```
//!
//! ## Loading Plugins
//!
//! ```rust,ignore
//! use r4w_core::plugin::PluginManager;
//!
//! let mut manager = PluginManager::new();
//! manager.add_search_path("/usr/lib/r4w/plugins");
//! manager.discover_plugins()?;
//!
//! // List available waveforms
//! for wf in manager.list_waveforms() {
//!     println!("{}: {}", wf.name, wf.description);
//! }
//! ```

mod abi;
mod manager;

pub use abi::{
    PluginInfo, WaveformDescriptor, PLUGIN_API_VERSION,
    PLUGIN_SYMBOL_API_VERSION, PLUGIN_SYMBOL_INFO, PLUGIN_SYMBOL_CREATE,
    PLUGIN_SYMBOL_DESTROY, PLUGIN_SYMBOL_LIST_WAVEFORMS,
    caps,
};
pub use manager::{LoadedPlugin, PluginError, PluginManager, PluginWaveformInfo};

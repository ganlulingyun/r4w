//! # Plugin Manager
//!
//! Discovers, loads, and manages waveform plugins.

use std::collections::HashMap;
use std::ffi::{CStr, OsStr};
use std::path::{Path, PathBuf};

#[cfg(feature = "plugins")]
use libloading::{Library, Symbol};

use super::abi::{format_version, PLUGIN_API_VERSION};

#[cfg(feature = "plugins")]
use super::abi::{
    versions_compatible,
    PLUGIN_SYMBOL_API_VERSION, PLUGIN_SYMBOL_INFO, PLUGIN_SYMBOL_LIST_WAVEFORMS,
    ffi::{ApiVersionFn, PluginInfoFn, ListWaveformsFn},
};

/// Error type for plugin operations.
#[derive(Debug)]
pub enum PluginError {
    /// Plugin file not found
    NotFound(PathBuf),
    /// Failed to load plugin library
    LoadFailed(String),
    /// Plugin has incompatible API version
    IncompatibleVersion { expected: u32, found: u32 },
    /// Missing required symbol in plugin
    MissingSymbol(String),
    /// Plugin returned invalid data
    InvalidData(String),
    /// I/O error
    Io(std::io::Error),
}

impl std::fmt::Display for PluginError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PluginError::NotFound(path) => {
                write!(f, "plugin not found: {}", path.display())
            }
            PluginError::LoadFailed(msg) => write!(f, "failed to load plugin: {}", msg),
            PluginError::IncompatibleVersion { expected, found } => {
                write!(
                    f,
                    "incompatible plugin version: expected {}, found {}",
                    format_version(*expected),
                    format_version(*found)
                )
            }
            PluginError::MissingSymbol(sym) => {
                write!(f, "missing required symbol: {}", sym)
            }
            PluginError::InvalidData(msg) => write!(f, "invalid plugin data: {}", msg),
            PluginError::Io(e) => write!(f, "I/O error: {}", e),
        }
    }
}

impl std::error::Error for PluginError {}

impl From<std::io::Error> for PluginError {
    fn from(e: std::io::Error) -> Self {
        PluginError::Io(e)
    }
}

/// Information about a waveform provided by a plugin.
#[derive(Debug, Clone)]
pub struct PluginWaveformInfo {
    /// Plugin that provides this waveform
    pub plugin_name: String,
    /// Waveform ID for creation
    pub id: String,
    /// Human-readable name
    pub name: String,
    /// Description
    pub description: String,
    /// Minimum sample rate
    pub min_sample_rate: f64,
    /// Maximum sample rate
    pub max_sample_rate: f64,
    /// Capability flags
    pub capabilities: u32,
}

/// Information about a loaded plugin.
pub struct LoadedPlugin {
    /// Plugin file path
    pub path: PathBuf,
    /// Plugin name
    pub name: String,
    /// Plugin version
    pub version: String,
    /// Plugin description
    pub description: String,
    /// Plugin author
    pub author: String,
    /// Number of waveforms
    pub waveform_count: u32,
    /// API version
    pub api_version: u32,
    /// Waveforms provided by this plugin
    pub waveforms: Vec<PluginWaveformInfo>,
    /// Library handle (when plugins feature is enabled)
    #[cfg(feature = "plugins")]
    _library: Option<Library>,
}

impl std::fmt::Debug for LoadedPlugin {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LoadedPlugin")
            .field("path", &self.path)
            .field("name", &self.name)
            .field("version", &self.version)
            .field("description", &self.description)
            .field("author", &self.author)
            .field("waveform_count", &self.waveform_count)
            .field("api_version", &self.api_version)
            .field("waveforms", &self.waveforms)
            .finish()
    }
}

/// Manages discovery and loading of waveform plugins.
///
/// # Example
///
/// ```rust
/// use r4w_core::plugin::PluginManager;
///
/// let mut manager = PluginManager::new();
///
/// // Add search paths
/// manager.add_search_path("/usr/lib/r4w/plugins");
/// manager.add_search_path("/opt/r4w/plugins");
///
/// // Discover plugins (won't find any in tests)
/// // manager.discover_plugins()?;
///
/// // List all waveforms
/// for wf in manager.list_waveforms() {
///     println!("{}: {}", wf.name, wf.description);
/// }
/// ```
pub struct PluginManager {
    /// Directories to search for plugins
    search_paths: Vec<PathBuf>,
    /// Loaded plugins by name
    plugins: HashMap<String, LoadedPlugin>,
    /// Waveform index: id -> (plugin_name, info)
    waveform_index: HashMap<String, PluginWaveformInfo>,
}

impl Default for PluginManager {
    fn default() -> Self {
        Self::new()
    }
}

impl PluginManager {
    /// Create a new plugin manager.
    pub fn new() -> Self {
        Self {
            search_paths: Vec::new(),
            plugins: HashMap::new(),
            waveform_index: HashMap::new(),
        }
    }

    /// Add a directory to search for plugins.
    pub fn add_search_path<P: AsRef<Path>>(&mut self, path: P) {
        let path = path.as_ref().to_path_buf();
        if !self.search_paths.contains(&path) {
            self.search_paths.push(path);
        }
    }

    /// Get all search paths.
    pub fn search_paths(&self) -> &[PathBuf] {
        &self.search_paths
    }

    /// Discover and load all plugins from search paths.
    ///
    /// Returns the number of plugins successfully loaded.
    pub fn discover_plugins(&mut self) -> Result<usize, PluginError> {
        let mut count = 0;

        for path in self.search_paths.clone() {
            if !path.exists() {
                tracing::debug!("Plugin search path does not exist: {}", path.display());
                continue;
            }

            if !path.is_dir() {
                continue;
            }

            let entries = std::fs::read_dir(&path)?;

            for entry in entries.flatten() {
                let file_path = entry.path();

                // Check for shared library extension
                if !is_shared_library(&file_path) {
                    continue;
                }

                match self.load_plugin(&file_path) {
                    Ok(_) => {
                        tracing::info!("Loaded plugin: {}", file_path.display());
                        count += 1;
                    }
                    Err(e) => {
                        tracing::warn!(
                            "Failed to load plugin {}: {}",
                            file_path.display(),
                            e
                        );
                    }
                }
            }
        }

        Ok(count)
    }

    /// Load a specific plugin from a path.
    ///
    /// When the `plugins` feature is enabled, this uses `libloading` to actually
    /// load the shared library and call its exported functions. Otherwise, it
    /// creates a stub entry for testing.
    #[cfg(feature = "plugins")]
    pub fn load_plugin(&mut self, path: &Path) -> Result<&LoadedPlugin, PluginError> {
        if !path.exists() {
            return Err(PluginError::NotFound(path.to_path_buf()));
        }

        // Load the library
        let lib = unsafe { Library::new(path) }
            .map_err(|e| PluginError::LoadFailed(e.to_string()))?;

        // Get and check API version
        let api_version_fn: Symbol<ApiVersionFn> = unsafe {
            lib.get(PLUGIN_SYMBOL_API_VERSION.as_bytes())
        }.map_err(|_| PluginError::MissingSymbol(PLUGIN_SYMBOL_API_VERSION.to_string()))?;

        let plugin_api_version = unsafe { api_version_fn() };
        if !versions_compatible(PLUGIN_API_VERSION, plugin_api_version) {
            return Err(PluginError::IncompatibleVersion {
                expected: PLUGIN_API_VERSION,
                found: plugin_api_version,
            });
        }

        // Get plugin info
        let plugin_info_fn: Symbol<PluginInfoFn> = unsafe {
            lib.get(PLUGIN_SYMBOL_INFO.as_bytes())
        }.map_err(|_| PluginError::MissingSymbol(PLUGIN_SYMBOL_INFO.to_string()))?;

        let info_ptr = unsafe { plugin_info_fn() };
        if info_ptr.is_null() {
            return Err(PluginError::InvalidData("plugin info returned null".to_string()));
        }

        let info = unsafe { &*info_ptr };
        let plugin_name = unsafe { c_str_to_string(info.name) }
            .ok_or_else(|| PluginError::InvalidData("invalid plugin name".to_string()))?;
        let plugin_version = unsafe { c_str_to_string(info.version) }
            .unwrap_or_else(|| "0.0.0".to_string());
        let description = unsafe { c_str_to_string(info.description) }
            .unwrap_or_default();
        let author = unsafe { c_str_to_string(info.author) }
            .unwrap_or_else(|| "Unknown".to_string());

        // Get waveform list if available
        let mut waveforms = Vec::new();
        if let Ok(list_fn) = unsafe {
            lib.get::<ListWaveformsFn>(PLUGIN_SYMBOL_LIST_WAVEFORMS.as_bytes())
        } {
            let mut count: u32 = 0;
            let descriptors = unsafe { list_fn(&mut count) };
            if !descriptors.is_null() && count > 0 {
                for i in 0..count as usize {
                    let desc = unsafe { &*descriptors.add(i) };
                    if let (Some(id), Some(name)) = (
                        unsafe { c_str_to_string(desc.id) },
                        unsafe { c_str_to_string(desc.name) },
                    ) {
                        waveforms.push(PluginWaveformInfo {
                            plugin_name: plugin_name.clone(),
                            id: id.clone(),
                            name,
                            description: unsafe { c_str_to_string(desc.description) }
                                .unwrap_or_default(),
                            min_sample_rate: desc.min_sample_rate,
                            max_sample_rate: desc.max_sample_rate,
                            capabilities: desc.capabilities,
                        });

                        // Also add to waveform index
                        if let Some(wf) = waveforms.last() {
                            self.waveform_index.insert(id, wf.clone());
                        }
                    }
                }
            }
        }

        let plugin = LoadedPlugin {
            path: path.to_path_buf(),
            name: plugin_name.clone(),
            version: plugin_version,
            description,
            author,
            waveform_count: info.waveform_count,
            api_version: plugin_api_version,
            waveforms,
            _library: Some(lib),
        };

        self.plugins.insert(plugin_name.clone(), plugin);
        Ok(self.plugins.get(&plugin_name).unwrap())
    }

    /// Load a specific plugin from a path (stub version without libloading).
    #[cfg(not(feature = "plugins"))]
    pub fn load_plugin(&mut self, path: &Path) -> Result<&LoadedPlugin, PluginError> {
        if !path.exists() {
            return Err(PluginError::NotFound(path.to_path_buf()));
        }

        // Create a stub plugin entry for testing without libloading
        let plugin_name = path
            .file_stem()
            .and_then(|s| s.to_str())
            .map(|s| s.strip_prefix("lib").unwrap_or(s))
            .map(|s| s.strip_suffix("_plugin").unwrap_or(s))
            .unwrap_or("unknown")
            .to_string();

        let plugin = LoadedPlugin {
            path: path.to_path_buf(),
            name: plugin_name.clone(),
            version: "0.0.0".to_string(),
            description: format!("Plugin loaded from {}", path.display()),
            author: "Unknown".to_string(),
            waveform_count: 0,
            api_version: PLUGIN_API_VERSION,
            waveforms: Vec::new(),
        };

        self.plugins.insert(plugin_name.clone(), plugin);
        Ok(self.plugins.get(&plugin_name).unwrap())
    }

    /// Unload a plugin by name.
    pub fn unload_plugin(&mut self, name: &str) -> bool {
        if let Some(plugin) = self.plugins.remove(name) {
            // Remove waveforms from index
            for wf in &plugin.waveforms {
                self.waveform_index.remove(&wf.id);
            }
            true
        } else {
            false
        }
    }

    /// Get a loaded plugin by name.
    pub fn get_plugin(&self, name: &str) -> Option<&LoadedPlugin> {
        self.plugins.get(name)
    }

    /// List all loaded plugins.
    pub fn list_plugins(&self) -> Vec<&LoadedPlugin> {
        self.plugins.values().collect()
    }

    /// List all available waveforms from all plugins.
    pub fn list_waveforms(&self) -> Vec<&PluginWaveformInfo> {
        self.waveform_index.values().collect()
    }

    /// Find a waveform by ID.
    pub fn find_waveform(&self, id: &str) -> Option<&PluginWaveformInfo> {
        self.waveform_index.get(id)
    }

    /// Check if a waveform is available.
    pub fn has_waveform(&self, id: &str) -> bool {
        self.waveform_index.contains_key(id)
    }

    /// Get the number of loaded plugins.
    pub fn plugin_count(&self) -> usize {
        self.plugins.len()
    }

    /// Get the total number of waveforms available.
    pub fn waveform_count(&self) -> usize {
        self.waveform_index.len()
    }

    /// Register a waveform manually (useful for built-in waveforms).
    pub fn register_waveform(&mut self, info: PluginWaveformInfo) {
        self.waveform_index.insert(info.id.clone(), info);
    }

    /// Clear all loaded plugins.
    pub fn clear(&mut self) {
        self.plugins.clear();
        self.waveform_index.clear();
    }
}

/// Check if a path has a shared library extension.
fn is_shared_library(path: &Path) -> bool {
    let extension = path.extension().and_then(OsStr::to_str);

    match extension {
        Some("so") => true,        // Linux
        Some("dll") => true,       // Windows
        Some("dylib") => true,     // macOS
        _ => false,
    }
}

/// Helper to convert C string pointer to Rust String.
///
/// # Safety
///
/// The pointer must be a valid null-terminated C string.
#[allow(dead_code)]
unsafe fn c_str_to_string(ptr: *const std::ffi::c_char) -> Option<String> {
    if ptr.is_null() {
        return None;
    }
    CStr::from_ptr(ptr).to_str().ok().map(|s| s.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plugin_manager_new() {
        let manager = PluginManager::new();
        assert_eq!(manager.plugin_count(), 0);
        assert_eq!(manager.waveform_count(), 0);
    }

    #[test]
    fn test_add_search_path() {
        let mut manager = PluginManager::new();
        manager.add_search_path("/usr/lib/r4w");
        manager.add_search_path("/opt/r4w");
        manager.add_search_path("/usr/lib/r4w"); // Duplicate

        assert_eq!(manager.search_paths().len(), 2);
    }

    #[test]
    fn test_register_waveform() {
        let mut manager = PluginManager::new();

        let info = PluginWaveformInfo {
            plugin_name: "builtin".to_string(),
            id: "test_wf".to_string(),
            name: "Test Waveform".to_string(),
            description: "A test waveform".to_string(),
            min_sample_rate: 1000.0,
            max_sample_rate: 10_000_000.0,
            capabilities: 0x03,
        };

        manager.register_waveform(info);

        assert!(manager.has_waveform("test_wf"));
        assert!(!manager.has_waveform("nonexistent"));

        let wf = manager.find_waveform("test_wf").unwrap();
        assert_eq!(wf.name, "Test Waveform");
    }

    #[test]
    fn test_is_shared_library() {
        assert!(is_shared_library(Path::new("/usr/lib/libfoo.so")));
        assert!(is_shared_library(Path::new("C:\\Windows\\foo.dll")));
        assert!(is_shared_library(Path::new("/usr/lib/libfoo.dylib")));
        assert!(!is_shared_library(Path::new("/usr/bin/foo")));
        assert!(!is_shared_library(Path::new("/home/user/foo.txt")));
    }

    #[test]
    fn test_unload_plugin() {
        let mut manager = PluginManager::new();

        // Create a temporary file to simulate a plugin
        let temp_dir = std::env::temp_dir();
        let plugin_path = temp_dir.join("libtest_plugin.so");
        std::fs::write(&plugin_path, b"dummy").unwrap();

        // Load and unload
        manager.load_plugin(&plugin_path).ok();
        assert!(manager.get_plugin("test").is_some());

        manager.unload_plugin("test");
        assert!(manager.get_plugin("test").is_none());

        // Cleanup
        std::fs::remove_file(plugin_path).ok();
    }

    #[test]
    fn test_list_waveforms() {
        let mut manager = PluginManager::new();

        manager.register_waveform(PluginWaveformInfo {
            plugin_name: "p1".to_string(),
            id: "wf1".to_string(),
            name: "Waveform 1".to_string(),
            description: "First".to_string(),
            min_sample_rate: 1000.0,
            max_sample_rate: 1_000_000.0,
            capabilities: 0x01,
        });

        manager.register_waveform(PluginWaveformInfo {
            plugin_name: "p2".to_string(),
            id: "wf2".to_string(),
            name: "Waveform 2".to_string(),
            description: "Second".to_string(),
            min_sample_rate: 2000.0,
            max_sample_rate: 2_000_000.0,
            capabilities: 0x02,
        });

        let waveforms = manager.list_waveforms();
        assert_eq!(waveforms.len(), 2);
    }
}

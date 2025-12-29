//! # Configuration System
//!
//! Provides YAML-based configuration for R4W applications, including:
//!
//! - Device configuration (driver, sample rate, frequency, gain)
//! - Buffer settings (ring sizes, DMA buffers, overflow policies)
//! - Real-time settings (priorities, CPU affinity, memory locking)
//! - Logging and metrics configuration
//! - Hardware profiles for different SDR devices
//!
//! ## Configuration Search Path
//!
//! Configuration is loaded from the first file found:
//! 1. Path specified via `R4W_CONFIG` environment variable
//! 2. `./r4w.yaml` (current directory)
//! 3. `~/.config/r4w/config.yaml` (user config)
//! 4. `/etc/r4w/config.yaml` (system config)
//!
//! ## Example Configuration
//!
//! ```yaml
//! device:
//!   driver: "soapysdr"
//!   sample_rate: 2.4e6
//!   center_frequency: 915e6
//!   rx_gain: 40.0
//!
//! buffers:
//!   rx_ring_size: 1048576
//!   overflow_policy: "drop"
//!
//! realtime:
//!   enable: true
//!   rx_priority: 80
//!   lock_memory: true
//! ```

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Error type for configuration operations.
#[derive(Debug, Clone)]
pub enum ConfigError {
    /// Configuration file not found
    NotFound(String),
    /// Failed to read configuration file
    ReadError(String),
    /// Failed to parse configuration
    ParseError(String),
    /// Invalid configuration value
    ValidationError(String),
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigError::NotFound(msg) => write!(f, "config not found: {}", msg),
            ConfigError::ReadError(msg) => write!(f, "failed to read config: {}", msg),
            ConfigError::ParseError(msg) => write!(f, "failed to parse config: {}", msg),
            ConfigError::ValidationError(msg) => write!(f, "invalid config: {}", msg),
        }
    }
}

impl std::error::Error for ConfigError {}

/// SDR device configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct DeviceConfig {
    /// Driver name (simulator, soapysdr, uhd, rtlsdr, file)
    pub driver: String,
    /// Driver-specific arguments
    pub args: String,
    /// Sample rate in Hz
    pub sample_rate: f64,
    /// Center frequency in Hz
    pub center_frequency: f64,
    /// RX gain in dB
    pub rx_gain: f64,
    /// TX gain in dB
    pub tx_gain: f64,
    /// Antenna port name
    pub antenna: String,
    /// Clock source (internal, external, gpsdo)
    pub clock_source: String,
    /// Time source (internal, external, gpsdo)
    pub time_source: String,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self {
            driver: "simulator".to_string(),
            args: String::new(),
            sample_rate: 1_000_000.0,
            center_frequency: 915_000_000.0,
            rx_gain: 30.0,
            tx_gain: 30.0,
            antenna: "RX".to_string(),
            clock_source: "internal".to_string(),
            time_source: "internal".to_string(),
        }
    }
}

/// Buffer overflow policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum OverflowPolicy {
    /// Drop samples on overflow
    Drop,
    /// Block producer until space available
    Block,
    /// Log warning but continue
    Warn,
}

impl Default for OverflowPolicy {
    fn default() -> Self {
        OverflowPolicy::Drop
    }
}

/// Buffer configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct BufferConfig {
    /// RX ring buffer size in samples
    pub rx_ring_size: usize,
    /// TX ring buffer size in samples
    pub tx_ring_size: usize,
    /// Number of DMA buffers
    pub dma_buffer_count: usize,
    /// Size of each DMA buffer in samples
    pub dma_buffer_size: usize,
    /// Overflow handling policy
    pub overflow_policy: OverflowPolicy,
}

impl Default for BufferConfig {
    fn default() -> Self {
        Self {
            rx_ring_size: 1_048_576,  // 1M samples
            tx_ring_size: 262_144,    // 256K samples
            dma_buffer_count: 8,
            dma_buffer_size: 65_536,  // 64K samples
            overflow_policy: OverflowPolicy::Drop,
        }
    }
}

/// CPU affinity configuration.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
#[serde(default)]
pub struct CpuAffinityConfig {
    /// CPUs for RX thread
    pub rx: Vec<usize>,
    /// CPUs for TX thread
    pub tx: Vec<usize>,
    /// CPUs for DSP threads
    pub dsp: Vec<usize>,
}

/// Real-time configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct RealtimeConfig {
    /// Enable real-time features
    pub enable: bool,
    /// RX thread priority (0-99 on Linux)
    pub rx_priority: i32,
    /// TX thread priority (0-99 on Linux)
    pub tx_priority: i32,
    /// DSP thread priority (0-99 on Linux)
    pub dsp_priority: i32,
    /// Lock memory to prevent page faults
    pub lock_memory: bool,
    /// CPU affinity settings
    pub cpu_affinity: CpuAffinityConfig,
}

impl Default for RealtimeConfig {
    fn default() -> Self {
        Self {
            enable: false,
            rx_priority: 80,
            tx_priority: 80,
            dsp_priority: 70,
            lock_memory: false,
            cpu_affinity: CpuAffinityConfig::default(),
        }
    }
}

/// Log format.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LogFormat {
    /// Compact human-readable format
    Compact,
    /// Full format with all fields
    Full,
    /// JSON structured logging
    Json,
}

impl Default for LogFormat {
    fn default() -> Self {
        LogFormat::Compact
    }
}

/// Logging configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct LogConfig {
    /// Log level (trace, debug, info, warn, error)
    pub level: String,
    /// Log format
    pub format: LogFormat,
    /// Log file path (empty = stderr only)
    pub file: String,
    /// Log rotation size (e.g., "100MB")
    pub rotate_size: String,
    /// Number of rotated files to keep
    pub rotate_keep: usize,
}

impl Default for LogConfig {
    fn default() -> Self {
        Self {
            level: "info".to_string(),
            format: LogFormat::Compact,
            file: String::new(),
            rotate_size: "100MB".to_string(),
            rotate_keep: 5,
        }
    }
}

/// Metrics (Prometheus) configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct MetricsConfig {
    /// Enable metrics endpoint
    pub enable: bool,
    /// Bind address (e.g., "0.0.0.0:9090")
    pub bind: String,
    /// Metrics path (e.g., "/metrics")
    pub path: String,
}

impl Default for MetricsConfig {
    fn default() -> Self {
        Self {
            enable: false,
            bind: "127.0.0.1:9090".to_string(),
            path: "/metrics".to_string(),
        }
    }
}

/// Tracing (OpenTelemetry) configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct TracingConfig {
    /// Enable distributed tracing
    pub enable: bool,
    /// Exporter type (otlp, jaeger, zipkin)
    pub exporter: String,
    /// Exporter endpoint
    pub endpoint: String,
}

impl Default for TracingConfig {
    fn default() -> Self {
        Self {
            enable: false,
            exporter: "otlp".to_string(),
            endpoint: "http://localhost:4317".to_string(),
        }
    }
}

/// Capture format.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CaptureFormat {
    /// SigMF metadata format
    SigMF,
    /// Raw IQ samples
    Raw,
}

impl Default for CaptureFormat {
    fn default() -> Self {
        CaptureFormat::SigMF
    }
}

/// Packet capture configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct CaptureConfig {
    /// Enable packet capture
    pub enable: bool,
    /// Capture format
    pub format: CaptureFormat,
    /// Capture directory
    pub path: String,
    /// File rotation size (e.g., "1GB")
    pub rotate_size: String,
}

impl Default for CaptureConfig {
    fn default() -> Self {
        Self {
            enable: false,
            format: CaptureFormat::SigMF,
            path: "/var/capture/r4w".to_string(),
            rotate_size: "1GB".to_string(),
        }
    }
}

/// Waveform plugin configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct WaveformConfig {
    /// Plugin search directories
    pub plugin_dirs: Vec<String>,
    /// Waveforms to load on startup
    pub autoload: Vec<String>,
}

impl Default for WaveformConfig {
    fn default() -> Self {
        Self {
            plugin_dirs: vec![
                "/usr/lib/r4w/waveforms".to_string(),
                "~/.local/lib/r4w/waveforms".to_string(),
            ],
            autoload: Vec::new(),
        }
    }
}

/// Complete R4W configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct R4wConfig {
    /// Configuration version
    pub version: String,
    /// Device configuration
    pub device: DeviceConfig,
    /// Buffer configuration
    pub buffers: BufferConfig,
    /// Real-time configuration
    pub realtime: RealtimeConfig,
    /// Logging configuration
    pub logging: LogConfig,
    /// Metrics configuration
    pub metrics: MetricsConfig,
    /// Tracing configuration
    pub tracing: TracingConfig,
    /// Capture configuration
    pub capture: CaptureConfig,
    /// Waveform configuration
    pub waveforms: WaveformConfig,
    /// Hardware profiles (name -> config)
    pub profiles: HashMap<String, DeviceConfig>,
}

impl Default for R4wConfig {
    fn default() -> Self {
        Self {
            version: "1.0".to_string(),
            device: DeviceConfig::default(),
            buffers: BufferConfig::default(),
            realtime: RealtimeConfig::default(),
            logging: LogConfig::default(),
            metrics: MetricsConfig::default(),
            tracing: TracingConfig::default(),
            capture: CaptureConfig::default(),
            waveforms: WaveformConfig::default(),
            profiles: HashMap::new(),
        }
    }
}

impl R4wConfig {
    /// Load configuration from the default search path.
    ///
    /// Search order:
    /// 1. `R4W_CONFIG` environment variable
    /// 2. `./r4w.yaml`
    /// 3. `~/.config/r4w/config.yaml`
    /// 4. `/etc/r4w/config.yaml`
    ///
    /// Returns default config if no file is found.
    pub fn load() -> Result<Self, ConfigError> {
        // Check environment variable first
        if let Ok(path) = std::env::var("R4W_CONFIG") {
            if Path::new(&path).exists() {
                return Self::load_from(Path::new(&path));
            }
        }

        // Check standard paths
        let paths = Self::config_search_paths();
        for path in &paths {
            if path.exists() {
                return Self::load_from(path);
            }
        }

        // No config found, return defaults
        Ok(Self::default())
    }

    /// Load configuration from a specific file.
    pub fn load_from(path: &Path) -> Result<Self, ConfigError> {
        let content = std::fs::read_to_string(path)
            .map_err(|e| ConfigError::ReadError(format!("{}: {}", path.display(), e)))?;

        Self::parse(&content)
    }

    /// Parse configuration from a YAML string.
    pub fn parse(yaml: &str) -> Result<Self, ConfigError> {
        serde_yaml::from_str(yaml)
            .map_err(|e| ConfigError::ParseError(e.to_string()))
    }

    /// Save configuration to a file.
    pub fn save(&self, path: &Path) -> Result<(), ConfigError> {
        let content = serde_yaml::to_string(self)
            .map_err(|e| ConfigError::ParseError(e.to_string()))?;

        std::fs::write(path, content)
            .map_err(|e| ConfigError::ReadError(format!("{}: {}", path.display(), e)))
    }

    /// Apply a hardware profile by name.
    ///
    /// Merges the profile's device config with the current device config.
    pub fn with_profile(&self, name: &str) -> Result<Self, ConfigError> {
        let profile = self
            .profiles
            .get(name)
            .ok_or_else(|| ConfigError::NotFound(format!("profile '{}' not found", name)))?;

        let mut config = self.clone();
        config.device = profile.clone();
        Ok(config)
    }

    /// Get configuration search paths.
    pub fn config_search_paths() -> Vec<PathBuf> {
        let mut paths = vec![PathBuf::from("./r4w.yaml")];

        // User config directory
        if let Some(config_dir) = directories::ProjectDirs::from("", "", "r4w") {
            paths.push(config_dir.config_dir().join("config.yaml"));
        }

        // System config
        paths.push(PathBuf::from("/etc/r4w/config.yaml"));

        paths
    }

    /// Validate the configuration.
    pub fn validate(&self) -> Result<(), ConfigError> {
        // Validate sample rate
        if self.device.sample_rate <= 0.0 {
            return Err(ConfigError::ValidationError(
                "sample_rate must be positive".to_string(),
            ));
        }

        // Validate buffer sizes
        if self.buffers.rx_ring_size == 0 {
            return Err(ConfigError::ValidationError(
                "rx_ring_size must be > 0".to_string(),
            ));
        }

        // Validate priorities
        if self.realtime.enable {
            if !(0..=99).contains(&self.realtime.rx_priority) {
                return Err(ConfigError::ValidationError(
                    "rx_priority must be 0-99".to_string(),
                ));
            }
        }

        Ok(())
    }

    /// Generate example configuration YAML.
    pub fn example_yaml() -> String {
        let config = Self {
            profiles: {
                let mut profiles = HashMap::new();
                profiles.insert(
                    "rtlsdr_v3".to_string(),
                    DeviceConfig {
                        driver: "rtlsdr".to_string(),
                        sample_rate: 2_400_000.0,
                        rx_gain: 40.0,
                        ..Default::default()
                    },
                );
                profiles.insert(
                    "usrp_b200".to_string(),
                    DeviceConfig {
                        driver: "uhd".to_string(),
                        args: "type=b200".to_string(),
                        sample_rate: 10_000_000.0,
                        clock_source: "gpsdo".to_string(),
                        ..Default::default()
                    },
                );
                profiles
            },
            ..Default::default()
        };

        serde_yaml::to_string(&config).unwrap_or_default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = R4wConfig::default();
        assert_eq!(config.device.driver, "simulator");
        assert_eq!(config.buffers.rx_ring_size, 1_048_576);
        assert!(!config.realtime.enable);
    }

    #[test]
    fn test_parse_yaml() {
        let yaml = r#"
device:
  driver: "rtlsdr"
  sample_rate: 2.4e6
  rx_gain: 35.0

buffers:
  rx_ring_size: 2097152
  overflow_policy: block

realtime:
  enable: true
  rx_priority: 90
"#;

        let config = R4wConfig::parse(yaml).unwrap();
        assert_eq!(config.device.driver, "rtlsdr");
        assert_eq!(config.device.sample_rate, 2_400_000.0);
        assert_eq!(config.device.rx_gain, 35.0);
        assert_eq!(config.buffers.rx_ring_size, 2_097_152);
        assert_eq!(config.buffers.overflow_policy, OverflowPolicy::Block);
        assert!(config.realtime.enable);
        assert_eq!(config.realtime.rx_priority, 90);
    }

    #[test]
    fn test_parse_partial_yaml() {
        let yaml = r#"
device:
  driver: "file"
"#;

        let config = R4wConfig::parse(yaml).unwrap();
        assert_eq!(config.device.driver, "file");
        // Defaults should be applied
        assert_eq!(config.device.sample_rate, 1_000_000.0);
        assert_eq!(config.buffers.rx_ring_size, 1_048_576);
    }

    #[test]
    fn test_profiles() {
        let yaml = r#"
device:
  driver: "simulator"

profiles:
  hackrf:
    driver: "soapysdr"
    args: "driver=hackrf"
    sample_rate: 8e6
"#;

        let config = R4wConfig::parse(yaml).unwrap();
        let hackrf_config = config.with_profile("hackrf").unwrap();
        assert_eq!(hackrf_config.device.driver, "soapysdr");
        assert_eq!(hackrf_config.device.sample_rate, 8_000_000.0);
    }

    #[test]
    fn test_validation() {
        let mut config = R4wConfig::default();
        assert!(config.validate().is_ok());

        config.device.sample_rate = -1.0;
        assert!(config.validate().is_err());

        config.device.sample_rate = 1e6;
        config.buffers.rx_ring_size = 0;
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_example_yaml() {
        let yaml = R4wConfig::example_yaml();
        assert!(yaml.contains("device:"));
        assert!(yaml.contains("buffers:"));
        // Should be valid YAML
        let parsed = R4wConfig::parse(&yaml);
        assert!(parsed.is_ok());
    }

    #[test]
    fn test_serialize_deserialize() {
        let config = R4wConfig::default();
        let yaml = serde_yaml::to_string(&config).unwrap();
        let parsed: R4wConfig = serde_yaml::from_str(&yaml).unwrap();
        assert_eq!(config.device.driver, parsed.device.driver);
        assert_eq!(config.buffers.rx_ring_size, parsed.buffers.rx_ring_size);
    }

    #[test]
    fn test_overflow_policy_serde() {
        let yaml = "overflow_policy: block";
        let policy: OverflowPolicy = serde_yaml::from_str(yaml.split(": ").nth(1).unwrap()).unwrap();
        assert_eq!(policy, OverflowPolicy::Block);
    }

    #[test]
    fn test_config_search_paths() {
        let paths = R4wConfig::config_search_paths();
        assert!(!paths.is_empty());
        assert!(paths[0].ends_with("r4w.yaml"));
    }
}

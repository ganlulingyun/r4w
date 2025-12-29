//! # Real-Time Thread Spawning
//!
//! Provides utilities for spawning threads with real-time priorities,
//! CPU affinity, and memory locking.
//!
//! ## Platform Support
//!
//! - **Linux**: Full support for SCHED_FIFO, CPU affinity, mlockall
//! - **macOS**: Limited support (no SCHED_FIFO, limited affinity)
//! - **Windows**: Limited support (thread priority classes)
//!
//! ## Example
//!
//! ```rust,no_run
//! use r4w_core::rt::{RtConfig, RtPriority, spawn_rt_thread};
//!
//! let config = RtConfig::builder()
//!     .name("rx_thread")
//!     .priority(RtPriority::High)
//!     .cpu_affinity(&[0])
//!     .lock_memory(true)
//!     .build();
//!
//! let handle = spawn_rt_thread(config, || {
//!     // Real-time processing loop
//!     loop {
//!         // Process samples...
//!         break; // Example only
//!     }
//! }).unwrap();
//!
//! handle.join().unwrap();
//! ```

use std::thread::{self, JoinHandle};

/// Error type for real-time thread operations.
#[derive(Debug, Clone)]
pub enum RtError {
    /// Failed to spawn thread
    SpawnFailed(String),
    /// Failed to set thread priority
    PriorityFailed(String),
    /// Failed to set CPU affinity
    AffinityFailed(String),
    /// Failed to lock memory
    MlockFailed(String),
    /// Operation not supported on this platform
    NotSupported(String),
}

impl std::fmt::Display for RtError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RtError::SpawnFailed(msg) => write!(f, "failed to spawn thread: {}", msg),
            RtError::PriorityFailed(msg) => write!(f, "failed to set priority: {}", msg),
            RtError::AffinityFailed(msg) => write!(f, "failed to set CPU affinity: {}", msg),
            RtError::MlockFailed(msg) => write!(f, "failed to lock memory: {}", msg),
            RtError::NotSupported(msg) => write!(f, "operation not supported: {}", msg),
        }
    }
}

impl std::error::Error for RtError {}

/// Real-time priority levels.
///
/// These map to platform-specific priority values:
/// - Linux: SCHED_FIFO priorities 1-99
/// - macOS: Thread priority hints
/// - Windows: Thread priority classes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtPriority {
    /// Normal priority (no real-time scheduling)
    Normal,
    /// Low real-time priority (SCHED_FIFO 1-33 on Linux)
    Low,
    /// Medium real-time priority (SCHED_FIFO 34-66 on Linux)
    Medium,
    /// High real-time priority (SCHED_FIFO 67-90 on Linux)
    High,
    /// Critical real-time priority (SCHED_FIFO 91-99 on Linux)
    Critical,
    /// Custom priority value (platform-specific)
    Custom(i32),
}

impl RtPriority {
    /// Convert to Linux SCHED_FIFO priority (1-99).
    #[cfg(target_os = "linux")]
    pub fn to_linux_priority(self) -> i32 {
        match self {
            RtPriority::Normal => 0,
            RtPriority::Low => 20,
            RtPriority::Medium => 50,
            RtPriority::High => 80,
            RtPriority::Critical => 95,
            RtPriority::Custom(p) => p.clamp(1, 99),
        }
    }

    /// Check if this is a real-time priority.
    pub fn is_realtime(&self) -> bool {
        !matches!(self, RtPriority::Normal)
    }
}

impl Default for RtPriority {
    fn default() -> Self {
        RtPriority::Normal
    }
}

/// Configuration for spawning a real-time thread.
#[derive(Debug, Clone)]
pub struct RtConfig {
    /// Thread name (for debugging)
    pub name: String,
    /// Thread priority
    pub priority: RtPriority,
    /// CPU cores to pin the thread to (empty = no affinity)
    pub cpu_affinity: Vec<usize>,
    /// Stack size in bytes (0 = default)
    pub stack_size: usize,
    /// Whether to lock memory (mlockall)
    pub lock_memory: bool,
}

impl RtConfig {
    /// Create a new builder for RtConfig.
    pub fn builder() -> RtConfigBuilder {
        RtConfigBuilder::new()
    }

    /// Create a default configuration.
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            priority: RtPriority::Normal,
            cpu_affinity: Vec::new(),
            stack_size: 0,
            lock_memory: false,
        }
    }
}

impl Default for RtConfig {
    fn default() -> Self {
        Self::new("rt_thread")
    }
}

/// Builder for RtConfig.
pub struct RtConfigBuilder {
    config: RtConfig,
}

impl RtConfigBuilder {
    /// Create a new builder with defaults.
    pub fn new() -> Self {
        Self {
            config: RtConfig::default(),
        }
    }

    /// Set the thread name.
    pub fn name(mut self, name: &str) -> Self {
        self.config.name = name.to_string();
        self
    }

    /// Set the thread priority.
    pub fn priority(mut self, priority: RtPriority) -> Self {
        self.config.priority = priority;
        self
    }

    /// Set CPU affinity (pin to specific cores).
    pub fn cpu_affinity(mut self, cpus: &[usize]) -> Self {
        self.config.cpu_affinity = cpus.to_vec();
        self
    }

    /// Set stack size in bytes.
    pub fn stack_size(mut self, size: usize) -> Self {
        self.config.stack_size = size;
        self
    }

    /// Enable/disable memory locking.
    pub fn lock_memory(mut self, lock: bool) -> Self {
        self.config.lock_memory = lock;
        self
    }

    /// Build the configuration.
    pub fn build(self) -> RtConfig {
        self.config
    }
}

impl Default for RtConfigBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Spawn a thread with real-time configuration.
///
/// # Arguments
///
/// * `config` - Real-time configuration
/// * `f` - Function to run in the thread
///
/// # Errors
///
/// Returns `RtError` if thread creation or configuration fails.
///
/// # Notes
///
/// - Real-time priority requires elevated privileges on Linux (CAP_SYS_NICE or root)
/// - Memory locking requires elevated privileges
/// - The function will proceed with degraded capabilities if privileges are insufficient
pub fn spawn_rt_thread<F, T>(config: RtConfig, f: F) -> Result<JoinHandle<T>, RtError>
where
    F: FnOnce() -> T + Send + 'static,
    T: Send + 'static,
{
    let mut builder = thread::Builder::new().name(config.name.clone());

    if config.stack_size > 0 {
        builder = builder.stack_size(config.stack_size);
    }

    let priority = config.priority;
    let affinity = config.cpu_affinity.clone();
    let lock_mem = config.lock_memory;

    builder
        .spawn(move || {
            // Apply real-time settings within the thread
            apply_rt_settings(priority, &affinity, lock_mem);
            f()
        })
        .map_err(|e| RtError::SpawnFailed(e.to_string()))
}

/// Apply real-time settings to the current thread.
fn apply_rt_settings(priority: RtPriority, affinity: &[usize], lock_memory: bool) {
    // Set priority
    if priority.is_realtime() {
        if let Err(e) = set_thread_priority(priority) {
            tracing::warn!("Failed to set thread priority: {}", e);
        }
    }

    // Set CPU affinity
    if !affinity.is_empty() {
        if let Err(e) = set_cpu_affinity(affinity) {
            tracing::warn!("Failed to set CPU affinity: {}", e);
        }
    }

    // Lock memory
    if lock_memory {
        if let Err(e) = lock_all_memory() {
            tracing::warn!("Failed to lock memory: {}", e);
        }
    }
}

/// Set thread priority (platform-specific).
#[cfg(target_os = "linux")]
fn set_thread_priority(priority: RtPriority) -> Result<(), RtError> {
    use std::mem;

    if !priority.is_realtime() {
        return Ok(());
    }

    let linux_priority = priority.to_linux_priority();

    unsafe {
        let mut param: libc::sched_param = mem::zeroed();
        param.sched_priority = linux_priority;

        let result = libc::sched_setscheduler(0, libc::SCHED_FIFO, &param);
        if result != 0 {
            return Err(RtError::PriorityFailed(format!(
                "sched_setscheduler failed: {}",
                std::io::Error::last_os_error()
            )));
        }
    }

    Ok(())
}

#[cfg(not(target_os = "linux"))]
fn set_thread_priority(_priority: RtPriority) -> Result<(), RtError> {
    // On non-Linux platforms, just log and continue
    tracing::debug!("Real-time priority not supported on this platform");
    Ok(())
}

/// Set CPU affinity (platform-specific).
#[cfg(target_os = "linux")]
fn set_cpu_affinity(cpus: &[usize]) -> Result<(), RtError> {
    use std::mem;

    if cpus.is_empty() {
        return Ok(());
    }

    unsafe {
        let mut set: libc::cpu_set_t = mem::zeroed();
        libc::CPU_ZERO(&mut set);

        for &cpu in cpus {
            if cpu < libc::CPU_SETSIZE as usize {
                libc::CPU_SET(cpu, &mut set);
            }
        }

        let result = libc::sched_setaffinity(0, mem::size_of::<libc::cpu_set_t>(), &set);
        if result != 0 {
            return Err(RtError::AffinityFailed(format!(
                "sched_setaffinity failed: {}",
                std::io::Error::last_os_error()
            )));
        }
    }

    Ok(())
}

#[cfg(not(target_os = "linux"))]
fn set_cpu_affinity(_cpus: &[usize]) -> Result<(), RtError> {
    tracing::debug!("CPU affinity not supported on this platform");
    Ok(())
}

/// Lock all current and future memory pages.
#[cfg(target_os = "linux")]
fn lock_all_memory() -> Result<(), RtError> {
    unsafe {
        let result = libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE);
        if result != 0 {
            return Err(RtError::MlockFailed(format!(
                "mlockall failed: {}",
                std::io::Error::last_os_error()
            )));
        }
    }
    Ok(())
}

#[cfg(not(target_os = "linux"))]
fn lock_all_memory() -> Result<(), RtError> {
    tracing::debug!("Memory locking not supported on this platform");
    Ok(())
}

/// Get the number of available CPU cores.
#[allow(dead_code)]
pub fn available_cpus() -> usize {
    std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1)
}

/// Suggested CPU allocation for SDR workloads.
///
/// Returns (rx_cpus, tx_cpus, dsp_cpus) based on available cores.
#[allow(dead_code)]
pub fn suggest_cpu_allocation() -> (Vec<usize>, Vec<usize>, Vec<usize>) {
    let n = available_cpus();

    if n >= 8 {
        // Plenty of cores: dedicated RX, TX, multiple DSP
        (vec![0], vec![1], (2..n).collect())
    } else if n >= 4 {
        // Moderate: dedicated RX/TX, shared DSP
        (vec![0], vec![1], vec![2, 3])
    } else if n >= 2 {
        // Limited: share RX/DSP, separate TX
        (vec![0], vec![1], vec![0])
    } else {
        // Single core: everything shared
        (vec![0], vec![0], vec![0])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_priority_is_realtime() {
        assert!(!RtPriority::Normal.is_realtime());
        assert!(RtPriority::Low.is_realtime());
        assert!(RtPriority::Medium.is_realtime());
        assert!(RtPriority::High.is_realtime());
        assert!(RtPriority::Critical.is_realtime());
        assert!(RtPriority::Custom(50).is_realtime());
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn test_priority_to_linux() {
        assert_eq!(RtPriority::Normal.to_linux_priority(), 0);
        assert_eq!(RtPriority::Low.to_linux_priority(), 20);
        assert_eq!(RtPriority::Medium.to_linux_priority(), 50);
        assert_eq!(RtPriority::High.to_linux_priority(), 80);
        assert_eq!(RtPriority::Critical.to_linux_priority(), 95);
        assert_eq!(RtPriority::Custom(150).to_linux_priority(), 99);
        assert_eq!(RtPriority::Custom(-10).to_linux_priority(), 1);
    }

    #[test]
    fn test_config_builder() {
        let config = RtConfig::builder()
            .name("test_thread")
            .priority(RtPriority::High)
            .cpu_affinity(&[0, 1])
            .stack_size(1024 * 1024)
            .lock_memory(true)
            .build();

        assert_eq!(config.name, "test_thread");
        assert_eq!(config.priority, RtPriority::High);
        assert_eq!(config.cpu_affinity, vec![0, 1]);
        assert_eq!(config.stack_size, 1024 * 1024);
        assert!(config.lock_memory);
    }

    #[test]
    fn test_spawn_normal_thread() {
        let config = RtConfig::builder()
            .name("test_normal")
            .priority(RtPriority::Normal)
            .build();

        let handle = spawn_rt_thread(config, || 42).unwrap();
        let result = handle.join().unwrap();
        assert_eq!(result, 42);
    }

    #[test]
    fn test_available_cpus() {
        let n = available_cpus();
        assert!(n >= 1);
    }

    #[test]
    fn test_suggest_cpu_allocation() {
        let (rx, tx, dsp) = suggest_cpu_allocation();
        assert!(!rx.is_empty());
        assert!(!tx.is_empty());
        assert!(!dsp.is_empty());
    }
}

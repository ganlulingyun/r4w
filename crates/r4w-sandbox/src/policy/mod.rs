//! Security policy definitions for sandboxed waveforms.
//!
//! This module provides seccomp profiles and capability definitions
//! for configuring waveform isolation.

use serde::{Deserialize, Serialize};

/// Seccomp profile for syscall filtering
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum SeccompProfile {
    /// Allow all syscalls (no filtering)
    Permissive,

    /// DSP-optimized profile: allows memory, threading, timing
    ///
    /// Allows:
    /// - Basic I/O (read, write, close)
    /// - Memory management (mmap, munmap, mprotect, mlock)
    /// - Threading (clone, futex)
    /// - Timing (clock_gettime, nanosleep)
    /// - Signals (rt_sigaction, rt_sigprocmask)
    ///
    /// Denies:
    /// - Network syscalls
    /// - File creation
    /// - Process execution
    DSP,

    /// Strict profile: minimal syscalls only
    ///
    /// Only allows:
    /// - read, write, close
    /// - mmap (anonymous only)
    /// - exit, exit_group
    Strict,

    /// Custom profile from JSON path
    Custom(String),
}

impl Default for SeccompProfile {
    fn default() -> Self {
        Self::Permissive
    }
}

impl SeccompProfile {
    /// Get the syscalls allowed by this profile
    pub fn allowed_syscalls(&self) -> &[&'static str] {
        match self {
            Self::Permissive => &[], // Empty means all allowed
            Self::DSP => &[
                "read",
                "write",
                "close",
                "mmap",
                "munmap",
                "mprotect",
                "mlock",
                "munlock",
                "madvise",
                "brk",
                "clone",
                "clone3",
                "futex",
                "set_robust_list",
                "get_robust_list",
                "clock_gettime",
                "clock_nanosleep",
                "nanosleep",
                "rt_sigaction",
                "rt_sigprocmask",
                "rt_sigreturn",
                "sched_yield",
                "sched_getaffinity",
                "sched_setaffinity",
                "exit",
                "exit_group",
                "getrandom",
                "memfd_create",
            ],
            Self::Strict => &[
                "read",
                "write",
                "close",
                "mmap",
                "munmap",
                "brk",
                "exit",
                "exit_group",
            ],
            Self::Custom(_) => &[], // Custom profile loaded from file
        }
    }
}

/// Linux capabilities that may be retained in a sandbox
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Capability {
    /// CAP_SYS_NICE - Allow real-time scheduling priority
    SysNice,
    /// CAP_IPC_LOCK - Allow memory locking (mlock)
    IpcLock,
    /// CAP_NET_RAW - Allow raw socket access (for SDR drivers)
    NetRaw,
    /// CAP_NET_ADMIN - Allow network configuration
    NetAdmin,
    /// CAP_SYS_RAWIO - Allow raw I/O port access (for FPGA)
    SysRawio,
    /// CAP_DAC_OVERRIDE - Allow file permission override
    DacOverride,
    /// CAP_SETUID - Allow UID changes
    Setuid,
    /// CAP_SETGID - Allow GID changes
    Setgid,
}

impl Capability {
    /// Get the string name of this capability
    pub fn name(&self) -> &'static str {
        match self {
            Self::SysNice => "CAP_SYS_NICE",
            Self::IpcLock => "CAP_IPC_LOCK",
            Self::NetRaw => "CAP_NET_RAW",
            Self::NetAdmin => "CAP_NET_ADMIN",
            Self::SysRawio => "CAP_SYS_RAWIO",
            Self::DacOverride => "CAP_DAC_OVERRIDE",
            Self::Setuid => "CAP_SETUID",
            Self::Setgid => "CAP_SETGID",
        }
    }

    /// Convert to the caps crate capability type
    #[cfg(feature = "process")]
    pub fn to_caps_capability(&self) -> Option<caps::Capability> {
        use caps::Capability as CapsCap;
        Some(match self {
            Self::SysNice => CapsCap::CAP_SYS_NICE,
            Self::IpcLock => CapsCap::CAP_IPC_LOCK,
            Self::NetRaw => CapsCap::CAP_NET_RAW,
            Self::NetAdmin => CapsCap::CAP_NET_ADMIN,
            Self::SysRawio => CapsCap::CAP_SYS_RAWIO,
            Self::DacOverride => CapsCap::CAP_DAC_OVERRIDE,
            Self::Setuid => CapsCap::CAP_SETUID,
            Self::Setgid => CapsCap::CAP_SETGID,
        })
    }

    /// Capabilities recommended for DSP processing
    pub fn dsp_recommended() -> Vec<Self> {
        vec![Self::SysNice, Self::IpcLock]
    }

    /// Capabilities recommended for FPGA access
    pub fn fpga_recommended() -> Vec<Self> {
        vec![Self::SysNice, Self::IpcLock, Self::SysRawio]
    }

    /// Capabilities recommended for network SDR drivers
    pub fn sdr_recommended() -> Vec<Self> {
        vec![Self::SysNice, Self::IpcLock, Self::NetRaw]
    }
}

impl std::fmt::Display for Capability {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

/// Resource limits for a sandbox
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResourceLimits {
    /// Memory limit in bytes
    pub memory_bytes: Option<usize>,
    /// CPU shares (relative weight)
    pub cpu_shares: Option<u64>,
    /// CPU quota in microseconds per period
    pub cpu_quota_us: Option<u64>,
    /// CPU period in microseconds
    pub cpu_period_us: Option<u64>,
    /// Maximum number of processes
    pub max_pids: Option<u32>,
    /// Maximum open file descriptors
    pub max_fds: Option<u64>,
    /// Maximum locked memory (bytes)
    pub max_locked_memory: Option<usize>,
}

impl Default for ResourceLimits {
    fn default() -> Self {
        Self {
            memory_bytes: Some(512 * 1024 * 1024), // 512 MB
            cpu_shares: Some(1024),
            cpu_quota_us: None, // Unlimited
            cpu_period_us: Some(100_000), // 100ms
            max_pids: Some(100),
            max_fds: Some(1024),
            max_locked_memory: Some(64 * 1024 * 1024), // 64 MB
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_seccomp_profile_syscalls() {
        assert!(SeccompProfile::Permissive.allowed_syscalls().is_empty());
        assert!(!SeccompProfile::DSP.allowed_syscalls().is_empty());
        assert!(SeccompProfile::DSP
            .allowed_syscalls()
            .contains(&"mmap"));
        assert!(!SeccompProfile::Strict
            .allowed_syscalls()
            .contains(&"clone"));
    }

    #[test]
    fn test_capability_name() {
        assert_eq!(Capability::SysNice.name(), "CAP_SYS_NICE");
        assert_eq!(Capability::IpcLock.name(), "CAP_IPC_LOCK");
    }

    #[test]
    fn test_dsp_recommended_caps() {
        let caps = Capability::dsp_recommended();
        assert!(caps.contains(&Capability::SysNice));
        assert!(caps.contains(&Capability::IpcLock));
    }
}

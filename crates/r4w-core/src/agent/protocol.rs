//! Agent protocol definitions
//!
//! JSON-based command/response protocol for remote control.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Commands sent from controller to agent
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
pub enum AgentCommand {
    /// Get agent status and device info
    Status,

    /// Ping to check connectivity
    Ping,

    /// Start transmitting via UDP
    StartTx {
        /// Target address for UDP samples (host:port)
        target: String,
        /// Waveform type (BPSK, QPSK, etc.)
        waveform: String,
        /// Sample rate in Hz
        sample_rate: f64,
        /// Message to transmit
        message: String,
        /// SNR to add (-1 for none)
        #[serde(default = "default_snr")]
        snr: f64,
        /// Packets per second (0 = as fast as possible)
        #[serde(default = "default_pps")]
        pps: u32,
        /// Repeat message continuously
        #[serde(default)]
        repeat: bool,
    },

    /// Stop transmitting
    StopTx,

    /// Start receiving/benchmarking UDP samples
    StartRx {
        /// UDP port to listen on
        port: u16,
        /// Waveform type for demodulation
        waveform: String,
        /// Sample rate in Hz
        sample_rate: f64,
        /// Enable metrics reporting
        #[serde(default = "default_true")]
        report_metrics: bool,
        /// Metrics report interval in seconds
        #[serde(default = "default_metrics_interval")]
        metrics_interval: u64,
    },

    /// Stop receiving
    StopRx,

    /// Get current metrics
    GetMetrics,

    /// Configure agent settings
    Configure {
        /// Settings to update
        settings: HashMap<String, String>,
    },

    /// List available waveforms
    ListWaveforms,

    /// Shutdown the agent
    Shutdown,
}

fn default_snr() -> f64 {
    -1.0
}
fn default_pps() -> u32 {
    100
}
fn default_true() -> bool {
    true
}
fn default_metrics_interval() -> u64 {
    1
}

/// Response from agent to controller
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum AgentResponse {
    /// Command successful
    Ok {
        #[serde(skip_serializing_if = "Option::is_none")]
        message: Option<String>,
        #[serde(skip_serializing_if = "Option::is_none")]
        data: Option<serde_json::Value>,
    },

    /// Command failed
    Error {
        message: String,
        #[serde(skip_serializing_if = "Option::is_none")]
        code: Option<i32>,
    },

    /// Agent status response
    Status(AgentStatus),

    /// Pong response
    Pong {
        timestamp: u64,
    },

    /// Metrics data
    Metrics(MetricsData),

    /// List of available waveforms
    Waveforms {
        waveforms: Vec<WaveformInfo>,
    },
}

/// Agent status information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentStatus {
    /// Agent version
    pub version: String,
    /// Device information
    pub device: DeviceInfo,
    /// Current TX task status
    pub tx_task: TaskStatus,
    /// Current RX task status
    pub rx_task: TaskStatus,
    /// Uptime in seconds
    pub uptime_secs: u64,
}

/// Device information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceInfo {
    /// Hostname
    pub hostname: String,
    /// Operating system
    pub os: String,
    /// Architecture
    pub arch: String,
    /// CPU info
    pub cpu: String,
    /// Memory (MB)
    pub memory_mb: u64,
    /// IP addresses
    pub ip_addresses: Vec<String>,
}

impl DeviceInfo {
    /// Gather device information from the system
    pub fn gather() -> Self {
        let hostname = hostname::get()
            .map(|h| h.to_string_lossy().to_string())
            .unwrap_or_else(|_| "unknown".to_string());

        let os = std::env::consts::OS.to_string();
        let arch = std::env::consts::ARCH.to_string();

        // Get IP addresses (simplified)
        let ip_addresses = Self::get_ip_addresses();

        // Get memory info (Linux-specific)
        let memory_mb = Self::get_memory_mb();

        // Get CPU info
        let cpu = Self::get_cpu_info();

        Self {
            hostname,
            os,
            arch,
            cpu,
            memory_mb,
            ip_addresses,
        }
    }

    fn get_ip_addresses() -> Vec<String> {
        // Try to read from /proc on Linux
        if let Ok(output) = std::process::Command::new("hostname")
            .arg("-I")
            .output()
        {
            if output.status.success() {
                return String::from_utf8_lossy(&output.stdout)
                    .split_whitespace()
                    .map(|s| s.to_string())
                    .collect();
            }
        }
        vec![]
    }

    fn get_memory_mb() -> u64 {
        // Try to read from /proc/meminfo on Linux
        if let Ok(content) = std::fs::read_to_string("/proc/meminfo") {
            for line in content.lines() {
                if line.starts_with("MemTotal:") {
                    let parts: Vec<&str> = line.split_whitespace().collect();
                    if parts.len() >= 2 {
                        if let Ok(kb) = parts[1].parse::<u64>() {
                            return kb / 1024;
                        }
                    }
                }
            }
        }
        0
    }

    fn get_cpu_info() -> String {
        // Try to read from /proc/cpuinfo on Linux
        if let Ok(content) = std::fs::read_to_string("/proc/cpuinfo") {
            for line in content.lines() {
                if line.starts_with("model name") || line.starts_with("Model") {
                    if let Some(value) = line.split(':').nth(1) {
                        return value.trim().to_string();
                    }
                }
            }
            // For ARM, might be listed differently
            for line in content.lines() {
                if line.starts_with("Hardware") {
                    if let Some(value) = line.split(':').nth(1) {
                        return value.trim().to_string();
                    }
                }
            }
        }
        std::env::consts::ARCH.to_string()
    }
}

/// Task status
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "state", rename_all = "snake_case")]
pub enum TaskStatus {
    /// No task running
    Idle,
    /// Task is running
    Running {
        /// Task start time (Unix timestamp)
        started_at: u64,
        /// Task configuration summary
        config: String,
    },
    /// Task failed
    Failed {
        /// Error message
        error: String,
        /// When it failed (Unix timestamp)
        failed_at: u64,
    },
}

impl Default for TaskStatus {
    fn default() -> Self {
        Self::Idle
    }
}

/// Metrics data reported by agent
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MetricsData {
    /// Timestamp (Unix millis)
    pub timestamp: u64,
    /// Metrics type (tx or rx)
    pub kind: MetricsKind,
    /// Samples processed/sent
    pub samples: u64,
    /// Bytes transferred
    pub bytes: u64,
    /// Throughput (samples/sec)
    pub throughput_sps: f64,
    /// For RX: symbols detected
    #[serde(skip_serializing_if = "Option::is_none")]
    pub symbols_detected: Option<u64>,
    /// For RX: average latency (microseconds)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub avg_latency_us: Option<f64>,
    /// For RX: estimated BER
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ber_estimate: Option<f64>,
    /// For TX: packets sent
    #[serde(skip_serializing_if = "Option::is_none")]
    pub packets_sent: Option<u64>,
}

/// Type of metrics
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum MetricsKind {
    Tx,
    Rx,
}

/// Waveform information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WaveformInfo {
    pub name: String,
    pub full_name: String,
    pub bits_per_symbol: u8,
    pub carries_data: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_command_serialization() {
        let cmd = AgentCommand::StartTx {
            target: "192.168.1.100:5000".to_string(),
            waveform: "QPSK".to_string(),
            sample_rate: 48000.0,
            message: "Hello".to_string(),
            snr: 15.0,
            pps: 100,
            repeat: true,
        };

        let json = serde_json::to_string(&cmd).unwrap();
        assert!(json.contains("start_tx"));
        assert!(json.contains("QPSK"));

        let parsed: AgentCommand = serde_json::from_str(&json).unwrap();
        match parsed {
            AgentCommand::StartTx { waveform, .. } => {
                assert_eq!(waveform, "QPSK");
            }
            _ => panic!("Wrong command type"),
        }
    }

    #[test]
    fn test_response_serialization() {
        let resp = AgentResponse::Ok {
            message: Some("Started TX".to_string()),
            data: None,
        };

        let json = serde_json::to_string(&resp).unwrap();
        assert!(json.contains("ok"));
    }
}

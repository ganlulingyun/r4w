//! Remote SDR Agent
//!
//! This module provides a lightweight agent that runs on remote devices (e.g., Raspberry Pi)
//! and accepts commands over TCP to control SDR operations.
//!
//! ## Architecture
//!
//! ```text
//! GUI/CLI (Controller) ──TCP:6000──> Agent (Remote Device)
//!                                         │
//!                                         ▼
//!                                    r4w commands
//!                                    (udp-send, benchmark)
//! ```
//!
//! ## Protocol
//!
//! JSON-based commands over TCP with newline delimiters.

pub mod protocol;
pub mod server;
pub mod client;

pub use protocol::{AgentCommand, AgentResponse, AgentStatus, DeviceInfo, TaskStatus};
pub use server::AgentServer;
pub use client::AgentClient;

/// Default agent control port
pub const DEFAULT_AGENT_PORT: u16 = 6000;

/// Default metrics reporting port (UDP)
pub const DEFAULT_METRICS_PORT: u16 = 6001;

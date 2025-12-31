//! Meshtastic Protobuf Message Definitions
//!
//! This module defines the protobuf message types used by Meshtastic devices.
//! Messages are defined using prost derive macros for compatibility with the
//! Meshtastic protocol.
//!
//! ## Message Hierarchy
//!
//! ```text
//! MeshPacket (wire header + encrypted Data)
//!   └── Data (portnum + payload)
//!         ├── Position (GPS coordinates)
//!         ├── User (node info)
//!         ├── Telemetry (device/environment metrics)
//!         ├── Text (chat messages)
//!         └── ... other portnums
//! ```
//!
//! ## Usage
//!
//! ```ignore
//! use r4w_core::mesh::proto::{Data, PortNum, Position};
//!
//! // Encode a position message
//! let pos = Position {
//!     latitude_i: 374220000,  // 37.422° N
//!     longitude_i: -1220840000, // 122.084° W
//!     altitude: Some(10),
//!     ..Default::default()
//! };
//! let data = Data::position(pos);
//! let bytes = data.encode_to_vec();
//! ```

#[cfg(feature = "meshtastic-interop")]
mod messages;

#[cfg(feature = "meshtastic-interop")]
pub use messages::*;

// Re-export PortNum enum for all builds (useful for packet type identification)
mod portnum;
pub use portnum::PortNum;

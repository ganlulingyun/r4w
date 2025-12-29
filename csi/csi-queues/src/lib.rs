//! # CSI Queues
//!
//! Lock-free SPSC queues for the Crypto Service Interface boundary.
//!
//! **Status: Stub** - Architecture placeholder only.
//!
//! ## Planned Features
//!
//! - Plaintext input queue (RED → CSI)
//! - Ciphertext output queue (CSI → BLACK)
//! - Ciphertext input queue (BLACK → CSI)
//! - Plaintext output queue (CSI → RED)
//!
//! ## Design Constraints
//!
//! - `no_std` compatible
//! - Lock-free (bbqueue-based)
//! - Pre-allocated buffers
//! - Zero-copy where possible
//!
//! trace:FR-CSI-002 | ai:claude

#![cfg_attr(not(feature = "std"), no_std)]

pub use csi_core::CryptoResult;

// Placeholder for future implementation
// See docs/CRYPTO_BOUNDARY.md for queue architecture

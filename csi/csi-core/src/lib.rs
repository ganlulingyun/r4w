//! # CSI Core
//!
//! Core types and flow management for the Crypto Service Interface.
//!
//! **Status: Stub** - Architecture placeholder only.
//!
//! ## Planned Features
//!
//! - Message types: PlaintextIn, CiphertextOut, CiphertextIn, PlaintextOut
//! - Flow table with policy binding
//! - Replay protection with sliding window
//! - Control plane messages (CreateFlow, DestroyFlow, Rekey, Zeroize)
//!
//! ## Design Constraints
//!
//! - `no_std` compatible
//! - No heap allocation (heapless types)
//! - Compile-time size bounds
//!
//! trace:FR-CSI-001 | ai:claude

#![cfg_attr(not(feature = "std"), no_std)]

/// Crypto operation result codes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CryptoResult {
    /// Operation succeeded
    Ok = 0,
    /// Authentication failed (MAC mismatch)
    AuthFail = 1,
    /// Replay detected (sequence already seen)
    Replay = 2,
    /// Flow not found
    FlowNotFound = 3,
    /// Key not bound to flow
    KeyNotBound = 4,
    /// Policy violation
    PolicyViolation = 5,
    /// Queue full
    QueueFull = 6,
    /// Zeroized state (no operations allowed)
    Zeroized = 7,
}

/// Service type classification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ServiceType {
    /// Voice traffic (low latency)
    Voice = 0,
    /// Data traffic (reliable)
    Data = 1,
    /// Signaling (control plane)
    Signaling = 2,
}

// Placeholder for future implementation
// See docs/CRYPTO_BOUNDARY.md for full type definitions

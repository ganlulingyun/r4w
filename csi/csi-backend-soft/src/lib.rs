//! # CSI Software Backend
//!
//! Software AEAD implementation using ChaCha20-Poly1305.
//!
//! **Status: Stub** - Architecture placeholder only.
//!
//! ## Planned Features
//!
//! - ChaCha20-Poly1305 AEAD encryption/decryption
//! - Key derivation from key references
//! - Secure key storage with zeroization
//!
//! ## Design Constraints
//!
//! - `no_std` compatible
//! - RustCrypto implementations (audited, pure Rust)
//! - Constant-time operations
//!
//! trace:FR-CSI-003 | ai:claude

#![cfg_attr(not(feature = "std"), no_std)]

pub use csi_core::{CryptoResult, ServiceType};

// Placeholder for future implementation
// See docs/CRYPTO_BOUNDARY.md for backend interface

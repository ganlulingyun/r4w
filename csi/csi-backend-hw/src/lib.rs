//! # CSI Hardware Backend
//!
//! Hardware crypto acceleration for embedded targets.
//!
//! **Status: Stub** - Architecture placeholder only.
//!
//! ## Planned Features
//!
//! - STM32H7 CRYP peripheral (AES-GCM, DES)
//! - Zynq PS crypto accelerator
//! - Secure element integration (ATECC608, SE050)
//!
//! ## Design Constraints
//!
//! - `no_std` compatible
//! - HAL trait abstraction
//! - DMA support for high throughput
//!
//! trace:FR-CSI-004 | ai:claude

#![cfg_attr(not(feature = "std"), no_std)]

pub use csi_core::{CryptoResult, ServiceType};

// Placeholder for future implementation
// See docs/CRYPTO_BOUNDARY.md for hardware backend requirements

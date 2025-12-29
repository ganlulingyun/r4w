# Link-16 Porting Guide

Link-16 (TADIL-J) - NATO tactical data link for secure, jam-resistant sharing of situational awareness.

## Overview

| Property | Value |
|----------|-------|
| Frequency Range | 960-1215 MHz (L-band) |
| Number of Frequencies | 51 |
| Frequency Spacing | 3 MHz |
| Modulation | MSK (Minimum Shift Keying) |
| Access Method | TDMA (Time Division Multiple Access) |
| Frame Duration | 12.8 minutes (1536 time slots) |
| Message Format | J-series (J2.2-J28.2) |
| FEC | Reed-Solomon |

## Implementation Status

```
┌─────────────────────────────────────────────────────────────────┐
│ Link-16 Implementation Status                                   │
├─────────────────────────────────────────────────────────────────┤
│ [██████████████████████████████░░░░░░░░░░░░░░░░░░] 76%          │
├─────────────────────────────────────────────────────────────────┤
│ ✅ MSK Modulation                (10% effort) - Complete        │
│ ✅ TDMA Timing                   (8% effort)  - Complete        │
│ ✅ J-Series Message Codec        (12% effort) - Complete        │
│ ✅ Reed-Solomon FEC              (8% effort)  - 80% (basic)     │
│ ✅ Interleaver                   (5% effort)  - Complete        │
│ ✅ Pulse Formatter               (6% effort)  - Complete        │
│ ✅ Track Database                (8% effort)  - Complete        │
│ ✅ NPG Management                (6% effort)  - Complete        │
│ ✅ Framework                     (5% effort)  - Complete        │
│ ⚠️  HoppingPattern               (12% effort) - Simulator        │
│ ⚠️  TransecProvider              (10% effort) - Simulator        │
│ ⚠️  TimeSync (NTRU)              (5% effort)  - Partial (60%)    │
│ ⚠️  NetworkController            (5% effort)  - Partial (70%)    │
└─────────────────────────────────────────────────────────────────┘
```

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                       Link-16 Terminal                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    J-Series Messages                     │   │
│  │  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐         │   │
│  │  │  J2.2   │ │  J3.2   │ │  J7.0   │ │  J28.2  │ ...     │   │
│  │  │ Track   │ │Position │ │  PPLI   │ │  Free   │         │   │
│  │  └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘         │   │
│  └───────┼───────────┼───────────┼───────────┼──────────────┘   │
│          └───────────┴───────────┴───────────┘                  │
│                             │                                   │
│                    ┌────────▼────────┐                          │
│                    │  Message Codec   │                         │
│                    │  (Unclassified)  │                         │
│                    └────────┬─────────┘                         │
│                             │                                   │
│         ┌───────────────────┼───────────────────┐               │
│         ▼                   ▼                   ▼               │
│  ┌──────────────┐    ┌──────────────┐    ┌─────────────┐        │
│  │ Reed-Solomon │    │ Interleaver  │    │   TRANSEC   │        │
│  │     FEC      │    │              │    │  (COMSEC)   │        │
│  │(Unclassified)│    │(Unclassified)│    │ (CLASSIFIED)│        │
│  └──────┬───────┘    └──────┬───────┘    └──────┬──────┘        │
│         │                  │                   │                │
│         └──────────────────┴───────────────────┘                │
│                            │                                    │
│         ┌──────────────────┼──────────────────┐                 │
│         ▼                  ▼                  ▼                 │
│  ┌──────────────┐   ┌─────────────┐   ┌─────────────┐           │
│  │    Pulse     │   │   Hopping   │   │   Network   │           │
│  │  Formatter   │   │   Pattern   │   │  Controller │           │
│  │(Unclassified)│   │  (COMSEC)   │   │  (Partial)  │           │
│  └──────┬───────┘   └──────┬──────┘   └──────┬──────┘           │
│         │                 │                  │                  │
│         └─────────────────┴──────────────────┘                  │
│                           │                                     │
│                  ┌────────▼────────┐                            │
│                  │  MSK Modulator  │                            │
│                  │  (Unclassified) │                            │
│                  └────────┬────────┘                            │
│                           │                                     │
│                           ▼                                     │
│                    RF Output (51 frequencies)                   │
│                    TDMA Slot Timing                             │
└─────────────────────────────────────────────────────────────────┘
```

## Traits to Implement

### 1. HoppingPattern (12% of total effort)

The frequency hopping pattern generator. This is COMSEC (communications security) data.

```rust
// Location: crates/r4w-core/src/waveform/link16/traits.rs

pub trait HoppingPattern: Send + Sync {
    /// Initialize with crypto variable and net ID
    fn initialize(&mut self, crypto_var: &[u8], net_id: u16);

    /// Get frequency for a given time slot
    fn get_frequency(&self, slot: TimeSlot) -> Frequency;

    /// Get all frequencies for an epoch (8 slots)
    fn get_epoch_frequencies(&self, epoch: u8) -> Vec<Frequency>;

    /// Reset pattern generator
    fn reset(&mut self);

    /// Check if initialized
    fn is_initialized(&self) -> bool;

    /// Get pattern ID (for debugging/verification)
    fn pattern_id(&self) -> u32;
}
```

**Implementation Notes:**
- Link-16 uses 51 frequencies in the 960-1215 MHz band
- The hopping pattern is derived from cryptographic variables
- Each net has a unique pattern
- Pattern must avoid known interference frequencies (TACAN, etc.)

### 2. TransecProvider (10% of total effort)

TRANSEC encryption for Link-16 messages.

```rust
pub trait TransecProvider: Send + Sync {
    /// Load cryptographic key
    fn load_key(&mut self, key: &[u8]) -> Result<(), Link16Error>;

    /// Encrypt data for transmission
    fn encrypt(&self, data: &[u8], slot: TimeSlot) -> Vec<u8>;

    /// Decrypt received data
    fn decrypt(&self, data: &[u8], slot: TimeSlot) -> Result<Vec<u8>, Link16Error>;

    /// Check if key is loaded
    fn is_loaded(&self) -> bool;

    /// Get current crypto mode
    fn mode(&self) -> CryptoMode;

    /// Zeroize all key material
    fn zeroize(&mut self);

    /// Get key status
    fn key_status(&self) -> KeyStatus;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CryptoMode {
    Plain,      // No encryption (training mode)
    Exercise,   // Exercise keys
    Operational, // Operational keys
}

#[derive(Clone, Debug)]
pub struct KeyStatus {
    pub loaded: bool,
    pub mode: CryptoMode,
    pub key_id: Option<u32>,
    pub expires: Option<TimeSlot>,
}
```

### 3. TimeSync (5% of total effort)

Network Time Reference Unit synchronization.

```rust
pub trait TimeSync: Send + Sync {
    /// Get current network time
    fn get_network_time(&self) -> NetworkTime;

    /// Check if synchronized to network
    fn is_synchronized(&self) -> bool;

    /// Process received time reference
    fn process_time_reference(&mut self, reference: &TimeReference) -> Result<(), SyncError>;

    /// Get synchronization quality
    fn sync_quality(&self) -> SyncQuality;

    /// Get current time slot
    fn current_slot(&self) -> TimeSlot;

    /// Get time until next slot boundary
    fn time_to_slot_boundary(&self) -> Duration;
}

#[derive(Clone, Debug)]
pub struct NetworkTime {
    pub epoch: u8,
    pub frame: u16,
    pub slot: u16,
    pub sub_slot: f64,  // 0.0 - 1.0 within slot
}

impl NetworkTime {
    /// Convert to TimeSlot
    pub fn time_slot(&self) -> TimeSlot {
        TimeSlot {
            epoch: self.epoch,
            slot: self.slot,
        }
    }
}
```

### 4. NetworkController (5% of total effort)

Network participation controller.

```rust
pub trait NetworkController: Send + Sync {
    /// Join a Link-16 network
    fn join_network(&mut self, net_id: u16, terminal_id: u8) -> Result<(), Link16Error>;

    /// Leave current network
    fn leave_network(&mut self);

    /// Check if currently in a network
    fn is_in_network(&self) -> bool;

    /// Get current network ID
    fn current_net_id(&self) -> Option<u16>;

    /// Subscribe to a Participation Group
    fn subscribe_npg(&mut self, npg: Npg);

    /// Unsubscribe from a Participation Group
    fn unsubscribe_npg(&mut self, npg: Npg);

    /// Get subscribed NPGs
    fn subscribed_npgs(&self) -> Vec<Npg>;

    /// Check if a slot is assigned for transmission
    fn is_tx_slot(&self, slot: TimeSlot) -> bool;

    /// Get assigned TX slots
    fn assigned_tx_slots(&self) -> Vec<TimeSlot>;

    /// Set terminal mode
    fn set_terminal_mode(&mut self, mode: TerminalMode);

    /// Get current terminal mode
    fn terminal_mode(&self) -> TerminalMode;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TerminalMode {
    ReceiveOnly,  // Only receive
    Active,       // Full participation
    Relay,        // Relay messages
}
```

## J-Series Message Types

The J-series message codec is already implemented (unclassified). Key message types:

| Message | Function | Status |
|---------|----------|--------|
| J2.2 | Air Track | ✅ Implemented |
| J3.2 | Surface Track | ✅ Implemented |
| J7.0 | PPLI (Precise Position Location ID) | ✅ Implemented |
| J7.2 | Mission Assignment | ✅ Implemented |
| J12.0 | Mission Management | ✅ Implemented |
| J28.2 | Free Text | ✅ Implemented |

## Step-by-Step Porting Procedure

### Phase 1: Create Classified Crate

```bash
mkdir -p crates/link16-classified
cd crates/link16-classified
cargo init --lib
```

### Phase 2: Implement Hopping Pattern

```rust
// crates/link16-classified/src/hopper.rs
use r4w_core::waveform::link16::traits::*;
use r4w_core::waveform::link16::types::*;

pub struct ClassifiedHopper {
    crypto_var: Option<Vec<u8>>,
    net_id: Option<u16>,
    pattern_cache: Vec<Frequency>,
    initialized: bool,
}

impl ClassifiedHopper {
    pub fn new() -> Self {
        Self {
            crypto_var: None,
            net_id: None,
            pattern_cache: Vec::new(),
            initialized: false,
        }
    }

    /// Generate hopping pattern from crypto variable
    /// THIS IS COMSEC DATA
    fn generate_pattern(&mut self) {
        let crypto_var = self.crypto_var.as_ref().unwrap();
        let net_id = self.net_id.unwrap();

        // CLASSIFIED: Actual pattern generation algorithm
        // This produces a sequence of 51 frequency indices
        // that repeats in a pseudo-random order

        // Placeholder structure:
        self.pattern_cache = (0..51)
            .map(|i| {
                // Real algorithm uses crypto_var to permute frequencies
                let idx = (i ^ (crypto_var[i % crypto_var.len()] as usize)) % 51;
                Frequency(idx as u8)
            })
            .collect();

        self.initialized = true;
    }

    /// Get frequency for a specific slot
    fn compute_frequency(&self, slot: TimeSlot) -> Frequency {
        // CLASSIFIED: Map slot to pattern position
        // Real algorithm considers epoch, slot, and crypto state

        let pattern_idx = ((slot.epoch as usize * 192) + slot.slot as usize)
            % self.pattern_cache.len();
        self.pattern_cache[pattern_idx]
    }
}

impl HoppingPattern for ClassifiedHopper {
    fn initialize(&mut self, crypto_var: &[u8], net_id: u16) {
        self.crypto_var = Some(crypto_var.to_vec());
        self.net_id = Some(net_id);
        self.generate_pattern();
    }

    fn get_frequency(&self, slot: TimeSlot) -> Frequency {
        if !self.initialized {
            return Frequency(0);  // Default frequency
        }
        self.compute_frequency(slot)
    }

    fn get_epoch_frequencies(&self, epoch: u8) -> Vec<Frequency> {
        (0..192)  // 192 slots per epoch
            .map(|slot| {
                self.get_frequency(TimeSlot {
                    epoch,
                    slot: slot as u16,
                })
            })
            .collect()
    }

    fn reset(&mut self) {
        // Zeroize crypto material
        if let Some(ref mut cv) = self.crypto_var {
            for b in cv.iter_mut() {
                *b = 0;
            }
        }
        self.crypto_var = None;
        self.net_id = None;
        self.pattern_cache.clear();
        self.initialized = false;
    }

    fn is_initialized(&self) -> bool {
        self.initialized
    }

    fn pattern_id(&self) -> u32 {
        // Hash of pattern for verification
        self.pattern_cache
            .iter()
            .enumerate()
            .fold(0u32, |acc, (i, f)| acc ^ ((f.0 as u32) << (i % 24)))
    }
}
```

### Phase 3: Implement TRANSEC Provider

```rust
// crates/link16-classified/src/transec.rs
use r4w_core::waveform::link16::traits::*;
use r4w_core::waveform::link16::types::*;
use zeroize::Zeroize;

pub struct ClassifiedTransec {
    key: Option<TransecKey>,
    mode: CryptoMode,
    frame_counter: u64,
}

#[derive(Zeroize)]
#[zeroize(drop)]
struct TransecKey {
    key_bytes: [u8; 32],
    key_id: u32,
}

impl ClassifiedTransec {
    pub fn new() -> Self {
        Self {
            key: None,
            mode: CryptoMode::Plain,
            frame_counter: 0,
        }
    }

    /// Encrypt a single block
    /// CLASSIFIED ALGORITHM
    fn encrypt_block(&self, data: &[u8], slot: TimeSlot) -> Vec<u8> {
        let key = self.key.as_ref().unwrap();

        // CLASSIFIED: Actual encryption algorithm
        // Uses key, slot timing, and frame counter

        // Placeholder: XOR with key-derived stream
        data.iter()
            .enumerate()
            .map(|(i, &b)| b ^ key.key_bytes[i % 32] ^ (slot.slot as u8))
            .collect()
    }

    /// Decrypt a single block
    fn decrypt_block(&self, data: &[u8], slot: TimeSlot) -> Result<Vec<u8>, Link16Error> {
        // CLASSIFIED: Actual decryption algorithm
        // Symmetric to encrypt_block

        let key = self.key.as_ref().ok_or(Link16Error::NoKey)?;

        Ok(data
            .iter()
            .enumerate()
            .map(|(i, &b)| b ^ key.key_bytes[i % 32] ^ (slot.slot as u8))
            .collect())
    }
}

impl TransecProvider for ClassifiedTransec {
    fn load_key(&mut self, key: &[u8]) -> Result<(), Link16Error> {
        if key.len() < 32 {
            return Err(Link16Error::InvalidKey);
        }

        let mut key_bytes = [0u8; 32];
        key_bytes.copy_from_slice(&key[..32]);

        self.key = Some(TransecKey {
            key_bytes,
            key_id: u32::from_le_bytes([key[0], key[1], key[2], key[3]]),
        });

        self.mode = CryptoMode::Operational;
        Ok(())
    }

    fn encrypt(&self, data: &[u8], slot: TimeSlot) -> Vec<u8> {
        match self.mode {
            CryptoMode::Plain => data.to_vec(),
            _ => self.encrypt_block(data, slot),
        }
    }

    fn decrypt(&self, data: &[u8], slot: TimeSlot) -> Result<Vec<u8>, Link16Error> {
        match self.mode {
            CryptoMode::Plain => Ok(data.to_vec()),
            _ => self.decrypt_block(data, slot),
        }
    }

    fn is_loaded(&self) -> bool {
        self.key.is_some()
    }

    fn mode(&self) -> CryptoMode {
        self.mode
    }

    fn zeroize(&mut self) {
        self.key = None;  // Zeroize derive handles secure clearing
        self.mode = CryptoMode::Plain;
        self.frame_counter = 0;
    }

    fn key_status(&self) -> KeyStatus {
        KeyStatus {
            loaded: self.key.is_some(),
            mode: self.mode,
            key_id: self.key.as_ref().map(|k| k.key_id),
            expires: None,
        }
    }
}
```

### Phase 4: Create Full Integration

```rust
// crates/link16-classified/src/lib.rs
mod hopper;
mod transec;
mod time_sync;
mod net_controller;

pub use hopper::ClassifiedHopper;
pub use transec::ClassifiedTransec;
pub use time_sync::ClassifiedTimeSync;
pub use net_controller::ClassifiedNetController;

use r4w_core::waveform::link16::{Link16, Link16Builder};

/// Create operational Link-16 terminal with classified components
pub fn create_mids_terminal(
    sample_rate: f64,
    terminal_id: u8,
) -> Result<Link16, Box<dyn std::error::Error>> {
    Ok(Link16Builder::new()
        .sample_rate(sample_rate)
        .terminal_id(terminal_id)
        .hopper(Box::new(ClassifiedHopper::new()))
        .transec(Box::new(ClassifiedTransec::new()))
        .time_sync(Box::new(ClassifiedTimeSync::new()))
        .net_controller(Box::new(ClassifiedNetController::new()))
        .build()?)
}
```

## Bridging Existing C++ MIDS Implementation

If you have an existing C++ MIDS/JTIDS implementation:

```cpp
// mids_interface.hpp
class MidsHopper {
public:
    virtual ~MidsHopper() = default;
    virtual void initialize(const uint8_t* crypto_var, size_t len, uint16_t net_id) = 0;
    virtual uint8_t getFrequency(uint8_t epoch, uint16_t slot) = 0;
    virtual void reset() = 0;
};

// Factory function
extern "C" MidsHopper* create_mids_hopper();
extern "C" void destroy_mids_hopper(MidsHopper* hopper);
```

Using CXX for C++ interop:

```rust
// crates/link16-classified/src/hopper_cxx.rs
#[cxx::bridge]
mod ffi {
    unsafe extern "C++" {
        include!("mids_interface.hpp");

        type MidsHopper;

        fn create_mids_hopper() -> *mut MidsHopper;
        fn destroy_mids_hopper(hopper: *mut MidsHopper);

        unsafe fn initialize(
            self: Pin<&mut MidsHopper>,
            crypto_var: *const u8,
            len: usize,
            net_id: u16,
        );
        fn getFrequency(self: &MidsHopper, epoch: u8, slot: u16) -> u8;
        fn reset(self: Pin<&mut MidsHopper>);
    }
}

pub struct CxxHopper {
    inner: *mut ffi::MidsHopper,
}

unsafe impl Send for CxxHopper {}
unsafe impl Sync for CxxHopper {}

impl CxxHopper {
    pub fn new() -> Self {
        Self {
            inner: unsafe { ffi::create_mids_hopper() },
        }
    }
}

impl HoppingPattern for CxxHopper {
    fn initialize(&mut self, crypto_var: &[u8], net_id: u16) {
        unsafe {
            Pin::new_unchecked(&mut *self.inner)
                .initialize(crypto_var.as_ptr(), crypto_var.len(), net_id);
        }
    }

    fn get_frequency(&self, slot: TimeSlot) -> Frequency {
        let idx = unsafe { (*self.inner).getFrequency(slot.epoch, slot.slot) };
        Frequency(idx)
    }

    fn reset(&mut self) {
        unsafe {
            Pin::new_unchecked(&mut *self.inner).reset();
        }
    }

    // ... implement remaining methods
}

impl Drop for CxxHopper {
    fn drop(&mut self) {
        unsafe { ffi::destroy_mids_hopper(self.inner) };
    }
}
```

## Testing with Real Equipment

### MIDS Low Volume Terminal (MIDS-LVT) Testing

```rust
#[cfg(test)]
mod integration_tests {
    use super::*;

    #[test]
    #[ignore]  // Requires hardware
    fn test_mids_lvt_compatibility() {
        // Load test crypto from secure source
        let crypto_var = load_test_crypto();

        let mut terminal = create_mids_terminal(10_000_000.0, 1).unwrap();
        terminal.load_crypto(&crypto_var).unwrap();
        terminal.join_network(1, 1).unwrap();

        // Receive from MIDS-LVT
        let rf_samples = capture_rf_samples();
        let messages = terminal.demodulate(&rf_samples);

        // Verify we can decode messages from real terminal
        assert!(!messages.is_empty());
    }
}
```

### Loopback Testing

```rust
#[test]
fn test_tx_rx_loopback() {
    let crypto_var = [0u8; 32];  // Test key

    let mut tx_terminal = create_mids_terminal(10_000_000.0, 1).unwrap();
    let mut rx_terminal = create_mids_terminal(10_000_000.0, 2).unwrap();

    tx_terminal.load_crypto(&crypto_var).unwrap();
    rx_terminal.load_crypto(&crypto_var).unwrap();

    tx_terminal.join_network(1, 1).unwrap();
    rx_terminal.join_network(1, 2).unwrap();

    // Create J7.0 PPLI message
    let ppli = PpliMessage {
        latitude: 38.8977,
        longitude: -77.0365,
        altitude: 10000.0,
        // ...
    };

    let tx_samples = tx_terminal.transmit_ppli(&ppli);
    let rx_messages = rx_terminal.receive(&tx_samples);

    assert_eq!(rx_messages.len(), 1);
    // Verify PPLI content
}
```

## Cross-Compilation for MIDS Platforms

| Platform | Target | Toolchain |
|----------|--------|-----------|
| MIDS-LVT | PowerPC | Custom |
| MIDS JTRS | ARM Cortex-A | GNU |
| F-16 (simulation) | x86_64-linux | Standard |
| F-35 (simulation) | aarch64-linux | Standard |

## Security Checklist

- [ ] Crypto variables zeroized after use
- [ ] Pattern never logged or exposed
- [ ] TRANSEC keys managed by approved KMI
- [ ] Time sync hardened against spoofing
- [ ] NPG access controlled
- [ ] No classified data in core dumps
- [ ] Stack/heap protections enabled

## See Also

- [General Build Procedures](./BUILD_PROCEDURES.md)
- [Main Porting Guide](../PORTING_GUIDE_MILITARY.md)

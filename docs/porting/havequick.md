# HAVEQUICK Porting Guide

HAVEQUICK (HQ/HQ-II) - UHF Anti-Jam frequency hopping system for military aviation.

## Overview

| Property | Value |
|----------|-------|
| Frequency Range | 225-400 MHz (UHF Military) |
| Number of Frequencies | 3,584 (28 MHz spacing / 7,168 with extended) |
| Hop Rate | Varies by mode (tens to hundreds/sec) |
| Modulation | AM (voice), ASK (data) |
| Time Source | GPS or TOD (Time of Day) |
| Key Management | Word of Day (WOD) |

## Implementation Status

```
┌─────────────────────────────────────────────────────────────────┐
│ HAVEQUICK Implementation Status                                 │
├─────────────────────────────────────────────────────────────────┤
│ [██████████████████████░░░░░░░░░░░░░░░░░░░░░░░░░░] 58%          │
├─────────────────────────────────────────────────────────────────┤
│ ✅ AM Voice Modulation           (12% effort) - Complete        │
│ ✅ ASK Data Modulation           (10% effort) - Complete        │
│ ✅ Channel Frequency Math        (8% effort)  - Complete        │
│ ✅ WOD Parsing (structure)       (10% effort) - Complete        │
│ ✅ Framework                     (5% effort)  - Complete        │
│ ⚠️  HoppingAlgorithm             (22% effort) - Simulator        │
│ ⚠️  TimeSyncProtocol             (12% effort) - Partial          │
│ ⚠️  NetController                (10% effort) - Simulator        │
│ ⚠️  TransecProvider (KY-58)      (11% effort) - Simulator        │
└─────────────────────────────────────────────────────────────────┘
```

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     HAVEQUICK Waveform                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐              ┌──────────────┐                 │
│  │    Voice     │              │    Data      │                 │
│  │    (AM)      │              │   (ASK)      │                 │
│  └──────┬───────┘              └──────┬───────┘                 │
│         │                             │                         │
│         └─────────────┬───────────────┘                         │
│                       │                                         │
│              ┌────────▼────────┐                                │
│              │  Optional KY-58 │                                │
│              │    Encryption   │                                │
│              │   (CLASSIFIED)  │                                │
│              └────────┬────────┘                                │
│                       │                                         │
│         ┌─────────────┼─────────────┐                           │
│         ▼             ▼             ▼                           │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐                │
│  │   Hopping   │ │     Net     │ │    Time     │                │
│  │  Algorithm  │ │ Controller  │ │    Sync     │                │
│  │ (CLASSIFIED)│ │ (CLASSIFIED)│ │ (CLASSIFIED)│                │
│  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘                │
│         │               │               │                       │
│         └───────────────┴───────────────┘                       │
│                         │                                       │
│                ┌────────▼────────┐                              │
│                │  AM Modulator   │                              │
│                │  (Unclassified) │                              │
│                └────────┬────────┘                              │
│                         │                                       │
│                         ▼                                       │
│                    RF Output                                    │
│              (Frequency-Hopping)                                │
└─────────────────────────────────────────────────────────────────┘
```

## Traits to Implement

### 1. HoppingAlgorithm (22% of total effort)

The HAVEQUICK hopping algorithm based on Word of Day.

```rust
// Location: crates/r4w-core/src/waveform/havequick/traits.rs

pub trait HoppingAlgorithm: Send + Sync {
    /// Initialize with Word of Day and network parameters
    fn initialize(&mut self, wod: &WordOfDay, net_id: NetId, tod: TimeOfDay);

    /// Get current channel frequency
    fn get_current_channel(&self) -> ChannelNumber;

    /// Convert channel number to frequency in Hz
    fn channel_to_frequency(&self, channel: ChannelNumber) -> f64;

    /// Advance to next hop
    fn advance_hop(&mut self);

    /// Synchronize to time of day
    fn sync_to_time(&mut self, tod: TimeOfDay);

    /// Get current hop number within day
    fn current_hop_number(&self) -> u64;

    /// Predict channel for future hop
    fn predict_channel(&self, hop_number: u64) -> ChannelNumber;

    /// Check if hopper is valid
    fn is_valid(&self) -> bool;

    /// Reset hopper state
    fn reset(&mut self);
}
```

**Implementation Notes:**
- HAVEQUICK uses the Word of Day as a key to generate the hopping sequence
- The algorithm produces a pseudo-random channel sequence
- Time synchronization is critical - typically uses GPS
- Different nets can use different segments of the WOD

### 2. NetController (10% of total effort)

Manages network membership and WOD distribution.

```rust
pub trait NetController: Send + Sync {
    /// Load Word of Day for a network
    fn load_wod(&mut self, net_id: NetId, wod: WordOfDay) -> Result<(), HavequickError>;

    /// Get WOD for a network (if authorized)
    fn get_wod(&self, net_id: NetId) -> Option<&WordOfDay>;

    /// Check authorization for a network
    fn is_authorized(&self, net_id: NetId) -> bool;

    /// Get list of authorized networks
    fn get_authorized_nets(&self) -> Vec<NetId>;

    /// Remove all WODs (zeroization)
    fn zeroize(&mut self);

    /// Get net status
    fn get_net_status(&self, net_id: NetId) -> Option<NetStatus>;

    /// Set active network
    fn set_active_net(&mut self, net_id: NetId) -> Result<(), HavequickError>;
}

#[derive(Clone, Debug)]
pub struct NetStatus {
    pub net_id: NetId,
    pub wod_loaded: bool,
    pub synchronized: bool,
    pub last_activity: Option<TimeOfDay>,
}
```

### 3. TimeSyncProtocol (12% of total effort)

Time synchronization for coordinated hopping.

```rust
pub trait TimeSyncProtocol: Send + Sync {
    /// Get current Time of Day
    fn get_tod(&self) -> TimeOfDay;

    /// Check if GPS lock is available
    fn has_gps_lock(&self) -> bool;

    /// Get GPS-derived time (if available)
    fn get_gps_time(&self) -> Option<TimeOfDay>;

    /// Set manual time (for GPS-denied operation)
    fn set_manual_time(&mut self, tod: TimeOfDay);

    /// Process incoming sync pulse
    fn process_sync_pulse(&mut self, pulse: &SyncPulse) -> Result<(), SyncError>;

    /// Get time quality indicator
    fn time_quality(&self) -> TimeQuality;

    /// Check if synchronized
    fn is_synchronized(&self) -> bool;
}

#[derive(Clone, Debug)]
pub struct TimeQuality {
    pub source: TimeSource,
    pub accuracy_ns: u64,
    pub last_update: TimeOfDay,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TimeSource {
    Gps,
    Network,
    Manual,
    Unknown,
}
```

### 4. TransecProvider (11% of total effort)

For systems using KY-58 or similar voice encryption.

```rust
pub trait TransecProvider: Send + Sync {
    /// Load encryption key
    fn load_key(&mut self, key: &TransecKey) -> Result<(), TransecError>;

    /// Encrypt audio frame
    fn encrypt_audio(&self, audio: &[i16]) -> Result<Vec<u8>, TransecError>;

    /// Decrypt audio frame
    fn decrypt_audio(&self, encrypted: &[u8]) -> Result<Vec<i16>, TransecError>;

    /// Check if encryption is active
    fn is_active(&self) -> bool;

    /// Set encryption mode
    fn set_mode(&mut self, mode: TransecMode) -> Result<(), TransecError>;

    /// Zeroize all key material
    fn zeroize(&mut self);
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TransecMode {
    Plain,       // No encryption
    Cipher,      // Encrypted
    CipherText,  // Text-only encryption
}
```

**Note:** KY-58 is a legacy system. Modern implementations may use different COMSEC equipment.

## Word of Day Structure

The WOD is the key material for HAVEQUICK. The framework handles the structure, but the algorithm that uses it is classified.

```rust
// Already implemented in r4w-core (unclassified)
#[derive(Clone)]
pub struct WordOfDay {
    /// WOD identifier (Julian date typically)
    pub id: WodId,
    /// Net segments within the WOD
    pub segments: Vec<WodSegment>,
    /// Validity period
    pub valid_from: TimeOfDay,
    pub valid_until: TimeOfDay,
}

#[derive(Clone)]
pub struct WodSegment {
    pub segment_id: u8,
    pub data: [u8; 32],  // Segment data
}
```

## Step-by-Step Porting Procedure

### Phase 1: Environment Setup

```bash
# Create classified implementation crate
mkdir -p crates/havequick-classified
cd crates/havequick-classified
cargo init --lib
```

### Phase 2: Implement Hopping Algorithm

```rust
// crates/havequick-classified/src/hopper.rs
use r4w_core::waveform::havequick::traits::*;

pub struct ClassifiedHopper {
    wod: Option<WordOfDay>,
    net_id: Option<NetId>,
    current_hop: u64,
    day_start: TimeOfDay,
}

impl ClassifiedHopper {
    pub fn new() -> Self {
        Self {
            wod: None,
            net_id: None,
            current_hop: 0,
            day_start: TimeOfDay::midnight(),
        }
    }

    /// Compute channel from WOD and hop number
    /// THIS IS THE CLASSIFIED ALGORITHM
    fn compute_channel(&self, hop_number: u64) -> ChannelNumber {
        let wod = self.wod.as_ref().unwrap();
        let net_id = self.net_id.unwrap();

        // Get the segment for this net
        let segment = &wod.segments[net_id.0 as usize % wod.segments.len()];

        // CLASSIFIED: The actual algorithm that converts
        // (segment_data, hop_number) -> channel
        //
        // Placeholder structure:
        let seed = segment.data[0] as u64 ^ (hop_number << 8);
        ChannelNumber((seed % 3584) as u16)
    }
}

impl HoppingAlgorithm for ClassifiedHopper {
    fn initialize(&mut self, wod: &WordOfDay, net_id: NetId, tod: TimeOfDay) {
        self.wod = Some(wod.clone());
        self.net_id = Some(net_id);
        self.day_start = tod.day_start();
        self.current_hop = self.tod_to_hop(tod);
    }

    fn get_current_channel(&self) -> ChannelNumber {
        self.compute_channel(self.current_hop)
    }

    fn channel_to_frequency(&self, channel: ChannelNumber) -> f64 {
        // HAVEQUICK frequency calculation (unclassified)
        225_000_000.0 + (channel.0 as f64 * 25_000.0)
    }

    fn advance_hop(&mut self) {
        self.current_hop += 1;
    }

    fn sync_to_time(&mut self, tod: TimeOfDay) {
        self.current_hop = self.tod_to_hop(tod);
    }

    fn current_hop_number(&self) -> u64 {
        self.current_hop
    }

    fn predict_channel(&self, hop_number: u64) -> ChannelNumber {
        self.compute_channel(hop_number)
    }

    fn is_valid(&self) -> bool {
        self.wod.is_some() && self.net_id.is_some()
    }

    fn reset(&mut self) {
        self.wod = None;
        self.net_id = None;
        self.current_hop = 0;
    }
}

impl ClassifiedHopper {
    fn tod_to_hop(&self, tod: TimeOfDay) -> u64 {
        // Convert time of day to hop number
        // Based on hop rate (varies by mode)
        let secs_since_midnight = tod.secs_since(self.day_start);
        // Assuming 100 hops/second for this example
        (secs_since_midnight * 100.0) as u64
    }
}
```

### Phase 3: GPS Time Integration

```rust
// crates/havequick-classified/src/time_sync.rs
use r4w_core::waveform::havequick::traits::*;

pub struct GpsTimeSync {
    gps_receiver: Option<GpsReceiver>,
    manual_offset: Option<Duration>,
    last_gps_update: Option<Instant>,
}

impl GpsTimeSync {
    pub fn new() -> Self {
        Self {
            gps_receiver: None,
            manual_offset: None,
            last_gps_update: None,
        }
    }

    pub fn connect_gps(&mut self, device: &str) -> Result<(), GpsError> {
        self.gps_receiver = Some(GpsReceiver::connect(device)?);
        Ok(())
    }
}

impl TimeSyncProtocol for GpsTimeSync {
    fn get_tod(&self) -> TimeOfDay {
        if let Some(gps) = &self.gps_receiver {
            if let Some(time) = gps.get_time() {
                return time;
            }
        }

        // Fallback to system time + manual offset
        let system_time = SystemTime::now();
        if let Some(offset) = self.manual_offset {
            TimeOfDay::from_system(system_time) + offset
        } else {
            TimeOfDay::from_system(system_time)
        }
    }

    fn has_gps_lock(&self) -> bool {
        self.gps_receiver
            .as_ref()
            .map(|gps| gps.has_lock())
            .unwrap_or(false)
    }

    fn get_gps_time(&self) -> Option<TimeOfDay> {
        self.gps_receiver.as_ref()?.get_time()
    }

    fn set_manual_time(&mut self, tod: TimeOfDay) {
        let system_tod = TimeOfDay::from_system(SystemTime::now());
        self.manual_offset = Some(tod - system_tod);
    }

    fn process_sync_pulse(&mut self, pulse: &SyncPulse) -> Result<(), SyncError> {
        // Process network timing pulse
        // Used for GPS-denied synchronization
        Ok(())
    }

    fn time_quality(&self) -> TimeQuality {
        if self.has_gps_lock() {
            TimeQuality {
                source: TimeSource::Gps,
                accuracy_ns: 100,  // GPS typically ~100ns
                last_update: self.get_tod(),
            }
        } else {
            TimeQuality {
                source: TimeSource::Manual,
                accuracy_ns: 1_000_000,  // 1ms without GPS
                last_update: self.get_tod(),
            }
        }
    }

    fn is_synchronized(&self) -> bool {
        self.has_gps_lock() || self.manual_offset.is_some()
    }
}
```

### Phase 4: Integration with R4W

```rust
// crates/havequick-classified/src/lib.rs
mod hopper;
mod time_sync;
mod net_controller;
mod transec;

pub use hopper::ClassifiedHopper;
pub use time_sync::GpsTimeSync;
pub use net_controller::ClassifiedNetController;
pub use transec::Ky58Transec;

use r4w_core::waveform::havequick::HavequickBuilder;

/// Create operational HAVEQUICK with classified components
pub fn create_operational_havequick(
    sample_rate: f64,
    gps_device: Option<&str>,
) -> Result<r4w_core::waveform::havequick::Havequick, Box<dyn std::error::Error>> {
    let mut time_sync = GpsTimeSync::new();
    if let Some(device) = gps_device {
        time_sync.connect_gps(device)?;
    }

    Ok(HavequickBuilder::new()
        .sample_rate(sample_rate)
        .hopper(Box::new(ClassifiedHopper::new()))
        .time_sync(Box::new(time_sync))
        .net_controller(Box::new(ClassifiedNetController::new()))
        .transec(Box::new(Ky58Transec::new()))
        .build()?)
}
```

## Bridging Existing C HAVEQUICK Implementation

```c
// existing_havequick.h
typedef struct HQState HQState;

HQState* hq_create(void);
void hq_destroy(HQState* state);
void hq_load_wod(HQState* state, const uint8_t* wod, size_t wod_len);
void hq_set_net(HQState* state, uint8_t net_id);
void hq_set_tod(HQState* state, uint32_t secs, uint32_t usecs);
uint16_t hq_get_channel(HQState* state);
void hq_advance(HQState* state);
```

Rust FFI wrapper:

```rust
// crates/havequick-classified/src/hopper_ffi.rs
use std::ffi::c_void;

#[repr(C)]
struct HQState { _private: [u8; 0] }

extern "C" {
    fn hq_create() -> *mut HQState;
    fn hq_destroy(state: *mut HQState);
    fn hq_load_wod(state: *mut HQState, wod: *const u8, wod_len: usize);
    fn hq_set_net(state: *mut HQState, net_id: u8);
    fn hq_set_tod(state: *mut HQState, secs: u32, usecs: u32);
    fn hq_get_channel(state: *mut HQState) -> u16;
    fn hq_advance(state: *mut HQState);
}

pub struct FfiHopper {
    state: *mut HQState,
}

// Safety: C implementation must be thread-safe
unsafe impl Send for FfiHopper {}
unsafe impl Sync for FfiHopper {}

impl FfiHopper {
    pub fn new() -> Self {
        Self {
            state: unsafe { hq_create() }
        }
    }
}

impl HoppingAlgorithm for FfiHopper {
    fn initialize(&mut self, wod: &WordOfDay, net_id: NetId, tod: TimeOfDay) {
        let wod_bytes = wod.to_bytes();
        unsafe {
            hq_load_wod(self.state, wod_bytes.as_ptr(), wod_bytes.len());
            hq_set_net(self.state, net_id.0);
            hq_set_tod(self.state, tod.secs(), tod.usecs());
        }
    }

    fn get_current_channel(&self) -> ChannelNumber {
        ChannelNumber(unsafe { hq_get_channel(self.state) })
    }

    fn advance_hop(&mut self) {
        unsafe { hq_advance(self.state) };
    }

    // ... remaining trait methods
}

impl Drop for FfiHopper {
    fn drop(&mut self) {
        unsafe { hq_destroy(self.state) };
    }
}
```

## Cross-Compilation for Avionics Platforms

HAVEQUICK is primarily used in aviation, so target platforms include:

| Platform | Target | Notes |
|----------|--------|-------|
| VxWorks (common) | Custom | Requires VxWorks SDK |
| Integrity RTOS | Custom | Green Hills toolchain |
| LynxOS | `x86_64-unknown-lynxos` | POSIX-like |
| Linux (simulation) | `x86_64-unknown-linux-gnu` | Development |

### VxWorks Build Example

```makefile
# Makefile for VxWorks cross-compilation
WIND_BASE ?= /opt/WindRiver/vxworks-7
RUSTC_TARGET = armv7-wrs-vxworks-eabihf

.PHONY: all
all:
	CARGO_BUILD_TARGET=$(RUSTC_TARGET) \
	CC=$(WIND_BASE)/compilers/arm/bin/arm-wrs-vxworks-gcc \
	cargo build --release --target $(RUSTC_TARGET)
```

## Security Checklist

- [ ] WOD storage uses secure memory
- [ ] WOD automatically zeroized at expiration
- [ ] GPS spoofing detection implemented
- [ ] No classified data in logs
- [ ] Time sync validated against multiple sources
- [ ] Net authorization enforced
- [ ] KY-58 keys properly managed (if applicable)

## See Also

- [General Build Procedures](./BUILD_PROCEDURES.md)
- [Main Porting Guide](../PORTING_GUIDE_MILITARY.md)

# SINCGARS Porting Guide

Single Channel Ground and Airborne Radio System (SINCGARS) - VHF Frequency Hopping tactical radio.

## Overview

This guide describes how to port the SINCGARS waveform to the R4W framework. SINCGARS uses Frequency Hopping Spread Spectrum (FHSS) with certain classified elements that must be provided by authorized implementers.

The implementation separates **unclassified framework code** from **classified algorithms** using Rust traits.

| Property | Value |
|----------|-------|
| Frequency Range | 30-88 MHz (VHF) |
| Channel Spacing | 25 kHz |
| Number of Channels | 2,320 |
| Hop Rate | 100+ hops/second |
| Modulation | FM (voice), FSK (data) |
| Voice Codec | CVSD (16/32 kbps) |
| Data Rates | 75-16,000 bps |

## Implementation Status

```
┌─────────────────────────────────────────────────────────────────┐
│ SINCGARS Implementation Status                                  │
├─────────────────────────────────────────────────────────────────┤
│ [████████████████████████████░░░░░░░░░░░░░░░░░░░░] 59%          │
├─────────────────────────────────────────────────────────────────┤
│ ✅ FM Modulation/Demodulation    (15% effort) - Complete        │
│ ✅ CVSD Voice Codec              (12% effort) - Complete        │
│ ✅ FSK Data Modem                (10% effort) - Complete        │
│ ✅ Message Framing               (8% effort)  - Complete        │
│ ✅ Framework                     (5% effort)  - Complete        │
│ ⚠️  HoppingAlgorithm             (20% effort) - Simulator        │
│ ⚠️  TransecProvider              (12% effort) - Simulator        │
│ ⚠️  TimeSyncProtocol             (8% effort)  - Simulator        │
│ ⚠️  NetIdMapper                  (5% effort)  - Simulator        │
│ ⚠️  CryptoProvider               (5% effort)  - Simulator        │
└─────────────────────────────────────────────────────────────────┘
```

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     SINCGARS Waveform                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │    Voice     │    │    Data      │    │   Control    │       │
│  │   (CVSD)     │    │   (FSK)      │    │  Messages    │       │
│  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘       │
│         │                   │                   │               │
│         └───────────────────┴───────────────────┘               │
│                             │                                   │
│                    ┌────────▼────────┐                          │
│                    │  Frame Builder   │                         │
│                    │  (Unclassified)  │                         │
│                    └────────┬─────────┘                         │
│                             │                                   │
│         ┌───────────────────┼───────────────────┐               │
│         ▼                   ▼                   ▼               │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐          │
│  │   TRANSEC   │    │   Hopping   │    │    Time     │          │
│  │   Crypto    │    │  Algorithm  │    │    Sync     │          │
│  │ (CLASSIFIED)│    │ (CLASSIFIED)│    │ (CLASSIFIED)│          │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘          │
│         │                  │                   │                │
│         └──────────────────┴───────────────────┘                │
│                            │                                    │
│                   ┌────────▼────────┐                           │
│                   │  FM Modulator   │                           │
│                   │  (Unclassified) │                           │
│                   └────────┬────────┘                           │
│                            │                                    │
│                            ▼                                    │
│                      RF Output                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Traits to Implement

### 1. HoppingAlgorithm (20% of total effort)

The core TRANSEC hopping algorithm. This is the most complex classified component.

```rust
// Location: crates/r4w-core/src/waveform/sincgars/traits.rs

pub trait HoppingAlgorithm: Send + Sync {
    /// Initialize with TRANSEC key and network parameters
    fn initialize(&mut self, key: &TransecKey, net_id: NetId, time: SincgarsTime);

    /// Get current channel (0-2319)
    fn get_current_channel(&self) -> ChannelNumber;

    /// Advance to next hop
    fn advance_hop(&mut self);

    /// Synchronize to a specific time
    fn sync_to_time(&mut self, time: SincgarsTime);

    /// Convert absolute time to hop number
    fn time_to_hop_number(&self, time: SincgarsTime) -> u64;

    /// Check if hopper is initialized and valid
    fn is_valid(&self) -> bool;

    /// Reset to initial state
    fn reset(&mut self);

    /// Get current hop count
    fn get_hop_number(&self) -> u64;

    /// Look ahead at upcoming channels (for transmit scheduling)
    fn peek_channels(&self, count: usize) -> Vec<ChannelNumber>;
}
```

**Implementation Notes:**
- The hopping algorithm uses the TRANSEC key as a seed for a pseudo-random sequence
- Channel selection must match exactly with other radios on the network
- Time synchronization is critical - all radios must be within ±1ms
- Hop dwell time varies by mode (typically 5-10ms)

**Test Vectors:**
Your implementation should be validated against known test vectors from classified documentation.

### 2. TransecProvider (12% of total effort)

Manages TRANSEC key material.

```rust
pub trait TransecProvider: Send + Sync {
    /// Load a key from key ID
    fn load_key(&mut self, key_id: KeyId) -> Result<TransecKey, TransecError>;

    /// Securely zeroize all key material
    fn zeroize(&mut self);

    /// Check if a valid key is loaded
    fn has_valid_key(&self) -> bool;

    /// Get metadata about current key (key ID, expiration, etc.)
    fn get_key_metadata(&self) -> Option<KeyMetadata>;

    /// Derive session key for crypto operations
    fn derive_session_key(&self, context: &[u8]) -> Result<SessionKey, TransecError>;

    /// Get reference to current key (for hopping algorithm)
    fn get_current_key(&self) -> Option<&TransecKey>;
}
```

**Implementation Notes:**
- Keys are typically loaded from a fill device (KYK-13, AN/CYZ-10)
- Must support key rollover (multiple keys with activation times)
- Zeroization must be cryptographically complete
- Consider hardware security module integration for key storage

### 3. CryptoProvider (5% of total effort)

Encrypts/decrypts data frames.

```rust
pub trait CryptoProvider: Send + Sync {
    /// Encrypt plaintext
    fn encrypt(&self, plaintext: &[u8], context: &CryptoContext) -> Result<Vec<u8>, CryptoError>;

    /// Decrypt ciphertext
    fn decrypt(&self, ciphertext: &[u8], context: &CryptoContext) -> Result<Vec<u8>, CryptoError>;

    /// Generate sync vector for transmission
    fn generate_sync_vector(&self) -> SyncVector;

    /// Process received sync vector
    fn process_sync_vector(&mut self, sync: &SyncVector) -> Result<(), CryptoError>;

    /// Securely zeroize crypto state
    fn zeroize(&mut self);

    /// Check if crypto is ready for operation
    fn is_ready(&self) -> bool;

    /// Initialize with session key
    fn initialize(&mut self, key: &SessionKey) -> Result<(), CryptoError>;

    /// Get current frame counter (for replay protection)
    fn get_frame_counter(&self) -> u64;
}
```

### 4. TimeSyncProtocol (8% of total effort)

Network time synchronization for coordinated hopping.

```rust
pub trait TimeSyncProtocol: Send + Sync {
    /// Get current network time
    fn get_network_time(&self) -> SincgarsTime;

    /// Process received time sync message
    fn process_sync_message(&mut self, msg: &TimeSyncMessage) -> Result<(), SyncError>;

    /// Generate time sync message for transmission
    fn generate_sync_message(&self) -> TimeSyncMessage;

    /// Check if synchronized to network
    fn is_synchronized(&self) -> bool;

    /// Get synchronization quality (offset, jitter)
    fn sync_quality(&self) -> SyncQuality;

    /// Force resynchronization
    fn resync(&mut self);
}
```

**Implementation Notes:**
- SINCGARS uses GPS time when available
- Without GPS, uses network time synchronization protocol
- Synchronization must be maintained within ±1ms for reliable hopping

### 5. NetIdMapper (5% of total effort)

Maps network IDs to configurations.

```rust
pub trait NetIdMapper: Send + Sync {
    /// Get configuration for a network ID
    fn get_net_config(&self, net_id: NetId) -> Option<NetConfig>;

    /// Check if authorized for a network
    fn is_authorized(&self, net_id: NetId) -> bool;

    /// Get list of authorized networks
    fn authorized_networks(&self) -> Vec<NetId>;

    /// Load network configuration
    fn load_config(&mut self, net_id: NetId, config: NetConfig) -> Result<(), NetError>;
}
```

## Unclassified Framework Components

The following components are provided by R4W and are unclassified:

### RF Parameters (Public Knowledge)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Frequency Range | 30-87.975 MHz | VHF band |
| Channel Spacing | 25 kHz | Standard channelization |
| Number of Channels | 2320 | Across the band |
| Hop Rate | 100 hops/sec | Standard mode |
| Modulation | FM | ±6 kHz deviation |
| Data Rates | 16 kbps (voice), up to 16 kbps (data) | |

### Voice Codec

SINCGARS uses CVSD (Continuously Variable Slope Delta modulation) for voice:
- 16 kbps digitized voice
- Frame-based processing
- Provided in `sincgars/audio.rs`

### Data Modes

Multiple data modes are supported:
- Low-speed data (75-2400 bps)
- Medium-speed data (4800 bps)
- High-speed data (16 kbps)

Framing and error correction provided in `sincgars/data.rs`.

## Testing with Simulator

For development and testing, dummy implementations are provided:

```rust
use r4w_core::waveform::sincgars::simulator::{
    SimulatorHopper,
    SimulatorTransec,
    SimulatorNetMapper,
    SimulatorTimeSync,
    SimulatorCrypto,
};

// Creates a fully functional SINCGARS for testing
// Uses deterministic, non-secure algorithms
let test_sincgars = SincgarsBuilder::simulator().build().unwrap();
```

The simulator implementations:
- Use deterministic LFSR-based hopping (not secure, but functional)
- Accept any "key" for testing
- Provide consistent behavior for unit tests
- Generate realistic-looking but non-classified sequences

## Step-by-Step Porting Procedure

### Phase 1: Environment Setup

```bash
# 1. Clone R4W repository (on classified network or air-gapped)
git clone /path/to/r4w.git
cd r4w

# 2. Vendor dependencies (if not already done)
cargo vendor

# 3. Create classified implementation crate
mkdir -p crates/sincgars-classified
cd crates/sincgars-classified
cargo init --lib
```

### Phase 2: Create Classified Crate Structure

```
crates/sincgars-classified/
├── Cargo.toml
├── src/
│   ├── lib.rs
│   ├── hopper.rs          # HoppingAlgorithm implementation
│   ├── transec.rs         # TransecProvider implementation
│   ├── crypto.rs          # CryptoProvider implementation
│   ├── time_sync.rs       # TimeSyncProtocol implementation
│   ├── net_mapper.rs      # NetIdMapper implementation
│   └── tests/
│       └── test_vectors.rs  # Classified test vectors
└── test_vectors/
    └── hopping_sequences.bin  # Binary test data
```

### Phase 3: Implement Traits

```rust
// crates/sincgars-classified/src/lib.rs
use r4w_core::waveform::sincgars::traits::*;

mod hopper;
mod transec;
mod crypto;
mod time_sync;
mod net_mapper;

pub use hopper::ClassifiedHopper;
pub use transec::ClassifiedTransec;
pub use crypto::ClassifiedCrypto;
pub use time_sync::ClassifiedTimeSync;
pub use net_mapper::ClassifiedNetMapper;

/// Create a fully-configured SINCGARS with classified components
pub fn create_operational_sincgars(sample_rate: f64) -> r4w_core::waveform::sincgars::Sincgars {
    r4w_core::waveform::sincgars::SincgarsBuilder::new()
        .sample_rate(sample_rate)
        .hopper(Box::new(ClassifiedHopper::new()))
        .transec(Box::new(ClassifiedTransec::new()))
        .crypto(Box::new(ClassifiedCrypto::new()))
        .time_sync(Box::new(ClassifiedTimeSync::new()))
        .net_mapper(Box::new(ClassifiedNetMapper::new()))
        .build()
        .expect("Failed to build SINCGARS")
}
```

### Phase 4: Hopper Implementation Example

```rust
// crates/sincgars-classified/src/hopper.rs
use r4w_core::waveform::sincgars::traits::*;

pub struct ClassifiedHopper {
    key: Option<TransecKey>,
    net_id: Option<NetId>,
    current_hop: u64,
    state: HopperState,
}

struct HopperState {
    // Classified algorithm state
    // ...
}

impl ClassifiedHopper {
    pub fn new() -> Self {
        Self {
            key: None,
            net_id: None,
            current_hop: 0,
            state: HopperState::default(),
        }
    }

    /// Core hopping algorithm (CLASSIFIED)
    fn compute_channel(&self, hop_number: u64) -> ChannelNumber {
        // This is where the classified algorithm goes
        // The actual implementation uses the TRANSEC key and
        // a cryptographic function to generate a pseudo-random
        // channel sequence that is synchronized across all radios
        // on the same net with the same key.

        // PLACEHOLDER - Replace with actual algorithm
        let key_bytes = self.key.as_ref().unwrap().as_bytes();
        let net = self.net_id.unwrap().0 as u64;

        // Example structure (NOT the real algorithm):
        // channel = f(key, net_id, hop_number) mod 2320
        let seed = key_bytes[0] as u64 ^ (hop_number << 8) ^ (net << 16);
        ChannelNumber((seed % 2320) as u16)
    }
}

impl HoppingAlgorithm for ClassifiedHopper {
    fn initialize(&mut self, key: &TransecKey, net_id: NetId, time: SincgarsTime) {
        self.key = Some(key.clone());
        self.net_id = Some(net_id);
        self.current_hop = self.time_to_hop_number(time);
        self.state = HopperState::initialize(key, net_id);
    }

    fn get_current_channel(&self) -> ChannelNumber {
        self.compute_channel(self.current_hop)
    }

    fn advance_hop(&mut self) {
        self.current_hop += 1;
    }

    fn sync_to_time(&mut self, time: SincgarsTime) {
        self.current_hop = self.time_to_hop_number(time);
    }

    fn time_to_hop_number(&self, time: SincgarsTime) -> u64 {
        // Convert time to hop number based on hop rate
        // Assuming 100 hops/second
        (time.as_millis() / 10) as u64
    }

    fn is_valid(&self) -> bool {
        self.key.is_some() && self.net_id.is_some()
    }

    fn reset(&mut self) {
        self.key = None;
        self.net_id = None;
        self.current_hop = 0;
        self.state.zeroize();
    }

    fn get_hop_number(&self) -> u64 {
        self.current_hop
    }

    fn peek_channels(&self, count: usize) -> Vec<ChannelNumber> {
        (0..count)
            .map(|i| self.compute_channel(self.current_hop + i as u64))
            .collect()
    }
}

impl Drop for ClassifiedHopper {
    fn drop(&mut self) {
        // Secure zeroization
        self.reset();
    }
}
```

### Phase 5: Build Configuration

```toml
# crates/sincgars-classified/Cargo.toml
[package]
name = "sincgars-classified"
version = "0.1.0"
edition = "2021"

[lib]
crate-type = ["rlib", "staticlib"]  # Both for Rust and C linking

[dependencies]
r4w-core = { path = "../r4w-core" }
zeroize = { version = "1.7", features = ["derive"] }  # Secure memory clearing

[dev-dependencies]
# Test dependencies
```

### Phase 6: Integration

```toml
# Your application's Cargo.toml
[dependencies]
r4w-core = { path = "crates/r4w-core" }
sincgars-classified = { path = "crates/sincgars-classified" }

[features]
default = ["classified"]
classified = []
simulator = []  # For testing with unclassified stubs
```

```rust
// src/main.rs
use sincgars_classified::create_operational_sincgars;

fn main() {
    let radio = create_operational_sincgars(48000.0);

    // Load TRANSEC key (from fill device)
    let key = load_transec_key("/dev/fill_device")?;
    radio.transec().load_key(key)?;

    // Join network
    radio.join_network(NetId(1), key_id)?;

    // Transmit
    let audio = record_audio();
    let rf_samples = radio.modulate(&audio);
    transmit(rf_samples);
}
```

### Phase 7: Testing

```rust
// crates/sincgars-classified/src/tests/test_vectors.rs
#[cfg(test)]
mod tests {
    use super::*;

    // Load test vectors from classified documentation
    const TEST_KEY: &[u8] = include_bytes!("../../test_vectors/test_key.bin");
    const EXPECTED_SEQUENCE: &[u16] = &[
        // First 100 channels for test key/net combination
        1423, 892, 2105, 341, 1867, /* ... */
    ];

    #[test]
    fn test_hopping_sequence() {
        let mut hopper = ClassifiedHopper::new();
        let key = TransecKey::from_bytes(TEST_KEY);

        hopper.initialize(&key, NetId(1), SincgarsTime::from_secs(0));

        for (i, &expected) in EXPECTED_SEQUENCE.iter().enumerate() {
            let channel = hopper.get_current_channel();
            assert_eq!(
                channel.0, expected,
                "Hop {} mismatch: got {}, expected {}",
                i, channel.0, expected
            );
            hopper.advance_hop();
        }
    }

    #[test]
    fn test_time_synchronization() {
        let mut hopper1 = ClassifiedHopper::new();
        let mut hopper2 = ClassifiedHopper::new();
        let key = TransecKey::from_bytes(TEST_KEY);

        // Initialize at different times
        hopper1.initialize(&key, NetId(1), SincgarsTime::from_secs(1000));
        hopper2.initialize(&key, NetId(1), SincgarsTime::from_secs(1000));

        // Should produce identical sequences
        for _ in 0..1000 {
            assert_eq!(
                hopper1.get_current_channel(),
                hopper2.get_current_channel()
            );
            hopper1.advance_hop();
            hopper2.advance_hop();
        }
    }
}
```

## Bridging Existing C/C++ SINCGARS Implementation

If you have an existing C/C++ SINCGARS implementation:

```c
// existing_sincgars.h (your existing code)
typedef struct SincgarsState SincgarsState;

SincgarsState* sincgars_create(void);
void sincgars_destroy(SincgarsState* state);
void sincgars_init(SincgarsState* state, const uint8_t* key, uint32_t net_id, uint64_t time);
uint16_t sincgars_get_channel(SincgarsState* state);
void sincgars_advance(SincgarsState* state);
```

Create Rust bridge:

```rust
// crates/sincgars-classified/src/hopper_ffi.rs
use std::ffi::c_void;

#[repr(C)]
struct SincgarsState {
    _private: [u8; 0],
}

extern "C" {
    fn sincgars_create() -> *mut SincgarsState;
    fn sincgars_destroy(state: *mut SincgarsState);
    fn sincgars_init(state: *mut SincgarsState, key: *const u8, net_id: u32, time: u64);
    fn sincgars_get_channel(state: *mut SincgarsState) -> u16;
    fn sincgars_advance(state: *mut SincgarsState);
}

pub struct FfiHopper {
    state: *mut SincgarsState,
}

unsafe impl Send for FfiHopper {}
unsafe impl Sync for FfiHopper {}

impl FfiHopper {
    pub fn new() -> Self {
        Self {
            state: unsafe { sincgars_create() },
        }
    }
}

impl HoppingAlgorithm for FfiHopper {
    fn initialize(&mut self, key: &TransecKey, net_id: NetId, time: SincgarsTime) {
        unsafe {
            sincgars_init(self.state, key.as_ptr(), net_id.0, time.as_secs());
        }
    }

    fn get_current_channel(&self) -> ChannelNumber {
        ChannelNumber(unsafe { sincgars_get_channel(self.state) })
    }

    fn advance_hop(&mut self) {
        unsafe { sincgars_advance(self.state) };
    }

    // ... implement remaining methods
}

impl Drop for FfiHopper {
    fn drop(&mut self) {
        unsafe { sincgars_destroy(self.state) };
    }
}
```

Build configuration for linking:

```rust
// build.rs
fn main() {
    // Link existing C library
    println!("cargo:rustc-link-lib=static=sincgars_legacy");
    println!("cargo:rustc-link-search=native=/path/to/legacy/lib");

    // Rebuild if library changes
    println!("cargo:rerun-if-changed=/path/to/legacy/lib/libsincgars_legacy.a");
}
```

## Cross-Compilation for Target Platforms

### Common SINCGARS Platforms

| Platform | Target Triple | Notes |
|----------|---------------|-------|
| Harris RF | `armv7-unknown-linux-gnueabihf` | ARM Cortex-A |
| Thales | `aarch64-unknown-linux-gnu` | ARM64 |
| L3Harris | `powerpc-unknown-linux-gnu` | PowerPC |
| Custom DSP | Custom toolchain | May need bare metal |

### Example: ARM Cross-Compilation

```bash
# Setup
rustup target add armv7-unknown-linux-gnueabihf
sudo apt install gcc-arm-linux-gnueabihf

# Configure
cat > .cargo/config.toml << 'EOF'
[target.armv7-unknown-linux-gnueabihf]
linker = "arm-linux-gnueabihf-gcc"
EOF

# Build
cargo build --release --target armv7-unknown-linux-gnueabihf

# Output
ls target/armv7-unknown-linux-gnueabihf/release/
```

## Security Considerations

### For Framework Developers (This Repository)

- Never attempt to reverse-engineer or guess classified algorithms
- All trait interfaces are designed to be algorithm-agnostic
- Simulator implementations are clearly marked as non-secure
- No classified information should ever be committed

### For Classified Implementers

- Implement traits in a separate, access-controlled repository
- Use proper key management and zeroization
- Follow NSA/DoD guidelines for handling classified algorithms
- Ensure proper build isolation and supply chain security

### Security Checklist

- [ ] All key material uses `zeroize` on drop
- [ ] No classified data in debug output
- [ ] Timing-constant comparisons for crypto
- [ ] No heap allocations with classified data (or use secure allocator)
- [ ] Memory mapped registers cleared on shutdown
- [ ] Test vectors verified against reference implementation
- [ ] Code review by cleared personnel
- [ ] Build reproducibility verified

## Compliance

This framework is designed to facilitate compliant SINCGARS implementations:

- **IATO/ATO**: Framework code can be reviewed independently
- **Type 1 Crypto**: Crypto trait allows NSA-approved implementations
- **TEMPEST**: Framework makes no assumptions about emissions
- **COMSEC**: Key material handling delegated to classified module

## See Also

- [General Build Procedures](./BUILD_PROCEDURES.md)
- [Main Porting Guide](../PORTING_GUIDE_MILITARY.md)
- [Security Guide](../SECURITY_GUIDE.md)
- [Isolation Guide](../ISOLATION_GUIDE.md)

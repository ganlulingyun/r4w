# Crypto Boundary Integration Design

## Overview

This document describes how R4W supports integration with a Crypto Service Interface (CSI) to enable commercial secure SDR applications. The architecture separates trusted (RED) plaintext processing from untrusted (BLACK) RF/PHY processing.

## Terminology

| Term | Definition |
|------|------------|
| **RED** | Trusted domain: plaintext voice/data, mission logic, key management |
| **BLACK** | Untrusted domain: ciphertext, waveform/PHY, RF processing, over-the-air |
| **Crypto Boundary** | Policy-enforced interface between RED and BLACK domains |
| **CSI** | Crypto Service Interface - the API that enforces the boundary |

## Architecture

### Current R4W Stack (No Crypto Boundary)

```
┌─────────────────────────────────────────┐
│ Application (CLI, GUI, Mission Logic)   │
│                                         │
│   message.as_bytes() ──────────────────►│
├─────────────────────────────────────────┤
│ Waveform Trait                          │
│   modulate(&[u8]) -> Vec<IQSample>      │
│   demodulate(&[IQSample]) -> DemodResult│
├─────────────────────────────────────────┤
│ HAL (StreamHandle, TunerControl)        │
├─────────────────────────────────────────┤
│ RF Hardware / Simulator                 │
└─────────────────────────────────────────┘
```

### With CSI Integration (Commercial Secure)

```
┌─────────────────────────────────────────┐
│ Application (Voice Codec, Data Stack)   │  RED
│                                         │
│   PlaintextIn { payload, policy_id }    │
├═════════════════════════════════════════┤
│ Crypto Service Interface (CSI)          │  ← CRYPTO BOUNDARY
│   - Flow management                     │
│   - AEAD encryption/decryption          │
│   - Replay protection                   │
│   - Key reference (no raw keys)         │
│   - Zeroization                         │
│                                         │
│   CiphertextOut { ciphertext }          │
├═════════════════════════════════════════┤
│ Waveform Trait (sees only ciphertext)   │  BLACK
│   modulate(&[u8]) -> Vec<IQSample>      │
├─────────────────────────────────────────┤
│ HAL (StreamHandle, TunerControl)        │  BLACK
├─────────────────────────────────────────┤
│ RF Hardware                             │  BLACK
└─────────────────────────────────────────┘
```

## Why R4W is Already Compatible

### 1. Waveform Trait Takes Opaque Bytes

```rust
pub trait Waveform {
    fn modulate(&self, data: &[u8]) -> Vec<IQSample>;
    fn demodulate(&self, samples: &[IQSample]) -> DemodResult;
}
```

The waveform layer treats input as opaque bytes. It doesn't know or care whether the bytes are plaintext or ciphertext. This is exactly what a crypto boundary requires.

### 2. HAL is Already "Untrusted"

The HAL layer (`StreamHandle`, `TunerControl`, `ClockControl`) only deals with:
- IQ samples
- Frequencies and gains
- Timing and synchronization

No plaintext ever needs to touch this layer.

### 3. Real-Time Infrastructure Aligns

The physical layer plan includes:
- Lock-free ring buffers (`RingBuffer<T>`)
- Pre-allocated buffer pools
- `no_std` compatibility
- Deterministic timing

These are exactly what CSI requires for embedded deployment.

## CSI Integration Points

### TX Path

```rust
// Application submits plaintext
csi.submit_plaintext(PlaintextIn {
    flow_id: 100,
    service: ServiceType::Voice,
    policy_id: 1,
    seq: next_seq(),
    aad: None,
    payload: voice_frame,
})?;

// CSI produces ciphertext
if let Some(ct) = csi.poll_ciphertext() {
    // Waveform modulates ciphertext (opaque bytes)
    let samples = waveform.modulate(&ct.ciphertext);
    stream.write(&samples, None, timeout)?;
}
```

### RX Path

```rust
// HAL receives samples
let (count, ts) = stream.read(&mut buffer, timeout)?;

// Waveform demodulates to ciphertext
let result = waveform.demodulate(&buffer[..count]);

// CSI decrypts and verifies
csi.submit_ciphertext(CiphertextIn {
    flow_id: 100,
    seq: Some(result.seq),
    ciphertext: result.bits.into(),
    rx_meta: None,
})?;

// Application receives verified plaintext
if let Some(pt) = csi.poll_plaintext() {
    match pt.result {
        CryptoResult::Ok => process_voice(pt.payload),
        CryptoResult::AuthFail => log_security_event(),
        CryptoResult::Replay => drop_duplicate(),
        _ => handle_error(pt.result),
    }
}
```

## CSI Specification Summary

### Message Classes

| Direction | Message | Purpose |
|-----------|---------|---------|
| RED → CSI | `PlaintextIn` | Submit framed payload for encryption |
| CSI → BLACK | `CiphertextOut` | Authenticated ciphertext for PHY |
| BLACK → CSI | `CiphertextIn` | Received ciphertext for verification |
| CSI → RED | `PlaintextOut` | Verified plaintext (empty on auth fail) |

### Control Plane

| Command | Purpose |
|---------|---------|
| `CreateFlow` | Establish secure channel with policy binding |
| `DestroyFlow` | Tear down channel |
| `BindKey` | Associate key reference with flow |
| `Rekey` | Rotate key, increment epoch |
| `Zeroize` | Securely erase keys/state |
| `StatusQuery` | Health, counters, error state |

### Security Properties

1. **Directionality**: Plaintext only enters CSI; ciphertext only exits
2. **No RF metadata with plaintext**: Frequency, modulation, etc. never cross boundary
3. **Bounded metadata**: AAD is small and fixed max size (64 bytes)
4. **Replay protection**: Per-flow sliding window (64-128 packets)
5. **Explicit policy**: Crypto algorithms chosen by policy_id, not waveform code
6. **Zeroization**: Explicit command with observable state transition
7. **Deterministic failures**: Auth fail returns empty payload, never partial plaintext

### Rust Trait

```rust
pub trait CryptoService {
    // Control plane
    fn control(&mut self, cmd: ControlMsg) -> Result<ControlReply, CryptoResult>;
    fn status(&self) -> StatusReply;

    // TX path
    fn submit_plaintext(&mut self, msg: PlaintextIn) -> Result<(), CryptoResult>;
    fn poll_ciphertext(&mut self) -> Option<CiphertextOut>;

    // RX path
    fn submit_ciphertext(&mut self, msg: CiphertextIn) -> Result<(), CryptoResult>;
    fn poll_plaintext(&mut self) -> Option<PlaintextOut>;
}
```

## Implementation Strategy

### Phase 1: Stub Integration (Current)

- Document architecture compatibility
- Define integration points
- No code changes to r4w-core

### Phase 2: CSI Core (Separate Crate)

```
csi/
├── csi-core/          # no_std: types, flow table, replay window
├── csi-queues/        # no_std: bbqueue-based SPSC channels
├── csi-backend-soft/  # no_std: ChaCha20-Poly1305 AEAD
└── csi-backend-hw/    # no_std: hardware crypto acceleration
```

### Phase 3: R4W Integration

```
crates/r4w-secure/
├── src/
│   ├── lib.rs         # Integration layer
│   ├── pipeline.rs    # CSI + Waveform pipeline
│   └── voice.rs       # Voice codec integration
└── Cargo.toml         # feature-gated dependency on csi-core
```

### Phase 4: Embedded Deployment

- STM32H7 target (Cortex-M7)
- Zynq target (ARM + FPGA for PHY acceleration)
- Hardware crypto backend (secure element or HW AES)

## Feature Gating

```toml
# crates/r4w-core/Cargo.toml
[features]
default = []
secure = []  # Marker feature indicating crypto-boundary awareness

# crates/r4w-secure/Cargo.toml (future)
[dependencies]
r4w-core = { path = "../r4w-core", features = ["secure"] }
csi-core = { path = "../../csi/csi-core" }
```

## Compliance Considerations

For commercial secure SDR posture, implementations MUST:

1. **Enforce size bounds** - Use `heapless` types with compile-time limits
2. **Keep plaintext out of PHY** - Waveform only sees ciphertext
3. **Use key references** - Never raw key material in waveform code
4. **Enforce replay protection** - Per-flow sliding window
5. **Provide zeroization** - Observable state transition
6. **Separate control/data planes** - Different message types and queues

## References

- Common Criteria Protection Profiles for secure communications
- HAIPE behavioral model (abstracted, not protocol-specific)
- RustCrypto AEAD implementations (ChaCha20-Poly1305, AES-GCM)

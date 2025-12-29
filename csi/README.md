# Crypto Service Interface (CSI)

**Status: Architecture Only** - This directory contains stub crates for a future Crypto Service Interface implementation. No functional code exists yet.

## Purpose

CSI provides a policy-enforced boundary between trusted (RED) and untrusted (BLACK) domains for commercial secure SDR applications.

## Architecture

```
┌─────────────────────────────────────────┐
│ Application (Voice Codec, Data Stack)   │  RED (Trusted)
│   PlaintextIn { payload, policy_id }    │
├═════════════════════════════════════════┤
│ Crypto Service Interface (CSI)          │  ← CRYPTO BOUNDARY
│   - Flow management                      │
│   - AEAD encryption/decryption          │
│   - Replay protection                    │
│   - Key reference (no raw keys)         │
│   - Zeroization                         │
├═════════════════════════════════════════┤
│ Waveform Trait (sees only ciphertext)   │  BLACK (Untrusted)
│   modulate(&[u8]) -> Vec<IQSample>      │
├─────────────────────────────────────────┤
│ HAL / RF Hardware                        │  BLACK
└─────────────────────────────────────────┘
```

## Crates

| Crate | Purpose | Dependencies |
|-------|---------|--------------|
| `csi-core` | Types, flow table, replay window | `no_std`, `heapless` |
| `csi-queues` | SPSC channels for RED/BLACK boundary | `no_std`, `bbqueue` |
| `csi-backend-soft` | Software AEAD (ChaCha20-Poly1305) | `no_std`, `chacha20poly1305` |
| `csi-backend-hw` | Hardware crypto acceleration | `no_std`, HAL traits |

## Design Principles

1. **no_std from day one** - All crates must work on embedded targets
2. **Heapless types** - Compile-time bounds, no runtime allocation
3. **Key references only** - No raw key material crosses boundaries
4. **Explicit zeroization** - Observable state transitions
5. **Policy-constrained** - Crypto algorithms selected by policy, not code

## Target Platforms

- STM32H7 (Cortex-M7) - Primary embedded target
- Zynq 7000 (ARM + FPGA) - PHY acceleration
- Linux x86_64 - Development and testing

## Integration with R4W

R4W's `Waveform` trait is already compatible with CSI:
- `modulate(&[u8])` treats input as opaque bytes
- HAL layer only handles IQ samples
- No plaintext touches the PHY layer

See `docs/CRYPTO_BOUNDARY.md` for full integration design.

## Implementation Status

Not implemented. This directory exists to:
1. Reserve the namespace
2. Document the intended architecture
3. Provide a starting point for future work

## References

- `docs/CRYPTO_BOUNDARY.md` - Full integration design
- `crypto_boundary_notes.txt` - Original design discussion

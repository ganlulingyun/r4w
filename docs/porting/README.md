# Porting Guides

This directory contains detailed porting guides for integrating R4W waveforms into operational environments.

## Quick Reference

| Guide | Description | Effort Level |
|-------|-------------|--------------|
| [BUILD_PROCEDURES.md](./BUILD_PROCEDURES.md) | General build, linking, FFI, and cross-compilation | Required reading |
| [sincgars.md](./sincgars.md) | SINCGARS VHF frequency hopping | High (classified) |
| [havequick.md](./havequick.md) | HAVEQUICK UHF frequency hopping | High (classified) |
| [link16.md](./link16.md) | Link-16 tactical data link | High (COMSEC) |
| [milstd188110.md](./milstd188110.md) | MIL-STD-188-110 HF modem | Low (unclassified) |
| [p25.md](./p25.md) | APCO P25 public safety radio | Medium (proprietary codec) |

## Porting Complexity

```
┌─────────────────────────────────────────────────────────────────┐
│                    Porting Complexity                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  SINCGARS  ████████████████████░░░░░░░░░  Classified hopping    │
│  HAVEQUICK ███████████████████░░░░░░░░░░  Classified hopping    │
│  Link-16   ██████████████████████░░░░░░░  COMSEC + complexity   │
│  MIL-STD   ██░░░░░░░░░░░░░░░░░░░░░░░░░░░  Viterbi optimization  │
│  P25       ██████████████░░░░░░░░░░░░░░░  Proprietary codec     │
│                                                                 │
│  Legend: █ = Work required  ░ = Complete                        │
└─────────────────────────────────────────────────────────────────┘
```

## Where to Start

### For Classified Waveforms (SINCGARS, HAVEQUICK, Link-16)

1. **Read [BUILD_PROCEDURES.md](./BUILD_PROCEDURES.md)** - Understand linking options, FFI, and secure builds
2. Read the specific waveform guide
3. Set up secure development environment
4. Implement trait interfaces
5. Test against known test vectors
6. Integration testing

### For Unclassified Waveforms (MIL-STD-188-110)

1. Fork R4W repository
2. Implement enhancements directly
3. Submit pull request

### For Proprietary Waveforms (P25)

1. Evaluate codec options (DVSI license, hardware vocoder, etc.)
2. Implement remaining components
3. Test interoperability

## Architecture Overview

All military waveforms use a common architecture:

```
┌─────────────────────────────────────────────────────────────────┐
│                    Your Application                             │
├─────────────────────────────────────────────────────────────────┤
│                         │                                       │
│         ┌───────────────┴───────────────┐                       │
│         ▼                               ▼                       │
│  ┌─────────────────┐          ┌─────────────────┐               │
│  │   R4W Framework │          │ Your Classified │               │
│  │   (Unclassified)│          │ Implementation  │               │
│  │   ───────────── │          │  ─────────────  │               │
│  │  • Modulation   │          │ • Hopping Algo  │               │
│  │  • FEC          │◄────────►│ • TRANSEC       │               │
│  │  • Framing      │  Traits  │ • Crypto        │               │
│  │  • Timing       │          │ • Time Sync     │               │
│  └─────────────────┘          └─────────────────┘               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Key Concepts

### Trait-Based Integration

Classified components implement Rust traits defined in `r4w-core`:

```rust
// Your classified crate
impl HoppingAlgorithm for ClassifiedHopper {
    fn get_current_channel(&self) -> ChannelNumber {
        // Classified implementation
    }
}
```

### Build Options

| Method | Use Case | Complexity |
|--------|----------|------------|
| Cargo (rlib) | Pure Rust integration | Low |
| Cargo (staticlib) | C/C++ linking | Medium |
| Cargo + FFI | Existing C/C++ code | Medium |
| Non-Cargo (rustc) | Secure/air-gapped builds | High |
| Cross-compilation | Embedded targets | Medium |

### Security Considerations

- Use `zeroize` crate for secure memory clearing
- Never log classified data
- Build in secure environment
- Verify against test vectors
- Follow organizational security procedures

## Related Documentation

- [Main Porting Guide](../PORTING_GUIDE_MILITARY.md) - Status overview and effort percentages
- [OVERVIEW.md](../../OVERVIEW.md) - Platform architecture
- [CLAUDE.md](../../CLAUDE.md) - Development guidelines

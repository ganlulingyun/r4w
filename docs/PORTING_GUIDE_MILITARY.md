# Military Waveform Porting Guide

This document describes the implementation status of military waveforms in R4W and provides guidance for porting classified components.

## Detailed Porting Guides

For step-by-step porting instructions, see the detailed guides in [`docs/porting/`](./porting/):

| Guide | Description |
|-------|-------------|
| **[BUILD_PROCEDURES.md](./porting/BUILD_PROCEDURES.md)** | Build options, linking, FFI, C/C++ interop, cross-compilation |
| [sincgars.md](./porting/sincgars.md) | SINCGARS implementation details and trait requirements |
| [havequick.md](./porting/havequick.md) | HAVEQUICK implementation with GPS time sync |
| [link16.md](./porting/link16.md) | Link-16 TDMA, COMSEC, and J-series messages |
| [milstd188110.md](./porting/milstd188110.md) | MIL-STD-188-110 enhancements (fully unclassified) |
| [p25.md](./porting/p25.md) | P25 voice codec options and trunking |

## Overview

R4W implements military waveforms using a **framework + traits** architecture that separates:

1. **Unclassified Framework Code** - Signal processing, modulation/demodulation, message framing
2. **Classified Component Traits** - Interfaces for security-critical algorithms
3. **Simulator Implementations** - Unclassified test/training implementations

The classified components are represented by Rust traits. Organizations with appropriate clearances can implement these traits with actual classified algorithms, compile them in a secure environment, and link them with the unclassified framework.

## Waveform Implementation Status

### SINCGARS (VHF Frequency Hopping)

| Component | Status | Classification | Effort % | Complete % |
|-----------|--------|----------------|----------|------------|
| **Framework** | Complete | UNCLASSIFIED | 5% | 100% |
| FM Modulation/Demodulation | Complete | UNCLASSIFIED | 15% | 100% |
| CVSD Voice Codec | Complete | UNCLASSIFIED | 12% | 100% |
| Data Modem (FSK) | Complete | UNCLASSIFIED | 10% | 100% |
| Message Framing | Complete | UNCLASSIFIED | 8% | 100% |
| **HoppingAlgorithm** | Simulator only | CLASSIFIED | 20% | 20% |
| **TransecProvider** | Simulator only | CLASSIFIED | 12% | 15% |
| **NetIdMapper** | Simulator only | CLASSIFIED | 5% | 20% |
| **TimeSyncProtocol** | Simulator only | CLASSIFIED | 8% | 25% |
| **CryptoProvider** | Simulator only | CLASSIFIED | 5% | 10% |

**Effort-Weighted Completion: 62%**
- Unclassified (50% of effort): 100% complete → 50%
- Classified (50% of effort): 18% average → 9%
- Total weighted: **59%**

#### Required for Operational Use

To port SINCGARS for operational use, implement these traits:

```rust
// crates/r4w-core/src/waveform/sincgars/traits.rs

pub trait HoppingAlgorithm: Send + Sync {
    fn initialize(&mut self, key: &TransecKey, net_id: NetId, time: SincgarsTime);
    fn get_current_channel(&self) -> ChannelNumber;
    fn advance_hop(&mut self);
    fn sync_to_time(&mut self, time: SincgarsTime);
    fn time_to_hop_number(&self, time: SincgarsTime) -> u64;
    fn is_valid(&self) -> bool;
    fn reset(&mut self);
    fn get_hop_number(&self) -> u64;
    fn peek_channels(&self, count: usize) -> Vec<ChannelNumber>;
}

pub trait TransecProvider: Send + Sync {
    fn load_key(&mut self, key_id: KeyId) -> Result<TransecKey, TransecError>;
    fn zeroize(&mut self);
    fn has_valid_key(&self) -> bool;
    fn get_key_metadata(&self) -> Option<KeyMetadata>;
    fn derive_session_key(&self, context: &[u8]) -> Result<SessionKey, TransecError>;
    fn get_current_key(&self) -> Option<&TransecKey>;
}

pub trait CryptoProvider: Send + Sync {
    fn encrypt(&self, plaintext: &[u8], context: &CryptoContext) -> Result<Vec<u8>, CryptoError>;
    fn decrypt(&self, ciphertext: &[u8], context: &CryptoContext) -> Result<Vec<u8>, CryptoError>;
    fn generate_sync_vector(&self) -> SyncVector;
    fn process_sync_vector(&mut self, sync: &SyncVector) -> Result<(), CryptoError>;
    fn zeroize(&mut self);
    fn is_ready(&self) -> bool;
    fn initialize(&mut self, key: &SessionKey) -> Result<(), CryptoError>;
    fn get_frame_counter(&self) -> u64;
}
```

---

### HAVEQUICK (UHF Frequency Hopping)

| Component | Status | Classification | Effort % | Complete % |
|-----------|--------|----------------|----------|------------|
| **Framework** | Complete | UNCLASSIFIED | 5% | 100% |
| AM Voice Modulation | Complete | UNCLASSIFIED | 12% | 100% |
| ASK Data Modulation | Complete | UNCLASSIFIED | 10% | 100% |
| Channel Frequency Math | Complete | UNCLASSIFIED | 8% | 100% |
| WOD Parsing | Complete | UNCLASSIFIED | 10% | 100% |
| **HoppingAlgorithm** | Simulator only | CLASSIFIED | 22% | 20% |
| **TimeSyncProtocol** | Partial | CLASSIFIED | 12% | 40% |
| **NetController** | Simulator only | CLASSIFIED | 10% | 25% |
| **TransecProvider** | Simulator only | CLASSIFIED (KY-58) | 11% | 15% |

**Effort-Weighted Completion: 63%**
- Unclassified (45% of effort): 100% complete → 45%
- Classified (55% of effort): 24% average → 13%
- Total weighted: **58%**

#### Required for Operational Use

```rust
// crates/r4w-core/src/waveform/havequick/traits.rs

pub trait HoppingAlgorithm: Send + Sync {
    fn initialize(&mut self, wod: &WordOfDay, net_id: NetId, tod: TimeOfDay);
    fn get_current_channel(&self) -> ChannelNumber;
    fn advance_hop(&mut self);
    fn sync_to_time(&mut self, tod: TimeOfDay);
    fn current_hop_number(&self) -> u64;
    fn predict_channel(&self, hop_number: u64) -> ChannelNumber;
}

pub trait NetController: Send + Sync {
    fn load_wod(&mut self, net_id: NetId, wod: WordOfDay) -> Result<(), HavequickError>;
    fn get_wod(&self, net_id: NetId) -> Option<&WordOfDay>;
    fn is_authorized(&self, net_id: NetId) -> bool;
    fn get_authorized_nets(&self) -> Vec<NetId>;
    fn zeroize(&mut self);
}
```

---

### Link-16 (L-Band TDMA + Frequency Hopping)

| Component | Status | Classification | Effort % | Complete % |
|-----------|--------|----------------|----------|------------|
| **Framework** | Complete | UNCLASSIFIED | 5% | 100% |
| MSK Modulation | Complete | UNCLASSIFIED | 10% | 100% |
| TDMA Timing | Complete | UNCLASSIFIED | 8% | 100% |
| J-Series Message Codec | Complete | UNCLASSIFIED | 12% | 100% |
| Reed-Solomon FEC | Simulator | UNCLASSIFIED | 8% | 80% |
| Interleaver | Complete | UNCLASSIFIED | 5% | 100% |
| Pulse Formatter | Complete | UNCLASSIFIED | 6% | 100% |
| Track Database | Complete | UNCLASSIFIED | 8% | 100% |
| NPG Management | Complete | UNCLASSIFIED | 6% | 100% |
| **HoppingPattern** | Simulator only | COMSEC | 12% | 20% |
| **TransecProvider** | Simulator only | CLASSIFIED | 10% | 15% |
| **TimeSync (NTRU)** | Partial | UNCLASSIFIED | 5% | 60% |
| **NetworkController** | Partial | UNCLASSIFIED | 5% | 70% |

**Effort-Weighted Completion: 78%**
- Core unclassified (68% of effort): 96% average → 65%
- Classified/COMSEC (22% of effort): 18% average → 4%
- Partial components (10% of effort): 65% average → 7%
- Total weighted: **76%**

#### Required for Operational Use

```rust
// crates/r4w-core/src/waveform/link16/traits.rs

pub trait HoppingPattern: Send + Sync {
    fn initialize(&mut self, crypto_var: &[u8], net_id: u16);
    fn get_frequency(&self, slot: TimeSlot) -> Frequency;
    fn get_epoch_frequencies(&self, epoch: u8) -> Vec<Frequency>;
    fn reset(&mut self);
}

pub trait TransecProvider: Send + Sync {
    fn load_key(&mut self, key: &[u8]) -> Result<(), Link16Error>;
    fn encrypt(&self, data: &[u8], slot: TimeSlot) -> Vec<u8>;
    fn decrypt(&self, data: &[u8], slot: TimeSlot) -> Result<Vec<u8>, Link16Error>;
    fn is_loaded(&self) -> bool;
    fn mode(&self) -> CryptoMode;
    fn zeroize(&mut self);
}
```

---

### MIL-STD-188-110 (HF Serial Tone)

| Component | Status | Classification | Effort % | Complete % |
|-----------|--------|----------------|----------|------------|
| PSK Modulation (BPSK/QPSK/8-PSK) | Complete | UNCLASSIFIED | 25% | 100% |
| Convolutional FEC | Complete | UNCLASSIFIED | 20% | 100% |
| Block Interleaver | Complete | UNCLASSIFIED | 15% | 100% |
| Preamble Generation | Complete | UNCLASSIFIED | 15% | 100% |
| Data Rate Modes | Complete | UNCLASSIFIED | 10% | 100% |
| Viterbi Decoder | Basic | UNCLASSIFIED | 15% | 60% |

**Effort-Weighted Completion: 94%**
- Complete components (85% of effort): 100% → 85%
- Viterbi decoder (15% of effort): 60% → 9%
- Total weighted: **94%**

*Note: This is a fully unclassified waveform. The Viterbi decoder works but could be optimized.*

---

### APCO P25 (Public Safety Digital Radio)

| Component | Status | Classification | Effort % | Complete % |
|-----------|--------|----------------|----------|------------|
| C4FM Modulation | Complete | UNCLASSIFIED | 12% | 100% |
| CQPSK Modulation | Complete | UNCLASSIFIED | 10% | 100% |
| H-DQPSK (Phase 2) | Complete | UNCLASSIFIED | 12% | 100% |
| Frame Sync | Complete | UNCLASSIFIED | 8% | 100% |
| NID/NAC Encoding | Complete | UNCLASSIFIED | 5% | 100% |
| IMBE Voice Codec | Not implemented | PROPRIETARY | 18% | 0% |
| AMBE+2 Voice Codec | Not implemented | PROPRIETARY | 15% | 0% |
| Trunking Protocol | Not implemented | UNCLASSIFIED | 12% | 0% |
| AES Encryption | Not implemented | UNCLASSIFIED | 8% | 0% |

**Effort-Weighted Completion: 47%**
- RF layer (47% of effort): 100% complete → 47%
- Voice codecs (33% of effort): 0% → 0%
- Trunking/encryption (20% of effort): 0% → 0%
- Total weighted: **47%**

#### Notes on P25 Codecs

The IMBE and AMBE+2 voice codecs used by P25 are proprietary (DVSI). Options:

1. **Codec2** - Open source codec, not P25 compatible but similar quality
2. **mbelib** - Open source IMBE/AMBE decoder (receive only, legal gray area)
3. **DVSI license** - Purchase vocoder license from DVSI for full compatibility
4. **Hardware vocoder** - Use DVSI AMBE chip (e.g., DV3000) via serial interface

---

## Porting Process

### For Classified Components

1. **Obtain Authorization**
   - Ensure proper clearances and facility accreditation
   - Obtain necessary agreements (CDA, MOU, etc.)

2. **Set Up Secure Environment**
   - Air-gapped development system
   - Classified build environment
   - Code review procedures

3. **Implement Traits**
   ```rust
   // In your classified crate
   use r4w_core::waveform::sincgars::traits::*;

   pub struct ClassifiedHopper {
       // Classified state
   }

   impl HoppingAlgorithm for ClassifiedHopper {
       fn initialize(&mut self, key: &TransecKey, net_id: NetId, time: SincgarsTime) {
           // Classified implementation
       }
       // ... other methods
   }
   ```

4. **Build with Feature Flags**
   ```toml
   # Cargo.toml
   [features]
   default = ["simulator"]
   simulator = []
   classified = []  # Enable classified implementations
   ```

5. **Testing**
   - Use simulator implementations for unclassified testing
   - Classified testing in secure environment only

### For Unclassified Components

Components like voice codecs, improved FEC, or better sync can be contributed directly:

1. Fork the repository
2. Implement the improvement
3. Add tests
4. Submit pull request

---

## Security Considerations

### Do NOT

- Commit classified code to this repository
- Discuss classified algorithms in issues/PRs
- Attempt to reverse-engineer classified implementations
- Use simulator implementations for actual secure communications

### DO

- Use simulator implementations for training and development
- Implement classified components in separate, secure repositories
- Follow your organization's security procedures
- Report security issues responsibly

---

## Summary Table

| Waveform | Effort-Weighted | Unclassified Work | Classified/Proprietary | Primary Missing |
|----------|-----------------|-------------------|------------------------|-----------------|
| SINCGARS | **59%** | 50% (done) | 50% (stubs) | Hopping, TRANSEC, Crypto |
| HAVEQUICK | **58%** | 45% (done) | 55% (stubs) | Hopping, WOD, KY-58 |
| Link-16 | **76%** | 68% (done) | 22% (stubs) | Hopping, TRANSEC |
| MIL-STD-188-110 | **94%** | 100% (all) | 0% | Viterbi optimization |
| P25 | **47%** | 47% (RF done) | 53% (missing) | Voice codec, Trunking |

### Effort Breakdown by Category

```
┌─────────────────────────────────────────────────────────────────┐
│                    Total Effort Distribution                    │
├─────────────────────────────────────────────────────────────────┤
│ SINCGARS:   [██████████████████████░░░░░░░░░░░░░░░░░░] 59%      │
│ HAVEQUICK:  [█████████████████████░░░░░░░░░░░░░░░░░░░] 58%      │
│ Link-16:    [██████████████████████████████░░░░░░░░░░] 76%      │
│ MIL-STD:    [█████████████████████████████████████░░░] 94%      │
│ P25:        [██████████████████░░░░░░░░░░░░░░░░░░░░░░] 47%      │
├─────────────────────────────────────────────────────────────────┤
│ Overall Average (effort-weighted): 67%                          │
└─────────────────────────────────────────────────────────────────┘

Legend:
  ███ = Complete (unclassified, fully functional)
  ░░░ = Remaining (classified stubs or not implemented)
```

---

## Contact

For questions about porting or contributing unclassified improvements, please open a GitHub issue.

For classified integration inquiries, contact your program office or contracting organization.

---
title: '<img src="images/r4w_logo_1.png" height="150px" style="margin-bottom: 20px;"><br>Security & Isolation'
subtitle: "Defense in Depth for SDR Systems"
author: "R4W Development Team<br>(Aida, Joe Mooney, Claude Code)"
date: "December 2025"
header-includes: |
  <style>
    .reveal pre {
      margin: 0 auto;
      width: fit-content;
    }
    .reveal pre code {
      text-align: left;
    }
    .reveal section img {
      max-height: 60vh;
    }
  </style>
---

## Why SDR Security Matters

SDR systems face unique threats:

| Threat | Risk |
|--------|------|
| **Malicious RF signals** | Buffer overflows, DoS |
| **Untrusted waveforms** | Code execution |
| **Key compromise** | Traffic decryption |
| **Physical access** | Hardware tampering |

R4W provides **8 isolation levels**.

---

## Defense in Depth

```
┌──────────────────────────────────────────────────────────┐
│                    Application Layer                      │
│  • Input validation • Authentication • Audit logging      │
├──────────────────────────────────────────────────────────┤
│                    R4W Framework Layer                    │
│  • Memory-safe Rust • Type-safe interfaces                │
│  • Zeroization • Constant-time crypto                     │
├──────────────────────────────────────────────────────────┤
│                    OS/Runtime Layer                       │
│  • Namespaces • Seccomp • SELinux/AppArmor               │
├──────────────────────────────────────────────────────────┤
│                    Hardware Layer                         │
│  • Secure boot • TPM/HSM • Physical controls              │
└──────────────────────────────────────────────────────────┘
```

---

## 8 Isolation Levels

| Level | Mechanism | Overhead | Use Case |
|-------|-----------|----------|----------|
| L1 | Rust memory safety | 0% | All code |
| L2 | Linux namespaces | ~1% | Process isolation |
| L3 | Seccomp + LSM | ~2% | System call filtering |
| L4 | Container (Docker) | ~5% | Portable isolation |
| L5 | MicroVM (Firecracker) | ~10% | Strong isolation |
| L6 | Full VM (KVM) | ~15% | Complete separation |
| L7 | FPGA partition | Varies | Hardware isolation |
| L8 | Air gap | N/A | Maximum security |

---

## Trust Boundaries

```
┌────────────────────────────────────────────────────────┐
│                    Untrusted Zone                       │
│  RF Environment - Adversary can transmit anything       │
├────────────────────────────────────────────────────────┤
│              Trust Boundary: RF Front-End               │
├────────────────────────────────────────────────────────┤
│  SDR/ADC - Physical interface                           │
├────────────────────────────────────────────────────────┤
│              Trust Boundary: Driver/Userspace           │
├────────────────────────────────────────────────────────┤
│  R4W Processing - Sample processing                     │
├────────────────────────────────────────────────────────┤
│              Trust Boundary: Application                │
├────────────────────────────────────────────────────────┤
│                    Trusted Zone                         │
│  Crypto/Keys - Classified/sensitive data                │
└────────────────────────────────────────────────────────┘
```

---

## Level 1: Rust Memory Safety

**Zero runtime cost protection:**

- No null pointer dereferences
- No buffer overflows
- No use-after-free
- No data races
- Bounds checking on arrays

```rust
// Safe by default
fn process(samples: &[IQSample]) -> Vec<bool> {
    samples.iter()  // Bounds checked
        .map(|s| s.magnitude() > threshold)
        .collect()
}
```

---

## Level 2: Linux Namespaces

Process isolation without containers:

```rust
use r4w_sandbox::Namespace;

let sandbox = Namespace::builder()
    .user()      // Separate user IDs
    .mount()     // Private filesystem
    .network()   // No network access
    .pid()       // Isolated process tree
    .build()?;

sandbox.spawn(waveform_process)?;
```

Overhead: ~1%

---

## Level 3: Seccomp + LSM

Restrict system calls:

```rust
use r4w_sandbox::SeccompFilter;

let filter = SeccompFilter::new()
    .allow_read()
    .allow_write()
    .allow_mmap()
    .deny_network()
    .deny_exec()
    .build()?;

filter.apply()?;
```

Only permitted syscalls succeed.

---

## Level 4: Container (Docker)

Portable isolation with OCI containers:

```bash
# Run waveform in container
docker run --rm \
  --read-only \
  --cap-drop=ALL \
  --security-opt=no-new-privileges \
  --network=none \
  r4w/waveform:lora
```

Overhead: ~5%

---

## Level 5: MicroVM (Firecracker)

Lightweight VM for strong isolation:

```rust
use r4w_sandbox::Firecracker;

let vm = Firecracker::builder()
    .kernel("/path/to/vmlinux")
    .rootfs("/path/to/rootfs.ext4")
    .vcpu_count(1)
    .mem_size_mib(128)
    .build()?;

vm.start()?;
```

Sub-100ms boot time.

---

## Level 6: Full VM (KVM)

Complete OS isolation:

| Feature | MicroVM | Full VM |
|---------|---------|---------|
| Boot time | <100ms | 2-10s |
| Memory | 128 MB | 512+ MB |
| Device emulation | Minimal | Full QEMU |
| Use case | Untrusted waveforms | Full OS |

---

## Level 7: FPGA Partition

Hardware isolation on FPGA:

```
┌───────────────────────────────────────────┐
│              FPGA Fabric                   │
├───────────────┬───────────────────────────┤
│  Partition A  │  Partition B              │
│  (Trusted)    │  (Untrusted Waveform)     │
│               │                           │
│  Core DSP     │  Custom processing        │
│  DMA ctrl     │  No DMA access            │
│  Full AXI     │  Limited registers        │
├───────────────┼───────────────────────────┤
│       Hardware Firewall (no crossing)      │
└───────────────────────────────────────────┘
```

---

## Threat Model

| Adversary | Capabilities | Mitigations |
|-----------|--------------|-------------|
| **Passive RF** | Intercept signals | Encryption, spread spectrum |
| **Active RF** | Inject signals | Input validation |
| **Network** | Attack control interfaces | TLS, auth |
| **Local** | Access processes | Isolation levels |
| **Physical** | Hardware access | Secure boot, tamper detect |
| **Supply chain** | Compromised deps | Audits, SBOM |

---

## Memory Zeroization

Sensitive data cleared on drop:

```rust
use zeroize::Zeroize;

struct CryptoKey {
    key: [u8; 32],
}

impl Drop for CryptoKey {
    fn drop(&mut self) {
        self.key.zeroize();  // Secure clear
    }
}
```

Keys never left in memory.

---

## Constant-Time Operations

Prevent timing side-channels:

```rust
use subtle::ConstantTimeEq;

// BAD: Variable-time comparison
if key == expected { ... }

// GOOD: Constant-time comparison
if key.ct_eq(&expected).into() { ... }
```

Used for all crypto operations.

---

## Audit Logging

Every security-relevant event logged:

```rust
use r4w_audit::AuditLog;

audit.log(AuditEvent::WaveformLoaded {
    name: "lora",
    hash: "sha256:abc...",
    isolation_level: 4,
    user: "operator",
})?;
```

Append-only, tamper-evident.

---

## Secure Deployment

Production checklist:

- [ ] Enable Secure Boot
- [ ] Configure SELinux/AppArmor
- [ ] Disable unnecessary services
- [ ] Use TLS for all control interfaces
- [ ] Enable audit logging
- [ ] Configure firewall rules
- [ ] Set appropriate isolation level
- [ ] Verify SBOM

---

## Network Security

```
┌─────────────────────────────────────────────────────────┐
│                    R4W Control Plane                     │
├─────────────────────────────────────────────────────────┤
│  REST API    │  gRPC        │  WebSocket                │
│  HTTPS/TLS   │  mTLS        │  WSS                      │
├─────────────────────────────────────────────────────────┤
│              Authentication                              │
│  • API keys  • Certificates  • OAuth2/OIDC              │
├─────────────────────────────────────────────────────────┤
│              Authorization                               │
│  • Role-based  • Attribute-based  • Capability-based    │
└─────────────────────────────────────────────────────────┘
```

---

## Crypto Key Management

| Key Type | Storage | Protection |
|----------|---------|------------|
| **Master keys** | HSM/TPM | Hardware |
| **Session keys** | Memory only | Zeroization |
| **Hop sequences** | Encrypted | Access control |
| **Bitstream auth** | eFuse/BBRAM | One-time write |

Never store keys in plaintext.

---

## FPGA Bitstream Security

```
┌─────────────────────────────────────────────────────────┐
│                Bitstream Protection                      │
├─────────────────────────────────────────────────────────┤
│  1. Encryption (AES-256-GCM)                            │
│  2. Authentication (RSA/ECDSA signature)                │
│  3. Anti-tamper (readback disabled)                     │
│  4. Secure boot chain                                   │
│  5. Key storage in eFuse or BBRAM                       │
└─────────────────────────────────────────────────────────┘
```

---

## Side-Channel Mitigations

| Attack | Mitigation |
|--------|------------|
| **Timing** | Constant-time code |
| **Power analysis** | DPA-resistant impl |
| **EM emanation** | Shielding, filtering |
| **Cache timing** | Flushing, partitioning |
| **Spectre/Meltdown** | OS patches, isolation |

---

## Choosing Isolation Level

| Scenario | Recommended Level |
|----------|-------------------|
| Internal development | L1-L2 |
| Trusted waveforms | L2-L3 |
| Third-party waveforms | L4-L5 |
| Untrusted/adversarial | L5-L6 |
| Classified systems | L6-L8 |
| Maximum security | L8 (air gap) |

---

## Security Summary

| Layer | Protection |
|-------|------------|
| **Language** | Rust memory safety |
| **Process** | Namespaces, seccomp |
| **Container** | Docker, Firecracker |
| **Hardware** | FPGA partitions |
| **Network** | TLS, authentication |
| **Crypto** | HSM, zeroization |
| **Audit** | Logging, SBOM |

**8 levels of defense in depth.**

---

## Questions?

**R4W - Security & Isolation**

github.com/joemooney/r4w

Docs: `docs/SECURITY_GUIDE.md`

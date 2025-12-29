# R4W Security Guide

This document provides comprehensive security guidance for developing, deploying, and operating R4W-based SDR systems. It covers memory safety, cryptographic handling, process isolation, network security, and special considerations for military/classified environments.

## Table of Contents

1. [Security Architecture Overview](#security-architecture-overview)
2. [Threat Model](#threat-model)
3. [Memory Safety](#memory-safety)
4. [Cryptographic Key Management](#cryptographic-key-management)
5. [Process Isolation](#process-isolation)
6. [Sandboxing](#sandboxing)
7. [Waveform Isolation](#waveform-isolation)
8. [Network Security](#network-security)
9. [Secure Deployment](#secure-deployment)
10. [Audit Logging](#audit-logging)
11. [Side-Channel Considerations](#side-channel-considerations)
12. [Physical Security](#physical-security)
13. [Military and Classified Environments](#military-and-classified-environments)
14. [Secure Development Practices](#secure-development-practices)
15. [Incident Response](#incident-response)

---

## Security Architecture Overview

### Defense in Depth

R4W implements a layered security approach:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Application Layer                               │
│  ┌─────────────────────────────────────────────────────────────────────┐│
│  │ Waveform Applications (user code)                                   ││
│  │ • Input validation                                                  ││
│  │ • Authentication/Authorization                                      ││
│  │ • Audit logging                                                     ││
│  └─────────────────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────────────────┤
│                         R4W Framework Layer                             │
│  ┌─────────────────────────────────────────────────────────────────────┐│
│  │ r4w-core, r4w-sim, r4w-fpga                                         ││
│  │ • Memory-safe Rust implementation                                   ││
│  │ • Type-safe interfaces                                              ││
│  │ • Zeroization of sensitive data                                     ││
│  │ • Constant-time crypto operations                                   ││
│  └─────────────────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────────────────┤
│                         OS/Runtime Layer                                │
│  ┌─────────────────────────────────────────────────────────────────────┐│
│  │ • Process isolation (namespaces, cgroups)                           ││
│  │ • Capability dropping                                               ││
│  │ • Seccomp filtering                                                 ││
│  │ • SELinux/AppArmor policies                                         ││
│  └─────────────────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────────────────┤
│                         Hardware Layer                                  │
│  ┌─────────────────────────────────────────────────────────────────────┐│
│  │ • FPGA bitstream authentication                                     ││
│  │ • Secure boot chain                                                 ││
│  │ • Hardware crypto modules (TPM, HSM)                                ││
│  │ • Physical access controls                                          ││
│  └─────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────┘
```

### Security Boundaries

```
┌───────────────────────────────────────────────────────────────────────────┐
│                          Untrusted Zone                                   │
│  ┌─────────────────┐                                                      │
│  │ RF Environment  │  Adversary can transmit arbitrary signals            │
│  └────────┬────────┘                                                      │
│           │                                                               │
├───────────┼───────────────────────────────────────────────────────────────┤
│           ▼           Trust Boundary: RF Front-End                        │
├───────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐                                                      │
│  │    SDR/ADC      │  Physical interface                                  │
│  └────────┬────────┘                                                      │
│           │                                                               │
├───────────┼───────────────────────────────────────────────────────────────┤
│           ▼           Trust Boundary: Driver/Userspace                    │
├───────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐                                                      │
│  │ R4W Processing  │  Sample processing                                   │
│  └────────┬────────┘                                                      │
│           │                                                               │
├───────────┼───────────────────────────────────────────────────────────────┤
│           ▼           Trust Boundary: Application Interface               │
├───────────────────────────────────────────────────────────────────────────┤
│                          Trusted Zone                                     │
│  ┌─────────────────┐                                                      │
│  │ Crypto/Keys     │  Classified/sensitive data                           │
│  └─────────────────┘                                                      │
└───────────────────────────────────────────────────────────────────────────┘
```

---

## Threat Model

### Adversary Capabilities

| Adversary Type | Capabilities | Mitigations |
|----------------|--------------|-------------|
| **Passive RF Monitor** | Intercept transmitted signals | Encryption, spread spectrum |
| **Active RF Attacker** | Inject malicious RF signals | Input validation, authentication |
| **Network Attacker** | Attack control interfaces | Firewalls, TLS, authentication |
| **Local Attacker** | Access to system processes | Process isolation, capabilities |
| **Physical Attacker** | Hardware access | Tamper detection, secure boot |
| **Supply Chain** | Compromised dependencies | Vendoring, audits, SBOM |

### Assets to Protect

| Asset | Sensitivity | Protection |
|-------|-------------|------------|
| Cryptographic keys | Critical | HSM, zeroization, access control |
| Hop sequences | Secret | Memory isolation, no logging |
| Transmitted data | Variable | Encryption, integrity checks |
| System configuration | Sensitive | File permissions, integrity |
| Audit logs | Important | Append-only, signed |
| Bitstreams | Sensitive | Authentication, encryption |

### Attack Surfaces

```
┌─────────────────────────────────────────────────────────────────┐
│                       Attack Surfaces                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  RF Interface                                                   │
│  ├── Malformed signals → Buffer overflow, DoS                   │
│  ├── Signal injection → Protocol confusion                      │
│  └── Jamming → Availability                                     │
│                                                                 │
│  Control Interface (CLI/API)                                    │
│  ├── Command injection                                          │
│  ├── Authentication bypass                                      │
│  └── Configuration manipulation                                 │
│                                                                 │
│  Network Interface                                              │
│  ├── UDP transport spoofing                                     │
│  ├── TCP control channel attacks                                │
│  └── TLS downgrade/bypass                                       │
│                                                                 │
│  File System                                                    │
│  ├── Key file theft                                             │
│  ├── Configuration tampering                                    │
│  └── Log manipulation                                           │
│                                                                 │
│  Inter-Process                                                  │
│  ├── Shared memory attacks                                      │
│  ├── Signal injection                                           │
│  └── File descriptor leaks                                      │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Memory Safety

### Rust's Memory Safety Guarantees

R4W is written in Rust, providing:
- No null pointer dereferences
- No buffer overflows
- No use-after-free
- No data races
- Bounds checking on array access

### Safe Pattern: Validated Input Processing

```rust
use r4w_core::error::{R4wError, Result};

/// Process RF samples with validation
pub fn process_samples(samples: &[IQSample]) -> Result<Vec<bool>> {
    // Validate input bounds
    if samples.is_empty() {
        return Err(R4wError::InvalidInput("empty sample buffer".into()));
    }

    if samples.len() > MAX_SAMPLES {
        return Err(R4wError::InvalidInput(
            format!("sample count {} exceeds max {}", samples.len(), MAX_SAMPLES)
        ));
    }

    // Validate individual samples (detect hardware errors)
    for (i, sample) in samples.iter().enumerate() {
        if !sample.i.is_finite() || !sample.q.is_finite() {
            return Err(R4wError::InvalidInput(
                format!("non-finite sample at index {}", i)
            ));
        }
    }

    // Safe processing
    Ok(demodulate_validated(samples))
}
```

### Safe Pattern: Bounded Allocations

```rust
/// Maximum allocation sizes to prevent DoS
pub const MAX_SYMBOL_COUNT: usize = 65536;
pub const MAX_SAMPLE_RATE: u32 = 10_000_000;
pub const MAX_FFT_SIZE: usize = 8192;

pub struct WaveformConfig {
    symbol_count: usize,
    sample_rate: u32,
}

impl WaveformConfig {
    pub fn new(symbol_count: usize, sample_rate: u32) -> Result<Self> {
        if symbol_count > MAX_SYMBOL_COUNT {
            return Err(R4wError::InvalidConfig(
                format!("symbol_count {} exceeds max {}", symbol_count, MAX_SYMBOL_COUNT)
            ));
        }

        if sample_rate > MAX_SAMPLE_RATE {
            return Err(R4wError::InvalidConfig(
                format!("sample_rate {} exceeds max {}", sample_rate, MAX_SAMPLE_RATE)
            ));
        }

        Ok(Self { symbol_count, sample_rate })
    }
}
```

### Unsafe Code Guidelines

When unsafe code is required (FPGA register access, performance-critical sections):

```rust
/// FPGA register access requires unsafe due to memory-mapped I/O
///
/// # Safety
/// - `base_addr` must be a valid mapped FPGA register base
/// - Caller must ensure no concurrent access to the same register
/// - Register offset must be within the valid range for this IP core
#[inline]
pub unsafe fn write_register(base_addr: *mut u32, offset: usize, value: u32) {
    // Validate offset is within expected range
    debug_assert!(offset < MAX_REGISTER_OFFSET, "register offset out of range");

    // Volatile write to ensure it's not optimized away
    core::ptr::write_volatile(base_addr.add(offset), value);

    // Memory barrier for correct ordering
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
}

// Encapsulate unsafe in safe interface
pub struct FpgaRegisters {
    base: *mut u32,
    size: usize,
}

impl FpgaRegisters {
    pub fn write(&self, offset: usize, value: u32) -> Result<()> {
        if offset >= self.size {
            return Err(R4wError::InvalidRegister(offset));
        }

        // Safe interface to unsafe operation
        unsafe {
            write_register(self.base, offset, value);
        }
        Ok(())
    }
}
```

### Memory Zeroization

Always zeroize sensitive data when no longer needed:

```rust
use zeroize::{Zeroize, ZeroizeOnDrop};

/// Cryptographic key that zeroizes on drop
#[derive(Zeroize, ZeroizeOnDrop)]
pub struct CryptoKey {
    key_material: [u8; 32],
    key_id: u32,
}

impl CryptoKey {
    pub fn new(material: [u8; 32], id: u32) -> Self {
        Self {
            key_material: material,
            key_id: id,
        }
    }

    /// Explicitly zeroize key material
    pub fn destroy(&mut self) {
        self.zeroize();
    }
}

// Example usage
fn load_and_use_key() {
    let mut key = CryptoKey::new([0x42; 32], 1);

    // Use key...
    encrypt_with_key(&key);

    // Key is automatically zeroized when dropped
} // <-- key.zeroize() called here automatically

/// Session context with automatic cleanup
#[derive(ZeroizeOnDrop)]
pub struct SecureSession {
    #[zeroize(skip)]  // Don't zeroize the waveform config
    waveform: WaveformConfig,

    session_key: [u8; 32],
    hop_sequence: Vec<u32>,
    iv: [u8; 16],
}
```

---

## Cryptographic Key Management

### Key Hierarchy

```
┌─────────────────────────────────────────────────────────────────┐
│                      Key Hierarchy                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Master Key (KEK)                                               │
│  └── Device Root Key (hardware-bound)                           │
│      │                                                          │
│      ├── TRANSEC Key                                            │
│      │   └── Session Key (hop sequence)                         │
│      │                                                          │
│      ├── COMSEC Key                                             │
│      │   └── Traffic Encryption Key (per-message)               │
│      │                                                          │
│      └── Bitstream Key                                          │
│          └── FPGA configuration encryption                      │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Key Storage

```rust
use std::fs::OpenOptions;
use std::os::unix::fs::OpenOptionsExt;

/// Secure key file creation with restricted permissions
pub fn create_key_file(path: &std::path::Path) -> std::io::Result<std::fs::File> {
    OpenOptions::new()
        .write(true)
        .create_new(true)  // Fail if exists (prevent overwrite attacks)
        .mode(0o600)       // Owner read/write only
        .open(path)
}

/// Key storage with encryption at rest
pub struct KeyStore {
    path: std::path::PathBuf,
    master_key: [u8; 32],  // Derived from hardware/password
}

impl KeyStore {
    /// Load key with integrity verification
    pub fn load_key(&self, key_id: &str) -> Result<CryptoKey> {
        let encrypted_data = std::fs::read(self.path.join(key_id))?;

        // Verify MAC before decryption
        let (ciphertext, mac) = encrypted_data.split_at(encrypted_data.len() - 32);
        let computed_mac = hmac_sha256(&self.master_key, ciphertext);

        // Constant-time comparison to prevent timing attacks
        if !constant_time_eq(&computed_mac, mac) {
            return Err(R4wError::KeyIntegrityError);
        }

        // Decrypt key material
        let key_material = aes_gcm_decrypt(&self.master_key, ciphertext)?;

        Ok(CryptoKey::from_bytes(&key_material)?)
    }

    /// Store key with encryption and integrity
    pub fn store_key(&self, key_id: &str, key: &CryptoKey) -> Result<()> {
        let key_bytes = key.to_bytes();

        // Encrypt key material
        let ciphertext = aes_gcm_encrypt(&self.master_key, &key_bytes)?;

        // Add MAC
        let mac = hmac_sha256(&self.master_key, &ciphertext);

        let mut output = ciphertext;
        output.extend_from_slice(&mac);

        // Write to file with restricted permissions
        let file = create_key_file(&self.path.join(key_id))?;
        std::io::Write::write_all(&mut std::io::BufWriter::new(file), &output)?;

        Ok(())
    }
}
```

### Hardware Security Module (HSM) Integration

```rust
/// HSM abstraction for key operations
pub trait HsmProvider: Send + Sync {
    /// Generate key in HSM (never leaves hardware)
    fn generate_key(&self, key_type: KeyType) -> Result<KeyHandle>;

    /// Import wrapped key to HSM
    fn import_key(&self, wrapped: &[u8]) -> Result<KeyHandle>;

    /// Encrypt using HSM-stored key
    fn encrypt(&self, handle: KeyHandle, plaintext: &[u8]) -> Result<Vec<u8>>;

    /// Decrypt using HSM-stored key
    fn decrypt(&self, handle: KeyHandle, ciphertext: &[u8]) -> Result<Vec<u8>>;

    /// Derive session key
    fn derive_key(&self, handle: KeyHandle, context: &[u8]) -> Result<KeyHandle>;

    /// Destroy key in HSM
    fn destroy_key(&self, handle: KeyHandle) -> Result<()>;
}

/// TPM 2.0 HSM implementation
pub struct Tpm2Provider {
    context: tss_esapi::Context,
}

impl HsmProvider for Tpm2Provider {
    fn generate_key(&self, key_type: KeyType) -> Result<KeyHandle> {
        // Use TPM to generate key
        // Key material never leaves the TPM
        todo!("TPM2 key generation")
    }

    // ... other methods
}

/// Software fallback (for development/testing only)
pub struct SoftwareHsm {
    keys: std::sync::RwLock<std::collections::HashMap<KeyHandle, Vec<u8>>>,
}

impl HsmProvider for SoftwareHsm {
    fn generate_key(&self, key_type: KeyType) -> Result<KeyHandle> {
        use rand::RngCore;

        let mut key = vec![0u8; key_type.size()];
        rand::thread_rng().fill_bytes(&mut key);

        let handle = KeyHandle::new_random();
        self.keys.write().unwrap().insert(handle, key);

        Ok(handle)
    }

    // ... other methods
}
```

### Key Zeroization Protocol

```rust
/// Secure key destruction sequence
pub fn emergency_zeroize() {
    // 1. Zeroize all session keys
    SESSION_KEYS.write().unwrap().zeroize();

    // 2. Zeroize hop sequences
    HOP_SEQUENCES.write().unwrap().zeroize();

    // 3. Zeroize TRANSEC state
    TRANSEC_STATE.write().unwrap().zeroize();

    // 4. Clear FPGA key registers
    if let Ok(fpga) = FPGA_HANDLE.lock() {
        unsafe {
            fpga.write_register(CRYPTO_KEY_BASE, 0);
            fpga.write_register(CRYPTO_KEY_BASE + 4, 0);
            // ... all key registers
        }
    }

    // 5. Memory barrier to ensure writes complete
    std::sync::atomic::fence(std::sync::atomic::Ordering::SeqCst);

    // 6. Log zeroization event (no sensitive data)
    log::warn!("Emergency key zeroization performed");
}

// Register signal handlers for emergency zeroization
fn setup_emergency_handlers() {
    use signal_hook::consts::*;

    let signals = [SIGTERM, SIGINT, SIGHUP, SIGUSR1];

    for sig in signals {
        signal_hook::flag::register(sig, std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)))
            .expect("Failed to register signal handler");
    }

    // USR1 triggers emergency zeroization
    unsafe {
        signal_hook::low_level::register(SIGUSR1, || {
            emergency_zeroize();
        }).expect("Failed to register SIGUSR1 handler");
    }
}
```

---

## Process Isolation

### Privilege Separation Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                    Privilege Separation                          │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌───────────────┐     ┌───────────────┐     ┌─────────────────┐ │
│  │ Control Proc  │     │  DSP Process  │     │ Crypto Process  │ │
│  │               │     │               │     │                 │ │
│  │ • CLI/API     │     │ • Modulation  │     │ • Key storage   │ │
│  │ • Config      │     │ • Demod       │     │ • Encryption    │ │
│  │ • Logging     │     │ • FFT/FIR     │     │ • Signing       │ │
│  │               │     │ • FPGA I/O    │     │                 │ │
│  │ User: r4w     │     │ User: r4w-dsp │     │ User: r4w-crypto│ │
│  │ Caps: none    │     │ Caps: sys_nice│     │ Caps: ipc_lock  │ │
│  └───────┬───────┘     └───────┬───────┘     └───────┬─────────┘ │
│          │                     │                     │           │
│          └──────────┬──────────┴──────────┬──────────┘           │
│                     │                     │                      │
│              ┌──────▼──────┐       ┌──────▼──────┐               │
│              │ Unix Socket │       │ Shared Mem  │               │
│              │ (control)   │       │ (samples)   │               │
│              └─────────────┘       └─────────────┘               │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

### Linux Namespaces

```rust
use nix::sched::{CloneFlags, unshare};
use nix::unistd::{setuid, setgid, Uid, Gid};

/// Create isolated DSP process with minimal privileges
pub fn spawn_isolated_dsp() -> Result<std::process::Child> {
    // Fork and isolate
    match unsafe { nix::unistd::fork() } {
        Ok(nix::unistd::ForkResult::Child) => {
            // New namespace for network, PID, mount
            unshare(
                CloneFlags::CLONE_NEWNET |   // Isolated network
                CloneFlags::CLONE_NEWPID |   // PID namespace
                CloneFlags::CLONE_NEWNS      // Mount namespace
            )?;

            // Drop to unprivileged user
            setgid(Gid::from_raw(DSP_GID))?;
            setuid(Uid::from_raw(DSP_UID))?;

            // Execute DSP process
            std::process::Command::new("/usr/lib/r4w/r4w-dsp")
                .exec();

            unreachable!()
        }
        Ok(nix::unistd::ForkResult::Parent { child }) => {
            Ok(std::process::Child::from_raw_handle(child.as_raw() as _))
        }
        Err(e) => Err(e.into()),
    }
}
```

### Capability Dropping

```rust
use caps::{CapSet, Capability};

/// Drop all capabilities except those needed
pub fn drop_capabilities(keep: &[Capability]) -> Result<()> {
    // Get current capabilities
    let current = caps::read(None, CapSet::Effective)?;

    // Drop all capabilities not in keep list
    for cap in current.iter() {
        if !keep.contains(&cap) {
            caps::drop(None, CapSet::Effective, cap)?;
            caps::drop(None, CapSet::Permitted, cap)?;
            caps::drop(None, CapSet::Inheritable, cap)?;
        }
    }

    // Lock capabilities (prevent regaining)
    prctl::set_securebits(
        prctl::Secbits::NOROOT |
        prctl::Secbits::NOROOT_LOCKED |
        prctl::Secbits::NO_SETUID_FIXUP |
        prctl::Secbits::NO_SETUID_FIXUP_LOCKED
    )?;

    Ok(())
}

/// DSP process capabilities
pub fn setup_dsp_capabilities() -> Result<()> {
    drop_capabilities(&[
        Capability::CAP_SYS_NICE,      // RT priority
        Capability::CAP_IPC_LOCK,      // mlock for real-time
    ])
}

/// Crypto process capabilities
pub fn setup_crypto_capabilities() -> Result<()> {
    drop_capabilities(&[
        Capability::CAP_IPC_LOCK,      // mlock for keys
    ])
}
```

### Seccomp Filtering

```rust
use seccompiler::{BpfMap, SeccompAction, SeccompFilter, SeccompRule};

/// Build seccomp filter for DSP process
pub fn build_dsp_seccomp_filter() -> SeccompFilter {
    SeccompFilter::new(
        vec![
            // Allow basic I/O
            (libc::SYS_read, vec![]),
            (libc::SYS_write, vec![]),
            (libc::SYS_close, vec![]),

            // Allow memory operations
            (libc::SYS_mmap, vec![]),
            (libc::SYS_munmap, vec![]),
            (libc::SYS_mprotect, vec![]),
            (libc::SYS_mlock, vec![]),

            // Allow shared memory
            (libc::SYS_shmat, vec![]),
            (libc::SYS_shmget, vec![]),
            (libc::SYS_shmdt, vec![]),

            // Allow FPGA memory mapping
            (libc::SYS_open, vec![
                // Only allow /dev/mem and /dev/uio*
                SeccompRule::new(vec![
                    SeccompCondition::new(0, SeccompCmpOp::Eq,
                        b"/dev/mem\0".as_ptr() as u64)?,
                ])?,
            ]),

            // Allow signals
            (libc::SYS_rt_sigaction, vec![]),
            (libc::SYS_rt_sigprocmask, vec![]),

            // Allow thread operations
            (libc::SYS_futex, vec![]),
            (libc::SYS_clone, vec![]),

            // Allow timing
            (libc::SYS_clock_gettime, vec![]),
            (libc::SYS_nanosleep, vec![]),

            // Required for exit
            (libc::SYS_exit, vec![]),
            (libc::SYS_exit_group, vec![]),
        ].into_iter().collect(),
        SeccompAction::Kill,  // Kill on disallowed syscall
        SeccompAction::Allow, // Allow listed syscalls
        std::env::consts::ARCH.try_into()?,
    )
}

/// Apply seccomp filter
pub fn apply_seccomp(filter: SeccompFilter) -> Result<()> {
    // Cannot remove filter once applied
    prctl::set_no_new_privs(true)?;

    seccompiler::apply_filter(&filter)?;

    log::info!("Seccomp filter applied");
    Ok(())
}
```

---

## Sandboxing

### Container-Based Isolation

```yaml
# docker-compose.security.yml
version: '3.8'

services:
  r4w-dsp:
    image: r4w/dsp:latest
    security_opt:
      - no-new-privileges:true
      - seccomp:seccomp-dsp.json
      - apparmor:r4w-dsp-profile
    cap_drop:
      - ALL
    cap_add:
      - SYS_NICE
      - IPC_LOCK
    read_only: true
    tmpfs:
      - /tmp:noexec,nosuid,size=100M
    volumes:
      - /dev/uio0:/dev/uio0:ro  # FPGA access
      - samples:/data/samples:rw
    mem_limit: 512m
    cpus: 2
    pids_limit: 100
    networks:
      - r4w-internal

  r4w-crypto:
    image: r4w/crypto:latest
    security_opt:
      - no-new-privileges:true
      - seccomp:seccomp-crypto.json
      - apparmor:r4w-crypto-profile
    cap_drop:
      - ALL
    cap_add:
      - IPC_LOCK
    read_only: true
    volumes:
      - keys:/data/keys:ro
      - /dev/tpm0:/dev/tpm0:rw  # TPM access
    mem_limit: 128m
    cpus: 1
    pids_limit: 50
    networks:
      - r4w-internal

  r4w-control:
    image: r4w/control:latest
    security_opt:
      - no-new-privileges:true
      - seccomp:seccomp-control.json
    cap_drop:
      - ALL
    read_only: true
    volumes:
      - config:/etc/r4w:ro
      - logs:/var/log/r4w:rw
    ports:
      - "127.0.0.1:8080:8080"  # Only localhost
    networks:
      - r4w-internal

networks:
  r4w-internal:
    driver: bridge
    internal: true

volumes:
  samples:
  keys:
  config:
  logs:
```

### AppArmor Profile

```bash
# /etc/apparmor.d/usr.lib.r4w.r4w-dsp
#include <tunables/global>

profile r4w-dsp /usr/lib/r4w/r4w-dsp flags=(attach_disconnected) {
  #include <abstractions/base>

  # Deny network access
  deny network,

  # Allow FPGA device access
  /dev/uio[0-9]* rw,
  /dev/mem r,
  /sys/class/uio/** r,

  # Allow shared memory
  /dev/shm/r4w-* rw,

  # Deny sensitive paths
  deny /etc/shadow r,
  deny /etc/passwd r,
  deny /root/** rwx,
  deny /home/*/.ssh/** rwx,

  # Read-only config
  /etc/r4w/** r,

  # Library access
  /usr/lib/** rm,
  /lib/** rm,

  # Temp files
  owner /tmp/r4w-* rw,

  # No execution of other programs
  deny /bin/** x,
  deny /usr/bin/** x,
  deny /sbin/** x,
}
```

### Systemd Sandboxing

```ini
# /etc/systemd/system/r4w-dsp.service
[Unit]
Description=R4W DSP Processor
After=network.target

[Service]
Type=simple
ExecStart=/usr/lib/r4w/r4w-dsp
User=r4w-dsp
Group=r4w-dsp

# Filesystem restrictions
ProtectSystem=strict
ProtectHome=true
PrivateTmp=true
PrivateDevices=false  # Need FPGA access
DeviceAllow=/dev/uio0 rw
DeviceAllow=/dev/mem r

# Network restrictions
PrivateNetwork=true

# Capability restrictions
CapabilityBoundingSet=CAP_SYS_NICE CAP_IPC_LOCK
AmbientCapabilities=CAP_SYS_NICE CAP_IPC_LOCK
NoNewPrivileges=true

# Seccomp
SystemCallFilter=@basic-io @memory-management @signal @process
SystemCallFilter=~@privileged @resources

# Resource limits
MemoryMax=512M
TasksMax=100
CPUQuota=200%

# Lock memory for real-time
LockPersonality=true
MemoryDenyWriteExecute=true

[Install]
WantedBy=multi-user.target
```

---

## Waveform Isolation

For environments requiring strict separation between waveforms (e.g., encrypted vs. unencrypted traffic, multi-tenant deployments, or multi-level security), R4W provides comprehensive isolation mechanisms.

### Isolation Levels Overview

| Level | Mechanism | Use Case |
|-------|-----------|----------|
| **L1** | Rust memory safety | Basic safety, development |
| **L2** | Linux namespaces | Multi-tenant, privilege separation |
| **L3** | Seccomp + SELinux/AppArmor | Defense contractors, government |
| **L4** | Containers (Docker/Podman) | Cloud deployment, easy management |
| **L5** | MicroVMs (Firecracker) | High assurance, rapid isolation |
| **L6** | Full VMs (KVM/QEMU) | Certification requirements |
| **L7** | Hardware isolation | FPGA partitioning, CPU pinning |
| **L8** | Air gap | Classified operations |

### Quick Start: r4w-sandbox

The `r4w-sandbox` crate provides a unified API for waveform isolation:

```rust
use r4w_sandbox::{Sandbox, IsolationLevel, WaveformConfig};

// Create isolated waveform sandbox
let sandbox = Sandbox::builder()
    .isolation_level(IsolationLevel::L3_LSM)
    .waveform("BPSK")
    .namespaces(Namespaces::PID | Namespaces::NET | Namespaces::MOUNT)
    .seccomp_profile(SeccompProfile::DSP)
    .memory_limit(512 * 1024 * 1024)
    .build()?;

// Execute waveform in isolated environment
let result = sandbox.run(|| {
    waveform.modulate(&data)
})?;
```

### Multi-Waveform Deployment

```yaml
# docker-compose.isolated.yml
services:
  waveform-encrypted:
    image: r4w/waveform:latest
    environment:
      - WAVEFORM=AES256-BPSK
      - CLASSIFICATION=SECRET
    security_opt:
      - seccomp:waveform-profile.json
      - apparmor:r4w-waveform
    networks:
      - encrypted-net

  waveform-unencrypted:
    image: r4w/waveform:latest
    environment:
      - WAVEFORM=BPSK
      - CLASSIFICATION=UNCLASSIFIED
    security_opt:
      - seccomp:waveform-profile.json
      - apparmor:r4w-waveform
    networks:
      - unencrypted-net

networks:
  encrypted-net:
    internal: true
  unencrypted-net:
    internal: true
```

### Detailed Documentation

For comprehensive coverage of isolation mechanisms including:
- Hardware isolation (CPU pinning, NUMA, Intel CAT, IOMMU)
- FPGA partitioning with AXI firewalls
- MicroVM and full VM configurations
- Cross-domain solutions for MLS environments
- Memory protection with encrypted buffers
- Data diode and air-gap architectures

See **[ISOLATION_GUIDE.md](./ISOLATION_GUIDE.md)** for the complete Waveform Isolation Guide.

---

## Network Security

### TLS Configuration

```rust
use rustls::{Certificate, PrivateKey, ServerConfig};
use std::sync::Arc;

/// Create TLS server configuration
pub fn create_tls_config(
    cert_path: &std::path::Path,
    key_path: &std::path::Path,
) -> Result<Arc<ServerConfig>> {
    // Load certificate chain
    let cert_file = std::fs::File::open(cert_path)?;
    let certs: Vec<Certificate> = rustls_pemfile::certs(&mut std::io::BufReader::new(cert_file))?
        .into_iter()
        .map(Certificate)
        .collect();

    // Load private key
    let key_file = std::fs::File::open(key_path)?;
    let key = rustls_pemfile::pkcs8_private_keys(&mut std::io::BufReader::new(key_file))?
        .into_iter()
        .next()
        .ok_or_else(|| R4wError::Config("no private key found".into()))?;

    // Build secure config
    let config = ServerConfig::builder()
        .with_safe_defaults()  // TLS 1.2+, secure ciphers only
        .with_no_client_auth()
        .with_single_cert(certs, PrivateKey(key))?;

    Ok(Arc::new(config))
}

/// Minimum TLS version
pub const MIN_TLS_VERSION: &str = "TLSv1.3";

/// Allowed cipher suites (in preference order)
pub const ALLOWED_CIPHERS: &[&str] = &[
    "TLS_AES_256_GCM_SHA384",
    "TLS_CHACHA20_POLY1305_SHA256",
    "TLS_AES_128_GCM_SHA256",
];
```

### Authenticated UDP Transport

```rust
use chacha20poly1305::{ChaCha20Poly1305, Key, Nonce};
use chacha20poly1305::aead::{Aead, NewAead};

/// Authenticated UDP packet format
///
/// ```text
/// +------------------+------------------+------------------+
/// | Nonce (12 bytes) | Ciphertext (var) | Tag (16 bytes)  |
/// +------------------+------------------+------------------+
/// ```
pub struct AuthenticatedUdpTransport {
    cipher: ChaCha20Poly1305,
    socket: std::net::UdpSocket,
    sequence: std::sync::atomic::AtomicU64,
}

impl AuthenticatedUdpTransport {
    pub fn new(key: &[u8; 32], bind_addr: &str) -> Result<Self> {
        let cipher = ChaCha20Poly1305::new(Key::from_slice(key));
        let socket = std::net::UdpSocket::bind(bind_addr)?;

        Ok(Self {
            cipher,
            socket,
            sequence: std::sync::atomic::AtomicU64::new(0),
        })
    }

    /// Send authenticated packet
    pub fn send(&self, data: &[u8], dest: &std::net::SocketAddr) -> Result<()> {
        // Generate unique nonce from sequence number
        let seq = self.sequence.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        let mut nonce_bytes = [0u8; 12];
        nonce_bytes[4..].copy_from_slice(&seq.to_le_bytes());
        let nonce = Nonce::from_slice(&nonce_bytes);

        // Encrypt and authenticate
        let ciphertext = self.cipher.encrypt(nonce, data)
            .map_err(|_| R4wError::CryptoError("encryption failed".into()))?;

        // Build packet
        let mut packet = Vec::with_capacity(12 + ciphertext.len());
        packet.extend_from_slice(&nonce_bytes);
        packet.extend_from_slice(&ciphertext);

        self.socket.send_to(&packet, dest)?;
        Ok(())
    }

    /// Receive and verify authenticated packet
    pub fn recv(&self, buf: &mut [u8]) -> Result<(usize, std::net::SocketAddr)> {
        let mut packet = vec![0u8; buf.len() + 12 + 16];
        let (len, addr) = self.socket.recv_from(&mut packet)?;

        if len < 12 + 16 {
            return Err(R4wError::CryptoError("packet too short".into()));
        }

        // Extract nonce and ciphertext
        let nonce = Nonce::from_slice(&packet[..12]);
        let ciphertext = &packet[12..len];

        // Decrypt and verify
        let plaintext = self.cipher.decrypt(nonce, ciphertext)
            .map_err(|_| R4wError::CryptoError("authentication failed".into()))?;

        let copy_len = plaintext.len().min(buf.len());
        buf[..copy_len].copy_from_slice(&plaintext[..copy_len]);

        Ok((copy_len, addr))
    }
}
```

### Firewall Rules

```bash
#!/bin/bash
# r4w-firewall.sh - Configure firewall for R4W deployment

# Flush existing rules
iptables -F
ip6tables -F

# Default deny
iptables -P INPUT DROP
iptables -P FORWARD DROP
iptables -P OUTPUT DROP

# Allow loopback
iptables -A INPUT -i lo -j ACCEPT
iptables -A OUTPUT -o lo -j ACCEPT

# Allow established connections
iptables -A INPUT -m state --state ESTABLISHED,RELATED -j ACCEPT
iptables -A OUTPUT -m state --state ESTABLISHED,RELATED -j ACCEPT

# R4W Control API (TLS only, from specific network)
iptables -A INPUT -p tcp --dport 8443 -s 192.168.1.0/24 -j ACCEPT

# R4W Sample Transport (authenticated UDP)
iptables -A INPUT -p udp --dport 5000:5100 -s 192.168.1.0/24 -j ACCEPT
iptables -A OUTPUT -p udp --sport 5000:5100 -d 192.168.1.0/24 -j ACCEPT

# NTP for time sync (required for TDMA waveforms)
iptables -A OUTPUT -p udp --dport 123 -j ACCEPT

# Log dropped packets
iptables -A INPUT -j LOG --log-prefix "R4W_DROP: " --log-level 4
iptables -A OUTPUT -j LOG --log-prefix "R4W_DROP: " --log-level 4

# Drop everything else
iptables -A INPUT -j DROP
iptables -A OUTPUT -j DROP

echo "Firewall configured for R4W"
```

---

## Secure Deployment

### Secure Boot Chain

```
┌─────────────────────────────────────────────────────────────────┐
│                      Secure Boot Chain                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  1. UEFI Secure Boot                                            │
│     └── Verify bootloader signature                             │
│                                                                 │
│  2. Verified Kernel                                             │
│     └── dm-verity root filesystem                               │
│                                                                 │
│  3. Measured Boot (TPM)                                         │
│     └── PCR measurements to TPM                                 │
│                                                                 │
│  4. Application Attestation                                     │
│     └── Verify R4W binary signature                             │
│                                                                 │
│  5. FPGA Bitstream                                              │
│     └── Authenticated bitstream loading                         │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Binary Signing

```bash
#!/bin/bash
# sign-release.sh - Sign R4W release binaries

# Sign binary with GPG
gpg --armor --detach-sign --local-user release@r4w.io target/release/r4w

# Generate SHA256 checksums
sha256sum target/release/r4w > target/release/r4w.sha256

# Sign checksum file
gpg --armor --detach-sign --local-user release@r4w.io target/release/r4w.sha256

# Create SBOM (Software Bill of Materials)
cargo sbom --output-format spdx-json > target/release/r4w.sbom.json
gpg --armor --detach-sign --local-user release@r4w.io target/release/r4w.sbom.json

echo "Release signed and checksummed"
```

### Verification Script

```bash
#!/bin/bash
# verify-release.sh - Verify R4W release integrity

set -e

RELEASE_DIR="${1:-.}"
GPG_KEY="release@r4w.io"

echo "Verifying R4W release..."

# Import release key
gpg --keyserver keys.openpgp.org --recv-keys "$GPG_KEY"

# Verify checksums
echo "Verifying checksum signature..."
gpg --verify "$RELEASE_DIR/r4w.sha256.asc" "$RELEASE_DIR/r4w.sha256"

echo "Verifying binary checksum..."
cd "$RELEASE_DIR"
sha256sum -c r4w.sha256
cd -

# Verify binary signature
echo "Verifying binary signature..."
gpg --verify "$RELEASE_DIR/r4w.asc" "$RELEASE_DIR/r4w"

# Verify SBOM
echo "Verifying SBOM signature..."
gpg --verify "$RELEASE_DIR/r4w.sbom.json.asc" "$RELEASE_DIR/r4w.sbom.json"

echo "All verifications passed!"
```

### Configuration Hardening

```yaml
# /etc/r4w/config.yaml
# Secure R4W configuration

security:
  # Require TLS for all network connections
  require_tls: true
  min_tls_version: "1.3"

  # Certificate verification
  verify_certificates: true
  ca_bundle: /etc/r4w/ca-bundle.crt

  # Authentication
  require_auth: true
  auth_method: certificate  # or 'psk', 'token'

  # Key management
  key_storage: tpm2  # or 'file', 'hsm'
  auto_zeroize: true
  key_rotation_days: 30

runtime:
  # Process isolation
  run_as_user: r4w
  drop_capabilities: true
  enable_seccomp: true

  # Resource limits
  max_memory_mb: 512
  max_cpu_percent: 200

  # Real-time settings
  enable_rt_scheduling: true
  mlockall: true

logging:
  # Security logging
  level: info
  format: json
  destination: syslog

  # Audit settings
  log_config_changes: true
  log_key_operations: true
  log_authentication: true

  # Never log sensitive data
  redact_keys: true
  redact_samples: false  # OK to log sample counts

network:
  # Bind to localhost only by default
  bind_address: "127.0.0.1"
  control_port: 8443

  # UDP transport
  sample_port_range: "5000-5100"
  authenticate_udp: true
```

---

## Audit Logging

### Structured Audit Log Format

```rust
use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};

/// Audit log event
#[derive(Debug, Serialize, Deserialize)]
pub struct AuditEvent {
    /// Event timestamp (UTC)
    pub timestamp: DateTime<Utc>,

    /// Event type
    pub event_type: AuditEventType,

    /// Event severity
    pub severity: AuditSeverity,

    /// Actor (user, process, or system)
    pub actor: AuditActor,

    /// Resource affected
    pub resource: String,

    /// Action taken
    pub action: String,

    /// Outcome
    pub outcome: AuditOutcome,

    /// Additional context (no sensitive data)
    pub context: std::collections::HashMap<String, String>,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuditEventType {
    Authentication,
    Authorization,
    KeyOperation,
    ConfigChange,
    WaveformOperation,
    SystemEvent,
    SecurityAlert,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuditSeverity {
    Info,
    Warning,
    Error,
    Critical,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct AuditActor {
    pub actor_type: String,  // "user", "process", "system"
    pub id: String,
    pub ip_address: Option<String>,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuditOutcome {
    Success,
    Failure,
    Partial,
}

impl AuditEvent {
    /// Log event to audit log
    pub fn log(&self) -> Result<()> {
        let json = serde_json::to_string(self)?;

        // Write to audit log (append-only)
        let mut file = std::fs::OpenOptions::new()
            .append(true)
            .create(true)
            .open("/var/log/r4w/audit.log")?;

        use std::io::Write;
        writeln!(file, "{}", json)?;

        // Also send to syslog for centralized logging
        syslog::log(syslog::Facility::LOG_AUTH, syslog::Severity::LOG_INFO, &json);

        Ok(())
    }
}

// Audit logging macros
#[macro_export]
macro_rules! audit_log {
    ($event_type:expr, $action:expr, $outcome:expr, $($key:expr => $value:expr),*) => {{
        let mut context = std::collections::HashMap::new();
        $(
            context.insert($key.to_string(), $value.to_string());
        )*

        let event = AuditEvent {
            timestamp: chrono::Utc::now(),
            event_type: $event_type,
            severity: AuditSeverity::Info,
            actor: get_current_actor(),
            resource: module_path!().to_string(),
            action: $action.to_string(),
            outcome: $outcome,
            context,
        };

        event.log()
    }};
}

// Example usage
fn load_key(key_id: &str) -> Result<CryptoKey> {
    let result = key_store.load(key_id);

    audit_log!(
        AuditEventType::KeyOperation,
        "load_key",
        if result.is_ok() { AuditOutcome::Success } else { AuditOutcome::Failure },
        "key_id" => key_id,
        "key_type" => "transec"
    )?;

    result
}
```

### Log Integrity

```rust
use sha2::{Sha256, Digest};

/// Audit log with hash chain for integrity
pub struct SecureAuditLog {
    file: std::fs::File,
    prev_hash: [u8; 32],
    sequence: u64,
}

impl SecureAuditLog {
    pub fn new(path: &std::path::Path) -> Result<Self> {
        let file = std::fs::OpenOptions::new()
            .append(true)
            .create(true)
            .open(path)?;

        // Initial hash (could be from TPM measurement)
        let prev_hash = [0u8; 32];

        Ok(Self {
            file,
            prev_hash,
            sequence: 0,
        })
    }

    /// Write log entry with hash chain
    pub fn write(&mut self, event: &AuditEvent) -> Result<()> {
        // Serialize event
        let event_json = serde_json::to_string(event)?;

        // Create hash chain entry
        let mut hasher = Sha256::new();
        hasher.update(&self.prev_hash);
        hasher.update(&self.sequence.to_le_bytes());
        hasher.update(event_json.as_bytes());
        let hash: [u8; 32] = hasher.finalize().into();

        // Write entry with hash
        let entry = serde_json::json!({
            "seq": self.sequence,
            "prev_hash": hex::encode(&self.prev_hash),
            "hash": hex::encode(&hash),
            "event": event,
        });

        use std::io::Write;
        writeln!(self.file, "{}", entry)?;
        self.file.sync_all()?;  // Ensure durability

        // Update state
        self.prev_hash = hash;
        self.sequence += 1;

        Ok(())
    }

    /// Verify log integrity
    pub fn verify(path: &std::path::Path) -> Result<bool> {
        let file = std::fs::File::open(path)?;
        let reader = std::io::BufReader::new(file);

        let mut prev_hash = [0u8; 32];
        let mut expected_seq = 0u64;

        use std::io::BufRead;
        for line in reader.lines() {
            let entry: serde_json::Value = serde_json::from_str(&line?)?;

            // Verify sequence
            let seq = entry["seq"].as_u64().unwrap();
            if seq != expected_seq {
                log::error!("Sequence mismatch: expected {}, got {}", expected_seq, seq);
                return Ok(false);
            }

            // Verify previous hash
            let stored_prev = hex::decode(entry["prev_hash"].as_str().unwrap())?;
            if stored_prev != prev_hash {
                log::error!("Previous hash mismatch at seq {}", seq);
                return Ok(false);
            }

            // Verify current hash
            let event_json = serde_json::to_string(&entry["event"])?;
            let mut hasher = Sha256::new();
            hasher.update(&prev_hash);
            hasher.update(&seq.to_le_bytes());
            hasher.update(event_json.as_bytes());
            let computed_hash: [u8; 32] = hasher.finalize().into();

            let stored_hash = hex::decode(entry["hash"].as_str().unwrap())?;
            if stored_hash != computed_hash {
                log::error!("Hash mismatch at seq {}", seq);
                return Ok(false);
            }

            prev_hash = computed_hash;
            expected_seq += 1;
        }

        log::info!("Verified {} log entries", expected_seq);
        Ok(true)
    }
}
```

---

## Side-Channel Considerations

### Timing Attacks

```rust
/// Constant-time comparison to prevent timing attacks
pub fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }

    // XOR all bytes, accumulate difference
    let mut diff = 0u8;
    for (x, y) in a.iter().zip(b.iter()) {
        diff |= x ^ y;
    }

    // Return true only if all bytes matched
    diff == 0
}

/// Constant-time table lookup
pub fn constant_time_lookup<T: Copy>(table: &[T], index: usize) -> T {
    // Touch all elements to avoid cache timing
    let mut result = table[0];

    for (i, &item) in table.iter().enumerate() {
        // Constant-time conditional select
        let select = constant_time_select(i == index);
        result = constant_time_conditional_select(result, item, select);
    }

    result
}

#[inline(never)]  // Prevent compiler optimizations
fn constant_time_select(condition: bool) -> u8 {
    (condition as u8).wrapping_neg()  // 0xFF if true, 0x00 if false
}

#[inline(never)]
fn constant_time_conditional_select<T: Copy>(a: T, b: T, select: u8) -> T {
    // This is a simplified version - real implementation needs
    // to work at byte level for proper constant-time behavior
    if select == 0xFF { b } else { a }
}
```

### Power Analysis Mitigation

```rust
/// Masked cryptographic operations
pub struct MaskedCrypto {
    /// Random mask for power analysis resistance
    mask: [u8; 32],
}

impl MaskedCrypto {
    pub fn new() -> Self {
        let mut mask = [0u8; 32];
        rand::thread_rng().fill_bytes(&mut mask);
        Self { mask }
    }

    /// Masked XOR operation
    pub fn masked_xor(&self, data: &mut [u8], key: &[u8]) {
        // Apply mask to key
        let masked_key: Vec<u8> = key.iter()
            .zip(self.mask.iter().cycle())
            .map(|(k, m)| k ^ m)
            .collect();

        // XOR with masked key
        for (d, mk) in data.iter_mut().zip(masked_key.iter().cycle()) {
            *d ^= mk;
        }

        // Remove mask
        for (d, m) in data.iter_mut().zip(self.mask.iter().cycle()) {
            *d ^= m;
        }
    }

    /// Refresh mask periodically
    pub fn refresh_mask(&mut self) {
        rand::thread_rng().fill_bytes(&mut self.mask);
    }
}
```

### Cache Attacks

```rust
/// Cache-resistant memory access patterns
pub mod cache_resistant {
    /// Prefetch all elements to fill cache
    pub fn prefetch_array<T>(array: &[T]) {
        for element in array.iter() {
            // Touch each element
            std::hint::black_box(element);
        }
    }

    /// Access memory in fixed pattern
    pub fn fixed_access_pattern<T: Copy>(array: &[T], indices: &[usize]) -> Vec<T> {
        // Always access all elements
        let mut results = Vec::with_capacity(indices.len());

        for &target_idx in indices {
            let mut result = array[0];

            // Touch all elements to hide access pattern
            for (i, &item) in array.iter().enumerate() {
                if i == target_idx {
                    result = item;
                }
                std::hint::black_box(&item);
            }

            results.push(result);
        }

        results
    }
}
```

---

## Physical Security

### Tamper Detection

```rust
/// Hardware tamper detection interface
pub trait TamperDetector: Send + Sync {
    /// Check if tamper event detected
    fn is_tampered(&self) -> bool;

    /// Get tamper event details
    fn get_tamper_events(&self) -> Vec<TamperEvent>;

    /// Clear tamper flag (requires authorization)
    fn clear_tamper(&mut self, auth: &Authorization) -> Result<()>;

    /// Arm tamper detection
    fn arm(&mut self) -> Result<()>;
}

#[derive(Debug, Clone)]
pub struct TamperEvent {
    pub timestamp: std::time::SystemTime,
    pub event_type: TamperEventType,
    pub sensor_id: u8,
}

#[derive(Debug, Clone)]
pub enum TamperEventType {
    CaseOpen,
    VoltageGlitch,
    TemperatureExceeded,
    ClockGlitch,
    ProbeDetected,
}

/// Tamper response actions
pub fn handle_tamper_event(event: &TamperEvent) {
    log::error!("TAMPER DETECTED: {:?}", event);

    // 1. Emergency key zeroization
    emergency_zeroize();

    // 2. Clear FPGA configuration
    if let Ok(fpga) = FPGA_HANDLE.lock() {
        fpga.clear_configuration();
    }

    // 3. Log event before shutdown
    audit_log!(
        AuditEventType::SecurityAlert,
        "tamper_detected",
        AuditOutcome::Failure,
        "event_type" => format!("{:?}", event.event_type),
        "sensor_id" => event.sensor_id.to_string()
    ).ok();

    // 4. Shutdown system
    std::process::exit(1);
}
```

### Environmental Monitoring

```rust
/// Environmental monitoring for physical security
pub struct EnvironmentalMonitor {
    temperature_range: (f32, f32),  // Celsius
    voltage_range: (f32, f32),      // Volts
    last_check: std::time::Instant,
}

impl EnvironmentalMonitor {
    pub fn new() -> Self {
        Self {
            temperature_range: (-20.0, 85.0),  // Industrial temp range
            voltage_range: (3.0, 3.6),         // 3.3V nominal
            last_check: std::time::Instant::now(),
        }
    }

    /// Check environmental parameters
    pub fn check(&mut self) -> Result<EnvironmentalStatus> {
        let temp = self.read_temperature()?;
        let voltage = self.read_voltage()?;

        let mut alerts = Vec::new();

        // Temperature check
        if temp < self.temperature_range.0 || temp > self.temperature_range.1 {
            alerts.push(EnvironmentalAlert::TemperatureOutOfRange(temp));
        }

        // Voltage check (could indicate glitching attack)
        if voltage < self.voltage_range.0 || voltage > self.voltage_range.1 {
            alerts.push(EnvironmentalAlert::VoltageOutOfRange(voltage));
        }

        // Rapid voltage changes (glitch detection)
        // Would require historical data

        self.last_check = std::time::Instant::now();

        Ok(EnvironmentalStatus {
            temperature: temp,
            voltage,
            alerts,
            timestamp: std::time::SystemTime::now(),
        })
    }

    fn read_temperature(&self) -> Result<f32> {
        // Read from hardware sensor
        // Platform-specific implementation
        todo!()
    }

    fn read_voltage(&self) -> Result<f32> {
        // Read from hardware sensor
        todo!()
    }
}
```

---

## Military and Classified Environments

### Classification Levels

```
┌─────────────────────────────────────────────────────────────────┐
│                    R4W Classification Architecture              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  UNCLASSIFIED                                                   │
│  ├── r4w-core (DSP, modulation, FEC)                            │
│  ├── r4w-sim (channel models, simulation)                       │
│  ├── r4w-gui (educational tools)                                │
│  └── r4w-cli (command interface)                                │
│                                                                 │
│  ─ ─ ─ ─ ─ ─ ─ ─ ─ Classification Boundary ─ ─ ─ ─ ─ ─ ─ ─ ─    │
│                                                                 │
│  CLASSIFIED (separate repository)                               │
│  ├── Hopping algorithms (SINCGARS, HAVEQUICK, Link-16)          │
│  ├── TRANSEC key generation                                     │
│  ├── COMSEC implementations                                     │
│  └── Operational configurations                                 │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Air-Gapped Build Process

```bash
#!/bin/bash
# classified-build.sh - Build process for classified components
# Run on air-gapped system only

set -e

# Verify air-gap (no network interfaces up except loopback)
INTERFACES=$(ip link show up | grep -v "lo:" | wc -l)
if [ "$INTERFACES" -gt 0 ]; then
    echo "ERROR: Network interfaces detected. This build must run air-gapped."
    exit 1
fi

# Verify build environment
echo "Checking build environment..."
rustc --version
cargo --version

# Verify source integrity
echo "Verifying source checksums..."
sha256sum -c CHECKSUMS.sha256

# Build with classified features
echo "Building with classified features..."
cargo build --release --features classified

# Generate build report
echo "Generating build report..."
cat > build-report.txt << EOF
Build Date: $(date -u +"%Y-%m-%d %H:%M:%S UTC")
Builder: $(whoami)@$(hostname)
Rust Version: $(rustc --version)
Cargo Version: $(cargo --version)
Git Commit: $(git rev-parse HEAD 2>/dev/null || echo "N/A")

Source Verification: PASSED
Build Status: SUCCESS

Binary Checksums:
$(sha256sum target/release/r4w-classified)
$(sha256sum target/release/libr4w_classified.a)
EOF

# Sign build artifacts
gpg --armor --sign build-report.txt

echo "Build complete. Transfer artifacts via approved methods."
```

### Secure Memory Handling

```rust
/// Memory region that is locked and zeroized
pub struct SecureMemory<T> {
    data: Box<[T]>,
    _guard: MlockGuard,
}

struct MlockGuard {
    ptr: *const u8,
    len: usize,
}

impl Drop for MlockGuard {
    fn drop(&mut self) {
        unsafe {
            // Unlock memory before freeing
            libc::munlock(self.ptr as *const libc::c_void, self.len);
        }
    }
}

impl<T: Default + Clone + Zeroize> SecureMemory<T> {
    /// Allocate secure, locked memory
    pub fn new(size: usize) -> Result<Self> {
        // Allocate
        let data: Box<[T]> = vec![T::default(); size].into_boxed_slice();

        // Lock in RAM (prevent swapping)
        let ptr = data.as_ptr() as *const u8;
        let len = std::mem::size_of_val(&*data);

        let result = unsafe { libc::mlock(ptr as *const libc::c_void, len) };
        if result != 0 {
            return Err(R4wError::Memory("mlock failed".into()));
        }

        // Advise kernel not to dump this memory
        unsafe {
            libc::madvise(
                ptr as *mut libc::c_void,
                len,
                libc::MADV_DONTDUMP
            );
        }

        Ok(Self {
            data,
            _guard: MlockGuard { ptr, len },
        })
    }
}

impl<T: Zeroize> Drop for SecureMemory<T> {
    fn drop(&mut self) {
        // Zeroize before drop
        for item in self.data.iter_mut() {
            item.zeroize();
        }
    }
}
```

### Cross-Domain Guard Interface

```rust
/// Interface for cross-domain (classification level) data transfer
pub trait CrossDomainGuard: Send + Sync {
    /// Validate data for transfer to lower classification
    fn validate_downgrade(&self, data: &[u8], from: Classification, to: Classification)
        -> Result<bool>;

    /// Log cross-domain transfer attempt
    fn log_transfer(&self, transfer: &TransferRecord) -> Result<()>;

    /// Get current guard policy
    fn get_policy(&self) -> &GuardPolicy;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Classification {
    Unclassified,
    Confidential,
    Secret,
    TopSecret,
}

#[derive(Debug)]
pub struct TransferRecord {
    pub timestamp: std::time::SystemTime,
    pub from_level: Classification,
    pub to_level: Classification,
    pub data_type: String,
    pub size_bytes: usize,
    pub approved: bool,
    pub approver: String,
}

/// Downgrade filter - removes classified content
pub fn downgrade_filter(data: &WaveformData, target: Classification) -> Result<WaveformData> {
    let mut filtered = data.clone();

    // Remove classified fields based on target level
    if target < Classification::Secret {
        filtered.hop_sequence = None;
        filtered.transec_params = None;
    }

    if target < Classification::Confidential {
        filtered.comsec_key_id = None;
        filtered.net_params = None;
    }

    Ok(filtered)
}
```

---

## Secure Development Practices

### Code Review Checklist

```markdown
## Security Code Review Checklist

### Memory Safety
- [ ] No unbounded allocations
- [ ] All unsafe blocks documented with safety invariants
- [ ] Sensitive data uses zeroization
- [ ] No panics in security-critical paths

### Input Validation
- [ ] All external input validated
- [ ] Bounds checked on array/slice access
- [ ] No integer overflow in size calculations
- [ ] Format string parameters sanitized

### Cryptography
- [ ] Using standard library crypto (not custom)
- [ ] Constant-time comparisons for secrets
- [ ] Proper random number generation
- [ ] Keys zeroized after use

### Authentication/Authorization
- [ ] Authentication required for sensitive operations
- [ ] Authorization checks before resource access
- [ ] Session handling is secure
- [ ] Error messages don't leak information

### Logging
- [ ] No sensitive data in logs
- [ ] Security events are logged
- [ ] Log injection prevented

### Network
- [ ] TLS required for network connections
- [ ] Certificate validation enabled
- [ ] Timeout limits set
- [ ] Rate limiting implemented
```

### Dependency Audit

```bash
#!/bin/bash
# audit-deps.sh - Audit dependencies for vulnerabilities

echo "Running cargo audit..."
cargo audit

echo "Checking for unmaintained dependencies..."
cargo audit --unmaintained

echo "Generating dependency tree..."
cargo tree > deps.txt

echo "Checking for known vulnerable crates..."
# Check against advisory database
cargo deny check advisories

echo "Checking licenses..."
cargo deny check licenses

echo "Generating SBOM..."
cargo sbom --output-format cyclonedx-json > sbom.json
```

### Fuzzing Setup

```rust
// fuzz/fuzz_targets/demodulate.rs
#![no_main]
use libfuzzer_sys::fuzz_target;
use r4w_core::waveform::lora::LoraWaveform;
use r4w_core::waveform::Waveform;

fuzz_target!(|data: &[u8]| {
    // Convert bytes to IQ samples
    if data.len() < 8 || data.len() % 4 != 0 {
        return;
    }

    let samples: Vec<IQSample> = data.chunks(4)
        .map(|c| {
            let i = i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0;
            let q = i16::from_le_bytes([c[2], c[3]]) as f32 / 32768.0;
            IQSample { i, q }
        })
        .collect();

    // Fuzz demodulation
    let waveform = LoraWaveform::new(7, 125_000).unwrap();
    let _ = waveform.demodulate(&samples);
});
```

```bash
# Run fuzzing
cargo +nightly fuzz run demodulate -- -max_len=65536 -timeout=10
```

---

## Incident Response

### Security Incident Procedure

```
┌─────────────────────────────────────────────────────────────────┐
│                  Security Incident Response                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  1. DETECTION                                                   │
│     • Monitor alerts                                            │
│     • User reports                                              │
│     • Audit log analysis                                        │
│                                                                 │
│  2. CONTAINMENT                                                 │
│     • Isolate affected systems                                  │
│     • Preserve evidence                                         │
│     • Emergency key zeroization if needed                       │
│                                                                 │
│  3. INVESTIGATION                                               │
│     • Analyze logs                                              │
│     • Determine scope                                           │
│     • Identify root cause                                       │
│                                                                 │
│  4. ERADICATION                                                 │
│     • Remove threat                                             │
│     • Patch vulnerabilities                                     │
│     • Update configurations                                     │
│                                                                 │
│  5. RECOVERY                                                    │
│     • Restore systems                                           │
│     • Rotate keys                                               │
│     • Verify integrity                                          │
│                                                                 │
│  6. LESSONS LEARNED                                             │
│     • Document incident                                         │
│     • Update procedures                                         │
│     • Training updates                                          │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Emergency Contacts

```yaml
# /etc/r4w/emergency-contacts.yaml
security_team:
  email: security@organization.com
  phone: "+1-xxx-xxx-xxxx"
  escalation_time: 15m

incident_response:
  email: ir@organization.com
  phone: "+1-xxx-xxx-xxxx"
  escalation_time: 30m

# For classified deployments
security_officer:
  name: "[FSO Name]"
  phone: "[FSO Phone]"

# For military deployments
comsec_custodian:
  name: "[COMSEC Custodian]"
  phone: "[Custodian Phone]"
```

---

## See Also

- [Isolation Guide](./ISOLATION_GUIDE.md) - Comprehensive waveform isolation (containers, VMs, hardware)
- [FPGA Developer's Guide](./FPGA_DEVELOPERS_GUIDE.md) - FPGA security considerations
- [Waveform Developer's Guide](./WAVEFORM_DEVELOPERS_GUIDE.md) - Secure waveform development
- [Military Porting Guide](./PORTING_GUIDE_MILITARY.md) - Classified component handling
- [Build Procedures](./porting/BUILD_PROCEDURES.md) - Secure build process

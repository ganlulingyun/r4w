# Build Procedures and Integration Guide

This guide covers the build procedures, linking strategies, and integration options for porting R4W waveforms to operational environments.

## Table of Contents

1. [Build Architecture Overview](#build-architecture-overview)
2. [Rust Library Types](#rust-library-types)
3. [Cargo-Based Builds](#cargo-based-builds)
4. [Non-Cargo Builds](#non-cargo-builds)
5. [C/C++ Interoperability](#cc-interoperability)
6. [Bridging Existing C/C++ Implementations](#bridging-existing-cc-implementations)
7. [Final Executable Options](#final-executable-options)
8. [Cross-Compilation](#cross-compilation)
9. [Secure Build Environments](#secure-build-environments)
10. [Verification and Testing](#verification-and-testing)

---

## Build Architecture Overview

R4W uses a trait-based architecture that allows classified components to be implemented separately and linked at build time:

```
┌──────────────────────────────────────────────────────────────────────┐
│                        Final Executable                              │
├──────────────────────────────────────────────────────────────────────┤
│  ┌──────────────────┐    ┌──────────────────┐    ┌─────────────────┐ │
│  │  Your Classified │    │   R4W Framework  │    │   Third-Party   │ │
│  │  Implementation  │    │    (r4w-core)    │    │    Libraries    │ │
│  │   (rlib/cdylib)  │    │     (rlib)       │    │   (rlib/sys)    │ │
│  └────────┬─────────┘    └────────┬─────────┘    └────────┬────────┘ │
│           │                       │                       │          │
│           └───────────────────────┴───────────────────────┘          │
│                                   │                                  │
│                            Static Linking                            │
│                           (Single Binary)                            │
└──────────────────────────────────────────────────────────────────────┘
```

### Key Design Principles

1. **Trait Interfaces**: Classified algorithms implement Rust traits defined in `r4w-core`
2. **Compile-Time Binding**: Implementations are selected via Cargo features or direct dependency
3. **No Runtime Loading**: Avoid dynamic loading of classified code for security
4. **Single Binary**: Prefer statically-linked executables for deployment

---

## Rust Library Types

### rlib (Rust Library) - Recommended

The default Rust library format. Contains Rust-specific metadata for optimization.

```toml
# Cargo.toml
[lib]
crate-type = ["rlib"]  # Default, can be omitted
```

**Advantages:**
- Full optimization across crate boundaries (LTO)
- Dead code elimination
- Smallest final binary size
- Best for pure Rust integration

**Use when:** Linking Rust code with other Rust code (most common case)

### staticlib (Static Library)

C-compatible static library (`.a` on Unix, `.lib` on Windows).

```toml
[lib]
crate-type = ["staticlib"]
```

**Advantages:**
- Links with C/C++ code
- No runtime dependencies
- Single binary deployment

**Use when:** Integrating with existing C/C++ codebases

### cdylib (C Dynamic Library)

C-compatible shared library (`.so` on Linux, `.dylib` on macOS, `.dll` on Windows).

```toml
[lib]
crate-type = ["cdylib"]
```

**Advantages:**
- Plugin architecture
- Shared across multiple executables
- Hot-swappable (with care)

**Use when:** Plugin systems or when shared libraries are required

### dylib (Rust Dynamic Library)

Rust-specific dynamic library. Rarely used in practice.

```toml
[lib]
crate-type = ["dylib"]
```

**Avoid unless:** You specifically need Rust-to-Rust dynamic linking

---

## Cargo-Based Builds

### Standard Build

```bash
# Development build
cargo build

# Release build (optimized)
cargo build --release

# With specific features
cargo build --release --features "classified"
```

### Feature-Based Component Selection

Configure which implementations to use via Cargo features:

```toml
# Cargo.toml
[features]
default = ["simulator"]
simulator = []           # Unclassified simulator stubs
classified = []          # Real classified implementations

[dependencies]
r4w-core = { path = "../r4w-core" }

# Conditional dependencies based on features
[target.'cfg(feature = "classified")'.dependencies]
classified-hopper = { path = "../classified-hopper" }
```

### Workspace Configuration

For multi-crate projects:

```toml
# Root Cargo.toml
[workspace]
members = [
    "crates/r4w-core",
    "crates/r4w-sim",
    "crates/classified-impl",  # Your classified crate
    "crates/final-app",
]

[workspace.dependencies]
r4w-core = { path = "crates/r4w-core" }
```

### Build Scripts

For complex builds, use `build.rs`:

```rust
// build.rs
fn main() {
    // Link with external C library
    println!("cargo:rustc-link-lib=static=crypto_impl");
    println!("cargo:rustc-link-search=native=/path/to/lib");

    // Rebuild if these change
    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-env-changed=CRYPTO_LIB_PATH");
}
```

---

## Non-Cargo Builds

For secure environments where Cargo cannot access the internet, or for integration with existing build systems.

### Vendoring Dependencies

```bash
# Create vendor directory with all dependencies
cargo vendor

# This creates .cargo/config.toml automatically:
# [source.crates-io]
# replace-with = "vendored-sources"
# [source.vendored-sources]
# directory = "vendor"
```

### Direct rustc Invocation

For build systems that can't use Cargo:

```bash
# Compile a single crate
rustc --edition 2021 \
    --crate-type rlib \
    --crate-name classified_hopper \
    -L dependency=./deps \
    --extern r4w_core=./deps/libr4w_core.rlib \
    src/lib.rs \
    -o libclassified_hopper.rlib

# Link final executable
rustc --edition 2021 \
    --crate-type bin \
    -L dependency=./deps \
    --extern r4w_core=./deps/libr4w_core.rlib \
    --extern classified_hopper=./deps/libclassified_hopper.rlib \
    src/main.rs \
    -o waveform_app
```

### Makefile Integration

```makefile
# Makefile for non-Cargo builds
RUSTC = rustc
RUSTFLAGS = --edition 2021 -C opt-level=3 -C lto

DEPS_DIR = ./target/deps
OUT_DIR = ./target/release

# Build r4w-core
$(DEPS_DIR)/libr4w_core.rlib: crates/r4w-core/src/lib.rs
	$(RUSTC) $(RUSTFLAGS) --crate-type rlib --crate-name r4w_core \
		-L $(DEPS_DIR) $< -o $@

# Build classified implementation
$(DEPS_DIR)/libclassified.rlib: crates/classified/src/lib.rs $(DEPS_DIR)/libr4w_core.rlib
	$(RUSTC) $(RUSTFLAGS) --crate-type rlib --crate-name classified \
		-L $(DEPS_DIR) --extern r4w_core=$(DEPS_DIR)/libr4w_core.rlib \
		$< -o $@

# Final executable
$(OUT_DIR)/waveform: src/main.rs $(DEPS_DIR)/libr4w_core.rlib $(DEPS_DIR)/libclassified.rlib
	$(RUSTC) $(RUSTFLAGS) --crate-type bin \
		-L $(DEPS_DIR) \
		--extern r4w_core=$(DEPS_DIR)/libr4w_core.rlib \
		--extern classified=$(DEPS_DIR)/libclassified.rlib \
		$< -o $@
```

### CMake Integration

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.15)
project(waveform_app)

# Find Rust toolchain
find_program(CARGO cargo REQUIRED)
find_program(RUSTC rustc REQUIRED)

# Custom target for Rust build
add_custom_target(rust_libs ALL
    COMMAND ${CARGO} build --release --manifest-path ${CMAKE_SOURCE_DIR}/Cargo.toml
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Building Rust libraries"
)

# Link Rust static library with C++ code
add_executable(waveform_app main.cpp)
target_link_libraries(waveform_app
    ${CMAKE_SOURCE_DIR}/target/release/libr4w_core.a
    ${CMAKE_SOURCE_DIR}/target/release/libclassified.a
    pthread dl m  # Common Rust runtime dependencies
)
add_dependencies(waveform_app rust_libs)
```

---

## C/C++ Interoperability

### Exposing Rust to C/C++

Create C-compatible functions using `extern "C"`:

```rust
// src/ffi.rs
use std::ffi::{c_char, c_int, CStr};
use std::ptr;

use crate::waveform::sincgars::SincgarsWaveform;

/// Opaque handle for C code
pub struct SincgarsHandle {
    inner: Box<SincgarsWaveform>,
}

/// Create a new SINCGARS waveform instance
/// Returns NULL on failure
#[no_mangle]
pub extern "C" fn sincgars_new(sample_rate: f64) -> *mut SincgarsHandle {
    match SincgarsWaveform::new(sample_rate) {
        Ok(waveform) => Box::into_raw(Box::new(SincgarsHandle {
            inner: Box::new(waveform),
        })),
        Err(_) => ptr::null_mut(),
    }
}

/// Free a SINCGARS waveform instance
#[no_mangle]
pub extern "C" fn sincgars_free(handle: *mut SincgarsHandle) {
    if !handle.is_null() {
        unsafe {
            drop(Box::from_raw(handle));
        }
    }
}

/// Modulate data
/// Returns number of samples written, or -1 on error
#[no_mangle]
pub extern "C" fn sincgars_modulate(
    handle: *mut SincgarsHandle,
    data: *const u8,
    data_len: usize,
    samples_out: *mut f32,
    samples_capacity: usize,
) -> c_int {
    if handle.is_null() || data.is_null() || samples_out.is_null() {
        return -1;
    }

    unsafe {
        let waveform = &(*handle).inner;
        let input = std::slice::from_raw_parts(data, data_len);
        let output = std::slice::from_raw_parts_mut(samples_out, samples_capacity);

        match waveform.modulate_to_buffer(input, output) {
            Ok(count) => count as c_int,
            Err(_) => -1,
        }
    }
}

/// Set hopping algorithm (for classified implementations)
#[no_mangle]
pub extern "C" fn sincgars_set_hopper(
    handle: *mut SincgarsHandle,
    hopper_ptr: *mut std::ffi::c_void,
    hopper_vtable: *const HopperVTable,
) -> c_int {
    // Bridge to C hopper implementation
    // See "Bridging Existing C/C++ Implementations" section
    0
}
```

### C Header Generation

Use `cbindgen` to automatically generate C headers:

```toml
# cbindgen.toml
language = "C"
header = "/* Auto-generated, do not edit */"
include_guard = "R4W_SINCGARS_H"
autogen_warning = "/* Warning: auto-generated by cbindgen. Do not modify. */"

[export]
include = ["SincgarsHandle"]

[fn]
prefix = "R4W_"
```

```bash
# Generate header
cbindgen --config cbindgen.toml --output include/r4w_sincgars.h
```

Generated header:

```c
/* r4w_sincgars.h - Auto-generated */
#ifndef R4W_SINCGARS_H
#define R4W_SINCGARS_H

#include <stdint.h>
#include <stddef.h>

typedef struct SincgarsHandle SincgarsHandle;

SincgarsHandle* R4W_sincgars_new(double sample_rate);
void R4W_sincgars_free(SincgarsHandle* handle);
int R4W_sincgars_modulate(
    SincgarsHandle* handle,
    const uint8_t* data,
    size_t data_len,
    float* samples_out,
    size_t samples_capacity
);

#endif /* R4W_SINCGARS_H */
```

### C++ Wrapper Class

```cpp
// r4w_sincgars.hpp
#pragma once
#include <memory>
#include <vector>
#include <stdexcept>

extern "C" {
#include "r4w_sincgars.h"
}

namespace r4w {

class Sincgars {
public:
    explicit Sincgars(double sample_rate)
        : handle_(R4W_sincgars_new(sample_rate), &R4W_sincgars_free)
    {
        if (!handle_) {
            throw std::runtime_error("Failed to create SINCGARS waveform");
        }
    }

    std::vector<float> modulate(const std::vector<uint8_t>& data) {
        // Estimate output size (conservative)
        std::vector<float> samples(data.size() * 100);

        int result = R4W_sincgars_modulate(
            handle_.get(),
            data.data(),
            data.size(),
            samples.data(),
            samples.size()
        );

        if (result < 0) {
            throw std::runtime_error("Modulation failed");
        }

        samples.resize(result);
        return samples;
    }

private:
    std::unique_ptr<SincgarsHandle, decltype(&R4W_sincgars_free)> handle_;
};

} // namespace r4w
```

---

## Bridging Existing C/C++ Implementations

When you have existing C/C++ classified implementations, you can bridge them into the Rust trait system.

### Method 1: Trait Objects with C Function Pointers

Define a vtable structure that C code can populate:

```rust
// src/ffi/hopper_bridge.rs
use crate::waveform::sincgars::traits::{HoppingAlgorithm, ChannelNumber, TransecKey, NetId, SincgarsTime};

/// C-compatible vtable for hopping algorithm
#[repr(C)]
pub struct HopperVTable {
    pub initialize: unsafe extern "C" fn(
        ctx: *mut std::ffi::c_void,
        key: *const u8,
        key_len: usize,
        net_id: u32,
        time_secs: u64,
    ),
    pub get_current_channel: unsafe extern "C" fn(ctx: *mut std::ffi::c_void) -> u16,
    pub advance_hop: unsafe extern "C" fn(ctx: *mut std::ffi::c_void),
    pub reset: unsafe extern "C" fn(ctx: *mut std::ffi::c_void),
    pub destroy: unsafe extern "C" fn(ctx: *mut std::ffi::c_void),
}

/// Rust wrapper around C hopper implementation
pub struct CHopper {
    ctx: *mut std::ffi::c_void,
    vtable: HopperVTable,
}

// Safety: The C implementation must be thread-safe
unsafe impl Send for CHopper {}
unsafe impl Sync for CHopper {}

impl CHopper {
    /// Create from C function pointers
    ///
    /// # Safety
    /// Caller must ensure vtable functions are valid and ctx is properly initialized
    pub unsafe fn from_c(ctx: *mut std::ffi::c_void, vtable: HopperVTable) -> Self {
        Self { ctx, vtable }
    }
}

impl HoppingAlgorithm for CHopper {
    fn initialize(&mut self, key: &TransecKey, net_id: NetId, time: SincgarsTime) {
        unsafe {
            (self.vtable.initialize)(
                self.ctx,
                key.as_ptr(),
                key.len(),
                net_id.0,
                time.as_secs(),
            );
        }
    }

    fn get_current_channel(&self) -> ChannelNumber {
        unsafe {
            ChannelNumber((self.vtable.get_current_channel)(self.ctx))
        }
    }

    fn advance_hop(&mut self) {
        unsafe {
            (self.vtable.advance_hop)(self.ctx);
        }
    }

    fn reset(&mut self) {
        unsafe {
            (self.vtable.reset)(self.ctx);
        }
    }

    // ... implement other trait methods
}

impl Drop for CHopper {
    fn drop(&mut self) {
        unsafe {
            (self.vtable.destroy)(self.ctx);
        }
    }
}
```

### C Implementation Side

```c
// classified_hopper.c
#include <stdlib.h>
#include <stdint.h>

typedef struct {
    uint8_t key[32];
    uint32_t net_id;
    uint64_t hop_count;
    // ... classified state
} HopperState;

static void hopper_initialize(void* ctx, const uint8_t* key, size_t key_len,
                               uint32_t net_id, uint64_t time_secs) {
    HopperState* state = (HopperState*)ctx;
    memcpy(state->key, key, key_len < 32 ? key_len : 32);
    state->net_id = net_id;
    state->hop_count = time_secs / 10;  // Example: 10-second hops
    // ... classified initialization
}

static uint16_t hopper_get_channel(void* ctx) {
    HopperState* state = (HopperState*)ctx;
    // ... classified algorithm
    return (uint16_t)(state->hop_count % 2320);  // SINCGARS has 2320 channels
}

static void hopper_advance(void* ctx) {
    HopperState* state = (HopperState*)ctx;
    state->hop_count++;
    // ... classified advancement
}

static void hopper_reset(void* ctx) {
    HopperState* state = (HopperState*)ctx;
    state->hop_count = 0;
}

static void hopper_destroy(void* ctx) {
    HopperState* state = (HopperState*)ctx;
    // Secure zeroization
    memset(state, 0, sizeof(HopperState));
    free(state);
}

// Create vtable for Rust
HopperVTable create_hopper_vtable(void) {
    return (HopperVTable){
        .initialize = hopper_initialize,
        .get_current_channel = hopper_get_channel,
        .advance_hop = hopper_advance,
        .reset = hopper_reset,
        .destroy = hopper_destroy,
    };
}

void* create_hopper_context(void) {
    return calloc(1, sizeof(HopperState));
}
```

### Method 2: bindgen for Existing C Headers

If you have existing C headers, use `bindgen` to generate Rust bindings:

```toml
# Cargo.toml
[build-dependencies]
bindgen = "0.69"
```

```rust
// build.rs
use std::env;
use std::path::PathBuf;

fn main() {
    // Tell cargo to link the C library
    println!("cargo:rustc-link-lib=static=classified_crypto");
    println!("cargo:rustc-link-search=native=/path/to/classified/lib");

    // Generate bindings
    let bindings = bindgen::Builder::default()
        .header("wrapper.h")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        // Only generate bindings for these functions
        .allowlist_function("crypto_.*")
        .allowlist_type("CryptoContext")
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
```

```rust
// src/lib.rs
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
```

### Method 3: cxx for C++ Interop

For C++ code, `cxx` provides safer interop:

```toml
# Cargo.toml
[dependencies]
cxx = "1.0"

[build-dependencies]
cxx-build = "1.0"
```

```rust
// src/lib.rs
#[cxx::bridge]
mod ffi {
    unsafe extern "C++" {
        include!("classified/hopper.hpp");

        type ClassifiedHopper;

        fn create_hopper() -> UniquePtr<ClassifiedHopper>;
        fn initialize(self: Pin<&mut ClassifiedHopper>, key: &[u8], net_id: u32);
        fn get_channel(self: &ClassifiedHopper) -> u16;
        fn advance(self: Pin<&mut ClassifiedHopper>);
    }
}

// Wrapper implementing Rust trait
pub struct CxxHopper {
    inner: cxx::UniquePtr<ffi::ClassifiedHopper>,
}

impl HoppingAlgorithm for CxxHopper {
    fn get_current_channel(&self) -> ChannelNumber {
        ChannelNumber(self.inner.get_channel())
    }
    // ... etc
}
```

---

## Final Executable Options

### Option 1: Single Static Binary (Recommended)

Most secure, no external dependencies:

```toml
# Cargo.toml
[profile.release]
lto = true           # Link-time optimization
codegen-units = 1    # Better optimization
strip = true         # Remove symbols
panic = "abort"      # Smaller binary
```

```bash
# Build
cargo build --release

# Result: single statically-linked executable
file target/release/waveform_app
# waveform_app: ELF 64-bit LSB pie executable, x86-64, statically linked
```

### Option 2: Shared Library Plugin

For systems requiring dynamic loading:

```toml
# Cargo.toml
[lib]
crate-type = ["cdylib"]
name = "r4w_sincgars"
```

```bash
# Build shared library
cargo build --release

# Result: libr4w_sincgars.so
```

### Option 3: Static Library for C/C++ Integration

```toml
# Cargo.toml
[lib]
crate-type = ["staticlib"]
name = "r4w_sincgars"
```

```bash
# Build
cargo build --release

# Link with C++ application
g++ -o app main.cpp \
    -L target/release -l:libr4w_sincgars.a \
    -lpthread -ldl -lm
```

### Option 4: WebAssembly for Simulation/Training

```bash
# Install wasm target
rustup target add wasm32-unknown-unknown

# Build
cargo build --release --target wasm32-unknown-unknown

# Result: waveform.wasm
```

---

## Cross-Compilation

### Target Triples

Common targets for SDR platforms:

| Target | Description |
|--------|-------------|
| `x86_64-unknown-linux-gnu` | Standard Linux x64 |
| `x86_64-unknown-linux-musl` | Static Linux x64 (no glibc) |
| `aarch64-unknown-linux-gnu` | ARM64 Linux (Raspberry Pi 4, Jetson) |
| `armv7-unknown-linux-gnueabihf` | ARM32 Linux (Raspberry Pi 3) |
| `arm-unknown-linux-gnueabi` | ARM32 soft-float |
| `powerpc-unknown-linux-gnu` | PowerPC (some embedded) |

### Cross-Compilation Setup

```bash
# Install target
rustup target add aarch64-unknown-linux-gnu

# Install cross-compiler
sudo apt install gcc-aarch64-linux-gnu

# Configure linker
mkdir -p .cargo
cat > .cargo/config.toml << 'EOF'
[target.aarch64-unknown-linux-gnu]
linker = "aarch64-linux-gnu-gcc"
rustflags = ["-C", "target-feature=+crt-static"]
EOF

# Build
cargo build --release --target aarch64-unknown-linux-gnu
```

### Using cross Tool

Easier cross-compilation using Docker:

```bash
# Install cross
cargo install cross

# Build for ARM64
cross build --release --target aarch64-unknown-linux-gnu

# Build for ARM32
cross build --release --target armv7-unknown-linux-gnueabihf
```

### Custom Cross Configuration

```toml
# Cross.toml
[target.aarch64-unknown-linux-gnu]
image = "ghcr.io/cross-rs/aarch64-unknown-linux-gnu:main"

[target.aarch64-unknown-linux-gnu.env]
passthrough = [
    "CLASSIFIED_LIB_PATH",
    "CRYPTO_KEY_FILE",
]

# Mount classified library directory
[target.aarch64-unknown-linux-gnu.pre-build]
script = """
mkdir -p /classified
cp -r $CLASSIFIED_LIB_PATH/* /classified/
"""
```

### Cross-Linking with C Libraries

When linking with C libraries for a different target:

```rust
// build.rs
fn main() {
    let target = std::env::var("TARGET").unwrap();

    let lib_path = match target.as_str() {
        "aarch64-unknown-linux-gnu" => "/opt/classified/lib/aarch64",
        "armv7-unknown-linux-gnueabihf" => "/opt/classified/lib/armv7hf",
        "x86_64-unknown-linux-gnu" => "/opt/classified/lib/x86_64",
        _ => panic!("Unsupported target: {}", target),
    };

    println!("cargo:rustc-link-search=native={}", lib_path);
    println!("cargo:rustc-link-lib=static=classified_crypto");
}
```

---

## Secure Build Environments

### Air-Gapped Build System

For classified development:

```bash
# 1. On internet-connected machine: vendor all dependencies
cargo vendor --versioned-dirs
tar czf vendor.tar.gz vendor .cargo/config.toml

# 2. Transfer vendor.tar.gz to air-gapped system (via approved media)

# 3. On air-gapped system:
tar xzf vendor.tar.gz
cargo build --release --offline
```

### Reproducible Builds

Ensure builds are reproducible for verification:

```toml
# Cargo.toml
[profile.release]
lto = true
codegen-units = 1

# Cargo.lock should be committed
```

```bash
# Lock Rust version
rustup override set 1.75.0

# Build with locked dependencies
cargo build --release --locked

# Verify hash
sha256sum target/release/waveform_app
```

### Build Environment Checklist

For classified builds:

- [ ] Air-gapped or approved network
- [ ] Verified Rust toolchain (signed, checksummed)
- [ ] Vendored dependencies (no network access)
- [ ] Locked dependency versions (`Cargo.lock`)
- [ ] Reproducible build settings
- [ ] Build system documentation
- [ ] Output verification (checksums)
- [ ] Secure storage of build artifacts

---

## Verification and Testing

### Unit Testing Classified Components

Test implementations against known test vectors:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    // Test vector from classified documentation
    // (This example uses unclassified placeholder values)
    #[test]
    fn test_hopper_sequence() {
        let mut hopper = ClassifiedHopper::new();

        let test_key = TransecKey::from_bytes(&[0x01; 32]);
        let net_id = NetId(1234);
        let start_time = SincgarsTime::from_secs(1000000);

        hopper.initialize(&test_key, net_id, start_time);

        // Expected channel sequence (from test vectors)
        let expected = [1423, 892, 2105, 341, 1867];

        for &expected_channel in &expected {
            assert_eq!(hopper.get_current_channel().0, expected_channel);
            hopper.advance_hop();
        }
    }
}
```

### Integration Testing

```rust
// tests/integration.rs
use r4w_core::waveform::sincgars::SincgarsBuilder;

#[test]
fn test_full_tx_rx_chain() {
    let tx = SincgarsBuilder::new()
        .sample_rate(48000.0)
        .with_classified_hopper(create_test_hopper())
        .build()
        .unwrap();

    let rx = SincgarsBuilder::new()
        .sample_rate(48000.0)
        .with_classified_hopper(create_test_hopper())
        .build()
        .unwrap();

    let test_data = b"Test message";
    let modulated = tx.modulate(test_data);
    let demodulated = rx.demodulate(&modulated);

    assert_eq!(&demodulated, test_data);
}
```

### Interoperability Testing

```rust
#[test]
fn test_interop_with_reference_implementation() {
    // Load known-good signal from reference implementation
    let reference_signal = load_test_vector("sincgars_reference.bin");

    let rx = SincgarsBuilder::new()
        .sample_rate(48000.0)
        .with_classified_hopper(create_production_hopper())
        .build()
        .unwrap();

    let decoded = rx.demodulate(&reference_signal);

    assert_eq!(decoded, b"REFERENCE TEST MESSAGE");
}
```

---

## Next Steps

See the waveform-specific porting guides for detailed implementation requirements:

- [SINCGARS Porting Guide](./sincgars.md)
- [HAVEQUICK Porting Guide](./havequick.md)
- [Link-16 Porting Guide](./link16.md)
- [MIL-STD-188-110 Porting Guide](./milstd188110.md)
- [P25 Porting Guide](./p25.md)

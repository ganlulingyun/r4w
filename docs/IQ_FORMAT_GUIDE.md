# IQ Sample Format Guide

This guide covers the unified `IqFormat` module in R4W, which provides a single source of truth for IQ sample format handling across the codebase.

## Overview

The `IqFormat` enum in `r4w_core::io` handles all IQ sample formats used in:
- File I/O (SigMF recordings)
- UDP streaming (GNU Radio, SDR hardware)
- GNSS scenario generation
- Benchmark tools

## Supported Formats

| Format | Bytes/Sample | SigMF Type | Description |
|--------|--------------|------------|-------------|
| `Cf64` | 16 | `cf64_le` | Complex float64, maximum precision |
| `Cf32` | 8 | `cf32_le` | Complex float32, USRP/GNU Radio compatible (default) |
| `Ci16` | 4 | `ci16_le` | Complex signed int16, compact storage |
| `Ci8` | 2 | `ci8` | Complex signed int8, very compact |
| `Cu8` | 2 | `cu8` | Complex unsigned int8, RTL-SDR native format |

### Format Aliases

The `IqFormat::from_str()` method accepts many common aliases:

| Format | Accepted Aliases |
|--------|------------------|
| Cf64 | `f64`, `cf64`, `cf64_le`, `complex64` |
| Cf32 | `f32`, `cf32`, `cf32_le`, `ettus`, `float32`, `float` |
| Ci16 | `i16`, `ci16`, `ci16_le`, `sc16`, `int16`, `short` |
| Ci8 | `i8`, `ci8`, `int8` |
| Cu8 | `u8`, `cu8`, `uint8`, `rtlsdr` |

## Scaling Conventions

IQ samples are normalized to the range `[-1.0, 1.0]` in the internal representation (`IQSample` is `Complex<f64>`).

### Integer Formats

| Format | Write Scaling | Read Scaling | Range |
|--------|--------------|--------------|-------|
| Ci16 | `* 32767.0` clamped | `/ 32768.0` | -32768 to 32767 |
| Ci8 | `* 127.0` clamped | `/ 128.0` | -128 to 127 |
| Cu8 | `(+1.0) * 127.5` | `(- 127.5) / 127.5` | 0 to 255 |

The Cu8 format uses offset binary encoding (DC at 127.5), which is the native format for RTL-SDR devices.

## API Usage

### Rust API

```rust
use r4w_core::io::IqFormat;
use r4w_core::types::IQSample;

// Parse format from CLI argument
let format = IqFormat::from_str("ettus").unwrap();
assert_eq!(format, IqFormat::Cf32);

// Check properties
println!("Bytes per sample: {}", format.bytes_per_sample());
println!("SigMF datatype: {}", format.sigmf_datatype());
println!("Display name: {}", format.display_name());

// Write samples to file
let samples = vec![IQSample::new(0.5, -0.5), IQSample::new(-1.0, 1.0)];
let mut file = std::fs::File::create("output.iq").unwrap();
format.write_samples(&mut file, &samples).unwrap();

// Read samples from file
let mut file = std::fs::File::open("output.iq").unwrap();
let decoded = format.read_samples(&mut file, 2).unwrap();

// Byte-level operations (for UDP packets, memory-mapped files)
let bytes = format.to_bytes(&samples);
let parsed = format.parse_bytes(&bytes);
```

### CLI Usage

#### GNSS Scenario Generation

```bash
# Default format (cf64)
cargo run --bin r4w -- gnss scenario --preset open-sky --output signal.iq

# USRP/Ettus compatible (cf32)
cargo run --bin r4w -- gnss scenario --preset open-sky --format ettus --output usrp.iq

# Compact storage (ci16)
cargo run --bin r4w -- gnss scenario --preset open-sky --format sc16 --output compact.iq

# Very compact (ci8)
cargo run --bin r4w -- gnss scenario --preset open-sky --format ci8 --output tiny.iq

# RTL-SDR compatible (cu8)
cargo run --bin r4w -- gnss scenario --preset open-sky --format rtlsdr --output rtlsdr.iq
```

## SigMF Integration

The `IqFormat` integrates seamlessly with SigMF (Signal Metadata Format):

```rust
use r4w_sim::hal::sigmf::{SigMfWriter, SigMfReader};

// Write SigMF recording (cf32 by default)
let mut writer = SigMfWriter::create("recording", 1e6, 915e6)?;
writer.write_samples(&samples)?;
writer.close()?;

// Write SigMF with specific format
let mut writer = SigMfWriter::create_with_format(
    "recording",
    1e6,      // sample rate
    915e6,    // center frequency
    "ci16_le" // datatype
)?;
```

The SigMF reader automatically detects the format from metadata:

```rust
let mut reader = SigMfReader::open("recording.sigmf-meta")?;
println!("Format: {}", reader.metadata().global.datatype);
let samples = reader.read_all()?;
```

## File Size Comparison

For 1 million samples:

| Format | File Size | Relative |
|--------|-----------|----------|
| Cf64 | 16 MB | 8x |
| Cf32 | 8 MB | 4x |
| Ci16 | 4 MB | 2x |
| Ci8 | 2 MB | 1x (baseline) |
| Cu8 | 2 MB | 1x |

## Precision vs Size Trade-offs

| Format | Dynamic Range | SNR Floor | Use Case |
|--------|--------------|-----------|----------|
| Cf64 | ~300 dB | Unlimited | Scientific analysis, accumulation |
| Cf32 | ~150 dB | ~-144 dBFS | General SDR, GNU Radio |
| Ci16 | ~96 dB | ~-90 dBFS | Hardware streaming, archives |
| Ci8 | ~48 dB | ~-42 dBFS | Low-bandwidth storage |
| Cu8 | ~48 dB | ~-42 dBFS | RTL-SDR compatibility |

## Interoperability

### GNU Radio

R4W's `cf32_le` format is directly compatible with GNU Radio's default file sink. Use the SigMF metadata for full compatibility:

```bash
# Generate file compatible with GNU Radio
cargo run --bin r4w -- gnss scenario --format ettus --output gnuradio_compat.iq
```

### USRP/Ettus Research

The `ettus` alias maps to `cf32_le`, which is the native UHD streaming format.

### RTL-SDR

The `rtlsdr` alias maps to `cu8`, which is the native RTL-SDR output format. Note the DC offset at 127.5.

### MATLAB/Octave

```matlab
% Read cf32 file
fid = fopen('signal.iq', 'rb');
data = fread(fid, [2, Inf], 'float32');
fclose(fid);
iq = complex(data(1,:), data(2,:));

% Read ci16 file
fid = fopen('signal.iq', 'rb');
data = fread(fid, [2, Inf], 'int16');
fclose(fid);
iq = complex(data(1,:), data(2,:)) / 32768;
```

### Python

```python
import numpy as np

# Read cf32 file
iq = np.fromfile('signal.iq', dtype=np.complex64)

# Read ci16 file
raw = np.fromfile('signal.iq', dtype=np.int16)
iq = (raw[0::2] + 1j * raw[1::2]) / 32768.0

# Read cu8 file (RTL-SDR format)
raw = np.fromfile('signal.iq', dtype=np.uint8)
iq = ((raw[0::2] - 127.5) + 1j * (raw[1::2] - 127.5)) / 127.5
```

## Migration from Legacy Code

If you have code using the old scattered format implementations, migrate to `IqFormat`:

### Before (benchmark/receiver.rs)
```rust
// Old approach
let sample = match format {
    SampleFormat::Float32 => {
        let i = f32::from_le_bytes(...) as f64;
        let q = f32::from_le_bytes(...) as f64;
        IQSample::new(i, q)
    }
    SampleFormat::Int16 => {
        let i = i16::from_le_bytes(...) as f64 / 32768.0;
        let q = i16::from_le_bytes(...) as f64 / 32768.0;
        IQSample::new(i, q)
    }
};
```

### After
```rust
// New approach
let iq_format = format.to_iq_format();
let samples = iq_format.parse_bytes(&data);
```

## Troubleshooting

### Common Issues

1. **Wrong scaling for integer formats**: Ensure you use the correct scaling constants. `Ci16` uses 32768 for reading (not 32767).

2. **Endianness**: All formats use little-endian byte order. The `_le` suffix in SigMF datatypes confirms this.

3. **RTL-SDR DC offset**: Cu8 format has DC at 127.5, not 128. This is the RTL-SDR convention.

4. **File size mismatch**: Verify `file_size / bytes_per_sample = num_samples`. Use `IqFormat::bytes_per_sample()` to get the correct size.

### Validation

```bash
# Verify format roundtrip
cargo test -p r4w-core io::format
```

## Version History

- **v0.1.0** (2026-02-06): Initial unified IqFormat implementation
  - Single source of truth for all IQ formats
  - Migrated CLI, benchmark, SigMF, GUI, and HAL code
  - Added ci8 and cu8 support to GNSS scenario generator

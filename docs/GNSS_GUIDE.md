# GNSS Developer's Guide

R4W includes a complete GNSS signal simulation subsystem: four constellation waveforms, a multi-satellite scenario engine with realistic propagation models, CLI tools, a GUI simulator, and Jupyter workshops.

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [GNSS Waveforms](#gnss-waveforms)
- [Signal Acquisition and Tracking](#signal-acquisition-and-tracking)
- [Coordinate Library](#coordinate-library)
- [Environment Models](#environment-models)
- [Scenario Engine](#scenario-engine)
- [GNSS Scenario Generator](#gnss-scenario-generator)
- [CLI Reference](#cli-reference)
- [GUI Simulator](#gui-simulator)
- [Jupyter Workshops](#jupyter-workshops)
- [API Reference](#api-reference)
- [Examples](#examples)

---

## Architecture Overview

The GNSS subsystem is organized in four layers:

```
Layer 4: CLI (r4w gnss) + GUI (GnssSimulatorView)
Layer 3: GNSS Scenario Composer (r4w-core/waveform/gnss/)
Layer 2: GNSS Environment Models (r4w-core/waveform/gnss/environment/)
Layer 1: Generic Scenario Engine (r4w-sim/scenario/)
         Coordinate Library (r4w-core/coordinates.rs)
```

**Crate boundaries:**
- `r4w-core` contains coordinate types, GNSS waveforms, environment models, and the scenario composer
- `r4w-sim` contains the generic scenario engine (Emitter trait, ScenarioEngine, Trajectory)
- `r4w-cli` provides the `r4w gnss` command
- `r4w-gui` provides the GnssSimulatorView in r4w-explorer

---

## GNSS Waveforms

Four constellation signals are implemented, each conforming to the `Waveform` trait:

| Signal | Type | Frequency | Chipping Rate | Code Length | Modulation |
|--------|------|-----------|---------------|-------------|------------|
| GPS L1 C/A | `GpsL1Ca` | 1575.42 MHz | 1.023 Mchip/s | 1023 chips | BPSK(1) |
| GPS L5 | `GpsL5` | 1176.45 MHz | 10.23 Mchip/s | 10230 chips | QPSK(10) |
| GLONASS L1OF | `GlonassL1of` | ~1602 MHz (FDMA) | 0.511 Mchip/s | 511 chips | BPSK(0.5) |
| Galileo E1 | `GalileoE1` | 1575.42 MHz | 1.023 Mchip/s | 4092 chips | CBOC |

Each waveform supports `modulate()`, `demodulate()`, and `constellation_points()` via the standard `Waveform` trait.

### PRN Code Generators

Each constellation has a dedicated PRN code generator:

- **`GpsCaCodeGenerator`** - Gold codes for PRN 1-32. Call `generate_code()` to produce a 1023-chip code.
- **`GpsL5CodeGenerator`** - I5/Q5 codes (10230 chips). Construct with `new_i5(prn)` or `new_q5(prn)`.
- **`GlonassCodeGenerator`** - Maximal-length sequence (511 chips). Takes a `frequency_channel: i8` parameter for FDMA slot assignment.
- **`GalileoE1CodeGenerator`** - Real ICD memory codes from Galileo OS SIS ICD v2.1.
  - `new_e1b(prn)` — E1B data channel codes (PRN 1-50)
  - `new_e1c(prn)` — E1C pilot channel codes (PRN 1-50, data-free)
  - `secondary_code()` — 25-chip E1C secondary code
  - Source: GNSS-matlab repository, ~330 KB embedded binary

### Transmit Power

Typical space vehicle transmit power levels used in scenario generation:

| Signal | TX Power |
|--------|----------|
| GPS L1 C/A | 14.3 dBW |
| GPS L5 | 15.5 dBW |
| GLONASS L1OF | 14.7 dBW |
| Galileo E1 | 15.0 dBW |

---

## Signal Acquisition and Tracking

### PCPS Acquisition

`PcpsAcquisition` implements FFT-based Parallel Code Phase Search:

- Searches across configurable Doppler frequency bins
- Returns code phase and Doppler estimate for detected satellites
- Works with any GNSS waveform that produces a PRN code

### Tracking Channels

`TrackingChannel` provides DLL/PLL tracking loops:

- **DLL (Delay Lock Loop)** - Code phase tracking with early/prompt/late correlators
- **PLL (Phase Lock Loop)** - Carrier phase and frequency tracking

---

## Coordinate Library

`r4w-core/src/coordinates.rs` provides geodetic types and conversions used throughout the GNSS subsystem.

### Types

| Type | Description |
|------|-------------|
| `EcefPosition` | Earth-Centered Earth-Fixed position (x, y, z in meters) |
| `EcefVelocity` | ECEF velocity (vx, vy, vz in m/s) |
| `LlaPosition` | Geodetic latitude/longitude/altitude (WGS-84) |
| `LookAngle` | Elevation, azimuth, and slant range from observer to target |

### Functions

| Function | Description |
|----------|-------------|
| `lla_to_ecef(lla)` | Convert geodetic to ECEF using WGS-84 ellipsoid |
| `ecef_to_lla(ecef)` | Convert ECEF to geodetic (Bowring iterative method) |
| `look_angle(observer, observer_lla, target)` | Compute elevation, azimuth, range via ENU rotation |
| `range_rate(obs_pos, obs_vel, tgt_pos, tgt_vel)` | Radial velocity between observer and target |
| `fspl_db(distance_m, frequency_hz)` | Free-space path loss in dB |

### Constants

- `WGS84_A` = 6,378,137.0 m (semi-major axis)
- `WGS84_F` = 1/298.257223563 (flattening)
- `SPEED_OF_LIGHT` = 299,792,458.0 m/s

---

## Environment Models

Located in `r4w-core/src/waveform/gnss/environment/`. These models add realistic propagation effects to scenario generation.

### Keplerian Orbits (`orbit.rs`)

`KeplerianOrbit` propagates satellite positions using classical orbital mechanics.

**Orbital elements:**

| Field | Description |
|-------|-------------|
| `a` | Semi-major axis (meters) |
| `e` | Eccentricity (0 = circular) |
| `i` | Inclination (radians) |
| `omega_0` | Right ascension of ascending node (radians) |
| `omega` | Argument of perigee (radians) |
| `m0` | Mean anomaly at epoch (radians) |
| `t_epoch` | Epoch time (seconds) |
| `omega_dot` | RAAN precession rate (rad/s, J2 perturbation) |

**Nominal orbit constructors:**

| Constructor | Altitude | Inclination | Planes | Period |
|-------------|----------|-------------|--------|--------|
| `gps_nominal(plane, slot)` | 26,559.7 km | 55.0 deg | 6 | ~11h 58m |
| `galileo_nominal(plane, slot)` | 29,600 km | 56.0 deg | 3 | ~14h 5m |
| `glonass_nominal(plane, slot)` | 25,508 km | 64.8 deg | 3 | ~11h 16m |

The Kepler equation is solved with Newton-Raphson iteration (20 iterations, 1e-14 tolerance). Positions are transformed from ECI to ECEF accounting for Earth rotation.

### Klobuchar Ionospheric Model (`ionosphere.rs`)

`KlobucharModel` implements the GPS broadcast ionospheric correction using 8 coefficients (alpha[4] + beta[4]).

**Key characteristics:**
- Computes group delay at L1 frequency as a function of elevation, azimuth, user position, and GPS time of day
- Scales to other frequencies via 1/f^2 relationship: `scale_to_frequency(delay_l1, freq_hz)`
- Typical L1 delay: 5-15 meters (higher at low elevations)
- `default_broadcast()` provides typical mid-latitude coefficients

### Saastamoinen Tropospheric Model (`troposphere.rs`)

`SaastamoinenModel` computes tropospheric delay from meteorological parameters.

**Components:**
- **Hydrostatic (dry)**: ~2.3 m zenith delay at sea level, depends on pressure
- **Wet**: ~0.1-0.3 m zenith delay, depends on temperature and humidity
- **Mapping function**: Elevation-dependent scaling using 1/sin(el) with Chao correction

**Constructors:**
- `standard_atmosphere()` - ISA at sea level (15 C, 1013.25 hPa, 50% humidity)
- `at_altitude(height_m)` - Standard atmosphere scaled to altitude using lapse rate

### Multipath Presets (`multipath.rs`)

`GnssMultipathPreset` maps environment types to tapped delay line configurations:

| Preset | Taps | Description |
|--------|------|-------------|
| `OpenSky` | 1 | Direct path only, minimal reflections |
| `Suburban` | 3 | Direct + 2 reflections (50/120 ns, -6/-12 dB) |
| `UrbanCanyon` | 5 | Direct + 4 reflections (30-500 ns, -3 to -14 dB) |
| `Indoor` | 6 | Attenuated direct (-3 dB) + 5 reflections (20-400 ns) |

Each tap is a `MultipathTap { delay_s, power_db, phase_rad }`.

### Antenna Patterns (`antenna.rs`)

`AntennaPattern` models receiver antenna gain as a function of elevation:

| Pattern | Description | Typical Use |
|---------|-------------|-------------|
| `Isotropic` | 0 dBi uniform gain | Reference/testing |
| `Hemispherical { peak_gain_dbi }` | Uniform above horizon | Simple simulations |
| `Patch { peak_gain_dbi, beamwidth_deg }` | cos^n roll-off pattern | Consumer GNSS receivers |
| `ChokeRing { peak_gain_dbi }` | Sharp cutoff, multipath rejection | Geodetic/survey receivers |

`default_patch()` returns a 5 dBi patch with 150 deg beamwidth.

`BodyAttitude { roll_deg, pitch_deg, yaw_deg }` accounts for platform tilt effects on effective elevation.

---

## Scenario Engine

The generic scenario engine in `r4w-sim/src/scenario/` composes multiple emitters into a single IQ stream. It is not GNSS-specific and can be used for any multi-emitter simulation.

### Emitter Trait

Any signal source implements this trait:

```rust
pub trait Emitter: Send + Sync {
    fn state_at(&self, t: f64) -> EmitterState;
    fn generate_iq(&self, t: f64, num_samples: usize, sample_rate: f64) -> Vec<IQSample>;
    fn carrier_frequency_hz(&self) -> f64;
    fn nominal_power_dbm(&self) -> f64;
    fn id(&self) -> String;
}
```

`EmitterState` provides position (`EcefPosition`), velocity (`EcefVelocity`), power (dBm), and active flag.

### Trajectory

Receiver motion profiles:

| Variant | Parameters | Description |
|---------|------------|-------------|
| `Static` | `position: LlaPosition` | Fixed location |
| `Linear` | `start: LlaPosition, velocity_enu: [f64; 3]` | Constant velocity (East/North/Up m/s) |
| `Waypoints` | `points: Vec<(f64, LlaPosition)>` | Timed waypoint sequence with interpolation |
| `Circular` | `center, radius_m, omega_rad_s, initial_bearing_deg` | Circular orbit around a point |

All variants return `TrajectoryState { position, velocity, time_s }` via `state_at(t)`.

### ScenarioEngine

`ScenarioEngine::new(config, emitters, trajectory)` creates the compositor. Per block:

1. Compute receiver position from trajectory
2. For each emitter: compute geometry (range, Doppler, FSPL, elevation/azimuth)
3. Generate per-emitter baseband IQ
4. Apply Doppler shift with continuous phase accumulation
5. Scale amplitude by FSPL and transmit power
6. Sum all emitter contributions
7. Add composite AWGN noise

`ScenarioConfig` fields: `duration_s`, `sample_rate`, `center_frequency_hz`, `block_size`, `noise_floor_dbw_hz`, `seed`.

---

## GNSS Scenario Generator

The GNSS-specific layer in `r4w-core/src/waveform/gnss/` wraps the generic engine with constellation knowledge.

### GnssScenario

Top-level API:

```rust
// From a preset
let mut scenario = GnssScenario::from_preset(GnssScenarioPreset::OpenSky);

// Generate all samples
let iq = scenario.generate();

// Or block by block
while !scenario.is_done() {
    let block = scenario.generate_block(2046);
    // process block...
}

// Check satellite visibility
for sv in scenario.satellite_status() {
    println!("PRN {} el={:.1} az={:.1} C/N0={:.1}",
        sv.prn, sv.elevation_deg, sv.azimuth_deg, sv.cn0_dbhz);
}

// Write to file
scenario.write_output(&iq, Path::new("output.iq"))?;
```

### Scenario Presets

| Preset | Satellites | Receiver | Environment |
|--------|-----------|----------|-------------|
| `OpenSky` | 4 GPS L1 C/A (PRN 1,7,13,23) | Static, Philadelphia | Iono + tropo, no multipath |
| `UrbanCanyon` | 6 GPS L1 C/A (PRN 1,3,7,13,17,23) | Static | Iono + tropo + urban multipath, 15 deg mask |
| `MultiConstellation` | 3 GPS + 3 Galileo | Static | Iono + tropo, no multipath |
| `Driving` | 5 GPS L1 C/A | Linear, 15 m/s East | Iono + tropo + suburban multipath |
| `Walking` | 4 GPS L1 C/A | Linear, 1.5 m/s East | Iono + tropo, no multipath |
| `HighDynamics` | 6 GPS L1 C/A | Linear, 300 m/s East | Iono + tropo, no multipath |

### SatelliteEmitter

Wraps a `Waveform` with orbital mechanics and atmospheric models:

- Computes satellite ECEF position/velocity from `KeplerianOrbit` at any time
- Generates code-phase-aligned baseband IQ from geometric pseudorange (no generate-then-delay waste)
- Per-sample Doppler interpolation with continuous phase accumulation (no block-level quantization)
- Applies ionospheric and tropospheric delays as carrier phase shifts
- Reports `SatelliteStatus` with elevation, azimuth, range, Doppler, C/N0, delay breakdowns

### Configuration

`GnssScenarioConfig` has four sections:

**`satellites: Vec<SatelliteConfig>`**
- `signal` - GpsL1Ca, GpsL5, GlonassL1of, or GalileoE1
- `prn` - PRN number
- `plane`, `slot` - Orbital plane and slot indices
- `tx_power_dbw` - Transmit power
- `nav_data` - Enable navigation data modulation

**`receiver: ReceiverConfig`**
- `position: LlaPosition` - Receiver location
- `antenna: AntennaPattern` - Antenna type
- `elevation_mask_deg` - Minimum elevation (default 5 deg)
- `noise_figure_db` - Receiver noise figure (default 2 dB)
- `bandwidth_hz` - Front-end bandwidth (default 2.046 MHz)

**`environment: EnvironmentConfig`**
- `ionosphere_enabled` / `ionosphere_model` - Klobuchar on/off and custom coefficients
- `troposphere_enabled` / `troposphere_model` - Saastamoinen on/off and custom parameters
- `multipath_enabled` / `multipath_preset` - Multipath environment type

**`output: OutputConfig`**
- `sample_rate` - Output sample rate (default 2.046 MHz)
- `duration_s` - Scenario duration (default 0.001 s)
- `block_size` - Processing block size (0 = auto)
- `seed` - PRNG seed for reproducibility

---

## CLI Reference

All GNSS commands are under `r4w gnss`:

### Signal Exploration

```bash
# Show all signal parameters
r4w gnss info --signal all

# Side-by-side comparison table
r4w gnss compare

# PRN code analysis with cross-correlation
r4w gnss code --prn 1 --cross-prn 7

# Single-satellite simulation
r4w gnss simulate --prn 1 --cn0 40 --doppler 1000

# Generate baseband IQ with nav data
r4w gnss generate --signal GPS-L1CA --prn 1 --bits 10
```

### Scenario Generation

```bash
# List available presets
r4w gnss scenario --list-presets

# Generate open-sky scenario (default 1ms, 2.046 MHz)
r4w gnss scenario --preset open-sky -o output.iq

# Custom duration and sample rate
r4w gnss scenario --preset urban-canyon --duration 0.01 --sample-rate 4092000 -o urban.iq

# From YAML configuration file
r4w gnss scenario --config my_scenario.yaml -o custom.iq
```

### Output Formats

Multiple IQ output formats are supported via `--format`:

```bash
# Default: cf64 (Complex float64, 16 bytes/sample)
r4w gnss scenario --preset open-sky -o output.iq

# USRP/Ettus-compatible interleaved float32 (8 bytes/sample)
r4w gnss scenario --preset open-sky --format ettus -o usrp.iq

# Compact signed 16-bit (4 bytes/sample)
r4w gnss scenario --preset open-sky --format sc16 -o compact.iq

# RTL-SDR native unsigned 8-bit (2 bytes/sample)
r4w gnss scenario --preset open-sky --format rtlsdr -o rtlsdr.iq
```

| Format | Alias | Bytes/Sample | Use Case |
|--------|-------|-------------|----------|
| `cf64` | — | 16 | Maximum precision, analysis |
| `cf32` | `ettus` | 8 | USRP/Ettus hardware, GNU Radio |
| `ci16` | `sc16` | 4 | Compact storage, moderate precision |
| `ci8` | — | 2 | Low-bandwidth, embedded |
| `cu8` | `rtlsdr` | 2 | RTL-SDR native format |

### Precise Ephemeris (SP3/IONEX)

For cm-level satellite positions and accurate ionospheric corrections, use SP3 and IONEX files from CODE/IGS:

```bash
# Requires the 'ephemeris' feature
cargo run --bin r4w --features ephemeris -- gnss scenario --preset open-sky \
    --sp3 /path/to/COD0OPSFIN_20260050000_01D_05M_ORB.SP3 \
    --ionex /path/to/COD0OPSFIN_20260050000_01D_01H_GIM.INX \
    --duration 0.01 --output precise.iq
```

SP3 files provide interpolated satellite positions (1-5 minute intervals) and clock corrections (microsecond-level biases displayed per-PRN). IONEX files provide global TEC grids for ionospheric delay computation, replacing the simpler Klobuchar broadcast model.

### Receiver Position and Time

```bash
# Custom receiver position (WGS-84)
r4w gnss scenario --preset open-sky --lat 48.8566 --lon 2.3522 --alt 35.0 -o paris.iq

# Set GPS time (auto-discovers visible satellites)
r4w gnss scenario --preset open-sky --time "2026-02-08T12:00:00Z" -o timed.iq

# Filter by signal type
r4w gnss scenario --preset multi-constellation --signals GPS-L1CA,Galileo-E1 -o filtered.iq

# Export preset to YAML for editing
r4w gnss scenario --preset urban-canyon --export-preset > custom_scenario.yaml
r4w gnss scenario --config custom_scenario.yaml -o custom.iq
```

### Satellite Auto-Discovery

When `--lat/--lon/--alt` and `--time` are provided, the scenario engine automatically discovers visible satellites using the real-world PRN lookup tables (GPS 31 SVs from NAVCEN, Galileo 24 SVs from GSC, GLONASS 24 SVs). Only satellites above the elevation mask are included.

The CLI displays a satellite status table and signal statistics after generation:

```
PRN  Signal      Elev    Azim    Range(km)    Doppler(Hz)  C/N0(dB-Hz)  Iono(m)  Tropo(m)
  1  GPS-L1CA    45.2    120.3    22,145.7       1,234.5      42.3        8.2      4.1
  7  GPS-L1CA    67.8     45.1    20,890.2        -567.2      45.1        6.1      3.2
 13  GPS-L1CA    23.4    245.6    24,102.8       2,891.0      38.7       12.4      6.8
 23  GPS-L1CA    78.1    310.2    20,456.3        -123.4      46.8        5.3      2.9
```

---

## GUI Simulator

The `GnssSimulatorView` in r4w-explorer provides interactive scenario visualization:

- **Sky plot** - Polar projection showing satellite positions with elevation rings and cardinal labels, color-coded by constellation
- **C/N0 bars** - Bar chart of carrier-to-noise density per visible satellite
- **IQ waveform** - Time-domain display of composite I and Q channels
- **Controls** - Preset selector, duration/sample-rate sliders, environment toggles (ionosphere, troposphere, multipath)

Launch with:

```bash
cargo run --bin r4w-explorer
# Navigate to GNSS Simulator view
```

### Pipeline Builder GNSS Blocks

The visual pipeline builder includes a GNSS category (teal) with two blocks for interactive signal exploration:

**GnssScenarioSource** — Multi-satellite IQ generator (source block, 0 inputs → 1 IQ output):
- Preset selector: OpenSky, UrbanCanyon, Driving, Walking, HighDynamics, MultiConstellation
- Configurable sample rate (1-20 MHz), duration (0.1 ms - 6 s), receiver position (lat/lon/alt)
- Noise figure and elevation mask controls
- Test panel caps duration at 10 ms for responsiveness

**GnssAcquisition** — PCPS acquisition engine (1 IQ input → 1 Real output):
- Signal type selector: GPS-L1CA, GPS-L5, Galileo-E1, GLONASS-L1OF
- Configurable PRN (1-50), Doppler search range, step size, and detection threshold
- Outputs detection status, code phase, Doppler estimate, and peak metric

**GnssOpenSky preset** connects: Scenario Source → Acquisition → IQ Output

```bash
cargo run --bin r4w-explorer
# Pipeline Builder → Presets → GNSS Open Sky
```

---

## Jupyter Workshops

Four notebooks in `notebooks/` provide hands-on GNSS learning:

### 09: GNSS Scenario Generation

- GNSS signal basics (frequency, chipping rate, code length, modulation)
- Generating multi-satellite scenarios via CLI
- Comparing open-sky vs multi-constellation spectra
- Urban canyon multipath effects on IQ signals
- Exercises: PCPS acquisition on composite IQ, per-satellite C/N0 estimation

### 10: GNSS Environment Models

- 3D orbital visualization with Keplerian propagation
- Klobuchar ionospheric delay vs elevation and time of day
- Saastamoinen tropospheric delay vs elevation and altitude
- Multipath tap profiles for different environments
- Antenna gain patterns (patch, choke ring, hemispherical)
- Complete link budget calculation from SV to receiver

### 11: GNSS Precise Ephemeris

- SP3 file format and CODE FTP server data
- Precise orbit interpolation (Lagrange, cm-level accuracy)
- Satellite clock corrections from SP3
- IONEX TEC maps for ionospheric delay
- Comparison: Keplerian vs precise orbits

### 12: Filter Design (GNSS-relevant)

- FIR anti-aliasing filter design for GNSS front-ends
- Polyphase decimation for sample rate reduction
- Parks-McClellan equiripple design for narrowband filtering

---

## API Reference

### Key Types (r4w-core)

```
r4w_core::coordinates::{EcefPosition, EcefVelocity, LlaPosition, LookAngle}
r4w_core::coordinates::{lla_to_ecef, ecef_to_lla, look_angle, range_rate, fspl_db}

r4w_core::waveform::gnss::{GpsL1Ca, GpsL5, GlonassL1of, GalileoE1}
r4w_core::waveform::gnss::{GpsCaCodeGenerator, GpsL5CodeGenerator,
                            GlonassCodeGenerator, GalileoE1CodeGenerator}
r4w_core::waveform::gnss::{PcpsAcquisition, TrackingChannel, LnavMessage}

r4w_core::waveform::gnss::{GnssScenario, GnssScenarioConfig, GnssScenarioPreset}
r4w_core::waveform::gnss::{SatelliteEmitter, SatelliteConfig, ReceiverConfig,
                            EnvironmentConfig, OutputConfig}

r4w_core::waveform::gnss::{KeplerianOrbit, KlobucharModel, SaastamoinenModel,
                            GnssMultipathPreset, AntennaPattern, BodyAttitude}
```

### Key Types (r4w-sim)

```
r4w_sim::scenario::{Emitter, EmitterState, EmitterStatus}
r4w_sim::scenario::{ScenarioEngine, ScenarioConfig}
r4w_sim::scenario::{Trajectory, TrajectoryState}
```

---

## Examples

### Generate and analyze a scenario in Rust

```rust
use r4w_core::waveform::gnss::{GnssScenario, GnssScenarioPreset};
use std::path::Path;

let mut scenario = GnssScenario::from_preset(GnssScenarioPreset::OpenSky);

// Print visible satellites
for sv in scenario.satellite_status() {
    if sv.visible {
        println!("PRN {:2} | El {:5.1} | Az {:5.1} | C/N0 {:4.1} dB-Hz",
            sv.prn, sv.elevation_deg, sv.azimuth_deg, sv.cn0_dbhz);
    }
}

// Generate IQ and write to file
let iq = scenario.generate();
scenario.write_output(&iq, Path::new("/tmp/gnss.iq")).unwrap();
```

### Custom multi-constellation configuration

```rust
use r4w_core::waveform::gnss::*;
use r4w_core::coordinates::LlaPosition;

let config = GnssScenarioConfig {
    satellites: vec![
        SatelliteConfig::gps_l1ca(1, 0, 0),
        SatelliteConfig::gps_l1ca(7, 1, 2),
        SatelliteConfig::galileo_e1(1, 0, 0),
        SatelliteConfig::galileo_e1(5, 1, 3),
    ],
    receiver: ReceiverConfig {
        position: LlaPosition::new(51.5, -0.1, 30.0), // London
        antenna: AntennaPattern::default_patch(),
        elevation_mask_deg: 10.0,
        noise_figure_db: 2.0,
        bandwidth_hz: 2_046_000.0,
    },
    environment: EnvironmentConfig {
        ionosphere_enabled: true,
        troposphere_enabled: true,
        multipath_enabled: true,
        multipath_preset: GnssMultipathPreset::Suburban,
        ..Default::default()
    },
    output: OutputConfig {
        sample_rate: 4_092_000.0, // 4x chipping rate
        duration_s: 0.010,        // 10 ms
        ..Default::default()
    },
};

let mut scenario = GnssScenario::new(config);
let iq = scenario.generate();
```

### Reading generated IQ files in Python

```python
import numpy as np
import struct

def read_iq(path):
    with open(path, 'rb') as f:
        data = f.read()
    samples = struct.unpack(f'<{len(data)//8}d', data)
    return np.array(samples[0::2]) + 1j * np.array(samples[1::2])

iq = read_iq('/tmp/gnss.iq')
print(f"Samples: {len(iq)}, Mean power: {10*np.log10(np.mean(np.abs(iq)**2)):.1f} dB")
```

---

## Requirements Traceability

| Requirement | Title | Status |
|-------------|-------|--------|
| FR-032 | GPS L1 C/A Waveform | Completed |
| FR-033 | GPS L5 Waveform | Completed |
| FR-034 | GLONASS L1OF Waveform | Completed |
| FR-035 | Galileo E1 Waveform | Completed |
| FR-036 | GNSS PRN Code Generation | Completed |
| FR-037 | GNSS Acquisition and Tracking | Completed |
| FR-038 | GNSS CLI Subcommand | Completed |
| FR-039 | GNSS Scenario Engine | Completed |
| FR-040 | GNSS Environment Models | Completed |
| FR-041 | GNSS IQ Signal Generator | Completed |
| FR-042 | GNSS Scenario CLI and GUI | Completed |

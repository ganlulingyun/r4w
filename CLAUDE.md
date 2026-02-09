# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**R4W - Rust for Waveforms** - A platform for developing, testing, and deploying SDR waveforms in Rust. Provides reusable DSP libraries, educational tools, and production-ready components for waveform development.

### Architecture

- **r4w-core**: Core DSP algorithms, timing, RT primitives, configuration
  - `waveform/`: 42+ waveform implementations (including GNSS: GPS L1 C/A, GPS L5, GLONASS L1OF, Galileo E1)
  - `waveform/gnss/environment/`: Keplerian orbits, Klobuchar ionosphere, Saastamoinen troposphere, multipath presets, antenna patterns
  - `waveform/gnss/scenario*.rs`: Multi-satellite GNSS IQ scenario generation with realistic channel effects
  - `coordinates.rs`: ECEF/LLA coordinate types, geodetic conversions, look angles, range rate, FSPL
  - `filters/`: Digital filters with trait-based architecture
    - `traits.rs`: Filter, RealFilter, FirFilterOps, FrequencyResponse traits
    - `fir.rs`: FirFilter with lowpass/highpass/bandpass/bandstop, Kaiser window design
    - `iir.rs`: IirFilter with Butterworth, Chebyshev I/II, Bessel via cascaded biquads
    - `polyphase.rs`: PolyphaseDecimator, PolyphaseInterpolator, Resampler, HalfbandFilter
    - `remez.rs`: Parks-McClellan equiripple FIR design (RemezSpec builder)
    - `pulse_shaping.rs`: RRC, RC, Gaussian filters (also implement Filter traits)
    - `windows.rs`: Hamming, Hann, Blackman, Kaiser window functions
  - `analysis/`: Spectrum analyzer, waterfall generator, signal statistics, peak detection
  - `timing.rs`: Multi-clock model (SampleClock, WallClock, HardwareClock, SyncedTime)
  - `rt/`: Lock-free ring buffers, buffer pools, RT thread spawning
  - `config.rs`: YAML-based configuration system
- **r4w-sim**: SDR simulation, HAL traits, channel models
  - `hal/`: StreamHandle, TunerControl, ClockControl, SdrDeviceExt traits
  - `channel.rs`: AWGN, Rayleigh, Rician, CFO, TDL multipath (EPA/EVA/ETU), Jake's Doppler
  - `doppler.rs`: Jake's/Clarke's, Flat, and Gaussian Doppler models
  - `scenario/`: Generic multi-emitter IQ scenario engine (trajectory, emitter trait, Doppler/FSPL/noise)
  - `simulator.rs`: Software SDR simulator
- **r4w-fpga**: FPGA acceleration (Xilinx Zynq, Lattice iCE40/ECP5)
- **r4w-sandbox**: Waveform isolation (8 security levels)
- **r4w-gui**: Educational egui application (run with `cargo run --bin r4w-explorer`)
  - `views/pipeline_wizard.rs`: Visual pipeline builder with 40+ blocks, TX/RX/Channel loading, test panel
  - `views/block_metadata.rs`: Block documentation, formulas, code links, tests, performance info
- **r4w-cli**: Command-line interface (run with `cargo run --bin r4w`)
- **r4w-web**: WebAssembly entry point for browser deployment

### Key Commands

```bash
# Run GUI
cargo run --bin r4w-explorer

# CLI examples
cargo run --bin r4w -- info --sf 7 --bw 125
cargo run --bin r4w -- simulate --message "Hello R4W!" --snr 10.0
cargo run --bin r4w -- waveform --list

# Mesh networking
cargo run --bin r4w -- mesh info
cargo run --bin r4w -- mesh status --preset LongFast --region US
cargo run --bin r4w -- mesh send -m "Hello mesh!" --dest broadcast
cargo run --bin r4w -- mesh simulate --nodes 4 --messages 10

# Waveform comparison (BER vs SNR)
cargo run --bin r4w -- compare -w BPSK,QPSK,16QAM --snr-min -5 --snr-max 20
cargo run --bin r4w -- compare --list  # Show available waveforms

# Shell completions (bash/zsh/fish/powershell)
cargo run --bin r4w -- completions bash > ~/.local/share/bash-completion/completions/r4w

# Record/Playback signals (SigMF format)
cargo run --bin r4w -- record -o test.sigmf --generate tone --duration 5.0
cargo run --bin r4w -- playback -i test.sigmf --info

# GNSS signal exploration
cargo run --bin r4w -- gnss info --signal all
cargo run --bin r4w -- gnss compare
cargo run --bin r4w -- gnss code --prn 1 --cross-prn 7
cargo run --bin r4w -- gnss simulate --prn 1 --cn0 40 --doppler 1000
cargo run --bin r4w -- gnss generate --signal GPS-L1CA --prn 1 --bits 10

# GNSS scenario generation (multi-satellite IQ with channel effects)
cargo run --bin r4w -- gnss scenario --list-presets
cargo run --bin r4w -- gnss scenario --preset open-sky --duration 0.001 --output test.iq
cargo run --bin r4w -- gnss scenario --preset urban-canyon --duration 0.01
cargo run --bin r4w -- gnss scenario --preset multi-constellation --sample-rate 4092000

# GNSS with precise ephemeris (SP3) and ionosphere (IONEX) - requires 'ephemeris' feature
cargo run --bin r4w --features ephemeris -- gnss scenario --preset open-sky \
    --sp3 /path/to/COD0OPSFIN_20260050000_01D_05M_ORB.SP3 \
    --ionex /path/to/COD0OPSFIN_20260050000_01D_01H_GIM.INX \
    --duration 0.01 --output precise.iq

# Output formats: cf64, cf32/ettus (USRP-compatible), ci16/sc16, ci8, cu8/rtlsdr
cargo run --bin r4w -- gnss scenario --preset open-sky --format ettus --output usrp.iq
cargo run --bin r4w -- gnss scenario --preset open-sky --format sc16 --output compact.iq
cargo run --bin r4w -- gnss scenario --preset open-sky --format rtlsdr --output rtlsdr.iq

# Signal analysis
cargo run --bin r4w -- analyze spectrum -i file.sigmf-meta --fft-size 1024
cargo run --bin r4w -- analyze waterfall -i file.sigmf-meta --output waterfall.png
cargo run --bin r4w -- analyze stats -i file.sigmf-meta --format json
cargo run --bin r4w -- analyze peaks -i file.sigmf-meta --threshold 10

# Generate gallery images
cargo run --example gallery_generate -p r4w-sim --features image

# Prometheus metrics
cargo run --bin r4w -- metrics --format prometheus
cargo run --bin r4w -- metrics --serve --port 9090

# Run in browser (WASM)
cd crates/r4w-web && trunk serve

# Docker
docker run -it r4w:latest r4w --help
docker-compose run r4w-dev  # Development container

# cargo-binstall (pre-built binaries)
cargo binstall r4w-cli

# Cross-compile for ARM
make build-cli-arm64
make deploy-arm64 REMOTE_HOST=joe@raspberrypi

# Generate HTML documentation from markdown (requires pandoc)
make docs-html     # Output: docs/html/ (21 files with TOC and syntax highlighting)
make docs-clean    # Remove generated HTML
```

### Waveform Development

All waveforms implement the `Waveform` trait:
- `modulate(&self, bits: &[bool]) -> Vec<IQSample>`
- `demodulate(&self, samples: &[IQSample]) -> Vec<bool>`
- `constellation_points(&self) -> Vec<IQSample>`
- `get_modulation_stages()` / `get_demodulation_steps()` for education

See OVERVIEW.md for the full Waveform Developer's Guide and Porting Guide.

### Technical Notes

- LoRa uses Chirp Spread Spectrum (CSS) modulation
- FFT-based demodulation (multiply by downchirp, find peak)
- Pipeline: Whitening → Hamming FEC → Interleaving → Gray Code → CSS
- Spreading factors 5-12, bandwidths 125/250/500 kHz
- PSK/FSK/QAM waveforms for comparison and education

### Recent Updates

- **Pipeline Test Panel** - Collapsible test panel in Pipeline Builder for testing individual blocks. Input patterns: Random, AllZeros, AllOnes, Alternating, PRBS-7. View tabs: Bits (binary display), Time Domain (I/Q waveforms), Constellation (IQ diagram), Spectrum (DFT). Simplified PSK/QAM processing for visualization.
- **Block Metadata System** - Comprehensive documentation for pipeline blocks (`block_metadata.rs`). Each block has: implementation location (file:line with "View Code" to open in VS Code), mathematical formulas with variable explanations, unit tests with "Run" buttons, performance info (complexity, SIMD/GPU support), standards references with links. Properties panel shows collapsible Documentation/Formulas/Implementation/Tests/Performance/Standards sections.
- **TX/RX/Channel Pipeline Separation** - Waveform specs v1.1 format with separate `tx`, `rx`, and `channel` sections. Load menu offers TX/RX/Loopback options. Block ID conventions: TX(1-99), RX(100-199), Channel(200-299). Loopback mode auto-connects TX → Channel → RX.
- **Demodulator Blocks** - Added PskDemodulator, QamDemodulator, FskDemodulator block types with property editors and YAML serialization.
- **Updated Waveform Specs** - All specs (bpsk, qpsk, fsk, lora, cw) updated to v1.1 format with complete TX, RX, and Channel pipeline definitions.
- **Visual Pipeline Builder** - Graphical signal processing pipeline designer (`r4w-gui/views/pipeline_wizard.rs`). 40+ block types in 10 categories (Source, Coding, Mapping, Modulation, Filtering, Rate Conversion, Synchronization, Impairments, Recovery, Output). Interactive canvas with zoom/pan, Bezier curve connections, click-to-connect ports. 12 preset templates (BPSK/QPSK/16-QAM/LoRa/OFDM/FSK/DSSS/DMR transmitters, TX-Channel-RX systems, Parallel I/Q demo). Auto-layout with topological sorting, pipeline validation (cycle detection, unconnected ports), YAML export, snap-to-grid, keyboard shortcuts.
- **Enhanced Waveform Wizard** - Filtering step (FIR/IIR options, sample rate conversion), Synchronization step (timing/carrier recovery, AGC, equalization), Frame Structure step (TDMA, packet formats, CRC). 11 wizard steps total.
- **Generic Filter Trait Architecture** - Extensible filter framework with `Filter`, `RealFilter`, `FirFilterOps`, `FrequencyResponse` traits. FirFilter supports lowpass/highpass/bandpass/bandstop with multiple window functions (Blackman, Hamming, Hann, Kaiser). Kaiser window design with auto β and order calculation. Pulse shaping filters (RRC, RC, Gaussian) now implement all traits for polymorphic usage and frequency response analysis. Special-purpose filters: `moving_average()`, `differentiator()`, `hilbert()`.
- **Parks-McClellan Equiripple FIR Design** - Optimal FIR filter design using Remez exchange algorithm. `RemezSpec` builder with `lowpass()`, `highpass()`, `bandpass()`, `bandstop()`, `differentiator()`, `hilbert()`. Configurable passband/stopband weights, `estimate_order()` for filter sizing. Produces equiripple (Chebyshev) error distribution for minimum filter length.
- **Polyphase Sample Rate Conversion** - Efficient decimation and interpolation using polyphase decomposition. `PolyphaseDecimator` for M-fold downsampling, `PolyphaseInterpolator` for L-fold upsampling, `Resampler` for rational L/M conversion (e.g., 48kHz↔44.1kHz), `HalfbandFilter` for 2x with ~50% compute savings. All include built-in anti-aliasing/anti-imaging filters.
- **IIR Filters** - IirFilter with cascaded biquad (SOS) implementation for numerical stability. Butterworth (maximally flat), Chebyshev Type I (equiripple passband), Chebyshev Type II (equiripple stopband), Bessel (maximally flat group delay). Bilinear transform with frequency pre-warping. Analysis: `frequency_response()`, `magnitude_response_db()`, `phase_response()`, `group_delay_at()`, `is_stable()`.
- **Real Galileo E1 ICD Codes** - Replaced simulated LFSR codes with real memory codes from Galileo OS SIS ICD v2.1. E1B (data) and E1C (pilot) codes for PRN 1-50, plus E1C secondary code. New API: `GalileoE1CodeGenerator::new_e1b(prn)`, `new_e1c(prn)`, `secondary_code()`. Source: GNSS-matlab repository. Binary size: ~330 KB embedded.
- **Unified IqFormat Module** - New `r4w_core::io::IqFormat` enum provides single source of truth for IQ sample formats across the codebase. Supports cf64 (16 bytes), cf32/ettus (8 bytes), ci16/sc16 (4 bytes), ci8 (2 bytes), cu8/rtlsdr (2 bytes). Replaces scattered format handling in CLI, benchmark, SigMF, and GUI code. Full roundtrip I/O, SigMF datatype strings, and comprehensive string aliases.
- **SP3 Precise Ephemeris & IONEX TEC Maps** - cm-level satellite positions from CODE FTP server SP3 files. Global ionospheric TEC grids from IONEX files for accurate ionospheric delay. Satellite clock corrections extracted from SP3 (microsecond-level biases displayed per-PRN). CLI: `--sp3 <path>`, `--ionex <path>`. Requires `--features ephemeris`.
- **Multi-Format IQ Output** - USRP/Ettus-compatible interleaved float32 format (`--format ettus`), plus sc16 (signed 16-bit) for compact storage. Formats: cf64 (16 bytes/sample), cf32/ettus (8 bytes), ci16/sc16 (4 bytes), ci8 (2 bytes), cu8/rtlsdr (2 bytes).
- **Corrected C/N0 Link Budget** - Fixed C/N0 calculation: EIRP(dBW) - FSPL + Gr + 204 dBW/Hz. Realistic 30-45 dB-Hz values for GPS L1 at varying elevations.
- **GNSS Scenario Enhancements** - Real-world PRN lookup tables (GPS 31 SVs from NAVCEN, Galileo 24 SVs from GSC, GLONASS 24 SVs). Galileo orbit calibration (Walker 24/3/1, RAAN/M0 offsets at 2026 epoch). Per-sample Doppler interpolation. GPS time conversion. CLI: `--lat/--lon/--alt`, `--time`, `--signals` filter, `--export-preset`, `--config` YAML file. Auto-discovery of visible satellites.
- **Emergency Distress Beacons** - 121.5 MHz / 243 MHz swept-tone AM beacon waveforms per ICAO Annex 10. ELT (aircraft), EPIRB (maritime), PLB (personal), Military (243 MHz). Factory names: ELT-121.5, EPIRB-121.5, PLB-121.5, Beacon-243.
- **GNSS IQ Scenario Generator** - Multi-satellite composite IQ generation with Keplerian orbits, Klobuchar ionosphere, Saastamoinen troposphere, multipath presets, antenna patterns, orbital Doppler, and FSPL. CLI: `r4w gnss scenario`. GUI: GNSS Simulator view with sky plot and C/N0 bars. Presets: OpenSky, UrbanCanyon, Driving, Walking, HighDynamics, MultiConstellation.
- **Generic Scenario Engine** - Reusable multi-emitter IQ composition framework in r4w-sim with `Emitter` trait, trajectory models, per-emitter Doppler/path-loss/channel, and composite noise.
- **Coordinate Library** - ECEF/LLA/ENU types with WGS-84 conversions, look angles, range rate, FSPL in r4w-core.
- **GNSS Environment Models** - Keplerian orbit propagation (GPS/Galileo/GLONASS nominal orbits), Klobuchar ionospheric delay, Saastamoinen tropospheric delay, multipath presets (OpenSky/Suburban/UrbanCanyon/Indoor), antenna patterns (Isotropic/Patch/ChokeRing).
- **Jupyter Workshops** - 12 interactive tutorials including GNSS scenario generation, environment models, precise ephemeris, and signal verification (`notebooks/09_*.ipynb` through `notebooks/12_*.ipynb`)
- **GNSS Waveforms** - GPS L1 C/A, GPS L5, GLONASS L1OF, Galileo E1 with PRN code generation, FFT-based PCPS acquisition, DLL/PLL tracking loops, navigation data encoding/decoding, and `r4w gnss` CLI subcommand
- **Enhanced Channel Simulation** - Jake's/Clarke's Doppler model, Tapped Delay Line (TDL) multipath with 3GPP profiles (EPA, EVA, ETU)
- **CLI Analysis Tools** - `r4w analyze` subcommands: spectrum, waterfall, stats, peaks
- **Signal Gallery** - 23 PNG images: constellations, spectra, channel effects (`gallery/`)
- **Jupyter Notebooks** - 8 interactive tutorials with Python wrapper (`notebooks/`)
- **Deployment Options** - Docker image, cargo-binstall manifest, GitHub Actions release workflow
- **CLI Enhancements** - Record/Playback (SigMF), Prometheus metrics, shell completions, waveform comparison
- **Example Gallery** - Getting-started examples for modulation, channels, LoRa, mesh networking
- **GitHub Actions CI** - Automated testing, cross-platform builds, WASM builds, performance regression tracking
- **Mesh CLI Commands** - `r4w mesh` subcommands for LoRa mesh networking (status, send, neighbors, simulate, info)
- **Mesh Networking Module** - Full mesh stack with MeshtasticNode, FloodRouter, CSMA/CA MAC, LoRaMesh integration
- **Physical Layer Architecture** (Session 18):
  - Multi-clock timing model (SampleClock, WallClock, HardwareClock, GPS/PTP SyncedTime)
  - Real-time primitives (lock-free SPSC ring buffers, buffer pools, RT thread spawning)
  - YAML configuration system with hardware profiles
  - Enhanced HAL traits (StreamHandle, TunerControl, ClockControl)
- Added TETRA, DMR, and 3G ALE waveform implementations
- Enhanced waveform-spec schema with TDMA/FDMA, radar, classified components
- Renamed project to "R4W - Rust for Waveforms"
- Created platform vision with Waveform Developer's Guide
- Documented FPGA integration architecture
- Remote Lab for distributed testing on Raspberry Pis

## Requirements Management

This project uses AIDA for requirements tracking. **Do NOT maintain a separate REQUIREMENTS.md file.**

Requirements database: `requirements.yaml`

### CLI Commands
```bash
aida list                              # List all requirements
aida list --status draft               # Filter by status
aida show <ID>                         # Show requirement details (e.g., FR-0042)
aida add --title "..." --description "..." --status draft  # Add new requirement
aida edit <ID> --status completed      # Update status
aida comment add <ID> "..."            # Add implementation note
```

### During Development
- When implementing a feature, update its requirement status
- Add comments to requirements with implementation decisions
- Create child requirements for sub-tasks discovered during implementation
- Link related requirements with: `aida rel add --from <FROM> --to <TO> --type <Parent|Verifies|References>`

### Session Workflow
If you work conversationally without explicit /aida-req calls, use `/aida-capture` at session end to review and capture any requirements that were discussed but not yet added to the database.

## Code Traceability

### Inline Trace Comments
When implementing requirements, add inline trace comments:

```rust
// trace:FR-0042 | ai:claude
fn implement_feature() {
    // Implementation
}
```

Format: `// trace:<SPEC-ID> | ai:<tool>[:<confidence>]`

### Commit Message Format
**Standard format:**
```
[AI:tool] type(scope): description (REQ-ID)
```

**Examples:**
```
[AI:claude] feat(auth): add login validation (FR-0042)
[AI:claude:med] fix(api): handle null response (BUG-0023)
chore(deps): update dependencies
docs: update README
```

**Rules:**
- `[AI:tool]` - Required when commit includes AI-assisted code
- `type` - Required: feat, fix, docs, style, refactor, perf, test, build, ci, chore, revert
- `(scope)` - Optional: component or area affected
- `(REQ-ID)` - Required for feat/fix commits, optional for chore/docs

**Confidence levels:**
- `[AI:claude]` - High confidence (implied, >80% AI-generated)
- `[AI:claude:med]` - Medium (40-80% AI with modifications)
- `[AI:claude:low]` - Low (<40% AI, mostly human)

**Configuration:**
Set `AIDA_COMMIT_STRICT=true` to reject non-conforming commits, or create `.aida/commit-config`.

## Claude Code Skills

This project uses AIDA requirements-driven development:

### /aida-req
Add new requirements with AI evaluation:
- Interactive requirement gathering
- Immediate database storage with draft status
- Background AI evaluation for quality feedback
- Follow-up actions: improve, split, link, accept

### /aida-implement
Implement requirements with traceability:
- Load and display requirement context
- Break down into child requirements as needed
- Update requirements during implementation
- Add inline traceability comments to code

### /aida-capture
Review session and capture missed requirements:
- Scan conversation for discussed features/bugs/ideas
- Identify implemented work not yet in requirements database
- Prompt to add missing requirements or update statuses
- Use at end of conversational sessions as a safety net

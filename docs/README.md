# R4W Documentation

This directory contains comprehensive documentation for the R4W (Rust for Waveforms) platform.

## Documentation Index

### Getting Started

| Document | Description |
|----------|-------------|
| [OVERVIEW.md](../OVERVIEW.md) | Platform vision, architecture, and quick start guide |
| [CLAUDE.md](../CLAUDE.md) | Development guidelines and commands |

### Developer Guides

| Document | Audience | Description |
|----------|----------|-------------|
| [WAVEFORM_DEVELOPERS_GUIDE.md](./WAVEFORM_DEVELOPERS_GUIDE.md) | Software Engineers | Complete guide to developing, debugging, testing, and deploying waveforms |
| [PHYSICAL_LAYER_GUIDE.md](./PHYSICAL_LAYER_GUIDE.md) | Platform Engineers | Timing model, HAL, RT primitives, configuration, observability |
| [IQ_FORMAT_GUIDE.md](./IQ_FORMAT_GUIDE.md) | All Developers | Unified IQ sample formats, file I/O, SigMF integration, format conversion |
| [GNSS_GUIDE.md](./GNSS_GUIDE.md) | GNSS Developers | GPS/Galileo/GLONASS signal generation, scenario simulation, precise ephemeris |
| [TICK_SCHEDULER_GUIDE.md](./TICK_SCHEDULER_GUIDE.md) | Platform Engineers | Discrete event simulation, time scaling, tick-based event handling |
| [REALTIME_SCHEDULER_GUIDE.md](./REALTIME_SCHEDULER_GUIDE.md) | Platform Engineers | TX/RX coordination, FHSS, TDMA, production timing |
| [FPGA_DEVELOPERS_GUIDE.md](./FPGA_DEVELOPERS_GUIDE.md) | FPGA Engineers | IP core library, register maps, integration procedures, collaboration with software |
| [SECURITY_GUIDE.md](./SECURITY_GUIDE.md) | Security Engineers | Memory safety, cryptographic handling, process isolation, secure deployment |
| [ISOLATION_GUIDE.md](./ISOLATION_GUIDE.md) | Security/Platform | Waveform isolation (containers, VMs, hardware separation, MLS) |

### Porting Guides

| Document | Description |
|----------|-------------|
| [PORTING_GUIDE_MILITARY.md](./PORTING_GUIDE_MILITARY.md) | Overview of military waveform implementation status and porting architecture |
| [porting/README.md](./porting/README.md) | Quick reference for all porting guides |
| [porting/BUILD_PROCEDURES.md](./porting/BUILD_PROCEDURES.md) | Build options, linking, FFI, cross-compilation |

#### Waveform-Specific Porting

| Document | Waveform | Complexity |
|----------|----------|------------|
| [porting/sincgars.md](./porting/sincgars.md) | SINCGARS VHF frequency hopping | High (classified) |
| [porting/havequick.md](./porting/havequick.md) | HAVEQUICK UHF frequency hopping | High (classified) |
| [porting/link16.md](./porting/link16.md) | Link-16 tactical data link | High (COMSEC) |
| [porting/milstd188110.md](./porting/milstd188110.md) | MIL-STD-188-110 HF modem | Low (unclassified) |
| [porting/p25.md](./porting/p25.md) | APCO P25 public safety radio | Medium (proprietary codec) |

### Related Documentation

| Location | Description |
|----------|-------------|
| `waveform-spec/` | Waveform specification schema and examples |
| `vivado/` | Xilinx Vivado IP cores and build scripts |
| `lattice/` | Lattice FPGA designs and open-source toolchain |

## Documentation Map

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         R4W Documentation                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  OVERVIEW.md ────────────────────────────────────────────────────────┐  │
│  │ Vision, Architecture, Quick Start                                 │  │
│  │                                                                   │  │
│  ├───────────────────────────────────────────────────────────────────┤  │
│                                                                         │
│  Developer Guides                                                       │
│  ├── WAVEFORM_DEVELOPERS_GUIDE.md                                       │
│  │   └── Development → Debugging → Testing → Deployment                 │
│  │                                                                      │
│  ├── PHYSICAL_LAYER_GUIDE.md                                            │
│  │   └── Timing → HAL → RT Primitives → Config → Observability          │
│  │                                                                      │
│  ├── GNSS_GUIDE.md                                                      │
│  │   └── Waveforms → Scenario Engine → Pipeline Blocks → Precise Ephem  │
│  │                                                                      │
│  ├── TICK_SCHEDULER_GUIDE.md                                            │
│  │   └── Tick Events → Time Scaling → Sleep/Wake → DES Patterns         │
│  │                                                                      │
│  ├── REALTIME_SCHEDULER_GUIDE.md                                        │
│  │   └── TX/RX Coordination → FHSS → TDMA → Production Timing          │
│  │                                                                      │
│  ├── FPGA_DEVELOPERS_GUIDE.md                                           │
│  │   └── IP Cores → Register Maps → Integration → Collaboration         │
│  │                                                                      │
│  ├── SECURITY_GUIDE.md                                                  │
│  │   └── Memory → Crypto → Isolation → Deployment → Audit               │
│  │                                                                      │
│  └── ISOLATION_GUIDE.md                                                 │
│      └── Process → Container → VM → Hardware → Air Gap                  │
│                                                                         │
│  ├───────────────────────────────────────────────────────────────────┤  │
│                                                                         │
│  Porting Guides                                                         │
│  ├── PORTING_GUIDE_MILITARY.md (Overview)                               │
│  │                                                                      │
│  └── porting/                                                           │
│      ├── BUILD_PROCEDURES.md                                            │
│      ├── sincgars.md (VHF FHSS)                                         │
│      ├── havequick.md (UHF FHSS)                                        │
│      ├── link16.md (L-Band TDMA)                                        │
│      ├── milstd188110.md (HF PSK)                                       │
│      └── p25.md (Public Safety)                                         │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## Audience Quick Reference

| I want to... | Read... |
|--------------|---------|
| Understand R4W architecture | [OVERVIEW.md](../OVERVIEW.md) |
| Implement a new waveform | [WAVEFORM_DEVELOPERS_GUIDE.md](./WAVEFORM_DEVELOPERS_GUIDE.md) |
| Work with IQ sample formats | [IQ_FORMAT_GUIDE.md](./IQ_FORMAT_GUIDE.md) |
| Generate GNSS signals | [GNSS_GUIDE.md](./GNSS_GUIDE.md) |
| Use timing, HAL, RT primitives | [PHYSICAL_LAYER_GUIDE.md](./PHYSICAL_LAYER_GUIDE.md) |
| Use discrete event simulation | [TICK_SCHEDULER_GUIDE.md](./TICK_SCHEDULER_GUIDE.md) |
| Design FPGA accelerators | [FPGA_DEVELOPERS_GUIDE.md](./FPGA_DEVELOPERS_GUIDE.md) |
| Secure my deployment | [SECURITY_GUIDE.md](./SECURITY_GUIDE.md) |
| Isolate waveforms (containers, VMs) | [ISOLATION_GUIDE.md](./ISOLATION_GUIDE.md) |
| Port existing military waveforms | [PORTING_GUIDE_MILITARY.md](./PORTING_GUIDE_MILITARY.md) |
| Cross-compile for ARM | [porting/BUILD_PROCEDURES.md](./porting/BUILD_PROCEDURES.md) |
| Use the Pipeline Builder | [OVERVIEW.md](../OVERVIEW.md) → Pipeline Builder section |
| Use the GUI/CLI tools | [OVERVIEW.md](../OVERVIEW.md) → Quick Start |

## Contributing

Documentation improvements are welcome. When contributing:

1. Follow the existing document structure
2. Include diagrams using ASCII art for terminal compatibility
3. Provide code examples in Rust where applicable
4. Update this index when adding new documents

## Document Status

| Document | Status | Last Updated |
|----------|--------|--------------|
| OVERVIEW.md | Complete | 2026-02 |
| WAVEFORM_DEVELOPERS_GUIDE.md | Complete | 2025-12 |
| PHYSICAL_LAYER_GUIDE.md | Complete | 2025-12 |
| IQ_FORMAT_GUIDE.md | Complete | 2026-02 |
| GNSS_GUIDE.md | Complete | 2026-02 |
| TICK_SCHEDULER_GUIDE.md | Complete | 2025-12 |
| REALTIME_SCHEDULER_GUIDE.md | Complete | 2025-12 |
| FPGA_DEVELOPERS_GUIDE.md | Complete | 2025-12 |
| SECURITY_GUIDE.md | Complete | 2025-12 |
| ISOLATION_GUIDE.md | Complete | 2025-12 |
| PORTING_GUIDE_MILITARY.md | Complete | 2025-12 |
| porting/BUILD_PROCEDURES.md | Complete | 2025-12 |
| porting/sincgars.md | Complete | 2025-12 |
| porting/havequick.md | Complete | 2025-12 |
| porting/link16.md | Complete | 2025-12 |
| porting/milstd188110.md | Complete | 2025-12 |
| porting/p25.md | Complete | 2025-12 |

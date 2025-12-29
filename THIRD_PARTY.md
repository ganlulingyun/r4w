# Third-Party Licenses and Acknowledgments

This file documents third-party software, specifications, and intellectual property used or referenced by the R4W project.

## Rust Dependencies

All Rust dependencies are permissively licensed (MIT, Apache-2.0, BSD, or similar). Key dependencies include:

| Crate | License | Purpose |
|-------|---------|---------|
| rustfft | Apache-2.0/MIT | FFT algorithms |
| num-complex | MIT/Apache-2.0 | Complex number types |
| egui/eframe | MIT/Apache-2.0 | GUI framework |
| tokio | MIT | Async runtime |
| serde | MIT/Apache-2.0 | Serialization |
| tracing | MIT | Structured logging |
| libloading | ISC | Dynamic library loading |

For a complete list, run: `cargo tree --prefix none -f "{p} {l}"`

## FPGA Toolchain

The Lattice FPGA support uses open-source tools:

| Tool | License | Source |
|------|---------|--------|
| Yosys | ISC | https://github.com/YosysHQ/yosys |
| nextpnr | ISC | https://github.com/YosysHQ/nextpnr |
| Project IceStorm | ISC | https://github.com/YosysHQ/icestorm |
| Project Trellis | ISC | https://github.com/YosysHQ/prjtrellis |
| Icarus Verilog | GPL-2.0 | https://github.com/steveicarus/iverilog |

## Specifications and Standards

The following specifications are referenced or implemented:

| Specification | Source | Notes |
|---------------|--------|-------|
| LoRa Modulation | Semtech | Chirp Spread Spectrum is patented by Semtech |
| DMR (Digital Mobile Radio) | ETSI TS 102 361 | Open standard |
| TETRA | ETSI EN 300 392 | Open standard |
| P25 | TIA-102 | Open standard (voice codecs are proprietary) |
| OFDM | IEEE 802.11 | Foundational patents expired |

## Patent Notices

### LoRa / Chirp Spread Spectrum
LoRa modulation technology is patented by Semtech Corporation. This project implements LoRa for educational and research purposes. Commercial use may require licensing from Semtech.

### Voice Codecs (Not Implemented)
The IMBE and AMBE+2 voice codecs used by P25 and DMR are proprietary to Digital Voice Systems, Inc. (DVSI). This project does not implement these codecs; they are represented as stubs/interfaces only.

## References

- SDR-LoRa Paper: "SDR-LoRa, an open-source, full-fledged implementation of LoRa on Software-Defined-Radios"
- GNU Radio Project: https://gnuradio.org
- RustFFT: https://github.com/ejmahler/RustFFT

## Acknowledgments

- The Rust community for excellent SDR-related crates
- YosysHQ for open-source FPGA tooling
- GNU Radio project for pioneering open-source SDR

# Waveform Specification System

This directory contains the waveform specification schema and examples for defining new digital waveforms.

## Overview

The specification system provides a structured way to define waveforms that can be:
1. Used as documentation for existing waveforms
2. Fed to Claude as prompts to generate new implementations
3. Validated for consistency and feasibility
4. Used by the Waveform Wizard GUI

## Files

- `schema.yaml` - Complete specification schema with all parameters and documentation
- `examples/` - Example waveform specifications
  - `lpd-tactical.yaml` - LPD/LPI tactical communications waveform

## Schema Structure

```
waveform:           # Identity and classification
modulation:         # Modulation scheme (PSK, FSK, QAM, etc.)
spread_spectrum:    # DSSS, FHSS, CSS spreading
pulse_shaping:      # Spectral shaping filters
timing:             # Symbol/sample rates
synchronization:    # Preamble, sync words
channel_coding:     # FEC, interleaving, scrambling
framing:            # Packet structure
spectral:           # Bandwidth and PSD requirements
channel:            # Channel requirements (BER, SNR)
security:           # Encryption (optional)
implementation:     # Complexity hints
metrics:            # Calculated performance metrics
```

## LPD/LPI Waveform Design

For Low Probability of Detection/Intercept waveforms, key parameters are:

| Parameter | Typical Value | Purpose |
|-----------|---------------|---------|
| `spread_spectrum.dsss.chips_per_symbol` | 127-1023 | Processing gain |
| `spread_spectrum.dsss.pn_sequence.type` | gold, kasami | Good cross-correlation |
| `spectral.psd.target_dbm_hz` | < -140 | Below noise floor |
| `modulation.differential.enabled` | true | No phase reference needed |

### Processing Gain

Processing gain is the key LPD/LPI metric:

```
PG_dB = 10 * log10(chips_per_symbol)

Examples:
  31 chips:  PG = 15 dB
  63 chips:  PG = 18 dB
  127 chips: PG = 21 dB
  255 chips: PG = 24 dB
  511 chips: PG = 27 dB
  1023 chips: PG = 30 dB
```

### Signal Below Noise Floor

Thermal noise floor at room temperature: **-174 dBm/Hz**

With a 10 dB noise figure receiver: **-164 dBm/Hz**

To be undetectable, your signal PSD should be 10+ dB below this:
- Target PSD: **< -174 dBm/Hz** (absolute minimum)
- Practical target: **< -180 dBm/Hz** for good margin

## Usage

### As a Prompt Template

1. Copy `schema.yaml` or an example
2. Fill in your requirements
3. Provide to Claude with: "Implement this waveform: [paste YAML]"
4. Claude will generate the Rust implementation

### With the Waveform Wizard

1. Open the GUI waveform wizard
2. Answer the interactive questions
3. Wizard generates a complete specification
4. Export and optionally refine the YAML
5. Feed back to Claude for implementation

## Building Blocks Required

The specification references these building blocks (implemented in `r4w-core`):

### Waveforms

| Block | Module | Status |
|-------|--------|--------|
| BPSK/QPSK/8-PSK | `waveform/psk.rs` | ✅ Complete |
| 16/64/256-QAM | `waveform/qam.rs` | ✅ Complete |
| BFSK/4-FSK | `waveform/fsk.rs` | ✅ Complete |
| OFDM | `waveform/ofdm.rs` | ✅ Complete |
| CSS (LoRa) | `waveform/lora.rs` | ✅ Complete |
| DSSS | `waveform/dsss.rs` | ✅ Complete |
| FHSS | `waveform/fhss.rs` | ✅ Complete |
| AM/FM | `waveform/am.rs`, `fm.rs` | ✅ Complete |
| OOK/ASK | `waveform/ook.rs`, `ask.rs` | ✅ Complete |
| PPM | `waveform/ppm.rs` | ✅ Complete |
| CW | `waveform/cw.rs` | ✅ Complete |
| Zigbee/802.15.4 | `waveform/zigbee.rs` | ✅ Complete |
| UWB Impulse Radio | `waveform/uwb.rs` | ✅ Complete |
| FMCW Radar | `waveform/fmcw.rs` | ✅ Complete |
| ADS-B | `waveform/adsb.rs` | ✅ Complete |
| STANAG 4285 | `waveform/stanag4285.rs` | ✅ Complete |
| ALE | `waveform/ale.rs` | ✅ Complete |
| SINCGARS | `waveform/sincgars/` | ⚙️ Framework (~65%) |
| HAVEQUICK | `waveform/havequick/` | ⚙️ Framework (~70%) |
| Link-16 | `waveform/link16/` | ⚙️ Framework (~75%) |
| MIL-STD-188-110 | `waveform/milstd188110.rs` | ✅ Complete |
| P25 | `waveform/p25.rs` | ⚙️ Framework (~65%) |

**Framework Status Legend:**
- ✅ **Complete** - Fully functional, ready for use
- ⚙️ **Framework** - Unclassified signal processing complete; classified/proprietary components (hopping algorithms, TRANSEC, voice codecs) use simulator stubs. Percentage indicates unclassified work complete. See [PORTING_GUIDE_MILITARY.md](/docs/PORTING_GUIDE_MILITARY.md) for implementing operational versions.

### Spreading Codes

| Block | Module | Status |
|-------|--------|--------|
| LFSR (M-sequences) | `spreading/lfsr.rs` | ✅ Complete |
| Gold Codes | `spreading/gold.rs` | ✅ Complete |
| Barker Codes | `spreading/barker.rs` | ✅ Complete |

### Pulse Shaping Filters

| Block | Module | Status |
|-------|--------|--------|
| Root Raised Cosine (RRC) | `filters/pulse_shaping.rs` | ✅ Complete |
| Raised Cosine (RC) | `filters/pulse_shaping.rs` | ✅ Complete |
| Gaussian | `filters/pulse_shaping.rs` | ✅ Complete |

### Forward Error Correction

| Block | Module | Status |
|-------|--------|--------|
| Hamming Code | `coding.rs` | ✅ Complete |
| Gray Coding | `coding.rs` | ✅ Complete |
| Interleaving | `coding.rs` | ✅ Complete |
| Whitening (LFSR) | `whitening.rs` | ✅ Complete |
| Convolutional (K=7) | `waveform/stanag4285.rs` | ✅ Complete (in STANAG) |

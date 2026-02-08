# R4W Waveform Specifications

This directory contains unified waveform specifications that combine:
- **Waveform Specification**: The parameters and characteristics (WHAT)
- **Pipeline Implementation**: The signal processing chain (HOW)
- **TX/RX/Channel Separation**: Complete transceiver definitions (v1.1)

## Schema Version: r4w-waveform-spec v1.1

### Unified Format Structure

```yaml
waveform:           # Identity and classification
  name: "..."
  full_name: "..."
  description: "..."
  classification: { type, category }
  standards: [...]

modulation:         # Modulation parameters
  domain: "phase|frequency|amplitude|hybrid"
  scheme: "BPSK|QPSK|16-QAM|FSK|..."
  order: N
  bits_per_symbol: N
  constellation: { type, points, gray_coded }

spread_spectrum:    # Optional spread spectrum
  enabled: true|false
  technique: "dsss|fhss|css|thss"
  ...

pulse_shaping:      # Pulse shaping filter
  enabled: true|false
  filter: { type, rolloff, span_symbols }

timing:             # Timing parameters
  symbol_rate: N
  sample_rate: N
  samples_per_symbol: N

channel_coding:     # FEC, interleaving, scrambling
  enabled: true|false
  fec: { type, rate }
  ...

spectral:           # Spectral characteristics
  bandwidth: { occupied_99_hz, null_to_null_hz }
  spectral_efficiency: N.N

# NEW in v1.1: Separate TX/RX/Channel sections
tx:                 # Transmitter pipeline
  description: "..."
  blocks:
    - id: 1-99
      name: "..."
      type: "..."
      enabled: true|false
      ...parameters
  connections:
    - from: [block_id, port]
      to: [block_id, port]

rx:                 # Receiver pipeline
  description: "..."
  blocks:
    - id: 100-199
      name: "..."
      type: "..."
  connections: [...]

channel:            # Channel model (optional)
  description: "..."
  blocks:
    - id: 200-299
      name: "..."
      type: "..."
  connections: [...]

metrics:            # Performance metrics
  theoretical_ber: { awgn, rayleigh }
  required_snr_db: { ber_1e-3, ber_1e-5 }

implementation:     # Rust implementation hints
  module: "..."
  struct_name: "..."
  existing: true|false
  path: "..."
```

### Block ID Conventions

- **TX blocks**: IDs 1-99
- **RX blocks**: IDs 100-199
- **Channel blocks**: IDs 200-299

This allows the loopback mode to combine all sections and automatically connect TX → Channel → RX.

## Available Specs

| File | Waveform | Type | TX/RX | Description |
|------|----------|------|-------|-------------|
| `cw.yaml` | CW | Analog | ✓ | Continuous Wave (unmodulated carrier) |
| `bpsk.yaml` | BPSK | Digital | ✓ | Binary Phase Shift Keying |
| `qpsk.yaml` | QPSK | Digital | ✓ | Quadrature Phase Shift Keying |
| `fsk.yaml` | 2-FSK | Digital | ✓ | Binary Frequency Shift Keying |
| `lora.yaml` | LoRa | Spread Spectrum | ✓ | Chirp Spread Spectrum (CSS) |

## Usage

### In Pipeline Builder
1. Click **Load** in the toolbar
2. Select a waveform from the dropdown
3. Choose which section to load:
   - **TX Pipeline** - Transmitter blocks only
   - **RX Pipeline** - Receiver blocks only
   - **Loopback** - TX → Channel → RX (complete system)
   - **Channel Only** - Just the channel model

### In CLI
```bash
# Future: r4w waveform generate --spec specs/bpsk.yaml --mode tx
```

### Programmatically
```rust
let yaml = std::fs::read_to_string("specs/bpsk.yaml")?;

// Detect available sections
let (has_tx, has_rx, has_channel, _) = Pipeline::detect_spec_sections(&yaml);

// Load specific section
let tx_pipeline = Pipeline::from_yaml_with_mode(&yaml, LoadMode::TxOnly)?;
let rx_pipeline = Pipeline::from_yaml_with_mode(&yaml, LoadMode::RxOnly)?;
let loopback = Pipeline::from_yaml_with_mode(&yaml, LoadMode::Loopback)?;
```

## Block Types Reference

### Source Blocks
- `Bit Source` - Random/pattern bit generation
- `Symbol Source` - Symbol stream generation
- `File Source` - Read from file

### Coding Blocks
- `Scrambler` - LFSR-based whitening
- `FEC Encoder` - Convolutional, LDPC, Turbo, Hamming, Reed-Solomon
- `Interleaver` - Block interleaving

### Mapping Blocks
- `Gray Mapper` - Gray code bit-to-symbol mapping
- `Constellation Mapper` - Direct constellation mapping

### Modulation Blocks
- `PSK Modulator` - BPSK, QPSK, 8-PSK, etc.
- `QAM Modulator` - 16-QAM, 64-QAM, 256-QAM
- `FSK Modulator` - 2-FSK, 4-FSK, GFSK
- `CSS Modulator` - LoRa chirp modulation

### Filtering Blocks
- `RRC Filter` - Root Raised Cosine
- `Pulse Shaper` - RRC, Gaussian, Rectangular
- `FIR Filter` - General FIR filtering

### Recovery Blocks (RX)
- `AGC` - Automatic Gain Control
- `Timing Recovery` - Gardner, Mueller-Muller, Early-Late
- `Carrier Recovery` - Costas Loop, Decision-Directed

### Demodulation Blocks (RX)
- `PSK Demodulator` - Phase demodulation
- `QAM Demodulator` - Amplitude/phase demodulation
- `FSK Demodulator` - Frequency demodulation

### Channel Blocks
- `AWGN Channel` - Additive White Gaussian Noise
- `Fading Channel` - Rayleigh/Rician fading
- `Frequency Offset` - CFO simulation

### Output Blocks
- `Bit Output` - Bit stream sink
- `IQ Output` - IQ sample sink
- `File Output` - Write to file

## Creating New Specs

1. Copy an existing spec as a template
2. Fill in the waveform parameters
3. Define TX, RX, and Channel pipeline sections
4. Use standard block ID conventions (TX: 1-99, RX: 100-199, Channel: 200-299)
5. Test by loading in Pipeline Builder
6. Verify all three modes work: TX only, RX only, and Loopback

## Block Metadata

Each block type has associated metadata including:
- **Implementation**: Rust source file location
- **Formulas**: Mathematical equations
- **Tests**: Unit tests that can be run
- **Performance**: Complexity, SIMD/GPU support
- **Standards**: Relevant specifications

View this information in the Pipeline Builder by selecting a block and expanding the Documentation sections in the Properties panel.

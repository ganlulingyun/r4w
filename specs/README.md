# R4W Waveform Specifications

This directory contains unified waveform specifications that combine:
- **Waveform Specification**: The parameters and characteristics (WHAT)
- **Pipeline Implementation**: The signal processing chain (HOW)

## Schema Version: r4w-waveform-spec v1.0

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

pipeline:           # Implementation pipeline
  blocks:
    - id: N
      name: "..."
      type: "..."
      enabled: true|false
      ...parameters
  connections:
    - from: [block_id, port]
      to: [block_id, port]

metrics:            # Performance metrics
  theoretical_ber: { awgn, rayleigh }
  required_snr_db: { ber_1e-3, ber_1e-5 }

implementation:     # Rust implementation hints
  module: "..."
  struct_name: "..."
  existing: true|false
  path: "..."
```

## Available Specs

| File | Waveform | Description |
|------|----------|-------------|
| `cw.yaml` | CW | Continuous Wave (unmodulated carrier) |
| `bpsk.yaml` | BPSK | Binary Phase Shift Keying |
| `qpsk.yaml` | QPSK | Quadrature Phase Shift Keying |

## Usage

### In Pipeline Builder
1. Click **Load** in the toolbar
2. Select a `.yaml` file
3. Pipeline loads with auto-layout

### In CLI
```bash
# Future: r4w waveform generate --spec specs/bpsk.yaml
```

### Programmatically
```rust
let yaml = std::fs::read_to_string("specs/bpsk.yaml")?;
let pipeline = Pipeline::from_yaml(&yaml)?;
```

## Creating New Specs

1. Copy an existing spec as a template
2. Fill in the waveform parameters
3. Define the pipeline blocks and connections
4. Test by loading in Pipeline Builder

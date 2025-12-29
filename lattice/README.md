# R4W Lattice FPGA Support

Open-source FPGA acceleration for R4W waveform generation targeting Lattice iCE40 and ECP5 devices.

## Supported Devices

| Device | Package | Board Examples | Clock | Features |
|--------|---------|----------------|-------|----------|
| iCE40-HX8K | CT256 | iCEstick, TinyFPGA BX | 48 MHz | Basic LoRa chirp, LUT-based NCO |
| ECP5-25K | CABGA381 | ULX3S, OrangeCrab | 100 MHz (PLL) | Full NCO, SDRAM support |

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      R4W Top Level                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │  SPI Slave   │  │  Chirp Gen   │  │        NCO           │   │
│  │              │──│  (LoRa CSS)  │  │  (LUT or CORDIC)     │   │
│  │  16-bit addr │  │              │  │                      │   │
│  │  32-bit data │  │  SF 5-12     │  │  24-bit phase acc    │   │
│  └──────────────┘  └──────────────┘  └──────────────────────┘   │
│         │                 │                    │                │
│         └─────────────────┴────────────────────┘                │
│                           │                                     │
│                    ┌──────┴──────┐                              │
│                    │   I/Q MUX   │                              │
│                    │   12-bit    │                              │
│                    └──────┬──────┘                              │
│                           │                                     │
│                    ┌──────┴──────┐                              │
│                    │   DAC Out   │                              │
│                    │  (GPIO/SPI) │                              │
│                    └─────────────┘                              │
└─────────────────────────────────────────────────────────────────┘
```

## Register Map

| Address | Name | Description |
|---------|------|-------------|
| 0x0000 | CTRL | Control register (start, upchirp, NCO enable, reset) |
| 0x0004 | SF | Spreading factor (5-12) |
| 0x0008 | STATUS | Status (busy, done, PLL locked) |
| 0x000C | SYMBOL | Symbol value for chirp modulation |
| 0x0010 | NCO_FREQ | NCO frequency word (24-bit) |
| 0x0014 | NCO_PHASE | NCO phase offset (24-bit) |
| 0x0018 | DATA_I | I channel output (read-only) |
| 0x001C | DATA_Q | Q channel output (read-only) |
| 0x0020 | ID | IP identification (R4LI/R4LE) |
| 0x0024 | VERSION | IP version |

### Control Register (0x0000)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | START | Start chirp generation (auto-clear) |
| 1 | UPCHIRP | 1=upchirp, 0=downchirp |
| 4 | NCO_EN | Enable standalone NCO output |
| 7 | SOFT_RST | Soft reset (auto-clear) |

## Toolchain Requirements

Open-source toolchain only:

- **Yosys** - Synthesis
- **nextpnr-ice40** / **nextpnr-ecp5** - Place & Route
- **IceStorm** - iCE40 bitstream tools (icepack, iceprog)
- **Project Trellis** - ECP5 bitstream tools (ecppack)
- **Icarus Verilog** - Simulation
- **openFPGALoader** - Universal FPGA programmer

### Installation (Ubuntu/Debian)

```bash
# Yosys and nextpnr
sudo apt install yosys nextpnr-ice40 nextpnr-ecp5

# IceStorm tools
sudo apt install fpga-icestorm

# Project Trellis
sudo apt install prjtrellis-tools

# Simulation
sudo apt install iverilog gtkwave

# Programming
sudo apt install openfpgaloader
```

## Building

```bash
cd lattice/scripts

# Build for iCE40-HX8K (default)
make ice40

# Build for ECP5-25K
make ecp5

# Run simulation
make sim

# View waveforms
make wave

# Lint check
make lint

# Resource report
make report_ice40
make report_ecp5
```

## Programming

```bash
# Program iCE40 via FTDI
make prog_ice40

# Program ECP5 (ULX3S)
make prog_ecp5
```

## SPI Protocol

The SPI interface uses a simple command protocol:

### Write Transaction
```
CS low
[CMD:8] [ADDR:16] [DATA:32]
CS high

CMD bit 7 = 1 (write)
```

### Read Transaction
```
CS low
[CMD:8] [ADDR:16] [DATA:32 dummy TX, receive on MISO]
CS high

CMD bit 7 = 0 (read)
```

### Python Example

```python
import spidev

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0

def write_reg(addr, data):
    tx = [0x80]  # Write command
    tx += [(addr >> 8) & 0xFF, addr & 0xFF]
    tx += [(data >> 24) & 0xFF, (data >> 16) & 0xFF,
           (data >> 8) & 0xFF, data & 0xFF]
    spi.xfer2(tx)

def read_reg(addr):
    tx = [0x00]  # Read command
    tx += [(addr >> 8) & 0xFF, addr & 0xFF]
    tx += [0x00, 0x00, 0x00, 0x00]  # Dummy bytes
    rx = spi.xfer2(tx)
    return (rx[3] << 24) | (rx[4] << 16) | (rx[5] << 8) | rx[6]

# Read IP ID
ip_id = read_reg(0x0020)
print(f"IP ID: 0x{ip_id:08X}")

# Generate upchirp with SF7, symbol 0
write_reg(0x0004, 7)      # SF = 7
write_reg(0x000C, 0)      # Symbol = 0
write_reg(0x0000, 0x03)   # Start upchirp
```

## Directory Structure

```
lattice/
├── design/
│   ├── r4w_top_ice40.v       # iCE40 top-level
│   ├── r4w_top_ecp5.v        # ECP5 top-level with PLL
│   └── constraints/
│       ├── ice40_hx8k.pcf    # iCE40 pin constraints
│       └── ecp5_25k.lpf      # ECP5 pin constraints
├── ip/
│   ├── r4w_spi_slave/        # SPI slave interface
│   ├── r4w_nco/              # NCO (LUT/CORDIC)
│   └── r4w_chirp_gen/        # LoRa chirp generator
├── sim/
│   └── tb_r4w_top_ice40.v    # Testbench
└── scripts/
    └── Makefile              # Build automation
```

## Resource Usage

### iCE40-HX8K

| Resource | Used | Available | % |
|----------|------|-----------|---|
| LUTs | ~1200 | 7680 | 16% |
| FFs | ~400 | 7680 | 5% |
| BRAMs | 2 | 32 | 6% |

### ECP5-25K

| Resource | Used | Available | % |
|----------|------|-----------|---|
| LUTs | ~2000 | 24288 | 8% |
| FFs | ~600 | 24288 | 2% |
| BRAMs | 0 | 56 | 0% |
| PLLs | 1 | 2 | 50% |

## Pin Mapping Notes

### iCE40 (iCEstick-compatible)
- SPI on PMOD header pins
- DAC on GPIO bank
- 4 status LEDs

### ECP5 (ULX3S-compatible)
- 25 MHz oscillator input
- SPI on GPIO header
- 12-bit DAC I/Q on GPIO banks
- 8 status LEDs
- SDRAM interface (reserved for future)

## Future Enhancements

- [ ] DMA for continuous streaming
- [ ] SDRAM sample buffer (ECP5)
- [ ] Multiple waveform support
- [ ] ADC input for RX path
- [ ] Higher-order interpolation

# R4W Vivado FPGA Design

FPGA acceleration layer for R4W LoRa SDR implementation, targeting Xilinx Zynq-7020 (PYNQ-Z2).

## IP Cores

| Core | Description | Resources (est.) |
|------|-------------|------------------|
| r4w_fft | 1024-point FFT/IFFT | 15K LUTs, 32 DSPs |
| r4w_fir | 256-tap FIR filter | 8K LUTs, 128 DSPs |
| r4w_chirp_gen | LoRa chirp generator | 2K LUTs, 4 DSPs |
| r4w_chirp_corr | LoRa chirp correlator | 12K LUTs, 16 DSPs |
| r4w_nco | Numerically controlled oscillator | 1.5K LUTs |

## Address Map

| Address | Size | IP Core |
|---------|------|---------|
| 0x4000_0000 | 64KB | r4w_fft |
| 0x4001_0000 | 64KB | r4w_fir |
| 0x4002_0000 | 64KB | r4w_chirp_gen |
| 0x4003_0000 | 64KB | r4w_chirp_corr |
| 0x4004_0000 | 64KB | r4w_nco |
| 0x4040_0000 | 64KB | AXI DMA |

## Register Maps

All IP cores follow consistent register layouts matching `crates/r4w-fpga/src/zynq/registers.rs`.

### Common Register Structure
- 0x00: CTRL - Control register (START, RESET, mode bits)
- 0x04-0x0C: Configuration registers
- 0x08: STATUS - Status register (DONE, BUSY, ERROR)
- 0x10+: Data I/O registers
- 0x1C: VERSION - IP core version (read-only)
- 0x20: ID - IP core identifier (read-only)

### I/Q Data Format
- 32-bit packed: [31:16] = I (signed 16-bit), [15:0] = Q (signed 16-bit)
- Full-scale: ±32767 (0x7FFF)

## Building

### Prerequisites
- Xilinx Vivado 2023.2 or later
- PYNQ-Z2 board files installed

### Create Project
```bash
cd vivado
vivado -mode batch -source scripts/build_project.tcl
```

### Generate Bitstream
```bash
vivado -mode batch -source scripts/build_bitstream.tcl
```

Output: `output/r4w_design.bit`

### Package IP Cores (for reuse)
```bash
vivado -mode batch -source scripts/package_ip.tcl
```

## Deployment to PYNQ-Z2

### Copy Files
```bash
scp output/r4w_design.bit xilinx@pynq:/home/xilinx/
scp device-tree/r4w-overlay.dts xilinx@pynq:/home/xilinx/
```

### Compile Device Tree
```bash
# On PYNQ
dtc -O dtb -o r4w-overlay.dtbo -b 0 -@ r4w-overlay.dts
```

### Load Bitstream
```bash
# On PYNQ
sudo fpgautil -b r4w_design.bit -o r4w-overlay.dtbo
```

### Verify
```bash
# Check devices appeared
ls /dev/uio*
cat /proc/device-tree/r4w_fft@40000000/compatible
```

## Testing with Rust

```bash
# Build with Zynq feature
cargo build --release -p r4w-fpga --features zynq

# Run hardware test
sudo ./target/release/examples/fpga_test
```

## Directory Structure

```
vivado/
├── ip/
│   ├── common/           # Shared modules
│   │   ├── axi_lite_slave.v
│   │   └── iq_pack.v
│   ├── r4w_fft/
│   ├── r4w_fir/
│   ├── r4w_chirp_gen/
│   ├── r4w_chirp_corr/
│   └── r4w_nco/
├── design/
│   └── constraints/
│       └── pynq_z2.xdc
├── scripts/
│   ├── build_project.tcl
│   ├── build_bd.tcl
│   ├── build_bitstream.tcl
│   └── package_ip.tcl
├── device-tree/
│   ├── r4w-overlay.dts
│   └── pl.dtsi
├── sim/                  # Testbenches (TODO)
└── output/               # Build outputs
```

## Technical Notes

### CORDIC Implementation
The NCO and chirp cores use CORDIC for sin/cos generation:
- 16-stage iterative pipeline
- 16-bit output precision
- ~17 clock latency per sample

### FFT Architecture
- Wraps Xilinx FFT IP (placeholder in current version)
- Radix-4, burst I/O mode
- Scaled arithmetic to prevent overflow
- 64-1024 point runtime configurable

### Chirp Generation
LoRa chirp phase: φ(n) = 2π × [n²/(2N) + symbol×n/N]
- Frequency linearly sweeps from 0 to BW
- N = 2^SF samples per symbol
- CORDIC generates I = cos(φ), Q = sin(φ)

### Chirp Correlation (Demodulation)
1. Multiply received samples by conjugate downchirp
2. FFT of dechirped signal
3. Peak bin = symbol value
4. Peak magnitude = confidence

## License

Part of the R4W SDR project. See main repository LICENSE.

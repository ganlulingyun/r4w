# Workshop 01: Getting Started with R4W

**Duration:** 30 minutes
**Difficulty:** Beginner
**Prerequisites:** Rust toolchain, Git

## Objectives

By the end of this workshop, you will:
- Have R4W installed and running
- Understand the project structure
- Run your first modulation/demodulation
- Navigate the CLI and Explorer GUI

---

## 1. Installation

### 1.1 Clone the Repository

```bash
git clone https://github.com/joemooney/r4w.git
cd r4w
```

### 1.2 Build the Project

```bash
# Release build (faster execution)
cargo build --release

# Debug build (faster compilation, slower execution)
cargo build
```

### 1.3 Verify Installation

```bash
# Run tests to verify everything works
cargo test --workspace
```

Expected output: All tests should pass!

---

## 2. Project Structure

```
r4w/
├── crates/
│   ├── r4w-core/      # Core DSP algorithms and waveforms
│   ├── r4w-sim/       # Simulation and channel models
│   ├── r4w-gui/       # Explorer GUI (egui)
│   ├── r4w-cli/       # Command-line interface
│   └── r4w-web/       # WebAssembly entry point
├── workshops/         # You are here!
├── OVERVIEW.md        # Architecture and vision
├── CLAUDE.md          # Development guidance
└── Cargo.toml         # Workspace manifest
```

### Key Crates

| Crate | Purpose |
|-------|---------|
| `r4w-core` | Modulation, demodulation, FFT, filters, waveform definitions |
| `r4w-sim` | Channel simulation (AWGN, Rayleigh, Rician), hardware abstraction |
| `r4w-gui` | Visual exploration tool - see signals in real-time |
| `r4w-cli` | Command-line tool for scripting and batch processing |

---

## 3. Your First CLI Commands

### 3.1 Get Help

```bash
cargo run --bin r4w -- --help
```

This shows all available commands.

### 3.2 List Available Waveforms

```bash
cargo run --bin r4w -- waveform --list
```

You should see waveforms like:
- `bpsk` - Binary Phase Shift Keying
- `qpsk` - Quadrature PSK
- `lora` - LoRa (Chirp Spread Spectrum)
- `fsk` - Frequency Shift Keying
- And more!

### 3.3 Get Waveform Info

```bash
cargo run --bin r4w -- waveform --info lora
```

This shows details about the LoRa waveform:
- Spreading Factor (SF)
- Bandwidth (BW)
- Bit rate calculation
- Symbols per chirp

### 3.4 Modulate a Message

```bash
cargo run --bin r4w -- modulate --message "Hello R4W!" --waveform bpsk
```

This converts text to I/Q samples using BPSK modulation.

---

## 4. Launch the Explorer GUI

The Explorer is your visual laboratory for understanding waveforms.

```bash
cargo run --bin r4w-explorer
```

### 4.1 What You'll See

When Explorer launches, you'll see:

```
┌─────────────────────────────────────────────────────────────┐
│  R4W Explorer                                        [- □ x]│
├─────────────────────────────────────────────────────────────┤
│ ┌─Waveform Control──┐  ┌─Constellation─────────────────────┐│
│ │ Waveform: [BPSK ▼]│  │         *                         ││
│ │ Message: [Hello  ]│  │    *         *                    ││
│ │ SNR: [10.0] dB    │  │         *                         ││
│ │ [Generate] [Demod]│  └───────────────────────────────────┘│
│ └───────────────────┘                                       │
│ ┌─Time Domain──────────────────────────────────────────────┐│
│ │  /\  /\  /\  /\  /\  /\                                  ││
│ │ /  \/  \/  \/  \/  \/  \                                 ││
│ └──────────────────────────────────────────────────────────┘│
│ ┌─Spectrum (FFT)───────────────────────────────────────────┐│
│ │    ▄██▄                                                  ││
│ │  ▄██████▄                                                ││
│ └──────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

### 4.2 Quick Tour

1. **Waveform Selector**: Choose which modulation to explore
2. **Message Input**: Enter text to modulate
3. **SNR Slider**: Add noise to simulate real-world conditions
4. **Constellation View**: See the symbol positions (dots)
5. **Time Domain**: See the actual I/Q waveform
6. **Spectrum**: See the frequency content (FFT)

---

## 5. Hands-On Exercise: Compare Waveforms

### Task: Compare BPSK and QPSK

1. Launch Explorer: `cargo run --bin r4w-explorer`
2. Set message to "HELLO"
3. Select BPSK - note the constellation (2 points)
4. Switch to QPSK - now 4 points!
5. Try 8-PSK - 8 points around a circle

### Questions to Consider:

- How does the number of points affect data rate?
- What happens when you add noise (lower SNR)?
- Which modulation is more robust to noise?

---

## 6. Run in Browser (Optional)

R4W can run in your web browser using WebAssembly!

```bash
# Install trunk if not already
cargo install trunk

# Serve the web app
cd crates/r4w-web
trunk serve
```

Then open http://localhost:8080 in your browser.

---

## 7. Key Takeaways

1. **R4W has two main interfaces**: CLI for scripting, Explorer for learning
2. **Waveforms are pluggable**: Many modulation types available
3. **Everything is I/Q samples**: The fundamental data type
4. **Simulation built-in**: Add noise, channel effects, etc.

---

## 8. Next Steps

Continue to [Workshop 02: Understanding I/Q Signals](02-iq-signals.md) to learn the fundamentals of complex samples that power all of R4W.

---

## Quick Reference Card

```
┌────────────────────────────────────────────┐
│         R4W Quick Reference                │
├────────────────────────────────────────────┤
│ GUI:     cargo run --bin r4w-explorer      │
│ CLI:     cargo run --bin r4w -- <command>  │
│ Web:     trunk serve (in crates/r4w-web)   │
│ Test:    cargo test --workspace            │
│ Build:   cargo build --release             │
├────────────────────────────────────────────┤
│ CLI Commands:                              │
│   waveform --list    List all waveforms    │
│   waveform --info X  Info about waveform X │
│   modulate           Convert text to IQ    │
│   simulate           Add channel effects   │
├────────────────────────────────────────────┤
│ Key Types:                                 │
│   IQSample    Complex<f64> (I + jQ)        │
│   Waveform    Modulator/demodulator trait  │
│   SdrDevice   Hardware abstraction         │
└────────────────────────────────────────────┘
```

---

## Troubleshooting

### "cargo command not found"
Install Rust: https://rustup.rs

### "failed to compile"
Update Rust: `rustup update`

### GUI doesn't launch
On Linux, you may need: `sudo apt install libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev`

### Tests fail
Ensure you're on the latest main branch: `git pull origin master`

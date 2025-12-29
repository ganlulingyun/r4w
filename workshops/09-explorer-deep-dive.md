# Workshop 09: R4W Explorer Deep Dive

**Duration:** 60 minutes
**Difficulty:** Beginner to Intermediate
**Prerequisites:** Workshop 01 completed (others helpful but not required)

## Objectives

By the end of this workshop, you will:
- Know every control in the Explorer GUI
- Understand what each visualization shows
- Be able to diagnose signal problems visually
- Use Explorer as a learning and debugging tool

---

## 1. Launching Explorer

```bash
cargo run --bin r4w-explorer
```

You'll see the main window with multiple panels.

---

## 2. The Main Window Layout

```
┌─────────────────────────────────────────────────────────────────────┐
│  R4W Explorer                                              [─ □ ×]  │
├─────────────────────────────────────────────────────────────────────┤
│ ┌─────────────────────┐ ┌───────────────────────────────────────────┤
│ │                     │ │                                           │
│ │   Control Panel     │ │         Constellation Diagram             │
│ │                     │ │                                           │
│ │   ┌─Waveform────┐   │ │              Q                            │
│ │   │ QPSK      ▼ │   │ │              ▲                            │
│ │   └─────────────┘   │ │          ●   │   ●                        │
│ │                     │ │              │                            │
│ │   ┌─Parameters──┐   │ │         ─────┼─────                       │
│ │   │ SNR: [20] dB│   │ │              │                            │
│ │   │ Rate: [1000]│   │ │          ●   │   ●                        │
│ │   └─────────────┘   │ │              ▼                            │
│ │                     │ │              I                            │
│ │   ┌─Message─────┐   │ │                                           │
│ │   │ Hello R4W!  │   │ │                                           │
│ │   └─────────────┘   │ │                                           │
│ │                     │ │                                           │
│ │   [▶ Generate]      │ │                                           │
│ │   [⟳ Demodulate]    │ └───────────────────────────────────────────┤
│ │                     │ ┌───────────────────────────────────────────┤
│ │   ┌─Channel─────┐   │ │                                           │
│ │   │ [✓] AWGN    │   │ │         Time Domain (I/Q)                 │
│ │   │ [ ] Fading  │   │ │                                           │
│ │   └─────────────┘   │ │   I: ╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲                 │
│ │                     │ │   Q: ╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱                 │
│ └─────────────────────┘ │                                           │
│                         └───────────────────────────────────────────┤
│ ┌───────────────────────────────────────────────────────────────────┤
│ │                                                                   │
│ │                     Spectrum / FFT                                │
│ │                                                                   │
│ │    ▁▂▃▅██████▅▃▂▁                                                 │
│ │   ───────┼───────                                                 │
│ │          center                                                   │
│ │                                                                   │
│ └───────────────────────────────────────────────────────────────────┤
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3. Control Panel - Waveform Selection

### 3.1 Waveform Dropdown

| Waveform | What It Does |
|----------|--------------|
| **CW** | Continuous Wave - just a tone |
| **OOK** | On-Off Keying - carrier on/off |
| **ASK** | Amplitude Shift Keying - multiple amplitudes |
| **BPSK** | Binary PSK - 2 phases (0°, 180°) |
| **QPSK** | Quadrature PSK - 4 phases |
| **8-PSK** | 8 phases - 3 bits/symbol |
| **16-QAM** | 16-point constellation |
| **64-QAM** | 64-point constellation |
| **FSK** | Frequency shift keying |
| **GFSK** | Gaussian FSK (Bluetooth) |
| **LoRa** | Chirp spread spectrum |

### 3.2 When to Use Each

| Goal | Choose |
|------|--------|
| Just learning | Start with BPSK |
| See phase changes | QPSK or 8-PSK |
| See amplitude+phase | 16-QAM |
| Maximum efficiency | 64-QAM |
| Long range | LoRa |
| Simple on/off | OOK |

---

## 4. Control Panel - Parameters

### 4.1 SNR (Signal-to-Noise Ratio)

```
SNR Slider: [0 dB ════════●════════ 30 dB]
```

| Value | What You'll See |
|-------|-----------------|
| 30 dB | Perfect constellation points |
| 20 dB | Slight fuzziness |
| 10 dB | Noticeable clouds |
| 5 dB | Points start merging |
| 0 dB | At noise floor |
| -5 dB | Below noise (spread spectrum only) |

**Try this:**
1. Set BPSK, SNR = 30 dB → Clean dots
2. Lower to 5 dB → Fuzzy clouds
3. Switch to 16-QAM at 5 dB → Errors!
4. Switch to LoRa at -5 dB → Still works!

### 4.2 Symbol Rate

```
Symbol Rate: [1000] symbols/sec
```

Controls how fast symbols are transmitted:
- Higher rate → More data, wider bandwidth
- Lower rate → Less data, narrower bandwidth

**Watch the spectrum:**
- Low rate: Narrow peak
- High rate: Wide peak

### 4.3 Spreading Factor (LoRa only)

```
SF: [7 ═══●═══════════ 12]
```

| SF | Symbols | Range | Speed |
|----|---------|-------|-------|
| 7 | 128 | Shorter | Fastest |
| 8 | 256 | ↓ | ↓ |
| 9 | 512 | ↓ | ↓ |
| 10 | 1024 | ↓ | ↓ |
| 11 | 2048 | ↓ | ↓ |
| 12 | 4096 | Longest | Slowest |

### 4.4 Bandwidth (LoRa only)

```
BW: ○ 125 kHz  ● 250 kHz  ○ 500 kHz
```

| BW | Effect |
|----|--------|
| 125 kHz | Longest range, slowest |
| 250 kHz | Balanced |
| 500 kHz | Shortest range, fastest |

---

## 5. Control Panel - Message

### 5.1 Text Input

```
┌─Message───────────────┐
│ Hello R4W!            │
└───────────────────────┘
```

- Type any text to modulate
- ASCII characters converted to bits
- Longer messages = more symbols

### 5.2 Special Patterns

Try these for testing:
- `AAAA` - Repeated pattern
- `0000` - All zeros in ASCII
- Single letter - Minimal symbols

---

## 6. Control Panel - Actions

### 6.1 Generate Button

```
[▶ Generate]
```

What it does:
1. Converts message to bits
2. Runs modulation algorithm
3. Applies channel effects (noise, fading)
4. Updates all visualizations

### 6.2 Demodulate Button

```
[⟳ Demodulate]
```

What it does:
1. Takes received samples
2. Runs demodulation algorithm
3. Converts symbols back to bits
4. Shows decoded message

### 6.3 Clear Button

```
[✕ Clear]
```

Resets all displays and internal state.

---

## 7. Control Panel - Channel Effects

### 7.1 AWGN Checkbox

```
[✓] AWGN Noise
```

Adds white Gaussian noise based on SNR setting.

**Without AWGN:** Perfect signal (unrealistic)
**With AWGN:** Realistic thermal noise

### 7.2 Fading Checkbox

```
[ ] Rayleigh Fading
[✓] Rician Fading (K=10)
```

| Type | When to Use |
|------|-------------|
| None | Testing modulation only |
| Rayleigh | Indoor, NLOS scenarios |
| Rician | Outdoor with line-of-sight |

### 7.3 Multipath Checkbox

```
[ ] Multipath (3-tap)
```

Adds delayed copies of the signal, simulating reflections.

---

## 8. Constellation Diagram

### 8.1 What It Shows

The constellation displays **where symbols land** in the I/Q plane:

```
            Q (Quadrature)
            ▲
     ●      │      ●      ← QPSK: 4 points
            │
   ─────────┼─────────► I (In-phase)
            │
     ●      │      ●
```

### 8.2 Reading the Constellation

| Appearance | Meaning |
|------------|---------|
| Tight dots | Clean signal, low noise |
| Fuzzy clouds | Noise present |
| Rotating pattern | Frequency offset |
| Stretched/skewed | I/Q imbalance |
| Wrong number of points | Wrong waveform selected |

### 8.3 Interactive Features

- **Zoom**: Scroll wheel
- **Pan**: Click and drag
- **Hover**: See symbol values

### 8.4 Exercise: Noise Visualization

1. Select QPSK
2. SNR = 30 dB → 4 tight points
3. SNR = 15 dB → Small clouds
4. SNR = 5 dB → Clouds overlap!

---

## 9. Time Domain Display

### 9.1 What It Shows

The I and Q components over time:

```
I Component:
  ╱╲    ╱╲    ╱╲    ╱╲
 ╱  ╲  ╱  ╲  ╱  ╲  ╱  ╲
─────╲╱────╲╱────╲╱────╲╱──

Q Component:
    ╱╲    ╱╲    ╱╲    ╱╲
   ╱  ╲  ╱  ╲  ╱  ╲  ╱  ╲
──╲╱────╲╱────╲╱────╲╱─────
```

### 9.2 What to Look For

| Pattern | Meaning |
|---------|---------|
| Smooth sine waves | CW or unmodulated carrier |
| Step changes | Symbol boundaries |
| Random-looking | DSSS or spread spectrum |
| Bursts of carrier | OOK |
| Constant amplitude | PSK or FSK |
| Varying amplitude | ASK or QAM |

### 9.3 Zoom Controls

- **Horizontal zoom**: See more/fewer symbols
- **Vertical zoom**: See amplitude detail
- **Time cursor**: Click to see exact values

---

## 10. Spectrum Display (FFT)

### 10.1 What It Shows

The **frequency content** of your signal:

```
Power (dB)
   ▲
   │      ▄██▄
   │    ▄██████▄
   │  ▄████████████▄
   │▄██████████████████▄
   └────────────────────► Frequency
          center
```

### 10.2 Reading the Spectrum

| Feature | Meaning |
|---------|---------|
| Main lobe width | Signal bandwidth |
| Peak height | Signal power |
| Noise floor | Background level |
| Side lobes | Spectral leakage |

### 10.3 Bandwidth Observations

Try this experiment:
1. Set symbol rate = 1000
2. Note the main lobe width
3. Set symbol rate = 2000
4. Width doubles!

**Rule:** Bandwidth ≈ Symbol Rate × (1 + roll-off)

### 10.4 Spectrum vs Modulation

| Waveform | Spectrum Shape |
|----------|----------------|
| CW | Single spike |
| OOK | Wide main lobe (on/off edges) |
| BPSK/QPSK | Sinc-like with side lobes |
| FSK | Two peaks (mark/space) |
| LoRa | Swept/chirped appearance |

---

## 11. Educational Panels

### 11.1 Modulation Stages

Shows step-by-step modulation:

```
┌─Modulation Stages──────────────────┐
│ 1. Input: "H" (01001000)           │
│ 2. Symbol mapping: [0,1] → 135°    │
│ 3. Pulse shaping: RRC α=0.35       │
│ 4. Upsampling: 8 samples/symbol    │
└────────────────────────────────────┘
```

### 11.2 Demodulation Steps

Shows how symbols are recovered:

```
┌─Demodulation Steps─────────────────┐
│ 1. Matched filter correlation      │
│ 2. Symbol timing recovery          │
│ 3. Phase estimation: 134.8°        │
│ 4. Decision: closest to 135° → 01  │
└────────────────────────────────────┘
```

### 11.3 BER Display

Shows bit error rate:

```
Transmitted: 1011001101...
Received:    1011001101...
BER: 0.00% (0 errors in 80 bits)
```

---

## 12. Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `G` | Generate |
| `D` | Demodulate |
| `C` | Clear |
| `+` / `-` | Zoom |
| `R` | Reset view |
| `S` | Save screenshot |
| `Esc` | Close dialog |

---

## 13. Diagnostic Scenarios

### 13.1 "Why Is My BER So High?"

Checklist:
1. Check SNR - is it too low?
2. Check constellation - clouds overlapping?
3. Try simpler modulation (BPSK)
4. Disable fading temporarily

### 13.2 "Signal Looks Weird"

Checklist:
1. Is the right waveform selected?
2. Is channel enabled when you expect?
3. Check time domain for unexpected patterns
4. Spectrum: is bandwidth correct?

### 13.3 "Demod Doesn't Match Input"

Checklist:
1. Did you click Demodulate?
2. Check for synchronization issues
3. SNR might be too low
4. Try with clean channel first

---

## 14. Quick Reference Card

```
┌──────────────────────────────────────────────────────────────┐
│                 R4W Explorer Quick Reference                 │
├──────────────────────────────────────────────────────────────┤
│ CONSTELLATION:                                               │
│   • Tight points = good SNR                                  │
│   • Fuzzy clouds = noisy                                     │
│   • Rotating = freq offset                                   │
│   • Wrong count = wrong modulation                           │
├──────────────────────────────────────────────────────────────┤
│ TIME DOMAIN:                                                 │
│   • I = real part (horizontal)                               │
│   • Q = imaginary part (vertical)                            │
│   • Constant envelope = PSK/FSK                              │
│   • Varying envelope = ASK/QAM                               │
├──────────────────────────────────────────────────────────────┤
│ SPECTRUM:                                                    │
│   • Width = bandwidth                                        │
│   • Height = power                                           │
│   • Floor = noise level                                      │
│   • More lobes = sharper transitions                         │
├──────────────────────────────────────────────────────────────┤
│ TYPICAL SNR THRESHOLDS:                                      │
│   BPSK:    ~5 dB for low BER                                 │
│   QPSK:    ~8 dB                                             │
│   16-QAM:  ~15 dB                                            │
│   64-QAM:  ~22 dB                                            │
│   LoRa:    Can work below 0 dB!                              │
└──────────────────────────────────────────────────────────────┘
```

---

## 15. Key Takeaways

1. **Constellation = Signal Health**: First place to look
2. **Time Domain = Waveform Shape**: See amplitude patterns
3. **Spectrum = Bandwidth**: Regulatory compliance
4. **SNR Slider = Reality Check**: Test noise immunity
5. **Stages/Steps = Learning**: Understand internals

---

## 16. Next Steps

Continue to [Workshop 10: Real-Time and FPGA Concepts](10-rt-fpga.md) to learn about RT primitives, hardware acceleration, and production deployment.

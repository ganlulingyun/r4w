# Workshop 05: QAM and Constellation Diagrams

**Duration:** 60 minutes
**Difficulty:** Intermediate
**Prerequisites:** Workshops 01-04 completed

## Objectives

By the end of this workshop, you will:
- Understand Quadrature Amplitude Modulation (QAM)
- Read and interpret constellation diagrams like a pro
- Know when to use QAM vs PSK vs ASK
- Build a 16-QAM modulator/demodulator

---

## 1. What is QAM?

### 1.1 The Concept

QAM combines **amplitude AND phase** modulation:
- ASK: Varies amplitude only
- PSK: Varies phase only
- QAM: Varies **both**!

Think of it as placing symbols on a 2D grid in the I/Q plane.

### 1.2 Why QAM?

| Modulation | Symbols | Bits/Symbol | Efficiency |
|------------|---------|-------------|------------|
| BPSK | 2 | 1 | Low |
| QPSK | 4 | 2 | Medium |
| 16-QAM | 16 | 4 | High |
| 64-QAM | 64 | 6 | Very High |
| 256-QAM | 256 | 8 | Extreme |

WiFi uses up to 1024-QAM!

### 1.3 The Trade-off

More symbols = More bits/symbol = **Less noise margin**

```
QPSK: 4 points, widely spaced     256-QAM: 256 points, tightly packed
        01                              ●●●●●●●●●●●●●●●●
         ●                              ●●●●●●●●●●●●●●●●
                                        ●●●●●●●●●●●●●●●●
  11 ●       ● 00                       ●●●●●●●●●●●●●●●●
                                        ●●●●●●●●●●●●●●●●
         ●                              ●●●●●●●●●●●●●●●●
        10                              ●●●●●●●●●●●●●●●●
                                        ←tiny error margin→
```

---

## 2. Understanding Constellation Diagrams

### 2.1 What You're Looking At

A constellation diagram shows **symbol positions** in the I/Q plane:
- Horizontal axis: I (In-phase)
- Vertical axis: Q (Quadrature)
- Each dot: A valid symbol position

### 2.2 Reading Constellations

**Clean Signal:**
```
    Q
    ▲
    │  ●     ●     ●     ●
    │
    │  ●     ●     ●     ●
    │
    │  ●     ●     ●     ●
    │
    │  ●     ●     ●     ●
    └──────────────────────► I
           16-QAM
```

**Noisy Signal:**
```
    Q
    ▲
    │  ○·    ·○    ·○·   ○·
    │   ·  ·    · ·    ·   ·
    │  ·○·   ○·    ○·   ·○
    │   ·     ·  ·  ·  ·
    │  ○·   ·○·   ·○   ○·
    │    ·  ·  ·    · ·
    │  ·○    ○·   ·○·   ○·
    └──────────────────────► I
        Noisy 16-QAM
```

### 2.3 What Problems Look Like

| Problem | Constellation Appearance |
|---------|-------------------------|
| Noise | Cloud around each point |
| Phase offset | Rotation of entire pattern |
| Frequency offset | Spinning/rotating |
| I/Q imbalance | Stretched/skewed |
| Amplitude compression | Points don't reach corners |

---

## 3. Gray Coding

### 3.1 The Problem

Without Gray coding, adjacent symbols might differ by multiple bits:

```
Bad (Binary):     Good (Gray):
  00   01           00   01
   ●   ●             ●   ●
          → 2 bit
          error       → 1 bit
  10   11           10   11  error
   ●   ●             ●   ●
```

### 3.2 Gray Code Mapping

Adjacent symbols differ by **exactly 1 bit**:

```
16-QAM Gray Code:
    0000  0001  0011  0010
    0100  0101  0111  0110
    1100  1101  1111  1110
    1000  1001  1011  1010
```

### 3.3 Code: Gray Conversion

```rust
/// Convert binary to Gray code
fn to_gray(n: u32) -> u32 {
    n ^ (n >> 1)
}

/// Convert Gray code to binary
fn from_gray(g: u32) -> u32 {
    let mut n = g;
    let mut mask = n >> 1;
    while mask != 0 {
        n ^= mask;
        mask >>= 1;
    }
    n
}
```

---

## 4. Building 16-QAM

### 4.1 Symbol Mapping

16-QAM uses a 4×4 grid of symbols:

```rust
/// 16-QAM constellation points (normalized)
const QAM16_POINTS: [(f64, f64); 16] = [
    (-3.0, -3.0), (-3.0, -1.0), (-3.0,  1.0), (-3.0,  3.0),  // 0-3
    (-1.0, -3.0), (-1.0, -1.0), (-1.0,  1.0), (-1.0,  3.0),  // 4-7
    ( 1.0, -3.0), ( 1.0, -1.0), ( 1.0,  1.0), ( 1.0,  3.0),  // 8-11
    ( 3.0, -3.0), ( 3.0, -1.0), ( 3.0,  1.0), ( 3.0,  3.0),  // 12-15
];

/// Normalization factor for unit average power
const QAM16_SCALE: f64 = 1.0 / (10.0_f64).sqrt();  // sqrt(mean power)
```

### 4.2 Modulator

```rust
use r4w_core::types::IQSample;

fn modulate_16qam(symbols: &[u8], samples_per_symbol: usize) -> Vec<IQSample> {
    let mut samples = Vec::new();

    for &sym in symbols {
        let idx = (sym & 0x0F) as usize;  // 4 bits -> 0-15
        let (i, q) = QAM16_POINTS[idx];

        let sample = IQSample::new(i * QAM16_SCALE, q * QAM16_SCALE);

        for _ in 0..samples_per_symbol {
            samples.push(sample);
        }
    }
    samples
}
```

### 4.3 Demodulator

```rust
fn demodulate_16qam(samples: &[IQSample], samples_per_symbol: usize) -> Vec<u8> {
    samples
        .chunks(samples_per_symbol)
        .filter(|c| !c.is_empty())
        .map(|chunk| {
            // Average the chunk
            let sum: IQSample = chunk.iter().copied().sum();
            let avg = sum / chunk.len() as f64;

            // Unscale
            let i = avg.re / QAM16_SCALE;
            let q = avg.im / QAM16_SCALE;

            // Decision: find nearest grid point
            let i_idx = ((i + 4.0) / 2.0).round().clamp(0.0, 3.0) as u8;
            let q_idx = ((q + 4.0) / 2.0).round().clamp(0.0, 3.0) as u8;

            i_idx * 4 + q_idx
        })
        .collect()
}
```

---

## 5. QAM vs PSK: When to Use Each

### 5.1 PSK Advantages

- **Constant envelope**: Can use efficient saturated amplifiers
- **Simpler AGC**: No amplitude information to preserve
- **More robust**: In fading channels

### 5.2 QAM Advantages

- **Higher efficiency**: More bits per symbol
- **Better spectral efficiency**: For high data rates

### 5.3 Decision Guide

| Scenario | Choose |
|----------|--------|
| Satellite (fading, power amp) | PSK |
| WiFi (good channel, speed needed) | QAM |
| Low-power IoT | PSK or simple QAM |
| Cable modem (excellent channel) | 256-QAM or higher |

---

## 6. Complete QAM Waveform

### 6.1 Full Implementation

```rust
use r4w_core::waveform::{Waveform, WaveformInfo};
use r4w_core::types::IQSample;

pub struct Qam16Waveform {
    samples_per_symbol: usize,
}

impl Qam16Waveform {
    const POINTS: [(f64, f64); 16] = [
        (-3.0, -3.0), (-3.0, -1.0), (-3.0,  1.0), (-3.0,  3.0),
        (-1.0, -3.0), (-1.0, -1.0), (-1.0,  1.0), (-1.0,  3.0),
        ( 1.0, -3.0), ( 1.0, -1.0), ( 1.0,  1.0), ( 1.0,  3.0),
        ( 3.0, -3.0), ( 3.0, -1.0), ( 3.0,  1.0), ( 3.0,  3.0),
    ];
    const SCALE: f64 = 0.316227766;  // 1/sqrt(10)

    pub fn new(samples_per_symbol: usize) -> Self {
        Self { samples_per_symbol }
    }

    fn bits_to_symbol(bits: &[bool]) -> u8 {
        let mut sym = 0u8;
        for (i, &bit) in bits.iter().take(4).enumerate() {
            if bit {
                sym |= 1 << (3 - i);
            }
        }
        sym
    }

    fn symbol_to_bits(sym: u8) -> [bool; 4] {
        [
            (sym & 0b1000) != 0,
            (sym & 0b0100) != 0,
            (sym & 0b0010) != 0,
            (sym & 0b0001) != 0,
        ]
    }
}

impl Waveform for Qam16Waveform {
    fn modulate(&mut self, bits: &[bool]) -> Vec<IQSample> {
        let mut samples = Vec::new();

        for chunk in bits.chunks(4) {
            let mut padded = [false; 4];
            padded[..chunk.len()].copy_from_slice(chunk);
            let sym = Self::bits_to_symbol(&padded);

            let (i, q) = Self::POINTS[sym as usize];
            let sample = IQSample::new(i * Self::SCALE, q * Self::SCALE);

            for _ in 0..self.samples_per_symbol {
                samples.push(sample);
            }
        }
        samples
    }

    fn demodulate(&mut self, samples: &[IQSample]) -> Vec<bool> {
        let mut bits = Vec::new();

        for chunk in samples.chunks(self.samples_per_symbol) {
            if chunk.is_empty() { continue; }

            let sum: IQSample = chunk.iter().copied().sum();
            let avg = sum / chunk.len() as f64;

            let i = avg.re / Self::SCALE;
            let q = avg.im / Self::SCALE;

            // Nearest grid point
            let i_idx = ((i + 4.0) / 2.0).round().clamp(0.0, 3.0) as u8;
            let q_idx = ((q + 4.0) / 2.0).round().clamp(0.0, 3.0) as u8;
            let sym = i_idx * 4 + q_idx;

            bits.extend(Self::symbol_to_bits(sym));
        }
        bits
    }

    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "16-QAM".to_string(),
            description: "16-point Quadrature Amplitude Modulation".to_string(),
            samples_per_symbol: self.samples_per_symbol,
            bits_per_symbol: 4,
        }
    }
}
```

---

## 7. Explorer Exercises

### 7.1 Visualize QAM Constellations

1. Launch: `cargo run --bin r4w-explorer`
2. Select 16-QAM
3. Observe the 4×4 grid pattern
4. Try 64-QAM if available

### 7.2 Noise Experiment

1. Set SNR = 30 dB → Tight points
2. SNR = 20 dB → Small clouds
3. SNR = 15 dB → Clouds overlap
4. SNR = 10 dB → Bit errors start!

### 7.3 Compare Efficiency

At same symbol rate:
- BPSK: 1 bit/symbol × 1000 sym/s = 1000 bps
- QPSK: 2 bits/symbol × 1000 sym/s = 2000 bps
- 16-QAM: 4 bits/symbol × 1000 sym/s = 4000 bps
- 64-QAM: 6 bits/symbol × 1000 sym/s = 6000 bps

---

## 8. Advanced Topics

### 8.1 Error Vector Magnitude (EVM)

EVM measures **constellation quality**:

```
EVM = RMS(error vectors) / RMS(reference)

    Q           Error vector
    ▲            ↓
    │   ○──────→●  (received vs ideal)
    │
    └──────────────► I
```

Lower EVM = Better signal quality

### 8.2 Adaptive Modulation

Modern systems switch modulation based on channel:
```
Good channel  → 256-QAM (high rate)
Medium channel → 64-QAM
Poor channel  → 16-QAM
Very poor     → QPSK
Terrible      → BPSK (most robust)
```

This is how WiFi achieves fast downloads when close to router!

---

## 9. Key Takeaways

1. **QAM = ASK + PSK**: Uses both amplitude and phase
2. **More points = More bits**: 16-QAM gives 4 bits/symbol
3. **Gray coding is essential**: Minimizes errors from near-neighbors
4. **Constellation = Visual debugging**: See noise, phase errors, I/Q issues
5. **Trade-off**: Efficiency vs robustness

---

## 10. Next Steps

Continue to [Workshop 06: Spread Spectrum](06-spread-spectrum.md) to learn about DSSS, FHSS, and the LoRa chirp modulation that uses spreading for extreme range.

---

## Quick Reference

```
┌───────────────────────────────────────────────────────────┐
│                 QAM Quick Reference                       │
├───────────────────────────────────────────────────────────┤
│ QAM Types:                                                │
│   4-QAM  (= QPSK)  → 2 bits/symbol                        │
│   16-QAM           → 4 bits/symbol                        │
│   64-QAM           → 6 bits/symbol                        │
│   256-QAM          → 8 bits/symbol                        │
│   1024-QAM         → 10 bits/symbol                       │
├───────────────────────────────────────────────────────────┤
│ Constellation Grid:                                       │
│   16-QAM: 4×4 grid with points at ±1, ±3                  │
│   64-QAM: 8×8 grid with points at ±1, ±3, ±5, ±7          │
├───────────────────────────────────────────────────────────┤
│ Normalization (unit average power):                       │
│   16-QAM: scale = 1/√10                                   │
│   64-QAM: scale = 1/√42                                   │
├───────────────────────────────────────────────────────────┤
│ SNR Requirements (for BER ≈ 10⁻⁵):                        │
│   QPSK:   ~10 dB                                          │
│   16-QAM: ~15 dB                                          │
│   64-QAM: ~20 dB                                          │
│   256-QAM: ~25 dB                                         │
├───────────────────────────────────────────────────────────┤
│ Gray Code: Adjacent symbols differ by 1 bit               │
│   Reduces BER by ~factor of 2                             │
└───────────────────────────────────────────────────────────┘
```

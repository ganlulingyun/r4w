# Workshop 03: Basic Modulation - CW, OOK, ASK

**Duration:** 45 minutes
**Difficulty:** Beginner
**Prerequisites:** Workshops 01-02 completed

## Objectives

By the end of this workshop, you will:
- Understand Continuous Wave (CW) as the simplest signal
- Implement On-Off Keying (OOK) - the "hello world" of digital modulation
- Build Amplitude Shift Keying (ASK) with multiple levels
- See how these waveforms appear in Explorer

---

## 1. Continuous Wave (CW) - The Building Block

### 1.1 What is CW?

CW is just a pure sine wave at a fixed frequency - the simplest RF signal.

```
Amplitude
    ▲
  1 │    ╱╲      ╱╲      ╱╲      ╱╲
    │  ╱    ╲  ╱    ╲  ╱    ╲  ╱    ╲
  0 ├─────────────────────────────────► Time
    │        ╲╱      ╲╱      ╲╱
 -1 │
```

### 1.2 CW in I/Q Domain

A CW signal traces a **circle** in the I/Q plane:

```
    Q
    ▲
    │    ╭───╮
    │  ╱       ╲    ← rotating phasor
    │ │         │
────┼──────────────► I
    │ │         │
    │  ╲       ╱
    │    ╰───╯
```

### 1.3 Code: Generate CW

```rust
use r4w_core::types::IQSample;
use std::f64::consts::PI;

fn generate_cw(
    frequency: f64,    // Hz (relative to center)
    sample_rate: f64,  // samples per second
    duration: f64,     // seconds
) -> Vec<IQSample> {
    let n_samples = (sample_rate * duration) as usize;
    (0..n_samples)
        .map(|n| {
            let t = n as f64 / sample_rate;
            let phase = 2.0 * PI * frequency * t;
            IQSample::new(phase.cos(), phase.sin())
        })
        .collect()
}

// Example: 1 kHz tone for 100 ms at 10 kHz sample rate
let cw = generate_cw(1000.0, 10000.0, 0.1);
```

### 1.4 Explorer Exercise

1. Launch Explorer: `cargo run --bin r4w-explorer`
2. Select the CW waveform
3. Observe:
   - **Constellation**: A single point rotating around origin
   - **Time Domain**: Clean sine waves for I and Q
   - **Spectrum**: A single spike at the carrier

---

## 2. On-Off Keying (OOK) - Digital Hello World

### 2.1 What is OOK?

OOK is the simplest digital modulation:
- **1 bit** = Carrier ON
- **0 bit** = Carrier OFF

```
Data:    1   0   1   1   0   1   0   0

Signal: ╱╲  ...  ╱╲  ╱╲  ...  ╱╲  ...  ...
        ON  OFF ON  ON  OFF ON  OFF OFF
```

### 2.2 Real-World OOK

You use OOK every day!
- **Garage door remotes**: 315/433 MHz OOK
- **Car key fobs**: Simple OOK or ASK
- **IR remotes**: LED on/off for 1/0

### 2.3 Code: OOK Modulator

```rust
use r4w_core::types::IQSample;

fn modulate_ook(
    data: &[bool],
    samples_per_bit: usize,
) -> Vec<IQSample> {
    let mut samples = Vec::with_capacity(data.len() * samples_per_bit);

    for &bit in data {
        let amplitude = if bit { 1.0 } else { 0.0 };
        for _ in 0..samples_per_bit {
            // OOK at baseband: just amplitude, no phase
            samples.push(IQSample::new(amplitude, 0.0));
        }
    }
    samples
}

// Example: modulate "Hi" (ASCII bits)
let message = [true, false, true, true, false, true, false, false];  // simplified
let ook_samples = modulate_ook(&message, 100);
```

### 2.4 OOK Constellation

OOK has only **two points** on the constellation:

```
    Q
    ▲
    │
    │
────┼─────*─────*──► I
    │     ▲     ▲
    │    OFF   ON
    │   (0,0) (1,0)
```

### 2.5 Explorer Exercise

1. Select OOK waveform in Explorer
2. Enter a short message
3. Observe:
   - **Constellation**: Two points at 0 and 1 on I-axis
   - **Time Domain**: Bursts of carrier, then silence
   - **Spectrum**: Wide bandwidth due to sharp on/off

---

## 3. Amplitude Shift Keying (ASK) - More Levels

### 3.1 What is ASK?

ASK extends OOK with **multiple amplitude levels**:

```
2-ASK (same as OOK):     0 ────── 1
4-ASK:                   0 ── 1 ── 2 ── 3
8-ASK:                   0 1 2 3 4 5 6 7
```

### 3.2 More Bits Per Symbol

| ASK Type | Levels | Bits per Symbol |
|----------|--------|-----------------|
| 2-ASK    | 2      | 1               |
| 4-ASK    | 4      | 2               |
| 8-ASK    | 8      | 3               |

### 3.3 Code: 4-ASK Modulator

```rust
fn modulate_4ask(
    symbols: &[u8],  // Values 0-3
    samples_per_symbol: usize,
) -> Vec<IQSample> {
    let amplitudes = [0.0, 0.33, 0.67, 1.0];  // 4 levels

    let mut samples = Vec::new();
    for &sym in symbols {
        let amp = amplitudes[sym as usize];
        for _ in 0..samples_per_symbol {
            samples.push(IQSample::new(amp, 0.0));
        }
    }
    samples
}

// Symbols: 0, 1, 2, 3, 2, 1, 0
let symbols = vec![0, 1, 2, 3, 2, 1, 0];
let ask_samples = modulate_4ask(&symbols, 50);
```

### 3.4 ASK Constellation

4-ASK constellation:
```
    Q
    ▲
    │
────┼──*──*──*──*──► I
    │  ▲  ▲  ▲  ▲
    │ 00 01 10 11
```

### 3.5 The Problem with ASK

ASK is **sensitive to amplitude variations**:
- Fading makes the signal weaker
- AGC issues can shift all levels
- Noise affects higher levels more

This is why we often prefer PSK!

---

## 4. Hands-On: Build OOK in R4W

### 4.1 The Waveform Trait

R4W waveforms implement a common interface:

```rust
pub trait Waveform {
    fn modulate(&mut self, bits: &[bool]) -> Vec<IQSample>;
    fn demodulate(&mut self, samples: &[IQSample]) -> Vec<bool>;
    fn info(&self) -> WaveformInfo;
}
```

### 4.2 OOK Implementation

Here's a simplified OOK waveform:

```rust
use r4w_core::waveform::{Waveform, WaveformInfo, CommonParams};
use r4w_core::types::IQSample;

pub struct OokWaveform {
    samples_per_bit: usize,
}

impl OokWaveform {
    pub fn new(samples_per_bit: usize) -> Self {
        Self { samples_per_bit }
    }
}

impl Waveform for OokWaveform {
    fn modulate(&mut self, bits: &[bool]) -> Vec<IQSample> {
        let mut samples = Vec::new();
        for &bit in bits {
            let amp = if bit { 1.0 } else { 0.0 };
            for _ in 0..self.samples_per_bit {
                samples.push(IQSample::new(amp, 0.0));
            }
        }
        samples
    }

    fn demodulate(&mut self, samples: &[IQSample]) -> Vec<bool> {
        samples
            .chunks(self.samples_per_bit)
            .map(|chunk| {
                // Average amplitude decision
                let avg: f64 = chunk.iter().map(|s| s.norm()).sum::<f64>()
                    / chunk.len() as f64;
                avg > 0.5
            })
            .collect()
    }

    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "OOK".to_string(),
            description: "On-Off Keying".to_string(),
            samples_per_symbol: self.samples_per_bit,
            bits_per_symbol: 1,
        }
    }
}
```

### 4.3 Test Your OOK

```rust
fn test_ook() {
    let mut ook = OokWaveform::new(10);

    let message = vec![true, false, true, true, false];
    let modulated = ook.modulate(&message);
    let demodulated = ook.demodulate(&modulated);

    assert_eq!(message, demodulated);
    println!("OOK round-trip: OK!");
}
```

---

## 5. Demodulation: How to Detect OOK

### 5.1 Envelope Detection

The classic OOK demodulator uses **envelope detection**:

```
1. Compute magnitude: |sample| = √(I² + Q²)
2. Average over symbol period
3. Compare to threshold
4. Decide 1 or 0
```

### 5.2 Decision Threshold

```
Amplitude
    ▲
  1 ├────────────────
    │    ████████████  ← bit = 1
    │    █  █  █  █
0.5 ├─────────────────  ← threshold
    │
    │▄▄▄▄           ▄▄  ← bit = 0
  0 └────────────────────► Time
```

### 5.3 Adaptive Threshold

Real systems use AGC (Automatic Gain Control):

```rust
fn adaptive_threshold(samples: &[IQSample]) -> f64 {
    let magnitudes: Vec<f64> = samples.iter().map(|s| s.norm()).collect();
    let max = magnitudes.iter().cloned().fold(0.0, f64::max);
    let min = magnitudes.iter().cloned().fold(f64::MAX, f64::min);

    // Threshold at midpoint
    (max + min) / 2.0
}
```

---

## 6. Explorer Deep Dive

### 6.1 Controls for This Workshop

| Control | What It Does |
|---------|--------------|
| Waveform | Select CW, OOK, or ASK |
| Symbol Rate | Bits per second |
| SNR | Signal-to-Noise Ratio |
| Message | Text to transmit |

### 6.2 What to Observe

**For CW:**
- Time: Perfect sine waves
- Spectrum: Single narrow peak
- Constellation: One point

**For OOK:**
- Time: Bursts of carrier
- Spectrum: Wide main lobe
- Constellation: Two points (0 and 1)

**For ASK:**
- Time: Varying amplitude bursts
- Spectrum: Similar to OOK
- Constellation: Multiple points on I-axis

---

## 7. Challenges

### Challenge 1: OOK Remote Decoder
Design a decoder that can detect OOK from a garage remote (timing recovery needed!).

### Challenge 2: 8-ASK Implementation
Extend the 4-ASK code to 8 levels. What happens to noise immunity?

### Challenge 3: ASK with Pulse Shaping
Add a raised cosine filter to reduce spectral splatter.

---

## 8. Key Takeaways

1. **CW is the foundation** - Everything builds on the carrier
2. **OOK is the simplest digital mod** - Just on/off!
3. **ASK adds levels** - More bits, but noise sensitive
4. **Envelope detection** - Simple demodulation
5. **Amplitude mods are fading-sensitive** - Phase mods are better

---

## 9. Next Steps

Continue to [Workshop 04: FSK and PSK](04-fsk-psk.md) to learn about frequency and phase-based modulation, which are more robust than amplitude-based schemes.

---

## Quick Reference

```
┌──────────────────────────────────────────────────────┐
│          Basic Modulation Reference                  │
├──────────────────────────────────────────────────────┤
│ CW:  Constant amplitude, constant phase              │
│      s(t) = A × cos(2πft)                            │
│                                                      │
│ OOK: Carrier on (1) or off (0)                       │
│      s(t) = d(t) × cos(2πft)   where d ∈ {0, 1}      │
│                                                      │
│ ASK: Multiple amplitude levels                       │
│      s(t) = A(d) × cos(2πft)   where d ∈ {0..N-1}    │
├──────────────────────────────────────────────────────┤
│ Constellation Points:                                │
│   OOK:  2 points  [0, 1] on I-axis                   │
│   4ASK: 4 points  [0, 0.33, 0.67, 1] on I-axis       │
├──────────────────────────────────────────────────────┤
│ Demodulation:                                        │
│   1. Compute envelope: |s| = √(I² + Q²)              │
│   2. Average over symbol                             │
│   3. Compare to threshold                            │
│   4. Decide bit/symbol                               │
└──────────────────────────────────────────────────────┘
```

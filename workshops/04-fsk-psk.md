# Workshop 04: FSK and PSK Modulation

**Duration:** 60 minutes
**Difficulty:** Intermediate
**Prerequisites:** Workshops 01-03 completed

## Objectives

By the end of this workshop, you will:
- Understand Frequency Shift Keying (FSK) and its variants
- Master Phase Shift Keying (PSK) including BPSK, QPSK, 8-PSK
- Compare PSK with ASK for noise resilience
- Implement basic FSK and BPSK waveforms

---

## 1. Frequency Shift Keying (FSK)

### 1.1 The Concept

FSK encodes data by **shifting the carrier frequency**:

```
Data:     1       0       1       1       0

         f_high  f_low   f_high  f_high  f_low
         ╱╲╱╲   /╲/╲    ╱╲╱╲    ╱╲╱╲   /╲/╲
        ╱    ╲ /    \  ╱    ╲  ╱    ╲ /    \
```

- **Bit 1**: Higher frequency (more oscillations)
- **Bit 0**: Lower frequency (fewer oscillations)

### 1.2 FSK Parameters

| Parameter | Description |
|-----------|-------------|
| f₀ (Mark) | Frequency for bit 1 |
| f₁ (Space) | Frequency for bit 0 |
| Δf | Frequency deviation (f₀ - f₁)/2 |
| h | Modulation index = Δf / bit_rate |

### 1.3 Modulation Index

The **modulation index h** affects spectrum and detection:

- **h < 0.5**: Spectrally efficient, harder to detect
- **h = 0.5**: MSK (Minimum Shift Keying) - sweet spot
- **h > 1**: Wide bandwidth, easy to detect

### 1.4 Code: FSK Modulator

```rust
use r4w_core::types::IQSample;
use std::f64::consts::PI;

fn modulate_fsk(
    bits: &[bool],
    samples_per_bit: usize,
    freq_deviation: f64,  // Hz
    sample_rate: f64,
) -> Vec<IQSample> {
    let mut samples = Vec::new();
    let mut phase = 0.0;

    for &bit in bits {
        let freq = if bit { freq_deviation } else { -freq_deviation };

        for _ in 0..samples_per_bit {
            samples.push(IQSample::new(phase.cos(), phase.sin()));
            phase += 2.0 * PI * freq / sample_rate;
        }
    }
    samples
}

// Example: FSK with ±500 Hz deviation
let bits = vec![true, false, true, true, false];
let fsk = modulate_fsk(&bits, 100, 500.0, 10000.0);
```

### 1.5 FSK in the I/Q Plane

FSK traces **different circles** for each frequency:

```
    Q                 Q
    ▲                 ▲
    │   ╱╲           │ ╱──╲
    │  ╱  ╲          │╱    ╲
────┼──────►     ────┼───────►
    │  ╲  ╱          │╲    ╱
    │   ╲╱           │ ╲──╱

   Fast rotation      Slow rotation
   (bit = 1)         (bit = 0)
```

---

## 2. FSK Variants

### 2.1 GFSK (Gaussian FSK)

Used in **Bluetooth** and **LoRa preamble**:
- Gaussian filter smooths frequency transitions
- Reduces spectral splatter
- Slight ISI (inter-symbol interference)

### 2.2 MSK (Minimum Shift Keying)

Special case where h = 0.5:
- Continuous phase
- Smallest bandwidth for given bit rate
- Used in GSM (GMSK variant)

### 2.3 AFSK (Audio FSK)

FSK over audio frequencies:
- Modems: Bell 103 (300 baud), Bell 202 (1200 baud)
- Amateur radio: AX.25 packet (1200/2200 Hz)

---

## 3. Phase Shift Keying (PSK)

### 3.1 The Concept

PSK encodes data by **changing the carrier phase**:

```
BPSK:  Phase 0° = bit 0,  Phase 180° = bit 1

    Q                    Q
    ▲                    ▲
    │                    │
    │                    │
────*────────► I     ────────*──► I
    │  (0°)              │    (180°)
    │ bit = 0            │   bit = 1
```

### 3.2 BPSK (Binary PSK)

The simplest PSK - just two phases:

```rust
fn modulate_bpsk(bits: &[bool], samples_per_bit: usize) -> Vec<IQSample> {
    let mut samples = Vec::new();
    for &bit in bits {
        let phase = if bit { PI } else { 0.0 };
        let symbol = IQSample::new(phase.cos(), phase.sin());

        for _ in 0..samples_per_bit {
            samples.push(symbol);
        }
    }
    samples
}
```

BPSK Constellation:
```
    Q
    ▲
    │
    │
    *────────┼────────*──► I
   -1        │        +1
  bit=1      │      bit=0
```

### 3.3 QPSK (Quadrature PSK)

Four phases = 2 bits per symbol:

```
    Q
    ▲
    │    01
    │     *
    │        ╲
11 *──────────┼──────────* 00
    │        ╱
    │     *
    │    10
```

Phase mapping:
- 00 → 45°   (I=0.707, Q=0.707)
- 01 → 135°  (I=-0.707, Q=0.707)
- 10 → 315°  (I=0.707, Q=-0.707)
- 11 → 225°  (I=-0.707, Q=-0.707)

### 3.4 Code: QPSK Modulator

```rust
use std::f64::consts::PI;

fn modulate_qpsk(dibits: &[(bool, bool)], samples_per_symbol: usize) -> Vec<IQSample> {
    let mut samples = Vec::new();

    for &(b0, b1) in dibits {
        let phase = match (b0, b1) {
            (false, false) => PI / 4.0,       // 00 → 45°
            (false, true)  => 3.0 * PI / 4.0, // 01 → 135°
            (true, false)  => -PI / 4.0,      // 10 → 315°
            (true, true)   => -3.0 * PI / 4.0,// 11 → 225°
        };

        let symbol = IQSample::new(phase.cos(), phase.sin());
        for _ in 0..samples_per_symbol {
            samples.push(symbol);
        }
    }
    samples
}
```

### 3.5 8-PSK

Eight phases = 3 bits per symbol:

```
    Q
    ▲
  011│ 010
     *   *   001
       ╲ │ ╱
 110 *───┼───* 000
       ╱ │ ╲
     *   *   111
  101│ 100
```

---

## 4. PSK vs ASK: Noise Resilience

### 4.1 The Key Difference

| Aspect | ASK | PSK |
|--------|-----|-----|
| Information | Amplitude | Phase |
| Fading impact | Severe | Minimal |
| Noise margin | Variable | Constant |
| Complexity | Simple | Moderate |

### 4.2 Why PSK Wins

PSK has **constant envelope** (amplitude):
- Power amplifiers can run saturated (efficient!)
- Fading affects all symbols equally
- Decision is based on angle, not distance

### 4.3 Visual Comparison

```
ASK (noise affects amplitude):        PSK (noise affects angle):
    Q                                     Q
    ▲                                     ▲
    │                                     │    ╱╲
    │     ←noise→                        │   ╱  ╲ ← noise region
    *─────*─────*─────*──► I              │  *────*──► I
   0.0   0.33  0.67  1.0                 │   ╲  ╱
   ↑          ↑                          │    ╲╱
  close to    farther
   zero      from zero                   Equal distance from origin!
```

---

## 5. PSK Demodulation

### 5.1 Coherent Detection

PSK requires knowing the **exact carrier phase**:

```rust
fn demodulate_bpsk(samples: &[IQSample], samples_per_symbol: usize) -> Vec<bool> {
    samples
        .chunks(samples_per_symbol)
        .map(|chunk| {
            // Average over symbol
            let sum: IQSample = chunk.iter().sum();
            let avg = sum / chunk.len() as f64;

            // Decision based on real part (I)
            avg.re < 0.0  // True if phase ≈ 180°
        })
        .collect()
}
```

### 5.2 Differential PSK (DPSK)

Avoids phase ambiguity by encoding **phase changes**:

```
DBPSK: bit = 1 if phase changed, 0 if same

Previous: 0°   Current: 180°  → bit = 1 (changed)
Previous: 180° Current: 180°  → bit = 0 (same)
```

### 5.3 QPSK Demodulation

```rust
fn demodulate_qpsk(samples: &[IQSample], samples_per_symbol: usize) -> Vec<(bool, bool)> {
    samples
        .chunks(samples_per_symbol)
        .map(|chunk| {
            let sum: IQSample = chunk.iter().sum();
            let avg = sum / chunk.len() as f64;

            // Quadrant detection
            let b0 = avg.re < 0.0;  // Left half
            let b1 = avg.im > 0.0;  // Upper half

            (b0, b1)
        })
        .collect()
}
```

---

## 6. Explorer Exercises

### 6.1 Compare BPSK and QPSK

1. Launch Explorer: `cargo run --bin r4w-explorer`
2. Select BPSK, enter message "HELLO"
3. Note: 2 constellation points
4. Switch to QPSK
5. Note: 4 constellation points, same message uses fewer symbols!

### 6.2 Noise Resilience Test

1. Set SNR to 20 dB - clean constellation
2. Lower to 10 dB - points start spreading
3. Lower to 5 dB - errors begin
4. Compare: Which modulation fails first, BPSK or QPSK?

### 6.3 Bandwidth Comparison

1. Watch the spectrum display
2. Compare BPSK and QPSK at same bit rate
3. QPSK uses half the bandwidth!

---

## 7. Hands-On: Complete PSK Waveform

### 7.1 BPSK Waveform Implementation

```rust
use r4w_core::waveform::{Waveform, WaveformInfo};
use r4w_core::types::IQSample;
use std::f64::consts::PI;

pub struct BpskWaveform {
    samples_per_symbol: usize,
}

impl BpskWaveform {
    pub fn new(samples_per_symbol: usize) -> Self {
        Self { samples_per_symbol }
    }
}

impl Waveform for BpskWaveform {
    fn modulate(&mut self, bits: &[bool]) -> Vec<IQSample> {
        let mut samples = Vec::with_capacity(bits.len() * self.samples_per_symbol);

        for &bit in bits {
            // BPSK: 0 → +1, 1 → -1
            let symbol = if bit {
                IQSample::new(-1.0, 0.0)
            } else {
                IQSample::new(1.0, 0.0)
            };

            for _ in 0..self.samples_per_symbol {
                samples.push(symbol);
            }
        }
        samples
    }

    fn demodulate(&mut self, samples: &[IQSample]) -> Vec<bool> {
        samples
            .chunks(self.samples_per_symbol)
            .filter(|c| !c.is_empty())
            .map(|chunk| {
                let avg_i: f64 = chunk.iter().map(|s| s.re).sum::<f64>()
                    / chunk.len() as f64;
                avg_i < 0.0
            })
            .collect()
    }

    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "BPSK".to_string(),
            description: "Binary Phase Shift Keying".to_string(),
            samples_per_symbol: self.samples_per_symbol,
            bits_per_symbol: 1,
        }
    }
}
```

### 7.2 Test Round-Trip

```rust
fn test_bpsk_roundtrip() {
    let mut bpsk = BpskWaveform::new(8);

    let original = vec![true, false, true, true, false, false, true];
    let modulated = bpsk.modulate(&original);
    let demodulated = bpsk.demodulate(&modulated);

    assert_eq!(original, demodulated);
    println!("BPSK round-trip: SUCCESS!");
}
```

---

## 8. FSK Detection Methods

### 8.1 Discriminator Detector

Classic FM receiver approach:
```rust
fn fsk_discriminator(samples: &[IQSample]) -> Vec<f64> {
    let mut freq_estimates = Vec::new();

    for i in 1..samples.len() {
        // Phase difference = instantaneous frequency
        let phase_diff = (samples[i] * samples[i-1].conj()).arg();
        freq_estimates.push(phase_diff);
    }

    freq_estimates
}
```

### 8.2 Matched Filter Detection

Correlate with expected waveforms:
```rust
fn fsk_matched_filter(
    samples: &[IQSample],
    template_f0: &[IQSample],
    template_f1: &[IQSample],
) -> Vec<bool> {
    samples
        .chunks(template_f0.len())
        .map(|chunk| {
            let corr_f0: f64 = chunk.iter()
                .zip(template_f0.iter())
                .map(|(a, b)| (a * b.conj()).re)
                .sum();
            let corr_f1: f64 = chunk.iter()
                .zip(template_f1.iter())
                .map(|(a, b)| (a * b.conj()).re)
                .sum();

            corr_f1 > corr_f0
        })
        .collect()
}
```

---

## 9. Key Takeaways

1. **FSK = Frequency changes**: Robust, but uses more bandwidth
2. **PSK = Phase changes**: Efficient, constant envelope
3. **BPSK = 1 bit/symbol**: Most robust PSK
4. **QPSK = 2 bits/symbol**: Double the efficiency of BPSK
5. **8-PSK = 3 bits/symbol**: Even more efficient, less robust
6. **PSK beats ASK** for noise immunity (equal energy symbols)

---

## 10. Next Steps

Continue to [Workshop 05: QAM and Constellation Diagrams](05-qam.md) to learn how combining amplitude AND phase gives even higher efficiency.

---

## Quick Reference

```
┌─────────────────────────────────────────────────────────┐
│               FSK/PSK Quick Reference                   │
├─────────────────────────────────────────────────────────┤
│ FSK: Encode bits as frequency shifts                    │
│   Bit 0 → f₀ (mark)                                     │
│   Bit 1 → f₁ (space)                                    │
│   h = Δf / bit_rate (modulation index)                  │
├─────────────────────────────────────────────────────────┤
│ PSK: Encode bits as phase shifts                        │
│   BPSK:  2 phases → 1 bit/symbol  (0°, 180°)            │
│   QPSK:  4 phases → 2 bits/symbol (45°, 135°, ...)      │
│   8-PSK: 8 phases → 3 bits/symbol                       │
├─────────────────────────────────────────────────────────┤
│ Key Formulas:                                           │
│   Bit rate = Symbol rate × bits/symbol                  │
│   BW ≈ Symbol rate × (1 + α)  [α = roll-off]            │
│                                                         │
│ BPSK: symbol = cos(π × bit)  → ±1                       │
│ QPSK: symbol = e^(jπ/4 × (2k+1))  k ∈ {0,1,2,3}         │
├─────────────────────────────────────────────────────────┤
│ Noise Immunity (best to worst):                         │
│   BPSK > QPSK > 8-PSK > 16-PSK                          │
│                                                         │
│ Spectral Efficiency (best to worst):                    │
│   16-PSK > 8-PSK > QPSK > BPSK                          │
└─────────────────────────────────────────────────────────┘
```

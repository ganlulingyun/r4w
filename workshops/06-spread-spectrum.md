# Workshop 06: Spread Spectrum Techniques

**Duration:** 75 minutes
**Difficulty:** Intermediate to Advanced
**Prerequisites:** Workshops 01-05 completed

## Objectives

By the end of this workshop, you will:
- Understand why spreading signals is useful
- Implement Direct Sequence Spread Spectrum (DSSS)
- Explore Frequency Hopping (FHSS)
- Deep dive into LoRa's Chirp Spread Spectrum (CSS)
- See the processing gain in action

---

## 1. Why Spread Spectrum?

### 1.1 The Problems with Narrowband

Traditional narrowband signals are vulnerable:
- **Interference**: One jammer can block your signal
- **Fading**: Narrow band = one frequency affected = total loss
- **Detection**: Easy to find and intercept
- **Multipath**: Severe at specific frequencies

### 1.2 The Solution: Spread It Out!

Spread spectrum trades bandwidth for robustness:

```
Narrowband:              Spread Spectrum:
    ▲ Power                  ▲ Power
    │                        │
    │  ▄█▄                   │ ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
    │ ▄███▄                  │▄████████████████▄
    └────────► f             └───────────────────► f
    (easy to jam)            (below noise floor!)
```

### 1.3 Processing Gain

The magic of spread spectrum:

```
Processing Gain (dB) = 10 × log₁₀(Spread BW / Data BW)

Example:
  Spread BW = 125 kHz
  Data BW = 1 kHz (data rate / 125)
  PG = 10 × log₁₀(125) = 21 dB!
```

This means you can receive signals **21 dB below the noise floor**!

---

## 2. Direct Sequence Spread Spectrum (DSSS)

### 2.1 The Concept

DSSS multiplies data by a high-rate **spreading code**:

```
Data bit:        1 1 1 1 1 1 1 1
                 × × × × × × × ×
Spreading code:  1 0 1 1 0 0 1 0  (chip sequence)
                 = = = = = = = =
Transmitted:     1 0 1 1 0 0 1 0  (looks like noise!)
```

### 2.2 Key Terms

| Term | Definition |
|------|------------|
| **Chip** | One bit of the spreading code |
| **Chip rate** | Speed of spreading code |
| **Spreading factor** | Chips per data bit |
| **PN sequence** | Pseudo-random noise code |

### 2.3 Code: DSSS Modulator

```rust
use r4w_core::types::IQSample;

fn generate_pn_sequence(length: usize, seed: u16) -> Vec<bool> {
    // Simple LFSR-based PN generator
    let mut lfsr = seed;
    (0..length)
        .map(|_| {
            let bit = (lfsr & 1) != 0;
            let new_bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1;
            lfsr = (lfsr >> 1) | ((new_bit as u16) << 15);
            bit
        })
        .collect()
}

fn modulate_dsss(
    data: &[bool],
    spreading_factor: usize,
    pn_seed: u16,
) -> Vec<IQSample> {
    let pn = generate_pn_sequence(spreading_factor, pn_seed);

    let mut samples = Vec::new();
    for &bit in data {
        for &chip in &pn {
            // XOR data with PN, then BPSK modulate
            let spread_bit = bit ^ chip;
            let sample = if spread_bit {
                IQSample::new(-1.0, 0.0)
            } else {
                IQSample::new(1.0, 0.0)
            };
            samples.push(sample);
        }
    }
    samples
}
```

### 2.4 DSSS Demodulation

The receiver **correlates** with the same PN code:

```rust
fn demodulate_dsss(
    samples: &[IQSample],
    spreading_factor: usize,
    pn_seed: u16,
) -> Vec<bool> {
    let pn = generate_pn_sequence(spreading_factor, pn_seed);

    samples
        .chunks(spreading_factor)
        .map(|chip_samples| {
            // Correlate with PN sequence
            let correlation: f64 = chip_samples
                .iter()
                .zip(pn.iter())
                .map(|(sample, &pn_chip)| {
                    let pn_val = if pn_chip { -1.0 } else { 1.0 };
                    sample.re * pn_val
                })
                .sum();

            correlation < 0.0  // Decision
        })
        .collect()
}
```

### 2.5 The Correlation Magic

When codes match → **strong peak**
When codes don't match → **near zero**

```
Matched (correct receiver):      Unmatched (wrong code):
   ▲                               ▲
   │    ▄                          │  ▄ ▄   ▄  ▄
   │   ███                         │ ▄█ █▄ █▄▄ █
   │  █████                        │██ ██ ██ ███
   └─────────►                     └───────────────►
      PEAK!                           just noise
```

---

## 3. Frequency Hopping Spread Spectrum (FHSS)

### 3.1 The Concept

FHSS jumps between frequencies according to a pattern:

```
Frequency
    ▲
 f₃ │     ▬▬▬
 f₂ │           ▬▬▬
 f₁ │  ▬▬▬            ▬▬▬
 f₀ │                       ▬▬▬
    └─────────────────────────────► Time
        hop  hop  hop  hop
```

### 3.2 Hop Sequence Generation

```rust
struct FhssSequence {
    num_channels: usize,
    seed: u32,
    current_index: usize,
    sequence: Vec<usize>,
}

impl FhssSequence {
    fn new(num_channels: usize, seed: u32) -> Self {
        // Generate pseudo-random hop sequence
        let mut sequence = Vec::with_capacity(num_channels);
        let mut lfsr = seed;

        for _ in 0..num_channels {
            let channel = (lfsr as usize) % num_channels;
            sequence.push(channel);
            lfsr = lfsr.wrapping_mul(1103515245).wrapping_add(12345);
        }

        Self { num_channels, seed, current_index: 0, sequence }
    }

    fn next_channel(&mut self) -> usize {
        let ch = self.sequence[self.current_index];
        self.current_index = (self.current_index + 1) % self.sequence.len();
        ch
    }
}
```

### 3.3 FHSS in Practice

- **Bluetooth**: 79 channels, 1600 hops/sec
- **WiFi (legacy)**: 75+ channels
- **Military radios**: Thousands of hops/sec

---

## 4. LoRa: Chirp Spread Spectrum (CSS)

### 4.1 What Makes LoRa Special

LoRa uses **chirps** - signals that sweep frequency:

```
Frequency
    ▲
    │        ╱│
    │      ╱  │
    │    ╱    │
    │  ╱      │
    │╱        │
    └─────────┴───► Time
      Up-chirp

Symbol 0: Start at f_min, end at f_max
Symbol 5: Start at f_min + 5Δf, wrap around
```

### 4.2 Why Chirps Work

1. **Resistant to Doppler**: Frequency shift just delays detection
2. **Resistant to multipath**: Different delays = different frequencies
3. **Below noise floor**: Processing gain from spreading

### 4.3 LoRa Parameters

| Parameter | Values | Effect |
|-----------|--------|--------|
| SF (Spreading Factor) | 7-12 | Higher = longer range, slower |
| BW (Bandwidth) | 125/250/500 kHz | Higher = faster, less range |
| CR (Coding Rate) | 4/5 to 4/8 | Higher = more robust |

### 4.4 Code: Generate LoRa Chirp

```rust
use r4w_core::types::IQSample;
use std::f64::consts::PI;

fn generate_lora_upchirp(
    sf: u32,        // Spreading factor (7-12)
    bw: f64,        // Bandwidth in Hz
    sample_rate: f64,
) -> Vec<IQSample> {
    let n_symbols = 1 << sf;  // 2^SF
    let symbol_duration = n_symbols as f64 / bw;
    let n_samples = (sample_rate * symbol_duration) as usize;

    (0..n_samples)
        .map(|n| {
            let t = n as f64 / sample_rate;
            // Chirp: frequency increases linearly
            let freq = -bw / 2.0 + (bw / symbol_duration) * t;
            let phase = 2.0 * PI * freq * t;
            IQSample::new(phase.cos(), phase.sin())
        })
        .collect()
}

fn generate_lora_downchirp(
    sf: u32,
    bw: f64,
    sample_rate: f64,
) -> Vec<IQSample> {
    let upchirp = generate_lora_upchirp(sf, bw, sample_rate);
    // Downchirp = conjugate of upchirp
    upchirp.iter().map(|s| s.conj()).collect()
}
```

### 4.5 LoRa Modulation

Each symbol shifts the chirp starting point:

```rust
fn modulate_lora_symbol(
    symbol: u32,    // 0 to 2^SF - 1
    sf: u32,
    bw: f64,
    sample_rate: f64,
) -> Vec<IQSample> {
    let n_symbols = 1 << sf;
    let base_chirp = generate_lora_upchirp(sf, bw, sample_rate);
    let samples_per_symbol = base_chirp.len();

    // Circular shift by symbol value
    let shift = (symbol as usize * samples_per_symbol) / n_symbols;

    let mut result = Vec::with_capacity(samples_per_symbol);
    for i in 0..samples_per_symbol {
        result.push(base_chirp[(i + shift) % samples_per_symbol]);
    }
    result
}
```

### 4.6 LoRa Demodulation

Multiply by downchirp, take FFT, find peak:

```rust
use rustfft::{FftPlanner, num_complex::Complex};

fn demodulate_lora(
    samples: &[IQSample],
    sf: u32,
    bw: f64,
    sample_rate: f64,
) -> Vec<u32> {
    let downchirp = generate_lora_downchirp(sf, bw, sample_rate);
    let symbol_len = downchirp.len();
    let n_symbols = 1 << sf;

    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(symbol_len);

    samples
        .chunks(symbol_len)
        .map(|chunk| {
            // Multiply by downchirp (dechirp)
            let mut dechirped: Vec<Complex<f64>> = chunk
                .iter()
                .zip(downchirp.iter())
                .map(|(s, d)| {
                    let product = s * d;
                    Complex::new(product.re, product.im)
                })
                .collect();

            // Pad if necessary
            dechirped.resize(symbol_len, Complex::new(0.0, 0.0));

            // FFT
            fft.process(&mut dechirped);

            // Find peak bin
            let (peak_bin, _) = dechirped.iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.norm().partial_cmp(&b.norm()).unwrap())
                .unwrap();

            // Convert bin to symbol
            ((peak_bin * n_symbols) / symbol_len) as u32
        })
        .collect()
}
```

---

## 5. Processing Gain Comparison

| Technique | Typical Processing Gain |
|-----------|------------------------|
| Narrowband | 0 dB |
| DSSS (SF=31) | 15 dB |
| DSSS (SF=127) | 21 dB |
| LoRa SF7 | 7 dB |
| LoRa SF12 | 12 dB |
| FHSS (79 ch) | 19 dB |

---

## 6. Explorer Exercises

### 6.1 Visualize LoRa Chirps

1. Launch: `cargo run --bin r4w-explorer`
2. Select LoRa waveform
3. Look at the spectrogram - see the sweeping chirps!
4. Change SF and observe the duration change

### 6.2 Compare Spreading Factors

| SF | Symbols | Time on Air | Range | Data Rate |
|----|---------|-------------|-------|-----------|
| 7 | 128 | Short | Less | Fast |
| 10 | 1024 | Medium | Medium | Medium |
| 12 | 4096 | Long | Max | Slow |

### 6.3 Noise Floor Experiment

1. Set high SNR → Clean FFT peak
2. Lower SNR to 0 dB → Peak still visible!
3. Lower to -10 dB → LoRa still works (processing gain!)
4. Try with BPSK at -10 dB → No detection

---

## 7. CDMA: Multiple Access with DSSS

### 7.1 The Magic of Orthogonal Codes

Multiple users can share the same frequency if their codes are **orthogonal**:

```rust
fn generate_walsh_codes(order: usize) -> Vec<Vec<i8>> {
    // Hadamard matrix based codes
    let mut codes = vec![vec![1i8]];

    while codes.len() < order {
        let n = codes.len();
        let mut new_codes = vec![vec![0i8; n * 2]; n * 2];

        for i in 0..n {
            for j in 0..n {
                // [H  H ]
                // [H -H]
                new_codes[i][j] = codes[i][j];
                new_codes[i][j + n] = codes[i][j];
                new_codes[i + n][j] = codes[i][j];
                new_codes[i + n][j + n] = -codes[i][j];
            }
        }
        codes = new_codes;
    }
    codes
}
```

### 7.2 CDMA in Action

```
User 1 (code A): [1  1 -1 -1] × data = [1  1 -1 -1]
User 2 (code B): [1 -1  1 -1] × data = [1 -1  1 -1]
                                    +
Received:                            [2  0  0 -2]

Correlate with A:  [1  1 -1 -1]·[2 0 0 -2] = 4 → User 1 bit = 1
Correlate with B:  [1 -1  1 -1]·[2 0 0 -2] = 0 → User 2 bit = 0 (no data)
```

---

## 8. Key Takeaways

1. **Spread spectrum = Robustness**: Trades bandwidth for noise immunity
2. **DSSS = Code multiplication**: Same frequency, different codes
3. **FHSS = Frequency hopping**: Different frequencies, same time
4. **LoRa CSS = Chirps**: Sweeping frequency for extreme range
5. **Processing gain**: Recover signals below noise floor
6. **CDMA = Sharing**: Multiple users, same band, orthogonal codes

---

## 9. Next Steps

Continue to [Workshop 07: Channel Effects](07-channel-effects.md) to learn about noise, fading, multipath, and how to simulate real-world conditions.

---

## Quick Reference

```
┌──────────────────────────────────────────────────────────────┐
│              Spread Spectrum Reference                       │
├──────────────────────────────────────────────────────────────┤
│ Processing Gain = 10 × log₁₀(Spread_BW / Data_BW)            │
├──────────────────────────────────────────────────────────────┤
│ DSSS:                                                        │
│   Tx: data XOR spreading_code → BPSK                         │
│   Rx: correlate with same code → decision                    │
│   PG = 10 × log₁₀(chips_per_bit)                             │
├──────────────────────────────────────────────────────────────┤
│ FHSS:                                                        │
│   Hop pattern from PN generator                              │
│   PG ≈ 10 × log₁₀(num_channels)                              │
├──────────────────────────────────────────────────────────────┤
│ LoRa CSS:                                                    │
│   Symbol = circular-shifted chirp                            │
│   Demod: dechirp × FFT → peak bin = symbol                   │
│   PG = SF (in dB)                                            │
│                                                              │
│   SF7:  128 symbols, ~3.9 kbps (125kHz BW)                   │
│   SF12: 4096 symbols, ~250 bps (125kHz BW)                   │
├──────────────────────────────────────────────────────────────┤
│ LoRa Link Budget:                                            │
│   Sensitivity ≈ -174 + 10log₁₀(BW) + NF - SF                 │
│   SF12 @ 125kHz: -137 dBm (incredible!)                      │
└──────────────────────────────────────────────────────────────┘
```

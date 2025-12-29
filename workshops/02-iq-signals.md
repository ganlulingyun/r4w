# Workshop 02: Understanding I/Q Signals

**Duration:** 45 minutes
**Difficulty:** Beginner
**Prerequisites:** Workshop 01 completed

## Objectives

By the end of this workshop, you will:
- Understand what I/Q (In-phase/Quadrature) signals are
- Know why complex numbers are used in SDR
- Visualize samples in time and frequency domains
- Interpret constellation diagrams

---

## 1. What Are I/Q Signals?

### 1.1 The Simple Answer

I/Q signals represent **two orthogonal components** of a signal:
- **I (In-phase)**: The cosine component
- **Q (Quadrature)**: The sine component (90° shifted)

Together, they can represent **any amplitude and phase** of a carrier.

### 1.2 The Math (Don't Panic!)

A signal can be written as:
```
s(t) = I(t) × cos(2πft) - Q(t) × sin(2πft)
```

Or more elegantly using complex numbers:
```
s(t) = (I + jQ) × e^(j2πft)
```

Where `j = √-1` (the imaginary unit).

### 1.3 Why Complex Numbers?

| Real-Only (Scalar) | Complex (I/Q) |
|-------------------|---------------|
| Amplitude only | Amplitude AND phase |
| Can't distinguish +f from -f | Full frequency information |
| Requires 2x sample rate | Efficient sampling |
| Used in old radios | Used in modern SDR |

---

## 2. Visualizing I/Q in R4W

### 2.1 The IQSample Type

In R4W, samples are defined as:

```rust
// From r4w-core/src/types.rs
pub type IQSample = Complex<f64>;

// Which means:
pub struct Complex<f64> {
    pub re: f64,  // I (In-phase)
    pub im: f64,  // Q (Quadrature)
}
```

### 2.2 Creating Samples in Code

```rust
use r4w_core::types::IQSample;
use num_complex::Complex;

// Create a sample at 45° with amplitude 1.0
let sample = IQSample::new(0.707, 0.707);  // I=0.707, Q=0.707

// Get magnitude and phase
let magnitude = sample.norm();  // 1.0
let phase = sample.arg();       // π/4 (45°)

// Create from polar coordinates
let sample2 = Complex::from_polar(1.0, std::f64::consts::FRAC_PI_4);
```

### 2.3 Launch Explorer

```bash
cargo run --bin r4w-explorer
```

---

## 3. Hands-On: The Constellation Diagram

### 3.1 What Is a Constellation?

A constellation diagram shows **where symbols land** in the I/Q plane.

```
           Q (Imaginary)
           ▲
           │    *  ← QPSK symbol at (I=0.7, Q=0.7)
           │   ╱
           │  ╱
           │ ╱ angle = 45°
    ───────┼──────────► I (Real)
           │
           │
           │
```

### 3.2 Exercise: Compare Modulations

In the Explorer:

1. **Select BPSK**
   - How many constellation points? (Answer: 2)
   - Where are they? (Answer: ±1 on the I-axis)

2. **Select QPSK**
   - How many points? (Answer: 4)
   - Notice they're at 45°, 135°, 225°, 315°

3. **Select 8-PSK**
   - How many points? (Answer: 8)
   - Are they closer together or farther apart?

### 3.3 Why It Matters

- **More points = more bits per symbol**
- **Closer points = easier to confuse with noise**
- This is the fundamental trade-off in digital modulation!

---

## 4. Time Domain View

### 4.1 What You See

The time domain shows the I and Q values over time:

```
I(t)  ▲      ╱╲      ╱╲      ╱╲
      │    ╱    ╲  ╱    ╲  ╱    ╲
      │  ╱        ╲        ╲        ╲
time ─┼──────────────────────────────────►
      │
      │

Q(t)  ▲        ╱╲      ╱╲
      │      ╱    ╲  ╱    ╲
      │    ╱        ╲        ╲
time ─┼──────────────────────────────────►
```

### 4.2 Exercise: See the Carrier

In Explorer:
1. Generate a simple CW (Continuous Wave) signal
2. Watch the time domain - you'll see a clean sine wave
3. Add noise (lower SNR) - see how it gets fuzzy

---

## 5. Frequency Domain (FFT)

### 5.1 What the Spectrum Shows

The FFT reveals the **frequency content** of your signal:

```
Power ▲
      │     ▄▄▄
      │   ▄██████▄
      │ ▄████████████▄
      │████████████████▄
      └─────────┬───────────► Frequency
             center
```

### 5.2 Key Observations

- **Bandwidth**: Width of the spectral lump
- **Center Frequency**: Where the signal is located
- **Power**: Height of the spectrum

### 5.3 Exercise: Bandwidth vs Data Rate

1. Select BPSK with low symbol rate
2. Note the bandwidth
3. Increase the symbol rate
4. See how bandwidth increases!

**Key insight**: Higher data rate = more bandwidth

---

## 6. Sample Rate and Nyquist

### 6.1 The Nyquist Theorem

To capture a signal of bandwidth B:
```
Sample Rate ≥ 2 × B
```

In practice, we use:
```
Sample Rate = 2.5 to 4 × B  (for margin)
```

### 6.2 Why It Matters in R4W

When you configure an SDR:
```rust
let mut config = SdrConfig::default();
config.sample_rate = 1_000_000.0;  // 1 MS/s
config.bandwidth = 200_000.0;       // 200 kHz
config.frequency = 915_000_000.0;   // 915 MHz
```

The sample rate must be at least 2× the bandwidth!

---

## 7. Code Exercise: Create and Analyze Samples

### 7.1 Generate a Tone

Create a new file `iq_exercise.rs`:

```rust
use r4w_core::types::IQSample;
use std::f64::consts::PI;

fn main() {
    let sample_rate = 1000.0;  // 1 kHz
    let frequency = 100.0;     // 100 Hz tone
    let duration = 0.1;        // 100 ms

    let num_samples = (sample_rate * duration) as usize;

    let samples: Vec<IQSample> = (0..num_samples)
        .map(|n| {
            let t = n as f64 / sample_rate;
            let phase = 2.0 * PI * frequency * t;
            IQSample::new(phase.cos(), phase.sin())
        })
        .collect();

    // Print first 10 samples
    for (i, s) in samples.iter().take(10).enumerate() {
        println!("Sample {}: I={:.3}, Q={:.3}, Mag={:.3}, Phase={:.1}°",
            i, s.re, s.im, s.norm(), s.arg().to_degrees());
    }
}
```

### 7.2 What to Observe

- Magnitude stays constant (1.0)
- Phase rotates around (0° → 360° → ...)
- I and Q are 90° out of phase

---

## 8. The I/Q Plane Animation

Imagine a point rotating on a circle:

```
Frame 1:    Frame 2:    Frame 3:    Frame 4:
    ▲           ▲           ▲           ▲
    │ *         │  *        │           │
    │           │    *      │       *   │
────┼──►    ────┼──►    ────┼──►    ────┼──►
    │           │           │     *     │
    │           │           │  *        │ *
```

This is what's happening to a carrier signal:
- The point rotates at the carrier frequency
- The I value is the x-coordinate (cosine)
- The Q value is the y-coordinate (sine)

---

## 9. Key Takeaways

1. **I/Q = Complete Information**
   - Real samples lose half the information
   - Complex samples capture amplitude AND phase

2. **Constellation = Symbol Alphabet**
   - Each point represents a unique symbol
   - More points = more bits, but less noise margin

3. **Time Domain = Raw Waveform**
   - See the actual I and Q traces
   - Good for seeing noise, transients

4. **Frequency Domain = Bandwidth**
   - See how much spectrum your signal uses
   - Essential for regulatory compliance

5. **Sample Rate = Capture Speed**
   - Must be at least 2× bandwidth
   - Higher is safer, but uses more memory

---

## 10. Next Steps

Continue to [Workshop 03: Basic Modulation](03-basic-modulation.md) to learn how we encode information into I/Q signals using CW, OOK, and ASK.

---

## Quick Reference: I/Q Essentials

```
┌─────────────────────────────────────────────────────┐
│               I/Q Quick Reference                   │
├─────────────────────────────────────────────────────┤
│ IQSample = Complex<f64> = { re: I, im: Q }          │
│                                                     │
│ Magnitude:  sample.norm() = √(I² + Q²)              │
│ Phase:      sample.arg()  = atan2(Q, I)             │
│                                                     │
│ From Polar: Complex::from_polar(mag, phase)         │
│ To Polar:   (sample.norm(), sample.arg())           │
├─────────────────────────────────────────────────────┤
│ Nyquist: Sample Rate ≥ 2 × Bandwidth                │
│                                                     │
│ Bandwidth = Data Rate × (1 + Roll-off)              │
├─────────────────────────────────────────────────────┤
│ Constellation Points = 2^(bits per symbol)          │
│   BPSK:  2 points  = 1 bit/symbol                   │
│   QPSK:  4 points  = 2 bits/symbol                  │
│   8PSK:  8 points  = 3 bits/symbol                  │
│   16QAM: 16 points = 4 bits/symbol                  │
└─────────────────────────────────────────────────────┘
```

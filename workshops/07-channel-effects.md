# Workshop 07: Channel Effects and Simulation

**Duration:** 60 minutes
**Difficulty:** Intermediate
**Prerequisites:** Workshops 01-06 completed

## Objectives

By the end of this workshop, you will:
- Understand common channel impairments
- Add AWGN (Additive White Gaussian Noise) to signals
- Simulate Rayleigh and Rician fading
- Model multipath and delay spread
- Use R4W's channel simulation features

---

## 1. Why Simulate Channels?

### 1.1 The Real World Is Messy

Your signal faces many challenges:

```
Transmitter                                      Receiver
    ┌──┐                                          ┌──┐
    │TX│ ═══════════════════════════════════════► │RX│
    └──┘         ↑         ↑         ↑            └──┘
                 │         │         │
              Noise    Fading   Multipath
               🌧️        📡🔀       🏢🏢
```

### 1.2 Channel Effects Summary

| Effect | Cause | Impact |
|--------|-------|--------|
| AWGN | Thermal noise, interference | Bit errors |
| Fading | Moving objects, reflections | Signal strength varies |
| Multipath | Reflections from buildings | Inter-symbol interference |
| Doppler | Movement | Frequency shift |
| Path Loss | Distance | Signal attenuation |

---

## 2. AWGN: Additive White Gaussian Noise

### 2.1 What Is AWGN?

The simplest noise model:
- **Additive**: Added to signal
- **White**: Equal power at all frequencies
- **Gaussian**: Amplitude follows bell curve

```
Clean signal:           With AWGN:
    ●     ●              ○···●·    ●○·
                        ·  ···· ○· ···
    ●     ●              ●··  ○   ●···
                           Signal + Noise cloud
```

### 2.2 Signal-to-Noise Ratio (SNR)

SNR measures signal quality:

```
SNR (dB) = 10 × log₁₀(Signal Power / Noise Power)
```

| SNR | Quality |
|-----|---------|
| > 20 dB | Excellent |
| 10-20 dB | Good |
| 0-10 dB | Marginal |
| < 0 dB | Below noise floor |

### 2.3 Code: Add AWGN

```rust
use r4w_core::types::IQSample;
use rand::Rng;
use rand_distr::{Distribution, Normal};

fn add_awgn(samples: &[IQSample], snr_db: f64) -> Vec<IQSample> {
    // Calculate signal power
    let signal_power: f64 = samples.iter()
        .map(|s| s.norm_sqr())
        .sum::<f64>() / samples.len() as f64;

    // Calculate noise power from SNR
    let snr_linear = 10.0_f64.powf(snr_db / 10.0);
    let noise_power = signal_power / snr_linear;
    let noise_std = (noise_power / 2.0).sqrt();  // Per I/Q component

    let mut rng = rand::thread_rng();
    let normal = Normal::new(0.0, noise_std).unwrap();

    samples.iter()
        .map(|s| {
            let noise_i = normal.sample(&mut rng);
            let noise_q = normal.sample(&mut rng);
            IQSample::new(s.re + noise_i, s.im + noise_q)
        })
        .collect()
}
```

### 2.4 Explorer Exercise

1. Select any waveform
2. Set SNR = 30 dB → Clean constellation
3. Lower to 10 dB → Fuzzy clouds
4. Lower to 3 dB → Errors start

---

## 3. Fading Channels

### 3.1 Why Signals Fade

In wireless channels, multiple signal copies arrive via different paths:

```
              ┌──────────┐
     Direct   │          │
    ─────────►│          │
              │ Receiver │
    ─────────►│          │
   Reflected  │          │
              └──────────┘
```

When copies combine:
- **In phase** → Stronger signal
- **Out of phase** → Weaker or cancelled

### 3.2 Rayleigh Fading

Used when there's **no direct line of sight**:

```rust
use num_complex::Complex;

fn rayleigh_fading(
    samples: &[IQSample],
    coherence_samples: usize,  // How fast the channel changes
) -> Vec<IQSample> {
    let mut rng = rand::thread_rng();
    let normal = Normal::new(0.0, 1.0 / 2.0_f64.sqrt()).unwrap();

    let mut result = Vec::with_capacity(samples.len());
    let mut current_gain = Complex::new(
        normal.sample(&mut rng),
        normal.sample(&mut rng)
    );

    for (i, sample) in samples.iter().enumerate() {
        if i % coherence_samples == 0 && i > 0 {
            // Channel changes
            current_gain = Complex::new(
                normal.sample(&mut rng),
                normal.sample(&mut rng)
            );
        }

        // Apply fading
        let faded = sample * current_gain;
        result.push(faded);
    }
    result
}
```

### 3.3 Rician Fading

Used when there **is** a direct line of sight:

```rust
fn rician_fading(
    samples: &[IQSample],
    k_factor: f64,  // Ratio of LOS power to scattered power
    coherence_samples: usize,
) -> Vec<IQSample> {
    let mut rng = rand::thread_rng();

    // LOS component power and scattered power
    let los_amp = (k_factor / (k_factor + 1.0)).sqrt();
    let scatter_std = (1.0 / (2.0 * (k_factor + 1.0))).sqrt();
    let normal = Normal::new(0.0, scatter_std).unwrap();

    let mut result = Vec::with_capacity(samples.len());
    let mut scatter = Complex::new(
        normal.sample(&mut rng),
        normal.sample(&mut rng)
    );

    for (i, sample) in samples.iter().enumerate() {
        if i % coherence_samples == 0 && i > 0 {
            scatter = Complex::new(
                normal.sample(&mut rng),
                normal.sample(&mut rng)
            );
        }

        // LOS + scattered
        let gain = Complex::new(los_amp, 0.0) + scatter;
        let faded = sample * gain;
        result.push(faded);
    }
    result
}
```

### 3.4 K-Factor Interpretation

| K-Factor | Meaning |
|----------|---------|
| 0 | Pure Rayleigh (no LOS) |
| 1-3 | Weak LOS |
| 5-10 | Strong LOS |
| > 15 | Near line-of-sight |
| ∞ | AWGN only (no fading) |

---

## 4. Multipath and Delay Spread

### 4.1 What Is Multipath?

Multiple copies of your signal arrive at different times:

```
             Direct (0 ns)
                ↓
Received:  ▁▃▅█▅▃▁
              +
Reflected (50 ns delay, -6 dB):
                    ▁▂▃▂▁
              =
Combined:  ▁▃▅█▆▅▃▂▁
              ↑ ↑
              ISI!
```

### 4.2 Tap Delay Line Model

```rust
struct MultipathChannel {
    taps: Vec<(usize, IQSample)>,  // (delay_samples, complex_gain)
}

impl MultipathChannel {
    fn new() -> Self {
        // Example: 3-tap channel
        Self {
            taps: vec![
                (0, IQSample::new(1.0, 0.0)),           // Direct, 0 dB
                (5, IQSample::new(0.5, 0.3)),          // Reflection 1, ~-5 dB
                (12, IQSample::new(-0.2, 0.1)),        // Reflection 2, ~-13 dB
            ],
        }
    }

    fn apply(&self, samples: &[IQSample]) -> Vec<IQSample> {
        let max_delay = self.taps.iter().map(|(d, _)| *d).max().unwrap_or(0);
        let output_len = samples.len() + max_delay;
        let mut output = vec![IQSample::new(0.0, 0.0); output_len];

        for (delay, gain) in &self.taps {
            for (i, sample) in samples.iter().enumerate() {
                output[i + delay] = output[i + delay] + sample * gain;
            }
        }

        output
    }
}
```

### 4.3 Delay Spread

**RMS Delay Spread** characterizes the channel:

| Environment | Typical Delay Spread |
|-------------|---------------------|
| Indoor | 10-50 ns |
| Urban | 1-5 μs |
| Suburban | 0.5-2 μs |
| Rural | 0.1-1 μs |

If delay spread > symbol duration → **ISI problems**

---

## 5. Doppler Shift

### 5.1 The Effect

Movement causes frequency shift:

```
Δf = f_c × (v / c) × cos(θ)

Where:
  f_c = carrier frequency
  v = relative velocity
  c = speed of light
  θ = angle of arrival
```

### 5.2 Code: Add Doppler

```rust
fn add_doppler(
    samples: &[IQSample],
    freq_shift_hz: f64,
    sample_rate: f64,
) -> Vec<IQSample> {
    let phase_increment = 2.0 * std::f64::consts::PI * freq_shift_hz / sample_rate;

    samples.iter().enumerate()
        .map(|(i, sample)| {
            let phase = phase_increment * i as f64;
            let rotation = IQSample::new(phase.cos(), phase.sin());
            sample * rotation
        })
        .collect()
}
```

---

## 6. R4W Channel Simulator

### 6.1 Using the Built-in Simulator

```rust
use r4w_sim::channel::{ChannelModel, AwgnChannel, RayleighChannel, RicianChannel};

// AWGN only
let awgn = AwgnChannel::new(10.0);  // 10 dB SNR
let noisy = awgn.apply(&samples);

// Rayleigh fading
let rayleigh = RayleighChannel::new(
    100,    // coherence_samples
    10.0,   // snr_db
);
let faded = rayleigh.apply(&samples);

// Rician fading
let rician = RicianChannel::new(
    5.0,    // K-factor
    100,    // coherence_samples
    10.0,   // snr_db
);
let los_faded = rician.apply(&samples);
```

### 6.2 Combining Effects

```rust
fn realistic_channel(samples: &[IQSample]) -> Vec<IQSample> {
    // Step 1: Multipath
    let multipath = MultipathChannel::new();
    let mp_samples = multipath.apply(samples);

    // Step 2: Fading
    let fading = RayleighChannel::new(50, 30.0);  // Mild fading
    let faded = fading.apply(&mp_samples);

    // Step 3: AWGN
    let awgn = AwgnChannel::new(10.0);
    awgn.apply(&faded)
}
```

---

## 7. Explorer Channel Lab

### 7.1 Experiment: SNR Threshold

Find the SNR where BER crosses 10%:

1. Start at SNR = 20 dB
2. Lower in 2 dB steps
3. Note where errors appear
4. Compare BPSK vs QPSK vs 16-QAM

**Expected Results:**
- BPSK: Works to ~4 dB
- QPSK: Works to ~7 dB
- 16-QAM: Works to ~14 dB

### 7.2 Experiment: Fading Visualization

1. Enable Rayleigh fading
2. Watch constellation spin/shrink
3. Note how symbols drift around
4. Compare with Rician (K=10) - more stable

### 7.3 Experiment: Spread Spectrum Advantage

1. Set SNR = 0 dB
2. Try BPSK → Errors everywhere
3. Try LoRa SF10 → Still works!
4. Processing gain in action!

---

## 8. BER vs SNR Curves

### 8.1 Theoretical Performance

```
BER
  1  ─────────────────────────────────────
     │
10⁻¹ │ ╲  ╲  ╲
     │   ╲  ╲  ╲
10⁻² │    ╲  ╲  ╲
     │     ╲  ╲  ╲
10⁻³ │      ╲  ╲  ╲
     │        ╲ ╲  ╲
10⁻⁴ │         ╲ ╲   ╲
     │           ╲ ╲    ╲
10⁻⁵ │            ╲  ╲    ╲
     └───┴───┴───┴───┴───┴───► SNR (dB)
         0   5   10  15  20
         ▲   ▲   ▲
      BPSK QPSK 16QAM
```

### 8.2 Code: Measure BER

```rust
fn measure_ber(
    tx_bits: &[bool],
    rx_bits: &[bool],
) -> f64 {
    let errors: usize = tx_bits.iter()
        .zip(rx_bits.iter())
        .filter(|(a, b)| a != b)
        .count();

    errors as f64 / tx_bits.len() as f64
}

fn snr_sweep(waveform: &mut dyn Waveform, snr_range: &[f64]) -> Vec<(f64, f64)> {
    let test_bits: Vec<bool> = (0..10000)
        .map(|i| (i % 2) == 0)
        .collect();

    snr_range.iter()
        .map(|&snr| {
            let modulated = waveform.modulate(&test_bits);
            let noisy = add_awgn(&modulated, snr);
            let demodulated = waveform.demodulate(&noisy);

            let ber = measure_ber(&test_bits, &demodulated);
            (snr, ber)
        })
        .collect()
}
```

---

## 9. Key Takeaways

1. **AWGN is the baseline**: Every system faces thermal noise
2. **Fading is reality**: Wireless channels fade constantly
3. **Rayleigh = no LOS**: Worst case for wireless
4. **Rician = with LOS**: Better, but still fading
5. **Multipath = ISI**: Symbol spacing must exceed delay spread
6. **Doppler = frequency shift**: Matters for high mobility
7. **Simulation enables testing**: Find problems before deployment

---

## 10. Next Steps

Continue to [Workshop 08: Building Your Own Waveform](08-building-waveforms.md) to learn how to implement a complete waveform from scratch.

---

## Quick Reference

```
┌──────────────────────────────────────────────────────────────┐
│                Channel Effects Reference                     │
├──────────────────────────────────────────────────────────────┤
│ AWGN:                                                        │
│   noise_power = signal_power / (10^(SNR_dB/10))              │
│   noise_std = sqrt(noise_power / 2) per component            │
├──────────────────────────────────────────────────────────────┤
│ Fading:                                                      │
│   Rayleigh: No LOS, |h| ~ Rayleigh distribution              │
│   Rician:   With LOS, K = LOS_power / scatter_power          │
│   Coherence time ≈ 1 / Doppler_spread                        │
├──────────────────────────────────────────────────────────────┤
│ Multipath:                                                   │
│   Delay spread = RMS of tap delays                           │
│   Symbol duration > delay spread avoids ISI                  │
├──────────────────────────────────────────────────────────────┤
│ Doppler:                                                     │
│   Δf = f_carrier × (velocity / c) × cos(angle)               │
│   At 900 MHz, 100 km/h → ~83 Hz shift                        │
├──────────────────────────────────────────────────────────────┤
│ Typical SNR Requirements (BER = 10⁻⁵):                       │
│   BPSK:    ~10 dB                                            │
│   QPSK:    ~10 dB (same as BPSK)                             │
│   16-QAM:  ~15 dB                                            │
│   64-QAM:  ~20 dB                                            │
│   LoRa SF7: ~-7 dB (negative! processing gain)               │
│   LoRa SF12: ~-20 dB                                         │
└──────────────────────────────────────────────────────────────┘
```

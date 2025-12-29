# Workshop 08: Building Your Own Waveform

**Duration:** 90 minutes
**Difficulty:** Advanced
**Prerequisites:** Workshops 01-07 completed

## Objectives

By the end of this workshop, you will:
- Understand the Waveform trait architecture
- Build a complete custom waveform from scratch
- Add visualization and educational features
- Integrate with the Explorer GUI
- Test and validate your waveform

---

## 1. The Waveform Trait

### 1.1 Core Interface

Every waveform in R4W implements this trait:

```rust
pub trait Waveform: Send + Sync {
    /// Modulate bits to I/Q samples
    fn modulate(&mut self, bits: &[bool]) -> Vec<IQSample>;

    /// Demodulate I/Q samples to bits
    fn demodulate(&mut self, samples: &[IQSample]) -> Vec<bool>;

    /// Get waveform information
    fn info(&self) -> WaveformInfo;

    /// Get constellation points for visualization
    fn constellation_points(&self) -> Vec<IQSample> {
        Vec::new()  // Default: no visualization
    }

    /// Get modulation stages for education
    fn get_modulation_stages(&self) -> Vec<ModulationStage> {
        Vec::new()  // Default: no stages
    }

    /// Get demodulation steps for education
    fn get_demodulation_steps(&self) -> Vec<DemodulationStep> {
        Vec::new()  // Default: no steps
    }
}
```

### 1.2 Supporting Types

```rust
pub struct WaveformInfo {
    pub name: String,
    pub description: String,
    pub samples_per_symbol: usize,
    pub bits_per_symbol: usize,
}

pub struct ModulationStage {
    pub name: String,
    pub description: String,
    pub samples: Vec<IQSample>,
}

pub struct DemodulationStep {
    pub name: String,
    pub description: String,
    pub data: StepData,
}

pub enum StepData {
    Samples(Vec<IQSample>),
    Symbols(Vec<u32>),
    Bits(Vec<bool>),
    Spectrum(Vec<f64>),
}
```

---

## 2. Let's Build: DQPSK Waveform

We'll implement **Differential QPSK** - a phase-change-based modulation.

### 2.1 Why DQPSK?

- No absolute phase reference needed
- Used in Bluetooth, TETRA, and more
- Good learning exercise

### 2.2 The Algorithm

```
DQPSK encodes bits as PHASE CHANGES, not absolute phases:

Dibit  | Phase Change
-------|-------------
  00   |    0°
  01   |   +90°
  11   |  +180°
  10   |   -90°

Example:
  Previous: 45°
  Dibit: 01 (+90°)
  New phase: 45° + 90° = 135°
```

---

## 3. Step-by-Step Implementation

### 3.1 Create the File

Create `crates/r4w-core/src/waveform/dqpsk.rs`:

```rust
//! # Differential QPSK (DQPSK) Waveform
//!
//! DQPSK encodes information in phase transitions rather than
//! absolute phase, eliminating the need for carrier recovery.

use crate::types::IQSample;
use crate::waveform::{
    CommonParams, DemodulationStep, ModulationStage, StepData,
    Waveform, WaveformInfo, VisualizationData,
};
use std::f64::consts::PI;

/// DQPSK waveform implementation.
pub struct DqpskWaveform {
    /// Samples per symbol
    samples_per_symbol: usize,
    /// Current phase state (for differential encoding)
    current_phase: f64,
    /// Last transmitted symbol (for visualization)
    last_stages: Vec<ModulationStage>,
    /// Last demod steps (for education)
    last_demod_steps: Vec<DemodulationStep>,
}

impl DqpskWaveform {
    /// Phase changes for each dibit (00, 01, 11, 10)
    const PHASE_CHANGES: [f64; 4] = [
        0.0,           // 00 → no change
        PI / 2.0,      // 01 → +90°
        PI,            // 11 → +180°
        -PI / 2.0,     // 10 → -90°
    ];

    /// Create a new DQPSK waveform.
    pub fn new(samples_per_symbol: usize) -> Self {
        Self {
            samples_per_symbol,
            current_phase: 0.0,
            last_stages: Vec::new(),
            last_demod_steps: Vec::new(),
        }
    }

    /// Convert two bits to dibit index.
    fn bits_to_dibit(b0: bool, b1: bool) -> usize {
        match (b0, b1) {
            (false, false) => 0,  // 00
            (false, true)  => 1,  // 01
            (true, true)   => 2,  // 11
            (true, false)  => 3,  // 10
        }
    }

    /// Convert dibit index to two bits.
    fn dibit_to_bits(dibit: usize) -> (bool, bool) {
        match dibit {
            0 => (false, false),
            1 => (false, true),
            2 => (true, true),
            3 => (true, false),
            _ => (false, false),
        }
    }

    /// Generate symbol samples at a given phase.
    fn generate_symbol(&self, phase: f64) -> Vec<IQSample> {
        let sample = IQSample::new(phase.cos(), phase.sin());
        vec![sample; self.samples_per_symbol]
    }

    /// Normalize angle to [-π, π].
    fn normalize_angle(angle: f64) -> f64 {
        let mut a = angle;
        while a > PI { a -= 2.0 * PI; }
        while a < -PI { a += 2.0 * PI; }
        a
    }
}
```

### 3.2 Implement Modulation

```rust
impl Waveform for DqpskWaveform {
    fn modulate(&mut self, bits: &[bool]) -> Vec<IQSample> {
        let mut samples = Vec::new();
        let mut stages = Vec::new();

        // Reset phase for each modulation
        self.current_phase = 0.0;

        // Process bits in pairs (dibits)
        for chunk in bits.chunks(2) {
            let b0 = chunk.get(0).copied().unwrap_or(false);
            let b1 = chunk.get(1).copied().unwrap_or(false);

            let dibit = Self::bits_to_dibit(b0, b1);
            let phase_change = Self::PHASE_CHANGES[dibit];

            // Apply differential encoding
            let prev_phase = self.current_phase;
            self.current_phase = Self::normalize_angle(self.current_phase + phase_change);

            // Generate samples
            let symbol_samples = self.generate_symbol(self.current_phase);

            // Record stage for visualization
            stages.push(ModulationStage {
                name: format!("Dibit {:02b} → Δφ={:.0}°",
                    (b0 as u8) << 1 | (b1 as u8),
                    phase_change.to_degrees()),
                description: format!(
                    "Phase: {:.0}° → {:.0}°",
                    prev_phase.to_degrees(),
                    self.current_phase.to_degrees()
                ),
                samples: symbol_samples.clone(),
            });

            samples.extend(symbol_samples);
        }

        self.last_stages = stages;
        samples
    }
```

### 3.3 Implement Demodulation

```rust
    fn demodulate(&mut self, samples: &[IQSample]) -> Vec<bool> {
        let mut bits = Vec::new();
        let mut steps = Vec::new();
        let mut prev_phase = 0.0;

        for (i, chunk) in samples.chunks(self.samples_per_symbol).enumerate() {
            if chunk.is_empty() { continue; }

            // Average samples in symbol period
            let sum: IQSample = chunk.iter().copied().sum();
            let avg = sum / chunk.len() as f64;

            // Get current phase
            let current_phase = avg.arg();

            // Calculate phase change
            let mut phase_diff = current_phase - prev_phase;
            phase_diff = Self::normalize_angle(phase_diff);

            // Decide which dibit based on phase change
            let dibit = if phase_diff > 3.0 * PI / 4.0 || phase_diff < -3.0 * PI / 4.0 {
                2  // 11 → 180°
            } else if phase_diff > PI / 4.0 {
                1  // 01 → +90°
            } else if phase_diff < -PI / 4.0 {
                3  // 10 → -90°
            } else {
                0  // 00 → 0°
            };

            let (b0, b1) = Self::dibit_to_bits(dibit);
            bits.push(b0);
            bits.push(b1);

            // Record step
            steps.push(DemodulationStep {
                name: format!("Symbol {}", i),
                description: format!(
                    "Δφ={:.0}° → dibit {:02b}",
                    phase_diff.to_degrees(),
                    (b0 as u8) << 1 | (b1 as u8)
                ),
                data: StepData::Samples(chunk.to_vec()),
            });

            prev_phase = current_phase;
        }

        self.last_demod_steps = steps;
        bits
    }
```

### 3.4 Implement Metadata Methods

```rust
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "DQPSK".to_string(),
            description: "Differential Quadrature Phase Shift Keying - \
                         encodes data in phase transitions".to_string(),
            samples_per_symbol: self.samples_per_symbol,
            bits_per_symbol: 2,
        }
    }

    fn constellation_points(&self) -> Vec<IQSample> {
        // DQPSK has 4 possible phases (0°, 90°, 180°, 270°)
        vec![
            IQSample::new(1.0, 0.0),    // 0°
            IQSample::new(0.0, 1.0),    // 90°
            IQSample::new(-1.0, 0.0),   // 180°
            IQSample::new(0.0, -1.0),   // 270°
        ]
    }

    fn get_modulation_stages(&self) -> Vec<ModulationStage> {
        self.last_stages.clone()
    }

    fn get_demodulation_steps(&self) -> Vec<DemodulationStep> {
        self.last_demod_steps.clone()
    }
}
```

---

## 4. Adding to the Registry

### 4.1 Update mod.rs

In `crates/r4w-core/src/waveform/mod.rs`, add:

```rust
pub mod dqpsk;
pub use dqpsk::DqpskWaveform;
```

### 4.2 Register in Factory

```rust
impl WaveformFactory {
    pub fn create(name: &str, params: CommonParams) -> Option<Box<dyn Waveform>> {
        match name.to_lowercase().as_str() {
            "dqpsk" => Some(Box::new(DqpskWaveform::new(
                params.samples_per_symbol.unwrap_or(8)
            ))),
            // ... other waveforms
            _ => None,
        }
    }
}
```

---

## 5. Testing Your Waveform

### 5.1 Unit Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dqpsk_roundtrip() {
        let mut waveform = DqpskWaveform::new(8);

        let original = vec![
            false, false,  // 00
            false, true,   // 01
            true, true,    // 11
            true, false,   // 10
        ];

        let modulated = waveform.modulate(&original);
        let demodulated = waveform.demodulate(&modulated);

        assert_eq!(original, demodulated);
    }

    #[test]
    fn test_constellation_points() {
        let waveform = DqpskWaveform::new(8);
        let points = waveform.constellation_points();

        assert_eq!(points.len(), 4);

        // Check they're on the unit circle
        for p in &points {
            assert!((p.norm() - 1.0).abs() < 0.001);
        }
    }

    #[test]
    fn test_differential_encoding() {
        let mut waveform = DqpskWaveform::new(8);

        // Sending 01 twice should give two different absolute phases
        let bits = vec![false, true, false, true];
        let samples = waveform.modulate(&bits);

        // First symbol: 0° + 90° = 90°
        // Second symbol: 90° + 90° = 180°
        let sym1 = &samples[0..8];
        let sym2 = &samples[8..16];

        let phase1 = sym1[0].arg();
        let phase2 = sym2[0].arg();

        assert!((phase1 - PI / 2.0).abs() < 0.01);
        assert!((phase2 - PI).abs() < 0.01);
    }
}
```

### 5.2 Run Tests

```bash
cargo test dqpsk --lib
```

---

## 6. Adding Visualization Support

### 6.1 Eye Diagram Data

```rust
impl DqpskWaveform {
    /// Generate eye diagram data.
    pub fn eye_diagram(&self, samples: &[IQSample]) -> VisualizationData {
        let traces: Vec<Vec<(f64, f64)>> = samples
            .chunks(self.samples_per_symbol * 2)  // 2 symbols per trace
            .map(|chunk| {
                chunk.iter().enumerate()
                    .map(|(i, s)| (i as f64 / self.samples_per_symbol as f64, s.re))
                    .collect()
            })
            .collect();

        VisualizationData::EyeDiagram { traces }
    }
}
```

### 6.2 Transition Diagram

```rust
impl DqpskWaveform {
    /// Get phase transitions for visualization.
    pub fn transition_diagram(&self, bits: &[bool]) -> Vec<(f64, f64)> {
        let mut transitions = Vec::new();
        let mut current_phase = 0.0;

        for chunk in bits.chunks(2) {
            let b0 = chunk.get(0).copied().unwrap_or(false);
            let b1 = chunk.get(1).copied().unwrap_or(false);
            let dibit = Self::bits_to_dibit(b0, b1);
            let phase_change = Self::PHASE_CHANGES[dibit];

            let start = current_phase;
            current_phase = Self::normalize_angle(current_phase + phase_change);
            let end = current_phase;

            transitions.push((start, end));
        }

        transitions
    }
}
```

---

## 7. Explorer Integration

### 7.1 Add to Waveform List

In `crates/r4w-gui/src/app.rs`:

```rust
const WAVEFORM_OPTIONS: &[&str] = &[
    "BPSK", "QPSK", "8-PSK",
    "4-QAM", "16-QAM", "64-QAM",
    "FSK", "GFSK",
    "DQPSK",  // Add here!
    "LoRa",
];
```

### 7.2 What You'll See

In Explorer with DQPSK:
- **Constellation**: 4 points at 0°, 90°, 180°, 270°
- **Time Domain**: Constant amplitude, phase jumps
- **Stages Panel**: Shows each dibit → phase change

---

## 8. Advanced Features

### 8.1 Pulse Shaping

Add raised cosine filtering:

```rust
impl DqpskWaveform {
    /// Apply raised cosine pulse shaping.
    pub fn with_pulse_shaping(mut self, roll_off: f64) -> Self {
        // Implementation would add RRC filter
        self
    }
}
```

### 8.2 Timing Recovery

Add symbol timing recovery for real-world use:

```rust
impl DqpskWaveform {
    /// Early-late gate timing recovery.
    pub fn timing_recovery(&self, samples: &[IQSample]) -> Vec<IQSample> {
        // Gardner algorithm or similar
        todo!()
    }
}
```

---

## 9. Complete Example

Here's the full waveform in one file:

```rust
// crates/r4w-core/src/waveform/dqpsk.rs

use crate::types::IQSample;
use crate::waveform::{
    DemodulationStep, ModulationStage, StepData, Waveform, WaveformInfo,
};
use std::f64::consts::PI;

pub struct DqpskWaveform {
    samples_per_symbol: usize,
    current_phase: f64,
    last_stages: Vec<ModulationStage>,
    last_demod_steps: Vec<DemodulationStep>,
}

impl DqpskWaveform {
    const PHASE_CHANGES: [f64; 4] = [0.0, PI / 2.0, PI, -PI / 2.0];

    pub fn new(samples_per_symbol: usize) -> Self {
        Self {
            samples_per_symbol,
            current_phase: 0.0,
            last_stages: Vec::new(),
            last_demod_steps: Vec::new(),
        }
    }

    fn bits_to_dibit(b0: bool, b1: bool) -> usize {
        ((b0 as usize) << 1) | (b1 as usize)
    }

    fn dibit_to_bits(d: usize) -> (bool, bool) {
        ((d >> 1) & 1 == 1, d & 1 == 1)
    }

    fn normalize(a: f64) -> f64 {
        let mut x = a;
        while x > PI { x -= 2.0 * PI; }
        while x < -PI { x += 2.0 * PI; }
        x
    }
}

impl Waveform for DqpskWaveform {
    fn modulate(&mut self, bits: &[bool]) -> Vec<IQSample> {
        self.current_phase = 0.0;
        let mut out = Vec::new();

        for chunk in bits.chunks(2) {
            let dibit = Self::bits_to_dibit(
                chunk.get(0).copied().unwrap_or(false),
                chunk.get(1).copied().unwrap_or(false),
            );
            self.current_phase = Self::normalize(
                self.current_phase + Self::PHASE_CHANGES[dibit]
            );
            let s = IQSample::new(self.current_phase.cos(), self.current_phase.sin());
            out.extend(std::iter::repeat(s).take(self.samples_per_symbol));
        }
        out
    }

    fn demodulate(&mut self, samples: &[IQSample]) -> Vec<bool> {
        let mut bits = Vec::new();
        let mut prev = 0.0;

        for chunk in samples.chunks(self.samples_per_symbol) {
            if chunk.is_empty() { continue; }
            let avg: IQSample = chunk.iter().copied().sum::<IQSample>() / chunk.len() as f64;
            let curr = avg.arg();
            let diff = Self::normalize(curr - prev);

            let dibit = if diff.abs() > 3.0 * PI / 4.0 { 2 }
                else if diff > PI / 4.0 { 1 }
                else if diff < -PI / 4.0 { 3 }
                else { 0 };

            let (b0, b1) = Self::dibit_to_bits(dibit);
            bits.push(b0);
            bits.push(b1);
            prev = curr;
        }
        bits
    }

    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "DQPSK".into(),
            description: "Differential QPSK".into(),
            samples_per_symbol: self.samples_per_symbol,
            bits_per_symbol: 2,
        }
    }

    fn constellation_points(&self) -> Vec<IQSample> {
        (0..4).map(|i| {
            let a = i as f64 * PI / 2.0;
            IQSample::new(a.cos(), a.sin())
        }).collect()
    }
}
```

---

## 10. Key Takeaways

1. **Waveform trait**: Core interface for all modulations
2. **Modulate/Demodulate**: The essential pair
3. **Stages/Steps**: Enable educational visualization
4. **Constellation**: Visual debugging aid
5. **Testing**: Round-trip validation is essential
6. **Factory pattern**: Easy integration

---

## 11. Next Steps

Continue to [Workshop 09: R4W Explorer Deep Dive](09-explorer-deep-dive.md) to learn every control and visualization in the GUI.

---

## Quick Reference

```
┌───────────────────────────────────────────────────────────────┐
│              Waveform Development Checklist                   │
├───────────────────────────────────────────────────────────────┤
│ □ Create struct with configuration fields                     │
│ □ Implement Waveform trait                                    │
│   □ modulate() - bits → samples                               │
│   □ demodulate() - samples → bits                             │
│   □ info() - metadata                                         │
│   □ constellation_points() - for visualization                │
│ □ Add to mod.rs exports                                       │
│ □ Register in WaveformFactory                                 │
│ □ Write unit tests                                            │
│   □ Round-trip test                                           │
│   □ Edge cases (empty input, odd bits)                        │
│   □ Noise tolerance                                           │
│ □ Add to Explorer waveform list                               │
│ □ Document with examples                                      │
├───────────────────────────────────────────────────────────────┤
│ Optional Enhancements:                                        │
│ □ Pulse shaping                                               │
│ □ Timing recovery                                             │
│ □ Carrier recovery                                            │
│ □ FEC integration                                             │
│ □ FPGA acceleration                                           │
└───────────────────────────────────────────────────────────────┘
```

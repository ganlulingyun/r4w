# R4W Waveform Implementation Prompt

This document provides all the context needed for an AI assistant to implement a new waveform in the R4W platform. Combine this document with a completed waveform specification YAML to generate a full implementation.

## How to Use This Document

1. **Copy this entire document** into a new AI chat session
2. **Append your waveform specification YAML** at the end (in the section marked below)
3. **Ask the AI to implement the waveform**

Example prompt:
```
Using the R4W implementation context and the waveform specification provided below,
please implement this waveform. Create all necessary files including the module,
factory registration, and tests.
```

---

## R4W Platform Context

### Project Structure

```
crates/
  r4w-core/
    src/
      waveform/           # <-- New waveforms go here
        mod.rs            # Module registration and Waveform trait
        ask.rs            # Example: ASK implementation
        fsk.rs            # Example: FSK implementation
        psk.rs            # Example: PSK implementation
        your_waveform.rs  # <-- Create this file
      types.rs            # IQSample and common types
      lib.rs              # Crate exports
```

### Core Types

```rust
/// I/Q Sample (complex baseband sample)
pub struct IQSample {
    pub re: f32,  // In-phase component
    pub im: f32,  // Quadrature component
}

impl IQSample {
    pub fn new(re: f32, im: f32) -> Self { Self { re, im } }
    pub fn from_polar(mag: f32, phase: f32) -> Self {
        Self { re: mag * phase.cos(), im: mag * phase.sin() }
    }
    pub fn norm_sqr(&self) -> f32 { self.re * self.re + self.im * self.im }
    pub fn norm(&self) -> f32 { self.norm_sqr().sqrt() }
    pub fn arg(&self) -> f32 { self.im.atan2(self.re) }
}
```

### The Waveform Trait

Every waveform MUST implement this trait:

```rust
use crate::types::IQSample;
use serde::{Deserialize, Serialize};
use std::fmt::Debug;

/// Information about a waveform for display and education
#[derive(Debug, Clone, Serialize)]
pub struct WaveformInfo {
    pub name: &'static str,           // Short name (e.g., "QPSK")
    pub full_name: &'static str,      // Full name (e.g., "Quadrature Phase Shift Keying")
    pub description: &'static str,    // Brief description
    pub complexity: u8,               // 1-5 scale
    pub bits_per_symbol: u8,          // 0 for CW, 1 for BPSK, 2 for QPSK, etc.
    pub carries_data: bool,           // true for data-carrying waveforms
    pub characteristics: &'static [&'static str],  // Key features for education
    pub history: &'static str,        // Historical background
    pub modern_usage: &'static str,   // Current applications
}

/// Common parameters shared by all waveforms
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommonParams {
    pub sample_rate: f64,    // Samples per second
    pub carrier_freq: f64,   // Carrier frequency (0.0 for baseband)
    pub amplitude: f64,      // Signal amplitude (0.0 to 1.0)
}

impl Default for CommonParams {
    fn default() -> Self {
        Self {
            sample_rate: 125_000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        }
    }
}

/// Result of demodulation
#[derive(Debug, Clone, Default)]
pub struct DemodResult {
    pub bits: Vec<u8>,           // Decoded bits (packed into bytes)
    pub symbols: Vec<u16>,       // Decoded symbols
    pub ber_estimate: Option<f64>,
    pub snr_estimate: Option<f64>,
    pub metadata: HashMap<String, f64>,
}

/// Visualization data for educational display
#[derive(Debug, Clone)]
pub struct VisualizationData {
    pub samples: Vec<IQSample>,          // Time-domain I/Q samples
    pub constellation: Vec<IQSample>,    // Constellation diagram points
    pub constellation_labels: Vec<String>,
    pub spectrum: Vec<f64>,
    pub description: String,
}

/// The main waveform trait
pub trait Waveform: Debug + Send + Sync {
    /// Get information about this waveform
    fn info(&self) -> WaveformInfo;

    /// Get the common parameters
    fn common_params(&self) -> &CommonParams;

    /// Modulate data into I/Q samples
    /// Input: raw bytes or individual bits (depending on waveform)
    fn modulate(&self, data: &[u8]) -> Vec<IQSample>;

    /// Demodulate I/Q samples back to data
    fn demodulate(&self, samples: &[IQSample]) -> DemodResult;

    /// Get the number of samples per symbol
    fn samples_per_symbol(&self) -> usize;

    /// Get visualization data for educational display (optional override)
    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        // Default implementation - override for better visuals
        let samples = self.modulate(data);
        VisualizationData {
            samples,
            constellation: Vec::new(),
            constellation_labels: Vec::new(),
            spectrum: Vec::new(),
            description: format!("{} modulated signal", self.info().name),
        }
    }
}
```

### Example Implementation (ASK - Amplitude Shift Keying)

Here's a complete example of a simple waveform implementation:

```rust
//! ASK (Amplitude Shift Keying) - Digital Amplitude Modulation
//!
//! ASK encodes digital data by mapping bits/symbols to discrete amplitude levels.
//!
//! ## Mathematical Definition
//!
//! s(t) = A[n] * cos(2*pi*fc*t)
//!
//! Where:
//! - A[n] = amplitude level for symbol n
//! - fc = carrier frequency

use super::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};
use crate::types::IQSample;
use std::f64::consts::PI;

/// ASK modulator/demodulator
#[derive(Debug, Clone)]
pub struct ASK {
    common: CommonParams,
    symbol_rate: f64,
    carrier_freq: f64,
    modulation_index: f64,  // 0.0 to 1.0
    num_levels: usize,      // 2 for binary, 4 for 4-ASK
}

impl ASK {
    pub fn new(
        common: CommonParams,
        symbol_rate: f64,
        carrier_freq: f64,
        modulation_index: f64,
    ) -> Self {
        Self {
            common,
            symbol_rate,
            carrier_freq,
            modulation_index,
            num_levels: 2,
        }
    }

    /// Create Binary ASK (2 amplitude levels)
    pub fn new_binary(common: CommonParams, symbol_rate: f64, carrier_freq: f64) -> Self {
        Self::new(common, symbol_rate, carrier_freq, 1.0)
    }

    /// Get samples per symbol
    fn sps(&self) -> usize {
        ((self.common.sample_rate / self.symbol_rate) as usize).max(1)
    }

    /// Map symbol to amplitude level
    fn symbol_to_amplitude(&self, symbol: u8) -> f64 {
        if symbol == 0 {
            1.0 - self.modulation_index
        } else {
            1.0 + self.modulation_index
        }
    }
}

impl Waveform for ASK {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "ASK",
            full_name: "Amplitude Shift Keying",
            description: "Encodes digital data by mapping symbols to amplitude levels",
            complexity: 2,
            bits_per_symbol: 1,
            carries_data: true,
            characteristics: &[
                "Discrete amplitude levels for symbols",
                "Simple envelope detection",
                "Susceptible to noise/fading",
            ],
            history: "ASK evolved from analog AM for digital data transmission.",
            modern_usage: "RFID systems, NFC, optical communications.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        let sps = self.sps();
        let omega = 2.0 * PI * self.carrier_freq / self.common.sample_rate;
        let base_amp = self.common.amplitude;

        let mut samples = Vec::new();
        let mut phase = 0.0;

        for &bit in data {
            let envelope = self.symbol_to_amplitude(bit & 1);
            for n in 0..sps {
                let p = phase + omega * n as f64;
                let amp = base_amp * envelope;
                samples.push(IQSample::new(
                    (amp * p.cos()) as f32,
                    (amp * p.sin()) as f32,
                ));
            }
            phase += omega * sps as f64;
        }

        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let sps = self.sps();
        let mut result = DemodResult::default();

        // Envelope detection
        for chunk in samples.chunks(sps) {
            let power: f64 = chunk.iter()
                .map(|s| s.norm_sqr() as f64)
                .sum::<f64>() / chunk.len() as f64;
            let envelope = power.sqrt();

            // Decision threshold
            let threshold = self.common.amplitude;
            let bit = if envelope > threshold { 1u8 } else { 0u8 };

            result.bits.push(bit);
            result.symbols.push(bit as u16);
        }

        result
    }

    fn samples_per_symbol(&self) -> usize {
        self.sps()
    }

    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);

        // Constellation: points on real axis
        let constellation = vec![
            IQSample::new((self.common.amplitude * (1.0 - self.modulation_index)) as f32, 0.0),
            IQSample::new((self.common.amplitude * (1.0 + self.modulation_index)) as f32, 0.0),
        ];

        VisualizationData {
            samples,
            constellation,
            constellation_labels: vec!["0".to_string(), "1".to_string()],
            spectrum: Vec::new(),
            description: format!("ASK: m={:.0}%, fc={:.0} Hz",
                self.modulation_index * 100.0, self.carrier_freq),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ask_roundtrip() {
        let common = CommonParams {
            sample_rate: 10000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let ask = ASK::new_binary(common, 100.0, 1000.0);

        let data = vec![0, 1, 1, 0, 1, 0, 0, 1];
        let samples = ask.modulate(&data);
        let result = ask.demodulate(&samples);

        assert_eq!(result.bits, data);
    }
}
```

### WaveformFactory Registration

After creating your waveform, register it in `crates/r4w-core/src/waveform/mod.rs`:

```rust
// 1. Add module declaration at top of file:
pub mod your_waveform;

// 2. Add to WaveformFactory::list():
pub fn list() -> Vec<&'static str> {
    vec![
        // ... existing waveforms ...
        "YOUR-WAVEFORM",  // Add your waveform
    ]
}

// 3. Add to WaveformFactory::create():
pub fn create(name: &str, sample_rate: f64) -> Option<Box<dyn Waveform>> {
    let common = CommonParams {
        sample_rate,
        carrier_freq: 0.0,
        amplitude: 1.0,
    };

    match name.to_uppercase().replace("-", "").replace("_", "").as_str() {
        // ... existing matches ...
        "YOURWAVEFORM" => Some(Box::new(your_waveform::YourWaveform::new(common, ...))),
        _ => None,
    }
}
```

### GUI Category Registration (Optional)

To make your waveform appear in the GUI dropdown, add it to a category in `crates/r4w-gui/src/app.rs`:

```rust
// Find the WaveformGroup::waveforms() method and add your waveform to the appropriate category:
impl WaveformGroup {
    pub fn waveforms(&self) -> &[&str] {
        match self {
            Self::Simple => &["CW"],
            Self::Pulse => &["OOK", "PPM", "ADS-B"],
            Self::Digital => &["BFSK", "4-FSK", "BPSK", "QPSK", "8-PSK"],
            Self::HighOrder => &["16-QAM", "64-QAM", "256-QAM"],
            Self::Analog => &["AM", "FM", "YOUR-WAVEFORM"],  // <-- Add here for analog
            Self::MultiCarrier => &["OFDM"],
            Self::SpreadSpectrum => &["DSSS", "DSSS-QPSK", "FHSS", "LoRa"],
            Self::IoTRadar => &["Zigbee", "UWB", "FMCW"],
        }
    }
}
```

Categories:
- **Simple**: Basic tones (CW)
- **Pulse**: On-off and pulse modulation (OOK, PPM, ADS-B)
- **Digital**: Basic digital modulations (FSK, PSK)
- **HighOrder**: High spectral efficiency (QAM)
- **Analog**: Analog modulations (AM, FM)
- **MultiCarrier**: OFDM-based
- **SpreadSpectrum**: Spread spectrum (DSSS, FHSS, LoRa/CSS)
- **IoTRadar**: IoT protocols and radar (Zigbee, UWB, FMCW)

### Testing Requirements

Every waveform implementation MUST include:

1. **Roundtrip Test** - Modulate then demodulate should recover original data
2. **Sample Count Test** - Verify correct number of samples generated
3. **Info Test** - Verify WaveformInfo fields are correct

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_roundtrip() {
        let waveform = YourWaveform::new(...);
        let original = vec![0, 1, 0, 1, 1, 0, 0, 1];

        let samples = waveform.modulate(&original);
        let result = waveform.demodulate(&samples);

        assert_eq!(result.bits, original);
    }

    #[test]
    fn test_sample_count() {
        let waveform = YourWaveform::new(...);
        let data = vec![1, 0, 1, 1];

        let samples = waveform.modulate(&data);

        assert_eq!(samples.len(), data.len() * waveform.samples_per_symbol());
    }

    #[test]
    fn test_info() {
        let waveform = YourWaveform::new(...);
        let info = waveform.info();

        assert!(!info.name.is_empty());
        assert!(info.bits_per_symbol > 0);
    }
}
```

### Common Mathematical Patterns

```rust
use std::f64::consts::PI;

// Phase calculation
let omega = 2.0 * PI * carrier_freq / sample_rate;
let phase = omega * sample_index as f64;

// Generate I/Q from phase
let sample = IQSample::new(
    (amplitude * phase.cos()) as f32,
    (amplitude * phase.sin()) as f32,
);

// Samples per symbol
let sps = (sample_rate / symbol_rate) as usize;

// Gray coding (for PSK/QAM)
fn gray_encode(n: u8) -> u8 { n ^ (n >> 1) }
fn gray_decode(n: u8) -> u8 {
    let mut n = n;
    let mut mask = n >> 1;
    while mask != 0 {
        n ^= mask;
        mask >>= 1;
    }
    n
}
```

### Build and Test Commands

```bash
# Build the entire project
cargo build

# Run tests for r4w-core
cargo test -p r4w-core

# Run a specific test
cargo test -p r4w-core -- your_waveform::tests::test_roundtrip

# Check with clippy
cargo clippy -p r4w-core

# Run the GUI to visualize your waveform
cargo run --bin r4w-explorer
```

---

## Waveform Specification

**PASTE YOUR WAVEFORM SPECIFICATION YAML BELOW THIS LINE:**

```yaml
# Your waveform specification goes here
# (Copy from your completed schema.yaml)
```

---

## Implementation Request

Based on the R4W platform context above and the waveform specification provided, please implement:

1. **The waveform module** (`crates/r4w-core/src/waveform/your_waveform.rs`)
   - Struct with all necessary parameters
   - Constructor(s) for common configurations
   - Full `Waveform` trait implementation
   - Helper functions as needed

2. **Module registration** (additions to `mod.rs`)
   - Module declaration
   - Factory list entry
   - Factory create match arm

3. **Tests** (in the same file)
   - Roundtrip test
   - Sample count test
   - Parameter-specific tests

4. **Documentation**
   - Module-level doc comments explaining the waveform
   - Mathematical definition in rustdoc
   - Usage examples

Focus on:
- Correct signal generation matching the specification
- Clean, idiomatic Rust code
- Proper error handling
- Educational value (clear constellation, good visualization)

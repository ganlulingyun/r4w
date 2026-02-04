//! Binary Offset Carrier (BOC) Modulation
//!
//! BOC modulation splits the signal spectrum away from the center frequency,
//! creating a split-spectrum shape. Used by Galileo E1 and GPS L1C.
//!
//! ## BOC(m, n) Parameters
//!
//! - Subcarrier frequency: f_s = m × 1.023 MHz
//! - Chipping rate: f_c = n × 1.023 MHz
//! - BOC(1,1): 1.023 MHz subcarrier, 1.023 Mchip/s (Galileo E1)
//!
//! ## CBOC(6,1,1/11) - Galileo E1
//!
//! Composite BOC using both BOC(1,1) and BOC(6,1) components:
//! ```text
//! CBOC = √(10/11) × BOC(1,1) ± √(1/11) × BOC(6,1)
//! ```

use std::f64::consts::PI;

/// BOC subcarrier generator
#[derive(Debug, Clone)]
pub struct BocSubcarrier {
    /// Subcarrier frequency ratio m (f_s = m × 1.023 MHz)
    m: u8,
    /// Chipping rate ratio n (f_c = n × 1.023 MHz)
    n: u8,
}

impl BocSubcarrier {
    /// Create a new BOC(m, n) subcarrier
    pub fn new(m: u8, n: u8) -> Self {
        Self { m, n }
    }

    /// BOC(1,1) for Galileo E1
    pub fn boc_1_1() -> Self {
        Self::new(1, 1)
    }

    /// BOC(6,1) for Galileo E1 CBOC component
    pub fn boc_6_1() -> Self {
        Self::new(6, 1)
    }

    /// Get the subcarrier value at a given phase (0 to 1)
    /// BOC uses a square wave subcarrier
    pub fn subcarrier(&self, chip_phase: f64) -> f64 {
        // Number of half-periods per chip = 2 * m / n
        let half_periods = 2.0 * self.m as f64 / self.n as f64;
        let sub_phase = (chip_phase * half_periods) % 2.0;
        if sub_phase < 1.0 { 1.0 } else { -1.0 }
    }

    /// Get the subcarrier frequency in Hz
    pub fn subcarrier_freq_hz(&self) -> f64 {
        self.m as f64 * 1_023_000.0
    }

    /// Get the chipping rate in Hz
    pub fn chipping_rate_hz(&self) -> f64 {
        self.n as f64 * 1_023_000.0
    }

    /// Generate BOC-modulated samples for a chip sequence
    pub fn modulate_chips(&self, chips: &[i8], samples_per_chip: usize) -> Vec<f64> {
        let mut samples = Vec::with_capacity(chips.len() * samples_per_chip);
        for (chip_idx, &chip) in chips.iter().enumerate() {
            for s in 0..samples_per_chip {
                let phase = chip_idx as f64 + s as f64 / samples_per_chip as f64;
                let subcarrier = self.subcarrier(phase);
                samples.push(chip as f64 * subcarrier);
            }
        }
        samples
    }
}

/// CBOC (Composite BOC) generator for Galileo E1
///
/// CBOC(6,1,1/11) = sqrt(10/11) × BOC(1,1) ± sqrt(1/11) × BOC(6,1)
/// E1B (data): uses minus sign
/// E1C (pilot): uses plus sign
#[derive(Debug, Clone)]
pub struct CbocGenerator {
    boc_1_1: BocSubcarrier,
    boc_6_1: BocSubcarrier,
    /// Weight for BOC(1,1) component
    w1: f64,
    /// Weight for BOC(6,1) component
    w6: f64,
    /// Sign for BOC(6,1) component (+1 for pilot, -1 for data)
    sign: f64,
}

impl CbocGenerator {
    /// Create CBOC for E1B (data channel)
    pub fn e1b() -> Self {
        Self {
            boc_1_1: BocSubcarrier::boc_1_1(),
            boc_6_1: BocSubcarrier::boc_6_1(),
            w1: (10.0_f64 / 11.0).sqrt(),
            w6: (1.0_f64 / 11.0).sqrt(),
            sign: -1.0, // Data channel
        }
    }

    /// Create CBOC for E1C (pilot channel)
    pub fn e1c() -> Self {
        Self {
            boc_1_1: BocSubcarrier::boc_1_1(),
            boc_6_1: BocSubcarrier::boc_6_1(),
            w1: (10.0_f64 / 11.0).sqrt(),
            w6: (1.0_f64 / 11.0).sqrt(),
            sign: 1.0, // Pilot channel
        }
    }

    /// Get CBOC subcarrier value at a given chip phase
    pub fn subcarrier(&self, chip_phase: f64) -> f64 {
        self.w1 * self.boc_1_1.subcarrier(chip_phase)
            + self.sign * self.w6 * self.boc_6_1.subcarrier(chip_phase)
    }

    /// Generate CBOC-modulated samples
    pub fn modulate_chips(&self, chips: &[i8], samples_per_chip: usize) -> Vec<f64> {
        let mut samples = Vec::with_capacity(chips.len() * samples_per_chip);
        for (chip_idx, &chip) in chips.iter().enumerate() {
            for s in 0..samples_per_chip {
                let phase = chip_idx as f64 + s as f64 / samples_per_chip as f64;
                let subcarrier = self.subcarrier(phase);
                samples.push(chip as f64 * subcarrier);
            }
        }
        samples
    }
}

/// Generate the power spectral density of BOC(m,n)
///
/// Returns (frequency_offset_hz, psd_db) pairs for visualization
pub fn boc_spectrum(m: u8, n: u8, num_points: usize) -> Vec<(f64, f64)> {
    let f_s = m as f64 * 1_023_000.0;
    let f_c = n as f64 * 1_023_000.0;
    let t_c = 1.0 / f_c;

    let mut spectrum = Vec::with_capacity(num_points);
    let f_max = 10.0 * f_s;

    for i in 0..num_points {
        let f = -f_max + 2.0 * f_max * i as f64 / (num_points - 1) as f64;

        // BOC PSD: sinc²(f × Tc) × tan²(π × f / (2 × fs))
        let sinc_arg = f * t_c;
        let sinc = if sinc_arg.abs() < 1e-10 {
            1.0
        } else {
            (PI * sinc_arg).sin() / (PI * sinc_arg)
        };

        let tan_arg = PI * f / (2.0 * f_s);
        let tan_val = if tan_arg.abs() > PI / 2.0 - 0.01 {
            100.0 // Limit near poles
        } else {
            tan_arg.tan()
        };

        let psd = sinc.powi(2) * tan_val.powi(2);
        let psd_db = if psd > 1e-20 { 10.0 * psd.log10() } else { -200.0 };

        spectrum.push((f, psd_db));
    }
    spectrum
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_boc_subcarrier() {
        let boc = BocSubcarrier::boc_1_1();
        // BOC(1,1) has 2 half-periods per chip
        assert_eq!(boc.subcarrier(0.0), 1.0);
        assert_eq!(boc.subcarrier(0.5), -1.0);
    }

    #[test]
    fn test_boc_6_1_subcarrier() {
        let boc = BocSubcarrier::boc_6_1();
        // BOC(6,1) has 12 half-periods per chip
        assert_eq!(boc.subcarrier(0.0), 1.0);
        // Changes sign at 1/12 of a chip
        let near_edge = 1.0 / 12.0 + 0.001;
        assert_eq!(boc.subcarrier(near_edge), -1.0);
    }

    #[test]
    fn test_cboc_weights() {
        let cboc = CbocGenerator::e1b();
        // Verify power normalization: w1² + w6² = 1
        let total_power = cboc.w1.powi(2) + cboc.w6.powi(2);
        assert!((total_power - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_boc_modulate() {
        let boc = BocSubcarrier::boc_1_1();
        let chips = vec![1i8, -1, 1];
        let samples = boc.modulate_chips(&chips, 4);
        assert_eq!(samples.len(), 12); // 3 chips × 4 samples/chip
    }
}

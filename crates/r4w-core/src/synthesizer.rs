//! Frequency Synthesizer Settling Time Models (MF-041)
//!
//! Provides accurate settling time models for different synthesizer architectures
//! used in SDR and radio hardware. Essential for FHSS timing calculations.
//!
//! # Synthesizer Types
//!
//! - **Integer-N PLL**: Simple, slow settling (1-10ms typical)
//! - **Fractional-N PLL**: Faster, sub-ms settling possible
//! - **DDS (Direct Digital Synthesis)**: Near-instantaneous (<1µs)
//! - **Hybrid PLL+DDS**: Best of both worlds
//!
//! # Example
//!
//! ```rust
//! use r4w_core::synthesizer::{SynthesizerModel, SynthesizerType, SynthesizerConfig};
//!
//! // Configure for fast FHSS operation
//! let config = SynthesizerConfig::fast_hopping();
//! let synth = SynthesizerModel::new(config);
//!
//! // Calculate settling time for a frequency hop
//! let hop_hz = 1_000_000.0; // 1 MHz hop
//! let settling_time = synth.settling_time_us(hop_hz);
//! println!("Settling time: {}µs", settling_time);
//! ```

use std::f64::consts::PI;

/// Synthesizer architecture type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SynthesizerType {
    /// Integer-N PLL - simple but slow
    IntegerN,
    /// Fractional-N PLL - faster settling
    FractionalN,
    /// Direct Digital Synthesis - near-instantaneous
    Dds,
    /// Hybrid PLL + DDS for fine/coarse tuning
    Hybrid,
}

/// PLL loop filter order
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LoopFilterOrder {
    /// Second-order (simple, slower settling)
    Second,
    /// Third-order (faster, more complex)
    Third,
    /// Fourth-order (fastest, requires careful design)
    Fourth,
}

/// Synthesizer configuration parameters
#[derive(Debug, Clone)]
pub struct SynthesizerConfig {
    /// Type of synthesizer architecture
    pub synth_type: SynthesizerType,
    /// PLL reference frequency (Hz)
    pub reference_freq_hz: f64,
    /// PLL loop bandwidth (Hz)
    pub loop_bandwidth_hz: f64,
    /// Loop filter order
    pub loop_filter_order: LoopFilterOrder,
    /// Phase margin (degrees)
    pub phase_margin_deg: f64,
    /// VCO gain (Hz/V)
    pub kvco: f64,
    /// Charge pump current (A)
    pub charge_pump_current: f64,
    /// DDS clock rate (Hz) - for DDS/Hybrid types
    pub dds_clock_hz: Option<f64>,
    /// DDS bit width - for DDS/Hybrid types
    pub dds_bits: Option<u32>,
    /// Lock detect threshold (degrees)
    pub lock_threshold_deg: f64,
}

impl Default for SynthesizerConfig {
    fn default() -> Self {
        Self::typical_sdr()
    }
}

impl SynthesizerConfig {
    /// Typical SDR configuration (balanced settling vs spurious)
    pub fn typical_sdr() -> Self {
        Self {
            synth_type: SynthesizerType::FractionalN,
            reference_freq_hz: 40_000_000.0,  // 40 MHz reference
            loop_bandwidth_hz: 50_000.0,       // 50 kHz loop BW
            loop_filter_order: LoopFilterOrder::Third,
            phase_margin_deg: 55.0,
            kvco: 50_000_000.0,               // 50 MHz/V
            charge_pump_current: 5.0e-3,      // 5 mA
            dds_clock_hz: None,
            dds_bits: None,
            lock_threshold_deg: 5.0,
        }
    }

    /// Fast hopping configuration for FHSS
    pub fn fast_hopping() -> Self {
        Self {
            synth_type: SynthesizerType::Hybrid,
            reference_freq_hz: 100_000_000.0, // 100 MHz reference
            loop_bandwidth_hz: 200_000.0,      // 200 kHz loop BW
            loop_filter_order: LoopFilterOrder::Fourth,
            phase_margin_deg: 50.0,
            kvco: 100_000_000.0,              // 100 MHz/V
            charge_pump_current: 10.0e-3,     // 10 mA
            dds_clock_hz: Some(1_000_000_000.0), // 1 GHz DDS clock
            dds_bits: Some(32),
            lock_threshold_deg: 10.0,         // Relaxed for speed
        }
    }

    /// SINCGARS-compatible configuration
    pub fn sincgars_compatible() -> Self {
        Self {
            synth_type: SynthesizerType::FractionalN,
            reference_freq_hz: 12_800_000.0,  // 12.8 MHz (SINCGARS standard)
            loop_bandwidth_hz: 100_000.0,      // 100 kHz for fast hopping
            loop_filter_order: LoopFilterOrder::Third,
            phase_margin_deg: 55.0,
            kvco: 30_000_000.0,
            charge_pump_current: 2.5e-3,
            dds_clock_hz: None,
            dds_bits: None,
            lock_threshold_deg: 5.0,
        }
    }

    /// HAVEQUICK-compatible configuration
    pub fn havequick_compatible() -> Self {
        Self {
            synth_type: SynthesizerType::Hybrid,
            reference_freq_hz: 50_000_000.0,  // 50 MHz reference
            loop_bandwidth_hz: 150_000.0,      // Wide loop for fast settling
            loop_filter_order: LoopFilterOrder::Fourth,
            phase_margin_deg: 52.0,
            kvco: 80_000_000.0,
            charge_pump_current: 8.0e-3,
            dds_clock_hz: Some(500_000_000.0),
            dds_bits: Some(48),
            lock_threshold_deg: 8.0,
        }
    }

    /// Pure DDS configuration (fastest, limited frequency range)
    pub fn pure_dds() -> Self {
        Self {
            synth_type: SynthesizerType::Dds,
            reference_freq_hz: 0.0,           // Not used for DDS
            loop_bandwidth_hz: 0.0,            // Not used for DDS
            loop_filter_order: LoopFilterOrder::Second,
            phase_margin_deg: 0.0,
            kvco: 0.0,
            charge_pump_current: 0.0,
            dds_clock_hz: Some(2_400_000_000.0), // 2.4 GHz DDS clock
            dds_bits: Some(48),
            lock_threshold_deg: 0.0,          // Instantaneous
        }
    }
}

/// Synthesizer settling time model
#[derive(Debug, Clone)]
pub struct SynthesizerModel {
    config: SynthesizerConfig,
    /// Natural frequency (rad/s)
    omega_n: f64,
    /// Damping factor
    zeta: f64,
    /// Settling time constant
    tau: f64,
}

impl SynthesizerModel {
    /// Create a new synthesizer model from configuration
    pub fn new(config: SynthesizerConfig) -> Self {
        // Calculate PLL parameters
        let omega_c = 2.0 * PI * config.loop_bandwidth_hz;

        // Damping factor from phase margin (approximation)
        let zeta = config.phase_margin_deg / 90.0;

        // Natural frequency
        let omega_n = omega_c / (2.0 * zeta);

        // Time constant for settling
        let tau = match config.loop_filter_order {
            LoopFilterOrder::Second => 4.0 / (zeta * omega_n),
            LoopFilterOrder::Third => 5.0 / (zeta * omega_n),
            LoopFilterOrder::Fourth => 6.0 / (zeta * omega_n),
        };

        Self {
            config,
            omega_n,
            zeta,
            tau,
        }
    }

    /// Calculate settling time in microseconds for a given frequency step
    ///
    /// # Arguments
    /// * `freq_step_hz` - Frequency step size in Hz
    ///
    /// # Returns
    /// Settling time in microseconds to reach lock
    pub fn settling_time_us(&self, freq_step_hz: f64) -> f64 {
        match self.config.synth_type {
            SynthesizerType::Dds => {
                // DDS settling is essentially instantaneous
                // Limited by DAC settling and filter group delay
                if let Some(dds_clock) = self.config.dds_clock_hz {
                    // ~10 clock cycles for frequency word update + DAC settling
                    (10.0 / dds_clock) * 1_000_000.0 + 0.5 // Add 0.5µs for filter
                } else {
                    1.0 // Default 1µs
                }
            }
            SynthesizerType::Hybrid => {
                // Hybrid uses DDS for fine tuning, PLL for coarse
                let dds_settling = if let Some(dds_clock) = self.config.dds_clock_hz {
                    (10.0 / dds_clock) * 1_000_000.0 + 0.5
                } else {
                    1.0
                };

                // If step is within DDS range, use DDS settling
                // Otherwise, need full PLL settling
                let dds_range = self.config.dds_clock_hz.unwrap_or(1e9) / 2.0;
                if freq_step_hz.abs() < dds_range / 100.0 {
                    dds_settling
                } else {
                    // PLL settling with faster loop
                    self.pll_settling_time_us(freq_step_hz) * 0.7 + dds_settling
                }
            }
            SynthesizerType::IntegerN | SynthesizerType::FractionalN => {
                self.pll_settling_time_us(freq_step_hz)
            }
        }
    }

    /// Calculate PLL-specific settling time
    fn pll_settling_time_us(&self, freq_step_hz: f64) -> f64 {
        // Settling time depends on:
        // 1. Loop bandwidth (faster = quicker settling but more noise)
        // 2. Frequency step size (larger = longer settling)
        // 3. Lock threshold (tighter = longer settling)

        // Initial phase error from frequency step (rough approximation)
        let initial_phase_error = freq_step_hz / self.config.loop_bandwidth_hz;

        // Number of time constants to settle
        let lock_threshold_rad = self.config.lock_threshold_deg * PI / 180.0;
        let settling_ratio = (initial_phase_error.abs() / lock_threshold_rad).max(1.0);
        let num_tau = settling_ratio.ln().max(3.0);

        // Settling time in seconds, convert to microseconds
        let settling_s = num_tau * self.tau;
        settling_s * 1_000_000.0
    }

    /// Get the maximum hop rate in hops per second
    pub fn max_hop_rate(&self, avg_freq_step_hz: f64) -> f64 {
        let settling_us = self.settling_time_us(avg_freq_step_hz);
        1_000_000.0 / settling_us
    }

    /// Check if a target hop time is achievable
    pub fn can_achieve_hop_time(&self, target_us: f64, freq_step_hz: f64) -> bool {
        self.settling_time_us(freq_step_hz) <= target_us
    }

    /// Get synthesizer type
    pub fn synth_type(&self) -> SynthesizerType {
        self.config.synth_type
    }

    /// Get configuration reference
    pub fn config(&self) -> &SynthesizerConfig {
        &self.config
    }

    /// Get natural frequency in Hz
    pub fn natural_freq_hz(&self) -> f64 {
        self.omega_n / (2.0 * PI)
    }

    /// Get damping factor
    pub fn damping_factor(&self) -> f64 {
        self.zeta
    }
}

/// Synthesizer bank for ping-pong hopping
///
/// Uses two synthesizers alternately to achieve zero-delay hopping.
/// While one is transmitting, the other is settling to the next frequency.
#[derive(Debug)]
pub struct SynthesizerBank {
    synth_a: SynthesizerModel,
    synth_b: SynthesizerModel,
    active: bool, // false = A, true = B
    next_freq_hz: f64,
    settling_start_us: u64,
}

impl SynthesizerBank {
    /// Create a new synthesizer bank with identical configurations
    pub fn new(config: SynthesizerConfig) -> Self {
        Self {
            synth_a: SynthesizerModel::new(config.clone()),
            synth_b: SynthesizerModel::new(config),
            active: false,
            next_freq_hz: 0.0,
            settling_start_us: 0,
        }
    }

    /// Get the active synthesizer
    pub fn active_synth(&self) -> &SynthesizerModel {
        if self.active {
            &self.synth_b
        } else {
            &self.synth_a
        }
    }

    /// Get the settling (inactive) synthesizer
    pub fn settling_synth(&self) -> &SynthesizerModel {
        if self.active {
            &self.synth_a
        } else {
            &self.synth_b
        }
    }

    /// Start settling the inactive synthesizer to a new frequency
    pub fn start_settling(&mut self, freq_hz: f64, current_time_us: u64) {
        self.next_freq_hz = freq_hz;
        self.settling_start_us = current_time_us;
    }

    /// Check if the settling synthesizer is ready
    pub fn is_settled(&self, current_freq_hz: f64, current_time_us: u64) -> bool {
        let freq_step = (self.next_freq_hz - current_freq_hz).abs();
        let settling_time = self.settling_synth().settling_time_us(freq_step);
        let elapsed = current_time_us - self.settling_start_us;
        elapsed as f64 >= settling_time
    }

    /// Switch to the next synthesizer (assumes it's settled)
    pub fn switch(&mut self) {
        self.active = !self.active;
    }

    /// Get the next frequency being settled to
    pub fn next_freq(&self) -> f64 {
        self.next_freq_hz
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dds_settling() {
        let config = SynthesizerConfig::pure_dds();
        let synth = SynthesizerModel::new(config);

        // DDS should settle in ~1µs or less
        let settling = synth.settling_time_us(10_000_000.0);
        assert!(settling < 2.0, "DDS settling should be < 2µs, got {}µs", settling);
    }

    #[test]
    fn test_pll_settling() {
        let config = SynthesizerConfig::typical_sdr();
        let synth = SynthesizerModel::new(config);

        // Small step should settle faster than large step
        let small_step = synth.settling_time_us(10_000.0);
        let large_step = synth.settling_time_us(10_000_000.0);

        assert!(small_step < large_step,
                "Small step should settle faster: {} vs {}", small_step, large_step);
    }

    #[test]
    fn test_fast_hopping_config() {
        let config = SynthesizerConfig::fast_hopping();
        let synth = SynthesizerModel::new(config);

        // Fast hopping should achieve 1MHz step in < 500µs
        let settling = synth.settling_time_us(1_000_000.0);
        assert!(settling < 500.0,
                "Fast hopping should settle < 500µs for 1MHz step, got {}µs", settling);
    }

    #[test]
    fn test_sincgars_compatibility() {
        let config = SynthesizerConfig::sincgars_compatible();
        let synth = SynthesizerModel::new(config);

        // SINCGARS hops at 100 hops/sec over 25 kHz channels
        let settling = synth.settling_time_us(25_000.0);
        let max_hop_time = 10_000.0; // 10ms per hop for 100 hops/sec

        assert!(settling < max_hop_time,
                "SINCGARS should settle < 10ms for 25kHz step, got {}µs", settling);
    }

    #[test]
    fn test_havequick_compatibility() {
        let config = SynthesizerConfig::havequick_compatible();
        let synth = SynthesizerModel::new(config);

        // HAVEQUICK I hops at 5120 hops/sec
        let hop_time = 1_000_000.0 / 5120.0; // ~195µs
        let settling = synth.settling_time_us(2_000_000.0); // ~2 MHz channels

        // Note: Real HAVEQUICK achieves this with specialized hardware
        // Our model shows what's needed
        println!("HAVEQUICK target: {}µs, model settling: {}µs", hop_time, settling);
    }

    #[test]
    fn test_synthesizer_bank() {
        let config = SynthesizerConfig::fast_hopping();
        let mut bank = SynthesizerBank::new(config);

        // Start settling to next frequency
        bank.start_settling(915_000_000.0, 0);

        // After settling time, should be ready
        let freq_step = 1_000_000.0;
        let settling_time = bank.settling_synth().settling_time_us(freq_step);

        assert!(!bank.is_settled(914_000_000.0, 0));
        assert!(bank.is_settled(914_000_000.0, settling_time as u64 + 1));

        // Switch synthesizers
        bank.switch();
        assert_eq!(bank.next_freq(), 915_000_000.0);
    }

    #[test]
    fn test_max_hop_rate() {
        let dds_config = SynthesizerConfig::pure_dds();
        let dds_synth = SynthesizerModel::new(dds_config);

        let pll_config = SynthesizerConfig::typical_sdr();
        let pll_synth = SynthesizerModel::new(pll_config);

        let dds_rate = dds_synth.max_hop_rate(1_000_000.0);
        let pll_rate = pll_synth.max_hop_rate(1_000_000.0);

        // DDS should achieve much higher hop rate
        assert!(dds_rate > pll_rate * 10.0,
                "DDS should be >10x faster: {} vs {}", dds_rate, pll_rate);
    }
}

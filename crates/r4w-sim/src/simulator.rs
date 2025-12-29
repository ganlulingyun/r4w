//! Software SDR Simulator
//!
//! This module provides a pure-software simulation of an SDR device,
//! perfect for testing, learning, and development without hardware.
//!
//! ## Features
//!
//! - Full duplex operation (simultaneous TX/RX)
//! - Configurable channel model (AWGN, fading, etc.)
//! - Realistic timing simulation
//! - Loopback mode for testing
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────┐
//! │                    Simulator                        │
//! │                                                     │
//! │  ┌─────────┐    ┌─────────────┐    ┌─────────────┐  │
//! │  │ TX Buf  │───►│   Channel   │───►│   RX Buf    │  │
//! │  └─────────┘    │   Model     │    └─────────────┘  │
//! │                 └─────────────┘                     │
//! │                                                     │
//! │  write_samples()               read_samples()       │
//! └─────────────────────────────────────────────────────┘
//! ```

use std::collections::VecDeque;
use std::time::Instant;

use r4w_core::timing::{HardwareClock, SampleClock, Timestamp, WallClock};
use r4w_core::types::IQSample;

use crate::channel::{Channel, ChannelConfig};
use crate::device::{DeviceCapabilities, SdrConfig, SdrDevice, SdrError, SdrResult};

/// Software SDR Simulator
///
/// Provides a pure-software simulation of an SDR device with full timing
/// model support. The simulator correlates sample clocks with wall time
/// and provides simulated hardware timestamps.
pub struct Simulator {
    /// Device name
    name: String,
    /// Current configuration
    config: SdrConfig,
    /// Channel model
    channel: Channel,
    /// Receive buffer
    rx_buffer: VecDeque<IQSample>,
    /// Transmit buffer
    tx_buffer: VecDeque<IQSample>,
    /// Is receiving active
    rx_active: bool,
    /// Is transmitting active
    tx_active: bool,
    /// Sample clock for precise DSP timing
    sample_clock: SampleClock,
    /// Wall clock for system time correlation
    wall_clock: WallClock,
    /// Simulated hardware clock
    hw_clock: HardwareClock,
    /// Instant when simulation started (for wall time correlation)
    start_instant: Instant,
    /// Enable loopback (TX → Channel → RX)
    loopback: bool,
    /// Maximum buffer size
    max_buffer_size: usize,
    /// External signal injection (for testing)
    injected_signal: Option<Vec<IQSample>>,
    /// Current position in injected signal
    inject_pos: usize,
}

impl Simulator {
    /// Create a new simulator with default settings
    pub fn new(config: SdrConfig) -> Self {
        let channel_config = ChannelConfig {
            sample_rate: config.sample_rate,
            snr_db: 30.0, // High SNR by default
            ..Default::default()
        };

        let sample_rate = config.sample_rate;
        let now = Instant::now();

        // Create simulated hardware clock at 100 MHz
        let hw_clock = HardwareClock::new(100_000_000.0);

        Self {
            name: "R4W SDR Simulator".to_string(),
            config,
            channel: Channel::new(channel_config),
            rx_buffer: VecDeque::with_capacity(65536),
            tx_buffer: VecDeque::with_capacity(65536),
            rx_active: false,
            tx_active: false,
            sample_clock: SampleClock::new(sample_rate),
            wall_clock: WallClock::now(),
            hw_clock,
            start_instant: now,
            loopback: true, // Loopback enabled by default
            max_buffer_size: 1_000_000,
            injected_signal: None,
            inject_pos: 0,
        }
    }

    /// Create simulator with custom channel model
    pub fn with_channel(config: SdrConfig, channel_config: ChannelConfig) -> Self {
        let mut sim = Self::new(config);
        sim.channel = Channel::new(channel_config);
        sim
    }

    /// Enable or disable loopback mode
    pub fn set_loopback(&mut self, enabled: bool) {
        self.loopback = enabled;
    }

    /// Get channel configuration
    pub fn channel_config(&self) -> &ChannelConfig {
        self.channel.config()
    }

    /// Set channel configuration
    pub fn set_channel_config(&mut self, config: ChannelConfig) {
        self.channel.set_config(config);
    }

    /// Set channel SNR
    pub fn set_snr(&mut self, snr_db: f64) {
        let mut config = self.channel.config().clone();
        config.snr_db = snr_db;
        self.channel.set_config(config);
    }

    /// Inject a signal to be "received"
    ///
    /// This is useful for testing the receiver without a transmitter.
    pub fn inject_signal(&mut self, signal: Vec<IQSample>) {
        self.injected_signal = Some(signal);
        self.inject_pos = 0;
    }

    /// Clear injected signal
    pub fn clear_injected(&mut self) {
        self.injected_signal = None;
        self.inject_pos = 0;
    }

    /// Get the full timestamp with all clocks.
    ///
    /// Returns a `Timestamp` containing:
    /// - Sample clock (precise DSP timing)
    /// - Wall clock (system time)
    /// - Hardware clock (simulated device time)
    pub fn full_timestamp(&self) -> Timestamp {
        Timestamp {
            sample: self.sample_clock.clone(),
            wall: self.wall_clock.clone(),
            hardware: Some(self.hw_clock.clone()),
            synced: None, // Simulator doesn't support GPS/PTP
        }
    }

    /// Get the sample clock.
    pub fn sample_clock(&self) -> &SampleClock {
        &self.sample_clock
    }

    /// Get the wall clock.
    pub fn wall_clock(&self) -> &WallClock {
        &self.wall_clock
    }

    /// Get the hardware clock.
    pub fn hardware_clock(&self) -> &HardwareClock {
        &self.hw_clock
    }

    /// Reset all clocks to zero.
    pub fn reset_time(&mut self) {
        self.sample_clock = SampleClock::new(self.config.sample_rate);
        self.wall_clock = WallClock::now();
        self.hw_clock = HardwareClock::new(100_000_000.0);
        self.start_instant = Instant::now();
    }

    /// Process internal loopback (TX → Channel → RX)
    fn process_loopback(&mut self) {
        if !self.loopback || self.tx_buffer.is_empty() {
            return;
        }

        // Move TX samples through channel to RX
        let tx_samples: Vec<IQSample> = self.tx_buffer.drain(..).collect();
        let rx_samples = self.channel.apply(&tx_samples);

        // Add to RX buffer (with overflow protection)
        for sample in rx_samples {
            if self.rx_buffer.len() < self.max_buffer_size {
                self.rx_buffer.push_back(sample);
            }
        }
    }

    /// Process injected signal
    fn process_injected(&mut self, num_samples: usize) {
        if let Some(ref signal) = self.injected_signal {
            for _ in 0..num_samples {
                if self.inject_pos < signal.len() {
                    if self.rx_buffer.len() < self.max_buffer_size {
                        // Apply channel effects to injected signal
                        let sample = signal[self.inject_pos];
                        let noisy = self.channel.apply(&[sample]);
                        self.rx_buffer.push_back(noisy[0]);
                    }
                    self.inject_pos += 1;
                }
            }
        }
    }
}

impl SdrDevice for Simulator {
    fn name(&self) -> &str {
        &self.name
    }

    fn capabilities(&self) -> DeviceCapabilities {
        DeviceCapabilities {
            can_tx: true,
            can_rx: true,
            full_duplex: true,
            min_frequency: 0.0,
            max_frequency: 10.0e9, // Simulated, so any frequency works
            max_sample_rate: 100.0e6,
            tx_channels: 1,
            rx_channels: 1,
        }
    }

    fn configure(&mut self, config: &SdrConfig) -> SdrResult<()> {
        self.config = config.clone();

        // Update channel sample rate
        let mut ch_config = self.channel.config().clone();
        ch_config.sample_rate = config.sample_rate;
        self.channel.set_config(ch_config);

        // Update sample clock rate
        self.sample_clock.set_sample_rate(config.sample_rate);

        Ok(())
    }

    fn config(&self) -> &SdrConfig {
        &self.config
    }

    fn start_rx(&mut self) -> SdrResult<()> {
        if self.rx_active {
            return Err(SdrError::AlreadyRunning);
        }
        self.rx_active = true;
        self.rx_buffer.clear();
        Ok(())
    }

    fn stop_rx(&mut self) -> SdrResult<()> {
        self.rx_active = false;
        Ok(())
    }

    fn start_tx(&mut self) -> SdrResult<()> {
        if self.tx_active {
            return Err(SdrError::AlreadyRunning);
        }
        self.tx_active = true;
        Ok(())
    }

    fn stop_tx(&mut self) -> SdrResult<()> {
        self.tx_active = false;
        self.tx_buffer.clear();
        Ok(())
    }

    fn read_samples(&mut self, num_samples: usize) -> SdrResult<Vec<IQSample>> {
        if !self.rx_active {
            return Err(SdrError::NotStarted);
        }

        // Process any pending loopback
        self.process_loopback();

        // Process injected signal
        self.process_injected(num_samples);

        // Read from RX buffer
        let available = self.rx_buffer.len().min(num_samples);
        let samples: Vec<IQSample> = self.rx_buffer.drain(..available).collect();

        // Advance all clocks
        let count = samples.len() as u64;
        self.sample_clock.advance(count);

        // Update wall clock based on elapsed real time
        let _elapsed = self.start_instant.elapsed();
        self.wall_clock = WallClock::now();

        // Advance simulated hardware clock (convert sample time to HW ticks)
        let sample_time = self.sample_clock.to_seconds();
        self.hw_clock.set_time_seconds(sample_time);

        Ok(samples)
    }

    fn write_samples(&mut self, samples: &[IQSample]) -> SdrResult<usize> {
        if !self.tx_active {
            return Err(SdrError::NotStarted);
        }

        // Add to TX buffer (with overflow protection)
        let space = self.max_buffer_size.saturating_sub(self.tx_buffer.len());
        let to_write = samples.len().min(space);

        for &sample in &samples[..to_write] {
            self.tx_buffer.push_back(sample);
        }

        Ok(to_write)
    }

    fn rx_available(&self) -> usize {
        self.rx_buffer.len()
    }

    fn tx_available(&self) -> usize {
        self.max_buffer_size.saturating_sub(self.tx_buffer.len())
    }

    fn is_receiving(&self) -> bool {
        self.rx_active
    }

    fn is_transmitting(&self) -> bool {
        self.tx_active
    }

    fn timestamp(&self) -> u64 {
        self.sample_clock.samples()
    }

    fn set_frequency(&mut self, freq: f64) -> SdrResult<()> {
        self.config.frequency = freq;
        Ok(())
    }

    fn set_rx_gain(&mut self, gain: f64) -> SdrResult<()> {
        self.config.rx_gain = gain;
        Ok(())
    }

    fn set_tx_gain(&mut self, gain: f64) -> SdrResult<()> {
        self.config.tx_gain = gain;
        Ok(())
    }
}

/// Convenience function to create a simulator for testing
pub fn create_test_simulator() -> Simulator {
    let config = SdrConfig::default();
    Simulator::new(config)
}

/// Create a simulator with specified SNR
pub fn create_simulator_with_snr(snr_db: f64) -> Simulator {
    let sdr_config = SdrConfig::default();
    let channel_config = ChannelConfig::with_snr(snr_db);
    Simulator::with_channel(sdr_config, channel_config)
}

#[cfg(test)]
mod tests {
    use super::*;
    use r4w_core::types::Complex;

    #[test]
    fn test_simulator_creation() {
        let config = SdrConfig::default();
        let sim = Simulator::new(config);

        assert_eq!(sim.name(), "R4W SDR Simulator");
        assert!(!sim.is_receiving());
        assert!(!sim.is_transmitting());
    }

    #[test]
    fn test_start_stop_rx() {
        let config = SdrConfig::default();
        let mut sim = Simulator::new(config);

        sim.start_rx().unwrap();
        assert!(sim.is_receiving());

        sim.stop_rx().unwrap();
        assert!(!sim.is_receiving());
    }

    #[test]
    fn test_loopback() {
        let config = SdrConfig::default();
        let mut sim = Simulator::new(config);
        sim.set_loopback(true);

        // High SNR for nearly perfect loopback
        sim.set_snr(100.0);

        sim.start_tx().unwrap();
        sim.start_rx().unwrap();

        // Write some samples
        let tx_samples: Vec<IQSample> = (0..100)
            .map(|i| Complex::new(i as f64 / 100.0, 0.0))
            .collect();

        let written = sim.write_samples(&tx_samples).unwrap();
        assert_eq!(written, 100);

        // Read them back
        let rx_samples = sim.read_samples(100).unwrap();
        assert!(!rx_samples.is_empty());
    }

    #[test]
    fn test_inject_signal() {
        let config = SdrConfig::default();
        let mut sim = Simulator::new(config);
        sim.set_snr(100.0); // High SNR

        // Inject a test signal
        let signal: Vec<IQSample> = (0..50)
            .map(|i| Complex::new(1.0, i as f64 * 0.1))
            .collect();
        sim.inject_signal(signal);

        sim.start_rx().unwrap();

        let rx = sim.read_samples(50).unwrap();
        assert_eq!(rx.len(), 50);
    }

    #[test]
    fn test_capabilities() {
        let config = SdrConfig::default();
        let sim = Simulator::new(config);
        let caps = sim.capabilities();

        assert!(caps.can_tx);
        assert!(caps.can_rx);
        assert!(caps.full_duplex);
    }

    #[test]
    fn test_timing_model() {
        let mut config = SdrConfig::default();
        config.sample_rate = 1_000_000.0; // 1 MHz
        let mut sim = Simulator::new(config);

        // Start with 0 samples
        assert_eq!(sim.timestamp(), 0);
        assert_eq!(sim.sample_clock().samples(), 0);

        // Inject and read samples
        let signal: Vec<IQSample> = (0..1000).map(|_| Complex::new(1.0, 0.0)).collect();
        sim.inject_signal(signal);
        sim.start_rx().unwrap();

        let _ = sim.read_samples(1000).unwrap();

        // Should have advanced by 1000 samples
        assert_eq!(sim.timestamp(), 1000);
        assert_eq!(sim.sample_clock().samples(), 1000);

        // At 1 MHz, 1000 samples = 1 ms
        let time_secs = sim.sample_clock().to_seconds();
        assert!((time_secs - 0.001).abs() < 1e-9);

        // Full timestamp should have all clocks
        let ts = sim.full_timestamp();
        assert_eq!(ts.sample.samples(), 1000);
        assert!(ts.hardware.is_some());
    }

    #[test]
    fn test_reset_time() {
        let config = SdrConfig::default();
        let mut sim = Simulator::new(config);

        // Inject and read to advance time
        let signal: Vec<IQSample> = (0..500).map(|_| Complex::new(1.0, 0.0)).collect();
        sim.inject_signal(signal);
        sim.start_rx().unwrap();
        let _ = sim.read_samples(500).unwrap();

        assert_eq!(sim.timestamp(), 500);

        // Reset time
        sim.reset_time();
        assert_eq!(sim.timestamp(), 0);
        assert_eq!(sim.sample_clock().samples(), 0);
    }
}

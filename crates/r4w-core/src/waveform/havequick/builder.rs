//! Builder pattern for HAVEQUICK construction
//!
//! Provides a convenient way to construct HAVEQUICK instances with
//! either simulator components (for testing) or classified implementations.

use super::framework::Havequick;
use super::simulator::*;
use super::traits::*;
use super::types::*;

/// Builder for HAVEQUICK instances
pub struct HavequickBuilder {
    sample_rate: f64,
    hopper: Option<Box<dyn HoppingAlgorithm>>,
    time_sync: Option<Box<dyn TimeSyncProtocol>>,
    net_controller: Option<Box<dyn NetController>>,
    voice_codec: Option<Box<dyn VoiceCodec>>,
    data_modem: Option<Box<dyn DataModem>>,
    transec: Option<Box<dyn TransecProvider>>,
}

impl HavequickBuilder {
    /// Create a new builder with default sample rate
    pub fn new() -> Self {
        Self {
            sample_rate: 48_000.0,
            hopper: None,
            time_sync: None,
            net_controller: None,
            voice_codec: None,
            data_modem: None,
            transec: None,
        }
    }

    /// Create a builder pre-configured with simulator components
    ///
    /// This is suitable for testing and development but provides
    /// NO operational security.
    pub fn simulator() -> Self {
        Self {
            sample_rate: 48_000.0,
            hopper: Some(Box::new(SimulatorHopper::new())),
            time_sync: Some(Box::new(SimulatorTimeSync::new())),
            net_controller: Some(Box::new(SimulatorNetController::new())),
            voice_codec: Some(Box::new(AmVoiceCodec::default())),
            data_modem: Some(Box::new(AskDataModem::default())),
            transec: Some(Box::new(SimulatorTransec::new())),
        }
    }

    /// Set sample rate
    pub fn with_sample_rate(mut self, sample_rate: f64) -> Self {
        self.sample_rate = sample_rate;
        self
    }

    /// Set hopping algorithm implementation
    pub fn with_hopper(mut self, hopper: Box<dyn HoppingAlgorithm>) -> Self {
        self.hopper = Some(hopper);
        self
    }

    /// Set time sync implementation
    pub fn with_time_sync(mut self, time_sync: Box<dyn TimeSyncProtocol>) -> Self {
        self.time_sync = Some(time_sync);
        self
    }

    /// Set net controller implementation
    pub fn with_net_controller(mut self, net_controller: Box<dyn NetController>) -> Self {
        self.net_controller = Some(net_controller);
        self
    }

    /// Set voice codec implementation
    pub fn with_voice_codec(mut self, voice_codec: Box<dyn VoiceCodec>) -> Self {
        self.voice_codec = Some(voice_codec);
        self
    }

    /// Set data modem implementation
    pub fn with_data_modem(mut self, data_modem: Box<dyn DataModem>) -> Self {
        self.data_modem = Some(data_modem);
        self
    }

    /// Set TRANSEC provider implementation
    pub fn with_transec(mut self, transec: Box<dyn TransecProvider>) -> Self {
        self.transec = Some(transec);
        self
    }

    /// Build the HAVEQUICK instance
    ///
    /// Returns an error if any required components are missing.
    pub fn build(self) -> Result<Havequick, HavequickError> {
        let hopper = self.hopper.ok_or(HavequickError::InvalidConfiguration(
            "No hopping algorithm provided".to_string(),
        ))?;

        let time_sync = self.time_sync.ok_or(HavequickError::InvalidConfiguration(
            "No time sync provided".to_string(),
        ))?;

        let net_controller = self.net_controller.ok_or(HavequickError::InvalidConfiguration(
            "No net controller provided".to_string(),
        ))?;

        let voice_codec = self.voice_codec.ok_or(HavequickError::InvalidConfiguration(
            "No voice codec provided".to_string(),
        ))?;

        let data_modem = self.data_modem.ok_or(HavequickError::InvalidConfiguration(
            "No data modem provided".to_string(),
        ))?;

        let transec = self.transec.ok_or(HavequickError::InvalidConfiguration(
            "No TRANSEC provider provided".to_string(),
        ))?;

        Ok(Havequick::new(
            self.sample_rate,
            hopper,
            time_sync,
            net_controller,
            voice_codec,
            data_modem,
            transec,
        ))
    }
}

impl Default for HavequickBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulator_builder() {
        let hq = HavequickBuilder::simulator().build();
        assert!(hq.is_ok());

        let hq = hq.unwrap();
        assert_eq!(hq.sample_rate(), 48_000.0);
    }

    #[test]
    fn test_custom_sample_rate() {
        let hq = HavequickBuilder::simulator()
            .with_sample_rate(96_000.0)
            .build()
            .unwrap();

        assert_eq!(hq.sample_rate(), 96_000.0);
    }

    #[test]
    fn test_incomplete_builder() {
        let result = HavequickBuilder::new().build();
        assert!(result.is_err());
    }
}

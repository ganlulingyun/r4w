//! Link-16 Builder
//!
//! Provides a builder pattern for constructing Link-16 instances.

use super::framework::Link16;
use super::simulator::*;
use super::traits::*;
use super::types::*;

/// Builder for Link-16 instances
pub struct Link16Builder {
    sample_rate: f64,
    hopper: Option<Box<dyn HoppingPattern>>,
    time_sync: Option<Box<dyn TimeSync>>,
    message_codec: Option<Box<dyn MessageCodec>>,
    fec: Option<Box<dyn ErrorCorrection>>,
    interleaver: Option<Box<dyn Interleaver>>,
    transec: Option<Box<dyn TransecProvider>>,
    modem: Option<Box<dyn MskModem>>,
    pulse_formatter: Option<Box<dyn PulseFormatter>>,
    net_controller: Option<Box<dyn NetworkController>>,
    track_db: Option<Box<dyn TrackDatabase>>,
    pulse_mode: PulseMode,
}

impl Link16Builder {
    /// Create a new builder with default sample rate
    pub fn new() -> Self {
        Self {
            sample_rate: 5_000_000.0, // 5 MHz default (supports 3 MHz bandwidth)
            hopper: None,
            time_sync: None,
            message_codec: None,
            fec: None,
            interleaver: None,
            transec: None,
            modem: None,
            pulse_formatter: None,
            net_controller: None,
            track_db: None,
            pulse_mode: PulseMode::Packed2,
        }
    }

    /// Create a builder with simulator implementations
    pub fn simulator() -> Self {
        Self::new()
            .with_hopper(Box::new(SimulatorHopper::new()))
            .with_time_sync(Box::new(SimulatorTimeSync::new()))
            .with_message_codec(Box::new(SimulatorMessageCodec::new()))
            .with_fec(Box::new(SimulatorReedSolomon::default()))
            .with_interleaver(Box::new(SimulatorInterleaver::default()))
            .with_transec(Box::new(SimulatorTransec::new()))
            .with_modem(Box::new(SimulatorMskModem::default()))
            .with_pulse_formatter(Box::new(SimulatorPulseFormatter::default()))
            .with_net_controller(Box::new(SimulatorNetController::new()))
            .with_track_db(Box::new(SimulatorTrackDb::new()))
    }

    /// Set sample rate
    pub fn with_sample_rate(mut self, sample_rate: f64) -> Self {
        self.sample_rate = sample_rate;
        self
    }

    /// Set hopping pattern implementation
    pub fn with_hopper(mut self, hopper: Box<dyn HoppingPattern>) -> Self {
        self.hopper = Some(hopper);
        self
    }

    /// Set time sync implementation
    pub fn with_time_sync(mut self, time_sync: Box<dyn TimeSync>) -> Self {
        self.time_sync = Some(time_sync);
        self
    }

    /// Set message codec implementation
    pub fn with_message_codec(mut self, codec: Box<dyn MessageCodec>) -> Self {
        self.message_codec = Some(codec);
        self
    }

    /// Set FEC implementation
    pub fn with_fec(mut self, fec: Box<dyn ErrorCorrection>) -> Self {
        self.fec = Some(fec);
        self
    }

    /// Set interleaver implementation
    pub fn with_interleaver(mut self, interleaver: Box<dyn Interleaver>) -> Self {
        self.interleaver = Some(interleaver);
        self
    }

    /// Set TRANSEC implementation
    pub fn with_transec(mut self, transec: Box<dyn TransecProvider>) -> Self {
        self.transec = Some(transec);
        self
    }

    /// Set MSK modem implementation
    pub fn with_modem(mut self, modem: Box<dyn MskModem>) -> Self {
        self.modem = Some(modem);
        self
    }

    /// Set pulse formatter implementation
    pub fn with_pulse_formatter(mut self, formatter: Box<dyn PulseFormatter>) -> Self {
        self.pulse_formatter = Some(formatter);
        self
    }

    /// Set network controller implementation
    pub fn with_net_controller(mut self, controller: Box<dyn NetworkController>) -> Self {
        self.net_controller = Some(controller);
        self
    }

    /// Set track database implementation
    pub fn with_track_db(mut self, track_db: Box<dyn TrackDatabase>) -> Self {
        self.track_db = Some(track_db);
        self
    }

    /// Set pulse mode
    pub fn with_pulse_mode(mut self, mode: PulseMode) -> Self {
        self.pulse_mode = mode;
        self
    }

    /// Build the Link-16 instance
    pub fn build(self) -> Result<Link16, Link16Error> {
        let hopper = self
            .hopper
            .ok_or_else(|| Link16Error::EncodingError("Hopper required".into()))?;
        let time_sync = self
            .time_sync
            .ok_or_else(|| Link16Error::EncodingError("TimeSync required".into()))?;
        let message_codec = self
            .message_codec
            .ok_or_else(|| Link16Error::EncodingError("MessageCodec required".into()))?;
        let fec = self
            .fec
            .ok_or_else(|| Link16Error::EncodingError("FEC required".into()))?;
        let interleaver = self
            .interleaver
            .ok_or_else(|| Link16Error::EncodingError("Interleaver required".into()))?;
        let transec = self
            .transec
            .ok_or_else(|| Link16Error::EncodingError("TRANSEC required".into()))?;
        let modem = self
            .modem
            .ok_or_else(|| Link16Error::EncodingError("Modem required".into()))?;
        let pulse_formatter = self
            .pulse_formatter
            .ok_or_else(|| Link16Error::EncodingError("PulseFormatter required".into()))?;
        let net_controller = self
            .net_controller
            .ok_or_else(|| Link16Error::EncodingError("NetworkController required".into()))?;
        let track_db = self
            .track_db
            .ok_or_else(|| Link16Error::EncodingError("TrackDatabase required".into()))?;

        let mut link16 = Link16::new(
            self.sample_rate,
            hopper,
            time_sync,
            message_codec,
            fec,
            interleaver,
            transec,
            modem,
            pulse_formatter,
            net_controller,
            track_db,
        );

        link16.set_pulse_mode(self.pulse_mode);

        Ok(link16)
    }
}

impl Default for Link16Builder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulator_builder() {
        let link16 = Link16Builder::simulator()
            .with_sample_rate(2_000_000.0)
            .with_pulse_mode(PulseMode::Standard)
            .build()
            .unwrap();

        assert_eq!(link16.sample_rate(), 2_000_000.0);
    }

    #[test]
    fn test_missing_components() {
        let result = Link16Builder::new().build();
        assert!(result.is_err());
    }
}

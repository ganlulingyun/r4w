//! Builder pattern for SINCGARS waveform
//!
//! Provides a clean API for assembling SINCGARS with either
//! simulator or classified implementations.

use super::traits::*;
use super::framework::Sincgars;
use super::simulator::*;
use super::types::SincgarsError;

/// Builder for SINCGARS waveform
///
/// # Example with Simulator
///
/// ```rust
/// use r4w_core::waveform::sincgars::SincgarsBuilder;
///
/// let sincgars = SincgarsBuilder::simulator().build().unwrap();
/// ```
///
/// # Example with Classified Components
///
/// ```rust,ignore
/// use r4w_core::waveform::sincgars::SincgarsBuilder;
/// use my_classified::*;
///
/// let sincgars = SincgarsBuilder::new()
///     .with_hopper(Box::new(ClassifiedHopper::new()))
///     .with_transec(Box::new(ClassifiedTransec::new()))
///     .with_net_mapper(Box::new(ClassifiedNetMapper::new()))
///     .with_time_sync(Box::new(ClassifiedTimeSync::new()))
///     .with_crypto(Box::new(ClassifiedCrypto::new()))
///     .build()
///     .unwrap();
/// ```
pub struct SincgarsBuilder {
    hopper: Option<Box<dyn HoppingAlgorithm>>,
    transec: Option<Box<dyn TransecProvider>>,
    net_mapper: Option<Box<dyn NetIdMapper>>,
    time_sync: Option<Box<dyn TimeSyncProtocol>>,
    crypto: Option<Box<dyn CryptoProvider>>,
}

impl SincgarsBuilder {
    /// Create a new empty builder
    ///
    /// You must provide all classified components before building.
    pub fn new() -> Self {
        Self {
            hopper: None,
            transec: None,
            net_mapper: None,
            time_sync: None,
            crypto: None,
        }
    }

    /// Create a builder pre-configured with simulator implementations
    ///
    /// This creates a functional but non-secure SINCGARS for testing.
    pub fn simulator() -> Self {
        Self {
            hopper: Some(Box::new(SimulatorHopper::new())),
            transec: Some(Box::new(SimulatorTransec::with_test_key())),
            net_mapper: Some(Box::new(SimulatorNetMapper::new())),
            time_sync: Some(Box::new(SimulatorTimeSync::new())),
            crypto: Some(Box::new(SimulatorCrypto::new())),
        }
    }

    /// Set the hopping algorithm implementation
    pub fn with_hopper(mut self, hopper: Box<dyn HoppingAlgorithm>) -> Self {
        self.hopper = Some(hopper);
        self
    }

    /// Set the TRANSEC provider implementation
    pub fn with_transec(mut self, transec: Box<dyn TransecProvider>) -> Self {
        self.transec = Some(transec);
        self
    }

    /// Set the net ID mapper implementation
    pub fn with_net_mapper(mut self, net_mapper: Box<dyn NetIdMapper>) -> Self {
        self.net_mapper = Some(net_mapper);
        self
    }

    /// Set the time synchronization implementation
    pub fn with_time_sync(mut self, time_sync: Box<dyn TimeSyncProtocol>) -> Self {
        self.time_sync = Some(time_sync);
        self
    }

    /// Set the cryptographic provider implementation
    pub fn with_crypto(mut self, crypto: Box<dyn CryptoProvider>) -> Self {
        self.crypto = Some(crypto);
        self
    }

    /// Build the SINCGARS waveform
    ///
    /// # Errors
    ///
    /// Returns an error if any required component is missing.
    pub fn build(self) -> Result<Sincgars, SincgarsError> {
        let hopper = self.hopper.ok_or_else(|| {
            SincgarsError::InvalidConfiguration("Missing hopping algorithm".into())
        })?;

        let transec = self.transec.ok_or_else(|| {
            SincgarsError::InvalidConfiguration("Missing TRANSEC provider".into())
        })?;

        let net_mapper = self.net_mapper.ok_or_else(|| {
            SincgarsError::InvalidConfiguration("Missing net ID mapper".into())
        })?;

        let time_sync = self.time_sync.ok_or_else(|| {
            SincgarsError::InvalidConfiguration("Missing time sync protocol".into())
        })?;

        let crypto = self.crypto.ok_or_else(|| {
            SincgarsError::InvalidConfiguration("Missing crypto provider".into())
        })?;

        Ok(Sincgars::new(hopper, transec, net_mapper, time_sync, crypto))
    }
}

impl Default for SincgarsBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulator_builder() {
        let sincgars = SincgarsBuilder::simulator().build();
        assert!(sincgars.is_ok());
    }

    #[test]
    fn test_empty_builder_fails() {
        let result = SincgarsBuilder::new().build();
        assert!(result.is_err());
    }

    #[test]
    fn test_partial_builder_fails() {
        let result = SincgarsBuilder::new()
            .with_hopper(Box::new(SimulatorHopper::new()))
            .build();
        assert!(result.is_err());
    }

    #[test]
    fn test_custom_builder() {
        let result = SincgarsBuilder::new()
            .with_hopper(Box::new(SimulatorHopper::new()))
            .with_transec(Box::new(SimulatorTransec::new()))
            .with_net_mapper(Box::new(SimulatorNetMapper::new()))
            .with_time_sync(Box::new(SimulatorTimeSync::new()))
            .with_crypto(Box::new(SimulatorCrypto::new()))
            .build();

        assert!(result.is_ok());
    }
}

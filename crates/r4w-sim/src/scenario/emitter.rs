//! Emitter trait for generic signal sources
//!
//! Any signal source in a scenario (satellite, ground station, jammer, etc.)
//! implements this trait to provide position, IQ generation, and RF parameters.

use r4w_core::coordinates::{EcefPosition, EcefVelocity};
use r4w_core::types::IQSample;

/// State of an emitter at a given time
#[derive(Debug, Clone)]
pub struct EmitterState {
    pub position: EcefPosition,
    pub velocity: EcefVelocity,
    /// Transmit power at this instant in dBm
    pub power_dbm: f64,
    /// Whether the emitter is active/visible
    pub active: bool,
}

/// Generic signal emitter in a scenario
// trace:FR-039 | ai:claude
pub trait Emitter: Send + Sync {
    /// Get emitter state (position, velocity, power) at time t
    fn state_at(&self, t: f64) -> EmitterState;

    /// Generate baseband IQ samples starting at time t
    ///
    /// The samples represent the signal at the emitter output before
    /// propagation effects. The scenario engine applies Doppler, path loss,
    /// and channel effects.
    fn generate_iq(&self, t: f64, num_samples: usize, sample_rate: f64) -> Vec<IQSample>;

    /// Carrier frequency in Hz (used for Doppler scaling)
    fn carrier_frequency_hz(&self) -> f64;

    /// Nominal transmit power in dBm
    fn nominal_power_dbm(&self) -> f64;

    /// Human-readable identifier for this emitter
    fn id(&self) -> String;
}

//! Generic scenario engine for multi-emitter IQ signal generation
//!
//! Provides a framework for composing signals from multiple emitters into a
//! single baseband IQ stream with realistic propagation effects (Doppler,
//! path loss, noise). GNSS-specific models plug in on top via the `Emitter` trait.

pub mod config;
pub mod emitter;
pub mod engine;
pub mod trajectory;

pub use config::ScenarioConfig;
pub use emitter::{Emitter, EmitterState};
pub use engine::{EmitterStatus, ScenarioEngine};
pub use trajectory::{Trajectory, TrajectoryState};

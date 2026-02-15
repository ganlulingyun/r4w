//! Neuromorphic Spike Encoder
//!
//! Encoding of continuous-valued signals to spike trains for neuromorphic computing.
//! Covers rate coding, temporal coding, delta modulation, and leaky integrate-and-fire
//! (LIF) neuron models. Spike trains are the native representation for neuromorphic
//! hardware such as Intel Loihi and IBM TrueNorth.
//!
//! ## Encoding Schemes
//!
//! - **Rate coding**: Signal amplitude maps to spike rate (Poisson-like).
//!   Higher amplitude produces more spikes per unit time.
//! - **Temporal coding**: Time-to-first-spike encoding where stronger stimuli
//!   produce earlier spikes. Encodes information in precise spike timing.
//! - **Delta modulation**: Spikes emitted on threshold crossings of the signal
//!   derivative (up-spike for positive change, down-spike for negative).
//! - **Leaky Integrate-and-Fire (LIF)**: Biophysically-inspired neuron model
//!   with membrane potential, leak current, firing threshold, and refractory period.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::neuromorphic_spike_encoder::{
//!     RateEncoder, SpikeDecoder, spike_rate, spike_count,
//! };
//!
//! // Encode a sine wave to spikes using rate coding
//! let signal: Vec<f64> = (0..100).map(|i| 0.5 + 0.5 * (i as f64 * 0.1).sin()).collect();
//! let mut encoder = RateEncoder::new(100.0, 42);
//! let spikes = encoder.encode(&signal, 0);
//!
//! // Measure spike statistics
//! let rate = spike_rate(&spikes, 100.0);
//! assert!(rate > 0.0);
//! let count = spike_count(&spikes);
//! assert!(count > 0);
//!
//! // Decode spikes back to continuous signal
//! let mut decoder = SpikeDecoder::new(100, 0.05);
//! let recovered = decoder.decode(&spikes, 100);
//! assert_eq!(recovered.len(), 100);
//! ```

/// A single spike event produced by a neuron.
///
/// Contains the timestamp (sample index) at which the spike occurred and
/// the neuron ID that produced it. An optional polarity field distinguishes
/// excitatory (+1) from inhibitory (-1) spikes, used by the delta modulation
/// encoder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpikeEvent {
    /// Sample index at which the spike occurred.
    pub timestamp: usize,
    /// Identifier of the neuron that fired.
    pub neuron_id: usize,
    /// Polarity: +1 for excitatory (up-spike), -1 for inhibitory (down-spike).
    pub polarity: i8,
}

impl SpikeEvent {
    /// Create a new excitatory spike event.
    pub fn new(timestamp: usize, neuron_id: usize) -> Self {
        Self {
            timestamp,
            neuron_id,
            polarity: 1,
        }
    }

    /// Create a new spike event with specified polarity.
    pub fn with_polarity(timestamp: usize, neuron_id: usize, polarity: i8) -> Self {
        Self {
            timestamp,
            neuron_id,
            polarity: if polarity >= 0 { 1 } else { -1 },
        }
    }
}

// ---------------------------------------------------------------------------
// Simple deterministic PRNG (xorshift64) — no external crate dependency
// ---------------------------------------------------------------------------

/// Minimal xorshift64 PRNG for spike generation.
#[derive(Debug, Clone)]
struct Xorshift64 {
    state: u64,
}

impl Xorshift64 {
    fn new(seed: u64) -> Self {
        // Ensure non-zero state
        Self {
            state: if seed == 0 { 0x5EED_CAFE_BABE_1234 } else { seed },
        }
    }

    /// Return next pseudo-random u64.
    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    /// Return a uniform f64 in [0, 1).
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / ((1u64 << 53) as f64)
    }
}

// ---------------------------------------------------------------------------
// RateEncoder
// ---------------------------------------------------------------------------

/// Rate encoder — maps signal amplitude to spike rate (Poisson-like).
///
/// For each input sample, the probability of emitting a spike is proportional
/// to the (clamped, normalized) amplitude:
///
///   P(spike at sample n) = clamp(x[n], 0, 1) * max_rate / sample_rate
///
/// This produces an inhomogeneous Poisson-like spike train whose average rate
/// tracks the signal envelope.
///
/// # Fields
///
/// * `max_rate` — Maximum spike rate (Hz) when amplitude = 1.0.
/// * `seed` — PRNG seed for reproducibility.
#[derive(Debug, Clone)]
pub struct RateEncoder {
    max_rate: f64,
    rng: Xorshift64,
}

impl RateEncoder {
    /// Create a new rate encoder.
    ///
    /// * `max_rate` — Maximum firing rate in Hz (e.g. 100.0).
    /// * `seed` — Random seed for reproducible spike trains.
    pub fn new(max_rate: f64, seed: u64) -> Self {
        Self {
            max_rate: max_rate.abs(),
            rng: Xorshift64::new(seed),
        }
    }

    /// Encode a signal to a spike train using rate coding.
    ///
    /// The signal is interpreted with an implicit sample rate equal to
    /// `max_rate` (i.e. 1 sample = 1 / max_rate seconds). Each sample value
    /// is clamped to [0, 1] and used as the per-sample spike probability.
    ///
    /// * `signal` — Input signal samples (ideally in [0, 1]).
    /// * `neuron_id` — Neuron ID to assign to produced spikes.
    pub fn encode(&mut self, signal: &[f64], neuron_id: usize) -> Vec<SpikeEvent> {
        let mut spikes = Vec::new();
        for (i, &sample) in signal.iter().enumerate() {
            let prob = clamp01(sample);
            if self.rng.next_f64() < prob {
                spikes.push(SpikeEvent::new(i, neuron_id));
            }
        }
        spikes
    }

    /// Encode with an explicit sample rate.
    ///
    /// The per-sample spike probability becomes:
    ///   P = clamp(x, 0, 1) * max_rate / sample_rate
    ///
    /// * `signal` — Input signal samples (ideally in [0, 1]).
    /// * `sample_rate` — Sampling rate in Hz.
    /// * `neuron_id` — Neuron ID to assign to produced spikes.
    pub fn encode_with_rate(
        &mut self,
        signal: &[f64],
        sample_rate: f64,
        neuron_id: usize,
    ) -> Vec<SpikeEvent> {
        let scale = self.max_rate / sample_rate;
        let mut spikes = Vec::new();
        for (i, &sample) in signal.iter().enumerate() {
            let prob = clamp01(sample) * scale;
            if self.rng.next_f64() < prob {
                spikes.push(SpikeEvent::new(i, neuron_id));
            }
        }
        spikes
    }
}

// ---------------------------------------------------------------------------
// TemporalEncoder
// ---------------------------------------------------------------------------

/// Temporal encoder — time-to-first-spike coding.
///
/// Encodes a vector of amplitudes by assigning an earlier spike to larger
/// amplitudes. The mapping is:
///
///   t_spike(x) = t_max * (1 - clamp(x, 0, 1))
///
/// where `t_max` is the encoding window length in samples. The strongest
/// input fires first; the weakest fires last (or not at all if below
/// `threshold`).
#[derive(Debug, Clone)]
pub struct TemporalEncoder {
    /// Length of the encoding window in samples.
    window_len: usize,
    /// Minimum amplitude to produce a spike.
    threshold: f64,
}

impl TemporalEncoder {
    /// Create a new temporal encoder.
    ///
    /// * `window_len` — Size of the encoding window in samples.
    /// * `threshold` — Minimum amplitude in [0, 1] to emit a spike.
    pub fn new(window_len: usize, threshold: f64) -> Self {
        Self {
            window_len: window_len.max(1),
            threshold: clamp01(threshold),
        }
    }

    /// Encode a vector of amplitudes to spike events.
    ///
    /// Each element index becomes the `neuron_id` and the spike timestamp
    /// is derived from the amplitude rank within the encoding window.
    ///
    /// * `amplitudes` — Input amplitudes (ideally in [0, 1]).
    pub fn encode(&self, amplitudes: &[f64]) -> Vec<SpikeEvent> {
        let mut spikes = Vec::new();
        for (neuron_id, &amp) in amplitudes.iter().enumerate() {
            let a = clamp01(amp);
            if a < self.threshold {
                continue;
            }
            // Higher amplitude → earlier spike (lower timestamp)
            let t = ((1.0 - a) * (self.window_len as f64)) as usize;
            let t = t.min(self.window_len - 1);
            spikes.push(SpikeEvent::new(t, neuron_id));
        }
        spikes
    }
}

// ---------------------------------------------------------------------------
// DeltaModEncoder
// ---------------------------------------------------------------------------

/// Delta modulation spike encoder.
///
/// Emits an up-spike (+1) when the signal increases by more than `threshold`
/// since the last spike, and a down-spike (-1) when it decreases by more
/// than `threshold`. This is analogous to the send-on-delta (SOD) or
/// level-crossing sampling strategy used in event-driven systems.
///
/// The internal reference tracks the signal level: after each spike, the
/// reference moves by `threshold` in the spike direction.
#[derive(Debug, Clone)]
pub struct DeltaModEncoder {
    /// Change magnitude required to emit a spike.
    threshold: f64,
    /// Internal reference level, updated on each spike.
    reference: f64,
}

impl DeltaModEncoder {
    /// Create a new delta modulation encoder.
    ///
    /// * `threshold` — Minimum change from reference to trigger a spike.
    pub fn new(threshold: f64) -> Self {
        Self {
            threshold: threshold.abs().max(1e-15),
            reference: 0.0,
        }
    }

    /// Create with an initial reference level.
    pub fn with_reference(threshold: f64, initial_ref: f64) -> Self {
        Self {
            threshold: threshold.abs().max(1e-15),
            reference: initial_ref,
        }
    }

    /// Reset the encoder state.
    pub fn reset(&mut self) {
        self.reference = 0.0;
    }

    /// Encode a signal into a spike train with up/down polarity.
    ///
    /// * `signal` — Input signal samples.
    /// * `neuron_id` — Neuron ID to assign to produced spikes.
    pub fn encode(&mut self, signal: &[f64], neuron_id: usize) -> Vec<SpikeEvent> {
        let mut spikes = Vec::new();
        for (i, &sample) in signal.iter().enumerate() {
            let diff = sample - self.reference;
            if diff >= self.threshold {
                // Up-spike: signal rose above threshold
                let num_spikes = (diff / self.threshold) as usize;
                for _ in 0..num_spikes {
                    self.reference += self.threshold;
                    spikes.push(SpikeEvent::with_polarity(i, neuron_id, 1));
                }
            } else if diff <= -self.threshold {
                // Down-spike: signal fell below threshold
                let num_spikes = ((-diff) / self.threshold) as usize;
                for _ in 0..num_spikes {
                    self.reference -= self.threshold;
                    spikes.push(SpikeEvent::with_polarity(i, neuron_id, -1));
                }
            }
        }
        spikes
    }

    /// Return the current internal reference level.
    pub fn reference(&self) -> f64 {
        self.reference
    }
}

// ---------------------------------------------------------------------------
// LifNeuron — Leaky Integrate-and-Fire neuron model
// ---------------------------------------------------------------------------

/// Leaky Integrate-and-Fire (LIF) neuron model.
///
/// The membrane potential evolves as:
///
///   V[n+1] = V[n] * decay + I[n]
///
/// where `decay = exp(-dt / tau_m)` and `tau_m` is the membrane time
/// constant. When V exceeds `v_threshold`, a spike is emitted, V is reset
/// to `v_reset`, and the neuron enters a refractory period during which
/// input is ignored.
///
/// This is the canonical spiking neuron model used throughout computational
/// neuroscience and neuromorphic engineering.
#[derive(Debug, Clone)]
pub struct LifNeuron {
    /// Membrane time constant (in samples).
    tau_m: f64,
    /// Firing threshold voltage.
    v_threshold: f64,
    /// Reset voltage after spike.
    v_reset: f64,
    /// Resting potential (minimum voltage).
    v_rest: f64,
    /// Refractory period in samples.
    refractory_period: usize,
    /// Exponential decay factor per sample: exp(-1 / tau_m).
    decay: f64,
    /// Current membrane potential.
    membrane_potential: f64,
    /// Samples remaining in refractory period.
    refractory_counter: usize,
    /// Neuron identifier.
    neuron_id: usize,
}

impl LifNeuron {
    /// Create a new LIF neuron.
    ///
    /// * `tau_m` — Membrane time constant in samples (controls leak rate).
    /// * `v_threshold` — Voltage at which the neuron fires.
    /// * `v_reset` — Voltage to reset to after firing.
    /// * `v_rest` — Resting potential (lower bound on passive decay).
    /// * `refractory_period` — Number of samples the neuron is inactive after firing.
    /// * `neuron_id` — Unique identifier for this neuron.
    pub fn new(
        tau_m: f64,
        v_threshold: f64,
        v_reset: f64,
        v_rest: f64,
        refractory_period: usize,
        neuron_id: usize,
    ) -> Self {
        let tau_m = tau_m.max(0.1);
        let decay = (-1.0 / tau_m).exp();
        Self {
            tau_m,
            v_threshold,
            v_reset,
            v_rest,
            refractory_period,
            decay,
            membrane_potential: v_rest,
            refractory_counter: 0,
            neuron_id,
        }
    }

    /// Create a LIF neuron with default parameters.
    ///
    /// Defaults: tau_m=20, threshold=1.0, reset=0.0, rest=0.0, refractory=5.
    pub fn default_neuron(neuron_id: usize) -> Self {
        Self::new(20.0, 1.0, 0.0, 0.0, 5, neuron_id)
    }

    /// Reset the neuron to its resting state.
    pub fn reset(&mut self) {
        self.membrane_potential = self.v_rest;
        self.refractory_counter = 0;
    }

    /// Process a single input current sample and return whether the neuron fired.
    pub fn step(&mut self, input_current: f64) -> bool {
        // Refractory check
        if self.refractory_counter > 0 {
            self.refractory_counter -= 1;
            return false;
        }

        // Leak toward rest and integrate input
        self.membrane_potential =
            self.v_rest + (self.membrane_potential - self.v_rest) * self.decay + input_current;

        // Fire?
        if self.membrane_potential >= self.v_threshold {
            self.membrane_potential = self.v_reset;
            self.refractory_counter = self.refractory_period;
            true
        } else {
            false
        }
    }

    /// Process an entire input signal and return the spike train.
    pub fn encode(&mut self, signal: &[f64]) -> Vec<SpikeEvent> {
        let mut spikes = Vec::new();
        for (i, &sample) in signal.iter().enumerate() {
            if self.step(sample) {
                spikes.push(SpikeEvent::new(i, self.neuron_id));
            }
        }
        spikes
    }

    /// Return the current membrane potential.
    pub fn membrane_potential(&self) -> f64 {
        self.membrane_potential
    }

    /// Return the membrane time constant.
    pub fn tau_m(&self) -> f64 {
        self.tau_m
    }

    /// Return the firing threshold.
    pub fn v_threshold(&self) -> f64 {
        self.v_threshold
    }

    /// Return the neuron ID.
    pub fn neuron_id(&self) -> usize {
        self.neuron_id
    }
}

// ---------------------------------------------------------------------------
// SpikeDecoder — reconstruct continuous signal from spike train
// ---------------------------------------------------------------------------

/// Spike train decoder — reconstructs a continuous signal from spikes.
///
/// Each spike is convolved with an exponentially decaying kernel:
///
///   k(t) = exp(-t / tau_decay)   for t >= 0
///
/// The output at each sample is the sum of contributions from all past spikes,
/// weighted by their polarity. This is equivalent to a first-order lowpass
/// filter on the spike train.
#[derive(Debug, Clone)]
pub struct SpikeDecoder {
    /// Number of output samples.
    output_len: usize,
    /// Decay time constant in samples.
    tau_decay: f64,
}

impl SpikeDecoder {
    /// Create a new spike decoder.
    ///
    /// * `output_len` — Length of the reconstructed signal in samples.
    /// * `tau_decay` — Exponential decay time constant in samples.
    pub fn new(output_len: usize, tau_decay: f64) -> Self {
        Self {
            output_len,
            tau_decay: tau_decay.max(0.001),
        }
    }

    /// Decode a spike train into a continuous signal.
    ///
    /// * `spikes` — Spike events to decode.
    /// * `length` — Number of output samples (overrides constructor `output_len`).
    pub fn decode(&self, spikes: &[SpikeEvent], length: usize) -> Vec<f64> {
        let len = if length > 0 { length } else { self.output_len };
        let mut output = vec![0.0; len];

        for spike in spikes {
            if spike.timestamp >= len {
                continue;
            }
            let polarity = spike.polarity as f64;
            for t in spike.timestamp..len {
                let dt = (t - spike.timestamp) as f64;
                let weight = polarity * (-dt / self.tau_decay).exp();
                // Stop when contribution is negligible
                if weight.abs() < 1e-10 {
                    break;
                }
                output[t] += weight;
            }
        }
        output
    }

    /// Decode using a rectangular kernel of width `kernel_width` samples.
    ///
    /// Each spike contributes a constant polarity value for `kernel_width`
    /// samples after the spike time.
    pub fn decode_rectangular(
        &self,
        spikes: &[SpikeEvent],
        length: usize,
        kernel_width: usize,
    ) -> Vec<f64> {
        let len = if length > 0 { length } else { self.output_len };
        let mut output = vec![0.0; len];

        for spike in spikes {
            if spike.timestamp >= len {
                continue;
            }
            let polarity = spike.polarity as f64;
            let end = (spike.timestamp + kernel_width).min(len);
            for t in spike.timestamp..end {
                output[t] += polarity;
            }
        }
        output
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Compute the average spike rate in Hz.
///
/// * `spikes` — Spike events.
/// * `duration_samples` — Total duration of the observation in samples.
///
/// Returns spikes-per-sample (multiply by sample rate for Hz).
pub fn spike_rate(spikes: &[SpikeEvent], duration_samples: f64) -> f64 {
    if duration_samples <= 0.0 || spikes.is_empty() {
        return 0.0;
    }
    spikes.len() as f64 / duration_samples
}

/// Compute inter-spike intervals (in samples) for a given neuron.
///
/// Returns a vector of time differences between consecutive spikes from
/// the specified neuron, sorted in order of occurrence.
pub fn inter_spike_interval(spikes: &[SpikeEvent], neuron_id: usize) -> Vec<usize> {
    let mut timestamps: Vec<usize> = spikes
        .iter()
        .filter(|s| s.neuron_id == neuron_id)
        .map(|s| s.timestamp)
        .collect();
    timestamps.sort_unstable();

    if timestamps.len() < 2 {
        return Vec::new();
    }

    timestamps
        .windows(2)
        .map(|w| w[1] - w[0])
        .collect()
}

/// Count the total number of spikes in a spike train.
pub fn spike_count(spikes: &[SpikeEvent]) -> usize {
    spikes.len()
}

/// Count spikes for a specific neuron.
pub fn spike_count_neuron(spikes: &[SpikeEvent], neuron_id: usize) -> usize {
    spikes.iter().filter(|s| s.neuron_id == neuron_id).count()
}

/// Compute the coincidence factor between two spike trains.
///
/// The coincidence factor Gamma measures the temporal overlap between two
/// spike trains within a tolerance window `delta` (in samples):
///
///   Gamma = (N_coinc - <N_coinc>) / (0.5 * (N_1 + N_2)) * (1 / (1 - 2 * delta * rate))
///
/// where N_coinc is the number of coincident spikes, <N_coinc> is the
/// expected number by chance, and rate is the average rate of the reference
/// train. Returns a value in [-inf, 1] where 1 = perfect match.
///
/// * `train_a` — First spike train (timestamps only; neuron_id ignored).
/// * `train_b` — Second spike train.
/// * `delta` — Coincidence window width in samples.
/// * `duration` — Total observation duration in samples.
pub fn coincidence_factor(
    train_a: &[SpikeEvent],
    train_b: &[SpikeEvent],
    delta: usize,
    duration: f64,
) -> f64 {
    if train_a.is_empty() || train_b.is_empty() || duration <= 0.0 {
        return 0.0;
    }

    let mut times_a: Vec<usize> = train_a.iter().map(|s| s.timestamp).collect();
    let mut times_b: Vec<usize> = train_b.iter().map(|s| s.timestamp).collect();
    times_a.sort_unstable();
    times_b.sort_unstable();

    let n_a = times_a.len() as f64;
    let n_b = times_b.len() as f64;

    // Count coincidences: for each spike in A, check if any spike in B is within delta
    let mut n_coinc = 0usize;
    let mut j_start = 0usize;
    for &ta in &times_a {
        // Advance j_start past spikes too early
        while j_start < times_b.len() && times_b[j_start] + delta < ta {
            j_start += 1;
        }
        // Check spikes in B within range
        let mut j = j_start;
        while j < times_b.len() {
            let tb = times_b[j];
            if tb > ta + delta {
                break;
            }
            // tb is within [ta - delta, ta + delta]
            if (ta as isize - tb as isize).unsigned_abs() <= delta {
                n_coinc += 1;
                break; // count at most one coincidence per spike in A
            }
            j += 1;
        }
    }

    // Expected coincidences by chance
    let rate_b = n_b / duration;
    let expected = 2.0 * delta as f64 * rate_b * n_a;

    let denom = 0.5 * (n_a + n_b);
    if denom == 0.0 {
        return 0.0;
    }

    let normalizer = 1.0 - 2.0 * delta as f64 * rate_b;
    if normalizer.abs() < 1e-15 {
        return 0.0;
    }

    (n_coinc as f64 - expected) / denom / normalizer
}

/// Compute the mean firing rate for each neuron in a population.
///
/// Returns a vector of `(neuron_id, rate)` tuples sorted by neuron_id.
///
/// * `spikes` — Spike events from multiple neurons.
/// * `duration_samples` — Total observation duration in samples.
pub fn population_rates(spikes: &[SpikeEvent], duration_samples: f64) -> Vec<(usize, f64)> {
    if spikes.is_empty() || duration_samples <= 0.0 {
        return Vec::new();
    }

    // Gather neuron IDs and counts
    let mut counts: Vec<(usize, usize)> = Vec::new();
    for spike in spikes {
        if let Some(entry) = counts.iter_mut().find(|(id, _)| *id == spike.neuron_id) {
            entry.1 += 1;
        } else {
            counts.push((spike.neuron_id, 1));
        }
    }

    counts.sort_by_key(|&(id, _)| id);
    counts
        .into_iter()
        .map(|(id, count)| (id, count as f64 / duration_samples))
        .collect()
}

/// Compute a peri-stimulus time histogram (PSTH) for a spike train.
///
/// Bins spike timestamps into `num_bins` equal-width bins over [0, duration).
///
/// * `spikes` — Spike events.
/// * `duration` — Total observation duration in samples.
/// * `num_bins` — Number of histogram bins.
///
/// Returns a vector of spike counts per bin.
pub fn psth(spikes: &[SpikeEvent], duration: usize, num_bins: usize) -> Vec<usize> {
    if num_bins == 0 || duration == 0 {
        return Vec::new();
    }
    let bin_width = (duration as f64) / (num_bins as f64);
    let mut histogram = vec![0usize; num_bins];
    for spike in spikes {
        if spike.timestamp < duration {
            let bin = (spike.timestamp as f64 / bin_width) as usize;
            let bin = bin.min(num_bins - 1);
            histogram[bin] += 1;
        }
    }
    histogram
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Clamp a value to [0, 1].
fn clamp01(x: f64) -> f64 {
    if x < 0.0 {
        0.0
    } else if x > 1.0 {
        1.0
    } else {
        x
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // SpikeEvent tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_spike_event_new() {
        let spike = SpikeEvent::new(42, 7);
        assert_eq!(spike.timestamp, 42);
        assert_eq!(spike.neuron_id, 7);
        assert_eq!(spike.polarity, 1);
    }

    #[test]
    fn test_spike_event_polarity() {
        let up = SpikeEvent::with_polarity(10, 0, 1);
        assert_eq!(up.polarity, 1);
        let down = SpikeEvent::with_polarity(20, 1, -1);
        assert_eq!(down.polarity, -1);
        // Zero maps to +1
        let zero = SpikeEvent::with_polarity(30, 2, 0);
        assert_eq!(zero.polarity, 1);
    }

    // -----------------------------------------------------------------------
    // RateEncoder tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_rate_encoder_zero_signal_no_spikes() {
        let mut enc = RateEncoder::new(100.0, 123);
        let signal = vec![0.0; 1000];
        let spikes = enc.encode(&signal, 0);
        assert_eq!(spikes.len(), 0, "Zero signal should produce no spikes");
    }

    #[test]
    fn test_rate_encoder_max_signal_many_spikes() {
        let mut enc = RateEncoder::new(100.0, 456);
        let signal = vec![1.0; 10000];
        let spikes = enc.encode(&signal, 0);
        // With probability 1.0 per sample, we should get most samples spiking
        // (PRNG can't produce exactly 0.0 uniformly, so nearly all should spike)
        assert!(
            spikes.len() > 9000,
            "Max-amplitude signal should produce many spikes, got {}",
            spikes.len()
        );
    }

    #[test]
    fn test_rate_encoder_monotonic_rate() {
        // Higher amplitude should produce more spikes on average
        let n = 10000;
        let mut enc_low = RateEncoder::new(100.0, 789);
        let mut enc_high = RateEncoder::new(100.0, 789);
        let low_signal = vec![0.2; n];
        let high_signal = vec![0.8; n];
        let low_count = enc_low.encode(&low_signal, 0).len();
        let high_count = enc_high.encode(&high_signal, 0).len();
        assert!(
            high_count > low_count,
            "Higher amplitude should produce more spikes: low={}, high={}",
            low_count,
            high_count
        );
    }

    #[test]
    fn test_rate_encoder_reproducible() {
        let signal = vec![0.5; 500];
        let mut enc1 = RateEncoder::new(100.0, 42);
        let mut enc2 = RateEncoder::new(100.0, 42);
        let s1 = enc1.encode(&signal, 0);
        let s2 = enc2.encode(&signal, 0);
        assert_eq!(s1.len(), s2.len(), "Same seed should produce same spikes");
        for (a, b) in s1.iter().zip(s2.iter()) {
            assert_eq!(a.timestamp, b.timestamp);
        }
    }

    #[test]
    fn test_rate_encoder_with_sample_rate() {
        let mut enc = RateEncoder::new(50.0, 99);
        let signal = vec![1.0; 10000];
        let spikes = enc.encode_with_rate(&signal, 1000.0, 0);
        // Expected: 50/1000 = 0.05 probability, so ~500 spikes from 10000 samples
        let count = spikes.len();
        assert!(
            count > 300 && count < 700,
            "Expected ~500 spikes, got {}",
            count
        );
    }

    #[test]
    fn test_rate_encoder_negative_values_clamped() {
        let mut enc = RateEncoder::new(100.0, 55);
        let signal = vec![-5.0; 1000];
        let spikes = enc.encode(&signal, 0);
        assert_eq!(spikes.len(), 0, "Negative values should be clamped to 0");
    }

    // -----------------------------------------------------------------------
    // TemporalEncoder tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_temporal_encoder_ordering() {
        let enc = TemporalEncoder::new(100, 0.0);
        let amplitudes = vec![0.9, 0.1, 0.5];
        let spikes = enc.encode(&amplitudes);
        assert_eq!(spikes.len(), 3);

        // Neuron 0 (amp=0.9) should fire earliest (lowest timestamp)
        let t0 = spikes.iter().find(|s| s.neuron_id == 0).unwrap().timestamp;
        let t1 = spikes.iter().find(|s| s.neuron_id == 1).unwrap().timestamp;
        let t2 = spikes.iter().find(|s| s.neuron_id == 2).unwrap().timestamp;
        assert!(
            t0 < t2 && t2 < t1,
            "Higher amplitude should fire first: t0={}, t2={}, t1={}",
            t0,
            t2,
            t1
        );
    }

    #[test]
    fn test_temporal_encoder_threshold() {
        let enc = TemporalEncoder::new(100, 0.5);
        let amplitudes = vec![0.3, 0.6, 0.1, 0.8];
        let spikes = enc.encode(&amplitudes);
        // Only neurons with amplitude >= 0.5 should fire
        assert_eq!(spikes.len(), 2);
        let ids: Vec<usize> = spikes.iter().map(|s| s.neuron_id).collect();
        assert!(ids.contains(&1)); // 0.6
        assert!(ids.contains(&3)); // 0.8
    }

    #[test]
    fn test_temporal_encoder_max_fires_at_zero() {
        let enc = TemporalEncoder::new(100, 0.0);
        let amplitudes = vec![1.0];
        let spikes = enc.encode(&amplitudes);
        assert_eq!(spikes.len(), 1);
        assert_eq!(spikes[0].timestamp, 0, "Max amplitude should fire at time 0");
    }

    // -----------------------------------------------------------------------
    // DeltaModEncoder tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_delta_mod_constant_no_spikes() {
        let mut enc = DeltaModEncoder::new(0.1);
        let signal = vec![0.0; 100];
        let spikes = enc.encode(&signal, 0);
        assert_eq!(spikes.len(), 0, "Constant signal should produce no spikes");
    }

    #[test]
    fn test_delta_mod_ramp_up() {
        let mut enc = DeltaModEncoder::with_reference(0.1, 0.0);
        // Ramp from 0 to 1 in 100 steps
        let signal: Vec<f64> = (0..100).map(|i| i as f64 / 100.0).collect();
        let spikes = enc.encode(&signal, 0);
        // Should get ~10 up-spikes (1.0 / 0.1 = 10)
        assert!(
            spikes.len() >= 8 && spikes.len() <= 12,
            "Expected ~10 up-spikes for ramp 0→1, got {}",
            spikes.len()
        );
        // All should be up-spikes
        for spike in &spikes {
            assert_eq!(spike.polarity, 1);
        }
    }

    #[test]
    fn test_delta_mod_ramp_down() {
        let mut enc = DeltaModEncoder::with_reference(0.1, 1.0);
        let signal: Vec<f64> = (0..100).map(|i| 1.0 - i as f64 / 100.0).collect();
        let spikes = enc.encode(&signal, 0);
        // All should be down-spikes
        for spike in &spikes {
            assert_eq!(spike.polarity, -1, "Ramp down should produce down-spikes");
        }
    }

    #[test]
    fn test_delta_mod_bidirectional() {
        let mut enc = DeltaModEncoder::with_reference(0.5, 0.0);
        // Signal: 0 → 1 → 0
        let signal: Vec<f64> = (0..20)
            .map(|i| if i < 10 { i as f64 / 10.0 } else { (20 - i) as f64 / 10.0 })
            .collect();
        let spikes = enc.encode(&signal, 0);
        let up_count = spikes.iter().filter(|s| s.polarity == 1).count();
        let down_count = spikes.iter().filter(|s| s.polarity == -1).count();
        assert!(up_count > 0, "Should have up-spikes");
        assert!(down_count > 0, "Should have down-spikes");
    }

    #[test]
    fn test_delta_mod_reference_tracking() {
        let mut enc = DeltaModEncoder::new(0.25);
        let signal = vec![0.0, 0.3, 0.6, 0.9];
        let _ = enc.encode(&signal, 0);
        // Reference should have moved close to signal endpoint
        assert!(
            (enc.reference() - 0.75).abs() < 0.3,
            "Reference should track signal: got {}",
            enc.reference()
        );
    }

    // -----------------------------------------------------------------------
    // LifNeuron tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_lif_no_input_no_spikes() {
        let mut neuron = LifNeuron::default_neuron(0);
        let signal = vec![0.0; 100];
        let spikes = neuron.encode(&signal);
        assert_eq!(spikes.len(), 0, "No input should produce no spikes");
    }

    #[test]
    fn test_lif_strong_input_fires() {
        let mut neuron = LifNeuron::new(10.0, 1.0, 0.0, 0.0, 0, 0);
        let signal = vec![2.0; 10];
        let spikes = neuron.encode(&signal);
        assert!(
            !spikes.is_empty(),
            "Strong constant input should produce spikes"
        );
    }

    #[test]
    fn test_lif_refractory_period() {
        let mut neuron = LifNeuron::new(5.0, 0.5, 0.0, 0.0, 10, 0);
        let signal = vec![1.0; 50];
        let spikes = neuron.encode(&signal);
        // Check that consecutive spikes are at least refractory_period apart
        if spikes.len() >= 2 {
            for pair in spikes.windows(2) {
                let gap = pair[1].timestamp - pair[0].timestamp;
                assert!(
                    gap > 10,
                    "Spikes should respect refractory period: gap={}",
                    gap
                );
            }
        }
    }

    #[test]
    fn test_lif_membrane_decays() {
        let mut neuron = LifNeuron::new(10.0, 100.0, 0.0, 0.0, 0, 0);
        // Inject a pulse then let it decay
        neuron.step(5.0);
        let v_after_pulse = neuron.membrane_potential();
        assert!(v_after_pulse > 0.0);

        // Let it decay for several steps with zero input
        for _ in 0..50 {
            neuron.step(0.0);
        }
        let v_decayed = neuron.membrane_potential();
        assert!(
            v_decayed < v_after_pulse,
            "Membrane should decay: {} should be < {}",
            v_decayed,
            v_after_pulse
        );
        assert!(
            v_decayed.abs() < 0.1,
            "Should decay close to rest: {}",
            v_decayed
        );
    }

    #[test]
    fn test_lif_reset_after_spike() {
        let mut neuron = LifNeuron::new(5.0, 1.0, 0.0, 0.0, 0, 0);
        // Drive until spike
        let mut fired = false;
        for _ in 0..100 {
            if neuron.step(0.5) {
                fired = true;
                break;
            }
        }
        assert!(fired, "Neuron should have fired");
        assert_eq!(
            neuron.membrane_potential(),
            0.0,
            "Membrane should reset after spike"
        );
    }

    #[test]
    fn test_lif_default_neuron() {
        let neuron = LifNeuron::default_neuron(5);
        assert_eq!(neuron.neuron_id(), 5);
        assert!((neuron.tau_m() - 20.0).abs() < 1e-10);
        assert!((neuron.v_threshold() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_lif_neuron_reset() {
        let mut neuron = LifNeuron::default_neuron(0);
        neuron.step(0.5);
        assert!(neuron.membrane_potential() > 0.0);
        neuron.reset();
        assert_eq!(neuron.membrane_potential(), 0.0);
    }

    // -----------------------------------------------------------------------
    // SpikeDecoder tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_decoder_empty_spikes() {
        let decoder = SpikeDecoder::new(100, 5.0);
        let output = decoder.decode(&[], 100);
        assert_eq!(output.len(), 100);
        assert!(output.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn test_decoder_single_spike() {
        let decoder = SpikeDecoder::new(100, 10.0);
        let spikes = vec![SpikeEvent::new(10, 0)];
        let output = decoder.decode(&spikes, 100);
        // At the spike time, output should be 1.0
        assert!((output[10] - 1.0).abs() < 1e-10);
        // Should decay after the spike
        assert!(output[20] < output[10]);
        assert!(output[30] < output[20]);
        // Before the spike, output should be 0
        assert_eq!(output[5], 0.0);
    }

    #[test]
    fn test_decoder_polarity() {
        let decoder = SpikeDecoder::new(50, 5.0);
        let spikes = vec![SpikeEvent::with_polarity(10, 0, -1)];
        let output = decoder.decode(&spikes, 50);
        // Inhibitory spike should produce negative values
        assert!(output[10] < 0.0);
    }

    #[test]
    fn test_decoder_rectangular_kernel() {
        let decoder = SpikeDecoder::new(50, 5.0);
        let spikes = vec![SpikeEvent::new(5, 0)];
        let output = decoder.decode_rectangular(&spikes, 50, 10);
        // Rectangular kernel: constant 1.0 from t=5 to t=14
        for i in 5..15 {
            assert!((output[i] - 1.0).abs() < 1e-10, "t={}: {}", i, output[i]);
        }
        assert_eq!(output[15], 0.0);
        assert_eq!(output[4], 0.0);
    }

    // -----------------------------------------------------------------------
    // Helper function tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_spike_rate_basic() {
        let spikes = vec![
            SpikeEvent::new(0, 0),
            SpikeEvent::new(50, 0),
            SpikeEvent::new(99, 0),
        ];
        let rate = spike_rate(&spikes, 100.0);
        assert!((rate - 0.03).abs() < 1e-10);
    }

    #[test]
    fn test_spike_rate_empty() {
        assert_eq!(spike_rate(&[], 100.0), 0.0);
        assert_eq!(spike_rate(&[SpikeEvent::new(0, 0)], 0.0), 0.0);
    }

    #[test]
    fn test_inter_spike_interval() {
        let spikes = vec![
            SpikeEvent::new(10, 0),
            SpikeEvent::new(30, 0),
            SpikeEvent::new(35, 1), // different neuron
            SpikeEvent::new(60, 0),
        ];
        let isi = inter_spike_interval(&spikes, 0);
        assert_eq!(isi, vec![20, 30]);
    }

    #[test]
    fn test_inter_spike_interval_single_spike() {
        let spikes = vec![SpikeEvent::new(10, 0)];
        let isi = inter_spike_interval(&spikes, 0);
        assert!(isi.is_empty());
    }

    #[test]
    fn test_spike_count() {
        let spikes = vec![
            SpikeEvent::new(0, 0),
            SpikeEvent::new(1, 1),
            SpikeEvent::new(2, 0),
        ];
        assert_eq!(spike_count(&spikes), 3);
        assert_eq!(spike_count_neuron(&spikes, 0), 2);
        assert_eq!(spike_count_neuron(&spikes, 1), 1);
        assert_eq!(spike_count_neuron(&spikes, 99), 0);
    }

    #[test]
    fn test_coincidence_factor_identical() {
        let train: Vec<SpikeEvent> = (0..10).map(|i| SpikeEvent::new(i * 10, 0)).collect();
        let gamma = coincidence_factor(&train, &train, 1, 100.0);
        // Identical trains should have high coincidence
        assert!(gamma > 0.5, "Identical trains should have high gamma: {}", gamma);
    }

    #[test]
    fn test_coincidence_factor_disjoint() {
        let train_a: Vec<SpikeEvent> = (0..5).map(|i| SpikeEvent::new(i * 10, 0)).collect();
        let train_b: Vec<SpikeEvent> = (0..5).map(|i| SpikeEvent::new(i * 10 + 5, 0)).collect();
        let gamma = coincidence_factor(&train_a, &train_b, 1, 100.0);
        // Disjoint trains with small window should have low coincidence
        assert!(
            gamma < 0.5,
            "Disjoint trains should have low gamma: {}",
            gamma
        );
    }

    #[test]
    fn test_coincidence_factor_empty() {
        let train = vec![SpikeEvent::new(5, 0)];
        assert_eq!(coincidence_factor(&[], &train, 1, 100.0), 0.0);
        assert_eq!(coincidence_factor(&train, &[], 1, 100.0), 0.0);
    }

    #[test]
    fn test_population_rates() {
        let spikes = vec![
            SpikeEvent::new(0, 0),
            SpikeEvent::new(10, 1),
            SpikeEvent::new(20, 0),
            SpikeEvent::new(30, 0),
            SpikeEvent::new(40, 2),
        ];
        let rates = population_rates(&spikes, 100.0);
        assert_eq!(rates.len(), 3);
        // Neuron 0: 3 spikes / 100 = 0.03
        assert!((rates[0].1 - 0.03).abs() < 1e-10);
        // Neuron 1: 1 spike / 100 = 0.01
        assert!((rates[1].1 - 0.01).abs() < 1e-10);
        // Neuron 2: 1 spike / 100 = 0.01
        assert!((rates[2].1 - 0.01).abs() < 1e-10);
    }

    #[test]
    fn test_psth() {
        let spikes = vec![
            SpikeEvent::new(5, 0),
            SpikeEvent::new(15, 0),
            SpikeEvent::new(16, 0),
            SpikeEvent::new(25, 0),
        ];
        let hist = psth(&spikes, 30, 3);
        // Bins: [0,10), [10,20), [20,30)
        assert_eq!(hist, vec![1, 2, 1]);
    }

    #[test]
    fn test_psth_empty() {
        let hist = psth(&[], 100, 10);
        assert_eq!(hist, vec![0; 10]);
    }

    // -----------------------------------------------------------------------
    // Roundtrip encode → decode tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_rate_encode_decode_roundtrip() {
        // Encode a constant high signal, decode, verify non-zero reconstruction
        let mut enc = RateEncoder::new(100.0, 1234);
        let signal = vec![0.8; 200];
        let spikes = enc.encode(&signal, 0);
        assert!(!spikes.is_empty());

        let decoder = SpikeDecoder::new(200, 10.0);
        let recovered = decoder.decode(&spikes, 200);
        // Recovered signal should have energy (not all zeros)
        let energy: f64 = recovered.iter().map(|x| x * x).sum();
        assert!(energy > 0.0, "Recovered signal should have energy");
    }

    #[test]
    fn test_delta_encode_decode_roundtrip() {
        let mut enc = DeltaModEncoder::new(0.1);
        // Ramp up
        let signal: Vec<f64> = (0..100).map(|i| i as f64 / 100.0).collect();
        let spikes = enc.encode(&signal, 0);

        let decoder = SpikeDecoder::new(100, 20.0);
        let recovered = decoder.decode(&spikes, 100);

        // Recovered should be generally increasing
        let first_quarter: f64 = recovered[..25].iter().sum::<f64>() / 25.0;
        let last_quarter: f64 = recovered[75..].iter().sum::<f64>() / 25.0;
        assert!(
            last_quarter > first_quarter,
            "Reconstructed ramp should increase: first_q={}, last_q={}",
            first_quarter,
            last_quarter
        );
    }

    #[test]
    fn test_lif_encode_decode_roundtrip() {
        let mut neuron = LifNeuron::new(10.0, 0.5, 0.0, 0.0, 3, 0);
        // Strong periodic input
        let signal: Vec<f64> = (0..200)
            .map(|i| 0.3 + 0.3 * (i as f64 * 0.1).sin())
            .collect();
        let spikes = neuron.encode(&signal);
        assert!(!spikes.is_empty(), "LIF should fire with this input");

        let decoder = SpikeDecoder::new(200, 15.0);
        let recovered = decoder.decode(&spikes, 200);
        let energy: f64 = recovered.iter().map(|x| x * x).sum();
        assert!(energy > 0.0, "Decoded signal should have energy");
    }

    // -----------------------------------------------------------------------
    // Edge case / clamp tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_clamp01() {
        assert_eq!(clamp01(-1.0), 0.0);
        assert_eq!(clamp01(0.0), 0.0);
        assert_eq!(clamp01(0.5), 0.5);
        assert_eq!(clamp01(1.0), 1.0);
        assert_eq!(clamp01(2.0), 1.0);
    }
}

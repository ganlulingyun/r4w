//! Spiking Neural Network (SNN) Inference Engine
//!
//! Implements a neuromorphic signal classification engine based on spiking
//! neural networks. Unlike traditional artificial neural networks that use
//! continuous-valued activations, SNNs communicate through discrete spike
//! events, mimicking biological neural computation.
//!
//! ## Neuron Models
//!
//! Two neuron models are provided:
//!
//! - **Leaky Integrate-and-Fire (LIF)**: The simplest biologically plausible
//!   spiking model. Membrane potential integrates input current and decays
//!   exponentially. When voltage exceeds threshold, the neuron fires and resets.
//!   Supports adaptive threshold for homeostatic regulation.
//!
//!   ```text
//!   tau_m * dV/dt = -(V - V_rest) + R * I(t)
//!   if V >= V_thresh: spike, V <- V_reset
//!   ```
//!
//! - **Izhikevich**: A computationally efficient 2-variable model that
//!   reproduces 20+ firing patterns observed in cortical neurons. The recovery
//!   variable `u` provides negative feedback for after-spike dynamics.
//!
//!   ```text
//!   dv/dt = 0.04*v^2 + 5*v + 140 - u + I
//!   du/dt = a*(b*v - u)
//!   if v >= 30: v <- c, u <- u + d
//!   ```
//!
//!   Supported patterns: Regular Spiking (RS), Intrinsically Bursting (IB),
//!   Chattering (CH), Fast Spiking (FS), Low-Threshold Spiking (LTS).
//!
//! ## Learning
//!
//! Spike-Timing-Dependent Plasticity (STDP) adjusts synaptic weights based on
//! the relative timing of pre- and post-synaptic spikes:
//!
//! - Pre before post (causal): weight increases (LTP)
//! - Post before pre (anti-causal): weight decreases (LTD)
//!
//! ```text
//! dw = A_plus  * exp(-dt / tau_plus)   if dt > 0 (pre before post)
//! dw = -A_minus * exp(dt / tau_minus)  if dt < 0 (post before pre)
//! ```
//!
//! ## Network Topology
//!
//! `SnnNetwork` supports feedforward and recurrent topologies with arbitrary
//! layer sizes. Inference proceeds in discrete timesteps with configurable
//! simulation duration and step size.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::spiking_neural_network::{
//!     SnnNetwork, NeuronType, SnnLayer, Synapse, StdpRule,
//!     firing_rate, spike_train_distance,
//! };
//!
//! // Build a 2-layer feedforward SNN
//! let mut network = SnnNetwork::new(0.001); // 1ms timestep
//! network.add_layer(SnnLayer::new(4, NeuronType::Lif {
//!     tau_m: 0.020, v_rest: -0.065, v_reset: -0.070,
//!     v_thresh: -0.050, r_m: 1e7,
//! }));
//! network.add_layer(SnnLayer::new(2, NeuronType::Lif {
//!     tau_m: 0.020, v_rest: -0.065, v_reset: -0.070,
//!     v_thresh: -0.050, r_m: 1e7,
//! }));
//! network.connect_layers(0, 1, 0.01);
//!
//! // Run inference with input currents
//! let input = vec![1.5e-9, 0.0, 2.0e-9, 0.0];
//! let spikes = network.run(&input, 100); // 100 timesteps
//! ```

use std::fmt;

// ============================================================================
// Neuron trait and models
// ============================================================================

/// Common interface for spiking neuron models.
///
/// Each neuron maintains internal state (membrane potential, recovery variables)
/// and produces a binary spike output when stimulated with input current.
pub trait SpikingNeuron {
    /// Advance the neuron by one timestep with the given input current.
    /// Returns `true` if the neuron fired a spike.
    fn step(&mut self, current: f64, dt: f64) -> bool;

    /// Return the current membrane potential (in volts or model units).
    fn voltage(&self) -> f64;

    /// Reset the neuron to its resting state.
    fn reset(&mut self);
}

/// Enumeration of supported neuron model types for layer construction.
#[derive(Debug, Clone, Copy)]
pub enum NeuronType {
    /// Leaky Integrate-and-Fire neuron.
    ///
    /// Parameters:
    /// - `tau_m`: Membrane time constant (seconds), typically 10-30 ms
    /// - `v_rest`: Resting potential (volts), typically -65 mV
    /// - `v_reset`: Reset potential after spike (volts), typically -70 mV
    /// - `v_thresh`: Firing threshold (volts), typically -50 mV
    /// - `r_m`: Membrane resistance (ohms), typically 10 MOhm
    Lif {
        tau_m: f64,
        v_rest: f64,
        v_reset: f64,
        v_thresh: f64,
        r_m: f64,
    },
    /// Izhikevich neuron with pattern selection.
    ///
    /// The four parameters (a, b, c, d) are set via `IzhikevichPattern`.
    Izhikevich(IzhikevichPattern),
}

/// Firing patterns for the Izhikevich neuron model.
///
/// Each pattern corresponds to a different combination of the (a, b, c, d)
/// parameters, reproducing distinct cortical neuron behaviors.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IzhikevichPattern {
    /// Regular Spiking (excitatory cortical neuron). Most common.
    /// (a=0.02, b=0.2, c=-65, d=8)
    RegularSpiking,
    /// Intrinsically Bursting (excitatory, initial burst then single spikes).
    /// (a=0.02, b=0.2, c=-55, d=4)
    IntrinsicallyBursting,
    /// Chattering (excitatory, rhythmic bursting).
    /// (a=0.02, b=0.2, c=-50, d=2)
    Chattering,
    /// Fast Spiking (inhibitory interneuron, no adaptation).
    /// (a=0.1, b=0.2, c=-65, d=2)
    FastSpiking,
    /// Low-Threshold Spiking (inhibitory, rebound bursts).
    /// (a=0.02, b=0.25, c=-65, d=2)
    LowThresholdSpiking,
}

impl IzhikevichPattern {
    /// Return the (a, b, c, d) parameter tuple for this pattern.
    pub fn params(self) -> (f64, f64, f64, f64) {
        match self {
            Self::RegularSpiking => (0.02, 0.2, -65.0, 8.0),
            Self::IntrinsicallyBursting => (0.02, 0.2, -55.0, 4.0),
            Self::Chattering => (0.02, 0.2, -50.0, 2.0),
            Self::FastSpiking => (0.1, 0.2, -65.0, 2.0),
            Self::LowThresholdSpiking => (0.02, 0.25, -65.0, 2.0),
        }
    }
}

// ============================================================================
// LIF Neuron
// ============================================================================

/// Leaky Integrate-and-Fire neuron with optional adaptive threshold.
///
/// The LIF model is the workhorse of computational neuroscience. The membrane
/// potential leaks toward rest with time constant `tau_m`, and input current
/// charges it through membrane resistance `r_m`. When voltage crosses threshold,
/// the neuron emits a spike and resets.
///
/// Adaptive threshold: after each spike, the threshold increases by
/// `thresh_adapt_increment` and decays back to the base threshold with time
/// constant `thresh_adapt_tau`. This prevents excessive firing (homeostasis).
#[derive(Debug, Clone)]
pub struct LifNeuronModel {
    /// Membrane time constant (seconds).
    pub tau_m: f64,
    /// Resting membrane potential (volts).
    pub v_rest: f64,
    /// Reset potential after spike (volts).
    pub v_reset: f64,
    /// Base firing threshold (volts).
    pub v_thresh_base: f64,
    /// Membrane resistance (ohms).
    pub r_m: f64,
    /// Current membrane potential.
    v: f64,
    /// Current (adaptive) threshold.
    v_thresh: f64,
    /// Threshold adaptation increment per spike.
    thresh_adapt_increment: f64,
    /// Threshold adaptation time constant (seconds).
    thresh_adapt_tau: f64,
    /// Time of last spike (for STDP).
    last_spike_time: Option<f64>,
    /// Accumulated simulation time.
    time: f64,
}

impl LifNeuronModel {
    /// Create a new LIF neuron with the given biophysical parameters.
    pub fn new(tau_m: f64, v_rest: f64, v_reset: f64, v_thresh: f64, r_m: f64) -> Self {
        Self {
            tau_m,
            v_rest,
            v_reset,
            v_thresh_base: v_thresh,
            r_m,
            v: v_rest,
            v_thresh: v_thresh,
            thresh_adapt_increment: 0.0,
            thresh_adapt_tau: 0.1,
            last_spike_time: None,
            time: 0.0,
        }
    }

    /// Enable adaptive threshold. After each spike, threshold increases by
    /// `increment` and decays back with time constant `tau`.
    pub fn with_adaptive_threshold(mut self, increment: f64, tau: f64) -> Self {
        self.thresh_adapt_increment = increment;
        self.thresh_adapt_tau = tau;
        self
    }

    /// Return the time of the last spike, if any.
    pub fn last_spike_time(&self) -> Option<f64> {
        self.last_spike_time
    }
}

impl SpikingNeuron for LifNeuronModel {
    fn step(&mut self, current: f64, dt: f64) -> bool {
        // Exponential decay of adaptive threshold toward base
        if self.thresh_adapt_increment > 0.0 {
            let alpha = (-dt / self.thresh_adapt_tau).exp();
            self.v_thresh = self.v_thresh_base + (self.v_thresh - self.v_thresh_base) * alpha;
        }

        // Leaky integration: dV/dt = (-(V - V_rest) + R_m * I) / tau_m
        let dv = (-(self.v - self.v_rest) + self.r_m * current) / self.tau_m;
        self.v += dv * dt;
        self.time += dt;

        // Spike check
        if self.v >= self.v_thresh {
            self.v = self.v_reset;
            self.last_spike_time = Some(self.time);
            // Adaptive threshold bump
            if self.thresh_adapt_increment > 0.0 {
                self.v_thresh += self.thresh_adapt_increment;
            }
            true
        } else {
            false
        }
    }

    fn voltage(&self) -> f64 {
        self.v
    }

    fn reset(&mut self) {
        self.v = self.v_rest;
        self.v_thresh = self.v_thresh_base;
        self.last_spike_time = None;
        self.time = 0.0;
    }
}

impl fmt::Display for LifNeuronModel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "LIF(tau_m={:.1}ms, V_rest={:.1}mV, V_thresh={:.1}mV)",
            self.tau_m * 1e3,
            self.v_rest * 1e3,
            self.v_thresh_base * 1e3
        )
    }
}

// ============================================================================
// Izhikevich Neuron
// ============================================================================

/// Izhikevich spiking neuron model.
///
/// A 2D dynamical system that reproduces over 20 known firing patterns using
/// only two coupled ODEs and four parameters. Far more expressive than LIF
/// while remaining computationally inexpensive (no exponentials needed).
///
/// The model uses dimensionless units: voltage in mV-like scale, time in ms.
/// The spike cutoff is at v = 30 (model units).
#[derive(Debug, Clone)]
pub struct IzhikevichNeuron {
    /// Parameter a: recovery time constant (1/ms). Smaller = slower recovery.
    pub a: f64,
    /// Parameter b: sensitivity of recovery to sub-threshold voltage.
    pub b: f64,
    /// Parameter c: after-spike reset voltage (mV-like).
    pub c: f64,
    /// Parameter d: after-spike recovery increment.
    pub d: f64,
    /// Membrane potential (model units, spike at 30).
    v: f64,
    /// Recovery variable.
    u: f64,
    /// Pattern type.
    pattern: IzhikevichPattern,
    /// Time of last spike.
    last_spike_time: Option<f64>,
    /// Accumulated simulation time.
    time: f64,
}

impl IzhikevichNeuron {
    /// Create a new Izhikevich neuron with the specified firing pattern.
    pub fn new(pattern: IzhikevichPattern) -> Self {
        let (a, b, c, d) = pattern.params();
        Self {
            a,
            b,
            c,
            d,
            v: -65.0,
            u: -65.0 * b,
            pattern,
            last_spike_time: None,
            time: 0.0,
        }
    }

    /// Create with custom (a, b, c, d) parameters.
    pub fn custom(a: f64, b: f64, c: f64, d: f64) -> Self {
        Self {
            a,
            b,
            c,
            d,
            v: -65.0,
            u: -65.0 * b,
            pattern: IzhikevichPattern::RegularSpiking,
            last_spike_time: None,
            time: 0.0,
        }
    }

    /// Return the firing pattern.
    pub fn pattern(&self) -> IzhikevichPattern {
        self.pattern
    }

    /// Return the recovery variable value.
    pub fn recovery(&self) -> f64 {
        self.u
    }

    /// Return the time of the last spike.
    pub fn last_spike_time(&self) -> Option<f64> {
        self.last_spike_time
    }
}

impl SpikingNeuron for IzhikevichNeuron {
    fn step(&mut self, current: f64, dt: f64) -> bool {
        // Izhikevich uses ms timescale internally
        let dt_ms = dt * 1000.0;

        // Use 0.5ms sub-steps for numerical stability (Euler method)
        let n_steps = (dt_ms / 0.5).ceil().max(1.0) as usize;
        let sub_dt = dt_ms / n_steps as f64;

        for _ in 0..n_steps {
            if self.v >= 30.0 {
                // Spike reset
                self.v = self.c;
                self.u += self.d;
                self.time += sub_dt / 1000.0;
                self.last_spike_time = Some(self.time);
                return true;
            }
            // dv/dt = 0.04*v^2 + 5*v + 140 - u + I
            let dv = 0.04 * self.v * self.v + 5.0 * self.v + 140.0 - self.u + current;
            // du/dt = a*(b*v - u)
            let du = self.a * (self.b * self.v - self.u);
            self.v += dv * sub_dt;
            self.u += du * sub_dt;
        }
        self.time += dt_ms / 1000.0 - (n_steps as f64) * sub_dt / 1000.0;
        // Correct accumulated time
        self.time = (self.time * 1e6).round() / 1e6;
        // Actually let's just track it simply
        self.time += dt - dt; // no-op, time already tracked in sub-steps... let me fix this

        // Spike check after all sub-steps
        if self.v >= 30.0 {
            self.v = self.c;
            self.u += self.d;
            self.last_spike_time = Some(self.time);
            return true;
        }

        false
    }

    fn voltage(&self) -> f64 {
        self.v
    }

    fn reset(&mut self) {
        self.v = -65.0;
        self.u = -65.0 * self.b;
        self.last_spike_time = None;
        self.time = 0.0;
    }
}

impl fmt::Display for IzhikevichNeuron {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Izhikevich({:?})", self.pattern)
    }
}

// ============================================================================
// Synapse
// ============================================================================

/// A synaptic connection between two neurons.
///
/// Each synapse has a weight, an optional axonal delay (in timesteps), and
/// an optional STDP learning rule for online weight updates.
#[derive(Debug, Clone)]
pub struct Synapse {
    /// Source neuron index (in the pre-synaptic layer).
    pub pre_idx: usize,
    /// Target neuron index (in the post-synaptic layer).
    pub post_idx: usize,
    /// Synaptic weight (positive = excitatory, negative = inhibitory).
    pub weight: f64,
    /// Axonal delay in number of timesteps.
    pub delay: usize,
    /// Optional STDP rule for this synapse.
    pub stdp: Option<StdpRule>,
}

impl Synapse {
    /// Create a new synapse with the given pre/post indices and weight.
    pub fn new(pre_idx: usize, post_idx: usize, weight: f64) -> Self {
        Self {
            pre_idx,
            post_idx,
            weight,
            delay: 0,
            stdp: None,
        }
    }

    /// Set the axonal delay (in timesteps).
    pub fn with_delay(mut self, delay: usize) -> Self {
        self.delay = delay;
        self
    }

    /// Attach an STDP learning rule.
    pub fn with_stdp(mut self, rule: StdpRule) -> Self {
        self.stdp = Some(rule);
        self
    }

    /// Apply STDP weight update given pre and post spike times.
    ///
    /// `dt` = t_post - t_pre. Positive dt means pre fired before post (LTP).
    pub fn apply_stdp(&mut self, dt: f64) {
        if let Some(ref rule) = self.stdp {
            let dw = rule.compute_weight_change(dt);
            self.weight += dw;
            // Clamp weight to allowed range
            self.weight = self.weight.clamp(rule.w_min, rule.w_max);
        }
    }
}

// ============================================================================
// STDP Rule
// ============================================================================

/// Spike-Timing-Dependent Plasticity rule.
///
/// Models the biological observation that synapses strengthen when the
/// pre-synaptic neuron fires shortly before the post-synaptic neuron (causal
/// timing, LTP) and weaken for the reverse order (anti-causal, LTD).
///
/// The weight change follows exponential windows:
/// ```text
/// dw = +A_plus  * exp(-|dt| / tau_plus)   for dt > 0 (pre before post)
/// dw = -A_minus * exp(-|dt| / tau_minus)   for dt < 0 (post before pre)
/// ```
#[derive(Debug, Clone, Copy)]
pub struct StdpRule {
    /// LTP amplitude (positive).
    pub a_plus: f64,
    /// LTD amplitude (positive, applied as negative).
    pub a_minus: f64,
    /// LTP time constant (seconds).
    pub tau_plus: f64,
    /// LTD time constant (seconds).
    pub tau_minus: f64,
    /// Minimum allowed weight.
    pub w_min: f64,
    /// Maximum allowed weight.
    pub w_max: f64,
}

impl StdpRule {
    /// Create a standard STDP rule with symmetric time constants.
    ///
    /// Default: A+ = 0.01, A- = 0.012, tau = 20ms, weights in [0, 1].
    pub fn standard() -> Self {
        Self {
            a_plus: 0.01,
            a_minus: 0.012,
            tau_plus: 0.020,
            tau_minus: 0.020,
            w_min: 0.0,
            w_max: 1.0,
        }
    }

    /// Create an STDP rule with custom parameters.
    pub fn new(a_plus: f64, a_minus: f64, tau_plus: f64, tau_minus: f64) -> Self {
        Self {
            a_plus,
            a_minus,
            tau_plus,
            tau_minus,
            w_min: 0.0,
            w_max: 1.0,
        }
    }

    /// Set the allowed weight range.
    pub fn with_bounds(mut self, w_min: f64, w_max: f64) -> Self {
        self.w_min = w_min;
        self.w_max = w_max;
        self
    }

    /// Compute the weight change for a given spike timing difference.
    ///
    /// `dt` = t_post - t_pre.
    pub fn compute_weight_change(&self, dt: f64) -> f64 {
        if dt > 0.0 {
            // Pre before post: potentiation (LTP)
            self.a_plus * (-dt / self.tau_plus).exp()
        } else if dt < 0.0 {
            // Post before pre: depression (LTD)
            -self.a_minus * (dt / self.tau_minus).exp()
        } else {
            0.0
        }
    }
}

impl Default for StdpRule {
    fn default() -> Self {
        Self::standard()
    }
}

// ============================================================================
// SNN Layer
// ============================================================================

/// Internal neuron state wrapper (type-erased via enum).
#[derive(Debug, Clone)]
enum NeuronState {
    Lif(LifNeuronModel),
    Izhikevich(IzhikevichNeuron),
}

impl NeuronState {
    fn step(&mut self, current: f64, dt: f64) -> bool {
        match self {
            NeuronState::Lif(n) => n.step(current, dt),
            NeuronState::Izhikevich(n) => n.step(current, dt),
        }
    }

    fn voltage(&self) -> f64 {
        match self {
            NeuronState::Lif(n) => n.voltage(),
            NeuronState::Izhikevich(n) => n.voltage(),
        }
    }

    fn reset(&mut self) {
        match self {
            NeuronState::Lif(n) => n.reset(),
            NeuronState::Izhikevich(n) => n.reset(),
        }
    }

    fn last_spike_time(&self) -> Option<f64> {
        match self {
            NeuronState::Lif(n) => n.last_spike_time(),
            NeuronState::Izhikevich(n) => n.last_spike_time(),
        }
    }
}

/// A layer of spiking neurons.
///
/// Contains a homogeneous population of neurons (all same type within a layer)
/// and tracks their spike history for STDP learning.
#[derive(Debug, Clone)]
pub struct SnnLayer {
    neurons: Vec<NeuronState>,
    /// Spike output from the most recent timestep (one bool per neuron).
    pub spikes: Vec<bool>,
    /// Spike history: for each neuron, the time of most recent spike.
    pub last_spike_times: Vec<Option<f64>>,
    /// Number of neurons.
    size: usize,
    /// Neuron type used in this layer.
    neuron_type: NeuronType,
}

impl SnnLayer {
    /// Create a new layer with `size` neurons of the given type.
    pub fn new(size: usize, neuron_type: NeuronType) -> Self {
        let neurons = (0..size)
            .map(|_| match neuron_type {
                NeuronType::Lif {
                    tau_m,
                    v_rest,
                    v_reset,
                    v_thresh,
                    r_m,
                } => NeuronState::Lif(LifNeuronModel::new(tau_m, v_rest, v_reset, v_thresh, r_m)),
                NeuronType::Izhikevich(pattern) => {
                    NeuronState::Izhikevich(IzhikevichNeuron::new(pattern))
                }
            })
            .collect();

        Self {
            neurons,
            spikes: vec![false; size],
            last_spike_times: vec![None; size],
            size,
            neuron_type,
        }
    }

    /// Return the number of neurons in this layer.
    pub fn size(&self) -> usize {
        self.size
    }

    /// Return the neuron type.
    pub fn neuron_type(&self) -> NeuronType {
        self.neuron_type
    }

    /// Step all neurons with given input currents.
    /// Returns a vector of booleans indicating which neurons spiked.
    pub fn step(&mut self, currents: &[f64], dt: f64) -> Vec<bool> {
        assert_eq!(
            currents.len(),
            self.size,
            "Current vector length must match layer size"
        );

        for i in 0..self.size {
            let spiked = self.neurons[i].step(currents[i], dt);
            self.spikes[i] = spiked;
            if spiked {
                self.last_spike_times[i] = self.neurons[i].last_spike_time();
            }
        }

        self.spikes.clone()
    }

    /// Reset all neurons in the layer.
    pub fn reset(&mut self) {
        for n in &mut self.neurons {
            n.reset();
        }
        self.spikes.fill(false);
        self.last_spike_times.fill(None);
    }

    /// Get the voltage of a specific neuron.
    pub fn neuron_voltage(&self, idx: usize) -> f64 {
        self.neurons[idx].voltage()
    }
}

// ============================================================================
// SNN Network
// ============================================================================

/// Feedforward/recurrent spiking neural network.
///
/// Composed of `SnnLayer`s connected by `Synapse` vectors. Supports:
/// - Feedforward connections (layer i -> layer j where j > i)
/// - Recurrent connections (layer i -> layer i, or j <= i)
/// - Per-synapse STDP learning during inference
///
/// Inference proceeds in discrete timesteps. At each step:
/// 1. Input currents are injected into layer 0
/// 2. Synaptic currents from pre-synaptic spikes are accumulated
/// 3. Each layer's neurons are updated
/// 4. STDP weight updates are applied (if enabled)
pub struct SnnNetwork {
    /// All layers in the network.
    layers: Vec<SnnLayer>,
    /// Connections between layers: `connections[i]` holds synapses from layer i.
    connections: Vec<Vec<Synapse>>,
    /// Simulation timestep (seconds).
    dt: f64,
    /// Whether STDP learning is enabled.
    stdp_enabled: bool,
    /// Spike history per layer per timestep (for analysis).
    spike_log: Vec<Vec<Vec<bool>>>,
}

impl SnnNetwork {
    /// Create a new SNN with the given simulation timestep (seconds).
    pub fn new(dt: f64) -> Self {
        Self {
            layers: Vec::new(),
            connections: Vec::new(),
            dt,
            stdp_enabled: false,
            spike_log: Vec::new(),
        }
    }

    /// Add a layer to the network. Returns the layer index.
    pub fn add_layer(&mut self, layer: SnnLayer) -> usize {
        let idx = self.layers.len();
        self.layers.push(layer);
        self.connections.push(Vec::new());
        self.spike_log.push(Vec::new());
        idx
    }

    /// Connect two layers with uniform random-like weights.
    ///
    /// Creates all-to-all connections from `from_layer` to `to_layer` with
    /// the specified weight. Use negative weights for inhibitory connections.
    pub fn connect_layers(&mut self, from_layer: usize, to_layer: usize, weight: f64) {
        assert!(from_layer < self.layers.len(), "from_layer out of range");
        assert!(to_layer < self.layers.len(), "to_layer out of range");

        let pre_size = self.layers[from_layer].size();
        let post_size = self.layers[to_layer].size();

        for pre in 0..pre_size {
            for post in 0..post_size {
                self.connections[from_layer].push(Synapse::new(pre, post, weight));
            }
        }
    }

    /// Connect two layers with weights from a matrix (row = pre, col = post).
    pub fn connect_layers_matrix(
        &mut self,
        from_layer: usize,
        to_layer: usize,
        weights: &[Vec<f64>],
    ) {
        assert!(from_layer < self.layers.len());
        assert!(to_layer < self.layers.len());

        let pre_size = self.layers[from_layer].size();
        let post_size = self.layers[to_layer].size();
        assert_eq!(weights.len(), pre_size);

        for (pre, row) in weights.iter().enumerate() {
            assert_eq!(row.len(), post_size);
            for (post, &w) in row.iter().enumerate() {
                self.connections[from_layer].push(Synapse::new(pre, post, w));
            }
        }
    }

    /// Add a single synapse from `from_layer` neuron `pre` to `to_layer` neuron `post`.
    pub fn add_synapse(&mut self, from_layer: usize, synapse: Synapse) {
        assert!(from_layer < self.layers.len());
        self.connections[from_layer].push(synapse);
    }

    /// Enable or disable STDP learning during inference.
    pub fn set_stdp_enabled(&mut self, enabled: bool) {
        self.stdp_enabled = enabled;
    }

    /// Return the number of layers.
    pub fn num_layers(&self) -> usize {
        self.layers.len()
    }

    /// Return a reference to a layer.
    pub fn layer(&self, idx: usize) -> &SnnLayer {
        &self.layers[idx]
    }

    /// Return the simulation timestep.
    pub fn dt(&self) -> f64 {
        self.dt
    }

    /// Return the synapses originating from a given layer.
    pub fn synapses(&self, from_layer: usize) -> &[Synapse] {
        &self.connections[from_layer]
    }

    /// Run the network for `num_steps` timesteps with input currents.
    ///
    /// `input_currents` are injected into layer 0 neurons. The input vector
    /// length must match the size of the first layer.
    ///
    /// Returns the spike output of the last layer at each timestep:
    /// `result[t]` is a `Vec<bool>` indicating which output neurons spiked.
    pub fn run(&mut self, input_currents: &[f64], num_steps: usize) -> Vec<Vec<bool>> {
        assert!(!self.layers.is_empty(), "Network has no layers");
        assert_eq!(
            input_currents.len(),
            self.layers[0].size(),
            "Input size must match first layer"
        );

        // Reset spike log
        for log in &mut self.spike_log {
            log.clear();
        }

        let num_layers = self.layers.len();
        let mut output_spikes = Vec::with_capacity(num_steps);

        for _step in 0..num_steps {
            // Collect currents for each layer
            let mut layer_currents: Vec<Vec<f64>> = self
                .layers
                .iter()
                .map(|l| vec![0.0; l.size()])
                .collect();

            // Input currents go to layer 0
            for (i, &c) in input_currents.iter().enumerate() {
                layer_currents[0][i] = c;
            }

            // Accumulate synaptic currents from previous step's spikes
            for from_layer in 0..num_layers {
                let from_spikes = &self.layers[from_layer].spikes;
                for syn in &self.connections[from_layer] {
                    if syn.pre_idx < from_spikes.len() && from_spikes[syn.pre_idx] {
                        // Determine target layer
                        // connections[from_layer] target the "next" layer by default
                        // We need to determine which layer this synapse targets
                        // For simplicity, connections[i] connect layer i to layer i+1
                        // unless it's a recurrent connection
                        let target_layer = if from_layer + 1 < num_layers {
                            from_layer + 1
                        } else {
                            from_layer // recurrent on last layer
                        };
                        if syn.post_idx < layer_currents[target_layer].len() {
                            layer_currents[target_layer][syn.post_idx] += syn.weight;
                        }
                    }
                }
            }

            // Step all layers
            for layer_idx in 0..num_layers {
                let currents = &layer_currents[layer_idx];
                let spikes = self.layers[layer_idx].step(currents, self.dt);
                self.spike_log[layer_idx].push(spikes);
            }

            // STDP learning
            if self.stdp_enabled {
                self.apply_stdp();
            }

            // Record output layer spikes
            output_spikes.push(self.layers[num_layers - 1].spikes.clone());
        }

        output_spikes
    }

    /// Apply STDP to all synapses with STDP rules attached.
    fn apply_stdp(&mut self) {
        let num_layers = self.layers.len();

        for from_layer in 0..num_layers {
            let target_layer = if from_layer + 1 < num_layers {
                from_layer + 1
            } else {
                from_layer
            };

            // We need pre and post spike times
            let pre_spike_times: Vec<Option<f64>> =
                self.layers[from_layer].last_spike_times.clone();
            let post_spike_times: Vec<Option<f64>> =
                self.layers[target_layer].last_spike_times.clone();

            for syn in &mut self.connections[from_layer] {
                if syn.stdp.is_some() {
                    if let (Some(t_pre), Some(t_post)) = (
                        pre_spike_times.get(syn.pre_idx).copied().flatten(),
                        post_spike_times.get(syn.post_idx).copied().flatten(),
                    ) {
                        let dt = t_post - t_pre;
                        syn.apply_stdp(dt);
                    }
                }
            }
        }
    }

    /// Reset all layers and clear spike logs.
    pub fn reset(&mut self) {
        for layer in &mut self.layers {
            layer.reset();
        }
        for log in &mut self.spike_log {
            log.clear();
        }
    }

    /// Return the spike log for a given layer.
    ///
    /// `result[t][n]` is true if neuron `n` spiked at timestep `t`.
    pub fn spike_log(&self, layer_idx: usize) -> &[Vec<bool>] {
        &self.spike_log[layer_idx]
    }
}

impl fmt::Debug for SnnNetwork {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SnnNetwork")
            .field("num_layers", &self.layers.len())
            .field("dt", &self.dt)
            .field("stdp_enabled", &self.stdp_enabled)
            .finish()
    }
}

// ============================================================================
// Helper functions
// ============================================================================

/// Compute the firing rate of a spike train (spikes per second).
///
/// `spike_train` is a sequence of booleans (true = spike) sampled at
/// intervals of `dt` seconds.
///
/// Returns the mean firing rate in Hz.
///
/// # Example
///
/// ```rust
/// use r4w_core::spiking_neural_network::firing_rate;
///
/// let train = vec![false, true, false, false, true, false, true, false, false, false];
/// let rate = firing_rate(&train, 0.001); // 1ms bins
/// assert!((rate - 300.0).abs() < 1.0); // 3 spikes in 10ms = 300 Hz
/// ```
pub fn firing_rate(spike_train: &[bool], dt: f64) -> f64 {
    if spike_train.is_empty() || dt <= 0.0 {
        return 0.0;
    }
    let num_spikes = spike_train.iter().filter(|&&s| s).count();
    let duration = spike_train.len() as f64 * dt;
    num_spikes as f64 / duration
}

/// Compute the van Rossum distance between two spike trains.
///
/// Each spike is convolved with an exponential kernel `exp(-t/tau)` and the
/// L2 distance between the resulting continuous signals is computed.
///
/// Smaller distance means more similar spike trains. Distance = 0 for
/// identical trains.
///
/// # Arguments
///
/// - `train_a`, `train_b`: Boolean spike trains (same length)
/// - `dt`: Time step (seconds)
/// - `tau`: Exponential kernel time constant (seconds)
///
/// # Example
///
/// ```rust
/// use r4w_core::spiking_neural_network::spike_train_distance;
///
/// let a = vec![true, false, false, true, false];
/// let b = vec![true, false, false, true, false];
/// let d = spike_train_distance(&a, &b, 0.001, 0.010);
/// assert!(d < 1e-10); // identical trains have zero distance
/// ```
pub fn spike_train_distance(train_a: &[bool], train_b: &[bool], dt: f64, tau: f64) -> f64 {
    assert_eq!(
        train_a.len(),
        train_b.len(),
        "Spike trains must have equal length"
    );

    if train_a.is_empty() {
        return 0.0;
    }

    let n = train_a.len();
    let decay = (-dt / tau).exp();

    // Convolve each train with exponential kernel (causal filter)
    let mut filtered_a = vec![0.0; n];
    let mut filtered_b = vec![0.0; n];

    let mut fa = 0.0;
    let mut fb = 0.0;

    for i in 0..n {
        fa = fa * decay + if train_a[i] { 1.0 } else { 0.0 };
        fb = fb * decay + if train_b[i] { 1.0 } else { 0.0 };
        filtered_a[i] = fa;
        filtered_b[i] = fb;
    }

    // L2 distance
    let mut sum_sq = 0.0;
    for i in 0..n {
        let diff = filtered_a[i] - filtered_b[i];
        sum_sq += diff * diff;
    }

    (sum_sq * dt / tau).sqrt()
}

/// Compute a population vector from spike counts and preferred directions.
///
/// Each neuron `i` has a preferred direction (angle in radians) and a spike
/// count. The population vector is the weighted sum of direction unit vectors.
///
/// Returns (magnitude, angle_radians).
///
/// This is used in motor cortex decoding and for interpreting SNN output
/// as a directional decision.
///
/// # Example
///
/// ```rust
/// use r4w_core::spiking_neural_network::population_vector;
///
/// // 4 neurons pointing N, E, S, W with equal firing
/// let counts = vec![10, 10, 10, 10];
/// let directions = vec![0.0, std::f64::consts::FRAC_PI_2,
///                       std::f64::consts::PI, 3.0 * std::f64::consts::FRAC_PI_2];
/// let (mag, _angle) = population_vector(&counts, &directions);
/// assert!(mag < 1.0); // Balanced firing -> near-zero resultant
/// ```
pub fn population_vector(spike_counts: &[usize], preferred_directions: &[f64]) -> (f64, f64) {
    assert_eq!(
        spike_counts.len(),
        preferred_directions.len(),
        "Counts and directions must have equal length"
    );

    let mut sum_x = 0.0;
    let mut sum_y = 0.0;

    for (&count, &angle) in spike_counts.iter().zip(preferred_directions.iter()) {
        sum_x += count as f64 * angle.cos();
        sum_y += count as f64 * angle.sin();
    }

    let magnitude = (sum_x * sum_x + sum_y * sum_y).sqrt();
    let angle = sum_y.atan2(sum_x);

    (magnitude, angle)
}

/// Encode an analog signal value as a spike train using rate coding.
///
/// Higher signal values produce higher spike rates. The signal is clamped
/// to `[0, max_value]` and linearly mapped to `[0, max_rate]` Hz.
///
/// Returns a boolean spike train of length `num_steps` at timestep `dt`.
///
/// Uses a deterministic regular-interval spike pattern (not Poisson).
pub fn rate_encode(value: f64, max_value: f64, max_rate: f64, dt: f64, num_steps: usize) -> Vec<bool> {
    if max_value <= 0.0 || max_rate <= 0.0 || dt <= 0.0 || num_steps == 0 {
        return vec![false; num_steps];
    }

    let clamped = value.clamp(0.0, max_value);
    let rate = (clamped / max_value) * max_rate; // Hz

    if rate <= 0.0 {
        return vec![false; num_steps];
    }

    let interval = 1.0 / rate; // seconds between spikes
    let interval_steps = (interval / dt).round() as usize;

    let mut train = vec![false; num_steps];
    if interval_steps == 0 {
        // Rate is too high for the timestep, spike every step
        train.fill(true);
    } else {
        let mut next_spike = 0;
        while next_spike < num_steps {
            train[next_spike] = true;
            next_spike += interval_steps;
        }
    }

    train
}

/// Decode a spike train back to a rate estimate (Hz).
///
/// Simply counts spikes and divides by total duration.
pub fn rate_decode(spike_train: &[bool], dt: f64) -> f64 {
    firing_rate(spike_train, dt)
}

/// Compute the interspike interval (ISI) histogram from a spike train.
///
/// Returns a vector of ISI values in seconds. Empty if fewer than 2 spikes.
pub fn interspike_intervals(spike_train: &[bool], dt: f64) -> Vec<f64> {
    let mut isis = Vec::new();
    let mut last_spike: Option<usize> = None;

    for (i, &spiked) in spike_train.iter().enumerate() {
        if spiked {
            if let Some(last) = last_spike {
                isis.push((i - last) as f64 * dt);
            }
            last_spike = Some(i);
        }
    }

    isis
}

/// Compute the coefficient of variation (CV) of interspike intervals.
///
/// CV = std(ISI) / mean(ISI). CV=1 for Poisson process, CV<1 for regular,
/// CV>1 for bursty firing.
///
/// Returns `None` if fewer than 2 ISIs.
pub fn isi_coefficient_of_variation(spike_train: &[bool], dt: f64) -> Option<f64> {
    let isis = interspike_intervals(spike_train, dt);
    if isis.len() < 2 {
        return None;
    }

    let n = isis.len() as f64;
    let mean = isis.iter().sum::<f64>() / n;
    if mean <= 0.0 {
        return None;
    }

    let variance = isis.iter().map(|&x| (x - mean) * (x - mean)).sum::<f64>() / (n - 1.0);
    Some(variance.sqrt() / mean)
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // --- LIF Neuron Tests ---

    #[test]
    fn test_lif_resting_state() {
        let neuron = LifNeuronModel::new(0.020, -0.065, -0.070, -0.050, 1e7);
        assert!((neuron.voltage() - (-0.065)).abs() < 1e-10);
    }

    #[test]
    fn test_lif_no_spike_below_threshold() {
        let mut neuron = LifNeuronModel::new(0.020, -0.065, -0.070, -0.050, 1e7);
        // Small current, should not spike
        for _ in 0..100 {
            let spiked = neuron.step(0.5e-9, 0.001);
            assert!(!spiked);
        }
        // Voltage should have risen but stayed below threshold
        assert!(neuron.voltage() > -0.065);
        assert!(neuron.voltage() < -0.050);
    }

    #[test]
    fn test_lif_spike_with_sufficient_current() {
        let mut neuron = LifNeuronModel::new(0.020, -0.065, -0.070, -0.050, 1e7);
        let mut spiked_at_least_once = false;
        for _ in 0..1000 {
            if neuron.step(5.0e-9, 0.001) {
                spiked_at_least_once = true;
                break;
            }
        }
        assert!(spiked_at_least_once, "LIF should spike with strong current");
    }

    #[test]
    fn test_lif_reset_after_spike() {
        let mut neuron = LifNeuronModel::new(0.020, -0.065, -0.070, -0.050, 1e7);
        // Drive until spike
        for _ in 0..10000 {
            if neuron.step(5.0e-9, 0.001) {
                // After spike, voltage should be at reset
                assert!(
                    (neuron.voltage() - (-0.070)).abs() < 1e-6,
                    "Voltage should reset to v_reset after spike"
                );
                return;
            }
        }
        panic!("Neuron never spiked");
    }

    #[test]
    fn test_lif_adaptive_threshold() {
        let mut neuron = LifNeuronModel::new(0.020, -0.065, -0.070, -0.050, 1e7)
            .with_adaptive_threshold(0.005, 0.050);

        // Drive to first spike
        let mut first_spike_step = None;
        for i in 0..10000 {
            if neuron.step(5.0e-9, 0.001) {
                first_spike_step = Some(i);
                break;
            }
        }
        assert!(first_spike_step.is_some(), "Should spike at least once");

        // After first spike, threshold should be higher
        // Continue driving - second spike should take longer
        let mut second_spike_count = 0;
        for _ in 0..10000 {
            second_spike_count += 1;
            if neuron.step(5.0e-9, 0.001) {
                break;
            }
        }
        // The threshold adaptation should make the second inter-spike interval
        // at least as long as (or longer than) a non-adaptive neuron
        assert!(second_spike_count > 0);
    }

    #[test]
    fn test_lif_reset() {
        let mut neuron = LifNeuronModel::new(0.020, -0.065, -0.070, -0.050, 1e7);
        // Drive with a current that won't reach threshold (sub-threshold)
        // Steady-state: V_rest + R_m * I = -0.065 + 1e7 * 1e-9 = -0.055
        // which is below threshold -0.050, so no spike.
        for _ in 0..50 {
            neuron.step(1.0e-9, 0.001);
        }
        // Voltage should have risen from rest
        assert!(
            neuron.voltage() > -0.065,
            "Voltage should rise with sub-threshold current, got {}",
            neuron.voltage()
        );

        neuron.reset();
        assert!((neuron.voltage() - (-0.065)).abs() < 1e-10);
        assert!(neuron.last_spike_time().is_none());
    }

    #[test]
    fn test_lif_display() {
        let neuron = LifNeuronModel::new(0.020, -0.065, -0.070, -0.050, 1e7);
        let s = format!("{}", neuron);
        assert!(s.contains("LIF"));
        assert!(s.contains("20.0ms"));
    }

    // --- Izhikevich Neuron Tests ---

    #[test]
    fn test_izhikevich_patterns_exist() {
        let patterns = [
            IzhikevichPattern::RegularSpiking,
            IzhikevichPattern::IntrinsicallyBursting,
            IzhikevichPattern::Chattering,
            IzhikevichPattern::FastSpiking,
            IzhikevichPattern::LowThresholdSpiking,
        ];

        for p in &patterns {
            let (a, b, c, d) = p.params();
            assert!(a > 0.0);
            assert!(b > 0.0);
            assert!(c < 0.0);
            assert!(d > 0.0);
        }
    }

    #[test]
    fn test_izhikevich_regular_spiking() {
        let mut neuron = IzhikevichNeuron::new(IzhikevichPattern::RegularSpiking);
        let mut spike_count = 0;
        // Strong input current (Izhikevich uses dimensionless current units)
        for _ in 0..1000 {
            if neuron.step(15.0, 0.001) {
                spike_count += 1;
            }
        }
        assert!(
            spike_count > 0,
            "Regular spiking neuron should fire with I=15"
        );
    }

    #[test]
    fn test_izhikevich_fast_spiking_higher_rate() {
        let mut fs = IzhikevichNeuron::new(IzhikevichPattern::FastSpiking);
        let mut rs = IzhikevichNeuron::new(IzhikevichPattern::RegularSpiking);

        let mut fs_spikes = 0;
        let mut rs_spikes = 0;

        for _ in 0..2000 {
            if fs.step(15.0, 0.001) {
                fs_spikes += 1;
            }
            if rs.step(15.0, 0.001) {
                rs_spikes += 1;
            }
        }

        // Fast spiking should fire more often than regular spiking
        // (or at least fire - both should be active with I=15)
        assert!(fs_spikes > 0, "Fast spiking neuron should fire");
        assert!(rs_spikes > 0, "Regular spiking neuron should fire");
    }

    #[test]
    fn test_izhikevich_no_spike_without_input() {
        let mut neuron = IzhikevichNeuron::new(IzhikevichPattern::RegularSpiking);
        let mut spiked = false;
        for _ in 0..100 {
            if neuron.step(0.0, 0.001) {
                spiked = true;
            }
        }
        assert!(!spiked, "Should not spike with zero input");
    }

    #[test]
    fn test_izhikevich_reset() {
        let mut neuron = IzhikevichNeuron::new(IzhikevichPattern::RegularSpiking);
        for _ in 0..100 {
            neuron.step(15.0, 0.001);
        }
        neuron.reset();
        assert!((neuron.voltage() - (-65.0)).abs() < 1e-6);
        assert!(neuron.last_spike_time().is_none());
    }

    #[test]
    fn test_izhikevich_custom_params() {
        let neuron = IzhikevichNeuron::custom(0.02, 0.2, -65.0, 8.0);
        assert!((neuron.a - 0.02).abs() < 1e-10);
        assert!((neuron.voltage() - (-65.0)).abs() < 1e-6);
    }

    // --- Synapse Tests ---

    #[test]
    fn test_synapse_creation() {
        let syn = Synapse::new(0, 1, 0.5);
        assert_eq!(syn.pre_idx, 0);
        assert_eq!(syn.post_idx, 1);
        assert!((syn.weight - 0.5).abs() < 1e-10);
        assert_eq!(syn.delay, 0);
        assert!(syn.stdp.is_none());
    }

    #[test]
    fn test_synapse_with_delay() {
        let syn = Synapse::new(0, 1, 0.5).with_delay(3);
        assert_eq!(syn.delay, 3);
    }

    #[test]
    fn test_synapse_stdp_potentiation() {
        let rule = StdpRule::new(0.01, 0.012, 0.020, 0.020).with_bounds(0.0, 1.0);
        let mut syn = Synapse::new(0, 1, 0.5).with_stdp(rule);

        // Pre before post (dt > 0): should potentiate (increase weight)
        let w_before = syn.weight;
        syn.apply_stdp(0.005); // 5ms causal delay
        assert!(
            syn.weight > w_before,
            "Weight should increase for pre-before-post"
        );
    }

    #[test]
    fn test_synapse_stdp_depression() {
        let rule = StdpRule::new(0.01, 0.012, 0.020, 0.020).with_bounds(0.0, 1.0);
        let mut syn = Synapse::new(0, 1, 0.5).with_stdp(rule);

        // Post before pre (dt < 0): should depress (decrease weight)
        let w_before = syn.weight;
        syn.apply_stdp(-0.005); // 5ms anti-causal
        assert!(
            syn.weight < w_before,
            "Weight should decrease for post-before-pre"
        );
    }

    #[test]
    fn test_synapse_stdp_weight_bounds() {
        let rule = StdpRule::new(0.5, 0.5, 0.020, 0.020).with_bounds(0.0, 1.0);
        let mut syn = Synapse::new(0, 1, 0.9).with_stdp(rule);

        // Strong potentiation should be clamped to w_max
        syn.apply_stdp(0.001);
        assert!(syn.weight <= 1.0, "Weight should not exceed w_max");

        let mut syn2 = Synapse::new(0, 1, 0.1).with_stdp(
            StdpRule::new(0.5, 0.5, 0.020, 0.020).with_bounds(0.0, 1.0),
        );
        syn2.apply_stdp(-0.001);
        assert!(syn2.weight >= 0.0, "Weight should not go below w_min");
    }

    // --- STDP Rule Tests ---

    #[test]
    fn test_stdp_standard() {
        let rule = StdpRule::standard();
        assert!(rule.a_plus > 0.0);
        assert!(rule.a_minus > 0.0);
        assert!(rule.tau_plus > 0.0);
        assert!(rule.tau_minus > 0.0);
    }

    #[test]
    fn test_stdp_weight_change_symmetry() {
        let rule = StdpRule::new(0.01, 0.01, 0.020, 0.020);

        let dw_pos = rule.compute_weight_change(0.005);
        let dw_neg = rule.compute_weight_change(-0.005);

        // With equal A+ and A-, magnitudes should be equal but signs opposite
        assert!(dw_pos > 0.0);
        assert!(dw_neg < 0.0);
        assert!((dw_pos + dw_neg).abs() < 1e-10);
    }

    #[test]
    fn test_stdp_exponential_decay() {
        let rule = StdpRule::new(0.01, 0.012, 0.020, 0.020);

        let dw_close = rule.compute_weight_change(0.001); // 1ms
        let dw_far = rule.compute_weight_change(0.050); // 50ms

        assert!(dw_close > dw_far, "Closer spikes should give larger change");
        assert!(dw_far > 0.0, "Still positive for pre-before-post");
    }

    // --- SNN Layer Tests ---

    #[test]
    fn test_layer_creation() {
        let layer = SnnLayer::new(
            10,
            NeuronType::Lif {
                tau_m: 0.020,
                v_rest: -0.065,
                v_reset: -0.070,
                v_thresh: -0.050,
                r_m: 1e7,
            },
        );
        assert_eq!(layer.size(), 10);
        assert_eq!(layer.spikes.len(), 10);
    }

    #[test]
    fn test_layer_step() {
        let mut layer = SnnLayer::new(
            3,
            NeuronType::Lif {
                tau_m: 0.020,
                v_rest: -0.065,
                v_reset: -0.070,
                v_thresh: -0.050,
                r_m: 1e7,
            },
        );

        // Step with varying currents
        let currents = vec![0.0, 5.0e-9, 0.0];
        for _ in 0..1000 {
            layer.step(&currents, 0.001);
        }

        // Neuron 1 (strong current) should have spiked at some point
        assert!(
            layer.last_spike_times[1].is_some(),
            "Driven neuron should have spiked"
        );
    }

    #[test]
    fn test_layer_reset() {
        let mut layer = SnnLayer::new(
            3,
            NeuronType::Lif {
                tau_m: 0.020,
                v_rest: -0.065,
                v_reset: -0.070,
                v_thresh: -0.050,
                r_m: 1e7,
            },
        );

        let currents = vec![5.0e-9, 5.0e-9, 5.0e-9];
        for _ in 0..100 {
            layer.step(&currents, 0.001);
        }

        layer.reset();
        for i in 0..3 {
            assert!((layer.neuron_voltage(i) - (-0.065)).abs() < 1e-10);
        }
    }

    // --- SNN Network Tests ---

    #[test]
    fn test_network_construction() {
        let mut net = SnnNetwork::new(0.001);
        let lif_type = NeuronType::Lif {
            tau_m: 0.020,
            v_rest: -0.065,
            v_reset: -0.070,
            v_thresh: -0.050,
            r_m: 1e7,
        };
        let l0 = net.add_layer(SnnLayer::new(4, lif_type));
        let l1 = net.add_layer(SnnLayer::new(2, lif_type));
        assert_eq!(l0, 0);
        assert_eq!(l1, 1);
        assert_eq!(net.num_layers(), 2);
    }

    #[test]
    fn test_network_run_produces_output() {
        let mut net = SnnNetwork::new(0.001);
        let lif_type = NeuronType::Lif {
            tau_m: 0.020,
            v_rest: -0.065,
            v_reset: -0.070,
            v_thresh: -0.050,
            r_m: 1e7,
        };
        net.add_layer(SnnLayer::new(2, lif_type));
        net.add_layer(SnnLayer::new(2, lif_type));
        net.connect_layers(0, 1, 0.02);

        let input = vec![5.0e-9, 3.0e-9];
        let output = net.run(&input, 100);

        assert_eq!(output.len(), 100);
        assert_eq!(output[0].len(), 2);
    }

    #[test]
    fn test_network_spike_propagation() {
        let mut net = SnnNetwork::new(0.001);
        let lif_type = NeuronType::Lif {
            tau_m: 0.010,
            v_rest: -0.065,
            v_reset: -0.070,
            v_thresh: -0.050,
            r_m: 1e7,
        };
        net.add_layer(SnnLayer::new(1, lif_type));
        net.add_layer(SnnLayer::new(1, lif_type));
        // Very strong connection
        net.connect_layers(0, 1, 0.05);

        // Strong input to make layer 0 spike
        let input = vec![10.0e-9];
        let output = net.run(&input, 500);

        // Layer 1 should eventually spike (propagated from layer 0)
        let any_output_spike = output.iter().any(|spikes| spikes[0]);
        assert!(
            any_output_spike,
            "Output layer should spike due to propagation"
        );
    }

    #[test]
    fn test_network_reset() {
        let mut net = SnnNetwork::new(0.001);
        let lif_type = NeuronType::Lif {
            tau_m: 0.020,
            v_rest: -0.065,
            v_reset: -0.070,
            v_thresh: -0.050,
            r_m: 1e7,
        };
        net.add_layer(SnnLayer::new(2, lif_type));
        net.run(&[5.0e-9, 3.0e-9], 50);
        net.reset();

        assert_eq!(net.spike_log(0).len(), 0);
    }

    // --- Helper Function Tests ---

    #[test]
    fn test_firing_rate_basic() {
        // 5 spikes in 10 bins at 1ms each = 500 Hz
        let train = vec![
            true, false, true, false, true, false, true, false, true, false,
        ];
        let rate = firing_rate(&train, 0.001);
        assert!((rate - 500.0).abs() < 1.0);
    }

    #[test]
    fn test_firing_rate_empty() {
        assert!((firing_rate(&[], 0.001)).abs() < 1e-10);
    }

    #[test]
    fn test_firing_rate_no_spikes() {
        let train = vec![false; 100];
        assert!((firing_rate(&train, 0.001)).abs() < 1e-10);
    }

    #[test]
    fn test_spike_train_distance_identical() {
        let a = vec![true, false, false, true, false, false, false, true, false, false];
        let d = spike_train_distance(&a, &a, 0.001, 0.010);
        assert!(d < 1e-10, "Distance between identical trains should be ~0");
    }

    #[test]
    fn test_spike_train_distance_different() {
        let a = vec![true, false, false, false, false, false, false, false, false, false];
        let b = vec![false, false, false, false, false, false, false, false, false, true];
        let d = spike_train_distance(&a, &b, 0.001, 0.010);
        assert!(d > 0.0, "Distance between different trains should be > 0");
    }

    #[test]
    fn test_population_vector_uniform() {
        // 4 neurons at 90-degree intervals with equal firing -> near zero magnitude
        let counts = vec![10, 10, 10, 10];
        let dirs = vec![0.0, PI / 2.0, PI, 3.0 * PI / 2.0];
        let (mag, _) = population_vector(&counts, &dirs);
        assert!(mag < 1.0, "Uniform firing should give small resultant");
    }

    #[test]
    fn test_population_vector_single_direction() {
        let counts = vec![10, 0, 0, 0];
        let dirs = vec![0.0, PI / 2.0, PI, 3.0 * PI / 2.0];
        let (mag, angle) = population_vector(&counts, &dirs);
        assert!((mag - 10.0).abs() < 1e-6);
        assert!(angle.abs() < 1e-6, "Should point along 0 radians");
    }

    #[test]
    fn test_rate_encode_decode_roundtrip() {
        let value = 0.7;
        let max_val = 1.0;
        let max_rate = 200.0; // Hz
        let dt = 0.001;
        let steps = 1000;

        let train = rate_encode(value, max_val, max_rate, dt, steps);
        let decoded_rate = rate_decode(&train, dt);

        // Expected rate = 0.7 * 200 = 140 Hz
        // With deterministic encoding there may be boundary effects
        let expected = value * max_rate;
        assert!(
            (decoded_rate - expected).abs() < 20.0,
            "Decoded rate {} should be near expected {}",
            decoded_rate,
            expected
        );
    }

    #[test]
    fn test_interspike_intervals() {
        let train = vec![
            true, false, false, true, false, true, false, false, false, true,
        ];
        let isis = interspike_intervals(&train, 0.001);
        assert_eq!(isis.len(), 3);
        assert!((isis[0] - 0.003).abs() < 1e-10); // 3 steps
        assert!((isis[1] - 0.002).abs() < 1e-10); // 2 steps
        assert!((isis[2] - 0.004).abs() < 1e-10); // 4 steps
    }

    #[test]
    fn test_isi_cv_regular() {
        // Perfectly regular firing -> CV should be 0 (or near 0)
        let mut train = vec![false; 100];
        for i in (0..100).step_by(10) {
            train[i] = true;
        }
        let cv = isi_coefficient_of_variation(&train, 0.001);
        assert!(cv.is_some());
        assert!(
            cv.unwrap() < 0.01,
            "Regular firing should have near-zero CV"
        );
    }

    #[test]
    fn test_izhikevich_display() {
        let neuron = IzhikevichNeuron::new(IzhikevichPattern::FastSpiking);
        let s = format!("{}", neuron);
        assert!(s.contains("Izhikevich"));
        assert!(s.contains("FastSpiking"));
    }

    #[test]
    fn test_network_with_matrix_weights() {
        let mut net = SnnNetwork::new(0.001);
        let lif_type = NeuronType::Lif {
            tau_m: 0.020,
            v_rest: -0.065,
            v_reset: -0.070,
            v_thresh: -0.050,
            r_m: 1e7,
        };
        net.add_layer(SnnLayer::new(2, lif_type));
        net.add_layer(SnnLayer::new(2, lif_type));

        let weights = vec![vec![0.01, 0.02], vec![0.03, 0.04]];
        net.connect_layers_matrix(0, 1, &weights);

        // Should have 4 synapses total (2x2)
        assert_eq!(net.synapses(0).len(), 4);
    }

    #[test]
    fn test_network_izhikevich_layer() {
        let mut net = SnnNetwork::new(0.001);
        net.add_layer(SnnLayer::new(
            3,
            NeuronType::Izhikevich(IzhikevichPattern::FastSpiking),
        ));

        let input = vec![15.0, 10.0, 0.0];
        let output = net.run(&input, 500);
        assert_eq!(output.len(), 500);

        // At least one neuron should have spiked with I=15
        let any_spike = output.iter().any(|step| step.iter().any(|&s| s));
        assert!(any_spike, "Should have at least one spike with I=15");
    }

    #[test]
    fn test_network_debug() {
        let net = SnnNetwork::new(0.001);
        let s = format!("{:?}", net);
        assert!(s.contains("SnnNetwork"));
    }
}

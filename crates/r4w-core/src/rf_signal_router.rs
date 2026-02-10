//! Multi-port RF signal routing, switching, and combining for SDR signal chains.
//!
//! This module provides an [`RfSignalRouter`] that models an NxM crossbar switch matrix
//! with per-route gain, frequency-selective filtering, isolation/crosstalk modelling,
//! crossfade switching, signal combining, splitting, and automatic
//! input selection based on signal power.
//!
//! Complex samples are represented as `(f64, f64)` tuples `(re, im)`.
//!
//! # Example
//!
//! ```
//! use r4w_core::rf_signal_router::{RfSignalRouter, Route};
//!
//! // Create a 4-input, 2-output router
//! let mut router = RfSignalRouter::new(4, 2);
//!
//! // Route input port 0 to output port 1 with 3 dB gain
//! router.add_route(Route::new(0, 1).with_gain_db(3.0));
//!
//! // Process a block of samples (one sample per input port)
//! let inputs = vec![(1.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)];
//! let outputs = router.process(&inputs);
//!
//! // Output port 1 receives the routed signal (with ~3 dB gain ≈ x1.413)
//! assert!(outputs[1].0 > 1.4);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// A single complex sample: `(re, im)`.
pub type Complex = (f64, f64);

/// Inline complex helpers (no external crate).
fn cx_add(a: Complex, b: Complex) -> Complex {
    (a.0 + b.0, a.1 + b.1)
}

#[allow(dead_code)]
fn cx_mul(a: Complex, b: Complex) -> Complex {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn cx_scale(a: Complex, s: f64) -> Complex {
    (a.0 * s, a.1 * s)
}

fn cx_mag_sq(a: Complex) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

// ---------------------------------------------------------------------------
// Route
// ---------------------------------------------------------------------------

/// Describes a single route from an input port to an output port.
#[derive(Debug, Clone)]
pub struct Route {
    /// Source input port index.
    pub input: usize,
    /// Destination output port index.
    pub output: usize,
    /// Linear gain applied along this route (default 1.0).
    pub gain: f64,
    /// Optional bandpass filter for frequency-selective routing.
    pub bandpass: Option<BandpassSpec>,
    /// Whether this route is currently enabled.
    pub enabled: bool,
}

/// Parameters for a simple single-pole bandpass (for demonstration).
#[derive(Debug, Clone, Copy)]
pub struct BandpassSpec {
    /// Center frequency in Hz.
    pub center_freq_hz: f64,
    /// Bandwidth in Hz.
    pub bandwidth_hz: f64,
    /// Sample rate in Hz.
    pub sample_rate_hz: f64,
}

impl Route {
    /// Create a new route with unity gain.
    pub fn new(input: usize, output: usize) -> Self {
        Self {
            input,
            output,
            gain: 1.0,
            bandpass: None,
            enabled: true,
        }
    }

    /// Set gain in dB.
    pub fn with_gain_db(mut self, db: f64) -> Self {
        self.gain = 10.0_f64.powf(db / 20.0);
        self
    }

    /// Set linear gain.
    pub fn with_gain_linear(mut self, g: f64) -> Self {
        self.gain = g;
        self
    }

    /// Attach a bandpass filter specification to this route.
    pub fn with_bandpass(mut self, center: f64, bw: f64, fs: f64) -> Self {
        self.bandpass = Some(BandpassSpec {
            center_freq_hz: center,
            bandwidth_hz: bw,
            sample_rate_hz: fs,
        });
        self
    }

    /// Enable or disable the route.
    pub fn set_enabled(&mut self, en: bool) {
        self.enabled = en;
    }
}

// ---------------------------------------------------------------------------
// Per-route bandpass filter state (simple 2nd-order resonator)
// ---------------------------------------------------------------------------

/// Minimal IIR bandpass state kept per-route.
#[derive(Debug, Clone)]
struct BpfState {
    /// Coefficients: b0, b1, b2, a1, a2
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
    // state variables (Direct Form I)
    x1: Complex,
    x2: Complex,
    y1: Complex,
    y2: Complex,
}

impl BpfState {
    fn from_spec(spec: &BandpassSpec) -> Self {
        let w0 = 2.0 * PI * spec.center_freq_hz / spec.sample_rate_hz;
        let bw_norm = spec.bandwidth_hz / spec.sample_rate_hz;
        let q = if bw_norm > 0.0 { 1.0 / (2.0 * bw_norm) } else { 1.0 };
        let alpha = w0.sin() / (2.0 * q);

        let b0 = alpha;
        let b1 = 0.0;
        let b2 = -alpha;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * w0.cos();
        let a2 = 1.0 - alpha;

        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1: a1 / a0,
            a2: a2 / a0,
            x1: (0.0, 0.0),
            x2: (0.0, 0.0),
            y1: (0.0, 0.0),
            y2: (0.0, 0.0),
        }
    }

    fn process(&mut self, x: Complex) -> Complex {
        let y_re = self.b0 * x.0 + self.b1 * self.x1.0 + self.b2 * self.x2.0
            - self.a1 * self.y1.0 - self.a2 * self.y2.0;
        let y_im = self.b0 * x.1 + self.b1 * self.x1.1 + self.b2 * self.x2.1
            - self.a1 * self.y1.1 - self.a2 * self.y2.1;
        let y = (y_re, y_im);
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }

    fn reset(&mut self) {
        self.x1 = (0.0, 0.0);
        self.x2 = (0.0, 0.0);
        self.y1 = (0.0, 0.0);
        self.y2 = (0.0, 0.0);
    }
}

// ---------------------------------------------------------------------------
// PortStats
// ---------------------------------------------------------------------------

/// Accumulated statistics for a single port.
#[derive(Debug, Clone)]
pub struct PortStats {
    /// Running sum of power (magnitude squared) seen at this port.
    pub total_power: f64,
    /// Number of samples processed through this port.
    pub sample_count: u64,
    /// Peak instantaneous power observed.
    pub peak_power: f64,
}

impl PortStats {
    fn new() -> Self {
        Self {
            total_power: 0.0,
            sample_count: 0,
            peak_power: 0.0,
        }
    }

    fn update(&mut self, sample: Complex) {
        let p = cx_mag_sq(sample);
        self.total_power += p;
        self.sample_count += 1;
        if p > self.peak_power {
            self.peak_power = p;
        }
    }

    /// Average power (linear). Returns 0 if no samples yet.
    pub fn average_power(&self) -> f64 {
        if self.sample_count == 0 {
            0.0
        } else {
            self.total_power / self.sample_count as f64
        }
    }

    /// Average power in dBFS (relative to full-scale = 1.0).
    pub fn average_power_dbfs(&self) -> f64 {
        let avg = self.average_power();
        if avg <= 0.0 {
            f64::NEG_INFINITY
        } else {
            10.0 * avg.log10()
        }
    }
}

// ---------------------------------------------------------------------------
// RfSignalRouter
// ---------------------------------------------------------------------------

/// Multi-port RF signal router implementing an NxM crossbar switch.
///
/// Supports per-route gain, bandpass filtering, isolation modelling,
/// crossfade switching, signal combining, splitting, and automatic
/// input selection based on power.
#[derive(Debug)]
pub struct RfSignalRouter {
    num_inputs: usize,
    num_outputs: usize,
    routes: Vec<Route>,
    bpf_states: Vec<Option<BpfState>>,
    /// Isolation between ports in dB (higher = better isolation).
    /// Crosstalk leaks at -isolation dB from every unconnected input to each output.
    isolation_db: f64,
    /// Crossfade length in samples for route switching transitions.
    crossfade_len: usize,
    /// Per-input-port statistics.
    input_stats: Vec<PortStats>,
    /// Per-output-port statistics.
    output_stats: Vec<PortStats>,
    /// Crossfade state: remaining samples in current transition.
    crossfade_remaining: usize,
    /// Previous gains snapshot (per route) used during crossfade.
    prev_gains: Vec<f64>,
}

impl RfSignalRouter {
    // ----- Construction -----

    /// Create a new router with `num_inputs` input ports and `num_outputs` output ports.
    /// No routes are configured initially.
    pub fn new(num_inputs: usize, num_outputs: usize) -> Self {
        Self {
            num_inputs,
            num_outputs,
            routes: Vec::new(),
            bpf_states: Vec::new(),
            isolation_db: 60.0, // default 60 dB isolation
            crossfade_len: 0,
            input_stats: (0..num_inputs).map(|_| PortStats::new()).collect(),
            output_stats: (0..num_outputs).map(|_| PortStats::new()).collect(),
            crossfade_remaining: 0,
            prev_gains: Vec::new(),
        }
    }

    // ----- Configuration -----

    /// Add a route. Returns the route index.
    pub fn add_route(&mut self, route: Route) -> usize {
        let bpf = route.bandpass.as_ref().map(BpfState::from_spec);
        self.routes.push(route);
        self.bpf_states.push(bpf);
        self.routes.len() - 1
    }

    /// Remove all routes.
    pub fn clear_routes(&mut self) {
        self.routes.clear();
        self.bpf_states.clear();
    }

    /// Set inter-port isolation in dB (e.g. 60 dB).
    pub fn set_isolation_db(&mut self, db: f64) {
        self.isolation_db = db;
    }

    /// Get current isolation in dB.
    pub fn isolation_db(&self) -> f64 {
        self.isolation_db
    }

    /// Set crossfade transition length in samples.
    pub fn set_crossfade_len(&mut self, samples: usize) {
        self.crossfade_len = samples;
    }

    /// Number of input ports.
    pub fn num_inputs(&self) -> usize {
        self.num_inputs
    }

    /// Number of output ports.
    pub fn num_outputs(&self) -> usize {
        self.num_outputs
    }

    /// Number of configured routes.
    pub fn num_routes(&self) -> usize {
        self.routes.len()
    }

    /// Get a reference to the route at the given index.
    pub fn route(&self, idx: usize) -> Option<&Route> {
        self.routes.get(idx)
    }

    /// Get a mutable reference to the route at the given index.
    pub fn route_mut(&mut self, idx: usize) -> Option<&mut Route> {
        self.routes.get_mut(idx)
    }

    /// Input port statistics.
    pub fn input_stats(&self, port: usize) -> Option<&PortStats> {
        self.input_stats.get(port)
    }

    /// Output port statistics.
    pub fn output_stats(&self, port: usize) -> Option<&PortStats> {
        self.output_stats.get(port)
    }

    /// Reset all statistics.
    pub fn reset_stats(&mut self) {
        for s in &mut self.input_stats {
            *s = PortStats::new();
        }
        for s in &mut self.output_stats {
            *s = PortStats::new();
        }
    }

    /// Reset bandpass filter states (clear memory).
    pub fn reset_filters(&mut self) {
        for bpf in &mut self.bpf_states {
            if let Some(ref mut f) = bpf {
                f.reset();
            }
        }
    }

    // ----- Processing -----

    /// Process one block of samples.
    ///
    /// `inputs` must have exactly `num_inputs` elements.
    /// Returns a vector of `num_outputs` output samples.
    ///
    /// Each output is the sum of all routed inputs (combining), plus
    /// crosstalk from isolation leakage.
    pub fn process(&mut self, inputs: &[Complex]) -> Vec<Complex> {
        assert_eq!(
            inputs.len(),
            self.num_inputs,
            "Expected {} input samples, got {}",
            self.num_inputs,
            inputs.len()
        );

        // Update input stats
        for (i, &s) in inputs.iter().enumerate() {
            self.input_stats[i].update(s);
        }

        let mut outputs = vec![(0.0_f64, 0.0_f64); self.num_outputs];

        // Apply each active route (combining at outputs)
        for (ri, route) in self.routes.iter().enumerate() {
            if !route.enabled {
                continue;
            }
            if route.input >= self.num_inputs || route.output >= self.num_outputs {
                continue;
            }

            let mut sample = inputs[route.input];

            // Bandpass filter (if configured)
            if let Some(ref mut bpf) = self.bpf_states[ri] {
                sample = bpf.process(sample);
            }

            // Determine effective gain (handle crossfade)
            let gain = if self.crossfade_remaining > 0 && ri < self.prev_gains.len() {
                let alpha =
                    1.0 - (self.crossfade_remaining as f64 / self.crossfade_len as f64);
                // Cosine crossfade
                let alpha_smooth = 0.5 * (1.0 - (PI * alpha).cos());
                self.prev_gains[ri] * (1.0 - alpha_smooth)
                    + route.gain * alpha_smooth
            } else {
                route.gain
            };

            sample = cx_scale(sample, gain);
            outputs[route.output] = cx_add(outputs[route.output], sample);
        }

        // Isolation / crosstalk model: every input leaks into every output
        // at -isolation_db unless there is an explicit route.
        let leak_gain = 10.0_f64.powf(-self.isolation_db / 20.0);
        for (oi, out) in outputs.iter_mut().enumerate() {
            for (ii, &inp) in inputs.iter().enumerate() {
                // Only add crosstalk if there is no explicit enabled route for this pair
                let has_route = self.routes.iter().any(|r| {
                    r.enabled && r.input == ii && r.output == oi
                });
                if !has_route {
                    *out = cx_add(*out, cx_scale(inp, leak_gain));
                }
            }
        }

        // Advance crossfade counter
        if self.crossfade_remaining > 0 {
            self.crossfade_remaining -= 1;
        }

        // Update output stats
        for (i, &s) in outputs.iter().enumerate() {
            self.output_stats[i].update(s);
        }

        outputs
    }

    /// Process a block of many samples (one vector per input port).
    ///
    /// Returns one vector per output port.
    pub fn process_block(&mut self, inputs: &[Vec<Complex>]) -> Vec<Vec<Complex>> {
        assert_eq!(inputs.len(), self.num_inputs);
        let block_len = inputs[0].len();
        for inp in inputs {
            assert_eq!(inp.len(), block_len);
        }

        let mut outputs: Vec<Vec<Complex>> =
            (0..self.num_outputs).map(|_| Vec::with_capacity(block_len)).collect();

        for n in 0..block_len {
            let frame: Vec<Complex> = inputs.iter().map(|ch| ch[n]).collect();
            let out = self.process(&frame);
            for (oi, &s) in out.iter().enumerate() {
                outputs[oi].push(s);
            }
        }

        outputs
    }

    // ----- Switching -----

    /// Trigger a crossfade transition. Snapshots current gains, so subsequent
    /// gain changes will be smoothly interpolated over `crossfade_len` samples.
    pub fn begin_crossfade(&mut self) {
        self.prev_gains = self.routes.iter().map(|r| r.gain).collect();
        self.crossfade_remaining = self.crossfade_len;
    }

    // ----- Splitting -----

    /// Split a single input to multiple outputs with equal power division.
    ///
    /// Adds routes from `input_port` to each port in `output_ports`.
    /// Insertion loss = 10*log10(N) dB for N outputs.
    pub fn add_split(
        &mut self,
        input_port: usize,
        output_ports: &[usize],
    ) -> Vec<usize> {
        let n = output_ports.len() as f64;
        let split_gain = 1.0 / n.sqrt(); // power split: each gets 1/N power
        output_ports
            .iter()
            .map(|&op| {
                self.add_route(Route::new(input_port, op).with_gain_linear(split_gain))
            })
            .collect()
    }

    /// Combine multiple inputs into a single output (power summation).
    ///
    /// Adds routes from each `input_port` to `output_port` with unity gain.
    pub fn add_combine(
        &mut self,
        input_ports: &[usize],
        output_port: usize,
    ) -> Vec<usize> {
        input_ports
            .iter()
            .map(|&ip| self.add_route(Route::new(ip, output_port)))
            .collect()
    }

    // ----- Automatic port selection -----

    /// Return the input port index with the highest average power.
    /// Returns `None` if no samples have been processed.
    pub fn strongest_input(&self) -> Option<usize> {
        self.input_stats
            .iter()
            .enumerate()
            .filter(|(_, s)| s.sample_count > 0)
            .max_by(|(_, a), (_, b)| {
                a.average_power()
                    .partial_cmp(&b.average_power())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
    }

    /// Auto-select: route the strongest input port to the given output,
    /// disabling other routes to that output. Returns the chosen input port index.
    pub fn auto_select_to_output(&mut self, output: usize) -> Option<usize> {
        let best = self.strongest_input()?;
        // Disable existing routes to this output
        for r in &mut self.routes {
            if r.output == output {
                r.enabled = false;
            }
        }
        self.add_route(Route::new(best, output));
        Some(best)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // 1. Basic construction
    #[test]
    fn test_new_router() {
        let r = RfSignalRouter::new(4, 2);
        assert_eq!(r.num_inputs(), 4);
        assert_eq!(r.num_outputs(), 2);
        assert_eq!(r.num_routes(), 0);
    }

    // 2. Single route pass-through
    #[test]
    fn test_single_route_passthrough() {
        let mut r = RfSignalRouter::new(2, 2);
        r.set_isolation_db(200.0); // effectively no crosstalk
        r.add_route(Route::new(0, 1));
        let out = r.process(&[(1.0, 0.5), (0.0, 0.0)]);
        assert!(approx_eq(out[1].0, 1.0, 1e-6));
        assert!(approx_eq(out[1].1, 0.5, 1e-6));
    }

    // 3. Gain in dB
    #[test]
    fn test_gain_db() {
        let mut r = RfSignalRouter::new(1, 1);
        r.set_isolation_db(200.0);
        r.add_route(Route::new(0, 0).with_gain_db(6.0));
        let out = r.process(&[(1.0, 0.0)]);
        // +6 dB ~ x1.9953
        assert!(approx_eq(out[0].0, 10.0_f64.powf(6.0 / 20.0), 1e-4));
    }

    // 4. Gain in linear
    #[test]
    fn test_gain_linear() {
        let mut r = RfSignalRouter::new(1, 1);
        r.set_isolation_db(200.0);
        r.add_route(Route::new(0, 0).with_gain_linear(0.5));
        let out = r.process(&[(2.0, 4.0)]);
        assert!(approx_eq(out[0].0, 1.0, EPS));
        assert!(approx_eq(out[0].1, 2.0, EPS));
    }

    // 5. Combining two inputs to one output
    #[test]
    fn test_combine() {
        let mut r = RfSignalRouter::new(3, 1);
        r.set_isolation_db(200.0);
        r.add_combine(&[0, 2], 0);
        let out = r.process(&[(1.0, 0.0), (0.0, 0.0), (0.0, 1.0)]);
        assert!(approx_eq(out[0].0, 1.0, 1e-6));
        assert!(approx_eq(out[0].1, 1.0, 1e-6));
    }

    // 6. Splitting with insertion loss
    #[test]
    fn test_split_insertion_loss() {
        let mut r = RfSignalRouter::new(1, 4);
        r.set_isolation_db(200.0);
        r.add_split(0, &[0, 1, 2, 3]);
        let out = r.process(&[(1.0, 0.0)]);
        // Split to 4 outputs: each gets gain = 1/sqrt(4) = 0.5
        for o in &out {
            assert!(approx_eq(o.0, 0.5, 1e-6));
        }
        // Sum of power = 4 * 0.25 = 1.0 (power conserved)
        let total_power: f64 = out.iter().map(|s| cx_mag_sq(*s)).sum();
        assert!(approx_eq(total_power, 1.0, 1e-6));
    }

    // 7. Disabled route
    #[test]
    fn test_disabled_route() {
        let mut r = RfSignalRouter::new(1, 1);
        r.set_isolation_db(200.0);
        let idx = r.add_route(Route::new(0, 0));
        r.route_mut(idx).unwrap().set_enabled(false);
        let out = r.process(&[(1.0, 0.0)]);
        assert!(approx_eq(out[0].0, 0.0, 1e-6));
    }

    // 8. Isolation / crosstalk
    #[test]
    fn test_isolation_crosstalk() {
        let mut r = RfSignalRouter::new(2, 1);
        // Route input 0 to output 0, but input 1 is unrouted
        r.add_route(Route::new(0, 0));
        r.set_isolation_db(40.0); // 40 dB isolation
        let out = r.process(&[(0.0, 0.0), (1.0, 0.0)]);
        // Input 1 leaks to output 0 at -40 dB = 0.01 linear
        let leak = 10.0_f64.powf(-40.0 / 20.0);
        assert!(approx_eq(out[0].0, leak, 1e-6));
    }

    // 9. No crosstalk on routed pair
    #[test]
    fn test_no_double_count_routed() {
        let mut r = RfSignalRouter::new(1, 1);
        r.set_isolation_db(0.0); // 0 dB = unity leak
        r.add_route(Route::new(0, 0).with_gain_linear(1.0));
        let out = r.process(&[(1.0, 0.0)]);
        // Should be exactly 1.0 (route), not 2.0 (route + leak)
        assert!(approx_eq(out[0].0, 1.0, 1e-6));
    }

    // 10. Port statistics
    #[test]
    fn test_port_stats() {
        let mut r = RfSignalRouter::new(1, 1);
        r.set_isolation_db(200.0);
        r.add_route(Route::new(0, 0));
        for _ in 0..100 {
            r.process(&[(1.0, 0.0)]);
        }
        let is = r.input_stats(0).unwrap();
        assert_eq!(is.sample_count, 100);
        assert!(approx_eq(is.average_power(), 1.0, EPS));
        assert!(approx_eq(is.peak_power, 1.0, EPS));
    }

    // 11. Reset statistics
    #[test]
    fn test_reset_stats() {
        let mut r = RfSignalRouter::new(1, 1);
        r.set_isolation_db(200.0);
        r.add_route(Route::new(0, 0));
        r.process(&[(1.0, 0.0)]);
        r.reset_stats();
        assert_eq!(r.input_stats(0).unwrap().sample_count, 0);
        assert_eq!(r.output_stats(0).unwrap().sample_count, 0);
    }

    // 12. Strongest input selection
    #[test]
    fn test_strongest_input() {
        let mut r = RfSignalRouter::new(3, 1);
        r.set_isolation_db(200.0);
        r.add_combine(&[0, 1, 2], 0);
        // Input 1 is strongest
        r.process(&[(0.1, 0.0), (5.0, 0.0), (0.2, 0.0)]);
        assert_eq!(r.strongest_input(), Some(1));
    }

    // 13. Auto-select routes strongest to output
    #[test]
    fn test_auto_select() {
        let mut r = RfSignalRouter::new(3, 1);
        r.set_isolation_db(200.0);
        r.add_combine(&[0, 1, 2], 0);
        // Make input 2 strongest
        for _ in 0..10 {
            r.process(&[(0.1, 0.0), (0.2, 0.0), (9.0, 0.0)]);
        }
        let chosen = r.auto_select_to_output(0);
        assert_eq!(chosen, Some(2));
        // Now only the auto-selected route + the newly added route should be active
        // The combine routes to output 0 are disabled; new route from 2->0 is added
        let out = r.process(&[(0.1, 0.0), (0.2, 0.0), (9.0, 0.0)]);
        assert!(out[0].0 > 8.0); // mostly input 2
    }

    // 14. Process block
    #[test]
    fn test_process_block() {
        let mut r = RfSignalRouter::new(2, 1);
        r.set_isolation_db(200.0);
        r.add_route(Route::new(0, 0));
        let input0 = vec![(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)];
        let input1 = vec![(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)];
        let out = r.process_block(&[input0, input1]);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].len(), 3);
        assert!(approx_eq(out[0][2].0, 3.0, 1e-6));
    }

    // 15. Bandpass filtering attenuates out-of-band
    #[test]
    fn test_bandpass_attenuation() {
        let mut r = RfSignalRouter::new(1, 1);
        r.set_isolation_db(200.0);
        // BPF centered at 1000 Hz, BW 100 Hz, Fs 8000 Hz
        r.add_route(
            Route::new(0, 0).with_bandpass(1000.0, 100.0, 8000.0),
        );

        // Feed a DC signal (0 Hz) - should be attenuated by BPF
        let mut dc_power = 0.0;
        for _ in 0..500 {
            let out = r.process(&[(1.0, 0.0)]);
            dc_power += cx_mag_sq(out[0]);
        }
        // Feed a 1000 Hz tone at Fs=8000
        r.reset_filters();
        let mut tone_power = 0.0;
        for n in 0..500 {
            let phase = 2.0 * PI * 1000.0 * n as f64 / 8000.0;
            let inp = (phase.cos(), phase.sin());
            let out = r.process(&[inp]);
            // Skip first 50 samples for transient
            if n >= 50 {
                tone_power += cx_mag_sq(out[0]);
            }
        }
        // The in-band tone should have significantly more power than DC
        assert!(
            tone_power > dc_power * 5.0,
            "In-band tone power ({tone_power}) should be much greater than DC ({dc_power})"
        );
    }

    // 16. Crossfade transition
    #[test]
    fn test_crossfade() {
        let mut r = RfSignalRouter::new(1, 1);
        r.set_isolation_db(200.0);
        r.set_crossfade_len(100);
        let idx = r.add_route(Route::new(0, 0).with_gain_linear(1.0));
        // Snapshot current gain and then change it
        r.begin_crossfade();
        r.routes[idx].gain = 0.0; // target gain = 0

        // First sample should still be close to old gain (1.0)
        let out_first = r.process(&[(1.0, 0.0)]);
        assert!(out_first[0].0 > 0.9, "First sample should be near 1.0, got {}", out_first[0].0);

        // Run remaining crossfade
        let mut last = (0.0, 0.0);
        for _ in 1..100 {
            last = r.process(&[(1.0, 0.0)])[0];
        }
        // Last sample of crossfade should be near 0.0
        assert!(last.0.abs() < 0.05, "End of crossfade should be near 0.0, got {}", last.0);
    }

    // 17. Clear routes
    #[test]
    fn test_clear_routes() {
        let mut r = RfSignalRouter::new(2, 2);
        r.add_route(Route::new(0, 0));
        r.add_route(Route::new(1, 1));
        assert_eq!(r.num_routes(), 2);
        r.clear_routes();
        assert_eq!(r.num_routes(), 0);
    }

    // 18. Average power dBFS
    #[test]
    fn test_average_power_dbfs() {
        let mut r = RfSignalRouter::new(1, 1);
        r.set_isolation_db(200.0);
        r.add_route(Route::new(0, 0).with_gain_linear(1.0));
        // Send unity amplitude -> power = 1.0 -> 0 dBFS
        for _ in 0..100 {
            r.process(&[(1.0, 0.0)]);
        }
        let dbfs = r.input_stats(0).unwrap().average_power_dbfs();
        assert!(approx_eq(dbfs, 0.0, 0.01));
    }

    // 19. Multiple routes to same output (combining)
    #[test]
    fn test_multiple_routes_same_output() {
        let mut r = RfSignalRouter::new(2, 1);
        r.set_isolation_db(200.0);
        r.add_route(Route::new(0, 0).with_gain_linear(1.0));
        r.add_route(Route::new(1, 0).with_gain_linear(1.0));
        let out = r.process(&[(1.0, 0.0), (0.0, 1.0)]);
        // Output = (1,0) + (0,1) = (1,1)
        assert!(approx_eq(out[0].0, 1.0, 1e-6));
        assert!(approx_eq(out[0].1, 1.0, 1e-6));
    }

    // 20. Complex signal with gain
    #[test]
    fn test_complex_signal_with_gain() {
        let mut r = RfSignalRouter::new(1, 1);
        r.set_isolation_db(200.0);
        r.add_route(Route::new(0, 0).with_gain_linear(2.0));
        let out = r.process(&[(3.0, -4.0)]);
        assert!(approx_eq(out[0].0, 6.0, EPS));
        assert!(approx_eq(out[0].1, -8.0, EPS));
    }

    // 21. Route accessor
    #[test]
    fn test_route_accessor() {
        let mut r = RfSignalRouter::new(2, 2);
        let idx = r.add_route(Route::new(0, 1).with_gain_db(10.0));
        let route = r.route(idx).unwrap();
        assert_eq!(route.input, 0);
        assert_eq!(route.output, 1);
        assert!(route.enabled);
        assert!(approx_eq(route.gain, 10.0_f64.powf(10.0 / 20.0), 1e-6));
    }

    // 22. Empty inputs after no processing returns None for strongest
    #[test]
    fn test_strongest_input_none() {
        let r = RfSignalRouter::new(2, 2);
        assert_eq!(r.strongest_input(), None);
    }
}

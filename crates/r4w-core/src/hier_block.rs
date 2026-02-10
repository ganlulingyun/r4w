//! # Hierarchical Block
//!
//! Composable sub-graph of DSP blocks that can be used as a single
//! processing unit. Allows building reusable composite blocks from
//! simpler primitives, similar to GNU Radio's `hier_block2`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::hier_block::{HierBlock, ProcessingStep};
//!
//! let mut hb = HierBlock::new("my_filter_chain");
//! hb.add_step(ProcessingStep::new("scale", |samples: &[f64]| {
//!     samples.iter().map(|&x| x * 2.0).collect()
//! }));
//! hb.add_step(ProcessingStep::new("offset", |samples: &[f64]| {
//!     samples.iter().map(|&x| x + 1.0).collect()
//! }));
//! let input = vec![1.0, 2.0, 3.0];
//! let output = hb.process(&input);
//! assert_eq!(output, vec![3.0, 5.0, 7.0]); // (x*2)+1
//! ```

/// A single processing step in a hierarchical block.
pub struct ProcessingStep {
    /// Step name.
    name: String,
    /// Processing function.
    func: Box<dyn Fn(&[f64]) -> Vec<f64> + Send>,
}

impl ProcessingStep {
    /// Create a new processing step.
    pub fn new<F>(name: &str, func: F) -> Self
    where
        F: Fn(&[f64]) -> Vec<f64> + Send + 'static,
    {
        Self {
            name: name.to_string(),
            func: Box::new(func),
        }
    }

    /// Get the step name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Process samples through this step.
    pub fn process(&self, input: &[f64]) -> Vec<f64> {
        (self.func)(input)
    }
}

impl std::fmt::Debug for ProcessingStep {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ProcessingStep")
            .field("name", &self.name)
            .finish()
    }
}

/// A single complex processing step.
pub struct ComplexProcessingStep {
    name: String,
    func: Box<dyn Fn(&[(f64, f64)]) -> Vec<(f64, f64)> + Send>,
}

impl ComplexProcessingStep {
    /// Create a new complex processing step.
    pub fn new<F>(name: &str, func: F) -> Self
    where
        F: Fn(&[(f64, f64)]) -> Vec<(f64, f64)> + Send + 'static,
    {
        Self {
            name: name.to_string(),
            func: Box::new(func),
        }
    }

    /// Get the step name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Process samples through this step.
    pub fn process(&self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        (self.func)(input)
    }
}

impl std::fmt::Debug for ComplexProcessingStep {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ComplexProcessingStep")
            .field("name", &self.name)
            .finish()
    }
}

/// Hierarchical block: a chain of processing steps.
#[derive(Debug)]
pub struct HierBlock {
    name: String,
    steps: Vec<ProcessingStep>,
    enabled: bool,
}

impl HierBlock {
    /// Create a new hierarchical block.
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            steps: Vec::new(),
            enabled: true,
        }
    }

    /// Add a processing step to the chain.
    pub fn add_step(&mut self, step: ProcessingStep) {
        self.steps.push(step);
    }

    /// Process samples through the entire chain.
    pub fn process(&self, input: &[f64]) -> Vec<f64> {
        if !self.enabled || self.steps.is_empty() {
            return input.to_vec();
        }
        let mut data = input.to_vec();
        for step in &self.steps {
            data = step.process(&data);
        }
        data
    }

    /// Process with intermediate results for debugging.
    pub fn process_debug(&self, input: &[f64]) -> Vec<(String, Vec<f64>)> {
        let mut results = Vec::with_capacity(self.steps.len() + 1);
        results.push(("input".to_string(), input.to_vec()));
        if !self.enabled {
            return results;
        }
        let mut data = input.to_vec();
        for step in &self.steps {
            data = step.process(&data);
            results.push((step.name().to_string(), data.clone()));
        }
        results
    }

    /// Get block name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get number of steps.
    pub fn num_steps(&self) -> usize {
        self.steps.len()
    }

    /// Get step names.
    pub fn step_names(&self) -> Vec<&str> {
        self.steps.iter().map(|s| s.name()).collect()
    }

    /// Enable or disable the block (disabled = passthrough).
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Check if enabled.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
}

/// Hierarchical block for complex samples.
#[derive(Debug)]
pub struct HierBlockComplex {
    name: String,
    steps: Vec<ComplexProcessingStep>,
    enabled: bool,
}

impl HierBlockComplex {
    /// Create a new complex hierarchical block.
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            steps: Vec::new(),
            enabled: true,
        }
    }

    /// Add a complex processing step.
    pub fn add_step(&mut self, step: ComplexProcessingStep) {
        self.steps.push(step);
    }

    /// Process complex samples through the chain.
    pub fn process(&self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if !self.enabled || self.steps.is_empty() {
            return input.to_vec();
        }
        let mut data = input.to_vec();
        for step in &self.steps {
            data = step.process(&data);
        }
        data
    }

    /// Get block name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get number of steps.
    pub fn num_steps(&self) -> usize {
        self.steps.len()
    }

    /// Enable or disable.
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
}

/// Build a common gain + offset chain.
pub fn gain_offset_block(name: &str, gain: f64, offset: f64) -> HierBlock {
    let mut hb = HierBlock::new(name);
    hb.add_step(ProcessingStep::new("gain", move |s: &[f64]| {
        s.iter().map(|&x| x * gain).collect()
    }));
    hb.add_step(ProcessingStep::new("offset", move |s: &[f64]| {
        s.iter().map(|&x| x + offset).collect()
    }));
    hb
}

/// Build a clip + scale chain.
pub fn clip_scale_block(name: &str, min: f64, max: f64, scale: f64) -> HierBlock {
    let mut hb = HierBlock::new(name);
    hb.add_step(ProcessingStep::new("clip", move |s: &[f64]| {
        s.iter().map(|&x| x.clamp(min, max)).collect()
    }));
    hb.add_step(ProcessingStep::new("scale", move |s: &[f64]| {
        s.iter().map(|&x| x * scale).collect()
    }));
    hb
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_single_step() {
        let mut hb = HierBlock::new("test");
        hb.add_step(ProcessingStep::new("double", |s: &[f64]| {
            s.iter().map(|&x| x * 2.0).collect()
        }));
        let output = hb.process(&[1.0, 2.0, 3.0]);
        assert_eq!(output, vec![2.0, 4.0, 6.0]);
    }

    #[test]
    fn test_chain() {
        let mut hb = HierBlock::new("chain");
        hb.add_step(ProcessingStep::new("add1", |s: &[f64]| {
            s.iter().map(|&x| x + 1.0).collect()
        }));
        hb.add_step(ProcessingStep::new("mul3", |s: &[f64]| {
            s.iter().map(|&x| x * 3.0).collect()
        }));
        let output = hb.process(&[1.0, 2.0]);
        assert_eq!(output, vec![6.0, 9.0]); // (1+1)*3=6, (2+1)*3=9
    }

    #[test]
    fn test_empty_block() {
        let hb = HierBlock::new("empty");
        let output = hb.process(&[1.0, 2.0]);
        assert_eq!(output, vec![1.0, 2.0]); // Passthrough.
    }

    #[test]
    fn test_disabled() {
        let mut hb = HierBlock::new("test");
        hb.add_step(ProcessingStep::new("double", |s: &[f64]| {
            s.iter().map(|&x| x * 2.0).collect()
        }));
        hb.set_enabled(false);
        let output = hb.process(&[1.0, 2.0]);
        assert_eq!(output, vec![1.0, 2.0]); // Passthrough when disabled.
    }

    #[test]
    fn test_process_debug() {
        let mut hb = HierBlock::new("debug");
        hb.add_step(ProcessingStep::new("step1", |s: &[f64]| {
            s.iter().map(|&x| x + 10.0).collect()
        }));
        hb.add_step(ProcessingStep::new("step2", |s: &[f64]| {
            s.iter().map(|&x| x * 2.0).collect()
        }));
        let debug = hb.process_debug(&[1.0]);
        assert_eq!(debug.len(), 3); // input + 2 steps
        assert_eq!(debug[0].0, "input");
        assert_eq!(debug[0].1, vec![1.0]);
        assert_eq!(debug[1].0, "step1");
        assert_eq!(debug[1].1, vec![11.0]);
        assert_eq!(debug[2].0, "step2");
        assert_eq!(debug[2].1, vec![22.0]);
    }

    #[test]
    fn test_gain_offset_block() {
        let hb = gain_offset_block("go", 3.0, -1.0);
        let output = hb.process(&[1.0, 2.0]);
        assert_eq!(output, vec![2.0, 5.0]); // 1*3-1=2, 2*3-1=5
    }

    #[test]
    fn test_clip_scale_block() {
        let hb = clip_scale_block("cs", -1.0, 1.0, 100.0);
        let output = hb.process(&[-5.0, 0.5, 5.0]);
        assert_eq!(output, vec![-100.0, 50.0, 100.0]);
    }

    #[test]
    fn test_step_names() {
        let mut hb = HierBlock::new("test");
        hb.add_step(ProcessingStep::new("a", |s: &[f64]| s.to_vec()));
        hb.add_step(ProcessingStep::new("b", |s: &[f64]| s.to_vec()));
        assert_eq!(hb.step_names(), vec!["a", "b"]);
        assert_eq!(hb.num_steps(), 2);
    }

    #[test]
    fn test_complex_hier_block() {
        let mut hb = HierBlockComplex::new("iq_chain");
        hb.add_step(ComplexProcessingStep::new(
            "conj",
            |s: &[(f64, f64)]| s.iter().map(|&(re, im)| (re, -im)).collect(),
        ));
        let output = hb.process(&[(1.0, 2.0), (3.0, 4.0)]);
        assert_eq!(output, vec![(1.0, -2.0), (3.0, -4.0)]);
    }

    #[test]
    fn test_empty_input() {
        let mut hb = HierBlock::new("test");
        hb.add_step(ProcessingStep::new("x2", |s: &[f64]| {
            s.iter().map(|&x| x * 2.0).collect()
        }));
        let output = hb.process(&[]);
        assert!(output.is_empty());
    }
}

//! # Sample Counter
//!
//! Counts samples passing through a pipeline, optionally tagging
//! milestones. Useful for debugging, alignment verification, and
//! ensuring correct sample counts through rate-changing blocks.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::sample_counter::SampleCounter;
//!
//! let mut counter = SampleCounter::new("tx_out");
//! counter.count(1024);
//! counter.count(1024);
//! assert_eq!(counter.total(), 2048);
//! ```

/// Sample counter with milestone tracking.
#[derive(Debug, Clone)]
pub struct SampleCounter {
    name: String,
    total: u64,
    window_count: u64,
    window_size: u64,
    milestones: Vec<(u64, String)>,
}

impl SampleCounter {
    /// Create a new sample counter.
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            total: 0,
            window_count: 0,
            window_size: 0,
            milestones: Vec::new(),
        }
    }

    /// Create with window-based counting.
    pub fn with_window(name: &str, window_size: u64) -> Self {
        Self {
            name: name.to_string(),
            total: 0,
            window_count: 0,
            window_size,
            milestones: Vec::new(),
        }
    }

    /// Count N samples.
    pub fn count(&mut self, n: u64) {
        self.total += n;
        if self.window_size > 0 {
            self.window_count += n;
        }
    }

    /// Count and pass through data (for pipeline insertion).
    pub fn pass_through<'a>(&mut self, data: &'a [f64]) -> &'a [f64] {
        self.count(data.len() as u64);
        data
    }

    /// Count and pass through complex data.
    pub fn pass_through_complex<'a>(&mut self, data: &'a [(f64, f64)]) -> &'a [(f64, f64)] {
        self.count(data.len() as u64);
        data
    }

    /// Get total samples counted.
    pub fn total(&self) -> u64 {
        self.total
    }

    /// Get counter name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Check if a window boundary was crossed, returns the number of windows completed.
    pub fn check_window(&mut self) -> u64 {
        if self.window_size == 0 {
            return 0;
        }
        let windows = self.window_count / self.window_size;
        if windows > 0 {
            self.window_count %= self.window_size;
        }
        windows
    }

    /// Add a milestone at the current count.
    pub fn mark(&mut self, label: &str) {
        self.milestones.push((self.total, label.to_string()));
    }

    /// Get all milestones.
    pub fn milestones(&self) -> &[(u64, String)] {
        &self.milestones
    }

    /// Get samples since last milestone.
    pub fn since_last_milestone(&self) -> u64 {
        if let Some((last, _)) = self.milestones.last() {
            self.total - last
        } else {
            self.total
        }
    }

    /// Reset counter.
    pub fn reset(&mut self) {
        self.total = 0;
        self.window_count = 0;
        self.milestones.clear();
    }
}

/// Multi-point sample counter for pipeline debugging.
#[derive(Debug, Clone)]
pub struct PipelineCounters {
    counters: Vec<SampleCounter>,
}

impl PipelineCounters {
    /// Create a new pipeline counter set.
    pub fn new() -> Self {
        Self {
            counters: Vec::new(),
        }
    }

    /// Add a named counter.
    pub fn add(&mut self, name: &str) -> usize {
        let idx = self.counters.len();
        self.counters.push(SampleCounter::new(name));
        idx
    }

    /// Count samples at a specific point.
    pub fn count(&mut self, index: usize, n: u64) {
        if let Some(c) = self.counters.get_mut(index) {
            c.count(n);
        }
    }

    /// Get all counters.
    pub fn counters(&self) -> &[SampleCounter] {
        &self.counters
    }

    /// Get a summary of all counts.
    pub fn summary(&self) -> Vec<(&str, u64)> {
        self.counters.iter().map(|c| (c.name(), c.total())).collect()
    }

    /// Check if all counters have the same total (pipeline integrity check).
    pub fn all_equal(&self) -> bool {
        if self.counters.is_empty() {
            return true;
        }
        let first = self.counters[0].total();
        self.counters.iter().all(|c| c.total() == first)
    }

    /// Get the ratio between two counters (for rate-change verification).
    pub fn ratio(&self, a: usize, b: usize) -> Option<f64> {
        let ca = self.counters.get(a)?;
        let cb = self.counters.get(b)?;
        if cb.total() == 0 {
            return None;
        }
        Some(ca.total() as f64 / cb.total() as f64)
    }

    /// Reset all counters.
    pub fn reset(&mut self) {
        for c in &mut self.counters {
            c.reset();
        }
    }
}

impl Default for PipelineCounters {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_count() {
        let mut c = SampleCounter::new("test");
        c.count(100);
        c.count(200);
        assert_eq!(c.total(), 300);
        assert_eq!(c.name(), "test");
    }

    #[test]
    fn test_pass_through() {
        let mut c = SampleCounter::new("pt");
        let data = vec![1.0, 2.0, 3.0];
        let out = c.pass_through(&data);
        assert_eq!(out.len(), 3);
        assert_eq!(c.total(), 3);
    }

    #[test]
    fn test_pass_through_complex() {
        let mut c = SampleCounter::new("ptc");
        let data = vec![(1.0, 2.0), (3.0, 4.0)];
        let out = c.pass_through_complex(&data);
        assert_eq!(out.len(), 2);
        assert_eq!(c.total(), 2);
    }

    #[test]
    fn test_window() {
        let mut c = SampleCounter::with_window("w", 100);
        c.count(50);
        assert_eq!(c.check_window(), 0);
        c.count(60);
        assert_eq!(c.check_window(), 1);
        // Remaining should be 10.
        c.count(95);
        assert_eq!(c.check_window(), 1);
    }

    #[test]
    fn test_milestones() {
        let mut c = SampleCounter::new("m");
        c.count(100);
        c.mark("start");
        c.count(200);
        c.mark("middle");
        c.count(50);
        assert_eq!(c.milestones().len(), 2);
        assert_eq!(c.milestones()[0], (100, "start".to_string()));
        assert_eq!(c.since_last_milestone(), 50); // 350 - 300
    }

    #[test]
    fn test_reset() {
        let mut c = SampleCounter::new("r");
        c.count(500);
        c.mark("m1");
        c.reset();
        assert_eq!(c.total(), 0);
        assert!(c.milestones().is_empty());
    }

    #[test]
    fn test_pipeline_counters() {
        let mut pc = PipelineCounters::new();
        let a = pc.add("input");
        let b = pc.add("output");
        pc.count(a, 1000);
        pc.count(b, 1000);
        assert!(pc.all_equal());
    }

    #[test]
    fn test_pipeline_ratio() {
        let mut pc = PipelineCounters::new();
        let a = pc.add("before_decim");
        let b = pc.add("after_decim");
        pc.count(a, 1000);
        pc.count(b, 250);
        let ratio = pc.ratio(a, b).unwrap();
        assert!((ratio - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_pipeline_summary() {
        let mut pc = PipelineCounters::new();
        pc.add("tx");
        pc.add("rx");
        pc.count(0, 100);
        pc.count(1, 200);
        let summary = pc.summary();
        assert_eq!(summary.len(), 2);
        assert_eq!(summary[0], ("tx", 100));
        assert_eq!(summary[1], ("rx", 200));
    }

    #[test]
    fn test_pipeline_reset() {
        let mut pc = PipelineCounters::new();
        pc.add("a");
        pc.count(0, 500);
        pc.reset();
        assert_eq!(pc.counters()[0].total(), 0);
    }
}

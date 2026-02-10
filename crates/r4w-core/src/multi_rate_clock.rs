//! # Multi-Rate Clock
//!
//! Clock divider and multiplier for synchronizing different sample rate
//! domains in an SDR processing chain. Generates tick events at derived
//! rates for scheduling sample processing across domains.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::multi_rate_clock::MultiRateClock;
//!
//! let mut clock = MultiRateClock::new(48000.0);
//! clock.add_derived_rate("baseband", 12000.0);
//! clock.add_derived_rate("symbol", 4800.0);
//!
//! // Advance by one master sample
//! let ticks = clock.tick();
//! // baseband ticks every 4 master samples, symbol every 10
//! ```

/// A derived clock rate.
#[derive(Debug, Clone)]
struct DerivedClock {
    name: String,
    rate: f64,
    /// Accumulator for fractional tick tracking.
    accumulator: f64,
    /// Step per master tick.
    step: f64,
    /// Total ticks generated.
    tick_count: u64,
}

/// Multi-rate clock manager.
#[derive(Debug, Clone)]
pub struct MultiRateClock {
    /// Master clock rate (Hz).
    master_rate: f64,
    /// Derived clocks.
    derived: Vec<DerivedClock>,
    /// Master tick count.
    master_ticks: u64,
}

/// Events generated on a tick.
#[derive(Debug, Clone)]
pub struct TickEvents {
    /// Which derived clocks ticked (by name).
    pub ticked: Vec<String>,
    /// Master tick count.
    pub master_tick: u64,
}

impl MultiRateClock {
    /// Create a new multi-rate clock with the given master rate.
    pub fn new(master_rate: f64) -> Self {
        Self {
            master_rate,
            derived: Vec::new(),
            master_ticks: 0,
        }
    }

    /// Add a derived clock at the given rate.
    pub fn add_derived_rate(&mut self, name: &str, rate: f64) {
        let step = rate / self.master_rate;
        self.derived.push(DerivedClock {
            name: name.to_string(),
            rate,
            accumulator: step, // Pre-load so first tick fires at correct time
            step,
            tick_count: 0,
        });
    }

    /// Advance the master clock by one sample and return tick events.
    pub fn tick(&mut self) -> TickEvents {
        self.master_ticks += 1;
        let mut ticked = Vec::new();

        for clock in &mut self.derived {
            clock.accumulator += clock.step;
            if clock.accumulator >= 1.0 {
                clock.accumulator -= 1.0;
                clock.tick_count += 1;
                ticked.push(clock.name.clone());
            }
        }

        TickEvents {
            ticked,
            master_tick: self.master_ticks,
        }
    }

    /// Advance by N master samples and return all tick events.
    pub fn tick_n(&mut self, n: usize) -> Vec<TickEvents> {
        let mut events = Vec::with_capacity(n);
        for _ in 0..n {
            events.push(self.tick());
        }
        events
    }

    /// Get the master clock rate.
    pub fn master_rate(&self) -> f64 {
        self.master_rate
    }

    /// Get the total master tick count.
    pub fn master_ticks(&self) -> u64 {
        self.master_ticks
    }

    /// Get the tick count for a derived clock.
    pub fn derived_tick_count(&self, name: &str) -> Option<u64> {
        self.derived
            .iter()
            .find(|c| c.name == name)
            .map(|c| c.tick_count)
    }

    /// Get the derived rate for a named clock.
    pub fn derived_rate(&self, name: &str) -> Option<f64> {
        self.derived.iter().find(|c| c.name == name).map(|c| c.rate)
    }

    /// Get the decimation ratio for a derived clock (master/derived).
    pub fn decimation_ratio(&self, name: &str) -> Option<f64> {
        self.derived_rate(name).map(|r| self.master_rate / r)
    }

    /// Get the number of derived clocks.
    pub fn num_derived(&self) -> usize {
        self.derived.len()
    }

    /// Reset all clocks.
    pub fn reset(&mut self) {
        self.master_ticks = 0;
        for clock in &mut self.derived {
            clock.accumulator = clock.step;
            clock.tick_count = 0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_integer_division() {
        let mut clock = MultiRateClock::new(48000.0);
        clock.add_derived_rate("bb", 12000.0); // divide by 4

        let events = clock.tick_n(48);
        let bb_ticks: usize = events.iter().filter(|e| e.ticked.contains(&"bb".to_string())).count();
        assert_eq!(bb_ticks, 12, "12kHz should tick 12 times in 48 master ticks");
    }

    #[test]
    fn test_non_integer_division() {
        let mut clock = MultiRateClock::new(48000.0);
        clock.add_derived_rate("sym", 4800.0); // divide by 10

        let events = clock.tick_n(100);
        let sym_ticks: usize = events.iter().filter(|e| e.ticked.contains(&"sym".to_string())).count();
        assert_eq!(sym_ticks, 10);
    }

    #[test]
    fn test_multiple_derived() {
        let mut clock = MultiRateClock::new(48000.0);
        clock.add_derived_rate("fast", 24000.0);
        clock.add_derived_rate("slow", 8000.0);

        let events = clock.tick_n(48);
        let fast: usize = events.iter().filter(|e| e.ticked.contains(&"fast".to_string())).count();
        let slow: usize = events.iter().filter(|e| e.ticked.contains(&"slow".to_string())).count();
        assert_eq!(fast, 24);
        assert_eq!(slow, 8);
    }

    #[test]
    fn test_master_ticks() {
        let mut clock = MultiRateClock::new(48000.0);
        clock.tick_n(100);
        assert_eq!(clock.master_ticks(), 100);
    }

    #[test]
    fn test_derived_tick_count() {
        let mut clock = MultiRateClock::new(48000.0);
        clock.add_derived_rate("bb", 12000.0);
        clock.tick_n(48);
        assert_eq!(clock.derived_tick_count("bb"), Some(12));
        assert_eq!(clock.derived_tick_count("nonexistent"), None);
    }

    #[test]
    fn test_decimation_ratio() {
        let mut clock = MultiRateClock::new(48000.0);
        clock.add_derived_rate("bb", 12000.0);
        assert!((clock.decimation_ratio("bb").unwrap() - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut clock = MultiRateClock::new(48000.0);
        clock.add_derived_rate("bb", 12000.0);
        clock.tick_n(100);
        assert!(clock.master_ticks() > 0);
        clock.reset();
        assert_eq!(clock.master_ticks(), 0);
        assert_eq!(clock.derived_tick_count("bb"), Some(0));
    }

    #[test]
    fn test_num_derived() {
        let mut clock = MultiRateClock::new(48000.0);
        assert_eq!(clock.num_derived(), 0);
        clock.add_derived_rate("a", 12000.0);
        clock.add_derived_rate("b", 8000.0);
        assert_eq!(clock.num_derived(), 2);
    }

    #[test]
    fn test_long_run_accuracy() {
        // Over 48000 master ticks, a 1000 Hz derived clock should tick ~1000 times.
        let mut clock = MultiRateClock::new(48000.0);
        clock.add_derived_rate("hz1k", 1000.0);
        clock.tick_n(48000);
        let ticks = clock.derived_tick_count("hz1k").unwrap();
        assert_eq!(ticks, 1000, "1kHz should tick exactly 1000 times in 48000 samples");
    }

    #[test]
    fn test_derived_rate() {
        let mut clock = MultiRateClock::new(48000.0);
        clock.add_derived_rate("bb", 12000.0);
        assert!((clock.derived_rate("bb").unwrap() - 12000.0).abs() < 1e-10);
    }
}

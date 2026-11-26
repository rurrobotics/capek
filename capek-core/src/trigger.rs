//! Sensor triggers: when to react to readings.
//!
//! A sensor task checks its triggers after every reading and runs the
//! callback of each one that fires. A [`Trigger`] says when to fire; a
//! [`Watcher`] remembers what it needs between readings to decide.
//!
//! - [`Below`](Trigger::Below) and [`Above`](Trigger::Above) fire once when
//!   the value crosses the threshold, not on every sample. Using a lower
//!   `Below` and a higher `Above` gives hysteresis. The first reading counts
//!   as a crossing, so an obstacle already there at start-up fires `Below`.
//! - [`Changed`](Trigger::Changed) fires when a reading differs from the one
//!   before.
//! - [`Error`](Trigger::Error) fires once when the sensor stops returning
//!   readings, and again only after it has returned one.
//! - [`Every`](Trigger::Every) fires on every reading.
//!
//! A missing reading leaves the thresholds as they were, so a dropout in the
//! middle does not make `Below` fire again.
//!
//! # Examples
//!
//! Stop below 60 mm, start again above 80 mm:
//!
//! ```
//! use capek_core::trigger::{Trigger, Watcher};
//! use capek_core::units::UnitExt;
//!
//! let mut stop = Watcher::new(Trigger::Below(60.mm()));
//! let mut start = Watcher::new(Trigger::Above(80.mm()));
//!
//! let readings = [200.mm(), 70.mm(), 55.mm(), 50.mm(), 70.mm(), 90.mm(), 95.mm()];
//! let fired: Vec<_> = readings
//!     .iter()
//!     .map(|&d| (stop.check(Some(d)), start.check(Some(d))))
//!     .collect();
//! assert_eq!(
//!     fired,
//!     [
//!         (false, true), // 200: above at start-up
//!         (false, false),
//!         (true, false), // 55: crossed below
//!         (false, false),
//!         (false, false), // 70: between the thresholds
//!         (false, true), // 90: crossed above
//!         (false, false),
//!     ]
//! );
//! ```

/// When a sensor reacts to its readings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Trigger<Q> {
    /// When the value drops below the threshold.
    Below(Q),
    /// When the value rises above the threshold.
    Above(Q),
    /// When the value differs from the previous reading.
    Changed,
    /// When the sensor stops returning readings.
    Error,
    /// On every reading.
    Every,
}

/// Decides, reading by reading, whether a [`Trigger`] fires.
///
/// ```
/// use capek_core::trigger::{Trigger, Watcher};
///
/// let mut error = Watcher::new(Trigger::<f32>::Error);
/// assert!(error.check(None)); // the sensor stopped answering
/// assert!(!error.check(None)); // still not, but already reported
/// assert!(!error.check(Some(1.0)));
/// assert!(error.check(None)); // stopped again
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Watcher<Q> {
    trigger: Trigger<Q>,
    /// For a threshold, whether the value is past it; for `Error`, whether
    /// the sensor is failing.
    past: bool,
    /// The previous reading, for `Changed`.
    last: Option<Q>,
}

impl<Q: PartialOrd + Copy> Watcher<Q> {
    /// A watcher for `trigger`, before any reading.
    pub fn new(trigger: Trigger<Q>) -> Self {
        Self {
            trigger,
            past: false,
            last: None,
        }
    }

    /// The trigger it watches for.
    pub fn trigger(&self) -> Trigger<Q> {
        self.trigger
    }

    /// Takes the next reading, `None` when the sensor gave none, and returns
    /// whether the trigger fires.
    pub fn check(&mut self, reading: Option<Q>) -> bool {
        match (self.trigger, reading) {
            (Trigger::Error, None) => !core::mem::replace(&mut self.past, true),
            (Trigger::Error, Some(_)) => {
                self.past = false;
                false
            }
            (_, None) => false,
            (Trigger::Below(t), Some(v)) => self.cross(v < t),
            (Trigger::Above(t), Some(v)) => self.cross(v > t),
            (Trigger::Changed, Some(v)) => self.last.replace(v) != Some(v),
            (Trigger::Every, Some(_)) => true,
        }
    }

    /// Fires when the value has just moved past the threshold.
    fn cross(&mut self, past: bool) -> bool {
        let fires = past && !self.past;
        self.past = past;
        fires
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(trigger: Trigger<f32>, readings: &[Option<f32>]) -> std::vec::Vec<bool> {
        let mut w = Watcher::new(trigger);
        readings.iter().map(|&r| w.check(r)).collect()
    }

    #[test]
    fn below_fires_once_per_crossing() {
        let r = [
            Some(10.0),
            Some(4.0),
            Some(3.0),
            Some(6.0),
            Some(5.0),
            Some(2.0),
        ];
        assert_eq!(
            run(Trigger::Below(5.0), &r),
            [false, true, false, false, false, true]
        );
    }

    #[test]
    fn above_fires_once_per_crossing() {
        let r = [Some(1.0), Some(6.0), Some(7.0), Some(5.0), Some(8.0)];
        assert_eq!(
            run(Trigger::Above(5.0), &r),
            [false, true, false, false, true]
        );
    }

    #[test]
    fn first_reading_counts_as_a_crossing() {
        assert_eq!(run(Trigger::Below(5.0), &[Some(1.0)]), [true]);
        assert_eq!(run(Trigger::Above(5.0), &[Some(9.0)]), [true]);
    }

    #[test]
    fn dropouts_keep_threshold_state() {
        let r = [Some(1.0), None, None, Some(2.0), Some(9.0), None, Some(1.0)];
        assert_eq!(
            run(Trigger::Below(5.0), &r),
            [true, false, false, false, false, false, true]
        );
    }

    #[test]
    fn error_fires_once_per_outage() {
        let r = [Some(1.0), None, None, Some(2.0), None];
        assert_eq!(run(Trigger::Error, &r), [false, true, false, false, true]);
    }

    #[test]
    fn changed_ignores_repeats_and_dropouts() {
        let r = [Some(1.0), Some(1.0), None, Some(1.0), Some(2.0)];
        assert_eq!(run(Trigger::Changed, &r), [true, false, false, false, true]);
    }

    #[test]
    fn every_fires_on_each_reading() {
        let r = [Some(1.0), None, Some(1.0)];
        assert_eq!(run(Trigger::Every, &r), [true, false, true]);
    }
}

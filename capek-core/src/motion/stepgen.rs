//! Step generation: from axis positions to the timing of every motor step.
//!
//! [`StepGen`] is fed the position of every axis once per time slice, usually
//! 1 ms. For each axis it works out which steps fall within the slice and
//! exactly when, by interpolating between the positions at the start and end
//! of the slice, and emits one [`StepWord`] per step.
//!
//! - A word is emitted only when a step happens, so slow motion accumulates
//!   time across slices.
//! - Positions are whole steps with the remainder carried, and each delay is
//!   measured from when the previous step was actually emitted, after
//!   rounding to a tick. Nothing drifts, however long the move.
//! - All axes advance by the same slice, so they stay synchronised in time on
//!   curves.
//!
//! # Examples
//!
//! Drive a planned move along a path and collect the steps of each wheel:
//!
//! ```
//! use capek_core::driver::StepWord;
//! use capek_core::motion::path::{Line, Path};
//! use capek_core::motion::planner::{plan_speeds, Span, Trajectory};
//! use capek_core::motion::profile::{State, Trapezoid};
//! use capek_core::motion::stepgen::StepGen;
//! use capek_core::units::UnitExt;
//! use core::time::Duration;
//!
//! let path = Line(100.mm());
//! let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
//! let spans = [Span::new(Path::<2>::length(&path))];
//! let pieces = plan_speeds(&profile, &spans, 0.mm_per_s(), 0.mm_per_s());
//! let mut trajectory = Trajectory::new(profile, pieces, State::REST);
//!
//! // One wheel step is 0.1 mm; the driver's timer runs at 4 MHz.
//! let mut stepgen = StepGen::new([0.1.mm(); 2], 4_000_000);
//! let mut words: [Vec<StepWord>; 2] = Default::default();
//! let dt = Duration::from_millis(1);
//! while !trajectory.is_done() {
//!     let state = trajectory.advance(dt);
//!     stepgen.slice(path.axes(state.s), dt, |axis, word| words[axis].push(word));
//! }
//! assert_eq!(words[0].len(), 1000);
//! assert_eq!(stepgen.steps(), [1000, 1000]);
//! ```

use crate::driver::StepWord;
use crate::units::Length;
use core::time::Duration;

/// Turns axis positions, sampled once per time slice, into step timings.
///
/// ```
/// use capek_core::motion::stepgen::StepGen;
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// // One axis, 0.1 mm per step, a 1 MHz timer.
/// let mut stepgen = StepGen::new([0.1.mm()], 1_000_000);
/// let mut delays = Vec::new();
/// // 1 mm in 10 ms: one step per millisecond.
/// for i in 1..=10 {
///     let position = (i as f32 * 0.1).mm();
///     stepgen.slice([position], Duration::from_millis(1), |_, w| delays.push(w.delay()));
/// }
/// // The first step falls half a step in, the rest one step apart.
/// assert_eq!(delays[0], 500);
/// assert!(delays[1..].iter().all(|&d| d == 1000));
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct StepGen<const N: usize> {
    /// Distance of one step on each axis, mm.
    step: [f32; N],
    /// Timer ticks per second.
    ticks_per_second: f32,
    /// Position at the end of the last slice, in steps from the origin.
    target: [f32; N],
    /// Steps emitted, net of direction, from the origin.
    count: [i32; N],
    /// When the last step on each axis was emitted, in ticks from the start
    /// of the next slice. Never positive once a slice ends.
    last: [f32; N],
    /// Position of the origin of the current move, in steps from where the
    /// emitted steps put each axis: the remainder carried between moves.
    base: [f32; N],
}

impl<const N: usize> StepGen<N> {
    /// A generator for axes whose steps move `step` each, for a driver whose
    /// timer runs at `ticks_per_second`.
    ///
    /// All axes start at zero, and the first delay on each axis counts from
    /// the start of the first slice.
    pub fn new(step: [Length; N], ticks_per_second: u32) -> Self {
        Self {
            step: step.map(|s| s.as_mm()),
            ticks_per_second: ticks_per_second as f32,
            target: [0.0; N],
            count: [0; N],
            last: [0.0; N],
            base: [0.0; N],
        }
    }

    /// Advances by one slice of `dt`, at the end of which each axis is at
    /// `axes`, relative to the start of the current move.
    ///
    /// Calls `emit` with the axis index and the word for every step in the
    /// slice, in order of time for each axis.
    pub fn slice(
        &mut self,
        axes: [Length; N],
        dt: Duration,
        mut emit: impl FnMut(usize, StepWord),
    ) {
        let ticks = dt.as_secs_f32() * self.ticks_per_second;
        for (k, axis) in axes.iter().enumerate() {
            let p0 = self.target[k];
            let p1 = self.base[k] + axis.as_mm() / self.step[k];
            // An axis has made step n once it is past n − 0.5.
            let wanted = libm::floorf(p1 + 0.5) as i32;
            let mut n = self.count[k];
            while n != wanted {
                let forward = wanted > n;
                let boundary = if forward {
                    n as f32 + 0.5
                } else {
                    n as f32 - 0.5
                };
                // When within the slice the axis crosses the boundary.
                let f = if p1 != p0 {
                    ((boundary - p0) / (p1 - p0)).clamp(0.0, 1.0)
                } else {
                    1.0
                };
                let delay = libm::roundf(f * ticks - self.last[k]).max(0.0) as u32;
                let word = StepWord::new(forward, delay);
                emit(k, word);
                self.last[k] += word.delay() as f32;
                n += if forward { 1 } else { -1 };
            }
            self.count[k] = n;
            self.target[k] = p1;
            // Measure from the next slice's start, and keep long pauses within
            // what a word can hold.
            self.last[k] = (self.last[k] - ticks).max(-(StepWord::MAX_DELAY as f32));
        }
    }

    /// Steps emitted on each axis since the start of the current move, net of
    /// direction.
    pub fn steps(&self) -> [i32; N] {
        self.count
    }

    /// Starts a new move whose axes start at zero.
    ///
    /// The part of a step each axis had moved past its last emitted step is
    /// carried into the new move, so that a sequence of moves does not drift.
    ///
    /// ```
    /// use capek_core::motion::stepgen::StepGen;
    /// use capek_core::units::UnitExt;
    /// use core::time::Duration;
    ///
    /// let mut stepgen = StepGen::new([1.mm()], 1_000_000);
    /// let dt = Duration::from_millis(1);
    /// // Two moves of 0.3 steps: neither makes a step on its own...
    /// stepgen.slice([0.3.mm()], dt, |_, _| unreachable!());
    /// stepgen.rebase();
    /// // ...but together they pass half a step.
    /// let mut steps = 0;
    /// stepgen.slice([0.3.mm()], dt, |_, _| steps += 1);
    /// assert_eq!(steps, 1);
    /// ```
    pub fn rebase(&mut self) {
        for k in 0..N {
            self.base[k] = self.target[k] - self.count[k] as f32;
            self.target[k] = self.base[k];
            self.count[k] = 0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::kinematics::{Differential, Pose};
    use crate::motion::path::{Line, Path};
    use crate::motion::planner::{Trajectory, plan_speeds};
    use crate::motion::profile::{Profile, SCurve, State};
    use crate::motion::route::{Route, RoutePath, Waypoint};
    use crate::units::UnitExt;

    const DT: Duration = Duration::from_millis(1);
    const TICKS: u32 = 4_000_000;

    /// Runs `path` with `profile` from rest to rest and collects every word.
    fn generate<P: Profile, T: Path<2>>(
        profile: P,
        path: &T,
        spans: &[crate::motion::planner::Span],
        step: Length,
    ) -> (StepGen<2>, [std::vec::Vec<StepWord>; 2], f32) {
        let pieces = plan_speeds(&profile, spans, 0.mm_per_s(), 0.mm_per_s());
        let mut trajectory = Trajectory::new(profile, pieces, State::REST);
        let mut stepgen = StepGen::new([step; 2], TICKS);
        let mut words: [std::vec::Vec<StepWord>; 2] = Default::default();
        let mut slices = 0u32;
        while !trajectory.is_done() {
            let state = trajectory.advance(DT);
            stepgen.slice(path.axes(state.s), DT, |k, w| words[k].push(w));
            slices += 1;
        }
        (stepgen, words, (DT * slices).as_secs_f32())
    }

    fn net(words: &[StepWord]) -> i32 {
        words.iter().map(|w| if w.forward() { 1 } else { -1 }).sum()
    }

    #[test]
    fn constant_speed_gives_even_steps() {
        // 100 mm/s with 0.1 mm steps: one step per millisecond, 4000 ticks.
        let mut stepgen = StepGen::new([0.1.mm()], TICKS);
        let mut words = std::vec::Vec::new();
        for i in 1..=100 {
            let s = Length::mm(i as f32 * 0.1);
            stepgen.slice([s], DT, |_, w| words.push(w));
        }
        assert_eq!(words.len(), 100);
        assert_eq!(words[0].delay(), 2000);
        assert!(words[1..].iter().all(|w| w.delay() == 4000 && w.forward()));
    }

    #[test]
    fn slow_motion_accumulates_across_slices() {
        // One step every 10 ms, sampled every millisecond.
        let mut stepgen = StepGen::new([1.mm()], TICKS);
        let mut words = std::vec::Vec::new();
        for i in 1..=100 {
            let s = Length::mm(i as f32 * 0.1);
            stepgen.slice([s], DT, |_, w| words.push(w));
        }
        assert_eq!(words.len(), 10);
        assert!(words[1..].iter().all(|w| w.delay() == 40_000), "{words:?}");
    }

    #[test]
    fn step_totals_are_exact_on_a_route() {
        let drive = Differential::new(56.mm(), 159.mm());
        let route = Route::from_slice(&[
            Waypoint::with_slope(600.mm(), 0.mm(), 0.deg()),
            Waypoint::with_slope(600.mm(), 300.mm(), 180.deg()),
            Waypoint::at(0.mm(), 300.mm()),
            Waypoint::at(0.mm(), 600.mm()),
        ])
        .unwrap();
        let path = RoutePath::new(Pose::ORIGIN, &route, &drive);
        let profile = SCurve {
            max_speed: 400.mm_per_s(),
            accel: 1200.mm_per_s2(),
            jerk: 8000.mm_per_s3(),
        };
        // One sixteenth microstep of a 400-step motor on a 56 mm wheel.
        let step = drive.wheel_distance(360.deg()) / (400.0 * 16.0);
        let spans = path.spans(&profile);
        let (stepgen, words, total) = generate(profile, &path, &spans, step);

        let end = path.end();
        for k in 0..2 {
            let expected = libm::roundf(end[k].as_mm() / step.as_mm()) as i32;
            assert_eq!(stepgen.steps()[k], expected, "axis {k}");
            assert_eq!(net(&words[k]), expected, "axis {k}");
            // The delays add up to when the last step happened.
            let span: u64 = words[k].iter().map(|w| w.delay() as u64).sum();
            let last_step = span as f32 / TICKS as f32;
            assert!(
                last_step <= total + 1e-3,
                "axis {k}: {last_step} vs {total}"
            );
        }
        // On the U-turn the wheels go different distances; on the in-place
        // turn they go opposite ways.
        assert_ne!(stepgen.steps()[0], stepgen.steps()[1]);
        assert!(words[0].iter().any(|w| !w.forward()) || words[1].iter().any(|w| !w.forward()));
    }

    #[test]
    fn long_moves_do_not_drift() {
        let step = 0.0275.mm();
        let expected = libm::roundf(10_000.0 / 0.0275) as i32;

        // Step counts stay exact over 10 m with ramps.
        let profile = SCurve {
            max_speed: 400.mm_per_s(),
            accel: 1200.mm_per_s2(),
            jerk: 8000.mm_per_s3(),
        };
        let spans = [crate::motion::planner::Span::new(10.m())];
        let (stepgen, _, _) = generate(profile, &Line(10.m()), &spans, step);
        assert_eq!(stepgen.steps(), [expected; 2]);

        // At constant speed every step time is known: step n comes at
        // (n - 0.5)·step / v. Positions are computed exactly here, so that
        // only the generator is tested: after 360 000 steps the delays still
        // add up to the last one's time. What is left is the f32 resolution
        // of a position near 10 m, about 0.001 mm or 10 ticks here, and it
        // does not grow with the number of steps.
        let mut stepgen = StepGen::new([step], TICKS);
        let (mut count, mut sum) = (0usize, 0u64);
        for i in 1..=25_000u32 {
            let s = Length::mm((i as f64 * 0.4) as f32);
            stepgen.slice([s], DT, |_, w| {
                count += 1;
                sum += w.delay() as u64;
            });
        }
        assert_eq!(count, expected as usize);
        let last = (expected as f64 - 0.5) * 0.0275 / 400.0 * TICKS as f64;
        assert!((sum as f64 - last).abs() <= 16.0, "{sum} ticks vs {last}");
    }

    /// A single axis that goes forward ten steps, then back fifteen.
    struct Back;

    impl Path<1> for Back {
        fn length(&self) -> Length {
            25.mm()
        }
        fn axes(&self, s: Length) -> [Length; 1] {
            let s = s.as_mm().clamp(0.0, 25.0);
            [Length::mm(if s < 10.0 { s } else { 20.0 - s })]
        }
    }

    #[test]
    fn direction_changes_land_on_the_right_word() {
        let mut stepgen = StepGen::new([1.mm()], TICKS);
        let mut words = std::vec::Vec::new();
        // 0.1 mm of path per slice, slower than a step per slice.
        for i in 1..=250 {
            let s = Length::mm(i as f32 * 0.1);
            stepgen.slice(Back.axes(s), DT, |_, w| words.push(w));
        }
        let first_back = words.iter().position(|w| !w.forward()).unwrap();
        assert_eq!(first_back, 10);
        assert!(words[..10].iter().all(|w| w.forward()));
        assert!(words[10..].iter().all(|w| !w.forward()));
        assert_eq!(net(&words), -5);
        assert_eq!(stepgen.steps(), [-5]);
    }

    #[test]
    fn rebase_carries_the_remainder() {
        let mut stepgen = StepGen::new([1.mm()], TICKS);
        let mut steps = 0;
        // Three moves of 0.4 steps make one step in total.
        for _ in 0..3 {
            stepgen.slice([0.4.mm()], DT, |_, _| steps += 1);
            stepgen.rebase();
        }
        assert_eq!(steps, 1);
    }

    #[test]
    fn delays_are_measured_from_emitted_steps() {
        // 0.3 ms between steps at 1 MHz rounds to 300 ticks, never drifting:
        // after 1000 steps the delays add up to exactly 300 ms.
        let mut stepgen = StepGen::new([1.mm()], 1_000_000);
        let mut total: u64 = 0;
        let mut count = 0;
        let slice = Duration::from_micros(900); // three steps per slice
        for i in 1..=334 {
            let s = Length::mm(i as f32 * 3.0);
            stepgen.slice([s], slice, |_, w| {
                total += w.delay() as u64;
                count += 1;
            });
        }
        assert_eq!(count, 1002);
        // The first step comes half a step in; the rest 300 ticks apart.
        assert_eq!(total, 150 + 1001 * 300);
    }
}

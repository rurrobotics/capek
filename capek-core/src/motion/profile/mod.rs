//! Motion profiles: how speed changes along a path.
//!
//! A [`Profile`] plans motion along one coordinate, the distance travelled
//! along the path. Planning gives a [`Plan`], which is advanced in time and
//! reports the [`State`] (position, speed and acceleration) at each step.
//!
//! The coordinate only moves forward: distances and speeds are non-negative,
//! and the path decides which way each axis turns. Negative inputs are treated
//! as zero.
//!
//! Profiles plan between non-zero speeds, which the route planner needs, and
//! [`Profile::stop`] can bring a plan to rest from any state. That is what lets
//! a move pause and replan.
//!
//! Available profiles:
//!
//! - [`Constant`]: a fixed speed with no ramps.
//! - [`Trapezoid`]: constant acceleration, computed in closed form.
//! - [`FastTrapezoid`]: constant acceleration, computed per step.
//! - [`Power`]: ninja's power curve, strong at rest and easing into cruise.
//! - [`FastPower`]: the power curve, computed per step.
//! - [`Sine`]: acceleration rises and falls as a half sine.
//! - [`SCurve`]: jerk-limited, continuous acceleration even when replanning.
//!
//! # Examples
//!
//! Pause a move halfway and continue it later:
//!
//! ```
//! use capek_core::motion::profile::{Plan, Profile, State, Trapezoid};
//! use capek_core::units::UnitExt;
//! use core::time::Duration;
//!
//! let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
//! let target = 1.m();
//!
//! let mut plan = profile.plan(State::REST, target, 0.mm_per_s());
//! let mut state = State::REST;
//! for _ in 0..800 {
//!     state = plan.advance(Duration::from_millis(1));
//! }
//!
//! // Stop as quickly as the limits allow.
//! let mut stop = profile.stop(state);
//! while !stop.is_done() {
//!     state = stop.advance(Duration::from_millis(1));
//! }
//! assert_eq!(state.v, 0.mm_per_s());
//!
//! // Later, finish the move from where the robot stopped.
//! let mut rest = profile.plan(state, target - state.s, 0.mm_per_s());
//! while !rest.is_done() {
//!     state = rest.advance(Duration::from_millis(1));
//! }
//! assert!((state.s - target).abs() < 0.001.mm());
//! ```

mod constant;
mod power;
mod scurve;
mod sine;
mod trapezoid;

pub use constant::{Constant, ConstantPlan};
pub use power::{FastPower, FastPowerPlan, Power, PowerPlan};
pub use scurve::{SCurve, SCurvePlan};
pub use sine::{Sine, SinePlan};
pub use trapezoid::{FastTrapezoid, FastTrapezoidPlan, Trapezoid, TrapezoidPlan};

use crate::units::{Acceleration, Length, Speed};
use core::time::Duration;

/// Where a motion is at one instant: position along the path, speed and
/// acceleration.
///
/// ```
/// use capek_core::motion::profile::State;
/// use capek_core::units::UnitExt;
///
/// let cruising = State { s: 250.mm(), v: 400.mm_per_s(), a: 0.mm_per_s2() };
/// assert_eq!(State::at(250.mm()).v, State::REST.v);
/// # let _ = cruising;
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct State {
    /// Distance travelled along the path.
    pub s: Length,
    /// Speed along the path.
    pub v: Speed,
    /// Acceleration along the path.
    pub a: Acceleration,
}

impl State {
    /// At rest at the start of the path.
    pub const REST: Self = Self::at(Length::ZERO);

    /// At rest at position `s`.
    ///
    /// ```
    /// use capek_core::motion::profile::State;
    /// use capek_core::units::UnitExt;
    ///
    /// let s = State::at(100.mm());
    /// assert_eq!(s.s, 100.mm());
    /// assert_eq!(s.v, 0.mm_per_s());
    /// ```
    pub const fn at(s: Length) -> Self {
        Self {
            s,
            v: Speed::ZERO,
            a: Acceleration::ZERO,
        }
    }
}

/// Plans motion along a path.
///
/// Implement it, together with [`Plan`], to add a profile of your own.
///
/// ```
/// use capek_core::motion::profile::{Plan, Profile, State, Trapezoid};
/// use capek_core::units::UnitExt;
///
/// fn time_to_cover<P: Profile>(profile: &P, distance: capek_core::units::Length) -> u32 {
///     let mut plan = profile.plan(State::REST, distance, 0.mm_per_s());
///     let mut ms = 0;
///     while !plan.is_done() {
///         plan.advance(core::time::Duration::from_millis(1));
///         ms += 1;
///     }
///     ms
/// }
///
/// let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
/// // 1/3 s to accelerate, 1/3 s to brake and 2 s of cruise.
/// assert!((time_to_cover(&profile, 0.9333.m()) as i32 - 2667).abs() <= 1);
/// ```
pub trait Profile: Clone {
    /// The plan this profile produces.
    type Plan: Plan;

    /// Plans a move of `distance` starting at `from` and ending at
    /// `end_speed`.
    ///
    /// The plan always ends exactly at `from.s + distance`. When the limits
    /// do not allow `end_speed` to be reached within `distance`, the plan ends
    /// at the closest speed they allow instead.
    fn plan(&self, from: State, distance: Length, end_speed: Speed) -> Self::Plan;

    /// Plans the shortest stop the limits allow, starting at `from`.
    fn stop(&self, from: State) -> Self::Plan;

    /// Sets the distance increment that per-step profiles step through the
    /// curve in.
    ///
    /// Parent components call this from their geometry, for example with the
    /// distance of one wheel microstep. Profiles that are not computed per
    /// step ignore it, which is the default.
    fn set_resolution(&mut self, resolution: Length) {
        let _ = resolution;
    }

    /// The cruise speed, the highest speed a plan reaches from rest.
    ///
    /// ```
    /// use capek_core::motion::profile::{Profile, Trapezoid};
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
    /// assert_eq!(profile.max_speed(), 400.mm_per_s());
    /// ```
    fn max_speed(&self) -> Speed;

    /// The highest acceleration a plan uses.
    ///
    /// A profile whose speed jumps, such as [`Constant`], reports an infinite
    /// acceleration.
    ///
    /// ```
    /// use capek_core::motion::profile::{Profile, Trapezoid};
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
    /// assert_eq!(profile.max_accel(), 1200.mm_per_s2());
    /// ```
    fn max_accel(&self) -> Acceleration;

    /// The same profile with every limit (speed, acceleration and jerk)
    /// multiplied by `factor`.
    ///
    /// Scaling all limits by the same factor keeps the shape of the motion.
    /// The speed planner uses it on curves, where the outer wheel moves
    /// faster than the centre: planning the centre with limits scaled by
    /// `1 / (1 + |κ|·track / 2)` keeps that wheel within the real limits.
    /// Parameters that are not limits, such as a resolution or an exponent,
    /// are kept.
    ///
    /// ```
    /// use capek_core::motion::profile::{Profile, Trapezoid};
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
    /// let half = profile.scaled(0.5);
    /// assert_eq!(half.max_speed(), 200.mm_per_s());
    /// assert_eq!(half.accel, 600.mm_per_s2());
    /// ```
    fn scaled(&self, factor: f32) -> Self;

    /// The highest speed reachable from `from` within `distance`, starting
    /// with no acceleration, capped at [`max_speed`](Self::max_speed).
    ///
    /// Deceleration mirrors acceleration, so this is also the highest speed
    /// from which the profile can slow down to `from` within `distance`. The
    /// speed planner uses it in both directions to choose speeds that every
    /// plan can actually reach.
    ///
    /// ```
    /// use capek_core::motion::profile::{Profile, Trapezoid};
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
    /// // v² = 2·a·d
    /// let v = profile.reachable(0.mm_per_s(), 15.mm());
    /// assert!((v - 189.737.mm_per_s()).abs() < 0.01.mm_per_s());
    /// assert_eq!(profile.reachable(0.mm_per_s(), 1.m()), 400.mm_per_s());
    /// ```
    fn reachable(&self, from: Speed, distance: Length) -> Speed;
}

/// A planned move, advanced in time.
///
/// ```
/// use capek_core::motion::profile::{Plan, Profile, State, Trapezoid};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
/// let mut plan = profile.plan(State::REST, 100.mm(), 0.mm_per_s());
/// let state = plan.advance(Duration::from_millis(100));
/// assert_eq!(state.a, 1200.mm_per_s2());
/// assert!(!plan.is_done());
/// ```
pub trait Plan: Clone {
    /// Moves the plan forward by `dt` and returns the state reached.
    ///
    /// Once the plan is done, it keeps returning its final state.
    fn advance(&mut self, dt: Duration) -> State;

    /// Whether the plan has reached its end.
    fn is_done(&self) -> bool;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::UnitExt;

    /// Runs a plan to the end and returns its final state.
    fn finish<P: Plan>(mut plan: P) -> State {
        let mut state = State::REST;
        let mut steps = 0;
        while !plan.is_done() {
            state = plan.advance(Duration::from_millis(1));
            steps += 1;
            assert!(steps < 60_000, "plan never finished");
        }
        state
    }

    /// `reachable` must agree with what `plan` can do, in both directions.
    fn check<P: Profile + core::fmt::Debug>(p: P, tol: f32) {
        let vmax = p.max_speed().as_mm_per_s();
        for (v0, d) in [
            (0.0, 5.0),
            (0.0, 50.0),
            (0.0, 2000.0),
            (100.0, 30.0),
            (300.0, 10.0),
        ] {
            let from = State {
                s: 0.mm(),
                v: v0.mm_per_s(),
                a: 0.mm_per_s2(),
            };
            let v1 = p.reachable(v0.mm_per_s(), d.mm()).as_mm_per_s();
            assert!(v1 <= vmax + 1e-3, "{p:?}: {v1} above max speed");
            assert!(v1 >= v0.min(vmax) - tol, "{p:?}: {v1} below start {v0}");
            assert!(v1 <= p.reachable(v0.mm_per_s(), (d * 2.0).mm()).as_mm_per_s() + 1e-3);

            // Speeding up to it fits in the distance...
            let end = finish(p.plan(from, d.mm(), v1.mm_per_s()));
            assert!(
                (end.v.as_mm_per_s() - v1).abs() <= tol,
                "{p:?}: {end:?} not {v1}"
            );

            // ...and so does slowing back down from it.
            let back = State {
                v: v1.mm_per_s(),
                ..from
            };
            let end = finish(p.plan(back, d.mm(), v0.mm_per_s()));
            let expected = v0.min(vmax);
            assert!(
                (end.v.as_mm_per_s() - expected).abs() <= tol,
                "{p:?}: braking from {v1} over {d} ended at {end:?}"
            );
        }
    }

    #[test]
    fn reachable_matches_every_profile() {
        let (v, a, j) = (400.mm_per_s(), 1200.mm_per_s2(), 8000.mm_per_s3());
        let res = (56.0 * core::f32::consts::PI / 400.0).mm();
        let mut ft = FastTrapezoid::new(v, a);
        ft.set_resolution(res);
        let mut fp = FastPower::new(v, a, 3.0);
        fp.set_resolution(res);

        check(Constant { speed: v }, 1e-3);
        check(
            Trapezoid {
                max_speed: v,
                accel: a,
            },
            1e-2,
        );
        check(
            Sine {
                max_speed: v,
                accel: a,
            },
            1e-2,
        );
        check(
            Power {
                max_speed: v,
                accel: a,
                p: 3.0,
            },
            0.05,
        );
        check(
            SCurve {
                max_speed: v,
                accel: a,
                jerk: j,
            },
            0.05,
        );
        // Whole increments: speeds are a few mm/s apart at low speed.
        check(ft, 6.0);
        check(fp, 6.0);
    }

    #[test]
    fn scaled_scales_the_limits() {
        let p = SCurve {
            max_speed: 400.mm_per_s(),
            accel: 1200.mm_per_s2(),
            jerk: 8000.mm_per_s3(),
        };
        let half = p.scaled(0.5);
        assert_eq!(half.max_speed(), 200.mm_per_s());
        assert_eq!(half.accel, 600.mm_per_s2());
        assert_eq!(half.jerk, 4000.mm_per_s3());

        let mut f = FastPower::new(400.mm_per_s(), 1200.mm_per_s2(), 3.0);
        f.set_resolution(0.44.mm());
        let half = f.scaled(0.5);
        assert_eq!(half.resolution(), 0.44.mm());
        assert_eq!(half.p, 3.0);
        assert_eq!(half.max_speed(), 200.mm_per_s());
    }
}

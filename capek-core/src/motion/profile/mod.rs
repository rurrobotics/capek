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
//! - [`Trapezoid`]: constant acceleration, computed in closed form.
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

mod trapezoid;

pub use trapezoid::{Trapezoid, TrapezoidPlan};

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

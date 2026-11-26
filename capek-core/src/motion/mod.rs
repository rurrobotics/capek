//! Motion control.
//!
//! Motion is the combination of independent choices: a [`profile`] decides
//! how fast to move along a path, a [`path`] decides where each axis is at each
//! point along it, and a step strategy decides when step timings are
//! computed. The time base shared by all axes is time itself, not steps.
//! [`kinematics`] converts between the robot's motion and its wheels, and
//! tracks its pose on the field. [`route`] builds paths through waypoints.
//!
//! # Examples
//!
//! ```
//! use capek_core::motion::profile::{Plan, Profile, State, Trapezoid};
//! use capek_core::units::UnitExt;
//! use core::time::Duration;
//!
//! let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
//! let mut plan = profile.plan(State::REST, 500.mm(), 0.mm_per_s());
//!
//! // A step generator advances the plan once per time slice.
//! while !plan.is_done() {
//!     let state = plan.advance(Duration::from_millis(1));
//!     assert!(state.v <= 400.mm_per_s());
//! }
//! ```

pub mod kinematics;
pub mod path;
pub mod profile;
pub mod route;

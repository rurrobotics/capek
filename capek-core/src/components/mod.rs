//! Components: what a `robot!` block is built from.
//!
//! A component holds every piece of logic that does not depend on the
//! hardware, and reaches the hardware through a driver supplied by the
//! platform. The same component works on any platform.

mod gpio;
mod hcsr04;
mod servo;
mod stepper;

pub use gpio::{Button, ButtonConfig, Output, OutputConfig};
pub use hcsr04::{Hcsr04, Hcsr04Config};
pub use servo::{Servo, ServoConfig};
pub use stepper::{Stepper, StepperConfig};

use crate::units::Length;

/// A sensor that measures the distance to the nearest object.
///
/// `Proximity` and other watchers accept any implementation, so one kind of
/// sensor can replace another, such as a time-of-flight sensor for an
/// [`Hcsr04`].
///
/// ```
/// use capek_core::components::DistanceSensor;
/// use capek_core::units::{Length, UnitExt};
///
/// /// Always sees a wall 300 mm away.
/// struct Wall;
/// impl DistanceSensor for Wall {
///     async fn distance(&mut self) -> Option<Length> {
///         Some(300.mm())
///     }
/// }
///
/// assert_eq!(embassy_futures::block_on(Wall.distance()), Some(300.mm()));
/// ```
#[allow(async_fn_in_trait)]
pub trait DistanceSensor {
    /// Takes a reading, or returns `None` when the sensor gives none, for
    /// example because it is not answering.
    async fn distance(&mut self) -> Option<Length>;
}

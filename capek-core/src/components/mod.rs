//! Components: what a `robot!` block is built from.
//!
//! A component holds every piece of logic that does not depend on the
//! hardware, and reaches the hardware through a driver supplied by the
//! platform. The same component works on any platform.

mod servo;
mod stepper;

pub use servo::{Servo, ServoConfig};
pub use stepper::{Stepper, StepperConfig};

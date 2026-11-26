//! The contract every platform crate fulfils.
//!
//! A platform crate, such as `capek-rp`, implements [`Platform`] once, naming
//! its driver for every kind of component. The `capek` facade derives the
//! names users see from the selected platform, so `Stepper<StepDir>` or
//! `Hcsr04` resolve to that platform's drivers. A platform that leaves
//! something out fails to compile in its own crate, so the contract is checked
//! rather than hoped for.
//!
//! The command context and the USB transport join the trait together with
//! their own traits.

use crate::driver::{PulseTimer, ServoDriver, StepperDriver};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

/// The drivers a platform supplies.
///
/// ```
/// use capek_core::driver::{PulseTimer, ServoDriver, StepWord, StepperDriver};
/// use capek_core::platform::Platform;
/// use core::convert::Infallible;
/// use core::time::Duration;
/// use embedded_hal::digital::{ErrorType, InputPin, OutputPin};
/// use embedded_hal_async::digital::Wait;
///
/// /// A platform for host tests, whose drivers do nothing.
/// struct Host;
///
/// struct Steps;
/// impl StepperDriver for Steps {
///     const TICK_HZ: u32 = 1_000_000;
///     fn set_enabled(&mut self, _on: bool) {}
///     fn free(&self) -> usize { usize::MAX }
///     async fn push(&mut self, _words: &[StepWord]) {}
///     async fn finish(&mut self) {}
///     fn halt(&mut self) -> u32 { 0 }
/// }
///
/// struct Pulse;
/// impl ServoDriver for Pulse {
///     fn set_pulse(&mut self, _width: Duration) {}
///     fn set_enabled(&mut self, _on: bool) {}
/// }
///
/// struct Echo;
/// impl PulseTimer for Echo {
///     async fn measure(&mut self, _t: Duration, _timeout: Duration) -> Option<Duration> { None }
/// }
///
/// struct Pin;
/// impl ErrorType for Pin { type Error = Infallible; }
/// impl InputPin for Pin {
///     fn is_high(&mut self) -> Result<bool, Infallible> { Ok(false) }
///     fn is_low(&mut self) -> Result<bool, Infallible> { Ok(true) }
/// }
/// impl OutputPin for Pin {
///     fn set_high(&mut self) -> Result<(), Infallible> { Ok(()) }
///     fn set_low(&mut self) -> Result<(), Infallible> { Ok(()) }
/// }
/// impl Wait for Pin {
///     async fn wait_for_high(&mut self) -> Result<(), Infallible> { Ok(()) }
///     async fn wait_for_low(&mut self) -> Result<(), Infallible> { Ok(()) }
///     async fn wait_for_rising_edge(&mut self) -> Result<(), Infallible> { Ok(()) }
///     async fn wait_for_falling_edge(&mut self) -> Result<(), Infallible> { Ok(()) }
///     async fn wait_for_any_edge(&mut self) -> Result<(), Infallible> { Ok(()) }
/// }
///
/// impl Platform for Host {
///     type StepDir = Steps;
///     type FourWire = Steps;
///     type Servo = Pulse;
///     type PulseTimer = Echo;
///     type Input = Pin;
///     type Output = Pin;
/// }
///
/// fn tick_rate<P: Platform>() -> u32 {
///     <P::StepDir as StepperDriver>::TICK_HZ
/// }
/// assert_eq!(tick_rate::<Host>(), 1_000_000);
/// ```
pub trait Platform {
    /// Stepper driver for step/dir wiring, such as an A4988 or a TMC2209.
    type StepDir: StepperDriver;
    /// Stepper driver for four coil pins, such as a 28BYJ-48 on a ULN2003.
    type FourWire: StepperDriver;
    /// Servo driver.
    type Servo: ServoDriver;
    /// Pulse timer for echo-ranging sensors such as the HC-SR04.
    type PulseTimer: PulseTimer;
    /// Digital input that can wait for edges, for buttons and switches.
    type Input: InputPin + Wait;
    /// Digital output.
    type Output: OutputPin;
}

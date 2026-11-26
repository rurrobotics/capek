//! What components ask of the hardware.
//!
//! Drivers are the hardware primitives behind components, supplied by a
//! platform crate: [`StepperDriver`], [`ServoDriver`] and [`PulseTimer`].
//! They are kept as small as possible; the logic that does not depend on the
//! hardware lives in the components that use them. Simple inputs and outputs
//! use the `embedded-hal` pin traits instead.

use core::time::Duration;

/// One motor step: its direction and the time since the previous step.
///
/// Bit 31 is the direction, set for forward. Bits 0 to 30 are the delay since
/// the previous step on the same axis, in ticks of the driver's timer. A
/// stepper driver consumes a stream of these and pulses its step pin after
/// each delay. Because the direction travels with each step, a reversal lands
/// on exactly the right pulse.
///
/// ```
/// use capek_core::driver::StepWord;
///
/// let word = StepWord::new(false, 4000);
/// assert!(!word.forward());
/// assert_eq!(word.delay(), 4000);
/// assert_eq!(StepWord::new(true, 4000).bits(), (1 << 31) | 4000);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct StepWord(u32);

impl StepWord {
    /// The longest delay a word can hold, in ticks.
    pub const MAX_DELAY: u32 = (1 << 31) - 1;

    const FORWARD: u32 = 1 << 31;

    /// A step in the given direction after `delay` ticks, capped at
    /// [`MAX_DELAY`](Self::MAX_DELAY).
    pub const fn new(forward: bool, delay: u32) -> Self {
        let delay = if delay > Self::MAX_DELAY {
            Self::MAX_DELAY
        } else {
            delay
        };
        Self(if forward {
            Self::FORWARD | delay
        } else {
            delay
        })
    }

    /// Whether the step is forward.
    pub const fn forward(self) -> bool {
        self.0 & Self::FORWARD != 0
    }

    /// Ticks since the previous step on the same axis.
    pub const fn delay(self) -> u32 {
        self.0 & Self::MAX_DELAY
    }

    /// The raw word, as a driver sends it to the hardware.
    pub const fn bits(self) -> u32 {
        self.0
    }

    /// A word from its raw bits.
    ///
    /// ```
    /// use capek_core::driver::StepWord;
    ///
    /// let word = StepWord::new(true, 123);
    /// assert_eq!(StepWord::from_bits(word.bits()), word);
    /// ```
    pub const fn from_bits(bits: u32) -> Self {
        Self(bits)
    }
}

/// A stepper motor driver: consumes [`StepWord`]s and pulses the motor.
///
/// Platforms supply one for each wiring, such as step/dir or four-wire. The
/// driver only emits steps at the delays it is given; everything else, from
/// steps per revolution to motion planning, lives in the components.
///
/// The `async` methods have no `Send` bound on their futures: Embassy runs
/// each task on a single-threaded executor, as `embedded-hal-async` assumes.
///
/// ```
/// use capek_core::driver::{StepWord, StepperDriver};
///
/// /// Counts the steps it is given instead of moving anything.
/// struct Counter {
///     position: i32,
///     enabled: bool,
/// }
///
/// impl StepperDriver for Counter {
///     const TICK_HZ: u32 = 1_000_000;
///     fn set_enabled(&mut self, on: bool) {
///         self.enabled = on;
///     }
///     fn free(&self) -> usize {
///         usize::MAX
///     }
///     async fn push(&mut self, words: &[StepWord]) {
///         for w in words {
///             self.position += if w.forward() { 1 } else { -1 };
///         }
///     }
///     async fn finish(&mut self) {}
///     fn halt(&mut self) -> u32 {
///         0
///     }
/// }
///
/// let mut driver = Counter { position: 0, enabled: false };
/// driver.set_enabled(true);
/// embassy_futures::block_on(driver.push(&[StepWord::new(true, 100); 3]));
/// assert_eq!(driver.position, 3);
/// ```
#[allow(async_fn_in_trait)]
pub trait StepperDriver {
    /// Rate of the timer the delays in [`StepWord`]s are counted in.
    const TICK_HZ: u32;

    /// Powers the motor coils on or off.
    fn set_enabled(&mut self, on: bool);

    /// How many words the driver can take without waiting.
    fn free(&self) -> usize;

    /// Queues `words`, waiting while the queue is full.
    async fn push(&mut self, words: &[StepWord]);

    /// Waits until every queued step has been emitted.
    async fn finish(&mut self);

    /// Stops at once, dropping the queue, and returns how many queued steps
    /// were not emitted.
    fn halt(&mut self) -> u32;
}

/// A servo driver: produces the control pulse.
///
/// The refresh period is a detail of the hardware and is set when the driver
/// is built; the component only asks for a pulse width.
///
/// ```
/// use capek_core::driver::ServoDriver;
/// use core::time::Duration;
///
/// struct Recorder(Duration);
///
/// impl ServoDriver for Recorder {
///     fn set_pulse(&mut self, width: Duration) {
///         self.0 = width;
///     }
///     fn set_enabled(&mut self, _on: bool) {}
/// }
///
/// let mut servo = Recorder(Duration::ZERO);
/// servo.set_pulse(Duration::from_micros(1500));
/// assert_eq!(servo.0, Duration::from_micros(1500));
/// ```
pub trait ServoDriver {
    /// Sets the width of the control pulse.
    fn set_pulse(&mut self, width: Duration);

    /// Starts or stops sending pulses.
    fn set_enabled(&mut self, on: bool);
}

/// Triggers a device and times the pulse it answers with.
///
/// This is the hardware primitive behind echo-ranging sensors such as the
/// HC-SR04: drive the trigger pin high for `trigger`, wait for the echo pin
/// to rise, and measure how long it stays high. The sensor component turns
/// that time into a distance, so core never needs a clock.
///
/// ```
/// use capek_core::driver::PulseTimer;
/// use core::time::Duration;
///
/// /// Always answers with a 1 ms echo.
/// struct Fixed;
///
/// impl PulseTimer for Fixed {
///     async fn measure(&mut self, _trigger: Duration, _timeout: Duration) -> Option<Duration> {
///         Some(Duration::from_millis(1))
///     }
/// }
///
/// let echo = embassy_futures::block_on(Fixed.measure(
///     Duration::from_micros(10),
///     Duration::from_millis(30),
/// ));
/// assert_eq!(echo, Some(Duration::from_millis(1)));
/// ```
#[allow(async_fn_in_trait)]
pub trait PulseTimer {
    /// Holds the trigger high for `trigger`, then returns how long the echo
    /// stayed high, or `None` if it did not start and end within `timeout`.
    async fn measure(&mut self, trigger: Duration, timeout: Duration) -> Option<Duration>;
}

//! Drivers for host tests that record what they are asked to do.

use crate::driver::{PulseTimer, ServoDriver, StepWord, StepperDriver};
use core::convert::Infallible;
use core::time::Duration;
use embedded_hal::digital::{ErrorType, InputPin, OutputPin};
use embedded_hal_async::digital::Wait;
use std::vec::Vec;

/// Records every word and enable change; `halt` reports `undone` steps.
#[derive(Debug, Default)]
pub struct MockStepper {
    pub words: Vec<StepWord>,
    pub enabled: bool,
    pub undone: u32,
}

impl StepperDriver for MockStepper {
    const TICK_HZ: u32 = 1_000_000;

    fn set_enabled(&mut self, on: bool) {
        self.enabled = on;
    }

    fn free(&self) -> usize {
        usize::MAX
    }

    async fn push(&mut self, words: &[StepWord]) {
        self.words.extend_from_slice(words);
    }

    async fn finish(&mut self) {}

    fn halt(&mut self) -> u32 {
        core::mem::take(&mut self.undone)
    }
}

/// Records the last pulse and whether pulses are being sent.
#[derive(Debug, Default)]
pub struct MockServo {
    pub pulse: Duration,
    pub enabled: bool,
}

impl ServoDriver for MockServo {
    fn set_pulse(&mut self, width: Duration) {
        self.pulse = width;
    }

    fn set_enabled(&mut self, on: bool) {
        self.enabled = on;
    }
}

/// A pin with a settable level that records which levels it waited for.
/// Waiting succeeds at once.
#[derive(Debug, Default)]
pub struct MockPin {
    pub level: bool,
    pub waited: Vec<bool>,
}

impl MockPin {
    pub fn low() -> Self {
        Self::default()
    }
}

impl ErrorType for MockPin {
    type Error = Infallible;
}

impl InputPin for MockPin {
    fn is_high(&mut self) -> Result<bool, Infallible> {
        Ok(self.level)
    }

    fn is_low(&mut self) -> Result<bool, Infallible> {
        Ok(!self.level)
    }
}

impl OutputPin for MockPin {
    fn set_high(&mut self) -> Result<(), Infallible> {
        self.level = true;
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Infallible> {
        self.level = false;
        Ok(())
    }
}

impl Wait for MockPin {
    async fn wait_for_high(&mut self) -> Result<(), Infallible> {
        self.waited.push(true);
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Infallible> {
        self.waited.push(false);
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Infallible> {
        self.waited.push(true);
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Infallible> {
        self.waited.push(false);
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Infallible> {
        Ok(())
    }
}

/// Answers every measurement with `echo` and records what it was asked for.
#[derive(Debug)]
pub struct MockPulseTimer {
    pub echo: Option<Duration>,
    pub asked: Option<(Duration, Duration)>,
}

impl MockPulseTimer {
    pub fn new(echo: Option<Duration>) -> Self {
        Self { echo, asked: None }
    }
}

impl PulseTimer for MockPulseTimer {
    async fn measure(&mut self, trigger: Duration, timeout: Duration) -> Option<Duration> {
        self.asked = Some((trigger, timeout));
        self.echo
    }
}

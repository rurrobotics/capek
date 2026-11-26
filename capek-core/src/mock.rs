//! Drivers for host tests that record what they are asked to do.

use crate::driver::{ServoDriver, StepWord, StepperDriver};
use core::time::Duration;
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

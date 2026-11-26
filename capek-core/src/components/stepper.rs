//! A stepper motor.

use crate::driver::{StepWord, StepperDriver};
use crate::units::Angle;
use core::f32::consts::PI;

/// Settings of a [`Stepper`].
///
/// ```
/// use capek_core::components::StepperConfig;
///
/// let config = StepperConfig { steps_per_rev: 400, invert: true, ..Default::default() };
/// assert_eq!(config.microsteps, 1);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepperConfig {
    /// Full steps per revolution of the motor shaft, such as 200 or 400.
    pub steps_per_rev: u32,
    /// Microsteps per full step the driver is set to, such as 1, 8 or 16.
    pub microsteps: u32,
    /// Whether forward steps turn the shaft the other way, for a motor
    /// mounted mirrored.
    pub invert: bool,
}

impl Default for StepperConfig {
    fn default() -> Self {
        Self {
            steps_per_rev: 200,
            microsteps: 1,
            invert: false,
        }
    }
}

/// A stepper motor behind a [`StepperDriver`].
///
/// It knows how many steps make a revolution, which way is forward and where
/// the shaft is. Steps reach it as [`StepWord`]s, usually from a step
/// generator; it flips their direction when the motor is inverted, counts
/// them, and hands them to its driver.
///
/// ```
/// use capek_core::components::{Stepper, StepperConfig};
/// # use capek_core::driver::{StepWord, StepperDriver};
/// # struct Driver;
/// # impl StepperDriver for Driver {
/// #     const TICK_HZ: u32 = 1_000_000;
/// #     fn set_enabled(&mut self, _on: bool) {}
/// #     fn free(&self) -> usize { usize::MAX }
/// #     async fn push(&mut self, _words: &[StepWord]) {}
/// #     async fn finish(&mut self) {}
/// #     fn halt(&mut self) -> u32 { 0 }
/// # }
/// use capek_core::units::UnitExt;
///
/// let config = StepperConfig { steps_per_rev: 400, microsteps: 16, ..Default::default() };
/// let mut stepper = Stepper::new(Driver, config);
/// assert_eq!(stepper.steps_per_rev(), 6400);
/// assert_eq!(stepper.steps_for(90.deg()), 1600);
///
/// embassy_futures::block_on(stepper.push(&[StepWord::new(true, 100); 1600]));
/// assert!((stepper.position() - 90.deg()).abs() < 1e-3.deg());
/// ```
#[derive(Debug)]
pub struct Stepper<D: StepperDriver> {
    driver: D,
    config: StepperConfig,
    /// Steps queued so far, net of direction, in the motor's own sense.
    steps: i32,
    /// Whether the last queued step was forward.
    last_forward: bool,
}

/// Words handed to the driver at once when inverting directions.
const CHUNK: usize = 32;

impl<D: StepperDriver> Stepper<D> {
    /// A stepper at position zero, with its coils off.
    pub fn new(mut driver: D, config: StepperConfig) -> Self {
        driver.set_enabled(false);
        Self {
            driver,
            config,
            steps: 0,
            last_forward: true,
        }
    }

    /// Steps per revolution of the shaft, microsteps included.
    pub fn steps_per_rev(&self) -> u32 {
        self.config.steps_per_rev * self.config.microsteps.max(1)
    }

    /// The angle of one step.
    ///
    /// ```
    /// use capek_core::components::{Stepper, StepperConfig};
    /// # use capek_core::driver::{StepWord, StepperDriver};
    /// # struct Driver;
    /// # impl StepperDriver for Driver {
    /// #     const TICK_HZ: u32 = 1_000_000;
    /// #     fn set_enabled(&mut self, _on: bool) {}
    /// #     fn free(&self) -> usize { usize::MAX }
    /// #     async fn push(&mut self, _words: &[StepWord]) {}
    /// #     async fn finish(&mut self) {}
    /// #     fn halt(&mut self) -> u32 { 0 }
    /// # }
    /// use capek_core::units::UnitExt;
    ///
    /// let stepper = Stepper::new(Driver, StepperConfig::default());
    /// assert!((stepper.step_angle() - 1.8.deg()).abs() < 1e-5.deg());
    /// ```
    pub fn step_angle(&self) -> Angle {
        Angle::rad(2.0 * PI / self.steps_per_rev() as f32)
    }

    /// The whole number of steps closest to turning by `angle`.
    pub fn steps_for(&self, angle: Angle) -> i32 {
        libm::roundf(angle.as_rad() / self.step_angle().as_rad()) as i32
    }

    /// The angle `steps` steps turn the shaft.
    pub fn angle_of(&self, steps: i32) -> Angle {
        self.step_angle() * steps as f32
    }

    /// Where the shaft is, from the steps queued so far.
    pub fn position(&self) -> Angle {
        self.angle_of(self.steps)
    }

    /// Steps queued so far, net of direction.
    pub fn steps(&self) -> i32 {
        self.steps
    }

    /// Declares the shaft to be at `angle`, for example after homing.
    pub fn set_position(&mut self, angle: Angle) {
        self.steps = self.steps_for(angle);
    }

    /// Powers the coils, holding the shaft.
    pub fn enable(&mut self) {
        self.driver.set_enabled(true);
    }

    /// Releases the coils, letting the shaft turn freely.
    pub fn disable(&mut self) {
        self.driver.set_enabled(false);
    }

    /// Queues `words` on the driver, waiting while its queue is full.
    ///
    /// Forward is the motor's forward: when the stepper is inverted, each
    /// word's direction is flipped on the way to the driver.
    pub async fn push(&mut self, words: &[StepWord]) {
        for w in words {
            self.steps += if w.forward() { 1 } else { -1 };
        }
        if let Some(last) = words.last() {
            self.last_forward = last.forward();
        }
        if !self.config.invert {
            self.driver.push(words).await;
            return;
        }
        let mut buf = [StepWord::new(true, 0); CHUNK];
        for chunk in words.chunks(CHUNK) {
            for (out, w) in buf.iter_mut().zip(chunk) {
                *out = StepWord::new(!w.forward(), w.delay());
            }
            self.driver.push(&buf[..chunk.len()]).await;
        }
    }

    /// How many words the driver can take without waiting.
    pub fn free(&self) -> usize {
        self.driver.free()
    }

    /// Waits until every queued step has been made.
    pub async fn finish(&mut self) {
        self.driver.finish().await;
    }

    /// Stops at once, dropping the queued steps, and returns how many were
    /// not made.
    ///
    /// The position is corrected for them, taking them to go the way the
    /// last queued step did. That holds unless the direction changed among
    /// the last queued steps, which a hard stop rarely meets.
    pub fn halt(&mut self) -> u32 {
        let dropped = self.driver.halt();
        let sign = if self.last_forward { 1 } else { -1 };
        self.steps -= sign * dropped as i32;
        dropped
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockStepper;
    use crate::units::UnitExt;
    use embassy_futures::block_on;

    fn stepper(config: StepperConfig) -> Stepper<MockStepper> {
        Stepper::new(MockStepper::default(), config)
    }

    #[test]
    fn converts_rotations_with_microstepping() {
        let s = stepper(StepperConfig {
            steps_per_rev: 400,
            microsteps: 16,
            ..Default::default()
        });
        assert_eq!(s.steps_per_rev(), 6400);
        assert_eq!(s.steps_for(360.deg()), 6400);
        assert_eq!(s.steps_for((-45).deg()), -800);
        assert!((s.angle_of(3200) - 180.deg()).abs() < 1e-3.deg());
    }

    #[test]
    fn inversion_flips_directions_at_the_driver() {
        let mut s = stepper(StepperConfig {
            invert: true,
            ..Default::default()
        });
        let words: std::vec::Vec<_> = (0..100).map(|i| StepWord::new(i < 70, 50)).collect();
        block_on(s.push(&words));
        // The component counts in the motor's sense...
        assert_eq!(s.steps(), 70 - 30);
        // ...and the driver sees every direction flipped, delays untouched.
        let sent = &s.driver.words;
        assert_eq!(sent.len(), 100);
        assert!(
            sent.iter()
                .zip(&words)
                .all(|(a, b)| a.forward() != b.forward() && a.delay() == b.delay())
        );
    }

    #[test]
    fn not_inverted_passes_words_through() {
        let mut s = stepper(StepperConfig::default());
        let words = [StepWord::new(false, 7); 5];
        block_on(s.push(&words));
        assert_eq!(s.driver.words, words);
        assert_eq!(s.steps(), -5);
    }

    #[test]
    fn enable_and_disable_reach_the_driver() {
        let mut s = stepper(StepperConfig::default());
        assert!(!s.driver.enabled);
        s.enable();
        assert!(s.driver.enabled);
        s.disable();
        assert!(!s.driver.enabled);
    }

    #[test]
    fn halt_corrects_the_position() {
        let mut s = stepper(StepperConfig::default());
        block_on(s.push(&[StepWord::new(true, 10); 50]));
        s.driver.undone = 20;
        assert_eq!(s.halt(), 20);
        assert_eq!(s.steps(), 30);
    }

    #[test]
    fn set_position_after_homing() {
        let mut s = stepper(StepperConfig::default());
        block_on(s.push(&[StepWord::new(true, 10); 50]));
        s.set_position(0.deg());
        assert_eq!(s.steps(), 0);
    }
}

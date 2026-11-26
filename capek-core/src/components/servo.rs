//! A hobby servo.

use crate::driver::ServoDriver;
use crate::units::Angle;
use core::time::Duration;

/// Settings of a [`Servo`].
///
/// The defaults are those of a typical hobby servo, as in ninja: a pulse of
/// 1000 to 2000 µs covering 180°, at 0.2 s per 60°.
///
/// ```
/// use capek_core::components::ServoConfig;
/// use core::time::Duration;
///
/// let config = ServoConfig { min_pulse: Duration::from_micros(320), ..Default::default() };
/// assert_eq!(config.max_pulse, Duration::from_micros(2000));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ServoConfig {
    /// Pulse width at angle zero.
    pub min_pulse: Duration,
    /// Pulse width at the end of the range.
    pub max_pulse: Duration,
    /// Angle the pulse range covers.
    pub range: Angle,
    /// Time to travel across the whole range.
    pub travel: Duration,
}

impl Default for ServoConfig {
    fn default() -> Self {
        Self {
            min_pulse: Duration::from_micros(1000),
            max_pulse: Duration::from_micros(2000),
            range: Angle::deg(180.0),
            travel: Duration::from_millis(600),
        }
    }
}

/// A hobby servo behind a [`ServoDriver`].
///
/// It maps angles onto its pulse range and estimates how long a move takes.
/// Core has no clock, so [`set`](Self::set) returns that time for the caller
/// to wait on instead of waiting itself.
///
/// ```
/// use capek_core::components::{Servo, ServoConfig};
/// # use capek_core::driver::ServoDriver;
/// # struct Driver;
/// # impl ServoDriver for Driver {
/// #     fn set_pulse(&mut self, _width: core::time::Duration) {}
/// #     fn set_enabled(&mut self, _on: bool) {}
/// # }
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let mut servo = Servo::new(Driver, ServoConfig::default());
/// let pulse = servo.pulse_for(90.deg());
/// assert!(pulse.abs_diff(Duration::from_micros(1500)) < Duration::from_micros(1));
///
/// servo.set(0.deg());
/// // 90° at 0.2 s per 60°.
/// let travel = servo.set(90.deg());
/// assert!(travel.abs_diff(Duration::from_millis(300)) < Duration::from_micros(1));
/// ```
#[derive(Debug)]
pub struct Servo<D: ServoDriver> {
    driver: D,
    config: ServoConfig,
    /// The angle last asked for, once there is one.
    angle: Option<Angle>,
}

impl<D: ServoDriver> Servo<D> {
    /// A servo whose angle is unknown until it is first set.
    pub fn new(driver: D, config: ServoConfig) -> Self {
        Self {
            driver,
            config,
            angle: None,
        }
    }

    /// The pulse width for `angle`, clamped to the range.
    pub fn pulse_for(&self, angle: Angle) -> Duration {
        let fraction = self.fraction(angle);
        let span = self.config.max_pulse.saturating_sub(self.config.min_pulse);
        self.config.min_pulse + span.mul_f32(fraction)
    }

    /// Moves to `angle`, clamped to the range, and returns how long the move
    /// takes.
    ///
    /// Sends pulses if it was not already. Until an angle has been set, the
    /// servo could be anywhere, so the first move is given the full travel
    /// time.
    pub fn set(&mut self, angle: Angle) -> Duration {
        self.driver.set_pulse(self.pulse_for(angle));
        self.driver.set_enabled(true);
        let to = self.fraction(angle);
        let from = self.angle.map(|a| self.fraction(a));
        self.angle = Some(self.config.range * to);
        match from {
            Some(from) => self.config.travel.mul_f32(libm::fabsf(to - from)),
            None => self.config.travel,
        }
    }

    /// The angle last set, or `None` if none has been.
    pub fn angle(&self) -> Option<Angle> {
        self.angle
    }

    /// Stops sending pulses, letting the servo go limp.
    pub fn disable(&mut self) {
        self.driver.set_enabled(false);
    }

    /// Where `angle` falls in the range, from 0 to 1.
    fn fraction(&self, angle: Angle) -> f32 {
        let range = self.config.range.as_rad();
        if range <= 0.0 {
            return 0.0;
        }
        (angle.as_rad() / range).clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockServo;
    use crate::units::UnitExt;

    fn close(a: Duration, b: Duration) -> bool {
        a.abs_diff(b) < Duration::from_micros(1)
    }

    fn servo(config: ServoConfig) -> Servo<MockServo> {
        Servo::new(MockServo::default(), config)
    }

    #[test]
    fn maps_angles_to_pulses() {
        let s = servo(ServoConfig::default());
        assert_eq!(s.pulse_for(0.deg()), Duration::from_micros(1000));
        assert_eq!(s.pulse_for(180.deg()), Duration::from_micros(2000));
        assert!(close(s.pulse_for(45.deg()), Duration::from_micros(1250)));
    }

    #[test]
    fn clamps_to_the_range() {
        let s = servo(ServoConfig::default());
        assert_eq!(s.pulse_for((-30).deg()), Duration::from_micros(1000));
        assert_eq!(s.pulse_for(200.deg()), Duration::from_micros(2000));
    }

    #[test]
    fn matches_ninja_calibration() {
        // The gripper on ninja: 320 to 1200 µs over 180°, open at 50°.
        let s = servo(ServoConfig {
            min_pulse: Duration::from_micros(320),
            max_pulse: Duration::from_micros(1200),
            ..Default::default()
        });
        let pulse = s.pulse_for(50.deg()).as_secs_f32() * 1e6;
        assert!((pulse - (320.0 + 50.0 * 880.0 / 180.0)).abs() < 0.01);
    }

    #[test]
    fn set_drives_the_pulse_and_estimates_travel() {
        let mut s = servo(ServoConfig::default());
        assert!(!s.driver.enabled);
        // Unknown start: the full travel time.
        assert_eq!(s.set(90.deg()), Duration::from_millis(600));
        assert!(s.driver.enabled);
        assert!(close(s.driver.pulse, Duration::from_micros(1500)));
        assert!(close(s.set(30.deg()), Duration::from_millis(200)));
        assert!((s.angle().unwrap() - 30.deg()).abs() < 1e-4.deg());
        s.disable();
        assert!(!s.driver.enabled);
    }

    #[test]
    fn remembers_the_clamped_angle() {
        let mut s = servo(ServoConfig::default());
        s.set(500.deg());
        assert!((s.angle().unwrap() - 180.deg()).abs() < 1e-4.deg());
        assert!(close(s.set(180.deg()), Duration::ZERO));
    }
}

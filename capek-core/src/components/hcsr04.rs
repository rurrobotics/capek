//! The HC-SR04 ultrasonic range finder.

use super::DistanceSensor;
use crate::driver::PulseTimer;
use crate::units::{Length, Speed};
use core::time::Duration;

/// Settings of an [`Hcsr04`].
///
/// ```
/// use capek_core::components::Hcsr04Config;
/// use capek_core::units::UnitExt;
///
/// // Hotter air carries sound faster.
/// let config = Hcsr04Config { speed_of_sound: 349.m_per_s(), ..Default::default() };
/// assert_eq!(config.max, 4.m());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hcsr04Config {
    /// Speed of sound in air: 343 m/s at 20 °C.
    pub speed_of_sound: Speed,
    /// Closest distance the sensor measures: 20 mm.
    pub min: Length,
    /// Farthest distance the sensor measures: 4 m.
    pub max: Length,
    /// Length of the trigger pulse: 10 µs.
    pub trigger: Duration,
    /// How long to wait for an echo before deciding the sensor is not
    /// answering: 60 ms, beyond the 38 ms echo it gives when nothing is in
    /// range.
    pub timeout: Duration,
}

impl Default for Hcsr04Config {
    fn default() -> Self {
        Self {
            speed_of_sound: Speed::m_per_s(343.0),
            min: Length::mm(20.0),
            max: Length::m(4.0),
            trigger: Duration::from_micros(10),
            timeout: Duration::from_millis(60),
        }
    }
}

/// An HC-SR04 ultrasonic range finder, timed by a [`PulseTimer`].
///
/// The platform triggers the sensor and times its echo; this turns the time
/// into a distance. Readings are clamped to the sensor's range: with nothing
/// in range the HC-SR04 gives a long echo, which reads as the far end, so
/// open space is not mistaken for a fault. Only a missing echo, from a dead
/// or unplugged sensor, gives no reading.
///
/// ```
/// use capek_core::components::{DistanceSensor, Hcsr04, Hcsr04Config};
/// use capek_core::driver::PulseTimer;
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// /// An echo of 583 µs: about 100 mm there and back.
/// struct Echo;
/// impl PulseTimer for Echo {
///     async fn measure(&mut self, _t: Duration, _timeout: Duration) -> Option<Duration> {
///         Some(Duration::from_micros(583))
///     }
/// }
///
/// let mut sensor = Hcsr04::new(Echo, Hcsr04Config::default());
/// let d = embassy_futures::block_on(sensor.distance()).unwrap();
/// assert!((d - 100.mm()).abs() < 0.1.mm());
/// ```
#[derive(Debug)]
pub struct Hcsr04<D: PulseTimer> {
    timer: D,
    config: Hcsr04Config,
}

impl<D: PulseTimer> Hcsr04<D> {
    /// A sensor timed by `timer`.
    pub fn new(timer: D, config: Hcsr04Config) -> Self {
        Self { timer, config }
    }

    /// The distance an echo of `echo` corresponds to: the sound travels
    /// there and back.
    pub fn distance_for(&self, echo: Duration) -> Length {
        let mm = self.config.speed_of_sound.as_mm_per_s() * echo.as_secs_f32() / 2.0;
        Length::mm(mm.clamp(self.config.min.as_mm(), self.config.max.as_mm()))
    }
}

impl<D: PulseTimer> DistanceSensor for Hcsr04<D> {
    async fn distance(&mut self) -> Option<Length> {
        let echo = self
            .timer
            .measure(self.config.trigger, self.config.timeout)
            .await?;
        Some(self.distance_for(echo))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockPulseTimer;
    use crate::units::UnitExt;
    use embassy_futures::block_on;

    fn sensor(echo: Option<Duration>) -> Hcsr04<MockPulseTimer> {
        Hcsr04::new(MockPulseTimer::new(echo), Hcsr04Config::default())
    }

    #[test]
    fn converts_echo_to_distance() {
        let mut s = sensor(Some(Duration::from_micros(1166)));
        let d = block_on(s.distance()).unwrap();
        assert!((d - 200.mm()).abs() < 0.1.mm(), "{d}");
    }

    #[test]
    fn matches_ninja() {
        // Ninja: mm = µs · 34 / 200, which is 340 m/s.
        let s = Hcsr04::new(
            MockPulseTimer::new(None),
            Hcsr04Config {
                speed_of_sound: 340.m_per_s(),
                ..Default::default()
            },
        );
        let d = s.distance_for(Duration::from_micros(2000));
        assert!((d - (2000.0 * 34.0 / 200.0).mm()).abs() < 0.01.mm());
    }

    #[test]
    fn nothing_in_range_reads_as_the_far_end() {
        let mut s = sensor(Some(Duration::from_millis(38)));
        assert_eq!(block_on(s.distance()), Some(4.m()));
    }

    #[test]
    fn too_close_reads_as_the_near_end() {
        let mut s = sensor(Some(Duration::from_micros(30)));
        assert_eq!(block_on(s.distance()), Some(20.mm()));
    }

    #[test]
    fn no_echo_is_no_reading() {
        let mut s = sensor(None);
        assert_eq!(block_on(s.distance()), None);
    }

    #[test]
    fn asks_for_the_configured_pulse() {
        let mut s = sensor(Some(Duration::from_millis(1)));
        block_on(s.distance());
        assert_eq!(
            s.timer.asked,
            Some((Duration::from_micros(10), Duration::from_millis(60)))
        );
    }
}

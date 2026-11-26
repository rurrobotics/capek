//! Differential-drive kinematics and odometry.
//!
//! [`Differential`] holds the drive geometry and converts between the motion
//! of the robot's centre and the distance each wheel rolls. [`Pose`] is where
//! the robot is on the field; odometry updates it from the distances the
//! wheels actually rolled.
//!
//! Headings are counter-clockwise from the field's x axis, so a positive turn
//! drives the right wheel forward and the left wheel back. Wheel pairs are
//! always ordered `[left, right]`.
//!
//! # Examples
//!
//! Drive forward, turn left in place, and track the pose:
//!
//! ```
//! use capek_core::motion::kinematics::{Differential, Pose};
//! use capek_core::units::UnitExt;
//!
//! let drive = Differential::new(56.mm(), 159.mm());
//! let mut pose = Pose::ORIGIN;
//!
//! // 500 mm straight: both wheels roll 500 mm.
//! let [left, right] = drive.wheels(500.mm(), 0.deg());
//! pose = drive.odometry(pose, left, right);
//!
//! // A quarter turn in place: the wheels roll in opposite directions.
//! let [left, right] = drive.wheels(0.mm(), 90.deg());
//! assert_eq!(left, -right);
//! pose = drive.odometry(pose, left, right);
//!
//! assert!((pose.x - 500.mm()).abs() < 0.01.mm());
//! assert!((pose.heading - 90.deg()).abs() < 0.01.deg());
//! ```

use crate::units::{Angle, Length};
use core::f32::consts::PI;

/// Geometry of a differential drive: two driven wheels on one axle.
///
/// ```
/// use capek_core::motion::kinematics::Differential;
/// use capek_core::units::UnitExt;
///
/// let drive = Differential::new(56.mm(), 159.mm());
/// // One wheel turn rolls one circumference.
/// assert!((drive.wheel_distance(360.deg()) - 175.929.mm()).abs() < 0.01.mm());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Differential {
    /// Diameter of the driven wheels.
    pub wheel_diameter: Length,
    /// Distance between the contact points of the two wheels.
    pub track: Length,
}

impl Differential {
    /// A drive with the given wheel diameter and track width.
    ///
    /// ```
    /// use capek_core::motion::kinematics::Differential;
    /// use capek_core::units::UnitExt;
    ///
    /// let drive = Differential::new(56.mm(), 159.mm());
    /// assert_eq!(drive.track, 159.mm());
    /// ```
    pub const fn new(wheel_diameter: Length, track: Length) -> Self {
        Self {
            wheel_diameter,
            track,
        }
    }

    /// How far a wheel rolls when it turns by `rotation`.
    ///
    /// ```
    /// use capek_core::motion::kinematics::Differential;
    /// use capek_core::units::UnitExt;
    ///
    /// let drive = Differential::new(56.mm(), 159.mm());
    /// assert!((drive.wheel_distance(1.0.rad()) - 28.mm()).abs() < 1e-4.mm());
    /// ```
    pub fn wheel_distance(&self, rotation: Angle) -> Length {
        self.wheel_diameter * (rotation.as_rad() / 2.0)
    }

    /// How far a wheel turns to roll `distance`.
    ///
    /// ```
    /// use capek_core::motion::kinematics::Differential;
    /// use capek_core::units::UnitExt;
    ///
    /// let drive = Differential::new(56.mm(), 159.mm());
    /// assert!((drive.wheel_rotation(28.mm()) - 1.0.rad()).abs() < 1e-5.rad());
    /// ```
    pub fn wheel_rotation(&self, distance: Length) -> Angle {
        Angle::rad(2.0 * (distance / self.wheel_diameter))
    }

    /// Distance each wheel rolls when the robot turns by `angle` in place,
    /// the right wheel forward for a positive angle.
    ///
    /// ```
    /// use capek_core::motion::kinematics::Differential;
    /// use capek_core::units::UnitExt;
    ///
    /// let drive = Differential::new(56.mm(), 100.mm());
    /// // Half a turn: each wheel rolls half the circle the track spans.
    /// let d = drive.turn_distance(180.deg());
    /// assert!((d - (50.0 * core::f32::consts::PI).mm()).abs() < 1e-3.mm());
    /// ```
    pub fn turn_distance(&self, angle: Angle) -> Length {
        self.track * (angle.as_rad() / 2.0)
    }

    /// Distances `[left, right]` the wheels roll when the centre travels
    /// `centre` while the heading changes by `turn`.
    ///
    /// ```
    /// use capek_core::motion::kinematics::Differential;
    /// use capek_core::units::UnitExt;
    ///
    /// let drive = Differential::new(56.mm(), 100.mm());
    /// // A quarter of a circle of radius 200 mm, turning left.
    /// let arc = (100.0 * core::f32::consts::PI).mm();
    /// let [left, right] = drive.wheels(arc, 90.deg());
    /// assert!(right > arc && left < arc);
    /// ```
    pub fn wheels(&self, centre: Length, turn: Angle) -> [Length; 2] {
        let offset = self.turn_distance(turn);
        [centre - offset, centre + offset]
    }

    /// The distance the centre travels and the change in heading when the
    /// wheels roll `left` and `right`. The inverse of [`wheels`](Self::wheels).
    ///
    /// ```
    /// use capek_core::motion::kinematics::Differential;
    /// use capek_core::units::UnitExt;
    ///
    /// let drive = Differential::new(56.mm(), 100.mm());
    /// let (centre, turn) = drive.body(90.mm(), 110.mm());
    /// assert_eq!(centre, 100.mm());
    /// assert!((turn - 0.2.rad()).abs() < 1e-6.rad());
    /// ```
    pub fn body(&self, left: Length, right: Length) -> (Length, Angle) {
        let centre = (left + right) / 2.0;
        let turn = Angle::rad((right - left) / self.track);
        (centre, turn)
    }

    /// The pose reached from `pose` after the wheels roll `left` and `right`.
    ///
    /// Call it with the distances rolled since the last update, as often as
    /// steps are counted. Each update is integrated as an arc of constant
    /// curvature, which is exact for straight lines, in-place turns and
    /// circular arcs, so the update rate only matters on paths whose
    /// curvature changes.
    ///
    /// ```
    /// use capek_core::motion::kinematics::{Differential, Pose};
    /// use capek_core::units::UnitExt;
    ///
    /// let drive = Differential::new(56.mm(), 100.mm());
    /// // Half a circle of radius 200 mm in a single update.
    /// let arc = (200.0 * core::f32::consts::PI).mm();
    /// let [left, right] = drive.wheels(arc, 180.deg());
    /// let pose = drive.odometry(Pose::ORIGIN, left, right);
    /// assert!(pose.x.abs() < 0.01.mm());
    /// assert!((pose.y - 400.mm()).abs() < 0.01.mm());
    /// ```
    pub fn odometry(&self, pose: Pose, left: Length, right: Length) -> Pose {
        let (centre, turn) = self.body(left, right);
        pose.advanced(centre, turn)
    }
}

/// Position and heading of the robot on the field.
///
/// The heading is counter-clockwise from the x axis and kept in
/// `(-180°, 180°]`.
///
/// ```
/// use capek_core::motion::kinematics::Pose;
/// use capek_core::units::UnitExt;
///
/// let start = Pose::new(250.mm(), 1.m(), 90.deg());
/// let ahead = start.advanced(100.mm(), 0.deg());
/// assert!((ahead.y - 1.1.m()).abs() < 1e-3.mm());
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Pose {
    /// Position along the field's x axis.
    pub x: Length,
    /// Position along the field's y axis.
    pub y: Length,
    /// Heading, counter-clockwise from the x axis.
    pub heading: Angle,
}

impl Pose {
    /// At the field origin, facing along the x axis.
    pub const ORIGIN: Self = Self {
        x: Length::ZERO,
        y: Length::ZERO,
        heading: Angle::ZERO,
    };

    /// A pose at `x`, `y` facing `heading`, wrapped into `(-180°, 180°]`.
    ///
    /// ```
    /// use capek_core::motion::kinematics::Pose;
    /// use capek_core::units::UnitExt;
    ///
    /// let pose = Pose::new(0.mm(), 0.mm(), 270.deg());
    /// assert!((pose.heading - (-90).deg()).abs() < 1e-4.deg());
    /// ```
    pub fn new(x: Length, y: Length, heading: Angle) -> Self {
        Self {
            x,
            y,
            heading: wrap(heading),
        }
    }

    /// The pose reached by moving the centre `distance` along an arc of
    /// constant curvature while the heading changes by `turn`.
    ///
    /// A zero `turn` is a straight line and a zero `distance` an in-place
    /// turn.
    ///
    /// ```
    /// use capek_core::motion::kinematics::Pose;
    /// use capek_core::units::UnitExt;
    ///
    /// let pose = Pose::ORIGIN.advanced(0.mm(), 45.deg());
    /// assert_eq!(pose.x, 0.mm());
    /// assert!((pose.heading - 45.deg()).abs() < 1e-4.deg());
    /// ```
    pub fn advanced(self, distance: Length, turn: Angle) -> Self {
        let th = self.heading.as_rad();
        let dth = turn.as_rad();
        // The chord of the arc points along the mean heading and is shorter
        // than the arc by sin(dθ/2) / (dθ/2).
        let half = dth / 2.0;
        let chord = if libm::fabsf(half) < 1e-4 {
            1.0 - half * half / 6.0
        } else {
            libm::sinf(half) / half
        };
        let d = distance * chord;
        let mean = th + half;
        Self::new(
            self.x + d * libm::cosf(mean),
            self.y + d * libm::sinf(mean),
            Angle::rad(th + dth),
        )
    }
}

/// Wraps an angle into `(-180°, 180°]`.
fn wrap(angle: Angle) -> Angle {
    let mut a = libm::fmodf(angle.as_rad(), 2.0 * PI);
    if a <= -PI {
        a += 2.0 * PI;
    } else if a > PI {
        a -= 2.0 * PI;
    }
    Angle::rad(a)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::UnitExt;

    fn drive() -> Differential {
        Differential::new(56.mm(), 159.mm())
    }

    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn rotation_and_distance_are_inverse() {
        let d = drive();
        for mm in [-300.0, 0.0, 1.0, 175.929, 1000.0] {
            let back = d.wheel_distance(d.wheel_rotation(mm.mm()));
            assert!(close(back.as_mm(), mm, 1e-3));
        }
    }

    #[test]
    fn turn_matches_ninja() {
        // Ninja rolls each wheel degrees·π/360·track to turn in place.
        let ninja = 90.0 * PI / 360.0 * 117.5;
        let d = Differential::new(56.mm(), 117.5.mm());
        assert!(close(d.turn_distance(90.deg()).as_mm(), ninja, 1e-3));
    }

    #[test]
    fn wheels_and_body_are_inverse() {
        let d = drive();
        for (c, t) in [(0.0, 90.0), (500.0, 0.0), (-200.0, -30.0), (314.0, 180.0)] {
            let [l, r] = d.wheels(c.mm(), t.deg());
            let (centre, turn) = d.body(l, r);
            assert!(close(centre.as_mm(), c, 1e-3));
            assert!(close(turn.as_deg(), t, 1e-3));
        }
    }

    #[test]
    fn straight_and_backwards() {
        let pose = Pose::new(0.mm(), 0.mm(), 30.deg()).advanced((-100).mm(), 0.deg());
        assert!(close(pose.x.as_mm(), -100.0 * libm::cosf(PI / 6.0), 1e-3));
        assert!(close(pose.y.as_mm(), -50.0, 1e-3));
    }

    #[test]
    fn square_returns_to_start() {
        let d = drive();
        let mut pose = Pose::ORIGIN;
        for _ in 0..4 {
            let [l, r] = d.wheels(400.mm(), 0.deg());
            pose = d.odometry(pose, l, r);
            let [l, r] = d.wheels(0.mm(), 90.deg());
            pose = d.odometry(pose, l, r);
        }
        assert!(close(pose.x.as_mm(), 0.0, 1e-2));
        assert!(close(pose.y.as_mm(), 0.0, 1e-2));
        assert!(close(pose.heading.as_deg(), 0.0, 1e-3));
    }

    #[test]
    fn arc_is_exact_in_any_number_of_updates() {
        // A full circle of radius 300 mm, counter-clockwise.
        let d = drive();
        let length = 2.0 * PI * 300.0;
        for updates in [1, 4, 1000] {
            let mut pose = Pose::ORIGIN;
            let [l, r] = d.wheels(
                (length / updates as f32).mm(),
                (360.0 / updates as f32).deg(),
            );
            for i in 0..updates {
                pose = d.odometry(pose, l, r);
                if updates == 4 && i == 0 {
                    // A quarter of the circle ends at (300, 300).
                    assert!(close(pose.x.as_mm(), 300.0, 1e-2));
                    assert!(close(pose.y.as_mm(), 300.0, 1e-2));
                }
            }
            assert!(close(pose.x.as_mm(), 0.0, 0.05), "{updates}: {pose:?}");
            assert!(close(pose.y.as_mm(), 0.0, 0.05), "{updates}: {pose:?}");
        }
    }

    #[test]
    fn heading_wraps() {
        let pose = Pose::ORIGIN.advanced(0.mm(), 190.deg());
        assert!(close(pose.heading.as_deg(), -170.0, 1e-3));
        let pose = Pose::new(0.mm(), 0.mm(), (-180).deg());
        assert!(close(pose.heading.as_deg(), 180.0, 1e-3));
        let pose = Pose::new(0.mm(), 0.mm(), 720.deg());
        assert!(close(pose.heading.as_deg(), 0.0, 1e-3));
    }
}

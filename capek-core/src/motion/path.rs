//! Paths: where each axis is at each point along a move.
//!
//! A [`Path`] maps the path coordinate `s`, from zero to
//! [`length`](Path::length), to the position of every axis. A profile decides
//! how `s` changes over time; the path decides what it means for each axis.
//! `s` only moves forward, and the path decides which way each axis turns.
//!
//! For a differential drive, `s` is the distance travelled by the robot's
//! centre, or for an in-place turn the distance each wheel rolls, and the axes
//! are the distances rolled by the `[left, right]` wheels.
//!
//! # Examples
//!
//! Follow a turn with a profile:
//!
//! ```
//! use capek_core::motion::kinematics::Differential;
//! use capek_core::motion::path::{Path, Turn};
//! use capek_core::motion::profile::{Plan, Profile, State, Trapezoid};
//! use capek_core::units::UnitExt;
//! use core::time::Duration;
//!
//! let drive = Differential::new(56.mm(), 159.mm());
//! let turn = Turn::new(90.deg(), &drive);
//! let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
//!
//! let mut plan = profile.plan(State::REST, turn.length(), 0.mm_per_s());
//! let mut wheels = [0.mm(); 2];
//! while !plan.is_done() {
//!     wheels = turn.axes(plan.advance(Duration::from_millis(1)).s);
//! }
//! assert_eq!(wheels, drive.wheels(0.mm(), 90.deg()));
//! ```

use super::kinematics::Differential;
use crate::units::{Angle, Length};

/// Where each of `N` axes is at each point along a move.
///
/// Implement it to add a path of your own.
///
/// ```
/// use capek_core::motion::path::Path;
/// use capek_core::units::{Length, UnitExt};
///
/// /// Drives the left wheel only, pivoting around the right one.
/// struct Pivot(Length);
///
/// impl Path<2> for Pivot {
///     fn length(&self) -> Length {
///         self.0
///     }
///     fn axes(&self, s: Length) -> [Length; 2] {
///         [Length::mm(s.as_mm().clamp(0.0, self.0.as_mm())), Length::ZERO]
///     }
/// }
///
/// assert_eq!(Pivot(100.mm()).end(), [100.mm(), 0.mm()]);
/// ```
pub trait Path<const N: usize> {
    /// Length of the path, the value of `s` at its end. Never negative.
    fn length(&self) -> Length;

    /// Position of each axis at `s`, relative to the start of the path.
    ///
    /// `s` is clamped to `[0, length]`.
    fn axes(&self, s: Length) -> [Length; N];

    /// Position of each axis at the end of the path.
    fn end(&self) -> [Length; N] {
        self.axes(self.length())
    }
}

/// A straight move: every axis travels the same distance. Negative moves
/// backwards.
///
/// For a differential drive this is driving straight; for a single axis it is
/// a plain move.
///
/// ```
/// use capek_core::motion::path::{Line, Path};
/// use capek_core::units::UnitExt;
///
/// let back = Line((-200).mm());
/// assert_eq!(Path::<2>::length(&back), 200.mm());
/// assert_eq!(back.axes(50.mm()), [(-50).mm(), (-50).mm()]);
/// assert_eq!(back.axes(50.mm()), [(-50).mm()]);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Line(pub Length);

impl<const N: usize> Path<N> for Line {
    fn length(&self) -> Length {
        self.0.abs()
    }

    fn axes(&self, s: Length) -> [Length; N] {
        let s = clamp(s, self.0.abs());
        [if self.0 < Length::ZERO { -s } else { s }; N]
    }
}

/// An in-place turn of a differential drive. Positive is counter-clockwise.
///
/// `s` is the distance each wheel rolls: the right wheel forward and the left
/// wheel back for a positive angle.
///
/// ```
/// use capek_core::motion::kinematics::Differential;
/// use capek_core::motion::path::{Path, Turn};
/// use capek_core::units::UnitExt;
///
/// let drive = Differential::new(56.mm(), 100.mm());
/// let turn = Turn::new((-90).deg(), &drive);
/// // Each wheel rolls a quarter of the circle the track spans.
/// assert!((turn.length() - (25.0 * core::f32::consts::PI).mm()).abs() < 1e-3.mm());
/// assert_eq!(turn.axes(10.mm()), [10.mm(), (-10).mm()]);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Turn {
    /// The angle turned.
    pub angle: Angle,
    /// Distance the right wheel rolls, negative for a clockwise turn.
    right: Length,
}

impl Turn {
    /// A turn by `angle` with the geometry of `drive`.
    ///
    /// ```
    /// use capek_core::motion::kinematics::Differential;
    /// use capek_core::motion::path::{Path, Turn};
    /// use capek_core::units::UnitExt;
    ///
    /// let drive = Differential::new(56.mm(), 159.mm());
    /// let turn = Turn::new(180.deg(), &drive);
    /// assert_eq!(turn.length(), drive.turn_distance(180.deg()));
    /// ```
    pub fn new(angle: Angle, drive: &Differential) -> Self {
        Self {
            angle,
            right: drive.turn_distance(angle),
        }
    }
}

impl Path<2> for Turn {
    fn length(&self) -> Length {
        self.right.abs()
    }

    fn axes(&self, s: Length) -> [Length; 2] {
        let s = clamp(s, self.right.abs());
        let right = if self.right < Length::ZERO { -s } else { s };
        [-right, right]
    }
}

/// Clamps `s` to `[0, length]`.
fn clamp(s: Length, length: Length) -> Length {
    Length::mm(s.as_mm().clamp(0.0, length.as_mm()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::kinematics::Pose;
    use crate::units::UnitExt;

    fn drive() -> Differential {
        Differential::new(56.mm(), 159.mm())
    }

    #[test]
    fn line_moves_every_axis_together() {
        let line = Line(300.mm());
        assert_eq!(Path::<2>::length(&line), 300.mm());
        assert_eq!(line.axes(120.mm()), [120.mm(), 120.mm()]);
        assert_eq!(line.axes(120.mm()), [120.mm(); 3]);
        assert_eq!(Path::<1>::end(&line), [300.mm()]);
    }

    #[test]
    fn line_backwards() {
        let line = Line((-300).mm());
        assert_eq!(Path::<2>::length(&line), 300.mm());
        assert_eq!(Path::<2>::end(&line), [(-300).mm(); 2]);
    }

    #[test]
    fn s_is_clamped() {
        let line = Line(100.mm());
        assert_eq!(line.axes((-5).mm()), [0.mm(); 2]);
        assert_eq!(line.axes(150.mm()), [100.mm(); 2]);
        let turn = Turn::new(90.deg(), &drive());
        assert_eq!(turn.axes(1.m()), turn.end());
        assert_eq!(turn.axes((-1).m()), [0.mm(); 2]);
    }

    #[test]
    fn turn_ends_where_kinematics_says() {
        let d = drive();
        for deg in [90.0, -45.0, 180.0, 720.0] {
            let turn = Turn::new(deg.deg(), &d);
            assert_eq!(turn.end(), d.wheels(0.mm(), deg.deg()));
        }
    }

    #[test]
    fn turn_spins_in_place() {
        let d = drive();
        let turn = Turn::new(90.deg(), &d);
        let mut pose = Pose::ORIGIN;
        let mut last = [0.mm(); 2];
        // Walk along the turn and integrate odometry on the way.
        for i in 1..=100 {
            let axes = turn.axes(turn.length() * (i as f32 / 100.0));
            let (centre, _) = d.body(axes[0], axes[1]);
            assert_eq!(centre, 0.mm());
            pose = d.odometry(pose, axes[0] - last[0], axes[1] - last[1]);
            last = axes;
        }
        assert!(pose.x.abs() < 1e-3.mm() && pose.y.abs() < 1e-3.mm());
        assert!((pose.heading - 90.deg()).abs() < 1e-3.deg());
    }

    #[test]
    fn zero_paths_are_empty() {
        assert_eq!(Path::<2>::length(&Line(0.mm())), 0.mm());
        assert_eq!(Turn::new(0.deg(), &drive()).end(), [0.mm(); 2]);
    }
}

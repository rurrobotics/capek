//! Waypoint routes for a differential drive.
//!
//! A [`Route`] is a list of [`Waypoint`]s. [`RoutePath::new`] turns it into a
//! [`Path`], starting from the robot's current [`Pose`]:
//!
//! - The segment between two waypoints is a cubic Hermite curve whose end
//!   tangents follow the waypoints' slopes. A missing slope takes the
//!   direction of the straight line between the two waypoints, so a segment
//!   with no slope at either end is exactly a straight line.
//! - A waypoint with a slope is passed smoothly. At a waypoint without one
//!   where the route bends, the robot stops and turns in place.
//! - If the first segment does not leave along the current heading, the robot
//!   turns in place first.
//!
//! The path coordinate `s` is the distance travelled by the robot's centre,
//! and on in-place turns the distance each wheel rolls, as for
//! [`Line`](super::path::Line) and [`Turn`](super::path::Turn).
//!
//! # Examples
//!
//! ```
//! use capek_core::motion::kinematics::{Differential, Pose};
//! use capek_core::motion::path::Path;
//! use capek_core::motion::route::{Route, RoutePath, SegmentKind, Waypoint};
//! use capek_core::units::UnitExt;
//!
//! let drive = Differential::new(56.mm(), 159.mm());
//! let route = Route::from_slice(&[
//!     // Through (500, 0) heading 90°, without stopping...
//!     Waypoint::with_slope(500.mm(), 0.mm(), 90.deg()),
//!     // ...to (1000, 500), then stop and turn to drive back to the start.
//!     Waypoint::at(1000.mm(), 500.mm()),
//!     Waypoint::at(0.mm(), 0.mm()),
//! ])
//! .unwrap();
//!
//! let path = RoutePath::new(Pose::ORIGIN, &route, &drive);
//! let kinds: Vec<_> = path.segments().map(|s| s.kind).collect();
//! assert_eq!(
//!     kinds,
//!     [SegmentKind::Curve, SegmentKind::Curve, SegmentKind::Turn, SegmentKind::Straight]
//! );
//! # let _ = path.length();
//! ```

use super::kinematics::{Differential, Pose};
use super::path::Path;
use super::planner::{MAX_SPANS, Span};
use crate::units::{Angle, Length};
use core::f32::consts::PI;
use heapless::Vec;

/// Most waypoints a [`Route`] holds.
pub const MAX_WAYPOINTS: usize = 16;

/// Intervals in the arc-length table of each curved segment.
const SAMPLES: usize = 32;

/// Spans each curved segment is cut into for speed planning.
const CURVE_SPANS: usize = 8;

// A route of turns and curves must fit in one planned move.
const _: () = assert!(MAX_WAYPOINTS * (1 + CURVE_SPANS) <= MAX_SPANS);
const _: () = assert!(SAMPLES.is_multiple_of(CURVE_SPANS));

/// A point the robot drives through, with an optional heading there.
///
/// ```
/// use capek_core::motion::route::Waypoint;
/// use capek_core::units::UnitExt;
///
/// let corner = Waypoint::at(500.mm(), 0.mm());
/// let smooth = Waypoint::with_slope(500.mm(), 0.mm(), 90.deg());
/// assert_eq!(corner.slope, None);
/// assert_eq!(smooth.slope, Some(90.deg()));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    /// Position along the field's x axis.
    pub x: Length,
    /// Position along the field's y axis.
    pub y: Length,
    /// Heading the robot should have when it passes through, counter-clockwise
    /// from the x axis. Without one, the robot stops here and turns if the
    /// route bends.
    pub slope: Option<Angle>,
}

impl Waypoint {
    /// A waypoint without a slope.
    pub const fn at(x: Length, y: Length) -> Self {
        Self { x, y, slope: None }
    }

    /// A waypoint passed smoothly with heading `slope`.
    pub const fn with_slope(x: Length, y: Length, slope: Angle) -> Self {
        Self {
            x,
            y,
            slope: Some(slope),
        }
    }
}

/// A list of waypoints, driven in order from the robot's current pose.
pub type Route = Vec<Waypoint, MAX_WAYPOINTS>;

/// What a segment of a [`RoutePath`] does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentKind {
    /// Turns in place. `s` is the distance each wheel rolls.
    Turn,
    /// Drives straight.
    Straight,
    /// Follows a Hermite curve.
    Curve,
}

/// One segment of a [`RoutePath`], as seen from the outside.
///
/// ```
/// use capek_core::motion::kinematics::{Differential, Pose};
/// use capek_core::motion::route::{Route, RoutePath, SegmentKind, Waypoint};
/// use capek_core::units::UnitExt;
///
/// let drive = Differential::new(56.mm(), 159.mm());
/// let route = Route::from_slice(&[Waypoint::at(0.mm(), 300.mm())]).unwrap();
/// let path = RoutePath::new(Pose::ORIGIN, &route, &drive);
///
/// // Facing x, the robot first turns 90° to face the waypoint.
/// let first = path.segments().next().unwrap();
/// assert_eq!(first.kind, SegmentKind::Turn);
/// assert_eq!(first.length, drive.turn_distance(90.deg()));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RouteSegment {
    /// What the segment does.
    pub kind: SegmentKind,
    /// Path coordinate where the segment starts.
    pub start: Length,
    /// Length of the segment in the path coordinate.
    pub length: Length,
}

/// Arc length and heading sampled along a Hermite curve.
#[derive(Debug, Clone, PartialEq)]
struct Table {
    /// Arc length from the start of the curve, mm.
    s: [f32; SAMPLES + 1],
    /// Heading relative to the start of the curve, unwrapped, rad.
    heading: [f32; SAMPLES + 1],
}

impl Table {
    /// Heading change at arc length `s`, interpolated linearly.
    fn heading_at(&self, s: f32) -> f32 {
        // Index of the last sample at or before `s`.
        let i = self.s.partition_point(|&x| x <= s).clamp(1, SAMPLES) - 1;
        let span = self.s[i + 1] - self.s[i];
        let f = if span > 0.0 {
            ((s - self.s[i]) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };
        self.heading[i] + f * (self.heading[i + 1] - self.heading[i])
    }
}

/// A cubic Hermite curve with tangents as long as its chord.
struct Hermite {
    p0: [f32; 2],
    p1: [f32; 2],
    t0: [f32; 2],
    t1: [f32; 2],
}

impl Hermite {
    fn new(p0: [f32; 2], p1: [f32; 2], a0: f32, a1: f32) -> Self {
        let len = libm::hypotf(p1[0] - p0[0], p1[1] - p0[1]);
        Self {
            p0,
            p1,
            t0: [len * libm::cosf(a0), len * libm::sinf(a0)],
            t1: [len * libm::cosf(a1), len * libm::sinf(a1)],
        }
    }

    /// Derivative of the position with respect to the parameter `t`.
    fn velocity(&self, t: f32) -> [f32; 2] {
        let h00 = 6.0 * t * t - 6.0 * t;
        let h10 = 3.0 * t * t - 4.0 * t + 1.0;
        let h01 = -h00;
        let h11 = 3.0 * t * t - 2.0 * t;
        let mut v = [0.0; 2];
        for (k, out) in v.iter_mut().enumerate() {
            *out = h00 * self.p0[k] + h10 * self.t0[k] + h01 * self.p1[k] + h11 * self.t1[k];
        }
        v
    }

    fn speed(&self, t: f32) -> f32 {
        let v = self.velocity(t);
        libm::hypotf(v[0], v[1])
    }

    /// Arc length between parameters `a` and `b`, by five-point
    /// Gauss-Legendre quadrature.
    fn arc(&self, a: f32, b: f32) -> f32 {
        const NODES: [(f32, f32); 5] = [
            (0.0, 0.568_888_9),
            (-0.538_469_3, 0.478_628_7),
            (0.538_469_3, 0.478_628_7),
            (-0.906_179_8, 0.236_926_9),
            (0.906_179_8, 0.236_926_9),
        ];
        let (mid, half) = ((a + b) / 2.0, (b - a) / 2.0);
        NODES
            .iter()
            .map(|&(x, w)| w * self.speed(mid + half * x))
            .sum::<f32>()
            * half
    }

    fn table(&self) -> Table {
        let mut table = Table {
            s: [0.0; SAMPLES + 1],
            heading: [0.0; SAMPLES + 1],
        };
        let direction = |t: f32| {
            let v = self.velocity(t);
            libm::atan2f(v[1], v[0])
        };
        let mut prev = direction(0.0);
        for i in 1..=SAMPLES {
            let (a, b) = ((i - 1) as f32 / SAMPLES as f32, i as f32 / SAMPLES as f32);
            table.s[i] = table.s[i - 1] + self.arc(a, b);
            let dir = direction(b);
            table.heading[i] = table.heading[i - 1] + wrap(dir - prev);
            prev = dir;
        }
        table
    }
}

/// What a piece does, with what it needs to evaluate it.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Shape {
    /// In place, counter-clockwise for `1.0` and clockwise for `-1.0`.
    Turn(f32),
    Straight,
    /// Along the curve whose table has this index.
    Curve(usize),
}

/// One piece of a route, in mm and rad.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Piece {
    shape: Shape,
    /// Path coordinate at the start and length.
    start: f32,
    length: f32,
    /// Wheel positions and heading at the start.
    wheels: [f32; 2],
    heading: f32,
}

/// A [`Route`] turned into a [`Path`] for a differential drive.
///
/// The axes are the distances rolled by the `[left, right]` wheels since the
/// start of the route.
///
/// ```
/// use capek_core::motion::kinematics::{Differential, Pose};
/// use capek_core::motion::path::Path;
/// use capek_core::motion::route::{Route, RoutePath, Waypoint};
/// use capek_core::units::UnitExt;
///
/// let drive = Differential::new(56.mm(), 159.mm());
/// let route = Route::from_slice(&[Waypoint::at(400.mm(), 0.mm())]).unwrap();
/// let path = RoutePath::new(Pose::ORIGIN, &route, &drive);
/// assert_eq!(path.length(), 400.mm());
/// assert_eq!(path.end(), [400.mm(), 400.mm()]);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct RoutePath {
    drive: Differential,
    pieces: Vec<Piece, { 2 * MAX_WAYPOINTS }>,
    tables: Vec<Table, MAX_WAYPOINTS>,
    length: f32,
}

impl RoutePath {
    /// The path that drives `route` from `start` with the geometry of `drive`.
    ///
    /// Waypoints that coincide with the previous one are skipped.
    pub fn new(start: Pose, route: &Route, drive: &Differential) -> Self {
        let mut path = Self {
            drive: *drive,
            pieces: Vec::new(),
            tables: Vec::new(),
            length: 0.0,
        };
        let mut wheels = [0.0; 2];
        let mut heading = start.heading.as_rad();
        let mut from = [start.x.as_mm(), start.y.as_mm()];
        let mut from_slope = None;

        for w in route {
            let to = [w.x.as_mm(), w.y.as_mm()];
            let chord = libm::hypotf(to[0] - from[0], to[1] - from[1]);
            // Closer than a micrometre counts as the same point.
            if chord < 1e-3 {
                continue;
            }
            let direction = libm::atan2f(to[1] - from[1], to[0] - from[0]);
            let a0 = from_slope.map_or(direction, |a: Angle| a.as_rad());
            let a1 = w.slope.map_or(direction, |a| a.as_rad());

            let turn = wrap(a0 - heading);
            if libm::fabsf(turn) > 1e-4 {
                let length = libm::fabsf(drive.turn_distance(Angle::rad(turn)).as_mm());
                let sign = if turn > 0.0 { 1.0 } else { -1.0 };
                path.push(Shape::Turn(sign), length, &mut wheels, &mut heading);
            }

            if libm::fabsf(wrap(a0 - direction)) < 1e-4 && libm::fabsf(wrap(a1 - direction)) < 1e-4
            {
                path.push(Shape::Straight, chord, &mut wheels, &mut heading);
            } else {
                let table = Hermite::new(from, to, a0, a1).table();
                let length = table.s[SAMPLES];
                // At most one table per waypoint, so this always fits.
                let _ = path.tables.push(table);
                let index = path.tables.len() - 1;
                path.push(Shape::Curve(index), length, &mut wheels, &mut heading);
            }
            from = to;
            from_slope = w.slope;
        }
        path
    }

    /// Appends a piece and moves the running wheel positions and heading to
    /// its end.
    fn push(&mut self, shape: Shape, length: f32, wheels: &mut [f32; 2], heading: &mut f32) {
        let piece = Piece {
            shape,
            start: self.length,
            length,
            wheels: *wheels,
            heading: *heading,
        };
        // At most a turn and a segment per waypoint, so this always fits.
        let _ = self.pieces.push(piece);
        let (centre, turn) = self.local(&piece, length);
        let [l, r] = self.drive.wheels(Length::mm(centre), Angle::rad(turn));
        *wheels = [wheels[0] + l.as_mm(), wheels[1] + r.as_mm()];
        *heading += turn;
        self.length += length;
    }

    /// Distance travelled by the centre and heading change `s` into a piece.
    fn local(&self, piece: &Piece, s: f32) -> (f32, f32) {
        match piece.shape {
            Shape::Turn(sign) => (0.0, sign * 2.0 * s / self.drive.track.as_mm()),
            Shape::Straight => (s, 0.0),
            Shape::Curve(i) => (s, self.tables[i].heading_at(s)),
        }
    }

    /// The piece containing `s` and the distance into it.
    fn find(&self, s: Length) -> Option<(&Piece, f32)> {
        let s = s.as_mm().clamp(0.0, self.length);
        let i = self
            .pieces
            .partition_point(|p| p.start <= s)
            .checked_sub(1)?;
        let piece = &self.pieces[i];
        Some((piece, (s - piece.start).min(piece.length)))
    }

    /// Heading of the robot at `s`, in `(-180°, 180°]`.
    ///
    /// ```
    /// use capek_core::motion::kinematics::{Differential, Pose};
    /// use capek_core::motion::path::Path;
    /// use capek_core::motion::route::{Route, RoutePath, Waypoint};
    /// use capek_core::units::UnitExt;
    ///
    /// let drive = Differential::new(56.mm(), 159.mm());
    /// let route = Route::from_slice(&[
    ///     Waypoint::with_slope(500.mm(), 500.mm(), 90.deg()),
    /// ])
    /// .unwrap();
    /// let path = RoutePath::new(Pose::ORIGIN, &route, &drive);
    /// assert!((path.heading(path.length()) - 90.deg()).abs() < 0.01.deg());
    /// ```
    pub fn heading(&self, s: Length) -> Angle {
        let rad = match self.find(s) {
            Some((piece, local)) => piece.heading + self.local(piece, local).1,
            None => return Angle::ZERO,
        };
        Angle::rad(wrap(rad))
    }

    /// The segments of the path in order.
    ///
    /// The speed planner uses them to stop at turns and to slow down on
    /// curves.
    pub fn segments(&self) -> impl Iterator<Item = RouteSegment> + '_ {
        self.pieces.iter().map(|p| RouteSegment {
            kind: match p.shape {
                Shape::Turn(_) => SegmentKind::Turn,
                Shape::Straight => SegmentKind::Straight,
                Shape::Curve(_) => SegmentKind::Curve,
            },
            start: Length::mm(p.start),
            length: Length::mm(p.length),
        })
    }
}

impl RoutePath {
    /// The path cut into spans for the speed planner.
    ///
    /// Turns in place must start and end at rest. Each curve is cut into
    /// spans whose limits are scaled by `1 / (1 + |κ|·track / 2)`, with the
    /// highest curvature `κ` the robot drives in that span, so that the outer
    /// wheel stays within the profile's limits.
    ///
    /// ```
    /// use capek_core::motion::kinematics::{Differential, Pose};
    /// use capek_core::motion::route::{Route, RoutePath, Waypoint};
    /// use capek_core::units::UnitExt;
    ///
    /// let drive = Differential::new(56.mm(), 159.mm());
    /// let route = Route::from_slice(&[
    ///     Waypoint::at(500.mm(), 0.mm()),
    ///     Waypoint::at(500.mm(), 500.mm()),
    /// ])
    /// .unwrap();
    /// let spans = RoutePath::new(Pose::ORIGIN, &route, &drive).spans();
    ///
    /// // Straight, stop, turn, stop, straight.
    /// assert_eq!(spans.len(), 3);
    /// assert!(spans[0].rest_after && spans[1].rest_after && !spans[2].rest_after);
    /// ```
    pub fn spans(&self) -> Vec<Span, MAX_SPANS> {
        let mut spans: Vec<Span, MAX_SPANS> = Vec::new();
        let half_track = self.drive.track.as_mm() / 2.0;
        for piece in &self.pieces {
            match piece.shape {
                Shape::Turn(_) => {
                    if let Some(prev) = spans.last_mut() {
                        prev.rest_after = true;
                    }
                    let _ = spans.push(Span {
                        rest_after: true,
                        ..Span::new(Length::mm(piece.length))
                    });
                }
                Shape::Straight => {
                    let _ = spans.push(Span::new(Length::mm(piece.length)));
                }
                Shape::Curve(i) => {
                    let table = &self.tables[i];
                    let per = SAMPLES / CURVE_SPANS;
                    for k in 0..CURVE_SPANS {
                        let range = k * per..(k + 1) * per;
                        let mut kappa: f32 = 0.0;
                        for j in range.clone() {
                            let ds = table.s[j + 1] - table.s[j];
                            if ds > 0.0 {
                                let dh = table.heading[j + 1] - table.heading[j];
                                kappa = kappa.max(libm::fabsf(dh / ds));
                            }
                        }
                        let length = table.s[range.end] - table.s[range.start];
                        let _ = spans.push(Span {
                            scale: 1.0 / (1.0 + kappa * half_track),
                            ..Span::new(Length::mm(length))
                        });
                    }
                }
            }
        }
        spans
    }
}

impl Path<2> for RoutePath {
    fn length(&self) -> Length {
        Length::mm(self.length)
    }

    fn axes(&self, s: Length) -> [Length; 2] {
        let Some((piece, local)) = self.find(s) else {
            return [Length::ZERO; 2];
        };
        let (centre, turn) = self.local(piece, local);
        let [l, r] = self.drive.wheels(Length::mm(centre), Angle::rad(turn));
        [
            Length::mm(piece.wheels[0]) + l,
            Length::mm(piece.wheels[1]) + r,
        ]
    }
}

/// Wraps an angle in radians into `(-π, π]`.
fn wrap(a: f32) -> f32 {
    let mut a = libm::fmodf(a, 2.0 * PI);
    if a <= -PI {
        a += 2.0 * PI;
    } else if a > PI {
        a -= 2.0 * PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::UnitExt;

    fn drive() -> Differential {
        Differential::new(56.mm(), 159.mm())
    }

    fn route(points: &[Waypoint]) -> Route {
        Route::from_slice(points).unwrap()
    }

    fn kinds(path: &RoutePath) -> std::vec::Vec<SegmentKind> {
        path.segments().map(|s| s.kind).collect()
    }

    /// Walks the path in small steps and integrates odometry from the wheel
    /// positions, checking that they never jump.
    fn drive_along(path: &RoutePath, start: Pose) -> Pose {
        let d = drive();
        let steps = (path.length().as_mm() / 0.5) as usize + 1;
        let mut pose = start;
        let mut last = [0.mm(); 2];
        for i in 1..=steps {
            let axes = path.axes(path.length() * (i as f32 / steps as f32));
            for k in 0..2 {
                assert!((axes[k] - last[k]).abs() < 2.mm(), "wheel jump at {i}");
            }
            pose = d.odometry(pose, axes[0] - last[0], axes[1] - last[1]);
            last = axes;
        }
        pose
    }

    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn segments_without_slopes_are_straight_lines() {
        let path = RoutePath::new(
            Pose::ORIGIN,
            &route(&[
                Waypoint::at(300.mm(), 0.mm()),
                Waypoint::at(600.mm(), 0.mm()),
            ]),
            &drive(),
        );
        // Collinear: no turn at the middle waypoint.
        assert_eq!(kinds(&path), [SegmentKind::Straight, SegmentKind::Straight]);
        assert_eq!(path.length(), 600.mm());
        assert_eq!(path.axes(450.mm()), [450.mm(), 450.mm()]);
    }

    #[test]
    fn corners_stop_and_turn() {
        let d = drive();
        let path = RoutePath::new(
            Pose::ORIGIN,
            &route(&[
                Waypoint::at(300.mm(), 0.mm()),
                Waypoint::at(300.mm(), 300.mm()),
            ]),
            &d,
        );
        assert_eq!(
            kinds(&path),
            [
                SegmentKind::Straight,
                SegmentKind::Turn,
                SegmentKind::Straight
            ]
        );
        let turn = path.segments().nth(1).unwrap();
        assert_eq!(turn.length, d.turn_distance(90.deg()));
        let end = drive_along(&path, Pose::ORIGIN);
        assert!(close(end.x.as_mm(), 300.0, 1e-2) && close(end.y.as_mm(), 300.0, 1e-2));
        assert!(close(end.heading.as_deg(), 90.0, 1e-2));
    }

    #[test]
    fn turns_take_the_short_way() {
        let path = RoutePath::new(
            Pose::new(0.mm(), 0.mm(), 170.deg()),
            &route(&[Waypoint::at(0.mm(), (-300).mm())]),
            &drive(),
        );
        // From 170° to -90°: 100° counter-clockwise.
        let turn = path.segments().next().unwrap();
        assert_eq!(turn.kind, SegmentKind::Turn);
        assert!(close(
            turn.length.as_mm(),
            drive().turn_distance(100.deg()).as_mm(),
            1e-3
        ));
        assert!(path.axes(turn.length)[1] > 0.mm());
    }

    #[test]
    fn turns_first_when_not_facing_the_route() {
        let path = RoutePath::new(
            Pose::new(100.mm(), 100.mm(), 0.deg()),
            &route(&[Waypoint::at(100.mm(), 500.mm())]),
            &drive(),
        );
        assert_eq!(kinds(&path), [SegmentKind::Turn, SegmentKind::Straight]);
    }

    #[test]
    fn slopes_give_matching_tangents_at_joints() {
        let d = drive();
        let path = RoutePath::new(
            Pose::ORIGIN,
            &route(&[
                Waypoint::with_slope(500.mm(), 200.mm(), 45.deg()),
                Waypoint::with_slope(700.mm(), 700.mm(), 120.deg()),
                Waypoint::at(200.mm(), 900.mm()),
            ]),
            &d,
        );
        // A turn to face along the first chord, then smooth all the way.
        let k = kinds(&path);
        assert_eq!(k[0], SegmentKind::Turn);
        assert!(k[1..].iter().all(|&k| k == SegmentKind::Curve));
        for seg in path.segments().skip(2) {
            let before = path.heading(seg.start - 0.01.mm());
            let after = path.heading(seg.start + 0.01.mm());
            assert!((before - after).abs() < 0.1.deg(), "{before} vs {after}");
        }
        let second = path.segments().nth(2).unwrap().start;
        assert!((path.heading(second) - 45.deg()).abs() < 0.01.deg());
    }

    #[test]
    fn odometry_ends_at_the_last_waypoint() {
        let routes = [
            route(&[
                Waypoint::with_slope(500.mm(), 0.mm(), 90.deg()),
                Waypoint::at(1000.mm(), 500.mm()),
                Waypoint::at(0.mm(), 0.mm()),
            ]),
            route(&[
                Waypoint::with_slope(400.mm(), 400.mm(), 0.deg()),
                Waypoint::with_slope(800.mm(), 0.mm(), (-90).deg()),
                Waypoint::with_slope(400.mm(), (-400).mm(), 180.deg()),
            ]),
        ];
        for r in &routes {
            let path = RoutePath::new(Pose::ORIGIN, r, &drive());
            let end = drive_along(&path, Pose::ORIGIN);
            let last = r.last().unwrap();
            assert!(close(end.x.as_mm(), last.x.as_mm(), 1.0), "{end:?}");
            assert!(close(end.y.as_mm(), last.y.as_mm(), 1.0), "{end:?}");
            let diff = wrap((end.heading - path.heading(path.length())).as_rad());
            assert!(diff.abs() < 0.1_f32.to_radians(), "{end:?}");
        }
    }

    #[test]
    fn arc_length_tables_match_numeric_integration() {
        let h = Hermite::new([0.0, 0.0], [500.0, 300.0], 0.0, 1.5);
        let table = h.table();
        // Simpson's rule with many panels.
        let n = 10_000;
        let mut sum = h.speed(0.0) + h.speed(1.0);
        for i in 1..n {
            let w = if i % 2 == 1 { 4.0 } else { 2.0 };
            sum += w * h.speed(i as f32 / n as f32);
        }
        let exact = sum / (3.0 * n as f32);
        assert!(close(table.s[SAMPLES], exact, 1e-3 * exact));
        // The table ends at the end tangent.
        assert!(close(table.heading[SAMPLES], 1.5, 1e-4));
    }

    #[test]
    fn coincident_waypoints_are_skipped() {
        let path = RoutePath::new(
            Pose::ORIGIN,
            &route(&[
                Waypoint::at(0.mm(), 0.mm()),
                Waypoint::at(200.mm(), 0.mm()),
                Waypoint::at(200.mm(), 0.mm()),
            ]),
            &drive(),
        );
        assert_eq!(kinds(&path), [SegmentKind::Straight]);
    }

    #[test]
    fn empty_route_is_empty() {
        let path = RoutePath::new(Pose::ORIGIN, &Route::new(), &drive());
        assert_eq!(path.length(), 0.mm());
        assert_eq!(path.axes(10.mm()), [0.mm(); 2]);
        assert_eq!(path.heading(0.mm()), 0.deg());
    }
}

//! Speed planning along a path.
//!
//! A path is cut into [`Span`]s, stretches with one set of limits: a straight
//! line, an in-place turn, or a short piece of a curve. On a curve the outer
//! wheel moves faster than the centre, so a curved span carries a `scale`
//! below one, and the profile plans it with every limit multiplied by that
//! scale. [`plan_speeds`] then picks the speed at every junction between
//! spans:
//!
//! 1. Each junction starts at the lower cruise speed of its two spans, or at
//!    zero where the robot must stop, such as around an in-place turn.
//! 2. A backward pass lowers each junction to what the profile can brake down
//!    from in time for the next one.
//! 3. A forward pass lowers each junction to what the profile can reach from
//!    the previous one.
//!
//! The result is one [`Piece`] per span, which the profile plans with
//! `profile.scaled(piece.scale).plan(state, piece.length, piece.end_speed)`.
//! Both passes ask the profile itself through [`Profile::reachable`], so every
//! planned junction speed is one the plans can actually reach.
//!
//! # Examples
//!
//! ```
//! use capek_core::motion::planner::{plan_speeds, Span};
//! use capek_core::motion::profile::{Profile, Trapezoid};
//! use capek_core::units::UnitExt;
//!
//! let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
//! let spans = [
//!     Span::new(1.m()),
//!     // A curved stretch where the outer wheel is 1.6 times faster.
//!     Span { scale: 1.0 / 1.6, ..Span::new(200.mm()) },
//!     Span::new(1.m()),
//! ];
//! let pieces = plan_speeds(&profile, &spans, 0.mm_per_s(), 0.mm_per_s());
//!
//! // Enter and leave the curve at its scaled cruise speed, and stop at the end.
//! assert_eq!(pieces[0].end_speed, 250.mm_per_s());
//! assert_eq!(pieces[1].end_speed, 250.mm_per_s());
//! assert_eq!(pieces[2].end_speed, 0.mm_per_s());
//! ```

use super::profile::Profile;
use crate::units::{Length, Speed};
use heapless::Vec;

/// Most spans a single planned move holds: enough for a full route.
pub const MAX_SPANS: usize = 144;

/// A stretch of a path with one set of limits.
///
/// ```
/// use capek_core::motion::planner::Span;
/// use capek_core::units::UnitExt;
///
/// let straight = Span::new(500.mm());
/// assert_eq!(straight.scale, 1.0);
/// assert!(!straight.rest_after);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Span {
    /// Length in the path coordinate.
    pub length: Length,
    /// Factor applied to every limit of the profile on this span, at most
    /// one. On a curve it is `1 / (1 + |κ|·track / 2)`, so that the outer
    /// wheel stays within the real limits.
    pub scale: f32,
    /// Whether the robot must be at rest at the end of this span, for example
    /// before or after an in-place turn.
    pub rest_after: bool,
}

impl Span {
    /// A span with the profile's own limits that does not need to stop.
    pub const fn new(length: Length) -> Self {
        Self {
            length,
            scale: 1.0,
            rest_after: false,
        }
    }
}

/// A span with the speed planned for its end.
///
/// ```
/// use capek_core::motion::planner::{plan_speeds, Span};
/// use capek_core::motion::profile::{Plan, Profile, State, Trapezoid};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
/// let spans = [Span::new(300.mm()), Span { scale: 0.5, ..Span::new(300.mm()) }];
///
/// // Run the pieces one after another.
/// let mut state = State::REST;
/// for piece in plan_speeds(&profile, &spans, 0.mm_per_s(), 0.mm_per_s()) {
///     let mut plan = profile.scaled(piece.scale).plan(state, piece.length, piece.end_speed);
///     while !plan.is_done() {
///         state = plan.advance(Duration::from_millis(1));
///     }
/// }
/// assert_eq!(state, State::at(600.mm()));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Piece {
    /// Length in the path coordinate.
    pub length: Length,
    /// Factor to scale the profile by, as in [`Span::scale`].
    pub scale: f32,
    /// Speed at the end of the piece.
    pub end_speed: Speed,
}

/// Plans the speed at the end of every span, starting at `start` and ending
/// at `end`.
///
/// Spans with no length are dropped. The start speed is taken as it is: when
/// the robot is already too fast to slow down for what follows, the first
/// plans end at the closest speeds they can, as every plan does.
pub fn plan_speeds<P: Profile>(
    profile: &P,
    spans: &[Span],
    start: Speed,
    end: Speed,
) -> Vec<Piece, MAX_SPANS> {
    let mut pieces: Vec<Piece, MAX_SPANS> = Vec::new();
    // The scaled profile of each span, kept alongside to avoid rebuilding it.
    let mut scaled: Vec<P, MAX_SPANS> = Vec::new();
    for span in spans.iter().filter(|s| s.length > Length::ZERO) {
        let p = profile.scaled(span.scale);
        let cap = if span.rest_after {
            Speed::ZERO
        } else {
            p.max_speed()
        };
        if let Some(prev) = pieces.last_mut() {
            prev.end_speed = min(prev.end_speed, p.max_speed());
        }
        // Spans beyond the capacity are dropped; no path produces that many.
        if pieces
            .push(Piece {
                length: span.length,
                scale: span.scale,
                end_speed: cap,
            })
            .is_err()
        {
            break;
        }
        let _ = scaled.push(p);
    }
    let n = pieces.len();
    if n == 0 {
        return pieces;
    }
    pieces[n - 1].end_speed = min(pieces[n - 1].end_speed, end.max_zero());

    // Backward: each junction no faster than the span after it can brake
    // from in time for the junction that follows.
    for i in (0..n - 1).rev() {
        let next = scaled[i + 1].reachable(pieces[i + 1].end_speed, pieces[i + 1].length);
        pieces[i].end_speed = min(pieces[i].end_speed, next);
    }
    // Forward: each junction no faster than the span before it can reach.
    let mut v = start.max_zero();
    for (piece, p) in pieces.iter_mut().zip(scaled.iter()) {
        piece.end_speed = min(piece.end_speed, p.reachable(v, piece.length));
        v = piece.end_speed;
    }
    pieces
}

fn min(a: Speed, b: Speed) -> Speed {
    if b < a { b } else { a }
}

/// Clamping to non-negative speeds.
trait MaxZero {
    fn max_zero(self) -> Self;
}

impl MaxZero for Speed {
    fn max_zero(self) -> Self {
        if self < Speed::ZERO {
            Speed::ZERO
        } else {
            self
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::kinematics::{Differential, Pose};
    use crate::motion::path::Path;
    use crate::motion::profile::{Plan, SCurve, State, Trapezoid};
    use crate::motion::route::{Route, RoutePath, SegmentKind, Waypoint};
    use crate::units::UnitExt;
    use core::time::Duration;

    const DT: Duration = Duration::from_millis(1);

    fn trapezoid() -> Trapezoid {
        Trapezoid {
            max_speed: 400.mm_per_s(),
            accel: 1200.mm_per_s2(),
        }
    }

    fn scurve() -> SCurve {
        SCurve {
            max_speed: 400.mm_per_s(),
            accel: 1200.mm_per_s2(),
            jerk: 8000.mm_per_s3(),
        }
    }

    fn drive() -> Differential {
        Differential::new(56.mm(), 159.mm())
    }

    /// A route with a tight U-turn and a corner.
    fn tight_route() -> RoutePath {
        let route = Route::from_slice(&[
            Waypoint::with_slope(600.mm(), 0.mm(), 0.deg()),
            Waypoint::with_slope(600.mm(), 200.mm(), 180.deg()),
            Waypoint::at(0.mm(), 200.mm()),
            Waypoint::at(0.mm(), 600.mm()),
        ])
        .unwrap();
        RoutePath::new(Pose::ORIGIN, &route, &drive())
    }

    /// Runs the pieces along `path` and returns the highest wheel speed and
    /// the path positions where the robot was at rest. Checks on the way that
    /// every plan reaches its planned end speed.
    fn drive_along<P: Profile>(profile: &P, path: &RoutePath, pieces: &[Piece]) -> (f32, f32) {
        let mut state = State::REST;
        let mut last = path.axes(0.mm());
        let mut fastest: f32 = 0.0;
        for piece in pieces {
            let mut plan = profile
                .scaled(piece.scale)
                .plan(state, piece.length, piece.end_speed);
            while !plan.is_done() {
                state = plan.advance(DT);
                let axes = path.axes(state.s);
                for k in 0..2 {
                    let v = (axes[k] - last[k]).abs().as_mm() / DT.as_secs_f32();
                    fastest = fastest.max(v);
                }
                last = axes;
            }
            let tol = 0.05 + piece.end_speed.as_mm_per_s() * 1e-3;
            assert!(
                (state.v - piece.end_speed).abs().as_mm_per_s() <= tol,
                "planned {:?}, reached {:?}",
                piece.end_speed,
                state.v
            );
        }
        (fastest, (state.s - path.length()).abs().as_mm())
    }

    #[test]
    fn straight_line_is_one_piece() {
        let pieces = plan_speeds(
            &trapezoid(),
            &[Span::new(1.m())],
            0.mm_per_s(),
            0.mm_per_s(),
        );
        assert_eq!(pieces.len(), 1);
        assert_eq!(pieces[0].end_speed, 0.mm_per_s());
    }

    #[test]
    fn caps_at_the_slower_span() {
        let spans = [
            Span::new(1.m()),
            Span {
                scale: 0.5,
                ..Span::new(500.mm())
            },
            Span::new(1.m()),
        ];
        let pieces = plan_speeds(&trapezoid(), &spans, 0.mm_per_s(), 100.mm_per_s());
        assert_eq!(pieces[0].end_speed, 200.mm_per_s());
        assert_eq!(pieces[1].end_speed, 200.mm_per_s());
        assert_eq!(pieces[2].end_speed, 100.mm_per_s());
    }

    #[test]
    fn short_spans_limit_by_reachability() {
        // Too short to reach cruise speed before having to stop.
        let spans = [
            Span::new(20.mm()),
            Span {
                rest_after: true,
                ..Span::new(20.mm())
            },
        ];
        let p = trapezoid();
        let pieces = plan_speeds(&p, &spans, 0.mm_per_s(), 0.mm_per_s());
        let v = p.reachable(0.mm_per_s(), 20.mm());
        assert_eq!(pieces[0].end_speed, v);
        assert_eq!(pieces[1].end_speed, 0.mm_per_s());
    }

    #[test]
    fn corner_speed_is_zero() {
        let path = tight_route();
        let pieces = plan_route(&scurve(), &path);
        let turns: std::vec::Vec<_> = path
            .segments()
            .filter(|s| s.kind == SegmentKind::Turn)
            .map(|s| (s.start, s.start + s.length))
            .collect();
        assert!(!turns.is_empty());
        // Every junction at either end of a turn is planned at rest.
        let mut s = 0.mm();
        for piece in &pieces {
            s += piece.length;
            for &(a, b) in &turns {
                if (s - a).abs() < 1e-3.mm() || (s - b).abs() < 1e-3.mm() {
                    assert_eq!(piece.end_speed, 0.mm_per_s(), "at {s}");
                }
            }
        }
    }

    #[test]
    fn no_wheel_exceeds_its_limit_on_tight_curves() {
        let path = tight_route();
        let spans = path.spans(&trapezoid());
        assert!(
            spans.iter().any(|s| s.scale < 0.7),
            "the route should be tight"
        );
        for (name, fastest, gap) in [
            {
                let p = trapezoid();
                let (f, g) = drive_along(&p, &path, &plan_route(&p, &path));
                ("trapezoid", f, g)
            },
            {
                let p = scurve();
                let (f, g) = drive_along(&p, &path, &plan_route(&p, &path));
                ("scurve", f, g)
            },
        ] {
            assert!(
                fastest <= 400.0 * 1.01,
                "{name}: a wheel reached {fastest} mm/s"
            );
            assert!(gap < 1e-3, "{name}: ended {gap} mm from the end");
        }
    }

    /// The highest wheel acceleration, and where, driving the pieces along
    /// `path`.
    ///
    /// Each wheel's acceleration is `a·dw/ds + v²·d²w/ds²`, from the plan's
    /// speed and acceleration and the path's geometry, so that it does not
    /// depend on how the plans are stitched together in time.
    fn wheel_accel<P: Profile>(profile: &P, path: &RoutePath, pieces: &[Piece]) -> (f32, Length) {
        let h = 0.5;
        let mut state = State::REST;
        let mut worst = (0.0_f32, 0.mm());
        for piece in pieces {
            let mut plan = profile
                .scaled(piece.scale)
                .plan(state, piece.length, piece.end_speed);
            while !plan.is_done() {
                state = plan.advance(DT);
                // Keep the stencil inside one segment: at the stops around a
                // turn the wheels change direction, which is fine at rest.
                let seg = path
                    .segments()
                    .filter(|g| g.start <= state.s)
                    .last()
                    .unwrap();
                let (lo, hi) = (seg.start.as_mm() + h, (seg.start + seg.length).as_mm() - h);
                let s = state.s.as_mm().clamp(lo, hi.max(lo));
                let [a, b, c] = [s - h, s, s + h].map(|x| path.axes(x.mm()));
                for k in 0..2 {
                    let d1 = (c[k] - a[k]).as_mm() / (2.0 * h);
                    let d2 = (c[k] - b[k] * 2.0 + a[k]).as_mm() / (h * h);
                    let (v, acc) = (state.v.as_mm_per_s(), state.a.as_mm_per_s2());
                    let wheel = (acc * d1 + v * v * d2).abs();
                    if wheel > worst.0 {
                        worst = (wheel, state.s);
                    }
                }
            }
        }
        worst
    }

    /// Plans `path` for `profile` from rest to rest.
    fn plan_route<P: Profile>(profile: &P, path: &RoutePath) -> Vec<Piece, MAX_SPANS> {
        plan_speeds(profile, &path.spans(profile), 0.mm_per_s(), 0.mm_per_s())
    }

    #[test]
    fn no_wheel_exceeds_its_acceleration_on_tight_curves() {
        let path = tight_route();
        let p = trapezoid();
        let (worst, at) = wheel_accel(&p, &path, &plan_route(&p, &path));
        assert!(worst <= 1200.0 * 1.03, "trapezoid: {worst} mm/s² at {at}");
        let p = scurve();
        let (worst, at) = wheel_accel(&p, &path, &plan_route(&p, &path));
        assert!(worst <= 1200.0 * 1.03, "scurve: {worst} mm/s² at {at}");
    }

    #[test]
    fn without_scaling_the_outer_wheel_would_be_too_fast() {
        // The same route planned as if it were straight.
        let path = tight_route();
        let p = trapezoid();
        let spans: std::vec::Vec<_> = path
            .spans(&p)
            .iter()
            .map(|s| Span { scale: 1.0, ..*s })
            .collect();
        let (fastest, _) = drive_along(
            &p,
            &path,
            &plan_speeds(&p, &spans, 0.mm_per_s(), 0.mm_per_s()),
        );
        assert!(fastest > 400.0 * 1.2, "{fastest}");
    }

    #[test]
    fn empty_input_is_empty() {
        assert!(plan_speeds(&trapezoid(), &[], 0.mm_per_s(), 0.mm_per_s()).is_empty());
        let zero = [Span::new(0.mm())];
        assert!(plan_speeds(&trapezoid(), &zero, 0.mm_per_s(), 0.mm_per_s()).is_empty());
    }
}

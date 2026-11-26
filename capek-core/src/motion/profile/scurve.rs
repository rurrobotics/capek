use super::{Plan, Profile, State};
use crate::units::{Acceleration, Jerk, Length, Speed};
use core::time::Duration;

/// Jerk-limited motion: acceleration ramps up and down at `jerk`, never
/// jumping.
///
/// A rest-to-rest move has the classic seven phases: jerk up, constant
/// acceleration, jerk down, cruise, and the same three mirrored to stop.
/// Short speed changes skip the constant acceleration, and short moves skip
/// the cruise. Every state is computed in closed form.
///
/// Unlike the other profiles, `SCurve` starts from the acceleration in the
/// state it is given, so [`stop`](Profile::stop) and replanning in the middle
/// of a ramp keep acceleration continuous. Two cases cannot be continuous:
///
/// - a starting acceleration beyond `accel` is clamped to it;
/// - a deceleration too strong to bring to zero before the speed reaches
///   zero is reduced to the strongest one that can be.
///
/// When the distance is too short even to bring the starting acceleration
/// back to zero, the plan ends at its distance still accelerating, and the
/// next plan carries on from there.
///
/// ```
/// use capek_core::motion::profile::{Plan, Profile, SCurve, State};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let profile = SCurve {
///     max_speed: 400.mm_per_s(),
///     accel: 1200.mm_per_s2(),
///     jerk: 8000.mm_per_s3(),
/// };
/// let mut plan = profile.plan(State::REST, 1.m(), 0.mm_per_s());
///
/// // Halfway up the first jerk phase, then stop smoothly from there.
/// let state = plan.advance(Duration::from_millis(75));
/// assert!((state.a - 600.mm_per_s2()).abs() < 1.mm_per_s2());
/// let mut stop = profile.stop(state);
/// let next = stop.advance(Duration::from_millis(1));
/// assert!((next.a - state.a).abs() < 10.mm_per_s2());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SCurve {
    /// Cruise speed. Must be positive.
    pub max_speed: Speed,
    /// Peak acceleration and deceleration. Must be positive.
    pub accel: Acceleration,
    /// Rate at which acceleration changes. Must be positive.
    pub jerk: Jerk,
}

/// The limits in mm, s.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Limits {
    vmax: f32,
    a: f32,
    j: f32,
}

/// A jerk-limited speed change from `vs` with acceleration `a_s` to `ve` with
/// no acceleration.
///
/// Measured in the direction of the change (`sigma`), the acceleration goes
/// from `alpha` to the peak `ap` at jerk `j`, holds, then returns to zero at
/// jerk `j`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Ramp {
    vs: f32,
    ve: f32,
    sigma: f32,
    alpha: f32,
    ap: f32,
    j: f32,
    /// Ends of the three phases, s.
    t1: f32,
    t2: f32,
    t3: f32,
    /// Distance from the start of the ramp and speed at the ends of the
    /// first two phases.
    s1: f32,
    v1: f32,
    s2: f32,
    v2: f32,
    /// Total distance.
    dist: f32,
}

impl Ramp {
    /// The ramp from `(vs, a_s)` to `(ve, 0)`.
    ///
    /// It must be feasible: either the start acceleration points away from
    /// `ve`, or there is room to bring it to zero before reaching `ve`. See
    /// [`Limits::ramp_to`].
    fn new(vs: f32, a_s: f32, ve: f32, lim: &Limits) -> Self {
        let (a, j) = (lim.a, lim.j);
        let sigma = direction(vs, a_s, ve);
        let delta = libm::fabsf(ve - vs);
        let alpha = sigma * a_s;

        let mut ap = if 2.0 * j * delta + alpha * alpha >= 2.0 * a * a {
            a
        } else {
            libm::sqrtf((2.0 * j * delta + alpha * alpha) / 2.0)
        };
        // Rounding can leave the peak just below a positive start.
        ap = ap.max(alpha);

        let d1 = (ap - alpha) / j;
        let dv2 = (delta - (ap * ap - alpha * alpha) / (2.0 * j) - ap * ap / (2.0 * j)).max(0.0);
        let d2 = if ap > 0.0 { dv2 / ap } else { 0.0 };
        let d3 = ap / j;

        let v1 = vs + sigma * (alpha * d1 + j * d1 * d1 / 2.0);
        let s1 = vs * d1 + sigma * (alpha * d1 * d1 / 2.0 + j * d1 * d1 * d1 / 6.0);
        let v2 = v1 + sigma * ap * d2;
        let s2 = s1 + v1 * d2 + sigma * ap * d2 * d2 / 2.0;
        let dist = s2 + v2 * d3 + sigma * (ap * d3 * d3 / 2.0 - j * d3 * d3 * d3 / 6.0);

        Self {
            vs,
            ve,
            sigma,
            alpha,
            ap,
            j,
            t1: d1,
            t2: d1 + d2,
            t3: d1 + d2 + d3,
            s1,
            v1,
            s2,
            v2,
            dist,
        }
    }

    /// Distance, speed and acceleration `tau` seconds into the ramp.
    fn at(&self, tau: f32) -> (f32, f32, f32) {
        let (sg, j, ap) = (self.sigma, self.j, self.ap);
        if tau < self.t1 {
            let al = self.alpha;
            let s = self.vs * tau + sg * (al * tau * tau / 2.0 + j * tau * tau * tau / 6.0);
            (
                s,
                self.vs + sg * (al * tau + j * tau * tau / 2.0),
                sg * (al + j * tau),
            )
        } else if tau < self.t2 {
            let t = tau - self.t1;
            let s = self.s1 + self.v1 * t + sg * ap * t * t / 2.0;
            (s, self.v1 + sg * ap * t, sg * ap)
        } else {
            let t = (tau - self.t2).min(self.t3 - self.t2);
            let s = self.s2 + self.v2 * t + sg * (ap * t * t / 2.0 - j * t * t * t / 6.0);
            (
                s,
                self.v2 + sg * (ap * t - j * t * t / 2.0),
                sg * (ap - j * t),
            )
        }
    }

    /// Time at which the ramp has covered `d`, found by bisection: distance
    /// only grows with time, since speed stays non-negative.
    fn time_at(&self, d: f32) -> f32 {
        let (mut lo, mut hi) = (0.0, self.t3);
        for _ in 0..40 {
            let mid = (lo + hi) / 2.0;
            if self.at(mid).0 < d {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        hi
    }
}

/// Direction of a ramp from `vs` with acceleration `a_s` to `ve`: `1.0` to
/// speed up, `-1.0` to slow down. Staying at the same speed means first
/// moving the way the acceleration does, then back.
fn direction(vs: f32, a_s: f32, ve: f32) -> f32 {
    if ve > vs || (ve == vs && a_s < 0.0) {
        1.0
    } else {
        -1.0
    }
}

/// One or two ramps that take a state to a speed with no acceleration.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Ramps {
    first: Ramp,
    second: Option<Ramp>,
}

impl Ramps {
    fn dist(&self) -> f32 {
        self.first.dist + self.second.map_or(0.0, |r| r.dist)
    }
}

impl Limits {
    /// The starting acceleration the plan uses: within `±accel`, and a
    /// deceleration no stronger than can be brought to zero before the speed
    /// reaches zero.
    fn start_accel(&self, v0: f32, a0: f32) -> f32 {
        let a0 = a0.clamp(-self.a, self.a);
        a0.max(-libm::sqrtf(2.0 * self.j * v0))
    }

    /// The speed reached by bringing acceleration `a0` to zero at once.
    fn settle(&self, v0: f32, a0: f32) -> f32 {
        (v0 + a0 * libm::fabsf(a0) / (2.0 * self.j)).max(0.0)
    }

    /// Ramps from `(v0, a0)` to `(v, 0)`. When the start acceleration would
    /// carry the speed past `v` even if brought to zero at once, that comes
    /// first, followed by a ramp back.
    fn ramp_to(&self, v0: f32, a0: f32, v: f32) -> Ramps {
        let alpha = direction(v0, a0, v) * a0;
        let room = 2.0 * self.j * libm::fabsf(v - v0);
        if alpha <= 0.0 || room >= alpha * alpha * (1.0 - 1e-5) {
            return Ramps {
                first: Ramp::new(v0, a0, v, self),
                second: None,
            };
        }
        let vs = self.settle(v0, a0);
        Ramps {
            first: Ramp::new(v0, a0, vs, self),
            second: Some(Ramp::new(vs, 0.0, v, self)),
        }
    }
}

/// One piece of a plan.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Segment {
    Ramp(Ramp),
    Cruise(f32),
}

/// A segment with its start time and position and its duration.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Piece {
    seg: Segment,
    t0: f32,
    s0: f32,
    dur: f32,
}

/// Collects the pieces of a plan.
struct Builder {
    pieces: [Piece; 4],
    count: usize,
    t: f32,
    s: f32,
}

impl Builder {
    fn new(s0: f32) -> Self {
        let none = Piece {
            seg: Segment::Cruise(0.0),
            t0: 0.0,
            s0,
            dur: 0.0,
        };
        Self {
            pieces: [none; 4],
            count: 0,
            t: 0.0,
            s: s0,
        }
    }

    fn push(&mut self, seg: Segment, dur: f32, dist: f32) {
        if dur > 0.0 {
            self.pieces[self.count] = Piece {
                seg,
                t0: self.t,
                s0: self.s,
                dur,
            };
            self.count += 1;
            self.t += dur;
            self.s += dist;
        }
    }

    fn ramp(&mut self, r: Ramp) {
        self.push(Segment::Ramp(r), r.t3, r.dist);
    }

    fn ramps(&mut self, r: Ramps) {
        self.ramp(r.first);
        if let Some(second) = r.second {
            self.ramp(second);
        }
    }

    fn cruise(&mut self, v: f32, dist: f32) {
        // Shorter than a micrometre is rounding left over from the ramps.
        if v > 0.0 && dist > 1e-3 {
            self.push(Segment::Cruise(v), dist / v, dist);
        }
    }

    fn finish(self, end: State, peak: f32) -> SCurvePlan {
        SCurvePlan {
            t: 0.0,
            pieces: self.pieces,
            count: self.count,
            total: self.t,
            end,
            peak,
        }
    }
}

impl SCurve {
    fn limits(&self) -> Limits {
        let lim = Limits {
            vmax: self.max_speed.as_mm_per_s(),
            a: self.accel.as_mm_per_s2(),
            j: self.jerk.as_mm_per_s3(),
        };
        debug_assert!(lim.vmax > 0.0, "SCurve::max_speed must be positive");
        debug_assert!(lim.a > 0.0, "SCurve::accel must be positive");
        debug_assert!(lim.j > 0.0, "SCurve::jerk must be positive");
        lim
    }
}

/// Finds the value in `[lo, hi]` where `f` crosses `target`, assuming `f`
/// is continuous and `f(lo) <= target < f(hi)` or the reverse.
fn bisect(mut lo: f32, mut hi: f32, target: f32, f: impl Fn(f32) -> f32) -> f32 {
    let below_at_lo = f(lo) <= target;
    for _ in 0..40 {
        let mid = (lo + hi) / 2.0;
        if (f(mid) <= target) == below_at_lo {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

impl Profile for SCurve {
    type Plan = SCurvePlan;

    fn plan(&self, from: State, distance: Length, end_speed: Speed) -> SCurvePlan {
        let lim = self.limits();
        let d = distance.as_mm().max(0.0);
        let v0 = from.v.as_mm_per_s().max(0.0);
        let a0 = lim.start_accel(v0, from.a.as_mm_per_s2());
        let v1 = end_speed.as_mm_per_s().clamp(0.0, lim.vmax);
        let s0 = from.s.as_mm();
        let end_s = Length::mm(s0 + d);
        let vs = lim.settle(v0, a0);
        let peak = v0.max(if a0 > 0.0 { vs } else { 0.0 });
        let mut b = Builder::new(s0);

        let settle = Ramp::new(v0, a0, vs, &lim);
        // Comparisons allow for rounding, so that a move planned at exactly a
        // reachable speed is not mistaken for an unreachable one.
        let fits = |x: f32| x <= d * (1.0 + 1e-5) + 1e-6;
        if a0 != 0.0 && !fits(settle.dist) {
            // Too short even to bring the acceleration to zero: stop the
            // plan at its distance, still accelerating.
            let t = settle.time_at(d);
            let (_, v, a) = settle.at(t);
            b.push(Segment::Ramp(settle), t, d);
            let end = State {
                s: end_s,
                v: Speed::mm_per_s(v),
                a: Acceleration::mm_per_s2(a),
            };
            return b.finish(end, peak.max(v));
        }

        let needed = |vp: f32| lim.ramp_to(v0, a0, vp).dist() + Ramp::new(vp, 0.0, v1, &lim).dist;
        let at_rest = |v: f32| State {
            s: end_s,
            v: Speed::mm_per_s(v),
            a: Acceleration::ZERO,
        };

        if !fits(needed(v1)) {
            // The end speed is out of reach: end at the closest speed that
            // fits, between where settling leaves the speed and `v1`.
            // Bisection keeps one end that fits, so the speed it returns
            // always covers no more than the distance.
            let ve = bisect(vs, v1, d, |v| lim.ramp_to(v0, a0, v).dist());
            b.ramps(lim.ramp_to(v0, a0, ve));
            return b.finish(at_rest(ve), peak.max(ve));
        }

        let vp = if needed(lim.vmax) <= d {
            lim.vmax
        } else {
            bisect(v1, lim.vmax, d, needed)
        };
        b.ramps(lim.ramp_to(v0, a0, vp));
        b.cruise(vp, d - needed(vp));
        b.ramp(Ramp::new(vp, 0.0, v1, &lim));
        b.finish(at_rest(v1), peak.max(vp))
    }

    fn stop(&self, from: State) -> SCurvePlan {
        let lim = self.limits();
        let v0 = from.v.as_mm_per_s().max(0.0);
        let a0 = lim.start_accel(v0, from.a.as_mm_per_s2());
        let s0 = from.s.as_mm();
        let ramps = lim.ramp_to(v0, a0, 0.0);
        let mut b = Builder::new(s0);
        b.ramps(ramps);
        let peak = v0.max(if a0 > 0.0 { lim.settle(v0, a0) } else { 0.0 });
        b.finish(State::at(Length::mm(s0 + ramps.dist())), peak)
    }

    fn max_accel(&self) -> Acceleration {
        self.accel
    }

    fn max_speed(&self) -> Speed {
        self.max_speed
    }

    fn scaled(&self, factor: f32) -> Self {
        Self {
            max_speed: self.max_speed * factor,
            accel: self.accel * factor,
            jerk: self.jerk * factor,
        }
    }

    fn reachable(&self, from: Speed, distance: Length) -> Speed {
        let lim = self.limits();
        let v0 = from.as_mm_per_s().max(0.0);
        let d = distance.as_mm().max(0.0);
        if v0 >= lim.vmax || Ramp::new(v0, 0.0, lim.vmax, &lim).dist <= d {
            return self.max_speed;
        }
        // The ramp's distance only grows with its end speed.
        let v = bisect(v0, lim.vmax, d, |v| Ramp::new(v0, 0.0, v, &lim).dist);
        Speed::mm_per_s(v)
    }
}

/// A move planned by [`SCurve`].
///
/// ```
/// use capek_core::motion::profile::{Plan, Profile, SCurve, State};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let profile = SCurve {
///     max_speed: 400.mm_per_s(),
///     accel: 1200.mm_per_s2(),
///     jerk: 8000.mm_per_s3(),
/// };
/// let mut plan = profile.plan(State::REST, 50.mm(), 0.mm_per_s());
/// assert!(plan.peak_speed() < 400.mm_per_s()); // too short to reach cruise
///
/// let end = plan.advance(Duration::from_secs(1));
/// assert!(plan.is_done());
/// assert_eq!(end.s, 50.mm());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SCurvePlan {
    /// Time elapsed, in seconds.
    t: f32,
    pieces: [Piece; 4],
    count: usize,
    /// Total duration, s.
    total: f32,
    /// Final state, with the position kept exact.
    end: State,
    /// Highest speed, mm/s.
    peak: f32,
}

impl SCurvePlan {
    /// Total time the plan takes.
    ///
    /// ```
    /// use capek_core::motion::profile::{Profile, SCurve, State};
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = SCurve {
    ///     max_speed: 400.mm_per_s(),
    ///     accel: 1200.mm_per_s2(),
    ///     jerk: 8000.mm_per_s3(),
    /// };
    /// let plan = profile.plan(State::REST, 1.m(), 0.mm_per_s());
    /// // Each ramp takes 400/1200 + 1200/8000 s and covers 96.7 mm.
    /// assert!((plan.duration().as_secs_f32() - 2.9833).abs() < 1e-3);
    /// ```
    pub fn duration(&self) -> Duration {
        Duration::from_secs_f32(self.total)
    }

    /// Highest speed reached during the plan.
    ///
    /// ```
    /// use capek_core::motion::profile::{Profile, SCurve, State};
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = SCurve {
    ///     max_speed: 400.mm_per_s(),
    ///     accel: 1200.mm_per_s2(),
    ///     jerk: 8000.mm_per_s3(),
    /// };
    /// let plan = profile.plan(State::REST, 1.m(), 0.mm_per_s());
    /// assert_eq!(plan.peak_speed(), 400.mm_per_s());
    /// ```
    pub fn peak_speed(&self) -> Speed {
        Speed::mm_per_s(self.peak)
    }

    /// The state at the current time.
    fn state(&self) -> State {
        if self.t >= self.total {
            return self.end;
        }
        let piece = self.pieces[..self.count]
            .iter()
            .rev()
            .find(|p| p.t0 <= self.t)
            .unwrap_or(&self.pieces[0]);
        let tau = self.t - piece.t0;
        let (s, v, a) = match piece.seg {
            Segment::Ramp(r) => r.at(tau),
            Segment::Cruise(v) => (v * tau, v, 0.0),
        };
        State {
            // Rounding allowed in planning can put the ramps a hair past the
            // end; never report a position beyond it.
            s: Length::mm((piece.s0 + s).min(self.end.s.as_mm())),
            v: Speed::mm_per_s(v),
            a: Acceleration::mm_per_s2(a),
        }
    }
}

impl Plan for SCurvePlan {
    fn advance(&mut self, dt: Duration) -> State {
        self.t += dt.as_secs_f32();
        self.state()
    }

    fn is_done(&self) -> bool {
        self.t >= self.total
    }

    fn overshoot(&self) -> Duration {
        Duration::from_secs_f32((self.t - self.total).max(0.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::UnitExt;

    const DT: Duration = Duration::from_millis(1);

    fn profile() -> SCurve {
        SCurve {
            max_speed: 400.mm_per_s(),
            accel: 1200.mm_per_s2(),
            jerk: 8000.mm_per_s3(),
        }
    }

    /// Runs a plan to the end in 1 ms slices, checking the limits and that
    /// speed and acceleration stay continuous, starting from `from`.
    fn run(mut plan: SCurvePlan, from: State) -> State {
        let vmax = 400.0_f32.max(from.v.as_mm_per_s()) + 1e-2;
        let dt = DT.as_secs_f32();
        let mut prev = from;
        let mut steps = 0;
        while !plan.is_done() {
            let s = plan.advance(DT);
            steps += 1;
            assert!(s.v.as_mm_per_s() <= vmax, "speed limit at {steps}: {s:?}");
            assert!(
                s.v.as_mm_per_s() >= -1e-3,
                "negative speed at {steps}: {s:?}"
            );
            assert!(s.a.abs() <= 1200.mm_per_s2() + 1e-2.mm_per_s2(), "{s:?}");
            let dv = (s.v - prev.v).abs().as_mm_per_s();
            assert!(dv <= 1200.0 * dt + 1e-2, "speed jump {dv} at {steps}");
            if !plan.is_done() {
                let da = (s.a - prev.a).abs().as_mm_per_s2();
                assert!(
                    da <= 8000.0 * dt + 0.05,
                    "jerk limit {da} at {steps}: {s:?}"
                );
            }
            assert!(s.s >= prev.s - 1e-4.mm(), "went backwards at {steps}");
            prev = s;
            assert!(steps < 60_000, "plan never finished");
        }
        prev
    }

    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() <= tol
    }

    fn moving(v: f32, a: f32) -> State {
        State {
            s: 0.mm(),
            v: v.mm_per_s(),
            a: a.mm_per_s2(),
        }
    }

    #[test]
    fn ramp_matches_the_closed_form() {
        let lim = profile().limits();
        // Long enough to reach `accel`: 400/1200 + 1200/8000 s.
        let r = Ramp::new(0.0, 0.0, 400.0, &lim);
        assert!(close(r.t3, 0.48333, 1e-4));
        assert!(close(r.dist, 200.0 * r.t3, 1e-2));
        // Too short to reach it: 2·√(Δv / j).
        let r = Ramp::new(100.0, 0.0, 150.0, &lim);
        assert!(close(r.t3, 2.0 * libm::sqrtf(50.0 / 8000.0), 1e-5));
        assert!(close(r.ap, libm::sqrtf(50.0 * 8000.0), 1e-2));
        let (s, v, a) = r.at(r.t3);
        assert!(close(s, r.dist, 1e-3) && close(v, 150.0, 1e-3) && close(a, 0.0, 1e-3));
    }

    #[test]
    fn rest_to_rest_has_seven_phases() {
        let plan = profile().plan(State::REST, 1.m(), 0.mm_per_s());
        assert_eq!(plan.peak_speed(), 400.mm_per_s());
        assert_eq!(plan.count, 3);
        assert_eq!(run(plan, State::REST), State::at(1.m()));
    }

    #[test]
    fn short_moves_end_exactly() {
        for d in [100.mm(), 10.mm(), 0.5.mm()] {
            let plan = profile().plan(State::REST, d, 0.mm_per_s());
            assert!(plan.peak_speed() < 400.mm_per_s());
            assert_eq!(run(plan, State::REST), State::at(d));
        }
    }

    #[test]
    fn plans_between_moving_states() {
        let from = moving(200.0, 0.0);
        let end = run(profile().plan(from, 500.mm(), 100.mm_per_s()), from);
        assert_eq!(end.s, 500.mm());
        assert_eq!(end.v, 100.mm_per_s());
    }

    #[test]
    fn slows_down_from_above_cruise() {
        let from = moving(600.0, 0.0);
        for d in [1.m(), 200.mm()] {
            let end = run(profile().plan(from, d, 0.mm_per_s()), from);
            assert_eq!(end, State::at(d));
        }
    }

    #[test]
    fn unreachable_end_speed_never_overshoots() {
        let from = moving(400.0, 0.0);
        let end = run(profile().plan(from, 20.mm(), 0.mm_per_s()), from);
        assert_eq!(end.s, 20.mm());
        assert!(end.v > 300.mm_per_s());

        let end = run(
            profile().plan(State::REST, 10.mm(), 400.mm_per_s()),
            State::REST,
        );
        assert_eq!(end.s, 10.mm());
        // The single ramp that covers exactly 10 mm.
        let lim = profile().limits();
        assert!(close(
            Ramp::new(0.0, 0.0, end.v.as_mm_per_s(), &lim).dist,
            10.0,
            1e-2
        ));
    }

    #[test]
    fn stop_from_every_phase_is_smooth() {
        let p = profile();
        let mut plan = p.plan(State::REST, 1.m(), 0.mm_per_s());
        while !plan.is_done() {
            let state = plan.advance(DT);
            let end = run(p.stop(state), state);
            assert_eq!(end.v, 0.mm_per_s());
            assert_eq!(end.a, 0.mm_per_s2());
        }
    }

    #[test]
    fn replanning_mid_ramp_is_smooth() {
        let p = profile();
        let mut plan = p.plan(State::REST, 1.m(), 0.mm_per_s());
        while !plan.is_done() {
            let state = plan.advance(Duration::from_millis(7));
            // Somewhere else, at a different end speed.
            for (d, v1) in [(300.0, 0.0), (50.0, 200.0), (5.0, 0.0)] {
                let next = p.plan(state, d.mm(), v1.mm_per_s());
                let end = run(next, state);
                assert_eq!(end.s, state.s + d.mm());
            }
        }
    }

    #[test]
    fn too_short_to_settle_ends_still_accelerating() {
        let from = moving(100.0, 1200.0);
        let end = run(profile().plan(from, 0.5.mm(), 0.mm_per_s()), from);
        assert_eq!(end.s, 0.5.mm());
        assert!(end.a > 0.mm_per_s2());
    }

    #[test]
    fn clamps_unsettleable_deceleration() {
        // At 1 mm/s, 1200 mm/s² of deceleration cannot fade before stopping.
        let from = moving(1.0, -1200.0);
        let mut plan = profile().plan(from, 10.mm(), 0.mm_per_s());
        let first = plan.advance(DT);
        assert!(first.a > (-200).mm_per_s2());
        let end = run(plan, first);
        assert_eq!(end, State::at(10.mm()));
    }

    #[test]
    fn zero_distance_at_rest_is_done() {
        let p = profile();
        let mut plan = p.plan(State::at(5.mm()), 0.mm(), 0.mm_per_s());
        assert!(plan.is_done());
        assert_eq!(plan.advance(DT), State::at(5.mm()));
        assert!(p.stop(State::at(5.mm())).is_done());
    }

    #[test]
    fn negative_inputs_are_zero() {
        assert!(
            profile()
                .plan(State::REST, (-10).mm(), (-5).mm_per_s())
                .is_done()
        );
    }

    #[test]
    fn coarse_steps_match_fine_steps() {
        let mut fine = profile().plan(State::REST, 500.mm(), 0.mm_per_s());
        let mut coarse = fine;
        for i in 1..=600 {
            let f = fine.advance(DT);
            if i % 50 == 0 {
                let c = coarse.advance(Duration::from_millis(50));
                assert!(close(c.s.as_mm(), f.s.as_mm(), 1e-2));
                assert!(close(c.v.as_mm_per_s(), f.v.as_mm_per_s(), 1e-2));
            }
        }
    }

    #[test]
    fn stop_at_exactly_the_reachable_speed_ends_at_rest() {
        // Without allowing for rounding, a stop that needs 1e-5 mm too much
        // was taken as unreachable and ended far above rest.
        let lim = profile().limits();
        for i in 1..100 {
            let d = i as f32 * 0.73;
            let v0 = bisect(0.0, 400.0, d, |v| Ramp::new(0.0, 0.0, v, &lim).dist);
            let from = moving(v0, 0.0);
            let end = run(profile().plan(from, d.mm(), 0.mm_per_s()), from);
            assert!(end.v < 0.05.mm_per_s(), "from {v0} over {d}: {end:?}");
        }
    }
}

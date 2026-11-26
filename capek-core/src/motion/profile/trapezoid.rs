use super::{Plan, Profile, State};
use crate::units::{Acceleration, Length, Speed};
use core::time::Duration;

/// Constant acceleration up to a cruise speed, then constant deceleration.
///
/// Speed over time is a trapezoid, or a triangle when the move is too short to
/// reach `max_speed`. Every state is computed in closed form, so the plan is
/// exact whatever the time step.
///
/// Acceleration jumps between `accel`, zero and `-accel`. The acceleration in
/// the state a plan starts from is ignored.
///
/// ```
/// use capek_core::motion::profile::{Plan, Profile, State, Trapezoid};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
/// let plan = profile.plan(State::REST, 400.mm(), 0.mm_per_s());
///
/// // 1/3 s to reach 400 mm/s, 2/3 s of cruise, 1/3 s to stop.
/// assert!((plan.duration().as_secs_f32() - 4.0 / 3.0).abs() < 1e-4);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Trapezoid {
    /// Cruise speed. Must be positive.
    pub max_speed: Speed,
    /// Acceleration and deceleration. Must be positive.
    pub accel: Acceleration,
}

impl Profile for Trapezoid {
    type Plan = TrapezoidPlan;

    fn plan(&self, from: State, distance: Length, end_speed: Speed) -> TrapezoidPlan {
        let vmax = self.max_speed.as_mm_per_s();
        let a = self.accel.as_mm_per_s2();
        debug_assert!(vmax > 0.0, "Trapezoid::max_speed must be positive");
        debug_assert!(a > 0.0, "Trapezoid::accel must be positive");

        let d = distance.as_mm().max(0.0);
        let v0 = from.v.as_mm_per_s().max(0.0);
        let mut v1 = end_speed.as_mm_per_s().clamp(0.0, vmax);

        // Keep the end speed within what the distance allows.
        let reach_sq = 2.0 * a * d;
        if v1 > v0 {
            v1 = v1.min(libm::sqrtf(v0 * v0 + reach_sq));
        } else if v0 * v0 - v1 * v1 > reach_sq {
            v1 = libm::sqrtf(v0 * v0 - reach_sq);
        }

        // Peak speed: the triangle's apex, capped at the cruise speed. When the
        // start speed is above the cruise speed and the distance is too short
        // to slow down to it, the whole move is one deceleration.
        let apex = libm::sqrtf((reach_sq + v0 * v0 + v1 * v1) / 2.0);
        let vp = apex.min(vmax).max(v1);

        let d1 = libm::fabsf(vp * vp - v0 * v0) / (2.0 * a);
        let d3 = (vp * vp - v1 * v1) / (2.0 * a);
        let cruise = (d - d1 - d3).max(0.0);

        let t1 = libm::fabsf(vp - v0) / a;
        let t2 = t1 + if vp > 0.0 { cruise / vp } else { 0.0 };
        let t3 = t2 + (vp - v1) / a;

        let s0 = from.s.as_mm();
        TrapezoidPlan {
            t: 0.0,
            s0,
            v0,
            a1: if vp >= v0 { a } else { -a },
            t1,
            s1: s0 + d1,
            vp,
            t2,
            s3: s0 + d - d3,
            a,
            t3,
            v1,
            end: Length::mm(s0 + d),
        }
    }

    fn stop(&self, from: State) -> TrapezoidPlan {
        // One deceleration, built directly: going through `plan` would let
        // rounding in `v0² - 2ad` leave a small end speed.
        let v0 = from.v.as_mm_per_s().max(0.0);
        let a = self.accel.as_mm_per_s2();
        let s0 = from.s.as_mm();
        TrapezoidPlan {
            t: 0.0,
            s0,
            v0,
            a1: -a,
            t1: 0.0,
            s1: s0,
            vp: v0,
            t2: 0.0,
            s3: s0,
            a,
            t3: v0 / a,
            v1: 0.0,
            end: Length::mm(s0 + v0 * v0 / (2.0 * a)),
        }
    }
}

/// A move planned by [`Trapezoid`].
///
/// ```
/// use capek_core::motion::profile::{Plan, Profile, State, Trapezoid};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
/// let mut plan = profile.plan(State::REST, 50.mm(), 0.mm_per_s());
/// assert!(plan.peak_speed() < 400.mm_per_s()); // too short to reach cruise
///
/// let end = plan.advance(Duration::from_secs(1));
/// assert!(plan.is_done());
/// assert_eq!(end.s, 50.mm());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrapezoidPlan {
    /// Time elapsed, in seconds.
    t: f32,
    /// Start position and speed, mm and mm/s.
    s0: f32,
    v0: f32,
    /// Acceleration of the first phase, signed, mm/s².
    a1: f32,
    /// End of the first phase: time and position.
    t1: f32,
    s1: f32,
    /// Cruise speed, mm/s.
    vp: f32,
    /// End of the cruise phase.
    t2: f32,
    /// Position where the final deceleration starts.
    s3: f32,
    /// Deceleration magnitude, mm/s².
    a: f32,
    /// End of the plan.
    t3: f32,
    /// End speed, mm/s.
    v1: f32,
    /// End position, kept exact.
    end: Length,
}

impl TrapezoidPlan {
    /// Total time the plan takes.
    ///
    /// ```
    /// use capek_core::motion::profile::{Profile, State, Trapezoid};
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1000.mm_per_s2() };
    /// let plan = profile.plan(State::REST, 40.mm(), 0.mm_per_s());
    /// // A triangle: 0.2 s up to 200 mm/s, 0.2 s down.
    /// assert!((plan.duration().as_secs_f32() - 0.4).abs() < 1e-5);
    /// ```
    pub fn duration(&self) -> Duration {
        Duration::from_secs_f32(self.t3)
    }

    /// Highest speed reached during the plan.
    ///
    /// ```
    /// use capek_core::motion::profile::{Profile, State, Trapezoid};
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = Trapezoid { max_speed: 400.mm_per_s(), accel: 1000.mm_per_s2() };
    /// let plan = profile.plan(State::REST, 1.m(), 0.mm_per_s());
    /// assert_eq!(plan.peak_speed(), 400.mm_per_s());
    /// ```
    pub fn peak_speed(&self) -> Speed {
        Speed::mm_per_s(self.vp.max(self.v0))
    }

    /// The state at the current time.
    fn state(&self) -> State {
        let t = self.t;
        if t >= self.t3 {
            return State {
                s: self.end,
                v: Speed::mm_per_s(self.v1),
                a: Acceleration::ZERO,
            };
        }
        let (s, v, a) = if t < self.t1 {
            let v = self.v0 + self.a1 * t;
            (self.s0 + (self.v0 + v) / 2.0 * t, v, self.a1)
        } else if t < self.t2 {
            (self.s1 + self.vp * (t - self.t1), self.vp, 0.0)
        } else {
            let tau = t - self.t2;
            let v = self.vp - self.a * tau;
            (self.s3 + (self.vp + v) / 2.0 * tau, v, -self.a)
        };
        State {
            s: Length::mm(s),
            v: Speed::mm_per_s(v),
            a: Acceleration::mm_per_s2(a),
        }
    }
}

impl Plan for TrapezoidPlan {
    fn advance(&mut self, dt: Duration) -> State {
        self.t += dt.as_secs_f32();
        self.state()
    }

    fn is_done(&self) -> bool {
        self.t >= self.t3
    }
}

/// Resolution a Fast profile uses until a parent component sets one.
pub(super) const DEFAULT_RESOLUTION: Length = Length::mm(0.1);

/// [`Trapezoid`] computed per step with David Austin's recurrence, as ninja
/// does.
///
/// The move is split into distance increments of one resolution, and the time
/// each increment takes comes from the previous one through
/// `c[n] = c[n-1] - 2·c[n-1] / (4n + 1)`, which needs no square root. Parent
/// components set the resolution from their geometry, usually one wheel
/// microstep, through [`Profile::set_resolution`]. Until then it is 0.1 mm.
/// [`advance`](Plan::advance) interpolates position within an increment.
///
/// The plan stays within a few percent of [`Trapezoid`]. A move reaches the
/// whole number of increments closest to `max_speed`, so the cruise speed can
/// differ from it by a fraction of one increment's worth of speed.
///
/// Differences from ninja's `TrapezoidProfile`, which this reproduces for
/// rest-to-rest moves:
///
/// - it uses `f32` instead of `f64`;
/// - it plans between non-zero speeds, starting the recurrence from the
///   increment that matches the start speed;
/// - deceleration runs the recurrence backwards instead of replaying the
///   acceleration delays, which gives the same delays without storing them;
/// - a distance that is not a whole number of increments ends with a partial
///   increment at cruise speed.
///
/// ```
/// use capek_core::motion::profile::{FastTrapezoid, Plan, Profile, State};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let mut profile = FastTrapezoid::new(400.mm_per_s(), 1200.mm_per_s2());
/// // One microstep of a 56 mm wheel with 400 steps per revolution.
/// profile.set_resolution((56.0 * core::f32::consts::PI / 400.0).mm());
///
/// let mut plan = profile.plan(State::REST, 400.mm(), 0.mm_per_s());
/// let mut end = State::REST;
/// while !plan.is_done() {
///     end = plan.advance(Duration::from_millis(1));
/// }
/// assert_eq!(end, State::at(400.mm()));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FastTrapezoid {
    /// Cruise speed. Must be positive.
    pub max_speed: Speed,
    /// Acceleration and deceleration. Must be positive.
    pub accel: Acceleration,
    /// Distance increment the recurrence steps through.
    resolution: Length,
}

impl FastTrapezoid {
    /// A profile with the given limits and the default resolution of 0.1 mm.
    ///
    /// ```
    /// use capek_core::motion::profile::FastTrapezoid;
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = FastTrapezoid::new(400.mm_per_s(), 1200.mm_per_s2());
    /// assert_eq!(profile.resolution(), 0.1.mm());
    /// ```
    pub const fn new(max_speed: Speed, accel: Acceleration) -> Self {
        Self {
            max_speed,
            accel,
            resolution: DEFAULT_RESOLUTION,
        }
    }

    /// The distance increment the recurrence steps through.
    ///
    /// ```
    /// use capek_core::motion::profile::{FastTrapezoid, Profile};
    /// use capek_core::units::UnitExt;
    ///
    /// let mut profile = FastTrapezoid::new(400.mm_per_s(), 1200.mm_per_s2());
    /// profile.set_resolution(0.44.mm());
    /// assert_eq!(profile.resolution(), 0.44.mm());
    /// ```
    pub const fn resolution(&self) -> Length {
        self.resolution
    }

    /// Resolution in mm and acceleration in mm/s², checked.
    fn params(&self) -> (f32, f32) {
        let res = self.resolution.as_mm();
        let a = self.accel.as_mm_per_s2();
        debug_assert!(res > 0.0, "FastTrapezoid resolution must be positive");
        debug_assert!(a > 0.0, "FastTrapezoid::accel must be positive");
        (res, a)
    }
}

/// Index of the increment the recurrence is at when moving at `v`: the
/// number of increments needed to reach `v` from rest.
pub(super) fn index(v: f32, a: f32, res: f32) -> u32 {
    libm::roundf(v * v / (2.0 * a * res)) as u32
}

/// Time of increment `n` when starting the recurrence there.
///
/// From rest this is Austin's corrected first delay, `0.676·c0`. Further up
/// the recurrence matches the exact `c0·(√(n+1) − √n)`, which is used instead.
pub(super) fn start_delay(n: u32, a: f32, res: f32) -> f32 {
    let c0 = libm::sqrtf(2.0 * res / a);
    if n == 0 {
        0.676 * c0
    } else {
        let n = n as f32;
        c0 * (libm::sqrtf(n + 1.0) - libm::sqrtf(n))
    }
}

impl Profile for FastTrapezoid {
    type Plan = FastTrapezoidPlan;

    fn plan(&self, from: State, distance: Length, end_speed: Speed) -> FastTrapezoidPlan {
        let (res, a) = self.params();
        let vmax = self.max_speed.as_mm_per_s();
        debug_assert!(vmax > 0.0, "FastTrapezoid::max_speed must be positive");

        let d = distance.as_mm().max(0.0);
        // Increments in the move. Snap values a rounding error away from a
        // whole number, so a move of whole steps has no sliver increment.
        let mut steps = d / res;
        if libm::fabsf(steps - libm::roundf(steps)) < 1e-3 {
            steps = libm::roundf(steps);
        }
        let whole = steps as u32;

        let nmax = (vmax * vmax / (2.0 * a * res)) as u32;
        let n0 = index(from.v.as_mm_per_s().max(0.0), a, res);
        let n1 = index(end_speed.as_mm_per_s().clamp(0.0, vmax), a, res)
            .min(nmax)
            // Keep the end speed within what the distance allows.
            .min(n0.saturating_add(whole))
            .max(n0.saturating_sub(whole));

        // In increment indices speed squared is linear, so the trapezoid
        // becomes piecewise linear. The apex is where the two ramps meet.
        let apex = ((steps + n0 as f32 + n1 as f32) / 2.0) as u32;
        let peak = apex.min(nmax).max(n1);
        let ramps = peak.abs_diff(n0) + (peak - n1);
        let cruise = (steps - ramps as f32).max(0.0);

        let s0 = from.s.as_mm();
        FastTrapezoidPlan::new(s0, res, a, n0, peak, cruise, n1, Length::mm(s0 + d))
    }

    fn stop(&self, from: State) -> FastTrapezoidPlan {
        let (res, a) = self.params();
        let n0 = index(from.v.as_mm_per_s().max(0.0), a, res);
        let s0 = from.s.as_mm();
        FastTrapezoidPlan::new(s0, res, a, n0, n0, 0.0, 0, Length::mm(s0 + n0 as f32 * res))
    }

    fn set_resolution(&mut self, resolution: Length) {
        self.resolution = resolution;
    }
}

/// Phase of a [`FastTrapezoidPlan`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// From the start speed to the peak, up or down.
    Ramp,
    /// At the peak speed.
    Cruise,
    /// From the peak down to the end speed.
    Brake,
    /// Finished.
    Done,
}

/// A move planned by [`FastTrapezoid`].
///
/// ```
/// use capek_core::motion::profile::{FastTrapezoid, Plan, Profile, State};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let profile = FastTrapezoid::new(400.mm_per_s(), 1200.mm_per_s2());
/// let mut plan = profile.plan(State::REST, 50.mm(), 0.mm_per_s());
///
/// let end = plan.advance(Duration::from_secs(1));
/// assert!(plan.is_done());
/// assert_eq!(end.s, 50.mm());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FastTrapezoidPlan {
    /// Start position and increment length, mm.
    s0: f32,
    res: f32,
    /// Acceleration magnitude, mm/s².
    a: f32,
    phase: Phase,
    /// Whether the first ramp speeds up.
    ramp_up: bool,
    /// Recurrence index and the time of increment `n`, in seconds.
    n: u32,
    c: f32,
    /// Index at the peak, increments of cruise left, index at the end.
    peak: u32,
    cruise: f32,
    last: u32,
    /// Increments completed.
    k: f32,
    /// Length of the current increment, as a fraction of one, its time and
    /// the time spent in it so far.
    len: f32,
    dur: f32,
    into: f32,
    /// End position, kept exact.
    end: Length,
}

impl FastTrapezoidPlan {
    #[allow(clippy::too_many_arguments)]
    fn new(
        s0: f32,
        res: f32,
        a: f32,
        n0: u32,
        peak: u32,
        cruise: f32,
        last: u32,
        end: Length,
    ) -> Self {
        let mut plan = Self {
            s0,
            res,
            a,
            phase: Phase::Ramp,
            ramp_up: peak >= n0,
            n: n0,
            c: start_delay(n0, a, res),
            peak,
            cruise,
            last,
            k: 0.0,
            len: 0.0,
            dur: 0.0,
            into: 0.0,
            end,
        };
        plan.begin();
        plan
    }

    /// Sets up the next increment, moving through phases with nothing left.
    fn begin(&mut self) {
        loop {
            match self.phase {
                Phase::Ramp if self.ramp_up && self.n < self.peak => {
                    self.len = 1.0;
                    self.dur = self.c;
                    return;
                }
                Phase::Ramp if !self.ramp_up && self.n > self.peak => {
                    self.step_down();
                    return;
                }
                Phase::Ramp => self.phase = Phase::Cruise,
                // Below a thousandth of an increment, the rest is rounding.
                Phase::Cruise if self.cruise > 1e-3 => {
                    self.len = self.cruise.min(1.0);
                    self.dur = self.len * self.c;
                    return;
                }
                Phase::Cruise => self.phase = Phase::Brake,
                Phase::Brake if self.n > self.last => {
                    self.step_down();
                    return;
                }
                Phase::Brake => self.phase = Phase::Done,
                Phase::Done => return,
            }
        }
    }

    /// Starts a decelerating increment: the recurrence run backwards.
    fn step_down(&mut self) {
        let n = self.n as f32;
        self.c *= (4.0 * n + 1.0) / (4.0 * n - 1.0);
        self.n -= 1;
        self.len = 1.0;
        self.dur = self.c;
    }

    /// Completes the current increment.
    fn finish(&mut self) {
        self.k += self.len;
        match self.phase {
            Phase::Ramp if self.ramp_up => {
                self.n += 1;
                self.c -= 2.0 * self.c / (4.0 * self.n as f32 + 1.0);
            }
            Phase::Cruise => self.cruise -= self.len,
            _ => {}
        }
    }

    /// Highest speed reached during the plan.
    ///
    /// ```
    /// use capek_core::motion::profile::{FastTrapezoid, Profile, State};
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = FastTrapezoid::new(400.mm_per_s(), 1200.mm_per_s2());
    /// let plan = profile.plan(State::REST, 1.m(), 0.mm_per_s());
    /// assert!((plan.peak_speed() - 400.mm_per_s()).abs() < 1.mm_per_s());
    /// ```
    pub fn peak_speed(&self) -> Speed {
        let n = self.peak.max(self.n) as f32;
        Speed::mm_per_s(libm::sqrtf(2.0 * self.a * self.res * n))
    }

    /// The state at the current time.
    fn state(&self) -> State {
        let a = match self.phase {
            Phase::Ramp if self.ramp_up => self.a,
            Phase::Ramp | Phase::Brake => -self.a,
            Phase::Cruise => 0.0,
            Phase::Done => {
                let v = libm::sqrtf(2.0 * self.a * self.res * self.last as f32);
                return State {
                    s: self.end,
                    v: Speed::mm_per_s(v),
                    a: Acceleration::ZERO,
                };
            }
        };
        let k = self.k + self.len * self.into / self.dur;
        State {
            s: Length::mm(self.s0 + k * self.res),
            v: Speed::mm_per_s(self.res / self.c),
            a: Acceleration::mm_per_s2(a),
        }
    }
}

impl Plan for FastTrapezoidPlan {
    fn advance(&mut self, dt: Duration) -> State {
        let mut left = dt.as_secs_f32();
        while self.phase != Phase::Done {
            let rest = self.dur - self.into;
            if left < rest {
                self.into += left;
                break;
            }
            left -= rest;
            self.finish();
            self.into = 0.0;
            self.begin();
        }
        self.state()
    }

    fn is_done(&self) -> bool {
        self.phase == Phase::Done
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::UnitExt;

    const DT: Duration = Duration::from_millis(1);

    fn profile() -> Trapezoid {
        Trapezoid {
            max_speed: 400.mm_per_s(),
            accel: 1200.mm_per_s2(),
        }
    }

    /// Runs a plan to the end in 1 ms slices, checking the limits on the way.
    fn run(profile: &Trapezoid, mut plan: TrapezoidPlan, from: State) -> (State, f32) {
        let vmax = profile.max_speed.as_mm_per_s().max(from.v.as_mm_per_s());
        let dv_max = profile.accel.as_mm_per_s2() * DT.as_secs_f32();
        let mut prev = from;
        let mut t = 0.0;
        while !plan.is_done() {
            let s = plan.advance(DT);
            t += DT.as_secs_f32();
            assert!(
                s.v.as_mm_per_s() <= vmax + 1e-3,
                "speed limit at {t}: {s:?}"
            );
            assert!(s.v.as_mm_per_s() >= -1e-3, "negative speed at {t}: {s:?}");
            let dv = (s.v - prev.v).abs().as_mm_per_s();
            assert!(dv <= dv_max + 1e-2, "speed jump {dv} at {t}");
            assert!(s.s >= prev.s - 1e-4.mm(), "went backwards at {t}");
            prev = s;
            assert!(t < 60.0, "plan never finished");
        }
        (prev, t)
    }

    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn rest_to_rest_reaches_cruise() {
        let p = profile();
        let plan = p.plan(State::REST, 1.m(), 0.mm_per_s());
        assert_eq!(plan.peak_speed(), 400.mm_per_s());
        // 1000 mm at 400 mm/s plus one extra accel time (400/1200 s).
        assert!(close(plan.duration().as_secs_f32(), 2.5 + 1.0 / 3.0, 1e-4));
        let (end, _) = run(&p, plan, State::REST);
        assert_eq!(end.s, 1.m());
        assert_eq!(end.v, 0.mm_per_s());
    }

    #[test]
    fn short_move_is_a_triangle() {
        let p = profile();
        let plan = p.plan(State::REST, 30.mm(), 0.mm_per_s());
        // v_peak = sqrt(a * d) for a symmetric triangle.
        assert!(close(plan.peak_speed().as_mm_per_s(), 189.737, 1e-2));
        let (end, _) = run(&p, plan, State::REST);
        assert_eq!(end.s, 30.mm());
    }

    #[test]
    fn plans_between_moving_states() {
        let p = profile();
        let from = State {
            s: 100.mm(),
            v: 200.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        let plan = p.plan(from, 500.mm(), 100.mm_per_s());
        let (end, _) = run(&p, plan, from);
        assert_eq!(end.s, 600.mm());
        assert_eq!(end.v, 100.mm_per_s());
    }

    #[test]
    fn slows_down_from_above_cruise() {
        let p = profile();
        let from = State {
            s: 0.mm(),
            v: 600.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        let mut plan = p.plan(from, 1.m(), 0.mm_per_s());
        let first = plan.advance(DT);
        assert!(first.a < 0.mm_per_s2());
        let (end, _) = run(&p, plan, first);
        assert_eq!(end.s, 1.m());
    }

    #[test]
    fn unreachable_end_speed_never_overshoots() {
        let p = profile();
        let from = State {
            s: 0.mm(),
            v: 400.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        // Stopping from 400 mm/s needs 66.7 mm; give it 20 mm.
        let plan = p.plan(from, 20.mm(), 0.mm_per_s());
        let (end, _) = run(&p, plan, from);
        assert_eq!(end.s, 20.mm());
        // v1² = 400² - 2 * 1200 * 20
        assert!(close(end.v.as_mm_per_s(), 334.664, 1e-2));

        // Asking for more speed than the distance allows.
        let plan = p.plan(State::REST, 10.mm(), 400.mm_per_s());
        let (end, _) = run(&p, plan, State::REST);
        assert!(close(end.v.as_mm_per_s(), 154.919, 1e-2));
    }

    #[test]
    fn stop_from_every_phase() {
        let p = profile();
        let mut plan = p.plan(State::REST, 1.m(), 0.mm_per_s());
        for _ in 0..3000 {
            let state = plan.advance(DT);
            let stop = p.stop(state);
            let expected = state.v.as_mm_per_s().powi(2) / 2400.0;
            let (end, _) = run(&p, stop, state);
            assert_eq!(end.v, 0.mm_per_s());
            assert!(close((end.s - state.s).as_mm(), expected, 1e-3));
        }
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
        let p = profile();
        let plan = p.plan(State::REST, (-10).mm(), (-5).mm_per_s());
        assert!(plan.is_done());
    }

    #[test]
    fn coarse_steps_match_fine_steps() {
        let p = profile();
        let mut fine = p.plan(State::REST, 500.mm(), 0.mm_per_s());
        let mut coarse = fine;
        let mut f = State::REST;
        for i in 1..=200 {
            f = fine.advance(DT);
            if i % 50 == 0 {
                let c = coarse.advance(Duration::from_millis(50));
                assert!(close(c.s.as_mm(), f.s.as_mm(), 1e-3));
                assert!(close(c.v.as_mm_per_s(), f.v.as_mm_per_s(), 1e-2));
            }
        }
        let _ = f;
    }
}

#[cfg(test)]
mod fast_tests {
    use super::*;
    use crate::units::UnitExt;

    const DT: Duration = Duration::from_millis(1);

    /// One microstep of ninja's drivetrain: 56 mm wheels, 400 steps per turn.
    fn ninja_resolution() -> f32 {
        56.0 * core::f32::consts::PI / 400.0
    }

    fn profile() -> FastTrapezoid {
        let mut p = FastTrapezoid::new(400.mm_per_s(), 1200.mm_per_s2());
        p.set_resolution(ninja_resolution().mm());
        p
    }

    fn exact() -> Trapezoid {
        Trapezoid {
            max_speed: 400.mm_per_s(),
            accel: 1200.mm_per_s2(),
        }
    }

    /// Runs a plan to the end in 1 ms slices, checking the limits on the way.
    fn run(mut plan: FastTrapezoidPlan, from: State) -> State {
        let vmax = 1.01 * 400.0_f32.max(from.v.as_mm_per_s());
        let mut prev = from;
        let mut steps = 0;
        while !plan.is_done() {
            let s = plan.advance(DT);
            steps += 1;
            assert!(s.v.as_mm_per_s() <= vmax, "speed limit at {steps}: {s:?}");
            assert!(s.v.as_mm_per_s() >= 0.0, "negative speed at {steps}");
            assert!(s.s >= prev.s - 1e-4.mm(), "went backwards at {steps}");
            prev = s;
            assert!(steps < 60_000, "plan never finished");
        }
        prev
    }

    /// The time of every increment of a plan, in seconds.
    fn delays(mut plan: FastTrapezoidPlan) -> Vec<f32> {
        let mut out = Vec::new();
        while plan.phase != Phase::Done {
            out.push(plan.dur);
            plan.finish();
            plan.begin();
        }
        out
    }

    /// Ninja's `TrapezoidProfile::delays`, in timer ticks at 4 MHz, in the
    /// linear units capek uses. Kept in `f64` as ninja has it.
    fn ninja_delays(steps: u32) -> Vec<u32> {
        let res = ninja_resolution() as f64;
        let (vmax, accel) = (400.0_f64, 1200.0_f64);
        let f = 4_000_000.0_f64;
        let c0 = f * libm::sqrt(2.0 * res / accel);
        let max_n = (vmax * vmax) / (2.0 * accel * res);
        let accel_steps = core::cmp::min(libm::floor(max_n) as u32, steps / 2);
        let coast_steps = steps.saturating_sub(2 * accel_steps);

        let mut vec = Vec::new();
        let mut cn = 0.676 * c0;
        let mut n = 0.0;
        for _ in 0..accel_steps {
            vec.push(cn as u32);
            n += 1.0;
            cn = cn - (2.0 * cn) / (4.0 * n + 1.0);
        }
        for _ in 0..coast_steps {
            vec.push(cn as u32);
        }
        for i in (0..accel_steps).rev() {
            let d = vec[i as usize];
            vec.push(d);
        }
        vec
    }

    #[test]
    fn reproduces_ninja_delays() {
        let res = ninja_resolution();
        // Long, triangular, odd and tiny moves.
        for steps in [2000, 300, 151, 7, 1] {
            let plan = profile().plan(State::REST, (steps as f32 * res).mm(), 0.mm_per_s());
            let ours = delays(plan);
            let ninja = ninja_delays(steps);
            assert_eq!(ours.len(), ninja.len(), "{steps} steps");
            for (i, (c, n)) in ours.iter().zip(ninja.iter()).enumerate() {
                let c = c * 4_000_000.0;
                let n = *n as f32;
                assert!(
                    (c - n).abs() <= 1.0 + 1e-4 * n,
                    "{steps} steps, delay {i}: {c} vs ninja {n}"
                );
            }
        }
    }

    #[test]
    fn follows_the_exact_trapezoid() {
        for distance in [1.m(), 100.mm(), 20.mm()] {
            let mut fast = profile().plan(State::REST, distance, 0.mm_per_s());
            let mut slow = exact().plan(State::REST, distance, 0.mm_per_s());
            let total = slow.duration().as_secs_f32();
            let end = run(fast, State::REST);
            assert_eq!(end, State::at(distance));

            let mut t = 0.0;
            while !slow.is_done() || !fast.is_done() {
                let f = fast.advance(DT);
                let s = slow.advance(DT);
                t += DT.as_secs_f32();
                // Within 2 % of the distance, and of the peak speed. Austin's
                // shortened first increment also puts the fast plan about
                // 9 ms ahead, worth up to 4 mm at 400 mm/s.
                assert!(
                    (f.s - s.s).abs() < distance * 0.02 + 4.mm(),
                    "{f:?} vs {s:?}"
                );
                // Speed per increment starts at about √(a·res), 24 mm/s here,
                // where the exact profile starts from zero.
                assert!(
                    (f.v - s.v).abs() < slow.peak_speed() * 0.05 + 30.mm_per_s(),
                    "{t}: {f:?} vs {s:?}"
                );
            }
            assert!(
                (t - total).abs() < total * 0.02,
                "took {t} s, not {total} s"
            );
        }
    }

    #[test]
    fn plans_between_moving_states() {
        let from = State {
            s: 100.mm(),
            v: 200.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        let mut plan = profile().plan(from, 500.mm(), 100.mm_per_s());
        let first = plan.advance(DT);
        assert!((first.v - 200.mm_per_s()).abs() < 5.mm_per_s());
        let end = run(plan, first);
        assert_eq!(end.s, 600.mm());
        // Speeds are whole increment indices: about 5 mm/s apart at 100 mm/s.
        assert!((end.v - 100.mm_per_s()).abs() < 5.mm_per_s());
    }

    #[test]
    fn slows_down_from_above_cruise() {
        let from = State {
            s: 0.mm(),
            v: 600.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        let mut plan = profile().plan(from, 1.m(), 0.mm_per_s());
        let first = plan.advance(DT);
        assert!(first.a < 0.mm_per_s2());
        assert!(first.v < 600.mm_per_s());
        let end = run(plan, first);
        assert_eq!(end, State::at(1.m()));
    }

    #[test]
    fn unreachable_end_speed_never_overshoots() {
        let from = State {
            s: 0.mm(),
            v: 400.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        let end = run(profile().plan(from, 20.mm(), 0.mm_per_s()), from);
        assert_eq!(end.s, 20.mm());
        // v1² = 400² - 2 * 1200 * 20, to within an increment.
        assert!((end.v.as_mm_per_s() - 334.664).abs() < 3.0);

        let end = run(
            profile().plan(State::REST, 10.mm(), 400.mm_per_s()),
            State::REST,
        );
        assert_eq!(end.s, 10.mm());
        assert!((end.v.as_mm_per_s() - 154.919).abs() < 3.0);
    }

    #[test]
    fn stop_from_every_phase() {
        let p = profile();
        let mut plan = p.plan(State::REST, 1.m(), 0.mm_per_s());
        while !plan.is_done() {
            let state = plan.advance(DT);
            let expected = state.v.as_mm_per_s().powi(2) / 2400.0;
            let end = run(p.stop(state), state);
            assert_eq!(end.v, 0.mm_per_s());
            let travelled = (end.s - state.s).as_mm();
            assert!(
                (travelled - expected).abs() < 0.05 * expected + 2.0 * ninja_resolution(),
                "stopped in {travelled} mm, expected {expected} mm"
            );
        }
    }

    #[test]
    fn partial_increment_ends_exactly() {
        let distance = (10.5 * ninja_resolution()).mm();
        let plan = profile().plan(State::REST, distance, 0.mm_per_s());
        let d = delays(plan);
        assert_eq!(d.len(), 11);
        assert_eq!(run(plan, State::REST), State::at(distance));
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
        for i in 1..=200 {
            let f = fine.advance(DT);
            if i % 50 == 0 {
                let c = coarse.advance(Duration::from_millis(50));
                assert!((c.s - f.s).abs() < 1e-2.mm());
                assert_eq!(c.v, f.v);
            }
        }
    }
}

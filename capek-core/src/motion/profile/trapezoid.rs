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

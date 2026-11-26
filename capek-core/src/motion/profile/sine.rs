use super::{Plan, Profile, State};
use crate::units::{Acceleration, Length, Speed};
use core::f32::consts::PI;
use core::time::Duration;

/// Ramps whose acceleration rises and falls as a half sine.
///
/// Each speed change follows `a(t) = accel·sin(π·t / T)`, so acceleration
/// starts and ends at zero and never jumps, without the cost of planning an
/// [`SCurve`](super::SCurve). Like [`Trapezoid`](super::Trapezoid), a move is a
/// ramp up, a cruise and a ramp down, or two ramps when it is too short to
/// reach `max_speed`. Every state is computed in closed form.
///
/// A half-sine ramp covers the same distance as a constant-acceleration ramp
/// at `2·accel / π`, so a move takes about 1.57 times as long to change speed
/// as a trapezoid with the same peak acceleration.
///
/// The acceleration in the state a plan starts from is ignored, so replanning
/// in the middle of a ramp makes acceleration jump. [`SCurve`](super::SCurve)
/// takes it into
/// account.
///
/// ```
/// use capek_core::motion::profile::{Plan, Profile, Sine, State};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let profile = Sine { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
/// let mut plan = profile.plan(State::REST, 1.m(), 0.mm_per_s());
///
/// // Acceleration starts from zero instead of jumping to `accel`.
/// let start = plan.advance(Duration::from_millis(1));
/// assert!(start.a < 10.mm_per_s2());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sine {
    /// Cruise speed. Must be positive.
    pub max_speed: Speed,
    /// Peak acceleration and deceleration. Must be positive.
    pub accel: Acceleration,
}

/// A half-sine speed change from `v0` to `v1` over `t` seconds.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Ramp {
    v0: f32,
    v1: f32,
    t: f32,
}

impl Ramp {
    /// The ramp from `v0` to `v1` with peak acceleration `a`.
    fn new(v0: f32, v1: f32, a: f32) -> Self {
        Self {
            v0,
            v1,
            t: PI * libm::fabsf(v1 - v0) / (2.0 * a),
        }
    }

    fn distance(&self) -> f32 {
        (self.v0 + self.v1) / 2.0 * self.t
    }

    /// Distance, speed and acceleration `tau` seconds into the ramp.
    fn at(&self, tau: f32) -> (f32, f32, f32) {
        let dv = self.v1 - self.v0;
        let w = PI / self.t;
        let (sin, cos) = (libm::sinf(w * tau), libm::cosf(w * tau));
        let s = self.v0 * tau + dv / 2.0 * (tau - sin / w);
        let v = self.v0 + dv / 2.0 * (1.0 - cos);
        (s, v, dv / 2.0 * w * sin)
    }
}

impl Profile for Sine {
    type Plan = SinePlan;

    fn plan(&self, from: State, distance: Length, end_speed: Speed) -> SinePlan {
        let vmax = self.max_speed.as_mm_per_s();
        let a = self.accel.as_mm_per_s2();
        debug_assert!(vmax > 0.0, "Sine::max_speed must be positive");
        debug_assert!(a > 0.0, "Sine::accel must be positive");
        // The constant acceleration that covers the same distance.
        let ae = 2.0 * a / PI;

        let d = distance.as_mm().max(0.0);
        let v0 = from.v.as_mm_per_s().max(0.0);
        let mut v1 = end_speed.as_mm_per_s().clamp(0.0, vmax);

        // From here on the plan is a trapezoid's at `ae`.
        let reach_sq = 2.0 * ae * d;
        if v1 > v0 {
            v1 = v1.min(libm::sqrtf(v0 * v0 + reach_sq));
        } else if v0 * v0 - v1 * v1 > reach_sq {
            v1 = libm::sqrtf(v0 * v0 - reach_sq);
        }
        let apex = libm::sqrtf((reach_sq + v0 * v0 + v1 * v1) / 2.0);
        let vp = apex.min(vmax).max(v1);

        let up = Ramp::new(v0, vp, a);
        let down = Ramp::new(vp, v1, a);
        let cruise = (d - up.distance() - down.distance()).max(0.0);
        let s0 = from.s.as_mm();
        SinePlan {
            t: 0.0,
            s0,
            up,
            vp,
            t2: up.t + if vp > 0.0 { cruise / vp } else { 0.0 },
            s2: s0 + up.distance() + cruise,
            down,
            end: Length::mm(s0 + d),
        }
    }

    fn stop(&self, from: State) -> SinePlan {
        let v0 = from.v.as_mm_per_s().max(0.0);
        let s0 = from.s.as_mm();
        let down = Ramp::new(v0, 0.0, self.accel.as_mm_per_s2());
        SinePlan {
            t: 0.0,
            s0,
            up: Ramp::new(v0, v0, 1.0),
            vp: v0,
            t2: 0.0,
            s2: s0,
            down,
            end: Length::mm(s0 + down.distance()),
        }
    }
}

/// A move planned by [`Sine`].
///
/// ```
/// use capek_core::motion::profile::{Plan, Profile, Sine, State};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let profile = Sine { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
/// let mut plan = profile.plan(State::REST, 50.mm(), 0.mm_per_s());
/// assert!(plan.peak_speed() < 400.mm_per_s()); // too short to reach cruise
///
/// let end = plan.advance(Duration::from_secs(1));
/// assert!(plan.is_done());
/// assert_eq!(end.s, 50.mm());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SinePlan {
    /// Time elapsed, in seconds.
    t: f32,
    /// Start position, mm.
    s0: f32,
    /// First ramp, from the start speed to the cruise speed.
    up: Ramp,
    /// Cruise speed, mm/s.
    vp: f32,
    /// End of the cruise: time and position.
    t2: f32,
    s2: f32,
    /// Last ramp, down to the end speed.
    down: Ramp,
    /// End position, kept exact.
    end: Length,
}

impl SinePlan {
    /// Total time the plan takes.
    ///
    /// ```
    /// use capek_core::motion::profile::{Profile, Sine, State};
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = Sine { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
    /// let plan = profile.plan(State::REST, 1.m(), 0.mm_per_s());
    /// // Each ramp takes π·400 / 2400 s and covers 104.7 mm.
    /// assert!((plan.duration().as_secs_f32() - 3.0236).abs() < 1e-3);
    /// ```
    pub fn duration(&self) -> Duration {
        Duration::from_secs_f32(self.t2 + self.down.t)
    }

    /// Highest speed reached during the plan.
    ///
    /// ```
    /// use capek_core::motion::profile::{Profile, Sine, State};
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = Sine { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2() };
    /// let plan = profile.plan(State::REST, 1.m(), 0.mm_per_s());
    /// assert_eq!(plan.peak_speed(), 400.mm_per_s());
    /// ```
    pub fn peak_speed(&self) -> Speed {
        Speed::mm_per_s(self.vp.max(self.up.v0))
    }

    /// The state at the current time.
    fn state(&self) -> State {
        let t = self.t;
        let (s, v, a) = if t >= self.t2 + self.down.t {
            return State {
                s: self.end,
                v: Speed::mm_per_s(self.down.v1),
                a: Acceleration::ZERO,
            };
        } else if t < self.up.t {
            let (s, v, a) = self.up.at(t);
            (self.s0 + s, v, a)
        } else if t < self.t2 {
            let s1 = self.s0 + self.up.distance();
            (s1 + self.vp * (t - self.up.t), self.vp, 0.0)
        } else {
            let (s, v, a) = self.down.at(t - self.t2);
            (self.s2 + s, v, a)
        };
        State {
            s: Length::mm(s),
            v: Speed::mm_per_s(v),
            a: Acceleration::mm_per_s2(a),
        }
    }
}

impl Plan for SinePlan {
    fn advance(&mut self, dt: Duration) -> State {
        self.t += dt.as_secs_f32();
        self.state()
    }

    fn is_done(&self) -> bool {
        self.t >= self.t2 + self.down.t
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::UnitExt;

    const DT: Duration = Duration::from_millis(1);

    fn profile() -> Sine {
        Sine {
            max_speed: 400.mm_per_s(),
            accel: 1200.mm_per_s2(),
        }
    }

    /// Runs a plan to the end in 1 ms slices, checking the limits and the
    /// continuity of speed and acceleration on the way.
    fn run(mut plan: SinePlan, from: State) -> State {
        let vmax = 400.0_f32.max(from.v.as_mm_per_s()) + 1e-3;
        let dt = DT.as_secs_f32();
        let mut prev = from;
        let mut steps = 0;
        while !plan.is_done() {
            let s = plan.advance(DT);
            steps += 1;
            assert!(s.v.as_mm_per_s() <= vmax, "speed limit at {steps}: {s:?}");
            assert!(s.v.as_mm_per_s() >= -1e-3, "negative speed at {steps}");
            assert!(s.a.abs() <= 1200.mm_per_s2() + 1e-2.mm_per_s2());
            let dv = (s.v - prev.v).abs().as_mm_per_s();
            assert!(dv <= 1200.0 * dt + 1e-2, "speed jump {dv} at {steps}");
            if !plan.is_done() && steps > 1 {
                // The steepest ramp here takes π·400 / 2400 s, so the
                // acceleration changes by at most accel·π / T per second.
                let da = (s.a - prev.a).abs().as_mm_per_s2();
                assert!(
                    da < 1200.0 * PI / 0.05 * dt,
                    "acceleration jump {da} at {steps}"
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

    #[test]
    fn ramp_covers_the_trapezoid_distance_at_two_over_pi() {
        let r = Ramp::new(100.0, 400.0, 1200.0);
        let trapezoid = (400.0_f32 * 400.0 - 100.0 * 100.0) / (2.0 * 2.0 * 1200.0 / PI);
        assert!(close(r.distance(), trapezoid, 1e-2));
        let (s, v, a) = r.at(r.t);
        assert!(close(s, r.distance(), 1e-2));
        assert!(close(v, 400.0, 1e-3));
        assert!(close(a, 0.0, 1e-2));
        // Peak acceleration halfway.
        assert!(close(r.at(r.t / 2.0).2, 1200.0, 1e-2));
    }

    #[test]
    fn rest_to_rest_reaches_cruise() {
        let plan = profile().plan(State::REST, 1.m(), 0.mm_per_s());
        assert_eq!(plan.peak_speed(), 400.mm_per_s());
        let end = run(plan, State::REST);
        assert_eq!(end, State::at(1.m()));
    }

    #[test]
    fn short_move_is_two_ramps() {
        let plan = profile().plan(State::REST, 30.mm(), 0.mm_per_s());
        // A trapezoid's triangle at 2·accel / π: sqrt(ae · d).
        let ae = 2.0 * 1200.0 / PI;
        assert!(close(
            plan.peak_speed().as_mm_per_s(),
            libm::sqrtf(ae * 30.0),
            1e-2
        ));
        assert_eq!(run(plan, State::REST), State::at(30.mm()));
    }

    #[test]
    fn plans_between_moving_states() {
        let from = State {
            s: 100.mm(),
            v: 200.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        let end = run(profile().plan(from, 500.mm(), 100.mm_per_s()), from);
        assert_eq!(end.s, 600.mm());
        assert_eq!(end.v, 100.mm_per_s());
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
        assert_eq!(run(plan, first), State::at(1.m()));
    }

    #[test]
    fn unreachable_end_speed_never_overshoots() {
        let ae = 2.0 * 1200.0 / PI;
        let from = State {
            s: 0.mm(),
            v: 400.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        let end = run(profile().plan(from, 20.mm(), 0.mm_per_s()), from);
        assert_eq!(end.s, 20.mm());
        assert!(close(
            end.v.as_mm_per_s(),
            libm::sqrtf(160_000.0 - 2.0 * ae * 20.0),
            1e-2
        ));

        let end = run(
            profile().plan(State::REST, 10.mm(), 400.mm_per_s()),
            State::REST,
        );
        assert!(close(
            end.v.as_mm_per_s(),
            libm::sqrtf(2.0 * ae * 10.0),
            1e-2
        ));
    }

    #[test]
    fn stop_from_every_phase() {
        let p = profile();
        let mut plan = p.plan(State::REST, 1.m(), 0.mm_per_s());
        while !plan.is_done() {
            let state = plan.advance(DT);
            // The stop starts from zero acceleration, so only check the end.
            let mut stop = p.stop(state);
            let mut end = state;
            while !stop.is_done() {
                end = stop.advance(DT);
            }
            let expected = PI * state.v.as_mm_per_s().powi(2) / 4800.0;
            assert_eq!(end.v, 0.mm_per_s());
            assert!(close((end.s - state.s).as_mm(), expected, 1e-2));
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
}

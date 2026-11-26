use super::{Plan, Profile, State};
use crate::units::{Acceleration, Length, Speed};
use core::time::Duration;

/// A fixed speed with no ramps: jumps straight to `speed` and stops dead.
///
/// Suits motors that can start at full speed, such as a 28BYJ-48, and tests.
///
/// A plan moves at `speed` for its whole distance, whatever the speed it
/// starts from. Its final state reports the requested end speed, capped at
/// `speed`, so the next plan can pick up from it. Acceleration is always
/// reported as zero, since the speed changes are instantaneous. [`stop`]
/// brings any state to rest on the spot.
///
/// [`stop`]: Profile::stop
///
/// ```
/// use capek_core::motion::profile::{Constant, Plan, Profile, State};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let profile = Constant { speed: 200.mm_per_s() };
/// let mut plan = profile.plan(State::REST, 100.mm(), 0.mm_per_s());
/// assert_eq!(plan.duration(), Duration::from_millis(500));
///
/// // Full speed from the first step.
/// assert_eq!(plan.advance(Duration::from_millis(1)).v, 200.mm_per_s());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Constant {
    /// Speed of the whole move. Must be positive.
    pub speed: Speed,
}

impl Profile for Constant {
    type Plan = ConstantPlan;

    fn plan(&self, from: State, distance: Length, end_speed: Speed) -> ConstantPlan {
        let v = self.speed.as_mm_per_s();
        debug_assert!(v > 0.0, "Constant::speed must be positive");

        let d = distance.as_mm().max(0.0);
        let s0 = from.s.as_mm();
        ConstantPlan {
            t: 0.0,
            s0,
            v,
            t_end: d / v,
            end: Length::mm(s0 + d),
            v1: end_speed.as_mm_per_s().clamp(0.0, v),
        }
    }

    fn stop(&self, from: State) -> ConstantPlan {
        ConstantPlan {
            t: 0.0,
            s0: from.s.as_mm(),
            v: self.speed.as_mm_per_s(),
            t_end: 0.0,
            end: from.s,
            v1: 0.0,
        }
    }

    fn max_accel(&self) -> Acceleration {
        Acceleration::mm_per_s2(f32::INFINITY)
    }

    fn max_speed(&self) -> Speed {
        self.speed
    }

    fn scaled(&self, factor: f32) -> Self {
        Self {
            speed: self.speed * factor,
        }
    }

    fn reachable(&self, _from: Speed, _distance: Length) -> Speed {
        // Speed changes are instantaneous.
        self.speed
    }
}

/// A move planned by [`Constant`].
///
/// ```
/// use capek_core::motion::profile::{Constant, Plan, Profile, State};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let profile = Constant { speed: 200.mm_per_s() };
/// let mut plan = profile.plan(State::REST, 100.mm(), 0.mm_per_s());
///
/// let end = plan.advance(Duration::from_secs(1));
/// assert!(plan.is_done());
/// assert_eq!(end, State::at(100.mm()));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConstantPlan {
    /// Time elapsed, in seconds.
    t: f32,
    /// Start position, mm.
    s0: f32,
    /// Speed of the move, mm/s.
    v: f32,
    /// End of the plan, in seconds.
    t_end: f32,
    /// End position, kept exact.
    end: Length,
    /// Speed reported once the plan is done, mm/s.
    v1: f32,
}

impl ConstantPlan {
    /// Total time the plan takes.
    ///
    /// ```
    /// use capek_core::motion::profile::{Constant, Profile, State};
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = Constant { speed: 400.mm_per_s() };
    /// let plan = profile.plan(State::REST, 1.m(), 0.mm_per_s());
    /// assert_eq!(plan.duration().as_secs_f32(), 2.5);
    /// ```
    pub fn duration(&self) -> Duration {
        Duration::from_secs_f32(self.t_end)
    }

    /// The state at the current time.
    fn state(&self) -> State {
        if self.t >= self.t_end {
            return State {
                s: self.end,
                v: Speed::mm_per_s(self.v1),
                a: Acceleration::ZERO,
            };
        }
        State {
            s: Length::mm(self.s0 + self.v * self.t),
            v: Speed::mm_per_s(self.v),
            a: Acceleration::ZERO,
        }
    }
}

impl Plan for ConstantPlan {
    fn advance(&mut self, dt: Duration) -> State {
        self.t += dt.as_secs_f32();
        self.state()
    }

    fn is_done(&self) -> bool {
        self.t >= self.t_end
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::UnitExt;

    const DT: Duration = Duration::from_millis(1);

    fn profile() -> Constant {
        Constant {
            speed: 200.mm_per_s(),
        }
    }

    /// Runs a plan to the end in 1 ms slices, checking the speed on the way.
    fn run(mut plan: ConstantPlan) -> (State, u32) {
        let mut prev = plan.state();
        let mut steps = 0;
        while !plan.is_done() {
            let s = plan.advance(DT);
            steps += 1;
            if !plan.is_done() {
                assert_eq!(s.v, 200.mm_per_s(), "speed changed at step {steps}");
            }
            assert_eq!(s.a, 0.mm_per_s2());
            assert!(s.s >= prev.s, "went backwards at step {steps}");
            prev = s;
            assert!(steps < 60_000, "plan never finished");
        }
        (prev, steps)
    }

    #[test]
    fn moves_at_speed_and_stops_dead() {
        let plan = profile().plan(State::REST, 100.mm(), 0.mm_per_s());
        let (end, steps) = run(plan);
        assert!((steps as i32 - 500).abs() <= 1);
        assert_eq!(end, State::at(100.mm()));
    }

    #[test]
    fn ignores_start_speed() {
        let from = State {
            s: 50.mm(),
            v: 600.mm_per_s(),
            a: 100.mm_per_s2(),
        };
        let mut plan = profile().plan(from, 100.mm(), 0.mm_per_s());
        let first = plan.advance(DT);
        assert_eq!(first.v, 200.mm_per_s());
        assert_eq!(first.a, 0.mm_per_s2());
        let (end, _) = run(plan);
        assert_eq!(end.s, 150.mm());
    }

    #[test]
    fn end_speed_is_capped_at_speed() {
        let mut plan = profile().plan(State::REST, 10.mm(), 100.mm_per_s());
        assert_eq!(plan.advance(Duration::from_secs(1)).v, 100.mm_per_s());

        let mut plan = profile().plan(State::REST, 10.mm(), 500.mm_per_s());
        assert_eq!(plan.advance(Duration::from_secs(1)).v, 200.mm_per_s());
    }

    #[test]
    fn stop_is_immediate() {
        let mut plan = profile().plan(State::REST, 100.mm(), 0.mm_per_s());
        let state = plan.advance(Duration::from_millis(250));
        let mut stop = profile().stop(state);
        assert!(stop.is_done());
        assert_eq!(stop.advance(DT), State::at(state.s));
    }

    #[test]
    fn zero_distance_is_done() {
        let mut plan = profile().plan(State::at(5.mm()), 0.mm(), 0.mm_per_s());
        assert!(plan.is_done());
        assert_eq!(plan.advance(DT), State::at(5.mm()));
    }

    #[test]
    fn negative_inputs_are_zero() {
        let mut plan = profile().plan(State::REST, (-10).mm(), (-5).mm_per_s());
        assert!(plan.is_done());
        assert_eq!(plan.advance(DT), State::REST);
    }

    #[test]
    fn coarse_steps_match_fine_steps() {
        let mut fine = profile().plan(State::REST, 100.mm(), 0.mm_per_s());
        let mut coarse = fine;
        for i in 1..=400 {
            let f = fine.advance(DT);
            if i % 50 == 0 {
                let c = coarse.advance(Duration::from_millis(50));
                assert!((c.s - f.s).abs() < 1e-3.mm());
            }
        }
    }
}

use super::trapezoid::{DEFAULT_RESOLUTION, index, start_delay};
use super::{Plan, Profile, State};
use crate::units::{Acceleration, Length, Speed};
use core::time::Duration;

/// Ninja's power curve: full acceleration at rest, easing into cruise.
///
/// Speed over distance follows `v = v∞·√(u / (1 + u^(p−1))^(1/(p−1)))`, with
/// `u = s·2·accel / v∞²`. At rest this accelerates at `accel`, and the
/// acceleration fades as speed approaches `v∞`; a larger `p` keeps it strong
/// for longer and fades it more sharply. The curve never reaches `v∞`, so,
/// like ninja, the ramp ends at `u = (p + 2) / 2`, and `v∞` is chosen so that
/// this point is exactly `max_speed`. Deceleration mirrors acceleration.
///
/// Time along the curve has no closed form, so the plan integrates it
/// numerically: durations with Simpson's rule, and position with fourth-order
/// Runge-Kutta in steps of at most 1 ms. This costs a few `powf` calls per
/// step; [`FastPower`] avoids most of them.
///
/// A plan that starts above `max_speed` first brakes at a constant `accel`
/// until it meets the curve. Acceleration jumps where ramps start and end.
/// The acceleration in the state a plan starts from is ignored.
///
/// Ninja's `PowerProfile` takes `v∞` as its `max_speed`. For the same motion,
/// set `max_speed` here to [`Power::max_speed_for`] of ninja's value, about
/// 0.96 times it when `p` is 3.
///
/// ```
/// use capek_core::motion::profile::{Plan, Power, Profile, State};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let profile = Power { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2(), p: 3.0 };
/// let mut plan = profile.plan(State::REST, 1.m(), 0.mm_per_s());
///
/// // Full acceleration from rest, fading as the robot speeds up.
/// let early = plan.advance(Duration::from_millis(10));
/// let later = plan.advance(Duration::from_millis(300));
/// assert!(early.a > later.a);
/// assert!(later.v < 400.mm_per_s());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Power {
    /// Speed at the end of the ramp, the cruise speed. Must be positive.
    pub max_speed: Speed,
    /// Acceleration at rest, the largest along the curve. Must be positive.
    pub accel: Acceleration,
    /// Shape of the curve. Must be greater than 1; ninja uses 2 or 3.
    pub p: f32,
}

impl Power {
    /// The `max_speed` that gives the same motion as ninja's `PowerProfile`
    /// with the asymptotic speed `v_inf` and exponent `p`.
    ///
    /// ```
    /// use capek_core::motion::profile::Power;
    /// use capek_core::units::UnitExt;
    ///
    /// let max_speed = Power::max_speed_for(1000.mm_per_s(), 3.0);
    /// assert!((max_speed - 963.6.mm_per_s()).abs() < 0.1.mm_per_s());
    /// ```
    pub fn max_speed_for(v_inf: Speed, p: f32) -> Speed {
        let q = p - 1.0;
        let u = (p + 2.0) / 2.0;
        v_inf * libm::sqrtf(u / libm::powf(1.0 + libm::powf(u, q), 1.0 / q))
    }

    /// The curve these limits describe.
    fn shape(&self) -> Shape {
        let vmax = self.max_speed.as_mm_per_s();
        let a = self.accel.as_mm_per_s2();
        debug_assert!(vmax > 0.0, "Power::max_speed must be positive");
        debug_assert!(a > 0.0, "Power::accel must be positive");
        debug_assert!(self.p > 1.0, "Power::p must be greater than 1");

        let p = self.p;
        let q = p - 1.0;
        let vinf = vmax / (Self::max_speed_for(Speed::mm_per_s(1.0), p).as_mm_per_s());
        Shape {
            vinf,
            vmax,
            a,
            p,
            q,
            len: vinf * vinf / (2.0 * a),
            sigma_end: libm::sqrtf((p + 2.0) / 2.0),
        }
    }
}

/// The power curve in terms of `σ = √u`.
///
/// Speed is `v∞·σ·f(σ)` and acceleration `accel·f(σ)^(2p)`, with
/// `f(σ) = (1 + σ^(2q))^(−1/(2q))` and `q = p − 1`. Time along the curve is
/// `dt = dσ / (κ·f(σ))` with `κ = accel / v∞`, which, unlike time over
/// distance, stays finite at rest.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Shape {
    /// Asymptotic speed and cruise speed, mm/s.
    vinf: f32,
    vmax: f32,
    /// Acceleration at rest, mm/s².
    a: f32,
    /// Exponent and `p − 1`.
    p: f32,
    q: f32,
    /// Distance for one unit of `u`, mm.
    len: f32,
    /// `σ` at the end of the ramp.
    sigma_end: f32,
}

impl Shape {
    fn f(&self, sigma: f32) -> f32 {
        libm::powf(1.0 + libm::powf(sigma, 2.0 * self.q), -1.0 / (2.0 * self.q))
    }

    fn speed(&self, sigma: f32) -> f32 {
        self.vinf * sigma * self.f(sigma)
    }

    fn accel(&self, sigma: f32) -> f32 {
        self.a * libm::powf(self.f(sigma), 2.0 * self.p)
    }

    /// `σ` where the curve reaches speed `v`, capped at the end of the ramp.
    fn sigma_at(&self, v: f32) -> f32 {
        if v >= self.vmax {
            return self.sigma_end;
        }
        let w = v * v / (self.vinf * self.vinf);
        let u = w / libm::powf(1.0 - libm::powf(w, self.q), 1.0 / self.q);
        libm::sqrtf(u).min(self.sigma_end)
    }

    /// Rate of change of `σ` over time, without its sign.
    fn rate(&self, sigma: f32) -> f32 {
        self.a / self.vinf * self.f(sigma)
    }

    /// Time to move along the curve between `from` and `to`, in seconds.
    fn time(&self, from: f32, to: f32) -> f32 {
        // Composite Simpson's rule on the smooth integrand 1 / rate.
        const PANELS: usize = 32;
        let (lo, hi) = if from < to { (from, to) } else { (to, from) };
        let h = (hi - lo) / PANELS as f32;
        let mut sum = 1.0 / self.rate(lo) + 1.0 / self.rate(hi);
        for i in 1..PANELS {
            let w = if i % 2 == 1 { 4.0 } else { 2.0 };
            sum += w / self.rate(lo + h * i as f32);
        }
        sum * h / 3.0
    }

    /// Distance between two points on the curve, in mm, without its sign.
    fn distance(&self, from: f32, to: f32) -> f32 {
        libm::fabsf(to * to - from * from) * self.len
    }
}

fn sq(x: f32) -> f32 {
    x * x
}

/// One piece of a plan.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Segment {
    /// Constant acceleration `a` from speed `v0`: cruise, or braking down to
    /// the curve from above `max_speed`.
    Linear { v0: f32, a: f32 },
    /// Along the curve from `σ = from` to `σ = to`, up or down.
    Curve { from: f32, to: f32 },
}

/// A segment with its start position, mm, and duration, s.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Piece {
    seg: Segment,
    s0: f32,
    dur: f32,
}

/// Collects the pieces of a plan.
struct Builder {
    shape: Shape,
    pieces: [Piece; 3],
    count: usize,
    s: f32,
}

impl Builder {
    fn new(shape: Shape, s0: f32) -> Self {
        let none = Piece {
            seg: Segment::Linear { v0: 0.0, a: 0.0 },
            s0,
            dur: 0.0,
        };
        Self {
            shape,
            pieces: [none; 3],
            count: 0,
            s: s0,
        }
    }

    fn push(&mut self, seg: Segment, dur: f32, distance: f32) {
        if dur > 0.0 {
            self.pieces[self.count] = Piece {
                seg,
                s0: self.s,
                dur,
            };
            self.count += 1;
            self.s += distance;
        }
    }

    /// Brakes at `accel` from `v0` to `v1`.
    fn brake(&mut self, v0: f32, v1: f32) {
        let a = self.shape.a;
        let seg = Segment::Linear { v0, a: -a };
        self.push(seg, (v0 - v1) / a, (v0 * v0 - v1 * v1) / (2.0 * a));
    }

    fn cruise(&mut self, v: f32, distance: f32) {
        // Shorter than a micrometre is rounding left over from the ramps.
        if v > 0.0 && distance > 1e-3 {
            self.push(Segment::Linear { v0: v, a: 0.0 }, distance / v, distance);
        }
    }

    fn curve(&mut self, from: f32, to: f32) {
        if from != to {
            let (dur, distance) = (self.shape.time(from, to), self.shape.distance(from, to));
            self.push(Segment::Curve { from, to }, dur, distance);
        }
    }

    fn finish(self, end: Length, v_end: f32, peak: f32) -> PowerPlan {
        let sigma = match self.pieces[0].seg {
            Segment::Curve { from, .. } => from,
            Segment::Linear { .. } => 0.0,
        };
        PowerPlan {
            shape: self.shape,
            pieces: self.pieces,
            count: self.count,
            i: 0,
            tau: 0.0,
            sigma,
            end,
            v_end,
            peak,
        }
    }
}

impl Profile for Power {
    type Plan = PowerPlan;

    fn plan(&self, from: State, distance: Length, end_speed: Speed) -> PowerPlan {
        let shape = self.shape();
        let (a, vmax, len) = (shape.a, shape.vmax, shape.len);
        let d = distance.as_mm().max(0.0);
        let v0 = from.v.as_mm_per_s().max(0.0);
        let mut v1 = end_speed.as_mm_per_s().clamp(0.0, vmax);
        let s0 = from.s.as_mm();
        let end = Length::mm(s0 + d);
        let mut b = Builder::new(shape, s0);

        if v0 > vmax {
            // Brake at `accel` down to the curve, then follow it down to the
            // end speed. Find the speed where the two meet so that they cover
            // exactly the distance: the distance they need only grows with it.
            let sigma1 = shape.sigma_at(v1);
            let needed = |vx: f32| {
                (v0 * v0 - vx * vx) / (2.0 * a) + shape.distance(shape.sigma_at(vx), sigma1)
            };
            if needed(v1) >= d {
                // Even braking straight to the end speed takes too long.
                let v_end = libm::sqrtf((v0 * v0 - 2.0 * a * d).max(v1 * v1));
                b.brake(v0, v_end);
                return b.finish(end, v_end, v0);
            }
            let vx = if needed(vmax) <= d {
                vmax
            } else {
                let (mut lo, mut hi) = (v1, vmax);
                for _ in 0..32 {
                    let mid = (lo + hi) / 2.0;
                    if needed(mid) > d {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                lo
            };
            let sx = shape.sigma_at(vx);
            b.brake(v0, vx);
            b.cruise(vx, d - needed(vx));
            b.curve(sx, sigma1);
            return b.finish(end, shape.speed(sigma1), v0);
        }

        // In u = σ² speed squared moves linearly with distance on both ramps,
        // so the plan has the shape of a trapezoid there.
        let span = d / len;
        let u0 = sq(shape.sigma_at(v0));
        let mut u1 = sq(shape.sigma_at(v1));
        // Keep the end speed within what the distance allows.
        u1 = u1.min(u0 + span).max(u0 - span).max(0.0);
        v1 = shape.speed(libm::sqrtf(u1));

        let apex = (span + u0 + u1) / 2.0;
        let up = apex.min(sq(shape.sigma_end)).max(u1);
        let (sigma0, sigma_p, sigma1) = (libm::sqrtf(u0), libm::sqrtf(up), libm::sqrtf(u1));
        let vp = shape.speed(sigma_p);

        b.curve(sigma0, sigma_p);
        b.cruise(vp, d - ((up - u0) + (up - u1)) * len);
        b.curve(sigma_p, sigma1);
        b.finish(end, v1, vp.max(v0))
    }

    fn stop(&self, from: State) -> PowerPlan {
        let shape = self.shape();
        let v0 = from.v.as_mm_per_s().max(0.0);
        let s0 = from.s.as_mm();
        let mut b = Builder::new(shape, s0);
        let mut d = 0.0;
        if v0 > shape.vmax {
            b.brake(v0, shape.vmax);
            d += (v0 * v0 - shape.vmax * shape.vmax) / (2.0 * shape.a);
        }
        let sigma = shape.sigma_at(v0);
        b.curve(sigma, 0.0);
        d += shape.distance(sigma, 0.0);
        b.finish(Length::mm(s0 + d), 0.0, v0)
    }

    fn max_speed(&self) -> Speed {
        self.max_speed
    }

    fn scaled(&self, factor: f32) -> Self {
        Self {
            max_speed: self.max_speed * factor,
            accel: self.accel * factor,
            p: self.p,
        }
    }

    fn reachable(&self, from: Speed, distance: Length) -> Speed {
        let shape = self.shape();
        let v0 = from.as_mm_per_s().max(0.0);
        if v0 >= shape.vmax {
            return self.max_speed;
        }
        // Speed squared moves linearly in u along the curve.
        let u = sq(shape.sigma_at(v0)) + distance.as_mm().max(0.0) / shape.len;
        let sigma = libm::sqrtf(u).min(shape.sigma_end);
        Speed::mm_per_s(shape.speed(sigma).min(self.max_speed.as_mm_per_s()))
    }
}

/// A move planned by [`Power`].
///
/// ```
/// use capek_core::motion::profile::{Plan, Power, Profile, State};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let profile = Power { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2(), p: 3.0 };
/// let mut plan = profile.plan(State::REST, 50.mm(), 0.mm_per_s());
/// assert!(plan.peak_speed() < 400.mm_per_s()); // too short to reach cruise
///
/// let end = plan.advance(Duration::from_secs(1));
/// assert!(plan.is_done());
/// assert_eq!(end.s, 50.mm());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PowerPlan {
    shape: Shape,
    pieces: [Piece; 3],
    count: usize,
    /// Current piece and time spent in it, s.
    i: usize,
    tau: f32,
    /// Position on the curve, while in a curve piece.
    sigma: f32,
    /// End position, kept exact, and end speed, mm/s.
    end: Length,
    v_end: f32,
    /// Highest speed, mm/s.
    peak: f32,
}

impl PowerPlan {
    /// Longest Runge-Kutta step, in seconds.
    const STEP: f32 = 1e-3;

    /// Total time the plan takes.
    ///
    /// ```
    /// use capek_core::motion::profile::{Power, Profile, State};
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = Power { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2(), p: 3.0 };
    /// let plan = profile.plan(State::REST, 1.m(), 0.mm_per_s());
    /// // 2.89 s: slower than a trapezoid with the same limits, at 2.83 s.
    /// assert!((plan.duration().as_secs_f32() - 2.887).abs() < 1e-3);
    /// ```
    pub fn duration(&self) -> Duration {
        let total: f32 = self.pieces[..self.count].iter().map(|p| p.dur).sum();
        Duration::from_secs_f32(total)
    }

    /// Highest speed reached during the plan.
    ///
    /// ```
    /// use capek_core::motion::profile::{Power, Profile, State};
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = Power { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2(), p: 3.0 };
    /// let plan = profile.plan(State::REST, 1.m(), 0.mm_per_s());
    /// assert!((plan.peak_speed() - 400.mm_per_s()).abs() < 0.01.mm_per_s());
    /// ```
    pub fn peak_speed(&self) -> Speed {
        Speed::mm_per_s(self.peak)
    }

    /// Moves `σ` along the current curve piece by `dt`, towards `to`.
    fn integrate(&mut self, to: f32, dt: f32) {
        let sign = if to >= self.sigma { 1.0 } else { -1.0 };
        let steps = libm::ceilf(dt / Self::STEP).max(1.0);
        let h = dt / steps;
        let rate = |s: f32| sign * self.shape.rate(s.max(0.0));
        let mut s = self.sigma;
        for _ in 0..steps as u32 {
            let k1 = rate(s);
            let k2 = rate(s + h / 2.0 * k1);
            let k3 = rate(s + h / 2.0 * k2);
            let k4 = rate(s + h * k3);
            s += h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        }
        // Never pass the end of the piece through rounding.
        self.sigma = if sign > 0.0 { s.min(to) } else { s.max(to) };
    }

    /// The state at the current time.
    fn state(&self) -> State {
        if self.i >= self.count {
            return State {
                s: self.end,
                v: Speed::mm_per_s(self.v_end),
                a: Acceleration::ZERO,
            };
        }
        let piece = self.pieces[self.i];
        let (s, v, a) = match piece.seg {
            Segment::Linear { v0, a } => {
                let t = self.tau;
                (piece.s0 + v0 * t + a * t * t / 2.0, v0 + a * t, a)
            }
            Segment::Curve { from, to } => {
                let sigma = self.sigma;
                let sh = &self.shape;
                let moved = sh.distance(from, sigma);
                let a = sh.accel(sigma);
                if to > from {
                    (piece.s0 + moved, sh.speed(sigma), a)
                } else {
                    (piece.s0 + moved, sh.speed(sigma), -a)
                }
            }
        };
        State {
            s: Length::mm(s),
            v: Speed::mm_per_s(v),
            a: Acceleration::mm_per_s2(a),
        }
    }
}

impl Plan for PowerPlan {
    fn advance(&mut self, dt: Duration) -> State {
        let mut left = dt.as_secs_f32();
        while self.i < self.count {
            let piece = self.pieces[self.i];
            let rest = piece.dur - self.tau;
            if left < rest {
                self.tau += left;
                if let Segment::Curve { to, .. } = piece.seg {
                    self.integrate(to, left);
                }
                break;
            }
            // Finish the piece exactly and start the next one.
            left -= rest;
            self.i += 1;
            self.tau = 0.0;
            if let Some(Piece {
                seg: Segment::Curve { from, .. },
                ..
            }) = self.pieces[..self.count].get(self.i)
            {
                self.sigma = *from;
            }
        }
        self.state()
    }

    fn is_done(&self) -> bool {
        self.i >= self.count
    }
}

/// [`Power`] computed per step with ninja's recurrence.
///
/// Like [`FastTrapezoid`](super::FastTrapezoid), the move is split into
/// distance increments of one resolution, and the time of each increment comes
/// from the previous one. Ninja adds a power term to Austin's recurrence,
/// `c[n] = c[n-1] − 2·c[n-1] / (4n + 1 + k·nᵖ)` with
/// `k = 4·(2·res·accel / v∞²)^(p−1)`, which bends the constant-acceleration
/// ramp into the power curve. Parent components set the resolution through
/// [`Profile::set_resolution`]; until then it is 0.1 mm.
/// [`advance`](Plan::advance) interpolates position within an increment.
///
/// The plan stays within a few percent of [`Power`], and `max_speed` has the
/// same meaning: the cruise speed actually reached, not ninja's `v∞`.
///
/// Differences from ninja's `PowerProfile`, which this reproduces for
/// rest-to-rest moves once `max_speed` is converted with
/// [`Power::max_speed_for`]:
///
/// - it uses `f32` instead of `f64`;
/// - it plans between non-zero speeds, starting the recurrence from the
///   increment that matches the start speed;
/// - deceleration runs the recurrence backwards instead of replaying the
///   acceleration delays;
/// - a plan that starts above `max_speed` first brakes at a constant `accel`,
///   with Austin's recurrence, until it meets the curve, as [`Power`] does;
/// - a distance that is not a whole number of increments ends with a partial
///   increment at cruise speed.
///
/// ```
/// use capek_core::motion::profile::{FastPower, Plan, Profile, State};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let mut profile = FastPower::new(400.mm_per_s(), 1200.mm_per_s2(), 3.0);
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
pub struct FastPower {
    /// Speed at the end of the ramp, the cruise speed. Must be positive.
    pub max_speed: Speed,
    /// Acceleration at rest, the largest along the curve. Must be positive.
    pub accel: Acceleration,
    /// Shape of the curve. Must be greater than 1; ninja uses 2 or 3.
    pub p: f32,
    /// Distance increment the recurrence steps through.
    resolution: Length,
}

impl FastPower {
    /// A profile with the given limits and the default resolution of 0.1 mm.
    ///
    /// ```
    /// use capek_core::motion::profile::FastPower;
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = FastPower::new(400.mm_per_s(), 1200.mm_per_s2(), 3.0);
    /// assert_eq!(profile.resolution(), 0.1.mm());
    /// ```
    pub const fn new(max_speed: Speed, accel: Acceleration, p: f32) -> Self {
        Self {
            max_speed,
            accel,
            p,
            resolution: DEFAULT_RESOLUTION,
        }
    }

    /// The distance increment the recurrence steps through.
    ///
    /// ```
    /// use capek_core::motion::profile::{FastPower, Profile};
    /// use capek_core::units::UnitExt;
    ///
    /// let mut profile = FastPower::new(400.mm_per_s(), 1200.mm_per_s2(), 3.0);
    /// profile.set_resolution(0.44.mm());
    /// assert_eq!(profile.resolution(), 0.44.mm());
    /// ```
    pub const fn resolution(&self) -> Length {
        self.resolution
    }

    /// The curve and its scale in increments.
    fn steps(&self) -> Steps {
        let shape = Power {
            max_speed: self.max_speed,
            accel: self.accel,
            p: self.p,
        }
        .shape();
        let res = self.resolution.as_mm();
        debug_assert!(res > 0.0, "FastPower resolution must be positive");
        // Increments per unit of u.
        let per_u = shape.len / res;
        Steps {
            shape,
            res,
            per_u,
            k: 4.0 / libm::powf(per_u, shape.q),
            nmax: (per_u * sq(shape.sigma_end)) as u32,
        }
    }
}

/// The power curve measured in increments.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Steps {
    shape: Shape,
    /// Increment length, mm.
    res: f32,
    /// Increments per unit of u, ninja's `v∞² / (2·accel·res)`.
    per_u: f32,
    /// Coefficient of the power term in the recurrence.
    k: f32,
    /// Index at the end of the ramp.
    nmax: u32,
}

impl Steps {
    /// Index on the curve at speed `v`, at most `nmax`.
    fn index(&self, v: f32) -> u32 {
        let n = libm::roundf(self.per_u * sq(self.shape.sigma_at(v))) as u32;
        n.min(self.nmax)
    }

    fn sigma(&self, n: f32) -> f32 {
        libm::sqrtf(n / self.per_u)
    }

    /// Speed on the curve at index `n`.
    fn speed(&self, n: u32) -> f32 {
        self.shape.speed(self.sigma(n as f32))
    }

    /// Time of increment `n` when starting the recurrence there.
    ///
    /// From rest this is Austin's corrected first delay. Further up it is the
    /// increment divided by the curve's speed halfway through it.
    fn start_delay(&self, n: u32) -> f32 {
        if n == 0 {
            start_delay(0, self.shape.a, self.res)
        } else {
            self.res / self.shape.speed(self.sigma(n as f32 + 0.5))
        }
    }

    /// The denominator of the recurrence at index `n`, less one.
    fn term(&self, n: u32) -> f32 {
        let n = n as f32;
        4.0 * n + self.k * libm::powf(n, self.shape.p)
    }
}

impl Profile for FastPower {
    type Plan = FastPowerPlan;

    fn plan(&self, from: State, distance: Length, end_speed: Speed) -> FastPowerPlan {
        let st = self.steps();
        let (res, a, vmax) = (st.res, st.shape.a, st.shape.vmax);

        let d = distance.as_mm().max(0.0);
        // Snap values a rounding error away from a whole number of increments.
        let mut steps = d / res;
        if libm::fabsf(steps - libm::roundf(steps)) < 1e-3 {
            steps = libm::roundf(steps);
        }
        let whole = steps as u32;
        let v0 = from.v.as_mm_per_s().max(0.0);
        let v1 = end_speed.as_mm_per_s().clamp(0.0, vmax);
        let s0 = from.s.as_mm();
        let end = Length::mm(s0 + d);

        if v0 > vmax {
            // Brake at `accel` down to the curve, counted in Austin's indices
            // `m`, then follow the curve down. As in `Power`, find where the
            // two meet so that they fit the distance.
            let m0 = index(v0, a, res);
            let n1 = st.index(v1);
            let brake_speed = |m: u32| libm::sqrtf(2.0 * a * res * m as f32);
            let needed = |m: u32| (m0 - m) + st.index(brake_speed(m)).saturating_sub(n1);
            let m1 = index(v1, a, res).min(m0);
            if needed(m1) >= whole {
                // Even braking straight to the end speed takes too long.
                let m_end = m0.saturating_sub(whole).max(m1);
                let cruise = steps - (m0 - m_end) as f32;
                let v_end = brake_speed(m_end);
                return FastPowerPlan::new(st, s0, m0, m_end, 0, cruise, 0, end, v_end, v0);
            }
            let m_top = index(vmax, a, res).min(m0);
            let mx = if needed(m_top) <= whole {
                m_top
            } else {
                // Largest index whose plan fits; `needed` grows with it.
                let (mut lo, mut hi) = (m1, m_top);
                while hi - lo > 1 {
                    let mid = (lo + hi) / 2;
                    if needed(mid) <= whole {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                lo
            };
            let nx = st.index(brake_speed(mx));
            let cruise = steps - needed(mx) as f32;
            return FastPowerPlan::new(st, s0, m0, mx, nx, cruise, n1, end, st.speed(n1), v0);
        }

        let n0 = st.index(v0);
        let n1 = st
            .index(v1)
            // Keep the end speed within what the distance allows.
            .min(n0.saturating_add(whole))
            .max(n0.saturating_sub(whole));
        // As in `FastTrapezoid`, both ramps are linear in the index.
        let apex = ((steps + n0 as f32 + n1 as f32) / 2.0) as u32;
        let peak = apex.min(st.nmax).max(n1);
        let cruise = (steps - ((peak - n0) + (peak - n1)) as f32).max(0.0);
        let peak_v = st.speed(peak).max(v0);
        FastPowerPlan::start(st, s0, n0, peak, cruise, n1, end, st.speed(n1), peak_v)
    }

    fn stop(&self, from: State) -> FastPowerPlan {
        let st = self.steps();
        let (res, a) = (st.res, st.shape.a);
        let v0 = from.v.as_mm_per_s().max(0.0);
        let s0 = from.s.as_mm();
        if v0 > st.shape.vmax {
            let m0 = index(v0, a, res);
            let mx = index(st.shape.vmax, a, res).min(m0);
            let nx = st.index(st.shape.vmax);
            let end = Length::mm(s0 + (m0 - mx + nx) as f32 * res);
            return FastPowerPlan::new(st, s0, m0, mx, nx, 0.0, 0, end, 0.0, v0);
        }
        let n0 = st.index(v0);
        let end = Length::mm(s0 + n0 as f32 * res);
        FastPowerPlan::start(st, s0, n0, n0, 0.0, 0, end, 0.0, v0)
    }

    fn set_resolution(&mut self, resolution: Length) {
        self.resolution = resolution;
    }

    fn max_speed(&self) -> Speed {
        self.max_speed
    }

    fn scaled(&self, factor: f32) -> Self {
        Self {
            max_speed: self.max_speed * factor,
            accel: self.accel * factor,
            p: self.p,
            resolution: self.resolution,
        }
    }

    fn reachable(&self, from: Speed, distance: Length) -> Speed {
        // In whole increments, as `plan` counts them.
        let st = self.steps();
        let v0 = from.as_mm_per_s().max(0.0);
        if v0 >= st.shape.vmax {
            return self.max_speed;
        }
        let whole = (distance.as_mm().max(0.0) / st.res + 1e-3) as u32;
        let n = st.index(v0).saturating_add(whole).min(st.nmax);
        Speed::mm_per_s(st.speed(n).min(self.max_speed.as_mm_per_s()))
    }
}

/// Phase of a [`FastPowerPlan`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FastPhase {
    /// Braking at `accel` from above `max_speed` down to the curve.
    Brake,
    /// Up the curve to the peak.
    Ramp,
    /// At the peak speed.
    Cruise,
    /// Down the curve to the end speed.
    Down,
    /// Finished.
    Done,
}

/// A move planned by [`FastPower`].
///
/// ```
/// use capek_core::motion::profile::{FastPower, Plan, Profile, State};
/// use capek_core::units::UnitExt;
/// use core::time::Duration;
///
/// let profile = FastPower::new(400.mm_per_s(), 1200.mm_per_s2(), 3.0);
/// let mut plan = profile.plan(State::REST, 50.mm(), 0.mm_per_s());
///
/// let end = plan.advance(Duration::from_secs(1));
/// assert!(plan.is_done());
/// assert_eq!(end.s, 50.mm());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FastPowerPlan {
    st: Steps,
    /// Start position, mm.
    s0: f32,
    phase: FastPhase,
    /// Austin index while braking, and where braking ends.
    m: u32,
    m_end: u32,
    /// Index on the curve and the time of the current increment, s.
    n: u32,
    c: f32,
    /// Index at the peak, increments of cruise left, index at the end.
    peak: u32,
    cruise: f32,
    last: u32,
    /// Increments completed.
    k: f32,
    /// Length of the current increment, as a fraction of one, its time, the
    /// time spent in it so far, and its acceleration.
    len: f32,
    dur: f32,
    into: f32,
    acc: f32,
    /// End position, kept exact, end speed and highest speed, mm/s.
    end: Length,
    v_end: f32,
    peak_v: f32,
}

impl FastPowerPlan {
    /// A plan that brakes from Austin index `m0` to `m_end`, then continues
    /// on the curve from index `nx`.
    #[allow(clippy::too_many_arguments)]
    fn new(
        st: Steps,
        s0: f32,
        m0: u32,
        m_end: u32,
        nx: u32,
        cruise: f32,
        last: u32,
        end: Length,
        v_end: f32,
        peak_v: f32,
    ) -> Self {
        let c = start_delay(m0, st.shape.a, st.res);
        Self::build(
            st,
            s0,
            FastPhase::Brake,
            m0,
            m_end,
            nx,
            c,
            nx,
            cruise,
            last,
            end,
            v_end,
            peak_v,
        )
    }

    /// A plan that starts on the curve at index `n0`.
    #[allow(clippy::too_many_arguments)]
    fn start(
        st: Steps,
        s0: f32,
        n0: u32,
        peak: u32,
        cruise: f32,
        last: u32,
        end: Length,
        v_end: f32,
        peak_v: f32,
    ) -> Self {
        let c = st.start_delay(n0);
        Self::build(
            st,
            s0,
            FastPhase::Ramp,
            0,
            0,
            n0,
            c,
            peak,
            cruise,
            last,
            end,
            v_end,
            peak_v,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn build(
        st: Steps,
        s0: f32,
        phase: FastPhase,
        m: u32,
        m_end: u32,
        n: u32,
        c: f32,
        peak: u32,
        cruise: f32,
        last: u32,
        end: Length,
        v_end: f32,
        peak_v: f32,
    ) -> Self {
        let mut plan = Self {
            st,
            s0,
            phase,
            m,
            m_end,
            n,
            c,
            peak,
            cruise,
            last,
            k: 0.0,
            len: 0.0,
            dur: 0.0,
            into: 0.0,
            acc: 0.0,
            end,
            v_end,
            peak_v,
        };
        plan.begin();
        plan
    }

    /// Sets up the next increment, moving through phases with nothing left.
    fn begin(&mut self) {
        loop {
            match self.phase {
                FastPhase::Brake if self.m > self.m_end => {
                    let m = self.m as f32;
                    self.c *= (4.0 * m + 1.0) / (4.0 * m - 1.0);
                    self.m -= 1;
                    self.set(1.0, -self.st.shape.a);
                    return;
                }
                FastPhase::Brake => self.phase = FastPhase::Ramp,
                FastPhase::Ramp if self.n < self.peak => {
                    let a = self.curve_accel(self.n);
                    self.set(1.0, a);
                    return;
                }
                FastPhase::Ramp => self.phase = FastPhase::Cruise,
                // Below a thousandth of an increment, the rest is rounding.
                FastPhase::Cruise if self.cruise > 1e-3 => {
                    self.set(self.cruise.min(1.0), 0.0);
                    return;
                }
                FastPhase::Cruise => self.phase = FastPhase::Down,
                FastPhase::Down if self.n > self.last => {
                    let t = self.st.term(self.n);
                    self.c *= (t + 1.0) / (t - 1.0);
                    self.n -= 1;
                    let a = -self.curve_accel(self.n);
                    self.set(1.0, a);
                    return;
                }
                FastPhase::Down => self.phase = FastPhase::Done,
                FastPhase::Done => return,
            }
        }
    }

    fn set(&mut self, len: f32, acc: f32) {
        self.len = len;
        self.dur = len * self.c;
        self.acc = acc;
    }

    /// Acceleration of the curve over increment `n`.
    fn curve_accel(&self, n: u32) -> f32 {
        self.st.shape.accel(self.st.sigma(n as f32 + 0.5))
    }

    /// Completes the current increment.
    fn finish(&mut self) {
        self.k += self.len;
        match self.phase {
            FastPhase::Ramp => {
                self.n += 1;
                self.c -= 2.0 * self.c / (self.st.term(self.n) + 1.0);
            }
            FastPhase::Cruise => self.cruise -= self.len,
            _ => {}
        }
    }

    /// Highest speed reached during the plan.
    ///
    /// ```
    /// use capek_core::motion::profile::{FastPower, Profile, State};
    /// use capek_core::units::UnitExt;
    ///
    /// let profile = FastPower::new(400.mm_per_s(), 1200.mm_per_s2(), 3.0);
    /// let plan = profile.plan(State::REST, 1.m(), 0.mm_per_s());
    /// assert!((plan.peak_speed() - 400.mm_per_s()).abs() < 1.mm_per_s());
    /// ```
    pub fn peak_speed(&self) -> Speed {
        Speed::mm_per_s(self.peak_v)
    }

    /// The state at the current time.
    fn state(&self) -> State {
        if self.phase == FastPhase::Done {
            return State {
                s: self.end,
                v: Speed::mm_per_s(self.v_end),
                a: Acceleration::ZERO,
            };
        }
        let k = self.k + self.len * self.into / self.dur;
        State {
            s: Length::mm(self.s0 + k * self.st.res),
            v: Speed::mm_per_s(self.st.res / self.c),
            a: Acceleration::mm_per_s2(self.acc),
        }
    }
}

impl Plan for FastPowerPlan {
    fn advance(&mut self, dt: Duration) -> State {
        let mut left = dt.as_secs_f32();
        while self.phase != FastPhase::Done {
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
        self.phase == FastPhase::Done
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::profile::Trapezoid;
    use crate::units::UnitExt;

    const DT: Duration = Duration::from_millis(1);

    fn profile(p: f32) -> Power {
        Power {
            max_speed: 400.mm_per_s(),
            accel: 1200.mm_per_s2(),
            p,
        }
    }

    /// Runs a plan to the end in 1 ms slices, checking the limits on the way.
    fn run(mut plan: PowerPlan, from: State) -> (State, f32) {
        let vmax = 400.0_f32.max(from.v.as_mm_per_s()) + 1e-2;
        let dv_max = 1200.0 * DT.as_secs_f32() + 1e-2;
        let mut prev = from;
        let mut t = 0.0;
        while !plan.is_done() {
            let s = plan.advance(DT);
            t += DT.as_secs_f32();
            assert!(s.v.as_mm_per_s() <= vmax, "speed limit at {t}: {s:?}");
            assert!(s.v.as_mm_per_s() >= -1e-3, "negative speed at {t}: {s:?}");
            assert!(s.a.abs() <= 1200.mm_per_s2() + 1e-2.mm_per_s2());
            let dv = (s.v - prev.v).abs().as_mm_per_s();
            assert!(dv <= dv_max, "speed jump {dv} at {t}");
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
    fn follows_the_curve() {
        for p in [1.5, 2.0, 3.0, 5.0] {
            let pr = profile(p);
            let sh = pr.shape();
            let mut plan = pr.plan(State::REST, 2.m(), 0.mm_per_s());
            let ramp_end = sh.sigma_end * sh.sigma_end * sh.len;
            loop {
                let s = plan.advance(DT);
                if s.s.as_mm() >= ramp_end {
                    break;
                }
                let u = s.s.as_mm() / sh.len;
                let q = p - 1.0;
                let v = sh.vinf * libm::sqrtf(u / libm::powf(1.0 + libm::powf(u, q), 1.0 / q));
                assert!(close(s.v.as_mm_per_s(), v, 1e-2), "p = {p}: {s:?} vs {v}");
            }
        }
    }

    #[test]
    fn starts_at_full_acceleration_and_reaches_max_speed() {
        let p = profile(3.0);
        let mut plan = p.plan(State::REST, 2.m(), 0.mm_per_s());
        let first = plan.advance(Duration::from_micros(10));
        assert!(close(first.a.as_mm_per_s2(), 1200.0, 0.1));
        let (end, _) = run(plan, first);
        assert_eq!(end, State::at(2.m()));

        let mut plan = p.plan(State::REST, 2.m(), 0.mm_per_s());
        let mid = plan.advance(Duration::from_secs(2));
        assert!(close(mid.v.as_mm_per_s(), 400.0, 1e-3));
        assert_eq!(mid.a, 0.mm_per_s2());
    }

    #[test]
    fn matches_ninja_asymptote() {
        // Ninja ends its ramp at u = (p + 2) / 2, where p = 3 gives 0.9636·v∞.
        let sh = profile(3.0).shape();
        assert!(close(sh.vmax / sh.vinf, 0.9636, 1e-4));
        // For p = 2 the ramp ends at u = 2, where v = √(2/3)·v∞.
        let sh = profile(2.0).shape();
        assert!(close(sh.vmax / sh.vinf, libm::sqrtf(2.0 / 3.0), 1e-5));
    }

    #[test]
    fn duration_matches_advancing() {
        for d in [2.m(), 100.mm(), 5.mm()] {
            let plan = profile(3.0).plan(State::REST, d, 0.mm_per_s());
            let (end, t) = run(plan, State::REST);
            assert_eq!(end, State::at(d));
            assert!(close(t, plan.duration().as_secs_f32(), 1.5e-3));
        }
    }

    #[test]
    fn slower_than_trapezoid() {
        let trap = Trapezoid {
            max_speed: 400.mm_per_s(),
            accel: 1200.mm_per_s2(),
        };
        let t = trap.plan(State::REST, 1.m(), 0.mm_per_s()).duration();
        let p = profile(3.0)
            .plan(State::REST, 1.m(), 0.mm_per_s())
            .duration();
        assert!(p > t);
    }

    #[test]
    fn short_move_never_reaches_cruise() {
        let plan = profile(3.0).plan(State::REST, 30.mm(), 0.mm_per_s());
        assert!(plan.peak_speed() < 400.mm_per_s());
        let (end, _) = run(plan, State::REST);
        assert_eq!(end, State::at(30.mm()));
    }

    #[test]
    fn plans_between_moving_states() {
        let p = profile(3.0);
        let from = State {
            s: 100.mm(),
            v: 200.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        let mut plan = p.plan(from, 500.mm(), 100.mm_per_s());
        let first = plan.advance(DT);
        assert!(close(first.v.as_mm_per_s(), 200.0, 1.5));
        let (end, _) = run(plan, first);
        assert_eq!(end.s, 600.mm());
        assert!(close(end.v.as_mm_per_s(), 100.0, 1e-2));
    }

    #[test]
    fn slows_down_from_above_cruise() {
        let p = profile(3.0);
        let from = State {
            s: 0.mm(),
            v: 600.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        // Long enough to brake to cruise, and too short to.
        for d in [1.m(), 200.mm(), 60.mm()] {
            let mut plan = p.plan(from, d, 0.mm_per_s());
            let first = plan.advance(DT);
            assert!(close(first.a.as_mm_per_s2(), -1200.0, 1e-3));
            let (end, _) = run(plan, first);
            assert_eq!(end.s, d);
        }
    }

    #[test]
    fn unreachable_end_speed_never_overshoots() {
        let p = profile(3.0);
        let from = State {
            s: 0.mm(),
            v: 400.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        let (end, _) = run(p.plan(from, 20.mm(), 0.mm_per_s()), from);
        assert_eq!(end.s, 20.mm());
        assert!(end.v > 300.mm_per_s());

        let (end, _) = run(p.plan(State::REST, 10.mm(), 400.mm_per_s()), State::REST);
        assert_eq!(end.s, 10.mm());
        // The start of the curve is nearly constant acceleration.
        assert!(close(end.v.as_mm_per_s(), 154.919, 2.0));
    }

    #[test]
    fn stop_from_every_phase() {
        let p = profile(3.0);
        let sh = p.shape();
        let mut plan = p.plan(State::REST, 1.m(), 0.mm_per_s());
        while !plan.is_done() {
            let state = plan.advance(Duration::from_millis(5));
            let sigma = sh.sigma_at(state.v.as_mm_per_s());
            let expected = sh.distance(sigma, 0.0);
            let (end, _) = run(p.stop(state), state);
            assert_eq!(end.v, 0.mm_per_s());
            assert!(close((end.s - state.s).as_mm(), expected, 1e-3));
        }

        // From above cruise.
        let fast = State {
            s: 0.mm(),
            v: 500.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        let (end, _) = run(p.stop(fast), fast);
        assert_eq!(end.v, 0.mm_per_s());
    }

    #[test]
    fn zero_distance_at_rest_is_done() {
        let p = profile(3.0);
        let mut plan = p.plan(State::at(5.mm()), 0.mm(), 0.mm_per_s());
        assert!(plan.is_done());
        assert_eq!(plan.advance(DT), State::at(5.mm()));
        assert!(p.stop(State::at(5.mm())).is_done());
    }

    #[test]
    fn negative_inputs_are_zero() {
        let plan = profile(3.0).plan(State::REST, (-10).mm(), (-5).mm_per_s());
        assert!(plan.is_done());
    }

    #[test]
    fn coarse_steps_match_fine_steps() {
        let mut fine = profile(3.0).plan(State::REST, 500.mm(), 0.mm_per_s());
        let mut coarse = fine;
        for i in 1..=600 {
            let f = fine.advance(DT);
            if i % 50 == 0 {
                let c = coarse.advance(Duration::from_millis(50));
                assert!(close(c.s.as_mm(), f.s.as_mm(), 1e-2), "{c:?} vs {f:?}");
                assert!(close(c.v.as_mm_per_s(), f.v.as_mm_per_s(), 1e-2));
            }
        }
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

    fn profile(p: f32) -> FastPower {
        let mut f = FastPower::new(400.mm_per_s(), 1200.mm_per_s2(), p);
        f.set_resolution(ninja_resolution().mm());
        f
    }

    fn exact(p: f32) -> Power {
        Power {
            max_speed: 400.mm_per_s(),
            accel: 1200.mm_per_s2(),
            p,
        }
    }

    /// Runs a plan to the end in 1 ms slices, checking the limits on the way.
    fn run(mut plan: FastPowerPlan, from: State) -> State {
        let vmax = 1.01 * 400.0_f32.max(from.v.as_mm_per_s());
        let mut prev = from;
        let mut steps = 0;
        while !plan.is_done() {
            let s = plan.advance(DT);
            steps += 1;
            assert!(s.v.as_mm_per_s() <= vmax, "speed limit at {steps}: {s:?}");
            assert!(s.v.as_mm_per_s() >= 0.0, "negative speed at {steps}");
            assert!(s.a.abs() <= 1200.mm_per_s2() + 1e-2.mm_per_s2());
            assert!(s.s >= prev.s - 1e-4.mm(), "went backwards at {steps}");
            prev = s;
            assert!(steps < 60_000, "plan never finished");
        }
        prev
    }

    /// The time of every increment of a plan, in seconds.
    fn delays(mut plan: FastPowerPlan) -> Vec<f32> {
        let mut out = Vec::new();
        while plan.phase != FastPhase::Done {
            out.push(plan.dur);
            plan.finish();
            plan.begin();
        }
        out
    }

    /// Ninja's `PowerProfile::delays`, in timer ticks at 4 MHz, in the linear
    /// units capek uses, with its default limits: v∞ = 4000 mm/s,
    /// 12000 mm/s². Kept in `f64` as ninja has it.
    fn ninja_delays(steps: u32, p: f64) -> Vec<u32> {
        let res = ninja_resolution() as f64;
        let (vinf, accel) = (4000.0_f64, 12000.0_f64);
        let f = 4_000_000.0_f64;
        let c0 = f * libm::sqrt(2.0 * res / accel);
        let max_n = (vinf * vinf) / (2.0 * accel * res) * (p + 2.0) / 2.0;
        let accel_steps = core::cmp::min(libm::floor(max_n) as u32, steps / 2);
        let coast_steps = steps.saturating_sub(2 * accel_steps);

        let mut vec = Vec::new();
        let mut cn = 0.676 * c0;
        let mut n = 0.0;
        let k = 4.0 * libm::pow((2.0 * res * accel) / (vinf * vinf), p - 1.0);
        for _ in 0..accel_steps {
            vec.push(cn as u32);
            n += 1.0;
            cn = cn - (2.0 * cn) / (4.0 * n + 1.0 + k * libm::pow(n, p));
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
        for p in [2.0_f32, 3.0] {
            let max_speed = Power::max_speed_for(4000.mm_per_s(), p);
            let mut profile = FastPower::new(max_speed, 12000.mm_per_s2(), p);
            profile.set_resolution(res.mm());
            // Long, triangular, odd and tiny moves.
            for steps in [10000, 3000, 101, 7, 1] {
                let plan = profile.plan(State::REST, (steps as f32 * res).mm(), 0.mm_per_s());
                let ours = delays(plan);
                let ninja = ninja_delays(steps, p as f64);
                assert_eq!(ours.len(), ninja.len(), "p = {p}, {steps} steps");
                for (i, (c, n)) in ours.iter().zip(ninja.iter()).enumerate() {
                    let c = c * 4_000_000.0;
                    let n = *n as f32;
                    assert!(
                        (c - n).abs() <= 1.0 + 1e-4 * n,
                        "p = {p}, {steps} steps, delay {i}: {c} vs ninja {n}"
                    );
                }
            }
        }
    }

    #[test]
    fn follows_the_exact_curve() {
        for p in [2.0, 3.0] {
            for distance in [1.m(), 100.mm(), 20.mm()] {
                let fast = profile(p).plan(State::REST, distance, 0.mm_per_s());
                let mut slow = exact(p).plan(State::REST, distance, 0.mm_per_s());
                let total = slow.duration().as_secs_f32();
                assert_eq!(run(fast, State::REST), State::at(distance));

                let mut fast = fast;
                let mut t = 0.0;
                while !slow.is_done() || !fast.is_done() {
                    let f = fast.advance(DT);
                    let s = slow.advance(DT);
                    t += DT.as_secs_f32();
                    // As for `FastTrapezoid`: within 2 % plus the lead of
                    // Austin's shortened first increment, and the speed of
                    // one increment at the start.
                    assert!(
                        (f.s - s.s).abs() < distance * 0.02 + 4.mm(),
                        "p = {p}, {t}: {f:?} vs {s:?}"
                    );
                    assert!(
                        (f.v - s.v).abs() < slow.peak_speed() * 0.05 + 30.mm_per_s(),
                        "p = {p}, {t}: {f:?} vs {s:?}"
                    );
                }
                assert!(
                    (t - total).abs() < total * 0.02,
                    "took {t} s, not {total} s"
                );
            }
        }
    }

    #[test]
    fn plans_between_moving_states() {
        let from = State {
            s: 100.mm(),
            v: 200.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        let mut plan = profile(3.0).plan(from, 500.mm(), 100.mm_per_s());
        let first = plan.advance(DT);
        assert!((first.v - 200.mm_per_s()).abs() < 5.mm_per_s());
        let end = run(plan, first);
        assert_eq!(end.s, 600.mm());
        // Speeds are whole increment indices: a few mm/s apart at 100 mm/s.
        assert!((end.v - 100.mm_per_s()).abs() < 5.mm_per_s());
    }

    #[test]
    fn slows_down_from_above_cruise() {
        let from = State {
            s: 0.mm(),
            v: 600.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        // Long enough to brake to cruise, too short to, and too short to
        // brake to the end speed at all.
        for d in [1.m(), 200.mm(), 60.mm(), 20.mm()] {
            let mut plan = profile(3.0).plan(from, d, 0.mm_per_s());
            let first = plan.advance(DT);
            assert_eq!(first.a, (-1200).mm_per_s2());
            assert!(first.v < 600.mm_per_s());
            let end = run(plan, first);
            assert_eq!(end.s, d);
        }
    }

    #[test]
    fn unreachable_end_speed_never_overshoots() {
        let from = State {
            s: 0.mm(),
            v: 400.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        let end = run(profile(3.0).plan(from, 20.mm(), 0.mm_per_s()), from);
        assert_eq!(end.s, 20.mm());
        assert!(end.v > 300.mm_per_s());

        let end = run(
            profile(3.0).plan(State::REST, 10.mm(), 400.mm_per_s()),
            State::REST,
        );
        assert_eq!(end.s, 10.mm());
        assert!((end.v.as_mm_per_s() - 154.919).abs() < 5.0);
    }

    #[test]
    fn stop_from_every_phase() {
        let p = profile(3.0);
        let sh = exact(3.0).shape();
        let mut plan = p.plan(State::REST, 1.m(), 0.mm_per_s());
        while !plan.is_done() {
            let state = plan.advance(DT);
            let expected = sh.distance(sh.sigma_at(state.v.as_mm_per_s()), 0.0);
            let end = run(p.stop(state), state);
            assert_eq!(end.v, 0.mm_per_s());
            let travelled = (end.s - state.s).as_mm();
            assert!(
                (travelled - expected).abs() < 0.05 * expected + 2.0 * ninja_resolution(),
                "stopped in {travelled} mm, expected {expected} mm"
            );
        }

        let fast = State {
            s: 0.mm(),
            v: 500.mm_per_s(),
            a: 0.mm_per_s2(),
        };
        let end = run(p.stop(fast), fast);
        assert_eq!(end.v, 0.mm_per_s());
    }

    #[test]
    fn partial_increment_ends_exactly() {
        let distance = (10.5 * ninja_resolution()).mm();
        let plan = profile(3.0).plan(State::REST, distance, 0.mm_per_s());
        assert_eq!(delays(plan).len(), 11);
        assert_eq!(run(plan, State::REST), State::at(distance));
    }

    #[test]
    fn zero_distance_at_rest_is_done() {
        let p = profile(3.0);
        let mut plan = p.plan(State::at(5.mm()), 0.mm(), 0.mm_per_s());
        assert!(plan.is_done());
        assert_eq!(plan.advance(DT), State::at(5.mm()));
        assert!(p.stop(State::at(5.mm())).is_done());
    }

    #[test]
    fn negative_inputs_are_zero() {
        assert!(
            profile(3.0)
                .plan(State::REST, (-10).mm(), (-5).mm_per_s())
                .is_done()
        );
    }

    #[test]
    fn coarse_steps_match_fine_steps() {
        let mut fine = profile(3.0).plan(State::REST, 500.mm(), 0.mm_per_s());
        let mut coarse = fine;
        for i in 1..=600 {
            let f = fine.advance(DT);
            if i % 50 == 0 {
                let c = coarse.advance(Duration::from_millis(50));
                assert!((c.s - f.s).abs() < 1e-2.mm());
                assert_eq!(c.v, f.v);
            }
        }
    }
}

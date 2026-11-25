//! PID controller.
//!
//! [`Pid`] turns the difference between where you want to be (the setpoint)
//! and where you are (the measurement) into an actuator command. It includes
//! the features a real control loop needs:
//!
//! - **Output limits** to keep the command within what the actuator accepts.
//! - **Anti-windup**: the integral stops growing while the output is
//!   saturated, so the loop does not overshoot after a long saturation.
//! - **Derivative on measurement**: the derivative term reacts to how the
//!   measurement moves, not to the error, so a setpoint change gives no
//!   "derivative kick".
//! - **Derivative filtering**: an optional first-order low-pass on the
//!   derivative term, which otherwise amplifies sensor noise.
//!
//! Setpoint and measurement can be any [`Quantity`]: `f32`,
//! [`Length`](crate::units::Length) or [`Angle`](crate::units::Angle). Errors
//! are computed in the quantity's storage unit (millimetres for lengths,
//! radians for angles), so choose gains for that unit. The output is a plain
//! `f32` in whatever unit the actuator takes: a duty cycle, a speed or a step
//! rate.
//!
//! # Examples
//!
//! ```
//! use core::time::Duration;
//! use capek_core::pid::Pid;
//! use capek_core::units::UnitExt;
//!
//! // Hold an arm at 90°, commanding a PWM duty in -1.0..=1.0.
//! let mut pid = Pid::new(2.0, 0.5, 0.05).with_output_limits(-1.0, 1.0);
//! let dt = Duration::from_millis(10);
//!
//! let duty = pid.update(90.deg(), 80.deg(), dt);
//! assert!(duty > 0.0); // below target, push up
//! ```

use core::time::Duration;

use crate::units::Quantity;

/// A PID controller with output limits, anti-windup, derivative on
/// measurement and optional derivative filtering.
///
/// ```
/// use core::time::Duration;
/// use capek_core::pid::Pid;
///
/// let mut pid = Pid::new(1.0, 0.0, 0.0);
/// // Pure proportional: output is gain times error.
/// assert_eq!(pid.update(10.0_f32, 4.0_f32, Duration::from_millis(1)), 6.0);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    out_min: f32,
    out_max: f32,
    d_alpha: f32,
    integral: f32,
    d_filtered: f32,
    last_measurement: Option<f32>,
}

impl Pid {
    /// Creates a controller with proportional, integral and derivative gains.
    /// The output is unlimited and the derivative is unfiltered until you set
    /// them.
    ///
    /// ```
    /// use capek_core::pid::Pid;
    /// let pid = Pid::new(1.2, 0.3, 0.01);
    /// assert_eq!(pid.gains(), (1.2, 0.3, 0.01));
    /// ```
    pub const fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            out_min: f32::NEG_INFINITY,
            out_max: f32::INFINITY,
            d_alpha: 1.0,
            integral: 0.0,
            d_filtered: 0.0,
            last_measurement: None,
        }
    }

    /// Clamps the output to `min..=max`. The bounds are swapped if given in
    /// the wrong order.
    ///
    /// ```
    /// use core::time::Duration;
    /// use capek_core::pid::Pid;
    ///
    /// let mut pid = Pid::new(100.0, 0.0, 0.0).with_output_limits(-1.0, 1.0);
    /// assert_eq!(pid.update(10.0_f32, 0.0_f32, Duration::from_millis(1)), 1.0);
    /// ```
    pub const fn with_output_limits(mut self, min: f32, max: f32) -> Self {
        if min <= max {
            self.out_min = min;
            self.out_max = max;
        } else {
            self.out_min = max;
            self.out_max = min;
        }
        self
    }

    /// Low-pass filters the derivative term with smoothing factor `alpha`
    /// (see [`Ema`](crate::filter::Ema)). `1.0` means no filtering; smaller
    /// values smooth more. Clamped to `0.0..=1.0`.
    ///
    /// ```
    /// use capek_core::pid::Pid;
    /// let pid = Pid::new(1.0, 0.0, 0.2).with_derivative_filter(0.3);
    /// ```
    pub const fn with_derivative_filter(mut self, alpha: f32) -> Self {
        self.d_alpha = alpha.clamp(0.0, 1.0);
        self
    }

    /// Replaces the gains, for tuning at runtime. The integral is kept, so the
    /// output does not jump.
    ///
    /// ```
    /// use capek_core::pid::Pid;
    ///
    /// let mut pid = Pid::new(1.0, 0.0, 0.0);
    /// pid.set_gains(2.0, 0.1, 0.0);
    /// assert_eq!(pid.gains(), (2.0, 0.1, 0.0));
    /// ```
    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    /// The current `(kp, ki, kd)`.
    ///
    /// ```
    /// use capek_core::pid::Pid;
    /// assert_eq!(Pid::new(1.0, 2.0, 3.0).gains(), (1.0, 2.0, 3.0));
    /// ```
    pub const fn gains(&self) -> (f32, f32, f32) {
        (self.kp, self.ki, self.kd)
    }

    /// Runs one control step and returns the command.
    ///
    /// `dt` is the time since the previous call. A zero `dt` skips the
    /// integral and derivative terms for that step.
    ///
    /// ```
    /// use core::time::Duration;
    /// use capek_core::pid::Pid;
    /// use capek_core::units::UnitExt;
    ///
    /// let mut pid = Pid::new(0.01, 0.0, 0.0);
    /// // 100 mm short of target, 0.01 per mm.
    /// let cmd = pid.update(500.mm(), 400.mm(), Duration::from_millis(20));
    /// assert!((cmd - 1.0).abs() < 1e-5);
    /// ```
    pub fn update<Q: Quantity>(&mut self, setpoint: Q, measurement: Q, dt: Duration) -> f32 {
        let sp = setpoint.to_f32();
        let pv = measurement.to_f32();
        let dt = dt.as_secs_f32();
        let error = sp - pv;

        let p = self.kp * error;

        let d = match (self.last_measurement, dt > 0.0) {
            (Some(last), true) => {
                // Derivative on measurement: -kd * d(pv)/dt.
                let raw = -self.kd * (pv - last) / dt;
                self.d_filtered += self.d_alpha * (raw - self.d_filtered);
                self.d_filtered
            }
            _ => 0.0,
        };
        self.last_measurement = Some(pv);

        // Anti-windup by conditional integration: only accept the new integral
        // if it keeps the output inside the limits, or moves it back toward
        // them.
        let candidate = self.integral + self.ki * error * dt;
        let unclamped = p + candidate + d;
        let saturated_high = unclamped > self.out_max && error > 0.0;
        let saturated_low = unclamped < self.out_min && error < 0.0;
        if !(saturated_high || saturated_low) {
            self.integral = candidate;
        }

        (p + self.integral + d).clamp(self.out_min, self.out_max)
    }

    /// Clears the integral and derivative history. Call it when the loop has
    /// been inactive, for example after re-enabling a motor.
    ///
    /// ```
    /// use core::time::Duration;
    /// use capek_core::pid::Pid;
    ///
    /// let dt = Duration::from_millis(10);
    /// let mut pid = Pid::new(0.0, 10.0, 0.0);
    /// pid.update(1.0_f32, 0.0_f32, dt);
    /// pid.reset();
    /// assert_eq!(pid.update(0.0_f32, 0.0_f32, dt), 0.0);
    /// ```
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.d_filtered = 0.0;
        self.last_measurement = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: Duration = Duration::from_millis(10);

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-4
    }

    #[test]
    fn integral_accumulates_over_time() {
        let mut pid = Pid::new(0.0, 2.0, 0.0);
        // error 1, ki 2, dt 0.01 -> +0.02 per step
        for i in 1..=5 {
            let out = pid.update(1.0_f32, 0.0_f32, DT);
            assert!(close(out, 0.02 * i as f32), "step {i}: {out}");
        }
    }

    #[test]
    fn derivative_opposes_measurement_motion() {
        let mut pid = Pid::new(0.0, 0.0, 1.0);
        assert_eq!(pid.update(0.0_f32, 0.0_f32, DT), 0.0); // no history yet
        // Measurement rose 0.1 in 10 ms -> rate 10, kd 1 -> -10
        assert!(close(pid.update(0.0_f32, 0.1_f32, DT), -10.0));
    }

    #[test]
    fn no_derivative_kick_on_setpoint_change() {
        let mut pid = Pid::new(0.0, 0.0, 1.0);
        pid.update(0.0_f32, 5.0_f32, DT);
        // Setpoint jumps, measurement does not move.
        assert_eq!(pid.update(100.0_f32, 5.0_f32, DT), 0.0);
    }

    #[test]
    fn derivative_filter_smooths() {
        let mut raw = Pid::new(0.0, 0.0, 1.0);
        let mut filtered = Pid::new(0.0, 0.0, 1.0).with_derivative_filter(0.2);
        raw.update(0.0_f32, 0.0_f32, DT);
        filtered.update(0.0_f32, 0.0_f32, DT);
        let r = raw.update(0.0_f32, 0.1_f32, DT);
        let f = filtered.update(0.0_f32, 0.1_f32, DT);
        assert!(close(f, 0.2 * r));
    }

    #[test]
    fn anti_windup_stops_integral_growth_while_saturated() {
        let mut pid = Pid::new(1.0, 10.0, 0.0).with_output_limits(-1.0, 1.0);
        // Large error for a long time: output pinned at the limit.
        for _ in 0..1000 {
            assert_eq!(pid.update(10.0_f32, 0.0_f32, DT), 1.0);
        }
        // Once the target is reached the output drops right away instead of
        // staying saturated while a huge integral unwinds.
        let out = pid.update(0.0_f32, 0.0_f32, DT);
        assert!(out < 1.0, "output {out}");
    }

    #[test]
    fn closes_the_loop_on_a_simple_plant() {
        // Plant: position integrates the command (a velocity), with a
        // constant disturbance the integral term must cancel.
        let mut pid = Pid::new(4.0, 2.0, 0.1).with_output_limits(-50.0, 50.0);
        let mut pos = 0.0_f32;
        let dt = DT.as_secs_f32();
        for _ in 0..2000 {
            let v = pid.update(100.0_f32, pos, DT);
            pos += (v - 3.0) * dt;
        }
        assert!((pos - 100.0).abs() < 0.1, "position {pos}");
    }

    #[test]
    fn zero_dt_is_proportional_only() {
        let mut pid = Pid::new(1.0, 100.0, 100.0);
        pid.update(0.0_f32, 0.0_f32, DT);
        assert_eq!(pid.update(3.0_f32, 1.0_f32, Duration::ZERO), 2.0);
    }

    #[test]
    fn swapped_limits_are_normalised() {
        let mut pid = Pid::new(100.0, 0.0, 0.0).with_output_limits(1.0, -1.0);
        assert_eq!(pid.update(1.0_f32, 0.0_f32, DT), 1.0);
        assert_eq!(pid.update(-1.0_f32, 0.0_f32, DT), -1.0);
    }
}

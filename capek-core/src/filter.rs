//! Signal filters for smoothing sensor readings.
//!
//! Filters are plain structs with no async code and no hardware access. Feed
//! them one sample at a time with [`Filter::update`]. They work on any
//! [`Quantity`]: plain `f32`, [`Length`](crate::units::Length) or
//! [`Angle`](crate::units::Angle).
//!
//! | Filter | What it does | Typical use |
//! |---|---|---|
//! | [`Ema`] | First-order low-pass (exponential moving average). | General smoothing; cheap and tunable by cutoff frequency. |
//! | [`MovingAverage`] | Mean of the last `N` samples. | Averaging out white noise with a fixed window. |
//! | [`Median`] | Median of the last `N` samples. | Removing isolated spikes, such as stray ultrasonic echoes. |
//! | [`Biquad`] | Second-order IIR: low-pass, high-pass or notch. | Sharper roll-off than [`Ema`]; removing drift or a vibration frequency. |
//! | [`Kalman`] | Scalar Kalman filter for a slowly changing value. | Fusing noisy readings when you know the noise levels. |
//! | [`RateLimit`] | Caps the change per sample. | Slew-limiting setpoints or rejecting physically impossible jumps. |
//!
//! Filters chain with [`Filter::then`]: the output of the first is the input
//! of the second. All filters let the first sample through so the output
//! starts at the signal instead of ramping up from zero.
//!
//! # Examples
//!
//! ```
//! use capek_core::filter::{Ema, Filter, Median};
//! use capek_core::units::UnitExt;
//!
//! // Drop spikes first, then smooth.
//! let mut f = Median::<3>::new().then(Ema::new(0.5));
//!
//! let readings = [100.mm(), 102.mm(), 900.mm(), 101.mm()];
//! let mut last = 0.mm();
//! for r in readings {
//!     last = f.update(r);
//! }
//! // The 900 mm spike never reached the output.
//! assert!(last < 110.mm());
//! ```

use core::f32::consts::{FRAC_1_SQRT_2, PI};

use crate::units::Quantity;

/// A filter that turns a stream of samples into a smoothed stream.
///
/// Implement it to plug your own filter in anywhere a built-in one is
/// accepted.
///
/// ```
/// use capek_core::filter::Filter;
/// use capek_core::units::Quantity;
///
/// /// Passes samples through unchanged, but never below zero.
/// struct ClampNegative;
///
/// impl Filter for ClampNegative {
///     fn update<Q: Quantity>(&mut self, x: Q) -> Q {
///         Q::from_f32(x.to_f32().max(0.0))
///     }
///     fn reset(&mut self) {}
/// }
///
/// assert_eq!(ClampNegative.update(-3.0_f32), 0.0);
/// ```
pub trait Filter {
    /// Feeds one sample and returns the filtered value.
    ///
    /// Generic over the quantity so one filter type serves `f32`, `Length`
    /// and `Angle` alike. Feed a given filter instance a single quantity type;
    /// mixing types in one instance mixes their storage units.
    fn update<Q: Quantity>(&mut self, x: Q) -> Q;

    /// Forgets all history. The next sample starts fresh.
    fn reset(&mut self);

    /// Chains `next` after this filter.
    ///
    /// ```
    /// use capek_core::filter::{Filter, RateLimit};
    ///
    /// let mut f = RateLimit::new(10.0_f32).then(RateLimit::new(5.0_f32));
    /// f.update(0.0_f32);
    /// // The tighter limit wins.
    /// assert_eq!(f.update(100.0), 5.0);
    /// ```
    fn then<F: Filter>(self, next: F) -> Chain<Self, F>
    where
        Self: Sized,
    {
        Chain {
            first: self,
            second: next,
        }
    }
}

/// Two filters applied one after the other. Built by [`Filter::then`].
///
/// ```
/// use capek_core::filter::{Filter, Kalman, Median};
/// use capek_core::units::UnitExt;
///
/// let mut f = Median::<5>::new().then(Kalman::new(0.1.mm(), 2.mm()));
/// let y = f.update(10.mm());
/// assert_eq!(y, 10.mm());
/// ```
#[derive(Debug, Clone)]
pub struct Chain<A, B> {
    first: A,
    second: B,
}

impl<A: Filter, B: Filter> Filter for Chain<A, B> {
    fn update<Q: Quantity>(&mut self, x: Q) -> Q {
        self.second.update(self.first.update(x))
    }

    fn reset(&mut self) {
        self.first.reset();
        self.second.reset();
    }
}

/// Exponential moving average, a first-order low-pass filter.
///
/// Each output is `alpha * sample + (1 - alpha) * previous`. A small `alpha`
/// smooths more but reacts more slowly. Build it from `alpha` directly with
/// [`Ema::new`], or from a cutoff frequency with [`Ema::from_cutoff`].
///
/// ```
/// use capek_core::filter::{Ema, Filter};
///
/// let mut ema = Ema::new(0.5);
/// assert_eq!(ema.update(10.0_f32), 10.0); // first sample passes through
/// assert_eq!(ema.update(20.0), 15.0);
/// assert_eq!(ema.update(20.0), 17.5);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Ema {
    alpha: f32,
    state: Option<f32>,
}

impl Ema {
    /// Creates a filter with smoothing factor `alpha`, clamped to `0.0..=1.0`.
    ///
    /// ```
    /// use capek_core::filter::Ema;
    ///
    /// assert_eq!(Ema::new(0.2).alpha(), 0.2);
    /// assert_eq!(Ema::new(3.0).alpha(), 1.0);
    /// ```
    pub const fn new(alpha: f32) -> Self {
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            state: None,
        }
    }

    /// Creates a low-pass filter with the given cutoff frequency, for samples
    /// arriving at `sample_hz`.
    ///
    /// Uses `alpha = 1 - exp(-2π · cutoff / sample_rate)`, which places the
    /// filter's pole exactly where the equivalent continuous RC filter has it.
    ///
    /// ```
    /// use capek_core::filter::Ema;
    ///
    /// // Ultrasonic sensor polled every 60 ms, keep changes slower than 2 Hz.
    /// let ema = Ema::from_cutoff(2.0, 1000.0 / 60.0);
    /// assert!(ema.alpha() > 0.5 && ema.alpha() < 0.6);
    /// ```
    pub fn from_cutoff(cutoff_hz: f32, sample_hz: f32) -> Self {
        Self::new(1.0 - libm::expf(-2.0 * PI * cutoff_hz / sample_hz))
    }

    /// The smoothing factor.
    ///
    /// ```
    /// use capek_core::filter::Ema;
    /// assert_eq!(Ema::new(0.3).alpha(), 0.3);
    /// ```
    pub const fn alpha(&self) -> f32 {
        self.alpha
    }
}

impl Filter for Ema {
    fn update<Q: Quantity>(&mut self, x: Q) -> Q {
        let x = x.to_f32();
        let y = match self.state {
            Some(prev) => self.alpha * x + (1.0 - self.alpha) * prev,
            None => x,
        };
        self.state = Some(y);
        Q::from_f32(y)
    }

    fn reset(&mut self) {
        self.state = None;
    }
}

/// Fixed-size ring buffer shared by the windowed filters.
#[derive(Debug, Clone, Copy)]
struct Window<const N: usize> {
    buf: [f32; N],
    len: usize,
    next: usize,
}

impl<const N: usize> Window<N> {
    const fn new() -> Self {
        assert!(N > 0, "filter window must hold at least one sample");
        Self {
            buf: [0.0; N],
            len: 0,
            next: 0,
        }
    }

    /// Adds a sample and returns the samples currently in the window, in no
    /// particular order.
    fn push(&mut self, x: f32) -> &[f32] {
        self.buf[self.next] = x;
        self.next = (self.next + 1) % N;
        self.len = (self.len + 1).min(N);
        &self.buf[..self.len]
    }

    fn clear(&mut self) {
        self.len = 0;
        self.next = 0;
    }
}

/// Mean of the last `N` samples, also called a boxcar filter.
///
/// Until `N` samples have arrived, it returns the mean of the samples so far.
///
/// ```
/// use capek_core::filter::{Filter, MovingAverage};
///
/// let mut avg = MovingAverage::<4>::new();
/// assert_eq!(avg.update(4.0_f32), 4.0);
/// assert_eq!(avg.update(8.0), 6.0);
/// avg.update(0.0);
/// avg.update(0.0);
/// assert_eq!(avg.update(4.0), 3.0); // oldest sample (4) dropped: (8+0+0+4)/4
/// ```
#[derive(Debug, Clone, Copy)]
pub struct MovingAverage<const N: usize> {
    window: Window<N>,
}

impl<const N: usize> MovingAverage<N> {
    /// Creates an empty filter. `N` must be at least 1.
    ///
    /// ```
    /// use capek_core::filter::MovingAverage;
    /// let avg = MovingAverage::<8>::new();
    /// ```
    pub const fn new() -> Self {
        Self {
            window: Window::new(),
        }
    }
}

impl<const N: usize> Default for MovingAverage<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Filter for MovingAverage<N> {
    fn update<Q: Quantity>(&mut self, x: Q) -> Q {
        // Summing the window each time avoids the drift a running sum
        // accumulates in f32. Windows are small, so this is cheap.
        let samples = self.window.push(x.to_f32());
        let sum: f32 = samples.iter().sum();
        Q::from_f32(sum / samples.len() as f32)
    }

    fn reset(&mut self) {
        self.window.clear();
    }
}

/// Median of the last `N` samples.
///
/// Removes isolated spikes completely instead of smearing them like an
/// average does. Until `N` samples have arrived, it returns the median of the
/// samples so far. For an even count it returns the lower of the two middle
/// values. Each update sorts a copy of the window, so keep `N` small.
///
/// ```
/// use capek_core::filter::{Filter, Median};
///
/// let mut m = Median::<3>::new();
/// m.update(5.0_f32);
/// m.update(6.0);
/// assert_eq!(m.update(1000.0), 6.0); // the spike is ignored
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Median<const N: usize> {
    window: Window<N>,
}

impl<const N: usize> Median<N> {
    /// Creates an empty filter. `N` must be at least 1.
    ///
    /// ```
    /// use capek_core::filter::Median;
    /// let m = Median::<5>::new();
    /// ```
    pub const fn new() -> Self {
        Self {
            window: Window::new(),
        }
    }
}

impl<const N: usize> Default for Median<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Filter for Median<N> {
    fn update<Q: Quantity>(&mut self, x: Q) -> Q {
        let samples = self.window.push(x.to_f32());
        let len = samples.len();
        let mut sorted = [0.0_f32; N];
        let sorted = &mut sorted[..len];
        sorted.copy_from_slice(samples);
        // Insertion sort: tiny, allocation-free and fast for small windows.
        for i in 1..len {
            let mut j = i;
            while j > 0 && sorted[j - 1] > sorted[j] {
                sorted.swap(j - 1, j);
                j -= 1;
            }
        }
        Q::from_f32(sorted[(len - 1) / 2])
    }

    fn reset(&mut self) {
        self.window.clear();
    }
}

/// Second-order IIR filter (biquad).
///
/// Build it with [`Biquad::low_pass`], [`Biquad::high_pass`] or
/// [`Biquad::notch`]. The coefficients follow Robert Bristow-Johnson's
/// *Audio EQ Cookbook*, and the filter runs in transposed direct form II.
/// With `q = 1/√2` the low-pass and high-pass are Butterworth filters, which
/// [`Biquad::butterworth_low_pass`] and [`Biquad::butterworth_high_pass`]
/// pick for you.
///
/// On the first sample the internal state is set as if that sample had been
/// present forever, so a low-pass starts at the signal instead of ramping up
/// from zero.
///
/// ```
/// use capek_core::filter::{Biquad, Filter};
///
/// // 100 Hz IMU, keep motion below 10 Hz.
/// let mut lp = Biquad::butterworth_low_pass(10.0, 100.0);
/// assert!((lp.update(1.0_f32) - 1.0).abs() < 1e-5);
/// // A steady input stays steady.
/// for _ in 0..10 {
///     assert!((lp.update(1.0_f32) - 1.0).abs() < 1e-4);
/// }
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
    primed: bool,
}

impl Biquad {
    /// Builds a filter from normalised coefficients (`a0` already divided
    /// out). Use this for coefficients computed elsewhere.
    ///
    /// ```
    /// use capek_core::filter::{Biquad, Filter};
    ///
    /// // b = [1, 0, 0], a = [1, 0, 0] passes the signal through.
    /// let mut identity = Biquad::from_coefficients(1.0, 0.0, 0.0, 0.0, 0.0);
    /// assert_eq!(identity.update(3.0_f32), 3.0);
    /// ```
    pub const fn from_coefficients(b0: f32, b1: f32, b2: f32, a1: f32, a2: f32) -> Self {
        Self {
            b0,
            b1,
            b2,
            a1,
            a2,
            z1: 0.0,
            z2: 0.0,
            primed: false,
        }
    }

    /// Shared cookbook terms: `(cos w0, alpha)`.
    fn terms(freq_hz: f32, q: f32, sample_hz: f32) -> (f32, f32) {
        let w0 = 2.0 * PI * freq_hz / sample_hz;
        (libm::cosf(w0), libm::sinf(w0) / (2.0 * q))
    }

    fn normalised(b0: f32, b1: f32, b2: f32, a0: f32, a1: f32, a2: f32) -> Self {
        Self::from_coefficients(b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)
    }

    /// Low-pass filter with corner frequency `cutoff_hz` and quality `q`.
    ///
    /// ```
    /// use capek_core::filter::Biquad;
    /// let lp = Biquad::low_pass(5.0, 0.707, 100.0);
    /// ```
    pub fn low_pass(cutoff_hz: f32, q: f32, sample_hz: f32) -> Self {
        let (cos, alpha) = Self::terms(cutoff_hz, q, sample_hz);
        let b1 = 1.0 - cos;
        Self::normalised(b1 / 2.0, b1, b1 / 2.0, 1.0 + alpha, -2.0 * cos, 1.0 - alpha)
    }

    /// High-pass filter with corner frequency `cutoff_hz` and quality `q`.
    /// Removes slow drift and constant offsets.
    ///
    /// ```
    /// use capek_core::filter::{Biquad, Filter};
    ///
    /// let mut hp = Biquad::high_pass(1.0, 0.707, 100.0);
    /// // A constant offset is removed.
    /// assert!(hp.update(5.0_f32).abs() < 1e-5);
    /// ```
    pub fn high_pass(cutoff_hz: f32, q: f32, sample_hz: f32) -> Self {
        let (cos, alpha) = Self::terms(cutoff_hz, q, sample_hz);
        let b0 = (1.0 + cos) / 2.0;
        Self::normalised(b0, -2.0 * b0, b0, 1.0 + alpha, -2.0 * cos, 1.0 - alpha)
    }

    /// Notch filter that removes `center_hz`. A higher `q` gives a narrower
    /// notch. Useful against a known vibration, such as motor frequency.
    ///
    /// ```
    /// use capek_core::filter::Biquad;
    /// let notch = Biquad::notch(25.0, 5.0, 200.0);
    /// ```
    pub fn notch(center_hz: f32, q: f32, sample_hz: f32) -> Self {
        let (cos, alpha) = Self::terms(center_hz, q, sample_hz);
        Self::normalised(1.0, -2.0 * cos, 1.0, 1.0 + alpha, -2.0 * cos, 1.0 - alpha)
    }

    /// Butterworth low-pass: maximally flat passband, `q = 1/√2`.
    ///
    /// ```
    /// use capek_core::filter::Biquad;
    /// let lp = Biquad::butterworth_low_pass(10.0, 100.0);
    /// ```
    pub fn butterworth_low_pass(cutoff_hz: f32, sample_hz: f32) -> Self {
        Self::low_pass(cutoff_hz, FRAC_1_SQRT_2, sample_hz)
    }

    /// Butterworth high-pass: maximally flat passband, `q = 1/√2`.
    ///
    /// ```
    /// use capek_core::filter::Biquad;
    /// let hp = Biquad::butterworth_high_pass(0.5, 100.0);
    /// ```
    pub fn butterworth_high_pass(cutoff_hz: f32, sample_hz: f32) -> Self {
        Self::high_pass(cutoff_hz, FRAC_1_SQRT_2, sample_hz)
    }

    /// Gain for a constant input.
    fn dc_gain(&self) -> f32 {
        let den = 1.0 + self.a1 + self.a2;
        if den == 0.0 {
            0.0
        } else {
            (self.b0 + self.b1 + self.b2) / den
        }
    }
}

impl Filter for Biquad {
    fn update<Q: Quantity>(&mut self, x: Q) -> Q {
        let x = x.to_f32();
        if !self.primed {
            // Steady state for a constant input x with output y = gain * x.
            let y = self.dc_gain() * x;
            self.z2 = self.b2 * x - self.a2 * y;
            self.z1 = self.b1 * x - self.a1 * y + self.z2;
            self.primed = true;
        }
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        Q::from_f32(y)
    }

    fn reset(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
        self.primed = false;
    }
}

/// Scalar Kalman filter for a value that changes slowly.
///
/// Models the true value as a random walk. Both noise levels are standard
/// deviations, given in the signal's own unit. `process_noise` is how much the
/// true value typically drifts between two samples. `measurement_noise` is how
/// much a single reading typically jitters. The ratio between them sets how
/// much the filter trusts new readings: a high measurement noise means
/// smoother output that reacts more slowly.
///
/// Unlike [`Ema`], the gain adapts. It starts high, so the estimate converges
/// quickly, then settles at the steady-state value.
///
/// ```
/// use capek_core::filter::{Filter, Kalman};
/// use capek_core::units::UnitExt;
///
/// // Readings jitter by about ±2 mm, the target drifts about 0.1 mm per sample.
/// let mut k = Kalman::new(0.1.mm(), 2.mm());
/// assert_eq!(k.update(100.mm()), 100.mm());
/// let y = k.update(104.mm());
/// assert!(y > 100.mm() && y < 104.mm());
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Kalman {
    q: f32,
    r: f32,
    estimate: f32,
    variance: f32,
    primed: bool,
}

impl Kalman {
    /// Creates a filter from two standard deviations in the signal's unit.
    /// The sign is ignored.
    ///
    /// ```
    /// use capek_core::filter::Kalman;
    /// use capek_core::units::UnitExt;
    ///
    /// let distance = Kalman::new(0.1.mm(), 2.mm());
    /// let heading = Kalman::new(0.05.deg(), 1.deg());
    /// let raw = Kalman::new(0.01_f32, 0.5_f32);
    /// ```
    pub fn new<Q: Quantity>(process_noise: Q, measurement_noise: Q) -> Self {
        let q = process_noise.to_f32();
        let r = measurement_noise.to_f32();
        Self {
            q: q * q,
            r: r * r,
            estimate: 0.0,
            variance: 0.0,
            primed: false,
        }
    }

    /// The current estimate's standard deviation, a measure of its
    /// uncertainty, in the signal's unit. Zero before the first sample.
    ///
    /// ```
    /// use capek_core::filter::{Filter, Kalman};
    /// use capek_core::units::{Length, UnitExt};
    ///
    /// let mut k = Kalman::new(0.1.mm(), 2.mm());
    /// k.update(10.mm());
    /// let before: Length = k.std_dev();
    /// k.update(10.mm());
    /// assert!(k.std_dev::<Length>() < before); // more readings, more certainty
    /// ```
    pub fn std_dev<Q: Quantity>(&self) -> Q {
        Q::from_f32(libm::sqrtf(self.variance))
    }
}

impl Filter for Kalman {
    fn update<Q: Quantity>(&mut self, x: Q) -> Q {
        let z = x.to_f32();
        if !self.primed {
            self.estimate = z;
            self.variance = self.r;
            self.primed = true;
            return x;
        }
        // Predict: the true value may have drifted.
        let p = self.variance + self.q;
        // Update: blend in the reading by the Kalman gain.
        let gain = if p + self.r > 0.0 {
            p / (p + self.r)
        } else {
            1.0
        };
        self.estimate += gain * (z - self.estimate);
        self.variance = (1.0 - gain) * p;
        Q::from_f32(self.estimate)
    }

    fn reset(&mut self) {
        self.primed = false;
    }
}

/// Limits how much the output may change per sample (a slew-rate limiter).
///
/// The output follows the input but moves at most `max_step` per update.
/// `max_step` is given in the signal's own unit.
///
/// ```
/// use capek_core::filter::{Filter, RateLimit};
/// use capek_core::units::UnitExt;
///
/// let mut r = RateLimit::new(2.mm());
/// assert_eq!(r.update(0.mm()), 0.mm());
/// assert_eq!(r.update(10.mm()), 2.mm());
/// assert_eq!(r.update(10.mm()), 4.mm());
/// assert_eq!(r.update(3.mm()), 3.mm()); // within reach, so exact
/// ```
#[derive(Debug, Clone, Copy)]
pub struct RateLimit {
    max_step: f32,
    state: Option<f32>,
}

impl RateLimit {
    /// Creates a limiter allowing at most `max_step` change per sample.
    /// The sign is ignored.
    ///
    /// ```
    /// use capek_core::filter::RateLimit;
    /// use capek_core::units::{Angle, UnitExt};
    ///
    /// let r = RateLimit::new((-5).deg());
    /// assert_eq!(r.max_step::<Angle>(), 5.deg());
    /// ```
    pub fn new<Q: Quantity>(max_step: Q) -> Self {
        Self {
            max_step: libm::fabsf(max_step.to_f32()),
            state: None,
        }
    }

    /// The largest change allowed per sample, in the signal's unit.
    ///
    /// ```
    /// use capek_core::filter::RateLimit;
    /// use capek_core::units::{Length, UnitExt};
    ///
    /// assert_eq!(RateLimit::new(1.5.mm()).max_step::<Length>(), 1.5.mm());
    /// ```
    pub fn max_step<Q: Quantity>(&self) -> Q {
        Q::from_f32(self.max_step)
    }
}

impl Filter for RateLimit {
    fn update<Q: Quantity>(&mut self, x: Q) -> Q {
        let x = x.to_f32();
        let y = match self.state {
            Some(prev) => prev + (x - prev).clamp(-self.max_step, self.max_step),
            None => x,
        };
        self.state = Some(y);
        Q::from_f32(y)
    }

    fn reset(&mut self) {
        self.state = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::{Length, UnitExt};

    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() <= tol
    }

    /// Feeds a sine of `freq_hz` and returns the peak output amplitude after
    /// the transient has died out.
    fn steady_amplitude(f: &mut impl Filter, freq_hz: f32, sample_hz: f32) -> f32 {
        let n = (sample_hz * 4.0) as usize;
        let mut peak = 0.0_f32;
        for i in 0..n {
            let x = libm::sinf(2.0 * PI * freq_hz * i as f32 / sample_hz);
            let y: f32 = f.update(x);
            if i > n / 2 {
                peak = peak.max(y.abs());
            }
        }
        peak
    }

    #[test]
    fn ema_formula() {
        let mut ema = Ema::new(0.2);
        let mut expected: Option<f32> = None;
        for s in [100.0_f32, 50.0, 50.0, 200.0] {
            let e = expected.map_or(s, |prev| 0.2 * s + 0.8 * prev);
            expected = Some(e);
            assert!(close(ema.update(s), e, 1e-4));
        }
    }

    #[test]
    fn ema_cutoff_attenuates_high_frequencies() {
        let sample_hz = 1000.0;
        let pass = steady_amplitude(&mut Ema::from_cutoff(10.0, sample_hz), 1.0, sample_hz);
        let stop = steady_amplitude(&mut Ema::from_cutoff(10.0, sample_hz), 200.0, sample_hz);
        assert!(pass > 0.95, "passband amplitude {pass}");
        assert!(stop < 0.1, "stopband amplitude {stop}");
    }

    #[test]
    fn ema_reset_starts_fresh() {
        let mut ema = Ema::new(0.1);
        ema.update(0.0_f32);
        ema.update(100.0_f32);
        ema.reset();
        assert_eq!(ema.update(42.0_f32), 42.0);
    }

    #[test]
    fn ema_works_on_lengths() {
        let mut ema = Ema::new(0.5);
        ema.update(10.mm());
        assert_eq!(ema.update(20.mm()), 15.mm());
    }

    #[test]
    fn moving_average_window_slides() {
        let mut avg = MovingAverage::<3>::new();
        assert_eq!(avg.update(3.0_f32), 3.0);
        assert_eq!(avg.update(6.0_f32), 4.5);
        assert_eq!(avg.update(9.0_f32), 6.0);
        assert_eq!(avg.update(0.0_f32), 5.0); // (6 + 9 + 0) / 3
        avg.reset();
        assert_eq!(avg.update(1.0_f32), 1.0);
    }

    #[test]
    fn median_partial_and_full_window() {
        let mut m = Median::<5>::new();
        assert_eq!(m.update(3.0_f32), 3.0);
        assert_eq!(m.update(1.0_f32), 1.0); // [1, 3] -> lower middle
        assert_eq!(m.update(2.0_f32), 2.0); // [1, 2, 3]
        m.update(10.0_f32);
        assert_eq!(m.update(11.0_f32), 3.0); // [1, 2, 3, 10, 11]
        assert_eq!(m.update(0.0_f32), 2.0); // oldest (3) replaced: [0, 1, 2, 10, 11]
        m.reset();
        assert_eq!(m.update(7.0_f32), 7.0);
    }

    #[test]
    fn biquad_low_pass_response() {
        let fs = 1000.0;
        let pass = steady_amplitude(&mut Biquad::butterworth_low_pass(50.0, fs), 5.0, fs);
        let corner = steady_amplitude(&mut Biquad::butterworth_low_pass(50.0, fs), 50.0, fs);
        let stop = steady_amplitude(&mut Biquad::butterworth_low_pass(50.0, fs), 400.0, fs);
        assert!(close(pass, 1.0, 0.02), "passband {pass}");
        // Butterworth is -3 dB (1/sqrt 2) at the corner.
        assert!(close(corner, FRAC_1_SQRT_2, 0.03), "corner {corner}");
        assert!(stop < 0.03, "stopband {stop}");
    }

    #[test]
    fn biquad_high_pass_response() {
        let fs = 1000.0;
        let stop = steady_amplitude(&mut Biquad::butterworth_high_pass(50.0, fs), 2.0, fs);
        let pass = steady_amplitude(&mut Biquad::butterworth_high_pass(50.0, fs), 300.0, fs);
        assert!(stop < 0.01, "stopband {stop}");
        assert!(close(pass, 1.0, 0.02), "passband {pass}");
    }

    #[test]
    fn biquad_notch_removes_center_only() {
        let fs = 1000.0;
        let center = steady_amplitude(&mut Biquad::notch(100.0, 5.0, fs), 100.0, fs);
        let away = steady_amplitude(&mut Biquad::notch(100.0, 5.0, fs), 20.0, fs);
        assert!(center < 0.02, "center {center}");
        assert!(away > 0.95, "away {away}");
    }

    #[test]
    fn biquad_primes_to_steady_state() {
        let mut lp = Biquad::butterworth_low_pass(5.0, 100.0);
        for _ in 0..20 {
            assert!(close(lp.update(42.0_f32), 42.0, 1e-3));
        }
        lp.reset();
        assert!(close(lp.update(-7.0_f32), -7.0, 1e-4));
    }

    #[test]
    fn kalman_converges_and_gains_confidence() {
        let mut k = Kalman::new(0.01_f32, 1.0_f32);
        let mut v = f32::INFINITY;
        let mut y = 0.0;
        // Alternating noise around 50.
        for i in 0..200 {
            let noise = if i % 2 == 0 { 1.0 } else { -1.0 };
            y = k.update(50.0_f32 + noise);
            let sd: f32 = k.std_dev();
            assert!(sd <= v);
            v = sd;
        }
        assert!(close(y, 50.0, 0.2), "estimate {y}");
    }

    #[test]
    fn kalman_tracks_a_step_faster_with_more_process_noise() {
        let mut slow = Kalman::new(0.01_f32, 1.0_f32);
        let mut fast = Kalman::new(1.0_f32, 1.0_f32);
        for _ in 0..50 {
            slow.update(0.0_f32);
            fast.update(0.0_f32);
        }
        let (s, f): (f32, f32) = (slow.update(10.0_f32), fast.update(10.0_f32));
        assert!(f > s);
    }

    #[test]
    fn rate_limit_both_directions() {
        let mut r = RateLimit::new(3.0_f32);
        r.update(10.0_f32);
        assert_eq!(r.update(0.0_f32), 7.0);
        assert_eq!(r.update(0.0_f32), 4.0);
        assert_eq!(r.update(20.0_f32), 7.0);
    }

    #[test]
    fn typed_parameters_use_the_signal_unit() {
        // 5 degrees per sample, fed angles.
        let mut r = RateLimit::new(5.deg());
        r.update(0.deg());
        let y: crate::units::Angle = r.update(90.deg());
        assert!(close(y.as_deg(), 5.0, 1e-4));

        // Kalman noise in cm equals the same noise in mm.
        let mut a = Kalman::new(0.1.cm(), 0.2.cm());
        let mut b = Kalman::new(1.mm(), 2.mm());
        for x in [10.mm(), 14.mm(), 9.mm()] {
            let (ya, yb): (Length, Length) = (a.update(x), b.update(x));
            assert!(close(ya.as_mm(), yb.as_mm(), 1e-4));
        }
    }

    #[test]
    fn chain_applies_in_order_and_resets_both() {
        let mut f = Median::<3>::new().then(Ema::new(0.5));
        let out: Length = f.update(10.mm());
        assert_eq!(out, 10.mm());
        f.reset();
        assert_eq!(f.update(99.mm()), 99.mm());
    }
}

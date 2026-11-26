//! What components ask of the hardware.
//!
//! Drivers are the hardware primitives behind components, supplied by a
//! platform. This module holds the types they share with motion control.

/// One motor step: its direction and the time since the previous step.
///
/// Bit 31 is the direction, set for forward. Bits 0 to 30 are the delay since
/// the previous step on the same axis, in ticks of the driver's timer. A
/// stepper driver consumes a stream of these and pulses its step pin after
/// each delay. Because the direction travels with each step, a reversal lands
/// on exactly the right pulse.
///
/// ```
/// use capek_core::driver::StepWord;
///
/// let word = StepWord::new(false, 4000);
/// assert!(!word.forward());
/// assert_eq!(word.delay(), 4000);
/// assert_eq!(StepWord::new(true, 4000).bits(), (1 << 31) | 4000);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct StepWord(u32);

impl StepWord {
    /// The longest delay a word can hold, in ticks.
    pub const MAX_DELAY: u32 = (1 << 31) - 1;

    const FORWARD: u32 = 1 << 31;

    /// A step in the given direction after `delay` ticks, capped at
    /// [`MAX_DELAY`](Self::MAX_DELAY).
    pub const fn new(forward: bool, delay: u32) -> Self {
        let delay = if delay > Self::MAX_DELAY {
            Self::MAX_DELAY
        } else {
            delay
        };
        Self(if forward {
            Self::FORWARD | delay
        } else {
            delay
        })
    }

    /// Whether the step is forward.
    pub const fn forward(self) -> bool {
        self.0 & Self::FORWARD != 0
    }

    /// Ticks since the previous step on the same axis.
    pub const fn delay(self) -> u32 {
        self.0 & Self::MAX_DELAY
    }

    /// The raw word, as a driver sends it to the hardware.
    pub const fn bits(self) -> u32 {
        self.0
    }

    /// A word from its raw bits.
    ///
    /// ```
    /// use capek_core::driver::StepWord;
    ///
    /// let word = StepWord::new(true, 123);
    /// assert_eq!(StepWord::from_bits(word.bits()), word);
    /// ```
    pub const fn from_bits(bits: u32) -> Self {
        Self(bits)
    }
}

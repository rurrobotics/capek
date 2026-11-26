//! Buttons and plain outputs on digital pins.

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

/// Settings of a [`Button`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ButtonConfig {
    /// Whether the pin reads low while pressed, as for a button to ground
    /// with a pull-up. The default.
    pub active_low: bool,
}

impl Default for ButtonConfig {
    fn default() -> Self {
        Self { active_low: true }
    }
}

/// A button or switch on an input pin.
///
/// It waits on pin edges instead of polling. There is no debouncing: a
/// bouncing contact can report a press more than once.
///
/// ```
/// use capek_core::components::{Button, ButtonConfig};
/// # use core::convert::Infallible;
/// # use embedded_hal::digital::{ErrorType, InputPin};
/// # use embedded_hal_async::digital::Wait;
/// # struct Pin(bool);
/// # impl ErrorType for Pin { type Error = Infallible; }
/// # impl InputPin for Pin {
/// #     fn is_high(&mut self) -> Result<bool, Infallible> { Ok(self.0) }
/// #     fn is_low(&mut self) -> Result<bool, Infallible> { Ok(!self.0) }
/// # }
/// # impl Wait for Pin {
/// #     async fn wait_for_high(&mut self) -> Result<(), Infallible> { Ok(()) }
/// #     async fn wait_for_low(&mut self) -> Result<(), Infallible> { Ok(()) }
/// #     async fn wait_for_rising_edge(&mut self) -> Result<(), Infallible> { Ok(()) }
/// #     async fn wait_for_falling_edge(&mut self) -> Result<(), Infallible> { Ok(()) }
/// #     async fn wait_for_any_edge(&mut self) -> Result<(), Infallible> { Ok(()) }
/// # }
///
/// // A button to ground with a pull-up reads low while pressed.
/// let mut button = Button::new(Pin(false), ButtonConfig::default());
/// assert_eq!(button.is_pressed(), Ok(true));
/// ```
#[derive(Debug)]
pub struct Button<I: InputPin + Wait> {
    pin: I,
    config: ButtonConfig,
}

impl<I: InputPin + Wait> Button<I> {
    /// A button on `pin`.
    pub fn new(pin: I, config: ButtonConfig) -> Self {
        Self { pin, config }
    }

    /// Whether the button is pressed now.
    pub fn is_pressed(&mut self) -> Result<bool, I::Error> {
        if self.config.active_low {
            self.pin.is_low()
        } else {
            self.pin.is_high()
        }
    }

    /// Waits until the button is pressed, returning at once if it already is.
    pub async fn wait_pressed(&mut self) -> Result<(), I::Error> {
        if self.config.active_low {
            self.pin.wait_for_low().await
        } else {
            self.pin.wait_for_high().await
        }
    }

    /// Waits until the button is released, returning at once if it already
    /// is.
    pub async fn wait_released(&mut self) -> Result<(), I::Error> {
        if self.config.active_low {
            self.pin.wait_for_high().await
        } else {
            self.pin.wait_for_low().await
        }
    }

    /// Waits for the next press: for the button to be released, if it is
    /// held, and then pressed.
    pub async fn wait_press(&mut self) -> Result<(), I::Error> {
        self.wait_released().await?;
        self.wait_pressed().await
    }
}

/// Settings of an [`Output`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct OutputConfig {
    /// Whether the pin is driven low to switch the output on, as for an LED
    /// to the supply. Off by default.
    pub active_low: bool,
}

/// A plain digital output, such as an LED, a relay or a pump.
///
/// ```
/// use capek_core::components::{Output, OutputConfig};
/// # use core::convert::Infallible;
/// # use embedded_hal::digital::{ErrorType, OutputPin};
/// # struct Pin(bool);
/// # impl ErrorType for Pin { type Error = Infallible; }
/// # impl OutputPin for Pin {
/// #     fn set_high(&mut self) -> Result<(), Infallible> { self.0 = true; Ok(()) }
/// #     fn set_low(&mut self) -> Result<(), Infallible> { self.0 = false; Ok(()) }
/// # }
///
/// let mut led = Output::new(Pin(false), OutputConfig::default());
/// led.on().unwrap();
/// assert!(led.is_on());
/// led.toggle().unwrap();
/// assert!(!led.is_on());
/// ```
#[derive(Debug)]
pub struct Output<O: OutputPin> {
    pin: O,
    config: OutputConfig,
    on: bool,
}

impl<O: OutputPin> Output<O> {
    /// An output on `pin`, switched off.
    pub fn new(pin: O, config: OutputConfig) -> Self {
        let mut output = Self {
            pin,
            config,
            on: true,
        };
        // An error here shows on the first switch; the state stays off.
        let _ = output.set(false);
        output.on = false;
        output
    }

    /// Switches the output on or off.
    pub fn set(&mut self, on: bool) -> Result<(), O::Error> {
        if on != self.config.active_low {
            self.pin.set_high()?;
        } else {
            self.pin.set_low()?;
        }
        self.on = on;
        Ok(())
    }

    /// Switches the output on.
    pub fn on(&mut self) -> Result<(), O::Error> {
        self.set(true)
    }

    /// Switches the output off.
    pub fn off(&mut self) -> Result<(), O::Error> {
        self.set(false)
    }

    /// Switches the output to the other state.
    pub fn toggle(&mut self) -> Result<(), O::Error> {
        self.set(!self.on)
    }

    /// Whether the output is on.
    pub fn is_on(&self) -> bool {
        self.on
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockPin;
    use embassy_futures::block_on;

    #[test]
    fn active_low_button() {
        let mut b = Button::new(MockPin::low(), ButtonConfig::default());
        assert_eq!(b.is_pressed(), Ok(true));
        b.pin.level = true;
        assert_eq!(b.is_pressed(), Ok(false));
    }

    #[test]
    fn active_high_button() {
        let mut b = Button::new(MockPin::low(), ButtonConfig { active_low: false });
        assert_eq!(b.is_pressed(), Ok(false));
    }

    #[test]
    fn waits_for_the_right_level() {
        let mut b = Button::new(MockPin::low(), ButtonConfig::default());
        block_on(b.wait_pressed()).unwrap();
        assert_eq!(b.pin.waited, [false]);
        block_on(b.wait_press()).unwrap();
        // A press is a release followed by a press.
        assert_eq!(b.pin.waited, [false, true, false]);
    }

    #[test]
    fn output_levels() {
        let mut o = Output::new(MockPin::low(), OutputConfig::default());
        assert!(!o.pin.level);
        o.on().unwrap();
        assert!(o.pin.level && o.is_on());

        let mut o = Output::new(MockPin::low(), OutputConfig { active_low: true });
        // Off means high for an active-low output.
        assert!(o.pin.level && !o.is_on());
        o.toggle().unwrap();
        assert!(!o.pin.level && o.is_on());
    }
}

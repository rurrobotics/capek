//! Physical quantities: [`Length`] and [`Angle`].
//!
//! capek APIs take a quantity, never a bare `f32`, so a signature such as
//! `fn drive(&mut self, distance: Length)` documents itself. The caller picks
//! whichever unit is natural, the same way [`core::time::Duration`] is built
//! with `from_millis` or `from_secs`. No API ever has to convert.
//!
//! The internal storage unit is private: lengths are kept in millimetres and
//! angles in radians. That can change without breaking callers.
//!
//! Both types are `#[repr(transparent)]` over `f32`, so they have exactly the
//! size, alignment and ABI of a plain `f32` and add no runtime cost.
//!
//! # Examples
//!
//! ```
//! use capek_core::units::{Angle, Length, UnitExt};
//!
//! // Build quantities in any unit.
//! let a = Length::mm(160.0);
//! let b = Length::m(0.18);
//! assert_eq!((a + b).as_mm(), 340.0);
//!
//! // The literal extension trait gives the same values, shorter.
//! assert_eq!(160.mm(), a);
//! assert_eq!(0.5.m(), Length::mm(500.0));
//! assert_eq!((-90).deg(), -Angle::deg(90.0));
//!
//! // Dividing two lengths gives a plain ratio.
//! let wheel_circumference = Length::mm(56.0) * core::f32::consts::PI;
//! let revolutions: f32 = 340.mm() / wheel_circumference;
//! assert!((revolutions - 1.932).abs() < 0.001);
//!
//! // Lengths and angles do not mix. This would not compile:
//! // let wrong = 10.mm() + 90.deg();
//! ```

use core::f32::consts::PI;
use core::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

/// Arithmetic shared by all quantity types.
macro_rules! quantity_ops {
    ($name:ident) => {
        impl $name {
            /// Zero of this quantity.
            pub const ZERO: Self = Self(0.0);

            /// Absolute value.
            ///
            /// ```
            #[doc = concat!("use capek_core::units::", stringify!($name), ";")]
            #[doc = concat!("assert_eq!(", stringify!($name), "::ZERO.abs(), ", stringify!($name), "::ZERO);")]
            /// ```
            pub fn abs(self) -> Self {
                Self(libm::fabsf(self.0))
            }
        }

        impl Add for $name {
            type Output = Self;
            fn add(self, rhs: Self) -> Self {
                Self(self.0 + rhs.0)
            }
        }

        impl AddAssign for $name {
            fn add_assign(&mut self, rhs: Self) {
                self.0 += rhs.0;
            }
        }

        impl Sub for $name {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self {
                Self(self.0 - rhs.0)
            }
        }

        impl SubAssign for $name {
            fn sub_assign(&mut self, rhs: Self) {
                self.0 -= rhs.0;
            }
        }

        impl Neg for $name {
            type Output = Self;
            fn neg(self) -> Self {
                Self(-self.0)
            }
        }

        /// Scaling by a plain number.
        impl Mul<f32> for $name {
            type Output = Self;
            fn mul(self, rhs: f32) -> Self {
                Self(self.0 * rhs)
            }
        }

        /// Scaling by a plain number.
        impl Div<f32> for $name {
            type Output = Self;
            fn div(self, rhs: f32) -> Self {
                Self(self.0 / rhs)
            }
        }

        /// Ratio of two quantities of the same kind.
        impl Div for $name {
            type Output = f32;
            fn div(self, rhs: Self) -> f32 {
                self.0 / rhs.0
            }
        }
    };
}

/// A length. Build it with [`Length::mm`], [`Length::cm`] or [`Length::m`].
///
/// ```
/// use capek_core::units::Length;
///
/// let wheel_diameter = Length::mm(56.0);
/// let track = Length::cm(15.9);
/// assert!(track > wheel_diameter);
/// assert_eq!(format!("{wheel_diameter}"), "56 mm");
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq, PartialOrd)]
#[repr(transparent)]
pub struct Length(f32);

quantity_ops!(Length);

impl Length {
    /// A length in millimetres.
    ///
    /// ```
    /// use capek_core::units::Length;
    /// assert_eq!(Length::mm(1000.0), Length::m(1.0));
    /// ```
    pub const fn mm(v: f32) -> Self {
        Self(v)
    }

    /// A length in centimetres.
    ///
    /// ```
    /// use capek_core::units::Length;
    /// assert_eq!(Length::cm(5.0).as_mm(), 50.0);
    /// ```
    pub const fn cm(v: f32) -> Self {
        Self(v * 10.0)
    }

    /// A length in metres.
    ///
    /// ```
    /// use capek_core::units::Length;
    /// assert_eq!(Length::m(0.25).as_mm(), 250.0);
    /// ```
    pub const fn m(v: f32) -> Self {
        Self(v * 1000.0)
    }

    /// The length in millimetres.
    ///
    /// ```
    /// use capek_core::units::Length;
    /// assert_eq!(Length::cm(1.5).as_mm(), 15.0);
    /// ```
    pub const fn as_mm(self) -> f32 {
        self.0
    }

    /// The length in centimetres.
    ///
    /// ```
    /// use capek_core::units::Length;
    /// assert_eq!(Length::mm(25.0).as_cm(), 2.5);
    /// ```
    pub const fn as_cm(self) -> f32 {
        self.0 / 10.0
    }

    /// The length in metres.
    ///
    /// ```
    /// use capek_core::units::Length;
    /// assert_eq!(Length::mm(1500.0).as_m(), 1.5);
    /// ```
    pub const fn as_m(self) -> f32 {
        self.0 / 1000.0
    }
}

impl core::fmt::Display for Length {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{} mm", self.0)
    }
}

/// An angle. Positive is counter-clockwise. Build it with [`Angle::deg`] or
/// [`Angle::rad`].
///
/// ```
/// use capek_core::units::Angle;
///
/// let left = Angle::deg(90.0);
/// let right = -left;
/// assert_eq!(right.as_deg(), -90.0);
/// assert_eq!(format!("{right}"), "-90°");
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq, PartialOrd)]
#[repr(transparent)]
pub struct Angle(f32);

quantity_ops!(Angle);

impl Angle {
    /// An angle in degrees.
    ///
    /// ```
    /// use capek_core::units::Angle;
    /// let a = Angle::deg(180.0);
    /// assert!((a.as_rad() - core::f32::consts::PI).abs() < 1e-6);
    /// ```
    pub const fn deg(v: f32) -> Self {
        Self(v * (PI / 180.0))
    }

    /// An angle in radians.
    ///
    /// ```
    /// use capek_core::units::Angle;
    /// let a = Angle::rad(core::f32::consts::FRAC_PI_2);
    /// assert!((a.as_deg() - 90.0).abs() < 1e-4);
    /// ```
    pub const fn rad(v: f32) -> Self {
        Self(v)
    }

    /// The angle in degrees.
    ///
    /// ```
    /// use capek_core::units::Angle;
    /// assert!((Angle::deg(45.0).as_deg() - 45.0).abs() < 1e-4);
    /// ```
    pub const fn as_deg(self) -> f32 {
        self.0 * (180.0 / PI)
    }

    /// The angle in radians.
    ///
    /// ```
    /// use capek_core::units::Angle;
    /// assert_eq!(Angle::rad(1.0).as_rad(), 1.0);
    /// ```
    pub const fn as_rad(self) -> f32 {
        self.0
    }
}

impl core::fmt::Display for Angle {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}°", self.as_deg())
    }
}

/// Builds quantities from number literals: `160.mm()`, `0.5.m()`, `(-90).deg()`.
///
/// Implemented for `i32` and `f32`, which covers unsuffixed integer and float
/// literals. Negative integer literals need parentheses, because `-90.deg()`
/// parses as `-(90.deg())`; both give the same value here.
///
/// ```
/// use capek_core::units::{Angle, Length, UnitExt};
///
/// assert_eq!(160.mm(), Length::mm(160.0));
/// assert_eq!(2.5.cm(), Length::mm(25.0));
/// assert_eq!(1.m(), Length::mm(1000.0));
/// assert_eq!((-90).deg(), Angle::deg(-90.0));
/// assert_eq!(1.5.rad(), Angle::rad(1.5));
/// ```
pub trait UnitExt {
    /// This many millimetres.
    fn mm(self) -> Length;
    /// This many centimetres.
    fn cm(self) -> Length;
    /// This many metres.
    fn m(self) -> Length;
    /// This many degrees.
    fn deg(self) -> Angle;
    /// This many radians.
    fn rad(self) -> Angle;
}

impl UnitExt for f32 {
    fn mm(self) -> Length {
        Length::mm(self)
    }
    fn cm(self) -> Length {
        Length::cm(self)
    }
    fn m(self) -> Length {
        Length::m(self)
    }
    fn deg(self) -> Angle {
        Angle::deg(self)
    }
    fn rad(self) -> Angle {
        Angle::rad(self)
    }
}

impl UnitExt for i32 {
    fn mm(self) -> Length {
        Length::mm(self as f32)
    }
    fn cm(self) -> Length {
        Length::cm(self as f32)
    }
    fn m(self) -> Length {
        Length::m(self as f32)
    }
    fn deg(self) -> Angle {
        Angle::deg(self as f32)
    }
    fn rad(self) -> Angle {
        Angle::rad(self as f32)
    }
}

/// A scalar value that generic numeric code, such as [`crate::filter`], can
/// work on: `f32`, [`Length`] and [`Angle`].
///
/// The conversion uses the private storage unit, so it is only meaningful as a
/// round trip: `Q::from_f32(q.to_f32()) == q`. Do not use it to read a value in
/// a particular unit; use the `as_*` accessors for that.
///
/// ```
/// use capek_core::units::{Length, Quantity, UnitExt};
///
/// fn halve<Q: Quantity>(q: Q) -> Q {
///     Q::from_f32(q.to_f32() / 2.0)
/// }
///
/// assert_eq!(halve(10.mm()), 5.mm());
/// assert_eq!(halve(3.0_f32), 1.5);
/// ```
pub trait Quantity: Copy {
    /// The value in this type's storage unit.
    fn to_f32(self) -> f32;
    /// Builds a value from a number in this type's storage unit.
    fn from_f32(v: f32) -> Self;
}

impl Quantity for f32 {
    fn to_f32(self) -> f32 {
        self
    }
    fn from_f32(v: f32) -> Self {
        v
    }
}

impl Quantity for Length {
    fn to_f32(self) -> f32 {
        self.0
    }
    fn from_f32(v: f32) -> Self {
        Self(v)
    }
}

impl Quantity for Angle {
    fn to_f32(self) -> f32 {
        self.0
    }
    fn from_f32(v: f32) -> Self {
        Self(v)
    }
}

// The zero-cost claim in the module docs, checked at compile time.
const _: () = assert!(core::mem::size_of::<Length>() == core::mem::size_of::<f32>());
const _: () = assert!(core::mem::size_of::<Angle>() == core::mem::size_of::<f32>());

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-4
    }

    #[test]
    fn length_units_agree() {
        assert_eq!(Length::m(1.0), Length::mm(1000.0));
        assert_eq!(Length::cm(1.0), Length::mm(10.0));
        assert!(close(Length::mm(1234.0).as_m(), 1.234));
        assert!(close(Length::mm(1234.0).as_cm(), 123.4));
    }

    #[test]
    fn angle_units_agree() {
        assert!(close(Angle::deg(180.0).as_rad(), PI));
        assert!(close(Angle::rad(PI).as_deg(), 180.0));
        assert!(close(Angle::deg(-90.0).as_deg(), -90.0));
    }

    #[test]
    fn arithmetic() {
        assert_eq!(10.mm() + 5.mm(), 15.mm());
        assert_eq!(10.mm() - 15.mm(), (-5).mm());
        assert_eq!(-3.mm(), (-3).mm());
        assert_eq!(4.mm() * 2.5, 10.mm());
        assert_eq!(10.mm() / 4.0, 2.5.mm());
        assert_eq!(10.mm() / 4.mm(), 2.5);
        assert_eq!((-2).mm().abs(), 2.mm());

        let mut a = Angle::rad(1.0);
        a += Angle::rad(0.5);
        a -= Angle::rad(0.25);
        assert_eq!(a, Angle::rad(1.25));
    }

    #[test]
    fn literals_match_constructors() {
        assert_eq!(160.mm(), Length::mm(160.0));
        assert_eq!(160.0.mm(), Length::mm(160.0));
        assert_eq!(3.cm(), Length::cm(3.0));
        assert_eq!(2.m(), Length::m(2.0));
        assert_eq!(90.deg(), Angle::deg(90.0));
        assert_eq!(1.rad(), Angle::rad(1.0));
    }

    #[test]
    fn display() {
        assert_eq!(format!("{}", Length::cm(5.6)), "56 mm");
        assert_eq!(format!("{}", Angle::deg(-90.0)), "-90°");
    }
}

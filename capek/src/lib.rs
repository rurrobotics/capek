//! capek: a small robotics framework for Embassy.
//!
//! This crate is the chip-facing half of capek. It will re-export everything from
//! [`capek_core`] and will hold the RP2040/RP2350 drivers that implement the
//! core capability traits.
//!
//! Exactly one chip feature must be enabled: `rp2040` or `rp2350`.
#![no_std]

#[cfg(not(any(feature = "rp2350", feature = "rp2040")))]
compile_error!("capek: enable exactly one chip feature: `rp2350` or `rp2040`");

#[cfg(all(feature = "rp2350", feature = "rp2040"))]
compile_error!("capek: features `rp2350` and `rp2040` are mutually exclusive");

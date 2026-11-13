//! Hardware-independent core of the capek robotics framework.
//!
//! This crate never depends on a chip HAL. Everything in it compiles for the
//! host as well as for microcontrollers, so motion math, filters and
//! components can be unit-tested with `cargo test -p capek-core`.
//!
//! Chip backends (the `capek` crate for RP2040/RP2350) implement the
//! capability traits defined here. Components are generic over those traits,
//! so they work unchanged on any backend.
#![cfg_attr(not(test), no_std)]
#![warn(missing_docs)]

pub mod filter;
pub mod units;

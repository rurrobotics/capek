//! Wire types shared between capek firmware and host-side controllers.
//!
//! The firmware and the host both depend on this crate, which replaces the
//! `include!("../../firmware/src/packet.rs")` pattern used by ninja. It is
//! `no_std` by default; enable the `std` feature on the host side.
#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![warn(missing_docs)]

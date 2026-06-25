#![no_std]

#[cfg(not(any(feature = "rp2350", feature = "rp2040")))]
compile_error!("capek: enable exactly one chip feature: `rp2350` or `rp2040`");

#[cfg(all(feature = "rp2350", feature = "rp2040"))]
compile_error!("capek: features `rp2350` and `rp2040` are mutually exclusive");

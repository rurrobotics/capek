# AGENTS.md

Guidance for AI coding agents working in this repository. Human contributors
may find it useful too. The project overview lives in [README.md](README.md),
and the full architecture and plan of action in [BLUEPRINT.md](BLUEPRINT.md).
Read both before working.

## Workflow

- Work on the `dev` branch. `stable` is the main branch and is only updated by
  the maintainer.
- Make small commits, one logical change each.
- **Never commit without approval.** Stage the change, show the maintainer the
  diff summary and the proposed message, and wait for an explicit yes.
- Use [Conventional Commits](https://www.conventionalcommits.org):
  `feat(core): ...`, `fix(rp): ...`, `docs: ...`, `ci: ...`, `chore: ...`.
  Scopes follow the crate or module, for example `core`, `rp`, `macros` or
  `proto`.
- End every commit written with an agent with a `Co-Authored-By` trailer
  naming the model.
- Ask before making design decisions that the README or existing code does not
  settle.

## Code rules

- `capek-core` and `capek-contrib` must never depend on a chip HAL
  (`embassy-rp` or similar). They have to build and test on the host.
- Everything outside tests is `no_std`. No heap allocation; use `heapless` when
  a collection is needed.
- Use `f32`, not `f64`. The RP2040 has no FPU and the RP2350 only has an f32 FPU.
- Public APIs take quantity types from `capek_core::units` (`Length`, `Angle`)
  for physical values, never a bare `f32`, so the signature documents the unit.
- Use `core::time::Duration` for general-purpose time spans, such as the time
  step of a motion plan or a quantity divided by time. Use `embassy_time`
  types for time measured against the board's clock, such as timeouts, timers
  and timestamps, because they map directly to hardware ticks and are faster.
  `capek-core` code that needs board time takes it through a trait or generic
  rather than depending on `embassy_time` directly.
- Shared external dependency versions go in `[workspace.dependencies]` and must
  match the versions `embassy-rp` pulls in.
- Every public item gets rustdoc. Crates use `#![warn(missing_docs)]`.
  Documentation lives in rustdoc, the README and BLUEPRINT.md; there is no
  separate docs folder. Keep BLUEPRINT.md in sync when a design decision
  changes, and tick off its plan of action as commits land.
- Every public type, trait and function gets a runnable rustdoc example where
  one makes sense. Modules get an example showing typical use. Examples run as
  doctests, so keep them compiling.
- Add unit tests for hardware-independent logic in `capek-core`.

## Checks

Run these before proposing a commit. CI runs the same steps with warnings
treated as errors.

```sh
cargo fmt --all --check
RUSTFLAGS="-D warnings" cargo clippy -p capek-core -p capek-proto -p capek-macros --all-targets
cargo test -p capek-core -p capek-proto -p capek-macros
RUSTFLAGS="-D warnings" cargo clippy -p capek --features rp2040 --target thumbv6m-none-eabi
RUSTFLAGS="-D warnings" cargo clippy -p capek --features rp2350 --target thumbv8m.main-none-eabihf
```

## Reference code

The [ninja](https://github.com/rurrobotics/ninja) firmware is the base capek is
built from. Its `master` branch targets the RP2350 and its `hybrid` branch the
RP2040. When porting a driver or profile from it, keep its behaviour and note
any intentional difference in the rustdoc.

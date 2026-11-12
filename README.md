# capek

A small robotics framework for [Embassy](https://embassy.dev), written in Rust.

capek grew out of the [ninja](https://github.com/rurrobotics/ninja) Eurobot robot
firmware. Its goal is to make a robot's firmware a short description of *what is
wired where*, plus the strategy code. Peripheral plumbing, task setup and motion
control come from the framework.

> **Status:** early development. The workspace layout exists; the core
> library is being written. Nothing here is usable on a robot yet. The
> sections below describe the intended design. The [Roadmap](#roadmap)
> lists which parts exist.

## Goals

- **Multiple boards.** RP2040 and RP2350 first. Other chip families can follow
  by adding a backend, without touching components.
- **Plug and play.** A `robot!` block maps pins to drivers and drivers to
  components. The macro generates the setup code.
- **Easy to override.** Every driver, component and motion profile is a plain
  type behind a trait. Your own types slot in the same way the built-in ones do.
- **Little abstraction in user code.** Strategies are ordinary `async fn`s that
  call methods on components and combine them with `join` and `select`.

## Crates

| Crate | Purpose | Builds on host |
|---|---|---|
| `capek-core` | Capability traits, motion control, filters, components, topics. No chip HAL. | yes |
| `capek` | RP2040/RP2350 drivers implementing the core traits. Re-exports `capek-core`. | no |
| `capek-macros` | `robot!` and `#[capek::main]`. | yes |
| `capek-proto` | Wire types shared by the firmware and host-side controllers. | yes |

`capek-core` is deliberately free of `embassy-rp`. Motion math, filters and
components can be unit-tested on a PC, and a new chip family only needs a new
backend.

## Design

### Drivers, components, robot

- A **driver** talks to hardware, such as a PIO stepper, a servo or an
  ultrasonic sensor. It implements a capability trait like `Stepper`, `Servo`
  or `DistanceSensor`.
- A **component** is a mechanism built from drivers, such as a differential
  drivetrain, a gripper or a homed linear axis. It is generic over the
  capability traits, so it accepts any driver that implements them.
- The **robot** struct owns the components. Your strategy borrows it.

Components nest through traits: `Drivetrain<L: Stepper, R: Stepper>` works with
any two steppers, whether built-in, your own, or shared from another task.

### Runtime model

Actuators are owned directly by the strategy, as in ninja:

```rust
join(robot.drivetrain.drive(Mm(160.0)), robot.gripper.open()).await;
```

Dropping a motion future stops the motor immediately (the *hard stop*).

Only things that genuinely run on their own become Embassy tasks: sensors that
sample continuously, the communication link, and any component you explicitly
mark as shared. Those tasks publish typed **topics** (built on `embassy-sync`)
that the strategy can wait on, for example "proximity is close".

### Motion control

Steppers receive their step timing from a **step source**, streamed to the
hardware in small chunks. The RP backend uses PIO and DMA, so pulse timing
never depends on the CPU. Three sources are planned:

| Source | How it works | Pause and resume | Change speed mid-move |
|---|---|---|---|
| `Ramp` (default) | The acceleration ramp is precomputed once per profile and indexed per step. | yes | yes |
| `Realtime` | Speed is integrated at every step. | yes | yes, including the profile itself |
| `Table` | Replays a delay table you supply, ninja style. | no | no |

With `Ramp` and `Realtime`, a running move can be paused, for example by a
proximity sensor. The motors decelerate, wait, accelerate again and still finish
the full distance.

### The `robot!` block (planned)

```rust
capek::robot! {
    name: Ninja,
    components {
        drivetrain: Drivetrain {
            left:  PioStepper { step: PIN_6,  dir: PIN_5,  pio: PIO1, sm: 0, dma: DMA_CH1 },
            right: PioStepper { step: PIN_27, dir: PIN_28, pio: PIO1, sm: 1, dma: DMA_CH2 },
            wheel_diameter: Mm(56.0), track: Mm(159.0), steps_per_rev: 400,
            pause_on: proximity,
        },
    }
    sensors {
        proximity: Proximity {
            sensor: Ultrasonic { trig: PIN_21, echo: PIN_19 },
            threshold: Mm(60.0),
        },
    }
}
```

Uppercase identifiers such as `PIN_6` are peripherals. Nested `Type { .. }`
blocks are child drivers. Everything else is an ordinary Rust expression.
Everything the macro generates is public API, so you can also write the setup
by hand.

## Roadmap

- [x] **M0** Workspace scaffold and CI
- [ ] **M1** `capek-core`: traits, step sources, profiles, filters, kinematics
- [ ] **M2** RP drivers: PIO stepper, servo, ultrasonic, GPIO
- [ ] **M3** Components: drivetrain, linear axis, gripper, proximity, starter
- [ ] **M4** `robot!` and `#[capek::main]`
- [ ] **M5** Link: transport trait, USB serial, `capek-proto`
- [ ] **M6** Port ninja to capek
- [ ] **M7** Shared components as tasks

## Development

```sh
# Host tests for the hardware-independent crates
cargo test -p capek-core -p capek-proto -p capek-macros

# Type-check the chip crate
cargo check -p capek --features rp2040 --target thumbv6m-none-eabi
cargo check -p capek --features rp2350 --target thumbv8m.main-none-eabihf
```

## AI usage

capek is developed with the help of an AI coding assistant, implementing a
human-designed architecture on top of the [ninja](https://github.com/rurrobotics/ninja)
firmware. Every commit is reviewed and approved by a maintainer before it lands.

## License

GPL-3.0-or-later. See [LICENSE](LICENSE).

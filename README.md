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

- **Multiple boards.** RP2040 and RP2350 first. Each platform is its own crate,
  selected with a feature on `capek`. Switching platform changes pins and the
  feature flag, not the robot's structure or strategy.
- **Plug and play.** A `robot!` block maps pins to drivers and drivers to
  components. The macro generates the setup code.
- **Easy to override.** Every driver, component and motion profile is a plain
  type behind a trait. Your own types slot in the same way the built-in ones do.
- **Little abstraction in user code.** Strategies are ordinary `async fn`s that
  call methods on components and combine them with `join` and `select`.
- **Smooth, interruptible motion.** S-curve profiles, curved routes through
  waypoints, and moves that can be paused by a sensor and still finish.

## Crates

| Crate | Purpose | Builds on host |
|---|---|---|
| `capek` | The crate you depend on. Re-exports everything; a feature such as `rp2040` selects the platform. | no |
| `capek-core` | Components, motion control, filters, command mutexes, driver traits. No chip HAL. | yes |
| `capek-rp` | RP2040/RP2350 platform: the drivers behind the components. | no |
| `capek-macros` | `robot!` and `#[capek::main]`. | yes |
| `capek-proto` | Wire types shared by the firmware and host-side controllers. | yes |

`capek-core` is deliberately free of any chip HAL. Motion math, filters and
components can be unit-tested on a PC, and a new chip family only needs a new
platform crate.

## Design

The full architecture and the commit-by-commit plan of action are in
[BLUEPRINT.md](BLUEPRINT.md). This section is the short version.

### Components and drivers

- A **component** is what you write in a `robot!` block: `Stepper`, `Servo`,
  `Button`, `Ultrasonic`, and mechanisms built from them, such as a
  `Drivetrain` from two `Stepper`s or a `Gripper` from a `Servo`. Components
  are platform-neutral and hold the logic that does not depend on hardware,
  such as steps per revolution or servo calibration.
- A **driver** is the hardware primitive behind a component, supplied by the
  selected platform. On RP, a `Stepper` is driven through PIO and DMA.
- In a `robot!` block the driver sits in its own `driver:` block, visually
  separate from the component's fields. Steppers come in two wirings,
  `StepDir` and `FourWire`, and `driver: YourDriver { .. }` plugs in your own.
- Each platform crate implements one `Platform` trait naming its drivers, so
  the compiler checks that a platform is complete.

### Runtime model

There are three kinds of parts at runtime:

- **Owned components** are declared without a command mutex. They are direct
  fields of the robot your `main` receives, and your strategy calls them
  directly. Dropping a motion future stops the motor immediately.
- **Commanded components** run as their own task. They are driven through a
  **command mutex** declared at the top of the `robot!` block. The strategy, a
  sensor and the communication link can all send commands to the same mutex.
- **Sensors** run as tasks. They publish their latest reading and fire
  **trigger callbacks**, for example when a distance drops below a threshold.

```rust
async fn game(r: &mut Ninja) {
    join(r.drive.forward(160.mm()), r.gripper.open()).await;
    r.drive.turn((-90).deg()).await;
    r.drive.follow(Route::from_slice(&[
        Waypoint::at(600.mm(), 0.mm()),                     // straight segment
        Waypoint::with_slope(900.mm(), 300.mm(), 90.deg()), // smooth curve into a heading
    ])).await;
}
```

Commands are enums with fields, such as `Stop`, `Start`, `Forward(Length)` or
`Follow(Route)`. How competing writers are resolved is configurable per mutex.
The default gives every writer its own hold: if the proximity sensor sends
`Stop`, the robot decelerates and waits, and only the proximity sensor's
`Start` lets it continue. The interrupted move then finishes its full
distance.

### Motion control

Motion is split into three independent choices:

- **Profile:** how speed changes along the path. `Trapezoid`, `Power` (eases
  into cruise) and `Sine` (smooth acceleration), with `FastTrapezoid` and
  `FastPower` as cheap per-step versions; `SCurve` limits jerk for the
  smoothest motion, and `Constant` skips ramps entirely.
- **Path:** where each wheel should be. Straight moves, in-place turns, and
  routes through **waypoints**. A segment between waypoints is straight,
  unless the waypoints carry a slope (a heading), in which case the robot
  follows a smooth curve through them. Speed is limited automatically on tight
  curves, so neither wheel exceeds its limits.
- **Step strategy:** when step timings are computed.

| Strategy | How it works |
|---|---|
| `Buffered` (default) | Generates a short window ahead and discards the unsent part when a command changes the motion. |
| `Realtime` | Computes each step just before the hardware needs it. |
| `Precomputed` | Computes the whole move before it starts. |

Step timings are streamed to the hardware. The RP backend uses PIO and DMA,
so pulse timing never depends on the CPU, and both wheels stay synchronised
in time even on curves.

### The `robot!` block (planned)

```rust
capek::robot! {
    name: Ninja,

    commands {
        drive: DriveCommand,
    }

    components {
        drivetrain: Drivetrain {
            commands: drive,
            left: Stepper {
                steps_per_rev: 400,
                driver: StepDir { step: PIN_6, dir: PIN_5 },
            },
            right: Stepper {
                steps_per_rev: 400,
                invert: true,
                driver: StepDir { step: PIN_27, dir: PIN_28 },
            },
            geometry: Differential::new(56.mm(), 159.mm()),
            profile: SCurve { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2(), jerk: 8000.mm_per_s3() },
        },
        gripper: Gripper {                    // no commands: a direct field of the robot
            servo: Servo { driver: PwmServo { pin: PIN_15 } },
            open: 50.deg(),
            close: 153.deg(),
        },
    }

    sensors {
        proximity: Proximity {
            sensor: Ultrasonic { driver: Hcsr04 { trig: PIN_21, echo: PIN_19 } },
            filter: Ema::new(0.2),
            on: {
                Below(60.mm()) => drive.send(DriveCommand::Stop),
                Above(80.mm()) => drive.send(DriveCommand::Start),
            },
        },
    }
}
```

Uppercase identifiers such as `PIN_6` are peripherals. Nested `Type { .. }`
blocks are child components. Everything else is an ordinary Rust expression.
Platform resources such as PIO state machines are allocated automatically.
Everything the macro generates is public API, so you can also write the setup
by hand.

## Roadmap

- [x] **M0** Workspace scaffold and CI
- [ ] **M1** `capek-core`: units, filters, PID, profiles, waypoint routes, step generation, platform contract, base components, triggers
- [ ] **M2** RP platform crate: step/dir and four-wire steppers, servos, GPIO
- [ ] **M3** Command mutexes, step strategies and components
- [ ] **M4** `robot!` and `#[capek::main]`
- [ ] **M5** Link: transport trait, USB serial, `capek-proto`
- [ ] **M6** Port ninja to capek

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

# capek blueprint

This document describes how capek is put together and the order in which it is
being built. It is written for contributors, human or AI. The
[README](README.md) has the short overview. [AGENTS.md](AGENTS.md) has the
workflow and code rules.

The design is not frozen. When a decision changes, update this file in the same
commit as the code that depends on it.

## Contents

1. [Background](#background)
2. [Principles](#principles)
3. [Crates and modules](#crates-and-modules)
4. [Platforms, components and drivers](#platforms-components-and-drivers)
5. [Runtime model](#runtime-model)
6. [Command mutexes](#command-mutexes)
7. [Sensors and triggers](#sensors-and-triggers)
8. [Motion control](#motion-control)
9. [The `robot!` macro](#the-robot-macro)
10. [Link](#link)
11. [Extending capek](#extending-capek)
12. [Coming from ninja](#coming-from-ninja)
13. [Plan of action](#plan-of-action)
14. [Testing](#testing)
15. [Open points and assumptions](#open-points-and-assumptions)

## Background

capek grew out of [ninja](https://github.com/rurrobotics/ninja), firmware for a
Eurobot robot. Ninja's `master` branch runs on a Pico 2W (RP2350) and its
`hybrid` branch on a Pico W (RP2040). Ninja works well, and its PIO-and-DMA
stepper timing, acceleration profiles, postcard framing and `join`/`select`
strategies carry over into capek. Some things in ninja are hard to reuse:

- **Peripheral plumbing by hand.** The actuator task takes fifteen concrete
  peripheral arguments, and types such as
  `Drivetrain<'d, PIO1, 0, 1, PowerProfile, DMA_CH1, DMA_CH2>` are written out
  manually.
- **Board variants are forks.** The two branches differ only in target, boot
  code, pins and calibration, yet they are maintained separately.
- **One task owns every actuator,** because the link and the strategy both need
  them.
- **A flat `config.rs`** mixes chip setup, robot geometry, calibration and
  network credentials.
- **The protocol is shared through `include!`** between firmware and host.
- **Motion runs from one precomputed delay table per move.** Both wheels replay
  the same table, so the robot can only drive straight or spin. A move cannot
  be paused smoothly, and S-curve profiles are not possible.

capek's goal is that a robot's firmware becomes a short description of what is
wired where, plus the strategy code.

## Principles

- **Thin macros, plain output.** Everything `robot!` generates is public API you
  could write by hand. The macro saves typing; it does not hide behaviour.
- **Components are platform-neutral, drivers are per platform.** You configure
  components such as `Stepper` and `Servo`. Each platform supplies the drivers
  behind them. Switching platform changes pins, the feature flag and the
  target, not the robot's structure or strategy.
- **Components nest.** A `Drivetrain` is built from two `Stepper`s, a `Gripper`
  from a `Servo`.
- **Tasks only where concurrency is real.** Sensors, the link and components
  with a command mutex run as tasks. Everything else is a plain struct.
- **HAL-neutral core.** `capek-core` never depends on a chip HAL, so it builds
  and tests on a PC.
- **`f32` for physics, integers for counts.** The RP2040 has no FPU and the
  RP2350 only has an `f32` FPU. Step counts and timer ticks stay integers so
  nothing drifts.
- **Quantities in public APIs.** Physical values are `Length`, `Angle`, `Speed`
  and so on, never a bare `f32`, so a signature documents its unit.

## Crates and modules

| Crate | Role | Builds on host |
|---|---|---|
| `capek` | The crate users depend on. Re-exports the core, the macros and the selected platform. A feature such as `rp2040` selects the platform. | no |
| `capek-core` | Units, filters, PID, motion, command mutexes, driver traits, the `Platform` contract, components. | yes |
| `capek-rp` | RP2040 and RP2350 platform: drivers, chip setup, resource allocation, USB transport. | no |
| `capek-macros` | `robot!` and `#[capek::main]`. | yes |
| `capek-proto` | Wire types shared by firmware and host controllers. | yes |

Further platforms, such as `capek-stm32` or `capek-esp`, are added as crates of
their own, next to `capek-rp`.

Target layout of `capek-core`:

```
capek-core/src/
├── units.rs          Length, Angle, Speed, Acceleration, Jerk, UnitExt, Quantity
├── filter.rs         Ema, MovingAverage, Median, Biquad, Kalman, RateLimit, chaining
├── pid.rs            Pid with limits, anti-windup, derivative filtering
├── trigger.rs        Trigger<Q>: Below, Above, Changed, Error, Every
├── platform.rs       Platform trait: the contract every platform crate fulfils
├── driver.rs         StepperDriver, ServoDriver, StepWord
├── command/
│   ├── mod.rs        CommandMutex, Writer, Command, Status
│   └── policy.rs     Policy; PerWriterHold, LastWins, Priority
├── motion/
│   ├── profile/
│   │   ├── mod.rs    Profile, Plan, State
│   │   ├── constant.rs Constant
│   │   ├── trapezoid.rs Trapezoid, FastTrapezoid
│   │   ├── power.rs  Power, FastPower
│   │   ├── sine.rs   Sine
│   │   └── scurve.rs SCurve
│   ├── kinematics.rs Differential, Pose
│   ├── path.rs       Path<N>; Line, Turn
│   ├── route.rs      Waypoint, Route, Hermite segments
│   ├── planner.rs    speed planning along a path
│   ├── stepgen.rs    StepGen
│   ├── strategy.rs   StepStrategy: Buffered, Realtime, Precomputed
│   └── table.rs      raw StepWord replay
├── build.rs          Build, Ctx, Module
├── components/
│   ├── stepper.rs    Stepper<D: StepperDriver>
│   ├── servo.rs      Servo<D: ServoDriver>
│   ├── gpio.rs       Button<I>, Output<O>
│   ├── ultrasonic.rs Ultrasonic<O, I>, implements DistanceSensor
│   ├── drivetrain.rs Drivetrain<L, R>
│   ├── linear_axis.rs LinearAxis<S, H>
│   ├── gripper.rs    Gripper<S>
│   ├── proximity.rs  Proximity<S: DistanceSensor>
│   └── starter.rs    Starter<I>
└── behavior/         sequence, match clock
```

Target layout of the other crates:

```
capek/src/            facade: platform selection, component aliases, prelude
capek-rp/src/         Rp (impl Platform), RpCtx, chip setup, PioStepDir, PioFourWire,
                      PioServo, PwmServo, GPIO, UsbSerial
capek-macros/src/     robot! parser and expansion, #[capek::main]
capek-proto/src/      Envelope, Response, framing constants
examples/             ninja (RP2350) and hybrid (RP2040)
```

## Platforms, components and drivers

### Two layers

A **component** is what you write in a `robot!` block: `Stepper`, `Servo`,
`Button`, `Ultrasonic`, `Drivetrain`, and so on. Components live in
`capek-core`. They hold every piece of logic that does not depend on the
hardware:

- a `Stepper` knows its steps per revolution, microstepping, direction
  inversion, enable handling and position;
- a `Servo` knows its pulse range, the angle that range covers and how long it
  takes to travel;
- an `Ultrasonic` sensor knows how to time an echo and turn it into a distance.

A **driver** is the hardware primitive behind a component, supplied by the
platform. It is kept as small as possible:

```rust
pub struct StepWord(u32);   // bit 31 = direction, bits 0..=30 = delay in ticks

pub trait StepperDriver {
    const TICK_HZ: u32;
    fn set_enabled(&mut self, on: bool);
    fn free(&self) -> usize;                       // queue room, used by Realtime
    async fn push(&mut self, words: &[StepWord]);
    async fn finish(&mut self);                    // every queued step emitted
    fn halt(&mut self) -> u32;                     // hard stop; returns steps not emitted
}

pub trait ServoDriver {
    fn set_pulse(&mut self, width: Duration);
    fn set_enabled(&mut self, on: bool);
}
```

Simple I/O components use the `embedded-hal` and `embedded-hal-async` pin
traits as their driver interface, so `Button<I: InputPin + Wait>` and
`Output<O: OutputPin>` work with any HAL.

### Stepper drivers

Steppers are wired in two common ways, and every platform supplies a driver
for each:

| Driver | Wiring | Typical hardware |
|---|---|---|
| `StepDir` | a step pin and a direction pin, optionally an enable pin | A4988, DRV8825, TMC22xx in step/dir mode |
| `FourWire` | four coil pins, full-step or half-step sequences | 28BYJ-48 with a ULN2003, an H-bridge on a bipolar motor |

Both consume the same `StepWord` stream, so motion control does not care which
one is fitted. `StepDir` is the default.

### The `Platform` contract

Each platform crate implements one trait, which names its driver for every
component:

```rust
pub trait Platform {
    type Ctx: Ctx;
    type StepDir: StepperDriver;
    type FourWire: StepperDriver;
    type Servo: ServoDriver;
    type Input: InputPin + Wait;
    type Output: OutputPin;
    type UsbSerial: Transport;
}
```

A platform crate that leaves something out fails to compile in its own crate,
so the contract is checked rather than hoped for.

### Selecting a platform

The `capek` facade contains the only platform `cfg`. It derives the names users
see from the selected platform:

```rust
#[cfg(any(feature = "rp2040", feature = "rp2350"))]
pub type Selected = capek_rp::Rp;

type P = Selected;
pub type StepDir  = <P as Platform>::StepDir;
pub type FourWire = <P as Platform>::FourWire;

pub type Stepper<D = StepDir>                  = capek_core::Stepper<D>;
pub type Servo<D = <P as Platform>::Servo>     = capek_core::Servo<D>;
pub type Button                                = capek_core::Button<<P as Platform>::Input>;
pub type Output                                = capek_core::Output<<P as Platform>::Output>;
pub type Ultrasonic = capek_core::Ultrasonic<<P as Platform>::Output, <P as Platform>::Input>;
pub type Drivetrain<L = Stepper, R = Stepper>  = capek_core::Drivetrain<L, R>;
```

- `Stepper` is generic over its driver. Which driver it gets is decided by the
  `driver:` block in `robot!` (see below); in hand-written code it is
  `Stepper<StepDir>`, `Stepper<FourWire>` or `Stepper<my::Tmc2209>`.
- Exactly one platform feature may be enabled; `capek` keeps its compile-time
  check for that.

Every alias is a concrete type, so components owned directly by the robot
struct have nameable types.

### Configuring a component and its driver

Component fields and driver fields are visually separate. Component fields
sit directly in the block; the driver is a nested `driver:` block whose name
selects the driver kind:

```rust
left: Stepper {
    steps_per_rev: 400,
    invert: true,
    driver: StepDir { step: PIN_6, dir: PIN_5, en: PIN_10 },
},
conveyor: Stepper {
    steps_per_rev: 2048,
    driver: FourWire { coils: [PIN_2, PIN_3, PIN_4, PIN_5], mode: HalfStep },
},
lift: Stepper {
    steps_per_rev: 200,
    driver: StepDir { step: PIN_7, dir: PIN_8, pio: PIO1, sm: 3 },
},
```

- `StepDir` and `FourWire` are the facade's names for the selected platform's
  drivers. A custom driver is just another block, such as
  `driver: my::Tmc2209 { uart: UART0 }`.
- The macro builds the driver from its block, then passes it to the
  component together with the component's own fields. An unknown field is a
  normal compile error pointing at that field.
- Pins are platform-specific anyway, so drivers may also accept optional
  platform-only fields. On RP these are `pio`, `sm` and `dma`, and they belong
  inside the `driver:` block. When they are left out, the platform allocates
  the resources itself.
- Components with a single obvious driver, such as `Servo`, `Button` or
  `Ultrasonic`, still use a `driver:` block, so the split looks the same
  everywhere.

### The RP platform

- `PioStepDir` and `PioFourWire` drive steppers through PIO. Words reach the
  PIO through chunked DMA, so the CPU never times a pulse.
- `PioServo` and `PwmServo` drive servos, through PIO or a hardware PWM slice.
- `RpCtx` owns the PIO blocks, loads each PIO program once, and hands out state
  machines and DMA channels in order.
- Each driver declares how many state machines and DMA channels it needs.
  Generated code checks the total against the chip at compile time, so running
  out is a compile error rather than a panic at boot.
- Chip setup emits the RP2040 boot code or the RP2350 binary info.

## Runtime model

A capek robot has three kinds of parts at runtime.

**Owned components** are any components declared without `commands:`. They are
plain fields of the robot argument, and the strategy calls them directly, as
in ninja. Dropping one of their futures halts the hardware immediately. That
is the *hard stop*.

**Commanded components** are declared with `commands: <mutex>`. They run as
their own Embassy task and are driven only through that
[command mutex](#command-mutexes), so the strategy, sensor callbacks and the
link can all control them without sharing ownership.

**Sensors** run as tasks. They publish their latest value and fire
[trigger callbacks](#sensors-and-triggers).

```rust
#[capek::main]
async fn main(mut robot: Ninja) {
    robot.extension.home().await;      // owned component: a direct field
    robot.starter.wait().await;        // sensor topic
    strategy::game(&mut robot).await;
}

async fn game(r: &mut Ninja) {
    join(r.drive.forward(160.mm()), r.extension.extend()).await;
    r.drive.turn((-90).deg()).await;
    r.drive.follow(Route::from_slice(&[
        Waypoint::at(600.mm(), 0.mm()),
        Waypoint::with_slope(900.mm(), 300.mm(), 90.deg()),
    ])).await;
}
```

Whether a component is owned or commanded is a configuration choice. The same
`Drivetrain` works both ways; adding or removing `commands:` is the only
difference.

Shared state between tasks lives in `'static` topics built on `embassy-sync`,
generated by the macro into `mod topics`:

| Topic | Built on | Used for |
|---|---|---|
| `Watch<T, N>` | `embassy_sync::watch::Watch` | latest sensor value |
| `Signal<T>` | `embassy_sync::signal::Signal` | one-shot events, such as the starter cord |
| `CommandMutex<C, P>` | blocking mutex plus wakers | commands to a component |

## Command mutexes

A command mutex carries commands to one commanded component. It is declared in
the head of `robot!`, passed to the component, and written to by any number of
writers.

```rust
pub struct CommandMutex<C: Command, P: Policy = PerWriterHold>;
pub struct Writer<C>;                              // a writer identity plus the mutex

impl<C> Writer<C> {
    pub fn send(&self, cmd: C) -> Seq;             // synchronous, usable in callbacks
    pub async fn call(&self, cmd: C) -> Outcome;   // send and await completion
}

impl<C, P> CommandMutex<C, P> {                    // used by the component task
    pub async fn next(&self) -> Event<C>;
    pub fn report(&self, seq: Seq, status: Status);
}

pub trait Command { fn class(&self) -> Class; }   // Hold, Release, Abort or Goal
pub trait Policy  { fn decide(&mut self, writer: WriterId, class: Class) -> Decision; }
```

- `send` never blocks, so a sensor callback can call it.
- `call` sends a command and waits until the component reports it done.
  Dropping the `call` future sends `Abort`, so `select` and timeouts still
  cancel motion across the task boundary.
- The component reports a status that anyone can observe:
  `Idle`, `Running(seq)`, `Held(holders)` or `Done(seq, outcome)`.
- Every writer has its own identity. The macro assigns one to the strategy
  handle, to each sensor callback and to the link.

### Arbitration policies

When several writers share a mutex, the policy decides what happens. It is set
per mutex with `policy:` and defaults to `PerWriterHold`.

| Policy | Behaviour | Good for |
|---|---|---|
| `PerWriterHold` | `Stop` adds that writer's hold. `Start` removes only that writer's hold. Motion runs when no holds remain. New goals are accepted and wait while held. | Safety stops: the strategy cannot release a hold set by the proximity sensor. |
| `LastWins` | One slot. The newest command replaces the pending one. | Components with a single writer. |
| `Priority` | Each writer has a level. A command is accepted only at or above the current owner's level, until the owner releases. | Layered control, such as safety above link above strategy. |

A custom policy is any type implementing `Policy`.

### Drivetrain commands

```rust
pub enum DriveCommand {
    Stop,                   // Hold: decelerate and pause, keep the remaining path
    Start,                  // Release: continue the paused move from rest
    Abort,                  // decelerate and drop the move
    Forward(Length),
    Backward(Length),
    Turn(Angle),            // in place, positive is counter-clockwise
    Follow(Route),          // waypoints with optional slopes
    SetPose(Pose),          // define the field frame for routes
    SetProfile(DriveProfile),
}
```

The strategy handle wraps these in methods: `forward`, `backward`, `turn`,
`follow`, `stop`, `start`, `abort`. An owned `Drivetrain` has the same methods
directly. Other components get their own enums, for example
`GripperCommand { Open, Close, Angle(Angle) }` and
`LinearAxisCommand { Home, Extend, Retract, MoveTo(Length), Stop, Start, Abort }`.

## Sensors and triggers

A sensor task reads its hardware periodically, filters the reading, publishes
it to a `Watch`, and checks its triggers.

```rust
pub enum Trigger<Q> { Below(Q), Above(Q), Changed, Error, Every }
```

- `Below` and `Above` fire once when the value crosses the threshold, not on
  every sample. Using a lower `Below` and a higher `Above` gives hysteresis.
- `Error` fires when the sensor stops returning readings, for example an
  ultrasonic sensor with no echo.
- Callbacks run inside the sensor task. They must be synchronous and short.
  They usually call `send` on a command mutex writer.

```rust
proximity: Proximity {
    sensor: Ultrasonic { driver: Hcsr04 { trig: PIN_21, echo: PIN_19 } },
    filter: Ema::new(0.2),
    period: 60.ms(),
    on: {
        Below(60.mm()) => drive.send(DriveCommand::Stop),
        Above(80.mm()) => drive.send(DriveCommand::Start),
        Error          => drive.send(DriveCommand::Stop),
    },
},
```

`Proximity` accepts any component implementing `DistanceSensor`, so a
time-of-flight sensor can replace the ultrasonic one later.

## Motion control

Motion is the combination of three independent choices:

- a **profile** decides how fast to move along the path;
- a **path** decides where each axis is at each point along it;
- a **step strategy** decides when step timings are computed.

The time base shared by all axes is time itself, not steps. That is what makes
S-curve profiles and curved routes possible.

### Profiles

A profile plans motion along one coordinate: the distance travelled along the
path.

```rust
pub struct State { pub s: Length, pub v: Speed, pub a: Acceleration }

pub trait Profile: Clone {
    type Plan: Plan;
    fn plan(&self, from: State, distance: Length, end_speed: Speed) -> Self::Plan;
    fn stop(&self, from: State) -> Self::Plan;     // shortest stop the limits allow
    fn set_resolution(&mut self, resolution: Length) {}   // used by Fast profiles
}

pub trait Plan: Clone {
    fn advance(&mut self, dt: Duration) -> State;
    fn is_done(&self) -> bool;
}
```

Each profile is its own type with only the parameters it needs:

| Profile | Speed change | Computed |
|---|---|---|
| `Constant { speed }` | none: jumps straight to `speed` and stops dead | closed form |
| `Trapezoid { max_speed, accel }` | constant acceleration | closed form |
| `FastTrapezoid::new(max_speed, accel)` | constant acceleration | per-step recurrence |
| `Power { max_speed, accel, p }` | strong at rest, easing into cruise | numeric integration |
| `FastPower::new(max_speed, accel, p)` | strong at rest, easing into cruise | per-step recurrence |
| `Sine { max_speed, accel }` | acceleration rises and falls as a half sine | closed form |
| `SCurve { max_speed, accel, jerk }` | jerk-limited, seven phases | closed form |

- `max_speed` is the cruise speed the profile actually reaches, and `accel`
  is the peak acceleration. Deceleration mirrors acceleration.
- `Constant` suits motors that can start at full speed, such as a 28BYJ-48,
  and tests. It moves at `speed` for the whole distance, ignoring the start
  speed, and its final state reports the requested end speed capped at
  `speed`, so the next plan can pick up from it. `stop()` is immediate.
- `Power` follows ninja's power curve: speed over distance is
  `v = v∞·√(u / (1 + u^(p−1))^(1/(p−1)))` with `u = s·2·accel / v∞²`. It
  starts at full `accel` and eases into cruise; larger `p` eases later and
  more sharply. The curve only approaches `v∞`, so, like ninja, the ramp ends
  at `u = (p + 2) / 2`, and `v∞` is chosen so that this point is `max_speed`.
  Time along the curve has no closed form: with `σ = √u` it stays finite at
  rest, so durations come from Simpson's rule and `advance` integrates `σ`
  with fourth-order Runge-Kutta. A plan that starts above `max_speed` brakes
  at a constant `accel` until it meets the curve.
- `Sine` has no acceleration jumps at the ends of a ramp, without the cost
  of planning an S-curve. A half-sine ramp covers the same distance as a
  constant-acceleration ramp at `2·accel / π`, so it plans like a
  `Trapezoid` at that acceleration. It ignores the acceleration it starts
  from, so replanning mid-ramp makes acceleration jump.
- The `Fast` profiles run David Austin's per-step recurrence, which avoids
  square roots per sample. They step through the curve in distance increments
  of `resolution`, which the parent component sets from its geometry through
  `Profile::set_resolution` (one wheel microstep for a `Drivetrain`); the
  method does nothing on the other profiles. `advance` interpolates between
  increments. The resolution is a private field, so the `Fast` profiles are
  built with `new` and start at 0.1 mm until a parent sets it. `FastPower`
  uses ninja's variant of the recurrence, with a power term, and like `Power`
  brakes at a constant `accel` when it starts above `max_speed`.
- `State` includes acceleration, so `stop()` works from any point, including
  the middle of an S-curve ramp. That is what lets every profile pause and
  replan. `SCurve` is the one profile that starts from that acceleration, so
  it stays continuous through a stop or replan. Its building block is a
  closed-form ramp from any speed and acceleration to a speed at rest in
  acceleration, and the peak speed is found by bisection on ramp distances.
  A starting acceleration beyond `accel`, or a deceleration too strong to
  fade before the speed reaches zero, is clamped. A move too short to bring
  the acceleration to zero ends at its distance still accelerating.
- Profiles plan between non-zero speeds, which the route planner needs. The
  recurrences start from the step index that matches the start speed.
- The path coordinate only moves forward: distances and speeds are
  non-negative, and the path decides which way each axis turns.
- A plan always ends exactly at its distance. When the limits do not allow the
  requested end speed within that distance, it ends at the closest speed they
  allow, so a move never overshoots its target.

Ninja's two profiles map to `FastTrapezoid` and `FastPower` with `p` = 3. Ninja's
`max_speed` for its power profile is the asymptote `v∞`, so the same motion
needs `max_speed` set to about 0.96 times ninja's value when `p` is 3.

### Paths

```rust
pub trait Path<const N: usize> {
    fn length(&self) -> Length;
    fn axes(&self, s: Length) -> [Length; N];      // where each axis is at s
    fn end(&self) -> [Length; N] { self.axes(self.length()) }
}
```

`s` runs from zero to `length()`, which is never negative; the path decides
which way each axis moves. For a differential drive:

- `Line(Length)` drives straight; negative is backwards. It moves every axis
  together, so it is a `Path<N>` for any `N` and also serves single axes.
- `Turn::new(Angle, &Differential)` spins in place. It takes the geometry
  because each wheel's distance depends on the track width.
- `Route` follows waypoints.

`Differential` holds the drive geometry: wheel diameter and track width. It
converts between path positions and wheel rotations. Each wheel's `Stepper`
component converts rotations into steps, using its own steps per revolution
and microstepping. `Pose` is the drivetrain's odometry: it integrates the steps
actually made into x, y and heading. Routes are expressed in this field frame,
and `SetPose` defines it. Headings are counter-clockwise from the x axis and
wheel pairs are ordered `[left, right]`. Each odometry update is integrated as
an arc of constant curvature, which is exact for lines, turns and circular
arcs whatever the update rate.

The path coordinate `s` is the distance travelled by the robot's centre. For
an in-place turn it is the distance each wheel rolls.

A `Stepper` used on its own, outside a parent component, has its own profile
and strategy and moves by angle. Inside a `Drivetrain` or `LinearAxis`, the
parent drives it.

### Waypoint routes

```rust
pub struct Waypoint { pub x: Length, pub y: Length, pub slope: Option<Angle> }

impl Waypoint {
    pub const fn at(x: Length, y: Length) -> Self;
    pub const fn with_slope(x: Length, y: Length, slope: Angle) -> Self;
}

pub type Route = heapless::Vec<Waypoint, MAX_WAYPOINTS>;   // 16 by default
```

- The slope of a waypoint is the heading the robot should have when it passes
  through it.
- The segment between two waypoints is a cubic Hermite curve whose end
  tangents follow the waypoints' slopes. A missing slope takes the direction
  of the straight line between the two waypoints. A segment with no slope at
  either end is therefore exactly a straight line.
- At a waypoint without a slope where the route bends, the robot stops and
  turns in place. A waypoint with a slope is passed smoothly, without stopping.
- A route starts at the current pose. If the first segment does not leave
  along the current heading, the robot turns in place first.
- When a route is planned, each curved segment gets a small arc-length table:
  curvature and both wheel distances sampled along it. `axes(s)` is then a
  lookup with interpolation.

### Speed planning

On a curve the outer wheel moves faster than the centre. The planner keeps
both wheels within the profile's limits:

1. Sample the path and limit the centre speed at each sample to
   `v_wheel_max / (1 + |κ| · track / 2)`, where κ is the curvature.
2. Force the speed to zero at corners.
3. Run a forward pass and a backward pass to find the speed actually reachable
   at each sample, given the profile's acceleration and jerk limits.
4. Let the profile plan each piece between those junction speeds.

Profile limits are therefore real wheel limits, for straight lines, turns and
curves alike.

### Step generation

`StepGen` turns a plan and a path into step timings for every axis. It is
plain code and is tested on the host.

- Every time slice `dt`, 1 ms by default, it advances the plan, maps the path
  position to each axis, and converts that to a position in steps.
- A phase accumulator per axis computes the exact time of each step and emits
  one `StepWord` per step: bit 31 is the direction, bits 0 to 30 are the delay
  since the previous step, in timer ticks.
- A word is emitted only when a step happens, so slow motion simply
  accumulates time across slices.
- Positions are integer steps with carried remainders, so nothing drifts over
  long moves.
- All axes advance by the same `dt`, so they stay synchronised in time on
  curves.

### Step strategies

A step strategy decides when `StepGen` runs relative to the hardware.

| Strategy | Generates | When a command changes the motion | Reaction time | Memory |
|---|---|---|---|---|
| `Buffered { horizon }` (default, about 20 ms) | up to `horizon` ahead, into a ring per axis | discards the unsent words, rewinds `StepGen` to the last sent word and replans | one DMA chunk in flight | one ring per axis |
| `Realtime` | only when the driver's queue has room | replans immediately | the queue depth, at most 8 words on RP | none |
| `Precomputed { max_steps }` | the whole move before motion starts | discards the rest of the table and plans the stop or remainder on the spot | one DMA chunk in flight | `max_steps` words per axis |

Raw ninja-style tables remain available through `run_table(&words)`, which
replays them without pausing.

### Hardware side on RP

One PIO program serves every step/dir stepper. It pulls a word, sets the
direction pin from bit 31, pulses the step pin and waits for the delay. A
sentinel word raises an interrupt at the end of a move. Because the direction
travels with each word, a reversal lands on exactly the right pulse. The
four-wire driver turns the same words into coil patterns before they reach its
PIO program.

## The `robot!` macro

```rust
capek::robot! {
    name: Ninja,

    commands {
        drive: DriveCommand,                         // policy: PerWriterHold
    }

    components {
        drivetrain: Drivetrain {
            commands: drive,
            left: Stepper {
                steps_per_rev: 400,
                driver: StepDir { step: PIN_6, dir: PIN_5, en: PIN_10 },
            },
            right: Stepper {
                steps_per_rev: 400,
                invert: true,
                driver: StepDir { step: PIN_27, dir: PIN_28, en: PIN_12 },
            },
            geometry: Differential::new(56.mm(), 159.mm()),
            profile: SCurve { max_speed: 400.mm_per_s(), accel: 1200.mm_per_s2(), jerk: 8000.mm_per_s3() },
            strategy: Buffered { horizon: 20.ms() },
        },
        gripper: Gripper {                           // no commands: a direct field of the robot
            servo: Servo {
                min_pulse: 320.us(), max_pulse: 1200.us(), period: 1786.us(),
                driver: PwmServo { pin: PIN_15 },
            },
            open: 50.deg(),
            close: 153.deg(),
        },
        extension: LinearAxis {                      // no commands: a direct field of the robot
            stepper: Stepper {
                steps_per_rev: 400,
                driver: StepDir { step: PIN_17, dir: PIN_18, en: PIN_13 },
            },
            home: Button { driver: Input { pin: PIN_8, pull: Up } },
        },
    }

    sensors {
        proximity: Proximity {
            sensor: Ultrasonic { driver: Hcsr04 { trig: PIN_21, echo: PIN_19 } },
            filter: Ema::new(0.2),
            period: 60.ms(),
            on: {
                Below(60.mm()) => drive.send(DriveCommand::Stop),
                Above(80.mm()) => drive.send(DriveCommand::Start),
                Error          => drive.send(DriveCommand::Stop),
            },
        },
        starter: Starter {
            active: Low,
            input: Button { driver: Input { pin: PIN_16, pull: Up } },
        },
    }

    link: UsbSerial {},
}
```

### Parsing rules

A field value is one of three things, decided by its shape:

- A nested `Type { .. }` block is a child component, built first and passed to
  its parent. A `driver: Kind { .. }` block is the component's driver.
- An all-caps identifier such as `PIN_6`, `PIO1` or `DMA_CH1` is a peripheral
  and becomes `p.NAME`.
- Anything else is an ordinary Rust expression.

Driver fields go in the `driver:` block, component fields beside it, as
described in [Configuring a component and its driver](#configuring-a-component-and-its-driver).
Defaults fill in whatever is not listed.

`commands: <name>` refers to a mutex from the `commands` block and makes the
component a commanded one. Inside an `on` block, a mutex name refers to a
writer with its own identity.

### What it generates

- `mod topics`: one `CommandMutex` per declared command type, and one `Watch`
  or `Signal` per sensor.
- Interrupt bindings for exactly the peripherals in use, through the selected
  platform.
- One task per sensor, per commanded component and for the link. Each task
  takes the peripherals it needs and builds its part inside.
- The robot struct: owned components as direct fields, `Writer` handles for
  commanded components, sensor endpoints. Field types are the facade aliases,
  such as `Drivetrain` or `Gripper`, so they are always nameable.
- `init`, which sets up the chip through the selected platform, moves each
  peripheral where it belongs and spawns the tasks.
- The compile-time check of platform resources, such as PIO state machines.
- `RobotCommand`, the link's command enum.
- `#[capek::main]`, which wraps the Embassy entry point, runs the platform's
  chip setup, and hands the robot to your `main`.

Generated code only names paths under `::capek`, never a platform crate
directly. A pin used twice is a move error from the compiler at its second use.

## Link

The link task reads frames from a transport (USB serial first, Wi-Fi later),
decodes them with postcard, and sends the commands to the right mutex with its
own writer identity. The `Transport` trait and the link task are platform-
neutral; each platform supplies its USB serial transport.

The macro generates the command enum from the `commands` block:

```rust
pub enum RobotCommand {
    Drive(DriveCommand),
}
```

The host controller depends on the same types through `capek-proto`, which
replaces ninja's `include!` of the firmware's packet file.

## Extending capek

| You want to… | Do this |
|---|---|
| Use your own stepper or servo driver | Implement `StepperDriver` or `ServoDriver` and `Build`, then write `driver: YourDriver { .. }` in `robot!`. |
| Use your own profile, path or filter | Implement `Profile` and `Plan`, `Path<N>`, or `Filter`. |
| Write your own component | A struct generic over driver traits or other components, implementing `Build`. |
| Make a component commandable | Define a command enum that implements `Command`, and implement `Module` reading from a `CommandMutex`. |
| Change arbitration | Implement `Policy` and set `policy: YourPolicy` on the mutex. |
| React to sensor events | Use callbacks in `on`, or implement `Module` and publish your own topics. |
| Skip the macro | Write the topics, tasks and `init` by hand. |
| Add a platform | Create a crate with drivers and an `impl Platform`, then add one feature and one `cfg` line to `capek`. |

## Coming from ninja

| ninja | capek |
|---|---|
| `actuators/stepper.rs` | the `Stepper` component with RP's `PioStepDir` driver |
| `actuators/servo.rs` | the `Servo` component with RP's `PioServo` driver |
| `actuators/drivetrain.rs` | `Drivetrain`, combining two steppers with a profile, path and step strategy |
| `actuators/extension.rs` | `LinearAxis` |
| `actuators/gripper.rs` | `Gripper` |
| `profiles/trapezoid.rs` | `FastTrapezoid` |
| `profiles/power.rs` | `FastPower` with `p` = 3, with `max_speed` as the speed actually reached |
| whole-move delay tables | the `Precomputed` strategy, or `run_table` |
| `sensors/ultrasonic.rs` | the `Ultrasonic` component, waiting on edges instead of spinning |
| `sensors/proximity.rs` and the aborting `select` | `Proximity` triggers sending `Stop` and `Start`; the move finishes after release |
| `tasks/receiver.rs` and the global signals | the link task and `RobotCommand` |
| `tasks/actuator.rs` | gone: commanded components are tasks, owned ones are fields of the robot |
| `config.rs` | fields in `robot!` |
| `packet.rs` shared with `include!` | `capek-proto` |
| `master` and `hybrid` branches | two example crates differing in pins, calibration and the platform feature |

## Plan of action

Work happens on the `dev` branch, one small commit at a time. Every commit
builds, passes the checks in AGENTS.md, and is approved by the maintainer
before it is made. The messages below are proposals; the list may be refined
as work progresses.

### M0: Scaffold (done)

- [x] `chore: add capek-core and capek-proto crates`
- [x] `ci: add GitHub Actions workflow`
- [x] `docs: describe goals, crates and design in README`
- [x] `docs: add AGENTS.md and CLAUDE.md`

### M1: Core (in progress)

- [x] `feat(core): add Length and Angle quantities`
- [x] `docs: require rustdoc examples and quantity types`
- [x] `feat(core): add signal filters`
- [x] `feat(core): add PID controller`
- [x] `docs: add blueprint and restructure the design overview`
- [x] `docs: separate driver blocks in robot! configuration`
- [x] `feat(core): add Speed, Acceleration and Jerk quantities`
- [x] `feat(core): add profile traits and Trapezoid profile`
- [x] `feat(core): add Constant profile`
- [x] `feat(core): add FastTrapezoid profile`
- [x] `feat(core): add Power profile`
- [x] `feat(core): add FastPower profile`
- [x] `feat(core): add Sine profile`
- [x] `feat(core): add SCurve profile`
- [x] `feat(core): add differential-drive kinematics and odometry`
- [x] `feat(core): add Path trait with line and turn`
- [ ] `feat(core): add waypoint routes with slopes`
- [ ] `feat(core): add curvature-aware speed planner`
- [ ] `feat(core): add time-sliced step generator`
- [ ] `feat(core): add driver traits and the Platform contract`
- [ ] `feat(core): add Stepper and Servo components`
- [ ] `feat(core): add button, output and ultrasonic components`
- [ ] `feat(core): add sensor triggers`

### M2: RP platform

- [ ] `chore: add capek-rp crate and make capek a facade`
- [ ] `feat(rp): add RpCtx, chip setup and resource allocation`
- [ ] `feat(rp): add PIO step/dir stepper driver`
- [ ] `feat(rp): add PIO four-wire stepper driver`
- [ ] `feat(rp): add PIO and PWM servo drivers`
- [ ] `feat(rp): implement Platform for RP`
- [ ] `feat: add component aliases for the selected platform`
- [ ] `feat(examples): add stepper examples for RP2040 and RP2350`
- [ ] `ci: build capek-rp and the examples`

### M3: Commands and components

- [ ] `feat(core): add command mutex`
- [ ] `feat(core): add arbitration policies`
- [ ] `feat(core): add Build, Ctx and Module traits`
- [ ] `feat(core): add step strategies`
- [ ] `feat(core): add drivetrain component`
- [ ] `feat(core): add gripper component`
- [ ] `feat(core): add linear axis component`
- [ ] `feat(core): add proximity sensor`
- [ ] `feat(core): add starter sensor`

### M4: Macros

- [ ] `docs(examples): add hand-written expansion of a robot! block`
- [ ] `feat(macros): parse robot! blocks`
- [ ] `feat(macros): generate topics, tasks and init`
- [ ] `feat(macros): generate writers and trigger callbacks`
- [ ] `feat(macros): add #[capek::main]`
- [ ] `test(macros): add expansion snapshots`

### M5: Link

- [ ] `feat(proto): add envelope and framing`
- [ ] `feat(core): add transport trait and link task`
- [ ] `feat(rp): add USB CDC transport`
- [ ] `feat(macros): generate RobotCommand`
- [ ] `feat(proto): add host controller example`

### M6: Port ninja

- [ ] `feat(core): add sequence and match clock behaviours`
- [ ] `feat(examples): port ninja to capek`
- [ ] `feat(examples): port hybrid to capek`

### Later

- A second platform crate, to prove that switching is seamless.
- Following routes in reverse.
- Blending separate commands without stopping between them.
- Wi-Fi link through cyw43 and embassy-net.
- DC motors with encoders and a PID velocity loop.
- IMU and other I2C sensors.
- A `cargo-generate` project template.

## Testing

Host tests, run with `cargo test -p capek-core`:

- **Profiles:** limits are never exceeded, speed is continuous (except for
  `Constant`), `Sine` and `SCurve` acceleration is continuous, `SCurve` jerk
  stays bounded, `stop()` reaches rest from every phase, and each `Fast`
  profile stays within a few percent of its exact counterpart.
  `FastTrapezoid` and `FastPower` reproduce ninja's delay tables within
  tolerance.
- **Routes:** segments without slopes are exact straight lines, slopes give
  matching tangents at joints, corners stop and turn, and arc-length tables
  match numeric integration.
- **Planner:** no wheel exceeds its limits on tight curves, and corner speed is
  zero.
- **Step generation:** per-axis step totals are exact, wheel ratios on curves
  are right, long moves do not drift, and direction changes land on the right
  word.
- **Components with mock drivers:** `Stepper` converts rotations to steps with
  microstepping and inversion, and `Servo` maps angles to pulses.
- **Command mutexes:** the strategy's `Start` cannot release a hold set by the
  proximity sensor, goals wait while held, dropping `call` aborts, and the
  other policies behave as documented.
- **Strategies:** `Buffered` rewinds and replans correctly, and all three
  strategies end at the same position.
- **Drivetrain with mock drivers:** stopping mid-route and starting again
  completes the whole route, and odometry ends at the last waypoint.

CI runs formatting, clippy with warnings as errors, the tests, and cross-builds
for `thumbv6m-none-eabi` and `thumbv8m.main-none-eabihf`.

On hardware: block the ultrasonic sensor in the middle of a straight move and
in the middle of a curved route. The robot should decelerate, wait, resume and
still end on its mark. Compare the end positions of the three strategies, and
run the same strategy with a step/dir and a four-wire stepper.

## Open points and assumptions

- 1 ms slices and a 20 ms `Buffered` horizon should be fine on the RP2040 at up
  to about 2000 steps per second per axis. This is to be measured in M2.
- A 1 MHz tick gives a 31-bit delay range of about 35 minutes per step.
- `Precomputed` needs `max_steps × 4 bytes × axes` of RAM and is opt-in.
- The embassy-rp 0.10 PIO API must support driving the direction pin from the
  word, through `out pins` or side-set. This is to be confirmed in M2.
- Erasing PIO state machine and DMA channel types inside the RP drivers costs
  one `match` per DMA chunk. This is to be confirmed in M2.
- How the compile-time resource check is expressed generically, so the macro
  stays platform-neutral, is to be settled in M2 and M4.

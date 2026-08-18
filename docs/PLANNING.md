# UnrealVehicleDynamics planning and design

This document records the intended model boundaries, coordinate conventions, runtime behavior, and
implementation sequence. See [README.md](../README.md) for the project overview and quick start, and
[INSTALL.md](INSTALL.md) for host and editor setup.

UnrealVehicleDynamics is a vehicle-dynamics and controls simulator built around Unreal Engine and a
shared C++ dynamics core. It targets aerial and marine vehicles and integrates external control
systems, starting with ArduPilot SITL. The first version is a fixed-wing aircraft; boat support
follows.
The headless executable is named `uvd`, short for **Unreal Vehicle Dynamics**.

Unreal/Chaos owns the world, rigid-body motion, and collisions. A plain C++ core calculates the
vehicle forces and also runs headless for trim, linearization, scripted inputs, comparison, and
replay. Vehicle parameters and run settings live in data files so normal tuning does not require
editing the core or Unreal internals.

The aircraft model is checked against JSBSim before ArduPlane closes the loop in Unreal.
[UNREAL.md](UNREAL.md) covers the Cesium map, georeference, and Unreal project setup.
[AERO.md](AERO.md) covers the current fixed-wing work. [WATER.md](WATER.md) covers the later
boat path, starting with Unreal Water, Buoyancy, and Chaos before adding the hydrodynamics needed for
controls work.
[CPP_STYLE.md](CPP_STYLE.md) defines the controls-oriented C++ conventions used by the core.

The intended controls workflow is:

```text
configure -> trim -> linearize -> excite -> compare -> close the loop -> replay
```

## First target

Start with one air-started educational aircraft in an ISA atmosphere with optional constant wind.
It sends ideal center-of-mass truth to ArduPilot's JSON backend and supports scripted controls and
ArduPilot SITL. ArduPilot may still apply its own SITL sensor models after receiving that truth. The
first deliverable is a Development Editor plugin, a tiny test project, and a Cesium air-start map.
Each run config supplies its starting LLA.

No takeoff, landing gear, sensor noise, HIL, marine runtime, multiple vehicles, ROS, RPC, or hot
reloading yet. A packaged app can wait.

Unreal should run on Linux or macOS with UE 5.8 and a supported native compiler. UE 5.8.1 with
Xcode 26.6 is the first tested macOS baseline. Unreal runs on
the host. ArduPilot SITL runs in Docker and talks to Unreal over UDP. Both host platforms must pass
the same coordinate, timing, force, and networking checks.

## Repo layout

```text
core/       C++23 fixed-size Eigen math, aircraft equations, rigid dynamics and integrator
app/        uvd executable, run configuration, analysis, simulation and orchestration
unreal/     Cesium map and georeference, plugin lifecycle, Chaos callbacks and snapshots
ardupilot/  pinned ArduPlane container and live transport probe
verification/ scientific reference comparisons and convergence checks
```

`core/` has no Unreal headers, sockets, global filesystem state, or wall clock. CMake builds it on
its own. Unreal and the CLI compile the same source files as C++23. CMake sets
`CMAKE_CXX_STANDARD 23`; the Unreal module requests `CppStandardVersion.Latest`. Preflight records
the real compile command and verifies the C++23 features used by the shared code on Linux and macOS.
Every shared public header is compiled through both CMake and Unreal before either build is accepted.
The runtime uses fixed-size Eigen vectors and matrices; the three- and four-element types do not
allocate, and their dimensions are checked at compile time.

The first version has one aircraft equation set compiled into C++. JSON holds the parameters; it
isn't a language for inventing new models.

The application is split by substantial capability under `app/`: `main.cpp` declares commands,
dispatches them, and owns the external SITL process lifecycle, while configuration, simulation,
aircraft analysis, and comparison each have one cohesive
implementation file. `app.hpp` is private application wiring, not a public vehicle-model API. Add
to an existing capability until a genuinely separate subsystem, such as SITL orchestration, earns
its own module; do not create one file per command, class, or helper.

## Coordinates and units

The world frame is right-handed NED (`+X North`, `+Y East`, `+Z Down`). The body frame is
right-handed FRD (`+X Forward`, `+Y Right`, `+Z Down`). Core values use SI units and `double`.

The reference point is the configured center of mass, with constant mass and inertia in v1. Attitude
is an active Hamilton quaternion `(w,x,y,z)` from body to NED. Body angular rate `(p,q,r)`, forces,
and moments use FRD, with moments taken about the center of mass. Model wrenches exclude gravity and
contact so the active integrator adds each once. Gravity is `9.80665 m/s²`.

```cpp
struct RigidBodyState {
    Vector3 position_ned_m;
    Quaternion q_body_to_ned;
    Vector3 velocity_body_mps;
    Vector3 omega_body_radps;
};

struct BodyWrench {
    Vector3 force_body_N;
    Vector3 moment_body_Nm;
};
```

The NED origin is fixed by latitude, longitude, and MSL altitude. ArduPilot starts with the same
home. The Cesium level uses that point after converting MSL altitude to ellipsoid height with
`h_ellipsoid = H_MSL + geoid_undulation`.

### Unreal conversion

V1 uses one fixed Cesium georeference at the cartographic origin with identity transform, scale 100,
and no origin shifting or rebasing. At that origin Cesium's local Unreal frame is East, South, Up.
Define the world polar-vector map and body polar-vector map as:

```text
C = [ 0  1  0 ]       B = [ 1  0  0 ]
    [-1  0  0 ]           [ 0  1  0 ]
    [ 0  0 -1 ]           [ 0  0 -1 ]
```

`C` maps core NED into the Cesium Unreal world. `B` maps body FRD into the aircraft actor's local
Forward, Right, Up frame. Both are reflections, so axial vectors use `H_C = -C` and `H_B = -B`.

| Core value | Unreal/Chaos value |
|---|---|
| NED position or lever arm | `100 C p` cm |
| NED linear velocity or acceleration | `100 C v`, `100 C a` |
| body force in actor-local axes | `100 B F` kg-cm/s² |
| body angular rate or acceleration | `H_B omega`, `H_B alpha` |
| body moment in actor-local axes | `10000 H_B M` kg-cm²/s² |
| body inertia | `10000 H_B I H_B^T` kg-cm² |
| actor rotation matrix | `R_ue = C R_body_to_ned B` |

When the Chaos API expects world axes, rotate the body wrench into NED and use `C` for force and
`H_C` for moment. Apply net force at the center of mass and net torque separately. Don't add `r x F`
twice. Read the Chaos center-of-mass position, but use the body rotation as the aircraft attitude.

Tests cover basis vectors, round trips, quaternion/matrix agreement, cross products, power, and two
simple unit checks: 1 N on 1 kg for one second gives 1 m/s, and 1 N·m on 1 kg·m² for one second gives
1 rad/s.

## Time and aircraft model

Integer physics ticks are the clock. Seconds are calculated from the tick instead of accumulated.

```cpp
struct StepContext {
    uint64_t step_index;
    double fixed_dt_s;
    double t0_s() const { return step_index * fixed_dt_s; }
    double t1_s() const { return (step_index + 1) * fixed_dt_s; }
};

struct AircraftModelOutput {
    AerodynamicsOutput aerodynamics;
    PropellerOutput propulsion;
    BodyWrench total_wrench;
};

AircraftModelOutput evaluate_aerosonde(
    const RigidBodyState&,
    const AircraftEffectorState&,
    const AtmosphereSnapshot&,
    const AerosondeParameters&);
```

The aircraft, surfaces, propeller, and atmosphere are algebraic in v1. Model evaluation has no
hidden state, I/O, locks, random calls, or wall-clock dependence. Air data is recalculated for every
RK4 trial state.

The CLI integrates this contact-free rigid body:

```text
position_dot_ned  = R(q) velocity_body
q_dot             = 0.5 q [0, omega_body]
velocity_dot_body = force_body/m + R(q)^T [0,0,g] - omega_body x velocity_body
omega_dot_body    = I^-1 (moment_body - omega_body x (I omega_body))
```

Normalize trial quaternions before evaluating the model. Normalize and consistently sign the final
quaternion after each step. The CLI and Chaos should return the same wrench for the same state, but
their trajectories only need to converge within stated tolerances.

## What happens each tick

Keep the stages separate:

```text
raw ArduPilot PWM
  -> named normalized command
  -> physical surface angles and throttle
  -> aircraft model
  -> force and moment
```

Log each stage. The channel map holds the ArduPilot function, PWM min/trim/max, reversal, physical
direction, and limits.

For interval `[t_k, t_(k+1))`:

1. Choose one command from a scripted schedule or a valid PWM frame.
2. Hold it for the whole interval and every RK4 stage.
3. Convert it to physical effectors. Diagnostic tests may supply effectors directly.
4. The CLI samples atmosphere and forces at every RK4 stage. Chaos samples once at `t_k` and holds
   that wrench for the step.
5. Commit `x_(k+1)` once.
6. Build sensors from the new state and send/log them at `t_(k+1)`.

The project sends ideal truth at the JSON boundary. ArduPilot SITL may still apply its own sensor
models internally. The first simulated IMU location is the center of mass:

```text
gyro = omega_body_(k+1)
velocity_ned_k  = R(q_k)  velocity_body_k
velocity_ned_k1 = R(q_k1) velocity_body_k1
acceleration_ned = (velocity_ned_k1 - velocity_ned_k) / dt
specific_force_body = R(q_k1)^T (acceleration_ned - [0,0,g])
EAS = TAS sqrt(rho / 1.225)
```

This is an interval-average accelerometer sample stamped at the interval end. Supported level rest
reads `[0,0,-g]`; free fall reads zero. There is no packet at `t_0`; the first step produces the
first packet at `t_1`. Sensor errors, offsets, latency, GPS errors, magnetometer details, cameras,
and HIL in this project come later.

## ArduPilot JSON connection

The first version uses ArduPilot's JSON UDP backend, pinned to commit
[`e0652af4c0c9657c04672ed3e21b71de75c74763`](https://github.com/ArduPilot/ardupilot/tree/e0652af4c0c9657c04672ed3e21b71de75c74763).
The useful source files are
[`SIM_JSON.cpp`](https://github.com/ArduPilot/ardupilot/blob/e0652af4c0c9657c04672ed3e21b71de75c74763/libraries/SITL/SIM_JSON.cpp)
and its [protocol README](https://github.com/ArduPilot/ardupilot/blob/e0652af4c0c9657c04672ed3e21b71de75c74763/libraries/SITL/examples/JSON/readme.md).

SITL sends a binary PWM packet first, then waits for JSON. Instance 0 uses UDP port 9002. Accept only
the exact packet forms: 40 bytes with magic 18458 for 16 channels, or 72 bytes with magic 29569 for
32 channels. Decode bytes explicitly; don't cast a UDP buffer to a C++ struct. Lock replies to the
endpoint that sent the valid packet. The first Linux and macOS targets are little-endian.

Start SITL with the same resolved latitude, longitude, MSL altitude, and heading as the core NED
origin. Each reply sends local NED position and velocity, not another LLA conversion. Cesium uses
LLA to place the map and may calculate it again for display and checks, but it is not in the controls
feedback path. Always send EAS because the pinned JSON backend does not correctly infer nonzero-wind
airspeed from `velocity_wind` alone.

The wire record is one leading newline, the compact JSON object, and one trailing newline. The JSON
has no `frame_count` field:

```json
{"timestamp":0.016666666666666666,"imu":{"gyro":[0,0,0],"accel_body":[0,0,-9.80665]},"position":[0,0,0],"quaternion":[1,0,0,0],"velocity":[25,0,0],"airspeed":25,"no_time_sync":false,"no_lockstep":false}
```

The connection starts in `Discovering`, with the aircraft held at trim until the first valid frame.
During `Warm-up`, the trim command advances the aircraft while SITL's estimator, mode, arming, and
output settle. The intended released mapping is one new frame to one physics interval; a duplicate
receives the cached reply without advancing time. The U0 timing probe must prove Unreal can preserve
that mapping before closed-loop work begins. A malformed packet, skipped frame count, or timeout
moves the run to `Failed` without accepting another commanded interval.

The first frame count `c0` maps to interval 0. Frame `c0+k` maps to interval `k` and receives a reply
timestamped `(k+1)dt`. Before release, the trim command drives the plant even though incoming PWM is
recorded. The first frame accepted after release drives the next interval.

For `delta = uint32(new_count - last_count)`: 0 is duplicate, 1 is next, `2..2^31-1` is a forward
gap, `2^31` is invalid, and larger values are stale. Simulation time starts at zero for every run.
Startup and packet timeouts use monotonic wall time and never commit a partial step.

A small wrapper owns the controller session: it starts one fresh ArduPilot Docker container with an
isolated state volume, loads and reads back the parameters, watches MAVLink readiness, and collects
logs. The launcher gives the container a host address that reaches Unreal's UDP socket; the exact
Docker networking setup may differ between Linux and macOS. The Unreal plugin owns the JSON socket
and physics session. A narrow host control channel lets the wrapper request `Release` or `Fail`; the
plugin applies that request only at the next unique PWM boundary. Release requires the expected
mode, armed state, healthy estimator, verified parameters, and stable PWM near trim.

The ArduPlane `--rate` value must equal the reciprocal physics timestep. The first JSON discovery
packet may advertise the backend's 1200 Hz constructor default; release waits until later packets
advertise the configured rate. Test 60, 120, and 240 Hz, then keep the slowest rate that meets
convergence, real-time, and zero-drop checks. Every run uses a fresh SITL container; simulation time
never resets while retaining an old controller container.

Blocking UDP stays on a socket worker. A custom POD Chaos callback handles the bounded pre/post-step
exchange. Rendering reads the latest immutable snapshot.

## Unreal physics details

UE 5.8 source and the local smoke probe show that `AsyncPhysicsTickComponent` runs before simulation
and requires the dedicated physics-thread body handle; game-thread `FBodyInstance` access is invalid
there. The game thread is frozen, so it cannot block on UDP. Simulation code uses the internal async
physics handle. Chaos uses
mixed precision, supports rotational inertia through principal moments and a mass-frame rotation,
and uses scalar translational mass. That last limit prevents Chaos from representing a full coupled
6x6 mass matrix.

The first setup uses async fixed timestep, normal substepping off, and inertia conditioning off while
checking the model. Log requested and completed steps, dropped steps, callback time, and real-time
factor. Recheck these source findings in the first Unreal probe. Public background:
[coordinates](https://dev.epicgames.com/documentation/en-us/unreal-engine/coordinate-system-and-spaces-in-unreal-engine?application_version=5.8),
[sub-stepping](https://dev.epicgames.com/documentation/en-us/unreal-engine/physics-sub-stepping-in-unreal-engine?application_version=5.8),
and [units](https://dev.epicgames.com/documentation/en-us/unreal-engine/units-of-measurement-in-unreal-engine?application_version=5.8).

Async fixed physics is driven by elapsed Unreal time, not by PWM arrival, and may schedule several
steps during one rendered frame. U0 must therefore run 30/60/144 Hz rendering against 60/120/240 Hz
physics, then inject a render hitch and a delayed PWM. No stale command may produce an unlogged step.
If this fails, stop before U1 and choose measured command hold, explicit Chaos pacing, or a core-owned
plant with a kinematic Unreal body. Hard packet lockstep is not assumed until this probe passes.

## Config and run output

There are two authored JSON files. `aircraft.json` describes the reusable vehicle: its parameters,
channel map, limits, valid range, and source. `run.json` says what to do this time: which
aircraft to load, where to start, which frontend and rate to use, the atmosphere, controls, stop
condition, and tolerances. Its `world` section holds the
Cesium provider and asset IDs, origin latitude and longitude, MSL altitude, geoid separation, and
starting heading. The Cesium ion token comes from `CESIUM_ION_TOKEN` and is never written to a run
bundle.

Both use schema version 1. Unknown fields, duplicate keys, nonfinite values, bad ranges, and
unsupported versions fail validation. Paths are relative to the file that contains them.
Defaults are resolved once and written out. P0 checks in one complete aircraft, headless run, command
schedule, and schema set so the file contract is proven by an actual runnable example.

Keep the first dependency set small: Eigen for linear algebra, nlohmann/json and
nlohmann-json-schema-validator for files, CLI11 for commands, and Catch2 for tests. Pin exact
releases, source hashes, licenses, and the Unreal build settings in `dependencies.lock.json`.
Configuration and CLI code may use exceptions; the per-tick model and physics boundary may not
throw.

Each run looks like this:

```text
runs/<run-id>/
  manifest.json
  resolved_config.json
  initial_state.json
  signals.csv
  signals.json
  inputs/
  results/
  controller/
```

`inputs/` copies the source files and earlier results needed to rerun the case. `results/` holds trim,
linearization, or comparison output. Closed-loop runs put packet traces, parameters, MAVLink status,
stdout/stderr, and DataFlash logs in `controller/`.

The manifest records the machine, toolchain, UE and ArduPilot versions, fixed clock, warnings, stop
reason, and metrics. The bundle copies its authored inputs, while the manifest records the Git commit
and whether the working tree was dirty. Dirty runs are labelled `exploratory`.

Every signal row describes the state at tick `n` and the command, environment, and wrench used over
interval `[n-1,n)`. `initial_state.json` is tick 0; CSV rows start at tick 1. `signals.json` gives
units, frames, meaning, and sample timing. For a final state tick `N`, execute intervals `0..N-1` and
write rows `1..N`.

The logged wrench is the left-endpoint value used by Chaos and the first RK4 stage. Extra RK stages
can go in an optional debug trace. Altitude-dependent atmosphere follows the same rule.

## CLI

```text
uvd validate <run.json>
uvd simulate <run.json>
uvd trim <run.json>
uvd linearize <run.json>
uvd compare <run-directory> <reference-run-or-model>
uvd sitl <run.json>
uvd replay <run-directory>
```

Scripted excitation is a normal simulation with a command schedule. Trim writes an operating point.
Linearization differentiates the same one-step map using a three-value attitude error, not raw
quaternion components. `compare` aligns signal IDs and integer ticks and writes metrics and plots.
`sitl` starts the test Unreal project and one fresh managed ArduPlane session, then owns cleanup and
the final run bundle.

Replay runs the CLI from the saved initial state and accepted commands. Replaying a CLI run is a
normal replay; replaying an Unreal run is a cross-backend comparison. Comparisons line up integer
ticks and signal IDs. A different clock needs an explicit resampling rule.

## Build order

| Step | Work | Check |
|---|---|---|
| P0 | state, clock, schemas, examples, logs and CLI | implemented and passing |
| B0 | licenses and dependency lock | implemented and passing |
| A1-A4 | [aircraft model](AERO.md) | implemented; reference-model and numerical checks pass |
| B1 | Linux/macOS UE setup | UE 5.8 macOS build and smoke pass; Linux release host remains |
| G0 | [Cesium groundwork](UNREAL.md) | not yet accepted |
| U0 | Unreal mechanics and pacing probe | finite 120 Hz smoke passes; full mechanics/rate matrix remains |
| U1 | aircraft in Unreal | shared wrench path runs through Chaos; comparison not yet accepted |
| U2 | full Chaos and UDP exchange | live controller transport probe passes; Unreal transport remains |
| U3 | ArduPilot closed loop | pinned container and launcher pass independently; closed loop remains |

The end-to-end example is:

```text
validate -> trim -> linearize -> 1% elevator doublet -> compare -> SITL -> replay
```

The authoritative release evidence and the line between implemented and proven are in
[V1.md](V1.md).

## Later

Later work covers flight-data calibration and uncertainty, takeoff and landing, gear, ground effect,
global flights with a moving tangent frame, alpha-dot and other acceleration-dependent models, live
tuning, dynamic plug-ins, ROS/RPC, multiple vehicles, detailed sensor errors, cameras, hardware HIL,
different controller and physics rates, and marine runtime.

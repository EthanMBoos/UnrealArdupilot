# UnrealVehicleDynamics

UnrealVehicleDynamics is a vehicle-dynamics and controls simulator built around
Unreal Engine and a shared C++ dynamics core. It targets aerial and marine
vehicles and integrates external control systems, starting with ArduPilot SITL.
The first vehicle is a fixed-wing Aerosonde; boat support follows.

The headless `uvd` executable supports model evaluation, simulation, trim, and
linearization. Aircraft parameters and run settings live in JSON, so controls
work does not require editing the model source.

The headless tools compare the fixed-wing model with JSBSim, and Unreal runs it
with ArduPlane. See [PLANNING.md](docs/PLANNING.md) for the roadmap, and
[docs/AERO.md](docs/AERO.md) for the aircraft model.

## Architecture

The shared C++ core answers one question: **given the vehicle state, controls,
and environment, what forces and moments act on the vehicle?** It contains the
engineering equations. It does not render anything.

```text
controller command
        |
        v
shared C++ force model
        |
        | force + moment
        v
motion integrator
        |
        | new position, velocity and attitude
        +------> Unreal renders the vehicle
        +------> simulated sensors feed the controller
```

Unreal is the main simulation runtime, not a visualizer bolted onto a separate
solver. The shared C++ sources compile directly into the Unreal plugin. The
current path loads the run configuration and lets Unreal own the fixed physics
clock, world, actor, and collisions while the core supplies the vehicle math.
When Chaos owns motion, the Unreal component gives the core a state in standard
SI and NED/FRD coordinates on each physics tick. The core returns a body force
and moment and the component applies it once. ArduPlane receives the committed
state and returns PWM commands through its JSON backend.

The headless CLI replaces Chaos with the core's built-in motion solver. That
solver turns forces and moments into the next vehicle state using a standard
fourth-order Runge-Kutta method, commonly called RK4. With no engine startup, it
can quickly evaluate, simulate, trim, and linearize the same model. This is the
engineering test bench for the Unreal runtime: the same state reaches the same
force model, and the RK4 trajectory can be checked as the timestep is refined.

Chaos owns motion when its rigid-body model can represent the problem, as in the
first aircraft and boat demos. If an engineering model needs physics Chaos
cannot express, the core owns that vehicle's state and Unreal renders the
committed pose. The tool is still native to Unreal: configuration, controls,
sensors, maps, timing, logging, replay, and visualization stay in the same
workflow. Only the solver responsible for advancing the state changes.

Water is the hardest version of this contract and may justify a paper by itself.
The first demo uses one Unreal Water wave definition for both the visible
surface and buoyancy queries. Before that result is accepted, sampled water
heights must remain on the rendered surface through pause, reset, replay, and
different render and physics rates. A later seakeeping model will require richer
wave and hull physics; if stock Unreal Water cannot reproduce it, a
project-owned render path must consume the same saved wave components, phase,
and simulation timestamp as the force model. A visible crest where the physics
sees a trough is a failed simulation.

## Quick start

Install the host tools as described in [INSTALL.md](docs/INSTALL.md).
Build the headless simulator:

```bash
cmake -S . -B build
cmake --build build
```

Then run a case from the repository root:

```bash
build/uvd evaluate examples/run.json
build/uvd simulate examples/run.json --duration 10 --output runs/headless.csv
build/uvd trim examples/run.json
build/uvd linearize examples/run.json --output runs/linear-model.json
```

With Unreal Engine 5.8, Cesium for Unreal, Xcode on macOS, and Docker installed,
run the live Cesium/Chaos/ArduPlane case:

```bash
cp .env.example .env
# Put CESIUM_ION_TOKEN=... in .env, then:
./run.py
```

The launcher builds the shared dependencies and Unreal plugin, starts the
ArduPlane container, opens the full Unreal window, and cleans both processes up
when either exits. The Cesium token stays in the ignored local `.env` file.

The two lightweight numerical checks are:

```bash
python3 validation/timestep_convergence.py
python3 validation/jsbsim_compare.py
```

## Repository layout

```text
core/       fixed-wing equations, rigid-body dynamics, and integration
cli/        small headless engineering test bench
unreal/     Unreal host project and Cesium/Chaos/ArduPilot plugin
ardupilot/  ArduPlane container and FBWA startup helper
validation/ optional JSBSim comparison and timestep-convergence check
examples/   aircraft parameters and the v1 run configuration
docs/       aircraft, Unreal, water, and C++ design details
```

The v1 currently flies the Aerosonde locally with UE 5.8.1, Cesium, Chaos, and
ArduPlane stabilizing the aircraft in Fly By Wire A (FBWA) mode. FBWA exercises
ArduPlane's basic roll and pitch controllers without requiring a mission,
autonomous navigation, takeoff, or landing. The CLI compiles the same force
model and provides the fast headless path for model inspection and controls
work. This is a useful working simulation, not yet a claim of physical
Aerosonde fidelity; see
[ARDUPILOT.md](docs/ARDUPILOT.md) for the first engineering push.

## License

Project code and documentation use the [MIT License](LICENSE). Third-party
licenses and packaging boundaries are recorded in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

# UnrealVehicleDynamics

UnrealVehicleDynamics is a vehicle-dynamics and controls simulator built around
Unreal Engine and a shared C++ dynamics core. It targets aerial and marine
vehicles and integrates external control systems, starting with ArduPilot SITL.
The first vehicle is a fixed-wing Aerosonde; boat support follows.

The headless `uvd` executable supports validation, simulation, trim,
linearization, scripted excitation, comparison, and deterministic replay.
Aircraft parameters and run settings live in JSON, so controls work does not
require editing the model source.

The fixed-wing model is checked against JSBSim before ArduPlane closes the loop
in Unreal. See [PLANNING.md](docs/PLANNING.md) for the roadmap, and
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
solver. The shared C++ sources compile directly into the Unreal plugin. Unreal
loads the run configuration, owns the fixed physics clock, world, actors,
collisions, and controller connection, while the core supplies the vehicle
math. When Chaos owns motion, the adapter gives the core a state in standard SI
and NED/FRD coordinates on each physics tick. The core returns a body force and
moment; the adapter applies them once and records the command, force, and
resulting state together. Rendering reads that committed state; it never drives
the controls calculation.

The headless CLI replaces Chaos with the core's built-in motion solver. That
solver turns forces and moments into the next vehicle state using a standard
fourth-order Runge-Kutta method, commonly called RK4. With no engine startup, it
can quickly trim, linearize, excite, compare, and replay the same model and data.
This is how the Unreal result earns trust: the same state must produce the same
force and moment in both paths, trajectories must converge as the timestep is
refined, and changing render rate, adding Cesium scenery, or introducing a
render hitch must not silently change the physics.

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
mkdir build && cd build
cmake .. && make
```

Then run a case from the repository root:

```bash
build/uvd validate examples/runs/headless.json
build/uvd simulate examples/runs/headless.json
build/uvd trim examples/runs/headless.json
build/uvd linearize examples/runs/headless.json
```

Runs are written under `runs/` by default. Useful follow-up commands are:

```bash
build/uvd replay <run-directory>
build/uvd compare <left-run> <right-run-or-model>
```

## Repository layout

```text
core/       fixed-wing equations, rigid-body dynamics, and integration
adapters/   ArduPilot, Unreal/Chaos, and reference-data boundaries
unreal/     Cesium world, plugin lifecycle, physics, and snapshots
tools/      headless CLI and offline JSBSim checks
examples/   aircraft parameters, run configuration, and input schedules
docs/       aircraft, Unreal, water, and C++ design details
```

The headless fixed-wing path is implemented. Unreal, Cesium, Chaos,
ArduPilot JSON/UDP, and closed-loop integration are the next major stages.

## License

Project code and documentation use the [MIT License](LICENSE). Third-party
licenses and packaging boundaries are recorded in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

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
solver. The shared C++ sources compile directly into the Unreal plugin. The
current open-loop path loads scripted run configuration and lets Unreal own the
fixed physics clock, world, actor, and collisions while the core supplies the
vehicle math. When Chaos owns motion, the Unreal component gives the core a state in
standard SI and NED/FRD coordinates on each physics tick. The core returns a
body force and moment and the component applies it once. The closed-loop
controller connection and full per-tick evidence stream remain v1 work.

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

With Unreal Engine 5.8 installed, compile the plugin and run the finite local
Chaos smoke case:

```bash
build/uvd unreal examples/runs/unreal_smoke.json --preflight
build/uvd unreal examples/runs/unreal_smoke.json
python3 verification/unreal_u0.py --uvd build/uvd
python3 verification/unreal_u1.py --uvd build/uvd
python3 verification/unreal_u2.py --uvd build/uvd
python3 verification/unreal_u3.py --uvd build/uvd
```

The smoke command opens a visible air-started aircraft, runs scripted controls
for six simulated seconds, exits automatically, and saves an Unreal report and
engine log in its run bundle. The U0 command repeats the full Open World launch
for the mechanics and timing matrix; requested render caps and achieved frame
rates are both saved because the full scene may be GPU-limited below a cap.

Runs are written under `runs/` by default. Useful follow-up commands are:

```bash
build/uvd replay <run-directory>
build/uvd compare <left-run> <right-run-or-model>
```

## Repository layout

```text
core/       fixed-wing equations, rigid-body dynamics, and integration
app/        uvd executable, configuration, simulation, analysis, and SITL lifecycle
unreal/     Unreal plugin, open-loop Chaos runtime, and smoke evidence
ardupilot/  pinned ArduPlane container and live transport probe
verification/ JSBSim reference comparison and numerical convergence checks
examples/   aircraft parameters, run configuration, and input schedules
docs/       aircraft, Unreal, water, and C++ design details
```

The headless fixed-wing path, finite UE 5.8/Chaos open-loop smoke run, and local
U0 mechanics/timing and U1 model/trajectory suites are implemented. The
ArduPlane container now also completes a live 240-frame lockstep exchange with
Unreal, and the local U2 suite verifies duplicate, gap, stale, malformed, and
timeout handling in the real plugin. A readiness-gated local U3 flight now arms
ArduPlane in FBWA, releases stable PWM into Chaos, and passes a cross-backend
command replay. Cesium placement and the Linux acceptance repeat remain before
the project is a proven v1. See
[V1.md](docs/V1.md) for the exact evidence gate.

## License

Project code and documentation use the [MIT License](LICENSE). Third-party
licenses and packaging boundaries are recorded in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

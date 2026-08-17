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
in Unreal. See [PLANNING.md](docs/PLANNING.md) for the overall design and roadmap,
[docs/AERO.md](docs/AERO.md) for the aircraft model, and
[docs/CPP_STYLE.md](docs/CPP_STYLE.md) for the controls-oriented C++ conventions.

## Architecture

```text
aircraft parameters + run configuration
                  |
          shared C++ force model
             /           \
   headless rigid body   Unreal + Chaos
             |                 |
 trim, linearize, replay   ArduPilot SITL
```

The core uses SI units, NED world axes, FRD body axes, and fixed-size Eigen
math. Unreal/Chaos owns real-time rigid-body motion and collisions; the
headless CLI owns its own deterministic integrator for analysis and testing.

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

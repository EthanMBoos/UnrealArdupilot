# Repository instructions

## Engineering sources of truth

- Read `README.md`, `docs/AERO.md`, and `docs/CPP_STYLE.md` before changing the vehicle model.
- Keep the shared core in SI units with NED world axes and FRD body axes.
- Preserve conventional controls notation, explicit frame/unit suffixes, and the staged equation flow
  documented in `docs/CPP_STYLE.md`.
- Keep I/O, JSON, Unreal types, sockets, and process control out of `core/`.
- Use fixed-size Eigen objects in per-tick model code.

## Project shape

- `core/` owns reusable vehicle equations and the optional headless integrator.
- `cli/` owns the small `uvd` engineering interface.
- `unreal/` owns Cesium, Chaos, ArduPilot transport, and visual runtime concerns.
- `validation/` owns optional reference-library adapters and numerical checks.
- `ardupilot/` owns the v1 ArduPlane container and startup helper.

Do not add a framework merely to support one command or one configuration file. A direct parser,
ordinary JSON output, and focused script are preferable until a repeated need appears.

## Checks after C++ changes

Format the touched project files, configure CMake, build, and exercise the affected command:

```sh
cmake -S . -B build
cmake --build build
build/uvd evaluate examples/run.json
build/uvd simulate examples/run.json --duration 1
build/uvd trim examples/run.json
build/uvd linearize examples/run.json
```

For changes to integration or the aircraft equations, also run:

```sh
python3 validation/timestep_convergence.py
python3 validation/jsbsim_compare.py --samples 50
```

The JSBSim check is optional when its Python package is unavailable; report that clearly. For Unreal
runtime changes, build and launch with `./run.py` and inspect the actual editor/game log.

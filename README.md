# UnrealVehicleDynamics

A small fixed-wing simulation that runs ArduPlane against an aircraft in Unreal Engine and places
it with Cesium.

This is intentionally a v1, not a simulation framework. There is one aircraft, one run file, one
flight mode, and one launcher.

## What runs

```text
Unreal body position
    -> Cesium longitude / latitude / ellipsoid height
    -> local NED state
    -> Aerosonde force and moment model
    -> Unreal Chaos
    -> ArduPilot JSON state
    -> ArduPlane PWM commands
```

The aircraft starts 500 m above the configured origin on a trimmed air-start. Unreal sends that
state to ArduPlane while ArduPlane initializes. Once GPS and EKF are ready, the small container
helper selects FBWA, arms, and tells Unreal to use the ArduPlane outputs instead of the trim values.

Cesium is the source of the live LLA position. ArduPilot's JSON backend expects local NED, so the
plugin converts the Cesium LLA to NED once and gives that same state to both the force model and
ArduPilot. The Unreal log prints the LLA-to-NED result once per simulated second.

## Requirements

- Unreal Engine 5.8 (tested with 5.8.1)
- Cesium for Unreal installed in that engine (tested with 2.29.0)
- Xcode on macOS
- Docker Desktop
- CMake 3.28 or newer
- Python 3

The launcher uses the normal UE 5.8 install location on macOS. For another installation, set
`UVD_UNREAL_EDITOR` to the UnrealEditor executable.

For Cesium World Terrain, export a Cesium ion token before launching:

```sh
export CESIUM_ION_TOKEN=your_token
```

Without a token, the run still uses Cesium coordinates and displays Cesium's built-in ellipsoid
instead of ion terrain.

## Run it

From the repository root:

```sh
./run.py
```

The first run downloads Eigen, compiles the Unreal plugin, and builds ArduPlane in Docker. Later
runs reuse those builds. Close the Unreal window or press Control-C in the terminal to stop Unreal
and the container.

Edit [examples/run.json](examples/run.json) to change the origin, initial state, wind, trim, ports,
or Cesium asset ID. The corresponding aircraft constants and PWM channel map are in
[examples/aircraft/aerosonde.json](examples/aircraft/aerosonde.json).

## Code kept in v1

- `core/`: force model, coordinate conversion, and the few state helpers used at runtime
- `unreal/`: the host project and one simulation component
- `ardupilot/`: the ArduPlane image and the short FBWA startup helper
- `run.py`: build, launch, and cleanup
- `examples/`: the only aircraft and run configuration

There is deliberately no schema system, offline CLI, evidence bundle, replay framework, fault
injector, acceptance-gate matrix, or version-release process. Git history contains the earlier
verification work if any of it becomes useful later.

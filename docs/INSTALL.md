# Installation and use

There are two entry points. `uvd` is the fast headless engineering tool. `run.py` builds and launches
the complete Unreal, Cesium, Chaos, and ArduPlane simulation.

## Headless CLI

Requirements are CMake 3.28 or newer and a C++23 compiler. CMake downloads Eigen and nlohmann/json.

```sh
cmake -S . -B build
cmake --build build
```

From the repository root:

```sh
build/uvd evaluate examples/run.json
build/uvd simulate examples/run.json --duration 10 --output runs/headless.csv
build/uvd trim examples/run.json
build/uvd linearize examples/run.json --output runs/linear-model.json
```

`evaluate` reports air data, coefficients, and the aerodynamic, propeller, and total body wrench at
the run's initial state. Pass `--input sample.json` or `--input -` to evaluate another state and
command. `simulate` uses constant trim commands and the core RK4 integrator. Its JSON summary goes to
standard output; `--output` additionally saves the trajectory as CSV.

Two lightweight checks live outside the runtime:

```sh
python3 validation/timestep_convergence.py
python3 validation/jsbsim_compare.py --samples 50
```

The first needs only Python's standard library. The second is optional and needs the `jsbsim` Python
package. It compares the aerodynamic equations with the small JSBSim fixture under
`validation/reference/`; it does not make JSBSim a simulator dependency.

## Unreal v1

The working local baseline is:

- Unreal Engine 5.8.1
- Cesium for Unreal 2.29
- Xcode on macOS
- Docker Desktop
- Python 3

On macOS, make sure the full Xcode installation is active:

```sh
sudo xcode-select -s /Applications/Xcode.app/Contents/Developer
xcodebuild -version
```

Install Cesium for Unreal into UE 5.8 through Epic Games Launcher. The project expects the plugin to
be available to the engine; it is not copied into this repository.

For Cesium World Terrain, make a local token file once:

```sh
cp .env.example .env
```

Then edit `.env`:

```dotenv
CESIUM_ION_TOKEN=your_token
```

The file is ignored by Git and the token is not logged. Without it, Cesium coordinates still work
and the built-in ellipsoid is displayed instead of ion terrain.

Launch from the repository root:

```sh
./run.py
```

The launcher:

1. locates Unreal Editor 5.8;
2. configures CMake and stages Eigen for UnrealBuildTool;
3. compiles the editor target;
4. builds the ArduPlane Docker image if it is missing;
5. opens the full Unreal game window and starts the container; and
6. stops the remaining process when either side exits.

If Unreal is installed somewhere else, set `UVD_UNREAL_EDITOR` to its executable. Edit
`examples/run.json` for the origin, initial state, wind, trim, ports, or Cesium asset. Edit
`examples/aircraft/aerosonde.json` for the model parameters and PWM mapping.

## Source tools

The repository includes `.clang-format` and `.clangd`. Formatting the project-owned C++ is enough:

```sh
clang-format -i cli/*.cpp cli/*.hpp core/include/uvd/*.hpp core/src/*.cpp
```

Generated `build/`, Unreal `Binaries/`, `Intermediate/`, `Saved/`, staged Eigen headers, local runs,
and `.env` are ignored.

# Project plan

## Goal

Build a readable vehicle-dynamics and controls simulator in Unreal Engine for aerial and marine
vehicles. Unreal is the main runtime: it owns the world, visualization, interaction, and—when it is
appropriate—rigid-body motion. A shared C++ engineering core keeps the force models independent of
Unreal so they can be exercised quickly and compared with established tools.

The first end-to-end slice is one air-started Aerosonde controlled by ArduPlane over Cesium terrain.
The next major slice is marine dynamics. PX4, richer sensors, uncertainty studies, and
experiment-grade artifacts follow when a concrete study needs them.

The development loop is:

```text
configure -> inspect -> trim -> linearize -> simulate -> close the loop -> validate
```

## Architecture

```text
core/       reusable equations, frames, geodesy, rigid body, and RK4
cli/        small headless engineering interface
unreal/     Cesium/Chaos runtime and controller transport
ardupilot/  ArduPlane container and startup helper
validation/ optional adapters to reference libraries and numerical checks
examples/   the current aircraft and run data
docs/       model contracts and future plans
```

The force-model boundary is the important seam:

```text
state + command + environment -> force + moment + diagnostic outputs
```

`core/` contains that seam and no engine types, sockets, JSON, process launching, or rendering. Both
the Unreal plugin and `uvd` compile these sources. They do not communicate through a subprocess or
live CLI pipe.

For the current aircraft, Unreal converts the actor state into NED/FRD SI values, calls the core,
and applies the resulting body wrench to Chaos. The CLI replaces Chaos with RK4. That makes the CLI
fast and mathematically convenient without turning it into the production runtime.

If a later model needs motion Chaos cannot express faithfully, the model may use a core-owned
integrator inside the plugin and drive a kinematic Unreal pose. This is expected for advanced
seakeeping work. Configuration, time, controls, sensors, and visualization still remain in the
Unreal workflow.

## Why keep the CLI

The CLI is the engineering test bench for work that is awkward inside an editor:

- evaluate a force model at a precise state;
- solve a steady operating point;
- derive a local linear model for controls analysis;
- integrate a trajectory quickly at several timesteps; and
- adapt the model to JSBSim, Marine Systems Simulator, or another reference implementation.

It is not the launch authority for Unreal and it does not need schemas, dependency injection,
evidence-bundle types, or a general experiment engine today. Add a CLI command when it exposes a
useful operation on the shared model. Put research-specific comparison logic in `validation/` so an
industry library never becomes a runtime dependency.

The first command set is intentionally small:

```text
uvd evaluate <run.json> [--input sample.json|-]
uvd simulate <run.json> [--duration s] [--dt s] [--output trajectory.csv]
uvd trim <run.json> [--output trim.json]
uvd linearize <run.json> [--output model.json]
```

Ordinary JSON and CSV are sufficient interfaces for Python, MATLAB, Julia, and plotting tools.

## Frames and timing

The controls-facing world frame is North-East-Down. The body frame is Forward-Right-Down. Values
use SI units and names carry frame or unit suffixes where ambiguity is possible. Quaternions are
Hamilton `(w,x,y,z)`, active, body-to-NED.

Cesium provides WGS84 longitude, latitude, and ellipsoid height. The Unreal plugin converts the live
Cesium position to local NED for the model and ArduPlane. The configured geoid undulation bridges
ellipsoid and MSL altitude.

Unreal owns the fixed aircraft physics interval. Controller output is held until a new accepted PWM
frame is available. The headless integrator uses an explicit timestep, reevaluates forces at each
RK4 stage, and has no render clock.

Perfect bitwise agreement between Chaos and RK4 is not a goal. The useful invariants are that the
same model input gives the same wrench, both solvers remain finite and plausible in the declared
envelope, and RK4 error shrinks as its timestep is refined.

## Fixed-wing progression

### F0 — working v1

- public BYU/MAVSim educational Aerosonde coefficient and propeller model;
- SI/NED/FRD shared core;
- Cesium coordinate path and chase camera;
- Chaos body with ArduPlane JSON/PWM loop;
- readiness-gated FBWA handoff;
- headless evaluate, RK4 simulation, trim, and linearization; and
- lightweight timestep and optional JSBSim checks.

This slice works locally. It is a useful controls simulation, not physical validation of a named
airframe.

### F1 — experiment quality

Add only what the first publishable aircraft experiment needs:

- actuator dynamics and explicit delays;
- sensor noise, bias, rate, and latency;
- repeatable wind and turbulence inputs;
- a recorded command/state log with enough metadata to reproduce a run;
- open-loop excitation and comparison utilities; and
- measured runtime timing behavior under selected render loads.

This is where a small case format or run manifest may become worthwhile. It should grow from actual
experiments, not recreate a generic workflow system in advance.

### F2 — external and physical validation

- broaden JSBSim comparisons beyond the mirrored coefficient fixture;
- compare against another independent six-degree-of-freedom implementation;
- calibrate uncertain parameters against flight or wind-tunnel data;
- report sensitivity and uncertainty rather than a single trajectory; and
- define claims narrowly enough that the evidence truly supports them.

### F3 — additional controllers and vehicles

Add PX4 through a controller adapter once the state/sensor contract is mature. Add another aircraft
only after vehicle-specific data can live cleanly outside the fixed-wing equations.

## Marine progression

Marine work uses the same architectural seam but raises a stricter rendering/physics question: the
surface seen by the force model must be the surface rendered by Unreal.

1. Start with flat-water buoyancy and static equilibrium in Unreal Water/Chaos.
2. Add simple regular waves, measuring whether visual and queried heights stay aligned.
3. Compare restoring and damping behavior with analytical results and an established marine
   library.
4. Introduce a core-owned time-domain seakeeping solver when stock buoyancy is no longer adequate.
5. Drive Unreal from the committed solver pose and render the same saved wave components, phases,
   and timestamp used by the forces.
6. Add propulsion, current, nonlinear drag, contact, and maneuvering one effect at a time.

[WATER.md](WATER.md) contains the detailed theory and validation ladder.

## Validation strategy

Validation is layered rather than branded as one universal gate:

1. **Equation checks:** dimensions, signs, limiting cases, and finite behavior.
2. **Numerical checks:** trim residuals, local linear response, conservation where applicable, and
   timestep refinement.
3. **Reference-library checks:** compare like-for-like subsets with JSBSim or a marine library.
4. **Runtime checks:** confirm Unreal frame conversions, applied forces, controller timing, and
   Cesium placement.
5. **Physical checks:** compare against flight, tank, sea-trial, or published experimental data.

Agreement with a fixture that mirrors the same published equations catches implementation errors;
it is not independent physical validation. A visually convincing Unreal run catches integration and
presentation failures; it is not numerical validation. Both are useful when described honestly.

## Dependency posture

The runtime stays small: Eigen in the shared core, nlohmann/json in the CLI, Unreal/Cesium for the
visual host, and the ArduPlane container for v1 controls. Reference libraries remain optional. Exact
versions should be recorded for a published experiment, but normal development does not need a
lockfile or release process for every local tool.

## Near-term work

1. Keep the v1 flight easy to launch and visually inspect.
2. Add a real aircraft mesh and verify its scale, center of mass, collision, and body-axis alignment.
3. Exercise CLI trim away from the saved operating point and add a small scripted perturbation.
4. Record enough Unreal and ArduPlane state to plot one closed-loop flight alongside headless model
   evaluations.
5. Decide the first research question before expanding sensors, faults, replay, or release tooling.

# ArduPilot direction

The first product push is making this simulator useful to ArduPilot fixed-wing
engineers. The Aerosonde is a working example, not the product. The product is a
short path from an engineer's aircraft knowledge to a repeatable controls
experiment:

```text
aircraft evidence -> C++ plant -> headless checks -> ArduPlane experiment
                                                    |
                         DataFlash + simulator truth + report
                                                    |
                                      real-flight comparison
```

An engineer should be able to bring an aircraft definition or plant, a Docker
image containing their ArduPlane build, and their parameters; connect the tools
they already use; run a repeatable experiment; and leave with native ArduPilot
logs beside simulator truth.

[AERO.md](AERO.md) documents the current fixed-wing equations and validation.
[PLANNING.md](PLANNING.md) covers the broader multi-vehicle roadmap. This
document owns the ArduPilot workflow and near-term implementation order.

## Current state

The v1 runs the complete path:

```text
Cesium LLA -> NED/FRD state -> shared C++ model -> Unreal Chaos
                    ^                              |
                    |                              v
               ArduPlane JSON <------------- PWM commands
```

`run.py` builds the project and bundled ArduPlane image, opens the full Unreal
window, waits for GPS and estimator readiness, enters Fly By Wire A (FBWA),
arms, and releases the aircraft. FBWA exercises the basic roll and pitch
controllers without first requiring a mission, autonomous navigation, takeoff,
or landing.

The `uvd` CLI compiles the same force model and supports evaluation, RK4
simulation, trim, and linearization. It is the fast path for checking a plant
before putting ArduPlane around it.

The current loop is still an in-repository demo. Its plant interface is shaped
around `AerosondeParameters`; the launcher assumes one image, parameter file,
mode, and RC sequence; startup logic lives in the bundled image; external GCS
access is not exposed; and runs do not yet preserve DataFlash beside structured
truth.

## Aircraft and plant contract

Users will normally bring their own aircraft. Useful evidence may come from
equations, coefficient tables, geometry tools, CFD, wind-tunnel or propulsion
tests, an existing model, or flight logs. No particular upstream tool is
required.

All supported runtime plants are reviewable C++ committed in this repository.
Reference tools may generate validation cases, but a supported run does not
depend on MATLAB, an external flight-dynamics process, a proprietary plant
binary, or a co-simulation bridge.

Two entry paths are enough initially:

- A conventional fixed wing can use the current equation form with different
  mass, inertia, geometry, derivatives, propulsion, and actuator data.
- A different model can implement the shared plant boundary:

```text
vehicle state + physical effectors + environment
    -> body force + body moment + diagnostics
```

The same plant compiles into the CLI and Unreal plugin. ArduPilot transport,
frames, Cesium placement, logging, and experiment control remain independent of
the selected plant.

The core uses SI units, North-East-Down world axes, and Forward-Right-Down body
axes. Each aircraft must make its reference dimensions, moment reference, CG,
inertia convention, thrust line, control signs, and visual transform explicit.
A practical package can remain ordinary source and data:

```text
aircraft/my_aircraft/
  MyAircraftPlant.hpp
  MyAircraftPlant.cpp
  aircraft.json
  ardupilot.parm
  MODEL.md
  evidence/
  validation/
  visual/
```

Before ArduPlane is attached, the CLI should check frames and units, mass and
inertia, control direction, representative coefficient cases, trim, stability,
time-step convergence, actuator limits, propulsion behavior, and declared model
envelope. The result should show values and residuals, not only pass or fail.

## ArduPlane contract

Docker is the only supported ArduPlane execution boundary. The project must
never assume a host-built executable. The bundled image remains the default,
while an engineer may supply an image and the command that starts ArduPlane
inside it:

```sh
./run.py --ardupilot-image my-team/ardupilot:controller-branch \
  --arduplane-command /workspace/build/sitl/bin/arduplane \
  --params my-plane.parm
```

These options describe the intended interface; the current launcher does not
yet implement them.

The supplied image only needs a runnable ArduPlane SITL binary. It does not need
project scripts or a project-specific entrypoint. A supervisor owned by this
repository runs outside the image, mounts inputs and a writable run directory,
configures the JSON and MAVLink addresses, waits for requested readiness,
drives the experiment, and shuts both sides down cleanly.

The project should support a small tested window across several stable
ArduPlane release families, not require the newest release. At startup it should
detect and record the firmware version and available capabilities. A scenario
that needs a newer feature should say so clearly; native system identification
is useful when present but is not required for the basic FBWA workflow.

ArduPilot parameters stay in ArduPilot's format and are interpreted by the
supplied firmware. Begin with that firmware's defaults, apply the user's
overlay, report rejected names, and save the resolved set. Clean and persistent
virtual EEPROM behavior must be explicit so an old run cannot silently change a
new one.

Mission Planner, QGroundControl, MAVProxy, or another normal MAVLink ground
station should be able to connect during a run. The supervisor automates
startup and requested inputs; it does not replace or monopolize the GCS.

The JSON boundary maps PWM to physical effectors. It must expose and validate
servo functions, reversal, trim, travel, mixer behavior, rate, lag, deadband,
saturation, packet timing, and lost or duplicate frames. Those details are part
of the plant experiment, not hidden transport behavior.

## Engineering workflow

The first standard experiments should be repeatable FBWA roll and pitch
sequences. They exercise the main fixed-wing control behavior without making
takeoff, landing, terrain contact, or mission behavior prerequisites.

A small scenario needs only an initial state, mode, timed commands, wind,
duration, and a few assertions. The same scenario should eventually run through
either host:

```sh
./run.py --headless --params my-plane.parm
./run.py --unreal --params my-plane.parm
```

Headless ArduPlane execution will connect its JSON backend to the
CLI's RK4 solver for fast iteration and CI. Unreal remains the interactive,
geospatial, collision, and future perception runtime. Both call the same plant,
although Chaos and RK4 are not expected to produce identical trajectories.

Each run should leave one self-contained directory:

```text
runs/<run-id>/
  run.json
  summary.json
  report.html
  truth.csv
  events.csv
  ardupilot/
    supplied.parm
    resolved.parm
    flight.BIN
    telemetry.tlog
```

The summary records the repository commit, plant, scenario, random seed,
physics step, ArduPilot image and digest, firmware identity, parameters, timing,
events, and stop reason. Truth includes state and air data, PWM and physical
effectors, forces and moments, wind, and plant envelope diagnostics.

The first report should show commanded versus actual response, ArduPilot
estimate versus truth, PWM versus physical effector, air data and energy,
saturation or failsafes, timing health, and envelope violations. It should
compare two firmware images, parameter sets, or plant revisions. Existing
ArduPilot tools remain the right place for general DataFlash analysis; this
project adds synchronized truth and reproducible cross-run comparison.

Real flight provides the final comparison. An engineer can repeat the same
input, align the DataFlash response with simulation, update the C++ plant, and
validate the change against maneuvers that were not used for fitting.

## Unreal engineering view

Unreal should show spatial truth and discrepancies, not become another ground
control station. The first useful view needs:

- chase, orbit, free, and onboard cameras with one-key aircraft reacquisition;
- a compact strip for simulation time, real-time factor, link age, mode,
  arming, estimator health, wind, airspeed, angle of attack, sideslip, and
  clearly labeled altitude references;
- optional body axes, wind and velocity, force vectors, flight trail, surface
  saturation, mission, and terrain context; and
- a translucent ArduPilot-estimated aircraft over simulator truth, making
  estimator position and attitude error visible.

Generic plots belong in existing analysis tools and the post-run report. A
small live panel may show the response for the active experiment. Later replay
should render recorded truth rather than rerun Chaos and assume it reproduces
the same state.

## First feedback release

An outside fixed-wing engineer can:

1. add a parameter-compatible aircraft or a C++ plant without changing
   ArduPilot transport or the Unreal runtime;
2. provide a Docker image from a tested ArduPlane release family and a parameter
   overlay;
3. connect their normal GCS;
4. run and repeat a documented FBWA response case; and
5. inspect DataFlash, telemetry, simulator truth, and a comparison report.

The strongest first demonstration is one FBWA excitation run against two
firmware images or parameter sets, with enough recorded state to reproduce and
explain the difference.

## Implementation order

1. Put the Aerosonde behind a selectable plant boundary shared by the CLI and
   Unreal.
2. Accept an engineer-owned ArduPlane image and move supervision outside the
   bundled image. Detect firmware version and capabilities.
3. Preserve synchronized DataFlash, telemetry, parameters, truth, events, and
   run metadata. Expose normal GCS access.
4. Add repeatable FBWA roll and pitch scenarios, a focused report, and run
   comparison.
5. Add the minimal Unreal cameras, status, spatial diagnostics, and
   truth-versus-estimate view.
6. Connect the same scenario and logging contracts to the headless RK4 loop.
7. Add flight-response correlation and optional system-identification support
   when an early user needs it.

The plant boundary and container workflow can progress independently, then meet
in the first feedback release.

## Early feedback

Ask early users what aircraft evidence they already have, which ArduPlane
versions and build practices they use, what first engineering decision the
simulation must support, which logs and plots they inspect, which truth signals
would help, and what experiment they want to run next. Those answers should
choose the next model check or workflow feature.

## Deferred

The first release does not need PX4 or marine integration, a large aircraft
catalog, automatic model generation, external runtime plants, ROS 2 as a
dependency, another GCS inside Unreal, a generic dashboard, a sensor catalog,
photoreal weather physics, multi-vehicle simulation, a graphical scenario
editor, a large firmware matrix, or takeoff and landing before ground contact is
credible.

## References

- [ArduPilot simulation overview](https://ardupilot.org/dev/docs/simulation-2.html)
- [ArduPilot JSON simulator protocol](https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON)
- [Using SITL and connecting a GCS](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html)
- [ArduPilot AutoTest](https://ardupilot.org/dev/docs/the-ardupilot-autotest-framework.html)
- [ArduPilot Plane system identification](https://ardupilot.org/plane/docs/common-systemid-mode-operation.html)
- [ArduPilot servo functions](https://ardupilot.org/plane/docs/servo-functions.html)
- [ArduPilot DataFlash logs](https://ardupilot.org/plane/docs/common-logs.html)
- [ArduPilot WebTools](https://ardupilot.org/dev/docs/common-webtools.html)
- [ArduPlane release notes](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/ArduPlane/ReleaseNotes.txt)
- [ArduPilot CI containers](https://github.com/ArduPilot/ardupilot_dev_docker)

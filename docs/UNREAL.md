# Unreal setup

Unreal is the main visual and closed-loop runtime. Cesium supplies the geospatial position, Chaos
advances the actor, the shared C++ core supplies force and moment, and ArduPlane supplies controls.

```text
run config LLA + air-start state
    -> Cesium georeference and aircraft actor
    -> live Cesium LLA converted to local NED/FRD
    -> shared force and moment model
    -> Chaos physics
    -> committed state sent to ArduPlane
    -> PWM command for the next interval
```

Use `./run.py` to build and launch the supported path. The launcher opens the full game window, not a
headless or cropped render. It also starts the ArduPlane container and shuts both processes down
together.

## Starting location and Cesium

The run file provides latitude, longitude, MSL altitude, geoid undulation, heading, and a local NED
air-start. Cesium height is measured from the reference ellipsoid while ArduPilot home altitude is
MSL. With signed undulation `N = h_ellipsoid - H_MSL`, the placement is:

```text
h_ellipsoid = H_MSL + N
```

V1 uses one fixed `CesiumGeoreference` at the configured origin. At that origin Cesium's Unreal axes
are East, South, Up, while the core uses North, East, Down. The simulation component owns that
conversion and Unreal-centimetre/metric conversion; controls code sees only NED/FRD and SI units.

The launcher reads `CESIUM_ION_TOKEN` from the ignored repository-root `.env` file. With a token, it
loads the configured ion tileset. Without one, the same coordinate path runs against Cesium's
built-in ellipsoid.

The aircraft starts above the origin and does not wait for terrain streaming. Terrain is scenery in
v1; takeoff, landing, and collision behavior are future work.

## Aircraft visual

Chaos uses a hidden simple body whose mass and inertia come from the aircraft file. The visible
mesh is a non-colliding child and does not change the vehicle dynamics. With no visual
configuration, Unreal builds the included generic fixed-wing shape using the aircraft span and
chord.

To use another static mesh, import a GLB or FBX into the Unreal project and add this optional block
to the aircraft JSON:

```json
"visual": {
  "asset": "/Game/Aircraft/MyAircraft/SM_MyAircraft.SM_MyAircraft",
  "scale": 1.0,
  "rotation_rpy_deg": [0.0, 0.0, 0.0],
  "offset_body_m": [0.0, 0.0, 0.0]
}
```

Use the Static Mesh asset's copied object path. The offset is expressed in the core's FRD body
frame from the vehicle CG to the mesh origin. Rotation aligns the imported mesh with Unreal body
axes: X forward, Y right, and Z up. If the asset cannot be loaded, the generic aircraft remains the
fallback. Aircraft-specific contact geometry and animated control surfaces are separate future
options, not requirements for using a plant.

## Runtime ownership

For the aircraft, Chaos owns position, velocity, attitude, collision, and the fixed physics step.
Each interval the plugin:

1. reads the current Unreal/Cesium pose and Chaos velocities;
2. converts them to the core's NED/FRD state;
3. evaluates atmosphere, aerodynamics, and propulsion;
4. applies one body force and moment to Chaos;
5. sends the committed state and sensors to ArduPlane; and
6. holds the latest accepted PWM command for the following interval.

The CMake CLI does not run beside Unreal and does not receive a live state stream. It is an
independent engineering test bench that compiles the same model sources. Later models that Chaos
cannot represent, such as a specialized seakeeping solver, can use the core integrator in-process
and publish the resulting pose to Unreal.

## Current limits

The v1 is local and air-started. It does not yet establish render-rate invariance, cross-platform
repeatability, sensor realism, terrain contact, or physical fidelity to a real aircraft. Very long
routes will eventually need a moving local frame as Earth curvature changes the local gravity
direction. Marine work will also need one authoritative wave state shared by rendering and forces;
see [WATER.md](WATER.md).

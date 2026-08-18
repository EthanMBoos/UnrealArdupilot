# Unreal setup

The finite scripted Chaos smoke path is implemented without Cesium. This document describes the G0
geospatial target that comes next; it should not be read as a claim that Cesium placement already
works in the repository.

Cesium loads the real-world map. Unreal spawns the vehicle in that map. Chaos moves the vehicle,
while the C++ core calculates its aerodynamic, propulsion, or marine forces.

On startup, Unreal reads the starting LLA and heading from the run config. Cesium loads the map at
that location, then Unreal spawns the vehicle and starts the physics loop. ArduPilot starts with the
same home and receives local NED position and velocity after each step. Cesium may convert the pose
back to LLA for display and checking, but it is not part of the controls feedback path.

```text
startup config LLA
    -> shared Cesium origin and ArduPilot home
    -> vehicle spawn
    -> core force and moment
    -> Chaos physics step
    -> NED state + simulated sensors
    -> ArduPilot command
    -> next physics step
```

## Starting location

The vehicle can start at any valid location covered by the selected Cesium tileset. On each run,
Unreal reads the LLA and heading, moves the Cesium map origin there, and spawns the vehicle at that
point. Air-start physics does not wait for terrain to finish streaming.

Use Unreal Engine 5.8 (5.8.1 is the tested macOS baseline) and [Cesium for Unreal
2.28.0](https://github.com/CesiumGS/cesium-unreal/releases/tag/v2.28.0). The first coordinate probe
only needs a `CesiumGeoreference`, a camera, the aircraft, and the simulation component. The map
smoke test then adds Cesium World Terrain; aerial imagery and `CesiumSunSky` are optional visuals.

The run config stores the map asset IDs and starting LLA. The Cesium ion token comes from
`CESIUM_ION_TOKEN` and is not saved in the repo or run logs. The project bootstrap reads that
variable and applies it to the tileset; Cesium does not do that automatically.

## Two details that matter

V1 uses one fixed `CesiumGeoreference` at the cartographic origin with identity transform, scale 100,
and no rebasing or origin-shift component. At that origin Cesium's Unreal axes are East, South, Up,
while the core uses North, East, Down. The Unreal simulation component owns that conversion. The exact matrices
and unit conversions are in the main [README](../README.md). The Chaos vehicle does not use a globe
anchor that changes its orientation while moving.

### Coordinates seen by controls engineers

The core matches normal controls notation: NED position; FRD body velocity `u,v,w` and rates `p,q,r`;
and body forces `X,Y,Z` and moments `L,M,N` about the center of mass. Vehicle files, logs, trim results,
linear models, and model code use the same convention in SI units.

The Unreal simulation component handles Cesium axes, Unreal centimeters, and `FVector`. Controls code only receives
the standard NED/FRD state and returns an FRD force and moment.

Cesium height is measured from the Earth ellipsoid. ArduPilot altitude is MSL. Store the signed geoid
undulation `N = h_ellipsoid - H_MSL`, then place Cesium at `h_ellipsoid = H_MSL + N`. Save the value
and its source with the run.

Streamed terrain is scenery for the first air-start tests. Takeoff, landing, and ground vehicles can
use Cesium collision meshes after their loading and collision behavior has been checked.

Boat work uses the same map and LLA path. Unreal Water supplies the actual water surface; Cesium
supplies the surrounding terrain and imagery. Disable Cesium's water appearance in the test area so
Unreal Water is the only displaced surface.

This groundwork is done when the vehicle spawns at its configured LLA, North and East are not
swapped, altitude matches Mission Planner, Mission Planner's position agrees with the LLA derived
from the same NED state, and turning Cesium on does not change the calculated vehicle forces.

Very long cross-country or global runs may eventually need a moving local physics frame as Earth's
curvature changes the local gravity direction.

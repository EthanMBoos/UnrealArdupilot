# Boat simulation — later

Get the aircraft working first. Boat work should start with the simplest Unreal setup that can float,
move in waves, and hit things without falling apart.

## First pass: use Unreal

Start with one displacement boat in one gentle regular deep-water wave. Unreal Water supplies the
visible and queried surface, Unreal Buoyancy pontoons keep the boat afloat, and one Chaos rigid body
handles all six motion axes and collisions.

One small config holds wave height, period, direction, and mean water level. M0 uses Unreal Water's
fixed `9.8 m/s^2` gravity. A tiny Unreal wave generator turns the config into a Water Waves Asset:

```text
amplitude  = wave height / 2
wavelength = 9.8 * period^2 / (2*pi)
phase      = 0
```

```text
wave config
  -> Unreal Water Waves Asset
       -> rendered water
       -> CPU water-height queries
            -> buoyancy pontoons
                 -> Chaos boat
```

This should look believable and respond to changing wave settings because rendering and buoyancy
query the same Unreal wave list. It is still a game-physics boat, not an engineering seakeeping
model. Pontoons and tuned damping do not calculate added mass, radiation, diffraction, or a real
hull's measured wave response.

Keep the first case in deep water and away from shore with one low-steepness regular wave. There is
no second material displacement or hidden visual wave layer. Use the real mass, center of mass,
and inertia. Calibrate the pontoons to the target flat-water draft and upright balance. Fit empirical
damping only from a saved decay test, while foam, color, reflections, and tiny normal-map ripples
remain visual.

Before adding more waves, CPU sample markers must stay on the rendered surface through crests and
troughs. Changing height, period, or direction must update the surface, markers, and boat together.
Pause, reset, and replay must return the same sampled heights. Flat-water displacement, draft, and
upright balance should match the configured boat; heave and roll decay should stay stable and
repeatable; and the floating boat should survive a simple dock collision.

Stock async buoyancy captures water time from the rendered game frame, so several Chaos steps may
query the same wave time. M0 must test this at different render and physics rates. If it does not stay
aligned, keep the Unreal Water wave asset but evaluate the pontoons from that wave list once per
physics tick.

These checks show that the pieces are connected. They do not show that the boat matches a real hull.

The [UE 5.6 Water Waves Asset](https://dev.epicgames.com/documentation/en-us/unreal-engine/simulating-waves-using-the-water-waves-asset-in-unreal-engine?application_version=5.6)
and [Buoyancy Component](https://dev.epicgames.com/documentation/en-us/unreal-engine/water-buoyancy-component-in-unreal-engine?application_version=5.6)
show the stock path. [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim/tree/ca8629df4e191235753dfae92ef725d30b923364)
is useful for wave queries and hull clipping, although its rendering and physics use separate wave
objects and it is not a full seakeeping solver.

[HydroChrono](https://github.com/Project-SEA-Stack/HydroChrono/tree/0648a2f880ae4237760b745b2f9ff4696559b76b)
shows BEM loads, radiation convolution, and live graphics. It is archived and has a large dependency
stack, so it serves as a reference rather than a base. [WEC-Sim 7.1](https://github.com/WEC-Sim/WEC-Sim/tree/v7.1.0)
provides another independent comparison for the later solver.

UE4 Ocean Project and ASVSim do not buy much here. The first duplicates wave math between CPU and
materials. The second mainly covers three-axis calm-water maneuvering.

## When stock buoyancy is not enough

Only build the rest when a real use case needs engineering wave response or tests show that stock
buoyancy is too crude.

The first engineering target is still small: one stationary displacement hull, one regular wave,
zero current, zero forward speed, and small motion about a fixed equilibrium pose.

Some terms appear throughout the engineering plan. **BEM coefficients** are offline force data
calculated from the hull shape. **Radiation memory** means the wake from earlier boat motion still
affects the force now. An **RAO** gives the amplitude and phase of boat motion caused by a wave at one
frequency.

Do not mix this model with planing, nonlinear hydrostatics, maneuvering, or contact. Those are
different problems.

```text
mode:             small-motion linear seakeeping
forward speed:    zero
current:          zero
wave theory:      first-order linear
viscous drag:     off while checking the equations
propeller/rudder: off
contact:          off
```

## First engineering test

Use one wave frequency and heading that exists directly in the hydrodynamic data. Run the full 6x6
mass, damping, stiffness, and wave-force calculation. Compare motion amplitude and phase with a
separate frequency-domain result.

This catches frame, unit, sign, coupling, integration, logging, and Unreal pose-transfer mistakes
before irregular waves or radiation fitting make the problem harder.

## Why Chaos stops owning the motion

Chaos can represent one translational mass, a center-of-mass offset, and a full rotational inertia
through principal axes. It cannot represent a general 6x6 marine mass matrix. In particular, it
cannot represent different effective mass in surge, sway, and heave or the translation/rotation
coupling common in hydrodynamic data.

For the engineering version, the core therefore integrates the exact 6x6 boat state. Unreal reads
the committed pose and renders a kinematic boat. Contact is off; an unexpected collision ends the
case instead of feeding a physically wrong Chaos impulse back into the solver.

A delayed correction force based on last step's acceleration is not the default. It changes phase,
can inject energy, and still cannot fix contact. If a Chaos approximation is ever tried, compare its
inverse-mass operator with the full one first. More than 1% relative error is an immediate rejection;
anything smaller still needs timestep, coupled-motion, RAO, work, and energy tests.

Before sending motion to Unreal:

1. Build a double-precision CLI solve for the full 6x6 system.
2. Add waves and radiation state with deterministic save/restore.
3. Drive a kinematic Unreal boat and check that its pose and render timestamp match the committed
   core state.

Supporting dynamic contact later would need a solver that uses the same generalized inverse mass
inside its contact equations. That is a separate project.

## Linear model

Let `xi = [x,y,z,roll,pitch,yaw]` use fixed equilibrium-frame axes:

```text
(M_RB + A_inf) xi_ddot
  + integral_0^t K_r(t-tau) xi_dot(tau) dtau
  + K_hs xi
  = tau_exc(t) + tau_external(t)
```

This first model has no nonlinear Coriolis term. A linear viscous matrix can be added after the base
equations agree with the references. The marine state is separate from the stateless aircraft model.

## Hydrodynamic coefficient files

Marine controls and autonomy commonly use the same coordinates as aircraft controls: NED for
position and FRD for the vehicle body, all in SI units. This is the convention used by Fossen/MSS
marine models and it matches ArduPilot. Marine force vectors are usually written
`[X,Y,Z,K,M,N]`, where `K` is roll moment.

Naval hydrodynamics programs such as Capytaine, WAMIT, and WEC-Sim commonly use X forward, Y port,
and Z up. That convention comes from naval architecture and wave-response work; it does not mean
the underlying physics is different. Imported hydrodynamic data is converted once so the runtime
boat model, controls code, logs, and ArduPilot all remain in NED/FRD.

For the first pass, use Capytaine offline to generate and check a fixed coefficient set. The runtime
reads a small normalized artifact, not Capytaine files or code. Capytaine and WEC-Sim remain
comparison tools; the time-domain solver is implemented in this repo.

Use this source convention for the first mesh pipeline:

```text
axes:       X forward, Y port, Z up
origin:     boat center of mass at equilibrium
DOFs:       surge, sway, heave, roll, pitch, yaw
units:      SI
```

Convert into core FRD with:

```text
Q = diag(+1,-1,-1, +1,-1,-1)

nu_core   = Q nu_bem
tau_core  = Q tau_bem
A_core    = Q A_bem Q^T
B_core    = Q B_bem Q^T
K_core    = Q K_bem Q^T
Fexc_core = Q Fexc_bem
```

Save the mesh and input hashes, solver version, axes, origin, scale, displaced volume, waterline,
density, gravity, depth, frequencies, headings, phasor convention, `A(omega)`, `B(omega)`, `A_inf`,
`K_hs`, excitation forces, and coefficient-quality report.

Check three mesh resolutions, displacement and hydrostatics, matrix symmetry, nonnegative radiation
damping, positive `M_RB + A_inf`, sensible free modes, frequency coverage, impulse-response decay,
and irregular-frequency spikes. Reference-point changes need a full matrix transform, not hand-edited
moment signs.

## Radiation memory

Using the [WEC-Sim Cummins convention](https://wec-sim.github.io/WEC-Sim/main/theory/theory.html):

```text
K_r(t) = 2/pi integral_0^inf B(omega) cos(omega t) domega
F_rad  = -A_inf xi_ddot - integral K_r(t-tau) xi_dot(tau) dtau
```

For `exp(i omega t)`:

```text
Khat_r(i omega) = B(omega) + i omega [A(omega) - A_inf]
```

Build a direct finite-history convolution in the CLI first. It is slow but easy to inspect. Once it
works, add the faster coupled state-space model
`z_dot = A_r z + B_r xi_dot`, `tau_memory = -(C_r z + D_r xi_dot)`.

Record the frequency range, windowing, extrapolation, fit error, and state order. The fitted model
needs stable poles and a positive-real/passivity check. Grid and time-history energy tests are still
useful, but they are not a mathematical passivity proof.

The CLI reference integrates the complete `[xi, xi_dot, z]` plant. Regular waves can be represented
with sine/cosine oscillator states. Recorded inputs state whether they are held linearly or
piecewise-constant. Save radiation state in checkpoints; only reset it to zero when the case starts
from a boat that has been motionless forever.

## One wave description

Physics, logs, and later rendering need the same realized wave components:

```cpp
struct WaveComponent {
    double omega_radps;
    double wavenumber_radpm;
    double direction_ned_rad;
    double amplitude_m;
    double phase_rad;
};
```

Start with regular waves. Add deterministic PM/JONSWAP component sums later. Save the actual sorted
component list and its hash, not only a random seed.

Use one convention everywhere:

```text
beta:       propagation direction clockwise from North
eta_up:     positive upward
psi_j:      omega_j (t-t_epoch) - k_j d_j dot (x_NE-origin_NE) + phase_j
eta_up:     sum_j amplitude_j cos(psi_j)
dispersion: omega_j^2 = g k_j tanh(k_j depth)
```

Amplitude means crest amplitude, or half the regular-wave height. For a spectrum,
`a_j = sqrt(2 integral_bin S_omega d omega)` and
`Hm0 = 4 sqrt(sum_j a_j^2/2)`.

The same list drives surface elevation, water-particle motion, pressure, and BEM excitation at the
fixed equilibrium point:

```text
sum_j Re{F_exc,j a_j exp(i psi_B,j(t))}
```

Normalize external phasor conventions while importing. Capytaine uses `exp(-i omega t)`, so its
complex excitation is conjugated once for this project's `exp(+i omega t)` convention. Missing wave
frequencies or headings are import errors.

Stock Unreal Water is good enough for the simple first pass, but it cannot reproduce this later
component model exactly: its spectral generator is unfinished, arbitrary phase is not carried
through every CPU/GPU path, and its clock and large-world handling differ. If the engineering solver
reaches this point, feed the same components and pose timestamp into a small project-owned render
path. Then compare GPU height with the core wave function over time, pause, seek, and world rebasing.

## Checking the engineering solver

The frequency-domain reference is:

```text
[-omega^2 (M_RB + A(omega)) + i omega B(omega) + K_hs] xi_hat
  = F_exc(omega,beta) a_hat
```

For time-domain tests, use small regular waves, no speed, current, viscous drag, propulsion, contact,
or rendering dependency. Ramp the wave, drop the startup transient, and fit complex amplitude over
several periods.

Start with 2% complex error for direct convolution against the frequency result and 2% for the
state-space model against direct convolution. Allow 5% through the final core-to-Unreal pose and log
path, including sample timing.

Use absolute tolerances near a zero response. Test head, beam, and one oblique wave, including all
coupled motion axes. Add chirp or multisine tests so a steady RAO does not hide bad transient memory.

Matching Capytaine or WEC-Sim still does not prove that the hull model matches a real boat. That
eventually needs published decay and complex RAO data. Roll damping will probably need a separate
viscous model or test data.

## Maneuvering is separate

Calm-water steering comes later as its own mode:

```text
waves:      off
radiation:  off
model:      MMG or measured maneuvering derivatives
actuation:  propeller and rudder
reference:  SIMMAN KVLCC2, KCS, or DTMB data
```

Do not combine maneuvering and seakeeping until each works alone and there is a clear rule for shared
added mass, damping, wave encounter speed, and mean drift.

Nonlinear buoyancy and Froude-Krylov forces also wait. When added, they replace the matching linear
term; they are not stacked on top of it.

## Rough order

| Step | Work | Check |
|---|---|---|
| M0 | Unreal Water, Buoyancy and Chaos | water, queries, flotation, replay, decay and dock tests pass |
| E0 | one-frequency 6x6 case | amplitude and phase match the frequency-domain calculation |
| W1 | saved wave components | reset and replay reproduce the same wave |
| W2 | hydrodynamic coefficient import | mesh and coefficient checks pass |
| W3 | CLI 6x6 solve | mass and hydrostatic hand cases pass |
| W4 | direct radiation and excitation | regular-wave response agrees within 2% |
| W5 | state-space radiation | stability, passivity and direct-convolution checks pass |
| W6 | core solver driving Unreal | pose, timing, replay and RAO checks pass; contact remains off |
| W7 | matching Unreal water render | CPU/GPU height and phase agree |
| W8 | viscous damping and real data | decay and RAO evidence are saved |
| M1 | separate calm-water maneuvering | standard benchmark maneuvers pass |

Planing, forward-speed seakeeping, maneuvering in waves, nonlinear hydrostatics, engineering contact,
mooring, towing, directional seas, CFD tuning, and perception-grade water can all wait until the
earlier pieces are useful.

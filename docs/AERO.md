# Fixed-wing model

The first aircraft is small on purpose: one educational model, working headless before it goes into
Unreal or talks to ArduPilot. Shared coordinates, timing, files, and data flow are in
[README.md](../README.md).

## The aircraft

Use `mavsim_aerosonde_educational` from BYU MAVSim's pinned
[`aerosonde_parameters.py`](https://github.com/byu-magicc/mavsim_public/blob/a8489bbc30ab9e746dccac8ac614ab2f8acac6bb/mavsim_python/parameters/aerosonde_parameters.py)
at commit `a8489bbc30ab9e746dccac8ac614ab2f8acac6bb`.

Keep its mass, inertia, geometry, derivatives, motor, and propeller parameters together in one
`aircraft.json`. The pinned BYU file is the parameter source; the equations written below define the
runtime model. The current public MAVSim dynamics file is an educational template, so it is not used
as an executable reference.

The positive `Jxz` value from the parameter file enters the FRD inertia matrix with negative
off-diagonal terms:

```text
I_body = [ Jx    0  -Jxz ]
         [  0   Jy    0   ]
         [-Jxz   0   Jz   ]
```

This is a controls example, not a real Aerosonde twin. The file describes an 11 kg aircraft flying
around 25 m/s. The repo describes the real aircraft as roughly 25 kg and 35 m/s. Without flight data,
the work can show that the equations behave correctly and that the implementation matches the same
equations in JSBSim. Physical accuracy is still unknown.

Use this range for the JSBSim comparison:

| Quantity | Range |
|---|---|
| true airspeed | 18–40 m/s |
| angle of attack | -10–12° |
| sideslip | ±10° |
| normalized body rates | ±0.2 |
| control surfaces | ±15° |
| altitude | 0–3000 m |

Outside it, the model only needs to stay finite and let aerodynamic loads fall to zero with
airspeed. Good stall behavior, ground effect, gear, takeoff, and landing all need better data later.

## Air data

The aircraft function receives rigid state, physical effectors, and atmosphere. It calculates air
data every time it is called, including every RK4 stage:

```text
v_air_body = velocity_body - R_body_to_ned^T wind_ned
V          = norm(v_air_body)
alpha      = atan2(w, u)
beta       = atan2(v, sqrt(u^2 + w^2))
qbar       = 0.5 rho V^2

p_hat = p b / (2 V)
q_hat = q c / (2 V)
r_hat = r b / (2 V)
```

At `V=0`, aerodynamic force and moment are exactly zero. Below `V=1e-12 m/s`, set the normalized
rate terms to zero rather than divide by `V`; dynamic pressure drives the remaining aerodynamic
loads smoothly to zero.

There are no `alpha_dot` terms in this model because the source data doesn't contain them. Adding
previous-step acceleration would quietly create a delayed, timestep-dependent model. If a later
aircraft needs unsteady aerodynamics, give it a real state-space or implicit model.

The C++ implementation preserves the same mathematical stages instead of expanding them into one
large function:

```text
state + atmosphere
  -> AirData {V, alpha, beta, qbar, p_hat, q_hat, r_hat}
  -> AeroCoefficientSet {C_L, C_D, C_Y, C_ell, C_m, C_n}
  -> AerodynamicsOutput {air data, coefficients, body wrench}
```

Runtime vectors, matrices, and quaternions are fixed-size Eigen types. Domain structs and member
names carry the frame and units; bare vector indices are not used to distinguish physical values.
`fixed_wing.cpp` keeps the complete aircraft wrench calculation together; `rigid_body.cpp` owns
generic mechanics and integration.

## Aerodynamics

All angular derivatives and surface angles are per radian.

The high-angle lift guard uses:

```text
sigma = [1 + exp(-M(alpha-alpha0)) + exp(M(alpha+alpha0))]
        / [(1 + exp(-M(alpha-alpha0))) (1 + exp(M(alpha+alpha0)))]

CL_base = (1-sigma)(CL0 + CL_alpha alpha)
          + sigma [2 sign(alpha) sin(alpha)^2 cos(alpha)]
```

Evaluate the same expression with a numerically stable logistic form rather than allowing either
exponential to overflow.

```text
CL = CL_base(alpha) + CL_q q_hat + CL_de de
CD = CD_p + (CL0 + CL_alpha alpha)^2 / (pi e AR)
     + CD_q q_hat + CD_de de

CY = CY0 + CY_beta beta + CY_p p_hat + CY_r r_hat
     + CY_da da + CY_dr dr

Cl = Cl0 + Cl_beta beta + Cl_p p_hat + Cl_r r_hat
     + Cl_da da + Cl_dr dr
Cm = Cm0 + Cm_alpha alpha + Cm_q q_hat + Cm_de de
Cn = Cn0 + Cn_beta beta + Cn_p p_hat + Cn_r r_hat
     + Cn_da da + Cn_dr dr
```

Use `CD_p` as the runtime drag baseline. The source's `CD0` and `CDalpha` values belong to its linear
design model and are not extra drag terms. Keep the small cross terms too, including
`Cl_dr = 0.0024` and `Cn_da = -0.011`.

This model deliberately uses the signed `CD_de de` term shown above. Do not replace it with
`CD_de abs(de)` without creating a new model version and updating the JSBSim fixture.

The source rotates lift and drag through angle of attack while side force stays on body `+Y`:

```text
Fx = qbar S (-CD cos(alpha) + CL sin(alpha))
Fy = qbar S CY
Fz = qbar S (-CD sin(alpha) - CL cos(alpha))

[L, M, N] = qbar S [b Cl, c Cm, b Cn]
```

This gives an FRD wrench about the center of mass. It excludes gravity and propulsion.

### High angles

Keep the Beard/McLain lift blend with the source values `alpha0 = 0.47 rad` and `M = 50`, implemented
without exponential overflow. It is only a numerical guardrail. Drag still comes from the linear
lift expression, and there is no stall hysteresis or separated-flow model.

## Controls and propeller

The channel map turns ArduPilot PWM into named normalized commands. The actuator map turns those
into physical values:

```cpp
struct AircraftCommand {
    double aileron;        // -1..1
    double elevator;       // -1..1
    double rudder;         // -1..1
    double throttle;       //  0..1
};
```

```cpp
struct AircraftEffectorState {
    double aileron_rad;
    double elevator_rad;
    double rudder_rad;
    double throttle;       // 0..1
};
```

Surface direction, neutral, and limits come from `aircraft.json`. Surfaces move instantly in the
first version. Positive `da` must produce positive roll moment, positive `de` follows the written
coefficient law and produces negative pitch moment near trim, and positive `dr` follows the written
side-force and yaw coefficients. The channel map records the real left/right surface motion needed
to produce those model signs. Any later lag needs explicit actuator state.

Use the BYU electric motor and polynomial `C_T/C_Q` propeller equations with their original
constants:

```text
C_T(J) = CT2 J^2 + CT1 J + CT0
C_Q(J) = CQ2 J^2 + CQ1 J + CQ0
J      = 2 pi V_a / (omega D)

a = rho D^5 CQ0 / (4 pi^2)
b = rho D^4 CQ1 V_a / (2 pi) + K_Q^2 / R_motor
c = rho D^3 CQ2 V_a^2 - K_Q V_in / R_motor + K_Q i_0

omega = (-b + sqrt(b^2 - 4ac)) / (2a)
T     = rho n^2 D^4 C_T(J)
Q     = rho n^2 D^5 C_Q(J),       n = omega / (2 pi)
```

Thrust points along body `+X`; reaction torque acts about `X`; an offset propeller also adds
`r_prop x T`. `aircraft.json` declares the fitted advance-ratio range. Reference runs fail when they
leave it. Exploratory runs may continue only with finite output and a logged out-of-range flag;
invalid roots or nonfinite values always fail the run.

Battery state, spool time, propwash, gyroscopic effects, variable pitch, and multiple engines can
wait.

## Atmosphere

Use ISA temperature, pressure, and density with constant NED wind. Gravity belongs to the rigid-body
integrator, not the atmosphere or aircraft wrench. Gusts and spatial weather can be added later as
pure position/time queries.

## Trim

The first trim point is straight, level flight at 25 m/s in still ISA air. Propeller reaction torque
means it is a full six-degree-of-freedom solve:

```text
fixed:       yaw=roll=p=q=r=0
unknowns:    u, v, w, pitch, aileron, elevator, rudder, throttle
residuals:   velocity_dot_body(3), omega_dot_body(3), TAS-25, velocity_ned.Down
bounds:      alpha [-10,12]°, beta ±10°, surfaces ±15°, throttle [0,1]
```

Position is allowed to move; straight flight is not a stationary point in NED. Save the complete
state, command, effectors, residuals, bounds, solver result, and environment in
`results/operating_point.json`.

Trim passes when each linear and angular acceleration residual is at most `1e-5` in SI units,
airspeed and vertical-speed error are each at most `1e-5 m/s`, and no control sits on a bound. A 10 s
CLI run must also drift less than `0.05 m/s` in airspeed, `0.5 m` in altitude, and `0.1°` in attitude.

Later runs may reference this operating point by path.

## Linearization

Differentiate the same deterministic one-step map with central differences. Use this local state:

```text
[position_ned(3), velocity_body(3), attitude_error_body(3), omega_body(3)]
```

Perturb attitude on the right with `q = qbar Exp(delta_theta)`. Measure next-step attitude error in
the nominal next body's tangent frame. Save `xbar_0`, `ubar`, `xbar_1`, and labelled discrete
`A_d/B_d` matrices. Including `xbar_1` matters because the trimmed aircraft still moves through NED.

The default `aircraft_command` input uses normalized aileron, elevator, rudder, and throttle for
controls work. The diagnostic `aircraft_effector` input bypasses that mapping and uses surface
radians and throttle directly when checking equations.

Save scales, limits, perturbation sizes, `dt`, integrator, and input boundary in
`results/linear_model.json`.

Run differences at `h`, `h/2`, and `h/4`. The scaled change in both A and B should be no more than
`1e-3`, with the third result confirming convergence rather than roundoff. A perturbation that
crosses a limit fails instead of switching to a one-sided difference. A small doublet should stay
within 2% scaled state error over two seconds.

## Scripted controls

Schedules use integer ticks. A row with `apply_tick=k` takes effect before interval
`[t_k,t_(k+1))` and is held through that step. Optional `time_s` must match `k*dt`. Tick 0 gives every
input; later rows may omit channels to hold their old value.

Each schedule names its input boundary, whether values are absolute or offsets from trim, the trim
path, and its final hold/stop rule. Reject duplicate or out-of-order ticks, unknown channels, off-grid
times, and limit violations.

## JSBSim comparison

Use JSBSim 1.3.1 at pinned commit
[`3b25f25e49b42d0489c04ac805674fc1450ca579`](https://github.com/JSBSim-Team/jsbsim/commit/3b25f25e49b42d0489c04ac805674fc1450ca579).
Record the exact package filename and SHA-256.

A checked-in aero-only JSBSim fixture contains the same educational equations. It has no engine,
gear, dynamic controls, or output system. This catches sign, unit, axis, coefficient, and reference-
point mistakes. It does not prove the model matches a real aircraft.

For each test point:

1. Reset through the documented
   [`FGFDMExec`](https://jsbsim-team.github.io/jsbsim/python/FGFDMExec.html) API.
2. Set position, altitude, attitude, body velocity, rates, and zero wind. Let JSBSim calculate alpha,
   beta, and dynamic pressure.
3. Check the resulting air data and use JSBSim's density for the core comparison.
4. Set actual surface positions in radians. Keep the aero reference point at the center of mass.
5. Suspend integration, evaluate once, and confirm time and state did not move.
6. Read coefficients and body forces/moments, convert them to SI, and compare.
7. Reset between samples and repeat a few cases in different orders and fresh processes.

Required coverage is every coefficient basis case, the range boundaries, and 200 fixed-seed mixed
samples. Run 10,000 samples nightly or before finishing this checkpoint.

Starting tolerances:

```text
coefficient: |dC| <= 1e-10 + 1e-9 |C_ref|
force:       |dF| <= 1e-5 N + 1e-8 qbar S
moment:      |dM| <= 1e-6 N-m + 1e-8 qbar S max(b,c)
```

If unit conversion noise forces a looser tolerance, measure it first. Deliberate sign, rate-scaling,
and reference-point mistakes still need to fail. Propulsion uses direct equation and golden-value
checks rather than this aero-only fixture.

For this fixture, JSBSim body axes are FRD and its aerodynamic reference point is at the center of
mass. Its construction coordinates are different (`X` aft, `Y` right, `Z` up), so point coordinates
need an explicit conversion.

## Checks before Unreal

The CLI loader performs the basic mass, inertia, timestep, and required-field checks needed by the
current aircraft file. `uvd evaluate` exposes air data, coefficients, effectors, and each wrench for
focused hand cases. Broader schema and envelope checking should be added when more than one vehicle
file makes it useful.

The optional JSBSim comparison checks the aero-only fixture over boundary, basis, and random cases.
The timestep script runs the RK4 trajectory at `1/60`, `1/120`, and `1/240` and verifies that the
successive state differences shrink. `uvd trim` reports its residuals and `uvd linearize` produces
the discrete local model at that operating point.

## Order

| Step | Work | Done when |
|---|---|---|
| A1 | parameters and pure aero | model evaluates finitely in the declared envelope |
| A2 | JSBSim comparison | coefficients and aero wrenches agree across the sampled range |
| A3 | propeller, ISA and trim | the saved operating point has small residuals |
| A4 | RK4 and linearization | timestep error shrinks and finite local matrices are produced |

The first Cesium/Chaos/ArduPlane path and its next engineering milestones are described in
[ARDUPILOT.md](ARDUPILOT.md). Flight-data validation, better stall data, ground operation, richer
propulsion, sensor errors, and more aircraft come later.

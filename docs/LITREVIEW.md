# Literature review and research direction

**Primary window:** January 2025 through 17 August 2026

## Executive conclusion

The strongest research contribution available to this project is not another Unreal-based UAV
simulator. Unreal, AirSim descendants, Gazebo, JSBSim, PX4, ArduPilot, and newer modular simulation
frameworks already cover much of that feature space. Recent papers frequently combine a game engine,
an autopilot, ROS, geospatial scenery, or synthetic sensors. Those integrations are useful, but the
combination alone is no longer a convincing novelty claim.

The more defensible opportunity is an **open, deterministic, controls-oriented fixed-wing benchmark**
that uses one inspectable nonlinear plant for:

- headless trim, linearization, excitation, comparison, and replay;
- real-time execution in Unreal without allowing rendering to redefine the plant;
- ArduPlane and PX4 closed-loop experiments with identical initial conditions, actuator and sensor
  contracts, disturbances, timing rules, and metrics; and
- a traceable verification and validation ladder ending in held-out flight data and uncertainty
  bounds.

This leads to a clear strategy:

1. Finish ArduPlane first because its external JSON interface fits the existing architecture and is
   the shortest path to an end-to-end fixed-wing result.
2. Keep the core, cases, logs, and scientific identity autopilot-neutral.
3. Add PX4 only after the ArduPlane path is stable, then publish the stronger same-plant comparison.
4. Treat Unreal as the geospatial, contact, and future perception layer—not as evidence of aerodynamic
   fidelity.
5. Do not call the project a digital twin or high-fidelity aircraft model until physical validation
   supports those terms.

The current code is a promising start because it already treats trim, linearization, deterministic
replay, timestep convergence, explicit frames, and reference comparison as first-class operations.
The research contribution will come from turning those pieces into a public experimental contract and
then supplying evidence that other researchers can reproduce.

## Scope and method

This is a structured narrative review, not a PRISMA systematic review or a bibliometric census. It
prioritizes 2025 and 2026 peer-reviewed papers, proceedings, and clearly labeled preprints. Older work
is included only where it defines a major simulator or verification precedent. Official project
documentation and repositories are used for current software capabilities; vendor pages are treated
as product claims, not academic validation.

The search covered combinations of:

- fixed-wing UAV, flight dynamics, system identification, uncertainty, and flight validation;
- simulation, benchmark, reproducibility, SIL/SITL, HIL/HITL, and digital twin;
- Unreal Engine, AirSim, Colosseum, Cosys-AirSim, AeroSim, Gazebo, and JSBSim; and
- ArduPilot, ArduPlane, PX4, ROS 2, differentiable physics, and reinforcement learning.

The main limitation is that 2026 is incomplete. Preprints can change status, software repositories can
move quickly, and a qualitative literature search cannot support an absolute “first” claim. Before a
paper submission, the novelty statement should be rerun through Scopus, Web of Science, IEEE Xplore,
Google Scholar, and the target venue's proceedings.

## What exists in this repository today

The distinction between implemented and planned work matters because a literature review can easily
turn a roadmap into an accidental claim.

| Capability | Status on 18 August 2026 | Research meaning |
|---|---|---|
| C++ fixed-wing force and moment model | Implemented | Transparent reference plant, presently using educational Aerosonde parameters |
| Six-degree-of-freedom headless rigid-body propagation | Implemented | Deterministic contact-free experiments are possible |
| JSON aircraft and run configuration | Implemented | Cases can be defined without editing equations |
| Validation, scripted simulation, trim, and discrete linearization | Implemented | Good basis for a controls benchmark |
| Run bundles, comparison, and deterministic replay | Implemented | Good basis for artifact reproducibility |
| Unit/sign checks and low-speed/high-angle finite sweep | Implemented | Code verification, not physical validation |
| Aero-only JSBSim comparison fixture | Implemented | Detects equation, frame, sign, unit, and scaling errors; the fixture mirrors the same equations |
| Physical Aerosonde validation | Not implemented | No claim of real-aircraft fidelity is yet supported |
| Unreal/Chaos open-loop runtime | Smoke and local U0/U1 suites implemented | Mechanics/timing, matched-state wrench, and trajectory-refinement probes pass locally; G0 and cross-platform evidence remain |
| Cesium runtime | Not implemented | Geospatial placement remains G0 work |
| ArduPlane JSON/UDP transport | Live local transport and fault suite implemented | Consecutive lockstep PWM/state exchange and explicit duplicate/gap/stale/malformed/timeout handling work; readiness, arming, and closed-loop acceptance remain |
| PX4 adapter | Not implemented | A research opportunity, not current capability |
| Sensor errors, actuator dynamics, HIL, or flight data | Not implemented | Required for stronger closed-loop and sim-to-real claims |

Agreement with the checked-in JSBSim fixture is **software verification**, not independent aerodynamic
validation: both sides encode the same educational equations. It is still valuable because independent
implementations expose sign, axis, unit, normalized-rate, and reference-point errors. Real validation
requires a physical reference such as wind-tunnel measurements or held-out flight trajectories.

## State of the field in 2025–2026

### Simulator breadth is no longer scarce

The 2025 version of Dimmig et al.'s survey catalogs 44 aerial-robot simulators and compares 14 in
depth. Its important lesson is not that one package wins; simulator choice remains use-case dependent,
and dynamics, sensors, speed, extensibility, maintenance, and environment fidelity trade against one
another. Fixed-wing users remain less uniformly served than multirotor perception and learning users,
but a new feature checklist is not enough to establish novelty
([Dimmig et al., 2025](https://doi.org/10.1109/MRA.2024.3433171)).

Recent general platforms continue to expand the same crowded space. ProVANT presents a modular
Gazebo/ROS environment for SIL and HIL control work across multirotors, tilt-rotors, and aerial
manipulators ([Morais et al., 2025](https://doi.org/10.3390/aerospace12090762)). AeroQT combines
JSBSim, MAVLink, PX4, simulation, HIL, and real-flight views in a C++/Qt test bench
([Aziz and Loya, 2025](https://doi.org/10.3389/arc.2025.14524)). The 2026
`aerial-autonomy-stack` supplies a common PX4/ArduPilot interface, ROS 2, perception, containers, and
faster-than-real-time execution, although its emphasis is vertically integrated multirotor and VTOL
autonomy rather than conventional fixed-wing model analysis
([Panerati et al., 2026](https://arxiv.org/abs/2602.07264)).

Two ICUAS 2026 papers narrow the gap further. ROSplane 2.0 explicitly presents lean, readable,
modular fixed-wing research software and an aerodynamic-model workflow intended to ease movement
from simulation to flight ([Reid et al., 2026](https://arxiv.org/abs/2510.01041)). Bolz et al.'s
fixed-wing swarm testbed already reports JSBSim dynamics, deterministic seeds, synchronized replay,
and safety/effectiveness logs ([ICUAS program, 2026](https://controls.papercept.net/conferences/scripts/rtf/ICUAS26_ContentListWeb_3.html)).
These are strong precedents for readability and repeatability, so this project must distinguish itself
through the complete verification ladder and controlled production-autopilot experiment—not those
features in isolation.

The implication is direct: modularity, MAVLink, SIL/HIL, or two autopilot connectors are useful
capabilities, but none is a sufficient paper contribution in isolation.

The overlap is even clearer in current software. ArduPilot's official Gazebo integration already
includes a fixed-wing Zephyr model and lockstep JSON exchange, while PX4's current Gazebo path
includes conventional fixed-wing models and lockstep simulation. Project AirSim provides
deterministic stepping and PX4 lockstep for its supported vehicles. PteroSim, a proprietary 2026
entrant, markets Unreal Engine 5, six-degree-of-freedom fixed-wing and VTOL models, and both PX4 and
ArduPilot SITL. PteroSim's performance and fidelity statements are vendor claims rather than
peer-reviewed evidence, but its feature list makes the competitive point: connecting a fixed-wing
plant, Unreal, and two autopilots is not by itself a research result.

### Unreal is valuable, but it is usually the visual and scenario layer

AirSim established the familiar combination of Unreal rendering, vehicle APIs, synthetic sensors,
and PX4/ArduPilot connectivity. Microsoft's repository now directs users to successors rather than an
actively developed original platform. Colosseum moved the design to Unreal Engine 5 but was archived
on 11 July 2026. Cosys-AirSim continues the ecosystem with newer Unreal support and richer sensing,
while the original JSBSim project itself ships an open Unreal reference application. AeroSim takes a
more general co-simulation approach with FMI, middleware, and swappable Unreal or Omniverse rendering.

Academic work uses these pieces primarily for environment, perception, and mission research. A 2025
IFAC paper combines Unreal Engine 5, AirSim, ROS 2, ArduPilot, and image-based solar-plant inspection
([Andrade et al., 2025](https://doi.org/10.1016/j.ifacol.2025.09.198)). A 2025 DASC paper combines
Unreal, Cesium, Colosseum, and PX4 SITL for urban manned–unmanned formation experiments
([Lee, Kim, and Lee, 2025](https://doi.org/10.1109/DASC66011.2025.11257249)). RflyUT-Sim combines
Unreal-based scenes with a much broader traffic, communication, fault, and low-altitude operations
framework ([Li et al., 2025, preprint](https://arxiv.org/abs/2512.24112)).

The emerging architecture also supports separating visual fidelity from dynamics. Unreal Robotics
Lab embeds MuJoCo as the physics authority while Unreal handles rendering and scene construction
([Embley-Riches et al., ICRA 2026](https://arxiv.org/abs/2504.14135)). That is conceptually close to
this project's separation between an inspectable vehicle model and Unreal's world. The separation is
good design, but it must be quantified: rendering load, tick scheduling, coordinate conversion, and
real-time coupling must not silently change the experiment.

Therefore, “an Unreal fixed-wing simulator” is not a safe novelty claim. A better claim is a verified
fixed-wing experiment whose dynamics contract remains measurable across headless and Unreal runtimes.

### Fixed-wing research is raising the validation bar

The most relevant 2025–2026 fixed-wing work is not satisfied with a plausible coefficient table.
Researchers are deriving models from aerodynamic tools, identifying them from flight data, validating
modal behavior, and quantifying uncertainty.

- Cavalheiro and Guerreiro automate an AVL-to-Gazebo aerodynamic pipeline and synchronize PX4 and
  simulator parameters. This means that coefficient generation and PX4/Gazebo model deployment are
  already active publication territory
  ([2025](https://doi.org/10.1109/YEF-ECE66503.2025.11117433)).
- Matt et al. combine open-loop and closed-loop flight data to identify small flying-wing dynamics,
  including phugoid behavior and closed-/broken-loop frequency-response checks
  ([2025](https://doi.org/10.2514/1.C038147)).
- Løw-Hansen et al. identify a Skywalker X8 model using estimated aerodynamic angles and a
  stability-frame formulation intended to ease comparison across models
  ([2025](https://doi.org/10.1007/s13272-025-00816-3)).
- Figueira et al. identify nonlinear aero-propulsive coupling from limited flight data and validate
  predictive behavior on out-of-sample maneuvers
  ([2025](https://doi.org/10.2514/1.C037964)).
- Jayanti et al. compare CFD-derived six-degree-of-freedom dynamics with three flight datasets and
  explicitly evaluate longitudinal and lateral modes. Residual errors are traced to coefficients,
  assumptions, and initial-condition mismatch rather than hidden behind trajectory plots
  ([2026](https://doi.org/10.3311/PPtr.41190)).
- Siswantara et al. combine unsteady overset CFD, system identification, and flight telemetry for a
  Skywalker 1800 longitudinal model
  ([2026, early access](https://doi.org/10.1016/j.taml.2026.100702)).

FalconWing raises the bar from another direction: it releases an ultralight fixed-wing platform,
identifies nonlinear dynamics from real flight, builds a photorealistic world from real imagery, and
demonstrates zero-shot vision-only landing on hardware
([Miao et al., 2025 preprint/workshop](https://arxiv.org/abs/2505.01383)). Its flight envelope and
task are narrow, but it shows why an open simulator is much stronger when it closes the loop with a
physical aircraft and a concrete research result.

For this repository, a generic educational Aerosonde model can support code verification and
benchmark development. It cannot support claims about an actual Aerosonde's behavior without new
data and parameter provenance.

The evidence culture is also moving toward reusable datasets. García-Gascón et al. publish 240
fixed-wing missions and more than 32 hours of synchronized telemetry, mission definitions, and
parameter snapshots across PX4 and INAV aircraft
([2026](https://doi.org/10.1038/s41597-026-06716-3)). That collection is not aerodynamic ground truth,
but it is a useful model for the provenance and packaging expected of a public benchmark.

### Digital-twin language now implies a physical relationship

Recent reviews distinguish a simulation model from a dynamic twin that is synchronized with and
updated from a physical counterpart. Reviews of drone and advanced-air-mobility twins identify
sim-to-real error, standardization, consistency, verification, certification, and continuous update
as unresolved problems ([Moon, 2025](https://journal.kci.go.kr/kiots/archive/articleView?artiId=ART003259737);
[Zhang et al., 2025](https://doi.org/10.3390/drones9060394)).

Valencia et al. combine ROS/Gazebo, ArduPilot, reconstructed terrain, and fixed-wing and multirotor
flight logs from Andean missions, then compare simulated and physical positioning
([2025](https://doi.org/10.1007/s10846-025-02276-7)). Guevara et al. explicitly combine model-,
hardware-, and experiment-based validation in a UAV twin methodology
([2025 preprint](https://doi.org/10.2139/ssrn.5145160)). These works reinforce a useful vocabulary
rule: until a particular physical aircraft supplies calibration and validation data, this project is
a **simulation and benchmark platform**, not that aircraft's digital twin.

NASA and engineering V&V guidance likewise frame credibility as an application-specific accumulation
of code verification, solution verification, validation, and uncertainty quantification—not as a
property conferred by a renderer or a single reference comparison
([NASA/FAA, 2026](https://ntrs.nasa.gov/api/citations/20260001729/downloads/NASA-TM-20260001729.pdf);
[ABS, 2024](https://ww2.eagle.org/content/dam/eagle/rules-and-guides/current/design_and_analysis/348-guidance-notes-on-verification-and-validation-of-models%2C-simulations%2C-and-digital-twins-2024/348-vandv-gn-nov24.pdf)).

### Learning and differentiable simulators are advancing quickly

The learning literature is moving toward fast batched dynamics, differentiability, and hybrid
physics/data models. Zhang et al. demonstrate real-world agile flight trained through differentiable
first-principles physics, although their training model is intentionally a simple point mass
([2025](https://doi.org/10.1038/s42256-025-01048-0)). Michek, Mehta, and Huebsch use
physics-informed neural networks for flight-dynamics parameter estimation
([2025](https://doi.org/10.2514/1.J063991)), while Harp et al. use a physics-informed Gaussian-process
mean and real T-38 data to estimate aerodynamic behavior with uncertainty
([2025 preprint; 2026 journal version](https://arxiv.org/abs/2501.01000)).

Fixed-wing learning platforms are appearing as well. FALCON-S proposes GPU-batched six-degree-of-
freedom fixed-wing dynamics, actuator and sensor effects, disturbances, and ground effect, but its
ICLR 2026 submission was withdrawn and should be treated as a preprint rather than peer-reviewed
evidence ([OpenReview](https://openreview.net/forum?id=BIDlQ5ifz4)). Differentiable aircraft and
aeroelastic tools also already exist
([Cea and Palacios, 2025](https://doi.org/10.1016/j.cpc.2025.109547)).

This makes GPU acceleration, reinforcement learning, or automatic differentiation poor first-paper
novelty claims by themselves. They are better later extensions serving a sharper question such as
uncertainty-aware controller tuning or differentiable same-plant co-design.

## Competitive position

| System or line of work | Strongest capability relevant here | What remains open for this project |
|---|---|---|
| JSBSim | Mature configurable aircraft FDM; headless, autopilot, Python, MATLAB, and Unreal paths | A smaller controls-readable benchmark contract and measured multi-runtime/autopilot experiments |
| PX4 + Gazebo/JSBSim | Strong first-party simulation and academic/ROS 2 ecosystem | Same-plant comparison with ArduPlane and explicit controls-analysis artifacts |
| ArduPilot SITL + Gazebo/JSBSim | Broad vehicle support, mature ArduPlane, simple external JSON boundary | Modern Unreal fixed-wing path with rigorous timing and headless equivalence evidence |
| Project AirSim | Deterministic stepping and PX4 lockstep for multirotor and VTOL configurations | Conventional fixed-wing model analysis and same-plant production-autopilot comparison |
| AirSim descendants | Photorealistic scenes, APIs, sensors, PX4/ArduPilot connectivity | Conventional fixed-wing V&V, trim/linearization, model uncertainty, and experiment traceability |
| AeroSim | Modular co-simulation, FMI, middleware, renderer independence | Focused open fixed-wing benchmark evidence rather than framework breadth |
| PteroSim | Proprietary product claiming UE5, fixed-wing/VTOL dynamics, and PX4/ArduPilot SITL | Open equations, independent V&V, reproducible cases, and publishable controlled experiments |
| ProVANT | Controls-oriented SIL/HIL and modular Gazebo/ROS experiments | Transparent conventional fixed-wing plant and cross-autopilot experiment contract |
| AeroQT | Lightweight JSBSim/MAVLink simulation and HIL test bench | Deterministic offline analysis, Unreal equivalence, systematic metrics, and uncertainty |
| FalconWing | Real-to-sim-to-real fixed-wing platform and physical landing result | Broader flight-dynamics envelope, modal validation, and production-autopilot comparison |
| FALCON-S preprint | GPU fixed-wing learning and ground-effect benchmark | Peer-reviewed validation and a non-learning V&V/autopilot workflow |
| This repository today | Readable C++ equations, headless RK4, trim, linearization, comparison, replay | Unreal, either autopilot, physical validation, uncertainty, richer actuator/sensor models |

### Why not just use JSBSim?

This is the most important build-versus-extend question. JSBSim is mature, configurable, actively
maintained C++, runs headless, has Python and MATLAB paths, connects to both flight-stack ecosystems,
and already ships an Unreal reference application. Reimplementing a smaller six-degree-of-freedom
model is not itself a contribution, and a new general-purpose FDM would be difficult to justify.

The defensible reason to retain this core is narrower: it is a compact, equation-traceable reference
plant whose state, integration, linearization, and experiment semantics can be held under direct
control. JSBSim should remain a reference and possible alternate plant, not be framed as an obsolete
competitor. The project should measure whether its smaller core actually improves auditability,
linear-model extraction, deterministic replay, or cross-runtime equivalence. If it cannot demonstrate
one of those advantages, the benchmark would be stronger built on JSBSim than beside it.

## Where this project can stand out

### 1. Same-plant ArduPlane–PX4 fixed-wing benchmark

This is the strongest combination of novelty, feasibility, and fit.

Most platform comparisons change several things at once: simulator, plant, sensor implementation,
airframe parameters, controller mode, rate, and tuning. That makes it difficult to attribute a result
to the autopilot. This project can remove those confounders by feeding both flight stacks the same
plant and realized experiment.

RouthSearch demonstrates that cross-stack testing is already scientifically productive: it evaluated
PID parameter safety across ArduPilot and PX4 and reported bugs in both. Its executions used the
stacks' respective simulators, so the plant was not the controlled variable
([Wang et al., 2025](https://doi.org/10.1145/3728904)). A fixed-wing same-plant study can improve that
experimental control, especially if it adds formal safety oracles, metamorphic cases, or automated
test generation rather than only comparing tracking plots.

The scientific contribution should be the protocol, cases, measurements, and artifacts—not a claim
that one autopilot is universally better. Each autopilot should receive a documented, competent
tuning process rather than a comparison of arbitrary defaults.

Minimum scenario families:

- straight-and-level trim, commanded airspeed and altitude changes, turns, and waypoint tracking;
- longitudinal and lateral doublets, chirps, or multisines;
- deterministic gust/turbulence realizations and aerodynamic-parameter perturbations;
- actuator lag, rate limit, saturation, bias, and partial-loss cases;
- airspeed, IMU, GPS, and timing errors introduced at one explicitly named boundary; and
- duplicate, delayed, missing, and reordered simulator/autopilot packets.

Minimum outputs:

- tracking error, settling time, overshoot, control activity, saturation time, and failure recovery;
- estimator error separated from plant state;
- timing jitter, missed steps, real-time factor, and run-to-run dispersion;
- every resolved controller parameter and firmware revision; and
- a complete case bundle containing initial state, commands, realized disturbances, signals, stop
  reason, metrics, and provenance.

### 2. A formal verification and validation ladder

The project should label evidence by what it proves:

```text
equation and invariant tests                 code verification
        |
JSBSim mirrored-equation comparison          independent implementation check
        |
timestep and linearization convergence       numerical/solution verification
        |
headless versus Unreal wrench comparison     adapter and frame verification
        |
open-loop trajectory and modal comparisons   runtime integration evidence
        |
ArduPlane and PX4 benchmark cases             closed-loop application evidence
        |
held-out wind-tunnel or flight data           physical model validation
        |
parameter/posterior propagation               uncertainty quantification
```

Publishing the ladder, its machine-readable acceptance criteria, and the evidence artifacts can be a
contribution because current simulator papers often use “high fidelity” without identifying which
layer was actually tested.

### 3. Quantify when model fidelity changes a controls conclusion

Rather than attempting the most detailed model possible, test which omitted effects change the
answer to a declared controls question. Candidate ablations are actuator dynamics, propeller–airframe
coupling, nonlinear stall behavior, gust spectra, sensor latency, ground effect, and parameter
uncertainty.

The useful result is not “model B is more realistic.” It is, for example, “controller ranking,
stability margin, saturation rate, or recovery envelope changes beyond a stated threshold when this
effect is included.” That connects flight-model fidelity directly to research decisions.

### 4. Flight-data calibration and uncertainty propagation

This is the strongest eventual journal contribution and the highest validation burden. Collect
designed excitations on a known small fixed-wing aircraft, jointly estimate wind, actuator, propulsion,
and aerodynamic uncertainty, and evaluate on flights withheld from calibration. Propagate the
resulting parameter ensemble through both autopilots and report distributions—not only nominal lines—
for modes, tracking, saturation, and failure probability.

The current readable coefficient model is a good prior or mean model. A physics-informed Gaussian
process or learned residual could later model only the discrepancy, preserving symmetry, zero-load at
zero dynamic pressure, frame conventions, and interpretable derivatives. A black-box replacement for
the entire plant would sacrifice a major project advantage.

### 5. Controls-readable research software

Controls readability is not enough for a major dynamics paper, but it is a real adoption advantage.
The same symbol names, SI units, NED/FRD frames, equation staging, trim definition, Jacobian convention,
and signal semantics should appear in code, documentation, cases, and results. MATLAB users should be
able to audit the equations and consume the linear models without reverse-engineering Unreal types or
C++ application plumbing.

This strengthens every other contribution and could support a software paper once the system has a
stable release, external research use, archival examples, automated installation, and a DOI-backed
artifact.

The artifact should make reproduction a first-class research output. Teetaert et al. identify shared,
repeatable benchmarks and remote sim-to-real access as mechanisms for improving robotics
reproducibility ([2025](https://doi.org/10.1109/MRA.2025.3527291)). For this project, the corresponding
deliverable is smaller and more controllable: immutable case manifests, pinned firmware and
parameters, saved disturbances, replayable logs, declared tolerances, and one command that regenerates
each table or figure.

## ArduPilot or PX4?

### Recommendation

**Implement ArduPlane first; do not make ArduPilot the permanent scientific boundary. Add PX4 second
as a controlled comparison.**

ArduPilot is the right first vertical slice because:

- ArduPlane is operationally mature and fixed-wing support is central to the project;
- the documented JSON/UDP backend is deliberately easy for an external physics simulator to implement;
- the current timing, PWM, sensor, and Docker design already targets that interface; and
- completing one real path is more valuable than maintaining two half-built adapters.

PX4 is the right second backend because:

- its Gazebo, ROS 2, offboard-autonomy, and academic ecosystem is highly visible;
- official simulation support includes multiple fixed-wing and VTOL paths;
- its BSD licensing and modular interfaces are attractive to research and commercial users; and
- holding the plant constant turns a second adapter into a scientific comparison rather than a feature.

Supporting only ArduPilot could still produce a useful open-source tool and an ArduPlane-focused case
study. It gives up the clearest comparative research question and makes the work easier to dismiss as
one more integration. Supporting both before the ArduPlane path is validated would create unnecessary
schedule risk. The sequence—not an either/or choice—is the answer.

Architecturally, neither autopilot should leak into `uvd_core`, aircraft files, atmosphere, trim,
linearization, or generic case semantics. Adapter-specific configuration can remain adapter-specific.
The common boundary should express physical effectors going into the plant and timestamped simulated
measurements coming out.

## Naming

The project has adopted `UnrealVehicleDynamics`, matching the existing `uvd` executable and its
intended scope across aerial and marine vehicles. It avoids making ArduPilot the permanent identity
or a future PX4 adapter look like an exception. The name still has two important caveats:

1. “Unreal” ties the identity to a renderer even though the strongest research contribution is the headless
   experimental contract.
2. Unreal is a third-party name. Epic's current EULA grants no general trademark right
   beyond required notices and directs further trademark use to its branding process
   ([Epic EULA](https://www.unrealengine.com/eula/unreal)). This is a branding risk to review, not a
   legal conclusion.

The naming implications are therefore:

- keep `uvd` as the CLI;
- describe the artifact in papers with a precise, non-grandiose subtitle such as “a deterministic,
  controls-oriented vehicle simulation and validation framework”;
- keep the implementation autopilot-neutral while accurately saying “ArduPlane adapter” until PX4
  passes the same contract; and
- review Epic's current trademark guidance before the first public archival release.

## Recommended publication program

### Stage 1: validated software foundation

Finish the current headless benchmark, document a declared operating envelope, and add the Unreal and
ArduPlane paths. Publish versioned cases, numerical convergence, force/moment comparison, trim and mode
tables, open-loop excitations, deterministic replay, and headless/Unreal equivalence measurements.

This is suitable for a research-software venue such as JOSS or SoftwareX only when the repository has a
stable public release, archival DOI, complete installation path, examples, and evidence of research
use. An aerospace simulation conference paper is another reasonable first outlet. Do not claim
physical Aerosonde validation.

### Stage 2: same-plant autopilot benchmark

Add PX4 without changing the plant or cases. Publish the benchmark protocol, tuned configurations,
scenario corpus, cross-autopilot results, and complete run artifacts. ICUAS, AIAA SciTech/Aviation,
or a simulation and information-systems venue are plausible targets; an expanded study could fit the
Journal of Aerospace Information Systems.

### Stage 3: calibrated model and uncertainty

Partner with a lab or obtain access to an instrumented fixed-wing aircraft. Use designed open- and
closed-loop excitations, calibrate on a subset, and validate on held-out flights. Propagate uncertainty
through both closed loops. This is the path toward the Journal of Aircraft, Journal of Guidance,
Control, and Dynamics, or CEAS Aeronautical Journal.

### Stage 4: application paper

Use Unreal and Cesium for a problem that actually needs them: terrain-following safety, autonomous
landing, perception in geospatial scenes, wind-aware inspection, or ship/shore operations. The
application—not rendering itself—must create the research question.

Marine work should remain out of the first fixed-wing paper. HoloOcean 2.0 and current ASV simulators
already combine Unreal-class rendering, Fossen-style models, sensors, and ROS. A later marine paper
would need domain-standard validation such as decay, RAO, turning-circle, or zig-zag evidence, or a
genuinely shared cross-domain experiment contract.

## Claims to avoid

| Claim | Use only when |
|---|---|
| “High fidelity” | The variable, envelope, reference, uncertainty, and quantitative error are stated |
| “Validated against JSBSim” | Clarified as implementation/reference agreement, not physical validation |
| “Digital twin” | A particular physical asset supplies continuing calibration or synchronization data |
| “Autopilot agnostic” | At least two autopilots pass the same automated contract |
| “Deterministic simulator” | The exact deterministic boundary is named; full SITL scheduling variance is measured separately |
| “Same experiment” | Commands, realized disturbances, sensors, timing, initial state, parameters, and scoring are identical |
| “First” | A formal, current novelty search supports the exact qualified statement |
| “Better autopilot” | Tuning, modes, estimator assumptions, failures, and statistical uncertainty make the comparison fair |

## Concrete next research milestones

1. Freeze a versioned fixed-wing benchmark specification and declared validity envelope.
2. Record coefficient provenance and distinguish educational parameters from measured ones.
3. Publish reference force/moment grids, trims across airspeed, discrete linear models, modal tables,
   doublets, and timestep-convergence results.
4. Add actuator dynamics and a deterministic, saved gust realization before comparing controllers.
5. Complete the ArduPlane adapter and quantify packet, sensor, and zero-order-hold timing.
6. Prove headless and Unreal matched-state wrench agreement, then measure trajectory convergence and
   sensitivity to rendering load.
7. Add PX4 through the same physical boundary and run the identical scenario corpus.
8. Archive every release and benchmark result with exact firmware, compiler, configuration, and data
   provenance.
9. Obtain real fixed-wing data or a research partner before using digital-twin or physical-fidelity
   language.

## Selected references and current software sources

### Peer-reviewed and proceedings sources

- Aziz, F., and Loya, A. “Multi-Mode MAVLink Test Bench AeroQT for Aircraft Simulation and Real-Flight
  Validation.” *Aerospace Research Communications* 3, 2025.
  [doi:10.3389/arc.2025.14524](https://doi.org/10.3389/arc.2025.14524).
- Cavalheiro, F. S., and Guerreiro, B. J. “Fixed-Wing UAV Simulation in PX4 and Gazebo: An AVL-Based
  Approach.” *YEF-ECE*, 2025.
  [doi:10.1109/YEF-ECE66503.2025.11117433](https://doi.org/10.1109/YEF-ECE66503.2025.11117433).
- Cea, A., and Palacios, R. “JAX-Based Aeroelastic Simulation Engine for Differentiable Aircraft
  Dynamics.” *Computer Physics Communications*, 2025.
  [doi:10.1016/j.cpc.2025.109547](https://doi.org/10.1016/j.cpc.2025.109547).
- Dimmig, C. A., Silano, G., McGuire, K., Gabellieri, C., Hönig, W., Moore, J. L., and Kobilarov, M.
  “Survey of Simulators for Aerial Robots: An Overview and In-Depth Systematic Comparisons.”
  *IEEE Robotics & Automation Magazine* 32(2), 153–166, 2025.
  [doi:10.1109/MRA.2024.3433171](https://doi.org/10.1109/MRA.2024.3433171).
- Figueira, J. C., et al. “Nonlinear Aero-Propulsive Modeling for Fixed-Wing eVTOL UAV from Flight Test
  Data.” *Journal of Aircraft* 62(2), 300–312, 2025.
  [doi:10.2514/1.C037964](https://doi.org/10.2514/1.C037964).
- Jayanti, E. B., et al. “Validation of a Small UAV Dynamic Model Using CFD and Flight Test Data.”
  *Periodica Polytechnica Transportation Engineering* 54(1), 88–104, 2026.
  [doi:10.3311/PPtr.41190](https://doi.org/10.3311/PPtr.41190).
- García-Gascón, C., Bas-Bolufer, J., Castelló-Pedrero, P., and García-Manrique, J. A. “An Open
  Benchmark Dataset for Machine Learning and Intelligent Trajectory Optimization in Fixed-Wing
  Unmanned Aerial Systems.” *Scientific Data* 13, 364, 2026.
  [doi:10.1038/s41597-026-06716-3](https://doi.org/10.1038/s41597-026-06716-3).
- Lee, U., Kim, T., and Lee, S. “An Aviation Manned-Unmanned Teaming Simulation in Urban Environments
  to Compare Autonomous Flight Formations.” *DASC*, 2025.
  [doi:10.1109/DASC66011.2025.11257249](https://doi.org/10.1109/DASC66011.2025.11257249).
- Løw-Hansen, B., et al. “Modeling and Identification of a Small Fixed-Wing UAV Using Estimated
  Aerodynamic Angles.” *CEAS Aeronautical Journal*, 2025.
  [doi:10.1007/s13272-025-00816-3](https://doi.org/10.1007/s13272-025-00816-3).
- Matt, J. J., Chao, H., Shawon, M. H., Svoboda, B. C., and Hagerott, S. G. “System Identification for
  Small Flying-Wing Unmanned Aircraft Using Open-Loop and Closed-Loop Flight Data.” *Journal of
  Aircraft* 62(4), 961–977, 2025.
  [doi:10.2514/1.C038147](https://doi.org/10.2514/1.C038147).
- Michek, N. E., Mehta, P. M., and Huebsch, W. W. “Flight Dynamics Modeling Using Physics-Informed
  Neural Networks.” *AIAA Journal*, 2025.
  [doi:10.2514/1.J063991](https://doi.org/10.2514/1.J063991).
- Morais, J. E., et al. “ProVANT Simulator: A Virtual Unmanned Aerial Vehicle Platform for Control
  System Development.” *Aerospace* 12(9), 762, 2025.
  [doi:10.3390/aerospace12090762](https://doi.org/10.3390/aerospace12090762).
- Andrade, F. A. A., Sivertsen, A., Moura, M., et al. “Unreal Engine 5 Simulations of Solar Plant
  Inspections by Unmanned Aerial Systems with Robot Operating System 2.” *IFAC-PapersOnLine* 59(10),
  1173–1178, 2025.
  [doi:10.1016/j.ifacol.2025.09.198](https://doi.org/10.1016/j.ifacol.2025.09.198).
- Siswantara, A. I., et al. “Flight Test Validation of High-Fidelity Overset Mesh Predictions for UAV
  Longitudinal Dynamics.” *Theoretical and Applied Mechanics Letters*, early access, 2026.
  [doi:10.1016/j.taml.2026.100702](https://doi.org/10.1016/j.taml.2026.100702).
- Valencia, E., et al. “An Open-Source UAV Digital Twin Framework: A Case Study on Remote Sensing in
  the Andean Mountains.” *Journal of Intelligent & Robotic Systems* 111, 71, 2025.
  [doi:10.1007/s10846-025-02276-7](https://doi.org/10.1007/s10846-025-02276-7).
- Wang, S., Dong, Z., Li, H., Shen, L., Peng, X., and She, D. “RouthSearch: Inferring PID Parameter
  Specification for Flight Control Program by Coordinate Search.” *Proceedings of the ACM on Software
  Engineering* 2(ISSTA), 640–662, 2025.
  [doi:10.1145/3728904](https://doi.org/10.1145/3728904).
- Zhang, T., Grzelak, D., Zhao, W., Islam, M. A., Fricke, H., and Aßmann, U. “A Review on the
  Construction, Modeling, and Consistency of Digital Twins for Advanced Air Mobility Applications.”
  *Drones* 9(6), 394, 2025.
  [doi:10.3390/drones9060394](https://doi.org/10.3390/drones9060394).
- Teetaert, S., Zhao, W., Loquercio, A., et al. “Advancing Reproducibility, Benchmarks, and Education
  With Remote Sim2real: Remote Simulation to Real Robot Hardware.” *IEEE Robotics & Automation
  Magazine* 32(1), 117–123, 2025.
  [doi:10.1109/MRA.2025.3527291](https://doi.org/10.1109/MRA.2025.3527291).
- Zhang, Y., Hu, Y., Song, Y., et al. “Learning Vision-Based Agile Flight via Differentiable Physics.”
  *Nature Machine Intelligence* 7, 954–966, 2025.
  [doi:10.1038/s42256-025-01048-0](https://doi.org/10.1038/s42256-025-01048-0).

### Preprints and emerging work

- Embley-Riches, J., Liu, J., Julier, S., and Kanoulas, D. “Unreal Robotics Lab: A High-Fidelity
  Robotics Simulator with Advanced Physics and Rendering.” ICRA 2026 / 2025 preprint.
  [arXiv:2504.14135](https://arxiv.org/abs/2504.14135).
- Guevara, B. S., Moya, V., Gandolfo, D., and Toibero, J. M. “Development and Experimental Validation
  of UAV Digital Twins.” 2025 preprint.
  [doi:10.2139/ssrn.5145160](https://doi.org/10.2139/ssrn.5145160).
- Harp, D. I., Ott, J., Asmar, D. M., Alora, J., and Kochenderfer, M. J. “Physics-Informed Gaussian
  Processes for Safe Envelope Expansion.” 2025 preprint; journal version published in 2026.
  [arXiv:2501.01000](https://arxiv.org/abs/2501.01000).
- Li, Z., Tao, T., Fu, R., Wang, L., Zhang, D., and Quan, Q. “RflyUT-Sim: A Simulation Platform for
  Development and Testing of Complex Low-Altitude Traffic Control.” 2025 preprint.
  [arXiv:2512.24112](https://arxiv.org/abs/2512.24112).
- Miao, Y., Shen, W., Cui, H., and Mitra, S. “FalconWing: An Open-Source Platform for Ultra-Light
  Fixed-Wing Aircraft Research.” 2025 preprint/workshop paper.
  [arXiv:2505.01383](https://arxiv.org/abs/2505.01383).
- Panerati, J., Sajjadi, S., Soleymanpour, S., Mehta, V., and Mantegh, I. “aerial-autonomy-stack—a
  Faster-than-Real-Time, Autopilot-Agnostic, ROS2 Framework to Simulate and Deploy Perception-Based
  Drones.” ICUAS 2026 / preprint.
  [arXiv:2602.07264](https://arxiv.org/abs/2602.07264).
- Reid, I., Ritchie, J., Moore, J., Sutherland, B., Snow, G., Tokumaru, P., and McLain, T. “ROSplane
  2.0: A Fixed-Wing Autopilot for Research.” ICUAS 2026 / 2025 preprint.
  [arXiv:2510.01041](https://arxiv.org/abs/2510.01041).
- Bolz, W., Faber, F., Lork, J., Cella, M., Zendel, O., and d'Apolito, F. “An Integrated Testbed for
  Mission-Level Autonomy Evaluation in Evolving Disaster Scenarios with Fixed-Wing Swarms.” ICUAS
  2026, 696–703. No public manuscript or indexed DOI was found; evidence is limited to the
  [official program](https://controls.papercept.net/conferences/scripts/rtf/ICUAS26_ContentListWeb_3.html).
- “FALCON-S: Fixed-Wing Aerodynamics and Learning Control Suite.” Withdrawn ICLR 2026 submission;
  retained only as evidence of an emerging direction.
  [OpenReview](https://openreview.net/forum?id=BIDlQ5ifz4).

### Official software and standards sources

- [ArduPilot SITL overview](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html),
  [external JSON interface](https://ardupilot.org/dev/docs/sitl-with-JSON.html),
  [JSBSim integration](https://ardupilot.org/dev/docs/sitl-with-jsbsim.html), and
  [Gazebo plugin](https://github.com/ArduPilot/ardupilot_gazebo).
- [PX4 simulation overview](https://docs.px4.io/main/en/simulation/),
  [current Gazebo integration](https://docs.px4.io/v1.17/en/sim_gazebo_gz/), and
  [legacy/community JSBSim integration](https://docs.px4.io/v1.12/en/simulation/jsbsim/).
- [JSBSim](https://github.com/JSBSim-Team/jsbsim) and its
  [Unreal reference application](https://github.com/JSBSim-Team/jsbsim/blob/master/UnrealEngine/README.md).
- [Microsoft AirSim](https://github.com/microsoft/AirSim),
  [Colosseum archive](https://github.com/CodexLabsLLC/Colosseum), and
  [Cosys-AirSim](https://github.com/Cosys-Lab/Cosys-AirSim).
- [AeroSim](https://github.com/aerosim-open/aerosim) and its
  [architecture documentation](https://aerosim.readthedocs.io/en/latest/).
- [Project AirSim](https://github.com/iamaisim/ProjectAirSim) and its
  [simulation clock documentation](https://iamaisim.github.io/ProjectAirSim/development/scene/sim_clock_internal.html).
- [PteroSim](https://github.com/PteroLabsAI/PteroSim-UAV-Simulator) (proprietary vendor claims only).
- [NASA/FAA VVUQ report, 2026](https://ntrs.nasa.gov/api/citations/20260001729/downloads/NASA-TM-20260001729.pdf)
  and [ABS model/simulation/digital-twin V&V guidance, 2024](https://ww2.eagle.org/content/dam/eagle/rules-and-guides/current/design_and_analysis/348-guidance-notes-on-verification-and-validation-of-models%2C-simulations%2C-and-digital-twins-2024/348-vandv-gn-nov24.pdf).

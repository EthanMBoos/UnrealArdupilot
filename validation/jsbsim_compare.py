#!/usr/bin/env python3
"""Compare uvd's pure aero model with an aero-only JSBSim fixture."""

import argparse
import json
import math
import pathlib
import random
import subprocess
import sys

FT_PER_M = 3.280839895013123
LBF_TO_N = 4.4482216152605
LBFFT_TO_NM = 1.3558179483314004
SLUGFT3_TO_KGM3 = 515.3788183931961
SPAN = 2.8956
CHORD = 0.18994


def make_cases(count, seed):
    rng = random.Random(seed)
    cases = []
    boundaries = [
        (18.0, math.radians(-10), math.radians(-10)),
        (18.0, math.radians(12), math.radians(10)),
        (40.0, math.radians(-10), math.radians(10)),
        (40.0, math.radians(12), math.radians(-10)),
    ]
    for speed, alpha, beta in boundaries:
        cases.append((speed, alpha, beta, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    # Basis cases exercise every rate and control independently in both signs.
    for index in range(6):
        for sign in (-1.0, 1.0):
            values = [25.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0]
            values[3 + index] = sign * (0.1 if index < 3 else math.radians(10))
            cases.append(tuple(values))
    while len(cases) < count:
        cases.append((
            rng.uniform(18, 40), rng.uniform(math.radians(-10), math.radians(12)),
            rng.uniform(math.radians(-10), math.radians(10)), rng.uniform(-0.2, 0.2),
            rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2),
            rng.uniform(math.radians(-15), math.radians(15)),
            rng.uniform(math.radians(-15), math.radians(15)),
            rng.uniform(math.radians(-15), math.radians(15)), rng.uniform(0, 3000)))
    return cases[:count]


def jsbsim_values(fdm, case):
    speed, alpha, beta, phat, qhat, rhat, da, de, dr, altitude = case
    u = speed * math.cos(beta) * math.cos(alpha)
    v = speed * math.sin(beta)
    w = speed * math.cos(beta) * math.sin(alpha)
    fdm["ic/h-sl-ft"] = altitude * FT_PER_M
    fdm["ic/u-fps"], fdm["ic/v-fps"], fdm["ic/w-fps"] = u * FT_PER_M, v * FT_PER_M, w * FT_PER_M
    fdm["ic/p-rad_sec"] = phat * 2 * speed / SPAN
    fdm["ic/q-rad_sec"] = qhat * 2 * speed / CHORD
    fdm["ic/r-rad_sec"] = rhat * 2 * speed / SPAN
    fdm["ic/phi-rad"] = fdm["ic/theta-rad"] = fdm["ic/psi-true-rad"] = 0.0
    if not fdm.run_ic():
        raise RuntimeError("JSBSim RunIC failed")
    fdm["uvd/aileron-rad"], fdm["uvd/elevator-rad"], fdm["uvd/rudder-rad"] = da, de, dr
    before = fdm.get_sim_time()
    fdm.suspend_integration()
    if not fdm.run():
        raise RuntimeError("JSBSim suspended evaluation failed")
    if fdm.get_sim_time() != before:
        raise RuntimeError("JSBSim state advanced during static evaluation")
    density = fdm["atmosphere/rho-slugs_ft3"] * SLUGFT3_TO_KGM3
    # Feed the state JSBSim actually committed back to the core. This removes
    # feet/metres initialization roundoff from a coefficient-level comparison.
    committed_velocity = [fdm["velocities/u-fps"]/FT_PER_M,fdm["velocities/v-fps"]/FT_PER_M,fdm["velocities/w-fps"]/FT_PER_M]
    committed_speed = math.sqrt(sum(value*value for value in committed_velocity))
    committed_rates = [fdm["aero/bi2vel"]*fdm["velocities/p-aero-rad_sec"]*2*committed_speed/SPAN,
                       fdm["aero/ci2vel"]*fdm["velocities/q-aero-rad_sec"]*2*committed_speed/CHORD,
                       fdm["aero/bi2vel"]*fdm["velocities/r-aero-rad_sec"]*2*committed_speed/SPAN]
    probe = {
        "state": {"position_ned_m":[0,0,0], "q_body_to_ned":[1,0,0,0],
                  "velocity_body_mps":committed_velocity,
                  "omega_body_radps":committed_rates},
        "effectors":{"aileron_rad":da,"elevator_rad":de,"rudder_rad":dr,"throttle":0},
        "altitude_msl_m":altitude,"density_kgpm3":density,
    }
    reference = {
        "air_data":[fdm["velocities/vtrue-fps"] / FT_PER_M, fdm["aero/alpha-rad"], fdm["aero/beta-rad"]],
        "qbar_pa": fdm["aero/qbar-psf"] * 47.88025898033584,
        "coefficients":[fdm["uvd/CD"],fdm["uvd/CL"],fdm["uvd/CY"],fdm["uvd/Cl"],fdm["uvd/Cm"],fdm["uvd/Cn"]],
        "force_n":[fdm["forces/fbx-aero-lbs"]*LBF_TO_N,fdm["forces/fby-aero-lbs"]*LBF_TO_N,fdm["forces/fbz-aero-lbs"]*LBF_TO_N],
        "moment_nm":[fdm["moments/l-aero-lbsft"]*LBFFT_TO_NM,fdm["moments/m-aero-lbsft"]*LBFFT_TO_NM,fdm["moments/n-aero-lbsft"]*LBFFT_TO_NM],
    }
    fdm.resume_integration()
    return probe, reference


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uvd", default="build/uvd")
    parser.add_argument("--run", default="examples/run.json")
    parser.add_argument("--samples", type=int, default=50)
    parser.add_argument("--seed", type=int, default=20260817)
    args = parser.parse_args()
    try:
        import jsbsim
    except ImportError as exc:
        raise SystemExit("Install the optional `jsbsim` Python package to run this check") from exc
    version = getattr(jsbsim, "__version__", "unknown")
    fixture_root = pathlib.Path(__file__).resolve().parent / "reference" / "jsbsim"
    fdm = jsbsim.FGFDMExec(str(fixture_root))
    fdm.set_debug_level(0)
    fdm.set_aircraft_path(str(fixture_root))
    # Create the three direct-effector properties before the XML functions bind.
    fdm.set_property_value("uvd/aileron-rad", 0.0)
    fdm.set_property_value("uvd/elevator-rad", 0.0)
    fdm.set_property_value("uvd/rudder-rad", 0.0)
    if not fdm.load_model("aerosonde_aero"):
        raise SystemExit("Could not load aero-only JSBSim fixture")
    probes, references = zip(*(jsbsim_values(fdm, case) for case in make_cases(args.samples, args.seed)))
    actual = []
    for probe in probes:
        process = subprocess.run(
            [args.uvd, "evaluate", args.run, "--input", "-"],
            input=json.dumps(probe), text=True, capture_output=True, check=True)
        actual.append(json.loads(process.stdout))
    worst = {"air_data":0.0,"coefficients":0.0,"force_n":0.0,"moment_nm":0.0}
    for index, (got, ref) in enumerate(zip(actual, references)):
        got_air = [got["air_data"]["tas_mps"],got["air_data"]["alpha_rad"],got["air_data"]["beta_rad"]]
        qbar_s = ref["qbar_pa"] * 0.55
        groups = (("air_data",got_air,ref["air_data"],1e-10,0.0),
                  ("coefficients",got["coefficients"],ref["coefficients"],1e-10,1e-9),
                  ("force_n",got["aerodynamic_force_n"],ref["force_n"],1e-5,1e-8*qbar_s),
                  ("moment_nm",got["aerodynamic_moment_nm"],ref["moment_nm"],1e-6,1e-8*qbar_s*max(SPAN,CHORD)))
        for name, lhs, rhs, absolute, relative_scale in groups:
            for a, b in zip(lhs, rhs):
                error = abs(a-b); worst[name] = max(worst[name], error)
                tolerance = absolute + (relative_scale*abs(b) if name == "coefficients" else relative_scale)
                if error > tolerance:
                    raise SystemExit(f"case {index} {name} mismatch: got {a}, ref {b}, error {error}")
    print(json.dumps({"passed":True,"samples":len(actual),"seed":args.seed,"jsbsim_version":version,"worst_absolute":worst}, indent=2))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Compare final states at successively halved RK4 timesteps."""

import argparse
import json
import math
import subprocess


def simulate(uvd, run, duration, dt):
    process = subprocess.run(
        [uvd, "simulate", run, "--duration", str(duration), "--dt", str(dt)],
        text=True, capture_output=True, check=True)
    return json.loads(process.stdout)["final_state"]


def distance(left, right, field):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left[field], right[field])))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uvd", default="build/uvd")
    parser.add_argument("--run", default="examples/run.json")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--dt", type=float, default=1 / 60)
    args = parser.parse_args()

    coarse = simulate(args.uvd, args.run, args.duration, args.dt)
    medium = simulate(args.uvd, args.run, args.duration, args.dt / 2)
    fine = simulate(args.uvd, args.run, args.duration, args.dt / 4)
    result = {"passed": True, "duration_s": args.duration, "timesteps_s": [args.dt, args.dt / 2, args.dt / 4]}
    for name in ("position_ned_m", "velocity_body_mps", "omega_body_radps"):
        coarse_error = distance(coarse, medium, name)
        fine_error = distance(medium, fine, name)
        result[name] = {
            "coarse_to_medium": coarse_error,
            "medium_to_fine": fine_error,
            "error_reduction": coarse_error / fine_error if fine_error else None,
        }
        if fine_error > coarse_error:
            result["passed"] = False
    print(json.dumps(result, indent=2))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

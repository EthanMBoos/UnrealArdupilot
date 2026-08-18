#!/usr/bin/env python3
"""Exercise the complete deterministic headless v1 workflow."""

import argparse
import json
import pathlib
import subprocess
import tempfile


ROOT = pathlib.Path(__file__).resolve().parents[1]


def run(command, *, expect_success=True):
    process = subprocess.run(command, text=True, capture_output=True)
    if (process.returncode == 0) != expect_success:
        raise RuntimeError(
            f"unexpected exit {process.returncode}: {' '.join(map(str, command))}\n"
            f"stdout:\n{process.stdout}\nstderr:\n{process.stderr}"
        )
    return process


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def require_passed(path):
    value = json.loads(path.read_text())
    if not value.get("passed"):
        raise RuntimeError(f"acceptance report did not pass: {path}")
    return value


def exercise_bad_inputs(uvd, root, run_config, unreal_config, aircraft_config):
    failures = {}

    bad_aircraft = dict(aircraft_config)
    bad_aircraft["mass_kg"] = -1
    path = root / "bad-mass.json"
    write_json(path, bad_aircraft)
    failures["negative_mass"] = run([uvd, "validate", path], expect_success=False).returncode

    bad_aircraft = json.loads(json.dumps(aircraft_config))
    bad_aircraft["channel_map"][1]["function"] = "aileron"
    path = root / "duplicate-channel-function.json"
    write_json(path, bad_aircraft)
    failures["duplicate_channel_function"] = run(
        [uvd, "validate", path], expect_success=False
    ).returncode

    bad_run = json.loads(json.dumps(run_config))
    bad_run["unknown"] = True
    path = root / "unknown-run-field.json"
    write_json(path, bad_run)
    failures["unknown_field"] = run([uvd, "validate", path], expect_success=False).returncode

    bad_run = json.loads(json.dumps(run_config))
    bad_run["schema_version"] = 2
    path = root / "unsupported-schema.json"
    write_json(path, bad_run)
    failures["unsupported_schema"] = run(
        [uvd, "validate", path], expect_success=False
    ).returncode

    bad_run = json.loads(json.dumps(run_config))
    bad_run["controls"]["schedule"] = list(reversed(bad_run["controls"]["schedule"]))
    path = root / "out-of-order-schedule.json"
    write_json(path, bad_run)
    failures["out_of_order_schedule"] = run(
        [uvd, "validate", path], expect_success=False
    ).returncode

    bad_run = json.loads(json.dumps(run_config))
    bad_run["controls"]["schedule"][0]["values"]["throttle"] = 2.0
    path = root / "control-limit.json"
    write_json(path, bad_run)
    failures["control_limit"] = run(
        [uvd, "validate", path], expect_success=False
    ).returncode

    bad_run = json.loads(json.dumps(run_config))
    bad_run["clock"]["motion_solver"] = "chaos"
    path = root / "headless-chaos.json"
    write_json(path, bad_run)
    failures["headless_solver"] = run(
        [uvd, "validate", path], expect_success=False
    ).returncode

    bad_run = json.loads(json.dumps(unreal_config))
    bad_run["clock"]["fixed_dt_s"] = 0.007
    path = root / "nonintegral-controller-rate.json"
    write_json(path, bad_run)
    failures["controller_rate"] = run(
        [uvd, "validate", path], expect_success=False
    ).returncode
    return failures


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uvd", default=str(ROOT / "build" / "uvd"))
    parser.add_argument("--report")
    args = parser.parse_args()
    uvd = str(pathlib.Path(args.uvd).resolve())

    source_run_path = ROOT / "examples" / "runs" / "headless.json"
    source_aircraft_path = ROOT / "examples" / "aircraft" / "aerosonde.json"
    source_unreal_path = ROOT / "examples" / "runs" / "unreal_ardupilot.json"
    source_unreal_smoke_path = ROOT / "examples" / "runs" / "unreal_smoke.json"
    source_run = json.loads(source_run_path.read_text())
    source_unreal = json.loads(source_unreal_path.read_text())
    source_aircraft = json.loads(source_aircraft_path.read_text())

    with tempfile.TemporaryDirectory(prefix="uvd-headless-acceptance-") as temporary:
        root = pathlib.Path(temporary)
        aircraft_path = root / "aircraft.json"
        run_path = root / "run.json"
        write_json(aircraft_path, source_aircraft)
        source_run["aircraft"] = {"path": "aircraft.json"}
        source_unreal["aircraft"] = {"path": "aircraft.json"}
        source_run["output_root"] = "runs"
        write_json(run_path, source_run)

        run([uvd, "validate", aircraft_path])
        run([uvd, "validate", run_path])
        run([uvd, "validate", source_unreal_smoke_path])
        run([uvd, "trim", run_path, "--output", root / "trim"])
        run([uvd, "linearize", run_path, "--output", root / "linear"])
        run([uvd, "simulate", run_path, "--output", root / "simulation"])
        run([uvd, "replay", root / "simulation"])

        trim = require_passed(root / "trim" / "results" / "operating_point.json")
        linear = require_passed(root / "linear" / "results" / "linear_model.json")
        replay = require_passed(
            root / "simulation" / "replay" / "results" / "replay_comparison.json"
        )
        if not replay.get("byte_identical"):
            raise RuntimeError("headless replay was numerically equal but not byte-identical")

        report = {
            "schema_version": 1,
            "passed": True,
            "trim": {
                "iterations": trim["solver"]["iterations"],
                "max_residual": max(
                    *(
                        abs(value)
                        for group in (
                            trim["residuals"]["velocity_dot_body_mps2"],
                            trim["residuals"]["omega_dot_body_radps2"],
                        )
                        for value in group
                    ),
                    abs(trim["residuals"]["true_airspeed_error_mps"]),
                    abs(trim["residuals"]["velocity_ned_down_mps"]),
                ),
            },
            "linearization": linear["convergence"],
            "replay": {"byte_identical": True},
            "rejected_inputs": exercise_bad_inputs(
                uvd, root, source_run, source_unreal, source_aircraft
            ),
        }

    rendered = json.dumps(report, indent=2) + "\n"
    print(rendered, end="")
    if args.report:
        report_path = pathlib.Path(args.report)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(rendered)


if __name__ == "__main__":
    main()

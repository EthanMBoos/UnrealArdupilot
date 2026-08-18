#!/usr/bin/env python3
"""Run and aggregate the Unreal U0 Chaos mechanics and pacing probes."""

import argparse
import copy
import datetime
import json
import math
import pathlib
import subprocess


ROOT = pathlib.Path(__file__).resolve().parents[1]
PHYSICS_RATES_HZ = (60, 120, 240)
RENDER_RATES_HZ = (30, 60, 144)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n")


def vector_error(left, right):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def state_errors(left, right):
    return {
        "position_ned_m": vector_error(left["position_ned_m"], right["position_ned_m"]),
        "q_body_to_ned": vector_error(left["q_body_to_ned"], right["q_body_to_ned"]),
        "velocity_body_mps": vector_error(
            left["velocity_body_mps"], right["velocity_body_mps"]
        ),
        "omega_body_radps": vector_error(
            left["omega_body_radps"], right["omega_body_radps"]
        ),
    }


def state_errors_pass(errors):
    return (
        errors["position_ned_m"] <= 2e-3
        and errors["q_body_to_ned"] <= 2e-4
        and errors["velocity_body_mps"] <= 2e-3
        and errors["omega_body_radps"] <= 2e-4
    )


def configure_case(template, name, physics_hz, render_hz, kind="aircraft"):
    value = copy.deepcopy(template)
    final_tick = physics_hz
    value["run_id"] = name
    value["aircraft"] = {
        "path": str((ROOT / "examples" / "aircraft" / "aerosonde.json").resolve())
    }
    value["clock"]["fixed_dt_s"] = 1.0 / physics_hz
    value["stop"] = {"final_tick": final_tick}
    value["unreal_probe"] = {"kind": kind, "render_rate_hz": render_hz}
    value["controls"]["schedule"] = [
        {
            "apply_tick": 0,
            "values": {
                "aileron": 0.0,
                "elevator": -0.08,
                "rudder": 0.0,
                "throttle": 0.55,
            },
        },
        {"apply_tick": final_tick // 2, "values": {"elevator": -0.07}},
        {"apply_tick": 3 * final_tick // 4, "values": {"elevator": -0.08}},
    ]
    if kind != "aircraft":
        value["initial_state"] = {
            "position_ned_m": [0.0, 0.0, -100.0],
            "q_body_to_ned": [1.0, 0.0, 0.0, 0.0],
            "velocity_body_mps": [0.0, 0.0, 0.0],
            "omega_body_radps": [0.0, 0.0, 0.0],
        }
        value["controls"]["schedule"] = [
            {
                "apply_tick": 0,
                "values": {
                    "aileron": 0.0,
                    "elevator": 0.0,
                    "rudder": 0.0,
                    "throttle": 0.0,
                },
            }
        ]
    return value


def run_case(uvd, root, name, config):
    config_path = root / "configs" / f"{name}.json"
    bundle = root / "cases" / name
    write_json(config_path, config)
    print(f"running {name}", flush=True)
    process = subprocess.run(
        [str(uvd), "unreal", str(config_path), "--output", str(bundle)],
        text=True,
        capture_output=True,
    )
    report_path = bundle / "results" / "unreal_smoke.json"
    if not report_path.exists():
        raise RuntimeError(
            f"U0 case {name} produced no report (exit {process.returncode})\n"
            f"stdout:\n{process.stdout}\nstderr:\n{process.stderr}"
        )
    report = json.loads(report_path.read_text())
    if process.returncode != 0 or not report.get("passed"):
        raise RuntimeError(
            f"U0 case {name} failed (exit {process.returncode}):\n"
            f"{json.dumps(report, indent=2)}\nstdout:\n{process.stdout}\n"
            f"stderr:\n{process.stderr}"
        )
    print(f"passed  {name}", flush=True)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uvd", default=str(ROOT / "build" / "uvd"))
    parser.add_argument("--output")
    args = parser.parse_args()

    uvd = pathlib.Path(args.uvd).resolve()
    if not uvd.exists():
        raise SystemExit(f"uvd executable does not exist: {uvd}")
    if args.output:
        output = pathlib.Path(args.output).resolve()
    else:
        stamp = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
        output = ROOT / "runs" / f"unreal_u0_{stamp}"
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)

    template = json.loads((ROOT / "examples" / "runs" / "unreal_u0.json").read_text())
    cases = {}
    case_summaries = []
    for physics_hz in PHYSICS_RATES_HZ:
        for render_hz in RENDER_RATES_HZ:
            name = f"matrix_p{physics_hz}_r{render_hz}"
            config = configure_case(template, name, physics_hz, render_hz)
            report = run_case(uvd, output, name, config)
            cases[name] = report
            case_summaries.append(
                {
                    "name": name,
                    "physics_rate_hz": physics_hz,
                    "render_rate_hz": render_hz,
                    "metrics": report["metrics"],
                }
            )

    comparisons = []
    render_cap_observations = []
    for physics_hz in PHYSICS_RATES_HZ:
        reference = cases[f"matrix_p{physics_hz}_r60"]["final_state"]
        for render_hz in (30, 144):
            name = f"p{physics_hz}_r{render_hz}_vs_r60"
            errors = state_errors(
                cases[f"matrix_p{physics_hz}_r{render_hz}"]["final_state"],
                reference,
            )
            comparisons.append(
                {"name": name, "passed": state_errors_pass(errors), "errors": errors}
            )
        observed = {
            render_hz: cases[f"matrix_p{physics_hz}_r{render_hz}"]["metrics"][
                "observed_render_rate_hz"
            ]
            for render_hz in RENDER_RATES_HZ
        }
        render_cap_observations.append(
            {
                "physics_rate_hz": physics_hz,
                "requested_render_rate_hz": list(RENDER_RATES_HZ),
                "observed_render_rate_hz": observed,
                "note": (
                    "Observed rates are evidence, not a mechanics pass condition; "
                    "the full Open World may be GPU-limited below a requested cap."
                ),
            }
        )

    hitch_name = "render_hitch_p120_r30"
    hitch_config = configure_case(template, hitch_name, 120, 30)
    hitch_config["unreal_probe"]["render_hitch"] = {
        "at_tick": 40,
        "duration_s": 0.25,
    }
    hitch = run_case(uvd, output, hitch_name, hitch_config)
    hitch_errors = state_errors(
        hitch["final_state"], cases["matrix_p120_r30"]["final_state"]
    )
    hitch_passed = (
        state_errors_pass(hitch_errors)
        and hitch["metrics"]["physics_steps_during_render_hitch"] > 0
    )

    delay_name = "delayed_command_p120_r60"
    delay_config = configure_case(template, delay_name, 120, 60)
    delay_config["controls"]["schedule"][1]["arrival_tick"] = 66
    delayed = run_case(uvd, output, delay_name, delay_config)
    delay_passed = (
        delayed["metrics"]["late_command_updates"] == 1
        and delayed["metrics"]["held_due_to_delay_intervals"] == 6
        and delayed["metrics"]["command_interval_records"]
        == delayed["metrics"]["completed_steps"]
    )

    mechanics = {}
    for kind in ("unit_force", "unit_torque"):
        name = f"{kind}_p120_r60"
        config = configure_case(template, name, 120, 60, kind=kind)
        report = run_case(uvd, output, name, config)
        mechanics[kind] = {
            "passed": report["metrics"]["mechanics_response_passed"],
            "response_error": report["probe"]["mechanics_response_error"],
            "body_configuration": report["probe"]["body_configuration"],
        }

    passed = (
        all(item["passed"] for item in comparisons)
        and hitch_passed
        and delay_passed
        and all(item["passed"] for item in mechanics.values())
    )
    aggregate = {
        "schema_version": 1,
        "passed": passed,
        "matrix": case_summaries,
        "render_invariance": comparisons,
        "render_cap_observations": render_cap_observations,
        "render_hitch": {
            "passed": hitch_passed,
            "errors_against_unhit_case": hitch_errors,
            "physics_steps_during_hitch": hitch["metrics"][
                "physics_steps_during_render_hitch"
            ],
        },
        "delayed_command_surrogate": {
            "passed": delay_passed,
            "late_command_updates": delayed["metrics"]["late_command_updates"],
            "held_intervals": delayed["metrics"]["held_due_to_delay_intervals"],
            "note": "Deterministic arrival-delay probe; live UDP semantics are U2.",
        },
        "mechanics": mechanics,
    }
    write_json(output / "unreal_u0_report.json", aggregate)
    print(json.dumps(aggregate, indent=2))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Run and aggregate the Unreal U1 model and trajectory comparisons."""

import argparse
import copy
import csv
import datetime
import json
import math
import pathlib
import subprocess


ROOT = pathlib.Path(__file__).resolve().parents[1]
PHYSICS_RATES_HZ = (60, 120, 240)
STATE_GROUPS = {
    "position_ned_m": ("pn_m", "pe_m", "pd_m"),
    "q_body_to_ned": ("qw", "qx", "qy", "qz"),
    "velocity_body_mps": ("u_mps", "v_mps", "w_mps"),
    "omega_body_radps": ("p_radps", "q_radps", "r_radps"),
}


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n")


def configure_case(template, name, physics_hz, frontend):
    value = copy.deepcopy(template)
    value["run_id"] = name
    value["frontend"] = frontend
    value["aircraft"] = {
        "path": str((ROOT / "examples" / "aircraft" / "aerosonde.json").resolve())
    }
    value["clock"]["fixed_dt_s"] = 1.0 / physics_hz
    value["clock"]["motion_solver"] = "chaos" if frontend == "unreal" else "rk4"
    value["stop"] = {"final_tick": physics_hz}
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
        {"apply_tick": physics_hz // 2, "values": {"elevator": -0.07}},
        {"apply_tick": 3 * physics_hz // 4, "values": {"elevator": -0.08}},
    ]
    if frontend == "unreal":
        value["unreal_probe"] = {"kind": "aircraft", "render_rate_hz": 60}
    else:
        value.pop("unreal_probe", None)
    return value


def run_command(command, label):
    print(f"running {label}", flush=True)
    process = subprocess.run(command, text=True, capture_output=True)
    if process.returncode != 0:
        raise RuntimeError(
            f"{label} failed (exit {process.returncode})\n"
            f"stdout:\n{process.stdout}\nstderr:\n{process.stderr}"
        )
    print(f"passed  {label}", flush=True)


def load_signals(path):
    with path.open(newline="") as source:
        return {
            int(row["tick"]): {name: float(value) for name, value in row.items()}
            for row in csv.DictReader(source)
        }


def compare_rows(left, right):
    result = {}
    combined_squared = 0.0
    combined_count = 0
    for group, columns in STATE_GROUPS.items():
        squared = 0.0
        maximum = 0.0
        count = 0
        for left_row, right_row in zip(left, right):
            for column in columns:
                error = left_row[column] - right_row[column]
                squared += error * error
                maximum = max(maximum, abs(error))
                count += 1
        result[group] = {"rms": math.sqrt(squared / count), "max_abs": maximum}
        combined_squared += squared
        combined_count += count
    result["combined_state_rms"] = math.sqrt(combined_squared / combined_count)
    return result


def compare_same_ticks(left, right):
    ticks = sorted(set(left) & set(right))
    if not ticks:
        raise RuntimeError("signal files have no common ticks")
    result = compare_rows([left[tick] for tick in ticks], [right[tick] for tick in ticks])
    result["common_ticks"] = len(ticks)
    return result


def compare_refinement(coarse, fine, ratio):
    ticks = [tick for tick in sorted(coarse) if tick * ratio in fine]
    result = compare_rows(
        [coarse[tick] for tick in ticks], [fine[tick * ratio] for tick in ticks]
    )
    result["common_times"] = len(ticks)
    return result


def compare_wrenches(uvd, aircraft, sample_path):
    lines = [line for line in sample_path.read_text().splitlines() if line]
    samples = [json.loads(line) for line in lines]
    process = subprocess.run(
        [str(uvd), "model-probe", str(aircraft)],
        input="\n".join(lines) + "\n",
        text=True,
        capture_output=True,
        check=True,
    )
    reference = [json.loads(line) for line in process.stdout.splitlines() if line]
    if len(reference) != len(samples):
        raise RuntimeError("model probe returned a different sample count")
    maximum_force_error = 0.0
    maximum_moment_error = 0.0
    for sample, expected in zip(samples, reference):
        actual = sample["unreal_total_wrench"]
        maximum_force_error = max(
            maximum_force_error,
            *(abs(a - b) for a, b in zip(actual["force_body_N"], expected["total_force_n"])),
        )
        maximum_moment_error = max(
            maximum_moment_error,
            *(abs(a - b) for a, b in zip(actual["moment_body_Nm"], expected["total_moment_nm"])),
        )
    return {
        "passed": maximum_force_error <= 1e-10 and maximum_moment_error <= 1e-10,
        "samples": len(samples),
        "maximum_force_error_N": maximum_force_error,
        "maximum_moment_error_Nm": maximum_moment_error,
    }


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
        output = ROOT / "runs" / f"unreal_u1_{stamp}"
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)

    template = json.loads((ROOT / "examples" / "runs" / "unreal_u0.json").read_text())
    aircraft = ROOT / "examples" / "aircraft" / "aerosonde.json"
    signals = {"unreal": {}, "headless": {}}
    wrench_results = []
    for physics_hz in PHYSICS_RATES_HZ:
        for frontend, command in (("headless", "simulate"), ("unreal", "unreal")):
            name = f"{frontend}_p{physics_hz}"
            config = configure_case(template, name, physics_hz, frontend)
            config_path = output / "configs" / f"{name}.json"
            bundle = output / "cases" / name
            write_json(config_path, config)
            run_command(
                [str(uvd), command, str(config_path), "--output", str(bundle)], name
            )
            signals[frontend][physics_hz] = load_signals(bundle / "signals.csv")
            if frontend == "unreal":
                wrench = compare_wrenches(
                    uvd, aircraft, bundle / "unreal_model_samples.jsonl"
                )
                wrench["physics_rate_hz"] = physics_hz
                wrench_results.append(wrench)

    cross_backend = []
    for physics_hz in PHYSICS_RATES_HZ:
        result = compare_same_ticks(
            signals["unreal"][physics_hz], signals["headless"][physics_hz]
        )
        result["physics_rate_hz"] = physics_hz
        cross_backend.append(result)

    refinement = {"unreal": [], "headless": []}
    for frontend in refinement:
        for coarse_hz, fine_hz in ((60, 120), (120, 240)):
            result = compare_refinement(
                signals[frontend][coarse_hz],
                signals[frontend][fine_hz],
                fine_hz // coarse_hz,
            )
            result["coarse_rate_hz"] = coarse_hz
            result["fine_rate_hz"] = fine_hz
            refinement[frontend].append(result)

    cross_rms = [item["combined_state_rms"] for item in cross_backend]
    convergence_passed = (
        cross_rms[2] < cross_rms[1] < cross_rms[0]
        and refinement["unreal"][1]["combined_state_rms"]
        < refinement["unreal"][0]["combined_state_rms"]
        and refinement["headless"][1]["combined_state_rms"]
        < refinement["headless"][0]["combined_state_rms"]
    )
    passed = all(item["passed"] for item in wrench_results) and convergence_passed
    report = {
        "schema_version": 1,
        "passed": passed,
        "matched_state_wrench": wrench_results,
        "cross_backend_trajectory": cross_backend,
        "timestep_refinement": refinement,
        "convergence_passed": convergence_passed,
    }
    write_json(output / "unreal_u1_report.json", report)
    print(json.dumps(report, indent=2))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

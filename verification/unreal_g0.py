#!/usr/bin/env python3
"""Verify Cesium placement and its isolation from the model wrench."""

import argparse
import datetime
import json
import math
import pathlib
import subprocess


ROOT = pathlib.Path(__file__).resolve().parents[1]


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n")


def load_json(path):
    return json.loads(path.read_text())


def first_model_sample(bundle):
    with (bundle / "unreal_model_samples.jsonl").open() as source:
        return json.loads(next(source))


def vector_error(left, right):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uvd", default=str(ROOT / "build" / "uvd"))
    parser.add_argument("--run", default=str(ROOT / "examples/runs/unreal_g0.json"))
    parser.add_argument("--output")
    args = parser.parse_args()

    uvd = pathlib.Path(args.uvd).resolve()
    source_path = pathlib.Path(args.run).resolve()
    if args.output:
        output = pathlib.Path(args.output).resolve()
    else:
        stamp = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
        output = ROOT / "runs" / f"unreal_g0_{stamp}"
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)

    geospatial_config = load_json(source_path)
    geospatial_config["aircraft"]["path"] = str(
        (source_path.parent / geospatial_config["aircraft"]["path"]).resolve()
    )
    reference_config = json.loads(json.dumps(geospatial_config))
    reference_config["run_id"] = "unreal_g0_reference"
    reference_config["world"].pop("cesium")
    geospatial_path = output / "geospatial_config.json"
    reference_path = output / "reference_config.json"
    write_json(geospatial_path, geospatial_config)
    write_json(reference_path, reference_config)

    geospatial_bundle = output / "geospatial"
    reference_bundle = output / "reference"
    subprocess.run(
        [str(uvd), "unreal", str(reference_path), "--output", str(reference_bundle)],
        check=True,
    )
    subprocess.run(
        [
            str(uvd),
            "unreal",
            str(geospatial_path),
            "--output",
            str(geospatial_bundle),
        ],
        check=True,
    )

    reference_manifest = load_json(reference_bundle / "manifest.json")
    geospatial_manifest = load_json(geospatial_bundle / "manifest.json")
    geospatial = geospatial_manifest["geospatial"]
    reference_sample = first_model_sample(reference_bundle)
    geospatial_sample = first_model_sample(geospatial_bundle)
    reference_wrench = reference_sample["unreal_total_wrench"]
    geospatial_wrench = geospatial_sample["unreal_total_wrench"]
    force_error = vector_error(
        reference_wrench["force_body_N"], geospatial_wrench["force_body_N"]
    )
    moment_error = vector_error(
        reference_wrench["moment_body_Nm"],
        geospatial_wrench["moment_body_Nm"],
    )
    state_position_error = vector_error(
        reference_sample["state"]["position_ned_m"],
        geospatial_sample["state"]["position_ned_m"],
    )
    checks = {
        "reference_complete": reference_manifest["status"] == "complete",
        "geospatial_complete": geospatial_manifest["status"] == "complete",
        "cesium_enabled": geospatial["enabled"],
        "cesium_probe_passed": geospatial["passed"],
        "cesium_version_recorded": bool(
            geospatial_manifest.get("cesium_for_unreal_version")
        ),
        "ellipsoid_height": abs(
            geospatial["origin_longitude_latitude_height"][2] - 1370.0
        )
        <= 1e-9,
        "origin": geospatial["origin_error_m"] <= 1e-6,
        "round_trip": geospatial["round_trip_error_m"] <= 1e-6,
        "north_east_down_axes": geospatial["axis_error_cm"] <= 1e-2,
        "spawn_position": geospatial["spawn_error_cm"] <= 1e-2,
        "starting_heading": geospatial["heading_error_deg"] <= 1e-6,
        "matched_initial_position": state_position_error <= 1e-3,
        "unchanged_force": force_error <= 1e-4,
        "unchanged_moment": moment_error <= 1e-4,
    }
    report = {
        "schema_version": 1,
        "passed": all(checks.values()),
        "checks": checks,
        "geospatial": geospatial,
        "matched_state_position_error_m": state_position_error,
        "wrench_difference": {
            "force_norm_N": force_error,
            "moment_norm_Nm": moment_error,
        },
        "reference_bundle": str(reference_bundle),
        "geospatial_bundle": str(geospatial_bundle),
    }
    write_json(output / "unreal_g0_report.json", report)
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Run or inspect the readiness-gated FBWA case and replay its commands."""

import argparse
import csv
import datetime
import json
import math
import pathlib
import subprocess


ROOT = pathlib.Path(__file__).resolve().parents[1]
STATE_COLUMNS = {
    "position_ned_m": ("pn_m", "pe_m", "pd_m"),
    "velocity_body_mps": ("u_mps", "v_mps", "w_mps"),
    "omega_body_radps": ("p_radps", "q_radps", "r_radps"),
}
COMMAND_COLUMNS = ("cmd_aileron", "cmd_elevator", "cmd_rudder", "cmd_throttle")


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n")


def load_csv(path):
    with path.open(newline="") as source:
        return list(csv.DictReader(source))


def vector(row, names):
    return tuple(float(row[name]) for name in names)


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def euler_degrees(row):
    w, x, y, z = vector(row, ("qw", "qx", "qy", "qz"))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


def analyze_flight(manifest, session, rows):
    metrics = manifest["metrics"]
    release_tick = int(metrics["controller_release_tick"])
    final_window = rows[-1200:]
    release_row = rows[release_tick - 1]
    final_row = rows[-1]
    final_attitude = [euler_degrees(row) for row in final_window]
    final_rates = [
        abs(float(row[name]))
        for row in final_window
        for name in ("p_radps", "q_radps", "r_radps")
    ]
    final_surfaces = [
        abs(float(row[name]))
        for row in final_window
        for name in ("cmd_aileron", "cmd_elevator", "cmd_rudder")
    ]
    final_airspeed = [float(row["eas_mps"]) for row in final_window]
    post_release_altitude = [-float(row["pd_m"]) for row in rows[release_tick:]]
    checks = {
        "manifest_complete": manifest["status"] == "complete",
        "session_passed": session["passed"],
        "fbwa": session["mode"] == "FBWA",
        "armed": session["armed"],
        "ekf_healthy": session["ekf_healthy"],
        "parameters_verified": session["parameters_verified"],
        "release_recorded": metrics["controller_released"] and release_tick >= 0,
        "post_release_duration": len(rows) - release_tick >= 2400,
        "one_frame_per_step": metrics["accepted_controller_frames"]
        == metrics["completed_steps"]
        == len(rows),
        "no_transport_faults": metrics["controller_transport_passed"],
        "airborne": min(post_release_altitude) >= 250.0,
        "altitude_excursion": abs(
            -float(final_row["pd_m"]) - -float(release_row["pd_m"])
        )
        <= 120.0,
        "final_airspeed": min(final_airspeed) >= 15.0
        and max(final_airspeed) <= 35.0,
        "final_attitude": max(abs(roll) for roll, _ in final_attitude) <= 15.0
        and max(abs(pitch) for _, pitch in final_attitude) <= 15.0,
        "final_body_rates": max(final_rates) <= 0.5,
        "final_controls_not_saturated": max(final_surfaces) < 0.9,
        "flight_throttle": min(float(row["cmd_throttle"]) for row in final_window)
        >= 0.5,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "release_tick": release_tick,
        "post_release_duration_s": (len(rows) - release_tick)
        * float(manifest["fixed_dt_s"]),
        "release_altitude_above_origin_m": -float(release_row["pd_m"]),
        "final_altitude_above_origin_m": -float(final_row["pd_m"]),
        "final_equivalent_airspeed_mps": float(final_row["eas_mps"]),
        "final_roll_pitch_deg": euler_degrees(final_row),
        "maximum_final_window_body_rate_radps": max(final_rates),
    }


def replay_config(bundle, rows):
    value = json.loads((bundle / "resolved_config.json").read_text())
    value["frontend"] = "headless"
    value["run_id"] = "unreal_u3_command_replay"
    value["clock"]["motion_solver"] = "rk4"
    value["aircraft"]["path"] = str((bundle / "inputs" / "aircraft.json").resolve())
    value.pop("controller", None)
    value["world"].pop("cesium", None)
    value["stop"] = {"final_tick": len(rows)}
    schedule = []
    previous = None
    for row in rows:
        command = tuple(float(row[name]) for name in COMMAND_COLUMNS)
        if command == previous:
            continue
        schedule.append(
            {
                "apply_tick": int(row["tick"]) - 1,
                "values": {
                    "aileron": command[0],
                    "elevator": command[1],
                    "rudder": command[2],
                    "throttle": command[3],
                },
            }
        )
        previous = command
    value["controls"]["schedule"] = schedule
    return value


def compare_replay(unreal_rows, replay_rows):
    if len(unreal_rows) != len(replay_rows):
        raise RuntimeError("Unreal and replay signal lengths differ")
    squared = {name: 0.0 for name in STATE_COLUMNS}
    maximum = {name: 0.0 for name in STATE_COLUMNS}
    attitude_squared = 0.0
    attitude_maximum = 0.0
    maximum_command_error = 0.0
    for unreal, replay in zip(unreal_rows, replay_rows):
        maximum_command_error = max(
            maximum_command_error,
            *(abs(float(unreal[name]) - float(replay[name])) for name in COMMAND_COLUMNS),
        )
        for group, columns in STATE_COLUMNS.items():
            error = norm(
                tuple(a - b for a, b in zip(vector(unreal, columns), vector(replay, columns)))
            )
            squared[group] += error * error
            maximum[group] = max(maximum[group], error)
        left = vector(unreal, ("qw", "qx", "qy", "qz"))
        right = vector(replay, ("qw", "qx", "qy", "qz"))
        dot = abs(sum(a * b for a, b in zip(left, right)))
        angle = 2.0 * math.acos(max(-1.0, min(1.0, dot)))
        attitude_squared += angle * angle
        attitude_maximum = max(attitude_maximum, angle)
    count = len(unreal_rows)
    result = {
        name: {"rms": math.sqrt(squared[name] / count), "max": maximum[name]}
        for name in STATE_COLUMNS
    }
    result["attitude_rad"] = {
        "rms": math.sqrt(attitude_squared / count),
        "max": attitude_maximum,
    }
    result["maximum_command_error"] = maximum_command_error
    result["passed"] = (
        result["position_ned_m"]["rms"] <= 25.0
        and result["velocity_body_mps"]["rms"] <= 5.0
        and result["omega_body_radps"]["rms"] <= 1.0
        and result["attitude_rad"]["rms"] <= math.radians(20.0)
        and maximum_command_error == 0.0
    )
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uvd", default=str(ROOT / "build" / "uvd"))
    parser.add_argument("--run", default=str(ROOT / "examples/runs/unreal_ardupilot.json"))
    parser.add_argument("--bundle")
    parser.add_argument("--output")
    args = parser.parse_args()

    uvd = pathlib.Path(args.uvd).resolve()
    if args.output:
        output = pathlib.Path(args.output).resolve()
    else:
        stamp = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
        output = ROOT / "runs" / f"unreal_u3_{stamp}"
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    if args.bundle:
        bundle = pathlib.Path(args.bundle).resolve()
    else:
        bundle = output / "flight"
        subprocess.run(
            [str(uvd), "sitl", str(pathlib.Path(args.run).resolve()), "--output", str(bundle)],
            check=True,
        )

    manifest = json.loads((bundle / "manifest.json").read_text())
    session = json.loads((bundle / "controller" / "session.json").read_text())
    unreal_rows = load_csv(bundle / "signals.csv")
    flight = analyze_flight(manifest, session, unreal_rows)
    dataflash_logs = list((bundle / "controller" / "logs").glob("*.BIN"))
    flight["checks"]["dataflash_log"] = any(
        path.stat().st_size > 0 for path in dataflash_logs
    )
    flight["checks"]["packet_trace"] = (
        bundle / "controller" / "frames.csv"
    ).stat().st_size > 0
    flight["passed"] = all(flight["checks"].values())
    flight["dataflash_logs"] = [str(path) for path in dataflash_logs]

    config = replay_config(bundle, unreal_rows)
    config_path = output / "replay_config.json"
    replay_bundle = output / "replay"
    write_json(config_path, config)
    subprocess.run(
        [str(uvd), "simulate", str(config_path), "--output", str(replay_bundle)],
        check=True,
    )
    replay = compare_replay(unreal_rows, load_csv(replay_bundle / "signals.csv"))
    report = {
        "schema_version": 1,
        "passed": flight["passed"] and replay["passed"],
        "flight": flight,
        "cross_backend_command_replay": replay,
        "bundle": str(bundle),
    }
    write_json(output / "unreal_u3_report.json", report)
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

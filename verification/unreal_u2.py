#!/usr/bin/env python3
"""Exercise Unreal's live controller transport success and failure paths."""

import argparse
import copy
import datetime
import json
import os
import pathlib
import signal
import socket
import struct
import subprocess
import time


ROOT = pathlib.Path(__file__).resolve().parents[1]
RATE_HZ = 120
FINAL_TICK = 8
PWM = (1500,) * 16


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n")


def configure_case(template, name, port):
    value = copy.deepcopy(template)
    value["run_id"] = name
    value["aircraft"] = {
        "path": str((ROOT / "examples" / "aircraft" / "aerosonde.json").resolve())
    }
    value["stop"] = {"final_tick": FINAL_TICK}
    value["controller"]["udp_port"] = port
    value["controller"]["startup_timeout_s"] = 60.0
    value["controller"]["packet_timeout_s"] = 0.5
    value["controller"]["warmup_s"] = FINAL_TICK / RATE_HZ
    return value


def packet(frame_count):
    return struct.pack("<HHI16H", 18458, RATE_HZ, frame_count, *PWM)


def receive_reply(sock, deadline, minimum_timestamp=None):
    while time.monotonic() < deadline:
        sock.settimeout(min(0.15, max(0.001, deadline - time.monotonic())))
        try:
            payload, _ = sock.recvfrom(4096)
        except (ConnectionRefusedError, socket.timeout):
            return None
        try:
            reply = json.loads(payload.decode("utf-8").strip())
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        if minimum_timestamp is None or reply.get("timestamp", -1.0) >= minimum_timestamp:
            return reply
    return None


def exchange(sock, target, frame_count, startup=False):
    timeout = 55.0 if startup else 4.0
    deadline = time.monotonic() + timeout
    expected_timestamp = (frame_count + 1) / RATE_HZ
    while time.monotonic() < deadline:
        sock.sendto(packet(frame_count), target)
        reply = receive_reply(sock, min(deadline, time.monotonic() + 0.15), expected_timestamp)
        if reply is not None:
            return reply
    raise RuntimeError(f"no state reply for controller frame {frame_count}")


def wait_for_exit(process, timeout=30.0):
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            stdout, stderr = process.communicate(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            stdout, stderr = process.communicate()
        raise RuntimeError(
            f"Unreal probe timed out\nstdout:\n{stdout}\nstderr:\n{stderr}"
        )
    return process.returncode, stdout, stderr


def launch(uvd, config_path, bundle):
    return subprocess.Popen(
        [
            str(uvd),
            "unreal",
            str(config_path),
            "--transport-probe",
            "--output",
            str(bundle),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )


def load_result(bundle):
    report_path = bundle / "results" / "unreal_smoke.json"
    manifest_path = bundle / "manifest.json"
    if not report_path.exists() or not manifest_path.exists():
        raise RuntimeError(f"Unreal did not write a complete report in {bundle}")
    return json.loads(report_path.read_text()), json.loads(manifest_path.read_text())


def run_case(uvd, template, output, name, port):
    print(f"running {name}", flush=True)
    config = configure_case(template, name, port)
    config_path = output / "configs" / f"{name}.json"
    bundle = output / "cases" / name
    write_json(config_path, config)
    process = launch(uvd, config_path, bundle)
    target = ("127.0.0.1", port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    try:
        if name == "duplicate":
            first = exchange(sock, target, 0, startup=True)
            duplicate = exchange(sock, target, 0)
            if first != duplicate:
                raise RuntimeError("duplicate frame did not receive the cached state reply")
            for frame_count in range(1, FINAL_TICK):
                exchange(sock, target, frame_count)
        elif name == "gap":
            exchange(sock, target, 0, startup=True)
            sock.sendto(packet(2), target)
        elif name == "stale":
            exchange(sock, target, 0, startup=True)
            exchange(sock, target, 1)
            sock.sendto(packet(0), target)
        elif name == "malformed":
            deadline = time.monotonic() + 55.0
            while process.poll() is None and time.monotonic() < deadline:
                sock.sendto(b"not-a-controller-packet", target)
                time.sleep(0.15)
        elif name == "timeout":
            exchange(sock, target, 0, startup=True)
        else:
            raise RuntimeError(f"unknown probe case: {name}")
        return_code, stdout, stderr = wait_for_exit(process)
    finally:
        sock.close()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10.0)

    report, manifest = load_result(bundle)
    expected_reason = {
        "duplicate": "completed",
        "gap": "controller_frame_gap",
        "stale": "controller_stale_frame",
        "malformed": "malformed_controller_packet",
        "timeout": "controller_packet_timeout",
    }[name]
    metrics = report["metrics"]
    checks = {
        "expected_stop_reason": report["stop_reason"] == expected_reason,
        "manifest_stop_reason": manifest["stop_reason"] == expected_reason,
        "expected_process_status": return_code == (0 if name == "duplicate" else 2),
        "expected_report_status": report["passed"] == (name == "duplicate"),
    }
    if name == "duplicate":
        checks.update(
            {
                "all_frames_accepted": metrics["accepted_controller_frames"]
                == FINAL_TICK,
                "duplicate_observed": metrics["duplicate_controller_frames"] >= 1,
                "transport_passed": metrics["controller_transport_passed"],
            }
        )
    elif name == "gap":
        checks["gap_observed"] = metrics["controller_frame_gaps"] == 1
    elif name == "stale":
        checks["stale_observed"] = metrics["stale_controller_frames"] == 1
    elif name == "malformed":
        checks["malformed_observed"] = metrics["malformed_controller_frames"] == 1
    elif name == "timeout":
        checks["first_frame_accepted"] = metrics["accepted_controller_frames"] == 1

    result = {
        "name": name,
        "passed": all(checks.values()),
        "checks": checks,
        "stop_reason": report["stop_reason"],
        "metrics": {
            "completed_steps": metrics["completed_steps"],
            "accepted_controller_frames": metrics["accepted_controller_frames"],
            "duplicate_controller_frames": metrics["duplicate_controller_frames"],
            "malformed_controller_frames": metrics["malformed_controller_frames"],
            "controller_frame_gaps": metrics["controller_frame_gaps"],
            "stale_controller_frames": metrics["stale_controller_frames"],
        },
    }
    if not result["passed"]:
        result["launcher_stdout"] = stdout
        result["launcher_stderr"] = stderr
    print(f"{'passed' if result['passed'] else 'failed'}  {name}", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uvd", default=str(ROOT / "build" / "uvd"))
    parser.add_argument("--output")
    parser.add_argument("--base-port", type=int, default=9202)
    args = parser.parse_args()

    uvd = pathlib.Path(args.uvd).resolve()
    if not uvd.exists():
        raise SystemExit(f"uvd executable does not exist: {uvd}")
    if args.output:
        output = pathlib.Path(args.output).resolve()
    else:
        stamp = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
        output = ROOT / "runs" / f"unreal_u2_{stamp}"
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)

    template = json.loads((ROOT / "examples" / "runs" / "unreal_u2.json").read_text())
    names = ("duplicate", "gap", "stale", "malformed", "timeout")
    results = [
        run_case(uvd, template, output, name, args.base_port + index)
        for index, name in enumerate(names)
    ]
    report = {
        "schema_version": 1,
        "passed": all(result["passed"] for result in results),
        "cases": results,
    }
    write_json(output / "unreal_u2_report.json", report)
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Probe the real pinned ArduPlane JSON transport without Unreal or vehicle motion."""

import argparse
import json
import os
import pathlib
import platform
import socket
import struct
import subprocess
import sys
import time


ROOT = pathlib.Path(__file__).resolve().parents[1]
IMAGE = "uvd-ardupilot:e0652af-u3"
MAGIC_BY_SIZE = {40: (18458, 16), 72: (29569, 32)}


def decode(packet):
    expected = MAGIC_BY_SIZE.get(len(packet))
    if expected is None:
        raise RuntimeError(f"unexpected PWM packet size {len(packet)}")
    magic, channels = expected
    header = struct.unpack_from("<HHI", packet)
    if header[0] != magic:
        raise RuntimeError(f"unexpected PWM magic {header[0]}")
    pwm = struct.unpack_from(f"<{channels}H", packet, 8)
    return {"rate_hz": header[1], "frame_count": header[2], "pwm": pwm}


def sensor_record(timestamp):
    record = {
        "timestamp": timestamp,
        "imu": {"gyro": [0.0, 0.0, 0.0], "accel_body": [0.0, 0.0, -9.80665]},
        "position": [0.0, 0.0, 0.0],
        "quaternion": [1.0, 0.0, 0.0, 0.0],
        "velocity": [25.0, 0.0, 0.0],
        "airspeed": 25.0,
        "no_time_sync": False,
        "no_lockstep": False,
    }
    return ("\n" + json.dumps(record, separators=(",", ":")) + "\n").encode()


def stop_container(name):
    subprocess.run(
        ["docker", "stop", "--time", "1", name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=50)
    parser.add_argument("--report", type=pathlib.Path)
    args = parser.parse_args()
    if args.frames < 2:
        raise RuntimeError("--frames must be at least 2")

    name = f"uvd-transport-probe-{os.getpid()}"
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp.bind(("0.0.0.0", 9002))
    udp.settimeout(30.0)
    docker_arguments = ["docker", "run", "--rm", "--name", name]
    if platform.system() == "Linux":
        docker_arguments.extend(
            ["--add-host", "host.docker.internal:host-gateway"]
        )
    docker_arguments.extend(
        [
            "--tmpfs",
            "/root",
            "-e",
            "UVD_HOST_ADDRESS=host.docker.internal",
            "-e",
            "UVD_HOME=40.2338,-111.6585,1387.0,0.0",
            IMAGE,
        ]
    )
    controller = subprocess.Popen(
        docker_arguments,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    frames = []
    started = time.monotonic()
    probe_error = None
    try:
        try:
            for index in range(args.frames):
                packet, endpoint = udp.recvfrom(256)
                frame = decode(packet)
                delta = (
                    frame["frame_count"] - frames[-1]["frame_count"]
                ) & 0xFFFFFFFF if frames else 1
                if delta != 1:
                    raise RuntimeError("ArduPlane frame count was not consecutive")
                frames.append(frame)
                udp.sendto(sensor_record((index + 1) / 120.0), endpoint)
        except (socket.timeout, RuntimeError) as error:
            probe_error = error
    finally:
        stop_container(name)
        try:
            output, _ = controller.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            controller.kill()
            output, _ = controller.communicate()
        udp.close()

    if probe_error:
        raise RuntimeError(f"{probe_error}\nArduPlane output:\n{output}")

    elapsed = time.monotonic() - started
    settled_rates = [frame["rate_hz"] for frame in frames[5:]]
    if not settled_rates or any(rate != 120 for rate in settled_rates):
        raise RuntimeError(
            "ArduPlane JSON rate did not settle to 120 Hz: "
            + repr([frame["rate_hz"] for frame in frames])
        )
    report = {
        "schema_version": 1,
        "passed": True,
        "image": IMAGE,
        "frames": len(frames),
        "discovery_frame_rate_hz": frames[0]["rate_hz"],
        "settled_frame_rate_hz": settled_rates[-1],
        "first_frame_count": frames[0]["frame_count"],
        "last_frame_count": frames[-1]["frame_count"],
        "wall_time_s": elapsed,
        "controller_started_json_backend": "JSON control interface set to"
        in output,
    }
    if not report["controller_started_json_backend"]:
        raise RuntimeError("ArduPlane output did not confirm the JSON backend")
    rendered = json.dumps(report, indent=2) + "\n"
    print(rendered, end="")
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(rendered)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"transport probe: {error}", file=sys.stderr)
        sys.exit(2)

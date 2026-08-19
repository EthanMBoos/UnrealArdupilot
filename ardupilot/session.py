#!/usr/bin/env python3
"""Supervise one ArduPlane process and release a ready closed-loop session."""

import json
import os
import pathlib
import signal
import socket
import subprocess
import sys
import time

from pymavlink import mavutil


EXPECTED_PARAMETERS = {
    "SERVO1_FUNCTION": 4.0,
    "SERVO1_TRIM": 1512.0,
    "SERVO2_FUNCTION": 19.0,
    "SERVO2_TRIM": 1828.0,
    "SERVO3_FUNCTION": 70.0,
    "SERVO4_FUNCTION": 21.0,
    "SERVO4_TRIM": 1499.0,
}
REQUIRED_EKF_FLAGS = 1 | 2 | 4 | 16 | 32
stop_requested = False


def request_stop(_signum, _frame):
    global stop_requested
    stop_requested = True


def write_report(path, report):
    destination = pathlib.Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(".tmp")
    temporary.write_text(json.dumps(report, indent=2) + "\n")
    temporary.replace(destination)


def parameter_name(message):
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="replace")
    return value.rstrip("\x00")


def send_rc_override(connection, armed):
    ignore = 65535
    connection.mav.rc_channels_override_send(
        connection.target_system,
        connection.target_component,
        1500,
        1600 if armed else 1500,
        1775 if armed else 1000,
        1500,
        ignore,
        ignore,
        ignore,
        ignore,
    )


def send_release(host, port, command):
    endpoint = (socket.gethostbyname(host), port)
    control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for _ in range(5):
            control.sendto((command + "\n").encode("ascii"), endpoint)
            time.sleep(0.05)
    finally:
        control.close()


def main():
    host = os.environ["UVD_HOST_ADDRESS"]
    sim_address = os.environ["UVD_SIM_ADDRESS"]
    home = os.environ["UVD_HOME"]
    rate_hz = int(os.environ.get("UVD_RATE_HZ", "120"))
    control_port = int(os.environ.get("UVD_CONTROL_PORT", "9003"))
    readiness_timeout_s = float(os.environ.get("UVD_READINESS_TIMEOUT_S", "45"))
    evidence_path = os.environ.get("UVD_EVIDENCE_PATH", "/evidence/session.json")
    evidence_directory = str(pathlib.Path(evidence_path).parent)

    connection = mavutil.mavlink_connection(
        "udpin:0.0.0.0:14550", source_system=255, autoreconnect=False
    )
    command = [
        "/opt/ardupilot/arduplane",
        "--model",
        f"JSON:{sim_address}",
        "--home",
        home,
        "--speedup",
        "1",
        "--rate",
        str(rate_hz),
        "--serial0",
        "udpclient:127.0.0.1:14550",
        "--defaults",
        "/opt/ardupilot/uvd.parm",
    ]
    controller = subprocess.Popen(command, cwd=evidence_directory)
    started = time.monotonic()
    last_stream_request = 0.0
    last_parameter_request = 0.0
    last_mode_request = 0.0
    last_arm_request = 0.0
    last_override = 0.0
    last_report = 0.0
    mode = "UNKNOWN"
    armed = False
    gps_fix = 0
    ekf_flags = 0
    parameters = {}
    status_text = []
    command_acks = []
    released = False
    failed = False
    failure_reason = ""

    def report():
        return {
            "schema_version": 1,
            "passed": released and not failed,
            "released": released,
            "failure_reason": failure_reason,
            "arming_policy": "force_after_explicit_readiness",
            "target_system": connection.target_system,
            "target_component": connection.target_component,
            "mode": mode,
            "armed": armed,
            "gps_fix_type": gps_fix,
            "ekf_flags": ekf_flags,
            "required_ekf_flags": REQUIRED_EKF_FLAGS,
            "ekf_healthy": (ekf_flags & REQUIRED_EKF_FLAGS) == REQUIRED_EKF_FLAGS,
            "parameters": parameters,
            "parameters_verified": all(
                parameters.get(name) == expected
                for name, expected in EXPECTED_PARAMETERS.items()
            ),
            "status_text": status_text[-40:],
            "command_acks": command_acks[-20:],
            "wall_time_s": time.monotonic() - started,
        }

    try:
        while not stop_requested and controller.poll() is None:
            now = time.monotonic()
            message = connection.recv_match(blocking=True, timeout=0.1)
            if message is not None:
                message_type = message.get_type()
                if message_type == "HEARTBEAT" and message.type != mavutil.mavlink.MAV_TYPE_GCS:
                    connection.target_system = message.get_srcSystem()
                    connection.target_component = message.get_srcComponent()
                    mode = mavutil.mode_string_v10(message)
                    armed = bool(
                        message.base_mode
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                elif message_type == "GPS_RAW_INT":
                    gps_fix = int(message.fix_type)
                elif message_type == "EKF_STATUS_REPORT":
                    ekf_flags = int(message.flags)
                elif message_type == "PARAM_VALUE":
                    name = parameter_name(message)
                    if name in EXPECTED_PARAMETERS:
                        parameters[name] = float(message.param_value)
                elif message_type == "STATUSTEXT":
                    text = message.text
                    if isinstance(text, bytes):
                        text = text.decode("utf-8", errors="replace")
                    status_text.append(text.rstrip("\x00"))
                elif message_type == "COMMAND_ACK":
                    command_acks.append(
                        {"command": int(message.command), "result": int(message.result)}
                    )

            if connection.target_system:
                if now - last_stream_request >= 2.0:
                    connection.mav.request_data_stream_send(
                        connection.target_system,
                        connection.target_component,
                        mavutil.mavlink.MAV_DATA_STREAM_ALL,
                        10,
                        1,
                    )
                    last_stream_request = now
                if now - last_parameter_request >= 1.0:
                    for name in EXPECTED_PARAMETERS:
                        if name not in parameters:
                            connection.mav.param_request_read_send(
                                connection.target_system,
                                connection.target_component,
                                name.encode("ascii"),
                                -1,
                            )
                    last_parameter_request = now
                if now - last_override >= 0.4:
                    send_rc_override(connection, armed)
                    last_override = now
                if mode != "FBWA" and now - last_mode_request >= 1.0:
                    mode_id = connection.mode_mapping().get("FBWA")
                    if mode_id is not None:
                        connection.mav.set_mode_send(
                            connection.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            mode_id,
                        )
                    last_mode_request = now

                parameters_verified = all(
                    parameters.get(name) == expected
                    for name, expected in EXPECTED_PARAMETERS.items()
                )
                ekf_healthy = (
                    ekf_flags & REQUIRED_EKF_FLAGS
                ) == REQUIRED_EKF_FLAGS
                ready_to_arm = (
                    mode == "FBWA"
                    and gps_fix >= 3
                    and ekf_healthy
                    and parameters_verified
                )
                if ready_to_arm and not armed and now - last_arm_request >= 1.0:
                    connection.mav.command_long_send(
                        connection.target_system,
                        connection.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0,
                        1.0,
                        21196.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                    )
                    last_arm_request = now
                if ready_to_arm and armed and not released:
                    send_release(host, control_port, "release")
                    released = True
                    write_report(evidence_path, report())

            if not released and now - started >= readiness_timeout_s:
                failed = True
                failure_reason = "readiness_timeout"
                send_release(host, control_port, "fail")
                write_report(evidence_path, report())
                while not stop_requested and controller.poll() is None:
                    time.sleep(0.2)
                break
            if now - last_report >= 1.0:
                write_report(evidence_path, report())
                last_report = now
    finally:
        if controller.poll() is None:
            controller.terminate()
            try:
                controller.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                controller.kill()
                controller.wait()
        if not released and not failed:
            failed = True
            failure_reason = "controller_exit"
        write_report(evidence_path, report())

    return 0 if released and not failed else 2


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    try:
        sys.exit(main())
    except Exception as error:
        print(f"session supervisor: {error}", file=sys.stderr, flush=True)
        try:
            send_release(
                os.environ.get("UVD_HOST_ADDRESS", "host.docker.internal"),
                int(os.environ.get("UVD_CONTROL_PORT", "9003")),
                "fail",
            )
        except OSError:
            pass
        sys.exit(2)

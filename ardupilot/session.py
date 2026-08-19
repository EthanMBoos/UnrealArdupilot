#!/usr/bin/env python3
"""Start ArduPlane, arm FBWA after its estimators are ready, and release Unreal."""

import os
import signal
import socket
import subprocess
import sys
import time

from pymavlink import mavutil


running = True


def stop(_signal, _frame):
    global running
    running = False


def release_unreal(host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as control:
        for _ in range(5):
            control.sendto(b"release\n", (socket.gethostbyname(host), port))
            time.sleep(0.05)


def rc_override(connection, armed):
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


def main():
    host = os.environ["UVD_HOST_ADDRESS"]
    sim_address = os.environ["UVD_SIM_ADDRESS"]
    connection = mavutil.mavlink_connection(
        "udpin:0.0.0.0:14550", source_system=255, autoreconnect=False
    )
    plane = subprocess.Popen([
        "/opt/ardupilot/arduplane",
        "--model", f"JSON:{sim_address}",
        "--home", os.environ["UVD_HOME"],
        "--speedup", "1",
        "--rate", os.environ.get("UVD_RATE_HZ", "120"),
        "--serial0", "udpclient:127.0.0.1:14550",
        "--defaults", "/opt/ardupilot/uvd.parm",
    ])

    mode = ""
    armed = False
    gps_fix = 0
    ekf_flags = 0
    released = False
    last_request = 0.0
    required_ekf = 1 | 2 | 4 | 16 | 32
    try:
        while running and plane.poll() is None:
            message = connection.recv_match(blocking=True, timeout=0.1)
            if message:
                kind = message.get_type()
                if kind == "HEARTBEAT" and message.type != mavutil.mavlink.MAV_TYPE_GCS:
                    connection.target_system = message.get_srcSystem()
                    connection.target_component = message.get_srcComponent()
                    mode = mavutil.mode_string_v10(message)
                    armed = bool(
                        message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                elif kind == "GPS_RAW_INT":
                    gps_fix = int(message.fix_type)
                elif kind == "EKF_STATUS_REPORT":
                    ekf_flags = int(message.flags)

            now = time.monotonic()
            if connection.target_system and now - last_request >= 0.5:
                connection.mav.request_data_stream_send(
                    connection.target_system,
                    connection.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    10,
                    1,
                )
                rc_override(connection, armed)
                if mode != "FBWA":
                    mode_id = connection.mode_mapping().get("FBWA")
                    if mode_id is not None:
                        connection.mav.set_mode_send(
                            connection.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            mode_id,
                        )
                elif gps_fix >= 3 and (ekf_flags & required_ekf) == required_ekf:
                    if not armed:
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
                    elif not released:
                        release_unreal(
                            host, int(os.environ.get("UVD_CONTROL_PORT", "9003"))
                        )
                        print("ArduPlane is armed in FBWA; releasing vehicle dynamics", flush=True)
                        released = True
                last_request = now
    finally:
        if plane.poll() is None:
            plane.terminate()
            try:
                plane.wait(timeout=5)
            except subprocess.TimeoutExpired:
                plane.kill()
                plane.wait()
    return plane.returncode or 0


if __name__ == "__main__":
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    sys.exit(main())

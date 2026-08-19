#!/usr/bin/env python3
"""Build and run the one Unreal + Cesium + ArduPlane v1."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parent
PROJECT = ROOT / "unreal" / "UnrealVehicleDynamics.uproject"
CONFIG = ROOT / "examples" / "run.json"
IMAGE = "uvd-ardupilot:v1"


def unreal_editor():
    configured = os.environ.get("UVD_UNREAL_EDITOR")
    candidates = [
        Path(configured) if configured else None,
        Path("/Users/Shared/Epic Games/UE_5.8/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor"),
        Path("/Users/Shared/Epic Games/UE_5.8/Engine/Binaries/Mac/UnrealEditor"),
        Path("/opt/UnrealEngine/Engine/Binaries/Linux/UnrealEditor"),
    ]
    for candidate in candidates:
        if candidate and candidate.is_file():
            return candidate.resolve()
    raise RuntimeError("Unreal Editor 5.8 was not found; set UVD_UNREAL_EDITOR")


def engine_root(editor):
    for parent in editor.parents:
        if (parent / "Build" / "Build.version").is_file():
            return parent
    raise RuntimeError("could not find the Unreal Engine root")


def build(editor):
    subprocess.run(["cmake", "-S", str(ROOT), "-B", str(ROOT / "build")], check=True)
    subprocess.run(["cmake", "--build", str(ROOT / "build")], check=True)
    engine = engine_root(editor)
    if sys.platform == "darwin":
        script = engine / "Build" / "BatchFiles" / "Mac" / "Build.sh"
        platform = "Mac"
    else:
        script = engine / "Build" / "BatchFiles" / "Linux" / "Build.sh"
        platform = "Linux"
    subprocess.run([
        str(script), "UnrealVehicleDynamicsEditor", platform, "Development",
        f"-Project={PROJECT}", "-WaitMutex", "-NoHotReloadFromIDE",
    ], check=True)
    inspected = subprocess.run(
        ["docker", "image", "inspect", IMAGE],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if inspected.returncode:
        subprocess.run([
            "docker", "build", "-t", IMAGE, "-f",
            str(ROOT / "ardupilot" / "Dockerfile"), str(ROOT),
        ], check=True)


def main():
    editor = unreal_editor()
    build(editor)
    config = json.loads(CONFIG.read_text())
    origin = config["origin"]
    home = ",".join(str(value) for value in [
        origin["latitude_deg"], origin["longitude_deg"],
        origin["altitude_msl_m"], origin["heading_deg"],
    ])
    rate = round(1.0 / config["fixed_dt_s"])
    container_name = f"uvd-ardupilot-{os.getpid()}"
    unreal = subprocess.Popen([
        str(editor), str(PROJECT), "-game", "-log", "-novsync",
        f"-UvdRun={CONFIG}", f"-UvdFixedDt={config['fixed_dt_s']}",
    ])
    docker_command = ["docker", "run", "--rm", "--name", container_name]
    if sys.platform.startswith("linux"):
        docker_command += ["--add-host", "host.docker.internal:host-gateway"]
    docker_command += [
        "-e", "UVD_HOST_ADDRESS=host.docker.internal",
        "-e", f"UVD_HOME={home}",
        "-e", f"UVD_RATE_HZ={rate}",
        "-e", f"UVD_CONTROL_PORT={config['controller']['control_port']}",
        IMAGE,
    ]
    controller = subprocess.Popen(docker_command)
    stopping = False

    def stop(_signal=None, _frame=None):
        nonlocal stopping
        if _signal is not None:
            stopping = True
        if unreal.poll() is None:
            unreal.terminate()
        subprocess.run(
            ["docker", "stop", "--timeout", "3", container_name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while unreal.poll() is None and controller.poll() is None:
            time.sleep(0.5)
    finally:
        stop()
        unreal.wait()
        controller.wait()
    if stopping:
        return 0
    return unreal.returncode or controller.returncode


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"uvd: {error}", file=sys.stderr)
        sys.exit(1)

#!/bin/sh
set -eu

: "${UVD_HOST_ADDRESS:?Set UVD_HOST_ADDRESS to the Unreal host address}"
: "${UVD_HOME:?Set UVD_HOME to lat,lon,alt,heading}"
: "${UVD_RATE_HZ:=120}"

# ArduPilot's simulator socket expects a numeric address even though the
# MAVLink serial transport accepts hostnames.
set -- $(getent ahostsv4 "${UVD_HOST_ADDRESS}")
UVD_SIM_ADDRESS="$1"
: "${UVD_SIM_ADDRESS:?Could not resolve UVD_HOST_ADDRESS}"

if [ "${UVD_SESSION_MODE:-transport}" = "closed_loop" ]; then
  export UVD_SIM_ADDRESS
  exec /opt/ardupilot-venv/bin/python /opt/ardupilot/session.py
fi

exec /opt/ardupilot/arduplane \
  --model "JSON:${UVD_SIM_ADDRESS}" \
  --home "${UVD_HOME}" \
  --speedup 1 \
  --rate "${UVD_RATE_HZ}" \
  --serial0 "udpclient:${UVD_HOST_ADDRESS}:14550" \
  --defaults /opt/ardupilot/uvd.parm \
  "$@"

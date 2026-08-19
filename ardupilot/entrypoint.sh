#!/bin/sh
set -eu

: "${UVD_HOST_ADDRESS:?Set UVD_HOST_ADDRESS}"
: "${UVD_HOME:?Set UVD_HOME}"

set -- $(getent ahostsv4 "${UVD_HOST_ADDRESS}")
export UVD_SIM_ADDRESS="$1"
exec /opt/ardupilot-venv/bin/python /opt/ardupilot/session.py

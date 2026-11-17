#!/usr/bin/env bash
# RawBot Studio hardware doctor: USB adapter -> permission -> port holders -> LeRobot motor scan
# (read-only) -> training service -> robot runtime / ROS joint states. Names the first link
# that fails and how to fix it. Nothing moves.
#
#   ./scripts/hw-doctor.sh [robot_type] [--role follower|leader] [--port /dev/ttyACM0]
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${PHYSIOS_CONFIG:-$HERE/configuration/physios.env}"
[[ -f "$CONFIG" ]] && { set -a; # shellcheck disable=SC1090
  source "$CONFIG"; set +a; }
exec "${PHYSIOS_PYTHON:-python3}" "$HERE/scripts/hw_doctor.py" "$@"

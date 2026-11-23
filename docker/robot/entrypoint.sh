#!/usr/bin/env bash
# PHYSIOS robot runtime entry point.
#   run    (default) Zenoh router (unless one is already listening) + robot server bringup:
#          physical_ai_server, rosbridge (:9090), bag recorder, web_video_server (:8080).
#          The bringup only serves ROS services and topics. It never moves the robot:
#          recording, inference and rollouts always start from an operator action.
#   shell  an interactive shell with ROS sourced
#   *      run the given command with ROS sourced
set -eo pipefail
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${COLCON_WS:-/root/ros2_ws}/install/setup.bash"

DATA="${PHYSIOS_CONTAINER_DATA:-/physios/data}"
mkdir -p "$DATA/huggingface/lerobot" "$DATA/outputs/train" "$DATA/robot-config" \
         "${PHYSICAL_AI_MANAGER_DATA:-/workspace/physios}"

# operator robot configurations (data/robot-config/*_config.yaml) next to the built-in ones
SHARE="$(ros2 pkg prefix physical_ai_server)/share/physical_ai_server/config"
shopt -s nullglob
# the container keeps its files across restarts: first remove the operator configs copied last
# time, so an edited, fixed or deleted file in data/robot-config takes effect
if [[ -f "$SHARE/.physios-operator-configs" ]]; then
  while read -r old; do [[ -n "$old" ]] && rm -f "$SHARE/$old"; done < "$SHARE/.physios-operator-configs"
fi
: > "$SHARE/.physios-operator-configs"
for f in "$DATA"/robot-config/*_config.yaml; do
  # every config is loaded as parameters of one node: a file ROS 2 cannot load (an empty list,
  # bad YAML) would stop the whole robot server, so such a file is skipped with the reason
  if why="$(python3 - "$f" <<'PY'
import sys, yaml
try:
    d = yaml.safe_load(open(sys.argv[1]))
    robots = d['physical_ai_server']['ros__parameters']
    bad = ['%s.%s' % (r, k) for r, ps in robots.items() for k, v in (ps or {}).items()
           if v is None or v == [] or (isinstance(v, dict) and any(x in (None, []) for x in v.values()))]
    print('empty value: ' + ', '.join(bad) if bad else '')
except Exception as e:  # noqa: BLE001
    print('cannot read it: %s' % e)
PY
)" && [[ -z "$why" ]]; then
    cp -f "$f" "$SHARE/" && basename "$f" >> "$SHARE/.physios-operator-configs" \
      && echo "[physios-robot] robot config: $(basename "$f")"
  else
    echo "[physios-robot] SKIPPED robot config $(basename "$f") ($why); fix it and restart"
  fi
done
shopt -u nullglob

zenoh_listening() { (exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null; }

case "${1:-run}" in
  run)
    if [[ "${PHYSIOS_ROBOT_START_ZENOH:-1}" == 1 ]] && ! zenoh_listening; then
      echo "[physios-robot] starting the Zenoh router"
      ros2 run rmw_zenoh_cpp rmw_zenohd &
      for _ in $(seq 1 20); do zenoh_listening && break; sleep 0.5; done
    fi
    echo "[physios-robot] starting the robot server bringup (no motion)"
    exec ros2 launch physical_ai_server physical_ai_server_bringup.launch.py ;;
  shell) exec bash ;;
  *) exec "$@" ;;
esac

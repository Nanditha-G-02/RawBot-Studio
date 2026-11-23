#!/bin/bash
# physios-xarm: start UFACTORY's arm driver (ros2_control) for RawBot Studio.
#   PHYSIOS_XARM_IP     the arm controller's IP address (required), e.g. 192.168.1.189
#   PHYSIOS_XARM_TYPE   lite6 (default) | xarm5 | xarm6 | xarm7 | uf850
#   PHYSIOS_XARM_GRIPPER  none (default) | gripper | vacuum
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WS}/install/setup.bash
if [[ "${1:-}" == shell ]]; then exec bash; fi
: "${PHYSIOS_XARM_IP:?set PHYSIOS_XARM_IP to the arm controller address, e.g. 192.168.1.189}"
TYPE="${PHYSIOS_XARM_TYPE:-lite6}"
case "$TYPE" in
  lite6) RT=lite; DOF=6 ;; xarm5) RT=xarm; DOF=5 ;; xarm6) RT=xarm; DOF=6 ;;
  xarm7) RT=xarm; DOF=7 ;; uf850) RT=uf850; DOF=6 ;;
  *) echo "[physios-xarm] unknown PHYSIOS_XARM_TYPE '$TYPE'" >&2; exit 2 ;;
esac
G=false; V=false
case "${PHYSIOS_XARM_GRIPPER:-none}" in gripper) G=true ;; vacuum) V=true ;; esac
# the robot runtime starts the Zenoh router; wait for it so the driver joins RawBot Studio's graph
for _ in $(seq 1 60); do (exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null && break; sleep 1; done
echo "[physios-xarm] $TYPE at $PHYSIOS_XARM_IP (gripper: ${PHYSIOS_XARM_GRIPPER:-none}); the controller holds the current pose"
exec ros2 launch ${WS}/rawbot_ufactory.launch.py robot_ip:="$PHYSIOS_XARM_IP" \
  robot_type:=$RT dof:=$DOF add_gripper:=$G add_vacuum_gripper:=$V

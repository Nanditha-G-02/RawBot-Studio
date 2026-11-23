# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0
#
# Optional UFACTORY arm driver (Lite6 / xArm / 850) for RawBot Studio: UFACTORY's own xarm_ros2
# (jazzy branch) in ros2_control mode, on the same ROS graph as the physios-robot container
# (ROS 2 Jazzy, rmw_zenoh, ROS_DOMAIN_ID 30). Nothing is installed on the host, so an existing
# ROS 2 (e.g. Humble) setup there is untouched.
#
# Topics (Lite6, hw_ns 'ufactory'): /ufactory/joint_states and /joint_states (state),
# /lite6_traj_controller/joint_trajectory (commands, trajectory_msgs/JointTrajectory).
# The container moves nothing by itself: the trajectory controller holds the current pose until
# RawBot Studio (supervised hardware test, jog, rollout) sends a command.

ARG BASE_IMAGE=ros:jazzy-ros-base
FROM ${BASE_IMAGE}

ARG XARM_ROS2_REPOSITORY=https://github.com/xArm-Developer/xarm_ros2.git
ARG XARM_ROS2_COMMIT=d837f2e17f7f36043103b367fca3af8172626245

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    ROS_DOMAIN_ID=30 \
    WS=/root/xarm_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
      git ros-dev-tools ros-${ROS_DISTRO}-rmw-zenoh-cpp \
      ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-launch-param-builder \
 && rm -rf /var/lib/apt/lists/*

# UFACTORY's driver at a pinned commit (with the C++ SDK submodule); only the packages the arm
# driver and ros2_control need (no MoveIt, Gazebo or RViz)
RUN mkdir -p ${WS}/src && cd ${WS}/src \
 && git init -q xarm_ros2 && cd xarm_ros2 \
 && git remote add origin "${XARM_ROS2_REPOSITORY}" \
 && git fetch -q --depth 1 origin "${XARM_ROS2_COMMIT}" && git checkout -q FETCH_HEAD \
 && git submodule update -q --init --depth 1 xarm_sdk/cxx \
 && rm -rf xarm_gazebo xarm_moveit_config xarm_moveit_servo xarm_planner demo thirdparty

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && cd ${WS} \
 && apt-get update \
 && (rosdep init 2>/dev/null || true) && rosdep update --rosdistro ${ROS_DISTRO} \
 && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} \
      --skip-keys "gazebo_ros gazebo_ros2_control gz_ros2_control ros_gz_sim" \
 && rm -rf /var/lib/apt/lists/* \
 && colcon build --packages-up-to xarm_controller uf_ros_lib --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && source install/setup.bash \
 && python3 -c "import uf_ros_lib.uf_robot_utils" \
 && ros2 pkg prefix joint_trajectory_controller >/dev/null && ros2 pkg prefix xarm_controller >/dev/null \
 && rm -rf build log

COPY docker/xarm/rawbot_ufactory.launch.py ${WS}/rawbot_ufactory.launch.py
COPY docker/xarm/entrypoint.sh /usr/local/bin/physios-xarm
RUN chmod +x /usr/local/bin/physios-xarm
ENTRYPOINT ["/usr/local/bin/physios-xarm"]

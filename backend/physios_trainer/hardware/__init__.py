# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""Flexible robot setup: which arms are present, how they are identified, and what can run.

* ``ports``        - USB serial discovery from sysfs and who holds each port (read-only)
* ``registry``     - device-role registry, per-robot setups, calibration profiles (on disk)
* ``capabilities`` - what the current hardware allows, with a reason for everything it does not
* ``manager``      - connection state, monitoring, recovery, calibration and jog sessions
* ``hw_worker``    - runs in the operator's LeRobot environment and talks to the motors
* ``jog_worker``   - supervised follower jog over ROS (same path as host rollout)
"""

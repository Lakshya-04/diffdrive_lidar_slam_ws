#!/bin/bash
set -e

# Source the base ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Source the local workspace overlay
if [ -f /home/rosuser/diff_drive_ws/install/setup.bash ]; then
  source /home/rosuser/diff_drive_ws/install/setup.bash
fi

# Execute the command passed to docker run
exec "$@"
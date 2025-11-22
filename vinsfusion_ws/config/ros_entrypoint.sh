#!/bin/bash
set -e

# Source ROS setup
source "/opt/ros/noetic/setup.bash"

# Source the catkin workspace if it exists
if [ -f "/root/catkin_ws/devel/setup.bash" ]; then
    source "/root/catkin_ws/devel/setup.bash"
fi

# Execute the command passed to the container
exec "$@"
#!/bin/bash
set -e

# Setup the ROS environment
source "/root/catkin_ws/devel/setup.bash"

# Execute the command passed to the container
exec "$@"

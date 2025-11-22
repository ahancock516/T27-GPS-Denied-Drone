#!/bin/bash
set -e

# Source the ROS setup script
source "/opt/ros/noetic/setup.bash"

# Execute the command passed to the container (e.g., "bash")
exec "$@"
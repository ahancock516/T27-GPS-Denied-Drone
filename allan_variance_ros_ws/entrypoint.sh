#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# Execute the command passed to the container
exec "$@"
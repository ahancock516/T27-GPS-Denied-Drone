# Use an arm64 Ubuntu 20.04 base image with ROS Noetic
FROM arm64v8/ros:noetic-ros-base

# 1. Install MAVROS package
# The 'ros-noetic-mavros-extras' package includes useful plugins
RUN apt-get update && apt-get install -y \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
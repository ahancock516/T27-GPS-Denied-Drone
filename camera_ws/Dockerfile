# Use the official ROS Noetic image for ARM64 architecture
FROM arm64v8/ros:noetic-ros-base

# Install system dependencies needed for the camera node
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    libraspberrypi-dev \
    libraspberrypi-bin \
    ros-noetic-camera-info-manager \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-compressed-image-transport \
    ros-noetic-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# Create a catkin workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Clone the raspicam_node source code
RUN git clone https://github.com/UbiquityRobotics/raspicam_node.git

# Build the workspace
WORKDIR /root/catkin_ws
# Source the ROS environment to use rosdep and catkin_make
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; \
     rosdep install --from-paths src --ignore-src -r -y; \
     catkin_make"

# Add the workspace's setup file to the .bashrc for automatic sourcing
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

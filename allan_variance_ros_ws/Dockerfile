FROM arm64v8/ros:noetic-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV CATKIN_WS=/root/catkin_ws

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    bc \
    ros-noetic-rosbag \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-tf \
    && rm -rf /var/lib/apt/lists/*
    
# Force install newer numpy (remove old one first if it exists)
RUN python3 -m pip uninstall -y numpy || true
RUN python3 -m pip install --no-cache-dir "numpy==1.23.5"

# Install other Python dependencies
RUN python3 -m pip install --no-cache-dir \
    "matplotlib>=3.5" \
    "scipy>=1.7" \
    pyyaml

# Initialize rosdep (if not already initialized)
RUN rosdep update || true

# Create catkin workspace
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}

# Clone allan_variance_ros repository
RUN cd ${CATKIN_WS}/src && \
    git clone https://github.com/ori-drs/allan_variance_ros.git

# Install ROS dependencies
RUN apt-get update && \
    cd ${CATKIN_WS} && \
    . /opt/ros/noetic/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN cd ${CATKIN_WS} && \
    rm -rf build devel .catkin_workspace && \
    . /opt/ros/noetic/setup.sh && \
    catkin build allan_variance_ros

# Create data directory
RUN mkdir -p /data

# Copy and set up entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
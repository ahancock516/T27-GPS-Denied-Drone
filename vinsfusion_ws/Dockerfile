FROM arm64v8/ros:noetic-ros-base AS ceres-builder

# Install only the build dependencies for Ceres Solver
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    liblapack-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone, configure, compile, and install Ceres Solver (version 2.1.0)
# Use -j1 to prevent OOM errors on Raspberry Pi
RUN git clone --recurse-submodules https://ceres-solver.googlesource.com/ceres-solver /root/ceres-solver \
    && cd /root/ceres-solver \
    && git checkout 2.1.0 \
    && mkdir build && cd build \
    && cmake .. -DEXECINFO_LIBRARY="" -DCXX11=ON -DCMAKE_BUILD_TYPE=Release \
    && make -j2 \
    && make install

# Use an arm64 Ubuntu 20.04 base image with ROS Noetic
FROM arm64v8/ros:noetic-ros-base AS builder_base

# Copy the pre-compiled Ceres libraries from the first stage
COPY --from=ceres-builder /usr/local /usr/local

# Run ldconfig to update the linker cache to find the new Ceres libraries
RUN ldconfig

# Install the remaining runtime and ROS dependencies for VINS-Fusion
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-pandas \
    python3-matplotlib \
    libeigen3-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    # Add Ceres's runtime dependencies
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    liblapack-dev \
    # Add additional dependencies
    libboost-all-dev \
    libmetis-dev \
    # Add the ROS dependencies
    ros-noetic-cv-bridge \
    ros-noetic-tf \
    ros-noetic-image-transport \
    ros-noetic-nav-msgs \
    ros-noetic-nodelet \
    ros-noetic-camera-calibration-parsers \
    ros-noetic-camera-info-manager \
    ros-noetic-visualization-msgs \
    ros-noetic-rviz \
    #added for mavros messages & haptic feeback
    ros-noetic-mavros-msgs \
    ros-noetic-mavros \
    espeak \
    && rm -rf /var/lib/apt/lists/*

FROM builder_base

# Build VINS-Fusion with single-threaded compilation to avoid OOM
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/cyp4x141/VINS-Fusion-noetic-Opencv4.git VINS-Fusion
WORKDIR /root/catkin_ws

# Use -j1 for single-threaded build to prevent memory issues on Pi
RUN . /opt/ros/noetic/setup.sh && catkin_make -j2 -DCMAKE_CXX_STANDARD=14

# RPi.GPIO is deprecated on pi 5.
# RUN pip3 install RPi.GPIO 
#RUN pip3 install --upgrade pip && pip3 uninstall -y RPi.GPIO || true && pip3 install --ignore-installed rpi-lgpio
RUN apt-get update && apt-get install -y \
    gpiod \
    libgpiod-dev \
    python3-libgpiod \
    && pip3 install gpiod \
    && rm -rf /var/lib/apt/lists/*

# Setup entrypoint
COPY config/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["rostopic", "list"]

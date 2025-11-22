FROM arm64v8/ros:noetic-ros-base AS opencv-builder

# 1. Install build tools and ALL of OpenCV's dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    libgtk2.0-dev \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libdc1394-22-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. Download a specific, stable version of OpenCV and OpenCV-Contrib
WORKDIR /root
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.5.zip && \
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.5.5.zip && \
    unzip opencv.zip && \
    unzip opencv_contrib.zip && \
    mv opencv-4.5.5 opencv && \
    mv opencv_contrib-4.5.5 opencv_contrib

# 3. Configure and build OpenCV (This is the long step)
WORKDIR /root/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=/root/opencv_contrib/modules \
    -D WITH_TBB=ON \
    -D WITH_V4L=ON \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    .. && \
    make -j4 && \
    make install

FROM arm64v8/ros:noetic-ros-base

# 1. Copy the new OpenCV version from the builder stage
COPY --from=opencv-builder /usr/local /usr/local
# Update the linker cache to find the new OpenCV libraries
RUN ldconfig

# 2. Install dependencies for Pangolin and ORB-SLAM3
RUN apt-get update && apt-get install -y \
    build-essential cmake git libeigen3-dev libglew-dev libgl1-mesa-dev xvfb libepoxy-dev \
    python3-pip \
    libgtk2.0-dev \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libdc1394-22-dev \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m pip install --no-cache-dir wheel

# 3. Build and Install Pangolin (locking to v0.6 for stability)
RUN xvfb-run --auto-servernum --server-args="-screen 0 1024x768x24" \
    /bin/sh -c "git clone --recursive https://github.com/stevenlovegrove/Pangolin.git /root/Pangolin && \
    cd /root/Pangolin && \
    git checkout v0.6 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j4 && \
    make install"

RUN git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git /root/ORB_SLAM3

RUN cd /root/ORB_SLAM3/Thirdparty/DBoW2 && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j2 && \
    cd ../../g2o && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j2 && \
    cd ../../.. && \
    sed -i '/^# Build examples/,$d' CMakeLists.txt && \
    mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j2

# 4c. Build the ORB-SLAM3 ROS Wrapper
# This step builds the final ROS node, which depends on the library from the step above
# 4c. Build the ORB-SLAM3 ROS Wrapper using Catkin
# This is the final step with the new fix
RUN mkdir -p /root/catkin_ws/src && \
    # Copy the correct catkin-compatible ROS wrapper into the workspace
    cp -r /root/ORB_SLAM3/Examples_old/ROS/ORB_SLAM3 /root/catkin_ws/src/ && \
    # IMPORTANT: Create the ignore file to hide the ancient rosbuild package
    touch /root/ORB_SLAM3/Examples_old/ROS/CATKIN_IGNORE && \
    # Build the workspace
    cd /root/catkin_ws && \
    . /opt/ros/noetic/setup.sh && catkin_make
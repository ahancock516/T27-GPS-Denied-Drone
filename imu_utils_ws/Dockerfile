# Use the official ARM64 ROS Noetic base image for Raspberry Pi 4/5
FROM arm64v8/ros:noetic-ros-base

# Set the shell to bash
SHELL ["/bin/bash", "-c"]

# Prevent apt-get from asking for user input during builds
ENV DEBIAN_FRONTEND=noninteractive

# Install build tools, git, and other dependencies for imu_utils
# The ros-base image is minimal and doesn't include a compiler by default.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake\
    git \
    libdw-dev \
    libeigen3-dev \
    libceres-dev \
    && rm -rf /var/lib/apt/lists/*

# Create and switch to the catkin workspace source directory
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

# Clone the imu_utils repository and its required dependency (code_utils)
RUN git clone https://github.com/gaowenliang/imu_utils.git
RUN git clone https://github.com/gaowenliang/code_utils.git

#Removing Broken test Utils in code_utils CMakeLists.txt
RUN sed -i '/<build_depend>roscpp<\/build_depend>/a <depend>code_utils<\/depend>' /catkin_ws/src/imu_utils/package.xml
RUN sed -i 's/add_executable(sumpixel_test/#add_executable(sumpixel_test/' /catkin_ws/src/code_utils/CMakeLists.txt
RUN sed -i 's/add_executable(matIO_test/#add_executable(matIO_test/' /catkin_ws/src/code_utils/CMakeLists.txt
RUN sed -i 's/target_link_libraries(sumpixel_test/#target_link_libraries(sumpixel_test/' /catkin_ws/src/code_utils/CMakeLists.txt
RUN sed -i 's/target_link_libraries(matIO_test/#target_link_libraries(matIO_test/' /catkin_ws/src/code_utils/CMakeLists.txt

# Switch to the workspace root
WORKDIR /catkin_ws

# Source the ROS environment and build the packages
RUN source /opt/ros/noetic/setup.bash && catkin_make

# Create an entrypoint script to source the workspace automatically
RUN echo '#!/bin/bash' >> /ros_entrypoint.sh && \
    echo 'set -e' >> /ros_entrypoint.sh && \
    echo 'source "/opt/ros/noetic/setup.bash"' >> /ros_entrypoint.sh && \
    echo 'source "/root/catkin_ws/devel/setup.bash"' >> /ros_entrypoint.sh && \
    echo 'exec "$@"' >> /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

# The default command when running the container is to start a bash shell
CMD ["bash"]
# Start from a base ROS image that has common libraries
FROM arm64v8/ros:noetic-ros-base

# Install essential ROS tools and GUI components
RUN apt-get update && apt-get install -y \
    ros-noetic-rqt \
    ros-noetic-rqt-image-view \
    ros-noetic-image-view \
    ros-noetic-rqt-plot \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Set up the entrypoint to source ROS automatically
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# The default command will be a bash shell
CMD ["bash"]

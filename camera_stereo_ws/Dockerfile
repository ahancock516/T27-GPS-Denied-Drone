# Use the official ARM64 ROS 1 Noetic base image
FROM arm64v8/ros:noetic-ros-base

# STEP 1: Install prerequisites for adding the PPA
RUN apt-get update && apt-get install -y \
    curl \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# STEP 2: Add the stable CTU-MRS PPA
RUN curl https://ctu-mrs.github.io/ppa-stable/add_ppa.sh | bash

# STEP 3: Update package lists and install the pre-compiled ROS driver
# This installs libcamera and the ROS wrapper in one step
RUN apt-get update && apt-get install -y \
    ros-noetic-libcamera-ros-driver \
    && rm -rf /var/lib/apt/lists/*

# STEP 4: Create final configs with the correct pixel format
RUN mkdir -p /etc/ros/config && \
    # Create the left.yaml config file
    echo 'camera_name: "left"' > /etc/ros/config/left.yaml && \
    echo 'camera_id: 0' >> /etc/ros/config/left.yaml && \
    echo 'stream_role: "video"' >> /etc/ros/config/left.yaml && \
    echo 'pixel_format: "BGR888"' >> /etc/ros/config/left.yaml && \
    echo 'frame_id: "left_camera_frame"' >> /etc/ros/config/left.yaml && \
    echo 'use_ros_time: true' >> /etc/ros/config/left.yaml && \
    echo 'resolution:' >> /etc/ros/config/left.yaml && \
    echo '  width: 640' >> /etc/ros/config/left.yaml && \
    echo '  height: 480' >> /etc/ros/config/left.yaml && \
    echo 'calib_url: "file:///root/calib/left.yaml"' >> /etc/ros/config/left.yaml && \
    # Create the right.yaml config file
    echo 'camera_name: "right"' > /etc/ros/config/right.yaml && \
    echo 'camera_id: 1' >> /etc/ros/config/right.yaml && \
    echo 'stream_role: "video"' >> /etc/ros/config/right.yaml && \
    echo 'pixel_format: "BGR888"' >> /etc/ros/config/right.yaml && \
    echo 'frame_id: "right_camera_frame"' >> /etc/ros/config/right.yaml && \
    echo 'use_ros_time: true' >> /etc/ros/config/right.yaml && \
    echo 'resolution:' >> /etc/ros/config/right.yaml && \
    echo '  width: 640' >> /etc/ros/config/right.yaml && \
    echo '  height: 480' >> /etc/ros/config/right.yaml && \
    echo 'calib_url: "file:///root/calib/right.yaml"' >> /etc/ros/config/right.yaml && \
    # Create the final, correct launch file
    echo '<launch>' > /etc/ros/stereo_bringup.launch && \
    echo '  ' >> /etc/ros/stereo_bringup.launch && \
    echo '  <node pkg="nodelet" type="nodelet" name="libcamera_nodelet_manager" args="manager" output="screen"/>' >> /etc/ros/stereo_bringup.launch && \
    echo '' >> /etc/ros/stereo_bringup.launch && \
    echo '  ' >> /etc/ros/stereo_bringup.launch && \
    echo '  <node pkg="nodelet" type="nodelet" name="left_camera" args="load libcamera_ros_driver/LibcameraRosDriver libcamera_nodelet_manager" output="screen">' >> /etc/ros/stereo_bringup.launch && \
    echo '      <rosparam file="/etc/ros/config/left.yaml" />' >> /etc/ros/stereo_bringup.launch && \
    echo '      <remap from="/camera" to="/left" />' >> /etc/ros/stereo_bringup.launch && \
    echo '  </node>' >> /etc/ros/stereo_bringup.launch && \
    echo '' >> /etc/ros/stereo_bringup.launch && \
    echo '  ' >> /etc/ros/stereo_bringup.launch && \
    echo '  <node pkg="nodelet" type="nodelet" name="right_camera" args="load libcamera_ros_driver/LibcameraRosDriver libcamera_nodelet_manager" output="screen">' >> /etc/ros/stereo_bringup.launch && \
    echo '      <rosparam file="/etc/ros/config/right.yaml" />' >> /etc/ros/stereo_bringup.launch && \
    echo '      <remap from="/camera" to="/right" />' >> /etc/ros/stereo_bringup.launch && \
    echo '  </node>' >> /etc/ros/stereo_bringup.launch && \
    echo '</launch>' >> /etc/ros/stereo_bringup.launch

# STEP 5: Use the default ROS entrypoint.
ENTRYPOINT ["/ros_entrypoint.sh"]
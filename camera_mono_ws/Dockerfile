# Use the official ARM64 ROS 1 Noetic base image
FROM arm64v8/ros:noetic-ros-base

# STEP 1: Install prerequisites
RUN apt-get update && apt-get install -y \
    curl \
    software-properties-common \
    nano \
 && rm -rf /var/lib/apt/lists/*

# STEP 2: Add the stable CTU-MRS PPA
RUN curl https://ctu-mrs.github.io/ppa-stable/add_ppa.sh | bash

# STEP 3: Install the pre-compiled ROS driver (no rotate package)
RUN apt-get update && apt-get install -y \
    ros-noetic-libcamera-ros-driver \
    ros-noetic-image-proc \
 && rm -rf /var/lib/apt/lists/*

# STEP 4: Create camera config and bringup launch
RUN mkdir -p /etc/ros/config && \
    cat > /etc/ros/config/camera.yaml <<'EOF'
camera_name: "mono_camera"
camera_id: 0
stream_role: "video"
pixel_format: "BGR888"
frame_id: "camera_frame"
use_ros_time: true
resolution:
  width: 640
  height: 480
calib_url: "file:///root/calib/camera.yaml"
EOF

RUN cat > /etc/ros/camera_bringup.launch <<'EOF'
<launch>
  <!-- Camera nodelet manager -->
  <node pkg="nodelet" type="nodelet" name="libcamera_nodelet_manager" args="manager" output="screen"/>

  <!-- Camera driver nodelet -->
  <node pkg="nodelet" type="nodelet" name="mono_camera"
        args="load libcamera_ros_driver/LibcameraRosDriver libcamera_nodelet_manager"
        output="screen">
    <rosparam file="/etc/ros/config/camera.yaml" />
  </node>
</launch>
EOF

# STEP 5: Use the default ROS entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

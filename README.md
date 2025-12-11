# GPS-Denied Drone Navigation: VINS-Fusion vs. ORB-SLAM3

## 📖 Project Overview
This repository documents the development, calibration, and evaluation of a custom UAV designed for GPS-denied environments. The project compares the performance of **VINS-Fusion** and **ORB-SLAM** using a visual-inertial sensor suite in both indoor and outdoor environments.

## 🚁 Hardware Setup
**The UAV Build:**
*   **Frame:** [7” Quadcopter]
*   **Flight Controller:** [Aero Selfie H743]
*   **Onboard Computer:** [Raspberry Pi 5 8Gb]
*   **Camera:** [Raspberry Pi Camera 3 Wide]
*   **Dual IMU:** [BMI088/BMI270]

## Development Workflow

The results in this project were achieved through the following four-phase process:
 
### Phase 1: Sensor Calibration (Kalibr)
Accurate state estimation requires precise calibration of the camera intrinsics and the camera-IMU extrinsics.

1.	**Tools Used:** 
  a.	[Kalibr](https://github.com/ethz-asl/kalibr)
  b.	[Allan Variance](https://github.com/ori-drs/allan_variance_ros)

2.	**Target:** [AprilGrid 6x6 0.8m](https://github.com/ethz-asl/kalibr/wiki/calibration-targets)

3.	**Calibration Process:**
  1)	Rosbag Capture
  Replace the camera topic with the name of your camera topic.
  '''rosrun image_view image_view image:=/mono_camera/image_raw'''
  '''rosbag record -O calibration_data.bag /mono_camera/image_raw /mavros/imu/data_raw'''
  With the resulting camera_calibration_data.bag, run the camera calibration with Kalibr.
  2)	Camera Calibration
    •	The AprilGrid must remain stationary
    •	Rotate and move the AprilGrid around the camera while keeping the AprilGrid within the frame of the camera.
  ''' rosrun kalibr kalibr_calibrate_cameras --target target.yaml --bag data.bag --models pinhole-radtan --topics /camera/image_raw '''
  3)	Camera + IMU Calibration
    • Recorded bag files exciting all IMU axes while keeping the target in view.
    • It is recommended to record a new bag file.
    • Rotate and move the camera while keeping the AprilGrid stationary.
    • Replace the imu topic with the name of your imu topic.
  '''rosrun image_view image_view image:=/mono_camera/image_raw'''
  '''rosbag record -O camera_imu_calibration_data.bag /mono_camera/image_raw /mavros/imu/data_raw'''
  '''rosrun kalibr kalibr_calibrate_imu_camera --target target.yaml --cam camchain.yaml --imu imu.yaml --bag data.bag --time-calibration '''
  5)  **Output:** See `config/calibration_results.yaml` for the resulting matrices used in the SLAM nodes.
  
  Generic Kalibr Run Commands:
  ''' Kalibr Run Commands:
  
  kalibr_calibrate_cameras \
      --bag [your_bag_file.bag] \
      --topics /cam0/image_raw /cam1/image_raw \
      --models pinhole-radtan pinhole-radtan \
      --target [your_target_file.yaml]
  
  kalibr_calibrate_imu_camera \
      --bag ~/ros_bags/imu_cam_calib.bag \
      --cam ~/kalibr_ws/camchain-cam_calib.yaml \
      --imu ~/kalibr_ws/my_imu.yaml \
      --target ~/kalibr_ws/src/kalibr/targets/april_6x6_A4.yaml
  '''

### Phase 2: Data Collection
We collected datasets in two distinct environments to test robustness.
*   **Indoor:** [e.g., Warehouse environment, low texture, controlled lighting]
*   **Outdoor:** [e.g., Open field, wind disturbance, dynamic lighting]
*   **Format:** All data was recorded as ROS bags containing `/mono_camera/image_raw` and `/mavros/imu/data_raw` topics.

### Phase 3: Algorithm Implementation

#### VINS-Fusion Configuration
*   Modified the `realsense_stereo_imu_config.yaml` to include our specific Kalibr results.
*   **Loop Closure:** Enabled/Disabled (Specify which).
*   **Command:**
    ```bash
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node /path/to/config/my_uav_config.yaml
    ```

#### ORB-SLAM Configuration
*   Generated a custom `.yaml` settings file based on the camera intrinsics.
*   **Mode:** Monocular-Inertial / Stereo-Inertial.
*   **Command:**
    ```bash
    rosrun ORB_SLAM3 Stereo_Inertial /path/to/vocab /path/to/settings.yaml
    ```

### Phase 4: Evaluation & Results
We compared the estimated trajectories against [Ground Truth source, e.g., OptiTrack or RTK GPS] or simply compared the two algorithms against each other (Loop Closure consistency).

#### 🏠 Indoor Results
*   **Observation:** [e.g., VINS-Fusion handled low-texture walls better than ORB-SLAM.]
*   *(Insert trajectory plot image here)*

#### 🌲 Outdoor Results
*   **Observation:** [e.g., ORB-SLAM maintained better lock during fast rotations.]
*   *(Insert trajectory plot image here)*

## 📈 Conclusion
[Brief summary of which algorithm worked best for your specific drone build].

## 📦 Dependencies
*   ROS [Noetic/Melodic]
*   Ceres Solver
*   OpenCV
*   Eigen

## 🤝 Acknowledgements
*   [HKUST VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
*   [UZH-RPG ORB-SLAM3](https://github.com/UZH-RPG/ORB_SLAM3)

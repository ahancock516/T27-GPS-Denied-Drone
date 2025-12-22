# 🚁 GPS-Denied Drone Navigation (Team 27)

**Autonomous navigation in signal-constrained environments using VINS-Fusion on a Raspberry Pi 5.**

![Status](https://img.shields.io/badge/Status-VIO_Functional-green)
![Platform](https://img.shields.io/badge/Platform-Raspberry_Pi_5-C51A4A)
![Algorithm](https://img.shields.io/badge/Algorithm-VINS--Fusion-blue)

## 📖 Project Overview
This project addresses the challenge of Unmanned Aerial Vehicle (UAV) navigation in GPS-denied environments. We developed a low-cost, 3D-printed UAV platform and implemented **VINS-Fusion** to provide alternative positioning when GPS signals are unavailable. 

While **ORB-SLAM3** was evaluated, our testing confirmed that **VINS-Fusion** (running at 200Hz IMU rate) provided superior stability and loop closure for our specific hardware configuration.

**Key Achievement:** Verified autonomous state estimation with an average deviation of **1.28m** over a 422m outdoor loop (Mills Pond Park Test).

## 🎥 Results

<img width="1000" height="600" alt="Screenshot 2025-12-02 235545" src="https://github.com/user-attachments/assets/91399a89-ad7b-4b09-92ed-ae9eae84a2cc" />
<img width="1000" height="600" alt="Screenshot 2025-12-07 192435" src="https://github.com/user-attachments/assets/410caec2-53f8-4ad8-b60f-1c44233a9717" />
<br>*Figure: VINS-Fusion estimated trajectory vs. Ground Truth GPS during outdoor testing.*
<img width="1000" height="600" alt="GPS Guess V Actual" src="https://github.com/user-attachments/assets/88c6d45f-8bf1-4edf-91a0-8c6339e1cdee" />

## ⚙️ Hardware Specs
The system is built on a custom quadcopter frame.

| Component | Model | Notes |
| :--- | :--- | :--- |
| **Companion Computer** | Raspberry Pi 5 (8GB) | Selected for Quad-Core 2.4GHz CPU |
| **Flight Controller** | Aero Selfie H743 | Running ArduPilot |
| **Camera** | Raspberry Pi Camera 3 Wide | Monocular setup |
| **Optical Flow** | Mico Air MTF-02P | For velocity estimation assistance |
| **GPS/Compass** | HGLRC M100 Pro | Used for Ground Truth comparison |
| **ESC** | 60A 4-in-1 | 2S-6S LiPo rated |
| **Telemetry** | ELRS Receiver | Radio Control & Telemetry Data |
| **Radio** | Radiomaster Pocket | Radio Control |

## ⚠️ Critical Lessons Learned (Read Before Building)
1.  **IMU Frequency is King:** VINS-Fusion **requires** a minimum 200Hz IMU update rate. Anything lower (50Hz) caused immediate drift and failure in our tests.
2.  **Magnetic Interference:** The compass is highly sensitive to the high-current wires from the 3-phase motors. **Solution:** You must mount the GPS/Compass module on a mast, at least 8cm away from the main battery and ESC wiring.
3.  **Initialization:** The system requires complete stillness to initialize the IMU bias, followed by slow, consistent movement to establish the VIO lock.
4.  **Compute Limits:** The Raspberry Pi 5 cannot run ORB-SLAM3 smoothly in real-time alongside ROS overhead. VINS-Fusion was much more efficient for this architecture.
5.  **Serial stream rates** Configuring the Aero Selfie H743 RAW IMU Stream Rate too high will overload the flight controller CPU and block out or interfere with critical parameters including GPS. We found 200Hz to be achievable and ideal.

## 3D Printed Parts & Files
<img width="1920" height="1080" alt="drone_render" src="https://github.com/user-attachments/assets/6316caf9-ed12-422c-a506-5a72a74fb92b" />
--Link to 3D Print Repository--

| Part Name | Qty | Description | Material |
| :--- | :--- | :--- | :--- |
| [`ai_drone_arm.stl`](3D_Print/ai_drone_arm.stl) | 4 | Motor arms | PLA |
| [`ai_drone_bottom_chassis.stl`](3D_Print/ai_drone_bottom_chassis.stl) | 1 | Chassis Bottom | PLA |
| [`ai_drone_camera_hinge_back.stl`](3D_Print/ai_drone_camera_hinge_back.stl) | 1 | Camera Base Mount | TPU |
| [`ai_drone_camera_hinge_front.stl`](3D_Print/ai_drone_camera_hinge_front.stl) | 1 | Camera Mount | TPU |
| [`ai_drone_top_chassis.stl`](3D_Print/ai_drone_top_chassis.stl) | 1 | Chassis Top | PLA |
| [`ai_drone_under_plate.stl`](3D_Print/ai_drone_under_plate.stl) | 1 | Chassis Under Plate | PLA |
*note all are printed at 30% gyroid infill.

## System Payload
![Drone Payload](https://github.com/user-attachments/assets/41458662-f246-4f83-a422-26ed67716800)

## System Wiring Diagram
![4-H743_60AESCStackConnectivityDiagram](https://github.com/user-attachments/assets/34ae5887-13ba-4162-abba-5c8f6e4e9b3c)
*https://aeroselfie.myshopify.com/products/aero-selfie-h743-flight-controller-stack-30x30-stack-with-60a-4in1-esc*

> ⚠️ **Important:** Ensure the Motors are mapped to the correct ESC Channels.
https://ardupilot.org/copter/docs/initial-setup.html

## System Control
<img width="576" height="385" alt="image (16)" src="https://github.com/user-attachments/assets/cf254d8a-e666-407e-a83c-c9cb223d8ad5" />
<img width="560" height="407" alt="image (15)" src="https://github.com/user-attachments/assets/bbe49cf9-fa1f-43c1-b4b1-3b1bbae7bcb2" />
https://cdn.shopify.com/s/files/1/0609/8324/7079/files/Pocket_1.pdf

## 🛠️ Installation & Setup

<details>
<summary><b>🔻Installing Debian GNU/Linux 12 (Bookworm) - (15 min)
</b></summary>

## 🍓 Tutorial: Installing "Legacy" Bookworm OS on Raspberry Pi 5

This guide covers how to install the **Legacy** version of Raspberry Pi OS (Bookworm). 

> **Why use Legacy?** 
> The standard RPi OS uses **Wayland**, which breaks compatibility with some remote desktop tools (like RealVNC), screen recording software, and Python GUI automation libraries (like `pyautogui`). The **Legacy** version maintains the classic **X11** window system while keeping the core OS up to date.

### Prerequisites
*   **Hardware:** Raspberry Pi 5
*   **Storage:** 128GB (or larger) microSD card
*   **Software:** [Raspberry Pi Imager](https://www.raspberrypi.com/software/) installed on your PC/Mac

### Installation Steps

1.  **Launch Raspberry Pi Imager** on your computer.
2.  **Select Device:**
    *   Click `CHOOSE DEVICE` → Select **Raspberry Pi 5**.
3.  **Select OS:**
    *   Click `CHOOSE OS`.
    *   Select **Raspberry Pi OS (other)**.
    *   Select **Raspberry Pi OS (Legacy, 64-bit)**.
    *   *⚠️ Verify the description says "A port of Debian Bookworm" (ensure you do not select the older Bullseye version).*
4.  **Select Storage:**
    *   Click `CHOOSE STORAGE` → Select your **128GB SD Card**.
5.  **Configure Settings (Recommended):**
    *   Click `NEXT`.
    *   When prompted to apply OS customisation, click **EDIT SETTINGS**.
    *   **General:** Set your `hostname`, `username`, `password`, and `Wi-Fi` credentials.
    *   **Services:** Enable **SSH** (password authentication) if you need headless access.
6.  **Write:**
    *   Click `SAVE` → `YES` → `YES`.
    *   *Wait for verification to complete.*

---
*Verified for Raspberry Pi 5 as of 2025.*
</details>

<details>
<summary><b>🔻Installing Docker on Raspberry Pi 5 - (3 min)</b></summary> 

### Prerequisites

1. Update your system:
```bash
sudo apt update && sudo apt upgrade -y
```

2. Install Docker using the official convenience script:
```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

3. Add your user to the docker group (to run Docker without sudo):
```bash
sudo usermod -aG docker $USER
```

4. Log out and back in for the group changes to take effect, then verify the installation:
```bash
docker --version
docker run hello-world
```

</details>

<details>
<summary><b>🔻Installing the Repository - (1 Hr)</b></summary>

1. Clone the repository:
```bash
git clone https://github.com/ahancock516/T27-GPS-Denied-Drone.git
cd T27-GPS-Denied-Drone
```

2. Build and run with Docker Compose:
```bash
docker compose up --build
```

For Launching Specific Containers
```bash
make vinsfusion #camera_mono, mavros, roscore, etc..
```

3. To stop the containers:
```bash
docker compose down
```
or
```bash
make clean
```
>⚠️ The Raspberry Pi 5 has limited resources, so it may be necessary to 'make build service-name' individually for each service.

</details>

<details>
<summary><b>🔻Sensor Calibration - (>3Hr)</b></summary>

## Camera Calibration
Accurate state estimation requires precise calibration of the camera intrinsics and the camera-IMU extrinsics.

## Calibration Workflow
> ⚠️ **Important:** Kalibr cannot be installed on Raspberry Pi 5 and must be run on a separate computer (Linux or Mac recommended).

Because the Raspberry Pi 5 lacks the compute power to run the Kalibr optimization suite, the calibration process is split into two phases: **Data Collection** (on the Drone) and **Data Processing** (on a Linux Workstation).

### 🎥 Phase A: Data Collection
*Complete these steps to generate the raw data needed for calibration.*

**Tools Used:**

a. [Kalibr](https://github.com/ethz-asl/kalibr)<br>
b. [Allan Variance ROS](https://github.com/ori-drs/allan_variance_ros)<br>
c. [AprilGrid 6x6 0.8m](https://github.com/ethz-asl/kalibr/wiki/downloads)<br>

## Prerequisites
- Flight Controller with IMU properly connected
- Camera publishing to ROS topic
- AprilGrid calibration target
- External drive for data storage
- Kalibr installed on a laptop/desktop

## Step 1: Allan Variance Collection (IMU Calibration)

### 1.1 Verify IMU Data Stream
Before recording, confirm your IMU meets the following requirements:
- IMU publishing rate: **minimum 200Hz**
- Topic `/mavros/imu/data_raw` is publishing valid data

> Refer to setting up the publishing rate for Mavlink 
> Refer to setting up the publishing rate for Mavlink Parameters in the [ArduPilot Configuration](#ardupilot-mavlink-configuration) section.

You can verify this with:
```bash
rostopic hz /mavros/imu/data_raw
rostopic echo /mavros/imu/data_raw
```

### 1.2 Record Static IMU Data
**⚠️ CRITICAL: The Flight Controller must remain completely stationary for the entire 3-hour recording period.**

Use the `ros_tools` container (or any container with ROS environment and rosbag functionality):
```bash
rosbag record -O allan_variance_data.bag --duration=10800 /mavros/imu/data_raw
```

This will record for 3 hours (10,800 seconds). Once complete, copy the bag file to an external drive for processing on your laptop.

### 1.3 Generate Allan Variance Parameters
On your laptop with Kalibr installed, process the bag file to extract Allan Variance parameters:
```bash
rosrun kalibr kalibr_allan --bag allan_variance_data.bag --axis-yz
```

**Output:** Allan variance plots and parameters needed for `imu.yaml`

### 1.4 Create imu.yaml Configuration
Create `imu.yaml` using the Allan Variance parameters obtained above:
```yaml
#Accelerometers
accelerometer_noise_density: 1.86e-03   # [m/s^2/sqrt(Hz)]
accelerometer_random_walk: 4.33e-04     # [m/s^3/sqrt(Hz)]

#Gyroscopes
gyroscope_noise_density: 1.87e-04       # [rad/s/sqrt(Hz)]
gyroscope_random_walk: 2.66e-05         # [rad/s^2/sqrt(Hz)]

rostopic: /mavros/imu/data_raw
update_rate: 200.0                       # Hz
```
---

### 📂 Phase B: Processing & Config Generation (Workstation)
>Compute Requirement: The following steps require high-performance optimization and GUI-based tools (Kalibr) that are not compatible with the Raspberry Pi. Move your recorded .bag files to a Linux/Ubuntu laptop or desktop to proceed.

1. Transfer Data
Copy your imu_static.bag and calibration_data.bag from the Raspberry Pi to your workstation via SFTP or a USB drive.

2. Enter the Calibration Environment
We have created a dedicated repository containing the Dockerized Kalibr suite and conversion scripts to transform raw calibration data into VINS-Fusion/ORB-SLAM3 configuration files.

➜['Open the kalibr_slam_converter Repository'](https://github.com/ahancock516/kalibr_slam_converter)

3. Execution Summary (Inside the Toolkit)
Inside the linked repository, you will perform the following:
Allan Variance Analysis: Run the scripts to find accelerometer_noise_density and gyroscope_random_walk.
Camera Intrinsics: Solve for the lens distortion and focal length.
Camera-IMU Extrinsics: Determine the exact spatial transformation (Rotation/Translation) between the Pi Camera and the Flight Controller's IMU.
YAML Export: Automatically generate the vins_config.yaml and camera.yaml files.

### Example Configuration Files

Reference these example configurations based on your calibration results:

**VINS-Fusion:**
- [`vinsfusion_ws/config/camera_config.yaml`](vinsfusion_ws/config/camera_config.yaml) - Camera parameters for VINS_Fusion
- [`vinsfusion_ws/config/vins_mono_imu_config.yaml`](vinsfusion_ws/config/vins_mono_imu_config.yaml) - Complete VINS-Fusion configuration

**ORB-SLAM3:**
- [`orbslam3_ws/config/pi_camera_inertial.yaml`](orbslam3_ws/config/pi_camera_inertial.yaml) - Camera-IMU configuration for ORB-SLAM3

> 💡 **Note:** These are example files. You must update them with your own calibration results from Kalibr.

</details>

<details>
<summary><b>🔻ArduPilot & MAVLink Configuration - (10 min)</b></summary>
<a name="ardupilot-mavlink-configuration"></a>

## Configuring the Flight Controller to Raspberry Pi 5 Data Bridge

We have provided a Vehicle_Params.params file that capture all of the settings used during our Mills-Pond-Park autonomous flight tests

1. Open QGroundControl
2. Navigate to Vehicle Configuration
3. Parameters->Tools
4. Load the Params file

#### ⚙️ Optimized VIO Data Stream
To achieve high-fidelity state estimation, the following parameters were tuned to provide a high-frequency, low-noise data stream from the H743 to the Raspberry Pi 5.

| Parameter | Value | Description |
| :--- | :--- | :--- |
| **`SCHED_LOOP_RATE`** | `400` | Increases FC internal processing frequency to 400Hz to minimize sensor jitter. |
| **`SERIAL2_BAUD`** | `921` | Sets **Telem 2** to 921600 Baud for high-bandwidth MAVLink 2 communication. |
| **`SR2_RAW_SENS`** | `200` | **200Hz Raw IMU stream** (Accel/Gyro) required for VINS-Fusion stability. |
| **`SR2_EXTRA2`** | `200` | Matches high-rate Attitude data stream to the IMU frequency. |
| **`INS_GYRO_FILTER`** | `20` | Low-pass filter at 20Hz to remove motor vibrations from the data stream. |
| **`INS_ACCEL_FILTER`**| `20` | Low-pass filter at 20Hz for cleaner visual-inertial integration. |
| **`INS_FAST_SAMPLE`** | `1` | Enables the highest internal sampling rate of the IMU chips. |


> **Data Rate Verification:** After writing these parameters, run `rostopic hz /mavros/imu/data_raw` on the Raspberry Pi. If the rate is below 200Hz, check for serial cable interference or CPU bottlenecking on the FC.
</details>

<details>
<summary><b>🔻Optional Systemctl Service </b></summary>

⚙️ Automation: Systemd Service
To ensure the drone is "ready to fly" as soon as it is powered on, use the following systemd configuration to launch the Docker stack automatically.

1. Create the service file:
```bash
sudo nano /etc/systemd/system/vins_startup.service
```

2. Paste the following configuration:
```Ini
[Unit]
Description=T27 VINS-Fusion Docker Autostart
After=docker.service network-online.target
Requires=docker.service

[Service]
Restart=always
User=pi
WorkingDirectory=/home/pi/T27-GPS-Denied-Drone
ExecStart=/usr/bin/make vinsfusion
ExecStop=/usr/bin/docker compose down

[Install]
WantedBy=multi-user.target
```
3. Enable and start the service:
```bash
sudo systemctl daemon-reload
sudo systemctl enable vins_startup.service
sudo systemctl start vins_startup.service
```
>Reference the Radiomaster Radio Diagram

The VINS-Fusion container is currently configured to launch a startup python script that listens for a radio input through the /mavros/rc/out channel data. When the upper-right (SD) momentary bumper of the Radiomaster Pocket is depressed, the script sends a pwm output to GPIO 24 (pin 18) of the raspberry pi 5. This output pin is used to drive the base of an NPN PN2222A transistor which drives an active buzzer. 

####  Active Buzzer Wiring
![alt text](<Active Buzzer Diagram.png>)

#### 🤖 Logic & Behavior
The VINS-Fusion container runs a background monitor that listens to the `/mavros/rc/out` MAVLink channel. The following audio patterns indicate the current state of the navigation stack:

| Event | Sound Pattern | Meaning / Status |
| :--- | :--- | :--- |
| **System Boot** | 🎵 3 Short Beeps | Docker container is live and the monitor script is active. |
| **Button Pressed** | 🎵 1 Quick Blip | Bumper signal detected from the Radiomaster Pocket. |
| **VIO Lock** | 🎵 2 Medium Beeps | **VINS STABLE:** Algorithm initialized; VIO is generating state estimation. |
| **Button De-Pressed** | Short Pause + 🎵 1 Quick Blip | Algorithm Finished & Exited Gracefully |

> When the **VIO Lock** is achieved, the system also pushes a MAVLink statustext message. If you are connected via telemetry to **QGroundControl**, you will hear the audible voice notification: *"VINS STABLE"*.

Happy Testing!

</details>

# 📚 Citations & References

## Algorithms & Software

**VINS-Fusion**  
Qin, T., Li, P., & Shen, S. (2018). *VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator.* IEEE Transactions on Robotics. [GitHub Repository](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

**ORB-SLAM3**  
Campos, C., Elvira, R., Rodríguez, J. J. G., Montiel, J. M., & Tardós, J. D. (2021). *ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual–Inertial, and Multi-Map SLAM.* IEEE Transactions on Robotics. [GitHub Repository](https://github.com/UZ-SLAMLab/ORB_SLAM3)

**Kalibr**  
Furgale, P., Rehder, J., & Siegwart, R. (2013). *Unified Temporal and Spatial Calibration for Multi-Sensor Systems.* IEEE/RSJ International Conference on Intelligent Robots and Systems. [GitHub Repository](https://github.com/ethz-asl/kalibr)

## Flight Stack

**ArduPilot**  
ArduPilot Development Team. *ArduCopter: An open-source multirotor flight control system.* [Documentation](https://ardupilot.org/copter/)

**MAVROS**  
*MAVLink to ROS Gateway.* [GitHub Repository](https://github.com/mavlink/mavros)

---

**Team 27 | GPS-Denied Navigation Project**  
*Special thanks to the HKUST Aerial Robotics Group for their foundational work on Visual-Inertial Odometry.*

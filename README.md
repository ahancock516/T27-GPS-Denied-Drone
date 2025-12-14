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
*(Place your "Figure 9" image from the PDF here - showing the Blue vs Red line)*
*Figure: VINS-Fusion estimated trajectory vs. Ground Truth GPS during outdoor testing.*

## ⚙️ Hardware Specs
The system is built on a custom 7-inch quadcopter frame.

| Component | Model | Notes |
| :--- | :--- | :--- |
| **Companion Computer** | Raspberry Pi 5 (8GB) | Selected for Quad-Core 2.4GHz CPU |
| **Flight Controller** | Aero Selfie H743 | Running ArduPilot |
| **Camera** | Raspberry Pi Camera 3 Wide | Monocular setup |
| **Optical Flow** | Mico Air MTF-02P | For velocity estimation assistance |
| **GPS/Compass** | HGLRC M100 Pro | Used for Ground Truth comparison |
| **ESC** | 60A 4-in-1 | 2S-6S LiPo rated |

## ⚠️ Critical Lessons Learned (Read Before Building)
1.  **IMU Frequency is King:** VINS-Fusion **requires** a minimum 200Hz IMU update rate. Anything lower (50Hz) caused immediate drift and failure in our tests.
2.  **Magnetic Interference:** The compass is highly sensitive to the high-current wires from the 3-phase motors. **Solution:** You must mount the GPS/Compass module on a mast, at least 8cm away from the main battery and ESC wiring.
3.  **Initialization:** The system requires complete stillness to initialize the IMU bias, followed by slow, consistent movement to establish the VIO lock.
4.  **Compute Limits:** The Raspberry Pi 5 cannot run ORB-SLAM3 smoothly in real-time alongside ROS overhead. VINS-Fusion was much more efficient for this architecture.

## 🛠️ Installation & Setup

### Option A: Docker (Recommended)
We have containerized the environment to handle the complex dependencies of VINS-Fusion and ROS Noetic.

```bash
# Clone the repository
git clone https://github.com/ahancock516/T27-GPS-Denied-Drone.git
cd T27-GPS-Denied-Drone

# Build the image
docker build -t drone-vins .

# Run the container with hardware access
docker run -it --net=host --privileged \
    -v /dev:/dev \
    drone-vins bash

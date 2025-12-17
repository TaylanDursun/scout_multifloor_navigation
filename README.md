# Principles of Robot Autonomy - Scout 2.0 Project

This repository contains the ROS2 Humble packages developed for the **ITU KON414E - Principles of Robot Autonomy** course (Fall 2025-2026).

## 📂 Project Contents

This repository consists of three main packages:

* **ugv_gazebo_sim:** Simulation environments for the AgileX Scout 2.0 robot (Hospital World & Empty World).
* **scout_motion:** Custom ROS2 node for open-loop velocity control (developed in HW1).
* **scout_description:** Robot model files (URDF/Xacro) integrated with ZED 2i Camera and LiDAR sensors (developed in HW2).

## 🚀 Installation

To set up this project on your local machine, follow these steps:

1. **Create a ROS2 workspace:**
   ```bash
   mkdir -p ~/scout_ws/src
   cd ~/scout_ws/src
   ```

2. **Clone this repository:**
   ```bash
   git clone https://github.com/TaylanDursun/scout-v2-autonomy.git .
   ```

3. **Install dependencies and build:**
   ```bash
   cd ~/scout_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

## 🏥 Simulation Usage (HW3)

To launch the robot and sensors within the Hospital environment, use the following command:

```bash
ros2 launch scout_gazebo_sim scout_hospital.launch.py
```

## 🎥 Demo Video

A demonstration of the project running in Gazebo and RViz (showing LiDAR scans and Camera feed) can be viewed here:

[**👉 Scout 2.0 Hospital Simulation - Demo Video (Click Here)**](https://drive.google.com/file/d/15tuyzffxXq9uZGli95QJyApKHphawJ0A/view?usp=sharing)

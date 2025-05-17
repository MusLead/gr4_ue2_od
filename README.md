# 🤖 Trajectory Recorder – Task B2.2

This ROS 2 project allows students to **analyze the robot's position using Odometry and IMU data**. These data streams are published via ROS 2 topics. Your task is to create a custom node that **subscribes to these topics** and records the data for later analysis.

---

## 📌 Prerequisites

This project is intended to run on **Ubuntu 22.04** with **ROS 2 Humble (or later)** properly installed.

### ✅ Required ROS 2 Packages

Ensure the following ROS 2 packages are installed:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-xacro
```

> 🧭 If ROS 2 is not installed, follow the official guide:
> [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

---

## 🚀 How to Build and Run the Node

### 1. Clone the repository

```bash
git clone https://github.com/MusLead/gr4_ue2_od.git
cd gr4_ue2_od
```

### 2. Build the workspace

```bash
colcon build
```

### 3. Source the workspace

```bash
source install/setup.bash
```

### 4. Run the trajectory recording node

```bash
ros2 run trajectory_recorder recorder
```

---

## 🔍 What the Node Does

* Subscribes to:

  * Odometry topic (`/rosbot_base_controller/odom`)
  * IMU topic (`/imu_broadcaster/imu`)
* Logs the following data:

  * **Odometry**: X and Y position
  * **IMU**: Orientation (quaternion), angular velocity, linear acceleration
* Writes the recorded data into:

  * `odometry.txt`
  * `imu.txt`
* Optionally, it can also publish a live **`nav_msgs/Path`** message, which can be visualized in **RViz2**.

---

## 📊 Plotting the Trajectory

The `.txt` output files can be plotted using **Gnuplot**.

For detailed instructions on plotting, please refer to **Übungsblatt 2** from *Robotik – Hochschule Fulda, SoSe 2025*.

---

## 📚 Resources

* [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
* [Gnuplot Manual](http://www.gnuplot.info/documentation.html)
* [Q&A-ChatGPT 17.05.2025](https://chatgpt.com/share/6828be49-8fa8-8007-86fa-60b23236abb1) 
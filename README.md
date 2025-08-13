# Differential Drive Robot Simulation with Lidar

A **ROS 2 + Ignition Gazebo** simulation of a differential drive robot equipped with a **2D lidar sensor**, designed as a foundation for advanced robotics research and personal projects.  
This repository is structured for **easy expansion** — future versions will include **3D lidar simulation**, multiple robots, and various simulated environments.

![Simulation Screenshot](docs/images/sim_preview.png)  
*Example: Differential drive robot with lidar running in Ignition Gazebo.*

---

## Features

- 🛞 **Differential Drive Motion** — Controlled via `/cmd_vel`  
- 📡 **2D Lidar Sensor** — Publishes laser scans on `/scan`  
- 🌍 **Ignition Gazebo Integration** — Realistic physics and rendering  
- 🛠 **Modular URDF** — Easily extendable for new sensors or robot variants  
- 🔄 **ROS 2 Topic Interface** — Compatible with standard navigation and SLAM stacks  

---

## Installation

### 1. Clone the Repository
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/my_diffdrive_lidar_sim.git


References:
https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors/tree/main
https://medium.com/@alitekes1/gazebo-sim-plugin-and-sensors-for-acquire-data-from-simulation-environment-681d8e2ad853
https://www.youtube.com/watch?v=wOa1m8hzrgQ


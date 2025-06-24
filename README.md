# USV Simulation in Gazebo with ROS 2

This repository contains a Gazebo simulation of a simple Unmanned Surface Vehicle (USV) integrated with ROS 2 Humble. The simulation supports thrust control, sensor integration, and basic autonomous navigation.

## Features

### 1. USV modeling in Gazebo
- Constructed using links and joints
- Includes:
  - Main hull
  - Thrusters
  - Camera sensor

### 2. Plugin integration
- Buoyancy
- Hydrodynamics
- Thruster
- Camera

### 3. ROS 2-based thrust control
- Uses `ros_gz_bridge` to interface between ROS and Gazebo
- Topics:
  - `/my_boat/thrust1`, `/my_boat/thrust2` for control
  - `/camera/image` for image data

## Autonomous navigation scenario

- Add static obstacles to the world
- Detect obstacles using camera data
- Avoid obstacles using differential thrust logic

## Installation
```bash
mkdir -p ~/omp_tp/src
cd /omp_tp/src
git clone https://github.com/seungjae99/omp_tp.git
cd .. && colcon build
source install/setup.bash
```

## Usage

```bash
# Launch the Gazebo simulation
# in tp_worlds directory(cd ~/omp_tp/src/omp_tp/tp_worlds)
ign gazebo tp_world.sdf

# play the gazebo simulation

# Launch the bridge node(gazebo <-> ROS2) + obstacle avoidance node + rviz2(camera image)
ros2 launch omp_tp omp_tp.launch.py
```

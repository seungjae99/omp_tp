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

## Usage

```bash
# Launch the Gazebo simulation
ign gazebo tp_world.sdf

# Run ROS-Gazebo bridge
ros2 run ros_gz_bridge parameter_bridge \
  /my_boat/thrust1@std_msgs/msg/Float64@gz.msgs.Double \
  /my_boat/thrust2@std_msgs/msg/Float64@gz.msgs.Double \
  /world/tp_world/model/my_boat/link/base_link/sensor/camera_sensor/image@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /world/tp_world/model/my_boat/link/base_link/sensor/camera_sensor/image:=/camera/image
```

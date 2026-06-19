# onboard_iacquabot

`onboard_iacquabot` is a ROS 2 package that contains the onboard software support for the IAcquaBot Unmanned Surface Vehicle (USV).

---

## 1. System Overview

This package integrates sensory perception (bathymetry, LiDAR, camera), autopilot bridging via MAVROS, and microcontroller actuator interfacing via micro-ROS.

---

## 2. Directory Structure

```directory
onboard_iacquabot/
├── config/                 # MAVROS and PX4 bridge configurations
├── experimental_data/      # Telemetry logs and system identification pipeline
│   ├── step1_least_squares.py      # Stage 1: Bounded linear regression
│   ├── step2_joint_optimization.py # Stage 2: Joint dynamics-thruster optimization
│   └── generate_pdf_report.py      # Validation report generator
├── launch/                 # ROS 2 launch files
└── src/                    # ROS 2 driver and telemetry nodes
```

---



## 3. ROS 2 Communication Interface

The table below lists the ROS 2 topics published or subscribed by the IAcquaBot system, organizing the network interface by module.

| Topic | Message Type | Direction | Nominal Rate (Hz) | Description |
| :--- | :--- | :---: | :---: | :--- |
| **PX4 Autopilot (MAVROS)** | | | | |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Pub | 30 | Local position and orientation |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | Pub | 30 | Linear and angular velocities |
| `/mavros/odometry/in` | `nav_msgs/Odometry` | Pub | 30 | Full vehicle odometry |
| `/mavros/imu/data` | `sensor_msgs/Imu` | Pub | 50 | IMU acceleration and angular rate |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | Pub | 5 | GNSS position (lat, lon, alt) |
| `/mavros/state` | `mavros_msgs/State` | Pub | 1 | Autopilot state (mode, armed status) |
| `/mavros/manual_control/control` | `mavros_msgs/ManualControl` | Pub | 20 | Radio manual control inputs |
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | Sub | 20 | Velocity setpoint for autonomous navigation |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | Sub | 20 | Local position setpoint |
| `/mavros/actuator_control` | `mavros_msgs/ActuatorControl` | Pub | 20 | Low-level motor output command commands |
| **Microcontroller (micro-ROS)** | | | | |
| `/pwm_outputs` | `std_msgs/UInt16MultiArray` | Sub | 20 | PWM command signals $\in [1100, 1900]\,\mu\text{s}$ for the 4 thrusters |
| **Bathymetry Module** | | | | |
| `/lowrance/nmea_raw` | `std_msgs/String` | Pub | -- | Raw NMEA sentences from echo sounder |
| `/lowrance/gps/fix` | `sensor_msgs/NavSatFix` | Pub | -- | Bathymetric GPS fix |
| `/lowrance/gps/vel` | `geometry_msgs/TwistStamped` | Pub | -- | Speed and heading over ground |
| `/lowrance/depth` | `std_msgs/Float32` | Pub | -- | Transduced water depth (m) |
| `/lowrance/water_temp` | `std_msgs/Float32` | Pub | -- | Water temperature measurement (°C) |
| `/lowrance/heading/mag_deg` | `std_msgs/Float64` | Pub | -- | Magnetic compass heading |
| `/lowrance/heading/true_deg` | `std_msgs/Float64` | Pub | -- | True heading over ground |
| **Surface Perception Module** | | | | |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Pub | 10 | 3D LiDAR point cloud |
| `/livox/imu` | `sensor_msgs/Imu` | Pub | 200 | LiDAR integrated IMU telemetry |
| `/iacquabot/sensors/camera/image_raw` | `sensor_msgs/Image` | Pub | 15 | Raw RGB camera frame stream |

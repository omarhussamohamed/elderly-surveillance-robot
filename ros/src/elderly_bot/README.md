# Elderly Bot

**Autonomous Indoor Monitoring Robot (ROS Melodic)**

This repository contains the software stack for **Elderly Bot**, an indoor mobile robot designed for **environment monitoring and SLAM mapping** in elderly-care environments.

The system is currently validated for **mapping and sensor monitoring**. Navigation and patrol are prepared but intentionally disabled.

---

## Platform

- **OS**: Ubuntu 18.04 LTS
- **Middleware**: ROS Melodic (Python 2)
- **Compute**: NVIDIA Jetson (Nano)
- **Microcontroller**: ESP32 (wheel encoders & low-level I/O)
- **Visualization**: Foxglove Studio

---

## Current Verified Capabilities

### Mapping
- 2D SLAM using **GMapping**
- RPLidar-based laser scanning
- EKF-based odometry fusion (wheel encoders + IMU)

### Sensors & Safety
- Digital gas sensor (D0 output)
- Buzzer alert on gas detection
- Jetson temperature monitoring

### System Architecture
- Clean TF tree with no duplicate publishers
- Static geometry defined in URDF
- Motion estimation handled only by EKF

### Cloud (Optional)
- AWS IoT Core telemetry
- AWS Kinesis Video Streams
- Disabled by default during mapping

---

## What Is NOT Active Yet

- Autonomous navigation (move_base)
- Localization using AMCL
- Patrol behavior

These components are configured but **not required** for mapping.

---

## Quick Start (Mapping)

### Setup & Installation
```bash
cd ~/catkin_ws/src/elderly_bot
./install_dependencies.sh
```

### Build Workspace
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Launch Mapping
```bash
roslaunch elderly_bot mapping.launch
```

### Save Generated Map
```bash
rosrun map_server map_saver -f maps/my_house
```

### Visualization (Foxglove Studio)

Monitor these recommended topics:
- `/map` – Occupancy grid
- `/scan` – Laser scan data
- `/tf` – Transform tree
- `/odom` – Odometry estimates
- `/gas_detected` – Gas sensor alerts
- `/jetson_temperature` – Temperature monitoring

---

## Design Rules (Important)

These rules maintain system integrity:

- **EKF** is the only publisher of `odom → base_link`
- **GMapping** is the only publisher of `map → odom`
- **URDF** publishes static TF only
- Hardware-validated axes are never modified in software

---

## Status

This repository reflects the current, tested system state and avoids claiming unimplemented functionality.
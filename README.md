# Elderly Bot  
Autonomous Indoor Monitoring Robot (ROS Melodic)

This repository contains the software stack for **Elderly Bot**, an indoor mobile robot
designed for **environment monitoring and SLAM mapping** in elderly-care environments.

The system is currently validated for **mapping and sensor monitoring**.
Navigation and patrol are prepared but intentionally disabled.

---

## Platform

- Ubuntu 18.04 LTS  
- ROS Melodic (Python 2)  
- NVIDIA Jetson (Nano / Xavier-class)  
- ESP32 (wheel encoders & low-level I/O)  

Visualization is done using **Foxglove Studio**.

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

```bash
cd ~/catkin_ws/src/elderly_bot
./install_dependencies.sh

cd ~/catkin_ws
catkin_make
source devel/setup.bash

roslaunch elderly_bot mapping.launch

## Save the generated map:
rosrun map_server map_saver -f maps/my_house

Visualization (Foxglove)

Recommended topics:
- /map
- /scan
- /tf
- /odom
- /gas_detected
- /jetson_temperature

Design Rules (Important)
- EKF is the only publisher of odom → base_link
- GMapping is the only publisher of map → odom
- URDF publishes static TF only
- Hardware-validated axes are never modified in software

Status
This repository reflects the current, tested system state
and avoids claiming unimplemented functionality.
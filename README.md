# Elderly Bot - Autonomous Indoor Monitoring Robot

**ROS Melodic | NVIDIA Jetson Nano | 4WD Skid-Steer Platform**

A complete autonomous indoor monitoring robot designed for elderly care environments, featuring SLAM mapping, waypoint-based patrol, real-time gas detection with alerts, live video streaming to AWS Kinesis, and cloud telemetry via AWS IoT Core.

> **⚠️ AUTHORITATIVE DOCUMENTATION**  
> Hardware truth → [HARDWARE.md](HARDWARE.md)  
> System architecture truth → [SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)  
> Do **NOT** trust any other .md file for hardware or architecture information — they may be outdated or deleted.

---

## Project Overview

The Elderly Bot is a production-ready ROS 1 system running on NVIDIA Jetson Nano (Ubuntu 18.04 LTS, ROS Melodic) with:

- **SLAM mapping** using RPLidar A1 + GMapping (optimized for Jetson Nano: 1.0 s map update interval, 500 particles, likelihood_field model for robustness to noise)
- **Autonomous navigation** using move_base (Navfn global planner + DWA local planner) and AMCL localization (500 particles, tf_broadcast: true, base_footprint frame)
- **Robust odometry** fusing wheel encoders (from ESP32) + MPU-9250 IMU (via robot_localization EKF with skid-steer slip compensation: no wheel vyaw fusion, high yaw covariance, two_d_mode: true)
- **Gas detection & safety** using MQ-6 sensor (GPIO digital mode on Jetson BOARD pin 18) with automatic buzzer alerts (active buzzer on pin 16, 5-second auto-shutoff)
- **Live video streaming** to AWS Kinesis Video Streams (KVS) at 1280×720 @ 15 fps using hardware-accelerated encoding (nvh264enc) for low CPU usage on Jetson
- **AWS IoT Core integration** for real-time telemetry, alerts, and remote commands (certificates loaded securely via environment variables)
- **System health monitoring** (`system_health_monitor.py`) with automatic respawn of critical nodes (e.g., rplidar_node, ekf_localization_node) and `/diagnostics` topic
- **Primary visualization** — Foxglove Bridge (`ws://<jetson_ip>:8765`) displaying `/map`, `/scan`, `/tf`, `/odom`, `/gas_detected`, `/jetson_temperature` (RViz configs deprecated)

The system is fully Python 2 compatible (ROS Melodic requirement), uses merged costmap configurations, and includes idempotent setup scripts.

**Author**: Omar H. (Giza, Egypt)  
**Date**: January 2026  
**ROS Version**: Melodic (Ubuntu 18.04 LTS)  
**Platform**: NVIDIA Jetson Nano (4GB, JetPack 4.6.x)

---

## Quick Start

### 1. Initial Setup & Dependencies

```bash
# Install all ROS and Python dependencies (idempotent, usually <30s)
cd ~/catkin_ws/src/elderly_bot
./install_dependencies.sh

# Build the workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Set AWS environment variables (recommended: add to ~/.bashrc)
export AWS_ROOT_CA_PATH="/home/omar/catkin_ws/src/elderly_bot/aws_certs/AmazonRootCA1.pem"
export AWS_CERT_PATH="/home/omar/catkin_ws/src/elderly_bot/aws_certs/certificate.pem.crt"
export AWS_KEY_PATH="/home/omar/catkin_ws/src/elderly_bot/aws_certs/private.pem.key"
source ~/.bashrc

# Verify hardware connections
ls /dev/ttyUSB*          # Should show RPLidar (/dev/ttyUSB0)
i2cdetect -y -r 1        # Should show 0x68 (MPU-9250)
```

---

### 2. ESP32 WiFi & Firmware Setup (First Time Only)

- Upload `firmware/elderly_bot_esp32_wifi.ino` using Arduino IDE (ESP32 Dev Module board).
- Connect to temporary WiFi AP **ElderlyBotESP32** and configure:
  - Your home WiFi SSID/password
  - Jetson Nano IP address (e.g., 192.168.1.16)

ESP32 stores settings permanently (`Preferences.h`) and publishes `/odom` at 20 Hz.

---

### 3. Launch Modes

#### Core system (hardware bringup + EKF + sensors + optional cloud):

```bash
roslaunch elderly_bot bringup.launch enable_cloud:=true
```

#### Mapping (create new map with autonomous exploration):

```bash
roslaunch elderly_bot mapping.launch
# Save the map when exploration finishes:
rosrun map_server map_saver -f ~/catkin_ws/src/elderly_bot/maps/my_house
```

#### Navigation & Patrol (use saved map, autonomous patrol):

```bash
roslaunch elderly_bot navigation.launch map_file:=~/catkin_ws/src/elderly_bot/maps/my_house.yaml
# Start autonomous patrol (cycles patrol_goals.yaml waypoints):
rosrun elderly_bot patrol_client.py
```

#### Live Video Streaming (AWS KVS 720p):

```bash
roslaunch elderly_bot kvs_stream.launch
# View live feed in AWS Console → Kinesis Video Streams → RobotStream
```

---

### 4. Viewing & Monitoring

- **Primary viewer:** Foxglove Studio → Connect to `ws://<jetson_ip>:8765`
  - Displays: `/map`, `/scan`, `/tf`, `/odom`, `/gas_detected`, `/jetson_temperature`, costmaps
- **Cloud monitoring:**
  - AWS IoT Core → telemetry, alerts, commands
  - AWS Kinesis Video Streams → live 720p video (us-east-1 region)
- **Diagnostics & health:** `/diagnostics` topic (updated every 10 s)
- **Logs:** `cat ~/.ros/log/latest/rosout.log` or `rqt_console`

---

## Key Features & Safety Mechanisms

- **Odometry reliability:** EKF trusts IMU gyro over wheel encoders for yaw (handles skid-steer slip); high yaw covariance prevents drift accumulation.
- **Gas safety:** MQ-6 auto-triggers buzzer on detection; publishes `/gas_detected` (Bool); 5-second buzzer timeout.
- **Patrol robustness:** `patrol_client.py` retries failed goals (max 3), logs success rate and statistics.
- **Performance optimizations:** AMCL 500 particles, GMapping 1.0 s updates, hardware video encoding.
- **Security:** AWS certificates loaded via environment variables (no hardcoding).
- **Reliability:** All critical nodes (rplidar, EKF, ESP32 serial) have `respawn="true"` and health monitoring.
- **Visualization:** Foxglove Bridge (primary); RViz configs retained only for fallback/debugging.

---

## Dependencies

- **ROS Melodic packages** (installed via apt):  
  `navigation`, `robot_localization`, `gmapping`, `amcl`, `move_base`, `dwa_local_planner`, `rplidar_ros`, `rosserial_python`, `imu_filter_madgwick`, `diagnostic_aggregator`, `cv_bridge`, `image_transport`, `compressed_image_transport`
- **Python 2 packages** (via pip):  
  `paho-mqtt`, `smbus2`, `Jetson.GPIO`, `jetson-stats`, `pyyaml`, `opencv-python`
- **Other:**  
  GStreamer (for KVS), Arduino IDE (for ESP32 firmware)
- **One-command install:**  
  `./install_dependencies.sh` (idempotent, includes udev rules for RPLidar)

---

## Documentation & Support

- `HARDWARE.md` — Authoritative hardware specs, wiring, pinouts
- `SYSTEM_OVERVIEW.md` — Authoritative architecture, nodes, TF tree
- `aws_certs/README.md` — AWS certificate setup
- `CHANGELOG.md` — Full update history
- `docs/TROUBLESHOOTING.md` — Common errors and fixes

---

## For Issues

- Check logs:  
  `cat ~/.ros/log/latest/rosout.log`
- Run diagnostics:  
  `roswtf` or `rqt_console`
- Hardware questions:  
  See `HARDWARE.md`
- Architecture questions:  
  See `SYSTEM_OVERVIEW.md`
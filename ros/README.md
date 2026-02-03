# Robot System (ROS)

> **Part of:** [Elderly Surveillance Robot](../README.md)

**Autonomous Indoor Monitoring Robot using ROS Melodic**

This module contains the ROS stack for **Elderly Bot**, an indoor mobile robot designed for environment monitoring and SLAM mapping.

---

## Platform

| Component | Specification |
|-----------|---------------|
| **OS** | Ubuntu 18.04 LTS |
| **Middleware** | ROS Melodic |
| **Compute** | NVIDIA Jetson Nano |
| **Microcontroller** | ESP32 (wheel encoders, I/O) |
| **Visualization** | Foxglove Studio |

---

## Current Capabilities

### ✅ Verified Features
- **Mapping:** 2D SLAM using GMapping + RPLidar
- **Odometry:** EKF fusion (wheel encoders + IMU)
- **Sensors:** Gas detection, temperature monitoring
- **Alerts:** Buzzer on gas detection
- **Cloud:** AWS IoT Core telemetry (optional)

### ⏳ Prepared (Not Active)
- Autonomous navigation (move_base)
- AMCL localization
- Patrol behavior

---

## Quick Start

### Installation
```bash
cd ~/catkin_ws/src/elderly_bot
./install_dependencies.sh
```

### Build
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Run Mapping
```bash
roslaunch elderly_bot mapping.launch
```

### Save Map
```bash
rosrun map_server map_saver -f maps/my_house
```

---

## Design Rules

These rules maintain system integrity:

| Publisher | Transform |
|-----------|-----------|
| **EKF** | `odom → base_link` |
| **GMapping** | `map → odom` |
| **URDF** | Static TF only |

---

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | OccupancyGrid | SLAM map |
| `/scan` | LaserScan | LiDAR data |
| `/odom` | Odometry | Robot position |
| `/gas_detected` | Bool | Gas sensor alert |
| `/jetson_temperature` | Float32 | CPU temperature |

---

## Cloud Integration

When enabled, telemetry is sent to AWS IoT Core:
- **Topic:** `elderly_bot/telemetry`
- **Protocol:** MQTT over TLS
- **Format:** JSON

📖 See [cloud/README.md](../cloud/README.md) for AWS setup.

---

## Related Documentation

- [Main README](../README.md) — System overview
- [Cloud Setup](../cloud/README.md) — AWS configuration
- [Hardware Guide](docs/HARDWARE.md) — Hardware setup
- [System Overview](docs/SYSTEM_OVERVIEW.md) — Architecture details
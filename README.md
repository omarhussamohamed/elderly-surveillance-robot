# Elderly Bot - Autonomous Indoor Monitoring Robot

**ROS Melodic | Jetson Nano | 4WD Skid-Steer Platform**

An autonomous indoor monitoring robot with mapping, navigation, and AWS IoT cloud integration capabilities.

> **⚠️ AUTHORITATIVE DOCUMENTATION**  
> Hardware truth → [HARDWARE.md](HARDWARE.md)  
> System architecture truth → [SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)  
> Do NOT trust any other .md file for hardware or architecture information — they may be outdated or deleted.

---

## Project Overview

The Elderly Bot is a complete ROS 1 system designed for autonomous indoor monitoring with:
- **SLAM mapping** using RPLidar A1 and GMapping
- **Autonomous navigation** using move_base and AMCL
- **Sensor fusion** for robust odometry (wheel encoders + IMU)
- **Gas detection** using MQ-6 sensor with alert system
- **Cloud connectivity** via AWS IoT Core for remote monitoring

---

## Hardware Configuration

### Computing Platform
- **Main Controller**: NVIDIA Jetson Nano (Ubuntu 18.04, ROS Melodic)
- **Motor Controller**: ESP32 (Arduino rosserial, WiFi)

### Robot Geometry
| Parameter | Value |
|-----------|-------|
| Dimensions | 35cm × 25cm × 14cm (L×W×H) |
| Drivetrain | 4WD skid-steer |
| Track width | 0.26m |
| Wheelbase | 0.17m |
| Wheel diameter | 0.065m |

### Motors & Encoders
- **Motor**: JGB37-520, 12V, 110 RPM, 90:1 gearbox
- **Encoder**: Hall effect quadrature, 11 PPR × 90 = **3960 ticks/revolution**
- **Distance per tick**: 0.0000515m (calibrated)
- **Debouncing**: 100µs software filter

### Sensors
| Sensor | Model | Interface | Purpose |
|--------|-------|-----------|---------|
| **Lidar** | RPLidar A1 | USB (/dev/ttyUSB0) | 2D laser scanning (360°, 8Hz) |
| **IMU** | MPU-9250 | I2C (bus 1, 0x68) | Orientation (gyro + accel, mag disabled) |
| **Gas Sensor** | MQ-6 | GPIO (Pin 18) | LPG/natural gas detection |
| **Jetson Stats** | jtop | Software | Temperature & power monitoring |

**IMU Notes:**
- Magnetometer disabled due to indoor electromagnetic interference
- Dynamic gyroscope calibration on startup
- Madgwick filter fusion for orientation estimation
- Connected directly to Jetson I2C: SDA→Pin 3, SCL→Pin 5

**Gas Sensor Notes:**
- GPIO Mode (default): Digital D0 output (active-low with external pull-up)
- Binary detection with 500ms debouncing
- Auto-triggers buzzer alarm on gas detection

### Actuators
- **Active Buzzer**: Alert/alarm notifications (Pin 16, transistor-driven, 5V)
  - Continuous beeping pattern (0.1s ON/OFF)
  - ROS topic: `/buzzer_command` (std_msgs/Bool)

### Motor Drivers
- 2× L298N dual H-bridge motor drivers

---

## Repository Structure

```
elderly_bot/
├── config/                      # Configuration files
├── launch/                      # Launch files
├── scripts/                     # ROS nodes (Python)
├── urdf/                        # Robot description
├── rviz/                        # Visualization configs
├── firmware/                    # ESP32 Arduino code
├── aws_certs/                   # AWS IoT certificates (gitignored)
├── maps/                        # Saved maps
├── install_dependencies.sh      # Dependency installer
├── CMakeLists.txt              # Catkin build config
├── package.xml                 # ROS package manifest
├── README.md                   # This file
├── HARDWARE.md                 # Hardware configuration (authoritative)
└── SYSTEM_OVERVIEW.md          # System architecture (authoritative)
```

---

## Quick Start

### 1. Initial Setup (First Time Only)

```bash
cd ~/catkin_ws/src/elderly_bot
chmod +x install_dependencies.sh
./install_dependencies.sh
```

This installs:
- ROS navigation stack
- Python dependencies (AWSIoTPythonSDK, jtop, smbus2, etc.)
- RPLidar ROS driver
- robot_localization package
- imu_filter_madgwick

### 2. Build the Workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Hardware Connections

- **RPLidar**: Connect to USB → `/dev/ttyUSB0`
- **ESP32**: Ensure WiFi connection (see HARDWARE.md for details)
- **IMU**: Verify I2C connection: `i2cdetect -y -r 1` (should show 0x68)
- **Gas Sensor**: Connected to Pin 18 with external pull-up resistor
- **Buzzer**: Connected to Pin 16 via transistor

See [HARDWARE.md](HARDWARE.md) for complete wiring details.

### 4. Launch Modes

**Mapping Mode** (Create a new map):
```bash
roslaunch elderly_bot mapping.launch
```

**Navigation Mode** (Use existing map):
```bash
roslaunch elderly_bot navigation.launch map_file:=/path/to/map.yaml
```

**Full System** (sensors, IMU, optional cloud):
```bash
roslaunch elderly_bot bringup.launch enable_cloud:=false
```

**Patrol Mode** (Autonomous waypoint patrol):
```bash
roslaunch elderly_bot navigation.launch
rosrun elderly_bot patrol_client.py
```

---

## System Architecture

### TF Tree

```
map
 └── odom                    [AMCL or EKF]
     └── base_footprint      [robot_localization EKF]
         └── base_link       [robot_state_publisher]
             ├── laser       [180° yaw for rear-mounted lidar]
             └── imu_link    [aligned with robot frame]
```

### Node Graph

```
┌─────────────────┐
│ ESP32 Firmware  │  (Arduino rosserial)
│  - Motor control│
│  - Encoders     │
└────────┬────────┘
         │ /cmd_vel (Twist)
         │ /odom (Odometry)
         ▼
┌─────────────────┐     ┌──────────────┐
│ robot_localiza- │◄────┤ mpu9250_node │
│ tion (EKF)      │     │  IMU driver  │
└────────┬────────┘     └──────────────┘
         │ odom→base_footprint TF
         ▼
┌─────────────────┐     ┌──────────────┐
│   RPLidar Node  │────►│  GMapping    │  (Mapping mode)
│ /scan (LaserScan)│     │  or AMCL     │  (Navigation)
└─────────────────┘     └──────┬───────┘
                               │ map→odom TF
                               ▼
                        ┌──────────────┐
                        │  move_base   │
                        │  navigation  │
                        └──────────────┘
```

### Important Topics

| Topic | Type | Publisher | Purpose |
|-------|------|-----------|---------|
| `/cmd_vel` | Twist | move_base | Motor velocity commands |
| `/odom` | Odometry | ESP32 | Wheel encoder odometry |
| `/scan` | LaserScan | rplidar_node | 2D laser scans |
| `/imu/data_raw` | Imu | mpu9250_node | Raw IMU measurements |
| `/imu/data` | Imu | imu_filter_madgwick | Fused IMU with orientation |
| `/gas_detected` | Bool | sensors_actuators_node | Gas detection status |
| `/buzzer_command` | Bool | (command) | Buzzer control |
| `/jetson_temperature` | Temperature | sensors_actuators_node | Jetson temp |
| `/jetson_power` | Float32 | sensors_actuators_node | Power consumption |

---

## AWS IoT & Video Streaming Integration

### AWS Kinesis Video Streams (KVS)

The robot streams live camera feed to AWS for remote monitoring. Credentials are embedded in the launch file for seamless operation.

**Launch Camera Streaming:**
```bash
# Camera only (no AWS streaming)
roslaunch elderly_bot camera_streaming.launch enable_kvs:=false

# Camera + AWS KVS streaming (default)
roslaunch elderly_bot camera_streaming.launch
```

**Features:**
- 1280x720 @ 30fps MJPEG camera feed
- Streams to AWS KVS stream: `RobotStream` in `us-east-1` region
- Automatic reconnection on network interruptions
- Publishes `/camera/image_raw` and `/camera/image_raw/compressed` topics

**View Stream:**
- AWS Console: https://us-east-1.console.aws.amazon.com/kinesisvideo/

### AWS IoT Core (Optional)

1. Follow instructions in `aws_certs/README.md` to download certificates
2. Update `config/cloud_config.yaml` with your endpoint and Thing name
3. Test connection: `python2 scripts/final_handshake.py` (if final_handshake.py is available)

**Features:**
- Publishes telemetry to `elderly_bot/telemetry` (1 Hz)
- Publishes alerts to `elderly_bot/alerts` (event-driven)
- Receives commands from `elderly_bot/commands`

**Launch with Cloud:**
```bash
roslaunch elderly_bot bringup.launch enable_cloud:=true
```

---

## Calibration & Tuning

See [SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md) for complete operational modes and parameter details.

**Key Calibration Points**:
- Encoder resolution: 3960 ticks/rev (calibrated)
- IMU gyro: Auto-calibrated on startup (robot must be stationary)
- Navigation parameters: Tuned in config/*.yaml files

---

## Troubleshooting

### Common Issues

**Lidar not detected**:
```bash
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0
```

**IMU not detected**:
```bash
i2cdetect -y -r 1  # Should show 0x68
sudo apt-get install i2c-tools python3-smbus
```

**ESP32 not connecting**:
- Verify WiFi credentials in firmware
- Check Jetson is on same network
- Verify rosserial connection: `rostopic echo /rosout`

**Gas sensor always triggered**:
- Check pull-up resistor (2.2-4.7kΩ to 3.3V)
- Verify polarity setting in `sensors_actuators.yaml`
- Allow 24-hour burn-in period for MQ-6

**Navigation fails**:
- Verify map is loaded: `rostopic echo /map -n1`
- Check TF tree: `rosrun tf view_frames`
- Ensure AMCL localized: `rostopic echo /amcl_pose -n1`

### Logs and Diagnostics

```bash
# View all active nodes
rosnode list

# Check TF tree
rosrun tf view_frames
evince frames.pdf

# Monitor topics
rostopic hz /scan
rostopic hz /odom
rostopic hz /imu/data

# Debug navigation
rostopic echo /move_base/status
rostopic echo /move_base/feedback
```

---

## Development

### Building from Source

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Running Tests

```bash
# Test AWS connection
python ~/catkin_ws/src/elderly_bot/scripts/final_handshake.py

# Test IMU
rostopic echo /imu/data_raw

# Test gas sensor
rostopic echo /gas_detected
```

### Code Style

- Python: PEP 8, 4 spaces, snake_case
- Launch files: 2-space indentation
- YAML configs: 2-space indentation, comments above params

---

## License & Contributors

**Project**: Graduation Project - Elderly Care Robot  
**Author**: Omar H. (Cairo)  
**Date**: January 2026  
**ROS Version**: Melodic (Ubuntu 18.04)  
**Platform**: NVIDIA Jetson Nano

---

## Documentation

- **[HARDWARE.md](HARDWARE.md)**: Complete hardware specifications & wiring (authoritative source)
- **[SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)**: System architecture & operational modes (authoritative source)
- **[aws_certs/README.md](aws_certs/README.md)**: AWS IoT Core certificate setup

---

## Support

For issues, check ROS logs:
```bash
cat ~/.ros/log/latest/rosout.log
```

For hardware/architecture questions, refer to HARDWARE.md and SYSTEM_OVERVIEW.md.



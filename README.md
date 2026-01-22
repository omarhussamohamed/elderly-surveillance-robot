# Elderly Bot - Autonomous Indoor Monitoring Robot

A complete ROS 1 Melodic system for a 4WD autonomous indoor monitoring robot with mapping and patrol capabilities.

## Overview

The Elderly Bot is designed for autonomous indoor monitoring with two operational modes:
1. **Mapping Mode**: Autonomous exploration and map creation
2. **Navigation Mode**: Patrol predefined waypoints using the saved map

## Hardware Specifications

### Computing
- **Main Computer**: NVIDIA Jetson Nano (Ubuntu 18.04 + ROS Melodic)
- **Motor Controller**: ESP32 38-pin (Arduino firmware with rosserial)

### Robot Geometry
- **Dimensions**: 35cm (L) × 25cm (W) × 14cm (H)
- **Drivetrain**: 4WD skid-steer
- **Track Width**: 0.26m
- **Wheelbase**: 0.17m
- **Wheel Diameter**: 0.065m

### Motors & Encoders
- **Motor**: JGB37-520, 12V, 110 RPM, 90:1 gearbox
- **Encoder**: Hall effect quadrature, 11 PPR × 90:1 × 4 edges = **3960 ticks/rev**
- **Wheel Diameter**: 65mm (radius 32.5mm)
- **Effective Radius**: 32.5mm (calibrated for 1:1 kinematic mapping)
- **Distance per tick**: 0.0000515m
- **Debouncing**: 100µs software filter for noise rejection
- See [HARDWARE_MAP.md](HARDWARE_MAP.md) and [KINEMATIC_FIX_APPLIED.md](KINEMATIC_FIX_APPLIED.md)

### Sensors
- **Lidar**: RPLidar A1 (mounted 0.30m above ground)
- **IMU**: MPU-9250 (gyroscope + accelerometer only)
  - Connected via I2C to Jetson Nano (bus 1, address 0x68)
  - Dynamic gyro calibration on startup
  - **Magnetometer DISABLED** (indoor EMI from motors/PSU/battery)
  - Madgwick fusion provides orientation from gyro+accel
  - See [docs/IMU_CALIBRATION.md](docs/IMU_CALIBRATION.md)
- **Gas Sensor**: MQ-6 (LPG/natural gas detection)
  - **GPIO Mode** (default): Digital D0 output → Jetson Pin 18 (GPIO 24)
  - Binary detection: HIGH = gas detected, LOW = no gas
  - Alternative I2C mode: A0 analog via ADS1115 ADC (requires additional hardware)
  - Wiring: [docs/GAS_SENSOR_WIRING.md](docs/GAS_SENSOR_WIRING.md)
  - Currently ENABLED for testing
- **Jetson Monitoring**: System temperature and power consumption
  - Uses jetson-stats (jtop) library
  - Real-time hardware health monitoring

### Actuators
- **Active Buzzer**: Alert/alarm notifications (5V, transistor-driven)
  - Jetson Pin 12 (GPIO 18) → 1kΩ → 2N2222 Base
  - Buzzer driven by 5V rail via transistor switching (30-50mA)
  - Auto-shutoff after 5 seconds for safety
  - Wiring: [docs/BUZZER_WIRING.md](docs/BUZZER_WIRING.md)
  - Currently ENABLED for testing

### Motor Drivers
- 2× L298N dual H-bridge motor drivers

## System Architecture

### TF Tree
```
map
 └── odom                    [published by AMCL or robot_localization]
     └── base_footprint      [published by robot_localization EKF]
         └── base_link       [published by robot_state_publisher]
             ├── laser       [180° yaw rotation for backward-facing lidar]
             └── imu_link    [aligned with robot frame]
```

**TF Publishers:**
- `robot_state_publisher`: Broadcasts all URDF transforms (base_footprint, base_link, laser, imu_link)
- `robot_localization` EKF: Publishes odom → base_footprint transform
- AMCL (navigation mode): Publishes map → odom transform

## Quick Links

### Essential Documentation
- **[QUICK_START.md](QUICK_START.md)**: Step-by-step setup guide
- **[SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)**: Complete system architecture
- **[HARDWARE_MAP.md](HARDWARE_MAP.md)**: Hardware configuration reference

### Critical Fixes & Calibration
- **[TF_FRAME_ALIGNMENT_FIX.md](TF_FRAME_ALIGNMENT_FIX.md)**: ⭐ **TF coordinate frame alignment fix** ⭐
- **[COMPLETE_TF_DEPLOYMENT.md](COMPLETE_TF_DEPLOYMENT.md)**: 🚀 **Complete TF fix deployment procedure** 🚀
- **[DRIFT_FIX_APPLIED.md](DRIFT_FIX_APPLIED.md)**: ✅ **Solution for map ghosting/rotation drift** ✅
- **[KINEMATIC_FIX_APPLIED.md](KINEMATIC_FIX_APPLIED.md)**: 🎯 **Fix for 1cm→1m odometry scaling** 🎯

### Testing & Validation
- **[PHYSICAL_VALIDATION_PROTOCOL.md](PHYSICAL_VALIDATION_PROTOCOL.md)**: Manual testing procedures
- **[IMU_MOUNTING_FIX_REFERENCE.md](IMU_MOUNTING_FIX_REFERENCE.md)**: IMU orientation scenarios

### Setup Guides
- **[ROSSERIAL_GUIDE.md](ROSSERIAL_GUIDE.md)**: rosserial WiFi setup
- **[ESP32_FIRMWARE_UPLOAD.txt](ESP32_FIRMWARE_UPLOAD.txt)**: Firmware upload instructions
- **[docs/IMU_CALIBRATION.md](docs/IMU_CALIBRATION.md)**: IMU sensor fusion
- **[docs/MPU9250_JETSON_SETUP.md](docs/MPU9250_JETSON_SETUP.md)**: IMU hardware setup

## Hardware Setup

### Connections
- **RPLidar**: Connect to `/dev/ttyUSB0`
- **ESP32**: WiFi connection (see HARDWARE_MAP.md for credentials)
- **IMU**: Connected directly to Jetson I2C bus 1 (see docs/MPU9250_JETSON_SETUP.md)
  - SDA → Pin 3 on J21 header
  - SCL → Pin 5 on J21 header
  - VCC → 3.3V, GND → GND
- **Gas Sensor (Optional)**: MQ-6 via ADS1115 ADC on I2C bus 1
  - ADS1115 address: 0x48
  - MQ-6 analog output → ADS1115 channel A0
  - VCC → 5V, GND → GND
- **Buzzer (Optional)**: Active buzzer on Jetson GPIO
  - Configure pin number in config/sensors_actuators.yaml
  - Uses BOARD pin numbering
- **AWS IoT Core (Optional)**: Cloud connectivity
  - Certificates stored in ~/aws_certs/ on Jetson
  - See aws_certs/README.md for setup instructions

### Serial Permissions
```bash
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect
```

## Usage

See **[SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)** for detailed operational modes and usage instructions.

### Testing Hardware Only

Test sensors and motors without navigation:

```bash
roslaunch elderly_bot bringup.launch

# In another terminal, check topics:
rostopic list
rostopic echo /scan
rostopic echo /imu/data
rostopic echo /wheel_odom

# Test motors:
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" -r 10
```

## Configuration

See **[SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)** for detailed configuration instructions.

## Package Structure

```
elderly_bot/
├── config/                      # Configuration files
│   ├── amcl.yaml               # AMCL localization parameters
│   ├── costmap_common_params.yaml
│   ├── dwa_local_planner.yaml  # Local planner tuning
│   ├── ekf.yaml                # Sensor fusion configuration
│   ├── global_costmap.yaml
│   ├── gmapping.yaml           # SLAM parameters
│   ├── local_costmap.yaml
│   └── patrol_goals.yaml       # Patrol waypoints
├── firmware/
│   ├── elderly_bot_esp32_wifi.ino      # Unified ESP32 firmware (WiFi rosserial)
│   ├── motor_and_encoder_HW_test.ino   # Motor behavior ground truth
│   └── motor_and_encoder_SW_test.ino   # Software test firmware
├── launch/
│   ├── bringup.launch          # Hardware interfaces
│   ├── mapping.launch          # Autonomous mapping mode
│   └── navigation.launch       # Navigation and patrol mode
├── maps/                        # Saved maps (generated)
│   ├── house_map.yaml
│   └── house_map.pgm
├── scripts/
│   ├── mpu9250_node.py         # Jetson IMU driver
│   └── patrol_client.py        # Patrol action client
├── CMakeLists.txt
├── package.xml
├── install_dependencies.sh
└── README.md
```

## Troubleshooting

### Robot doesn't move
- Check `/cmd_vel` is being published: `rostopic echo /cmd_vel`
- Check ESP32 connection: `rostopic list | grep wheel_odom`
- Verify motor wiring and power supply
- Check motor direction inversion flags

### Poor localization
- Ensure sufficient features in environment
- Check IMU calibration (keep robot still during startup)
- Tune AMCL parameters in `config/amcl.yaml`
- Verify TF tree: `rosrun tf view_frames`

### Navigation failures
- Check costmap inflation radius
- Tune DWA planner parameters
- Verify footprint matches robot dimensions
- Check for TF transform errors: `roswtf`

### Lidar not detected
- Check USB connection: `ls -l /dev/ttyUSB*`
- Verify udev rules: `cat /etc/udev/rules.d/99-rplidar.rules`
- Check permissions: `groups` (should include dialout)
- Test lidar: `roslaunch rplidar_ros view_rplidar.launch`

### ESP32 communication issues
- Check WiFi connection (see HARDWARE_MAP.md for credentials)
- Verify rosserial connection: `rostopic list`
- Check ESP32 serial output for errors
- Ensure ros_lib is generated on Jetson
- See `ROSSERIAL_GUIDE.md` for detailed rosserial troubleshooting

## Safety Features

- **Velocity Clamping**: All commands limited to safe speeds
- **Timeout Protection**: Motors stop if no command received for 500ms
- **Obstacle Avoidance**: Dynamic costmap with inflation
- **Recovery Behaviors**: Automatic recovery from navigation failures
- **Watchdog**: ESP32 monitors command stream

## Performance Specifications

- **Mapping Speed**: ~0.15 m/s during exploration
- **Navigation Speed**: Up to 0.25 m/s
- **Localization Accuracy**: ±5cm position, ±3° orientation
- **Control Frequency**: 100 Hz (ESP32), 10 Hz (move_base)
- **Sensor Fusion**: 50 Hz (EKF)

## ROS Topics

### Published by ESP32
- `/wheel_odom` (nav_msgs/Odometry) - Wheel odometry
- `/imu/data` (sensor_msgs/Imu) - IMU measurements

### Published by Jetson
- `/scan` (sensor_msgs/LaserScan) - Lidar data
- `/odom` (nav_msgs/Odometry) - Fused odometry (EKF)
- `/map` (nav_msgs/OccupancyGrid) - Occupancy grid map
- `/move_base/goal` (move_base_msgs/MoveBaseActionGoal) - Navigation goals

### Subscribed
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

## Parameters

### Patrol Client
```bash
rosrun elderly_bot patrol_client.py \
    _patrol_file:=/path/to/goals.yaml \
    _goal_timeout:=300.0 \
    _inter_goal_delay:=2.0 \
    _max_retries:=3
```

## Development

### Building
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Testing
```bash
# Test individual components
roslaunch elderly_bot bringup.launch
roslaunch rplidar_ros view_rplidar.launch

# Check TF tree
rosrun tf view_frames
evince frames.pdf

# Monitor topics
rostopic hz /scan
rostopic hz /wheel_odom
rostopic hz /imu/data
```

## License

BSD 3-Clause License

## Support

For issues and questions:
1. Check this README
2. Review ROS logs: `roscd elderly_bot && cat ~/.ros/log/latest/*.log`
3. Check ROS wiki: http://wiki.ros.org

## References

- [ROS Navigation Stack](http://wiki.ros.org/navigation)
- [robot_localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
- [gmapping](http://wiki.ros.org/gmapping)
- [AMCL](http://wiki.ros.org/amcl)
- [REP-103: Standard Units and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)



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
- **Encoder**: Quadrature, 11 PPR, 90:1 reduction, 4-edge counting
- **Total Resolution**: 990 ticks per wheel revolution
- **Calibrated Wheel Radius**: 19.4mm (effective, under load)
- **Debouncing**: 100µs software filter for noise rejection
- See [docs/ENCODER_CALIBRATION.md](docs/ENCODER_CALIBRATION.md)

### Sensors
- **Lidar**: RPLidar A1 (mounted 0.30m above ground)
- **IMU**: MPU-9250 (gyroscope + accelerometer + magnetometer)
  - Connected via I2C to Jetson Nano
  - Dynamic gyro calibration on startup
  - Magnetometer for absolute heading
  - See [docs/IMU_CALIBRATION.md](docs/IMU_CALIBRATION.md)

### Motor Drivers
- 2× L298N dual H-bridge motor drivers

## System Architecture

### TF Tree
```
map
 └── odom
     └── base_footprint
         └── base_link
             ├── laser
             └── imu_link
```

## Quick Links

- **[QUICK_START.md](QUICK_START.md)**: Step-by-step setup guide
- **[SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)**: Complete system architecture and operation
- **[HARDWARE_MAP.md](HARDWARE_MAP.md)**: Hardware configuration reference
- **[ROSSERIAL_GUIDE.md](ROSSERIAL_GUIDE.md)**: rosserial setup and troubleshooting
- **[DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md)**: Final preparation checklist
- **[docs/IMU_CALIBRATION.md](docs/IMU_CALIBRATION.md)**: IMU sensor fusion and calibration
- **[docs/ENCODER_CALIBRATION.md](docs/ENCODER_CALIBRATION.md)**: Encoder debouncing and odometry tuning
- **[docs/MPU9250_JETSON_SETUP.md](docs/MPU9250_JETSON_SETUP.md)**: IMU hardware setup guide

## Hardware Setup

### Connections
- **RPLidar**: Connect to `/dev/ttyUSB0`
- **ESP32**: WiFi connection (see HARDWARE_MAP.md for credentials)
- **IMU**: Connected directly to Jetson I2C bus 1 (see docs/MPU9250_JETSON_SETUP.md)
  - SDA → Pin 3 on J21 header
  - SCL → Pin 5 on J21 header
  - VCC → 3.3V, GND → GND

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



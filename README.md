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
- **Total Resolution**: 3960 ticks per wheel revolution

### Sensors
- **Lidar**: RPLidar A1 (mounted 0.30m above ground)
- **IMU**: MPU-9250 (gyroscope + accelerometer)

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

### Node Responsibilities

**ESP32 (Arduino)**
- Encoder reading via interrupts
- Wheel velocity PID control (100 Hz)
- Skid-steer kinematics
- IMU data publishing
- Safety timeouts and velocity clamping

**Jetson Nano (ROS)**
- Sensor fusion (robot_localization EKF)
- Mapping (gmapping)
- Localization (AMCL)
- Path planning (move_base)
- Patrol logic

## Installation

### Prerequisites
- Ubuntu 18.04
- ROS Melodic
- Arduino IDE (for ESP32 programming)

### Quick Install

```bash
cd ~/catkin_ws/src/elderly_bot
chmod +x install_dependencies.sh
./install_dependencies.sh
```

This script installs:
- ROS navigation stack
- robot_localization
- gmapping
- explore_lite
- rplidar_ros
- rosserial
- All required dependencies

### Manual Installation

If you prefer manual installation:

```bash
# Install ROS packages
sudo apt-get install -y \
    ros-melodic-navigation \
    ros-melodic-robot-localization \
    ros-melodic-gmapping \
    ros-melodic-explore-lite \
    ros-melodic-rplidar-ros \
    ros-melodic-rosserial-arduino \
    ros-melodic-rosserial-python

# Build workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### ESP32 Firmware Setup

⚠️ **CRITICAL**: This firmware requires **ESP32 Arduino Core 2.0.17**. Do NOT use Core 3.x due to compatibility issues. See `firmware/ESP32_CORE_COMPATIBILITY.md` for details.

1. **Install Arduino IDE** (if not already installed)
   - Download from: https://www.arduino.cc/en/software

2. **Add ESP32 Board Support**
   - File → Preferences
   - Add to "Additional Board Manager URLs":
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Tools → Board → Board Manager
   - Search "esp32"
   - **Select version 2.0.17** from dropdown
   - Click Install

3. **Install Required Libraries**
   - Sketch → Include Library → Manage Libraries
   - Install: "Rosserial Arduino Library"
   - Install: "MPU9250" by hideakitai

4. **Generate ROS Library for Arduino**
   ```bash
   cd ~/Arduino/libraries
   rm -rf ros_lib
   rosrun rosserial_arduino make_libraries.py .
   ```

5. **Upload Firmware**
   - Open: `~/catkin_ws/src/elderly_bot/firmware/elderly_bot_esp32.ino`
   - Select Board: "ESP32 Dev Module"
   - **Verify Core Version**: Tools → Board → Board Manager → esp32 should show 2.0.17
   - Select Port: `/dev/ttyUSB1` (or appropriate port)
   - Upload

**Troubleshooting ESP32 Issues**: If you experience compilation errors or crashes, see `firmware/FIRMWARE_TROUBLESHOOTING.md` for comprehensive troubleshooting. For detailed technical information about ESP32 Core compatibility, see `firmware/ESP32_CORE_COMPATIBILITY.md`.

## Hardware Setup

### Connections
- **RPLidar**: Connect to `/dev/ttyUSB0`
- **ESP32**: Connect to `/dev/ttyUSB1`
- **IMU**: I2C (SDA=21, SCL=22)

### Serial Permissions
```bash
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect
```

## Usage

### Mode 1: Autonomous Mapping

Create a map of your environment:

```bash
# Terminal 1: Start mapping
roslaunch elderly_bot mapping.launch

# The robot will autonomously explore using explore_lite
# Monitor progress in RViz

# When exploration is complete, save the map:
rosrun map_server map_saver -f ~/catkin_ws/src/elderly_bot/maps/house_map
```

**What happens:**
- Robot autonomously explores unknown areas
- gmapping builds a 2D occupancy grid map
- explore_lite identifies and navigates to frontier regions
- EKF fuses wheel odometry and IMU for accurate pose estimation

### Mode 2: Navigation & Patrol

Navigate using the saved map:

```bash
# Terminal 1: Start navigation
roslaunch elderly_bot navigation.launch map_file:=~/catkin_ws/src/elderly_bot/maps/house_map.yaml

# Terminal 2: Start patrol
rosrun elderly_bot patrol_client.py
```

**What happens:**
- AMCL localizes robot on the saved map
- move_base plans paths to patrol waypoints
- Robot cycles through waypoints indefinitely
- Dynamic obstacle avoidance enabled

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

### Patrol Waypoints

Edit patrol goals in `config/patrol_goals.yaml`:

```yaml
patrol_goals:
  - name: "Living Room"
    x: 2.0
    y: 1.5
    yaw: 0.0
  
  - name: "Kitchen"
    x: 5.0
    y: 1.0
    yaw: 1.57
```

**To find coordinates:**
1. Launch navigation mode
2. Use "2D Pose Estimate" in RViz to move robot
3. Note coordinates from `/amcl_pose` topic
4. Update `patrol_goals.yaml`

### Velocity Limits

Edit in `config/dwa_local_planner.yaml`:

```yaml
max_vel_x: 0.25        # m/s
max_vel_theta: 1.0     # rad/s
```

### PID Tuning (ESP32)

Edit in `firmware/elderly_bot_esp32.ino`:

```cpp
float kp = 25.0;
float ki = 5.0;
float kd = 0.5;
```

### Motor Direction Inversion

If motors spin in wrong direction, edit in firmware:

```cpp
bool invert_fl = false;  // Front-Left
bool invert_fr = true;   // Front-Right
bool invert_rl = false;  // Rear-Left
bool invert_rr = true;   // Rear-Right
```

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
│   ├── elderly_bot_esp32.ino           # ESP32 Arduino firmware
│   ├── ESP32_CORE_COMPATIBILITY.md     # ESP32 Core compatibility guide
│   ├── FIRMWARE_CHANGES.md             # Firmware changelog
│   └── FIRMWARE_TROUBLESHOOTING.md     # Troubleshooting guide
├── launch/
│   ├── bringup.launch          # Hardware interfaces
│   ├── mapping.launch          # Autonomous mapping mode
│   └── navigation.launch       # Navigation and patrol mode
├── maps/                        # Saved maps (generated)
│   ├── house_map.yaml
│   └── house_map.pgm
├── scripts/
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
- Check baud rate (115200)
- Verify rosserial connection: `rostopic list`
- Check Arduino serial monitor for errors
- Ensure ros_lib is up to date
- See `firmware/FIRMWARE_TROUBLESHOOTING.md` for detailed troubleshooting

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



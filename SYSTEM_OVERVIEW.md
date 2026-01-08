# Elderly Bot - System Overview

## Complete Package Contents

This is a production-ready ROS 1 Melodic package for an autonomous indoor 4WD monitoring robot.

### Package Structure

```
elderly_bot/
├── CMakeLists.txt                    # Catkin build configuration
├── package.xml                       # ROS package manifest
├── README.md                         # Comprehensive documentation
├── QUICK_START.md                    # Quick start guide
├── SYSTEM_OVERVIEW.md               # This file
├── HARDWARE_MAP.md                  # Hardware configuration reference
├── ROSSERIAL_GUIDE.md               # rosserial setup and troubleshooting
│
├── config/                          # Configuration files (YAML)
│   ├── amcl.yaml                   # AMCL localization parameters
│   ├── costmap_common_params.yaml  # Shared costmap configuration
│   ├── dwa_local_planner.yaml      # DWA local planner tuning
│   ├── ekf.yaml                    # EKF sensor fusion config
│   ├── global_costmap.yaml         # Global costmap parameters
│   ├── gmapping.yaml               # GMapping SLAM parameters
│   ├── local_costmap.yaml          # Local costmap parameters
│   └── patrol_goals.yaml           # Patrol waypoints
│
├── firmware/                        # ESP32 Arduino firmware
│   ├── elderly_bot_esp32_wifi.ino   # Unified ESP32 firmware (WiFi rosserial)
│   ├── motor_and_encoder_HW_test.ino # Hardware test firmware
│   └── motor_and_encoder_SW_test.ino # Software test firmware
│
├── launch/                          # ROS launch files
│   ├── bringup.launch              # Hardware interfaces
│   ├── mapping.launch              # Autonomous mapping mode
│   └── navigation.launch           # Navigation & patrol mode
│
├── maps/                            # Saved maps (generated)
│   └── README.md                   # Map directory documentation
│
├── rviz/                            # RViz configurations
│   ├── mapping.rviz                # Mapping visualization
│   └── navigation.rviz             # Navigation visualization
│
├── scripts/                         # Python scripts
│   ├── mpu9250_node.py             # Jetson IMU driver
│   └── patrol_client.py            # Patrol action client
│
└── install_dependencies.sh          # Dependency installation script
```

## System Architecture

### Hardware Layer
- **Jetson Nano**: Main computer (Ubuntu 18.04 + ROS Melodic)
- **ESP32**: Motor controller (Arduino + rosserial over WiFi)
- **RPLidar A1**: 360° laser scanner
- **MPU-9250**: IMU (gyro + accel) - connected directly to Jetson
- **4× Motors**: JGB37-520 with encoders
- **2× L298N**: Motor drivers

### Software Stack

#### ESP32 Responsibilities (Dual-Core Architecture)
- **Core 0**: WiFi rosserial communication, odometry publishing
- **Core 1**: Motor control, encoder reading, PWM updates
- Differential drive kinematics (left = linear.x - angular.z, right = linear.x + angular.z)
- Quadrature encoder reading (interrupt-driven)
- Hardware PWM motor control with minimum power thresholds
- Safety timeouts (800ms) and motor shutdown on timeout
- Publishes: `/wheel_odom` (10Hz)
- Subscribes: `/cmd_vel` (rosserial over WiFi)

#### Jetson Nano Responsibilities
- IMU data acquisition (direct I2C connection)
- Sensor fusion (robot_localization EKF)
- SLAM (gmapping)
- Localization (AMCL)
- Path planning (move_base)
- Autonomous exploration (explore_lite)
- Patrol coordination
- Publishes: `/odom`, `/scan`, `/imu/data`, `/map`

### TF Tree (Strictly Enforced)
```
map
 └── odom                    [published by AMCL]
     └── base_footprint      [published by robot_localization]
         └── base_link       [static]
             ├── laser       [static]
             └── imu_link    [static]
```

## Operational Modes

### MODE 1: Autonomous Mapping
**Purpose**: Create a map of the environment

**Command**:
```bash
roslaunch elderly_bot mapping.launch
```

**Active Nodes**:
- Hardware bringup (rosserial, rplidar)
- robot_localization (EKF)
- gmapping (SLAM)
- explore_lite (autonomous exploration)

**Behavior**:
- Robot autonomously explores unknown areas
- Builds occupancy grid map in real-time
- Continues until manual stop or completion
- Save map: `rosrun map_server map_saver -f <path>`

### MODE 2: Navigation & Patrol
**Purpose**: Autonomous patrol using saved map

**Command**:
```bash
roslaunch elderly_bot navigation.launch
rosrun elderly_bot patrol_client.py
```

**Active Nodes**:
- Hardware bringup
- robot_localization (EKF)
- map_server (static map)
- AMCL (localization)
- move_base (navigation)
- patrol_client (goal sequencer)

**Behavior**:
- Localizes on saved map
- Cycles through patrol waypoints indefinitely
- Dynamic obstacle avoidance
- Automatic recovery from failures

## Navigation Stack Configuration

### Global Planner
- **Algorithm**: NavfnROS (Dijkstra-based)
- **Costmap**: Static map + obstacles
- **Update Rate**: 1 Hz

### Local Planner
- **Algorithm**: DWA (Dynamic Window Approach)
- **Costmap**: Rolling window (4m × 4m)
- **Update Rate**: 10 Hz
- **Max Linear Velocity**: 0.25 m/s
- **Max Angular Velocity**: 1.0 rad/s

### Localization
- **Algorithm**: AMCL (particle filter)
- **Particles**: 100-1000 adaptive
- **Laser Beams**: 60 (downsampled from 360)

### Sensor Fusion
- **Algorithm**: Extended Kalman Filter
- **Inputs**: Wheel odometry + IMU
- **Output**: Fused odometry @ 50 Hz
- **Frame**: odom → base_footprint

## Safety Features

1. **Velocity Clamping**: All commands limited to safe speeds
2. **Timeout Protection**: Motors stop if no cmd_vel for 500ms
3. **Obstacle Avoidance**: 0.30m inflation radius
4. **Recovery Behaviors**: Automatic recovery from stuck situations
5. **Footprint Checking**: Collision detection before movement
6. **Emergency Stop**: Safe motor shutdown on errors

## Key Parameters

### Robot Geometry
- Footprint: 35cm × 25cm rectangular
- Wheel radius: 3.25cm
- Track width: 26cm
- Wheelbase: 17cm

### Velocity Limits
- Max linear: 0.25 m/s (indoor safe speed)
- Max angular: 1.0 rad/s
- Min linear: 0.05 m/s (overcome friction)

### Encoder Resolution
- 4900 ticks per wheel revolution
- Provides ~0.05mm linear resolution

### Control Frequencies
- ESP32 motor control: 50 Hz (20ms intervals)
- ESP32 odometry: 10 Hz (100ms intervals)
- EKF fusion: 50 Hz
- Local planner: 10 Hz
- Global planner: 1 Hz

## Communication Topics

### Published by ESP32
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/wheel_odom` | nav_msgs/Odometry | 10 Hz | Wheel odometry (WiFi rosserial) |

### Published by Jetson
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | 5-10 Hz | Lidar scan |
| `/odom` | nav_msgs/Odometry | 50 Hz | Fused odometry |
| `/imu/data` | sensor_msgs/Imu | 50 Hz | IMU measurements (direct I2C) |
| `/map` | nav_msgs/OccupancyGrid | 0.5 Hz | Occupancy map |

### Subscribed
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | Initial pose for AMCL |
| `/move_base_simple/goal` | geometry_msgs/PoseStamped | Single navigation goal |

## Configuration Files Explained

### `ekf.yaml`
- Fuses wheel odometry and IMU
- 2D mode (ignores z, roll, pitch)
- Publishes odom → base_footprint transform

### `gmapping.yaml`
- SLAM parameters for indoor mapping
- 5cm map resolution
- Tuned for skid-steer odometry errors

### `amcl.yaml`
- Particle filter localization
- Adaptive particle count (100-1000)
- Tuned for RPLidar A1

### `dwa_local_planner.yaml`
- Local trajectory optimization
- Velocity limits and acceleration
- Obstacle avoidance weights

### `costmap_common_params.yaml`
- Robot footprint definition
- Obstacle inflation (30cm)
- Sensor configuration

### `patrol_goals.yaml`
- List of waypoints (x, y, yaw)
- Cycled indefinitely by patrol client

## Tuning Guide

### Poor Localization
1. Increase AMCL particles
2. Adjust laser likelihood parameters
3. Check IMU calibration
4. Verify TF tree timing

### Navigation Failures
1. Increase inflation radius
2. Reduce max velocities
3. Tune DWA cost weights
4. Adjust recovery behaviors

### Motor Control Issues
1. Tune PID gains (kp, ki, kd)
2. Check encoder wiring
3. Verify motor direction flags
4. Adjust velocity limits

### Mapping Quality
1. Reduce exploration speed
2. Adjust gmapping scan matching
3. Increase particle count
4. Check odometry accuracy

## Build Instructions

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Key Documentation Files

- **[HARDWARE_MAP.md](HARDWARE_MAP.md)**: Complete hardware configuration reference
- **[ROSSERIAL_GUIDE.md](ROSSERIAL_GUIDE.md)**: rosserial setup and troubleshooting
- **[QUICK_START.md](QUICK_START.md)**: Step-by-step setup guide

## Testing Checklist

- [ ] ESP32 firmware uploads successfully
- [ ] Lidar publishes `/scan` at ~5-10 Hz
- [ ] IMU publishes `/imu/data` at ~100 Hz
- [ ] Wheel odometry publishes at ~100 Hz
- [ ] TF tree is complete (no missing transforms)
- [ ] Motors respond to `/cmd_vel` commands
- [ ] Robot stops when cmd_vel times out
- [ ] EKF publishes fused `/odom`
- [ ] Mapping creates reasonable map
- [ ] AMCL localizes on saved map
- [ ] move_base reaches navigation goals
- [ ] Patrol cycles through waypoints

## Performance Metrics

### Expected Performance
- **Mapping time**: ~10-20 min for typical house
- **Localization accuracy**: ±5cm position, ±3° orientation
- **Navigation success rate**: >95% in known environment
- **Obstacle avoidance**: 30cm minimum clearance
- **Battery life**: 2-4 hours (depends on battery)

## Troubleshooting Commands

```bash
# Check all topics
rostopic list

# Monitor topic rates
rostopic hz /scan
rostopic hz /wheel_odom
rostopic hz /odom

# View TF tree
rosrun tf view_frames
evince frames.pdf

# Check for errors
roswtf

# Monitor diagnostics
rostopic echo /diagnostics

# View logs
roscd elderly_bot
cat ~/.ros/log/latest/*.log
```

## Dependencies

### ROS Packages
- navigation (move_base, amcl, map_server)
- robot_localization
- gmapping
- explore_lite
- rplidar_ros
- rosserial_arduino
- rosserial_python

### System Packages
- python-pip
- python-yaml
- python-rospkg
- python-catkin-tools

### Arduino Libraries
- Rosserial Arduino Library
- MPU9250 (by hideakitai)

## Compliance

This package follows:
- **REP-103**: Standard Units and Coordinate Conventions
- **REP-105**: Coordinate Frames for Mobile Platforms
- **ROS Best Practices**: Standard naming, TF tree, topics
- **Commercial-Grade**: Safety, error handling, recovery

## Future Enhancements

Potential improvements:
- Add camera for visual monitoring
- Implement person detection
- Add voice alerts
- Web interface for remote monitoring
- Battery monitoring and auto-docking
- Multi-floor mapping
- Scheduling (patrol at specific times)

## Support

For issues:
1. Check README.md and QUICK_START.md
2. Review configuration files
3. Check ROS logs
4. Verify hardware connections
5. Test components individually

---

**Version**: 1.0.0  
**ROS Distribution**: Melodic  
**Target Platform**: Ubuntu 18.04  
**License**: BSD 3-Clause


